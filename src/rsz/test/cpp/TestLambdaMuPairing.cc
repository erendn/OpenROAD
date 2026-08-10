// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

// Unit tests for GlobalSizingConfig::resolveLambdaMuPairing - the iteration-2
// lambda <-> mu auto-pairing (plan §2.2-6, ruling #2).
//
// The finding it implements (S1-E, engine-verified on the iteration-1 pilot):
// under a mu policy that does not read lambda, the flow projection re-anchors
// every endpoint from a slack field each iteration, which discards whatever the
// E1/E2 updater did to the multiplier magnitudes - chen, tennakoon and
// livramento came back BYTE-IDENTICAL as collected. The lambda_update axis is
// then not identifiable at all, so a paper rule with no chosen mu policy takes
// endpoint_lambda automatically.
//
// What is deliberately pinned here:
//   * norm_subgradient is NEVER moved. It is rsz_baseline's own rule and the
//     ablation's denominator.
//   * A preset that already pins a lambda-reading mu policy is never rewritten
//     - which is every paper preset, so no Stage-2 claim column moves.
//   * An explicitly-set mu policy is always honored, including the annihilated
//     combinations, which stay a legitimate (and now named) cell.
//   * mu_auto_paired reports what happened, because RSZ-0417 echoes it and the
//     harness reads that to tell an auto-paired cell from a configured one.

#include <vector>

#include "gtest/gtest.h"
#include "rsz/GlobalSizingConfig.hh"
#include "utl/Logger.h"

namespace rsz {
namespace {

using LambdaUpdate = GlobalSizingConfig::LambdaUpdate;
using MuPolicy = GlobalSizingConfig::MuPolicy;
using Preset = GlobalSizingConfig::Preset;

// Every rule except the baseline's own.
const std::vector<LambdaUpdate>& paperRules()
{
  static const std::vector<LambdaUpdate> rules = {
      LambdaUpdate::kFlachSlackScaling,
      LambdaUpdate::kChenSubgradient,
      LambdaUpdate::kTennakoonRatio,
      LambdaUpdate::kSharmaCexp,
      LambdaUpdate::kReimannDwns,
      LambdaUpdate::kLivramentoRatio,
  };
  return rules;
}

// The three that do not read lambda at the endpoint boundary.
const std::vector<MuPolicy>& annihilatingPolicies()
{
  static const std::vector<MuPolicy> policies = {
      MuPolicy::kReseedEachIter,
      MuPolicy::kSeedOnce,
      MuPolicy::kUpdateAsLambda,
  };
  return policies;
}

// The E cells of the Stage-1 catalog: rsz_baseline plus one -lambda_update
// override, nothing else. Each must come out paired.
TEST(LambdaMuPairing, PaperRuleOnTheBaselineBundleAutoPairs)
{
  for (const LambdaUpdate rule : paperRules()) {
    utl::Logger logger;
    GlobalSizingConfig config;
    config.lambda_update = rule;
    ASSERT_EQ(config.mu_policy, MuPolicy::kReseedEachIter) << toString(rule);

    config.resolveLambdaMuPairing(&logger);
    EXPECT_EQ(config.mu_policy, MuPolicy::kEndpointLambda) << toString(rule);
    EXPECT_TRUE(config.mu_auto_paired) << toString(rule);
    EXPECT_EQ(logger.getWarningCount(), 1) << toString(rule);
  }
}

// It fires from any of the three annihilating policies, not just the struct
// default - seed_once and update_as_lambda anchor to the same slack-derived mu.
TEST(LambdaMuPairing, AutoPairsFromEveryAnnihilatingPolicy)
{
  for (const MuPolicy policy : annihilatingPolicies()) {
    utl::Logger logger;
    GlobalSizingConfig config;
    config.lambda_update = LambdaUpdate::kSharmaCexp;
    config.mu_policy = policy;
    config.resolveLambdaMuPairing(&logger);
    EXPECT_EQ(config.mu_policy, MuPolicy::kEndpointLambda) << toString(policy);
    EXPECT_TRUE(config.mu_auto_paired) << toString(policy);
  }
}

// rsz_baseline's own rule is exempt: its lambda has no paper magnitude to
// preserve, and auto-moving the baseline's mu policy would move the ablation's
// denominator.
TEST(LambdaMuPairing, NormSubgradientIsUntouched)
{
  for (const MuPolicy policy : annihilatingPolicies()) {
    utl::Logger logger;
    GlobalSizingConfig config;
    config.lambda_update = LambdaUpdate::kNormSubgradient;
    config.mu_policy = policy;
    config.resolveLambdaMuPairing(&logger);
    EXPECT_EQ(config.mu_policy, policy) << toString(policy);
    EXPECT_FALSE(config.mu_auto_paired) << toString(policy);
    EXPECT_EQ(logger.getWarningCount(), 0) << toString(policy);
  }
  // Including with a lambda-reading policy, which is a normal cross (Stage-1
  // M3) and must stay silent.
  utl::Logger logger;
  GlobalSizingConfig config;
  config.lambda_update = LambdaUpdate::kNormSubgradient;
  config.mu_policy = MuPolicy::kEndpointLambda;
  config.resolveLambdaMuPairing(&logger);
  EXPECT_EQ(config.mu_policy, MuPolicy::kEndpointLambda);
  EXPECT_FALSE(config.mu_auto_paired);
  EXPECT_EQ(logger.getWarningCount(), 0);
}

// A bundle that already reads lambda needs nothing done to it and must stay
// silent - no warn, no info, no change.
TEST(LambdaMuPairing, LambdaReadingPolicyIsLeftAlone)
{
  for (const MuPolicy policy : {MuPolicy::kEndpointLambda,
                                MuPolicy::kEndpointRatio,
                                MuPolicy::kEndpointAdditive}) {
    for (const LambdaUpdate rule : paperRules()) {
      utl::Logger logger;
      GlobalSizingConfig config;
      config.lambda_update = rule;
      config.mu_policy = policy;
      config.resolveLambdaMuPairing(&logger);
      EXPECT_EQ(config.mu_policy, policy) << toString(rule);
      EXPECT_FALSE(config.mu_auto_paired) << toString(rule);
      EXPECT_EQ(logger.getWarningCount(), 0) << toString(rule);
    }
  }
}

// THE preset guarantee: no paper column is rewritten by this rule. Every one of
// them already pins a lambda-reading policy (endpoint_lambda for the shared
// family, endpoint_additive for chen, endpoint_ratio for tennakoon and
// livramento), so the pairing is a no-op on all of them - and rsz_baseline
// keeps norm_subgradient, so it is exempt on the other branch.
TEST(LambdaMuPairing, NoPresetIsRewritten)
{
  for (const Preset preset : kAllPresets) {
    utl::Logger logger;
    GlobalSizingConfig config;
    config.applyPreset(preset);
    const MuPolicy before = config.mu_policy;
    config.resolveLambdaMuPairing(&logger);
    EXPECT_EQ(config.mu_policy, before) << toString(preset);
    EXPECT_FALSE(config.mu_auto_paired) << toString(preset);
    EXPECT_EQ(logger.getWarningCount(), 0) << toString(preset);
  }
}

// An explicit -mu_policy is always honored. When the explicit combination is an
// annihilated one, the run is NAMED (RSZ-0448, info) rather than blocked or
// silently rewritten: "measure the annihilation" is a legitimate cell, it just
// must never be the accidental default.
TEST(LambdaMuPairing, ExplicitPolicyIsHonoredAndAnnihilationIsNamed)
{
  for (const MuPolicy policy : annihilatingPolicies()) {
    for (const LambdaUpdate rule : paperRules()) {
      utl::Logger logger;
      GlobalSizingConfig config;
      config.lambda_update = rule;
      config.mu_policy = policy;
      config.mu_policy_explicit = true;
      config.resolveLambdaMuPairing(&logger);
      EXPECT_EQ(config.mu_policy, policy) << toString(rule);
      EXPECT_FALSE(config.mu_auto_paired) << toString(rule);
      // Info, not warn - the cell is legitimate.
      EXPECT_EQ(logger.getWarningCount(), 0) << toString(rule);
    }
  }
}

// An explicit lambda-reading policy is honored with nothing said at all: it is
// the pairing the rule wanted anyway.
TEST(LambdaMuPairing, ExplicitLambdaReadingPolicyIsSilent)
{
  utl::Logger logger;
  GlobalSizingConfig config;
  config.lambda_update = LambdaUpdate::kChenSubgradient;
  config.mu_policy = MuPolicy::kEndpointAdditive;
  config.mu_policy_explicit = true;
  config.resolveLambdaMuPairing(&logger);
  EXPECT_EQ(config.mu_policy, MuPolicy::kEndpointAdditive);
  EXPECT_FALSE(config.mu_auto_paired);
  EXPECT_EQ(logger.getWarningCount(), 0);
}

// Resolution is idempotent and mu_auto_paired always reports THIS call: a
// second pass over an already-paired config must not claim it paired anything.
TEST(LambdaMuPairing, IsIdempotent)
{
  utl::Logger logger;
  GlobalSizingConfig config;
  config.lambda_update = LambdaUpdate::kFlachSlackScaling;
  config.resolveLambdaMuPairing(&logger);
  ASSERT_TRUE(config.mu_auto_paired);

  config.resolveLambdaMuPairing(&logger);
  EXPECT_EQ(config.mu_policy, MuPolicy::kEndpointLambda);
  EXPECT_FALSE(config.mu_auto_paired);
  EXPECT_EQ(logger.getWarningCount(), 1);  // still just the first call's
}

// The paired config must still pass the cross-axis validator, or the pairing
// would have traded an identifiability failure for a hard rejection.
TEST(LambdaMuPairing, PairedConfigStillValidates)
{
  for (const LambdaUpdate rule : paperRules()) {
    utl::Logger logger;
    GlobalSizingConfig config;
    config.lambda_update = rule;
    config.resolveLambdaMuPairing(&logger);
    EXPECT_TRUE(config.validate(&logger)) << toString(rule);
  }
}

}  // namespace
}  // namespace rsz
