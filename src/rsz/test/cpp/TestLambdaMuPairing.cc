// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

// Unit tests for GlobalSizingConfig::resolveLambdaMuPairing, which picks a mu
// (endpoint multiplier) policy that works with the chosen lambda update rule.
//
// Under a mu policy that does not read lambda, the flow projection re-anchors
// every endpoint to a mu derived from endpoint slacks on every iteration. That
// rescales away whatever the lambda update rule did to the multiplier
// magnitudes, so different paper rules produce identical results. A paper rule
// with no explicitly chosen mu policy therefore gets endpoint_lambda
// automatically.
//
// The tests check that:
//   * norm_subgradient, the default rule, is never re-paired.
//   * A preset that already uses a lambda-reading mu policy is never rewritten;
//     every paper preset does.
//   * An explicitly set mu policy is always honored, even a combination that
//     cancels the lambda update; that case is only reported.
//   * mu_auto_paired reports what happened, because RSZ-0417 prints it so that
//     scripts can tell an auto-paired run from an explicitly configured one.

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

// The three policies that anchor mu to endpoint slacks and ignore lambda.
const std::vector<MuPolicy>& annihilatingPolicies()
{
  static const std::vector<MuPolicy> policies = {
      MuPolicy::kReseedEachIter,
      MuPolicy::kSeedOnce,
      MuPolicy::kUpdateAsLambda,
  };
  return policies;
}

// The default configuration with only -lambda_update changed to a paper rule
// must be auto-paired.
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

// Pairing applies to all three slack-anchored policies, not just the default:
// seed_once and update_as_lambda anchor to the same slack-derived mu.
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

// The default rule is exempt: its lambda magnitude has no meaning to preserve,
// and re-pairing it would change the default configuration's behavior.
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
  // Also with a lambda-reading policy, which is a valid combination and must
  // not log anything.
  utl::Logger logger;
  GlobalSizingConfig config;
  config.lambda_update = LambdaUpdate::kNormSubgradient;
  config.mu_policy = MuPolicy::kEndpointLambda;
  config.resolveLambdaMuPairing(&logger);
  EXPECT_EQ(config.mu_policy, MuPolicy::kEndpointLambda);
  EXPECT_FALSE(config.mu_auto_paired);
  EXPECT_EQ(logger.getWarningCount(), 0);
}

// A configuration whose mu policy already reads lambda is left unchanged, with
// no warning and no info message.
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

// No preset is changed by the pairing. Every paper preset already uses a
// lambda-reading policy (endpoint_additive for chen, endpoint_ratio for
// tennakoon and livramento, endpoint_lambda for the rest), and rsz_baseline
// uses norm_subgradient, which is exempt.
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

// An explicit -mu_policy is always honored. If the combination cancels the
// lambda update, RSZ-0448 reports it at info level instead of blocking or
// rewriting it: the user may want that combination, it just must never happen
// by default.
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
      // Info, not warning: the combination is allowed.
      EXPECT_EQ(logger.getWarningCount(), 0) << toString(rule);
    }
  }
}

// An explicit lambda-reading policy is honored without any message, since it
// is already a working pairing.
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

// Resolution is idempotent, and mu_auto_paired reflects only the latest call:
// a second call on an already-paired config must not report a pairing.
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

// The paired config must still pass validate(); otherwise auto-pairing would
// turn a run that merely ignores the lambda rule into a validation error.
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
