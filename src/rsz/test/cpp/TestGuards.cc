// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the downsize guard: the arithmetic of the local negative slack
// veto (Flach et al., TCAD 2014, Alg. 4 lines 1 and 12-14, Eq. 14), checked
// against hand-computed values, plus the preset settings and the validator's
// guard/engine warnings. The STA side of the guard (which slacks the snapshot
// captures, and whether they are the sweep-start or the live ones) is covered
// by the global_sizing_guard integration test.

#include "gtest/gtest.h"
#include "lr/Guards.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "utl/Logger.h"

namespace rsz {
namespace {

using DownsizeGuard = GlobalSizingConfig::DownsizeGuard;
using Preset = GlobalSizingConfig::Preset;

////////////////////////////////////////////////////////////////
// Eq. 14: gamma = 1 + scale * (-min(0, WNS) / T)

// A design that meets timing gets no hill-climbing allowance at all.
TEST(FlachGamma, NonViolatingDesignGivesUnitGamma)
{
  EXPECT_FLOAT_EQ(flachGamma(/*worst_slack=*/0.05f, /*T=*/1.0f, 1.0f), 1.0f);
  EXPECT_FLOAT_EQ(flachGamma(0.0f, 1.0f, 1.0f), 1.0f);
}

// WNS = -0.2 ns on a 1 ns clock -> gamma = 1 + 0.2/1.0 = 1.2.
TEST(FlachGamma, ViolatingDesignScalesWithWnsOverT)
{
  EXPECT_FLOAT_EQ(flachGamma(-0.2f, 1.0f, 1.0f), 1.2f);
  // A WNS as large as the period doubles the allowance.
  EXPECT_FLOAT_EQ(flachGamma(-1.0f, 1.0f, 1.0f), 2.0f);
  // Half a period of violation on a 2 ns clock: 1 + 1.0/2.0 = 1.5.
  EXPECT_FLOAT_EQ(flachGamma(-1.0f, 2.0f, 1.0f), 1.5f);
}

// The tolerance scales only the hill-climbing term: 0 fixes gamma at 1 (no
// degradation allowed from iteration 0), 2 doubles the paper's allowance.
TEST(FlachGamma, ToleranceScalesTheHillClimbingTerm)
{
  EXPECT_FLOAT_EQ(flachGamma(-0.2f, 1.0f, 0.0f), 1.0f);
  EXPECT_FLOAT_EQ(flachGamma(-0.2f, 1.0f, 2.0f), 1.4f);
}

// No clock -> nothing to normalize by -> no hill climbing.
TEST(FlachGamma, NoClockGivesUnitGamma)
{
  EXPECT_FLOAT_EQ(flachGamma(-0.2f, 0.0f, 1.0f), 1.0f);
}

////////////////////////////////////////////////////////////////
// negativeSlackAfter: min(0, slack - delta)

// Positive slack absorbs a small delay increase and contributes nothing.
TEST(NegativeSlackAfter, PositiveSlackAbsorbsSmallDelta)
{
  EXPECT_FLOAT_EQ(negativeSlackAfter(0.10f, 0.04f), 0.0f);
}

// ... but not a big one: 0.10 - 0.15 = -0.05.
TEST(NegativeSlackAfter, PositiveSlackBreaksOnLargeDelta)
{
  EXPECT_FLOAT_EQ(negativeSlackAfter(0.10f, 0.15f), -0.05f);
}

// An already-violating net gets worse one-for-one: -0.05 - 0.03 = -0.08.
TEST(NegativeSlackAfter, ViolatingNetDegradesWithDelta)
{
  EXPECT_FLOAT_EQ(negativeSlackAfter(-0.05f, 0.03f), -0.08f);
}

// A speed-up (negative delta) lifts a violating net back toward zero, and is
// clamped at 0 once it is no longer violating: -0.05 - (-0.09) = +0.04 -> 0.
TEST(NegativeSlackAfter, SpeedupLiftsViolatingNet)
{
  EXPECT_FLOAT_EQ(negativeSlackAfter(-0.05f, -0.03f), -0.02f);
  EXPECT_FLOAT_EQ(negativeSlackAfter(-0.05f, -0.09f), 0.0f);
}

// Zero delta gives the gate's current contribution; this is how snapshot()
// computes Flach's originalSlack.
TEST(NegativeSlackAfter, ZeroDeltaIsTheOriginalContribution)
{
  EXPECT_FLOAT_EQ(negativeSlackAfter(-0.05f, 0.0f), -0.05f);
  EXPECT_FLOAT_EQ(negativeSlackAfter(0.05f, 0.0f), 0.0f);
}

////////////////////////////////////////////////////////////////
// Alg. 4 line 13: accept iff candidate >= gamma * original

// gamma = 1.2, original = -0.10 -> the allowance floor is -0.12.
TEST(LocalSlackVeto, AllowsDegradationUpToTheGammaAllowance)
{
  const float gamma = 1.2f;
  const float orig = -0.10f;
  // Right at the floor: accepted (the paper rejects on strict <).
  EXPECT_TRUE(localSlackVetoOk(-0.12f, orig, gamma));
  // Inside it: accepted - this is the hill climbing.
  EXPECT_TRUE(localSlackVetoOk(-0.115f, orig, gamma));
  // Past it: rejected.
  EXPECT_FALSE(localSlackVetoOk(-0.13f, orig, gamma));
}

// An improvement is always accepted, whatever gamma is.
TEST(LocalSlackVeto, AlwaysAcceptsAnImprovement)
{
  EXPECT_TRUE(localSlackVetoOk(-0.05f, -0.10f, 1.0f));
  EXPECT_TRUE(localSlackVetoOk(0.0f, -0.10f, 1.0f));
}

// gamma = 1 (converged design): no degradation is tolerated any more.
TEST(LocalSlackVeto, UnitGammaForbidsDegradation)
{
  EXPECT_TRUE(localSlackVetoOk(-0.10f, -0.10f, 1.0f));
  EXPECT_FALSE(localSlackVetoOk(-0.1001f, -0.10f, 1.0f));
}

// A gate on no violating net has original = 0, so gamma * original = 0 and any
// new local negative slack is rejected, however large gamma is.
TEST(LocalSlackVeto, ZeroOriginalRejectsAnyNewViolation)
{
  EXPECT_TRUE(localSlackVetoOk(0.0f, 0.0f, 5.0f));
  EXPECT_FALSE(localSlackVetoOk(-0.001f, 0.0f, 5.0f));
}

////////////////////////////////////////////////////////////////
// The veto gated by the near-met latch (localSlackVetoOkGated)

// Before timing is nearly met (active = false) the veto accepts every
// candidate, because Sharma et al. (ICCAD 2015) apply the driver/sink slack
// check only during power recovery. A candidate that badly degrades local
// slack, and the zero-original case the plain veto would reject, are both
// accepted.
TEST(VetoActivation, InactiveVetoPassesEveryCandidate)
{
  EXPECT_TRUE(localSlackVetoOkGated(false, -5.0f, -0.10f, 1.0f));
  EXPECT_TRUE(localSlackVetoOkGated(false, -0.13f, -0.10f, 1.2f));
  EXPECT_TRUE(localSlackVetoOkGated(false, -0.001f, 0.0f, 5.0f));
}

// Once active (the near-met latch is set) it applies Flach's acceptance test
// exactly.
TEST(VetoActivation, ActiveVetoEnforcesTheFlachRule)
{
  // At the gamma allowance floor: accepted; past it: rejected.
  EXPECT_TRUE(localSlackVetoOkGated(true, -0.12f, -0.10f, 1.2f));
  EXPECT_FALSE(localSlackVetoOkGated(true, -0.13f, -0.10f, 1.2f));
  // The zero-original boundary still rejects any new violation.
  EXPECT_FALSE(localSlackVetoOkGated(true, -0.001f, 0.0f, 5.0f));
}

// Without gating (near_met_gate_frac < 0 sets LrState::near_met at iteration 0,
// so active is always true) the result must match the plain veto exactly. The
// flach, reimann and mangiras presets rely on this.
TEST(VetoActivation, DefaultActiveMatchesTheRawVeto)
{
  const float cases[][3] = {{-0.12f, -0.10f, 1.2f},
                            {-0.13f, -0.10f, 1.2f},
                            {-0.05f, -0.10f, 1.0f},
                            {-0.001f, 0.0f, 5.0f},
                            {0.0f, -0.10f, 1.0f}};
  for (const auto& c : cases) {
    EXPECT_EQ(localSlackVetoOkGated(true, c[0], c[1], c[2]),
              localSlackVetoOk(c[0], c[1], c[2]));
  }
}

////////////////////////////////////////////////////////////////
// Config: presets and the validator's guard/engine warnings

TEST(GuardConfig, DefaultIsDepthBudgetAndBaselinePresetKeepsIt)
{
  EXPECT_EQ(GlobalSizingConfig{}.downsize_guard, DownsizeGuard::kDepthBudget);
  GlobalSizingConfig config;
  config.applyPreset(Preset::kRszBaseline);
  EXPECT_EQ(config.downsize_guard, DownsizeGuard::kDepthBudget);
  EXPECT_FLOAT_EQ(config.gamma_local_slack, 1.0f);
}

// The three presets whose papers use Flach's acceptance test: flach itself,
// reimann (which inherits it), and mangiras (which builds on Flach's method
// with a different lambda seed).
TEST(GuardConfig, PaperPresetsSelectTheVeto)
{
  for (const Preset p : {Preset::kFlach, Preset::kReimann, Preset::kMangiras}) {
    GlobalSizingConfig config;
    config.applyPreset(p);
    EXPECT_EQ(config.downsize_guard, DownsizeGuard::kLocalSlackVeto)
        << "preset " << toString(p);
  }
}

TEST(GuardConfig, ParseRoundTrip)
{
  DownsizeGuard guard = DownsizeGuard::kDepthBudget;
  EXPECT_TRUE(parseDownsizeGuard("local_slack_veto", guard));
  EXPECT_EQ(guard, DownsizeGuard::kLocalSlackVeto);
  EXPECT_TRUE(parseDownsizeGuard("none", guard));
  EXPECT_EQ(guard, DownsizeGuard::kNone);
  EXPECT_TRUE(parseDownsizeGuard("depth_budget", guard));
  EXPECT_EQ(guard, DownsizeGuard::kDepthBudget);
  EXPECT_FALSE(parseDownsizeGuard("bogus", guard));
  EXPECT_EQ(guard, DownsizeGuard::kDepthBudget);  // unchanged on failure
}

// The two mismatched guard/engine combinations are allowed (validate() still
// returns true) but each logs a warning; the matched pairs log none.
TEST(GuardConfig, GuardEngineCrossesWarnButValidate)
{
  utl::Logger logger;

  GlobalSizingConfig jacobi_veto;
  jacobi_veto.sweep_engine
      = GlobalSizingConfig::SweepEngineKind::kJacobiSnapshot;
  jacobi_veto.downsize_guard = DownsizeGuard::kLocalSlackVeto;
  EXPECT_TRUE(jacobi_veto.validate(&logger));
  EXPECT_EQ(logger.getWarningCount(), 1);

  GlobalSizingConfig gs_budget;
  gs_budget.sweep_engine
      = GlobalSizingConfig::SweepEngineKind::kGaussSeidelTopo;
  gs_budget.downsize_guard = DownsizeGuard::kDepthBudget;
  EXPECT_TRUE(gs_budget.validate(&logger));
  EXPECT_EQ(logger.getWarningCount(), 2);

  // The matched pairs: Jacobi + budget (rsz_baseline) and GS + veto (flach).
  GlobalSizingConfig baseline;
  baseline.applyPreset(Preset::kRszBaseline);
  EXPECT_TRUE(baseline.validate(&logger));
  GlobalSizingConfig flach;
  flach.applyPreset(Preset::kFlach);
  EXPECT_TRUE(flach.validate(&logger));
  EXPECT_EQ(logger.getWarningCount(), 2);  // no new warnings
}

// Enabling both slew-coupling cost terms counts the immediate sink level twice.
// This is allowed but logs a warning.
TEST(GuardConfig, PhiPlusFanoutSlewWarns)
{
  utl::Logger logger;
  GlobalSizingConfig config;
  config.cost_global_phi = true;
  config.cost_fanout_slew = true;
  EXPECT_TRUE(config.validate(&logger));
  EXPECT_EQ(logger.getWarningCount(), 1);
}

}  // namespace
}  // namespace rsz
