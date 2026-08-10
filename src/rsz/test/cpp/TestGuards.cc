// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the M5 F3 axis (downsize_guard): the pure arithmetic of
// Flach's local-negative-slack veto (TCAD'14 Alg. 4 lines 1/12-14, Eq. 14),
// against hand-computed values. The STA-facing half of the guard (which slacks
// the snapshot freezes, and whether they are the sweep-start or the live ones)
// is exercised by the global_sizing_guard integration test; the preset bundles
// and the validator's guard/engine cross-warnings are asserted here.

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

// The tolerance knob scales only the hill-climbing part: 0 pins gamma to 1
// (degradation forbidden from iteration 0), 2 doubles the paper's allowance.
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

// Zero delta reproduces the gate's current contribution - this is how
// snapshot() computes Flach's originalSlack.
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

// The paper's boundary case (flach_et_al.md §13.8): a gate on no violating net
// has original = 0, so gamma * original = 0 and ANY new local negative slack is
// rejected - however permissive gamma is.
TEST(LocalSlackVeto, ZeroOriginalRejectsAnyNewViolation)
{
  EXPECT_TRUE(localSlackVetoOk(0.0f, 0.0f, 5.0f));
  EXPECT_FALSE(localSlackVetoOk(-0.001f, 0.0f, 5.0f));
}

////////////////////////////////////////////////////////////////
// C2: the veto gated by the near-met latch (localSlackVetoOkGated)

// Before the run is near-met (active = false) the veto passes EVERY candidate -
// Sharma applies the driver/sink slack check only in power recovery
// (sharma_et_al.md §5.2). Even a candidate that badly degrades local slack, and
// even the zero-original case the raw veto would reject, is accepted.
TEST(VetoActivation, InactiveVetoPassesEveryCandidate)
{
  EXPECT_TRUE(localSlackVetoOkGated(false, -5.0f, -0.10f, 1.0f));
  EXPECT_TRUE(localSlackVetoOkGated(false, -0.13f, -0.10f, 1.2f));
  EXPECT_TRUE(localSlackVetoOkGated(false, -0.001f, 0.0f, 5.0f));
}

// Once active (near-met latched) it enforces Flach's acceptance test exactly.
TEST(VetoActivation, ActiveVetoEnforcesTheFlachRule)
{
  // At the gamma allowance floor: accepted; past it: rejected.
  EXPECT_TRUE(localSlackVetoOkGated(true, -0.12f, -0.10f, 1.2f));
  EXPECT_FALSE(localSlackVetoOkGated(true, -0.13f, -0.10f, 1.2f));
  // The zero-original boundary still rejects any new violation.
  EXPECT_FALSE(localSlackVetoOkGated(true, -0.001f, 0.0f, 5.0f));
}

// The ungated default (active always true, from LrState::near_met latched at
// iteration 0 when near_met_gate_frac < 0) must be byte-identical to the raw
// veto - flach/reimann/mangiras are unchanged.
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
// Config: presets and the validator's guard/engine cross-warnings

TEST(GuardConfig, DefaultIsDepthBudgetAndBaselinePresetKeepsIt)
{
  EXPECT_EQ(GlobalSizingConfig{}.downsize_guard, DownsizeGuard::kDepthBudget);
  GlobalSizingConfig config;
  config.applyPreset(Preset::kRszBaseline);
  EXPECT_EQ(config.downsize_guard, DownsizeGuard::kDepthBudget);
  EXPECT_FLOAT_EQ(config.gamma_local_slack, 1.0f);
}

// The three presets whose papers run Flach's acceptance test: flach itself,
// reimann (which inherits it), and mangiras (Flach's machinery, new seed).
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

// Both guard/engine crosses are allowed (validate() still returns true) but
// each emits its warning; the matched pairs emit none.
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

// M3 carry-over: composing the two slew-coupling cost terms double-prices the
// immediate sink level. Allowed, warned.
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
