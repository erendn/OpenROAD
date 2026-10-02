// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the lambda and mu update rules. Each updater's per-arc and
// per-endpoint arithmetic is a free function with no STA dependency, so these
// tests check the formulas against hand-computed values. The full update()
// path (STA and graph traversal) is covered by the global_sizing_lambda_update
// integration test. The arc read the arrival-based updaters share
// (LrState::consistentArcRead) is also checked against OpenSTA on a small
// timed design, and so is the move of the multipliers onto the edges a cell
// swap re-creates (LrState::refreshLiveEdges).

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "gtest/gtest.h"
#include "lr/LambdaUpdater.hh"
#include "lr/LrState.hh"
#include "odb/db.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"
#include "rsz/GlobalSizingConfig.hh"
#include "sta/Delay.hh"
#include "sta/EquivCells.hh"
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/Scene.hh"
#include "sta/Transition.hh"
#include "tst/IntegratedFixture.h"
#include "tst/fixture.h"

namespace rsz {
namespace {

constexpr float kTol = 1e-5f;
constexpr float kFloor = 1e-12f;

// --- norm_subgradient (default) --------------------------------------------

TEST(LambdaUpdaterFormula, NormSubgradientTightArcUnchanged)
{
  // Tight arc: a_to - a_from == d, so g = 0 and lambda is unchanged.
  EXPECT_NEAR(
      normSubgradientLambda(1.0f, 0.1f, 0.0f, 0.1f, 0.6f, kFloor), 1.0f, kTol);
}

TEST(LambdaUpdaterFormula, NormSubgradientFullSlackShrinks)
{
  // Full slack: (d-(a_to-a_from))/d = (0.1-0.2)/0.1 = -1, scale = 1-0.6 = 0.4.
  EXPECT_NEAR(
      normSubgradientLambda(1.0f, 0.1f, 0.0f, 0.2f, 0.6f, kFloor), 0.4f, kTol);
}

TEST(LambdaUpdaterFormula, NormSubgradientFloored)
{
  // A small lambda pushed below the floor is clamped up to it.
  EXPECT_NEAR(normSubgradientLambda(kFloor, 0.1f, 0.0f, 1.0f, 1.0f, kFloor),
              kFloor,
              1e-18f);
}

TEST(LambdaUpdaterReject, AlphaHalvesOnRejection)
{
  NormSubgradientUpdater updater(0.6f);
  EXPECT_NEAR(updater.currentStep(), 0.6f, kTol);
  updater.onPassRejected();
  EXPECT_NEAR(updater.currentStep(), 0.3f, kTol);
  updater.onPassRejected();
  EXPECT_NEAR(updater.currentStep(), 0.15f, kTol);
}

// --- flach_slack_scaling (Flach et al., TCAD 2014, Alg. 2) -----------------

TEST(LambdaUpdaterFormula, FlachViolatingArcGrows)
{
  // slack=-0.1 (violating), T=1, k=1: (1 + 0.1/1)^{1/1} = 1.1.
  EXPECT_NEAR(flachSlackScaleFactor(-0.1f, 1.0f, 1.0f), 1.1f, kTol);
}

TEST(LambdaUpdaterFormula, FlachNonViolatingArcShrinks)
{
  // slack=0.2 (positive), T=1, k=2: (1.2)^{-2} = 1/1.44 = 0.694444.
  EXPECT_NEAR(flachSlackScaleFactor(0.2f, 1.0f, 2.0f), 0.694444f, kTol);
}

TEST(LambdaUpdaterFormula, FlachZeroSlackAndNoClock)
{
  // slack==0 takes the violating branch: (1+0)^{1/k} = 1.
  EXPECT_NEAR(flachSlackScaleFactor(0.0f, 1.0f, 3.0f), 1.0f, kTol);
  // T<=0 disables the update.
  EXPECT_NEAR(flachSlackScaleFactor(-0.1f, 0.0f, 1.0f), 1.0f, kTol);
}

TEST(LambdaUpdaterFormula, FlachKSchedule)
{
  // Far from feasible, mid-run: k_init.
  EXPECT_NEAR(flachKForIter(0, 20, -5.0f, 1.0f, 1.0f, 4.0f, 1.0f), 1.0f, kTol);
  // Near-feasible (wns within 10% of T), mid-run: k_tns_small.
  EXPECT_NEAR(flachKForIter(5, 20, -0.05f, 1.0f, 1.0f, 4.0f, 1.0f), 4.0f, kTol);
  // Endgame (last ~10% of iterations): k_final, overriding near-feasible.
  EXPECT_NEAR(
      flachKForIter(19, 20, -0.05f, 1.0f, 1.0f, 4.0f, 2.0f), 2.0f, kTol);
}

// --- chen_subgradient (Chen et al., ICCAD 1998) ----------------------------

TEST(LambdaUpdaterFormula, ChenRhoSchedule)
{
  EXPECT_NEAR(chenRho(1, 2.0f), 2.0f, kTol);  // k = max(1,1) = 1
  EXPECT_NEAR(chenRho(4, 2.0f), 0.5f, kTol);  // 2/4
  EXPECT_NEAR(chenRho(0, 2.0f), 2.0f, kTol);  // k floored to 1
}

TEST(LambdaUpdaterFormula, ChenAdditiveStep)
{
  // violation = (a_from + d - a_to)/T = (0 + 0.1 - 0.15)/1 = -0.05;
  // 1 + 2*(-0.05) = 0.9.
  EXPECT_NEAR(
      chenSubgradientLambda(1.0f, 0.0f, 0.15f, 0.1f, 2.0f, 1.0f, kFloor),
      0.9f,
      kTol);
}

TEST(LambdaUpdaterFormula, ChenStepIsClockPeriodRelative)
{
  // The step is a fraction of T, so halving T doubles the same violation's
  // effect: violation = -0.05/0.5 = -0.1; 1 + 2*(-0.1) = 0.8.
  EXPECT_NEAR(
      chenSubgradientLambda(1.0f, 0.0f, 0.15f, 0.1f, 2.0f, 0.5f, kFloor),
      0.8f,
      kTol);
}

TEST(LambdaUpdaterFormula, ChenNoClockLeavesLambdaAlone)
{
  // T <= 0 (no clock) leaves no scale to normalize against.
  EXPECT_NEAR(
      chenSubgradientLambda(1.0f, 0.0f, 0.15f, 0.1f, 2.0f, 0.0f, kFloor),
      1.0f,
      kTol);
}

// Why the step is normalized by T: violations in seconds (~1e-10) are tiny
// next to an O(1) lambda, so the unnormalized step `lambda + rho * violation`
// rounds back to lambda in float32 and lambda never changes. Normalizing by T
// keeps the step representable at the paper's default c = 1.
TEST(LambdaUpdaterFormula, ChenStepSurvivesRealisticSiMagnitudes)
{
  const float T = 0.35e-9f;  // 0.35 ns clock, as the smoke designs use
  const float a_from = 1.0e-10f;
  const float d = 5.0e-11f;
  const float a_to = 2.0e-10f;  // violation = -5e-11 s, i.e. -1/7 of T
  const float lambda = 0.5f;    // the O(1) magnitude the projection produces
  const float updated
      = chenSubgradientLambda(lambda, a_from, a_to, d, 1.0f, T, kFloor);
  EXPECT_NE(updated, lambda);
  EXPECT_NEAR(updated, lambda + (-5.0e-11f / T), 1e-6f);
}

TEST(LambdaUpdaterFormula, ChenFloored)
{
  // A large negative violation drives lambda below the floor.
  EXPECT_NEAR(chenSubgradientLambda(0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, kFloor),
              kFloor,
              1e-18f);
}

// --- livramento_ratio (Livramento et al., DATE 2013, Alg. 1) ---------------

TEST(LambdaUpdaterFormula, LivramentoRatio)
{
  // Alg. 1 line 13: (a_from + d)/a_to. Critical arc (a_to == a_from + d) is
  // neutral.
  EXPECT_NEAR(livramentoRatioFactor(0.1f, 0.2f, 0.1f), 1.0f, kTol);
  // Non-critical: (1 + 1)/3.
  EXPECT_NEAR(livramentoRatioFactor(1.0f, 3.0f, 1.0f), 2.0f / 3.0f, kTol);
}

// Livramento and Tennakoon use different local step sizes (Livramento Eq. 9:
// rho_k = lambda/a_i; Tennakoon Fig. 13: rho_k = lambda/(a_i - D_i)), so the
// two rules agree only on a critical arc and differ everywhere else. That is
// why livramento_ratio is a separate updater.
TEST(LambdaUpdaterFormula, LivramentoDiffersFromTennakoonOffTheCriticalArc)
{
  // Critical arc: both neutral.
  EXPECT_NEAR(livramentoRatioFactor(0.1f, 0.2f, 0.1f),
              tennakoonRatioFactor(0.1f, 0.2f, 0.1f),
              kTol);
  // Non-critical arc: livramento 2/3, tennakoon 1/2.
  EXPECT_NEAR(livramentoRatioFactor(1.0f, 3.0f, 1.0f), 2.0f / 3.0f, kTol);
  EXPECT_NEAR(tennakoonRatioFactor(1.0f, 3.0f, 1.0f), 0.5f, kTol);
}

// Alg. 1 line 14 (arcs from primary inputs, D_ji/a_i) needs no separate
// branch: it is line 13 with a_from == 0. Tennakoon's form instead returns
// 0/(a_to - d) = 0 here, which drops lambda to the floor, and under a
// multiplicative update it can never recover from zero.
TEST(LambdaUpdaterFormula, LivramentoPiSourcedArcIsLine13AtZeroArrival)
{
  EXPECT_NEAR(livramentoRatioFactor(0.0f, 3.0f, 1.0f), 1.0f / 3.0f, kTol);
  EXPECT_NEAR(tennakoonRatioFactor(0.0f, 3.0f, 1.0f), 0.0f, kTol);
}

TEST(LambdaUpdaterFormula, LivramentoNonPositiveArrivalIsNeutral)
{
  // a_to <= 0 (unconstrained / negative arrival sink) has no usable
  // denominator.
  EXPECT_NEAR(livramentoRatioFactor(1.0f, 0.0f, 1.0f), 1.0f, kTol);
  EXPECT_NEAR(livramentoRatioFactor(1.0f, -1.0f, 1.0f), 1.0f, kTol);
}

// --- tennakoon_ratio (Tennakoon and Sechen, ICCAD 2002, Fig. 13) -----------

TEST(LambdaUpdaterFormula, TennakoonRatio)
{
  // Critical arc: a_to - d == a_from, ratio = 1.
  EXPECT_NEAR(tennakoonRatioFactor(0.1f, 0.2f, 0.1f), 1.0f, kTol);
  // Slack arc: a_to - d = 0.2, ratio = 0.1/0.2 = 0.5.
  EXPECT_NEAR(tennakoonRatioFactor(0.1f, 0.3f, 0.1f), 0.5f, kTol);
  // Degenerate denominator (a_to - d <= 0): no-op.
  EXPECT_NEAR(tennakoonRatioFactor(0.1f, 0.05f, 0.1f), 1.0f, kTol);
}

// --- sharma_cexp (Sharma et al., ICCAD 2015, Fig. 2) -----------------------

TEST(LambdaUpdaterFormula, SharmaCexpGrowsWhileViolating)
{
  // WPD = T - wns = 1.5 > r*T = 1.01, so cexp *= WPD/T = 1.5.
  EXPECT_NEAR(sharmaCexpStep(2.0f, -0.5f, 1.0f, 1.01f, 10.0f), 3.0f, kTol);
}

TEST(LambdaUpdaterFormula, SharmaCexpShrinksWhenMet)
{
  // WPD = 0.98 <= r*T; cexp *= 1 + 10*(0.98-1.01)/1.01 = 1 - 0.297029 =
  // 0.70297.
  EXPECT_NEAR(sharmaCexpStep(1.0f, 0.02f, 1.0f, 1.01f, 10.0f), 0.702970f, kTol);
  // No clock: cexp unchanged.
  EXPECT_NEAR(sharmaCexpStep(1.7f, -0.5f, 0.0f, 1.01f, 10.0f), 1.7f, kTol);
}

// An iteration with a lot of slack drives the shrink factor negative, but cexp
// is floored at a small positive value, not 0, so it can grow again later.
TEST(LambdaUpdaterFormula, SharmaCexpFlooredNotAbsorbing)
{
  // WPD = 0.5 <= r*T; factor = 1 + 10*(0.5-1.01)/1.01 = -4.0495; 0.5*(-4.0495)
  // = -2.02, floored to 1e-3 rather than 0.
  EXPECT_NEAR(sharmaCexpStep(0.5f, 0.5f, 1.0f, 1.01f, 10.0f), 1e-3f, 1e-9f);
  // A later violating iteration grows it again (WPD = 1.5 > r*T ->
  // cexp *= 1.5), which would be impossible from 0.
  EXPECT_NEAR(sharmaCexpStep(1e-3f, -0.5f, 1.0f, 1.01f, 10.0f), 1.5e-3f, 1e-9f);
}

// Fig. 2 line 10: the per-arc factor is (1 - slack_to/T)^cexp, the slack at
// the arc's sink relative to the clock period.
TEST(LambdaUpdaterFormula, SharmaCritFactor)
{
  // Violating arc (slack < 0): 1 + |slack|/T >= 1, dual ascent. slack=-0.1,
  // T=1, cexp=1 -> (1.1)^1 = 1.1; cexp=2 -> 1.21.
  EXPECT_NEAR(sharmaCritFactor(-0.1f, 1.0f, 1.0f), 1.1f, kTol);
  EXPECT_NEAR(sharmaCritFactor(-0.1f, 1.0f, 2.0f), 1.21f, kTol);
  // Met arc (slack > 0): linear decay 1 - slack/T, self-damping. slack=0.2 ->
  // 0.8.
  EXPECT_NEAR(sharmaCritFactor(0.2f, 1.0f, 1.0f), 0.8f, kTol);
  // Self-damping to 1 as slack -> 0 (a critical arc is neutral).
  EXPECT_NEAR(sharmaCritFactor(0.0f, 1.0f, 5.0f), 1.0f, kTol);
  // The base is bounded for |slack| <= T: a violation up to T gives a base of
  // at most 2. slack=-0.9, T=1 -> 1.9.
  EXPECT_NEAR(sharmaCritFactor(-0.9f, 1.0f, 1.0f), 1.9f, kTol);
  EXPECT_NEAR(sharmaCritFactor(-1.0f, 1.0f, 1.0f), 2.0f, kTol);  // slack = -T
  // slack > T: base 1 - slack/T crosses zero, floored positive (1e-3) so a
  // multiplicative lambda never hard-zeros. slack=1.5, T=1 -> base -0.5 ->
  // 1e-3.
  EXPECT_NEAR(sharmaCritFactor(1.5f, 1.0f, 1.0f), 1e-3f, 1e-9f);
  // cexp = 0: base^0 = 1 for any slack (the neutral exponent).
  EXPECT_NEAR(sharmaCritFactor(-0.5f, 1.0f, 0.0f), 1.0f, kTol);
  // No clock: no-op.
  EXPECT_NEAR(sharmaCritFactor(-0.1f, 0.0f, 2.0f), 1.0f, kTol);
}

// --- sharma_arc_slack (Sharma et al., TCAD 2020, Alg. 3) ------------------
//
// lambda <- lambda * (D/T)^K with D/T = 1 - s/T, where s is the arc's own
// slack. An arc is critical iff s < 0 (Sec. IV-C-2). Chinnery and Sharma (ISPD
// 2022, Sec. 4) use K = 4 on critical and 1 on non-critical arcs in the timing
// phase, and 1 and 4 in the power phase: the config defaults.

TEST(LambdaUpdaterFormula, SharmaArcSlackExponentsArePhaseAndSignDependent)
{
  const GlobalSizingConfig config;
  // Timing phase: critical 4, non-critical 1.
  EXPECT_FLOAT_EQ(sharmaArcSlackExponent(-0.1f, false, config), 4.0f);
  EXPECT_FLOAT_EQ(sharmaArcSlackExponent(0.2f, false, config), 1.0f);
  // Power phase: critical 1, non-critical 4.
  EXPECT_FLOAT_EQ(sharmaArcSlackExponent(-0.1f, true, config), 1.0f);
  EXPECT_FLOAT_EQ(sharmaArcSlackExponent(0.2f, true, config), 4.0f);
  // Zero slack is not negative, so the arc is non-critical.
  EXPECT_FLOAT_EQ(sharmaArcSlackExponent(0.0f, false, config), 1.0f);
  EXPECT_FLOAT_EQ(sharmaArcSlackExponent(0.0f, true, config), 4.0f);
}

TEST(LambdaUpdaterFormula, SharmaArcSlackFactorTimingPhase)
{
  const GlobalSizingConfig config;
  // s = -0.1T: (1 + 0.1)^4 = 1.4641.
  EXPECT_NEAR(sharmaArcSlackFactor(-0.1f, 1.0f, false, config), 1.4641f, kTol);
  // s = +0.2T: (1 - 0.2)^1 = 0.8.
  EXPECT_NEAR(sharmaArcSlackFactor(0.2f, 1.0f, false, config), 0.8f, kTol);
  // The same arcs in seconds, with T = 340 ps.
  EXPECT_NEAR(
      sharmaArcSlackFactor(-34e-12f, 340e-12f, false, config), 1.4641f, kTol);
  EXPECT_NEAR(
      sharmaArcSlackFactor(68e-12f, 340e-12f, false, config), 0.8f, kTol);
}

TEST(LambdaUpdaterFormula, SharmaArcSlackFactorPowerPhase)
{
  const GlobalSizingConfig config;
  // s = -0.1T: (1 + 0.1)^1 = 1.1.
  EXPECT_NEAR(sharmaArcSlackFactor(-0.1f, 1.0f, true, config), 1.1f, kTol);
  // s = +0.2T: (1 - 0.2)^4 = 0.4096.
  EXPECT_NEAR(sharmaArcSlackFactor(0.2f, 1.0f, true, config), 0.4096f, kTol);
}

TEST(LambdaUpdaterFormula, SharmaArcSlackFactorIsOneAtZeroSlack)
{
  // The base is 1 - 0/T = 1, so the exponent does not matter.
  const GlobalSizingConfig config;
  EXPECT_FLOAT_EQ(sharmaArcSlackFactor(0.0f, 1.0f, false, config), 1.0f);
  EXPECT_FLOAT_EQ(sharmaArcSlackFactor(0.0f, 1.0f, true, config), 1.0f);
}

TEST(LambdaUpdaterFormula, SharmaArcSlackFactorFloorsTheBaseAboveT)
{
  // s = 1.5T: the base 1 - 1.5 = -0.5 is floored at 1e-3, then raised to K.
  const GlobalSizingConfig config;
  EXPECT_NEAR(sharmaArcSlackFactor(1.5f, 1.0f, false, config), 1e-3f, 1e-9f);
  EXPECT_NEAR(sharmaArcSlackFactor(1.5f, 1.0f, true, config), 1e-12f, 1e-18f);
}

TEST(LambdaUpdaterFormula, SharmaArcSlackFactorWithoutClockIsOne)
{
  const GlobalSizingConfig config;
  EXPECT_FLOAT_EQ(sharmaArcSlackFactor(-0.1f, 0.0f, false, config), 1.0f);
  EXPECT_FLOAT_EQ(sharmaArcSlackFactor(0.2f, 0.0f, true, config), 1.0f);
  EXPECT_FLOAT_EQ(sharmaArcSlackFactor(0.2f, -1.0f, true, config), 1.0f);
}

// --- reimann_dwns (Reimann et al., ISPD 2016, Alg. 3) ----------------------

TEST(LambdaUpdaterFormula, ReimannRhoSchedule)
{
  EXPECT_NEAR(reimannRhoInc(1, 0.05f), 0.1f, kTol);  // 0.05*(1+1)
  EXPECT_NEAR(reimannRhoDec(1, 0.05f), 0.8f, kTol);  // 0.05*(15+1)
}

TEST(LambdaUpdaterFormula, ReimannDegradedArcGrows)
{
  // slack_curr(-0.11) <= slack_init(-0.1): increase branch.
  // denom = max(dwns,eps)*rho_inc = 1.0*0.1 = 0.1;
  // base = 1 - (-0.11 - (-0.1))/0.1 = 1 - (-0.1) = 1.1; ^{1/1} = 1.1.
  EXPECT_NEAR(reimannScaleFactor(-0.11f, -0.1f, 1.0f, 1.0f, 0.1f, 0.8f, 1.0f),
              1.1f,
              kTol);
}

TEST(LambdaUpdaterFormula, ReimannImprovedArcShrinks)
{
  // slack_curr(0.1) > slack_init(-0.1): decrease branch.
  // base = 1 + (0.1 - (-0.1))/(1*0.8) = 1.25; ^{-1} = 0.8.
  EXPECT_NEAR(reimannScaleFactor(0.1f, -0.1f, 1.0f, 1.0f, 0.1f, 0.8f, 1.0f),
              0.8f,
              kTol);
}

// The reimann_setpoint option only changes which reference slack the updater
// passes to reimannScaleFactor as slack_init: the paper's s_init (the arc's
// initial slack) or slack_target (the margin), which is not in the paper. On
// an arc that started violating (s_init = -0.3) and improved but still
// violates (slack_curr = -0.1), the two references take opposite branches.
TEST(LambdaUpdaterFormula, ReimannSetpointFlipsTheBranchOnAnImprovingArc)
{
  // Paper reference (slack_init = -0.3): slack_curr(-0.1) > -0.3 -> decrease
  // branch, factor < 1. The update holds timing near the initial slack, so an
  // improving arc loses lambda even though it still violates.
  // base = 1 + (-0.1 - (-0.3))/(1*0.8) = 1.25; ^{-1} = 0.8.
  const float faithful
      = reimannScaleFactor(-0.1f, -0.3f, 0.1f, 1.0f, 1.0f, 0.8f, 1.0f);
  EXPECT_NEAR(faithful, 0.8f, kTol);
  EXPECT_LT(faithful, 1.0f);

  // slack_target reference (slack_init = margin = 0): slack_curr(-0.1) <= 0 ->
  // increase branch on the still-violating arc, factor > 1 (dual ascent).
  // denom = max(dwns,0.1T)*rho_inc = max(0.1,0.1)*1 = 0.1;
  // base = 1 - (-0.1 - 0)/0.1 = 2; ^{1} = 2.
  const float setpoint
      = reimannScaleFactor(-0.1f, 0.0f, 0.1f, 1.0f, 1.0f, 0.8f, 1.0f);
  EXPECT_NEAR(setpoint, 2.0f, kTol);
  EXPECT_GT(setpoint, 1.0f);
}

// --- mu policy helpers -----------------------------------------------------

TEST(LambdaUpdaterFormula, MuSeedRaw)
{
  // gap = margin - slack = 0 - (-0.5) = 0.5; 0.5^2 = 0.25.
  EXPECT_NEAR(muSeedRaw(-0.5f, 0.0f, 2.0f), 0.25f, kTol);
  // Positive slack: no contribution.
  EXPECT_NEAR(muSeedRaw(0.5f, 0.0f, 2.0f), 0.0f, kTol);
}

TEST(LambdaUpdaterFormula, MuUpdateFactor)
{
  // 1 + (margin - slack)/T = 1 + 0.5 = 1.5.
  EXPECT_NEAR(muUpdateFactor(-0.5f, 0.0f, 1.0f), 1.5f, kTol);
  // Clamped to [0, 2] from below.
  EXPECT_NEAR(muUpdateFactor(3.0f, 0.0f, 1.0f), 0.0f, kTol);
  // No clock: no-op.
  EXPECT_NEAR(muUpdateFactor(-0.5f, 0.0f, 0.0f), 1.0f, kTol);
}

// endpoint_ratio (Tennakoon Fig. 13 branch 1, identical to Livramento Alg. 1
// line 12): mu_k *= a_k / required_k.
TEST(LambdaUpdaterFormula, MuRatioFactor)
{
  // Violating endpoint (a > required, i.e. slack < 0): factor > 1, mu grows.
  // a=0.3, required=0.2 -> 1.5.
  EXPECT_NEAR(muRatioFactor(0.3f, 0.2f), 1.5f, kTol);
  // Met endpoint (a < required): factor < 1, mu damps. a=0.2, required=0.3.
  EXPECT_NEAR(muRatioFactor(0.2f, 0.3f), 2.0f / 3.0f, kTol);
  // Tends to 1 as a -> required (the papers' own gain control: a -> A_0).
  EXPECT_NEAR(muRatioFactor(0.25f, 0.25f), 1.0f, kTol);
  // Guards: a non-positive arrival or required is neutral (never collapses mu).
  EXPECT_NEAR(muRatioFactor(0.0f, 0.2f), 1.0f, kTol);
  EXPECT_NEAR(muRatioFactor(0.2f, -0.1f), 1.0f, kTol);
}

// endpoint_additive (Chen et al., SOLVE_LDP step 3, i = 0 branch):
// mu_k <- max(0, mu_k + rho*(-slack)/T).
TEST(LambdaUpdaterFormula, MuAdditiveStep)
{
  // Violating endpoint (-slack > 0): mu grows. mu=1, slack=-0.1, rho=1, T=1 ->
  // 1 + 0.1 = 1.1.
  EXPECT_NEAR(muAdditiveStep(1.0f, -0.1f, 1.0f, 1.0f), 1.1f, kTol);
  // Met endpoint: mu decays. mu=1, slack=0.1 -> 1 - 0.1 = 0.9.
  EXPECT_NEAR(muAdditiveStep(1.0f, 0.1f, 1.0f, 1.0f), 0.9f, kTol);
  // rho_k -> 0 damps the step (the paper's gain control): a small rho barely
  // moves mu even on a large violation. rho=0.01, slack=-1, T=1 -> 1 + 0.01.
  EXPECT_NEAR(muAdditiveStep(1.0f, -1.0f, 0.01f, 1.0f), 1.01f, kTol);
  // Floored at 0, but an additive step can leave 0: a met endpoint can drive
  // mu to 0, and a later violation brings it back.
  EXPECT_NEAR(muAdditiveStep(0.05f, 1.0f, 1.0f, 1.0f), 0.0f, kTol);
  EXPECT_NEAR(muAdditiveStep(0.0f, -0.1f, 1.0f, 1.0f), 0.1f, kTol);
  // No clock: no-op.
  EXPECT_NEAR(muAdditiveStep(1.0f, -0.1f, 1.0f, 0.0f), 1.0f, kTol);
}

// --- factory dispatch ------------------------------------------------------

TEST(LambdaUpdaterFactory, DispatchesEveryOption)
{
  using LU = GlobalSizingConfig::LambdaUpdate;
  for (const LU option : {LU::kNormSubgradient,
                          LU::kFlachSlackScaling,
                          LU::kChenSubgradient,
                          LU::kTennakoonRatio,
                          LU::kSharmaCexp,
                          LU::kReimannDwns,
                          LU::kLivramentoRatio,
                          LU::kSharmaArcSlack}) {
    GlobalSizingConfig config;
    config.lambda_update = option;
    std::unique_ptr<LambdaUpdater> updater = makeLambdaUpdater(config);
    ASSERT_NE(updater, nullptr);
  }
  // The default updater starts its step at beta; sharma starts cexp at 1.
  GlobalSizingConfig config;
  config.beta = 0.6f;
  EXPECT_NEAR(makeLambdaUpdater(config)->currentStep(), 0.6f, kTol);
  config.lambda_update = LU::kSharmaCexp;
  EXPECT_NEAR(makeLambdaUpdater(config)->currentStep(), 1.0f, kTol);
}

// --- consistent rise/fall read ---------------------------------------------
//
// pickCriticalArcTransition is the STA-free core of
// LrState::consistentArcRead: it selects the (a_from, d) pair that produces
// the edge's own worst propagated arrival (max a_from + d). The edge below is
// rise/fall asymmetric, so its two (arc, transition) reads propagate different
// arrivals:
//   rise: a_from = 5, d = 3  -> propagated 8
//   fall: a_from = 2, d = 7  -> propagated 9   (the actual worst)
// Taking the worst arrival over transitions (5) and the max arc delay (7)
// separately would give a_from + d = 12, which no transition produces.

TEST(ConsistentArcRead, PicksTheWorstPropagatedArcTransition)
{
  const std::vector<ArcTransitionRead> reads = {{5.0f, 3.0f}, {2.0f, 7.0f}};
  const ArcTransitionRead crit = pickCriticalArcTransition(reads);
  EXPECT_NEAR(crit.a_from, 2.0f, kTol);
  EXPECT_NEAR(crit.d, 7.0f, kTol);
}

TEST(ConsistentArcRead, CriticalArcReadsZeroNotAPhantomPositive)
{
  // On the vertex's critical in-edge a_to = the edge's own worst propagated
  // arrival = 9. The consistent read gives violation a_from + d - a_to = 0.
  const std::vector<ArcTransitionRead> reads = {{5.0f, 3.0f}, {2.0f, 7.0f}};
  const float a_to = 9.0f;

  // Mixing transitions: 5 + 7 - 9 = +3, a false violation that would push λ
  // up on every iteration.
  EXPECT_GT(5.0f + 7.0f - a_to, 0.0f);

  // The consistent pair (2, 7) gives exactly 0: tight, not violating.
  const ArcTransitionRead crit = pickCriticalArcTransition(reads);
  EXPECT_NEAR(crit.a_from + crit.d - a_to, 0.0f, kTol);
}

TEST(ConsistentArcRead, MetArcReadsNegativeNotPhantomTight)
{
  // A non-critical (met) edge: a different, more critical in-edge sets the
  // to-vertex arrival to 12. The consistent read gives 9 - 12 = -3 (met),
  // while mixing transitions would give 12 - 12 = 0 and make the edge look
  // tight.
  const std::vector<ArcTransitionRead> reads = {{5.0f, 3.0f}, {2.0f, 7.0f}};
  const float a_to = 12.0f;
  const ArcTransitionRead crit = pickCriticalArcTransition(reads);
  EXPECT_NEAR(crit.a_from + crit.d - a_to, -3.0f, kTol);
  EXPECT_LT(crit.a_from + crit.d - a_to, 5.0f + 7.0f - a_to);
}

TEST(ConsistentArcRead, EmptyReadsReturnZero)
{
  const std::vector<ArcTransitionRead> reads;
  const ArcTransitionRead crit = pickCriticalArcTransition(reads);
  EXPECT_NEAR(crit.a_from, 0.0f, kTol);
  EXPECT_NEAR(crit.d, 0.0f, kTol);
}

// LrState::consistentArcRead on a timed design: the TestObjectivePower.v
// netlist with the fixture's 0.5 ns clock and zero I/O delays. The expected
// values are OpenSTA's own arrivals. On every data in-edge of u_nand/ZN (two
// inputs) and u_inv/ZN (one input), a_from is the arrival OpenSTA reports at
// the from pin in one of its transitions and a_from + d <= a_to. The in-edge
// that sets the output's arrival reproduces it exactly.
class ConsistentArcReadOnSta : public tst::IntegratedFixture
{
 public:
  ConsistentArcReadOnSta()
      : tst::IntegratedFixture(tst::IntegratedFixture::Technology::kNangate45,
                               "_main/src/rsz/test/")
  {
  }
};

TEST_F(ConsistentArcReadOnSta, ReadsOpenStaArrivals)
{
  readVerilogAndSetup("TestObjectivePower.v");
  sta_->updateTiming(false);
  const sta::MinMax* max = sta::MinMax::max();
  GlobalSizingConfig config;
  LrState state;
  state.sta = sta_.get();
  state.graph = sta_->graph();
  state.config = &config;
  state.dcalc_ap = sta_->cmdScene()->dcalcAnalysisPtIndex(max);
  // 1 fs, far above float rounding of arrivals near 1e-10 s.
  constexpr float kArrivalTol = 1e-15f;

  for (const char* name : {"u_nand", "u_inv"}) {
    sta::Instance* inst = db_network_->dbToSta(block_->findInst(name));
    ASSERT_NE(inst, nullptr) << name;
    sta::Vertex* to_v
        = state.graph->pinDrvrVertex(db_network_->findPin(inst, "ZN"));
    ASSERT_NE(to_v, nullptr) << name;
    const float a_to = sta::delayAsFloat(sta_->arrival(
        to_v, sta::RiseFallBoth::riseFall(), sta_->scenes(), max));
    float worst = -LrState::kSlackSentinel;
    int edges = 0;
    sta::VertexInEdgeIterator edge_iter(to_v, state.graph);
    while (edge_iter.hasNext()) {
      sta::Edge* edge = edge_iter.next();
      if (!state.isDataArc(edge)) {
        continue;
      }
      sta::Vertex* from_v = edge->from(state.graph);
      const LrState::ConsistentArcRead read
          = state.consistentArcRead(edge, from_v, to_v);
      const float rise = sta::delayAsFloat(sta_->arrival(
          from_v, sta::RiseFallBoth::rise(), sta_->scenes(), max));
      const float fall = sta::delayAsFloat(sta_->arrival(
          from_v, sta::RiseFallBoth::fall(), sta_->scenes(), max));
      EXPECT_TRUE(read.a_from == rise || read.a_from == fall)
          << name << ": a_from " << read.a_from << ", OpenSTA rise " << rise
          << " fall " << fall;
      EXPECT_FLOAT_EQ(read.a_to, a_to) << name;
      EXPECT_LE(read.a_from + read.d, read.a_to + kArrivalTol) << name;
      worst = std::max(worst, read.a_from + read.d);
      ++edges;
    }
    EXPECT_EQ(edges, std::string(name) == "u_nand" ? 2 : 1);
    EXPECT_NEAR(worst, a_to, kArrivalTol) << name;
  }
}

// LrState::arcSlack, the slack of a data edge: the worst of q_to - a_from - d
// over its (arc, transition) pairs, each read in the arc's own transitions
// (Sharma et al., TCAD 2020, Sec. IV-C-2). The expected values are OpenSTA's
// pin slacks. A path through an edge is a path through both of its pins, so
// the edge's slack is at least each pin's slack. A pin's worst path enters it
// through one of its in-edges, so the least slack over a pin's in-edges equals
// the pin's slack. Checked on every pin of the design that has data in-edges.
class ArcSlackOnSta : public ConsistentArcReadOnSta
{
};

TEST_F(ArcSlackOnSta, IsTheWorstSlackOfAPathThroughTheEdge)
{
  readVerilogAndSetup("TestObjectivePower.v");
  sta_->updateTiming(false);
  const sta::MinMax* max = sta::MinMax::max();
  GlobalSizingConfig config;
  LrState state;
  state.sta = sta_.get();
  state.graph = sta_->graph();
  state.config = &config;
  state.dcalc_ap = sta_->cmdScene()->dcalcAnalysisPtIndex(max);
  constexpr float kSlackTol = 1e-15f;
  const auto slack
      = [&](sta::Vertex* v) { return sta::delayAsFloat(sta_->slack(v, max)); };
  const auto constrained
      = [](const float x) { return std::fabs(x) < LrState::kSlackSentinel; };

  int pins = 0;
  sta::VertexIterator vertex_iter(state.graph);
  while (vertex_iter.hasNext()) {
    sta::Vertex* to_v = vertex_iter.next();
    const float slack_to = slack(to_v);
    if (!constrained(slack_to)) {
      continue;
    }
    float least = LrState::kSlackSentinel;
    sta::VertexInEdgeIterator edge_iter(to_v, state.graph);
    while (edge_iter.hasNext()) {
      sta::Edge* edge = edge_iter.next();
      if (!state.isDataArc(edge)) {
        continue;
      }
      sta::Vertex* from_v = edge->from(state.graph);
      const float arc_slack = state.arcSlack(edge, from_v, to_v);
      const std::string name = to_v->name(db_network_);
      ASSERT_TRUE(constrained(arc_slack)) << name;
      EXPECT_GE(arc_slack, slack_to - kSlackTol) << name;
      const float slack_from = slack(from_v);
      if (constrained(slack_from)) {
        EXPECT_GE(arc_slack, slack_from - kSlackTol) << name;
      }
      least = std::min(least, arc_slack);
    }
    if (least < LrState::kSlackSentinel) {
      EXPECT_NEAR(least, slack_to, kSlackTol) << to_v->name(db_network_);
      ++pins;
    }
  }
  EXPECT_GT(pins, 20);
}

// --- multipliers across cell swaps -----------------------------------------
//
// carryMultipliers is the STA-free core of LrState::refreshLiveEdges. Each
// case gives, per multiplier slot (edge id), the ends of the edge that held it
// before a cell swap and of the edge that holds it after. P, Q, R and S are
// four pin pairs into one output pin (vertex 9). An empty entry is a slot no
// live edge holds.

const EdgeEnds kP{.from = 1, .to = 9};
const EdgeEnds kQ{.from = 2, .to = 9};
const EdgeEnds kR{.from = 3, .to = 9};
const EdgeEnds kS{.from = 4, .to = 9};
const EdgeEnds kNone{};

TEST(CarryMultipliers, NewIdBeyondTheVectorTakesItsPinPairMultiplier)
{
  // Q's edge is re-created with id 2, past the end of lambda. It takes Q's
  // 0.6 instead of 0; the slot it left holds no edge and drops to 0.
  std::vector<EdgeEnds> slot_ends = {kP, kQ};
  std::vector<float> lambda = {0.4f, 0.6f};
  const EdgeCarryStats stats
      = carryMultipliers({kP, kNone, kQ}, slot_ends, lambda);
  ASSERT_EQ(lambda.size(), 3u);
  EXPECT_FLOAT_EQ(lambda[0], 0.4f);
  EXPECT_FLOAT_EQ(lambda[1], 0.0f);
  EXPECT_FLOAT_EQ(lambda[2], 0.6f);
  EXPECT_EQ(slot_ends, (std::vector<EdgeEnds>{kP, kNone, kQ}));
  EXPECT_EQ(stats.carried, 1);
  EXPECT_EQ(stats.started_at_zero, 0);
}

TEST(CarryMultipliers, RecycledIdOfAnotherArcTakesItsOwnPinPairMultiplier)
{
  // Q's and R's edges are re-created on each other's ids. Each takes its own
  // pin pair's multiplier, not the one its slot held.
  std::vector<EdgeEnds> slot_ends = {kP, kQ, kR};
  std::vector<float> lambda = {0.1f, 0.2f, 0.3f};
  const EdgeCarryStats stats
      = carryMultipliers({kP, kR, kQ}, slot_ends, lambda);
  EXPECT_FLOAT_EQ(lambda[0], 0.1f);
  EXPECT_FLOAT_EQ(lambda[1], 0.3f);
  EXPECT_FLOAT_EQ(lambda[2], 0.2f);
  EXPECT_EQ(slot_ends, (std::vector<EdgeEnds>{kP, kR, kQ}));
  EXPECT_EQ(stats.carried, 2);
  EXPECT_EQ(stats.started_at_zero, 0);
}

TEST(CarryMultipliers, RecycledIdThatMatchesIsLeftAlone)
{
  // Both edges are re-created on the ids they had: nothing to carry.
  std::vector<EdgeEnds> slot_ends = {kP, kQ};
  std::vector<float> lambda = {0.1f, 0.2f};
  const EdgeCarryStats stats = carryMultipliers({kP, kQ}, slot_ends, lambda);
  EXPECT_EQ(lambda, (std::vector<float>{0.1f, 0.2f}));
  EXPECT_EQ(slot_ends, (std::vector<EdgeEnds>{kP, kQ}));
  EXPECT_EQ(stats.carried, 0);
  EXPECT_EQ(stats.started_at_zero, 0);
}

TEST(CarryMultipliers, OneEdgeBecomingTwoSplitsItsMultiplier)
{
  // Q had one edge (0.5). The new cell has two arcs between Q's pins, on the
  // recycled id 1 and the new id 2; each gets half.
  std::vector<EdgeEnds> slot_ends = {kP, kQ};
  std::vector<float> lambda = {0.8f, 0.5f};
  const EdgeCarryStats stats
      = carryMultipliers({kP, kQ, kQ}, slot_ends, lambda);
  EXPECT_FLOAT_EQ(lambda[0], 0.8f);
  EXPECT_FLOAT_EQ(lambda[1], 0.25f);
  EXPECT_FLOAT_EQ(lambda[2], 0.25f);
  EXPECT_EQ(stats.carried, 2);
}

TEST(CarryMultipliers, TwoEdgesBecomingOneGetTheirSum)
{
  // Q had two edges (0.3 + 0.5). The new cell has one, on the new id 3.
  std::vector<EdgeEnds> slot_ends = {kP, kQ, kQ};
  std::vector<float> lambda = {0.8f, 0.3f, 0.5f};
  const EdgeCarryStats stats
      = carryMultipliers({kP, kNone, kNone, kQ}, slot_ends, lambda);
  EXPECT_FLOAT_EQ(lambda[0], 0.8f);
  EXPECT_FLOAT_EQ(lambda[1], 0.0f);
  EXPECT_FLOAT_EQ(lambda[2], 0.0f);
  EXPECT_FLOAT_EQ(lambda[3], 0.8f);
  EXPECT_EQ(stats.carried, 1);
}

TEST(CarryMultipliers, TwoEdgesBecomingOneOnAnIdOfTheSamePairGetTheirSum)
{
  // As above, but the surviving edge is re-created on id 2, which Q already
  // held. Its slot matches, yet Q lost slot 1, so Q is still carried: the
  // edge gets 0.3 + 0.5, not just the 0.5 its slot held.
  std::vector<EdgeEnds> slot_ends = {kP, kQ, kQ};
  std::vector<float> lambda = {0.8f, 0.3f, 0.5f};
  const EdgeCarryStats stats
      = carryMultipliers({kP, kNone, kQ}, slot_ends, lambda);
  EXPECT_FLOAT_EQ(lambda[0], 0.8f);
  EXPECT_FLOAT_EQ(lambda[1], 0.0f);
  EXPECT_FLOAT_EQ(lambda[2], 0.8f);
  EXPECT_EQ(stats.carried, 1);
}

TEST(CarryMultipliers, PinPairWithNoHistoryStartsAtZero)
{
  // The new cell has no arc between Q's pins and a new arc between S's pins,
  // re-created on Q's old id 1 and the new id 2. S had no edge before, so
  // both start at 0 instead of inheriting Q's 0.5.
  std::vector<EdgeEnds> slot_ends = {kP, kQ};
  std::vector<float> lambda = {0.8f, 0.5f};
  const EdgeCarryStats stats
      = carryMultipliers({kP, kS, kS}, slot_ends, lambda);
  EXPECT_FLOAT_EQ(lambda[0], 0.8f);
  EXPECT_FLOAT_EQ(lambda[1], 0.0f);
  EXPECT_FLOAT_EQ(lambda[2], 0.0f);
  EXPECT_EQ(stats.carried, 0);
  EXPECT_EQ(stats.started_at_zero, 2);
}

TEST(CarryMultipliers, UnaffectedEdgesKeepTheirExactValues)
{
  // Only R's edge is re-created (new id 4). P's and Q's two edges keep their
  // ids and their values bit for bit, including Q's uneven pair.
  std::vector<EdgeEnds> slot_ends = {kP, kQ, kQ, kR};
  std::vector<float> lambda = {0.1f, 0.2f, 0.7f, 0.3f};
  const EdgeCarryStats stats
      = carryMultipliers({kP, kQ, kQ, kNone, kR}, slot_ends, lambda);
  EXPECT_EQ(lambda[0], 0.1f);
  EXPECT_EQ(lambda[1], 0.2f);
  EXPECT_EQ(lambda[2], 0.7f);
  EXPECT_FLOAT_EQ(lambda[4], 0.3f);
  EXPECT_EQ(stats.carried, 1);
}

TEST(CarryMultipliers, ADeadSlotDoesNotFeedALaterRefresh)
{
  // First swap: Q's edge moves from id 1 to the new id 2. Second swap: Q gets
  // a second edge on the recycled id 1. Q's total before the second swap is
  // 0.6 (slot 2 only); the slot-1 entry from before the first swap is gone.
  std::vector<EdgeEnds> slot_ends = {kP, kQ};
  std::vector<float> lambda = {0.4f, 0.6f};
  carryMultipliers({kP, kNone, kQ}, slot_ends, lambda);
  const EdgeCarryStats stats
      = carryMultipliers({kP, kQ, kQ}, slot_ends, lambda);
  EXPECT_FLOAT_EQ(lambda[0], 0.4f);
  EXPECT_FLOAT_EQ(lambda[1], 0.3f);
  EXPECT_FLOAT_EQ(lambda[2], 0.3f);
  EXPECT_EQ(stats.carried, 2);
}

// LrState::refreshLiveEdges on OpenSTA. The ASAP7 AOI221xp5 and AOI221x1 are
// equivalent cells whose timing arc sets differ: between C and Y the xp5 has
// ten arc sets (nine conditional, one unconditional) and the x1 nine. Swapping
// one for the other makes OpenSTA delete the instance's edges and create new
// ones. After the refresh, each pin pair's new edges share the multipliers its
// old edges had. Nangate45 has no such pair of cells.
class CarryAcrossSwapOnSta : public tst::Fixture
{
 protected:
  CarryAcrossSwapOnSta()
  {
    const std::string asap7 = "_main/src/rsz/test/asap7/";
    library_ = readLiberty(asap7 + "asap7sc7p5t_AO_RVT_FF_nldm_211120.lib.gz");
    odb::dbTech* tech
        = loadTechLef("asap7", asap7 + "asap7_tech_1x_201209.lef");
    loadLibaryLef(tech, "asap7sc", asap7 + "asap7sc7p5t_28_R_1x_220121a.lef");
    odb::dbChip* chip = odb::dbChip::create(db_.get(), tech);
    block_ = odb::dbBlock::create(chip, "top");
    block_->setDieArea(odb::Rect(0, 0, 1000, 1000));
    db_network_ = sta_->getDbNetwork();
    db_network_->setBlock(block_);
    sta_->postReadDef(block_);

    for (const char* in : {"A1", "A2", "B1", "B2", "C"}) {
      makeBTerm(block_, in);
    }
    makeBTerm(block_, "Y", {.io_type = odb::dbIoType::OUTPUT});
    makeInst(block_,
             db_->findMaster(kSmall),
             "u",
             {.iterms = {{.net_name = "A1", .term_name = "A1"},
                         {.net_name = "A2", .term_name = "A2"},
                         {.net_name = "B1", .term_name = "B1"},
                         {.net_name = "B2", .term_name = "B2"},
                         {.net_name = "C", .term_name = "C"},
                         {.net_name = "Y", .term_name = "Y"}}});
  }

  // The data edges of instance u, the ones a swap re-creates.
  std::vector<sta::Edge*> instanceEdges(const LrState& state) const
  {
    sta::Instance* inst = db_network_->dbToSta(block_->findInst("u"));
    sta::Vertex* y
        = state.graph->pinDrvrVertex(db_network_->findPin(inst, "Y"));
    std::vector<sta::Edge*> edges;
    sta::VertexInEdgeIterator edge_iter(y, state.graph);
    while (edge_iter.hasNext()) {
      sta::Edge* edge = edge_iter.next();
      if (state.isDataArc(edge)
          && db_network_->instance(edge->from(state.graph)->pin()) == inst) {
        edges.push_back(edge);
      }
    }
    return edges;
  }

  static constexpr const char* kSmall = "AOI221xp5_ASAP7_75t_R";
  static constexpr const char* kLarge = "AOI221x1_ASAP7_75t_R";
  sta::LibertyLibrary* library_ = nullptr;
  odb::dbBlock* block_ = nullptr;
  sta::dbNetwork* db_network_ = nullptr;
};

TEST_F(CarryAcrossSwapOnSta, NewEdgesShareTheirPinPairMultipliers)
{
  const sta::LibertyCell* small = library_->findLibertyCell(kSmall);
  const sta::LibertyCell* large = library_->findLibertyCell(kLarge);
  ASSERT_NE(small, nullptr);
  ASSERT_NE(large, nullptr);
  ASSERT_TRUE(sta::equivCells(small, large));
  ASSERT_FALSE(sta::equivCellsArcs(small, large));

  GlobalSizingConfig config;
  LrState state;
  state.sta = sta_.get();
  state.graph = sta_->ensureGraph();
  state.config = &config;
  state.logger = &logger_;
  state.allocate();

  // Give every edge of u a distinct multiplier: 1, 2, 3, ... in graph order.
  // Sum them per input pin; every edge of u ends at Y.
  std::map<std::string, float> pin_sum;
  float value = 0.0f;
  for (sta::Edge* edge : instanceEdges(state)) {
    value += 1.0f;
    state.lambda[state.graph->id(edge)] = value;
    pin_sum[edge->from(state.graph)->name(db_network_)] += value;
  }
  // 26 edges: four per A1, A2, B1 and B2, and ten for C.
  ASSERT_EQ(value, 26.0f);
  ASSERT_EQ(pin_sum.size(), 5u);
  // The wire edges into u's pins are not re-created; they keep their value.
  sta::Edge* wire = nullptr;
  {
    sta::Vertex* port_c = state.graph->pinDrvrVertex(
        db_network_->dbToSta(block_->findBTerm("C")));
    sta::VertexOutEdgeIterator edge_iter(port_c, state.graph);
    wire = edge_iter.next();
    state.lambda[state.graph->id(wire)] = 0.125f;
  }

  // OpenSTA may hand a re-created edge any free id, so only what the rule
  // fixes regardless of the ids is checked. Every pin keeps its total. C -> Y
  // goes from ten edges to nine and back, so C is always carried, and each of
  // its edges gets its total divided by nine, then by ten.
  for (const char* master : {kLarge, kSmall}) {
    ASSERT_TRUE(block_->findInst("u")->swapMaster(db_->findMaster(master)));
    const EdgeCarryStats stats = state.refreshLiveEdges();
    const int c_edges = master == kLarge ? 9 : 10;
    EXPECT_GE(stats.carried, c_edges) << master;
    EXPECT_EQ(stats.started_at_zero, 0) << master;

    std::map<std::string, int> pin_edges;
    std::map<std::string, float> new_sum;
    for (sta::Edge* edge : instanceEdges(state)) {
      const std::string pin = edge->from(state.graph)->name(db_network_);
      const float lam = state.lambda[state.graph->id(edge)];
      ++pin_edges[pin];
      new_sum[pin] += lam;
      if (pin == "u/C") {
        const float share = pin_sum[pin] / c_edges;
        EXPECT_NEAR(lam, share, 1e-5f * share) << master;
      }
    }
    EXPECT_EQ(pin_edges,
              (std::map<std::string, int>{{"u/A1", 4},
                                          {"u/A2", 4},
                                          {"u/B1", 4},
                                          {"u/B2", 4},
                                          {"u/C", c_edges}}))
        << master;
    for (const auto& [pin, sum] : pin_sum) {
      EXPECT_NEAR(new_sum[pin], sum, 1e-5f * sum) << master << " " << pin;
    }
    EXPECT_EQ(state.lambda[state.graph->id(wire)], 0.125f) << master;
  }
}

}  // namespace
}  // namespace rsz
