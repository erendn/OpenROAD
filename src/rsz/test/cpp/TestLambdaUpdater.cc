// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the M1 lambda-update family. Each updater's per-arc /
// per-endpoint arithmetic lives in a pure free function (no STA), so these
// tests assert the formula against hand-computed values. The full update()
// path (STA + graph traversal) is exercised by the global_sizing_lambda_update
// integration test.

#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "lr/LambdaUpdater.hh"
#include "lr/LrState.hh"
#include "rsz/GlobalSizingConfig.hh"

namespace rsz {
namespace {

constexpr float kTol = 1e-5f;
constexpr float kFloor = 1e-12f;

// --- baseline norm_subgradient ---------------------------------------------

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

// --- flach_slack_scaling ---------------------------------------------------

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

// --- chen_subgradient ------------------------------------------------------

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

// The regression this normalization exists for: with SI-second violations
// (~1e-10) against an O(1) lambda, the un-normalized step `lambda + rho*
// violation` rounds straight back to lambda in float32 and chen's dual ascent
// silently never fires. Normalizing by T keeps the step representable at the
// paper-default c = 1.
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

// --- livramento_ratio ------------------------------------------------------

TEST(LambdaUpdaterFormula, LivramentoRatio)
{
  // Alg. 1 line 13: (a_from + d)/a_to. Critical arc (a_to == a_from + d) is
  // neutral.
  EXPECT_NEAR(livramentoRatioFactor(0.1f, 0.2f, 0.1f), 1.0f, kTol);
  // Non-critical: (1 + 1)/3.
  EXPECT_NEAR(livramentoRatioFactor(1.0f, 3.0f, 1.0f), 2.0f / 3.0f, kTol);
}

// The reason this updater exists rather than hosting livramento on
// tennakoon_ratio: the two rules use different local step sizes (Livramento
// Eq. 9 rho_k = lambda/a_i; Tennakoon Fig. 13 rho_k = lambda/(a_i - D_i)), so
// they agree ONLY on a critical arc and diverge everywhere else.
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

// Alg. 1 line 14 (PI-sourced arcs, D_ji/a_i) needs no branch of its own: it is
// line 13 at a_from == 0. Unlike tennakoon's form, which returns 0/(a_to - d)
// = 0 here and collapses lambda to the floor - zero is absorbing under a
// multiplicative update.
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

// --- tennakoon_ratio -------------------------------------------------------

TEST(LambdaUpdaterFormula, TennakoonRatio)
{
  // Critical arc: a_to - d == a_from, ratio = 1.
  EXPECT_NEAR(tennakoonRatioFactor(0.1f, 0.2f, 0.1f), 1.0f, kTol);
  // Slack arc: a_to - d = 0.2, ratio = 0.1/0.2 = 0.5.
  EXPECT_NEAR(tennakoonRatioFactor(0.1f, 0.3f, 0.1f), 0.5f, kTol);
  // Degenerate denominator (a_to - d <= 0): no-op.
  EXPECT_NEAR(tennakoonRatioFactor(0.1f, 0.05f, 0.1f), 1.0f, kTol);
}

// --- sharma_cexp -----------------------------------------------------------

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

// The absorbing-clamp fix (sharma audit §2 item 2): an over-met iteration
// drives the shrink factor negative, but cexp floors at a small POSITIVE value,
// not 0.
TEST(LambdaUpdaterFormula, SharmaCexpFlooredNotAbsorbing)
{
  // WPD = 0.5 <= r*T; factor = 1 + 10*(0.5-1.01)/1.01 = -4.0495; 0.5*(-4.0495)
  // = -2.02, floored to 1e-3 (NOT the absorbing 0 the old max(0,.) produced).
  EXPECT_NEAR(sharmaCexpStep(0.5f, 0.5f, 1.0f, 1.01f, 10.0f), 1e-3f, 1e-9f);
  // And the floor is not absorbing: a subsequent violating iteration regrows it
  // (WPD = 1.5 > r*T -> cexp *= 1.5), which the old 0-clamp could never do.
  EXPECT_NEAR(sharmaCexpStep(1e-3f, -0.5f, 1.0f, 1.01f, 10.0f), 1.5e-3f, 1e-9f);
}

// Re-pinned to the CORRECTED Fig. 2 line 10 base (1 - slack_to/T)^cexp - the
// sink-node slack over the clock period. The old test pinned (a_j/q_j)^cexp =
// 4, which was transcription evidence for the bug, not correctness evidence.
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
  // The base is BOUNDED for |slack| <= T (the old (a_j/q_j) diverged as q->0+):
  // a deep-but-<=T violation gives base <= 2, not tens. slack=-0.9, T=1 -> 1.9.
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

// --- reimann_dwns ----------------------------------------------------------

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

// The reimann_setpoint knob (C1) changes only which reference slack the updater
// feeds reimannScaleFactor as slack_init: the faithful s_init (the frozen
// initial slack) vs. the disclosed slack_target (the margin). On an arc that
// STARTED violating (s_init = -0.3) and IMPROVED but is still violating
// (slack_curr = -0.1), the two references pick opposite branches - which is the
// whole point of the adaptation.
TEST(LambdaUpdaterFormula, ReimannSetpointFlipsTheBranchOnAnImprovingArc)
{
  // Faithful s_init reference (slack_init = -0.3): slack_curr(-0.1) > -0.3 ->
  // DECREASE branch, factor < 1 (the measured decay: the servo holds near the
  // input timing, so an improving arc sheds lambda).
  // base = 1 + (-0.1 - (-0.3))/(1*0.8) = 1.25; ^{-1} = 0.8.
  const float faithful
      = reimannScaleFactor(-0.1f, -0.3f, 0.1f, 1.0f, 1.0f, 0.8f, 1.0f);
  EXPECT_NEAR(faithful, 0.8f, kTol);
  EXPECT_LT(faithful, 1.0f);

  // slack_target reference (slack_init = margin = 0): slack_curr(-0.1) <= 0 ->
  // INCREASE branch fires on the still-violating arc, factor > 1 (dual ascent).
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

// endpoint_ratio (Tennakoon Fig. 13 branch 1 = Livramento Alg. 1 L12):
// mu_k *= a_k / required_k.
TEST(LambdaUpdaterFormula, MuRatioFactor)
{
  // Violating endpoint (a > required, i.e. slack < 0): factor > 1, mu grows.
  // a=0.3, required=0.2 -> 1.5.
  EXPECT_NEAR(muRatioFactor(0.3f, 0.2f), 1.5f, kTol);
  // Met endpoint (a < required): factor < 1, mu damps. a=0.2, required=0.3.
  EXPECT_NEAR(muRatioFactor(0.2f, 0.3f), 2.0f / 3.0f, kTol);
  // Self-damping to 1 as a -> required (the papers' own gain control: a ->
  // A_0).
  EXPECT_NEAR(muRatioFactor(0.25f, 0.25f), 1.0f, kTol);
  // Guards: a non-positive arrival or required is neutral (never collapses mu).
  EXPECT_NEAR(muRatioFactor(0.0f, 0.2f), 1.0f, kTol);
  EXPECT_NEAR(muRatioFactor(0.2f, -0.1f), 1.0f, kTol);
}

// endpoint_additive (Chen SOLVE_LDP step 3 i=0 branch):
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
  // Floored at 0, but 0 is NOT absorbing under an additive step: a met endpoint
  // can drive mu to 0, and a later violation recovers it.
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
                          LU::kLivramentoRatio}) {
    GlobalSizingConfig config;
    config.lambda_update = option;
    std::unique_ptr<LambdaUpdater> updater = makeLambdaUpdater(config);
    ASSERT_NE(updater, nullptr);
  }
  // The baseline seeds its step from beta; sharma seeds cexp at 1.
  GlobalSizingConfig config;
  config.beta = 0.6f;
  EXPECT_NEAR(makeLambdaUpdater(config)->currentStep(), 0.6f, kTol);
  config.lambda_update = LU::kSharmaCexp;
  EXPECT_NEAR(makeLambdaUpdater(config)->currentStep(), 1.0f, kTol);
}

// --- consistent rise/fall read (C3 item 1) ---------------------------------
//
// pickCriticalArcTransition is the pure core of LrState::consistentArcRead: it
// selects the (a_from, d) pair realizing an edge's own worst propagated arrival
// (max a_from + d). The synthetic edge below is rise/fall-asymmetric — its two
// (arc, transition) reads propagate different arrivals:
//   rise: a_from = 5, d = 3  -> propagated 8
//   fall: a_from = 2, d = 7  -> propagated 9   (the realized worst)
// The retired collapse read instead took a_from = worst arrival over
// transitions (5) and d = max arc delay (7) INDEPENDENTLY, reconstructing
// a_from + d = 12.

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

  // Old collapse: 5 + 7 - 9 = +3, a spurious violation that pushed λ up as
  // phantom "dual ascent" (the λ-creep source).
  EXPECT_GT(5.0f + 7.0f - a_to, 0.0f);

  // Fixed: the realized pair (2, 7) gives exactly 0 — tight, not violating.
  const ArcTransitionRead crit = pickCriticalArcTransition(reads);
  EXPECT_NEAR(crit.a_from + crit.d - a_to, 0.0f, kTol);
}

TEST(ConsistentArcRead, MetArcReadsNegativeNotPhantomTight)
{
  // A non-critical (met) edge: a different, more critical in-edge sets the
  // to-vertex arrival to 12. The consistent read gives 9 - 12 = -3 (met),
  // strictly more slack than the old collapse's 12 - 12 = 0 (phantom "tight").
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

}  // namespace
}  // namespace rsz
