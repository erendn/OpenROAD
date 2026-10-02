// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for computeTimingWeight, which sets the timing weight tw in the
// LR cost `power + tw·Σλ·d`, and for the Livramento α schedule. The STA walk
// that fills TimingScaleInput is covered by the global_sizing* integration
// tests.
//
// The main property is how each option reacts to a uniform rescale of λ:
//   - auto_median: t_med = median_g(Σλ·d) is proportional to λ, so tw ∝ 1/λ
//     and tw·λ·d is exactly invariant to the rescale. The magnitude of λ
//     never reaches the candidate costs. For example, the global power ratio
//     of Mangiras and Dimitrakopoulos (Technologies 2021, Eq. 6), which
//     scales all multipliers uniformly, has no effect.
//   - unit: d_med = median_g(Σ d) contains no λ, so tw does not change and
//     tw·λ·d scales by the rescale factor. The magnitude of λ is preserved.
//
// The Livramento α schedule (Livramento et al., DATE 2013, Alg. 1 line 9) is
// the one part of the timing weight that changes every iteration; its
// direction and its floors are tested here.

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "lr/CostTerms.hh"
#include "lr/FlowProjection.hh"
#include "lr/LambdaSeeder.hh"
#include "lr/TimingScale.hh"
#include "rsz/GlobalSizingConfig.hh"

namespace rsz {
namespace {

using TimingScale = GlobalSizingConfig::TimingScale;
using MuPolicy = GlobalSizingConfig::MuPolicy;
using PowerObjective = GlobalSizingConfig::PowerObjective;

constexpr float kTol = 1e-4f;
constexpr float kFloor = 1e-12f;

// A hand-built three-gate design. It is asymmetric (different leakages and
// different per-gate λ and d) so each median is a real choice rather than a
// tie, and a bug that turned the anchor into a constant would show.
TimingScaleInput makeInput()
{
  TimingScaleInput in;
  // leakage, Σλ·d, Σd
  const float leak[] = {1.0f, 2.0f, 4.0f};
  const float lam[] = {3.0f, 5.0f, 11.0f};  // per-gate Σλ over its out-pins
  const float del[] = {0.5f, 2.0f, 8.0f};   // per-gate Σd
  for (int i = 0; i < 3; ++i) {
    TimingScaleInput::Gate g;
    g.leakage = leak[i];
    g.delay = del[i];
    g.has_delay = true;
    g.lambda_delay = lam[i] * del[i];
    g.has_pressure = true;
    in.gates.push_back(g);
  }
  return in;
}

// Uniformly rescale λ by c. Only the λ-carrying field changes; Σd depends on
// the library and the load, not on the multipliers.
TimingScaleInput rescaleLambda(const TimingScaleInput& in, const float c)
{
  TimingScaleInput out = in;
  for (TimingScaleInput::Gate& g : out.gates) {
    g.lambda_delay *= c;
  }
  return out;
}

TEST(TimingScaleTest, UnitIsTheMedianRatioOfLeakageToDelay)
{
  const TimingScaleInput in = makeInput();
  const TimingScaleWeight w = computeTimingWeight(in, TimingScale::kUnit, 1.0f);

  EXPECT_FALSE(w.degenerate);
  // Medians of {1,2,4} and {0.5,2,8}.
  EXPECT_NEAR(w.l_med, 2.0f, kTol);
  EXPECT_NEAR(w.anchor_med, 2.0f, kTol);
  EXPECT_NEAR(w.tw, 1.0f, kTol);
}

TEST(TimingScaleTest, AutoMedianIsTheBiasedMedianRatioOfLeakageToLambdaDelay)
{
  const TimingScaleInput in = makeInput();
  const TimingScaleWeight w
      = computeTimingWeight(in, TimingScale::kAutoMedian, 64.0f);

  EXPECT_FALSE(w.degenerate);
  EXPECT_NEAR(w.l_med, 2.0f, kTol);
  // Medians of {1.5, 10, 88}.
  EXPECT_NEAR(w.anchor_med, 10.0f, kTol);
  EXPECT_NEAR(w.tw, 64.0f * 2.0f / 10.0f, kTol);
}

// Under unit, tw does not change when λ is rescaled, so the timing term
// tw·Σλ·d scales with λ and the magnitude of the initial λ reaches the
// candidate cost. Together with mu_policy=endpoint_lambda, which preserves
// that magnitude through the flow projection, this is what lets
// lambda_init_value affect the result.
TEST(TimingScaleTest, UnitIsInvariantUnderUniformLambdaRescale)
{
  const TimingScaleInput base = makeInput();
  const TimingScaleWeight w1
      = computeTimingWeight(base, TimingScale::kUnit, 1.0f);

  // c = 12 is Flach's initial λ relative to Chen's and Sharma's (1); 0.01
  // checks a shrink.
  for (const float c : {12.0f, 100.0f, 0.01f}) {
    const TimingScaleWeight wc
        = computeTimingWeight(rescaleLambda(base, c), TimingScale::kUnit, 1.0f);
    EXPECT_NEAR(wc.tw, w1.tw, kTol) << "tw moved under a uniform λ rescale by "
                                    << c << "; unit must be λ-free";
    // ...and therefore the timing term scales by exactly c.
    EXPECT_NEAR(wc.tw * c, w1.tw * c, kTol);
  }
}

// Under auto_median, tw absorbs a uniform λ rescale exactly, so tw·λ·d does
// not change and the magnitude of λ has no effect on the cost. This is the
// behavior `unit` exists to avoid.
TEST(TimingScaleTest, AutoMedianCancelsAUniformLambdaRescaleExactly)
{
  const TimingScaleInput base = makeInput();
  const TimingScaleWeight w1
      = computeTimingWeight(base, TimingScale::kAutoMedian, 64.0f);

  for (const float c : {12.0f, 100.0f, 0.01f}) {
    const TimingScaleWeight wc = computeTimingWeight(
        rescaleLambda(base, c), TimingScale::kAutoMedian, 64.0f);
    // tw ∝ 1/c ...
    EXPECT_NEAR(wc.tw, w1.tw / c, kTol * w1.tw);
    // ... so the product tw·(c·λ)·d is the same as at c = 1: the magnitude of
    // the initial λ is divided back out before any candidate is evaluated.
    EXPECT_NEAR(wc.tw * c, w1.tw, kTol * w1.tw);
  }
}

// livramento_alpha uses auto_median's λ-invariant base, not unit's. The
// livramento preset's seed produces λ values that are arc delays in seconds,
// so their magnitude is an artifact of the time unit and must not reach the
// cost; α scales the invariant base instead.
TEST(TimingScaleTest, LivramentoAlphaSharesAutoMediansAnchor)
{
  const TimingScaleInput in = makeInput();
  const TimingScaleWeight autom
      = computeTimingWeight(in, TimingScale::kAutoMedian, 1.0f);
  const TimingScaleWeight liv
      = computeTimingWeight(in, TimingScale::kLivramentoAlpha, 1.0f);
  EXPECT_NEAR(liv.tw, autom.tw, kTol);
  EXPECT_NE(liv.tw, computeTimingWeight(in, TimingScale::kUnit, 1.0f).tw);
}

// ...and it therefore inherits auto_median's λ-invariance.
TEST(TimingScaleTest, LivramentoAlphaBaseCancelsAUniformLambdaRescale)
{
  const TimingScaleInput base = makeInput();
  const TimingScaleWeight w1
      = computeTimingWeight(base, TimingScale::kLivramentoAlpha, 1.0f);
  const TimingScaleWeight w12 = computeTimingWeight(
      rescaleLambda(base, 12.0f), TimingScale::kLivramentoAlpha, 1.0f);
  EXPECT_NEAR(w12.tw * 12.0f, w1.tw, kTol * w1.tw);
}

// timing_bias only applies to auto_median and livramento_alpha (the default
// configuration uses 64). Under unit the timing/leakage balance comes from λ
// itself, so timing_bias must have no effect; the paper presets that use unit
// set it to 1.0 to make that explicit.
TEST(TimingScaleTest, TimingBiasIsInertUnderUnit)
{
  const TimingScaleInput in = makeInput();
  const TimingScaleWeight a = computeTimingWeight(in, TimingScale::kUnit, 1.0f);
  const TimingScaleWeight b
      = computeTimingWeight(in, TimingScale::kUnit, 64.0f);
  EXPECT_NEAR(a.tw, b.tw, kTol);
}

TEST(TimingScaleTest, DegenerateInputsFallBackToOne)
{
  // No gates at all.
  TimingScaleInput empty;
  EXPECT_TRUE(computeTimingWeight(empty, TimingScale::kUnit, 1.0f).degenerate);
  EXPECT_NEAR(
      computeTimingWeight(empty, TimingScale::kUnit, 1.0f).tw, 1.0f, kTol);

  // Leakage present, but no gate has the option's anchor. The two options
  // differ on which input is degenerate: a gate whose λ is at the floor still
  // has a delay.
  TimingScaleInput no_pressure = makeInput();
  for (TimingScaleInput::Gate& g : no_pressure.gates) {
    g.has_pressure = false;
  }
  EXPECT_TRUE(computeTimingWeight(no_pressure, TimingScale::kAutoMedian, 64.0f)
                  .degenerate);
  EXPECT_FALSE(
      computeTimingWeight(no_pressure, TimingScale::kUnit, 1.0f).degenerate);

  // A non-positive median is degenerate too (a zero-leakage library).
  TimingScaleInput zero_leak = makeInput();
  for (TimingScaleInput::Gate& g : zero_leak.gates) {
    g.leakage = 0.0f;
  }
  EXPECT_TRUE(
      computeTimingWeight(zero_leak, TimingScale::kUnit, 1.0f).degenerate);
}

// Under the total-power objective each gate's power sample is the objective
// power of its current cell, and the rest of the formula is unchanged. The
// powers {10, 3, 50} have the median 10, where the leakages {1, 2, 4} have
// the median 2; the anchors are makeInput's t_med = 10 and d_med = 2.
TEST(TimingScaleTest, TotalObjectiveSamplesObjectivePower)
{
  TimingScaleInput in = makeInput();
  const float power[] = {10.0f, 3.0f, 50.0f};
  for (int i = 0; i < 3; ++i) {
    in.gates[i].power = power[i];
  }

  const TimingScaleWeight autom = computeTimingWeight(
      in, TimingScale::kAutoMedian, 64.0f, PowerObjective::kTotal);
  EXPECT_FALSE(autom.degenerate);
  EXPECT_NEAR(autom.l_med, 10.0f, kTol);
  EXPECT_NEAR(autom.anchor_med, 10.0f, kTol);
  EXPECT_NEAR(autom.tw, 64.0f * 10.0f / 10.0f, kTol);

  const TimingScaleWeight unit = computeTimingWeight(
      in, TimingScale::kUnit, 1.0f, PowerObjective::kTotal);
  EXPECT_NEAR(unit.l_med, 10.0f, kTol);
  EXPECT_NEAR(unit.tw, 10.0f / 2.0f, kTol);

  // The leakage objective ignores the power samples.
  const TimingScaleWeight leak = computeTimingWeight(
      in, TimingScale::kAutoMedian, 64.0f, PowerObjective::kLeakage);
  EXPECT_NEAR(leak.l_med, 2.0f, kTol);
  EXPECT_NEAR(leak.tw, 64.0f * 2.0f / 10.0f, kTol);

  // Zero objective power is degenerate even where the leakage is not.
  for (TimingScaleInput::Gate& g : in.gates) {
    g.power = 0.0f;
  }
  EXPECT_TRUE(computeTimingWeight(
                  in, TimingScale::kAutoMedian, 64.0f, PowerObjective::kTotal)
                  .degenerate);
}

// Livramento et al., DATE 2013, Alg. 1 line 9: α ← α·(A_o / max_j a_j),
// computed as α·(T / (T - WNS)).
TEST(TimingScaleTest, LivramentoAlphaFixedPointIsZeroWns)
{
  // max_j a_j == A_o is the controller's fixed point: α must not move.
  EXPECT_NEAR(rescheduleLivramentoAlpha(0.7f, 1.0f, 0.0f), 0.7f, kTol);
}

TEST(TimingScaleTest, LivramentoAlphaShrinksOnViolationAndGrowsOnSlack)
{
  // Violating (WNS < 0): arrivals overshoot the target, so α shrinks and
  // tw = base/α rises, giving more timing pressure. This is the behavior
  // studied in the paper's Figs. 1-2.
  EXPECT_LT(rescheduleLivramentoAlpha(1.0f, 1.0f, -0.25f), 1.0f);
  EXPECT_NEAR(
      rescheduleLivramentoAlpha(1.0f, 1.0f, -0.25f), 1.0f / 1.25f, kTol);

  // Slack to spare (WNS > 0): α grows, tw falls, leakage is recovered.
  EXPECT_GT(rescheduleLivramentoAlpha(1.0f, 1.0f, 0.5f), 1.0f);
  EXPECT_NEAR(rescheduleLivramentoAlpha(1.0f, 1.0f, 0.5f), 1.0f / 0.5f, kTol);

  // The update is multiplicative, so steps compound and α₀ never washes out
  // (which is why α₀ and ‖λ₀‖ are one degree of freedom, not two).
  const float once = rescheduleLivramentoAlpha(1.0f, 1.0f, -0.25f);
  EXPECT_NEAR(rescheduleLivramentoAlpha(once, 1.0f, -0.25f),
              1.0f / (1.25f * 1.25f),
              kTol);
}

// The paper states no clamp, but max_j a_j → 0 would make α explode, so the
// denominator is floored here.
TEST(TimingScaleTest, LivramentoAlphaFloorGuardBoundsTheRatio)
{
  // WNS == T means the worst arrival is 0, and an unguarded ratio would be
  // T/0. The floor caps one step's growth at 1/kLivramentoArrivalFloorFrac.
  const float guarded = rescheduleLivramentoAlpha(1.0f, 1.0f, 1.0f);
  EXPECT_TRUE(std::isfinite(guarded));
  EXPECT_NEAR(guarded, 1.0f / kLivramentoArrivalFloorFrac, kTol);

  // Past the floor it saturates rather than running away.
  EXPECT_NEAR(rescheduleLivramentoAlpha(1.0f, 1.0f, 2.0f),
              1.0f / kLivramentoArrivalFloorFrac,
              kTol);
}

// The schedule compounds, so on a design that never meets timing α shrinks
// every iteration and would underflow to 0 over a long run. tw = base/α would
// then overflow to +inf and make every candidate cost inf or NaN. The
// denominator floor does not prevent this (it bounds one step, not the running
// product); only the floor on α does. This can happen within the
// livramento_partial preset's 60-iteration limit.
TEST(TimingScaleTest, LivramentoAlphaFloorSurvivesALongViolatingRun)
{
  // A steadily violating design: WNS = -0.6T shrinks α by 1/1.6 every step.
  float alpha = 1.0f;
  for (int i = 0; i < 500; ++i) {
    alpha = rescheduleLivramentoAlpha(alpha, 1.0f, -0.6f);
  }
  EXPECT_GE(alpha, kLivramentoAlphaFloor);
  EXPECT_GT(alpha, 0.0f);
  // Callers rely on base/α staying finite for any base the sizer produces (tw
  // can reach about 1e13).
  EXPECT_TRUE(std::isfinite(1.0e13f / alpha));
}

// The schedule reports whether the floor on α ever clamped, so the driver can
// log it at loop exit. This cannot be inferred from the final α: once the
// floor binds, α sits at the floor, and a run that merely converged to a low α
// looks the same as one that ran away.
TEST(TimingScaleTest, LivramentoAlphaFloorReportsWhenItBinds)
{
  bool bound = false;
  // A step nowhere near the floor does not set the flag.
  rescheduleLivramentoAlpha(1.0f, 1.0f, -0.6f, &bound);
  EXPECT_FALSE(bound);

  // A steadily violating run drives α into the floor, and the flag records it.
  float alpha = 1.0f;
  for (int i = 0; i < 500; ++i) {
    alpha = rescheduleLivramentoAlpha(alpha, 1.0f, -0.6f, &bound);
  }
  EXPECT_TRUE(bound);
  EXPECT_NEAR(alpha, kLivramentoAlphaFloor, 0.0f);

  // The flag is only ever set, never cleared, so a later step that moves α off
  // the floor does not hide that the floor was hit.
  alpha = rescheduleLivramentoAlpha(alpha, 1.0f, 0.5f, &bound);
  EXPECT_GT(alpha, kLivramentoAlphaFloor);
  EXPECT_TRUE(bound);

  // The output parameter is optional.
  EXPECT_NEAR(rescheduleLivramentoAlpha(1.0f, 1.0f, 0.0f), 1.0f, kTol);
}

TEST(TimingScaleTest, LivramentoAlphaNoOpsWithoutAClock)
{
  // T <= 0: no SDC clock, so there is no target. The other T-normalized
  // updates also do nothing in this case.
  EXPECT_NEAR(rescheduleLivramentoAlpha(0.3f, 0.0f, -0.5f), 0.3f, kTol);
  EXPECT_NEAR(rescheduleLivramentoAlpha(0.3f, -1.0f, -0.5f), 0.3f, kTol);
}

// === Mangiras Eq. 6, end to end =============================================
//
// The mangiras_partial preset relies on the design-wide power ratio
// (ΣP / Σ minP)^K of Mangiras and Dimitrakopoulos (Technologies 2021, Eq. 6)
// reaching the candidate costs as real λ pressure. Under
// timing_scale=auto_median it cannot, because tw ∝ 1/λ divides it back out;
// mu_policy=endpoint_lambda together with timing_scale=unit lets it through.
// These tests chain the STA-free functions:
//
//   seed (Eq. 6) → projection (endpoint_lambda) → tw (unit) → Σ_i λ_i·d_i.
//
// They use two leakage states with identical timing that differ only in the
// total leakage ΣP, and check that a candidate sees the Eq. 6 ratio between
// them under `unit` and no difference under `auto_median`.

// The fixture graph, in projection visit order (descending level):
//   a --e0--> c --e2--> d(endpoint 0)
//   b --e1--/
// d anchors the endpoint boundary; c redistributes its in-arcs to it.
ProjectionTopology eq6Fixture()
{
  ProjectionTopology topo;
  topo.in_edges = {2, 0, 1};   // d's in-arc, then c's
  topo.out_edges = {2, 0, 1};  // c's out-arc, then a's, then b's

  ProjectionTopology::Vertex d;
  d.in_begin = 0;
  d.in_end = 1;
  d.endpoint = 0;

  ProjectionTopology::Vertex c;
  c.in_begin = 1;
  c.in_end = 3;
  c.out_begin = 0;
  c.out_end = 1;

  ProjectionTopology::Vertex a;
  a.out_begin = 1;
  a.out_end = 2;

  ProjectionTopology::Vertex b;
  b.out_begin = 2;
  b.out_end = 3;

  topo.vertices = {d, c, a, b};
  return topo;
}

// Fixed arc delays for the three edges (e0, e1, e2). Timing is identical in
// the two leakage states, so d_med (unit) and these delays never change.
constexpr float kEq6ArcDelay[3] = {0.3f, 0.2f, 0.5f};
constexpr float kEq6MinLeak
    = 1.0f;  // Σ minP, the virtual minimum total leakage
constexpr float kEq6MedLeak = 2.0f;  // l_med, held fixed (see below)
constexpr float kEq6K = 2.0f;        // Mangiras' K (Eq. 6 exponent)

// Runs one total-leakage state through the whole chain and returns the
// effective λ pressure a candidate sees, tw · Σ_i λ_i·d_i.
float eq6EffectivePressure(const float total_leak, const TimingScale scale)
{
  // (1) SEED: Eq. 6 sets the endpoint arc to (a_k/r_k · ΣP/ΣminP)^K. The
  //     timing ratio a_k/r_k = 0.5/1.0 is fixed, so the two states differ
  //     only by the global power ratio (total_leak/kEq6MinLeak)^K.
  const float endpoint_seed
      = mangirasEndpointArcLambda(0.5f, 1.0f, total_leak, kEq6MinLeak, kEq6K);
  // Internal arcs seeded nonuniformly; the projection anchors them to the
  // endpoint boundary, so their magnitude is set by the Eq. 6 seed.
  std::vector<float> lambda
      = {endpoint_seed * 0.4f, endpoint_seed * 0.9f, endpoint_seed};
  std::vector<float> mu = {0.0f};

  // (2) PROJECT under endpoint_lambda. The projection is positively
  //     homogeneous, so the whole projected field scales with the Eq. 6 seed
  //     (e0 + e1 = e2 = seed).
  projectFlowBalance(eq6Fixture(),
                     MuPolicy::kEndpointLambda,
                     /*derive_endpoint_mu=*/true,
                     kFloor,
                     mu,
                     lambda);

  // (3) SCALE: compute tw. Σλ·d is the timing pressure (and auto_median's
  //     t_med). l_med and d_med are the same in both states, which isolates
  //     the Eq. 6 global ratio (in a real design the median gate is not
  //     affected by the few gates whose leakage changes ΣP).
  const std::vector<ArcLambdaDelay> arcs = {{lambda[0], kEq6ArcDelay[0]},
                                            {lambda[1], kEq6ArcDelay[1]},
                                            {lambda[2], kEq6ArcDelay[2]}};
  const float lambda_delay = perArcTimingCost(arcs);  // Σλ·d
  TimingScaleInput in;
  TimingScaleInput::Gate g;
  g.leakage = kEq6MedLeak;
  g.lambda_delay = lambda_delay;  // auto_median's t_med anchor (∝ λ)
  g.has_pressure = true;
  g.delay
      = kEq6ArcDelay[0] + kEq6ArcDelay[1] + kEq6ArcDelay[2];  // unit's d_med
  g.has_delay = true;
  in.gates.push_back(g);
  const float tw = computeTimingWeight(in, scale, /*timing_bias=*/1.0f).tw;

  // (4) COST: the weight on a candidate's own-gate timing term.
  return tw * lambda_delay;
}

constexpr float kEq6LeakHi = 6.0f;  // ΣP for state A
constexpr float kEq6LeakLo = 2.0f;  // ΣP for state B (identical timing)

// Under `unit`, the leakage state reaches the candidate cost, and the ratio
// between the two states is exactly the Eq. 6 global power ratio (P_A/P_B)^K,
// not 1.
TEST(TimingScaleTest, Eq6GlobalPowerRatioSurvivesToScoringUnderUnit)
{
  const float pressure_hi
      = eq6EffectivePressure(kEq6LeakHi, TimingScale::kUnit);
  const float pressure_lo
      = eq6EffectivePressure(kEq6LeakLo, TimingScale::kUnit);

  EXPECT_GT(pressure_hi, 0.0f);
  EXPECT_GT(pressure_hi, pressure_lo);  // the leakage state is visible at all
  // ...by exactly (6/2)^2 = 9: the Eq. 6 ratio passes through seed, projection
  // and timing weight unchanged.
  const float expected = std::pow(kEq6LeakHi / kEq6LeakLo, kEq6K);
  EXPECT_NEAR(pressure_hi / pressure_lo, expected, expected * kTol);
}

// Negative control: under auto_median, tw ∝ 1/λ divides the Eq. 6 ratio back
// out, so both leakage states give the candidate the same pressure. This is
// why mangiras_partial needs `unit`.
TEST(TimingScaleTest, Eq6GlobalPowerRatioIsCancelledUnderAutoMedian)
{
  const float pressure_hi
      = eq6EffectivePressure(kEq6LeakHi, TimingScale::kAutoMedian);
  const float pressure_lo
      = eq6EffectivePressure(kEq6LeakLo, TimingScale::kAutoMedian);

  // Identical to within float noise: tw·Σλ·d = timing_bias·l_med regardless of
  // the seed magnitude, so the two leakage states are indistinguishable here.
  EXPECT_NEAR(pressure_hi, pressure_lo, pressure_hi * kTol);
}

}  // namespace
}  // namespace rsz
