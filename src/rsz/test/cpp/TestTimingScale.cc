// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the MB balance-core's A1 axis: computeTimingWeight, the pure
// core of the objective scale (the STA walk that fills its TimingScaleInput is
// exercised by the global_sizing* integration tests).
//
// The headline assertion is the λ-rescale property, which is the whole reason
// the axis exists. It is pinned in BOTH directions, because the contrast is the
// finding:
//   - auto_median: t_med = median_g(Σλ·d) is proportional to λ, so tw ∝ 1/λ and
//     the product tw·λ·d is EXACTLY invariant to a uniform rescale. λ's
//     magnitude cannot reach candidate scoring. This is mechanism 2 of the
//     plan's "λ-magnitude finding", and it is what cancelled Mangiras Eq. 6's
//     global power ratio before the first candidate was ever scored.
//   - unit: d_med = median_g(Σ d) contains no λ, so tw is untouched by the
//     rescale and the product scales by c. λ's magnitude survives - which is
//     what makes the Eq. 6 end-to-end test possible.
//
// Plus the Livramento α schedule (Alg. 1 L9), whose direction and floor guard
// are pinned here since it is the axis's one per-iteration component.

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

constexpr float kTol = 1e-4f;
constexpr float kFloor = 1e-12f;

// A hand-built three-gate design. Deliberately asymmetric (different leakages,
// different per-gate λ and d) so a median is a real choice rather than a tie,
// and so a bug that collapsed the anchor to a constant would show.
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

// Uniformly rescale λ by c. Only the λ-carrying field moves; Σd is a property
// of the library and the load, not of the multipliers.
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

// THE headline property (direction 1): unit's tw does not move when λ does, so
// the timing term tw·Σλ·d scales with λ and the seed magnitude reaches the
// candidate cost. mu_policy=endpoint_lambda buys the projection; this buys
// candidate scoring; together they are what makes lambda_init_value live.
TEST(TimingScaleTest, UnitIsInvariantUnderUniformLambdaRescale)
{
  const TimingScaleInput base = makeInput();
  const TimingScaleWeight w1
      = computeTimingWeight(base, TimingScale::kUnit, 1.0f);

  // Flach's 12 against Chen/Sharma's 1, and a shrink for good measure.
  for (const float c : {12.0f, 100.0f, 0.01f}) {
    const TimingScaleWeight wc
        = computeTimingWeight(rescaleLambda(base, c), TimingScale::kUnit, 1.0f);
    EXPECT_NEAR(wc.tw, w1.tw, kTol) << "tw moved under a uniform λ rescale by "
                                    << c << "; unit must be λ-free";
    // ...and therefore the priced timing term scales by exactly c.
    EXPECT_NEAR(wc.tw * c, w1.tw * c, kTol);
  }
}

// THE headline property (direction 2): the contrast. auto_median's tw absorbs
// the rescale exactly, so tw·λ·d is constant and λ's magnitude is dead. This is
// the property `unit` exists to remove - pinning it makes the finding
// executable rather than an analytic claim in a doc.
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
    // ... so the product tw·(c·λ)·d is the same number it was at c = 1: the
    // seed magnitude is divided straight back out before any candidate is
    // scored.
    EXPECT_NEAR(wc.tw * c, w1.tw, kTol * w1.tw);
  }
}

// livramento_alpha's base is auto_median's λ-INVARIANT anchor, not unit's. The
// preset runs a seed whose λ is an arc delay in seconds, so a λ-preserving base
// would preserve a unit artifact; α scales the invariant base instead. Pinning
// this stops a later "tidy-up" from folding it in with unit.
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

// ...and it therefore inherits auto_median's λ-invariance, which is the
// property it is chosen for.
TEST(TimingScaleTest, LivramentoAlphaBaseCancelsAUniformLambdaRescale)
{
  const TimingScaleInput base = makeInput();
  const TimingScaleWeight w1
      = computeTimingWeight(base, TimingScale::kLivramentoAlpha, 1.0f);
  const TimingScaleWeight w12 = computeTimingWeight(
      rescaleLambda(base, 12.0f), TimingScale::kLivramentoAlpha, 1.0f);
  EXPECT_NEAR(w12.tw * 12.0f, w1.tw, kTol * w1.tw);
}

// timing_bias is auto_median's private balance knob (rsz_baseline's 64). Under
// the λ-free options the balance is carried by λ itself, so the knob must be
// inert - which is why the paper presets pin it to 1.0 to say so out loud.
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

  // Leakage present, but no gate carries the option's anchor. Note the two
  // options disagree about which input is degenerate, which is the point: a
  // gate whose λ sits at the floor still has a delay.
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

// Livramento Alg. 1 L9: α ← α·(A_o / max_j a_j), read as α·(T / (T - WNS)).
TEST(TimingScaleTest, LivramentoAlphaFixedPointIsZeroWns)
{
  // max_j a_j == A_o is the controller's fixed point: α must not move.
  EXPECT_NEAR(rescheduleLivramentoAlpha(0.7f, 1.0f, 0.0f), 0.7f, kTol);
}

TEST(TimingScaleTest, LivramentoAlphaShrinksOnViolationAndGrowsOnSlack)
{
  // Violating (WNS < 0): arrivals overshoot the target, so α shrinks and
  // tw = base/α rises - more timing pressure. This is the direction the
  // paper's Figs. 1-2 ablate.
  EXPECT_LT(rescheduleLivramentoAlpha(1.0f, 1.0f, -0.25f), 1.0f);
  EXPECT_NEAR(
      rescheduleLivramentoAlpha(1.0f, 1.0f, -0.25f), 1.0f / 1.25f, kTol);

  // Slack to spare (WNS > 0): α grows, tw falls, leakage is recovered.
  EXPECT_GT(rescheduleLivramentoAlpha(1.0f, 1.0f, 0.5f), 1.0f);
  EXPECT_NEAR(rescheduleLivramentoAlpha(1.0f, 1.0f, 0.5f), 1.0f / 0.5f, kTol);

  // Multiplicative, so it compounds - α₀ never washes out (which is why α₀ and
  // ‖λ₀‖ are one degree of freedom, not two).
  const float once = rescheduleLivramentoAlpha(1.0f, 1.0f, -0.25f);
  EXPECT_NEAR(rescheduleLivramentoAlpha(once, 1.0f, -0.25f),
              1.0f / (1.25f * 1.25f),
              kTol);
}

// DEVIATION (ours): L9 states no clamp, and max_j a_j → 0 makes α explode.
TEST(TimingScaleTest, LivramentoAlphaFloorGuardBoundsTheRatio)
{
  // WNS == T means the worst arrival has collapsed to 0: an unguarded ratio is
  // T/0. The floor caps one step's growth at 1/kLivramentoArrivalFloorFrac.
  const float guarded = rescheduleLivramentoAlpha(1.0f, 1.0f, 1.0f);
  EXPECT_TRUE(std::isfinite(guarded));
  EXPECT_NEAR(guarded, 1.0f / kLivramentoArrivalFloorFrac, kTol);

  // Past the floor it saturates rather than running away.
  EXPECT_NEAR(rescheduleLivramentoAlpha(1.0f, 1.0f, 2.0f),
              1.0f / kLivramentoArrivalFloorFrac,
              kTol);
}

// The schedule compounds, so on a design that never closes timing α shrinks
// every iteration and would underflow to 0 over a long run - and tw = base/α
// then overflows to +inf, poisoning every candidate cost. The denominator floor
// does not prevent this (it bounds one step's ratio, not the accumulator); only
// the α floor does. Reachable at livramento_partial's own 60-iteration cap.
TEST(TimingScaleTest, LivramentoAlphaFloorSurvivesALongViolatingRun)
{
  // A steadily violating design: WNS = -0.6T shrinks α by 1/1.6 every step.
  float alpha = 1.0f;
  for (int i = 0; i < 500; ++i) {
    alpha = rescheduleLivramentoAlpha(alpha, 1.0f, -0.6f);
  }
  EXPECT_GE(alpha, kLivramentoAlphaFloor);
  EXPECT_GT(alpha, 0.0f);
  // The postcondition callers rely on: base/α stays finite for any base this
  // engine produces (tw has been observed as high as ~1e13).
  EXPECT_TRUE(std::isfinite(1.0e13f / alpha));
}

// The A-axis diagnostic (iteration-2 plan §2.2-5): the driver reports at loop
// exit whether the accumulator floor ever clamped. It has to be reported by the
// schedule rather than inferred from the terminal α, because the clamp is
// exactly what hides itself - once it binds, α sits AT the floor and a run that
// merely converged low is indistinguishable from one that ran away.
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

  // The flag is a run-level latch: it is SET, never cleared, so a later step
  // that grows α off the floor does not erase the fact that the guard bound.
  alpha = rescheduleLivramentoAlpha(alpha, 1.0f, 0.5f, &bound);
  EXPECT_GT(alpha, kLivramentoAlphaFloor);
  EXPECT_TRUE(bound);

  // And the out-param is optional - every other caller passes nothing.
  EXPECT_NEAR(rescheduleLivramentoAlpha(1.0f, 1.0f, 0.0f), 1.0f, kTol);
}

TEST(TimingScaleTest, LivramentoAlphaNoOpsWithoutAClock)
{
  // T <= 0: no SDC clock, so no target to servo to. Matches how the other
  // T-normalized strategies no-op.
  EXPECT_NEAR(rescheduleLivramentoAlpha(0.3f, 0.0f, -0.5f), 0.3f, kTol);
  EXPECT_NEAR(rescheduleLivramentoAlpha(0.3f, -1.0f, -0.5f), 0.3f, kTol);
}

// === Eq. 6 end-to-end acceptance (the MB milestone's named criterion) ========
//
// The mangiras_partial bundle's headline is that Mangiras Eq. 6's design-global
// power ratio (ΣP / Σ minP)^K reaches candidate scoring as real λ pressure. The
// hardening pass proved that ratio was cancelled before any candidate was
// scored (auto_median's tw ∝ 1/λ divided it straight back out); the item-5 flip
// to mu_policy=endpoint_lambda + timing_scale=unit is what un-cancels it. This
// test is that milestone's own acceptance proof, run link-by-link on the pure
// cores:
//
//   seed (Eq. 6) → projection (endpoint_lambda) → tw (unit) → Σ_i λ_i·d_i.
//
// It drives TWO leakage states that are identical in timing and differ only in
// the design-global leakage ΣP, then asserts the Eq. 6 ratio between them is
// what a candidate sees under `unit` — and, as the negative control, is dead
// under `auto_median` (mangiras audit §3(e), §1's survival-chain verdict).

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

// Fixed arc delays for the three edges (e0, e1, e2). Timing side is identical
// across the two leakage states, so d_med (unit) and the delays here never
// move.
constexpr float kEq6ArcDelay[3] = {0.3f, 0.2f, 0.5f};
constexpr float kEq6MinLeak
    = 1.0f;  // Σ minP, the virtual minimum total leakage
constexpr float kEq6MedLeak = 2.0f;  // l_med, held fixed (see below)
constexpr float kEq6K = 2.0f;        // Mangiras' K (Eq. 6 exponent)

// One design-global leakage state through the whole chain; returns the
// effective λ pressure a candidate sees, tw · Σ_i λ_i·d_i.
float eq6EffectivePressure(const float total_leak, const TimingScale scale)
{
  // (1) SEED — Eq. 6 writes the endpoint boundary arc from a_k/r_k · ΣP/ΣminP,
  //     raised to K. The met timing ratio a_k/r_k = 0.5/1.0 is fixed, so the
  //     two states differ ONLY by the global power ratio
  //     (total_leak/kEq6MinLeak)^K.
  const float endpoint_seed
      = mangirasEndpointArcLambda(0.5f, 1.0f, total_leak, kEq6MinLeak, kEq6K);
  // Internal arcs seeded nonuniformly; the projection anchors them to the
  // endpoint boundary, so their magnitude is set by the Eq. 6 seed.
  std::vector<float> lambda
      = {endpoint_seed * 0.4f, endpoint_seed * 0.9f, endpoint_seed};
  std::vector<float> mu = {0.0f};

  // (2) PROJECT under endpoint_lambda — positively homogeneous, so the whole
  //     post-projection field scales with the Eq. 6 seed (e0 + e1 = e2 = seed).
  projectFlowBalance(eq6Fixture(),
                     MuPolicy::kEndpointLambda,
                     /*derive_endpoint_mu=*/true,
                     kFloor,
                     mu,
                     lambda);

  // (3) SCALE — the frozen anchor. Σλ·d is the priced timing pressure (also
  //     auto_median's t_med); l_med and d_med are held fixed across the two
  //     states, isolating the Eq. 6 GLOBAL ratio (a real design's median gate
  //     is robust to the few gates whose leakage moves ΣP).
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

  // (4) SCORE — what a candidate's own-gate timing term is weighted by.
  return tw * lambda_delay;
}

constexpr float kEq6LeakHi = 6.0f;  // ΣP for state A
constexpr float kEq6LeakLo = 2.0f;  // ΣP for state B (identical timing)

// THE Eq. 6 acceptance: under `unit`, the leakage state reaches the candidate
// cost, and the ratio between the two states is exactly the Eq. 6 global power
// ratio (P_A/P_B)^K — not 1. This is §1's survival-chain verdict as a test.
TEST(TimingScaleTest, Eq6GlobalPowerRatioSurvivesToScoringUnderUnit)
{
  const float pressure_hi
      = eq6EffectivePressure(kEq6LeakHi, TimingScale::kUnit);
  const float pressure_lo
      = eq6EffectivePressure(kEq6LeakLo, TimingScale::kUnit);

  EXPECT_GT(pressure_hi, 0.0f);
  EXPECT_GT(pressure_hi, pressure_lo);  // the leakage state is visible at all
  // ...and by exactly (6/2)^2 = 9: the Eq. 6 ratio survives seed → projection →
  // unit scoring undistorted.
  const float expected = std::pow(kEq6LeakHi / kEq6LeakLo, kEq6K);
  EXPECT_NEAR(pressure_hi / pressure_lo, expected, expected * kTol);
}

// The negative control: the exact quantity the hardening pass proved dead.
// auto_median's tw ∝ 1/λ divides the Eq. 6 ratio straight back out, so both
// leakage states hand the candidate the identical pressure. Pinning this makes
// "why unit is required" executable rather than an analytic claim.
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
