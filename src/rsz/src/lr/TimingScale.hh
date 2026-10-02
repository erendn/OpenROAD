// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <vector>

#include "LrState.hh"
#include "rsz/GlobalSizingConfig.hh"

namespace rsz {

class Resizer;
class LRSubproblem;

// Timing weight: how the factor tw in the sizing cost
// power + tw * sum(lambda * d) is chosen.
//
// As with ProjectionTopology, the arithmetic is a function over a plain
// structure so it can be unit-tested without STA; the STA walk only fills the
// structure in.

// STA-free view of the design: one entry per editable gate, built by
// collectTimingScaleInput() and consumed by computeTimingWeight().
struct TimingScaleInput
{
  struct Gate
  {
    // Leakage of the gate's current cell (or its area-based substitute, see
    // LRSubproblem::leakageOrArea). Every editable gate with a liberty cell has
    // one; l_med is their median under the leakage objective.
    float leakage = 0.0f;
    // Objective power of the gate's current cell (see ObjectivePower), which
    // is its leakage under the leakage objective. l_med is its median under
    // the total-power objective.
    float power = 0.0f;
    // The gate's timing pressure, whose median is t_med: the sum over its
    // output pins of (sum of lambda over the pin's in-arcs) * d. Under
    // timing_cost = per_arc each in-arc's lambda is multiplied by that arc's
    // own delay instead. Only meaningful when has_pressure; a gate whose output
    // pins all sit at the lambda floor contributes no t_med sample.
    float lambda_delay = 0.0f;
    bool has_pressure = false;
    // Sum of d over the gate's output pins that drive at least one data arc:
    // the same per-gate quantity without lambda. `unit` takes its median d_med.
    //
    // The membership test deliberately differs from has_pressure: it is
    // structural (does the pin drive a data arc?) rather than a lambda
    // threshold. A threshold would bring lambda back into d_med, because a
    // uniform rescale of lambda could move arcs across the floor and change
    // which gates are sampled, and `unit` must not depend on the magnitude of
    // lambda (see TimingScaleTest.UnitIsInvariantUnderUniformLambdaRescale).
    float delay = 0.0f;
    bool has_delay = false;
  };
  std::vector<Gate> gates;
};

// Result of computeTimingWeight(), including the medians behind it so the
// caller can log them (computeTimingWeight() itself does not log).
struct TimingScaleWeight
{
  float tw = 1.0f;
  // Median power sample: leakage, or objective power under the total-power
  // objective.
  float l_med = 0.0f;
  // t_med (median over gates of sum(lambda * d)) under auto_median and
  // livramento_alpha; d_med (median over gates of sum(d)) under unit.
  float anchor_med = 0.0f;
  // No gate had a power sample, none had the anchor quantity, or a median
  // was not positive. tw then falls back to 1.0.
  bool degenerate = false;
};

// Computes tw from the design (no STA). The result is fixed for the run.
//   auto_median, livramento_alpha: tw = timing_bias * l_med / t_med,
//                                  t_med = median over gates of sum(lambda * d)
//   unit:                          tw = l_med / d_med,
//                                  d_med = median over gates of sum(d)
// l_med is the median of each gate's leakage, or under the total-power
// objective of each gate's objective power, so that tw keeps the same
// power/timing balance under either objective.
//
// The options differ in how tw responds to the magnitude of lambda:
//   - Under auto_median, t_med is proportional to lambda, so tw ~ 1/lambda and
//     tw * lambda * d is exactly invariant to a uniform rescale of lambda. The
//     magnitude of lambda never reaches the candidate costs. For example, the
//     global power ratio of Mangiras et al., Eq. 6, which multiplies the whole
//     field uniformly, cancels before the first candidate is evaluated.
//   - Under unit, d_med contains no lambda, so scaling lambda by c scales
//     tw * sum(lambda * d) by c. Combined with mu_policy = endpoint_lambda
//     (which makes the projection commute with a uniform rescale), the
//     magnitude of the lambda seed carries through to the candidate cost.
// TestTimingScale.cc checks both properties.
//
// livramento_alpha deliberately uses auto_median's lambda-invariant anchor
// rather than unit's: the seeds it runs with have no meaningful lambda
// magnitude (the default seed is an arc delay in seconds), so preserving that
// magnitude would preserve a unit artifact (see
// GlobalSizingConfig::TimingScale::kLivramentoAlpha). Its alpha multiplies
// this fixed base every iteration (see rescheduleLivramentoAlpha); the anchor
// itself is not recomputed.
TimingScaleWeight computeTimingWeight(
    const TimingScaleInput& in,
    GlobalSizingConfig::TimingScale scale,
    float timing_bias,
    GlobalSizingConfig::PowerObjective objective
    = GlobalSizingConfig::PowerObjective::kLeakage);

// Livramento et al., DATE 2013, Alg. 1 line 9 (no STA):
//   alpha <- alpha * (A_o / max_j a_j)
// where A_o is the timing target and max_j a_j the worst arrival over the
// outputs. A_o is read as the clock period T and max_j a_j as T - WNS, which
// is exact when every endpoint is required at T (the paper's single-clock
// setting) and the closest available reading otherwise.
//
// Direction: WNS = 0 gives a ratio of 1 and alpha is unchanged. A violating
// design (WNS < 0) has T - WNS > T, so alpha shrinks and tw = base / alpha
// grows: more timing pressure. A design with positive slack grows alpha and
// recovers leakage.
//
// The paper states no clamp, floor or reset. Two numerical guards are added:
//   - A floor on the denominator. As max_j a_j -> 0 one step's ratio explodes,
//     and at 0 it divides by zero. Flooring at kLivramentoArrivalFloorFrac * T
//     bounds a single step. It only matters when the worst arrival has
//     collapsed to about 0, a case the paper does not consider.
//   - A floor on alpha. The update is multiplicative, so on a design that
//     never meets timing alpha shrinks by a roughly constant factor every
//     iteration and underflows over a long run. tw = base / alpha would then
//     overflow to inf and poison every candidate cost. The denominator floor
//     does not prevent this. kLivramentoAlphaFloor keeps base / alpha finite.
// With T <= 0 (no clock) there is no target and alpha is returned unchanged
// (apart from the floor), like the other strategies that normalize by T.
//
// The result is always >= kLivramentoAlphaFloor, so callers may divide by it.
//
// If `alpha_floor_bound` is non-null it is set to true (never cleared) on any
// step where the alpha floor clamped, so a caller can report whether the guard
// ever took effect during a run, which the clamp would otherwise hide.
float rescheduleLivramentoAlpha(float alpha,
                                float T,
                                float wns,
                                bool* alpha_floor_bound = nullptr);

// The denominator floor as a fraction of T, and the floor on alpha. Exposed for
// the tests. They are numerical guards, not tuning choices, so they are not
// configurable.
inline constexpr float kLivramentoArrivalFloorFrac = 0.01f;
inline constexpr float kLivramentoAlphaFloor = 1e-12f;

// Fills `in` from the current graph, multipliers and library. Reads
// state.lambda, so it must run after seeding and projection; the driver calls
// it once, before the loop.
//
// `scale` only selects which half of each Gate to fill. Under auto_median the
// lambda-free half is skipped, so no extra gate delays are computed.
// computeTimingWeight() uses whatever it is given, so a test can evaluate one
// hand-built input under both options.
void collectTimingScaleInput(LrState& state,
                             Resizer* resizer,
                             const LRSubproblem& subproblem,
                             GlobalSizingConfig::TimingScale scale,
                             TimingScaleInput& in);

// Collects, computes and logs tw for config.timing_scale. The sweep engines
// call it once before the loop. Under livramento_alpha this returns the base;
// the driver divides it by the current alpha each iteration.
float timingWeightBase(LrState& state,
                       Resizer* resizer,
                       const LRSubproblem& subproblem);

}  // namespace rsz
