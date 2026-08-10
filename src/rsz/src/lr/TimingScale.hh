// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <vector>

#include "LrState.hh"
#include "rsz/GlobalSizingConfig.hh"

namespace rsz {

class Resizer;
class LRSubproblem;

// A1 axis - the objective scale: what fixes `tw` in `leakage + tw·Σλ·d`.
//
// Same device as FlowProjection's ProjectionTopology and M4's TraversalEntry:
// the arithmetic that decides the answer is a pure function over a
// hand-buildable structure, so it is unit-testable without STA, and the STA
// walk is reduced to filling it in.

// The scale's STA-free view of the design: one entry per editable gate, built
// by collectTimingScaleInput and consumed by computeTimingWeight.
struct TimingScaleInput
{
  struct Gate
  {
    // Leakage (or the leakage-equivalent area cost - LRSubproblem::
    // leakageOrArea) of the gate's current cell. Every editable gate with a
    // liberty cell contributes one, which is what l_med is the median of.
    float leakage = 0.0f;
    // Σ over the gate's output pins of (Σλ over the pin's in-arcs) · d - the
    // per-gate timing pressure auto_median takes t_med over. Only meaningful
    // when has_pressure; a gate whose every output pin sits at the λ floor
    // contributes no t_med sample, exactly as the historical walk did.
    float lambda_delay = 0.0f;
    bool has_pressure = false;
    // Σ d over the gate's output pins carrying at least one data arc - the same
    // median-gate anchor with λ dropped out. `unit` takes d_med over this.
    //
    // Note the membership test differs from has_pressure's on purpose: it is
    // *structural* (does this pin drive a data arc?) rather than λ-thresholded.
    // A λ-thresholded test would put λ back into d_med through the back door -
    // a uniform rescale of λ could move arcs across the floor and change which
    // gates are sampled - and exact λ-invariance is the whole point of the
    // option (pinned by
    // TimingScaleTest.UnitIsInvariantUnderUniformLambdaRescale).
    float delay = 0.0f;
    bool has_delay = false;
  };
  std::vector<Gate> gates;
};

// What computeTimingWeight decided, including the medians behind it so the
// caller can trace them (the pure core stays logger-free).
struct TimingScaleWeight
{
  float tw = 1.0f;
  float l_med = 0.0f;
  // t_med (= median_g Σλ·d) under auto_median; d_med (= median_g Σ d) under
  // unit and livramento_alpha.
  float anchor_med = 0.0f;
  // No gate carried leakage, or none carried the option's anchor, or a median
  // came out non-positive. tw falls back to 1.0.
  bool degenerate = false;
};

// Pure core (no STA): the frozen design anchor for `tw`.
//   auto_median / livramento_alpha: tw = timing_bias · l_med / t_med,
//                                   t_med = median_g(Σλ·d)
//   unit:                           tw = l_med / d_med, d_med = median_g(Σ d)
//
// The λ-invariance contract, which is this axis's reason to exist:
//   - auto_median's t_med is proportional to λ, so tw ∝ 1/λ and tw·λ·d is
//     *exactly* invariant to a uniform rescale of λ. λ's magnitude cannot reach
//     candidate scoring under it - this is mechanism 2 of the plan's
//     "λ-magnitude finding", and it is why Mangiras Eq. 6's global power ratio
//     (a uniform multiplier on the whole field) was cancelled before the first
//     candidate was scored.
//   - unit's d_med contains no λ at all, so tw is literally λ-free: scaling λ
//   by
//     c scales tw·Σλ·d by c. Combined with mu_policy = endpoint_lambda (which
//     makes the E3 projection commute with a uniform rescale), λ's magnitude
//     survives from the seed all the way to the candidate cost.
// Both directions are pinned by TestTimingScale.cc.
//
// livramento_alpha shares auto_median's λ-INVARIANT anchor, not unit's, and
// deliberately: the seeds it runs on carry no meaningful λ magnitude, so a
// λ-preserving base would preserve a unit artifact (see
// GlobalSizingConfig::TimingScale::kLivramentoAlpha). Its α is a per-iteration
// multiplier on top of this frozen base (see rescheduleLivramentoAlpha), not a
// re-anchoring.
TimingScaleWeight computeTimingWeight(const TimingScaleInput& in,
                                      GlobalSizingConfig::TimingScale scale,
                                      float timing_bias);

// Pure core (no STA): Livramento DATE'13 Alg. 1 L9, `α ← α·(A_o / max_j a_j)`,
// with A_o the timing target and max_j a_j the worst arrival over the outputs.
// We read A_o as the clock period T and max_j a_j as T - WNS, which is exact
// when every endpoint is required at T (the paper's single-clock contest
// setting) and the closest available reading otherwise.
//
// Direction check: WNS = 0 gives ratio T/T = 1, so α is unchanged - the fixed
// point at max_j a_j = A_o. A violating design (WNS < 0) has T - WNS > T, so
// ratio < 1, α shrinks, and tw = base/α rises: more timing pressure. A design
// with slack to spare grows α and recovers leakage. That is the controller the
// paper's Figs. 1-2 ablate.
//
// DEVIATION (ours, not the paper's): L9 has no stated clamp, floor or reset.
// Two guards, both numerical rather than behavioral:
//   - the DENOMINATOR floor. max_j a_j → 0 makes one step's ratio explode (the
//     audit's stated failure mode) and at exactly 0 it is a division by zero.
//     Flooring at kLivramentoArrivalFloorFrac·T bounds a single step's growth.
//     It only binds on a design whose worst arrival has collapsed to ~0, which
//     is not a regime the paper contemplates.
//   - the ACCUMULATOR floor. The schedule is multiplicative and compounds
//     without bound in the *shrinking* direction: on a design that never closes
//     timing α falls by a roughly constant factor every iteration (measured
//     ~0.38x/iter), so it underflows toward zero over a long run - and tw =
//     base/α then overflows to +inf, poisoning every candidate cost with
//     inf/nan. Bounding the denominator does not prevent this; only bounding α
//     does. kLivramentoAlphaFloor keeps base/α finite for any base this engine
//     produces. Reachable, not hypothetical: livramento_partial's own cap is 60
//     iterations.
// With T <= 0 (no clock) there is no target to servo to and α is returned
// unchanged (bar the floor), matching how the other T-normalized strategies
// no-op without a clock.
//
// Postcondition: the result is always >= kLivramentoAlphaFloor, so callers may
// divide by it unguarded.
//
// `alpha_floor_bound` is the A-axis diagnostic (S1 §3.3, iteration-2 plan
// §2.2-5): when non-null it is SET (never cleared) on any step where the
// accumulator floor actually clamped, so a caller can accumulate one flag over
// a whole run and report whether the α-runaway guard ever bound. That turns the
// α-runaway hypothesis into a measurement instead of an inference from the
// terminal value, which the clamp itself would otherwise hide.
float rescheduleLivramentoAlpha(float alpha,
                                float T,
                                float wns,
                                bool* alpha_floor_bound = nullptr);

// The denominator floor, as a fraction of T, and the floor on α itself. Exposed
// for the tests that pin the guards; deliberately not config knobs (they are
// numerical guards, not balance choices).
inline constexpr float kLivramentoArrivalFloorFrac = 0.01f;
inline constexpr float kLivramentoAlphaFloor = 1e-12f;

// STA walk: fill `in` from the current graph, multiplier field and library.
// Reads state.lambda, so it must run after the seed + projection - the driver
// calls it once, before the loop, which is this axis's iteration-0 freeze.
//
// `scale` only selects which half of each Gate to fill: under auto_median the
// λ-free half is skipped, so the walk reduces *exactly* to the historical one
// (same pins visited, same gateDelay calls, same arithmetic) and rsz_baseline
// stays byte-identical by construction rather than by argument.
// computeTimingWeight itself stays option-agnostic over whatever a caller hands
// it, which is what lets TestTimingScale.cc score one hand-built input under
// both options.
void collectTimingScaleInput(LrState& state,
                             Resizer* resizer,
                             const LRSubproblem& subproblem,
                             GlobalSizingConfig::TimingScale scale,
                             TimingScaleInput& in);

// collect + compute + trace, for config.timing_scale. What the sweep engines
// call once before the loop to freeze the anchor. Under livramento_alpha this
// returns the *base*; the driver divides it by the live α each iteration.
float timingWeightBase(LrState& state,
                       Resizer* resizer,
                       const LRSubproblem& subproblem);

}  // namespace rsz
