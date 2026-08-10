// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <vector>

#include "LrState.hh"

namespace rsz {

// B2 axis - Lagrangian cost-model coupling terms (M3). The per-gate cost lives
// in LRSubproblem; this file holds the two *policy-level* main-thread passes
// the φ and delta-delay terms need (they must run before the parallel snapshot
// phase so the sweep workers only read frozen data), plus the pure arithmetic
// cores each term prices - factored out here so they can be unit-tested against
// hand-computed values with no STA (the established M1/M2 pattern).

// === Pure formula cores (unit-tested; no STA) ==============================

// Candidate-induced output-slew change on a driver pin, from the same
// resistive slew model the DRC filter uses: new_slew = slew * cand_R/R, so
// Δslew = slew * (cand_R/R - 1). 0 when the current drive resistance is
// non-positive (no usable slope). cand_R > R (a weaker/downsized cell) gives
// Δslew > 0 (slower edges); cand_R < R (a stronger cell) gives Δslew < 0.
float candidateSlewDelta(float cur_slew, float drive_res, float cand_drive_res);

// Livramento (cost_fanout_slew) / Flach (cost_global_phi) share this shape: a
// per-output-pin sensitivity sum, frozen on the main thread, priced against the
// candidate's output-slew change. Returns sens_sum * slew_delta (a λ-weighted
// delay change; the caller multiplies by timing_weight).
float slewSensitivityCost(float sens_sum, float slew_delta);

// Flach Eq. 11 φ recurrence for one timing arc i->j:
//   φ_{i→j} = λ_{i→j} · (δd_{i→j}/δslew_i)
//             + (δslew_j/δslew_i) · { Σ φ_{j→k}   if i→j is the dominant arc
//                                   { 0            otherwise
// `downstream_sum` = Σ φ over the arcs driven by arc i→j (the out-arcs of node
// j). `dominant` selects the non-zero branch (only the worst-slew arc into j
// propagates the cumulative downstream sensitivity, to avoid multi-counting it
// across a gate's fan-in - Flach §VII-C).
float flachPhiArc(float lambda,
                  float dd_dslew,
                  float dslew_dslew,
                  float downstream_sum,
                  bool dominant);

// The λ·delay timing cost of one driver pin, in two forms (C3 item 5). The
// per-gate cost every paper prints is the per-arc sum Σ_i λ_i·d_i over the
// gate-internal arcs into the pin (Flach Eq. 5's first sum, Livramento Alg. 2
// L10, Reimann Alg. 1 L13, Mangiras Eq. 2 — each prices each arc's own λ
// against its own delay). The shipped cost instead collapses it to (Σ_i
// λ_i)·d_worst with a single port-worst gateDelay lookup: exact for a
// single-input gate, but it OVERPRICES the non-worst sibling arcs of a
// multi-input gate, since d_worst >= every d_i. Both forms are factored out
// here so the divergence is gtest-pinned and the two LRSubproblem call sites
// price through a named function; the fix (should the runtime cost allow it) is
// to feed the per-arc form instead of the port-worst one. See the C3 completion
// notes for the measured runtime decision.
struct ArcLambdaDelay
{
  float lambda = 0.0f;
  float delay = 0.0f;
};
float perArcTimingCost(const std::vector<ArcLambdaDelay>& arcs);
float portWorstTimingCost(float lambda_sum, float port_worst_delay);

// Delta-delay referencing: price an arc delay relative to a per-arc reference
// rather than absolutely. Returns d_cand - d_ref.
//
// NOT Ozdal's Eqs. 7-10, despite the axis name - see cost_delta_delay in
// GlobalSizingConfig.hh for the full divergence. In one line: Ozdal references
// a *load* and re-evaluates delay_ref_k(s_i^j) from each candidate's own table
// (Eq. 7), so his reference moves with the candidate; `d_ref` here is the arc's
// frozen incumbent delay, which does not. M6 must build Eq. 7's reference
// itself; it cannot reuse LrState::prev_delay.
float deltaDelayReferenced(float d_cand, float d_ref);

// === Policy-level main-thread passes =======================================

// cost_global_phi: back-propagate the cumulative λ-weighted delay sensitivity φ
// over every data arc, once per iteration, in reverse-topological order
// (path-outputs -> inputs). Fills state.phi (indexed by sta::Edge::id, same
// space as state.lambda). Reads live STA (slews, loads, table finite
// differences) so it MUST run on the main thread before the parallel snapshot
// phase. Gate arcs get table-derived sensitivities; wire arcs pass slew through
// (δd/δslew = 0, δslew/δslew = 1) under the lumped-capacitance model Flach's
// estimation assumes (§IV).
void computePhiSensitivities(LrState& state);

// cost_delta_delay: snapshot the current per-arc delays into state.prev_delay
// (indexed by sta::Edge::id) as the reference for the next sweep's referenced
// costs. Call once per iteration before the sweep, on the main thread. The
// snapshot is candidate-independent by construction, which is exactly where it
// parts company with Ozdal's Eq. 7 (see deltaDelayReferenced above).
void captureReferenceDelays(LrState& state);

}  // namespace rsz
