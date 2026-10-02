// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <vector>

#include "LrState.hh"

namespace rsz {

// Optional terms of the per-gate Lagrangian cost (cost_* options in
// GlobalSizingConfig). The per-gate cost itself is in LRSubproblem. This file
// holds the pure formulas each term uses, which are unit tested without STA,
// and the two main-thread passes the φ and delta-delay terms need. Those passes
// run before the parallel snapshot phase so the sweep workers only read frozen
// data.

// === Pure formulas (no STA) ================================================

// Output-slew change on a driver pin caused by a candidate cell, using the
// same resistive slew model as the DRC filter: new_slew = slew * cand_R / R, so
// Δslew = slew * (cand_R / R - 1). Returns 0 when the current drive resistance
// is not positive. A weaker cell (cand_R > R) gives Δslew > 0; a stronger cell
// gives Δslew < 0.
float candidateSlewDelta(float cur_slew, float drive_res, float cand_drive_res);

// Shared by cost_fanout_slew (Livramento et al., DATE 2013) and
// cost_global_phi (Flach et al., TCAD 2014): a per-output-pin sensitivity sum,
// frozen on the main thread, times the candidate's output-slew change. The
// result is a λ-weighted delay change; the caller multiplies it by the timing
// weight.
float slewSensitivityCost(float sens_sum, float slew_delta);

// Flach Eq. 11 φ recurrence for one timing arc i->j:
//   φ_{i→j} = λ_{i→j} · (δd_{i→j}/δslew_i)
//             + (δslew_j/δslew_i) · { Σ φ_{j→k}   if i→j is the dominant arc
//                                   { 0            otherwise
// `downstream_sum` is Σ φ over the arcs driven by arc i→j (the out-arcs of
// node j). Only the dominant arc into j, the one with the worst slew,
// propagates the downstream sum, so it is not counted once per fanin arc
// (Flach Sec. VII-C).
float flachPhiArc(float lambda,
                  float dd_dslew,
                  float dslew_dslew,
                  float downstream_sum,
                  bool dominant);

// The λ·delay timing cost of one driver pin, in two forms. The papers price
// each gate-internal arc into the pin against its own delay, Σ_i λ_i·d_i
// (Flach Eq. 5, first sum; Livramento Alg. 2 line 10; Reimann Alg. 1 line 13;
// Mangiras Eq. 2). By default LRSubproblem uses the cheaper (Σ_i λ_i)·d_worst,
// which needs one port-worst gateDelay lookup per pin instead of one lookup per
// arc; timing_cost = per_arc selects the per-arc form. The two agree for a
// single-input gate; for a multi-input gate the port-worst form overprices the
// non-worst arcs, since d_worst >= every d_i.
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
// This is not the reference of Ozdal et al., ICCAD 2011, Eq. 7. There the
// reference is a load, evaluated through each candidate's own delay table, so
// it changes with the candidate. Here `d_ref` is the arc's current delay,
// which is the same for every candidate (see cost_delta_delay in
// GlobalSizingConfig.hh).
float deltaDelayReferenced(float d_cand, float d_ref);

// === Main-thread passes ====================================================

// cost_global_phi: back-propagate the cumulative λ-weighted delay sensitivity φ
// over every data arc, once per iteration, in reverse topological order (path
// outputs to inputs). Fills state.phi, indexed by sta::Edge::id like
// state.lambda. Reads live STA (slews, loads, table lookups), so it must run on
// the main thread before the parallel snapshot phase. Gate arcs get
// sensitivities from the Liberty tables; wire arcs pass the slew through
// (δd/δslew = 0, δslew/δslew = 1), matching the lumped-capacitance model Flach
// assumes for sensitivities (Sec. VII-C).
void computePhiSensitivities(LrState& state);

// cost_delta_delay: copy the current per-arc delays into state.prev_delay
// (indexed by sta::Edge::id) as the reference for the next sweep. Call once per
// iteration before the sweep, on the main thread.
void captureReferenceDelays(LrState& state);

}  // namespace rsz
