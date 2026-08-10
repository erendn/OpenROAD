// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

namespace rsz {

// F3 axis (downsize_guard) - the arithmetic of Flach's local-negative-slack
// veto (TCAD'14 Alg. 4 lines 1/12-14, Eq. 14; flach_et_al.md §6), factored out
// of LRSubproblem as pure functions so it is unit-testable against
// hand-computed values.
//
// Unlike the other axes this one is not a strategy class: the guard runs inside
// evaluateSnapshot, on a worker thread, against a frozen GateSnapshot - the
// snapshot freezes WHICH slacks the veto sees (that is where the gs_local /
// gs_incremental staleness split lives, see GlobalSizingConfig::DownsizeGuard),
// and these functions are the decision rule applied to them.

// Eq. 14 with the gamma_local_slack tolerance scale s:
//
//   gamma = 1 + s * (-min(0, worst_slack) / T)
//
// gamma >= 1 always. It exceeds 1 while the design violates - the veto then
// tolerates bounded local degradation, which is what lets the greedy sweep
// climb hills early - and decays to 1 as WNS approaches 0, forbidding
// degradation near convergence. s = 1 is exactly the paper; s = 0 pins gamma to
// 1 (no hill climbing). Returns 1 when T <= 0 (no clock: nothing to normalize
// by, so no hill climbing).
float flachGamma(float worst_slack, float T, float tolerance_scale);

// One net's contribution to the gate's local negative slack after a candidate
// cell adds `delay_delta` to the delay of the arc feeding that net:
// min(0, slack - delta). Positive slacks are discarded (they contribute 0), so
// summing this over the gate's driver nets and its sink net gives Flach's
// "local negative slack", always <= 0. delta = 0 gives the gate's current
// (originalSlack) contribution; a candidate that speeds the arc up (delta < 0)
// lifts a violating net back toward zero.
float negativeSlackAfter(float slack, float delay_delta);

// Alg. 4 line 13, inverted to an acceptance test: the candidate is rejected iff
//   candidate_local_slack < gamma * original_local_slack
// so it is accepted iff candidate >= gamma * original. Both local slacks are
// <= 0 and gamma >= 1, so gamma * original <= original: the candidate may
// degrade the local negative slack, but only by the gamma-scaled allowance.
// Boundary (the paper's, flach_et_al.md §13.8): when original == 0 - the gate
// sits on no violating net - any new local negative slack is rejected outright,
// whatever gamma is.
bool localSlackVetoOk(float candidate_local_slack,
                      float original_local_slack,
                      float gamma);

// The veto as applied in the sweep, gated by the near-met latch (C2). Sharma
// activates the driver/sink slack check only in power recovery, "after the
// design timing is within 1% of the target" (sharma_et_al.md §5.2): while
// `active` is false (the run is not yet near-met) every candidate passes, so
// the veto is inert. An ungated preset keeps `active` true from iteration 0
// (LrState::near_met latched at the start when near_met_gate_frac < 0), so this
// is exactly localSlackVetoOk - flach/reimann/mangiras are unchanged.
bool localSlackVetoOkGated(bool active,
                           float candidate_local_slack,
                           float original_local_slack,
                           float gamma);

}  // namespace rsz
