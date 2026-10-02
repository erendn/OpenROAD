// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

namespace rsz {

// Arithmetic of the local-negative-slack veto used by
// downsize_guard = local_slack_veto (Flach et al., TCAD 2014, Alg. 4 lines 1
// and 12-14, Eq. 14). Kept as pure functions so they can be unit tested.
//
// The veto runs inside LRSubproblem::evaluateSnapshot, on a worker thread,
// against a frozen GateSnapshot. The snapshot decides which slacks the veto
// sees (frozen at sweep start or refreshed per commit, see
// GlobalSizingConfig::GsRefresh); these functions are the rule applied to them.

// Eq. 14 with the gamma_local_slack tolerance scale s:
//
//   gamma = 1 + s * (-min(0, worst_slack) / T)
//
// gamma >= 1. While the design violates timing, gamma > 1 and the veto accepts
// a bounded local degradation, which lets the greedy sweep climb hills early in
// the run. As WNS approaches 0, gamma decays to 1 and no degradation is
// allowed. s = 1 is the paper's rule; s = 0 disables hill climbing. Returns 1
// when T <= 0 (no clock period to normalize by).
float flachGamma(float worst_slack, float T, float tolerance_scale);

// One net's contribution to the gate's local negative slack after a candidate
// cell adds `delay_delta` to the delay of the arc driving that net:
// min(0, slack - delay_delta). Positive slacks contribute 0, so the sum over
// the gate's driver nets and its sink net is the paper's local negative slack,
// which is always <= 0. delay_delta = 0 gives the current cell's contribution
// (the paper's originalSlack); a faster arc (delay_delta < 0) moves a violating
// net back toward zero.
float negativeSlackAfter(float slack, float delay_delta);

// Alg. 4 line 13 as an acceptance test. The paper rejects a candidate when
//   candidate_local_slack < gamma * original_local_slack,
// so it is accepted iff candidate >= gamma * original. Both local slacks are
// <= 0 and gamma >= 1, so gamma * original <= original: the candidate may
// worsen the local negative slack, but only within the gamma-scaled allowance.
// When original == 0 (the gate touches no violating net), any new local
// negative slack is rejected, whatever gamma is.
bool localSlackVetoOk(float candidate_local_slack,
                      float original_local_slack,
                      float gamma);

// The veto as the sweep applies it, gated by the near-met latch
// (LrState::near_met). Sharma et al. (ICCAD 2015, Sec. V-A) apply the
// driver/sink slack check only during power recovery, "after the design timing
// is within 1% of the target": while `active` is false every candidate passes.
// With near_met_gate_frac < 0 the latch is set from the first iteration, so
// `active` is always true and this is exactly localSlackVetoOk.
bool localSlackVetoOkGated(bool active,
                           float candidate_local_slack,
                           float original_local_slack,
                           float gamma);

}  // namespace rsz
