// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <vector>

namespace sta {
class Instance;
class LibertyCell;
}  // namespace sta

namespace rsz {

struct LrState;

// Post-sweep max-cap re-check.
//
// The sweep's electrical filter (LRSubproblem::candidateDrcOkSnapshot) tests
// each candidate against the load frozen in the gate's snapshot. The Jacobi
// engine snapshots every gate before any commit, so a candidate cannot see the
// load that its neighbours' moves add to the net it drives: the filter cannot
// catch max-cap violations that the sweep itself creates. The Gauss-Seidel
// engine snapshots each gate just before evaluating it, so it sees earlier
// commits, but no traversal order lets it see later ones.
//
// This pass therefore runs after a sweep's commits. It re-evaluates max-cap on
// the nets the moved gates touch, using the committed loads and limits, and
// reverts the moves that pushed a net into violation. It is part of the shared
// sweep machinery rather than a config option. Under relax_max_cap it leaves
// the nets whose driver has a max-capacitance multiplier, since the cost
// prices their violations instead.
//
// It does not add max-cap to the LR objective and does not repair violations;
// it only refuses to keep a move that caused one. A net that violated before
// the sweep still violates afterwards, but the sweep cannot make it worse. Only
// max-cap is re-checked: the filter's slew estimate uses the candidate's own
// drive resistance, so it does not have the same blind spot.
//
// Interaction with output_drc_veto = relative. The relative rule admits moves
// on a pin that already violates its limit, and reverting them here would undo
// that option. On a pin that already violates max-cap this cannot happen: the
// load is the same before and after the swap, so "does not worsen the
// violation" means the new cell's cap limit is not lower (driverLimitDelta >=
// 0), while a move is only reverted on a net it made worse (slack_delta < 0,
// see selectCapReverts). Two cases can still be reverted:
//   * a pin that violates only its slew limit. Its cap excess is 0, so the cap
//     check there is the absolute rule against the frozen load, which allows a
//     cell with a lower cap limit than the current one.
//   * another output pin of a multi-output gate that was within its limits.
//     The relative rule applies per pin, so that pin is also only checked
//     against the frozen load and may get a lower limit.
// In both cases a neighbour's commit can push that net into violation. The
// move is then reverted and counted in RSZ-0443 like any other revert.

// One moved gate's signed effect on one net's max-cap slack, in farads.
struct CapContribution
{
  // Index into the sweep's list of moved gates (their commit order). Used only
  // to identify the gate and to break ties, so the selection depends on that
  // order and nothing else.
  int mover = 0;
  // How the gate's cell change moved this net's cap slack. On a net it drives:
  // the change in its own effective cap limit, so a downsize that lowers the
  // limit is negative. On a net it loads: minus the change in its input pin
  // cap, so an upsize that adds load is negative. Negative always means the
  // move pushed the net toward violation, and reverting it gives back exactly
  // this much slack.
  float slack_delta = 0.0f;
};

// Pure function (no STA): which of a violating net's moved gates to revert.
//
// Returns the fewest contributors, largest offender first, that bring `slack`
// back to non-negative. Reverting every contributor of a shared net would put
// it back exactly where the next sweep found it, so the same moves would be
// proposed and reverted on every iteration and those gates would never settle.
// Reverting just enough leaves the net at its limit, where the sweep's own
// frozen check rejects further growth.
//
// Empty when the net does not violate, or when no contributor pushed it toward
// violation: a move that only relieved a net is never reverted for that net.
// When the contributors together cannot recover the slack (the net already
// violated before the sweep), all of them are returned, since each one still
// made a violating net worse.
std::vector<int> selectCapReverts(
    float slack,
    const std::vector<CapContribution>& contributions);

// Pure function (no STA): how a swap changed the effective max-cap limit on a
// pin the moved gate drives, in farads. Positive means the new cell raised the
// limit (an upsize, which can only help). Negative means it lowered it: a
// downsize lowers the limit without changing the load, so whether it causes a
// violation depends on a load the frozen filter read before the sweep.
//
// `effective_limit` is what the live check reports for the committed cell: the
// tighter of its Liberty limit and any SDC max_capacitance. Returns 0 in two
// cases:
//   * an SDC limit binds (effective_limit < the committed cell's Liberty
//     limit, or that cell declares none). The previous cell would have been
//     clamped to the same limit, so the swap did not change it.
//   * the previous cell declared no limit. Reverting would remove the limit
//     rather than raise it, so there is no finite slack to give back and
//     nothing to rank the move by. Under-attributing is preferred to ranking it
//     above every real contributor. This is essentially unreachable inside one
//     Liberty equivalence group.
// Otherwise the Liberty limit binds before and after, and the result is the
// difference of the two limits. The previous limit is not clamped to the
// committed one, since that would zero out every downsize (where prev > cur).
float driverLimitDelta(bool cur_liberty_exists,
                       float cur_liberty_limit,
                       bool prev_liberty_exists,
                       float prev_liberty_limit,
                       float effective_limit);

// One cell the sweep replaced, with what it replaced.
struct MovedGate
{
  sta::Instance* inst = nullptr;
  sta::LibertyCell* prev_cell = nullptr;
  // Not used by the re-check; lets the caller count a revert against the
  // sweep's upsize or downsize total.
  bool was_downsize = false;
};

struct CapRecheckStats
{
  int passes = 0;
  int reverted = 0;
  // True when the pass bound was reached with reverts still happening, i.e. the
  // re-check stopped early rather than because it ran clean.
  bool bound_hit = false;
};

// Maximum number of re-check passes. The pass after one with reverts is
// normally clean; the bound limits the parasitics updates spent on cascades
// (reverting a gate restores its old input cap, which can push a different net
// over its limit). Termination does not depend on the bound: every pass that
// continues removes at least one gate from the moved set, and reverting all of
// them restores the pre-sweep netlist, which has no sweep-created violations.
inline constexpr int kCapRecheckMaxPasses = 4;

// Re-check the nets of the moved gates against the committed netlist and
// revert the moves that created a max-cap violation. On return `movers` holds
// only the moves that were kept. Updates parasitics itself, since the check
// reads live loads; if anything was reverted, the caller's own post-sweep
// parasitics and timing update is still needed.
CapRecheckStats recheckMaxCapAfterSweep(LrState& state,
                                        std::vector<MovedGate>& movers);

}  // namespace rsz
