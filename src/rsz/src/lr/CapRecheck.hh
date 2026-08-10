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

// The post-sweep max-cap re-check (iteration-2 plan §2.2-1).
//
// WHY THIS EXISTS. The sweep's electrical veto
// (LRSubproblem::candidateDrcOkSnapshot) tests each candidate against the
// gate's FROZEN snapshot load. Under the Jacobi engine every gate is
// snapshotted before any commit, so no candidate can see the load its
// neighbours' moves are about to add to the net it drives - the veto is
// structurally blind to the max-cap violations the sweep itself creates. That
// blindness is the campaign's number-one loss channel: max-cap ERC decides
// essentially every loss tail (stage1_analysis/SYNTHESIS.md §3.3, and the S1-T
// review's traced finding at the checkOutputMaxCap call site). The Gauss-Seidel
// engine snapshots just-in-time against genuinely live loads, so it sees every
// EARLIER commit - but it still cannot see a LATER one, and no traversal order
// can.
//
// The fix that covers both engines is therefore a bounded re-check AFTER the
// commits: re-evaluate max-cap on the nets the sweep's movers touch, now that
// the loads and the limits are the committed ones, and revert the movers that
// pushed a net into violation.
//
// This is a shared-machinery correctness fix, not an axis. It corrects an
// implementation artifact of the sweep engines rather than any paper's choice,
// so it runs identically for rsz_baseline and for every preset - there is no
// config knob and no per-preset behaviour (plan §2.2-1: "no principle-B
// conflict").
//
// WHAT IT DOES NOT DO. It does not add a max-cap constraint to the LR
// objective, and it does not repair a violation - it only refuses to keep the
// move that caused one. A net that was already violating before the phase stays
// violating; what changes is that the sweep can no longer make it worse. It
// also prices max-cap only: max-slew is left to the candidate filter, which is
// not load-blind in the same structural way (the slew estimate rides on the
// candidate's own drive resistance).
//
// INTERPLAY WITH output_drc_veto = relative (A2). The relative veto newly
// admits moves on a pin the gate ALREADY violates, and a re-check that reverted
// them would silently undo the axis for any preset pinning both - which
// flach_partial and chinnery_partial do. It mostly cannot, and the boundary is
// worth stating precisely rather than as a blanket "cannot".
//
// A mover is only ever a revert candidate on a net it made WORSE
// (`slack_delta < 0`, selectCapReverts). ON THE CAP-DIRTY PIN THAT UNLOCKED THE
// MOVE that is impossible: the load is frozen across the swap, so the veto's
// "does not worsen" reduces exactly to "the candidate's cap ceiling is not
// lower", i.e. driverLimitDelta >= 0, and the mover cannot be in that net's
// harmful set. There the two rules really are the same rule.
//
// Two cases the argument does NOT cover, both of which the re-check may revert:
//   * a pin unlocked on the SLEW leg. Its cap excess is 0, so the cap leg runs
//     the absolute rule ("clears the frozen load"), which permits a candidate
//     whose cap ceiling is LOWER than the incumbent's.
//   * any OTHER output pin of a multi-output mover that was clean. The relative
//     bar binds per pin, so a clean pin is likewise judged only against the
//     frozen load and may take a lower ceiling.
// In both, a neighbour's commit can then push that net live-violating and the
// climb-out is reverted with it. That is this pass doing its job - the move did
// lower a ceiling on a violating net - but it means a preset pinning relative
// can see the axis partially undone, reported only as an ordinary RSZ-0443
// count. Anyone measuring the axis should read that counter.

// One mover's signed effect on ONE net's max-cap slack, in farads.
struct CapContribution
{
  // Which mover, as an index into the sweep's mover list (i.e. its commit
  // order). Only used to identify and to break ties, so the selection is a
  // function of that order and of nothing else.
  int mover = 0;
  // How this mover's own cell change moved this net's cap slack. On a net it
  // drives: the delta of its own effective cap limit, so a downsize that lowers
  // the ceiling is negative - the S1-T review's second half, where the veto's
  // limit comes from the candidate's own max_capacitance. On a net it loads:
  // the negated delta of its input pin cap, so an upsize that raises the load
  // is negative. Either way NEGATIVE means this mover pushed the net toward
  // violation, and reverting it hands exactly this much slack back.
  float slack_delta = 0.0f;
};

// Pure core (no STA): which of a violating net's movers to revert.
//
// Returns the FEWEST contributors, biggest offender first, that bring `slack`
// back to non-negative - not every contributor. Minimality is what makes the
// re-check converge: reverting the whole contributing set of a shared net puts
// the net back exactly where the next sweep found it, so the same set is
// re-proposed and re-reverted every iteration and those gates never settle.
// Handing back just enough leaves the net AT its ceiling, where the sweep's own
// frozen check then rejects further growth on its own.
//
// Empty when the net is not violating, and empty when nothing that touched it
// pushed it that way - a mover that only relieved a net is never reverted for
// it, which is what keeps an innocent bystander's move out of the revert set on
// a net some other gate ruined. When the contributors together cannot recover
// the slack (the net was already violating before the sweep), all of them are
// returned: each one is still a move that made a violating net worse.
std::vector<int> selectCapReverts(
    float slack,
    const std::vector<CapContribution>& contributions);

// Pure core (no STA): how a swap moved the EFFECTIVE max-cap ceiling on a pin
// the mover drives, in farads. Positive = the mover raised its own ceiling (an
// upsize; it can only help). Negative = it lowered it, which is the S1-T
// review's second half - a downsize lowers the limit while raising nothing, so
// whether that is a violation depends entirely on a load the frozen veto read
// before the sweep.
//
// `effective_limit` is what the live check reports for the COMMITTED cell,
// i.e. the tighter of that cell's Liberty limit and any SDC max_capacitance.
// Two cases return 0 rather than a delta:
//   * an SDC rule is the binding ceiling (effective_limit < the committed
//     cell's Liberty limit, or that cell declares none). The pre-sweep cell
//     would have been clamped to the same ceiling, so the swap did not move it.
//   * the pre-sweep cell declared no ceiling at all. Reverting would REMOVE the
//     ceiling rather than raise it, so there is no finite slack figure to hand
//     back and the biggest-offender ordering has nothing to rank it by;
//     deliberately under-attributing beats out-ranking every real contributor.
//     Essentially unreachable inside one Liberty equivalence group.
// Otherwise the cell's Liberty limit binds both before and after, and the delta
// is simply the two limits' difference - NOT clamped at the committed one,
// which would silently zero out every downsize (a downsize is exactly the case
// prev > cur).
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
  // Which half of the sweep's move tally this gate belongs to. Not read by the
  // re-check itself; kept so a caller can report a revert against the right
  // one.
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

// Pass bound. A pass that reverts anything leaves every net it touched at or
// above its ceiling, so one more pass normally runs clean; the bound only caps
// the parasitics re-estimates spent on the cascading case (reverting a gate
// restores its input cap, which can push a DIFFERENT net over). Termination
// does not depend on it: each pass strictly shrinks the mover set, and
// reverting every mover restores the pre-sweep netlist, which by construction
// has none.
inline constexpr int kCapRecheckMaxPasses = 4;

// Re-check the movers' nets against the committed state and revert the ones
// that created a max-cap violation. Consumes `movers`: on return it holds the
// moves that survived, so the caller can report the kept tally. Re-estimates
// parasitics itself (the check reads live loads); leaves the netlist dirty for
// the caller's own post-sweep update when it reverted anything.
CapRecheckStats recheckMaxCapAfterSweep(LrState& state,
                                        std::vector<MovedGate>& movers);

}  // namespace rsz
