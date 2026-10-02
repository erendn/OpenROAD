// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <cstddef>
#include <functional>
#include <string_view>
#include <vector>

namespace rsz {

// Restricted LRS move sets (config move_set): the decomposition of a
// swappable-equivalence group into a grid of (width rank x Vth flavor), and
// the two move sets that index into it.
//
// Kept separate from LRSubproblem, as InitSelect is from InitPass, because the
// rules depend only on the group's members and can be unit-tested on
// hand-built groups without a Liberty library, STA graph or design.
// LRSubproblem does the STA-facing part (what the group is, each member's
// flavor and rank keys, what a candidate costs, and whether it passes the DRC
// filter and the downsize guard).
//
// The same grid serves both move sets:
//   - Fast-OLR (Sharma et al., ICCAD 2015, Fig. 9): from the fifth LDP
//     iteration on, replace exhaustive enumeration with a local descent along
//     the width axis within the current Vth flavor and its two neighbours.
//     The paper reports that this local search also makes the solution more
//     stable.
//   - The restricted mode of Mangiras and Dimitrakopoulos (Technologies 2021,
//     Sec. 4.3): candidates are one size step up or down from the current
//     cell, with any Vth.
//
// About "width": the input has no size or drive dimension. `rank_key` is the
// caller's leakage-equivalent cost (LRSubproblem::leakageOrArea), and the
// width rank is the rank of that key within a Vth flavor. Within one flavor of
// a real library, leakage grows with drive strength, so the rank works as a
// proxy for size. "Next bigger size" therefore means "next member up the
// leakage ranking of this flavor", and "cell(w, Vth)" means "the member of
// flavor Vth at width rank w".
struct GridMember
{
  // Opaque Vth-flavor identity; members sharing a key are one flavor. Supplied
  // by the caller (the index from Resizer::cellVTType). It is only used to
  // group members and order the resulting flavors.
  int vth_key = 0;
  // The width/drive proxy: the member's leakage-equivalent cost. Always defined
  // (leakageOrArea falls back to scaled area), so unlike InitSelect there is no
  // unrankable member here and no "cannot be placed" case.
  float rank_key = 0.0f;
  // Final tie-break, so the grid depends only on the group's members, not on
  // the order in which the library lists them.
  std::string_view name;
};

// One member's position on the grid.
struct GridCoord
{
  // 0-based, ascending: flavor 0 is the least-leaky (highest-Vth) flavor. The
  // order is by the flavor's mean rank key, then by vth_key - the same
  // ascending-average-leakage convention Resizer's own
  // LibraryAnalysisData::sort_vt_categories uses to order HVT/RVT/LVT/uLVT.
  int flavor = 0;
  // 0-based, ascending rank_key within the member's own flavor. Consecutive by
  // construction: a flavor with n members occupies ranks 0..n-1.
  int width_rank = 0;
};

// Decompose `group` into the grid. Returns one coord per member, in input
// order.
//
// Deterministic and order-independent: flavors sort by (mean rank key,
// vth_key), members within a flavor by (rank key, name). A single-flavor
// library (no cell carries a Vt implant-layer signature, as in Nangate45)
// yields one column, flavor 0, which both move sets handle without special
// cases.
std::vector<GridCoord> buildSizeVthGrid(const std::vector<GridMember>& group);

// Marker for an empty grid slot: the incumbent's own position (it is a member
// of the group but never a candidate), or a rank a ragged flavor does not
// reach.
inline constexpr int kNoGridCandidate = -1;

// The grid's column layout, addressable by (flavor, width rank). It depends
// only on the group's members, so the caller builds it once per library cell
// and caches it; neither move set allocates per gate.
//
// `slot` is the flat concatenation of the columns and holds candidate indices,
// which exclude the incumbent, so the incumbent's own position reads back
// kNoGridCandidate. Both move sets start from that position.
struct GridLayout
{
  // Offsets into `slot`, size flavorCount()+1.
  std::vector<int> column_begin;
  std::vector<int> slot;

  int flavorCount() const
  {
    return column_begin.empty() ? 0 : static_cast<int>(column_begin.size()) - 1;
  }
  // How many width ranks flavor `f` has. 0 for a flavor with no members.
  int columnHeight(int flavor) const;
  // The candidate at (flavor, width rank), or kNoGridCandidate.
  int at(int flavor, int width_rank) const;
};

// Build the layout of a decomposition. `coords` are the coords of the
// candidate cells (the incumbent excluded); `cur` is the incumbent's own
// coord, for which the layout reserves an empty slot so that "one size up from
// here" is defined.
GridLayout buildGridLayout(const std::vector<GridCoord>& coords, GridCoord cur);

// One winner of a Fast-OLR walk, with the cost the walk already computed, so
// the caller's argmin does not evaluate those cells again (Fast-OLR exists to
// reduce the number of cell evaluations).
struct FastOlrWinner
{
  int candidate = kNoGridCandidate;
  float cost = 0.0f;
};

// Fast-OLR (Sharma et al., ICCAD 2015, Fig. 9). Returns the `candidates` set
// of line 19: up to two winners per visited Vth flavor, one per direction.
// Lines 17-18 repeat lines 5-16 with maxw replaced by minw; lines 5-7 are the
// initialization and line 16 the insert, so the descending walk starts again
// from the same seed and inserts its own winner. `candidates` is a set, so a
// cell returned by both directions appears once. The caller applies line 20
// (argmin over the returned set, subject to its downsize guard).
//
// A flavor whose walk never beats the incumbent adds nothing to the set. For
// the current flavor this matches the paper's "bestcell is still
// cell(currw, Vth)", since the caller's argmin already starts at the
// incumbent.
//
// `valid(i)` is the caller's candidate filter (the DRC check, plus the
// power-phase filter when it is on) and `cost(i)` is the cost of candidate i;
// each runs at most once per visited member per flavor. Neither may have side
// effects.
//
// The figure leaves some details open. This implementation resolves them as
// follows:
//
//  1. The walk starts one step away from currw. Read literally, line 8 starts
//     the ascending walk at w = currw after line 6 has set
//     bestcost = cost(cell(currw, Vth)), so the first comparison is a cell
//     against itself; it fails `<` and the walk breaks immediately in both
//     directions. Fast-OLR would then only evaluate the three cells at the
//     current width, which does not fit the 3.3x reduction in cell
//     evaluations the paper reports (Fig. 10): from about 30 options per gate
//     (10 sizes x 3 Vth, as in the ISPD contest libraries) that leaves about
//     9 evaluations, not 3. So the walks are currw+1..maxw and currw-1..minw.
//  2. An invalid cell is skipped, not a stop. Line 10's "Ensure c is valid"
//     is read as a skip, so one hole in a ragged grid, or one cap-limited
//     cell, cannot cut the walk short. Stopping instead could only see fewer
//     cells than the paper intends.
//  3. An invalid seed does not set the walk's threshold. Lines 5-7 do not test
//     the seed, because the paper's grid is complete. Taken literally, a
//     DRC-rejected seed would set a bestcost that no reachable cell has to
//     beat and could remove a whole Vth flavor from the search. So when the
//     seed is invalid the walk compares against the incumbent's cost instead.
//     This is rule 2 applied to the seed, and it can only widen the search.
//     The incumbent, cell(currw, curVth), is in the paper's own line-20 set.
//  4. A ragged grid clamps. Line 5's cell(currw, Vth) assumes every flavor has
//     every width. When a neighbouring flavor has fewer ranks than
//     `cur.width_rank`, the walk starts from that flavor's highest rank
//     instead of skipping the flavor.
std::vector<FastOlrWinner> fastOlrCandidates(
    const GridLayout& layout,
    GridCoord cur,
    float cur_cost,
    const std::function<bool(int)>& valid,
    const std::function<float(int)>& cost);

// The restricted move set of Mangiras and Dimitrakopoulos (Technologies 2021,
// Sec. 4.3): a gate may move only to its next bigger or next smaller size,
// without limiting Vth swaps. The band applies to the width rank while the
// flavor is free, so a pure Vth swap (same rank, other flavor) is allowed.
//
// Ragged columns clamp, as in rule 4 of fastOlrCandidates. The paper's library
// is a complete 10 sizes x 3 Vth grid, so "the same size in another flavor" is
// always defined; ours need not be. Comparing raw ranks across flavors of
// different depths would break the Vth-swap rule: a gate at rank 5 of an
// 8-deep flavor could reach nothing in a 3-deep one. So the incumbent's rank
// is first clamped into the candidate's column.
bool withinSizeStep(const GridLayout& layout, GridCoord cur, GridCoord cand);

}  // namespace rsz
