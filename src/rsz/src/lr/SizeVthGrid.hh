// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <cstddef>
#include <functional>
#include <string_view>
#include <vector>

namespace rsz {

// The pure core of the F4 axis (LRS candidate move set): the decomposition of a
// swappable-equivalence group into the (drive/width rank x Vth flavor) GRID the
// two restricted move sets index into, plus the two move sets themselves.
//
// Split out of LRSubproblem the way InitSelect is split out of InitPass, and
// for the same reason: the rules below are decidable from the group's members
// alone, so they can be gtested against hand-built groups with no Liberty
// library, no STA graph, and no design. LRSubproblem owns the STA-facing half
// (what the group is, what each member's flavor key and rank key are, what a
// candidate costs, and whether it clears DRC and the F3 guard).
//
// ONE grid, TWO consumers (REAUDIT F2 - "one (w,Vth) grid indexing serves
// both"):
//   - Sharma ICCAD'15 Fig. 9 Fast-OLR, the paper's stability device: from the
//     5th LDP iteration on, replace exhaustive enumeration with a hill descent
//     along the width axis inside the current Vth flavor and its two
//     neighbours.
//   - Mangiras Technologies'21 §4.3's restricted mode: candidates are the +-1
//     size step from the current cell, with Vth free.
//
// HONESTY NOTE (the same one the N axis carries, GlobalSizingConfig.hh:32-37 -
// and it is load-bearing here, because this file's whole subject is a "width"
// axis). There is NO size or drive dimension in the input. `rank_key` is the
// caller's leakage-equivalent cost (LRSubproblem::leakageOrArea), and the WIDTH
// RANK below is that key's rank INSIDE a Vth flavor. Within one flavor of a
// real library leakage is monotone in drive strength, which is what makes the
// proxy serviceable; it is a proxy nonetheless. "Next bigger size" therefore
// means "next member up the leakage ranking of this flavor", and "cell(w, Vth)"
// means "the member of flavor Vth at width rank w".
struct GridMember
{
  // Opaque Vth-flavor identity - members sharing a key are one flavor. Supplied
  // by the caller (Resizer::cellVTType's index); the core never interprets it,
  // it only groups by it and orders the resulting flavors.
  int vth_key = 0;
  // The width/drive proxy: the member's leakage-equivalent cost. Always defined
  // (leakageOrArea falls back to scaled area), so unlike InitSelect there is no
  // unrankable member here and no "cannot be placed" case.
  float rank_key = 0.0f;
  // Final tie-break, so the grid is a function of the group's MEMBERS and never
  // of the order the library handed the group over in.
  std::string_view name;
};

// One member's position on the grid.
struct GridCoord
{
  // 0-based, ascending: flavor 0 is the least-leaky (highest-Vth) flavor. The
  // order is by the flavor's MEAN rank key, then by vth_key - the same
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
// library (nothing in it carries an implant-layer signature, which is the whole
// of nangate45 and most smoke libraries) yields one column, flavor 0 - the
// degenerate case both move sets below handle without special-casing.
std::vector<GridCoord> buildSizeVthGrid(const std::vector<GridMember>& group);

// Marker for an empty grid slot: the incumbent's own position (it is a member
// of the group but never a candidate), or a rank a ragged flavor does not
// reach.
inline constexpr int kNoGridCandidate = -1;

// The grid's column layout, addressable by (flavor, width rank). Built once per
// swappable GROUP - it is a pure function of the group's members, so the caller
// caches it per library cell and neither move set allocates per gate.
//
// `slot` is the flat concatenation of the columns and holds CANDIDATE indices,
// which exclude the incumbent, so the incumbent's own position reads back
// kNoGridCandidate. That hole is exactly where both move sets start from.
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

// Build the layout of a decomposition. `coords` are the coords of the CANDIDATE
// cells (the incumbent excluded); `cur` is the incumbent's own coord, which the
// layout reserves a hole for so "one size up from here" has a referent.
GridLayout buildGridLayout(const std::vector<GridCoord>& coords, GridCoord cur);

// One winner of a Fast-OLR walk, with the cost the descent already paid to
// score it - so the caller's argmin does not re-run the cost model on cells the
// descent has just evaluated (the whole point of the option is to spend FEWER
// cell evaluations).
struct FastOlrWinner
{
  int candidate = kNoGridCandidate;
  float cost = 0.0f;
};

// SHARMA ICCAD'15 Fig. 9 (sharma_et_al.md §5.2). Returns the paper's line-19
// `candidates` set: up to TWO winners per visited Vth flavor, one per
// direction. Lines 17-18 say "repeat lines 5-16 and replace maxw by minw", and
// lines 5-7 are the re-initialization while 16 is the insert - so the
// descending walk re-seeds from the same cell and inserts its own winner.
// `candidates` is a SET in the paper, so a cell both directions return appears
// once. The caller applies line 20 (argmin over the returned set, subject to
// its own local-slack guard, which on this engine is the F3 axis).
//
// A flavor whose walk never beats the incumbent contributes nothing to the
// returned set, which is the same statement as the paper's "bestcell is still
// cell(currw, Vth)" for the current flavor - the caller's argmin already starts
// at the incumbent.
//
// `valid(i)` is the caller's hard DRC filter and `cost(i)` scores candidate i;
// each runs at most once per visited member per flavor. Neither may have side
// effects.
//
// FOUR readings this implementation fixes, all recorded because the paper's
// figure under-specifies them (sharma_et_al.md §13 items 5-6):
//
//  1. THE WALK STARTS ONE STEP OUT, not at `currw`. Read literally, Fig. 9 line
//     8 begins its ascending walk at `w = currw` while line 6 has already set
//     `bestcost = cost(cell(currw, Vth))`, so the very first comparison is a
//     cell against itself, fails `<`, and hits the `else break` - the walk
//     would be dead in both directions and Fast-OLR would degenerate to
//     "evaluate the three cells at the current width". That contradicts the
//     paper's own measurement: Fig. 10 (§V-A) reports 3.3x fewer cell
//     evaluations, which over a ~30-option library is ~9 evaluations, not 3.
//     SOURCING (corrected 2026-08-09, re-audit delta T8): the 3.3x is Sharma's;
//     the 30-option figure is NOT - Sharma never prints its library's option
//     count, and 30 is the ISPD "10 sizes x 3 Vth" library recorded in
//     mangiras_et_al.md §4, used here only to turn the paper's ratio into a
//     count. The reading does not rest on it: the literal walk is degenerate at
//     every library size, and the arithmetic is corroboration. So the intended
//     walk is currw+1..maxw and currw-1..minw, which is what this does.
//  2. AN INVALID CELL IS SKIPPED, NOT A STOP. Line 10's "Ensure c is valid" is
//     read as a guard clause (continue), so one hole in a ragged grid - or one
//     cap-limited cell - cannot truncate the descent. The alternative reading
//     (break on invalid) can only see FEWER cells than the paper intends, so
//     this is the conservative choice.
//  3. AN INVALID SEED DOES NOT SET THE WALK'S THRESHOLD EITHER. Lines 5-7 have
//     no validity test at all, because the paper's grid is rectangular and
//     complete and it never contemplates an unusable seed; taken literally, a
//     DRC-rejected seed would install a `bestcost` no reachable cell has to
//     beat and could eliminate a whole Vth flavor from the search. The walk
//     therefore starts from the INCUMBENT's cost when its seed is invalid,
//     which is the same rule as reading 2 applied to the seed (an invalid cell
//     never moves bestcost anywhere) and, like it, can only widen the search.
//     The incumbent is always a legitimate origin: it is `cell(currw, curVth)`,
//     which is in the paper's own line-20 candidate set.
//  4. A RAGGED GRID CLAMPS. Line 5's cell(currw, Vth) presumes a rectangular
//     width x Vth grid (§13 item 6 says handling of missing combinations is
//     unstated). When a neighbouring flavor is shorter than `cur.width_rank`,
//     the walk starts from that flavor's nearest existing rank rather than
//     dropping the flavor - a flavor that exists is a flavor the paper
//     searches.
std::vector<FastOlrWinner> fastOlrCandidates(
    const GridLayout& layout,
    GridCoord cur,
    float cur_cost,
    const std::function<bool(int)>& valid,
    const std::function<float(int)>& cost);

// MANGIRAS Technologies (MDPI) 2021, 9, 92, §4.3 (mangiras_et_al.md §8; the
// paper is the journal extension of MOCAST'21, not a TCAD paper): the
// restricted move set - "each gate may move only to its next bigger or next
// smaller size (+-1 size step) while Vth swaps remain unrestricted". So the
// band is on the WIDTH rank while
// the flavor is free, which is what keeps a pure Vth swap (same rank, other
// flavor) legal.
//
// RAGGED COLUMNS CLAMP, exactly as reading 4 above. The paper's library is a
// rectangular 10 sizes x 3 Vth, so "the same size in another flavor" is always
// defined; ours need not be. Comparing raw per-flavor ranks across a ragged
// grid would make the second half of the rule FALSE - a gate at rank 5 of an
// 8-deep flavor could reach nothing at all in a 3-deep one, so it could never
// swap Vth again for the whole run - so the incumbent's rank is first clamped
// into the candidate's own column. Same resolution, same reason, and it is the
// only one under which "Vth swaps remain unrestricted" survives a library whose
// flavors have different depths.
bool withinSizeStep(const GridLayout& layout, GridCoord cur, GridCoord cand);

}  // namespace rsz
