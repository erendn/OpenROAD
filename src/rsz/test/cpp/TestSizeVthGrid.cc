// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

// Unit tests for the F4 axis (LRS candidate move set): the (width rank x Vth
// flavor) grid of lr/SizeVthGrid.hh and the two restricted move sets that index
// it - Sharma ICCAD'15 Fig. 9 Fast-OLR and Mangiras Technologies'21 §4.3's +-1
// size step. The STA-facing half (what the swappable group is, each member's
// flavor key and leakage-equivalent rank key, candidate cost, the DRC filter
// and the F3 guard) is exercised by the global_sizing_move_set and
// global_sizing_preset integration tests.
//
// The groups below ARE the "constructed libraries": each member carries only
// the three things the core reads - an opaque Vth-flavor key, the rank key, and
// a name for the final tie-break.
//
// What is deliberately pinned here:
//   * The decomposition is a function of the group's MEMBERS, never of the
//     order the library handed them over in, and flavors are ordered by mean
//     rank key (the least-leaky flavor first) rather than by key value.
//   * A single-flavor library - which is nangate45 and most smoke libraries,
//     where no master carries an implant-layer signature - is one column, and
//     both move sets behave sensibly on it rather than degenerating to empty.
//   * Fast-OLR's descent arithmetic against costs computed by hand from Fig. 9:
//     where it stops, that it returns up to two winners per flavor, that an
//     invalid cell is skipped rather than ending the walk, and that a
//     non-improving flavor contributes nothing.
//   * The +-1 band is a WIDTH-rank band across all flavors, so a pure Vth swap
//     at the same rank stays legal - that is what "Vth swaps remain
//     unrestricted" means.

#include <algorithm>
#include <cstddef>
#include <vector>

#include "gtest/gtest.h"
#include "lr/SizeVthGrid.hh"

namespace rsz {
namespace {

GridMember member(const char* name, const int vth_key, const float rank_key)
{
  GridMember m;
  m.vth_key = vth_key;
  m.rank_key = rank_key;
  m.name = name;
  return m;
}

// A rectangular 3-width x 2-flavor library, handed over in a deliberately
// scrambled order. Flavor 7 ("HVT") is uniformly cheaper than flavor 3 ("LVT"),
// so the decomposition must put 7 at index 0 even though its key is larger -
// the order is by mean rank key, not by the opaque flavor identity.
std::vector<GridMember> rectangularGroup()
{
  return {
      member("lvt_mid", 3, 20.0f),
      member("hvt_big", 7, 3.0f),
      member("lvt_big", 3, 30.0f),
      member("hvt_small", 7, 1.0f),
      member("lvt_small", 3, 10.0f),
      member("hvt_mid", 7, 2.0f),
  };
}

GridCoord coordOf(const std::vector<GridMember>& group,
                  const std::vector<GridCoord>& coords,
                  const char* name)
{
  for (size_t i = 0; i < group.size(); ++i) {
    if (group[i].name == name) {
      return coords[i];
    }
  }
  return {.flavor = -1, .width_rank = -1};
}

////////////////////////////////////////////////////////////////
// Grid construction

TEST(SizeVthGrid, RectangularGroupDecomposes)
{
  const std::vector<GridMember> group = rectangularGroup();
  const std::vector<GridCoord> coords = buildSizeVthGrid(group);

  // Flavor 7's mean key is 2.0, flavor 3's is 20.0, so 7 is flavor 0.
  EXPECT_EQ(coordOf(group, coords, "hvt_small").flavor, 0);
  EXPECT_EQ(coordOf(group, coords, "hvt_mid").flavor, 0);
  EXPECT_EQ(coordOf(group, coords, "hvt_big").flavor, 0);
  EXPECT_EQ(coordOf(group, coords, "lvt_small").flavor, 1);
  EXPECT_EQ(coordOf(group, coords, "lvt_mid").flavor, 1);
  EXPECT_EQ(coordOf(group, coords, "lvt_big").flavor, 1);

  // Width ranks are per-flavor and ascend with the rank key.
  EXPECT_EQ(coordOf(group, coords, "hvt_small").width_rank, 0);
  EXPECT_EQ(coordOf(group, coords, "hvt_mid").width_rank, 1);
  EXPECT_EQ(coordOf(group, coords, "hvt_big").width_rank, 2);
  EXPECT_EQ(coordOf(group, coords, "lvt_small").width_rank, 0);
  EXPECT_EQ(coordOf(group, coords, "lvt_mid").width_rank, 1);
  EXPECT_EQ(coordOf(group, coords, "lvt_big").width_rank, 2);
}

// The grid must be a function of the members, not of the order they arrive in:
// Resizer::getSwappableCells returns an unstably-sorted vector with the
// as-given cell prepended, so the same group reaches this core in different
// orders on different gates of the same design.
TEST(SizeVthGrid, InputOrderDoesNotMoveTheGrid)
{
  std::vector<GridMember> group = rectangularGroup();
  const std::vector<GridCoord> reference = buildSizeVthGrid(group);

  std::vector<GridMember> rotated = group;
  for (size_t shift = 1; shift < group.size(); ++shift) {
    std::rotate(rotated.begin(), rotated.begin() + 1, rotated.end());
    const std::vector<GridCoord> coords = buildSizeVthGrid(rotated);
    for (const GridMember& m : group) {
      const GridCoord want = coordOf(group, reference, m.name.data());
      const GridCoord got = coordOf(rotated, coords, m.name.data());
      EXPECT_EQ(got.flavor, want.flavor) << m.name << " shift " << shift;
      EXPECT_EQ(got.width_rank, want.width_rank)
          << m.name << " shift " << shift;
    }
  }
}

// Equal rank keys inside a flavor are broken by cell name, so the ranking is a
// total order and two members can never share a (flavor, width_rank) slot.
TEST(SizeVthGrid, TiedRankKeysBreakOnName)
{
  const std::vector<GridMember> group = {
      member("zz", 0, 5.0f),
      member("aa", 0, 5.0f),
      member("mm", 0, 5.0f),
  };
  const std::vector<GridCoord> coords = buildSizeVthGrid(group);
  EXPECT_EQ(coordOf(group, coords, "aa").width_rank, 0);
  EXPECT_EQ(coordOf(group, coords, "mm").width_rank, 1);
  EXPECT_EQ(coordOf(group, coords, "zz").width_rank, 2);
}

// nangate45 and the smoke libraries: no master carries an implant-layer
// signature, so cellVTType hands every cell the same key and the whole group is
// one column. Fast-OLR then searches that column alone and the +-1 band is a
// plain size step - both must still work.
TEST(SizeVthGrid, SingleFlavorLibraryIsOneColumn)
{
  const std::vector<GridMember> group = {
      member("x4", 0, 4.0f),
      member("x1", 0, 1.0f),
      member("x2", 0, 2.0f),
  };
  const std::vector<GridCoord> coords = buildSizeVthGrid(group);
  for (const GridCoord& c : coords) {
    EXPECT_EQ(c.flavor, 0);
  }
  EXPECT_EQ(coordOf(group, coords, "x1").width_rank, 0);
  EXPECT_EQ(coordOf(group, coords, "x2").width_rank, 1);
  EXPECT_EQ(coordOf(group, coords, "x4").width_rank, 2);
}

TEST(SizeVthGrid, SingleMemberAndEmptyGroup)
{
  const std::vector<GridMember> one = {member("only", 5, 42.0f)};
  const std::vector<GridCoord> coords = buildSizeVthGrid(one);
  ASSERT_EQ(coords.size(), 1u);
  EXPECT_EQ(coords[0].flavor, 0);
  EXPECT_EQ(coords[0].width_rank, 0);

  EXPECT_TRUE(buildSizeVthGrid({}).empty());
}

// A flavor with one very cheap member and one very expensive one can have a
// higher MEAN than a flavor whose members are all mid-range; the ordering key
// is the mean and this pins that reading rather than "the cheapest member".
TEST(SizeVthGrid, FlavorOrderIsByMeanRankKey)
{
  const std::vector<GridMember> group = {
      member("spread_lo", 1, 1.0f),
      member("spread_hi", 1, 99.0f),  // flavor 1 mean = 50
      member("mid_a", 2, 40.0f),
      member("mid_b", 2, 42.0f),  // flavor 2 mean = 41
  };
  const std::vector<GridCoord> coords = buildSizeVthGrid(group);
  EXPECT_EQ(coordOf(group, coords, "mid_a").flavor, 0);
  EXPECT_EQ(coordOf(group, coords, "spread_lo").flavor, 1);
}

////////////////////////////////////////////////////////////////
// Sharma Fig. 9 Fast-OLR

// A one-column harness: `costs[i]` is candidate i's cost and the incumbent sits
// at `cur_rank`, which is the hole in the column. Candidate i's width rank is
// its index for i < cur_rank and index+1 above it, so the column reads
// costs[0..n] with the incumbent spliced in at cur_rank.
struct Column
{
  std::vector<GridCoord> coords;
  std::vector<float> costs;
  GridCoord cur;
};

// The winners' candidate indices, in return order.
std::vector<int> indices(const std::vector<FastOlrWinner>& winners)
{
  std::vector<int> out;
  out.reserve(winners.size());
  for (const FastOlrWinner& w : winners) {
    out.push_back(w.candidate);
  }
  return out;
}

Column singleFlavorColumn(const std::vector<float>& candidate_costs,
                          const int cur_rank)
{
  Column column;
  column.cur = {.flavor = 0, .width_rank = cur_rank};
  for (int i = 0; i < static_cast<int>(candidate_costs.size()); ++i) {
    const int rank = (i < cur_rank) ? i : i + 1;
    column.coords.push_back({.flavor = 0, .width_rank = rank});
    column.costs.push_back(candidate_costs[i]);
  }
  return column;
}

std::vector<FastOlrWinner> runFastOlr(const Column& column,
                                      const float cur_cost,
                                      const std::vector<int>& invalid = {})
{
  return fastOlrCandidates(
      buildGridLayout(column.coords, column.cur),
      column.cur,
      cur_cost,
      [&invalid](const int i) {
        return std::find(invalid.begin(), invalid.end(), i) == invalid.end();
      },
      [&column](const int i) { return column.costs[i]; });
}

// The free-form harness: coords + costs given directly, no incumbent splice.
std::vector<FastOlrWinner> runFastOlrOn(const std::vector<GridCoord>& coords,
                                        const GridCoord cur,
                                        const float cur_cost,
                                        const std::vector<float>& costs,
                                        const std::vector<int>& invalid = {})
{
  return fastOlrCandidates(
      buildGridLayout(coords, cur),
      cur,
      cur_cost,
      [&invalid](const int i) {
        return std::find(invalid.begin(), invalid.end(), i) == invalid.end();
      },
      [&costs](const int i) { return costs[i]; });
}

// The descent stops at the first non-improving step. Column costs, incumbent
// spliced in at rank 2:
//   rank  0    1    2(inc) 3    4    5
//   cost  9.0  7.0  10.0   6.0  5.0  8.0
// Up from rank 3: 6.0 < 10.0 (take), 5.0 < 6.0 (take), 8.0 > 5.0 -> break, so
// the up-winner is the rank-4 cell (candidate index 3).
// Down from rank 1: 7.0 < 10.0 (take), 9.0 > 7.0 -> break, so the down-winner
// is the rank-1 cell (candidate index 1). Both survive: they are two DIFFERENT
// candidates, and Fig. 9 lines 17-18 insert one winner per direction.
TEST(FastOlr, DescendsUntilTheFirstNonImprovingStep)
{
  const Column column = singleFlavorColumn({9.0f, 7.0f, 6.0f, 5.0f, 8.0f}, 2);
  const std::vector<FastOlrWinner> winners
      = runFastOlr(column, /*cur_cost=*/10.0f);
  ASSERT_EQ(winners.size(), 2u);
  // Order: the ascending walk is inserted first.
  EXPECT_EQ(winners[0].candidate, 3);  // rank 4, cost 5.0
  EXPECT_EQ(winners[1].candidate, 1);  // rank 1, cost 7.0
  // The descent hands back the cost it already paid, so the caller's argmin
  // does not re-run the cost model on a cell the walk just evaluated.
  EXPECT_FLOAT_EQ(winners[0].cost, 5.0f);
  EXPECT_FLOAT_EQ(winners[1].cost, 7.0f);
}

// The rank-5 cell at cost 1.0 is GLOBALLY optimal but sits behind a rise at
// rank 4, so the descent never reaches it. That is the whole point of Fig. 9:
// local search makes incremental changes instead of jumping to the global
// argmin, which is what stops it from destabilizing a nearly-settled solution
// late in a 160-iteration run (the paper's Fig. 11).
TEST(FastOlr, DoesNotJumpOverARiseToTheGlobalOptimum)
{
  const Column column = singleFlavorColumn({9.0f, 7.0f, 6.0f, 8.0f, 1.0f}, 2);
  const std::vector<int> winners = indices(runFastOlr(column, 10.0f));
  // Up: 6.0 < 10.0 (take rank 3), then 8.0 > 6.0 -> break. Down: 7.0 (take).
  ASSERT_EQ(winners.size(), 2u);
  EXPECT_EQ(winners[0], 2);
  EXPECT_EQ(winners[1], 1);
  EXPECT_TRUE(std::find(winners.begin(), winners.end(), 4) == winners.end());
}

// A flavor whose walk never beats the incumbent contributes nothing: the
// caller's argmin already starts at the incumbent, which is Fig. 9's
// "bestcell is still cell(currw, Vth)".
TEST(FastOlr, NoImprovementYieldsNoCandidate)
{
  const Column column = singleFlavorColumn({20.0f, 15.0f, 12.0f, 30.0f}, 2);
  EXPECT_TRUE(runFastOlr(column, /*cur_cost=*/1.0f).empty());
}

// Fig. 9 line 10 is read as a guard clause: an invalid cell is skipped, so it
// neither moves bestcost nor ends the walk. Here rank 3 (candidate 2, cost 6.0)
// is invalid; the walk continues to rank 4 (candidate 3, cost 5.0) and takes it
// - a break-on-invalid reading would have returned nothing upward.
TEST(FastOlr, InvalidCellIsSkippedNotAStop)
{
  const Column column = singleFlavorColumn({9.0f, 7.0f, 6.0f, 5.0f, 8.0f}, 2);
  const std::vector<int> winners
      = indices(runFastOlr(column, /*cur_cost=*/10.0f, /*invalid=*/{2}));
  ASSERT_EQ(winners.size(), 2u);
  EXPECT_EQ(winners[0], 3);
  EXPECT_EQ(winners[1], 1);
}

// Header reading 3: an INVALID seed does not set the walk's threshold either -
// the descent then measures against the incumbent. Fig. 9 lines 5-7 have no
// validity test (its grid is rectangular and complete, so it never contemplates
// an unusable seed); taken literally, a DRC-rejected seed would install a
// bestcost no reachable cell has to beat and silently eliminate a whole Vth
// flavor. Flavor 1's seed (candidate 2, rank 1, cost 4.0) is invalid here, so
// the walk starts from the incumbent's 10.0 and the rank-2 cell at 6.0 DOES
// improve on it. The literal reading would have returned nothing.
TEST(FastOlr, InvalidSeedDoesNotTruncateItsFlavor)
{
  std::vector<GridCoord> coords = {
      {.flavor = 0, .width_rank = 0},  // 0: incumbent's flavor, below it
      {.flavor = 0, .width_rank = 2},  // 1: incumbent's flavor, above it
      {.flavor = 1, .width_rank = 1},  // 2: other flavor, the seed
      {.flavor = 1, .width_rank = 2},  // 3: other flavor, one up
  };
  const std::vector<float> costs = {11.0f, 12.0f, 4.0f, 6.0f};
  const GridCoord cur = {.flavor = 0, .width_rank = 1};
  const std::vector<int> winners
      = indices(runFastOlrOn(coords, cur, 10.0f, costs, /*invalid=*/{2}));
  ASSERT_EQ(winners.size(), 1u);
  EXPECT_EQ(winners[0], 3);  // the rank-2 cell at 6.0, reachable again

  // With the same seed VALID, it wins its flavor outright and its cheaper cost
  // then correctly stops the walk at rank 2 (6.0 does not beat 4.0).
  const std::vector<FastOlrWinner> valid_winners
      = runFastOlrOn(coords, cur, 10.0f, costs);
  ASSERT_EQ(valid_winners.size(), 1u);
  EXPECT_EQ(valid_winners[0].candidate, 2);
  EXPECT_FLOAT_EQ(valid_winners[0].cost, 4.0f);
}

// Fig. 9 line 4 searches the current flavor and its two neighbours - and only
// those. Flavor 3 is two steps away from the incumbent's flavor 1 and must not
// be visited however cheap it is.
TEST(FastOlr, SearchesOnlyTheAdjacentVthFlavors)
{
  const std::vector<GridCoord> coords = {
      {.flavor = 0, .width_rank = 0},
      {.flavor = 2, .width_rank = 0},
      {.flavor = 3, .width_rank = 0},
  };
  const std::vector<float> costs = {5.0f, 4.0f, 0.001f};
  const std::vector<int> winners = indices(
      runFastOlrOn(coords, {.flavor = 1, .width_rank = 0}, 10.0f, costs));
  ASSERT_EQ(winners.size(), 2u);
  EXPECT_TRUE(std::find(winners.begin(), winners.end(), 0) != winners.end());
  EXPECT_TRUE(std::find(winners.begin(), winners.end(), 1) != winners.end());
  EXPECT_TRUE(std::find(winners.begin(), winners.end(), 2) == winners.end());
}

// A ragged grid clamps rather than dropping the flavor: flavor 1 has only two
// members (ranks 0 and 1) while the incumbent sits at rank 4, so flavor 1's
// walk starts from its topmost member instead of being skipped.
TEST(FastOlr, ShortNeighbourFlavorClampsToItsTopRank)
{
  const std::vector<GridCoord> coords = {
      {.flavor = 1, .width_rank = 0},
      {.flavor = 1, .width_rank = 1},
  };
  const std::vector<float> costs = {8.0f, 3.0f};
  const std::vector<int> winners = indices(
      runFastOlrOn(coords, {.flavor = 0, .width_rank = 4}, 10.0f, costs));
  // Seed at the clamped rank 1 (cost 3.0) wins; the downward walk re-seeds from
  // the same cell and 8.0 does not improve on 3.0, so there is one winner.
  ASSERT_EQ(winners.size(), 1u);
  EXPECT_EQ(winners[0], 1);
}

// Both directions return the same cell when neither improves on the seed. The
// paper's `candidates` is a SET, so the winner appears once.
TEST(FastOlr, BothDirectionsReturningTheSeedYieldOneCandidate)
{
  const std::vector<GridCoord> coords = {
      {.flavor = 1, .width_rank = 0},
      {.flavor = 1, .width_rank = 1},
      {.flavor = 1, .width_rank = 2},
  };
  const std::vector<float> costs = {9.0f, 2.0f, 9.0f};
  const std::vector<int> winners = indices(
      runFastOlrOn(coords, {.flavor = 0, .width_rank = 1}, 10.0f, costs));
  ASSERT_EQ(winners.size(), 1u);
  EXPECT_EQ(winners[0], 1);
}

TEST(FastOlr, EmptyCandidateSetIsEmpty)
{
  EXPECT_TRUE(
      runFastOlrOn({}, {.flavor = 0, .width_rank = 0}, 1.0f, {}).empty());
}

////////////////////////////////////////////////////////////////
// Mangiras §4.3 +-1 size step

// A rectangular layout, where the clamp is inert and the band reads directly.
GridLayout rectangularLayout(const int flavors, const int height)
{
  std::vector<GridCoord> coords;
  for (int f = 0; f < flavors; ++f) {
    for (int r = 0; r < height; ++r) {
      coords.push_back({.flavor = f, .width_rank = r});
    }
  }
  return buildGridLayout(coords, {.flavor = 0, .width_rank = 0});
}

TEST(SizeStep, BandIsOnWidthRankAcrossEveryFlavor)
{
  const GridLayout layout = rectangularLayout(/*flavors=*/10, /*height=*/8);
  const GridCoord cur = {.flavor = 1, .width_rank = 3};
  // Same flavor, adjacent ranks.
  EXPECT_TRUE(withinSizeStep(layout, cur, {.flavor = 1, .width_rank = 2}));
  EXPECT_TRUE(withinSizeStep(layout, cur, {.flavor = 1, .width_rank = 4}));
  // A pure Vth swap - same rank, other flavor - is legal: "Vth swaps remain
  // unrestricted".
  EXPECT_TRUE(withinSizeStep(layout, cur, {.flavor = 0, .width_rank = 3}));
  EXPECT_TRUE(withinSizeStep(layout, cur, {.flavor = 9, .width_rank = 3}));
  // A step in both at once is still one SIZE step, so it is legal too.
  EXPECT_TRUE(withinSizeStep(layout, cur, {.flavor = 0, .width_rank = 4}));
  // Two size steps is not, however near the flavor.
  EXPECT_FALSE(withinSizeStep(layout, cur, {.flavor = 1, .width_rank = 1}));
  EXPECT_FALSE(withinSizeStep(layout, cur, {.flavor = 1, .width_rank = 5}));
  EXPECT_FALSE(withinSizeStep(layout, cur, {.flavor = 0, .width_rank = 0}));
}

// THE RAGGED-COLUMN CASE, and the reason the band takes a layout at all. Width
// ranks are numbered independently per flavor, so on a library whose flavors
// have different depths a raw rank comparison makes the rule's second half
// FALSE: a gate high up a deep column could reach nothing at all in a shallow
// one and could therefore never swap Vth again for the whole run. The
// incumbent's rank is clamped into the candidate's own column first - the same
// resolution Fast-OLR's seed takes - so every flavor stays reachable.
TEST(SizeStep, ShallowFlavorStaysReachableFromADeepColumn)
{
  std::vector<GridCoord> coords;
  for (int r = 0; r < 8; ++r) {
    coords.push_back({.flavor = 0, .width_rank = r});  // deep flavor
  }
  for (int r = 0; r < 3; ++r) {
    coords.push_back({.flavor = 1, .width_rank = r});  // shallow flavor
  }
  const GridLayout layout
      = buildGridLayout(coords, {.flavor = 0, .width_rank = 0});
  const GridCoord cur = {.flavor = 0, .width_rank = 5};

  // Its own column bands normally.
  EXPECT_TRUE(withinSizeStep(layout, cur, {.flavor = 0, .width_rank = 4}));
  EXPECT_TRUE(withinSizeStep(layout, cur, {.flavor = 0, .width_rank = 6}));
  EXPECT_FALSE(withinSizeStep(layout, cur, {.flavor = 0, .width_rank = 3}));

  // The shallow flavor tops out at rank 2, so the reference clamps to 2 and its
  // top two cells stay reachable. Without the clamp the gate could reach NO
  // cell of flavor 1 and its Vth would be frozen for the run.
  EXPECT_TRUE(withinSizeStep(layout, cur, {.flavor = 1, .width_rank = 2}));
  EXPECT_TRUE(withinSizeStep(layout, cur, {.flavor = 1, .width_rank = 1}));
  EXPECT_FALSE(withinSizeStep(layout, cur, {.flavor = 1, .width_rank = 0}));

  // A flavor with no members at all is not reachable, clamp or no clamp.
  EXPECT_FALSE(withinSizeStep(layout, cur, {.flavor = 2, .width_rank = 0}));
}

// The band applied to a real decomposition, end to end.
TEST(SizeStep, AppliedToADecomposedGroup)
{
  const std::vector<GridMember> group = {
      member("hvt_0", 7, 1.0f),
      member("hvt_1", 7, 2.0f),
      member("hvt_2", 7, 3.0f),
      member("hvt_3", 7, 4.0f),
      member("lvt_0", 3, 10.0f),
      member("lvt_3", 3, 40.0f),
  };
  const std::vector<GridCoord> coords = buildSizeVthGrid(group);
  const GridCoord cur = coordOf(group, coords, "hvt_1");
  const GridLayout layout = buildGridLayout(
      std::vector<GridCoord>(coords.begin() + 1, coords.end()), coords[0]);

  EXPECT_TRUE(withinSizeStep(layout, cur, coordOf(group, coords, "hvt_0")));
  EXPECT_TRUE(withinSizeStep(layout, cur, coordOf(group, coords, "hvt_2")));
  EXPECT_FALSE(withinSizeStep(layout, cur, coordOf(group, coords, "hvt_3")));
  // The LVT column has only two members, so both are within one step of the
  // clamped reference - the restriction removes nothing there.
  EXPECT_TRUE(withinSizeStep(layout, cur, coordOf(group, coords, "lvt_0")));
  EXPECT_TRUE(withinSizeStep(layout, cur, coordOf(group, coords, "lvt_3")));
}

////////////////////////////////////////////////////////////////
// GridLayout

// The layout reserves the incumbent's slot even though it is not a candidate,
// so "one size up from here" has a referent and the hole reads back as such.
TEST(GridLayout, ReservesTheIncumbentSlot)
{
  const std::vector<GridCoord> coords = {
      {.flavor = 0, .width_rank = 0},
      {.flavor = 0, .width_rank = 2},
  };
  const GridLayout layout
      = buildGridLayout(coords, {.flavor = 0, .width_rank = 1});
  EXPECT_EQ(layout.flavorCount(), 1);
  EXPECT_EQ(layout.columnHeight(0), 3);
  EXPECT_EQ(layout.at(0, 0), 0);
  EXPECT_EQ(layout.at(0, 1), kNoGridCandidate);  // the incumbent
  EXPECT_EQ(layout.at(0, 2), 1);
  // Out of range in either direction.
  EXPECT_EQ(layout.at(0, 3), kNoGridCandidate);
  EXPECT_EQ(layout.at(0, -1), kNoGridCandidate);
  EXPECT_EQ(layout.at(1, 0), kNoGridCandidate);
  EXPECT_EQ(layout.columnHeight(1), 0);
  EXPECT_EQ(layout.columnHeight(-1), 0);
}

// A flavor the incumbent is not in and no candidate reaches leaves a genuine
// gap in the column range; heights must count only what exists.
TEST(GridLayout, EmptyFlavorHasZeroHeight)
{
  const std::vector<GridCoord> coords = {{.flavor = 2, .width_rank = 0}};
  const GridLayout layout
      = buildGridLayout(coords, {.flavor = 0, .width_rank = 0});
  EXPECT_EQ(layout.flavorCount(), 3);
  EXPECT_EQ(layout.columnHeight(0), 1);  // the incumbent's own
  EXPECT_EQ(layout.columnHeight(1), 0);
  EXPECT_EQ(layout.columnHeight(2), 1);
}

}  // namespace
}  // namespace rsz
