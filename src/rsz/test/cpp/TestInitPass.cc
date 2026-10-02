// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

// Unit tests for the initial-solution modes (-init_mode): the per-mode
// selection rules in lr/InitSelect.hh, using hand-built groups of swappable
// cells. The STA side of the pass (which instances are editable, excluding the
// clock network, dont-touch, and applying the swap) is covered by the
// global_sizing_init_* and global_sizing_dont_touch integration tests.
//
// Each group below is a hand-built library: a list of candidates with the two
// keys the selector ranks by (cell leakage first, drive resistance as the
// tie-break) plus a name for the final tie-break. group[0] is always the
// current cell, as InitPass builds it.
//
// The tests check that:
//   * min_size and max_size pick the lowest- and highest-leakage members; the
//     chen and livramento presets depend on this.
//   * random depends only on (init_seed, instance name), not on iteration
//     order, thread count, or the placement seed.
//   * average is the lower median of the ranking, which for a group with no
//     full ties lies between the min_size and max_size picks.
//   * min_size_fixviol's repair walk climbs the same ranking and stops at the
//     cheapest member that clears the gate's electrical violation, leaving the
//     gate at minimum size when no member does.
//   * the slew fix pass climbs the same ranking, skips members it may not
//     install, and without a clearing member takes the one closest to the
//     slew limit.

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "lr/InitSelect.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "utl/Logger.h"

namespace rsz {
namespace {

using InitMode = GlobalSizingConfig::InitMode;
using LambdaSeed = GlobalSizingConfig::LambdaSeed;
using Preset = GlobalSizingConfig::Preset;

InitCandidate cell(const char* name,
                   const float leakage,
                   const float drive_resistance = 1.0f)
{
  InitCandidate c;
  c.has_leakage = true;
  c.leakage = leakage;
  c.drive_resistance = drive_resistance;
  c.name = name;
  return c;
}

// A five-member group with distinct leakages, listed in an order that is
// neither ascending nor descending, and whose current cell (index 0) is
// neither the min nor the max. Ranked ascending: E(1) C(2) A(4) D(8) B(16).
std::vector<InitCandidate> group5()
{
  return {cell("A", 4.0f),
          cell("B", 16.0f),
          cell("C", 2.0f),
          cell("D", 8.0f),
          cell("E", 1.0f)};
}

////////////////////////////////////////////////////////////////
// min_size / max_size

TEST(InitSelect, MinSizePicksTheLowestLeakageMember)
{
  EXPECT_EQ(selectInitCandidate(group5(), InitMode::kMinSize, 0), 4u);
}

TEST(InitSelect, MaxSizePicksTheHighestLeakageMember)
{
  EXPECT_EQ(selectInitCandidate(group5(), InitMode::kMaxSize, 0), 1u);
}

// The leakage tie-break, in both directions: equal leakage is resolved by drive
// resistance, and the two modes use it in opposite ways (min_size wants the
// weak driver, max_size the strong one). `average`'s ranking uses the same
// secondary key.
TEST(InitSelect, LeakageTiesBreakOnDriveResistance)
{
  const std::vector<InitCandidate> group = {cell("A", 4.0f, 2.0f),
                                            cell("weak", 4.0f, 9.0f),
                                            cell("strong", 4.0f, 0.5f)};
  EXPECT_EQ(selectInitCandidate(group, InitMode::kMinSize, 0), 1u);
  EXPECT_EQ(selectInitCandidate(group, InitMode::kMaxSize, 0), 2u);
}

// A candidate whose Liberty cell has no leakage cannot be ranked, so the
// deterministic modes skip it, even when it is the current cell; the first
// rankable candidate then replaces it.
TEST(InitSelect, UnrankableCandidatesAreSkipped)
{
  InitCandidate unrankable;
  unrankable.name = "no_leakage";
  std::vector<InitCandidate> group = {unrankable, cell("A", 4.0f)};
  EXPECT_EQ(selectInitCandidate(group, InitMode::kMinSize, 0), 1u);
  EXPECT_EQ(selectInitCandidate(group, InitMode::kMaxSize, 0), 1u);
  EXPECT_EQ(selectInitCandidate(group, InitMode::kAverage, 0), 1u);

  // ...and when nothing can be ranked, every deterministic mode keeps the
  // current cell rather than picking arbitrarily.
  group = {unrankable, unrankable};
  EXPECT_EQ(selectInitCandidate(group, InitMode::kMinSize, 0), 0u);
  EXPECT_EQ(selectInitCandidate(group, InitMode::kMaxSize, 0), 0u);
  EXPECT_EQ(selectInitCandidate(group, InitMode::kAverage, 0), 0u);
}

// n = 1: an instance with no swappable equivalents keeps its cell under every
// mode, random included.
TEST(InitSelect, SingletonGroupsAreAlwaysTheAsGivenCell)
{
  const std::vector<InitCandidate> group = {cell("only", 3.0f)};
  for (const InitMode mode : {InitMode::kMinSize,
                              InitMode::kMaxSize,
                              InitMode::kAverage,
                              InitMode::kRandom}) {
    for (const uint64_t draw : {0ULL, 1ULL, 12345ULL, ~0ULL}) {
      EXPECT_EQ(selectInitCandidate(group, mode, draw), 0u);
    }
  }
}

////////////////////////////////////////////////////////////////
// average - the lower median of the ranking

TEST(InitSelect, AverageIsTheLowerMedianOfTheRanking)
{
  // Ranked E(1) C(2) A(4) D(8) B(16); floor((5-1)/2) = 2 -> A. A happens to be
  // the current cell in this fixture; the next test shows that is not the
  // rule.
  EXPECT_EQ(selectInitCandidate(group5(), InitMode::kAverage, 0), 0u);
}

// Even n takes the lower median: with 4 members, index floor(3/2) = 1 of the
// ranking, not 2. Ranked: E(1) C(2) A(4) D(8) -> C.
TEST(InitSelect, AverageTakesTheLowerMedianAtEvenN)
{
  const std::vector<InitCandidate> group
      = {cell("A", 4.0f), cell("C", 2.0f), cell("D", 8.0f), cell("E", 1.0f)};
  EXPECT_EQ(selectInitCandidate(group, InitMode::kAverage, 0), 1u);
}

// `average` takes the median of the same ranking whose ends min_size and
// max_size pick, so on a group with no full ties the three picks are ordered
// min < average < max.
TEST(InitSelect, AverageSitsBetweenMinSizeAndMaxSize)
{
  const std::vector<InitCandidate> group = group5();
  const float min_leak
      = group[selectInitCandidate(group, InitMode::kMinSize, 0)].leakage;
  const float mid_leak
      = group[selectInitCandidate(group, InitMode::kAverage, 0)].leakage;
  const float max_leak
      = group[selectInitCandidate(group, InitMode::kMaxSize, 0)].leakage;
  EXPECT_LT(min_leak, mid_leak);
  EXPECT_LT(mid_leak, max_leak);
}

// The median depends on the group, not on the order the library lists it in.
// Same members with the tail reversed -> same cell. (Index 0 stays fixed
// because it is always the current cell.)
TEST(InitSelect, AverageIsIndependentOfGroupOrder)
{
  std::vector<InitCandidate> forward = group5();
  std::vector<InitCandidate> reversed = {forward[0]};
  for (size_t i = forward.size(); i > 1; --i) {
    reversed.push_back(forward[i - 1]);
  }
  EXPECT_EQ(
      forward[selectInitCandidate(forward, InitMode::kAverage, 0)].name,
      reversed[selectInitCandidate(reversed, InitMode::kAverage, 0)].name);
}

// ...including when the ranking keys cannot separate two candidates: the name
// is the final tie-break, so a full tie still has one answer whatever the
// order.
TEST(InitSelect, AverageBreaksFullTiesOnCellName)
{
  const std::vector<InitCandidate> ab
      = {cell("x", 9.0f), cell("a", 4.0f, 1.0f), cell("b", 4.0f, 1.0f)};
  const std::vector<InitCandidate> ba
      = {cell("x", 9.0f), cell("b", 4.0f, 1.0f), cell("a", 4.0f, 1.0f)};
  // Ranked a, b, x either way; lower median of 3 is index 1 = "b".
  EXPECT_EQ(ab[selectInitCandidate(ab, InitMode::kAverage, 0)].name, "b");
  EXPECT_EQ(ba[selectInitCandidate(ba, InitMode::kAverage, 0)].name, "b");
}

////////////////////////////////////////////////////////////////
// min_size_fixviol: the repair walk's upsize selection
//
// `clears(i)` stands in for the STA check (does group[i] leave this gate's
// output pins free of max-cap and max-slew violations?). The fixtures express
// it as a leakage threshold, which is how the real check behaves on a
// monotone library: a bigger cell has a higher cap limit and a lower drive
// resistance, so once one member clears, every stronger one does too.

// Clears at or above `min_leak`, as in a monotone library.
auto clearsFrom(const std::vector<InitCandidate>& group, const float min_leak)
{
  return [&group, min_leak](const size_t i) {
    return group[i].leakage >= min_leak;
  };
}

// The pass takes the cheapest cell that clears, not the strongest in the
// group. Taking the strongest would spend leakage the violation does not
// require, when the purpose of the pass is to stay close to the minimum-leakage
// solution.
TEST(InitFixviol, TakesTheCheapestMemberThatClears)
{
  // Ranked E(1) C(2) A(4) D(8) B(16); the current cell after the min-size
  // reset is E, the first in the ranking.
  const std::vector<InitCandidate> group = group5();
  EXPECT_EQ(selectFixviolUpsize(group, 4, clearsFrom(group, 3.0f)),
            0u);  // A(4)
  EXPECT_EQ(selectFixviolUpsize(group, 4, clearsFrom(group, 8.0f)),
            3u);  // D(8)
}

// A gate that clears at the next member takes that member and does not skip
// ahead.
TEST(InitFixviol, ClimbsOneRankAtATime)
{
  const std::vector<InitCandidate> group = group5();
  EXPECT_EQ(selectFixviolUpsize(group, 4, clearsFrom(group, 2.0f)),
            2u);  // C(2)
}

// No member of the group clears the violation, so the gate is left at minimum
// size rather than upsized to the top for nothing. The pass does not spend
// leakage on a gate it cannot fix.
TEST(InitFixviol, ExhaustionLeavesTheGateAtMinimum)
{
  const std::vector<InitCandidate> group = group5();
  EXPECT_EQ(selectFixviolUpsize(group, 4, [](size_t) { return false; }), 4u);
  // Same answer when the only clearing members are below the current cell: the
  // walk starts one rank above it, so the pass never downsizes.
  EXPECT_EQ(selectFixviolUpsize(group, 1, clearsFrom(group, 0.0f)), 1u);
}

// A gate that is not violating never reaches the selector, but if it did, a
// gate whose current cell is already the top rank has nothing above it.
TEST(InitFixviol, TheTopOfTheRankingHasNowhereToClimb)
{
  const std::vector<InitCandidate> group = group5();
  EXPECT_EQ(selectFixviolUpsize(group, 1, [](size_t) { return true; }), 1u);
}

// The walk climbs the same total order `average` uses, so the answer depends
// only on the group's members and on `clears`, never on the order the library
// lists them in.
TEST(InitFixviol, SelectionIsIndependentOfGroupOrder)
{
  const std::vector<InitCandidate> forward = group5();
  // Same five members in a different order, with the current cell E still the
  // group's minimum.
  const std::vector<InitCandidate> shuffled
      = {forward[4], forward[3], forward[0], forward[1], forward[2]};
  for (const float threshold : {1.5f, 3.0f, 5.0f, 9.0f, 20.0f}) {
    const size_t a
        = selectFixviolUpsize(forward, 4, clearsFrom(forward, threshold));
    const size_t b
        = selectFixviolUpsize(shuffled, 0, clearsFrom(shuffled, threshold));
    EXPECT_EQ(forward[a].name, shuffled[b].name) << "threshold " << threshold;
  }
}

// ...and a full tie in both ranking keys resolves on the cell name, so two
// equally good repairs still have one answer.
TEST(InitFixviol, FullTiesResolveOnCellName)
{
  const std::vector<InitCandidate> ab
      = {cell("min", 1.0f), cell("b", 4.0f, 1.0f), cell("a", 4.0f, 1.0f)};
  const std::vector<InitCandidate> ba
      = {cell("min", 1.0f), cell("a", 4.0f, 1.0f), cell("b", 4.0f, 1.0f)};
  EXPECT_EQ(ab[selectFixviolUpsize(ab, 0, clearsFrom(ab, 4.0f))].name, "a");
  EXPECT_EQ(ba[selectFixviolUpsize(ba, 0, clearsFrom(ba, 4.0f))].name, "a");
}

// Members that cannot be ranked are not repair candidates: the walk climbs the
// ranking, and a cell with no Liberty leakage has no place in it. (If the
// current cell is unrankable, there is no rank to start above, so every
// rankable member is a candidate.)
TEST(InitFixviol, UnrankableMembersAreNotRepairCandidates)
{
  InitCandidate unrankable;
  unrankable.name = "no_leakage";
  const std::vector<InitCandidate> group
      = {cell("min", 1.0f), unrankable, cell("big", 9.0f)};
  EXPECT_EQ(selectFixviolUpsize(group, 0, [](size_t) { return true; }), 2u);

  const std::vector<InitCandidate> unrankable_incumbent
      = {unrankable, cell("small", 1.0f), cell("big", 9.0f)};
  EXPECT_EQ(
      selectFixviolUpsize(unrankable_incumbent, 0, [](size_t) { return true; }),
      1u);
}

////////////////////////////////////////////////////////////////
// slew_fix_pass: the slew repair's upsize selection
//
// `slew_excess(i)` stands in for the STA side: how far group[i]'s estimated
// output slew would exceed the limit, or nullopt when group[i] may not be
// installed (it would overload a fanin driver). The fixtures give it per cell
// name; a name missing from the map may not be installed. Ranked ascending,
// group5 is E(1) C(2) A(4) D(8) B(16), and the current cell is E (index 4),
// the minimum-leakage member, at an excess of 1.

using Excesses = std::map<std::string, std::optional<float>>;

auto excessOf(const std::vector<InitCandidate>& group, const Excesses& by_name)
{
  return [&group, &by_name](const size_t i) -> std::optional<float> {
    const auto it = by_name.find(std::string(group[i].name));
    return it != by_name.end() ? it->second : std::nullopt;
  };
}

// Sharma et al. upsize minimally: of the members that clear the slew limit (A,
// D and B), the lowest-leakage one, A.
TEST(SlewFixSelect, TakesTheLowestLeakageMemberThatClears)
{
  const std::vector<InitCandidate> group = group5();
  const Excesses excess{{"C", 0.5f}, {"A", 0.0f}, {"D", 0.0f}, {"B", 0.0f}};
  EXPECT_EQ(selectSlewFixUpsize(group, 4, 1.0f, excessOf(group, excess)),
            0u);  // A
}

// A, which clears, would overload a fanin driver, so the next member that
// clears, D, is taken.
TEST(SlewFixSelect, SkipsAMemberThatMayNotBeInstalled)
{
  const std::vector<InitCandidate> group = group5();
  const Excesses excess{{"C", 0.5f}, {"D", 0.0f}, {"B", 0.0f}};
  EXPECT_EQ(selectSlewFixUpsize(group, 4, 1.0f, excessOf(group, excess)),
            3u);  // D
}

// No member clears the limit: the one closest to it, D at 0.2, is taken,
// although B ranks higher.
TEST(SlewFixSelect, WithoutAClearingMemberTakesTheSmallestExcess)
{
  const std::vector<InitCandidate> group = group5();
  const Excesses excess{{"C", 0.6f}, {"A", 0.3f}, {"D", 0.2f}, {"B", 0.25f}};
  EXPECT_EQ(selectSlewFixUpsize(group, 4, 1.0f, excessOf(group, excess)),
            3u);  // D
}

// A and D are equally close to the limit: the lower-ranked A is taken.
TEST(SlewFixSelect, EqualExcessesKeepTheLowerRankedMember)
{
  const std::vector<InitCandidate> group = group5();
  const Excesses excess{{"C", 0.5f}, {"A", 0.3f}, {"D", 0.3f}, {"B", 0.4f}};
  EXPECT_EQ(selectSlewFixUpsize(group, 4, 1.0f, excessOf(group, excess)),
            0u);  // A
}

// The current cell stays when no member that may be installed comes closer to
// the limit than its own excess, or when none may be installed.
TEST(SlewFixSelect, KeepsTheCurrentCellWhenNothingIsCloser)
{
  const std::vector<InitCandidate> group = group5();
  const Excesses farther{{"C", 1.0f}, {"A", 1.5f}, {"D", 2.0f}};
  EXPECT_EQ(selectSlewFixUpsize(group, 4, 1.0f, excessOf(group, farther)), 4u);
  EXPECT_EQ(selectSlewFixUpsize(group, 4, 1.0f, excessOf(group, {})), 4u);
}

// The pass only upsizes. From A (index 0), C and E would clear the limit but
// rank below A, so they are never asked; D is the closest member above A.
TEST(SlewFixSelect, OnlyUpsizes)
{
  const std::vector<InitCandidate> group = group5();
  std::vector<std::string> asked;
  const size_t pick = selectSlewFixUpsize(group, 0, 1.0f, [&](const size_t i) {
    asked.emplace_back(group[i].name);
    const Excesses excess{{"E", 0.0f}, {"C", 0.0f}, {"D", 0.5f}, {"B", 0.7f}};
    return excess.at(asked.back());
  });
  EXPECT_EQ(pick, 3u);  // D
  EXPECT_EQ(asked, (std::vector<std::string>{"D", "B"}));
}

// The members are asked once each, in ranking order, and the climb stops at
// the first member that clears.
TEST(SlewFixSelect, AsksInRankingOrderAndStopsAtTheFirstClear)
{
  const std::vector<InitCandidate> group = group5();
  std::vector<std::string> asked;
  const Excesses excess{{"C", 0.5f}, {"A", 0.0f}, {"D", 0.0f}, {"B", 0.0f}};
  selectSlewFixUpsize(group, 4, 1.0f, [&](const size_t i) {
    asked.emplace_back(group[i].name);
    return excess.at(asked.back());
  });
  EXPECT_EQ(asked, (std::vector<std::string>{"C", "A"}));
}

////////////////////////////////////////////////////////////////
// random: the seeded per-instance draw

// The draw is a pure function of (init_seed, instance name): the same pair
// gives the identical draw, so a run with a fixed seed is reproducible.
TEST(InitDraw, SameSeedAndInstanceReproduceTheSameDraw)
{
  EXPECT_EQ(initDraw(7, "u_alu/add_1"), initDraw(7, "u_alu/add_1"));
  EXPECT_EQ(
      selectInitCandidate(group5(), InitMode::kRandom, initDraw(7, "i0")),
      selectInitCandidate(group5(), InitMode::kRandom, initDraw(7, "i0")));
}

// Different seeds change the draw, and on a group with n > 1 they change the
// selection; otherwise init_seed would have no effect on the initial netlist.
TEST(InitDraw, DifferentSeedsSelectDifferentCells)
{
  const std::vector<InitCandidate> group = group5();
  std::set<size_t> selected;
  for (int seed = 0; seed < 32; ++seed) {
    selected.insert(
        selectInitCandidate(group, InitMode::kRandom, initDraw(seed, "i0")));
  }
  // 32 seeds over a 5-member group: every member should come up.
  EXPECT_EQ(selected.size(), group.size());
}

// Different instances under one seed also differ: the draw is per instance,
// not per run, so a single seed produces a mixed netlist rather than one
// global choice.
TEST(InitDraw, DifferentInstancesUnderOneSeedDifferToo)
{
  const std::vector<InitCandidate> group = group5();
  std::set<size_t> selected;
  for (int i = 0; i < 32; ++i) {
    const std::string inst = "u_top/inst_" + std::to_string(i);
    selected.insert(
        selectInitCandidate(group, InitMode::kRandom, initDraw(0, inst)));
  }
  EXPECT_EQ(selected.size(), group.size());
}

// The draw reads no iteration state, so visiting the same instances in any
// order (as a different instance iterator, thread count, or OpenROAD version
// might) gives the identical per-instance selection. This is the reason
// initDraw exists.
TEST(InitDraw, SelectionIsIndependentOfVisitOrder)
{
  const std::vector<InitCandidate> group = group5();
  std::vector<std::string> instances;
  instances.reserve(16);
  for (int i = 0; i < 16; ++i) {
    instances.push_back("u_top/inst_" + std::to_string(i));
  }

  std::vector<size_t> forward;
  forward.reserve(instances.size());
  for (const std::string& inst : instances) {
    forward.push_back(
        selectInitCandidate(group, InitMode::kRandom, initDraw(3, inst)));
  }
  // Same instances, visited back to front.
  for (size_t i = instances.size(); i > 0; --i) {
    EXPECT_EQ(selectInitCandidate(
                  group, InitMode::kRandom, initDraw(3, instances[i - 1])),
              forward[i - 1])
        << "instance " << instances[i - 1] << " moved with the visit order";
  }
}

// The draw indexes the group in cell-name order, so the order the library
// returns (an unstable sort by drive resistance, with the current cell
// prepended and dont-use filtering applied) cannot change the selection.
// Otherwise one `set_dont_use` elsewhere in a script would shift every index
// and silently change the draw for every instance under the same seed.
TEST(InitDraw, SelectionIsIndependentOfGroupOrder)
{
  const std::vector<InitCandidate> forward = group5();
  // Same members in a different order, including a different member at index
  // 0, as happens when the incoming netlist uses different sizes.
  const std::vector<InitCandidate> shuffled
      = {forward[2], forward[4], forward[0], forward[3], forward[1]};
  for (int seed = 0; seed < 24; ++seed) {
    const uint64_t draw = initDraw(seed, "u_top/i0");
    EXPECT_EQ(
        forward[selectInitCandidate(forward, InitMode::kRandom, draw)].name,
        shuffled[selectInitCandidate(shuffled, InitMode::kRandom, draw)].name)
        << "seed " << seed << " drew a different cell from a reordered group";
  }
}

// The draw covers the whole group, including the current cell: the mode is a
// uniform draw over the equivalence group, not over the alternatives to the
// current cell. Unrankable members are included too: the sweep costs a cell
// without leakage by scaled area rather than excluding it, so the
// deterministic modes skip such a cell only because it cannot be ranked.
TEST(InitDraw, TheAsGivenCellIsInTheDrawPopulation)
{
  const std::vector<InitCandidate> group = group5();
  bool saw_as_given = false;
  for (int seed = 0; seed < 64 && !saw_as_given; ++seed) {
    saw_as_given
        = selectInitCandidate(group, InitMode::kRandom, initDraw(seed, "i0"))
          == 0u;
  }
  EXPECT_TRUE(saw_as_given);
}

// A non-finite ranking key would make `average`'s sort comparator
// non-transitive (NaN compares false in both directions while the rest stay
// ordered), and an invalid strict weak ordering is undefined behavior in
// std::sort, not merely a wrong median. Such candidates are dropped from the
// ranking. min_size and max_size use a linear scan and are not changed, so
// they need not agree with `average` in this case.
TEST(InitSelect, AverageIgnoresNonFiniteRankingKeys)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<InitCandidate> group
      = {cell("A", 4.0f), cell("nan", nan), cell("C", 2.0f), cell("inf", inf)};
  // Ranked over the finite members only: C(2) A(4) -> lower median C.
  EXPECT_EQ(group[selectInitCandidate(group, InitMode::kAverage, 0)].name, "C");

  // A non-finite drive resistance excludes a candidate the same way.
  group = {cell("A", 4.0f), cell("bad_drive", 1.0f, nan), cell("C", 2.0f)};
  EXPECT_EQ(group[selectInitCandidate(group, InitMode::kAverage, 0)].name, "C");

  // When nothing is rankable, the current cell is kept.
  group = {cell("A", nan), cell("B", inf)};
  EXPECT_EQ(selectInitCandidate(group, InitMode::kAverage, 0), 0u);
}

////////////////////////////////////////////////////////////////
// Config: defaults, presets and the validator

TEST(InitConfig, DefaultIsAsGivenWithSeedZero)
{
  const GlobalSizingConfig config;
  EXPECT_EQ(config.init_mode, InitMode::kAsGiven);
  EXPECT_EQ(config.init_seed, 0);
  GlobalSizingConfig baseline;
  baseline.applyPreset(Preset::kRszBaseline);
  EXPECT_EQ(baseline.init_mode, InitMode::kAsGiven);
  EXPECT_EQ(baseline.init_seed, 0);
}

// The initial solution of each preset. Four papers state one:
//  - chen (Chen et al., SOLVE_LRS/μ step 1, "x_i := L_i") and livramento
//    (Livramento et al., Alg. 1 line 2) reset every gate to minimum leakage
//    and stop there; neither applies a repair pass at initialization
//    (Livramento's Alg. 3 FIX_VIOLATIONS runs every iteration, not at init).
//  - flach (Flach et al., Fig. 1a-b) resets and then repairs the electrical
//    violations the reset creates, which is min_size_fixviol.
//  - sharma (Sharma et al., ICCAD 2015, Sec. III-A) resets and repairs max
//    capacitance from outputs to inputs, which is min_size_fixcap; its slew
//    repair from inputs to outputs is slew_fix_pass.
// The other presets keep the netlist as given; chinnery does so because the
// paper resizes the netlist the flow hands it.
TEST(InitConfig, PresetInitModes)
{
  const struct
  {
    Preset preset;
    InitMode mode;
  } cases[] = {
      {Preset::kRszBaseline, InitMode::kAsGiven},
      {Preset::kChen, InitMode::kMinSize},
      {Preset::kTennakoon, InitMode::kAsGiven},
      {Preset::kFlach, InitMode::kMinSizeFixviol},
      {Preset::kSharmaSeq, InitMode::kMinSizeFixcap},
      {Preset::kReimann, InitMode::kAsGiven},
      {Preset::kMangiras, InitMode::kAsGiven},
      {Preset::kLivramento, InitMode::kMinSize},
      {Preset::kChinnery, InitMode::kAsGiven},
  };
  EXPECT_EQ(std::size(cases), std::size(kAllPresets))
      << "every preset needs a row: a table that silently omits one stops "
         "pinning its C axis";
  for (const auto& c : cases) {
    GlobalSizingConfig config;
    config.applyPreset(c.preset);
    EXPECT_EQ(config.init_mode, c.mode)
        << "preset " << toString(c.preset) << " init_mode";
  }
  // No preset draws randomly, so none of them reads init_seed.
  for (const Preset p : kAllPresets) {
    GlobalSizingConfig config;
    config.applyPreset(p);
    EXPECT_EQ(config.init_seed, 0) << "preset " << toString(p);
  }
}

// Whether each preset sizes registers. None of the papers resizes registers,
// so every paper preset turns it off; the default and rsz_baseline keep it on.
TEST(InitConfig, PresetRegisterSizing)
{
  EXPECT_TRUE(GlobalSizingConfig{}.size_registers);
  const struct
  {
    Preset preset;
    bool size_registers;
  } cases[] = {
      {Preset::kRszBaseline, true},
      {Preset::kChen, false},
      {Preset::kTennakoon, false},
      {Preset::kFlach, false},
      {Preset::kSharmaSeq, false},
      {Preset::kReimann, false},
      {Preset::kMangiras, false},
      {Preset::kLivramento, false},
      {Preset::kChinnery, false},
  };
  EXPECT_EQ(std::size(cases), std::size(kAllPresets))
      << "every preset needs a row";
  for (const auto& c : cases) {
    GlobalSizingConfig config;
    config.applyPreset(c.preset);
    EXPECT_EQ(config.size_registers, c.size_registers)
        << "preset " << toString(c.preset);
  }
}

TEST(InitConfig, ParseRoundTrip)
{
  const struct
  {
    const char* name;
    InitMode mode;
  } cases[] = {{"as_given", InitMode::kAsGiven},
               {"min_size", InitMode::kMinSize},
               {"max_size", InitMode::kMaxSize},
               {"min_size_fixviol", InitMode::kMinSizeFixviol},
               {"min_size_fixcap", InitMode::kMinSizeFixcap},
               {"random", InitMode::kRandom},
               {"average", InitMode::kAverage}};
  for (const auto& c : cases) {
    InitMode mode = InitMode::kMaxSize;
    EXPECT_TRUE(parseInitMode(c.name, mode)) << c.name;
    EXPECT_EQ(mode, c.mode) << c.name;
    EXPECT_STREQ(toString(c.mode), c.name);
  }

  // Former mode names are rejected, not aliased, so an outdated script that
  // says min_size_max_vt fails instead of silently running something else.
  InitMode mode = InitMode::kMinSize;
  EXPECT_FALSE(parseInitMode("min_size_max_vt", mode));
  EXPECT_FALSE(parseInitMode("max_size_min_vt", mode));
  EXPECT_FALSE(parseInitMode("disabled", mode));
  EXPECT_FALSE(parseInitMode("bogus", mode));
  EXPECT_EQ(mode, InitMode::kMinSize);  // unchanged on failure
}

// Every init mode, including min_size_fixviol and min_size_fixcap, passes
// validate() without a warning.
TEST(InitConfig, EveryInitModeValidates)
{
  utl::Logger logger;
  GlobalSizingConfig config;
  for (const InitMode mode : {InitMode::kAsGiven,
                              InitMode::kMinSize,
                              InitMode::kMaxSize,
                              InitMode::kMinSizeFixviol,
                              InitMode::kMinSizeFixcap,
                              InitMode::kRandom,
                              InitMode::kAverage}) {
    config.init_mode = mode;
    EXPECT_TRUE(config.validate(&logger)) << toString(mode);
  }
  EXPECT_EQ(logger.getWarningCount(), 0);
}

// RSZ-0442: only `random` reads init_seed, so varying it under any other mode
// produces identical runs while RSZ-0417 still reports a different seed per
// run. This warns rather than rejects, because the setting itself is harmless.
TEST(InitConfig, InertInitSeedWarns)
{
  utl::Logger logger;
  GlobalSizingConfig config;
  config.init_mode = InitMode::kRandom;
  config.init_seed = 7;
  EXPECT_TRUE(config.validate(&logger));
  EXPECT_EQ(logger.getWarningCount(), 0);  // the one mode that reads it

  config.init_seed = 0;
  for (const InitMode mode :
       {InitMode::kAsGiven, InitMode::kMinSize, InitMode::kAverage}) {
    config.init_mode = mode;
    EXPECT_TRUE(config.validate(&logger));
  }
  EXPECT_EQ(logger.getWarningCount(), 0);  // the default seed is never flagged

  int expected_warnings = 0;
  config.init_seed = 7;
  for (const InitMode mode :
       {InitMode::kAsGiven, InitMode::kMinSize, InitMode::kAverage}) {
    config.init_mode = mode;
    EXPECT_TRUE(config.validate(&logger)) << toString(mode);
    EXPECT_EQ(logger.getWarningCount(), ++expected_warnings) << toString(mode);
  }
}

// RSZ-0421: state_adaptive infers past criticality from the current sizes, so
// every init mode that rewrites them is rejected.
TEST(InitConfig, StateAdaptiveRejectsEveryNonAsGivenMode)
{
  utl::Logger logger;
  GlobalSizingConfig config;
  config.lambda_seed = LambdaSeed::kStateAdaptive;
  config.init_mode = InitMode::kAsGiven;
  EXPECT_TRUE(config.validate(&logger));
  for (const InitMode mode : {InitMode::kMinSize,
                              InitMode::kMaxSize,
                              InitMode::kMinSizeFixviol,
                              InitMode::kMinSizeFixcap,
                              InitMode::kRandom,
                              InitMode::kAverage}) {
    config.init_mode = mode;
    EXPECT_THROW(config.validate(&logger), std::runtime_error)
        << toString(mode);
  }
}

// RSZ-0462/0463: slew_fix_pass resizes gates before the seed, also under
// as_given, so the seeds that read the as-given sizes treat it as they treat
// an init mode: state_adaptive is rejected and estimation_loop warns. The
// default seed reads no sizes, so the pass alone validates cleanly.
TEST(InitConfig, SlewFixPassCountsAsAnInitPassForTheSeed)
{
  GlobalSizingConfig config;
  config.init_mode = InitMode::kAsGiven;
  config.slew_fix_pass = true;
  {
    utl::Logger logger;
    config.lambda_seed = LambdaSeed::kStateAdaptive;
    EXPECT_THROW(config.validate(&logger), std::runtime_error);
  }
  {
    utl::Logger logger;
    config.lambda_seed = LambdaSeed::kEstimationLoop;
    EXPECT_TRUE(config.validate(&logger));
    EXPECT_EQ(logger.getWarningCount(), 1);
  }
  {
    utl::Logger logger;
    config.lambda_seed = LambdaSeed::kDelayPropCritMu;
    EXPECT_TRUE(config.validate(&logger));
    EXPECT_EQ(logger.getWarningCount(), 0);
  }
}

// RSZ-0422: with any init mode other than as_given, estimation_loop's warm
// start sees the initialized netlist rather than the original one. This makes
// the warm start less meaningful but not invalid, so it only warns.
TEST(InitConfig, EstimationLoopWarnsOnEveryNonAsGivenMode)
{
  utl::Logger logger;
  GlobalSizingConfig config;
  config.lambda_seed = LambdaSeed::kEstimationLoop;
  config.init_mode = InitMode::kAsGiven;
  EXPECT_TRUE(config.validate(&logger));
  EXPECT_EQ(logger.getWarningCount(), 0);

  int expected_warnings = 0;
  for (const InitMode mode : {InitMode::kMinSize,
                              InitMode::kMaxSize,
                              InitMode::kMinSizeFixviol,
                              InitMode::kMinSizeFixcap,
                              InitMode::kRandom,
                              InitMode::kAverage}) {
    config.init_mode = mode;
    EXPECT_TRUE(config.validate(&logger)) << toString(mode);
    EXPECT_EQ(logger.getWarningCount(), ++expected_warnings) << toString(mode);
  }
}

}  // namespace
}  // namespace rsz
