// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

// Unit tests for the N axis (initial solution): the per-mode selection rules in
// lr/InitSelect.hh, against hand-built swappable-equivalence groups. The
// STA-facing half of the pass - which instances are editable, the clock-network
// exclusion, dont-touch, and applying the swap - is exercised by the
// global_sizing_init and global_sizing_dont_touch integration tests.
//
// The groups below ARE the "constructed libraries": each is a list of
// candidates carrying the two keys the selector ranks by (cell leakage first,
// drive resistance as the tie-break) plus a name for the final tie-break.
// group[0] is always the as-given cell, exactly as InitPass builds it.
//
// What is deliberately pinned here:
//   * min_size / max_size keep their pre-rename semantics EXACTLY. The
//     iteration-2 rename was a rename; a change in what those two select would
//     silently re-base every chen/livramento preset run.
//   * random is reproducible from (init_seed, instance name) alone - not from
//     iteration order, not from a thread count, and never from the placement
//     seed.
//   * average is the LOWER median of the ranking, which for a group with no
//     full ties sits between min_size's and max_size's picks.
//   * min_size_fixviol's repair walk climbs that same ranking and stops at the
//     CHEAPEST member that clears the gate's electrical violation, leaving the
//     gate at minimum when nothing in the group does.

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
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

// A five-member group with distinct leakages, handed over in an order that is
// neither ascending nor descending, and whose as-given cell (index 0) is
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
// min_size / max_size - the renamed modes, unchanged semantics

TEST(InitSelect, MinSizePicksTheLowestLeakageMember)
{
  EXPECT_EQ(selectInitCandidate(group5(), InitMode::kMinSize, 0), 4u);
}

TEST(InitSelect, MaxSizePicksTheHighestLeakageMember)
{
  EXPECT_EQ(selectInitCandidate(group5(), InitMode::kMaxSize, 0), 1u);
}

// The leakage tie-break, in both directions: equal leakage is resolved by drive
// resistance, and the two modes read it opposite ways (min_size wants the WEAK
// driver, max_size the strong one). This is the rule the pre-rename selector
// used and the one `average`'s ranking reuses as its secondary key.
TEST(InitSelect, LeakageTiesBreakOnDriveResistance)
{
  const std::vector<InitCandidate> group = {cell("A", 4.0f, 2.0f),
                                            cell("weak", 4.0f, 9.0f),
                                            cell("strong", 4.0f, 0.5f)};
  EXPECT_EQ(selectInitCandidate(group, InitMode::kMinSize, 0), 1u);
  EXPECT_EQ(selectInitCandidate(group, InitMode::kMaxSize, 0), 2u);
}

// A candidate whose Liberty gives no leakage at all cannot be ranked, so the
// deterministic modes skip it - including when it is the as-given cell, which
// the first rankable candidate then displaces.
TEST(InitSelect, UnrankableCandidatesAreSkipped)
{
  InitCandidate unrankable;
  unrankable.name = "no_leakage";
  std::vector<InitCandidate> group = {unrankable, cell("A", 4.0f)};
  EXPECT_EQ(selectInitCandidate(group, InitMode::kMinSize, 0), 1u);
  EXPECT_EQ(selectInitCandidate(group, InitMode::kMaxSize, 0), 1u);
  EXPECT_EQ(selectInitCandidate(group, InitMode::kAverage, 0), 1u);

  // ...and when nothing at all can be ranked, every deterministic mode keeps
  // the as-given cell rather than picking arbitrarily.
  group = {unrankable, unrankable};
  EXPECT_EQ(selectInitCandidate(group, InitMode::kMinSize, 0), 0u);
  EXPECT_EQ(selectInitCandidate(group, InitMode::kMaxSize, 0), 0u);
  EXPECT_EQ(selectInitCandidate(group, InitMode::kAverage, 0), 0u);
}

// n = 1: a cell with no swappable equivalents keeps its own cell under every
// mode, random included (a one-member group has one answer).
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
  // Ranked E(1) C(2) A(4) D(8) B(16); floor((5-1)/2) = 2 -> A, the group's
  // own as-given cell here, which is a coincidence of this fixture and not the
  // rule (the next test moves it).
  EXPECT_EQ(selectInitCandidate(group5(), InitMode::kAverage, 0), 0u);
}

// Even n takes the LOWER median: with 4 members, index floor(3/2) = 1 of the
// ranking, not 2. Ranked: E(1) C(2) A(4) D(8) -> C.
TEST(InitSelect, AverageTakesTheLowerMedianAtEvenN)
{
  const std::vector<InitCandidate> group
      = {cell("A", 4.0f), cell("C", 2.0f), cell("D", 8.0f), cell("E", 1.0f)};
  EXPECT_EQ(selectInitCandidate(group, InitMode::kAverage, 0), 1u);
}

// The ranking `average` medians is the same order min_size and max_size take
// their ends of, so on a group with no full ties the three picks are ordered.
// This is what makes "min / middle / max" an honest description of the axis.
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

// Determinism of the median: the answer is a property of the group, not of the
// order the library happened to enumerate it in. Same members, reversed tail ->
// same cell. (Index 0 is held fixed because it is the as-given cell by
// definition, not a free permutation.)
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
// enumeration order.
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
// min_size_fixviol - the repair walk's upsize selection
//
// `clears(i)` stands in for the STA-facing check (does group[i] leave this
// gate's output pins free of max-cap and max-slew violations?). The fixtures
// below express it as a leakage threshold, which is the shape the real check
// has on a monotone library: a bigger cell has a higher cap limit and a lower
// drive resistance, so once one member clears, every stronger one does too.

// Clears at or above `min_leak` - a monotone library.
auto clearsFrom(const std::vector<InitCandidate>& group, const float min_leak)
{
  return [&group, min_leak](const size_t i) {
    return group[i].leakage >= min_leak;
  };
}

// The pass takes the CHEAPEST cell that clears, not the strongest in the group.
// Taking the strongest would be a leakage bill the violation did not require,
// on a pass whose entire purpose is to preserve the min-leakage solution.
TEST(InitFixviol, TakesTheCheapestMemberThatClears)
{
  // Ranked E(1) C(2) A(4) D(8) B(16); the incumbent after the min-size reset is
  // E, the ranking's first element.
  const std::vector<InitCandidate> group = group5();
  EXPECT_EQ(selectFixviolUpsize(group, 4, clearsFrom(group, 3.0f)),
            0u);  // A(4)
  EXPECT_EQ(selectFixviolUpsize(group, 4, clearsFrom(group, 8.0f)),
            3u);  // D(8)
}

// One step is one step: a gate that clears at the very next member does not
// skip ahead.
TEST(InitFixviol, ClimbsOneRankAtATime)
{
  const std::vector<InitCandidate> group = group5();
  EXPECT_EQ(selectFixviolUpsize(group, 4, clearsFrom(group, 2.0f)),
            2u);  // C(2)
}

// GROUP EXHAUSTION. Nothing in the group clears the violation, so the gate is
// left at minimum rather than upsized to the top for nothing. The pass repairs
// violations; it does not spend leakage on a gate it cannot fix.
TEST(InitFixviol, ExhaustionLeavesTheGateAtMinimum)
{
  const std::vector<InitCandidate> group = group5();
  EXPECT_EQ(selectFixviolUpsize(group, 4, [](size_t) { return false; }), 4u);
  // Same answer when the only clearing members are BELOW the incumbent: the
  // walk starts one rank above it, so the pass never downsizes to "repair".
  EXPECT_EQ(selectFixviolUpsize(group, 1, clearsFrom(group, 0.0f)), 1u);
}

// A gate that is not violating never reaches the selector, but if it did, a
// group whose incumbent is already the top rank has nothing above it.
TEST(InitFixviol, TheTopOfTheRankingHasNowhereToClimb)
{
  const std::vector<InitCandidate> group = group5();
  EXPECT_EQ(selectFixviolUpsize(group, 1, [](size_t) { return true; }), 1u);
}

// DETERMINISM. The walk climbs the same total order `average` medians, so the
// answer is a function of the group's MEMBERS and of `clears` - never of the
// order the library handed the group over in.
TEST(InitFixviol, SelectionIsIndependentOfGroupOrder)
{
  const std::vector<InitCandidate> forward = group5();
  // Same five members, enumerated differently, with the incumbent E still the
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
// equally-good repairs still have one answer.
TEST(InitFixviol, FullTiesResolveOnCellName)
{
  const std::vector<InitCandidate> ab
      = {cell("min", 1.0f), cell("b", 4.0f, 1.0f), cell("a", 4.0f, 1.0f)};
  const std::vector<InitCandidate> ba
      = {cell("min", 1.0f), cell("a", 4.0f, 1.0f), cell("b", 4.0f, 1.0f)};
  EXPECT_EQ(ab[selectFixviolUpsize(ab, 0, clearsFrom(ab, 4.0f))].name, "a");
  EXPECT_EQ(ba[selectFixviolUpsize(ba, 0, clearsFrom(ba, 4.0f))].name, "a");
}

// Members that cannot be RANKED are not repair candidates: the walk climbs the
// ranking, and a cell with no Liberty leakage has no place in it. (An
// unrankable INCUMBENT is a different matter - there is no rank to start above,
// so every rankable member is in play.)
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
// random - the seeded per-instance draw

// The draw is a pure function of (init_seed, instance name): the same pair
// reproduces bit-identically, which is what makes a pinned seed a reproducible
// experiment cell.
TEST(InitDraw, SameSeedAndInstanceReproduceTheSameDraw)
{
  EXPECT_EQ(initDraw(7, "u_alu/add_1"), initDraw(7, "u_alu/add_1"));
  EXPECT_EQ(
      selectInitCandidate(group5(), InitMode::kRandom, initDraw(7, "i0")),
      selectInitCandidate(group5(), InitMode::kRandom, initDraw(7, "i0")));
}

// Different seeds move the draw, and on a group with n > 1 they move the
// SELECTION - a seed that could not change the initial netlist would make the
// variation axis inert.
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

// Two instances under ONE seed also differ - the draw is per instance, not per
// run, so a single seed produces a mixed netlist rather than one global choice.
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

// ORDER AND THREAD INDEPENDENCE, the property (b) initDraw exists for. The draw
// reads no iteration state, so visiting the same instances in any order - as a
// different instance iterator, a different thread count, or a different
// OpenROAD version would - yields the identical per-instance selection.
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

// The draw indexes the group in CANONICAL cell-name order, so the order the
// library hands the group over in - an unstable sort by drive resistance, with
// the as-given cell prepended and dont-use/limit filtering applied - cannot
// move the selection. Without this, one `set_dont_use` elsewhere in a script
// would shift every index and silently re-draw EVERY instance under a seed the
// log still records as the same experiment cell.
TEST(InitDraw, SelectionIsIndependentOfGroupOrder)
{
  const std::vector<InitCandidate> forward = group5();
  // Same members, different enumeration order (including a different member
  // sitting at index 0, which is what a differently-sized incoming netlist
  // gives you).
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

// The draw covers the WHOLE group, the as-given cell included - the mode is a
// uniform draw over the equivalence group, not over "the alternatives to what
// is there". Unrankable members stay in the population too: the sweep engine
// prices a leakage-less cell through scaled area rather than excluding it, so
// the deterministic modes skip such a cell only because it cannot be RANKED.
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
// non-transitive - NaN compares false in both directions while the rest stay
// ordered - and an invalid strict weak ordering is undefined behavior in
// std::sort, not merely a wrong median. Such candidates are dropped from the
// ranking; min_size/max_size are a linear scan and are deliberately left alone,
// so this asserts the two do not have to agree in that corner.
TEST(InitSelect, AverageIgnoresNonFiniteRankingKeys)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<InitCandidate> group
      = {cell("A", 4.0f), cell("nan", nan), cell("C", 2.0f), cell("inf", inf)};
  // Ranked over the finite members only: C(2) A(4) -> lower median C.
  EXPECT_EQ(group[selectInitCandidate(group, InitMode::kAverage, 0)].name, "C");

  // A non-finite DRIVE resistance disqualifies a candidate the same way.
  group = {cell("A", 4.0f), cell("bad_drive", 1.0f, nan), cell("C", 2.0f)};
  EXPECT_EQ(group[selectInitCandidate(group, InitMode::kAverage, 0)].name, "C");

  // And when nothing is rankable, the as-given cell stands.
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

// The C axis, per preset - which papers state an initial solution and which
// one. Four do:
//  - chen (SOLVE_LRS/μ step 1 "x_i := L_i") and livramento (Alg. 1 L2) reset to
//    minimum leakage and stop there; neither states a repair pass at init
//    (Livramento's Alg. 3 FIX_VIOLATIONS is per-iteration, not init).
//  - flach (Fig. 1 (a)+(b)) and sharma (§8) reset AND repair the electrical
//    violations the reset creates, which is min_size_fixviol. Both ran as_given
//    until the it2 fixviol rider (RULED 2026-08-08): the mode landed in it2
//    pass 2 and adopting it restores the papers' own initialization.
// The rest run as_given, chinnery deliberately so (§5 step 2: it resizes the
// netlist the flow hands it).
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
      {Preset::kSharmaSeq, InitMode::kMinSizeFixviol},
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
               {"random", InitMode::kRandom},
               {"average", InitMode::kAverage}};
  for (const auto& c : cases) {
    InitMode mode = InitMode::kMaxSize;
    EXPECT_TRUE(parseInitMode(c.name, mode)) << c.name;
    EXPECT_EQ(mode, c.mode) << c.name;
    EXPECT_STREQ(toString(c.mode), c.name);
  }

  // The pre-rename spellings are GONE, not aliased: a script that still says
  // min_size_max_vt must fail loudly rather than quietly run something else.
  InitMode mode = InitMode::kMinSize;
  EXPECT_FALSE(parseInitMode("min_size_max_vt", mode));
  EXPECT_FALSE(parseInitMode("max_size_min_vt", mode));
  EXPECT_FALSE(parseInitMode("disabled", mode));
  EXPECT_FALSE(parseInitMode("bogus", mode));
  EXPECT_EQ(mode, InitMode::kMinSize);  // unchanged on failure
}

// The reservation is LIFTED: min_size_fixviol's electrical-repair half exists,
// so validate() no longer rejects it and every mode stands on its own.
TEST(InitConfig, EveryInitModeValidates)
{
  utl::Logger logger;
  GlobalSizingConfig config;
  for (const InitMode mode : {InitMode::kAsGiven,
                              InitMode::kMinSize,
                              InitMode::kMaxSize,
                              InitMode::kMinSizeFixviol,
                              InitMode::kRandom,
                              InitMode::kAverage}) {
    config.init_mode = mode;
    EXPECT_TRUE(config.validate(&logger)) << toString(mode);
  }
  EXPECT_EQ(logger.getWarningCount(), 0);
}

// RSZ-0442: only `random` reads init_seed, so a sweep that varies it under any
// other mode produces bit-identical runs while RSZ-0417 still echoes a
// different seed per run. Warn, not reject - the seed is harmless, the sweep is
// what is wrong.
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

// RSZ-0421, axis-complete: state_adaptive reads the CURRENT sizes to infer past
// criticality, so every mode that rewrites them is a hard reject - the new two
// included.
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
                              InitMode::kRandom,
                              InitMode::kAverage}) {
    config.init_mode = mode;
    EXPECT_THROW(config.validate(&logger), std::runtime_error)
        << toString(mode);
  }
}

// RSZ-0422, axis-complete: estimation_loop's warm start targets the initialized
// state rather than the as-given one, which muddies it without invalidating it.
// Soft - the cross stays a legal ablation cell.
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
                              InitMode::kRandom,
                              InitMode::kAverage}) {
    config.init_mode = mode;
    EXPECT_TRUE(config.validate(&logger)) << toString(mode);
    EXPECT_EQ(logger.getWarningCount(), ++expected_warnings) << toString(mode);
  }
}

}  // namespace
}  // namespace rsz
