// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

// Unit tests for the revert selection of the post-sweep max-cap re-check
// (lr/CapRecheck.hh), against hand-built contribution lists. The STA-facing
// half
// - which nets a mover touches, what its live cap slack is, and applying the
// revert - is exercised by the global_sizing_cap_recheck{,_gs} integration
// tests, which reproduce the Jacobi blind spot end to end on a design built for
// it.
//
// A "contribution" is one mover's signed effect on ONE net's max-cap slack, in
// farads: negative means that mover pushed the net toward violation and that
// reverting it hands exactly that much slack back. The two physical mechanisms
// (a driver whose downsize lowered its own cap limit, a load whose upsize
// raised the net's capacitance) collapse into that one signed number by
// construction, which is what lets the selection rule below be stated once.
//
// What is deliberately pinned here:
//   * MINIMALITY. The selection gives back the fewest moves that clear the net,
//     biggest offender first - not every contributor. Reverting the whole
//     contributing set would put a shared net back exactly where the next sweep
//     found it, so the same set would be re-proposed and re-reverted forever
//     and those gates would never settle.
//   * ATTRIBUTION. A mover that relieved the net, or left it alone, is never
//     reverted for it. On a net some other gate ruined, that other gate is a
//     mover too and it is the one selected.
//   * DETERMINISM. The answer is a function of the contribution values and the
//     movers' commit order, and of nothing else - not of the order the caller
//     happened to collect the contributions in.

#include <algorithm>
#include <vector>

#include "gtest/gtest.h"
#include "lr/CapRecheck.hh"

namespace rsz {
namespace {

CapContribution hurt(const int mover, const float farads)
{
  return CapContribution{.mover = mover, .slack_delta = -farads};
}

CapContribution helped(const int mover, const float farads)
{
  return CapContribution{.mover = mover, .slack_delta = farads};
}

std::vector<int> sorted(std::vector<int> v)
{
  std::ranges::sort(v);
  return v;
}

////////////////////////////////////////////////////////////////
// The driver-side contribution: how a swap moved the gate's OWN cap ceiling
//
// This is the S1-T review's second half - a downsize lowers the limit while
// raising nothing, so whether that is a violation depends entirely on a load
// the frozen veto read before the sweep. It is arithmetic over four Liberty
// reads plus the effective (SDC-clamped) limit, so it is pinned here rather
// than in an integration leg: reproducing it end to end needs the frozen load
// to be simultaneously LOW enough that the LR cost prefers the downsize and
// HIGH enough that the neighbours' upsizes cross the smaller cell's limit, and
// a library whose limits step by 2x per drive strength leaves no such window on
// a design small enough to be a golden.

// The case that matters, in the units the library actually uses: a BUF_X16
// (965.58 fF ceiling) downsized to a BUF_X8 (484.01 fF). The mover gave up
// 481.57 fF of headroom, and that is what reverting it hands back.
TEST(CapRecheck, ADownsizeReportsTheCeilingItGaveUp)
{
  EXPECT_FLOAT_EQ(driverLimitDelta(true, 484.009f, true, 965.576f, 484.009f),
                  -481.567f);
}

// The opposite direction: an upsize raises its own ceiling, so it can only help
// the net it drives and is never a contributor.
TEST(CapRecheck, AnUpsizeReportsAPositiveDelta)
{
  EXPECT_FLOAT_EQ(driverLimitDelta(true, 965.576f, true, 484.009f, 965.576f),
                  481.567f);
  EXPECT_FLOAT_EQ(driverLimitDelta(true, 100.0f, true, 100.0f, 100.0f), 0.0f);
}

// An SDC max_capacitance tighter than the cell's own limit is the binding
// ceiling whatever cell sits there, so the swap did not move it. Without this
// branch a design-wide `set_max_capacitance` would be attributed to whichever
// gate happened to move.
TEST(CapRecheck, AnSdcCeilingIsNotAttributedToTheMover)
{
  // Committed cell's Liberty limit 484, SDC clamps the effective limit to 40.
  EXPECT_FLOAT_EQ(driverLimitDelta(true, 484.009f, true, 965.576f, 40.0f),
                  0.0f);
  // ...including when the committed cell declares no Liberty limit at all.
  EXPECT_FLOAT_EQ(driverLimitDelta(false, 0.0f, true, 965.576f, 40.0f), 0.0f);
}

// A ceiling where there was none: reverting would REMOVE the constraint rather
// than raise it, so there is no finite figure to rank the mover by.
// Deliberately under-attributed rather than priced at the whole new limit,
// which would out-rank every real contributor on the net.
TEST(CapRecheck, ACeilingWhereThereWasNoneIsNotAttributed)
{
  EXPECT_FLOAT_EQ(driverLimitDelta(true, 484.009f, false, 0.0f, 484.009f),
                  0.0f);
  EXPECT_FLOAT_EQ(driverLimitDelta(true, 484.009f, true, 0.0f, 484.009f), 0.0f);
}

////////////////////////////////////////////////////////////////
// Nothing to do

// A net that still meets its limit costs no move, however many gates moved on
// it. The re-check refuses moves that CREATE a violation; it is not a budget.
TEST(CapRecheck, AMetNetRevertsNothing)
{
  const std::vector<CapContribution> c = {hurt(0, 1.0f), hurt(1, 1.0f)};
  EXPECT_TRUE(selectCapReverts(5.0f, c).empty());
  // Exactly at the limit is met, matching the sign convention of OpenSTA's own
  // capacitance-check slack.
  EXPECT_TRUE(selectCapReverts(0.0f, c).empty());
}

TEST(CapRecheck, NoContributorsRevertsNothing)
{
  EXPECT_TRUE(selectCapReverts(-1.0f, {}).empty());
}

// The bystander case, and the reason attribution is part of the rule: this net
// is violating, and both movers on it only RELIEVED it (they downsized, so they
// load their driver less). Somebody else ruined the net - on that net they are
// a mover with a negative contribution, and they are the one selected there.
TEST(CapRecheck, MoversThatOnlyRelievedTheNetAreNeverReverted)
{
  const std::vector<CapContribution> c = {helped(0, 2.0f), helped(1, 3.0f)};
  EXPECT_TRUE(selectCapReverts(-4.0f, c).empty());
}

////////////////////////////////////////////////////////////////
// Minimality

// One contributor is enough to clear the net, so exactly one move is given up -
// even though three of them pushed it over together.
TEST(CapRecheck, GivesBackTheFewestMovesThatClearTheNet)
{
  const std::vector<CapContribution> c
      = {hurt(0, 1.0f), hurt(1, 4.0f), hurt(2, 1.0f)};
  // -3.0 + 4.0 = +1.0: mover 1 alone restores the net.
  EXPECT_EQ(selectCapReverts(-3.0f, c), std::vector<int>({1}));
}

// ...and the order is biggest offender first, so the count really is minimal:
// taking the small ones first would have cost three moves here instead of one.
TEST(CapRecheck, TakesTheBiggestOffenderFirst)
{
  const std::vector<CapContribution> c
      = {hurt(0, 0.5f), hurt(1, 0.5f), hurt(2, 3.0f), hurt(3, 0.5f)};
  EXPECT_EQ(selectCapReverts(-2.0f, c), std::vector<int>({2}));
}

// When one is not enough it keeps going, still largest-first, and still stops
// the moment the net is clear.
TEST(CapRecheck, KeepsGoingUntilTheNetIsClear)
{
  const std::vector<CapContribution> c
      = {hurt(0, 1.0f), hurt(1, 2.0f), hurt(2, 3.0f), hurt(3, 4.0f)};
  // -8.0: 4 + 3 = 7 is not enough, + 2 = 9 is. Mover 0 survives.
  EXPECT_EQ(selectCapReverts(-8.0f, c), std::vector<int>({3, 2, 1}));
}

// A relieving mover on the same net does not shield a harmful one: the live
// slack already nets everyone's effect, so the only question is who to take
// back.
TEST(CapRecheck, ARelievingMoverDoesNotShieldAHarmfulOne)
{
  const std::vector<CapContribution> c = {helped(0, 10.0f), hurt(1, 2.0f)};
  EXPECT_EQ(selectCapReverts(-1.0f, c), std::vector<int>({1}));
}

////////////////////////////////////////////////////////////////
// Group exhaustion

// The net was already violating before the sweep, so handing back every
// contribution still leaves it violating. Everything that made it worse goes -
// each one is still a move that degraded a violating net, which is exactly what
// the sweep's own frozen check refuses to do on a net it can see is violating.
TEST(CapRecheck, ExhaustionRevertsEveryContributor)
{
  const std::vector<CapContribution> c
      = {hurt(0, 1.0f), hurt(1, 2.0f), helped(2, 0.5f)};
  // -20.0 + 3.0 is still deeply negative.
  EXPECT_EQ(sorted(selectCapReverts(-20.0f, c)), std::vector<int>({0, 1}));
}

////////////////////////////////////////////////////////////////
// Determinism

// Equal offenders tie-break on the mover index, i.e. on the sweep's commit
// order, so the selection cannot depend on the order the caller collected the
// contributions in. Without this a net whose contributors are identical - the
// symmetric-fanout case the integration test is built on - would revert an
// arbitrary one of them.
TEST(CapRecheck, EqualOffendersTieBreakOnCommitOrder)
{
  const std::vector<CapContribution> ascending
      = {hurt(0, 1.0f), hurt(1, 1.0f), hurt(2, 1.0f)};
  const std::vector<CapContribution> shuffled
      = {hurt(2, 1.0f), hurt(0, 1.0f), hurt(1, 1.0f)};
  EXPECT_EQ(selectCapReverts(-1.5f, ascending), std::vector<int>({0, 1}));
  EXPECT_EQ(selectCapReverts(-1.5f, shuffled), std::vector<int>({0, 1}));
}

// And with distinct values the answer is a pure function of the values, so any
// permutation of the same contributions selects the same movers.
TEST(CapRecheck, SelectionIsIndependentOfCollectionOrder)
{
  std::vector<CapContribution> c
      = {hurt(0, 1.0f), hurt(1, 4.0f), hurt(2, 2.0f), hurt(3, 3.0f)};
  const std::vector<int> expected = selectCapReverts(-6.0f, c);
  EXPECT_EQ(expected, std::vector<int>({1, 3}));
  std::ranges::reverse(c);
  EXPECT_EQ(selectCapReverts(-6.0f, c), expected);
}

}  // namespace
}  // namespace rsz
