// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

// Unit tests for the revert selection of the post-sweep max-cap re-check
// (lr/CapRecheck.hh), using hand-built contribution lists. The STA side (which
// nets a mover touches, their live cap slack, and applying the revert) is
// covered by the global_sizing_cap_recheck{,_gs} integration tests. Those use a
// design where the sweep's candidate filter, which checks each gate against a
// load snapshotted before any gate commits, cannot see the load that
// neighboring moves add.
//
// A "contribution" is one mover's signed effect on one net's max-cap slack, in
// farads. Negative means the mover pushed the net toward violation, and
// reverting it gives back exactly that much slack. A driver whose downsize
// lowered its own cap limit and a load whose upsize raised the net's
// capacitance are both expressed as this one signed number, so the selection
// rule only has to be stated once.
//
// The tests check three properties:
//   * Minimality. The selection reverts the fewest moves that clear the net,
//     biggest offender first, not every contributor. Reverting the whole
//     contributing set would put a shared net back exactly where the next
//     sweep found it, so the same moves would be proposed and reverted forever
//     and those gates would never settle.
//   * Attribution. A mover that relieved the net, or did not affect it, is
//     never reverted for it. If another gate caused the violation, that gate
//     is also a mover and is the one selected.
//   * Determinism. The result depends only on the contribution values and the
//     movers' commit order, not on the order the caller collected the
//     contributions in.

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
// Driver-side contribution: how a swap moved the gate's own cap limit
//
// A downsize lowers the limit without adding any load, so whether it causes a
// violation depends on a load the candidate filter read before the sweep. This
// is simple arithmetic on the Liberty limits and the effective (SDC-clamped)
// limit, so it is tested here rather than in an integration test. Reproducing
// it end to end would need a load low enough that the LR cost prefers the
// downsize and high enough that the neighbors' upsizes cross the smaller
// cell's limit; with a library whose limits double per drive strength there is
// no such window on a design small enough for a golden file.

// A BUF_X16 (965.58 fF limit) downsized to a BUF_X8 (484.01 fF). The mover
// gave up 481.57 fF of headroom, which is what reverting it gives back.
TEST(CapRecheck, ADownsizeReportsTheCeilingItGaveUp)
{
  EXPECT_FLOAT_EQ(driverLimitDelta(true, 484.009f, true, 965.576f, 484.009f),
                  -481.567f);
}

// An upsize raises its own limit, so it can only help the net it drives and is
// never a contributor.
TEST(CapRecheck, AnUpsizeReportsAPositiveDelta)
{
  EXPECT_FLOAT_EQ(driverLimitDelta(true, 965.576f, true, 484.009f, 965.576f),
                  481.567f);
  EXPECT_FLOAT_EQ(driverLimitDelta(true, 100.0f, true, 100.0f, 100.0f), 0.0f);
}

// An SDC max_capacitance tighter than the cell's own limit is the binding
// limit whatever cell is there, so the swap did not change it. Otherwise a
// design-wide `set_max_capacitance` would be blamed on whichever gate moved.
TEST(CapRecheck, AnSdcCeilingIsNotAttributedToTheMover)
{
  // Committed cell's Liberty limit 484, SDC clamps the effective limit to 40.
  EXPECT_FLOAT_EQ(driverLimitDelta(true, 484.009f, true, 965.576f, 40.0f),
                  0.0f);
  // ...including when the committed cell declares no Liberty limit at all.
  EXPECT_FLOAT_EQ(driverLimitDelta(false, 0.0f, true, 965.576f, 40.0f), 0.0f);
}

// The previous cell had no limit. Reverting would remove the limit rather than
// raise it, so there is no finite amount to rank the mover by. Returning 0
// under-attributes it; pricing it at the whole new limit would out-rank every
// real contributor on the net.
TEST(CapRecheck, ACeilingWhereThereWasNoneIsNotAttributed)
{
  EXPECT_FLOAT_EQ(driverLimitDelta(true, 484.009f, false, 0.0f, 484.009f),
                  0.0f);
  EXPECT_FLOAT_EQ(driverLimitDelta(true, 484.009f, true, 0.0f, 484.009f), 0.0f);
}

////////////////////////////////////////////////////////////////
// Nothing to do

// A net that still meets its limit reverts nothing, however many gates moved
// on it. The re-check only undoes moves that create a violation.
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

// The net is violating, but both movers on it only relieved it (they
// downsized, so they load their driver less). Some other gate caused the
// violation; on that gate's net it has a negative contribution and is
// selected there.
TEST(CapRecheck, MoversThatOnlyRelievedTheNetAreNeverReverted)
{
  const std::vector<CapContribution> c = {helped(0, 2.0f), helped(1, 3.0f)};
  EXPECT_TRUE(selectCapReverts(-4.0f, c).empty());
}

////////////////////////////////////////////////////////////////
// Minimality

// One contributor is enough to clear the net, so exactly one move is reverted
// even though three of them pushed it over together.
TEST(CapRecheck, GivesBackTheFewestMovesThatClearTheNet)
{
  const std::vector<CapContribution> c
      = {hurt(0, 1.0f), hurt(1, 4.0f), hurt(2, 1.0f)};
  // -3.0 + 4.0 = +1.0: mover 1 alone restores the net.
  EXPECT_EQ(selectCapReverts(-3.0f, c), std::vector<int>({1}));
}

// Biggest offender first keeps the count minimal: taking the small ones first
// would have reverted three moves here instead of one.
TEST(CapRecheck, TakesTheBiggestOffenderFirst)
{
  const std::vector<CapContribution> c
      = {hurt(0, 0.5f), hurt(1, 0.5f), hurt(2, 3.0f), hurt(3, 0.5f)};
  EXPECT_EQ(selectCapReverts(-2.0f, c), std::vector<int>({2}));
}

// When one move is not enough, it keeps going largest-first and stops as soon
// as the net is clear.
TEST(CapRecheck, KeepsGoingUntilTheNetIsClear)
{
  const std::vector<CapContribution> c
      = {hurt(0, 1.0f), hurt(1, 2.0f), hurt(2, 3.0f), hurt(3, 4.0f)};
  // -8.0: 4 + 3 = 7 is not enough, + 2 = 9 is. Mover 0 survives.
  EXPECT_EQ(selectCapReverts(-8.0f, c), std::vector<int>({3, 2, 1}));
}

// A relieving mover on the same net does not protect a harmful one: the live
// slack already includes everyone's effect, so the only question is which
// moves to revert.
TEST(CapRecheck, ARelievingMoverDoesNotShieldAHarmfulOne)
{
  const std::vector<CapContribution> c = {helped(0, 10.0f), hurt(1, 2.0f)};
  EXPECT_EQ(selectCapReverts(-1.0f, c), std::vector<int>({1}));
}

////////////////////////////////////////////////////////////////
// Group exhaustion

// The net was already violating before the sweep, so reverting every
// contribution still leaves it violating. Every move that made it worse is
// reverted, since each one degraded a violating net, which the sweep's own
// candidate filter refuses to do on a net it can see is violating.
TEST(CapRecheck, ExhaustionRevertsEveryContributor)
{
  const std::vector<CapContribution> c
      = {hurt(0, 1.0f), hurt(1, 2.0f), helped(2, 0.5f)};
  // -20.0 + 3.0 is still deeply negative.
  EXPECT_EQ(sorted(selectCapReverts(-20.0f, c)), std::vector<int>({0, 1}));
}

////////////////////////////////////////////////////////////////
// Determinism

// Equal offenders are ordered by mover index, which is the sweep's commit
// order, so the selection does not depend on the order the caller collected
// the contributions in. Without this, a net with identical contributors (the
// symmetric-fanout case the integration test uses) would revert an arbitrary
// one of them.
TEST(CapRecheck, EqualOffendersTieBreakOnCommitOrder)
{
  const std::vector<CapContribution> ascending
      = {hurt(0, 1.0f), hurt(1, 1.0f), hurt(2, 1.0f)};
  const std::vector<CapContribution> shuffled
      = {hurt(2, 1.0f), hurt(0, 1.0f), hurt(1, 1.0f)};
  EXPECT_EQ(selectCapReverts(-1.5f, ascending), std::vector<int>({0, 1}));
  EXPECT_EQ(selectCapReverts(-1.5f, shuffled), std::vector<int>({0, 1}));
}

// With distinct values the result depends only on the values, so any
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
