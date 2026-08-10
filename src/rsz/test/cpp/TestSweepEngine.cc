// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the M4 Gauss-Seidel sweep engine's one extractable pure
// function: orderTraversal (the F2 traversal ordering). The full engine (JIT
// snapshots, per-commit refresh, estimation-loop-under-GS) is exercised by the
// global_sizing_sweep_engine integration test; here we assert the ordering
// direction per traversal, the stable-tiebreak rule, and - most importantly for
// the plan's determinism requirement - that the order is independent of the
// input order (the mechanism that makes the single-threaded GS engine
// deterministic).

#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "lr/SweepEngine.hh"
#include "rsz/GlobalSizingConfig.hh"

namespace rsz {
namespace {

using Traversal = GlobalSizingConfig::Traversal;

// Build one entry (inst is unused by orderTraversal, so nullptr is fine).
TraversalEntry entry(const float key, const uint64_t tiebreak)
{
  TraversalEntry e;
  e.key = key;
  e.tiebreak = tiebreak;
  e.inst = nullptr;
  return e;
}

// Order a copy and return the resulting tiebreak sequence.
std::vector<uint64_t> orderedIds(std::vector<TraversalEntry> entries,
                                 const Traversal traversal)
{
  orderTraversal(entries, traversal);
  std::vector<uint64_t> ids;
  ids.reserve(entries.size());
  for (const TraversalEntry& e : entries) {
    ids.push_back(e.tiebreak);
  }
  return ids;
}

// forward_topo: ascending key (lowest output-vertex level first).
TEST(SweepEngineTraversal, ForwardTopoAscendingKey)
{
  const std::vector<TraversalEntry> e
      = {entry(3.0f, 30), entry(1.0f, 10), entry(2.0f, 20)};
  EXPECT_EQ(orderedIds(e, Traversal::kForwardTopo),
            (std::vector<uint64_t>{10, 20, 30}));
}

// reverse_topo: descending key (highest output-vertex level first).
TEST(SweepEngineTraversal, ReverseTopoDescendingKey)
{
  const std::vector<TraversalEntry> e
      = {entry(3.0f, 30), entry(1.0f, 10), entry(2.0f, 20)};
  EXPECT_EQ(orderedIds(e, Traversal::kReverseTopo),
            (std::vector<uint64_t>{30, 20, 10}));
}

// criticality_sorted: ascending key = most-negative slack (most critical)
// first, positive slack (least critical) last.
TEST(SweepEngineTraversal, CriticalitySortedMostCriticalFirst)
{
  const std::vector<TraversalEntry> e
      = {entry(0.5f, 1), entry(-0.3f, 2), entry(0.1f, 3)};
  EXPECT_EQ(orderedIds(e, Traversal::kCriticalitySorted),
            (std::vector<uint64_t>{2, 3, 1}));
}

// Ties on the key resolve on the smaller tiebreak in EVERY mode (the tiebreak
// is always ascending, independent of the key direction) - so equal-level or
// equal-slack gates get a single deterministic order.
TEST(SweepEngineTraversal, TiebreakAlwaysAscendingOnEqualKeys)
{
  const std::vector<TraversalEntry> e
      = {entry(2.0f, 30), entry(2.0f, 10), entry(2.0f, 20)};
  const std::vector<uint64_t> expect{10, 20, 30};
  EXPECT_EQ(orderedIds(e, Traversal::kForwardTopo), expect);
  EXPECT_EQ(orderedIds(e, Traversal::kReverseTopo), expect);
  EXPECT_EQ(orderedIds(e, Traversal::kCriticalitySorted), expect);
}

// The order is a total order (key then tiebreak), so it does not depend on the
// input order: shuffling the entries yields the same result every time. This is
// what makes the single-threaded GS sweep deterministic across runs.
TEST(SweepEngineTraversal, DeterministicRegardlessOfInputOrder)
{
  std::vector<TraversalEntry> base;
  base.reserve(50);
  for (int i = 0; i < 50; ++i) {
    // Deliberately repeat some keys so the tiebreak path is exercised.
    base.push_back(entry(static_cast<float>(i % 7), static_cast<uint64_t>(i)));
  }

  for (const Traversal t : {Traversal::kForwardTopo,
                            Traversal::kReverseTopo,
                            Traversal::kCriticalitySorted}) {
    const std::vector<uint64_t> ref = orderedIds(base, t);
    std::mt19937 rng(12345);
    for (int trial = 0; trial < 20; ++trial) {
      std::vector<TraversalEntry> shuffled = base;
      std::shuffle(shuffled.begin(), shuffled.end(), rng);
      EXPECT_EQ(orderedIds(shuffled, t), ref);
    }
  }
}

// Build one decision at a given relative cost improvement over the incumbent.
LRSubproblem::GateDecision decision(const float improvement_frac,
                                    const bool is_downsize)
{
  LRSubproblem::GateDecision d;
  d.baseline_cost = 100.0f;
  d.best_cost = 100.0f * (1.0f - improvement_frac);
  d.best_is_downsize = is_downsize;
  return d;
}

// The acceptance deadband is config, not a constant (hardening finding #10).
// 0.02 is rsz_baseline's own noise filter; 0.0 is the plain LRS argmin every
// paper specifies and what all eight paper presets pin.
TEST(SweepEngineAccept, UpsizeHysteresisGatesSmallUpsizeGainsOnly)
{
  // The knob's whole effect: a sub-deadband upsize gain. The baseline rejects
  // it as LR-cost noise; the papers take it, because it is the argmin.
  EXPECT_FALSE(acceptGateMove(decision(0.01f, /*is_downsize=*/false), 0.02f));
  EXPECT_TRUE(acceptGateMove(decision(0.01f, /*is_downsize=*/false), 0.0f));

  // Above the deadband, both agree - the knob only ever gates the margin.
  EXPECT_TRUE(acceptGateMove(decision(0.03f, /*is_downsize=*/false), 0.02f));
  EXPECT_TRUE(acceptGateMove(decision(0.03f, /*is_downsize=*/false), 0.0f));
}

TEST(SweepEngineAccept, DownsizesIgnoreTheDeadbandAtEitherSetting)
{
  // The asymmetry is the point: lambda is at the floor on a non-critical gate,
  // so any cost drop is a real leakage gain. Unchanged by the knob.
  for (const float tol : {0.0f, 0.02f}) {
    EXPECT_TRUE(acceptGateMove(decision(0.01f, /*is_downsize=*/true), tol))
        << "tol " << tol;
    EXPECT_TRUE(acceptGateMove(decision(0.001f, /*is_downsize=*/true), tol))
        << "tol " << tol;
  }
}

TEST(SweepEngineAccept, ArgminIsStrictSoNonImprovingMovesNeverAccept)
{
  // At 0.0 the rule is "strictly better", not "not worse" - a zero-gain or
  // cost-increasing candidate is still rejected in both directions. Without
  // this the papers' 0.0 would churn the netlist on ties.
  for (const bool downsize : {false, true}) {
    for (const float tol : {0.0f, 0.02f}) {
      EXPECT_FALSE(acceptGateMove(decision(0.0f, downsize), tol))
          << "tie, downsize " << downsize << " tol " << tol;
      EXPECT_FALSE(acceptGateMove(decision(-0.05f, downsize), tol))
          << "worse, downsize " << downsize << " tol " << tol;
    }
  }
}

// F4 switch-over (re-audit delta D2). fast_olr_start_iter is a 1-based
// λ-UPDATED LDP iteration and LrState::iter is a 0-based SWEEP index, and the
// two coincide because sweep 0 is the pre-update sweep rather than an LDP
// iteration. The bug this pins was an `iter + 1` that counted sweep 0 as
// iteration 1 and fired Sharma's switch on the 4th λ-updated pass instead of
// the paper's 5th.
TEST(FastOlrSwitchOver, ThePaperDefaultFiresOnTheFifthLambdaUpdatedSweep)
{
  const int paper_default = 5;
  // Sweep 0 is the pre-update sweep; sweeps 1-4 are the paper's "exhaustive OLR
  // for the first 4 iterations".
  for (int sweep = 0; sweep <= 4; ++sweep) {
    EXPECT_FALSE(fastOlrActive(sweep, paper_default)) << "sweep " << sweep;
  }
  // Sweep 5 is LDP iteration 5 - "Fast-OLR from iteration 5".
  EXPECT_TRUE(fastOlrActive(5, paper_default));
  EXPECT_TRUE(fastOlrActive(6, paper_default));
}

// The two settings a smoke test can use to reach the path on a design that
// converges in a handful of sweeps: 1 activates from the first λ-updated sweep,
// 0 from the pre-update sweep before it. Both are what the config header's doc
// promises, and 0 is the only value that makes sweep 0 pruned.
TEST(FastOlrSwitchOver, SmallStartIterationsActivateWhereTheDocSays)
{
  EXPECT_FALSE(fastOlrActive(0, 1));
  EXPECT_TRUE(fastOlrActive(1, 1));
  EXPECT_TRUE(fastOlrActive(0, 0));
}

// The config -> SnapshotInputs wiring, which no golden can see.
//
// An axis field that sweepInputs forgets to assign leaves the sweep running the
// SnapshotInputs struct default, and on the small designs in this suite that is
// invisible: measured directly, forcing Fast-OLR off for every sweep moves not
// one behavioural line in global_sizing_{closable,termination,guard,
// met_recovery} - only the RSZ-0417 echo of the config it did not obey. So the
// assignment itself is what has to be pinned. Both frozen A2/F4 fields are
// covered here; the guard branch is not exercised because it is the one path
// that queries STA.
TEST(SweepInputs, FreezesTheConfiguredAxisChoices)
{
  GlobalSizingConfig config;
  config.downsize_guard = GlobalSizingConfig::DownsizeGuard::kDepthBudget;
  LrState state;
  state.config = &config;

  // A2: the veto mode reaches the snapshot, in both settings.
  config.output_drc_veto = GlobalSizingConfig::OutputDrcVeto::kAbsolute;
  EXPECT_EQ(sweepInputs(state).output_drc_veto,
            GlobalSizingConfig::OutputDrcVeto::kAbsolute);
  config.output_drc_veto = GlobalSizingConfig::OutputDrcVeto::kRelative;
  EXPECT_EQ(sweepInputs(state).output_drc_veto,
            GlobalSizingConfig::OutputDrcVeto::kRelative);

  // F4: the move set, and the switch-over resolved against the sweep index -
  // the D2 semantic, checked here at the call site rather than on the bare
  // predicate, since the off-by-one it replaced lived at the call site.
  config.move_set = GlobalSizingConfig::MoveSet::kSharmaFastOlr;
  config.fast_olr_start_iter = 5;
  state.iter = 4;
  EXPECT_EQ(sweepInputs(state).move_set,
            GlobalSizingConfig::MoveSet::kSharmaFastOlr);
  EXPECT_FALSE(sweepInputs(state).fast_olr_active);
  state.iter = 5;
  EXPECT_TRUE(sweepInputs(state).fast_olr_active);
}

}  // namespace
}  // namespace rsz
