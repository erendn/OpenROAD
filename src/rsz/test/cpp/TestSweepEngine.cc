// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the parts of the sweep engines that need no STA: the
// Gauss-Seidel traversal order (orderTraversal), the move acceptance rule, the
// Fast-OLR switch-over, and the config-to-sweep wiring. The full engine
// (just-in-time snapshots, per-commit refresh, the estimation loop under
// Gauss-Seidel) is covered by the global_sizing_sweep_engine integration test.
// The traversal tests check the direction of each ordering, the tie-break rule,
// and that the order does not depend on the input order, which is what makes
// the single-threaded Gauss-Seidel engine deterministic.

#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "lr/SweepEngine.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "utl/Logger.h"

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

// Ties on the key go to the smaller tiebreak in every mode (the tiebreak is
// always ascending, whatever the key direction), so gates with equal level or
// equal slack get one deterministic order.
TEST(SweepEngineTraversal, TiebreakAlwaysAscendingOnEqualKeys)
{
  const std::vector<TraversalEntry> e
      = {entry(2.0f, 30), entry(2.0f, 10), entry(2.0f, 20)};
  const std::vector<uint64_t> expect{10, 20, 30};
  EXPECT_EQ(orderedIds(e, Traversal::kForwardTopo), expect);
  EXPECT_EQ(orderedIds(e, Traversal::kReverseTopo), expect);
  EXPECT_EQ(orderedIds(e, Traversal::kCriticalitySorted), expect);
}

// The order is a total order (key, then tiebreak), so it does not depend on
// the input order: shuffling the entries gives the same result every time.
// This is what makes the single-threaded Gauss-Seidel sweep deterministic
// across runs.
TEST(SweepEngineTraversal, DeterministicRegardlessOfInputOrder)
{
  std::vector<TraversalEntry> base;
  base.reserve(50);
  for (int i = 0; i < 50; ++i) {
    // Repeat some keys so the tiebreak path is exercised.
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

// The upsize acceptance threshold is configurable. The default 0.02 filters
// out upsizes whose cost gain is within LR cost noise; 0.0 takes the plain
// argmin of the LR subproblem, as every paper does, and is what the paper
// presets use.
TEST(SweepEngineAccept, UpsizeHysteresisGatesSmallUpsizeGainsOnly)
{
  // An upsize gain below the threshold: rejected at 0.02 as noise, accepted at
  // 0.0 because it is the argmin.
  EXPECT_FALSE(acceptGateMove(decision(0.01f, /*is_downsize=*/false), 0.02f));
  EXPECT_TRUE(acceptGateMove(decision(0.01f, /*is_downsize=*/false), 0.0f));

  // Above the threshold both settings accept; the threshold only affects small
  // gains.
  EXPECT_TRUE(acceptGateMove(decision(0.03f, /*is_downsize=*/false), 0.02f));
  EXPECT_TRUE(acceptGateMove(decision(0.03f, /*is_downsize=*/false), 0.0f));
}

TEST(SweepEngineAccept, DownsizesIgnoreTheDeadbandAtEitherSetting)
{
  // Downsizes are never filtered: on a non-critical gate lambda is at its
  // floor, so any cost drop is a real leakage gain.
  for (const float tol : {0.0f, 0.02f}) {
    EXPECT_TRUE(acceptGateMove(decision(0.01f, /*is_downsize=*/true), tol))
        << "tol " << tol;
    EXPECT_TRUE(acceptGateMove(decision(0.001f, /*is_downsize=*/true), tol))
        << "tol " << tol;
  }
}

TEST(SweepEngineAccept, ArgminIsStrictSoNonImprovingMovesNeverAccept)
{
  // At 0.0 the rule is "strictly better", not "not worse": a zero-gain or
  // cost-increasing candidate is rejected in both directions. Otherwise a
  // threshold of 0.0 would keep swapping cells on ties.
  for (const bool downsize : {false, true}) {
    for (const float tol : {0.0f, 0.02f}) {
      EXPECT_FALSE(acceptGateMove(decision(0.0f, downsize), tol))
          << "tie, downsize " << downsize << " tol " << tol;
      EXPECT_FALSE(acceptGateMove(decision(-0.05f, downsize), tol))
          << "worse, downsize " << downsize << " tol " << tol;
    }
  }
}

// Fast-OLR switch-over (Sharma et al., ICCAD 2015, Fig. 9).
// fast_olr_start_iter is a 1-based count of λ-updated LDP iterations and
// LrState::iter is a 0-based sweep index. The two coincide because sweep 0 runs
// before the first λ update and is not an LDP iteration. Counting sweep 0 as
// iteration 1 would switch on the 4th λ-updated sweep instead of the paper's
// 5th.
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

// Small start values, useful for reaching Fast-OLR on a design that converges
// in a few sweeps: 1 activates it from the first λ-updated sweep, 0 from the
// sweep before it. Both match the GlobalSizingConfig documentation, and 0 is
// the only value that restricts sweep 0.
TEST(FastOlrSwitchOver, SmallStartIterationsActivateWhereTheDocSays)
{
  EXPECT_FALSE(fastOlrActive(0, 1));
  EXPECT_TRUE(fastOlrActive(1, 1));
  EXPECT_TRUE(fastOlrActive(0, 0));
}

// The copy from GlobalSizingConfig into SnapshotInputs. If sweepInputs forgot
// to copy a field, the sweep would silently use the SnapshotInputs default. On
// the small designs in this test suite that often does not change the result
// (for example, forcing Fast-OLR off changes nothing in the output of
// global_sizing_{closable,termination,guard,met_recovery} except the RSZ-0417
// config echo), so the copy itself is tested here. The downsize guard branch
// is not exercised because it is the one path that queries STA.
TEST(SweepInputs, FreezesTheConfiguredAxisChoices)
{
  GlobalSizingConfig config;
  config.downsize_guard = GlobalSizingConfig::DownsizeGuard::kDepthBudget;
  LrState state;
  state.config = &config;

  // The output DRC veto mode reaches the snapshot, in both settings.
  config.output_drc_veto = GlobalSizingConfig::OutputDrcVeto::kAbsolute;
  EXPECT_EQ(sweepInputs(state).output_drc_veto,
            GlobalSizingConfig::OutputDrcVeto::kAbsolute);
  config.output_drc_veto = GlobalSizingConfig::OutputDrcVeto::kRelative;
  EXPECT_EQ(sweepInputs(state).output_drc_veto,
            GlobalSizingConfig::OutputDrcVeto::kRelative);

  // The move set, and the Fast-OLR switch-over resolved against the sweep
  // index. This checks the call site, not just fastOlrActive, because an
  // off-by-one in the index passed by the caller would not show up in the
  // predicate tests above.
  config.move_set = GlobalSizingConfig::MoveSet::kSharmaFastOlr;
  config.fast_olr_start_iter = 5;
  state.iter = 4;
  EXPECT_EQ(sweepInputs(state).move_set,
            GlobalSizingConfig::MoveSet::kSharmaFastOlr);
  EXPECT_FALSE(sweepInputs(state).fast_olr_active);
  state.iter = 5;
  EXPECT_TRUE(sweepInputs(state).fast_olr_active);

  // The register switch, in both settings.
  config.size_registers = false;
  EXPECT_FALSE(sweepInputs(state).size_registers);
  config.size_registers = true;
  EXPECT_TRUE(sweepInputs(state).size_registers);

  // The timing pricing, in both settings.
  config.timing_cost = GlobalSizingConfig::TimingCost::kPerArc;
  EXPECT_TRUE(sweepInputs(state).cost.per_arc);
  config.timing_cost = GlobalSizingConfig::TimingCost::kWorstArc;
  EXPECT_FALSE(sweepInputs(state).cost.per_arc);

  // The run's objective-power model. Without it the cost would price leakage
  // under every objective.
  EXPECT_EQ(sweepInputs(state).objective_power, &state.objective_power);

  // The max-capacitance multipliers, only under relax_max_cap. Without them
  // the sweep would silently keep its max-cap filter and price nothing.
  config.relax_max_cap = true;
  EXPECT_EQ(sweepInputs(state).cap_multipliers, &state.cap_multipliers);
  config.relax_max_cap = false;
  EXPECT_EQ(sweepInputs(state).cap_multipliers, nullptr);
}

// The power-phase filter applies only to a sweep of the power phase, and only
// when it is configured: the two are combined once per sweep.
TEST(SweepInputs, PowerPhaseFilterNeedsTheSettingAndThePowerPhase)
{
  utl::Logger logger;
  GlobalSizingConfig config;
  config.downsize_guard = GlobalSizingConfig::DownsizeGuard::kDepthBudget;
  LrState state;
  state.config = &config;
  state.logger = &logger;

  config.power_phase_filter = false;
  state.power_phase = false;
  EXPECT_FALSE(sweepInputs(state).power_phase_filter);
  state.power_phase = true;
  EXPECT_FALSE(sweepInputs(state).power_phase_filter);

  config.power_phase_filter = true;
  state.power_phase = false;
  EXPECT_FALSE(sweepInputs(state).power_phase_filter);
  state.power_phase = true;
  EXPECT_TRUE(sweepInputs(state).power_phase_filter);

  // The update's phase is not the sweep's.
  state.power_phase = false;
  state.prev_sweep_power_phase = true;
  EXPECT_FALSE(sweepInputs(state).power_phase_filter);
}

}  // namespace
}  // namespace rsz
