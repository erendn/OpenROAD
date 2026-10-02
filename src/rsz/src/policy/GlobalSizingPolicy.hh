// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>

#include "OptimizationPolicy.hh"
#include "OptimizerTypes.hh"
#include "lr/LrState.hh"
#include "lr/SweepEngine.hh"
#include "lr/ViolationRepair.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "sta/MinMax.hh"

namespace sta {
class dbNetwork;
}  // namespace sta

namespace rsz {

class InitPass;
class LambdaSeeder;
class LambdaUpdater;
class FlowProjection;
class BestTracker;
class Termination;

// GlobalSizingPolicy: Lagrangian-Relaxation-driven global sizing + Vt
// assignment, packaged as an OptimizationPolicy phase.
//
// Each step of the algorithm is a strategy object (src/rsz/src/lr/) selected
// by GlobalSizingConfig and constructed once in start(). This class owns the
// shared LrState and the strategies and runs them in order:
//
//   init (InitPass) -> allocate -> seed (LambdaSeeder) -> project
//   (FlowProjection) -> repeat { update (LambdaUpdater) -> project ->
//   [restart] -> repeat { sweep (SweepEngine) -> STA update -> [cap fix
//   pass] } -> pass accept/reject } until Termination, then restore best
//   (BestTracker).
//
// Skips the OptimizationPolicy generator/candidate pipeline and the
// target_collector - LR is not target-driven.
class GlobalSizingPolicy : public OptimizationPolicy
{
 public:
  GlobalSizingPolicy(Resizer& resizer,
                     MoveCommitter& committer,
                     RepairSetupContext& setup_context,
                     const OptimizerRunConfig& config);
  ~GlobalSizingPolicy() override;

  const char* name() const override { return "GlobalSizingPolicy"; }
  bool start() override;
  void iterate() override;

 private:
  // === Diagnostics ==========================================================
  // Design totals. total_power is the objective power (see IterMetrics) of
  // the instances total_leakage counts.
  struct DesignSnap
  {
    double total_leakage = 0.0;
    double total_power = 0.0;
    double total_area = 0.0;
    int instances = 0;
    int with_leakage = 0;
  };
  DesignSnap computeDesignSnap() const;

  // Log the effective configuration (preset and every option) at the start of
  // the run.
  void logEffectiveConfig() const;

  // Under the total-power objective, reads OpenSTA's switching activities
  // into the objective-power model. Must run before the first cell swap,
  // because OpenSTA discards its activities on every swap. A no-op under the
  // leakage objective.
  void cacheActivities();

  // Refresh the per-edge data the optional cost terms read during the sweep:
  // the reverse-topological φ pass (cost_global_phi) and the per-arc reference
  // delays (cost_delta_delay). Runs on the main thread before each sweep; a
  // no-op when both options are off.
  void prepareCostTerms();

  // What one subproblem solve did: the sweeps it ran, and their move counts
  // summed.
  struct SubproblemStats
  {
    SweepEngine::Stats totals;
    int sweeps = 0;
    // The last sweep still kept a move, so the solve stopped at
    // max_inner_sweeps rather than because nothing changed.
    bool hit_sweep_cap = false;
    // The max-capacitance fix passes after the sweeps (cap_fix_pass), summed.
    // Their resizes are not sweep moves, so `totals` does not count them.
    RepairWalkStats cap_fix;
  };

  // Solves the LR subproblem for the current multipliers: sweeps, refreshing
  // parasitics and timing after each sweep, until a sweep keeps no move
  // (moves minus cap re-check reverts) or max_inner_sweeps sweeps have run.
  // Multipliers and `timing_weight` stay fixed; the cost terms that read
  // timing are refreshed before every sweep. With cap_fix_pass, each sweep's
  // timing update is followed by the max-capacitance fix pass, which updates
  // timing again if it resized a gate. With max_inner_sweeps = 1 this is one
  // sweep and its timing update. With relax_max_cap, the run's first solve
  // sets the initial max-capacitance multipliers from `timing_weight`.
  SubproblemStats solveSubproblem(float timing_weight);

  // With relax_max_cap, updates the max-capacitance multipliers (Livramento
  // et al., Alg. 1 line 15) from the timing the lambda update just read.
  // Called right after every lambda update. A no-op otherwise.
  void updateCapMultipliers();

  // With relax_max_cap, logs the initial beta, the number of priced pins, and
  // how many of them are over their max capacitance at the start of the run
  // and at the end (RSZ-0464). A no-op otherwise.
  void logCapMultipliers() const;

  // With cap_fix_pass, logs what the fix passes after the main loop's sweeps
  // did (RSZ-0459). The estimation loop's sweeps are not counted. A no-op
  // without the pass.
  void logCapFixSummary(const RepairWalkStats& stats, int sweeps) const;

  // With max_inner_sweeps > 1, logs the main loop's sweeps and how many of
  // its iterations stopped at the cap (RSZ-0455). The estimation loop's
  // sweeps are not counted. A no-op with one sweep per iteration.
  void logInnerLoopSummary(int sweeps,
                           int iterations,
                           int sweep_cap_iters) const;

  // With restart_each_iteration, records every gate's cell right after the
  // init pass. A no-op otherwise.
  void captureInitialCells();

  // With restart_each_iteration, sets every gate back to the cell
  // captureInitialCells() recorded, as Chen et al. solve every subproblem
  // from the lower size bound (ICCAD 1998, SOLVE_LRS/mu step 1). Then
  // refreshes the live edges, parasitics and timing, so the subproblem solve
  // and the WNS read before it see the restored netlist. Called after the
  // multiplier update and projection of every iteration but the first. A
  // no-op otherwise.
  void restartFromInitialCells(int iter);

  // Logs how many timing edges re-created by cell swaps over the whole run,
  // the estimation loop included, had their multipliers rewritten (RSZ-0456;
  // see LrState::refreshLiveEdges). Silent when no swap re-created an edge.
  void logEdgeCarry() const;

  // Reimann Alg. 2 loop 1: est_loop_iters dry-run iterations that estimate a
  // warm-start lambda field. Each iteration solves the subproblem, updates
  // lambda from the resulting timing, then rolls the solve back via the
  // journal (engine-agnostic dry run) so only lambda carries forward. Called
  // before the main loop when lambda_seed == estimation_loop.
  void runEstimationLoop(float timing_weight);

  // === Policy state =========================================================
  GlobalSizingConfig gs_config_;
  sta::dbNetwork* db_network_ = nullptr;
  // Shared LR multiplier state + read-only STA handles, passed to strategies.
  LrState state_;
  // The cells right after the init pass, for restart_each_iteration.
  CellAssignment initial_cells_;

  // Strategies, constructed from gs_config_ in start().
  std::unique_ptr<InitPass> init_pass_;
  std::unique_ptr<LambdaSeeder> seeder_;
  std::unique_ptr<LambdaUpdater> updater_;
  std::unique_ptr<FlowProjection> projection_;
  std::unique_ptr<SweepEngine> sweep_engine_;
  std::unique_ptr<BestTracker> best_tracker_;
  std::unique_ptr<Termination> termination_;

  const sta::MinMax* policy_max_ = sta::MinMax::max();
};

}  // namespace rsz
