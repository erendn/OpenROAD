// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>

#include "OptimizationPolicy.hh"
#include "OptimizerTypes.hh"
#include "lr/LrState.hh"
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
class SweepEngine;
class BestTracker;
class Termination;

// GlobalSizingPolicy: Lagrangian-Relaxation-driven global sizing + Vt
// assignment, packaged as an OptimizationPolicy phase.
//
// The engine is decomposed into per-axis strategies (src/rsz/src/lr/) selected
// by GlobalSizingConfig and constructed once in start(). This class is the
// driver: it owns the shared LrState and the strategies and sequences them:
//
//   init (InitPass) -> allocate -> seed (LambdaSeeder) -> project
//   (FlowProjection) -> repeat { update (LambdaUpdater) -> project ->
//   sweep (SweepEngine) -> STA update -> pass accept/reject } until
//   Termination, then restore best (BestTracker).
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
  struct DesignSnap
  {
    double total_leakage = 0.0;
    double total_area = 0.0;
    int instances = 0;
    int with_leakage = 0;
  };
  DesignSnap computeDesignSnap() const;

  // Echo the effective config (preset + every knob) as an INFO run header.
  void logEffectiveConfig() const;

  // B2 (M3): refresh the policy-level cost-term state the sweep workers read
  // frozen - the reverse-topological φ pass (cost_global_phi) and the per-arc
  // reference-delay store (cost_delta_delay). Runs on the main thread before
  // each sweep; a no-op when both flags are off.
  void prepareCostTerms();

  // Reimann Alg. 2 loop 1: est_loop_iters dry-run sweeps that estimate a
  // warm-start lambda field. Each iteration commits a sweep, updates lambda
  // from the resulting timing, then rolls the sweep back via the journal
  // (engine-agnostic dry run) so only lambda carries forward. Called before the
  // main loop when lambda_seed == estimation_loop.
  void runEstimationLoop(float timing_weight);

  // === Policy state =========================================================
  GlobalSizingConfig gs_config_;
  sta::dbNetwork* db_network_ = nullptr;
  // Shared LR multiplier state + read-only STA handles, passed to strategies.
  LrState state_;

  // Per-axis strategies, constructed from gs_config_ in start().
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
