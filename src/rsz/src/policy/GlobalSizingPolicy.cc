// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "GlobalSizingPolicy.hh"

#include <algorithm>
#include <memory>
#include <optional>

#include "OptimizationPolicy.hh"
#include "OptimizerTypes.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "lr/BestTracker.hh"
#include "lr/CapMultipliers.hh"
#include "lr/CostTerms.hh"
#include "lr/FlowProjection.hh"
#include "lr/InitPass.hh"
#include "lr/LambdaSeeder.hh"
#include "lr/LambdaUpdater.hh"
#include "lr/LrState.hh"
#include "lr/ObjectivePower.hh"
#include "lr/SweepEngine.hh"
#include "lr/Termination.hh"
#include "lr/TimingScale.hh"
#include "lr/ViolationRepair.hh"
#include "odb/db.h"
#include "rsz/GlobalSizingConfig.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Fuzzy.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Scene.hh"
#include "sta/Sta.hh"
#include "utl/Logger.h"
#include "utl/ThreadPool.h"

namespace rsz {

using utl::RSZ;

GlobalSizingPolicy::GlobalSizingPolicy(Resizer& resizer,
                                       MoveCommitter& committer,
                                       RepairSetupContext& setup_context,
                                       const OptimizerRunConfig& config)
    : OptimizationPolicy(resizer, committer, setup_context, config)
{
}

GlobalSizingPolicy::~GlobalSizingPolicy() = default;

GlobalSizingPolicy::DesignSnap GlobalSizingPolicy::computeDesignSnap() const
{
  DesignSnap s;
  const ObjectivePower& power = state_.objective_power;
  std::unique_ptr<sta::LeafInstanceIterator> iit(
      network_->leafInstanceIterator());
  while (iit->hasNext()) {
    sta::Instance* inst = iit->next();
    sta::LibertyCell* cell = network_->libertyCell(inst);
    if (cell == nullptr) {
      continue;
    }
    ++s.instances;
    const std::optional<float> leak = resizer_.cellLeakage(cell);
    if (leak.has_value()) {
      s.total_leakage += *leak;
      ++s.with_leakage;
      // Under the leakage objective this adds *leak, so total_power equals
      // total_leakage exactly.
      s.total_power += power.power(
          cell, *leak, sizingPowerInputs(power, resizer_, inst, policy_max_));
    }
    odb::dbMaster* master = db_network_->staToDb(db_network_->cell(cell));
    if (master != nullptr && master->isCoreAutoPlaceable()) {
      s.total_area += resizer_.dbuToMeters(master->getWidth())
                      * resizer_.dbuToMeters(master->getHeight());
    }
  }
  return s;
}

void GlobalSizingPolicy::logEffectiveConfig() const
{
  // Always logged, in a fixed key=value format that scripts can parse. It
  // reports the effective value of every option, after the preset and all
  // individual overrides are applied. report_global_sizing_config differs: it
  // reports only what the user set (`undefined` for an option not set).
  // Append new options to the last ` | ` section so earlier tokens keep their
  // positions for scripts that parse this line.
  //
  // `preset=` reports the preset that was requested. Without -preset no preset
  // was applied and the options keep their struct defaults, so it prints
  // `unset` rather than the default enum value.
  const char* preset_name
      = gs_config_.preset_explicit ? toString(gs_config_.preset) : "unset";
  logger_->info(
      RSZ,
      417,
      "GLOBAL_SIZING config: preset={} init={} init_seed={} seed={} update={} "
      "mu_policy={} mu_autopair={} projection={} sweep={} gs_refresh={} "
      "traversal={} guard={} move_set={} fast_olr_start={} "
      "output_drc_veto={} "
      "timing_scale={} term={} best={} | max_iter={} "
      "beta={:.3g} mu_exp={:.3g} lambda_floor={:.3g} timing_bias={:.3g} "
      "budget_safety={:.3g} upsize_hyst={:.3g} clock_net={} margin={:.3g} "
      "| lambda_init={:.3g} "
      "seed_exp={:.3g} est_iters={} update_c={:.3g} "
      "flach_k={:.3g}/{:.3g}/{:.3g} sharma_r={:.3g} sharma_k={:.3g} "
      "reimann_rho={:.3g} reimann_k={:.3g} reimann_ksched={:.3g}/{:.3g}/{:.3g} "
      "reimann_setpoint={} livramento_alpha0={:.3g} "
      "| cost_upstream_load={} cost_fanout_slew={} cost_global_phi={} "
      "cost_delta_delay={} | gamma_local_slack={:.3g} "
      "stagnation={}/{}/{:.3g}/{} near_met_gate={:.3g} "
      "chinnery={:.3g}/{:.3g}/{:.3g}/{:.3g}/{}/{:.3g} "
      "best_tns_frac={:.3g} | size_registers={} power_objective={} "
      "power_phase_filter={} arc_slack_k={:.3g}/{:.3g}/{:.3g}/{:.3g} "
      "max_inner_sweeps={} restart_each_iteration={} timing_cost={} "
      "cap_fix_pass={} slew_fix_pass={} relax_max_cap={}",
      preset_name,
      toString(gs_config_.init_mode),
      gs_config_.init_seed,
      toString(gs_config_.lambda_seed),
      toString(gs_config_.lambda_update),
      toString(gs_config_.mu_policy),
      gs_config_.mu_auto_paired,
      toString(gs_config_.kkt_projection),
      toString(gs_config_.sweep_engine),
      toString(gs_config_.gs_refresh),
      toString(gs_config_.traversal),
      toString(gs_config_.downsize_guard),
      toString(gs_config_.move_set),
      gs_config_.fast_olr_start_iter,
      toString(gs_config_.output_drc_veto),
      toString(gs_config_.timing_scale),
      toString(gs_config_.termination),
      toString(gs_config_.best_tracker),
      gs_config_.max_iterations,
      gs_config_.beta,
      gs_config_.mu_exponent,
      gs_config_.lambda_floor,
      gs_config_.timing_bias,
      gs_config_.budget_safety_factor,
      gs_config_.upsize_hysteresis,
      gs_config_.include_clock_network,
      gs_config_.setup_slack_margin,
      gs_config_.lambda_init_value,
      gs_config_.lambda_seed_exponent,
      gs_config_.est_loop_iters,
      gs_config_.lambda_update_c,
      gs_config_.flach_k_init,
      gs_config_.flach_k_tns_small,
      gs_config_.flach_k_final,
      gs_config_.sharma_r,
      gs_config_.sharma_k,
      gs_config_.reimann_rho_init,
      gs_config_.reimann_k,
      gs_config_.reimann_k_est,
      gs_config_.reimann_k_lo,
      gs_config_.reimann_k_hi,
      toString(gs_config_.reimann_setpoint),
      gs_config_.livramento_alpha0,
      gs_config_.cost_upstream_load,
      gs_config_.cost_fanout_slew,
      gs_config_.cost_global_phi,
      gs_config_.cost_delta_delay,
      gs_config_.gamma_local_slack,
      gs_config_.stagnation_window,
      gs_config_.stagnation_count,
      gs_config_.stagnation_improve_frac,
      gs_config_.stagnation_require_tns,
      gs_config_.near_met_gate_frac,
      gs_config_.term_tns_target_frac,
      gs_config_.term_wns_target_frac,
      gs_config_.term_tns_improve_frac,
      gs_config_.term_power_improve_frac,
      gs_config_.term_improve_window,
      gs_config_.term_wall_limit_s,
      gs_config_.best_tns_target_frac,
      gs_config_.size_registers,
      toString(gs_config_.power_objective),
      gs_config_.power_phase_filter,
      gs_config_.arc_slack_k_timing_crit,
      gs_config_.arc_slack_k_timing_noncrit,
      gs_config_.arc_slack_k_power_crit,
      gs_config_.arc_slack_k_power_noncrit,
      gs_config_.max_inner_sweeps,
      gs_config_.restart_each_iteration,
      toString(gs_config_.timing_cost),
      gs_config_.cap_fix_pass,
      gs_config_.slew_fix_pass,
      gs_config_.relax_max_cap);
}

void GlobalSizingPolicy::cacheActivities()
{
  ObjectivePower& power = state_.objective_power;
  if (!power.totalPower()) {
    return;
  }
  power.cacheActivities(sta_, sta_->cmdScene());
  if (!logger_->debugCheck(RSZ, "global_sizing_activity", 1)) {
    return;
  }
  // In the format of the `activity` pin property, so a script can compare the
  // two.
  std::unique_ptr<sta::LeafInstanceIterator> iit(
      network_->leafInstanceIterator());
  while (iit->hasNext()) {
    std::unique_ptr<sta::InstancePinIterator> pit(
        network_->pinIterator(iit->next()));
    while (pit->hasNext()) {
      const sta::Pin* pin = pit->next();
      if (const PinActivity* activity = power.activity(pin)) {
        debugPrint(logger_,
                   RSZ,
                   "global_sizing_activity",
                   1,
                   "activity {} {:.5e} {:.3f}",
                   network_->pathName(pin),
                   activity->density,
                   activity->duty);
      }
    }
  }
}

void GlobalSizingPolicy::runEstimationLoop(const float timing_weight)
{
  const int est_iters = std::max(0, gs_config_.est_loop_iters);
  if (est_iters == 0) {
    return;
  }
  // Reimann Alg. 2, first loop. setEstimationPhase switches the reimann
  // updater's k schedule to k_est; it is a no-op for the other updaters.
  updater_->setEstimationPhase(true);
  for (int i = 0; i < est_iters; ++i) {
    resizer_.journalBegin();
    // Commit one subproblem solve so the updater sees the resulting timing,
    // update lambda from it, then roll the solve back through the journal so
    // only lambda carries forward. This works for any sweep engine, since all
    // of them journal their commits. journalRestore refreshes parasitics and
    // required times itself, so the next iteration (and the main loop) start
    // from the initial timing.
    solveSubproblem(timing_weight);
    // The solve's swaps can re-create edges; the update reads them.
    state_.refreshLiveEdges();
    updater_->update(state_, i + 1);
    updateCapMultipliers();
    // Never the run's first projection (the post-seed projection in iterate()
    // is), so the endpoint-pressure μ policies do not derive μ here.
    projection_->project(state_, /*first_projection=*/false);
    resizer_.journalRestore();
  }
  updater_->setEstimationPhase(false);
  debugPrint(logger_,
             RSZ,
             "global_sizing",
             2,
             "LR estimation loop: {} dry-run iterations, lambda warm-started",
             est_iters);
}

void GlobalSizingPolicy::prepareCostTerms()
{
  // Both passes read the current (pre-sweep) timing on the main thread and
  // fill per-edge vectors the workers read. computePhiSensitivities uses the
  // current lambda, so it must run after the per-iteration lambda update and
  // projection.
  if (gs_config_.cost_global_phi) {
    computePhiSensitivities(state_);
  }
  if (gs_config_.cost_delta_delay) {
    captureReferenceDelays(state_);
  }
}

GlobalSizingPolicy::SubproblemStats GlobalSizingPolicy::solveSubproblem(
    const float timing_weight)
{
  SubproblemStats stats;
  // Livramento et al., Alg. 1 line 5, before the run's first sweep in the
  // main loop or the estimation loop, so that beta0 is set for the timing
  // weight that sweep uses.
  if (gs_config_.relax_max_cap && !state_.cap_multipliers.started()) {
    state_.cap_multipliers.start(state_, timing_weight);
  }
  int kept = 0;
  do {
    // Covers the previous sweep's swaps and the estimation loop's restore. The
    // per-edge cost-term vectors follow lambda's size.
    state_.refreshLiveEdges();
    prepareCostTerms();
    const SweepEngine::Stats sweep
        = sweep_engine_->sweep(state_, timing_weight);
    estimate_parasitics_->updateParasitics();
    sta_->findRequireds();
    if (gs_config_.cap_fix_pass) {
      // Livramento et al., Alg. 2 line 23. The pass prices cells with the
      // multipliers, so it needs the edges this sweep's swaps re-created.
      state_.refreshLiveEdges();
      stats.cap_fix
          += sweep_engine_->fixMaxCapViolations(state_, timing_weight);
    }
    ++stats.sweeps;
    stats.totals.moves += sweep.moves;
    stats.totals.upsizes += sweep.upsizes;
    stats.totals.downsizes += sweep.downsizes;
    stats.totals.cap_reverts += sweep.cap_reverts;
    stats.totals.cap_recheck_bound_hit |= sweep.cap_recheck_bound_hit;
    // Net change: a sweep whose moves the cap re-check all undid leaves the
    // netlist as it was, so repeating it would repeat the same moves. Under
    // relax_max_cap the re-check undoes only moves on nets without a
    // max-capacitance multiplier.
    kept = sweep.moves - sweep.cap_reverts;
    debugPrint(logger_,
               RSZ,
               "global_sizing",
               2,
               "LR inner loop: sweep {} made {} moves, {} undone by the cap "
               "re-check",
               stats.sweeps,
               sweep.moves,
               sweep.cap_reverts);
  } while (kept > 0 && stats.sweeps < gs_config_.max_inner_sweeps);
  stats.hit_sweep_cap = kept > 0;
  return stats;
}

void GlobalSizingPolicy::logInnerLoopSummary(const int sweeps,
                                             const int iterations,
                                             const int sweep_cap_iters) const
{
  if (gs_config_.max_inner_sweeps <= 1) {
    return;
  }
  logger_->info(RSZ,
                455,
                "GLOBAL_SIZING inner loop: {} sweep(s) in {} iteration(s); {} "
                "iteration(s) stopped at the cap of {} sweeps.",
                sweeps,
                iterations,
                sweep_cap_iters,
                gs_config_.max_inner_sweeps);
}

void GlobalSizingPolicy::logCapFixSummary(const RepairWalkStats& stats,
                                          const int sweeps) const
{
  if (!gs_config_.cap_fix_pass) {
    return;
  }
  logger_->info(RSZ,
                459,
                "GLOBAL_SIZING cap fix pass: {} gate(s) over their max "
                "capacitance, summed over {} sweep(s); {} resized to clear "
                "it, {} resized but still over it, {} left as they were.",
                stats.violating,
                sweeps,
                stats.cleared,
                stats.replaced - stats.cleared,
                stats.violating - stats.replaced);
}

void GlobalSizingPolicy::updateCapMultipliers()
{
  if (gs_config_.relax_max_cap) {
    state_.cap_multipliers.update(state_);
  }
}

void GlobalSizingPolicy::logCapMultipliers() const
{
  if (!gs_config_.relax_max_cap) {
    return;
  }
  const CapMultipliers& multipliers = state_.cap_multipliers;
  logger_->info(RSZ,
                464,
                "GLOBAL_SIZING max-cap multipliers: beta0={:.3g} on {} driver "
                "pin(s); {} over their max capacitance at the start, {} at the "
                "end.",
                multipliers.beta0(),
                multipliers.pricedPins(),
                multipliers.violatingAtStart(),
                multipliers.countViolating(state_));
}

void GlobalSizingPolicy::captureInitialCells()
{
  if (gs_config_.restart_each_iteration) {
    initial_cells_.capture(state_);
  }
}

void GlobalSizingPolicy::restartFromInitialCells(const int iter)
{
  if (!gs_config_.restart_each_iteration) {
    return;
  }
  // The record also holds gates the sweep never sizes. They still have their
  // recorded cells, so the restore skips them.
  const int restored = initial_cells_.restore(state_);
  // A restored cell can have other timing arcs than the cell it replaces, so
  // the swap can re-create edges. The refresh gives them the multipliers just
  // projected, before anything reads them.
  state_.refreshLiveEdges();
  estimate_parasitics_->updateParasitics();
  sta_->findRequireds();
  debugPrint(logger_,
             RSZ,
             "global_sizing",
             2,
             "LR restart: iteration {} set {} cells back to their initial "
             "cells",
             iter + 1,
             restored);
}

void GlobalSizingPolicy::logEdgeCarry() const
{
  const EdgeCarryStats& carry = state_.edge_carry;
  if (!carry.any()) {
    return;
  }
  logger_->info(RSZ,
                456,
                "GLOBAL_SIZING multipliers: {} re-created timing edge(s) took "
                "their pins' earlier multipliers; {} started at 0.",
                carry.carried,
                carry.started_at_zero);
}

bool GlobalSizingPolicy::start()
{
  if (!OptimizationPolicy::start()) {
    return false;
  }
  gs_config_ = resizer_.globalSizingConfig();
  // Resolve the lambda/mu pairing on this run's copy of the config before
  // validating, so validate() and the config log line (RSZ-0417) see the
  // effective mu policy rather than the one the preset left.
  gs_config_.resolveLambdaMuPairing(logger_);
  if (!gs_config_.validate(logger_)) {
    return false;
  }
  db_network_ = resizer_.dbNetwork();

  // Read-only handles shared with every strategy (base start() has set sta_ /
  // network_ / graph_). dcalc_ap is filled by iterate() before the seed.
  state_.sta = sta_;
  state_.network = network_;
  state_.db_network = db_network_;
  state_.graph = graph_;
  state_.resizer = &resizer_;
  state_.logger = logger_;
  state_.config = &gs_config_;
  state_.max = policy_max_;
  state_.objective_power.setTotalPower(
      gs_config_.power_objective == GlobalSizingConfig::PowerObjective::kTotal);

  // Phase B fans the per-gate evaluations across the OpenROAD thread budget
  // (threadCount()-1 workers; a zero-worker pool runs inline). Each worker
  // reads only the frozen snapshots, read-only Liberty/SDC, and its own
  // ArcDelayCalc copy, so results are independent of worker count and the
  // apply order stays the snapshot vector order.
  thread_pool_ = makeWorkerThreadPool();

  // Construct the strategies once from this run's config.
  init_pass_ = makeInitPass(gs_config_);
  seeder_ = makeLambdaSeeder(gs_config_);
  updater_ = makeLambdaUpdater(gs_config_);
  projection_ = makeFlowProjection(gs_config_);
  sweep_engine_ = makeSweepEngine(gs_config_, &resizer_, thread_pool_.get());
  best_tracker_ = makeBestTracker(gs_config_);
  termination_ = makeTermination(gs_config_);

  logEffectiveConfig();
  return true;
}

void GlobalSizingPolicy::iterate()
{
  if (converged_) {
    return;
  }

  // Before the input metrics are taken and before any cell is swapped.
  cacheActivities();

  const DesignSnap pre = computeDesignSnap();
  const float wns_pre = sta::delayAsFloat(sta_->worstSlack(policy_max_));
  const float tns_pre
      = sta::delayAsFloat(sta_->totalNegativeSlack(policy_max_));
  debugPrint(logger_,
             RSZ,
             "global_sizing",
             1,
             "Pre-global sizing design: instances={} (with leakage={}) "
             "leakage={:.3g}W area={:.3g}m^2 WNS={} TNS={}",
             pre.instances,
             pre.with_leakage,
             pre.total_leakage,
             pre.total_area,
             sta::delayAsString(wns_pre, 3, sta_),
             sta::delayAsString(tns_pre, 1, sta_));

  // The solution handed to this phase. best_tracker = reimann_score measures
  // every iterate against it (Reimann Eq. 6 uses deltas relative to the input
  // solution).
  state_.metrics_init
      = IterMetrics{.wns = wns_pre,
                    .tns = tns_pre,
                    .leakage = static_cast<float>(pre.total_leakage),
                    .power = static_cast<float>(pre.total_power),
                    .area = static_cast<float>(pre.total_area)};
  // The tracker must see the input before the init pass runs.
  best_tracker_->recordInput(state_);

  // Outer journal: wraps the init pass + LR so the inner LR-loop checkpoints
  // nest under one phase-level ECO (committed at the end; see the journalEnd
  // below).
  resizer_.journalBegin();

  init_pass_->run(state_);
  captureInitialCells();

  // STA preamble the seed / sweep / DRC rely on, plus the analysis point for
  // arc-delay reads. Then size the multiplier vectors from the current graph.
  sta_->findRequireds();
  sta_->checkCapacitancesPreamble(sta_->scenes());
  sta_->checkSlewsPreamble();
  sta_->checkFanoutPreamble();
  const sta::Scene* scene = sta_->cmdScene();
  state_.dcalc_ap = scene->dcalcAnalysisPtIndex(policy_max_);
  state_.allocate();
  // Snapshot the pre-LR timing (clock period, worst slack, per-vertex slack)
  // for updaters that reference the initial solution (reimann_dwns).
  state_.captureInitialTiming();

  seeder_->seed(state_);
  // The run's first projection: endpoint_ratio / endpoint_additive derive their
  // initial μ from the seeded λ here (μ_0 ∝ the seed magnitude).
  projection_->project(state_, /*first_projection=*/true);

  sweep_engine_->init(state_);

  // Base timing weight (see TimingScale.hh). Every timing_scale option
  // computes it once here, from the seeded and projected multipliers, and
  // keeps it for the whole run.
  const float timing_weight_base
      = sweep_engine_->computeTimingWeightBase(state_);
  // timing_scale = livramento_alpha divides the base by Livramento's α, the
  // only part of the scale that changes per iteration; livramento_alpha0 is
  // its initial value (Livramento Alg. 1 line 6). Unused by other options.
  float livramento_alpha = gs_config_.livramento_alpha0;
  // How many times α was rescheduled and whether its floor ever clamped.
  // Reported once at loop exit (RSZ-0444).
  int livramento_reschedules = 0;
  bool livramento_alpha_floor_bound = false;

  if (gs_config_.lambda_seed
      == GlobalSizingConfig::LambdaSeed::kEstimationLoop) {
    runEstimationLoop(timing_weight_base);
  }

  const int max_iter = termination_->maxIterations(state_);
  const float wns_eps = 1e-12f;

  const bool trace_level_1 = logger_->debugCheck(RSZ, "global_sizing", 1);

  int total_committed = 0;
  int total_attempted = 0;
  int total_upsizes = 0;
  int total_downsizes = 0;
  int total_cap_reverts = 0;
  bool cap_recheck_bound_hit = false;
  // Sweeps run by the main loop, and the iterations whose subproblem solve
  // stopped at max_inner_sweeps. Reported once at loop exit (RSZ-0455).
  int total_sweeps = 0;
  int sweep_cap_iters = 0;
  // What the max-capacitance fix passes did. Reported once at loop exit
  // (RSZ-0459).
  RepairWalkStats total_cap_fix;
  int accepted_iters = 0;
  int rejected_iters = 0;
  // The best tracker owns the pass-level journal, because whether an
  // iteration's sweeps are kept is a best-solution decision (see
  // BestTracker). wns_pass_reject checkpoints after every iteration that does
  // not regress WNS and undoes the rest in endLoop(); the other options keep
  // every pass and reinstate the best cell assignment in restore().
  best_tracker_->beginLoop(state_);
  for (int iter = 0; iter < max_iter; ++iter) {
    // Publish the iteration index for the strategies that are not handed one
    // (the sweep engines and, through them, the candidate move set).
    state_.iter = iter;
    if (termination_->stopBeforeSweep(state_, iter)) {
      break;
    }

    // Covers the previous iteration's swaps and a rejected pass's undo, before
    // the updater and projection read λ.
    state_.refreshLiveEdges();

    // WNS of the current solution: the previous iteration's result, or the
    // initial solution at iteration 0. The near-met latch and Livramento's α
    // follow the solutions, so they read it rather than the WNS after a
    // restart, which is the same initial-cell WNS at every iteration.
    const float wns_current = sta::delayAsFloat(sta_->worstSlack(policy_max_));

    if (iter > 0) {
      updater_->update(state_, iter);
      updateCapMultipliers();
      projection_->project(state_, /*first_projection=*/false);
      restartFromInitialCells(iter);
    }

    // The WNS the subproblem solve starts from, read after the restart, so the
    // WNS-regression test below measures the solve, not the cells the restart
    // set back. Without a restart it equals wns_current.
    const float wns0 = sta::delayAsFloat(sta_->worstSlack(policy_max_));

    // Near-met phase latch. Only the driver writes it; strategies read
    // state_.near_met. Updated from the current WNS so this iteration's veto
    // and stagnation check both see the current phase. Once set it stays set;
    // with near_met_gate_frac < 0 it is set here at iteration 0. See
    // nearMetLatched.
    state_.near_met = nearMetLatched(
        state_.near_met, gs_config_.near_met_gate_frac, state_.T, wns_current);

    // timing_scale = livramento_alpha. Livramento Alg. 1 runs STA (line 8),
    // then reschedules α (line 9), then solves the LRS with it (line 10); the
    // λ update (lines 11-16) and the projection (line 17) come after.
    // wns_current is that STA result (the λ update changes multipliers, not
    // timing), so rescheduling here and sweeping below follows the paper's
    // order. The other options use the fixed base weight.
    float timing_weight = timing_weight_base;
    if (gs_config_.timing_scale
        == GlobalSizingConfig::TimingScale::kLivramentoAlpha) {
      livramento_alpha
          = rescheduleLivramentoAlpha(livramento_alpha,
                                      state_.T,
                                      wns_current,
                                      &livramento_alpha_floor_bound);
      ++livramento_reschedules;
      // Livramento's α weights the power term (Eq. 1, Eq. 7, Alg. 2 line 10).
      // This objective keeps the power term at weight 1, so the timing weight
      // is scaled by 1/α instead. rescheduleLivramentoAlpha floors its
      // result, so α is always positive and the division is safe.
      timing_weight = timing_weight_base / livramento_alpha;
      debugPrint(
          logger_,
          RSZ,
          "global_sizing",
          2,
          "LR livramento alpha: iter={} WNS={} alpha={:.3g} -> tw={:.3g}",
          iter,
          sta::delayAsString(wns_current, 3, sta_),
          livramento_alpha,
          timing_weight);
    }

    const SubproblemStats subproblem = solveSubproblem(timing_weight);
    const SweepEngine::Stats& totals = subproblem.totals;
    total_sweeps += subproblem.sweeps;
    if (subproblem.hit_sweep_cap) {
      ++sweep_cap_iters;
    }
    total_cap_fix += subproblem.cap_fix;
    // Moves the iteration kept: its commits minus those undone by the
    // post-sweep max-cap re-checks (CapRecheck.hh). The attempted, upsize and
    // downsize totals count every commit.
    const int iter_moves = totals.moves - totals.cap_reverts;
    total_cap_reverts += totals.cap_reverts;
    cap_recheck_bound_hit |= totals.cap_recheck_bound_hit;
    const float wns1 = sta::delayAsFloat(sta_->worstSlack(policy_max_));

    // From here on, everything runs once per iteration, over all of its
    // sweeps.
    const float wns_delta = wns1 - wns0;
    // "This iteration found nothing to do", as the termination rules use it.
    // It uses the raw commit count, not the kept count: a sweep whose moves
    // were all undone by the cap re-check did find improving candidates, and
    // the next iteration sweeps with updated multipliers. Counting it as a
    // zero-move pass would let the cap re-check trigger fixed_iters'
    // consecutive zero-move exit.
    const bool no_benefit = (totals.moves == 0);
    // A measurement, not a decision: did this iteration make WNS worse (by
    // more than wns_eps)? Each strategy decides what to do with it: the
    // fixed_iters termination counts consecutive regressions, norm_subgradient
    // halves its step, and the wns_pass_reject tracker rolls the pass back.
    // Keeping the measurement here and the reactions in the strategies keeps
    // those options independent of each other.
    const bool wns_regressed = sta::fuzzyLess(wns_delta, -wns_eps);

    total_attempted += totals.moves;
    total_upsizes += totals.upsizes;
    total_downsizes += totals.downsizes;

    // The updater's own reaction (alpha halving under norm_subgradient; a
    // no-op for the other updaters).
    if (wns_regressed) {
      updater_->onPassRejected();
    }

    // The best tracker's pass policy, which owns the journal checkpoint. Its
    // verdict, not the raw measurement, is what "accepted" means below, so the
    // counters and the trace report what happened to the pass: a tracker that
    // keeps every pass never rejects one.
    const bool pass_rejected
        = best_tracker_->considerPass(state_, wns_regressed);
    if (pass_rejected) {
      ++rejected_iters;
    } else {
      total_committed += iter_moves;
      ++accepted_iters;
    }

    // Design metrics of this iterate, used by the termination rule, the best
    // tracker and the level-1 trace. Computing them walks every instance, so
    // it is skipped when nothing reads them (e.g. best=wns_pass_reject and
    // term=fixed_iters with the trace off).
    const bool want_metrics = trace_level_1 || best_tracker_->needsMetrics()
                              || termination_->needsMetrics();
    IterMetrics metrics;
    if (want_metrics) {
      const DesignSnap iter_snap = computeDesignSnap();
      metrics = IterMetrics{
          .wns = wns1,
          .tns = sta::delayAsFloat(sta_->totalNegativeSlack(policy_max_)),
          .leakage = static_cast<float>(iter_snap.total_leakage),
          .power = static_cast<float>(iter_snap.total_power),
          .area = static_cast<float>(iter_snap.total_area)};
    }
    best_tracker_->consider(state_, iter, metrics);

    if (trace_level_1) {
      // λ statistics over the active (data-arc) multipliers only.
      float lmin = 0.0f;
      float lmax = 0.0f;
      float lsum = 0.0f;
      int lcount = 0;
      for (const float l : state_.lambda) {
        if (l > 0.0f) {
          lmin = (lcount == 0) ? l : std::min(lmin, l);
          lmax = std::max(lmax, l);
          lsum += l;
          ++lcount;
        }
      }
      const float lmean = lcount ? lsum / static_cast<float>(lcount) : 0.0f;
      // Lagrangian value L(x, λ) at the current iterate - a diagnostic only
      // (see lagrangianEstimate: it is neither Q(λ) nor a bound in the discrete
      // setting). No control decision reads it; reported as `lag=`.
      const LagrangianTerms lag_terms = computeLagrangianTerms(state_);
      const float lag = lagrangianEstimate(metrics.power,
                                           timing_weight,
                                           lag_terms.lambda_delay_sum,
                                           lag_terms.mu_required_sum);
      // Fixed key=value order → trivially parseable for convergence plots.
      debugPrint(logger_,
                 RSZ,
                 "global_sizing",
                 1,
                 "iter={} wns={:.6g} tns={:.6g} leakage={:.6g}{} area={:.6g} "
                 "up={} down={} accepted={} alpha={:.3g} lmin={:.3g} "
                 "lmax={:.3g} lmean={:.3g} lsum={:.3g} lag={:.6g}",
                 iter + 1,
                 metrics.wns,
                 metrics.tns,
                 metrics.leakage,
                 totalPowerField(state_, metrics),
                 metrics.area,
                 totals.upsizes,
                 totals.downsizes,
                 pass_rejected ? 0 : 1,
                 updater_->currentStep(),
                 lmin,
                 lmax,
                 lmean,
                 lsum,
                 lag);
    }

    const bool stop = termination_->stopAfterSweep(
        state_, iter, wns_regressed, no_benefit, metrics);
    // The stop check may have handed over to the power phase. The next
    // iteration sweeps in the phase it decided, while the next multiplier
    // update still closes this iteration (see LrState::prev_sweep_power_phase).
    state_.prev_sweep_power_phase = state_.power_phase;
    state_.power_phase = termination_->inPowerPhase();
    if (stop) {
      break;
    }
  }

  // End-of-loop hook for the termination rule. A rule that logs a run summary
  // logs it here too, so the summary also appears when the loop stops at
  // max_iterations. Only the threshold-battery rule logs anything.
  termination_->reportRunEnd(state_);

  // Close the pass-level journal the tracker opened: wns_pass_reject undoes the
  // drift past its last checkpoint here; every other option commits the passes.
  best_tracker_->endLoop(state_);

  // Reinstate the best iterate the tracker recorded. This runs after the
  // pass-level journal is closed, so the tracker's choice is final. (The swaps
  // land in the phase-level journal, which is committed below.) The
  // replacements need a parasitics and timing update before the QoR numbers
  // below are read.
  if (best_tracker_->restore(state_)) {
    estimate_parasitics_->updateParasitics();
    sta_->findRequireds();
  }

  // Commit the phase ECO. There is no end-of-phase WNS check: power recovery
  // spends positive slack, so requiring WNS_after >= WNS_pre would discard
  // valid leakage savings on a design that meets timing, and none of the
  // papers has such a rule. A do-no-harm rule, if wanted, belongs in
  // BestTracker with the other selection rules.
  resizer_.journalEnd();

  const DesignSnap post = computeDesignSnap();
  const float wns_post = sta::delayAsFloat(sta_->worstSlack(policy_max_));
  const float tns_post
      = sta::delayAsFloat(sta_->totalNegativeSlack(policy_max_));
  const auto rel = [](double after, double before) {
    return before > 0.0 ? 100.0 * (after - before) / before : 0.0;
  };
  const int total_iters = accepted_iters + rejected_iters;

  // Headline: kept moves vs. attempted moves. They diverge when sweeps are
  // rolled back by the wns_pass_reject pass check (the only rule that rejects
  // a pass), when the max-cap re-check reverts moves, or when the end-of-loop
  // restore reverts drift past the best iterate. Its "sweeps" count
  // iterations; with max_inner_sweeps > 1, RSZ-0455 below counts the sweeps.
  logger_->info(RSZ,
                400,
                "GLOBAL_SIZING: {} cells replaced (loop); "
                "{}/{} sweeps accepted, {} rolled back; "
                "{} replacements attempted in total "
                "({} upsize, {} downsize).",
                total_committed,
                accepted_iters,
                total_iters,
                rejected_iters,
                total_attempted,
                total_upsizes,
                total_downsizes);
  logInnerLoopSummary(total_sweeps, total_iters, sweep_cap_iters);
  logCapFixSummary(total_cap_fix, total_sweeps);
  logCapMultipliers();
  logEdgeCarry();

  // Final α of timing_scale = livramento_alpha, and whether its floor ever
  // clamped. The floor keeps base/α finite on a design that never meets
  // timing; without the flag, a final α at the floor could not be told apart
  // from a schedule that simply converged low.
  if (gs_config_.timing_scale
      == GlobalSizingConfig::TimingScale::kLivramentoAlpha) {
    logger_->info(RSZ,
                  444,
                  "GLOBAL_SIZING timing_scale=livramento_alpha: "
                  "terminal alpha={:.6g} (alpha0={:.3g}, {} reschedules); "
                  "alpha_floor_bound={} (floor={:.3g}).",
                  livramento_alpha,
                  gs_config_.livramento_alpha0,
                  livramento_reschedules,
                  livramento_alpha_floor_bound,
                  kLivramentoAlphaFloor);
  }

  // Always reported, even when zero. Reverts are moves the sweep committed and
  // the post-sweep max-cap re-check undid because they pushed a net past its
  // max-cap limit. They are counted in the attempted totals of RSZ-0400 above
  // but not in its kept total.
  logger_->info(RSZ,
                443,
                "GLOBAL_SIZING cap re-check: {} move(s) reverted for creating "
                "a max-cap violation the sweep's frozen veto could not see; "
                "pass_bound_hit={}.",
                total_cap_reverts,
                cap_recheck_bound_hit);

  // QoR before -> after. This is the line that answers "what did it improve
  // and what did it regress" -- read the arrows, not just the deltas.
  logger_->info(RSZ,
                409,
                "GLOBAL_SIZING QoR: "
                "WNS {} -> {} ({}); "
                "TNS {} -> {} ({}); "
                "leakage {:.3g} -> {:.3g}W ({:+.2f}%); "
                "area {:.3g} -> {:.3g}m^2 ({:+.2f}%).",
                sta::delayAsString(wns_pre, 3, sta_),
                sta::delayAsString(wns_post, 3, sta_),
                sta::delayAsString(wns_post - wns_pre, 3, sta_),
                sta::delayAsString(tns_pre, 1, sta_),
                sta::delayAsString(tns_post, 1, sta_),
                sta::delayAsString(tns_post - tns_pre, 1, sta_),
                pre.total_leakage,
                post.total_leakage,
                rel(post.total_leakage, pre.total_leakage),
                pre.total_area,
                post.total_area,
                rel(post.total_area, pre.total_area));

  // Explain the all-zero summary case explicitly: the design did get
  // churned, but every sweep blew the WNS guard so every pass was
  // rolled back and the netlist is back to where it started.
  if (total_committed == 0 && total_attempted > 0) {
    logger_->info(RSZ,
                  412,
                  "GLOBAL_SIZING: nothing kept -- all {} rejected sweeps "
                  "regressed WNS and were rolled back by the "
                  "best_tracker=wns_pass_reject pass check; the netlist is "
                  "unchanged from the start of this phase. "
                  "The {} attempted replacements were tentative only.",
                  rejected_iters,
                  total_attempted);
  }

  markRunComplete(true);
}

}  // namespace rsz
