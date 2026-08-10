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
#include "lr/CostTerms.hh"
#include "lr/FlowProjection.hh"
#include "lr/InitPass.hh"
#include "lr/LambdaSeeder.hh"
#include "lr/LambdaUpdater.hh"
#include "lr/LrState.hh"
#include "lr/SweepEngine.hh"
#include "lr/Termination.hh"
#include "lr/TimingScale.hh"
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
  // §3.4 run header: the authoritative echo of the effective config (preset +
  // every axis + every scalar), always on, and the per-run record the ablation
  // harness parses.
  //
  // It COVERS the same axes as report_global_sizing_config but is not a mirror
  // of it, and the difference is load-bearing: that proc reads the dbProperties
  // and reports what the user SET (printing `undefined` for a knob with no
  // property), while this reports the EFFECTIVE value after a preset bundle and
  // every override. On a preset-less run they therefore say the same thing in
  // two words - the proc's `-preset:  undefined` and this line's
  // `preset=unset`.
  //
  // `preset=` reports the preset that was REQUESTED, not the struct's default
  // value: without a `-preset`, no bundle was applied and the axes below are
  // struct defaults that merely coincide with rsz_baseline's. `unset` (rather
  // than `none`) because the collector's own per-run line already spells
  // "the engine never ran" as `gs=none` (ORFS genSummary.py), and two
  // different facts must not share a word in the harness record.
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
      "best_tns_frac={:.3g}",
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
      gs_config_.best_tns_target_frac);
}

void GlobalSizingPolicy::runEstimationLoop(const float timing_weight)
{
  const int est_iters = std::max(0, gs_config_.est_loop_iters);
  if (est_iters == 0) {
    return;
  }
  // Reimann Alg. 2 loop 1. setEstimationPhase pins the reimann k-schedule to
  // k_est; a no-op for every other updater.
  updater_->setEstimationPhase(true);
  for (int i = 0; i < est_iters; ++i) {
    resizer_.journalBegin();
    // Commit one sweep so the updater sees the resulting timing, update lambda
    // from that would-be picture, then roll the sweep back through the journal
    // so only lambda carries forward. The rollback is engine-agnostic (any
    // SweepEngine's commits are journaled), which M4's Gauss-Seidel engine
    // reuses. journalRestore refreshes parasitics + required times itself, so
    // the next iteration (and the main loop) start from the initial timing.
    prepareCostTerms();
    sweep_engine_->sweep(state_, timing_weight);
    estimate_parasitics_->updateParasitics();
    sta_->findRequireds();
    updater_->update(state_, i + 1);
    // Estimation-loop projections are never the run's first (the main loop's
    // post-seed projection is); the endpoint-pressure policies derive μ there,
    // not here.
    projection_->project(state_, /*first_projection=*/false);
    resizer_.journalRestore();
  }
  updater_->setEstimationPhase(false);
  debugPrint(logger_,
             RSZ,
             "global_sizing",
             2,
             "LR estimation loop: {} dry-run sweeps, lambda warm-started",
             est_iters);
}

void GlobalSizingPolicy::prepareCostTerms()
{
  // Both passes read the current (pre-sweep) timing on the main thread and
  // freeze per-edge vectors the workers consume; guarded so the rsz_baseline
  // path does no extra work. computePhiSensitivities uses the current lambda,
  // so it runs after the per-iteration lambda update/projection.
  if (gs_config_.cost_global_phi) {
    computePhiSensitivities(state_);
  }
  if (gs_config_.cost_delta_delay) {
    captureReferenceDelays(state_);
  }
}

bool GlobalSizingPolicy::start()
{
  if (!OptimizationPolicy::start()) {
    return false;
  }
  gs_config_ = resizer_.globalSizingConfig();
  // Resolve the lambda/mu pairing on this run's frozen copy BEFORE validating,
  // so validate()'s own cross-axis rules (and RSZ-0417 below) see the effective
  // policy rather than the one the bundle happened to leave behind.
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

  // Phase B fans the per-gate evaluations across the OpenROAD thread budget
  // (threadCount()-1 workers; a zero-worker pool runs inline). Each worker
  // reads only the frozen snapshots, read-only Liberty/SDC, and its own
  // ArcDelayCalc copy, so results are independent of worker count and the
  // apply order stays the snapshot vector order.
  thread_pool_ = makeWorkerThreadPool();

  // Construct the per-axis strategies once from the frozen config.
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

  // The solution handed to this phase, which the reimann_score tracker measures
  // every iterate against (Eq. 6's deltas are relative to the input solution).
  state_.metrics_init
      = IterMetrics{.wns = wns_pre,
                    .tns = tns_pre,
                    .leakage = static_cast<float>(pre.total_leakage),
                    .area = static_cast<float>(pre.total_area)};

  // Outer journal: wraps the init pass + LR so the inner LR-loop checkpoints
  // nest under one phase-level ECO (committed at the end; see the journalEnd
  // below).
  resizer_.journalBegin();

  init_pass_->run(state_);

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

  // A1: the frozen design/library anchor (TimingScale.hh). Every timing_scale
  // option freezes it here, at iteration 0, off the seeded+projected field.
  const float timing_weight_base
      = sweep_engine_->computeTimingWeightBase(state_);
  // Livramento's α rides on top of that base and is the one per-iteration part
  // of the scale; livramento_alpha0 seeds it (Alg. 1 L6). Inert under every
  // other option, which never divide by it.
  float livramento_alpha = gs_config_.livramento_alpha0;
  // A-axis diagnostic (§2.2-5): how many times α was rescheduled and whether
  // its accumulator floor ever clamped. Reported once at loop exit (RSZ-0444).
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
  int accepted_iters = 0;
  int rejected_iters = 0;
  // H2 owns the pass-level journal: whether a sweep survives is a
  // best-solution decision, and the options answer it incompatibly (see
  // BestTracker). rsz_baseline's wns_pass_reject checkpoints on WNS
  // non-regression and drops the drift at endLoop; every other option keeps
  // all passes and selects by cell assignment in restore().
  best_tracker_->beginLoop(state_);
  for (int iter = 0; iter < max_iter; ++iter) {
    // Publish the iteration index for the strategies that are not handed one
    // (the sweep engines and, through them, the F4 move set).
    state_.iter = iter;
    if (termination_->stopBeforeSweep(state_, iter)) {
      break;
    }

    // C3 item 3: extend λ to the live edge-id space before the updater and
    // projection read it, so arcs the previous sweep's rebuild minted past the
    // allocate()-sized space (or the estimation loop's churn minted before
    // iteration 0) get a neutral slot and are priced from here on, instead of
    // being silently skipped and cascading a λ blackout at any fully-re-minted
    // vertex (tennakoon audit §5.4). Counted at debug level 2.
    state_.growToLiveEdges();

    if (iter > 0) {
      updater_->update(state_, iter);
      projection_->project(state_, /*first_projection=*/false);
    }

    const float wns0 = sta::delayAsFloat(sta_->worstSlack(policy_max_));

    // C2 near-met phase latch (driver-owned; strategies read state_.near_met
    // only). Updated from the fresh pre-sweep WNS so this iteration's veto and
    // this iteration's stagnation check both see the current phase. Permanent
    // once set; ungated presets (near_met_gate_frac < 0) latch it here at
    // iteration 0. See nearMetLatched / the sharma audit §2 items 3/6.
    state_.near_met = nearMetLatched(
        state_.near_met, gs_config_.near_met_gate_frac, state_.T, wns0);

    // A1/livramento_alpha: Alg. 1's order is load-bearing - STA (L8), then the
    // α reschedule (L9), then the LRS (L10) consumes it; only afterwards the λ
    // update (L11-16) and the projection (L17). wns0 is that fresh STA (the λ
    // update above moves multipliers, not timing), so rescheduling here and
    // sweeping below is the paper's sequence. Under every other option the
    // weight is the frozen base and this is a no-op.
    float timing_weight = timing_weight_base;
    if (gs_config_.timing_scale
        == GlobalSizingConfig::TimingScale::kLivramentoAlpha) {
      livramento_alpha = rescheduleLivramentoAlpha(
          livramento_alpha, state_.T, wns0, &livramento_alpha_floor_bound);
      ++livramento_reschedules;
      // 1/α: Livramento's α weights the POWER term (Eq. 1 / reduced Lagrangian
      // Eq. 7 / Alg. 2 L10), and our objective normalizes leakage to 1.
      // Unguarded by design: rescheduleLivramentoAlpha floors its result, so α
      // is always positive and base/α stays finite.
      timing_weight = timing_weight_base / livramento_alpha;
      debugPrint(
          logger_,
          RSZ,
          "global_sizing",
          2,
          "LR livramento alpha: iter={} WNS={} alpha={:.3g} -> tw={:.3g}",
          iter,
          sta::delayAsString(wns0, 3, sta_),
          livramento_alpha,
          timing_weight);
    }

    prepareCostTerms();
    const SweepEngine::Stats sweep
        = sweep_engine_->sweep(state_, timing_weight);
    // What the sweep KEPT: its commits less the moves its own post-sweep
    // max-cap re-check undid (CapRecheck.hh). The raw tallies stay attempt
    // counts.
    const int iter_moves = sweep.moves - sweep.cap_reverts;
    total_cap_reverts += sweep.cap_reverts;
    cap_recheck_bound_hit |= sweep.cap_recheck_bound_hit;
    estimate_parasitics_->updateParasitics();
    sta_->findRequireds();
    const float wns1 = sta::delayAsFloat(sta_->worstSlack(policy_max_));

    const float wns_delta = wns1 - wns0;
    // "This sweep found nothing to do", which is what the H1 termination rules
    // mean by it - deliberately the RAW commit count, not the kept one. A sweep
    // whose moves the cap re-check all undid did find improving candidates, and
    // the next sweep faces a different frozen picture; reading those as
    // no-benefit passes would let the cap channel drive fixed_iters'
    // consecutive-zero exit, moving the termination axis as a side effect of an
    // electrical fix.
    const bool no_benefit = (sweep.moves == 0);
    // Measurement, not a decision: "this sweep's WNS came out worse than the
    // pre-sweep WNS". Small regressions are deliberately allowed. Each strategy
    // decides for itself what to do with it - the fixed_iters termination
    // counts consecutive ones, norm_subgradient halves its step on one, and the
    // wns_pass_reject tracker rolls back on one. Keeping the measurement here
    // and the responses in the strategies is what keeps those three axes
    // independent.
    const bool wns_regressed = sta::fuzzyLess(wns_delta, -wns_eps);

    total_attempted += sweep.moves;
    total_upsizes += sweep.upsizes;
    total_downsizes += sweep.downsizes;

    // The updater's private reaction (alpha halving under norm_subgradient; a
    // no-op for every paper updater).
    if (wns_regressed) {
      updater_->onPassRejected();
    }

    // The H2 tracker's pass policy, which owns the journal checkpoint. Its
    // verdict - not the raw measurement - is what "accepted" means below, so
    // the counters and the trace field report what actually happened to the
    // pass: under a tracker that keeps every pass, nothing is ever rejected.
    const bool pass_rejected
        = best_tracker_->considerPass(state_, wns_regressed);
    if (pass_rejected) {
      ++rejected_iters;
    } else {
      total_committed += iter_moves;
      ++accepted_iters;
    }

    // The iterate's design metrics: the H1 termination rules and the H2 best
    // tracker decide on them, and the level-1 trace reports them. The design
    // walk that fills them is O(instances), so it only runs when something
    // actually reads it - the rsz_baseline bundle (best=wns_pass_reject,
    // term=fixed_iters, neither of which reads `metrics`) with the trace off
    // pays nothing, as before M5.
    const bool want_metrics = trace_level_1 || best_tracker_->needsMetrics()
                              || termination_->needsMetrics();
    IterMetrics metrics;
    if (want_metrics) {
      const DesignSnap iter_snap = computeDesignSnap();
      metrics = IterMetrics{
          .wns = wns1,
          .tns = sta::delayAsFloat(sta_->totalNegativeSlack(policy_max_)),
          .leakage = static_cast<float>(iter_snap.total_leakage),
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
      const float lag = lagrangianEstimate(metrics.leakage,
                                           timing_weight,
                                           lag_terms.lambda_delay_sum,
                                           lag_terms.mu_required_sum);
      // Fixed key=value order → trivially parseable for convergence plots.
      debugPrint(logger_,
                 RSZ,
                 "global_sizing",
                 1,
                 "iter={} wns={:.6g} tns={:.6g} leakage={:.6g} area={:.6g} "
                 "up={} down={} accepted={} alpha={:.3g} lmin={:.3g} "
                 "lmax={:.3g} lmean={:.3g} lsum={:.3g} lag={:.6g}",
                 iter + 1,
                 metrics.wns,
                 metrics.tns,
                 metrics.leakage,
                 metrics.area,
                 sweep.upsizes,
                 sweep.downsizes,
                 pass_rejected ? 0 : 1,
                 updater_->currentStep(),
                 lmin,
                 lmax,
                 lmean,
                 lsum,
                 lag);
    }

    if (termination_->stopAfterSweep(
            state_, iter, wns_regressed, no_benefit, metrics)) {
      break;
    }
  }

  // H1's end-of-loop hook: an option that publishes a run record publishes it
  // here too, so the record exists on the max_iterations path and not only when
  // the option's own verdict ended the loop. Every option but the threshold
  // battery says nothing.
  termination_->reportRunEnd(state_);

  // Close the pass-level journal the tracker opened: wns_pass_reject undoes the
  // drift past its last checkpoint here; every other option commits the passes.
  best_tracker_->endLoop(state_);

  // H2: reinstate the best iterate the tracker recorded. This runs after the
  // pass-level journal is closed, so nothing can roll its replacements back and
  // the tracker's choice is final. (Note they are not journaled at all: the
  // tracker's endLoop() left `journal_` null, so `journal=true` in restore() is
  // inert here. Harmless now that nothing arbitrates the phase result; it was
  // load-bearing when an end-of-phase accept could revert them.) The
  // replacements need a parasitics + timing refresh before the post-phase QoR
  // numbers are read.
  if (best_tracker_->restore(state_)) {
    estimate_parasitics_->updateParasitics();
    sta_->findRequireds();
  }

  // Commit the phase ECO. Deliberately NO end-of-phase WNS accept: power
  // recovery spends positive slack, so `WNS_after >= WNS_pre` discards
  // legitimate leakage wins on a met design and no paper has such a rule. If a
  // do-no-harm floor is ever wanted again it belongs on the H2 axis with the
  // other selection rules, not in this driver-owned accept. (History:
  // gs-guard's outer_guard, deleted 2026-07-29; see the plan's "engine fix 2".)
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
  // rolled back by the wns_pass_reject pass check (H2, the only rule that
  // rejects a pass), or when its end-of-loop restore reverts drift past the
  // best iterate.
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

  // A-axis terminal-α record (§2.2-5, the S1 §3.3 diagnostic ask). Only this
  // option has a live α, so only this option reports one - and it reports BOTH
  // halves: where the schedule ended up, and whether the accumulator floor ever
  // clamped on the way. The clamp is what keeps tw = base/α finite on a design
  // that never closes, so without the flag a terminal α sitting at the floor is
  // indistinguishable from a schedule that merely converged low, and the
  // α-runaway question stays an inference. No behaviour change.
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

  // The cap channel, always reported so a run's log says what the electrical
  // re-check cost it - "0" is a measurement, not a missing line (plan §2.2-1;
  // max-cap ERC decides essentially every loss tail, SYNTHESIS §3.3). Reverts
  // are moves the sweep committed and the post-sweep re-check undid because
  // they pushed a net past its max-cap limit; they are counted in the
  // "attempted" totals of RSZ-0400 above and not in its kept total.
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
