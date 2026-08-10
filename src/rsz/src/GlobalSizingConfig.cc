// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "rsz/GlobalSizingConfig.hh"

#include <cstring>

#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

const char* toString(const GlobalSizingConfig::InitMode mode)
{
  switch (mode) {
    case GlobalSizingConfig::InitMode::kAsGiven:
      return "as_given";
    case GlobalSizingConfig::InitMode::kMinSize:
      return "min_size";
    case GlobalSizingConfig::InitMode::kMaxSize:
      return "max_size";
    case GlobalSizingConfig::InitMode::kMinSizeFixviol:
      return "min_size_fixviol";
    case GlobalSizingConfig::InitMode::kRandom:
      return "random";
    case GlobalSizingConfig::InitMode::kAverage:
      return "average";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::LambdaSeed seed)
{
  switch (seed) {
    case GlobalSizingConfig::LambdaSeed::kDelayPropCritMu:
      return "delay_proportional_crit_mu";
    case GlobalSizingConfig::LambdaSeed::kConstant:
      return "constant";
    case GlobalSizingConfig::LambdaSeed::kStateAdaptive:
      return "state_adaptive";
    case GlobalSizingConfig::LambdaSeed::kEstimationLoop:
      return "estimation_loop";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::LambdaUpdate update)
{
  switch (update) {
    case GlobalSizingConfig::LambdaUpdate::kNormSubgradient:
      return "norm_subgradient";
    case GlobalSizingConfig::LambdaUpdate::kFlachSlackScaling:
      return "flach_slack_scaling";
    case GlobalSizingConfig::LambdaUpdate::kChenSubgradient:
      return "chen_subgradient";
    case GlobalSizingConfig::LambdaUpdate::kTennakoonRatio:
      return "tennakoon_ratio";
    case GlobalSizingConfig::LambdaUpdate::kSharmaCexp:
      return "sharma_cexp";
    case GlobalSizingConfig::LambdaUpdate::kReimannDwns:
      return "reimann_dwns";
    case GlobalSizingConfig::LambdaUpdate::kLivramentoRatio:
      return "livramento_ratio";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::MuPolicy policy)
{
  switch (policy) {
    case GlobalSizingConfig::MuPolicy::kReseedEachIter:
      return "reseed_each_iter";
    case GlobalSizingConfig::MuPolicy::kSeedOnce:
      return "seed_once";
    case GlobalSizingConfig::MuPolicy::kUpdateAsLambda:
      return "update_as_lambda";
    case GlobalSizingConfig::MuPolicy::kEndpointLambda:
      return "endpoint_lambda";
    case GlobalSizingConfig::MuPolicy::kEndpointRatio:
      return "endpoint_ratio";
    case GlobalSizingConfig::MuPolicy::kEndpointAdditive:
      return "endpoint_additive";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::KktProjection projection)
{
  switch (projection) {
    case GlobalSizingConfig::KktProjection::kProportionalReverseTopo:
      return "proportional_reverse_topo";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::SweepEngineKind engine)
{
  switch (engine) {
    case GlobalSizingConfig::SweepEngineKind::kJacobiSnapshot:
      return "jacobi_snapshot";
    case GlobalSizingConfig::SweepEngineKind::kGaussSeidelTopo:
      return "gauss_seidel_topo";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::GsRefresh refresh)
{
  switch (refresh) {
    case GlobalSizingConfig::GsRefresh::kLocal:
      return "gs_local";
    case GlobalSizingConfig::GsRefresh::kIncremental:
      return "gs_incremental";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::Traversal traversal)
{
  switch (traversal) {
    case GlobalSizingConfig::Traversal::kForwardTopo:
      return "forward_topo";
    case GlobalSizingConfig::Traversal::kReverseTopo:
      return "reverse_topo";
    case GlobalSizingConfig::Traversal::kCriticalitySorted:
      return "criticality_sorted";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::DownsizeGuard guard)
{
  switch (guard) {
    case GlobalSizingConfig::DownsizeGuard::kDepthBudget:
      return "depth_budget";
    case GlobalSizingConfig::DownsizeGuard::kLocalSlackVeto:
      return "local_slack_veto";
    case GlobalSizingConfig::DownsizeGuard::kNone:
      return "none";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::MoveSet move_set)
{
  switch (move_set) {
    case GlobalSizingConfig::MoveSet::kFullLibrary:
      return "full_library";
    case GlobalSizingConfig::MoveSet::kSharmaFastOlr:
      return "sharma_fast_olr";
    case GlobalSizingConfig::MoveSet::kMangirasSizeStep:
      return "mangiras_size_step";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::OutputDrcVeto veto)
{
  switch (veto) {
    case GlobalSizingConfig::OutputDrcVeto::kAbsolute:
      return "absolute";
    case GlobalSizingConfig::OutputDrcVeto::kRelative:
      return "relative";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::TimingScale scale)
{
  switch (scale) {
    case GlobalSizingConfig::TimingScale::kAutoMedian:
      return "auto_median";
    case GlobalSizingConfig::TimingScale::kUnit:
      return "unit";
    case GlobalSizingConfig::TimingScale::kLivramentoAlpha:
      return "livramento_alpha";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::TerminationKind termination)
{
  switch (termination) {
    case GlobalSizingConfig::TerminationKind::kFixedIters:
      return "fixed_iters";
    case GlobalSizingConfig::TerminationKind::kStagnationWindows:
      return "stagnation_windows";
    case GlobalSizingConfig::TerminationKind::kThresholdBattery:
      return "threshold_battery";
    case GlobalSizingConfig::TerminationKind::kPureCap:
      return "pure_cap";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::BestTrackerKind tracker)
{
  switch (tracker) {
    case GlobalSizingConfig::BestTrackerKind::kNone:
      return "none";
    case GlobalSizingConfig::BestTrackerKind::kFlachDominance:
      return "flach_dominance";
    case GlobalSizingConfig::BestTrackerKind::kReimannScore:
      return "reimann_score";
    case GlobalSizingConfig::BestTrackerKind::kWnsPassReject:
      return "wns_pass_reject";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::ReimannSetpoint setpoint)
{
  switch (setpoint) {
    case GlobalSizingConfig::ReimannSetpoint::kSInit:
      return "s_init";
    case GlobalSizingConfig::ReimannSetpoint::kSlackTarget:
      return "slack_target";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::Preset preset)
{
  switch (preset) {
    case GlobalSizingConfig::Preset::kRszBaseline:
      return "rsz_baseline";
    case GlobalSizingConfig::Preset::kChen:
      return "chen_partial";
    case GlobalSizingConfig::Preset::kTennakoon:
      return "tennakoon_partial";
    case GlobalSizingConfig::Preset::kFlach:
      return "flach_partial";
    case GlobalSizingConfig::Preset::kSharmaSeq:
      return "sharma_seq_partial";
    case GlobalSizingConfig::Preset::kReimann:
      return "reimann_partial";
    case GlobalSizingConfig::Preset::kMangiras:
      return "mangiras_partial";
    case GlobalSizingConfig::Preset::kLivramento:
      return "livramento_partial";
    case GlobalSizingConfig::Preset::kChinnery:
      return "chinnery_partial";
  }
  return "unknown";
}

bool parsePreset(const char* name, GlobalSizingConfig::Preset& out)
{
  if (std::strcmp(name, "rsz_baseline") == 0) {
    out = GlobalSizingConfig::Preset::kRszBaseline;
  } else if (std::strcmp(name, "chen_partial") == 0) {
    out = GlobalSizingConfig::Preset::kChen;
  } else if (std::strcmp(name, "tennakoon_partial") == 0) {
    out = GlobalSizingConfig::Preset::kTennakoon;
  } else if (std::strcmp(name, "flach_partial") == 0) {
    out = GlobalSizingConfig::Preset::kFlach;
  } else if (std::strcmp(name, "sharma_seq_partial") == 0) {
    out = GlobalSizingConfig::Preset::kSharmaSeq;
  } else if (std::strcmp(name, "reimann_partial") == 0) {
    out = GlobalSizingConfig::Preset::kReimann;
  } else if (std::strcmp(name, "mangiras_partial") == 0) {
    out = GlobalSizingConfig::Preset::kMangiras;
  } else if (std::strcmp(name, "livramento_partial") == 0) {
    out = GlobalSizingConfig::Preset::kLivramento;
  } else if (std::strcmp(name, "chinnery_partial") == 0) {
    out = GlobalSizingConfig::Preset::kChinnery;
  } else {
    return false;
  }
  return true;
}

bool parseInitMode(const char* name, GlobalSizingConfig::InitMode& out)
{
  using IM = GlobalSizingConfig::InitMode;
  if (std::strcmp(name, "as_given") == 0) {
    out = IM::kAsGiven;
  } else if (std::strcmp(name, "min_size") == 0) {
    out = IM::kMinSize;
  } else if (std::strcmp(name, "max_size") == 0) {
    out = IM::kMaxSize;
  } else if (std::strcmp(name, "min_size_fixviol") == 0) {
    out = IM::kMinSizeFixviol;
  } else if (std::strcmp(name, "random") == 0) {
    out = IM::kRandom;
  } else if (std::strcmp(name, "average") == 0) {
    out = IM::kAverage;
  } else {
    return false;
  }
  return true;
}

bool parseLambdaSeed(const char* name, GlobalSizingConfig::LambdaSeed& out)
{
  using LS = GlobalSizingConfig::LambdaSeed;
  if (std::strcmp(name, "delay_proportional_crit_mu") == 0) {
    out = LS::kDelayPropCritMu;
  } else if (std::strcmp(name, "constant") == 0) {
    out = LS::kConstant;
  } else if (std::strcmp(name, "state_adaptive") == 0) {
    out = LS::kStateAdaptive;
  } else if (std::strcmp(name, "estimation_loop") == 0) {
    out = LS::kEstimationLoop;
  } else {
    return false;
  }
  return true;
}

bool parseLambdaUpdate(const char* name, GlobalSizingConfig::LambdaUpdate& out)
{
  using LU = GlobalSizingConfig::LambdaUpdate;
  if (std::strcmp(name, "norm_subgradient") == 0) {
    out = LU::kNormSubgradient;
  } else if (std::strcmp(name, "flach_slack_scaling") == 0) {
    out = LU::kFlachSlackScaling;
  } else if (std::strcmp(name, "chen_subgradient") == 0) {
    out = LU::kChenSubgradient;
  } else if (std::strcmp(name, "tennakoon_ratio") == 0) {
    out = LU::kTennakoonRatio;
  } else if (std::strcmp(name, "sharma_cexp") == 0) {
    out = LU::kSharmaCexp;
  } else if (std::strcmp(name, "reimann_dwns") == 0) {
    out = LU::kReimannDwns;
  } else if (std::strcmp(name, "livramento_ratio") == 0) {
    out = LU::kLivramentoRatio;
  } else {
    return false;
  }
  return true;
}

bool parseMuPolicy(const char* name, GlobalSizingConfig::MuPolicy& out)
{
  using MP = GlobalSizingConfig::MuPolicy;
  if (std::strcmp(name, "reseed_each_iter") == 0) {
    out = MP::kReseedEachIter;
  } else if (std::strcmp(name, "seed_once") == 0) {
    out = MP::kSeedOnce;
  } else if (std::strcmp(name, "update_as_lambda") == 0) {
    out = MP::kUpdateAsLambda;
  } else if (std::strcmp(name, "endpoint_lambda") == 0) {
    out = MP::kEndpointLambda;
  } else if (std::strcmp(name, "endpoint_ratio") == 0) {
    out = MP::kEndpointRatio;
  } else if (std::strcmp(name, "endpoint_additive") == 0) {
    out = MP::kEndpointAdditive;
  } else {
    return false;
  }
  return true;
}

bool parseSweepEngine(const char* name,
                      GlobalSizingConfig::SweepEngineKind& out)
{
  using SE = GlobalSizingConfig::SweepEngineKind;
  if (std::strcmp(name, "jacobi_snapshot") == 0) {
    out = SE::kJacobiSnapshot;
  } else if (std::strcmp(name, "gauss_seidel_topo") == 0) {
    out = SE::kGaussSeidelTopo;
  } else {
    return false;
  }
  return true;
}

bool parseGsRefresh(const char* name, GlobalSizingConfig::GsRefresh& out)
{
  using GR = GlobalSizingConfig::GsRefresh;
  if (std::strcmp(name, "gs_local") == 0) {
    out = GR::kLocal;
  } else if (std::strcmp(name, "gs_incremental") == 0) {
    out = GR::kIncremental;
  } else {
    return false;
  }
  return true;
}

bool parseTraversal(const char* name, GlobalSizingConfig::Traversal& out)
{
  using TR = GlobalSizingConfig::Traversal;
  if (std::strcmp(name, "forward_topo") == 0) {
    out = TR::kForwardTopo;
  } else if (std::strcmp(name, "reverse_topo") == 0) {
    out = TR::kReverseTopo;
  } else if (std::strcmp(name, "criticality_sorted") == 0) {
    out = TR::kCriticalitySorted;
  } else {
    return false;
  }
  return true;
}

bool parseDownsizeGuard(const char* name,
                        GlobalSizingConfig::DownsizeGuard& out)
{
  using DG = GlobalSizingConfig::DownsizeGuard;
  if (std::strcmp(name, "depth_budget") == 0) {
    out = DG::kDepthBudget;
  } else if (std::strcmp(name, "local_slack_veto") == 0) {
    out = DG::kLocalSlackVeto;
  } else if (std::strcmp(name, "none") == 0) {
    out = DG::kNone;
  } else {
    return false;
  }
  return true;
}

bool parseMoveSet(const char* name, GlobalSizingConfig::MoveSet& out)
{
  using MS = GlobalSizingConfig::MoveSet;
  if (std::strcmp(name, "full_library") == 0) {
    out = MS::kFullLibrary;
  } else if (std::strcmp(name, "sharma_fast_olr") == 0) {
    out = MS::kSharmaFastOlr;
  } else if (std::strcmp(name, "mangiras_size_step") == 0) {
    out = MS::kMangirasSizeStep;
  } else {
    return false;
  }
  return true;
}

bool parseOutputDrcVeto(const char* name,
                        GlobalSizingConfig::OutputDrcVeto& out)
{
  using ODV = GlobalSizingConfig::OutputDrcVeto;
  if (std::strcmp(name, "absolute") == 0) {
    out = ODV::kAbsolute;
  } else if (std::strcmp(name, "relative") == 0) {
    out = ODV::kRelative;
  } else {
    return false;
  }
  return true;
}

bool parseTimingScale(const char* name, GlobalSizingConfig::TimingScale& out)
{
  using TS = GlobalSizingConfig::TimingScale;
  if (std::strcmp(name, "auto_median") == 0) {
    out = TS::kAutoMedian;
  } else if (std::strcmp(name, "unit") == 0) {
    out = TS::kUnit;
  } else if (std::strcmp(name, "livramento_alpha") == 0) {
    out = TS::kLivramentoAlpha;
  } else {
    return false;
  }
  return true;
}

bool parseTermination(const char* name,
                      GlobalSizingConfig::TerminationKind& out)
{
  using TK = GlobalSizingConfig::TerminationKind;
  if (std::strcmp(name, "fixed_iters") == 0) {
    out = TK::kFixedIters;
  } else if (std::strcmp(name, "stagnation_windows") == 0) {
    out = TK::kStagnationWindows;
  } else if (std::strcmp(name, "threshold_battery") == 0) {
    out = TK::kThresholdBattery;
  } else if (std::strcmp(name, "pure_cap") == 0) {
    out = TK::kPureCap;
  } else {
    return false;
  }
  return true;
}

bool parseBestTracker(const char* name,
                      GlobalSizingConfig::BestTrackerKind& out)
{
  using BT = GlobalSizingConfig::BestTrackerKind;
  if (std::strcmp(name, "none") == 0) {
    out = BT::kNone;
  } else if (std::strcmp(name, "flach_dominance") == 0) {
    out = BT::kFlachDominance;
  } else if (std::strcmp(name, "reimann_score") == 0) {
    out = BT::kReimannScore;
  } else if (std::strcmp(name, "wns_pass_reject") == 0) {
    out = BT::kWnsPassReject;
  } else {
    return false;
  }
  return true;
}

bool parseReimannSetpoint(const char* name,
                          GlobalSizingConfig::ReimannSetpoint& out)
{
  using RS = GlobalSizingConfig::ReimannSetpoint;
  if (std::strcmp(name, "s_init") == 0) {
    out = RS::kSInit;
  } else if (std::strcmp(name, "slack_target") == 0) {
    out = RS::kSlackTarget;
  } else {
    return false;
  }
  return true;
}

void GlobalSizingConfig::applyPreset(const Preset p)
{
  // Reset every axis and scalar to its struct default, then override only the
  // axes this preset changes. NOTE the struct defaults are *almost* the
  // rsz_baseline bundle: best_tracker deliberately defaults to the paper value
  // instead (the M5 default flip), so the baseline restores it explicitly
  // below. Every other field still defaults to what rsz_baseline wants. Scalar
  // knobs and init_mode set explicitly after the preset still win, because
  // Resizer::initBlock applies dbProperty overrides after this call. Paper
  // constants (flach_k_*, sharma_*, reimann_*, lambda_update_c) already hold
  // each paper's value as their struct default, so a paper preset only needs to
  // select its updater - its constants are read only by the matching updater.
  *this = GlobalSizingConfig{};
  preset = p;
  // Set here and nowhere else: this function IS "a preset was requested", so
  // the flag cannot drift from the fact it records. The reset above clears it
  // first, so it never survives into a config that no longer carries a preset.
  preset_explicit = true;
  switch (p) {
    case Preset::kRszBaseline:
      // The pre-M5 engine's best-solution rule is the inline best-WNS journal
      // checkpoint the driver used to run for every preset; the post-M5
      // hardening pass relocated it onto this axis as wns_pass_reject, and the
      // baseline is the only bundle that pins it (no paper has it). Together
      // with best_tracker's struct default now being flach_dominance (the M5
      // default flip, §3.2-H), that keeps rsz_baseline's H2 rule intact.
      // (Bit-for-bit with the PRE-ABLATION engine no longer holds - the two
      // WNS-conditioned mechanisms were removed from this bundle too; see the
      // Preset enum doc in GlobalSizingConfig.hh.)
      best_tracker = BestTrackerKind::kWnsPassReject;
      break;
    case Preset::kChen:
      // Chen-Chu-Wong ICCAD'98 (SYNTHESIS §4 column "chen 98").
      //
      // DEFERRED (hence _partial):
      //  - E3: Chen's exact theoretical projection; we only have
      //    proportional_reverse_topo.
      //  - F1: SOLVE_LRS/μ step 1 re-initializes x := L (every gate to its
      //    lower bound) at the START OF EVERY DUAL ITERATION, not once. The N
      //    axis is a one-time pre-LR pass by construction, so the init pass
      //    covers the FIRST LRS only; iterations 2..k warm-start from the
      //    previous sizes. That per-iteration restart is an F1/LRS behavior
      //    with no axis, and it is the paper's own step 1 - see chen_et_al.md
      //    §6 ablation 5 ("re-initialize x:=L each dual iteration (paper's step
      //    1) vs. warm-start from previous sizes"). Surfaced by the bucket-2
      //    pass.
      // ADAPTED (unavoidable under OpenSTA, not a deferral):
      //  - F1/F4: the paper sizes continuously and solves the LRS in closed
      //    form; our library is discrete, so the LRS is a discrete greedy.
      //  - A1: area objective -> leakageOrArea.
      //  - C: Chen's L_i is a lower bound on a continuous SIZE and the paper
      //    has no Vth dimension at all (1998); init_mode = min_size is the
      //    discrete library's expression of that corner (lowest-leakage
      //    equivalent cell, drive-strength tie-break).
      //  - H1: Chen's headline duality-gap certificate is untranslatable, not
      //    deferred - Q(λ) <= primal rests on the continuous posynomial
      //    relaxation, and NLDM delays over a discrete library are neither
      //    convex nor posynomial, so our Lagrangian estimate is a trace, not a
      //    bound (see lagrangianEstimate in lr/Termination.hh). Hence a pure
      //    cap (C2):
      //    termination = pure_cap runs the ρ_k = c/k schedule to max_iterations
      //    with none of fixed_iters' legacy early-exits. (Half of that argument
      //    was settled another way: fixed_iters' WNS-meets-margin exit, which
      //    would have halted the run at first feasibility, was DELETED on
      //    2026-07-29 - see Termination::stopBeforeSweep. What still justifies
      //    this pin is that fixed_iters' two surviving exits - 3 rejected
      //    sweeps or 2 zero-move sweeps - can truncate the ρ_k schedule before
      //    it has moved the design, and Chen's §8 min-area leakage-recovery
      //    regime is reached late. chen audit §2.4.)
      // RETAINED NON-PAPER CONSTRUCTION (deliberate, see below): depth_budget.
      // LANDED (C1): mu_policy = endpoint_additive (set in the
      // post-shared-block override) - SOLVE_LDP step 3's i=0 branch, lambda_j0
      // += rho_k*(a_j-A_0), the endpoint pressure this preset had no live
      // counterpart for under endpoint_lambda (chen audit §2.1). Shares
      // lambda_update_c with branch 2. RESTORED (bucket-2): upsize_hysteresis =
      // 0 - the plain LRS argmin this paper specifies. The shared 0.02 upsize
      // deadband no longer rides it (see the non-baseline block at the end of
      // applyPreset).
      lambda_update = LambdaUpdate::kChenSubgradient;
      lambda_seed = LambdaSeed::kConstant;
      lambda_init_value = 1.0f;
      // A1 (MB item 5): the λ-free median anchor, tw = l_med/d_med. Under it λ
      // IS the timing/leakage ratio on a median gate, which is the transferable
      // content of this paper's λ constant; timing_bias is orthogonal and
      // pinned to 1.0 to say so (64 is rsz_baseline's knob, and it was the
      // confound every cross-paper comparison shared until this axis landed).
      timing_scale = TimingScale::kUnit;
      timing_bias = 1.0f;
      // C: the paper's LRS starts from the minimum-size solution (SOLVE_LRS/μ
      // step 1, "for i := 1 to n do x_i := L_i"; Table 1 likewise reports
      // against "the minimum-size solution"). Until the bucket-2 fidelity pass
      // this bundle ran as_given, which was an undocumented divergence rather
      // than a deferral - the option existed and the axis works (the init
      // pass's clock-filter inversion that made it dead is fixed, hardening
      // #3).
      init_mode = InitMode::kMinSize;
      // The guard is our own depth budget, which appears in NO paper: Chen'98
      // predates Flach's local-slack check by 16 years and specifies no
      // candidate-rejection rule at all, so the paper-faithful guard is
      // arguably `none`. Retained deliberately (user decision, bucket-2): a
      // guard-free downsize sweep is a different experiment, not a more
      // faithful one, and which guard these pre-Flach presets should carry is a
      // campaign-stage ablation question (plan §6 Stage-1: guard ∈ {none,
      // depth_budget, local_slack_veto} on chen/tennakoon/livramento), not a
      // bundle default to guess at now. Same class as hardening finding #10.
      // H1 (C2): pure_cap - the ρ_k schedule runs to the cap, no early-exits.
      termination = TerminationKind::kPureCap;
      // H2 (C2): none. Chen'98 has no best-so-far snapshot/restore (exact
      // convergence made it unnecessary); the final iterate stands, coherent
      // with pure_cap. The struct default flach_dominance rode this preset
      // undisclosed and could swap a Flach mechanism in under Chen's name near
      // feasibility (chen audit §2.3).
      best_tracker = BestTrackerKind::kNone;
      // H1 budget (C2): ~100. Chen's own stop (the duality gap) is
      // untranslatable, so a cap is the right mechanism; Fig. 8 shows
      // cold-start convergence at ~50-100 iterations (a plot reading, re-audit
      // F4 - not a paper guarantee). The rsz_baseline default 20 truncated the
      // ρ_k = c/k schedule badly (chen audit §2.2).
      max_iterations = 100;
      break;
    case Preset::kTennakoon:
      // Tennakoon-Sechen ICCAD'02 Forge (SYNTHESIS §4 column "tenn 02"). This
      // bundle is its Fig. 13 branch-2 ratio update on the baseline seed, and
      // nothing else.
      //
      // DEFERRED (hence _partial):
      //  - D(c)/C(d): the paper's HEADLINE - KKT-extraction of the implied
      //    multipliers from gate sizes via the steepest-descent delay-contour
      //    pre-phase. Deferred with the continuous lineage by a §0 locked
      //    decision, so this preset carries the paper's supporting half only.
      //    Tennakoon therefore keeps the baseline seed.
      //  - H1: its "until convergence" gap-style exit (unspecified in the
      //    paper); we run pure_cap (C2) - a fixed budget with none of
      //    fixed_iters' surviving early-exits (3 rejected sweeps / 2 zero-move
      //    sweeps), either of which can truncate the min-area leakage-recovery
      //    regime before the schedule reaches it (tennakoon audit §2.5). NOTE
      //    the WNS-meets-margin stop that used to be this item's headline was
      //    DELETED from fixed_iters on 2026-07-29, so it is no longer part of
      //    what pure_cap buys here.
      // RETAINED NON-PAPER CONSTRUCTION (deliberate, see below): depth_budget.
      //
      // See also the Fig. 13 erratum in papers/summaries/tennakoon_et_al.md:
      // branch 1 (primary outputs) targets mu, not lambda; branch 3 (input
      // drivers) is untranslatable (hardening finding #8).
      // LANDED (C1): branch 1 (lambda_j0 *= a_j/A_0) is now mu_policy =
      // endpoint_ratio (set in the post-shared-block override) - the paper's
      // own multiplicative endpoint pressure, identical to Livramento Alg. 1
      // L12. This supersedes the old "available as update_as_lambda" pointer,
      // which was only an adapted relative (tennakoon audit §2 item 2).
      // RESTORED (bucket-2): upsize_hysteresis = 0 - the plain LRS argmin
      // this paper specifies. The shared 0.02 upsize deadband no longer
      // rides it (see the non-baseline block at the end of applyPreset).
      lambda_update = LambdaUpdate::kTennakoonRatio;
      // A1 (MB item 5): auto_median, NOT `unit` - deliberately, and the reason
      // is this preset's SEED. `unit` preserves λ's magnitude, which is only
      // worth preserving when the seed sets it meaningfully. This bundle runs
      // OpenROAD's delay-proportional seed (its own paper's λ-init mechanism is
      // unavailable - see DEFERRED above), and that seed writes λ_e = d_e: an
      // arc delay in SECONDS, ~1e-10. Its magnitude is a unit artifact, not a
      // balance choice. Under mu_policy=endpoint_lambda the projection no
      // longer renormalizes it away (that is the whole point of
      // endpoint_lambda), so `unit` would carry the artifact straight into the
      // cost and shrink the timing term by ~1e10 - a total pure-leakage
      // collapse, measured. Only auto_median divides it back out (t_med ∝ λ ⇒
      // tw ∝ 1/λ). This is exactly the plan's stated reason rsz_baseline keeps
      // auto_median - "their λ has no meaningful magnitude by construction, so
      // they need it" - and the discriminator is the seed, not the preset's
      // name.
      //
      // timing_bias IS live under auto_median (it is that option's balance
      // knob), so it must carry the balance this preset's λ cannot. 12 - the
      // only balance constant the corpus justifies (Flach's λ_init, whose
      // transferable content the item-3 artifact showed is "timing should be a
      // few times leakage on a median gate"). Note bias = 12 here is EXACTLY
      // equivalent to λ = 12 under `unit`: both put tw·Σλ·d ≈ 12·l_med on the
      // median gate. So the whole paper family shares one balance convention
      // whichever anchor it reaches it through, and a cross-paper comparison is
      // finally apples-to-apples - which was the point of retiring
      // rsz_baseline's arbitrary 64.
      timing_scale = TimingScale::kAutoMedian;
      timing_bias = 12.0f;
      // The guard is our own depth budget, which appears in NO paper:
      // Tennakoon'02 predates Flach's local-slack check by 12 years and states
      // no candidate-rejection rule, so the paper-faithful guard is arguably
      // `none`. Retained deliberately (user decision, bucket-2) for the same
      // reason as chen_partial: guard-free is a different experiment, not a
      // more faithful one, and the choice is a campaign-stage ablation question
      // (plan §6 Stage-1: guard ∈ {none, depth_budget, local_slack_veto} on
      // chen/tennakoon/livramento). Same class as hardening finding #10.
      // H1 (C2): pure_cap - the LagrangeM update runs to the cap, no
      // early-exits (tennakoon audit §2.5).
      termination = TerminationKind::kPureCap;
      // H2 (C2): none. Tennakoon'02 has no best-so-far - §5 Phase 3 takes "the
      // last LRS/λ solve", coherent with pure_cap. The struct default
      // flach_dominance rode this preset undisclosed (tennakoon audit §2.4).
      best_tracker = BestTrackerKind::kNone;
      // H1 budget (C2): 100, symmetric with chen. The paper states no
      // iteration number (only CPU seconds and "few, well-behaved steps" from
      // the DEFERRED contour warm start this cold-started bundle does not
      // have), so this is a documented non-paper choice under a cold start -
      // the rsz_baseline default 20 was an unexamined truncation (tennakoon
      // audit §2.3).
      max_iterations = 100;
      break;
    case Preset::kFlach:
      // Flach TCAD'14 (SYNTHESIS §4 column "flach 14").
      //
      // DEFERRED (hence _partial):
      //  - I: its post-passes TR (criticality-sorted upsizing to TNS~0) and PR
      //    (Vth-first then downsize, to fixpoint) are a HEADLINE contribution
      //    and Table IV's leakage depends on them. The post-pass axis (§3.I) is
      //    unimplemented framework-wide.
      // LANDED (it2 rider, RULED 2026-08-08): C - the paper's own initial
      // solution (Fig. 1 steps (a)+(b): every gate to its lowest-leakage
      // version, then Li et al.'s reverse-topological pass removing the
      // load/slew violations that reset creates) IS init_mode =
      // min_size_fixviol, so pinning it is a fidelity restoration, not a preset
      // redesign; see init_mode below.
      // RESTORED (bucket-2): upsize_hysteresis = 0 - the plain LRS argmin
      // this paper specifies. The shared 0.02 upsize deadband no longer
      // rides it (see the non-baseline block at the end of applyPreset).
      lambda_update = LambdaUpdate::kFlachSlackScaling;
      lambda_seed = LambdaSeed::kConstant;
      lambda_init_value = 12.0f;
      // A1 (MB item 5): the λ-free median anchor, tw = l_med/d_med. Under it λ
      // IS the timing/leakage ratio on a median gate, which is the transferable
      // content of this paper's λ constant; timing_bias is orthogonal and
      // pinned to 1.0 to say so (64 is rsz_baseline's knob, and it was the
      // confound every cross-paper comparison shared until this axis landed).
      timing_scale = TimingScale::kUnit;
      timing_bias = 1.0f;
      // C (it2 rider, RULED 2026-08-08): Fig. 1 (a)+(b) - min-leakage reset
      // then the reverse-topological electrical repair. This is the starting
      // point every number in the paper is measured from, and it is the load
      // veto's premise: Alg. 4 only forbids INCREASING load violation, which
      // keeps the run violation-free only because (b) made it violation-free
      // first. as_given was a divergence, not a deferral - the mode landed in
      // it2 pass 2 (InitPass + fixviol).
      // ADAPTED, two ways, neither reproducible from this paper: (i) step (b)
      // is Li et al. [2]'s procedure at α = 0.7 and §13 item 19 says the
      // meaning of α "is entirely delegated to Li et al. - not reproducible
      // from this paper alone", so our repair carries no α; (ii) Li's pass
      // re-picks the lowest-leakage legal version for EVERY gate, ours upsizes
      // only the gates that violate. On the min-size netlist step (a) hands it
      // the two coincide - a non-violating gate's lowest-leakage legal version
      // is the minimum it already holds - so the difference is confined to a
      // violating gate no group member can fix, which the repair leaves at
      // minimum and reports (RSZ-0445) rather than moving. What happens to that
      // gate AFTERWARDS is the veto's business, not the repair's: under
      // output_drc_veto = relative, pinned just below, the sweep may still size
      // it as long as the violation does not grow, which is what Alg. 4 line 6
      // says. (Under the absolute default it would stay at minimum all run -
      // the composition the re-audit delta records as D1.)
      init_mode = InitMode::kMinSizeFixviol;
      // A2 (RULED 2026-08-09, re-audit delta D1): Alg. 4 line 6 is "if load
      // violation has INCREASED" (flach_et_al.md:148, and :166 for why - the
      // veto exists to stop violations spawning downstream slew problems and
      // pushing the delay tables into extrapolation, not to freeze gates). Our
      // output side was absolute, and the RA justified the extra strictness as
      // "compensating the missing fixviol init"; the init above IS that
      // compensation, so the justification expired with it and the paper's own
      // rule is what this preset now runs.
      //
      // ADAPTED: the paper's relative rule is stated for the LOAD (max-cap)
      // constraint, and §VII-D's max-slew handling is "removed in the initial
      // solution [...] during LRS sweeps controlled indirectly via the load and
      // local-negative-slack vetoes" (flach_et_al.md:275) - i.e. Flach prints
      // no per-candidate slew veto at all. Our mode applies one rule to both
      // limits, so on the slew leg `relative` is the closer of the two options
      // rather than a transcription: it is weaker than the absolute veto the
      // paper does not have, and still stronger than the nothing it does.
      output_drc_veto = OutputDrcVeto::kRelative;
      // Flach's sensitivity-based global term (Eq. 5 φ). Shipped OFF in the
      // ISPD'13 contest run but is the paper's headline cost contribution, so
      // the flach_partial preset turns it on (M3).
      cost_global_phi = true;
      // Alg. 4's two vetoes (M5): the load veto is the always-on DRC filter;
      // the local-negative-slack veto with gamma hill-climbing (Eq. 14) is the
      // F3 guard. Alg. 1's (TNS < 10% T) and-lower-leakage dominance rule with
      // restore-best is the H2 tracker.
      downsize_guard = DownsizeGuard::kLocalSlackVeto;
      best_tracker = BestTrackerKind::kFlachDominance;
      // H1 (C2): pure_cap. Historically the decisive argument was fixed_iters'
      // WNS-meets-margin exit, which halted the run at the FIRST feasible
      // iterate and made the k=4 leakage-recovery phase, the k_final endgame
      // and the dominance tracker's harvest structurally unreachable (flach
      // audit §2.1). That exit was DELETED on 2026-07-29 (Termination::
      // stopBeforeSweep), so it no longer distinguishes the two options. The
      // pin stands on what remains: fixed_iters still stops after 3 rejected or
      // 2 zero-move sweeps, and this bundle's asymmetric k-schedule spends its
      // early iterations moving λ rather than cells, so pure_cap is what lets
      // the schedule reach the cap and flach_dominance harvest the
      // lowest-leakage feasible iterate.
      termination = TerminationKind::kPureCap;
      // Alg. 1 line 12's convergence criterion is never defined (§13.2); Fig. 4
      // is the only quantitative evidence and shows ~120 iterations (its k
      // switch lands at 57). 20 was the rsz_baseline struct default riding this
      // preset, not a Flach number. Integration smokes override it back down
      // with an explicit -max_iterations AFTER -preset (the override is
      // order-sensitive by design) to keep their runtimes.
      max_iterations = 120;
      break;
    case Preset::kSharmaSeq:
      // Sharma ICCAD'15 (SYNTHESIS §4 column "sharma 15"), sequential mechanics
      // only - hence _seq, and _partial for the rest.
      //
      // DEFERRED:
      //  - G: ★MEE + DNT, the paper's parallelization headline (5.23x
      //    self-speedup). Deferred by a §0 locked decision (threading beyond
      //    the current Jacobi), which is what _seq records.
      //  - I: Fast-GTR post-pass.
      // LANDED (it2 rider, RULED 2026-08-08): C - the min-leakage start plus
      // its electrical repair (§8: "initialization repairs the min-leakage
      // solution with a reverse-topological max-cap fix pass then a
      // forward-topological max-slew upsize pass"; §12 item 11) IS init_mode =
      // min_size_fixviol, so pinning it is a fidelity restoration, not a preset
      // redesign; see init_mode below.
      // RESTORED (bucket-2): upsize_hysteresis = 0 - the plain LRS argmin
      // this paper specifies. The shared 0.02 upsize deadband no longer
      // rides it (see the non-baseline block at the end of applyPreset).
      lambda_update = LambdaUpdate::kSharmaCexp;
      lambda_seed = LambdaSeed::kConstant;
      lambda_init_value = 1.0f;
      // F4 (it2 pass 3): ★Fast-OLR, restored. This is THE paper's LRS - §5.2
      // and §6 both describe the run as "exhaustive OLR for the first 4
      // iterations, Fast-OLR from iteration 5", and every experimental variant
      // in §9 ("All variants use Fast-OLR (from iteration 5) and early exit")
      // uses it, so the preset was previously running a configuration the paper
      // never reports. It is also the paper's STABILITY device, which matters
      // here more than the 3.3x evaluation saving: this bundle's max_iterations
      // is 160, i.e. entirely past the ~100-iteration point where Fig. 11 shows
      // plain OLR destabilizing TNS and ending ~11% worse in power (REAUDIT
      // §3.5 called its absence the sharpest live QoR risk in the framework).
      // fast_olr_start_iter keeps the paper's 5. The move_set=full_library arm
      // stays selectable, which is what the Stage-3 X9 A/B measures.
      move_set = MoveSet::kSharmaFastOlr;
      // C (it2 rider, RULED 2026-08-08): the min-leakage start plus its
      // electrical repair (§8, §12 item 11). It is not decoration here either -
      // the paper enforces max-cap/max-slew INVARIANTLY, by starting clean and
      // then skipping any OLR candidate that would violate, so as_given ran the
      // invariant from a netlist that never established it. ADAPTED: our repair
      // is one reverse-topological pass that clears cap and slew together;
      // Sharma's is a reverse-topological cap pass then a forward-topological
      // slew pass. Same fixpoint intent, Flach's traversal shape (which this
      // paper's own lineage inherits).
      //
      // SOURCING (corrected 2026-08-09, re-audit delta T1). This is a
      // divergence from a structure the paper DOES pin: §5.1 spells out the
      // two passes ("a reverse-topological pass upsizes gates that have
      // capacitance violations, then a forward-topological pass upsizes to
      // satisfy slew constraints", sharma_et_al.md:82) and §12 item 11 names
      // the initialization as "min-leakage cells + cap-then-slew repair
      // passes". The §13 item 14 pointer that stood here excuses the wrong
      // thing: what that item leaves unspecified is WHICH CELL is picked when
      // "minimally upsizing" for cap/slew and in what tie-break order - a
      // different degree of freedom, and the one our repair really does fill
      // in (lowest-ranked group member that clears). So the pass structure is
      // a stated adaptation, not an underspecification we are filling in, and
      // the residue it carries is exactly what a second, forward pass exists
      // to handle: our walk re-reads each gate's LOAD live (that is what the
      // reverse-topological order buys) but takes its measured SLEW from the
      // post-reset snapshot (InitPass.cc), so slew is judged against pre-repair
      // input slews instead of being propagated forward. RSZ-0445 counts what
      // the pass leaves violating either way.
      init_mode = InitMode::kMinSizeFixviol;
      // A2: NOT pinned, deliberately, and the asymmetry with flach_partial -
      // its twin in the fixviol pin, which does pin relative - is the papers'
      // own. Flach's Alg. 4 line 6 forbids an INCREASE in load violation;
      // Sharma's Fig. 9 line 10 rejects a candidate that IS invalid ("a cell is
      // invalid if it causes cap or slew violations", sharma_et_al.md:274), and
      // §8 calls the enforcement INVARIANT rather than incremental. absolute -
      // the struct default - is that rule, so this preset inherits it.
      // A1 (MB item 5): the λ-free median anchor, tw = l_med/d_med. Under it λ
      // IS the timing/leakage ratio on a median gate, which is the transferable
      // content of this paper's λ constant; timing_bias is orthogonal and
      // pinned to 1.0 to say so (64 is rsz_baseline's knob, and it was the
      // confound every cross-paper comparison shared until this axis landed).
      timing_scale = TimingScale::kUnit;
      timing_bias = 1.0f;
      // B2: Eq. 5's candidate cost is local arcs + Flach's downstream
      // delay/slew sensitivity term ΔD_n^λ. sharma_et_al.md §5.2: exact λ-delay
      // would need incremental STA per candidate, so "only local arcs are
      // retimed (Li et al.), plus Flach et al.'s global delay/slew
      // sensitivities for the rest of the fanout cone"; §6 is the same cost
      // ("leakage + lambda-delay-cost per gate (Eq. 5)"); §11 lists ΔD_n^λ
      // among what the paper takes from Flach [15]. Our cost_global_phi IS that
      // term (Flach Eq. 5 drainNets / Eq. 11 φ), so leaving it off ran the
      // local-arcs-only variant - which §12's ablation 8 names as the OTHER arm
      // of this axis ("local arcs only (Li) vs. local arcs + downstream
      // sensitivity term (Flach, Eq. 5)"), i.e. the preset was running Li's
      // cost model under Sharma's name. Off until the bucket-2 pass (the
      // bucket-1 audit flagged it as an undocumented divergence).
      //
      // Validator interactions, both clean: cost_delta_delay stays off
      // (RSZ-0424 hard-rejects φ ⊕ delta-delay), and cost_fanout_slew stays off
      // (RSZ-0429 warns that both price the immediate sink level's slew
      // sensitivity). φ subsumes the fanout term here - its Eq. 11 recurrence
      // starts at that same level - so this bundle prices it once, via the
      // paper's own term.
      //
      // STILL SHORT OF EQ. 5, and the disclosure is owed here because the
      // paragraph above reads as if cost_global_phi completed it (RA §7,
      // landed 2026-08-09). What we price is "Eq. 5 minus its side-arc and
      // side-net terms": Eq. 5's local-arc set includes the SIDE-ARCS of
      // sibling gates sharing a fanin - a candidate's input-cap change slows
      // the shared fanin driver, shifting those siblings' arcs - and its second
      // sum runs over drain-nets UNION side-nets, i.e. the siblings' downstream
      // cones as well (sharma_et_al.md:110,114). Our upstream-load term prices
      // only the driver's own arc delay shift, and no term walks the sibling
      // arcs at all, so this cost systematically under-prices a candidate's
      // effect on its fanin's other sinks. Framework-wide (REAUDIT F3): the
      // same term is missing from flach and mangiras, which share Eq. 5's
      // lineage, so the gap is uniform across those three columns rather than
      // a sharma-only handicap.
      cost_global_phi = true;
      // Sharma adopts Flach's acceptance check verbatim (§5, Fig. 5 line 21):
      // "computing the change in the slack of the driver and the sink nets
      // because the locally optimal cell [...] might significantly worsen the
      // TNS. We apply this check as we recover power, after the design timing
      // is within 1% of the [target]". So the paper-faithful guard is the
      // local-slack veto, not our own depth budget (which no paper contains).
      //
      // C2: the veto activates at the paper's own gate. near_met_gate_frac =
      // 0.01 latches the run "near-met" once WNS reaches within 1% of the
      // target, and the veto passes every candidate until then - the literal
      // reading of "we apply this check as we recover power, after the design
      // timing is within 1% of the [target]", which supersedes the previous
      // gamma-decay stand-in (sharma audit §2 item 6). Flach's gamma
      // hill-climbing (Eq. 14) still shapes the tolerance once active.
      downsize_guard = DownsizeGuard::kLocalSlackVeto;
      // H2: NOT pinned, so this preset runs the struct default
      // flach_dominance - disclosed here (RA §7, landed 2026-08-09) because for
      // THIS paper a best-power memory is nearly right, which made the silence
      // worse than chen's or tennakoon's. Sharma's flow does keep one: the
      // greedy post-pass starts "from the least-power solution obtained from
      // LDP" (§5.3, with §13.9 leaving its feasibility condition ambiguous).
      // But Flach's rule is a different object - its |TNS| < 10%*T gate can
      // harvest a still-violating iterate, and its dominance test is Flach's -
      // so read it as ADAPTED (the closest available host for §5.3's
      // least-power start), not as the paper's own rule. The post-pass that
      // would consume that start is the I-axis deferral above; when it lands,
      // §13.9 gets decided explicitly instead of inherited.
      // Sharma's early exit (§5.2): sets of 5 iterations, stop after 2
      // consecutive sets in which neither the average nor the best-so-far power
      // improved. The struct defaults already hold those constants.
      //
      // C2: the same 1% near-met gate activates the stagnation monitor. The
      // paper frames the early exit for the "timing almost met" regime; ungated
      // it stopped a still-closing run at 3*window = 15 iterations mid-closure,
      // because a timing-closing sweep monotonically raises leakage so every
      // window reads stagnant (sharma audit §2 item 3). One constant serves the
      // monitor and the guard above.
      termination = TerminationKind::kStagnationWindows;
      near_met_gate_frac = 0.01f;
      // H1 budget (C2): 160, the paper's observed maximum (35-160 LDP
      // iterations, avg 95, σ 49; §5.2/§6) - disclosed as an OBSERVATION, not a
      // spec. The gated stagnation exit above does the real stopping, which is
      // the paper's design. 20 (the rsz_baseline default) truncated even the
      // paper's fastest design by ~40% (sharma audit §2 item 4).
      max_iterations = 160;
      break;
    case Preset::kReimann:
      // Reimann ISPD'16 (SYNTHESIS §4 column "reim 16"). Both of its ★ headline
      // components (the estimation-loop seed and the dWNS-normalized update)
      // ARE here; the _partial suffix is for the rest.
      //
      // DEFERRED:
      //  - B2/D: §3.2 runs estimation loop 1 with a cheap top-1 ranking filter
      //    instead of the full LRS; our estimation_loop seeder runs the full
      //    sweep, so loop 1 costs what a real iteration costs. The ranking
      //    filter is unimplemented and unscoped.
      //  - A1: the paper's objective is leakage + dynamic + area with the
      //    alpha/beta/theta weights of Eqs. 2/5 (a headline contribution). This
      //    bundle runs `unit` instead, which is design-derived. STILL DEFERRED
      //    after the MB item-4a audit, deliberately: `timing_weight ≡ α/β` is
      //    the affine equivalent ONLY IF θ/β is *simultaneously* pinned to the
      //    paper's ratio, because Eq. 2 weights area alongside power. Shipping
      //    α/β alone would be an imitation of the scheme, not the scheme, so it
      //    is not shipped. Doing it honestly needs a θ-weighted area term in
      //    the candidate cost - a B2 cost-model component with its own flag,
      //    not something this axis may switch on behind the cost model's back.
      //
      //    What the audit settled, so the later pass need not re-derive it:
      //     - Eq. 5 has a SIGN DEFECT, and it is not an extraction artifact:
      //       α = N_{C_REF}/(D(c_n) − D(c_0)) with c_n the largest/lowest-Vth
      //       (fastest) option and c_0 the smallest/highest-Vth (slowest), so
      //       D(c_n) < D(c_0) at a fixed load and α < 0 as literally written -
      //       which would make the LRS MAXIMIZE Σλd. Eq. 2 inherits the operand
      //       order from power/area (correct there, both positive) without
      //       flipping it for the inverse-monotone metric. Implement
      //       α = N/|D(c_n) − D(c_0)|. The paper never acknowledges this.
      //     - N_{C_REF} is a mathematical NO-OP: it is the common numerator of
      //       all four scaling factors (α, β, θ, ζ), so it scales the whole
      //       objective uniformly - and Alg. 1 only ever argmins over one
      //       gate's options, the λ update is slack-driven and scale-free, and
      //       the score function uses percentage deltas. Only the RATIOS α/β
      //       and θ/β carry behavior. Omit it.
      //     - "library-derived from a reference cell's dynamic range" (the
      //       phrase this file used until MB item 4a) is a MISNOMER, retired:
      //       there is no single range - each weight uses its own metric's
      //       range over C_REF's options - and "dynamic" collides with dynamic
      //       power, which is exactly what β is NOT computed from ("the power
      //       scaling factor β calculated based only on leakage power is
      //       applied to both metrics"). The paper's wording is "the average
      //       power or area change between cell options": each term is measured
      //       in average option-steps of the reference cell.
      //     - Four things the paper never specifies, decided here so the later
      //       pass inherits a spec rather than the ambiguity (all four are
      //       adaptations, not the paper): (1) the "reference output load" -
      //       the phrase occurs exactly once and is never defined, and α scales
      //       directly with it - read as C_REF's own input capacitance x the
      //       library's default fanout of 4, the standard
      //       delay-characterization point; (2) which inverter is C_REF ("In
      //       our experiments C_REF is an inverter" is the entire statement) -
      //       read as the median-drive inverter footprint, so the steps are
      //       typical rather than extremal; (3) θ's Vth level, unstated for its
      //       single-level area range - read as the library's
      //       nominal/highest-Vth level, matching β's leakage-only framing; (4)
      //       a genuine textual ambiguity over whether c_n/c_0 are the extremes
      //       of the LIBRARY or of C_REF's own option set - the prose says
      //       library, the framing sentence and N_{C_REF}'s definition say
      //       C_REF, and they give very different weights - read as C_REF's own
      //       options, which is the only reading under which N_{C_REF} ("of
      //       C_REF") is well-defined at all.
      //  - I: the ETR -> EPR -> ETR placement-aware post-pass recipe.
      // RESTORED (bucket-2): upsize_hysteresis = 0 - the plain LRS argmin
      // this paper specifies. The shared 0.02 upsize deadband no longer
      // rides it (see the non-baseline block at the end of applyPreset).
      lambda_update = LambdaUpdate::kReimannDwns;
      lambda_seed = LambdaSeed::kEstimationLoop;
      // A1: auto_median rather than the paper's own α/β (deferred above) and
      // rather than `unit`. The estimation_loop seed builds on OpenROAD's
      // delay-proportional λ (arc delay in SECONDS), so this preset's λ
      // magnitude is a unit artifact and `unit` would preserve the artifact -
      // see the same note on tennakoon_partial. Fitting, since Reimann's own
      // §1 motivation for the estimation phase is that "LR from arbitrary
      // multipliers diverges": the paper does not claim a meaningful λ_0
      // either. timing_bias = 12 is the family's shared balance (see
      // tennakoon_partial), live here because auto_median's tw reads it.
      timing_scale = TimingScale::kAutoMedian;
      timing_bias = 12.0f;
      // Alg. 2 lines 14-18: the exponential score (Eq. 6) picks the stored
      // solution and it is restored at the end.
      //
      // The guard is hosted on Flach's veto because Reimann's own Alg. 1 line
      // 10 - "if new slack < gamma * original slack then go to the next option"
      // - is that veto in form: a per-candidate rejection on local slack
      // degradation, scaled by a tolerance factor. It is NOT a citation of the
      // paper's prose: gamma occurs exactly once in the paper (that line), with
      // no definition, no value, and no schedule, so the hill-climbing decay we
      // run for it is Flach's Eq. 14 (gamma_local_slack), not Reimann's. What
      // "original slack" and "new slack" range over is likewise unstated; we
      // read them as Flach's local negative slack over the driver and sink
      // nets. Both readings are adaptations, not the paper.
      downsize_guard = DownsizeGuard::kLocalSlackVeto;
      best_tracker = BestTrackerKind::kReimannScore;
      // C1: the disclosed NON-PAPER setpoint adaptation. Reimann's own servo
      // references each arc's frozen INITIAL slack (s_init) - it holds the LRS
      // near the input timing, its §7 power-recovery regime. OpenROAD inserts
      // this preset in an improve-from-violating flow instead, where that
      // reference sits above every subsequent slack, so the increase branch
      // never fires and the field decays to the floor (the measured B1 "decay"
      // row - a regime mismatch, not an implementation error; reimann audit
      // §2.1). slack_target re-references the servo to the slack TARGET so the
      // increase branch fires on violating arcs and every Reimann mechanism
      // stays live. This is NOT Reimann's method (see the loud disclosure on
      // ReimannSetpoint), so a run of this preset may not be attributed to the
      // paper; s_init stays available for an M7 regime-aligned run.
      reimann_setpoint = ReimannSetpoint::kSlackTarget;
      // H1 (C2): pure_cap. Until 2026-07-29 fixed_iters' WNS-meets-margin exit
      // was uniquely fatal here: in Reimann's own regime (an optimized,
      // FEASIBLE input) the run exited before the first sweep - zero
      // iterations, the whole power/area-recovery method amputated at birth
      // (reimann audit §2.2). That exit is now deleted for every option
      // (Termination::stopBeforeSweep), so it is no longer the reason for this
      // pin and the audit item's precondition for a regime-aligned run is
      // satisfied outright. The pin stands on the paper having no early exit of
      // its own: fixed_iters' 3-reject and zero-move stops would still truncate
      // Alg. 2's loop-2 schedule.
      termination = TerminationKind::kPureCap;
      // H1 budget (C2): 20 - claimed explicitly with the paper's citation
      // (Alg. 2's loop-2 cap; ~12 used on average). The inherited struct
      // default happened to equal 20, so a future default change would silently
      // de-paper this preset; pinning it makes the coincidence intentional
      // (reimann audit §2.3).
      max_iterations = 20;
      break;
    case Preset::kMangiras:
      // Mangiras & Dimitrakopoulos, Technologies (MDPI) 2021, 9, 92 - the
      // journal extension of their MOCAST'21 paper (SYNTHESIS §4 column
      // "mang 21"). NOT a TCAD paper: this block read "TCAD'21" until the
      // re-audit delta caught it (T2, corrected 2026-08-09), and the TCAD'21
      // this group really does have is Stefanidis et al., the multi-corner
      // work this paper cites for its critical-corner selection
      // (mangiras_et_al.md ref [10]). Its ★ headline (the state-adaptive seed)
      // and its ★ as-given init ARE here; the _partial suffix is for the rest.
      //
      // DEFERRED:
      //  - A3: its multi-corner scope. Deferred by a §0 locked decision.
      //  - I: its TR/PR-lite post-passes (next-size-up on many-endpoint gates;
      //    STA-verified single steps).
      // THE EQ. 6 SURVIVAL CHAIN (MB item 5, C1-C4): Eq. 6's global power ratio
      // (ΣP / Σ minP)^K is a uniform multiplier on the seed field. Under the
      // struct default (auto_median + reseed_each_iter) it was cancelled
      // exactly
      // - tw ∝ 1/λ divides it out, and re-seeding μ each iteration erases the
      // boundary after one projection - so only Eq. 5's per-arc shape survived
      // and the headline was neutralized. The item-5 bundle below un-cancels
      // it: timing_scale=unit is the one λ-free anchor (tw = l_med/d_med, so
      // λ's magnitude reaches scoring) and mu_policy=endpoint_lambda (the
      // shared default) makes the E3 projection positively homogeneous, so the
      // seed boundary is preserved, not overwritten. The chain seed →
      // projection → unit scoring is pinned end-to-end by
      // TestTimingScale.Eq6GlobalPowerRatioSurvivesToScoringUnderUnit (and its
      // auto_median negative control). Residual: homogeneity holds only above
      // the λ floor (an intended small balance knob; see FlowProjection).
      //
      // Mangiras is the Flach LR host (multiplicative update Eq. 4 ==
      // flach_slack_scaling) with the state-adaptive initialization Eqs. 5-7.
      // RESTORED (bucket-2): upsize_hysteresis = 0 - the plain LRS argmin
      // this paper specifies. The shared 0.02 upsize deadband no longer
      // rides it (see the non-baseline block at the end of applyPreset).
      lambda_update = LambdaUpdate::kFlachSlackScaling;
      lambda_seed = LambdaSeed::kStateAdaptive;
      // F4 (it2 pass 3): §4.3's restricted move set - "each gate may move only
      // to its next bigger or next smaller size (+-1 size step) while Vth swaps
      // remain unrestricted" - which this bundle's DEFERRED list carried as a
      // missing component until the axis existed. Present in BOTH the journal
      // and the MOCAST'21 conference version (its Table II = journal Table 3),
      // so it is not a late add-on.
      //
      // DISCLOSED TENSION, because it decides what this column means. §4.3 is
      // the paper's SECONDARY study: Tables 1-2 (the headline 24-27% WNS /
      // 36-39% TNS / 42-45% runtime numbers) are full-library, and Tables 3-4
      // are the restricted-mode counterparts. So this pin aligns the preset
      // with Tables 3-4 rather than 1-2. It is still the right default here:
      // (a) the paper's CLAIM is about the initialization, and it is reproduced
      // in both modes (restricted: 36%/39% WNS/TNS, ~2% leakage), so the
      // headline is not what the restriction changes; (b) §4.3's motivation -
      // preserve detailed routes at the very end of the flow - is precisely
      // OpenROAD's post-route repair regime, which is where rsz runs; and
      // (c) the framework's own bookkeeping had already recorded the full-
      // library evaluation as a fidelity GAP of this preset, so removing it is
      // a restoration. move_set=full_library remains selectable and is the
      // Tables 1-2 arm for any ablation that wants it.
      move_set = MoveSet::kMangirasSizeStep;
      // A1 (MB item 5): the λ-free median anchor, tw = l_med/d_med. Under it λ
      // IS the timing/leakage ratio on a median gate, which is the transferable
      // content of this paper's λ constant; timing_bias is orthogonal and
      // pinned to 1.0 to say so (64 is rsz_baseline's knob, and it was the
      // confound every cross-paper comparison shared until this axis landed).
      timing_scale = TimingScale::kUnit;
      timing_bias = 1.0f;
      // "Keeps the entire LR machinery of Flach et al. unchanged and replaces
      // only the LM initialization" - so the guard is Flach's veto. Pinning it
      // is what keeps a mangiras-vs-flach comparison an isolation of the seed
      // axis, which is the paper's whole claim.
      downsize_guard = DownsizeGuard::kLocalSlackVeto;
      // H2: NOT pinned either, so the struct default flach_dominance rides
      // this preset - claimed here (RA §7, landed 2026-08-09) on the same
      // ground as the guard directly above. Flach's Alg. 1 best-solution rule
      // is part of the "entire LR machinery of Flach et al." this paper keeps
      // unchanged, so the inheritance is IMPORTED MACHINERY rather than an
      // undisclosed default of ours - defensible in a way it is not for
      // chen/tennakoon, whose papers have no best-so-far at all. One residue,
      // recorded rather than fixed: the rule's |TNS| < 10%*T gate can harvest a
      // still-violating iterate, which sits oddly with an ECO method whose
      // input is an almost-closed design (mangiras audit §2 item 5).
      // Its convergence rule (§5.3.e): stop when TNS *and* total leakage both
      // improve by less than 1% across two consecutive iterations - the
      // stagnation-window rule with the TNS clause on at 1%.
      //
      // window = 1, count = 1 is the faithful parameterization: a window is one
      // iteration, so each iteration is compared against the one before it and
      // a single stagnant comparison stops the run - exactly "improvement over
      // two consecutive iterations is below 1%". window = 2 (used until the
      // bucket-1 fidelity pass) instead averages leakage/TNS over disjoint
      // 2-iteration blocks and compares block to block, so it first fires at
      // iteration 4 and only on even iterations, which is a different rule.
      // Sharma's block semantics is unchanged - it is the same mechanism read
      // with the paper's own constants (window 5, count 2).
      termination = TerminationKind::kStagnationWindows;
      stagnation_window = 1;
      stagnation_count = 1;
      stagnation_improve_frac = 0.01f;
      stagnation_require_tns = true;
      // The window-1 rule is ungated (near_met_gate_frac stays < 0): Mangiras'
      // §5.3.e rule is correct always-on and must not change (mangiras audit
      // §1). H1 budget (C2): 20, claimed explicitly with the §13.9 citation
      // (the paper's cap is unstated; its plots run to 20 and the stagnation
      // rule does the real stopping at ~5 iterations). Pinning it stops a
      // struct-default change from silently moving this preset (mangiras audit
      // §2 item 4).
      max_iterations = 20;
      break;
    case Preset::kLivramento:
      // Livramento DATE'13 (SYNTHESIS §4 column "livra 13").
      //
      // DEFERRED (hence _partial):
      //  - A2: ★Lagrangian handling of the electrical (max-cap/max-slew)
      //    constraints - the paper's headline. We run the shared always-on DRC
      //    veto instead.
      //  - The beta cap/slew relaxation.
      //  - I: its fix-viol post-pass (Alg. 3 FIX_VIOLATIONS, β-coupled).
      // LANDED (C1): C - Alg. 1 L2's min-leakage init IS init_mode =
      // min_size (livramento audit §2 item 4: the DEFERRED
      // classification was wrong - the option exists and works, and unlike
      // flach/sharma there is no clean-start trap because Alg. 3 is a
      // per-iteration repair, not an init pass). The bundle now flips it.
      // LANDED (MB item 4b): the α schedule (Alg. 1 L9), previously deferred -
      // see timing_scale below and its ADAPTATION note (the base anchor and the
      // denominator floor guard are ours; α₀ is unstated in the paper).
      // RETAINED NON-PAPER CONSTRUCTION (deliberate, see below): depth_budget.
      //
      // Livramento DATE'13: the fanout-slew cost term (Alg. 2 lines 12-14) is
      // its headline third cost component.
      //
      // The λ update is Livramento's OWN (Alg. 1 line 13, lambda *= (a_j +
      // D_ji)/a_i), not Tennakoon's. Until the post-M5 hardening audit this
      // preset was hosted on tennakoon_ratio on the strength of the summary's
      // TL;DR calling the update "Tennakoon-Sechen-style"; the PDF shows the
      // two rules have different denominators (Livramento's local step size is
      // rho_k = lambda_ji/a_i, Eq. 9; Tennakoon's Fig. 13 is
      // lambda_ji/(a_i - D_ji)) and agree only on a critical arc.
      //
      // Adaptation note (deferred): the β cap/slew relaxation. This preset
      // ablates the fanout-slew cost term, the line-13 update and (as of MB
      // item 4b) the α schedule, on the baseline seed.
      // RESTORED (bucket-2): upsize_hysteresis = 0 - the plain LRS argmin
      // this paper specifies. The shared 0.02 upsize deadband no longer
      // rides it (see the non-baseline block at the end of applyPreset).
      lambda_update = LambdaUpdate::kLivramentoRatio;
      cost_fanout_slew = true;
      // A1 (MB item 5): the paper's own rescheduled weight, Alg. 1 L9 - no
      // longer deferred. α is re-derived from fresh arrivals before every LRS
      // and applied as 1/α on our timing side (the paper's α weights POWER).
      // The base it scales is auto_median's λ-INVARIANT anchor, at
      // timing_bias = 12 (the family's shared balance - see tennakoon_partial;
      // α₀ = 1 then means "start where flach starts" and let the controller
      // servo from there, which is all the paper's unstated α₀ can mean). That
      // base is
      // forced, not chosen: this bundle runs the delay-proportional seed, whose
      // λ is an arc delay in SECONDS, so a λ-preserving base would preserve a
      // unit artifact and collapse the timing term - the same reason
      // tennakoon/reimann keep auto_median. The paper does not specify λ_0
      // either ("λ ← initial vector ∈ Ω_λ", unspecified), so there is no paper
      // magnitude being discarded here. See TimingScale::kLivramentoAlpha.
      timing_scale = TimingScale::kLivramentoAlpha;
      timing_bias = 12.0f;
      // The guard is our own depth budget, which appears in NO paper: unlike
      // Flach/Sharma/Reimann/Mangiras, Livramento states no local-slack
      // acceptance check (its Alg. 1 accepts the LRS argmin outright), so the
      // paper-faithful guard is arguably `none`. Retained deliberately (user
      // decision, bucket-2) for the same reason as chen/tennakoon: guard-free
      // is a different experiment, not a more faithful one, and the choice is a
      // campaign-stage ablation question (plan §6 Stage-1: guard ∈ {none,
      // depth_budget, local_slack_veto}). Same class as hardening finding #10.
      // Livramento keeps a best-feasible snapshot and restores it at the end
      // (its Alg. 1 stores the solution whenever timing is met and leakage
      // improves) - that is Flach's dominance rule, so we host it there
      // (adaptation note). Its LDP runs "up to 60 iterations" (§5).
      best_tracker = BestTrackerKind::kFlachDominance;
      max_iterations = 60;
      // H1 (C2): pure_cap. The paper has NO convergence test (§5: fixed budget,
      // no early exit), so any early exit is ours, not its. Historically the
      // sharpest one was fixed_iters' WNS-meets-margin exit, which fired before
      // the α reschedule and halted at first feasibility, making L9's "once
      // met, α grows again" recovery half - the exact phenomenon the paper's
      // Figs. 1-2 ablation turns on - structurally unreachable (livramento
      // audit §2.2). That exit was DELETED on 2026-07-29
      // (Termination::stopBeforeSweep); the pin stands on fixed_iters'
      // surviving 3-reject / zero-move stops, which would still cut the
      // 60-iteration budget the paper states.
      termination = TerminationKind::kPureCap;
      // C (C1): Alg. 1 L2 initializes every gate to its minimum-leakage
      // implementation. That is init_mode = min_size, and it has no clean-start
      // trap for this paper (Alg. 3's violation repair is per-iteration, not
      // init), so the flip is landed here. mu_policy = endpoint_ratio is set in
      // the post-shared-block override below.
      init_mode = InitMode::kMinSize;
      break;
    case Preset::kChinnery:
      // Chinnery & Sharma, "Integrating LR Gate Sizing in an Industrial
      // Place-and-Route Flow", ISPD'22 pp. 39-48 (chinnery_et_al.md; re-audit
      // reaudit/chinnery_et_al_reaudit.md, confidence high - all 10 pages read
      // visually, every equation/threshold/table verified). It is the
      // industrial productization of the Sharma TCAD'20 line, so this bundle is
      // deliberately the sharma-lineage machinery running the paper's OWN
      // two-phase schedule.
      //
      // WHAT THIS COLUMN MEASURES - stated once, because this paper's suffix
      // hides a larger gap than the others'. Reproduced here is the paper's
      // algorithmic core: the lambda = 1 seed, a slack-proportional
      // multiplicative update, the reverse-topological proportional KKT
      // projection, its Eq. 4 local-arc candidate cost, Flach's local-slack
      // guard, and the two-phase timing -> power termination battery with the
      // paper's own five constants and its 80-iteration cap. NOT reproduced is
      // the paper's engineering payload - the calibrated fast internal timer,
      // deterministic multithreading, multi-corner/multi-mode, and the
      // flow-placement recipe - which is most of what those four developer-
      // years bought (§7) and all of which is outside this framework's scope by
      // standing ruling. So a chinnery column that wins or loses is evidence
      // about the SCHEDULE, not about the industrial sizer.
      //
      // REPRODUCED, and worth naming because two of these are
      // OURS-BY-COINCIDENCE rather than adopted for this preset:
      //  - E3: "proportionately distributing the sum of the multipliers on the
      //    outgoing timing arcs among the incoming timing arcs, at every timing
      //    node, in reverse topological order" (§6) IS
      //    kkt_projection = proportional_reverse_topo, verbatim. This paper is
      //    the one whose projection the framework implements literally.
      //  - E4: Eq. 2's endpoint constraint (sum of in-arc multipliers = the
      //    endpoint multiplier) IS mu_policy = endpoint_lambda, the shared
      //    non-baseline default set below - mu derived from the in-arc lambda
      //    sum, not chosen. No override is needed here; that is the point.
      //  - H1: the full threshold battery, both phases, all five criteria and
      //    the 72 h cap (see TerminationKind::kThresholdBattery).
      //  - N: the paper resizes the netlist the flow hands it and never adds or
      //    deletes cells (§5 step 2, §7 item 8), so init_mode stays as_given -
      //    the struct default, deliberately, not by omission.
      //
      // ADAPTED (unavoidable or disclosed; not deferrals):
      //  - E1/E2: the update rule. The paper uses Sharma ICCAD'17 [13]
      //    ("multipliers on critical (non-critical) arcs are increased
      //    (decreased) in proportion to their timing slack") and defers the
      //    formula to Sharma TCAD'20 [7]; neither is implemented here. We host
      //    it on sharma_cexp - Sharma ICCAD'15's lambda *= (1 - slack/T)^cexp -
      //    which is the same authors' same slack-proportional multiplicative
      //    family, one paper earlier, with ONE exponent instead of the
      //    phase-swapped pair. It is a host, not a citation: a chinnery-vs-
      //    sharma_seq contrast is therefore a contrast of SCHEDULES (battery +
      //    full library vs stagnation windows + Fast-OLR) on a shared updater,
      //    which is the honest reading of what this framework can compare.
      //  - F3: "local slack degradation is effectively prevented by a large
      //    constant weight (1,000,000)" in the weighted libcell choice (§6/§8)
      //    becomes Flach's local-slack veto - a hard rejection instead of a
      //    1e6-weighted penalty. The two differ only where an LRS gain exceeds
      //    a million times the degradation, which is what "effectively
      //    prevented" says will not happen. gamma_local_slack = 0 is part of
      //    that reading; see the pin below.
      //  - A2 (added 2026-08-09, closing re-audit delta D3 - this list omitted
      //    the electrical veto while disclosing every other constraint-side
      //    item): "libcell alternatives that would INCREASE max-load-
      //    capacitance or max-input-slew violations are skipped outright" (§6,
      //    chinnery_et_al.md:102) is a relative rule, and output_drc_veto =
      //    relative is it - see the pin below. Two residues keep this an
      //    adaptation. (i) §6 names the max-input-slew at the sink node; our
      //    slew leg reads the DRIVER's own output-pin limit under the
      //    Elmore-linear estimate - the same net quantity at the other end, and
      //    the modeling every preset here shares. (ii) §8 records a further
      //    relaxation for designs "where the constraints are unmeetable"
      //    (:148), and the paper does not say what the surviving violation is
      //    measured against. Our reference is the INCUMBENT cell as the sweep
      //    finds it, re-read from the LIVE load every sweep - NOT a frozen
      //    flow-entry level. The two anchors are simply different, and neither
      //    dominates: on a pin whose load is stable ours is the tighter (it
      //    ratchets down as the gate improves), while on a pin whose load grows
      //    the bar rises with it and ours is the looser. That residue is RA F9
      //    and stays open; closing it needs a third anchor that snapshots the
      //    per-pin levels once at flow entry.
      //  - A1: `unit` (tw = l_med/d_med) for the paper's normalization scheme -
      //    power/area normalized to the average cell's, delays to the average
      //    arc delay of the most critical path (§6, from Reimann [6]). Ours is
      //    a median over the design rather than a mean over a reference set;
      //    the content that transfers is that lambda = 1 must mean "timing ~=
      //    leakage on a typical gate", which is exactly what this option gives.
      //  - A1/objective: Eq. 1 minimizes normalized power PLUS normalized area;
      //    we minimize leakageOrArea (area substitutes only on a library that
      //    publishes no leakage). The paper's own area-term ablation (Tables
      //    3-6: ~1% leakage for a large illegal-cell reduction) is therefore
      //    not expressible - see DEFERRED.
      //  - B1(c): the calibrated fast internal timer (NLDM+Elmore inside the
      //    sizer, calibrated once against a signoff CCS+Arnoldi timer, Eq. 5's
      //    pin-cap beta and Eq. 6's slew-scaling omega). This does not survive
      //    translation at all: OpenSTA is BOTH the reference and the internal
      //    timer here, so the miscorrelation those factors correct does not
      //    exist to be corrected (SYNTHESIS B3). Recorded as an adaptation
      //    rather than a deferral because there is nothing to implement.
      //
      // DEFERRED (hence _partial):
      //  - E1/E2 THE HEADLINE DEFERRAL: the phase-dependent exponent swap -
      //    exponent 4 on critical and 1 on non-critical arcs during the timing
      //    phase, swapped to 1/4 during power recovery (§6). Blocked twice
      //    over: the formula is deferred to [7] (the PDF is on disk at
      //    papers/references/sharma_chinnery_reimann_bhardwaj_chu_tcad_2020.pdf
      //    and has never been read) and the critical/non-critical threshold is
      //    unstated in this paper (§13 item 3). Consequence: our two phases
      //    differ ONLY in which exits are live, not in how lambda moves, so
      //    this preset reproduces the paper's stopping structure without its
      //    in-phase dynamics. REAUDIT §5-H6.
      //  - B2: sibling ("side") arcs are in Eq. 4's local-arc set - arcs of
      //    cells sharing a fanin - and our candidate cost has no side-arc term
      //    at all (REAUDIT F3, a framework-wide gap shared with
      //    flach/sharma/mangiras). The paper's lambda-weight sibling-arc
      //    skipping (§7 item 2, skip sibling arcs contributing < 1% of the
      //    local multiplier sum) exists only to make that term cheap, so its
      //    absence follows from the term's absence rather than being a second
      //    deferral.
      //  - F4: history-based adaptive libcell pruning (§7 item 3: after the
      //    first iteration with < 10% of cells changing, evaluate only the top
      //    P = 20% of each cell's cost-ranked alternates for M-1 iterations,
      //    M adaptive from 3). No F4 option expresses it - it is a cost-ranked
      //    prefix, not a neighbourhood, so neither sharma_fast_olr (a
      //    width-axis hill descent) nor mangiras_size_step (a +-1 band) is it.
      //    We run the full library, which is the un-approximated form of what
      //    the pruning approximates for a 20% speedup.
      //  - A1: the objective SELECTOR (leakage vs total power, +- an explicit
      //    area term - the four Tables 3-6 configurations). Total power needs a
      //    switching-activity source and the area term needs its own weighted
      //    cost component; neither exists (REAUDIT §6.1 item 4).
      //  - I: the "levelized fast WNS/TNS/DRV optimization" that follows every
      //    LR call in the host flow (§7 item 10) is the safety net that repairs
      //    LR-timer inaccuracy; the flow-placement decisions around it
      //    (pre-CTS/post-CTS/both, the skip-if-WNS-near-zero gate, the
      //    end-of-place roll-back) are flow scheduling, not preset content.
      //  - G: deterministic multithreading via extended MEEs (Algorithm 1) -
      //    the paper's one genuinely new algorithmic device. Out of scope by
      //    the standing threading ruling; we keep sequential semantics, which
      //    the scheme exists to make parallel runs match.
      //  - A3: multi-corner (per-corner vectors, dominant-corner pruning) and
      //    multi-mode (timing-graph replication). Out of scope by ruling.
      //  - H1 (framework-wide): the per-arc lambda*d cost is priced port-worst
      //    here, and this family's printed cost is the one most exposed to that
      //    seam (REAUDIT §5-H1).
      // ALIGNED, not deferred: the paper does not resize registers and does no
      // buffering or cell insertion/deletion inside the sizer (§8), which is
      // also exactly what GLOBAL_SIZING does. Its ~5% penalty vs the host tool
      // for that restriction is a property of the comparison, not of us.
      lambda_update = LambdaUpdate::kSharmaCexp;
      // D: "all lambda = 1 before the first iteration" (§6). The sharma/chen
      // constant-1 seed, live under this bundle's unit + endpoint_lambda pair
      // (see lambda_init_value's doc for why it is inert under others).
      lambda_seed = LambdaSeed::kConstant;
      lambda_init_value = 1.0f;
      // A1: the lambda-free median anchor. With lambda = 1 seeded uniformly,
      // `unit` makes the objective "leakage + (timing on a median gate)", which
      // is the normalized p_hat + a_hat + sum lambda*d_hat of Eq. 3 with the
      // paper's own lambda. timing_bias is orthogonal under `unit` and pinned
      // to 1.0 to say so.
      timing_scale = TimingScale::kUnit;
      timing_bias = 1.0f;
      // B2: Eq. 4's local arcs are "arcs of fanin nets and fanin cells, arcs of
      // g itself, arcs of sibling cells, and arcs of fanout nets and fanout
      // cells". cost_upstream_load (on by default) prices the fanin half;
      // cost_fanout_slew is the only mechanism that prices the FANOUT half - a
      // frozen per-output-pin sensitivity times the candidate's output-slew
      // change, i.e. a linearization of what the paper recomputes exactly on
      // its local graph copy. Turning it on is the closer reading of Eq. 4 than
      // leaving the fanout level unpriced.
      //
      // cost_global_phi stays OFF, and that is a real difference from
      // sharma_seq_partial: Flach's phi back-propagates a sensitivity over the
      // WHOLE downstream cone, while Chinnery's local set stops one level out
      // and calls even sibling arcs second-order (§7 item 2). This paper cites
      // Flach [5] for the local-slack guard, not for a downstream sensitivity
      // term. Keeping phi off also keeps RSZ-0429 (phi + fanout_slew both price
      // the immediate sink level) silent, so the fanout level is priced once.
      cost_fanout_slew = true;
      // F4: the full alternate set every iteration - the paper's own
      // enumeration minus its runtime pruning (DEFERRED above). Pinned
      // explicitly so a future struct-default change cannot move this preset
      // into a restricted move set it never had.
      move_set = MoveSet::kFullLibrary;
      // A2 (RULED 2026-08-09): §6's "skipped outright" filter is written
      // against an INCREASE in violations, not against their existence, and
      // this preset starts from the incoming netlist (init_mode = as_given, §5
      // step 2), so unlike flach/sharma it meets the LR loop with whatever
      // violations the design arrived with. That is precisely the case the two
      // modes disagree on. See the A2 entry in ADAPTED above for the two
      // residues and for why this closes re-audit delta D3.
      output_drc_veto = OutputDrcVeto::kRelative;
      // F3: "Following Flach et al. [5], the change in local slack is also
      // computed per alternate libcell, and the libcell with the minimum
      // weighted sum of LRS cost and local slack degradation is chosen; local
      // slack degradation is effectively prevented by a large constant weight
      // (1,000,000)" (§6). So the guard is Flach's veto.
      //
      // gamma_local_slack = 0 (NOT the default 1.0) is the second half of that
      // sentence. Our veto's gamma = 1 + gamma_local_slack*(-WNS/T) is Flach's
      // Eq. 14 hill-climbing tolerance, which Chinnery does not have: a 1e6
      // weight admits no tolerance schedule at all, it just forbids
      // degradation. 0.0 makes gamma == 1 from the first iteration, which is
      // "prevented". Leaving the default would run a Flach mechanism this paper
      // never adopted under Chinnery's name - the undisclosed-divergence class
      // the bucket passes exist to kill.
      //
      // near_met_gate_frac likewise stays at the ungated default: Chinnery's
      // local-slack term is part of EVERY libcell evaluation with no timing
      // precondition, unlike Sharma's "we apply this check as we recover power,
      // after the design timing is within 1% of the target", which is what
      // sharma_seq_partial pins 0.01 for. The battery's phases already carry
      // this paper's timing-vs-power structure; adding Sharma's gate on top
      // would be a second, unsourced phase boundary.
      downsize_guard = DownsizeGuard::kLocalSlackVeto;
      gamma_local_slack = 0.0f;
      // H1: the battery, with the paper's five constants pinned rather than
      // inherited. They ARE the struct defaults today (they were written for
      // this paper), and that is exactly why they are restated: this is the
      // only preset that selects the battery, so a struct-default change would
      // otherwise silently de-paper it - the reimann max_iterations precedent
      // (reimann audit §2.3).
      // Timing phase: TNS within 10% of T, or WNS within 1% of it, or TNS
      // improving < 10% over the window. Power phase: power improving < 1%
      // (the paper's own loosening from 0.1%, for a 13% speedup). Window 3 =
      // "over the last 3 iterations"; the cap is 72 h.
      termination = TerminationKind::kThresholdBattery;
      term_tns_target_frac = 0.10f;
      term_wns_target_frac = 0.01f;
      term_tns_improve_frac = 0.10f;
      term_power_improve_frac = 0.01f;
      term_improve_window = 3;
      term_wall_limit_s = 259200.0f;
      // H2: none. The sizer has no best-so-far snapshot/restore - §5 step 6
      // commits the final assignment - and the struct default flach_dominance
      // would import a Flach mechanism (a TNS-gated leakage-dominance harvest)
      // this paper does not have, the same undisclosed-default problem
      // chen/tennakoon fixed.
      //
      // CONSEQUENCE, stated because it is sharper here than for chen/tennakoon:
      // the battery's second power-phase exit STOPS the run, it does not undo
      // the sweep that tripped it, so a chinnery run that ends on kTnsDegraded
      // commits the degraded assignment. That is faithful and it is also a
      // known hole: in the paper the stop-loss is backed by a HOST-FLOW
      // roll-back ("roll back subset of changes if WNS is degraded", Fig. 4)
      // and by the levelized fast optimization that follows every LR call -
      // neither of which exists around GLOBAL_SIZING. Adding a restore here
      // would be a non-paper mechanism in a paper column (principle B), so the
      // exit ships as the paper has it and this is the disclosure. A campaign
      // arm that ends on that reason should be read as "did not converge", not
      // as a result; RSZ-0451 names the reason for exactly that purpose.
      best_tracker = BestTrackerKind::kNone;
      // H1 budget: 80, the paper's own hard iteration cap (§5; "almost never"
      // reached - its runs average 23 iterations, 13 timing + 10 power, and the
      // battery is what actually stops them).
      max_iterations = 80;
      break;
  }

  // Every paper's LRS is a sequential topological sweep that commits each gate
  // before evaluating the next (Flach Alg. 3, Livramento, Reimann, ...), so the
  // paper presets share the mechanics-fidelity default: the Gauss-Seidel engine
  // with the paper-faithful truncated local refresh in forward-topological
  // order (M4). rsz_baseline keeps the parallel Jacobi engine. gs_local /
  // forward_topo are also the struct defaults; set explicitly to document the
  // intent and stay correct if the defaults ever change.
  //
  // Chinnery is no exception even though its sizer is multi-threaded: it
  // resizes "in forward topological order" and its extended mutual-exclusion
  // edges (Algorithm 1) exist precisely to make the parallel run reproduce one
  // deterministic sequential order, which is the semantics this engine has.
  if (p != Preset::kRszBaseline) {
    sweep_engine = SweepEngineKind::kGaussSeidelTopo;
    gs_refresh = GsRefresh::kLocal;
    traversal = Traversal::kForwardTopo;
    // E4 (MB item 5): mu is *derived*, not chosen - the endpoint's own in-arc
    // lambda sum, which the projection then anchors to. This is the papers'
    // shared endpoint treatment (Flach's uniform seed covers endpoint arcs and
    // Alg. 2 slack-scales them; Livramento's lambda_jpo Alg. 1 L12 and
    // Mangiras's Eq. 6 lambda_ik are ordinary multipliers that ANCHOR the
    // reverse-topological distribution rather than being overwritten by it),
    // and it is what lets a uniform lambda seed magnitude survive the
    // projection at all: the other three options anchor to a slack-derived mu
    // that carries no information about lambda's magnitude, so they cancel a
    // uniform seed constant c at iteration 0 whatever the update rule does.
    // Baseline keeps reseed_each_iter (no paper contains it).
    //
    // This is the family default; the C1 override switch below replaces it on
    // the three presets whose papers have an explicit endpoint-multiplier
    // writer (chen -> endpoint_additive, tennakoon/livramento ->
    // endpoint_ratio), which endpoint_lambda alone left with no live
    // dual-ascent branch (the B1 freeze rows). endpoint_ratio/additive still
    // derive mu_0 from the endpoint in-arc lambda sum (so seed magnitude
    // survives), then add the papers' own branch-1 pressure on top - see the
    // MuPolicy enum doc.
    mu_policy = MuPolicy::kEndpointLambda;
    // Every paper's LRS commits the plain argmin over the candidate set, with
    // no acceptance deadband of any kind: the subproblem IS "pick the candidate
    // minimizing leakage + λ-delay". Sharma §5.2 - "the cell minimizing
    // (lambda-delay-cost + leakage) is chosen"; Mangiras §5 - "after all
    // options are tried, the minimizer is committed"; Chen's SOLVE_LRS/μ is a
    // greedy coordinate descent that takes any improving move. The rest of the
    // family is the same greedy per-gate argmin (hardening finding #10 audited
    // the seven presets that then existed; chinnery's Eq. 4 - the minimum
    // weighted sum of LRS cost and local-slack change - joined them unchanged).
    // The 0.02 upsize hysteresis is OpenROAD's own noise filter
    // (hardening finding #10): until the bucket-2 pass it was an unconditional
    // constant inside both sweep engines, so it silently rode every preset and
    // priced a 2% tax into every paper's objective. rsz_baseline and the struct
    // default keep it (it is the baseline's own heuristic); the paper presets
    // restore the argmin.
    //
    // Any candidate rejection a paper DOES specify is a downsize_guard (Flach's
    // local-slack veto and its relatives), which is a separate axis - this is
    // only the shared cost-noise deadband.
    upsize_hysteresis = 0.0f;
  }

  // E4 endpoint-pressure override (C1). The three pre-Flach presets whose
  // papers carry an explicit endpoint-multiplier writer get it as a first-class
  // option instead of the shared endpoint_lambda default, which left them with
  // no live dual-ascent branch (the measured B1 freeze/decay rows). Chen's is
  // additive (SOLVE_LDP step 3 i=0), Tennakoon's Fig. 13 branch 1 and
  // Livramento's Alg. 1 L12 are the identical multiplicative ratio. Placed
  // after the shared block, which sets the endpoint_lambda default for every
  // non-baseline preset.
  switch (p) {
    case Preset::kChen:
      mu_policy = MuPolicy::kEndpointAdditive;
      break;
    case Preset::kTennakoon:
    case Preset::kLivramento:
      mu_policy = MuPolicy::kEndpointRatio;
      break;
    default:
      break;
  }
}

namespace {

// The three mu policies that DERIVE mu from the endpoint's own in-arc lambda
// sum, so the projection they anchor is positively homogeneous in lambda and
// whatever the E1/E2 updater did to the field survives it. endpoint_ratio and
// endpoint_additive derive mu_0 that way and then add their paper's own
// branch-1 pressure, which is still a lambda-derived quantity.
//
// The complement - reseed_each_iter, seed_once, update_as_lambda - all anchor
// to seedBaselineMu's slack field ((margin - slack)^p normalized to max 1),
// which carries no information about lambda at all. That is the annihilation:
// the endpoint boundary is rewritten from slack every projection, so two runs
// whose updaters produced very different lambda fields are redistributed onto
// the same one. See the MuPolicy enum doc block.
bool muPolicyReadsLambda(const GlobalSizingConfig::MuPolicy policy)
{
  switch (policy) {
    case GlobalSizingConfig::MuPolicy::kEndpointLambda:
    case GlobalSizingConfig::MuPolicy::kEndpointRatio:
    case GlobalSizingConfig::MuPolicy::kEndpointAdditive:
      return true;
    case GlobalSizingConfig::MuPolicy::kReseedEachIter:
    case GlobalSizingConfig::MuPolicy::kSeedOnce:
    case GlobalSizingConfig::MuPolicy::kUpdateAsLambda:
      return false;
  }
  return false;
}

}  // namespace

void GlobalSizingConfig::resolveLambdaMuPairing(utl::Logger* logger)
{
  mu_auto_paired = false;
  // rsz_baseline's own rule is exempt (see the header): its lambda has no paper
  // magnitude to preserve, and auto-moving the baseline's mu policy would move
  // the ablation's denominator.
  if (lambda_update == LambdaUpdate::kNormSubgradient) {
    return;
  }
  // A bundle that already reads lambda at the endpoint boundary needs nothing:
  // this covers every paper preset (flach/sharma/mangiras/reimann/chinnery take
  // endpoint_lambda, chen endpoint_additive, tennakoon/livramento
  // endpoint_ratio), so a preset column is never rewritten by this rule.
  if (muPolicyReadsLambda(mu_policy)) {
    return;
  }
  if (mu_policy_explicit) {
    // Honored, and named. The combination is exactly the one the pilot found
    // unidentifiable, so a run that lands here by accident would report a null
    // lambda axis as a result. Info, not warn: measuring the annihilation is a
    // legitimate cell (it is what the iteration-1 E cells did), it just must
    // never be the silent default.
    logger->info(RSZ,
                 448,
                 "GLOBAL_SIZING: lambda_update={} with mu_policy={} is a "
                 "known-annihilated combination - {} re-anchors the endpoint "
                 "boundary from a slack field every projection, which discards "
                 "the lambda magnitude the updater produced, so the "
                 "lambda_update axis is not identifiable in this cell "
                 "(iteration-2 plan §2.2-6). Honoring the explicit policy.",
                 toString(lambda_update),
                 toString(mu_policy),
                 toString(mu_policy));
    return;
  }
  const MuPolicy previous = mu_policy;
  mu_policy = MuPolicy::kEndpointLambda;
  mu_auto_paired = true;
  logger->warn(
      RSZ,
      447,
      "GLOBAL_SIZING: lambda_update={} auto-pairs with "
      "mu_policy=endpoint_lambda (was {}). Under a mu policy that does "
      "not read lambda the flow projection re-anchors every endpoint "
      "from a slack field, which annihilates the update rule's effect "
      "on the multiplier field - the lambda_update axis would not be "
      "identifiable (iteration-2 plan §2.2-6). Pass -mu_policy "
      "explicitly to override.",
      toString(lambda_update),
      toString(previous));
}

bool GlobalSizingConfig::validate(utl::Logger* logger) const
{
  // Single home for the SYNTHESIS §5 cross-axis constraints. Each rejection
  // cites its constraint.
  //
  // init_seed is read by exactly one mode. A sweep that varies it under any
  // other mode produces bit-identical runs while RSZ-0417 still echoes a
  // different init_seed per run, so the analysis would read a real within-arm
  // variance of zero instead of a misconfigured cell. Warn, don't reject: the
  // seed is harmless, it is the sweep that is wrong.
  if (init_seed != 0 && init_mode != InitMode::kRandom) {
    logger->warn(RSZ,
                 442,
                 "GLOBAL_SIZING: init_seed={} is inert under init_mode={} - "
                 "only init_mode=random draws from it, so every seed gives the "
                 "same initial solution.",
                 init_seed,
                 toString(init_mode));
  }
  // fast_olr_start_iter is read by exactly one move set, and the reasoning is
  // RSZ-0442's verbatim: a sweep that varies an inert knob produces
  // bit-identical runs while RSZ-0417 still echoes a different value per run,
  // so the analysis reads a real within-arm variance of zero instead of a
  // misconfigured cell. Warn, don't reject - the value is harmless, it is the
  // sweep that is wrong.
  if (fast_olr_start_iter != GlobalSizingConfig{}.fast_olr_start_iter
      && move_set != MoveSet::kSharmaFastOlr) {
    logger->warn(RSZ,
                 449,
                 "GLOBAL_SIZING: fast_olr_start_iter={} is inert under "
                 "move_set={} - only sharma_fast_olr has a switch-over "
                 "iteration, so every value gives the same run.",
                 fast_olr_start_iter,
                 toString(move_set));
  }
  // The six Chinnery battery constants are read by exactly one termination
  // rule, and the reasoning is RSZ-0442/0449's verbatim: a sweep that varies an
  // inert knob produces bit-identical runs while RSZ-0417 still echoes a
  // different `chinnery=` field per run, so the analysis reads a real
  // within-arm variance of zero instead of a misconfigured cell. Warn, don't
  // reject - the values are harmless, it is the sweep that is wrong. One
  // warning for the group: they are one paper's one rule, and a cell that gets
  // the termination wrong gets all six wrong together.
  //
  // KNOWN LIMITATION, shared with RSZ-0442 and RSZ-0449: "differs from the
  // struct default" is the proxy for "somebody set this". A preset that pins a
  // constant to a value equal to the default (chinnery pins all six, and all
  // six ARE the defaults - they were written for that paper) therefore goes
  // inert silently if its termination is overridden. Closing that needs
  // per-knob provenance, like mu_policy_explicit; it is not done for any of the
  // three checks, so the family stays consistent rather than half-fixed.
  const GlobalSizingConfig defaults;
  if (termination != TerminationKind::kThresholdBattery
      && (term_tns_target_frac != defaults.term_tns_target_frac
          || term_wns_target_frac != defaults.term_wns_target_frac
          || term_tns_improve_frac != defaults.term_tns_improve_frac
          || term_power_improve_frac != defaults.term_power_improve_frac
          || term_improve_window != defaults.term_improve_window
          || term_wall_limit_s != defaults.term_wall_limit_s)) {
    logger->warn(RSZ,
                 452,
                 "GLOBAL_SIZING: the term_* battery constants "
                 "({:.3g}/{:.3g}/{:.3g}/{:.3g}/{}/{:.3g}) are inert under "
                 "termination={} - only threshold_battery reads them, so every "
                 "value gives the same run.",
                 term_tns_target_frac,
                 term_wns_target_frac,
                 term_tns_improve_frac,
                 term_power_improve_frac,
                 term_improve_window,
                 term_wall_limit_s,
                 toString(termination));
  }
  // §5.3: the state-adaptive seed reads the *current* sizes to infer past
  // criticality (the leakage-vs-min power ratio). An init pass would erase
  // that state before the seed runs, so it is incompatible with as_given only.
  if (lambda_seed == LambdaSeed::kStateAdaptive
      && init_mode != InitMode::kAsGiven) {
    logger->error(RSZ,
                  421,
                  "GLOBAL_SIZING: lambda_seed=state_adaptive requires the "
                  "as-given initial solution (init_mode=as_given); got "
                  "init_mode={} (SYNTHESIS §5.3: the seed infers past "
                  "criticality from current sizes, which any other init mode "
                  "erases).",
                  toString(init_mode));
    return false;
  }
  // §5.3-adjacent (MB item 5): Mangiras Eq. 6 sets each endpoint's lambda as a
  // boundary condition the reverse-topological distribution anchors to (§7:
  // "Endpoint-arc LMs keep their Equation (6) values and act as boundary
  // conditions"). Re-seeding mu from endpoint slacks every iteration honors
  // that boundary at iteration 0 and destroys it at iteration 1 - the seed's
  // whole contribution survives exactly one projection. The two options are
  // therefore incoherent together. Warn rather than reject: the cross is a
  // legitimate ablation cell (it measures precisely what Eq. 6's boundary is
  // worth), it is just never what a user means by "run Mangiras".
  if (lambda_seed == LambdaSeed::kStateAdaptive
      && mu_policy == MuPolicy::kReseedEachIter) {
    logger->warn(RSZ,
                 436,
                 "GLOBAL_SIZING: lambda_seed=state_adaptive with "
                 "mu_policy=reseed_each_iter honors Mangiras Eq. 6's endpoint "
                 "boundary at iteration 0 and overwrites it at iteration 1; "
                 "mu_policy=endpoint_lambda preserves it.");
  }
  // estimation_loop restores the initial solution each dry-run iteration, so
  // the init pass is not erased but the estimate targets the initialized state,
  // which muddies the warm-start; warn rather than reject.
  if (lambda_seed == LambdaSeed::kEstimationLoop
      && init_mode != InitMode::kAsGiven) {
    logger->warn(RSZ,
                 422,
                 "GLOBAL_SIZING: lambda_seed=estimation_loop is intended for "
                 "the as-given initial solution; init_mode={} makes the "
                 "estimated multipliers target the initialized state.",
                 toString(init_mode));
  }
  // §5.6-adjacent: cost_global_phi (Flach) and cost_delta_delay (Ozdal) are two
  // different global-effect estimators for the same downstream-impact cost
  // component; enabling both double-prices it with incompatible models. Hard
  // reject. cost_fanout_slew composes with either (it prices only the immediate
  // fanout, a distinct neighborhood term).
  if (cost_global_phi && cost_delta_delay) {
    logger->error(RSZ,
                  424,
                  "GLOBAL_SIZING: cost_global_phi and cost_delta_delay are "
                  "mutually exclusive (SYNTHESIS §5.6: they are two different "
                  "global-effect estimators for the same cost component). "
                  "Enable at most one.");
    return false;
  }
  // Both terms include the immediate sink level's λ_sink·(δd/δslew): φ's Eq. 11
  // recurrence starts there and the fanout-slew term IS that level. Composing
  // them double-prices it. Allowed (the fanout term is exact where φ's
  // dominant-arc collapse is heuristic, so the cross is interesting) but
  // flagged. No preset composes them.
  if (cost_global_phi && cost_fanout_slew) {
    logger->warn(RSZ,
                 429,
                 "GLOBAL_SIZING: cost_global_phi and cost_fanout_slew both "
                 "price the immediate sink level's slew sensitivity, so "
                 "enabling both double-prices it.");
  }
  // F1/F2: the Jacobi engine evaluates one stable sweep-start snapshot with no
  // per-commit refresh and in a fixed order, so traversal and gs_refresh are
  // Gauss-Seidel-only knobs. Setting either to a non-default value under Jacobi
  // has no effect; warn rather than reject (the value is simply ignored).
  if (sweep_engine == SweepEngineKind::kJacobiSnapshot
      && (traversal != Traversal::kForwardTopo
          || gs_refresh != GsRefresh::kLocal)) {
    logger->warn(RSZ,
                 425,
                 "GLOBAL_SIZING: traversal={} and gs_refresh={} are ignored by "
                 "the jacobi_snapshot engine (Gauss-Seidel-only knobs).",
                 toString(traversal),
                 toString(gs_refresh));
  }
  // §3.2-F guard/engine crosses. Both are allowed - they are interesting
  // ablation cells - but each pairs a guard with the engine it was not designed
  // for, so each is flagged.
  //
  // The veto's local slacks come from the sweep-start snapshot under Jacobi
  // (there is no per-commit refresh to read), so every gate in the sweep is
  // vetoed against the same stale timing picture, and simultaneous accepted
  // downsizes can still add up on a shared path - exactly the overshoot the
  // depth budget was built to prevent.
  if (sweep_engine == SweepEngineKind::kJacobiSnapshot
      && downsize_guard == DownsizeGuard::kLocalSlackVeto) {
    logger->warn(RSZ,
                 430,
                 "GLOBAL_SIZING: downsize_guard=local_slack_veto under the "
                 "jacobi_snapshot engine can only test candidates against the "
                 "frozen sweep-start required times, and does not bound the "
                 "sum of simultaneous downsizes on a path (SYNTHESIS §3.2-F).");
  }
  // Under Gauss-Seidel the budget is redundant conservatism: commits are
  // sequential, so the paper-faithful guard is the local-slack veto, which sees
  // each gate's real post-commit slack instead of a depth-normalized share of a
  // sweep-start budget.
  if (sweep_engine == SweepEngineKind::kGaussSeidelTopo
      && downsize_guard == DownsizeGuard::kDepthBudget) {
    logger->warn(RSZ,
                 431,
                 "GLOBAL_SIZING: downsize_guard=depth_budget under the "
                 "gauss_seidel_topo engine keeps a Jacobi-era guard (budgets "
                 "frozen at sweep start) where the papers use the local-slack "
                 "veto (SYNTHESIS §3.2-F).");
  }
  return true;
}

}  // namespace rsz
