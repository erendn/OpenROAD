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
    case GlobalSizingConfig::InitMode::kMinSizeFixcap:
      return "min_size_fixcap";
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
    case GlobalSizingConfig::LambdaUpdate::kSharmaArcSlack:
      return "sharma_arc_slack";
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
    case GlobalSizingConfig::BestTrackerKind::kLivramentoFeasible:
      return "livramento_feasible";
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

const char* toString(const GlobalSizingConfig::PowerObjective objective)
{
  switch (objective) {
    case GlobalSizingConfig::PowerObjective::kLeakage:
      return "leakage";
    case GlobalSizingConfig::PowerObjective::kTotal:
      return "total";
  }
  return "unknown";
}

const char* toString(const GlobalSizingConfig::TimingCost cost)
{
  switch (cost) {
    case GlobalSizingConfig::TimingCost::kWorstArc:
      return "worst_arc";
    case GlobalSizingConfig::TimingCost::kPerArc:
      return "per_arc";
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
  } else if (std::strcmp(name, "min_size_fixcap") == 0) {
    out = IM::kMinSizeFixcap;
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
  } else if (std::strcmp(name, "sharma_arc_slack") == 0) {
    out = LU::kSharmaArcSlack;
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
  } else if (std::strcmp(name, "livramento_feasible") == 0) {
    out = BT::kLivramentoFeasible;
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

bool parsePowerObjective(const char* name,
                         GlobalSizingConfig::PowerObjective& out)
{
  using PO = GlobalSizingConfig::PowerObjective;
  if (std::strcmp(name, "leakage") == 0) {
    out = PO::kLeakage;
  } else if (std::strcmp(name, "total") == 0) {
    out = PO::kTotal;
  } else {
    return false;
  }
  return true;
}

bool parseTimingCost(const char* name, GlobalSizingConfig::TimingCost& out)
{
  using TC = GlobalSizingConfig::TimingCost;
  if (std::strcmp(name, "worst_arc") == 0) {
    out = TC::kWorstArc;
  } else if (std::strcmp(name, "per_arc") == 0) {
    out = TC::kPerArc;
  } else {
    return false;
  }
  return true;
}

void GlobalSizingConfig::applyPreset(const Preset p)
{
  // Reset every field to its struct default, then set only the fields this
  // preset changes. The struct defaults match rsz_baseline except for
  // best_tracker, which defaults to flach_dominance, so rsz_baseline sets its
  // own tracker below. Options given after the preset still win, because
  // Resizer::initBlock applies the dbProperty overrides after this call. The
  // paper constants (flach_k_*, sharma_*, reimann_*, lambda_update_c) default
  // to each paper's value and are read only by the matching updater, so a
  // paper preset only has to select its updater.
  //
  // Timing/power balance. The objective is power + tw * sum(lambda * d).
  // Presets that seed lambda with a paper constant use timing_scale = unit,
  // where tw = l_med / d_med does not depend on lambda. lambda is then the
  // timing/power ratio on a median gate, so the paper's constant keeps its
  // meaning. timing_bias has no effect under unit and is set to 1.0 to make
  // that explicit.
  //
  // Presets that keep OpenROAD's delay-proportional seed start with lambda
  // equal to an arc delay in seconds (about 1e-10), a magnitude that means
  // nothing. Under unit that magnitude would reach the cost and make the
  // timing term vanish. These presets use auto_median (or livramento_alpha,
  // which is built on it), where tw is proportional to 1 / lambda and the
  // magnitude cancels. They set timing_bias = 12, which puts the timing term
  // at about 12 times the power of a median gate: the same balance that
  // Flach's initial multiplier of 12 gives under unit.
  //
  // Presets whose paper has no early-exit rule use termination = pure_cap,
  // which stops only at max_iterations. The default fixed_iters also stops
  // after 3 rejected sweeps or 2 sweeps without moves, which can end a
  // paper's multiplier schedule before it has moved the design.
  *this = GlobalSizingConfig{};
  preset = p;
  // Set only here, because this function is the only way a preset is
  // applied. The reset above clears it first.
  preset_explicit = true;
  switch (p) {
    case Preset::kRszBaseline:
      // OpenROAD's own best-solution rule: keep the last sweep whose WNS
      // matched or beat every earlier sweep. No paper uses it.
      best_tracker = BestTrackerKind::kWnsPassReject;
      break;
    case Preset::kChen:
      // Chen, Chu and Wong, ICCAD 1998. Settings:
      //  - Additive subgradient update (SOLVE_LDP step 3) and its endpoint
      //    update (mu_policy = endpoint_additive, set after the switch). The
      //    paper leaves the step size free; rho_k = c/k is used.
      //  - Constant initial multipliers. The paper starts from an arbitrary
      //    vector; 1 is used here.
      //  - Every subproblem is solved from minimum size (SOLVE_LRS/mu step
      //    1): the init pass sets every gate to its minimum size, and every
      //    later iteration starts by setting each gate back to that cell
      //    (restart_each_iteration).
      //  - Each subproblem is solved by repeating the sweep with the
      //    multipliers fixed until no gate changes (SOLVE_LRS/mu step 4).
      //
      // Differences from the paper:
      //  - The paper sizes continuously and resizes each gate in closed form
      //    (Lemma 2). The library here is discrete, so the subproblem is
      //    solved by greedy passes over the candidate cells.
      //  - The paper repeats the sweep with no limit; its continuous, convex
      //    subproblem settles. With discrete cells two gates can swap back
      //    and forth, so the loop stops after 10 sweeps.
      //  - The paper minimizes area; here the objective is leakage, with area
      //    used only on a library that has no leakage data.
      //  - The paper stops when the duality gap is small (SOLVE_LDP step 6).
      //    That bound relies on the convex posynomial formulation. NLDM
      //    delays over a discrete library are not convex, so the Lagrangian
      //    value is not a bound here, and the run stops at a fixed cap.
      //  - The paper has no candidate-rejection rule, so no downsize guard
      //    would be closest. The preset keeps OpenROAD's depth budget; use
      //    -downsize_guard to change it.
      //
      // Not implemented:
      //  - Chen's projection onto the nearest point satisfying the KKT
      //    conditions (SOLVE_LDP step 4). The proportional
      //    reverse-topological projection is used instead.
      lambda_update = LambdaUpdate::kChenSubgradient;
      lambda_seed = LambdaSeed::kConstant;
      lambda_init_value = 1.0f;
      timing_scale = TimingScale::kUnit;
      timing_bias = 1.0f;
      // min_size picks the lowest-leakage cell of each group, with drive
      // strength as the tie-break: the discrete counterpart of the paper's
      // lower size bound. The paper has no Vth dimension.
      init_mode = InitMode::kMinSize;
      restart_each_iteration = true;
      max_inner_sweeps = 10;
      termination = TerminationKind::kPureCap;
      // The paper keeps the final iterate; it has no best-so-far restore.
      best_tracker = BestTrackerKind::kNone;
      // Fig. 8 shows convergence from a cold start after about 50 to 100
      // iterations (read from the plot).
      max_iterations = 100;
      break;
    case Preset::kTennakoon:
      // Tennakoon and Sechen, ICCAD 2002 (Forge). Settings:
      //  - The multiplicative arrival-ratio update of Fig. 13 (second case,
      //    internal gates) on OpenROAD's delay-proportional seed.
      //  - Fig. 13's first case, for arcs into the primary outputs, is the
      //    endpoint update mu_policy = endpoint_ratio, set after the switch.
      //  - Each subproblem is solved by repeating the topological sweep with
      //    the multipliers fixed until no gate changes (Sec. 3.2).
      //
      // Differences from the paper:
      //  - The paper runs until convergence without stating a test. Here the
      //    run uses a fixed budget.
      //  - The paper repeats the sweep with no limit; its continuous, convex
      //    subproblem settles. With discrete cells two gates can swap back
      //    and forth, so the loop stops after 10 sweeps.
      //  - The downsize guard is the depth budget, as in chen_partial.
      //
      // Not implemented:
      //  - The steepest-descent and contour search (Sec. 5.1-5.2) that derive
      //    the starting multipliers from the gate sizes. This is the paper's
      //    main contribution; the preset uses OpenROAD's seed instead.
      //  - Fig. 13's third case (arcs from primary-input drivers).
      lambda_update = LambdaUpdate::kTennakoonRatio;
      timing_scale = TimingScale::kAutoMedian;
      timing_bias = 12.0f;
      max_inner_sweeps = 10;
      termination = TerminationKind::kPureCap;
      // The paper keeps the result of the last subproblem solve.
      best_tracker = BestTrackerKind::kNone;
      // The paper reports run times but no iteration count; 100 matches
      // chen_partial.
      max_iterations = 100;
      break;
    case Preset::kFlach:
      // Flach et al., TCAD 2014. Settings:
      //  - Alg. 2 update (asymmetric slack scaling with the k schedule). All
      //    multipliers start at 12, the authors' ISPD 2013 contest value;
      //    the paper does not state one.
      //  - The initial solution of Sec. IV / Fig. 1 (min_size_fixviol).
      //  - The two candidate filters of Alg. 4 (output_drc_veto = relative,
      //    downsize_guard = local_slack_veto).
      //  - The best-solution rule of Alg. 1 (flach_dominance).
      //  - No global sensitivity term phi (Eqs. 5 and 11; see cost_global_phi
      //    below).
      //
      // Differences from the paper:
      //  - The initial repair differs from Li et al.'s procedure (see
      //    init_mode below).
      //  - The load veto also applies to max slew (see output_drc_veto).
      //
      // Not implemented:
      //  - The timing recovery and power reduction passes that follow LR
      //    (Sec. VIII, Algs. 6-7). The paper's final leakage (Table IV)
      //    depends on them.
      lambda_update = LambdaUpdate::kFlachSlackScaling;
      lambda_seed = LambdaSeed::kConstant;
      lambda_init_value = 12.0f;
      timing_scale = TimingScale::kUnit;
      timing_bias = 1.0f;
      // Sec. IV: every gate starts at its lowest-leakage version, then a
      // pass from outputs to inputs removes the load and slew violations
      // that start creates. Alg. 4 only forbids increasing a load violation,
      // so it keeps the run free of violations only because this start is
      // free of them.
      //
      // The paper's repair is Li et al.'s procedure with alpha = 0.7, whose
      // meaning the paper leaves to Li et al.; the repair here has no alpha.
      // Li's pass also re-picks the lowest-leakage legal version of every
      // gate, while this one upsizes only the gates that violate. After the
      // minimum-size reset the two agree, except on a violating gate that no
      // cell of its group can fix: the repair leaves it at minimum and
      // reports it (RSZ-0445). The relative veto below still lets the sweep
      // resize that gate later, as long as its violation does not grow.
      init_mode = InitMode::kMinSizeFixviol;
      // Alg. 4 line 6 rejects a version only if the load violation has
      // increased; the rule exists to keep violations from spreading and to
      // avoid extrapolating the delay tables, not to freeze gates. The paper
      // states it for max load only: slew violations are removed in the
      // initial solution and have no per-candidate check. The relative mode
      // applies the same rule to both limits, so on slew it is stricter than
      // the paper but looser than the absolute veto.
      output_drc_veto = OutputDrcVeto::kRelative;
      // The paper turns phi off for its benchmarks with RC wires, which it
      // calls incompatible with the technique (Sec. IX-B). The wires here are
      // RC as well.
      cost_global_phi = false;
      // Alg. 4 lines 12-14: reject a version that makes the local negative
      // slack worse than gamma times its original value, with gamma from
      // Eq. 14.
      // Alg. 1 line 9: store a solution when |TNS| < 10% of T and its
      // leakage is lower, and restore the best one at the end.
      downsize_guard = DownsizeGuard::kLocalSlackVeto;
      best_tracker = BestTrackerKind::kFlachDominance;
      // Alg. 1 line 12 never defines convergence. The early iterations of
      // the k schedule move the multipliers more than the cells, so the
      // fixed_iters exits could stop the run before the leakage recovery
      // phase and before the best-solution rule has seen the lowest-leakage
      // feasible iterate.
      termination = TerminationKind::kPureCap;
      // Fig. 4 shows about 120 iterations, with the k switch at iteration 57.
      // Tests that need short runs pass -max_iterations after -preset.
      max_iterations = 120;
      break;
    case Preset::kSharmaSeq:
      // Sharma et al., ICCAD 2015, sequential part only. Settings:
      //  - The multiplier update of Fig. 2, lambda *= (1 - slack/T)^cexp,
      //    with all multipliers initialized to 1 (Sec. III-A).
      //  - The initial solution of Sec. III-A: the minimum-leakage reset and
      //    the reverse-topological max-cap repair (min_size_fixcap), then the
      //    forward-topological slew repair (slew_fix_pass).
      //  - Fast-OLR (Sec. V-A, Fig. 9) from iteration 5.
      //  - The Eq. 5 cost, including Flach's downstream sensitivity term.
      //  - Flach's local-slack check and the early exit (Sec. V-B), both
      //    active once timing is within 1% of the target.
      //
      // Differences from the paper:
      //  - When no cell clears a gate's slew, which the paper leaves open,
      //    the slew repair takes the cell closest to the limit (see init_mode
      //    below).
      //  - The cost omits Eq. 5's side arcs and side nets (see
      //    cost_global_phi below).
      //  - The best-solution rule is Flach's (see termination below).
      //
      // Not implemented:
      //  - Multi-threading with mutual exclusion edges and DAG netlist
      //    traversal (Sec. IV), the paper's main contribution. Hence "_seq".
      //  - The greedy post-pass (Sec. III-C) and Fast-GTR (Sec. V-C).
      lambda_update = LambdaUpdate::kSharmaCexp;
      lambda_seed = LambdaSeed::kConstant;
      lambda_init_value = 1.0f;
      // Every run in the paper uses exhaustive OLR for the first four
      // iterations and Fast-OLR from the fifth (Sec. VI);
      // fast_olr_start_iter defaults to 5. Fast-OLR also stabilizes the run:
      // Fig. 11 shows plain OLR destabilizing TNS after about 100
      // iterations, and this preset allows up to 160.
      move_set = MoveSet::kSharmaFastOlr;
      // Sec. III-A: after the reset and the two repairs, the paper keeps
      // max-cap and max-slew satisfied throughout by skipping any candidate
      // that would violate them, which only works from a clean start.
      //
      // min_size_fixcap is the reset and the capacitance pass, slew_fix_pass
      // the slew pass (see fixMaxSlewViolations). Both judge a cell with the
      // sweep's electrical model, not with full STA. When no cell clears a
      // gate's slew, the slew pass takes the one closest to the limit, if it
      // is closer than the gate's own cell. Gates still violating afterwards
      // are reported in RSZ-0445 and RSZ-0461.
      init_mode = InitMode::kMinSizeFixcap;
      slew_fix_pass = true;
      // output_drc_veto stays absolute: the paper treats a cell as invalid
      // if it causes a cap or slew violation, not only if it increases one.
      timing_scale = TimingScale::kUnit;
      timing_bias = 1.0f;
      // Eq. 5 prices the local arcs plus Flach's lambda-delay change on the
      // drain nets due to the output slew change, which is cost_global_phi.
      // cost_fanout_slew stays off: phi already prices the immediate sink
      // level (see RSZ-0429).
      //
      // Eq. 5 also includes the side arcs (arcs of gates that share a fanin)
      // and the side nets. No term here prices them, so the cost
      // underestimates a candidate's effect on its fanin's other sinks. The
      // flach and mangiras presets have the same gap.
      cost_global_phi = true;
      // Sharma applies Flach's local-slack check "as we recover power, after
      // the design timing is within 1% of the target delay" (Sec. V-A).
      // near_met_gate_frac = 0.01 (below) keeps the veto inactive until WNS
      // is within 1% of the target; after that, Flach's Eq. 14 tolerance
      // applies.
      downsize_guard = DownsizeGuard::kLocalSlackVeto;
      // best_tracker keeps the flach_dominance default. The paper's greedy
      // post-pass starts from the least-power LDP solution (Sec. III-C), so
      // remembering a best solution is close to the paper. Flach's rule is
      // not the same, though: its |TNS| < 10% of T condition can keep a
      // solution that still violates timing.
      //
      // Early exit (Sec. V-B): in sets of 5 iterations, stop after 2
      // consecutive sets in which neither the average power nor the best
      // power improved. The struct defaults hold these constants. The paper
      // applies the rule once timing is almost met. Without that condition,
      // a run that is still fixing timing raises leakage every sweep, looks
      // stagnant, and would stop after 15 iterations. near_met_gate_frac
      // gates both this rule and the veto above.
      termination = TerminationKind::kStagnationWindows;
      near_met_gate_frac = 0.01f;
      // The paper's runs took 35 to 160 iterations (Sec. V-B); the early exit
      // does the actual stopping.
      max_iterations = 160;
      break;
    case Preset::kReimann:
      // Reimann et al., ISPD 2016. Settings:
      //  - The Alg. 3 update, normalized by the WNS degradation and aimed at
      //    each arc's initial slack, and the estimation loop of Alg. 2 (lines
      //    3-9) as the multiplier seed.
      //  - The local slack check of Alg. 1 line 10 (local_slack_veto).
      //  - The Eq. 6 score to select the best solution (reimann_score).
      //  - Total power, leakage plus dynamic, as the power of Eq. 1.
      //
      // Differences from the paper:
      //  - The timing weight comes from auto_median instead of the paper's
      //    scale factors (see below).
      //
      // Not implemented:
      //  - The paper's estimation loop solves the subproblem with the cheap
      //    option ranking of Sec. 3.4 (one option per gate). Here each
      //    estimation iteration is a full sweep.
      //  - The area term of the objective (Eqs. 1 and 4) and the
      //    alpha/beta/theta scale factors (Eqs. 2 and 5).
      //  - Solution refinement (Sec. 3.5): timing recovery, power reduction,
      //    timing recovery.
      //
      // Notes for implementing the scale factors:
      //  - alpha/beta alone is not enough: Eq. 2 weights area alongside
      //    power, so theta/beta and an area term in the candidate cost are
      //    needed too.
      //  - As printed, Eq. 5 gives alpha < 0, because c_n (largest, lowest
      //    Vth) is faster than c_0. Use N_CREF / |D(c_n) - D(c_0)|.
      //  - N_CREF is common to all scale factors and only scales the whole
      //    objective, so only the ratios alpha/beta and theta/beta matter.
      //  - The paper does not define the reference output load, which
      //    inverter is C_REF, the Vth level used for theta, or whether c_n
      //    and c_0 are the extremes of the library or of C_REF's options.
      //    Reasonable choices: C_REF's input capacitance times a fanout of
      //    4; the median-drive inverter; the highest Vth; and C_REF's own
      //    options, the only reading under which N_CREF is well defined.
      lambda_update = LambdaUpdate::kReimannDwns;
      lambda_seed = LambdaSeed::kEstimationLoop;
      // The estimation loop starts from OpenROAD's delay-proportional seed,
      // so this preset uses auto_median. The paper's own initial multipliers
      // are unspecified, so no paper magnitude is lost.
      timing_scale = TimingScale::kAutoMedian;
      timing_bias = 12.0f;
      // Alg. 1 line 10 skips an option "if new slack < gamma * original
      // slack", which has the form of Flach's veto. The paper does not
      // define gamma or the slacks, so this uses Flach's local negative
      // slack over the driver and sink nets and Flach's Eq. 14 for gamma.
      downsize_guard = DownsizeGuard::kLocalSlackVeto;
      // Alg. 2 lines 1 and 14-19: store the input solution, replace it only
      // with a solution of higher Eq. 6 score, and restore the best at the
      // end.
      best_tracker = BestTrackerKind::kReimannScore;
      // The cost, the timing weight and the Eq. 6 score all read total power.
      power_objective = PowerObjective::kTotal;
      // Alg. 3 compares each arc's slack with its slack in the input
      // solution, so the update holds timing near the input and spends any
      // slack gained on power.
      reimann_setpoint = ReimannSetpoint::kSInit;
      // The paper's convergence metrics are not specified, so the run goes
      // to the iteration limit.
      termination = TerminationKind::kPureCap;
      // The paper's main-loop limit (Sec. 4); its runs converged in 12
      // iterations on average. Set explicitly so that a change of the struct
      // default does not change this preset.
      max_iterations = 20;
      break;
    case Preset::kMangiras:
      // Mangiras and Dimitrakopoulos, Technologies 2021, 9, 92 (journal
      // version of their MOCAST 2021 paper). The method is Flach et al.'s LR
      // sizer (Eq. 4 is Flach's multiplicative update) with a new multiplier
      // initialization (Eqs. 5-7) computed from the incoming sizes, so the
      // initial solution stays as_given. Settings:
      //  - Flach's update and the state-adaptive seed.
      //  - The restricted move set of Sec. 4.3.
      //  - Flach's local-slack veto and best-solution rule.
      //  - The stopping rule of Sec. 4.1.
      //
      // Eq. 6 multiplies the endpoint multipliers, and through the projection
      // the whole field, by the global power ratio (sum P / sum min P)^K.
      // For that factor to reach the cost, the seed's magnitude has to
      // survive: timing_scale = unit keeps tw independent of lambda, and
      // mu_policy = endpoint_lambda (set after the switch) makes the
      // projection scale with lambda. Under auto_median with
      // reseed_each_iter the factor would cancel exactly.
      // TestTimingScale.Eq6GlobalPowerRatioSurvivesToScoringUnderUnit checks
      // this. It holds only above lambda_floor.
      //
      // Not implemented:
      //  - Multi-corner optimization.
      //  - The timing recovery and power recovery steps that follow LR
      //    (Sec. 2): next-size-up on gates that affect many endpoints, then
      //    single-step downsizing verified by incremental timing.
      lambda_update = LambdaUpdate::kFlachSlackScaling;
      lambda_seed = LambdaSeed::kStateAdaptive;
      // Sec. 4.3: each gate may move only to its next bigger or next smaller
      // size, while Vth swaps stay unrestricted. The paper motivates this
      // at the end of the flow, to keep detailed routes, which is where
      // OpenROAD repairs timing. The paper's main results (Tables 1-2) use
      // the full library and Tables 3-4 use this restriction; the benefit
      // of the initialization holds in both. move_set = full_library gives
      // the Tables 1-2 setting.
      move_set = MoveSet::kMangirasSizeStep;
      timing_scale = TimingScale::kUnit;
      timing_bias = 1.0f;
      // The paper keeps Flach's sizer and changes only the multiplier
      // initialization (Sec. 4.1), so the guard is Flach's veto.
      downsize_guard = DownsizeGuard::kLocalSlackVeto;
      // best_tracker keeps the flach_dominance default, which is part of
      // Flach's sizer. Its |TNS| < 10% of T condition can keep a solution
      // that still violates timing, which does not fit a method whose input
      // is an almost-closed design.
      //
      // Sec. 4.1: stop when timing and leakage both improve by less than 1%
      // across two iterations. A window of 1 compares each iteration with
      // the one before and stops at the first stagnant comparison, which is
      // that rule. A window of 2 would instead average over disjoint
      // two-iteration blocks, first firing at iteration 4 and only on even
      // iterations.
      termination = TerminationKind::kStagnationWindows;
      stagnation_window = 1;
      stagnation_count = 1;
      stagnation_improve_frac = 0.01f;
      stagnation_require_tns = true;
      // The stopping rule is not gated on near-met timing
      // (near_met_gate_frac stays negative). The paper states no iteration
      // cap; its plots run to 20 iterations and the stopping rule usually
      // ends the run well before. Set explicitly so that a change of the
      // struct default does not change this preset.
      max_iterations = 20;
      break;
    case Preset::kLivramento:
      // Livramento et al., DATE 2013. Settings:
      //  - The update of Alg. 1 line 13, lambda *= (a_j + D_ji) / a_i. This
      //    is not Tennakoon's Fig. 13 rule: the local step sizes differ
      //    (lambda_ji / a_i here, Eq. 9, versus lambda_ji / (a_i - D_ji)),
      //    and the two agree only on a critical arc.
      //  - The endpoint update of Alg. 1 line 12 (mu_policy =
      //    endpoint_ratio, set after the switch).
      //  - The fanout-slew cost term of Alg. 2 lines 12-14.
      //  - The power-weight schedule of Alg. 1 line 9 (livramento_alpha).
      //  - Every gate starts at minimum leakage (Alg. 1 line 2).
      //  - Lagrangian relaxation of the max-cap constraints (Eq. 3): a
      //    multiplier beta per output pin, set (Alg. 1 line 5) and updated
      //    (line 15) with lambda, and priced on a candidate's own pins and its
      //    fanin drivers (Alg. 2 lines 8 and 11) and in FIX_VIOLATIONS
      //    (Alg. 3 line 7) (relax_max_cap).
      //  - FIX_VIOLATIONS (Alg. 3) after every subproblem solve: the gates
      //    over their max capacitance are resized, from outputs to inputs
      //    (cap_fix_pass).
      //  - A fixed budget of 60 iterations (Sec. V), returning the
      //    lowest-power iteration without setup, max capacitance or max slew
      //    violations (Alg. 1 line 20, livramento_feasible), or the final
      //    iteration if there is none.
      //
      // Differences from the paper:
      //  - The base that alpha scales, and alpha's initial value, are ours
      //    (see timing_scale below).
      //  - The initial beta is ours; the paper asks only for a positive
      //    value. At the first sweep, an excess equal to the median limit
      //    costs 1% of the median gate's power (see initialCapBeta).
      //  - Primary inputs have no beta, although the paper prices them: an
      //    input port has no Liberty limit. The paper does not mention
      //    registers; they get a beta as primary inputs, although this preset
      //    does not resize them (see lr/CapMultipliers.hh). Dont-touch and
      //    clock-network gates have none, and the nets they drive keep the
      //    hard max-cap check.
      //  - Only Liberty pin limits are priced; a limit set only with
      //    set_max_capacitance is not seen.
      //  - Max slew keeps its hard check (see relax_max_cap below).
      //  - When no width clears a violation, which the paper leaves open,
      //    FIX_VIOLATIONS takes the width closest to the limit (see
      //    selectCapFixOption). It tries the widths of the gate's swappable
      //    group, not every width (see fixMaxCapViolations).
      //  - When no iteration is free of violations, which the paper does not
      //    cover, the final iteration is returned. It comes after the last
      //    FIX_VIOLATIONS pass, so that pass's repairs are kept.
      //  - Alg. 2 accepts the lowest-cost option without a slack check, so no
      //    downsize guard would be closest. The preset keeps the depth
      //    budget, as in chen_partial.
      lambda_update = LambdaUpdate::kLivramentoRatio;
      cost_fanout_slew = true;
      // Alg. 1 line 9 rescales the power weight alpha from fresh arrival
      // times before every subproblem solve. alpha weights power in the
      // paper, so here it divides the timing weight. This preset keeps
      // OpenROAD's delay-proportional seed, so the base that alpha scales is
      // the auto_median weight with timing_bias = 12. The paper gives no
      // initial alpha (Alg. 1 line 6) and no initial lambda values;
      // livramento_alpha0 = 1 starts from the same balance as the other
      // auto_median presets. See TimingScale::kLivramentoAlpha.
      timing_scale = TimingScale::kLivramentoAlpha;
      timing_bias = 12.0f;
      best_tracker = BestTrackerKind::kLivramentoFeasible;
      max_iterations = 60;
      // The paper runs a fixed number of iterations with no convergence
      // test (Alg. 1 line 19).
      termination = TerminationKind::kPureCap;
      // Alg. 1 line 2. The paper repairs violations inside every iteration
      // (Alg. 2 line 23), not in an initial pass, so min_size_fixviol is not
      // needed here.
      init_mode = InitMode::kMinSize;
      cap_fix_pass = true;
      // The paper also defines a multiplier gamma for max slew but never uses
      // it, since controlling max capacitance kept slew within its limits.
      // Here max slew is not priced, and the sweep keeps its hard max-slew
      // check.
      relax_max_cap = true;
      break;
    case Preset::kChinnery:
      // Chinnery and Sharma, "Integrating LR Gate Sizing in an Industrial
      // Place-and-Route Flow", ISPD 2022. The preset covers the paper's LR
      // algorithm. Its engineering (a fast calibrated internal timer,
      // deterministic multi-threading, multi-corner/multi-mode analysis and
      // placement in the flow) is out of scope. Settings (all from Sec. 4
      // unless noted):
      //  - All multipliers start at 1. The paper updates them with the rule
      //    of Sharma et al., ICCAD 2017 [13], in the form of Sharma et al.,
      //    TCAD 2020 [7], Alg. 3: after each iteration every arc's multiplier
      //    is multiplied by (1 - s/T)^K, where s is the arc's slack and T the
      //    clock period. An arc is critical when s < 0 ([7], Sec. IV-C-2). K
      //    is 4 on critical and 1 on non-critical arcs in the timing phase,
      //    and 1 and 4 in the power phase.
      //  - In the power phase, candidates with more power than the current
      //    cell are skipped. Table 2 only names this filter ("skip higher
      //    power libcells in power recovery").
      //  - The projection distributes "the sum of the multipliers on the
      //    outgoing timing arcs among the incoming timing arcs, at every
      //    timing node, in reverse topological order", which is exactly
      //    proportional_reverse_topo.
      //  - The endpoint constraint of Eq. 2 (the in-arc multipliers of an
      //    endpoint sum to its multiplier) is mu_policy = endpoint_lambda,
      //    set after the switch.
      //  - The Eq. 4 local-arc cost, Flach's local-slack check, and a filter
      //    that skips candidates which increase a max-cap or max-slew
      //    violation.
      //  - The two-phase termination (timing, then power) with the paper's
      //    thresholds and its 80-iteration cap.
      //  - The sizer resizes the netlist it is given and never adds or
      //    removes cells (Sec. 3), so init_mode stays as_given. It does not
      //    resize registers (size_registers, off for every paper preset).
      //  - Total power, leakage plus dynamic, one of the paper's objectives
      //    (Tables 3-6).
      //
      // Differences from the paper:
      //  - Update rule: the base is 1 - s/T rather than [7]'s D/T (see
      //    sharmaArcSlackFactor), floored at a small positive value for an arc
      //    whose slack exceeds T.
      //  - Local slack: the paper adds the local slack degradation to the
      //    cost with a weight of 1,000,000. Here Flach's veto rejects any
      //    degradation. The two differ only when the cost gain exceeds a
      //    million times the degradation.
      //  - Electrical filter: see output_drc_veto below.
      //  - Normalization: the paper normalizes power and area by the average
      //    cell and delay by the average arc delay of the most critical path.
      //    timing_scale = unit uses medians over the design instead. Both
      //    make lambda = 1 mean "timing about equal to power on a typical
      //    gate".
      //  - Objective: the paper minimizes normalized power plus area
      //    (Eq. 1). Here the objective is total power, with area used only
      //    in place of leakage on a library that has no leakage data.
      //  - Timer calibration (Sec. 5.2-5.3): the paper calibrates its
      //    internal timer against the sign-off timer. Here OpenSTA is both,
      //    so there is nothing to calibrate.
      //
      // Not implemented:
      //  - Sibling arcs in the Eq. 4 cost (the flach, sharma and mangiras
      //    presets have the same gap). The sibling-arc skipping of Sec. 6
      //    only speeds up that term.
      //  - History-based adaptive libcell pruning (Sec. 6). The full library
      //    is evaluated instead, which is what the pruning approximates.
      //  - The area term of the objective (Tables 3-6). It needs its own
      //    cost term.
      //  - The fast levelized WNS/TNS/DRV optimization that the host flow
      //    runs after every LR call (Sec. 7).
      //  - Deterministic multi-threading with extra mutual exclusion edges
      //    (Sec. 5.1, Alg. 1).
      //  - Multi-corner and multi-mode analysis.
      lambda_update = LambdaUpdate::kSharmaArcSlack;
      // The exponents equal the struct defaults; they are set explicitly
      // because this is the only preset that uses them.
      arc_slack_k_timing_crit = 4.0f;
      arc_slack_k_timing_noncrit = 1.0f;
      arc_slack_k_power_crit = 1.0f;
      arc_slack_k_power_noncrit = 4.0f;
      power_phase_filter = true;
      lambda_seed = LambdaSeed::kConstant;
      lambda_init_value = 1.0f;
      // With lambda = 1 everywhere, unit makes the objective power plus the
      // timing of a median gate, the normalized form of Eq. 3 with the
      // paper's lambda.
      timing_scale = TimingScale::kUnit;
      timing_bias = 1.0f;
      // Eq. 4's local arcs are "arcs of fanin nets and fanins cells, arcs of
      // cell g, arcs of sibling cells ..., and arcs of fanout nets and fanout
      // cells". cost_upstream_load (on by default) prices the fanin side.
      // cost_fanout_slew prices the fanout side, as a frozen per-pin
      // sensitivity times the candidate's output-slew change: a
      // linearization of what the paper recomputes exactly on a local copy
      // of the timing graph.
      //
      // cost_global_phi stays off: Flach's phi covers the whole downstream
      // cone, while Eq. 4 stops one level out. With phi off, the fanout level
      // is priced once (see RSZ-0429).
      cost_fanout_slew = true;
      // Set explicitly so that a change of the struct default does not
      // restrict this preset's move set.
      move_set = MoveSet::kFullLibrary;
      // "We skip libcell alternatives that would increase max load
      // capacitance or max input slew violations" (Sec. 4). The rule is
      // against an increase, and this preset starts from the incoming
      // netlist with whatever violations it has, which is exactly where the
      // relative and absolute modes differ.
      //
      // Two differences remain. The paper checks the max input slew at the
      // sink; the slew check here reads the driver's output-pin limit with
      // the linear slew estimate, as for every preset. For designs whose
      // constraints cannot be met, the paper only asks not to worsen the
      // outstanding violations (Sec. 5) without saying against what level.
      // Here the reference is the current cell's violation under the live
      // load, re-read every sweep, not a level frozen at flow entry. That is
      // tighter on a pin whose load is stable and looser on one whose load
      // grows.
      output_drc_veto = OutputDrcVeto::kRelative;
      // Following Flach et al., the paper computes the local slack change of
      // every alternative and prevents degradation with a large weight
      // (Sec. 4), so the guard is Flach's veto. gamma_local_slack = 0 makes
      // gamma = 1 from the first iteration, which forbids any degradation;
      // the paper has no Eq. 14 tolerance schedule.
      //
      // near_met_gate_frac stays at its ungated default: the paper checks
      // local slack for every alternative, with no timing precondition like
      // Sharma's 1% rule.
      downsize_guard = DownsizeGuard::kLocalSlackVeto;
      gamma_local_slack = 0.0f;
      // The timing phase ends when TNS is within 10% of the clock period,
      // WNS within 1% of it, or TNS improves by less than 10% over the last
      // 3 iterations. The power phase ends when power improves by less than
      // 1% over the last 3 iterations (the paper loosened this from 0.1% to
      // save runtime, Sec. 6) or when TNS gets worse than at the end of the
      // timing phase. The run is also capped at 72 hours. These values equal
      // the struct defaults; they are set explicitly because this is the
      // only preset that uses them.
      termination = TerminationKind::kThresholdBattery;
      term_tns_target_frac = 0.10f;
      term_wns_target_frac = 0.01f;
      term_tns_improve_frac = 0.10f;
      term_power_improve_frac = 0.01f;
      term_improve_window = 3;
      term_wall_limit_s = 259200.0f;
      // The paper commits the final assignment; it has no best-so-far
      // restore. A run that ends because TNS degraded in the power phase
      // therefore keeps the degraded result (RSZ-0451 reports the reason).
      // In the paper, the host flow can roll back changes if WNS degrades
      // (Fig. 4) and runs a fast optimization after every LR call; neither
      // exists around this sizer.
      best_tracker = BestTrackerKind::kNone;
      // The paper's iteration cap (Sec. 4). Its runs averaged 23 iterations,
      // 13 in the timing phase and 10 in the power phase.
      max_iterations = 80;
      // The cost, the timing weight and the power-phase exit all read total
      // power.
      power_objective = PowerObjective::kTotal;
      break;
  }

  // Every paper's subproblem visits the gates in topological order and
  // commits each gate before evaluating the next (e.g. Flach Alg. 3,
  // Livramento Alg. 2, Reimann Alg. 1), so the paper presets use the
  // sequential Gauss-Seidel engine. This includes Chinnery: its sizer is
  // multi-threaded, but it resizes in forward topological order, and its
  // mutual exclusion edges exist to reproduce one deterministic sequential
  // order. rsz_baseline keeps the parallel Jacobi engine. gs_refresh and
  // traversal are the struct defaults, set here to make the choice explicit.
  if (p != Preset::kRszBaseline) {
    sweep_engine = SweepEngineKind::kGaussSeidelTopo;
    gs_refresh = GsRefresh::kLocal;
    traversal = Traversal::kForwardTopo;
    // mu is derived from the endpoint's own in-arc lambda sum rather than
    // from endpoint slack. This matches the papers, where arcs into an
    // endpoint carry ordinary multipliers that anchor the reverse-topological
    // distribution (Flach Alg. 2, Livramento Alg. 1 line 12, Mangiras
    // Eq. 6). It is also what lets the magnitude of a uniform lambda seed
    // survive the projection: the slack-derived policies anchor each
    // endpoint to a value that carries no information about lambda, so they
    // cancel a uniform seed constant at the first projection, whatever the
    // update rule does. rsz_baseline keeps reseed_each_iter. Chen, Tennakoon
    // and Livramento override this below.
    mu_policy = MuPolicy::kEndpointLambda;
    // The default configuration accepts an upsize only if it lowers the cost
    // by more than 2%, which filters out moves caused by noise. None of the
    // papers uses this rule: each subproblem commits the lowest-cost
    // candidate (e.g. Chen SOLVE_LRS, Sharma Sec. III-B, Mangiras Sec. 2),
    // so the paper presets take the lowest-cost candidate directly.
    // Candidate rejection rules that a paper does specify are handled by
    // downsize_guard.
    upsize_hysteresis = 0.0f;
    // None of the papers resizes registers.
    size_registers = false;
  }

  // Papers with an explicit endpoint multiplier update get it instead of
  // endpoint_lambda, under which their endpoints would have no dual-ascent
  // step. Chen's is additive (SOLVE_LDP step 3, i = 0). Tennakoon's Fig. 13
  // first case and Livramento's Alg. 1 line 12 are the same multiplicative
  // ratio. Both options still derive the initial mu from the endpoint's
  // in-arc lambda sum. This must come after the block above, which sets
  // endpoint_lambda for every paper preset.
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

// True for the mu policies that derive mu from the endpoint's own in-arc
// lambda sum. The projection anchored to them scales with lambda, so the
// update rule's effect on the multiplier field survives it. endpoint_ratio
// and endpoint_additive add their paper's endpoint update on top, which is
// still derived from lambda.
//
// The other policies anchor to seedBaselineMu's slack field
// ((margin - slack)^p, normalized to a maximum of 1), which carries no
// information about lambda. The projection then rewrites the endpoint
// boundary from slack every iteration, so two runs whose update rules
// produced very different lambda fields are redistributed onto the same one.
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

// Only threshold_battery has a power phase. Under any other termination the
// run stays in the timing phase, so sharma_arc_slack keeps its timing-phase
// exponents and the power-phase filter never applies.
void warnIfThePhaseNeverChanges(const GlobalSizingConfig& config,
                                utl::Logger* logger)
{
  using TerminationKind = GlobalSizingConfig::TerminationKind;
  if (config.termination == TerminationKind::kThresholdBattery) {
    return;
  }
  if (config.lambda_update
      == GlobalSizingConfig::LambdaUpdate::kSharmaArcSlack) {
    logger->warn(RSZ,
                 439,
                 "GLOBAL_SIZING: lambda_update=sharma_arc_slack switches its "
                 "exponents in the power phase of "
                 "termination=threshold_battery; under termination={} it "
                 "uses the timing-phase exponents throughout.",
                 toString(config.termination));
  }
  if (config.power_phase_filter) {
    logger->warn(RSZ,
                 440,
                 "GLOBAL_SIZING: power_phase_filter applies only in the "
                 "power phase of termination=threshold_battery; under "
                 "termination={} it has no effect.",
                 toString(config.termination));
  }
}

// The slew fix pass resizes gates before the multiplier seed, also under
// init_mode = as_given. The seeds that assume the as-given sizes see it as
// they see an init pass; RSZ-0421 and RSZ-0422 cover the other init modes.
bool checkSeedAfterSlewFixPass(const GlobalSizingConfig& config,
                               utl::Logger* logger)
{
  using LambdaSeed = GlobalSizingConfig::LambdaSeed;
  if (!config.slew_fix_pass
      || config.init_mode != GlobalSizingConfig::InitMode::kAsGiven) {
    return true;
  }
  if (config.lambda_seed == LambdaSeed::kStateAdaptive) {
    logger->error(RSZ,
                  462,
                  "GLOBAL_SIZING: lambda_seed=state_adaptive cannot be used "
                  "with slew_fix_pass: the seed infers past criticality from "
                  "the current sizes, which the slew fix pass changes before "
                  "the seed reads them.");
    return false;
  }
  if (config.lambda_seed == LambdaSeed::kEstimationLoop) {
    logger->warn(RSZ,
                 463,
                 "GLOBAL_SIZING: lambda_seed=estimation_loop is intended for "
                 "the as-given initial solution; slew_fix_pass makes the "
                 "estimated multipliers target the repaired state.");
  }
  return true;
}

// Every LR iteration needs at least one sweep. The restart discards the
// previous iteration's sizes, so with one sweep per iteration each iteration
// has a single greedy pass to rebuild them from the initial cells.
bool checkInnerLoop(const GlobalSizingConfig& config, utl::Logger* logger)
{
  if (config.max_inner_sweeps < 1) {
    logger->error(RSZ,
                  454,
                  "GLOBAL_SIZING: max_inner_sweeps={} must be at least 1.",
                  config.max_inner_sweeps);
    return false;
  }
  if (config.restart_each_iteration && config.max_inner_sweeps == 1) {
    logger->warn(RSZ,
                 457,
                 "GLOBAL_SIZING: restart_each_iteration with "
                 "max_inner_sweeps=1 sets every gate back to its initial "
                 "cell at each iteration and leaves one sweep to resize it.");
  }
  return true;
}

// relax_max_cap credits a pin below its max capacitance, so a gate's cost
// can be negative. upsize_hysteresis asks an upsize to lower the cost by a
// fraction of it, which a negative cost always passes.
void warnIfHysteresisMeetsSignedCost(const GlobalSizingConfig& config,
                                     utl::Logger* logger)
{
  if (config.relax_max_cap && config.upsize_hysteresis > 0.0f) {
    logger->warn(RSZ,
                 465,
                 "GLOBAL_SIZING: relax_max_cap can make a gate's cost "
                 "negative, and upsize_hysteresis={} then accepts every "
                 "upsize that lowers it.",
                 config.upsize_hysteresis);
  }
}

}  // namespace

void GlobalSizingConfig::resolveLambdaMuPairing(utl::Logger* logger)
{
  mu_auto_paired = false;
  // rsz_baseline's own update rule is exempt (see the header): its lambda
  // magnitude has no meaning to preserve, and changing its mu policy would
  // change the baseline.
  if (lambda_update == LambdaUpdate::kNormSubgradient) {
    return;
  }
  // Nothing to do if the mu policy already reads lambda. Every paper preset
  // sets such a policy, so this rule never changes a preset.
  if (muPolicyReadsLambda(mu_policy)) {
    return;
  }
  if (mu_policy_explicit) {
    // An explicit policy is honored. Under this combination the projection
    // discards what the update rule did to the lambda magnitude, so say so;
    // info rather than a warning, because the user asked for it.
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
  // Cross-option consistency checks. A conflict that makes the run
  // meaningless is an error; a combination that is legal but probably
  // unintended is a warning.
  //
  // The sweep count, and the restart that relies on it.
  if (!checkInnerLoop(*this, logger)) {
    return false;
  }
  // init_seed is read only by init_mode = random. Under any other mode, runs
  // with different seeds are identical even though the config echo
  // (RSZ-0417) prints different seeds, so a seed sweep would silently show
  // zero run-to-run variation. Warn only: the value itself is harmless.
  if (init_seed != 0 && init_mode != InitMode::kRandom) {
    logger->warn(RSZ,
                 442,
                 "GLOBAL_SIZING: init_seed={} is inert under init_mode={} - "
                 "only init_mode=random draws from it, so every seed gives the "
                 "same initial solution.",
                 init_seed,
                 toString(init_mode));
  }
  // Same for fast_olr_start_iter, which only move_set = sharma_fast_olr
  // reads.
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
  // Same for the six constants that only termination = threshold_battery
  // reads, with one warning for the group.
  //
  // Limitation shared by these three checks: a value equal to the struct
  // default is treated as unset. A preset that sets a constant to its
  // default value (chinnery_partial sets all six battery constants this
  // way) is therefore not warned about when its termination is overridden.
  // Fixing that needs per-option tracking like mu_policy_explicit.
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
  warnIfThePhaseNeverChanges(*this, logger);
  warnIfHysteresisMeetsSignedCost(*this, logger);
  // The state-adaptive seed (Mangiras Eqs. 5-6) infers past criticality from
  // the current sizes (each cell's leakage relative to the minimum). Any
  // init pass would replace those sizes before the seed reads them.
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
  // Mangiras Eq. 6 sets each endpoint multiplier as a boundary condition for
  // the reverse-topological distribution (Eq. 7). Re-seeding mu from
  // endpoint slack every iteration keeps that boundary for the first
  // projection only. Allowed, but it is not the paper's method.
  if (lambda_seed == LambdaSeed::kStateAdaptive
      && mu_policy == MuPolicy::kReseedEachIter) {
    logger->warn(RSZ,
                 436,
                 "GLOBAL_SIZING: lambda_seed=state_adaptive with "
                 "mu_policy=reseed_each_iter honors Mangiras Eq. 6's endpoint "
                 "boundary at iteration 0 and overwrites it at iteration 1; "
                 "mu_policy=endpoint_lambda preserves it.");
  }
  // estimation_loop restores the initial solution after each dry-run
  // iteration, so an init pass is not erased, but the estimate then targets
  // the initialized state rather than the incoming one, which weakens the
  // warm start. Warn only.
  if (lambda_seed == LambdaSeed::kEstimationLoop
      && init_mode != InitMode::kAsGiven) {
    logger->warn(RSZ,
                 422,
                 "GLOBAL_SIZING: lambda_seed=estimation_loop is intended for "
                 "the as-given initial solution; init_mode={} makes the "
                 "estimated multipliers target the initialized state.",
                 toString(init_mode));
  }
  if (!checkSeedAfterSlewFixPass(*this, logger)) {
    return false;
  }
  // cost_global_phi and cost_delta_delay are two different estimators of the
  // same downstream cost component; enabling both prices it twice with
  // incompatible models. cost_fanout_slew prices only the immediate fanout
  // and combines with either.
  if (cost_global_phi && cost_delta_delay) {
    logger->error(RSZ,
                  424,
                  "GLOBAL_SIZING: cost_global_phi and cost_delta_delay are "
                  "mutually exclusive (SYNTHESIS §5.6: they are two different "
                  "global-effect estimators for the same cost component). "
                  "Enable at most one.");
    return false;
  }
  // Both terms include the immediate sink level's λ_sink·(δd/δslew): φ's
  // Eq. 11 recurrence starts there, and the fanout-slew term is exactly that
  // level. Allowed, since the fanout term is exact where φ's dominant-arc
  // simplification is not, but flagged. No preset combines them.
  if (cost_global_phi && cost_fanout_slew) {
    logger->warn(RSZ,
                 429,
                 "GLOBAL_SIZING: cost_global_phi and cost_fanout_slew both "
                 "price the immediate sink level's slew sensitivity, so "
                 "enabling both double-prices it.");
  }
  // The Jacobi engine evaluates one sweep-start snapshot in a fixed order
  // with no per-commit refresh, so traversal and gs_refresh only affect the
  // Gauss-Seidel engine. A non-default value under Jacobi is ignored.
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
  // Guard/engine combinations. Both are allowed, but each pairs a guard with
  // the engine it was not designed for, so each is flagged.
  //
  // Under Jacobi the veto's local slacks come from the sweep-start snapshot
  // (there is no per-commit refresh to read), so every gate is checked
  // against the same stale timing, and downsizes accepted in the same sweep
  // can still add up on a shared path, which is exactly the overshoot the
  // depth budget prevents.
  if (sweep_engine == SweepEngineKind::kJacobiSnapshot
      && downsize_guard == DownsizeGuard::kLocalSlackVeto) {
    logger->warn(RSZ,
                 430,
                 "GLOBAL_SIZING: downsize_guard=local_slack_veto under the "
                 "jacobi_snapshot engine can only test candidates against the "
                 "frozen sweep-start required times, and does not bound the "
                 "sum of simultaneous downsizes on a path (SYNTHESIS §3.2-F).");
  }
  // Under Gauss-Seidel the budget is needlessly conservative: commits are
  // sequential, so the local-slack veto can check each gate's actual slack
  // after the previous commits instead of a depth-normalized share of a
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
