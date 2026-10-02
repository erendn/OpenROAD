// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

namespace utl {
class Logger;
}  // namespace utl

namespace rsz {

// Tunables for the Lagrangian-Relaxation global sizing driver. Configured via
// the `set_global_sizing_config` Tcl command (read out of dbProperties by
// Resizer::initBlock()) and consumed by GlobalSizingPolicy.
//
// The LR engine is split into interchangeable strategies (src/rsz/src/lr/).
// Each enum below selects the strategy for one part of the algorithm. A named
// Preset sets all of them to the choices of one published LR sizer
// (applyPreset), options set individually afterwards override the preset, and
// validate() checks the combination. The field defaults match the
// rsz_baseline preset except for best_tracker.
struct GlobalSizingConfig
{
  // Initial solution: before LR starts, optionally replace every editable
  // instance with another member of its swappable-equivalence group. The
  // group is ranked by Resizer::cellLeakage(), with drive resistance breaking
  // ties. There is no separate size or Vt axis, so "min", "max" and "average"
  // below are positions in that leakage ranking.
  enum class InitMode
  {
    // Keep the incoming netlist (default).
    kAsGiven = 0,
    // Lowest-ranked member of each group.
    kMinSize = 1,
    // Highest-ranked member of each group.
    kMaxSize = 2,
    // kMinSize followed by an electrical repair pass. The pass visits the
    // gates from outputs to inputs and upsizes each gate whose own output pins
    // violate max-cap or max-slew to the lowest-leakage member that clears
    // them. Deterministic.
    //
    // It is a repair pass, not an optimizer. A gate that no member of its
    // group can fix stays at minimum and is counted in RSZ-0445. The pass uses
    // the same electrical model as the sweep's candidate filter (Liberty cap
    // limits and a drive-resistance slew estimate) rather than full STA, so a
    // max_capacitance set only in SDC is not seen.
    kMinSizeFixviol = 3,
    // Uniform per-instance draw over the whole group (the current cell
    // included), seeded by init_seed.
    kRandom = 4,
    // Lower median of the ranked group, index floor((n-1)/2). Deterministic.
    kAverage = 5,
    // kMinSizeFixviol with a repair pass that checks max-cap only. Sharma et
    // al. (ICCAD 2015, Sec. III-A) repair capacitance from outputs to inputs
    // and then slew from inputs to outputs; slew_fix_pass is that second
    // pass.
    kMinSizeFixcap = 6,
  };
  InitMode init_mode = InitMode::kAsGiven;

  // Seed for init_mode = random. Default 0. It is independent of the global
  // placement seed, so placement and initial sizing can be varied separately.
  // Each instance's draw is keyed by a hash of (init_seed, instance name), so
  // a seed gives the same initial netlist regardless of visit order or thread
  // count.
  int init_seed = 0;

  // Initial Lagrange multipliers (lambda on timing arcs, mu on endpoints).
  enum class LambdaSeed
  {
    // Lambda proportional to arc delay; mu from endpoint criticality, biased
    // toward the worst endpoints (default).
    kDelayPropCritMu = 0,
    // Every data arc starts at lambda_init_value.
    kConstant = 1,
    // State-adaptive seed of Mangiras & Dimitrakopoulos (Technologies 2021,
    // Eqs. 5-7): per-arc lambda = (timing criticality ratio x leakage ratio
    // to the smallest cell) ^ lambda_seed_exponent, then projected onto the
    // flow constraints (Eq. 7 is that projection). It infers past criticality
    // from the current sizes, so it requires init_mode = as_given.
    kStateAdaptive = 2,
    // Estimation loop of Reimann et al. (ISPD 2016, Alg. 2, first loop):
    // start from the default seed, then run est_loop_iters trial iterations
    // (evaluate, update lambda, roll back the sizes) to warm-start the
    // multipliers before the main loop.
    kEstimationLoop = 3,
  };
  LambdaSeed lambda_seed = LambdaSeed::kDelayPropCritMu;

  // Lambda update rule applied after each sweep.
  enum class LambdaUpdate
  {
    // Multiplicative normalized subgradient step with step size beta, halved
    // when a sweep is rejected (default).
    kNormSubgradient = 0,
    // Flach et al., TCAD 2014, Alg. 2: asymmetric multiplicative slack
    // scaling, lambda *= (1 + |slack|/T)^(1/k) on violating arcs and
    // lambda *= (1 + slack/T)^(-k) on the others, with k following the
    // flach_k_* schedule.
    kFlachSlackScaling = 1,
    // Chen et al., ICCAD 1998: additive subgradient
    // lambda += rho_k * (arc violation), rho_k = lambda_update_c / k.
    kChenSubgradient = 2,
    // Tennakoon & Sechen, ICCAD 2002, Fig. 13: multiplicative local arrival
    // ratio lambda *= a_from / (a_to - d). Has no constants.
    kTennakoonRatio = 3,
    // Sharma et al., ICCAD 2015, Fig. 2 line 10: lambda *= (1 - slack_j/T)^cexp
    // with slack_j the slack of the arc's sink node and cexp following the
    // sharma_r / sharma_k schedule.
    kSharmaCexp = 4,
    // Reimann et al., ISPD 2016, Alg. 3: asymmetric multiplicative update
    // normalized by the change in WNS and referenced to reimann_setpoint, with
    // separate increase and decrease step schedules (reimann_*).
    kReimannDwns = 5,
    // Livramento et al., DATE 2013, Alg. 1 line 13: local multiplicative
    // update lambda *= (a_from + d) / a_to. It differs from kTennakoonRatio in
    // the denominator; the two agree only on critical arcs
    // (a_to = a_from + d). Line 14 (arcs from primary inputs) is the
    // a_from = 0 case of the same formula.
    kLivramentoRatio = 6,
    // Sharma et al., TCAD 2020, Alg. 3: lambda *= (1 - s/T)^K with s the
    // arc's own slack. K is arc_slack_k_timing_* in the timing phase of
    // termination = threshold_battery and arc_slack_k_power_* in its power
    // phase, the critical value on arcs with s < 0.
    kSharmaArcSlack = 7,
  };
  LambdaUpdate lambda_update = LambdaUpdate::kNormSubgradient;

  // How the endpoint multipliers mu are maintained. mu is the boundary
  // condition of the flow projection (FlowProjection.cc) and is also read by
  // the duality-gap report (Termination.cc).
  //
  // The slack-derived policies (reseed_each_iter, seed_once,
  // update_as_lambda) anchor the projection to a mu computed from endpoint
  // slacks, which carries no information about the magnitude of lambda. The
  // projection therefore rescales away any uniform change the lambda update
  // makes, and the constant seed value has no effect. The endpoint_* policies
  // derive mu from lambda itself, so the magnitude survives the projection.
  // See resolveLambdaMuPairing().
  enum class MuPolicy
  {
    // Re-seed mu from the endpoint slacks every iteration,
    // mu_k ~ max(0, margin - slack_k)^mu_exponent, normalized (default).
    kReseedEachIter = 0,
    // Seed mu once at iteration 0 and keep it.
    kSeedOnce = 1,
    // Scale mu every iteration by a factor derived from endpoint slack, like
    // lambda. A simplified relative of the papers' endpoint update, which is
    // kEndpointRatio.
    kUpdateAsLambda = 2,
    // mu is the sum of the endpoint's incoming lambdas, so arcs into an
    // endpoint are ordinary arcs updated by the lambda rule. This is how
    // Flach et al., Livramento et al. (Alg. 1 line 12) and Mangiras &
    // Dimitrakopoulos (Eq. 6) treat endpoints.
    kEndpointLambda = 3,
    // Multiplicative endpoint update of Tennakoon & Sechen (Fig. 13, first
    // branch) and Livramento et al. (Alg. 1 line 12), which is the same
    // formula in both papers. mu starts as the endpoint's incoming lambda sum,
    // then mu_k *= a_k / required_k every iteration: it grows on a violating
    // endpoint and settles as a_k approaches required_k.
    kEndpointRatio = 4,
    // Additive endpoint update of Chen et al. (SOLVE_LDP step 3, i = 0). mu
    // starts as the endpoint's incoming lambda sum, then
    // mu_k = max(0, mu_k + rho_k * (-slack_k / T)) with the rho_k schedule and
    // lambda_update_c constant of kChenSubgradient.
    kEndpointAdditive = 5,
  };
  MuPolicy mu_policy = MuPolicy::kReseedEachIter;
  // True when the user set -mu_policy explicitly. Set by the dbProperty
  // reader, not by applyPreset. resolveLambdaMuPairing() never changes an
  // explicit policy.
  bool mu_policy_explicit = false;
  // True when resolveLambdaMuPairing() changed mu_policy. Reported as
  // `mu_autopair` in the RSZ-0417 configuration echo.
  bool mu_auto_paired = false;

  // Projection of lambda onto the flow-conservation (KKT) constraints.
  enum class KktProjection
  {
    // At every node, in reverse topological order, distribute the outgoing
    // lambda sum over the incoming arcs in proportion to their current values
    // (default and only option).
    kProportionalReverseTopo = 0,
  };
  KktProjection kkt_projection = KktProjection::kProportionalReverseTopo;

  // How one LR sweep over the gates is executed.
  enum class SweepEngineKind
  {
    // Parallel Jacobi sweep: every gate is evaluated against one snapshot
    // taken at the start of the sweep, so a gate does not see changes made to
    // its fanin in the same sweep (default).
    kJacobiSnapshot = 0,
    // Sequential Gauss-Seidel sweep in `traversal` order: each gate is
    // evaluated on a fresh snapshot and its choice is committed before the
    // next gate, so later gates see earlier commits. This is the LRS of the
    // papers (e.g. Flach et al., Alg. 3). gs_refresh sets how much timing is
    // updated after each commit. Runs single-threaded because the snapshots
    // read live STA, which also makes it deterministic.
    kGaussSeidelTopo = 1,
  };
  SweepEngineKind sweep_engine = SweepEngineKind::kJacobiSnapshot;

  // Timing refresh after each commit of the gauss_seidel_topo engine. Ignored
  // by jacobi_snapshot, which has no per-commit refresh.
  enum class GsRefresh
  {
    // Local timing update (Flach et al., Sec. VII-A): re-estimate the
    // parasitics of the nets just changed, so the next snapshot sees fresh
    // loads and forward delays. Required times are not propagated, so slacks
    // keep their sweep-start values during the sweep, as in the paper
    // (default).
    kLocal = 0,
    // Incremental STA after each commit (parasitics, then required times), so
    // the next snapshot sees consistent arrival and required times. Slower.
    // The per-gate snapshot itself reads no required times, so the two modes
    // make different choices only with downsize_guard = local_slack_veto.
    kIncremental = 1,
  };
  GsRefresh gs_refresh = GsRefresh::kLocal;

  // Gate order of the gauss_seidel_topo engine. jacobi_snapshot ignores it
  // (validate() warns if it is changed). Ties break on a stable instance id,
  // so every order is deterministic.
  enum class Traversal
  {
    // Increasing level: a gate's fanin drivers commit before the gate
    // (Flach et al., Alg. 3) (default).
    kForwardTopo = 0,
    // Decreasing level (outputs to inputs).
    kReverseTopo = 1,
    // Most critical gate first (ascending sweep-start slack).
    kCriticalitySorted = 2,
  };
  Traversal traversal = Traversal::kForwardTopo;

  // Acceptance test a candidate cell must pass before the LR cost decides. The
  // max-cap / max-slew filter (output_drc_veto) is separate and applies under
  // every guard.
  enum class DownsizeGuard
  {
    // Depth-normalized slack budget per gate, fixed at sweep start (default).
    // A candidate with lower leakage than the current cell may add at most
    // budget_safety_factor * budget delay on each output pin; upsizes are not
    // limited. The budgets along any path sum to at most the path slack, which
    // makes simultaneous (Jacobi) downsizes safe. This is an OpenROAD
    // heuristic, not taken from a paper.
    kDepthBudget = 0,
    // Local negative slack veto of Flach et al. (Alg. 4 lines 1 and 12-14,
    // Eq. 14). A gate's local negative slack is the sum of the negative slacks
    // of its fanin nets and its output net. A candidate is rejected if it
    // makes that sum worse than gamma times the original, where gamma >= 1
    // decays to 1 as the design converges (see gamma_local_slack). The test
    // applies to every candidate, upsizes included: an upsize whose larger
    // input capacitance slows its driver can be rejected.
    //
    // The slacks come from the sweep-start timing under jacobi_snapshot and
    // gs_local (stale during the sweep, as in the paper) and are updated after
    // each commit under gs_incremental. They are not queried live under
    // gs_local because an OpenSTA slack query would trigger a full timing
    // update for every gate.
    kLocalSlackVeto = 1,
    // No guard; the LR cost alone decides (the max-cap / max-slew filter still
    // applies).
    kNone = 2,
  };
  DownsizeGuard downsize_guard = DownsizeGuard::kDepthBudget;

  // Which members of a gate's swappable group the per-gate subproblem
  // evaluates. The restricted sets index the group by (drive rank, Vt flavor)
  // (lr/SizeVthGrid.hh). The rank is the leakage ranking within a flavor, so
  // "next bigger size" means the next member up that ranking.
  enum class MoveSet
  {
    // Every member, every iteration (default).
    kFullLibrary = 0,
    // Fast-OLR of Sharma et al. (ICCAD 2015, Sec. V-A, Fig. 9). From
    // fast_olr_start_iter on, the exhaustive scan is replaced by a hill descent
    // along the size ranking of the current Vt flavor and its two neighboring
    // flavors, stopping in each direction at the first step that does not
    // improve the cost. Earlier iterations scan every member, as in the paper.
    // Besides saving runtime, local moves avoid the late-run TNS instability
    // that the paper shows for the exhaustive scan (Fig. 11). The paper's
    // local slack check (Fig. 9 lines 20-21) is downsize_guard =
    // local_slack_veto, not part of this option.
    kSharmaFastOlr = 1,
    // Restricted move set of Mangiras & Dimitrakopoulos (Technologies 2021,
    // Sec. 4.3): a gate may move only one size step up or down, while Vt swaps
    // stay unrestricted. Applies from the first iteration.
    kMangirasSizeStep = 2,
  };
  MoveSet move_set = MoveSet::kFullLibrary;

  // Iteration at which move_set = sharma_fast_olr switches to the hill
  // descent. It is 1-based and counts only iterations that update lambda; 0
  // also includes the initial sweep before the first update. Default 5, as in
  // Sharma et al. (the first four iterations are exhaustive). Ignored by the
  // other move sets.
  int fast_olr_start_iter = 5;

  // What the sweep's max-cap / max-slew filter does on an output pin that the
  // current cell already violates. On the input side the filter always rejects
  // only a candidate that worsens an existing max-cap violation on a fanin net.
  // In both modes the limits are hard filters, except max capacitance on the
  // pins that relax_max_cap prices into the cost instead.
  enum class OutputDrcVeto
  {
    // Reject any candidate that exceeds its own output-pin limits (default).
    // If no member of the group can clear a pin, every candidate is rejected
    // and the gate keeps the cell it had when the loop started. With
    // init_mode = min_size_fixviol, such a gate stays at minimum size.
    kAbsolute = 0,
    // Reject only a candidate that makes an existing violation on the pin
    // worse. An equal violation is accepted, and a clean pin gets the
    // absolute check. This is the rule of Flach et al. (Alg. 4 line 6, "if
    // load violation has increased") and Chinnery & Sharma (ISPD 2022,
    // Sec. 4). See outputLimitAdmits() in lr/ElectricalModel.hh.
    //
    // Two limitations:
    //  * The reference is the current cell's violation at the current load,
    //    re-read every sweep, so a violation can grow over the run when the
    //    load of the gate grows.
    //  * It compares violation amounts, not transitions. Each side is measured
    //    against its own pin's limit, so when max_transition is not monotone
    //    within a group (sky130 buf_8 declares 7.65 ns, the stronger buf_16
    //    5.01 ns) a weaker candidate can show the smaller violation while
    //    driving the larger transition.
    kRelative = 1,
  };
  OutputDrcVeto output_drc_veto = OutputDrcVeto::kAbsolute;

  // Optional terms of the per-gate Lagrangian cost (LRSubproblem). The gate's
  // own arc term, sum(lambda * delay), is always included.
  //
  // Price the change in the upstream drivers' lambda-weighted arc delays
  // caused by the candidate's input load (analogous to Chen et al.,
  // Lemma 2). Default on.
  bool cost_upstream_load = true;
  // Price the change in the immediate fanout arc delays caused by the
  // candidate's output slew (Livramento et al., Alg. 2 lines 12-14). The
  // per-output-pin sensitivity sum(lambda * d(delay)/d(slew)) is precomputed
  // on the main thread and scaled by the candidate's output slew change.
  // Default off.
  bool cost_fanout_slew = false;
  // Price the candidate's output slew change against the accumulated slew
  // sensitivity phi of its whole downstream cone (Flach et al., Eqs. 5 and
  // 11). phi is computed once per iteration in a reverse topological pass.
  // Default off. Cannot be combined with cost_delta_delay.
  bool cost_global_phi = false;
  // Price arc delays relative to each arc's delay in the previous iteration
  // (LrState::prev_delay) instead of absolutely. Default off; no preset uses
  // it. Cannot be combined with cost_global_phi.
  //
  // This is not the delta-delay model of Ozdal et al. (ICCAD 2011,
  // Eqs. 7-10). There the reference is the candidate's own delay at the
  // previous-iteration fanout load, so it differs per candidate, and the
  // model feeds a tree dynamic program. Here the reference is one constant
  // per arc, so it cancels when candidates of the same gate are compared.
  bool cost_delta_delay = false;

  // How sum(lambda * delay) prices the timing arcs into a driver pin, in the
  // gate's own arc term, the upstream-load term and the timing weight's
  // anchor. The output-slew terms (cost_fanout_slew, cost_global_phi) are the
  // same under both.
  enum class TimingCost
  {
    // The sum of lambda over the pin's arcs times the worst delay among them
    // (default). One delay lookup per pin, but every arc other than the worst
    // is priced at the worst arc's delay.
    kWorstArc = 0,
    // Each arc's lambda times that arc's own delay, as the papers price it
    // (Flach et al., Eq. 5; Livramento et al., Alg. 2 line 10; Reimann et
    // al., Alg. 1 line 13). One delay lookup per arc.
    kPerArc = 1
  };
  TimingCost timing_cost = TimingCost::kWorstArc;

  // How the weight tw in the per-gate objective
  //   power + tw * sum(lambda * delay)
  // is set, where power is the cell's leakage, or its total power under
  // power_objective = kTotal. This decides whether the magnitude of lambda
  // matters. In every option the medians below are computed once, before the
  // loop, from the iteration-0 multipliers, so the objective does not drift
  // during the run. Only livramento_alpha's alpha factor changes from
  // iteration to iteration.
  enum class TimingScale
  {
    // tw = timing_bias * l_med / t_med, where l_med is the median gate power
    // and t_med the median over gates of sum(lambda * delay) (default). Since
    // t_med scales with lambda, tw * lambda * delay does not change when all
    // of lambda is scaled uniformly: the lambda magnitude has no effect and
    // timing_bias sets the balance.
    kAutoMedian = 0,
    // tw = l_med / d_med, with d_med the median over gates of sum(delay). tw
    // does not depend on lambda, so lambda is the timing/power ratio on a
    // median gate: lambda = 12 makes timing about 12 times power at the
    // seed. timing_bias is not read.
    //
    // A raw SI weight (tw = 1) is not used. The papers' lambda constants were
    // tuned on the ISPD 2012/2013 contest library; in another library's units
    // the same constants make the objective almost pure timing or almost pure
    // leakage. What carries over between libraries is the ratio, which this
    // option expresses.
    kUnit = 1,
    // Livramento et al., DATE 2013, Alg. 1 line 9: tw = base / alpha, with
    // base the auto_median weight and alpha updated from fresh arrival times
    // before every sweep as
    //   alpha = alpha * (A_o / max_j a_j),
    // starting at livramento_alpha0. The paper puts alpha on the power term,
    // which is 1/alpha on the timing term here. The fixed point is WNS = 0: a
    // violating design lowers alpha and so raises timing pressure, and a
    // design with spare slack raises alpha and recovers leakage.
    //
    // The paper gives no initial values for alpha or lambda. The
    // lambda-invariant auto_median base is used because the default lambda
    // seed is in seconds, so its magnitude is a unit artifact that has to be
    // divided out. Only the ratio of the two terms matters and the base
    // removes the lambda magnitude, so livramento_alpha0 is the only scale to
    // tune; lambda_init_value has no effect under this option.
    kLivramentoAlpha = 2,
  };
  TimingScale timing_scale = TimingScale::kAutoMedian;

  // When the LR loop stops. Every option is also bounded by max_iterations.
  //
  // No option stops because timing is met, and none skips the loop when the
  // input already meets timing: global sizing minimizes power subject to
  // timing, so a design that meets timing is a normal input. Two options read
  // the worst slack for other purposes: threshold_battery uses its WNS/TNS
  // targets to switch from the timing phase to the power phase, and
  // stagnation_windows waits for the near-met latch before it starts
  // monitoring.
  enum class TerminationKind
  {
    // Stop at max_iterations, after 3 consecutive rejected iterations
    // (iterations that made WNS worse), or after 2 consecutive iterations that
    // moved nothing (default).
    kFixedIters = 0,
    // Early exit of Sharma et al. (ICCAD 2015, Sec. V-B): stop when neither
    // the average power nor the lowest power found so far improves during two
    // consecutive sets of 5 iterations. The stagnation_* constants set the
    // window, the count, the improvement threshold and whether TNS must also
    // stagnate. The same constants express the rule of Mangiras &
    // Dimitrakopoulos (stop when TNS and leakage improve by less than 1%
    // across two consecutive iterations) with window 1, count 1, frac 0.01
    // and require_tns on.
    kStagnationWindows = 1,
    // Threshold criteria of Chinnery & Sharma (ISPD 2022, Sec. 4), applied in
    // two phases:
    //   timing phase - ends when TNS is within term_tns_target_frac of the
    //     clock period T, WNS is within term_wns_target_frac of T, or TNS
    //     improves by less than term_tns_improve_frac over the last
    //     term_improve_window iterations. The run then continues in the
    //     power phase (RSZ-0450).
    //   power phase - ends the run (RSZ-0451) when power improves by less
    //     than term_power_improve_frac over that window, or when TNS becomes
    //     worse than at the end of the timing phase.
    // term_wall_limit_s is a wall-clock limit in both phases. The paper also
    // changes the lambda update exponents between the phases; that is not
    // implemented, so here the phases differ only in their stop criteria.
    kThresholdBattery = 2,
    // Stop only at max_iterations. Used by paper presets whose paper has no
    // early exit, so that the paper's multiplier schedule runs to completion.
    kPureCap = 3,
  };
  TerminationKind termination = TerminationKind::kFixedIters;

  // Which iterate survives the LR loop. LR iterates are not monotone, so the
  // papers keep the best solution seen and restore it at the end. Each option
  // owns the loop's checkpoint journal (BestTracker), so they are mutually
  // exclusive. The default is kFlachDominance, while rsz_baseline selects
  // kWnsPassReject, so a run with no preset does not match rsz_baseline here.
  enum class BestTrackerKind
  {
    // Keep the final iterate.
    kNone = 0,
    // Flach et al., Alg. 1 lines 9-13: an iterate replaces the stored best if
    // |TNS| < best_tns_target_frac * T and its power is lower. The best is
    // restored at the end of the run.
    kFlachDominance = 1,
    // Reimann et al., Eq. 6: exponentially weighted score of the changes in
    // power, area, timing violation and WNS relative to the input solution;
    // keep the solution with the best score. The input scores 0 and is kept
    // when no iterate scores higher. Small WNS noise is tolerated when power
    // or area improve, while real timing degradation scores strongly
    // negative.
    kReimannScore = 2,
    // Keep the netlist of the last sweep whose WNS matched or beat every
    // earlier sweep, using journal checkpoints, and undo later changes at loop
    // exit. The norm_subgradient step halving on a rejected sweep uses the
    // driver's own WNS measurement, not this option.
    kWnsPassReject = 3,
    // Livramento et al., Alg. 1 line 20: among the iterates with no setup,
    // max capacitance or max slew violation, keep the one with the lowest
    // power. If no iterate is free of violations, the final iterate is kept.
    kLivramentoFeasible = 4,
  };
  BestTrackerKind best_tracker = BestTrackerKind::kFlachDominance;

  // Named configurations. Each paper preset selects the options that
  // correspond to one published LR sizer; options it does not set keep the
  // rsz_baseline choice, and paper constants come from the field defaults
  // below. Every paper preset is suffixed `_partial` because none implements
  // every component of its paper; applyPreset() lists what each one omits
  // and where it adapts the paper (for example discrete cells instead of
  // continuous sizing). Options set individually after the preset override
  // it.
  enum class Preset
  {
    // OpenROAD's own LR sizer: every option at its default, with the
    // wns_pass_reject best tracker.
    kRszBaseline = 0,  // "rsz_baseline"
    kChen = 1,         // "chen_partial": additive subgradient, constant(1) seed
    kTennakoon = 2,    // "tennakoon_partial": arrival-ratio update
    kFlach = 3,        // "flach_partial": asymmetric slack scaling,
                       // constant(12) seed, local slack veto, dominance
                       // tracker
    kSharmaSeq = 4,    // "sharma_seq_partial": cexp update, constant(1) seed,
                       // Fast-OLR, local slack veto, stagnation windows
    kReimann = 5,      // "reimann_partial": dWNS update, estimation-loop seed
    kMangiras = 6,     // "mangiras_partial": state-adaptive seed with Flach's
                       // update, size-step move set
    kLivramento = 7,   // "livramento_partial": ratio update, cost_fanout_slew,
                       // alpha-scheduled timing weight, cap fix pass
    kChinnery = 8,     // "chinnery_partial": two-phase threshold termination,
                       // arc-slack update, constant(1) seed, local slack veto
  };
  // The preset that seeded this config. Because the default is kRszBaseline,
  // preset_explicit (set only by applyPreset) tells whether a preset was
  // actually requested; the configuration echo prints `unset` when it was not.
  // Neither field changes engine behavior.
  Preset preset = Preset::kRszBaseline;
  bool preset_explicit = false;

  // Optional clock network sizing: Global sizing excludes clock network
  // instances by default. Can be enabled for post-CTS timing repair for better
  // clock performance.
  bool include_clock_network = false;
  // Whether global sizing may resize registers: instances whose Liberty cell
  // is sequential (flip-flops, latches and statetable cells). When false,
  // neither the init pass nor the sweeps touch them. Default true; the paper
  // presets set false.
  bool size_registers = true;
  // The power each gate's cost minimizes. The termination rules and best
  // trackers that measure the design's power (stagnation_windows,
  // threshold_battery, flach_dominance, reimann_score) sum the same power over
  // the design.
  enum class PowerObjective
  {
    // The cell's leakage (default).
    kLeakage = 0,
    // The cell's leakage, plus its internal power, plus the switching power
    // its input capacitance adds to its fanin nets. The switching activity is
    // OpenSTA's, read once per run. See lr/ObjectivePower.hh.
    kTotal = 1
  };
  PowerObjective power_objective = PowerObjective::kLeakage;
  // In the power phase of termination = threshold_battery, skip every
  // candidate cell whose objective power is above the current cell's
  // (Chinnery and Sharma, ISPD 2022, Table 2: "skip higher power libcells in
  // power recovery"). Default false; chinnery_partial sets true.
  bool power_phase_filter = false;
  // Slack target of the LR loop. It enters the per-gate downsize budget
  // (slack - margin), the endpoint mu seed and the slack references of the
  // lambda updates. It does not stop the loop. Default 0.
  float setup_slack_margin = 0.0f;
  // Maximum number of LR iterations. Default 20; the paper presets use their
  // paper's iteration budget.
  int max_iterations = 20;
  // Maximum number of sweeps per LR iteration. After each multiplier update
  // the sweep is repeated with the multipliers held fixed, and timing
  // refreshed after every sweep, until a sweep keeps no move or this many
  // sweeps have run. Chen et al. (ICCAD 1998, SOLVE_LRS/mu step 4) and
  // Tennakoon and Sechen (ICCAD 2002, Sec. 3.2) repeat until nothing
  // improves. Their continuous, convex subproblem settles; with discrete
  // cells two gates can swap back and forth, so the loop needs a cap. Must be
  // at least 1. Default 1 (one sweep per iteration); chen_partial and
  // tennakoon_partial set 10.
  int max_inner_sweeps = 1;
  // At the start of every LR iteration after the first, set every sized gate
  // back to the cell it had right after the init pass, so every subproblem is
  // solved from the same sizes. Chen et al. (ICCAD 1998, SOLVE_LRS/mu step 1)
  // start every subproblem solve from the lower size bound. Their solver
  // reaches the same optimum from any start; one greedy sweep over discrete
  // cells does not, so the restart is meant for max_inner_sweeps > 1, which
  // lets each iteration rebuild the sizes it discards. Default false;
  // chen_partial sets true.
  bool restart_each_iteration = false;
  // After every sweep, walk the sized gates from outputs to inputs and resize
  // each gate whose output load exceeds its Liberty max capacitance, choosing
  // among the cells of its group with its current Vt flavor (Livramento et
  // al., DATE 2013, Alg. 3). See lr/ViolationRepair.hh for the selection
  // rule. Default false; livramento_partial sets true.
  bool cap_fix_pass = false;
  // Once, after init_mode's pass and before the first LR iteration, walk the
  // sized gates from inputs to outputs and upsize each gate whose output slew
  // exceeds its limit, choosing among the cells of its group with its current
  // Vt flavor and rejecting any cell that would push a fanin driver over its
  // max capacitance (Sharma et al., ICCAD 2015, Sec. III-A). See
  // lr/ViolationRepair.hh for the selection rule. Default false;
  // sharma_seq_partial sets true.
  bool slew_fix_pass = false;
  // Move the max-capacitance limits into the Lagrangian (Livramento et al.,
  // DATE 2013, Eq. 3). Each output pin of a sized gate, or of a register that
  // would be sized with size_registers on, whose cell declares a Liberty max
  // capacitance gets a multiplier beta, updated after every iteration with
  // lambda. A candidate pays beta * (load - limit)
  // on its output pins and on its fanin drivers instead of being rejected for
  // exceeding a limit, and the post-sweep max-cap re-check does not revert it.
  // The max-slew check is unchanged. See lr/CapMultipliers.hh. Default false;
  // livramento_partial sets true.
  bool relax_max_cap = false;
  // Step size α for the dual-subgradient update on λ.
  //   λ_e ← max(floor, λ_e · (1 + α · g_e_norm))
  // with g_e_norm ∈ [-1, 0]. Tight arcs (g=0) are unchanged; arcs at full
  // slack (g=-1) shrink to (1-α)·λ. Halved on pass rejection.
  float beta = 0.6f;
  // Endpoint seed exponent: mu_k ~ max(0, margin - slack_k)^p.
  float mu_exponent = 2.0f;
  // Floor for multipliers (subgradient floor so unused arcs can re-enter).
  float lambda_floor = 1e-12f;
  // Dimensionless balance between timing pressure and power cost.
  // bias = 1.0 keeps Σλ·d (scaled) ≈ power cost on the median gate.
  //
  // Read by timing_scale = auto_median and livramento_alpha
  // (tw = timing_bias * l_med / t_med) and ignored by timing_scale = unit. The
  // paper presets set 12 with auto_median or livramento_alpha and 1 with unit.
  float timing_bias = 64.0f;
  // Safety derate (<= 1) on the per-gate distributed downsize budget. The
  // depth-normalized distribution already guarantees per-path budget sums
  // <= path slack, so 1.0 is feasible in theory; a value < 1 adds margin for
  // the un-modeled slew cascade / estimated-vs-routed parasitic gap.
  float budget_safety_factor = 1.0f;
  // Relative cost improvement an upsize must achieve to be accepted
  // (acceptGateMove); a downsize is accepted on any improvement. This filters
  // out moves caused by cost noise that would churn the design without a real
  // timing gain. Default 0.02. The papers take the lowest-cost candidate
  // without such a rule, so the paper presets set 0.
  float upsize_hysteresis = 0.02f;

  // Paper constants. Each default is the value from the paper that uses it.

  // Initial lambda of lambda_seed = constant. Flach et al. use 12, Sharma et
  // al. use 1, and Chen et al. give no value. The value only matters when mu
  // is derived from lambda (mu_policy = endpoint_*) and timing_scale = unit;
  // otherwise the projection and the auto_median weight cancel it (see
  // MuPolicy and TimingScale).
  float lambda_init_value = 12.0f;
  // Exponent K of the state-adaptive seed (Mangiras & Dimitrakopoulos,
  // Eqs. 5-6); larger values spread the initial lambdas further. The paper
  // uses 2; 1 converges more slowly and 3 or 4 bring no further gain.
  float lambda_seed_exponent = 2.0f;
  // Number of trial iterations of lambda_seed = estimation_loop. Reimann et
  // al. only say "a few"; 3 is our choice.
  int est_loop_iters = 3;

  // Step scale of chen_subgradient and endpoint_additive:
  // rho_k = lambda_update_c / k, applied to the violation divided by the clock
  // period. The paper leaves rho_k free. Normalizing by T keeps the constant
  // independent of the library, and a step in raw seconds would be lost in
  // float precision against a lambda of order 1.
  float lambda_update_c = 1.0f;
  // k schedule of flach_slack_scaling (Flach et al., Alg. 2): k starts at
  // flach_k_init, rises to flach_k_tns_small once timing is nearly met
  // (faster leakage recovery), and returns to flach_k_final (<= 1) in the
  // final iterations to remove the remaining violations.
  float flach_k_init = 1.0f;
  float flach_k_tns_small = 4.0f;
  float flach_k_final = 1.0f;
  // cexp schedule of sharma_cexp (Sharma et al., Fig. 2): relaxed target
  // r * T and shrink rate k.
  float sharma_r = 1.01f;
  float sharma_k = 10.0f;
  // Exponents K of sharma_arc_slack on critical (slack < 0) and non-critical
  // arcs, in the timing and the power phase. The values are those of Chinnery
  // and Sharma, ISPD 2022, Sec. 4: "an exponent of 4 (1)" in timing
  // improvement and "1 (4)" in power recovery. Sharma et al., TCAD 2020 use
  // (4, 1) and (1, 6) on ISPD 2012 designs, (1, 0.25) and (1, 4) on ISPD 2013
  // designs (Tables II and III).
  float arc_slack_k_timing_crit = 4.0f;
  float arc_slack_k_timing_noncrit = 1.0f;
  float arc_slack_k_power_crit = 1.0f;
  float arc_slack_k_power_noncrit = 4.0f;
  // Constants of reimann_dwns (Reimann et al., Alg. 3):
  // rho_inc = rho_init * (1 + iter), rho_dec = rho_init * (15 + iter). The
  // exponent k follows Eq. 7: reimann_k_est during the estimation loop,
  // reimann_k_lo (< 1) after timing got worse, reimann_k_hi (> 1) after it
  // improved, and reimann_k (1, neutral) otherwise. The paper only gives the
  // ranges, noting that they affect convergence speed but not final quality;
  // the values here are our choice.
  float reimann_rho_init = 0.05f;
  float reimann_k = 1.0f;
  float reimann_k_est = 5.0f;
  float reimann_k_lo = 0.5f;
  float reimann_k_hi = 2.0f;
  // Slack reference of reimann_dwns. The paper compares each arc's slack with
  // its initial slack S_init, which holds the solution near the input timing
  // (power recovery that does not chase input violations). When the input
  // violates timing and the sizer improves it, every later slack stays above
  // S_init, the increase branch never fires, and lambda decays to the floor.
  enum class ReimannSetpoint
  {
    // Reference = each arc's initial slack S_init, as in the paper (default).
    kSInit = 0,
    // Reference = setup_slack_margin. This departs from the paper: the target
    // becomes feasibility instead of the input timing, so the increase branch
    // fires on violating arcs, and the rest of the update (rho schedules,
    // Eq. 7 k schedule, asymmetric exponents) works as published.
    kSlackTarget = 1
  };
  ReimannSetpoint reimann_setpoint = ReimannSetpoint::kSInit;
  // Initial alpha of timing_scale = livramento_alpha (Livramento et al.,
  // Alg. 1 line 6, "initial positive value"). The paper gives no value; 1.0
  // starts at the auto_median balance. See kLivramentoAlpha.
  float livramento_alpha0 = 1.0f;

  // Tolerance of downsize_guard = local_slack_veto. Flach et al., Eq. 14 is
  //   gamma = 1 + (-min(0, WNS) / T)
  // - it exceeds 1 while the design violates timing (bounded local
  // degradation is allowed, so the sweep can climb hills) and decays to 1 as
  // the design converges (degradation is then forbidden). This option scales
  // the hill-climbing part:
  //   gamma = 1 + gamma_local_slack * (-min(0, WNS) / T)
  // so 1.0 (default) is exactly Eq. 14, 0.0 forbids any local degradation
  // from the first iteration, and > 1 is more permissive than the paper.
  float gamma_local_slack = 1.0f;

  // Constants of termination = stagnation_windows. Iterations are grouped
  // into windows of stagnation_window iterations, and the run stops after
  // stagnation_count consecutive windows that improve by no more than
  // stagnation_improve_frac. A window of 1 compares each iteration with the
  // previous one; a window w > 1 compares the averages of consecutive
  // disjoint w-iteration blocks. The defaults are Sharma et al.'s rule
  // (window 5, count 2, any improvement resets the count).
  int stagnation_window = 5;
  int stagnation_count = 2;
  float stagnation_improve_frac = 0.0f;
  // If true, a window is stagnant only if TNS also failed to improve by
  // stagnation_improve_frac (Mangiras & Dimitrakopoulos: "TNS and total
  // leakage"). If false, only power is tested (Sharma et al.).
  bool stagnation_require_tns = false;
  // Near-met gate. The run is latched "near met" once
  // WNS >= -near_met_gate_frac * T and stays latched (LrState::near_met).
  // Until then the stagnation_windows monitor is inactive and
  // local_slack_veto accepts every candidate. This is Sharma et al.'s "after
  // the design timing is within 1% of the target"; sharma_seq_partial sets
  // 0.01. A negative value (default) disables the gate: the run counts as
  // near met from the first iteration.
  float near_met_gate_frac = -1.0f;
  // Constants of termination = threshold_battery (Chinnery & Sharma,
  // Sec. 4). The timing targets are fractions of the clock period T (TNS
  // within 10% of T, WNS within 1%). The improvement thresholds are relative
  // improvements over the last term_improve_window iterations (TNS 10%,
  // power 1%). The wall-clock limit is the paper's 72 hours; the paper's
  // 80-iteration limit is max_iterations.
  float term_tns_target_frac = 0.10f;
  float term_wns_target_frac = 0.01f;
  float term_tns_improve_frac = 0.10f;
  float term_power_improve_frac = 0.01f;
  int term_improve_window = 3;
  float term_wall_limit_s = 259200.0f;

  // Constant of best_tracker = flach_dominance: an iterate may replace the
  // stored best only if |TNS| < best_tns_target_frac * T (Flach et al., "TNS
  // is less than 10% of T").
  float best_tns_target_frac = 0.10f;

  // Reset the config to its defaults, then apply the given preset. Options
  // read from dbProperties after this call override the preset.
  void applyPreset(Preset p);

  // Pair the lambda update with a mu policy that keeps its effect. Call once,
  // on the per-run copy of the config, before validate().
  //
  // Under the slack-derived mu policies (reseed_each_iter, seed_once,
  // update_as_lambda) the flow projection re-anchors every endpoint to a
  // slack-derived mu at every iteration, which discards the lambda magnitudes
  // the update rule produced. Different update rules then give the same
  // multipliers. So when a paper lambda update is combined with such a policy
  // and the user did not choose the mu policy, this switches to
  // endpoint_lambda and warns (RSZ-0447). An explicit choice is kept and
  // reported at info level (RSZ-0448).
  //
  // norm_subgradient is left alone: it is the default rule, its lambda
  // magnitude has no meaning to preserve, and rsz_baseline must stay
  // unchanged.
  void resolveLambdaMuPairing(utl::Logger* logger);

  // Check cross-option constraints. A hard violation reports an RSZ error
  // (logger->error is [[noreturn]]); a soft one warns. Returns true when no
  // hard constraint fired, so start() can bail out without relying on the
  // throw. Called once from GlobalSizingPolicy::start().
  //
  // Hard: max_inner_sweeps must be at least 1, lambda_seed = state_adaptive
  // requires init_mode = as_given, and cost_global_phi excludes
  // cost_delta_delay (two estimators of the same downstream effect). Warnings
  // cover options that have no effect in the selected mode (init_seed,
  // fast_olr_start_iter, the term_* constants, traversal / gs_refresh under
  // jacobi_snapshot, and the phase-dependent sharma_arc_slack and
  // power_phase_filter without threshold_battery), combinations that
  // double-count or undo each other (cost_global_phi with cost_fanout_slew,
  // state_adaptive with reseed_each_iter, restart_each_iteration with one
  // sweep per iteration), estimation_loop after an init pass, and the
  // guard/engine pairs jacobi_snapshot + local_slack_veto and
  // gauss_seidel_topo + depth_budget.
  bool validate(utl::Logger* logger) const;
};

// Every Preset value, in declaration order. A check that must hold for every
// preset should iterate this instead of a hand-written list, so that a new
// preset is covered automatically. A table that maps each preset to an
// expected value can assert its length against this array.
inline constexpr GlobalSizingConfig::Preset kAllPresets[] = {
    GlobalSizingConfig::Preset::kRszBaseline,
    GlobalSizingConfig::Preset::kChen,
    GlobalSizingConfig::Preset::kTennakoon,
    GlobalSizingConfig::Preset::kFlach,
    GlobalSizingConfig::Preset::kSharmaSeq,
    GlobalSizingConfig::Preset::kReimann,
    GlobalSizingConfig::Preset::kMangiras,
    GlobalSizingConfig::Preset::kLivramento,
    GlobalSizingConfig::Preset::kChinnery,
};

// Enum <-> Tcl/report string helpers (used by the configuration echo and the
// dbProperty reader). toString returns "unknown" for an out-of-range value.
const char* toString(GlobalSizingConfig::InitMode mode);
const char* toString(GlobalSizingConfig::LambdaSeed seed);
const char* toString(GlobalSizingConfig::LambdaUpdate update);
const char* toString(GlobalSizingConfig::MuPolicy policy);
const char* toString(GlobalSizingConfig::KktProjection projection);
const char* toString(GlobalSizingConfig::SweepEngineKind engine);
const char* toString(GlobalSizingConfig::GsRefresh refresh);
const char* toString(GlobalSizingConfig::Traversal traversal);
const char* toString(GlobalSizingConfig::DownsizeGuard guard);
const char* toString(GlobalSizingConfig::MoveSet move_set);
const char* toString(GlobalSizingConfig::OutputDrcVeto veto);
const char* toString(GlobalSizingConfig::TimingScale scale);
const char* toString(GlobalSizingConfig::TerminationKind termination);
const char* toString(GlobalSizingConfig::BestTrackerKind tracker);
const char* toString(GlobalSizingConfig::ReimannSetpoint setpoint);
const char* toString(GlobalSizingConfig::PowerObjective objective);
const char* toString(GlobalSizingConfig::TimingCost cost);
const char* toString(GlobalSizingConfig::Preset preset);
// Parse a preset name; returns false on an unrecognized name.
bool parsePreset(const char* name, GlobalSizingConfig::Preset& out);
// Parse an option name; returns false and leaves `out` unchanged on an
// unrecognized name.
bool parseInitMode(const char* name, GlobalSizingConfig::InitMode& out);
bool parseLambdaSeed(const char* name, GlobalSizingConfig::LambdaSeed& out);
bool parseLambdaUpdate(const char* name, GlobalSizingConfig::LambdaUpdate& out);
bool parseMuPolicy(const char* name, GlobalSizingConfig::MuPolicy& out);
bool parseSweepEngine(const char* name,
                      GlobalSizingConfig::SweepEngineKind& out);
bool parseGsRefresh(const char* name, GlobalSizingConfig::GsRefresh& out);
bool parseTraversal(const char* name, GlobalSizingConfig::Traversal& out);
bool parseDownsizeGuard(const char* name,
                        GlobalSizingConfig::DownsizeGuard& out);
bool parseMoveSet(const char* name, GlobalSizingConfig::MoveSet& out);
bool parseOutputDrcVeto(const char* name,
                        GlobalSizingConfig::OutputDrcVeto& out);
bool parseTimingScale(const char* name, GlobalSizingConfig::TimingScale& out);
bool parseTermination(const char* name,
                      GlobalSizingConfig::TerminationKind& out);
bool parseBestTracker(const char* name,
                      GlobalSizingConfig::BestTrackerKind& out);
bool parseReimannSetpoint(const char* name,
                          GlobalSizingConfig::ReimannSetpoint& out);
bool parsePowerObjective(const char* name,
                         GlobalSizingConfig::PowerObjective& out);
bool parseTimingCost(const char* name, GlobalSizingConfig::TimingCost& out);

}  // namespace rsz
