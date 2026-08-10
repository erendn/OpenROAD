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
// The engine is decomposed into per-axis strategies (see src/rsz/src/lr/); each
// enum below selects the implementation for one axis. Every enum currently has
// a single ` = current behavior` option; later ablation milestones (M1+) add
// the alternative options and their paper constants. A named `Preset` bundles a
// choice for every axis; `applyPreset` sets them, `validate` enforces
// cross-axis constraints. `rsz_baseline` reproduced the pre-ablation engine
// bit-for-bit at M0; it no longer does, because the two pre-campaign engine
// fixes (2026-07-25, 2026-07-29) removed the end-of-phase WNS accept and
// fixed_iters' WNS-meets-margin exit from EVERY bundle, the baseline included,
// so the control column and the paper arms share one phase semantics.
struct GlobalSizingConfig
{
  // === N - Initial solution =================================================
  // Optional pre-LR initialization: replace every editable instance with
  // another member of its swappable-equivalence group before LR runs.
  //
  // HONESTY NOTE (binding, iteration-2 plan §2.1): the selector RANKS a group
  // by `Resizer::cellLeakage()`, with cell drive resistance as the tie-break.
  // It has no size or Vt dimension of its own - leakage is the size/drive
  // PROXY. "min"/"max"/"middle" below therefore mean min/max/middle OF THE
  // cellLeakage()-RANKED GROUP, which is what the older kMinSizeMaxVt /
  // kMaxSizeMinVt spellings over-claimed in the other direction.
  enum class InitMode
  {
    // Keep the incoming netlist. The "as-given" initial solution; a no-op.
    kAsGiven = 0,
    // Lowest-ranked member of each group (= the old min_size_max_vt).
    kMinSize = 1,
    // Highest-ranked member (= the old max_size_min_vt).
    kMaxSize = 2,
    // kMinSize + reverse-topological electrical repair - the flach/sharma
    // C-init. After the min-size reset the pass walks the same editable gates
    // from outputs toward inputs and upsizes each one whose OWN output pins
    // violate max-cap or max-slew, to the lowest-leakage member of its
    // swappable group that clears them. Deterministic and seed-free.
    //
    // WHAT IT DOES NOT GUARANTEE. It is a repair pass, not an optimizer, and
    // three limits follow. (a) A gate whose swappable group contains no member
    // that clears its violation is LEFT AT MINIMUM - the pass will not spend
    // leakage on a gate it cannot actually fix - so remaining violations are
    // possible and are counted in RSZ-0445 rather than hidden. (b) It judges by
    // the same model the sweep's own candidate filter uses (Liberty cap limit,
    // drive-resistance-linear slew estimate calibrated at the measured
    // post-reset slew), not by a full STA per gate, and a max_capacitance set
    // only by SDC is outside that view. (c) It does not distinguish violations
    // the reset INTRODUCED from ones the incoming netlist already had; on a
    // netlist reset to minimum that distinction has no operational meaning.
    kMinSizeFixviol = 3,
    // Per-instance uniform draw over the whole group (the as-given cell
    // included), driven by init_seed. A netlist-side variation source that is
    // INDEPENDENT of the global-placement seed by construction - see init_seed.
    kRandom = 4,
    // The lower median of the ranked group, index floor((n-1)/2). Deterministic
    // and seed-free.
    kAverage = 5,
  };
  InitMode init_mode = InitMode::kAsGiven;

  // Seed for init_mode = random. Its own flag, never derived from and never
  // correlated with the global-placement seed: placement perturbation and
  // netlist initialization are two SEPARATE variation sources for the
  // variability study, and deriving one from the other would confound them.
  // Each instance's draw is keyed by a hash of (init_seed, instance name), so
  // one seed reproduces the same initialization whatever order - and whatever
  // thread count - the instances are visited in.
  int init_seed = 0;

  // === D - lambda/mu initialization =========================================
  enum class LambdaSeed
  {
    // Delay-proportional lambda seed + WNS-biased crit^p mu seed (current).
    kDelayPropCritMu = 0,
    // Constant lambda = lambda_init_value on every data arc (Flach 12,
    // Sharma/Chen 1); mu seeded from endpoint criticality as in the baseline.
    kConstant = 1,
    // Mangiras Eqs. 5-7 state-adaptive seed: per-arc raw lambda =
    // (timing-criticality ratio x leakage-ratio-vs-min)^lambda_seed_exponent,
    // made KKT-consistent by the proportional_reverse_topo projection (E3),
    // which the driver applies after every seed (Eq. 7 == that projection at
    // seed time). Requires init_mode = as_given (validate()).
    kStateAdaptive = 2,
    // Reimann Alg. 2 loop 1: seed the baseline lambda, then the driver runs
    // est_loop_iters dry-run sweeps (evaluate + update lambda + roll back) to
    // estimate a warm-start multiplier field before the main loop.
    kEstimationLoop = 3,
  };
  LambdaSeed lambda_seed = LambdaSeed::kDelayPropCritMu;

  // === E1/E2 - lambda update ================================================
  enum class LambdaUpdate
  {
    // Multiplicative normalized dual-subgradient with reject-halved step
    // (current). The alpha halving on pass rejection is this updater's private
    // feature.
    kNormSubgradient = 0,
    // Flach TCAD'14 Alg. 2: asymmetric multiplicative slack scaling
    // lambda *= (1+|slack|/T)^{+1/k} (violating) / (1+slack/T)^{-k}
    // (non-violating), with the k-schedule 1 -> 4 -> <=1.
    kFlachSlackScaling = 1,
    // Chen-Chu-Wong ICCAD'98 additive subgradient lambda += rho_k*(arc
    // violation), rho_k = c/k. The theory baseline.
    kChenSubgradient = 2,
    // Tennakoon-Sechen ICCAD'02 (Forge, Fig. 13): constant-free multiplicative
    // local arrival-ratio update lambda *= a_from/(a_to - d).
    kTennakoonRatio = 3,
    // Sharma ICCAD'15 (Fig. 2 line 10): multiplicative criticality update
    // lambda *= (1 - slack_j/T)^cexp - the sink-node slack over the clock
    // period - with the accumulating cexp schedule (r, k). (The old
    // (a_j/q_j)^cexp reading was a transcription error; see LambdaUpdater.cc.)
    kSharmaCexp = 4,
    // Reimann ISPD'16 (Alg. 3): dWNS-normalized, S_init-targeted asymmetric
    // multiplicative update with rho_inc/rho_dec schedules.
    kReimannDwns = 5,
    // Livramento DATE'13 (Alg. 1 line 13): local multiplicative update
    // lambda *= (a_from + d)/a_to. Tennakoon-inspired (its local step size
    // rho_k = lambda_ji/a_i, Eq. 9) but NOT Tennakoon's rule: Tennakoon's own
    // Fig. 13 is rho_k = lambda_ji/(a_i - D_ji), a different denominator. The
    // two agree only on a critical arc (a_i = a_j + D) and diverge everywhere
    // else. Livramento's Alg. 1 line 14 (PI-sourced arcs, D_ji/a_i) is the
    // a_from = 0 degenerate case of line 13, so one formula covers both.
    kLivramentoRatio = 6,
  };
  LambdaUpdate lambda_update = LambdaUpdate::kNormSubgradient;

  // === E4 - endpoint (mu) multiplier policy =================================
  // How the per-endpoint multipliers mu are maintained each iteration. The
  // papers under-specify endpoint handling (SYNTHESIS §2.4.5); making the
  // choice explicit is itself a knob.
  //
  // mu is read in exactly two places: the projection's endpoint boundary
  // condition (FlowProjection.cc - the only control path) and the duality-gap
  // diagnostic (Termination.cc). So this axis and the projection's endpoint
  // anchoring are one mechanism seen from two sides, and this axis decides
  // whether a uniform lambda seed magnitude survives the projection - see
  // projectFlowBalance's contract and the plan's "λ-magnitude finding".
  enum class MuPolicy
  {
    // Re-seed mu from the current endpoint slacks every iteration (current):
    // WNS-biased mu_k ~ max(0, margin - slack_k)^p, then normalized.
    kReseedEachIter = 0,
    // Seed mu once (via the seeder at iter 0) and never touch it again.
    kSeedOnce = 1,
    // Update mu multiplicatively from endpoint slack, analogous to lambda
    // (no re-seed). An ADAPTED relative of the papers' branch-1 endpoint
    // update, NOT the faithful one: it scales mu by a slack-derived factor
    // rather than the papers' a_k/required_k ratio. The faithful Tennakoon Fig.
    // 13 branch 1 / Livramento Alg. 1 L12 is kEndpointRatio (C1), which
    // supersedes this as the tennakoon/livramento endpoint writer (tennakoon
    // audit §2 item 2).
    kUpdateAsLambda = 2,
    // mu is not chosen at all: it is *derived* as the endpoint's own in-arc
    // lambda sum, which the projection then anchors to (a no-op rescale), so
    // arcs into an endpoint are ordinary lambda arcs, slack-scaled by the E1/E2
    // updater like any other arc. This is the papers' endpoint treatment -
    // Flach's uniform seed covers endpoint arcs and Alg. 2 slack-scales them;
    // Livramento's lambda_jpo (Alg. 1 L12) and Mangiras's Eq. 6 lambda_ik are
    // ordinary multipliers that *anchor* the reverse-topological distribution
    // rather than being overwritten by it.
    //
    // The other three options all anchor to a mu seeded from a slack field
    // (seedBaselineMu: (margin - slack)^p normalized to max 1), which carries
    // no information about lambda's magnitude - so they cancel a uniform seed
    // constant c at iteration 0 no matter what the update rule does. This one
    // makes the projection commute with a uniform rescale, so lambda_init_value
    // (Flach's 12 vs Chen's 1) is a live knob under it and dead under them.
    kEndpointLambda = 3,
    // Endpoint pressure as the papers' own multiplicative branch-1 update
    // (Tennakoon Fig. 13 branch 1 = Livramento Alg. 1 L12 - the identical
    // formula). mu is derived once from the endpoint's in-arc lambda sum (like
    // endpoint_lambda at the first projection, so mu_0 is proportional to the
    // seed magnitude), then per iteration mu_k <- mu_k * (a_k / required_k),
    // and the projection ANCHORS the endpoint in-arcs to that mu. Ascent on a
    // violating endpoint (ratio > 1); self-damping to 1 as a_k -> required_k -
    // the papers' own gain control, no rho schedule needed. Restores dual
    // ascent for tennakoon_partial and livramento_partial (C1).
    kEndpointRatio = 4,
    // Endpoint pressure as Chen SOLVE_LDP step 3's i=0 branch (branch 1):
    // lambda_j0 += rho_k*(a_j - A_0). Same mu derivation from the
    // endpoint-lambda seed (mu_0 proportional to the seed constant c), then per
    // iteration mu_k <- max(0, mu_k + rho_k*(-slack_k)/T) with rho_k = c/(k*T)
    // - the same T-normalized schedule the chen_subgradient branch-2 update
    // already uses (lambda_update_c is shared between the two branches, the
    // paper's single rho_k). rho_k -> 0 damps the growth (the paper's gain
    // control). Restores dual ascent for chen_partial (C1).
    kEndpointAdditive = 5,
  };
  MuPolicy mu_policy = MuPolicy::kReseedEachIter;
  // Set by the dbProperty reader when the user passed `-mu_policy` explicitly.
  // NOT set by applyPreset: a preset's mu choice is a bundle default, and the
  // auto-pairing below has to be able to tell "the bundle left mu at the
  // baseline's re-seed" from "the user asked for the re-seed". An explicitly
  // set policy is always honored.
  bool mu_policy_explicit = false;
  // Set by resolveLambdaMuPairing when it moved mu_policy. Echoed in RSZ-0417
  // (`mu_autopair=`) so the harness can tell an auto-paired cell from one whose
  // mu policy was configured, without re-deriving the rule.
  bool mu_auto_paired = false;

  // === E3 - KKT projection ==================================================
  enum class KktProjection
  {
    // Proportional reverse-topological redistribution onto the flow-balance
    // polytope (current).
    kProportionalReverseTopo = 0,
  };
  KktProjection kkt_projection = KktProjection::kProportionalReverseTopo;

  // === F1 - LRS sweep engine ================================================
  enum class SweepEngineKind
  {
    // 3-phase parallel Jacobi sweep over frozen per-gate snapshots, with the
    // depth-normalized downsize budget guard (current). Evaluates every gate
    // against one stable sweep-start snapshot, so downstream gates do NOT see
    // upstream commits within the same sweep.
    kJacobiSnapshot = 0,
    // Sequential Gauss-Seidel sweep in `traversal` order: build a just-in-time
    // per-gate snapshot, evaluate it, and commit the winning replacement before
    // moving to the next gate, so downstream gates see fresh upstream state
    // (the papers' standard LRS mechanics - Flach Alg. 3, Livramento, Reimann).
    // The per-commit refresh fidelity is `gs_refresh`. Runs single-threaded on
    // the main thread (the JIT snapshot reads live STA), hence deterministic.
    kGaussSeidelTopo = 1,
  };
  SweepEngineKind sweep_engine = SweepEngineKind::kJacobiSnapshot;

  // === F1 sub-mode - Gauss-Seidel per-commit refresh fidelity ===============
  // Only consumed by the gauss_seidel_topo engine (ignored under Jacobi, which
  // has no per-commit refresh). Selects how much timing state is refreshed
  // after each committed replacement so the next gate's snapshot reads it.
  enum class GsRefresh
  {
    // Paper-faithful truncated local update (Flach "local timing update",
    // flach_et_al.md §5/VII-A): after each commit, re-estimate only the
    // just-touched nets' parasitics (updateParasitics is already incremental -
    // it re-estimates only nets modified since the last call) so the next
    // snapshot reads fresh fanin-driver load, own-arc, and sink-net forward
    // delays (OpenSTA recomputes forward delays lazily on the slew/loadCap
    // query). Required times are NOT propagated per commit, so the depth-budget
    // guard stays frozen at its sweep-start value (Flach: required times are
    // necessarily stale mid-sweep). The cheaper mode.
    kLocal = 0,
    // OpenSTA incremental timing after each commit: updateParasitics THEN
    // findRequireds, so the next snapshot reads globally-consistent forward and
    // backward timing. Slower; exists to measure the staleness error of
    // gs_local. NOTE (M4): because the per-gate snapshot reads only forward
    // electrical state plus the frozen depth budget - never required times -
    // both modes select identical cells under the M4 depth-budget guard; they
    // differ only in per-commit runtime. The fidelity becomes decision-relevant
    // at M5, when the local-slack-veto guard reads mid-sweep required times.
    kIncremental = 1,
  };
  GsRefresh gs_refresh = GsRefresh::kLocal;

  // === F2 - Gauss-Seidel traversal order ====================================
  // Only consumed by the gauss_seidel_topo engine. Jacobi always evaluates in
  // its stable snapshot (leaf-instance-iterator) order; traversal is a
  // GS-engine knob for now (a validator warning fires if it is set to a
  // non-default value under Jacobi).
  enum class Traversal
  {
    // Increasing output-vertex level: a gate's fanin drivers commit before it,
    // so it sees fresh upstream state (Flach Alg. 3 forward-topological sweep).
    kForwardTopo = 0,
    // Decreasing output-vertex level (outputs -> inputs).
    kReverseTopo = 1,
    // Most-critical gate first (ascending sweep-start slack). Ties in every
    // mode break on a stable instance id, so the order is deterministic.
    kCriticalitySorted = 2,
  };
  Traversal traversal = Traversal::kForwardTopo;

  // === F3 - Downsize / candidate acceptance guard ===========================
  // The safety test a candidate cell must pass before the LR cost decides.
  // (The hard electrical DRC filter is a separate, always-on veto - A2.)
  enum class DownsizeGuard
  {
    // Per-vertex depth-normalized slack budget, frozen at sweep start
    // (current). A candidate with lower leakage than the current cell may add
    // at most `budget_safety_factor * budget` delay on every output pin.
    // Upsizes are unconstrained. The budget exists to make *simultaneous*
    // (Jacobi) downsizes safe: the per-path sum of gate budgets is <= the path
    // slack, so no path can be overshot. It is our own construction, not a
    // paper's.
    kDepthBudget = 0,
    // Flach's local-negative-slack veto with gamma hill-climbing acceptance
    // (Alg. 4 lines 1/12-14, Eq. 14 - flach_et_al.md §6). The gate's *local
    // negative slack* is the sum of negative slacks over its driver nets and
    // its sink net; a candidate is rejected when it makes that sum worse than
    // gamma * originalSlack, with gamma >= 1 shrinking to 1 as the design
    // converges (see gamma_local_slack). Unlike kDepthBudget this is Flach's
    // per-candidate acceptance test and applies to EVERY candidate, upsizes
    // included: an upsize that wrecks its fanin driver's slack (its bigger
    // input cap slows the driver) is rejected too. The axis keeps the
    // `downsize_guard` name for continuity with the plan (§3.2-F).
    //
    // Required times enter the decision here, which is what makes gs_refresh
    // decision-relevant: the veto reads the slacks frozen at sweep start under
    // jacobi_snapshot and gs_local (stale mid-sweep - Flach's own caveat, his
    // local timing update propagates delays but not required times), and live
    // mid-sweep slacks under gs_incremental (which refreshes required times per
    // commit). Reading slacks live under gs_local is not an option: an OpenSTA
    // slack query lazily reruns findAllArrivals + findRequireds, which would
    // both erase the staleness this ablation measures and cost a full timing
    // update per gate.
    kLocalSlackVeto = 1,
    // No guard: the LR cost alone decides (the electrical DRC veto still
    // applies). Measures what the guards are worth.
    kNone = 2,
  };
  DownsizeGuard downsize_guard = DownsizeGuard::kDepthBudget;

  // === F4 - LRS candidate move set ==========================================
  // Which members of a gate's swappable-equivalence group the per-gate
  // subproblem is even allowed to consider. Distinct from the F3 guard, which
  // filters candidates the cost has already been asked about, and from the A2
  // DRC veto: this axis decides what the sweep ENUMERATES.
  //
  // Both restricted options index the same per-library-cell (drive/width rank x
  // Vth flavor) decomposition of the group - `lr/SizeVthGrid.hh`, the pure core
  // REAUDIT F2 calls for ("one (w,Vth) grid indexing serves both"). Read that
  // header's honesty note before reading "width" here: the rank key is the
  // member's leakage-equivalent cost, ranked INSIDE a Vth flavor, so "next
  // bigger size" means "next member up that flavor's leakage ranking".
  enum class MoveSet
  {
    // Every member of the swappable group, every iteration (current). The
    // exhaustive OLR of Flach Alg. 3, Mangiras Alg. 1 and Sharma's own first
    // four iterations.
    kFullLibrary = 0,
    // Sharma ICCAD'15 Fig. 9 Fast-OLR (sharma_et_al.md §5.2), the paper's
    // ★ stability device: from `fast_olr_start_iter` on, replace the exhaustive
    // scan with a hill descent along the width axis of the current Vth flavor
    // and its two neighbours, stopping each direction at the first
    // non-improving step. Iterations before the switch-over run kFullLibrary,
    // which is the paper's own schedule.
    //
    // Its point is NOT (only) the 3.3x drop in cell evaluations. Local search
    // makes incremental changes, whereas jumping to the global argmin perturbs
    // a nearly-settled solution late in the run: the paper's Fig. 11 shows
    // plain OLR destabilizing TNS on b19_fast after ~100 iterations and ending
    // ~11% worse in power. That is why this option exists at all - the campaign
    // runs sharma at its paper budget of 160 iterations, i.e. entirely past
    // Fig. 11's knee (REAUDIT §3.5).
    //
    // The paper's Fig. 9 line 20-21 slack guard is NOT here: it is Flach's
    // local-slack check, adopted verbatim, and that is the F3 axis
    // (downsize_guard = local_slack_veto, gated by near_met_gate_frac - the
    // same "after the design timing is within 1% of the target" condition).
    // sharma_seq_partial already pins both, so the guard half of Fig. 9 was
    // implemented before the enumeration half.
    kSharmaFastOlr = 1,
    // Mangiras Technologies'21 §4.3 (mangiras_et_al.md §8, and see the venue
    // note in the mangiras_partial block): the restricted move set -
    // "each gate may move only to its next bigger or next smaller size (+-1
    // size step) while Vth swaps remain unrestricted", so the neighborhood is a
    // width-rank band across ALL flavors and a pure Vth swap stays legal. The
    // paper's motivation is physical: at the very end of a flow a +-1 step
    // preserves detailed routes. Unlike Fast-OLR it is a mode, not a schedule -
    // it applies from the first iteration.
    kMangirasSizeStep = 2,
  };
  MoveSet move_set = MoveSet::kFullLibrary;

  // Sharma Fig. 9's switch-over iteration: the 1-based λ-UPDATED LDP iteration
  // at which the pruned enumeration activates; the pre-update sweep does not
  // count. The paper states 5 without justification (sharma_et_al.md §13
  // item 7) and runs exhaustive OLR for the first four iterations. Read only by
  // move_set = sharma_fast_olr; 1 makes Fast-OLR active from the first
  // λ-updated sweep and 0 from the pre-update sweep before it (which is what a
  // smoke test that converges in three sweeps needs to exercise it at all).
  int fast_olr_start_iter = 5;

  // === A2 - Electrical (DRC) constraint handling ============================
  // What the sweep's always-on max-cap / max-slew filter does on a pin the
  // CURRENT cell already violates. The INPUT side has always been relative (a
  // candidate whose bigger input pin cap worsens an already-violating fanin
  // net's max-cap is rejected; one that does not is admitted); this axis is the
  // OUTPUT side of the same filter, where the check was unconditionally
  // absolute until the iteration-2 veto pass.
  //
  // Neither option relaxes the objective: max-cap and max-slew stay hard
  // filters on candidate cells, never priced terms. The Lagrangian treatment of
  // the electrical constraints is Livramento's β/γ penalty (the OTHER arm of
  // this axis), and it is not implemented - see the livramento_partial block.
  enum class OutputDrcVeto
  {
    // Reject any candidate whose own output-pin limits are exceeded, whatever
    // the current cell was doing. The shipped default and the pre-existing
    // behaviour, kept so `rsz_baseline` stays stock.
    //
    // The cost of the strictness is a freeze-out: on a pin no member of the
    // swappable group can clear, EVERY candidate is rejected, so the gate is
    // pinned at whatever cell it entered the loop holding for the whole run.
    // Composed with init_mode = min_size_fixviol, whose repair leaves such a
    // gate at MINIMUM and says so (RSZ-0445), that pin is the minimum size -
    // which is what makes this a measurable axis rather than a safety margin.
    kAbsolute = 0,
    // Reject only a candidate that WORSENS a violation the current cell already
    // has on that pin; an equal-violation candidate is admitted, and on a clean
    // pin the absolute check applies unchanged. Flach Alg. 4 line 6 - "if load
    // violation has increased" (flach_et_al.md:148, 166) - and Chinnery §6,
    // "libcell alternatives that would increase max-load-capacitance or
    // max-input-slew violations are skipped outright" (chinnery_et_al.md:102).
    //
    // In both papers the rule is safe because nothing is violating when the
    // loop starts, so "do not increase" and "do not create" coincide; it is
    // only on a dirty pin that the two part company, and that is the whole
    // content of this axis. See lr/ElectricalModel.hh outputLimitAdmits for the
    // per-pin arithmetic and the tie semantics.
    //
    // TWO PROPERTIES IT DOES NOT HAVE, both of which the shape invites a reader
    // to assume:
    //   * it is not run-level monotone. The bar is the incumbent's excess as
    //     the sweep finds it, re-read from the LIVE load every sweep, so a gate
    //     whose net grows gets a higher bar than the level it previously
    //     reached. Reimann's Alg. 1 anchors on the frozen FLOW-ENTRY level
    //     instead; that is a third mode, not this one (RA F9).
    //   * it compares violations, not transitions. Each side's excess is
    //     measured against its own port's limit, so on a library with
    //     non-monotone per-cell max_transition inside an equivalence group
    //     (sky130: buf_8 declares 7.65 ns, the stronger buf_16 5.01) a weaker
    //     candidate can show the smaller excess while driving the larger
    //     transition. Kept, because "the violation has increased" is what both
    //     papers write and what per-pin ERC measures; see the DECISION (a)
    //     block in LRSubproblem::snapshot.
    //     **RULED: kept (user, 2026-08-09; ITERATION_2_PLAN §9 item 14.)** The
    //     escalation asked whether to add a value-side conjunct; the answer is
    //     no. It ships violation-based, so this paragraph is the disclosure,
    //     not a pending fix.
    kRelative = 1,
  };
  OutputDrcVeto output_drc_veto = OutputDrcVeto::kAbsolute;

  // === B2 - Cost model terms (LRSubproblem) =================================
  // Independent on/off flags for the per-gate Lagrangian cost terms. The
  // gate's own output-arc term (Σλ·d) is always on; these toggle the coupling
  // terms. The rsz_baseline bundle keeps only cost_upstream_load on, matching
  // today's cost expression bit-for-bit.
  //
  // cost_upstream_load (Chen Lemma-2 analog, current second term): price the
  // load change this candidate induces on its upstream drivers' λ-weighted
  // arc delays. On by default.
  bool cost_upstream_load = true;
  // cost_fanout_slew (Livramento Alg. 2 lines 12-14, "sinkArcs"): price the
  // slew change this candidate causes on its immediate-fanout arc delays. The
  // per-output-pin sensitivity Σλ_ik·(δd_ik/δslew) is frozen on the main
  // thread and scaled by the candidate's output-slew change. Off by default.
  bool cost_fanout_slew = false;
  // cost_global_phi (Flach Eq. 5 drainNets / Eq. 11 φ): price the candidate's
  // output-slew change against the cumulative back-propagated sensitivity φ of
  // the whole downstream cone. φ is computed once per iteration in a
  // reverse-topological policy pass. Off by default (Flach shipped ISPD'13 with
  // φ off); the flach_partial preset turns it on. Mutually exclusive with
  // cost_delta_delay (validate()).
  bool cost_global_phi = false;
  // cost_delta_delay: price arc delays relative to a previous-iteration per-arc
  // reference (LrState::prev_delay) rather than absolutely. Off by default; no
  // preset selects it. Mutually exclusive with cost_global_phi.
  //
  // NOT Ozdal's Eqs. 7-10 (post-M5 hardening audit). It was written at M3 as an
  // approximation of them and the M3-era comments cited them directly; do not
  // trust that citation, and do not treat prev_delay as an Ozdal reference. The
  // paper's model differs in kind, not in detail:
  //   - Eq. 7's reference is a LOAD, not a delay: delay_ref_k(s_i^j) =
  //     delay_k(Σ_{t∈fanout(i)} cap(s_t^ref)) - the candidate's OWN delay table
  //     evaluated at the load the fanout cells present at their previous-
  //     iteration sizes. Ours is the arc's incumbent delay, straight off the
  //     graph (captureReferenceDelays).
  //   - Eq. 7 is therefore candidate-DEPENDENT (it is a function of s_i^j);
  //     ours is one constant per arc, identical for every candidate. This is
  //     the whole reason our term is decision-neutral (the reference cancels
  //     out of every pairwise comparison) while Ozdal's is not.
  //   - Eq. 9 KEEPS the reference delay in the subnode weight (power +
  //     Σ μ_k·delay_ref_k) and Eq. 10 puts only the Δcap-induced first-order
  //     delta (Δcap · Σ μ_k ∂T_k/∂cap|_ref) on the DAG edges. We subtract the
  //     reference from every arc instead - closer to the opposite of Eq. 9.
  //   - Eqs. 9/10 only mean anything inside Ozdal's tree-DP over (driver
  //     candidate -> fanout candidate) edges, which is what the delta form
  //     exists to make tractable. Our greedy per-gate sweep has no such edges.
  // M6 (the Ozdal margin updater + tree-DP LRS) must build Eq. 7's reference
  // itself; prev_delay is not a head start on it.
  bool cost_delta_delay = false;

  // === A1 - Objective scale (timing vs leakage balance) =====================
  // How the coefficient `tw` on the Σλ·d term is fixed, i.e. what the objective
  //   leakage + tw · Σλ·d
  // actually balances. This axis decides whether λ's *magnitude* means
  // anything: it is one of the three coupled mechanisms of the plan's
  // "λ-magnitude finding" (the other two are the E3 projection and the E4
  // endpoint anchor).
  //
  // FREEZE SEMANTICS (uniform across every option): the design/library anchor
  // is computed once, before the loop, off the iteration-0 multiplier field,
  // and never recomputed. That is what auto_median has always done, and the
  // other options match it deliberately - a per-iteration anchor would make the
  // objective a moving target and no two options would be comparable. The one
  // per-iteration component is livramento_alpha's α, which is a *multiplier on*
  // the frozen anchor, not a re-anchoring (see below).
  enum class TimingScale
  {
    // Design-median auto-scale (current): tw = timing_bias · l_med / t_med with
    // t_med = median_g(Σλ·d). NOTE t_med ∝ λ, hence tw ∝ 1/λ, hence tw·λ·d is
    // *exactly* invariant to a uniform rescale of λ - this option destroys λ's
    // magnitude by construction. rsz_baseline needs it precisely because its λ
    // has no meaningful magnitude to preserve; `timing_bias` (64) is the real
    // balance knob under it, and it is the confound every cross-paper
    // comparison shared before this axis existed.
    kAutoMedian = 0,
    // λ-free median anchor: tw = l_med / d_med with d_med = median_g(Σ d) - the
    // same median-gate anchor as auto_median with λ dropped out of it. `tw` no
    // longer depends on λ, so a uniform rescale of λ survives all the way into
    // candidate scoring, and λ acquires a literal reading: **λ is the
    // timing/leakage ratio on a median gate**. λ=12 ⇒ timing ≈ 12× leakage at
    // seed; λ=1 ⇒ ≈ 1×. `timing_bias` is orthogonal under this option and the
    // paper presets pin it to 1.0.
    //
    // This is NOT the plan's original literal `unit` (raw SI, tw = 1). That
    // reading was evaluated against the ISPD 2012/2013 contest library
    // (src/gpl/test/library/a2a/contest.lib, which the papers' λ constants were
    // calibrated on: time_unit 1ps, leakage_power_unit 1uW, median leakage 80
    // µW, arc delays ~12-300 ps - so λ=12 means 12 µW/ps and λ·d ≈ 4.5× a
    // median gate's leakage) and REJECTED: λ=12's content is a *ratio* that
    // library happened to realize, not a physical constant. Our smoke library's
    // median leakage is ~3000× smaller, so a contest-calibrated λ in SI
    // over-weights timing here by ~3000× - a pure timing minimizer, the mirror
    // image of tw = 1's ~260× leakage collapse. Both literal readings fail,
    // from opposite sides. The transferable content of "λ=12" is "the timing
    // term should be a few times the leakage term on a median gate", and that
    // is what this option implements. It self-checks on the contest library:
    // there l_med/d_med ≈ 80/30 ≈ 2.7 µW/ps, so Flach's 12 ≈ 4.5 natural units
    // - recovering the contest's own 4.5× balance. See the plan's "Item 3 - the
    // unit convention".
    kUnit = 1,
    // Livramento DATE'13 Alg. 1 L9's rescheduled weight: tw = base / α, with
    // `base` the λ-invariant auto_median anchor (see the ADAPTATION note below
    // for why it is that and not `unit`) and α re-derived from *fresh* arrivals
    // before every LRS sweep as
    //   α ← α · (A_o / max_j a_j)
    // seeded at livramento_alpha0. Livramento's α is on the POWER term (Eq. 1,
    // the reduced Lagrangian Eq. 7, Alg. 2 L10 - confirmed three ways), so on
    // our timing side it maps as **1/α**. It is a multiplicative feedback
    // controller with a fixed point at max_j a_j = A_o (i.e. WNS = 0): a
    // violating design shrinks α and so raises timing pressure, a design with
    // slack to spare grows α and so recovers leakage.
    //
    // The schedule IS the paper's contribution - its own Figs. 1-2 are the
    // ablation, and a static knob cannot express it at all. (Read those figures
    // carefully: the precise finding is that timing violations are never
    // brought under control while power sits flat, NOT "never converges"; the
    // no-schedule run ends at *lower* leakage precisely because it is cheating
    // on timing, which is why Alg. 1 L20 returns the best solution *without
    // violations*.)
    //
    // ADAPTATION - the base anchor is ours, not the paper's, and it is the
    // λ-INVARIANT one. Two facts force this:
    //   1. α₀ is genuinely unstated (Alg. 1 L6: "α ← initial positive value";
    //      §V gives no number) and the schedule is multiplicative, so α₀ never
    //      washes out - it is a permanent scale on the whole trajectory. Read
    //      literally with α₀ = 1 the objective is raw-SI `leakage + Σλd`, which
    //      the λ-magnitude finding predicts collapses to a pure leakage
    //      minimizer (~260× on our smoke library). So α must scale *something*.
    //   2. Livramento's λ init is equally unstated ("λ ← initial vector ∈ Ω_λ",
    //      values unspecified), and the seed this preset actually carries is
    //      OpenROAD's own delay-proportional one, whose λ is an arc delay in
    //      SECONDS. Its magnitude is a unit artifact, not a balance choice - so
    //      the base must divide it back out. `unit` would preserve the artifact
    //      and collapse the timing term by ~1e10 (measured). auto_median's
    //      anchor is λ-invariant by construction (t_med ∝ λ ⇒ tw ∝ 1/λ), which
    //      is exactly the property needed here.
    // So: base = timing_bias · l_med / t_med, α₀ = 1 ⇒ "start at the
    // median-gate balance and let the schedule take it from there", with the
    // preset pinning timing_bias = 12.0 (the auto_median/livramento_alpha paper
    // family's shared balance - livramento_alpha DOES read timing_bias, unlike
    // unit) rather than rsz_baseline's 64.
    //
    // Note α₀ and ‖λ₀‖ are **one degree of freedom, not two** (only the ratio
    // of α·p to λ·D has behavioral content) - but under a λ-invariant base ‖λ₀‖
    // is divided out anyway, so α₀ is the only live coordinate here. Fix it at
    // 1 and sweep it deliberately if the campaign wants that axis; sweeping
    // lambda_init_value would do nothing under this option.
    kLivramentoAlpha = 2,
  };
  TimingScale timing_scale = TimingScale::kAutoMedian;

  // === H1 - Termination =====================================================
  // Every option is bounded by max_iterations. Only kFixedIters carries
  // OpenROAD's legacy early-exits; the paper options carry their paper's rule
  // and nothing else, so each stays one clean concept.
  //
  // NO OPTION ON THIS AXIS STOPS BECAUSE TIMING IS MET, AND NONE GATES LOOP
  // ENTRY ON TIMING (pre-campaign engine fix 2, 2026-07-29). kFixedIters used
  // to stop before the first sweep once WNS met setup_slack_margin, which made
  // GLOBAL_SIZING a hard no-op on every design that arrived already meeting
  // timing - zero sweeps, zero moves - and that is the majority regime for the
  // campaign's power-recovery question. All eight trusted paper specs minimize
  // leakage/power/area SUBJECT TO timing, so a feasible input is a normal (for
  // reimann/mangiras/chinnery/tennakoon, the INTENDED) input, and none of them
  // stops merely because timing is met.
  //
  // Two options still READ the worst slack, neither as a stop, and the
  // distinction is the whole point: kThresholdBattery's WNS/TNS targets are
  // Chinnery's phase HANDOVER (timing phase -> power phase, the run continues),
  // and kStagnationWindows' near-met latch only ACTIVATES its monitor (sharma
  // §5.2). A third, kFixedIters' 3-consecutive-reject exit, is WNS-DERIVED
  // through the driver's wns_regressed measurement but fires only after a sweep
  // - see FixedItersTermination. Do not reintroduce a "timing met => stop" or a
  // "timing met => do not start" rule on this axis.
  enum class TerminationKind
  {
    // Fixed iteration cap plus the two non-timing early-exits (3 consecutive
    // rejections, 2 consecutive zero-move passes). Its WNS-meets-margin exit
    // was removed by the 2026-07-29 ruling; see the axis note above.
    kFixedIters = 0,
    // Sharma ICCAD'15 early-exit (sharma_et_al.md §5.2/§6): "terminate if
    // neither the average power, nor the minimum power solution found thus far,
    // improve during two consecutive sets of iterations", a set being 5
    // iterations. Parameterized by stagnation_window / stagnation_count /
    // stagnation_improve_frac / stagnation_require_tns, which also express
    // Mangiras' rule ("improvement of TNS *and* total leakage across two
    // consecutive iterations below 1%", mangiras_et_al.md §5) - same concept,
    // different constants: window 1, count 1, frac 0.01, require_tns on.
    // (window 1 IS "two consecutive iterations": a one-iteration window is
    // compared against its predecessor. window 2 would average over disjoint
    // 2-iteration blocks and compare block to block - a different rule that
    // first fires at iteration 4 and only on even iterations.)
    kStagnationWindows = 1,
    // Chinnery & Sharma ISPD'22 threshold battery (chinnery_et_al.md §5/§6).
    // The paper runs two sequential phases and each owns its own exits, so the
    // criteria are gated by phase rather than OR-ed together (see
    // BatteryPhase):
    //   timing improvement phase - ends when TNS is within
    //     term_tns_target_frac of the clock period, WNS within
    //     term_wns_target_frac of it, or TNS improvement falls below
    //     term_tns_improve_frac over the last term_improve_window iterations.
    //     Reaching any of these hands over to the power phase (RSZ-0450); it
    //     does not stop the run.
    //   power reduction phase - ends the run (RSZ-0451) when power improvement
    //     falls below term_power_improve_frac over that window, or when TNS
    //     degrades past its end-of-timing-phase value (the paper's second
    //     power-phase exit - a stop-loss that fired in 24-36% of its runs and
    //     which it names as a convergence deficiency, not a recipe).
    // The term_wall_limit_s wall-clock cap is a hard safety cap in either phase
    // (as is max_iterations, the paper's 80).
    //
    // ADAPTATION: the paper's phases also swap the multiplier update exponents
    // (§6, deferred to [7]); ours do not, so the phase is a termination regime
    // here and nothing more. That single deferral is the sharpest entry in
    // chinnery_partial's DEFERRED list (see applyPreset); it is why the preset
    // carries the suffix, not a reason for it not to exist.
    kThresholdBattery = 2,
    // Pure cap (C2): stops ONLY at max_iterations - neither the
    // consecutive-reject nor the zero-move exit. kFixedIters' two early-exits
    // are rsz_baseline's own tested ideas, not paper mechanisms, and (our
    // reading, not the audits') a consecutive-reject or zero-move stop still
    // truncates a paper schedule whose λ trajectory has not yet moved the
    // design - flach's k=4 leakage-recovery phase and livramento's α-controller
    // recovery half are both reached late. The pre-Flach and Flach-family paper
    // presets that carry no early-exit rule of their own select this so their
    // paper's schedule runs to completion
    // (chen/tennakoon/livramento/flach/reimann). rsz_baseline keeps
    // kFixedIters. The C2 audit anchors - chen §2.4, tennakoon §2.5, livramento
    // §2.2, flach §2.1, reimann §2.2 - are about the DELETED WNS exit; only
    // their trailing sentences cover the two move-driven exits this option
    // still buys.
    //
    // Historical note (2026-07-29): kFixedIters' THIRD legacy exit - stop once
    // WNS meets setup_slack_margin - was the reason reimann's entire method
    // exited at iteration 0 on a feasible input. It is gone from kFixedIters
    // too, so that half of the C2 argument is now MOOT rather than universal:
    // every option runs past first feasibility, and only the two move-driven
    // exits still distinguish kFixedIters from kPureCap.
    kPureCap = 3,
  };
  TerminationKind termination = TerminationKind::kFixedIters;

  // === H2 - Best-solution tracking ==========================================
  // The axis owns "which of the sweeps I ran survives", including the LR loop's
  // pass-level journal (see BestTracker). Its options are therefore mutually
  // exclusive by construction: two rules cannot both own the answer.
  //
  // NOTE (M5): the default is kFlachDominance, NOT a ` = current behavior`
  // option - the deliberate default-behavior flip the plan calls for (§3.2-H,
  // fixing gap §2.4.1: every LR paper keeps a best-so-far solution and restores
  // it, because the multiplier dynamics make late iterates non-monotone).
  // NOTE (post-M5 hardening): rsz_baseline pins kWnsPassReject, which is the
  // pre-M5 driver's inline best-WNS journal rule relocated onto this axis. It
  // used to run for EVERY preset from the shared driver, underneath whatever
  // paper rule the preset selected; no paper contains it.
  enum class BestTrackerKind
  {
    // No best tracking: every sweep is kept and the final iterate stands.
    kNone = 0,
    // Flach TCAD'14 Alg. 1 lines 9-13: a solution is "better" than the stored
    // one iff |TNS| < best_tns_target_frac * T AND its leakage is lower;
    // snapshot on every such iterate and restore it at the end of the run.
    kFlachDominance = 1,
    // Reimann ISPD'16 Eq. 6: exponential weighted score over the changes in
    // power, area, timing violation and WNS relative to the *input* solution;
    // snapshot whenever the score beats the best seen. Tolerates sub-picosecond
    // WNS noise when power/area improve, but scores real timing degradation
    // strongly negative.
    kReimannScore = 2,
    // rsz_baseline's rule (pre-M5 driver behavior): keep the netlist at the
    // last sweep whose WNS matched or beat every previous sweep's, via journal
    // checkpoints, and undo the drift past the last checkpoint at loop exit.
    // A regressing sweep also feeds the norm_subgradient updater's private
    // alpha halving - but that coupling runs off the driver's WNS measurement,
    // not off this option, so the two axes stay independent.
    kWnsPassReject = 3,
  };
  BestTrackerKind best_tracker = BestTrackerKind::kFlachDominance;

  // === Named presets ========================================================
  // NAMING (§3.3, applied by the bucket-1 fidelity pass): a preset missing any
  // component its paper specifies is suffixed `_partial`, and its doc block in
  // applyPreset() lists what is missing. A component that simply cannot be
  // evaluated identically under OpenSTA (continuous sizing, contest timer,
  // area-vs-leakage) is a documented *adaptation* and does NOT earn the suffix.
  //
  // The audit found EVERY paper preset has at least one deferred component, so
  // every one carries `_partial`. That is the honest state of the framework -
  // no preset reproduces its paper's complete method - not a labelling quirk.
  // Read the suffix as "do not attribute a campaign result to <paper>'s method
  // on the strength of this preset"; read the doc block for what is missing.
  // Only rsz_baseline is unsuffixed, because it is not a paper.
  enum class Preset
  {
    // The current OpenROAD LR sizer; every axis set to its ` = current
    // behavior` option. NOTE it is no longer bit-identical to the pre-ablation
    // engine: the two WNS-conditioned mechanisms (the end-of-phase accept and
    // fixed_iters' pre-sweep exit) were deleted from this bundle too, because
    // both made a met-timing design a no-op and the baseline is the column
    // every paper arm is scored against.
    kRszBaseline = 0,  // "rsz_baseline"
                       // Paper presets. Each selects its paper's axis options;
                       // every axis it does not name stays at the rsz_baseline
                       // option. Paper constants come from the struct defaults
                       // below, which already hold each paper's value.
    kChen = 1,        // "chen_partial": additive subgradient + constant(1) seed
    kTennakoon = 2,   // "tennakoon_partial": constant-free arrival-ratio
    kFlach = 3,       // "flach_partial": asymmetric slack scaling +
                      // constant(12) seed + cost_global_phi + veto + dominance
    kSharmaSeq = 4,   // "sharma_seq_partial": cexp criticality + constant(1)
                      // seed + veto + stagnation windows
    kReimann = 5,     // "reimann_partial": dWNS update + estimation-loop seed
    kMangiras = 6,    // "mangiras_partial": state-adaptive seed, Flach host
    kLivramento = 7,  // "livramento_partial": livramento-ratio update +
                      // cost_fanout_slew, baseline seed
    kChinnery = 8,    // "chinnery_partial": the two-phase threshold battery on
                      // the sharma-lineage updater, constant(1) seed, veto
  };
  // Informational: records which preset seeded this config. Individual knobs
  // set after a preset override its axis values.
  //
  // `preset` alone CANNOT answer "which preset was asked for", because its
  // default is a real preset name: a run that passed no `-preset` is
  // indistinguishable from one that passed `-preset rsz_baseline`. That
  // ambiguity is invisible in the code and load-bearing in the harness - the
  // RSZ-0417 echo is the ablation campaign's per-run proof that the intended
  // cell took effect, and a preset-less run claiming `preset=rsz_baseline`
  // reads as a control arm. It is not even true of the axes: the struct
  // defaults are *almost* the rsz_baseline bundle, but best_tracker defaults
  // to the paper value (see applyPreset), so a preset-less run does not carry
  // rsz_baseline's tracker either.
  //
  // Provenance flag, the mu_policy_explicit idiom. Unlike that one it is set
  // inside applyPreset rather than by the dbProperty reader, because
  // applyPreset IS the only way a preset arrives; the echo prints `unset` when
  // it is false (GlobalSizingPolicy::logEffectiveConfig). No engine code
  // branches on it - it is a record, not an axis.
  Preset preset = Preset::kRszBaseline;
  bool preset_explicit = false;

  // === Shared scalar knobs ==================================================
  // Optional clock network sizing: Global sizing excludes clock network
  // instances by default. Can be enabled for post-CTS timing repair for better
  // clock performance.
  bool include_clock_network = false;
  float setup_slack_margin = 0.0f;
  int max_iterations = 20;
  // Step size α for the dual-subgradient update on λ.
  //   λ_e ← max(floor, λ_e · (1 + α · g_e_norm))
  // with g_e_norm ∈ [-1, 0]. Tight arcs (g=0) are unchanged; arcs at full
  // slack (g=-1) shrink to (1-α)·λ. Halved on pass rejection.
  float beta = 0.6f;
  // Endpoint seed exponent: mu_k ~ max(0, margin - slack_k)^p.
  float mu_exponent = 2.0f;
  // Floor for multipliers (subgradient floor so unused arcs can re-enter).
  float lambda_floor = 1e-12f;
  // Dimensionless balance between timing pressure and leakage cost.
  // bias = 1.0 keeps Σλ·d (scaled) ≈ leakage cost on the median gate.
  //
  // READ BY timing_scale = auto_median AND livramento_alpha, whose shared
  // non-λ-free anchor is tw = timing_bias·l_med/anchor_med
  // (TimingScale.cc:92-93; livramento_alpha then divides by its α).
  // rsz_baseline owns the 64, and the auto_median/livramento_alpha paper family
  // (tennakoon, reimann, livramento) pins 12.0 - its shared balance, NOT 1.0.
  // Only under `unit` is the knob inert (lambda_free: λ itself is the
  // median-gate timing/leakage ratio), and there the unit presets (flach,
  // sharma, chen, mangiras) pin 1.0 to say so.
  float timing_bias = 64.0f;
  // Safety derate (<= 1) on the per-gate distributed downsize budget. The
  // depth-normalized distribution already guarantees per-path budget sums
  // <= path slack, so 1.0 is feasible in theory; a value < 1 adds margin for
  // the un-modeled slew cascade / estimated-vs-routed parasitic gap.
  float budget_safety_factor = 1.0f;
  // Asymmetric acceptance deadband on the LR-cost improvement (acceptGateMove):
  // an UPSIZE must beat the incumbent cost by this fraction to be accepted; a
  // downsize needs any improvement at all. It filters LR-cost noise that would
  // otherwise churn the design without a meaningful timing win.
  //
  // NOT A PAPER MECHANISM (hardening finding #10). Every paper's LRS takes the
  // plain argmin over the candidate set - Flach Alg. 3, Sharma Eq. 5,
  // Livramento Alg. 1, Chen SOLVE_LRS/μ - with no acceptance tolerance of any
  // kind. It was a shared, unconditional 0.02 constant in the sweep engines and
  // therefore rode every preset; the bucket-2 pass promoted it to this knob so
  // the OpenROAD heuristic is a knob rather than a hidden term in every paper's
  // objective. 0.02 is the struct default (== rsz_baseline, which owns it);
  // every paper preset pins 0.0, restoring the plain argmin.
  float upsize_hysteresis = 0.02f;

  // === D paper seed constants (defaults = each paper's value) ===============
  // constant seed: lambda on every data arc. Flach's contest value is 12;
  // Sharma states 1; Chen states only "an arbitrary initial vector".
  //
  // LIVE ONLY as a constant seed under mu_policy = endpoint_lambda/
  // endpoint_additive AND timing_scale = unit (the item-5 constant-seed
  // presets: flach, sharma_seq, chen). It was INERT under the old bundles
  // (reseed_each_iter + auto_median), and the post-M5 hardening audit recorded
  // exactly that: the mu-anchored projection cancels a uniform seed c
  // (c*target/(m*c) = target/m), and auto_median's t_med ∝ λ then divides out
  // any residual global scale (tw ∝ 1/λ), so flach(12) and sharma/chen(1) once
  // ran the IDENTICAL field. C1-C4 changed both halves: endpoint_lambda makes
  // the projection positively homogeneous (the seed magnitude is preserved, not
  // cancelled - TestFlowProjection homogeneity pins) and unit's tw is λ-free
  // (the magnitude reaches scoring - TestTimingScale). Under those bundles
  // flach(12) and chen/sharma(1) now run DIFFERENT fields and this IS a live
  // ablation knob. It stays inert under rsz_baseline (delay-proportional seed +
  // auto_median) and any auto_median/reseed leg - the Pre-M7 legality table's
  // "Live only as constant seed + endpoint_lambda + unit" (flach audit §2.4).
  float lambda_init_value = 12.0f;
  // Mangiras exponent K in Eqs. 5-6 (sharpens the initial lambda spread).
  // Paper sets K = 2; K = 1 converges slower, K = 3/4 give no further gain.
  float lambda_seed_exponent = 2.0f;
  // Reimann estimation-loop (Alg. 2 loop 1) iteration count. The paper only
  // says "a few" and that it differs from the main-loop cap; 3 is a small
  // default (the main loop then starts warm).
  int est_loop_iters = 3;

  // === E1/E2 paper update constants (defaults = each paper's value) =========
  // Chen additive step scale: rho_k = lambda_update_c / k, applied to the
  // arc violation normalized by the clock period (see chenSubgradientLambda -
  // the paper leaves rho_k free, and stepping in raw SI seconds is a float32
  // no-op against an O(1) lambda). Dimensionless and library-portable, so 1.0
  // is a real default rather than a per-library calibration.
  float lambda_update_c = 1.0f;
  // Flach k-schedule (Alg. 2 / §5): k starts at k_init, rises to k_tns_small
  // once timing is near-feasible (leakage drops faster), and falls back to
  // k_final (<=1) in the final iterations to squeeze out residual violations.
  float flach_k_init = 1.0f;
  float flach_k_tns_small = 4.0f;
  float flach_k_final = 1.0f;
  // Sharma cexp schedule (Fig. 2): relaxed target r*T and shrink rate k.
  float sharma_r = 1.01f;
  float sharma_k = 10.0f;
  // Reimann update (Alg. 3): rho_inc = rho_init*(1+iter),
  // rho_dec = rho_init*(15+iter). The exponent k follows the quality-driven
  // schedule of Eq. 7: k_est during the estimation loop, k_lo (<1) when timing
  // degraded, k_hi (>1) when it improved, reimann_k (=1, neutral) otherwise.
  // The paper leaves the <1 / >1 ranges empirical ("do not affect final
  // quality, only convergence speed"); these defaults are our choice.
  float reimann_rho_init = 0.05f;
  float reimann_k = 1.0f;
  float reimann_k_est = 5.0f;
  float reimann_k_lo = 0.5f;
  float reimann_k_hi = 2.0f;
  // Reimann servo setpoint (C1). Alg. 3 references each arc's FROZEN INITIAL
  // slack S_init - the paper's servo holds the LRS near the INPUT timing (its
  // §7 power-recovery regime, where input violations are deliberately NOT
  // chased). In OpenROAD's usual improve-from-violating flow that reference
  // sits above every subsequent slack, so the increase branch never fires and
  // the field decays to the floor (reimann audit §2.1).
  enum class ReimannSetpoint
  {
    // Faithful: reference = the per-arc frozen initial slack S_init (the
    // paper).
    kSInit = 0,
    // NON-PAPER ADAPTATION (disclosed): reference = the slack TARGET (the
    // setup_slack_margin), i.e. S_init := margin. The setpoint becomes
    // "feasibility" instead of "input timing", so the increase branch fires on
    // violating arcs (magnitude |slack|/(max(ΔWNS,0.1T)·ρ_inc), ΔWNS the live
    // violation depth) and every Reimann mechanism (ρ schedules, Eq. 7
    // k-schedule, asymmetric exponents) stays live with the paper's own gain
    // control. This is NOT Reimann's method - the S_init reference IS the
    // paper's point - so a run under it may never be attributed to "Reimann";
    // kSInit stays the default so an M7 regime-aligned run can use the faithful
    // reading. See reimann audit §2.1 fix (ii) / §3(f).
    kSlackTarget = 1
  };
  ReimannSetpoint reimann_setpoint = ReimannSetpoint::kSInit;
  // Livramento alpha schedule seed (Alg. 1 L6, "α ← initial positive value").
  // Only read by timing_scale = livramento_alpha. The paper states no value and
  // it is not recoverable from the corpus: Livramento credits the schedule to
  // Tennakoon-Sechen TCAD 2008, which is NOT the Tennakoon-Sechen paper in
  // papers/ (that is ICCAD'02 Forge, a different paper). 1.0 is our choice, and
  // under this option's λ-INVARIANT (auto_median) base it means "start at the
  // median-gate balance". Because α₀ and the λ-seed magnitude are one degree of
  // freedom AND the λ-invariant base divides ‖λ₀‖ out anyway, α₀ is the only
  // live coordinate: sweep THIS field to explore the axis. Sweeping
  // lambda_init_value does nothing under this option (contra any older gloss -
  // see the kLivramentoAlpha enum doc, whose ADAPTATION note is authoritative).
  float livramento_alpha0 = 1.0f;

  // === F3 guard constant ====================================================
  // Hill-climbing tolerance of the local-slack veto. Flach's Eq. 14 is
  //   gamma = 1 + (-min(0, WNS) / T)
  // - it exceeds 1 while the design violates (the veto then tolerates bounded
  // local degradation, letting the sweep climb hills) and decays to 1 as the
  // design converges (degradation is then forbidden). This knob scales the
  // hill-climbing part:
  //   gamma = 1 + gamma_local_slack * (-min(0, WNS) / T)
  // so 1.0 (default) is exactly Eq. 14, 0.0 forbids any local degradation from
  // the first iteration, and > 1 is more permissive than the paper.
  float gamma_local_slack = 1.0f;

  // === H1 termination constants (defaults = each paper's value) =============
  // Sharma early exit: a "set" of iterations (window), how many consecutive
  // stagnant windows end the run, and what counts as an improvement. The paper
  // says "do not improve", i.e. any improvement at all resets the count
  // (frac = 0); Mangiras' 1%-over-2-iterations rule is the same rule with
  // window = 1, count = 1, frac = 0.01 and the TNS clause on - a window of 1
  // compares each iteration against its predecessor, which is what "across two
  // consecutive iterations" means. A window of w > 1 averages over disjoint
  // w-iteration blocks and compares block to block (Sharma's reading, w = 5).
  int stagnation_window = 5;
  int stagnation_count = 2;
  float stagnation_improve_frac = 0.0f;
  // When true a window only counts as stagnant if TNS *also* failed to improve
  // by stagnation_improve_frac (Mangiras: "TNS and total leakage"). When false
  // the test is Sharma's power-only rule.
  bool stagnation_require_tns = false;
  // Near-met phase gate (C2). The driver latches a run "near-met" once its
  // worst slack reaches within near_met_gate_frac of the setup target - i.e.
  // WNS >= -near_met_gate_frac * T - and the latch stays set for the rest of
  // the run (LrState::near_met). Two consumers gate on it: the
  // stagnation_windows monitor (inactive until near-met) and the
  // local_slack_veto guard (passes every candidate until near-met). Both
  // express Sharma's "we apply this check as we recover power, after the design
  // timing is within 1% of the target" (sharma_et_al.md §5.2, one constant for
  // both - sharma audit §2 items 3/6). A NEGATIVE value (the default) DISABLES
  // gating: the run is near-met from iteration 0, so the monitor and veto are
  // always active - which is what keeps mangiras' window rule (mangiras audit
  // §1, correct ungated) and flach/reimann's veto unchanged.
  // sharma_seq_partial pins it at the paper's 0.01 (1%).
  float near_met_gate_frac = -1.0f;
  // Chinnery threshold battery: the timing targets are relative to the clock
  // period T (TNS within 10% of T, WNS within 1% of it); the improvement
  // thresholds are relative improvements over the last term_improve_window
  // iterations (TNS < 10%, power < 1%); the wall-clock cap is the paper's 72 h.
  // (Its 80-iteration cap is just max_iterations.)
  float term_tns_target_frac = 0.10f;
  float term_wns_target_frac = 0.01f;
  float term_tns_improve_frac = 0.10f;
  float term_power_improve_frac = 0.01f;
  int term_improve_window = 3;
  float term_wall_limit_s = 259200.0f;

  // === H2 best-tracker constant =============================================
  // Flach's dominance gate: an iterate may only displace the stored best when
  // |TNS| < best_tns_target_frac * T ("TNS is less than 10% of T").
  float best_tns_target_frac = 0.10f;

  // Set every axis field to the given preset's bundle. Individual knobs read
  // from dbProperties after this call override their axis values.
  void applyPreset(Preset p);

  // Resolve the lambda <-> mu pairing (iteration-2 plan §2.2-6, ruling #2)
  // before validate() runs. Call once, on the frozen per-run copy of the
  // config.
  //
  // THE FINDING THIS EXISTS FOR (S1-E, engine-verified on the iteration-1
  // pilot): every paper lambda-update rule is ANNIHILATED by the flow
  // projection under any mu policy that does not read lambda. The three
  // slack-seeded policies (reseed_each_iter, seed_once, update_as_lambda)
  // anchor the projection to a mu derived from a slack field, so whatever the
  // updater did to lambda's magnitude is rescaled away at the endpoint boundary
  // and redistributed identically - chen, tennakoon and livramento came back
  // BYTE-IDENTICAL as collected (S1 §3.1, PREREG C-8). The lambda axis is not
  // measurable at all under them, which is an identifiability failure, not a
  // result.
  //
  // So: a non-baseline lambda rule with no explicitly chosen mu policy and no
  // lambda-reading one in the bundle auto-selects endpoint_lambda, with an
  // RSZ-0447 warn stating the pairing. An explicit mu_policy is always honored;
  // when the explicit combination is one of the annihilated ones, RSZ-0448 says
  // so at info level and nothing is blocked - "measure the annihilation" is a
  // legitimate cell, it just must not be the accidental default.
  //
  // norm_subgradient is untouched: it is rsz_baseline's own rule, its magnitude
  // has no paper meaning to preserve, and auto-moving the baseline's mu policy
  // would move the denominator of the whole ablation.
  void resolveLambdaMuPairing(utl::Logger* logger);

  // Enforce cross-axis compatibility constraints (SYNTHESIS §5). A hard
  // violation raises an RSZ error (logger->error is [[noreturn]], aborting the
  // phase loudly with the cited constraint); a soft violation emits an RSZ
  // warning. Returns true when no hard constraint fired (the return value lets
  // start() bail without relying on the throw). Called once from
  // GlobalSizingPolicy::start(). Active rules: state_adaptive requires
  // init_mode = as_given (hard, §5.3); estimation_loop warns unless as_given;
  // cost_global_phi and cost_delta_delay are mutually exclusive (hard, §5.6:
  // two different global-effect estimators cannot both drive the cost); the
  // Jacobi engine ignores traversal / gs_refresh, so setting either to a
  // non-default value under jacobi_snapshot warns (soft); cost_global_phi with
  // cost_fanout_slew double-prices the immediate sink level (soft); and the two
  // guard/engine crosses - {jacobi + local_slack_veto} and {gauss_seidel +
  // depth_budget} - are allowed but flagged (soft, §3.2-F).
  bool validate(utl::Logger* logger) const;
};

// Every Preset value, in declaration order. Iterate THIS in an "every preset
// must ..." invariant instead of spelling the list out: a hand-written list
// silently stops covering the next preset that lands, and several of those
// invariants are what keep an OpenROAD-baseline mechanism (the upsize deadband,
// the WNS pass-reject tracker) out of a paper column. A bundle table that maps
// each preset to an expected value cannot iterate this, but it can assert its
// own length against it.
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

// Enum <-> Tcl/report string helpers (used by the config header echo and the
// dbProperty reader). Return "unknown" / kRszBaseline on an unrecognized value.
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
const char* toString(GlobalSizingConfig::Preset preset);
// Parse a preset name; returns false on an unrecognized name.
bool parsePreset(const char* name, GlobalSizingConfig::Preset& out);
// Parse an axis-option name; returns false on an unrecognized name.
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

}  // namespace rsz
