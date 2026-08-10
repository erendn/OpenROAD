// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include "LrState.hh"

namespace rsz {

struct GlobalSizingConfig;

// H1 axis - termination rules. The driver runs iterations 0..maxIterations()-1;
// stopBeforeSweep() is queried at the top of each iteration (before the λ
// update
// + sweep) and stopAfterSweep() after the sweep, its STA update and the
// driver's pass verdict. Each returns true to stop; the strategy logs its own
// reason at debug level 1.
class Termination
{
 public:
  virtual ~Termination() = default;
  // Whether stopAfterSweep() reads `metrics`. False for fixed_iters, so the
  // driver can skip the per-iteration design walk that fills them.
  virtual bool needsMetrics() const { return false; }
  // Iteration cap (config.max_iterations, or the struct default if <= 0).
  int maxIterations(const LrState& state) const;
  // Pre-sweep hook. Base returns false and NO shipped option overrides that
  // verdict: the axis has no pre-sweep stop, by the 2026-07-29 ruling (see
  // GlobalSizingConfig::TerminationKind). kFixedIters used to stop here once
  // WNS met setup_slack_margin, which fired at iteration 0 on any design that
  // arrived meeting timing - the loop ran ZERO sweeps and GLOBAL_SIZING was a
  // hard no-op, measured on recover_power1 @ 2.0 ns (WNS +1.760) and
  // repair_setup2 @ 0.60 ns (WNS +0.222) as "0/0 sweeps accepted, 0 cells
  // replaced". Since kFixedIters is the only shipped preset's option
  // (rsz_baseline), that silently zeroed the campaign's CONTROL column.
  //
  // Kept as a hook, and kept returning bool, for per-iteration bookkeeping
  // (kThresholdBattery stamps its wall clock at iter 0) and so a future
  // NON-timing pre-sweep rule - the battery's wall-clock cap is the obvious
  // candidate - remains expressible. Do not reintroduce a timing-conditioned
  // one.
  virtual bool stopBeforeSweep(LrState& /* state */, int /* iter */)
  {
    return false;
  }
  virtual bool stopAfterSweep(LrState& state,
                              int iter,
                              bool reject,
                              bool no_benefit,
                              const IterMetrics& metrics)
      = 0;
  // Called once by the driver after the LR loop, whether the loop ended on this
  // strategy's verdict or on max_iterations. Exists so an option that PUBLISHES
  // a run record can publish it on every path: a record emitted only from the
  // stop verdict is missing exactly on the runs that did not converge, and
  // "hit the cap" then looks like "record dropped". Default: say nothing.
  virtual void reportRunEnd(LrState& /* state */) {}
};

// Fixed iteration cap plus OpenROAD's two legacy early-exits: 3 consecutive
// rejections or 2 consecutive zero-move passes. Both are checked AFTER a sweep,
// so neither can no-op a met design a priori - unlike the pre-sweep
// WNS-meets-margin exit this class used to carry, which never let the first
// sweep run at all (removed 2026-07-29; see Termination::stopBeforeSweep). The
// paper options below carry only their paper's rule, so each option stays one
// concept.
//
// HONEST NOTE on the 3-reject exit: it is still WNS-DERIVED. Its `reject` input
// is the driver's `wns_regressed` measurement (GlobalSizingPolicy.cc), so on a
// met design where recovery spends positive slack it can end a run after three
// sweeps. It is the in-loop analogue of the deleted end-of-phase accept and it
// pairs with best_tracker=wns_pass_reject, which rolls those same passes back.
// That pairing is a recorded open decision (plan, engine fix 2 §4): it was NOT
// removed because it is a named H2/H1 ablation cell rather than an entry or
// termination gate, and it could not be reproduced on any benchmark in the
// suite (on recover_power1 at every clock from 2.0 ns down to 0.26 ns the
// downsizes REDUCE driver load and WNS rises, so nothing is ever rejected). The
// zero-move exit is genuinely move-driven and cannot fire before a sweep has
// run.
class FixedItersTermination : public Termination
{
 public:
  bool stopAfterSweep(LrState& state,
                      int iter,
                      bool reject,
                      bool no_benefit,
                      const IterMetrics& metrics) override;

 private:
  int consec_reject_ = 0;
  int consec_zero_ = 0;
};

// Sharma's early exit (sharma_et_al.md §5.2): "terminate if neither the average
// power, nor the minimum power solution found thus far, improve during two
// consecutive sets of iterations", a set being 5 iterations. Generalized by the
// config constants (stagnation_window / _count / _improve_frac / _require_tns),
// which also express Mangiras' rule - TNS *and* leakage improving by less than
// 1% across two consecutive iterations - with window 1, count 1, frac 0.01 and
// the TNS clause on. Window 1 compares each iteration against its predecessor
// (Mangiras); window w > 1 averages disjoint w-iteration blocks and compares
// block to block (Sharma's "sets", w = 5). Same mechanism, different readings -
// do not collapse one into the other.
//
// Near-met gate (C2): when near_met_gate_frac >= 0 the monitor is inactive
// (returns false, accumulates nothing) until the driver latches LrState::
// near_met - Sharma frames the early exit for the "timing almost met" regime,
// and an ungated monitor stops a still-closing run at 3*window iterations
// mid-closure (sharma audit §2 item 3). The default (frac < 0) leaves near_met
// set from iteration 0, so Mangiras' always-on window rule is unchanged.
class StagnationWindowsTermination : public Termination
{
 public:
  bool needsMetrics() const override { return true; }
  bool stopAfterSweep(LrState& state,
                      int iter,
                      bool reject,
                      bool no_benefit,
                      const IterMetrics& metrics) override;

 private:
  // Accumulators over the window in progress.
  float leak_sum_ = 0.0f;
  float tns_sum_ = 0.0f;
  int in_window_ = 0;
  // Previous window's averages (0 = no previous window yet).
  float prev_leak_avg_ = 0.0f;
  float prev_tns_avg_ = 0.0f;
  bool have_prev_ = false;
  // Min-power memory: the best leakage ever seen, and its value at the start of
  // the window in progress (so we can ask whether THIS window improved it).
  float best_leak_ = 0.0f;
  float best_leak_at_window_start_ = 0.0f;
  bool have_best_ = false;
  int stagnant_windows_ = 0;
};

// Pure cap (C2): the paper cell for presets that carry no early-exit rule of
// their own. Never stops before max_iterations - it reads nothing and holds no
// state. What it opts out of is kFixedIters' consecutive-reject and zero-move
// exits, which are rsz_baseline's own tested ideas (framing decision 1) and can
// still truncate a paper schedule before its λ trajectory has moved the design
// - the move-driven half of the C2 audit items (chen §2.4, tennakoon §2.5,
// livramento §2.2, flach §2.1, reimann §2.2). The WNS-exit half of those same
// items was closed differently, by deleting the exit on 2026-07-29, so it is no
// longer part of what this option buys. Wired to chen/tennakoon/livramento/
// flach/reimann; rsz_baseline keeps kFixedIters.
class PureCapTermination : public Termination
{
 public:
  bool stopAfterSweep(LrState& /* state */,
                      int /* iter */,
                      bool /* reject */,
                      bool /* no_benefit */,
                      const IterMetrics& /* metrics */) override
  {
    return false;
  }
};

// Which of Chinnery's two phases the loop is in (§5: "a timing improvement
// phase first, then a power reduction phase"). The phase selects which exit
// criteria are live; a timing-phase exit hands over to the power phase rather
// than ending the run, and only a power-phase exit (or the wall-clock cap) ends
// it.
//
// ADAPTATION: in the paper the phase ALSO swaps the multiplier update exponents
// (4/1 for critical/non-critical during timing, 1/4 during power recovery -
// chinnery_et_al.md §6, deferred to [7]). Our loop does not, so here the phase
// is purely a termination regime. That is the "flattened onto our single-phase
// loop" note on TerminationKind::kThresholdBattery.
enum class BatteryPhase
{
  kTiming = 0,
  kPower = 1,
};

// Chinnery's exit conditions. Each belongs to exactly one phase (§5) except the
// wall-clock cap, which is a hard safety cap on the whole run.
enum class StopReason
{
  kNone,
  kTnsTarget,    // timing phase: TNS within term_tns_target_frac of the period
  kWnsTarget,    // timing phase: WNS within term_wns_target_frac of it
  kTnsStall,     // timing phase: TNS improved < term_tns_improve_frac / window
  kPowerStall,   // power phase: power improved < term_power_improve_frac/window
  kTnsDegraded,  // power phase: TNS worse than at the end of the timing phase
  kWallClock,    // either phase: term_wall_limit_s exceeded
  // REPORTING ONLY - never returned by thresholdBatteryStop. The loop ended on
  // max_iterations (the paper's 80) with no battery criterion satisfied, i.e.
  // the run did not converge. It is in this enum rather than a separate string
  // so RSZ-0451's reason field has one vocabulary for a parser to read.
  kIterationCap,
};

// Chinnery's threshold battery (chinnery_et_al.md §5/§6). Runs the paper's two
// phases as termination regimes: the timing-improvement phase's exits hand over
// to the power-reduction phase, whose exits end the run. See BatteryPhase and
// StopReason.
//
// The phase boundary is REPORTED, not only traced: RSZ-0450 records the
// handover (which criterion fired, at which iteration) and RSZ-0451 the stop.
// The paper's own headline convergence figure is the split - "averaged 23
// iterations: 13 timing-phase + 10 power-phase" (§5) - so a run of this option
// that reports only a total sweep count cannot be compared with it, and the
// campaign harness parses these records for exactly that split.
class ThresholdBatteryTermination : public Termination
{
 public:
  bool needsMetrics() const override { return true; }
  bool stopBeforeSweep(LrState& state, int iter) override;
  bool stopAfterSweep(LrState& state,
                      int iter,
                      bool reject,
                      bool no_benefit,
                      const IterMetrics& metrics) override;
  void reportRunEnd(LrState& state) override;

 private:
  // The run record both exit paths share: total iterations, split by phase, and
  // the reason. `metrics` is the last iteration's.
  void reportStop(LrState& state,
                  StopReason reason,
                  const IterMetrics& metrics);

  // Per-iteration history; the improvement rules compare against the entry
  // term_improve_window iterations back.
  std::vector<float> tns_history_;
  std::vector<float> leak_history_;
  // Latches to kPower on the first timing-phase exit and never returns (the
  // paper's phases are sequential, not interleaved).
  BatteryPhase phase_ = BatteryPhase::kTiming;
  // TNS as the timing phase ended - the reference of the paper's SECOND
  // power-phase exit (kTnsDegraded). Only meaningful once phase_ is kPower.
  float tns_at_handover_ = 0.0f;
  // Sweeps run, and the 0-based iteration the handover fired on (-1 = never),
  // so the run record can report the paper's phase split rather than a total
  // the reader has to attribute by hand.
  int iters_run_ = 0;
  int handover_iter_ = -1;
  // Whether reportStop already published this run's record, so the end-of-loop
  // hook only speaks for a run that ended on max_iterations.
  bool reported_ = false;
  // The last iteration's metrics, for the cap-exit record (which has no
  // stopAfterSweep call of its own to carry them).
  IterMetrics last_metrics_;
  // Set at iteration 0 (stopBeforeSweep), so term_wall_limit_s measures the LR
  // loop rather than the setup that precedes it.
  std::chrono::steady_clock::time_point start_;
};

// === Pure cores (unit-tested against hand-computed values) ==================

// Relative improvement of a lower-is-better metric from `prev` to `cur`:
// (prev - cur) / |prev|, so 0.03 means "3% better". Negative when it got worse.
// Returns 0 when prev is 0 (no scale to be relative to).
float relImprovement(float prev, float cur);

// The C2 near-met phase latch's transition rule (the driver applies it each
// iteration and stores the result in LrState::near_met). A run is near-met once
// its worst slack `wns` reaches within `gate_frac` of the setup target, i.e.
// wns >= -gate_frac * T (Sharma's power-recovery gate, sharma_et_al.md §5.2:
// begins "after the design timing is within 1% of the target"). Once latched it
// stays latched (`latched` in => true out) - the phase is permanent for the
// run. A NEGATIVE gate_frac disables gating: the run is near-met from the start
// (the ungated default that keeps mangiras' window rule and flach/reimann's
// veto always-on). T <= 0 (no clock) leaves a not-yet-latched run un-latched:
// there is no target to be near. The per-run reset is LrState::allocate()
// clearing near_met to false.
bool nearMetLatched(bool latched, float gate_frac, float T, float wns);

// Sharma's window test: the window is stagnant iff NEITHER the window-average
// power improved (vs. the previous window's average) NOR the best-so-far power
// improved during the window, each by more than `frac`. `require_tns` adds
// Mangiras' clause: the window-average TNS must also have failed to improve.
// `have_prev` is false for the first window, which is never stagnant (there is
// nothing to compare against).
bool stagnantWindow(bool have_prev,
                    float prev_leak_avg,
                    float cur_leak_avg,
                    float best_leak_before,
                    float best_leak_now,
                    float frac,
                    bool require_tns,
                    float prev_tns_avg,
                    float cur_tns_avg);

// The battery, evaluated for `phase` only. `tns`/`wns` are the current
// (post-sweep) values, `T` the clock period; `tns_window_ago` /
// `leak_window_ago` are the values term_improve_window iterations back and are
// only consulted when `have_window` (enough history); `tns_at_handover` is the
// TNS as the timing phase ended and is read only in kPower (pass anything in
// kTiming - there is no handover yet). All fracs are the config constants.
//
// A non-kNone return means "this phase's exit criterion fired": in kPower (and
// for kWallClock in either phase) that ends the run; in kTiming it means the
// timing phase is over and the caller hands over to kPower. The phase gating is
// what keeps the power-stall clause from firing while the design is still
// closing timing - Chinnery's power exits are power-phase criteria, and testing
// them against a design that is still in timing improvement measures the LR
// loop's leakage plateau, not convergence.
StopReason thresholdBatteryStop(BatteryPhase phase,
                                float tns,
                                float wns,
                                float T,
                                float leakage,
                                bool have_window,
                                float tns_window_ago,
                                float leak_window_ago,
                                float tns_at_handover,
                                float elapsed_s,
                                const GlobalSizingConfig& config);

const char* toString(StopReason reason);

// The Lagrangian value L(x, λ) at the CURRENT sweep's assignment (debug
// diagnostic only - no control decision reads it). It is reported in the
// level-1 trace as `lag=`. Under the KKT flow conditions the projection
// enforces (Σλ_in = Σλ_out internally, Σλ_in(k) = μ_k at endpoints), every
// arrival-time variable cancels out of the Lagrangian (Chen Lemma 1,
// chen_et_al.md §6), so this reduces to the sweep's own cost minus the endpoint
// required-time constant:
//
//   L(x, λ) = leakage(x) + Σ_e λ_e·d_e(x) - Σ_k μ_k·r_k
//
// evaluated in the sweep's own cost units, i.e. with the auto timing weight
// applied to the multiplier terms (the sweep minimizes leakage + tw·Σλ·d, so tw
// scales λ).
//
// NAMED `lagrangianEstimate`, NOT `dualEstimate`, because it is NEITHER the
// Lagrangian dual Q(λ) NOR a bound on the optimum (chen audit §5.6 / §4.1), for
// three reasons: (1) it is evaluated at the *greedy sweep's* x, not at the true
// argmin of the discrete separable subproblem that Q(λ) = min_x L(x, λ)
// requires, so it over-estimates Q; (2) even the true Q(λ) ≤ primal rests on
// Chen's continuous convex (posynomial) relaxation, and NLDM delays over a
// discrete cell library are neither convex nor posynomial, so no strong duality
// holds; (3) `leakage` here is the design's Liberty leakage, while the sweep's
// cost uses LRSubproblem::leakageOrArea, which substitutes scaled area on a
// library that publishes no leakage — on such a library the primal term is 0
// and only the multiplier terms carry information. Read it as a convergence
// trace, not a certificate: primal - L is approximately -tw·Σ_k μ_k·slack_k, so
// it tracks the μ-weighted violation and should shrink as the multipliers
// converge; it can go negative once timing is met.
//
// F3 caveat for any future consumer (chen audit §4.1): a gap-based stop
// (primal - L below a tolerance) is NOT sufficient on its own, because the
// min-area primal can report a tiny gap while still violating timing — a
// gap-based stop must also check primal feasibility (WNS ≥ margin) before
// declaring convergence.
float lagrangianEstimate(float leakage,
                         float timing_weight,
                         float lambda_delay_sum,
                         float mu_required_sum);

// Main-thread STA walk feeding lagrangianEstimate: Σ_e λ_e·d_e over the data
// arcs and Σ_k μ_k·r_k over the endpoints. Only called when the level-1 trace
// is on.
struct LagrangianTerms
{
  float lambda_delay_sum = 0.0f;
  float mu_required_sum = 0.0f;
};
LagrangianTerms computeLagrangianTerms(LrState& state);

std::unique_ptr<Termination> makeTermination(const GlobalSizingConfig& config);

}  // namespace rsz
