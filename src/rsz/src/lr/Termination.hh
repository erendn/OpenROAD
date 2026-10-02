// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "LrState.hh"

namespace rsz {

struct GlobalSizingConfig;

// Termination rule of the LR loop. The driver runs iterations
// 0..maxIterations()-1. stopBeforeSweep() is called at the top of each
// iteration (before the λ update and the sweep) and stopAfterSweep() after the
// sweep, its STA update and the driver's accept/reject decision. Each returns
// true to stop; the strategy logs its own reason at debug level 1.
class Termination
{
 public:
  virtual ~Termination() = default;
  // Whether stopAfterSweep() reads `metrics`. False for fixed_iters, so the
  // driver can skip the per-iteration design walk that fills them.
  virtual bool needsMetrics() const { return false; }
  // Iteration cap (config.max_iterations, or the struct default if <= 0).
  int maxIterations(const LrState& state) const;
  // Pre-sweep hook. No strategy stops here. It exists for per-iteration
  // bookkeeping (ThresholdBatteryTermination starts its wall clock at
  // iteration 0) and for a possible non-timing pre-sweep rule such as a
  // wall-clock cap. Do not add a timing-based stop here: a rule such as "stop
  // once WNS meets setup_slack_margin" fires at iteration 0 on a design that
  // already meets timing, so no sweep runs and the sizer does nothing.
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
  // Whether the run is in a power-recovery phase, as decided by the last
  // stopAfterSweep(). The driver reads it after each stop check and publishes
  // it in LrState::power_phase. Only ThresholdBatteryTermination has phases;
  // the default is false.
  virtual bool inPowerPhase() const { return false; }
  // Called once by the driver after the LR loop, whether the loop ended on this
  // strategy's verdict or on max_iterations. A strategy that logs a run summary
  // uses it to log one for runs that hit the cap too; otherwise the summary
  // would be missing exactly for the runs that did not converge. Default: log
  // nothing.
  virtual void reportRunEnd(LrState& /* state */) {}
};

// The default rule: a fixed iteration cap plus two early exits, 3 consecutive
// rejected passes or 2 consecutive passes without moves. Both are checked
// after a sweep, so neither can stop a design that already meets timing before
// the first sweep. The paper-specific strategies below implement only their
// paper's rule.
//
// Note that the rejection exit depends on WNS: `reject` is the driver's
// `wns_regressed` test (GlobalSizingPolicy.cc), so on a design that meets
// timing, a run that spends positive slack to recover power can stop after
// three sweeps. It pairs with best_tracker = wns_pass_reject, which rolls back
// those same passes. The zero-move exit depends only on moves.
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

// Early exit of Sharma et al., ICCAD 2015, Sec. V-B: "The LDP solver can be
// terminated if neither the average power, nor the minimum power solution
// found thus far, improve during two consecutive sets of iterations", with a
// set of 5 iterations. The config constants (stagnation_window, _count,
// _improve_frac, _require_tns) generalize it. With window 1, count 1, frac
// 0.01 and the TNS clause on, it is the rule of Mangiras and Dimitrakopoulos
// (Technologies 2021, Sec. 4.1): stop when TNS and leakage improve by less
// than 1% across two iterations. A window of 1 compares each iteration with
// the previous one; a window w > 1 averages disjoint w-iteration blocks and
// compares each block with the previous one.
//
// Near-met gate: when near_met_gate_frac >= 0 the monitor does nothing (it
// neither stops nor accumulates) until the driver sets LrState::near_met.
// Sharma et al. apply the early exit once timing is almost met; without the
// gate, the monitor can stop a run that is still closing timing, as early as
// 3 * window iterations. With the default (negative frac) near_met is set from
// iteration 0 and the monitor is always active.
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
  float power_sum_ = 0.0f;
  float tns_sum_ = 0.0f;
  int in_window_ = 0;
  // Previous window's averages (0 = no previous window yet).
  float prev_power_avg_ = 0.0f;
  float prev_tns_avg_ = 0.0f;
  bool have_prev_ = false;
  // Min-power memory: the best power ever seen, and its value at the start of
  // the window in progress (so we can ask whether THIS window improved it).
  float best_power_ = 0.0f;
  float best_power_at_window_start_ = 0.0f;
  bool have_best_ = false;
  int stagnant_windows_ = 0;
};

// Stops only at max_iterations. Used by presets whose paper has no early-exit
// rule: the rejection and zero-move exits of FixedItersTermination are
// OpenROAD heuristics, and they can end a paper's schedule before its
// multipliers have had time to move the design.
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

// Which phase of Chinnery and Sharma (ISPD 2022, Sec. 4) the loop is in: "the
// LR sizer first has a timing improvement phase, then a power reduction
// phase". The phase selects which exit criteria apply. A timing-phase exit
// hands over to the power phase instead of ending the run; only a power-phase
// exit (or the wall-clock cap) ends it.
//
// In the paper the phase also changes the multiplier update exponents (4 for
// critical and 1 for non-critical arcs during timing improvement, the reverse
// during power recovery) and skips higher-power cells during power recovery.
// Here those are lambda_update = sharma_arc_slack and power_phase_filter,
// which read the phase through Termination::inPowerPhase().
enum class BatteryPhase
{
  kTiming = 0,
  kPower = 1,
};

// Exit conditions of Chinnery and Sharma. Each belongs to exactly one phase,
// except the wall-clock cap, which bounds the whole run.
enum class StopReason
{
  kNone,
  kTnsTarget,    // timing phase: TNS within term_tns_target_frac of the period
  kWnsTarget,    // timing phase: WNS within term_wns_target_frac of it
  kTnsStall,     // timing phase: TNS improved < term_tns_improve_frac / window
  kPowerStall,   // power phase: power improved < term_power_improve_frac/window
  kTnsDegraded,  // power phase: TNS worse than at the end of the timing phase
  kWallClock,    // either phase: term_wall_limit_s exceeded
  // Reporting only; thresholdBatteryStop never returns it. The loop reached
  // max_iterations (80 in the paper) without meeting any criterion, i.e. the
  // run did not converge. Kept in this enum so RSZ-0451 reports every stop
  // reason with the same set of strings.
  kIterationCap,
};

// Termination criteria of Chinnery and Sharma, ISPD 2022, Sec. 4. The paper's
// two phases are run as termination regimes: an exit of the timing improvement
// phase hands over to the power reduction phase, whose exits end the run. See
// BatteryPhase and StopReason.
//
// The handover and the stop are logged at info level: RSZ-0450 reports the
// handover (which criterion fired, at which iteration) and RSZ-0451 the stop.
// The paper reports convergence as this split ("LR averaged 23 iterations in
// our runs, with 13 of the timing phase and 10 of the power phase"), so a
// total iteration count alone would not be comparable.
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
  bool inPowerPhase() const override { return phase_ == BatteryPhase::kPower; }
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
  std::vector<float> power_history_;
  // Switches to kPower on the first timing-phase exit and never switches back
  // (the paper's phases are sequential, not interleaved).
  BatteryPhase phase_ = BatteryPhase::kTiming;
  // TNS at the end of the timing phase, the reference of the paper's second
  // power-phase exit (kTnsDegraded). Only meaningful once phase_ is kPower.
  float tns_at_handover_ = 0.0f;
  // Sweeps run, and the 0-based iteration of the handover (-1 = none), so the
  // run summary can report the iterations of each phase.
  int iters_run_ = 0;
  int handover_iter_ = -1;
  // Whether reportStop already logged this run's summary, so reportRunEnd
  // logs one only for a run that ended on max_iterations.
  bool reported_ = false;
  // The last iteration's metrics, for the summary of a run that hit the cap
  // (reportRunEnd has no metrics argument).
  IterMetrics last_metrics_;
  // Set at iteration 0 (stopBeforeSweep), so term_wall_limit_s measures the LR
  // loop rather than the setup that precedes it.
  std::chrono::steady_clock::time_point start_;
};

// === Pure helpers (no STA) ==================================================

// Relative improvement of a lower-is-better metric from `prev` to `cur`:
// (prev - cur) / |prev|, so 0.03 means "3% better". Negative when it got worse.
// Returns 0 when prev is 0 (no scale to be relative to).
float relImprovement(float prev, float cur);

// Update rule of the near-met latch. The driver applies it each iteration and
// stores the result in LrState::near_met; the stagnation monitor and the
// local-slack veto are inactive until it is set. A run becomes near-met once
// its worst slack `wns` is within `gate_frac` of the setup target, i.e.
// wns >= -gate_frac * T. Sharma et al. (ICCAD 2015, Sec. V-A) use this
// condition to start power recovery: "after the design timing is within 1% of
// the target delay". Once set it stays set for the rest of the run (`latched`
// true gives true). A negative gate_frac disables gating: the run is near-met
// from the start. With T <= 0 (no clock) an unset latch stays unset, since
// there is no target to be near. LrState::allocate() clears it for each run.
bool nearMetLatched(bool latched, float gate_frac, float T, float wns);

// Sharma's window test: the window is stagnant iff NEITHER the window-average
// power improved (vs. the previous window's average) NOR the best-so-far power
// improved during the window, each by more than `frac`. `require_tns` adds
// Mangiras' clause: the window-average TNS must also have failed to improve.
// `have_prev` is false for the first window, which is never stagnant (there is
// nothing to compare against).
bool stagnantWindow(bool have_prev,
                    float prev_power_avg,
                    float cur_power_avg,
                    float best_power_before,
                    float best_power_now,
                    float frac,
                    bool require_tns,
                    float prev_tns_avg,
                    float cur_tns_avg);

// The battery, evaluated for `phase` only. `tns`/`wns`/`power` are the current
// (post-sweep) values, `T` the clock period; `tns_window_ago` /
// `power_window_ago` are the values term_improve_window iterations back and are
// only consulted when `have_window` (enough history); `tns_at_handover` is the
// TNS as the timing phase ended and is read only in kPower (pass anything in
// kTiming - there is no handover yet). All fracs are the config constants.
//
// A result other than kNone means this phase's exit criterion fired: in kPower
// (and for kWallClock in either phase) that ends the run; in kTiming the timing
// phase is over and the caller hands over to kPower. Gating by phase keeps the
// power-stall test from firing while the design is still closing timing, where
// a power plateau does not mean convergence.
StopReason thresholdBatteryStop(BatteryPhase phase,
                                float tns,
                                float wns,
                                float T,
                                float power,
                                bool have_window,
                                float tns_window_ago,
                                float power_window_ago,
                                float tns_at_handover,
                                float elapsed_s,
                                const GlobalSizingConfig& config);

const char* toString(StopReason reason);

// " total_power=<W>" under the total-power objective, empty under the leakage
// objective. Log lines that report leakage (RSZ-0450, RSZ-0451 and the level-1
// trace) append it, so they also show the power the strategies read.
std::string totalPowerField(const LrState& state, const IterMetrics& metrics);

// The Lagrangian L(x, λ) at the current sweep's assignment. A debug
// diagnostic only; no control decision reads it. It is printed in the level-1
// trace as `lag=`. Under the flow conditions the projection enforces
// (Σλ_in = Σλ_out at internal nodes, Σλ_in(k) = μ_k at endpoints), all
// arrival-time terms cancel out of the Lagrangian (Chen et al., ICCAD 1998,
// Lemma 1), leaving the sweep's own cost minus the endpoint required-time
// constant:
//
//   L(x, λ) = power(x) + Σ_e λ_e·d_e(x) - Σ_k μ_k·r_k
//
// evaluated in the sweep's cost units, i.e. with the timing weight tw applied
// to the multiplier terms (the sweep minimizes power + tw·Σλ·d). power(x) is
// IterMetrics::power.
//
// It is not the dual function Q(λ) and not a bound on the optimum, which is
// why it is not called a dual estimate: (1) it is evaluated at the greedy
// sweep's x, not at the exact minimizer that Q(λ) = min_x L(x, λ) requires, so
// it over-estimates Q; (2) the bound Q(λ) ≤ primal in Chen et al. relies on a
// continuous convex (posynomial) model, and NLDM delays over a discrete
// library are neither; (3) `power` here counts only the instances whose cell
// has Liberty leakage data, while the sweep's cost prices every cell, with
// LRSubproblem::leakageOrArea's scaled area standing in for missing leakage,
// so on a library without leakage the primal term is 0. Treat it as a
// convergence trace: primal - L is approximately -tw·Σ_k μ_k·slack_k, which
// follows the μ-weighted violation and should shrink as the multipliers
// converge; it can go negative once timing is met.
//
// A stop rule based on this gap (primal - L below a tolerance) must also check
// primal feasibility (WNS ≥ margin): the gap can be small while timing is
// still violated.
float lagrangianEstimate(float power,
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
