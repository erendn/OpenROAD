// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "Termination.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>

#include "db_sta/dbSta.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/Sta.hh"
#include "sta/Transition.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

int Termination::maxIterations(const LrState& state) const
{
  return (state.config->max_iterations > 0)
             ? state.config->max_iterations
             : GlobalSizingConfig{}.max_iterations;
}

////////////////////////////////////////////////////////////////
// fixed_iters (baseline)

bool FixedItersTermination::stopAfterSweep(LrState& state,
                                           int /* iter */,
                                           const bool reject,
                                           const bool no_benefit,
                                           const IterMetrics& /* metrics */)
{
  if (reject) {
    ++consec_reject_;
  } else {
    consec_reject_ = 0;
  }
  if (consec_reject_ >= 3) {
    debugPrint(state.logger,
               RSZ,
               "global_sizing",
               1,
               "LR stop: 3 consecutive rejections");
    return true;
  }
  if (no_benefit && !reject) {
    if (++consec_zero_ >= 2) {
      debugPrint(state.logger,
                 RSZ,
                 "global_sizing",
                 1,
                 "LR stop: 2 consecutive zero-move passes");
      return true;
    }
  } else {
    consec_zero_ = 0;
  }
  return false;
}

////////////////////////////////////////////////////////////////
// Pure cores

float relImprovement(const float prev, const float cur)
{
  const float scale = std::abs(prev);
  if (scale == 0.0f) {
    return 0.0f;
  }
  return (prev - cur) / scale;
}

bool nearMetLatched(const bool latched,
                    const float gate_frac,
                    const float T,
                    const float wns)
{
  if (latched) {
    return true;  // permanent for the run
  }
  if (gate_frac < 0.0f) {
    return true;  // gating disabled: near-met from the start
  }
  if (T <= 0.0f) {
    return false;  // no clock: no target to be within a fraction of
  }
  return wns >= -gate_frac * T;
}

bool stagnantWindow(const bool have_prev,
                    const float prev_leak_avg,
                    const float cur_leak_avg,
                    const float best_leak_before,
                    const float best_leak_now,
                    const float frac,
                    const bool require_tns,
                    const float prev_tns_avg,
                    const float cur_tns_avg)
{
  if (!have_prev) {
    return false;
  }
  const bool avg_improved = relImprovement(prev_leak_avg, cur_leak_avg) > frac;
  const bool best_improved
      = relImprovement(best_leak_before, best_leak_now) > frac;
  if (avg_improved || best_improved) {
    return false;
  }
  if (require_tns) {
    // TNS is <= 0 and "improving" means moving toward 0, i.e. |TNS| shrinking.
    const bool tns_improved
        = relImprovement(std::abs(prev_tns_avg), std::abs(cur_tns_avg)) > frac;
    if (tns_improved) {
      return false;
    }
  }
  return true;
}

StopReason thresholdBatteryStop(const BatteryPhase phase,
                                const float tns,
                                const float wns,
                                const float T,
                                const float leakage,
                                const bool have_window,
                                const float tns_window_ago,
                                const float leak_window_ago,
                                const float tns_at_handover,
                                const float elapsed_s,
                                const GlobalSizingConfig& config)
{
  // The 72 h cap is a hard safety cap on the whole run (§5), listed apart from
  // the phase criteria, so it is tested first and in both phases.
  if (elapsed_s > config.term_wall_limit_s) {
    return StopReason::kWallClock;
  }
  switch (phase) {
    case BatteryPhase::kTiming:
      // §5: "Timing phase stops when TNS < 10% of target clock period, or WNS
      // < 1% thereof, or TNS improves < 10% over the last 3 iterations."
      //
      // Timing targets are relative to the clock period; with no clock (T <= 0)
      // there is nothing to be relative to, so they never fire.
      if (T > 0.0f) {
        if (std::abs(tns) < config.term_tns_target_frac * T) {
          return StopReason::kTnsTarget;
        }
        if (std::abs(std::min(0.0f, wns)) < config.term_wns_target_frac * T) {
          return StopReason::kWnsTarget;
        }
      }
      // |TNS| is the lower-is-better quantity.
      if (have_window
          && relImprovement(std::abs(tns_window_ago), std::abs(tns))
                 < config.term_tns_improve_frac) {
        return StopReason::kTnsStall;
      }
      return StopReason::kNone;
    case BatteryPhase::kPower:
      // §5, the SECOND power-phase exit: the phase also ends "if TNS degrades
      // worse than it was at the end of the timing phase". It is a STOP-LOSS,
      // not a controller - the paper reports it firing in ~24% of post-CTS and
      // ~36% of pre-CTS runs and flags that as a convergence deficiency of its
      // own method (chinnery re-audit, "Implications for implementation"). Read
      // it that way: a chinnery run that ends here converged badly, it did not
      // converge.
      //
      // Tested BEFORE the power stall because it is the more specific verdict:
      // a run whose timing has come apart is not merely out of power to
      // recover. It needs no window: the reference is the phase boundary, so
      // this exit is live from the power phase's first iteration (which is also
      // the earliest the paper's own criterion could fire).
      //
      // THE COMPARISON IS EXACT, and that is a deliberate reading with a sharp
      // edge. Every other threshold in this battery is a relative fraction;
      // this one has no stated tolerance, and the paper's constraint language
      // for the power phase is "do not worsen existing violations" (§8), i.e.
      // the bar really is zero. So one power-recovery move that costs a
      // femtosecond of TNS ends the phase, and on a design that hands over
      // still violating the phase can be one iteration long. That IS the
      // paper's stop-loss firing (it reports 24-36% of runs ending here), but
      // an implementer looking for a tolerance knob should know there is none
      // by choice, not by omission: inventing one would be a non-paper constant
      // in a paper column. Campaign watch item, not a defect to patch.
      if (tns < tns_at_handover) {
        return StopReason::kTnsDegraded;
      }
      // §5: "Power phase stops when power reduction averages < 1% over the last
      // 3 iterations".
      if (have_window
          && relImprovement(leak_window_ago, leakage)
                 < config.term_power_improve_frac) {
        return StopReason::kPowerStall;
      }
      return StopReason::kNone;
  }
  return StopReason::kNone;
}

const char* toString(const StopReason reason)
{
  switch (reason) {
    case StopReason::kNone:
      return "none";
    case StopReason::kTnsTarget:
      return "TNS target met";
    case StopReason::kWnsTarget:
      return "WNS target met";
    case StopReason::kTnsStall:
      return "TNS improvement stalled";
    case StopReason::kPowerStall:
      return "power improvement stalled";
    case StopReason::kTnsDegraded:
      return "TNS degraded past the end of the timing phase";
    case StopReason::kWallClock:
      return "wall-clock limit";
    case StopReason::kIterationCap:
      return "iteration cap reached, no battery criterion met";
  }
  return "unknown";
}

float lagrangianEstimate(const float leakage,
                         const float timing_weight,
                         const float lambda_delay_sum,
                         const float mu_required_sum)
{
  return leakage + timing_weight * (lambda_delay_sum - mu_required_sum);
}

LagrangianTerms computeLagrangianTerms(LrState& state)
{
  LagrangianTerms terms;
  sta::Graph* graph = state.graph;

  sta::VertexIterator vit(graph);
  while (vit.hasNext()) {
    sta::Vertex* v = vit.next();
    sta::VertexOutEdgeIterator eit(v, graph);
    while (eit.hasNext()) {
      sta::Edge* e = eit.next();
      if (!state.isDataArc(e)) {
        continue;
      }
      const sta::EdgeId id = graph->id(e);
      if (static_cast<size_t>(id) >= state.lambda.size()) {
        continue;
      }
      terms.lambda_delay_sum += state.lambda[id] * state.edgeMaxArcDelay(e);
    }
  }

  for (size_t k = 0; k < state.endpoint_vertices.size(); ++k) {
    const float required
        = sta::delayAsFloat(state.sta->required(state.endpoint_vertices[k],
                                                sta::RiseFallBoth::riseFall(),
                                                state.sta->scenes(),
                                                state.max));
    // Unconstrained endpoints report a sentinel required time; they contribute
    // no endpoint constraint to relax, and their mu is 0 anyway.
    if (!std::isfinite(required)
        || std::abs(required) >= LrState::kSlackSentinel) {
      continue;
    }
    terms.mu_required_sum += state.mu[k] * required;
  }
  return terms;
}

////////////////////////////////////////////////////////////////
// stagnation_windows (Sharma / Mangiras)

bool StagnationWindowsTermination::stopAfterSweep(LrState& state,
                                                  const int iter,
                                                  bool /* reject */,
                                                  bool /* no_benefit */,
                                                  const IterMetrics& metrics)
{
  // Near-met gate (C2, sharma audit §2 item 3): while the design is still
  // closing timing the monitor stays inactive - it neither stops nor
  // accumulates window state, so the stagnation windows count from the start of
  // power recovery, not from iteration 0. Ungated presets (near_met_gate_frac <
  // 0) latch near_met at iteration 0, so this never fires and Mangiras'
  // window-1 rule is unchanged.
  if (!state.near_met) {
    return false;
  }

  const GlobalSizingConfig& cfg = *state.config;
  const int window = std::max(1, cfg.stagnation_window);
  const int count = std::max(1, cfg.stagnation_count);

  // Min-power memory: power is non-monotone in the iteration count, which is
  // exactly why Sharma tracks the best-so-far as well as the window average.
  const float best_before = have_best_ ? best_leak_ : metrics.leakage;
  if (!have_best_ || metrics.leakage < best_leak_) {
    best_leak_ = metrics.leakage;
    have_best_ = true;
  }
  if (in_window_ == 0) {
    best_leak_at_window_start_ = best_before;
  }

  leak_sum_ += metrics.leakage;
  tns_sum_ += metrics.tns;
  ++in_window_;
  if (in_window_ < window) {
    return false;
  }

  const float leak_avg = leak_sum_ / static_cast<float>(in_window_);
  const float tns_avg = tns_sum_ / static_cast<float>(in_window_);
  const bool stagnant = stagnantWindow(have_prev_,
                                       prev_leak_avg_,
                                       leak_avg,
                                       best_leak_at_window_start_,
                                       best_leak_,
                                       cfg.stagnation_improve_frac,
                                       cfg.stagnation_require_tns,
                                       prev_tns_avg_,
                                       tns_avg);
  stagnant_windows_ = stagnant ? stagnant_windows_ + 1 : 0;

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR stagnation window (iter {}): leak_avg={:.6g} (prev {:.6g}) "
             "tns_avg={:.6g} best_leak={:.6g} -> {} ({}/{})",
             iter + 1,
             leak_avg,
             prev_leak_avg_,
             tns_avg,
             best_leak_,
             stagnant ? "stagnant" : "improving",
             stagnant_windows_,
             count);

  prev_leak_avg_ = leak_avg;
  prev_tns_avg_ = tns_avg;
  have_prev_ = true;
  leak_sum_ = 0.0f;
  tns_sum_ = 0.0f;
  in_window_ = 0;

  if (stagnant_windows_ >= count) {
    debugPrint(state.logger,
               RSZ,
               "global_sizing",
               1,
               "LR stop: {} consecutive stagnant windows of {} iterations",
               stagnant_windows_,
               window);
    return true;
  }
  return false;
}

////////////////////////////////////////////////////////////////
// threshold_battery (Chinnery)

bool ThresholdBatteryTermination::stopBeforeSweep(LrState& /* state */,
                                                  const int iter)
{
  // The wall-clock budget bounds the LR loop, so start it at the loop - not at
  // the strategy's construction in start(), which would charge the init pass,
  // the seed, and the estimation-loop dry runs against it.
  if (iter == 0) {
    start_ = std::chrono::steady_clock::now();
  }
  return false;
}

bool ThresholdBatteryTermination::stopAfterSweep(LrState& state,
                                                 const int iter,
                                                 bool /* reject */,
                                                 bool /* no_benefit */,
                                                 const IterMetrics& metrics)
{
  const GlobalSizingConfig& cfg = *state.config;
  tns_history_.push_back(metrics.tns);
  leak_history_.push_back(metrics.leakage);
  iters_run_ = iter + 1;
  last_metrics_ = metrics;

  const int window = std::max(1, cfg.term_improve_window);
  const int n = static_cast<int>(tns_history_.size());
  const bool have_window = (n > window);
  const float tns_ago = have_window ? tns_history_[n - 1 - window] : 0.0f;
  const float leak_ago = have_window ? leak_history_[n - 1 - window] : 0.0f;
  const float elapsed_s
      = std::chrono::duration<float>(std::chrono::steady_clock::now() - start_)
            .count();

  const StopReason reason = thresholdBatteryStop(phase_,
                                                 metrics.tns,
                                                 metrics.wns,
                                                 state.T,
                                                 metrics.leakage,
                                                 have_window,
                                                 tns_ago,
                                                 leak_ago,
                                                 tns_at_handover_,
                                                 elapsed_s,
                                                 cfg);
  if (reason == StopReason::kNone) {
    return false;
  }
  // A timing-phase criterion ends the timing phase, not the run: the design has
  // reached its timing target, so the loop hands over to power reduction and
  // only that phase's exits can stop it from here. The wall-clock cap is a hard
  // safety cap and stops the run from either phase.
  if (phase_ == BatteryPhase::kTiming && reason != StopReason::kWallClock) {
    phase_ = BatteryPhase::kPower;
    // The reference of the second power-phase exit (§5): TNS as the timing
    // phase ended.
    tns_at_handover_ = metrics.tns;
    handover_iter_ = iter;
    // Restart the improvement window at the phase boundary. §5's power exit is
    // "power reduction averages < 1% over the last 3 iterations" *of the power
    // phase*: a window straddling the boundary would compare against a
    // timing-phase leakage, and the timing phase raises leakage (it upsizes to
    // close timing), so the comparison would read as a negative improvement and
    // trip kPowerStall on the power phase's very first iteration - every run.
    // The paper's power phase averages 10 iterations, so it plainly measures
    // its own progress.
    tns_history_.clear();
    leak_history_.clear();
    // INFO, not debug: the paper's convergence claim IS the phase split ("23
    // iterations: 13 timing + 10 power", §5), so the boundary is a run record,
    // not a trace detail. See the class doc.
    state.logger->info(RSZ,
                       450,
                       "GLOBAL_SIZING threshold_battery: timing phase ended "
                       "after {} iteration(s) ({}); handing over to the "
                       "power-reduction phase [tns={:.6g} wns={:.6g} "
                       "leakage={:.6g} T={:.6g}].",
                       iter + 1,
                       toString(reason),
                       metrics.tns,
                       metrics.wns,
                       metrics.leakage,
                       state.T);
    return false;
  }
  reportStop(state, reason, metrics);
  return true;
}

void ThresholdBatteryTermination::reportStop(LrState& state,
                                             const StopReason reason,
                                             const IterMetrics& metrics)
{
  reported_ = true;
  // The paper's convergence figure is the SPLIT ("23 iterations: 13 timing + 10
  // power", §5), so publish both halves rather than a total the reader has to
  // attribute: an unqualified "N iterations in the power phase" reads as N
  // power-phase iterations when N is the run total, and the harness that parses
  // this record would then double-count the timing phase.
  const int timing_iters
      = (handover_iter_ >= 0) ? handover_iter_ + 1 : iters_run_;
  state.logger->info(RSZ,
                     451,
                     "GLOBAL_SIZING threshold_battery: stopped after {} "
                     "iteration(s) ({} timing + {} power) in the {} phase ({}) "
                     "[tns={:.6g} wns={:.6g} leakage={:.6g} T={:.6g}].",
                     iters_run_,
                     timing_iters,
                     iters_run_ - timing_iters,
                     phase_ == BatteryPhase::kTiming ? "timing" : "power",
                     toString(reason),
                     metrics.tns,
                     metrics.wns,
                     metrics.leakage,
                     state.T);
}

void ThresholdBatteryTermination::reportRunEnd(LrState& state)
{
  // Only a run that ended on max_iterations gets here unreported. Publishing it
  // is the point: a record emitted only from the stop verdict is missing
  // exactly on the runs that did NOT converge, which are the interesting ones,
  // and its absence is then indistinguishable from a dropped log line.
  if (reported_ || iters_run_ == 0) {
    return;
  }
  reportStop(state, StopReason::kIterationCap, last_metrics_);
}

std::unique_ptr<Termination> makeTermination(const GlobalSizingConfig& config)
{
  switch (config.termination) {
    case GlobalSizingConfig::TerminationKind::kFixedIters:
      return std::make_unique<FixedItersTermination>();
    case GlobalSizingConfig::TerminationKind::kStagnationWindows:
      return std::make_unique<StagnationWindowsTermination>();
    case GlobalSizingConfig::TerminationKind::kThresholdBattery:
      return std::make_unique<ThresholdBatteryTermination>();
    case GlobalSizingConfig::TerminationKind::kPureCap:
      return std::make_unique<PureCapTermination>();
  }
  return std::make_unique<FixedItersTermination>();
}

}  // namespace rsz
