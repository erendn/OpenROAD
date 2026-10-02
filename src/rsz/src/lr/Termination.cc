// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "Termination.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>

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
// fixed_iters (default)

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
// Pure helpers

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
                    const float prev_power_avg,
                    const float cur_power_avg,
                    const float best_power_before,
                    const float best_power_now,
                    const float frac,
                    const bool require_tns,
                    const float prev_tns_avg,
                    const float cur_tns_avg)
{
  if (!have_prev) {
    return false;
  }
  const bool avg_improved
      = relImprovement(prev_power_avg, cur_power_avg) > frac;
  const bool best_improved
      = relImprovement(best_power_before, best_power_now) > frac;
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
                                const float power,
                                const bool have_window,
                                const float tns_window_ago,
                                const float power_window_ago,
                                const float tns_at_handover,
                                const float elapsed_s,
                                const GlobalSizingConfig& config)
{
  // The wall-clock cap (72 h in the paper) bounds the whole run and belongs to
  // neither phase, so it is tested first, in both phases.
  if (elapsed_s > config.term_wall_limit_s) {
    return StopReason::kWallClock;
  }
  switch (phase) {
    case BatteryPhase::kTiming:
      // The timing phase stops when TNS is below 10% of the clock period, or
      // WNS below 1% of it, or TNS improves less than 10% over the last 3
      // iterations.
      //
      // The targets are relative to the clock period; with no clock (T <= 0)
      // they never fire.
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
      // Second power-phase exit: the phase also stops if TNS "degrades to
      // worse than at the end of the LR timing phase". This is a stop-loss,
      // not convergence: the paper reports it ending about 24% of post-CTS and
      // 36% of pre-CTS runs, "indicating the need for better convergence".
      //
      // Tested before the power stall because it is the more specific reason:
      // a run whose timing has degraded has not merely run out of power to
      // recover. It needs no history window, since its reference is the phase
      // boundary, so it can fire from the first power-phase iteration.
      //
      // The comparison is exact. The paper gives no tolerance for this test,
      // so any TNS loss ends the phase, and a design that still violates timing
      // at the handover can have a power phase of one iteration.
      if (tns < tns_at_handover) {
        return StopReason::kTnsDegraded;
      }
      // The power phase stops when power reduction averages less than 1% over
      // the last 3 iterations.
      if (have_window
          && relImprovement(power_window_ago, power)
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

std::string totalPowerField(const LrState& state, const IterMetrics& metrics)
{
  if (state.config->power_objective
      != GlobalSizingConfig::PowerObjective::kTotal) {
    return "";
  }
  return fmt::format(" total_power={:.6g}", metrics.power);
}

float lagrangianEstimate(const float power,
                         const float timing_weight,
                         const float lambda_delay_sum,
                         const float mu_required_sum)
{
  return power + timing_weight * (lambda_delay_sum - mu_required_sum);
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
  // Inactive until the run is near-met (see the class comment), so the
  // windows count from the start of power recovery, not from iteration 0.
  if (!state.near_met) {
    return false;
  }

  const GlobalSizingConfig& cfg = *state.config;
  const int window = std::max(1, cfg.stagnation_window);
  const int count = std::max(1, cfg.stagnation_count);

  // Best power so far: power is not monotone in the iteration count, which is
  // why Sharma et al. track the best solution as well as the window average.
  const float best_before = have_best_ ? best_power_ : metrics.power;
  if (!have_best_ || metrics.power < best_power_) {
    best_power_ = metrics.power;
    have_best_ = true;
  }
  if (in_window_ == 0) {
    best_power_at_window_start_ = best_before;
  }

  power_sum_ += metrics.power;
  tns_sum_ += metrics.tns;
  ++in_window_;
  if (in_window_ < window) {
    return false;
  }

  const float power_avg = power_sum_ / static_cast<float>(in_window_);
  const float tns_avg = tns_sum_ / static_cast<float>(in_window_);
  const bool stagnant = stagnantWindow(have_prev_,
                                       prev_power_avg_,
                                       power_avg,
                                       best_power_at_window_start_,
                                       best_power_,
                                       cfg.stagnation_improve_frac,
                                       cfg.stagnation_require_tns,
                                       prev_tns_avg_,
                                       tns_avg);
  stagnant_windows_ = stagnant ? stagnant_windows_ + 1 : 0;

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR stagnation window (iter {}): power_avg={:.6g} (prev {:.6g}) "
             "tns_avg={:.6g} best_power={:.6g} -> {} ({}/{})",
             iter + 1,
             power_avg,
             prev_power_avg_,
             tns_avg,
             best_power_,
             stagnant ? "stagnant" : "improving",
             stagnant_windows_,
             count);

  prev_power_avg_ = power_avg;
  prev_tns_avg_ = tns_avg;
  have_prev_ = true;
  power_sum_ = 0.0f;
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
  // The wall-clock limit applies to the LR loop, so start the clock at the
  // loop, not when the strategy is constructed in start(); that would also
  // count the init pass, the multiplier seed and the estimation-loop runs.
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
  power_history_.push_back(metrics.power);
  iters_run_ = iter + 1;
  last_metrics_ = metrics;

  const int window = std::max(1, cfg.term_improve_window);
  const int n = static_cast<int>(tns_history_.size());
  const bool have_window = (n > window);
  const float tns_ago = have_window ? tns_history_[n - 1 - window] : 0.0f;
  const float power_ago = have_window ? power_history_[n - 1 - window] : 0.0f;
  const float elapsed_s
      = std::chrono::duration<float>(std::chrono::steady_clock::now() - start_)
            .count();

  const StopReason reason = thresholdBatteryStop(phase_,
                                                 metrics.tns,
                                                 metrics.wns,
                                                 state.T,
                                                 metrics.power,
                                                 have_window,
                                                 tns_ago,
                                                 power_ago,
                                                 tns_at_handover_,
                                                 elapsed_s,
                                                 cfg);
  if (reason == StopReason::kNone) {
    return false;
  }
  // A timing-phase criterion ends the timing phase, not the run: the loop hands
  // over to power reduction, and from then on only that phase's exits can stop
  // it. The wall-clock cap stops the run from either phase.
  if (phase_ == BatteryPhase::kTiming && reason != StopReason::kWallClock) {
    phase_ = BatteryPhase::kPower;
    // Reference of the second power-phase exit: TNS at the end of the timing
    // phase.
    tns_at_handover_ = metrics.tns;
    handover_iter_ = iter;
    // Restart the improvement window at the phase boundary, so the power exit
    // measures the power phase's own progress. A window that straddles the
    // boundary would compare with a timing-phase power; the timing phase
    // upsizes and raises power, so the power phase would see a negative
    // improvement and stop on its first iteration.
    tns_history_.clear();
    power_history_.clear();
    // Logged at info level because the phase split is part of the run
    // summary; see the class comment.
    state.logger->info(RSZ,
                       450,
                       "GLOBAL_SIZING threshold_battery: timing phase ended "
                       "after {} iteration(s) ({}); handing over to the "
                       "power-reduction phase [tns={:.6g} wns={:.6g} "
                       "leakage={:.6g}{} T={:.6g}].",
                       iter + 1,
                       toString(reason),
                       metrics.tns,
                       metrics.wns,
                       metrics.leakage,
                       totalPowerField(state, metrics),
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
  // Report the iterations of each phase, not just the total: "N iterations in
  // the power phase" would read as N power-phase iterations when N is the run
  // total.
  const int timing_iters
      = (handover_iter_ >= 0) ? handover_iter_ + 1 : iters_run_;
  state.logger->info(RSZ,
                     451,
                     "GLOBAL_SIZING threshold_battery: stopped after {} "
                     "iteration(s) ({} timing + {} power) in the {} phase ({}) "
                     "[tns={:.6g} wns={:.6g} leakage={:.6g}{} T={:.6g}].",
                     iters_run_,
                     timing_iters,
                     iters_run_ - timing_iters,
                     phase_ == BatteryPhase::kTiming ? "timing" : "power",
                     toString(reason),
                     metrics.tns,
                     metrics.wns,
                     metrics.leakage,
                     totalPowerField(state, metrics),
                     state.T);
}

void ThresholdBatteryTermination::reportRunEnd(LrState& state)
{
  // Only a run that ended on max_iterations gets here without a summary. Log
  // one so that runs that did not converge are reported too.
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
