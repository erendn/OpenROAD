// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "BestTracker.hh"

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>

#include "rsz/GlobalSizingConfig.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Fuzzy.hh"
#include "sta/Sta.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

////////////////////////////////////////////////////////////////
// Pass-level journal (default: keep every sweep)

void BestTracker::beginLoop(LrState& state)
{
  state.resizer->journalBegin();
}

void BestTracker::endLoop(LrState& state)
{
  // Commit every sweep. The papers run all their iterations and then select;
  // the selection is restore()'s job, by cell assignment.
  state.resizer->journalEnd();
}

////////////////////////////////////////////////////////////////
// wns_pass_reject (rsz_baseline)

void WnsPassRejectTracker::beginLoop(LrState& state)
{
  // Reference WNS: the WNS when the LR journal opens (after initialization,
  // seeding and the estimation loop). The loop checkpoints whenever a sweep
  // matches or beats it.
  best_wns_ = sta::delayAsFloat(state.sta->worstSlack(state.max));
  state.resizer->journalBegin();
}

bool WnsPassRejectTracker::considerPass(LrState& state,
                                        const bool wns_regressed)
{
  // Best-so-far: keep track of the best WNS so far but don't restore a sweep
  // that worsens WNS just yet to allow oscillation.
  const float current_wns = sta::delayAsFloat(state.sta->worstSlack(state.max));
  if (!wns_regressed && sta::fuzzyGreaterEqual(current_wns, best_wns_)
      && !state.resizer->overMaxArea()) {
    state.resizer->journalEnd();  // checkpoint
    state.resizer->journalBegin();
    best_wns_ = current_wns;
  }
  return wns_regressed;
}

void WnsPassRejectTracker::endLoop(LrState& state)
{
  // The journal is always open at loop exit; undo any drift past the last
  // checkpoint so the live state matches the best LR achieved (or the
  // pre-loop state if it never checkpointed).
  state.resizer->journalRestore();
}

////////////////////////////////////////////////////////////////
// Snapshot / restore

void SnapshotBestTracker::capture(LrState& state, const int iter)
{
  best_cells_.capture(state);
  has_best_ = true;
  best_iter_ = iter;
}

bool SnapshotBestTracker::restore(LrState& state)
{
  if (!hasBest()) {
    return false;
  }
  const int replaced = best_cells_.restore(state);
  // Iterations are reported from 1, so iteration 0 is the input solution.
  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             1,
             "LR best-solution restore: iteration {}, {} cells reinstated",
             best_iter_ + 1,
             replaced);
  return replaced > 0;
}

////////////////////////////////////////////////////////////////
// Pure cores

bool flachDominates(const float tns,
                    const float power,
                    const float T,
                    const float tns_target_frac,
                    const bool have_best,
                    const float best_power)
{
  if (T <= 0.0f) {
    return false;
  }
  if (std::abs(tns) >= tns_target_frac * T) {
    return false;
  }
  return !have_best || power < best_power;
}

bool livramentoViolationFree(const float wns,
                             const size_t max_cap_violations,
                             const size_t max_slew_violations)
{
  return wns >= 0.0f && max_cap_violations == 0 && max_slew_violations == 0;
}

bool livramentoReplacesBest(const bool violation_free,
                            const float power,
                            const bool have_best,
                            const float best_power)
{
  return violation_free && (!have_best || power < best_power);
}

float reimannScore(const float d_power,
                   const float d_area,
                   const float d_tv,
                   const float d_wns)
{
  return -(d_power + d_area + std::exp2(-(d_tv + d_wns)) - 1.0f);
}

ScoreDeltas scoreDeltas(const IterMetrics& init, const IterMetrics& cur)
{
  ScoreDeltas d;
  if (init.power != 0.0f) {
    d.d_power = (cur.power - init.power) / std::abs(init.power);
  }
  if (init.area != 0.0f) {
    d.d_area = (cur.area - init.area) / std::abs(init.area);
  }
  // Timing violation = |TNS|; improvement is a reduction, hence init - cur.
  const float tv_init = std::abs(init.tns);
  if (tv_init != 0.0f) {
    d.d_tv = (tv_init - std::abs(cur.tns)) / tv_init;
  }
  // WNS is higher-is-better, so improvement is cur - init, normalized by the
  // input violation to stay dimensionless.
  const float wns_scale = std::abs(init.wns);
  if (wns_scale != 0.0f) {
    d.d_wns = (cur.wns - init.wns) / wns_scale;
  }
  return d;
}

////////////////////////////////////////////////////////////////
// flach_dominance

void DominanceBestTracker::consider(LrState& state,
                                    const int iter,
                                    const IterMetrics& metrics)
{
  if (!flachDominates(metrics.tns,
                      metrics.power,
                      state.T,
                      state.config->best_tns_target_frac,
                      hasBest(),
                      best_power_)) {
    return;
  }
  best_power_ = metrics.power;
  capture(state, iter);
  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR best (dominance): iter={} tns={:.6g} power={:.6g}",
             iter + 1,
             metrics.tns,
             metrics.power);
}

////////////////////////////////////////////////////////////////
// reimann_score

void ScoreBestTracker::recordInput(LrState& state)
{
  capture(state, kInputIter);
}

void ScoreBestTracker::consider(LrState& state,
                                const int iter,
                                const IterMetrics& metrics)
{
  const ScoreDeltas d = scoreDeltas(state.metrics_init, metrics);
  const float score = reimannScore(d.d_power, d.d_area, d.d_tv, d.d_wns);
  // A tie keeps the stored best.
  if (score <= best_score_) {
    return;
  }
  best_score_ = score;
  capture(state, iter);
  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR best (score): iter={} score={:.6g} (dP={:.4g} dA={:.4g} "
             "dTV={:.4g} dWNS={:.4g})",
             iter + 1,
             score,
             d.d_power,
             d.d_area,
             d.d_tv,
             d.d_wns);
}

////////////////////////////////////////////////////////////////
// livramento_feasible

bool FeasibleSelection::offer(const int iter,
                              const float wns,
                              const size_t max_cap_violations,
                              const size_t max_slew_violations,
                              const float power)
{
  const bool clean
      = livramentoViolationFree(wns, max_cap_violations, max_slew_violations);
  ++iterations;
  if (clean) {
    ++violation_free_count;
  }
  if (!livramentoReplacesBest(clean, power, hasBest(), best_power)) {
    return false;
  }
  best_iter = iter;
  best_power = power;
  return true;
}

void FeasibleBestTracker::consider(LrState& state,
                                   const int iter,
                                   const IterMetrics& metrics)
{
  const size_t cap_violations = state.sta->maxCapacitanceViolationCount();
  const size_t slew_violations = state.sta->maxSlewViolationCount();
  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR best (violation-free): iter={} wns={:.6g} max_cap_viol={} "
             "max_slew_viol={} power={:.6g}",
             iter + 1,
             metrics.wns,
             cap_violations,
             slew_violations,
             metrics.power);
  if (selection_.offer(
          iter, metrics.wns, cap_violations, slew_violations, metrics.power)) {
    capture(state, iter);
  }
}

bool FeasibleBestTracker::restore(LrState& state)
{
  // Iterations are reported from 1.
  const std::string outcome
      = selection_.hasBest()
            ? fmt::format("restoring iteration {}, the lowest-power one",
                          selection_.best_iter + 1)
            : std::string("keeping the final iteration");
  state.logger->info(RSZ,
                     460,
                     "GLOBAL_SIZING best solution: {} of {} iteration(s) had "
                     "no setup, max capacitance or max slew violation; {}.",
                     selection_.violation_free_count,
                     selection_.iterations,
                     outcome);
  return SnapshotBestTracker::restore(state);
}

std::unique_ptr<BestTracker> makeBestTracker(const GlobalSizingConfig& config)
{
  switch (config.best_tracker) {
    case GlobalSizingConfig::BestTrackerKind::kNone:
      return std::make_unique<NoBestTracker>();
    case GlobalSizingConfig::BestTrackerKind::kWnsPassReject:
      return std::make_unique<WnsPassRejectTracker>();
    case GlobalSizingConfig::BestTrackerKind::kFlachDominance:
      return std::make_unique<DominanceBestTracker>();
    case GlobalSizingConfig::BestTrackerKind::kReimannScore:
      return std::make_unique<ScoreBestTracker>();
    case GlobalSizingConfig::BestTrackerKind::kLivramentoFeasible:
      return std::make_unique<FeasibleBestTracker>();
  }
  return std::make_unique<NoBestTracker>();
}

}  // namespace rsz
