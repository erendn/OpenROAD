// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "BestTracker.hh"

#include <cmath>
#include <memory>
#include <utility>

#include "rsz/GlobalSizingConfig.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Fuzzy.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
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
  // Oscillation baseline = WNS at the moment the LR journal opens
  // (post-init, post-seed, post-estimation-loop). The loop checkpoints
  // whenever it matches or beats this.
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
  sta::Network* network = state.network;
  best_cells_.clear();
  std::unique_ptr<sta::LeafInstanceIterator> iit(
      network->leafInstanceIterator());
  while (iit->hasNext()) {
    sta::Instance* inst = iit->next();
    sta::LibertyCell* cell = network->libertyCell(inst);
    if (cell == nullptr || state.resizer->dontTouch(inst)) {
      continue;
    }
    best_cells_.emplace_back(inst, cell);
  }
  best_iter_ = iter;
}

bool SnapshotBestTracker::restore(LrState& state)
{
  if (!hasBest()) {
    return false;
  }
  sta::Network* network = state.network;
  int replaced = 0;
  // Serial, and only where the live cell actually drifted from the record - so
  // an iterate that was already the last one costs nothing.
  for (const auto& [inst, cell] : best_cells_) {
    if (network->libertyCell(inst) == cell) {
      continue;
    }
    if (state.resizer->replaceCell(inst, cell, /*journal=*/true)) {
      ++replaced;
    }
  }
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
                    const float leakage,
                    const float T,
                    const float tns_target_frac,
                    const bool have_best,
                    const float best_leakage)
{
  if (T <= 0.0f) {
    return false;
  }
  if (std::abs(tns) >= tns_target_frac * T) {
    return false;
  }
  return !have_best || leakage < best_leakage;
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
  if (init.leakage != 0.0f) {
    d.d_power = (cur.leakage - init.leakage) / std::abs(init.leakage);
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
                      metrics.leakage,
                      state.T,
                      state.config->best_tns_target_frac,
                      hasBest(),
                      best_leakage_)) {
    return;
  }
  best_leakage_ = metrics.leakage;
  capture(state, iter);
  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR best (dominance): iter={} tns={:.6g} leakage={:.6g}",
             iter + 1,
             metrics.tns,
             metrics.leakage);
}

////////////////////////////////////////////////////////////////
// reimann_score

void ScoreBestTracker::consider(LrState& state,
                                const int iter,
                                const IterMetrics& metrics)
{
  const ScoreDeltas d = scoreDeltas(state.metrics_init, metrics);
  const float score = reimannScore(d.d_power, d.d_area, d.d_tv, d.d_wns);
  // best_score_ starts at 0 == the input solution's score (Alg. 2 line 14 seeds
  // the best with the input solution), so this one test also enforces that an
  // iterate must beat the input to be stored at all.
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
  }
  return std::make_unique<NoBestTracker>();
}

}  // namespace rsz
