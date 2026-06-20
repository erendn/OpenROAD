// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "PowerReliefPolicy.hh"

#include <algorithm>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "DelayEstimator.hh"
#include "MoveCommitter.hh"
#include "OptimizerTypes.hh"
#include "SizeDownFanoutCandidate.hh"
#include "UnbufferCandidate.hh"
#include "VtSwapCandidate.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Fuzzy.hh"
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/Path.hh"
#include "sta/PathExpanded.hh"
#include "sta/PortDirection.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

namespace {

// Read the leakage of `cell` from the Resizer's leakage cache. Returns 0 when
// the Liberty does not expose a leakage value -- in that case sensitivity
// ranking degenerates to "first candidate that fits."
float cellLeakageValue(Resizer& resizer, sta::LibertyCell* cell)
{
  if (cell == nullptr) {
    return 0.0f;
  }
  const auto leak = resizer.cellLeakage(cell);
  return leak.has_value() ? *leak : 0.0f;
}

}  // namespace

namespace {

// Total Liberty leakage power of every logic stdcell in the design.
// Used as the before/after baseline for the policy's saved-power report.
float computeTotalLogicLeakage(Resizer& resizer, sta::Network* network)
{
  float total_leak = 0.0f;
  std::unique_ptr<sta::LeafInstanceIterator> iter(
      network->leafInstanceIterator());
  while (iter->hasNext()) {
    sta::Instance* inst = iter->next();
    if (resizer.isLogicStdCell(inst)) {
      const auto leak = resizer.cellLeakage(network->libertyCell(inst));
      if (leak) {
        total_leak += *leak;
      }
    }
  }
  return total_leak;
}

}  // namespace

void PowerReliefPolicy::iterate()
{
  // Establish before/after baselines once, outside the convergence loop
  sta_->findRequireds();
  sta::Slack pre_wns = sta_->worstSlack(max_);
  sta::Slack pre_tns = sta_->totalNegativeSlack(max_);
  const float pre_leakage = computeTotalLogicLeakage(resizer_, network_);
  const double pre_area = resizer_.computeDesignArea();

  // Unbuffer pre-pass: Timing repair often inserts buffers that may become
  // redundant once timing is met.
  const int unbuffered = runUnbufferPass(pre_wns, pre_tns);
  debugPrint(logger_,
             RSZ,
             "power_relief",
             1,
             "Unbuffer Pass: Removed {} buffers",
             unbuffered);

  // Convergence loop: Each pass re-gathers candidates against the slack
  // frontier left by the previous pass. Stop as soon as a pass commits zero
  // swaps (no more cascade opportunities) or the configured ceiling binds.
  int num_swaps = 0;
  int last_pass_swaps = 0;
  int passes_run = 0;
  for (int pass = 1; pass <= max_passes_; ++pass) {
    const int pass_swaps = runPass(pre_wns, pre_tns);
    passes_run = pass;
    debugPrint(logger_,
               RSZ,
               "power_relief",
               1,
               "Downsize Pass {}/{}: Committed {} swaps",
               pass,
               max_passes_,
               pass_swaps);
    num_swaps += pass_swaps;
    last_pass_swaps = pass_swaps;
    if (pass_swaps == 0) {
      break;
    }
  }
  passes_run -= (last_pass_swaps == 0 ? 1 : 0);

  const int total_moves = num_swaps + unbuffered;
  if (total_moves > 0) {
    const float post_leakage = computeTotalLogicLeakage(resizer_, network_);
    const float saved_leakage = pre_leakage - post_leakage;
    const float saved_leakage_pct
        = (pre_leakage > 0) ? (saved_leakage / pre_leakage) * 100.0f : 0.0f;
    const double post_area = resizer_.computeDesignArea();
    const double saved_area = pre_area - post_area;
    const double saved_area_pct
        = (pre_area > 0) ? (saved_area / pre_area) * 100.0 : 0.0;

    logger_->info(RSZ,
                  290,
                  "Power Relief reclaimed power/area on {} instances "
                  "({} swaps over {} pass{}, {} buffers removed). "
                  "Saved {:.2e} leakage power ({:.2f}%); "
                  "saved {:.2e} m^2 area ({:.2f}%).",
                  total_moves,
                  num_swaps,
                  passes_run,
                  passes_run > 1 ? "es" : "",
                  unbuffered,
                  saved_leakage,
                  saved_leakage_pct,
                  saved_area,
                  saved_area_pct);
  }

  markRunComplete(true);
}

int PowerReliefPolicy::runPass(sta::Slack& pre_wns, sta::Slack& pre_tns)
{
  // 1. Gather every logic instance with positive slack. Re-built per pass
  // because committed swaps in prior passes shifted the slack landscape.
  std::vector<std::pair<sta::Instance*, sta::Slack>> candidates;
  std::unique_ptr<sta::LeafInstanceIterator> leaf_iter(
      network_->leafInstanceIterator());
  while (leaf_iter->hasNext()) {
    sta::Instance* inst = leaf_iter->next();
    if (resizer_.dontTouch(inst) || !resizer_.isLogicStdCell(inst)) {
      continue;
    }
    const sta::Slack slack = getInstanceSlack(inst);
    if (sta::fuzzyGreater(slack, config_.setup_slack_margin)) {
      candidates.emplace_back(inst, slack);
    }
  }

  // 2. Sort candidates by slack descending (loosest first). Touching
  // loosest-slack gates first maximizes the chance that local delay penalties
  // stay safely under the per-gate budget.
  std::ranges::sort(candidates, [](const auto& lhs, const auto& rhs) {
    return lhs.second > rhs.second;
  });

  debugPrint(logger_,
             RSZ,
             "power_relief",
             2,
             "  pass scan: {} positive-slack candidates (batch_size={})",
             candidates.size(),
             batch_size_);

  // 3. Score every candidate into a pending batch. When the batch fills, apply
  // it under a single journal with a per-batch WNS/TNS check; on regression,
  // bisect-and-retry so good moves are not discarded alongside the offending
  // one.
  std::vector<BatchEntry> pending_batch;
  pending_batch.reserve(batch_size_);
  int pass_swaps = 0;

  for (const auto& [inst, inst_slack] : candidates) {
    sta::Pin* drvr_pin = outputPin(inst);
    if (drvr_pin == nullptr) {
      continue;
    }
    std::optional<BatchEntry> best = findBestSwap(inst, drvr_pin, inst_slack);
    if (!best.has_value()) {
      continue;
    }
    pending_batch.push_back(*best);
    if (static_cast<int>(pending_batch.size()) >= batch_size_) {
      pass_swaps += applyBatchWithBisection(pending_batch, pre_wns, pre_tns);
      pending_batch.clear();
    }
  }

  if (!pending_batch.empty()) {
    pass_swaps += applyBatchWithBisection(pending_batch, pre_wns, pre_tns);
    pending_batch.clear();
  }

  return pass_swaps;
}

int PowerReliefPolicy::runUnbufferPass(sta::Slack& pre_wns, sta::Slack& pre_tns)
{
  std::vector<BatchEntry> pending;
  pending.reserve(batch_size_);
  int removed = 0;

  std::unique_ptr<sta::LeafInstanceIterator> leaf_iter(
      network_->leafInstanceIterator());
  while (leaf_iter->hasNext()) {
    sta::Instance* inst = leaf_iter->next();
    if (resizer_.dontTouch(inst)) {
      continue;
    }
    sta::LibertyCell* cell = network_->libertyCell(inst);
    if (cell == nullptr || !cell->isBuffer()) {
      continue;
    }
    // Skip clock buffers: Don't touch the clock tree.
    std::unique_ptr<sta::InstancePinIterator> pit(network_->pinIterator(inst));
    while (pit->hasNext()) {
      sta::Pin* pin = pit->next();
      const sta::PortDirection* dir = network_->direction(pin);
      if (dir->isOutput() && sta_->isClock(pin, sta_->cmdMode())) {
        return false;
      }
    }
    // Skip buffers on critical paths: Only remove buffers whose worst pin
    // slack is comfortably positive. Resizer::canRemoveBuffer checks electrical
    // legality (max-cap on the merged net, etc.); the slack guard adds timing
    // legality on top.
    const sta::Slack slack = getInstanceSlack(inst);
    if (slack <= config_.setup_slack_margin) {
      continue;
    }
    if (!resizer_.canRemoveBuffer(inst, /*honor_dont_touch_fixed=*/true)) {
      continue;
    }

    BatchEntry e;
    e.kind = BatchEntry::Kind::kUnbuffer;
    e.inst = inst;
    pending.push_back(e);
    if (static_cast<int>(pending.size()) >= batch_size_) {
      removed += applyBatchWithBisection(pending, pre_wns, pre_tns);
      pending.clear();
    }
  }
  if (!pending.empty()) {
    removed += applyBatchWithBisection(pending, pre_wns, pre_tns);
  }
  return removed;
}

sta::Slack PowerReliefPolicy::getInstanceSlack(sta::Instance* inst)
{
  sta::Slack worst_slack = sta::INF;
  auto pin_iter
      = std::unique_ptr<sta::InstancePinIterator>(network_->pinIterator(inst));
  while (pin_iter->hasNext()) {
    sta::Pin* pin = pin_iter->next();
    if (network_->direction(pin)->isAnyOutput()) {
      sta::Vertex* vertex = graph_->pinDrvrVertex(pin);
      if (vertex) {
        const sta::Slack pin_slack = sta_->slack(vertex, max_);
        worst_slack = std::min(worst_slack, pin_slack);
      }
    }
  }
  return worst_slack;
}

sta::Pin* PowerReliefPolicy::outputPin(sta::Instance* inst)
{
  auto pin_iter
      = std::unique_ptr<sta::InstancePinIterator>(network_->pinIterator(inst));
  while (pin_iter->hasNext()) {
    sta::Pin* pin = pin_iter->next();
    if (network_->direction(pin)->isAnyOutput()) {
      return pin;
    }
  }
  return nullptr;
}

std::optional<PowerReliefPolicy::BatchEntry> PowerReliefPolicy::findBestSwap(
    sta::Instance* inst,
    sta::Pin* drvr_pin,
    sta::Slack inst_slack)
{
  sta::Vertex* drvr_vertex = graph_->pinDrvrVertex(drvr_pin);
  if (drvr_vertex == nullptr) {
    return std::nullopt;
  }

  sta::Path* endpoint_path = sta_->vertexWorstSlackPath(drvr_vertex, max_);
  if (endpoint_path == nullptr) {
    return std::nullopt;
  }

  sta::PathExpanded expanded(endpoint_path, sta_);
  int path_index = -1;
  for (size_t i = 0; i < expanded.size(); ++i) {
    if (expanded.path(i)->vertex(sta_) == drvr_vertex) {
      path_index = static_cast<int>(i);
      break;
    }
  }
  if (path_index == -1) {
    return std::nullopt;
  }

  Target target = makePathDriverTarget(
      endpoint_path, expanded, path_index, inst_slack, resizer_);

  std::optional<ArcDelayState> arc_delay_opt
      = DelayEstimator::buildContext(resizer_, target, 0);
  if (!arc_delay_opt) {
    return std::nullopt;
  }
  const ArcDelayState& arc_delay = *arc_delay_opt;

  sta::LibertyCell* curr_cell = network_->libertyCell(inst);
  if (curr_cell == nullptr) {
    return std::nullopt;
  }
  const float curr_leak = cellLeakageValue(resizer_, curr_cell);
  const float curr_delay = arc_delay.target().current_model_delay;

  // Score a candidate by power saved per ps of slack consumed
  // (Δleakage / Δdelay). Free-or-faster candidates (Δdelay ≤ 0) get infinity
  // rank so they dominate. Returns nullopt for any candidate that cannot save
  // leakage or does not fit within the slack budget.
  auto score = [&](sta::LibertyCell* cand) -> std::optional<float> {
    if (cand == curr_cell) {
      return std::nullopt;
    }
    const float cand_leak = cellLeakageValue(resizer_, cand);
    const float delta_leak = curr_leak - cand_leak;
    if (delta_leak <= 0.0f) {
      return std::nullopt;
    }
    DelayEstimate est = DelayEstimator::estimate(arc_delay, cand);
    if (!est.legal) {
      return std::nullopt;
    }
    const float delta_delay = est.candidate_delay - curr_delay;
    if (delta_delay >= inst_slack) {
      return std::nullopt;
    }
    if (delta_delay <= 0.0f) {
      return std::numeric_limits<float>::infinity();
    }
    return delta_leak / delta_delay;
  };

  // VT-swap pool is preferred over size-down because VT-equiv cells preserve
  // area, drive strength, and C_in -- only the leakage / delay tradeoff
  // changes. Size-down also weakens the driver, which can propagate slew to
  // downstream gates (an effect the single-stage delay model here does not
  // see). Hence: best VT wins outright if any qualifies; size-down is the
  // fallback.
  sta::LibertyCellSeq vt_equiv_cells = resizer_.getVTEquivCells(curr_cell);
  std::optional<float> best_ratio;
  sta::LibertyCell* best_cell = nullptr;
  for (sta::LibertyCell* cand : vt_equiv_cells) {
    if (cand == curr_cell) {
      // VT pool is sorted by leakage ascending; cells beyond curr have >=
      // leakage and cannot save power.
      break;
    }
    std::optional<float> r = score(cand);
    if (!r.has_value()) {
      continue;
    }
    if (!best_ratio.has_value() || *r > *best_ratio) {
      best_ratio = r;
      best_cell = cand;
    }
  }
  if (best_cell != nullptr) {
    return BatchEntry{BatchEntry::Kind::kVtSwap,
                      inst,
                      drvr_pin,
                      curr_cell,
                      best_cell,
                      inst_slack};
  }

  // Size-down fallback: Scan all smaller-area swappable cells, score by
  // the same Δleakage / Δdelay ratio. Smaller area is the gate-electrical
  // pre-filter (size-down by definition); leakage saving still has to be
  // positive for the candidate to be admissible.
  sta::LibertyCellSeq swappable_cells = resizer_.getSwappableCells(curr_cell);
  for (sta::LibertyCell* cand : swappable_cells) {
    if (cand == curr_cell) {
      continue;
    }
    if (cand->area() >= curr_cell->area()) {
      continue;
    }
    std::optional<float> r = score(cand);
    if (!r.has_value()) {
      continue;
    }
    if (!best_ratio.has_value() || *r > *best_ratio) {
      best_ratio = r;
      best_cell = cand;
    }
  }
  if (best_cell != nullptr) {
    return BatchEntry{BatchEntry::Kind::kSizeDown,
                      inst,
                      drvr_pin,
                      curr_cell,
                      best_cell,
                      inst_slack};
  }
  return std::nullopt;
}

int PowerReliefPolicy::applyBatchWithBisection(
    const std::vector<BatchEntry>& entries,
    sta::Slack& pre_wns,
    sta::Slack& pre_tns)
{
  if (entries.empty()) {
    return 0;
  }

  committer_.beginJournal();
  for (const BatchEntry& e : entries) {
    switch (e.kind) {
      case BatchEntry::Kind::kVtSwap: {
        VtSwapCandidate cand(resizer_,
                             Target{},  // not used downstream of apply()
                             e.drvr_pin,
                             e.inst,
                             e.curr_cell,
                             e.target_cell);
        committer_.commit(cand);
        break;
      }
      case BatchEntry::Kind::kSizeDown: {
        SizeDownFanoutCandidate cand(resizer_,
                                     Target{},
                                     e.drvr_pin,
                                     e.inst,
                                     e.drvr_pin,
                                     e.curr_cell,
                                     e.target_cell,
                                     e.inst_slack);
        committer_.commit(cand);
        break;
      }
      case BatchEntry::Kind::kUnbuffer: {
        UnbufferCandidate cand(resizer_, Target{}, e.inst);
        committer_.commit(cand);
        break;
      }
    }
  }

  resizer_.updateParasiticsAndTiming();
  sta_->findRequireds();
  const sta::Slack post_wns = sta_->worstSlack(max_);
  const sta::Slack post_tns = sta_->totalNegativeSlack(max_);
  const bool wns_regress = sta::fuzzyLess(post_wns, pre_wns);
  const bool tns_regress = sta::fuzzyLess(post_tns, pre_tns);

  if (!wns_regress && !tns_regress) {
    committer_.commitJournal();
    pre_wns = post_wns;
    pre_tns = post_tns;
    return static_cast<int>(entries.size());
  }

  // Regression detected. Roll back the batch.
  committer_.restoreJournal();

  if (entries.size() == 1) {
    // Single move caused the regression -- discard and report. Unbuffer entries
    // have no curr_cell / target_cell, so format the dropped move with whatever
    // fields are populated.
    const BatchEntry& e = entries[0];
    const char* curr_name
        = (e.curr_cell != nullptr) ? e.curr_cell->name().c_str() : "(buffer)";
    const char* target_name
        = (e.target_cell != nullptr) ? e.target_cell->name().c_str() : "remove";
    debugPrint(logger_,
               RSZ,
               "power_relief",
               2,
               "Power Relief: Dropped {} -> {} (single move regressed "
               "WNS {} -> {}; TNS {} -> {})",
               curr_name,
               target_name,
               delayAsString(pre_wns, 3, sta_),
               delayAsString(post_wns, 3, sta_),
               delayAsString(pre_tns, 1, sta_),
               delayAsString(post_tns, 1, sta_));
    return 0;
  }

  // Bisect: Try first half, then second half, recursively. Each surviving
  // sub-batch ratchets pre_wns / pre_tns so subsequent halves see the updated
  // baseline.
  const size_t mid = entries.size() / 2;
  const std::vector<BatchEntry> first(entries.begin(), entries.begin() + mid);
  const std::vector<BatchEntry> second(entries.begin() + mid, entries.end());

  debugPrint(logger_,
             RSZ,
             "power_relief",
             2,
             "Power Relief: Batch of {} regressed; bisecting into {} + {}",
             entries.size(),
             first.size(),
             second.size());

  int kept = 0;
  kept += applyBatchWithBisection(first, pre_wns, pre_tns);
  kept += applyBatchWithBisection(second, pre_wns, pre_tns);
  return kept;
}

}  // namespace rsz
