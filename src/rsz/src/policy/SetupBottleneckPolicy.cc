// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "SetupBottleneckPolicy.hh"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "MoveCommitter.hh"
#include "OptimizerTypes.hh"
#include "RepairTargetCollector.hh"
#include "est/EstimateParasitics.h"
#include "policy/OptimizationPolicy.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Fuzzy.hh"
#include "sta/GraphClass.hh"
#include "sta/NetworkClass.hh"
#include "utl/Logger.h"
#include "utl/timer.h"

namespace rsz {

using utl::RSZ;

namespace {
constexpr int kDelayDigits = 3;

bool isTimingBetter(sta::Slack new_tns,
                    sta::Slack old_tns,
                    sta::Slack new_wns,
                    sta::Slack old_wns)
{
  return sta::fuzzyGreater(new_tns, old_tns)
         || (sta::fuzzyEqual(new_tns, old_tns)
             && sta::fuzzyGreater(new_wns, old_wns));
}
}  // namespace

SetupBottleneckPolicy::SetupBottleneckPolicy(Resizer& resizer,
                                             MoveCommitter& committer,
                                             RepairSetupContext& setup_context,
                                             const OptimizerRunConfig& config)
    : SetupLegacyBase(resizer, committer, setup_context, config)
{
  is_experimental = true;
}

void SetupBottleneckPolicy::iterate()
{
  buildMainMoveSequence(/*log_sequence=*/false);
  repairBottlenecks();
  committer_.printTrackerPhaseSummary(
      "Bottleneck Phase Summary", "Bottleneck Endpoint Profiler", true);
  markRunComplete(true);
}

void SetupBottleneckPolicy::repairBottlenecks()
{
  int& opto_iteration = setup_context_.iteration;
  const char phase_marker = phaseMarkerForIndex(setup_context_.phase_index);
  const utl::DebugScopedTimer timer(
      logger_,
      RSZ,
      "repair_setup",
      10,
      fmt::format("BOTTLENECK{} Phase Time: {{}}", phase_marker));
  committer_.capturePrePhaseSlack();
  // Tracker attribution: This phase has no focus endpoint
  committer_.setCurrentEndpoint(nullptr);
  rejected_pin_moves_current_endpoint_.clear();

  printProgress(opto_iteration, false, phase_marker);

  sta::Slack wns = 0.0;
  sta::Slack tns = 0.0;
  getCurrentTiming(tns, wns);

  const int max_sweeps = policy_config_.bottleneck_max_sweeps;
  for (int sweep = 1; sweep <= max_sweeps; sweep++) {
    const bool continue_sweeps = runSweep(sweep, phase_marker, tns, wns);
    if (!continue_sweeps) {
      break;
    }
  }

  printProgress(opto_iteration, true, phase_marker);
  // Leave an accurate fix-rate baseline for downstream phases.
  setup_context_.previous_tns = sta::delayAsFloat(totalNegativeSlack(max_));
}

bool SetupBottleneckPolicy::runSweep(int sweep,
                                     char phase_marker,
                                     sta::Slack& tns,
                                     sta::Slack& wns)
{
  const int min_path_count
      = std::max(1, policy_config_.bottleneck_min_path_count);
  const int max_endpoints = policy_config_.bottleneck_max_endpoints;

  const std::vector<const sta::Pin*> pins
      = target_collector_->collectBottlenecks(min_path_count, max_endpoints);

  if (sweep == 1) {
    target_collector_->reportBottlenecks(min_path_count);
  }

  if (pins.empty()) {
    return false;
  }

  if (policy_config_.bottleneck_report_only) {
    debugPrint(logger_,
               RSZ,
               "repair_setup",
               1,
               "BOTTLENECK{} Phase: report-only mode, no repairs",
               phase_marker);
    return false;
  }

  std::vector<CandidatePin> candidates;
  candidates.reserve(pins.size());
  for (const sta::Pin* pin : pins) {
    candidates.push_back({pin, network_->pathName(pin)});
  }

  const sta::Slack sweep_start_tns = tns;
  const int accepted
      = processSweepPins(candidates, phase_marker, sweep, tns, wns);

  if (resizer_.overMaxArea() || accepted == 0) {
    return false;
  }

  if (hasInsufficientTnsGain(sweep_start_tns, tns)) {
    debugPrint(
        logger_,
        RSZ,
        "repair_setup",
        2,
        "BOTTLENECK{} Phase: aborting after sweep {} due to low relative TNS "
        "gain",
        phase_marker,
        sweep);
    return false;
  }

  return true;
}

int SetupBottleneckPolicy::processSweepPins(
    const std::vector<CandidatePin>& candidates,
    char phase_marker,
    int sweep,
    sta::Slack& tns,
    sta::Slack& wns)
{
  const int top_pins = policy_config_.bottleneck_top_pins;
  const int max_rejections = policy_config_.bottleneck_max_rejections;

  int attempts = 0;
  int accepted = 0;
  int consecutive_rejections = 0;

  for (const CandidatePin& candidate : candidates) {
    if (top_pins > 0 && attempts >= top_pins) {
      break;
    }
    if (network_->findPin(candidate.name) == candidate.pin) {
      const PinRepairResult result
          = repairPinJournaled(candidate.pin, tns, wns, phase_marker);
      if (result == PinRepairResult::kAccepted) {
        attempts++;
        accepted++;
        consecutive_rejections = 0;
      } else if (result == PinRepairResult::kRejected) {
        attempts++;
        consecutive_rejections++;
        if (max_rejections > 0 && consecutive_rejections >= max_rejections) {
          debugPrint(
              logger_,
              RSZ,
              "repair_setup",
              2,
              "BOTTLENECK{} Phase: aborting sweep {} after {} consecutive "
              "rejections",
              phase_marker,
              sweep,
              consecutive_rejections);
          break;
        }
      }
    }

    if (resizer_.overMaxArea()) {
      break;
    }
  }

  debugPrint(logger_,
             RSZ,
             "repair_setup",
             1,
             "BOTTLENECK{} Phase: Sweep {} attempted {} pins, "
             "accepted {} moves, TNS {}",
             phase_marker,
             sweep,
             attempts,
             accepted,
             delayAsString(tns, 1, sta_));

  return accepted;
}

SetupBottleneckPolicy::PinRepairResult
SetupBottleneckPolicy::repairPinJournaled(const sta::Pin* pin,
                                          sta::Slack& tns,
                                          sta::Slack& wns,
                                          char phase_marker)
{
  int& opto_iteration = setup_context_.iteration;
  sta::Vertex* drvr_vertex = graph_->pinDrvrVertex(pin);
  if (drvr_vertex == nullptr) {
    return PinRepairResult::kSkipped;
  }
  const sta::Slack focus_slack = sta_->slack(drvr_vertex, max_);
  if (sta::fuzzyGreaterEqual(focus_slack, config_.setup_slack_margin)) {
    return PinRepairResult::kSkipped;
  }

  committer_.beginJournal();
  Target target;
  if (!makePinTarget(pin, focus_slack, target)) {
    committer_.restoreJournal();
    return PinRepairResult::kRejected;
  }
  committer_.trackViolatorWithTimingInfo(target.driver_pin,
                                         target.vertex(resizer_),
                                         focus_slack,
                                         *target_collector_);

  const std::unordered_set<MoveType>* rejected_types = nullptr;
  auto rejected_itr = rejected_pin_moves_current_endpoint_.find(pin);
  if (rejected_itr != rejected_pin_moves_current_endpoint_.end()) {
    rejected_types = &rejected_itr->second;
  }

  int changed = 0;
  std::optional<MoveType> accepted_type;
  logRepairTarget(target);
  tryRepairTarget(
      target, /*repairs_per_pass=*/1, changed, rejected_types, accepted_type);
  if (changed == 0) {
    committer_.restoreJournal();
    return PinRepairResult::kRejected;
  }

  estimate_parasitics_->updateParasitics();
  sta_->findRequireds();

  sta::Slack new_wns = 0.0;
  sta::Slack new_tns = 0.0;
  getCurrentTiming(new_tns, new_wns);

  const bool better = isTimingBetter(new_tns, tns, new_wns, wns);

  debugPrint(logger_,
             RSZ,
             "repair_setup",
             3,
             "BOTTLENECK{}: pin {} move {} {} (TNS {} -> {}, WNS {} -> {})",
             phase_marker,
             network_->pathName(pin),
             accepted_type.has_value() ? moveName(*accepted_type) : "none",
             better ? "accepted" : "rejected",
             delayAsString(tns, 1, sta_),
             delayAsString(new_tns, 1, sta_),
             delayAsString(wns, kDelayDigits, sta_),
             delayAsString(new_wns, kDelayDigits, sta_));

  opto_iteration++;
  if (better) {
    committer_.commitJournal();
    tns = new_tns;
    wns = new_wns;
    printProgress(opto_iteration, false, phase_marker);
    return PinRepairResult::kAccepted;
  }

  committer_.restoreJournal();
  if (accepted_type.has_value()) {
    rejected_pin_moves_current_endpoint_[pin].insert(*accepted_type);
  }
  printProgress(opto_iteration, false, phase_marker);
  return PinRepairResult::kRejected;
}

bool SetupBottleneckPolicy::hasInsufficientTnsGain(sta::Slack sweep_start_tns,
                                                   sta::Slack current_tns) const
{
  if (policy_config_.bottleneck_min_tns_gain <= 0.0) {
    return false;
  }
  if (sta::fuzzyGreaterEqual(sweep_start_tns, 0.0)) {
    return true;
  }
  const double delta_tns = current_tns - sweep_start_tns;
  if (delta_tns <= 0.0) {
    return true;
  }
  const double rel_gain = delta_tns / std::abs(sweep_start_tns);
  return rel_gain < policy_config_.bottleneck_min_tns_gain;
}

void SetupBottleneckPolicy::getCurrentTiming(sta::Slack& tns,
                                             sta::Slack& wns) const
{
  tns = totalNegativeSlack(max_);
  wns = 0.0;
  sta::Vertex* worst_vertex = nullptr;
  sta_->worstSlack(max_, wns, worst_vertex);
}

}  // namespace rsz
