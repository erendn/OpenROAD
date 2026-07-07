// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "SetupCommonViolatorsPolicy.hh"

#include <algorithm>
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
}  // namespace

SetupCommonViolatorsPolicy::SetupCommonViolatorsPolicy(
    Resizer& resizer,
    MoveCommitter& committer,
    RepairSetupContext& setup_context,
    const OptimizerRunConfig& config)
    : SetupLegacyBase(resizer, committer, setup_context, config)
{
  is_experimental = true;
}

void SetupCommonViolatorsPolicy::iterate()
{
  buildMainMoveSequence(/*log_sequence=*/false);
  repairCommonViolators();
  committer_.printTrackerPhaseSummary("Common Violators Phase Summary",
                                      "Common Violators Endpoint Profiler",
                                      true);
  markRunComplete(true);
}

void SetupCommonViolatorsPolicy::repairCommonViolators()
{
  int& opto_iteration = setup_context_.iteration;
  const char phase_marker = phaseMarkerForIndex(setup_context_.phase_index);
  const utl::DebugScopedTimer timer(
      logger_,
      RSZ,
      "repair_setup",
      10,
      fmt::format("COMMON_VIOLATORS{} Phase Time: {{}}", phase_marker));
  committer_.capturePrePhaseSlack();
  // Tracker attribution: This phase has no focus endpoint
  committer_.setCurrentEndpoint(nullptr);
  rejected_pin_moves_current_endpoint_.clear();

  const int min_path_count = std::max(1, policy_config_.common_min_path_count);
  const int max_endpoints = policy_config_.common_max_endpoints;
  const int max_sweeps = std::max(1, policy_config_.common_max_sweeps);
  const int top_pins = policy_config_.common_top_pins;

  printProgress(opto_iteration, false, phase_marker);

  sta::Slack wns = 0.0;
  sta::Vertex* worst_vertex = nullptr;
  sta_->worstSlack(max_, wns, worst_vertex);
  sta::Slack tns = totalNegativeSlack(max_);

  for (int sweep = 1; sweep <= max_sweeps; sweep++) {
    std::vector<const sta::Pin*> pins
        = target_collector_->collectCommonViolators(min_path_count,
                                                    max_endpoints);
    if (sweep == 1) {
      target_collector_->reportCommonViolators(min_path_count);
    }
    if (pins.empty()) {
      break;
    }
    if (policy_config_.common_report_only) {
      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 1,
                 "COMMON_VIOLATORS{} Phase: report-only mode, no repairs",
                 phase_marker);
      break;
    }

    // Names captured while every pool pin is alive; each pin is re-resolved
    // by name before repair because committed unbuffer/rebuffer moves can
    // delete instances collected earlier in the sweep (rebuffer removes
    // existing buffers that were never a move's subject).  The ranking is
    // allowed to go stale within a sweep; the next sweep re-collects.
    std::vector<std::string> pin_names;
    pin_names.reserve(pins.size());
    for (const sta::Pin* pin : pins) {
      pin_names.emplace_back(network_->pathName(pin));
    }

    int attempts = 0;
    int accepted = 0;
    for (size_t index = 0; index < pins.size(); index++) {
      if (top_pins > 0 && attempts >= top_pins) {
        break;
      }
      const sta::Pin* pin = pins[index];
      // Liveness guard: skip pins deleted by an earlier committed move this
      // sweep (findPin misses) without dereferencing the stale pointer.
      if (network_->findPin(pin_names[index]) != pin) {
        continue;
      }
      attempts++;

      if (repairPinJournaled(pin, tns, wns, phase_marker)) {
        accepted++;
      }

      if (resizer_.overMaxArea()) {
        printProgress(opto_iteration, true, phase_marker);
        return;
      }
    }

    debugPrint(logger_,
               RSZ,
               "repair_setup",
               1,
               "COMMON_VIOLATORS{} Phase: Sweep {} attempted {} pins, "
               "accepted {} moves, TNS {}",
               phase_marker,
               sweep,
               attempts,
               accepted,
               delayAsString(tns, 1, sta_));

    if (accepted == 0) {
      break;
    }
  }

  printProgress(opto_iteration, true, phase_marker);
  // Leave an accurate fix-rate baseline for the downstream phases.
  setup_context_.previous_tns = sta::delayAsFloat(totalNegativeSlack(max_));
}

bool SetupCommonViolatorsPolicy::repairPinJournaled(const sta::Pin* pin,
                                                    sta::Slack& tns,
                                                    sta::Slack& wns,
                                                    const char phase_marker)
{
  int& opto_iteration = setup_context_.iteration;
  sta::Vertex* drvr_vertex = graph_->pinDrvrVertex(pin);
  if (drvr_vertex == nullptr) {
    return false;
  }
  // The pin's own worst slack, not a focus endpoint's slack: makePinTarget
  // builds the target on this pin's worst path.
  const sta::Slack focus_slack = sta_->slack(drvr_vertex, max_);
  // Stale-ranking guard: earlier accepted moves this sweep may have already
  // fixed this pin's paths; a move here cannot improve TNS, so skip it.
  if (sta::fuzzyGreaterEqual(focus_slack, config_.setup_slack_margin)) {
    return false;
  }

  committer_.beginJournal();
  Target target;
  if (!makePinTarget(pin, focus_slack, target)) {
    committer_.commitJournal();  // Empty commit; no ECO changes pending.
    return false;
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
    committer_.commitJournal();
    return false;
  }

  estimate_parasitics_->updateParasitics();
  sta_->findRequireds();

  const sta::Slack new_tns = totalNegativeSlack(max_);
  sta::Slack new_wns = 0.0;
  sta::Vertex* worst_vertex = nullptr;
  sta_->worstSlack(max_, new_wns, worst_vertex);

  // Accept on global TNS improvement; WNS breaks TNS ties. This is the phase's
  // core rule: a move helping many endpoints cannot be rejected for failing to
  // help one focus endpoint.
  const bool better
      = sta::fuzzyGreater(new_tns, tns)
        || (sta::fuzzyEqual(new_tns, tns) && sta::fuzzyGreater(new_wns, wns));

  debugPrint(logger_,
             RSZ,
             "repair_setup",
             3,
             "COMMON_VIOLATORS{}: pin {} move {} {} (TNS {} -> {}, WNS {} -> "
             "{})",
             phase_marker,
             network_->pathName(pin),
             accepted_type.has_value() ? moveName(*accepted_type) : "none",
             better ? "accepted" : "rejected",
             delayAsString(tns, 1, sta_),
             delayAsString(new_tns, 1, sta_),
             delayAsString(wns, kDelayDigits, sta_),
             delayAsString(new_wns, kDelayDigits, sta_));

  opto_iteration++;
  bool committed = false;
  if (better) {
    committer_.commitJournal();
    tns = new_tns;
    wns = new_wns;
    committed = true;
  } else {
    committer_.restoreJournal();
    if (accepted_type.has_value()) {
      rejected_pin_moves_current_endpoint_[pin].insert(*accepted_type);
    }
  }
  printProgress(opto_iteration, false, phase_marker);
  return committed;
}

}  // namespace rsz
