// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <string>
#include <vector>

#include "SetupLegacyBase.hh"

namespace rsz {

// Experimental phase that repairs high-leverage structural bottleneck gates
// before per-endpoint legacy repair phases run. A bottleneck is a driver pin
// crossed by the worst paths of multiple violating endpoints; one good move
// there improves many endpoints at once.
class SetupBottleneckPolicy : public SetupLegacyBase
{
 public:
  SetupBottleneckPolicy(Resizer& resizer,
                        MoveCommitter& committer,
                        RepairSetupContext& setup_context,
                        const OptimizerRunConfig& config);

  const char* name() const override { return "SetupBottleneckPolicy"; }
  void iterate() override;

 private:
  enum class PinRepairResult
  {
    kAccepted,
    kRejected,
    kSkipped
  };

  struct CandidatePin
  {
    const sta::Pin* pin{nullptr};
    std::string name;
  };

  void repairBottlenecks();
  bool runSweep(int sweep, char phase_marker, sta::Slack& tns, sta::Slack& wns);
  int processSweepPins(const std::vector<CandidatePin>& candidates,
                       char phase_marker,
                       int sweep,
                       sta::Slack& tns,
                       sta::Slack& wns);
  PinRepairResult repairPinJournaled(const sta::Pin* pin,
                                     sta::Slack& tns,
                                     sta::Slack& wns,
                                     char phase_marker);
  bool hasInsufficientTnsGain(sta::Slack sweep_start_tns,
                              sta::Slack current_tns) const;
  void getCurrentTiming(sta::Slack& tns, sta::Slack& wns) const;
};

}  // namespace rsz
