// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include "SetupLegacyBase.hh"

namespace rsz {

// Experimental phase that repairs "common violators" before the per-endpoint
// legacy phases run. A common violator is a driver pin crossed by the worst
// paths of multiple violating endpoints; one good move there improves many
// endpoints at once.
//
// Per sweep, pins are ranked by the sum of slack of the endpoints whose worst
// path crosses them (most negative first) and repaired one journaled move at a
// time. Each move is accepted on global TNS improvement (WNS as tiebreak)
// rather than a single endpoint's slack, so moves that help many endpoints
// cannot be rejected for not helping a focus endpoint. The pin pool is
// re-collected after every committed move because committed topology moves
// (unbuffer, rebuffer) can delete instances backing pins collected earlier.
class SetupCommonViolatorsPolicy : public SetupLegacyBase
{
 public:
  SetupCommonViolatorsPolicy(Resizer& resizer,
                             MoveCommitter& committer,
                             RepairSetupContext& setup_context,
                             const OptimizerRunConfig& config);

  const char* name() const override { return "SetupCommonViolatorsPolicy"; }
  void iterate() override;

 private:
  void repairCommonViolators();
  // Repair one pin under its own journal; returns true when a move was
  // committed (accepted on global TNS/WNS improvement).
  bool repairPinJournaled(const sta::Pin* pin,
                          sta::Slack& tns,
                          sta::Slack& wns,
                          char phase_marker);
};

}  // namespace rsz
