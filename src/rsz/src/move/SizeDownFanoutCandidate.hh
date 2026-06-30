// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/NetworkClass.hh"

namespace sta {
class LibertyCell;
}  // namespace sta

namespace rsz {

// Candidate that replaces one non-critical load instance with a smaller cell.
//
// The generator pre-screens loads on the driver's net and picks a weaker
// cell that reduces area/leakage while staying within the available slack
// budget.
//
// estimate() reports the generator-computed local delay change as a
// delta_arrival.
//
// apply() performs the cell swap via Resizer::replaceCell.
class SizeDownFanoutCandidate : public MoveCandidate
{
 public:
  // === Construction =========================================================
  SizeDownFanoutCandidate(Resizer& resizer,
                          const Target& target,
                          sta::Pin* drvr_pin,
                          sta::Instance* inst,
                          sta::Pin* load_pin,
                          sta::LibertyCell* current_cell,
                          sta::LibertyCell* replacement,
                          sta::Slack slack,
                          float worst_delay_change);

  // === MoveCandidate API ====================================================
  Estimate estimate() override;
  MoveResult apply() override;
  MoveType type() const override { return MoveType::kSizeDownFanout; }

 private:
  // === Candidate state ======================================================
  sta::Pin* drvr_pin_{nullptr};
  sta::Instance* inst_{nullptr};
  sta::Pin* load_pin_{nullptr};
  sta::LibertyCell* current_cell_{nullptr};
  sta::LibertyCell* replacement_{nullptr};
  sta::Slack slack_{0.0};
  float worst_delay_change_{0.0f};
};

}  // namespace rsz
