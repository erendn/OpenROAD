// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "odb/geom.h"
#include "rsz/Resizer.hh"

namespace sta {
class Instance;
class Pin;
}  // namespace sta

namespace rsz {

class RelocateCandidate : public MoveCandidate
{
 public:
  RelocateCandidate(Resizer& resizer,
                    const Target& target,
                    sta::Instance* drvr_inst,
                    sta::Pin* drvr_pin,
                    const odb::Point& orig_loc,
                    const odb::Point& requested_loc,
                    float setup_slack_margin,
                    bool legalize,
                    int max_skew_dbu);

  Estimate estimate() override;
  MoveResult apply() override;
  MoveType type() const override { return MoveType::kRelocate; }

  // Default cap (DBU) on the Manhattan skew between the requested target and
  // the legalized location dpl's legalCellPos returns. Past this the snap has
  // pulled the cell too far from the slack-guided target to be meaningful.
  // Overridable via RSZ_RELOCATE_MAX_SKEW_DBU.
  static constexpr int kPlacementDisplacementLimitDbu = 10000;

 private:
  sta::Instance* drvr_inst_;
  sta::Pin* drvr_pin_;
  odb::Point orig_loc_;
  odb::Point requested_loc_;
  float setup_slack_margin_;
  // Snap the target via dpl legalCellPos (else use the raw clamp-to-core loc).
  bool legalize_;
  // Reject if legalization moves the target more than this (DBU) off request.
  int max_skew_dbu_;
};

}  // namespace rsz
