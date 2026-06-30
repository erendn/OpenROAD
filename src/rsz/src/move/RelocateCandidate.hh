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
                    float setup_slack_margin);

  Estimate estimate() override;
  MoveResult apply() override;
  MoveType type() const override { return MoveType::kRelocate; }

  // Past this Manhattan skew (DBU) between the requested target and the
  // pixel dpl's diamond search actually returned, the neighborhood was too
  // crowded to honor the target intent and the resulting placement is no
  // longer guided by the slack signal.
  static constexpr int kPlacementDisplacementLimitDbu = 10000;

 private:
  sta::Instance* drvr_inst_;
  sta::Pin* drvr_pin_;
  odb::Point orig_loc_;
  odb::Point requested_loc_;
  float setup_slack_margin_;
};

}  // namespace rsz
