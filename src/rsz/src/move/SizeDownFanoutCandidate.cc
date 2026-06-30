// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "SizeDownFanoutCandidate.hh"

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

SizeDownFanoutCandidate::SizeDownFanoutCandidate(Resizer& resizer,
                                                 const Target& target,
                                                 sta::Pin* drvr_pin,
                                                 sta::Instance* inst,
                                                 sta::Pin* load_pin,
                                                 sta::LibertyCell* current_cell,
                                                 sta::LibertyCell* replacement,
                                                 sta::Slack slack,
                                                 const float worst_delay_change)
    : MoveCandidate(resizer, target),
      drvr_pin_(drvr_pin),
      inst_(inst),
      load_pin_(load_pin),
      current_cell_(current_cell),
      replacement_(replacement),
      slack_(slack),
      worst_delay_change_(worst_delay_change)
{
}

Estimate SizeDownFanoutCandidate::estimate()
{
  // SizeDownFanoutGenerator already computed the move's net local delay change
  // for the chosen cell. A positive worst_delay_change_ means a stage slowed
  // down, so the arrival delta is its negation -- typically negative, since a
  // size-down spends timing for area/leakage. Feasibility (delay budget) was
  // already vetted by the generator, so `legal` stays true.
  return {.legal = true,
          .score = -worst_delay_change_,
          .delta_arrival = -worst_delay_change_,
          .scope = EstimateScope::kLocal,
          .estimated = true};
}

MoveResult SizeDownFanoutCandidate::apply()
{
  if (!resizer_.replaceCell(inst_, replacement_)) {
    debugPrint(resizer_.logger(),
               RSZ,
               "size_down_fanout_move",
               3,
               "REJECT SizeDownFanoutMove {} -> {}: ({} -> {}) slack={}",
               resizer_.network()->pathName(drvr_pin_),
               resizer_.network()->pathName(load_pin_),
               current_cell_->name(),
               replacement_->name(),
               delayAsString(slack_, 3, resizer_.staState()));
    return rejectedMove();
  }

  debugPrint(resizer_.logger(),
             RSZ,
             "size_down_fanout_move",
             3,
             "ACCEPT SizeDownFanoutMove {} -> {}: ({} -> {}) slack={}",
             resizer_.network()->pathName(drvr_pin_),
             resizer_.network()->pathName(load_pin_),
             current_cell_->name(),
             replacement_->name(),
             delayAsString(slack_, 3, resizer_.staState()));

  return {
      .accepted = true,
      .type = MoveType::kSizeDownFanout,
      .move_count = 1,
      .touched_instances = {inst_},
  };
}

}  // namespace rsz
