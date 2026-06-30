// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "RerouteCandidate.hh"

#include <vector>

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "est/EstimateParasitics.h"
#include "grt/GlobalRouter.h"
#include "odb/db.h"
#include "rsz/Resizer.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/MinMax.hh"
#include "sta/Network.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

RerouteCandidate::RerouteCandidate(Resizer& resizer,
                                   const Target& target,
                                   sta::Pin* driver_pin,
                                   sta::Instance* driver_inst,
                                   odb::dbNet* db_net,
                                   const float current_resistance,
                                   const float estimated_resistance)
    : MoveCandidate(resizer, target),
      driver_pin_(driver_pin),
      driver_inst_(driver_inst),
      db_net_(db_net),
      current_resistance_(current_resistance),
      estimated_resistance_(estimated_resistance)
{
}

Estimate RerouteCandidate::estimate()
{
  // Convert the wire resistance reduction into a path-arrival delta with a
  // lumped Elmore step (delta_R * C_load). Coarse on purpose -- a real
  // distributed-RC wire delay would account for the resistance being
  // distributed along the routing tree -- but it produces an honest
  // seconds-valued estimate for ranking.
  const sta::Scene* scene = target_.activeScene(resizer_);
  const sta::MinMax* min_max = target_.minMax(resizer_);
  float delta_arrival = 0.0f;
  if (scene != nullptr && min_max != nullptr) {
    const float load_cap = resizer_.sta()->graphDelayCalc()->loadCap(
        driver_pin_, scene, min_max);
    delta_arrival = (current_resistance_ - estimated_resistance_) * load_cap;
  }
  return {.legal = true,
          .score = delta_arrival,
          .delta_arrival = delta_arrival,
          .scope = EstimateScope::kLocal,
          .estimated = true};
}

MoveResult RerouteCandidate::apply()
{
  grt::GlobalRouter* global_router = resizer_.globalRouter();
  if (global_router->isNetResAware(db_net_)) {
    return rejectedMove();
  }

  global_router->setResistanceAware(true);
  global_router->addDirtyNet(db_net_);
  global_router->setNetIsResAware(db_net_, true);
  resizer_.estimateParasitics()->parasiticsInvalid(db_net_);

  debugPrint(resizer_.logger(),
             RSZ,
             "reroute_move",
             1,
             "ACCEPT RerouteMove {}: Rerouted net {} (resistance {} -> {} "
             "estimated)",
             resizer_.network()->pathName(driver_pin_),
             db_net_->getName(),
             current_resistance_,
             estimated_resistance_);
  return {
      .accepted = true,
      .type = MoveType::kReroute,
      .move_count = 1,
      .touched_instances = {driver_inst_},
  };
}

}  // namespace rsz
