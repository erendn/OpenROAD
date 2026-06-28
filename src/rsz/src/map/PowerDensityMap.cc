// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "PowerDensityMap.hh"

#include <memory>

#include "DesignStateMap.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "odb/geom.h"
#include "rsz/Resizer.hh"
#include "sta/PowerClass.hh"
#include "sta/Scene.hh"

namespace rsz {

PowerDensityMapBuilder::PowerDensityMapBuilder(Resizer* resizer)
    : DesignStateMapBuilder(resizer)
{
}

bool PowerDensityMapBuilder::available() const
{
  return resizer_->block() != nullptr && resizer_->sta() != nullptr
         && resizer_->sta()->cmdScene() != nullptr;
}

std::unique_ptr<DesignStateMap>
PowerDensityMapBuilder::build(const odb::Rect& bounds, int bins_x, int bins_y)
{
  odb::dbBlock* block = resizer_->block();
  sta::dbSta* sta = resizer_->sta();
  if (block == nullptr || sta == nullptr) {
    return nullptr;
  }
  sta::Scene* scene = sta->cmdScene();
  if (scene == nullptr) {
    return nullptr;
  }
  sta::dbNetwork* network = resizer_->dbNetwork();

  auto map = makeUniformMap(bounds, bins_x, bins_y);

  constexpr double kWattsToMicrowatts = 1e6;
  for (odb::dbInst* inst : block->getInsts()) {
    if (!inst->getPlacementStatus().isPlaced() || inst->isHierarchical()) {
      continue;
    }
    const sta::PowerResult power = sta->power(network->dbToSta(inst), scene);
    const double pwr = power.total();
    if (pwr <= 0.0) {
      continue;
    }
    const odb::Rect box = inst->getBBox()->getBox();
    const odb::Point center((box.xMin() + box.xMax()) / 2,
                            (box.yMin() + box.yMax()) / 2);
    map->addPoint(center, pwr * kWattsToMicrowatts);
  }

  return map;
}

}  // namespace rsz
