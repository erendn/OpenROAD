// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "PlacementDensityMap.hh"

#include <memory>
#include <utility>
#include <vector>

#include "DesignStateMap.hh"
#include "odb/db.h"
#include "odb/dbTransform.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"
#include "rsz/Resizer.hh"

namespace rsz {

PlacementDensityMapBuilder::PlacementDensityMapBuilder(Resizer* resizer)
    : DesignStateMapBuilder(resizer)
{
}

bool PlacementDensityMapBuilder::available() const
{
  return resizer_->block() != nullptr;
}

std::unique_ptr<DesignStateMap> PlacementDensityMapBuilder::build(
    const odb::Rect& bounds,
    int bins_x,
    int bins_y)
{
  odb::dbBlock* block = resizer_->block();
  if (block == nullptr) {
    return nullptr;
  }

  auto map = makeUniformMap(bounds, bins_x, bins_y);

  // Walk the hierarchy, flattening child blocks through their transforms so a
  // hierarchical design is handled the same as a flat one.
  std::vector<std::pair<odb::dbBlock*, odb::dbTransform>> blocks
      = {{block, odb::dbTransform()}};

  while (!blocks.empty()) {
    auto [cur_block, transform] = blocks.back();
    blocks.pop_back();

    for (odb::dbInst* inst : cur_block->getInsts()) {
      if (!inst->getPlacementStatus().isPlaced()) {
        continue;
      }
      odb::dbMaster* master = inst->getMaster();
      if (!include_filler_ && master->isFiller()) {
        continue;
      }
      if (!include_taps_
          && (master->getType() == odb::dbMasterType::CORE_WELLTAP
              || master->isEndCap())) {
        continue;
      }

      if (inst->isHierarchical()) {
        odb::dbTransform child_transform = inst->getTransform();
        child_transform.concat(transform);
        blocks.emplace_back(inst->getChild(), child_transform);
        continue;
      }

      odb::Rect inst_box = inst->getBBox()->getBox();
      transform.apply(inst_box);
      map->addToMap(inst_box, 100.0);
    }
  }

  return map;
}

}  // namespace rsz
