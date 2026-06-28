// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "PinDensityMap.hh"

#include <memory>
#include <utility>
#include <vector>

#include "DesignStateMap.hh"
#include "odb/db.h"
#include "odb/dbTransform.h"
#include "odb/geom.h"
#include "rsz/Resizer.hh"

namespace rsz {

PinDensityMapBuilder::PinDensityMapBuilder(Resizer* resizer)
    : DesignStateMapBuilder(resizer)
{
}

bool PinDensityMapBuilder::available() const
{
  return resizer_->block() != nullptr;
}

std::unique_ptr<DesignStateMap>
PinDensityMapBuilder::build(const odb::Rect& bounds, int bins_x, int bins_y)
{
  odb::dbBlock* block = resizer_->block();
  if (block == nullptr) {
    return nullptr;
  }

  auto map = makeUniformMap(bounds, bins_x, bins_y);

  // Walk the hierarchy, flattening child blocks through their transforms.
  std::vector<std::pair<odb::dbBlock*, odb::dbTransform>> blocks
      = {{block, odb::dbTransform()}};

  while (!blocks.empty()) {
    auto [cur_block, transform] = blocks.back();
    blocks.pop_back();

    for (odb::dbInst* inst : cur_block->getInsts()) {
      if (!inst->getPlacementStatus().isPlaced()) {
        continue;
      }
      if (inst->isHierarchical()) {
        odb::dbTransform child_transform = inst->getTransform();
        child_transform.concat(transform);
        blocks.emplace_back(inst->getChild(), child_transform);
        continue;
      }

      for (odb::dbITerm* iterm : inst->getITerms()) {
        if (iterm->getSigType().isSupply()) {
          continue;
        }
        odb::Rect bbox;
        bbox.mergeInit();
        for (auto& [layer, geom_bbox] : iterm->getGeometries()) {
          bbox.merge(geom_bbox);
        }
        if (bbox.isInverted()) {
          continue;
        }
        transform.apply(bbox);
        // Count the pin once, at the center of its access geometry.
        const odb::Point center((bbox.xMin() + bbox.xMax()) / 2,
                                (bbox.yMin() + bbox.yMax()) / 2);
        map->addPoint(center, 1.0);
      }
    }
  }

  return map;
}

}  // namespace rsz
