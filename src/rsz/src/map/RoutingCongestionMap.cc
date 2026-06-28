// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "RoutingCongestionMap.hh"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "DesignStateMap.hh"
#include "odb/db.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"
#include "rsz/Resizer.hh"

namespace rsz {

RoutingCongestionMapBuilder::RoutingCongestionMapBuilder(Resizer* resizer)
    : DesignStateMapBuilder(resizer)
{
}

bool RoutingCongestionMapBuilder::available() const
{
  odb::dbBlock* block = resizer_->block();
  return block != nullptr && block->getGCellGrid() != nullptr;
}

std::unique_ptr<DesignStateMap> RoutingCongestionMapBuilder::build(
    const odb::Rect& /* bounds */,
    int /* bins_x */,
    int /* bins_y */)
{
  odb::dbBlock* block = resizer_->block();
  if (block == nullptr) {
    return nullptr;
  }
  odb::dbGCellGrid* grid = block->getGCellGrid();
  if (grid == nullptr) {
    return nullptr;
  }

  const auto hor_congestion
      = grid->getDirectionCongestionMap(odb::dbTechLayerDir::HORIZONTAL);
  const auto ver_congestion
      = grid->getDirectionCongestionMap(odb::dbTechLayerDir::VERTICAL);
  if (hor_congestion.numElems() == 0 || ver_congestion.numElems() == 0) {
    return nullptr;
  }

  // Bin edges follow the GCell grid lines, with the die edge closing the last
  // column/row (matching the GUI routing-congestion heatmap).
  std::vector<int> x_grid;
  std::vector<int> y_grid;
  grid->getGridX(x_grid);
  grid->getGridY(y_grid);
  if (x_grid.empty() || y_grid.empty()) {
    return nullptr;
  }
  std::vector<int> x_edges(x_grid.begin(), x_grid.end());
  x_edges.push_back(block->getDieArea().xMax());
  std::vector<int> y_edges(y_grid.begin(), y_grid.end());
  y_edges.push_back(block->getDieArea().yMax());

  auto map = std::make_unique<DesignStateMap>(
      name(), unit(), std::move(x_edges), std::move(y_edges));

  for (uint32_t x_idx = 0; x_idx < hor_congestion.numRows(); ++x_idx) {
    for (uint32_t y_idx = 0; y_idx < hor_congestion.numCols(); ++y_idx) {
      const auto& hor = hor_congestion(x_idx, y_idx);
      const auto& ver = ver_congestion(x_idx, y_idx);

      // -1 means capacity is not well defined for that direction.
      const double hor_cong
          = hor.capacity != 0 ? static_cast<double>(hor.usage) / hor.capacity
                              : -1.0;
      const double ver_cong
          = ver.capacity != 0 ? static_cast<double>(ver.usage) / ver.capacity
                              : -1.0;
      const double congestion = std::max(hor_cong, ver_cong);
      if (congestion < 0.0) {
        continue;
      }

      const int next_x = (x_idx + 1 == x_grid.size())
                             ? block->getDieArea().xMax()
                             : x_grid[x_idx + 1];
      const int next_y = (y_idx + 1 == y_grid.size())
                             ? block->getDieArea().yMax()
                             : y_grid[y_idx + 1];
      const odb::Rect gcell_rect(x_grid[x_idx], y_grid[y_idx], next_x, next_y);
      map->addToMap(gcell_rect, congestion * 100.0);
    }
  }

  return map;
}

}  // namespace rsz
