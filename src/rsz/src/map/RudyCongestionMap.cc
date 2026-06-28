// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "RudyCongestionMap.hh"

#include <memory>
#include <utility>
#include <vector>

#include "DesignStateMap.hh"
#include "grt/GlobalRouter.h"
#include "grt/Rudy.h"
#include "odb/geom.h"
#include "rsz/Resizer.hh"

namespace rsz {

RudyCongestionMapBuilder::RudyCongestionMapBuilder(Resizer* resizer)
    : DesignStateMapBuilder(resizer)
{
}

bool RudyCongestionMapBuilder::available() const
{
  return resizer_->block() != nullptr && resizer_->globalRouter() != nullptr;
}

std::unique_ptr<DesignStateMap> RudyCongestionMapBuilder::build(
    const odb::Rect& /* bounds */,
    int /* bins_x */,
    int /* bins_y */)
{
  grt::GlobalRouter* grouter = resizer_->globalRouter();
  if (grouter == nullptr) {
    return nullptr;
  }
  grt::Rudy* rudy = grouter->getRudy();
  if (rudy == nullptr) {
    return nullptr;
  }
  rudy->calculateRudy();

  const auto [num_x, num_y] = rudy->getGridSize();
  if (num_x == 0 || num_y == 0) {
    return nullptr;
  }

  // Build bin edges aligned to the RUDY tiles so each tile value lands in
  // exactly one bin.
  std::vector<int> x_edges;
  std::vector<int> y_edges;
  x_edges.reserve(num_x + 1);
  y_edges.reserve(num_y + 1);
  for (int x = 0; x < num_x; ++x) {
    x_edges.push_back(rudy->getTile(x, 0).getRect().xMin());
  }
  x_edges.push_back(rudy->getTile(num_x - 1, 0).getRect().xMax());
  for (int y = 0; y < num_y; ++y) {
    y_edges.push_back(rudy->getTile(0, y).getRect().yMin());
  }
  y_edges.push_back(rudy->getTile(0, num_y - 1).getRect().yMax());

  auto map = std::make_unique<DesignStateMap>(
      name(), unit(), std::move(x_edges), std::move(y_edges));

  for (int x = 0; x < num_x; ++x) {
    for (int y = 0; y < num_y; ++y) {
      const grt::Rudy::Tile& tile = rudy->getTile(x, y);
      map->addToMap(tile.getRect(), tile.getRudy());
    }
  }

  return map;
}

}  // namespace rsz
