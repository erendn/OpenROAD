// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "DesignStateMaps.hh"

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "DesignStateMap.hh"
#include "DesignStateMapBuilder.hh"
#include "PinDensityMap.hh"
#include "PlacementDensityMap.hh"
#include "PowerDensityMap.hh"
#include "RoutingCongestionMap.hh"
#include "RudyCongestionMap.hh"
#include "odb/db.h"
#include "odb/geom.h"
#include "rsz/Resizer.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

DesignStateMaps::DesignStateMaps(Resizer* resizer) : resizer_(resizer)
{
}

DesignStateMaps::~DesignStateMaps() = default;

void DesignStateMaps::setDefaultBinCount(int bins_x, int bins_y)
{
  if (bins_x > 0) {
    default_bins_x_ = bins_x;
  }
  if (bins_y > 0) {
    default_bins_y_ = bins_y;
  }
}

void DesignStateMaps::invalidate(DesignStateMapType type)
{
  cache_.erase(type);
}

void DesignStateMaps::invalidateAll()
{
  cache_.clear();
}

std::unique_ptr<DesignStateMapBuilder> DesignStateMaps::makeBuilder(
    DesignStateMapType type)
{
  switch (type) {
    case DesignStateMapType::kPlacementDensity:
      return std::make_unique<PlacementDensityMapBuilder>(resizer_);
    case DesignStateMapType::kPinDensity:
      return std::make_unique<PinDensityMapBuilder>(resizer_);
    case DesignStateMapType::kEstimatedCongestion:
      return std::make_unique<RudyCongestionMapBuilder>(resizer_);
    case DesignStateMapType::kRoutingCongestion:
      return std::make_unique<RoutingCongestionMapBuilder>(resizer_);
    case DesignStateMapType::kPowerDensity:
      return std::make_unique<PowerDensityMapBuilder>(resizer_);
  }
  return nullptr;
}

const DesignStateMap* DesignStateMaps::get(DesignStateMapType type)
{
  auto it = cache_.find(type);
  if (it != cache_.end()) {
    return it->second.get();
  }

  std::unique_ptr<DesignStateMapBuilder> builder = makeBuilder(type);
  if (builder == nullptr || !builder->available()) {
    return nullptr;
  }

  odb::dbBlock* block = resizer_->block();
  if (block == nullptr) {
    return nullptr;
  }
  // Prefer the core area; fall back to the die area when no core is defined.
  odb::Rect bounds = block->getCoreArea();
  if (bounds.dx() <= 0 || bounds.dy() <= 0) {
    bounds = block->getDieArea();
  }
  if (bounds.dx() <= 0 || bounds.dy() <= 0) {
    return nullptr;
  }

  std::unique_ptr<DesignStateMap> map
      = builder->build(bounds, default_bins_x_, default_bins_y_);
  if (map == nullptr) {
    return nullptr;
  }
  map->finalize();

  auto [inserted, ok] = cache_.emplace(type, std::move(map));
  return inserted->second.get();
}

void DesignStateMaps::report(DesignStateMapType type)
{
  utl::Logger* logger = resizer_->logger();
  const DesignStateMap* map = get(type);
  if (map == nullptr) {
    logger->report(
        "Design state map '{}' is not available in the current "
        "design state.",
        typeName(type));
    return;
  }

  odb::dbBlock* block = resizer_->block();
  const double dbu = block->getDbUnitsPerMicron();
  const odb::Rect& b = map->bounds();
  const DesignStateMap::Stats s = map->stats();
  // Suffix appended after a value, e.g. " %"; empty for a unitless map.
  const std::string unit_suffix
      = map->unit().empty() ? "" : (" " + map->unit());

  if (map->unit().empty()) {
    logger->report("Design State Map: {}", map->name());
  } else {
    logger->report("Design State Map: {} [{}]", map->name(), map->unit());
  }
  logger->report("  Region: ({:.2f}, {:.2f}) - ({:.2f}, {:.2f}) um",
                 b.xMin() / dbu,
                 b.yMin() / dbu,
                 b.xMax() / dbu,
                 b.yMax() / dbu);
  logger->report("  Grid: {} x {} bins ({:.2f} x {:.2f} um per bin)",
                 map->binsX(),
                 map->binsY(),
                 (b.dx() / static_cast<double>(map->binsX())) / dbu,
                 (b.dy() / static_cast<double>(map->binsY())) / dbu);
  logger->report("  Values: min {:.2f}  mean {:.2f}  max {:.2f}{}",
                 s.min,
                 s.mean,
                 s.max,
                 unit_suffix);
  logger->report("  Occupied bins: {} / {}", s.nonzero_bins, s.total_bins);

  // Hotspot bins, sorted by value (descending), then by location for a
  // deterministic order.
  std::vector<std::tuple<double, int, int>> hotspots;
  for (int iy = 0; iy < map->binsY(); ++iy) {
    for (int ix = 0; ix < map->binsX(); ++ix) {
      const double v = map->binValue(ix, iy);
      if (v > 0.0) {
        hotspots.emplace_back(v, ix, iy);
      }
    }
  }
  std::ranges::sort(hotspots, [](const auto& a, const auto& b) {
    if (std::get<0>(a) != std::get<0>(b)) {
      return std::get<0>(a) > std::get<0>(b);
    }
    if (std::get<1>(a) != std::get<1>(b)) {
      return std::get<1>(a) < std::get<1>(b);
    }
    return std::get<2>(a) < std::get<2>(b);
  });

  const int top_n = std::min<int>(10, static_cast<int>(hotspots.size()));
  logger->report("  Hotspots (top {}):", top_n);
  for (int i = 0; i < top_n; ++i) {
    const auto& [value, ix, iy] = hotspots[i];
    const odb::Point center = map->binCenter(ix, iy);
    logger->report("    {}: {:.2f}{} at ({:.2f}, {:.2f}) um",
                   i + 1,
                   value,
                   unit_suffix,
                   center.x() / dbu,
                   center.y() / dbu);
  }

  // ASCII rendering (top row first) under the debug group.
  if (logger->debugCheck(RSZ, "design_state_map", 1)) {
    const char* ramp = " .:-=+*#%@";
    const double max_value = map->maxValue();
    for (int iy = map->binsY() - 1; iy >= 0; --iy) {
      std::string row;
      for (int ix = 0; ix < map->binsX(); ++ix) {
        const double n
            = max_value > 0.0 ? map->binValue(ix, iy) / max_value : 0.0;
        const int level = std::clamp(static_cast<int>(n * 9.0), 0, 9);
        row.push_back(ramp[level]);
      }
      debugPrint(logger, RSZ, "design_state_map", 1, "{}", row);
    }
  }
}

bool DesignStateMaps::parseType(const std::string& name,
                                DesignStateMapType& type)
{
  if (name == "placement_density") {
    type = DesignStateMapType::kPlacementDensity;
    return true;
  }
  if (name == "pin_density") {
    type = DesignStateMapType::kPinDensity;
    return true;
  }
  if (name == "estimated_congestion") {
    type = DesignStateMapType::kEstimatedCongestion;
    return true;
  }
  if (name == "routing_congestion") {
    type = DesignStateMapType::kRoutingCongestion;
    return true;
  }
  if (name == "power_density") {
    type = DesignStateMapType::kPowerDensity;
    return true;
  }
  return false;
}

std::string DesignStateMaps::typeName(DesignStateMapType type)
{
  switch (type) {
    case DesignStateMapType::kPlacementDensity:
      return "placement_density";
    case DesignStateMapType::kPinDensity:
      return "pin_density";
    case DesignStateMapType::kEstimatedCongestion:
      return "estimated_congestion";
    case DesignStateMapType::kRoutingCongestion:
      return "routing_congestion";
    case DesignStateMapType::kPowerDensity:
      return "power_density";
  }
  return "unknown";
}

}  // namespace rsz
