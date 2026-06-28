// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>
#include <string>

#include "DesignStateMapBuilder.hh"
#include "odb/geom.h"

namespace rsz {

class Resizer;
class DesignStateMap;

// Routing congestion: per-GCell usage/capacity ratio from the routed design,
// taking the worse of the horizontal and vertical directions.  Read from the
// ODB GCell grid, which is populated by global routing, so this is only
// available post-GRT.  The map is aligned to the GCell grid (non-uniform bins).
// Reported as a percentage (>100% means over capacity).
class RoutingCongestionMapBuilder : public DesignStateMapBuilder
{
 public:
  explicit RoutingCongestionMapBuilder(Resizer* resizer);

  std::string name() const override { return "Routing Congestion"; }
  std::string unit() const override { return "%"; }
  bool available() const override;
  std::unique_ptr<DesignStateMap> build(const odb::Rect& bounds,
                                        int bins_x,
                                        int bins_y) override;
};

}  // namespace rsz
