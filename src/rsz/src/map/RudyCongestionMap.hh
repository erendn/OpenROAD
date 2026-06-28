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

// Estimated routing congestion (RUDY): per-tile routing demand estimated from
// net bounding boxes via grt::Rudy.  Computed from placement, so it is
// available both pre- and post-GRT (it does not need actual routes, and
// grt::Rudy bootstraps the routing grid it needs).  The raw RUDY value is a
// dimensionless routing-demand density (higher is more congested), so the map
// is unitless.
class RudyCongestionMapBuilder : public DesignStateMapBuilder
{
 public:
  explicit RudyCongestionMapBuilder(Resizer* resizer);

  std::string name() const override { return "Estimated Congestion (RUDY)"; }
  std::string unit() const override { return ""; }
  bool available() const override;
  std::unique_ptr<DesignStateMap> build(const odb::Rect& bounds,
                                        int bins_x,
                                        int bins_y) override;
};

}  // namespace rsz
