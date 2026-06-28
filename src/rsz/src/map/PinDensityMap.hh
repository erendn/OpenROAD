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

// Pin density: count of signal pins (instance terminals) per bin.  Pure ODB
// geometry, valid both pre- and post-GRT.  A proxy for pin-access congestion:
// regions with many pins are harder to route and poor places to add cells.
class PinDensityMapBuilder : public DesignStateMapBuilder
{
 public:
  explicit PinDensityMapBuilder(Resizer* resizer);

  std::string name() const override { return "Pin Density"; }
  std::string unit() const override { return "pins"; }
  bool available() const override;
  std::unique_ptr<DesignStateMap> build(const odb::Rect& bounds,
                                        int bins_x,
                                        int bins_y) override;
};

}  // namespace rsz
