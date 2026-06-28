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

// Placement density: per-bin percentage of area occupied by placed instances.
// Pure ODB geometry, valid both pre- and post-GRT.  Mirrors the computation in
// gui/src/heatMapPlacementDensity.cpp so values are comparable to the GUI map,
// but without any dependency on the GUI.
class PlacementDensityMapBuilder : public DesignStateMapBuilder
{
 public:
  explicit PlacementDensityMapBuilder(Resizer* resizer);

  std::string name() const override { return "Placement Density"; }
  std::string unit() const override { return "%"; }
  bool available() const override;
  std::unique_ptr<DesignStateMap> build(const odb::Rect& bounds,
                                        int bins_x,
                                        int bins_y) override;

 private:
  // Optimization cares about logic occupancy, so fillers/taps are excluded by
  // default.
  bool include_filler_ = false;
  bool include_taps_ = false;
};

}  // namespace rsz
