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

// Power density: total instance power (internal + switching + leakage) summed
// per bin, in microwatts.  Uses STA power analysis on the current scene, so a
// scene must exist; meaningful switching power additionally needs propagated
// activities (clocks / a power activity model).  Valid both pre- and post-GRT.
class PowerDensityMapBuilder : public DesignStateMapBuilder
{
 public:
  explicit PowerDensityMapBuilder(Resizer* resizer);

  std::string name() const override { return "Power Density"; }
  std::string unit() const override { return "uW"; }
  bool available() const override;
  std::unique_ptr<DesignStateMap> build(const odb::Rect& bounds,
                                        int bins_x,
                                        int bins_y) override;
};

}  // namespace rsz
