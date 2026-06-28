// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>
#include <string>

#include "DesignStateMap.hh"
#include "odb/geom.h"

namespace rsz {

class Resizer;

// Interface for one design-state map provider.  A builder knows how to read a
// particular signal (placement density, congestion, power, ...) from the data
// producers and produce a populated DesignStateMap.  Adding a new map type is
// just adding a builder.
//
// build() runs single-threaded and must not mutate the database.  Builders with
// a native grid (e.g. RUDY tiles or the GCell grid) construct the map from
// explicit edges and may ignore the suggested bounds/bin counts; simple
// builders use makeUniformMap().
class DesignStateMapBuilder
{
 public:
  virtual ~DesignStateMapBuilder() = default;

  // Human-readable name and value unit (e.g. "Placement Density", "%").
  virtual std::string name() const = 0;
  virtual std::string unit() const = 0;

  // Whether the map can be built in the current design state.
  virtual bool available() const = 0;

  // Build and populate a map.  bounds/bins are the manager's defaults; native
  // grid builders may ignore them.  Returns nullptr if nothing could be built.
  virtual std::unique_ptr<DesignStateMap> build(const odb::Rect& bounds,
                                                int bins_x,
                                                int bins_y)
      = 0;

 protected:
  explicit DesignStateMapBuilder(Resizer* resizer) : resizer_(resizer) {}

  std::unique_ptr<DesignStateMap> makeUniformMap(const odb::Rect& bounds,
                                                 int bins_x,
                                                 int bins_y) const
  {
    return std::make_unique<DesignStateMap>(
        name(), unit(), bounds, bins_x, bins_y);
  }

  Resizer* resizer_;
};

}  // namespace rsz
