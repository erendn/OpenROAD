// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <map>
#include <memory>
#include <string>

#include "DesignStateMap.hh"

namespace rsz {

class Resizer;
class DesignStateMapBuilder;

enum class DesignStateMapType
{
  kPlacementDensity,
  kPinDensity,
  kEstimatedCongestion,
  kRoutingCongestion,
  kPowerDensity,
};

// Owns and caches the design-state maps for an rsz session.  Maps are built on
// demand from their builders and cached; callers invalidate them explicitly
// (Phase 1) or, once a repair consumer exists, at pass boundaries / on commit.
//
// Single-threaded: build, update, and query all happen on the main thread.
class DesignStateMaps
{
 public:
  explicit DesignStateMaps(Resizer* resizer);
  ~DesignStateMaps();

  // Returns the cached map, building it if needed.  Returns nullptr if the
  // builder is unavailable in the current design state.
  const DesignStateMap* get(DesignStateMapType type);

  void invalidate(DesignStateMapType type);
  void invalidateAll();

  // Logs grid metadata, statistics and hotspot bins for the map (no files).
  // An ASCII rendering is emitted under the "design_state_map" debug group.
  void report(DesignStateMapType type);

  void setDefaultBinCount(int bins_x, int bins_y);

  // Maps a command type string (e.g. "placement_density") to the enum.
  static bool parseType(const std::string& name, DesignStateMapType& type);
  static std::string typeName(DesignStateMapType type);

 private:
  std::unique_ptr<DesignStateMapBuilder> makeBuilder(DesignStateMapType type);

  Resizer* resizer_;
  int default_bins_x_ = 64;
  int default_bins_y_ = 64;
  std::map<DesignStateMapType, std::unique_ptr<DesignStateMap>> cache_;
};

}  // namespace rsz
