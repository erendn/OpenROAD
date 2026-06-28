// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <string>
#include <vector>

#include "odb/geom.h"

namespace rsz {

// A 2D scalar field over a rectangular region of the block, binned into a grid.
// Builders (see DesignStateMapBuilder) populate the bins; rsz queries them by
// point or region.  This is intentionally independent of the GUI heatmap layer
// (gui::HeatMapDataSource): it reuses the same data producers but is a
// lightweight, query-oriented structure rather than a rendering one.
//
// Execution model: timing repair runs serially, so reads and updates happen on
// a single thread and may interleave.  Query methods are marked const as
// hygiene (so an experimental multi-threaded consumer could freeze-and-share a
// map later), but serial correctness does not depend on freezing or locking.
//
// The grid is stored as explicit bin edges so a future provider (e.g. routing
// congestion) can align bins to the GCell grid without an API change.  Phase 1
// builds a uniform grid.
class DesignStateMap
{
 public:
  struct Stats
  {
    double min = 0.0;
    double max = 0.0;
    double mean = 0.0;
    int nonzero_bins = 0;
    int total_bins = 0;
  };

  // Builds a uniform grid of bins_x by bins_y bins over bounds.
  DesignStateMap(std::string name,
                 std::string unit,
                 const odb::Rect& bounds,
                 int bins_x,
                 int bins_y);

  // Builds a grid from explicit bin edges (DBU).  Lets a builder align bins to
  // a native grid (e.g. RUDY tiles or the GCell grid) instead of a uniform one.
  // Each edge vector must be sorted ascending and have at least two entries.
  DesignStateMap(std::string name,
                 std::string unit,
                 std::vector<int> x_edges,
                 std::vector<int> y_edges);

  // --- population (called by builders) ---
  // Distributes value over the bins overlapping region, weighted by the
  // fraction of each bin the region covers:
  //   bin += value * area(region ∩ bin) / area(bin)
  // When region is aligned to the bin edges (RUDY/GCell case) the whole value
  // lands in the matching bin.
  void addToMap(const odb::Rect& region, double value);
  // Adds value to the single bin containing p (for count- or per-instance
  // quantities such as pin counts or power).  No-op if p is outside.
  void addPoint(const odb::Point& p, double value);
  // (Re)computes the cached maximum used for normalization.  Call after
  // population (or after a batch of incremental updates).
  void finalize();

  // --- queries (const) ---
  bool valid() const { return !bins_.empty(); }
  // Value of the bin containing p (0 if p is outside the region).
  double valueAt(const odb::Point& p) const;
  // Area-weighted mean of the bins overlapping r.
  double regionAverage(const odb::Rect& r) const;
  // Maximum bin value over the bins overlapping r.
  double regionMax(const odb::Rect& r) const;
  // valueAt(p) scaled to [0, 1] by the map maximum (0 if the map is empty).
  double normalizedAt(const odb::Point& p) const;
  Stats stats() const;

  // --- metadata ---
  const std::string& name() const { return name_; }
  const std::string& unit() const { return unit_; }
  const odb::Rect& bounds() const { return bounds_; }
  int binsX() const { return bins_x_; }
  int binsY() const { return bins_y_; }
  double maxValue() const { return max_value_; }
  odb::Point binCenter(int ix, int iy) const;
  double binValue(int ix, int iy) const;

 private:
  // Returns the bin column/row containing the coordinate, or -1 if outside.
  int binIndexX(int x) const;
  int binIndexY(int y) const;
  int idx(int ix, int iy) const { return iy * bins_x_ + ix; }

  std::string name_;
  std::string unit_;
  odb::Rect bounds_;
  int bins_x_ = 0;
  int bins_y_ = 0;
  std::vector<int> x_edges_;  // size bins_x_ + 1, DBU
  std::vector<int> y_edges_;  // size bins_y_ + 1, DBU
  std::vector<double> bins_;  // row-major, size bins_x_ * bins_y_
  double max_value_ = 0.0;
};

}  // namespace rsz
