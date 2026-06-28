// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "DesignStateMap.hh"

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "odb/geom.h"

namespace rsz {

DesignStateMap::DesignStateMap(std::string name,
                               std::string unit,
                               const odb::Rect& bounds,
                               int bins_x,
                               int bins_y)
    : name_(std::move(name)),
      unit_(std::move(unit)),
      bounds_(bounds),
      bins_x_(std::max(1, bins_x)),
      bins_y_(std::max(1, bins_y))
{
  // Uniform edges spanning the bounds, with the last edge pinned to the max so
  // rounding never leaves a gap at the right/top border.
  const int64_t width = bounds_.dx();
  const int64_t height = bounds_.dy();
  x_edges_.resize(bins_x_ + 1);
  for (int i = 0; i <= bins_x_; ++i) {
    x_edges_[i] = bounds_.xMin() + static_cast<int>(width * i / bins_x_);
  }
  x_edges_[bins_x_] = bounds_.xMax();
  y_edges_.resize(bins_y_ + 1);
  for (int i = 0; i <= bins_y_; ++i) {
    y_edges_[i] = bounds_.yMin() + static_cast<int>(height * i / bins_y_);
  }
  y_edges_[bins_y_] = bounds_.yMax();

  bins_.assign(static_cast<size_t>(bins_x_) * bins_y_, 0.0);
}

DesignStateMap::DesignStateMap(std::string name,
                               std::string unit,
                               std::vector<int> x_edges,
                               std::vector<int> y_edges)
    : name_(std::move(name)),
      unit_(std::move(unit)),
      bounds_(x_edges.front(), y_edges.front(), x_edges.back(), y_edges.back()),
      bins_x_(static_cast<int>(x_edges.size()) - 1),
      bins_y_(static_cast<int>(y_edges.size()) - 1),
      x_edges_(std::move(x_edges)),
      y_edges_(std::move(y_edges))
{
  bins_.assign(static_cast<size_t>(bins_x_) * bins_y_, 0.0);
}

int DesignStateMap::binIndexX(int x) const
{
  if (x < x_edges_.front() || x > x_edges_.back()) {
    return -1;
  }
  const auto it = std::upper_bound(x_edges_.begin(), x_edges_.end(), x);
  int i = static_cast<int>(it - x_edges_.begin()) - 1;
  return std::clamp(i, 0, bins_x_ - 1);
}

int DesignStateMap::binIndexY(int y) const
{
  if (y < y_edges_.front() || y > y_edges_.back()) {
    return -1;
  }
  const auto it = std::upper_bound(y_edges_.begin(), y_edges_.end(), y);
  int i = static_cast<int>(it - y_edges_.begin()) - 1;
  return std::clamp(i, 0, bins_y_ - 1);
}

void DesignStateMap::addToMap(const odb::Rect& region, double value)
{
  // Clip the region to the map bounds.
  const int rlx = std::max(region.xMin(), bounds_.xMin());
  const int rly = std::max(region.yMin(), bounds_.yMin());
  const int rux = std::min(region.xMax(), bounds_.xMax());
  const int ruy = std::min(region.yMax(), bounds_.yMax());
  if (rlx >= rux || rly >= ruy) {
    return;
  }

  const int ix0 = binIndexX(rlx);
  const int ix1 = binIndexX(rux - 1);
  const int iy0 = binIndexY(rly);
  const int iy1 = binIndexY(ruy - 1);
  if (ix0 < 0 || iy0 < 0) {
    return;
  }

  for (int iy = iy0; iy <= iy1; ++iy) {
    const int by0 = y_edges_[iy];
    const int by1 = y_edges_[iy + 1];
    const int oy0 = std::max(rly, by0);
    const int oy1 = std::min(ruy, by1);
    if (oy0 >= oy1) {
      continue;
    }
    for (int ix = ix0; ix <= ix1; ++ix) {
      const int bx0 = x_edges_[ix];
      const int bx1 = x_edges_[ix + 1];
      const int ox0 = std::max(rlx, bx0);
      const int ox1 = std::min(rux, bx1);
      if (ox0 >= ox1) {
        continue;
      }
      const double inter
          = static_cast<double>(ox1 - ox0) * static_cast<double>(oy1 - oy0);
      const double bin_area
          = static_cast<double>(bx1 - bx0) * static_cast<double>(by1 - by0);
      if (bin_area > 0.0) {
        bins_[idx(ix, iy)] += value * inter / bin_area;
      }
    }
  }
}

void DesignStateMap::addPoint(const odb::Point& p, double value)
{
  const int ix = binIndexX(p.x());
  const int iy = binIndexY(p.y());
  if (ix < 0 || iy < 0) {
    return;
  }
  bins_[idx(ix, iy)] += value;
}

void DesignStateMap::finalize()
{
  max_value_ = 0.0;
  for (const double v : bins_) {
    max_value_ = std::max(max_value_, v);
  }
}

double DesignStateMap::valueAt(const odb::Point& p) const
{
  const int ix = binIndexX(p.x());
  const int iy = binIndexY(p.y());
  if (ix < 0 || iy < 0) {
    return 0.0;
  }
  return bins_[idx(ix, iy)];
}

double DesignStateMap::regionAverage(const odb::Rect& r) const
{
  const int rlx = std::max(r.xMin(), bounds_.xMin());
  const int rly = std::max(r.yMin(), bounds_.yMin());
  const int rux = std::min(r.xMax(), bounds_.xMax());
  const int ruy = std::min(r.yMax(), bounds_.yMax());
  if (rlx >= rux || rly >= ruy) {
    return 0.0;
  }

  const int ix0 = binIndexX(rlx);
  const int ix1 = binIndexX(rux - 1);
  const int iy0 = binIndexY(rly);
  const int iy1 = binIndexY(ruy - 1);
  if (ix0 < 0 || iy0 < 0) {
    return 0.0;
  }

  double weighted_sum = 0.0;
  double area_sum = 0.0;
  for (int iy = iy0; iy <= iy1; ++iy) {
    const int oy0 = std::max(rly, y_edges_[iy]);
    const int oy1 = std::min(ruy, y_edges_[iy + 1]);
    if (oy0 >= oy1) {
      continue;
    }
    for (int ix = ix0; ix <= ix1; ++ix) {
      const int ox0 = std::max(rlx, x_edges_[ix]);
      const int ox1 = std::min(rux, x_edges_[ix + 1]);
      if (ox0 >= ox1) {
        continue;
      }
      const double inter
          = static_cast<double>(ox1 - ox0) * static_cast<double>(oy1 - oy0);
      weighted_sum += bins_[idx(ix, iy)] * inter;
      area_sum += inter;
    }
  }
  return area_sum > 0.0 ? weighted_sum / area_sum : 0.0;
}

double DesignStateMap::regionMax(const odb::Rect& r) const
{
  const int rlx = std::max(r.xMin(), bounds_.xMin());
  const int rly = std::max(r.yMin(), bounds_.yMin());
  const int rux = std::min(r.xMax(), bounds_.xMax());
  const int ruy = std::min(r.yMax(), bounds_.yMax());
  if (rlx >= rux || rly >= ruy) {
    return 0.0;
  }

  const int ix0 = binIndexX(rlx);
  const int ix1 = binIndexX(rux - 1);
  const int iy0 = binIndexY(rly);
  const int iy1 = binIndexY(ruy - 1);
  if (ix0 < 0 || iy0 < 0) {
    return 0.0;
  }

  double result = 0.0;
  for (int iy = iy0; iy <= iy1; ++iy) {
    for (int ix = ix0; ix <= ix1; ++ix) {
      result = std::max(result, bins_[idx(ix, iy)]);
    }
  }
  return result;
}

double DesignStateMap::normalizedAt(const odb::Point& p) const
{
  if (max_value_ <= 0.0) {
    return 0.0;
  }
  return valueAt(p) / max_value_;
}

DesignStateMap::Stats DesignStateMap::stats() const
{
  Stats s;
  s.total_bins = static_cast<int>(bins_.size());
  if (bins_.empty()) {
    return s;
  }
  s.min = bins_.front();
  double sum = 0.0;
  for (const double v : bins_) {
    s.min = std::min(s.min, v);
    s.max = std::max(s.max, v);
    sum += v;
    if (v > 0.0) {
      ++s.nonzero_bins;
    }
  }
  s.mean = sum / s.total_bins;
  return s;
}

odb::Point DesignStateMap::binCenter(int ix, int iy) const
{
  const int cx = (x_edges_[ix] + x_edges_[ix + 1]) / 2;
  const int cy = (y_edges_[iy] + y_edges_[iy + 1]) / 2;
  return {cx, cy};
}

double DesignStateMap::binValue(int ix, int iy) const
{
  return bins_[idx(ix, iy)];
}

}  // namespace rsz
