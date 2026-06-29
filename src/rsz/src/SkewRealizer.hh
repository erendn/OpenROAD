// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <map>

#include "db_sta/dbSta.hh"
#include "sta/MinMax.hh"
#include "sta/NetworkClass.hh"

namespace sta {
class LibertyCell;
}

namespace utl {
class Logger;
}

namespace est {
class EstimateParasitics;
}

namespace rsz {

class Resizer;

// SkewRealizer: post-CTS clock-skew optimization (useful skew).
//
// Computes a delay-only (one-sided) skew schedule with SkewScheduler, then
// physically realizes it by inserting clock delay buffers on selected register
// clock pins.  Each candidate register is journaled and kept only if it
// improves global setup WNS (or TNS, WNS-neutral) without worsening hold.
// Iterates a few passes so secondary critical paths (TNS) are picked up after
// the worst path is addressed.  Modifies the clock network (run after CTS).
class SkewRealizer : public sta::dbStaState
{
 public:
  explicit SkewRealizer(Resizer* resizer);

  void optimizeClockSkew(float budget_fraction,
                         const char* buffer_cell_name,
                         int max_buffers_per_reg,
                         int max_buffer_count,
                         int max_passes,
                         bool verbose);

 private:
  void init();
  sta::LibertyCell* pickDelayCell(const char* name);
  void buildClkPinMap(std::map<const sta::Instance*, const sta::Pin*>& clk_pin);
  // Insert up to n clock buffers in series before clk_pin; returns the number
  // actually inserted.
  int insertClkDelay(const sta::Pin* clk_pin, sta::LibertyCell* cell, int n);

  utl::Logger* logger_ = nullptr;
  sta::dbNetwork* db_network_ = nullptr;
  Resizer* resizer_;
  est::EstimateParasitics* estimate_parasitics_ = nullptr;

  const sta::MinMax* max_ = sta::MinMax::max();
  const sta::MinMax* min_ = sta::MinMax::min();
};

}  // namespace rsz
