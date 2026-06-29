// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <vector>

#include "db_sta/dbSta.hh"

namespace utl {
class Logger;
}

namespace rsz {

class Resizer;
class SkewScheduler;

// SkewAnalysis: read-only "useful-skew headroom" probe.  Thin reporting layer
// over SkewScheduler using a two-sided (+/-B) skew budget; it estimates how
// much setup WNS clock skew scheduling could recover WITHOUT changing the
// netlist, clock tree, or SDC.  See SkewScheduler for the constraint model and
// caveats.
class SkewAnalysis : public sta::dbStaState
{
 public:
  explicit SkewAnalysis(Resizer* resizer);

  // budget_fractions: skew budgets as a fraction of the (min) clock period.
  // A negative entry means "unbounded".  dump_file may be null/empty.
  void estimateUsefulSkew(const std::vector<float>& budget_fractions,
                          const char* dump_file,
                          bool verbose);

 private:
  // Write the per-register skew solution to dump_file (instance  skew_seconds).
  void dumpSchedule(const SkewScheduler& sched,
                    const std::vector<double>& skew,
                    const char* dump_file);

  utl::Logger* logger_ = nullptr;
  Resizer* resizer_;
};

}  // namespace rsz
