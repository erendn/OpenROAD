// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "SkewAnalysis.hh"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "SkewScheduler.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Network.hh"
#include "utl/Logger.h"

namespace rsz {

using sta::delayAsString;
using utl::RSZ;

SkewAnalysis::SkewAnalysis(Resizer* resizer) : resizer_(resizer)
{
}

void SkewAnalysis::dumpSchedule(const SkewScheduler& sched,
                                const std::vector<double>& skew,
                                const char* dump_file)
{
  FILE* f = std::fopen(dump_file, "w");
  if (f == nullptr) {
    logger_->warn(RSZ, 3228, "Could not open {} for skew dump.", dump_file);
    return;
  }
  std::fprintf(f, "# register_instance  skew_seconds (extra clock latency)\n");
  int nonzero = 0;
  for (int i = 1; i <= sched.numRegisters(); ++i) {
    if (std::abs(skew[i]) > 1e-13) {
      nonzero++;
      std::fprintf(f,
                   "%s %.6e\n",
                   network_->pathName(sched.nodeInstance(i)).c_str(),
                   skew[i]);
    }
  }
  std::fclose(f);
  logger_->info(
      RSZ, 3229, "Wrote {} non-zero skew targets to {}.", nonzero, dump_file);
}

void SkewAnalysis::estimateUsefulSkew(
    const std::vector<float>& budget_fractions,
    const char* dump_file,
    const bool verbose)
{
  logger_ = resizer_->logger_;
  dbStaState::init(resizer_->sta_);

  SkewScheduler sched(resizer_);
  sched.build();

  if (sched.empty()) {
    logger_->warn(RSZ,
                  3220,
                  "Useful-skew analysis: no register-to-register setup arcs "
                  "found; nothing to analyze.");
    return;
  }

  constexpr int digits = 4;
  const double analyzed_wns = sched.analyzedWns();
  const double period = sched.period();

  logger_->info(RSZ,
                3221,
                "Useful-skew headroom analysis (read-only; netlist and clock "
                "are not modified).");
  logger_->info(RSZ,
                3222,
                "  Registers: {}  setup arcs: {}  hold arcs: {}  unconstrained "
                "endpoints skipped: {}",
                sched.numRegisters(),
                sched.numSetupArcs(),
                sched.numHoldArcs(),
                sched.numSkippedEndpoints());
  if (sched.numHoldViolations() > 0) {
    logger_->info(RSZ,
                  3223,
                  "  {} endpoints already violate hold; schedule is guarded to "
                  "not worsen them.",
                  sched.numHoldViolations());
  }
  logger_->info(
      RSZ,
      3224,
      "  Design WNS: {}   analyzed-arc WNS: {}",
      delayAsString(static_cast<sta::Delay>(sched.designWns()), digits, sta_),
      delayAsString(static_cast<sta::Delay>(analyzed_wns), digits, sta_));

  // Sweep the requested budgets (two-sided).  Track the unbounded result for
  // stats and the largest finite budget for the optional dump.
  bool have_unbounded = false;
  double unbounded_ach = analyzed_wns;
  double dump_budget = -1.0;
  double dump_ach = analyzed_wns;
  bool have_dump_budget = false;

  for (const float f : budget_fractions) {
    bool capped = false;
    double budget;
    std::string label;
    if (f < 0.0f) {
      budget = -1.0;
      label = "unbounded skew";
    } else {
      if (period <= 0.0) {
        continue;  // fractional budget needs a clock period
      }
      budget = static_cast<double>(f) * period;
      label = fmt::format(
          "budget {:.1f}% (={})",
          f * 100.0,
          delayAsString(static_cast<sta::Delay>(budget), digits, sta_));
    }

    const double ach
        = sched.maxAchievableWns(budget, /*one_sided=*/false, capped);
    const double gain = ach - analyzed_wns;
    logger_->info(RSZ,
                  3225,
                  "  achievable WNS @ {}: {} (gain {}){}",
                  label,
                  delayAsString(static_cast<sta::Delay>(ach), digits, sta_),
                  delayAsString(static_cast<sta::Delay>(gain), digits, sta_),
                  capped ? "  [capped at search bound]" : "");

    if (f < 0.0f) {
      have_unbounded = true;
      unbounded_ach = ach;
    } else if (budget > dump_budget) {
      dump_budget = budget;
      dump_ach = ach;
      have_dump_budget = true;
    }
  }

  // Stats from the unbounded solution (most skew used).
  if (have_unbounded) {
    std::vector<double> x;
    if (sched.solveVector(unbounded_ach, -1.0, /*one_sided=*/false, x)) {
      int nonzero = 0;
      double max_abs = 0.0;
      for (int i = 1; i <= sched.numRegisters(); ++i) {
        if (std::abs(x[i]) > 1e-13) {
          nonzero++;
        }
        max_abs = std::max(max_abs, std::abs(x[i]));
      }
      logger_->info(
          RSZ,
          3226,
          "  At unbounded skew: {} of {} registers need non-zero "
          "skew, max |skew| = {}.",
          nonzero,
          sched.numRegisters(),
          delayAsString(static_cast<sta::Delay>(max_abs), digits, sta_));
    }
  }

  if (verbose) {
    logger_->info(RSZ,
                  3227,
                  "  Note: per-endpoint worst-path arcs give an optimistic "
                  "bound; CRPR, generated/gated clocks and latches are "
                  "first-order ignored.");
  }

  // Optional schedule dump at the largest finite budget (else unbounded).
  if (dump_file != nullptr && dump_file[0] != '\0') {
    std::vector<double> x;
    if (have_dump_budget
        && sched.solveVector(dump_ach, dump_budget, /*one_sided=*/false, x)) {
      dumpSchedule(sched, x, dump_file);
    } else if (have_unbounded
               && sched.solveVector(
                   unbounded_ach, -1.0, /*one_sided=*/false, x)) {
      dumpSchedule(sched, x, dump_file);
    }
  }
}

}  // namespace rsz
