// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "SkewRealizer.hh"

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "SkewScheduler.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "odb/db.h"
#include "rsz/Resizer.hh"
#include "sta/Clock.hh"
#include "sta/Delay.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/Scene.hh"
#include "sta/Sdc.hh"
#include "sta/SdcClass.hh"
#include "sta/Transition.hh"
#include "utl/Logger.h"

namespace rsz {

using sta::delayAsString;
using utl::RSZ;

SkewRealizer::SkewRealizer(Resizer* resizer) : resizer_(resizer)
{
}

void SkewRealizer::init()
{
  logger_ = resizer_->logger_;
  dbStaState::init(resizer_->sta_);
  db_network_ = resizer_->db_network_;
  estimate_parasitics_ = resizer_->estimate_parasitics_;
  sta_->updateTiming(false);
}

sta::LibertyCell* SkewRealizer::pickDelayCell(const char* name)
{
  if (name != nullptr && name[0] != '\0') {
    sta::LibertyCell* cell = network_->findLibertyCell(name);
    if (cell == nullptr) {
      logger_->warn(RSZ, 3236, "Clock buffer cell {} not found.", name);
    }
    return cell;
  }
  if (resizer_->clk_buffers_.empty()) {
    std::vector<std::string> names;
    resizer_->inferClockBufferList(nullptr, names);  // populates clk_buffers_
  }
  // Smallest-area clock buffer gives the finest delay granularity.
  sta::LibertyCell* best = nullptr;
  for (sta::LibertyCell* cell : resizer_->clk_buffers_) {
    if (best == nullptr || cell->area() < best->area()) {
      best = cell;
    }
  }
  return best;
}

void SkewRealizer::buildClkPinMap(
    std::map<const sta::Instance*, const sta::Pin*>& clk_pin)
{
  sta::ClockSet clks;
  for (sta::Clock* clk : sta_->cmdSdc()->clocks()) {
    clks.insert(clk);
  }
  if (clks.empty()) {
    return;
  }
  sta::PinSet pins = sta_->findRegisterClkPins(&clks,
                                               sta::RiseFallBoth::riseFall(),
                                               /*registers=*/true,
                                               /*latches=*/false,
                                               sta_->cmdMode());
  for (const sta::Pin* pin : pins) {
    const sta::Instance* inst = network_->instance(pin);
    if (inst != nullptr) {
      clk_pin[inst] = pin;  // one clock pin per register
    }
  }
}

int SkewRealizer::insertClkDelay(const sta::Pin* clk_pin,
                                 sta::LibertyCell* cell,
                                 const int n)
{
  const odb::Point loc = db_network_->location(clk_pin);
  int inserted = 0;
  for (int i = 0; i < n; ++i) {
    // Each call splices one more buffer between the current driver and the
    // clock pin, extending the delay chain toward the register.
    sta::Instance* buf
        = resizer_->insertBufferBeforeLoad(const_cast<sta::Pin*>(clk_pin),
                                           cell,
                                           &loc,
                                           "clkskew_buf",
                                           "clkskew_net");
    if (buf == nullptr) {
      break;
    }
    ++inserted;
  }
  return inserted;
}

void SkewRealizer::optimizeClockSkew(const float budget_fraction,
                                     const char* buffer_cell_name,
                                     const int max_buffers_per_reg,
                                     const int max_buffer_count,
                                     const int max_passes,
                                     const bool verbose)
{
  init();
  constexpr int digits = 4;
  constexpr double eps = 1e-13;

  sta::LibertyCell* cell = pickDelayCell(buffer_cell_name);
  if (cell == nullptr) {
    logger_->warn(
        RSZ, 3230, "Useful skew: no clock buffer available; nothing to do.");
    return;
  }

  // Per-buffer clock delay (chain load ~= one buffer input).
  const sta::Scene* scene = sta_->cmdScene();
  sta::LibertyPort *in_port, *out_port;
  cell->bufferPorts(in_port, out_port);
  const float chain_load = resizer_->portCapacitance(in_port, scene);
  const double per_buffer_delay
      = resizer_->bufferDelay(cell, chain_load, scene, max_);
  if (per_buffer_delay <= 0.0) {
    logger_->warn(
        RSZ, 3231, "Useful skew: could not estimate clock buffer delay.");
    return;
  }

  std::map<const sta::Instance*, const sta::Pin*> inst_clk_pin;
  buildClkPinMap(inst_clk_pin);
  if (inst_clk_pin.empty()) {
    logger_->warn(RSZ, 3232, "Useful skew: no register clock pins found.");
    return;
  }

  const sta::Slack wns0 = sta_->worstSlack(max_);
  const sta::Slack tns0 = sta_->totalNegativeSlack(max_);

  logger_->info(
      RSZ,
      3233,
      "Useful-skew optimization (post-CTS): clock buffer {}, "
      "~{} per buffer, budget {:.1f}%.",
      cell->name(),
      delayAsString(static_cast<sta::Delay>(per_buffer_delay), digits, sta_),
      budget_fraction * 100.0);

  int total_buffers = 0;
  int total_regs_skewed = 0;

  for (int pass = 0; pass < max_passes; ++pass) {
    SkewScheduler sched(resizer_);
    sched.build();
    if (sched.empty()) {
      break;
    }
    const double period = sched.period();
    const double budget = (budget_fraction > 0.0f && period > 0.0)
                              ? static_cast<double>(budget_fraction) * period
                              : -1.0;
    bool capped = false;
    const double delta_star
        = sched.maxAchievableWns(budget, /*one_sided=*/true, capped);
    if (verbose) {
      logger_->info(
          RSZ,
          3238,
          "  pass {}: one-sided WNS headroom {} -> {} (gain {})",
          pass + 1,
          delayAsString(
              static_cast<sta::Delay>(sched.analyzedWns()), digits, sta_),
          delayAsString(static_cast<sta::Delay>(delta_star), digits, sta_),
          delayAsString(
              static_cast<sta::Delay>(delta_star - sched.analyzedWns()),
              digits,
              sta_));
    }
    // Stop if less than ~one buffer of WNS headroom remains.
    if (delta_star - sched.analyzedWns() < per_buffer_delay) {
      break;
    }
    std::map<const sta::Instance*, double> targets;
    if (!sched.solveSchedule(delta_star, budget, /*one_sided=*/true, targets)) {
      break;
    }

    // Rank registers needing meaningful skew by target (worst path first).
    std::vector<std::pair<const sta::Instance*, double>> ranked;
    for (const auto& [inst, target] : targets) {
      if (target > 0.5 * per_buffer_delay) {
        ranked.emplace_back(inst, target);
      }
    }
    std::ranges::sort(ranked, [](const auto& lhs, const auto& rhs) {
      return lhs.second > rhs.second;
    });
    if (ranked.empty()) {
      break;
    }

    int pass_buffers = 0;
    int pass_kept = 0;
    int pass_rejected = 0;
    est::IncrementalParasiticsGuard guard(estimate_parasitics_);
    for (const auto& [inst, target] : ranked) {
      if (total_buffers >= max_buffer_count) {
        break;
      }
      auto it = inst_clk_pin.find(inst);
      if (it == inst_clk_pin.end()) {
        continue;
      }
      const sta::Pin* clk_pin = it->second;
      if (resizer_->dontTouch(inst) || resizer_->dontTouch(clk_pin)) {
        continue;
      }

      int n = static_cast<int>(std::lround(target / per_buffer_delay));
      n = std::clamp(n, 1, max_buffers_per_reg);
      n = std::min(n, max_buffer_count - total_buffers);
      if (n <= 0) {
        continue;
      }

      const sta::Slack wns_before = sta_->worstSlack(max_);
      const sta::Slack hold_before = sta_->worstSlack(min_);
      const sta::Slack tns_before = sta_->totalNegativeSlack(max_);

      resizer_->journalBegin();
      const int inserted = insertClkDelay(clk_pin, cell, n);
      estimate_parasitics_->updateParasitics();
      const sta::Slack wns_after = sta_->worstSlack(max_);
      const sta::Slack hold_after = sta_->worstSlack(min_);
      const sta::Slack tns_after = sta_->totalNegativeSlack(max_);

      const bool hold_ok = hold_after >= hold_before - eps;
      // WNS-primary acceptance.  Accept a WNS-neutral register only when its
      // TNS reduction outweighs the clock delay we spent on it (so we do not
      // pepper the design with buffers for negligible TNS gains).
      const double tns_gain
          = static_cast<double>(tns_after) - static_cast<double>(tns_before);
      const bool improved = wns_after > wns_before + eps
                            || (std::abs(wns_after - wns_before) <= eps
                                && tns_gain > inserted * per_buffer_delay);

      if (inserted > 0 && hold_ok && improved) {
        resizer_->journalEnd();
        total_buffers += inserted;
        pass_buffers += inserted;
        ++total_regs_skewed;
        ++pass_kept;
        debugPrint(logger_,
                   RSZ,
                   "useful_skew",
                   1,
                   "skew {} by {} buffers (WNS {} -> {})",
                   network_->pathName(inst),
                   inserted,
                   delayAsString(wns_before, digits, sta_),
                   delayAsString(wns_after, digits, sta_));
      } else {
        resizer_->journalRestore();
        ++pass_rejected;
      }
    }
    if (verbose) {
      logger_->info(RSZ,
                    3239,
                    "  pass {}: kept {} registers ({} buffers), rejected {}.",
                    pass + 1,
                    pass_kept,
                    pass_buffers,
                    pass_rejected);
    }
    if (pass_buffers == 0) {
      break;
    }
  }

  sta_->updateTiming(false);
  const sta::Slack wns1 = sta_->worstSlack(max_);
  const sta::Slack tns1 = sta_->totalNegativeSlack(max_);

  logger_->info(RSZ,
                3234,
                "Useful skew: inserted {} clock buffers on {} registers.",
                total_buffers,
                total_regs_skewed);
  logger_->info(RSZ,
                3235,
                "  WNS {} -> {}   TNS {} -> {}",
                delayAsString(wns0, digits, sta_),
                delayAsString(wns1, digits, sta_),
                delayAsString(tns0, digits, sta_),
                delayAsString(tns1, digits, sta_));
}

}  // namespace rsz
