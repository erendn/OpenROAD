// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <optional>
#include <vector>

#include "OptimizationPolicy.hh"

namespace sta {
class Instance;
class LibertyCell;
class Pin;
}  // namespace sta

namespace rsz {

class DelayEstimator;

class PowerReliefPolicy : public OptimizationPolicy
{
 public:
  using OptimizationPolicy::OptimizationPolicy;

  void iterate() override;
  const char* name() const override { return "PowerReliefPolicy"; }

 private:
  // One scheduled but not-yet-applied power-relief move.
  // Three flavors: VT swap and size-down both replace a cell, unbuffer removes
  // the instance entirely.
  struct BatchEntry
  {
    enum class Kind
    {
      kVtSwap,
      kSizeDown,
      kUnbuffer,
    };
    Kind kind = Kind::kVtSwap;
    sta::Instance* inst = nullptr;
    sta::Pin* drvr_pin = nullptr;
    sta::LibertyCell* curr_cell = nullptr;
    sta::LibertyCell* target_cell = nullptr;
    sta::Slack inst_slack = 0;
  };

  sta::Slack getInstanceSlack(sta::Instance* inst);
  sta::Pin* outputPin(sta::Instance* inst);

  // Find the highest-sensitivity (Δleakage / Δdelay) candidate cell for `inst`.
  // VT swap pool is searched first; size-down pool is searched only if VT
  // yields no candidate. Returns nullopt when no candidate both saves leakage
  // and fits inside `inst_slack`.
  std::optional<BatchEntry> findBestSwap(sta::Instance* inst,
                                         sta::Pin* drvr_pin,
                                         sta::Slack inst_slack);

  // Apply a batch of scheduled moves under a single journal, refresh STA,
  // check WNS/TNS regression, and commit or bisect-and-retry on failure.
  // Returns the number of moves that ultimately committed.  Updates
  // `pre_wns` / `pre_tns` to the post-commit baseline so later batches
  // ratchet against the new slack frontier.
  int applyBatchWithBisection(const std::vector<BatchEntry>& entries,
                              sta::Slack& pre_wns,
                              sta::Slack& pre_tns);

  // Run one convergence pass: Gather every positive-slack instance, score with
  // sensitivity ranking, batch-apply with bisection.
  int runPass(sta::Slack& pre_wns, sta::Slack& pre_tns);

  // One-shot pass that attempts to remove every buffer instance whose worst pin
  // slack exceeds the configured margin and that passes electrical legality
  // check.
  int runUnbufferPass(sta::Slack& pre_wns, sta::Slack& pre_tns);

  // Max moves per regression-checked journal batch
  int batch_size_ = 8;

  // Max convergence passes before forced exit. Early-exits when a pass commits
  // zero moves.
  int max_passes_ = 1;
};

}  // namespace rsz
