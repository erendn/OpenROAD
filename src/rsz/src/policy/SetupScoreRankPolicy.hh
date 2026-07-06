// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <optional>
#include <unordered_set>

#include "OptimizerTypes.hh"
#include "SetupLegacyPolicy.hh"

namespace rsz {

// Experimental setup-repair phase that uses MoveCandidate::estimate() to pick
// the per-target move (selected via -phases "SCORE_RANK").
//
// Behaviorally identical to SetupLegacyPolicy except in the per-target
// candidate decision: instead of trying generators in -sequence order and
// committing the first acceptable candidate, this policy gathers every
// applicable candidate and tries them in three bands, committing the first
// acceptable one.
//
//   Band 1: Estimated candidates with score > 0, ranked by Estimate.score
//           (descending, stable -- ties keep -sequence order). Moves the
//           model predicts to improve arrival get first refusal.
//   Band 2: Unestimated candidates (Estimate.estimated == false; today
//           BufferMove and SplitLoadMove) in legacy -sequence order. This
//           preserves buffering's legacy role as the fallback after the
//           cheap sizing moves.
//   Band 3: Estimated candidates with score <= 0, ranked descending. Scores
//           rank candidates rather than gate them -- a negative prediction
//           only loses first refusal, it stays reachable.
//
// On commit-time rejection (for example a max-cap re-check flip), the policy
// falls through to the next candidate in band order.
class SetupScoreRankPolicy : public SetupLegacyPolicy
{
 public:
  SetupScoreRankPolicy(Resizer& resizer,
                       MoveCommitter& committer,
                       RepairSetupContext& setup_context,
                       const OptimizerRunConfig& config);

  const char* name() const override { return "SetupScoreRankPolicy"; }

 protected:
  bool tryRepairTarget(const Target& target,
                       int repairs_per_pass,
                       int& changed,
                       const std::unordered_set<MoveType>* rejected_types,
                       std::optional<MoveType>& accepted_type) override;

  const char* phaseName() const override { return "SCORE_RANK"; }
  const char* phaseSummaryTitle() const override
  {
    return "SCORE_RANK Phase Summary";
  }
  const char* phaseEndpointProfilerTitle() const override
  {
    return "SCORE_RANK Phase Endpoint Profiler";
  }
};

}  // namespace rsz
