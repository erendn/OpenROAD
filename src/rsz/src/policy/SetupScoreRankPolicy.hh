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
// applicable candidate, partitions them into two tiers, and commits the best
// one.
//
//   Tier 1: Candidates whose Estimate.estimated == true.
//           Ranked by Estimate.score (descending). No threshold -- a negative
//           score is still preferred over Tier 2 because the estimator may be
//           wrong in direction at this stage and we use it only to rank, not to
//           gate.
//   Tier 2: Candidates whose Estimate.estimated == false (today: BufferMove,
//           SplitLoadMove). Tried in legacy -sequence order so they keep their
//           existing priority without being sentinel-favored.
//
// On commit-time rejection inside Tier 1 (for example a max-cap re-check flip),
// the policy falls through to the next Tier 1 candidate and only falls into
// Tier 2 once all Tier 1 candidates have been exhausted.
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
