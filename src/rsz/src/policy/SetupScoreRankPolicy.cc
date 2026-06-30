// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "SetupScoreRankPolicy.hh"

#include <algorithm>
#include <memory>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

#include "MoveCandidate.hh"
#include "MoveCommitter.hh"
#include "MoveGenerator.hh"
#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"
#include "sta/GraphClass.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

namespace {

// One per-target candidate paired with its estimate and originating move type.
// Held by value in a single owning vector so the two tiers can index into the
// same storage without juggling lifetimes.
struct ScoredCandidate
{
  std::unique_ptr<MoveCandidate> candidate;
  Estimate estimate;
  MoveType type;
};

}  // namespace

SetupScoreRankPolicy::SetupScoreRankPolicy(Resizer& resizer,
                                           MoveCommitter& committer,
                                           RepairSetupContext& setup_context,
                                           const OptimizerRunConfig& config)
    : SetupLegacyPolicy(resizer, committer, setup_context, config)
{
  is_experimental = true;
}

bool SetupScoreRankPolicy::tryRepairTarget(
    const Target& target,
    const int repairs_per_pass,
    int& changed,
    const std::unordered_set<MoveType>* rejected_types,
    std::optional<MoveType>& accepted_type)
{
  Target live_target = target;
  sta::Vertex* live_vertex = live_target.vertex(resizer_);
  if (live_vertex != nullptr) {
    live_target.fanout = fanout(live_vertex);
  }

  // Collect candidates into two tiers based on whether the candidate produced
  // a real timing estimate. Generators are walked in move_sequence_ order so
  // tier-2 insertion order matches the legacy -sequence ordering.
  std::vector<ScoredCandidate> estimated_pool;
  std::vector<ScoredCandidate> unestimated_pool;
  for (const std::unique_ptr<MoveGenerator>& generator_ptr : move_generators_) {
    MoveGenerator& generator = *generator_ptr;
    const MoveType type = generator.type();
    if (rejected_types != nullptr && rejected_types->contains(type)) {
      continue;
    }
    if (!generator.isApplicable(live_target)) {
      continue;
    }

    debugPrint(logger_,
               RSZ,
               "score_rank",
               2,
               "Considering {} for {}",
               generator.name(),
               network_->pathName(live_target.driver_pin));

    auto candidates = generator.generate(live_target);
    for (std::unique_ptr<MoveCandidate>& candidate : candidates) {
      Estimate estimate = candidate->estimate();
      if (!estimate.legal) {
        continue;
      }
      ScoredCandidate entry{std::move(candidate), estimate, type};
      if (estimate.estimated) {
        estimated_pool.push_back(std::move(entry));
      } else {
        unestimated_pool.push_back(std::move(entry));
      }
    }
  }

  // Tier 1: Estimated candidates ranked by score (descending). No threshold:
  // estimates may be wrong in direction at this stage, so we only use them to
  // order candidates, not to reject any.
  std::ranges::stable_sort(
      estimated_pool,
      [](const ScoredCandidate& lhs, const ScoredCandidate& rhs) {
        return lhs.estimate.score > rhs.estimate.score;
      });

  // Commit attempt helper -- shared between the two tiers.
  auto tryCommit = [&](ScoredCandidate& entry) -> bool {
    committer_.trackMoveAttempt(live_target.driver_pin, entry.type);
    const MoveResult result = committer_.commit(*entry.candidate);
    if (!result.accepted) {
      return false;
    }
    changed += repairProgressIncrement(result.type, repairs_per_pass);
    accepted_type = result.type;
    return true;
  };

  for (ScoredCandidate& entry : estimated_pool) {
    debugPrint(logger_,
               RSZ,
               "score_rank",
               3,
               "Tier1 candidate move={} score={:e}",
               moveName(entry.type),
               entry.estimate.score);
    if (tryCommit(entry)) {
      return true;
    }
  }

  // Tier 2: Unestimated candidates in legacy -sequence order. Reached only once
  // every estimated candidate has been tried and rejected at commit time, so
  // BufferMove / SplitLoadMove keep their existing priority while estimated
  // moves get first refusal.
  for (ScoredCandidate& entry : unestimated_pool) {
    debugPrint(logger_,
               RSZ,
               "score_rank",
               3,
               "Tier2 candidate move={}",
               moveName(entry.type));
    if (tryCommit(entry)) {
      return true;
    }
  }

  return false;
}

}  // namespace rsz
