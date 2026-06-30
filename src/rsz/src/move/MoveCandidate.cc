// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "MoveCandidate.hh"

#include <optional>

#include "DelayEstimator.hh"
#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"

namespace rsz {

MoveCandidate::MoveCandidate(Resizer& resizer, const Target& target)
    : resizer_(resizer), target_(target)
{
}

MoveResult MoveCandidate::rejectedMove() const
{
  return {
      .accepted = false,
      .type = type(),
      .touched_instances = {},
  };
}

Estimate MoveCandidate::estimateCellSwap(const sta::LibertyCell* replacement)
{
  Estimate estimate;
  // Feasibility was already vetted by the generator; estimation only scores
  // the timing impact, so it never flips `legal`.
  estimate.legal = true;
  estimate.scope = EstimateScope::kLocal;

  // Prefer the MT-prepared snapshot; build it lazily for single-threaded
  // policies that do not run the prepare stage.
  const ArcDelayState* context = nullptr;
  std::optional<ArcDelayState> lazy_context;
  if (target_.isPrepared(kArcDelayStateCache)) {
    context = &target_.arc_delay.value();
  } else {
    lazy_context
        = DelayEstimator::buildContext(resizer_, target_, /*delay_levels=*/0);
    if (lazy_context.has_value()) {
      context = &lazy_context.value();
    }
  }

  if (context != nullptr) {
    const DelayEstimate delay_estimate
        = DelayEstimator::estimate(*context, replacement);
    estimate.delta_arrival = delay_estimate.arrival_impr;
    estimate.score = delay_estimate.arrival_impr;
    estimate.estimated = true;
  }
  // On buildContext failure delta_arrival stays 0, estimated stays false, and
  // legal stays true: no timing insight, but no change to legacy move
  // selection. A score-ranking policy will treat this as unestimated and place
  // it in the tier-2 fallback.
  return estimate;
}

}  // namespace rsz
