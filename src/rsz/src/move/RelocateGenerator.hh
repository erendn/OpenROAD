// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>
#include <vector>

#include "MoveCandidate.hh"
#include "MoveGenerator.hh"
#include "OptimizerTypes.hh"
#include "odb/geom.h"
#include "rsz/Resizer.hh"

namespace rsz {

// Produces one slack-guided relocation candidate per path-driver target.
// Target location is the slack-weighted centroid of negative-slack fanouts,
// optionally refined by a 1D Elmore-optimal position on the line from the
// previous driver to that centroid.
class RelocateGenerator : public MoveGenerator
{
 public:
  explicit RelocateGenerator(const GeneratorContext& context);

  MoveType type() const override { return MoveType::kRelocate; }
  bool isApplicable(const Target& target) const override;
  std::vector<std::unique_ptr<MoveCandidate>> generate(
      const Target& target) override;

  // Minimum HPWL (DBU) from driver to target to attempt a move. Below
  // this, the move is noise-level and only adds DPL churn.
  static constexpr int kMinMoveThresholdDbu = 20000;

 private:
  // One output-net sink kept for the collateral-slack back-off guard.
  struct SinkInfo
  {
    odb::Point loc;
    double pin_cap{0.0};
    sta::Slack slack{0.0};  // worst over rise/fall
  };

  // Returns the Elmore-optimal target (or centroid fallback), after backing
  // the displacement off so no currently-safe sink is pushed into violation.
  // Sets `out_target` and returns true on success; returns false if there are
  // no critical loads or the (possibly backed-off) move is below the noise
  // threshold.
  bool computeTargetLocation(const Target& gen_target,
                             sta::Vertex* drvr_vertex,
                             const sta::RiseFall* rf,
                             const odb::Point& drvr_loc,
                             odb::Point& out_target) const;

  // Shrinks the move along orig->requested so every currently-safe sink keeps
  // at least `setup_slack_margin` of slack under a first-order wire-delay
  // model. Returns the largest safe location; false if even a minimal safe
  // step stays below kMinMoveThresholdDbu. A no-op (returns `requested`) when
  // there is no wire RC model or no safe sinks to protect.
  bool backOffForSafeSinks(const odb::Point& orig,
                           const odb::Point& requested,
                           const std::vector<SinkInfo>& safe_sinks,
                           float margin,
                           double wire_res,
                           double wire_cap,
                           odb::Point& out_target) const;

  // Slack band (s) added above setup_slack_margin to decide how aggressively
  // marginal positive-slack sinks pull on the centroid. 0 reproduces the
  // negative-slack-only centroid. Read once from RSZ_RELOCATE_SLACK_BAND.
  double slack_band_{0.0};
};

}  // namespace rsz
