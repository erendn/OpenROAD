// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"

namespace sta {
class Instance;
}

namespace rsz {

// Candidate that removes one existing buffer instance (buffer removal /
// bypass).
//
// The generator pre-checks that the buffer has a single output, that the
// downstream capacitance stays within max-cap limits, and that local slack
// is sufficient.
//
// estimate() surfaces the path-arrival delta that Resizer::estimatedSlackOK
// already computed during the eligibility check.
//
// apply() calls Resizer::removeBuffer to short-circuit the buffer's input and
// output nets and delete the instance.
class UnbufferCandidate : public MoveCandidate
{
 public:
  // === Construction =========================================================
  UnbufferCandidate(Resizer& resizer,
                    const Target& target,
                    sta::Instance* drvr,
                    float net_arrival_delta);

  // === MoveCandidate API ====================================================
  Estimate estimate() override;
  MoveResult apply() override;
  MoveType type() const override { return MoveType::kUnbuffer; }

 private:
  // === Candidate state ======================================================
  sta::Instance* drvr_{nullptr};
  float net_arrival_delta_{0.0f};
};

}  // namespace rsz
