// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "UnbufferCandidate.hh"

#include <vector>

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"
#include "sta/Network.hh"

namespace rsz {

UnbufferCandidate::UnbufferCandidate(Resizer& resizer,
                                     const Target& target,
                                     sta::Instance* drvr,
                                     const float net_arrival_delta)
    : MoveCandidate(resizer, target),
      drvr_(drvr),
      net_arrival_delta_(net_arrival_delta)
{
}

Estimate UnbufferCandidate::estimate()
{
  // The generator's slack guard (Resizer::estimatedSlackOK) already vetted
  // feasibility and computed the path-arrival delta. Surface that value as
  // delta_arrival without recomputing it.
  //
  // net_arrival_delta = delay_imp - delay_degrad already scores the unified
  // {fanin, target}-stage arrival axis the other estimated moves use: the
  // target term is the removed buffer's stage delay (delay_imp) and the fanin
  // term is the previous driver's recomputed delay at the merged load
  // (delay_degrad), so no additional fanin penalty applies here.
  return {.legal = true,
          .score = net_arrival_delta_,
          .delta_arrival = net_arrival_delta_,
          .scope = EstimateScope::kLocal,
          .estimated = true};
}

MoveResult UnbufferCandidate::apply()
{
  const bool accepted = resizer_.removeBuffer(drvr_);
  return {
      .accepted = accepted,
      .type = MoveType::kUnbuffer,
      .move_count = accepted ? 1 : 0,
      .touched_instances = accepted ? std::vector<sta::Instance*>{drvr_}
                                    : std::vector<sta::Instance*>{},
  };
}

}  // namespace rsz
