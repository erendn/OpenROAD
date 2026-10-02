// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "Guards.hh"

#include <algorithm>

namespace rsz {

float flachGamma(const float worst_slack,
                 const float T,
                 const float tolerance_scale)
{
  if (T <= 0.0f) {
    return 1.0f;
  }
  const float violation = -std::min(0.0f, worst_slack);
  return 1.0f + tolerance_scale * (violation / T);
}

float negativeSlackAfter(const float slack, const float delay_delta)
{
  return std::min(0.0f, slack - delay_delta);
}

bool localSlackVetoOk(const float candidate_local_slack,
                      const float original_local_slack,
                      const float gamma)
{
  return candidate_local_slack >= gamma * original_local_slack;
}

bool localSlackVetoOkGated(const bool active,
                           const float candidate_local_slack,
                           const float original_local_slack,
                           const float gamma)
{
  return !active
         || localSlackVetoOk(
             candidate_local_slack, original_local_slack, gamma);
}

}  // namespace rsz
