// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "ElectricalModel.hh"

#include <algorithm>

#include "db_sta/dbSta.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/Sta.hh"
#include "sta/Transition.hh"

namespace rsz {

float outputMaxCapExcess(const sta::LibertyPort* output_port,
                         const float output_cap,
                         const sta::MinMax* max_mm)
{
  float max_cap = 0.0f;
  bool cap_limit_exists = false;
  output_port->capacitanceLimit(max_mm, max_cap, cap_limit_exists);
  if (!cap_limit_exists || max_cap <= 0.0f) {
    return 0.0f;
  }
  return std::max(0.0f, output_cap - max_cap);
}

bool checkOutputMaxCap(const sta::LibertyPort* output_port,
                       const float output_cap,
                       const sta::MinMax* max_mm)
{
  return outputMaxCapExcess(output_port, output_cap, max_mm) > 0.0f;
}

float outputSlewFactor(const float slew,
                       const float drive_res,
                       const float load)
{
  return (drive_res > 0.0f && load > 0.0f) ? slew / (drive_res * load) : 0.0f;
}

float outputMaxSlewExcess(sta::dbSta* sta,
                          const sta::LibertyPort* candidate_port,
                          const float output_slew_factor,
                          const float output_cap,
                          const sta::Scene* scene,
                          const sta::MinMax* max_mm)
{
  const float new_slew
      = output_slew_factor * candidate_port->driveResistance() * output_cap;
  float max_slew = 0.0f;
  bool slew_limit_exists = false;
  sta->findSlewLimit(
      candidate_port, scene, max_mm, max_slew, slew_limit_exists);
  if (!slew_limit_exists) {
    return 0.0f;
  }
  return std::max(0.0f, new_slew - max_slew);
}

bool checkOutputMaxSlew(sta::dbSta* sta,
                        const sta::LibertyPort* candidate_port,
                        const float output_slew_factor,
                        const float output_cap,
                        const sta::Scene* scene,
                        const sta::MinMax* max_mm)
{
  return outputMaxSlewExcess(
             sta, candidate_port, output_slew_factor, output_cap, scene, max_mm)
         > 0.0f;
}

float portInputCap(const sta::LibertyCell* cell,
                   const char* port_name,
                   const sta::MinMax* max_mm)
{
  const sta::LibertyPort* port = cell->findLibertyPort(port_name);
  if (port == nullptr) {
    return 0.0f;
  }
  float cap = 0.0f;
  for (auto rf : sta::RiseFall::range()) {
    cap = std::max(cap, port->capacitance(rf, max_mm));
  }
  return cap;
}

}  // namespace rsz
