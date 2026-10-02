// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <optional>

namespace sta {
class dbSta;
class LibertyCell;
class LibertyPort;
class MinMax;
class Scene;
}  // namespace sta

namespace rsz {

// The electrical model used to judge candidate cells.
//
// The passes below measure a cell's max-cap and max-slew standing at a pin,
// and they must measure it the same way:
//   * the sweep's DRC filter (LRSubproblem::candidateDrcOkSnapshot), which
//     decides what a sweep may commit. With output_drc_veto = absolute it asks
//     whether the candidate leaves the pin free of violations; with relative
//     it asks whether the candidate is no worse than the cell it replaces
//     (outputLimitAdmits below);
//   * the min_size_fixviol init repair and the max-capacitance fix pass
//     (ViolationRepair.cc). The init repair produces the sweep's starting
//     netlist. With a stricter rule a repair would change gates the sweep
//     already accepts; with a looser one it would leave gates the sweep then
//     refuses to touch;
//   * the post-sweep max-cap re-check (CapRecheck.cc), which uses the same
//     input capacitances to attribute a violation to a moved gate.
//
// All functions are pure Liberty/SDC reads with no graph mutation, so they are
// safe to call from the sweep's worker threads.

// The Liberty max_capacitance of `output_port`, in farads, or nothing when the
// port declares no positive limit.
std::optional<float> libertyMaxCap(const sta::LibertyPort* output_port,
                                   const sta::MinMax* max_mm);

// By how much would `output_port` driving `output_cap` exceed its Liberty
// max_capacitance, in farads? Clamped at 0, so 0 means "within the limit" and
// also "the port declares no limit" - there is nothing to violate either way.
// This is the primitive; the boolean below is `excess > 0`.
float outputMaxCapExcess(const sta::LibertyPort* output_port,
                         float output_cap,
                         const sta::MinMax* max_mm);

// Would `output_port` driving `output_cap` violate its Liberty max_capacitance?
// False when the port declares no limit - there is nothing to violate.
bool checkOutputMaxCap(const sta::LibertyPort* output_port,
                       float output_cap,
                       const sta::MinMax* max_mm);

// Slew calibration factor k = slew / (R * C) for a linear (Elmore-style) slew
// model. k * R_candidate * C reproduces the measured slew for the current cell
// and scales it by the candidate's drive resistance. Returns 0 when there is no
// load or no drive resistance, which turns the slew check off rather than
// guessing.
float outputSlewFactor(float slew, float drive_res, float load);

// By how much would `candidate_port` driving `output_cap` exceed its slew
// limit, in seconds, with the slew estimated as
// `output_slew_factor * driveResistance * output_cap`? Clamped at 0 (0 = within
// the limit, or no slew limit applies to the port). The primitive behind the
// boolean below.
float outputMaxSlewExcess(sta::dbSta* sta,
                          const sta::LibertyPort* candidate_port,
                          float output_slew_factor,
                          float output_cap,
                          const sta::Scene* scene,
                          const sta::MinMax* max_mm);

// Would `candidate_port` driving `output_cap` violate its slew limit, with the
// slew estimated as `output_slew_factor * driveResistance * output_cap`? False
// when no slew limit applies to the port.
bool checkOutputMaxSlew(sta::dbSta* sta,
                        const sta::LibertyPort* candidate_port,
                        float output_slew_factor,
                        float output_cap,
                        const sta::Scene* scene,
                        const sta::MinMax* max_mm);

// Output-side DRC decision for one pin against one limit (output_drc_veto).
// Both arguments are clamped-at-0 excesses from the functions above: the
// candidate's, and the current cell's own on the same pin and limit.
//
//   absolute (`relative == false`): admit only a candidate that is itself
//     within the limit. Same result as !checkOutputMaxCap/!checkOutputMaxSlew.
//   relative (`relative == true`): admit any candidate that does not violate
//     the limit by more than the current cell does. This is the rule of Flach
//     et al., TCAD 2014, Alg. 4 line 6 ("if load violation has increased") and
//     Chinnery et al., ISPD 2022, Sec. 4 (skip alternatives that would increase
//     max load capacitance or max input slew violations).
//
// A candidate with an equal excess is admitted (`<=`, not `<`): both papers
// forbid an increase, and a Vth swap at equal drive strength is exactly such an
// equal-excess move.
//
// When the current cell is within the limit (cur_excess == 0) both modes give
// the same answer; the mode only matters on a pin that already violates.
inline bool outputLimitAdmits(const float cand_excess,
                              const float cur_excess,
                              const bool relative)
{
  return cand_excess <= (relative ? cur_excess : 0.0f);
}

// Worst rise/fall input capacitance of `cell`'s named port, i.e. the load the
// cell presents to the driver of that pin. 0 when the cell has no such port.
float portInputCap(const sta::LibertyCell* cell,
                   const char* port_name,
                   const sta::MinMax* max_mm);

}  // namespace rsz
