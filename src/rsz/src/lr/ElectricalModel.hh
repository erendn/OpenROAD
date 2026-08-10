// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

namespace sta {
class dbSta;
class LibertyCell;
class LibertyPort;
class MinMax;
class Scene;
}  // namespace sta

namespace rsz {

// The candidate ELECTRICAL MODEL, in one place.
//
// Three passes measure the same thing - a cell's max-cap and max-slew standing
// at a pin - and they are only meaningful if they measure it the same way:
//   * the sweep's own DRC filter (LRSubproblem::candidateDrcOkSnapshot), which
//     decides what a sweep may commit. Under `output_drc_veto = absolute` it
//     asks "would this cell leave the gate free of violations?"; under
//     `relative` it asks the weaker "...no worse off than the cell it
//     replaces?" (outputLimitAdmits below). Both readings are the same
//     measurement against a different bar;
//   * the min_size_fixviol init repair (InitPass.cc), which hands the sweep its
//     starting netlist - a repair judged by a stricter rule would hand the
//     sweep gates it considers fine, one judged by a looser rule would leave
//     gates the sweep then refuses to touch;
//   * the post-sweep max-cap re-check (CapRecheck.cc), which reads the same
//     input capacitances back to attribute a violation to a mover.
// Keeping one copy is what makes that agreement a property of the code rather
// than of three comments promising it.
//
// All of them are pure Liberty/SDC reads with no graph mutation, so they are
// safe to call from the sweep's worker threads.

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

// The Elmore-linear slew calibration k = slew / (R * C), so that
// k * R_candidate * C reproduces the MEASURED slew for the incumbent cell and
// scales it by the candidate's drive resistance and the live load. Returns 0 on
// a degenerate measurement (no load or no drive resistance), which makes the
// slew leg abstain rather than guess.
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

// The output-side DRC verdict for ONE pin against ONE limit (the A2 axis'
// `output_drc_veto`). Both excesses are the clamped-at-0 figures above: the
// candidate's, and the CURRENT cell's own on the same pin against the same
// limit.
//
//   absolute (`relative == false`): admit only a candidate that is itself
//     clean. Identical to `!checkOutput...`, which is what keeps the shipped
//     default byte-for-byte what it was before the mode existed.
//   relative (`relative == true`): Flach Alg. 4 line 6, "if load violation has
//     INCREASED" (flach_et_al.md:148,166) and Chinnery §6's "alternatives that
//     would increase max-load-capacitance or max-input-slew violations are
//     skipped" (chinnery_et_al.md:102). On a pin the current cell already
//     violates, admit any candidate that does not violate it by MORE.
//
// TIE SEMANTICS: an equal-violation candidate is ADMITTED. Both papers forbid
// an INCREASE, not a violation, and a Vth swap at equal drive is exactly the
// equal-excess move they mean to leave available. `<=`, not `<`.
//
// When the current cell is clean (cur_excess == 0) the relative rule collapses
// to the absolute one, so a clean pin behaves identically under both modes -
// the mode only ever governs an ALREADY-violating pin.
inline bool outputLimitAdmits(const float cand_excess,
                              const float cur_excess,
                              const bool relative)
{
  return cand_excess <= (relative ? cur_excess : 0.0f);
}

// Worst-transition input capacitance of `cell`'s named port - the load the cell
// presents to whatever drives that pin. 0 when the cell has no such port (an
// incompatible swap none of the three passes would make).
float portInputCap(const sta::LibertyCell* cell,
                   const char* port_name,
                   const sta::MinMax* max_mm);

}  // namespace rsz
