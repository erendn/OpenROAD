// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string_view>
#include <vector>

#include "rsz/GlobalSizingConfig.hh"

namespace rsz {

// The pure core of the N axis (initial solution): which member of a swappable-
// equivalence group each init_mode picks. Split out of InitPass so the
// selection rules can be unit-tested against hand-built groups without a
// Liberty library; InitPass owns the STA-facing half (which instances are
// editable, what the group is, and applying the swap).
//
// HONESTY NOTE (iteration-2 plan §2.1): the ranking key is cell leakage, with
// drive resistance as the tie-break. There is no size or Vt dimension here -
// leakage is the size/drive proxy, so "min"/"max"/"median" mean min/max/median
// OF THE LEAKAGE-RANKED GROUP.
struct InitCandidate
{
  // Cells whose Liberty gives no leakage at all cannot be ranked; the
  // deterministic modes skip them (as the pre-rename selector did), while a
  // random draw still includes them - the group, not the ranking, is what it
  // draws from.
  bool has_leakage = false;
  float leakage = 0.0f;
  float drive_resistance = 0.0f;
  // Final tie-break for the `average` ranking, so the median element does not
  // depend on the order the library happens to hand the group over in.
  std::string_view name;
};

// Index into `group` of the cell `mode` selects. `group[0]` MUST be the
// as-given cell (the modes are defined relative to it, and it is the answer
// when nothing in the group can be ranked).
//
// `draw` is only read by init_mode = random; use initDraw() to produce it. The
// random draw ranges over the whole group in canonical cell-name order (see
// InitSelect.cc); the deterministic modes rank it and therefore skip members
// that cannot be ranked.
size_t selectInitCandidate(const std::vector<InitCandidate>& group,
                           GlobalSizingConfig::InitMode mode,
                           uint64_t draw);

// The pure core of min_size_fixviol's repair step: which member of a gate's
// leakage-ranked group the pass upsizes a violating gate to.
//
// Walks the SAME ascending ranking `average` medians (leakage, then the weaker
// drive, then cell name), starting one place above `current`, and returns the
// FIRST member for which `clears` is true - the cheapest cell that removes the
// gate's electrical violation, not the strongest one in the group. That is the
// pass's whole policy: it repairs violations, it does not optimize. `clears` is
// the STA-facing half (InitPass.cc) and is called at most once per member, in
// ranking order.
//
// Returns `current` unchanged when the group is exhausted without a member that
// clears - the pass will not spend leakage on a gate it cannot actually fix.
// Such gates are counted and reported, so "remaining violations" is a number
// the log states rather than a silence.
//
// What the SWEEP then does with such a gate is the A2 axis' business, not this
// pass's: under output_drc_veto = absolute its DRC filter admits only a
// candidate that fully clears, so the gate stays where this pass left it for
// the whole run; under relative the filter admits any candidate that does not
// worsen the violation, so the gate can still be sized (see
// GlobalSizingConfig::OutputDrcVeto).
//
// Because the ranking is a total order (cell name is the final tie-break), the
// answer is a function of the group's MEMBERS and of `clears`, never of the
// order the library handed the group over in.
size_t selectFixviolUpsize(const std::vector<InitCandidate>& group,
                           size_t current,
                           const std::function<bool(size_t)>& clears);

// The per-instance random draw for init_mode = random.
//
// Two properties this function exists to guarantee, both required by the
// dissertation's variability study:
//   (a) INDEPENDENCE FROM PLACEMENT. init_seed is its own flag and is never
//       derived from the global-placement seed. Netlist initialization and
//       placement perturbation are two separate variation sources; deriving one
//       from the other would confound them in exactly the comparison they exist
//       to support.
//   (b) ORDER AND THREAD INDEPENDENCE. The draw is a hash of (init_seed,
//       instance name), not a stream drawn from a shared generator, so an
//       instance gets the same cell whatever order the instance iterator visits
//       it in and whatever the thread count is. A stateful RNG would make the
//       initial solution a function of the iteration order, which is neither
//       stable across OpenROAD versions nor reproducible from the log. The
//       draw's other half is in selectInitCandidate: it indexes the group in
//       CANONICAL cell-name order, so the group vector's own order - which is
//       an unstable sort by drive resistance with the as-given cell in front -
//       cannot move the answer either.
//
// What (a) does NOT claim: the GROUP each draw ranges over is a property of the
// incoming netlist, and the netlist reaching this phase does depend on
// placement. That is inherent, not an artifact - Resizer::getSwappableCells
// filters candidates relative to the cell that is actually there (its area and
// leakage ratio limits are ratios TO it), so "the swappable group" only exists
// relative to an incoming cell. What is independent of placement is the seed
// and the draw it produces.
uint64_t initDraw(int init_seed, std::string_view instance_name);

}  // namespace rsz
