// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string_view>
#include <vector>

#include "rsz/GlobalSizingConfig.hh"

namespace rsz {

// Selection rules of the initial solution: which member of a swappable-
// equivalence group each init_mode picks. Kept separate from InitPass so the
// rules can be unit-tested on hand-built groups without a Liberty library;
// InitPass does the STA-facing part (which instances are editable, what the
// group is, and applying the swap).
//
// The ranking key is cell leakage, with drive resistance as the tie-break.
// There is no separate size or Vt dimension: leakage stands in for size and
// drive, so "min", "max" and "median" refer to the leakage ranking.
struct InitCandidate
{
  // Cells whose Liberty gives no leakage cannot be ranked. The deterministic
  // modes skip them, while a random draw still includes them, because it
  // draws from the group, not from the ranking.
  bool has_leakage = false;
  float leakage = 0.0f;
  float drive_resistance = 0.0f;
  // Final tie-break for the `average` ranking, so the median element does not
  // depend on the order the library happens to hand the group over in.
  std::string_view name;
};

// Index into `group` of the cell `mode` selects. `group[0]` must be the
// as-given cell (the modes are defined relative to it, and it is the answer
// when nothing in the group can be ranked).
//
// `draw` is only read by init_mode = random; use initDraw() to produce it. The
// random draw ranges over the whole group in cell-name order (see
// InitSelect.cc); the deterministic modes rank the group and therefore skip
// members that cannot be ranked.
size_t selectInitCandidate(const std::vector<InitCandidate>& group,
                           GlobalSizingConfig::InitMode mode,
                           uint64_t draw);

// Repair step of min_size_fixviol and min_size_fixcap: which member of a
// gate's leakage-ranked group the pass upsizes a violating gate to.
//
// Walks the same ascending ranking that `average` takes the median of
// (leakage, then the weaker drive, then cell name), starting one place above
// `current`, and returns the first member for which `clears` is true: the
// cheapest cell that removes the gate's electrical violation, not the
// strongest one in the group. The pass repairs violations; it does not
// optimize. `clears` is the STA-facing check (clearsViolations in
// ViolationRepair.hh) and is called at most once per member, in ranking order.
//
// Returns `current` unchanged when no member clears the violation, since the
// pass does not spend leakage on a gate it cannot fix. Such gates are counted
// and reported in the log.
//
// What the sweep then does with such a gate depends on output_drc_veto: under
// absolute, the DRC filter admits only a candidate that fully clears, so the
// gate stays where this pass left it for the whole run; under relative, it
// admits any candidate that does not worsen the violation, so the gate can
// still be sized (see GlobalSizingConfig::OutputDrcVeto).
//
// The ranking is a total order (cell name is the final tie-break), so the
// answer depends only on the group's members and on `clears`, not on the
// order in which the library lists the group.
size_t selectFixviolUpsize(const std::vector<InitCandidate>& group,
                           size_t current,
                           const std::function<bool(size_t)>& clears);

// The slew fix pass (slew_fix_pass): which member of a gate's leakage-ranked
// group the pass upsizes a gate over its max slew to. Sharma et al. (ICCAD
// 2015, Sec. III-A) upsize such gates minimally, and keep the max-cap limits
// met throughout (Sec. II).
//
// Climbs the same ranking as selectFixviolUpsize, from one place above
// `current`. `slew_excess(i)` is how far member i's output slew would exceed
// its limit (0 = within it), or nullopt when member i may not be installed.
// Returns the first member with excess 0, the lowest-leakage cell that clears
// the violation. If none clears it, returns the member with the smallest
// excess if that is below `current_excess`, the lower-ranked one on a tie;
// otherwise `current`. `slew_excess` is called at most once per member, in
// ranking order.
size_t selectSlewFixUpsize(
    const std::vector<InitCandidate>& group,
    size_t current,
    float current_excess,
    const std::function<std::optional<float>(size_t)>& slew_excess);

// The per-instance random draw for init_mode = random. It guarantees two
// properties:
//   (a) Independence from placement. init_seed is its own option and is never
//       derived from the global placement seed, so netlist initialization and
//       placement perturbation remain separate sources of variation.
//   (b) Independence from visit order and thread count. The draw is a hash of
//       (init_seed, instance name), not a stream from a shared generator, so
//       an instance gets the same cell whatever order it is visited in. A
//       stateful RNG would make the initial solution depend on the iteration
//       order, which is neither stable across OpenROAD versions nor
//       reproducible from the log. selectInitCandidate completes this by
//       indexing the group in cell-name order, so the order of the group
//       vector (an unstable sort by drive resistance with the as-given cell in
//       front) cannot change the result either.
//
// The group each draw ranges over still depends on the incoming netlist, and
// so on placement: Resizer::getSwappableCells filters candidates relative to
// the current cell (its area and leakage limits are ratios to that cell).
// Only the seed and the draw are independent of placement.
uint64_t initDraw(int init_seed, std::string_view instance_name);

}  // namespace rsz
