// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "InitSelect.hh"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string_view>
#include <vector>

#include "rsz/GlobalSizingConfig.hh"

namespace rsz {

namespace {

using InitMode = GlobalSizingConfig::InitMode;

// min_size / max_size. Rank by leakage; on a leakage tie prefer the weaker
// drive (higher drive resistance) for min_size and the stronger for max_size;
// on a tie in both keep the earlier candidate in group order. That is why this
// is a scan and not a sort: `group[0]` (the as-given cell) starts as the
// incumbent and keeps a full tie.
size_t scanExtremum(const std::vector<InitCandidate>& group,
                    const bool want_min)
{
  size_t best = 0;
  for (size_t i = 1; i < group.size(); ++i) {
    const InitCandidate& cand = group[i];
    if (!cand.has_leakage) {
      continue;
    }
    if (!group[best].has_leakage) {
      best = i;
      continue;
    }
    if (cand.leakage != group[best].leakage) {
      const bool better = want_min ? cand.leakage < group[best].leakage
                                   : cand.leakage > group[best].leakage;
      if (better) {
        best = i;
      }
      continue;
    }
    const float best_drive = group[best].drive_resistance;
    if (cand.drive_resistance == best_drive) {
      continue;
    }
    const bool better_drive = want_min ? cand.drive_resistance > best_drive
                                       : cand.drive_resistance < best_drive;
    if (better_drive) {
      best = i;
    }
  }
  return best;
}

// The group's ascending ranking, as indices into `group`. The primary and
// secondary keys are those of scanExtremum (so on a group with no full ties
// the first and last elements are the min_size and max_size picks); cell name
// breaks a full tie, so the ranking never depends on the library's group
// order. Shared by `average` (which takes its median) and by the repair walks
// (which climb it), so they agree on what "up" means.
std::vector<size_t> rankedAscending(const std::vector<InitCandidate>& group)
{
  std::vector<size_t> ranked;
  ranked.reserve(group.size());
  for (size_t i = 0; i < group.size(); ++i) {
    // Both keys must be finite, not merely present. A NaN key would make the
    // comparator below non-transitive (NaN compares false in both directions
    // while the rest stay ordered), and an invalid strict weak ordering is
    // undefined behavior in std::sort, which can crash rather than just give a
    // wrong median. scanExtremum needs no such guard, since it only does
    // pairwise comparisons in a linear scan.
    if (group[i].has_leakage && std::isfinite(group[i].leakage)
        && std::isfinite(group[i].drive_resistance)) {
      ranked.push_back(i);
    }
  }
  std::sort(ranked.begin(), ranked.end(), [&group](size_t a, size_t b) {
    const InitCandidate& ca = group[a];
    const InitCandidate& cb = group[b];
    if (ca.leakage != cb.leakage) {
      return ca.leakage < cb.leakage;
    }
    if (ca.drive_resistance != cb.drive_resistance) {
      return ca.drive_resistance > cb.drive_resistance;
    }
    return ca.name < cb.name;
  });
  return ranked;
}

// average: the lower median of the ascending ranking.
size_t lowerMedian(const std::vector<InitCandidate>& group)
{
  const std::vector<size_t> ranked = rankedAscending(group);
  if (ranked.empty()) {
    return 0;
  }
  return ranked[(ranked.size() - 1) / 2];
}

// The members ranked above `current`, in ascending order: the cells the
// repair passes may upsize a gate to. An incumbent that cannot be ranked (its
// Liberty gives no leakage) is not in the ranking, and then every rankable
// member is returned.
std::vector<size_t> upsizeLadder(const std::vector<InitCandidate>& group,
                                 const size_t current)
{
  std::vector<size_t> ranked = rankedAscending(group);
  for (size_t i = 0; i < ranked.size(); ++i) {
    if (ranked[i] == current) {
      ranked.erase(ranked.begin(), ranked.begin() + i + 1);
      break;
    }
  }
  return ranked;
}

// random: the draw indexes a canonical order (ascending cell name), never the
// group vector as built. The group vector comes from
// Resizer::getSwappableCells: sta::equivCells sorted by drive resistance with
// an unstable sort, filtered by dont-use and the sizing limits, with the
// as-given cell prepended. Indexing it directly would make the result for a
// given seed depend on which cell happens to be instantiated, on a
// `set_dont_use` elsewhere in the script shifting every index, and on how the
// standard library breaks a sort tie. Liberty cell names are unique inside a
// group, so the name order is total. The population is still the whole group,
// the as-given cell included.
size_t canonicalDraw(const std::vector<InitCandidate>& group,
                     const uint64_t draw)
{
  std::vector<size_t> order(group.size());
  for (size_t i = 0; i < order.size(); ++i) {
    order[i] = i;
  }
  // The modulo bias is bounded by group_size / 2^64, negligible for any
  // Liberty equivalence group.
  const size_t k = static_cast<size_t>(draw % group.size());
  std::nth_element(order.begin(),
                   order.begin() + k,
                   order.end(),
                   [&group](const size_t a, const size_t b) {
                     return group[a].name < group[b].name;
                   });
  return order[k];
}

}  // namespace

size_t selectInitCandidate(const std::vector<InitCandidate>& group,
                           const InitMode mode,
                           const uint64_t draw)
{
  if (group.empty()) {
    return 0;
  }
  switch (mode) {
    case InitMode::kMinSize:
      return scanExtremum(group, /*want_min=*/true);
    case InitMode::kMaxSize:
      return scanExtremum(group, /*want_min=*/false);
    case InitMode::kAverage:
      return lowerMedian(group);
    case InitMode::kRandom:
      // Uniform over the whole group, the as-given cell included. This is also
      // the sweep's candidate set: the sweep prices a cell with no Liberty
      // leakage by scaled area (LRSubproblem::leakageOrArea) rather than
      // excluding it. The deterministic modes drop such a cell only because it
      // cannot be ranked, not because it is ineligible.
      return canonicalDraw(group, draw);
    case InitMode::kMinSizeFixviol:
    case InitMode::kMinSizeFixcap:
      // The selection step of min_size_fixviol and min_size_fixcap is
      // min_size; the electrical repair that follows is done by InitPass.
      // InitModePass::run maps the mode to min_size before calling here, so
      // this case is not reached. It still returns the min_size pick, not the
      // as-given cell, so a caller that passes the unmapped mode gets the
      // right selection.
      return scanExtremum(group, /*want_min=*/true);
    case InitMode::kAsGiven:
      // Never runs the pass at all.
      return 0;
  }
  return 0;
}

size_t selectFixviolUpsize(const std::vector<InitCandidate>& group,
                           const size_t current,
                           const std::function<bool(size_t)>& clears)
{
  for (const size_t member : upsizeLadder(group, current)) {
    if (clears(member)) {
      return member;
    }
  }
  return current;
}

size_t selectSlewFixUpsize(
    const std::vector<InitCandidate>& group,
    const size_t current,
    const float current_excess,
    const std::function<std::optional<float>(size_t)>& slew_excess)
{
  size_t best = current;
  float best_excess = current_excess;
  for (const size_t member : upsizeLadder(group, current)) {
    const std::optional<float> excess = slew_excess(member);
    if (!excess.has_value()) {
      continue;
    }
    if (*excess <= 0.0f) {
      return member;
    }
    // Strictly smaller, so a tie keeps the lower-ranked member.
    if (*excess < best_excess) {
      best = member;
      best_excess = *excess;
    }
  }
  return best;
}

uint64_t initDraw(const int init_seed, const std::string_view instance_name)
{
  // FNV-1a over the instance name, mixed with the seed and run through
  // splitmix64's finalizer. Any stateless (seed, name) -> uint64 hash with good
  // avalanche would do; what matters is that it reads no iteration state.
  uint64_t h = 1469598103934665603ULL;
  for (const char c : instance_name) {
    h ^= static_cast<uint8_t>(c);
    h *= 1099511628211ULL;
  }
  h ^= static_cast<uint64_t>(static_cast<uint32_t>(init_seed))
       * 0x9E3779B97F4A7C15ULL;
  h ^= h >> 30;
  h *= 0xBF58476D1CE4E5B9ULL;
  h ^= h >> 27;
  h *= 0x94D049BB133111EBULL;
  h ^= h >> 31;
  return h;
}

}  // namespace rsz
