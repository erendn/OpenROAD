// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "SizeVthGrid.hh"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

namespace rsz {

namespace {

// A non-finite rank key would make the comparators below a non-transitive
// ordering (NaN compares false in both directions while everything else stays
// ordered), and an invalid strict weak ordering is undefined behavior inside
// std::sort - a crash in the middle of repair_timing, not a mis-ranked cell.
// InitSelect drops such a member from its ranking; here every member must get a
// coord, so the key is pinned to 0 instead and the member ranks with the
// cheapest of its flavor, tie-broken by name. leakageOrArea is finite for every
// cell of every library we have measured; this is the guard, not the case.
float finiteKey(const float key)
{
  return std::isfinite(key) ? key : 0.0f;
}

// One walk of Fig. 9 lines 5-16: seed at `start`, then step by `step` while the
// cost strictly descends. Returns the winning candidate and the cost the walk
// already paid for it, or kNoGridCandidate when the walk found no selectable
// improvement on its seed (which for the incumbent's own flavor means "keep the
// current cell").
//
// `seed` is the candidate at the starting rank (kNoGridCandidate at the
// incumbent's own hole) and `seed_cost` the cost the descent measures against -
// the seed's own cost when it is selectable, the incumbent's otherwise (header
// reading 3). `seed_selectable` is the caller's already-computed validity of
// the seed, hoisted so the two directions do not re-test it.
FastOlrWinner descendColumn(const GridLayout& layout,
                            const int flavor,
                            const int start,
                            const int step,
                            const int seed,
                            const bool seed_selectable,
                            const float seed_cost,
                            const std::function<bool(int)>& valid,
                            const std::function<float(int)>& cost)
{
  // Fig. 9 lines 6-7: the seed is bestcell, when it is one at all.
  FastOlrWinner best;
  if (seed_selectable) {
    best = {.candidate = seed, .cost = seed_cost};
  }
  float best_cost = seed_cost;
  const int end = layout.columnHeight(flavor);
  for (int rank = start + step; rank >= 0 && rank < end; rank += step) {
    const int member = layout.at(flavor, rank);
    if (member == kNoGridCandidate) {
      // The incumbent's own slot: nothing to evaluate, and it is not a hole in
      // the descent - the cell is simply the one we started from.
      continue;
    }
    // Fig. 9 line 10, read as a guard clause: an invalid cell is skipped, so it
    // neither moves bestcost nor ends the walk (header reading 2).
    if (!valid(member)) {
      continue;
    }
    const float member_cost = cost(member);
    if (member_cost < best_cost) {
      best_cost = member_cost;
      best = {.candidate = member, .cost = member_cost};
      continue;
    }
    // Fig. 9 line 14.
    break;
  }
  return best;
}

}  // namespace

int GridLayout::columnHeight(const int flavor) const
{
  if (flavor < 0 || flavor >= flavorCount()) {
    return 0;
  }
  return column_begin[flavor + 1] - column_begin[flavor];
}

int GridLayout::at(const int flavor, const int width_rank) const
{
  if (width_rank < 0 || width_rank >= columnHeight(flavor)) {
    return kNoGridCandidate;
  }
  return slot[column_begin[flavor] + width_rank];
}

std::vector<GridCoord> buildSizeVthGrid(const std::vector<GridMember>& group)
{
  std::vector<GridCoord> coords(group.size());
  if (group.empty()) {
    return coords;
  }

  // Flavor identity -> (sum of rank keys, member count), so the flavors can be
  // ordered by their mean key. std::vector rather than a map: a swappable group
  // holds a handful of Vth flavors at most, and a linear scan keeps the result
  // independent of any hash order.
  struct Flavor
  {
    int vth_key = 0;
    double key_sum = 0.0;
    int count = 0;
  };
  std::vector<Flavor> flavors;
  for (const GridMember& member : group) {
    auto it = std::find_if(
        flavors.begin(), flavors.end(), [&member](const Flavor& flavor) {
          return flavor.vth_key == member.vth_key;
        });
    if (it == flavors.end()) {
      flavors.push_back({.vth_key = member.vth_key});
      it = flavors.end() - 1;
    }
    it->key_sum += finiteKey(member.rank_key);
    ++it->count;
  }
  // Ascending mean rank key = the least-leaky (highest-Vth) flavor first, the
  // same convention Resizer::LibraryAnalysisData::sort_vt_categories uses to
  // order HVT/RVT/LVT/uLVT. vth_key breaks a tie so the order never depends on
  // the group vector's own order.
  std::sort(
      flavors.begin(), flavors.end(), [](const Flavor& a, const Flavor& b) {
        const double mean_a = a.key_sum / a.count;
        const double mean_b = b.key_sum / b.count;
        if (mean_a != mean_b) {
          return mean_a < mean_b;
        }
        return a.vth_key < b.vth_key;
      });

  // Members of each flavor, ranked ascending by (rank key, name).
  for (int flavor = 0; flavor < static_cast<int>(flavors.size()); ++flavor) {
    std::vector<size_t> members;
    for (size_t i = 0; i < group.size(); ++i) {
      if (group[i].vth_key == flavors[flavor].vth_key) {
        members.push_back(i);
      }
    }
    std::sort(members.begin(), members.end(), [&group](size_t a, size_t b) {
      const float key_a = finiteKey(group[a].rank_key);
      const float key_b = finiteKey(group[b].rank_key);
      if (key_a != key_b) {
        return key_a < key_b;
      }
      return group[a].name < group[b].name;
    });
    for (int rank = 0; rank < static_cast<int>(members.size()); ++rank) {
      coords[members[rank]] = {.flavor = flavor, .width_rank = rank};
    }
  }
  return coords;
}

GridLayout buildGridLayout(const std::vector<GridCoord>& coords,
                           const GridCoord cur)
{
  GridLayout layout;
  // The incumbent's own flavor and rank must have a slot even though it is not
  // a candidate, so the column heights count it.
  int flavor_count = cur.flavor + 1;
  for (const GridCoord& coord : coords) {
    flavor_count = std::max(flavor_count, coord.flavor + 1);
  }
  std::vector<int> heights(flavor_count, 0);
  heights[cur.flavor] = cur.width_rank + 1;
  for (const GridCoord& coord : coords) {
    heights[coord.flavor]
        = std::max(heights[coord.flavor], coord.width_rank + 1);
  }

  layout.column_begin.resize(flavor_count + 1, 0);
  for (int flavor = 0; flavor < flavor_count; ++flavor) {
    layout.column_begin[flavor + 1]
        = layout.column_begin[flavor] + heights[flavor];
  }
  layout.slot.assign(layout.column_begin.back(), kNoGridCandidate);
  for (int i = 0; i < static_cast<int>(coords.size()); ++i) {
    layout.slot[layout.column_begin[coords[i].flavor] + coords[i].width_rank]
        = i;
  }
  return layout;
}

std::vector<FastOlrWinner> fastOlrCandidates(
    const GridLayout& layout,
    const GridCoord cur,
    const float cur_cost,
    const std::function<bool(int)>& valid,
    const std::function<float(int)>& cost)
{
  std::vector<FastOlrWinner> candidates;
  if (layout.slot.empty()) {
    return candidates;
  }

  // Fig. 9 line 4: the current Vth flavor and its two neighbours.
  for (int flavor = cur.flavor - 1; flavor <= cur.flavor + 1; ++flavor) {
    const int height = layout.columnHeight(flavor);
    if (height == 0) {
      continue;
    }
    // Fig. 9 line 5, with the ragged-grid clamp (header reading 4): a flavor
    // shorter than the incumbent's width starts from its topmost member.
    const int start = std::min(cur.width_rank, height - 1);
    const int seed = layout.at(flavor, start);
    // The seed of the incumbent's own flavor IS the incumbent (it is the hole
    // in that column). A seed that is absent or DRC-rejected does not set the
    // walk's threshold - the descent then measures against the incumbent
    // (header reading 3).
    const bool seed_selectable = (seed != kNoGridCandidate) && valid(seed);
    const float seed_cost = seed_selectable ? cost(seed) : cur_cost;
    // Fig. 9 lines 8-16 ascending, then lines 17-18: "repeat lines 5-16 and
    // replace maxw by minw" - the descending walk RE-seeds from the same cell
    // and inserts its own winner, so a flavor contributes up to two candidates.
    for (const int step : {+1, -1}) {
      const FastOlrWinner winner = descendColumn(layout,
                                                 flavor,
                                                 start,
                                                 step,
                                                 seed,
                                                 seed_selectable,
                                                 seed_cost,
                                                 valid,
                                                 cost);
      // `candidates` is a SET in the paper; both directions return the seed
      // when neither improves on it.
      if (winner.candidate != kNoGridCandidate
          && std::find_if(candidates.begin(),
                          candidates.end(),
                          [&winner](const FastOlrWinner& seen) {
                            return seen.candidate == winner.candidate;
                          })
                 == candidates.end()) {
        candidates.push_back(winner);
      }
    }
  }
  return candidates;
}

bool withinSizeStep(const GridLayout& layout,
                    const GridCoord cur,
                    const GridCoord cand)
{
  // Clamp the incumbent's rank into the candidate's own column before banding,
  // so a flavor shallower than the incumbent's rank stays reachable and "Vth
  // swaps remain unrestricted" survives a ragged library (see the header).
  const int height = layout.columnHeight(cand.flavor);
  if (height == 0) {
    return false;
  }
  const int reference = std::min(cur.width_rank, height - 1);
  return std::abs(cand.width_rank - reference) <= 1;
}

}  // namespace rsz
