// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "LrState.hh"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "db_sta/dbSta.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "rsz/Resizer.hh"
#include "sta/Clock.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/Liberty.hh"
#include "sta/Mode.hh"
#include "sta/Network.hh"
#include "sta/Sdc.hh"
#include "sta/Sta.hh"
#include "sta/TimingArc.hh"
#include "sta/TimingRole.hh"
#include "sta/Transition.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

bool LrState::isDataArc(const sta::Edge* edge) const
{
  const sta::TimingRole* role = edge->role();
  if (role != nullptr && role->isTimingCheck()) {
    return false;
  }
  if (edge->isDisabledLoop()) {
    return false;
  }
  if (role == sta::TimingRole::latchDtoQ()
      || role == sta::TimingRole::latchEnToQ()) {
    return false;
  }
  return true;
}

float LrState::edgeMaxArcDelay(sta::Edge* edge) const
{
  sta::TimingArcSet* arc_set = edge->timingArcSet();
  if (arc_set == nullptr) {
    return 0.0f;
  }
  float max_d = 0.0f;
  for (sta::TimingArc* arc : arc_set->arcs()) {
    const sta::ArcDelay d = graph->arcDelay(edge, arc, dcalc_ap);
    const float df = sta::delayAsFloat(d);
    max_d = std::max(df, max_d);
  }
  return max_d;
}

ArcTransitionRead pickCriticalArcTransition(
    const std::vector<ArcTransitionRead>& reads)
{
  ArcTransitionRead best;
  bool any = false;
  float best_prop = 0.0f;
  for (const ArcTransitionRead& r : reads) {
    const float prop = r.a_from + r.d;
    if (!any || prop > best_prop) {
      best = r;
      best_prop = prop;
      any = true;
    }
  }
  return best;  // {0, 0} when reads is empty
}

LrState::ConsistentArcRead LrState::consistentArcRead(sta::Edge* edge,
                                                      sta::Vertex* from_v,
                                                      sta::Vertex* to_v) const
{
  ConsistentArcRead out;
  // The vertex constraint value: to_v's worst rise/fall arrival (the max over
  // all in-edges' contributions, so every edge's a_from + d is <= this).
  out.a_to = sta::delayAsFloat(
      sta->arrival(to_v, sta::RiseFallBoth::riseFall(), sta->scenes(), max));

  sta::TimingArcSet* arc_set = edge->timingArcSet();
  if (arc_set == nullptr) {
    out.a_from = sta::delayAsFloat(sta->arrival(
        from_v, sta::RiseFallBoth::riseFall(), sta->scenes(), max));
    return out;
  }
  // For each (arc, transition), the from-arrival in the arc's input transition
  // plus the arc's delay is the arrival this pair propagates to to_v. The pair
  // that produces the edge's own worst propagated arrival is the consistent
  // read (a_from + d <= a_to by construction).
  std::vector<ArcTransitionRead> reads;
  reads.reserve(arc_set->arcs().size());
  for (sta::TimingArc* arc : arc_set->arcs()) {
    // Sta::arrival matches paths against the RiseFallBoth objects, which
    // Transition::asRiseFallBoth() does not return, so the transition is
    // converted through RiseFall. An arc whose input edge is neither rise nor
    // fall reads the worst of both.
    const sta::RiseFall* from_rf = arc->fromEdge()->asRiseFall();
    const sta::RiseFallBoth* from_rfb = from_rf != nullptr
                                            ? from_rf->asRiseFallBoth()
                                            : sta::RiseFallBoth::riseFall();
    const float a_from
        = sta::delayAsFloat(sta->arrival(from_v, from_rfb, sta->scenes(), max));
    const float d = sta::delayAsFloat(graph->arcDelay(edge, arc, dcalc_ap));
    reads.push_back({a_from, d});
  }
  const ArcTransitionRead crit = pickCriticalArcTransition(reads);
  out.a_from = crit.a_from;
  out.d = crit.d;
  return out;
}

float LrState::arcSlack(sta::Edge* edge,
                        sta::Vertex* from_v,
                        sta::Vertex* to_v) const
{
  // Sta::arrival and Sta::required match paths against the RiseFallBoth
  // objects, so each transition is converted through RiseFall, as in
  // consistentArcRead.
  const auto both = [](const sta::Transition* tr) {
    const sta::RiseFall* rf = tr->asRiseFall();
    return rf != nullptr ? rf->asRiseFallBoth() : sta::RiseFallBoth::riseFall();
  };
  float worst = kSlackSentinel;
  sta::TimingArcSet* arc_set = edge->timingArcSet();
  if (arc_set == nullptr) {
    return worst;
  }
  for (sta::TimingArc* arc : arc_set->arcs()) {
    const float a_from = sta::delayAsFloat(
        sta->arrival(from_v, both(arc->fromEdge()), sta->scenes(), max));
    const float q_to = sta::delayAsFloat(
        sta->required(to_v, both(arc->toEdge()), sta->scenes(), max));
    if (std::fabs(a_from) >= kSlackSentinel
        || std::fabs(q_to) >= kSlackSentinel) {
      continue;
    }
    const float d = sta::delayAsFloat(graph->arcDelay(edge, arc, dcalc_ap));
    worst = std::min(worst, q_to - a_from - d);
  }
  return worst;
}

namespace {

// The ends of every live data edge, indexed by edge id; empty for an id no
// live data edge holds. Sized to the largest data-edge id + 1.
std::vector<EdgeEnds> liveEdgeEnds(const LrState& state, size_t size_hint)
{
  sta::Graph* graph = state.graph;
  std::vector<EdgeEnds> ends;
  ends.reserve(size_hint);
  sta::VertexIterator vit(graph);
  while (vit.hasNext()) {
    sta::Vertex* v = vit.next();
    const sta::VertexId from = graph->id(v);
    sta::VertexOutEdgeIterator eit(v, graph);
    while (eit.hasNext()) {
      sta::Edge* e = eit.next();
      if (!state.isDataArc(e)) {
        continue;
      }
      const size_t id = graph->id(e);
      if (id >= ends.size()) {
        ends.resize(id + 1);
      }
      ends[id] = {.from = from, .to = graph->id(e->to(graph))};
    }
  }
  return ends;
}

uint64_t pinPairKey(const EdgeEnds& ends)
{
  return (static_cast<uint64_t>(ends.from) << 32) | ends.to;
}

// One carried pin pair: the multipliers in its previous slots, summed, and
// its live edges.
struct PinPairCarry
{
  float sum = 0.0f;
  bool has_history = false;
  int live_edges = 0;
};

using PinPairCarries = std::unordered_map<uint64_t, PinPairCarry>;

// The carry entry of the pin pair `ends`, or nullptr when that pair is not
// carried or the slot holds no edge. Takes the map const or mutable.
template <typename Carries>
auto findCarry(Carries& carries,
               const EdgeEnds& ends) -> decltype(&carries.begin()->second)
{
  if (ends.empty()) {
    return nullptr;
  }
  const auto it = carries.find(pinPairKey(ends));
  return it == carries.end() ? nullptr : &it->second;
}

void addCarry(PinPairCarries& carries, const EdgeEnds& ends)
{
  if (!ends.empty()) {
    carries.try_emplace(pinPairKey(ends));
  }
}

// Pass 1, detection only: the pin pairs whose slots changed. Nothing is
// written yet, because a recycled slot still holds its previous pair's
// multiplier, which pass 2 adds to that pair's sum.
PinPairCarries findCarriedPairs(const std::vector<EdgeEnds>& slot_ends,
                                const std::vector<EdgeEnds>& live_ends)
{
  PinPairCarries carries;
  for (size_t slot = 0; slot < live_ends.size(); ++slot) {
    if (slot_ends[slot] == live_ends[slot]) {
      continue;
    }
    addCarry(carries, slot_ends[slot]);
    addCarry(carries, live_ends[slot]);
  }
  return carries;
}

// Pass 2a: sums each carried pair's previous slots and counts its live edges.
void tallyCarriedPairs(const std::vector<EdgeEnds>& slot_ends,
                       const std::vector<EdgeEnds>& live_ends,
                       const std::vector<float>& lambda,
                       PinPairCarries& carries)
{
  for (size_t slot = 0; slot < live_ends.size(); ++slot) {
    PinPairCarry* previous_pair = findCarry(carries, slot_ends[slot]);
    if (previous_pair != nullptr) {
      previous_pair->sum += lambda[slot];
      previous_pair->has_history = true;
    }
    PinPairCarry* live_pair = findCarry(carries, live_ends[slot]);
    if (live_pair != nullptr) {
      ++live_pair->live_edges;
    }
  }
}

// Pass 2b: writes each carried pair's even share into its live edges, and 0
// into the slots no live edge holds.
EdgeCarryStats writeCarriedShares(const std::vector<EdgeEnds>& live_ends,
                                  const PinPairCarries& carries,
                                  std::vector<float>& lambda)
{
  EdgeCarryStats stats;
  for (size_t slot = 0; slot < live_ends.size(); ++slot) {
    if (live_ends[slot].empty()) {
      lambda[slot] = 0.0f;
      continue;
    }
    const PinPairCarry* pair = findCarry(carries, live_ends[slot]);
    if (pair == nullptr) {
      continue;
    }
    if (pair->has_history) {
      lambda[slot] = pair->sum / static_cast<float>(pair->live_edges);
      ++stats.carried;
    } else {
      lambda[slot] = 0.0f;
      ++stats.started_at_zero;
    }
  }
  return stats;
}

}  // namespace

EdgeCarryStats carryMultipliers(std::vector<EdgeEnds> live_ends,
                                std::vector<EdgeEnds>& slot_ends,
                                std::vector<float>& lambda)
{
  const size_t size = std::max(lambda.size(), live_ends.size());
  lambda.resize(size, 0.0f);
  live_ends.resize(size);
  slot_ends.resize(size);

  PinPairCarries carries = findCarriedPairs(slot_ends, live_ends);
  EdgeCarryStats stats;
  if (!carries.empty()) {
    tallyCarriedPairs(slot_ends, live_ends, lambda, carries);
    stats = writeCarriedShares(live_ends, carries, lambda);
  }
  slot_ends = std::move(live_ends);
  return stats;
}

void LrState::allocate()
{
  // lambda is indexed by edge id, which is sparse, so it is sized to the
  // largest data-edge id + 1.
  lambda_ends = liveEdgeEnds(*this, 0);
  const size_t n_edges = std::max<size_t>(lambda_ends.size(), 1);
  lambda_ends.resize(n_edges);
  lambda.assign(n_edges, 0.0f);
  edge_carry = {};
  const auto data_edge_count = std::ranges::count_if(
      lambda_ends, [](const EdgeEnds& ends) { return !ends.empty(); });

  // Clear the near-met latch for each run. The driver sets it again each
  // iteration from the current WNS; clearing it here starts every run as not
  // near-met (the state is reused across global_sizing calls).
  near_met = false;
  // Every run starts in the timing phase.
  power_phase = false;
  prev_sweep_power_phase = false;
  // Reset the iteration index too: the estimation loop's dry-run sweeps run
  // before the main loop writes it, and a value left over from a previous
  // global_sizing call would switch Fast-OLR on for them.
  iter = 0;
  cap_multipliers = {};

  // Per-edge cost-term stores, sized only when their option is enabled so the
  // default configuration allocates nothing extra.
  phi.assign(config->cost_global_phi ? n_edges : 0, 0.0f);
  prev_delay.assign(config->cost_delta_delay ? n_edges : 0, 0.0f);

  // Endpoint bookkeeping
  endpoint_vertices.clear();
  endpoint_index.clear();
  const sta::VertexSet& eps = sta->endpoints();
  endpoint_vertices.reserve(eps.size());
  endpoint_index.reserve(eps.size());
  for (sta::Vertex* v : eps) {
    endpoint_index.emplace(v, static_cast<int>(endpoint_vertices.size()));
    endpoint_vertices.push_back(v);
  }
  mu.assign(endpoint_vertices.size(), 0.0f);

  debugPrint(logger,
             RSZ,
             "global_sizing",
             2,
             "LR allocate: edges={} (max_id={}), endpoints={}, dcalc_ap={}",
             data_edge_count,
             n_edges - 1,
             endpoint_vertices.size(),
             dcalc_ap);
}

EdgeCarryStats LrState::refreshLiveEdges()
{
  // phi and prev_delay are resized from lambda.size() by their own passes
  // before each sweep, so growing lambda is enough.
  const size_t old_size = lambda.size();
  const EdgeCarryStats stats
      = carryMultipliers(liveEdgeEnds(*this, old_size), lambda_ends, lambda);
  edge_carry += stats;
  if (stats.any()) {
    debugPrint(logger,
               RSZ,
               "global_sizing",
               2,
               "LR refresh: {} re-created edge(s) took their pins' earlier "
               "multipliers, {} started at 0 (lambda {} -> {})",
               stats.carried,
               stats.started_at_zero,
               old_size,
               lambda.size());
  }
  return stats;
}

void LrState::captureInitialTiming()
{
  // Clock period T = max period over all SDC clocks (the design's dominant
  // period). Single-clock designs get exactly that clock's period; multi-clock
  // handling is per-arc via slack_init below, so a scalar T is only a
  // normalizer and the max is a safe choice. 0 when no clock is defined.
  T = 0.0f;
  for (const sta::Clock* clk : sta->cmdMode()->sdc()->clocks()) {
    T = std::max(T, clk->period());
  }

  wns_init = sta::delayAsFloat(sta->worstSlack(max));

  // Per-vertex initial slack, indexed by sta::Graph vertex id (same scheme as
  // vertex_budget). Unconstrained vertices report the sentinel.
  size_t max_id = 0;
  {
    sta::VertexIterator vit(graph);
    while (vit.hasNext()) {
      max_id = std::max(max_id, static_cast<size_t>(graph->id(vit.next())));
    }
  }
  slack_init.assign(max_id + 1, kSlackSentinel);
  sta::VertexIterator vit(graph);
  while (vit.hasNext()) {
    sta::Vertex* v = vit.next();
    slack_init[graph->id(v)] = sta::delayAsFloat(sta->slack(v, max));
  }

  debugPrint(logger,
             RSZ,
             "global_sizing",
             2,
             "LR capture: T={:.3g} wns_init={:.3g} vertices={}",
             T,
             wns_init,
             slack_init.size());
}

void CellAssignment::capture(const LrState& state)
{
  sta::Network* network = state.network;
  cells_.clear();
  std::unique_ptr<sta::LeafInstanceIterator> iit(
      network->leafInstanceIterator());
  while (iit->hasNext()) {
    sta::Instance* inst = iit->next();
    sta::LibertyCell* cell = network->libertyCell(inst);
    if (cell == nullptr || state.resizer->dontTouch(inst)) {
      continue;
    }
    cells_.emplace_back(inst, cell);
  }
}

int CellAssignment::restore(LrState& state) const
{
  sta::Network* network = state.network;
  int replaced = 0;
  // Serial, and only where the live cell has drifted from the record, so an
  // unchanged netlist costs nothing.
  for (const auto& [inst, cell] : cells_) {
    const sta::LibertyCell* current = network->libertyCell(inst);
    if (current == cell) {
      continue;
    }
    if (state.resizer->replaceCell(inst, cell, /*journal=*/true)) {
      ++replaced;
      debugPrint(state.logger,
                 RSZ,
                 "global_sizing_restore",
                 1,
                 "restore {} {} -> {}",
                 network->pathName(inst),
                 current->name(),
                 cell->name());
    }
  }
  return replaced;
}

}  // namespace rsz
