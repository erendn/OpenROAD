// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "LrState.hh"

#include <algorithm>
#include <cstddef>
#include <vector>

#include "db_sta/dbSta.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "sta/Clock.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/Mode.hh"
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
  // Per (arc, transition): the from-arrival in the arc's input transition plus
  // that arc's delay is the arrival this pair propagates to to_v. Pick the pair
  // realizing the edge's own worst propagated arrival — that (a_from, d) is the
  // consistent read (a_from + d <= a_to by construction).
  std::vector<ArcTransitionRead> reads;
  reads.reserve(arc_set->arcs().size());
  for (sta::TimingArc* arc : arc_set->arcs()) {
    const sta::RiseFallBoth* from_rf = arc->fromEdge()->asRiseFallBoth();
    const float a_from
        = sta::delayAsFloat(sta->arrival(from_v, from_rf, sta->scenes(), max));
    const float d = sta::delayAsFloat(graph->arcDelay(edge, arc, dcalc_ap));
    reads.push_back({a_from, d});
  }
  const ArcTransitionRead crit = pickCriticalArcTransition(reads);
  out.a_from = crit.a_from;
  out.d = crit.d;
  return out;
}

void LrState::allocate()
{
  // Walk the graph once to discover max EdgeId (lambda is keyed by
  // sta::Edge::id, which is sparse - size to max_id + 1)
  sta::EdgeId max_edge_id = 0;
  int data_edge_count = 0;
  sta::VertexIterator vit(graph);
  while (vit.hasNext()) {
    sta::Vertex* v = vit.next();
    sta::VertexOutEdgeIterator eit(v, graph);
    while (eit.hasNext()) {
      sta::Edge* e = eit.next();
      if (!isDataArc(e)) {
        continue;
      }
      const sta::EdgeId id = graph->id(e);
      max_edge_id = std::max(id, max_edge_id);
      ++data_edge_count;
    }
  }

  const size_t n_edges = static_cast<size_t>(max_edge_id) + 1;
  lambda.assign(n_edges, 0.0f);

  // Per-run reset of the near-met phase latch (C2). The driver re-latches it
  // each iteration from the live WNS; clearing it here gives every run a fresh
  // "not near-met yet" start (state is reused across global_sizing calls).
  near_met = false;
  // Same per-run reset for the iteration index: the estimation loop's dry-run
  // sweeps run before the main loop writes it, and a value left over from a
  // previous global_sizing call would switch Fast-OLR on for them.
  iter = 0;

  // B2 cost-term per-edge stores: sized only when their flag is on so the
  // rsz_baseline path allocates nothing extra (M3).
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
             max_edge_id,
             endpoint_vertices.size(),
             dcalc_ap);
}

int LrState::growToLiveEdges()
{
  const size_t old_size = lambda.size();
  sta::EdgeId max_edge_id = 0;
  int reminted = 0;
  sta::VertexIterator vit(graph);
  while (vit.hasNext()) {
    sta::Vertex* v = vit.next();
    sta::VertexOutEdgeIterator eit(v, graph);
    while (eit.hasNext()) {
      sta::Edge* e = eit.next();
      if (!isDataArc(e)) {
        continue;
      }
      const sta::EdgeId id = graph->id(e);
      max_edge_id = std::max(id, max_edge_id);
      if (static_cast<size_t>(id) >= old_size) {
        ++reminted;
      }
    }
  }

  // Grow only (never shrink): a stale slot for a removed arc is harmless, but a
  // live arc without a slot is unpriced. New arcs get a neutral 0 so the next
  // projection redistributes real values over them (leaning on the equal-split
  // rescue in FlowProjection). phi/prev_delay are re-sized from lambda.size()
  // by their own per-iteration passes, so growing lambda is enough.
  const size_t needed = static_cast<size_t>(max_edge_id) + 1;
  if (needed > old_size) {
    lambda.resize(needed, 0.0f);
  }
  if (reminted > 0) {
    debugPrint(logger,
               RSZ,
               "global_sizing",
               2,
               "LR grow: {} re-minted data arc(s) now priced (lambda {} -> {})",
               reminted,
               old_size,
               lambda.size());
  }
  return reminted;
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

}  // namespace rsz
