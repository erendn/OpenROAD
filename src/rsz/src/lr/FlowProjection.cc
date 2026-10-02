// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "FlowProjection.hh"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <vector>

#include "rsz/GlobalSizingConfig.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

namespace {

// Rescales a vertex's in-arc multipliers so they sum to `target` with each at
// least `floor`. A plain max(lambda * scale, floor) leaves the sum above the
// target whenever an arc is clamped up to the floor, which breaks flow
// conservation. Instead this water-fills: scale the unclamped arcs to carry the
// remainder (target minus floor * n_clamped), clamp any that fall below the
// floor, and repeat. Each pass clamps at least one more arc or finishes, so
// there are at most in_count passes. The sum equals the target exactly when the
// target is feasible (target >= floor * in_count). Otherwise every arc is set
// to the floor and the sum is floor * in_count > target: keeping multipliers
// off zero, where a multiplicative update could never move them again, takes
// priority over exact conservation.
//
// `in_sum` is the sum of the in-arcs before rescaling; the caller guarantees it
// is > 0. Returns true when an arc was clamped.
bool rescaleInArcsWithFloor(const std::vector<int>& in_edges,
                            const int begin,
                            const int end,
                            const float in_sum,
                            const float target,
                            const float floor,
                            std::vector<float>& lambda)
{
  const float scale = target / in_sum;
  // Fast path: nothing clamps (the usual case with the default floor of 1e-12),
  // so one proportional scale is exact. Check before writing anything.
  bool any_below = false;
  for (int i = begin; i < end; ++i) {
    if (lambda[in_edges[i]] * scale < floor) {
      any_below = true;
      break;
    }
  }
  if (!any_below) {
    for (int i = begin; i < end; ++i) {
      lambda[in_edges[i]] *= scale;
    }
    return false;
  }

  // Slow path (an arc would clamp up to the floor): water-fill on the original
  // weights, which are still intact because the fast path wrote nothing.
  const int count = end - begin;
  std::vector<float> weight(count);
  std::vector<char> clamped(count, 0);
  for (int j = 0; j < count; ++j) {
    weight[j] = lambda[in_edges[begin + j]];
  }
  int n_clamped = 0;
  for (;;) {
    float unclamped_weight = 0.0f;
    for (int j = 0; j < count; ++j) {
      if (!clamped[j]) {
        unclamped_weight += weight[j];
      }
    }
    const float residual = target - floor * static_cast<float>(n_clamped);
    if (unclamped_weight <= 0.0f || residual <= 0.0f) {
      // Infeasible target: nothing is left for the unclamped arcs. Clamp every
      // remaining arc to the floor (sum = floor * count > target). Otherwise an
      // arc with a large lambda that has not yet dropped below the floor would
      // keep its value from before the projection.
      for (int j = 0; j < count; ++j) {
        if (!clamped[j]) {
          clamped[j] = 1;
          ++n_clamped;
        }
      }
      break;
    }
    const float s = residual / unclamped_weight;
    bool new_clamp = false;
    for (int j = 0; j < count; ++j) {
      if (!clamped[j] && weight[j] * s < floor) {
        clamped[j] = 1;
        ++n_clamped;
        new_clamp = true;
      }
    }
    if (!new_clamp) {
      // No arc fell below the floor at this scale: assign and finish. The
      // unclamped arcs carry `residual` and the clamped ones floor * n_clamped,
      // so the sum equals the target exactly.
      for (int j = 0; j < count; ++j) {
        if (!clamped[j]) {
          lambda[in_edges[begin + j]] = weight[j] * s;
        }
      }
      break;
    }
  }
  for (int j = 0; j < count; ++j) {
    if (clamped[j]) {
      lambda[in_edges[begin + j]] = floor;
    }
  }
  return true;
}

}  // namespace

ProjectionStats projectFlowBalance(const ProjectionTopology& topo,
                                   const GlobalSizingConfig::MuPolicy mu_policy,
                                   const bool derive_endpoint_mu,
                                   const float floor,
                                   std::vector<float>& mu,
                                   std::vector<float>& lambda)
{
  // Derive mu from the endpoint's own in-arc lambda sum, which makes the
  // rescale at the endpoint a no-op and the endpoint arcs the boundary
  // condition. This is always done for endpoint_lambda, which has no separate
  // mu update. For endpoint_ratio and endpoint_additive it is done on the first
  // projection of a run only; they update mu themselves afterwards, and later
  // projections anchor to it.
  using MP = GlobalSizingConfig::MuPolicy;
  const bool derive_mu = mu_policy == MP::kEndpointLambda
                         || (derive_endpoint_mu
                             && (mu_policy == MP::kEndpointRatio
                                 || mu_policy == MP::kEndpointAdditive));

  ProjectionStats stats;
  for (const ProjectionTopology::Vertex& v : topo.vertices) {
    const int in_count = v.in_end - v.in_begin;
    if (in_count == 0) {
      continue;
    }

    // An internal vertex none of whose data out-arcs has a multiplier (all of
    // them were created after the multipliers were sized, or it is a dangling
    // sink) has no downstream demand to conserve. Its target would be 0, and
    // the rescale would drive its in-arcs to the floor, wiping out real lambda
    // and cascading upstream. Leave its in-arcs unchanged. This is rare once
    // LrState::refreshLiveEdges() has run for the iteration.
    if (v.endpoint < 0 && v.out_begin == v.out_end) {
      continue;
    }

    float in_sum = 0.0f;
    for (int i = v.in_begin; i < v.in_end; ++i) {
      in_sum += lambda[topo.in_edges[i]];
    }

    // Target flow into v: its out-flow for an internal vertex (KKT flow
    // conservation), or the boundary condition for an endpoint.
    float target = 0.0f;
    if (v.endpoint >= 0) {
      if (derive_mu) {
        // The endpoint's own arcs are the boundary. Record the multiplier they
        // add up to, so mu stays the true endpoint multiplier for the
        // duality-gap diagnostic (and is the starting mu for endpoint_ratio and
        // endpoint_additive).
        target = in_sum;
        mu[v.endpoint] = in_sum;
      } else {
        target = mu[v.endpoint];
      }
    } else {
      for (int i = v.out_begin; i < v.out_end; ++i) {
        target += lambda[topo.out_edges[i]];
      }
    }

    if (in_sum > 0.0f) {
      if (rescaleInArcsWithFloor(topo.in_edges,
                                 v.in_begin,
                                 v.in_end,
                                 in_sum,
                                 target,
                                 floor,
                                 lambda)) {
        ++stats.floor_redistributed;
      }
      ++stats.rescaled;
    } else if (target > 0.0f) {
      const float share = target / static_cast<float>(in_count);
      for (int i = v.in_begin; i < v.in_end; ++i) {
        lambda[topo.in_edges[i]] = std::max(share, floor);
      }
      ++stats.zero_sum_fallback;
    }
  }
  return stats;
}

namespace {

// Fills `topo` from the timing graph: vertices in descending level order
// (endpoints before their fanin), each with its data-arc multiplier indices.
// The order within a level does not matter: a levelized graph has no edge
// between two vertices of the same level, so no vertex reads a multiplier that
// another vertex of its level writes.
void buildTopology(LrState& state, ProjectionTopology& topo)
{
  sta::Graph* graph = state.graph;
  const size_t lambda_size = state.lambda.size();

  std::vector<sta::Vertex*> vertices;
  {
    sta::VertexIterator vit(graph);
    while (vit.hasNext()) {
      vertices.push_back(vit.next());
    }
  }
  std::ranges::sort(vertices, [](const sta::Vertex* a, const sta::Vertex* b) {
    return a->level() > b->level();
  });

  // lambda is sized to the edge ids seen by allocate(). A sweep can replace
  // cells, and the following updateParasitics()/findRequireds() rebuild arcs
  // with new edge ids that may be >= lambda.size(). Such arcs have no
  // multiplier; skip them, as the updaters do.
  const auto multiplierIndex
      = [&state, graph, lambda_size](sta::Edge* e) -> int {
    if (!state.isDataArc(e)) {
      return -1;
    }
    const sta::EdgeId id = graph->id(e);
    return static_cast<size_t>(id) >= lambda_size ? -1 : static_cast<int>(id);
  };

  topo.vertices.clear();
  topo.in_edges.clear();
  topo.out_edges.clear();
  topo.vertices.reserve(vertices.size());
  for (sta::Vertex* v : vertices) {
    ProjectionTopology::Vertex tv;

    tv.in_begin = static_cast<int>(topo.in_edges.size());
    sta::VertexInEdgeIterator ieit(v, graph);
    while (ieit.hasNext()) {
      const int id = multiplierIndex(ieit.next());
      if (id >= 0) {
        topo.in_edges.push_back(id);
      }
    }
    tv.in_end = static_cast<int>(topo.in_edges.size());

    const auto ep_it = state.endpoint_index.find(v);
    if (ep_it != state.endpoint_index.end()) {
      tv.endpoint = ep_it->second;
    } else {
      tv.out_begin = static_cast<int>(topo.out_edges.size());
      sta::VertexOutEdgeIterator oeit(v, graph);
      while (oeit.hasNext()) {
        const int id = multiplierIndex(oeit.next());
        if (id >= 0) {
          topo.out_edges.push_back(id);
        }
      }
      tv.out_end = static_cast<int>(topo.out_edges.size());
    }

    topo.vertices.push_back(tv);
  }
}

}  // namespace

void ProportionalReverseTopoProjection::project(LrState& state,
                                                const bool first_projection)
{
  const GlobalSizingConfig& params = *state.config;

  buildTopology(state, topo_);
  const ProjectionStats stats = projectFlowBalance(topo_,
                                                   params.mu_policy,
                                                   first_projection,
                                                   params.lambda_floor,
                                                   state.mu,
                                                   state.lambda);

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR project: {} vertices rescaled ({} zero-sum fallbacks, {} "
             "floor-redistributed)",
             stats.rescaled,
             stats.zero_sum_fallback,
             stats.floor_redistributed);
}

std::unique_ptr<FlowProjection> makeFlowProjection(
    const GlobalSizingConfig& /* config */)
{
  // proportional_reverse_topo is the only KktProjection option, so config is
  // not read.
  return std::make_unique<ProportionalReverseTopoProjection>();
}

}  // namespace rsz
