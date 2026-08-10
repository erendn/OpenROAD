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

// Floor-aware exact rescale of a vertex's in-arc multipliers to sum to `target`
// with each >= floor (C3 item 2). The plain max(λ·scale, floor) rescale leaves
// Σ_in > target whenever an arc clamps UP to the floor, so post-projection λ is
// only approximately in Ω_λ — a conservation break every audit flagged as
// "stated nowhere". This water-fills instead: rescale the still-unclamped arcs
// to carry the residual (target minus the pinned floor·n_clamped), clamp any
// that fall below the floor, and repeat. Each pass clamps at least one new arc
// or finishes, so it runs at most in_count passes. Σ_in = target EXACTLY when
// the target is feasible (target >= floor·in_count); when it is not, every arc
// floors and Σ_in = floor·in_count > target — the documented infeasible
// fallback, where the floor's zero-absorption guard (why the floor exists at
// all) necessarily wins over exact conservation.
//
// `in_sum` (> 0, guaranteed by the caller) is the pre-rescale Σ of the in-arcs.
// Returns true when it took the redistribution slow path (an arc clamped).
bool rescaleInArcsWithFloor(const std::vector<int>& in_edges,
                            const int begin,
                            const int end,
                            const float in_sum,
                            const float target,
                            const float floor,
                            std::vector<float>& lambda)
{
  const float scale = target / in_sum;
  // Fast path: nothing clamps (the overwhelmingly common case at floor=1e-12),
  // so a single proportional scale is already exact. Detect without writing.
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
  // weights, which are still intact since the fast path did not write.
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
      // Infeasible target: no positive budget left for the unclamped arcs. Clamp
      // EVERY remaining arc so they all floor below (Σ_in = floor·count >
      // target); documented above. Without this, a large-λ arc that has not yet
      // fallen below the floor would keep its stale pre-projection value.
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
      // unclamped arcs carry `residual`, the clamped ones `floor·n_clamped`, so
      // Σ_in = target exactly.
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
  // Derive mu from the endpoint's own in-arc lambda sum (a no-op rescale that
  // makes the endpoint arcs the boundary) when the policy has no separate mu
  // writer (endpoint_lambda, always), or on the first projection of a run for
  // the endpoint-pressure policies (endpoint_ratio / endpoint_additive), which
  // maintain mu themselves thereafter and are anchored to on later projections.
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

    // C3 item 3c: an internal (non-endpoint) vertex whose data out-arcs are all
    // absent from the topology — every out-arc newly minted past the multiplier
    // space (the backward zero-cascade), or a dangling sink with none — has no
    // real downstream demand to conserve. Its target would be 0, which the
    // rescale below would drive its in-arcs to the floor, zeroing real λ and
    // cascading a blackout upstream (tennakoon audit §5.4). Leave its in-arcs
    // untouched. (After growToLiveEdges runs each iteration this is rare, but
    // it is the invariant the audits pinned.)
    if (v.endpoint < 0 && v.out_begin == v.out_end) {
      continue;
    }

    float in_sum = 0.0f;
    for (int i = v.in_begin; i < v.in_end; ++i) {
      in_sum += lambda[topo.in_edges[i]];
    }

    // Target flow into v: its own out-flow (KKT conservation) for an internal
    // vertex, the endpoint boundary condition for an endpoint.
    float target = 0.0f;
    if (v.endpoint >= 0) {
      if (derive_mu) {
        // The endpoint's own arcs are the boundary; record the multiplier they
        // amount to so mu stays the true endpoint multiplier for the
        // duality-gap diagnostic (and the seed for the endpoint-pressure
        // policies' first iteration).
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

// Fill `topo` from the timing graph: vertices in descending level order
// (endpoints before their predecessors), each with its data-arc multiplier
// indices. Within one level the order is immaterial - a levelized graph has no
// edge between two vertices of the same level, so no vertex reads a multiplier
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

  // lambda is sized to the edge-id space captured in allocate(). A sweep can
  // replace cells and the subsequent updateParasitics()/findRequireds() rebuild
  // arcs, minting edge ids beyond that space, so an id may now be >=
  // lambda.size(). Such arcs carry no multiplier; skip them, matching the guard
  // in the updater.
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
  // E3 axis has a single strategy in M0; the `none` (free multipliers) option
  // keyed on config.kkt_projection is added later.
  return std::make_unique<ProportionalReverseTopoProjection>();
}

}  // namespace rsz
