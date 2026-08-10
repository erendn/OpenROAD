// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>
#include <vector>

#include "LrState.hh"
#include "rsz/GlobalSizingConfig.hh"

namespace rsz {

// The projection's STA-free view of the timing graph, built from the sta::Graph
// by project() and consumed by projectFlowBalance. Vertices are stored in visit
// order (descending level - endpoints before their predecessors); each owns a
// contiguous range of in- and out-arc multiplier indices into state.lambda.
// Arcs that carry no multiplier (non-data arcs, and ids minted past the vector
// after a cell replacement rebuilt the graph) are simply absent.
//
// Same device as M4's TraversalEntry: the arithmetic that decides the answer is
// a pure function over a hand-buildable structure, so it is unit-testable
// without STA, and the STA walk is reduced to filling it in.
struct ProjectionTopology
{
  struct Vertex
  {
    int in_begin = 0;
    int in_end = 0;
    int out_begin = 0;
    int out_end = 0;
    // Index into mu when this vertex is a timing endpoint, else -1.
    int endpoint = -1;
  };
  std::vector<Vertex> vertices;
  // lambda indices, ranged into by Vertex::in_begin/in_end and
  // out_begin/out_end.
  std::vector<int> in_edges;
  std::vector<int> out_edges;
};

struct ProjectionStats
{
  int rescaled = 0;
  int zero_sum_fallback = 0;
  // Vertices whose in-arc rescale hit the floor-aware redistribution slow path
  // (C3 item 2): at least one in-arc clamped up to the floor, so the residual
  // was water-filled over the rest to keep Σ_in = target. 0 means every rescale
  // was the plain single scale (nothing clamped). Reported at debug level 2.
  int floor_redistributed = 0;
};

// Pure core (no STA): proportional reverse-topological flow projection.
// Rescales `lambda` in place so that, after it returns:
//   Σλ_in(v) = Σλ_out(v) for internal v
//   Σλ_in(k) = the endpoint target for each endpoint k
//
// The endpoint target is the E4 x E3 seam, and it is what decides whether a
// uniform λ seed constant survives:
//   - anchor to μ (kReseedEachIter / kSeedOnce / kUpdateAsLambda): the target
//     is μ_k, a slack-derived field independent of λ's magnitude, so a seed
//     λ ≡ c is rescaled to μ_k/m at every endpoint and c cancels exactly -
//     by induction down the levels the whole field is c-free (the hardening
//     pass's "λ-magnitude finding"). This is the historical behavior.
//   - anchor to λ (kEndpointLambda): the target is the endpoint's own in-sum,
//     so the rescale is a no-op there and the endpoint arcs' λ - ordinary
//     slack-scaled arcs, as in Flach / Livramento / Mangiras - are the boundary
//     condition. Both target and in_sum then scale with c, so every scale
//     factor is c-invariant and the projection commutes with a uniform rescale:
//     project(c·λ) = c·project(λ). The seed magnitude survives.
//     μ_k is written back as the in-sum actually used, so it stays the true
//     endpoint multiplier for the duality-gap diagnostic.
//   - derive-then-anchor (kEndpointRatio / kEndpointAdditive): μ is derived
//     from the endpoint in-sum on the first projection (`derive_endpoint_mu`
//     true, so μ_0 ∝ the seed magnitude), then the updater's applyMuPolicy
//     maintains it and every later projection anchors to it - the papers' own
//     branch-1 endpoint pressure. Homogeneity holds in both modes (μ_0 ∝ c,
//     and the anchor scale μ/in_sum is c-invariant when μ ∝ c).
ProjectionStats projectFlowBalance(const ProjectionTopology& topo,
                                   GlobalSizingConfig::MuPolicy mu_policy,
                                   bool derive_endpoint_mu,
                                   float floor,
                                   std::vector<float>& mu,
                                   std::vector<float>& lambda);

// E3 axis - KKT flow-balance projection of the multipliers. `first_projection`
// is the run's very first (post-seed) projection: it tells the
// endpoint-pressure policies to derive their initial μ from the seeded λ (see
// projectFlowBalance).
class FlowProjection
{
 public:
  virtual ~FlowProjection() = default;
  virtual void project(LrState& state, bool first_projection) = 0;
};

// Proportional reverse-topological redistribution (the de-facto standard).
class ProportionalReverseTopoProjection : public FlowProjection
{
 public:
  void project(LrState& state, bool first_projection) override;

 private:
  // Rebuilt from the graph on every project() (a sweep can mint arcs), but kept
  // across calls so the per-iteration rebuild reuses the capacity instead of
  // reallocating O(V+E) each time.
  ProjectionTopology topo_;
};

std::unique_ptr<FlowProjection> makeFlowProjection(
    const GlobalSizingConfig& config);

}  // namespace rsz
