// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>
#include <vector>

#include "LrState.hh"
#include "rsz/GlobalSizingConfig.hh"

namespace rsz {

// The projection's STA-free view of the timing graph, built from sta::Graph by
// project() and consumed by projectFlowBalance(). Vertices are stored in visit
// order (descending level, so endpoints come before their fanin); each owns a
// contiguous range of in- and out-arc multiplier indices into state.lambda.
// Arcs without a multiplier are left out: non-data arcs, and arcs whose ids lie
// beyond the multiplier vector because a cell replacement rebuilt the graph.
//
// Keeping the arithmetic in a function over this plain structure lets it be
// unit-tested without STA; the STA walk only fills the structure in.
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
  // Vertices where at least one in-arc was clamped up to the floor, so the
  // remainder was redistributed over the other in-arcs to keep the in-arc sum
  // equal to the target. Reported at debug level 2.
  int floor_redistributed = 0;
};

// Proportional reverse-topological flow projection (no STA). Rescales `lambda`
// in place so that afterwards
//   sum(lambda_in(v)) = sum(lambda_out(v))  for every internal vertex v
//   sum(lambda_in(k)) = target(k)           for every endpoint k
//
// The endpoint target depends on mu_policy, and it decides whether the
// magnitude of a uniform lambda seed survives the projection:
//   - Anchor to mu (kReseedEachIter, kSeedOnce, kUpdateAsLambda): the target is
//     mu_k, which comes from endpoint slack and does not depend on lambda. A
//     seed lambda = c is rescaled to mu_k/m on each of an endpoint's m in-arcs,
//     so c cancels, and by induction over the levels the whole field is
//     independent of c.
//   - Anchor to lambda (kEndpointLambda): the target is the endpoint's own
//     in-arc sum, so the rescale does nothing there, and the endpoint arcs,
//     updated like any other arc (as in Flach, Livramento and Mangiras), are
//     the boundary condition. Every scale factor is then independent of c, so
//     project(c * lambda) = c * project(lambda) and the seed magnitude
//     survives. mu_k is set to the in-arc sum so it stays the true endpoint
//     multiplier for the duality-gap diagnostic.
//   - Derive, then anchor (kEndpointRatio, kEndpointAdditive): on the first
//     projection (`derive_endpoint_mu` true) mu is set from the endpoint's
//     in-arc sum, so mu_0 is proportional to the seed magnitude. After that
//     applyMuPolicy() updates mu with the paper's endpoint rule, and every
//     later projection anchors to it. The result scales with c in both
//     phases.
ProjectionStats projectFlowBalance(const ProjectionTopology& topo,
                                   GlobalSizingConfig::MuPolicy mu_policy,
                                   bool derive_endpoint_mu,
                                   float floor,
                                   std::vector<float>& mu,
                                   std::vector<float>& lambda);

// KKT flow-balance projection of the multipliers. `first_projection` is true
// for the first projection of a run, right after seeding; it tells the
// endpoint_ratio and endpoint_additive policies to derive their initial mu from
// the seeded lambda (see projectFlowBalance).
class FlowProjection
{
 public:
  virtual ~FlowProjection() = default;
  virtual void project(LrState& state, bool first_projection) = 0;
};

// Proportional reverse-topological redistribution, the usual projection in LR
// gate sizing.
class ProportionalReverseTopoProjection : public FlowProjection
{
 public:
  void project(LrState& state, bool first_projection) override;

 private:
  // Rebuilt from the graph on every project(), since a sweep can add arcs, but
  // kept as a member so the rebuild reuses its capacity instead of
  // reallocating O(V+E) each time.
  ProjectionTopology topo_;
};

std::unique_ptr<FlowProjection> makeFlowProjection(
    const GlobalSizingConfig& config);

}  // namespace rsz
