// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "CostTerms.hh"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/NetworkClass.hh"
#include "sta/Scene.hh"
#include "sta/Sta.hh"
#include "sta/Transition.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

// ===========================================================================
// Pure formulas (no STA).
// ===========================================================================

float candidateSlewDelta(const float cur_slew,
                         const float drive_res,
                         const float cand_drive_res)
{
  if (drive_res <= 0.0f) {
    return 0.0f;
  }
  return cur_slew * (cand_drive_res / drive_res - 1.0f);
}

float slewSensitivityCost(const float sens_sum, const float slew_delta)
{
  return sens_sum * slew_delta;
}

float flachPhiArc(const float lambda,
                  const float dd_dslew,
                  const float dslew_dslew,
                  const float downstream_sum,
                  const bool dominant)
{
  const float local = lambda * dd_dslew;
  return dominant ? local + dslew_dslew * downstream_sum : local;
}

float deltaDelayReferenced(const float d_cand, const float d_ref)
{
  return d_cand - d_ref;
}

float perArcTimingCost(const std::vector<ArcLambdaDelay>& arcs)
{
  float cost = 0.0f;
  for (const ArcLambdaDelay& a : arcs) {
    cost += a.lambda * a.delay;
  }
  return cost;
}

float portWorstTimingCost(const float lambda_sum, const float port_worst_delay)
{
  return lambda_sum * port_worst_delay;
}

// ===========================================================================
// Main-thread passes.
// ===========================================================================

namespace {

// Worst rise/fall slew at v over all scenes, max corner.
float slewOf(LrState& state, sta::Vertex* v)
{
  return sta::delayAsFloat(state.sta->slew(
      v, sta::RiseFallBoth::riseFall(), state.sta->scenes(), state.max));
}

// Input-slew sensitivities of the gate arcs into `out_port`, by finite
// difference on the NLDM tables at the current input slew and output load.
// Outputs: s1 = δd/δslew, s2 = δslew_out/δslew, and base_out_slew = the output
// slew at the unperturbed point (used to pick the dominant arc). Falls back to
// the wire pass-through values (s1 = 0, s2 = 1) when the lookup is degenerate.
void gateArcSensitivity(LrState& state,
                        sta::LibertyPort* out_port,
                        float load,
                        float in_slew,
                        float& s1,
                        float& s2,
                        float& base_out_slew)
{
  s1 = 0.0f;
  s2 = 1.0f;
  base_out_slew = 0.0f;
  const sta::Scene* scene = state.sta->cmdScene();
  const sta::MinMax* max = state.max;

  const float delta = std::max(std::fabs(in_slew) * 0.05f, 1e-13f);
  sta::Slew in0[sta::RiseFall::index_count];
  sta::Slew in1[sta::RiseFall::index_count];
  for (int i : sta::RiseFall::rangeIndex()) {
    in0[i] = in_slew;
    in1[i] = in_slew + delta;
  }
  sta::ArcDelay d0[sta::RiseFall::index_count];
  sta::ArcDelay d1[sta::RiseFall::index_count];
  sta::Slew os0[sta::RiseFall::index_count];
  sta::Slew os1[sta::RiseFall::index_count];
  state.resizer->gateDelays(out_port, load, in0, scene, max, d0, os0);
  state.resizer->gateDelays(out_port, load, in1, scene, max, d1, os1);

  float d0m = -sta::INF;
  float d1m = -sta::INF;
  float os0m = -sta::INF;
  float os1m = -sta::INF;
  for (int i : sta::RiseFall::rangeIndex()) {
    d0m = std::max(d0m, sta::delayAsFloat(d0[i]));
    d1m = std::max(d1m, sta::delayAsFloat(d1[i]));
    os0m = std::max(os0m, sta::delayAsFloat(os0[i]));
    os1m = std::max(os1m, sta::delayAsFloat(os1[i]));
  }
  // Degenerate (no usable arc into this port): keep the pass-through defaults.
  if (d0m <= -sta::INF / 2 || os0m <= -sta::INF / 2) {
    return;
  }
  const float cand_s1 = (d1m - d0m) / delta;
  const float cand_s2 = (os1m - os0m) / delta;
  if (std::isfinite(cand_s1)) {
    s1 = std::max(0.0f, cand_s1);
  }
  if (std::isfinite(cand_s2)) {
    s2 = std::max(0.0f, cand_s2);
  }
  base_out_slew = os0m;
}

}  // namespace

void computePhiSensitivities(LrState& state)
{
  sta::Graph* graph = state.graph;
  sta::dbNetwork* network = state.db_network;
  sta::GraphDelayCalc* gdc = state.sta->graphDelayCalc();
  const sta::Scene* scene = state.sta->cmdScene();
  const sta::MinMax* max = state.max;

  state.phi.assign(state.lambda.size(), 0.0f);

  // Reverse-topological order: descending level, so a vertex's out-edges (which
  // point to higher levels) already have their φ computed before we compute the
  // φ of its in-edges.
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

  // Per in-edge working record for the current vertex.
  struct InArc
  {
    sta::EdgeId id = 0;
    float s1 = 0.0f;
    float s2 = 1.0f;
    bool gate = false;
    float base_out_slew = 0.0f;
  };
  std::vector<InArc> in_arcs;

  for (sta::Vertex* w : vertices) {
    // Σ φ over w's out data edges. They end at higher levels, so they are
    // already computed.
    float down_sum = 0.0f;
    {
      sta::VertexOutEdgeIterator oeit(w, graph);
      while (oeit.hasNext()) {
        sta::Edge* e = oeit.next();
        if (!state.isDataArc(e)) {
          continue;
        }
        const sta::EdgeId id = graph->id(e);
        if (static_cast<size_t>(id) < state.phi.size()) {
          down_sum += state.phi[id];
        }
      }
    }

    const sta::Pin* to_pin = w->pin();
    sta::LibertyPort* out_port = network->libertyPort(to_pin);
    const float load = gdc->loadCap(to_pin, scene, max);

    in_arcs.clear();
    sta::VertexInEdgeIterator ieit(w, graph);
    while (ieit.hasNext()) {
      sta::Edge* e = ieit.next();
      if (!state.isDataArc(e)) {
        continue;
      }
      const sta::EdgeId id = graph->id(e);
      if (static_cast<size_t>(id) >= state.phi.size()) {
        continue;
      }
      sta::Vertex* u = e->from(graph);
      const sta::Pin* from_pin = u->pin();
      const sta::Instance* fi = network->instance(from_pin);
      const bool gate = (fi != nullptr && fi == network->instance(to_pin));
      InArc rec;
      rec.id = id;
      rec.gate = gate;
      if (gate && out_port != nullptr) {
        gateArcSensitivity(state,
                           out_port,
                           load,
                           slewOf(state, u),
                           rec.s1,
                           rec.s2,
                           rec.base_out_slew);
      }
      in_arcs.push_back(rec);
    }

    // The dominant gate arc is the one with the worst (largest) output slew.
    // Only it propagates the downstream sensitivity, so a gate's input arcs do
    // not each count the whole cone again (Flach Sec. VII-C). A wire arc is
    // the load pin's only driver, so it is always dominant.
    int dom_idx = -1;
    float dom_slew = -sta::INF;
    for (size_t i = 0; i < in_arcs.size(); ++i) {
      if (in_arcs[i].gate && in_arcs[i].base_out_slew > dom_slew) {
        dom_slew = in_arcs[i].base_out_slew;
        dom_idx = static_cast<int>(i);
      }
    }
    for (size_t i = 0; i < in_arcs.size(); ++i) {
      const InArc& a = in_arcs[i];
      const bool dominant = a.gate ? (static_cast<int>(i) == dom_idx) : true;
      state.phi[a.id]
          = flachPhiArc(state.lambda[a.id], a.s1, a.s2, down_sum, dominant);
    }
  }

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR phi pass: {} vertices, {} edges",
             vertices.size(),
             state.phi.size());
}

void captureReferenceDelays(LrState& state)
{
  sta::Graph* graph = state.graph;
  state.prev_delay.assign(state.lambda.size(), 0.0f);
  sta::VertexIterator vit(graph);
  while (vit.hasNext()) {
    sta::Vertex* v = vit.next();
    sta::VertexOutEdgeIterator eit(v, graph);
    while (eit.hasNext()) {
      sta::Edge* e = eit.next();
      if (!state.isDataArc(e)) {
        continue;
      }
      const sta::EdgeId id = graph->id(e);
      if (static_cast<size_t>(id) >= state.prev_delay.size()) {
        continue;
      }
      state.prev_delay[id] = state.edgeMaxArcDelay(e);
    }
  }
}

}  // namespace rsz
