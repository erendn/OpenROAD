// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "TimingScale.hh"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "LRSubproblem.hh"
#include "db_sta/dbSta.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/PortDirection.hh"
#include "sta/Scene.hh"
#include "sta/Sta.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

namespace {

// Median by the same nth_element convention the historical walk used (lower of
// the two middles on an even count, since size/2 indexes the upper-middle and
// nth_element only guarantees the nth in place). Returns 0 on an empty range.
float median(std::vector<float>& v)
{
  if (v.empty()) {
    return 0.0f;
  }
  const auto mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  return v[mid];
}

}  // namespace

TimingScaleWeight computeTimingWeight(
    const TimingScaleInput& in,
    const GlobalSizingConfig::TimingScale scale,
    const float timing_bias)
{
  // Only `unit` drops λ from the anchor. livramento_alpha deliberately keeps
  // auto_median's λ-invariant anchor as its base - its α scales that, and the
  // seeds it runs on have no meaningful λ magnitude to preserve (see
  // TimingScale::kLivramentoAlpha).
  const bool lambda_free = (scale == GlobalSizingConfig::TimingScale::kUnit);

  std::vector<float> leakages;
  std::vector<float> anchors;
  leakages.reserve(in.gates.size());
  anchors.reserve(in.gates.size());
  for (const TimingScaleInput::Gate& g : in.gates) {
    leakages.push_back(g.leakage);
    if (lambda_free) {
      if (g.has_delay) {
        anchors.push_back(g.delay);
      }
    } else if (g.has_pressure) {
      anchors.push_back(g.lambda_delay);
    }
  }

  TimingScaleWeight out;
  out.degenerate = leakages.empty() || anchors.empty();
  if (!out.degenerate) {
    out.l_med = median(leakages);
    out.anchor_med = median(anchors);
    if (out.l_med <= 0.0f || out.anchor_med <= 0.0f) {
      out.degenerate = true;
    }
  }
  if (out.degenerate) {
    out.tw = 1.0f;
    return out;
  }

  // auto_median's bias is its own balance knob (rsz_baseline's 64). The λ-free
  // anchor carries the balance in λ itself, so it takes the median ratio neat -
  // multiplying by a bias here would just re-introduce a second, redundant
  // scale on an axis whose point is that λ's magnitude is the scale.
  out.tw = lambda_free ? (out.l_med / out.anchor_med)
                       : (timing_bias * out.l_med / out.anchor_med);
  return out;
}

float rescheduleLivramentoAlpha(const float alpha,
                                const float T,
                                const float wns,
                                bool* alpha_floor_bound)
{
  const float raw
      = (T <= 0.0f)
            ? alpha
            // max_j a_j read as T - WNS (exact when every endpoint
            // is required at T).
            : alpha * (T / std::max(T - wns, kLivramentoArrivalFloorFrac * T));
  if (alpha_floor_bound != nullptr && raw < kLivramentoAlphaFloor) {
    *alpha_floor_bound = true;
  }
  return std::max(raw, kLivramentoAlphaFloor);
}

void collectTimingScaleInput(LrState& state,
                             Resizer* resizer,
                             const LRSubproblem& subproblem,
                             const GlobalSizingConfig::TimingScale scale,
                             TimingScaleInput& in)
{
  const bool lambda_free = (scale == GlobalSizingConfig::TimingScale::kUnit);
  const GlobalSizingConfig& params = *state.config;
  sta::Network* network = state.network;
  sta::Graph* graph = state.graph;
  const sta::Scene* scene = state.sta->cmdScene();
  const int lambda_size = static_cast<int>(state.lambda.size());

  in.gates.clear();

  std::unique_ptr<sta::LeafInstanceIterator> iit(
      network->leafInstanceIterator());
  while (iit->hasNext()) {
    sta::Instance* inst = iit->next();
    if (resizer->dontTouch(inst)) {
      continue;
    }
    sta::LibertyCell* cell = network->libertyCell(inst);
    if (cell == nullptr) {
      continue;
    }

    TimingScaleInput::Gate gate;
    gate.leakage = subproblem.leakageOrArea(cell);

    std::unique_ptr<sta::InstancePinIterator> pit(network->pinIterator(inst));
    while (pit->hasNext()) {
      sta::Pin* pin = pit->next();
      const sta::PortDirection* dir = network->direction(pin);
      if (!dir->isOutput()) {
        continue;
      }
      sta::Vertex* v = graph->pinDrvrVertex(pin);
      if (v == nullptr) {
        continue;
      }
      float lam_sum = 0.0f;
      int arcs = 0;
      sta::VertexInEdgeIterator ieit(v, graph);
      while (ieit.hasNext()) {
        sta::Edge* e = ieit.next();
        if (!state.isDataArc(e)) {
          continue;
        }
        const sta::Pin* from_pin = e->from(graph)->pin();
        if (network->instance(from_pin) != inst) {
          continue;
        }
        const sta::EdgeId id = graph->id(e);
        if (std::cmp_greater_equal(id, lambda_size)) {
          continue;
        }
        lam_sum += state.lambda[id];
        ++arcs;
      }
      // auto_median samples a pin only where λ is meaningfully above the floor;
      // the λ-free anchor samples every pin that drives a data arc. Both need
      // the same `d`, so compute it once for whichever is active.
      const bool priced = (lam_sum > 4.0f * params.lambda_floor);
      const bool want_delay = lambda_free && arcs > 0;
      if (!priced && !want_delay) {
        continue;
      }
      sta::LibertyPort* port = network->libertyPort(pin);
      if (port == nullptr) {
        continue;
      }
      const float load
          = state.sta->graphDelayCalc()->loadCap(pin, scene, state.max);
      const float d
          = sta::delayAsFloat(resizer->gateDelay(port, load, scene, state.max));
      if (priced) {
        gate.lambda_delay += lam_sum * d;
        gate.has_pressure = true;
      }
      if (want_delay) {
        gate.delay += d;
        gate.has_delay = true;
      }
    }
    in.gates.push_back(gate);
  }
}

float timingWeightBase(LrState& state,
                       Resizer* resizer,
                       const LRSubproblem& subproblem)
{
  const GlobalSizingConfig& params = *state.config;
  TimingScaleInput in;
  collectTimingScaleInput(state, resizer, subproblem, params.timing_scale, in);
  const TimingScaleWeight w
      = computeTimingWeight(in, params.timing_scale, params.timing_bias);

  if (w.degenerate) {
    debugPrint(state.logger,
               RSZ,
               "global_sizing",
               1,
               "LR timing_weight: scale={} degenerate "
               "(gates={}, L_med={:.3g}, anchor_med={:.3g}); using 1.0",
               toString(params.timing_scale),
               in.gates.size(),
               w.l_med,
               w.anchor_med);
  } else {
    debugPrint(state.logger,
               RSZ,
               "global_sizing",
               1,
               "LR timing_weight: scale={} bias={:.3g} "
               "L_med={:.3g} anchor_med={:.3g} -> tw={:.3g}",
               toString(params.timing_scale),
               params.timing_bias,
               w.l_med,
               w.anchor_med,
               w.tw);
  }
  return w.tw;
}

}  // namespace rsz
