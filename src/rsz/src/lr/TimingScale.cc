// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "TimingScale.hh"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "LRSubproblem.hh"
#include "ObjectivePower.hh"
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

// Median by nth_element at index size/2, i.e. the upper of the two middle
// elements for an even count. Returns 0 for an empty vector.
float median(std::vector<float>& v)
{
  if (v.empty()) {
    return 0.0f;
  }
  const auto mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  return v[mid];
}

// The gate-internal data arcs into `inst`'s driver vertex `v` that have a
// multiplier.
void pricedInArcs(const LrState& state,
                  const sta::Instance* inst,
                  sta::Vertex* v,
                  std::vector<sta::Edge*>& arcs)
{
  arcs.clear();
  sta::VertexInEdgeIterator ieit(v, state.graph);
  while (ieit.hasNext()) {
    sta::Edge* e = ieit.next();
    if (!state.isDataArc(e)) {
      continue;
    }
    const sta::Pin* from_pin = e->from(state.graph)->pin();
    if (state.network->instance(from_pin) != inst) {
      continue;
    }
    if (std::cmp_greater_equal(state.graph->id(e), state.lambda.size())) {
      continue;
    }
    arcs.push_back(e);
  }
}

// sum(lambda * d) over `arcs`, each arc at its own delay at `load`.
float pinPerArcLambdaDelay(const LrState& state,
                           Resizer* resizer,
                           const std::vector<sta::Edge*>& arcs,
                           const float load,
                           const sta::Scene* scene)
{
  float lambda_delay = 0.0f;
  for (sta::Edge* e : arcs) {
    lambda_delay
        += state.lambda[state.graph->id(e)]
           * sta::delayAsFloat(resizer->arcSetDelay(e->timingArcSet(),
                                                    load,
                                                    scene,
                                                    state.max,
                                                    state.sta->arcDelayCalc()));
  }
  return lambda_delay;
}

}  // namespace

TimingScaleWeight computeTimingWeight(
    const TimingScaleInput& in,
    const GlobalSizingConfig::TimingScale scale,
    const float timing_bias,
    const GlobalSizingConfig::PowerObjective objective)
{
  // Only `unit` drops lambda from the anchor; livramento_alpha keeps
  // auto_median's anchor as its base (see the header).
  const bool lambda_free = (scale == GlobalSizingConfig::TimingScale::kUnit);
  const bool total_power
      = (objective == GlobalSizingConfig::PowerObjective::kTotal);

  std::vector<float> leakages;
  std::vector<float> anchors;
  leakages.reserve(in.gates.size());
  anchors.reserve(in.gates.size());
  for (const TimingScaleInput::Gate& g : in.gates) {
    leakages.push_back(total_power ? g.power : g.leakage);
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

  // timing_bias sets the power/timing balance for the lambda-invariant
  // anchor (64 by default). The lambda-free anchor takes its balance from the
  // magnitude of lambda itself, so it uses the median ratio directly; a bias
  // here would only add a second, redundant scale.
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
  const bool per_arc
      = (params.timing_cost == GlobalSizingConfig::TimingCost::kPerArc);
  sta::Network* network = state.network;
  sta::Graph* graph = state.graph;
  const sta::Scene* scene = state.sta->cmdScene();
  std::vector<sta::Edge*> pin_arcs;

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
    gate.power = state.objective_power.power(
        cell,
        gate.leakage,
        sizingPowerInputs(state.objective_power, *resizer, inst, state.max));

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
      pricedInArcs(state, inst, v, pin_arcs);
      float lam_sum = 0.0f;
      for (sta::Edge* e : pin_arcs) {
        lam_sum += state.lambda[graph->id(e)];
      }
      // auto_median samples a pin only where lambda is well above the floor;
      // the lambda-free anchor samples every pin that drives a data arc. Both
      // need the same `d`, so compute it once for whichever is active.
      const bool priced = (lam_sum > 4.0f * params.lambda_floor);
      const bool want_delay = lambda_free && !pin_arcs.empty();
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
        gate.lambda_delay
            += per_arc
                   ? pinPerArcLambdaDelay(state, resizer, pin_arcs, load, scene)
                   : lam_sum * d;
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
  const TimingScaleWeight w = computeTimingWeight(
      in, params.timing_scale, params.timing_bias, params.power_objective);
  // L_med is the median leakage, P_med the median objective power.
  const char* med_name
      = params.power_objective == GlobalSizingConfig::PowerObjective::kTotal
            ? "P_med"
            : "L_med";

  if (w.degenerate) {
    debugPrint(state.logger,
               RSZ,
               "global_sizing",
               1,
               "LR timing_weight: scale={} degenerate "
               "(gates={}, {}={:.3g}, anchor_med={:.3g}); using 1.0",
               toString(params.timing_scale),
               in.gates.size(),
               med_name,
               w.l_med,
               w.anchor_med);
  } else {
    debugPrint(state.logger,
               RSZ,
               "global_sizing",
               1,
               "LR timing_weight: scale={} bias={:.3g} "
               "{}={:.3g} anchor_med={:.3g} -> tw={:.3g}",
               toString(params.timing_scale),
               params.timing_bias,
               med_name,
               w.l_med,
               w.anchor_med,
               w.tw);
  }
  return w.tw;
}

}  // namespace rsz
