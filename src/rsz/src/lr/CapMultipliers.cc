// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "CapMultipliers.hh"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

#include "ElectricalModel.hh"
#include "LrState.hh"
#include "ObjectivePower.hh"
#include "ViolationRepair.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "sta/Graph.hh"
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

// Median by nth_element at index size/2, as the timing weight takes it. 0 for
// an empty vector.
float median(std::vector<float> v)
{
  if (v.empty()) {
    return 0.0f;
  }
  const auto mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  return v[mid];
}

float pinLoad(const LrState& state, const sta::Pin* pin)
{
  return state.sta->graphDelayCalc()->loadCap(
      pin, state.sta->cmdScene(), state.max);
}

// The Liberty limit of the pin's current cell.
std::optional<float> pinLimit(const LrState& state, const sta::Pin* pin)
{
  const sta::LibertyPort* port = state.network->libertyPort(pin);
  if (port == nullptr) {
    return std::nullopt;
  }
  return libertyMaxCap(port, state.max);
}

// The objective power of the gate's current cell, when the cell has leakage
// data.
std::optional<float> gatePower(const LrState& state, const sta::Instance* inst)
{
  sta::LibertyCell* cell = state.network->libertyCell(inst);
  const std::optional<float> leakage = state.resizer->cellLeakage(cell);
  if (!leakage.has_value()) {
    return std::nullopt;
  }
  const ObjectivePower& power = state.objective_power;
  return power.power(cell,
                     *leakage,
                     sizingPowerInputs(power, *state.resizer, inst, state.max));
}

}  // namespace

float capBetaCost(const float beta, const float load, const float limit)
{
  return beta * (load - limit);
}

float updatedCapBeta(const float beta, const float load, const float limit)
{
  return limit > 0.0f ? beta * (load / limit) : beta;
}

float initialCapBeta(const float median_power,
                     const float timing_weight,
                     const float median_limit)
{
  if (median_power <= 0.0f || timing_weight <= 0.0f || median_limit <= 0.0f) {
    return 1.0f;
  }
  const float beta0
      = kInitialCapBetaShare * median_power / (timing_weight * median_limit);
  return (std::isfinite(beta0) && beta0 > 0.0f) ? beta0 : 1.0f;
}

void CapMultipliers::start(const LrState& state, const float timing_weight)
{
  pins_.clear();
  std::vector<float> limits;
  std::unique_ptr<sta::LeafInstanceIterator> iit(
      state.network->leafInstanceIterator());
  while (iit->hasNext()) {
    const sta::Instance* inst = iit->next();
    // Registers are priced even when they are not resized (see the header).
    if (maySizeGateWithRegistersSized(state, inst)) {
      addPricedPins(state, inst, limits);
    }
  }
  // The power a candidate's cost trades against, so registers count only
  // when they are resized.
  std::vector<float> powers;
  for (sta::Instance* inst : sizableGates(state)) {
    if (const std::optional<float> power = gatePower(state, inst)) {
      powers.push_back(*power);
    }
  }

  const float median_power = median(powers);
  const float median_limit = median(limits);
  beta0_ = initialCapBeta(median_power, timing_weight, median_limit);
  sta::VertexId max_vertex = 0;
  for (const PricedPin& p : pins_) {
    max_vertex = std::max(max_vertex, p.vertex);
  }
  beta_.assign(pins_.empty() ? 0 : static_cast<size_t>(max_vertex) + 1, -1.0f);
  for (const PricedPin& p : pins_) {
    beta_[p.vertex] = beta0_;
  }
  violating_at_start_ = countViolating(state);
  started_ = true;
  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR cap multipliers: {} pins, median power {:.3g}, timing weight "
             "{:.3g}, median limit {:.3g} -> beta0={:.3g}",
             pins_.size(),
             median_power,
             timing_weight,
             median_limit,
             beta0_);
}

void CapMultipliers::addPricedPins(const LrState& state,
                                   const sta::Instance* inst,
                                   std::vector<float>& limits)
{
  sta::Network* network = state.network;
  sta::Graph* graph = state.graph;
  std::unique_ptr<sta::InstancePinIterator> pit(network->pinIterator(inst));
  while (pit->hasNext()) {
    const sta::Pin* pin = pit->next();
    if (!network->direction(pin)->isOutput()) {
      continue;
    }
    const sta::Vertex* v = graph->pinDrvrVertex(pin);
    const std::optional<float> limit = pinLimit(state, pin);
    if (v == nullptr || !limit.has_value()) {
      continue;
    }
    pins_.push_back({.pin = pin, .vertex = graph->id(v)});
    limits.push_back(*limit);
  }
}

void CapMultipliers::update(const LrState& state)
{
  int violating = 0;
  for (const PricedPin& p : pins_) {
    const std::optional<float> limit = pinLimit(state, p.pin);
    if (!limit.has_value()) {
      continue;
    }
    const float load = pinLoad(state, p.pin);
    float& beta = beta_[p.vertex];
    const float prev = beta;
    beta = updatedCapBeta(beta, load, *limit);
    if (load > *limit) {
      ++violating;
      debugPrint(state.logger,
                 RSZ,
                 "global_sizing_cap_multipliers",
                 1,
                 "multiplier {} load={:.4g} limit={:.4g}: {:.4g} -> {:.4g}",
                 state.network->pathName(p.pin),
                 load,
                 *limit,
                 prev,
                 beta);
    }
  }
  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR cap multipliers: {} of {} pins over their max capacitance",
             violating,
             pins_.size());
}

std::optional<float> CapMultipliers::beta(const sta::VertexId vertex) const
{
  if (vertex >= beta_.size() || beta_[vertex] < 0.0f) {
    return std::nullopt;
  }
  return beta_[vertex];
}

int CapMultipliers::countViolating(const LrState& state) const
{
  return static_cast<int>(
      std::ranges::count_if(pins_, [&state](const PricedPin& p) {
        const std::optional<float> limit = pinLimit(state, p.pin);
        return limit.has_value() && pinLoad(state, p.pin) > *limit;
      }));
}

}  // namespace rsz
