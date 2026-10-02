// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "ObjectivePower.hh"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ElectricalModel.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "sta/FuncExpr.hh"
#include "sta/Fuzzy.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/InternalPower.hh"
#include "sta/Liberty.hh"
#include "sta/LibertyClass.hh"
#include "sta/MinMax.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/PortDirection.hh"
#include "sta/PowerClass.hh"
#include "sta/Scene.hh"
#include "sta/Sdc.hh"
#include "sta/Sta.hh"
#include "sta/TimingArc.hh"
#include "sta/Transition.hh"

namespace rsz {

namespace {

using Op = sta::FuncExpr::Op;

// The duty used where no better estimate is available, as OpenSTA does for a
// related input that neither the output function nor a condition reads.
constexpr float kUnknownDuty = 0.5f;

// Truth-table enumeration is exponential in the number of ports an expression
// reads. Liberty functions and conditions read far fewer ports than this; an
// expression that reads more gets kUnknownDuty.
constexpr int kMaxExprPorts = 16;

// The distinct ports an expression reads; port i is bit i of an assignment.
struct ExprPorts
{
  std::array<const sta::LibertyPort*, kMaxExprPorts> ports{};
  int count = 0;
  bool overflow = false;

  int indexOf(const sta::LibertyPort* port) const
  {
    for (int i = 0; i < count; ++i) {
      if (ports[i] == port) {
        return i;
      }
    }
    return -1;
  }
};

void collectPorts(const sta::FuncExpr* expr, ExprPorts& ports)
{
  switch (expr->op()) {
    case Op::port:
      if (ports.indexOf(expr->port()) < 0) {
        if (ports.count == kMaxExprPorts) {
          ports.overflow = true;
        } else {
          ports.ports[ports.count++] = expr->port();
        }
      }
      return;
    case Op::not_:
      collectPorts(expr->left(), ports);
      return;
    case Op::and_:
    case Op::or_:
    case Op::xor_:
      collectPorts(expr->left(), ports);
      collectPorts(expr->right(), ports);
      return;
    case Op::one:
    case Op::zero:
      return;
  }
}

// The value of `expr` when each port takes its bit of `values`.
bool evalExpr(const sta::FuncExpr* expr,
              const ExprPorts& ports,
              const uint32_t values)
{
  switch (expr->op()) {
    case Op::port:
      return ((values >> ports.indexOf(expr->port())) & 1u) != 0;
    case Op::not_:
      return !evalExpr(expr->left(), ports, values);
    case Op::and_:
      return evalExpr(expr->left(), ports, values)
             && evalExpr(expr->right(), ports, values);
    case Op::or_:
      return evalExpr(expr->left(), ports, values)
             || evalExpr(expr->right(), ports, values);
    case Op::xor_:
      return evalExpr(expr->left(), ports, values)
             != evalExpr(expr->right(), ports, values);
    case Op::one:
      return true;
    case Op::zero:
      return false;
  }
  return false;
}

const PowerPin* findPin(const GatePowerInputs& gate, const std::string& name)
{
  for (const PowerPin& pin : gate.pins) {
    if (pin.port->name() == name) {
      return &pin;
    }
  }
  return nullptr;
}

// The duty of a register's internal state, read from the output pin whose
// function is that state.
float stateDuty(const sta::LibertyPort* state, const GatePowerInputs& gate)
{
  for (const PowerPin& pin : gate.pins) {
    const sta::FuncExpr* func = pin.port->function();
    if (func != nullptr && func->op() == Op::port
        && func->port()->name() == state->name()) {
      return pin.activity.duty;
    }
  }
  return kUnknownDuty;
}

float portDuty(const sta::LibertyPort* port, const GatePowerInputs& gate)
{
  if (port->direction()->isInternal()) {
    return stateDuty(port, gate);
  }
  const PowerPin* pin = findPin(gate, port->name());
  return pin != nullptr ? pin->activity.duty : 0.0f;
}

// The duties of `ports`, in the same order.
std::array<float, kMaxExprPorts> portDuties(const ExprPorts& ports,
                                            const GatePowerInputs& gate)
{
  std::array<float, kMaxExprPorts> duties{};
  for (int i = 0; i < ports.count; ++i) {
    duties[i] = portDuty(ports.ports[i], gate);
  }
  return duties;
}

// Probability of the assignment `values` with independent ports, leaving out
// port `skip` (-1 to use every port).
double assignmentProbability(const std::array<float, kMaxExprPorts>& duties,
                             const int count,
                             const uint32_t values,
                             const int skip)
{
  double probability = 1.0;
  for (int i = 0; i < count; ++i) {
    if (i != skip) {
      probability *= ((values >> i) & 1u) != 0 ? duties[i] : 1.0 - duties[i];
    }
  }
  return probability;
}

bool isSignalPort(const sta::LibertyPort* port)
{
  return port != nullptr
         && (port->direction()->isAnyInput()
             || port->direction()->isAnyOutput());
}

// The first output port `expr` reads, as OpenSTA's Power::findExprOutPort
// finds it.
const sta::LibertyPort* exprOutputPort(const sta::FuncExpr* expr)
{
  switch (expr->op()) {
    case Op::port:
      return expr->port()->direction()->isAnyOutput() ? expr->port() : nullptr;
    case Op::not_:
      return exprOutputPort(expr->left());
    case Op::and_:
    case Op::or_:
    case Op::xor_: {
      const sta::LibertyPort* port = exprOutputPort(expr->left());
      return port != nullptr ? port : exprOutputPort(expr->right());
    }
    case Op::one:
    case Op::zero:
      return nullptr;
  }
  return nullptr;
}

// The duty OpenSTA gives an input pin's internal_power entry that has a
// `when` condition. If the condition reads an output whose function reads
// the input, it is the probability that the input is sensitized; otherwise
// it is the probability of the condition.
float inputWhenDuty(const sta::LibertyCell* cell,
                    const sta::LibertyPort* input,
                    const sta::FuncExpr* when,
                    const GatePowerInputs& gate)
{
  const sta::LibertyPort* when_output = exprOutputPort(when);
  if (when_output == nullptr) {
    return exprDuty(when, gate);
  }
  const sta::LibertyPort* output = cell->findLibertyPort(when_output->name());
  if (output == nullptr) {
    return 1.0f;
  }
  const sta::FuncExpr* func = output->function();
  if (func != nullptr && func->hasPort(input)) {
    return exprDiffDuty(func, input, gate);
  }
  return exprDuty(when, gate);
}

// The duty OpenSTA gives the related input of an output's internal_power
// entry (Power::findInputDuty): the probability that the output function is
// sensitized to it; failing that, the probability of the entry's `when`
// condition; failing that, 0.5.
float relatedInputDuty(const sta::LibertyCell* cell,
                       const sta::FuncExpr* func,
                       const sta::InternalPower* pwr,
                       const GatePowerInputs& gate)
{
  const sta::LibertyPort* related = pwr->relatedPort();
  const sta::LibertyPort* from_port = cell->findLibertyPort(related->name());
  if (from_port == nullptr || findPin(gate, related->name()) == nullptr) {
    return 0.0f;
  }
  if (func != nullptr && func->hasPort(from_port)) {
    return exprDiffDuty(func, from_port, gate);
  }
  if (pwr->when() != nullptr) {
    return exprDuty(pwr->when(), gate);
  }
  return kUnknownDuty;
}

// Sums of density * duty over an output's internal_power entries, one per
// related power pin (which may be null).
using PgDutySums = std::vector<std::pair<const sta::LibertyPort*, float>>;

// The sum for `pg_pin`, starting at 0 on first use.
float& pgDutySum(PgDutySums& sums, const sta::LibertyPort* pg_pin)
{
  for (auto& [pg, sum] : sums) {
    if (pg == pg_pin) {
      return sum;
    }
  }
  return sums.emplace_back(pg_pin, 0.0f).second;
}

// Whether an output transition comes from the same input transition, as
// OpenSTA decides it: from the sense of the first timing arc set between the
// two ports, with positive-unate and non-unate arcs keeping the transition
// and a pair without arcs counting as positive.
bool keepsTransition(const sta::LibertyCell* cell,
                     const sta::LibertyPort* from,
                     const sta::LibertyPort* to)
{
  const sta::TimingArcSetSeq& arc_sets = cell->timingArcSets(from, to);
  if (arc_sets.empty()) {
    return true;
  }
  const sta::TimingSense sense = arc_sets[0]->sense();
  return sense == sta::TimingSense::positive_unate
         || sense == sta::TimingSense::non_unate;
}

}  // namespace

float exprDuty(const sta::FuncExpr* expr, const GatePowerInputs& gate)
{
  ExprPorts ports;
  collectPorts(expr, ports);
  if (ports.overflow) {
    return kUnknownDuty;
  }
  const std::array<float, kMaxExprPorts> duties = portDuties(ports, gate);
  double duty = 0.0;
  const uint32_t assignments = 1u << ports.count;
  for (uint32_t values = 0; values < assignments; ++values) {
    if (evalExpr(expr, ports, values)) {
      duty += assignmentProbability(duties, ports.count, values, /*skip=*/-1);
    }
  }
  return static_cast<float>(duty);
}

float exprDiffDuty(const sta::FuncExpr* expr,
                   const sta::LibertyPort* port,
                   const GatePowerInputs& gate)
{
  ExprPorts ports;
  collectPorts(expr, ports);
  if (ports.overflow) {
    return kUnknownDuty;
  }
  const int index = ports.indexOf(port);
  if (index < 0) {
    return 0.0f;
  }
  const std::array<float, kMaxExprPorts> duties = portDuties(ports, gate);
  const uint32_t bit = 1u << index;
  double duty = 0.0;
  const uint32_t assignments = 1u << ports.count;
  for (uint32_t values = 0; values < assignments; ++values) {
    if ((values & bit) == 0
        && evalExpr(expr, ports, values)
               != evalExpr(expr, ports, values | bit)) {
      duty += assignmentProbability(duties, ports.count, values, index);
    }
  }
  return static_cast<float>(duty);
}

void ObjectivePower::cacheActivities(sta::dbSta* sta, const sta::Scene* scene)
{
  network_ = sta->network();
  scene_ = scene;
  pvt_ = scene->sdc()->operatingConditions(sta::MinMax::max());
  activities_.clear();
  // The first query propagates activities through the whole design; the
  // others read the result.
  std::unique_ptr<sta::LeafInstanceIterator> inst_iter(
      network_->leafInstanceIterator());
  while (inst_iter->hasNext()) {
    const sta::Instance* inst = inst_iter->next();
    std::unique_ptr<sta::InstancePinIterator> pin_iter(
        network_->pinIterator(inst));
    while (pin_iter->hasNext()) {
      const sta::Pin* pin = pin_iter->next();
      if (isSignalPort(network_->libertyPort(pin))) {
        const sta::PwrActivity activity = sta->activity(pin, scene);
        activities_[pin] = {activity.density(), activity.duty()};
      }
    }
  }
}

const PinActivity* ObjectivePower::activity(const sta::Pin* pin) const
{
  auto it = activities_.find(pin);
  return it != activities_.end() ? &it->second : nullptr;
}

GatePowerInputs ObjectivePower::gateInputs(const sta::Instance* inst) const
{
  GatePowerInputs gate;
  std::unique_ptr<sta::InstancePinIterator> pin_iter(
      network_->pinIterator(inst));
  while (pin_iter->hasNext()) {
    const sta::Pin* pin = pin_iter->next();
    const sta::LibertyPort* port = network_->libertyPort(pin);
    if (!isSignalPort(port)) {
      continue;
    }
    PowerPin power_pin;
    power_pin.port = port;
    if (const PinActivity* pin_activity = activity(pin)) {
      power_pin.activity = *pin_activity;
    }
    if (port->direction()->isAnyInput()) {
      readFanin(pin, power_pin);
    }
    gate.pins.push_back(power_pin);
  }
  return gate;
}

void ObjectivePower::readFanin(const sta::Pin* pin, PowerPin& power_pin) const
{
  const sta::PinSet* drivers = network_->drivers(pin);
  if (drivers == nullptr) {
    return;
  }
  for (const sta::Pin* driver : *drivers) {
    const sta::LibertyPort* driver_port = network_->libertyPort(driver);
    if (network_->isTopLevelPort(driver) || driver_port == nullptr) {
      continue;
    }
    if (const PinActivity* driver_activity = activity(driver)) {
      power_pin.fanin_density = driver_activity->density;
    }
    power_pin.fanin_voltage = portVoltage(driver_port);
    return;
  }
}

float ObjectivePower::power(sta::LibertyCell* cell,
                            const float leakage,
                            const GatePowerInputs& gate) const
{
  if (!total_power_) {
    return leakage;
  }
  return leakage + internalPower(cell, gate) + inputSwitchingPower(cell, gate);
}

float ObjectivePower::internalPower(sta::LibertyCell* cell,
                                    const GatePowerInputs& gate) const
{
  const sta::LibertyCell* scene_cell = sceneCell(cell);
  if (scene_cell == nullptr) {
    return 0.0f;
  }
  // Summed pin by pin in the gate's pin order, as OpenSTA sums an instance.
  float internal = 0.0f;
  for (const PowerPin& pin : gate.pins) {
    const sta::LibertyPort* port = cell->findLibertyPort(pin.port->name());
    if (port == nullptr) {
      continue;
    }
    if (port->direction()->isAnyOutput()) {
      internal += outputInternalPower(cell, scene_cell, port, pin, gate);
    }
    if (port->direction()->isAnyInput()) {
      internal += inputInternalPower(cell, scene_cell, port, pin, gate);
    }
  }
  return internal;
}

float ObjectivePower::inputSwitchingPower(const sta::LibertyCell* cell,
                                          const GatePowerInputs& gate) const
{
  float switching = 0.0f;
  for (const PowerPin& pin : gate.pins) {
    if (!pin.port->direction()->isAnyInput()) {
      continue;
    }
    const float c_in
        = portInputCap(cell, pin.port->name().c_str(), sta::MinMax::max());
    switching += 0.5f * c_in * pin.fanin_voltage * pin.fanin_voltage
                 * pin.fanin_density;
  }
  return switching;
}

// Power::findInputInternalPower: each entry's energy at the pin's own slew,
// times its duty and the pin's density. The load is 0 except on a
// bidirectional pin, which is read at its output load.
float ObjectivePower::inputInternalPower(const sta::LibertyCell* cell,
                                         const sta::LibertyCell* scene_cell,
                                         const sta::LibertyPort* port,
                                         const PowerPin& pin,
                                         const GatePowerInputs& gate) const
{
  const sta::LibertyPort* scene_port = scenePort(port);
  if (scene_port == nullptr) {
    return 0.0f;
  }
  float internal = 0.0f;
  for (const sta::InternalPower& pwr : scene_cell->internalPowers()) {
    if (pwr.port() != scene_port) {
      continue;
    }
    const float load_cap
        = port->direction()->isAnyOutput() ? pin.load_cap : 0.0f;
    const float energy
        = tableEnergy(&pwr, pin.slew, /*invert=*/false, load_cap);
    const float duty = pwr.when() != nullptr
                           ? inputWhenDuty(cell, port, pwr.when(), gate)
                           : 1.0f;
    internal += energy * duty * pin.activity.density;
  }
  return internal;
}

// Power::findOutputInternalPower: each entry's energy at its related input's
// slew and the output load, times the output's density. Entries are weighted
// by density * duty of their related input, normalized over all entries that
// share a power pin.
float ObjectivePower::outputInternalPower(const sta::LibertyCell* cell,
                                          const sta::LibertyCell* scene_cell,
                                          const sta::LibertyPort* port,
                                          const PowerPin& pin,
                                          const GatePowerInputs& gate) const
{
  const sta::LibertyPort* scene_port = scenePort(port);
  const sta::FuncExpr* func = port->function();
  struct Entry
  {
    const sta::InternalPower* pwr;
    const PowerPin* from_pin;
    float density;
    float duty;
  };
  std::vector<Entry> entries;
  PgDutySums duty_sums;
  for (const sta::InternalPower& pwr : scene_cell->internalPowers()) {
    // An entry without a related input has no weight.
    if (pwr.port() != scene_port || pwr.relatedPort() == nullptr) {
      continue;
    }
    const PowerPin* from_pin = findPin(gate, pwr.relatedPort()->name());
    const float density
        = from_pin != nullptr ? from_pin->activity.density : 0.0f;
    const float duty = relatedInputDuty(cell, func, &pwr, gate);
    pgDutySum(duty_sums, pwr.relatedPgPin()) += density * duty;
    entries.push_back({&pwr, from_pin, density, duty});
  }

  float internal = 0.0f;
  for (const Entry& entry : entries) {
    const float sum = pgDutySum(duty_sums, entry.pwr->relatedPgPin());
    if (entry.from_pin == nullptr || sum == 0.0f) {
      continue;
    }
    const bool keep
        = keepsTransition(scene_cell, entry.pwr->relatedPort(), scene_port);
    const float energy
        = tableEnergy(entry.pwr, entry.from_pin->slew, !keep, pin.load_cap);
    const float weight = entry.density * entry.duty / sum;
    internal += weight * energy * pin.activity.density;
  }
  return internal;
}

float ObjectivePower::tableEnergy(const sta::InternalPower* pwr,
                                  const RiseFallSlews& slews,
                                  const bool invert,
                                  const float load_cap) const
{
  float energy = 0.0f;
  int count = 0;
  for (const sta::RiseFall* rf : sta::RiseFall::range()) {
    const sta::RiseFall* in_rf = invert ? rf->opposite() : rf;
    const float slew = slews[in_rf->index()];
    // OpenSTA averages only the transitions with a finite slew.
    if (!sta::fuzzyInf(slew)) {
      energy += pwr->power(rf, pvt_, slew, load_cap);
      ++count;
    }
  }
  if (count > 0) {
    energy /= count;
  }
  return energy;
}

sta::LibertyCell* ObjectivePower::sceneCell(sta::LibertyCell* cell) const
{
  return scene_ != nullptr ? cell->sceneCell(scene_, sta::MinMax::max()) : cell;
}

const sta::LibertyPort* ObjectivePower::scenePort(
    const sta::LibertyPort* port) const
{
  return scene_ != nullptr ? port->scenePort(scene_, sta::MinMax::max()) : port;
}

// The related power pin's voltage, then the library's operating conditions.
float ObjectivePower::portVoltage(const sta::LibertyPort* port) const
{
  const sta::LibertyCell* cell = sceneCell(port->libertyCell());
  const sta::LibertyLibrary* library
      = (cell != nullptr ? cell : port->libertyCell())->libertyLibrary();
  const sta::LibertyPort* pg_port = port->relatedPowerPort();
  if (pg_port != nullptr) {
    float voltage = 0.0f;
    bool exists = false;
    library->supplyVoltage(pg_port->voltageName(), voltage, exists);
    if (exists) {
      return voltage;
    }
  }
  const sta::Pvt* pvt
      = pvt_ != nullptr ? pvt_ : library->defaultOperatingConditions();
  return pvt != nullptr ? pvt->voltage() : 0.0f;
}

GatePowerInputs sizingPowerInputs(const ObjectivePower& power,
                                  Resizer& resizer,
                                  const sta::Instance* inst,
                                  const sta::MinMax* min_max)
{
  // The leakage objective reads none of them.
  if (!power.totalPower()) {
    return {};
  }
  GatePowerInputs gate = power.gateInputs(inst);
  sta::dbSta* sta = resizer.sta();
  const sta::Scene* scene = sta->cmdScene();
  const RiseFallSlews target_slews{resizer.targetSlew(sta::RiseFall::rise()),
                                   resizer.targetSlew(sta::RiseFall::fall())};
  for (PowerPin& pin : gate.pins) {
    if (pin.port->direction()->isAnyInput()) {
      pin.slew = target_slews;
    }
    if (pin.port->direction()->isAnyOutput()) {
      const sta::Pin* sta_pin = sta->network()->findPin(inst, pin.port);
      pin.load_cap = sta->graphDelayCalc()->loadCap(sta_pin, scene, min_max);
    }
  }
  return gate;
}

}  // namespace rsz
