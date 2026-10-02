// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <array>
#include <unordered_map>
#include <vector>

#include "sta/Transition.hh"

namespace sta {
class dbSta;
class FuncExpr;
class InternalPower;
class Instance;
class LibertyCell;
class LibertyPort;
class MinMax;
class Network;
class Pin;
class Pvt;
class Scene;
}  // namespace sta

namespace rsz {

class Resizer;

using RiseFallSlews = std::array<float, sta::RiseFall::index_count>;

// Switching activity of one pin: transitions per second and the probability
// that the signal is high.
struct PinActivity
{
  float density = 0.0f;
  float duty = 0.0f;
};

// One signal pin of a gate, as the objective-power model reads it. A
// candidate cell's port is matched to it by name.
struct PowerPin
{
  // The port of the gate's current cell.
  const sta::LibertyPort* port = nullptr;
  PinActivity activity;
  // Input pins: the rise and fall input slews (seconds) the internal power
  // tables are read at.
  RiseFallSlews slew{};
  // Input pins: the density of the net that drives the pin and the supply
  // voltage of its driver. Both are 0 when a top-level input drives the net,
  // because OpenSTA reports no switching power for such a net.
  float fanin_density = 0.0f;
  float fanin_voltage = 0.0f;
  // Output pins: the load (farads) the internal power tables are read at.
  float load_cap = 0.0f;
};

// Everything the model needs about one gate, frozen on the main thread.
struct GatePowerInputs
{
  std::vector<PowerPin> pins;
};

// The power the global sizer minimizes for one gate and candidate cell.
//
// Under the leakage objective it is the candidate's leakage. Under the total
// objective it is the sum of
//   * the candidate's leakage;
//   * its internal power, from the Liberty internal_power tables with
//     OpenSTA's formulas (Power::findInputInternalPower and
//     Power::findOutputInternalPower);
//   * the switching power its input capacitance adds to its fanin nets,
//     sum over input pins of 0.5 * C_in * V^2 * density of the fanin net.
// The output net's switching power is left out: it is the same for every
// candidate of the gate.
//
// OpenSTA weights the entries by `when` conditions and related-input
// activities with BDD code that is not public. Here the same probabilities
// come from truth tables over independent input duties, as OpenSTA assumes.
// One difference remains: a register state (such as IQ) in an expression is
// an ordinary variable here, with the duty of the output pin whose function
// is that state, whereas OpenSTA uses the state's own activity for the whole
// sub-expression below it. Both agree when the expression is the state alone.
//
// OpenSTA discards its activities whenever a cell is replaced, so
// cacheActivities() reads them once, before the first swap. Swaps between
// equivalent cells keep every logic function, so they stay valid for the run.
//
// cacheActivities() and gateInputs() run on the main thread. The power
// functions read only their GatePowerInputs, the cached scene and operating
// conditions, and Liberty data, so the sweep's worker threads may call them.
class ObjectivePower
{
 public:
  // Selects the total objective. The default is the leakage objective.
  void setTotalPower(bool total) { total_power_ = total; }
  bool totalPower() const { return total_power_; }

  // MAIN THREAD. Reads OpenSTA's activity (density and duty) of every signal
  // pin of every leaf instance and keeps it until the next call. This covers
  // each gate's own pins and the driver pins of its fanin nets.
  void cacheActivities(sta::dbSta* sta, const sta::Scene* scene);
  // The cached activity of `pin`, or nullptr if it was not read.
  const PinActivity* activity(const sta::Pin* pin) const;

  // MAIN THREAD. The signal pins of `inst` with their cached activities and,
  // for input pins, the density and supply voltage of the net driving them.
  // Slews and loads are left at 0 for the caller to set.
  GatePowerInputs gateInputs(const sta::Instance* inst) const;

  // Objective power of `cell` placed at the gate described by `gate`, in
  // watts. `leakage` is the cell's leakage as the caller measures it; under
  // the leakage objective it is returned unchanged.
  float power(sta::LibertyCell* cell,
              float leakage,
              const GatePowerInputs& gate) const;
  // The two dynamic parts of power(), in watts.
  float internalPower(sta::LibertyCell* cell,
                      const GatePowerInputs& gate) const;
  float inputSwitchingPower(const sta::LibertyCell* cell,
                            const GatePowerInputs& gate) const;

 private:
  // MAIN THREAD. Sets the fanin density and voltage of input pin `pin` from
  // the first driver of its net that is a library cell's pin.
  void readFanin(const sta::Pin* pin, PowerPin& power_pin) const;
  float inputInternalPower(const sta::LibertyCell* cell,
                           const sta::LibertyCell* scene_cell,
                           const sta::LibertyPort* port,
                           const PowerPin& pin,
                           const GatePowerInputs& gate) const;
  float outputInternalPower(const sta::LibertyCell* cell,
                            const sta::LibertyCell* scene_cell,
                            const sta::LibertyPort* port,
                            const PowerPin& pin,
                            const GatePowerInputs& gate) const;
  // The average of the rise and fall table energies (joules) of `pwr`. The
  // slew of each output transition is read from `slews` at the same
  // transition, or at the opposite one when `invert` is set.
  float tableEnergy(const sta::InternalPower* pwr,
                    const RiseFallSlews& slews,
                    bool invert,
                    float load_cap) const;
  sta::LibertyCell* sceneCell(sta::LibertyCell* cell) const;
  const sta::LibertyPort* scenePort(const sta::LibertyPort* port) const;
  // Supply voltage of `port`, looked up as OpenSTA's Power::portVoltage does.
  float portVoltage(const sta::LibertyPort* port) const;

  bool total_power_ = false;
  sta::Network* network_ = nullptr;
  const sta::Scene* scene_ = nullptr;
  // The SDC operating conditions OpenSTA reads power tables at (may be null).
  const sta::Pvt* pvt_ = nullptr;
  std::unordered_map<const sta::Pin*, PinActivity> activities_;
};

// MAIN THREAD. The inputs of `inst` to `power`, read where the global sizer's
// delay term reads delays: each input pin at the target slew Resizer::gateDelay
// uses, each output pin at its current `min_max` load. The sizer's per-gate
// cost, its timing-weight anchor and its design totals all read objective
// power at these inputs. Empty under the leakage objective, which reads none
// of them.
GatePowerInputs sizingPowerInputs(const ObjectivePower& power,
                                  Resizer& resizer,
                                  const sta::Instance* inst,
                                  const sta::MinMax* min_max);

// Probability that `expr` is true when every port it reads is independently
// high with that port's duty in `gate`. A register's internal state takes the
// duty of the output pin whose function is that state. Without such an output
// it is taken as 0.5; OpenSTA derives it from the register's next-state
// function instead.
float exprDuty(const sta::FuncExpr* expr, const GatePowerInputs& gate);

// Probability that a change of `port` changes `expr`: the duty of the Boolean
// difference of `expr` with respect to `port`, with the other ports as in
// exprDuty. 0 when `expr` does not read `port`.
float exprDiffDuty(const sta::FuncExpr* expr,
                   const sta::LibertyPort* port,
                   const GatePowerInputs& gate);

}  // namespace rsz
