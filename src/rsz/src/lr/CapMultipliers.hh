// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <optional>
#include <vector>

#include "sta/GraphClass.hh"
#include "sta/NetworkClass.hh"

namespace rsz {

struct LrState;

// Lagrangian relaxation of the max-capacitance limits (relax_max_cap), after
// Livramento et al., DATE 2013. A gate's output pins get a multiplier beta
// when global sizing resizes the gate, or would if size_registers were on, and
// the pin's cell declares a Liberty max_capacitance. The Lagrangian (Eq. 3)
// gains
//   sum over those pins of beta * (load - limit).
// A candidate cell pays this term on its own output pins, at its own limit
// (Alg. 2 line 11), and on the driver pins of its fanin nets, at a load that
// includes its input capacitance (line 8). The term is signed: a pin below its
// limit earns a credit. On a pin with a beta, the sweep's max-capacitance
// filter does not reject candidates, and the post-sweep re-check does not
// revert moves, so a violation is priced rather than forbidden.
//
// The paper's circuit graph has only gates, primary inputs and primary
// outputs, and it prices the primary inputs. It does not mention registers; a
// register output can only be a primary input there, so registers are priced
// even when size_registers is off. Such a register only prices the candidates
// that load it. The fix pass cannot resize it (as in the paper, whose Alg. 3
// resizes gates only) and the re-check leaves its net, so a violation there
// lasts until beta makes the cells that load it smaller. The primary inputs
// themselves get no beta, since an input port has no Liberty limit, and
// neither do the outputs of the other gates global sizing does not resize
// (dont-touch cells, macros, the clock network unless include_clock_network):
// the sweep keeps its filter on the nets they drive.

// beta * (load - limit): one pin's term of Eq. 3.
float capBetaCost(float beta, float load, float limit);

// Alg. 1 line 15: beta * load / limit. Eq. 10's step size, beta / limit,
// turns the subgradient step beta + step * (load - limit) into this product,
// so beta grows while the pin is over its limit, shrinks while it is below it,
// and stays the same at the limit. A limit that is not positive leaves beta
// unchanged.
float updatedCapBeta(float beta, float load, float limit);

// The share of a median gate's power that an excess of the median limit costs
// at the first sweep. The term is signed, so with a large share every pin
// below its limit earns a credit that rewards a cell with a larger limit, and
// the first sweep upsizes gates for that credit alone. A small share keeps the
// term negligible until beta has grown on the pins that stay over their limit.
inline constexpr float kInitialCapBetaShare = 0.01f;

// The initial value of every beta (Alg. 1 line 5). The paper asks only for a
// positive vector. This value makes an excess of `median_limit` cost
// kInitialCapBetaShare times `median_power` once the candidate cost weights it
// by `timing_weight`:
//   beta0 = kInitialCapBetaShare * median_power
//           / (timing_weight * median_limit).
// 1 when an input is not positive or the quotient is not a positive finite
// number, as the timing weight falls back to 1.
float initialCapBeta(float median_power,
                     float timing_weight,
                     float median_limit);

// The multipliers of one run.
class CapMultipliers
{
 public:
  // Records the priced pins and sets each beta to initialCapBeta of the median
  // objective power of the gates global sizing may resize (over those whose
  // cell has leakage data), `timing_weight` and the median limit of the pins.
  // The set of pins is fixed for the rest of the run. Main thread only.
  void start(const LrState& state, float timing_weight);
  bool started() const { return started_; }

  // Alg. 1 line 15 on every priced pin, at its current load and the limit of
  // its current cell. Main thread only.
  void update(const LrState& state);

  // The beta of the pin whose driver vertex has id `vertex`, or nothing when
  // the pin has none.
  std::optional<float> beta(sta::VertexId vertex) const;

  // The priced pins whose load is above the limit of their current cell. Main
  // thread only.
  int countViolating(const LrState& state) const;

  int pricedPins() const { return static_cast<int>(pins_.size()); }
  float beta0() const { return beta0_; }
  int violatingAtStart() const { return violating_at_start_; }

 private:
  struct PricedPin
  {
    const sta::Pin* pin = nullptr;
    sta::VertexId vertex = 0;
  };

  // Records the output pins of `inst` that have a driver vertex and whose
  // cell declares a Liberty max capacitance, and appends their limits to
  // `limits`.
  void addPricedPins(const LrState& state,
                     const sta::Instance* inst,
                     std::vector<float>& limits);

  std::vector<PricedPin> pins_;
  // Indexed by vertex id; negative where the vertex has no beta.
  std::vector<float> beta_;
  float beta0_ = 0.0f;
  int violating_at_start_ = 0;
  bool started_ = false;
};

}  // namespace rsz
