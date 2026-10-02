// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "LrState.hh"

namespace sta {
class Instance;
class LibertyCell;
class LibertyPort;
class Pin;
}  // namespace sta

namespace rsz {

class LRSubproblem;

// Repair of electrical violations by resizing gates in topological order.
// Three passes share the walk:
//   * the repair step of init_mode = min_size_fixviol (InitPass.hh), which
//     fixes max capacitance and max slew once, after the minimum-size reset,
//     and of min_size_fixcap, which fixes max capacitance only;
//   * the max-capacitance fix pass (cap_fix_pass), which runs after every
//     sweep (Livramento et al., DATE 2013, Alg. 3);
//   * the slew fix pass (slew_fix_pass), which runs once before the first
//     iteration (Sharma et al., ICCAD 2015, Sec. III-A).
// The capacitance repairs go from outputs to inputs because resizing a gate
// changes its input capacitance, which loads its drivers. Visiting sinks first
// means each driver is checked against the load of its already repaired
// fanout. The slew repair goes from inputs to outputs because a gate's output
// slew depends on its input slew. Visiting drivers first means each gate is
// checked against the input slew of its already repaired drivers.

// Whether global sizing may resize `inst`. The clock-network and register
// rules match the sweep's eligibility filter (LRSubproblem::snapshot), so the
// init pass, the repairs and the sweep size the same gates.
bool maySizeGate(const LrState& state, const sta::Instance* inst);

// maySizeGate without the register switch: whether global sizing would resize
// `inst` if size_registers were on.
bool maySizeGateWithRegistersSized(const LrState& state,
                                   const sta::Instance* inst);

// The gates global sizing may resize, in leaf-instance order.
std::vector<sta::Instance*> sizableGates(const LrState& state);

// The limits a walk repairs on a gate's output pins.
enum class RepairedLimits
{
  kMaxCapAndSlew,
  kMaxCap,
  kMaxSlew
};

// One output pin of a gate, captured from the timing before the walk starts
// (reverse walk) or when the walk reaches the gate (forward walk). `slew`,
// `measured_load` and `drive_res` come from that timing and do not change;
// `load` is refreshed when the walk reaches the gate, so a driver sees the
// fanout the walk has already repaired.
struct OutputPinSnap
{
  sta::Pin* pin = nullptr;
  const sta::LibertyPort* port = nullptr;
  float slew = 0.0f;
  float measured_load = 0.0f;
  float drive_res = 0.0f;
  float load = 0.0f;
};

// Would `cell` leave this gate's output pins within `limits`, at the loads in
// `outputs`? False when `cell` lacks one of the pins.
//
// Uses the same model as the sweep's candidate filter, through the shared
// lr/ElectricalModel.hh: the Liberty cap limit against the live load, and a
// slew estimate linear in drive resistance, calibrated at the slew measured
// before the walk. Matching the sweep matters: a stricter rule here would
// resize gates the sweep considers fine, and a looser one would leave gates
// the sweep then refuses to touch. As a consequence, a max_capacitance set
// only in SDC is not seen here; the post-sweep re-check (CapRecheck.hh)
// handles SDC limits.
bool clearsViolations(const LrState& state,
                      const std::vector<OutputPinSnap>& outputs,
                      const sta::LibertyCell* cell,
                      RepairedLimits limits);

// The cell a walk installs on a gate whose current cell violates: `current`
// itself to leave the gate as it is. `outputs` holds the gate's output pins at
// their live loads.
using RepairChoice = std::function<sta::LibertyCell*(
    sta::Instance* inst,
    sta::LibertyCell* current,
    const std::vector<OutputPinSnap>& outputs)>;

// What a walk did, or several walks summed.
struct RepairWalkStats
{
  // Gates whose current cell violated `limits` when the walk reached them.
  int violating = 0;
  // Gates the walk resized.
  int replaced = 0;
  // Resized gates whose new cell is within `limits`.
  int cleared = 0;

  RepairWalkStats& operator+=(const RepairWalkStats& other)
  {
    violating += other.violating;
    replaced += other.replaced;
    cleared += other.cleared;
    return *this;
  }
};

// Visit `instances` from outputs to inputs. For each gate whose current cell
// violates `limits` on its own output pins, install the cell `choose` returns.
// A gate's output load does not depend on its own cell (its cell sets its
// input pin caps, which load its drivers), so the loads are read once per gate,
// before `choose`. Each gate's slew is the one measured before the walk.
// Deterministic. Refreshes parasitics and timing once at the end if any cell
// changed, so the caller sees up-to-date slacks.
RepairWalkStats repairInReverseTopoOrder(
    LrState& state,
    const std::vector<sta::Instance*>& instances,
    RepairedLimits limits,
    const RepairChoice& choose);

// Visit `instances` from inputs to outputs, in the order of the Gauss-Seidel
// engine's forward_topo traversal, otherwise as repairInReverseTopoOrder. A
// gate's output pins, slew included, are read when the walk reaches it. Reading
// a slew makes OpenSTA recompute the delays that earlier resizes invalidated,
// so the gate's slew reflects the drivers the walk has already resized. As in
// the reverse walk, parasitics are re-estimated only at the end: refreshes
// parasitics and timing once if any cell changed.
RepairWalkStats repairInForwardTopoOrder(
    LrState& state,
    const std::vector<sta::Instance*>& instances,
    RepairedLimits limits,
    const RepairChoice& choose);

// One cell the max-capacitance fix pass may install on a violating gate.
struct CapFixOption
{
  // How far the cell would exceed its max capacitance at the gate's output
  // loads, summed over the output pins, in farads. 0 when it is within every
  // limit.
  float excess = 0.0f;
  // The cell's LR cost at the gate (see fixMaxCapViolations).
  float cost = 0.0f;
};

// Pure function (no STA): the option the fix pass installs.
//
// Among the options that clear the violation, the one with the lowest cost,
// as in Livramento et al., Alg. 3 lines 5-13. When none clears it, the one
// with the smallest excess, and among equal excesses the lowest cost. The
// paper's pseudocode accepts only a width that clears the violation, so this
// case is undefined there; its prose says that the cost's violation term keeps
// the choice meaningful when a violation cannot be fixed in the current
// iteration. The cell closest to its limit is the nearest equivalent without
// that term. With it (relax_max_cap), the excess still comes first, so the
// term only ranks options of equal excess.
//
// options[0] is the gate's current cell. Ties keep the earlier option, so the
// gate keeps its cell unless another option is strictly better. Returns 0 for
// an empty list.
size_t selectCapFixOption(const std::vector<CapFixOption>& options);

// The max-capacitance fix pass (cap_fix_pass), after Livramento et al.,
// DATE 2013, Alg. 3: walks the gates global sizing may resize and resizes each
// one whose output load exceeds its Liberty max capacitance, as
// selectCapFixOption picks. The options are the current cell and the members
// of the gate's swappable group with its Vt flavor (Alg. 3 changes widths
// only); the group is the one the sweep uses, which the resizer limits
// relative to the current cell, so a gate far over its limit can take several
// passes to reach the largest cell of its kind. An option's cost is
//   objective power + timing_weight * sum(lambda * d)
// over the gate's own arcs at its output load, priced as the sweep prices them
// (GlobalSizingConfig::timing_cost): Alg. 3 line 6. Under relax_max_cap it
// adds timing_weight * beta * (load - limit) over the gate's output pins, at
// the option's own limit (line 7, LRSubproblem::capBetaOwnCost).
//
// As in the paper, a larger input capacitance is not checked against the
// fanin nets; the walk visits the drivers later. A driver global sizing may
// not resize (a primary input, a dont-touch cell, a clock-network driver, or a
// register when size_registers is off) can be left over its limit.
//
// `subproblem` prices the cells; the multipliers in `state` must cover the
// live timing edges (LrState::refreshLiveEdges). Main thread only.
RepairWalkStats fixMaxCapViolations(LrState& state,
                                    LRSubproblem& subproblem,
                                    float timing_weight);

// The slew fix pass (slew_fix_pass), after Sharma et al., ICCAD 2015,
// Sec. III-A: walks `instances` from inputs to outputs and upsizes each gate
// whose output slew exceeds its limit, as selectSlewFixUpsize (InitSelect.hh)
// picks. The options are the members of the gate's swappable group with its Vt
// flavor that rank above it by leakage. An option may not be installed if its
// larger input capacitance would push a gate that drives one of its inputs
// over its max capacitance, or add load to a driver already over it
// (Resizer::replacementPreservesMaxCap, the sweep's own input-side check). The
// paper keeps both limits met throughout (Sec. II).
//
// A cell's output slew is estimated as clearsViolations does. The walk does
// not revisit a gate: a later upsize in its fanout can raise its load, within
// its max capacitance, but past its slew limit. Main thread only.
RepairWalkStats fixMaxSlewViolations(
    LrState& state,
    const std::vector<sta::Instance*>& instances);

}  // namespace rsz
