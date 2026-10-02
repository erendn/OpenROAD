// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "ViolationRepair.hh"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "ElectricalModel.hh"
#include "InitSelect.hh"
#include "LRSubproblem.hh"
#include "SweepEngine.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
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
#include "sta/Transition.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

namespace {

// True when any output pin of `inst` drives a clock.
bool drivesClock(const sta::Network* network,
                 sta::dbSta* sta,
                 const sta::Instance* inst)
{
  std::unique_ptr<sta::InstancePinIterator> port_iter(
      network->pinIterator(inst));
  while (port_iter->hasNext()) {
    sta::Pin* pin = port_iter->next();
    if (network->direction(pin)->isOutput()
        && sta->isClock(pin, sta->cmdMode())) {
      return true;
    }
  }
  return false;
}

// Capture the gate's output pins from the current timing. Empty when the gate
// has no output pin a walk can check.
std::vector<OutputPinSnap> snapOutputs(const LrState& state,
                                       const sta::Instance* inst)
{
  sta::Network* network = state.network;
  sta::Graph* graph = state.graph;
  sta::dbSta* sta = state.sta;
  const sta::Scene* scene = sta->cmdScene();
  const sta::MinMax* max_mm = state.max;

  std::vector<OutputPinSnap> outputs;
  std::unique_ptr<sta::InstancePinIterator> pit(network->pinIterator(inst));
  while (pit->hasNext()) {
    sta::Pin* pin = pit->next();
    if (!network->direction(pin)->isOutput()) {
      continue;
    }
    const sta::LibertyPort* port = network->libertyPort(pin);
    if (port == nullptr) {
      continue;
    }
    OutputPinSnap o;
    o.pin = pin;
    o.port = port;
    o.measured_load = sta->graphDelayCalc()->loadCap(pin, scene, max_mm);
    o.load = o.measured_load;
    o.drive_res = port->driveResistance();
    sta::Vertex* load_v = graph->pinLoadVertex(pin);
    o.slew = (load_v != nullptr)
                 ? sta::delayAsFloat(sta->slew(load_v,
                                               sta::RiseFallBoth::riseFall(),
                                               sta->scenes(),
                                               max_mm))
                 : 0.0f;
    outputs.push_back(o);
  }
  return outputs;
}

// Re-read the gate's output loads when the walk reaches it, so that the
// repairs already made to its fanout are included.
void refreshLoads(const LrState& state, std::vector<OutputPinSnap>& outputs)
{
  sta::dbSta* sta = state.sta;
  const sta::Scene* scene = sta->cmdScene();
  for (OutputPinSnap& o : outputs) {
    o.load = sta->graphDelayCalc()->loadCap(o.pin, scene, state.max);
  }
}

// How far `cell` exceeds its max capacitance at the loads in `outputs`, summed
// over the pins (see CapFixOption). Infinite when `cell` lacks one of the pins.
float maxCapExcess(const LrState& state,
                   const std::vector<OutputPinSnap>& outputs,
                   const sta::LibertyCell* cell)
{
  float excess = 0.0f;
  for (const OutputPinSnap& o : outputs) {
    const sta::LibertyPort* port = cell->findLibertyPort(o.port->name());
    if (port == nullptr) {
      return std::numeric_limits<float>::infinity();
    }
    excess += outputMaxCapExcess(port, o.load, state.max);
  }
  return excess;
}

// The snapshot inputs the fix pass prices cells with: the multipliers, beta
// included under relax_max_cap, and the sweep's arc pricing. The terms that
// price a gate's neighbours and the downsize guard are off, since the fix pass
// reads neither.
LRSubproblem::SnapshotInputs capFixInputs(const LrState& state)
{
  const GlobalSizingConfig& cfg = *state.config;
  LRSubproblem::SnapshotInputs in;
  in.lambda = state.lambda.data();
  in.lambda_size = static_cast<int>(state.lambda.size());
  in.prev_delay = state.prev_delay.data();
  in.prev_delay_size = static_cast<int>(state.prev_delay.size());
  in.cost
      = {.upstream_load = false,
         .fanout_slew = false,
         .global_phi = false,
         .delta_delay = cfg.cost_delta_delay,
         .per_arc = cfg.timing_cost == GlobalSizingConfig::TimingCost::kPerArc};
  in.include_clock_network = cfg.include_clock_network;
  in.size_registers = cfg.size_registers;
  in.objective_power = &state.objective_power;
  in.cap_multipliers = cfg.relax_max_cap ? &state.cap_multipliers : nullptr;
  in.guard = GlobalSizingConfig::DownsizeGuard::kNone;
  return in;
}

// One gate of a walk and its output pins.
struct WalkGate
{
  sta::Instance* inst = nullptr;
  std::vector<OutputPinSnap> outputs;
};

// `instances` in topological order: the same key and comparator as the
// Gauss-Seidel engine's forward_topo and reverse_topo traversals (minimum
// output-vertex level, stable instance id as tie-break). Gates with no output
// vertex keep the sentinel key; they have no output pin to check anyway.
std::vector<TraversalEntry> topoOrder(
    const LrState& state,
    const std::vector<sta::Instance*>& instances,
    const GlobalSizingConfig::Traversal traversal)
{
  sta::Network* network = state.network;
  std::vector<TraversalEntry> entries;
  entries.reserve(instances.size());
  for (sta::Instance* inst : instances) {
    TraversalEntry e;
    e.key = topoTraversalKey(network, state.graph, inst);
    e.tiebreak = static_cast<uint64_t>(network->id(inst));
    e.inst = inst;
    entries.push_back(e);
  }
  orderTraversal(entries, traversal);
  return entries;
}

// `instances` in reverse-topological order, each with its output pins
// captured before any repair.
std::vector<WalkGate> reverseTopoGates(
    const LrState& state,
    const std::vector<sta::Instance*>& instances)
{
  const std::vector<TraversalEntry> entries = topoOrder(
      state, instances, GlobalSizingConfig::Traversal::kReverseTopo);

  // Loads are re-read during the walk (that is how a driver sees its repaired
  // fanout); only the measured slew, which is not updated per gate, is taken
  // from here.
  std::vector<WalkGate> gates;
  gates.reserve(entries.size());
  for (const TraversalEntry& e : entries) {
    gates.push_back({.inst = e.inst, .outputs = snapOutputs(state, e.inst)});
  }
  return gates;
}

// One gate of a walk: if its current cell violates `limits` at the loads in
// `outputs`, install the cell `choose` returns and count it in `stats`.
void repairGate(LrState& state,
                sta::Instance* inst,
                const std::vector<OutputPinSnap>& outputs,
                const RepairedLimits limits,
                const RepairChoice& choose,
                RepairWalkStats& stats)
{
  sta::LibertyCell* cur_cell = state.network->libertyCell(inst);
  if (cur_cell == nullptr
      || clearsViolations(state, outputs, cur_cell, limits)) {
    return;
  }
  ++stats.violating;
  sta::LibertyCell* choice = choose(inst, cur_cell, outputs);
  if (choice == cur_cell || !state.resizer->replaceCell(inst, choice)) {
    return;
  }
  ++stats.replaced;
  if (clearsViolations(state, outputs, choice, limits)) {
    ++stats.cleared;
  }
}

// How far `cell` exceeds its slew limit at the loads in `outputs`, summed over
// the pins, with the slew estimated as in clearsViolations. Infinite when
// `cell` lacks one of the pins.
float maxSlewExcess(const LrState& state,
                    const std::vector<OutputPinSnap>& outputs,
                    const sta::LibertyCell* cell)
{
  sta::dbSta* sta = state.sta;
  const sta::Scene* scene = sta->cmdScene();
  float excess = 0.0f;
  for (const OutputPinSnap& o : outputs) {
    const sta::LibertyPort* port = cell->findLibertyPort(o.port->name());
    if (port == nullptr) {
      return std::numeric_limits<float>::infinity();
    }
    excess += outputMaxSlewExcess(
        sta,
        port,
        outputSlewFactor(o.slew, o.drive_res, o.measured_load),
        o.load,
        scene,
        state.max);
  }
  return excess;
}

// The Vt flavor of `cell`, as LRSubproblem::vthFlavorKey defines it.
int vthFlavor(const LrState& state, const sta::LibertyCell* cell)
{
  odb::dbMaster* master = state.db_network->staToDb(cell);
  return master != nullptr ? state.resizer->cellVTType(master).vt_index : 0;
}

// The cells the slew fix pass may install on a gate whose cell is `current`,
// ranked as the init pass ranks a group: `current` first, then the members of
// its swappable group with its Vt flavor.
struct SlewFixGroup
{
  std::vector<sta::LibertyCell*> cells;
  std::vector<InitCandidate> candidates;
};

SlewFixGroup slewFixGroup(const LrState& state, sta::LibertyCell* current)
{
  Resizer& resizer = *state.resizer;
  SlewFixGroup group;
  group.cells.push_back(current);
  const int vt = vthFlavor(state, current);
  for (sta::LibertyCell* cell : resizer.getSwappableCells(current)) {
    if (cell != current && vthFlavor(state, cell) == vt) {
      group.cells.push_back(cell);
    }
  }
  for (sta::LibertyCell* cell : group.cells) {
    const std::optional<float> leakage = resizer.cellLeakage(cell);
    group.candidates.push_back(
        {.has_leakage = leakage.has_value(),
         .leakage = leakage.value_or(0.0f),
         .drive_resistance = resizer.cellDriveResistance(cell),
         .name = cell->name()});
  }
  return group;
}

// The cells the fix pass may install on the gate in `snap`, with their
// options in the same order. The current cell comes first, so it wins a full
// tie. A candidate that lacks one of the output pins is left out.
struct CapFixChoices
{
  std::vector<sta::LibertyCell*> cells;
  std::vector<CapFixOption> options;
};

CapFixChoices capFixChoices(const LrState& state,
                            LRSubproblem& subproblem,
                            const LRSubproblem::GateSnapshot& snap,
                            const std::vector<OutputPinSnap>& outputs,
                            const float timing_weight)
{
  // The walk runs on the main thread, so the shared calculator can be used.
  sta::ArcDelayCalc* arc_delay_calc = state.sta->arcDelayCalc();
  CapFixChoices choices;
  auto add = [&](sta::LibertyCell* cell, const float leakage) {
    const float excess = maxCapExcess(state, outputs, cell);
    if (excess == std::numeric_limits<float>::infinity()) {
      return;
    }
    choices.cells.push_back(cell);
    choices.options.push_back(
        {.excess = excess,
         .cost = subproblem.ownCost(
             snap, cell, leakage, timing_weight, arc_delay_calc)});
  };
  add(snap.cur_cell, snap.cur_leakage);
  const int vt = subproblem.vthFlavorKey(snap.cur_cell);
  for (const LRSubproblem::Candidate& cand : snap.candidates) {
    if (subproblem.vthFlavorKey(cand.cell) == vt) {
      add(cand.cell, cand.leakage);
    }
  }
  return choices;
}

}  // namespace

std::vector<sta::Instance*> sizableGates(const LrState& state)
{
  std::vector<sta::Instance*> gates;
  std::unique_ptr<sta::LeafInstanceIterator> iit(
      state.network->leafInstanceIterator());
  while (iit->hasNext()) {
    sta::Instance* inst = iit->next();
    if (maySizeGate(state, inst)) {
      gates.push_back(inst);
    }
  }
  return gates;
}

bool maySizeGate(const LrState& state, const sta::Instance* inst)
{
  return maySizeGateWithRegistersSized(state, inst)
         && (state.config->size_registers
             || !state.network->libertyCell(inst)->isSequential());
}

bool maySizeGateWithRegistersSized(const LrState& state,
                                   const sta::Instance* inst)
{
  if (!state.resizer->isEditableLogicStdCell(inst)) {
    return false;
  }
  return state.config->include_clock_network
         || !drivesClock(state.network, state.sta, inst);
}

bool clearsViolations(const LrState& state,
                      const std::vector<OutputPinSnap>& outputs,
                      const sta::LibertyCell* cell,
                      const RepairedLimits limits)
{
  sta::dbSta* sta = state.sta;
  const sta::Scene* scene = sta->cmdScene();
  const sta::MinMax* max_mm = state.max;
  for (const OutputPinSnap& o : outputs) {
    const sta::LibertyPort* cand = cell->findLibertyPort(o.port->name());
    if (cand == nullptr) {
      return false;  // candidate missing this output port - not usable
    }
    if (limits != RepairedLimits::kMaxSlew
        && checkOutputMaxCap(cand, o.load, max_mm)) {
      return false;
    }
    if (limits != RepairedLimits::kMaxCap
        && checkOutputMaxSlew(
            sta,
            cand,
            outputSlewFactor(o.slew, o.drive_res, o.measured_load),
            o.load,
            scene,
            max_mm)) {
      return false;
    }
  }
  return true;
}

RepairWalkStats repairInReverseTopoOrder(
    LrState& state,
    const std::vector<sta::Instance*>& instances,
    const RepairedLimits limits,
    const RepairChoice& choose)
{
  RepairWalkStats stats;
  for (WalkGate& gate : reverseTopoGates(state, instances)) {
    std::vector<OutputPinSnap>& outs = gate.outputs;
    if (outs.empty()) {
      continue;
    }
    refreshLoads(state, outs);
    repairGate(state, gate.inst, outs, limits, choose, stats);
  }

  if (stats.replaced > 0) {
    // The callers' next steps read slacks, so leave timing up to date.
    state.resizer->updateParasiticsAndTiming();
  }
  return stats;
}

RepairWalkStats repairInForwardTopoOrder(
    LrState& state,
    const std::vector<sta::Instance*>& instances,
    const RepairedLimits limits,
    const RepairChoice& choose)
{
  RepairWalkStats stats;
  for (const TraversalEntry& e : topoOrder(
           state, instances, GlobalSizingConfig::Traversal::kForwardTopo)) {
    const std::vector<OutputPinSnap> outs = snapOutputs(state, e.inst);
    if (!outs.empty()) {
      repairGate(state, e.inst, outs, limits, choose, stats);
    }
  }

  if (stats.replaced > 0) {
    state.resizer->updateParasiticsAndTiming();
  }
  return stats;
}

size_t selectCapFixOption(const std::vector<CapFixOption>& options)
{
  size_t best = 0;
  for (size_t i = 1; i < options.size(); ++i) {
    const CapFixOption& o = options[i];
    const CapFixOption& b = options[best];
    // Lexicographic on (excess, cost): every option that clears has excess 0,
    // so if any clears, the cheapest of those wins.
    if (o.excess < b.excess || (o.excess == b.excess && o.cost < b.cost)) {
      best = i;
    }
  }
  return best;
}

RepairWalkStats fixMaxCapViolations(LrState& state,
                                    LRSubproblem& subproblem,
                                    const float timing_weight)
{
  const LRSubproblem::SnapshotInputs in = capFixInputs(state);
  auto choose = [&](sta::Instance* inst,
                    sta::LibertyCell* current,
                    const std::vector<OutputPinSnap>& outputs) {
    LRSubproblem::GateSnapshot snap;
    if (!subproblem.snapshot(inst, in, snap)) {
      return current;
    }
    const CapFixChoices choices
        = capFixChoices(state, subproblem, snap, outputs, timing_weight);
    const size_t pick = selectCapFixOption(choices.options);
    debugPrint(state.logger,
               RSZ,
               "global_sizing",
               5,
               "FIX  {}: {} -> {} (max-cap excess {:.3g} -> {:.3g} F)",
               state.network->pathName(inst),
               current->name(),
               choices.cells[pick]->name(),
               choices.options[0].excess,
               choices.options[pick].excess);
    return choices.cells[pick];
  };
  const RepairWalkStats walk = repairInReverseTopoOrder(
      state, sizableGates(state), RepairedLimits::kMaxCap, choose);
  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR cap fix pass: {} gate(s) over max capacitance, {} resized "
             "({} cleared)",
             walk.violating,
             walk.replaced,
             walk.cleared);
  return walk;
}

RepairWalkStats fixMaxSlewViolations(
    LrState& state,
    const std::vector<sta::Instance*>& instances)
{
  Resizer& resizer = *state.resizer;
  // replacementPreservesMaxCap reads OpenSTA's capacitance checks.
  state.sta->checkCapacitancesPreamble(state.sta->scenes());
  auto choose = [&](sta::Instance* inst,
                    sta::LibertyCell* current,
                    const std::vector<OutputPinSnap>& outputs) {
    const SlewFixGroup group = slewFixGroup(state, current);
    const size_t pick = selectSlewFixUpsize(
        group.candidates,
        /*current=*/0,
        maxSlewExcess(state, outputs, current),
        [&](const size_t i) -> std::optional<float> {
          if (!resizer.replacementPreservesMaxCap(inst, group.cells[i])) {
            return std::nullopt;
          }
          return maxSlewExcess(state, outputs, group.cells[i]);
        });
    debugPrint(state.logger,
               RSZ,
               "global_sizing",
               5,
               "SLEW {}: {} -> {}",
               state.network->pathName(inst),
               current->name(),
               group.cells[pick]->name());
    return group.cells[pick];
  };
  return repairInForwardTopoOrder(
      state, instances, RepairedLimits::kMaxSlew, choose);
}

}  // namespace rsz
