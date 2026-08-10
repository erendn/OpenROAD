// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "InitPass.hh"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ElectricalModel.hh"
#include "InitSelect.hh"
#include "SweepEngine.hh"
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
#include "sta/Transition.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

using InitMode = GlobalSizingConfig::InitMode;

InitModePass::Group& InitModePass::group(LrState& state,
                                         sta::LibertyCell* current_cell,
                                         GroupCache& cache) const
{
  const GroupCache::iterator cached = cache.find(current_cell);
  if (cached != cache.end()) {
    return cached->second;
  }

  Resizer& resizer = *state.resizer;
  Group group;
  // The as-given cell first: the modes are defined relative to it, and the
  // deterministic scan resolves a full tie in its favour by starting there.
  group.cells.push_back(current_cell);
  for (sta::LibertyCell* candidate : resizer.getSwappableCells(current_cell)) {
    if (candidate != current_cell) {
      group.cells.push_back(candidate);
    }
  }
  group.candidates.reserve(group.cells.size());
  for (sta::LibertyCell* cell : group.cells) {
    const std::optional<float> leakage = resizer.cellLeakage(cell);
    InitCandidate candidate;
    candidate.has_leakage = leakage.has_value();
    candidate.leakage = leakage.value_or(0.0f);
    candidate.drive_resistance = resizer.cellDriveResistance(cell);
    candidate.name = cell->name();
    group.candidates.push_back(candidate);
  }
  return cache.emplace(current_cell, std::move(group)).first->second;
}

sta::LibertyCell* InitModePass::selectCell(LrState& state,
                                           sta::Instance* inst,
                                           sta::LibertyCell* current_cell,
                                           const InitMode mode,
                                           GroupCache& cache) const
{
  Group& g = group(state, current_cell, cache);
  if (mode == InitMode::kRandom) {
    // Per instance, so nothing can be memoized - but the draw is keyed by the
    // instance's path name, not by visit order, so the result does not depend
    // on the iteration order or the thread count (see initDraw).
    const std::string inst_name = state.network->pathName(inst);
    const uint64_t draw = initDraw(state.config->init_seed, inst_name);
    return g.cells[selectInitCandidate(g.candidates, mode, draw)];
  }
  if (g.deterministic_choice == nullptr) {
    g.deterministic_choice
        = g.cells[selectInitCandidate(g.candidates, mode, /*draw=*/0)];
  }
  return g.deterministic_choice;
}

int InitModePass::run(LrState& state)
{
  const InitMode mode = state.config->init_mode;
  const bool include_clock_network = state.config->include_clock_network;
  Resizer& resizer = *state.resizer;
  sta::Network* network = state.network;
  sta::dbSta* sta = state.sta;
  utl::Logger* logger = state.logger;

  if (mode == InitMode::kAsGiven) {
    return 0;
  }

  // min_size_fixviol IS min_size plus the reverse-topological electrical repair
  // below, so the selector runs the min_size rule and the repair runs after.
  const bool fixviol = (mode == InitMode::kMinSizeFixviol);
  const InitMode select_mode = fixviol ? InitMode::kMinSize : mode;

  std::string target;
  switch (select_mode) {
    case InitMode::kMinSize:
      target = "lowest-leakage member";
      break;
    case InitMode::kMaxSize:
      target = "highest-leakage member";
      break;
    case InitMode::kAverage:
      target = "lower-median member";
      break;
    case InitMode::kRandom:
      target = "uniform draw (init_seed="
               + std::to_string(state.config->init_seed) + ")";
      break;
    case InitMode::kAsGiven:
    case InitMode::kMinSizeFixviol:
      // Unreachable: as_given returned above and min_size_fixviol was mapped to
      // min_size just above.
      return 0;
  }
  logger->info(RSZ,
               416,
               "GLOBAL_SIZING: init_mode={} - replacing every editable "
               "instance with the {} of its leakage-ranked swappable group.",
               toString(mode),
               target);

  int replacements = 0;
  GroupCache cache;
  // The gates the pass is allowed to touch, kept so the repair walk operates on
  // exactly the population the swap did (the two must not disagree).
  std::vector<sta::Instance*> editable;
  std::unique_ptr<sta::LeafInstanceIterator> iit(
      network->leafInstanceIterator());
  while (iit->hasNext()) {
    sta::Instance* inst = iit->next();
    if (!resizer.isEditableLogicStdCell(inst)) {
      continue;
    }
    // Clock-network exclusion, same rule as the sweep path's eligibility filter
    // (LRSubproblem::snapshot): drop the instance when any of its output pins
    // drives a clock. The two must agree - the init pass and the sweep size the
    // same gate population - so this mirrors snapshot()'s test exactly.
    bool is_clock = false;
    if (!include_clock_network) {
      std::unique_ptr<sta::InstancePinIterator> port_iter(
          network->pinIterator(inst));
      while (port_iter->hasNext()) {
        sta::Pin* pin = port_iter->next();
        if (network->direction(pin)->isOutput()
            && sta->isClock(pin, sta->cmdMode())) {
          is_clock = true;
          break;
        }
      }
    }
    if (is_clock) {
      continue;
    }
    editable.push_back(inst);

    sta::LibertyCell* current_cell = network->libertyCell(inst);
    sta::LibertyCell* replacement
        = selectCell(state, inst, current_cell, select_mode, cache);
    if (replacement != current_cell && resizer.replaceCell(inst, replacement)) {
      ++replacements;
    }
  }

  // The initial solution is applied into the outer journal by the driver; seed
  // / project / computeAutoTimingWeight need fresh slacks, so refresh
  // parasitics + required times here. The repair pass needs them too - it reads
  // measured slews and graph levels - so it forces the refresh even on the
  // degenerate run where the min-size reset moved nothing.
  if (replacements > 0 || fixviol) {
    resizer.updateParasiticsAndTiming();
  }
  logger->info(RSZ,
               415,
               "GLOBAL_SIZING: init pass replaced {}/{} editable instances.",
               replacements,
               editable.size());
  if (fixviol) {
    replacements += repairViolations(state, editable, cache);
  }
  return replacements;
}

namespace {

// One output pin of a gate, frozen right after the min-size reset's timing
// update. `slew` / `measured_load` / `drive_res` are that measurement and never
// change; `load` is refreshed at the moment the walk reaches the gate, so a
// driver sees the fanout the walk has already repaired.
struct OutputPinSnap
{
  sta::Pin* pin = nullptr;
  const sta::LibertyPort* port = nullptr;
  float slew = 0.0f;
  float measured_load = 0.0f;
  float drive_res = 0.0f;
  float load = 0.0f;
};

// Would `cell` leave this gate free of max-cap and max-slew violations on its
// own output pins?
//
// The model is the sweep's own candidate filter, called through the shared
// lr/ElectricalModel.hh so the two cannot drift: the Liberty cap limit against
// the live load, and a drive-resistance-linear slew estimate calibrated at the
// measured post-reset slew. Agreeing with the sweep that follows is the point -
// a repair pass judged by a stricter rule would hand the sweep gates it
// considers fine, and one judged by a looser rule would leave gates the sweep
// then refuses to touch. It does mean a max_capacitance set only by SDC is
// outside this pass's view; the post-sweep re-check (CapRecheck.hh) is the
// SDC-aware half.
//
// `outputs` carries the gate's live loads, refreshed once by refreshLoads()
// before the candidate walk. A gate's own output load is invariant across its
// OWN cell choice - swapping this cell moves its INPUT pin caps, which load its
// DRIVERS - so re-querying it per candidate would be the same answer at the
// price of a connectedCap plus parasitic traversal per group member.
bool clearsViolations(LrState& state,
                      const std::vector<OutputPinSnap>& outputs,
                      sta::LibertyCell* cell)
{
  sta::dbSta* sta = state.sta;
  const sta::Scene* scene = sta->cmdScene();
  const sta::MinMax* max_mm = state.max;
  for (const OutputPinSnap& o : outputs) {
    sta::LibertyPort* cand = cell->findLibertyPort(o.port->name());
    if (cand == nullptr) {
      return false;  // candidate missing this output port - not usable
    }
    if (checkOutputMaxCap(cand, o.load, max_mm)) {
      return false;
    }
    if (checkOutputMaxSlew(
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

// Freeze the gate's output pins against the post-reset timing. Empty when the
// gate drives nothing this pass can judge.
std::vector<OutputPinSnap> snapOutputs(LrState& state, sta::Instance* inst)
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

// Re-read the gate's output loads, once, at the moment the walk reaches it -
// which is where the repairs its fanout has already received show up.
void refreshLoads(LrState& state, std::vector<OutputPinSnap>& outputs)
{
  sta::dbSta* sta = state.sta;
  const sta::Scene* scene = sta->cmdScene();
  for (OutputPinSnap& o : outputs) {
    o.load = sta->graphDelayCalc()->loadCap(o.pin, scene, state.max);
  }
}

}  // namespace

int InitModePass::repairViolations(LrState& state,
                                   const std::vector<sta::Instance*>& instances,
                                   GroupCache& cache) const
{
  Resizer& resizer = *state.resizer;
  sta::Network* network = state.network;
  sta::Graph* graph = state.graph;
  utl::Logger* logger = state.logger;

  // Reverse-topological visit order, built with the SAME key and comparator the
  // Gauss-Seidel engine's `-traversal reverse_topo` uses (min output-vertex
  // level, stable instance id as tie-break), so "outputs toward inputs" means
  // one thing in this codebase. Gates with no output vertex keep the sentinel
  // key; snapOutputs drops them anyway.
  std::vector<TraversalEntry> entries;
  entries.reserve(instances.size());
  for (sta::Instance* inst : instances) {
    TraversalEntry e;
    e.key = topoTraversalKey(network, graph, inst);
    e.tiebreak = static_cast<uint64_t>(network->id(inst));
    e.inst = inst;
    entries.push_back(e);
  }
  orderTraversal(entries, GlobalSizingConfig::Traversal::kReverseTopo);

  // Every gate's electrical context is frozen against the post-reset STA BEFORE
  // any repair. The load half is re-read live during the walk (that is how a
  // driver sees its repaired fanout); only the measured slew, which no
  // per-gate STA update refreshes, comes from here.
  std::vector<std::vector<OutputPinSnap>> outputs;
  outputs.reserve(entries.size());
  for (const TraversalEntry& e : entries) {
    outputs.push_back(snapOutputs(state, e.inst));
  }

  int violating = 0;
  int upsized = 0;
  for (size_t i = 0; i < entries.size(); ++i) {
    sta::Instance* inst = entries[i].inst;
    std::vector<OutputPinSnap>& outs = outputs[i];
    if (outs.empty()) {
      continue;
    }
    refreshLoads(state, outs);
    sta::LibertyCell* cur_cell = network->libertyCell(inst);
    if (cur_cell == nullptr || clearsViolations(state, outs, cur_cell)) {
      continue;
    }
    ++violating;
    Group& g = group(state, cur_cell, cache);
    // group() puts the as-given cell - here the min-size one - at index 0.
    const size_t pick = selectFixviolUpsize(
        g.candidates, /*current=*/0, [&](const size_t index) {
          return clearsViolations(state, outs, g.cells[index]);
        });
    if (g.cells[pick] != cur_cell && resizer.replaceCell(inst, g.cells[pick])) {
      ++upsized;
    }
  }

  if (upsized > 0) {
    // Same contract as the swap half: the seed / projection / timing-weight
    // anchor that follow read slacks, so leave the design timing-valid.
    resizer.updateParasiticsAndTiming();
  }
  logger->info(RSZ,
               445,
               "GLOBAL_SIZING: min_size_fixviol repair - {} of {} editable "
               "gates violated max-cap/max-slew after the min-size reset; "
               "{} cleared by upsizing, {} still violating (no member of the "
               "swappable group clears them).",
               violating,
               instances.size(),
               upsized,
               violating - upsized);
  return upsized;
}

std::unique_ptr<InitPass> makeInitPass(const GlobalSizingConfig& /* config */)
{
  // The N axis has a single strategy: the equivalence-group swap (with
  // kAsGiven == as-given), plus min_size_fixviol's reverse-topological
  // violation-removal pass on top of the min_size selection.
  return std::make_unique<InitModePass>();
}

}  // namespace rsz
