// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "InitPass.hh"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "InitSelect.hh"
#include "ViolationRepair.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "rsz/Resizer.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
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
  // The as-given cell goes first: the modes are defined relative to it, and
  // the deterministic scan starts there, so it wins a full tie.
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
    // Per instance, so nothing can be memoized. The draw is keyed by the
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

namespace {

// The group member `mode` picks, as worded in RSZ-0416. as_given,
// min_size_fixviol and min_size_fixcap never reach it: applyInitMode() is not
// called for as_given and maps the other two to min_size.
std::string initTargetDescription(const InitMode mode, const int init_seed)
{
  switch (mode) {
    case InitMode::kMinSize:
      return "lowest-leakage member";
    case InitMode::kMaxSize:
      return "highest-leakage member";
    case InitMode::kAverage:
      return "lower-median member";
    case InitMode::kRandom:
      return "uniform draw (init_seed=" + std::to_string(init_seed) + ")";
    case InitMode::kAsGiven:
    case InitMode::kMinSizeFixviol:
    case InitMode::kMinSizeFixcap:
      break;
  }
  return "";
}

// The limits of the reverse-topological repair that follows the min-size
// reset of `mode`, or nullopt if `mode` has none.
std::optional<RepairedLimits> initRepairLimits(const InitMode mode)
{
  switch (mode) {
    case InitMode::kMinSizeFixviol:
      return RepairedLimits::kMaxCapAndSlew;
    case InitMode::kMinSizeFixcap:
      return RepairedLimits::kMaxCap;
    case InitMode::kAsGiven:
    case InitMode::kMinSize:
    case InitMode::kMaxSize:
    case InitMode::kRandom:
    case InitMode::kAverage:
      break;
  }
  return std::nullopt;
}

// The slew fix pass over `instances`, with its log line. Returns the number of
// gates upsized.
int runSlewFixPass(LrState& state, const std::vector<sta::Instance*>& instances)
{
  const RepairWalkStats stats = fixMaxSlewViolations(state, instances);
  state.logger->info(RSZ,
                     461,
                     "GLOBAL_SIZING: slew fix pass - {} of {} editable gates "
                     "were over their max slew; {} upsized to clear it, {} "
                     "upsized but still over it, {} left as they were.",
                     stats.violating,
                     instances.size(),
                     stats.cleared,
                     stats.replaced - stats.cleared,
                     stats.violating - stats.replaced);
  return stats.replaced;
}

}  // namespace

int InitModePass::run(LrState& state)
{
  const GlobalSizingConfig& config = *state.config;
  if (config.init_mode == InitMode::kAsGiven && !config.slew_fix_pass) {
    return 0;
  }
  // The gates the pass may touch, kept so every step works on exactly the
  // same gates.
  const std::vector<sta::Instance*> editable = sizableGates(state);
  int replacements = 0;
  if (config.init_mode != InitMode::kAsGiven) {
    replacements = applyInitMode(state, editable);
  }
  // Sharma et al. (ICCAD 2015, Sec. III-A) repair slew after capacitance.
  if (config.slew_fix_pass) {
    replacements += runSlewFixPass(state, editable);
  }
  return replacements;
}

int InitModePass::applyInitMode(
    LrState& state,
    const std::vector<sta::Instance*>& editable) const
{
  const InitMode mode = state.config->init_mode;
  Resizer& resizer = *state.resizer;
  sta::Network* network = state.network;
  utl::Logger* logger = state.logger;

  // min_size_fixviol and min_size_fixcap are min_size plus a
  // reverse-topological electrical repair, so the selector runs the min_size
  // rule and the repair runs after.
  const std::optional<RepairedLimits> repair = initRepairLimits(mode);
  const InitMode select_mode = repair.has_value() ? InitMode::kMinSize : mode;

  logger->info(RSZ,
               416,
               "GLOBAL_SIZING: init_mode={} - replacing every editable "
               "instance with the {} of its leakage-ranked swappable group.",
               toString(mode),
               initTargetDescription(select_mode, state.config->init_seed));

  int replacements = 0;
  GroupCache cache;
  for (sta::Instance* inst : editable) {
    sta::LibertyCell* current_cell = network->libertyCell(inst);
    sta::LibertyCell* replacement
        = selectCell(state, inst, current_cell, select_mode, cache);
    if (replacement != current_cell && resizer.replaceCell(inst, replacement)) {
      ++replacements;
    }
  }

  // The driver records the initial solution in its outer journal. The
  // multiplier seed, the projection and computeAutoTimingWeight need fresh
  // slacks, so update parasitics and timing here. The repair pass also needs
  // them (it reads measured slews and graph levels), so it forces the update
  // even when the min-size reset changed nothing.
  if (replacements > 0 || repair.has_value()) {
    resizer.updateParasiticsAndTiming();
  }
  logger->info(RSZ,
               415,
               "GLOBAL_SIZING: init pass replaced {}/{} editable instances.",
               replacements,
               editable.size());
  if (repair.has_value()) {
    replacements += repairViolations(state, editable, *repair, cache);
  }
  return replacements;
}

int InitModePass::repairViolations(LrState& state,
                                   const std::vector<sta::Instance*>& instances,
                                   const RepairedLimits limits,
                                   GroupCache& cache) const
{
  // group() puts the as-given cell (here the min-size one) at index 0.
  const RepairWalkStats stats = repairInReverseTopoOrder(
      state,
      instances,
      limits,
      [&](sta::Instance* /* inst */,
          sta::LibertyCell* current,
          const std::vector<OutputPinSnap>& outputs) {
        Group& g = group(state, current, cache);
        const size_t pick = selectFixviolUpsize(
            g.candidates, /*current=*/0, [&](const size_t index) {
              return clearsViolations(state, outputs, g.cells[index], limits);
            });
        return g.cells[pick];
      });
  utl::Logger* logger = state.logger;
  logger->info(
      RSZ,
      445,
      "GLOBAL_SIZING: {} repair - {} of {} editable gates violated {} "
      "after the min-size reset; {} cleared by upsizing, {} still "
      "violating (no member of the swappable group clears them).",
      toString(state.config->init_mode),
      stats.violating,
      instances.size(),
      limits == RepairedLimits::kMaxCap ? "max-cap" : "max-cap/max-slew",
      stats.replaced,
      stats.violating - stats.replaced);
  return stats.replaced;
}

std::unique_ptr<InitPass> makeInitPass(const GlobalSizingConfig& /* config */)
{
  // A single strategy: the equivalence-group swap (a no-op for kAsGiven),
  // plus the repair passes of min_size_fixviol, min_size_fixcap and
  // slew_fix_pass.
  return std::make_unique<InitModePass>();
}

}  // namespace rsz
