// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "InitSelect.hh"
#include "LrState.hh"
#include "ViolationRepair.hh"
#include "rsz/GlobalSizingConfig.hh"

namespace sta {
class Instance;
class LibertyCell;
}  // namespace sta

namespace rsz {

// Builds the initial solution. Transforms the live design before the LR state
// is seeded and returns the number of instances replaced.
class InitPass
{
 public:
  virtual ~InitPass() = default;
  virtual int run(LrState& state) = 0;
};

// Swaps every editable instance to the member of its swappable-equivalence
// group that init_mode selects. init_mode == kAsGiven keeps the incoming
// netlist, apart from the slew fix pass described below. The selection rules
// are in InitSelect.hh; this class does the STA-facing part.
//
// min_size_fixviol is min_size followed by the electrical repair below. This
// is the initial solution of Flach et al. (TCAD 2014, Sec. IV): start from the
// lowest-leakage version of every gate, then visit gates from outputs to
// inputs and give each the lowest-leakage version that meets its load and slew
// limits. Sharma et al. (ICCAD 2015, Sec. III-A) start the same way but fix
// capacitance in a reverse-topological pass and slew in a separate forward
// pass: min_size_fixcap is the reset and the capacitance pass, and
// slew_fix_pass adds the slew pass (fixMaxSlewViolations in
// ViolationRepair.hh) as the last step, after any init_mode.
class InitModePass : public InitPass
{
 public:
  int run(LrState& state) override;

 private:
  // One swappable-equivalence group, resolved once per distinct library cell.
  // `cells` is parallel to `candidates` and `cells[0]` is the as-given cell.
  struct Group
  {
    std::vector<sta::LibertyCell*> cells;
    std::vector<InitCandidate> candidates;
    // The pick, memoized for the modes that do not depend on the instance.
    sta::LibertyCell* deterministic_choice = nullptr;
  };
  using GroupCache = std::unordered_map<sta::LibertyCell*, Group>;

  Group& group(LrState& state,
               sta::LibertyCell* current_cell,
               GroupCache& cache) const;

  sta::LibertyCell* selectCell(LrState& state,
                               sta::Instance* inst,
                               sta::LibertyCell* current_cell,
                               GlobalSizingConfig::InitMode mode,
                               GroupCache& cache) const;

  // Swaps `editable` to init_mode's selection and runs its repair, if any.
  // Returns the number of instances replaced.
  int applyInitMode(LrState& state,
                    const std::vector<sta::Instance*>& editable) const;

  // Second step of min_size_fixviol and min_size_fixcap: walk `instances` from
  // outputs toward inputs (lr/ViolationRepair.hh) and upsize each gate whose
  // own output pins violate `limits` after the min-size reset, to the
  // lowest-leakage cell that clears the violation (or not at all if no member
  // of its group does). Deterministic. Returns the number of gates upsized.
  int repairViolations(LrState& state,
                       const std::vector<sta::Instance*>& instances,
                       RepairedLimits limits,
                       GroupCache& cache) const;
};

std::unique_ptr<InitPass> makeInitPass(const GlobalSizingConfig& config);

}  // namespace rsz
