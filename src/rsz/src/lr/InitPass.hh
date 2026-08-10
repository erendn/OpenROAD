// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "InitSelect.hh"
#include "LrState.hh"
#include "rsz/GlobalSizingConfig.hh"

namespace sta {
class Instance;
class LibertyCell;
}  // namespace sta

namespace rsz {

// N axis - Initial solution. Transforms the live design before LR state is
// seeded and returns the number of instances replaced.
class InitPass
{
 public:
  virtual ~InitPass() = default;
  virtual int run(LrState& state) = 0;
};

// The equivalence-group init pass: swap every editable instance to the member
// of its swappable-equivalence group that init_mode names. init_mode ==
// kAsGiven is the "as-given" initial solution and is a no-op. The per-mode
// selection rules live in InitSelect.hh; this class owns the STA-facing half.
//
// min_size_fixviol is min_size followed by the reverse-topological electrical
// repair pass below (the flach/sharma C-init, plan §2.1 / RA F6).
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

  // min_size_fixviol's second half: walk `instances` from outputs toward inputs
  // and upsize each gate whose own output pins violate max-cap or max-slew
  // after the min-size reset, until the violation clears or the gate's
  // swappable group is exhausted. Reverse-topological because repairing a gate
  // raises its input capacitance, which is load on its DRIVERS - visiting sinks
  // first means every driver is evaluated against the loads its repaired fanout
  // actually presents. Deterministic and seed-free. Returns the number of gates
  // it upsized.
  int repairViolations(LrState& state,
                       const std::vector<sta::Instance*>& instances,
                       GroupCache& cache) const;
};

std::unique_ptr<InitPass> makeInitPass(const GlobalSizingConfig& config);

}  // namespace rsz
