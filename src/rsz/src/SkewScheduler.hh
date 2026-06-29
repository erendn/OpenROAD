// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <map>
#include <utility>
#include <vector>

#include "db_sta/dbSta.hh"
#include "sta/MinMax.hh"
#include "sta/NetworkClass.hh"

namespace rsz {

class Resizer;

// SkewScheduler: builds the register-to-register difference-constraint graph
// from STA and solves clock-skew-scheduling feasibility (Deokar-Sapatnekar).
//
// Shared by the read-only probe (SkewAnalysis, two-sided budget +/-B) and the
// realizer (SkewRealizer, one-sided/delay-only budget 0..B).
//
// Model (slack-shift): per arc a->b, new_setup = base_setup + x(b) - x(a),
// new_hold = base_hold - x(b) + x(a), with base_* from STA worst paths.
// Constraints (difference form, feasible == no negative cycle, SPFA):
//   setup : x(a) - x(b) <= base_setup - delta
//   hold  : x(b) - x(a) <= max(base_hold, 0)        (never worsen current hold)
//   budget: 0..B two-sided  -> x(v)-x(0) in [-B, B]
//           one-sided       -> x(v)-x(0) in [0,  B]  (delay-only, realizable)
// Node = register dbInst; node 0 = ground (ports / fixed references).
class SkewScheduler : public sta::dbStaState
{
 public:
  explicit SkewScheduler(Resizer* resizer);

  // (Re)build the constraint graph from the current STA timing state.
  void build();

  bool empty() const { return setup_arcs_.empty(); }

  // Largest delta (achievable setup WNS) feasible under a per-register skew
  // budget.  budget < 0 == unbounded.  one_sided restricts x(v) >= 0.
  double maxAchievableWns(double budget, bool one_sided, bool& capped) const;

  // Solve at target delta; fill skew_out (register instance -> skew seconds),
  // normalized so ground = 0 (one_sided => all skew >= 0).  False if
  // infeasible.
  bool solveSchedule(double delta,
                     double budget,
                     bool one_sided,
                     std::map<const sta::Instance*, double>& skew_out) const;

  // Node-indexed feasible solution vector (for probe stats / dump).
  bool solveVector(double delta,
                   double budget,
                   bool one_sided,
                   std::vector<double>& x_out) const;

  // --- accessors for reporting -------------------------------------------
  double designWns() const { return design_wns_; }
  double analyzedWns() const { return analyzed_wns_; }
  double period() const { return period_; }
  int numRegisters() const { return n_nodes_ - 1; }
  int numSetupArcs() const { return static_cast<int>(setup_arcs_.size()); }
  int numHoldArcs() const { return static_cast<int>(hold_arcs_.size()); }
  int numHoldViolations() const { return hold_violation_arcs_; }
  int numSkippedEndpoints() const { return skipped_endpoints_; }
  const sta::Instance* nodeInstance(int id) const { return node_inst_[id]; }

  static constexpr double kDeltaTol = 5e-14;  // binary-search tol (~0.05 ps)

 private:
  void init();
  int nodeIndex(const sta::Pin* pin);
  void collectArcs();
  bool feasible(double delta,
                double budget,
                bool one_sided,
                std::vector<double>* x_out) const;

  Resizer* resizer_;

  const sta::MinMax* max_ = sta::MinMax::max();
  const sta::MinMax* min_ = sta::MinMax::min();

  std::map<const sta::Instance*, int> inst_node_;
  std::vector<const sta::Instance*> node_inst_;  // node id -> instance (0=null)

  struct SetupArc
  {
    int a;
    int b;
    double base;
  };
  struct HoldArc
  {
    int a;
    int b;
    double weight;  // max(base_hold, 0)
  };
  std::map<std::pair<int, int>, double> setup_map_;
  std::map<std::pair<int, int>, double> hold_map_;
  std::vector<SetupArc> setup_arcs_;
  std::vector<HoldArc> hold_arcs_;

  int n_nodes_ = 1;
  double analyzed_wns_ = 0.0;
  double design_wns_ = 0.0;
  double period_ = 0.0;
  int hold_violation_arcs_ = 0;
  int skipped_endpoints_ = 0;

  // A slack magnitude above this is treated as "unconstrained" (STA INF).
  static constexpr double kInfSlack = 1.0;
};

}  // namespace rsz
