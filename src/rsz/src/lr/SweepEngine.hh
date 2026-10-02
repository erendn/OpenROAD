// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "CapRecheck.hh"
#include "LRSubproblem.hh"
#include "LrState.hh"
#include "ViolationRepair.hh"
#include "rsz/GlobalSizingConfig.hh"

namespace sta {
class Graph;
class Instance;
class Network;
}  // namespace sta

namespace utl {
class ThreadPool;
}  // namespace utl

namespace rsz {

class Resizer;

// Solver for the per-iteration Lagrangian relaxation subproblem (LRS). Owns
// the LRSubproblem cost evaluator; one sweep chooses a discrete cell for each
// gate against the current relaxed objective. The caller runs the batched STA
// update after each sweep.
class SweepEngine
{
 public:
  struct Stats
  {
    // What the sweep committed, before the post-sweep max-cap re-check
    // (CapRecheck.hh). The re-check then undid `cap_reverts` of these moves,
    // so moves - cap_reverts were kept. The other counts are not reduced, so
    // the upsize/downsize split still describes what the sweep chose.
    int moves = 0;
    int upsizes = 0;
    int downsizes = 0;
    int cap_reverts = 0;
    // The re-check hit its pass limit while reverts were still cascading, so
    // the moved gates' nets may not be clean. The driver reports this in
    // RSZ-0443, since the revert count alone cannot tell a finished clean-up
    // from an interrupted one.
    bool cap_recheck_bound_hit = false;
  };

  virtual ~SweepEngine() = default;

  // Build the per-run cost machinery (leakage-equivalent scale). Call once
  // after the multiplier seed and projection, before computeTimingWeightBase
  // and the sweep loop.
  virtual void init(LrState& state) = 0;

  // The design-level anchor of the timing weight (see TimingScale.hh).
  // Called once before the loop and then kept fixed for the run. It is a
  // method here only because the engine owns the Resizer and LRSubproblem the
  // computation needs; the arithmetic is in TimingScale.cc.
  virtual float computeTimingWeightBase(LrState& state) = 0;

  // One sweep over all leaf instances; returns the move tally. `timing_weight`
  // scales the Σλ·d timing term against the power term.
  virtual Stats sweep(LrState& state, float timing_weight) = 0;

  // The max-capacitance fix pass (cap_fix_pass), run after a sweep. It is a
  // method here only because the engine owns the LRSubproblem that prices the
  // candidates; the pass is in ViolationRepair.cc.
  virtual RepairWalkStats fixMaxCapViolations(LrState& state,
                                              float timing_weight)
      = 0;
};

// Parallel Jacobi sweep over frozen per-gate snapshots, in three phases:
//   A buildSnapshots  - main thread: freeze each gate's timing/DRC state
//   B evaluate        - workers: evaluate every snapshot independently
//   C applyDecisions  - main thread: apply the winning replacements
// Every gate is evaluated against the same sweep-start state, so a gate does
// not see the moves of other gates in the same sweep.
class JacobiSnapshotSweep : public SweepEngine
{
 public:
  JacobiSnapshotSweep(Resizer* resizer, utl::ThreadPool* thread_pool);
  ~JacobiSnapshotSweep() override;

  void init(LrState& state) override;
  float computeTimingWeightBase(LrState& state) override;
  Stats sweep(LrState& state, float timing_weight) override;
  RepairWalkStats fixMaxCapViolations(LrState& state,
                                      float timing_weight) override;

 private:
  // Phase A: freeze the per-gate snapshots for every evaluable leaf instance.
  std::vector<LRSubproblem::GateSnapshot> buildSnapshots(LrState& state);
  // Phase C: apply the accepted replacements in vector order (no timing query).
  // Fills `movers` with what it replaced, for the post-sweep cap re-check.
  Stats applyDecisions(LrState& state,
                       const std::vector<LRSubproblem::GateDecision>& decisions,
                       int visited,
                       std::vector<MovedGate>& movers);

  Resizer* resizer_ = nullptr;
  utl::ThreadPool* thread_pool_ = nullptr;  // owned by the policy
  std::unique_ptr<LRSubproblem> subproblem_;
};

// Visit order of the Gauss-Seidel sweep. One entry per leaf instance: `key` is
// the ordering metric (output-vertex level for the topological modes, or
// sweep-start slack for criticality_sorted), `tiebreak` is a stable instance id
// so ties are broken deterministically.
struct TraversalEntry
{
  float key = 0.0f;
  uint64_t tiebreak = 0;
  sta::Instance* inst = nullptr;
};

// The gate's traversal key for the topological orders: the minimum level over
// its output vertices, or the float maximum when it has none (such a gate sorts
// last and is dropped by the eligibility filter anyway). Shared by the
// Gauss-Seidel engine and the repair walk of lr/ViolationRepair.hh so both use
// the same notion of "outputs toward inputs".
float topoTraversalKey(sta::Network* network,
                       sta::Graph* graph,
                       sta::Instance* inst);

// Order `entries` in place per `traversal`: ascending key for forward_topo and
// criticality_sorted (lowest level / most negative slack first), descending
// for reverse_topo; ties break on the smaller `tiebreak`. This is a total
// order, so the result does not depend on the input order. Pure (no STA).
void orderTraversal(std::vector<TraversalEntry>& entries,
                    GlobalSizingConfig::Traversal traversal);

// Whether sweep `iter` uses the Fast-OLR move set (Sharma et al., ICCAD 2015,
// Fig. 9) instead of the exhaustive scan. `fast_olr_start_iter` counts the
// paper's LDP iterations (a multiplier update followed by an LRS) from 1.
//
// No offset is needed. `iter` is the driver's 0-based iteration index, and
// iteration 0 sweeps before the first λ update (the driver updates λ only for
// iter > 0), so it prices the initial multipliers and is not an LDP
// iteration. Iteration i for i >= 1 is therefore LDP iteration i, and the
// default of 5 switches at the fifth λ-updated iteration, as in the paper.
// Comparing `iter + 1` would switch one iteration early.
inline bool fastOlrActive(const int iter, const int fast_olr_start_iter)
{
  return iter >= fast_olr_start_iter;
}

// Pack the multiplier, cost-term and guard vectors, and the sweep options
// from the config, into the struct that LRSubproblem::snapshot() reads. Called
// once per sweep by both engines. Exposed so the config-to-SnapshotInputs
// wiring can be unit-tested: a field left unset here silently falls back to
// the struct default, which regression tests would not notice.
//
// Reads STA only under downsize_guard = local_slack_veto, which needs the
// sweep-start WNS for Flach's gamma; under the other guards it only reads
// `state`'s vectors and `*state.config`.
LRSubproblem::SnapshotInputs sweepInputs(LrState& state);

// Whether a gate's best candidate is accepted. Shared by both engines so they
// make the same accept/reject decisions:
//   - Upsizes must lower the LR cost by more than `upsize_hysteresis`
//     (relative), which filters out moves caused by noise.
//   - Downsizes are accepted on any cost decrease: on a non-critical gate
//     lambda is at its floor and the cost is dominated by power, so any
//     decrease is a real power gain.
// With upsize_hysteresis = 0 this is the plain LRS argmin used in the papers
// (any strict improvement is accepted in both directions). The default
// configuration uses 0.02.
bool acceptGateMove(const LRSubproblem::GateDecision& decision,
                    float upsize_hysteresis);

// Sequential Gauss-Seidel sweep: visit leaf instances in `traversal` order,
// build a just-in-time snapshot of each gate, evaluate it, and commit the
// winning replacement before moving on, so downstream gates see fresh upstream
// state. This is how the LRS is solved in most of the LR sizing papers (e.g.
// Flach et al., TCAD 2014, Alg. 3). Runs single-threaded on the main thread,
// because the just-in-time snapshot reads live STA, so it is deterministic. It
// uses the same snapshot builder, cost evaluator, eligibility filter and
// acceptance rule as the Jacobi engine; what it adds is the per-commit timing
// refresh (gs_refresh) and the traversal order.
class GaussSeidelSweep : public SweepEngine
{
 public:
  explicit GaussSeidelSweep(Resizer* resizer);
  ~GaussSeidelSweep() override;

  void init(LrState& state) override;
  float computeTimingWeightBase(LrState& state) override;
  Stats sweep(LrState& state, float timing_weight) override;
  RepairWalkStats fixMaxCapViolations(LrState& state,
                                      float timing_weight) override;

 private:
  // Build the deterministic visit order for this sweep from the sweep-start
  // graph levels or slacks (config.traversal).
  std::vector<sta::Instance*> buildTraversalOrder(LrState& state);
  // Refresh after each commit so the next gate's snapshot reads fresh upstream
  // state (config.gs_refresh): incremental parasitics always, plus
  // required-time propagation under gs_incremental only.
  void refreshAfterCommit(LrState& state);

  Resizer* resizer_ = nullptr;
  std::unique_ptr<LRSubproblem> subproblem_;
};

std::unique_ptr<SweepEngine> makeSweepEngine(const GlobalSizingConfig& config,
                                             Resizer* resizer,
                                             utl::ThreadPool* thread_pool);

}  // namespace rsz
