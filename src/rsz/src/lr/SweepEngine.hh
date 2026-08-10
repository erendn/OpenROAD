// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "CapRecheck.hh"
#include "LRSubproblem.hh"
#include "LrState.hh"
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

// F1/F2 axis - the per-iteration LRS solver. Owns the LRSubproblem cost
// evaluator; one sweep chooses a discrete cell for each gate against the
// current relaxed objective. The caller runs the batched STA update after each
// sweep.
class SweepEngine
{
 public:
  struct Stats
  {
    // What the sweep committed, before the post-sweep max-cap re-check
    // (CapRecheck.hh) ran. `cap_reverts` of those moves were then undone, so
    // the net kept count is moves - cap_reverts; the raw tallies stay attempted
    // counts so the up/downsize split still describes what the sweep chose.
    int moves = 0;
    int upsizes = 0;
    int downsizes = 0;
    int cap_reverts = 0;
    // The re-check stopped on its pass bound with reverts still cascading, so
    // it is NOT known to have left the movers' nets clean. Surfaced by the
    // driver in RSZ-0443 - a revert count on its own reads as a successful
    // clean-up whether or not the re-check ran to a clean pass.
    bool cap_recheck_bound_hit = false;
  };

  virtual ~SweepEngine() = default;

  // Build per-run cost machinery (leakage-equivalent scale). Call once after
  // the seed + projection, before computeTimingWeightBase and the sweep loop.
  virtual void init(LrState& state) = 0;

  // The A1 axis's frozen design anchor for the timing weight (TimingScale.hh).
  // Called once before the loop - the iteration-0 freeze every option shares.
  // The engines only host it because they own the Resizer / LRSubproblem the
  // walk needs; the option's arithmetic lives in TimingScale.cc.
  virtual float computeTimingWeightBase(LrState& state) = 0;

  // One sweep over all leaf instances; returns the move tally. `timing_weight`
  // scales the Σλ·d timing term against the leakage objective.
  virtual Stats sweep(LrState& state, float timing_weight) = 0;
};

// Current 3-phase parallel Jacobi sweep over frozen per-gate snapshots:
//   A buildSnapshots  - main thread: freeze each gate's timing/DRC state
//   B parallel score  - workers: score every snapshot independently
//   C applyDecisions  - main thread: apply the winning replacements
// with the per-vertex depth-normalized downsize budget guard.
class JacobiSnapshotSweep : public SweepEngine
{
 public:
  JacobiSnapshotSweep(Resizer* resizer, utl::ThreadPool* thread_pool);
  ~JacobiSnapshotSweep() override;

  void init(LrState& state) override;
  float computeTimingWeightBase(LrState& state) override;
  Stats sweep(LrState& state, float timing_weight) override;

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

// F2 traversal ordering (Gauss-Seidel). One entry per leaf instance: `key` is
// the ordering metric (output-vertex level for the topological modes, or
// sweep-start slack for criticality_sorted), `tiebreak` is a stable instance id
// so ties are broken deterministically.
struct TraversalEntry
{
  float key = 0.0f;
  uint64_t tiebreak = 0;
  sta::Instance* inst = nullptr;
};

// The gate's traversal key for the TOPOLOGICAL orders: the minimum level over
// its output vertices, or +inf when it has none (such a gate sorts last and is
// dropped by the eligibility filter anyway). Shared so the Gauss-Seidel engine
// and min_size_fixviol's reverse-topological repair walk cannot disagree on
// what "outputs toward inputs" means.
float topoTraversalKey(sta::Network* network,
                       sta::Graph* graph,
                       sta::Instance* inst);

// Order `entries` in place per `traversal`: ascending key for forward_topo and
// criticality_sorted (lowest level / most-negative slack first), descending for
// reverse_topo; ties always break on the smaller `tiebreak`. A total order, so
// the result is independent of the input order (deterministic). Pure - no STA;
// unit-tested against hand-built entries.
void orderTraversal(std::vector<TraversalEntry>& entries,
                    GlobalSizingConfig::Traversal traversal);

// F4 switch-over: has sweep `iter` reached `fast_olr_start_iter`, the 1-based
// λ-updated LDP iteration from which Sharma Fig. 9's pruned enumeration
// replaces the exhaustive scan?
//
// There is no offset, and that is the whole content of this function. `iter` is
// the driver's 0-based sweep index, but sweep 0 runs BEFORE the first λ update
// (the driver gates the update on iter > 0 - RA finding F11), so it prices the
// raw seed and is not one of the paper's update-then-LRS iterations. Sweep i
// for i >= 1 therefore IS the paper's LDP iteration i, and the default 5 fires
// on the 5th λ-updated LRS, as printed. Comparing `iter + 1` counted the
// pre-update sweep as iteration 1 and fired one iteration early (re-audit
// delta D2, fixed 2026-08-09).
inline bool fastOlrActive(const int iter, const int fast_olr_start_iter)
{
  return iter >= fast_olr_start_iter;
}

// Package the per-run multiplier / cost-term / guard vectors, and every frozen
// axis choice, into the struct snapshot() reads. Called once per sweep by both
// engines; exposed so the config -> SnapshotInputs wiring is unit-testable,
// because an axis field left unassigned here is invisible to every golden (the
// sweep simply runs the struct default) and that is the failure mode this
// module has already hit once.
//
// Reads STA only under downsize_guard = local_slack_veto, which needs the
// sweep-start WNS for Flach's gamma; under the other guards it is a pure
// read of `state`'s vectors and `*state.config`.
LRSubproblem::SnapshotInputs sweepInputs(LrState& state);

// Asymmetric acceptance deadband on the LR-cost improvement, shared by both
// engines so their accept/reject decisions are directly comparable:
//   - Upsize moves: need the cost to drop by `upsize_hysteresis` (relative) -
//     the OpenROAD noise filter, config.upsize_hysteresis.
//   - Downsize moves: any drop is enough - on a non-critical gate lambda is at
//     the floor and the cost is leakage-dominated, so any drop is a real
//     leakage gain.
// At upsize_hysteresis = 0 this is the plain LRS argmin every paper specifies
// (both directions accept any strict improvement); 0.02 is rsz_baseline's
// heuristic. Pure - unit-tested at both values.
bool acceptGateMove(const LRSubproblem::GateDecision& decision,
                    float upsize_hysteresis);

// Sequential Gauss-Seidel sweep (F1 kGaussSeidelTopo): visit leaf instances in
// `traversal` order, building a just-in-time per-gate snapshot, scoring it, and
// committing the winning replacement before the next gate - so downstream gates
// see fresh upstream state (the papers' LRS mechanics). Runs single-threaded on
// the main thread (the JIT snapshot reads live STA), so it is deterministic.
// Reuses the same LRSubproblem snapshot builder, cost evaluator, eligibility
// filter, and asymmetric acceptance as the Jacobi engine; the only new state is
// the per-commit refresh (gs_refresh) and the traversal order.
class GaussSeidelSweep : public SweepEngine
{
 public:
  explicit GaussSeidelSweep(Resizer* resizer);
  ~GaussSeidelSweep() override;

  void init(LrState& state) override;
  float computeTimingWeightBase(LrState& state) override;
  Stats sweep(LrState& state, float timing_weight) override;

 private:
  // Build the deterministic per-gate visit order for this sweep from the
  // sweep-start graph levels / slacks (config.traversal).
  std::vector<sta::Instance*> buildTraversalOrder(LrState& state);
  // Per-commit refresh so the next gate's snapshot reads fresh upstream state
  // (config.gs_refresh): incremental parasitics always, plus required-time
  // propagation only under gs_incremental.
  void refreshAfterCommit(LrState& state);

  Resizer* resizer_ = nullptr;
  std::unique_ptr<LRSubproblem> subproblem_;
};

std::unique_ptr<SweepEngine> makeSweepEngine(const GlobalSizingConfig& config,
                                             Resizer* resizer,
                                             utl::ThreadPool* thread_pool);

}  // namespace rsz
