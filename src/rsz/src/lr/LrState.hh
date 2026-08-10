// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <unordered_map>
#include <vector>

#include "sta/GraphClass.hh"
#include "sta/MinMax.hh"

namespace sta {
class dbNetwork;
class dbSta;
class Edge;
class Graph;
class Network;
class Vertex;
}  // namespace sta

namespace utl {
class Logger;
}  // namespace utl

namespace rsz {

class Resizer;
struct GlobalSizingConfig;

// The design-level metrics one LR iteration produces, computed once by the
// driver after each sweep's STA update and handed to the strategies that make
// decisions from them (H1 termination, H2 best tracking) as well as to the
// level-1 trace. Leakage is in watts (or the leakage-equivalent area cost on an
// area-only library, matching LRSubproblem::leakageOrArea); area in m^2.
struct IterMetrics
{
  float wns = 0.0f;
  float tns = 0.0f;
  float leakage = 0.0f;
  float area = 0.0f;
};

// LrState bundles the mutable Lagrangian multiplier state and the read-only STA
// handles that every global-sizing strategy (src/rsz/src/lr/) reads or writes,
// so a strategy never touches GlobalSizingPolicy directly. GlobalSizingPolicy
// owns one LrState and passes it by reference to each strategy.
//
// The read-only handles are filled once in GlobalSizingPolicy::start(); the
// multiplier vectors are sized by allocate() at the start of each run and then
// seeded / updated / projected in place by the strategies.
struct LrState
{
  // === Read-only handles (set once at construction) =========================
  sta::dbSta* sta = nullptr;
  sta::Network* network = nullptr;
  sta::dbNetwork* db_network = nullptr;
  sta::Graph* graph = nullptr;
  Resizer* resizer = nullptr;
  utl::Logger* logger = nullptr;
  const GlobalSizingConfig* config = nullptr;
  const sta::MinMax* max = sta::MinMax::max();
  // Analysis point for arc-delay reads; set by the driver before allocate().
  sta::DcalcAPIndex dcalc_ap = 0;

  // Clock period T (seconds), the max period over all SDC clocks. Used by the
  // flach / sharma / reimann lambda updaters to normalize slack. 0 when no
  // clock is defined (those updaters then no-op). Filled by
  // captureInitialTiming().
  float T = 0.0f;
  // Initial (pre-LR) worst slack of the design, and per-vertex initial slack
  // indexed by sta::Graph vertex id (sentinel kSlackSentinel for unconstrained
  // vertices). The reimann_dwns updater references these; filled once by
  // captureInitialTiming() and never rewritten during the run.
  float wns_init = 0.0f;
  std::vector<float> slack_init;
  // Slack magnitude beyond which a vertex is treated as unconstrained (matches
  // the sentinel OpenSTA reports for vertices with no real required time).
  static constexpr float kSlackSentinel = 1e6f;
  // The design metrics of the solution handed to the LR phase (pre-init,
  // pre-loop). The reimann_score best-tracker measures every iterate's deltas
  // against this "input solution" (Eq. 6); filled by the driver.
  IterMetrics metrics_init;

  // Near-met phase latch (C2). Driver-written, strategy-read-only: the driver
  // sets it each iteration via nearMetLatched() and the stagnation monitor and
  // local-slack veto read it to gate their activation (Sharma's power-recovery
  // regime, sharma_et_al.md §5.2). Latched once set; allocate() resets it per
  // run. When near_met_gate_frac < 0 (the default) it is latched from iteration
  // 0, so both consumers are always active.
  bool near_met = false;

  // The 0-based index of the sweep the driver's MAIN loop is currently running.
  // Driver-written before each of those sweeps, strategy-read-only; the
  // estimation-loop dry sweeps that may precede the loop never write it, so
  // they all read 0 and therefore all run the exhaustive scan (which is what
  // LRSubproblem's own comment on the F4 gate already promises). Today its one
  // consumer is the F4 axis: sharma_fast_olr switches over at the paper's 5th
  // LDP iteration, so the sweep has to know which one it is. Every other
  // strategy that needs an iteration number already receives one as a call
  // argument.
  //
  // SWEEP INDEX vs LDP ITERATION: for i >= 1 they are THE SAME NUMBER, and
  // reading an offset into them is what caused the F4 off-by-one (re-audit
  // delta D2). Sweep 0 runs before the first λ update (the driver gates the
  // update on iter > 0), so it prices the raw seed rather than a dual iterate
  // and is not one of the papers' update-then-LRS iterations at all - RA
  // finding F11's loop-order artifact. It is the sweep COUNT that exceeds the
  // LDP-iteration count by one, not the indices; sweep i is the paper's 1-based
  // LDP iteration i, and SweepEngine compares this field against
  // fast_olr_start_iter with no offset.
  int iter = 0;

  // === Mutable multiplier state (sized by allocate) =========================
  // Per-edge multipliers, indexed by sta::Edge::id (sparse).
  std::vector<float> lambda;
  // Per-endpoint multipliers, indexed by a dense endpoint index.
  std::vector<float> mu;
  // Dense endpoint bookkeeping.
  std::vector<sta::Vertex*> endpoint_vertices;
  std::unordered_map<const sta::Vertex*, int> endpoint_index;
  // Per-vertex depth-normalized downsize budget, indexed by sta::Graph vertex
  // id. Rebuilt each sweep by the sweep engine; empty unless the depth_budget
  // guard is active.
  std::vector<float> vertex_budget;
  // Per-vertex slack frozen at sweep start, indexed by sta::Graph vertex id.
  // Rebuilt each sweep by the sweep engine (the one place the per-vertex slack
  // queries happen); the budgets are derived from it and the local-slack veto
  // tests candidates against it.
  std::vector<float> vertex_slack;

  // === B2 cost-term state (M3) - sized by allocate only when the flag is on ==
  // Per-edge cumulative back-propagated delay sensitivity φ (Flach Eq. 11),
  // indexed by sta::Edge::id (same space as lambda). Empty unless
  // config.cost_global_phi; filled once per iteration by
  // computePhiSensitivities (CostTerms.cc).
  std::vector<float> phi;
  // Per-edge previous-iteration incumbent arc delay (the cost_delta_delay
  // reference), indexed by sta::Edge::id. Empty unless config.cost_delta_delay;
  // refreshed once per iteration by captureReferenceDelays (CostTerms.cc). This
  // is NOT Ozdal's Eq. 7 reference, which is a load and is candidate-dependent
  // (see cost_delta_delay in GlobalSizingConfig.hh).
  std::vector<float> prev_delay;

  // Discover graph size (max data-edge id, endpoints) and size lambda_/mu_ +
  // endpoint bookkeeping. Reads the graph only; strategies fill the values.
  void allocate();

  // Snapshot the pre-LR timing picture: clock period T, worst slack, and
  // per-vertex slack. Call once after the first STA update and before the LR
  // loop. Only read by updaters that reference the initial solution
  // (reimann_dwns); harmless for the others.
  void captureInitialTiming();

  // Extend the multiplier storage to the live edge-id space (C3 item 3). A
  // sweep replaces cells and the follow-on updateParasitics()/findRequireds()
  // rebuild timing arcs, minting edge ids beyond the space allocate() sized. If
  // those ids are left out of `lambda`, the updater/projection/cost/φ passes
  // all skip them and a vertex whose data out-arcs are ALL out-of-range floors
  // its in-arcs to zero and cascades a λ blackout upstream (tennakoon audit
  // §5.4). Called by the driver at the top of each iteration: grows `lambda`
  // (and the active `phi`/`prev_delay` stores) with neutral 0 for the new arcs
  // so the next projection distributes real values over them, and returns the
  // count of re-minted data arcs for the level-2 debug counter (0 when nothing
  // grew).
  int growToLiveEdges();

  // === Shared graph helpers (used by several strategies) ====================
  bool isDataArc(const sta::Edge* edge) const;
  // Max data-arc delay across rise/fall for `edge` at dcalc_ap (0 if no arc
  // set).
  float edgeMaxArcDelay(sta::Edge* edge) const;

  // A self-consistent read of one data edge for the arrival-based updaters and
  // the Mangiras seed (C3 item 1). The old consumers combined three independent
  // worst-of-rise/fall reads — a_from = worst arrival at from_v, d = max arc
  // delay over the edge, a_to = worst arrival at to_v — whose transitions need
  // not agree, so a_from + d could exceed a_to even on the vertex's critical
  // in-edge and manufacture a strictly positive arc "violation" (the rise/fall
  // max-collapse artifact — chen §5.1, tennakoon §5.1, livramento §5.1,
  // mangiras §5.1; the likely source of the measured λ 7→8 creep). This read
  // instead picks the (a_from, d) of the arc/transition realizing the edge's
  // OWN worst propagated arrival and pairs it with the to-vertex arrival, so
  // a_from + d <= a_to holds by construction (equality exactly when this edge
  // is the vertex's critical in-edge). Per-(arc, transition) λ stays out of
  // scope framework-wide (every audit §4); only the inconsistent mixing is
  // fixed.
  struct ConsistentArcRead
  {
    float a_from = 0.0f;  // from-vertex arrival in the realizing transition
    float d = 0.0f;       // realizing arc's delay
    float a_to = 0.0f;    // to-vertex worst arrival (the vertex constraint)
  };
  ConsistentArcRead consistentArcRead(sta::Edge* edge,
                                      sta::Vertex* from_v,
                                      sta::Vertex* to_v) const;
};

// One (arc, transition) read of an edge: the from-vertex arrival in the arc's
// input transition and that arc's own delay. Their sum is the arrival the pair
// propagates to the to-vertex. Public so the pure selection core below is
// unit-testable without STA.
struct ArcTransitionRead
{
  float a_from = 0.0f;
  float d = 0.0f;
};

// Pure selection core (no STA): the (a_from, d) pair realizing an edge's own
// worst propagated arrival (max a_from + d) over its arcs/transitions. Ties
// resolve to the first maximizer; an empty list returns {0, 0}. This is the
// arithmetic behind LrState::consistentArcRead — factored out so the rise/fall
// consistency invariant is pinned on hand-computed values (C3 item 1).
ArcTransitionRead pickCriticalArcTransition(
    const std::vector<ArcTransitionRead>& reads);

}  // namespace rsz
