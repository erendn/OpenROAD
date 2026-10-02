// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "CapMultipliers.hh"
#include "ObjectivePower.hh"
#include "sta/GraphClass.hh"
#include "sta/LibertyClass.hh"
#include "sta/MinMax.hh"
#include "sta/NetworkClass.hh"
#include "sta/ObjectId.hh"

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

// Design metrics for one LR iteration, computed once by the driver after each
// sweep's timing update and passed to the strategies that use them
// (termination and best-solution tracking) and to the level-1 trace.
//
// `leakage` is the Liberty leakage summed over the instances whose cell has
// leakage data, in watts. `power` is the objective power (see ObjectivePower)
// summed over the same instances at their current cells and loads: equal to
// `leakage` under the leakage objective, and leakage plus dynamic power under
// the total objective. The strategies that measure the design's power read
// `power`. Area is in m^2.
struct IterMetrics
{
  float wns = 0.0f;
  float tns = 0.0f;
  float leakage = 0.0f;
  float power = 0.0f;
  float area = 0.0f;
};

// The ends of a timing edge: the graph ids of its from and to vertices. A
// cell swap keeps the instance's pins, and with them their vertices, so the
// ends name the same pin pair before and after the swap even when OpenSTA
// re-creates the edge between them. The default value marks a multiplier slot
// that no live edge holds (OpenSTA never uses vertex id 0).
struct EdgeEnds
{
  sta::VertexId from = sta::object_id_null;
  sta::VertexId to = sta::object_id_null;

  bool empty() const { return from == sta::object_id_null; }
  bool operator==(const EdgeEnds& other) const = default;
};

// The edges whose multipliers a live-edge refresh rewrote (see
// carryMultipliers). Summed over a run for the end-of-run report.
struct EdgeCarryStats
{
  // Edges that took a share of the earlier multipliers between their pins.
  int carried = 0;
  // Edges between pins that had no edge before; they start at 0.
  int started_at_zero = 0;

  bool any() const { return carried > 0 || started_at_zero > 0; }
  EdgeCarryStats& operator+=(const EdgeCarryStats& other)
  {
    carried += other.carried;
    started_at_zero += other.started_at_zero;
    return *this;
  }
};

// LrState holds the Lagrange multipliers and the read-only STA handles that
// the global-sizing strategies (src/rsz/src/lr/) read or write, so no strategy
// needs GlobalSizingPolicy directly. GlobalSizingPolicy owns one LrState and
// passes it by reference to each strategy.
//
// The handles are set once in GlobalSizingPolicy::start(). The multiplier
// vectors are sized by allocate() at the start of each run and then seeded,
// updated and projected in place by the strategies.
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

  // Clock period T in seconds: the largest period over all SDC clocks. The
  // lambda updaters use it to normalize slack. 0 when no clock is defined. Set
  // by captureInitialTiming().
  float T = 0.0f;
  // Worst slack of the design before LR, and per-vertex initial slack indexed
  // by sta::Graph vertex id (kSlackSentinel for unconstrained vertices). Read
  // by the reimann_dwns updater; set once by captureInitialTiming() and not
  // changed during the run.
  float wns_init = 0.0f;
  std::vector<float> slack_init;
  // Slack magnitude beyond which a vertex is treated as unconstrained (matches
  // the sentinel OpenSTA reports for vertices with no real required time).
  static constexpr float kSlackSentinel = 1e6f;
  // Design metrics of the solution handed to the LR phase, before
  // initialization and the loop. The reimann_score best tracker measures every
  // iterate against this input solution (Reimann et al., Eq. 6). Set by the
  // driver.
  IterMetrics metrics_init;

  // Near-met latch. Written by the driver, read-only for strategies. The
  // driver sets it each iteration via nearMetLatched(); the stagnation monitor
  // and the local-slack veto stay inactive until it is set. This models the
  // power-recovery phase of Sharma et al., which starts once timing is within
  // 1% of the target. Once set it stays set; allocate() clears it for each
  // run. When near_met_gate_frac < 0 (the default) it is set from iteration 0,
  // so both are always active.
  bool near_met = false;

  // Phase of termination = threshold_battery (Termination::inPowerPhase()).
  // Written by the driver after each stop check, read-only for strategies.
  // Both stay false for the other terminations and during the estimation
  // loop; allocate() clears them for each run.
  //
  // power_phase: the next iteration's sweeps belong to the power phase. The
  // power-phase filter reads it.
  //
  // prev_sweep_power_phase: the iteration that just ran belonged to the power
  // phase. The sharma_arc_slack update reads it. In Sharma et al., TCAD 2020,
  // Fig. 1, an LR iteration solves the subproblem, runs STA and then updates
  // the multipliers, so a phase ends with the update that follows its last
  // sweep. The driver runs that update at the top of the next iteration, after
  // the stop check that may have ended the phase, so it reads the previous
  // iteration's phase: the first update after the handover still uses the
  // timing-phase exponents.
  bool power_phase = false;
  bool prev_sweep_power_phase = false;

  // 0-based index of the iteration the driver's main loop is running. Written
  // by the driver at the start of each main-loop iteration, read-only for
  // strategies; every sweep of the iteration sees the same index. The
  // estimation-loop dry-run sweeps that may run before the loop do not write
  // it, so they all see 0 and use the exhaustive candidate scan. Its only
  // reader is the sweep engine's move-set choice: sharma_fast_olr switches to
  // Fast-OLR at the paper's 5th LDP iteration, so the sweep needs the
  // iteration index. Other strategies get the iteration number as an
  // argument.
  //
  // Iteration index i and LDP iteration i are the same number for i >= 1.
  // Iteration 0 sweeps before the first lambda update (the driver updates
  // only for iter > 0), so it evaluates the raw seed and is not one of the
  // papers' update-then-solve iterations. There is one more iteration than
  // LDP iterations, but iteration i is the paper's 1-based LDP iteration i,
  // so SweepEngine compares this field with fast_olr_start_iter without an
  // offset.
  int iter = 0;

  // === Mutable multiplier state (sized by allocate) =========================
  // Per-edge multipliers, indexed by sta::Edge::id (sparse).
  std::vector<float> lambda;
  // Per multiplier slot, the ends of the data edge that held it at the last
  // allocate() or refreshLiveEdges(); empty for a slot no live edge held. The
  // same size as lambda.
  std::vector<EdgeEnds> lambda_ends;
  // What refreshLiveEdges() rewrote since allocate(), for the end-of-run
  // report.
  EdgeCarryStats edge_carry;
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

  // === Optional cost-term state, sized by allocate() only when enabled ======
  // Per-edge cumulative back-propagated delay sensitivity phi (Flach et al.,
  // TCAD 2014, Eq. 11), indexed by sta::Edge id like lambda. Empty unless
  // config.cost_global_phi; filled once per iteration by
  // computePhiSensitivities (CostTerms.cc).
  std::vector<float> phi;
  // Per-edge arc delay of the previous iteration's solution (the
  // cost_delta_delay reference), indexed by sta::Edge id. Empty unless
  // config.cost_delta_delay; refreshed once per iteration by
  // captureReferenceDelays (CostTerms.cc). This differs from the reference in
  // Ozdal et al., Eq. 7, which is a load and depends on the candidate (see
  // cost_delta_delay in GlobalSizingConfig.hh).
  std::vector<float> prev_delay;

  // The power the run minimizes, and the activities it reads. Kept here
  // rather than in the subproblem so the driver and the sweep engine read the
  // same model.
  ObjectivePower objective_power;

  // The max-capacitance multipliers (relax_max_cap only). allocate() clears
  // them; the driver starts them before the first sweep and updates them with
  // lambda.
  CapMultipliers cap_multipliers;

  // Finds the graph size (largest data-edge id, endpoints) and sizes lambda, mu
  // and the endpoint bookkeeping, and records the ends of every data edge in
  // lambda_ends. Reads the graph only; strategies fill in the values.
  void allocate();

  // Records the timing before LR: clock period T, worst slack, and per-vertex
  // slack. Call once after the first STA update and before the LR loop.
  void captureInitialTiming();

  // Moves the multipliers onto the live data edges after cell swaps (see
  // carryMultipliers for the rule). When a cell is swapped for one whose
  // timing arcs differ, OpenSTA deletes the instance's edges and creates new
  // ones, which can take recycled ids of unrelated edges or ids past the end of
  // lambda. Without this, a new id would be skipped or priced at 0, and a
  // recycled id would keep the multiplier of its previous edge. Call it after
  // swaps and before the multiplier update, the projection and each sweep.
  // Swaps made during a sweep are not seen until the next call. Returns what
  // this call rewrote and adds it to `edge_carry`.
  EdgeCarryStats refreshLiveEdges();

  // === Shared graph helpers (used by several strategies) ====================
  bool isDataArc(const sta::Edge* edge) const;
  // Max arc delay over rise/fall for `edge` at dcalc_ap (0 if it has no arc
  // set).
  float edgeMaxArcDelay(sta::Edge* edge) const;

  // A self-consistent read of one data edge, for the arrival-based updaters
  // and the Mangiras seed. Three independent worst-of-rise/fall reads
  // (a_from = worst arrival at from_v, d = max arc delay over the edge,
  // a_to = worst arrival at to_v) need not come from the same transitions, so
  // a_from + d can exceed a_to even on the vertex's critical in-edge, which
  // looks like a positive arc violation. This read instead takes the
  // (a_from, d) of the arc and transition that produce the edge's own worst
  // propagated arrival and pairs it with the to-vertex arrival, so
  // a_from + d <= a_to holds by construction, with equality exactly when this
  // edge is the vertex's critical in-edge. Multipliers are still per edge, not
  // per (arc, transition).
  struct ConsistentArcRead
  {
    float a_from = 0.0f;  // from-vertex arrival in the realizing transition
    float d = 0.0f;       // realizing arc's delay
    float a_to = 0.0f;    // to-vertex worst arrival (the vertex constraint)
  };
  ConsistentArcRead consistentArcRead(sta::Edge* edge,
                                      sta::Vertex* from_v,
                                      sta::Vertex* to_v) const;

  // Slack of one data edge: the worst of q_to - a_from - d over its (arc,
  // transition) pairs, where a_from is the from-vertex arrival in the arc's
  // input transition and q_to the to-vertex required time in its output
  // transition. This is the slack of the worst path through the edge (Sharma
  // et al., TCAD 2020, Sec. IV-C-2). Adding the edge's arrival gap to the
  // to-vertex's worst slack does not give it, because that slack and that
  // arrival can come from different transitions. kSlackSentinel when no pair
  // is constrained.
  float arcSlack(sta::Edge* edge, sta::Vertex* from_v, sta::Vertex* to_v) const;
};

// The cell of every leaf instance that is not dont-touch, recorded so it can
// be reinstated later.
class CellAssignment
{
 public:
  void capture(const LrState& state);
  // Swaps every recorded instance whose cell has changed since capture() back
  // to its recorded cell, journaled. Returns the number of cells replaced;
  // the caller refreshes parasitics and timing. The global_sizing_restore
  // debug group prints each replacement.
  int restore(LrState& state) const;

 private:
  std::vector<std::pair<sta::Instance*, sta::LibertyCell*>> cells_;
};

// The core of LrState::refreshLiveEdges, without STA. `slot_ends` gives, per
// multiplier slot, the ends of the edge that held it at the previous refresh,
// and `live_ends` the same for the edges live now. A pin pair is carried when
// its slots changed: one of its live edges is in a slot that held another
// pair or no edge, or a slot it held no longer holds it. For each carried
// pair, the sum of the multipliers in its previous slots is split evenly over
// its live edges. That keeps the per-pin-pair sum, which is what the flow
// projection balances and the sweep's per-pin cost reads; with several arcs
// between two pins (conditional arcs), each gets their average. A carried pair
// with no previous slot starts at 0, a slot no live edge holds is set to 0,
// and every other slot keeps its value.
//
// lambda grows to cover the live edges and never shrinks; slot_ends becomes
// live_ends, padded to lambda's size. On entry lambda and slot_ends have the
// same size, as allocate() and the previous refresh leave them.
EdgeCarryStats carryMultipliers(std::vector<EdgeEnds> live_ends,
                                std::vector<EdgeEnds>& slot_ends,
                                std::vector<float>& lambda);

// One (arc, transition) read of an edge: the from-vertex arrival in the arc's
// input transition and that arc's delay. Their sum is the arrival the pair
// propagates to the to-vertex. Public so pickCriticalArcTransition() can be
// unit-tested without STA.
struct ArcTransitionRead
{
  float a_from = 0.0f;
  float d = 0.0f;
};

// Returns the (a_from, d) pair with the largest a_from + d, i.e. the one that
// produces the edge's worst propagated arrival (no STA). Ties go to the first
// maximum; an empty list returns {0, 0}. This is the selection inside
// LrState::consistentArcRead, kept separate so it can be tested on
// hand-computed values.
ArcTransitionRead pickCriticalArcTransition(
    const std::vector<ArcTransitionRead>& reads);

}  // namespace rsz
