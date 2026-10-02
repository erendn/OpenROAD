// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <optional>
#include <unordered_map>
#include <vector>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "lr/ObjectivePower.hh"
#include "lr/SizeVthGrid.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "rsz/Resizer.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/NetworkClass.hh"
#include "utl/Logger.h"

namespace sta {
class ArcDelayCalc;
class Edge;
class Pin;
class Scene;
}  // namespace sta

namespace rsz {

class CapMultipliers;
class Resizer;

// LRSubproblem: Evaluates the per-gate Lagrangian subproblem
//
//     minimize_{x ∈ S_i}
//       power(x)
//       + Σ_{e ∈ out(i)} λ_e · d_e(x)
//       + Σ_{p ∈ inputs(i)} Σ_{e ∈ arcs_to_drv(p)} λ_e · d_e(U, load_perturbed)
//
// where load_perturbed = load_U - C_in(current x_i, p) + C_in(candidate, p).
// power(x) is the candidate's objective power (lr/ObjectivePower.hh): its
// leakage, or under the total-power objective its leakage plus dynamic power.
// The first sum prices the gate's own internal arcs; the second prices the
// upstream driver U's delay change caused by varying the candidate's input
// capacitance on pin p.
//
// The own-arc term is always on. The upstream-load term, the two output-slew
// terms (fanout slew and global φ) and delta-delay referencing are switched by
// CostTermFlags. The defaults enable only the upstream-load term, which gives
// the expression above, except that by default every arc into a pin is priced
// at the pin's worst arc delay: (Σ_e λ_e) · max_e d_e. CostTermFlags::per_arc
// prices each arc at its own delay, as written.
//
// Under relax_max_cap the cost also prices max capacitance (Livramento et al.,
// DATE 2013, Eq. 3): beta * (load - limit) on the gate's output pins and on
// the drivers of its fanin nets, weighted like the timing terms (see
// lr/CapMultipliers.hh).
//
// === Threading model ========================================================
// The evaluation is split so a Jacobi sweep can run in parallel:
//   - snapshot(inst)         : MAIN THREAD ONLY. Reads live STA state (slews,
//                              load caps, cap checks), fills the lazy Liberty
//                              caches, and freezes everything needed to score a
//                              gate into a GateSnapshot.
//   - evaluateSnapshot(snap) : WORKER SAFE. Reads only `snap` + read-only
//                              Liberty data and the caller-provided per-thread
//                              ArcDelayCalc. No STA graph reads, no shared
//                              arc_delay_calc_, no cache writes, no mutation.
// Replacements chosen by evaluateSnapshot are applied later, serially, via
// applyReplacement.
class LRSubproblem : public sta::dbStaState
{
 public:
  // Optional cost terms (the cost_* options in GlobalSizingConfig). Copied
  // into each GateSnapshot so evaluateSnapshot never reads the config. The
  // defaults enable only the upstream-load term.
  struct CostTermFlags
  {
    bool upstream_load = true;
    bool fanout_slew = false;
    bool global_phi = false;
    bool delta_delay = false;
    // timing_cost = per_arc: price each arc at its own delay instead of the
    // pin's worst.
    bool per_arc = false;
  };

  // One gate-internal timing arc into a driver pin, priced on its own under
  // CostTermFlags::per_arc.
  struct ArcCtx
  {
    // The graph edge's arc set, from the driver's current cell. Its from()
    // port is the arc's input. A candidate cell is priced through its own
    // matching arc set.
    sta::TimingArcSet* arc_set = nullptr;
    float lambda = 0.0f;
    // cost_delta_delay only: this arc's previous-iteration delay.
    float ref_delay = 0.0f;
  };

  // Everything snapshot() reads besides the instance: the per-edge multiplier
  // and cost-term vectors (indexed by sta::Edge::id; null or empty when the
  // term is off), the per-vertex guard data, and the config choices each
  // snapshot freezes. Built once per sweep by sweepInputs() (SweepEngine.hh).
  struct SnapshotInputs
  {
    const float* lambda = nullptr;
    int lambda_size = 0;
    // Per-vertex depth-normalized downsize budget (kDepthBudget only), indexed
    // by sta::Graph vertex id. Empty under the other guards.
    const float* budget = nullptr;
    int budget_size = 0;
    const float* phi = nullptr;  // Flach Eq. 11, cost_global_phi only
    int phi_size = 0;
    const float* prev_delay
        = nullptr;  // incumbent-delay reference, cost_delta_delay only
    int prev_delay_size = 0;
    CostTermFlags cost;
    bool include_clock_network = false;
    // When false, snapshot() skips instances whose cell is sequential.
    bool size_registers = true;
    // The power the cost minimizes. Null prices leakage.
    const ObjectivePower* objective_power = nullptr;
    // Skip every candidate whose objective power is above the current cell's.
    // Set by the sweep engine from power_phase_filter and the run's phase.
    bool power_phase_filter = false;
    // relax_max_cap only: the max-capacitance multipliers the cost prices.
    // Null otherwise, which keeps every max-cap check.
    const CapMultipliers* cap_multipliers = nullptr;

    // === Downsize guard inputs ==============================================
    GlobalSizingConfig::DownsizeGuard guard
        = GlobalSizingConfig::DownsizeGuard::kDepthBudget;
    // kLocalSlackVeto only. Flach's Eq. 14 gamma for this sweep.
    float gamma = 1.0f;
    // kLocalSlackVeto only: per-vertex slack frozen at sweep start, indexed by
    // sta::Graph vertex id. Read when live_slacks is false - i.e. under Jacobi
    // (no per-commit refresh exists) and under gs_local (required times are
    // deliberately not propagated per commit, so the graph's are stale and
    // querying them would silently force a full timing update).
    const float* vertex_slack = nullptr;
    int vertex_slack_size = 0;
    // kLocalSlackVeto only: read slacks live from the graph instead of the
    // frozen vector. Set under gs_incremental, whose per-commit findRequireds
    // keeps them valid, so the veto sees each gate's post-commit timing. This
    // is what makes gs_refresh affect which cells are chosen.
    bool live_slacks = false;
    // kLocalSlackVeto only: the near-met latch (LrState::near_met) for this
    // sweep. When false the veto passes every candidate (Sharma applies it
    // only during power recovery). Without near-met gating it is always true.
    bool guard_active = true;

    // === Candidate move set =================================================
    GlobalSizingConfig::MoveSet move_set
        = GlobalSizingConfig::MoveSet::kFullLibrary;
    // kSharmaFastOlr only: has this sweep reached the switch-over iteration?
    // Before it, the sweep runs the exhaustive scan (the paper does so for its
    // first four iterations). Resolved by the sweep engine from LrState::iter
    // and fast_olr_start_iter so evaluateSnapshot reads no run state.
    bool fast_olr_active = false;

    // === Output-side DRC veto ===============================================
    // kRelative makes snapshot() freeze each output pin's current excess (see
    // OutputCtx) and candidateDrcOkSnapshot judge a candidate against it
    // instead of against zero. Under the kAbsolute default nothing extra is
    // frozen and a candidate must be within every output limit.
    GlobalSizingConfig::OutputDrcVeto output_drc_veto
        = GlobalSizingConfig::OutputDrcVeto::kAbsolute;
  };

  // Per-input-pin upstream context, captured once per snapshot() call and
  // reused across every candidate cell. Each entry corresponds to one input
  // pin of the instance whose driver belongs to a real upstream standard cell.
  // Pins with no driver (PIs), driverless nets, or whose upstream sum-of-λ is
  // at the floor are filtered out at build time.
  struct UpstreamCtx
  {
    // Input port of the instance under its current cell. Used to look up
    // the same port (by name) on each candidate cell so we can read the
    // candidate's input capacitance for this pin.
    const sta::LibertyPort* orig_in_port = nullptr;
    // Output port of the upstream driver U at this pin's driver. Constant
    // across candidates - only the load it sees changes per candidate.
    sta::LibertyPort* drv_port = nullptr;
    // Current load capacitance at U's driver pin (farads). Includes the current
    // cell's contribution; we subtract C_in(current) and add C_in(candidate) to
    // get the perturbed load each candidate.
    float load_U_cur = 0.0f;
    // Input capacitance on this pin under the instance's CURRENT cell.
    float c_in_cur = 0.0f;
    // Σλ over U's gate-internal data arcs that terminate at the driver pin.
    // These are the arcs whose delay depends on the load U drives.
    float lambda_U_drv = 0.0f;
    // Delta-delay reference: previous-iteration delay of U's driver arc (max
    // over its gate-internal arcs). Only frozen when cost_delta_delay is on;
    // the perturbed delay is priced relative to it.
    float ref_delay = 0.0f;
    // CostTermFlags::per_arc only: the arcs summed into lambda_U_drv.
    std::vector<ArcCtx> arcs;
    // kLocalSlackVeto only: slack at U's driver pin - this input pin's "driver
    // net" in Flach's veto. Frozen (sweep-start) or live per SnapshotInputs.
    float slack = 0.0f;
    // kLocalSlackVeto only: U's driver-arc delay at its CURRENT load, i.e. the
    // reference the perturbed delay is measured against. Candidate-invariant,
    // so it is frozen here rather than recomputed for every candidate.
    float d_drv_cur = 0.0f;
    // kLocalSlackVeto only: false when an earlier input pin of this gate is fed
    // by the SAME driver pin. Flach sums the local negative slack over distinct
    // driver *nets*, so a shared net must contribute once - but each pin still
    // needs its own entry, because each perturbs the driver's load separately.
    bool veto_counts = true;
    // kLocalSlackVeto only: the driver pin, used only to detect the shared-net
    // case above.
    const sta::Pin* drv_pin = nullptr;
  };

  // Frozen per-output-pin electrical state for one instance.
  struct OutputCtx
  {
    // Output port under the instance's current cell. Candidate ports are looked
    // up by name on each candidate cell.
    const sta::LibertyPort* port = nullptr;
    float load_cap = 0.0f;    // graph_delay_calc_->loadCap (frozen)
    float lambda_sum = 0.0f;  // Σλ over gate-internal arcs into this pin
    // CostTermFlags::per_arc only: the arcs summed into lambda_sum.
    std::vector<ArcCtx> arcs;
    // Elmore-slew DRC inputs (frozen). slew is the STA graph slew at this pin's
    // load vertex; drive_res is the current port's drive resistance.
    // The candidate's output slew is estimated as
    //   slew/(drive_res*load_cap) * cand_drive_res * load_cap.
    float slew = 0.0f;
    float drive_res = 0.0f;
    // outputSlewFactor(slew, drive_res, load_cap): the slew calibration every
    // candidate's estimate is scaled from. It depends only on the fields
    // above, so it is computed once here rather than per candidate.
    float slew_factor = 0.0f;
    // output_drc_veto = kRelative only: the current cell's own excess over
    // this pin's max-cap (farads) and max-slew (seconds) limits, clamped at 0.
    // Under the relative rule a candidate may not exceed these. Both stay 0
    // under kAbsolute, which skips the limit lookups.
    //
    // The slew excess uses the estimated slew (slew_factor with the current
    // port), not the measured graph slew, so the current cell and the
    // candidates are compared on the same model; see snapshot().
    //
    // Both values are recomputed from the live load and slew every time
    // snapshot() runs, i.e. once per gate per sweep. They are not a floor for
    // the rest of the run: if the net's load grows, the allowed excess grows
    // with it.
    float cap_excess_cur = 0.0f;
    float slew_excess_cur = 0.0f;
    // relax_max_cap only: this pin's max-capacitance multiplier. A pin with
    // one is priced instead of checked against its max capacitance; a pin
    // without one keeps the check.
    std::optional<float> cap_beta;
    // === Optional cost-term data, filled only when the term is on ===========
    // cost_fanout_slew: Σ over the immediate fanout (sink) arcs of
    // λ_ik·(δd_ik/δslew), the per-pin fanout-slew sensitivity (Livramento
    // Alg. 2 lines 12-14, linearized). Priced against the candidate's
    // output-slew change.
    float fanout_slew_sens = 0.0f;
    // cost_global_phi: Σ φ over the arcs driven by this output net (Flach
    // Eq. 13 at the gate's own net) = Σ φ over the pin's out data edges. φ
    // already accumulates the whole downstream cone, so this one scalar prices
    // the global downstream impact of the candidate's output-slew change.
    float phi_sink_sum = 0.0f;
    // cost_delta_delay: previous-iteration reference delay for this output pin
    // (max prev_delay over the gate-internal arcs into it). The own-arc term is
    // priced relative to it.
    float ref_delay = 0.0f;
    // kLocalSlackVeto only: slack at this output pin - the gate's "sink net" in
    // Flach's veto. Frozen (sweep-start) or live per SnapshotInputs.
    float slack = 0.0f;
    // kLocalSlackVeto only: the current cell's arc delay at this pin's load,
    // i.e. the reference the candidate's delay is measured against.
    // Candidate-invariant, so it is frozen here rather than recomputed for
    // every candidate.
    float d_cur = 0.0f;
  };

  // Snapshot of one driver pin's max-cap check on a fanin net.
  struct DriverCapCheck
  {
    float cap = 0.0f;        // current load cap at the driver pin
    float max_cap = 0.0f;    // cap limit
    float cap_slack = 0.0f;  // current cap slack
    bool corner_ok = false;  // max_cap > 0 && a corner was returned
    // relax_max_cap only: the driver pin has a max-capacitance multiplier, so
    // the cost prices its load (GateSnapshot::fanin_cap_betas) instead of this
    // check.
    bool priced = false;
  };

  // relax_max_cap only: a fanin driver pin with a max-capacitance multiplier,
  // and the gate's input pins on its net. Each driver appears once, however
  // many of the gate's input pins it feeds.
  struct FaninCapBeta
  {
    const sta::Pin* driver = nullptr;
    float beta = 0.0f;
    float load = 0.0f;   // current load cap at the driver pin
    float limit = 0.0f;  // Liberty max capacitance of the driver's current cell
    // Indices into GateSnapshot::inputs of the gate's input pins on the net.
    std::vector<size_t> inputs;
  };

  // Per-input-pin context for the input-side max-cap DRC
  // (Resizer::replacementPreservesMaxCap, frozen).
  struct InputMaxCapCtx
  {
    const sta::LibertyPort* in_port = nullptr;  // current cell's input port
    float old_cap = 0.0f;  // input pin cap under the CURRENT cell
    std::vector<DriverCapCheck> drivers;
  };

  // One swappable candidate with its precomputed leakage-equivalent cost.
  // The leakage ranks the candidate on the size/Vth grid and decides whether
  // it is a downsize. The cost reads the candidate's objective power instead,
  // which evaluateSnapshot computes from it.
  struct Candidate
  {
    sta::LibertyCell* cell = nullptr;
    float leakage = 0.0f;  // leakageOrArea(cell), precomputed on main thread
    // Position on the group's (width rank x Vth flavor) grid (lr/SizeVthGrid).
    // Frozen on the main thread (the flavor key is a dbMaster query); read only
    // by the two restricted move sets, and left at the default under
    // kFullLibrary, which never builds the grid.
    GridCoord grid;
  };

  // Everything evaluateSnapshot needs to score one instance, frozen on the
  // main thread.
  struct GateSnapshot
  {
    sta::Instance* inst = nullptr;
    sta::LibertyCell* cur_cell = nullptr;
    float cur_leakage = 0.0f;
    const sta::Scene* scene = nullptr;
    // Distributed downsize budget for this gate: the min over its output pins
    // of the depth-normalized slack budget  max(0, slack - margin) / depth,
    // frozen on the main thread (computed by the policy's computeSlackBudgets).
    // A downsize may add at most this much delay on any output pin (times a
    // safety factor). Because the per-path sum of these budgets is <= the path
    // slack, simultaneous (Jacobi) downsizes within budget cannot overshoot a
    // path.
    float budget = 0.0f;
    // Cost-term toggles frozen from the config so evaluateSnapshot reads no
    // shared state.
    CostTermFlags cost;
    // === Downsize guard =====================================================
    GlobalSizingConfig::DownsizeGuard guard
        = GlobalSizingConfig::DownsizeGuard::kDepthBudget;
    float gamma = 1.0f;
    // kLocalSlackVeto only (near-met gate): when false, the veto passes every
    // candidate this sweep.
    bool guard_active = true;
    // Flach's originalSlack: the gate's local negative slack under its CURRENT
    // cell, i.e. the sum of the negative slacks over its driver nets (the
    // upstream entries) and its sink net (the output pins). <= 0.
    float local_slack_orig = 0.0f;
    // === Candidate move set =================================================
    GlobalSizingConfig::MoveSet move_set
        = GlobalSizingConfig::MoveSet::kFullLibrary;
    bool fast_olr_active = false;
    // === Output-side DRC veto ===============================================
    GlobalSizingConfig::OutputDrcVeto output_drc_veto
        = GlobalSizingConfig::OutputDrcVeto::kAbsolute;
    // The current cell's position on the grid. `candidates` excludes the
    // current cell, so this is the gap in its own column, which is where both
    // restricted move sets start from.
    GridCoord cur_grid;
    // The group's column layout, addressable by (flavor, width rank). Borrowed
    // from LRSubproblem's per-library-cell cache, not owned: the grid is a pure
    // function of the group's members, so it is built once per cell rather than
    // once per instance per sweep. Null unless a restricted move set actually
    // reads a coord this sweep.
    const GridLayout* grid = nullptr;
    std::vector<OutputCtx> outputs;
    std::vector<UpstreamCtx> upstream;
    std::vector<InputMaxCapCtx> inputs;
    std::vector<Candidate> candidates;  // excludes cur_cell
    // The objective-power model and this gate's inputs to it. power_inputs is
    // filled only under the total-power objective.
    const ObjectivePower* objective_power = nullptr;
    GatePowerInputs power_inputs;
    // See SnapshotInputs::power_phase_filter.
    bool power_phase_filter = false;
    // The cost prices max capacitance (SnapshotInputs::cap_multipliers).
    bool relax_max_cap = false;
    std::vector<FaninCapBeta> fanin_cap_betas;
  };

  // Result of one per-gate evaluation, applied later in serial.
  struct GateDecision
  {
    sta::Instance* inst = nullptr;
    sta::LibertyCell* best_cell = nullptr;  // nullptr -> keep current
    float best_cost = 0.0f;                 // power + Σλ·d at best_cell
    float baseline_cost = 0.0f;             // same for current cell
    // True iff best_cell has strictly lower leakage-equivalent cost than the
    // current cell. Used by the outer loop to apply asymmetric acceptance:
    // any cost drop is enough on a downsize, but timing-noise hysteresis
    // still applies to upsizes. False when best_cell == nullptr.
    bool best_is_downsize = false;
  };

  explicit LRSubproblem(Resizer* resizer);
  ~LRSubproblem() override = default;

  void init();

  // MAIN THREAD ONLY. Capture the frozen state needed to evaluate `inst` (see
  // SnapshotInputs for what it reads). Returns false (and leaves `snap`
  // unspecified) when `inst` is don't-touch, has no liberty cell, or has no
  // usable output pin. The gate's frozen budget is the min over its output pins
  // of in.budget; its frozen local negative slack is the sum over its driver
  // nets and sink net.
  bool snapshot(sta::Instance* inst,
                const SnapshotInputs& in,
                GateSnapshot& snap);

  // WORKER SAFE. Evaluate the subproblem for a prepared snapshot using the
  // caller-provided per-thread ArcDelayCalc. `timing_weight` scales the Σλ·d
  // timing term against the power term. `budget_safety` (<= 1) scales
  // the gate's frozen downsize budget in the feasibility guard.
  GateDecision evaluateSnapshot(const GateSnapshot& snap,
                                float timing_weight,
                                float budget_safety,
                                sta::ArcDelayCalc* arc_delay_calc) const;

  // Leakage-equivalent cost for `cell`. Returns Resizer::cellLeakage when
  // the Liberty exposes leakage; otherwise returns area · area-to-leakage
  // scale (computed once at init() from the current design's distribution
  // of leakage and area on cells that DO have leakage). Mutates a lazy cache;
  // call only on the main thread.
  float leakageOrArea(sta::LibertyCell* cell) const;

  // Apply the LR-chosen replacement at `inst`. Wraps Resizer::replaceCell;
  // returns true on success. Called from GlobalSizingPolicy in serial inside
  // an open pass-level journal.
  bool applyReplacement(sta::Instance* inst, sta::LibertyCell* replacement);

  // WORKER SAFE. The cost of `cell` at the snapshotted instance from its own
  // power and arcs only: its objective power (given its leakage-equivalent
  // cost `leakage`) plus timing_weight * sum(lambda * d) over the arcs into
  // its output pins, priced as in evaluateSnapshot. The terms that price the
  // gate's fanin and fanout are left out. Infinity when `cell` lacks one of
  // the output pins. Used by the max-capacitance fix pass
  // (lr/ViolationRepair.hh).
  float ownCost(const GateSnapshot& snap,
                sta::LibertyCell* cell,
                float leakage,
                float timing_weight,
                sta::ArcDelayCalc* arc_delay_calc) const;

  // WORKER SAFE. The max-capacitance terms of `cell` at the snapshotted
  // instance (relax_max_cap), before the timing weight; 0 without
  // multipliers. capBetaOwnCost sums beta * (load - limit) over the output
  // pins that have a multiplier, at the frozen load and `cell`'s own Liberty
  // limit (Livramento et al., Alg. 2 line 11). capBetaFaninCost sums it over
  // the fanin drivers that have one, at the driver's limit and at its frozen
  // load with `cell`'s input capacitance in place of the current cell's on
  // every input pin on the driver's net (line 8).
  float capBetaOwnCost(const GateSnapshot& snap, sta::LibertyCell* cell) const;
  float capBetaFaninCost(const GateSnapshot& snap,
                         sta::LibertyCell* cell) const;

  // MAIN THREAD ONLY. The cell's Vth-flavor identity for the size/Vth grid,
  // cached per Liberty cell (Resizer::cellVTType's own map is keyed by dbMaster
  // and is also main-thread-only). 0 for a cell with no dbMaster.
  int vthFlavorKey(sta::LibertyCell* cell);

 private:
  bool isDataArc(const sta::Edge* edge) const;
  // Walks leaf instances once to populate area_to_leakage_scale_ and
  // expose any pure-area-only-library degenerate case.
  void computeLeakageScale();
  // The group's decomposition, memoized by the incumbent's library cell:
  // `getSwappableCells` is itself cached per source cell and returns a stable
  // sequence, and every input the grid reads (the members' flavor keys and
  // leakage-equivalent costs) is a property of those cells, so one entry serves
  // every instance holding that cell for the whole run.
  struct CachedGrid
  {
    // Coord per group member, in group order: index 0 is the incumbent and
    // index i+1 is candidates[i].
    std::vector<GridCoord> coords;
    GridLayout layout;
  };
  // MAIN THREAD ONLY. Point snap.grid at the group's cached decomposition and
  // fill snap.cur_grid and each candidate's coord from it. A no-op when this
  // sweep reads no coord: under move_set = full_library, and under
  // sharma_fast_olr before its switch-over iteration, where the sweep runs the
  // exhaustive scan.
  void buildCandidateGrid(GateSnapshot& snap);
  // MAIN THREAD ONLY. The cache lookup behind it; builds the entry on a miss.
  const CachedGrid& cachedGrid(sta::LibertyCell* cur_cell,
                               const std::vector<Candidate>& candidates,
                               float cur_leakage);

  // Worker-safe cost of running `cell` at the snapshotted instance.
  // `cell_power` is the objective power of `cell` at this instance.
  float evaluateCellCost(const GateSnapshot& snap,
                         sta::LibertyCell* cell,
                         float cell_power,
                         float timing_weight,
                         sta::ArcDelayCalc* arc_delay_calc) const;

  // Read the max-rise/fall input capacitance of `port` on `cell` (farads).
  // Returns 0 if the port is missing on the cell. Worker-safe (Liberty read).
  float portInputCap(sta::LibertyCell* cell, const char* port_name) const;

  // Worker-safe Σλ·d of output pin `o` with `cell` installed. `cand_port` is
  // the pin's port on `cell`.
  float ownArcsLambdaDelay(const GateSnapshot& snap,
                           const OutputCtx& o,
                           const sta::LibertyCell* cell,
                           const sta::LibertyPort* cand_port,
                           sta::ArcDelayCalc* arc_delay_calc) const;
  // Worker-safe Σλ·d of the upstream driver's arcs into its pin at `load`.
  float upstreamArcsLambdaDelay(const GateSnapshot& snap,
                                const UpstreamCtx& u,
                                float load,
                                sta::ArcDelayCalc* arc_delay_calc) const;
  // Worker-safe Σ λ·d over `arcs` with every arc on `cell` at `load`: the
  // cell the arcs belong to, or a candidate for it. Under cost_delta_delay
  // each delay is taken relative to its arc's reference.
  float perArcLambdaDelay(const GateSnapshot& snap,
                          const std::vector<ArcCtx>& arcs,
                          const sta::LibertyCell* cell,
                          float load,
                          sta::ArcDelayCalc* arc_delay_calc) const;
  // Worker-safe delay of the arc `arc_set` (from another cell of `cell`'s
  // group, or from `cell` itself) on `cell` at `load`.
  float arcDelayOnCell(sta::TimingArcSet* arc_set,
                       const sta::LibertyCell* cell,
                       float load,
                       const sta::Scene* scene,
                       sta::ArcDelayCalc* arc_delay_calc) const;

  // Worker-safe objective power of `cell` at the snapshotted instance, given
  // its leakage-equivalent cost `leakage`.
  float objectivePower(const GateSnapshot& snap,
                       sta::LibertyCell* cell,
                       float leakage) const;

  // Worker-safe DRC filter over a frozen snapshot. Returns true iff installing
  // `replacement` is electrically admissible - on the input side (fanin nets
  // due to larger input pin caps) and on each output pin (current load cap
  // against the new cell's cap limit, and estimated output slew against the new
  // cell's drive resistance).
  //
  // The input side always asks the relative question: a fanin net already
  // over its limit rejects only a candidate that adds load to it. The output
  // side asks whichever question snap.output_drc_veto names: the absolute
  // "introduces no violation" by default, or the relative "worsens no
  // existing violation" when output_drc_veto = relative.
  //
  // Under relax_max_cap the max-cap checks skip every pin that has a
  // multiplier, since the cost prices its load instead; the slew check stays.
  bool candidateDrcOkSnapshot(const GateSnapshot& snap,
                              sta::LibertyCell* replacement) const;

  // Worker-safe downsize feasibility guard over a frozen snapshot. Returns true
  // iff installing the (lower-leakage) `replacement` adds, on every output pin,
  // no more delay than `safety * snap.budget`. snap.budget is the depth-
  // normalized, distributed slack budget frozen by the policy: because the
  // per-path sum of gate budgets is <= path slack, simultaneous downsizes
  // within budget cannot overshoot, so no per-gate discount is needed. A gate
  // with no budget (<= 0) cannot be downsized.
  bool downsizeFitsSlackBudget(const GateSnapshot& snap,
                               sta::LibertyCell* replacement,
                               float safety,
                               sta::ArcDelayCalc* arc_delay_calc) const;

  // Worker-safe local-negative-slack veto (Flach Alg. 4 lines 12-14) over a
  // frozen snapshot. Recomputes the gate's local negative slack with
  // `replacement` installed - each sink net's slack shifted by the candidate's
  // own-arc delay change, each driver net's by the delay change its perturbed
  // load causes on that driver - and accepts iff the result is no worse than
  // gamma * snap.local_slack_orig. Applies to every candidate, upsizes
  // included: it is Flach's acceptance test, not a downsize-only guard.
  bool candidatePassesLocalSlackVeto(const GateSnapshot& snap,
                                     sta::LibertyCell* replacement,
                                     sta::ArcDelayCalc* arc_delay_calc) const;

  Resizer* resizer_ = nullptr;
  utl::Logger* logger_ = nullptr;
  sta::dbNetwork* db_network_ = nullptr;

  // Computed at init() from this design's (leakage, area) distribution on
  // instances whose current cell exposes Liberty leakage. Used by
  // leakageOrArea() to give area-only cells a leakage-equivalent cost.
  // Zero when no instance exposes leakage (degenerate area-only case).
  float area_to_leakage_scale_ = 0.0f;

  // Lazy Liberty-cell -> Vth-flavor key cache for the size/Vth grid. Stays
  // empty under move_set = full_library (nothing ever queries it).
  std::unordered_map<sta::LibertyCell*, int> vt_flavor_cache_;
  // Lazy incumbent-cell -> (coords, layout) cache, so a restricted move set
  // decomposes each swappable group once per run rather than once per instance
  // per sweep.
  std::unordered_map<sta::LibertyCell*, CachedGrid> grid_cache_;

  const sta::MinMax* max_ = sta::MinMax::max();
  bool initialized_ = false;
};

}  // namespace rsz
