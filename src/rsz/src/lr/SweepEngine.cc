// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "SweepEngine.hh"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <ranges>
#include <utility>
#include <vector>

#include "CapRecheck.hh"
#include "Guards.hh"
#include "LRSubproblem.hh"
#include "TimingScale.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "rsz/GlobalSizingConfig.hh"
#include "rsz/Resizer.hh"
#include "sta/ArcDelayCalc.hh"
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
#include "utl/Logger.h"
#include "utl/ThreadPool.h"

namespace rsz {

using utl::RSZ;

namespace {

// Freeze every vertex's slack at sweep start, indexed by vertex id. Both the
// depth-budget guard (which distributes these slacks) and the local-slack veto
// (which tests candidates against them, except under gs_incremental where it
// reads live slacks - see LRSubproblem::SnapshotInputs) consume this vector, so
// the STA slack queries happen exactly once per sweep here.
// Returns the vertices it walked, so computeSlackBudgets can reuse the
// enumeration instead of redoing it.
std::vector<sta::Vertex*> captureVertexSlacks(LrState& state)
{
  sta::Graph* graph = state.graph;
  size_t max_id = 0;
  sta::VertexIterator vit(graph);
  std::vector<sta::Vertex*> vertices;
  while (vit.hasNext()) {
    sta::Vertex* v = vit.next();
    vertices.push_back(v);
    max_id = std::max(max_id, static_cast<size_t>(graph->id(v)));
  }
  state.vertex_slack.assign(max_id + 1, 0.0f);
  for (sta::Vertex* v : vertices) {
    state.vertex_slack[graph->id(v)]
        = sta::delayAsFloat(state.sta->slack(v, state.max));
  }
  return vertices;
}

// Shared by both sweep engines (F1). Per-vertex downsize budget =
// max(0, slack(v) - margin) / depth(v), where depth(v) is the gate count on the
// longest path through v. Distributing by depth bounds the per-path budget sum
// by the path slack; using v's own (worst-path) slack keeps each gate safe on
// all its paths. Recomputed once per sweep from the sweep-start slacks
// (state.vertex_slack, filled by captureVertexSlacks). Under the sequential
// Gauss-Seidel engine the budgets are NOT refreshed mid-sweep, which is
// conservative-but-valid: each downsize adds at most its depth-normalized
// budget and the per-path budget sum is <= the path slack, so committing gates
// one at a time cannot overshoot a path even against the frozen sweep-start
// budgets.
void computeSlackBudgets(LrState& state, std::vector<sta::Vertex*> vertices)
{
  sta::Graph* graph = state.graph;
  sta::Network* network = state.network;
  const size_t n = state.vertex_slack.size();
  std::ranges::sort(vertices, [](const sta::Vertex* a, const sta::Vertex* b) {
    return a->level() < b->level();
  });

  // A gate-internal (cell) arc has both pins on the same leaf instance; only
  // these add a gate-delay term to a path, so only these increment the depth.
  auto is_gate_arc = [graph, network](sta::Edge* e) {
    const sta::Instance* fi = network->instance(e->from(graph)->pin());
    const sta::Instance* ti = network->instance(e->to(graph)->pin());
    return fi != nullptr && fi == ti;
  };

  // Forward pass (increasing level): Gates from a source up to and including v
  std::vector<int> fwd(n, 0);
  for (sta::Vertex* v : vertices) {
    int best = 0;
    sta::VertexInEdgeIterator ieit(v, graph);
    while (ieit.hasNext()) {
      sta::Edge* e = ieit.next();
      if (!state.isDataArc(e)) {
        continue;
      }
      const sta::VertexId uid = graph->id(e->from(graph));
      best = std::max(best, fwd[uid] + (is_gate_arc(e) ? 1 : 0));
    }
    fwd[graph->id(v)] = best;
  }

  // Backward pass (decreasing level): Gates from v (exclusive) to a sink
  std::vector<int> bwd(n, 0);
  for (sta::Vertex* v : std::views::reverse(vertices)) {
    int best = 0;
    sta::VertexOutEdgeIterator oeit(v, graph);
    while (oeit.hasNext()) {
      sta::Edge* e = oeit.next();
      if (!state.isDataArc(e)) {
        continue;
      }
      const sta::VertexId wid = graph->id(e->to(graph));
      best = std::max(best, bwd[wid] + (is_gate_arc(e) ? 1 : 0));
    }
    bwd[graph->id(v)] = best;
  }

  const float margin = state.config->setup_slack_margin;
  const float kSlackSentinel = 1e6f;
  state.vertex_budget.assign(n, 0.0f);
  for (sta::Vertex* v : vertices) {
    const sta::VertexId vid = graph->id(v);
    const int depth = std::max(1, fwd[vid] + bwd[vid]);
    const float slack = state.vertex_slack[vid];
    // Unconstrained vertices (no real required time) report a sentinel slack;
    // leave them effectively unbudgeted so genuinely free gates can downsize.
    state.vertex_budget[vid]
        = (slack >= kSlackSentinel)
              ? kSlackSentinel
              : std::max(0.0f, slack - margin) / static_cast<float>(depth);
  }
  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR budgets: {} vertices (max id {}), margin={}",
             vertices.size(),
             n - 1,
             sta::delayAsString(margin, 3, state.sta));
}

// Sweep-start guard state, shared by both engines. Each half only runs for the
// guard that reads it: the depth passes for depth_budget, the sweep-start slack
// freeze for depth_budget (which distributes it) and the local-slack veto
// (which tests candidates against it). Under downsize_guard=none neither runs,
// so the sweep does no per-vertex STA work at all.
void prepareGuardState(LrState& state)
{
  const GlobalSizingConfig::DownsizeGuard guard = state.config->downsize_guard;
  if (guard == GlobalSizingConfig::DownsizeGuard::kNone) {
    // Empty vectors make every gate's frozen budget the +inf sentinel and its
    // frozen slacks 0 - neither is read (evaluateSnapshot runs no guard).
    state.vertex_slack.clear();
    state.vertex_budget.clear();
    return;
  }
  std::vector<sta::Vertex*> vertices = captureVertexSlacks(state);
  if (guard == GlobalSizingConfig::DownsizeGuard::kDepthBudget) {
    computeSlackBudgets(state, std::move(vertices));
  } else {
    state.vertex_budget.clear();
  }
}

}  // namespace

// Package the per-run multiplier / cost-term / guard vectors once per sweep.
// The gamma the local-slack veto tests against (Flach Eq. 14) is computed here,
// from the sweep-start WNS: the paper re-reads "the current global worst slack"
// per gate, but that is a global query no engine can serve mid-sweep without a
// full timing update (and gs_local deliberately has none), so gamma is a
// per-sweep constant - it tracks the same convergence trend across iterations,
// which is all the schedule needs (adaptation note).
LRSubproblem::SnapshotInputs sweepInputs(LrState& state)
{
  const GlobalSizingConfig& cfg = *state.config;
  LRSubproblem::SnapshotInputs in;
  in.lambda = state.lambda.data();
  in.lambda_size = static_cast<int>(state.lambda.size());
  in.budget = state.vertex_budget.data();
  in.budget_size = static_cast<int>(state.vertex_budget.size());
  in.phi = state.phi.data();
  in.phi_size = static_cast<int>(state.phi.size());
  in.prev_delay = state.prev_delay.data();
  in.prev_delay_size = static_cast<int>(state.prev_delay.size());
  in.cost = {.upstream_load = cfg.cost_upstream_load,
             .fanout_slew = cfg.cost_fanout_slew,
             .global_phi = cfg.cost_global_phi,
             .delta_delay = cfg.cost_delta_delay};
  in.include_clock_network = cfg.include_clock_network;
  in.move_set = cfg.move_set;
  in.output_drc_veto = cfg.output_drc_veto;
  // Sharma Fig. 9's switch-over: "replaces exhaustive candidate evaluation from
  // the 5th LDP iteration onward" (sharma_et_al.md §5.2). fastOlrActive's doc
  // carries the sweep-index-vs-LDP-iteration argument (re-audit delta D2).
  in.fast_olr_active = fastOlrActive(state.iter, cfg.fast_olr_start_iter);
  in.guard = cfg.downsize_guard;
  in.vertex_slack = state.vertex_slack.data();
  in.vertex_slack_size = static_cast<int>(state.vertex_slack.size());
  if (cfg.downsize_guard
      == GlobalSizingConfig::DownsizeGuard::kLocalSlackVeto) {
    const float wns = sta::delayAsFloat(state.sta->worstSlack(state.max));
    in.gamma = flachGamma(wns, state.T, cfg.gamma_local_slack);
    // C2: gate the veto on the near-met latch. Ungated presets keep near_met
    // set from iteration 0, so this is always true and the veto is unchanged.
    in.guard_active = state.near_met;
    // Live slacks only where required times are actually fresh mid-sweep.
    in.live_slacks
        = (cfg.sweep_engine
               == GlobalSizingConfig::SweepEngineKind::kGaussSeidelTopo
           && cfg.gs_refresh == GlobalSizingConfig::GsRefresh::kIncremental);
    debugPrint(state.logger,
               RSZ,
               "global_sizing",
               2,
               "LR veto: gamma={:.4g} (wns={:.4g} T={:.4g}) slacks={}",
               in.gamma,
               wns,
               state.T,
               in.live_slacks ? "live" : "sweep-start");
  }
  return in;
}

float topoTraversalKey(sta::Network* network,
                       sta::Graph* graph,
                       sta::Instance* inst)
{
  float key = std::numeric_limits<float>::max();
  std::unique_ptr<sta::InstancePinIterator> pit(network->pinIterator(inst));
  while (pit->hasNext()) {
    sta::Pin* pin = pit->next();
    if (!network->direction(pin)->isOutput()) {
      continue;
    }
    sta::Vertex* v = graph->pinDrvrVertex(pin);
    if (v != nullptr) {
      key = std::min(key, static_cast<float>(v->level()));
    }
  }
  return key;
}

void orderTraversal(std::vector<TraversalEntry>& entries,
                    const GlobalSizingConfig::Traversal traversal)
{
  const bool descending
      = (traversal == GlobalSizingConfig::Traversal::kReverseTopo);
  std::sort(entries.begin(),
            entries.end(),
            [descending](const TraversalEntry& a, const TraversalEntry& b) {
              if (a.key != b.key) {
                return descending ? (a.key > b.key) : (a.key < b.key);
              }
              return a.tiebreak < b.tiebreak;
            });
}

bool acceptGateMove(const LRSubproblem::GateDecision& decision,
                    const float upsize_hysteresis)
{
  const float tol = decision.best_is_downsize ? 0.0f : upsize_hysteresis;
  return decision.best_cost < decision.baseline_cost * (1.0f - tol);
}

////////////////////////////////////////////////////////////////
// Jacobi (parallel-snapshot) engine

JacobiSnapshotSweep::JacobiSnapshotSweep(Resizer* resizer,
                                         utl::ThreadPool* thread_pool)
    : resizer_(resizer),
      thread_pool_(thread_pool),
      subproblem_(std::make_unique<LRSubproblem>(resizer))
{
}

JacobiSnapshotSweep::~JacobiSnapshotSweep() = default;

void JacobiSnapshotSweep::init(LrState& /* state */)
{
  subproblem_->init();
}

std::vector<LRSubproblem::GateSnapshot> JacobiSnapshotSweep::buildSnapshots(
    LrState& state)
{
  // Phase A (main thread, delays valid): freeze each evaluable gate's
  // timing/DRC state. snapshot() also reads loadCap/slew and warms the lazy
  // getSwappableCells / cellLeakage / net-driver caches, so the subsequent
  // parallel phase touches none of them. The B2 phi / prev_delay vectors and
  // the F3 guard context are frozen too (each empty/neutral when its flag is
  // off); the cost flags and the guard come from the frozen config. The
  // eligibility filter (don't-touch, no-liberty, clock-network exclusion, no
  // usable output pin) lives in snapshot(), shared with the GS engine.
  const LRSubproblem::SnapshotInputs in = sweepInputs(state);
  std::vector<LRSubproblem::GateSnapshot> snapshots;
  std::unique_ptr<sta::LeafInstanceIterator> iit(
      state.network->leafInstanceIterator());
  while (iit->hasNext()) {
    sta::Instance* inst = iit->next();
    LRSubproblem::GateSnapshot snap;
    if (subproblem_->snapshot(inst, in, snap)) {
      snapshots.push_back(std::move(snap));
    }
  }
  return snapshots;
}

SweepEngine::Stats JacobiSnapshotSweep::applyDecisions(
    LrState& state,
    const std::vector<LRSubproblem::GateDecision>& decisions,
    const int visited,
    std::vector<MovedGate>& movers)
{
  // Phase C (main thread, serial): apply accepted replacements in the snapshot
  // vector order so the result is independent of worker scheduling. Pure apply
  // loop - no slack/slew/arrival query may run here, or the single batched
  // timing update in the driver would fragment into many.
  sta::Network* network = state.network;
  const float upsize_hysteresis = state.config->upsize_hysteresis;
  int moves = 0;
  int evaluated = 0;
  int downsizes = 0;
  int upsizes = 0;

  for (const LRSubproblem::GateDecision& r : decisions) {
    if (r.best_cell == nullptr) {
      continue;
    }
    ++evaluated;
    if (acceptGateMove(r, upsize_hysteresis)) {
      sta::LibertyCell* prev = network->libertyCell(r.inst);
      if (subproblem_->applyReplacement(r.inst, r.best_cell)) {
        ++moves;
        movers.push_back({.inst = r.inst,
                          .prev_cell = prev,
                          .was_downsize = r.best_is_downsize});
        const float rel_gain
            = r.baseline_cost > 0.0f
                  ? (r.baseline_cost - r.best_cost) / r.baseline_cost
                  : 0.0f;
        if (r.best_is_downsize) {
          ++downsizes;
        } else {
          ++upsizes;
        }
        debugPrint(state.logger,
                   RSZ,
                   "global_sizing",
                   5,
                   "{} {}: {} -> {} (cost {:.3g} -> {:.3g}, gain {:.2f}%)",
                   r.best_is_downsize ? "DOWN" : "UP  ",
                   network->pathName(r.inst),
                   prev != nullptr ? prev->name() : "?",
                   r.best_cell->name(),
                   r.baseline_cost,
                   r.best_cost,
                   100.0f * rel_gain);
      }
    }
  }

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR sweep: {} instances visited, "
             "{} with an improving candidate, "
             "{} replacements applied ({} upsize, {} downsize)",
             visited,
             evaluated,
             moves,
             upsizes,
             downsizes);

  return {.moves = moves, .upsizes = upsizes, .downsizes = downsizes};
}

SweepEngine::Stats JacobiSnapshotSweep::sweep(LrState& state,
                                              const float timing_weight)
{
  // Phase A: freeze the sweep-start slacks (and, under the depth-budget guard,
  // distribute them into per-vertex budgets), then freeze per-gate state.
  prepareGuardState(state);
  std::vector<LRSubproblem::GateSnapshot> snapshots = buildSnapshots(state);

  // Phase B: Score every snapshot independently. Each worker uses its own
  // ArcDelayCalc copy (arc_delay_calc_ is single-threaded shared state); the
  // copy is cached per worker thread and refreshed if the source changes. With
  // a zero-worker pool, this runs inline on the calling thread.
  const float safety = state.config->budget_safety_factor;
  sta::ArcDelayCalc* const src = state.sta->arcDelayCalc();
  const std::vector<LRSubproblem::GateDecision> decisions
      = thread_pool_->parallelMap(
          snapshots,
          [this, timing_weight, safety, src](
              const LRSubproblem::GateSnapshot& snap) {
            static thread_local sta::ArcDelayCalc* cached_src = nullptr;
            static thread_local std::unique_ptr<sta::ArcDelayCalc> adc;
            if (adc == nullptr || cached_src != src) {
              adc.reset(src->copy());
              cached_src = src;
            }
            return subproblem_->evaluateSnapshot(
                snap, timing_weight, safety, adc.get());
          });

  // Phase C: Apply accepted moves serially.
  std::vector<MovedGate> movers;
  SweepEngine::Stats stats = applyDecisions(
      state, decisions, static_cast<int>(snapshots.size()), movers);

  // Phase D: the post-sweep max-cap re-check. Every snapshot in this engine was
  // frozen before any commit, so no candidate could see the load its neighbours
  // were about to add - this is where that blindness is corrected
  // (CapRecheck.hh).
  const CapRecheckStats recheck = recheckMaxCapAfterSweep(state, movers);
  stats.cap_reverts = recheck.reverted;
  stats.cap_recheck_bound_hit = recheck.bound_hit;
  return stats;
}

float JacobiSnapshotSweep::computeTimingWeightBase(LrState& state)
{
  return timingWeightBase(state, resizer_, *subproblem_);
}

////////////////////////////////////////////////////////////////
// Gauss-Seidel (sequential) engine

GaussSeidelSweep::GaussSeidelSweep(Resizer* resizer)
    : resizer_(resizer), subproblem_(std::make_unique<LRSubproblem>(resizer))
{
}

GaussSeidelSweep::~GaussSeidelSweep() = default;

void GaussSeidelSweep::init(LrState& /* state */)
{
  subproblem_->init();
}

float GaussSeidelSweep::computeTimingWeightBase(LrState& state)
{
  return timingWeightBase(state, resizer_, *subproblem_);
}

std::vector<sta::Instance*> GaussSeidelSweep::buildTraversalOrder(
    LrState& state)
{
  // One entry per leaf instance, keyed by the sweep-start graph state. The real
  // eligibility filter runs later in snapshot(); ineligible instances just get
  // skipped when their snapshot fails, so we key/sort them all here (their
  // position is irrelevant). tiebreak = the stable network object id, so ties
  // (same level / same slack) resolve deterministically.
  sta::Network* network = state.network;
  sta::Graph* graph = state.graph;
  const bool crit = (state.config->traversal
                     == GlobalSizingConfig::Traversal::kCriticalitySorted);

  std::vector<TraversalEntry> entries;
  std::unique_ptr<sta::LeafInstanceIterator> iit(
      network->leafInstanceIterator());
  while (iit->hasNext()) {
    sta::Instance* inst = iit->next();
    // Topological modes: the gate's (min) output-vertex level. Criticality:
    // the gate's worst (min) sweep-start slack. Instances with no output vertex
    // keep the sentinel key and sort last; snapshot() drops them anyway.
    float key = std::numeric_limits<float>::max();
    if (!crit) {
      key = topoTraversalKey(network, graph, inst);
    } else {
      std::unique_ptr<sta::InstancePinIterator> pit(network->pinIterator(inst));
      while (pit->hasNext()) {
        sta::Pin* pin = pit->next();
        if (!network->direction(pin)->isOutput()) {
          continue;
        }
        sta::Vertex* v = graph->pinDrvrVertex(pin);
        if (v == nullptr) {
          continue;
        }
        // A non-finite slack (unconstrained +inf sentinel, or a pathological
        // NaN) sorts as least-critical. Sanitizing NaN here is required: a NaN
        // key makes orderTraversal's comparator violate strict-weak-ordering
        // (std::sort undefined behavior). Level keys are always finite, which
        // is why topoTraversalKey needs no such guard.
        const float slack = sta::delayAsFloat(state.sta->slack(v, state.max));
        key = std::min(
            key,
            std::isfinite(slack) ? slack : std::numeric_limits<float>::max());
      }
    }
    TraversalEntry e;
    e.key = key;
    e.tiebreak = static_cast<uint64_t>(network->id(inst));
    e.inst = inst;
    entries.push_back(e);
  }

  orderTraversal(entries, state.config->traversal);

  std::vector<sta::Instance*> order;
  order.reserve(entries.size());
  for (const TraversalEntry& e : entries) {
    order.push_back(e.inst);
  }
  return order;
}

void GaussSeidelSweep::refreshAfterCommit(LrState& state)
{
  // Sequential commits: refresh so the NEXT gate's JIT snapshot reads fresh
  // upstream state. updateParasitics is already incremental - it re-estimates
  // only the nets this commit invalidated and marks their downstream forward
  // delays invalid, so the next snapshot's loadCap / slew queries recompute
  // them lazily (forward-only, up to the queried vertex level). Under
  // gs_incremental we additionally propagate required times (a full OpenSTA
  // incremental update); under gs_local we do not, leaving required times stale
  // mid-sweep (Flach's local timing update) - which is fine because the only
  // required-time consumer, the depth-budget guard, is frozen at the
  // sweep-start snapshot.
  state.resizer->estimateParasitics()->updateParasitics();
  if (state.config->gs_refresh == GlobalSizingConfig::GsRefresh::kIncremental) {
    state.sta->findRequireds();
  }
}

SweepEngine::Stats GaussSeidelSweep::sweep(LrState& state,
                                           const float timing_weight)
{
  // Guard state from the sweep-start STA snapshot (conservative-but-valid under
  // sequential commits; see computeSlackBudgets). Under gs_incremental the veto
  // re-reads slacks live per gate instead - see LRSubproblem::SnapshotInputs.
  prepareGuardState(state);
  const LRSubproblem::SnapshotInputs in = sweepInputs(state);
  const std::vector<sta::Instance*> order = buildTraversalOrder(state);
  const float safety = state.config->budget_safety_factor;
  const float upsize_hysteresis = state.config->upsize_hysteresis;
  sta::Network* network = state.network;

  int visited = 0;
  int evaluated = 0;
  int moves = 0;
  int upsizes = 0;
  int downsizes = 0;
  std::vector<MovedGate> movers;

  for (sta::Instance* inst : order) {
    // JIT snapshot against the live (post-previous-commit) state. Shares the
    // eligibility filter with Jacobi via snapshot().
    LRSubproblem::GateSnapshot snap;
    if (!subproblem_->snapshot(inst, in, snap)) {
      continue;
    }
    ++visited;
    // Serial main-thread evaluation: the shared ArcDelayCalc is safe here (no
    // parallel workers), so no per-thread copy is needed. Re-fetch each gate in
    // case a prior refresh swapped the calculator pointer.
    sta::ArcDelayCalc* adc = state.sta->arcDelayCalc();
    const LRSubproblem::GateDecision d
        = subproblem_->evaluateSnapshot(snap, timing_weight, safety, adc);
    if (d.best_cell == nullptr) {
      continue;
    }
    ++evaluated;
    if (!acceptGateMove(d, upsize_hysteresis)) {
      continue;
    }
    sta::LibertyCell* prev = network->libertyCell(d.inst);
    if (subproblem_->applyReplacement(d.inst, d.best_cell)) {
      ++moves;
      movers.push_back({.inst = d.inst,
                        .prev_cell = prev,
                        .was_downsize = d.best_is_downsize});
      const float rel_gain
          = d.baseline_cost > 0.0f
                ? (d.baseline_cost - d.best_cost) / d.baseline_cost
                : 0.0f;
      if (d.best_is_downsize) {
        ++downsizes;
      } else {
        ++upsizes;
      }
      debugPrint(state.logger,
                 RSZ,
                 "global_sizing",
                 5,
                 "{} {}: {} -> {} (cost {:.3g} -> {:.3g}, gain {:.2f}%)",
                 d.best_is_downsize ? "DOWN" : "UP  ",
                 network->pathName(d.inst),
                 prev != nullptr ? prev->name() : "?",
                 d.best_cell->name(),
                 d.baseline_cost,
                 d.best_cost,
                 100.0f * rel_gain);
      // Commit fresh upstream state before the next gate is snapshotted.
      refreshAfterCommit(state);
    }
  }

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "GS sweep ({}, {}): {} instances visited, "
             "{} with an improving candidate, "
             "{} replacements applied ({} upsize, {} downsize)",
             toString(state.config->traversal),
             toString(state.config->gs_refresh),
             visited,
             evaluated,
             moves,
             upsizes,
             downsizes);

  // The JIT snapshots above read genuinely live loads and limits, so each gate
  // saw every EARLIER commit of this sweep - but none of them could see a LATER
  // one, and no traversal order makes that possible. The same post-sweep
  // re-check the Jacobi engine needs closes that half (CapRecheck.hh).
  const CapRecheckStats recheck = recheckMaxCapAfterSweep(state, movers);

  return {.moves = moves,
          .upsizes = upsizes,
          .downsizes = downsizes,
          .cap_reverts = recheck.reverted,
          .cap_recheck_bound_hit = recheck.bound_hit};
}

std::unique_ptr<SweepEngine> makeSweepEngine(const GlobalSizingConfig& config,
                                             Resizer* resizer,
                                             utl::ThreadPool* thread_pool)
{
  switch (config.sweep_engine) {
    case GlobalSizingConfig::SweepEngineKind::kJacobiSnapshot:
      return std::make_unique<JacobiSnapshotSweep>(resizer, thread_pool);
    case GlobalSizingConfig::SweepEngineKind::kGaussSeidelTopo:
      return std::make_unique<GaussSeidelSweep>(resizer);
  }
  return std::make_unique<JacobiSnapshotSweep>(resizer, thread_pool);
}

}  // namespace rsz
