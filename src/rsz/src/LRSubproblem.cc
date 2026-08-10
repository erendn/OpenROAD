// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "LRSubproblem.hh"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "lr/CostTerms.hh"
#include "lr/ElectricalModel.hh"
#include "lr/Guards.hh"
#include "odb/db.h"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Liberty.hh"
#include "sta/LibertyClass.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/PortDirection.hh"
#include "sta/Scene.hh"
#include "sta/Sta.hh"
#include "sta/TimingRole.hh"
#include "sta/Transition.hh"

namespace rsz {

namespace {

// Resizer::area(Cell*) is protected. Compute the same value through the public
// dbuToMeters + db_network->staToDb pair so we don't friend-pierce the Resizer
// class. Matches Resizer::area(dbMaster*) exactly.
double cellAreaSI(const Resizer& resizer,
                  sta::dbNetwork* db_network,
                  sta::LibertyCell* cell)
{
  if (cell == nullptr) {
    return 0.0;
  }
  odb::dbMaster* master = db_network->staToDb(db_network->cell(cell));
  if (master == nullptr || !master->isCoreAutoPlaceable()) {
    return 0.0;
  }
  return resizer.dbuToMeters(master->getWidth())
         * resizer.dbuToMeters(master->getHeight());
}

// Finite-difference δd/δslew of the gate arc(s) into `out_port` at the given
// (input slew, load), for the cost_fanout_slew term. Main-thread only (uses the
// shared arc_delay_calc via Resizer::gateDelays). Returns a non-negative slope,
// or 0 on a degenerate lookup. Mirrors the gate-arc sensitivity computed in
// CostTerms.cc's φ pass but returns only δd/δslew (fanout-slew does not need
// the output-slew sensitivity); kept separate so this worker-prep path stays
// decoupled from LrState.
float fanoutArcDelaySens(Resizer* resizer,
                         const sta::LibertyPort* out_port,
                         const float load,
                         const float in_slew,
                         const sta::Scene* scene,
                         const sta::MinMax* max_mm)
{
  const float delta = std::max(std::fabs(in_slew) * 0.05f, 1e-13f);
  sta::Slew in0[sta::RiseFall::index_count];
  sta::Slew in1[sta::RiseFall::index_count];
  for (int i : sta::RiseFall::rangeIndex()) {
    in0[i] = in_slew;
    in1[i] = in_slew + delta;
  }
  sta::ArcDelay d0[sta::RiseFall::index_count];
  sta::ArcDelay d1[sta::RiseFall::index_count];
  sta::Slew os0[sta::RiseFall::index_count];
  sta::Slew os1[sta::RiseFall::index_count];
  resizer->gateDelays(out_port, load, in0, scene, max_mm, d0, os0);
  resizer->gateDelays(out_port, load, in1, scene, max_mm, d1, os1);
  float d0m = -sta::INF;
  float d1m = -sta::INF;
  for (int i : sta::RiseFall::rangeIndex()) {
    d0m = std::max(d0m, sta::delayAsFloat(d0[i]));
    d1m = std::max(d1m, sta::delayAsFloat(d1[i]));
  }
  if (d0m <= -sta::INF / 2) {
    return 0.0f;
  }
  const float s = (d1m - d0m) / delta;
  return std::isfinite(s) ? std::max(0.0f, s) : 0.0f;
}

}  // namespace

LRSubproblem::LRSubproblem(Resizer* resizer) : resizer_(resizer)
{
}

void LRSubproblem::init()
{
  if (initialized_) {
    return;
  }
  logger_ = resizer_->logger();
  dbStaState::init(resizer_->sta());
  db_network_ = resizer_->dbNetwork();
  computeLeakageScale();
  initialized_ = true;
}

void LRSubproblem::computeLeakageScale()
{
  // Build (leakage, area) pairs for instances whose current cell has both.
  std::vector<float> leakages;
  std::vector<float> areas;
  std::unique_ptr<sta::LeafInstanceIterator> iit(
      network_->leafInstanceIterator());
  while (iit->hasNext()) {
    sta::Instance* inst = iit->next();
    sta::LibertyCell* cell = network_->libertyCell(inst);
    if (cell == nullptr) {
      continue;
    }
    const std::optional<float> leak = resizer_->cellLeakage(cell);
    if (!leak.has_value()) {
      continue;
    }
    const double a = cellAreaSI(*resizer_, db_network_, cell);
    if (a <= 0.0) {
      continue;
    }
    leakages.push_back(*leak);
    areas.push_back(static_cast<float>(a));
  }

  if (leakages.empty()) {
    // Degenerate: no instance exposes leakage. leakageOrArea will return
    // raw area, which is order-preserving within this design.
    area_to_leakage_scale_ = 0.0f;
    return;
  }

  const auto mid = leakages.size() / 2;
  std::nth_element(leakages.begin(), leakages.begin() + mid, leakages.end());
  const float l_med = leakages[mid];
  std::nth_element(areas.begin(), areas.begin() + mid, areas.end());
  const float a_med = areas[mid];

  area_to_leakage_scale_ = (a_med > 0.0f) ? (l_med / a_med) : 0.0f;
}

float LRSubproblem::leakageOrArea(sta::LibertyCell* cell) const
{
  const std::optional<float> leak = resizer_->cellLeakage(cell);
  if (leak.has_value()) {
    return *leak;
  }
  const float a = static_cast<float>(cellAreaSI(*resizer_, db_network_, cell));
  return area_to_leakage_scale_ > 0.0f ? area_to_leakage_scale_ * a : a;
}

bool LRSubproblem::isDataArc(const sta::Edge* edge) const
{
  const sta::TimingRole* role = edge->role();
  if (role != nullptr && role->isTimingCheck()) {
    return false;
  }
  if (edge->isDisabledLoop()) {
    return false;
  }
  if (role == sta::TimingRole::latchDtoQ()
      || role == sta::TimingRole::latchEnToQ()) {
    return false;
  }
  return true;
}

float LRSubproblem::portInputCap(sta::LibertyCell* cell,
                                 const char* port_name) const
{
  return rsz::portInputCap(cell, port_name, max_);
}

bool LRSubproblem::applyReplacement(sta::Instance* inst,
                                    sta::LibertyCell* replacement)
{
  if (inst == nullptr || replacement == nullptr) {
    return false;
  }
  return resizer_->replaceCell(inst, replacement, /*journal=*/true);
}

bool LRSubproblem::snapshot(sta::Instance* inst,
                            const SnapshotInputs& in,
                            GateSnapshot& snap)
{
  init();

  if (resizer_->dontTouch(inst)) {
    return false;
  }
  sta::LibertyCell* cur_cell = network_->libertyCell(inst);
  if (cur_cell == nullptr) {
    return false;
  }

  const sta::Scene* scene = sta_->cmdScene();
  const sta::MinMax* max_mm = max_;
  const CostTermFlags cost = in.cost;
  const float* lambda = in.lambda;
  const int lambda_size = in.lambda_size;
  const int phi_size = in.phi_size;
  const int prev_delay_size = in.prev_delay_size;
  const bool veto
      = (in.guard == GlobalSizingConfig::DownsizeGuard::kLocalSlackVeto);

  snap.inst = inst;
  snap.cur_cell = cur_cell;
  snap.scene = scene;
  snap.cost = cost;
  snap.guard = in.guard;
  snap.gamma = in.gamma;
  snap.guard_active = in.guard_active;
  snap.move_set = in.move_set;
  snap.fast_olr_active = in.fast_olr_active;
  snap.output_drc_veto = in.output_drc_veto;
  snap.outputs.clear();
  snap.upstream.clear();
  snap.inputs.clear();
  snap.candidates.clear();

  // The veto's view of a vertex's slack. Live under gs_incremental (its
  // per-commit findRequireds has just revalidated required times, so the query
  // is cheap and sees this gate's real post-commit timing); read from the
  // sweep-start vector otherwise - under gs_local the graph's required times
  // are deliberately stale mid-sweep, and querying them would silently rerun a
  // full timing update per gate. Only called when the veto is the guard.
  auto vertex_slack = [this, &in](sta::Vertex* v) -> float {
    if (in.live_slacks) {
      return sta::delayAsFloat(sta_->slack(v, max_));
    }
    const sta::VertexId vid = graph_->id(v);
    return std::cmp_less(vid, in.vertex_slack_size) ? in.vertex_slack[vid]
                                                    : 0.0f;
  };

  // Min depth-normalized downsize budget over the kept output pins. The policy
  // precomputes a per-vertex budget (computeSlackBudgets) from the live slacks;
  // we just freeze the gate's worst (min) value so workers never touch the STA
  // graph.
  float worst_budget = std::numeric_limits<float>::max();

  std::unique_ptr<sta::InstancePinIterator> pit(network_->pinIterator(inst));
  while (pit->hasNext()) {
    sta::Pin* pin = pit->next();
    const sta::PortDirection* dir = network_->direction(pin);
    if (dir->isOutput()) {
      if (!in.include_clock_network && sta_->isClock(pin, sta_->cmdMode())) {
        return false;
      }
      sta::Vertex* v = graph_->pinDrvrVertex(pin);
      if (v == nullptr) {
        continue;
      }
      const sta::LibertyPort* out_port = network_->libertyPort(pin);
      if (out_port == nullptr) {
        continue;
      }
      float lam_sum = 0.0f;
      float ref_delay = 0.0f;
      sta::VertexInEdgeIterator ieit(v, graph_);
      while (ieit.hasNext()) {
        sta::Edge* e = ieit.next();
        if (!isDataArc(e)) {
          continue;
        }
        // Restrict to gate-internal arcs (from a pin on the same instance).
        const sta::Pin* from_pin = e->from(graph_)->pin();
        if (network_->instance(from_pin) != inst) {
          continue;
        }
        const sta::EdgeId id = graph_->id(e);
        if (std::cmp_greater_equal(id, lambda_size)) {
          continue;
        }
        lam_sum += lambda[id];
        // Delta-delay reference: worst previous-iteration delay over the arcs
        // this pin's aggregate gate delay stands in for.
        if (cost.delta_delay && std::cmp_less(id, prev_delay_size)) {
          ref_delay = std::max(ref_delay, in.prev_delay[id]);
        }
      }
      OutputCtx o;
      o.port = out_port;
      o.load_cap = graph_delay_calc_->loadCap(pin, scene, max_mm);
      o.lambda_sum = lam_sum;
      o.ref_delay = ref_delay;
      // Flach's veto: this output pin IS the gate's sink net. The candidate's
      // own-arc delay change (against the frozen current delay) shifts the
      // slack here one-for-one.
      if (veto) {
        o.slack = vertex_slack(v);
        o.d_cur = sta::delayAsFloat(
            resizer_->gateDelay(out_port, o.load_cap, scene, max_mm));
      }
      // Freeze the Elmore-slew DRC inputs. slew is the STA graph slew at the
      // output pin's load vertex; it is constant across candidates, so we read
      // it once here on the main thread.
      sta::Vertex* load_v = graph_->pinLoadVertex(pin);
      o.slew = (load_v != nullptr)
                   ? sta::delayAsFloat(sta_->slew(load_v,
                                                  sta::RiseFallBoth::riseFall(),
                                                  sta_->scenes(),
                                                  max_mm))
                   : 0.0f;
      o.drive_res = out_port->driveResistance();
      o.slew_factor = outputSlewFactor(o.slew, o.drive_res, o.load_cap);

      // A2 relative veto: freeze the CURRENT cell's own standing on this pin,
      // which is the bar candidateDrcOkSnapshot holds each candidate to. Once
      // per pin per sweep rather than once per candidate, and skipped entirely
      // under the absolute default.
      //
      // DECISION (a) - the slew comparison basis. The candidate's slew is an
      // ESTIMATE (o.slew_factor rescaled by the candidate's drive resistance),
      // while o.slew is the STA graph value. The incumbent's figure is computed
      // through the SAME estimator with the current port, so the two sides of
      // the "worsened?" test are the same kind of number. It costs essentially
      // nothing in fidelity - the calibration is defined so that
      // slew_factor * drive_res * load_cap reproduces o.slew for the current
      // port, up to the round-trip error of one divide and one multiply - and
      // where the calibration is degenerate (no load or no drive resistance)
      // the estimator abstains for both sides at once, so the pin reads clean
      // and the relative rule stays inert exactly where the model has nothing
      // to say.
      //
      // WHAT IT DOES NOT BUY, stated because the shape invites the opposite
      // reading: each side's excess is measured against ITS OWN port's limit,
      // so this compares VIOLATIONS, not transitions. On a library whose
      // equivalence group carries non-monotone per-cell max_transition (sky130
      // does: buf_8 declares 7.65 ns while the STRONGER buf_16 declares 5.01)
      // a weaker candidate can show the smaller excess while driving the
      // physically LARGER transition, and the relative rule admits it where the
      // absolute one did not. Kept rather than strengthened with an unsourced
      // second test: both papers write the rule against an increase in the
      // VIOLATION, per-pin violation is also what the ERC the campaign scores
      // measures, and the downstream cost of the larger transition is priced by
      // the cost model's slew-coupling terms, which both relative-pinned
      // presets enable. Recorded on GlobalSizingConfig::OutputDrcVeto.
      // RULED: kept (user, 2026-08-09; ITERATION_2_PLAN §9 item 14). The
      // value-side conjunct was escalated for a ruling rather than taken
      // unilaterally, because it would change what two paper columns measure;
      // the ruling is no. This is the shipped rule, not a deferred fix.
      if (in.output_drc_veto == GlobalSizingConfig::OutputDrcVeto::kRelative) {
        o.cap_excess_cur = outputMaxCapExcess(out_port, o.load_cap, max_mm);
        o.slew_excess_cur = outputMaxSlewExcess(
            sta_, out_port, o.slew_factor, o.load_cap, scene, max_mm);
      }

      // B2 output-slew coupling context (main thread; guarded by flags). Walk
      // the pin's out data edges (wire arcs to the sink pins):
      //  - phi_sink_sum = Σ φ over those wire arcs == Σ φ over the sink arcs
      //    (φ of a wire arc is the pass-through downstream sum), i.e. Flach
      //    Eq. 13 at this net.
      //  - fanout_slew_sens = Σ over the immediate sink gate arcs of
      //    λ_ik·(δd_ik/δslew), the linearized Livramento fanout term.
      if (cost.fanout_slew || cost.global_phi) {
        float phi_sum = 0.0f;
        float fanout_sens = 0.0f;
        sta::VertexOutEdgeIterator oeit(v, graph_);
        while (oeit.hasNext()) {
          sta::Edge* we = oeit.next();
          if (!isDataArc(we)) {
            continue;
          }
          const sta::EdgeId we_id = graph_->id(we);
          if (cost.global_phi && std::cmp_less(we_id, phi_size)) {
            phi_sum += in.phi[we_id];
          }
          if (!cost.fanout_slew) {
            continue;
          }
          sta::Vertex* sink_v = we->to(graph_);
          const sta::Pin* sink_in_pin = sink_v->pin();
          const sta::Instance* sink_inst = network_->instance(sink_in_pin);
          if (sink_inst == nullptr || sink_inst == inst) {
            continue;
          }
          const float sink_in_slew = sta::delayAsFloat(sta_->slew(
              sink_v, sta::RiseFallBoth::riseFall(), sta_->scenes(), max_mm));
          sta::VertexOutEdgeIterator geit(sink_v, graph_);
          while (geit.hasNext()) {
            sta::Edge* ge = geit.next();
            if (!isDataArc(ge)) {
              continue;
            }
            const sta::Pin* sink_out_pin = ge->to(graph_)->pin();
            if (network_->instance(sink_out_pin) != sink_inst) {
              continue;  // only the sink gate's internal arcs
            }
            const sta::EdgeId ge_id = graph_->id(ge);
            if (std::cmp_greater_equal(ge_id, lambda_size)) {
              continue;
            }
            const float lam_ik = lambda[ge_id];
            if (lam_ik <= 0.0f) {
              continue;
            }
            const sta::LibertyPort* sink_out_port
                = network_->libertyPort(sink_out_pin);
            if (sink_out_port == nullptr) {
              continue;
            }
            const float sink_load
                = graph_delay_calc_->loadCap(sink_out_pin, scene, max_mm);
            fanout_sens += lam_ik
                           * fanoutArcDelaySens(resizer_,
                                                sink_out_port,
                                                sink_load,
                                                sink_in_slew,
                                                scene,
                                                max_mm);
          }
        }
        o.phi_sink_sum = phi_sum;
        o.fanout_slew_sens = fanout_sens;
      }

      const sta::VertexId vid = graph_->id(v);
      const float vbudget = std::cmp_less(vid, in.budget_size)
                                ? in.budget[vid]
                                : std::numeric_limits<float>::max();
      worst_budget = std::min(worst_budget, vbudget);
      snap.outputs.push_back(o);
    } else if (dir->isInput()) {
      const sta::LibertyPort* in_port = network_->libertyPort(pin);

      // (a) Input-side max-cap DRC context for every input pin: freeze each
      // fanin driver's current cap-check so workers can replay
      // Resizer::replacementPreservesMaxCap without touching live STA.
      if (in_port != nullptr) {
        sta::PinSet* drivers = network_->drivers(pin);
        if (drivers != nullptr) {
          InputMaxCapCtx in_ctx;
          in_ctx.in_port = in_port;
          in_ctx.old_cap = portInputCap(cur_cell, in_port->name().c_str());
          for (const sta::Pin* driver_pin : *drivers) {
            float cap = 0.0f;
            float max_cap = 0.0f;
            float cap_slack = 0.0f;
            const sta::RiseFall* tr = nullptr;
            const sta::Scene* corner = nullptr;
            sta_->checkCapacitance(driver_pin,
                                   sta_->scenes(),
                                   max_mm,
                                   cap,
                                   max_cap,
                                   cap_slack,
                                   tr,
                                   corner);
            DriverCapCheck dc;
            dc.cap = cap;
            dc.max_cap = max_cap;
            dc.cap_slack = cap_slack;
            dc.corner_ok = (max_cap > 0.0f && corner != nullptr);
            in_ctx.drivers.push_back(dc);
          }
          snap.inputs.push_back(std::move(in_ctx));
        }
      }

      // (b) Upstream-Cin context: only input pins with real upstream pressure.
      sta::Vertex* in_v = graph_->pinLoadVertex(pin);
      if (in_v == nullptr) {
        continue;
      }
      // Locate the driver pin via the wire arc(s) feeding in_v. There's
      // typically exactly one; take the first valid one.
      sta::Pin* drv_pin = nullptr;
      sta::VertexInEdgeIterator wireIt(in_v, graph_);
      while (wireIt.hasNext()) {
        sta::Edge* w = wireIt.next();
        if (w->isDisabledLoop()) {
          continue;
        }
        sta::Pin* candidate_drv = w->from(graph_)->pin();
        if (candidate_drv != nullptr && candidate_drv != pin) {
          drv_pin = candidate_drv;
          break;
        }
      }
      if (drv_pin == nullptr) {
        continue;  // floating / no driver
      }
      sta::Instance* upstream_inst = network_->instance(drv_pin);
      if (upstream_inst == nullptr || upstream_inst == inst) {
        continue;
      }
      sta::LibertyCell* upstream_cell = network_->libertyCell(upstream_inst);
      if (upstream_cell == nullptr) {
        // PI / hierarchical / black box - no Liberty model to evaluate.
        continue;
      }
      sta::LibertyPort* drv_port = network_->libertyPort(drv_pin);
      if (drv_port == nullptr) {
        continue;
      }
      sta::Vertex* drv_v = graph_->pinDrvrVertex(drv_pin);
      if (drv_v == nullptr) {
        continue;
      }
      // Sum λ over U's gate-internal data arcs terminating at drv_pin.
      float lam_U = 0.0f;
      float ref_delay_U = 0.0f;
      sta::VertexInEdgeIterator drvIt(drv_v, graph_);
      while (drvIt.hasNext()) {
        sta::Edge* e = drvIt.next();
        if (!isDataArc(e)) {
          continue;
        }
        const sta::Pin* from_pin = e->from(graph_)->pin();
        if (network_->instance(from_pin) != upstream_inst) {
          continue;
        }
        const sta::EdgeId id = graph_->id(e);
        if (std::cmp_greater_equal(id, lambda_size)) {
          continue;
        }
        lam_U += lambda[id];
        if (cost.delta_delay && std::cmp_less(id, prev_delay_size)) {
          ref_delay_U = std::max(ref_delay_U, in.prev_delay[id]);
        }
      }
      // Skip pins with no real upstream pressure - saves the per-candidate
      // gateDelay call for arcs whose λ is essentially at floor anyway. Under
      // the veto the entry is kept regardless: this pin's driver net counts
      // toward the gate's local negative slack whatever its λ is.
      if (lam_U <= 0.0f && !veto) {
        continue;
      }
      if (in_port == nullptr) {
        continue;
      }
      UpstreamCtx u;
      u.orig_in_port = in_port;
      u.drv_port = drv_port;
      u.load_U_cur = graph_delay_calc_->loadCap(drv_pin, scene, max_mm);
      u.c_in_cur = portInputCap(cur_cell, in_port->name().c_str());
      u.lambda_U_drv = lam_U;
      u.ref_delay = ref_delay_U;
      // Flach's veto: this is one of the gate's driver nets. A candidate with a
      // different input cap perturbs U's load and so shifts the slack here.
      // Count each driver net once (Alg. 4 line 1 sums over nets) even when two
      // input pins share it - but keep both entries, since each pin perturbs
      // the driver's load in its own right.
      if (veto) {
        u.slack = vertex_slack(drv_v);
        u.d_drv_cur = sta::delayAsFloat(
            resizer_->gateDelay(drv_port, u.load_U_cur, scene, max_mm));
        u.veto_counts = std::ranges::none_of(
            snap.upstream, [drv_pin, this](const UpstreamCtx& prev) {
              return prev.drv_pin == drv_pin;
            });
        u.drv_pin = drv_pin;
      }
      snap.upstream.push_back(u);
    }
  }

  if (snap.outputs.empty()) {
    return false;
  }
  snap.budget = worst_budget;

  // Flach's originalSlack (Alg. 4 line 1): the local negative slack of the gate
  // as it stands - over its driver nets and its sink net, i.e. the same sum the
  // veto recomputes per candidate, at zero delay change.
  if (veto) {
    float orig = 0.0f;
    for (const OutputCtx& o : snap.outputs) {
      orig += negativeSlackAfter(o.slack, 0.0f);
    }
    for (const UpstreamCtx& u : snap.upstream) {
      if (u.veto_counts) {
        orig += negativeSlackAfter(u.slack, 0.0f);
      }
    }
    snap.local_slack_orig = orig;
  }

  // Precompute leakage-equivalent cost for the current cell and every
  // candidate now, on the main thread - leakageOrArea/getSwappableCells mutate
  // lazy caches and must not be touched from workers.
  snap.cur_leakage = leakageOrArea(cur_cell);
  sta::LibertyCellSeq candidates = resizer_->getSwappableCells(cur_cell);
  snap.candidates.reserve(candidates.size());
  for (sta::LibertyCell* cand : candidates) {
    if (cand == cur_cell) {
      continue;
    }
    Candidate c;
    c.cell = cand;
    c.leakage = leakageOrArea(cand);
    snap.candidates.push_back(c);
  }
  buildCandidateGrid(snap);

  return true;
}

int LRSubproblem::vthFlavorKey(sta::LibertyCell* cell)
{
  const auto cached = vt_flavor_cache_.find(cell);
  if (cached != vt_flavor_cache_.end()) {
    return cached->second;
  }
  // Resizer::cellVTType hashes the master's IMPLANT obstruction layers, which
  // is OpenROAD's own notion of a Vth flavor - and it is an IDENTITY, not an
  // order (a master with no implant obstruction, i.e. most of nangate45, gets
  // the single key 0). The grid does the ordering, by the flavors' mean rank
  // key. Main-thread only: cellVTType mutates its own lazy map, as does this
  // cache.
  int key = 0;
  if (odb::dbMaster* master = db_network_->staToDb(cell)) {
    key = resizer_->cellVTType(master).vt_index;
  }
  vt_flavor_cache_.emplace(cell, key);
  return key;
}

const LRSubproblem::CachedGrid& LRSubproblem::cachedGrid(
    sta::LibertyCell* cur_cell,
    const std::vector<Candidate>& candidates,
    const float cur_leakage)
{
  const auto cached = grid_cache_.find(cur_cell);
  if (cached != grid_cache_.end()) {
    return cached->second;
  }
  // The incumbent goes in FIRST, so its coord comes back at index 0 (the
  // convention InitPass's group vector uses for the same reason). It is not a
  // candidate, but it is a member of the group, and the grid's columns have to
  // contain it or "one size up from here" has no referent.
  std::vector<GridMember> members;
  members.reserve(candidates.size() + 1);
  members.push_back({.vth_key = vthFlavorKey(cur_cell),
                     .rank_key = cur_leakage,
                     .name = cur_cell->name()});
  for (const Candidate& cand : candidates) {
    members.push_back({.vth_key = vthFlavorKey(cand.cell),
                       .rank_key = cand.leakage,
                       .name = cand.cell->name()});
  }
  CachedGrid grid;
  grid.coords = buildSizeVthGrid(members);
  grid.layout = buildGridLayout(
      std::vector<GridCoord>(grid.coords.begin() + 1, grid.coords.end()),
      grid.coords[0]);
  return grid_cache_.emplace(cur_cell, std::move(grid)).first->second;
}

void LRSubproblem::buildCandidateGrid(GateSnapshot& snap)
{
  using MoveSet = GlobalSizingConfig::MoveSet;
  // Pay for the grid only on a sweep that actually reads a coord. kFullLibrary
  // never does - that is the rsz_baseline path - and neither does
  // sharma_fast_olr before its switch-over iteration, where the sweep runs the
  // exhaustive scan (Sharma's own first four iterations, plus every
  // estimation-loop dry sweep).
  if (snap.move_set == MoveSet::kFullLibrary
      || (snap.move_set == MoveSet::kSharmaFastOlr && !snap.fast_olr_active)) {
    return;
  }
  // Memoized per incumbent cell: the decomposition is a pure function of the
  // group's members, and getSwappableCells returns the same sequence for the
  // same source cell, so candidates[i] is always group member i+1.
  const CachedGrid& grid
      = cachedGrid(snap.cur_cell, snap.candidates, snap.cur_leakage);
  snap.grid = &grid.layout;
  snap.cur_grid = grid.coords[0];
  for (size_t i = 0; i < snap.candidates.size(); ++i) {
    snap.candidates[i].grid = grid.coords[i + 1];
  }
}

float LRSubproblem::evaluateCellCost(const GateSnapshot& snap,
                                     sta::LibertyCell* cell,
                                     const float cell_leakage,
                                     const float timing_weight,
                                     sta::ArcDelayCalc* arc_delay_calc) const
{
  float cost = cell_leakage;
  const sta::Scene* scene = snap.scene;
  const CostTermFlags& f = snap.cost;
  const bool slew_terms = f.fanout_slew || f.global_phi;
  // Output-cone term (always on): arcs that terminate at this instance's output
  // pins. Plus the flagged output-slew coupling terms (fanout-slew, global-φ),
  // which price the candidate's output-slew change regardless of lambda_sum.
  for (const OutputCtx& o : snap.outputs) {
    if (o.port == nullptr) {
      continue;
    }
    const bool need_own = (o.lambda_sum != 0.0f);  // timing pressure on the pin
    const bool need_slew
        = slew_terms && (o.fanout_slew_sens != 0.0f || o.phi_sink_sum != 0.0f);
    if (!need_own && !need_slew) {
      continue;  // matches the baseline skip when no term applies here
    }
    sta::LibertyPort* cand_port = cell->findLibertyPort(o.port->name());
    if (cand_port == nullptr) {
      // Candidate cell missing this output port - reject via huge cost.
      return std::numeric_limits<float>::infinity();
    }
    if (need_own) {
      const float d = sta::delayAsFloat(resizer_->gateDelay(
          cand_port, o.load_cap, scene, max_, arc_delay_calc));
      const float d_priced
          = f.delta_delay ? deltaDelayReferenced(d, o.ref_delay) : d;
      // C3 item 5: the shipped port-worst approximation (Σλ)·d_worst — one
      // gateDelay lookup for the pin. The paper-faithful cost is the per-arc
      // sum Σ_i λ_i·d_i (perArcTimingCost), which prices each gate-internal arc
      // against its own delay but needs a per-arc candidate delay lookup
      // instead of this one; kept as the default per the measured C3 runtime
      // decision (see the completion notes). This overprices the non-worst
      // sibling arcs of a multi-input gate (d_worst >= every d_i).
      cost += timing_weight * portWorstTimingCost(o.lambda_sum, d_priced);
    }
    if (need_slew) {
      const float slew_delta = candidateSlewDelta(
          o.slew, o.drive_res, cand_port->driveResistance());
      if (f.fanout_slew) {
        cost += timing_weight
                * slewSensitivityCost(o.fanout_slew_sens, slew_delta);
      }
      if (f.global_phi) {
        cost += timing_weight * slewSensitivityCost(o.phi_sink_sum, slew_delta);
      }
    }
  }
  // Upstream-Cin load term (cost_upstream_load, on by default): arcs inside
  // each upstream driver U that terminate at the driver pin feeding one of
  // inst's input pins. Their delay depends on the load U drives, which includes
  // inst's input capacitance on that pin. Substituting the candidate's input
  // cap perturbs the upstream's load and shifts its delay.
  if (f.upstream_load) {
    for (const UpstreamCtx& u : snap.upstream) {
      if (u.lambda_U_drv == 0.0f || u.drv_port == nullptr
          || u.orig_in_port == nullptr) {
        continue;
      }
      const float c_in_cand
          = portInputCap(cell, u.orig_in_port->name().c_str());
      if (c_in_cand == 0.0f) {
        // Candidate missing this input port - incompatible.
        return std::numeric_limits<float>::infinity();
      }
      // Numerical safety: extreme C_in mismatches can push the perturbed load
      // slightly negative. Clamp at zero rather than rejecting; the gateDelay
      // LUT is well-defined at zero load.
      const float load_pert
          = std::max(u.load_U_cur - u.c_in_cur + c_in_cand, 0.0f);
      const float d_U = sta::delayAsFloat(resizer_->gateDelay(
          u.drv_port, load_pert, scene, max_, arc_delay_calc));
      const float d_U_priced
          = f.delta_delay ? deltaDelayReferenced(d_U, u.ref_delay) : d_U;
      // C3 item 5: port-worst approximation, same tradeoff as the own-gate term
      // above (per-arc Σ λ_i·d_i is the faithful cost; kept port-worst per the
      // measured runtime decision).
      cost += timing_weight * portWorstTimingCost(u.lambda_U_drv, d_U_priced);
    }
  }
  return cost;
}

bool LRSubproblem::candidateDrcOkSnapshot(const GateSnapshot& snap,
                                          sta::LibertyCell* replacement) const
{
  // Input-side: reject if a fanin net's max-cap would be violated (or made
  // worse) by the new cell's larger input pin cap. Mirrors
  // Resizer::replacementPreservesMaxCap / checkMaxCapOK against the frozen
  // per-driver cap checks captured in snapshot().
  for (const InputMaxCapCtx& in : snap.inputs) {
    if (in.in_port == nullptr) {
      continue;
    }
    const float new_cap = portInputCap(replacement, in.in_port->name().c_str());
    const float cap_delta = new_cap - in.old_cap;
    if (cap_delta <= 0.0f) {
      continue;
    }
    for (const DriverCapCheck& dc : in.drivers) {
      if (!dc.corner_ok) {
        continue;
      }
      const float ncap = dc.cap + cap_delta;
      if (dc.cap_slack < 0.0f) {
        if (ncap > dc.cap) {
          return false;
        }
      } else if (ncap > dc.max_cap) {
        return false;
      }
    }
  }

  // Output-side: per-output-pin check against the new cell's cap/slew limits.
  // Under the absolute default the bar is zero excess, which is the pre-A2
  // `checkOutputMax{Cap,Slew}` test verbatim; under relative it is the current
  // cell's own excess, frozen per pin by snapshot().
  const bool relative
      = snap.output_drc_veto == GlobalSizingConfig::OutputDrcVeto::kRelative;
  for (const OutputCtx& o : snap.outputs) {
    if (o.port == nullptr) {
      continue;
    }
    sta::LibertyPort* cand_port = replacement->findLibertyPort(o.port->name());
    if (cand_port == nullptr) {
      return false;  // candidate missing this output port - reject
    }

    if (!outputLimitAdmits(outputMaxCapExcess(cand_port, o.load_cap, max_),
                           o.cap_excess_cur,
                           relative)) {
      return false;
    }

    if (!outputLimitAdmits(
            outputMaxSlewExcess(
                sta_, cand_port, o.slew_factor, o.load_cap, snap.scene, max_),
            o.slew_excess_cur,
            relative)) {
      return false;
    }
  }

  return true;
}

bool LRSubproblem::downsizeFitsSlackBudget(
    const GateSnapshot& snap,
    sta::LibertyCell* replacement,
    const float safety,
    sta::ArcDelayCalc* arc_delay_calc) const
{
  // snap.budget is the depth-normalized, distributed slack budget.
  const float budget = safety * snap.budget;
  if (budget <= 0.0f) {
    return false;
  }
  const sta::Scene* scene = snap.scene;
  for (const OutputCtx& o : snap.outputs) {
    if (o.port == nullptr) {
      continue;
    }
    sta::LibertyPort* cand_port = replacement->findLibertyPort(o.port->name());
    if (cand_port == nullptr) {
      return false;  // candidate missing this output port - reject
    }
    // Δd at the frozen load: extra gate delay the downsize adds on this pin.
    // Increasing the gate delay by Δd reduces the slack on every path through
    // the pin by Δd, so Δd must fit the budget.
    const float d_cur = sta::delayAsFloat(
        resizer_->gateDelay(o.port, o.load_cap, scene, max_, arc_delay_calc));
    const float d_cand = sta::delayAsFloat(resizer_->gateDelay(
        cand_port, o.load_cap, scene, max_, arc_delay_calc));
    if (d_cand - d_cur > budget) {
      return false;
    }
  }
  return true;
}

bool LRSubproblem::candidatePassesLocalSlackVeto(
    const GateSnapshot& snap,
    sta::LibertyCell* replacement,
    sta::ArcDelayCalc* arc_delay_calc) const
{
  const sta::Scene* scene = snap.scene;
  float cand_local = 0.0f;

  // Sink net: the candidate's own-arc delay change shifts the slack at each
  // output pin one-for-one. o.d_cur is the frozen current delay (the reference
  // is candidate-invariant, so only the candidate's delay is computed here).
  for (const OutputCtx& o : snap.outputs) {
    if (o.port == nullptr) {
      continue;
    }
    sta::LibertyPort* cand_port = replacement->findLibertyPort(o.port->name());
    if (cand_port == nullptr) {
      return false;  // candidate missing this output port - reject
    }
    const float d_cand = sta::delayAsFloat(resizer_->gateDelay(
        cand_port, o.load_cap, scene, max_, arc_delay_calc));
    cand_local += negativeSlackAfter(o.slack, d_cand - o.d_cur);
  }

  // Driver nets: the candidate's input capacitance perturbs each upstream
  // driver's load, shifting that driver's arc delay and so the slack on the
  // net it drives. This is the term that makes the veto bite on UPSIZES - a
  // bigger cell loads its fanin drivers more. Flach sums over distinct driver
  // NETS, so when two input pins share one (both tied to the same signal) only
  // the first entry counts: its slack is added once, and the perturbation is
  // that one pin's cap change rather than the sum of both (an under-estimate on
  // a rare topology, and the same per-pin approximation the upstream-load cost
  // term already makes).
  for (const UpstreamCtx& u : snap.upstream) {
    if (u.drv_port == nullptr || u.orig_in_port == nullptr || !u.veto_counts) {
      continue;
    }
    const float c_in_cand
        = portInputCap(replacement, u.orig_in_port->name().c_str());
    if (c_in_cand == 0.0f) {
      return false;  // candidate missing this input port - incompatible
    }
    const float load_pert
        = std::max(u.load_U_cur - u.c_in_cur + c_in_cand, 0.0f);
    const float d_U_cand = sta::delayAsFloat(resizer_->gateDelay(
        u.drv_port, load_pert, scene, max_, arc_delay_calc));
    cand_local += negativeSlackAfter(u.slack, d_U_cand - u.d_drv_cur);
  }

  return localSlackVetoOkGated(
      snap.guard_active, cand_local, snap.local_slack_orig, snap.gamma);
}

LRSubproblem::GateDecision LRSubproblem::evaluateSnapshot(
    const GateSnapshot& snap,
    const float timing_weight,
    const float budget_safety,
    sta::ArcDelayCalc* arc_delay_calc) const
{
  using DownsizeGuard = GlobalSizingConfig::DownsizeGuard;
  using MoveSet = GlobalSizingConfig::MoveSet;
  GateDecision result;
  result.inst = snap.inst;

  // Baseline cost with the current cell.
  result.baseline_cost = evaluateCellCost(
      snap, snap.cur_cell, snap.cur_leakage, timing_weight, arc_delay_calc);
  result.best_cost = result.baseline_cost;
  float best_leak = snap.cur_leakage;

  // The F3 guard. Split out of the loop because Fast-OLR applies it at a
  // different point than the exhaustive scan does: Sharma Fig. 9 checks cap and
  // slew inside the descent (line 10) but the local-slack guard only on the
  // winner (line 20-21), which is the same split the F3 axis already draws.
  auto passesGuard = [&](const Candidate& cand) {
    if (snap.guard == DownsizeGuard::kDepthBudget) {
      // A candidate with lower leakage than the current cell is a downsize;
      // only take it if its added delay fits the gate's distributed slack
      // budget. Upsizes are unconstrained - they only improve setup.
      return !(cand.leakage < snap.cur_leakage
               && !downsizeFitsSlackBudget(
                   snap, cand.cell, budget_safety, arc_delay_calc));
    }
    if (snap.guard == DownsizeGuard::kLocalSlackVeto) {
      // Flach's acceptance test - every candidate, not just downsizes.
      return candidatePassesLocalSlackVeto(snap, cand.cell, arc_delay_calc);
    }
    return true;
  };
  auto candidateCost = [&](const Candidate& cand) {
    return evaluateCellCost(
        snap, cand.cell, cand.leakage, timing_weight, arc_delay_calc);
  };
  // The argmin, run over whichever candidate the active move set offers.
  auto consider = [&](const Candidate& cand, const float cost) {
    if (cost < result.best_cost) {
      result.best_cost = cost;
      result.best_cell = cand.cell;
      best_leak = cand.leakage;
    }
  };

  const bool fast_olr = snap.move_set == MoveSet::kSharmaFastOlr
                        && snap.fast_olr_active && snap.grid != nullptr;
  if (fast_olr) {
    // Sharma Fig. 9. The descent asks only about DRC validity (line 10 - "a
    // cell is invalid if it causes cap or slew violations"); the F3 guard is
    // line 20-21's "such that local slack does not worsen" and runs on the
    // winners below.
    //
    // THE REACHABLE SET IS DELIBERATELY SMALLER THAN THE EXHAUSTIVE SCAN'S, and
    // that is the mechanism, not an oversight. A gate whose <=6 winners all
    // fail the guard makes no move this sweep, where full_library would have
    // fallen through to some further candidate. Fig. 9 has exactly that shape -
    // line 20 is an argmin over the line-19 SET subject to the guard, with no
    // fallback to the full library - and it is what makes the option's late
    // iterations incremental (its Fig. 11 stability claim). The X9 A/B measures
    // precisely this difference; do not "fix" it by widening the fallback.
    const std::vector<FastOlrWinner> winners = fastOlrCandidates(
        *snap.grid,
        snap.cur_grid,
        result.baseline_cost,
        [&](const int i) {
          return candidateDrcOkSnapshot(snap, snap.candidates[i].cell);
        },
        [&](const int i) { return candidateCost(snap.candidates[i]); });
    for (const FastOlrWinner& winner : winners) {
      const Candidate& cand = snap.candidates[winner.candidate];
      if (passesGuard(cand)) {
        // The descent already paid for this cost; re-running the cost model on
        // a cell it just evaluated would spend back the evaluations the option
        // exists to save.
        consider(cand, winner.cost);
      }
    }
  } else {
    for (const Candidate& cand : snap.candidates) {
      // Mangiras §4.3: only the +-1 width-rank band, any Vth flavor. Applied
      // before anything is evaluated - it is a restriction on the move set, not
      // a rejection of a considered move.
      if (snap.move_set == MoveSet::kMangirasSizeStep && snap.grid != nullptr
          && !withinSizeStep(*snap.grid, snap.cur_grid, cand.grid)) {
        continue;
      }
      // Hard DRC filter (A2, always on): reject any candidate that would
      // introduce a max-cap or max-slew violation.
      if (!candidateDrcOkSnapshot(snap, cand.cell)) {
        continue;
      }
      if (!passesGuard(cand)) {
        continue;
      }
      consider(cand, candidateCost(cand));
    }
  }

  if (result.best_cell != nullptr) {
    result.best_is_downsize = best_leak < snap.cur_leakage;
  }
  return result;
}

}  // namespace rsz
