// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "RelocateGenerator.hh"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <vector>

#include "MoveCommitter.hh"
#include "MoveGenerator.hh"
#include "OptimizerTypes.hh"
#include "RelocateCandidate.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "odb/db.h"
#include "rsz/Resizer.hh"
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Path.hh"
#include "utl/Logger.h"
#include "utl/env.h"

namespace rsz {

using utl::RSZ;

namespace {

inline int manhattan(const odb::Point& a, const odb::Point& b)
{
  return std::abs(a.getX() - b.getX()) + std::abs(a.getY() - b.getY());
}

// First-order increase in driver->sink wire (Elmore) delay when the driver
// moves so the branch length goes from len_old to len_new. Distributed wire:
// added series resistance R_w*dlen drives the sink pin cap plus half the new
// wire cap. Returns 0 when the driver moves toward the sink (no harm).
inline double branchDelayIncrease(const Resizer& resizer,
                                  double wire_res,
                                  double wire_cap,
                                  double pin_cap,
                                  int len_old_dbu,
                                  int len_new_dbu)
{
  const double dlen_m
      = resizer.dbuToMeters(len_new_dbu) - resizer.dbuToMeters(len_old_dbu);
  if (dlen_m <= 0.0) {
    return 0.0;
  }
  const double len_new_m = resizer.dbuToMeters(len_new_dbu);
  return wire_res * dlen_m * (pin_cap + 0.5 * wire_cap * len_new_m);
}

}  // namespace

RelocateGenerator::RelocateGenerator(const GeneratorContext& context)
    : MoveGenerator(context)
{
  // Slack band above setup_slack_margin for marginal-sink centroid pull.
  // 0 (default) reproduces the negative-slack-only centroid; the back-off
  // guard provides correctness regardless of this value.
  slack_band_ = utl::readEnvarDouble("RSZ_RELOCATE_SLACK_BAND", 0.0);
}

bool RelocateGenerator::isApplicable(const Target& target) const
{
  if (!MoveGenerator::isApplicable(target)) {
    return false;
  }

  // No rows means the design isn't placed; we can't legalize the move.
  if (resizer_.block()->getRows().empty()) {
    return false;
  }

  sta::Instance* drvr_inst = target.inst(resizer_);
  if (drvr_inst == nullptr) {
    return false;
  }

  if (resizer_.dontTouch(drvr_inst)) {
    return false;
  }

  if (!resizer_.isLogicStdCell(drvr_inst)) {
    return false;
  }

  // Sequential cells affect setup AND hold at the receiving stage; skip them
  // to avoid surprise hold violations after relocation.
  sta::Vertex* drvr_vertex = target.vertex(resizer_);
  if (drvr_vertex != nullptr && resizer_.isRegister(drvr_vertex)) {
    return false;
  }

  if (committer_.hasMoves(MoveType::kRelocate, drvr_inst)) {
    return false;
  }

  return true;
}

std::vector<std::unique_ptr<MoveCandidate>> RelocateGenerator::generate(
    const Target& target)
{
  std::vector<std::unique_ptr<MoveCandidate>> candidates;
  if (!isApplicable(target)) {
    return candidates;
  }

  sta::Pin* drvr_pin = target.resolvedPin(resizer_);
  sta::Instance* drvr_inst = target.inst(resizer_);
  sta::Vertex* drvr_vertex = target.vertex(resizer_);
  const sta::Path* drvr_path = target.driverPath(resizer_);
  if (drvr_pin == nullptr || drvr_inst == nullptr || drvr_vertex == nullptr
      || drvr_path == nullptr) {
    return candidates;
  }

  const odb::Point orig_loc = resizer_.dbNetwork()->location(drvr_pin);
  const sta::RiseFall* rf = drvr_path->transition(resizer_.sta());

  odb::Point requested_loc;
  if (!computeTargetLocation(
          target, drvr_vertex, rf, orig_loc, requested_loc)) {
    return candidates;
  }

  candidates.push_back(std::make_unique<RelocateCandidate>(
      resizer_,
      target,
      drvr_inst,
      drvr_pin,
      orig_loc,
      requested_loc,
      static_cast<float>(run_config_.setup_slack_margin)));
  return candidates;
}

bool RelocateGenerator::computeTargetLocation(const Target& gen_target,
                                              sta::Vertex* drvr_vertex,
                                              const sta::RiseFall* rf,
                                              const odb::Point& drvr_loc,
                                              odb::Point& out_target) const
{
  sta::dbSta* sta = resizer_.sta();
  sta::Network* network = resizer_.network();
  sta::dbNetwork* db_network = resizer_.dbNetwork();
  sta::Graph* graph = resizer_.graph();
  const sta::MinMax* max_mode = resizer_.maxAnalysisMode();
  const float margin = static_cast<float>(run_config_.setup_slack_margin);
  // Sinks at or above s_ref do not pull on the centroid; sinks below it pull
  // with strength proportional to how far below they are. With slack_band_=0
  // and margin=0 this reduces to the negative-slack-only centroid.
  const double s_ref = margin + slack_band_;

  double weighted_x = 0.0;
  double weighted_y = 0.0;
  double total_weight = 0.0;
  double weighted_pin_cap = 0.0;
  // Sinks currently meeting the margin; the back-off guard keeps them safe.
  std::vector<SinkInfo> safe_sinks;

  sta::VertexOutEdgeIterator edge_iter(drvr_vertex, graph);
  while (edge_iter.hasNext()) {
    sta::Edge* edge = edge_iter.next();
    if (!edge->isWire()) {
      continue;
    }
    sta::Vertex* fanout_vertex = edge->to(graph);
    // Worst slack over both transitions: a sink positive on the path rf but
    // negative on the other edge must still be treated as critical/at-risk.
    const sta::Slack slack = sta->slack(fanout_vertex, max_mode);
    const sta::Pin* load_pin = fanout_vertex->pin();
    const odb::Point load_loc = db_network->location(load_pin);
    sta::LibertyPort* load_port = network->libertyPort(load_pin);
    const double pin_cap
        = load_port != nullptr ? load_port->capacitance(rf, max_mode) : 0.0;

    const double weight = std::max(0.0, s_ref - slack);
    if (weight > 0.0) {
      weighted_x += weight * load_loc.getX();
      weighted_y += weight * load_loc.getY();
      total_weight += weight;
      weighted_pin_cap += weight * pin_cap;
    }
    if (slack >= margin) {
      safe_sinks.push_back({load_loc, pin_cap, slack});
    }
  }

  if (total_weight == 0.0) {
    debugPrint(resizer_.logger(),
               RSZ,
               "relocate_move",
               2,
               "REJECT RelocateMove: no critical loads");
    return false;
  }

  const odb::Point centroid(static_cast<int>(weighted_x / total_weight),
                            static_cast<int>(weighted_y / total_weight));
  const double C_pin_Sw = weighted_pin_cap / total_weight;

  const int centroid_dist = manhattan(centroid, drvr_loc);
  if (centroid_dist < kMinMoveThresholdDbu) {
    debugPrint(resizer_.logger(),
               RSZ,
               "relocate_move",
               2,
               "REJECT RelocateMove: centroid HPWL {} < min threshold {}",
               centroid_dist,
               kMinMoveThresholdDbu);
    return false;
  }

  const sta::Scene* scene = sta->cmdScene();
  const double R_w = resizer_.estimateParasitics()->wireSignalResistance(scene);
  const double C_w
      = resizer_.estimateParasitics()->wireSignalCapacitance(scene);

  // Elmore-optimal driver position on the previous-driver -> centroid line,
  // when all upstream inputs and a wire RC model are available; otherwise the
  // requested location is the plain centroid.
  auto elmore_target = [&]() -> std::optional<odb::Point> {
    if (R_w <= 0.0 || C_w <= 0.0) {
      return std::nullopt;
    }
    const sta::Path* in_path = gen_target.inputPath(resizer_);
    const sta::Path* prev_drvr_path = gen_target.prevDriverPath(resizer_);
    if (in_path == nullptr || prev_drvr_path == nullptr) {
      return std::nullopt;
    }
    const sta::Pin* prev_drvr_pin = prev_drvr_path->pin(sta);
    const sta::Pin* drvr_input_pin = in_path->pin(sta);
    const sta::Pin* drvr_pin_local = gen_target.driver_pin;
    if (prev_drvr_pin == nullptr || drvr_input_pin == nullptr
        || drvr_pin_local == nullptr) {
      return std::nullopt;
    }
    sta::LibertyPort* prev_drvr_port = network->libertyPort(prev_drvr_pin);
    sta::LibertyPort* drvr_input_port = network->libertyPort(drvr_input_pin);
    sta::LibertyPort* drvr_out_port = network->libertyPort(drvr_pin_local);
    if (prev_drvr_port == nullptr || drvr_input_port == nullptr
        || drvr_out_port == nullptr) {
      return std::nullopt;
    }

    const double R_PD = prev_drvr_port->driveResistance();
    const double R_D = drvr_out_port->driveResistance();
    const double C_in_D = drvr_input_port->capacitance(rf, max_mode);

    const odb::Point pd_loc = db_network->location(prev_drvr_pin);
    const int L_dbu = manhattan(centroid, pd_loc);
    if (L_dbu == 0) {
      return std::nullopt;
    }
    const double L_m = resizer_.dbuToMeters(L_dbu);

    // r* = L/2 + (R_D - R_PD)/(2*R_w) + (C_pin_Sw - C_in_D)/(2*C_w)
    const double r_star_m = 0.5 * L_m + (R_D - R_PD) / (2.0 * R_w)
                            + (C_pin_Sw - C_in_D) / (2.0 * C_w);
    const double f_star = std::clamp(r_star_m / L_m, 0.0, 1.0);

    debugPrint(resizer_.logger(),
               RSZ,
               "relocate_move",
               3,
               "RelocateMove target: f*={:.3f} L={} r*_m={:.3e} R_PD={:.3e} "
               "R_D={:.3e} C_in_D={:.3e} C_pin_Sw={:.3e}",
               f_star,
               L_dbu,
               r_star_m,
               R_PD,
               R_D,
               C_in_D,
               C_pin_Sw);

    return odb::Point(
        pd_loc.getX()
            + static_cast<int>(f_star * (centroid.getX() - pd_loc.getX())),
        pd_loc.getY()
            + static_cast<int>(f_star * (centroid.getY() - pd_loc.getY())));
  }();

  const odb::Point requested = elmore_target.value_or(centroid);

  const int target_dist = manhattan(requested, drvr_loc);
  if (target_dist < kMinMoveThresholdDbu) {
    debugPrint(resizer_.logger(),
               RSZ,
               "relocate_move",
               2,
               "REJECT RelocateMove: target HPWL {} < min threshold {}",
               target_dist,
               kMinMoveThresholdDbu);
    return false;
  }

  // Back off the displacement so no currently-safe sink is pushed below the
  // margin by the added driver->sink wire delay.
  if (!backOffForSafeSinks(
          drvr_loc, requested, safe_sinks, margin, R_w, C_w, out_target)) {
    debugPrint(resizer_.logger(),
               RSZ,
               "relocate_move",
               2,
               "REJECT RelocateMove: safe-sink back-off shrank move below "
               "min threshold {}",
               kMinMoveThresholdDbu);
    return false;
  }

  if (out_target != requested) {
    debugPrint(resizer_.logger(),
               RSZ,
               "relocate_move",
               3,
               "RelocateMove backed off ({},{}) -> ({},{}) to protect "
               "{} safe sink(s)",
               requested.getX(),
               requested.getY(),
               out_target.getX(),
               out_target.getY(),
               safe_sinks.size());
  }
  return true;
}

bool RelocateGenerator::backOffForSafeSinks(
    const odb::Point& orig,
    const odb::Point& requested,
    const std::vector<SinkInfo>& safe_sinks,
    const float margin,
    const double wire_res,
    const double wire_cap,
    odb::Point& out_target) const
{
  // Number of bisection steps when shrinking the move. ~12 gives sub-permille
  // resolution on the orig->requested segment, which is finer than DBU.
  constexpr int kBackoffIters = 12;

  auto point_at = [&](double t) {
    return odb::Point(
        orig.getX() + static_cast<int>(t * (requested.getX() - orig.getX())),
        orig.getY() + static_cast<int>(t * (requested.getY() - orig.getY())));
  };

  // No wire RC model or nothing to protect: there is no collateral wire-delay
  // to guard against, so honor the requested target unchanged.
  if (wire_res <= 0.0 || wire_cap <= 0.0 || safe_sinks.empty()) {
    out_target = requested;
    return manhattan(orig, out_target) >= kMinMoveThresholdDbu;
  }

  auto feasible = [&](double t) {
    const odb::Point p = point_at(t);
    for (const SinkInfo& sink : safe_sinks) {
      const double delay_increase
          = branchDelayIncrease(resizer_,
                                wire_res,
                                wire_cap,
                                sink.pin_cap,
                                manhattan(orig, sink.loc),
                                manhattan(p, sink.loc));
      if (sink.slack - delay_increase < margin) {
        return false;
      }
    }
    return true;
  };

  double t = 1.0;
  if (!feasible(1.0)) {
    // t=0 (no move) is always feasible since slacks are unchanged there.
    double lo = 0.0;
    double hi = 1.0;
    for (int i = 0; i < kBackoffIters; ++i) {
      const double mid = 0.5 * (lo + hi);
      if (feasible(mid)) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    t = lo;
  }

  out_target = point_at(t);
  return manhattan(orig, out_target) >= kMinMoveThresholdDbu;
}

}  // namespace rsz
