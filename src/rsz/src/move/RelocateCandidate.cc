// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "RelocateCandidate.hh"

#include <cmath>
#include <cstdlib>
#include <memory>

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "dpl/Opendp.h"
#include "est/EstimateParasitics.h"
#include "odb/db.h"
#include "odb/geom.h"
#include "rsz/Resizer.hh"
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Path.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

namespace {

inline int manhattan(const odb::Point& a, const odb::Point& b)
{
  return std::abs(a.getX() - b.getX()) + std::abs(a.getY() - b.getY());
}

}  // namespace

RelocateCandidate::RelocateCandidate(Resizer& resizer,
                                     const Target& target,
                                     sta::Instance* drvr_inst,
                                     sta::Pin* drvr_pin,
                                     const odb::Point& orig_loc,
                                     const odb::Point& requested_loc,
                                     const float setup_slack_margin,
                                     const bool legalize,
                                     const int max_skew_dbu)
    : MoveCandidate(resizer, target),
      drvr_inst_(drvr_inst),
      drvr_pin_(drvr_pin),
      orig_loc_(orig_loc),
      requested_loc_(requested_loc),
      setup_slack_margin_(setup_slack_margin),
      legalize_(legalize),
      max_skew_dbu_(max_skew_dbu)
{
}

Estimate RelocateCandidate::estimate()
{
  // Real score is not available without simulating the legalized location.
  // Apply-time guards reject non-improving moves; this score only signals
  // legality for policies that rank candidates of the same type.
  return {.legal = true, .score = 1.0f};
}

MoveResult RelocateCandidate::apply()
{
  sta::dbNetwork* db_network = resizer_.dbNetwork();
  odb::dbInst* db_inst = db_network->staToDb(drvr_inst_);
  dpl::Opendp* opendp = resizer_.opendp();
  if (db_inst == nullptr || opendp == nullptr) {
    return rejectedMove();
  }

  // Legalize the requested target to a real odb location the same way buffer
  // insertion does (Resizer::insertBufferPostProcess): clamp to core, then
  // optionally snap off-macro / off-die / on-row via dpl's existing
  // legalCellPos. We only need a legal *point*; the dpl grid is intentionally
  // not reserved (matching buffer insertion), and the detailed_placement after
  // repair_timing rebuilds it. Stage the request on the db inst, read the
  // snapped location back, then restore the original location so the guard
  // phase -- and every reject path below -- leaves odb untouched. Only the
  // final commit re-applies new_loc.
  resizer_.setLocation(db_inst, requested_loc_);
  if (legalize_) {
    opendp->legalCellPos(db_inst);
  }
  const odb::Point new_loc = db_inst->getLocation();
  db_inst->setLocation(orig_loc_.getX(), orig_loc_.getY());

  const int placement_skew = manhattan(new_loc, requested_loc_);
  if (placement_skew > max_skew_dbu_) {
    debugPrint(resizer_.logger(),
               RSZ,
               "relocate_move",
               2,
               "REJECT RelocateMove {}: placement skew {} > limit {}",
               resizer_.network()->pathName(drvr_pin_),
               placement_skew,
               max_skew_dbu_);
    return rejectedMove();
  }

  // Max-cap guards. Other moves rely on the outer journal to roll back
  // WNS/TNS regressions, but max-cap is a hard liberty limit the journal
  // doesn't watch. The wirelength of both the output net (driver -> loads)
  // and the input net (prev driver -> this gate's input pin) changes with
  // the relocation, so check both.
  //
  // The same output-net walk also runs the collateral-slack guard: the
  // generator already backed the requested target off so currently-safe sinks
  // stay above the margin, but legalCellPos may have snapped the actual
  // location up to max_skew_dbu_ away. Re-check against the committed new_loc
  // and reject if that snap would push a safe sink into violation.
  const sta::Scene* scene = resizer_.sta()->cmdScene();
  const sta::MinMax* max_mode = resizer_.maxAnalysisMode();
  const sta::Path* drvr_path = target_.driverPath(resizer_);
  const sta::RiseFall* rf
      = drvr_path != nullptr ? drvr_path->transition(resizer_.sta()) : nullptr;
  const double wire_cap
      = resizer_.estimateParasitics()->wireSignalCapacitance(scene);
  const double wire_res
      = resizer_.estimateParasitics()->wireSignalResistance(scene);
  if (wire_cap > 0.0) {
    sta::Vertex* drvr_vertex = resizer_.graph()->pinDrvrVertex(drvr_pin_);
    if (drvr_vertex != nullptr) {
      double old_out_hpwl_dbu = 0.0;
      double new_out_hpwl_dbu = 0.0;
      sta::VertexOutEdgeIterator out_iter(drvr_vertex, resizer_.graph());
      while (out_iter.hasNext()) {
        sta::Edge* edge = out_iter.next();
        if (!edge->isWire()) {
          continue;
        }
        sta::Vertex* load_vertex = edge->to(resizer_.graph());
        const sta::Pin* load_pin = load_vertex->pin();
        const odb::Point load_loc = db_network->location(load_pin);
        old_out_hpwl_dbu += manhattan(orig_loc_, load_loc);
        new_out_hpwl_dbu += manhattan(new_loc, load_loc);

        // Collateral-slack guard for a currently-safe sink. First-order
        // Elmore branch-delay increase (mirrors RelocateGenerator's model):
        // added series R_w*dlen drives the sink pin cap plus half the new
        // wire cap.
        if (wire_res > 0.0) {
          const sta::Slack sink_slack
              = resizer_.sta()->slack(load_vertex, max_mode);
          if (sink_slack >= setup_slack_margin_) {
            const int len_old = manhattan(orig_loc_, load_loc);
            const int len_new = manhattan(new_loc, load_loc);
            const double dlen_m
                = resizer_.dbuToMeters(len_new) - resizer_.dbuToMeters(len_old);
            if (dlen_m > 0.0) {
              sta::LibertyPort* load_port
                  = resizer_.network()->libertyPort(load_pin);
              const double pin_cap = (load_port != nullptr && rf != nullptr)
                                         ? load_port->capacitance(rf, max_mode)
                                         : 0.0;
              const double delay_increase
                  = wire_res * dlen_m
                    * (pin_cap
                       + 0.5 * wire_cap * resizer_.dbuToMeters(len_new));
              if (sink_slack - delay_increase < setup_slack_margin_) {
                debugPrint(
                    resizer_.logger(),
                    RSZ,
                    "relocate_move",
                    2,
                    "REJECT RelocateMove {}: dpl snap would violate safe sink "
                    "{} (slack {:.3e} - dD {:.3e} < margin {:.3e})",
                    resizer_.network()->pathName(drvr_pin_),
                    resizer_.network()->pathName(load_pin),
                    sink_slack,
                    delay_increase,
                    setup_slack_margin_);
                return rejectedMove();
              }
            }
          }
        }
      }
      const double out_cap_delta
          = wire_cap
            * (resizer_.dbuToMeters(static_cast<int>(new_out_hpwl_dbu))
               - resizer_.dbuToMeters(static_cast<int>(old_out_hpwl_dbu)));
      if (out_cap_delta > 0.0
          && !resizer_.checkMaxCapOK(drvr_pin_,
                                     static_cast<float>(out_cap_delta))) {
        debugPrint(resizer_.logger(),
                   RSZ,
                   "relocate_move",
                   2,
                   "REJECT RelocateMove {}: output max-cap would worsen "
                   "(cap_delta={:.3e})",
                   resizer_.network()->pathName(drvr_pin_),
                   out_cap_delta);
        return rejectedMove();
      }
    }

    // Input-net wirelength change, single-sink approximation:
    // delta ~= manhattan(prev_drvr, new) - manhattan(prev_drvr, orig).
    const sta::Path* prev_drvr_path = target_.prevDriverPath(resizer_);
    if (prev_drvr_path != nullptr) {
      const sta::Pin* prev_drvr_pin = prev_drvr_path->pin(resizer_.sta());
      if (prev_drvr_pin != nullptr
          && !resizer_.network()->isTopLevelPort(prev_drvr_pin)) {
        const odb::Point prev_drvr_loc = db_network->location(prev_drvr_pin);
        const int old_in_hpwl_dbu = manhattan(prev_drvr_loc, orig_loc_);
        const int new_in_hpwl_dbu = manhattan(prev_drvr_loc, new_loc);
        const double in_cap_delta = wire_cap
                                    * (resizer_.dbuToMeters(new_in_hpwl_dbu)
                                       - resizer_.dbuToMeters(old_in_hpwl_dbu));
        if (in_cap_delta > 0.0
            && !resizer_.checkMaxCapOK(prev_drvr_pin,
                                       static_cast<float>(in_cap_delta))) {
          debugPrint(resizer_.logger(),
                     RSZ,
                     "relocate_move",
                     2,
                     "REJECT RelocateMove {}: input max-cap on {} would worsen "
                     "(cap_delta={:.3e})",
                     resizer_.network()->pathName(drvr_pin_),
                     resizer_.network()->pathName(prev_drvr_pin),
                     in_cap_delta);
          return rejectedMove();
        }
      }
    }
  }

  // Commit: only mutate odb. Both kOrigin and kFlags are recorded by the
  // open ECO journal, so an outer journal restore undoes the relocation.
  // dpl's grid is intentionally left stale (matching replaceCell / buffer
  // insertion); the detailed_placement step after repair_timing rebuilds it.
  db_inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
  db_inst->setLocation(new_loc.getX(), new_loc.getY());

  // Mark every net touching the relocated instance as stale.
  std::unique_ptr<sta::InstancePinIterator> pin_iter(
      resizer_.network()->pinIterator(drvr_inst_));
  while (pin_iter->hasNext()) {
    const sta::Pin* pin = pin_iter->next();
    sta::Net* net = resizer_.network()->net(pin);
    if (net != nullptr) {
      resizer_.estimateParasitics()->parasiticsInvalid(net);
    }
  }

  debugPrint(resizer_.logger(),
             RSZ,
             "relocate_move",
             1,
             "ACCEPT RelocateMove {}: ({},{}) -> ({},{}) [requested ({},{})]",
             resizer_.network()->pathName(drvr_pin_),
             orig_loc_.getX(),
             orig_loc_.getY(),
             new_loc.getX(),
             new_loc.getY(),
             requested_loc_.getX(),
             requested_loc_.getY());

  return {
      .accepted = true,
      .type = MoveType::kRelocate,
      .move_count = 1,
      .touched_instances = {drvr_inst_},
  };
}

}  // namespace rsz
