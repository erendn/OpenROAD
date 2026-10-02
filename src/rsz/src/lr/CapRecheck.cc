// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "CapRecheck.hh"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <vector>

#include "ElectricalModel.hh"
#include "LrState.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "rsz/GlobalSizingConfig.hh"
#include "rsz/Resizer.hh"
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/PortDirection.hh"
#include "sta/Sta.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

std::vector<int> selectCapReverts(
    const float slack,
    const std::vector<CapContribution>& contributions)
{
  std::vector<int> reverts;
  if (slack >= 0.0f) {
    return reverts;
  }
  // Only a move that pushed this net toward violation can give slack back.
  std::vector<CapContribution> harmful;
  harmful.reserve(contributions.size());
  for (const CapContribution& c : contributions) {
    if (c.slack_delta < 0.0f) {
      harmful.push_back(c);
    }
  }
  // Largest offender first, so as few moves as possible are reverted. The
  // mover index breaks ties, so the result depends only on the commit order.
  std::ranges::sort(harmful,
                    [](const CapContribution& a, const CapContribution& b) {
                      if (a.slack_delta != b.slack_delta) {
                        return a.slack_delta < b.slack_delta;
                      }
                      return a.mover < b.mover;
                    });
  float recovered = slack;
  for (const CapContribution& c : harmful) {
    if (recovered >= 0.0f) {
      break;
    }
    recovered -= c.slack_delta;  // slack_delta < 0, so this adds slack back
    reverts.push_back(c.mover);
  }
  return reverts;
}

float driverLimitDelta(const bool cur_liberty_exists,
                       const float cur_liberty_limit,
                       const bool prev_liberty_exists,
                       const float prev_liberty_limit,
                       const float effective_limit)
{
  if (!cur_liberty_exists || effective_limit < cur_liberty_limit) {
    return 0.0f;  // an SDC limit binds; the swap did not change it
  }
  if (!prev_liberty_exists || prev_liberty_limit <= 0.0f) {
    return 0.0f;  // no limit before the swap; see the header
  }
  return effective_limit - prev_liberty_limit;
}

namespace {

// OpenSTA's live max-cap check on the net driven by `drvr_pin`. The limit is
// the tighter of the pin's current Liberty port limit and any SDC limit, and
// the load comes from the net's current pins and parasitics. Unlike the sweep's
// output-side filter, which reads only LibertyPort::capacitanceLimit, this
// includes SDC limits and so matches OpenSTA's own max-capacitance checks.
struct LiveCapCheck
{
  bool limit_exists = false;
  float limit = 0.0f;
  float slack = 0.0f;
};

LiveCapCheck liveCapCheck(sta::dbSta* sta,
                          const sta::Pin* drvr_pin,
                          const sta::MinMax* max_mm)
{
  float cap = 0.0f;
  float limit = 0.0f;
  float slack = 0.0f;
  const sta::RiseFall* rf = nullptr;
  const sta::Scene* scene = nullptr;
  sta->checkCapacitance(
      drvr_pin, sta->scenes(), max_mm, cap, limit, slack, rf, scene);
  LiveCapCheck out;
  // Same validity test the snapshot applies to its frozen per-driver checks
  // (LRSubproblem::DriverCapCheck::corner_ok).
  out.limit_exists = (scene != nullptr && limit > 0.0f);
  out.limit = limit;
  out.slack = slack;
  return out;
}

// STA wrapper over the pure driverLimitDelta: read the port limits of the
// committed cell and of the pre-sweep cell and pass both to it.
float driverLimitDelta(const sta::LibertyCell* prev_cell,
                       const sta::LibertyPort* out_port,
                       const sta::MinMax* max_mm,
                       const LiveCapCheck& live)
{
  float cur_limit = 0.0f;
  bool cur_exists = false;
  out_port->capacitanceLimit(max_mm, cur_limit, cur_exists);
  const sta::LibertyPort* prev_port
      = prev_cell->findLibertyPort(out_port->name());
  float prev_limit = 0.0f;
  bool prev_exists = false;
  if (prev_port != nullptr) {
    prev_port->capacitanceLimit(max_mm, prev_limit, prev_exists);
  }
  return rsz::driverLimitDelta(
      cur_exists, cur_limit, prev_exists, prev_limit, live.limit);
}

// One net under re-check, keyed by the driver pin that carries its limit.
struct NetRecheck
{
  LiveCapCheck live;
  std::vector<CapContribution> contributions;
};

// The per-pass working set: every net a moved gate touched, with each gate's
// signed effect on it. Kept in insertion order (the map only locates entries)
// so iteration is deterministic.
class NetTable
{
 public:
  // Add `mover`'s effect on the net driven by `drvr_pin`, querying the live
  // check once per net. Deltas accumulate: two input pins of one gate can be
  // on the same net, and reverting the gate gives back both.
  void add(sta::dbSta* sta,
           const sta::Pin* drvr_pin,
           const sta::MinMax* max_mm,
           const int mover,
           const float slack_delta,
           const LiveCapCheck* known_live = nullptr)
  {
    const auto [it, inserted] = index_.try_emplace(drvr_pin, nets_.size());
    if (inserted) {
      NetRecheck net;
      net.live = (known_live != nullptr) ? *known_live
                                         : liveCapCheck(sta, drvr_pin, max_mm);
      nets_.push_back(std::move(net));
    }
    NetRecheck& net = nets_[it->second];
    for (CapContribution& c : net.contributions) {
      if (c.mover == mover) {
        c.slack_delta += slack_delta;
        return;
      }
    }
    net.contributions.push_back({.mover = mover, .slack_delta = slack_delta});
  }

  const std::vector<NetRecheck>& nets() const { return nets_; }

  void clear()
  {
    nets_.clear();
    index_.clear();
  }

 private:
  std::vector<NetRecheck> nets_;
  std::unordered_map<const sta::Pin*, size_t> index_;
};

// Under relax_max_cap, a violation on a net whose driver has a
// max-capacitance multiplier is priced by the next iteration's multiplier and,
// if global sizing resizes the driver, repaired by the fix pass, as in
// Livramento et al. Reverting the move would undo the relaxation, so the
// re-check leaves such a net, even when its driver is a register the fix pass
// cannot resize.
bool pricedByMultiplier(const LrState& state, const sta::Pin* drvr_pin)
{
  if (!state.config->relax_max_cap) {
    return false;
  }
  const sta::Vertex* v = state.graph->pinDrvrVertex(drvr_pin);
  return v != nullptr
         && state.cap_multipliers.beta(state.graph->id(v)).has_value();
}

// Record the net that load pin `pin` of moved gate `mover_index` is on, with
// the slack `delta` the gate's larger input capacitance cost it. The cap limit
// on a net lives on its driver pin(s), not on this load pin.
void addLoadedNet(const LrState& state,
                  const sta::Pin* pin,
                  const int mover_index,
                  const float delta,
                  NetTable& table)
{
  sta::PinSet* drivers = state.network->drivers(pin);
  if (drivers == nullptr) {
    return;
  }
  for (const sta::Pin* drvr_pin : *drivers) {
    if (!pricedByMultiplier(state, drvr_pin)) {
      table.add(state.sta, drvr_pin, state.max, mover_index, delta);
    }
  }
}

// Record every net one moved gate touches, with the signed slack effect its
// cell change had on that net. A net relax_max_cap prices is left out.
void collectMoverNets(LrState& state,
                      const MovedGate& mover,
                      const int mover_index,
                      const sta::LibertyCell* cur_cell,
                      NetTable& table)
{
  sta::Network* network = state.network;
  sta::dbSta* sta = state.sta;
  const sta::MinMax* max_mm = state.max;

  std::unique_ptr<sta::InstancePinIterator> pit(
      network->pinIterator(mover.inst));
  while (pit->hasNext()) {
    sta::Pin* pin = pit->next();
    const sta::PortDirection* dir = network->direction(pin);
    if (dir->isOutput()) {
      const sta::LibertyPort* out_port = network->libertyPort(pin);
      if (out_port == nullptr) {
        continue;
      }
      if (pricedByMultiplier(state, pin)) {
        continue;
      }
      const LiveCapCheck live = liveCapCheck(sta, pin, max_mm);
      if (!live.limit_exists) {
        continue;
      }
      table.add(sta,
                pin,
                max_mm,
                mover_index,
                driverLimitDelta(mover.prev_cell, out_port, max_mm, live),
                &live);
    } else if (dir->isInput()) {
      const sta::LibertyPort* in_port = network->libertyPort(pin);
      if (in_port == nullptr) {
        continue;
      }
      const char* port_name = in_port->name().c_str();
      // Slack delta = -(cap delta): a bigger input pin loads the driver more.
      const float delta = portInputCap(mover.prev_cell, port_name, max_mm)
                          - portInputCap(cur_cell, port_name, max_mm);
      if (delta >= 0.0f) {
        continue;  // this move did not add load to its driver net
      }
      addLoadedNet(state, pin, mover_index, delta, table);
    }
  }
}

}  // namespace

CapRecheckStats recheckMaxCapAfterSweep(LrState& state,
                                        std::vector<MovedGate>& movers)
{
  CapRecheckStats stats;
  if (movers.empty()) {
    return stats;
  }

  sta::Network* network = state.network;
  NetTable table;
  std::vector<bool> revert;
  std::vector<MovedGate> kept;

  for (int pass = 0; pass < kCapRecheckMaxPasses && !movers.empty(); ++pass) {
    // The check reads live loads, so first update the parasitics invalidated
    // by the previous commits or reverts. This is incremental: only touched
    // nets are re-estimated, and under the Gauss-Seidel engine the per-commit
    // refresh has already done it.
    state.resizer->estimateParasitics()->updateParasitics();
    ++stats.passes;

    table.clear();
    for (size_t i = 0; i < movers.size(); ++i) {
      const sta::LibertyCell* cur_cell = network->libertyCell(movers[i].inst);
      if (cur_cell != nullptr && cur_cell != movers[i].prev_cell) {
        collectMoverNets(
            state, movers[i], static_cast<int>(i), cur_cell, table);
      }
    }

    revert.assign(movers.size(), false);
    bool any = false;
    for (const NetRecheck& net : table.nets()) {
      if (!net.live.limit_exists) {
        continue;
      }
      for (const int mover :
           selectCapReverts(net.live.slack, net.contributions)) {
        revert[mover] = true;
        any = true;
      }
    }
    if (!any) {
      break;
    }

    kept.clear();
    for (size_t i = 0; i < movers.size(); ++i) {
      if (!revert[i]) {
        kept.push_back(movers[i]);
        continue;
      }
      if (state.resizer->replaceCell(movers[i].inst, movers[i].prev_cell)) {
        ++stats.reverted;
      }
      // Drop the gate from the moved set even if the revert failed (the
      // pre-sweep master is gone from the db). A failed revert cannot succeed
      // on a later pass, and keeping it would re-select it on every pass and
      // break the guarantee that each pass shrinks the set.
    }
    movers = kept;
    stats.bound_hit = (pass + 1 == kCapRecheckMaxPasses);
  }

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR cap re-check: {} pass(es), {} move(s) reverted, {} kept{}",
             stats.passes,
             stats.reverted,
             movers.size(),
             stats.bound_hit ? " (pass bound reached)" : "");
  return stats;
}

}  // namespace rsz
