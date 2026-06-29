// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "SkewScheduler.hh"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "sta/Clock.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/MinMax.hh"
#include "sta/Network.hh"
#include "sta/Path.hh"
#include "sta/PathExpanded.hh"
#include "sta/Sdc.hh"
#include "sta/SdcClass.hh"

namespace rsz {

SkewScheduler::SkewScheduler(Resizer* resizer) : resizer_(resizer)
{
}

void SkewScheduler::init()
{
  dbStaState::init(resizer_->sta_);

  // Make sure arrivals/requireds reflect the current netlist/clock.
  sta_->updateTiming(false);

  sta::Slack worst_slack;
  sta::Vertex* worst_vertex;
  sta_->worstSlack(max_, worst_slack, worst_vertex);
  design_wns_ = worst_slack;

  // Min clock period drives the budget-fraction sweep.
  period_ = 0.0;
  for (const sta::Clock* clk : sta_->cmdSdc()->clocks()) {
    const float p = clk->period();
    if (p > 0.0f && (period_ == 0.0 || p < period_)) {
      period_ = p;
    }
  }
}

void SkewScheduler::build()
{
  init();
  collectArcs();
}

int SkewScheduler::nodeIndex(const sta::Pin* pin)
{
  // Ports and the top instance are fixed references -> ground node 0.
  if (network_->isTopLevelPort(pin)) {
    return 0;
  }
  const sta::Instance* inst = network_->instance(pin);
  if (inst == nullptr || inst == network_->topInstance()) {
    return 0;
  }
  auto it = inst_node_.find(inst);
  if (it != inst_node_.end()) {
    return it->second;
  }
  const int id = n_nodes_++;
  inst_node_[inst] = id;
  node_inst_.push_back(inst);  // node_inst_[id] == inst
  return id;
}

void SkewScheduler::collectArcs()
{
  inst_node_.clear();
  node_inst_.clear();
  node_inst_.push_back(nullptr);  // node 0 = ground
  n_nodes_ = 1;
  setup_map_.clear();
  hold_map_.clear();
  skipped_endpoints_ = 0;
  hold_violation_arcs_ = 0;

  sta::VertexSet& endpoints = sta_->endpoints();
  for (sta::Vertex* end : endpoints) {
    const sta::Pin* end_pin = end->pin();

    // One setup arc (max) and one hold arc (min) from the worst path at this
    // endpoint.  base slack already folds in current latency, clk-to-q,
    // setup/hold and the multi-corner worst case.
    for (const sta::MinMax* mm : {max_, min_}) {
      const sta::Slack slack = sta_->slack(end, mm);
      // Unconstrained endpoints return STA INF; skip them.
      if (!std::isfinite(static_cast<double>(slack))
          || std::abs(static_cast<double>(slack)) >= kInfSlack) {
        if (mm == max_) {
          skipped_endpoints_++;
        }
        continue;
      }
      sta::Path* path = sta_->vertexWorstSlackPath(end, mm);
      if (path == nullptr) {
        continue;
      }
      sta::PathExpanded expanded(path, sta_);
      const sta::Path* start
          = expanded.startPath();  // launch reg Q / input pin
      const sta::Pin* start_pin
          = (start != nullptr) ? start->pin(sta_) : nullptr;
      if (start_pin == nullptr) {
        continue;
      }
      const int a = nodeIndex(start_pin);
      const int b = nodeIndex(end_pin);
      if (a == 0 && b == 0) {
        continue;  // port-to-port: no adjustable skew variable
      }
      const auto key = std::make_pair(a, b);
      const double base = static_cast<double>(slack);
      if (mm == max_) {
        auto it = setup_map_.find(key);
        if (it == setup_map_.end() || base < it->second) {
          setup_map_[key] = base;
        }
      } else {
        if (base < 0.0) {
          hold_violation_arcs_++;
        }
        // Guard: never push any path's hold slack below its current value.
        const double weight = std::max(base, 0.0);
        auto it = hold_map_.find(key);
        if (it == hold_map_.end() || weight < it->second) {
          hold_map_[key] = weight;
        }
      }
    }
  }

  setup_arcs_.clear();
  analyzed_wns_ = std::numeric_limits<double>::max();
  for (const auto& [key, base] : setup_map_) {
    setup_arcs_.push_back({key.first, key.second, base});
    analyzed_wns_ = std::min(analyzed_wns_, base);
  }
  hold_arcs_.clear();
  for (const auto& [key, weight] : hold_map_) {
    hold_arcs_.push_back({key.first, key.second, weight});
  }
  if (setup_arcs_.empty()) {
    analyzed_wns_ = design_wns_;
  }
}

bool SkewScheduler::feasible(const double delta,
                             const double budget,
                             const bool one_sided,
                             std::vector<double>* x_out) const
{
  const int n = n_nodes_;
  // Edge u -> v with weight w encodes the difference constraint x(v)-x(u) <= w.
  std::vector<std::vector<std::pair<int, double>>> adj(n);
  // setup: x(a)-x(b) <= base-delta  =>  edge b -> a, weight base-delta
  for (const SetupArc& s : setup_arcs_) {
    adj[s.b].emplace_back(s.a, s.base - delta);
  }
  // hold: x(b)-x(a) <= weight  =>  edge a -> b, weight
  for (const HoldArc& h : hold_arcs_) {
    adj[h.a].emplace_back(h.b, h.weight);
  }
  // budget / floor edges relative to ground node 0.
  for (int v = 1; v < n; ++v) {
    if (budget >= 0.0) {
      adj[0].emplace_back(v, budget);  // x(v) - x(0) <= budget
    }
    if (one_sided) {
      adj[v].emplace_back(0, 0.0);  // x(0) - x(v) <= 0  ->  x(v) >= x(0)
    } else if (budget >= 0.0) {
      adj[v].emplace_back(0, budget);  // x(v) - x(0) >= -budget
    }
  }

  // SPFA from a virtual super-source (all nodes start at 0, all enqueued).
  // Feasible iff no negative cycle.
  std::vector<double> dist(n, 0.0);
  std::vector<int> relax_count(n, 0);
  std::vector<char> in_queue(n, 1);
  std::deque<int> queue;
  for (int i = 0; i < n; ++i) {
    queue.push_back(i);
  }
  constexpr double kEps = 1e-18;
  while (!queue.empty()) {
    const int u = queue.front();
    queue.pop_front();
    in_queue[u] = 0;
    const double du = dist[u];
    for (const auto& [v, w] : adj[u]) {
      if (du + w < dist[v] - kEps) {
        dist[v] = du + w;
        if (++relax_count[v] >= n) {
          return false;  // negative cycle -> infeasible
        }
        if (!in_queue[v]) {
          in_queue[v] = 1;
          queue.push_back(v);
        }
      }
    }
  }

  if (x_out != nullptr) {
    const double ground = dist[0];
    x_out->assign(n, 0.0);
    for (int i = 0; i < n; ++i) {
      double xi = dist[i] - ground;  // normalize so ground = 0
      if (one_sided && xi < 0.0) {
        xi = 0.0;  // clamp tiny numerical negatives
      }
      (*x_out)[i] = xi;
    }
  }
  return true;
}

double SkewScheduler::maxAchievableWns(const double budget,
                                       const bool one_sided,
                                       bool& capped_out) const
{
  capped_out = false;
  double lo = analyzed_wns_;  // always feasible (x = 0 reproduces base slacks)
  const double span
      = (period_ > 0.0) ? period_ : (std::abs(analyzed_wns_) + 1e-9);
  double hi = analyzed_wns_ + span;
  int expand = 0;
  while (feasible(hi, budget, one_sided, nullptr)) {
    lo = hi;
    hi += span;
    if (++expand > 8) {
      capped_out = true;  // headroom exceeds the search cap
      return lo;
    }
  }
  while (hi - lo > kDeltaTol) {
    const double mid = 0.5 * (lo + hi);
    if (feasible(mid, budget, one_sided, nullptr)) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}

bool SkewScheduler::solveVector(const double delta,
                                const double budget,
                                const bool one_sided,
                                std::vector<double>& x_out) const
{
  return feasible(delta, budget, one_sided, &x_out);
}

bool SkewScheduler::solveSchedule(
    const double delta,
    const double budget,
    const bool one_sided,
    std::map<const sta::Instance*, double>& skew_out) const
{
  std::vector<double> x;
  if (!feasible(delta, budget, one_sided, &x)) {
    return false;
  }
  skew_out.clear();
  for (int i = 1; i < n_nodes_; ++i) {
    skew_out[node_inst_[i]] = x[i];
  }
  return true;
}

}  // namespace rsz
