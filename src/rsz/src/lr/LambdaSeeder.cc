// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "LambdaSeeder.hh"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <unordered_map>

#include "db_sta/dbSta.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Sta.hh"
#include "sta/Transition.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

namespace {

// Worst rise/fall arrival at v under the policy scenes / max corner (same read
// the lambda updaters use).
float arrivalAt(LrState& s, sta::Vertex* v)
{
  return sta::delayAsFloat(
      s.sta->arrival(v, sta::RiseFallBoth::riseFall(), s.sta->scenes(), s.max));
}

bool isSentinel(float x)
{
  return std::fabs(x) >= LrState::kSlackSentinel;
}

// WNS-biased endpoint mu seed shared by the delay-proportional and constant
// seeders: mu_k = max(0, margin - slack_k)^p, then normalized so max(mu) = 1.
// Verbatim from the M0 baseline so those seeders stay bit-exact.
void seedBaselineMu(LrState& state)
{
  const GlobalSizingConfig& params = *state.config;
  float mu_max_raw = 0.0f;
  const float margin = params.setup_slack_margin;
  const float p = params.mu_exponent;
  for (size_t k = 0; k < state.endpoint_vertices.size(); ++k) {
    const sta::Slack slack
        = state.sta->slack(state.endpoint_vertices[k], state.max);
    const float slack_f = sta::delayAsFloat(slack);
    const float gap = margin - slack_f;
    const float mu = (gap > 0.0f) ? std::pow(gap, p) : 0.0f;
    state.mu[k] = mu;
    mu_max_raw = std::max(mu_max_raw, mu);
  }
  if (mu_max_raw > 0.0f) {
    for (float& mu : state.mu) {
      mu /= mu_max_raw;
    }
  }
}

// Minimum leakage over the cell's equivalent (swappable) group, including the
// cell itself (Mangiras minP(g)). Cached per LibertyCell. nullopt when no cell
// in the group reports a leakage value.
//
// Scope note (C3 §4.6, mangiras audit §13.11): the cache is keyed by
// LibertyCell, so it CANNOT be instance-conditional — DRC feasibility of the
// min-leakage cell is not consulted, and a dont_touch instance still counts at
// its group minimum in the global power ratio. This is a decided reading of a
// paper ambiguity; note the mild scope inconsistency that
// collectTimingScaleInput (TimingScale.cc) DOES skip dont_touch instances.
using MinLeakCache
    = std::unordered_map<sta::LibertyCell*, std::optional<float>>;
std::optional<float> minLeakEquivalent(Resizer& resizer,
                                       sta::LibertyCell* cell,
                                       MinLeakCache& cache)
{
  const auto it = cache.find(cell);
  if (it != cache.end()) {
    return it->second;
  }
  std::optional<float> best = resizer.cellLeakage(cell);
  for (sta::LibertyCell* candidate : resizer.getSwappableCells(cell)) {
    const std::optional<float> leak = resizer.cellLeakage(candidate);
    if (leak.has_value()) {
      best = best.has_value() ? std::min(*best, *leak) : *leak;
    }
  }
  cache.emplace(cell, best);
  return best;
}

}  // namespace

// ===========================================================================
// Pure formula cores (unit-tested; no STA).
// ===========================================================================

float constantSeedLambda(const float value, const float floor)
{
  return std::max(value, floor);
}

float mangirasInternalArcLambda(const float a_from,
                                const float d,
                                const float a_to,
                                const float leak,
                                const float min_leak,
                                const float exponent)
{
  const float tr = a_to > 0.0f ? std::max(0.0f, (a_from + d) / a_to) : 1.0f;
  const float pr = min_leak > 0.0f ? std::max(0.0f, leak / min_leak) : 1.0f;
  return std::pow(tr * pr, exponent);
}

float mangirasEndpointArcLambda(const float a_k,
                                const float r_k,
                                const float total_leak,
                                const float total_min_leak,
                                const float exponent)
{
  const float tr = r_k > 0.0f ? std::max(0.0f, a_k / r_k) : 1.0f;
  const float pr = total_min_leak > 0.0f
                       ? std::max(0.0f, total_leak / total_min_leak)
                       : 1.0f;
  return std::pow(tr * pr, exponent);
}

// ===========================================================================
// Seeder implementations.
// ===========================================================================

void DelayPropCritMuSeeder::seed(LrState& state)
{
  const GlobalSizingConfig& params = *state.config;
  sta::Graph* graph = state.graph;

  // λ_e = d_e  (delay-proportional seed, max arc delay across rise/fall)
  float lambda_sum = 0.0f;
  float lambda_max = 0.0f;
  int seeded = 0;
  sta::VertexIterator vit(graph);
  while (vit.hasNext()) {
    sta::Vertex* v = vit.next();
    sta::VertexOutEdgeIterator eit(v, graph);
    while (eit.hasNext()) {
      sta::Edge* e = eit.next();
      if (!state.isDataArc(e)) {
        continue;
      }
      const float d = state.edgeMaxArcDelay(e);
      const sta::EdgeId id = graph->id(e);
      const float seed = std::max(d, params.lambda_floor);
      state.lambda[id] = seed;
      lambda_sum += seed;
      lambda_max = std::max(lambda_max, seed);
      ++seeded;
    }
  }

  // μ_k = max(0, margin - slack_k)^p, normalized to max 1 - decouples the LR
  // pressure's scale from the raw slack units so the downstream λ·d terms are
  // predictable.
  seedBaselineMu(state);

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR seed delay_prop: {} data arcs (λ sum={:.3g}, max={:.3g}, "
             "avg={:.3g})",
             seeded,
             lambda_sum,
             lambda_max,
             seeded ? lambda_sum / seeded : 0.0f);
}

void ConstantSeeder::seed(LrState& state)
{
  const GlobalSizingConfig& params = *state.config;
  sta::Graph* graph = state.graph;

  const float value
      = constantSeedLambda(params.lambda_init_value, params.lambda_floor);
  int seeded = 0;
  sta::VertexIterator vit(graph);
  while (vit.hasNext()) {
    sta::Vertex* v = vit.next();
    sta::VertexOutEdgeIterator eit(v, graph);
    while (eit.hasNext()) {
      sta::Edge* e = eit.next();
      if (!state.isDataArc(e)) {
        continue;
      }
      state.lambda[graph->id(e)] = value;
      ++seeded;
    }
  }

  // mu follows the baseline (papers under-specify endpoint multipliers). Under
  // mu_policy = endpoint_lambda the projection derives mu from the endpoint's
  // own arcs and overwrites this - which is the point: a mu seeded from slack
  // carries no trace of `value`, so anchoring to it is what cancels the seed
  // constant (see projectFlowBalance).
  seedBaselineMu(state);

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR seed constant: {} data arcs at λ={:.3g}",
             seeded,
             value);
}

void StateAdaptiveSeeder::seed(LrState& state)
{
  const GlobalSizingConfig& params = *state.config;
  Resizer& resizer = *state.resizer;
  sta::Network* network = state.network;
  sta::Graph* graph = state.graph;
  const float K = params.lambda_seed_exponent;
  const float floor = params.lambda_floor;
  // Floor the seed and reject any non-finite value (a pathological arrival /
  // required time can overflow the ratio^K) so a bad seed never propagates
  // through the projection as NaN - mirrors the isfinite guard the updaters
  // use.
  //
  // C3 §5.2 (mangiras audit): note the inf -> floor completion points the WRONG
  // way. A non-finite ratio^K arises from a tiny positive r_k on a severely
  // VIOLATING endpoint, i.e. the most violating endpoint would seed the LEAST
  // pressure. This is latent, not live: reaching it needs (a/r)·pr >~ 2e19,
  // unreachable on physical designs, so the direction is left as-is rather than
  // adding an untested overflow-to-ceiling branch. The r_k <= 0 -> neutral case
  // (handled in the pure cores) is the documented, reachable guard.
  const auto finiteSeed = [floor](float lam) {
    return std::isfinite(lam) ? std::max(lam, floor) : floor;
  };

  // Design-global leakage and its virtual minimum (Eq. 6 power ratio), plus the
  // per-gate min-leakage cache reused for the internal-arc power ratio (Eq. 5).
  MinLeakCache min_leak_cache;
  double total_leak = 0.0;
  double total_min_leak = 0.0;
  {
    std::unique_ptr<sta::LeafInstanceIterator> iit(
        network->leafInstanceIterator());
    while (iit->hasNext()) {
      sta::Instance* inst = iit->next();
      sta::LibertyCell* cell = network->libertyCell(inst);
      if (cell == nullptr) {
        continue;
      }
      const std::optional<float> leak = resizer.cellLeakage(cell);
      if (!leak.has_value()) {
        continue;
      }
      const std::optional<float> min_leak
          = minLeakEquivalent(resizer, cell, min_leak_cache);
      total_leak += *leak;
      total_min_leak += min_leak.value_or(*leak);
    }
  }

  // Endpoint mu is accumulated per endpoint (Eq. 6 is the projection boundary),
  // so start from zero.
  std::fill(state.mu.begin(), state.mu.end(), 0.0f);

  int internal = 0;
  int endpoint = 0;
  sta::VertexIterator vit(graph);
  while (vit.hasNext()) {
    sta::Vertex* v = vit.next();
    sta::VertexOutEdgeIterator eit(v, graph);
    while (eit.hasNext()) {
      sta::Edge* e = eit.next();
      if (!state.isDataArc(e)) {
        continue;
      }
      const sta::EdgeId id = graph->id(e);
      sta::Vertex* from_v = e->from(graph);
      sta::Vertex* to_v = e->to(graph);

      const auto ep_it = state.endpoint_index.find(to_v);
      if (ep_it != state.endpoint_index.end()) {
        // Eq. 6: arc into a timing endpoint.
        const float a_k = arrivalAt(state, to_v);
        // Consistent required: read the endpoint's required time directly
        // rather than reconstructing r_k = a_k + slack_k, which mixes the
        // worst-arrival and worst-slack transitions and can undershoot the true
        // required, inflating a_k / r_k on near-critical endpoints (mangiras
        // audit §5.1; C3 item 1).
        const float r_k = sta::delayAsFloat(
            state.sta->required(to_v,
                                sta::RiseFallBoth::riseFall(),
                                state.sta->scenes(),
                                state.max));
        if (isSentinel(a_k) || isSentinel(r_k)) {
          state.lambda[id] = floor;
          continue;
        }
        const float lam
            = mangirasEndpointArcLambda(a_k,
                                        r_k,
                                        static_cast<float>(total_leak),
                                        static_cast<float>(total_min_leak),
                                        K);
        const float lam_f = finiteSeed(lam);
        state.lambda[id] = lam_f;
        state.mu[ep_it->second] += lam_f;
        ++endpoint;
      } else {
        // Eq. 5: internal arc i->j of gate g. g is the arc's driver instance
        // (the gate whose size controls this arc's delay in our per-edge model;
        // adaptation note - the paper's collapsed arc keys leakage to the sink
        // gate).
        //
        // Consistent (a_from, d, a_to) read so tr = (a_from + d)/a_to stays
        // <= 1 on the critical arc rather than exceeding it through the
        // rise/fall max-collapse (mangiras audit §5.1, seed flavor; C3 item 1).
        const LrState::ConsistentArcRead r
            = state.consistentArcRead(e, from_v, to_v);
        if (isSentinel(r.a_from) || isSentinel(r.a_to)) {
          state.lambda[id] = floor;
          continue;
        }
        float leak = 1.0f;
        float min_leak = 1.0f;
        const sta::Instance* g = network->instance(from_v->pin());
        sta::LibertyCell* cell
            = g != nullptr ? network->libertyCell(g) : nullptr;
        if (cell != nullptr) {
          const std::optional<float> leak_opt = resizer.cellLeakage(cell);
          const std::optional<float> min_opt
              = minLeakEquivalent(resizer, cell, min_leak_cache);
          if (leak_opt.has_value() && min_opt.has_value()) {
            leak = *leak_opt;
            min_leak = *min_opt;
          }
        }
        const float lam = mangirasInternalArcLambda(
            r.a_from, r.d, r.a_to, leak, min_leak, K);
        state.lambda[id] = finiteSeed(lam);
        ++internal;
      }
    }
  }

  debugPrint(
      state.logger,
      RSZ,
      "global_sizing",
      2,
      "LR seed state_adaptive: {} internal + {} endpoint arcs, K={:.3g}, "
      "total_leak={:.3g} total_min_leak={:.3g} (Eq.7 rescale by the "
      "projection)",
      internal,
      endpoint,
      K,
      total_leak,
      total_min_leak);
}

void EstimationLoopSeeder::seed(LrState& state)
{
  // Reimann Alg. 2 line 2 "set initial multipliers": use the baseline
  // delay-proportional / crit^p seed as the estimation starting point. The
  // driver's runEstimationLoop then refines lambda via dry-run sweeps.
  DelayPropCritMuSeeder().seed(state);
  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR seed estimation_loop: baseline start seeded ({} est iters "
             "follow in the driver)",
             state.config->est_loop_iters);
}

std::unique_ptr<LambdaSeeder> makeLambdaSeeder(const GlobalSizingConfig& config)
{
  switch (config.lambda_seed) {
    case GlobalSizingConfig::LambdaSeed::kDelayPropCritMu:
      return std::make_unique<DelayPropCritMuSeeder>();
    case GlobalSizingConfig::LambdaSeed::kConstant:
      return std::make_unique<ConstantSeeder>();
    case GlobalSizingConfig::LambdaSeed::kStateAdaptive:
      return std::make_unique<StateAdaptiveSeeder>();
    case GlobalSizingConfig::LambdaSeed::kEstimationLoop:
      return std::make_unique<EstimationLoopSeeder>();
  }
  return std::make_unique<DelayPropCritMuSeeder>();
}

}  // namespace rsz
