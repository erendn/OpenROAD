// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "LambdaUpdater.hh"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>

#include "db_sta/dbSta.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/Sta.hh"
#include "sta/Transition.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

namespace {

// Worst rise/fall arrival at v over all scenes, max analysis.
float arrivalOf(LrState& s, sta::Vertex* v)
{
  return sta::delayAsFloat(
      s.sta->arrival(v, sta::RiseFallBoth::riseFall(), s.sta->scenes(), s.max));
}

// Worst slack at v, max analysis.
float slackOf(LrState& s, sta::Vertex* v)
{
  return sta::delayAsFloat(s.sta->slack(v, s.max));
}

bool isSentinel(float x)
{
  return std::fabs(x) >= LrState::kSlackSentinel;
}

// Calls fn(edge, id, from_vertex, to_vertex) once for every data arc that has
// a multiplier slot, in (vertex, out-edge) order. Shared by all updaters so
// they select arcs the same way.
template <typename Fn>
void forEachDataArc(LrState& state, Fn&& fn)
{
  sta::Graph* graph = state.graph;
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
      if (static_cast<size_t>(id) >= state.lambda.size()) {
        continue;
      }
      fn(e, id, e->from(graph), e->to(graph));
    }
  }
}

// Stores the updated multiplier unless a multiplicative step overflowed it to
// inf or NaN. On a design whose timing cannot be met, the multiplicative
// updaters (flach, tennakoon, livramento, sharma, reimann) can push lambda past
// the float range. A non-finite value would poison the projection and every
// cost term, so the arc keeps its last finite value instead. The caller counts
// these frozen arcs and reports them at debug level 2.
void storeFiniteLambda(LrState& state, sta::EdgeId id, float nl, int& frozen)
{
  if (std::isfinite(nl)) {
    state.lambda[id] = nl;
  } else {
    ++frozen;
  }
}

}  // namespace

// ===========================================================================
// Update formulas (no STA).
// ===========================================================================

float muSeedRaw(const float slack, const float margin, const float exponent)
{
  const float gap = margin - slack;
  return gap > 0.0f ? std::pow(gap, exponent) : 0.0f;
}

float muUpdateFactor(const float slack, const float margin, const float T)
{
  if (T <= 0.0f) {
    return 1.0f;
  }
  return std::clamp(1.0f + (margin - slack) / T, 0.0f, 2.0f);
}

float muRatioFactor(const float arrival, const float required)
{
  // mu_k *= a_k / required_k. A violating endpoint (a > required) raises mu and
  // a met endpoint (a < required) lowers it. The factor tends to 1 as a
  // approaches required, so the rule damps itself. Returns 1 when either input
  // is non-positive (a primary input with zero arrival, or an unconstrained
  // required time).
  if (arrival <= 0.0f || required <= 0.0f) {
    return 1.0f;
  }
  return arrival / required;
}

float muAdditiveStep(const float mu,
                     const float slack,
                     const float rho,
                     const float T)
{
  // mu_k <- max(0, mu_k + rho_k*(a_j - A_0)/T) = max(0, mu + rho*(-slack)/T).
  // A violating endpoint (-slack > 0) raises mu and a met endpoint lowers it.
  // Unlike a multiplicative rule, an additive step can move mu away from 0, so
  // an endpoint that violates again recovers its pressure.
  if (T <= 0.0f) {
    return mu;
  }
  return std::max(0.0f, mu + rho * (-slack) / T);
}

float normSubgradientLambda(const float lambda,
                            const float d,
                            const float a_from,
                            const float a_to,
                            const float alpha,
                            const float floor)
{
  const float denom = std::max(d, floor);
  const float g = std::clamp((d - (a_to - a_from)) / denom, -1.0f, 0.0f);
  return std::max(lambda * (1.0f + alpha * g), floor);
}

float flachKForIter(const int iter,
                    const int max_iter,
                    const float wns,
                    const float T,
                    const float k_init,
                    const float k_small,
                    const float k_final)
{
  // Last ~10% of the iterations: k goes back to <= 1 to remove the remaining
  // violations. Checked first so it wins over the near-feasible case.
  if (max_iter > 0 && iter >= max_iter - std::max(1, max_iter / 10)) {
    return k_final;
  }
  // Near-feasible (WNS within 10% of T), used in place of the paper's "TNS
  // considered small" (Sec. IX): raise k so multipliers fall faster than they
  // rise and leakage is recovered.
  if (T > 0.0f && wns >= -0.1f * T) {
    return k_small;
  }
  return k_init;
}

float flachSlackScaleFactor(const float slack_to, const float T, const float k)
{
  if (T <= 0.0f || k <= 0.0f) {
    return 1.0f;
  }
  if (slack_to <= 0.0f) {
    // Violating arc (arrival >= required): (1 + |slack|/T)^(1/k) >= 1.
    return std::pow(1.0f + (-slack_to) / T, 1.0f / k);
  }
  // Arc with positive slack: (1 + slack/T)^(-k) < 1.
  return std::pow(1.0f + slack_to / T, -k);
}

float chenRho(const int iter, const float c)
{
  // rho_k = c/k with k = max(1, iter); the driver calls updaters for iter >= 1,
  // so the first update uses k = 1 as in the paper.
  return c / static_cast<float>(std::max(1, iter));
}

float chenSubgradientLambda(const float lambda,
                            const float a_from,
                            const float a_to,
                            const float d,
                            const float rho,
                            const float T,
                            const float floor)
{
  // Subgradient of the relaxed arc constraint a_from + d - a_to <= 0. Endpoint
  // constraints enter through mu.
  //
  // The paper steps by rho_k * (a_from + d - a_to) in its own time units. With
  // OpenSTA values in seconds a violation is around 1e-10 while lambda is
  // around 0.1 to 1, so in float `lambda + rho * violation` rounds back to
  // lambda and the update never takes effect for any reasonable c. Here the
  // violation is measured as a fraction of the clock period, which is the same
  // rule with rho_k scaled by 1/T. The paper only requires rho_k -> 0 and
  // sum(rho_k) = infinity, and c/(k*T) satisfies both. It also makes
  // lambda_update_c dimensionless and independent of the library, like the
  // other updaters, which all step on slack/T.
  if (T <= 0.0f) {
    return lambda;
  }
  const float violation = (a_from + d - a_to) / T;
  return std::max(floor, lambda + rho * violation);
}

float tennakoonRatioFactor(const float a_from, const float a_to, const float d)
{
  const float denom = a_to - d;
  if (denom <= 0.0f) {
    return 1.0f;
  }
  return a_from / denom;
}

float livramentoRatioFactor(const float a_from, const float a_to, const float d)
{
  // Alg. 1 line 13: lambda_ji *= (a_j + D_ji)/a_i. This is the subgradient
  // step lambda += rho_k * (a_j + D_ji - a_i) with Livramento's local step size
  // rho_k = lambda_ji/a_i (Eq. 9), which simplifies to the ratio above. Line
  // 14's form for arcs leaving a primary input (D_ji/a_i) is this expression
  // with a_from = 0. Unlike tennakoon's a_from/(a_to - d), it does not send
  // such an arc to zero, where a multiplicative update would leave it for good.
  if (a_to <= 0.0f) {
    return 1.0f;
  }
  return (a_from + d) / a_to;
}

// Floors for the Sharma update. kSharmaCexpFloor keeps the accumulating
// exponent positive, since at 0 it could never change again. kSharmaBaseFloor
// keeps the per-arc base 1 - slack/T positive when slack > T, a case the paper
// does not address.
constexpr float kSharmaCexpFloor = 1e-3f;
constexpr float kSharmaBaseFloor = 1e-3f;

float sharmaCexpStep(const float cexp,
                     const float wns,
                     const float T,
                     const float r,
                     const float k)
{
  if (T <= 0.0f) {
    return cexp;
  }
  const float wpd = T - wns;  // worst path delay = period - worst slack
  if (wpd > r * T) {
    return cexp * (wpd / T);
  }
  // Shrink branch (Fig. 2 line 7): once the design meets the relaxed target
  // r * T, the factor can go negative (with k = 10). Floor cexp at a small
  // positive value rather than 0. Both branches multiply cexp, so at 0 the
  // updater would stop for good after a single iteration with ample slack; a
  // positive floor lets cexp grow again if timing later gets worse.
  return std::max(kSharmaCexpFloor,
                  cexp * (1.0f + k * (wpd - r * T) / (r * T)));
}

float sharmaCritFactor(const float slack_to, const float T, const float cexp)
{
  // Fig. 2 line 10: the base is the slack violation at the arc's sink relative
  // to the clock period, (1 + (a_j - q_j)/T)^cexp = (1 - slack_j/T)^cexp. On a
  // violating arc (slack < 0) the base is 1 + |slack|/T >= 1, so lambda grows
  // (the base is at most 2 for |slack| <= T). On a met arc the base falls
  // linearly as 1 - slack/T and tends to 1 as slack -> 0. The violating side
  // matches flachSlackScaleFactor with exponent cexp instead of 1/k, but the
  // positive-slack sides differ (linear here, a reciprocal power there), so
  // the two are kept separate. For slack > T the base is floored.
  if (T <= 0.0f) {
    return 1.0f;
  }
  const float base = std::max(kSharmaBaseFloor, 1.0f - slack_to / T);
  return std::pow(base, cexp);
}

float sharmaArcSlackExponent(const float arc_slack,
                             const bool power_phase,
                             const GlobalSizingConfig& config)
{
  const bool critical = arc_slack < 0.0f;
  if (power_phase) {
    return critical ? config.arc_slack_k_power_crit
                    : config.arc_slack_k_power_noncrit;
  }
  return critical ? config.arc_slack_k_timing_crit
                  : config.arc_slack_k_timing_noncrit;
}

float sharmaArcSlackFactor(const float arc_slack,
                           const float T,
                           const bool power_phase,
                           const GlobalSizingConfig& config)
{
  // TCAD 2020 writes the base as D/T, D being the worst path delay through
  // the arc, and notes that it equals 1 - s/T when every start point has zero
  // arrival. The base is 1 - s/T here, so it is the base of sharmaCritFactor
  // with the arc's slack in place of its sink's.
  return sharmaCritFactor(
      arc_slack, T, sharmaArcSlackExponent(arc_slack, power_phase, config));
}

float reimannRhoInc(const int iter, const float rho_init)
{
  return rho_init * (1.0f + static_cast<float>(iter));
}

float reimannRhoDec(const int iter, const float rho_init)
{
  return rho_init * (15.0f + static_cast<float>(iter));
}

float reimannKForQuality(const bool in_estimation,
                         const bool have_prev,
                         const float wns_curr,
                         const float wns_prev,
                         const float k_est,
                         const float k_lo,
                         const float k_hi,
                         const float k_neutral)
{
  if (in_estimation) {
    return k_est;
  }
  if (!have_prev) {
    return k_neutral;
  }
  // eps guards against float noise flipping the branch on an unchanged WNS.
  constexpr float eps = 1e-12f;
  if (wns_curr < wns_prev - eps) {
    return k_lo;  // timing degraded -> smaller k grows lambda faster
  }
  if (wns_curr > wns_prev + eps) {
    return k_hi;  // solution improved -> larger k decays lambda faster
  }
  return k_neutral;
}

float reimannScaleFactor(const float slack_curr,
                         const float slack_init,
                         const float dwns,
                         const float T,
                         const float rho_inc,
                         const float rho_dec,
                         const float k)
{
  if (k <= 0.0f) {
    return 1.0f;
  }
  if (slack_curr <= slack_init) {
    // Arc at or below its reference slack (degraded): increase lambda.
    // base = 1 - (S_curr - S_init)/(dWNS * rho_inc) >= 1.
    //
    // In the paper dWNS is the worst-slack degradation caused by the optimizer,
    // which is meaningful when the input meets timing and sizing may degrade
    // it. When sizing starts from a violating design and improves it, dWNS is
    // about 0 every iteration and the factor would blow up. dWNS is therefore
    // floored at 0.1 * T, which keeps the factor bounded, and small when there
    // is no real degradation to react to.
    const float dwns_floor = T > 0.0f ? 0.1f * T : 1e-12f;
    const float denom = std::max(dwns, dwns_floor) * rho_inc;
    if (denom <= 0.0f) {
      return 1.0f;
    }
    return std::pow(1.0f - (slack_curr - slack_init) / denom, 1.0f / k);
  }
  // Arc slack above its reference: decrease lambda.
  if (T <= 0.0f || rho_dec <= 0.0f) {
    return 1.0f;
  }
  return std::pow(1.0f + (slack_curr - slack_init) / (T * rho_dec), -k);
}

// ===========================================================================
// Endpoint (mu) multipliers.
// ===========================================================================

void applyMuPolicy(LrState& state, int iter)
{
  const GlobalSizingConfig& params = *state.config;
  switch (params.mu_policy) {
    case GlobalSizingConfig::MuPolicy::kSeedOnce:
      // The seeder set mu at iteration 0; leave it.
      return;
    case GlobalSizingConfig::MuPolicy::kEndpointLambda:
      // mu is not maintained here. The projection recomputes it from the
      // endpoint's in-arc lambda, which the updater has just updated like any
      // other arc, and anchors to it.
      return;
    case GlobalSizingConfig::MuPolicy::kEndpointRatio: {
      // Multiplicative endpoint rule (Tennakoon and Sechen, Fig. 13, first
      // branch; Livramento et al., Alg. 1 line 12): mu_k *= a_k / required_k.
      // The first projection derived mu_0 from the endpoint's in-arc lambda
      // sum; every later projection anchors the in-arcs to the mu maintained
      // here. required = arrival + slack.
      for (size_t k = 0; k < state.endpoint_vertices.size(); ++k) {
        sta::Vertex* v = state.endpoint_vertices[k];
        const float arrival = arrivalOf(state, v);
        const float slack = slackOf(state, v);
        if (isSentinel(arrival) || isSentinel(slack)) {
          continue;
        }
        const float required = arrival + slack;
        state.mu[k] *= muRatioFactor(arrival, required);
      }
      return;
    }
    case GlobalSizingConfig::MuPolicy::kEndpointAdditive: {
      // Additive endpoint rule (Chen et al., SOLVE_LDP step 3, i = 0 case):
      // mu_k <- max(0, mu_k + rho_k*(-slack)/T) with rho_k = c/k (chenRho), the
      // same schedule and constant chen_subgradient uses for arcs. The first
      // projection derived mu_0 from the endpoint's in-arc lambda sum.
      const float rho = chenRho(iter, params.lambda_update_c);
      const float T = state.T;
      for (size_t k = 0; k < state.endpoint_vertices.size(); ++k) {
        const float slack = slackOf(state, state.endpoint_vertices[k]);
        if (isSentinel(slack)) {
          continue;
        }
        state.mu[k] = muAdditiveStep(state.mu[k], slack, rho, T);
      }
      return;
    }
    case GlobalSizingConfig::MuPolicy::kReseedEachIter: {
      // Re-seed mu from the current endpoint slacks, max(0, margin - slack)^p,
      // normalized to a maximum of 1. A fresh seed avoids the lock-in of a
      // multiplicative update, where an endpoint whose mu has reached 0 can
      // never become active again when its slack gets worse.
      float mu_max_raw = 0.0f;
      const float margin = params.setup_slack_margin;
      const float p = params.mu_exponent;
      for (size_t k = 0; k < state.endpoint_vertices.size(); ++k) {
        const float slack = slackOf(state, state.endpoint_vertices[k]);
        const float mu = muSeedRaw(slack, margin, p);
        state.mu[k] = mu;
        mu_max_raw = std::max(mu_max_raw, mu);
      }
      if (mu_max_raw > 0.0f) {
        for (float& mu : state.mu) {
          mu /= mu_max_raw;
        }
      }
      return;
    }
    case GlobalSizingConfig::MuPolicy::kUpdateAsLambda: {
      // Multiplicative endpoint update analogous to the lambda update, without
      // re-seeding. An endpoint seeded to 0 stays at 0.
      const float margin = params.setup_slack_margin;
      const float T = state.T;
      for (size_t k = 0; k < state.endpoint_vertices.size(); ++k) {
        const float slack = slackOf(state, state.endpoint_vertices[k]);
        if (isSentinel(slack)) {
          continue;
        }
        state.mu[k]
            = std::max(0.0f, state.mu[k] * muUpdateFactor(slack, margin, T));
      }
      return;
    }
  }
}

// ===========================================================================
// Updater implementations.
// ===========================================================================

void NormSubgradientUpdater::update(LrState& state, int iter)
{
  applyMuPolicy(state, iter);

  const GlobalSizingConfig& params = *state.config;
  const float alpha = std::clamp(alpha_, 0.0f, 1.0f);
  const float floor = params.lambda_floor;
  int updated = 0;
  int skipped = 0;
  forEachDataArc(state,
                 [&](sta::Edge* e,
                     sta::EdgeId id,
                     sta::Vertex* from_v,
                     sta::Vertex* to_v) {
                   const float d = state.edgeMaxArcDelay(e);
                   const float a_from = arrivalOf(state, from_v);
                   const float a_to = arrivalOf(state, to_v);
                   if (isSentinel(a_from) || isSentinel(a_to)) {
                     ++skipped;
                     return;
                   }
                   state.lambda[id] = normSubgradientLambda(
                       state.lambda[id], d, a_from, a_to, alpha, floor);
                   ++updated;
                 });

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR norm_subgradient: {} arcs stepped ({} unconstrained skipped), "
             "alpha={:.3g}",
             updated,
             skipped,
             alpha);
}

void FlachSlackScalingUpdater::update(LrState& state, int iter)
{
  applyMuPolicy(state, iter);

  const GlobalSizingConfig& params = *state.config;
  const float T = state.T;
  const float wns = sta::delayAsFloat(state.sta->worstSlack(state.max));
  const float k = flachKForIter(iter,
                                params.max_iterations,
                                wns,
                                T,
                                params.flach_k_init,
                                params.flach_k_tns_small,
                                params.flach_k_final);
  last_k_ = k;
  const float floor = params.lambda_floor;
  int frozen = 0;
  forEachDataArc(
      state, [&](sta::Edge*, sta::EdgeId id, sta::Vertex*, sta::Vertex* to_v) {
        const float slack_to = slackOf(state, to_v);
        if (isSentinel(slack_to)) {
          return;
        }
        const float nl = std::max(
            floor, state.lambda[id] * flachSlackScaleFactor(slack_to, T, k));
        storeFiniteLambda(state, id, nl, frozen);
      });

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR flach_slack_scaling: k={:.3g} T={:.3g} wns={:.3g} frozen={}",
             k,
             T,
             wns,
             frozen);
}

void ChenSubgradientUpdater::update(LrState& state, int iter)
{
  applyMuPolicy(state, iter);

  const GlobalSizingConfig& params = *state.config;
  const float rho = chenRho(iter, params.lambda_update_c);
  last_rho_ = rho;
  const float floor = params.lambda_floor;
  const float T = state.T;
  forEachDataArc(state,
                 [&](sta::Edge* e,
                     sta::EdgeId id,
                     sta::Vertex* from_v,
                     sta::Vertex* to_v) {
                   // Consistent (a_from, d, a_to) read, so that mixed
                   // rise/fall values cannot make the subgradient
                   // a_from + d - a_to positive on a critical arc and slowly
                   // inflate lambda (see LrState::consistentArcRead).
                   const LrState::ConsistentArcRead r
                       = state.consistentArcRead(e, from_v, to_v);
                   if (isSentinel(r.a_from) || isSentinel(r.a_to)) {
                     return;
                   }
                   state.lambda[id] = chenSubgradientLambda(
                       state.lambda[id], r.a_from, r.a_to, r.d, rho, T, floor);
                 });

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR chen_subgradient: rho={:.3g} T={:.3g}",
             rho,
             T);
}

void TennakoonRatioUpdater::update(LrState& state, int iter)
{
  applyMuPolicy(state, iter);

  const GlobalSizingConfig& params = *state.config;
  const float floor = params.lambda_floor;
  int frozen = 0;
  forEachDataArc(
      state,
      [&](sta::Edge* e,
          sta::EdgeId id,
          sta::Vertex* from_v,
          sta::Vertex* to_v) {
        // Consistent read (see LrState::consistentArcRead): a_from/(a_to - d)
        // stays <= 1 on the critical arc. Mixed rise/fall values could push it
        // above 1 and raise lambda for no reason.
        const LrState::ConsistentArcRead r
            = state.consistentArcRead(e, from_v, to_v);
        if (isSentinel(r.a_from) || isSentinel(r.a_to)) {
          return;
        }
        const float nl = std::max(
            floor,
            state.lambda[id] * tennakoonRatioFactor(r.a_from, r.a_to, r.d));
        storeFiniteLambda(state, id, nl, frozen);
      });

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR tennakoon_ratio update: frozen={}",
             frozen);
}

void LivramentoRatioUpdater::update(LrState& state, int iter)
{
  applyMuPolicy(state, iter);

  const GlobalSizingConfig& params = *state.config;
  const float floor = params.lambda_floor;
  int frozen = 0;
  forEachDataArc(
      state,
      [&](sta::Edge* e,
          sta::EdgeId id,
          sta::Vertex* from_v,
          sta::Vertex* to_v) {
        // Consistent read (see LrState::consistentArcRead): (a_from + d)/a_to
        // stays <= 1 on the critical arc. Mixed rise/fall values could push it
        // above 1 and raise lambda for no reason.
        const LrState::ConsistentArcRead r
            = state.consistentArcRead(e, from_v, to_v);
        if (isSentinel(r.a_from) || isSentinel(r.a_to)) {
          return;
        }
        const float nl = std::max(
            floor,
            state.lambda[id] * livramentoRatioFactor(r.a_from, r.a_to, r.d));
        storeFiniteLambda(state, id, nl, frozen);
      });

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR livramento_ratio update: frozen={}",
             frozen);
}

void SharmaCexpUpdater::update(LrState& state, int iter)
{
  applyMuPolicy(state, iter);

  const GlobalSizingConfig& params = *state.config;
  const float T = state.T;
  const float wns = sta::delayAsFloat(state.sta->worstSlack(state.max));
  cexp_ = sharmaCexpStep(cexp_, wns, T, params.sharma_r, params.sharma_k);
  const float floor = params.lambda_floor;
  int frozen = 0;
  forEachDataArc(
      state, [&](sta::Edge*, sta::EdgeId id, sta::Vertex*, sta::Vertex* to_v) {
        const float slack_to = slackOf(state, to_v);
        if (isSentinel(slack_to)) {
          return;
        }
        // Uses only the sink slack and T. Like flach, it reads no arrival
        // times, so it is not affected by mixed rise/fall values.
        const float nl = std::max(
            floor, state.lambda[id] * sharmaCritFactor(slack_to, T, cexp_));
        storeFiniteLambda(state, id, nl, frozen);
      });

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR sharma_cexp: cexp={:.3g} T={:.3g} wns={:.3g} frozen={}",
             cexp_,
             T,
             wns,
             frozen);
}

void SharmaArcSlackUpdater::update(LrState& state, int iter)
{
  applyMuPolicy(state, iter);

  const GlobalSizingConfig& params = *state.config;
  const float T = state.T;
  // The phase of the sweep this update follows; see
  // LrState::prev_sweep_power_phase.
  const bool power_phase = state.prev_sweep_power_phase;
  const float floor = params.lambda_floor;
  int frozen = 0;
  forEachDataArc(
      state,
      [&](sta::Edge* e,
          sta::EdgeId id,
          sta::Vertex* from_v,
          sta::Vertex* to_v) {
        const float arc_slack = state.arcSlack(e, from_v, to_v);
        if (isSentinel(arc_slack)) {
          return;
        }
        const float nl = std::max(
            floor,
            state.lambda[id]
                * sharmaArcSlackFactor(arc_slack, T, power_phase, params));
        storeFiniteLambda(state, id, nl, frozen);
      });

  const float k_crit = power_phase ? params.arc_slack_k_power_crit
                                   : params.arc_slack_k_timing_crit;
  const float k_noncrit = power_phase ? params.arc_slack_k_power_noncrit
                                      : params.arc_slack_k_timing_noncrit;
  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR sharma_arc_slack: iter={} phase={} k={:.3g}/{:.3g} T={:.3g} "
             "frozen={}",
             iter,
             power_phase ? "power" : "timing",
             k_crit,
             k_noncrit,
             T,
             frozen);
}

void ReimannDwnsUpdater::update(LrState& state, int iter)
{
  applyMuPolicy(state, iter);

  const GlobalSizingConfig& params = *state.config;
  const float T = state.T;
  const float wns_curr = sta::delayAsFloat(state.sta->worstSlack(state.max));
  // Reference slack. The paper compares each arc with its initial slack S_init
  // (kSInit). When sizing starts from a violating design and improves it, arc
  // slacks stay above S_init and the increase branch never acts. kSlackTarget
  // departs from the paper and uses the slack target (setup_slack_margin)
  // instead, so violating arcs do get more weight. See ReimannSetpoint in
  // GlobalSizingConfig.hh.
  const bool slack_target
      = params.reimann_setpoint
        == GlobalSizingConfig::ReimannSetpoint::kSlackTarget;
  const float margin = params.setup_slack_margin;
  // dWNS: worst-slack degradation since the input timing (kSInit), or the
  // current violation below the target (kSlackTarget).
  const float dwns = slack_target ? std::max(0.0f, margin - wns_curr)
                                  : std::max(0.0f, state.wns_init - wns_curr);
  const float rho_inc = reimannRhoInc(iter, params.reimann_rho_init);
  const float rho_dec = reimannRhoDec(iter, params.reimann_rho_init);
  last_rho_inc_ = rho_inc;
  // Quality-driven k schedule (Eq. 7): compare this iteration's WNS with the
  // previous update's. During estimation k is fixed at k_est.
  const float k = reimannKForQuality(in_estimation_,
                                     have_prev_,
                                     wns_curr,
                                     wns_prev_,
                                     params.reimann_k_est,
                                     params.reimann_k_lo,
                                     params.reimann_k_hi,
                                     params.reimann_k);
  if (!in_estimation_) {
    wns_prev_ = wns_curr;
    have_prev_ = true;
  }
  const float floor = params.lambda_floor;
  sta::Graph* graph = state.graph;
  int frozen = 0;
  forEachDataArc(
      state, [&](sta::Edge*, sta::EdgeId id, sta::Vertex*, sta::Vertex* to_v) {
        const float slack_curr = slackOf(state, to_v);
        if (isSentinel(slack_curr)) {
          return;
        }
        float slack_ref;
        if (slack_target) {
          slack_ref = margin;
        } else {
          const size_t vid = static_cast<size_t>(graph->id(to_v));
          slack_ref = vid < state.slack_init.size() ? state.slack_init[vid]
                                                    : slack_curr;
          if (isSentinel(slack_ref)) {
            return;
          }
        }
        const float nl = std::max(
            floor,
            state.lambda[id]
                * reimannScaleFactor(
                    slack_curr, slack_ref, dwns, T, rho_inc, rho_dec, k));
        storeFiniteLambda(state, id, nl, frozen);
      });

  debugPrint(state.logger,
             RSZ,
             "global_sizing",
             2,
             "LR reimann_dwns: rho_inc={:.3g} rho_dec={:.3g} dwns={:.3g} "
             "k={:.3g} est={} frozen={}",
             rho_inc,
             rho_dec,
             dwns,
             k,
             in_estimation_,
             frozen);
}

std::unique_ptr<LambdaUpdater> makeLambdaUpdater(
    const GlobalSizingConfig& config)
{
  switch (config.lambda_update) {
    case GlobalSizingConfig::LambdaUpdate::kNormSubgradient:
      return std::make_unique<NormSubgradientUpdater>(config.beta);
    case GlobalSizingConfig::LambdaUpdate::kFlachSlackScaling:
      return std::make_unique<FlachSlackScalingUpdater>();
    case GlobalSizingConfig::LambdaUpdate::kChenSubgradient:
      return std::make_unique<ChenSubgradientUpdater>();
    case GlobalSizingConfig::LambdaUpdate::kTennakoonRatio:
      return std::make_unique<TennakoonRatioUpdater>();
    case GlobalSizingConfig::LambdaUpdate::kLivramentoRatio:
      return std::make_unique<LivramentoRatioUpdater>();
    case GlobalSizingConfig::LambdaUpdate::kSharmaCexp:
      return std::make_unique<SharmaCexpUpdater>();
    case GlobalSizingConfig::LambdaUpdate::kReimannDwns:
      return std::make_unique<ReimannDwnsUpdater>();
    case GlobalSizingConfig::LambdaUpdate::kSharmaArcSlack:
      return std::make_unique<SharmaArcSlackUpdater>();
  }
  return std::make_unique<NormSubgradientUpdater>(config.beta);
}

}  // namespace rsz
