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

// Worst rise/fall arrival at v under the policy scenes / max corner.
float arrivalOf(LrState& s, sta::Vertex* v)
{
  return sta::delayAsFloat(
      s.sta->arrival(v, sta::RiseFallBoth::riseFall(), s.sta->scenes(), s.max));
}

// Worst slack at v under the max corner.
float slackOf(LrState& s, sta::Vertex* v)
{
  return sta::delayAsFloat(s.sta->slack(v, s.max));
}

bool isSentinel(float x)
{
  return std::fabs(x) >= LrState::kSlackSentinel;
}

// Visit every in-range data arc once, in the same (vertex, out-edge) order the
// baseline used, invoking fn(edge, id, from_vertex, to_vertex). Centralizes the
// data-arc filtering so every updater shares one traversal.
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

// Store the stepped multiplier unless a multiplicative step overflowed it to a
// non-finite value (C3 §5.2, sharma audit). On a design global sizing cannot
// close, a multiplicative/scaling updater (flach / tennakoon / livramento /
// sharma / reimann) can drive lambda past float32 max. Storing inf/NaN would
// poison the projection and every downstream cost term, so the arc is instead
// FROZEN at its last finite value. This is intended, bounded behavior — the
// last line of defense on an un-closable design, where the sweep makes no move
// once WNS has hit its floor, so the returned QoR is preserved by the best
// tracker — NOT a silent error: the caller counts `frozen` and reports it at
// debug level 2.
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
// Pure formula cores (unit-tested; no STA).
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
  // endpoint_ratio (Tennakoon Fig. 13 branch 1 = Livramento Alg. 1 L12):
  // mu_k *= a_k / required_k. Violating endpoint (a > required) grows mu;
  // met endpoint (a < required) damps it; a -> required damps the factor to 1
  // (the papers' own gain control). Guarded to neutral 1 when either input is
  // non-positive (a PI with zero arrival, an unconstrained required).
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
  // endpoint_additive (Chen SOLVE_LDP step 3 i=0 branch):
  // mu_k <- max(0, mu_k + rho_k*(a_j - A_0)/T) = max(0, mu + rho*(-slack)/T).
  // Violating endpoint (-slack > 0) grows mu; met endpoint decays it (0 is not
  // absorbing under an additive step, so a re-violating endpoint recovers).
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
  // Endgame (last ~10% of iterations): k back to <=1 to squeeze out residual
  // violations. Checked first so it wins over the near-feasible branch.
  if (max_iter > 0 && iter >= max_iter - std::max(1, max_iter / 10)) {
    return k_final;
  }
  // Near-feasible (WNS within 10% of T, the "TNS considered small" proxy of
  // Flach §5): raise k so leakage drops faster than it is added back.
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
    // Violating arc (arrival >= required): (1 + |slack|/T)^{+1/k} >= 1.
    return std::pow(1.0f + (-slack_to) / T, 1.0f / k);
  }
  // Non-violating arc (positive slack): (1 + slack/T)^{-k} < 1.
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
  // Subgradient of the relaxed arc constraint a_from + d - a_to (<= 0 at an
  // STA-consistent point; the endpoint pressure enters through mu).
  //
  // T-normalization (adaptation, and a necessary one): the paper's step is
  // rho_k*(a_from + d - a_to) in the contest's timing units. Applied to OpenSTA
  // quantities that is unrepresentable - a violation is SI seconds (~1e-10)
  // while the projected lambda is O(0.1-1), so `lambda + rho*violation` rounds
  // straight back to lambda in float32 (the float32 step at lambda is
  // lambda*1.2e-7) and the update silently never fires at any sane c. Stepping
  // on the violation as a FRACTION OF THE CLOCK PERIOD is the same rule with
  // rho_k rescaled by 1/T, which Chen's own conditions permit: the paper pins
  // only rho_k -> 0 and sum(rho_k) = inf, never a value or a unit (it says
  // "rho_k := arbitrary" - see chen_et_al.md), and c/(k*T) satisfies both. It
  // also makes lambda_update_c dimensionless and library-portable, matching
  // every other updater here, all of which step on a dimensionless slack/T
  // ratio.
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
  // Alg. 1 line 13: lambda_ji *= (a_j + D_ji)/a_i. This is Livramento's local
  // step size rho_k = lambda_ji/a_i (Eq. 9) folded into the subgradient step
  // lambda += rho_k*(a_j + D_ji - a_i), which telescopes to the ratio above.
  // Line 14's PI form (D_ji/a_i) is this expression at a_from = 0, so no
  // separate branch is needed - and unlike tennakoon's a_from/(a_to - d), it
  // does not collapse a PI-sourced arc to zero (zero is absorbing under a
  // multiplicative update).
  if (a_to <= 0.0f) {
    return 1.0f;
  }
  return (a_from + d) / a_to;
}

// Sharma criticality-base floors, documented at the constants (sharma audit
// §2 items 1-2). kSharmaCexpFloor keeps the accumulating exponent positive
// rather than the absorbing 0 the hardening clamp produced. kSharmaBaseFloor
// keeps the per-arc base positive when slack > T, the §13.3 edge the paper
// leaves unstated (the base 1 - slack/T crosses zero there).
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
  // Shrink branch (Fig. 2 line 7): when the design is under the relaxed target
  // the factor can go negative (k = 10). Floor cexp at a small POSITIVE value,
  // not 0: 0 is absorbing (both branches multiply cexp, so once 0 the updater
  // freezes permanently - one over-met iteration kills it), whereas a positive
  // floor lets cexp regrow through the growth branch if timing later regresses.
  return std::max(kSharmaCexpFloor,
                  cexp * (1.0f + k * (wpd - r * T) / (r * T)));
}

float sharmaCritFactor(const float slack_to, const float T, const float cexp)
{
  // Fig. 2 line 10, as corrected in sharma_et_al.md: the criticality base is
  // the sink-node slack violation over the CLOCK PERIOD,
  // (1 + (a_j - q_j)/T)^cexp = (1 - slack_j/T)^cexp - NOT the mistranscribed
  // (a_j/q_j)^cexp, which is unbounded as the required time q_j -> 0+ and was
  // the float32-saturation mechanism behind the B1 blocker row. On a violating
  // arc (slack < 0) the base is 1 + |slack|/T >= 1 (dual ascent, bounded by
  // 1 + |slack|/T <= 2 for |slack| <= T); on a met arc it decays linearly as
  // 1 - slack/T and self-damps to 1 as slack -> 0. This is
  // flachSlackScaleFactor's violating branch with exponent cexp instead of
  // 1/k, but the positive-slack arms differ (linear decay here, reciprocal
  // there), so the two are not merged. slack > T floors the base positive.
  if (T <= 0.0f) {
    return 1.0f;
  }
  const float base = std::max(kSharmaBaseFloor, 1.0f - slack_to / T);
  return std::pow(base, cexp);
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
    // Arc at or below its initial slack (degraded) -> increase lambda.
    // base = 1 - (S_curr - S_init)/(dWNS*rho_inc) = 1 + degradation/... >= 1.
    //
    // Adaptation note (§1): Reimann's dWNS is the global worst-slack
    // *degradation* the optimizer caused, meaningful only in the paper's
    // start-feasible / degrade-under-LR regime. In OpenROAD's usual
    // improve-from-violating regime dWNS is ~0 every iteration, so a raw floor
    // would collapse the denominator and blow the increase factor up. Floor
    // dWNS to a fraction of T so the factor stays bounded (and small when there
    // is no real global degradation to react to).
    const float dwns_floor = T > 0.0f ? 0.1f * T : 1e-12f;
    const float denom = std::max(dwns, dwns_floor) * rho_inc;
    if (denom <= 0.0f) {
      return 1.0f;
    }
    return std::pow(1.0f - (slack_curr - slack_init) / denom, 1.0f / k);
  }
  // Arc slack improved beyond initial -> decrease lambda.
  if (T <= 0.0f || rho_dec <= 0.0f) {
    return 1.0f;
  }
  return std::pow(1.0f + (slack_curr - slack_init) / (T * rho_dec), -k);
}

// ===========================================================================
// E4 - endpoint (mu) multiplier policy.
// ===========================================================================

void applyMuPolicy(LrState& state, int iter)
{
  const GlobalSizingConfig& params = *state.config;
  switch (params.mu_policy) {
    case GlobalSizingConfig::MuPolicy::kSeedOnce:
      // Seeder set mu at iter 0; leave it untouched.
      return;
    case GlobalSizingConfig::MuPolicy::kEndpointLambda:
      // mu is derived, not maintained: the projection recomputes it from the
      // endpoint's in-arc lambda (which this updater has just slack-scaled like
      // any other arc) and anchors to it. Nothing to do here.
      return;
    case GlobalSizingConfig::MuPolicy::kEndpointRatio: {
      // Branch 1 as the papers' multiplicative ratio (Tennakoon Fig. 13 =
      // Livramento Alg. 1 L12): mu_k *= a_k / required_k. mu_0 was derived from
      // the endpoint's in-arc lambda sum by the first projection; the
      // projection anchors the in-arcs to the mu maintained here every
      // subsequent iteration. required = arrival + slack.
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
      // Branch 1 as Chen's additive step: mu_k <- max(0, mu_k +
      // rho_k*(-slack)/T) with rho_k = c/k (chenRho), the same schedule and
      // constant the chen_subgradient branch-2 update uses. mu_0 was derived
      // from the endpoint-lambda seed by the first projection.
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
      // WNS-biased crit^p re-seed from the current endpoint slacks, normalized
      // to max 1. Fresh seed (rather than a multiplicative mu update) avoids
      // the lock-in where an endpoint whose mu reached the floor can never
      // re-activate when its slack regresses.
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
      // Multiplicative endpoint update analogous to lambda (no re-seed): the
      // reading most papers implicitly use. Endpoints seeded to 0 stay 0 (the
      // documented semantic difference this axis ablates).
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
                   // Consistent (a_from, d, a_to) read so the subgradient
                   // a_from + d - a_to never goes spuriously positive on a
                   // critical arc (C3 item 1; the λ-creep source).
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
        // Consistent read (C3 item 1): a_from/(a_to - d) stays <= 1 on the
        // critical arc instead of the collapse's spurious > 1 (phantom ascent).
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
        // Consistent read (C3 item 1): (a_from + d)/a_to stays <= 1 on the
        // critical arc instead of the collapse's spurious > 1 (phantom ascent).
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
        // Slack-only re-base (sharma audit §2 item 1): consume slack_to and T,
        // drop the arrival read and the q = a + slack reconstruction (which
        // also mixed rise/fall transitions) - the updater joins flach's
        // exemption from the collapse artifact.
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

void ReimannDwnsUpdater::update(LrState& state, int iter)
{
  applyMuPolicy(state, iter);

  const GlobalSizingConfig& params = *state.config;
  const float T = state.T;
  const float wns_curr = sta::delayAsFloat(state.sta->worstSlack(state.max));
  // C1 setpoint knob: the faithful servo references each arc's frozen initial
  // slack S_init (kSInit); the disclosed non-paper adaptation references the
  // slack target (kSlackTarget, S_init := margin) so the increase branch fires
  // on violating arcs. See ReimannSetpoint in GlobalSizingConfig.hh.
  const bool slack_target
      = params.reimann_setpoint
        == GlobalSizingConfig::ReimannSetpoint::kSlackTarget;
  const float margin = params.setup_slack_margin;
  // ΔWNS: worst-slack degradation from the input timing (kSInit), or the live
  // violation depth below the target (kSlackTarget).
  const float dwns = slack_target ? std::max(0.0f, margin - wns_curr)
                                  : std::max(0.0f, state.wns_init - wns_curr);
  const float rho_inc = reimannRhoInc(iter, params.reimann_rho_init);
  const float rho_dec = reimannRhoDec(iter, params.reimann_rho_init);
  last_rho_inc_ = rho_inc;
  // Quality-driven k-schedule (Eq. 7): compare this iteration's WNS to the last
  // update's; during estimation k is pinned to k_est.
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
  }
  return std::make_unique<NormSubgradientUpdater>(config.beta);
}

}  // namespace rsz
