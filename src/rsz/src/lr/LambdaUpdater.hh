// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>

#include "LrState.hh"

namespace rsz {

struct GlobalSizingConfig;

// Per-iteration update of the Lagrange multipliers. update() rewrites
// state.lambda in place from the current timing and leaves the endpoint
// multipliers (mu) to applyMuPolicy(). `iter` is 0-based; the driver calls
// update() only for iter >= 1. The update formulas are free functions below so
// they can be unit-tested against hand-computed values without running STA.
class LambdaUpdater
{
 public:
  virtual ~LambdaUpdater() = default;
  virtual void update(LrState& state, int iter) = 0;
  // Called when the driver rejects a pass. The base does nothing; updaters with
  // an adaptive step override it.
  virtual void onPassRejected() {}
  // Set to true by the driver during the dry-run estimation sweeps (Reimann et
  // al., ISPD 2016, Alg. 2, first loop), for updaters whose schedule treats
  // estimation differently (reimann_dwns uses its estimation exponent there).
  // The base does nothing.
  virtual void setEstimationPhase(bool /* on */) {}
  // Current step size, for the level-1 debug trace. -1 when the updater has no
  // scalar step.
  virtual float currentStep() const { return -1.0f; }
};

// === Endpoint (mu) multipliers ============================================
// Updates state.mu according to config.mu_policy: re-seed it from the endpoint
// slacks every iteration, keep the initial seed, or apply a per-endpoint update
// rule. Every updater calls this, so the lambda rule and the mu rule can be
// chosen independently. Under endpoint_lambda it does nothing: the projection
// derives mu from the endpoint's in-arc lambda.
void applyMuPolicy(LrState& state, int iter);

// === Update formulas (no STA; unit-tested) =================================
// Each returns the updated lambda (or a multiplicative factor or step) for one
// arc or endpoint, given the timing values the updater reads from STA.

// Raw mu seed before normalization: max(0, margin - slack)^p.
float muSeedRaw(float slack, float margin, float exponent);
// update_as_lambda factor for one endpoint: 1 + (margin - slack)/T, clamped to
// [0, 2]. 1 when T <= 0 (no clock).
float muUpdateFactor(float slack, float margin, float T);
// endpoint_ratio factor for one endpoint: a_k / required_k (Tennakoon and
// Sechen, ICCAD 2002, Fig. 13, first branch; the same rule as Livramento et
// al., DATE 2013, Alg. 1 line 12). 1 when either input is non-positive.
float muRatioFactor(float arrival, float required);
// endpoint_additive step for one endpoint: max(0, mu + rho*(-slack)/T) (Chen
// et al., ICCAD 1998, SOLVE_LDP step 3, the i = 0 case). mu is unchanged when
// T <= 0.
float muAdditiveStep(float mu, float slack, float rho, float T);

// norm_subgradient (the default):
//   g = clamp((d - (a_to - a_from)) / max(d, floor), -1, 0)
//   lambda <- max(floor, lambda * (1 + alpha * g))
// g is minus the arc's local slack (a_to - a_from - d) relative to its delay,
// so lambda keeps its value on an arc that sets its sink's arrival and shrinks
// on an arc with local slack.
float normSubgradientLambda(float lambda,
                            float d,
                            float a_from,
                            float a_to,
                            float alpha,
                            float floor);

// Flach et al., TCAD 2014: the exponent k starts at k_init, is raised to
// k_small once the design is close to meeting timing, and drops to k_final
// near the end of the run (the paper uses 1, 4 and a value <= 1).
float flachKForIter(int iter,
                    int max_iter,
                    float wns,
                    float T,
                    float k_init,
                    float k_small,
                    float k_final);
// Flach et al., TCAD 2014, Alg. 2: factor applied to an arc's lambda. A
// violating arc (slack <= 0) is scaled by (1 + |slack|/T)^(1/k), an arc with
// positive slack by (1 + slack/T)^(-k). With k > 1 multipliers therefore fall
// faster than they rise. 1 when T <= 0.
float flachSlackScaleFactor(float slack_to, float T, float k);

// Chen et al., ICCAD 1998: step size rho_k = c/k with k = max(1, iter), and
// the additive update lambda += rho * (a_from + d - a_to)/T, floored. The
// violation is measured as a fraction of the clock period rather than in
// seconds; see the definition for why. lambda is unchanged when T <= 0.
float chenRho(int iter, float c);
float chenSubgradientLambda(float lambda,
                            float a_from,
                            float a_to,
                            float d,
                            float rho,
                            float T,
                            float floor);

// Tennakoon and Sechen, ICCAD 2002: multiplicative factor a_from/(a_to - d),
// with no tuning constant. 1 when a_to - d <= 0.
float tennakoonRatioFactor(float a_from, float a_to, float d);

// Livramento et al., DATE 2013, Alg. 1 line 13: multiplicative factor
// (a_from + d)/a_to, with no tuning constant. 1 when a_to <= 0. This and the
// Tennakoon factor are both 1 on an arc that sets its sink's arrival
// (a_to = a_from + d) and differ on every other arc. Line 14's form for arcs
// leaving a primary input (d/a_to) is this expression with a_from = 0.
float livramentoRatioFactor(float a_from, float a_to, float d);

// Sharma et al., ICCAD 2015, Fig. 2: the exponent cexp carries over between
// iterations and is updated from the worst path delay WPD = T - WNS. It is
// floored at a small positive value so that one iteration with ample slack
// cannot freeze it at 0. The per-arc factor is (1 - slack_to/T)^cexp, the
// slack at the arc's sink relative to the clock period (Fig. 2 line 10); its
// base is floored at a small positive value when slack > T. 1 when T <= 0.
float sharmaCexpStep(float cexp, float wns, float T, float r, float k);
float sharmaCritFactor(float slack_to, float T, float cexp);

// Sharma et al., TCAD 2020, Alg. 3: the per-arc factor (1 - s/T)^K, with s
// the arc's own slack and the base floored as in sharmaCritFactor. An arc is
// critical iff s < 0 (Sec. IV-C-2). K is config.arc_slack_k_timing_crit or
// _noncrit in the timing phase and config.arc_slack_k_power_crit or _noncrit
// in the power phase. The factor is 1 when T <= 0.
float sharmaArcSlackExponent(float arc_slack,
                             bool power_phase,
                             const GlobalSizingConfig& config);
float sharmaArcSlackFactor(float arc_slack,
                           float T,
                           bool power_phase,
                           const GlobalSizingConfig& config);

// Reimann et al., ISPD 2016, Alg. 3: the step-size schedules rho_inc and
// rho_dec, and an asymmetric factor that compares each arc's current slack
// with a reference slack (in the paper, the arc's initial slack). The increase
// side is normalized by dWNS and the decrease side by T.
float reimannRhoInc(int iter, float rho_init);
float reimannRhoDec(int iter, float rho_init);
// Reimann et al., Eq. 7: k = k_est during estimation; k_lo (< 1) if WNS got
// worse since the previous iteration; k_hi (> 1) if it got better; k_neutral
// otherwise, including the first call (have_prev = false).
float reimannKForQuality(bool in_estimation,
                         bool have_prev,
                         float wns_curr,
                         float wns_prev,
                         float k_est,
                         float k_lo,
                         float k_hi,
                         float k_neutral);
float reimannScaleFactor(float slack_curr,
                         float slack_init,
                         float dwns,
                         float T,
                         float rho_inc,
                         float rho_dec,
                         float k);

// === Updater implementations ================================================

// Multiplicative, normalized subgradient update (the default). alpha is halved
// each time the driver rejects a pass.
class NormSubgradientUpdater : public LambdaUpdater
{
 public:
  explicit NormSubgradientUpdater(float alpha0) : alpha_(alpha0) {}
  void update(LrState& state, int iter) override;
  void onPassRejected() override { alpha_ *= 0.5f; }
  float currentStep() const override { return alpha_; }

 private:
  // Step size. Starts at config.beta, is halved on each rejected pass, and is
  // clamped to [0, 1] when used.
  float alpha_;
};

// Flach et al., TCAD 2014, Alg. 2: asymmetric multiplicative slack scaling.
class FlachSlackScalingUpdater : public LambdaUpdater
{
 public:
  void update(LrState& state, int iter) override;
  float currentStep() const override { return last_k_; }

 private:
  float last_k_ = 0.0f;
};

// Chen et al., ICCAD 1998: additive subgradient update with a diminishing
// step size.
class ChenSubgradientUpdater : public LambdaUpdater
{
 public:
  void update(LrState& state, int iter) override;
  float currentStep() const override { return last_rho_; }

 private:
  float last_rho_ = 0.0f;
};

// Tennakoon and Sechen, ICCAD 2002 (Forge): arrival-ratio update with no
// tuning constant.
class TennakoonRatioUpdater : public LambdaUpdater
{
 public:
  void update(LrState& state, int iter) override;
};

// Livramento et al., DATE 2013, Alg. 1 line 13: local multiplicative update.
class LivramentoRatioUpdater : public LambdaUpdater
{
 public:
  void update(LrState& state, int iter) override;
};

// Sharma et al., ICCAD 2015: criticality update whose exponent accumulates
// across iterations.
class SharmaCexpUpdater : public LambdaUpdater
{
 public:
  void update(LrState& state, int iter) override;
  float currentStep() const override { return cexp_; }

 private:
  // Kept across iterations; Fig. 2 line 3 initializes it only once.
  float cexp_ = 1.0f;
};

// Sharma et al., TCAD 2020, Alg. 3: arc-slack update whose exponents depend on
// the arc's criticality and the run's phase (LrState::prev_sweep_power_phase).
class SharmaArcSlackUpdater : public LambdaUpdater
{
 public:
  void update(LrState& state, int iter) override;
};

// Reimann et al., ISPD 2016, Alg. 3: dWNS-normalized update toward a reference
// slack, with the quality-driven k schedule of Eq. 7.
class ReimannDwnsUpdater : public LambdaUpdater
{
 public:
  void update(LrState& state, int iter) override;
  void setEstimationPhase(bool on) override { in_estimation_ = on; }
  float currentStep() const override { return last_rho_inc_; }

 private:
  float last_rho_inc_ = 0.0f;
  // WNS at the previous update() call, and whether there was one. Not updated
  // during estimation, where each dry-run iteration restores WNS.
  float wns_prev_ = 0.0f;
  bool have_prev_ = false;
  bool in_estimation_ = false;
};

std::unique_ptr<LambdaUpdater> makeLambdaUpdater(
    const GlobalSizingConfig& config);

}  // namespace rsz
