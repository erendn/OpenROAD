// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>

#include "LrState.hh"

namespace rsz {

struct GlobalSizingConfig;

// E1/E2 axis - per-iteration lambda update. update() mutates state.lambda in
// place from the current timing picture and delegates the endpoint (mu)
// multipliers to applyMuPolicy (E4); `iter` is 0-based (the driver calls this
// only for iter >= 1). The dual-subgradient / slack-scaling arithmetic lives in
// the pure free functions below so it can be unit-tested against hand-computed
// values without a full STA.
class LambdaUpdater
{
 public:
  virtual ~LambdaUpdater() = default;
  virtual void update(LrState& state, int iter) = 0;
  // Pass-rejection feedback from the driver. The base is a no-op; updaters with
  // an adaptive step override it.
  virtual void onPassRejected() {}
  // Estimation-loop phase toggle (Reimann Alg. 2 loop 1). The driver sets this
  // true around the dry-run estimation sweeps so an updater whose schedule
  // distinguishes estimation (reimann_dwns: k=5) can react. Base is a no-op.
  virtual void setEstimationPhase(bool /* on */) {}
  // Current step magnitude for the level-1 diagnostic line. -1 == not
  // applicable (updater has no scalar step).
  virtual float currentStep() const { return -1.0f; }
};

// === E4 - endpoint (mu) multiplier policy ===================================
// Maintains state.mu per state.config->mu_policy: reseed each iteration (the
// current WNS-biased crit^p seed), seed-once (no-op after the seeder), or a
// multiplicative update analogous to lambda. Shared by every updater so the
// lambda axis and the mu axis stay orthogonal. endpoint_lambda is a no-op here:
// its mu is derived by the projection from the endpoint's in-arc lambda.
void applyMuPolicy(LrState& state, int iter);

// === Pure formula cores (unit-tested; no STA) ===============================
// Each returns the post-update lambda (or a multiplicative factor / step) for
// one arc or endpoint, given the timing scalars the updater reads from STA.

// mu re-seed raw contribution before normalization: max(0, margin - slack)^p.
float muSeedRaw(float slack, float margin, float exponent);
// update_as_lambda per-endpoint factor: (1 + (margin - slack)/T), clamped to a
// bounded positive range; 1 when T <= 0 (no clock).
float muUpdateFactor(float slack, float margin, float T);
// endpoint_ratio per-endpoint factor: a_k / required_k (Tennakoon Fig. 13
// branch 1 = Livramento Alg. 1 L12); 1 when either input is non-positive.
float muRatioFactor(float arrival, float required);
// endpoint_additive per-endpoint step: max(0, mu + rho*(-slack)/T) (Chen
// SOLVE_LDP step 3 i=0 branch); mu unchanged when T <= 0.
float muAdditiveStep(float mu, float slack, float rho, float T);

// norm_subgradient (baseline): g = clamp((d-(a_to-a_from))/max(d,floor),-1,0),
// lambda <- max(floor, lambda*(1 + alpha*g)).
float normSubgradientLambda(float lambda,
                            float d,
                            float a_from,
                            float a_to,
                            float alpha,
                            float floor);

// flach: k-schedule 1 -> 4 -> <=1 driven by iteration and near-feasibility.
float flachKForIter(int iter,
                    int max_iter,
                    float wns,
                    float T,
                    float k_init,
                    float k_small,
                    float k_final);
// flach: (1+|slack|/T)^{+1/k} if slack<=0 (violating), (1+slack/T)^{-k}
// otherwise. 1 when T <= 0.
float flachSlackScaleFactor(float slack_to, float T, float k);

// chen: rho_k = c/k with k = max(1, iter); additive
// lambda += rho*((a_from + d - a_to)/T), floored. See the T-normalization note
// on the definition - the subgradient is stepped as a fraction of the clock
// period, not in seconds. lambda unchanged when T <= 0.
float chenRho(int iter, float c);
float chenSubgradientLambda(float lambda,
                            float a_from,
                            float a_to,
                            float d,
                            float rho,
                            float T,
                            float floor);

// tennakoon: constant-free ratio a_from/(a_to - d); 1 when a_to - d <= 0.
float tennakoonRatioFactor(float a_from, float a_to, float d);

// livramento: constant-free ratio (a_from + d)/a_to (Alg. 1 line 13); 1 when
// a_to <= 0. Both this and tennakoon's are 1 on a critical arc (a_to = a_from +
// d) and differ on every other arc. Alg. 1 line 14's PI form (d/a_to) needs no
// branch of its own - it is this expression at a_from = 0.
float livramentoRatioFactor(float a_from, float a_to, float d);

// sharma: cexp accumulates across iterations (WPD = T - wns), floored positive
// so an over-met iteration cannot permanently freeze it. Criticality factor
// (1 - slack_to/T)^cexp - the sink-node slack over the clock period (Fig. 2
// line 10), base floored positive for slack > T; 1 when T <= 0.
float sharmaCexpStep(float cexp, float wns, float T, float r, float k);
float sharmaCritFactor(float slack_to, float T, float cexp);

// reimann: rho schedules and the dWNS-normalized / T-normalized asymmetric
// factor referenced to the per-arc initial slack.
float reimannRhoInc(int iter, float rho_init);
float reimannRhoDec(int iter, float rho_init);
// reimann quality-driven k-schedule (Eq. 7): k_est during estimation; k_lo (<1)
// if timing degraded vs. the previous iteration; k_hi (>1) if it improved;
// k_neutral otherwise (or on the first call, have_prev=false).
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

// Multiplicative normalized dual-subgradient with alpha halved on each pass
// rejection (this updater's private feature), plus the E4 mu handling.
class NormSubgradientUpdater : public LambdaUpdater
{
 public:
  explicit NormSubgradientUpdater(float alpha0) : alpha_(alpha0) {}
  void update(LrState& state, int iter) override;
  void onPassRejected() override { alpha_ *= 0.5f; }
  float currentStep() const override { return alpha_; }

 private:
  // Step size; starts at config.beta, halved on each pass rejection. Clamped to
  // [0, 1] at use.
  float alpha_;
};

// Flach TCAD'14 Alg. 2 asymmetric multiplicative slack scaling.
class FlachSlackScalingUpdater : public LambdaUpdater
{
 public:
  void update(LrState& state, int iter) override;
  float currentStep() const override { return last_k_; }

 private:
  float last_k_ = 0.0f;
};

// Chen-Chu-Wong ICCAD'98 additive subgradient (the theory baseline).
class ChenSubgradientUpdater : public LambdaUpdater
{
 public:
  void update(LrState& state, int iter) override;
  float currentStep() const override { return last_rho_; }

 private:
  float last_rho_ = 0.0f;
};

// Tennakoon-Sechen ICCAD'02 (Forge) constant-free arrival-ratio update.
class TennakoonRatioUpdater : public LambdaUpdater
{
 public:
  void update(LrState& state, int iter) override;
};

// Livramento DATE'13 Alg. 1 line 13 local multiplicative update.
class LivramentoRatioUpdater : public LambdaUpdater
{
 public:
  void update(LrState& state, int iter) override;
};

// Sharma ICCAD'15 accumulating-cexp criticality update.
class SharmaCexpUpdater : public LambdaUpdater
{
 public:
  void update(LrState& state, int iter) override;
  float currentStep() const override { return cexp_; }

 private:
  // Accumulates across iterations (Fig. 2 line 3 is a one-time init).
  float cexp_ = 1.0f;
};

// Reimann ISPD'16 Alg. 3 dWNS-normalized, S_init-targeted update with the
// quality-driven k-schedule (Eq. 7).
class ReimannDwnsUpdater : public LambdaUpdater
{
 public:
  void update(LrState& state, int iter) override;
  void setEstimationPhase(bool on) override { in_estimation_ = on; }
  float currentStep() const override { return last_rho_inc_; }

 private:
  float last_rho_inc_ = 0.0f;
  // k-schedule state: WNS at the previous update() call and whether one exists.
  // Not tracked during estimation (WNS is restored each dry-run iteration).
  float wns_prev_ = 0.0f;
  bool have_prev_ = false;
  bool in_estimation_ = false;
};

std::unique_ptr<LambdaUpdater> makeLambdaUpdater(
    const GlobalSizingConfig& config);

}  // namespace rsz
