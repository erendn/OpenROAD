// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>

#include "LrState.hh"

namespace rsz {

struct GlobalSizingConfig;

// D axis - lambda/mu initialization. Fills state.lambda and state.mu from the
// current timing picture (allocate() must have sized them first). The driver
// runs the E3 projection right after seed(), so a seeder that only needs
// KKT-consistent multipliers (state_adaptive) sets raw per-arc values and lets
// the projection make them flow-consistent.
class LambdaSeeder
{
 public:
  virtual ~LambdaSeeder() = default;
  virtual void seed(LrState& state) = 0;
};

// === Pure formula cores (unit-tested; no STA) ==============================
// Each returns the raw (pre-projection) lambda for one arc given the timing /
// leakage scalars the seeder reads from STA.

// constant seed: max(value, floor).
float constantSeedLambda(float value, float floor);

// Mangiras Eq. 5 (internal arc i->j of gate g): raw lambda =
// ((a_from + d)/a_to * leak/min_leak)^exponent. The timing ratio is 1 for the
// arc that sets a_to and < 1 otherwise; the power ratio is >= 1 and protects a
// currently-large gate from premature downsizing. a_to <= 0 or min_leak <= 0
// fall back to a unit ratio.
float mangirasInternalArcLambda(float a_from,
                                float d,
                                float a_to,
                                float leak,
                                float min_leak,
                                float exponent);

// Mangiras Eq. 6 (arc into a timing endpoint k): raw lambda =
// (a_k/r_k * total_leak/total_min_leak)^exponent. The timing ratio grows with
// the endpoint violation; the power ratio is the design-global leakage-vs-floor
// ratio. r_k <= 0 or total_min_leak <= 0 fall back to a unit ratio.
float mangirasEndpointArcLambda(float a_k,
                                float r_k,
                                float total_leak,
                                float total_min_leak,
                                float exponent);

// === Seeder implementations ================================================

// Delay-proportional lambda seed (max arc delay across rise/fall) + WNS-biased
// endpoint mu seed mu_k ~ max(0, margin - slack_k)^p, mu normalized to max 1.
class DelayPropCritMuSeeder : public LambdaSeeder
{
 public:
  void seed(LrState& state) override;
};

// Constant lambda = lambda_init_value on every data arc (Flach 12, Sharma/Chen
// 1); mu seeded from endpoint criticality as in the baseline (papers
// under-specify mu).
class ConstantSeeder : public LambdaSeeder
{
 public:
  void seed(LrState& state) override;
};

// Mangiras Eqs. 5-6 state-adaptive raw seed. Sets internal-arc lambda (Eq. 5)
// and endpoint mu (Eq. 6, the projection boundary); the KKT-consistent Eq. 7
// rescale is the E3 proportional_reverse_topo projection the driver runs next.
class StateAdaptiveSeeder : public LambdaSeeder
{
 public:
  void seed(LrState& state) override;
};

// Reimann Alg. 2 loop 1: seed the baseline multipliers as the estimation
// starting point. The driver runs the est_loop_iters dry-run sweeps that refine
// them (GlobalSizingPolicy::runEstimationLoop).
class EstimationLoopSeeder : public LambdaSeeder
{
 public:
  void seed(LrState& state) override;
};

std::unique_ptr<LambdaSeeder> makeLambdaSeeder(
    const GlobalSizingConfig& config);

}  // namespace rsz
