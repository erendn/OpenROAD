// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>

#include "LrState.hh"

namespace rsz {

struct GlobalSizingConfig;

// Initial values of the Lagrange multipliers. seed() fills state.lambda and
// state.mu from the current timing (allocate() must have sized them). The
// driver runs the flow projection right after seed(), so a seeder that needs
// KKT-consistent multipliers (state_adaptive) can set raw per-arc values and
// let the projection balance them.
class LambdaSeeder
{
 public:
  virtual ~LambdaSeeder() = default;
  virtual void seed(LrState& state) = 0;
};

// === Seed formulas (no STA; unit-tested) ==================================
// Each returns the raw (pre-projection) lambda for one arc, given the timing
// and leakage values the seeder reads.

// constant seed: max(value, floor).
float constantSeedLambda(float value, float floor);

// Mangiras et al., Eq. 5 (internal arc i->j of gate g): raw lambda =
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

// Mangiras et al., Eq. 6 (arc into a timing endpoint k): raw lambda =
// (a_k/r_k * total_leak/total_min_leak)^exponent. The timing ratio grows with
// the endpoint violation; the power ratio compares the design's leakage with
// its minimum possible leakage. r_k <= 0 or total_min_leak <= 0 fall back to a
// unit ratio.
float mangirasEndpointArcLambda(float a_k,
                                float r_k,
                                float total_leak,
                                float total_min_leak,
                                float exponent);

// === Seeder implementations ================================================

// Default seed: lambda = the arc's delay (max over rise/fall), and endpoint
// mu_k = max(0, margin - slack_k)^p, normalized to a maximum of 1.
class DelayPropCritMuSeeder : public LambdaSeeder
{
 public:
  void seed(LrState& state) override;
};

// Constant lambda = lambda_init_value on every data arc (12 in Flach et al., 1
// in Chen et al. and Sharma et al.). mu is seeded from endpoint slack as in the
// default seeder, since the papers do not specify it.
class ConstantSeeder : public LambdaSeeder
{
 public:
  void seed(LrState& state) override;
};

// Mangiras et al., Eqs. 5-6: raw seed from the current sizing and timing. Sets
// internal-arc lambda (Eq. 5) and endpoint mu (Eq. 6, the projection's
// boundary condition). The Eq. 7 rescale that makes them KKT-consistent is the
// proportional_reverse_topo projection the driver runs next.
class StateAdaptiveSeeder : public LambdaSeeder
{
 public:
  void seed(LrState& state) override;
};

// Reimann et al., ISPD 2016, Alg. 2, first loop: start from the default seed.
// The driver then refines it with est_loop_iters dry-run iterations
// (GlobalSizingPolicy::runEstimationLoop).
class EstimationLoopSeeder : public LambdaSeeder
{
 public:
  void seed(LrState& state) override;
};

std::unique_ptr<LambdaSeeder> makeLambdaSeeder(
    const GlobalSizingConfig& config);

}  // namespace rsz
