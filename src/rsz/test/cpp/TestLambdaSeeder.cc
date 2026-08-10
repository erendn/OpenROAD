// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the M2 lambda-seed family and the config validator. Each
// seeder's per-arc arithmetic lives in a pure free function (no STA), so these
// tests assert the formula against hand-computed values; the full seed() path
// (STA + graph traversal) and the estimation loop are exercised by the
// global_sizing_lambda_seed integration test. The reimann k-schedule pure fn
// is tested here too (it lands with the estimation loop in M2).

#include <memory>
#include <stdexcept>

#include "gtest/gtest.h"
#include "lr/LambdaSeeder.hh"
#include "lr/LambdaUpdater.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "utl/Logger.h"

namespace rsz {
namespace {

constexpr float kTol = 1e-5f;
constexpr float kFloor = 1e-12f;

// --- constant seed ---------------------------------------------------------

TEST(LambdaSeederFormula, ConstantSeed)
{
  EXPECT_NEAR(constantSeedLambda(12.0f, kFloor), 12.0f, kTol);
  EXPECT_NEAR(constantSeedLambda(1.0f, kFloor), 1.0f, kTol);
  // A value below the floor is clamped up to it.
  EXPECT_NEAR(constantSeedLambda(1e-15f, kFloor), kFloor, 1e-18f);
}

// --- mangiras internal arc (Eq. 5) -----------------------------------------

TEST(LambdaSeederFormula, MangirasInternalCriticalEqualLeakage)
{
  // Critical arc (a_from + d == a_to -> tr = 1), min-leakage gate (pr = 1):
  // (1 * 1)^2 = 1.
  EXPECT_NEAR(mangirasInternalArcLambda(0.1f, 0.1f, 0.2f, 1.0f, 1.0f, 2.0f),
              1.0f,
              kTol);
}

TEST(LambdaSeederFormula, MangirasInternalSlackAndLeakage)
{
  // tr = (0.05+0.05)/0.2 = 0.5; pr = 4/1 = 4; (0.5*4)^2 = 4.
  EXPECT_NEAR(mangirasInternalArcLambda(0.05f, 0.05f, 0.2f, 4.0f, 1.0f, 2.0f),
              4.0f,
              kTol);
}

TEST(LambdaSeederFormula, MangirasInternalPowerRatioOnly)
{
  // tr = 1, pr = 3, K = 1 -> 3.
  EXPECT_NEAR(mangirasInternalArcLambda(0.1f, 0.1f, 0.2f, 3.0f, 1.0f, 1.0f),
              3.0f,
              kTol);
}

TEST(LambdaSeederFormula, MangirasInternalGuards)
{
  // a_to <= 0 -> timing ratio falls back to 1 (pr = 2, K = 1 -> 2).
  EXPECT_NEAR(mangirasInternalArcLambda(0.1f, 0.1f, 0.0f, 2.0f, 1.0f, 1.0f),
              2.0f,
              kTol);
  // min_leak <= 0 -> power ratio falls back to 1 (tr = 1, K = 1 -> 1).
  EXPECT_NEAR(mangirasInternalArcLambda(0.1f, 0.1f, 0.2f, 2.0f, 0.0f, 1.0f),
              1.0f,
              kTol);
}

// --- mangiras endpoint arc (Eq. 6) -----------------------------------------

TEST(LambdaSeederFormula, MangirasEndpointMet)
{
  // Met endpoint tr = a_k/r_k = 0.5/1.0 = 0.5; pr = 2/1 = 2; (0.5*2)^2 = 1.
  EXPECT_NEAR(
      mangirasEndpointArcLambda(0.5f, 1.0f, 2.0f, 1.0f, 2.0f), 1.0f, kTol);
}

TEST(LambdaSeederFormula, MangirasEndpointViolating)
{
  // Violating endpoint tr = 1.2/1.0 = 1.2; pr = 1; K = 1 -> 1.2.
  EXPECT_NEAR(
      mangirasEndpointArcLambda(1.2f, 1.0f, 1.0f, 1.0f, 1.0f), 1.2f, kTol);
}

TEST(LambdaSeederFormula, MangirasEndpointGuards)
{
  // r_k <= 0 -> timing ratio falls back to 1 (pr = 2, K = 1 -> 2).
  EXPECT_NEAR(
      mangirasEndpointArcLambda(0.5f, 0.0f, 2.0f, 1.0f, 1.0f), 2.0f, kTol);
  // total_min_leak <= 0 -> power ratio falls back to 1 (tr = 0.5, K = 1).
  EXPECT_NEAR(
      mangirasEndpointArcLambda(0.5f, 1.0f, 2.0f, 0.0f, 1.0f), 0.5f, kTol);
}

// --- reimann k-schedule (Eq. 7) --------------------------------------------

TEST(LambdaSeederFormula, ReimannKSchedule)
{
  // Estimation phase pins k to k_est regardless of the trend.
  EXPECT_NEAR(
      reimannKForQuality(true, false, -0.2f, -0.1f, 5.0f, 0.5f, 2.0f, 1.0f),
      5.0f,
      kTol);
  // First call (no previous WNS): neutral.
  EXPECT_NEAR(
      reimannKForQuality(false, false, -0.2f, -0.1f, 5.0f, 0.5f, 2.0f, 1.0f),
      1.0f,
      kTol);
  // Timing degraded (wns_curr < wns_prev): k_lo.
  EXPECT_NEAR(
      reimannKForQuality(false, true, -0.2f, -0.1f, 5.0f, 0.5f, 2.0f, 1.0f),
      0.5f,
      kTol);
  // Solution improved (wns_curr > wns_prev): k_hi.
  EXPECT_NEAR(
      reimannKForQuality(false, true, -0.05f, -0.1f, 5.0f, 0.5f, 2.0f, 1.0f),
      2.0f,
      kTol);
  // Unchanged: neutral.
  EXPECT_NEAR(
      reimannKForQuality(false, true, -0.1f, -0.1f, 5.0f, 0.5f, 2.0f, 1.0f),
      1.0f,
      kTol);
}

// --- factory dispatch ------------------------------------------------------

TEST(LambdaSeederFactory, DispatchesEveryOption)
{
  using LS = GlobalSizingConfig::LambdaSeed;
  GlobalSizingConfig config;

  config.lambda_seed = LS::kDelayPropCritMu;
  EXPECT_NE(
      dynamic_cast<DelayPropCritMuSeeder*>(makeLambdaSeeder(config).get()),
      nullptr);
  config.lambda_seed = LS::kConstant;
  EXPECT_NE(dynamic_cast<ConstantSeeder*>(makeLambdaSeeder(config).get()),
            nullptr);
  config.lambda_seed = LS::kStateAdaptive;
  EXPECT_NE(dynamic_cast<StateAdaptiveSeeder*>(makeLambdaSeeder(config).get()),
            nullptr);
  config.lambda_seed = LS::kEstimationLoop;
  EXPECT_NE(dynamic_cast<EstimationLoopSeeder*>(makeLambdaSeeder(config).get()),
            nullptr);
}

// --- preset seed bundles (M2 scope 6) --------------------------------------

TEST(GlobalSizingPreset, SeedBundles)
{
  using LS = GlobalSizingConfig::LambdaSeed;
  using LU = GlobalSizingConfig::LambdaUpdate;
  GlobalSizingConfig config;

  config.applyPreset(GlobalSizingConfig::Preset::kRszBaseline);
  EXPECT_EQ(config.lambda_seed, LS::kDelayPropCritMu);
  EXPECT_EQ(config.lambda_update, LU::kNormSubgradient);

  config.applyPreset(GlobalSizingConfig::Preset::kFlach);
  EXPECT_EQ(config.lambda_seed, LS::kConstant);
  EXPECT_NEAR(config.lambda_init_value, 12.0f, kTol);
  EXPECT_EQ(config.lambda_update, LU::kFlachSlackScaling);

  config.applyPreset(GlobalSizingConfig::Preset::kSharmaSeq);
  EXPECT_EQ(config.lambda_seed, LS::kConstant);
  EXPECT_NEAR(config.lambda_init_value, 1.0f, kTol);
  EXPECT_EQ(config.lambda_update, LU::kSharmaCexp);

  config.applyPreset(GlobalSizingConfig::Preset::kChen);
  EXPECT_EQ(config.lambda_seed, LS::kConstant);
  EXPECT_NEAR(config.lambda_init_value, 1.0f, kTol);
  EXPECT_EQ(config.lambda_update, LU::kChenSubgradient);

  // Tennakoon keeps the baseline seed (its contour seeder is deferred).
  config.applyPreset(GlobalSizingConfig::Preset::kTennakoon);
  EXPECT_EQ(config.lambda_seed, LS::kDelayPropCritMu);
  EXPECT_EQ(config.lambda_update, LU::kTennakoonRatio);

  config.applyPreset(GlobalSizingConfig::Preset::kMangiras);
  EXPECT_EQ(config.lambda_seed, LS::kStateAdaptive);
  EXPECT_EQ(config.lambda_update, LU::kFlachSlackScaling);

  config.applyPreset(GlobalSizingConfig::Preset::kReimann);
  EXPECT_EQ(config.lambda_seed, LS::kEstimationLoop);
  EXPECT_EQ(config.lambda_update, LU::kReimannDwns);
}

// --- config validator ------------------------------------------------------

TEST(GlobalSizingValidate, BaselineIsValid)
{
  utl::Logger logger;
  GlobalSizingConfig config;  // rsz_baseline defaults
  EXPECT_TRUE(config.validate(&logger));
}

TEST(GlobalSizingValidate, StateAdaptiveRequiresAsGiven)
{
  utl::Logger logger;
  GlobalSizingConfig config;
  config.lambda_seed = GlobalSizingConfig::LambdaSeed::kStateAdaptive;
  // as_given: accepted.
  config.init_mode = GlobalSizingConfig::InitMode::kAsGiven;
  EXPECT_TRUE(config.validate(&logger));
  // any other init mode: hard reject (logger->error is [[noreturn]] ->
  // throws). Every mode is covered in TestInitPass; the two ends are enough
  // here.
  config.init_mode = GlobalSizingConfig::InitMode::kMinSize;
  EXPECT_THROW(config.validate(&logger), std::runtime_error);
  config.init_mode = GlobalSizingConfig::InitMode::kMaxSize;
  EXPECT_THROW(config.validate(&logger), std::runtime_error);
}

TEST(GlobalSizingValidate, EstimationLoopWarnsButAllows)
{
  utl::Logger logger;
  GlobalSizingConfig config;
  config.lambda_seed = GlobalSizingConfig::LambdaSeed::kEstimationLoop;
  // as_given: valid, no warning.
  config.init_mode = GlobalSizingConfig::InitMode::kAsGiven;
  EXPECT_TRUE(config.validate(&logger));
  // any other init mode: soft (warns) but still valid.
  config.init_mode = GlobalSizingConfig::InitMode::kMinSize;
  EXPECT_TRUE(config.validate(&logger));
}

}  // namespace
}  // namespace rsz
