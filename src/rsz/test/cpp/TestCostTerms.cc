// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the M3 cost-model terms (B2 axis) and the mutual-exclusion
// validator rule. Each term's arithmetic lives in a pure free function (no
// STA), so these tests assert the formula against hand-computed values and
// chain the φ recurrence over the Flach inverter-chain example (Eq. 10/11). The
// full evaluateSnapshot path (STA reads + graph traversal + the φ / delta-delay
// policy passes) is exercised by the global_sizing_cost_terms integration test.

#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "lr/CostTerms.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "utl/Logger.h"

namespace rsz {
namespace {

constexpr float kTol = 1e-5f;

// --- candidateSlewDelta ----------------------------------------------------

TEST(CostTermsFormula, CandidateSlewDelta)
{
  // Stronger cell (smaller drive resistance) sharpens the edge: Δslew < 0.
  EXPECT_NEAR(candidateSlewDelta(0.1f, 1000.0f, 500.0f), -0.05f, kTol);
  // Weaker/downsized cell (larger drive resistance) slows the edge: Δslew > 0.
  EXPECT_NEAR(candidateSlewDelta(0.1f, 1000.0f, 2000.0f), 0.1f, kTol);
  // Same drive resistance: no change.
  EXPECT_NEAR(candidateSlewDelta(0.1f, 1000.0f, 1000.0f), 0.0f, kTol);
  // Non-positive current drive resistance: no usable slope -> 0.
  EXPECT_NEAR(candidateSlewDelta(0.1f, 0.0f, 500.0f), 0.0f, kTol);
}

// --- slewSensitivityCost ---------------------------------------------------

TEST(CostTermsFormula, SlewSensitivityCost)
{
  // sens_sum * slew_delta.
  EXPECT_NEAR(slewSensitivityCost(3.0f, -0.05f), -0.15f, kTol);
  EXPECT_NEAR(slewSensitivityCost(4.0f, 0.025f), 0.1f, kTol);
  EXPECT_NEAR(slewSensitivityCost(0.0f, 0.5f), 0.0f, kTol);
}

// --- flachPhiArc (Eq. 11) --------------------------------------------------

TEST(CostTermsFormula, FlachPhiArcDominant)
{
  // Dominant: λ·(δd/δslew) + (δslew/δslew)·Σφ.
  EXPECT_NEAR(flachPhiArc(2.0f, 0.5f, 0.8f, 10.0f, true), 9.0f, kTol);
}

TEST(CostTermsFormula, FlachPhiArcNonDominant)
{
  // Non-dominant: only the local term; the downstream sum is dropped.
  EXPECT_NEAR(flachPhiArc(2.0f, 0.5f, 0.8f, 10.0f, false), 1.0f, kTol);
  // A leaf arc (no downstream) collapses to the local term either way.
  EXPECT_NEAR(flachPhiArc(1.0f, 0.3f, 0.9f, 0.0f, true), 0.3f, kTol);
}

// Chain the recurrence over the Flach Fig. 3 inverter chain 0->1->2->3 and
// check it reproduces the Eq. 10 closed form (λ = 1 as in the paper's example).
TEST(CostTermsFormula, FlachPhiChainMatchesEq10)
{
  const float s1_01 = 0.5f;  // δd_{0→1}/δslew_0
  const float s1_12 = 0.4f;  // δd_{1→2}/δslew_1
  const float s1_23 = 0.3f;  // δd_{2→3}/δslew_2
  const float s2_01 = 0.9f;  // δslew_1/δslew_0
  const float s2_12 = 0.8f;  // δslew_2/δslew_1

  // Reverse-topological: leaf arc first, then back toward the input.
  const float phi_23 = flachPhiArc(1.0f, s1_23, 0.0f, 0.0f, true);
  const float phi_12 = flachPhiArc(1.0f, s1_12, s2_12, phi_23, true);
  const float phi_01 = flachPhiArc(1.0f, s1_01, s2_01, phi_12, true);

  // Eq. 10 closed form: s1_01 + s2_01·(s1_12 + s2_12·s1_23).
  const float eq10 = s1_01 + s2_01 * (s1_12 + s2_12 * s1_23);
  EXPECT_NEAR(phi_01, eq10, kTol);
  EXPECT_NEAR(phi_01, 1.076f, kTol);

  // Eq. 12: the whole-path λ-weighted delay change for an input slew change.
  const float delta_slew_0 = 0.1f;
  EXPECT_NEAR(slewSensitivityCost(phi_01, delta_slew_0), 0.1076f, kTol);
}

// --- deltaDelayReferenced --------------------------------------------------

TEST(CostTermsFormula, DeltaDelayReferenced)
{
  // A candidate slower than the reference pays a positive delta.
  EXPECT_NEAR(deltaDelayReferenced(0.30f, 0.25f), 0.05f, kTol);
  // A candidate matching the reference (the previous sizes) is delta-free.
  EXPECT_NEAR(deltaDelayReferenced(0.25f, 0.25f), 0.0f, kTol);
  // A faster candidate earns a negative delta.
  EXPECT_NEAR(deltaDelayReferenced(0.20f, 0.25f), -0.05f, kTol);
}

// --- synthetic term assembly (the shape evaluateCellCost prices) -----------

TEST(CostTermsFormula, FanoutSlewTermAssembly)
{
  // A stronger candidate (cand_R < R) sharpens the driver's edge, reducing the
  // λ-weighted fanout arc delay -> a negative (helpful) cost contribution.
  const float slew = 0.05f, drive_res = 1000.0f, cand_res = 500.0f;
  const float fanout_slew_sens = 4.0f, timing_weight = 2.0f;
  const float slew_delta = candidateSlewDelta(slew, drive_res, cand_res);
  const float term
      = timing_weight * slewSensitivityCost(fanout_slew_sens, slew_delta);
  EXPECT_NEAR(term, 2.0f * (4.0f * -0.025f), kTol);  // -0.2
}

// --- per-arc vs port-worst λ·d pricing (C3 item 5) -------------------------

TEST(CostTermsFormula, PerArcVsPortWorstTimingCost)
{
  // A two-input gate whose two gate-internal arcs into the output pin carry
  // different λ and different delay: arc A = (λ 2, d 10), arc B = (λ 3, d 4).
  const std::vector<ArcLambdaDelay> arcs = {{2.0f, 10.0f}, {3.0f, 4.0f}};

  // The paper-faithful per-arc cost prices each arc against its OWN delay:
  //   2·10 + 3·4 = 32.
  EXPECT_NEAR(perArcTimingCost(arcs), 32.0f, kTol);

  // The shipped port-worst approximation prices the whole λ-sum against the
  // worst arc delay: (2 + 3)·10 = 50.
  const float lambda_sum = 5.0f;
  const float d_worst = 10.0f;
  EXPECT_NEAR(portWorstTimingCost(lambda_sum, d_worst), 50.0f, kTol);

  // The approximation strictly overprices whenever a non-worst sibling arc
  // carries λ (here arc B's 3·4 becomes 3·10): 50 > 32.
  EXPECT_GT(portWorstTimingCost(lambda_sum, d_worst), perArcTimingCost(arcs));
}

TEST(CostTermsFormula, PerArcAndPortWorstAgreeOnASingleArc)
{
  // Single-input gate (or a gate whose one λ-carrying arc is also the worst):
  // the port-worst collapse is exact, so the two forms coincide.
  const std::vector<ArcLambdaDelay> arcs = {{4.0f, 7.0f}};
  EXPECT_NEAR(perArcTimingCost(arcs), portWorstTimingCost(4.0f, 7.0f), kTol);
  EXPECT_NEAR(perArcTimingCost(arcs), 28.0f, kTol);
}

// --- factory / preset bundles (M3 scope 7) ---------------------------------

TEST(GlobalSizingPreset, CostTermBundles)
{
  GlobalSizingConfig config;

  // rsz_baseline: only the upstream-load term.
  config.applyPreset(GlobalSizingConfig::Preset::kRszBaseline);
  EXPECT_TRUE(config.cost_upstream_load);
  EXPECT_FALSE(config.cost_fanout_slew);
  EXPECT_FALSE(config.cost_global_phi);
  EXPECT_FALSE(config.cost_delta_delay);

  // flach: global-φ on.
  config.applyPreset(GlobalSizingConfig::Preset::kFlach);
  EXPECT_TRUE(config.cost_global_phi);
  EXPECT_FALSE(config.cost_fanout_slew);
  EXPECT_FALSE(config.cost_delta_delay);

  // livramento: fanout-slew on, its OWN Alg. 1 line 13 update (NOT tennakoon's
  // - the two rules differ off the critical arc), baseline seed.
  config.applyPreset(GlobalSizingConfig::Preset::kLivramento);
  EXPECT_TRUE(config.cost_fanout_slew);
  EXPECT_FALSE(config.cost_global_phi);
  EXPECT_EQ(config.lambda_update,
            GlobalSizingConfig::LambdaUpdate::kLivramentoRatio);
  EXPECT_EQ(config.lambda_seed,
            GlobalSizingConfig::LambdaSeed::kDelayPropCritMu);

  // livramento_partial round-trips through the string parser. The `_partial`
  // suffix is part of the Tcl-visible name (§3.3): every paper preset carries
  // it, so the bare paper name must NOT parse.
  GlobalSizingConfig::Preset p;
  EXPECT_TRUE(parsePreset("livramento_partial", p));
  EXPECT_EQ(p, GlobalSizingConfig::Preset::kLivramento);
  EXPECT_FALSE(parsePreset("livramento", p));
}

// The RSZ-0417 `preset=` field must distinguish "no preset was requested" from
// "-preset rsz_baseline", because the ablation harness reads that line as the
// per-run proof that the intended cell took effect and would otherwise record
// every preset-less run as a control arm. `preset` alone cannot: its default
// IS a preset name.
TEST(GlobalSizingPreset, ProvenanceDistinguishesDefaultFromExplicitBaseline)
{
  const GlobalSizingConfig untouched;
  EXPECT_FALSE(untouched.preset_explicit);
  EXPECT_EQ(untouched.preset, GlobalSizingConfig::Preset::kRszBaseline);

  GlobalSizingConfig config;
  config.applyPreset(GlobalSizingConfig::Preset::kRszBaseline);
  EXPECT_TRUE(config.preset_explicit);
  EXPECT_EQ(config.preset, GlobalSizingConfig::Preset::kRszBaseline);

  // Every preset sets it, and the reset at the head of applyPreset never
  // leaves it stale from a previous bundle.
  for (const GlobalSizingConfig::Preset p : kAllPresets) {
    GlobalSizingConfig c;
    c.applyPreset(p);
    EXPECT_TRUE(c.preset_explicit) << toString(p);
    EXPECT_EQ(c.preset, p) << toString(p);
  }
}

// Why the old `preset=rsz_baseline` echo on a preset-less run was a FALSE claim
// and not merely a redundant one: the struct defaults are almost, but not
// exactly, the rsz_baseline bundle. best_tracker deliberately defaults to the
// paper value (the M5 flip), which the baseline bundle then restores - so a
// preset-less config does not carry rsz_baseline's tracker either.
//
// Its own test, named for the asymmetry: it is a property of the M5 default
// flip, not of the provenance flag, so a future default change should fail
// HERE rather than inside the RSZ-0417 provenance test.
TEST(GlobalSizingPreset, StructDefaultsAreNotTheBaselineBundle)
{
  const GlobalSizingConfig untouched;
  GlobalSizingConfig baseline;
  baseline.applyPreset(GlobalSizingConfig::Preset::kRszBaseline);
  EXPECT_EQ(untouched.best_tracker,
            GlobalSizingConfig::BestTrackerKind::kFlachDominance);
  EXPECT_EQ(baseline.best_tracker,
            GlobalSizingConfig::BestTrackerKind::kWnsPassReject);
}

// --- validator: cost_global_phi XOR cost_delta_delay -----------------------

TEST(GlobalSizingValidate, PhiAndDeltaDelayMutuallyExclusive)
{
  utl::Logger logger;
  GlobalSizingConfig config;

  // Each estimator alone is valid.
  config.cost_global_phi = true;
  config.cost_delta_delay = false;
  EXPECT_TRUE(config.validate(&logger));

  config.cost_global_phi = false;
  config.cost_delta_delay = true;
  EXPECT_TRUE(config.validate(&logger));

  // fanout-slew composes with either.
  config.cost_fanout_slew = true;
  EXPECT_TRUE(config.validate(&logger));  // fanout + delta_delay
  config.cost_delta_delay = false;
  config.cost_global_phi = true;
  EXPECT_TRUE(config.validate(&logger));  // fanout + global_phi

  // Both global estimators together: hard reject (logger->error throws).
  config.cost_global_phi = true;
  config.cost_delta_delay = true;
  EXPECT_THROW(config.validate(&logger), std::runtime_error);
}

}  // namespace
}  // namespace rsz
