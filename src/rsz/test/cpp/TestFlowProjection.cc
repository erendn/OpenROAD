// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the MB balance-core's E3 x E4 seam: projectFlowBalance, the
// pure core of the KKT flow-balance projection (the STA walk that fills its
// ProjectionTopology is exercised by the global_sizing* integration tests).
//
// The headline assertions are the two scale properties, which are what the
// milestone is about:
//   - anchored to mu (reseed_each_iter / seed_once / update_as_lambda): a
//     uniform seed constant c cancels *exactly*, so flach(12) and chen(1) run
//     the identical lambda field. This pins the hardening pass's
//     "λ-magnitude finding" as an executable proof rather than an analytic
//     claim in a doc.
//   - anchored to lambda (endpoint_lambda): the projection commutes with a
//     uniform rescale, so flach(12) = 12 x chen(1) and the seed magnitude is a
//     live knob.

#include <vector>

#include "gtest/gtest.h"
#include "lr/FlowProjection.hh"
#include "rsz/GlobalSizingConfig.hh"

namespace rsz {
namespace {

using MuPolicy = GlobalSizingConfig::MuPolicy;

constexpr float kTol = 1e-5f;
constexpr float kFloor = 1e-12f;

// The fixture graph, in projection visit order (descending level):
//
//   a --e0--> c --e2--> d(endpoint 0)
//   b --e1--/
//
// d is an endpoint (level 2); c is internal (level 1); a and b are sources
// (level 0, no in-arcs, so the projection skips them).
//
// Vertex 0 = d: in {e2},      endpoint 0
// Vertex 1 = c: in {e0, e1},  out {e2}
// Vertex 2 = a: in {},        out {e0}
// Vertex 3 = b: in {},        out {e1}
ProjectionTopology fixture()
{
  ProjectionTopology topo;
  topo.in_edges = {2, 0, 1};   // d's in-arcs, then c's
  topo.out_edges = {2, 0, 1};  // c's out-arc, then a's, then b's

  ProjectionTopology::Vertex d;
  d.in_begin = 0;
  d.in_end = 1;
  d.endpoint = 0;

  ProjectionTopology::Vertex c;
  c.in_begin = 1;
  c.in_end = 3;
  c.out_begin = 0;
  c.out_end = 1;

  ProjectionTopology::Vertex a;
  a.out_begin = 1;
  a.out_end = 2;

  ProjectionTopology::Vertex b;
  b.out_begin = 2;
  b.out_end = 3;

  topo.vertices = {d, c, a, b};
  return topo;
}

// Project a uniform seed lambda = c over the fixture under `policy`.
// `first_projection` drives the endpoint-pressure policies' μ derivation; it is
// inert for the mu-anchored and endpoint_lambda policies.
std::vector<float> projectUniformSeed(const float c,
                                      const MuPolicy policy,
                                      const bool first_projection = false)
{
  const ProjectionTopology topo = fixture();
  std::vector<float> lambda = {c, c, c};
  // mu as every seeder but Mangiras's sets it: a slack-derived field
  // normalized to max 1, carrying no trace of lambda's magnitude.
  std::vector<float> mu = {1.0f};
  projectFlowBalance(topo, policy, first_projection, kFloor, mu, lambda);
  return lambda;
}

// --- the hand-computed base case -------------------------------------------

TEST(FlowProjection, AnchoredToMuRescalesEndpointInArcsToMu)
{
  // d: in_sum = e2 = 2, target = mu = 1 -> scale 1/2 -> e2 = 2 * 1/2 = 1.
  // c: in_sum = e0 + e1 = 3 + 5 = 8, target = out_sum = e2 = 1
  //    -> scale 1/8 -> e0 = 3/8 = 0.375, e1 = 5/8 = 0.625.
  const ProjectionTopology topo = fixture();
  std::vector<float> lambda = {3.0f, 5.0f, 2.0f};
  std::vector<float> mu = {1.0f};

  const ProjectionStats stats = projectFlowBalance(
      topo, MuPolicy::kReseedEachIter, false, kFloor, mu, lambda);

  EXPECT_NEAR(lambda[2], 1.0f, kTol);
  EXPECT_NEAR(lambda[0], 0.375f, kTol);
  EXPECT_NEAR(lambda[1], 0.625f, kTol);
  // Flow conservation at c now holds: in 0.375 + 0.625 = 1 = out.
  EXPECT_NEAR(lambda[0] + lambda[1], lambda[2], kTol);
  // a and b have no in-arcs, so only d and c were rescaled.
  EXPECT_EQ(stats.rescaled, 2);
  EXPECT_EQ(stats.zero_sum_fallback, 0);
}

TEST(FlowProjection, AnchoredToLambdaLeavesEndpointInArcsAlone)
{
  // d: target = its own in_sum = 2 -> scale 1 -> e2 unchanged at 2.
  // c: in_sum = 8, target = out_sum = e2 = 2 -> scale 1/4
  //    -> e0 = 0.75, e1 = 1.25.
  const ProjectionTopology topo = fixture();
  std::vector<float> lambda = {3.0f, 5.0f, 2.0f};
  std::vector<float> mu = {1.0f};

  projectFlowBalance(
      topo, MuPolicy::kEndpointLambda, false, kFloor, mu, lambda);

  EXPECT_NEAR(lambda[2], 2.0f, kTol);
  EXPECT_NEAR(lambda[0], 0.75f, kTol);
  EXPECT_NEAR(lambda[1], 1.25f, kTol);
  EXPECT_NEAR(lambda[0] + lambda[1], lambda[2], kTol);
  // mu is written back as the boundary actually used, so it stays the true
  // endpoint multiplier for the duality-gap diagnostic (it was 1.0 going in).
  EXPECT_NEAR(mu[0], 2.0f, kTol);
}

// --- the milestone's headline: does the seed magnitude survive? -------------

TEST(FlowProjection, AnchoredToMuCancelsTheSeedConstantExactly)
{
  // The hardening pass's finding #1, as a regression test: under a mu-anchored
  // endpoint boundary, flach(12) and chen(1) produce the *identical* field, so
  // lambda_init_value is structurally inert. Exact equality, not near: c
  // divides straight back out.
  const std::vector<float> flach
      = projectUniformSeed(12.0f, MuPolicy::kReseedEachIter);
  const std::vector<float> chen
      = projectUniformSeed(1.0f, MuPolicy::kReseedEachIter);

  EXPECT_EQ(flach, chen);
  // Concretely: d anchors e2 to mu = 1; c then splits that 1 evenly over its
  // two equal in-arcs.
  EXPECT_NEAR(flach[2], 1.0f, kTol);
  EXPECT_NEAR(flach[0], 0.5f, kTol);
  EXPECT_NEAR(flach[1], 0.5f, kTol);
}

TEST(FlowProjection, AnchoredToMuCancelsTheSeedUnderEveryMuAnchoredPolicy)
{
  // seed_once and update_as_lambda differ only in how mu is *maintained*
  // between iterations; both still anchor to it, so both cancel the seed at
  // iteration 0 exactly as reseed_each_iter does. This is why item 2's audit
  // concluded that update_as_lambda is not the paper-faithful endpoint option.
  for (const MuPolicy policy : {MuPolicy::kReseedEachIter,
                                MuPolicy::kSeedOnce,
                                MuPolicy::kUpdateAsLambda}) {
    EXPECT_EQ(projectUniformSeed(12.0f, policy),
              projectUniformSeed(1.0f, policy))
        << "policy " << toString(policy) << " should cancel the seed constant";
  }
}

TEST(FlowProjection, AnchoredToLambdaMakesFlach12DifferFromChen1)
{
  // The milestone's acceptance: with the endpoint anchored to its own lambda,
  // the projection is positively homogeneous - project(c*lambda) =
  // c*project(lambda) - so the seed magnitude survives it.
  const std::vector<float> flach
      = projectUniformSeed(12.0f, MuPolicy::kEndpointLambda);
  const std::vector<float> chen
      = projectUniformSeed(1.0f, MuPolicy::kEndpointLambda);

  ASSERT_EQ(flach.size(), chen.size());
  EXPECT_NE(flach, chen);
  for (size_t i = 0; i < flach.size(); ++i) {
    EXPECT_NEAR(flach[i], 12.0f * chen[i], kTol) << "arc " << i;
    EXPECT_GT(flach[i], chen[i]) << "arc " << i;
  }
  // Concretely: e2 keeps the seed; c splits it over two equal in-arcs.
  EXPECT_NEAR(flach[2], 12.0f, kTol);
  EXPECT_NEAR(flach[0], 6.0f, kTol);
  EXPECT_NEAR(flach[1], 6.0f, kTol);
}

TEST(FlowProjection, AnchoredToLambdaIsHomogeneousOnANonUniformField)
{
  // Homogeneity is a property of the map, not of the uniform seed: rescaling
  // *any* field by c rescales the projection's output by exactly c. (This is
  // the induction step of the notes' analytic argument.)
  const ProjectionTopology topo = fixture();
  const float c = 7.0f;

  std::vector<float> base = {3.0f, 5.0f, 2.0f};
  std::vector<float> mu_base = {1.0f};
  projectFlowBalance(
      topo, MuPolicy::kEndpointLambda, false, kFloor, mu_base, base);

  std::vector<float> scaled = {3.0f * c, 5.0f * c, 2.0f * c};
  std::vector<float> mu_scaled = {1.0f};
  projectFlowBalance(
      topo, MuPolicy::kEndpointLambda, false, kFloor, mu_scaled, scaled);

  for (size_t i = 0; i < base.size(); ++i) {
    EXPECT_NEAR(scaled[i], c * base[i], kTol) << "arc " << i;
  }
  EXPECT_NEAR(mu_scaled[0], c * mu_base[0], kTol);
}

// --- the C1 endpoint-pressure policies (endpoint_ratio / endpoint_additive) --

TEST(FlowProjection, EndpointPressurePoliciesDeriveMuLikeEndpointLambdaAtFirst)
{
  // On the FIRST projection (derive_endpoint_mu = true) endpoint_ratio and
  // endpoint_additive behave EXACTLY like endpoint_lambda: μ_0 = the endpoint's
  // in-arc λ sum, a no-op rescale of the endpoint arc. So the whole field
  // matches endpoint_lambda's, which is how the seed magnitude survives the
  // first pass (μ_0 ∝ the seed).
  const std::vector<float> ref = projectUniformSeed(
      12.0f, MuPolicy::kEndpointLambda, /*first_projection=*/true);
  for (const MuPolicy policy :
       {MuPolicy::kEndpointRatio, MuPolicy::kEndpointAdditive}) {
    EXPECT_EQ(projectUniformSeed(12.0f, policy, /*first_projection=*/true), ref)
        << "policy " << toString(policy)
        << " should derive μ from λ on the first projection";
  }
}

TEST(FlowProjection, EndpointPressurePoliciesAreHomogeneousAtFirstProjection)
{
  // Seed magnitude survives the derive: project(c·λ) = c·project(λ), so
  // flach(12) = 12·chen(1) - the same "μ ∝ c" property endpoint_lambda has,
  // extended to the two new options (the plan's homogeneity pin).
  for (const MuPolicy policy :
       {MuPolicy::kEndpointRatio, MuPolicy::kEndpointAdditive}) {
    const std::vector<float> flach
        = projectUniformSeed(12.0f, policy, /*first_projection=*/true);
    const std::vector<float> chen
        = projectUniformSeed(1.0f, policy, /*first_projection=*/true);
    ASSERT_EQ(flach.size(), chen.size());
    for (size_t i = 0; i < flach.size(); ++i) {
      EXPECT_NEAR(flach[i], 12.0f * chen[i], kTol)
          << "policy " << toString(policy) << " arc " << i;
    }
  }
}

TEST(FlowProjection, EndpointRatioDerivesThenAnchorsAndPressureSurvives)
{
  // The full endpoint_ratio lifecycle: derive μ_0 once, then anchor the in-arcs
  // to the μ the updater maintains - and a raised μ (the papers' branch-1
  // pressure) survives the projection as raised in-arc λ.
  const ProjectionTopology topo = fixture();

  // First projection (derive): μ_0 = d's in-sum = e2 = 2 (no-op on e2); c
  // splits that 2 over e0+e1 (=8) -> 0.75, 1.25.
  std::vector<float> lambda = {3.0f, 5.0f, 2.0f};
  std::vector<float> mu = {1.0f};
  projectFlowBalance(topo,
                     MuPolicy::kEndpointRatio,
                     /*derive_endpoint_mu=*/true,
                     kFloor,
                     mu,
                     lambda);
  EXPECT_NEAR(mu[0], 2.0f, kTol);
  EXPECT_NEAR(lambda[2], 2.0f, kTol);
  EXPECT_NEAR(lambda[0], 0.75f, kTol);
  EXPECT_NEAR(lambda[1], 1.25f, kTol);

  // The updater raises μ on the violating endpoint (a/required > 1). A later
  // projection ANCHORS to it: e2 -> 3, and c re-splits 3 over its in-arcs.
  mu[0] = 3.0f;
  projectFlowBalance(topo,
                     MuPolicy::kEndpointRatio,
                     /*derive_endpoint_mu=*/false,
                     kFloor,
                     mu,
                     lambda);
  EXPECT_NEAR(lambda[2], 3.0f, kTol);  // pressure pushed onto the in-arc
  EXPECT_NEAR(lambda[0] + lambda[1], 3.0f, kTol);  // conservation at c
  EXPECT_GT(lambda[2], 2.0f);  // strictly grew - dual ascent survived
}

// --- the axis is fully registered ------------------------------------------

TEST(FlowProjection, EveryMuPolicyRoundTripsThroughItsStringName)
{
  // Without this, an enum value can reach the Tcl allowed-value list with no
  // parser behind it: set_global_sizing_config -mu_policy endpoint_lambda would
  // pass Tcl, then warn RSZ-420 and silently fall back to reseed_each_iter -
  // i.e. the seed-scale fix would appear to do nothing, with nothing failing.
  for (const MuPolicy policy : {MuPolicy::kReseedEachIter,
                                MuPolicy::kSeedOnce,
                                MuPolicy::kUpdateAsLambda,
                                MuPolicy::kEndpointLambda,
                                MuPolicy::kEndpointRatio,
                                MuPolicy::kEndpointAdditive}) {
    MuPolicy parsed = MuPolicy::kReseedEachIter;
    ASSERT_TRUE(parseMuPolicy(toString(policy), parsed))
        << "no parser for '" << toString(policy) << "'";
    EXPECT_EQ(parsed, policy);
  }
  MuPolicy unused = MuPolicy::kReseedEachIter;
  EXPECT_FALSE(parseMuPolicy("not_a_policy", unused));
}

// --- degenerate paths ------------------------------------------------------

TEST(FlowProjection, ZeroInSumFallbackSharesTheTargetEvenly)
{
  // c's in-arcs are both 0, so there is nothing to rescale proportionally: the
  // target is shared evenly instead. e2 = 2 is anchored to mu = 1 first
  // (scale 1/2 -> e2 = 1), so c's target is 1 over two arcs -> 0.5 each.
  const ProjectionTopology topo = fixture();
  std::vector<float> lambda = {0.0f, 0.0f, 2.0f};
  std::vector<float> mu = {1.0f};

  const ProjectionStats stats = projectFlowBalance(
      topo, MuPolicy::kReseedEachIter, false, kFloor, mu, lambda);

  EXPECT_NEAR(lambda[0], 0.5f, kTol);
  EXPECT_NEAR(lambda[1], 0.5f, kTol);
  EXPECT_EQ(stats.rescaled, 1);
  EXPECT_EQ(stats.zero_sum_fallback, 1);
}

TEST(FlowProjection, FloorClampsTheRescaledMultiplier)
{
  // A large floor beats the rescaled value everywhere: the floor is why
  // homogeneity holds only above it (the notes' one caveat on the proof).
  const ProjectionTopology topo = fixture();
  std::vector<float> lambda = {3.0f, 5.0f, 2.0f};
  std::vector<float> mu = {1.0f};

  projectFlowBalance(topo, MuPolicy::kReseedEachIter, false, 10.0f, mu, lambda);

  EXPECT_NEAR(lambda[0], 10.0f, kTol);
  EXPECT_NEAR(lambda[1], 10.0f, kTol);
  EXPECT_NEAR(lambda[2], 10.0f, kTol);
}

TEST(FlowProjection, ZeroTargetAndZeroInSumLeavesTheArcsAlone)
{
  // mu = 0 with a zero in-sum hits neither branch: nothing to rescale, nothing
  // to share. The arcs keep their values (they are not even floored).
  ProjectionTopology topo;
  topo.in_edges = {0};
  ProjectionTopology::Vertex d;
  d.in_begin = 0;
  d.in_end = 1;
  d.endpoint = 0;
  topo.vertices = {d};

  std::vector<float> lambda = {0.0f};
  std::vector<float> mu = {0.0f};
  const ProjectionStats stats = projectFlowBalance(
      topo, MuPolicy::kReseedEachIter, false, kFloor, mu, lambda);

  EXPECT_NEAR(lambda[0], 0.0f, kTol);
  EXPECT_EQ(stats.rescaled, 0);
  EXPECT_EQ(stats.zero_sum_fallback, 0);
}

TEST(FlowProjection, SourceVerticesWithNoInArcsAreSkipped)
{
  // A vertex with no in-arcs has no flow to redistribute; its out-arcs are the
  // responsibility of the vertices they feed. Nothing is written.
  ProjectionTopology topo;
  topo.out_edges = {0};
  ProjectionTopology::Vertex a;
  a.out_begin = 0;
  a.out_end = 1;
  topo.vertices = {a};

  std::vector<float> lambda = {3.0f};
  std::vector<float> mu;
  const ProjectionStats stats = projectFlowBalance(
      topo, MuPolicy::kEndpointLambda, false, kFloor, mu, lambda);

  EXPECT_NEAR(lambda[0], 3.0f, kTol);
  EXPECT_EQ(stats.rescaled, 0);
}

// --- C3 item 2: floor-aware exact conservation -----------------------------

TEST(FlowProjection, FloorConservationRedistributesTheClampedResidual)
{
  // One endpoint, two in-arcs of very different magnitude, anchored to mu = 5
  // with a LARGE floor = 1.0. The small arc (0.1) scales to 0.0495, below the
  // floor. The plain max(λ·scale, floor) would pin it at 1.0 AND leave the big
  // arc at 4.95, so Σ_in = 5.95 != target — the conservation break every audit
  // flagged. The floor-aware rescale pins the small arc at the floor (1.0) and
  // rescales the big arc to carry the residual 5 - 1 = 4, so Σ_in = 5 exactly.
  ProjectionTopology topo;
  topo.in_edges = {0, 1};
  ProjectionTopology::Vertex d;
  d.in_begin = 0;
  d.in_end = 2;
  d.endpoint = 0;
  topo.vertices = {d};

  std::vector<float> lambda = {10.0f, 0.1f};
  std::vector<float> mu = {5.0f};
  const ProjectionStats stats = projectFlowBalance(
      topo, MuPolicy::kReseedEachIter, false, 1.0f, mu, lambda);

  EXPECT_NEAR(lambda[0], 4.0f, kTol);
  EXPECT_NEAR(lambda[1], 1.0f, kTol);              // pinned at the floor
  EXPECT_NEAR(lambda[0] + lambda[1], 5.0f, kTol);  // Σ_in = target EXACTLY
  EXPECT_EQ(stats.floor_redistributed, 1);         // took the slow path once
}

TEST(FlowProjection, FloorConservationInfeasibleTargetFloorsEveryArc)
{
  // target < floor·in_count is the documented infeasible boundary: 2 arcs,
  // floor 1.0, target 1.5 (feasible would need >= 2.0). Every arc floors, so
  // Σ_in = floor·in_count = 2.0 > target — the floor's zero-absorption guard
  // (why the floor exists) necessarily wins over exact conservation here.
  ProjectionTopology topo;
  topo.in_edges = {0, 1};
  ProjectionTopology::Vertex d;
  d.in_begin = 0;
  d.in_end = 2;
  d.endpoint = 0;
  topo.vertices = {d};

  std::vector<float> lambda = {0.4f, 0.3f};
  std::vector<float> mu = {1.5f};
  projectFlowBalance(topo, MuPolicy::kReseedEachIter, false, 1.0f, mu, lambda);

  EXPECT_NEAR(lambda[0], 1.0f, kTol);
  EXPECT_NEAR(lambda[1], 1.0f, kTol);
  EXPECT_NEAR(lambda[0] + lambda[1], 2.0f, kTol);  // floor·in_count > target
}

TEST(FlowProjection, FloorConservationInfeasibleFloorsEvenAnUnclampedLargeArc)
{
  // Infeasible target (1.9 < floor·in_count = 3) where one in-arc is huge and
  // would NOT individually fall below the floor on the first scale. The
  // water-fill clamps the two tiny arcs, then breaks on residual <= 0 — and must
  // still floor the large arc, not leave it at its stale pre-projection value.
  ProjectionTopology topo;
  topo.in_edges = {0, 1, 2};
  ProjectionTopology::Vertex d;
  d.in_begin = 0;
  d.in_end = 3;
  d.endpoint = 0;
  topo.vertices = {d};

  std::vector<float> lambda = {100.0f, 0.01f, 0.01f};
  std::vector<float> mu = {1.9f};
  projectFlowBalance(topo, MuPolicy::kReseedEachIter, false, 1.0f, mu, lambda);

  EXPECT_NEAR(lambda[0], 1.0f, kTol);  // the large arc is floored, not stale
  EXPECT_NEAR(lambda[1], 1.0f, kTol);
  EXPECT_NEAR(lambda[2], 1.0f, kTol);
  EXPECT_NEAR(lambda[0] + lambda[1] + lambda[2], 3.0f, kTol);
}

// --- C3 item 3: mid-run-minted arcs (cascade guard + neutral pricing) -------

TEST(FlowProjection, MintedArcsCascadeGuardLeavesInArcsWhenOutArcsAllDropped)
{
  // An internal (non-endpoint) vertex whose data out-arcs were all minted past
  // the multiplier space appears with an EMPTY out range. Its target would be
  // 0, and the plain rescale (in_sum > 0, scale = 0) would drive its in-arcs to
  // the floor — the backward λ blackout cascading upstream (tennakoon audit
  // §5.4). The guard leaves the in-arcs untouched instead.
  ProjectionTopology topo;
  topo.in_edges = {0, 1};
  ProjectionTopology::Vertex c;
  c.in_begin = 0;
  c.in_end = 2;
  c.out_begin = 0;
  c.out_end = 0;  // every out-arc dropped (minted past lambda) -> empty range
  c.endpoint = -1;
  topo.vertices = {c};

  std::vector<float> lambda = {0.5f, 0.3f};
  std::vector<float> mu;
  const ProjectionStats stats = projectFlowBalance(
      topo, MuPolicy::kEndpointLambda, false, kFloor, mu, lambda);

  EXPECT_NEAR(lambda[0], 0.5f, kTol);  // untouched, not floored to ~0
  EXPECT_NEAR(lambda[1], 0.3f, kTol);
  EXPECT_EQ(stats.rescaled, 0);
}

TEST(FlowProjection, MintedInArcsArePricedFromZeroOnTheNextPass)
{
  // Two freshly-minted in-arcs (grown to neutral 0 by growToLiveEdges) feed an
  // endpoint whose boundary target is 0.6. With in_sum = 0 the equal-split
  // rescue distributes the target over them, so each minted arc is priced
  // (0 -> 0.3) on this projection — it participates in pricing next iteration.
  ProjectionTopology topo;
  topo.in_edges = {0, 1};
  ProjectionTopology::Vertex d;
  d.in_begin = 0;
  d.in_end = 2;
  d.endpoint = 0;
  topo.vertices = {d};

  std::vector<float> lambda = {0.0f, 0.0f};  // just-minted, neutral
  std::vector<float> mu = {0.6f};
  const ProjectionStats stats = projectFlowBalance(
      topo, MuPolicy::kReseedEachIter, false, kFloor, mu, lambda);

  EXPECT_NEAR(lambda[0], 0.3f, kTol);
  EXPECT_NEAR(lambda[1], 0.3f, kTol);
  EXPECT_EQ(stats.zero_sum_fallback, 1);
}

}  // namespace
}  // namespace rsz
