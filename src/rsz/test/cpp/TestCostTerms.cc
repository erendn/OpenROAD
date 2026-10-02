// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the optional LR cost terms, the preset cost settings, and the
// validator rule that makes cost_global_phi and cost_delta_delay mutually
// exclusive.
// Each term's arithmetic is a free function with no STA dependency, so these
// tests check the formulas against hand-computed values and chain the φ
// recurrence over the inverter-chain example of Flach et al., TCAD 2014
// (Eqs. 10-11). The full evaluateSnapshot path (STA reads, graph traversal,
// and the φ and delta-delay passes) is covered by the global_sizing_cost_terms
// integration test.
//
// Per-arc timing pricing (timing_cost = per_arc) is checked on the Nangate45
// library, with delays read by hand from NAND2_X1's Liberty tables: the
// resizer's per-arc delay, the gate's own arcs and its fanin driver's arcs in
// the per-gate cost, an arc without an exact match on the cell, and the
// timing-weight anchor. A candidate cell is checked to cost what it costs once
// installed.

#include <stdexcept>
#include <vector>

#include "LRSubproblem.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "gtest/gtest.h"
#include "lr/CostTerms.hh"
#include "lr/LrState.hh"
#include "lr/TimingScale.hh"
#include "odb/db.h"
#include "rsz/GlobalSizingConfig.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/Mode.hh"
#include "sta/NetworkClass.hh"
#include "sta/Scene.hh"
#include "sta/TimingArc.hh"
#include "sta/Transition.hh"
#include "tst/IntegratedFixture.h"
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

// --- flachPhiArc (Flach et al., TCAD 2014, Eq. 11) -------------------------

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

// Chain the recurrence over the inverter chain 0->1->2->3 of Flach et al.,
// Fig. 3, and check that it reproduces the closed form of Eq. 10 (λ = 1 as in
// the paper's example).
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

// --- term assembly (the form evaluateCellCost computes) --------------------

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

// --- per-arc vs port-worst λ·d cost ----------------------------------------

TEST(CostTermsFormula, PerArcVsPortWorstTimingCost)
{
  // A two-input gate whose two gate-internal arcs into the output pin carry
  // different λ and different delay: arc A = (λ 2, d 10), arc B = (λ 3, d 4).
  const std::vector<ArcLambdaDelay> arcs = {{2.0f, 10.0f}, {3.0f, 4.0f}};

  // The per-arc cost used by the papers weighs each arc's λ by that arc's own
  // delay: 2·10 + 3·4 = 32.
  EXPECT_NEAR(perArcTimingCost(arcs), 32.0f, kTol);

  // The port-worst approximation used by the sizer weighs the whole λ sum by
  // the worst arc delay: (2 + 3)·10 = 50.
  const float lambda_sum = 5.0f;
  const float d_worst = 10.0f;
  EXPECT_NEAR(portWorstTimingCost(lambda_sum, d_worst), 50.0f, kTol);

  // The approximation overestimates whenever a non-worst arc carries λ (here
  // arc B's 3·4 becomes 3·10): 50 > 32.
  EXPECT_GT(portWorstTimingCost(lambda_sum, d_worst), perArcTimingCost(arcs));
}

TEST(CostTermsFormula, PerArcAndPortWorstAgreeOnASingleArc)
{
  // Single-input gate (or a gate whose only λ-carrying arc is also the worst):
  // the port-worst form is exact, so the two forms agree.
  const std::vector<ArcLambdaDelay> arcs = {{4.0f, 7.0f}};
  EXPECT_NEAR(perArcTimingCost(arcs), portWorstTimingCost(4.0f, 7.0f), kTol);
  EXPECT_NEAR(perArcTimingCost(arcs), 28.0f, kTol);
}

// --- preset cost settings --------------------------------------------------

TEST(GlobalSizingPreset, CostTermBundles)
{
  GlobalSizingConfig config;

  // rsz_baseline: only the upstream-load term.
  config.applyPreset(GlobalSizingConfig::Preset::kRszBaseline);
  EXPECT_TRUE(config.cost_upstream_load);
  EXPECT_FALSE(config.cost_fanout_slew);
  EXPECT_FALSE(config.cost_global_phi);
  EXPECT_FALSE(config.cost_delta_delay);

  // flach: global-φ off, as in Flach et al.'s runs with RC wires (Sec. IX-B).
  config.applyPreset(GlobalSizingConfig::Preset::kFlach);
  EXPECT_FALSE(config.cost_global_phi);
  EXPECT_FALSE(config.cost_fanout_slew);
  EXPECT_FALSE(config.cost_delta_delay);

  // livramento: fanout-slew on, its own lambda update (Livramento et al.,
  // DATE 2013, Alg. 1 line 13), which differs from tennakoon's off the
  // critical arc, and the default seed.
  config.applyPreset(GlobalSizingConfig::Preset::kLivramento);
  EXPECT_TRUE(config.cost_fanout_slew);
  EXPECT_FALSE(config.cost_global_phi);
  EXPECT_EQ(config.lambda_update,
            GlobalSizingConfig::LambdaUpdate::kLivramentoRatio);
  EXPECT_EQ(config.lambda_seed,
            GlobalSizingConfig::LambdaSeed::kDelayPropCritMu);

  // livramento_partial round-trips through the string parser. Every paper
  // preset's Tcl name ends in `_partial`, so the bare paper name must not
  // parse.
  GlobalSizingConfig::Preset p;
  EXPECT_TRUE(parsePreset("livramento_partial", p));
  EXPECT_EQ(p, GlobalSizingConfig::Preset::kLivramento);
  EXPECT_FALSE(parsePreset("livramento", p));
}

// timing_cost = per_arc is set only explicitly: every preset keeps the default
// worst_arc pricing.
TEST(GlobalSizingPreset, EveryPresetPricesTheWorstArc)
{
  for (const GlobalSizingConfig::Preset p : kAllPresets) {
    GlobalSizingConfig config;
    config.applyPreset(p);
    EXPECT_EQ(config.timing_cost, GlobalSizingConfig::TimingCost::kWorstArc)
        << toString(p);
  }
}

// The RSZ-0417 `preset=` field must distinguish "no preset was requested" from
// "-preset rsz_baseline", because scripts read that line to confirm which
// configuration a run used. `preset` alone cannot do this, since its default
// value is a preset name.
TEST(GlobalSizingPreset, ProvenanceDistinguishesDefaultFromExplicitBaseline)
{
  const GlobalSizingConfig untouched;
  EXPECT_FALSE(untouched.preset_explicit);
  EXPECT_EQ(untouched.preset, GlobalSizingConfig::Preset::kRszBaseline);

  GlobalSizingConfig config;
  config.applyPreset(GlobalSizingConfig::Preset::kRszBaseline);
  EXPECT_TRUE(config.preset_explicit);
  EXPECT_EQ(config.preset, GlobalSizingConfig::Preset::kRszBaseline);

  // Every preset sets it, and the reset at the start of applyPreset never
  // leaves a stale value from a previous preset.
  for (const GlobalSizingConfig::Preset p : kAllPresets) {
    GlobalSizingConfig c;
    c.applyPreset(p);
    EXPECT_TRUE(c.preset_explicit) << toString(p);
    EXPECT_EQ(c.preset, p) << toString(p);
  }
}

// The struct defaults are close to, but not the same as, the rsz_baseline
// preset: best_tracker defaults to flach_dominance, while rsz_baseline sets it
// to wns_pass_reject. So reporting `preset=rsz_baseline` for a run without a
// preset would be wrong, not just redundant.
//
// This is a separate test so that a future change to the defaults fails here
// rather than in the RSZ-0417 test above.
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

// --- per-arc pricing on Nangate45 -------------------------------------------

// NAND2_X1's tables are indexed by input transition (index_1) and load
// (index_2). At the third transition, 0.0171859 ns, and the third load,
// 3.709790 fF, the worst of cell_rise and cell_fall is
//   A1 -> ZN: max(0.0236392, 0.0201499) = 0.0236392 ns
//   A2 -> ZN: max(0.0258440, 0.0203566) = 0.0258440 ns
constexpr float kGridSlew = 0.0171859e-9f;
constexpr float kGridLoad = 3.709790e-15f;
constexpr float kDelayA1 = 0.0236392e-9f;
constexpr float kDelayA2 = 0.0258440e-9f;
constexpr float kRelTol = 1e-5f;

// TestObjectivePower.v, where u_nand (NAND2_X1) is fed by the top-level inputs
// a and b and drives u_aoi's input A. Both inputs get the grid transition, and
// the resizer's input slews for NAND2_X1 are read from u_nand's pins, so every
// NAND2_X1 delay below is at the grid slew.
class PerArcPricingOnSta : public tst::IntegratedFixture
{
 public:
  PerArcPricingOnSta()
      : tst::IntegratedFixture(tst::IntegratedFixture::Technology::kNangate45,
                               "_main/src/rsz/test/")
  {
  }

 protected:
  void SetUp() override
  {
    readVerilogAndSetup("TestObjectivePower.v");
    const sta::Cell* top = db_network_->cell(db_network_->topInstance());
    for (const char* input : {"a", "b"}) {
      sta_->setInputSlew(db_network_->findPort(top, input),
                         sta::RiseFallBoth::riseFall(),
                         sta::MinMaxAll::all(),
                         kGridSlew,
                         sta_->cmdMode()->sdc());
    }
    resizer_.resizePreamble();
    sta_->findRequireds();
    sta_->checkCapacitancesPreamble(sta_->scenes());
    sta_->checkSlewsPreamble();
    sta_->checkFanoutPreamble();
    nand_ = db_network_->dbToSta(block_->findInst("u_nand"));
    resizer_.annotateInputSlews(nand_, sta_->cmdScene(), sta::MinMax::max());

    // Only u_nand's two arcs carry a multiplier.
    state_.sta = sta_.get();
    state_.network = db_network_;
    state_.db_network = db_network_;
    state_.graph = sta_->graph();
    state_.logger = &logger_;
    state_.config = &config_;
    state_.allocate();
    a1_zn_ = edge(nand_, "A1", "ZN");
    a2_zn_ = edge(nand_, "A2", "ZN");
    state_.lambda[sta_->graph()->id(a1_zn_)] = kLambdaA1;
    state_.lambda[sta_->graph()->id(a2_zn_)] = kLambdaA2;
    in_.lambda = state_.lambda.data();
    in_.lambda_size = static_cast<int>(state_.lambda.size());
    in_.guard = GlobalSizingConfig::DownsizeGuard::kNone;
  }

  // The edge of the timing graph from `inst`'s pin `from` to its pin `to`.
  sta::Edge* edge(sta::Instance* inst, const char* from, const char* to)
  {
    sta::Graph* graph = sta_->graph();
    sta::Vertex* from_v
        = graph->pinLoadVertex(db_network_->findPin(inst, from));
    sta::VertexOutEdgeIterator it(from_v, graph);
    while (it.hasNext()) {
      sta::Edge* e = it.next();
      if (e->to(graph)->pin() == db_network_->findPin(inst, to)) {
        return e;
      }
    }
    ADD_FAILURE() << "no edge " << from << " -> " << to;
    return nullptr;
  }

  float arcSetDelay(const sta::TimingArcSet* arc_set, const float load)
  {
    return sta::delayAsFloat(resizer_.arcSetDelay(arc_set,
                                                  load,
                                                  sta_->cmdScene(),
                                                  sta::MinMax::max(),
                                                  sta_->arcDelayCalc()));
  }

  // A snapshot of `inst` with every output load and fanin driver load set to
  // the grid load.
  LRSubproblem::GateSnapshot gridSnapshot(sta::Instance* inst)
  {
    LRSubproblem::GateSnapshot snap;
    EXPECT_TRUE(subproblem_.snapshot(inst, in_, snap));
    for (LRSubproblem::OutputCtx& o : snap.outputs) {
      o.load_cap = kGridLoad;
    }
    for (LRSubproblem::UpstreamCtx& u : snap.upstream) {
      u.load_U_cur = kGridLoad;
    }
    return snap;
  }

  // sum(lambda * d) of `inst`'s current cell: its cost minus its power, over
  // the timing weight.
  float lambdaDelay(sta::Instance* inst)
  {
    LRSubproblem::GateSnapshot snap = gridSnapshot(inst);
    snap.candidates.clear();
    const LRSubproblem::GateDecision d = subproblem_.evaluateSnapshot(
        snap, kTimingWeight, 1.0f, sta_->arcDelayCalc());
    return (d.baseline_cost - snap.cur_leakage) / kTimingWeight;
  }

  static constexpr float kLambdaA1 = 5.0f;
  static constexpr float kLambdaA2 = 2.0f;
  // Large enough that timing decides between NAND2 sizes.
  static constexpr float kTimingWeight = 1e6f;

  GlobalSizingConfig config_;
  LrState state_;
  LRSubproblem::SnapshotInputs in_;
  LRSubproblem subproblem_{&resizer_};
  sta::Instance* nand_ = nullptr;
  sta::Edge* a1_zn_ = nullptr;
  sta::Edge* a2_zn_ = nullptr;
};

// The per-arc delay is the worst of the arc's rise and fall delay, read from
// that arc's own tables.
TEST_F(PerArcPricingOnSta, ArcSetDelayMatchesTheLibertyTable)
{
  sta::Vertex* a1
      = sta_->graph()->pinLoadVertex(db_network_->findPin(nand_, "A1"));
  ASSERT_NEAR(sta::delayAsFloat(sta_->slew(a1,
                                           sta::RiseFallBoth::riseFall(),
                                           sta_->scenes(),
                                           sta::MinMax::max())),
              kGridSlew,
              kGridSlew * kRelTol);

  EXPECT_NEAR(arcSetDelay(edge(nand_, "A1", "ZN")->timingArcSet(), kGridLoad),
              kDelayA1,
              kDelayA1 * kRelTol);
  EXPECT_NEAR(arcSetDelay(edge(nand_, "A2", "ZN")->timingArcSet(), kGridLoad),
              kDelayA2,
              kDelayA2 * kRelTol);
}

// Under per_arc each arc into ZN is priced at its own delay:
//   5 * 0.0236392 + 2 * 0.0258440 = 0.1698840 ns.
// Under worst_arc the summed multiplier takes the slower arc's delay:
//   (5 + 2) * 0.0258440 = 0.1809080 ns.
TEST_F(PerArcPricingOnSta, PricesEachArcOfATwoInputGateAtItsOwnDelay)
{
  in_.cost.upstream_load = false;
  in_.cost.per_arc = true;
  EXPECT_NEAR(lambdaDelay(nand_), 0.1698840e-9f, 0.1698840e-9f * kRelTol);
  in_.cost.per_arc = false;
  EXPECT_NEAR(lambdaDelay(nand_), 0.1809080e-9f, 0.1809080e-9f * kRelTol);
}

// With cost_delta_delay each arc is priced against its own reference. With
// 0.004 ns on A1 -> ZN and 0.010 ns on A2 -> ZN:
//   per_arc:   5 * (0.0236392 - 0.004) + 2 * (0.0258440 - 0.010) = 0.129884 ns
//   worst_arc: (5 + 2) * (0.0258440 - 0.010) = 0.110908 ns,
// where worst_arc takes the larger reference for the whole pin. The same
// holds for u_nand's arcs in u_aoi's upstream-load term (see the next test).
TEST_F(PerArcPricingOnSta, ReferencesEachArcToItsOwnDelay)
{
  state_.prev_delay.assign(state_.lambda.size(), 0.0f);
  state_.prev_delay[sta_->graph()->id(a1_zn_)] = 0.004e-9f;
  state_.prev_delay[sta_->graph()->id(a2_zn_)] = 0.010e-9f;
  in_.prev_delay = state_.prev_delay.data();
  in_.prev_delay_size = static_cast<int>(state_.prev_delay.size());
  in_.cost.delta_delay = true;
  sta::Instance* aoi = db_network_->dbToSta(block_->findInst("u_aoi"));
  for (sta::Instance* inst : {nand_, aoi}) {
    in_.cost.per_arc = true;
    EXPECT_NEAR(lambdaDelay(inst), 0.129884e-9f, 0.129884e-9f * kRelTol);
    in_.cost.per_arc = false;
    EXPECT_NEAR(lambdaDelay(inst), 0.110908e-9f, 0.110908e-9f * kRelTol);
  }
}

// u_aoi's input A is driven by u_nand, and u_aoi's own arcs carry no
// multiplier, so its timing term is the upstream-load term alone: u_nand's
// arcs at the load u_aoi's input leaves on ZN, here the grid load. The values
// are those of the own-arc test above.
TEST_F(PerArcPricingOnSta, PricesTheFaninDriversArcsAtTheirOwnDelays)
{
  sta::Instance* aoi = db_network_->dbToSta(block_->findInst("u_aoi"));
  ASSERT_EQ(gridSnapshot(aoi).upstream.size(), 1);
  in_.cost.per_arc = true;
  EXPECT_NEAR(lambdaDelay(aoi), 0.1698840e-9f, 0.1698840e-9f * kRelTol);
  in_.cost.per_arc = false;
  EXPECT_NEAR(lambdaDelay(aoi), 0.1809080e-9f, 0.1809080e-9f * kRelTol);
}

// A candidate is priced through its own arcs: offered to u_nand, NAND2_X2
// costs what it costs once it is u_nand's cell, which is less than NAND2_X1
// costs. Both cells take the resizer's target slews here.
TEST_F(PerArcPricingOnSta, PricesACandidateLikeTheInstalledCell)
{
  resizer_.resetInputSlews();
  in_.cost.upstream_load = false;
  in_.cost.per_arc = true;
  const float nand2_x1 = lambdaDelay(nand_);
  sta::LibertyCell* nand2_x2 = db_network_->findLibertyCell("NAND2_X2");
  LRSubproblem::GateSnapshot snap = gridSnapshot(nand_);
  std::erase_if(snap.candidates, [&](const LRSubproblem::Candidate& c) {
    return c.cell != nand2_x2;
  });
  ASSERT_EQ(snap.candidates.size(), 1);
  const LRSubproblem::GateDecision d = subproblem_.evaluateSnapshot(
      snap, kTimingWeight, 1.0f, sta_->arcDelayCalc());
  ASSERT_EQ(d.best_cell, nand2_x2);
  const float as_candidate
      = (d.best_cost - snap.candidates[0].leakage) / kTimingWeight;

  ASSERT_TRUE(resizer_.replaceCell(nand_, nand2_x2, false));
  const float installed = lambdaDelay(nand_);
  EXPECT_NEAR(as_candidate, installed, installed * kRelTol);
  EXPECT_LT(installed, nand2_x1);
}

// Cells of one group can split a pin pair's `when` conditions differently, so
// an arc set of the current cell can have no exact match on a candidate. It
// is then priced at the candidate's worst arc between the same two pins. The
// case is built by handing NAND2_X1 AND2_X1's A1 -> ZN arc set: same pins, no
// condition, but positive-unate, so it matches none of NAND2_X1's sets. It is
// priced at NAND2_X1's A1 -> ZN delay, 0.0236392 ns, not at the slower
// A2 -> ZN arc's and not at 0.
TEST_F(PerArcPricingOnSta, PricesAnArcWithoutAnExactMatchAtItsPinsWorstArc)
{
  in_.cost.upstream_load = false;
  in_.cost.per_arc = true;
  sta::LibertyCell* and2 = db_network_->findLibertyCell("AND2_X1");
  sta::TimingArcSet* and2_a1_zn = and2->timingArcSets(
      and2->findLibertyPort("A1"), and2->findLibertyPort("ZN"))[0];
  LRSubproblem::GateSnapshot snap = gridSnapshot(nand_);
  ASSERT_EQ(snap.cur_cell->findTimingArcSet(and2_a1_zn), nullptr);
  snap.outputs[0].arcs = {{.arc_set = and2_a1_zn, .lambda = 1.0f}};
  snap.candidates.clear();
  const LRSubproblem::GateDecision d = subproblem_.evaluateSnapshot(
      snap, kTimingWeight, 1.0f, sta_->arcDelayCalc());
  EXPECT_NEAR((d.baseline_cost - snap.cur_leakage) / kTimingWeight,
              kDelayA1,
              kDelayA1 * kRelTol);
}

// The timing weight's anchor samples each gate's sum(lambda * d) at its
// current load. Only u_nand carries a multiplier, so it is the only sample
// with timing pressure. A set_load on its output net puts the load on the grid,
// so the values are those of the own-arc test above.
TEST_F(PerArcPricingOnSta, AnchorSamplesEachArcAtItsOwnDelay)
{
  const sta::Pin* zn = db_network_->findPin(nand_, "ZN");
  sta_->setNetWireCap(db_network_->net(zn),
                      /*subtract_pin_cap=*/true,
                      sta::MinMaxAll::all(),
                      kGridLoad,
                      sta_->cmdMode()->sdc());
  ASSERT_NEAR(
      sta_->graphDelayCalc()->loadCap(zn, sta_->cmdScene(), sta::MinMax::max()),
      kGridLoad,
      kGridLoad * kRelTol);
  const auto anchor = [&](const GlobalSizingConfig::TimingCost cost) {
    config_.timing_cost = cost;
    TimingScaleInput in;
    collectTimingScaleInput(state_,
                            &resizer_,
                            subproblem_,
                            GlobalSizingConfig::TimingScale::kAutoMedian,
                            in);
    std::vector<float> samples;
    for (const TimingScaleInput::Gate& g : in.gates) {
      if (g.has_pressure) {
        samples.push_back(g.lambda_delay);
      }
    }
    EXPECT_EQ(samples.size(), 1);
    return samples.empty() ? 0.0f : samples[0];
  };

  EXPECT_NEAR(anchor(GlobalSizingConfig::TimingCost::kPerArc),
              0.1698840e-9f,
              0.1698840e-9f * kRelTol);
  EXPECT_NEAR(anchor(GlobalSizingConfig::TimingCost::kWorstArc),
              0.1809080e-9f,
              0.1809080e-9f * kRelTol);
}

}  // namespace
}  // namespace rsz
