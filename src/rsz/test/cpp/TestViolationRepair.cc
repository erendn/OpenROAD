// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the repair passes of global sizing declared in
// lr/ViolationRepair.hh: the max-capacitance fix pass (cap_fix_pass;
// Livramento et al., DATE 2013, Alg. 3) and the slew fix pass (slew_fix_pass;
// Sharma et al., ICCAD 2015, Sec. III-A).
//
// selectCapFixOption is checked on hand-made options: among the cells that
// clear a violation the cheapest wins; when none clears it, the cell closest
// to its limit wins, with the cost breaking ties; the current cell stays
// unless another cell is strictly better. Only livramento_partial turns the
// pass on.
//
// The same-Vt rule needs a library with several Vt flavors; it is checked by
// the global_sizing_cap_fix_pass_vt integration test on asap7.
//
// The pass itself is checked on the Nangate45 library, where the expected
// choices are read from the Liberty file. NAND2_X1, NAND2_X2 and NAND2_X4
// declare a max_capacitance of 59.3567, 118.713 and 237.427 fF on ZN and a
// cell_leakage_power of 17.39336, 34.78663 and 69.57324 nW.
//
// The walk it shares with init_mode = min_size_fixviol is covered by the
// global_sizing_init_min_size_fixviol integration test, and the pass inside
// global sizing by global_sizing_cap_fix_pass.
//
// The slew fix pass is checked on u_aoi (AOI21_X1), whose input A is driven by
// u_nand (NAND2_X1). Its selection rule is tested on hand-made groups in
// TestInitPass, and the pass inside global sizing by
// global_sizing_slew_fix_pass. From the Liberty file: AOI21_X1, AOI21_X2 and
// AOI21_X4 have a cell_leakage_power of 27.858395, 55.716720 and 111.433338 nW
// and a worst-case capacitance on A of 1.626352, 3.136406 and 6.139254 fF.
//
// The max-capacitance multipliers of relax_max_cap (lr/CapMultipliers.hh;
// Livramento et al., DATE 2013, Eq. 3, Alg. 1 lines 5 and 15, Alg. 2 lines 8
// and 11, Alg. 3 line 7) are checked on hand values: the update, the signed
// term and the initial value; then, on the same two gates, the pins they
// price, the terms a candidate pays, a candidate the max-cap filter would
// reject, and the fix pass with the term. The relaxation inside global sizing
// is checked by global_sizing_relax_max_cap.

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "LRSubproblem.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "gtest/gtest.h"
#include "lr/CapMultipliers.hh"
#include "lr/CapRecheck.hh"
#include "lr/ElectricalModel.hh"
#include "lr/LrState.hh"
#include "lr/ViolationRepair.hh"
#include "odb/db.h"
#include "rsz/GlobalSizingConfig.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/Mode.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Scene.hh"
#include "sta/Transition.hh"
#include "tst/IntegratedFixture.h"

namespace rsz {
namespace {

constexpr float kFemto = 1e-15f;
constexpr float kNano = 1e-9f;

// Values here are products of femtofarads and nanowatts held in floats, so
// they are compared to 1e-5 relative.
void expectNearRel(const float actual, const float expected)
{
  EXPECT_NEAR(actual, expected, std::abs(expected) * 1e-5f);
}

// --- selectCapFixOption ----------------------------------------------------

// Two cells clear the violation, at costs 5 and 3: the one at 3 wins. The
// cheapest option overall (cost 0.5) still exceeds its limit, so it loses to
// both.
TEST(CapFixSelect, TakesTheCheapestOptionThatClears)
{
  const std::vector<CapFixOption> options{
      {.excess = 2 * kFemto, .cost = 1.0f},
      {.excess = 0.0f, .cost = 5.0f},
      {.excess = 0.0f, .cost = 3.0f},
      {.excess = 1 * kFemto, .cost = 0.5f},
  };
  EXPECT_EQ(selectCapFixOption(options), 2u);
}

// No cell clears the violation: the one with the smallest excess wins, even
// though it costs the most.
TEST(CapFixSelect, WithoutAClearingOptionTakesTheSmallestExcess)
{
  const std::vector<CapFixOption> options{
      {.excess = 3 * kFemto, .cost = 1.0f},
      {.excess = 1 * kFemto, .cost = 9.0f},
      {.excess = 2 * kFemto, .cost = 0.5f},
  };
  EXPECT_EQ(selectCapFixOption(options), 1u);
}

// Two cells share the smallest excess: the cheaper of them wins.
TEST(CapFixSelect, EqualExcessesBreakOnCost)
{
  const std::vector<CapFixOption> options{
      {.excess = 3 * kFemto, .cost = 1.0f},
      {.excess = 1 * kFemto, .cost = 9.0f},
      {.excess = 1 * kFemto, .cost = 4.0f},
  };
  EXPECT_EQ(selectCapFixOption(options), 2u);
}

// options[0] is the current cell. It stays when no other cell is closer to its
// limit, and on a full tie.
TEST(CapFixSelect, KeepsTheCurrentCellWhenNothingIsBetter)
{
  const std::vector<CapFixOption> options{
      {.excess = 1 * kFemto, .cost = 2.0f},
      {.excess = 2 * kFemto, .cost = 1.0f},
      {.excess = 1 * kFemto, .cost = 2.0f},
  };
  EXPECT_EQ(selectCapFixOption(options), 0u);
  EXPECT_EQ(selectCapFixOption({}), 0u);
}

// --- configuration ----------------------------------------------------------

// Only livramento_partial runs the pass: Livramento et al. repair max
// capacitance after every subproblem solve (Alg. 2 line 23).
TEST(CapFixConfig, OnlyLivramentoRunsThePass)
{
  EXPECT_FALSE(GlobalSizingConfig{}.cap_fix_pass);
  for (const GlobalSizingConfig::Preset preset : kAllPresets) {
    GlobalSizingConfig config;
    config.applyPreset(preset);
    EXPECT_EQ(config.cap_fix_pass,
              preset == GlobalSizingConfig::Preset::kLivramento)
        << "preset " << toString(preset);
  }
}

// Only sharma_seq_partial runs the slew pass, after its max-capacitance-only
// initial repair (Sharma et al., Sec. III-A).
TEST(SlewFixConfig, OnlySharmaRunsThePass)
{
  EXPECT_FALSE(GlobalSizingConfig{}.slew_fix_pass);
  for (const GlobalSizingConfig::Preset preset : kAllPresets) {
    GlobalSizingConfig config;
    config.applyPreset(preset);
    EXPECT_EQ(config.slew_fix_pass,
              preset == GlobalSizingConfig::Preset::kSharmaSeq)
        << "preset " << toString(preset);
  }
}

// Only livramento_partial relaxes max capacitance.
TEST(RelaxMaxCapConfig, OnlyLivramentoRelaxesMaxCap)
{
  EXPECT_FALSE(GlobalSizingConfig{}.relax_max_cap);
  for (const GlobalSizingConfig::Preset preset : kAllPresets) {
    GlobalSizingConfig config;
    config.applyPreset(preset);
    EXPECT_EQ(config.relax_max_cap,
              preset == GlobalSizingConfig::Preset::kLivramento)
        << "preset " << toString(preset);
  }
}

// --- the max-capacitance multipliers on hand values -------------------------

// Alg. 1 line 15: beta * load / limit. Over the limit beta grows, below it
// beta shrinks, at it beta stays. Without a limit there is nothing to update.
TEST(CapBetaUpdate, ScalesBetaByLoadOverLimit)
{
  EXPECT_FLOAT_EQ(updatedCapBeta(2.0f, 90 * kFemto, 60 * kFemto), 3.0f);
  EXPECT_FLOAT_EQ(updatedCapBeta(2.0f, 30 * kFemto, 60 * kFemto), 1.0f);
  EXPECT_FLOAT_EQ(updatedCapBeta(2.0f, 60 * kFemto, 60 * kFemto), 2.0f);
  EXPECT_EQ(updatedCapBeta(2.0f, 90 * kFemto, 0.0f), 2.0f);
}

// Eq. 3: beta * (load - limit) is positive over the limit and a credit below
// it. With beta = 1e6 W/F an excess of 1 fF costs 1 nW.
TEST(CapBetaCost, IsSignedAroundTheLimit)
{
  expectNearRel(capBetaCost(1e6f, 100 * kFemto, 59.3567f * kFemto),
                40.6433f * kNano);
  expectNearRel(capBetaCost(1e6f, 50 * kFemto, 59.3567f * kFemto),
                -9.3567f * kNano);
  EXPECT_EQ(capBetaCost(1e6f, 60 * kFemto, 60 * kFemto), 0.0f);
}

// beta0 = 0.01 * P_med / (tw * L_med): 20 nW, tw = 4e13 and 60 fF give
// 0.01 * 20e-9 / 2.4 = 8.33333e-11 W/F, so that an excess of the median limit
// costs 1% of the median power once the timing weight is applied.
TEST(CapBetaInitial,
     MakesAnExcessOfTheMedianLimitCostOnePercentOfTheMedianPower)
{
  expectNearRel(initialCapBeta(20 * kNano, 4e13f, 60 * kFemto), 8.33333e-11f);
}

// The paper asks only for a positive value, so a degenerate input falls back
// to 1, as the timing weight does.
TEST(CapBetaInitial, FallsBackToOneWithoutPositiveInputs)
{
  EXPECT_EQ(initialCapBeta(0.0f, 4e13f, 60 * kFemto), 1.0f);
  EXPECT_EQ(initialCapBeta(20 * kNano, 0.0f, 60 * kFemto), 1.0f);
  EXPECT_EQ(initialCapBeta(20 * kNano, 4e13f, 0.0f), 1.0f);
}

// --- the pass on Nangate45 --------------------------------------------------

// TestObjectivePower.v, where u_nand (NAND2_X1) drives net n1. Each test puts
// a load on n1 and runs the pass once.
class CapFixPassOnSta : public tst::IntegratedFixture
{
 public:
  CapFixPassOnSta()
      : tst::IntegratedFixture(tst::IntegratedFixture::Technology::kNangate45,
                               "_main/src/rsz/test/")
  {
  }

 protected:
  void SetUp() override
  {
    readVerilogAndSetup("TestObjectivePower.v");
    resizer_.resizePreamble();
    sta_->findRequireds();
    // As global sizing does before its first sweep.
    sta_->checkCapacitancesPreamble(sta_->scenes());
    sta_->checkSlewsPreamble();
    sta_->checkFanoutPreamble();
    nand_ = db_network_->dbToSta(block_->findInst("u_nand"));

    state_.sta = sta_.get();
    state_.network = db_network_;
    state_.db_network = db_network_;
    state_.graph = sta_->graph();
    state_.resizer = &resizer_;
    state_.logger = &logger_;
    state_.config = &config_;
    state_.allocate();
  }

  // Sets the total load on u_nand's output to `cap`.
  void loadNandOutput(const float cap)
  {
    const sta::Pin* zn = db_network_->findPin(nand_, "ZN");
    sta_->setNetWireCap(db_network_->net(zn),
                        /*subtract_pin_cap=*/true,
                        sta::MinMaxAll::all(),
                        cap,
                        sta_->cmdMode()->sdc());
    ASSERT_NEAR(sta_->graphDelayCalc()->loadCap(
                    zn, sta_->cmdScene(), sta::MinMax::max()),
                cap,
                cap * 1e-5f);
  }

  // Puts `lambda` on u_nand's arc from `from` to ZN.
  void setNandLambda(const char* from, const float lambda)
  {
    sta::Graph* graph = sta_->graph();
    const sta::Pin* to = db_network_->findPin(nand_, "ZN");
    sta::Vertex* from_v
        = graph->pinLoadVertex(db_network_->findPin(nand_, from));
    sta::VertexOutEdgeIterator it(from_v, graph);
    while (it.hasNext()) {
      sta::Edge* e = it.next();
      if (e->to(graph)->pin() == to) {
        state_.lambda[graph->id(e)] = lambda;
        return;
      }
    }
    ADD_FAILURE() << "no edge " << from << " -> ZN";
  }

  std::string nandCell() const
  {
    return db_network_->libertyCell(nand_)->name();
  }

  RepairWalkStats fix(const float timing_weight)
  {
    return fixMaxCapViolations(state_, subproblem_, timing_weight);
  }

  GlobalSizingConfig config_;
  LrState state_;
  LRSubproblem subproblem_{&resizer_};
  sta::Instance* nand_ = nullptr;
};

// 100 fF is over NAND2_X1's limit and within NAND2_X2's and NAND2_X4's. With
// no multipliers the cost is the leakage, so NAND2_X2 (34.79 nW) wins over
// NAND2_X4 (69.57 nW).
TEST_F(CapFixPassOnSta, ClearsWithTheLowerPowerCellWhenTimingIsFree)
{
  loadNandOutput(100 * kFemto);
  const RepairWalkStats stats = fix(1.0f);
  EXPECT_EQ(nandCell(), "NAND2_X2");
  EXPECT_EQ(stats.violating, 1);
  EXPECT_EQ(stats.replaced, 1);
  EXPECT_EQ(stats.cleared, 1);
}

// The same load, with multipliers on u_nand's arcs and a timing weight large
// enough that delay decides. NAND2_X4's rise and fall delays from A1 and A2
// are below NAND2_X2's at 59.36 and 118.71 fF and every input transition of
// the tables, so it is the faster cell at 100 fF and wins.
TEST_F(CapFixPassOnSta, ClearsWithTheFasterCellWhenTimingDominates)
{
  loadNandOutput(100 * kFemto);
  setNandLambda("A1", 5.0f);
  setNandLambda("A2", 2.0f);
  const RepairWalkStats stats = fix(1e6f);
  EXPECT_EQ(nandCell(), "NAND2_X4");
  EXPECT_EQ(stats.violating, 1);
  EXPECT_EQ(stats.cleared, 1);
}

// 300 fF is over every NAND2's limit. The excesses are 240.64 fF (X1),
// 181.29 fF (X2) and 62.57 fF (X4), so NAND2_X4 wins although, with no
// multipliers, NAND2_X1 costs the least. A second pass finds the gate still
// over its limit and leaves it, since no cell comes closer.
TEST_F(CapFixPassOnSta, WithoutAClearingCellTakesTheOneClosestToItsLimit)
{
  loadNandOutput(300 * kFemto);
  RepairWalkStats stats = fix(1.0f);
  EXPECT_EQ(nandCell(), "NAND2_X4");
  EXPECT_EQ(stats.violating, 1);
  EXPECT_EQ(stats.replaced, 1);
  EXPECT_EQ(stats.cleared, 0);

  loadNandOutput(300 * kFemto);
  stats = fix(1.0f);
  EXPECT_EQ(nandCell(), "NAND2_X4");
  EXPECT_EQ(stats.violating, 1);
  EXPECT_EQ(stats.replaced, 0);
}

// Only max capacitance is repaired. A 1 ps max_transition on the design puts
// u_nand over its slew limit at 50 fF, where it is within its max capacitance:
// the walk that also checks slew, as the min_size_fixviol repair does, counts
// it as violating, and the fix pass leaves it alone.
TEST_F(CapFixPassOnSta, LeavesAGateOverOnlyItsSlewLimit)
{
  loadNandOutput(50 * kFemto);
  sta_->setSlewLimit(db_network_->cell(db_network_->topInstance()),
                     sta::MinMax::max(),
                     1e-12f,
                     sta_->cmdMode()->sdc());
  const RepairChoice keep
      = [](sta::Instance* /* inst */,
           sta::LibertyCell* current,
           const std::vector<OutputPinSnap>& /* outputs */) { return current; };
  EXPECT_EQ(repairInReverseTopoOrder(
                state_, {nand_}, RepairedLimits::kMaxCapAndSlew, keep)
                .violating,
            1);

  const RepairWalkStats stats = fix(1.0f);
  EXPECT_EQ(nandCell(), "NAND2_X1");
  EXPECT_EQ(stats.violating, 0);
}

// Without a violation the pass changes nothing.
TEST_F(CapFixPassOnSta, LeavesAGateWithinItsLimit)
{
  loadNandOutput(50 * kFemto);
  const RepairWalkStats stats = fix(1.0f);
  EXPECT_EQ(nandCell(), "NAND2_X1");
  EXPECT_EQ(stats.violating, 0);
}

// --- the slew fix pass on Nangate45 ----------------------------------------

// The slew limit is a design max_transition set relative to u_aoi's measured
// slew. The pass estimates a cell's slew as the measured slew scaled by the
// ratio of drive resistances, so which cells clear the limit follows from those
// ratios, which each test checks first.
class SlewFixPassOnSta : public CapFixPassOnSta
{
 protected:
  void SetUp() override
  {
    CapFixPassOnSta::SetUp();
    aoi_ = db_network_->dbToSta(block_->findInst("u_aoi"));
  }

  // Sets the total load on `inst`'s output pin `pin` to `cap`.
  void loadOutput(sta::Instance* inst, const char* pin, const float cap)
  {
    const sta::Pin* out = db_network_->findPin(inst, pin);
    sta_->setNetWireCap(db_network_->net(out),
                        /*subtract_pin_cap=*/true,
                        sta::MinMaxAll::all(),
                        cap,
                        sta_->cmdMode()->sdc());
    ASSERT_NEAR(sta_->graphDelayCalc()->loadCap(
                    out, sta_->cmdScene(), sta::MinMax::max()),
                cap,
                cap * 1e-5f);
  }

  void setDesignMaxSlew(const float slew)
  {
    sta_->setSlewLimit(db_network_->cell(db_network_->topInstance()),
                       sta::MinMax::max(),
                       slew,
                       sta_->cmdMode()->sdc());
  }

  // The worst slew on `inst`'s output pin `pin` from the current timing.
  float outputSlew(sta::Instance* inst, const char* pin) const
  {
    sta::Vertex* v
        = sta_->graph()->pinLoadVertex(db_network_->findPin(inst, pin));
    return sta::delayAsFloat(sta_->slew(
        v, sta::RiseFallBoth::riseFall(), sta_->scenes(), sta::MinMax::max()));
  }

  // The drive resistance of `cell`'s output ZN relative to AOI21_X1's.
  float driveRatio(const char* cell) const
  {
    auto zn_res = [this](const char* name) {
      return db_network_->findLibertyCell(name)
          ->findLibertyPort("ZN")
          ->driveResistance();
    };
    return zn_res(cell) / zn_res("AOI21_X1");
  }

  std::string aoiCell() const { return db_network_->libertyCell(aoi_)->name(); }

  RepairWalkStats fixSlews(const std::vector<sta::Instance*>& instances)
  {
    return fixMaxSlewViolations(state_, instances);
  }

  sta::Instance* aoi_ = nullptr;
};

// At 75% of AOI21_X1's slew, AOI21_X2 and AOI21_X4 both clear the limit. The
// pass takes the lower-leakage one, AOI21_X2 (55.72 nW against 111.43 nW).
TEST_F(SlewFixPassOnSta, ClearsWithTheLowestLeakageCell)
{
  ASSERT_LT(driveRatio("AOI21_X2"), 0.75f);
  ASSERT_LT(driveRatio("AOI21_X4"), driveRatio("AOI21_X2"));
  loadOutput(aoi_, "ZN", 20 * kFemto);
  setDesignMaxSlew(0.75f * outputSlew(aoi_, "ZN"));
  const RepairWalkStats stats = fixSlews({aoi_});
  EXPECT_EQ(aoiCell(), "AOI21_X2");
  EXPECT_EQ(stats.violating, 1);
  EXPECT_EQ(stats.replaced, 1);
  EXPECT_EQ(stats.cleared, 1);
}

// At 40% of AOI21_X1's slew, only AOI21_X4 clears the limit, and the pass
// takes it.
TEST_F(SlewFixPassOnSta, TakesTheOnlyCellThatClears)
{
  ASSERT_GT(driveRatio("AOI21_X2"), 0.4f);
  ASSERT_LT(driveRatio("AOI21_X4"), 0.4f);
  loadOutput(aoi_, "ZN", 20 * kFemto);
  setDesignMaxSlew(0.4f * outputSlew(aoi_, "ZN"));
  const RepairWalkStats stats = fixSlews({aoi_});
  EXPECT_EQ(aoiCell(), "AOI21_X4");
  EXPECT_EQ(stats.cleared, 1);
}

// The same limit, with u_nand's load 3 fF below NAND2_X1's max_capacitance of
// 59.3567 fF. AOI21_X4 would add 6.139254 - 1.626352 = 4.512902 fF to it and
// push it over, so it is rejected. AOI21_X2 adds 1.510054 fF and stays within
// the limit. It does not clear the slew limit but comes closer than AOI21_X1,
// so the pass takes it.
TEST_F(SlewFixPassOnSta, RejectsACellThatWouldOverloadItsDriver)
{
  ASSERT_GT(driveRatio("AOI21_X2"), 0.4f);
  ASSERT_LT(driveRatio("AOI21_X4"), 0.4f);
  loadOutput(aoi_, "ZN", 20 * kFemto);
  loadOutput(nand_, "ZN", 56.3567f * kFemto);
  setDesignMaxSlew(0.4f * outputSlew(aoi_, "ZN"));
  const RepairWalkStats stats = fixSlews({aoi_});
  EXPECT_EQ(aoiCell(), "AOI21_X2");
  EXPECT_EQ(stats.violating, 1);
  EXPECT_EQ(stats.replaced, 1);
  EXPECT_EQ(stats.cleared, 0);
  EXPECT_EQ(nandCell(), "NAND2_X1");
}

// Only slew is repaired. At 30 fF u_aoi is over AOI21_X1's max_capacitance of
// 25.3296 fF but within the library's max_transition, so the pass leaves it.
TEST_F(SlewFixPassOnSta, LeavesAGateOverOnlyItsMaxCap)
{
  loadOutput(aoi_, "ZN", 30 * kFemto);
  ASSERT_LT(outputSlew(aoi_, "ZN"), 0.198535e-9f);
  const RepairWalkStats stats = fixSlews({aoi_});
  EXPECT_EQ(aoiCell(), "AOI21_X1");
  EXPECT_EQ(stats.violating, 0);
}

// Without a violation the pass changes nothing.
TEST_F(SlewFixPassOnSta, LeavesAGateWithinItsSlewLimit)
{
  loadOutput(aoi_, "ZN", 20 * kFemto);
  setDesignMaxSlew(2.0f * outputSlew(aoi_, "ZN"));
  const RepairWalkStats stats = fixSlews({aoi_});
  EXPECT_EQ(aoiCell(), "AOI21_X1");
  EXPECT_EQ(stats.violating, 0);
}

// The walk visits u_nand before u_aoi, whatever the order of its input, and
// reads u_aoi's slew after u_nand is resized. A 1 ps limit puts both over it.
// The choice upsizes u_nand to NAND2_X4 and leaves u_aoi. The slew the walk
// read for u_aoi is below the one before the walk, and equals the one OpenSTA
// reports once the walk has refreshed timing.
TEST_F(SlewFixPassOnSta, ReadsAGateSlewAfterItsDriverIsResized)
{
  loadOutput(nand_, "ZN", 40 * kFemto);
  loadOutput(aoi_, "ZN", 20 * kFemto);
  setDesignMaxSlew(1e-12f);
  const float aoi_slew_before = outputSlew(aoi_, "ZN");

  sta::LibertyCell* nand_x4 = db_network_->findLibertyCell("NAND2_X4");
  std::vector<std::string> visited;
  float aoi_slew_read = 0.0f;
  const RepairChoice choose = [&](sta::Instance* inst,
                                  sta::LibertyCell* current,
                                  const std::vector<OutputPinSnap>& outputs) {
    visited.push_back(db_network_->pathName(inst));
    if (inst == nand_) {
      return nand_x4;
    }
    aoi_slew_read = outputs[0].slew;
    return current;
  };
  const RepairWalkStats stats = repairInForwardTopoOrder(
      state_, {aoi_, nand_}, RepairedLimits::kMaxSlew, choose);

  EXPECT_EQ(visited, (std::vector<std::string>{"u_nand", "u_aoi"}));
  EXPECT_EQ(stats.violating, 2);
  EXPECT_EQ(nandCell(), "NAND2_X4");
  EXPECT_LT(aoi_slew_read, aoi_slew_before);
  EXPECT_NEAR(aoi_slew_read, outputSlew(aoi_, "ZN"), 1e-15f);
}

// --- the max-capacitance multipliers on Nangate45 ----------------------------

// From the Liberty file, the output pins of TestObjectivePower.v's nine gates
// have a max_capacitance of (fF) 59.3567 (u_nand), 25.3296 (u_aoi, u_xor),
// 60.5011 (u_mux), 60.73 (u_inv), 60.73 and 60.2722 (u_ff's Q and QN), 121.155
// (u_buf), 26.7029 (u_nor) and 23.2315 (u_oai). The gates have a
// cell_leakage_power of (nW) 17.39336, 27.858395, 36.163718, 35.92839,
// 14.353185, 79.112308, 43.06082, 21.199545 and 34.026125. The median limit
// (index 5 of 10) is 60.2722 fF and the median leakage (index 4 of 9) is
// 34.026125 nW. Registers are priced even when they are not resized, so
// without register sizing only u_ff's leakage drops out, and the median of
// the other eight (index 4 of 8) is still 34.026125 nW.
class CapMultipliersOnSta : public SlewFixPassOnSta
{
 protected:
  sta::VertexId driverVertex(sta::Instance* inst, const char* pin) const
  {
    sta::Graph* graph = sta_->graph();
    return graph->id(graph->pinDrvrVertex(db_network_->findPin(inst, pin)));
  }

  float betaAt(sta::Instance* inst, const char* pin) const
  {
    const std::optional<float> beta
        = state_.cap_multipliers.beta(driverVertex(inst, pin));
    EXPECT_TRUE(beta.has_value());
    return beta.value_or(0.0f);
  }

  // A snapshot as the sweep takes it, with the multipliers when
  // `with_multipliers`, and no lambda or guard.
  LRSubproblem::GateSnapshot snapshot(sta::Instance* inst,
                                      const bool with_multipliers)
  {
    LRSubproblem::SnapshotInputs in;
    in.lambda = state_.lambda.data();
    in.lambda_size = static_cast<int>(state_.lambda.size());
    in.guard = GlobalSizingConfig::DownsizeGuard::kNone;
    if (with_multipliers) {
      in.cap_multipliers = &state_.cap_multipliers;
    }
    LRSubproblem::GateSnapshot snap;
    EXPECT_TRUE(subproblem_.snapshot(inst, in, snap));
    return snap;
  }

  // The fanin context of `inst`'s input `port` in `snap`.
  static const LRSubproblem::InputMaxCapCtx& input(
      const LRSubproblem::GateSnapshot& snap,
      const char* port)
  {
    for (const LRSubproblem::InputMaxCapCtx& in : snap.inputs) {
      if (in.in_port->name() == port) {
        return in;
      }
    }
    ADD_FAILURE() << "no input " << port;
    return snap.inputs.front();
  }

  sta::LibertyCell* cell(const char* name) const
  {
    return db_network_->findLibertyCell(name);
  }

  // Sets the load on `inst`'s output `pin` to `total` with wire capacitance
  // added to the pin loads, so that a sink with a larger input capacitance
  // raises it, unlike loadOutput's total.
  void loadOutputOverPins(sta::Instance* inst,
                          const char* pin,
                          const float total)
  {
    const sta::Pin* out = db_network_->findPin(inst, pin);
    auto set_wire = [&](const float cap) {
      sta_->setNetWireCap(db_network_->net(out),
                          /*subtract_pin_cap=*/false,
                          sta::MinMaxAll::all(),
                          cap,
                          sta_->cmdMode()->sdc());
    };
    auto load = [&] {
      return sta_->graphDelayCalc()->loadCap(
          out, sta_->cmdScene(), sta::MinMax::max());
    };
    set_wire(0.0f);
    set_wire(total - load());
    ASSERT_NEAR(load(), total, total * 1e-5f);
  }
};

// Every output pin of a sized gate and of a register is priced: ten pins, and
// beta0 = 0.01 * 34.026125 nW / (2 * 60.2722 fF) = 2822.70 W/F at a timing
// weight of 2. Without register sizing u_ff's Q and QN keep their
// multipliers, and beta0 = 0.01 * 34.026125 nW / 60.2722 fF = 5645.41 W/F at
// a weight of 1.
TEST_F(CapMultipliersOnSta, StartPricesTheSizedGatesAndTheRegisters)
{
  state_.cap_multipliers.start(state_, 2.0f);
  EXPECT_EQ(state_.cap_multipliers.pricedPins(), 10);
  expectNearRel(state_.cap_multipliers.beta0(), 2822.7047f);
  expectNearRel(betaAt(nand_, "ZN"), 2822.7047f);

  config_.size_registers = false;
  state_.cap_multipliers = {};
  state_.cap_multipliers.start(state_, 1.0f);
  EXPECT_EQ(state_.cap_multipliers.pricedPins(), 10);
  expectNearRel(state_.cap_multipliers.beta0(), 5645.4095f);
  sta::Instance* ff = db_network_->dbToSta(block_->findInst("u_ff"));
  expectNearRel(betaAt(ff, "Q"), 5645.4095f);
  expectNearRel(betaAt(ff, "QN"), 5645.4095f);
}

// A register that drives the clock network is priced only with
// include_clock_network, like every gate on the clock network, as global
// sizing resizes it only then. A clock on u_ff's Q puts u_ff on the clock
// network, and with it u_buf and u_oai, which load q: without the option six
// of the ten pins are priced, with it all ten.
TEST_F(CapMultipliersOnSta, TheClockNetworkIsPricedOnlyWhenItIsSized)
{
  config_.size_registers = false;
  sta::Instance* ff = db_network_->dbToSta(block_->findInst("u_ff"));
  sta::Instance* buf = db_network_->dbToSta(block_->findInst("u_buf"));
  sta::PinSet clock_pins(db_network_);
  clock_pins.insert(db_network_->findPin(ff, "Q"));
  sta_->makeClock(
      "q_clk", clock_pins, false, 1e-9f, {0.0f, 0.5e-9f}, "", sta_->cmdMode());
  state_.cap_multipliers.start(state_, 1.0f);
  EXPECT_EQ(state_.cap_multipliers.pricedPins(), 6);
  EXPECT_FALSE(state_.cap_multipliers.beta(driverVertex(ff, "Q")).has_value());
  EXPECT_FALSE(state_.cap_multipliers.beta(driverVertex(buf, "Z")).has_value());
  EXPECT_TRUE(
      state_.cap_multipliers.beta(driverVertex(nand_, "ZN")).has_value());

  config_.include_clock_network = true;
  state_.cap_multipliers = {};
  state_.cap_multipliers.start(state_, 1.0f);
  EXPECT_EQ(state_.cap_multipliers.pricedPins(), 10);
  EXPECT_TRUE(state_.cap_multipliers.beta(driverVertex(ff, "Q")).has_value());
}

// beta0's median power is over the gates global sizing resizes, while its
// median limit is over every priced pin. With u_inv dont-touch and registers
// not resized, seven gates are resized; the median of their leakage (index 3
// of 7) is 34.026125 nW, where with u_ff's 79.112308 nW it would be 35.92839
// nW (index 4 of 8). Nine pins are priced, u_ff's included, with a median
// limit (index 4 of 9) of 59.3567 fF. So beta0 = 0.01 * 34.026125 nW /
// 59.3567 fF = 5732.48 W/F at a timing weight of 1.
TEST_F(CapMultipliersOnSta, TheMedianPowerIsOverTheResizedGates)
{
  config_.size_registers = false;
  resizer_.setDontTouch(db_network_->dbToSta(block_->findInst("u_inv")), true);
  state_.cap_multipliers.start(state_, 1.0f);
  EXPECT_EQ(state_.cap_multipliers.pricedPins(), 9);
  expectNearRel(state_.cap_multipliers.beta0(), 5732.4826f);
}

// Alg. 1 line 15 at the pin's load and its current cell's limit: 100 fF on
// NAND2_X1 (59.3567 fF) multiplies beta by 1.68473; after a swap to NAND2_X4
// (237.427 fF) the next update multiplies it by 0.421182.
TEST_F(CapMultipliersOnSta, UpdateUsesTheLoadAndTheCurrentCellsLimit)
{
  loadNandOutput(100 * kFemto);
  state_.cap_multipliers.start(state_, 1.0f);
  // u_nand is the only gate over its limit.
  EXPECT_EQ(state_.cap_multipliers.violatingAtStart(), 1);
  const float beta0 = betaAt(nand_, "ZN");

  state_.cap_multipliers.update(state_);
  expectNearRel(betaAt(nand_, "ZN"), beta0 * 1.6847298f);

  ASSERT_TRUE(resizer_.replaceCell(nand_, cell("NAND2_X4")));
  loadNandOutput(100 * kFemto);
  state_.cap_multipliers.update(state_);
  expectNearRel(betaAt(nand_, "ZN"), beta0 * 1.6847298f * 0.42118209f);
  EXPECT_EQ(state_.cap_multipliers.countViolating(state_), 0);
}

// Which pins u_aoi's snapshot prices: its own output, and u_nand, which
// drives its input A, at u_nand's limit and load (50 fF). B1 and B2 are driven
// by primary inputs, which have no multiplier and keep the max-cap check.
// Without multipliers nothing is priced.
TEST_F(CapMultipliersOnSta, SnapshotFreezesThePricedPins)
{
  loadNandOutput(50 * kFemto);
  state_.cap_multipliers.start(state_, 1.0f);
  const float beta0 = state_.cap_multipliers.beta0();

  const LRSubproblem::GateSnapshot snap = snapshot(aoi_, true);
  EXPECT_TRUE(snap.relax_max_cap);
  ASSERT_EQ(snap.outputs.size(), 1u);
  EXPECT_EQ(snap.outputs[0].cap_beta, beta0);
  ASSERT_EQ(snap.fanin_cap_betas.size(), 1u);
  const LRSubproblem::FaninCapBeta& nand = snap.fanin_cap_betas[0];
  EXPECT_EQ(nand.driver, db_network_->findPin(nand_, "ZN"));
  EXPECT_EQ(nand.beta, beta0);
  expectNearRel(nand.load, 50 * kFemto);
  expectNearRel(nand.limit, 59.3567f * kFemto);
  ASSERT_EQ(nand.inputs.size(), 1u);
  EXPECT_EQ(snap.inputs[nand.inputs[0]].in_port->name(), "A");
  EXPECT_TRUE(input(snap, "A").drivers[0].priced);
  for (const char* port : {"B1", "B2"}) {
    for (const LRSubproblem::DriverCapCheck& dc : input(snap, port).drivers) {
      EXPECT_FALSE(dc.priced) << port;
    }
  }

  const LRSubproblem::GateSnapshot plain = snapshot(aoi_, false);
  EXPECT_FALSE(plain.relax_max_cap);
  EXPECT_FALSE(plain.outputs[0].cap_beta.has_value());
  EXPECT_TRUE(plain.fanin_cap_betas.empty());
  EXPECT_FALSE(input(plain, "A").drivers[0].priced);
}

// Alg. 2 line 11 at u_nand with 100 fF on its output and beta = 1e6 W/F
// (1 nW per fF), at each candidate's own limit: 40.6433 nW over NAND2_X1's
// 59.3567 fF, and credits of 18.713 and 137.427 nW below NAND2_X2's 118.713
// and NAND2_X4's 237.427 fF. u_nand's inputs come from primary inputs, so it
// pays no fanin term.
TEST_F(CapMultipliersOnSta, OwnPinTermUsesTheCandidatesLimit)
{
  loadNandOutput(100 * kFemto);
  state_.cap_multipliers.start(state_, 1.0f);
  LRSubproblem::GateSnapshot snap = snapshot(nand_, true);
  ASSERT_TRUE(snap.outputs[0].cap_beta.has_value());
  snap.outputs[0].cap_beta = 1e6f;
  expectNearRel(subproblem_.capBetaOwnCost(snap, cell("NAND2_X1")),
                40.6433f * kNano);
  expectNearRel(subproblem_.capBetaOwnCost(snap, cell("NAND2_X2")),
                -18.713f * kNano);
  expectNearRel(subproblem_.capBetaOwnCost(snap, cell("NAND2_X4")),
                -137.427f * kNano);
  EXPECT_EQ(subproblem_.capBetaFaninCost(snap, cell("NAND2_X4")), 0.0f);
}

// Alg. 2 line 8 at u_aoi, whose input A loads u_nand (NAND2_X1, 59.3567 fF).
// With 59 fF on u_nand's output and beta = 1e6 W/F, AOI21_X1 leaves u_nand
// 0.3567 fF below its limit, a credit. AOI21_X2 and AOI21_X4 replace A's
// 1.626352 fF by 3.136406 and 6.139254 fF, which puts u_nand 1.153354 and
// 4.156202 fF over it.
TEST_F(CapMultipliersOnSta, FaninTermIncludesTheCandidatesInputCap)
{
  loadNandOutput(59 * kFemto);
  state_.cap_multipliers.start(state_, 1.0f);
  LRSubproblem::GateSnapshot snap = snapshot(aoi_, true);
  ASSERT_EQ(snap.fanin_cap_betas.size(), 1u);
  snap.fanin_cap_betas[0].beta = 1e6f;
  expectNearRel(subproblem_.capBetaFaninCost(snap, cell("AOI21_X1")),
                -0.3567f * kNano);
  expectNearRel(subproblem_.capBetaFaninCost(snap, cell("AOI21_X2")),
                1.153354f * kNano);
  expectNearRel(subproblem_.capBetaFaninCost(snap, cell("AOI21_X4")),
                4.156202f * kNano);
}

// A driver that feeds two of the gate's input pins is priced once, at a load
// that includes both pins' change (Alg. 2 line 8 sums over fanin gates).
// u_aoi's B1 is moved onto n1, so the snapshot finds u_nand behind A and B1
// and records it once. At 59 fF AOI21_X1 leaves u_nand 0.3567 fF below its
// limit, a credit taken once, and AOI21_X2 adds 1.510054 fF on A and
// 3.129761 - 1.647003 = 1.482758 fF on B1, which puts it 2.636112 fF over.
TEST_F(CapMultipliersOnSta, ADriverOnTwoInputPinsIsPricedOnce)
{
  odb::dbITerm* b1 = block_->findInst("u_aoi")->findITerm("B1");
  b1->disconnect();
  b1->connect(block_->findNet("n1"));
  loadNandOutput(59 * kFemto);
  state_.cap_multipliers.start(state_, 1.0f);
  LRSubproblem::GateSnapshot snap = snapshot(aoi_, true);
  ASSERT_EQ(snap.fanin_cap_betas.size(), 1u);
  LRSubproblem::FaninCapBeta& nand = snap.fanin_cap_betas[0];
  EXPECT_EQ(nand.driver, db_network_->findPin(nand_, "ZN"));
  ASSERT_EQ(nand.inputs.size(), 2u);
  nand.beta = 1e6f;
  expectNearRel(subproblem_.capBetaFaninCost(snap, cell("AOI21_X1")),
                -0.3567f * kNano);
  expectNearRel(subproblem_.capBetaFaninCost(snap, cell("AOI21_X2")),
                2.636112f * kNano);
}

// u_nand as NAND2_X2 with 62 fF on its output. NAND2_X1 would exceed its
// 59.3567 fF limit, within its slew limit. Without the relaxation the filter
// rejects it, and NAND2_X4 costs more leakage than NAND2_X2, so the gate
// stays. With beta = 1e5 W/F (0.1 nW per fF) NAND2_X1 is admitted and wins at
// its leakage plus its excess, 17.39336 + 0.1 * 2.6433 = 17.65769 nW, against
// 34.78663 - 0.1 * 56.713 = 29.11533 nW for NAND2_X2.
TEST_F(CapMultipliersOnSta, ACellOverItsLimitIsPricedNotRejected)
{
  ASSERT_TRUE(resizer_.replaceCell(nand_, cell("NAND2_X2")));
  loadNandOutput(62 * kFemto);
  sta::ArcDelayCalc* adc = sta_->arcDelayCalc();

  const LRSubproblem::GateSnapshot plain = snapshot(nand_, false);
  const sta::LibertyPort* x1_zn = cell("NAND2_X1")->findLibertyPort("ZN");
  ASSERT_EQ(outputMaxSlewExcess(sta_.get(),
                                x1_zn,
                                plain.outputs[0].slew_factor,
                                plain.outputs[0].load_cap,
                                plain.scene,
                                sta::MinMax::max()),
            0.0f);
  ASSERT_GT(
      outputMaxCapExcess(x1_zn, plain.outputs[0].load_cap, sta::MinMax::max()),
      0.0f);
  EXPECT_EQ(subproblem_.evaluateSnapshot(plain, 1.0f, 1.0f, adc).best_cell,
            nullptr);

  state_.cap_multipliers.start(state_, 1.0f);
  LRSubproblem::GateSnapshot snap = snapshot(nand_, true);
  snap.outputs[0].cap_beta = 1e5f;
  const LRSubproblem::GateDecision decision
      = subproblem_.evaluateSnapshot(snap, 1.0f, 1.0f, adc);
  EXPECT_EQ(decision.best_cell, cell("NAND2_X1"));
  expectNearRel(decision.best_cost, 17.65769f * kNano);
  expectNearRel(decision.baseline_cost, 29.11533f * kNano);
}

// FIX_VIOLATIONS with the beta term (Alg. 3 line 7). beta0 is taken for a
// timing weight equal to kInitialCapBetaShare and the pass runs at 1, which
// cancels the share: an excess costs P_med / L_med = 34.026125 / 60.2722 =
// 0.564541 nW per fF (CapMultipliersOnSta). At 100 fF,
// NAND2_X2 and NAND2_X4 clear NAND2_X1's violation. Without the term NAND2_X2
// wins on leakage (ClearsWithTheLowerPowerCellWhenTimingIsFree). With it
// NAND2_X4 wins: 69.57324 - 0.564541 * 137.427 = -8.00993 nW against 34.78663 -
// 0.564541 * 18.713 = 24.22238 nW. At 300 fF no cell clears, and NAND2_X4, the
// closest to its limit, wins with and without the term.
TEST_F(CapMultipliersOnSta, FixPassPricesTheTermAmongCellsThatClear)
{
  config_.relax_max_cap = true;
  loadNandOutput(100 * kFemto);
  state_.cap_multipliers.start(state_, kInitialCapBetaShare);
  RepairWalkStats stats = fix(1.0f);
  EXPECT_EQ(nandCell(), "NAND2_X4");
  EXPECT_EQ(stats.cleared, 1);

  ASSERT_TRUE(resizer_.replaceCell(nand_, cell("NAND2_X1")));
  loadNandOutput(300 * kFemto);
  stats = fix(1.0f);
  EXPECT_EQ(nandCell(), "NAND2_X4");
  EXPECT_EQ(stats.cleared, 0);
}

// The term never puts a cell that keeps the violation ahead of one that
// clears it. With beta0 for a timing weight of 2 * kInitialCapBetaShare and the
// pass at 1, an excess costs P_med / (2 * L_med) = 0.282270 nW per fF. At 100
// fF NAND2_X1 is then the cheapest, 17.39336 + 0.282270 * 40.6433 = 28.8658 nW,
// but stays over its limit; of the cells that clear it, NAND2_X2 (34.78663 -
// 0.282270 * 18.713 = 29.5045 nW) beats NAND2_X4 (69.57324 - 0.282270 * 137.427
// = 30.7817 nW).
TEST_F(CapMultipliersOnSta, FixPassClearsBeforeItPrices)
{
  config_.relax_max_cap = true;
  loadNandOutput(100 * kFemto);
  state_.cap_multipliers.start(state_, 2 * kInitialCapBetaShare);
  const RepairWalkStats stats = fix(1.0f);
  EXPECT_EQ(nandCell(), "NAND2_X2");
  EXPECT_EQ(stats.cleared, 1);
}

// The post-sweep max-cap re-check leaves the nets the cost prices. u_ff
// (DFF_X1) is priced as a register although register sizing is off; its Q
// drives q, which loads u_buf (BUF_X2). u_nand is made dont-touch, so it has
// no multiplier; it drives n1, which loads u_aoi's A. Both nets are put at
// 59 fF, below Q's 60.73 fF and NAND2_X1's 59.3567 fF. A sweep's moves to
// BUF_X8 (6.585178 fF on A instead of 1.779209) and AOI21_X4 (6.139254 fF
// instead of 1.626352) push both over. The re-check reverts u_aoi and keeps
// u_buf.
TEST_F(CapMultipliersOnSta, RecheckRevertsOnlyOnNetsWithoutAMultiplier)
{
  config_.size_registers = false;
  config_.relax_max_cap = true;
  sta::Instance* ff = db_network_->dbToSta(block_->findInst("u_ff"));
  sta::Instance* buf = db_network_->dbToSta(block_->findInst("u_buf"));
  resizer_.setDontTouch(nand_, true);
  loadOutputOverPins(ff, "Q", 59 * kFemto);
  loadOutputOverPins(nand_, "ZN", 59 * kFemto);
  state_.cap_multipliers.start(state_, 1.0f);
  ASSERT_TRUE(state_.cap_multipliers.beta(driverVertex(ff, "Q")).has_value());
  ASSERT_FALSE(
      state_.cap_multipliers.beta(driverVertex(nand_, "ZN")).has_value());

  // The re-check updates parasitics incrementally, as inside global sizing.
  est::IncrementalParasiticsGuard guard(&ep_);
  std::vector<MovedGate> movers{
      {.inst = buf, .prev_cell = cell("BUF_X2"), .was_downsize = false},
      {.inst = aoi_, .prev_cell = cell("AOI21_X1"), .was_downsize = false}};
  ASSERT_TRUE(resizer_.replaceCell(buf, cell("BUF_X8")));
  ASSERT_TRUE(resizer_.replaceCell(aoi_, cell("AOI21_X4")));
  const CapRecheckStats stats = recheckMaxCapAfterSweep(state_, movers);
  EXPECT_EQ(stats.reverted, 1);
  EXPECT_EQ(std::string(db_network_->libertyCell(buf)->name()), "BUF_X8");
  EXPECT_EQ(aoiCell(), "AOI21_X1");
}

}  // namespace
}  // namespace rsz
