// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the objective-power model of the global sizer
// (lr/ObjectivePower.hh) on the Nangate45 library, which has VDD = 1.10 V,
// 7x7 internal power tables in fJ and capacitances in fF.
//
// The tests check that:
//   * Under the total objective, NAND2_X1 at a slew and load on its table
//     grid gives its leakage plus the internal and input switching power
//     worked by hand from its Liberty tables.
//   * Under the leakage objective the model returns the leakage it is given,
//     exactly.
//   * The duty helpers give the probabilities worked by hand for AOI21_X1,
//     and a register's state takes the duty of the output that drives it.
//   * On a small design, the model's internal power of every instance, at the
//     instance's own pin slews and loads, equals OpenSTA's per-instance
//     internal power. That reference does not depend on the model.
//   * Each input pin reads the density and supply voltage of the gate that
//     drives its net, and nothing for a net driven by a top-level input.
//   * The activity cache holds OpenSTA's activities and keeps them across a
//     cell swap, where they still equal a fresh OpenSTA read.
//   * The per-gate subproblem's power-phase filter skips every candidate whose
//     power is above the current cell's.

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "LRSubproblem.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "gtest/gtest.h"
#include "lr/LrState.hh"
#include "lr/ObjectivePower.hh"
#include "odb/db.h"
#include "rsz/GlobalSizingConfig.hh"
#include "sta/ClkNetwork.hh"
#include "sta/Delay.hh"
#include "sta/FuncExpr.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/Mode.hh"
#include "sta/NetworkClass.hh"
#include "sta/PortDirection.hh"
#include "sta/PowerClass.hh"
#include "sta/Scene.hh"
#include "sta/Transition.hh"
#include "tst/IntegratedFixture.h"

namespace rsz {
namespace {

constexpr float kRelTol = 1e-5f;
constexpr float kVdd = 1.10f;
// A point on the grid of both NAND2_X1 internal_power tables: the third
// input transition (0.0171859 ns) and the third load (3.709790 fF).
constexpr float kGridSlew = 0.0171859e-9f;
constexpr float kGridLoad = 3.709790e-15f;

PowerPin inputPin(const sta::LibertyCell* cell,
                  const char* name,
                  const float density,
                  const float duty,
                  const float slew,
                  const float fanin_density)
{
  PowerPin pin;
  pin.port = cell->findLibertyPort(name);
  pin.activity = {density, duty};
  pin.slew[sta::RiseFall::riseIndex()] = slew;
  pin.slew[sta::RiseFall::fallIndex()] = slew;
  pin.fanin_density = fanin_density;
  pin.fanin_voltage = kVdd;
  return pin;
}

PowerPin outputPin(const sta::LibertyCell* cell,
                   const char* name,
                   const float density,
                   const float duty,
                   const float load_cap)
{
  PowerPin pin;
  pin.port = cell->findLibertyPort(name);
  pin.activity = {density, duty};
  pin.load_cap = load_cap;
  return pin;
}

// NAND2_X1 with A1 at 2e8 transitions/s and duty 0.4, A2 at 1e8 and 0.7, and
// ZN at 1.5e8, at the grid point. Each input's net switches as often as the
// input.
GatePowerInputs nand2Gate(const sta::LibertyCell* nand2)
{
  GatePowerInputs gate;
  gate.pins = {inputPin(nand2, "A1", 2e8f, 0.4f, kGridSlew, 2e8f),
               inputPin(nand2, "A2", 1e8f, 0.7f, kGridSlew, 1e8f),
               outputPin(nand2, "ZN", 1.5e8f, 0.5f, kGridLoad)};
  return gate;
}

class ObjectivePowerTest : public tst::IntegratedFixture
{
 public:
  ObjectivePowerTest()
      : tst::IntegratedFixture(tst::IntegratedFixture::Technology::kNangate45,
                               "_main/src/rsz/test/")
  {
  }

 protected:
  sta::LibertyCell* libertyCell(const char* name)
  {
    sta::LibertyCell* cell = db_network_->findLibertyCell(name);
    EXPECT_NE(cell, nullptr) << name;
    return cell;
  }

  sta::Instance* instance(const char* name)
  {
    odb::dbInst* inst = block_->findInst(name);
    EXPECT_NE(inst, nullptr) << name;
    return db_network_->dbToSta(inst);
  }

  // Reads the netlist, runs the resizer and STA preambles global sizing runs
  // before its loop, and sets every arc multiplier in `state` to 1. `in` then
  // prices a gate's own arcs only, with no downsize guard.
  void setUpSubproblem(GlobalSizingConfig& config,
                       LrState& state,
                       LRSubproblem::SnapshotInputs& in)
  {
    readVerilogAndSetup("TestObjectivePower.v");
    resizer_.resizePreamble();
    sta_->findRequireds();
    sta_->checkCapacitancesPreamble(sta_->scenes());
    sta_->checkSlewsPreamble();
    sta_->checkFanoutPreamble();
    state.sta = sta_.get();
    state.graph = sta_->graph();
    state.logger = &logger_;
    state.config = &config;
    state.allocate();
    std::ranges::fill(state.lambda, 1.0f);
    in.lambda = state.lambda.data();
    in.lambda_size = static_cast<int>(state.lambda.size());
    in.cost.upstream_load = false;
    in.guard = GlobalSizingConfig::DownsizeGuard::kNone;
  }

  // Sets each input pin's slew and each output pin's load to the values
  // OpenSTA's power report reads for `inst` (Power::getSlew and the
  // delay calculator's load).
  void setActualSlewsAndLoads(const sta::Instance* inst,
                              const sta::Scene* scene,
                              GatePowerInputs& gate)
  {
    sta::Graph* graph = sta_->graph();
    const sta::ClkNetwork* clk_network = scene->mode()->clkNetwork();
    const sta::DcalcAPIndex ap
        = scene->dcalcAnalysisPtIndex(sta::MinMax::max());
    for (PowerPin& pin : gate.pins) {
      const sta::Pin* sta_pin
          = db_network_->findPin(inst, pin.port->name().c_str());
      if (pin.port->direction()->isAnyInput()) {
        const sta::Vertex* vertex = graph->pinLoadVertex(sta_pin);
        for (const sta::RiseFall* rf : sta::RiseFall::range()) {
          pin.slew[rf->index()]
              = clk_network->isIdealClock(sta_pin)
                    ? clk_network->idealClkSlew(sta_pin, rf, sta::MinMax::max())
                    : sta::delayAsFloat(graph->slew(vertex, rf, ap));
        }
      }
      if (pin.port->direction()->isAnyOutput()) {
        pin.load_cap = sta_->graphDelayCalc()->loadCap(
            sta_pin, scene, sta::MinMax::max());
      }
    }
  }
};

// ZN = !(A1 & A2). Both ZN entries are negative-unate, have no `when`
// condition and share one power pin, so
//   internal = d_ZN * (w_A1 * E_A1 + w_A2 * E_A2)
// with w_x = d_x * P(ZN sensitized to x) / sum over both inputs, where
// P(sensitized to A1) = duty(A2) and P(sensitized to A2) = duty(A1), and E_x
// the mean of the rise and fall table energies.
TEST_F(ObjectivePowerTest, Nand2OnTheTableGridMatchesTheHandWorkedValue)
{
  sta::LibertyCell* nand2 = libertyCell("NAND2_X1");
  const GatePowerInputs gate = nand2Gate(nand2);

  // d_A1 * duty(A2) = 1.4e8 and d_A2 * duty(A1) = 0.4e8: weights 7/9 and 2/9.
  // Row 3, column 3 of rise_power and fall_power (fJ):
  //   A1: (2.183357 + 0.174115) / 2 = 1.178736
  //   A2: (2.872034 + 0.137653) / 2 = 1.5048435
  const double internal
      = 1.5e8 * (7.0 / 9.0 * 1.178736e-15 + 2.0 / 9.0 * 1.5048435e-15);
  // C_in is the larger of the rise and fall capacitance: 1.599032 fF on A1
  // and 1.664199 fF on A2.
  const double switching
      = 0.5 * 1.10 * 1.10 * (1.599032e-15 * 2e8 + 1.664199e-15 * 1e8);
  // cell_leakage_power: 17.393360 nW.
  const double leakage = 17.393360e-9;

  ObjectivePower model;
  model.setTotalPower(true);
  EXPECT_NEAR(model.internalPower(nand2, gate), internal, internal * kRelTol);
  EXPECT_NEAR(
      model.inputSwitchingPower(nand2, gate), switching, switching * kRelTol);
  const double total = leakage + internal + switching;
  EXPECT_NEAR(model.power(nand2, leakage, gate), total, total * kRelTol);
}

TEST_F(ObjectivePowerTest, LeakageObjectiveReturnsTheLeakageExactly)
{
  sta::LibertyCell* nand2 = libertyCell("NAND2_X1");
  const GatePowerInputs gate = nand2Gate(nand2);
  const std::optional<float> leakage = resizer_.cellLeakage(nand2);
  ASSERT_TRUE(leakage.has_value());

  ObjectivePower model;
  EXPECT_FALSE(model.totalPower());
  EXPECT_EQ(model.power(nand2, *leakage, gate), *leakage);
  // An area-based stand-in for a cell without leakage data passes through
  // the same way.
  EXPECT_EQ(model.power(nand2, 3.5e-7f, gate), 3.5e-7f);
}

// AOI21_X1, ZN = !(A | (B1 & B2)), with duty(A) = 0.2, duty(B1) = 0.3 and
// duty(B2) = 0.6:
//   P(ZN)                  = (1 - 0.2) * (1 - 0.3 * 0.6) = 0.656
//   P(ZN sensitized to A)  = P(!(B1 & B2))               = 0.82
//   P(ZN sensitized to B1) = P(!A) * P(B2)               = 0.48
TEST_F(ObjectivePowerTest, Aoi21DutiesMatchTheHandWorkedProbabilities)
{
  sta::LibertyCell* aoi = libertyCell("AOI21_X1");
  GatePowerInputs gate;
  gate.pins = {inputPin(aoi, "A", 1e8f, 0.2f, kGridSlew, 0.0f),
               inputPin(aoi, "B1", 1e8f, 0.3f, kGridSlew, 0.0f),
               inputPin(aoi, "B2", 1e8f, 0.6f, kGridSlew, 0.0f),
               outputPin(aoi, "ZN", 1e8f, 0.5f, kGridLoad)};
  const sta::FuncExpr* zn = aoi->findLibertyPort("ZN")->function();

  EXPECT_NEAR(exprDuty(zn, gate), 0.656f, 1e-6f);
  EXPECT_NEAR(exprDiffDuty(zn, aoi->findLibertyPort("A"), gate), 0.82f, 1e-6f);
  EXPECT_NEAR(exprDiffDuty(zn, aoi->findLibertyPort("B1"), gate), 0.48f, 1e-6f);
}

// DFF_X1's Q and QN are the register states IQ and IQN. A condition on a
// state takes the duty of the output whose function is that state, and 0.5
// when the gate has no such output.
TEST_F(ObjectivePowerTest, RegisterStateTakesTheDutyOfItsOutput)
{
  sta::LibertyCell* dff = libertyCell("DFF_X1");
  const sta::FuncExpr* iq = dff->findLibertyPort("Q")->function();
  const sta::FuncExpr* iqn = dff->findLibertyPort("QN")->function();
  ASSERT_EQ(iq->op(), sta::FuncExpr::Op::port);
  ASSERT_TRUE(iq->port()->direction()->isInternal());

  GatePowerInputs gate;
  gate.pins = {inputPin(dff, "D", 1e8f, 0.5f, kGridSlew, 0.0f),
               inputPin(dff, "CK", 4e9f, 0.5f, kGridSlew, 0.0f),
               outputPin(dff, "Q", 5e7f, 0.35f, kGridLoad),
               outputPin(dff, "QN", 5e7f, 0.65f, kGridLoad)};
  EXPECT_FLOAT_EQ(exprDuty(iq, gate), 0.35f);
  EXPECT_FLOAT_EQ(exprDuty(iqn, gate), 0.65f);

  gate.pins.resize(2);  // D and CK only
  EXPECT_FLOAT_EQ(exprDuty(iq, gate), 0.5f);
}

TEST_F(ObjectivePowerTest, InternalPowerMatchesOpenStaPerInstance)
{
  readVerilogAndSetup("TestObjectivePower.v");
  const sta::Scene* scene = sta_->cmdScene();
  ObjectivePower model;
  model.cacheActivities(sta_.get(), scene);

  int checked = 0;
  for (odb::dbInst* db_inst : block_->getInsts()) {
    sta::Instance* inst = db_network_->dbToSta(db_inst);
    const std::string name = db_inst->getName();
    const float expected = sta_->power(inst, scene).internal();
    GatePowerInputs gate = model.gateInputs(inst);
    setActualSlewsAndLoads(inst, scene, gate);
    const float internal
        = model.internalPower(db_network_->libertyCell(inst), gate);
    EXPECT_GT(expected, 0.0f) << name;
    EXPECT_NEAR(internal, expected, expected * kRelTol) << name;
    ++checked;
  }
  EXPECT_EQ(checked, 9);
}

TEST_F(ObjectivePowerTest, InputPinsReadTheGateDrivingTheirNet)
{
  readVerilogAndSetup("TestObjectivePower.v");
  ObjectivePower model;
  model.cacheActivities(sta_.get(), sta_->cmdScene());

  // u_aoi/A is driven by u_nand/ZN; B1 and B2 by the top-level inputs c, d.
  const GatePowerInputs gate = model.gateInputs(instance("u_aoi"));
  const float driver_density
      = sta_->activity(db_network_->findPin(instance("u_nand"), "ZN"),
                       sta_->cmdScene())
            .density();
  ASSERT_GT(driver_density, 0.0f);
  int inputs = 0;
  for (const PowerPin& pin : gate.pins) {
    if (!pin.port->direction()->isInput()) {
      continue;
    }
    ++inputs;
    if (pin.port->name() == "A") {
      EXPECT_EQ(pin.fanin_density, driver_density);
      EXPECT_FLOAT_EQ(pin.fanin_voltage, kVdd);
    } else {
      EXPECT_EQ(pin.fanin_density, 0.0f) << pin.port->name();
      EXPECT_EQ(pin.fanin_voltage, 0.0f) << pin.port->name();
    }
  }
  EXPECT_EQ(inputs, 3);
}

TEST_F(ObjectivePowerTest, ActivityCacheKeepsOpenStaActivitiesAcrossASwap)
{
  readVerilogAndSetup("TestObjectivePower.v");
  const sta::Scene* scene = sta_->cmdScene();
  ObjectivePower model;
  model.cacheActivities(sta_.get(), scene);

  auto expect_cache_matches_sta = [&](sta::Instance* inst) {
    std::unique_ptr<sta::InstancePinIterator> pin_iter(
        db_network_->pinIterator(inst));
    while (pin_iter->hasNext()) {
      const sta::Pin* pin = pin_iter->next();
      if (db_network_->libertyPort(pin) == nullptr) {
        continue;
      }
      const PinActivity* cached = model.activity(pin);
      ASSERT_NE(cached, nullptr) << db_network_->pathName(pin);
      const sta::PwrActivity fresh = sta_->activity(pin, scene);
      EXPECT_FLOAT_EQ(cached->density, fresh.density())
          << db_network_->pathName(pin);
      EXPECT_FLOAT_EQ(cached->duty, fresh.duty()) << db_network_->pathName(pin);
    }
  };
  for (odb::dbInst* db_inst : block_->getInsts()) {
    expect_cache_matches_sta(db_network_->dbToSta(db_inst));
  }

  // OpenSTA drops its activities on the swap and propagates them again on the
  // next read; the cache keeps its entries for the swapped gate's pins.
  sta::Instance* nand = instance("u_nand");
  ASSERT_TRUE(resizer_.replaceCell(nand, libertyCell("NAND2_X2"), false));
  expect_cache_matches_sta(nand);
}

// The power-phase filter of the per-gate subproblem (Chinnery and Sharma, ISPD
// 2022, Table 2: "skip higher power libcells in power recovery"), under the
// leakage objective. u_nand is NAND2_X1, the NAND2 with the least leakage in
// Nangate45. With every multiplier at 1 and a large timing weight the cost is
// almost all delay, so without the filter the subproblem picks a stronger
// NAND2. With the filter every stronger NAND2 is above NAND2_X1's power and is
// skipped, so the gate keeps its cell, in the exhaustive scan and in Fast-OLR.
// A lower-power cell stays allowed: from NAND2_X4 with no timing weight both
// pick NAND2_X1.
TEST_F(ObjectivePowerTest, PowerPhaseFilterSkipsCandidatesAboveTheCurrentCell)
{
  GlobalSizingConfig config;
  LrState state;
  LRSubproblem::SnapshotInputs in;
  setUpSubproblem(config, state, in);
  sta::LibertyCell* nand2_x1 = libertyCell("NAND2_X1");
  const float leakage_x1 = *resizer_.cellLeakage(nand2_x1);
  for (sta::LibertyCell* cell : resizer_.getSwappableCells(nand2_x1)) {
    if (cell != nand2_x1) {
      EXPECT_GT(*resizer_.cellLeakage(cell), leakage_x1) << cell->name();
    }
  }

  LRSubproblem subproblem(&resizer_);
  sta::Instance* nand = instance("u_nand");
  const auto best_cell = [&](const bool filter, const float timing_weight) {
    in.power_phase_filter = filter;
    LRSubproblem::GateSnapshot snap;
    EXPECT_TRUE(subproblem.snapshot(nand, in, snap));
    return subproblem
        .evaluateSnapshot(snap, timing_weight, 1.0f, sta_->arcDelayCalc())
        .best_cell;
  };
  constexpr float kTimingWeight = 1e6f;

  sta::LibertyCell* unfiltered = best_cell(false, kTimingWeight);
  ASSERT_NE(unfiltered, nullptr);
  EXPECT_GT(*resizer_.cellLeakage(unfiltered), leakage_x1);
  EXPECT_EQ(best_cell(true, kTimingWeight), nullptr);

  in.move_set = GlobalSizingConfig::MoveSet::kSharmaFastOlr;
  in.fast_olr_active = true;
  sta::LibertyCell* fast_olr = best_cell(false, kTimingWeight);
  ASSERT_NE(fast_olr, nullptr);
  EXPECT_GT(*resizer_.cellLeakage(fast_olr), leakage_x1);
  EXPECT_EQ(best_cell(true, kTimingWeight), nullptr);

  in.move_set = GlobalSizingConfig::MoveSet::kFullLibrary;
  in.fast_olr_active = false;
  ASSERT_TRUE(resizer_.replaceCell(nand, libertyCell("NAND2_X4"), false));
  EXPECT_EQ(best_cell(false, 0.0f), nand2_x1);
  EXPECT_EQ(best_cell(true, 0.0f), nand2_x1);
}

// The filter compares total power under the total objective, and Fast-OLR
// steps over the cells it skips. u_buf (BUF_X2) drives a top-level output.
// Nangate45's buffers and clock buffers form one group, ranked by leakage, and
// at u_buf CLKBUF_X3 has less leakage than BUF_X2 but more total power. From
// BUF_X2 the Fast-OLR descent therefore tries CLKBUF_X3 first. Without the
// filter it costs more than BUF_X2, the descent stops there and the gate keeps
// its cell. With the filter CLKBUF_X3 is skipped as a higher-power cell, the
// descent goes on and reaches the cell the exhaustive scan picks, the
// lowest-power buffer CLKBUF_X1. A filter that compared leakage would keep
// CLKBUF_X3 and stop the descent at it, and so would one applied only to the
// descent's winners.
TEST_F(ObjectivePowerTest, PowerPhaseFilterComparesTotalPowerInFastOlr)
{
  GlobalSizingConfig config;
  LrState state;
  LRSubproblem::SnapshotInputs in;
  setUpSubproblem(config, state, in);
  ObjectivePower model;
  model.setTotalPower(true);
  model.cacheActivities(sta_.get(), sta_->cmdScene());
  in.objective_power = &model;

  sta::Instance* buf = instance("u_buf");
  sta::LibertyCell* buf_x2 = libertyCell("BUF_X2");
  sta::LibertyCell* clkbuf_x3 = libertyCell("CLKBUF_X3");
  sta::LibertyCell* clkbuf_x1 = libertyCell("CLKBUF_X1");
  ASSERT_EQ(db_network_->libertyCell(buf), buf_x2);
  const GatePowerInputs inputs
      = sizingPowerInputs(model, resizer_, buf, sta::MinMax::max());
  const auto total_power = [&](sta::LibertyCell* cell) {
    return model.power(cell, *resizer_.cellLeakage(cell), inputs);
  };
  ASSERT_LT(*resizer_.cellLeakage(clkbuf_x3), *resizer_.cellLeakage(buf_x2));
  ASSERT_GT(total_power(clkbuf_x3), total_power(buf_x2));
  for (sta::LibertyCell* cell : resizer_.getSwappableCells(buf_x2)) {
    if (cell != clkbuf_x1) {
      EXPECT_GT(total_power(cell), total_power(clkbuf_x1)) << cell->name();
    }
  }

  LRSubproblem subproblem(&resizer_);
  constexpr float kTimingWeight = 1e4f;
  const auto best_cell = [&](const bool filter) {
    in.power_phase_filter = filter;
    LRSubproblem::GateSnapshot snap;
    EXPECT_TRUE(subproblem.snapshot(buf, in, snap));
    return subproblem
        .evaluateSnapshot(snap, kTimingWeight, 1.0f, sta_->arcDelayCalc())
        .best_cell;
  };
  // The descent's first step does not improve: offered alone, CLKBUF_X3 does
  // not beat BUF_X2 at this weight.
  {
    LRSubproblem::GateSnapshot snap;
    ASSERT_TRUE(subproblem.snapshot(buf, in, snap));
    std::erase_if(snap.candidates, [&](const LRSubproblem::Candidate& c) {
      return c.cell != clkbuf_x3;
    });
    ASSERT_EQ(snap.candidates.size(), 1);
    EXPECT_EQ(
        subproblem
            .evaluateSnapshot(snap, kTimingWeight, 1.0f, sta_->arcDelayCalc())
            .best_cell,
        nullptr);
  }
  EXPECT_EQ(best_cell(false), clkbuf_x1);
  EXPECT_EQ(best_cell(true), clkbuf_x1);

  in.move_set = GlobalSizingConfig::MoveSet::kSharmaFastOlr;
  in.fast_olr_active = true;
  EXPECT_EQ(best_cell(false), nullptr);
  EXPECT_EQ(best_cell(true), clkbuf_x1);
}

}  // namespace
}  // namespace rsz
