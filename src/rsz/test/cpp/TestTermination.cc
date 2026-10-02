// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for termination, best-solution tracking and the duality-gap
// diagnostic, against hand-computed values, plus the per-preset settings of
// these and related options. The STA-free functions are in lr/Termination.hh
// and lr/BestTracker.hh; the STA side (saving and restoring a cell assignment,
// the per-iteration metrics) is covered by the global_sizing_termination
// integration test.

#include <cstddef>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "lr/BestTracker.hh"
#include "lr/LrState.hh"
#include "lr/Termination.hh"
#include "rsz/GlobalSizingConfig.hh"
#include "utl/Logger.h"

namespace rsz {
namespace {

using BestTrackerKind = GlobalSizingConfig::BestTrackerKind;
using DownsizeGuard = GlobalSizingConfig::DownsizeGuard;
using KktProjection = GlobalSizingConfig::KktProjection;
using LambdaSeed = GlobalSizingConfig::LambdaSeed;
using LambdaUpdate = GlobalSizingConfig::LambdaUpdate;
using MoveSet = GlobalSizingConfig::MoveSet;
using MuPolicy = GlobalSizingConfig::MuPolicy;
using OutputDrcVeto = GlobalSizingConfig::OutputDrcVeto;
using PowerObjective = GlobalSizingConfig::PowerObjective;
using Preset = GlobalSizingConfig::Preset;
using InitMode = GlobalSizingConfig::InitMode;
using ReimannSetpoint = GlobalSizingConfig::ReimannSetpoint;
using SweepEngineKind = GlobalSizingConfig::SweepEngineKind;
using TerminationKind = GlobalSizingConfig::TerminationKind;
using TimingScale = GlobalSizingConfig::TimingScale;
using Traversal = GlobalSizingConfig::Traversal;

////////////////////////////////////////////////////////////////
// relImprovement: (prev - cur) / |prev| on a lower-is-better metric

TEST(RelImprovement, HandComputed)
{
  // 100 -> 95 is a 5% improvement.
  EXPECT_FLOAT_EQ(relImprovement(100.0f, 95.0f), 0.05f);
  // Going the wrong way is a negative improvement.
  EXPECT_FLOAT_EQ(relImprovement(100.0f, 110.0f), -0.10f);
  EXPECT_FLOAT_EQ(relImprovement(100.0f, 100.0f), 0.0f);
  // No scale to be relative to.
  EXPECT_FLOAT_EQ(relImprovement(0.0f, 5.0f), 0.0f);
}

////////////////////////////////////////////////////////////////
// stagnantWindow (Sharma's rule; Mangiras' rule with require_tns)

// Sharma: "neither the average power, nor the minimum power solution found thus
// far, improve", so either one improving keeps the run going.
TEST(StagnantWindow, EitherAverageOrBestImprovingKeepsRunning)
{
  // Average improved 100 -> 98 (2%), best-so-far flat: not stagnant.
  EXPECT_FALSE(stagnantWindow(
      true, 100.0f, 98.0f, 90.0f, 90.0f, 0.0f, false, 0.0f, 0.0f));
  // Average flat, but the window found a new min (90 -> 88): not stagnant.
  EXPECT_FALSE(stagnantWindow(
      true, 100.0f, 100.0f, 90.0f, 88.0f, 0.0f, false, 0.0f, 0.0f));
  // Neither improved: stagnant.
  EXPECT_TRUE(stagnantWindow(
      true, 100.0f, 100.0f, 90.0f, 90.0f, 0.0f, false, 0.0f, 0.0f));
  // Both got worse: stagnant.
  EXPECT_TRUE(stagnantWindow(
      true, 100.0f, 103.0f, 90.0f, 90.0f, 0.0f, false, 0.0f, 0.0f));
}

// The first window has nothing to compare against and is never stagnant.
TEST(StagnantWindow, FirstWindowIsNeverStagnant)
{
  EXPECT_FALSE(stagnantWindow(
      false, 0.0f, 100.0f, 100.0f, 100.0f, 0.0f, false, 0.0f, 0.0f));
}

// Mangiras' 1% threshold: an improvement below the fraction does not count.
TEST(StagnantWindow, ImprovementBelowFracIsStagnant)
{
  // 100 -> 99.5 is 0.5%, below the 1% bar: stagnant.
  EXPECT_TRUE(stagnantWindow(
      true, 100.0f, 99.5f, 90.0f, 90.0f, 0.01f, false, 0.0f, 0.0f));
  // 100 -> 98 is 2%, above it: not stagnant.
  EXPECT_FALSE(stagnantWindow(
      true, 100.0f, 98.0f, 90.0f, 90.0f, 0.01f, false, 0.0f, 0.0f));
}

// Mangiras' TNS clause: stagnant power with TNS still improving by more than
// the fraction keeps the run going (the rule needs both to stagnate).
TEST(StagnantWindow, RequireTnsKeepsRunningWhileTnsImproves)
{
  // Power flat; |TNS| 10 -> 8 is a 20% improvement.
  EXPECT_FALSE(stagnantWindow(
      true, 100.0f, 100.0f, 90.0f, 90.0f, 0.01f, true, -10.0f, -8.0f));
  // Power flat; |TNS| 10 -> 9.95 is 0.5%, below the bar: both stagnant.
  EXPECT_TRUE(stagnantWindow(
      true, 100.0f, 100.0f, 90.0f, 90.0f, 0.01f, true, -10.0f, -9.95f));
  // Without the clause, the same TNS progress is irrelevant (Sharma).
  EXPECT_TRUE(stagnantWindow(
      true, 100.0f, 100.0f, 90.0f, 90.0f, 0.01f, false, -10.0f, -8.0f));
}

////////////////////////////////////////////////////////////////
// nearMetLatched: the near-met latch
//
// The latch depends on WNS, but it only enables other mechanisms (sharma's
// stagnation monitor, flach's local-slack veto); it never skips the loop or
// forces a stop. So a design that already meets timing latches it at once, as
// the cases below expect, which does not conflict with
// NoOptionGatesLoopEntryOnTiming.

// The set condition is "within near_met_gate_frac of the target", i.e.
// WNS >= -frac*T. On a 100 ps clock (T = 0.1) at frac = 0.01 the band is
// -0.001.
TEST(NearMetLatch, SetConditionWithinTheGateFractionOfTarget)
{
  const float T = 0.1f;
  const float frac = 0.01f;
  // Exactly at the 1% band (-0.01*T = -0.001): latched.
  EXPECT_TRUE(nearMetLatched(false, frac, T, -0.001f));
  // Just outside it: not latched.
  EXPECT_FALSE(nearMetLatched(false, frac, T, -0.0011f));
  // Fully met (positive WNS): latched.
  EXPECT_TRUE(nearMetLatched(false, frac, T, 0.02f));
  // Deep in violation: not latched, so the stagnation monitor stays inactive
  // while timing is still being fixed.
  EXPECT_FALSE(nearMetLatched(false, frac, T, -0.05f));
}

// A negative gate fraction disables gating: the run is near-met from the start,
// whatever the WNS. This default keeps mangiras' window rule and the
// flach/reimann veto always on.
TEST(NearMetLatch, DisabledGateIsAlwaysNearMet)
{
  EXPECT_TRUE(nearMetLatched(false, -1.0f, 0.1f, -5.0f));
}

// Once latched the phase is permanent for the run: a later WNS regression does
// not clear it (Sharma's power recovery does not fall back into timing).
TEST(NearMetLatch, PermanentOnceSet)
{
  EXPECT_TRUE(nearMetLatched(true, 0.01f, 0.1f, -5.0f));
}

// No clock (T <= 0): there is no target, so a run that has not latched stays
// unlatched. The disabled-gate and already-latched cases are checked first and
// still return true.
TEST(NearMetLatch, NoClockNeverLatchesButShortcutsWin)
{
  EXPECT_FALSE(nearMetLatched(false, 0.01f, 0.0f, 0.0f));
  EXPECT_TRUE(nearMetLatched(false, -1.0f, 0.0f, -1.0f));
  EXPECT_TRUE(nearMetLatched(true, 0.01f, 0.0f, 0.0f));
}

// A fresh LrState is not near-met (allocate() clears it for each run), so the
// first iteration always evaluates the condition.
TEST(NearMetLatch, FreshStateIsNotNearMet)
{
  LrState state;
  EXPECT_FALSE(state.near_met);
}

////////////////////////////////////////////////////////////////
// PureCap: stops only at the iteration cap

// pure_cap ignores both conditions fixed_iters stops on, consecutive rejected
// passes and passes with no moves, both checked after the sweep. It reads no
// state at all, which the null-STA LrState here demonstrates.
TEST(PureCap, IgnoresEveryLegacyEarlyExit)
{
  GlobalSizingConfig config;
  config.termination = TerminationKind::kPureCap;
  std::unique_ptr<Termination> cap = makeTermination(config);
  EXPECT_FALSE(cap->needsMetrics());

  LrState state;
  state.config = &config;
  // pure_cap's stopBeforeSweep returns false without ever touching state.sta
  // (null here).
  EXPECT_FALSE(cap->stopBeforeSweep(state, 0));
  EXPECT_FALSE(cap->stopBeforeSweep(state, 5));
  // Repeated rejected passes with no moves: never stops.
  const IterMetrics met{
      .wns = 0.5f, .tns = 0.0f, .leakage = 100.0f, .power = 100.0f};
  for (int iter = 0; iter < 6; ++iter) {
    EXPECT_FALSE(cap->stopAfterSweep(
        state, iter, /*reject=*/true, /*no_benefit=*/true, met));
  }
}

// No termination option may skip the loop because timing is already met.
// Stopping before the first sweep once WNS meets setup_slack_margin would make
// global sizing do nothing on any design that already meets timing, so it
// would never recover leakage there.
//
// The test checks two things:
//   * every option returns false from stopBeforeSweep(), at iteration 0 and
//     later. stopBeforeSweep() is a no-op in the base class and only
//     threshold_battery overrides it, so this also checks that override.
//   * this holds with met timing in LrState. state.sta is null, so an exit
//     that queried STA would crash, but an exit could also read the timing
//     values LrState already holds, so those are set to a clearly met design.
// The global_sizing_met_recovery integration test covers the full flow.
TEST(Termination, NoOptionGatesLoopEntryOnTiming)
{
  for (const TerminationKind kind : {TerminationKind::kFixedIters,
                                     TerminationKind::kStagnationWindows,
                                     TerminationKind::kThresholdBattery,
                                     TerminationKind::kPureCap}) {
    GlobalSizingConfig config;
    config.termination = kind;
    // A margin that the met design below clears easily.
    config.setup_slack_margin = 0.0f;
    std::unique_ptr<Termination> term = makeTermination(config);

    LrState state;
    state.config = &config;
    // A clearly met design, in the fields a pre-sweep exit could read without
    // dereferencing anything.
    state.wns_init = 5.0f;
    state.T = 1.0f;
    state.near_met = true;

    EXPECT_FALSE(term->stopBeforeSweep(state, 0))
        << "termination=" << toString(kind)
        << " must enter its first sweep whatever the timing state";
    EXPECT_FALSE(term->stopBeforeSweep(state, 7))
        << "termination=" << toString(kind)
        << " must not acquire a pre-sweep stop at a later iteration";
  }
}

// On the same sequence of rejected passes, fixed_iters stops at the third
// consecutive rejection and pure_cap never does.
TEST(PureCap, WhereFixedItersStopsPureCapKeepsGoing)
{
  utl::Logger logger;
  GlobalSizingConfig config;
  LrState state;
  state.config = &config;
  state.logger = &logger;
  const IterMetrics met{
      .wns = -0.2f, .tns = -1.0f, .leakage = 100.0f, .power = 100.0f};

  FixedItersTermination fixed;
  EXPECT_FALSE(fixed.stopAfterSweep(state, 0, true, false, met));
  EXPECT_FALSE(fixed.stopAfterSweep(state, 1, true, false, met));
  EXPECT_TRUE(fixed.stopAfterSweep(state, 2, true, false, met));  // 3rd reject

  PureCapTermination cap;
  EXPECT_FALSE(cap.stopAfterSweep(state, 0, true, false, met));
  EXPECT_FALSE(cap.stopAfterSweep(state, 1, true, false, met));
  EXPECT_FALSE(cap.stopAfterSweep(state, 2, true, false, met));
}

////////////////////////////////////////////////////////////////
// StagnationGating: the near-met latch enables the stagnation monitor

// A config with the window-1 stagnation rule (each iteration vs its
// predecessor), so a flat-power sequence goes stagnant on the second active
// call and hand-computation is trivial.
GlobalSizingConfig window1StagnationConfig()
{
  GlobalSizingConfig config;
  config.termination = TerminationKind::kStagnationWindows;
  config.stagnation_window = 1;
  config.stagnation_count = 1;
  config.stagnation_improve_frac = 0.01f;
  config.stagnation_require_tns = false;
  return config;
}

// While the run is not near-met the monitor is inactive: it neither stops nor
// accumulates window state, so a run that is still fixing timing (with flat
// power, which would count as stagnant) is not stopped early. Sharma et al.
// apply this early exit only once timing is almost met.
TEST(StagnationGating, InactiveWhileNotNearMet)
{
  const GlobalSizingConfig config = window1StagnationConfig();
  LrState state;
  state.config = &config;
  state.near_met = false;

  StagnationWindowsTermination term;
  const IterMetrics flat{
      .wns = -0.2f, .tns = -5.0f, .leakage = 100.0f, .power = 100.0f};
  for (int iter = 0; iter < 20; ++iter) {
    EXPECT_FALSE(term.stopAfterSweep(state, iter, false, false, flat));
  }
}

// Once near-met the monitor runs the normal rule: the first window is never
// stagnant (nothing to compare), the second flat window is, and count = 1 stops
// the run there.
TEST(StagnationGating, ActiveAfterTheLatch)
{
  utl::Logger logger;
  const GlobalSizingConfig config = window1StagnationConfig();
  LrState state;
  state.config = &config;
  state.logger = &logger;
  state.near_met = true;

  StagnationWindowsTermination term;
  const IterMetrics flat{
      .wns = -0.001f, .tns = -0.01f, .leakage = 100.0f, .power = 100.0f};
  EXPECT_FALSE(
      term.stopAfterSweep(state, 0, false, false, flat));          // 1st window
  EXPECT_TRUE(term.stopAfterSweep(state, 1, false, false, flat));  // stagnant
}

// The windows count from the start of power recovery, not from iteration 0: a
// run that violates for several iterations (monitor inactive) and then latches
// near-met starts its first window at the latch. The inactive phase must leave
// no window state behind.
TEST(StagnationGating, WindowsCountFromTheLatchNotIterationZero)
{
  utl::Logger logger;
  const GlobalSizingConfig config = window1StagnationConfig();
  LrState state;
  state.config = &config;
  state.logger = &logger;

  StagnationWindowsTermination term;
  const IterMetrics flat{
      .wns = -0.2f, .tns = -5.0f, .leakage = 100.0f, .power = 100.0f};
  // Iterations 0-2: timing not yet met, monitor inactive (no accumulation).
  state.near_met = false;
  for (int iter = 0; iter < 3; ++iter) {
    EXPECT_FALSE(term.stopAfterSweep(state, iter, false, false, flat));
  }
  // Latch at iteration 3: this is the monitor's first window (not stagnant),
  // and only the flat window after it stops the run.
  state.near_met = true;
  EXPECT_FALSE(term.stopAfterSweep(state, 3, false, false, flat));
  EXPECT_TRUE(term.stopAfterSweep(state, 4, false, false, flat));
}

// The windows measure the metrics' power, which is total power under the
// total-power objective, not leakage. Window 1 compares each iteration with
// the one before.
TEST(StagnationGating, WindowsReadPowerNotLeakage)
{
  utl::Logger logger;
  const GlobalSizingConfig config = window1StagnationConfig();
  LrState state;
  state.config = &config;
  state.logger = &logger;
  state.near_met = true;
  const auto metrics = [](const float leakage, const float power) {
    return IterMetrics{
        .wns = -0.001f, .tns = -0.01f, .leakage = leakage, .power = power};
  };

  // Power 200 -> 150 is a 25% improvement (> 1%) while leakage is flat: the
  // run goes on.
  StagnationWindowsTermination improving;
  EXPECT_FALSE(improving.stopAfterSweep(
      state, 0, false, false, metrics(100.0f, 200.0f)));
  EXPECT_FALSE(improving.stopAfterSweep(
      state, 1, false, false, metrics(100.0f, 150.0f)));

  // Flat power is stagnant although leakage 100 -> 50 improves by 50%.
  StagnationWindowsTermination stagnant;
  EXPECT_FALSE(
      stagnant.stopAfterSweep(state, 0, false, false, metrics(100.0f, 200.0f)));
  EXPECT_TRUE(
      stagnant.stopAfterSweep(state, 1, false, false, metrics(50.0f, 200.0f)));
}

////////////////////////////////////////////////////////////////
// thresholdBatteryStop (Chinnery and Sharma, ISPD 2022)

GlobalSizingConfig batteryConfig()
{
  GlobalSizingConfig config;
  config.termination = TerminationKind::kThresholdBattery;
  return config;  // the paper's constants are the struct defaults
}

// TNS within 10% of T (T = 1.0 -> |TNS| < 0.1) ends the timing phase.
TEST(ThresholdBattery, TnsTarget)
{
  const GlobalSizingConfig config = batteryConfig();
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kTiming,
                                 -0.05f,
                                 -0.5f,
                                 1.0f,
                                 100.0f,
                                 false,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 config),
            StopReason::kTnsTarget);
  // At the bar, not under it: no exit from this rule (WNS is far off too).
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kTiming,
                                 -0.10f,
                                 -0.5f,
                                 1.0f,
                                 100.0f,
                                 false,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 config),
            StopReason::kNone);
}

// WNS within 1% of T (|WNS| < 0.01) ends the timing phase even with TNS off.
TEST(ThresholdBattery, WnsTarget)
{
  const GlobalSizingConfig config = batteryConfig();
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kTiming,
                                 -5.0f,
                                 -0.005f,
                                 1.0f,
                                 100.0f,
                                 false,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 config),
            StopReason::kWnsTarget);
  // A positive WNS means no violation at all: also inside the target.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kTiming,
                                 -5.0f,
                                 0.2f,
                                 1.0f,
                                 100.0f,
                                 false,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 config),
            StopReason::kWnsTarget);
}

// TNS improving by less than 10% over the trailing window ends the timing
// phase.
TEST(ThresholdBattery, TnsStall)
{
  const GlobalSizingConfig config = batteryConfig();
  // |TNS| 10 -> 9.5 over the window = 5% < 10%: stalled.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kTiming,
                                 -9.5f,
                                 -0.5f,
                                 1.0f,
                                 100.0f,
                                 true,
                                 -10.0f,
                                 200.0f,
                                 0.0f,
                                 0.0f,
                                 config),
            StopReason::kTnsStall);
  // |TNS| 10 -> 8 = 20%: still making progress (power is too, 200 -> 100).
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kTiming,
                                 -8.0f,
                                 -0.5f,
                                 1.0f,
                                 100.0f,
                                 true,
                                 -10.0f,
                                 200.0f,
                                 0.0f,
                                 0.0f,
                                 config),
            StopReason::kNone);
  // With no history the improvement rules cannot fire.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kTiming,
                                 -9.5f,
                                 -0.5f,
                                 1.0f,
                                 100.0f,
                                 false,
                                 -10.0f,
                                 200.0f,
                                 0.0f,
                                 0.0f,
                                 config),
            StopReason::kNone);
}

// In the power phase, power improving by less than 1% over the window ends the
// run.
TEST(ThresholdBattery, PowerStall)
{
  const GlobalSizingConfig config = batteryConfig();
  // TNS still improving 20%, but power 100 -> 99.5 is 0.5% < 1%. The
  // power-phase cases pass tns_at_handover == tns (timing unchanged since the
  // phase change), so the TNS-degradation exit does not fire and each case
  // tests only the rule it is named for.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kPower,
                                 -8.0f,
                                 -0.5f,
                                 1.0f,
                                 99.5f,
                                 true,
                                 -10.0f,
                                 100.0f,
                                 -8.0f,
                                 0.0f,
                                 config),
            StopReason::kPowerStall);
}

// Chinnery's power exits apply only in the power phase. While the design is
// still improving timing, a power plateau must not end the run. If all
// clauses were simply OR-ed together, this state would stop the loop after a
// few iterations, long before convergence.
TEST(ThresholdBattery, TimingPhaseIgnoresThePowerStall)
{
  const GlobalSizingConfig config = batteryConfig();
  // Power flat (100 -> 100, 0% < 1%) but TNS improving 20%: keep going.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kTiming,
                                 -8.0f,
                                 -0.5f,
                                 1.0f,
                                 100.0f,
                                 true,
                                 -10.0f,
                                 100.0f,
                                 0.0f,
                                 0.0f,
                                 config),
            StopReason::kNone);
  // Power getting worse during timing improvement is still not an exit; the
  // timing phase is allowed to spend power.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kTiming,
                                 -8.0f,
                                 -0.5f,
                                 1.0f,
                                 150.0f,
                                 true,
                                 -10.0f,
                                 100.0f,
                                 0.0f,
                                 0.0f,
                                 config),
            StopReason::kNone);
}

// Conversely, the timing targets apply only in the timing phase and must not
// fire again once the loop has moved to power reduction (they are normally
// true there, since the power phase starts because timing was met).
TEST(ThresholdBattery, PowerPhaseIgnoresTheTimingTargets)
{
  const GlobalSizingConfig config = batteryConfig();
  // TNS and WNS both inside their targets, power still improving 20%.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kPower,
                                 -0.01f,
                                 -0.001f,
                                 1.0f,
                                 80.0f,
                                 true,
                                 -0.02f,
                                 100.0f,
                                 -0.01f,
                                 0.0f,
                                 config),
            StopReason::kNone);
  // A TNS stall does not end the power phase either.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kPower,
                                 -9.5f,
                                 -0.5f,
                                 1.0f,
                                 80.0f,
                                 true,
                                 -10.0f,
                                 100.0f,
                                 -9.5f,
                                 0.0f,
                                 config),
            StopReason::kNone);
}

// Chinnery's power exit measures "power reduction ... over the last 3
// iterations" of the power phase. The termination strategy restarts the
// improvement window at the phase change; this checks the other half: with no
// window yet (right after the restart), the power phase cannot stop.
//
// Without the restart the window would span the phase change and compare
// against a timing-phase power. The timing phase raises power (it upsizes
// to fix timing), so the comparison would show a negative improvement and
// trigger kPowerStall on the first power-phase iteration. The power phase
// would then always last one iteration, while the paper reports an average of
// 10.
TEST(ThresholdBattery, PowerPhaseNeedsItsOwnWindowBeforeItCanStall)
{
  const GlobalSizingConfig config = batteryConfig();
  // have_window = false: the power phase has no history of its own yet. Even
  // with power that rose 100 -> 150 (what a window spanning the phase change
  // would see), no exit fires.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kPower,
                                 -0.01f,
                                 -0.001f,
                                 1.0f,
                                 150.0f,
                                 false,
                                 -0.02f,
                                 100.0f,
                                 -0.01f,
                                 0.0f,
                                 config),
            StopReason::kNone);
  // Once it has its own window, rising power is a stall (no reduction).
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kPower,
                                 -0.01f,
                                 -0.001f,
                                 1.0f,
                                 150.0f,
                                 true,
                                 -0.02f,
                                 100.0f,
                                 -0.01f,
                                 0.0f,
                                 config),
            StopReason::kPowerStall);
}

// Chinnery's second power-phase exit: the phase also ends "if TNS degrades
// worse than it was at the end of the timing phase". It stops the power phase
// from giving up timing it cannot recover. The paper reports that this exit
// fires in about 24% of post-CTS and 36% of pre-CTS runs.
TEST(ThresholdBattery, TnsDegradedPastTheHandover)
{
  const GlobalSizingConfig config = batteryConfig();
  // TNS was -5 at the phase change; the power phase has pushed it to -5.5.
  // Power is improving 20% (100 -> 80), so nothing else would stop this run.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kPower,
                                 -5.5f,
                                 -0.5f,
                                 1.0f,
                                 80.0f,
                                 true,
                                 -5.0f,
                                 100.0f,
                                 -5.0f,
                                 0.0f,
                                 config),
            StopReason::kTnsDegraded);
  // No tolerance is specified, and an exactly-unchanged TNS is not a
  // degradation, so the boundary value does not fire.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kPower,
                                 -5.0f,
                                 -0.5f,
                                 1.0f,
                                 80.0f,
                                 true,
                                 -5.0f,
                                 100.0f,
                                 -5.0f,
                                 0.0f,
                                 config),
            StopReason::kNone);
  // TNS better than at the phase change is the normal case.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kPower,
                                 -4.0f,
                                 -0.5f,
                                 1.0f,
                                 80.0f,
                                 true,
                                 -5.0f,
                                 100.0f,
                                 -5.0f,
                                 0.0f,
                                 config),
            StopReason::kNone);
}

// The exit needs no window: its reference is the TNS at the phase change, not
// a trailing average, so it can fire on the first power-phase iteration, as
// the paper's criterion can. (kPowerStall cannot fire there.)
TEST(ThresholdBattery, TnsDegradationFiresWithoutAWindow)
{
  const GlobalSizingConfig config = batteryConfig();
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kPower,
                                 -5.5f,
                                 -0.5f,
                                 1.0f,
                                 80.0f,
                                 false,
                                 0.0f,
                                 0.0f,
                                 -5.0f,
                                 0.0f,
                                 config),
            StopReason::kTnsDegraded);
}

// This exit applies only in the power phase. During the timing phase there is
// no phase-change TNS yet (the strategy only sets tns_at_handover_ when the
// phase changes), so a timing-phase call must ignore that argument.
TEST(ThresholdBattery, TimingPhaseIgnoresTheDegradationExit)
{
  const GlobalSizingConfig config = batteryConfig();
  // TNS -5 -> -5.5 (worse) while still improving less than 10%: the timing
  // phase's own kTnsStall is the verdict, never the degradation exit.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kTiming,
                                 -5.5f,
                                 -0.5f,
                                 1.0f,
                                 80.0f,
                                 true,
                                 -5.0f,
                                 100.0f,
                                 -5.0f,
                                 0.0f,
                                 config),
            StopReason::kTnsStall);
}

// Precedence: the degradation exit is more specific, so it is reported ahead
// of a simultaneous power stall (a run whose timing got worse did not merely
// run out of power to recover), but the wall-clock cap outranks both.
TEST(ThresholdBattery, DegradationOutranksThePowerStallButNotTheWallClock)
{
  const GlobalSizingConfig config = batteryConfig();
  // Both hold: TNS worse than the handover AND power flat.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kPower,
                                 -5.5f,
                                 -0.5f,
                                 1.0f,
                                 100.0f,
                                 true,
                                 -5.0f,
                                 100.0f,
                                 -5.0f,
                                 0.0f,
                                 config),
            StopReason::kTnsDegraded);
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kPower,
                                 -5.5f,
                                 -0.5f,
                                 1.0f,
                                 100.0f,
                                 true,
                                 -5.0f,
                                 100.0f,
                                 -5.0f,
                                 259201.0f,
                                 config),
            StopReason::kWallClock);
}

// === threshold_battery run reports (RSZ-0450/0451) ==========================
//
// These tests drive the termination strategy rather than the STA-free function,
// because they check what it logs. Scripts read the timing/power phase split
// from these two messages, so a missing message, or a run total reported
// under a per-phase label, would be misread.
//
// T = 0 disables the timing targets (there is no clock period), which leaves
// the TNS-improvement rule as the only timing-phase criterion and makes these
// sequences easy to follow.
TEST(ThresholdBatteryRecords, StopRecordCarriesThePhaseSplit)
{
  utl::Logger logger;
  const GlobalSizingConfig config = batteryConfig();
  LrState state;
  state.config = &config;
  state.logger = &logger;
  state.T = 0.0f;

  ThresholdBatteryTermination term;
  const IterMetrics flat{
      .wns = -1.0f, .tns = -100.0f, .leakage = 100.0f, .power = 100.0f};
  logger.redirectStringBegin();
  // The driver calls stopBeforeSweep first; that is where the strategy records
  // the start time for its 72 h wall-clock cap.
  term.stopBeforeSweep(state, 0);
  // Iterations 0-2 fill the window; iteration 3 is the first with a comparison,
  // and a flat TNS is a 0% improvement -> the timing phase ends there.
  for (int iter = 0; iter < 4; ++iter) {
    EXPECT_FALSE(term.stopAfterSweep(state, iter, false, false, flat));
  }
  // Power phase: its window restarts, so iterations 4-6 fill it and iteration 7
  // is the first that can stall. Flat power is a 0% reduction.
  for (int iter = 4; iter < 7; ++iter) {
    EXPECT_FALSE(term.stopAfterSweep(state, iter, false, false, flat));
  }
  EXPECT_TRUE(term.stopAfterSweep(state, 7, false, false, flat));
  const std::string out = logger.redirectStringEnd();

  EXPECT_NE(out.find("RSZ-0450"), std::string::npos) << out;
  EXPECT_NE(out.find("timing phase ended after 4 iteration(s)"),
            std::string::npos)
      << out;
  // 8 is the run total and the split is stated, so a reader (or a script)
  // cannot take "8" for the power phase's length.
  EXPECT_NE(out.find("stopped after 8 iteration(s) (4 timing + 4 power)"),
            std::string::npos)
      << out;
  EXPECT_NE(out.find("power improvement stalled"), std::string::npos) << out;

  // The end-of-loop hook does not double-report a run that already stopped.
  logger.redirectStringBegin();
  term.reportRunEnd(state);
  EXPECT_EQ(logger.redirectStringEnd().find("RSZ-0451"), std::string::npos);
}

// A run that ends at max_iterations meets no battery criterion, so
// stopAfterSweep never reports it, yet those are the runs whose convergence is
// most interesting. The end-of-loop hook reports them, so a missing report can
// only mean that this termination option was not selected.
TEST(ThresholdBatteryRecords, CapExitStillPublishesARecord)
{
  utl::Logger logger;
  const GlobalSizingConfig config = batteryConfig();
  LrState state;
  state.config = &config;
  state.logger = &logger;
  state.T = 0.0f;

  ThresholdBatteryTermination term;
  ThresholdBatteryTermination never_ran;
  logger.redirectStringBegin();
  term.stopBeforeSweep(state, 0);
  // TNS improving 20% per iteration: the timing phase never ends, so the run
  // can only stop at the cap.
  float tns = -100.0f;
  for (int iter = 0; iter < 5; ++iter) {
    const IterMetrics m{
        .wns = -1.0f, .tns = tns, .leakage = 100.0f, .power = 100.0f};
    EXPECT_FALSE(term.stopAfterSweep(state, iter, false, false, m));
    tns *= 0.8f;
  }
  const std::string during = logger.redirectStringEnd();
  EXPECT_EQ(during.find("RSZ-0451"), std::string::npos) << during;

  logger.redirectStringBegin();
  term.reportRunEnd(state);
  const std::string out = logger.redirectStringEnd();
  EXPECT_NE(out.find("RSZ-0451"), std::string::npos) << out;
  // Never handed over, so the whole run is the timing phase.
  EXPECT_NE(out.find("stopped after 5 iteration(s) (5 timing + 0 power)"),
            std::string::npos)
      << out;
  EXPECT_NE(out.find("iteration cap reached"), std::string::npos) << out;

  // A strategy that never saw a sweep (max_iterations <= 0, or a loop that
  // never ran) has nothing to report and must not invent a record.
  logger.redirectStringBegin();
  never_ran.reportRunEnd(state);
  EXPECT_EQ(logger.redirectStringEnd().find("RSZ-0451"), std::string::npos);
}

// The driver reads inPowerPhase() after each stop check to tell the
// sharma_arc_slack update and the power-phase filter which phase the run is
// in. It is false through the timing phase, turns true at the stop check that
// hands over (iteration 3 of the flat sequence in
// StopRecordCarriesThePhaseSplit), and stays true until the run stops.
TEST(ThresholdBatteryRecords, PowerPhaseQueryFlipsAtTheHandoverOnly)
{
  utl::Logger logger;
  const GlobalSizingConfig config = batteryConfig();
  LrState state;
  state.config = &config;
  state.logger = &logger;
  state.T = 0.0f;

  ThresholdBatteryTermination term;
  const IterMetrics flat{
      .wns = -1.0f, .tns = -100.0f, .leakage = 100.0f, .power = 100.0f};
  EXPECT_FALSE(term.inPowerPhase());
  term.stopBeforeSweep(state, 0);
  EXPECT_FALSE(term.inPowerPhase());
  for (int iter = 0; iter < 3; ++iter) {
    EXPECT_FALSE(term.stopAfterSweep(state, iter, false, false, flat));
    EXPECT_FALSE(term.inPowerPhase()) << "iteration " << iter;
  }
  EXPECT_FALSE(term.stopAfterSweep(state, 3, false, false, flat));
  EXPECT_TRUE(term.inPowerPhase());
  for (int iter = 4; iter < 7; ++iter) {
    EXPECT_FALSE(term.stopAfterSweep(state, iter, false, false, flat));
    EXPECT_TRUE(term.inPowerPhase()) << "iteration " << iter;
  }
  EXPECT_TRUE(term.stopAfterSweep(state, 7, false, false, flat));
  EXPECT_TRUE(term.inPowerPhase());
}

// The other rules have no phases, so the query keeps its default.
TEST(Termination, OnlyTheBatteryReportsAPowerPhase)
{
  const IterMetrics flat{
      .wns = -1.0f, .tns = -100.0f, .leakage = 100.0f, .power = 100.0f};
  for (const TerminationKind kind : {TerminationKind::kFixedIters,
                                     TerminationKind::kStagnationWindows,
                                     TerminationKind::kPureCap}) {
    utl::Logger logger;
    GlobalSizingConfig config;
    config.termination = kind;
    LrState state;
    state.config = &config;
    state.logger = &logger;
    state.near_met = true;
    std::unique_ptr<Termination> term = makeTermination(config);
    for (int iter = 0; iter < 12; ++iter) {
      term->stopBeforeSweep(state, iter);
      const bool stop = term->stopAfterSweep(state, iter, false, false, flat);
      EXPECT_FALSE(term->inPowerPhase())
          << toString(kind) << " iteration " << iter;
      if (stop) {
        break;
      }
    }
  }
}

// The power-phase exit measures the metrics' power, which is total power under
// the total-power objective, not leakage. As above, T = 0 and a flat TNS end
// the timing phase at iteration 3; iteration 7 is the first power-phase
// iteration with a window, and compares with iteration 4.
TEST(ThresholdBatteryRecords, PowerStallReadsPowerNotLeakage)
{
  utl::Logger logger;
  const GlobalSizingConfig config = batteryConfig();
  LrState state;
  state.config = &config;
  state.logger = &logger;
  state.T = 0.0f;
  const auto metrics = [](const float leakage, const float power) {
    return IterMetrics{
        .wns = -1.0f, .tns = -100.0f, .leakage = leakage, .power = power};
  };

  // Power 200 -> 170 over the window is a 15% reduction (> 1%) while leakage
  // is flat: no stall.
  ThresholdBatteryTermination falling_power;
  falling_power.stopBeforeSweep(state, 0);
  for (int iter = 0; iter < 4; ++iter) {
    EXPECT_FALSE(falling_power.stopAfterSweep(
        state, iter, false, false, metrics(100.0f, 200.0f)));
  }
  const float power[] = {200.0f, 190.0f, 180.0f, 170.0f};
  for (int iter = 4; iter < 8; ++iter) {
    EXPECT_FALSE(falling_power.stopAfterSweep(
        state, iter, false, false, metrics(100.0f, power[iter - 4])));
  }

  // Flat power stalls at iteration 7 although leakage 100 -> 70 is a 30%
  // reduction.
  ThresholdBatteryTermination flat_power;
  flat_power.stopBeforeSweep(state, 0);
  for (int iter = 0; iter < 4; ++iter) {
    EXPECT_FALSE(flat_power.stopAfterSweep(
        state, iter, false, false, metrics(100.0f, 200.0f)));
  }
  const float leakage[] = {100.0f, 90.0f, 80.0f, 70.0f};
  for (int iter = 4; iter < 7; ++iter) {
    EXPECT_FALSE(flat_power.stopAfterSweep(
        state, iter, false, false, metrics(leakage[iter - 4], 200.0f)));
  }
  logger.redirectStringBegin();
  EXPECT_TRUE(flat_power.stopAfterSweep(
      state, 7, false, false, metrics(70.0f, 200.0f)));
  const std::string out = logger.redirectStringEnd();
  EXPECT_NE(out.find("power improvement stalled"), std::string::npos) << out;
}

// RSZ-0450 and RSZ-0451 report leakage under either objective. Under the
// total-power objective, which the power exit then reads, they also report
// total power; under leakage they report leakage only.
TEST(ThresholdBatteryRecords, TotalPowerObjectiveAlsoReportsTotalPower)
{
  utl::Logger logger;
  const IterMetrics flat{
      .wns = -1.0f, .tns = -100.0f, .leakage = 100.0f, .power = 250.0f};
  // A flat run hands over at iteration 3 and stalls at iteration 7 (see
  // StopRecordCarriesThePhaseSplit).
  const auto run = [&](const PowerObjective objective) {
    GlobalSizingConfig config = batteryConfig();
    config.power_objective = objective;
    LrState state;
    state.config = &config;
    state.logger = &logger;
    state.T = 0.0f;
    ThresholdBatteryTermination term;
    logger.redirectStringBegin();
    term.stopBeforeSweep(state, 0);
    for (int iter = 0; iter < 8; ++iter) {
      term.stopAfterSweep(state, iter, false, false, flat);
    }
    return logger.redirectStringEnd();
  };

  const std::string total_log = run(PowerObjective::kTotal);
  EXPECT_NE(total_log.find("RSZ-0450"), std::string::npos) << total_log;
  EXPECT_NE(total_log.find("RSZ-0451"), std::string::npos) << total_log;
  const std::string total_fields
      = "[tns=-100 wns=-1 leakage=100 total_power=250 T=0].";
  const size_t total_first = total_log.find(total_fields);
  ASSERT_NE(total_first, std::string::npos) << total_log;
  EXPECT_NE(total_log.find(total_fields, total_first + 1), std::string::npos)
      << total_log;

  const std::string leakage_log = run(PowerObjective::kLeakage);
  EXPECT_NE(leakage_log.find("RSZ-0450"), std::string::npos) << leakage_log;
  EXPECT_NE(leakage_log.find("RSZ-0451"), std::string::npos) << leakage_log;
  EXPECT_EQ(leakage_log.find("total_power"), std::string::npos) << leakage_log;
  const std::string leakage_fields = "[tns=-100 wns=-1 leakage=100 T=0].";
  const size_t leakage_first = leakage_log.find(leakage_fields);
  ASSERT_NE(leakage_first, std::string::npos) << leakage_log;
  EXPECT_NE(leakage_log.find(leakage_fields, leakage_first + 1),
            std::string::npos)
      << leakage_log;
}

// The wall-clock cap is a hard safety limit, so it fires in either phase.
TEST(ThresholdBattery, WallClockCap)
{
  const GlobalSizingConfig config = batteryConfig();
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kPower,
                                 -8.0f,
                                 -0.5f,
                                 1.0f,
                                 50.0f,
                                 true,
                                 -10.0f,
                                 100.0f,
                                 -8.0f,
                                 259201.0f,
                                 config),
            StopReason::kWallClock);
  // Including the timing phase, where no other criterion holds.
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kTiming,
                                 -8.0f,
                                 -0.5f,
                                 1.0f,
                                 50.0f,
                                 true,
                                 -10.0f,
                                 100.0f,
                                 0.0f,
                                 259201.0f,
                                 config),
            StopReason::kWallClock);
}

// With no clock the timing targets have no scale and must not fire.
TEST(ThresholdBattery, NoClockDisablesTimingTargets)
{
  const GlobalSizingConfig config = batteryConfig();
  EXPECT_EQ(thresholdBatteryStop(BatteryPhase::kTiming,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 100.0f,
                                 false,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 config),
            StopReason::kNone);
}

////////////////////////////////////////////////////////////////
// flachDominates (Flach et al., TCAD 2014)

TEST(FlachDominance, RequiresBothTheTnsGateAndLowerPower)
{
  const float T = 1.0f;
  const float frac = 0.10f;  // |TNS| must be < 0.1
  // First qualifying iterate is always stored.
  EXPECT_TRUE(flachDominates(-0.05f, 100.0f, T, frac, false, 0.0f));
  // Qualifies on TNS and beats the stored power.
  EXPECT_TRUE(flachDominates(-0.05f, 90.0f, T, frac, true, 100.0f));
  // Qualifies on TNS but does not beat the stored power.
  EXPECT_FALSE(flachDominates(-0.05f, 110.0f, T, frac, true, 100.0f));
  // Lower power but outside the TNS gate: not a better solution.
  EXPECT_FALSE(flachDominates(-0.5f, 10.0f, T, frac, true, 100.0f));
  // Equal power does not displace the stored best (strict improvement).
  EXPECT_FALSE(flachDominates(-0.05f, 100.0f, T, frac, true, 100.0f));
}

TEST(FlachDominance, NoClockDisqualifiesEveryIterate)
{
  EXPECT_FALSE(flachDominates(0.0f, 10.0f, 0.0f, 0.10f, false, 0.0f));
}

////////////////////////////////////////////////////////////////
// livramento_feasible (Livramento et al., DATE 2013, Alg. 1 line 20)

TEST(LivramentoFeasible, ViolationFreeNeedsMetTimingAndNoCapOrSlewViolation)
{
  EXPECT_TRUE(livramentoViolationFree(0.1f, 0, 0));
  // Zero slack meets timing.
  EXPECT_TRUE(livramentoViolationFree(0.0f, 0, 0));
  EXPECT_FALSE(livramentoViolationFree(-0.001f, 0, 0));
  EXPECT_FALSE(livramentoViolationFree(0.1f, 1, 0));
  EXPECT_FALSE(livramentoViolationFree(0.1f, 0, 1));
}

TEST(LivramentoFeasible, ReplacesOnlyWithAViolationFreeLowerPowerIterate)
{
  // The first violation-free iterate is always stored.
  EXPECT_TRUE(livramentoReplacesBest(true, 100.0f, false, 0.0f));
  EXPECT_TRUE(livramentoReplacesBest(true, 90.0f, true, 100.0f));
  EXPECT_FALSE(livramentoReplacesBest(true, 110.0f, true, 100.0f));
  // A tie keeps the earlier iterate.
  EXPECT_FALSE(livramentoReplacesBest(true, 100.0f, true, 100.0f));
  // An iterate with a violation is never stored, however low its power.
  EXPECT_FALSE(livramentoReplacesBest(false, 10.0f, false, 0.0f));
  EXPECT_FALSE(livramentoReplacesBest(false, 10.0f, true, 100.0f));
}

// One iterate's end-of-iteration state, as the tracker reads it.
struct FeasibleIterate
{
  float wns;
  size_t cap_violations;
  size_t slew_violations;
  float power;
};

// The tracker's selection after offering the iterates in order, as the
// tracker does at the end of each iteration. `stored` lists the iterates
// offer() stored, each of which the tracker captures.
struct FeasibleRun
{
  FeasibleSelection selection;
  std::vector<int> stored;
};

FeasibleRun selectFeasible(const std::vector<FeasibleIterate>& iterates)
{
  FeasibleRun run;
  for (int i = 0; i < static_cast<int>(iterates.size()); ++i) {
    const FeasibleIterate& iterate = iterates[i];
    if (run.selection.offer(i,
                            iterate.wns,
                            iterate.cap_violations,
                            iterate.slew_violations,
                            iterate.power)) {
      run.stored.push_back(i);
    }
  }
  return run;
}

TEST(LivramentoFeasible, SkipsALowerPowerIterateWithAMaxCapViolation)
{
  const FeasibleRun r
      = selectFeasible({{0.1f, 0, 0, 100.0f}, {0.1f, 1, 0, 60.0f}});
  EXPECT_EQ(r.selection.best_iter, 0);
  EXPECT_EQ(r.selection.violation_free_count, 1);
}

TEST(LivramentoFeasible, SkipsALowerPowerIterateWithNegativeWns)
{
  const FeasibleRun r
      = selectFeasible({{0.1f, 0, 0, 100.0f}, {-0.01f, 0, 0, 60.0f}});
  EXPECT_EQ(r.selection.best_iter, 0);
  EXPECT_EQ(r.selection.violation_free_count, 1);
}

TEST(LivramentoFeasible, SkipsALowerPowerIterateWithASlewViolation)
{
  const FeasibleRun r
      = selectFeasible({{0.1f, 0, 0, 100.0f}, {0.1f, 0, 3, 60.0f}});
  EXPECT_EQ(r.selection.best_iter, 0);
  EXPECT_EQ(r.selection.violation_free_count, 1);
}

// Violation-free powers 100, 80, 80, 90 and 85, with a violating 50 between
// them: iterate 0 is stored first, iterate 1 replaces it with the lowest
// power, and iterate 2 ties it but comes later.
TEST(LivramentoFeasible, LowestPowerViolationFreeIterateWins)
{
  const FeasibleRun r = selectFeasible({{0.2f, 0, 0, 100.0f},
                                        {0.1f, 0, 0, 80.0f},
                                        {0.3f, 0, 0, 80.0f},
                                        {0.1f, 2, 1, 50.0f},
                                        {0.1f, 0, 0, 90.0f},
                                        {0.0f, 0, 0, 85.0f}});
  EXPECT_EQ(r.selection.best_iter, 1);
  EXPECT_EQ(r.stored, (std::vector<int>{0, 1}));
  EXPECT_EQ(r.selection.violation_free_count, 5);
  EXPECT_EQ(r.selection.iterations, 6);
}

// The final iterate then stands, since nothing was stored to restore.
TEST(LivramentoFeasible, NothingStoredWithoutAViolationFreeIterate)
{
  const FeasibleRun r = selectFeasible(
      {{-0.1f, 0, 0, 100.0f}, {0.1f, 1, 0, 80.0f}, {0.1f, 0, 1, 60.0f}});
  EXPECT_FALSE(r.selection.hasBest());
  EXPECT_TRUE(r.stored.empty());
  EXPECT_EQ(r.selection.violation_free_count, 0);
  EXPECT_EQ(r.selection.iterations, 3);
}

////////////////////////////////////////////////////////////////
// reimannScore (Reimann et al., ISPD 2016, Eq. 6)

// The input solution scores exactly 0, the value every iterate must beat.
TEST(ReimannScore, InputSolutionScoresZero)
{
  EXPECT_FLOAT_EQ(reimannScore(0.0f, 0.0f, 0.0f, 0.0f), 0.0f);
}

// A 10% power win with unchanged timing/area: score = -(-0.1 + 0 + 1 - 1) = 0.1
TEST(ReimannScore, PowerWinWithFlatTimingScoresPositive)
{
  EXPECT_FLOAT_EQ(reimannScore(-0.10f, 0.0f, 0.0f, 0.0f), 0.10f);
}

// Real timing degradation (dTV + dWNS = -1) makes the exponential term large:
// score = -(-0.1 + 0 + 2^1 - 1) = -(0.9) = -0.9. Strongly negative, so the
// solution is ignored however much power it saved.
TEST(ReimannScore, TimingDegradationDominatesThePowerWin)
{
  EXPECT_FLOAT_EQ(reimannScore(-0.10f, 0.0f, -0.5f, -0.5f), -0.9f);
}

// The "small window of compromise": a 1% timing degradation against a 10% power
// win still scores positive.
// score = -(-0.10 + 0 + 2^0.01 - 1) = 0.10 - 0.006956 = 0.093044
TEST(ReimannScore, TolerateTinyTimingNoiseForARealPowerWin)
{
  EXPECT_NEAR(reimannScore(-0.10f, 0.0f, -0.005f, -0.005f), 0.093044f, 1e-5f);
}

// Improving timing raises the score above the pure power/area part.
// dTV + dWNS = 0.5 -> -(0 + 0 + 2^-0.5 - 1) = 1 - 0.7071 = 0.2929
TEST(ReimannScore, TimingImprovementScoresPositive)
{
  EXPECT_NEAR(reimannScore(0.0f, 0.0f, 0.25f, 0.25f), 0.29289f, 1e-5f);
}

// scoreDeltas' sign conventions: power/area are changes (negative = better),
// TV/WNS are improvements (positive = better).
TEST(ScoreDeltas, SignConventions)
{
  const IterMetrics init{.wns = -0.20f,
                         .tns = -10.0f,
                         .leakage = 100.0f,
                         .power = 100.0f,
                         .area = 50.0f};
  const IterMetrics cur{.wns = -0.10f,
                        .tns = -5.0f,
                        .leakage = 90.0f,
                        .power = 90.0f,
                        .area = 55.0f};
  const ScoreDeltas d = scoreDeltas(init, cur);
  EXPECT_FLOAT_EQ(d.d_power, -0.10f);  // 10% less power
  EXPECT_FLOAT_EQ(d.d_area, 0.10f);    // 10% more area
  EXPECT_FLOAT_EQ(d.d_tv, 0.50f);      // |TNS| halved -> 50% improvement
  EXPECT_FLOAT_EQ(d.d_wns, 0.50f);     // WNS -0.2 -> -0.1 on a 0.2 scale
  // That solution is better on timing and power, worse on area:
  // -( -0.1 + 0.1 + 2^-1 - 1 ) = 0.5
  EXPECT_FLOAT_EQ(reimannScore(d.d_power, d.d_area, d.d_tv, d.d_wns), 0.5f);
}

TEST(ScoreDeltas, ZeroReferencesContributeZeroDeltas)
{
  const IterMetrics init;  // all zero
  const IterMetrics cur{.wns = -0.10f,
                        .tns = -5.0f,
                        .leakage = 90.0f,
                        .power = 90.0f,
                        .area = 55.0f};
  const ScoreDeltas d = scoreDeltas(init, cur);
  EXPECT_FLOAT_EQ(d.d_power, 0.0f);
  EXPECT_FLOAT_EQ(d.d_area, 0.0f);
  EXPECT_FLOAT_EQ(d.d_tv, 0.0f);
  EXPECT_FLOAT_EQ(d.d_wns, 0.0f);
}

// dPower is the change of the metrics' power, which is total power under the
// total-power objective, not of leakage. Here total power falls 10% while
// leakage rises 20%, with timing and area unchanged:
//   dPower = (360 - 400) / 400 = -0.1
//   score  = -(-0.1 + 0 + 2^0 - 1) = 0.1
// Read from leakage, dPower would be +0.2 and the score -0.2.
TEST(ScoreDeltas, PowerDeltaReadsPowerNotLeakage)
{
  const IterMetrics init{.wns = -0.20f,
                         .tns = -10.0f,
                         .leakage = 100.0f,
                         .power = 400.0f,
                         .area = 50.0f};
  const IterMetrics cur{.wns = -0.20f,
                        .tns = -10.0f,
                        .leakage = 120.0f,
                        .power = 360.0f,
                        .area = 50.0f};
  const ScoreDeltas d = scoreDeltas(init, cur);
  EXPECT_FLOAT_EQ(d.d_power, -0.10f);
  EXPECT_FLOAT_EQ(d.d_area, 0.0f);
  EXPECT_FLOAT_EQ(d.d_tv, 0.0f);
  EXPECT_FLOAT_EQ(d.d_wns, 0.0f);
  EXPECT_FLOAT_EQ(reimannScore(d.d_power, d.d_area, d.d_tv, d.d_wns), 0.1f);
}

////////////////////////////////////////////////////////////////
// lagrangianEstimate (debug diagnostic). It is L(x, λ) at the current iterate,
// not the dual function Q(λ), and not a bound.

// L(x, λ) = power + tw * (sum lambda*d - sum mu*r).
TEST(LagrangianEstimate, HandComputed)
{
  EXPECT_FLOAT_EQ(lagrangianEstimate(/*power=*/10.0f,
                                     /*timing_weight=*/2.0f,
                                     /*lambda_delay_sum=*/3.0f,
                                     /*mu_required_sum=*/5.0f),
                  10.0f + 2.0f * (3.0f - 5.0f));  // = 6
}

// The gap between the primal and L is the mu-weighted violation: with a
// flow-conserving lambda, sum lambda*d - sum mu*r == sum mu*(arrival -
// required) == sum mu*(-slack). A violating design (arrivals after their
// required times) puts L above the primal power, and a design that meets
// timing puts it below.
TEST(LagrangianEstimate, GapTracksTheMuWeightedViolation)
{
  const float power = 10.0f;
  const float tw = 1.0f;
  // Violating: sum lambda*d (12) exceeds sum mu*r (10) -> L above primal.
  EXPECT_GT(lagrangianEstimate(power, tw, 12.0f, 10.0f), power);
  // Timing met with slack: sum lambda*d (8) below sum mu*r (10) -> L below.
  EXPECT_LT(lagrangianEstimate(power, tw, 8.0f, 10.0f), power);
}

////////////////////////////////////////////////////////////////
// Config: defaults and preset settings

// By default global sizing keeps the best iterate (Flach dominance) and
// restores it at the end.
TEST(HConfig, DefaultBestTrackerIsFlachDominance)
{
  EXPECT_EQ(GlobalSizingConfig{}.best_tracker,
            BestTrackerKind::kFlachDominance);
  EXPECT_EQ(GlobalSizingConfig{}.termination, TerminationKind::kFixedIters);
}

// ... but rsz_baseline uses wns_pass_reject, OpenROAD's original rule that
// rejects a pass which makes WNS worse.
TEST(HConfig, RszBaselinePresetPinsWnsPassReject)
{
  GlobalSizingConfig config;
  config.applyPreset(Preset::kRszBaseline);
  EXPECT_EQ(config.best_tracker, BestTrackerKind::kWnsPassReject);
  EXPECT_EQ(config.termination, TerminationKind::kFixedIters);
}

// wns_pass_reject is OpenROAD's own rule and appears in no paper, so only
// rsz_baseline may use it. If a paper preset (or the struct default) used it,
// the paper's own best-solution rule would run on top of OpenROAD's.
TEST(HConfig, OnlyRszBaselineGetsWnsPassReject)
{
  EXPECT_NE(GlobalSizingConfig{}.best_tracker, BestTrackerKind::kWnsPassReject);
  for (const Preset p : kAllPresets) {
    if (p == Preset::kRszBaseline) {
      continue;
    }
    GlobalSizingConfig config;
    config.applyPreset(p);
    EXPECT_NE(config.best_tracker, BestTrackerKind::kWnsPassReject)
        << "paper preset " << toString(p)
        << " must not carry the rsz_baseline WNS pass-reject rule";
  }
}

// Every option must construct.
TEST(HConfig, BestTrackerFactoryDispatchesEveryOption)
{
  for (const BestTrackerKind kind : {BestTrackerKind::kNone,
                                     BestTrackerKind::kFlachDominance,
                                     BestTrackerKind::kReimannScore,
                                     BestTrackerKind::kWnsPassReject,
                                     BestTrackerKind::kLivramentoFeasible}) {
    GlobalSizingConfig config;
    config.best_tracker = kind;
    EXPECT_NE(makeBestTracker(config), nullptr);
  }
}

// Only wns_pass_reject reports a sweep that makes WNS worse as rejected; every
// other option keeps every pass, so the `accepted=` trace field and the
// RSZ-0400 counters read 'accepted' regardless of WNS (which the level-1 trace
// still reports as `wns=`).
TEST(HConfig, OnlyWnsPassRejectRejectsAPass)
{
  GlobalSizingConfig config;
  config.best_tracker = BestTrackerKind::kFlachDominance;
  LrState state;
  EXPECT_FALSE(makeBestTracker(config)->considerPass(state, true));

  config.best_tracker = BestTrackerKind::kReimannScore;
  EXPECT_FALSE(makeBestTracker(config)->considerPass(state, true));

  config.best_tracker = BestTrackerKind::kNone;
  EXPECT_FALSE(makeBestTracker(config)->considerPass(state, true));

  config.best_tracker = BestTrackerKind::kLivramentoFeasible;
  EXPECT_FALSE(makeBestTracker(config)->considerPass(state, true));
}

// The best-solution rule of every preset. Only livramento_partial uses
// livramento_feasible, the rule of its paper (Alg. 1 line 20).
TEST(HConfig, BestTrackerPerPreset)
{
  const struct
  {
    Preset preset;
    BestTrackerKind tracker;
  } cases[] = {
      {Preset::kRszBaseline, BestTrackerKind::kWnsPassReject},
      {Preset::kChen, BestTrackerKind::kNone},
      {Preset::kTennakoon, BestTrackerKind::kNone},
      {Preset::kFlach, BestTrackerKind::kFlachDominance},
      {Preset::kSharmaSeq, BestTrackerKind::kFlachDominance},
      {Preset::kReimann, BestTrackerKind::kReimannScore},
      {Preset::kMangiras, BestTrackerKind::kFlachDominance},
      {Preset::kLivramento, BestTrackerKind::kLivramentoFeasible},
      {Preset::kChinnery, BestTrackerKind::kNone},
  };
  EXPECT_EQ(std::size(cases), std::size(kAllPresets))
      << "every preset needs a row";
  for (const auto& c : cases) {
    GlobalSizingConfig config;
    config.applyPreset(c.preset);
    EXPECT_EQ(config.best_tracker, c.tracker)
        << "preset " << toString(c.preset) << " best_tracker";
  }

  for (const Preset p : kAllPresets) {
    if (p == Preset::kLivramento) {
      continue;
    }
    GlobalSizingConfig config;
    config.applyPreset(p);
    EXPECT_NE(config.best_tracker, BestTrackerKind::kLivramentoFeasible)
        << "preset " << toString(p);
  }
}

TEST(HConfig, PaperPresetBundles)
{
  GlobalSizingConfig flach;
  flach.applyPreset(Preset::kFlach);
  EXPECT_EQ(flach.best_tracker, BestTrackerKind::kFlachDominance);
  // Fig. 1a-b: minimum-leakage reset, then the reverse-topological electrical
  // repair. Alg. 4's load check relies on this clean start, since it only
  // forbids increasing an existing load violation.
  EXPECT_EQ(flach.init_mode, InitMode::kMinSizeFixviol);

  // Chen's SOLVE_LRS/μ starts from the minimum-size solution ("for i := 1 to n
  // do x_i := L_i"); init_mode = min_size is the discrete-library equivalent.
  GlobalSizingConfig chen;
  chen.applyPreset(Preset::kChen);
  EXPECT_EQ(chen.init_mode, InitMode::kMinSize);
  // state_adaptive is the only seed that cannot follow an init pass
  // (RSZ-0421); Chen uses a constant seed, so the preset is consistent.
  EXPECT_EQ(chen.lambda_seed, LambdaSeed::kConstant);

  GlobalSizingConfig reimann;
  reimann.applyPreset(Preset::kReimann);
  EXPECT_EQ(reimann.best_tracker, BestTrackerKind::kReimannScore);

  // Livramento keeps the lowest-power iterate without violations (Alg. 1
  // line 20); its LDP runs up to 60 iterations.
  GlobalSizingConfig livramento;
  livramento.applyPreset(Preset::kLivramento);
  EXPECT_EQ(livramento.best_tracker, BestTrackerKind::kLivramentoFeasible);
  EXPECT_EQ(livramento.max_iterations, 60);

  // Sharma: sets of 5 iterations, 2 stagnant sets, any improvement counts.
  GlobalSizingConfig sharma;
  sharma.applyPreset(Preset::kSharmaSeq);
  EXPECT_EQ(sharma.termination, TerminationKind::kStagnationWindows);
  EXPECT_EQ(sharma.stagnation_window, 5);
  EXPECT_EQ(sharma.stagnation_count, 2);
  EXPECT_FLOAT_EQ(sharma.stagnation_improve_frac, 0.0f);
  EXPECT_FALSE(sharma.stagnation_require_tns);
  // Minimum-leakage start plus its two repairs: max capacitance from outputs
  // to inputs, then slew from inputs to outputs (Sec. III-A). Sharma et al.
  // keep cap/slew clean throughout: start clean, then skip any OLR candidate
  // that would violate.
  EXPECT_EQ(sharma.init_mode, InitMode::kMinSizeFixcap);
  EXPECT_TRUE(sharma.slew_fix_pass);
  // Eq. 5's candidate cost is the local arcs plus Flach's downstream
  // sensitivity term, which is cost_global_phi. Without it the cost would be
  // local arcs only, a different method.
  EXPECT_TRUE(sharma.cost_global_phi);
  // ... and neither validator check fires: phi with delta_delay is rejected
  // (RSZ-0424), and phi with fanout_slew counts the sink level twice and warns
  // (RSZ-0429).
  EXPECT_FALSE(sharma.cost_delta_delay);
  EXPECT_FALSE(sharma.cost_fanout_slew);

  // Mangiras: TNS and leakage improving < 1% across two consecutive
  // iterations, the same rule with different constants. window = 1 is what
  // makes it "two consecutive iterations": each iteration is compared with the
  // previous one. window = 2 would average disjoint 2-iteration blocks and
  // compare block to block, which first fires at iteration 4 and only on even
  // iterations, a different rule.
  GlobalSizingConfig mangiras;
  mangiras.applyPreset(Preset::kMangiras);
  EXPECT_EQ(mangiras.termination, TerminationKind::kStagnationWindows);
  EXPECT_EQ(mangiras.stagnation_window, 1);
  EXPECT_EQ(mangiras.stagnation_count, 1);
  EXPECT_FLOAT_EQ(mangiras.stagnation_improve_frac, 0.01f);
  EXPECT_TRUE(mangiras.stagnation_require_tns);
}

// Endpoint (mu) policy and reimann_setpoint per preset. The three presets
// whose papers state an explicit endpoint multiplier update use it; the others
// use endpoint_lambda. Every preset keeps the paper's s_init setpoint.
TEST(HConfig, C1EndpointAndSetpointBundles)
{
  const struct
  {
    Preset preset;
    MuPolicy mu;
  } mu_cases[] = {
      {Preset::kChen, MuPolicy::kEndpointAdditive},
      {Preset::kTennakoon, MuPolicy::kEndpointRatio},
      {Preset::kLivramento, MuPolicy::kEndpointRatio},
      {Preset::kSharmaSeq, MuPolicy::kEndpointLambda},
      {Preset::kFlach, MuPolicy::kEndpointLambda},
      {Preset::kMangiras, MuPolicy::kEndpointLambda},
      {Preset::kReimann, MuPolicy::kEndpointLambda},
      // Chinnery's Eq. 2 endpoint constraint is the derived-mu policy, so
      // endpoint_lambda is this paper's own rule rather than a substitute.
      {Preset::kChinnery, MuPolicy::kEndpointLambda},
      {Preset::kRszBaseline, MuPolicy::kReseedEachIter},
  };
  EXPECT_EQ(std::size(mu_cases), std::size(kAllPresets))
      << "every preset needs a row: a table that silently omits one stops "
         "pinning its E4 policy";
  for (const auto& c : mu_cases) {
    GlobalSizingConfig config;
    config.applyPreset(c.preset);
    EXPECT_EQ(config.mu_policy, c.mu)
        << "preset " << toString(c.preset) << " mu_policy";
  }

  for (const Preset p : kAllPresets) {
    GlobalSizingConfig config;
    config.applyPreset(p);
    EXPECT_EQ(config.reimann_setpoint, ReimannSetpoint::kSInit)
        << "preset " << toString(p)
        << " must keep the faithful s_init setpoint";
  }

  // Livramento's minimum-leakage initialization (Alg. 1 line 2) stops at the
  // reset: Alg. 3 FIX_VIOLATIONS is a per-iteration repair, not an init pass,
  // so this preset uses min_size rather than min_size_fixviol.
  GlobalSizingConfig livramento;
  livramento.applyPreset(Preset::kLivramento);
  EXPECT_EQ(livramento.init_mode, InitMode::kMinSize);
}

// Termination settings per preset: pure_cap for chen, tennakoon, livramento,
// flach and reimann (rsz_baseline keeps fixed_iters and its early exits), each
// paper's max_iterations, best_tracker = none for chen and tennakoon, and the
// near-met gate (sharma only; mangiras stays ungated so its window rule is
// unchanged).
TEST(HConfig, C2TerminationBundles)
{
  const struct
  {
    Preset preset;
    TerminationKind term;
    int max_iter;
  } term_cases[] = {
      {Preset::kChen, TerminationKind::kPureCap, 100},
      {Preset::kTennakoon, TerminationKind::kPureCap, 100},
      {Preset::kLivramento, TerminationKind::kPureCap, 60},
      {Preset::kFlach, TerminationKind::kPureCap, 120},
      {Preset::kReimann, TerminationKind::kPureCap, 20},
      {Preset::kSharmaSeq, TerminationKind::kStagnationWindows, 160},
      {Preset::kMangiras, TerminationKind::kStagnationWindows, 20},
      {Preset::kChinnery, TerminationKind::kThresholdBattery, 80},
      {Preset::kRszBaseline, TerminationKind::kFixedIters, 20},
  };
  EXPECT_EQ(std::size(term_cases), std::size(kAllPresets))
      << "every preset needs a row";
  for (const auto& c : term_cases) {
    GlobalSizingConfig config;
    config.applyPreset(c.preset);
    EXPECT_EQ(config.termination, c.term)
        << "preset " << toString(c.preset) << " termination";
    EXPECT_EQ(config.max_iterations, c.max_iter)
        << "preset " << toString(c.preset) << " max_iterations";
  }

  // chen and tennakoon use `none`: their papers keep no best-so-far solution,
  // so the final iterate is the result (consistent with pure_cap). sharma and
  // mangiras keep flach_dominance, part of the Flach method they build on.
  GlobalSizingConfig chen;
  chen.applyPreset(Preset::kChen);
  EXPECT_EQ(chen.best_tracker, BestTrackerKind::kNone);
  GlobalSizingConfig tennakoon;
  tennakoon.applyPreset(Preset::kTennakoon);
  EXPECT_EQ(tennakoon.best_tracker, BestTrackerKind::kNone);
  GlobalSizingConfig sharma;
  sharma.applyPreset(Preset::kSharmaSeq);
  EXPECT_EQ(sharma.best_tracker, BestTrackerKind::kFlachDominance);
  GlobalSizingConfig mangiras;
  mangiras.applyPreset(Preset::kMangiras);
  EXPECT_EQ(mangiras.best_tracker, BestTrackerKind::kFlachDominance);

  // Near-met gate: only sharma uses the paper's 1% activation. Every other
  // preset keeps the disabled default (frac < 0), in particular mangiras,
  // whose window-1 rule is meant to run ungated.
  EXPECT_FLOAT_EQ(sharma.near_met_gate_frac, 0.01f);
  EXPECT_LT(GlobalSizingConfig{}.near_met_gate_frac, 0.0f);
  for (const Preset p : kAllPresets) {
    if (p == Preset::kSharmaSeq) {
      continue;
    }
    GlobalSizingConfig config;
    config.applyPreset(p);
    EXPECT_LT(config.near_met_gate_frac, 0.0f)
        << "preset " << toString(p) << " must stay ungated";
  }
}

// timing_scale and timing_bias per preset. timing_scale decides whether the
// magnitude of λ affects the cost (see the Eq. 6 test in TestTimingScale.cc).
//
// timing_bias is not 1.0 on every paper preset: the auto_median and
// livramento_alpha presets (tennakoon, reimann, livramento) use 12.0. It has
// no effect under unit (flach, sharma, chen, mangiras, chinnery), where 1.0
// indicates that λ carries the timing/leakage balance instead.
TEST(HConfig, C4TimingScaleAndBiasBundles)
{
  const struct
  {
    Preset preset;
    TimingScale scale;
    float bias;
  } cases[] = {
      {Preset::kRszBaseline, TimingScale::kAutoMedian, 64.0f},
      {Preset::kChen, TimingScale::kUnit, 1.0f},
      {Preset::kTennakoon, TimingScale::kAutoMedian, 12.0f},
      {Preset::kLivramento, TimingScale::kLivramentoAlpha, 12.0f},
      {Preset::kSharmaSeq, TimingScale::kUnit, 1.0f},
      {Preset::kReimann, TimingScale::kAutoMedian, 12.0f},
      {Preset::kFlach, TimingScale::kUnit, 1.0f},
      {Preset::kMangiras, TimingScale::kUnit, 1.0f},
      {Preset::kChinnery, TimingScale::kUnit, 1.0f},
  };
  EXPECT_EQ(std::size(cases), std::size(kAllPresets))
      << "every preset needs a row";
  for (const auto& c : cases) {
    GlobalSizingConfig config;
    config.applyPreset(c.preset);
    EXPECT_EQ(config.timing_scale, c.scale)
        << "preset " << toString(c.preset) << " timing_scale";
    EXPECT_FLOAT_EQ(config.timing_bias, c.bias)
        << "preset " << toString(c.preset) << " timing_bias";
  }

  // Mangiras' Eq. 6 global power ratio only reaches the cost with the λ-free
  // `unit` scale and endpoint_lambda together; check both on that preset.
  GlobalSizingConfig mangiras;
  mangiras.applyPreset(Preset::kMangiras);
  EXPECT_EQ(mangiras.timing_scale, TimingScale::kUnit);
  EXPECT_EQ(mangiras.mu_policy, MuPolicy::kEndpointLambda);
  // Neither uses the φ downstream-sensitivity cost term. Flach et al. turn it
  // off for RC wires (Sec. IX-B), and mangiras, which builds on Flach's
  // method, only changes the initialization (Eqs. 5-7).
  GlobalSizingConfig flach;
  flach.applyPreset(Preset::kFlach);
  EXPECT_FALSE(flach.cost_global_phi);
  EXPECT_FALSE(mangiras.cost_global_phi);
}

// Move set per preset. sharma uses Fast-OLR and mangiras the +-1 size step of
// its Sec. 4.3 (without it, the preset would match the setup of the paper's
// Tables 1-2 instead). Every other preset evaluates the full library, as its
// paper's LR subproblem does.
TEST(HConfig, MoveSetBundles)
{
  const struct
  {
    Preset preset;
    MoveSet move_set;
  } cases[] = {
      {Preset::kRszBaseline, MoveSet::kFullLibrary},
      {Preset::kChen, MoveSet::kFullLibrary},
      {Preset::kTennakoon, MoveSet::kFullLibrary},
      {Preset::kLivramento, MoveSet::kFullLibrary},
      {Preset::kSharmaSeq, MoveSet::kSharmaFastOlr},
      {Preset::kReimann, MoveSet::kFullLibrary},
      {Preset::kFlach, MoveSet::kFullLibrary},
      {Preset::kMangiras, MoveSet::kMangirasSizeStep},
      // Chinnery prunes candidates adaptively based on history (a cost-ranked
      // prefix), which no move set implements, so the preset evaluates the
      // full set of alternatives instead.
      {Preset::kChinnery, MoveSet::kFullLibrary},
  };
  EXPECT_EQ(std::size(cases), std::size(kAllPresets))
      << "every preset needs a row";
  for (const auto& c : cases) {
    GlobalSizingConfig config;
    config.applyPreset(c.preset);
    EXPECT_EQ(config.move_set, c.move_set)
        << "preset " << toString(c.preset) << " move_set";
  }

  // Sharma's switch-over stays at the paper's 5th LDP iteration in the preset
  // and in the struct default; integration tests lower it only to reach the
  // path on a design that converges in three sweeps.
  GlobalSizingConfig sharma;
  sharma.applyPreset(Preset::kSharmaSeq);
  EXPECT_EQ(sharma.fast_olr_start_iter, 5);
  EXPECT_EQ(GlobalSizingConfig{}.fast_olr_start_iter, 5);
  // The unrestricted move set remains the default.
  EXPECT_EQ(GlobalSizingConfig{}.move_set, MoveSet::kFullLibrary);
}

// output_drc_veto per preset. Only flach and chinnery state a relative rule
// ("if load violation has increased", Flach Alg. 4 line 6; "alternatives that
// would increase max-load-capacitance or max-input-slew violations are
// skipped", Chinnery and Sharma); the other papers state an absolute rule or
// none. The struct default stays absolute, OpenROAD's original behavior.
TEST(HConfig, OutputDrcVetoBundles)
{
  const struct
  {
    Preset preset;
    OutputDrcVeto veto;
  } cases[] = {
      {Preset::kRszBaseline, OutputDrcVeto::kAbsolute},
      // Chen et al. have no max-cap or max-slew constraint in their
      // formulation.
      {Preset::kChen, OutputDrcVeto::kAbsolute},
      {Preset::kTennakoon, OutputDrcVeto::kAbsolute},
      // Livramento adds the cap constraint to the cost (the beta/gamma penalty
      // terms) rather than filtering on it. That is not implemented, so the
      // preset keeps the default filter.
      {Preset::kLivramento, OutputDrcVeto::kAbsolute},
      // Sharma Fig. 9 line 10 rejects a candidate that is invalid ("a cell is
      // invalid if it causes cap or slew violations"), starting from a clean
      // netlist: the absolute rule.
      {Preset::kSharmaSeq, OutputDrcVeto::kAbsolute},
      // Reimann does state a relative rule (Alg. 1 lines 6-8, "no option may
      // worsen electrical violations beyond their initial levels"), but its
      // reference is the level at the start of the flow, while ours is the
      // current cell, re-read every sweep. Neither rule is stricter than the
      // other (ours tightens when a gate improves at a stable load and loosens
      // when its load grows), so relative would only approximate Reimann's
      // rule and the preset keeps absolute.
      {Preset::kReimann, OutputDrcVeto::kAbsolute},
      {Preset::kFlach, OutputDrcVeto::kRelative},
      // Mangiras: "any option violating a DRC is rejected outright" (Sec. 4.2,
      // Alg. 1), the absolute rule.
      {Preset::kMangiras, OutputDrcVeto::kAbsolute},
      {Preset::kChinnery, OutputDrcVeto::kRelative},
  };
  EXPECT_EQ(std::size(cases), std::size(kAllPresets))
      << "every preset needs a row";
  for (const auto& c : cases) {
    GlobalSizingConfig config;
    config.applyPreset(c.preset);
    EXPECT_EQ(config.output_drc_veto, c.veto)
        << "preset " << toString(c.preset) << " output_drc_veto";
  }

  // The default is OpenROAD's original behavior, so rsz_baseline and the
  // existing golden files are unaffected.
  EXPECT_EQ(GlobalSizingConfig{}.output_drc_veto, OutputDrcVeto::kAbsolute);

  // The two relative presets reach the loop differently: flach through the
  // min_size_fixviol repair (so only a gate the repair could not fix still
  // violates), chinnery through as_given (so every violation the design came
  // with is still there).
  GlobalSizingConfig flach;
  flach.applyPreset(Preset::kFlach);
  EXPECT_EQ(flach.init_mode, InitMode::kMinSizeFixviol);
  GlobalSizingConfig chinnery;
  chinnery.applyPreset(Preset::kChinnery);
  EXPECT_EQ(chinnery.init_mode, InitMode::kAsGiven);
}

// RSZ-0449, like RSZ-0442: only sharma_fast_olr has a switch-over iteration,
// so varying fast_olr_start_iter under any other move set produces identical
// runs while RSZ-0417 reports a different value per run. This warns rather
// than rejects, because the setting itself is harmless.
TEST(HConfig, Rsz0449WarnsOnInertFastOlrStartIter)
{
  utl::Logger logger;
  // Some presets trigger other validator warnings on purpose (the guard/engine
  // combinations, RSZ-0430/0431), so this checks the message text rather than
  // the warning count.
  auto validateLog = [&logger](const GlobalSizingConfig& config) {
    logger.redirectStringBegin();
    const bool valid = config.validate(&logger);
    const std::string out = logger.redirectStringEnd();
    EXPECT_TRUE(valid);
    return out;
  };

  // The one move set that reads it: silent at any value.
  GlobalSizingConfig live;
  live.move_set = MoveSet::kSharmaFastOlr;
  live.fast_olr_start_iter = 1;
  EXPECT_EQ(validateLog(live).find("RSZ-0449"), std::string::npos);

  for (const MoveSet move_set :
       {MoveSet::kFullLibrary, MoveSet::kMangirasSizeStep}) {
    // The default value is never flagged, whatever the move set; only a
    // changed value is.
    GlobalSizingConfig quiet;
    quiet.move_set = move_set;
    EXPECT_EQ(validateLog(quiet).find("RSZ-0449"), std::string::npos)
        << toString(move_set);

    GlobalSizingConfig inert;
    inert.move_set = move_set;
    inert.fast_olr_start_iter = 20;
    EXPECT_NE(validateLog(inert).find("RSZ-0449"), std::string::npos)
        << toString(move_set);
  }

  // sharma_seq_partial keeps the paper's 5 and every other preset keeps the
  // struct default, so no preset ever trips it.
  for (const Preset p : kAllPresets) {
    GlobalSizingConfig config;
    config.applyPreset(p);
    EXPECT_EQ(validateLog(config).find("RSZ-0449"), std::string::npos)
        << toString(p);
  }
}

// Every setting of chinnery_partial. This preset is built almost entirely from
// parts other presets also use (the sharma-style lambda update, the constant-1
// seed, Flach's veto, endpoint_lambda and the Gauss-Seidel engine), so what
// identifies it is the combination plus four settings of its own (the
// threshold battery, its constants, the 80-iteration cap, and gamma = 0).
// The settings it shares with other presets are also checked by the
// per-option tests above.
TEST(HConfig, ChinneryBundle)
{
  GlobalSizingConfig config;
  config.applyPreset(Preset::kChinnery);

  // Lambda seed: "all lambda = 1 before the first iteration".
  EXPECT_EQ(config.lambda_seed, LambdaSeed::kConstant);
  EXPECT_FLOAT_EQ(config.lambda_init_value, 1.0f);
  // Lambda update: the paper cites the rule of Sharma et al., ICCAD 2017, with
  // the formula and the phase-dependent exponents of Sharma et al., TCAD 2020
  // (4/1 in the timing phase, 1/4 in the power phase).
  EXPECT_EQ(config.lambda_update, LambdaUpdate::kSharmaArcSlack);
  EXPECT_FLOAT_EQ(config.arc_slack_k_timing_crit, 4.0f);
  EXPECT_FLOAT_EQ(config.arc_slack_k_timing_noncrit, 1.0f);
  EXPECT_FLOAT_EQ(config.arc_slack_k_power_crit, 1.0f);
  EXPECT_FLOAT_EQ(config.arc_slack_k_power_noncrit, 4.0f);
  // Table 2: "skip higher power libcells in power recovery".
  EXPECT_TRUE(config.power_phase_filter);
  // Initial solution: the paper resizes the netlist the flow hands it, so
  // as_given is intended here.
  EXPECT_EQ(config.init_mode, InitMode::kAsGiven);
  // Cost terms: Eq. 4's local arcs include fanin and fanout nets and cells.
  // phi (Flach's whole-cone sensitivity) is not used; the paper treats even
  // sibling arcs as second order. Keeping phi off also keeps RSZ-0429 silent,
  // so the sink level is counted once.
  EXPECT_TRUE(config.cost_upstream_load);
  EXPECT_TRUE(config.cost_fanout_slew);
  EXPECT_FALSE(config.cost_global_phi);
  EXPECT_FALSE(config.cost_delta_delay);
  // Downsize guard: Flach's veto with no hill-climbing tolerance. The paper's
  // 1,000,000 penalty weight "effectively prevents" degradation rather than
  // allowing some. gamma_local_slack = 0 makes Flach's Eq. 14 gamma == 1 from
  // the first iteration.
  EXPECT_EQ(config.downsize_guard, DownsizeGuard::kLocalSlackVeto);
  EXPECT_FLOAT_EQ(config.gamma_local_slack, 0.0f);
  EXPECT_FLOAT_EQ(GlobalSizingConfig{}.gamma_local_slack, 1.0f)
      << "the struct default is Flach's Eq. 14; chinnery is the one preset "
         "that opts out of the hill-climbing half";
  // Termination: the threshold battery with the paper's five constants and
  // 72 h cap, set explicitly rather than relying on the defaults.
  EXPECT_EQ(config.termination, TerminationKind::kThresholdBattery);
  EXPECT_FLOAT_EQ(config.term_tns_target_frac, 0.10f);
  EXPECT_FLOAT_EQ(config.term_wns_target_frac, 0.01f);
  EXPECT_FLOAT_EQ(config.term_tns_improve_frac, 0.10f);
  EXPECT_FLOAT_EQ(config.term_power_improve_frac, 0.01f);
  EXPECT_EQ(config.term_improve_window, 3);
  EXPECT_FLOAT_EQ(config.term_wall_limit_s, 259200.0f);
  // Best tracker: none. The only roll-back in the paper is a step of the
  // surrounding flow, outside the sizer; inside the sizer the battery's
  // TNS-degradation exit guards against the same failure.
  EXPECT_EQ(config.best_tracker, BestTrackerKind::kNone);
  // Iteration cap: the paper's own.
  EXPECT_EQ(config.max_iterations, 80);
  // Settings shared with other presets that are also part of this paper's
  // method: the reverse-topological proportional projection, the endpoint
  // constraint of Eq. 2, and the forward-topological sequential sweep that its
  // multithreading scheme reproduces.
  EXPECT_EQ(config.kkt_projection, KktProjection::kProportionalReverseTopo);
  EXPECT_EQ(config.mu_policy, MuPolicy::kEndpointLambda);
  EXPECT_EQ(config.sweep_engine, SweepEngineKind::kGaussSeidelTopo);
  EXPECT_EQ(config.traversal, Traversal::kForwardTopo);

  // The Tcl name round-trips, including the `_partial` suffix; the bare paper
  // name must not parse.
  Preset parsed = Preset::kRszBaseline;
  EXPECT_TRUE(parsePreset("chinnery_partial", parsed));
  EXPECT_EQ(parsed, Preset::kChinnery);
  EXPECT_FALSE(parsePreset("chinnery", parsed));
  EXPECT_STREQ(toString(Preset::kChinnery), "chinnery_partial");

  // The preset validates without warnings: gauss_seidel with the veto is the
  // intended guard/engine pairing, the phi/fanout_slew warning cannot fire
  // with phi off, and the battery constants are used by its own termination.
  utl::Logger logger;
  logger.redirectStringBegin();
  const bool valid = config.validate(&logger);
  const std::string out = logger.redirectStringEnd();
  EXPECT_TRUE(valid);
  EXPECT_EQ(out.find("RSZ-0429"), std::string::npos) << out;
  EXPECT_EQ(out.find("RSZ-0430"), std::string::npos) << out;
  EXPECT_EQ(out.find("RSZ-0431"), std::string::npos) << out;
  EXPECT_EQ(out.find("RSZ-0452"), std::string::npos) << out;
  EXPECT_EQ(out.find("RSZ-0439"), std::string::npos) << out;
  EXPECT_EQ(out.find("RSZ-0440"), std::string::npos) << out;
}

// Only the Chinnery preset uses the arc-slack update and the power-phase
// filter. The Sharma preset keeps its own ICCAD 2015 update.
TEST(HConfig, OnlyChinneryUsesTheArcSlackUpdateAndFilter)
{
  const GlobalSizingConfig defaults;
  EXPECT_FALSE(defaults.power_phase_filter);
  for (const Preset p : kAllPresets) {
    GlobalSizingConfig config;
    config.applyPreset(p);
    const bool chinnery = (p == Preset::kChinnery);
    EXPECT_EQ(config.lambda_update == LambdaUpdate::kSharmaArcSlack, chinnery)
        << toString(p);
    EXPECT_EQ(config.power_phase_filter, chinnery) << toString(p);
  }
  GlobalSizingConfig sharma;
  sharma.applyPreset(Preset::kSharmaSeq);
  EXPECT_EQ(sharma.lambda_update, LambdaUpdate::kSharmaCexp);
}

// RSZ-0439 and RSZ-0440: the arc-slack update and the power-phase filter read
// the phase of termination = threshold_battery. Under any other rule the run
// never leaves the timing phase, so the update keeps its timing exponents and
// the filter never applies.
TEST(HConfig, Rsz0439And0440WarnWithoutTheTwoPhaseTermination)
{
  utl::Logger logger;
  auto validateLog = [&logger](const GlobalSizingConfig& config) {
    logger.redirectStringBegin();
    const bool valid = config.validate(&logger);
    const std::string out = logger.redirectStringEnd();
    EXPECT_TRUE(valid);
    return out;
  };

  for (const TerminationKind term : {TerminationKind::kFixedIters,
                                     TerminationKind::kStagnationWindows,
                                     TerminationKind::kPureCap}) {
    GlobalSizingConfig rule;
    rule.termination = term;
    rule.lambda_update = LambdaUpdate::kSharmaArcSlack;
    const std::string rule_log = validateLog(rule);
    EXPECT_NE(rule_log.find("RSZ-0439"), std::string::npos) << toString(term);
    EXPECT_EQ(rule_log.find("RSZ-0440"), std::string::npos) << toString(term);

    GlobalSizingConfig filter;
    filter.termination = term;
    filter.power_phase_filter = true;
    const std::string filter_log = validateLog(filter);
    EXPECT_EQ(filter_log.find("RSZ-0439"), std::string::npos) << toString(term);
    EXPECT_NE(filter_log.find("RSZ-0440"), std::string::npos) << toString(term);

    GlobalSizingConfig neither;
    neither.termination = term;
    const std::string quiet_log = validateLog(neither);
    EXPECT_EQ(quiet_log.find("RSZ-0439"), std::string::npos) << toString(term);
    EXPECT_EQ(quiet_log.find("RSZ-0440"), std::string::npos) << toString(term);
  }

  // Under the two-phase termination both are silent.
  GlobalSizingConfig both = batteryConfig();
  both.lambda_update = LambdaUpdate::kSharmaArcSlack;
  both.power_phase_filter = true;
  const std::string both_log = validateLog(both);
  EXPECT_EQ(both_log.find("RSZ-0439"), std::string::npos) << both_log;
  EXPECT_EQ(both_log.find("RSZ-0440"), std::string::npos) << both_log;
}

// RSZ-0452, like RSZ-0442/0449: the six battery constants are read by only one
// termination rule, so changing them under any other rule produces identical
// runs while RSZ-0417 reports a different `chinnery=` field per run.
TEST(HConfig, Rsz0452WarnsOnInertBatteryConstants)
{
  utl::Logger logger;
  auto validateLog = [&logger](const GlobalSizingConfig& config) {
    logger.redirectStringBegin();
    const bool valid = config.validate(&logger);
    const std::string out = logger.redirectStringEnd();
    EXPECT_TRUE(valid);
    return out;
  };

  // The rule that reads them: silent at any value.
  GlobalSizingConfig live;
  live.termination = TerminationKind::kThresholdBattery;
  live.term_tns_target_frac = 0.5f;
  EXPECT_EQ(validateLog(live).find("RSZ-0452"), std::string::npos);

  for (const TerminationKind term : {TerminationKind::kFixedIters,
                                     TerminationKind::kStagnationWindows,
                                     TerminationKind::kPureCap}) {
    // Defaults are never flagged; only a changed value is.
    GlobalSizingConfig quiet;
    quiet.termination = term;
    EXPECT_EQ(validateLog(quiet).find("RSZ-0452"), std::string::npos)
        << toString(term);

    // Each of the six trips it on its own.
    for (int knob = 0; knob < 6; ++knob) {
      GlobalSizingConfig inert;
      inert.termination = term;
      switch (knob) {
        case 0:
          inert.term_tns_target_frac = 0.5f;
          break;
        case 1:
          inert.term_wns_target_frac = 0.5f;
          break;
        case 2:
          inert.term_tns_improve_frac = 0.5f;
          break;
        case 3:
          inert.term_power_improve_frac = 0.5f;
          break;
        case 4:
          inert.term_improve_window = 1;
          break;
        default:
          inert.term_wall_limit_s = 3600.0f;
          break;
      }
      EXPECT_NE(validateLog(inert).find("RSZ-0452"), std::string::npos)
          << toString(term) << " knob " << knob;
    }
  }

  // chinnery_partial sets all six to the paper's values under the one rule
  // that reads them, and every other preset leaves them at the struct default,
  // so no preset triggers the warning.
  for (const Preset p : kAllPresets) {
    GlobalSizingConfig config;
    config.applyPreset(p);
    EXPECT_EQ(validateLog(config).find("RSZ-0452"), std::string::npos)
        << toString(p);
  }
}

// RSZ-0436. The mangiras preset (state_adaptive seed with endpoint_lambda)
// preserves the Eq. 6 endpoint values and must validate silently. With
// reseed_each_iter instead, the Eq. 6 values are used at iteration 0 and
// overwritten at iteration 1, so validate() must warn (RSZ-0436) but still
// allow the run.
TEST(HConfig, Rsz0436WarnsOnlyOnTheIncoherentMangirasCross)
{
  utl::Logger logger;

  // The mangiras preset: consistent, no RSZ-0436.
  GlobalSizingConfig mangiras;
  mangiras.applyPreset(Preset::kMangiras);
  ASSERT_EQ(mangiras.lambda_seed, LambdaSeed::kStateAdaptive);
  ASSERT_EQ(mangiras.mu_policy, MuPolicy::kEndpointLambda);
  logger.redirectStringBegin();
  const bool mangiras_valid = mangiras.validate(&logger);
  const std::string mangiras_out = logger.redirectStringEnd();
  EXPECT_TRUE(mangiras_valid);
  EXPECT_EQ(mangiras_out.find("RSZ-0436"), std::string::npos)
      << "the shipped mangiras bundle must not trip RSZ-0436";

  // state_adaptive with reseed_each_iter: warns (RSZ-0436) but still valid.
  GlobalSizingConfig cross;
  cross.applyPreset(Preset::kMangiras);
  cross.mu_policy = MuPolicy::kReseedEachIter;
  logger.redirectStringBegin();
  const bool cross_valid = cross.validate(&logger);
  const std::string cross_out = logger.redirectStringEnd();
  EXPECT_TRUE(cross_valid)
      << "the cross is an ablation cell, warned not rejected";
  EXPECT_NE(cross_out.find("RSZ-0436"), std::string::npos)
      << "state_adaptive + reseed_each_iter must trip RSZ-0436";
}

// Every preset whose paper uses Flach's local-slack check must run it, not
// OpenROAD's depth budget (which appears in no paper and is the default used
// by rsz_baseline).
//
// Flach states it (Alg. 4); Reimann's Alg. 1 line 10 is the same veto;
// Mangiras "keeps the entire LR machinery of Flach et al. unchanged"; Sharma
// adopts it: "We apply this check as we recover power, after the design timing
// is within 1% of the [target]". Chinnery also cites Flach for it, as a
// penalty term with weight 1,000,000 that "effectively prevents" local-slack
// degradation, i.e. a hard veto with no hill-climbing tolerance, which is why
// chinnery_partial also sets gamma_local_slack = 0 (see ChinneryBundle).
//
// chen, tennakoon and livramento are not in this list: their papers predate or
// omit the check, and they keep the depth budget.
TEST(HConfig, FlachVetoPresetsDoNotRunTheDepthBudget)
{
  EXPECT_EQ(GlobalSizingConfig{}.downsize_guard, DownsizeGuard::kDepthBudget)
      << "the struct default (== rsz_baseline) owns the depth budget";
  for (const Preset p : {Preset::kFlach,
                         Preset::kSharmaSeq,
                         Preset::kReimann,
                         Preset::kMangiras,
                         Preset::kChinnery}) {
    GlobalSizingConfig config;
    config.applyPreset(p);
    EXPECT_EQ(config.downsize_guard, DownsizeGuard::kLocalSlackVeto)
        << "preset " << toString(p)
        << " runs a paper that adopts Flach's local-slack veto";
  }
}

// The 2% upsize threshold is OpenROAD's own filter for LR cost noise, not part
// of any paper: every paper's LR subproblem takes the plain argmin. Only
// rsz_baseline uses it; the paper presets set it to 0.
TEST(HConfig, OnlyRszBaselineGetsTheUpsizeHysteresis)
{
  EXPECT_FLOAT_EQ(GlobalSizingConfig{}.upsize_hysteresis, 0.02f)
      << "the struct default (== rsz_baseline) owns the deadband; changing it "
         "moves every default-config golden";

  GlobalSizingConfig baseline;
  baseline.applyPreset(Preset::kRszBaseline);
  EXPECT_FLOAT_EQ(baseline.upsize_hysteresis, 0.02f);

  for (const Preset p : kAllPresets) {
    if (p == Preset::kRszBaseline) {
      continue;
    }
    GlobalSizingConfig config;
    config.applyPreset(p);
    EXPECT_FLOAT_EQ(config.upsize_hysteresis, 0.0f)
        << "preset " << toString(p)
        << " must take the plain LRS argmin its paper specifies";
  }
}

TEST(HConfig, ParseRoundTrip)
{
  TerminationKind term = TerminationKind::kFixedIters;
  EXPECT_TRUE(parseTermination("stagnation_windows", term));
  EXPECT_EQ(term, TerminationKind::kStagnationWindows);
  EXPECT_TRUE(parseTermination("threshold_battery", term));
  EXPECT_EQ(term, TerminationKind::kThresholdBattery);
  EXPECT_TRUE(parseTermination("pure_cap", term));
  EXPECT_EQ(term, TerminationKind::kPureCap);
  EXPECT_TRUE(parseTermination(toString(TerminationKind::kPureCap), term));
  EXPECT_EQ(term, TerminationKind::kPureCap);
  EXPECT_FALSE(parseTermination("bogus", term));

  BestTrackerKind tracker = BestTrackerKind::kNone;
  EXPECT_TRUE(parseBestTracker("flach_dominance", tracker));
  EXPECT_EQ(tracker, BestTrackerKind::kFlachDominance);
  EXPECT_TRUE(parseBestTracker("reimann_score", tracker));
  EXPECT_EQ(tracker, BestTrackerKind::kReimannScore);
  EXPECT_TRUE(parseBestTracker("none", tracker));
  EXPECT_EQ(tracker, BestTrackerKind::kNone);
  EXPECT_TRUE(parseBestTracker("wns_pass_reject", tracker));
  EXPECT_EQ(tracker, BestTrackerKind::kWnsPassReject);
  EXPECT_TRUE(parseBestTracker("livramento_feasible", tracker));
  EXPECT_EQ(tracker, BestTrackerKind::kLivramentoFeasible);
  EXPECT_TRUE(parseBestTracker(toString(BestTrackerKind::kLivramentoFeasible),
                               tracker));
  EXPECT_EQ(tracker, BestTrackerKind::kLivramentoFeasible);
  EXPECT_FALSE(parseBestTracker("bogus", tracker));

  // output_drc_veto: both names round-trip, and toString/parse compose.
  OutputDrcVeto veto = OutputDrcVeto::kAbsolute;
  EXPECT_TRUE(parseOutputDrcVeto("relative", veto));
  EXPECT_EQ(veto, OutputDrcVeto::kRelative);
  EXPECT_TRUE(parseOutputDrcVeto("absolute", veto));
  EXPECT_EQ(veto, OutputDrcVeto::kAbsolute);
  EXPECT_FALSE(parseOutputDrcVeto("bogus", veto));
  EXPECT_TRUE(parseOutputDrcVeto(toString(OutputDrcVeto::kRelative), veto));
  EXPECT_EQ(veto, OutputDrcVeto::kRelative);

  // reimann_setpoint: both names round-trip, and toString/parse compose.
  ReimannSetpoint setpoint = ReimannSetpoint::kSInit;
  EXPECT_TRUE(parseReimannSetpoint("slack_target", setpoint));
  EXPECT_EQ(setpoint, ReimannSetpoint::kSlackTarget);
  EXPECT_TRUE(parseReimannSetpoint("s_init", setpoint));
  EXPECT_EQ(setpoint, ReimannSetpoint::kSInit);
  EXPECT_FALSE(parseReimannSetpoint("bogus", setpoint));
  EXPECT_TRUE(
      parseReimannSetpoint(toString(ReimannSetpoint::kSlackTarget), setpoint));
  EXPECT_EQ(setpoint, ReimannSetpoint::kSlackTarget);

  // power_objective: both names round-trip, and toString/parse compose.
  PowerObjective objective = PowerObjective::kLeakage;
  EXPECT_TRUE(parsePowerObjective("total", objective));
  EXPECT_EQ(objective, PowerObjective::kTotal);
  EXPECT_TRUE(parsePowerObjective("leakage", objective));
  EXPECT_EQ(objective, PowerObjective::kLeakage);
  EXPECT_FALSE(parsePowerObjective("bogus", objective));
  EXPECT_TRUE(parsePowerObjective(toString(PowerObjective::kTotal), objective));
  EXPECT_EQ(objective, PowerObjective::kTotal);

  // lambda_update sharma_arc_slack round-trips.
  LambdaUpdate update = LambdaUpdate::kNormSubgradient;
  EXPECT_TRUE(parseLambdaUpdate("sharma_arc_slack", update));
  EXPECT_EQ(update, LambdaUpdate::kSharmaArcSlack);
  EXPECT_STREQ(toString(LambdaUpdate::kSharmaArcSlack), "sharma_arc_slack");
}

// The power each preset minimizes. Reimann et al. and Chinnery and Sharma
// minimize total power, so their presets use total; every other preset keeps
// the leakage default.
TEST(HConfig, PresetPowerObjective)
{
  EXPECT_EQ(GlobalSizingConfig{}.power_objective, PowerObjective::kLeakage);
  const struct
  {
    Preset preset;
    PowerObjective objective;
  } cases[] = {
      {Preset::kRszBaseline, PowerObjective::kLeakage},
      {Preset::kChen, PowerObjective::kLeakage},
      {Preset::kTennakoon, PowerObjective::kLeakage},
      {Preset::kFlach, PowerObjective::kLeakage},
      {Preset::kSharmaSeq, PowerObjective::kLeakage},
      {Preset::kReimann, PowerObjective::kTotal},
      {Preset::kMangiras, PowerObjective::kLeakage},
      {Preset::kLivramento, PowerObjective::kLeakage},
      {Preset::kChinnery, PowerObjective::kTotal},
  };
  EXPECT_EQ(std::size(cases), std::size(kAllPresets))
      << "every preset needs a row";
  for (const auto& c : cases) {
    GlobalSizingConfig config;
    config.applyPreset(c.preset);
    EXPECT_EQ(config.power_objective, c.objective)
        << "preset " << toString(c.preset);
  }
}

// Sweeps per iteration. Chen et al. (ICCAD 1998, SOLVE_LRS/mu step 4) and
// Tennakoon and Sechen (ICCAD 2002, Sec. 3.2) repeat the sweep until nothing
// improves, so their presets run the inner loop with a cap of 10; every other
// preset keeps one sweep per iteration.
TEST(HConfig, PresetMaxInnerSweeps)
{
  EXPECT_EQ(GlobalSizingConfig{}.max_inner_sweeps, 1);
  const struct
  {
    Preset preset;
    int max_inner_sweeps;
  } cases[] = {
      {Preset::kRszBaseline, 1},
      {Preset::kChen, 10},
      {Preset::kTennakoon, 10},
      {Preset::kFlach, 1},
      {Preset::kSharmaSeq, 1},
      {Preset::kReimann, 1},
      {Preset::kMangiras, 1},
      {Preset::kLivramento, 1},
      {Preset::kChinnery, 1},
  };
  EXPECT_EQ(std::size(cases), std::size(kAllPresets))
      << "every preset needs a row";
  for (const auto& c : cases) {
    GlobalSizingConfig config;
    config.applyPreset(c.preset);
    EXPECT_EQ(config.max_inner_sweeps, c.max_inner_sweeps)
        << "preset " << toString(c.preset);
  }
}

// Only Chen et al. (ICCAD 1998, SOLVE_LRS/mu step 1) solve every subproblem
// from the lower size bound, so only chen_partial restarts each iteration.
TEST(HConfig, PresetRestartEachIteration)
{
  EXPECT_FALSE(GlobalSizingConfig{}.restart_each_iteration);
  for (const Preset preset : kAllPresets) {
    GlobalSizingConfig config;
    config.applyPreset(preset);
    EXPECT_EQ(config.restart_each_iteration, preset == Preset::kChen)
        << "preset " << toString(preset);
  }
}

// The restart with one sweep per iteration is legal but warned about
// (RSZ-0457); with more sweeps, or without the restart, nothing is warned.
TEST(HConfig, Rsz0457WarnsRestartWithOneSweep)
{
  utl::Logger logger;
  const struct
  {
    bool restart;
    int max_inner_sweeps;
    bool warned;
  } cases[] = {
      {true, 1, true},
      {true, 10, false},
      {false, 1, false},
  };
  for (const auto& c : cases) {
    GlobalSizingConfig config;
    config.restart_each_iteration = c.restart;
    config.max_inner_sweeps = c.max_inner_sweeps;
    logger.redirectStringBegin();
    EXPECT_TRUE(config.validate(&logger));
    EXPECT_EQ(logger.redirectStringEnd().find("RSZ-0457") != std::string::npos,
              c.warned)
        << "restart=" << c.restart
        << " max_inner_sweeps=" << c.max_inner_sweeps;
  }
}

// An iteration must run at least one sweep, so a cap below 1 is an error
// (RSZ-0454); 1 and any larger cap are accepted.
TEST(HConfig, Rsz0454RejectsMaxInnerSweepsBelowOne)
{
  utl::Logger logger;
  for (const int cap : {0, -1}) {
    GlobalSizingConfig config;
    config.max_inner_sweeps = cap;
    logger.redirectStringBegin();
    EXPECT_THROW(config.validate(&logger), std::runtime_error) << cap;
    EXPECT_NE(logger.redirectStringEnd().find("RSZ-0454"), std::string::npos)
        << cap;
  }
  for (const int cap : {1, 10}) {
    GlobalSizingConfig config;
    config.max_inner_sweeps = cap;
    EXPECT_TRUE(config.validate(&logger)) << cap;
  }
}

// The factories dispatch on the config enum (every option constructible).
TEST(HConfig, FactoriesDispatch)
{
  GlobalSizingConfig config;
  for (const TerminationKind t : {TerminationKind::kFixedIters,
                                  TerminationKind::kStagnationWindows,
                                  TerminationKind::kThresholdBattery,
                                  TerminationKind::kPureCap}) {
    config.termination = t;
    EXPECT_NE(makeTermination(config), nullptr) << toString(t);
  }
  for (const BestTrackerKind b : {BestTrackerKind::kNone,
                                  BestTrackerKind::kFlachDominance,
                                  BestTrackerKind::kReimannScore,
                                  BestTrackerKind::kLivramentoFeasible}) {
    config.best_tracker = b;
    EXPECT_NE(makeBestTracker(config), nullptr) << toString(b);
  }
}

}  // namespace
}  // namespace rsz
