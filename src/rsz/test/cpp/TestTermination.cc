// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the M5 H axis - termination (H1), best-solution tracking (H2)
// and the duality-gap diagnostic - against hand-computed values. The pure cores
// live in lr/Termination.hh and lr/BestTracker.hh; the STA-facing halves (the
// snapshot/restore of a cell assignment, the per-iteration metrics feed) are
// exercised by the global_sizing_termination integration test.

#include <iterator>
#include <memory>
#include <string>

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
// stagnantWindow (Sharma's rule; Mangiras' with require_tns)

// Sharma: "neither the average power, nor the minimum power solution found thus
// far, improve" - so EITHER improving keeps the run alive.
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

// Mangiras' 1% threshold: an improvement below the frac does not count.
TEST(StagnantWindow, ImprovementBelowFracIsStagnant)
{
  // 100 -> 99.5 is 0.5%, below the 1% bar: stagnant.
  EXPECT_TRUE(stagnantWindow(
      true, 100.0f, 99.5f, 90.0f, 90.0f, 0.01f, false, 0.0f, 0.0f));
  // 100 -> 98 is 2%, above it: not stagnant.
  EXPECT_FALSE(stagnantWindow(
      true, 100.0f, 98.0f, 90.0f, 90.0f, 0.01f, false, 0.0f, 0.0f));
}

// Mangiras' TNS clause: leakage stagnant but TNS still improving by more than
// the frac keeps the run alive (the rule needs BOTH stagnant).
TEST(StagnantWindow, RequireTnsKeepsRunningWhileTnsImproves)
{
  // Leakage flat; |TNS| 10 -> 8 is a 20% improvement.
  EXPECT_FALSE(stagnantWindow(
      true, 100.0f, 100.0f, 90.0f, 90.0f, 0.01f, true, -10.0f, -8.0f));
  // Leakage flat; |TNS| 10 -> 9.95 is 0.5%, below the bar: both stagnant.
  EXPECT_TRUE(stagnantWindow(
      true, 100.0f, 100.0f, 90.0f, 90.0f, 0.01f, true, -10.0f, -9.95f));
  // Without the clause, the same TNS progress is irrelevant (Sharma).
  EXPECT_TRUE(stagnantWindow(
      true, 100.0f, 100.0f, 90.0f, 90.0f, 0.01f, false, -10.0f, -8.0f));
}

////////////////////////////////////////////////////////////////
// nearMetLatched (C2): the near-met phase latch transition rule
//
// This is the one legitimately WNS-conditioned mechanism on the axis, and it
// ACTIVATES machinery (sharma's stagnation monitor, flach's local-slack veto)
// rather than gating entry or forcing a stop - so a met design latching it
// immediately, as the first case below asserts, is the intended behavior and
// not an instance of what NoOptionGatesLoopEntryOnTiming forbids.

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
  // Deep in violation: not latched (the sharma "stop at iteration 15
  // mid-closure" regime - the monitor that reads this stays inactive).
  EXPECT_FALSE(nearMetLatched(false, frac, T, -0.05f));
}

// A negative gate fraction disables gating: the run is near-met from the start,
// whatever the WNS. This is the ungated default that keeps mangiras' window
// rule and flach/reimann's veto always-on.
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

// No clock (T <= 0): no target to be within a fraction of, so a not-yet-latched
// run stays un-latched - but the disabled-gate and already-latched shortcuts
// both still win (they are checked first).
TEST(NearMetLatch, NoClockNeverLatchesButShortcutsWin)
{
  EXPECT_FALSE(nearMetLatched(false, 0.01f, 0.0f, 0.0f));
  EXPECT_TRUE(nearMetLatched(false, -1.0f, 0.0f, -1.0f));
  EXPECT_TRUE(nearMetLatched(true, 0.01f, 0.0f, 0.0f));
}

// The per-run reset: a fresh LrState is not near-met (allocate() re-clears it
// each run), so the first iteration always re-evaluates the condition.
TEST(NearMetLatch, FreshStateIsNotNearMet)
{
  LrState state;
  EXPECT_FALSE(state.near_met);
}

////////////////////////////////////////////////////////////////
// PureCap (C2): stops only at the cap

// pure_cap ignores both conditions fixed_iters stops on - consecutive rejects
// and zero-move passes, both checked after the sweep. It reads no state at all,
// proven here by driving it with a null-STA LrState.
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
  // Rejects and zero-move passes, again and again: never stops.
  const IterMetrics met{.wns = 0.5f, .tns = 0.0f, .leakage = 100.0f};
  for (int iter = 0; iter < 6; ++iter) {
    EXPECT_FALSE(cap->stopAfterSweep(
        state, iter, /*reject=*/true, /*no_benefit=*/true, met));
  }
}

// Pre-campaign engine fix 2 (2026-07-29): NO termination option may gate loop
// entry on timing. fixed_iters used to stop before the first sweep once WNS met
// setup_slack_margin, which made GLOBAL_SIZING a hard no-op (zero sweeps, zero
// moves) on every design that arrived already meeting timing - and rsz_baseline
// is the only shipped preset on fixed_iters, so it silently zeroed the
// campaign's control column.
//
// Two independent assertions here, because either alone is weak:
//   * every option returns false from stopBeforeSweep(), at iteration 0 and
//     later. `stopBeforeSweep` is now a defaulted base-class no-op that only
//     threshold_battery overrides, so this also pins that the override did not
//     grow a verdict.
//   * it holds with a MET timing state present in LrState. state.sta is null,
//   so
//     an exit that queried STA would segfault rather than return false - but a
//     reintroduced exit could just as easily read the plain floats LrState
//     already carries, so those are set to an emphatically-met design here.
//     Neither trick is a substitute for the integration goldens
//     (global_sizing_met_recovery legs 1-2), which are the real pin.
TEST(Termination, NoOptionGatesLoopEntryOnTiming)
{
  for (const TerminationKind kind : {TerminationKind::kFixedIters,
                                     TerminationKind::kStagnationWindows,
                                     TerminationKind::kThresholdBattery,
                                     TerminationKind::kPureCap}) {
    GlobalSizingConfig config;
    config.termination = kind;
    // A margin an already-met design clears with room to spare - the situation
    // the removed exit keyed on.
    config.setup_slack_margin = 0.0f;
    std::unique_ptr<Termination> term = makeTermination(config);

    LrState state;
    state.config = &config;
    // An emphatically met design, in the fields a pre-sweep exit could read
    // without dereferencing anything.
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

// The contrast that names the defect: on the identical reject sequence
// fixed_iters stops at the third consecutive rejection, pure_cap never does.
TEST(PureCap, WhereFixedItersStopsPureCapKeepsGoing)
{
  utl::Logger logger;
  GlobalSizingConfig config;
  LrState state;
  state.config = &config;
  state.logger = &logger;
  const IterMetrics met{.wns = -0.2f, .tns = -1.0f, .leakage = 100.0f};

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
// StagnationGating (C2): the near-met latch gates the monitor

// A config with the window-1 stagnation rule (each iteration vs its
// predecessor), so a flat-leakage sequence goes stagnant on the second active
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

// While the run is not near-met the monitor is inert: it neither stops nor
// accumulates window state, so a still-closing run (flat leakage, which WOULD
// read stagnant if active) is never truncated. This is the fix for the measured
// "stop at iteration 15 mid-closure" degeneracy (sharma audit §2 item 3).
TEST(StagnationGating, InactiveWhileNotNearMet)
{
  const GlobalSizingConfig config = window1StagnationConfig();
  LrState state;
  state.config = &config;
  state.near_met = false;

  StagnationWindowsTermination term;
  const IterMetrics flat{.wns = -0.2f, .tns = -5.0f, .leakage = 100.0f};
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
  const IterMetrics flat{.wns = -0.001f, .tns = -0.01f, .leakage = 100.0f};
  EXPECT_FALSE(
      term.stopAfterSweep(state, 0, false, false, flat));          // 1st window
  EXPECT_TRUE(term.stopAfterSweep(state, 1, false, false, flat));  // stagnant
}

// The windows count from the start of power recovery, not from iteration 0: a
// run that violates for several iterations (monitor inert) then latches
// near-met begins its first window at the latch. Proves latch permanence feeds
// the monitor and the inactive phase left no accumulated state behind.
TEST(StagnationGating, WindowsCountFromTheLatchNotIterationZero)
{
  utl::Logger logger;
  const GlobalSizingConfig config = window1StagnationConfig();
  LrState state;
  state.config = &config;
  state.logger = &logger;

  StagnationWindowsTermination term;
  const IterMetrics flat{.wns = -0.2f, .tns = -5.0f, .leakage = 100.0f};
  // Iterations 0-2: still closing, monitor inert (no accumulation).
  state.near_met = false;
  for (int iter = 0; iter < 3; ++iter) {
    EXPECT_FALSE(term.stopAfterSweep(state, iter, false, false, flat));
  }
  // Latch at iteration 3: this is the monitor's FIRST window (not stagnant),
  // and only the flat window after it stops the run - not the earlier ones.
  state.near_met = true;
  EXPECT_FALSE(term.stopAfterSweep(state, 3, false, false, flat));
  EXPECT_TRUE(term.stopAfterSweep(state, 4, false, false, flat));
}

////////////////////////////////////////////////////////////////
// thresholdBatteryStop (Chinnery)

GlobalSizingConfig batteryConfig()
{
  GlobalSizingConfig config;
  config.termination = TerminationKind::kThresholdBattery;
  return config;  // paper constants are the struct defaults
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
  // |TNS| 10 -> 8 = 20%: still making progress (leakage is too, 200 -> 100).
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

// Power improving by less than 1% over the window ends the run - in the power
// phase.
TEST(ThresholdBattery, PowerStall)
{
  const GlobalSizingConfig config = batteryConfig();
  // TNS still improving 20%, but leakage 100 -> 99.5 is 0.5% < 1%. Every
  // power-phase case here passes tns_at_handover == tns, i.e. timing has not
  // moved since the phase boundary, so the second power-phase exit is silent
  // and each case isolates the rule it is named for.
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

// Chinnery's power-based exits are power-phase criteria (§5). While the design
// is still improving timing, a leakage plateau must NOT end the run: the
// pre-gating battery OR-ed every clause together, so this exact state stopped
// the loop early (in practice around iteration 4) and the option measured the
// leakage plateau rather than convergence.
TEST(ThresholdBattery, TimingPhaseIgnoresThePowerStall)
{
  const GlobalSizingConfig config = batteryConfig();
  // Leakage flat (100 -> 100, 0% < 1%) but TNS improving 20%: keep going.
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
  // Leakage getting actively worse during timing improvement is still not an
  // exit - that is what the timing phase is for.
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

// The mirror of the above: the timing targets are timing-phase criteria and
// must not re-fire once the loop has handed over to power reduction (they are
// true by construction there - the phase only starts because timing was met).
TEST(ThresholdBattery, PowerPhaseIgnoresTheTimingTargets)
{
  const GlobalSizingConfig config = batteryConfig();
  // TNS and WNS both inside their targets, leakage still improving 20%.
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
// iterations" OF THE POWER PHASE. The strategy restarts the improvement window
// at the phase boundary; this pins the pure core's half of that contract - with
// no window yet (the state right after a restart), the power phase cannot stop.
//
// Without the restart the window straddles the boundary and reads a
// timing-phase leakage. The timing phase RAISES leakage (it upsizes to close
// timing), so that comparison is a negative improvement, trips kPowerStall on
// the power phase's first iteration, and the phase always lasts exactly one
// iteration - which cannot reproduce the paper's 10-iteration average.
TEST(ThresholdBattery, PowerPhaseNeedsItsOwnWindowBeforeItCanStall)
{
  const GlobalSizingConfig config = batteryConfig();
  // have_window = false: the power phase has not accumulated its own history.
  // Even against a leakage that rose 100 -> 150 (what a straddling window would
  // see), no exit fires.
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
  // And once it HAS its own window, a rising leakage is a stall (no reduction).
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

// Chinnery's SECOND power-phase exit (§5): the phase also ends "if TNS degrades
// worse than it was at the end of the timing phase". It is the paper's
// stop-loss against the power phase spending timing it cannot get back - and
// the exit it reports firing in ~24% of post-CTS / ~36% of pre-CTS runs, which
// it calls a convergence deficiency of its own method.
TEST(ThresholdBattery, TnsDegradedPastTheHandover)
{
  const GlobalSizingConfig config = batteryConfig();
  // Handover left TNS at -5; the power phase has pushed it to -5.5. Note the
  // leakage IS improving 20% (100 -> 80), so nothing else would stop this run.
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
  // TNS better than at the handover is the healthy case.
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

// The exit needs no window: its reference is the phase boundary, not a trailing
// average, so it is live on the power phase's very first iteration - which is
// also the earliest the paper's own criterion could fire. (kPowerStall cannot
// fire there; that asymmetry is deliberate.)
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

// It is a POWER-phase criterion. During timing improvement the whole point is
// that TNS moves, and it is compared against nothing - there is no handover yet
// (the strategy only writes tns_at_handover_ when the phase latches), so a
// timing-phase call must ignore the argument entirely.
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

// Precedence: the degradation exit is the more specific verdict, so it is
// reported ahead of a simultaneous power stall (a run whose timing came apart
// did not merely run out of power to recover) - but the wall-clock safety cap
// still outranks both.
TEST(ThresholdBattery, DegradationOutranksThePowerStallButNotTheWallClock)
{
  const GlobalSizingConfig config = batteryConfig();
  // Both hold: TNS worse than the handover AND leakage flat.
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

// === The battery's run records (RSZ-0450/0451) ==============================
//
// These drive the STRATEGY, not the pure core, because what they pin is what it
// publishes. The campaign harness reads the phase split off these two lines, so
// a record that is absent, or that reports a total under a per-phase label, is
// a data defect rather than a cosmetic one.
//
// T = 0 disables the timing targets (no clock to be a fraction of), which
// leaves the TNS-improvement rule as the only live timing-phase criterion and
// makes these sequences easy to reason about.
TEST(ThresholdBatteryRecords, StopRecordCarriesThePhaseSplit)
{
  utl::Logger logger;
  const GlobalSizingConfig config = batteryConfig();
  LrState state;
  state.config = &config;
  state.logger = &logger;
  state.T = 0.0f;

  ThresholdBatteryTermination term;
  const IterMetrics flat{.wns = -1.0f, .tns = -100.0f, .leakage = 100.0f};
  logger.redirectStringBegin();
  // The driver's sequence: stopBeforeSweep first, which is where the option
  // stamps the wall clock its 72 h cap measures against.
  term.stopBeforeSweep(state, 0);
  // Iterations 0-2 fill the window; iteration 3 is the first with a comparison,
  // and a flat TNS is a 0% improvement -> the timing phase ends there.
  for (int iter = 0; iter < 4; ++iter) {
    EXPECT_FALSE(term.stopAfterSweep(state, iter, false, false, flat));
  }
  // Power phase: its window restarts, so iterations 4-6 fill it and iteration 7
  // is the first that can stall. Flat leakage is a 0% reduction.
  for (int iter = 4; iter < 7; ++iter) {
    EXPECT_FALSE(term.stopAfterSweep(state, iter, false, false, flat));
  }
  EXPECT_TRUE(term.stopAfterSweep(state, 7, false, false, flat));
  const std::string out = logger.redirectStringEnd();

  EXPECT_NE(out.find("RSZ-0450"), std::string::npos) << out;
  EXPECT_NE(out.find("timing phase ended after 4 iteration(s)"),
            std::string::npos)
      << out;
  // The whole point: 8 is the run total and the split is stated, so a reader
  // (or a parser) cannot take "8" for the power phase's length.
  EXPECT_NE(out.find("stopped after 8 iteration(s) (4 timing + 4 power)"),
            std::string::npos)
      << out;
  EXPECT_NE(out.find("power improvement stalled"), std::string::npos) << out;

  // The end-of-loop hook does not double-report a run that already stopped.
  logger.redirectStringBegin();
  term.reportRunEnd(state);
  EXPECT_EQ(logger.redirectStringEnd().find("RSZ-0451"), std::string::npos);
}

// A run that ends on max_iterations satisfies no battery criterion, so
// stopAfterSweep never reports - and those are exactly the runs whose
// convergence behavior is worth reading. The end-of-loop hook publishes them,
// so "no record" can only mean "this option was not selected".
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
    const IterMetrics m{.wns = -1.0f, .tns = tns, .leakage = 100.0f};
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

// The wall-clock cap is a hard safety cap (§5), so it fires from either phase.
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
// flachDominates (H2)

TEST(FlachDominance, RequiresBothTheTnsGateAndLowerLeakage)
{
  const float T = 1.0f;
  const float frac = 0.10f;  // |TNS| must be < 0.1
  // First qualifying iterate is always stored.
  EXPECT_TRUE(flachDominates(-0.05f, 100.0f, T, frac, false, 0.0f));
  // Qualifies on TNS and beats the stored leakage.
  EXPECT_TRUE(flachDominates(-0.05f, 90.0f, T, frac, true, 100.0f));
  // Qualifies on TNS but does not beat the stored leakage.
  EXPECT_FALSE(flachDominates(-0.05f, 110.0f, T, frac, true, 100.0f));
  // Lower leakage but outside the TNS gate: not a better solution.
  EXPECT_FALSE(flachDominates(-0.5f, 10.0f, T, frac, true, 100.0f));
  // Equal leakage does not displace the stored best (strict improvement).
  EXPECT_FALSE(flachDominates(-0.05f, 100.0f, T, frac, true, 100.0f));
}

TEST(FlachDominance, NoClockDisqualifiesEveryIterate)
{
  EXPECT_FALSE(flachDominates(0.0f, 10.0f, 0.0f, 0.10f, false, 0.0f));
}

////////////////////////////////////////////////////////////////
// reimannScore (H2, Eq. 6)

// The input solution scores exactly 0 - the bar every iterate must beat.
TEST(ReimannScore, InputSolutionScoresZero)
{
  EXPECT_FLOAT_EQ(reimannScore(0.0f, 0.0f, 0.0f, 0.0f), 0.0f);
}

// A 10% power win with unchanged timing/area: score = -(-0.1 + 0 + 1 - 1) = 0.1
TEST(ReimannScore, PowerWinWithFlatTimingScoresPositive)
{
  EXPECT_FLOAT_EQ(reimannScore(-0.10f, 0.0f, 0.0f, 0.0f), 0.10f);
}

// Real timing degradation (dTV + dWNS = -1) blows the exponential up:
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
  const IterMetrics init{
      .wns = -0.20f, .tns = -10.0f, .leakage = 100.0f, .area = 50.0f};
  const IterMetrics cur{
      .wns = -0.10f, .tns = -5.0f, .leakage = 90.0f, .area = 55.0f};
  const ScoreDeltas d = scoreDeltas(init, cur);
  EXPECT_FLOAT_EQ(d.d_power, -0.10f);  // 10% less leakage
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
  const IterMetrics cur{
      .wns = -0.10f, .tns = -5.0f, .leakage = 90.0f, .area = 55.0f};
  const ScoreDeltas d = scoreDeltas(init, cur);
  EXPECT_FLOAT_EQ(d.d_power, 0.0f);
  EXPECT_FLOAT_EQ(d.d_area, 0.0f);
  EXPECT_FLOAT_EQ(d.d_tv, 0.0f);
  EXPECT_FLOAT_EQ(d.d_wns, 0.0f);
}

////////////////////////////////////////////////////////////////
// lagrangianEstimate (H, debug diagnostic; C3 item 4 rename - it is L(x, λ) at
// the current iterate, NOT the dual Q(λ) and not a bound)

// L(x, λ) = leakage + tw * (sum lambda*d - sum mu*r).
TEST(LagrangianEstimate, HandComputed)
{
  EXPECT_FLOAT_EQ(lagrangianEstimate(/*leakage=*/10.0f,
                                     /*timing_weight=*/2.0f,
                                     /*lambda_delay_sum=*/3.0f,
                                     /*mu_required_sum=*/5.0f),
                  10.0f + 2.0f * (3.0f - 5.0f));  // = 6
}

// The gap primal - L is the mu-weighted violation: with a flow-conserving
// lambda, sum lambda*d - sum mu*r == sum mu*(arrival - required) ==
// sum mu*(-slack), so a violating design (arrivals past their requireds) puts L
// ABOVE the primal leakage, and a design that meets timing pushes it below.
TEST(LagrangianEstimate, GapTracksTheMuWeightedViolation)
{
  const float leakage = 10.0f;
  const float tw = 1.0f;
  // Violating: sum lambda*d (12) exceeds sum mu*r (10) -> L above primal.
  EXPECT_GT(lagrangianEstimate(leakage, tw, 12.0f, 10.0f), leakage);
  // Timing met with slack: sum lambda*d (8) below sum mu*r (10) -> L below.
  EXPECT_LT(lagrangianEstimate(leakage, tw, 8.0f, 10.0f), leakage);
}

////////////////////////////////////////////////////////////////
// Config: the M5 default flip and the preset bundles

// The deliberate default-behavior change of M5 (plan §3.2-H, gap §2.4.1): a
// default-configured GLOBAL_SIZING now keeps and restores the best iterate.
TEST(HConfig, DefaultBestTrackerIsFlachDominance)
{
  EXPECT_EQ(GlobalSizingConfig{}.best_tracker,
            BestTrackerKind::kFlachDominance);
  EXPECT_EQ(GlobalSizingConfig{}.termination, TerminationKind::kFixedIters);
}

// ... but rsz_baseline stays pinned to the pre-M5 behavior, so it still
// reproduces the old engine bit-for-bit. Post-M5 hardening: that behavior is
// wns_pass_reject - the driver's inline best-WNS journal rule, relocated onto
// this axis - not kNone.
TEST(HConfig, RszBaselinePresetPinsWnsPassReject)
{
  GlobalSizingConfig config;
  config.applyPreset(Preset::kRszBaseline);
  EXPECT_EQ(config.best_tracker, BestTrackerKind::kWnsPassReject);
  EXPECT_EQ(config.termination, TerminationKind::kFixedIters);
}

// The whole point of the relocation: wns_pass_reject is an OpenROAD-baseline
// mechanism that no paper contains, so ONLY rsz_baseline may carry it. If a
// paper preset (or the struct default) ever picks it up again, the papers' own
// best-solution rules are running underneath OpenROAD's, which is the bug this
// axis move fixed.
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

// Every option must construct, including the relocated one.
TEST(HConfig, BestTrackerFactoryDispatchesEveryOption)
{
  for (const BestTrackerKind kind : {BestTrackerKind::kNone,
                                     BestTrackerKind::kFlachDominance,
                                     BestTrackerKind::kReimannScore,
                                     BestTrackerKind::kWnsPassReject}) {
    GlobalSizingConfig config;
    config.best_tracker = kind;
    EXPECT_NE(makeBestTracker(config), nullptr);
  }
}

// The pass policy is what the axis relocated. wns_pass_reject reports a
// WNS-regressing sweep as rejected; every other option keeps every pass, so the
// `accepted=` trace field and the RSZ-0400 counters read 'accepted' regardless
// of the WNS measurement (which the level-1 trace still reports as `wns=`).
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
}

TEST(HConfig, PaperPresetBundles)
{
  GlobalSizingConfig flach;
  flach.applyPreset(Preset::kFlach);
  EXPECT_EQ(flach.best_tracker, BestTrackerKind::kFlachDominance);
  // Fig. 1 (a)+(b): min-leakage reset then the reverse-topological electrical
  // repair. The bundle ran as_given until the it2 fixviol rider - and this is
  // the premise Alg. 4's load veto rests on, since that veto only forbids
  // INCREASING load violation.
  EXPECT_EQ(flach.init_mode, InitMode::kMinSizeFixviol);

  // Chen's SOLVE_LRS/μ starts from the minimum-size solution ("for i := 1 to n
  // do x_i := L_i"); init_mode = min_size is that corner in a discrete library.
  // The bundle ran as_given until the bucket-2 fidelity pass.
  GlobalSizingConfig chen;
  chen.applyPreset(Preset::kChen);
  EXPECT_EQ(chen.init_mode, InitMode::kMinSize);
  // The state_adaptive seed is the one seed an init pass is incompatible with
  // (RSZ-0421); Chen seeds constant, so the bundle is self-consistent.
  EXPECT_EQ(chen.lambda_seed, LambdaSeed::kConstant);

  GlobalSizingConfig reimann;
  reimann.applyPreset(Preset::kReimann);
  EXPECT_EQ(reimann.best_tracker, BestTrackerKind::kReimannScore);

  // Livramento's best-feasible snapshot rule ~= Flach's dominance; its LDP runs
  // up to 60 iterations.
  GlobalSizingConfig livramento;
  livramento.applyPreset(Preset::kLivramento);
  EXPECT_EQ(livramento.best_tracker, BestTrackerKind::kFlachDominance);
  EXPECT_EQ(livramento.max_iterations, 60);

  // Sharma: sets of 5 iterations, 2 stagnant sets, any improvement counts.
  GlobalSizingConfig sharma;
  sharma.applyPreset(Preset::kSharmaSeq);
  EXPECT_EQ(sharma.termination, TerminationKind::kStagnationWindows);
  EXPECT_EQ(sharma.stagnation_window, 5);
  EXPECT_EQ(sharma.stagnation_count, 2);
  EXPECT_FLOAT_EQ(sharma.stagnation_improve_frac, 0.0f);
  EXPECT_FALSE(sharma.stagnation_require_tns);
  // §8: the min-leakage start plus its electrical repair. Same rider as flach;
  // here the premise is the paper's INVARIANT cap/slew handling (start clean,
  // then skip any violating OLR candidate).
  EXPECT_EQ(sharma.init_mode, InitMode::kMinSizeFixviol);
  // Eq. 5's candidate cost is local arcs + Flach's downstream sensitivity term
  // (§5.2), which is cost_global_phi. Off until the bucket-2 fidelity pass -
  // the preset was running §12 ablation 8's OTHER arm (Li's local-arcs-only
  // cost) under Sharma's name.
  EXPECT_TRUE(sharma.cost_global_phi);
  // ... and the two validator crosses stay clean: phi ⊕ delta_delay is a hard
  // reject (RSZ-0424), phi + fanout_slew double-prices the sink level and warns
  // (RSZ-0429). This bundle must trip neither.
  EXPECT_FALSE(sharma.cost_delta_delay);
  EXPECT_FALSE(sharma.cost_fanout_slew);

  // Mangiras: TNS *and* leakage improving < 1% across two consecutive
  // iterations - the same rule with his constants. window = 1 is what makes it
  // "two consecutive iterations": each iteration is compared against its
  // predecessor. window = 2 would average over disjoint 2-iteration blocks and
  // compare block to block, which first fires at iteration 4 and only on even
  // iterations - a different rule (bucket-1 fidelity fix).
  GlobalSizingConfig mangiras;
  mangiras.applyPreset(Preset::kMangiras);
  EXPECT_EQ(mangiras.termination, TerminationKind::kStagnationWindows);
  EXPECT_EQ(mangiras.stagnation_window, 1);
  EXPECT_EQ(mangiras.stagnation_count, 1);
  EXPECT_FLOAT_EQ(mangiras.stagnation_improve_frac, 0.01f);
  EXPECT_TRUE(mangiras.stagnation_require_tns);
}

// C1 E4 endpoint-pressure + reimann setpoint wiring. The three pre-Flach
// presets whose papers carry an explicit endpoint-multiplier writer get it as a
// first-class option; everyone else keeps the shared endpoint_lambda default.
// reimann pins the disclosed slack_target setpoint; every other preset keeps
// the faithful s_init default (the override must not leak).
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
      // Chinnery's Eq. 2 endpoint constraint IS the derived-mu policy, so the
      // shared family default is this paper's own rule rather than a stand-in.
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

  // reimann is the only preset that flips the setpoint (a disclosed non-paper
  // adaptation); every other preset keeps the faithful s_init.
  for (const Preset p : kAllPresets) {
    if (p == Preset::kReimann) {
      continue;
    }
    GlobalSizingConfig config;
    config.applyPreset(p);
    EXPECT_EQ(config.reimann_setpoint, ReimannSetpoint::kSInit)
        << "preset " << toString(p)
        << " must keep the faithful s_init setpoint";
  }
  GlobalSizingConfig reimann;
  reimann.applyPreset(Preset::kReimann);
  EXPECT_EQ(reimann.reimann_setpoint, ReimannSetpoint::kSlackTarget);

  // Livramento's Alg. 1 L2 min-leak init is landed (C1). It stops at the reset:
  // Alg. 3 FIX_VIOLATIONS is a per-iteration repair, not an init pass, which is
  // what separates this bundle from flach/sharma's min_size_fixviol.
  GlobalSizingConfig livramento;
  livramento.applyPreset(Preset::kLivramento);
  EXPECT_EQ(livramento.init_mode, InitMode::kMinSize);
}

// C2 termination & regime hygiene: pure_cap on the five fixed_iters paper
// presets (rsz_baseline keeps its legacy exits), the paper max_iterations
// budgets, the chen/tennakoon best_tracker=none decision, and the near-met gate
// (sharma only; mangiras stays ungated so its window rule is unchanged).
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

  // H2 (C2): chen/tennakoon pin `none` - their papers have no best-so-far, the
  // final iterate stands (coherent with pure_cap). sharma/mangiras keep
  // flach_dominance (part of the imported Flach machinery / a near-right
  // least-power memory).
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

  // Near-met gate (C2): only sharma pins the paper's 1% activation. Every other
  // preset keeps the disabled default (frac < 0) - mangiras especially, whose
  // window-1 rule is correct ungated and must not change.
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

// C4 item 3: the item-5 objective-scale fields (timing_scale, timing_bias)
// were flipped per preset but never gtest-pinned - the one axis a rider could
// still silently move (mu_policy is pinned by C1EndpointAndSetpointBundles).
// The A1 axis decides whether λ's magnitude means anything, so these pins guard
// the exact quantity the Eq. 6 end-to-end test proves survives
// (TestTimingScale.cc).
//
// timing_bias is NOT uniformly 1.0 on the paper presets, contra the field's
// legacy "paper presets pin it to 1.0" gloss: the auto_median /
// livramento_alpha family (tennakoon, reimann, livramento) pins 12.0, its
// shared balance. It IS inert under unit (flach/sharma/chen/mangiras), where
// the paper's 1.0 documents that λ carries the balance instead.
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

  // Mangiras' Eq. 6 headline is the λ-free `unit` scale + endpoint_lambda: the
  // two fields that un-cancel the global power ratio. Pin both together on the
  // one preset the MB milestone exists for.
  GlobalSizingConfig mangiras;
  mangiras.applyPreset(Preset::kMangiras);
  EXPECT_EQ(mangiras.timing_scale, TimingScale::kUnit);
  EXPECT_EQ(mangiras.mu_policy, MuPolicy::kEndpointLambda);
  // Flach hosts the φ downstream-sensitivity cost term (Eq. 14); mangiras, its
  // guest, deliberately does not (it is a pure Eq. 5-7 initialization host).
  GlobalSizingConfig flach;
  flach.applyPreset(Preset::kFlach);
  EXPECT_TRUE(flach.cost_global_phi);
  EXPECT_FALSE(mangiras.cost_global_phi);
}

// it2 pass 3: the F4 move-set bundles, pinned for the same reason the A1 fields
// above are - the axis carries a preset's IDENTITY (sharma without Fast-OLR is
// a configuration its paper never reports, and mangiras without the +-1 step is
// its Tables 1-2 arm rather than its §4.3 one), so a rider must not be able to
// move it silently. Every other preset stays on the full library, which is what
// its paper's LRS enumerates.
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
      // Chinnery's own candidate-set device is history-based adaptive pruning
      // (a cost-ranked prefix), which no F4 option expresses, so the preset
      // enumerates the full alternate set - the un-approximated form.
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

  // Sharma's switch-over stays at the paper's 5th LDP iteration on the preset
  // and in the struct default; the smoke tests lower it only to reach the path
  // on a design that converges in three sweeps.
  GlobalSizingConfig sharma;
  sharma.applyPreset(Preset::kSharmaSeq);
  EXPECT_EQ(sharma.fast_olr_start_iter, 5);
  EXPECT_EQ(GlobalSizingConfig{}.fast_olr_start_iter, 5);
  // The X9 A/B needs the off path to remain a selectable configuration, not a
  // deleted one.
  EXPECT_EQ(GlobalSizingConfig{}.move_set, MoveSet::kFullLibrary);
}

// A2, the output-side DRC veto (re-audit delta D1/D3, RULED 2026-08-09). Pin
// only what a paper prescribes: flach and chinnery print a RELATIVE rule
// ("if load violation has increased", Alg. 4 line 6; "alternatives that would
// increase max-load-capacitance or max-input-slew violations are skipped", §6),
// and everyone else either prints an absolute one or prints nothing. The struct
// default must stay absolute so rsz_baseline is stock.
TEST(HConfig, OutputDrcVetoBundles)
{
  const struct
  {
    Preset preset;
    OutputDrcVeto veto;
  } cases[] = {
      {Preset::kRszBaseline, OutputDrcVeto::kAbsolute},
      // Chen 1998 has no DRC language at all - no max-cap or max-slew
      // constraint appears in the formulation.
      {Preset::kChen, OutputDrcVeto::kAbsolute},
      {Preset::kTennakoon, OutputDrcVeto::kAbsolute},
      // Livramento PRICES the cap constraint (the beta/gamma penalty terms)
      // rather than filtering on it - the OTHER arm of this axis, unimplemented
      // (RA F1). Nothing to pin here; it inherits the filter it has.
      {Preset::kLivramento, OutputDrcVeto::kAbsolute},
      // Sharma Fig. 9 line 10 rejects a candidate that IS invalid ("a cell is
      // invalid if it causes cap or slew violations") and §8 calls the
      // enforcement invariant - the absolute rule, from a clean start.
      {Preset::kSharmaSeq, OutputDrcVeto::kAbsolute},
      // Reimann DOES print a relative rule (Alg. 1 lines 6-8, "no option may
      // worsen electrical violations beyond their INITIAL levels"), but its
      // anchor is the frozen flow-entry level and ours is the live incumbent,
      // re-read every sweep. The two are different rules and NEITHER dominates
      // - ours ratchets down with a gate that improves at a stable load, and up
      // with one whose load grows - so pinning relative here would ship an
      // approximation of Reimann's rule under Reimann's name. Not pinned; RA F9
      // stays open for this preset, narrowed rather than closed.
      {Preset::kReimann, OutputDrcVeto::kAbsolute},
      {Preset::kFlach, OutputDrcVeto::kRelative},
      // Mangiras: "any option violating a DRC is rejected outright" (§4.2,
      // Alg. 1) - absolute, stated as such.
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

  // The default is the pre-A2 behaviour, which is what keeps rsz_baseline the
  // stock control column and every existing golden unmoved.
  EXPECT_EQ(GlobalSizingConfig{}.output_drc_veto, OutputDrcVeto::kAbsolute);

  // The two relative presets are exactly the two whose papers state the rule,
  // and they differ in WHY it bites: flach reaches the loop through the fixviol
  // repair (so only a gate the repair could not fix is still dirty), chinnery
  // through as_given (so every violation the design arrived with is).
  GlobalSizingConfig flach;
  flach.applyPreset(Preset::kFlach);
  EXPECT_EQ(flach.init_mode, InitMode::kMinSizeFixviol);
  GlobalSizingConfig chinnery;
  chinnery.applyPreset(Preset::kChinnery);
  EXPECT_EQ(chinnery.init_mode, InitMode::kAsGiven);
}

// RSZ-0449, the F4 twin of RSZ-0442: only sharma_fast_olr has a switch-over
// iteration, so sweeping fast_olr_start_iter under any other move set produces
// bit-identical runs while RSZ-0417 echoes a different value per run - the
// analysis would then read a real within-arm variance of zero instead of a
// misconfigured cell. Warn, not reject: the value is harmless, the sweep is
// what is wrong.
TEST(HConfig, Rsz0449WarnsOnInertFastOlrStartIter)
{
  utl::Logger logger;
  // The presets trip OTHER validator warnings by design (the §3.2-F guard /
  // engine crosses, RSZ-0430/0431), so this asserts on the message TEXT rather
  // than on a warning count.
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
    // The default value is never flagged, whatever the move set - only a sweep
    // that actually varies the knob is.
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

// it2 pass 4: the chinnery_partial bundle, pinned knob by knob. This preset is
// assembled almost entirely out of OTHER presets' machinery - the
// sharma-lineage updater, the constant-1 seed, Flach's veto, the shared
// endpoint_lambda / Gauss-Seidel family defaults - so what makes it a chinnery
// column is the COMBINATION plus four values that are its own (the battery, its
// five constants, the 80-iteration cap, gamma = 0). Every one of those is a
// rider away from becoming another preset's column, which is what this test
// exists to prevent. The axes it shares with the family are pinned by the
// C1/C2/C4/MoveSet bundle tests above.
TEST(HConfig, ChinneryBundle)
{
  GlobalSizingConfig config;
  config.applyPreset(Preset::kChinnery);

  // D: "all lambda = 1 before the first iteration" (§6).
  EXPECT_EQ(config.lambda_seed, LambdaSeed::kConstant);
  EXPECT_FLOAT_EQ(config.lambda_init_value, 1.0f);
  // E1/E2: hosted on the sharma-lineage updater - the ICCAD'17 [13] rule the
  // paper cites is not implemented and its formula is deferred to [7]. A HOST,
  // not a citation (see the preset doc block).
  EXPECT_EQ(config.lambda_update, LambdaUpdate::kSharmaCexp);
  // N: the paper resizes the netlist the flow hands it (§5 step 2) - as_given
  // is deliberate here, not an omission.
  EXPECT_EQ(config.init_mode, InitMode::kAsGiven);
  // B2: Eq. 4's local arcs include fanin AND fanout nets/cells; phi (Flach's
  // whole-cone sensitivity) is NOT in Chinnery's set - it calls even sibling
  // arcs second-order - and keeping it off is also what keeps RSZ-0429 silent
  // so the sink level is priced exactly once.
  EXPECT_TRUE(config.cost_upstream_load);
  EXPECT_TRUE(config.cost_fanout_slew);
  EXPECT_FALSE(config.cost_global_phi);
  EXPECT_FALSE(config.cost_delta_delay);
  // F3: Flach's veto, with NO hill-climbing tolerance - the 1,000,000 weight
  // "effectively prevents" degradation, it does not schedule a tolerance for
  // it. gamma_local_slack = 0 makes Flach's Eq. 14 gamma == 1 from iteration 1.
  EXPECT_EQ(config.downsize_guard, DownsizeGuard::kLocalSlackVeto);
  EXPECT_FLOAT_EQ(config.gamma_local_slack, 0.0f);
  EXPECT_FLOAT_EQ(GlobalSizingConfig{}.gamma_local_slack, 1.0f)
      << "the struct default is Flach's Eq. 14; chinnery is the one preset "
         "that opts out of the hill-climbing half";
  // H1: the battery and the paper's five constants + 72 h cap, restated rather
  // than inherited (the reimann max_iterations precedent).
  EXPECT_EQ(config.termination, TerminationKind::kThresholdBattery);
  EXPECT_FLOAT_EQ(config.term_tns_target_frac, 0.10f);
  EXPECT_FLOAT_EQ(config.term_wns_target_frac, 0.01f);
  EXPECT_FLOAT_EQ(config.term_tns_improve_frac, 0.10f);
  EXPECT_FLOAT_EQ(config.term_power_improve_frac, 0.01f);
  EXPECT_EQ(config.term_improve_window, 3);
  EXPECT_FLOAT_EQ(config.term_wall_limit_s, 259200.0f);
  // H2: no best-so-far. The only roll-back in the paper is a host-flow step
  // outside the sizer; what the sizer itself carries against the same failure
  // is the battery's second power-phase exit.
  EXPECT_EQ(config.best_tracker, BestTrackerKind::kNone);
  // H1 budget: the paper's own hard cap.
  EXPECT_EQ(config.max_iterations, 80);
  // Family defaults it takes unchanged, restated here because the doc block
  // claims each of them as a REPRODUCED mechanism of THIS paper: the
  // reverse-topological proportional projection (§6, verbatim), the endpoint
  // constraint of Eq. 2, and the forward-topological sequential sweep its MEE
  // scheme exists to reproduce.
  EXPECT_EQ(config.kkt_projection, KktProjection::kProportionalReverseTopo);
  EXPECT_EQ(config.mu_policy, MuPolicy::kEndpointLambda);
  EXPECT_EQ(config.sweep_engine, SweepEngineKind::kGaussSeidelTopo);
  EXPECT_EQ(config.traversal, Traversal::kForwardTopo);

  // The Tcl-visible name round-trips, suffix included (§3.3: the bare paper
  // name must not parse).
  Preset parsed = Preset::kRszBaseline;
  EXPECT_TRUE(parsePreset("chinnery_partial", parsed));
  EXPECT_EQ(parsed, Preset::kChinnery);
  EXPECT_FALSE(parsePreset("chinnery", parsed));
  EXPECT_STREQ(toString(Preset::kChinnery), "chinnery_partial");

  // And the bundle validates: it trips the guard/engine crosses' silent side
  // (gauss_seidel + veto is the pairing they were designed for), the phi/
  // fanout_slew double-pricing warning cannot fire with phi off, and the
  // battery constants are live under its own termination.
  utl::Logger logger;
  logger.redirectStringBegin();
  const bool valid = config.validate(&logger);
  const std::string out = logger.redirectStringEnd();
  EXPECT_TRUE(valid);
  EXPECT_EQ(out.find("RSZ-0429"), std::string::npos) << out;
  EXPECT_EQ(out.find("RSZ-0430"), std::string::npos) << out;
  EXPECT_EQ(out.find("RSZ-0431"), std::string::npos) << out;
  EXPECT_EQ(out.find("RSZ-0452"), std::string::npos) << out;
}

// RSZ-0452, the H1 twin of RSZ-0442/0449: the six battery constants are read by
// exactly one termination rule, so a sweep of them under any other rule
// produces bit-identical runs while RSZ-0417 echoes a different `chinnery=`
// field per run - a real within-arm variance of zero, read as data instead of
// as the misconfigured cell it is.
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
    // Defaults are never flagged - only a sweep that varies a knob is.
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

  // chinnery_partial pins all six at the paper's values under the one rule that
  // reads them, and every other preset leaves them at the struct default, so no
  // preset ever trips it.
  for (const Preset p : kAllPresets) {
    GlobalSizingConfig config;
    config.applyPreset(p);
    EXPECT_EQ(validateLog(config).find("RSZ-0452"), std::string::npos)
        << toString(p);
  }
}

// C4 item 3: RSZ-0436 pins the item-5 coherence decision. The mangiras bundle
// (state_adaptive seed + endpoint_lambda) preserves Eq. 6's endpoint boundary
// and must validate silently; the pre-flip cross (state_adaptive +
// reseed_each_iter) honors that boundary at iteration 0 and overwrites it at
// iteration 1, so validate() must warn (RSZ-0436) while still allowing the run
// - it is a legitimate "what is Eq. 6's boundary worth" ablation cell.
TEST(HConfig, Rsz0436WarnsOnlyOnTheIncoherentMangirasCross)
{
  utl::Logger logger;

  // The shipped mangiras bundle: coherent, no 436.
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

  // The pre-flip incoherent cross: warns (RSZ-0436) but still valid.
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

// F3: every preset whose paper adopts Flach's local-slack check must run it,
// not our own depth budget (which appears in no paper - it is the Jacobi-era
// construction the struct default / rsz_baseline owns).
//
// Flach states it (Alg. 4); Reimann's Alg. 1 line 10 is the same veto in form;
// Mangiras "keeps the entire LR machinery of Flach et al. unchanged"; and
// Sharma adopts it verbatim - "We apply this check as we recover power, after
// the design timing is within 1% of the [target]" (§5). sharma_seq nonetheless
// kept the depth budget until the bucket-1 fidelity pass. Chinnery cites Flach
// for it too, as a 1,000,000-weighted penalty term that "effectively prevents"
// local-slack degradation (§6/§8) - a hard veto with no hill-climbing
// tolerance, which is why chinnery_partial also pins gamma_local_slack = 0
// (see ChinneryBundle).
//
// chen / tennakoon / livramento are deliberately NOT in this list: their papers
// predate or omit the check, so what guard they should carry is a separate open
// question (they currently inherit the depth budget - see the plan's "Bucket-1
// fidelity fixes").
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

// Hardening finding #10: the 2% upsize deadband is OpenROAD's own LR-cost noise
// filter, not any paper's mechanism - every paper's LRS commits the plain
// argmin. It was an unconditional constant inside both sweep engines until the
// bucket-2 pass, so it rode all seven paper presets that then existed (the
// eighth, chinnery, was born with the argmin). rsz_baseline owns it; the
// papers restore the argmin.
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
  EXPECT_FALSE(parseBestTracker("bogus", tracker));

  // output_drc_veto (A2): both names round-trip, and toString/parse compose.
  OutputDrcVeto veto = OutputDrcVeto::kAbsolute;
  EXPECT_TRUE(parseOutputDrcVeto("relative", veto));
  EXPECT_EQ(veto, OutputDrcVeto::kRelative);
  EXPECT_TRUE(parseOutputDrcVeto("absolute", veto));
  EXPECT_EQ(veto, OutputDrcVeto::kAbsolute);
  EXPECT_FALSE(parseOutputDrcVeto("bogus", veto));
  EXPECT_TRUE(parseOutputDrcVeto(toString(OutputDrcVeto::kRelative), veto));
  EXPECT_EQ(veto, OutputDrcVeto::kRelative);

  // reimann_setpoint (C1): both names round-trip, and toString/parse compose.
  ReimannSetpoint setpoint = ReimannSetpoint::kSInit;
  EXPECT_TRUE(parseReimannSetpoint("slack_target", setpoint));
  EXPECT_EQ(setpoint, ReimannSetpoint::kSlackTarget);
  EXPECT_TRUE(parseReimannSetpoint("s_init", setpoint));
  EXPECT_EQ(setpoint, ReimannSetpoint::kSInit);
  EXPECT_FALSE(parseReimannSetpoint("bogus", setpoint));
  EXPECT_TRUE(
      parseReimannSetpoint(toString(ReimannSetpoint::kSlackTarget), setpoint));
  EXPECT_EQ(setpoint, ReimannSetpoint::kSlackTarget);
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
                                  BestTrackerKind::kReimannScore}) {
    config.best_tracker = b;
    EXPECT_NE(makeBestTracker(config), nullptr) << toString(b);
  }
}

}  // namespace
}  // namespace rsz
