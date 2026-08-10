// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for the A2 axis' pure core: outputLimitAdmits, the per-pin
// output-side DRC verdict that `output_drc_veto` selects between
// (lr/ElectricalModel.hh), plus the slew-estimator identity the relative mode's
// comparison basis rests on.
//
// The STA-facing halves - reading a Liberty cap/slew limit, and the sweep
// wiring in LRSubproblem::candidateDrcOkSnapshot - are exercised by the
// global_sizing_init_min_size_fixviol integration test, whose legs 3/4 are the
// end-to-end A/B: a gate the repair cannot fix is frozen at minimum under
// absolute and climbs out under relative. What is unit-tested here is the
// admission rule itself, because that is where the two modes differ and where a
// silent `<` for `<=` would change what a paper preset measures.

#include <cmath>

#include "gtest/gtest.h"
#include "lr/ElectricalModel.hh"

namespace rsz {
namespace {

// Readability: the two modes as named arguments rather than bare bools.
constexpr bool kAbsolute = false;
constexpr bool kRelative = true;

// A clean pin (the current cell is within its limit, excess 0) must behave
// IDENTICALLY under both modes - the mode only ever governs an already-
// violating pin. This is what keeps `relative` from being a blanket relaxation:
// it cannot create a violation where there was none.
TEST(OutputVeto, CleanPinBehavesIdenticallyUnderBothModes)
{
  const float clean = 0.0f;
  for (const float cand : {0.0f, 1e-15f, 1e-12f}) {
    EXPECT_EQ(outputLimitAdmits(cand, clean, kAbsolute),
              outputLimitAdmits(cand, clean, kRelative))
        << "candidate excess " << cand;
  }
  // ... and that shared behaviour is the absolute rule: only a clean candidate.
  EXPECT_TRUE(outputLimitAdmits(0.0f, clean, kRelative));
  EXPECT_FALSE(outputLimitAdmits(1e-15f, clean, kRelative));
}

// absolute: the bar is zero excess, whatever the current cell was doing. This
// is the shipped default and the pre-A2 behaviour, so a violating pin rejects
// every candidate that does not fully clear - including one that IMPROVES the
// violation. That freeze-out is the axis' subject.
TEST(OutputVeto, AbsoluteAdmitsOnlyCleanCandidates)
{
  const float dirty = 5e-15f;
  EXPECT_TRUE(outputLimitAdmits(0.0f, dirty, kAbsolute));
  EXPECT_FALSE(outputLimitAdmits(1e-15f, dirty, kAbsolute));  // improving
  EXPECT_FALSE(outputLimitAdmits(dirty, dirty, kAbsolute));   // equal
  EXPECT_FALSE(outputLimitAdmits(9e-15f, dirty, kAbsolute));  // worsening
}

// relative: Flach Alg. 4 line 6 / Chinnery §6. On a pin the current cell
// already violates, anything that does not violate it by MORE is admitted -
// which is exactly the climb-out the absolute mode forbids.
TEST(OutputVeto, RelativeAdmitsImprovingCandidatesOnADirtyPin)
{
  const float dirty = 5e-15f;
  EXPECT_TRUE(outputLimitAdmits(0.0f, dirty, kRelative));    // fully clears
  EXPECT_TRUE(outputLimitAdmits(1e-15f, dirty, kRelative));  // improving
  EXPECT_TRUE(outputLimitAdmits(4.99e-15f, dirty, kRelative));
}

// TIE SEMANTICS, stated in outputLimitAdmits' doc and pinned here because the
// papers' wording ("has INCREASED", "would INCREASE") is what decides it: an
// equal-violation candidate is admitted. A Vth swap at equal drive resistance
// is the move this buys, and a `<` here would silently forbid it.
TEST(OutputVeto, RelativeAdmitsAnEqualViolation)
{
  const float dirty = 5e-15f;
  EXPECT_TRUE(outputLimitAdmits(dirty, dirty, kRelative));
  EXPECT_FALSE(outputLimitAdmits(dirty, dirty, kAbsolute));
}

// A candidate that makes an existing violation worse is rejected under BOTH
// modes. relative is a relaxation of which candidates may be considered, never
// of the direction: no mode lets a violation grow.
TEST(OutputVeto, WorseningIsRejectedUnderBothModes)
{
  const float dirty = 5e-15f;
  const float worse = std::nextafter(dirty, 1.0f);
  EXPECT_FALSE(outputLimitAdmits(worse, dirty, kRelative));
  EXPECT_FALSE(outputLimitAdmits(worse, dirty, kAbsolute));
  EXPECT_FALSE(outputLimitAdmits(1e-12f, dirty, kRelative));
}

// DECISION (a), the relative mode's slew comparison basis: the incumbent's
// figure is computed through the SAME estimator the candidates are, not read
// off the STA graph. That is only like-for-like because the calibration is
// defined to reproduce the measurement at the current port - `factor * R * C`
// with the CURRENT R and C is the measured slew, exactly. Pinned here because
// the whole "did it worsen?" comparison rests on it.
TEST(OutputVeto, SlewEstimatorReproducesTheIncumbentMeasurement)
{
  const float slew = 8e-11f;
  const float drive_res = 4000.0f;
  const float load = 5e-15f;
  const float factor = outputSlewFactor(slew, drive_res, load);
  EXPECT_FLOAT_EQ(factor * drive_res * load, slew);
}

// ... and where the calibration is degenerate (no load, or no drive
// resistance), the estimator abstains for the incumbent exactly as it does for
// every candidate: the factor is 0, so both sides read a zero slew, the pin
// reads clean, and the relative rule is inert precisely where the model has
// nothing to say.
TEST(OutputVeto, DegenerateSlewCalibrationAbstains)
{
  EXPECT_FLOAT_EQ(outputSlewFactor(8e-11f, 0.0f, 5e-15f), 0.0f);
  EXPECT_FLOAT_EQ(outputSlewFactor(8e-11f, 4000.0f, 0.0f), 0.0f);
  // A zero estimate is within any limit, so the pin is clean under both modes.
  EXPECT_TRUE(outputLimitAdmits(0.0f, 0.0f, kAbsolute));
  EXPECT_TRUE(outputLimitAdmits(0.0f, 0.0f, kRelative));
}

}  // namespace
}  // namespace rsz
