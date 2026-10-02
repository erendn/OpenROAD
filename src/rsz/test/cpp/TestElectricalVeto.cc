// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

// Unit tests for outputLimitAdmits (lr/ElectricalModel.hh), the per-pin
// output-side max-cap / max-slew check selected by -output_drc_veto, and for
// the slew calibration the relative mode compares against.
//
// Reading Liberty limits and the sweep's candidate filter
// (LRSubproblem::candidateDrcOkSnapshot) are covered by the
// global_sizing_init_min_size_fixviol integration test, which compares the
// modes on a gate the initial repair cannot fix: under absolute the gate stays
// at minimum size, under relative it can be upsized. The admission
// rule itself is tested here, because it is where the two modes differ and
// where using `<` instead of `<=` would silently change the behavior.

#include <cmath>

#include "gtest/gtest.h"
#include "lr/ElectricalModel.hh"

namespace rsz {
namespace {

// Named values for the `relative` argument.
constexpr bool kAbsolute = false;
constexpr bool kRelative = true;

// On a clean pin (the current cell is within its limit, excess 0) both modes
// must give the same answer. The mode only matters on a pin that already
// violates, so relative can never create a new violation.
TEST(OutputVeto, CleanPinBehavesIdenticallyUnderBothModes)
{
  const float clean = 0.0f;
  for (const float cand : {0.0f, 1e-15f, 1e-12f}) {
    EXPECT_EQ(outputLimitAdmits(cand, clean, kAbsolute),
              outputLimitAdmits(cand, clean, kRelative))
        << "candidate excess " << cand;
  }
  // The shared behavior is the absolute rule: only a clean candidate passes.
  EXPECT_TRUE(outputLimitAdmits(0.0f, clean, kRelative));
  EXPECT_FALSE(outputLimitAdmits(1e-15f, clean, kRelative));
}

// Absolute mode (the default) admits only a candidate with zero excess,
// whatever the current cell does. On a violating pin this rejects every
// candidate that does not fully clear the violation, including one that
// reduces it, so the gate can get stuck at its current cell.
TEST(OutputVeto, AbsoluteAdmitsOnlyCleanCandidates)
{
  const float dirty = 5e-15f;
  EXPECT_TRUE(outputLimitAdmits(0.0f, dirty, kAbsolute));
  EXPECT_FALSE(outputLimitAdmits(1e-15f, dirty, kAbsolute));  // improving
  EXPECT_FALSE(outputLimitAdmits(dirty, dirty, kAbsolute));   // equal
  EXPECT_FALSE(outputLimitAdmits(9e-15f, dirty, kAbsolute));  // worsening
}

// Relative mode (Flach et al., TCAD 2014, Alg. 4 line 6; Chinnery and Sharma,
// ISPD 2022): on a pin the current cell already violates, any candidate that
// does not violate it by more is admitted. This lets the gate move out of a
// violation it cannot clear in one step.
TEST(OutputVeto, RelativeAdmitsImprovingCandidatesOnADirtyPin)
{
  const float dirty = 5e-15f;
  EXPECT_TRUE(outputLimitAdmits(0.0f, dirty, kRelative));    // fully clears
  EXPECT_TRUE(outputLimitAdmits(1e-15f, dirty, kRelative));  // improving
  EXPECT_TRUE(outputLimitAdmits(4.99e-15f, dirty, kRelative));
}

// A candidate with an equal violation is admitted. Both papers reject only a
// candidate that increases the violation. This keeps a Vth swap at equal drive
// resistance available; using `<` would forbid it.
TEST(OutputVeto, RelativeAdmitsAnEqualViolation)
{
  const float dirty = 5e-15f;
  EXPECT_TRUE(outputLimitAdmits(dirty, dirty, kRelative));
  EXPECT_FALSE(outputLimitAdmits(dirty, dirty, kAbsolute));
}

// A candidate that makes an existing violation worse is rejected in both
// modes. Relative mode relaxes which candidates are allowed, but never lets a
// violation grow.
TEST(OutputVeto, WorseningIsRejectedUnderBothModes)
{
  const float dirty = 5e-15f;
  const float worse = std::nextafter(dirty, 1.0f);
  EXPECT_FALSE(outputLimitAdmits(worse, dirty, kRelative));
  EXPECT_FALSE(outputLimitAdmits(worse, dirty, kAbsolute));
  EXPECT_FALSE(outputLimitAdmits(1e-12f, dirty, kRelative));
}

// In relative mode the current cell's slew is computed with the same estimator
// as the candidates' slews, not read from the timing graph. That comparison is
// only fair because the calibration factor is defined so that
// `factor * R * C`, with the current R and C, reproduces the measured slew
// exactly.
TEST(OutputVeto, SlewEstimatorReproducesTheIncumbentMeasurement)
{
  const float slew = 8e-11f;
  const float drive_res = 4000.0f;
  const float load = 5e-15f;
  const float factor = outputSlewFactor(slew, drive_res, load);
  EXPECT_FLOAT_EQ(factor * drive_res * load, slew);
}

// With no load or no drive resistance the calibration is degenerate and the
// factor is 0. The current cell and every candidate then read a zero slew, the
// pin reads clean, and the relative rule has no effect.
TEST(OutputVeto, DegenerateSlewCalibrationAbstains)
{
  EXPECT_FLOAT_EQ(outputSlewFactor(8e-11f, 0.0f, 5e-15f), 0.0f);
  EXPECT_FLOAT_EQ(outputSlewFactor(8e-11f, 4000.0f, 0.0f), 0.0f);
  // A zero estimate is within any limit, so the pin is clean in both modes.
  EXPECT_TRUE(outputLimitAdmits(0.0f, 0.0f, kAbsolute));
  EXPECT_TRUE(outputLimitAdmits(0.0f, 0.0f, kRelative));
}

}  // namespace
}  // namespace rsz
