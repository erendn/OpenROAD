// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <cstddef>
#include <memory>

#include "LrState.hh"

namespace rsz {

struct GlobalSizingConfig;

// Best-solution tracking across the run. recordInput() sees the netlist the
// sizer received, consider() is offered the current design and its metrics at
// the end of each iteration, and restore() reinstates the recorded best after
// the loop.
//
// The tracker also owns the loop's pass-level journal (beginLoop /
// considerPass / endLoop), because deciding which sweeps survive is a
// best-solution decision and the trackers answer it in incompatible ways: the
// snapshot trackers keep every pass and reinstate a recorded cell assignment at
// the end, while wns_pass_reject keeps the netlist at a journal checkpoint.
// Making them alternatives of one strategy ensures only one rule is active.
//
// The pass-level journal is closed in endLoop(), before restore() runs, so the
// cells restore() writes are never rolled back by it; restore() has the last
// word on the cell assignment. Nothing later undoes it either, since the
// phase-level ECO is committed unconditionally (GlobalSizingPolicy::iterate).
class BestTracker
{
 public:
  virtual ~BestTracker() = default;
  // Whether consider() reads `metrics`. False for the no-op tracker, so the
  // driver can skip the per-iteration design walk that fills them.
  virtual bool needsMetrics() const { return false; }

  // Called once with the netlist the sizer received, after its metrics are
  // taken (LrState::metrics_init) and before the init pass changes it.
  // Default: ignore it.
  virtual void recordInput(LrState& /* state */) {}

  // Open the pass-level journal. Default: one nested ECO that endLoop() commits
  // whole, i.e. every sweep is kept.
  virtual void beginLoop(LrState& state);

  // Called with each sweep's outcome, before consider(). `wns_regressed` says
  // that the sweep's WNS came out worse than before the sweep; what to do about
  // it is the tracker's policy. Returns whether the pass was rejected, which is
  // what the level-1 `accepted=` field and the RSZ-0400 accepted/rolled-back
  // counters report. The default never rejects: the papers keep every iterate
  // and select the best at the end.
  virtual bool considerPass(LrState& /* state */, bool /* wns_regressed */)
  {
    return false;
  }

  virtual void consider(LrState& state, int iter, const IterMetrics& metrics)
      = 0;

  // Close the pass-level journal. Default: commit it, keeping every sweep.
  virtual void endLoop(LrState& state);

  // Reinstate the recorded best. Returns true iff it replaced at least one cell
  // (so the driver knows whether it has to refresh parasitics + timing).
  virtual bool restore(LrState& state) = 0;
};

// No best tracking: every sweep is kept and the final iterate stands.
class NoBestTracker : public BestTracker
{
 public:
  void consider(LrState& /* state */,
                int /* iter */,
                const IterMetrics& /* metrics */) override
  {
  }
  bool restore(LrState& /* state */) override { return false; }
};

// OpenROAD's rule, used by the rsz_baseline preset: keep the netlist at the
// last sweep whose WNS matched or beat every earlier sweep's. Implemented with
// the journal: checkpoint on each such sweep and, at loop exit, undo any
// changes made after the last checkpoint.
//
// None of the LR papers rolls a sweep back when WNS gets worse (each keeps
// every iterate and picks the best at the end), and doing so works against a
// subgradient method, whose primal solution is not monotone. Only
// rsz_baseline uses it.
//
// A rejected pass is not undone right away: it stays in place so the loop can
// climb out of a local minimum, and is discarded at the end only if no later
// sweep checkpointed over it.
class WnsPassRejectTracker : public BestTracker
{
 public:
  void beginLoop(LrState& state) override;
  bool considerPass(LrState& state, bool wns_regressed) override;
  void consider(LrState& /* state */,
                int /* iter */,
                const IterMetrics& /* metrics */) override
  {
  }
  void endLoop(LrState& state) override;
  bool restore(LrState& /* state */) override { return false; }

 private:
  float best_wns_ = 0.0f;
};

// Snapshot and restore shared by the snapshot trackers: capture() records the
// current cell assignment, and restore() reinstates it.
class SnapshotBestTracker : public BestTracker
{
 public:
  bool needsMetrics() const override { return true; }
  bool restore(LrState& state) override;

 protected:
  // The iteration index capture() takes for the input solution.
  static constexpr int kInputIter = -1;

  void capture(LrState& state, int iter);
  bool hasBest() const { return has_best_; }

 private:
  bool has_best_ = false;
  int best_iter_ = kInputIter;
  CellAssignment best_cells_;
};

// Flach et al., TCAD 2014, Alg. 1 lines 9-13: an iterate replaces the stored
// best if |TNS| < best_tns_target_frac * T and its power is lower. Nothing is
// stored until an iterate qualifies, so if no iterate gets within the TNS
// limit there is nothing to restore and the final state stands (the paper does
// not cover this case).
class DominanceBestTracker : public SnapshotBestTracker
{
 public:
  void consider(LrState& state, int iter, const IterMetrics& metrics) override;

 private:
  float best_power_ = 0.0f;
};

// Reimann et al., ISPD 2016, Alg. 2: store the input solution first (line 1),
// then store an iterate whenever its Eq. 6 score beats the best so far (lines
// 14-16), and restore the best at the end (line 19). The input scores 0, so a
// run in which no iterate scores above 0 returns the input.
class ScoreBestTracker : public SnapshotBestTracker
{
 public:
  void recordInput(LrState& state) override;
  void consider(LrState& state, int iter, const IterMetrics& metrics) override;

 private:
  float best_score_ = 0.0f;
};

// Livramento et al., DATE 2013, Alg. 1 line 20: "return best V' without
// violations over all iterations". An iterate is free of violations when it
// meets setup timing and OpenSTA reports no max capacitance or max slew
// violation, counting the same limits as report_check_types. Of those
// iterates, the one with the lowest power is restored, and a tie keeps the
// earlier one. As in DominanceBestTracker, only iterates are considered, not
// the input netlist.
//
// The paper does not cover a run in which no iterate is free of violations.
// Nothing is stored then, so the final iterate stands; when the cap fix pass
// runs after every sweep, that iterate keeps all of its repairs.
//
// restore() reports how many iterates were free of violations and which one
// the run returns (RSZ-0460).
//
// FeasibleSelection holds the choice, apart from the STA, so that it can be
// tested on hand values. offer() takes one iterate's end-of-iteration state
// and returns whether that iterate becomes the stored best.
struct FeasibleSelection
{
  bool offer(int iter,
             float wns,
             size_t max_cap_violations,
             size_t max_slew_violations,
             float power);
  bool hasBest() const { return best_iter >= 0; }

  int iterations = 0;
  int violation_free_count = 0;
  int best_iter = -1;
  float best_power = 0.0f;
};

class FeasibleBestTracker : public SnapshotBestTracker
{
 public:
  void consider(LrState& state, int iter, const IterMetrics& metrics) override;
  bool restore(LrState& state) override;

 private:
  FeasibleSelection selection_;
};

// === Formulas (unit-tested against hand-computed values) ==================

// Flach's dominance test. `T` is the clock period; a non-positive T (no clock)
// disqualifies every iterate, since the TNS gate has no scale.
bool flachDominates(float tns,
                    float power,
                    float T,
                    float tns_target_frac,
                    bool have_best,
                    float best_power);

// Livramento's violation-free test: WNS >= 0 and no max capacitance or max
// slew violation.
bool livramentoViolationFree(float wns,
                             size_t max_cap_violations,
                             size_t max_slew_violations);

// Whether an iterate replaces the stored best under livramento_feasible: it
// must be free of violations and, if a best is stored, have strictly lower
// power.
bool livramentoReplacesBest(bool violation_free,
                            float power,
                            bool have_best,
                            float best_power);

// Reimann et al., Eq. 6:
//
//   score = -( dPower + dArea + 2^-(dTV + dWNS) - 1 )
//
// The paper does not spell out the sign conventions; these are the ones its
// described behavior requires, with deltas relative to the input solution:
//   d_power / d_area : relative change, so negative means improvement;
//   d_tv / d_wns     : relative improvement, so negative means degradation.
// Check: an unchanged solution scores 0 (all deltas 0 -> -(2^0 - 1) = 0); a 10%
// power reduction with unchanged timing scores +0.1; and timing degradation
// makes (dTV + dWNS) negative, so 2^-(...) grows quickly and the score becomes
// strongly negative. This is the paper's "small window of compromise": it
// tolerates picosecond-scale WNS noise but rejects real degradation.
float reimannScore(float d_power, float d_area, float d_tv, float d_wns);

// The four Eq. 6 deltas of `cur` against the input solution `init`, with the
// sign conventions above. TNS is compared as |TNS| (the "timing violation");
// WNS improvement is measured against |WNS_init| so it is dimensionless.
// Metrics with a zero reference contribute a zero delta.
struct ScoreDeltas
{
  float d_power = 0.0f;
  float d_area = 0.0f;
  float d_tv = 0.0f;
  float d_wns = 0.0f;
};
ScoreDeltas scoreDeltas(const IterMetrics& init, const IterMetrics& cur);

std::unique_ptr<BestTracker> makeBestTracker(const GlobalSizingConfig& config);

}  // namespace rsz
