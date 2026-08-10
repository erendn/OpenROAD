// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "LrState.hh"
#include "sta/LibertyClass.hh"
#include "sta/NetworkClass.hh"

namespace rsz {

struct GlobalSizingConfig;

// H2 axis - best-solution tracking across the run. consider() is offered the
// live design and its metrics at the end of each iteration; restore()
// reinstates the recorded best once the loop is over.
//
// This axis owns the LR loop's pass-level journal (beginLoop / considerPass /
// endLoop). That is deliberate: "which of the sweeps I just ran survives" is a
// best-solution decision, and the options answer it in incompatible ways - the
// snapshot trackers keep every pass and reinstate a recorded cell assignment at
// the end, while wns_pass_reject keeps the netlist at a journal checkpoint. An
// enum makes them exclusive by construction, which is the point: through M5 the
// checkpoint rule lived in the shared driver and ran *underneath* the snapshot
// trackers, so every paper preset silently carried OpenROAD's best-WNS rule on
// top of its own paper's rule (post-M5 hardening pass).
//
// Journal interaction (why restore() is called where it is): the pass-level
// journal closes in endLoop(), before restore() runs, so the cells this axis
// writes are never rolled back by it - which is what we want, since restore()'s
// whole job is to have the last word on the cell assignment. Nothing downstream
// can undo it either: the phase-level ECO is committed unconditionally
// (GlobalSizingPolicy::iterate). Until 2026-07-29 that was not true - the phase
// ended with a WNS accept that could revert a restored best wholesale - and
// removing it is what makes this axis's verdict final.
class BestTracker
{
 public:
  virtual ~BestTracker() = default;
  // Whether consider() reads `metrics`. False for the no-op tracker, so the
  // driver can skip the per-iteration design walk that fills them.
  virtual bool needsMetrics() const { return false; }

  // Open the pass-level journal. Default: one nested ECO that endLoop() commits
  // whole, i.e. every sweep is kept.
  virtual void beginLoop(LrState& state);

  // Offered each sweep's outcome, before consider(). `wns_regressed` is the
  // measurement "this sweep's WNS came out worse than the pre-sweep WNS"; what
  // to do about it is this axis's policy. Returns whether the pass was
  // *rejected* by that policy, which is what the level-1 `accepted=` field and
  // the RSZ-0400 accepted/rolled-back counters report. Default: never reject
  // (the papers commit every iterate unconditionally and select at the end).
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

// No best tracking at all: every sweep is kept and the final iterate stands.
// Measures what the tracking rules are worth.
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

// rsz_baseline's best-solution rule: keep the netlist at the last sweep whose
// WNS matched or beat every previous sweep's. Realized through the journal -
// checkpoint on each such sweep, and undo the drift past the last checkpoint at
// loop exit - which is what the pre-M5 driver did inline, relocated here
// verbatim so rsz_baseline stays byte-identical.
//
// Not a paper mechanism: no LR paper in the collection rolls a sweep back on
// WNS regression (each commits every iterate and selects a best at the end),
// and doing so fights a dual-subgradient method, whose primal is not monotone
// by design. Hence rsz_baseline pins it and nothing else does.
//
// Note a rejected pass is NOT immediately undone here either - it stays live so
// the loop can climb hills; it is only discarded at the end if no later sweep
// checkpointed over it.
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

// Shared snapshot/restore for the two real trackers: capture() records the
// current cell of every editable leaf instance, restore() replaces back only
// the instances whose cell has since drifted from the record.
class SnapshotBestTracker : public BestTracker
{
 public:
  bool needsMetrics() const override { return true; }
  bool restore(LrState& state) override;

 protected:
  void capture(LrState& state, int iter);
  bool hasBest() const { return best_iter_ >= 0; }

  int best_iter_ = -1;

 private:
  std::vector<std::pair<sta::Instance*, sta::LibertyCell*>> best_cells_;
};

// Flach TCAD'14 Alg. 1 lines 9-13: an iterate displaces the stored best iff
// |TNS| < best_tns_target_frac * T AND its leakage is lower. Nothing is stored
// until an iterate qualifies, so on a design LR never brings within the TNS
// gate there is no restore and the final state stands (the paper leaves this
// case open - flach_et_al.md §13.6).
class DominanceBestTracker : public SnapshotBestTracker
{
 public:
  void consider(LrState& state, int iter, const IterMetrics& metrics) override;

 private:
  float best_leakage_ = 0.0f;
};

// Reimann ISPD'16 Alg. 2 lines 14-16: store whenever the Eq. 6 score improves
// on the best score so far. The input solution scores 0 by construction, so the
// tracker only ever stores an iterate that beats it.
class ScoreBestTracker : public SnapshotBestTracker
{
 public:
  void consider(LrState& state, int iter, const IterMetrics& metrics) override;

 private:
  float best_score_ = 0.0f;
};

// === Pure cores (unit-tested against hand-computed values) ==================

// Flach's dominance test. `T` is the clock period; a non-positive T (no clock)
// disqualifies every iterate, since the TNS gate has no scale.
bool flachDominates(float tns,
                    float leakage,
                    float T,
                    float tns_target_frac,
                    bool have_best,
                    float best_leakage);

// Reimann Eq. 6:
//
//   score = -( dPower + dArea + 2^-(dTV + dWNS) - 1 )
//
// The paper's sign conventions are not spelled out; these are the ones its
// described behavior forces (flach-style deltas relative to the *input*
// solution):
//   d_power / d_area : relative CHANGE, so negative == improvement;
//   d_tv / d_wns     : relative IMPROVEMENT, so negative == degradation.
// Check: an unchanged solution scores 0 (all deltas 0 -> -(2^0 - 1) = 0); a 10%
// power win with unchanged timing scores +0.1; and timing degradation drives
// (dTV + dWNS) negative, which blows 2^-(...) up and the score strongly
// negative - the paper's "small window of compromise" that tolerates
// picosecond-scale WNS noise but rejects real degradation.
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
