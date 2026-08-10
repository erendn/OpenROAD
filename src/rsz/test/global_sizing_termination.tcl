# M5 smoke test: the H axis of GlobalSizingPolicy on a small design -
# termination (H1: fixed_iters, stagnation_windows, threshold_battery) and
# best-solution tracking (H2: none, flach_dominance, reimann_score), plus the
# paper presets that carry them and the default-flip check.
#
# What this asserts: each option's full path runs and produces the RSZ-0417/
# 0400/0409 run record (threshold_battery adds RSZ-0450/0451, its phase-boundary
# and stop records); the best trackers snapshot and restore a cell assignment
# without disturbing the phase's journal bookkeeping (a broken restore would
# show up as a corrupted QoR line or a crash). The termination PREDICATES
# (Sharma's window rule, Chinnery's battery), the dominance/score predicates and
# the dual estimate are unit-tested against hand-computed values by
# TestTermination.
#
# The default-flip check at the bottom is the point of the file: with no preset
# and no -best_tracker, GLOBAL_SIZING now keeps and restores its best iterate
# (flach_dominance), which is M5's one deliberate default-behavior change. The
# rsz_baseline preset stays pinned to none.
#
# OpenROAD Tcl has no in-session netlist reset, so the configurations run
# sequentially on one netlist: run 1 optimizes, later runs start from that state
# and mostly no-op. Each configuration still dispatches its rule (RSZ-0417).
# Single-threaded for a deterministic golden.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup2.def
read_sdc repair_setup2.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

# H1: the three termination rules. The paper constants come from the struct
# defaults; the tightened ones below force the rule to fire on this small
# design (which converges in a handful of iterations).
#
# stagnation_windows runs FIRST, on the un-optimized netlist, so the golden pins
# a real run of a new stop rule. Note it carries only Sharma's rule - neither of
# fixed_iters' legacy early exits (3 rejects, 2 zero-move passes) - so it keeps
# sweeping until a window stagnates, which is visible in its RSZ-0400 sweep
# count. (fixed_iters had a THIRD legacy exit, WNS-meets-margin, removed on
# 2026-07-29; see the post-feasible pair below.)
foreach {label args} {
  stagnation_sharma   {-termination stagnation_windows}
  stagnation_mangiras {-termination stagnation_windows -stagnation_window 2 -stagnation_count 1 -stagnation_improve_frac 0.01 -stagnation_require_tns 1}
  threshold_battery   {-termination threshold_battery}
  threshold_tight     {-termination threshold_battery -term_tns_improve_frac 0.5 -term_improve_window 1}
  fixed_iters         {-termination fixed_iters}
  pure_cap            {-termination pure_cap -max_iterations 6}
} {
  puts "=== termination $label ==="
  reset_global_sizing_config
  eval set_global_sizing_config $args
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}

# The post-feasible pair. Both legs run with a widened setup margin (-1 ns) so the
# design is FEASIBLE AGAINST THAT MARGIN from the start - note it is NOT met in
# absolute terms (this design enters at WNS -0.157 and the golden still reports
# violating endpoints); what matters is that WNS >= setup_slack_margin, which is
# exactly the condition the removed exit tested. That is where the H1 axis used to
# hide a hard no-op: fixed_iters' WNS-meets-margin exit fired before iteration 0
# and reported "0/0 sweeps accepted, 0 cells replaced". It was removed on
# 2026-07-29 (pre-campaign engine fix 2), so BOTH legs must now sweep - the
# leakage-recovery regime the papers exist for is reachable from either option.
# Genuinely-met coverage (WNS > 0 absolutely) lives in global_sizing_met_recovery. The RSZ-0400 sweep
# counts are the evidence: fixed_iters used to report 0/0 here and now enters its
# loop. The only remaining difference between the two is that fixed_iters can
# still stop early on its two MOVE-driven exits (3 rejects / 2 zero-move passes),
# which fire after a sweep has run rather than instead of one.
#
# Leg ORDER is load-bearing and deliberate. These legs run sequentially on one
# netlist (no in-session reset), and the first one to reach the post-feasible
# regime takes the available downsizes; the second then has nothing left to move.
# pure_cap goes FIRST so it keeps carrying real QoR content (its −50 % leakage is
# this file's evidence that a post-feasible run is productive at all), and
# fixed_iters follows carrying the assertion this fix needs - that its loop is
# ENTERED (nonzero sweeps) rather than skipped. fixed_iters' own move-and-keep
# evidence on a met design lives in global_sizing_met_recovery leg 1, which runs
# rsz_baseline (the one shipped fixed_iters preset) on a pristine met netlist
# through the real ORFS command path. Do not reorder these two without moving
# that evidence with them.
puts "=== termination pure_cap post-feasible (runs to the cap) ==="
reset_global_sizing_config
set_global_sizing_config -termination pure_cap -max_iterations 4 \
  -setup_slack_margin -1
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

puts "=== termination fixed_iters post-feasible (no longer amputated) ==="
reset_global_sizing_config
set_global_sizing_config -termination fixed_iters -max_iterations 4 \
  -setup_slack_margin -1
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# H2: the three best trackers. best_tns_target_frac widens/narrows the TNS gate
# a dominance snapshot must pass.
foreach {label args} {
  best_none        {-best_tracker none}
  best_dominance   {-best_tracker flach_dominance}
  best_dominance_wide {-best_tracker flach_dominance -best_tns_target_frac 10}
  best_score       {-best_tracker reimann_score}
} {
  puts "=== best_tracker $label ==="
  reset_global_sizing_config
  eval set_global_sizing_config $args
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}

# The paper presets carrying an H rule: sharma_seq_partial (stagnation
# windows), mangiras_partial (its 1%-over-2-iterations convergence rule -
# window 1 / count 1 as of the bucket-1 fidelity pass), reimann_partial (score
# tracker + pure_cap), livramento_partial (dominance + pure_cap).
#
# Their C2 budgets are the papers' (sharma 160, livramento 60 to the cap under
# pure_cap), and sharma's stop is now gated on the 1% near-met latch - on a
# stuck-violating netlist it never latches, so it would run to 160. This smoke
# only needs the rule to dispatch, so cap every preset back to a handful of
# iterations. The override must come AFTER -preset (order-sensitive by design).
#
# sharma_seq_partial and livramento_partial also carry an init pass, so their
# legs open with RSZ-0416/0415 (plus RSZ-0445 for sharma's min_size_fixviol) and
# sharma's RSZ-0400 count is the round trip that creates - reset to minimum,
# repair the two gates that then violate max-cap, LR upsizes back to the
# incoming netlist (RSZ-0409 +0.00%). The sweep counts, which are what this file
# pins, do not move.
foreach preset {sharma_seq_partial mangiras_partial reimann_partial \
  livramento_partial} {
  puts "=== preset $preset ==="
  reset_global_sizing_config
  set_global_sizing_config -preset $preset
  set_global_sizing_config -max_iterations 6
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}

# The M5 default flip: no preset, no -best_tracker -> flach_dominance (see the
# best= field of RSZ-0417). rsz_baseline stays pinned to none.
puts "=== default config (best=flach_dominance) ==="
reset_global_sizing_config
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

puts "=== preset rsz_baseline (best=none) ==="
reset_global_sizing_config
set_global_sizing_config -preset rsz_baseline
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# The config mirror: every M5 knob must be reported.
puts "=== report_global_sizing_config ==="
reset_global_sizing_config
set_global_sizing_config -termination stagnation_windows \
  -best_tracker reimann_score -stagnation_window 3 -stagnation_count 4 \
  -stagnation_improve_frac 0.02 -stagnation_require_tns 1 \
  -near_met_gate_frac 0.05 \
  -term_tns_target_frac 0.2 -term_wns_target_frac 0.02 \
  -term_tns_improve_frac 0.15 -term_power_improve_frac 0.03 \
  -term_improve_window 2 -term_wall_limit_s 3600 -best_tns_target_frac 0.25
report_global_sizing_config
