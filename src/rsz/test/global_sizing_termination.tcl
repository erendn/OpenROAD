# Smoke test for -termination (fixed_iters, stagnation_windows,
# threshold_battery, pure_cap) and -best_tracker (none, flach_dominance,
# reimann_score), the paper presets that use them, and the default best
# tracker. Each run must complete and print the RSZ-0417/0400/0409 records; a
# broken best-tracker restore would show up as a wrong QoR line or a crash.
# The stop rules and tracker tests are unit-tested by TestTermination.
#
# The runs share one netlist: the first optimizes it and later runs mostly make
# no moves, but each still selects its rule (RSZ-0417). Single-threaded for a
# deterministic golden.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup2.def
read_sdc repair_setup2.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

# The termination rules; the tightened values make them fire on this small
# design. stagnation_windows runs first, on the unoptimized netlist. It has
# none of fixed_iters' early exits (3 rejects, 2 passes with no moves), so it
# sweeps until a window stagnates.
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

# With -setup_slack_margin -1, WNS meets the margin from the start (the design
# still violates in absolute terms). Both runs must still sweep: no rule may
# stop before the first sweep just because the margin is met. pure_cap runs
# first and takes the available downsizes; fixed_iters then checks that its
# loop is entered (nonzero sweeps). global_sizing_met_recovery.tcl covers
# designs that meet timing outright.
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

# The best trackers. -best_tns_target_frac widens or narrows the TNS bound a
# dominance snapshot must pass.
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

# Paper presets with a termination rule or best tracker. They use the papers'
# iteration budgets (160 for sharma, whose stop also waits for a near-met latch
# that never fires on this design), so cap them at 6 iterations; the override
# must come after -preset. The init passes of sharma_seq_partial and
# livramento_partial add RSZ-0416/0415 (and RSZ-0445 and RSZ-0461) records.
foreach preset {sharma_seq_partial mangiras_partial reimann_partial \
  livramento_partial} {
  puts "=== preset $preset ==="
  reset_global_sizing_config
  set_global_sizing_config -preset $preset
  set_global_sizing_config -max_iterations 6
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}

# With no preset and no -best_tracker the tracker is flach_dominance (best= in
# RSZ-0417); rsz_baseline uses none.
puts "=== default config (best=flach_dominance) ==="
reset_global_sizing_config
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

puts "=== preset rsz_baseline (best=none) ==="
reset_global_sizing_config
set_global_sizing_config -preset rsz_baseline
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# report_global_sizing_config must list every termination and best-tracker
# option.
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
