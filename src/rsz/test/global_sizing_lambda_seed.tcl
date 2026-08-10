# M2 smoke test: exercise every lambda_seed option of GlobalSizingPolicy on a
# small design. For each option, set_global_sizing_config -lambda_seed selects
# the seeder (all other axes stay at the rsz_baseline defaults), the
# GLOBAL_SIZING phase runs, and the worst slack is reported. This asserts each
# seeder's full seed() path (STA reads + graph traversal) - and, for
# estimation_loop, the dry-run estimation pre-pass - runs without error and
# produces the RSZ-0417/0400/0409 run record; the per-formula arithmetic is
# checked by the TestLambdaSeeder gtest. Single-threaded for a deterministic
# golden.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup2.def
read_sdc repair_setup2.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

foreach seed {
  delay_proportional_crit_mu
  constant
  state_adaptive
  estimation_loop
} {
  puts "=== lambda_seed $seed ==="
  set_global_sizing_config -lambda_seed $seed
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}
