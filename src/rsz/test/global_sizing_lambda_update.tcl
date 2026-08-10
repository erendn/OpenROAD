# M1 smoke test: exercise every lambda_update option of GlobalSizingPolicy on a
# small design. For each option, set_global_sizing_config -lambda_update selects
# the updater (all other axes stay at the rsz_baseline defaults), the
# GLOBAL_SIZING phase runs, and the worst slack is reported. This asserts each
# updater's full update() path (STA reads + graph traversal) runs without error
# and produces the RSZ-0417/0400/0409 run record; the per-formula arithmetic is
# checked by the TestLambdaUpdater gtest. Single-threaded for a deterministic
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

foreach update {
  norm_subgradient
  flach_slack_scaling
  chen_subgradient
  tennakoon_ratio
  sharma_cexp
  reimann_dwns
  livramento_ratio
} {
  puts "=== lambda_update $update ==="
  set_global_sizing_config -lambda_update $update
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}
