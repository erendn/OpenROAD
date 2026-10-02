# Smoke test: exercise every mu_policy (endpoint multiplier) option of
# GlobalSizingPolicy on a small design. For each option,
# set_global_sizing_config -mu_policy selects the endpoint treatment (all other
# options stay at the rsz_baseline defaults), the GLOBAL_SIZING phase runs, and
# the worst slack is reported. This checks that each policy's full path
# (applyMuPolicy and the projection's endpoint derive/anchor branch) runs
# without error and produces the RSZ-0417/0400/0409 run records; the formulas
# and the derive-then-anchor sequence are checked by the TestLambdaUpdater and
# TestFlowProjection unit tests. Single-threaded for a deterministic golden.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup2.def
read_sdc repair_setup2.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

foreach policy {
  reseed_each_iter
  seed_once
  update_as_lambda
  endpoint_lambda
  endpoint_ratio
  endpoint_additive
} {
  puts "=== mu_policy $policy ==="
  set_global_sizing_config -mu_policy $policy
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}
