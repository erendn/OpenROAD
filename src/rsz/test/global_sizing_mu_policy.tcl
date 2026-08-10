# C1 smoke test: exercise every mu_policy (E4 endpoint-multiplier) option of
# GlobalSizingPolicy on a small design. For each option,
# set_global_sizing_config -mu_policy selects the endpoint treatment (all other
# axes stay at the rsz_baseline defaults), the GLOBAL_SIZING phase runs, and the
# worst slack is reported. This asserts each policy's full path (applyMuPolicy +
# the projection's endpoint derive/anchor branch) runs without error and
# produces the RSZ-0417/0400/0409 run record; the per-formula arithmetic and the
# derive-then-anchor lifecycle are checked by the TestLambdaUpdater /
# TestFlowProjection gtests. The two C1 additions (endpoint_ratio,
# endpoint_additive) are the point of this smoke - the M1 deviation-7 per-axis
# integration test the E4 axis deferred to its consuming milestone.
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
