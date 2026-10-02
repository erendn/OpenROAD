# Post-sweep max-capacitance re-check with the Gauss-Seidel engine. Same design
# and setup as global_sizing_cap_recheck.tcl; only -sweep_engine differs.
#
# Gauss-Seidel snapshots each gate just before evaluating it, after parasitics
# are updated for earlier commits, so the fifth buffer already sees the load
# added by the first four and its own check rejects the move: 4 replacements
# attempted, 0 reverted, same end state as Jacobi. The re-check still runs
# because a gate cannot see moves committed later in the same sweep.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def global_sizing_cap_recheck.def
create_clock -period 0.20 clk

set_dont_touch src
set_load 53 [get_nets n]
foreach net {m0 m1 m2 m3 m4 m5} {
  set_load 40 [get_nets $net]
}

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

puts "=== before: 1.88 fF of headroom on the driver's net ==="
report_check_types -max_capacitance

set_global_sizing_config -sweep_engine gauss_seidel_topo
repair_timing -setup -phases GLOBAL_SIZING

puts "=== after: the sequential engine never overshot, so nothing was reverted ==="
report_check_types -max_capacitance
report_worst_slack -max -digits 3
