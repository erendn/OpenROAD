# The post-sweep max-cap re-check (iteration-2 plan §2.2-1), Gauss-Seidel leg.
#
# The A/B partner of global_sizing_cap_recheck.tcl: same design, same loads, same
# clock, only -sweep_engine differs. Compare the two goldens line for line.
#
# WHAT THIS PINS - that the Gauss-Seidel JIT snapshot reads GENUINELY LIVE driver
# loads, which is the half of plan §2.2-1 that turned out to need verification
# rather than repair. Its per-commit refresh re-estimates parasitics, and
# GraphDelayCalc::loadCap recomputes a net's pin capacitances from the net's
# current pins on every call, so by the time the fifth buffer is snapshotted the
# headroom the first four consumed is already gone and its own frozen check
# declines the move. The engine therefore never overshoots here at all:
#
#   jacobi_snapshot    6 attempted, 2 reverted, 4 kept -> src/Q 60.63 (MET)
#   gauss_seidel_topo  4 attempted, 0 reverted, 4 kept -> src/Q 60.63 (MET)
#
# Same electrically-legal end state, reached by prevention rather than by
# correction. What the JIT path still cannot see is a LATER commit in the same
# sweep - no traversal order makes that possible - which is why the re-check runs
# for both engines and not only for Jacobi.
#
# Single-threaded (the GS engine is single-threaded by construction).
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
