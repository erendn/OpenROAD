# Post-sweep max-capacitance re-check (CapRecheck.hh) with the Jacobi engine.
# Jacobi checks every candidate against loads snapshotted before any move is
# committed, so a candidate cannot see what other moves in the same sweep add
# to the net it drives. The re-check runs after the commits and reverts moves
# until each net is back under its limit.
#
# global_sizing_cap_recheck.def: driver src drives six identical buffers
# b0..b5, each driving a 40 fF load. src/Q has 1.88 fF of max-cap headroom and
# each buffer upsize adds about 0.45 fF, so every move passes the check alone
# but all six together do not fit. Expected: 6 replacements attempted, 2
# reverted (RSZ-0443, printed even when 0), src/Q met. The re-check reverts as
# few moves as possible; reverting all six would let the next sweep propose
# them again.
#
# global_sizing_cap_recheck_gs.tcl runs the same design with the Gauss-Seidel
# engine. Single-threaded for a deterministic golden.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def global_sizing_cap_recheck.def
create_clock -period 0.20 clk

# Keep src from being upsized (which would raise its cap limit), so only the
# buffer upsizes change the headroom on n.
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

repair_timing -setup -phases GLOBAL_SIZING

puts "=== after: the kept moves fit under the limit ==="
report_check_types -max_capacitance
report_worst_slack -max -digits 3
