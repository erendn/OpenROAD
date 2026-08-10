# The post-sweep max-cap re-check (iteration-2 plan §2.2-1), Jacobi leg.
#
# WHAT THIS PINS. The sweep's electrical veto tests each candidate against the
# gate's FROZEN snapshot load, so under the Jacobi engine - where every gate is
# snapshotted before any commit - no candidate can see the load its neighbours
# are about to add to the net it drives. The re-check in CapRecheck.hh corrects
# that after the commits. This file is the acceptance evidence that it fires and
# what it costs.
#
# THE DESIGN (global_sizing_cap_recheck.def) is built for exactly one mechanism:
# a dont-touch driver `src` feeding six symmetric buffers b0..b5, each behind a
# flop of its own. Symmetric means all six want the same upsize in the same
# sweep. The loads pin the arithmetic instead of leaving it to the wire model:
#   * net n (src -> b0..b5) is loaded to 53 fF, so src/Q sits at 58.85 fF
#     against DFF_X1's own 60.73 fF Liberty cap limit - 1.88 fF of headroom;
#   * each m<i> (b<i> -> e<i>) is loaded to 40 fF, which is what makes upsizing
#     the buffers worth anything at all (they are the only movable gates on the
#     path, and src cannot be upsized to rescue the net).
# One buffer upsize adds ~0.45 fF to n. Every candidate therefore CLEARS the
# frozen check on its own (58.85 + 0.45 << 60.73) and all six clear it in the
# same sweep - and together they add ~2.68 fF, which does not fit. That gap is
# the blind spot, reproduced.
#
# WHAT TO READ IN THE GOLDEN:
#   RSZ-0400  6 replacements attempted, 4 kept - the re-check gave back 2;
#   RSZ-0443  the revert count, always printed (0 is a measurement, not a
#             missing line);
#   the final max-capacitance check: 60.63 vs 60.73, MET. Without the re-check
#             the same sweep lands at ~61.5 fF, a max-cap ERC violation - the
#             loss channel S1 §3.3 traced.
# The re-check gives back the FEWEST moves that clear the net (2 of the 6), not
# all of them: minimality is what stops the next sweep from re-proposing the
# same set forever. Four timing moves survive.
#
# ONE ENGINE PER FILE, as for the N axis: OpenROAD Tcl has no in-session netlist
# reset, so the Gauss-Seidel comparison runs on a pristine copy of this design in
# global_sizing_cap_recheck_gs.tcl. The two goldens are directly comparable.
#
# Single-threaded for a deterministic golden. The re-check is main-thread work
# that walks the movers in commit order, so it is thread-count invariant by
# construction (global_sizing_threads covers that axis).
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def global_sizing_cap_recheck.def
create_clock -period 0.20 clk

# The driver cannot rescue its own net by upsizing (which would raise its cap
# limit); only the sinks can overload it. That is what isolates the mechanism.
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
