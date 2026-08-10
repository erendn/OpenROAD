# it2 smoke test: exercise every move_set option of GlobalSizingPolicy - the F4
# axis, i.e. WHICH members of a gate's swappable group the per-gate subproblem
# is allowed to enumerate. For each option the GLOBAL_SIZING phase runs and the
# worst slack is reported. This asserts that each option's full path (the
# (width, Vth) grid decomposition on the main thread, the restricted enumeration
# in the worker) runs without error and produces the RSZ-0417/0400/0409 run
# record; the selection arithmetic - Sharma's Fig. 9 descent and Mangiras' +-1
# band - is checked against hand-built grids by the TestSizeVthGrid gtest.
# Single-threaded for a deterministic golden.
#
# Three things to know when reading the golden:
#
#  1. THE LEGS ARE CUMULATIVE, as in every other axis smoke here: leg N starts
#     from leg N-1's netlist, so only the FIRST leg sees a pristine design and
#     the later ones are mostly no-op. What a later leg pins is that its option
#     DISPATCHED (the RSZ-0417 `move_set=` field), not a QoR comparison.
#  2. THE PRISTINE SLOT GOES TO mangiras_size_step, not to the default option,
#     because it is the one whose mechanism is legible on a design this small.
#     Restricted to +-1 width rank the sweep cannot jump straight to a gate's
#     argmin, so it WALKS there one step per sweep: 4 replacements to reach the
#     same end state (WNS -0.157, leakage 2.07e-07 W) that full_library reaches
#     in 3 - and the identical pristine full_library run is on record as leg 1
#     of `global_sizing_lambda_update` (same design, same SDC, same defaults),
#     so the two numbers are directly comparable. More moves, same destination,
#     is exactly what a +-1-step restriction should look like.
#  3. `-fast_olr_start_iter 1` on the third leg is deliberate and is NOT the
#     paper's value. Sharma switches over at the 5th LDP iteration and the
#     struct default says so; this design converges in 2-4 sweeps, so at the
#     paper's 5 the Fast-OLR path would never execute and the leg would silently
#     test nothing. The fourth leg restores the default to pin that the
#     switch-over gate itself works - before it, the sweep runs the exhaustive
#     scan, which is Sharma's own first four iterations.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup2.def
read_sdc repair_setup2.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

puts "=== move_set mangiras_size_step (pristine) ==="
set_global_sizing_config -move_set mangiras_size_step
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

puts "=== move_set full_library ==="
set_global_sizing_config -move_set full_library
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

puts "=== move_set sharma_fast_olr (switch-over forced to iteration 1) ==="
set_global_sizing_config -move_set sharma_fast_olr -fast_olr_start_iter 1
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

puts "=== move_set sharma_fast_olr (paper switch-over, never reached here) ==="
reset_global_sizing_config -fast_olr_start_iter
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

puts "=== fast_olr_start_iter is inert off sharma_fast_olr (RSZ-0449) ==="
set_global_sizing_config -move_set full_library -fast_olr_start_iter 20
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

puts "=== report_global_sizing_config ==="
report_global_sizing_config
