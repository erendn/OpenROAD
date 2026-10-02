# Smoke test for -move_set, which limits the members of a gate's swappable
# group the per-gate subproblem may consider. Each run must complete and print
# the RSZ-0417/0400/0409 records; the selection rules (Sharma's Fig. 9
# descent, Mangiras' +-1 size step) are unit-tested by TestSizeVthGrid.
#
# The runs share one netlist, so only the first sees a fresh design.
# mangiras_size_step goes first because its effect is visible here: it moves a
# gate one size per sweep, so it needs 4 replacements to reach the end state
# that full_library reaches in 3 (first run of global_sizing_lambda_update.tcl).
# Sharma switches to Fast-OLR at iteration 5, but this design converges in 2-4
# sweeps, so the third run forces -fast_olr_start_iter 1 and the fourth
# restores the default to check the switch-over itself. Single-threaded for a
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
