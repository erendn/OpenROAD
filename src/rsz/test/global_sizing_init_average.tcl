# N-axis (initial solution) smoke: -init_mode average.
#
# One mode per file - see global_sizing_init_min_size.tcl for why. Same design
# and flow as the other three init smokes; only -init_mode differs.
#
# average is deterministic and seed-free: the LOWER median (index
# floor((n-1)/2)) of each swappable group ranked by cell leakage. It starts the
# loop from the middle of every group instead of an extreme, so it is the
# neutral point of the axis rather than a bound. The exact element choice, its
# even-n rule and its tie-breaks are pinned by the TestInitPass gtest; this file
# pins the flag, the pass and the run record.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup_dont_touch_sizeup.def
create_clock -period 0.35 clk
set_load 1.0 [all_outputs]

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

set_global_sizing_config -init_mode average
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
