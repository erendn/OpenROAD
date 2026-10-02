# Smoke test for -init_mode average: each instance starts from the lower median
# (index floor((n-1)/2)) of its swappable group ranked by leakage. See
# global_sizing_init_min_size.tcl for why each mode has its own file. The
# selection details are unit-tested by TestInitPass.
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
