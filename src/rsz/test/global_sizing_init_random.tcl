# Smoke test for -init_mode random with a fixed -init_seed. See
# global_sizing_init_min_size.tcl for why each mode has its own file.
#
# Each editable instance's starting cell is drawn from its swappable group using
# a hash of (init_seed, instance name), so the result does not depend on visit
# order or thread count, and init_seed is independent of the placement seed.
# Those properties are unit-tested by TestInitPass; this test checks that the
# option reaches the pass and that RSZ-0417 reports the seed.
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

set_global_sizing_config -init_mode random -init_seed 3
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
