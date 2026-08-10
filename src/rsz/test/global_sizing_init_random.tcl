# N-axis (initial solution) smoke: -init_mode random with a PINNED -init_seed.
#
# One mode per file - see global_sizing_init_min_size.tcl for why. Same design
# and flow as the other three init smokes; only -init_mode/-init_seed differ.
#
# random draws each editable instance's starting cell uniformly from its whole
# swappable group (the as-given cell included), keyed by a hash of (init_seed,
# instance path name). That keying is what makes the mode an experiment rather
# than noise: the same netlist and seed reproduce the same initial design
# whatever order the instances are visited in and whatever the thread count is,
# and init_seed is its own flag - never derived from, and never correlated with,
# the global-placement seed, because netlist initialization and placement
# perturbation are two SEPARATE variation sources for the variability study.
#
# Determinism, seed-to-seed variation and visit-order independence are pinned as
# properties by the TestInitPass gtest (which can vary all three cheaply); what
# this file adds is that the flag reaches the pass on a real netlist and that
# the seed is recorded in the RSZ-0417 run record, which is how a collected run
# is attributed to its cell.
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
