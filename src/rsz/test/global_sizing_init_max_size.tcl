# N-axis (initial solution) smoke: -init_mode max_size.
#
# One mode per file - see global_sizing_init_min_size.tcl for why (the axis
# rewrites the netlist before the LR loop, and OpenROAD Tcl has no in-session
# netlist reset, so shared-netlist legs would each start from the previous
# leg's design). Same design and flow as the other three init smokes; only
# -init_mode differs.
#
# max_size is the destructive bound of the axis: it upsizes every editable
# instance to the highest-leakage member of its group, so the LR loop has to
# recover leakage from a deliberately bad start. No preset selects it; it is a
# Stage-1 ablation cell (N2) and the survivorship-method anchor.
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

set_global_sizing_config -init_mode max_size
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
