# Coverage for the GlobalSizingConfig preset plumbing. Applies the rsz_baseline
# preset via set_global_sizing_config -preset, echoes it with
# report_global_sizing_config, and runs the GLOBAL_SIZING phase. rsz_baseline
# is the current engine, so the resized netlist must match the plain
# global_sizing.tcl result (same design, same golden metrics).
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup2.def
read_sdc repair_setup2.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_global_sizing_config -preset rsz_baseline
report_global_sizing_config

repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
