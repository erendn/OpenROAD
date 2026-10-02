# -lambda_update livramento_ratio as the only global sizing run on a fresh
# netlist. See global_sizing_lambda_fresh_tennakoon.tcl, which does the same
# for tennakoon_ratio, for how to read the two goldens together.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup2.def
read_sdc repair_setup2.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

set_global_sizing_config -lambda_update livramento_ratio
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
