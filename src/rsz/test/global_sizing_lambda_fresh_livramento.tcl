# Fresh-netlist half B of the lambda_update formula-collapse probe: run
# livramento_ratio as the FIRST and ONLY global sizing on a pristine netlist.
# Its sibling `global_sizing_lambda_fresh_tennakoon` runs tennakoon_ratio the
# same way, so the two goldens are a matched A/B from an identical start. That
# file's header carries the full rationale for the pair (why one file per rule,
# what a diff between the two means, and why they agree today); this one exists
# to be the other half of it.
#
# It also closes the lambda_update coverage gap the S1-E review named:
# livramento_ratio was the one enum value `global_sizing_lambda_update` never
# exercised. It now runs there too - and here, where the result is legible.
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
