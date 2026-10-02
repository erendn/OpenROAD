# -lambda_update tennakoon_ratio as the only global sizing run on a fresh
# netlist; global_sizing_lambda_fresh_livramento.tcl does the same for
# livramento_ratio. global_sizing_lambda_update.tcl cannot compare the rules
# because its later runs start from an already optimized netlist, and a second
# read_def fails (ODB-0251).
#
# The steps differ: lambda_ji/(a_i - D_ji) for Tennakoon and Sechen (ICCAD 2002,
# Fig. 13), lambda_ji/a_i for Livramento et al. (DATE 2013, Alg. 1 line 13).
# On this 3-gate design the difference changes no cell choice, so the two
# goldens differ only in RSZ-0417/0447, which name the rule. A change that
# moves only one golden changed that rule.
#
# Without -mu_policy, a paper lambda rule gets endpoint_lambda (RSZ-0447);
# under the other policies the flow projection discards the rule's effect.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup2.def
read_sdc repair_setup2.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

set_global_sizing_config -lambda_update tennakoon_ratio
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
