# Fresh-netlist half A of the lambda_update formula-collapse probe: run
# tennakoon_ratio as the FIRST and ONLY global sizing on a pristine netlist.
# Its sibling `global_sizing_lambda_fresh_livramento` runs livramento_ratio the
# same way, so the two goldens are a matched A/B from an identical start.
#
# WHY A SEPARATE FILE PER RULE. `global_sizing_lambda_update` runs every rule
# in one session, so every leg after the first starts from the previous leg's
# netlist. On this design the first leg saturates it and legs 2..7 all report
# "0 cells replaced; 2/2 sweeps" - identical lines that say nothing about the
# formulas, which is the masking the S1-E review found. A pristine second leg
# needs an in-session netlist reset and there is none: a second `read_def`
# raises ODB-0251 (chip already has a block). One file per rule is the same
# answer the N-axis init modes and the cap-recheck engines already took.
#
# WHAT THE PAIR IS FOR - and read this before "fixing" a diff. Tennakoon's
# Fig. 13 rule and Livramento's Alg. 1 L13 rule are NOT the same formula: the
# step is lambda_ji/(a_i - D_ji) for Tennakoon and lambda_ji/a_i for Livramento,
# so they agree only on a critical arc (a_i = a_j + D) and diverge everywhere
# else (see the LambdaUpdate enum doc). TODAY THE TWO GOLDENS AGREE ON EVERY
# RESULT LINE - RSZ-0400, RSZ-0409, the summary table and the worst slack are
# identical, and only the RSZ-0417/0447 lines differ, because those name the
# rule. That agreement is the recorded finding, not an accident: this 3-gate
# smoke design does not resolve the difference - its discrete candidate set is
# too coarse for the multiplier field's shape to change any argmin
# (chen_subgradient lands on the same result lines here too, while sharma_cexp
# does not). So the pair pins the collapse rather than hiding it behind the
# saturated zeros of the cumulative file. If a change makes exactly one of them
# move, that is a formula-level change in that one rule; if both move together,
# it is the shared machinery.
#
# Both legs run under the iteration-2 lambda/mu auto-pairing (RSZ-0447): a
# paper lambda rule with no explicitly chosen mu policy takes endpoint_lambda,
# without which the flow projection annihilates the rule outright and the
# comparison would be vacuous by construction.
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
