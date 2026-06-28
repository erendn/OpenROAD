# routing congestion map (requires global routing to populate the GCell grid)
source "helpers.tcl"
read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def gcd_nangate45_placed.def
read_sdc gcd_nangate45.sdc

set_routing_layers -signal metal2-metal10
global_route

report_design_state_map -type routing_congestion
