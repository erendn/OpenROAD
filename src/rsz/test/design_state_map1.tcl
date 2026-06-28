# report_design_state_map -type placement_density on a placed design
source "helpers.tcl"
read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def gcd_nangate45_placed.def

report_design_state_map -type placement_density -bins_x 8 -bins_y 8
