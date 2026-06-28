# pin density and estimated congestion (RUDY) maps; no global routing needed
source "helpers.tcl"
read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def gcd_nangate45_placed.def

report_design_state_map -type pin_density -bins_x 6 -bins_y 6
report_design_state_map -type estimated_congestion
