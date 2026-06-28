# power density map (uses STA power analysis on the current scene)
source "helpers.tcl"
read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def gcd_nangate45_placed.def
read_sdc gcd_nangate45.sdc

report_design_state_map -type power_density -bins_x 6 -bins_y 6
