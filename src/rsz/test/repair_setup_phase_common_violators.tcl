# repair_timing -setup with COMMON_VIOLATORS phase before LEGACY
set repair_args [list -phases "COMMON_VIOLATORS LEGACY" -skip_last_gasp -skip_crit_vt_swap]
source "repair_setup1.tcl"
