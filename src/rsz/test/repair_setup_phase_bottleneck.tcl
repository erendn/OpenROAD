# repair_timing -setup with BOTTLENECK phase before LEGACY
set repair_args [list -phases "BOTTLENECK LEGACY" -skip_last_gasp -skip_crit_vt_swap]
source "repair_setup1.tcl"
