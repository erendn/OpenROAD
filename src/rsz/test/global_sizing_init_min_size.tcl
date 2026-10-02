# Smoke test for -init_mode min_size, the initial cell assignment used by
# chen_partial and livramento_partial.
#
# Each init mode has its own file (global_sizing_init_*.tcl) because the mode
# rewrites the netlist before the LR loop and there is no in-session netlist
# reset (ord::clear leaves odb without a logger). They all read the same design,
# so their RSZ-0415/0400/0409 lines can be compared. repair_setup2 is not used
# because it is already at minimum leakage, where min_size does nothing.
# Single-threaded for a deterministic golden.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup_dont_touch_sizeup.def
create_clock -period 0.35 clk
set_load 1.0 [all_outputs]

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

set_global_sizing_config -init_mode min_size
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# The old -presize_mode option and the old -init_mode values must be rejected,
# not accepted as aliases.
puts "=== the pre-rename spellings are gone, not aliased ==="
foreach { opt value } {
  -presize_mode max_size_min_vt
  -init_mode min_size_max_vt
  -init_mode max_size_min_vt
  -init_mode disabled
} {
  if { [catch { set_global_sizing_config $opt $value }] } {
    puts "set_global_sizing_config $opt $value rejected"
  } else {
    puts "ERROR: set_global_sizing_config $opt $value still accepted"
  }
}

# A block written by an older build may carry a gs_presize_mode property, which
# global sizing does not read. RSZ-0441 warns about it instead of ignoring it.
puts "=== a stale gs_presize_mode property is reported, not silently ignored ==="
reset_global_sizing_config
odb::dbStringProperty_create [rsz::get_block] "gs_presize_mode" "min_size_max_vt"
repair_timing -setup -phases GLOBAL_SIZING
odb::dbProperty_destroy \
  [odb::dbStringProperty_find [rsz::get_block] "gs_presize_mode"]
