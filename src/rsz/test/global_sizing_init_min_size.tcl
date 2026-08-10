# N-axis (initial solution) smoke: -init_mode min_size.
#
# ONE MODE PER FILE, BY NECESSITY. The N axis rewrites the netlist BEFORE the LR
# loop, so a mode's whole effect is a function of the netlist it starts from.
# Legs sharing one netlist would each measure their mode against the previous
# leg's mutated design instead of the pristine one, which is exactly the
# comparison this axis exists to make. OpenROAD Tcl has no in-session netlist
# reset (`ord::clear` leaves odb without a logger), so a fresh netlist per leg
# means a file per leg: global_sizing_init_{min_size,max_size,average,random}.
# The four read the same design, so their RSZ-0415/0400/0409 lines are directly
# comparable; only the -init_mode differs.
#
# The design is the one the don't-touch test uses (reg1, 14 instances, clock
# tightened to 0.35 ns so the sizer has real work to do). It was chosen over the
# 3-instance repair_setup2 the pre-rename presize smoke ran on because every
# mode moves a different number of cells here - on repair_setup2 the netlist is
# already at minimum leakage, so min_size is a no-op and the modes are hard to
# tell apart. Single-threaded for a deterministic golden.
#
# min_size is chen_partial's and livramento_partial's initial solution (Chen
# SOLVE_LRS/μ step 1, Livramento Alg. 1 L2). The tail of this file pins that the
# pre-rename spellings are GONE rather than aliased. min_size_fixviol - which
# this file used to pin as a loud rejection - now runs, and has a file of its
# own (global_sizing_init_min_size_fixviol) whose leg 1 reads this same design.
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

# The dbProperty layer was renamed too (gs_presize_mode -> gs_init_mode), and an
# absent key is not an error - so a block written by an older build would have
# run the as-given solution with nothing in the log saying the axis was dropped.
# RSZ-0441 is that diagnostic. This leg is about the WARNING, not about QoR, so
# it deliberately runs on the netlist leg 1 left behind.
puts "=== a stale gs_presize_mode property is reported, not silently ignored ==="
reset_global_sizing_config
odb::dbStringProperty_create [rsz::get_block] "gs_presize_mode" "min_size_max_vt"
repair_timing -setup -phases GLOBAL_SIZING
odb::dbProperty_destroy \
  [odb::dbStringProperty_find [rsz::get_block] "gs_presize_mode"]
