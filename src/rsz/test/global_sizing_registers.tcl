# -size_registers 0 keeps global sizing off every register. The design is the
# one used by repair_setup_dont_touch_sizeup.tcl, which has DFF_X1 and DFF_X2
# flip-flops. -init_mode min_size would shrink every DFF_X2, so it shows that
# the init pass skips registers. The sweeps that follow run on the same gates.
# A second run with the default (registers sized) shows that the first one is
# not vacuous. Single-threaded for a deterministic golden.
source "helpers.tcl"

proc register_refs { } {
  set refs {}
  foreach inst [all_registers -cells] {
    dict set refs [get_full_name $inst] [get_property $inst ref_name]
  }
  return $refs
}

# Registers whose ref_name differs from the recorded one.
proc changed_registers { before_refs } {
  set changed {}
  dict for {inst_name before_ref} $before_refs {
    set after_ref [get_property [get_cells $inst_name] ref_name]
    if { $before_ref ne $after_ref } {
      lappend changed "$inst_name:$before_ref->$after_ref"
    }
  }
  return $changed
}

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup_dont_touch_sizeup.def
create_clock -period 0.35 clk
set_load 1.0 [all_outputs]

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

set before_refs [register_refs]
puts "Registers: [dict size $before_refs]"

puts "=== -size_registers 0: no register changes cell ==="
set_global_sizing_config -preset rsz_baseline -size_registers 0 \
  -init_mode min_size
report_global_sizing_config
repair_timing -setup -phases GLOBAL_SIZING
set changed [changed_registers $before_refs]
if { [llength $changed] != 0 } {
  error "registers resized with -size_registers 0: [join $changed {, }]"
}
puts "All [dict size $before_refs] registers kept their cells."

puts "=== reset clears -size_registers ==="
reset_global_sizing_config -size_registers
if { [odb::dbBoolProperty_find [rsz::get_block] "gs_size_registers"] eq "NULL" } {
  puts "gs_size_registers cleared"
} else {
  error "reset_global_sizing_config -size_registers left gs_size_registers set"
}

puts "=== default: registers are sized ==="
reset_global_sizing_config
set_global_sizing_config -init_mode min_size
repair_timing -setup -phases GLOBAL_SIZING
set changed [changed_registers $before_refs]
if { [llength $changed] == 0 } {
  error "no register changed with registers sized; the check above is vacuous"
}
puts "[llength $changed] register(s) changed cell: [join $changed {, }]"
