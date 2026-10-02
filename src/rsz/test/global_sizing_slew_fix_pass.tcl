# -slew_fix_pass upsizes, once before the first LR iteration, each gate whose
# output slew is above its limit, visiting the gates from inputs to outputs
# (Sharma et al., ICCAD 2015, Sec. III-A). sharma_seq_partial turns it on,
# after its minimum-size start and a max-capacitance-only repair
# (-init_mode min_size_fixcap).
#
# On gcd (nangate45) with a 0.05 ns max_transition, no gate is over its max
# capacitance after the minimum-size reset (RSZ-0445), and the slew pass
# upsizes the gates over the slew limit (RSZ-0461). The preset's sweeps reject
# any cell that violates a limit, so a gate the pass repairs stays repaired, and
# the run ends with no max slew violation. With -slew_fix_pass 0 the gates over
# the limit are left to the sweeps, which do not fix them all; that run also
# sets the preset's -init_mode min_size_fixcap from Tcl, so the report shows
# it. Each run starts from the input netlist. Single-threaded for a
# deterministic golden.
source "helpers.tcl"

proc cell_refs { } {
  set refs {}
  foreach inst [get_cells *] {
    dict set refs [get_full_name $inst] [get_property $inst ref_name]
  }
  return $refs
}

proc restore_input { input_refs } {
  dict for {name ref} $input_refs {
    set inst [get_cells $name]
    if { [get_property $inst ref_name] ne $ref } {
      replace_cell $inst NangateOpenCellLibrary/$ref
    }
  }
  estimate_parasitics -placement
}

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def gcd_nangate45_placed.def
read_sdc gcd_nangate45.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1
set_max_transition 0.05 [current_design]

set input_refs [cell_refs]

puts "=== sharma_seq_partial (slew_fix_pass=true) ==="
set_global_sizing_config -preset sharma_seq_partial
set_global_sizing_config -max_iterations 5
repair_timing -setup -phases GLOBAL_SIZING
set slew_violations [sta::max_slew_violation_count]
puts "max slew violations: $slew_violations"
if { $slew_violations != 0 } {
  error "sharma_seq_partial left $slew_violations max slew violations"
}

puts "=== sharma_seq_partial -slew_fix_pass 0 ==="
restore_input $input_refs
set_global_sizing_config -slew_fix_pass 0 -init_mode min_size_fixcap
report_global_sizing_config
repair_timing -setup -phases GLOBAL_SIZING
set slew_violations [sta::max_slew_violation_count]
puts "max slew violations: $slew_violations"
if { $slew_violations == 0 } {
  error "without the slew fix pass no max slew violation is left"
}

puts "=== a non-Boolean value is rejected ==="
if { ![catch { set_global_sizing_config -slew_fix_pass maybe }] } {
  error "-slew_fix_pass maybe was accepted"
}

puts "=== reset clears -slew_fix_pass ==="
reset_global_sizing_config -slew_fix_pass
if { [odb::dbBoolProperty_find [rsz::get_block] "gs_slew_fix_pass"] eq "NULL" } {
  puts "gs_slew_fix_pass cleared"
} else {
  error "reset_global_sizing_config -slew_fix_pass left gs_slew_fix_pass set"
}
