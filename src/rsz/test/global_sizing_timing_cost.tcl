# -timing_cost per_arc prices each timing arc into a gate's output pin at its
# own delay, where the default worst_arc prices every arc into the pin at the
# slowest arc's delay. On gcd (nangate45) with a 0.3 ns clock, rsz_baseline
# with per_arc pricing ends at a different netlist from rsz_baseline. The run
# echoes timing_cost=per_arc in RSZ-0417, and report_global_sizing_config
# reports the setting.
#
# Each run starts from the input netlist. Single-threaded for a deterministic
# golden.
source "helpers.tcl"

proc cell_refs { } {
  set refs {}
  foreach inst [get_cells *] {
    dict set refs [get_full_name $inst] [get_property $inst ref_name]
  }
  return $refs
}

# Number of instances whose cell differs between two recorded netlists.
proc count_diffs { refs_a refs_b } {
  set count 0
  dict for {name ref} $refs_a {
    if { [dict get $refs_b $name] ne $ref } {
      incr count
    }
  }
  return $count
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
create_clock [get_ports clk] -name core_clock -period 0.3

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

set input_refs [cell_refs]

puts "=== worst_arc ==="
set_global_sizing_config -preset rsz_baseline
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
set worst_arc_refs [cell_refs]

puts "=== per_arc ==="
restore_input $input_refs
set_global_sizing_config -preset rsz_baseline -timing_cost per_arc
report_global_sizing_config
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
set diffs [count_diffs $worst_arc_refs [cell_refs]]
if { $diffs == 0 } {
  error "-timing_cost per_arc gave the worst_arc netlist"
}
puts "$diffs instance(s) differ from the worst_arc result."

puts "=== an unknown pricing is rejected ==="
if { ![catch { set_global_sizing_config -timing_cost per_pin }] } {
  error "-timing_cost per_pin was accepted"
}

puts "=== reset clears -timing_cost ==="
reset_global_sizing_config -timing_cost
if { [odb::dbStringProperty_find [rsz::get_block] "gs_timing_cost"] eq "NULL" } {
  puts "gs_timing_cost cleared"
} else {
  error "reset_global_sizing_config -timing_cost left gs_timing_cost set"
}
