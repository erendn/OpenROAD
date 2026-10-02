# -power_objective total prices each candidate cell by its leakage, plus its
# internal power, plus the switching power its input pins add to their fanin
# nets. On gcd (nangate45) with a 0.3 ns clock that changes the result of the
# default leakage objective. (With looser clocks both objectives happen to end
# at the same netlist: the timing weight is normalized by the median power, and
# on this design dynamic power mostly scales with leakage.)
#
# The sizer reads OpenSTA's switching activities once, before the first cell
# swap. With the global_sizing_activity debug group on, it prints each cached
# activity in the format of the `activity` pin property. After the run every
# printed pin must still have that activity in OpenSTA: the sizer swaps a cell
# only for an equivalent one, so the logic functions, and with them the
# activities, do not change.
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

puts "=== leakage objective ==="
set_global_sizing_config -preset rsz_baseline
repair_timing -setup -phases GLOBAL_SIZING
set leakage_refs [cell_refs]

puts "=== total objective ==="
restore_input $input_refs
set_global_sizing_config -preset rsz_baseline -power_objective total
report_global_sizing_config
repair_timing -setup -phases GLOBAL_SIZING
set diffs [count_diffs $leakage_refs [cell_refs]]
if { $diffs == 0 } {
  error "-power_objective total gave the leakage objective's netlist"
}
puts "$diffs instance(s) differ from the leakage objective's result."

puts "=== the cached activities match OpenSTA after the run ==="
restore_input $input_refs
set_debug_level RSZ global_sizing_activity 1
with_output_to_variable log { repair_timing -setup -phases GLOBAL_SIZING }
set_debug_level RSZ global_sizing_activity 0
set checked 0
foreach line [split $log "\n"] {
  if { ![regexp {^\[DEBUG RSZ-global_sizing_activity\] activity (\S+) (\S+) (\S+)$} \
          $line -> pin_name density duty] } {
    continue
  }
  lassign [get_property [sta::find_pin $pin_name] activity] sta_density sta_duty
  if { $density ne $sta_density || $duty ne $sta_duty } {
    error "$pin_name: cached activity $density $duty,\
      OpenSTA $sta_density $sta_duty"
  }
  incr checked
}
if { $checked == 0 } {
  error "no cached activity was printed"
}
puts "$checked pin activities match OpenSTA."

puts "=== an unknown objective is rejected ==="
if { ![catch { set_global_sizing_config -power_objective dynamic }] } {
  error "-power_objective dynamic was accepted"
}

puts "=== reset clears -power_objective ==="
reset_global_sizing_config -power_objective
if { [odb::dbStringProperty_find [rsz::get_block] "gs_power_objective"] eq "NULL" } {
  puts "gs_power_objective cleared"
} else {
  error "reset_global_sizing_config -power_objective left gs_power_objective set"
}
