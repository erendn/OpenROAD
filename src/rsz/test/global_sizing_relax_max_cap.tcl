# -relax_max_cap prices max capacitance in the global sizing cost instead of
# forbidding it (Livramento et al., DATE 2013, Eq. 3). Each output pin of a
# sized gate, and of a register even when registers are not sized, gets a
# multiplier beta, and after every iteration beta is multiplied by the pin's
# load over its cell's limit (Alg. 1 line 15).
# livramento_partial turns it on.
#
# On gcd (nangate45), set_load puts 3000 fF on output port resp_msg[1], driven
# by _847_. No buffer drives 3000 fF within its limit (BUF_X32 allows
# 1904.3 fF), so _847_/Z is over its limit from the first sweep to the last,
# and RSZ-0464 counts it at the start and at the end. Every other pin stays
# within its limit. The global_sizing_cap_multipliers trace prints each update
# of a pin over its limit, so it shows only _847_/Z, whose beta grows at every
# update: the fix pass climbs one swappable group per sweep (CLKBUF_X3, BUF_X4,
# BUF_X8, BUF_X32; see global_sizing_cap_fix_pass), and the load stays above
# every limit on the way. At the end report_check_types finds the same single
# violation. Without the setting, RSZ-0464 and the trace are absent. Under
# rsz_baseline the setting runs too, with a warning about its upsize
# hysteresis (RSZ-0465).
# Single-threaded for a deterministic golden.
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

proc expect_cell { inst_name expected } {
  set ref [get_property [get_cells $inst_name] ref_name]
  puts "$inst_name: $ref"
  if { $ref ne $expected } {
    error "$inst_name is $ref, expected $expected"
  }
}

# Runs global sizing with the multiplier trace on and prints its log. Returns the
# beta updates of pins over their limit, as {pin before after} triples, and
# RSZ-0464's counts at the start and at the end ({} without the line).
proc run_with_multiplier_trace { } {
  set_debug_level RSZ global_sizing_cap_multipliers 1
  with_output_to_variable log { repair_timing -setup -phases GLOBAL_SIZING }
  set_debug_level RSZ global_sizing_cap_multipliers 0
  set updates {}
  set counts {}
  foreach line [split [string trimright $log "\n"] "\n"] {
    puts $line
    if { [regexp {multiplier (\S+) load=\S+ limit=\S+: (\S+) -> (\S+)} \
            $line -> pin before after] } {
      lappend updates [list $pin $before $after]
    }
    regexp {RSZ-0464\].* (\d+) over their max capacitance at the start, (\d+) at} \
      $line -> start_count end_count
  }
  if { [info exists start_count] } {
    set counts [list $start_count $end_count]
  }
  return [list $updates $counts]
}

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def gcd_nangate45_placed.def
read_sdc gcd_nangate45.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1
set_load 3000 [get_ports {resp_msg[1]}]

set input_refs [cell_refs]

puts "=== livramento_partial (relax_max_cap=true) ==="
set_global_sizing_config -preset livramento_partial -max_iterations 6
lassign [run_with_multiplier_trace] updates counts
expect_cell _847_ BUF_X32

# Six iterations update the multipliers five times, after each of the first
# five sweeps; every update is of _847_/Z, and each one raises its beta.
if { [llength $updates] != 5 } {
  error "expected 5 updates of a pin over its limit, got [llength $updates]"
}
set prev ""
foreach update $updates {
  lassign $update pin before after
  if { $pin ne "_847_/Z" } {
    error "$pin is over its limit; only _847_/Z should be"
  }
  if { !($after > $before) } {
    error "beta of _847_/Z went from $before to $after; it should grow"
  }
  if { $prev ne "" && $before != $prev } {
    error "beta of _847_/Z started an update at $before, not at $prev"
  }
  set prev $after
}
puts "beta of _847_/Z grew at all [llength $updates] updates."

# RSZ-0464's end count is the violation report_check_types finds.
report_check_types -max_capacitance -violators
set violations [sta::max_capacitance_violation_count]
if { $counts ne [list 1 $violations] || $violations != 1 } {
  error "RSZ-0464 counted {$counts}; report_check_types found $violations"
}
puts "RSZ-0464 and report_check_types agree: 1 max capacitance violation."

puts "=== livramento_partial -relax_max_cap 0 ==="
restore_input $input_refs
set_global_sizing_config -relax_max_cap 0
report_global_sizing_config
lassign [run_with_multiplier_trace] updates counts
if { [llength $updates] != 0 || $counts ne {} } {
  error "the multipliers ran with -relax_max_cap 0"
}
puts "No multipliers without the setting."

# A pin below its limit earns a credit, so a gate's cost can be negative, and
# rsz_baseline's relative upsize hysteresis then holds nothing back. RSZ-0465
# warns about the combination.
puts "=== rsz_baseline -relax_max_cap 1 ==="
restore_input $input_refs
reset_global_sizing_config
set_global_sizing_config -preset rsz_baseline -relax_max_cap 1 \
  -max_iterations 1
lassign [run_with_multiplier_trace] updates counts
if { $counts eq {} } {
  error "the multipliers did not run under rsz_baseline"
}

puts "=== a non-Boolean value is rejected ==="
if { ![catch { set_global_sizing_config -relax_max_cap maybe }] } {
  error "-relax_max_cap maybe was accepted"
}

puts "=== reset clears -relax_max_cap ==="
reset_global_sizing_config -relax_max_cap
if { [odb::dbBoolProperty_find [rsz::get_block] "gs_relax_max_cap"] eq "NULL" } {
  puts "gs_relax_max_cap cleared"
} else {
  error "reset_global_sizing_config -relax_max_cap left gs_relax_max_cap set"
}
