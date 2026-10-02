# -restart_each_iteration: at the start of every iteration after the first,
# global sizing sets every gate back to the cell it had right after the init
# pass, as Chen et al. (ICCAD 1998, SOLVE_LRS/mu step 1) solve every
# subproblem from the lower size bound. chen_partial turns it on.
#
# Leg 1 records the netlist right after the init pass with a run that applies
# chen_partial's init pass (minimum size, registers left alone) and whose
# single sweep changes nothing: -timing_bias 1e-20 removes the timing term, and
# -output_drc_veto relative lets a gate that violates a limit at minimum size
# keep its cell, so every gate keeps its lowest-leakage cell. Leg 2 records
# the netlist after one chen_partial iteration. Leg 3 runs two iterations with
# the global_sizing_restore trace, which prints every cell the restart sets
# back. The restart at the start of iteration 2 must set back exactly the gates
# iteration 1 changed, each from its iteration-1 cell to its post-init cell,
# without running the init pass again.
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

# Instances whose cell differs between two recorded netlists.
proc diff_insts { refs_a refs_b } {
  set insts {}
  dict for {name ref} $refs_a {
    if { [dict get $refs_b $name] ne $ref } {
      lappend insts $name
    }
  }
  return $insts
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

# Runs global sizing with the global_sizing_restore trace on and prints its
# log without the trace lines. Returns the log.
proc run_with_restore_trace { } {
  set_debug_level RSZ global_sizing_restore 1
  with_output_to_variable log { repair_timing -setup -phases GLOBAL_SIZING }
  set_debug_level RSZ global_sizing_restore 0
  foreach line [split [string trimright $log "\n"] "\n"] {
    if { ![string match {\[DEBUG*} $line] } {
      puts $line
    }
  }
  return $log
}

# The cells the trace says were set back, as a dict of instance name to
# {from to}.
proc restored_cells { log } {
  set restored {}
  foreach line [split $log "\n"] {
    if { [regexp {restore (\S+) (\S+) -> (\S+)} $line -> name from to] } {
      dict set restored $name [list $from $to]
    }
  }
  return $restored
}

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def gcd_nangate45_placed.def
read_sdc gcd_nangate45.sdc
create_clock [get_ports clk] -name core_clock -period 0.4

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

set input_refs [cell_refs]

puts "=== leg 1: the netlist right after the init pass ==="
set_global_sizing_config -preset rsz_baseline -init_mode min_size \
  -size_registers 0 -max_iterations 1 -timing_bias 1e-20 \
  -output_drc_veto relative
set log [run_with_restore_trace]
if { ![regexp {RSZ-0400.* 0 replacements attempted} $log] } {
  error "the sweep changed cells, so the result is not the post-init netlist"
}
set init_refs [cell_refs]
restore_input $input_refs

puts "=== leg 2: one chen_partial iteration ==="
reset_global_sizing_config
set_global_sizing_config -preset chen_partial -max_iterations 1
set log [run_with_restore_trace]
if { [dict size [restored_cells $log]] != 0 } {
  error "the first iteration set cells back"
}
set iter1_refs [cell_refs]
set changed [diff_insts $iter1_refs $init_refs]
if { [llength $changed] == 0 } {
  error "iteration 1 changed no gate, so the restart has nothing to undo"
}
puts "Iteration 1 changed [llength $changed] gates."
restore_input $input_refs

puts "=== leg 3: two chen_partial iterations ==="
set_global_sizing_config -max_iterations 2
report_global_sizing_config
set log [run_with_restore_trace]
if { [regexp -all {RSZ-0416} $log] != 1 } {
  error "the init pass ran more than once"
}
set restored [restored_cells $log]
if { [dict size $restored] != [llength $changed] } {
  error "expected [llength $changed] cells set back; got [dict size $restored]"
}
foreach name $changed {
  if { ![dict exists $restored $name] } {
    error "$name was changed by iteration 1 but not set back"
  }
  lassign [dict get $restored $name] from to
  if { $from ne [dict get $iter1_refs $name]
       || $to ne [dict get $init_refs $name] } {
    error "$name set back from $from to $to; expected from\
      [dict get $iter1_refs $name] to [dict get $init_refs $name]"
  }
}
puts "Iteration 2 set the [llength $changed] gates iteration 1 changed back to\
  their post-init cells."
restore_input $input_refs

# Leg 4: with the restart off, iteration 2 starts from iteration 1's cells.
puts "=== leg 4: chen_partial without the restart ==="
set_global_sizing_config -restart_each_iteration 0
report_global_sizing_config
set log [run_with_restore_trace]
if { [dict size [restored_cells $log]] != 0 } {
  error "cells were set back with the restart off"
}
restore_input $input_refs

# Leg 5: the estimation loop undoes each dry run itself and never restarts.
# One iteration, so only the estimation loop could set cells back.
puts "=== leg 5: the estimation loop does not restart ==="
reset_global_sizing_config
set_global_sizing_config -preset chen_partial -max_iterations 1 \
  -lambda_seed estimation_loop
set log [run_with_restore_trace]
if { [dict size [restored_cells $log]] != 0 } {
  error "the estimation loop set cells back"
}
restore_input $input_refs

# Leg 6: with one sweep per iteration the restart is warned about (RSZ-0457).
puts "=== leg 6: the restart with one sweep per iteration ==="
reset_global_sizing_config
set_global_sizing_config -preset chen_partial -max_iterations 1 \
  -max_inner_sweeps 1
set log [run_with_restore_trace]
if { ![regexp {RSZ-0457} $log] } {
  error "no RSZ-0457 for the restart with max_inner_sweeps=1"
}
restore_input $input_refs

# Leg 7: the WNS that steers the run follows the solutions, not the restart.
# Under -timing_scale livramento_alpha, the level-2 trace prints the WNS that
# reschedules alpha at each iteration, and the level-1 trace each iteration's
# result. At iteration k the reschedule must read iteration k-1's result, not
# the WNS of the initial cells the restart has just set back.
puts "=== leg 7: livramento_alpha reads the previous iteration's WNS ==="
reset_global_sizing_config
set_global_sizing_config -preset chen_partial -max_iterations 3 \
  -timing_scale livramento_alpha
set_debug_level RSZ global_sizing 2
with_output_to_variable log { repair_timing -setup -phases GLOBAL_SIZING }
set_debug_level RSZ global_sizing 0
set alpha_wns {}
set result_wns {}
foreach line [split $log "\n"] {
  if { [regexp {LR livramento alpha: iter=(\d+) WNS=(\S+)} $line -> k wns] } {
    dict set alpha_wns $k $wns
  } elseif { [regexp {iter=(\d+) wns=(\S+)} $line -> k wns] } {
    # In seconds; the alpha trace prints nanoseconds to 3 decimals.
    dict set result_wns $k [format %.3f [expr { $wns * 1e9 }]]
  }
}
foreach k {1 2} {
  set expected [dict get $result_wns $k]
  set got [dict get $alpha_wns $k]
  if { $got != $expected } {
    error "iteration [expr { $k + 1 }] rescheduled alpha from WNS $got;\
      iteration $k ended at $expected"
  }
  puts "Iteration [expr { $k + 1 }] rescheduled alpha from WNS $got, the\
    result of iteration $k."
}

puts "=== -restart_each_iteration takes a boolean ==="
if { ![catch { set_global_sizing_config -restart_each_iteration maybe }] } {
  error "-restart_each_iteration maybe was accepted"
}
puts "-restart_each_iteration maybe rejected"

puts "=== reset ==="
set_global_sizing_config -restart_each_iteration 1
reset_global_sizing_config -restart_each_iteration
if { [odb::dbBoolProperty_find [rsz::get_block] "gs_restart_each_iteration"]
     ne "NULL" } {
  error "reset_global_sizing_config -restart_each_iteration left\
    gs_restart_each_iteration set"
}
puts "gs_restart_each_iteration cleared"
