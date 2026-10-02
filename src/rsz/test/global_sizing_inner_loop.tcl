# -max_inner_sweeps: after each multiplier update, global sizing repeats the
# sweep with the multipliers fixed until a sweep keeps no move or the cap is
# reached, as Chen et al. (ICCAD 1998) and Tennakoon and Sechen (ICCAD 2002)
# solve each subproblem. The stop rules and trackers still run once per
# iteration. With a cap above 1, RSZ-0455 reports the sweeps and how many
# iterations stopped at the cap.
#
# Legs 1-3 run one iteration, so each is one subproblem solve, and start with
# -init_mode min_size, which puts every gate back at its minimum size, so each
# starts from the same netlist although the legs share one. The level-2 trace
# prints one line per sweep, and each leg checks how its loop stopped.
# Single-threaded for a deterministic golden.
source "helpers.tcl"

# Runs global sizing with the level-2 trace on and prints its log, keeping only
# the per-sweep lines of the trace. Returns what each sweep kept (moves minus
# the moves the cap re-check undid), in sweep order.
proc run_with_sweep_trace { } {
  set_debug_level RSZ global_sizing 2
  with_output_to_variable log { repair_timing -setup -phases GLOBAL_SIZING }
  set_debug_level RSZ global_sizing 0
  set kept {}
  foreach line [split [string trimright $log "\n"] "\n"] {
    if { [regexp {LR inner loop: sweep \d+ made (\d+) moves, (\d+) undone} \
            $line -> moves undone] } {
      lappend kept [expr { $moves - $undone }]
      puts $line
    } elseif { ![string match {\[DEBUG*} $line] } {
      puts $line
    }
  }
  return $kept
}

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup2.def
read_sdc repair_setup2.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

# Leg 1: the loop stops at the first sweep that keeps nothing, before the cap.
# The report and the config echo (RSZ-0417) show the setting.
puts "=== leg 1: cap 10, the loop stops when a sweep changes nothing ==="
set_global_sizing_config -preset rsz_baseline -init_mode min_size \
  -max_iterations 1 -max_inner_sweeps 10
report_global_sizing_config
set kept [run_with_sweep_trace]
if { [llength $kept] >= 10 || [lindex $kept end] != 0
     || [lsearch -exact [lrange $kept 0 end-1] 0] >= 0 } {
  error "expected sweeps that keep moves, then one that keeps none; got $kept"
}
puts "Stopped after [llength $kept] sweeps; the last one kept no move."

# Leg 2: the same solve with a cap of 2. The second sweep still keeps a move,
# so the cap stops the loop and RSZ-0455 counts the iteration.
puts "=== leg 2: cap 2, the cap stops the loop ==="
set_global_sizing_config -max_inner_sweeps 2
set kept [run_with_sweep_trace]
if { [llength $kept] != 2 || [lindex $kept end] == 0 } {
  error "expected 2 sweeps, the last one keeping a move; got $kept"
}
puts "Stopped at the cap; the last sweep kept [lindex $kept end] move(s)."

# Leg 3: without the setting, one sweep per iteration and no RSZ-0455.
puts "=== leg 3: reset, one sweep per iteration ==="
reset_global_sizing_config -max_inner_sweeps
if { [odb::dbIntProperty_find [rsz::get_block] "gs_max_inner_sweeps"] ne "NULL" } {
  error "reset_global_sizing_config -max_inner_sweeps left gs_max_inner_sweeps set"
}
puts "gs_max_inner_sweeps cleared"
set kept [run_with_sweep_trace]
if { [llength $kept] != 1 } {
  error "expected 1 sweep; got $kept"
}

# Leg 4: tennakoon_partial runs the loop with a cap of 10.
puts "=== leg 4: tennakoon_partial ==="
reset_global_sizing_config
set_global_sizing_config -preset tennakoon_partial -max_iterations 3
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# The cap must be at least 1. The Tcl command rejects a negative value, and the
# run rejects 0 (RSZ-0454).
puts "=== -max_inner_sweeps must be at least 1 ==="
if { ![catch { set_global_sizing_config -max_inner_sweeps -1 }] } {
  error "-max_inner_sweeps -1 was accepted"
}
set_global_sizing_config -max_inner_sweeps 0
if { ![catch { repair_timing -setup -phases GLOBAL_SIZING }] } {
  error "global sizing ran with -max_inner_sweeps 0"
}
