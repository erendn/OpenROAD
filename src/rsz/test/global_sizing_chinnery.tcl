# chinnery_partial end to end: the two-phase LR schedule of Chinnery and Sharma
# (ISPD 2022). threshold_battery runs a timing phase, hands over to a power
# phase (RSZ-0450) and then stops on its own criteria (RSZ-0451).
#
# The phase also drives the multiplier update and a candidate filter. The
# sharma_arc_slack update (Sharma et al., TCAD 2020, Alg. 3) multiplies each
# arc's multiplier by (1 - s/T)^K, with K = 4 on critical and 1 on non-critical
# arcs in the timing phase and 1 and 4 in the power phase. -power_phase_filter
# skips, in the power phase, every candidate above the current cell's power. In
# TCAD 2020, Fig. 1, an iteration solves the subproblem, runs STA and then
# updates the multipliers, so the update right after the handover still ends a
# timing-phase iteration and uses the timing exponents, while the sweep after
# it is the first with the filter. Legs 1 and 5 print the update and filter
# lines of the debug trace to show that order: leg 1 hands over after its
# second iteration, leg 5 after its first.
#
# The clock is relaxed to 0.34 ns (as in global_sizing_closable) so the timing
# phase does real work and then finishes; at 0.1 ns sizing cannot close the
# design. Leg 1 must close timing and stop before max_iterations (80).
#
# The preset minimizes total power, so RSZ-0450/0451 report total power next to
# leakage.
#
# The power-phase exit on TNS degradation does not fire on this design; the
# ThresholdBattery unit tests in TestTermination cover it.
#
# The legs share one netlist: leg 1 closes the design and later legs start from
# that state. Single-threaded for a deterministic golden.
source "helpers.tcl"

# Runs global sizing with the level-2 trace on and prints its log, keeping only
# the trace lines of the multiplier update and the power-phase filter.
proc run_with_phase_trace { } {
  set_debug_level RSZ global_sizing 2
  with_output_to_variable log { repair_timing -setup -phases GLOBAL_SIZING }
  set_debug_level RSZ global_sizing 0
  foreach line [split [string trimright $log "\n"] "\n"] {
    if { ![string match {\[DEBUG*} $line]
         || [regexp {sharma_arc_slack:|power-phase filter:} $line] } {
      puts $line
    }
  }
}

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup2.def
read_sdc repair_setup2.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

# Relaxed-clock variant: raise the period so sizing alone can reach feasibility
# (identical to global_sizing_closable's setup).
create_clock [get_ports clk] -period 0.34
set_input_delay -clock clk 0.02 [get_ports a1]
set_input_delay -clock clk 0.02 [get_ports a2]
set_input_delay -clock clk 0.02 [get_ports a3]
set_output_delay -clock clk 0.02 [get_ports y1]
set_output_delay -clock clk 0.02 [get_ports y2]

set_thread_count 1

# Leg 1: the preset with no overrides, so the paper's constants and its
# 80-iteration cap apply.
puts "=== leg 1: chinnery_partial, both phases, closes ==="
set_global_sizing_config -preset chinnery_partial
run_with_phase_trace
report_worst_slack -max -digits 3

# Leg 2: the same preset with -termination pure_cap and a 12-iteration cap. The
# run goes to the cap and prints no RSZ-0450/0451, which only the battery
# emits. If the battery stopped working, leg 1 would look like this leg. The
# run never leaves the timing phase, so the update and the filter warn
# (RSZ-0439, RSZ-0440).
puts "=== leg 2: same bundle on pure_cap (no battery, no phase records) ==="
reset_global_sizing_config
set_global_sizing_config -preset chinnery_partial
set_global_sizing_config -termination pure_cap -max_iterations 12
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# Leg 3: the term_* constants are read only by threshold_battery. Setting one
# under another termination has no effect, so it warns (RSZ-0452) instead of
# failing.
puts "=== leg 3: term_* constants are inert off the battery (RSZ-0452) ==="
reset_global_sizing_config
set_global_sizing_config -termination fixed_iters -term_improve_window 1 \
  -max_iterations 2
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# Leg 4: with 2 iterations the power phase's 3-iteration improvement window
# never fills, so the iteration cap ends the run. RSZ-0451 must still report
# the stop ("iteration cap reached, no battery criterion met").
puts "=== leg 4: cap exit still reports (2 iterations, no criterion met) ==="
reset_global_sizing_config
set_global_sizing_config -preset chinnery_partial
set_global_sizing_config -max_iterations 2
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# Leg 5: the design already meets the timing target, so the timing phase ends
# at the first stop check. Exactly one update uses the timing exponents; every
# later one uses the power exponents.
puts "=== leg 5: handover at the first stop check, one timing update ==="
reset_global_sizing_config
set_global_sizing_config -preset chinnery_partial
set_global_sizing_config -max_iterations 3
run_with_phase_trace
report_worst_slack -max -digits 3

# Leg 6: the update and the filter on the default configuration, as options.
# The report and the config echo (RSZ-0417) show both, with the four exponents.
puts "=== leg 6: sharma_arc_slack and -power_phase_filter on rsz_baseline ==="
reset_global_sizing_config
set_global_sizing_config -preset rsz_baseline -lambda_update sharma_arc_slack \
  -termination threshold_battery -power_phase_filter 1
report_global_sizing_config
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

puts "=== -power_phase_filter takes a Boolean, and reset clears it ==="
if { ![catch { set_global_sizing_config -power_phase_filter maybe }] } {
  error "-power_phase_filter maybe was accepted"
}
reset_global_sizing_config -power_phase_filter
if { [odb::dbBoolProperty_find [rsz::get_block] "gs_power_phase_filter"] eq "NULL" } {
  puts "gs_power_phase_filter cleared"
} else {
  error "reset_global_sizing_config -power_phase_filter left gs_power_phase_filter set"
}
