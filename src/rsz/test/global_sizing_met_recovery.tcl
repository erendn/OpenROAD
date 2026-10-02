# GLOBAL_SIZING must make moves and keep them on a design that already meets
# timing. Power recovery spends positive slack, so a met design is a normal
# input and WNS may get worse; neither entering the LR loop nor keeping its
# moves may depend on timing being met.
#
# repair_timing is called the way ORFS's repair_timing_helper calls it after
# detailed placement: no -setup, with -verbose. Without -setup or -hold,
# repair_timing does both, so repair_hold (RSZ-0033) also runs after each leg.
#
# The legs share one netlist, each starting from the previous leg's result.
# Single-threaded for a deterministic golden.
#
#   1. rsz_baseline: makes moves, keeps them, stays met. fixed_iters stops
#      after two passes with no moves, which can only happen after a sweep.
#   2. mangiras_partial: moves are kept even though WNS gets worse.
#   3. sharma_seq_partial: a met design latches near-met at iteration 0; the
#      latch only enables the stagnation monitor, and the loop must still run.
#   4. threshold_battery: meeting the WNS/TNS targets ends the timing phase
#      (RSZ-0450), not the run; sweeping continues until RSZ-0451.
#   5. -outer_guard (roll the phase back when WNS gets worse) is not an option.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def recover_power1.def
create_clock -period 2.0 clk

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

# The design meets timing before global sizing runs.
puts "=== met at iteration 0 ==="
report_worst_slack -max -digits 4

puts "=== leg 1: rsz_baseline, ORFS helper call form, met design ==="
reset_global_sizing_config
set_global_sizing_config -preset rsz_baseline
repair_timing -phases "GLOBAL_SIZING" -verbose
report_worst_slack -max -digits 4

puts "=== leg 2: mangiras_partial, same call form ==="
reset_global_sizing_config
set_global_sizing_config -preset mangiras_partial
repair_timing -phases "GLOBAL_SIZING" -verbose
report_worst_slack -max -digits 4

puts "=== leg 3: sharma_seq_partial (near-met latch fires at iteration 0) ==="
reset_global_sizing_config
set_global_sizing_config -preset sharma_seq_partial
repair_timing -phases "GLOBAL_SIZING" -verbose
report_worst_slack -max -digits 4

puts "=== leg 4: threshold_battery on a met design (handover, not a stop) ==="
reset_global_sizing_config
set_global_sizing_config -termination threshold_battery -max_iterations 6
repair_timing -phases "GLOBAL_SIZING" -verbose
report_worst_slack -max -digits 4

puts "=== leg 5: -outer_guard is gone, not merely off ==="
if { [catch { set_global_sizing_config -outer_guard 1 }] } {
  puts "set_global_sizing_config -outer_guard rejected"
} else {
  puts "ERROR: -outer_guard still accepted"
}
if { [catch { reset_global_sizing_config -outer_guard }] } {
  puts "reset_global_sizing_config -outer_guard rejected"
} else {
  puts "ERROR: reset_global_sizing_config -outer_guard still accepted"
}
