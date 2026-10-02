# The livramento_feasible best tracker (Livramento et al., DATE 2013, Alg. 1
# line 20) returns the lowest-power iterate that has no setup, max capacitance
# or max slew violation. livramento_partial uses it.
#
# On gcd (nangate45) with a 0.68 ns clock, livramento_partial's first
# iteration has a negative WNS, so it is skipped although its leakage is lower
# than that of the iteration returned. A run capped at one iteration shows
# both, and its RSZ-0460 line reports that the final iteration was kept. The
# next five iterations meet timing and have no max capacitance or max slew
# violation, and the second has the lowest leakage of them. The run must
# return the netlist of a run capped at two iterations, which differs from its
# final iterate (the result with -best_tracker none). RSZ-0460 names the
# iteration, and report_check_types finds no max capacitance or max slew
# violation afterwards.
#
# The last part checks the option itself: the report row, the RSZ-0417 echo
# and the rejection of an unknown value.
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

proc design_leakage { } {
  lassign [sta::design_power [sta::cmd_scene]] internal switching leakage total
  return $leakage
}

proc run_sizing { args } {
  reset_global_sizing_config
  set_global_sizing_config {*}$args
  repair_timing -setup -phases GLOBAL_SIZING
  return [cell_refs]
}

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def gcd_nangate45_placed.def
read_sdc gcd_nangate45.sdc
create_clock [get_ports clk] -name core_clock -period 0.68

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

set input_refs [cell_refs]

puts "=== livramento_partial capped at one iteration ==="
run_sizing -preset livramento_partial -max_iterations 1
set iter1_wns [sta::worst_slack -max]
set iter1_leakage [design_leakage]

puts "=== livramento_partial capped at two iterations ==="
restore_input $input_refs
set iter2_refs [run_sizing -preset livramento_partial -max_iterations 2 \
                  -best_tracker none]

puts "=== livramento_partial, final iterate kept ==="
restore_input $input_refs
set final_refs [run_sizing -preset livramento_partial -max_iterations 6 \
                  -best_tracker none]

puts "=== livramento_partial: iteration 2 is returned ==="
restore_input $input_refs
set best_refs [run_sizing -preset livramento_partial -max_iterations 6]
set diffs [count_diffs $iter2_refs $best_refs]
if { $diffs != 0 } {
  error "$diffs instance(s) differ from the iteration-2 netlist"
}
set final_diffs [count_diffs $final_refs $best_refs]
if { $final_diffs == 0 } {
  error "the returned netlist must differ from the final iterate"
}
puts "Iteration 2 returned: $final_diffs instance(s) differ from the final\
  iterate."
if { $iter1_wns >= 0 || $iter1_leakage >= [design_leakage] } {
  error "iteration 1 must have a negative WNS and less leakage than iteration 2"
}
report_worst_slack -max
report_check_types -max_capacitance -max_slew -violators
set cap_violations [sta::max_capacitance_violation_count]
set slew_violations [sta::max_slew_violation_count]
puts "max capacitance violations: $cap_violations,\
  max slew violations: $slew_violations"
if { $cap_violations != 0 || $slew_violations != 0 } {
  error "the returned netlist has max capacitance or max slew violations"
}

puts "=== -best_tracker livramento_feasible ==="
restore_input $input_refs
reset_global_sizing_config
set_global_sizing_config -preset rsz_baseline \
  -best_tracker livramento_feasible -max_iterations 2
report_global_sizing_config
repair_timing -setup -phases GLOBAL_SIZING

puts "=== an unknown value is rejected ==="
if { ![catch { set_global_sizing_config -best_tracker livramento }] } {
  error "-best_tracker livramento was accepted"
}
