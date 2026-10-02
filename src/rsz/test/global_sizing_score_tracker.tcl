# The reimann_score best tracker (Reimann et al., ISPD 2016, Alg. 2) stores
# the netlist global sizing receives as its first best solution, with score 0,
# and replaces it only with an iterate that scores higher.
#
# The first run cannot beat its input: -init_mode max_size more than doubles
# leakage, and two iterations do not bring it back, so every iterate scores
# below 0. The tracker must return the input cell for cell, which also shows
# that the input is stored before the init pass.
#
# The next runs show that a normal run still returns its best iterate. On this
# design and clock, reimann_partial scores best at iteration 1 and ends at a
# worse iterate. The full run must return the netlist of a run capped at one
# iteration, which differs from both the input and the final iterate (the
# result with -best_tracker none).
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
create_clock [get_ports clk] -name core_clock -period 0.4

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

set input_refs [cell_refs]
puts "Instances: [dict size $input_refs]"

puts "=== no iterate beats the input: the input is returned ==="
set fallback_refs [run_sizing -preset rsz_baseline \
                     -best_tracker reimann_score -init_mode max_size \
                     -max_iterations 2]
set diffs [count_diffs $input_refs $fallback_refs]
if { $diffs != 0 } {
  error "$diffs instance(s) differ from the input"
}
puts "All [dict size $input_refs] instances have their input cells."

puts "=== reimann_partial capped at one iteration ==="
restore_input $input_refs
set iter1_refs [run_sizing -preset reimann_partial -max_iterations 1]

puts "=== reimann_partial, final iterate kept ==="
restore_input $input_refs
set final_refs [run_sizing -preset reimann_partial -best_tracker none]

puts "=== reimann_partial: the best iterate is returned ==="
restore_input $input_refs
set best_refs [run_sizing -preset reimann_partial]
set diffs [count_diffs $iter1_refs $best_refs]
if { $diffs != 0 } {
  error "$diffs instance(s) differ from the iteration-1 netlist"
}
set input_diffs [count_diffs $input_refs $best_refs]
set final_diffs [count_diffs $final_refs $best_refs]
if { $input_diffs == 0 || $final_diffs == 0 } {
  error "the best iterate must differ from the input and the final iterate"
}
puts "Iteration 1 returned: $input_diffs instance(s) differ from the input,\
  $final_diffs from the final iterate."
