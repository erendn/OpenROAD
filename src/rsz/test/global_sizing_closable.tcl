# sharma_seq_partial on repair_setup2 with the clock relaxed to 0.34 ns, so
# sizing alone can close the design. (At 0.1 ns it cannot, the near-met latch
# never fires and the run always uses its whole iteration budget.) The golden
# checks that:
#   1. timing closes (RSZ-0409);
#   2. the near-met latch (WNS >= -0.01 * clock period) fires; the
#      stagnation_windows termination only stops the run after the latch, so
#      the run stops well before the 40-iteration cap;
#   3. the multipliers stay bounded; otherwise the run would never latch;
#   4. the design ends with no max-slew or max-cap violations.
#
# The preset's min_size_fixcap init pass upsizes U3 and U5, whose outputs
# violate max-cap in the incoming netlist; its slew fix pass finds no gate over
# its slew limit. The LR loop never fixes existing violations (it only rejects
# moves that make them worse), so without that repair the run would end with a
# violation.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup2.def
read_sdc repair_setup2.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

# Relaxed-clock variant: raise the period so sizing alone can reach feasibility.
create_clock [get_ports clk] -period 0.34
set_input_delay -clock clk 0.02 [get_ports a1]
set_input_delay -clock clk 0.02 [get_ports a2]
set_input_delay -clock clk 0.02 [get_ports a3]
set_output_delay -clock clk 0.02 [get_ports y1]
set_output_delay -clock clk 0.02 [get_ports y2]

# Set the cap after -preset, which would otherwise overwrite it. The cap only
# bounds the runtime if the latch breaks; a correct run stops itself earlier.
set_global_sizing_config -preset sharma_seq_partial
set_global_sizing_config -max_iterations 40
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# Check 4. Without the repair pass RSZ-0409 would only show a smaller leakage
# increase, so check the violators directly. Both sections must be empty.
puts "=== ERC after global sizing ==="
report_check_types -max_slew -max_capacitance -violators
