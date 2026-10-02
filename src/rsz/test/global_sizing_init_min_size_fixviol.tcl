# Smoke test for -init_mode min_size_fixviol: min_size followed by an electrical
# repair pass. The pass walks the editable gates from outputs to inputs and
# upsizes each gate whose outputs violate max-cap or max-slew to the
# lowest-leakage member of its swappable group that clears the violation.
# Outputs go first because a repaired gate adds input capacitance to its
# drivers.
#
# Leg 1 uses the same setup as the other init_mode tests (see
# global_sizing_init_min_size.tcl); nothing violates, so it matches the
# min_size result. Legs 2 and 3 tighten max_transition: in leg 2 the repair
# clears both violating gates; in leg 3 no group member meets the limit, so
# the gates stay at minimum and RSZ-0445 reports them.
#
# Legs 3-5 compare -output_drc_veto on those gates. absolute (the default, legs
# 3 and 5) rejects every candidate that does not clear the violation, so they
# stay at minimum; relative (leg 4) accepts any candidate that does not make
# the violation worse (Flach et al., TCAD 2014, Alg. 4 line 6), so they can be
# upsized again. Leg 5 resets the option and returns to leg 3's result.
#
# The legs share one netlist, so each leg from 2 on starts from the previous
# leg's result and the goldens depend on leg order. Single-threaded for a
# deterministic golden.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup_dont_touch_sizeup.def
create_clock -period 0.35 clk
set_load 1.0 [all_outputs]

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

puts "=== leg 1: the E1-comparable leg, no extra electrical constraint ==="
set_global_sizing_config -init_mode min_size_fixviol
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
report_check_types -max_capacitance -max_slew -violators

puts "=== leg 2: a slew limit the min-size reset breaks, and the pass repairs ==="
set_max_transition 0.06 [current_design]
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

puts "=== leg 3: a slew limit no group member can meet - reported, not hidden ==="
set_max_transition 0.03 [current_design]
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
report_check_types -max_capacitance -max_slew -violators

puts "=== leg 4: same state under -output_drc_veto relative - the climb-out ==="
set_global_sizing_config -output_drc_veto relative
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
report_check_types -max_capacitance -max_slew -violators

puts "=== leg 5: the reset path puts the absolute default back (RSZ-0417) ==="
reset_global_sizing_config -output_drc_veto
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
report_check_types -max_capacitance -max_slew -violators
