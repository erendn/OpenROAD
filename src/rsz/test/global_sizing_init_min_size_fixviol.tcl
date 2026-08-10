# N-axis (initial solution) smoke: -init_mode min_size_fixviol.
#
# ONE MODE PER FILE, BY NECESSITY - see global_sizing_init_min_size.tcl for the
# reasoning. This is the fifth file of that set and reads the same design with
# the same setup, so LEG 1's RSZ-0415/0400/0409 lines are directly comparable to
# global_sizing_init_{min_size,max_size,average,random}'s.
#
# min_size_fixviol is min_size followed by a reverse-topological electrical
# repair pass (iteration-2 plan §2.1, RA F6): after the min-size reset it walks
# the same editable gates from outputs toward inputs and upsizes each one whose
# own output pins violate max-cap or max-slew, to the lowest-leakage member of
# its swappable group that clears them. Outputs-first because repairing a gate
# raises its input capacitance, which is load on its DRIVERS - so by the time a
# driver is judged, its repaired fanout is already in its load.
#
# THE HONEST NUMBERS ON THIS DESIGN (leg 1). The min-size reset introduces NO
# electrical violation here - reg1's nets are short and its Liberty limits are
# generous - so the repair pass reports 0 of 14 and changes nothing, and leg 1's
# RSZ-0400/0409 lines are IDENTICAL to global_sizing_init_min_size's: the same
# 12 cells replaced, the same -15.51% leakage, the same -0.021 worst slack, with
# max-cap and max-slew clean in both. That is the comparison the plan asks for -
# min_size's leakage win with no more electrical violations - and on this design
# it comes out as an exact tie rather than as an improvement. Pinning the tie is
# the point: it says the repair costs nothing where there is nothing to repair.
#
# LEGS 2 AND 3 make the pass visible, by constraining the design until the reset
# to minimum actually breaks something. Both branches of the pass are exercised:
#   leg 2 (max_transition 0.06): 2 gates violate, both cleared by upsizing;
#   leg 3 (max_transition 0.03): 2 gates violate and NEITHER can be cleared -
#       no member of their swappable group meets the limit - so the pass leaves
#       them at minimum and RSZ-0445 says so. It repairs violations; it does not
#       spend leakage on a gate it cannot fix, and it does not pretend to have.
#
# LEGS 3 AND 4 ARE THE A2 PAIR (re-audit delta D1), and the reason this file
# hosts it: leg 3 is the only place in the suite where a gate the repair cannot
# fix actually exists, which is the one state the two output_drc_veto modes
# disagree about. Under the shipped `absolute` default (leg 3) the sweep's DRC
# filter rejects EVERY candidate for such a gate - none of them clears, by
# construction, since that is why the repair gave up - so the gate is pinned at
# minimum size for the whole run. Under `relative` (leg 4) the filter admits any
# candidate that does not make the violation WORSE, which is Flach Alg. 4
# line 6's actual rule, so the same gate can climb back out. flach_partial and
# chinnery_partial pin relative; rsz_baseline, which these legs run, keeps
# absolute, so leg 4 sets the knob by hand.
#   leg 4 (relative): the climb-out. Same constraint, same repair, more moves.
#   leg 5: reset_global_sizing_config -output_drc_veto puts the default back -
#       the RSZ-0417 echo is what pins the reset path, and the run lands back on
#       leg 3's exact end state from the opposite direction, which is the
#       control that makes the pair a demonstration rather than an anecdote.
# Legs 3-5 also report the violators, because "never worsens an existing
# violation" is the axis' whole claim and a leg that printed only worst slack
# could not tell a working relative bar from a vacuous one.
#
# OpenROAD Tcl has no in-session netlist reset, so every leg from 2 on starts
# from the previous leg's netlist - leg 5's apparent regression is exactly that,
# it inherits leg 4's climbed-out netlist. The min-size reset re-normalizes most
# of that away, but the swappable group is defined relative to the incoming
# cell, so the legs are order-sensitive by design and their goldens are pinned
# as a sequence.
#
# Single-threaded for a deterministic golden.
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
