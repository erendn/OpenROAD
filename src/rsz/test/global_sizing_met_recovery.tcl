# Pre-campaign engine fix 2 (2026-07-29): GLOBAL_SIZING must RUN, MOVE and KEEP
# its moves on a design that already meets timing, driven from the REAL command
# path. This is the regression the gs-guard session did not write.
#
# THE BUG this pins. Two independent WNS-conditioned mechanisms turned a met
# design into a no-op, and the campaign's control column hit both:
#   * fixed_iters' pre-sweep exit stopped the LR loop before iteration 0 once WNS
#     met setup_slack_margin -> "0/0 sweeps accepted, 0 cells replaced". Only
#     rsz_baseline is on fixed_iters, so this silently zeroed the BASELINE - the
#     denominator every paper arm is scored against.
#   * the outer phase accept (gs-guard's `outer_guard`) then rolled the init
#     pass + LR back whenever WNS_after < WNS_pre, and power recovery SPENDS
#     positive slack, so on a met design every leakage win failed that test.
# Both are gone for every preset, baseline included. No paper in the collection
# gates its LR entry or its termination on timing being met (they all minimize
# leakage/power/area SUBJECT TO timing, so a feasible input is normal - and for
# reimann/mangiras/chinnery/tennakoon it is the INTENDED input).
#
# WHY THE REAL COMMAND PATH MATTERS. The predecessor of this test called
# `repair_timing -setup -phases GLOBAL_SIZING` and asserted the outer accept's
# A/B on mangiras only. Both of its premises were true and it still missed the
# defect: mangiras is on stagnation_windows, so it never met the fixed_iters
# exit, and ITS OWN third leg recorded rsz_baseline's "0/0 sweeps" as CORRECT -
# a golden that pinned the bug. The call
# below is what ORFS's repair_timing_helper produces at the injection point
# (flow/scripts/detail_place.tcl, post-DPL) - no -setup, no -setup_slack_margin,
# trailing -verbose. Note that with neither -setup nor -hold, repair_timing sets
# BOTH (Resizer.tcl), so the helper form also runs repair_hold; that tail is part
# of the genuine path and is pinned here too.
#
# OpenROAD Tcl has no in-session netlist reset, so the legs run sequentially on
# one netlist. rsz_baseline goes FIRST, on the pristine netlist, because it is
# the arm that used to be a hard zero. Single-threaded for a deterministic
# golden.
#
#   1. rsz_baseline through the real path. Must report NONZERO RSZ-0400 counts
#      (the exact symptom the user observed was zero), spend area/leakage moves
#      on a met design, keep them (no outer rollback exists any more), and end
#      MET. It stops at 4 sweeps of its 20-iteration cap, which is the KEPT
#      fixed_iters exit doing its job: two consecutive zero-move passes, a
#      move-driven stop that only fires after the sweep has actually run. That
#      is the whole difference from the removed exit, which fired at iteration 0
#      before any sweep, on the timing state alone.
#   2. mangiras_partial through the same call. Same contract: it moves, its
#      flach_dominance restore survives, WNS stays met, leakage and area drop.
#   3. sharma_seq_partial - the only preset with near_met_gate=0.01, so a met
#      design latches near-met at iteration 0 and its stagnation monitor is live
#      immediately. That latch is the one WNS-conditioned mechanism deliberately
#      KEPT (it activates machinery, it does not gate entry), so this leg is here
#      to pin that keeping it does not reintroduce the no-op. Since the it2
#      fixviol rider it also runs an init pass (min_size_fixviol), which on this
#      design is nearly inert: RSZ-0415 resets 1 of 15 gates and RSZ-0445 finds
#      0 electrical violations, and the LR loop was going to upsize that gate
#      anyway - so the leg's RSZ-0400/0409 lines and every later leg are
#      byte-identical to the pre-rider golden. Only the three new records move.
#   4. -termination threshold_battery on a met design: its WNS/TNS target
#      criteria are Chinnery's timing->power phase HANDOVER, not a stop, so a met
#      design must hand over on iteration 1 and keep sweeping. This is the only
#      surviving option with WNS-keyed criteria and it has a live M7 cell (S1-K2),
#      so a regression turning the handover back into a stop would show up here as
#      a 1/1-sweep run. Since it2 pass 4 the phase boundary says so in words -
#      RSZ-0450 names the criterion and the iteration, RSZ-0451 the stop - so
#      that regression now moves a record, not only a count.
#   5. the knob is GONE, not defaulted: -outer_guard is rejected as an unknown key
#      by BOTH set_global_sizing_config and reset_global_sizing_config. RSZ-0417's
#      echo has no outer_guard= field either, and report_global_sizing_config has
#      no such row - both visible in the other global_sizing goldens, so this leg
#      does not re-dump the 50-row config report to say it again.
#
# LEG ENTANGLEMENT (read before inserting a leg). Legs share one netlist, so each
# starts from its predecessor's committed state; leg 2's percentages are smaller
# than a pristine-netlist run would give because leg 1 takes two of the downsizes
# first. Leg 2 is also the only leg that would catch a reintroduced end-of-phase
# accept in its previously-shipped baseline-only shape, since leg 1's WNS RISES
# (1.760 -> 1.764) and would pass such an accept unchanged. And because the ORFS
# call form runs repair_hold too, RSZ-0033 lands between legs - a hold-repair
# change would shift every later leg's numbers.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def recover_power1.def
create_clock -period 2.0 clk

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

# The design meets timing before anything runs - this is the whole premise.
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
