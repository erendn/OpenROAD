# it2 pass 4: the chinnery_partial preset end to end - Chinnery & Sharma's
# (ISPD'22) two-phase LR schedule, the ninth preset and the last paper column.
#
# WHY THIS DESIGN. The battery is the only termination option with a PHASE
# structure, so a smoke that only proves "it ran" proves nothing about it. This
# file uses the closable repair_setup2 variant (clock relaxed to 0.34 ns, the
# same asset global_sizing_closable builds) because it is the one design in the
# suite where the timing phase does REAL work and then finishes: the default
# 0.1 ns netlist is un-closable by sizing alone (every preset floors at WNS
# -0.157) and a met design hands over on iteration 1 without ever exercising a
# timing-phase criterion. Here the run walks both halves of the paper's §5 flow
# and closes.
#
# WHAT LEG 1 PINS, all read straight off the golden:
#   1. the BUNDLE - RSZ-0417 echoes every axis of the preset in one line:
#      seed=constant/lambda_init=1, update=sharma_cexp (the hosted
#      sharma-lineage rule; see the preset doc block for why it is a host and
#      not a citation), mu_policy=endpoint_lambda, projection=
#      proportional_reverse_topo (Chinnery's §6 projection, verbatim),
#      sweep=gauss_seidel_topo/forward_topo, guard=local_slack_veto with
#      gamma_local_slack=0 (the 1,000,000-weight reading), move_set=full_library,
#      timing_scale=unit/timing_bias=1, term=threshold_battery with the paper's
#      five constants + 72 h cap, best=none, max_iter=80, upsize_hyst=0.
#   2. the HANDOVER - RSZ-0450 after a 4-iteration timing phase, on the TNS
#      improvement criterion. The handover is a phase change, NOT a stop: the
#      run keeps sweeping, which is the property a regression would break by
#      turning the timing targets back into exits.
#   3. the POWER-PHASE STOP - RSZ-0451 at 8 iterations, on the power-improvement
#      criterion, i.e. the battery (not max_iterations = 80) is what ends the
#      run. The record states the paper's own convergence figure, the phase
#      split (4 timing + 4 power here, vs its 13 + 10 average).
#   4. TIMING CLOSES - RSZ-0409 "WNS -0.038 -> 0.018", worst slack max met.
#
# WHAT THE POWER PHASE IS HERE, since the golden shows it plainly and a reader
# should not have to infer it: leakage RISES across it (7.89e-08 at the handover
# to 9.63e-08 at the stop, +22%). That is not a bug and not a bad run - it is
# the shipped ADAPTATION. Chinnery's phases also swap the multiplier-update
# exponents (§6, deferred to Sharma TCAD'20 [7] - REAUDIT H6) and ours do not,
# so a phase change here alters WHICH EXITS ARE LIVE and nothing else: the
# second phase keeps running the same timing-pressure sweep and stops as soon as
# its power-improvement criterion has a window to fire on. So "power-reduction
# phase" in these records names the paper's phase, not an observed behavior, and
# a chinnery campaign column should be read the same way. It is the headline
# entry of the preset's DEFERRED list for exactly this reason.
#
# NOT EXERCISED HERE, deliberately recorded: the battery's second power-phase
# exit (TNS degrading past its end-of-timing-phase value) does not fire anywhere
# in this suite. The reason is the preset's own local-slack veto with no
# hill-climbing tolerance, which rejects the moves that would degrade timing -
# but that is a TENDENCY, not a guarantee: the veto is local (driver and sink
# nets only) and, under gs_local, reads slacks frozen at sweep start, so an
# accumulation of locally-clean moves can still cost an endpoint. On a real
# benchmark the exit can and per the paper does fire (24-36% of its runs). It is
# pinned by TestTermination (ThresholdBattery.TnsDegraded* and
# ThresholdBatteryRecords.*, six cases) rather than by a golden, and reaching it
# end-to-end needs a design this suite does not have. Do not manufacture one by
# loosening the veto here - that would change what the column measures.
#
# OpenROAD Tcl has no in-session netlist reset, so the legs run sequentially on
# one netlist: leg 1 closes the design and legs 2-3 start from that state.
# Single-threaded for a deterministic golden.
source "helpers.tcl"

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

# Leg 1: the preset exactly as shipped - no overrides, so the golden records the
# paper's own constants and the paper's own 80-iteration cap.
puts "=== leg 1: chinnery_partial, both phases, closes ==="
set_global_sizing_config -preset chinnery_partial
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# Leg 2: the battery is what stops leg 1, not the cap. Same bundle with the
# termination swapped to pure_cap and the cap cut to 12 (80 would only make the
# point slowly): the run now sweeps until the cap instead of stopping itself,
# and emits no RSZ-0450/0451 at all because those records belong to the battery.
# A regression that neutered the battery into "run to the cap" would make leg 1
# look like this one.
puts "=== leg 2: same bundle on pure_cap (no battery, no phase records) ==="
reset_global_sizing_config
set_global_sizing_config -preset chinnery_partial
set_global_sizing_config -termination pure_cap -max_iterations 12
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# Leg 3: RSZ-0452, the H1 twin of RSZ-0442/0449. The six term_* constants are
# read by threshold_battery alone, so varying one under any other rule gives
# bit-identical runs while RSZ-0417 keeps echoing a different `chinnery=` field
# - a within-arm variance of zero that reads as data instead of as the
# misconfigured sweep it is. Warned, not rejected: the value is harmless.
puts "=== leg 3: term_* constants are inert off the battery (RSZ-0452) ==="
reset_global_sizing_config
set_global_sizing_config -termination fixed_iters -term_improve_window 1 \
  -max_iterations 2
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# Leg 4: the cap exit still publishes a record. With only 2 iterations the
# power phase's improvement window (3) never fills, so no exit criterion can
# fire and the run ends on max_iterations - the "did not converge" case, which
# is precisely the one worth reading. RSZ-0451 comes from the end-of-loop hook
# instead of from a stop verdict, and says so: reason "iteration cap reached, no
# battery criterion met", with the 1 + 1 split still stated. (The design is
# already closed by leg 1, so the timing phase hands over on its first iteration
# - RSZ-0450 is present here; what is absent is any battery STOP.) Without the
# hook this run would print no stop record at all and a harness could not tell
# it from a dropped log line.
puts "=== leg 4: cap exit still reports (2 iterations, no criterion met) ==="
reset_global_sizing_config
set_global_sizing_config -preset chinnery_partial
set_global_sizing_config -max_iterations 2
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
