# Closable-benchmark regression smoke for the LR global-sizing engine (C1-C4).
#
# The default repair_setup2 (clock 0.1 ns) is UN-CLOSABLE by sizing alone: every
# preset floors at WNS -0.157 ns, so the sharma near-met latch (fires at
# WNS >= -0.01*T) never fires and the compounding updaters run their full budget
# while lambda cosmetically overflows (C2 completion notes). This test relaxes
# the clock to 0.34 ns so global sizing ALONE closes the design, and pins the
# three properties C1-C3 built, on the one preset that exercises all of them:
#
#   * sharma_seq_partial's C1 re-base (paper (1 - slack/T)^cexp formula),
#   * C2's driver-owned near-met latch + stagnation/guard gating (near_met_gate
#     = 0.01 in the RSZ-0417 echo),
#   * C3's consistent-transition arc reads feeding the updater.
#
# Acceptance, all read straight off the golden:
#   1. timing CLOSES     -> RSZ-0409 "WNS -0.038 -> 0.045"; worst slack max met.
#   2. the LATCH FIRES   -> RSZ-0400 stops at 15/15 sweeps, well under the 40 cap
#                           (term=stagnation_windows only stops post-latch; on the
#                           un-closable design sharma runs to its full cap).
#   3. lambda stays BOUNDED -> the run reaches feasibility and self-damps, so it
#                           closes+latches instead of overflowing; a regressed
#                           self-damp would miss near-met, skip the latch, and run
#                           to the cap, breaking this golden.
#
# WHERE THE MARGIN COMES FROM (it2 fixviol rider, 2026-08-08). Since
# sharma_seq_partial pins init_mode=min_size_fixviol, the run no longer starts
# from the incoming netlist. On THIS design the min-size reset is a no-op
# (RSZ-0415 "0/3": repair_setup2 ships all-_X1, i.e. already at minimum leakage)
# but the electrical repair is not: the pristine netlist VIOLATES max-cap on
# U3/ZN and U5/ZN, and RSZ-0445 upsizes both to clear it. That is why the
# numbers above moved, and the move is a legality fix rather than a QoR loss -
# the previous golden closed at 0.018 while SHIPPING U5/ZN's max-cap violation
# (the always-on load veto only forbids INCREASING violation, so nothing in the
# LR loop ever repaired it). The repaired start also lands feasible immediately,
# so the loop itself now makes zero moves and the stagnation monitor stops it
# earlier - 15 sweeps rather than 22. The latch still fires, which is the
# property this file exists to pin. Acceptance 4 below pins the legality half of
# that trade, so a dropped repair fails as a violation rather than passing as a
# smaller leakage rise.
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

# Cap after -preset (order-sensitive): 40 bounds runtime if a regression breaks
# the latch; the healthy run stops itself at 15 via post-latch stagnation.
set_global_sizing_config -preset sharma_seq_partial
set_global_sizing_config -max_iterations 40
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# Acceptance 4, and the one the fixviol rider's leakage number is TRADED for:
# the run must END electrically legal. Nothing else in this file would notice a
# dropped repair - RSZ-0409 would simply read a smaller leakage rise, which
# looks like an improvement - so the ERC state is pinned here explicitly. Empty
# under both headings is the pass; the pre-rider run left U5/ZN violating.
puts "=== ERC after global sizing ==="
report_check_types -max_slew -max_capacitance -violators
