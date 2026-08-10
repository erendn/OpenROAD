# M3 smoke test: exercise every B2 cost-model term of GlobalSizingPolicy on a
# small design. For each configuration, set_global_sizing_config selects the
# cost terms (all other axes stay at the rsz_baseline defaults), the
# GLOBAL_SIZING phase runs, and the worst slack is reported. This asserts each
# term's full path runs without error and produces the RSZ-0417/0400/0409 run
# record - the per-formula arithmetic is checked by the TestCostTerms gtest. The
# livramento and flach presets are exercised too (cost_fanout_slew and
# cost_global_phi respectively). Single-threaded for a deterministic golden.
#
# livramento_partial is also this suite's only timing_scale=livramento_alpha
# leg, so it is where the terminal-α record (RSZ-0444, iteration-2 plan §2.2-5)
# is pinned. Its golden line is a measurement worth reading rather than
# boilerplate: on this deliberately un-closable design α is driven all the way
# into its accumulator floor over the preset's 60 reschedules and
# alpha_floor_bound comes back TRUE - the α-runaway the S1-A review could only
# infer, now reported. Every other configuration in this file leaves the line
# out entirely, which is the point: only the option with a live α has one.
#
# OpenROAD Tcl has no in-session netlist reset, so the configurations run
# sequentially on one netlist: run 1 optimizes, later runs start from that state
# and mostly no-op. Each configuration's cost terms still dispatch (RSZ-0417).
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup2.def
read_sdc repair_setup2.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

# Individual cost-term flags on top of the baseline.
foreach {label args} {
  baseline           {}
  fanout_slew        {-cost_fanout_slew 1}
  global_phi         {-cost_global_phi 1}
  delta_delay        {-cost_delta_delay 1}
  upstream_off       {-cost_upstream_load 0}
  fanout_plus_phi    {-cost_fanout_slew 1 -cost_global_phi 1}
  fanout_plus_delta  {-cost_fanout_slew 1 -cost_delta_delay 1}
} {
  puts "=== cost_terms $label ==="
  reset_global_sizing_config
  eval set_global_sizing_config $args
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}

# Paper presets that turn a cost term on. Both also carry an init pass, so their
# legs open with RSZ-0416/0415 (and, for flach_partial's min_size_fixviol, the
# RSZ-0445 repair record). flach_partial's RSZ-0400 count is the round trip that
# creates: the reset drops all three gates to minimum, the repair upsizes the two
# that then violate max-cap, and the LR loop upsizes back to exactly the incoming
# netlist - RSZ-0409 stays +0.00% leakage / +0.00% area.
foreach preset {livramento_partial flach_partial} {
  puts "=== preset $preset ==="
  reset_global_sizing_config
  set_global_sizing_config -preset $preset
  # flach_partial's bundle carries the paper's ~120 iterations (Fig. 4). This
  # smoke only needs the cost term to dispatch, so cap it back to the shared
  # default. The override must come AFTER -preset (order-sensitive by design).
  if { $preset eq "flach_partial" } {
    set_global_sizing_config -max_iterations 20
  }
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}
