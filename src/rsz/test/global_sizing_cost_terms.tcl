# Smoke test for the cost terms. Each run enables some terms, on top of the
# default configuration or of a paper preset, and must complete and print the
# RSZ-0417/0400/0409 records; the arithmetic is unit-tested by TestCostTerms.
#
# livramento_partial is the only run with timing_scale livramento_alpha, so it
# is the only one that prints RSZ-0444. This design cannot be closed, so alpha
# is driven to its floor and the record reports alpha_floor_bound=true.
#
# The runs share one netlist: the first optimizes it and later runs mostly make
# no moves, but each still selects its terms (RSZ-0417). Single-threaded for a
# deterministic golden.
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

# Paper presets and their cost terms: livramento_partial turns on
# cost_fanout_slew, and flach_partial leaves cost_global_phi off, as Flach et
# al. do with RC wires. Their init passes add RSZ-0416/0415 (and RSZ-0445 for
# flach_partial's min_size_fixviol) records.
foreach preset {livramento_partial flach_partial} {
  puts "=== preset $preset ==="
  reset_global_sizing_config
  set_global_sizing_config -preset $preset
  # flach_partial uses the paper's ~120 iterations (Fig. 4). This test only
  # needs the preset to run, so cap it at the default 20. The override must
  # come after -preset.
  if { $preset eq "flach_partial" } {
    set_global_sizing_config -max_iterations 20
  }
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}

# An option given after the preset overrides it: cost_global_phi=true in
# RSZ-0417.
puts "=== preset flach_partial -cost_global_phi 1 ==="
reset_global_sizing_config
set_global_sizing_config -preset flach_partial
set_global_sizing_config -max_iterations 20 -cost_global_phi 1
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3
