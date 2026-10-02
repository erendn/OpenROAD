# Smoke test for -downsize_guard (depth_budget, local_slack_veto, none), the
# veto's gamma tolerance, the option combinations that warn
# (RSZ-0429/0430/0431), and the presets that use Flach's acceptance test. Each
# run must complete and print the RSZ-0417/0400/0409 records; the veto
# arithmetic (Flach et al., TCAD 2014, Eq. 14) is unit-tested by TestGuards.
# The veto reads required times during the sweep, so it runs under both
# Gauss-Seidel refresh modes.
#
# The runs share one netlist: the first optimizes it and later runs mostly make
# no moves, but each still selects its guard (RSZ-0417). Single-threaded for a
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

# The guards under the Gauss-Seidel engine, plus the gamma extremes
# (-gamma_local_slack 0 allows no local slack degradation at all). The veto runs
# first, on the unoptimized netlist, so the golden records its QoR. The
# depth_budget QoR is covered by global_sizing.ok and global_sizing_preset.ok.
foreach {label args} {
  gs_veto_local        {-sweep_engine gauss_seidel_topo -downsize_guard local_slack_veto -gs_refresh gs_local}
  gs_veto_incremental  {-sweep_engine gauss_seidel_topo -downsize_guard local_slack_veto -gs_refresh gs_incremental}
  gs_veto_no_climb     {-sweep_engine gauss_seidel_topo -downsize_guard local_slack_veto -gamma_local_slack 0}
  gs_veto_permissive   {-sweep_engine gauss_seidel_topo -downsize_guard local_slack_veto -gamma_local_slack 2}
  gs_no_guard          {-sweep_engine gauss_seidel_topo -downsize_guard none}
  jacobi_no_guard      {-sweep_engine jacobi_snapshot -downsize_guard none}
  gs_depth_budget      {-sweep_engine gauss_seidel_topo -downsize_guard depth_budget}
} {
  puts "=== downsize_guard $label ==="
  reset_global_sizing_config
  eval set_global_sizing_config $args
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}

# The guard/engine crosses: allowed, but each warns (RSZ-0430 / RSZ-0431).
puts "=== cross jacobi + local_slack_veto (warns RSZ-0430) ==="
reset_global_sizing_config
set_global_sizing_config -sweep_engine jacobi_snapshot \
  -downsize_guard local_slack_veto
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

puts "=== cross gauss_seidel + depth_budget (warns RSZ-0431) ==="
reset_global_sizing_config
set_global_sizing_config -sweep_engine gauss_seidel_topo \
  -downsize_guard depth_budget
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# The two slew-coupling cost terms both price the immediate sink level, so
# enabling both counts it twice. Allowed, but warns (RSZ-0429).
puts "=== cross cost_global_phi + cost_fanout_slew (warns RSZ-0429) ==="
reset_global_sizing_config
set_global_sizing_config -cost_global_phi 1 -cost_fanout_slew 1
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# The presets whose papers use Flach's acceptance test (Sharma et al. apply it
# during power recovery, Sec. 5). flach_partial and sharma_seq_partial also run
# min_size_fixviol or min_size_fixcap, which add RSZ-0416/0415/0445 records,
# and sharma_seq_partial runs the slew fix pass (RSZ-0461).
foreach preset {flach_partial reimann_partial mangiras_partial \
  sharma_seq_partial} {
  puts "=== preset $preset (local_slack_veto) ==="
  reset_global_sizing_config
  set_global_sizing_config -preset $preset
  # flach_partial uses the paper's ~120 iterations (Fig. 4). This test only
  # checks that the veto runs, so cap it at the default 20. The override must
  # come after -preset.
  if { $preset eq "flach_partial" } {
    set_global_sizing_config -max_iterations 20
  }
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}

# report_global_sizing_config must list every guard option. -upsize_hysteresis
# 0 is legal (the paper presets use it) and must be reported, not treated as
# unset.
puts "=== report_global_sizing_config ==="
reset_global_sizing_config
set_global_sizing_config -downsize_guard local_slack_veto \
  -gamma_local_slack 1.5 -upsize_hysteresis 0 -output_drc_veto relative
report_global_sizing_config
