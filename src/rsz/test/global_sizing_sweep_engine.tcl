# M4 smoke test: exercise the F1/F2 sweep-engine axis of GlobalSizingPolicy on a
# small design - both engines (jacobi_snapshot, gauss_seidel_topo) x the three
# GS traversals (forward_topo, reverse_topo, criticality_sorted) x both GS
# refresh modes (gs_local, gs_incremental), plus the estimation-loop-under-GS
# path and a paper preset now on GS. For each configuration
# set_global_sizing_config selects the engine/traversal/refresh (all other axes
# stay at the rsz_baseline defaults), the GLOBAL_SIZING phase runs, and the
# worst slack is reported. This asserts each configuration's full path
# (JIT snapshot build + evaluate + commit + per-commit refresh, or the Jacobi
# parallel path) runs without error and produces the RSZ-0417/0400/0409 run
# record. The traversal ORDERING logic is unit-tested by TestSweepEngine.
#
# Determinism: the Gauss-Seidel engine is single-threaded and orders gates by a
# total (key, stable-instance-id) sort, so it is deterministic by construction -
# the GS cell-assignment signature dumped below is pinned by the golden, so any
# nondeterminism would show up as a golden mismatch. (A separate process-level
# check confirmed identical 571-cell assignments across repeated runs on gcd.)
#
# OpenROAD Tcl has no in-session netlist reset, so the configurations run
# sequentially on one netlist: run 1 optimizes, later runs start from that state
# and mostly no-op. Each configuration still dispatches its engine (RSZ-0417).
# Single-threaded for a deterministic golden.
source "helpers.tcl"

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup2.def
read_sdc repair_setup2.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

# Both engines x the three GS traversals x both GS refresh modes.
foreach {label args} {
  jacobi_snapshot   {-sweep_engine jacobi_snapshot}
  gs_local_fwd      {-sweep_engine gauss_seidel_topo -gs_refresh gs_local -traversal forward_topo}
  gs_local_rev      {-sweep_engine gauss_seidel_topo -gs_refresh gs_local -traversal reverse_topo}
  gs_local_crit     {-sweep_engine gauss_seidel_topo -gs_refresh gs_local -traversal criticality_sorted}
  gs_incr_fwd       {-sweep_engine gauss_seidel_topo -gs_refresh gs_incremental -traversal forward_topo}
  gs_incr_rev       {-sweep_engine gauss_seidel_topo -gs_refresh gs_incremental -traversal reverse_topo}
  gs_incr_crit      {-sweep_engine gauss_seidel_topo -gs_refresh gs_incremental -traversal criticality_sorted}
} {
  puts "=== sweep_engine $label ==="
  reset_global_sizing_config
  eval set_global_sizing_config $args
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}

# Estimation-loop-under-GS: the Reimann seed's dry-run sweeps are journal-based
# and engine-agnostic, so they must work with the Gauss-Seidel engine. The
# reimann_partial preset now flips to GS, and the explicit combo below pins it.
foreach {label args} {
  reimann_preset_gs  {-preset reimann_partial}
  est_loop_gs_local  {-lambda_seed estimation_loop -sweep_engine gauss_seidel_topo -gs_refresh gs_local -traversal forward_topo}
  est_loop_gs_incr   {-lambda_seed estimation_loop -sweep_engine gauss_seidel_topo -gs_refresh gs_incremental -traversal forward_topo}
} {
  puts "=== estimation_loop $label ==="
  reset_global_sizing_config
  eval set_global_sizing_config $args
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}

# A paper preset that now runs on the Gauss-Seidel engine.
puts "=== preset chen_partial (gauss_seidel_topo) ==="
reset_global_sizing_config
set_global_sizing_config -preset chen_partial
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# GS determinism artifact: the sorted cell-assignment signature after a GS run.
# Pinned by the golden, so a nondeterministic result would fail the test.
puts "=== gs cell-assignment signature ==="
reset_global_sizing_config
set_global_sizing_config -sweep_engine gauss_seidel_topo -gs_refresh gs_local \
  -traversal forward_topo
repair_timing -setup -phases GLOBAL_SIZING
set sig {}
foreach inst [get_cells *] {
  lappend sig "[get_property $inst full_name]=[get_property $inst ref_name]"
}
foreach line [lsort $sig] {
  puts "SIG $line"
}
