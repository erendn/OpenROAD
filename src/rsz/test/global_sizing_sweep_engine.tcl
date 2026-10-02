# Smoke test for the sweep engines: jacobi_snapshot, and gauss_seidel_topo with
# each traversal (forward_topo, reverse_topo, criticality_sorted) and refresh
# mode (gs_local, gs_incremental), plus the estimation loop and a paper preset
# under Gauss-Seidel. Each run must complete and print the RSZ-0417/0400/0409
# records; the traversal order is unit-tested by TestSweepEngine.
#
# The Gauss-Seidel engine is single-threaded and orders gates by a total
# (key, instance id) sort, so the cell assignment printed at the end is
# deterministic and is checked by the golden.
#
# The runs share one netlist: the first optimizes it and later runs mostly make
# no moves, but each still selects its engine (RSZ-0417). Single-threaded for a
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

# Both engines, and each Gauss-Seidel traversal and refresh mode.
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

# The estimation-loop seed's dry-run sweeps are rolled back through the journal,
# independent of the engine, so they must also work under Gauss-Seidel.
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

# A paper preset that uses the Gauss-Seidel engine.
puts "=== preset chen_partial (gauss_seidel_topo) ==="
reset_global_sizing_config
set_global_sizing_config -preset chen_partial
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# Print the sorted cell assignment after a Gauss-Seidel run; a nondeterministic
# result fails the golden.
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
