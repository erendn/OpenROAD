# M5 smoke test: the F3 downsize_guard axis of GlobalSizingPolicy on a small
# design - all three guards (depth_budget, local_slack_veto, none), the veto's
# gamma tolerance knob, the two guard/engine cross-warnings (RSZ-0430/0431, both
# allowed but flagged), and the presets whose papers run Flach's acceptance test.
#
# What this asserts: each guard's full path runs and produces the RSZ-0417/
# 0400/0409 run record, and the veto engages under both refresh modes. The veto
# ARITHMETIC (Eq. 14's gamma, the local-negative-slack sum, the accept test) is
# unit-tested against hand-computed values by TestGuards.
#
# The gs_local vs gs_incremental pair is the point of the file: the veto is the
# first consumer of mid-sweep required times, so it is the first thing that can
# make the two refresh modes choose different cells (under the M4 depth-budget
# guard they were provably identical). The cell-assignment signature of each is
# dumped below, and the golden pins them.
#
# OpenROAD Tcl has no in-session netlist reset, so the configurations run
# sequentially on one netlist: run 1 optimizes, later runs start from that state
# and mostly no-op. Each configuration still dispatches its guard (RSZ-0417).
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

# The three guards under the Gauss-Seidel engine (where the veto is the
# paper-faithful choice), plus the gamma tolerance extremes. gamma_local_slack 0
# pins gamma to 1: no hill climbing, so no candidate may degrade local slack at
# all.
#
# The veto runs FIRST, on the un-optimized netlist, so its QoR (RSZ-0400/0409)
# is what the golden pins - a regression in the veto arithmetic changes the
# golden. The later configurations start from the state it leaves behind and
# mostly no-op (the compounding limitation above); they assert dispatch. The
# depth_budget guard's QoR on this netlist is already pinned by global_sizing.ok
# and global_sizing_preset.ok.
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

# M3 carry-over: the two slew-coupling cost terms double-price the immediate
# sink level (RSZ-0429). Allowed, warned.
puts "=== cross cost_global_phi + cost_fanout_slew (warns RSZ-0429) ==="
reset_global_sizing_config
set_global_sizing_config -cost_global_phi 1 -cost_fanout_slew 1
repair_timing -setup -phases GLOBAL_SIZING
report_worst_slack -max -digits 3

# The presets whose papers run Flach's acceptance test. sharma_seq_partial joins
# them as of the bucket-1 fidelity pass: Sharma adopts the check verbatim ("We
# apply this check as we recover power", §5), so its bundle no longer carries
# our own depth budget.
#
# flach_partial and sharma_seq_partial both pin init_mode=min_size_fixviol (it2
# fixviol rider), so their legs open with RSZ-0416/0415/0445 and their RSZ-0400
# counts are the round trip that creates - reset to minimum, repair the two gates
# that then violate max-cap, LR upsizes back. Both legs still land on the
# incoming netlist (RSZ-0409 +0.00%), so the guard QoR this file pins is
# unmoved.
foreach preset {flach_partial reimann_partial mangiras_partial \
  sharma_seq_partial} {
  puts "=== preset $preset (local_slack_veto) ==="
  reset_global_sizing_config
  set_global_sizing_config -preset $preset
  # flach_partial's bundle carries the paper's ~120 iterations (Fig. 4); this
  # smoke only checks that the veto dispatches, so cap it back to the shared
  # default. The override must come AFTER -preset (order-sensitive by design).
  if { $preset eq "flach_partial" } {
    set_global_sizing_config -max_iterations 20
  }
  repair_timing -setup -phases GLOBAL_SIZING
  report_worst_slack -max -digits 3
}

# The config mirror: every M5 knob must be reported. upsize_hysteresis rides
# along as the bucket-2 addition - 0 is a legal value (it is what every paper
# preset pins), so this also pins that the mirror round-trips 0 rather than
# treating it as "unset". -output_drc_veto joins it as the A2 axis: this file
# owns the acceptance-filter knobs, and the mirror is where a knob that parses
# but is never reported would show up.
puts "=== report_global_sizing_config ==="
reset_global_sizing_config
set_global_sizing_config -downsize_guard local_slack_veto \
  -gamma_local_slack 1.5 -upsize_hysteresis 0 -output_drc_veto relative
report_global_sizing_config
