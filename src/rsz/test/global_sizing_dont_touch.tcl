# Don't-touch instances must not be changed by any GLOBAL_SIZING path that
# writes the netlist. Each path has its own eligibility check, and paths 4 and
# 5 write cells back after the sweep filters have run:
#
#   1. InitPass init_mode           lr/ViolationRepair.cc (maySizeGate)
#   2. Jacobi snapshot + apply      LRSubproblem::snapshot / applyDecisions
#   3. Gauss-Seidel commit          SweepEngine.cc  GaussSeidelSweep::sweep
#   4. best-tracker capture/restore lr/BestTracker.cc   (SnapshotBestTracker)
#   5. estimation-loop rollback     GlobalSizingPolicy::runEstimationLoop
#
# The max-capacitance fix pass (-cap_fix_pass) picks its gates with path 1's
# check; it is not run here.
#
# Part 1 marks every instance don't-touch and checks that nothing changes.
# Part 2 marks only a few and checks that they hold while other instances still
# change, so Part 1 cannot pass just because nothing could move. The design is
# the one used by repair_setup_dont_touch_sizeup.tcl. Single-threaded for a
# deterministic golden.
source "helpers.tcl"

proc instance_refs { } {
  set refs {}
  foreach inst [get_cells *] {
    dict set refs [get_full_name $inst] [get_property $inst ref_name]
  }
  return $refs
}

# Names of the instances whose ref_name differs from the recorded snapshot.
proc changed_instances { before_refs } {
  set changed {}
  foreach inst [get_cells *] {
    set inst_name [get_full_name $inst]
    set after_ref [get_property $inst ref_name]
    if { [dict get $before_refs $inst_name] ne $after_ref } {
      lappend changed "$inst_name:[dict get $before_refs $inst_name]->$after_ref"
    }
  }
  return $changed
}

# Assert that none of $pinned moved. Reports the offending instances so a
# failure names the path that ignored the flag.
proc check_pinned_held { before_refs pinned stage } {
  set violations {}
  foreach change [changed_instances $before_refs] {
    set inst_name [lindex [split $change ":"] 0]
    if { [lsearch -exact $pinned $inst_name] != -1 } {
      lappend violations $change
    }
  }
  if { [llength $violations] != 0 } {
    error "DONT-TOUCH VIOLATED after $stage: [join $violations {, }]"
  }
  puts "$stage: all [llength $pinned] don't-touch instances held."
}

proc run_gs { stage } {
  puts "--- $stage ---"
  repair_timing -setup -phases GLOBAL_SIZING
}

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup_dont_touch_sizeup.def
create_clock -period 0.35 clk
set_load 1.0 [all_outputs]

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1

set all_refs [instance_refs]
set all_insts [dict keys $all_refs]

########################################################################
# Part 1: every instance pinned -> every path must be a netlist no-op.
########################################################################
foreach inst $all_insts {
  set_dont_touch $inst
}
puts "Part 1: pinned [llength $all_insts] instances (all)."

# 1. InitPass init pass. max_size would upsize every editable instance.
reset_global_sizing_config
set_global_sizing_config -init_mode max_size
run_gs "init_mode max_size"
check_pinned_held $all_refs $all_insts "init pass"

# 2. Jacobi sweep: snapshot() filter + applyDecisions commit.
reset_global_sizing_config
set_global_sizing_config -sweep_engine jacobi_snapshot
run_gs "jacobi_snapshot"
check_pinned_held $all_refs $all_insts "jacobi"

# 3. Gauss-Seidel sweep: JIT snapshot + per-gate commit + refreshAfterCommit.
reset_global_sizing_config
set_global_sizing_config -sweep_engine gauss_seidel_topo
run_gs "gauss_seidel_topo"
check_pinned_held $all_refs $all_insts "gauss_seidel"

# 4. Best-tracker capture + restore. best_tns_target_frac is widened so Flach's
# |TNS| < frac*T test passes on this design and the tracker stores and
# restores a solution; otherwise capture/restore never runs.
reset_global_sizing_config
set_global_sizing_config -best_tracker flach_dominance -best_tns_target_frac 1e6
run_gs "best_tracker flach_dominance (restore path)"
check_pinned_held $all_refs $all_insts "best_tracker restore"

# 5. Estimation-loop seed: dry-run sweeps are committed, then rolled back
# through the journal.
reset_global_sizing_config
set_global_sizing_config -lambda_seed estimation_loop -est_loop_iters 2
run_gs "estimation_loop seed (journal rollback)"
check_pinned_held $all_refs $all_insts "estimation_loop"

# After all five configurations the netlist must be exactly the one read in.
set part1_changed [changed_instances $all_refs]
if { [llength $part1_changed] != 0 } {
  error "netlist changed with every instance pinned: [join $part1_changed {, }]"
}
puts "Part 1 PASSED: netlist unchanged after all five mutation paths."

########################################################################
# Part 2: pin only the buffers; the rest of the design must still move.
# This shows that Part 1 did not pass just because nothing could move.
########################################################################
foreach inst $all_insts {
  unset_dont_touch $inst
}
set pinned {u2 u3 u4}
foreach inst $pinned {
  set_dont_touch $inst
}
puts "Part 2: pinned [llength $pinned] of [llength $all_insts] instances ($pinned)."

set before_refs [instance_refs]

# The init pass touches every editable instance, so it is the strictest test of
# a partial pin. It runs first, while the netlist still has room to change.
reset_global_sizing_config
set_global_sizing_config -init_mode max_size
run_gs "partial pin: init_mode max_size"
check_pinned_held $before_refs $pinned "partial init pass"

set changed [changed_instances $before_refs]
if { [llength $changed] == 0 } {
  error "Part 2 is vacuous: nothing moved even though 11 instances were free"
}
puts "Part 2 PASSED: [llength $changed] unpinned instance(s) moved while\
 [llength $pinned] pinned instances held."
