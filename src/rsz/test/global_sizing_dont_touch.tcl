# Regression: don't-touch instances must survive EVERY GLOBAL_SIZING mutation
# path untouched.
#
# The phase writes the netlist in five distinct places, each reached by a
# different axis option and each carrying its own eligibility filter. A
# don't-touch instance has to survive all of them:
#
#   1. InitPass init_mode           lr/InitPass.cc      (isEditableLogicStdCell)
#   2. Jacobi snapshot + apply      LRSubproblem::snapshot / applyDecisions
#   3. Gauss-Seidel commit          SweepEngine.cc  GaussSeidelSweep::sweep
#   4. best-tracker capture/restore lr/BestTracker.cc   (SnapshotBestTracker)
#   5. estimation-loop rollback     GlobalSizingPolicy::runEstimationLoop
#
# Filters 1-3 are separate code paths that each re-derive eligibility, so a
# fix in one does not protect the others; 4 and 5 write cells back after the
# sweep filters have already run, so they can resurrect a cell the filters
# excluded. This test pins all five at once.
#
# Structure. Part 1 pins every instance and asserts the whole phase is a no-op
# on the netlist. Part 1 alone is satisfiable vacuously - a phase that no-ops
# for an unrelated reason (no violations, no candidates) would pass it - so
# Part 2 pins only a subset and asserts the pinned cells hold WHILE the rest of
# the design still moves. Part 2 is what makes Part 1 mean something.
#
# The design is the one the LEGACY size-up don't-touch test uses (reg1, 14
# instances, clock tightened to 0.35 ns so setup is violated and the sizer has
# real work to do). Single-threaded for a deterministic golden.
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
# |TNS| < frac*T gate actually opens on this design and the tracker stores and
# restores a solution (the same widening global_sizing_termination.tcl uses) -
# otherwise capture/restore never runs and this leg tests nothing.
reset_global_sizing_config
set_global_sizing_config -best_tracker flach_dominance -best_tns_target_frac 1e6
run_gs "best_tracker flach_dominance (restore path)"
check_pinned_held $all_refs $all_insts "best_tracker restore"

# 5. Estimation-loop seed: dry-run sweeps committed then rolled back through the
# journal.
reset_global_sizing_config
set_global_sizing_config -lambda_seed estimation_loop -est_loop_iters 2
run_gs "estimation_loop seed (journal rollback)"
check_pinned_held $all_refs $all_insts "estimation_loop"

# The strongest form of Part 1: after five configurations covering every
# mutation path, the netlist is bit-for-bit the one we read in.
set part1_changed [changed_instances $all_refs]
if { [llength $part1_changed] != 0 } {
  error "netlist changed with every instance pinned: [join $part1_changed {, }]"
}
puts "Part 1 PASSED: netlist unchanged after all five mutation paths."

########################################################################
# Part 2: pin only the buffers; the rest of the design must still move.
# This is the anti-vacuity guard for Part 1.
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

# The init pass touches every editable instance, so it is the harshest single
# test of a partial pin, and it runs first while the netlist still has headroom.
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
