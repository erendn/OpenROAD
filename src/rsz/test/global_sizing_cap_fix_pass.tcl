# -cap_fix_pass resizes, after every sweep, each gate whose output load is
# above the max capacitance of its Liberty cell (Livramento et al., DATE 2013,
# Alg. 3). livramento_partial turns it on.
#
# On gcd (nangate45), set_load puts 3000 fF on output port resp_msg[1], driven
# by _847_ (BUF_X1). livramento_partial's minimum-size start makes _847_ a
# CLKBUF_X1 (max_capacitance 60.73 fF). No buffer in the library drives
# 3000 fF within its limit. While _847_ is a small buffer the sweep cannot move
# it: livramento_partial prices max capacitance instead of filtering on it
# (-relax_max_cap), but at 3000 fF every cell of its group is far over its slew
# limit (6.90 ns against 0.20 ns for CLKBUF_X1), which the sweep still checks.
# The pass takes the cell closest to its limit, which is the largest
# max_capacitance in the gate's swappable group. The group depends on the
# current cell (report_equiv_cells below), so _847_ climbs one group per sweep:
# CLKBUF_X3 (181.885 fF), BUF_X4 (242.31 fF), BUF_X8 (484.009 fF), then BUF_X32
# (1904.3 fF), the largest buffer. In the fifth sweep the pass leaves it there.
# In the sixth, where alpha has grown and the multiplier is still small, the
# sweep trades the priced excess for leakage and takes _847_ down to BUF_X16,
# whose slew is within its limit at that point, and the pass puts BUF_X32
# back: RSZ-0459 reports five resizes and one gate left as it was. With
# -cap_fix_pass 0 it stays a CLKBUF_X1.
#
# _847_ stays over its limit, so no iteration is free of violations, and the
# preset's livramento_feasible tracker keeps the final iteration, which comes
# after the last resize (RSZ-0460). Each run starts from the input netlist.
# Single-threaded for a deterministic golden.
source "helpers.tcl"

proc cell_refs { } {
  set refs {}
  foreach inst [get_cells *] {
    dict set refs [get_full_name $inst] [get_property $inst ref_name]
  }
  return $refs
}

proc restore_input { input_refs } {
  dict for {name ref} $input_refs {
    set inst [get_cells $name]
    if { [get_property $inst ref_name] ne $ref } {
      replace_cell $inst NangateOpenCellLibrary/$ref
    }
  }
  estimate_parasitics -placement
}

proc expect_cell { inst_name expected } {
  set ref [get_property [get_cells $inst_name] ref_name]
  puts "$inst_name: $ref"
  if { $ref ne $expected } {
    error "$inst_name is $ref, expected $expected"
  }
}

read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def gcd_nangate45_placed.def
read_sdc gcd_nangate45.sdc

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set_thread_count 1
set_load 3000 [get_ports {resp_msg[1]}]

set input_refs [cell_refs]

puts "=== the swappable groups _847_ passes through ==="
foreach cell {CLKBUF_X1 CLKBUF_X3 BUF_X4 BUF_X8} {
  report_equiv_cells [get_lib_cells $cell]
}

puts "=== livramento_partial (cap_fix_pass=true) ==="
set_global_sizing_config -preset livramento_partial
set_global_sizing_config -max_iterations 6
repair_timing -setup -phases GLOBAL_SIZING
expect_cell _847_ BUF_X32

puts "=== livramento_partial -cap_fix_pass 0 ==="
restore_input $input_refs
set_global_sizing_config -cap_fix_pass 0
report_global_sizing_config
repair_timing -setup -phases GLOBAL_SIZING
expect_cell _847_ CLKBUF_X1

puts "=== a non-Boolean value is rejected ==="
if { ![catch { set_global_sizing_config -cap_fix_pass maybe }] } {
  error "-cap_fix_pass maybe was accepted"
}

puts "=== reset clears -cap_fix_pass ==="
reset_global_sizing_config -cap_fix_pass
if { [odb::dbBoolProperty_find [rsz::get_block] "gs_cap_fix_pass"] eq "NULL" } {
  puts "gs_cap_fix_pass cleared"
} else {
  error "reset_global_sizing_config -cap_fix_pass left gs_cap_fix_pass set"
}
