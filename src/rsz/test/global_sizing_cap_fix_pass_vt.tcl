# -cap_fix_pass only tries cells with the gate's own Vt flavor, as
# Livramento et al. (DATE 2013, Alg. 3) change only the width when they fix a
# violation. On gcd (asap7, with the RVT, LVT and SLVT libraries), set_load
# puts 3000 fF on output port resp_msg[1], driven by output44
# (BUFx2_ASAP7_75t_L). No buffer drives that within its max_capacitance; the
# largest, BUFx24, declares 1474.56 fF in every Vt flavor. The run keeps the
# input cells (rsz_baseline starts as given), and -timing_bias 1e-6 makes the
# cost practically the cell's leakage. So among the cells closest to the limit
# the RVT one, with the lowest leakage, would win if the flavor were free. The
# pass keeps output44 in LVT and takes it to BUFx24_ASAP7_75t_L; the sweep
# cannot move it, since every candidate exceeds its limit.
#
# -best_tracker none keeps the last iterate. Single-threaded for a
# deterministic golden.
source "helpers.tcl"
source asap7/asap7.vars
read_liberty asap7/asap7sc7p5t_AO_RVT_FF_nldm_211120.lib.gz
read_liberty asap7/asap7sc7p5t_INVBUF_RVT_FF_nldm_220122.lib.gz
read_liberty asap7/asap7sc7p5t_OA_RVT_FF_nldm_211120.lib.gz
read_liberty asap7/asap7sc7p5t_SIMPLE_RVT_FF_nldm_211120.lib.gz
read_liberty asap7/asap7sc7p5t_SEQ_RVT_FF_nldm_220123.lib
read_liberty asap7/asap7sc7p5t_AO_LVT_FF_nldm_211120.lib.gz
read_liberty asap7/asap7sc7p5t_INVBUF_LVT_FF_nldm_220122.lib.gz
read_liberty asap7/asap7sc7p5t_OA_LVT_FF_nldm_211120.lib.gz
read_liberty asap7/asap7sc7p5t_SIMPLE_LVT_FF_nldm_211120.lib.gz
read_liberty asap7/asap7sc7p5t_SEQ_LVT_FF_nldm_220123.lib
read_liberty asap7/asap7sc7p5t_AO_SLVT_FF_nldm_211120.lib.gz
read_liberty asap7/asap7sc7p5t_INVBUF_SLVT_FF_nldm_220122.lib.gz
read_liberty asap7/asap7sc7p5t_OA_SLVT_FF_nldm_211120.lib.gz
read_liberty asap7/asap7sc7p5t_SIMPLE_SLVT_FF_nldm_211120.lib.gz
read_liberty asap7/asap7sc7p5t_SEQ_SLVT_FF_nldm_220123.lib
read_lef asap7/asap7_tech_1x_201209.lef
read_lef asap7/asap7sc7p5t_28_R_1x_220121a.lef
read_lef asap7/asap7sc7p5t_28_L_1x_220121a.lef
read_lef asap7/asap7sc7p5t_28_SL_1x_220121a.lef

read_def gcd_asap7_placed.def

read_sdc gcd.sdc
source asap7/setRC.tcl
estimate_parasitics -placement

set_thread_count 1
set_load 3000 [get_ports {resp_msg[1]}]

set_global_sizing_config -preset rsz_baseline -cap_fix_pass 1 \
  -best_tracker none -timing_bias 1e-6 -max_iterations 8
repair_timing -setup -phases GLOBAL_SIZING

set ref [get_property [get_cells output44] ref_name]
puts "output44: $ref"
if { $ref ne "BUFx24_ASAP7_75t_L" } {
  error "output44 is $ref, expected BUFx24_ASAP7_75t_L"
}
