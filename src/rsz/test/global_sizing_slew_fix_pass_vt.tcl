# -slew_fix_pass only tries cells with the gate's own Vt flavor. On gcd (asap7,
# with the RVT, LVT and SLVT libraries), set_load puts 60 fF on output port
# resp_msg[1], driven by output44 (BUFx2_ASAP7_75t_L), whose slew there is
# 162 ps, over a 120 ps max_transition. Every other LVT buffer is dont_use, and
# set_opt_config widens the resizer's sizing window so that output44's
# swappable group also holds BUFx24_ASAP7_75t_R and the SLVT buffers
# (report_equiv_cells), which rank above it by leakage and are fast enough. The
# pass therefore has no cell of output44's flavor to upsize it to and leaves it
# (RSZ-0461: "1 left as they were"). Without the Vt filter it would take
# BUFx24_ASAP7_75t_R, the first of those cells by leakage.
#
# Single-threaded for a deterministic golden.
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
set_load 60 [get_ports {resp_msg[1]}]
set_max_transition 120 [current_design]
set_opt_config -limit_sizing_leakage 100 -limit_sizing_area 100
set other_lvt_buffers {}
foreach cell [get_lib_cells BUFx*_ASAP7_75t_L] {
  if { [get_name $cell] ne "BUFx2_ASAP7_75t_L" } {
    lappend other_lvt_buffers $cell
  }
}
set_dont_use $other_lvt_buffers
report_equiv_cells [get_lib_cells BUFx2_ASAP7_75t_L]

set_global_sizing_config -preset rsz_baseline -slew_fix_pass 1 \
  -best_tracker none -max_iterations 1
repair_timing -setup -phases GLOBAL_SIZING
