# Gate Resizer

Gate Resizer commands are described below.  The `resizer` commands stop when
the design area is `-max_utilization util` percent of the core area. `util`
is between 0 and 100.  The `resizer` stops and reports an error if the max
utilization is exceeded.

## Commands

```{note}
- Parameters in square brackets `[-param param]` are optional.
- Parameters without square brackets `-param2 param2` are required.
```

### Set Don't Use

The `set_dont_use` command removes library cells from consideration by
the `resizer` engine and the `CTS` engine. `lib_cells` is a list of cells returned by `get_lib_cells`
or a list of cell names (`wildcards` allowed). For example, `DLY*` says do
not use cells with names that begin with `DLY` in all libraries.

```tcl
set_dont_use lib_cells 
```

### Unset Don't Use

The `unset_dont_use` command reverses the `set_dont_use` command.

```tcl
unset_dont_use lib_cells
```

### Reset Don't Use

The `reset_dont_use` restores the default dont use list.

```tcl
reset_dont_use
```

### Report Don't Use

The `report_dont_use` reports all the cells that are marked as dont use.

```tcl
report_dont_use
```

### Set Don't Touch

The `set_dont_touch` command prevents the resizer commands from
modifying instances or nets.

```tcl
set_dont_touch instances_nets 
```

### Unset Don't Touch

The `unset_dont_touch` command reverse the `set_dont_touch` command.

```tcl
unset_dont_touch instances_nets
```

### Report Don't Touch

The `report_dont_touch` reports all the instances and nets that are marked as dont touch.

```tcl
report_dont_touch
```

### Buffer Ports

The `buffer_ports -inputs` command adds a buffer between the input and its
loads.  The `buffer_ports -outputs` adds a buffer between the port driver
and the output port. Inserting buffers on input and output ports makes
the block input capacitances and output drives independent of the block
internals. It uses the buffer library cell defined by `-buffer_cell` if it is given.

```tcl
buffer_ports 
    [-inputs] 
    [-outputs] 
    [-max_utilization util]
    [-buffer_cell buf_cell]
    [-verbose]
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-inputs`, `-outputs` | Insert a buffer between the input and load, output and load respectively. The default behavior is `-inputs` and `-outputs` set if neither is specified. |
| `-max_utilization` | Defines the percentage of core area used. |
| `-buffer_cell`     | Specifies the buffer cell type to be used. |
| `-verbose`         | Enable verbose logging. |

#### Instance Name Prefixes

`buffer_ports` uses the following prefixes for the buffer instances that it inserts:

| Instance Prefix | Purpose |
| ----- | ----- |
| input | Buffering primary inputs |
| output | Buffering primary outputs |

### Remove Buffers

Use the `remove_buffers` command to remove buffers inserted by synthesis. This
step is recommended before using `repair_design` so that there is more flexibility
in buffering nets.  If buffer instances are specified, only specified buffer instances
will be removed regardless of dont-touch or fixed cell.  Direct input port to output port
feedthrough buffers will not be removed.
If no buffer instances are specified, all buffers will be removed except those that are associated with
dont-touch, fixed cell or direct input port to output port feedthrough buffering.

```tcl
remove_buffers
    [ instances ]
```

### Balance Row Usage

Command description pending.

```tcl
balance_row_usage
```

### Repair Design

The `repair_design` command inserts buffers on nets to repair max slew, max
capacitance and max fanout violations, and on long wires to reduce RC delay in
the wire. It also resizes gates to normalize slews.  Use `estimate_parasitics
-placement` before `repair_design` to estimate parasitics considered
during repair. Placement-based parasitics cannot accurately predict
routed parasitics, so a margin can be used to "over-repair" the design
to compensate.

```tcl
repair_design 
    [-max_wire_length max_length]
    [-slew_margin slew_margin]
    [-cap_margin cap_margin]
    [-max_utilization util]
    [-pre_placement]
    [-buffer_gain float_value] (deprecated)
    [-match_cell_footprint]
    [-reroute]
    [-verbose]
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-max_wire_length` | Maximum length of wires (in microns), defaults to a value that minimizes the wire delay for the wire RC values specified by `set_wire_rc`. |
| `-slew_margin` | Add a slew margin. The default value is `0`, the allowed values are integers `[0, 100]`. |
| `-cap_margin` | Add a capactitance margin. The default value is `0`, the allowed values are integers `[0, 100]`. |
| `-max_utilization` | Defines the percentage of core area used. |
| `-pre_placement` | Enables performing an initial pre-placement sizing and buffering round. |
| `-buffer_gain` | Deprecated alias for `-pre_placement`. The passed value is ignored. |
| `-match_cell_footprint` | Obey the Liberty cell footprint when swapping gates. |
| `-reroute` | Enable resistance-aware wire rerouting to fix slew violations post-GRT (experimental). |
| `-verbose` | Enable verbose logging on progress of the repair. |

#### Instance Name Prefixes

`repair_design` uses the following prefixes for the buffer instances that it inserts:

| Instance Prefix | Purpose |
| ----- | ----- |
| fanout | Fixing max fanout |
| gain | Gain based buffering |
| load_slew | Fixing max transition violations |
| max_cap | Fixing max capacitance |
| max_length | Fixing max length |
| wire | Repairs load slew, length, and max capacitance violations in net wire segment |

### Repair Tie Fanout

The `repair_tie_fanout` command connects each tie high/low load to a copy
of the tie high/low cell.

```tcl
repair_tie_fanout 
    [-separation dist]
    [-max_fanout fanout]
    [-verbose]
    lib_port
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-separation` | Tie high/low insts are separated from the load by this value (Liberty units, usually microns). |
| `-verbose` | Enable verbose logging of repair progress. |
| `lib_port` | Tie high/low port, which can be a library/cell/port name or object returned by `get_lib_pins`. |

### Repair Timing

The `repair_timing` command repairs setup and hold violations.  It
should be run after clock tree synthesis with propagated clocks.
Setup repair is done before hold repair so that hold repair does not
cause setup checks to fail.

The worst setup path is always repaired.  Next, violating paths to
endpoints are repaired to reduced the total negative slack.

```tcl
repair_timing 
    [-setup]
    [-hold]
    [-recover_power percent_of_paths_with_slack]
    [-setup_margin setup_margin]
    [-hold_margin hold_margin]
    [-slack_margin slack_margin]
    [-libraries libs]
    [-allow_setup_violations]
    [-sequence]
    [-phases]
    [-skip_pin_swap]
    [-skip_gate_cloning]
    [-skip_size_down_fanout]
    [-skip_buffering]
    [-skip_buffer_removal]
    [-skip_last_gasp]
    [-skip_vt_swap]
    [-skip_crit_vt_swap]
    [-repair_tns tns_end_percent]
    [-max_passes passes]
    [-max_iterations iterations]
    [-max_repairs_per_pass max_repairs_per_pass]
    [-max_utilization util]
    [-max_buffer_percent buffer_percent]
    [-match_cell_footprint]
    [-verbose]
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-setup` | Repair setup timing. |
| `-hold` | Repair hold timing. |
| `-recover_power` | Set the percentage of paths to recover power for. The default value is `0`, and the allowed values are floats `(0, 100]`. |
| `-setup_margin` | Add additional setup slack margin. |
| `-hold_margin` | Add additional hold slack margin. |
| `-allow_setup_violations` | While repairing hold violations, buffers are not inserted that will cause setup violations unless `-allow_setup_violations` is specified. |
| `-sequence` | Specify a particular order of setup timing optimization moves. The default is "unbuffer vt_swap sizeup swap buffer clone split". Obeys skip flags also. |
| `-phases` | Specify a particular order of setup timing optimization phases. The default is "LEGACY LAST_GASP CRIT_VT_SWAP". |
| `-skip_pin_swap` | Flag to skip pin swap. The default is to perform pin swap transform during setup fixing. |
| `-skip_gate_cloning` | Flag to skip gate cloning. The default is to perform gate cloning transform during setup fixing. |
| `-skip_size_down_fanout` | Flag to skip fanout gate down sizing. The default is to perform non-critical fanout gate down sizing transform during setup fixing. |
| `-skip_buffering` | Flag to skip rebuffering and load splitting. The default is to perform rebuffering and load splitting transforms during setup fixing. |
| `-skip_buffer_removal` | Flag to skip buffer removal.  The default is to perform buffer removal transform during setup fixing. |
| `-skip_last_gasp` | Flag to skip final ("last gasp") optimizations.  The default is to perform greedy sizing at the end of optimization. |
| `-skip_vt_swap` | Flag to skip threshold voltage (VT) swap optimizations.  The default is to perform VT swap optimization to improve timing QoR. |
| `-skip_crit_vt_swap` | Flag to skip critical threshold voltage (VT) swap optimizations at the end of optimization.  The default is to perform critical VT swap optimization to improve timing QoR beyond repairing just the worst path per each violating endpoint. |
| `-repair_tns` | Percentage of violating endpoints to repair (0-100). When `tns_end_percent` is zero, only the worst endpoint is repaired. When `tns_end_percent` is 100 (default), all violating endpoints are repaired. |
| `-max_repairs_per_pass` | Maximum repairs per pass, default is 1. On the worst paths, the maximum number of repairs is attempted. It gradually decreases until the final violations which only get 1 repair per pass. |
| `-max_utilization` | Defines the percentage of core area used. |
| `-max_iterations` | Defines the maximum number of iterations executed when repairing setup and hold violations. The default is `-1`, which disables the limit of iterations. |
| `-max_buffer_percent` | Specify a maximum number of buffers to insert to repair hold violations as a percentage of the number of instances in the design. The default value is `20`, and the allowed values are integers `[0, 100]`. |
| `-match_cell_footprint` | Obey the Liberty cell footprint when swapping gates. |
| `-verbose` | Enable verbose logging of the repair progress. |

Use`-recover_power` to specify the percent of paths with positive slack which
will be considered for gate resizing to save power. It is recommended that
this option be used with global routing based parasitics.

#### Instance Name Prefixes

`repair_timing` uses the following prefixes for the buffer and gate instances that it inserts:

| Instance Prefix | Purpose |
| ----- | ----- |
| clone | Gate cloning |
| hold | Hold fixing |
| rebuffer | Buffering for setup fixing |
| split | Split off non-critical loads behind a buffer to reduce load |

### Repair Clock Nets

The `clock_tree_synthesis` command inserts a clock tree in the design
but may leave a long wire from the clock input pin to the clock tree
root buffer.

The `repair_clock_nets` command inserts buffers in the
wire from the clock input pin to the clock root buffer.

```tcl
repair_clock_nets 
    [-max_wire_length max_wire_length]
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-max_wire_length` | Maximum length of wires (in microns), defaults to a value that minimizes the wire delay for the wire RC values specified by `set_wire_rc`. |

### Repair Clock Inverters

The repair_clock_inverters command replaces an inverter in the clock
tree with multiple fanouts with one inverter per fanout.  This
prevents the inverter from splitting up the clock tree seen by CTS.
It should be run before clock_tree_synthesis.

```tcl
repair_clock_inverters
```

### Report Design Area

The `report_design_area` command reports the area of the design's components
and the utilization.

```tcl
report_design_area
```

### Report Floating Nets

The `report_floating_nets` command reports nets with connected loads but no connected drivers.

```tcl
report_floating_nets 
    [-verbose]
```

### Report Overdriven Nets

The `report_overdriven_nets` command reports nets with connected by multiple drivers.

```tcl
report_overdriven_nets
    [-include_parallel_driven]
    [-verbose]
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-include_parallel_driven` | Include nets that are driven by multiple parallel drivers. |
| `-verbose` | Print the net names. |

### Eliminate Dead Logic

The `eliminate_dead_logic` command eliminates dead logic, i.e. it removes standard cell instances which can be removed without affecting the function of the design.

```tcl
eliminate_dead_logic
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-verbose` | Print the net names. |

## Useful Developer Commands

If you are a developer, you might find these useful. More details can be found in the [source file](./src/Resizer.cc) or the [swig file](./src/Resizer.i).

| Command Name | Description |
| ----- | ----- |
| `repair_setup_pin` | Repair setup pin violation. |
| `parse_time_margin_arg` | Get the raw value for timing margin (e.g. `slack_margin`, `setup_margin`, `hold_margin`) |
| `parse_percent_margin_arg` | Get the above margin in perentage format. |
| `parse_margin_arg` | Same as `parse_percent_margin_arg`. |
| `parse_max_util` | Check maximum utilization. |
| `parse_max_wire_length` | Get maximum wirelength. |
| `check_max_wire_length` | Check if wirelength is allowed by rsz for minimum delay. |

### Setting Optimization Configuration

The `set_opt_config` command configures optimization settings that apply to
data cell selection, affecting all optimization commands like repair_design and repair_timing.
However, this does not apply to clock cell selection in clock_tree_synthesis or repair_clock_nets.

```tcl
set_opt_config 
    [-limit_sizing_area float_value]
    [-limit_sizing_leakage float_value]
    [-keep_sizing_site boolean_value]
    [-keep_sizing_vt boolean_value]
    [-set_early_sizing_cap_ratio float_value]
    [-set_early_buffer_sizing_cap_ratio float_value]
    [-disable_buffer_pruning boolean_value]
    [-sizing_area_limit float_value] (deprecated)
    [-sizing_leakage_limit float_value] (deprecated)
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-limit_sizing_area` | Exclude cells from sizing if their area exceeds <float_value> times the current cell's area. For example, with 2.0, only cells with an area <= 2X the current cell's area are considered. The area is determined from LEF, not Liberty. |
| `-limit_sizing_leakage` | Exclude cells from sizing if their leakage power exceeds <float_value> times the current cell's leakage. For example, with 2.0, only cells with leakage <= 2X the current cell's leakage are considered. Leakage power is based on the current timing corner. |
| `-keep_sizing_site` | Ensure cells retain their original site type during sizing. This prevents short cells from being replaced by tall cells (or vice versa) in mixed-row designs. |
| `-keep_sizing_vt` | Preserve the cell's VT type during sizing, preventing swaps between HVT and LVT cells. This works only if VT layers are defined in the LEF obstruction section. |
| `-set_early_sizing_cap_ratio` | Maintain the specified ratio between input pin capacitance and output pin load when performing initial sizing of gates. |
| `-set_early_buffer_sizing_cap_ratio` | Maintain the specified ratio between input pin capacitance and output pin load when performing initial sizing of buffers. |
| `-disable_buffer_pruning` | Disable buffer pruning to improve hold fixing by not filtering out delay cells or slow buffers. |
| `-sizing_area_limit` | Deprecated.   Use -limit_sizing_area instead. |
| `-sizing_leakage_limit` | Deprecated.  Use -limit_sizing_leakage instead. |

### Reporting Optimization Configuration

The `report_opt_config` command reports current optimization configuration

```tcl
report_opt_config 
```

### Resetting Optimization Configuration

The `reset_opt_config` command resets optimization settings applied from `set_opt_config` command.
If no options are specified, all optimization configurations are reset.

```tcl
reset_opt_config 
    [-limit_sizing_area]
    [-limit_sizing_leakage]
    [-keep_sizing_site]
    [-keep_sizing_vt]
    [-set_early_sizing_cap_ratio]
    [-set_early_buffer_sizing_cap_ratio]
    [-disable_buffer_pruning]
    [-sizing_area_limit] (deprecated)
    [-sizing_leakage_limit] (deprecated)
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-limit_sizing_area` | Remove area restriction during sizing. |
| `-limit_sizing_leakage` | Remove leakage power restriction during sizing. |
| `-keep_sizing_site` | Remove site restriction during sizing. |
| `-keep_sizing_vt` | Remove VT type restriction during sizing. |
| `-set_early_sizing_cap_ratio` | Remove capacitance ratio setting for early sizing. |
| `-set_early_buffer_sizing_cap_ratio` | Remove capacitance ratio setting for early buffer sizing. |
| `-disable_buffer_pruning` | Restore buffer pruning for optimization. |
| `-sizing_area_limit` | Deprecated.  Use -limit_sizing_area instead. |
| `-sizing_leakage_limit` | Deprecated.  Use -limit_sizing_leakage instead. |

### Setting Global Sizing Configuration

The `set_global_sizing_config` command configures the Lagrangian-Relaxation
global sizing driver. These options only affect
`repair_timing -phases GLOBAL_SIZING`; they have no effect on other phases.
Values persist on the block as dbProperties.

Typical use sets only `-init_mode` and `-include_clock_network`; the
remaining options are LR-algorithm tuning hyperparameters that most users
should leave at their defaults.

```tcl
set_global_sizing_config
    [-preset name]
    [-init_mode mode]
    [-init_seed int_value]
    [-lambda_update rule]
    [-move_set set]
    [-fast_olr_start_iter int_value]
    [-output_drc_veto mode]
    [-termination rule]
    [-best_tracker rule]
    [-include_clock_network boolean_value]
    [-size_registers boolean_value]
    [-power_objective objective]
    [-power_phase_filter boolean_value]
    [-timing_cost cost]
    [-setup_slack_margin float_value]
    [-max_iterations int_value]
    [-max_inner_sweeps int_value]
    [-restart_each_iteration boolean_value]
    [-cap_fix_pass boolean_value]
    [-slew_fix_pass boolean_value]
    [-relax_max_cap boolean_value]
    [-beta float_value]
    [-mu_exponent float_value]
    [-lambda_floor float_value]
    [-timing_bias float_value]
    [-budget_safety_factor float_value]
    [-upsize_hysteresis float_value]
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-preset` | Named configuration that sets all algorithm options at once. `rsz_baseline` is OpenROAD's own configuration. `chen_partial`, `tennakoon_partial`, `flach_partial`, `sharma_seq_partial`, `reimann_partial`, `mangiras_partial`, `livramento_partial` and `chinnery_partial` follow published LR gate sizers; the `_partial` suffix means that the preset does not implement every component of the published method. Options given individually, in the same or a later call, override the preset. Without a preset, every option takes its own default. |
| `-init_mode` | Initial solution applied before the LR loop: every editable instance is replaced by another member of its equivalent-cell group. The group is ranked by cell leakage (drive resistance breaks ties), so `min_size`, `max_size` and `average` refer to positions in that ranking. One of `as_given` (default; keep the incoming netlist), `min_size` (lowest-ranked member), `max_size` (highest-ranked member), `min_size_fixviol` (`min_size`, followed by an output-to-input repair pass that upsizes each gate violating max capacitance or max slew on its own outputs to the lowest-ranked member that clears the violation; a gate that no member can fix stays at minimum and is reported by RSZ-0445), `min_size_fixcap` (the same, but the repair checks only max capacitance; Sharma et al. (ICCAD 2015) repair slew in a separate pass, `-slew_fix_pass`), `random` (uniform draw per instance, seeded by `-init_seed`), or `average` (lower median of the ranking). `chen_partial` and `livramento_partial` use `min_size`, `flach_partial` uses `min_size_fixviol`, and `sharma_seq_partial` uses `min_size_fixcap`. |
| `-init_seed` | Seed for `-init_mode random`. A non-negative integer; default 0. Ignored, with a warning, by the other modes. The draw for each instance depends only on the seed and the instance path name, so a seed gives the same initial netlist regardless of instance order or thread count. It is independent of the global placement seed. |
| `-lambda_update` | Rule that updates the Lagrange multiplier of each timing arc after every LR iteration. `norm_subgradient` (default) is OpenROAD's own normalized subgradient step. The other rules follow published sizers: `flach_slack_scaling` (Flach et al., TCAD 2014), `chen_subgradient` (Chen et al., ICCAD 1998), `tennakoon_ratio` (Tennakoon and Sechen, ICCAD 2002), `sharma_cexp` (Sharma et al., ICCAD 2015), `reimann_dwns` (Reimann et al., ISPD 2016), `livramento_ratio` (Livramento et al., DATE 2013) and `sharma_arc_slack` (Sharma et al., TCAD 2020). `sharma_arc_slack` multiplies each arc's multiplier by (1 − s/T)^K, where s is the arc's own slack and T the clock period. With `-termination threshold_battery`, K is 4 on arcs with negative slack and 1 on the others during the timing phase, and 1 and 4 during the power phase, so the power phase quickly lowers the multipliers of arcs with positive slack. The update that ends the last timing-phase iteration still uses the timing-phase values. Under any other termination the run never leaves the timing phase, which is warned about. `chinnery_partial` uses `sharma_arc_slack`. |
| `-move_set` | Which members of a gate's equivalent-cell group the per-gate LR subproblem evaluates. `full_library` (default) evaluates every member in every iteration. `sharma_fast_olr` (Sharma et al., ICCAD 2015, Fig. 9) evaluates every member until `-fast_olr_start_iter`, then switches to a local search: a hill descent through the sizes of the gate's current threshold-voltage flavor and its two neighboring flavors, stopping in each direction at the first step that does not improve the cost. `mangiras_size_step` (Mangiras and Dimitrakopoulos, Technologies 2021, Sec. 4.3) allows only a one-step size change up or down in every iteration, while threshold-voltage swaps stay unrestricted. Within a flavor, sizes are ranked by cell leakage, as for `-init_mode`. The restricted sets reduce runtime and limit how far a gate moves in one iteration, but they can miss a cell that the full scan would find. `sharma_seq_partial` and `mangiras_partial` use their matching restricted set; the other presets use `full_library`. |
| `-fast_olr_start_iter` | Iteration at which `-move_set sharma_fast_olr` switches to the local search. Iterations are counted from 1 over the iterations that update the multipliers; 0 also includes the initial sweep before the first update. A non-negative integer; default 5, so the first four iterations evaluate every cell, as in the paper. Ignored, with a warning, by the other move sets. |
| `-output_drc_veto` | How the max capacitance / max slew candidate filter treats an output pin that the gate's current cell already violates. `absolute` (default) rejects every candidate that exceeds its own limits; if no cell in the group can clear the pin, the gate keeps its current cell. `relative` rejects only a candidate that makes the existing violation worse; an equal violation is accepted, and a pin without a violation gets the absolute check. With either setting, a candidate cannot create a new violation at the load seen when its gate is evaluated. That load can still change when neighboring gates are resized in the same sweep: a max capacitance check after each sweep reverts such moves (RSZ-0443), but there is no equivalent check for max slew. With `-relax_max_cap`, the filter and that check leave max capacitance to the cost on the pins that have a multiplier. Neither setting repairs violations the design already has. `absolute` removes some of them, because on a violating pin it accepts only candidates that clear it, while `relative` generally keeps them. Check the result with `report_check_types -max_capacitance -max_slew`. The input side of the filter always uses the relative rule. `flach_partial` and `chinnery_partial` use `relative`, as their papers do. |
| `-termination` | LR stop rule. Every rule also stops at `-max_iterations`. `fixed_iters` (default) also stops after 3 consecutive iterations that worsened the worst slack or 2 consecutive iterations that changed nothing. `stagnation_windows` stops when power (and optionally TNS) stops improving. `threshold_battery` runs a timing phase followed by a power phase, each with its own stop thresholds. `pure_cap` stops only at `-max_iterations`. No rule stops just because timing is met: global sizing minimizes power subject to timing, so it also runs on designs that already meet timing. |
| `-best_tracker` | Which iteration's cells global sizing returns. The iterations do not improve steadily, so most rules remember the best iteration and put its cells back after the last one. `flach_dominance` (default; Flach et al., TCAD 2014) returns the lowest-power iteration whose total negative slack is within 10% of the clock period (`-best_tns_target_frac`). `reimann_score` (Reimann et al., ISPD 2016) scores each iteration by its changes in power, area, total negative slack and worst slack against the netlist global sizing received, and returns the best one, or that netlist if no iteration scores higher. `livramento_feasible` (Livramento et al., DATE 2013) returns the lowest-power iteration that has no setup violation and no max capacitance or max slew violation, as `report_check_types` counts them; if there is none, the final iteration is kept. RSZ-0460 reports how many iterations had no violation and which one was returned. `wns_pass_reject` keeps the cells of the last iteration that did not make the worst slack worse, whose worst slack matched or beat the best so far (starting from the worst slack before the first iteration), and that left the design within its maximum area; the changes after that iteration are undone, and if no iteration qualifies, all changes of the iterations are undone. `none` keeps the final iteration. Power is measured as `-power_objective` sets. `rsz_baseline` sets `wns_pass_reject`; `chen_partial`, `tennakoon_partial` and `chinnery_partial` set `none`; `reimann_partial` sets `reimann_score`; `livramento_partial` sets `livramento_feasible`; the other presets keep `flach_dominance`. |
| `-include_clock_network` | If true, allow global sizing to size clock network instances. Default false (clock instances are excluded). |
| `-size_registers` | If false, global sizing never changes the cell of a register (a flip-flop, latch or other sequential cell), neither in the initial solution set by `-init_mode` nor in the LR iterations. Default true. All paper presets set false, because none of the published sizers resizes registers; `rsz_baseline` keeps the default. |
| `-power_objective` | The power global sizing minimizes. `leakage` (default) is the leakage of each cell. `total` adds each cell's internal power and the switching power its input pins add to the nets that drive them. The power of the cell's own output net is left out, because it does not depend on which cell the gate takes. Internal power is read at the same input slew and load as the cell's delay. Switching activity comes from OpenSTA, as `report_power` uses it: the activity set by the user (for example with `set_power_activity` or `read_vcd`), and elsewhere OpenSTA's estimate propagated from the inputs and clocks. It is read once, before global sizing changes any cell, because resizing a gate to an equivalent cell does not change its logic function. With `total`, the stop rules and best-solution choices that compare the design's power between iterations (`-termination stagnation_windows` and `threshold_battery`, `-best_tracker flach_dominance`, `reimann_score` and `livramento_feasible`) compare total power too. `reimann_partial` and `chinnery_partial` set `total`, because their papers minimize total power. |
| `-power_phase_filter` | If true, during the power phase of `-termination threshold_battery` the LR subproblem skips every candidate cell whose power, as `-power_objective` measures it, is above the current cell's, so a gate can only keep or lower its power (Chinnery and Sharma, ISPD 2022, Table 2). Under any other termination it has no effect, which is warned about. Default false; `chinnery_partial` sets true. |
| `-timing_cost` | How each gate's cost prices the timing arcs that end at one output pin, each weighted by its own Lagrange multiplier. `worst_arc` (default) prices every arc at the delay of the slowest one, which needs one delay lookup per pin but overprices the faster arcs of a gate with several inputs. `per_arc` prices each arc at its own delay, as the published LR sizers do; it needs one delay lookup per arc, so global sizing runs longer. The setting applies wherever the cost prices arc delays: the gate's own arcs, the arcs of the gates that drive its inputs, and the median gate cost that `-timing_bias` is scaled against. When two cells of a group split an arc's `when` conditions differently, `per_arc` prices the arc at the slowest arc between the same two pins. No preset sets `per_arc`. |
| `-setup_slack_margin` | Slack target used inside the LR loop. It enters the per-gate downsize budget (`slack − margin`), the endpoint μ seed and the slack references of the λ updates. Default 0.0. Reaching the target does not stop the loop. |
| `-max_iterations` | Maximum LR outer-loop iterations. Default 20. |
| `-max_inner_sweeps` | Maximum number of sweeps in one LR iteration. After each multiplier update, global sizing repeats the sweep over all gates with the multipliers held fixed, updating timing after each sweep, until a sweep changes no gate or this many sweeps have run. A sweep whose changes were all undone by the max capacitance check after the sweep (RSZ-0443) counts as changing nothing. This follows Chen et al. (ICCAD 1998) and Tennakoon and Sechen (ICCAD 2002), who repeat until nothing improves; the cap is needed because with discrete cells two gates can keep swapping back and forth. The stop rules, the best-solution choices and the worst-slack check still run once per iteration. With a value above 1, RSZ-0455 reports the total number of sweeps and how many iterations stopped at the cap. An integer of at least 1; default 1 (one sweep per iteration). `chen_partial` and `tennakoon_partial` set 10. |
| `-restart_each_iteration` | If true, at the start of every LR iteration after the first, after the multiplier update, global sizing sets every gate back to the cell it had right after the initial solution (`-init_mode`), so every iteration sizes the design from the same starting cells. This follows Chen et al. (ICCAD 1998), who solve every subproblem starting from the minimum sizes. Their continuous solver reaches the same solution from any start, but sweeps over discrete cells do not, so use it with `-max_inner_sweeps` above 1, which lets each iteration rebuild the sizes the restart discards; with `-max_inner_sweeps 1` it is warned about (RSZ-0457). The cells after the last iteration are the result, unless `-best_tracker` picks an earlier iteration. RSZ-0400 counts the cell changes of every iteration, including those a later restart sets back. Default false; `chen_partial` sets true, with `-init_mode min_size` and `-max_inner_sweeps 10`. |
| `-cap_fix_pass` | If true, after every sweep global sizing visits the gates from outputs to inputs and resizes each gate whose output load is above the max capacitance of its cell, as Livramento et al. (DATE 2013, Alg. 3) do after every iteration. It chooses among the cells of the gate's equivalent-cell group that have the gate's threshold-voltage flavor. Of the cells that bring the load within their limit, it takes the one with the lowest LR cost: the cell's power plus the delays of its own timing arcs, weighted by their Lagrange multipliers and priced as `-timing_cost` sets, plus, with `-relax_max_cap`, the max capacitance term of its own output pins. If no cell does, it takes the one that exceeds its limit by the least, which can be the current cell. The cells it tries are those the LR iterations may swap the gate to, which the resizer limits relative to the gate's current cell, so a gate far over its limit can take several sweeps to reach the largest cell. Only max capacitance is repaired, and only against the limit in the Liberty library; a limit set with `set_max_capacitance` alone is not seen. A larger cell adds load to the gates that drive it; the pass visits those gates afterwards. The changes are not counted as sweep moves in RSZ-0400; RSZ-0459 reports how many gates the pass found over their limit and what it did with them. Default false; `livramento_partial` sets true. |
| `-slew_fix_pass` | If true, once before the first LR iteration, after the initial solution set by `-init_mode`, global sizing visits the gates from inputs to outputs and upsizes each gate whose output slew is above its limit, as Sharma et al. (ICCAD 2015, Sec. III-A) do after their capacitance repair. Inputs go first because a gate's slew depends on the slew of the gates that drive it, which the pass has then already repaired. It chooses among the larger cells of the gate's equivalent-cell group that have the gate's threshold-voltage flavor, and skips any cell whose input capacitance would push a gate that drives it over its max capacitance, or add load to a driver already over it. This driver check is the one the LR iterations apply, so it uses the limit OpenSTA checks, which includes a `set_max_capacitance`. Of the cells that bring the slew within the limit, it takes the one with the lowest leakage. If no cell does, it takes the one closest to the limit, or keeps the current cell if none is closer. A cell's slew is estimated from the gate's measured slew, scaled by the cell's drive resistance, as the LR iterations do. The pass does not revisit a gate, so a later upsize in a gate's fanout can raise that gate's slew again. RSZ-0461 reports how many gates were over their slew limit and what the pass did with them. Default false; `sharma_seq_partial` sets true, with `-init_mode min_size_fixcap`. |
| `-relax_max_cap` | If true, global sizing prices max capacitance instead of forbidding it, as Livramento et al. (DATE 2013, Eq. 3) do. Each output pin of a gate that global sizing may resize gets a Lagrange multiplier (β in the paper, unrelated to `-beta`), if its cell has a max capacitance in the Liberty library. Registers get one even under `-size_registers 0`, as the paper prices the primary inputs of its circuit model. Such a register is not resized, so neither the fix pass nor the check after each sweep repairs it: only the cells that load it, priced by its multiplier, can bring it back within its limit, as in the paper, whose fix pass resizes gates only. A candidate cell then adds β × (load − limit) to its cost on its own output pins, at its own limit, and on the output pins of the gates that drive its inputs, at a load that includes its input capacitance. A pin below its limit earns a credit. The term is weighted by the same timing weight as the delays. On a pin that has a multiplier, the LR iterations no longer reject a cell for exceeding a max capacitance, and the max capacitance check after each sweep (RSZ-0443) no longer undoes moves. A pin without a multiplier keeps both checks: a primary input, or the output of another gate that global sizing does not resize, such as a dont-touch cell, a macro, or a clock-network gate without `-include_clock_network`. Max slew keeps its check. Every β starts small: in the first sweep, an excess equal to the median limit costs 1% of the median gate's power. Because a pin below its limit earns a credit, a larger start would make the first sweep upsize gates for that credit alone. After every LR iteration, together with the timing multipliers, each β is multiplied by the pin's load divided by its cell's limit, so it grows while the pin is over its limit and shrinks while it is below it. Only the limits in the Liberty library are priced; a limit set with `set_max_capacitance` alone is not seen. With `-cap_fix_pass`, the fix pass adds the gate's own term to the cost it compares cells by. A pin below its limit can make a gate's cost negative, and `-upsize_hysteresis` then no longer holds back an upsize of that gate, which RSZ-0465 warns about. RSZ-0464 reports the initial β, the number of pins with a multiplier, and how many of them were over their limit at the start and at the end of the run. Default false; `livramento_partial` sets true. |
| `-beta` | Step size α for the dual-subgradient update on λ. Default 0.6. |
| `-mu_exponent` | Endpoint seed exponent for μ. Default 2.0. |
| `-lambda_floor` | Floor on per-edge multipliers so unused arcs can re-enter. Default 1e-12. |
| `-timing_bias` | Dimensionless balance between timing pressure and power cost. Default 64.0. |
| `-budget_safety_factor` | Safety derate (≤ 1) on the per-gate distributed downsize budget. Default 1.0. |
| `-upsize_hysteresis` | Relative LR cost improvement an upsize must exceed to be accepted; downsizes are accepted on any improvement. This filters out moves caused by cost noise that would change the design without a real timing gain. Default 0.02. The paper presets set 0.0 and take the lowest-cost candidate. |

### Reporting Global Sizing Configuration

The `report_global_sizing_config` command reports the current global sizing
configuration. Each value shows the configured override or `undefined` when
the option has not been set (the policy falls back to its built-in default).

```tcl
report_global_sizing_config
```

### Resetting Global Sizing Configuration

The `reset_global_sizing_config` command clears global sizing configuration
that was applied with `set_global_sizing_config`. With no options, all
global sizing configuration is reset. Cleared options revert to the
policy's built-in defaults.

```tcl
reset_global_sizing_config
    [-preset]
    [-init_mode]
    [-init_seed]
    [-lambda_update]
    [-move_set]
    [-fast_olr_start_iter]
    [-output_drc_veto]
    [-termination]
    [-best_tracker]
    [-include_clock_network]
    [-size_registers]
    [-power_objective]
    [-power_phase_filter]
    [-timing_cost]
    [-setup_slack_margin]
    [-max_iterations]
    [-max_inner_sweeps]
    [-restart_each_iteration]
    [-cap_fix_pass]
    [-slew_fix_pass]
    [-relax_max_cap]
    [-beta]
    [-mu_exponent]
    [-lambda_floor]
    [-timing_bias]
    [-budget_safety_factor]
    [-upsize_hysteresis]
```

### Finding Equivalent Cells

The `report_equiv_cells` command finds all functionally equivalent library cells for a given library cell with relative area and leakage power details.

```tcl
report_equiv_cells 
    [-match_cell_footprint]
    [-all]
    [-vt]
    lib_cell
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-match_cell_footprint` | Limit equivalent cell list to include only cells that match library cell_footprint attribute. |
| `-all` | List all equivalent cells, ignoring sizing restrictions and cell_footprint.  Cells excluded due to these restrictions are marked with an asterisk. |
| `-vt`  | List all threshold voltage (VT) equivalent cells such as HVT, RVT, LVT, SLVT. |

### Reporting Buffers

The `report_buffers` command reports all usable buffers to include for optimization.
Usable buffers are standard cell buffers that are not clock buffers, always on buffers,
level shifters, or buffers marked as dont-use.  VT type, cell site,
cell footprint and leakage are also reported.

```tcl
report_buffers
    [-filtered]
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-filtered` | Report buffers after filtering based on threshold voltage, cell footprint, drive strength and cell site.  Subset of filtered buffers are used for rebuffering. |

### Reporting Delay Estimator Accuracy

The `report_delay_estimator_accuracy` command compares a delay
estimator's predictions against the reference STA delay for a candidate
library cell at the given instance.  It is intended as a developer aid
for evaluating estimator accuracy when sizing.

```tcl
report_delay_estimator_accuracy
    -inst instance
    -lib_cell lib_cell
    -estimator estimator
    [-delay_levels level]
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-inst` | Instance whose driver pin is used as the candidate sizing site. Required. |
| `-lib_cell` | Liberty cell to evaluate as a replacement for `-inst`. Must resolve to exactly one cell. Required. |
| `-estimator` | Delay estimator to evaluate. One of `legacy`, `delay_estimator`, `legacy_mt`, `mt`. Required. |
| `-delay_levels` | Number of downstream stages to include in the accuracy comparison: `0`, `1`, or `2`. Default `0`. Not valid for the `legacy` estimator. |

### Optimizing Arithmetic Modules

The `replace_arith_modules` command optimizes design performance by intelligently swapping hierarchical arithmetic modules based on realistic timing models.
This command analyzes critical timing paths and replaces arithmetic modules with equivalent but architecturally different implementations to
improve Quality of Results (QoR) for the specified target.

#### Arithmetic Module Types

Yosys and OpenROAD support the following arithmetic module variants with different timing/area trade-offs.

ALU (Arithmetic Logic Unit) Variants

Han-Carlson (default)
: Balanced delay and area.  Best for general purpose applications.

Kogge-Stone
: Fastest, largest area.  Best for timing-constrained designs.

Brent-Kung
: Slower, smaller area.  Best for area-constrained designs.

Sklansky
: Moderate delay/area.  Best for balanced optimization.

MACC (Multiply-Accumulate) Variants

Booth (default)
: Balanced delay and area.  Best for general purpose applications.

Base (Han-Carlson)
: Fastest, potentially larger area.  Best for timing-constrained designs.

#### Requirements for Arithmetic Module Swap

1. Hierarchical netlist with arithmetic operators.  Yosys can produce such designs by enabling "wrapped operator synthesis".
In OpenROAD-flow-scripts, this can be done as follows:

cd OpenROAD-flow-scripts/flow

make SYNTH_WRAPPED_OPERATORS=1

This requires a Verilog netlist.  DEF netlist alone is not sufficient for hierarchical optimization.

2. Hierarchically linked design.  The design needs to be linked to preserve hierarchical boundaries.  For example,

link_design top -hier

read_db -hier

```tcl
replace_arith_modules 
    [-path_count num_critical_paths]
    [-slack_threshold float]
    [-target setup | hold | power | area]
```

#### Options

| Switch Name | Description |
| ----------- | ---------- |
| `-path_count`           | Number of critical paths to analyze to identify candidate arithmetic modules to swap. The default value is `1000`, and the allowed values are integers. |
| `-slack_threshold`      | Slack threshold in library time units.  Use positive values to include paths with small positive slack. The default value is `0.0`, and the allowed values are floats. |
| `-target`               | Optimization target. Valid types are `setup`, `hold`, `power`, `area`. Default type is `setup`, and the allowed value is string. |

#### Arguments

Setup
ALU: replace all candidate modules with Kogge-Stone (fastest)
MACC: replace all candidate modules with Base (fastest)

Hold
Not available yet

Power
Not available yet

Area
Not available yet

#### SEE ALSO

replace_hier_module

#### EXAMPLES

Arithmetic modules follow this naming convention per Yosys:

ALU_\<io_config\>_\<width\>_\<config\>_\<architecture\>

MACC_\<io_config\>_\<width\>_\<architecture\>

Examples:

ALU_20_0_25_0_25_unused_CO_X_HAN_CARLSON

ALU_20_0_25_0_25_unused_CO_X_KOGGE_STONE

ALU_20_0_25_0_25_unused_CO_X_BRENT_KUNG

ALU_25_0_20_0_25_unused_CO_X_SKLANSKY

\\MACC_14'10001011010100_19_BOOTH

\\MACC_14'10001011010100_19_BASE

## Example scripts

A typical `resizer` command file (after a design and Liberty libraries have
been read) is shown below.

```
read_sdc gcd.sdc

set_wire_rc -layer metal2

set_dont_use {CLKBUF_* AOI211_X1 OAI211_X1}

buffer_ports
repair_design -max_wire_length 100
repair_tie_fanout LOGIC0_X1/Z
repair_tie_fanout LOGIC1_X1/Z
#clock tree synthesis...
repair_timing
```

Note that OpenSTA commands can be used to report timing metrics before
or after resizing the design.

```
set_wire_rc -layer metal2
report_checks
report_tns
report_wns
report_checks

repair_design

report_checks
report_tns
report_wns
```

## Regression tests

There are a set of regression tests in `./test`. For more information, refer to this [section](../../README.md#regression-tests).

Simply run the following script:

```shell
./test/regression
```

## Limitations

## Using the Python interface to rsz

```{warning}
The `Python` interface is currently in development and is subject to change.
```

The `Python` API tries to stay close to the API defined in the `C++` class
`Resizer` that is located [here](./include/rsz/Resizer.hh).

When initializing a design, a sequence of `Python` commands might look like
the following:

```python
from openroad import Design, Tech

tech = Tech()
tech.readLiberty("my_lib.lib")
tech.readLef("my_tech.lef")

design = Design(tech)
design.readDef("my_design.def")

resizer = design.getResizer()
```

Here are some common operations on the `Resizer` object:

```python
resizer.setMaxUtilization(0.8)
resizer.bufferInputs(None, False)
resizer.bufferOutputs(None, False)
resizer.repairDesign(0.0, 0.0, 0.0, 0.0, False, False)
resizer.repairClkNets(0.0)
resizer.repairClkInverters()
resizer.repairHold(0.0, 0.0, False, 0.2, 10000, -1, False, False)
resizer.eliminateDeadLogic(True)
resizer.reportLongWires(10, 2)
resizer.reportDontUse()
resizer.reportDontTouch()
```

There are also some useful `Python` functions located in the
[`rsz_aux.py`](./test/rsz_aux.py) file that provide Pythonic keyword-argument
wrappers with unit conversions (microns → metres, nanoseconds → seconds,
percentages → fractions), but these are not considered part of the *final*
API and may be subject to change.

## License

BSD 3-Clause License. See [LICENSE](../../LICENSE) file.
