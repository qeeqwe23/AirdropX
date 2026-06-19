# AirdropX MATLAB/Simulink Helpers

These files are MATLAB ports of the pure algorithm parts of AirdropX.

- `airdropx_carp_release_point.m`: CARP v2 release point solver.
- `airdropx_pd_controller.m`: PD baseline height controller.
- `airdropx_pd_nw20_block.m`: Simulink-ready PD baseline wrapper for the 20 m 4-drop task.
- `airdropx_four_drop_schedule.m`: fixed-time 4-drop pulse scheduler.
- `airdropx_mass_cg_update.m`: 4-cargo mass and CG update.
- `airdropx_cargo_trajectory_point.m`: cargo trajectory point generator.
- `airdropx_nw20_metrics_update.m`: online NW20 height-stability metrics.
- `airdropx_report.m`: NW20 and CARP/CEP offline reports.
- `airdropx_carp_cep_block.m`: CARP-window gated drop scheduler and deterministic impact metric.
- `airdropx_plot.m`: dashboard and CARP/CEP target-circle plots.
- `airdropx_carp_monte_carlo.m`: single-release and four-drop Monte Carlo impact scatter.
- `airdropx_sim_params.m`: the single tuning/configuration entry point for the Simulink model.
- `update_airdropx_model_architecture.m`: normalizes the `.slx` wiring so blocks reference configuration variables.

Use them in MATLAB Function blocks first. Keep the JSBSim plant in Python/C++ during the first Simulink integration pass, then replace the plant later if you need code generation or a pure Simulink aircraft model.

## Centralized Parameters

Edit `airdropx_sim_params.m` for tunable mission/model values. `setup_airdropx_simulink`
publishes those values into the base workspace, and `untitled1.slx` references the
variable names instead of hard-coded numeric values.

Important fields:

- `cfg.drop_mode`: `1` uses fixed-time drops to drive JSBSim; `2` uses CARP/CEP.
- `cfg.sim.dt_s`, `cfg.sim.stop_time_s`: model step and stop time.
- `cfg.environment.*`: wind and reset command.
- `cfg.control.*`: target altitude, initial commands, and PD gains.
- `cfg.fixed_drop.*`: fixed four-drop schedule.
- `cfg.carp.*`: CARP target, release window, interval, count, and safety altitude.

Both the fixed scheduler and CARP/CEP block run in the model. Their commands are
kept separate and pass through `DropCommandSelect`; only the selected command
enters the S-Function drop input after `Unit Delay3`.

## NW20 20 m Fixed 4-Drop Simulink Wiring

This matches the repository `nw20_height_4drop` mission:

- no wind
- target altitude 20 m
- first drop at 10 s
- drop interval from `cfg.fixed_drop.interval_s` (`0.2 s` by default)
- four drops total
- PD baseline controller
- no CARP/CEP scoring in this mode

### S-Function Input Mux

Connect the S-Function input vector as:

1. `elevator_delta`
2. `throttle_cmd`
3. `wind_speed_mps`
4. `wind_dir_from_deg`
5. `drop_cmd`
6. `reset_cmd`

Use variables published by `airdropx_sim_params.m`:

```matlab
airdropx_wind_speed_mps
airdropx_wind_dir_from_deg
airdropx_reset_cmd
```

### PD MATLAB Function Block

Use one MATLAB Function block with this body:

```matlab
function [elevator_delta, throttle_cmd, delta_m_signal, h_err, u_raw] = PD_NW20(altitude_m, vz_up_mps, airspeed_mps, mass_kg, drop_count)
%#codegen
[elevator_delta, throttle_cmd, delta_m_signal, h_err, u_raw] = ...
    airdropx_pd_nw20_block(altitude_m, vz_up_mps, airspeed_mps, mass_kg, drop_count);
end
```

Inputs come from the S-Function Demux:

- `altitude_m`: output 2
- `vz_up_mps`: output 3
- `airspeed_mps`: output 4
- `mass_kg`: output 10
- `drop_count`: output 18

Keep two Unit Delay blocks between PD outputs and the S-Function input Mux:

- elevator delay initial condition: `0`
- throttle delay initial condition: `0.80`
- sample time: `dt`

### 4-Drop Scheduler Block

Use another MATLAB Function block:

```matlab
function [drop_cmd, next_drop_index, schedule_done] = DropSchedule(t)
%#codegen
[drop_cmd, next_drop_index, schedule_done] = airdropx_four_drop_schedule(t, ...
    airdropx_fixed_drop_start_s, airdropx_fixed_drop_interval_s, ...
    airdropx_fixed_drop_pulse_s, airdropx_fixed_drop_total);
end
```

Feed it with a Clock block. The fixed command goes to `DropCommandSelect`, not
directly to the S-Function input.

### Metrics Block

Optional online metrics MATLAB Function block:

```matlab
function metrics = NW20Metrics(t, altitude_m, vz_up_mps, elevator_delta, drop_count)
%#codegen
metrics = airdropx_nw20_metrics_update(t, altitude_m, vz_up_mps, elevator_delta, drop_count, 0);
end
```

For final reporting, log signals and run:

```matlab
report = airdropx_report(logsout, "nw20");
```

## CARP/CEP Mode Wiring

CARP/CEP mode runs alongside the fixed `DropSchedule` block. Set
`cfg.drop_mode = 2.0` in `airdropx_sim_params.m` to let CARP/CEP drive the
S-Function drop input; leave `cfg.drop_mode = 1.0` for fixed-time drops.

Use one MATLAB Function block:

```matlab
function [drop_cmd, release_latched, in_window, low_alt_safe, t_to_release_s, ...
          release_n_m, release_e_m, predicted_impact_n_m, predicted_impact_e_m, ...
          miss_distance_m, cep50_to_target_m, actual_release_n_m, actual_release_e_m, actual_release_alt_m, ...
          release_airspeed_mps, release_heading_deg, release_wind_n_mps, release_wind_e_mps, schedule_done] = ...
          CARP_CEP(t, pos_n_m, pos_e_m, altitude_m, airspeed_mps, heading_deg, wind_n_mps, wind_e_mps, drop_count)
%#codegen
[drop_cmd, release_latched, in_window, low_alt_safe, t_to_release_s, ...
 release_n_m, release_e_m, predicted_impact_n_m, predicted_impact_e_m, ...
 miss_distance_m, cep50_to_target_m, actual_release_n_m, actual_release_e_m, actual_release_alt_m, ...
 release_airspeed_mps, release_heading_deg, release_wind_n_mps, release_wind_e_mps, schedule_done] = ...
    airdropx_carp_cep_block(t, pos_n_m, pos_e_m, altitude_m, airspeed_mps, heading_deg, ...
                            wind_n_mps, wind_e_mps, drop_count, ...
                            airdropx_carp_target_n_m, airdropx_carp_target_e_m, ...
                            airdropx_carp_release_window_s, airdropx_carp_interval_s, ...
                            airdropx_carp_drop_total, airdropx_carp_min_safe_alt_m);
end
```

Inputs:

- `t`: Clock
- `pos_n_m`: S-Function output 12
- `pos_e_m`: S-Function output 13
- `altitude_m`: S-Function output 2
- `airspeed_mps`: S-Function output 4
- `heading_deg`: S-Function output 8
- `wind_n_mps`: S-Function output 16
- `wind_e_mps`: S-Function output 17
- `drop_count`: S-Function output 18

Connect `drop_cmd` to `DropCommandSelect`. Do not connect CARP/CEP directly to
the S-Function input; the selector keeps the fixed and CARP command paths
independent.

Log these signals for CARP/CEP reporting:

- `drop_count`
- `in_window`
- `miss_distance_m`
- `actual_release_n_m`
- `actual_release_e_m`
- `actual_release_alt_m`
- `release_airspeed_mps`
- `release_heading_deg`
- `release_wind_n_mps`
- `release_wind_e_mps`

Then run:

```matlab
report = airdropx_report(out.logsout, "carp");
```

To recreate the repository-style plots:

```matlab
airdropx_plot(out.logsout, "dashboard");
```

For the target-circle CARP/CEP plot:

```matlab
airdropx_plot(out.logsout, "carp");
```

For Monte Carlo scatter:

```matlab
mc = airdropx_carp_monte_carlo(out.logsout, "Samples", 300);
airdropx_plot(out.logsout, "carp", "MonteCarlo", mc);
```

For four-drop scatter, log `pos_n_m` and `pos_e_m` in addition to the
CARP/CEP signals, then run:

```matlab
mc4 = airdropx_carp_monte_carlo(out.logsout, "Mode", "fourdrop", "Samples", 300);
airdropx_plot(out.logsout, "carp", "MonteCarlo", mc4);
```
