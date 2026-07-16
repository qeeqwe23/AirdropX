# AirdropX Standalone MPC Sandbox

This folder is intentionally separate from the existing Simulink model and PD
controller. Nothing here is wired into `untitled1.slx`.

## Goal

The MPC sandbox is for longitudinal airdrop tuning:

- hold altitude during the drop window
- hold airspeed near the ballistic reference
- hold pitch near the release attitude
- generate data files that can be inspected before any Simulink integration

The standalone MPC sandbox commands a 20 m altitude target and 45 m/s airspeed
target. The default closed-loop entry condition uses 55.5 m/s with a 2.4 degree
flight-path angle and a 0.95 m internal altitude bias. In this full-load 20 m
case, entering at 45 m/s does not leave enough energy margin to keep altitude
smooth through the release transient; the exported CSV still evaluates against
the real 20 m / 45 m/s / 4 degree command.

## State, Input, Model

The default state vector is:

```text
x = [
  h_err_m
  vz_up_mps
  v_err_mps
  pitch_err_deg
  q_dps
  mass_err_kg
  cg_x_err_m
]
```

The input vector is:

```text
u = [
  elevator_delta
  throttle_cmd
]
```

The controller assumes a local discrete model:

```text
x(k+1) = A x(k) + B (u(k) - u_trim)
```

The included nominal model is only a safe starting point. Use
`airdropx_mpc_identify_from_csv` with exported JSBSim data to build a local
model for the current aircraft, mass, trim, and reference condition.

## Typical Offline Workflow

From the project root in MATLAB:

```matlab
addpath("matlab")
addpath("matlab/mpc")

csv = "matlab/results/codex_stability_check/timeseries.csv";
summary = airdropx_mpc_evaluate_csv(csv);
id = airdropx_mpc_identify_from_csv(csv);
demo = airdropx_mpc_demo_offline(csv);
```

To create a richer identification dataset without connecting MPC to Simulink:

```matlab
exc = airdropx_mpc_make_excitation_data("NumCases", 6);
```

This runs several existing PD/JSBSim cases with varied references and writes a
combined CSV containing `case_id`, `target_altitude_m`, `target_airspeed_mps`,
and `target_pitch_deg`. The identifier resets transitions at `case_id`
boundaries.

To create a separate Simulink identification model:

```matlab
idModel = airdropx_mpc_create_id_model;
```

This creates `matlab/mpc/airdropx_mpc_id.slx` from `matlab/untitled1.slx` and
adds two injection points before the JSBSim input Mux:

```text
elevator_to_plant = elevator_base + airdropx_mpc_elevator_excitation
throttle_to_plant = throttle_base + airdropx_mpc_throttle_excitation
```

The original `untitled1.slx` is not modified.

To run a real actuator-excitation experiment on that separate model:

```matlab
id = airdropx_mpc_run_id_experiment;
```

This writes `id_timeseries.csv`, `excitation_inputs.csv`, and
`identified_model.mat` under `matlab/results/mpc_id_experiment_*`.

If you open `airdropx_mpc_id.slx` manually, initialize the base workspace first:

```matlab
addpath("matlab")
addpath("matlab/mpc")
airdropx_mpc_setup_id_workspace
```

Without this, the two From Workspace blocks will complain that
`airdropx_mpc_elevator_excitation` and `airdropx_mpc_throttle_excitation` do not
exist.

To test MPC in closed loop on the identified model only:

```matlab
sim = airdropx_mpc_simulate_identified("matlab/results/mpc_id_experiment_real_01/identified_model.mat");
```

The demo writes:

```text
matlab/results/<run>/mpc/summary.csv
matlab/results/<run>/mpc/identified_model.mat
matlab/results/<run>/mpc/replay_commands.csv
```

`replay_commands.csv` is not a closed-loop simulation. It is a controller replay
over measured states so the suggested elevator/throttle behavior can be reviewed
before touching Simulink.

## Manual Closed-Loop Run

If you open `matlab/mpc/airdropx_mpc_closed_loop.slx` manually and press Run,
initialize the tuned closed-loop workspace first:

```matlab
addpath("matlab")
addpath("matlab/mpc")
airdropx_mpc_setup_closed_loop_workspace
open_system("airdropx_mpc_closed_loop")
```

The generated closed-loop `.slx` also stores this setup in its load/init
callbacks. If the model was created before this helper existed, regenerate it:

```matlab
airdropx_mpc_create_closed_loop_model
```

For manual visualization, create a separate MPC model that keeps the original VR
blocks enabled:

```matlab
airdropx_mpc_create_closed_loop_model( ...
    "TargetModel", "airdropx_mpc_closed_loop_vr", ...
    "DisableVR", false)
open_system("airdropx_mpc_closed_loop_vr")
```

Use the non-VR `airdropx_mpc_closed_loop.slx` for batch runs and metrics; use
the VR variant only for interactive viewing.

## Integration Rule

Do not edit `matlab/untitled1.slx` for MPC work. The MPC Simulink model is a
separate generated file:

```text
matlab/mpc/airdropx_mpc_closed_loop.slx
```

`airdropx_mpc_create_closed_loop_model` copies `matlab/untitled1.slx` as a
template and then edits only the copy under `matlab/mpc/`. The original model
remains the baseline PD/JSBSim model.
