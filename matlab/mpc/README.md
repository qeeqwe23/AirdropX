# AirdropX Strict Grey-Box MPC

This folder now follows the Slegers-style workflow adapted to AirdropX
longitudinal fixed-wing airdrop control. The two Simulink model files are
preserved:

```text
airdropx_mpc_closed_loop.slx
airdropx_mpc_id.slx
```

Only the MATLAB control code they call has been rebuilt.

## Reduced State

The online model uses:

```text
x = [h*, Vz, Va*, theta*_rad, q_radps]'
u = [delta_e*, delta_t*]'
```

The first five states are relative to the active drop-configuration trim. In
particular, `theta* = theta - theta0_sigma`; it is not hard-wired to a generic
pitch target. The preserved Simulink model still provides pitch in degrees;
`sfun_airdropx_mpc_controller` converts that boundary signal to radians before
calling the MPC core.

## Strict Candidate Model

Each drop configuration `sigma = 0..4` has its own trim and mass properties:

```text
m_sigma, Iy_sigma, xCG_sigma, Va0_sigma, theta0_sigma, delta_e0_sigma, delta_t0_sigma
```

The identified force/moment regression uses:

```text
phi = [Va*, alpha*, q, alphadot*, delta_e*, delta_t*, 1]
```

and estimates 21 parameters:

```text
theta_N = [N_V, N_alpha, N_q, N_alphadot, N_de, N_dt, b_N]
theta_X = [X_V, X_alpha, X_q, X_alphadot, X_de, X_dt, b_X]
theta_M = [M_V, M_alpha, M_q, M_alphadot, M_de, M_dt, b_M]
```

`airdropx_mpc_greybox_model` converts those force/moment derivatives into:

```text
dx/dt = A_c,sigma x + B_c,sigma u + d_c,sigma
```

with fixed kinematics:

```text
dh*/dt     = Vz
dtheta*/dt = q
```

Then it uses exact zero-order-hold discretization via `expm`.

## Identification

Use exported measurable channels only. The controller does not read JSBSim
lift, drag, pitching moment, or future states.

```matlab
addpath("matlab")
addpath("matlab/mpc")
id = airdropx_mpc_identify_from_csv( ...
    "matlab/results/some_run/closed_loop_timeseries.csv", ...
    "OutputMat", "matlab/results/mpc_greybox_model.mat");
```

For useful identification, run `airdropx_mpc_id.slx` with actuator excitation.
The ID workspace now creates distinct multi-sine elevator and throttle
excitations by default:

```matlab
airdropx_mpc_setup_id_workspace("Model", "airdropx_mpc_id")
```

The repeatable ID/export wrapper used for the current identified model is:

```matlab
r = airdropx_mpc_run_id_experiment( ...
    "OutputRoot", "matlab/results/mpc_id_slegers_iter02", ...
    "StopTimeS", 80, ...
    "InitialAltitudeM", 120, ...
    "InitialAirspeedMps", 50, ...
    "FixedDropStartS", 20, ...
    "FixedDropIntervalS", 12);

id = airdropx_mpc_identify_from_csv( ...
    r.timeseries_csv, ...
    "TargetAirspeedMps", 50, ...
    "OutputMat", "matlab/results/mpc_id_slegers_final/mpc_identified_slegers_final.mat");

q = airdropx_mpc_assess_identification( ...
    id, r.timeseries_csv, ...
    "OutputFile", "matlab/results/mpc_id_slegers_final/identification_quality_final.csv");
```

`airdropx_mpc_identify_from_csv` applies a small physical projection by default:
clearly invalid signs or extreme magnitudes for `N_alpha`, `X_V`, `X_dt`,
`M_alpha`, `M_q`, and `M_de` are returned to the prior or clipped to a
configured physical range. The raw values are retained in `raw_force_parameters`
inside each identified model.

The closed-loop workspace disables excitation:

```matlab
airdropx_mpc_setup_closed_loop_workspace("Model", "airdropx_mpc_closed_loop")
```

## Main Files

```text
airdropx_mpc_config.m             tuning, trims, mass/CG/Iy, RLS defaults
airdropx_mpc_trim_bank.m          five drop-configuration trim/mass bank
airdropx_mpc_greybox_model.m      21-parameter force/moment model conversion
airdropx_mpc_identify_from_csv.m  joint RLS identification
airdropx_mpc_excitation_profile.m multi-sine ID excitation
airdropx_mpc_controller.m         constrained rolling-horizon MPC
sfun_airdropx_mpc_controller.m    adapter for preserved SLX input/output
airdropx_mpc_evaluate_csv.m       tracking and four-drop impact metrics
airdropx_mpc_run_id_experiment.m  repeatable ID simulation/export wrapper
airdropx_mpc_assess_identification.m model quality checks and one-step RMSE
```

## Smoke Test

```matlab
cfg = airdropx_mpc_config;
[u, st, d] = airdropx_mpc_controller(zeros(7,1), [], cfg)
```

The first five elements are the reduced SI-unit state. Optional elements 6 and
7 are `[mass_err_kg, cg_x_err_m]` and are used only to choose the active
drop-configuration model. The expected output is a bounded
`[elevator_delta; throttle_cmd]` command near the active trim, with
`d.solver.type` reporting `quadprog` when Optimization Toolbox is available.
