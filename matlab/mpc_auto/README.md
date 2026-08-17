# AirdropX R2026a Auto MPC

This folder contains the new MATLAB R2026a route. It is intentionally separate
from `matlab/mpc`, which remains the old grey-box / hand-written QP baseline.

## Route

```text
JSBSim fixed-configuration data
-> iddata multi-experiment sets
-> n4sid / ssest model selection
-> MATLAB Model Predictive Control Toolbox mpc() objects
-> MPC0 ... MPC4 bank for Simulink Multiple MPC Controllers
-> optional bayesopt tuning evaluated on real JSBSim closed-loop runs
```

Measured outputs:

```text
Y = [altitude_m, airspeed_mps, pitch_deg, vz_up_mps, q_dps]
```

Manipulated variables:

```text
U = [elevator_cmd, throttle_cmd]
```

`mass_kg`, `cg_x_m`, and `drop_count` are metadata only. They select the active
configuration/controller; they are not model inputs.

## Files

```text
airdropx_auto_find_trim.m          bayesopt trim search with JSBSim as black box
airdropx_auto_make_excitation.m    random held elevator/throttle excitation
airdropx_auto_run_id_experiment.m  one fixed-configuration ID data run
airdropx_auto_generate_data.m      batch fixed-configuration data generation
airdropx_auto_build_iddata.m       CSV -> multi-experiment iddata splits
airdropx_auto_identify.m           n4sid/ssest model order search
airdropx_auto_validate_models.m    validation/test multi-step prediction report
airdropx_auto_build_mpc_bank.m     create MATLAB mpc() MPC0...MPC4 bank
airdropx_auto_reference.m          ref = [h, Va, pitch_trim, Vz, q]
airdropx_auto_score_timeseries.m   JSBSim closed-loop score function
airdropx_auto_final_test.m         score held-out closed-loop runs
airdropx_auto_mpc_objective.m      bayesopt objective for real JSBSim evaluation
airdropx_auto_tune_mpc.m           bayesopt tuning entry point
airdropx_auto_train_all.m          orchestration entry point
```

## Minimal Smoke

```matlab
addpath("matlab")
addpath("matlab/mpc")
addpath("matlab/mpc_auto")
p = airdropx_auto_make_excitation("StopTimeS", 1);
trim_bank = airdropx_auto_default_trim_bank;
```

## First Full Data/ID Pass

Use existing data:

```matlab
result = airdropx_auto_train_all( ...
    "DataRoot", "matlab/results/mpc_auto_data_some_run", ...
    "DoGenerateData", false);
```

Generate new data first:

```matlab
result = airdropx_auto_train_all( ...
    "DoGenerateData", true, ...
    "RunsPerConfig", 30, ...
    "StopTimeS", 30, ...
    "RecordStartS", 8);
```

The output `airdropx_learned_mpc.mat` contains:

```text
Plant0 ... Plant4
MPC0 ... MPC4
plant_bank
controllers
trim_bank
```

## Simulink Wiring Target

In the new Simulink closed-loop model, use Model Predictive Control Toolbox's
Multiple MPC Controllers block:

```text
mo     = [altitude_m, airspeed_mps, pitch_deg, vz_up_mps, q_dps]
ref    = airdropx_auto_reference(drop_count, trim_bank)
switch = drop_count + 1
mv     = [elevator_cmd, throttle_cmd]
```

The JSBSim S-Function output 20 is now `q_dps`, sourced from
`velocities/q-rad_sec`. If logs do not yet contain `q_dps`, the auto data path
falls back to differentiating `pitch_deg` only as a temporary compatibility aid.

## Tuning Rule

`airdropx_auto_tune_mpc` intentionally requires `EvaluationFcn`. The evaluation
function must run the real JSBSim closed-loop model with the candidate MPC bank
and return a timeseries table or CSV. This keeps the teacher as JSBSim, not the
identified plant.

## v29 Final Full-Mission Validation

After cfg0..cfg4 are all `verified`, `airdropx_auto_mpc_200m_all_configs` now
runs one additional acceptance test by default. This test does not tune any
controller and does not write new LearningBank observations.

It starts at cfg0 fully loaded, performs all four real payload drops, switches
through cfg0 -> cfg1 -> cfg2 -> cfg3 -> cfg4, disables the artificial
certification pulses, and keeps flying after the fourth drop to judge recovery.

For an already-completed v29 run, call it directly without rerunning learning:

```matlab
addpath("matlab")
addpath("matlab/mpc")
addpath("matlab/mpc_auto")
r = airdropx_auto_final_mission_validation( ...
    "ProjectRoot", pwd, ...
    "OutputRoot", "matlab/results/mpc_auto_200m_all_cfg_v16");
```

Outputs are written to:

```text
matlab/results/mpc_auto_200m_all_cfg_v16/final_mission_validation/
  final_mission_summary.csv
  final_mission_gate_report.csv
  drop_transition_summary.csv
  final_mission_curves.png
  FINAL_MISSION_PASS.txt or FINAL_MISSION_FAIL.txt
  final_mission_result.mat
  simulation/closed_loop_timeseries.csv
```

The mission-level transition gates cover the entire four-drop transient. The
final tail still uses the strict v29 altitude/vertical-speed stability limits.
`MISSION_PASS=1` is the final 200 m / 50 m/s complete-mission acceptance flag.

## v30 Unified Flight-Envelope Learning

v30 keeps the v29 per-mission learner intact and adds a cross-mission envelope layer:

```text
airdropx_auto_envelope_train
  -> airdropx_auto_run_any_mission(H,V)
     -> nearest PlantContextBank seed
     -> v29 equilibrium probe
     -> reuse Plant OR trim -> ID -> identify -> validate -> rebuild
     -> shared v29 Controller LearningBank
     -> cfg0..cfg4 VERIFIED
     -> Final Mission Validation
```

New files:

```text
airdropx_auto_plant_context_bank.m
airdropx_auto_run_any_mission.m
airdropx_auto_envelope_train.m
```

Start the resume-safe 20-200 m envelope trainer from the project root with:

```powershell
powershell -ExecutionPolicy Bypass -File .\run_v30_envelope_D_temp.ps1
```

The configured 30-70 m/s range is a search interval, not a guaranteed flight
envelope. Only complete cfg0->cfg4 Final Mission PASS contexts count as qualified.
See `V30_FLIGHT_ENVELOPE_README.txt` for details.
