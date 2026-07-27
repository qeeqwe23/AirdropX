# AirdropX MPC Outer Loop + PD Inner Loop

This folder contains the canonical "MPC outer loop + PD inner loop" prototype.
It is separate from `matlab/mpc`, which remains the direct-actuator MPC baseline.

Architecture:

```text
measured states -> MPC outer loop -> [pitch_ref_deg, throttle_cmd]
pitch_ref_deg + measured pitch/q -> PD inner loop -> elevator_delta
[elevator_delta, throttle_cmd] -> JSBSim plant
```

By default the outer loop uses the validated direct-MPC longitudinal demand as
an equivalent control allocation target, then converts that demand into a
pitch-reference command through the PD inner-loop inverse. Setting
`UseDirectMpcAllocation` to `false` switches to the pure pitch-reference nominal
outer model in this folder.

The generated Simulink model is:

```text
matlab/mpc_outer_pd/airdropx_mpc_outer_pd_closed_loop.slx
```

The canonical `.slx` keeps the original VR sink connected and points it to
`matlab/vr/airdropx_scene.wrl`. Batch runs disable VR in memory by default so
optimization remains deterministic and fast; set `"DisableVRForBatch", false`
when you specifically want a VR-enabled simulation run.

Regenerate and open the VR-connected model:

```matlab
addpath("matlab")
addpath("matlab/mpc")
addpath("matlab/mpc_outer_pd")
airdropx_mpc_outer_pd_create_model("DisableVR", false);
airdropx_mpc_outer_pd_setup_workspace;
open_system("airdropx_mpc_outer_pd_closed_loop")
```

Run a closed-loop case:

```matlab
addpath("matlab")
addpath("matlab/mpc")
addpath("matlab/mpc_outer_pd")
r = airdropx_mpc_outer_pd_run_closed_loop;
```

Run a warm-up case where the aircraft first flies for 20 s, then exports and
evaluates the following 30 s as a fresh time window. The fixed four-drop
schedule is shifted with the warm-up, so the first drop still appears at
`t = 10 s` in the exported CSV. The optimized warm-up preset prioritizes
altitude and pitch over airspeed:

```matlab
r = airdropx_mpc_outer_pd_run_optimized_warmup;
```

Run optimization and compare against the direct MPC baseline:

```matlab
c = airdropx_mpc_outer_pd_optimize_compare("MaxIterations", 9);
```

The comparison writes `comparison_summary.csv`, `iteration_summary.csv`, and
`best_run.json` under `matlab/results/mpc_outer_pd_compare_<timestamp>/`.
