# AirdropX MPC Outer Loop + PD Inner Loop

This folder contains the canonical "MPC outer loop + PD inner loop" prototype.
It is separate from `matlab/mpc`, which remains the direct-actuator MPC baseline.

Architecture:

```text
measured states -> MPC outer loop -> [pitch_ref_deg, throttle_cmd]
pitch_ref_deg + measured pitch/q -> PD inner loop -> elevator_delta
[elevator_delta, throttle_cmd] -> JSBSim plant
```

By default the outer loop uses the pure pitch-reference nominal model in this
folder: MPC commands `pitch_ref_deg` and `throttle_cmd`, then the PD inner loop
commands elevator. Set `UseDirectMpcAllocation` to `true` only when you want to
exercise the older direct-MPC allocation bridge.

The shared MPC solver defaults to constrained `quadprog` optimization when it is
available, so both the direct-MPC allocation mode and the pure pitch-reference
outer model honor configured input and rate limits inside the optimized move
sequence. If `quadprog` is unavailable, the solver automatically falls back to
the historical unconstrained move with final clipping.

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

Run the validated warm-up CARP case where the aircraft first flies for 20 s,
then exports and evaluates the following 30 s as a fresh time window. The
optimized warm-up preset uses a 20 m / 50 m/s / 4 deg reference and prioritizes
safe altitude, pitch stability, and four-drop release completion:

```matlab
r = airdropx_mpc_outer_pd_run_optimized_warmup;
```

The optimized CARP preset uses four separate along-track drop targets relative
to the target center:

```matlab
[0.8; 1.6; 2.4; 3.8]  % north offsets in meters
```

The CARP gate now waits for the aircraft to cross the release line before
latching the first release. The release window remains available as a
diagnostic tolerance, but it no longer causes an early release as soon as the
aircraft enters the window.

To sweep ballistic drag sensitivity while keeping the four different targets:

```matlab
s = airdropx_mpc_outer_pd_sweep_drag_targets( ...
    "DragScales", [0.85; 1.0; 1.15], ...
    "TargetSpreadsM", 0.8);
```

The sweep writes `sweep_summary.csv` and `best_drag_target_case.json` under
`matlab/results/mpc_outer_pd_drag_target_sweep_<timestamp>/`, with per-drop
miss columns for all four cargo releases.

Run optimization and compare against the direct MPC baseline:

```matlab
c = airdropx_mpc_outer_pd_optimize_compare("MaxIterations", 9);
```

The comparison writes `comparison_summary.csv`, `iteration_summary.csv`, and
`best_run.json` under `matlab/results/mpc_outer_pd_compare_<timestamp>/`.
