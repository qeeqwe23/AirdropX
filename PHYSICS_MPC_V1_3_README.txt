AirdropX Physics MPC v1.3 — Direct-Cfg Nonlinear Trim Baseline
===============================================================

Purpose
-------
Solve flight stability first. No CARP/drop-accuracy work is changed here.
No BayesOpt, online learning, model-order search or per-case MPC tuning.

Why v1.3 exists
---------------
v1.2 proved the 3-worker IC race was fixed and gave 13/15 repeatable nodes.
V45 cfg3/cfg4 still failed because the old 2-variable trim solve adjusted only
external elevator/throttle while the aircraft was reached through a real drop
transient. That does NOT prove the 45 m/s target configuration lacks a steady
operating point, especially because some V45 linear plants are mildly open-loop
unstable (rho slightly > 1), which MPC is expected to stabilize.

v1.3 cleanly separates offline modelling from the real mission:
  * OFFLINE trim/linearization: JSBSim starts directly at requested cfg0..cfg4
    mass/CG via airdropx_initial_drop_count.
  * ONLINE/mission validation: still starts cfg0 and performs real drops.

Three-variable physical trim
----------------------------
Offline steady unknowns:
  [pitch state, external elevator delta, throttle]

The solver is deterministic finite-difference Gauss-Newton with fixed probes,
step limits and line-search factors. It minimizes normalized physical residuals:
  Va - Vref
  vz
  q
  height slope
  Va slope

Pitch is NOT an online actuator/reference. It is only an equilibrium-state
unknown, analogous to solving angle of attack/attitude in a normal fixed-wing
trim problem. A candidate is accepted only if the original hard equilibrium
gates pass.

Physical input coordinates
--------------------------
Local models are identified in PHYSICAL elevator/throttle coordinates.
Direct-cfg autoTrimSettle offsets may vary with cfg offline. Runtime conversion
back to the S-function's external elevator-delta coordinate uses cfg0's hidden
offset at each speed node, because a real mission starts at cfg0 and that hidden
offset remains fixed while payloads are dropped.

Build procedure
---------------
1. Copy this patch into D:\vscode project\AirdropX.
2. Run:
     .\run_physics_mpc_preflight_D_temp.ps1
3. Run:
     .\run_physics_mpc_build_D_temp.ps1

The build script recompiles sfun_airdropx_jsbsim.mexw64 automatically when the
patched C++ source is newer, then runs a direct-cfg smoke test before launching
the 15-node 3-worker scan.

Smoke test requirement
----------------------
The direct-cfg smoke initializes cfg3 without drop pulses and must report:
  drop_count = 3
  mass ~= 2523 kg
If this fails, the scan does not start.

Expected diagnostics
--------------------
  matlab/results/mpc_physics_v1/physics_mpc_build_failures.csv
  matlab/results/mpc_physics_v1/physics_mpc_model_report.csv
  matlab/results/mpc_physics_v1/linearization/Vxx.xxx/cfgN/
      baseline_metrics.csv
      baseline_repeatability_metrics.csv
      deterministic_retrim/deterministic_trim_trace.csv   (if retrim needed)
      physics_linear_model.mat                            (if PASS)

Version/cache
-------------
Node version is physics_mpc_v1_3. v1.2 node caches are intentionally not reused,
because direct target-cfg initialization changes the physical modelling path.

Do NOT run CARP or final drop validation until the v1.3 physical bank is complete.
