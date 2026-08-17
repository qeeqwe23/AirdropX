AirdropX Physics MPC v1.2 — Flight Stability Baseline
====================================================

Goal
----
Before tuning the MPC or working on CARP/drop accuracy, make the physical
operating points and local models deterministic and repeatable.

Critical v1.1 root cause fixed
------------------------------
airdropx_sim_params normally generates one shared file:
  aircraft/MQ9_Reaper/generated/reset_20m_runtime.xml

With 3 process workers, V45/V50/V55 runs could overwrite that same XML while
another JSBSim instance was about to load it. This explains contradictory
results such as V50 cfg4 passing deterministic trim and then immediately
failing an identical zero-input recertification.

v1.2 fix:
- Physics-MPC calls airdropx_auto_run_id_experiment with IsolateGeneratedIc=true.
- Each simulation creates its own unique IC XML under the MQ9 generated folder.
- The unique file is passed explicitly to airdropx_sim_params/JSBSim.
- It is deleted automatically after the run.
- 3-worker process parallelism remains enabled.

Additional strictness
---------------------
1. Every accepted equilibrium is executed twice with fresh isolated IC files.
2. Both runs must pass the equilibrium hard gates.
3. Their Va/vz/q/height-slope/Va-slope/pitch/actuator metrics must agree within
   fixed repeatability tolerances.
4. baseline_repeatability_metrics.csv is written for every candidate node.
5. v1.1 linearization sample compression was fixed: out-of-local-range samples
   are masked, not deleted. Regression and validation only use genuinely
   contiguous Ts=0.1 s transitions.
6. No equilibrium, state, or model-fit gates were loosened.
7. No MPC weights were changed.
8. No BayesOpt/online learning/model-order search was introduced.

Important interpretation
------------------------
Do NOT regard the old v1.1 9/6 table as the physical flight envelope. It was
collected while the shared generated-IC race existed. Rebuild all 15 nodes.
The v1.2 version signature intentionally prevents reuse of v1.1 node caches.

Run
---
1) Overlay this patch onto D:\vscode project\AirdropX
2) Run:
     .\run_physics_mpc_preflight_D_temp.ps1
3) Then:
     .\run_physics_mpc_build_D_temp.ps1

Inspect after completion:
  matlab/results/mpc_physics_v1/physics_mpc_build_failures.csv
  matlab/results/mpc_physics_v1/physics_mpc_model_report.csv
  matlab/results/mpc_physics_v1/linearization/Vxxx.xxx/cfgN/
      baseline_metrics.csv
      baseline_after_retrim_metrics.csv       (when retrim was needed)
      baseline_repeatability_metrics.csv      (new in v1.2)
      deterministic_retrim/deterministic_trim_trace.csv

Next decision
-------------
Only after the clean v1.2 15-node scan:
- if equilibrium nodes remain impossible/repeatably bad, improve physical trim;
- if equilibrium is good but local modeling fails, fix excitation/modeling;
- only after the bank is complete do we tune/validate scheduled MPC and runtime
  Hcmd/Vcmd changes.

Cache/stale diagnostics
-----------------------
v1.2 node version is physics_mpc_v1_2. v1.1 cache files are never reused.
When a node is rebuilt, its old node output directory is removed first so old
v1.1 trim traces cannot be mistaken for fresh v1.2 evidence.
