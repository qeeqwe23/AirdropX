# AirdropX Physics-MPC v0.5.2 — isolated drop-timing comparison

## Why v0.5.2 exists

v0.5.1 completed the 2 s spacing scenario and wrote all of its result files, but the comparison MATLAB process then stopped making CPU progress before the `simultaneous_4x` scenario directory was created. Because the simultaneous release itself would not occur until t=10 s, that observation does **not** support blaming the cfg0->cfg4 payload jump. The stall point is instead consistent with the return/teardown boundary of scenario A, where its persistent JSBSim Oracle was explicitly destroyed by an `onCleanup` callback.

v0.5.2 therefore changes experiment orchestration rather than controller physics.

## What is unchanged

The following are unchanged from the validated v0.5.1/v0.5.0 controller path:

- JSBSim Physics Oracle v0.3.3;
- certified v0.3.3 95-point bank;
- Q and R;
- Bryson state/input scales;
- Np=Nc=100;
- hard elevator/throttle bounds;
- cfg0->1->2->3->4 physical point-mass semantics;
- trim/A/B/P/K scheduled from the certified physics bank;
- all mission performance gates.

No MEX, C++, S-Function, Q/R, horizon, or gate threshold is changed by this package.

## Main change: one MATLAB process per JSBSim scenario

The runner no longer calls two nonlinear JSBSim missions sequentially inside one MATLAB process.

Instead it launches:

1. optional/reused 2 s spacing scenario;
2. a dedicated cfg0->cfg4 one-sample jump probe;
3. the simultaneous four-payload mission;
4. a clean finalizer process that only reads MAT/CSV files and never initializes JSBSim.

Each JSBSim child writes a completion marker **after** all scenario MAT/CSV/summary files are saved. The PowerShell runner then allows a short normal shutdown grace period. If MATLAB/JSBSim teardown stalls after the completion marker, the runner terminates **only the child PID it created**. It never scans or kills unrelated MATLAB processes.

This makes a post-result MEX/JSBSim destructor stall unable to block the next experiment.

## Direct cfg0->cfg4 probe

Before the 40 s simultaneous mission, v0.5.2 runs a fresh-process probe that reproduces the actual simultaneous transition path:

- start from the certified cfg0 trim state;
- switch the common MPC model directly to cfg4;
- solve the cfg4 QP from that state;
- call the nonlinear Oracle at cfg4 twice with identical x/u;
- verify both calls return;
- verify cfg4 mass/CG/Iyy;
- verify algebraic closure;
- verify exact repeatability.

If this probe does not PASS, the full simultaneous mission is refused. This distinguishes a real cfg0->cfg4 physics/oracle problem from a process-lifecycle problem.

## Existing 2 s result reuse

By default the runner checks:

```text
matlab/results/physics_mpc_v051_drop_timing_compare/interval_2s/
```

If the completed `four_drop_mission.mat` and `four_drop_timeseries.csv` exist, they are reused and the known 2 s scenario is not rerun.

Use `-RerunInterval2s` to force a fresh isolated-process run.

## Install

From the unpacked v0.5.2 folder:

```powershell
.\install_drop_timing_v052.ps1
```

The installer only overlays MATLAB controller/orchestration files and the PowerShell runner.

## Run

Recommended now, because your v0.5.1 2 s result already exists:

```powershell
cd 'D:\vscode project\AirdropX'
.\run_phys_mpc_drop_timing_compare_D.ps1 -Mode Compare
```

It will reuse the 2 s result, run the direct cfg0->cfg4 probe in a fresh process, then run simultaneous_4x in another fresh process.

To run only the probe + simultaneous scenario while still using the existing 2 s result for the final comparison:

```powershell
.\run_phys_mpc_drop_timing_compare_D.ps1 -Mode SimultaneousOnly
```

To rerun the 2 s case as well:

```powershell
.\run_phys_mpc_drop_timing_compare_D.ps1 -Mode Compare -RerunInterval2s
```

## New result root

```text
matlab/results/physics_mpc_v052_drop_timing_compare/
```

Important files:

```text
simultaneous_cfg4_probe/
  probe_status.txt
  probe_summary.txt
  simultaneous_cfg4_probe.mat
  probe_complete.ok

simultaneous_4x/
  scenario_status.txt
  scenario_complete.ok
  four_drop_mission.mat
  four_drop_timeseries.csv
  four_drop_event_metrics.csv
  four_drop_summary.txt

process_lifecycle_summary.txt
finalize_terminal.txt
drop_timing_comparison.csv
drop_timing_comparison.txt
drop_timing_comparison.mat
drop_timing_comparison.png
```

## Interpreting a forced child exit

If `process_lifecycle_summary.txt` says:

```text
ForcedExitAfterResult = True
```

but the corresponding completion marker and mission summary exist, the **simulation already completed and saved its results**. The forced exit occurred only after result persistence, during process teardown. It is lifecycle evidence, not a mission failure.

If no completion marker appears before the timeout, the runner terminates only that child and reports the status phase where progress stopped. In particular the cfg0->cfg4 probe status can distinguish `ORACLE_INIT_*` from `CFG4_EVAL_*` stalls.
