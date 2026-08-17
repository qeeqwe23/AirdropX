# Physics-MPC v0.5.2 pre-delivery audit

## Root-cause boundary

The observed v0.5.1 stall occurred after scenario A result files existed and before scenario B output directory creation. Since simultaneous payload removal is executed only at t=10 s inside scenario B, it cannot explain a stall before scenario B begins. v0.5.2 therefore treats process/Oracle teardown as the primary hypothesis and separately probes cfg0->cfg4 in a fresh process.

## Safety of watchdog

The PowerShell runner records the PID returned by `Start-Process` and only terminates that PID. It never calls `Get-Process matlab | Stop-Process` and never touches unrelated MATLAB sessions.

A child is force-terminated after success only if its completion marker already exists. The marker is written after mission MAT/CSV/summary persistence.

## Physics invariants

No changes to Q/R, scales, horizon, hard input bounds, bank, Oracle MEX, point-mass semantics, or gate thresholds.

## Simultaneous semantics

`DropTimes_s=[10 10 10 10]` still yields cfg0->cfg4 in one sample. The Oracle receives cfgId=4 and reconstructs all four cargo point-mass weights as zero during that evaluation. v0.5.2 does not serialize the four removals across hidden substeps.

## Probe acceptance

The direct jump probe requires:

- cfg4 QP feasible from the certified cfg0 trim state;
- two cfg4 nonlinear Oracle evaluations return;
- both outputs finite;
- algebraic closure on both calls;
- state repeatability <= 1e-12;
- cfg4 mass error <= 1e-3 kg;
- cfg4 CG error <= 1e-9 m;
- cfg4 Iyy error <= 1e-6 kg m^2.

A failed probe prevents the full simultaneous mission.
