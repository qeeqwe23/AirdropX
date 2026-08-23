# Physics-MPC v0.6.1

This release does **not** retune the controller. It adds formal nonlinear altitude validation for the already validated v0.6.0 PreviewOnly controller.

- Locks `V=50 m/s`.
- Certifies every exact bank altitude `H=20:10:200 m` (19 missions).
- Every mission starts at cfg0 and performs the original close four-drop sequence at `10.0/10.2/10.4/10.6 s`, ending at cfg4.
- `EnableQSoft=false` for every mission.
- Q/R, Bryson scales, Np/Nc=100, input bounds and all mission gates remain unchanged.
- Up to three isolated MATLAB/JSBSim processes run in parallel. Each has its own completion marker and PID-scoped teardown watchdog.
- Finalizer audits all 95 `H x cfg` bank vertices for exactly common Q/R/scales/horizon and aggregates mission gates/peaks/recovery/realtime/prediction errors.
- Resume is automatic: completed height directories are reused unless `-Force` is passed.
