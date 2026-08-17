# Upgrade from v0.2.2

1. Extract v0.3.0 anywhere outside the AirdropX project.
2. Run `./install_into_airdropx.ps1` from PowerShell.
   - The current `matlab/phys_mpc` directory is backed up first.
   - The working S-Function source/MEX is not modified.
3. From `D:\vscode project\AirdropX`, run exactly once:
   `./run_phys_mpc_D.ps1 -Mode Smoke`
   Do not use `-SkipBuild`: v0.3.0 has a new Oracle C++ API.
4. If Smoke passes, subsequent checks/builds may use `-SkipBuild`.

A Smoke failure now saves `matlab/results/physics_mpc_v030/physics_smoke_failure.mat`
and prints the exact failed gate. Do not change Q/R or relax trim tolerances in response
to a failed preflight; diagnose the named physical/numerical gate instead.


The first Smoke also checks that the persistent Oracle fully resets between unrelated
calls and that JSBSim trim uses the primary elevator rather than an undeclared pitch-trim
actuator. These are structural gates; do not bypass them.

## Additional structural gates in v0.3.0

The persistent Oracle now resets JSBSim InitialCondition for every evaluation, verifies the requested `h/Va/gamma/theta/q` state after `RunIC`, and requires an explicit theta fixed-point residual in addition to q and height checks.
