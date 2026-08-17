# Upgrade from v0.3.1 to v0.3.2

1. Extract this package outside the AirdropX project.
2. Run:

```powershell
.\install_into_airdropx.ps1
```

Do not pass `-InstallSFunctionSource` for the Physics-MPC Smoke.

3. Because the Oracle C++ changed, the first run must rebuild it:

```powershell
cd 'D:\vscode project\AirdropX'
.\run_phys_mpc_D.ps1 -Mode Smoke
```

Do **not** use `-SkipBuild` on that first v0.3.2 run.

4. Do not run the 95-vertex bank until the Smoke is fully PASS. A failure is saved to:

`matlab/results/physics_mpc_v032/physics_smoke_failure.mat`

v0.3.2 keeps v0.3.1 evidence untouched and uses a separate v032 result directory.
