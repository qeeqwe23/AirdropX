# Upgrade from v0.3.2

1. Extract v0.3.3.
2. Run `./install_into_airdropx.ps1` without `-InstallSFunctionSource`.
3. Because the Oracle C++ changed, run the first Smoke **without** `-SkipBuild`:

```powershell
cd 'D:\vscode project\AirdropX'
.\run_phys_mpc_D.ps1 -Mode Smoke
```

The existing Simulink S-Function is not rebuilt by default.

v0.3.3 writes results to:

`matlab/results/physics_mpc_v033`

Do not run the 95-point bank until Smoke reaches Richardson linearization and terminal design successfully.
