# Upgrade from v0.3.0 to v0.3.1

1. Extract v0.3.1 outside the AirdropX project.
2. Run `./install_into_airdropx.ps1` from PowerShell.
3. Do not pass `-InstallSFunctionSource`; the working Simulink S-Function is not part of this fix.
4. Run the first Smoke **without** `-SkipBuild` because the Physics Oracle C++ source changed:

```powershell
cd 'D:\vscode project\AirdropX'
.\run_phys_mpc_D.ps1 -Mode Smoke
```

v0.3.1 writes to `matlab/results/physics_mpc_v031` and leaves v0.3.0 evidence intact.

The A->B->A gate tolerance has not been relaxed. The fix resets all JSBSim model
memories before reconstructing each Oracle map evaluation.
