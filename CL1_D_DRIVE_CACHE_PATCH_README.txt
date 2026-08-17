AirdropX Physics MPC CL-1 — D-drive cache patch

Purpose
- Prevent MATLAB/Simulink/Parallel Computing Toolbox from filling C:.
- No controller logic, MPC bank, trim, gates, or flight scenario is changed.

Redirected paths
- TEMP/TMP: D:\MATLAB_TEMP\AirdropX_physics_mpc_cl1\temp
- Parallel JobStorage: D:\MATLAB_TEMP\AirdropX_physics_mpc_cl1\jobs
- Main Simulink cache: D:\MATLAB_TEMP\AirdropX_physics_mpc_cl1\main_cache
- Main Simulink codegen: D:\MATLAB_TEMP\AirdropX_physics_mpc_cl1\main_codegen
- Per-speed worker cache/codegen: D:\AXC\phys_cl1\V045|V050|V055

Safety
- Startup preflight prints and verifies tempdir, CacheFolder, CodeGenFolder, JobStorageLocation, and SDI/DMR source.
- Any required path not on D: causes an immediate error before the 255 s closed-loop flights start.
- Each worker also verifies its tempdir is on D:.

Files to overwrite in AirdropX root
1. run_physics_mpc_fixed_stability_D_temp.ps1
2. check_physics_mpc_fixed_stability.ps1
3. matlab/mpc_auto/airdropx_physics_mpc_fixed_stability_scan.m

Run
.\run_physics_mpc_fixed_stability_D_temp.ps1

Monitor
.\check_physics_mpc_fixed_stability.ps1
