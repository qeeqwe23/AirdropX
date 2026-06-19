# Download And Run

Use this file when sharing the MATLAB/Simulink version of AirdropX with another
machine.

## What Is Included

- Simulink model: `matlab/untitled1.slx`
- Tunable parameters: `matlab/airdropx_sim_params.m`
- JSBSim aircraft model: `aircraft/MQ9_Reaper/`
- JSBSim source tree: `third_party/JSBSim/`
- Windows x64 JSBSim headers/library: `third_party/jsbsim-win64/`
- Prebuilt Windows x64 S-Function: `matlab/sfunc_jsbsim/sfun_airdropx_jsbsim.mexw64`
- CSV export runner: `matlab/airdropx_run_and_export.m`
- Package checker: `matlab/airdropx_release_check.m`

## Minimal MATLAB Commands

```matlab
cd('path/to/AirdropX')
addpath('matlab')
airdropx_release_check
r = airdropx_run_and_export("RunName", "download_test");
```

The CSV output appears in:

```text
matlab/results/download_test/
```

## If The MEX File Does Not Load

Rebuild it from MATLAB:

```matlab
cd('path/to/AirdropX')
addpath('matlab')
addpath('matlab/sfunc_jsbsim')
clear mex
build_sfun_airdropx_jsbsim
```

The default build script uses the bundled Windows x64 JSBSim development
package under `third_party/jsbsim-win64`.

## Editing Parameters

Only edit:

```text
matlab/airdropx_sim_params.m
```

Do not tune constants inside the Simulink model. The setup/export scripts
publish parameters into the MATLAB base workspace and refresh the model block
scripts automatically.

## Before Sending A Zip

Run:

```matlab
airdropx_release_check
```

Optional smoke simulation:

```matlab
airdropx_release_check("RunSmoke", true)
```

To create a clean zip from PowerShell:

```powershell
cd path/to/AirdropX
powershell -ExecutionPolicy Bypass -File scripts/package_matlab_release.ps1
```

The script excludes generated folders such as `matlab/results/`, `slprj/`, and
Simulink autosave/cache files.
