# AirdropX Offline GUI — Physics-MPC v1.4.0 integration

This package reconstructs the original PyQt6 GUI layout from the surviving AirdropX documentation/screenshot and connects it to the current Physics-MPC v1.4.0 MATLAB/JSBSim backend.

## What is implemented

- Original three-column dark/cyan UI structure: left mission/config console, center monitoring tabs, right accuracy/log panel, bottom status bar.
- Free numeric target altitude `H` and target speed `V` inputs.
- Wind modes: calm, constant, step, bidirectional step, ramp, sine and deterministic turbulence-like gusts.
- Original wind-direction field is retained. v1.4.0 is longitudinal, so the GUI projects meteorological wind direction onto the flight track and sends only that signed along-track component to the current MPC plant.
- All original formal v1.4.0 wind scenarios are retained as separate presets. Their original equations are unchanged.
- Sensor-realistic v1.4.0 path and independent cargo-scoring plant are used.
- Result CSVs are loaded back into the GUI for altitude, pitch/q, actuator, mass/CG/Iyy, wind and landing-accuracy visualization.
- Automatic exact-H/V model cache: if the selected H/V is not already available as a complete physics-bank + disturbance-calibration pair, the backend can build an exact 5-cfg bank and calibrate the wind disturbance map locally, then reuse it next time.

## Important distinction: free H/V vs formal certification

The GUI lets you enter arbitrary positive H/V values, but Physics-MPC v1.4.0 requires an exact physics vertex and an exact H/V wind-disturbance calibration. Therefore the first run at a new H/V may need to run trim/linearization + JSBSim disturbance calibration. This is not interpolation and it does not silently substitute a nearby certified point.

Custom wind or newly generated H/V points are **engineering exploration conditions**, not automatically the same as the published/formal v1.4.0 certification set. Select `论文验证预设` + an already certified H/V point when reproducing formal evidence.

## Installation into the existing AirdropX project

PowerShell:

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\install_into_airdropx.ps1 -ProjectRoot 'D:\vscode project\AirdropX'
```

Then:

```powershell
cd 'D:\vscode project\AirdropX\offline_gui_v140'
python -m pip install -r requirements.txt
python main.py
```

The GUI can auto-detect common MATLAB locations. You can also set `AIRDROPX_MATLAB_EXE` or enter the path in the left panel.

## Offline operation

No network service is used. The current source version launches local MATLAB with `-batch` and uses local JSBSim/MEX/assets. For a Windows `.exe`, run `build_python_exe.ps1`; that bundles the Python/PyQt6 front-end but still expects the local MATLAB backend unless you separately compile the MATLAB backend with MATLAB Compiler/Runtime.

## Runtime assets not contained in the uploaded RAR

The uploaded RAR does not contain the original PyQt6 source, `physics_bank.mat`, `wind_disturbance_model_v130.mat`, or the compiled Oracle MEX files. The GUI source here is therefore a reconstruction rather than byte-for-byte restoration. Existing runtime assets from the current working AirdropX installation remain required; new H/V model generation also needs the Physics Oracle build chain.

## Formal-code isolation

Only one existing function receives a backward-compatible GUI branch: `airdropx_wind_profile_v136.m`. For every original formal scenario name, it executes the original v1.3.6 code path unchanged. The GUI custom wind uses the reserved scenario name `gui_custom` and a process-local appdata configuration.
