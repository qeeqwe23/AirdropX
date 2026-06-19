# AirdropX MATLAB/Simulink Quick Start

This document is the portable entry point for the MATLAB/Simulink workflow.
It does not assume any absolute path on the original developer machine.

## Requirements

- Windows 10/11 x64 is the bundled, ready-to-run target.
- MATLAB with Simulink.
- The included `sfun_airdropx_jsbsim.mexw64` is built for Windows x64.
- If MATLAB reports that the MEX file is incompatible, rebuild it with a
  supported C++ compiler from MATLAB.

The repository includes:

- `third_party/JSBSim/`: JSBSim source tree.
- `third_party/jsbsim-win64/`: Windows x64 JSBSim headers and `JSBSim.lib`.
- `matlab/sfunc_jsbsim/sfun_airdropx_jsbsim.mexw64`: prebuilt S-Function.
- `matlab/untitled1.slx`: Simulink model.

## First Check After Download

Open MATLAB and run from the downloaded `AirdropX` directory:

```matlab
cd('path/to/AirdropX')
addpath('matlab')
airdropx_release_check
```

Expected result:

```text
AirdropX release check PASSED.
```

## Run A Simulation And Export CSV

```matlab
cd('path/to/AirdropX')
addpath('matlab')
r = airdropx_run_and_export("RunName", "my_first_run");
```

CSV files are written to:

```text
matlab/results/my_first_run/timeseries.csv
matlab/results/my_first_run/drop_table.csv
matlab/results/my_first_run/summary.csv
matlab/results/my_first_run/params.csv
```

## Open The Simulink Model

```matlab
cd('path/to/AirdropX')
setup_airdropx_simulink("OpenModel", true)
```

Before manually pressing Run in Simulink, refresh the model variables and block
scripts:

```matlab
update_airdropx_model_architecture("untitled1")
```

The export script does this refresh automatically.

## Rebuild The JSBSim S-Function

Only needed if the bundled MEX is missing or incompatible:

```matlab
cd('path/to/AirdropX')
addpath('matlab')
addpath('matlab/sfunc_jsbsim')
clear mex
build_sfun_airdropx_jsbsim
```

For non-Windows platforms, first build/install JSBSim for that platform, then
pass the install root:

```matlab
build_sfun_airdropx_jsbsim('/path/to/jsbsim-install')
```

## Tuning Entry Point

Edit one file for tunable simulation parameters:

```text
matlab/airdropx_sim_params.m
```

Important fields:

- `cfg.initial.*`: initial airspeed and attitude.
- `cfg.control.*`: target altitude, initial commands, controller gains.
- `cfg.fixed_drop.*`: fixed four-drop schedule.
- `cfg.carp.*`: CARP/CEP target and release settings.

Do not tune values directly inside the `.slx` model.
