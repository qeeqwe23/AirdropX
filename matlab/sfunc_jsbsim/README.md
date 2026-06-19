# JSBSim C++ S-Function Route

This folder contains a first-pass C++ S-Function wrapper for using JSBSim as the Simulink plant.

## Files

- `sfun_airdropx_jsbsim.cpp`: Level-2 C++ MEX S-Function.
- `build_sfun_airdropx_jsbsim.m`: MATLAB build helper.

## S-Function Parameters

Use an S-Function block with:

```matlab
sfun_airdropx_jsbsim(projectRoot, aircraftName, icName, dt)
```

Example:

```matlab
projectRoot = "path/to/AirdropX";
aircraftName = "MQ9_Reaper";
icName = fullfile(projectRoot, "aircraft", "MQ9_Reaper", "reset_20m");
dt = 1/120;
```

## Input Vector

Width: 6

| Index | Signal | Unit |
| --- | --- | --- |
| 1 | `elevator_delta` | normalized, added to current trim |
| 2 | `throttle_cmd` | normalized 0..1 |
| 3 | `wind_speed_mps` | m/s |
| 4 | `wind_dir_from_deg` | deg, meteorological from-direction |
| 5 | `drop_cmd` | rising edge triggers next cargo drop |
| 6 | `reset_cmd` | rising edge resets JSBSim |

## Output Vector

Width: 20

| Index | Signal | Unit |
| --- | --- | --- |
| 1 | `time` | s |
| 2 | `altitude_m` | m AGL |
| 3 | `vz_up_mps` | m/s |
| 4 | `airspeed_mps` | m/s |
| 5 | `groundspeed_mps` | m/s |
| 6 | `pitch_deg` | deg |
| 7 | `roll_deg` | deg |
| 8 | `heading_deg` | deg |
| 9 | `qbar_pa` | Pa |
| 10 | `mass_kg` | kg |
| 11 | `cg_x_m` | m |
| 12 | `pos_n_m` | m |
| 13 | `pos_e_m` | m |
| 14 | `elevator_cmd_norm` | normalized |
| 15 | `throttle_norm` | normalized |
| 16 | `wind_n_mps` | m/s |
| 17 | `wind_e_mps` | m/s |
| 18 | `drop_count` | count |
| 19 | `valid` | 1 = valid |
| 20 | reserved | - |

## Build

1. Ensure MATLAB uses a supported C++ compiler:

```matlab
mex -setup C++
```

2. Build with the bundled Windows x64 JSBSim development package:

```matlab
cd matlab/sfunc_jsbsim
build_sfun_airdropx_jsbsim
```

The repository also includes the full JSBSim source tree under
`third_party/JSBSim`. If the bundled `third_party/jsbsim-win64` library is
incompatible with another MATLAB/compiler setup, rebuild JSBSim from that
source tree:

```powershell
cd AirdropX/third_party
./build_jsbsim_win64.ps1
```

Then pass the resulting install root explicitly if needed:

```matlab
build_sfun_airdropx_jsbsim("C:\path\to\jsbsim\install")
```

## Notes

The wrapper intentionally keeps the Simulink interface narrow: one input vector and one output vector. After the MEX builds and the plant runs, the next step is to wrap outputs into a Bus Creator and feed them into the PD/ADRC, CARP, and drop-state-machine blocks.
