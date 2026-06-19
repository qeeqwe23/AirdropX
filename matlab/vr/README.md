# AirdropX Simulink 3D Animation / VR Sink

This folder contains a minimal Simulink 3D Animation scene for visualizing:

- aircraft position and attitude
- four cargo boxes released by `drop_count`
- a ground plane
- a red target mark at `N = 1000 m`, `E = 0 m`

## Files

- `airdropx_scene.wrl`: VRML scene for VR Sink.
- `../airdropx_vr_aircraft_pose.m`: converts S-Function aircraft telemetry to `Aircraft.translation` and `Aircraft.rotation`.
- `../airdropx_vr_cargo_pose.m`: detects four `drop_count` rising edges and outputs cargo translations/scales.

## Required Toolbox

In MATLAB:

```matlab
setup_airdropx_simulink
which vrworld
open_system('sl3dlib')
```

If `open_system('sl3dlib')` opens the library, drag a `VR Sink` block into your model.

## AircraftVR MATLAB Function Block

Add a MATLAB Function block named `AircraftVR`:

```matlab
function [translation, rotation] = AircraftVR(pos_n_m, pos_e_m, altitude_m, roll_deg, pitch_deg, heading_deg)
%#codegen
[translation, rotation] = airdropx_vr_aircraft_pose(pos_n_m, pos_e_m, altitude_m, roll_deg, pitch_deg, heading_deg);
end
```

Connect S-Function outputs:

```text
pos_n_m     <- output 12
pos_e_m     <- output 13
altitude_m  <- output 2
roll_deg    <- output 7
pitch_deg   <- output 6
heading_deg <- output 8
```

## CargoVR MATLAB Function Block

Add a MATLAB Function block named `CargoVR`:

```matlab
function [cargo1_translation, cargo2_translation, cargo3_translation, cargo4_translation, cargo1_scale, cargo2_scale, cargo3_scale, cargo4_scale] = CargoVR(t, pos_n_m, pos_e_m, altitude_m, airspeed_mps, heading_deg, wind_n_mps, wind_e_mps, drop_count)
%#codegen

[cargo1_translation, cargo2_translation, cargo3_translation, cargo4_translation, ...
 cargo1_scale, cargo2_scale, cargo3_scale, cargo4_scale] = ...
    airdropx_vr_cargo_pose(t, pos_n_m, pos_e_m, altitude_m, airspeed_mps, heading_deg, wind_n_mps, wind_e_mps, drop_count);

end
```

Connect inputs:

```text
t            <- Clock
pos_n_m      <- output 12
pos_e_m      <- output 13
altitude_m   <- output 2
airspeed_mps <- output 4
heading_deg  <- output 8
wind_n_mps   <- output 16
wind_e_mps   <- output 17
drop_count   <- output 18
```

## VR Sink Setup

In the VR Sink block:

1. Set the world file to:

```text
matlab/vr/airdropx_scene.wrl
```

2. Select these fields:

```text
Aircraft.translation
Aircraft.rotation
Cargo1.translation
Cargo1.scale
Cargo2.translation
Cargo2.scale
Cargo3.translation
Cargo3.scale
Cargo4.translation
Cargo4.scale
```

3. Wire the corresponding outputs from `AircraftVR` and `CargoVR`.

Optional automatic chase camera:

```text
ChaseView.position
ChaseView.orientation
```

Add a MATLAB Function block:

```matlab
function [position, orientation] = ChaseViewVR(pos_n_m, pos_e_m, altitude_m, heading_deg)
%#codegen
[position, orientation] = airdropx_vr_chase_view(pos_n_m, pos_e_m, altitude_m, heading_deg);
end
```

Connect:

```text
pos_n_m      <- output 12
pos_e_m      <- output 13
altitude_m   <- output 2
heading_deg  <- output 8
```

Then wire `position` and `orientation` to the matching VR Sink ports.

## Coordinate Convention

The scene uses:

```text
VR X = East
VR Y = Up
VR Z = -North
```

So a target at `N = 1000 m, E = 0 m` appears at:

```text
translation 0 0 -1000
```

## Notes

- VR visualization is an observer only; it should not feed back into the controller.
- If the aircraft nose points sideways or backwards, adjust the initial model orientation or the sign of `heading_deg` in `airdropx_vr_aircraft_pose.m`.
- The cargo blocks appear only after `drop_count` increases. Before release, their scale is `[0 0 0]`.
