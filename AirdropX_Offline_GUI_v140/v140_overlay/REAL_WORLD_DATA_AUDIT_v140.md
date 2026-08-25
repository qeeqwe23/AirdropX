# AirdropX Physics-MPC v1.4.0 — Real-world data audit

## Purpose

v1.3.x used a persistent nonlinear JSBSim plant correctly for dynamics, but the mission loop still consumed several JSBSim truth states directly. That is acceptable for an ideal-state-feedback SIL benchmark, not for a claim that the same controller inputs exist on a real aircraft.

v1.4.0 therefore separates **plant truth**, **simulated onboard sensors/state estimates**, and **offline scoring truth**. The formal v1.4.0 path defaults to sensor-realistic avionics and an independent cargo scoring plant.

## 1. Data that must NOT be read directly from JSBSim by the onboard controller

| Quantity | v1.3.x source | v1.4.0 onboard replacement | Real-aircraft counterpart |
|---|---|---|---|
| Along-track position | `D.pos_n_m` | noisy GNSS position + causal navigation filter | GNSS/INS NED/ENU position |
| Ground speed | `D.v_north_mps` | noisy GNSS velocity + filter | GNSS/INS velocity |
| Vertical speed | `D.vz_up_mps` | noisy GNSS vertical velocity + filter | GNSS/INS vertical velocity / fused VSI |
| Altitude `h` | JSBSim state | barometric altitude + causal vertical prediction | baro + GNSS/INS fusion |
| Airspeed `Va` | JSBSim state | pitot/air-data measurement + bias/noise/filter | pitot/static ADC |
| Flight-path angle `gamma` | JSBSim state | derived causally from estimated GNSS velocity vector | GNSS/INS velocity vector |
| Pitch `theta` | JSBSim state | AHRS measurement + filter | AHRS/INS attitude |
| Pitch rate `q` | JSBSim state | gyro measurement + filter | IMU gyro |
| Engine `N1/N2` | JSBSim state | engine telemetry noise/filter | FADEC/ECU/tachometer telemetry; observer if unavailable |
| Longitudinal wind | JSBSim truth | existing causal air/ground-speed wind estimator | ground velocity minus air-data vector |

The controller and release scheduler now receive `x_est`, estimated position, estimated ground/vertical speed and estimated wind. JSBSim truth is only the input stimulus to simulated sensors and is retained separately for scoring.

## 2. Data that may remain JSBSim truth in simulation, but must remain scoring-only

These signals are deliberately allowed in the SIL harness because they describe the plant/environment, not information available to the flight controller:

- exact carrier state and position for post-run error scoring;
- exact scripted wind for plant forcing and post-run wind-estimator error scoring;
- exact JSBSim mass, CG and Iyy for verifying that a commanded payload configuration was actually applied;
- exact fractional-release plant state for scoring the release-state estimation error;
- exact gust onset from the scripted wind profile for simulation certification plots/metrics.

None of the above may feed the formal onboard MPC or release decision in v1.4.0.

For a real flight test, the corresponding independent scoring references would need survey-grade/RTK navigation, calibrated air-data/meteorological instrumentation, payload/fuel mass-property records, and independent payload landing measurement.

## 3. Mass, CG and inertia: do not treat JSBSim truth as an onboard sensor

A real aircraft does not normally measure `mass`, `CG` and `Iyy` every 0.1 s. The deployable equivalent is a scheduled model based on:

1. pre-flight payload manifest and geometry;
2. fuel quantity / fuel-flow estimator;
3. positive release confirmation for each cargo item;
4. a configuration-to-mass/CG/inertia table or identified model.

The current mission already selects `cfg0..cfg4` from release events rather than reading JSBSim mass properties. JSBSim `mass/cg/Iyy` remain only formal simulation checks. Before hardware use, `FuelScale` should come from the aircraft fuel-quantity/flow system rather than a test-script constant.

## 4. Cargo landing truth was too circular in v1.3.x

The onboard release predictor and the previous scoring "truth" shared essentially the same ballistic structure. A predictor can look artificially accurate when it is judged by a near-copy of itself.

v1.4.0 adds `airdropx_cargo_truth_plant_v140` for scoring only. It intentionally differs from the guidance model:

- 2-D vector quadratic drag;
- vertical drag;
- altitude-varying air density;
- its own numerical integration;
- only one shared empirical calibration point: 20 m altitude, 78.6 m/s, 150.7649 m range.

This is still a simulation plant, not physical truth. In real testing, replace landing truth with payload RTK-GNSS/UWB beacon, ground camera photogrammetry, radar, or surveyed impact location.

## 5. Offline models that are still JSBSim-derived

Two important items are intentionally still JSBSim-derived because v1.4.0 is a SIL package:

- `physics_bank.mat`: trim + A/B model bank;
- `wind_disturbance_model_v130.mat`: identified `Gw` wind-increment disturbance map.

This is not an online truth leak. However, it means the controller is not yet flight-model-certified. For a real aircraft, keep the same file/interface format but replace or validate these data using one or more of:

- flight-test system identification from synchronized sensor/actuator logs;
- ground/engine tests for propulsion states;
- CFD/wind-tunnel aerodynamic derivatives;
- manufacturer aerodynamic/propulsion data;
- LPV interpolation fitted to flight-test operating points.

v1.4.0 also includes `matlab/phys_mpc/airdropx_phys_mpc_wind_disturbance_fit_flightlog_v140.m`. It fits a loader-compatible `Gw` directly from synchronized sensor-estimated state, actuator-command and estimated-wind flight logs; it does not require JSBSim truth. This provides a concrete replacement path for `wind_disturbance_model_v130.mat` once real flight data exist.

The recommended transition is **JSBSim prior -> flight-test residual identification -> validated flight model**, not to discard the physics bank entirely.

## 6. Signals that are legitimately known onboard

These do not need a substitute merely because they exist in the simulator:

- commanded elevator/throttle;
- target coordinates from the mission plan;
- controller sample time;
- commanded cargo release event;
- current configuration after a positively confirmed release;
- trim/model tables stored onboard;
- sensor calibration constants.

For hardware, a release command should be paired with an electrical/mechanical release-confirmation input before changing `cfg`.

## 7. Remaining realism gaps after v1.4.0

v1.4.0 removes the major ideal-state-feedback leak, but it is not a full HIL avionics model. Remaining work before a physical flight claim includes:

- sensor update-rate, latency, dropout and time-synchronization models;
- correlated GNSS/INS errors rather than only scalar white-noise/bias models;
- pitot position error and angle-of-attack/sideslip effects;
- wind shear/turbulence varying with altitude and position, not only a uniform longitudinal profile;
- actuator servo/saturation/rate validation against the real aircraft;
- fuel-estimation uncertainty and release-confirmation faults;
- flight-test replacement/validation of A/B/Gw;
- independent measured cargo impact truth.

These are deliberately documented rather than hidden behind a "realistic" label.
