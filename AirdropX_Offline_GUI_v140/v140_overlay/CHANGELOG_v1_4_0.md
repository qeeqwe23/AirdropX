# CHANGELOG — Physics-MPC v1.4.0

## Sensor-realistic onboard data path

- Removed formal direct JSBSim state feedback from the MPC/release path.
- Added `matlab/avionics/airdropx_avionics_init_v140.m` and `airdropx_avionics_step_v140.m`.
- Formal default state sources now emulate GNSS/INS, barometer, pitot/air-data, AHRS/IMU and engine N1/N2 telemetry.
- `gamma` is derived from the estimated navigation velocity vector, rather than read from JSBSim.
- Wind estimation continues to use only causal air/ground/vertical-speed measurements.
- Release position/height/velocity now come from the onboard estimate, not `D.pos_n_m` or the JSBSim state.
- Added explicit `-IdealStateFeedback` point-runner ablation; it is not formal hardware-realistic evidence.

## Independent cargo scoring plant

- Added `airdropx_cargo_truth_params_v140.m` and `airdropx_cargo_truth_plant_v140.m`.
- Formal landing error defaults to this independent 2-D vector-drag plant rather than the guidance predictor's companion truth model.
- The independent plant includes vertical drag and altitude-varying density and shares only the historical calibration datum.
- Added `-SharedCargoTruth` point-runner ablation; formal v1.4.0 requires the independent scoring plant.

## Truth isolation and diagnostics

- JSBSim carrier/wind/mass/CG/Iyy truth remains available only for plant propagation and post-run scoring.
- Added truth-vs-estimate timeseries columns and p95 onboard state/navigation estimation error metrics.
- Added formal gates requiring realistic avionics and independent cargo scoring.
- Added v1.4.0 same-seed base-equivalence audit comparing both truth and estimated states.
- Added static truth-isolation audit and independent cargo numerical audit.
- Added `airdropx_phys_mpc_wind_disturbance_fit_flightlog_v140.m` so the deployed `Gw` can later be replaced by a fit from real sensor/actuator/wind-estimator logs rather than JSBSim truth.

## Preserved v1.3.6 behavior

- transient-vs-absolute-wind evidence split;
- energy-aware strong-gust recovery;
- fractional release timing;
- v1.3.5 mass-refresh JSBSim Oracle;
- sine zero-wind settling window;
- hidden MATLAB child windows by default;
- maximum three parallel mission processes.

## Important interpretation change

Landing errors from v1.4.0 are intentionally not directly comparable with v1.3.6: both the release input state and the cargo scoring plant are now less idealized. An increase in error can be a realism correction rather than a controller regression.
