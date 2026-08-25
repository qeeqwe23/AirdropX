# AirdropX Physics-MPC v1.4.0 — sensor-realistic validation

v1.4.0 is the first package in this line that treats JSBSim as the **plant**, not as an ideal onboard sensor.

The v1.3.6 carrier/gust/release logic is retained, but the formal online path now consumes sensor-derived state/navigation estimates and is scored by an independent cargo model.

## Why the version jumps to 1.4.0

This is an architecture/validation change rather than another recovery-weight tuning pass. Previous versions could use exact JSBSim state/position in the MPC or release chain. A real aircraft cannot read `h, Va, gamma, theta, q, N1, N2, position` as simulator truth.

The formal v1.4.0 path is:

```text
persistent nonlinear JSBSim plant truth
              |
              v
    simulated physical sensors
 GNSS/INS + baro + pitot + AHRS/IMU
          + engine telemetry
              |
              v
       causal state estimate
              |
       +------+------+
       |             |
       v             v
 Physics-MPC     wind estimator
                       |
                       v
                 release guidance
```

Exact JSBSim values are retained only for plant propagation and post-run scoring.

## Real-world substitutes now used by the formal path

- along-track position / ground speed / vertical speed: GNSS/INS-style measurements;
- altitude: barometric measurement with navigation prediction;
- airspeed: pitot/air-data measurement;
- pitch: AHRS/INS;
- pitch rate: IMU gyro;
- `gamma`: derived from the estimated navigation velocity vector;
- N1/N2: engine ECU/FADEC/tach-style telemetry;
- longitudinal wind: existing causal air-data vs ground-velocity estimator.

The sensor model includes white noise, fixed run-to-run biases, slow bias random walks and causal filters. It is intentionally a first SIL avionics layer, not yet a complete HIL model with every latency/dropout/correlation effect.

## Release guidance no longer sees exact JSBSim navigation state

The fractional release scheduler now uses:

```text
estimated along-track position
estimated altitude
filtered GNSS ground/vertical velocity
estimated wind and wind-rate
```

For a fractional release inside the 0.1 s MPC hold, no fictitious extra sensor sample is created. The release state is causally extrapolated from the last available onboard estimate.

Truth release position/state remains in the log only to score estimation and landing error.

## Independent cargo scoring plant

v1.3.x judged the release predictor with a closely related ballistic truth function. v1.4.0 defaults to a separate scoring-only cargo plant:

- 2-D vector quadratic drag;
- vertical drag;
- altitude-varying air density;
- 5 ms integration;
- same single historical calibration datum only.

This deliberately introduces structural model mismatch. It is still a simulation scoring plant; real flight testing should use measured payload impact truth (RTK-GNSS/UWB/camera/radar/survey).

Use `-SharedCargoTruth` only as a non-formal ablation.

## What JSBSim is still allowed to provide

JSBSim remains the nonlinear aircraft plant. Therefore the harness still knows exact truth for:

- carrier state/position;
- scripted environment wind;
- mass/CG/Iyy;
- exact release-time plant state.

Those values are permitted for **simulation scoring only**, not the formal controller/release path.

See `REAL_WORLD_DATA_AUDIT_v140.md` for the complete classification.

## Offline model caveat

`physics_bank.mat` (trim/A/B) and `wind_disturbance_model_v130.mat` (`Gw`) are still JSBSim-derived. That is not an online truth leak, but it does mean this remains SIL rather than flight-model certification.

For a real aircraft, preserve the same interfaces but replace/validate these models with flight-test system identification, CFD/wind-tunnel/manufacturer data, and propulsion tests. v1.4.0 includes `airdropx_phys_mpc_wind_disturbance_fit_flightlog_v140.m` to fit a v1.3.0-loader-compatible `Gw` from synchronized onboard estimated-state/wind/control logs without JSBSim truth.

## Preserved v1.3.6 behavior

- absolute-wind evidence for release is separate from transient-gust evidence for carrier MPC;
- constant wind can settle back to exact base Physics-MPC;
- event-gated energy recovery remains available for strong gusts;
- fractional release timing remains enabled;
- the v1.3.5 mass-refresh Oracle is reused;
- sine uses a genuine zero-wind settling tail;
- MATLAB child windows are hidden by default;
- full runner uses at most 3 parallel mission processes.

## Install

```powershell
.\install_wind_disturbance_airdrop_v140.ps1
```

## Recommended run order

```powershell
cd 'D:\vscode project\AirdropX'

.\run_base_equivalence_audit_v140_D.ps1

.\run_wind_disturbance_airdrop_point_v140_D.ps1 -Scenario calm
.\run_wind_disturbance_airdrop_point_v140_D.ps1 -Scenario headwind_12
.\run_wind_disturbance_airdrop_point_v140_D.ps1 -Scenario sine_longitudinal
```

Only after those look structurally correct:

```powershell
.\run_wind_disturbance_airdrop_v140_D.ps1
```

## Useful realism ablations

Direct ideal state feedback (diagnostic only):

```powershell
.\run_wind_disturbance_airdrop_point_v140_D.ps1 -Scenario calm -IdealStateFeedback
```

Older shared cargo scoring model (diagnostic only):

```powershell
.\run_wind_disturbance_airdrop_point_v140_D.ps1 -Scenario calm -SharedCargoTruth
```

Do not use either ablation as the formal v1.4.0 result.

## Result roots

Base equivalence:

`matlab\results\physics_mpc_v140_sensor_base_equivalence_audit\`

Point tests:

`matlab\results\physics_mpc_v140_sensor_realistic_point\`

Full validation:

`matlab\results\physics_mpc_v140_sensor_realistic_airdrop_validation\`

## Interpretation warning

v1.4.0 landing errors are intentionally **not directly comparable with v1.3.6**. The controller/release system now receives imperfect estimated state and the landing score uses an intentionally different cargo plant. If error grows, first determine whether it is sensor/state-estimation sensitivity, release-model mismatch, or a real controller regression.
