# v1.1.1

- Added a longitudinal-only wind measurement model using groundspeed, TAS and vertical speed.
- Added a two-state Kalman wind/wind-rate observer.
- Added innovation-based fast re-acquisition for true wind steps.
- Added a persistent 10 Hz runtime wrapper.
- Added a dedicated temporary Simulink/JSBSim validation harness; no existing `.slx` is modified.
- Added eight headwind/tailwind truth scenarios.
- Added 50-seed sensor-noise Monte Carlo scoring per scenario.
- JSBSim wind truth channels are isolated to scoring and are never estimator inputs.
- No Physics-MPC Q/R/horizon/gates were modified.
- No C++/MEX files are included or replaced.
