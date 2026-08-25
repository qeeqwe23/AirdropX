# v1.1.2

- Fixed confirmed false `AirdropX:Wind:MissingSimulink` infrastructure failure.
- Removed `exist("sim","file")~=2` guard.
- Added robust Simulink capability probe: installation metadata, license test, and actual `load_system`.
- Added `SIMULINK_PREFLIGHT_START/OK` status markers.
- Retained process-private IC files and serial calm preflight from v1.1.1.
- Estimator equations, Kalman tuning, Monte Carlo settings, wind profiles and formal gates are unchanged.
