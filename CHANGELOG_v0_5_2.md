# v0.5.2 changelog

- Disabled v0.5.1 in-process sequential JSBSim scenario comparison.
- Added isolated MATLAB child-process orchestration for every nonlinear JSBSim scenario.
- Added child-PID-only watchdog; unrelated MATLAB processes are never terminated.
- Added durable completion/status markers before process teardown.
- Added `CloseOracleOnReturn` to the mission function so dedicated scenario processes can save/return before MEX teardown.
- Added fresh-process cfg0->cfg4 direct jump probe before the simultaneous mission.
- Added two identical cfg4 evaluations in the probe to verify repeatability and rule in/out the simultaneous mass-jump path directly.
- Added clean finalizer process that does not initialize JSBSim.
- Reuses an existing completed v0.5.1 interval_2s result by default.
- Physics/controller design, Q/R, horizon, bounds, gates, Oracle MEX and S-Function are unchanged.
