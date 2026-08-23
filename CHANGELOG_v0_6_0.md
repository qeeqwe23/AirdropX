# v0.6.0 changelog

- Added time-varying known-release preview MPC for the original 0.2 s four-drop schedule.
- Added affine trim-reference transition terms across cfg changes.
- Added stage-varying A/B, state trim and input trim across the prediction horizon.
- Added terminal P scheduling by the terminal cfg in the horizon.
- Added stage-specific delta-input bounds around the scheduled trim input while preserving the same absolute command bounds.
- Added optional q soft prediction constraints using the existing q state scale as the default limit.
- Added constant-schedule equivalence self-test against the already validated fixed-cfg condensed QP.
- Added transition and q-soft inequality self-tests.
- Added deterministic preview-QP precomputation before nonlinear flight so online timing measures QP solve, not matrix assembly.
- Added isolated MATLAB child-process execution and teardown watchdog inherited from the v0.5.2 lifecycle fix.
- Added reactive-vs-preview comparison output.
- Preserved Q/R, N=100, hard input bounds and mission acceptance thresholds.
