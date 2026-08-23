# Physics-MPC v0.8.2

## Fixed
- Corrected v0.8.1 `common_controller_pass` / `MixedHorizon` logic.
- The **actual receding-horizon MPC is now explicitly fixed to `Np=Nc=100`** for every H/V/cfg mission in this formal diagnostic envelope.
- Per-vertex `terminal.Np/Nc` values from the auto-horizon heuristic are preserved only as `Np_required/Nc_required` diagnostics; they no longer define controller identity and cannot block V65.
- Bank audit now reports both:
  - `mpc_horizon_fixed=1`, `Np=100`, `Nc=100`, `horizon_same=1` for the controller actually used;
  - `terminal_horizon_same` and `Np_required_range` for diagnostic auto-horizon variation.
- v0.8.2 automatically imports completed v0.8.1 speed slices and the 76 completed V45–V60 nonlinear missions. V65 v0.8.1 cases are not imported because they have no completion markers.

## Unchanged
- H=20:10:200 m, V=[45 50 55 60 65] m/s, cfg0..4.
- 0.2 s four-drop schedule [10.0 10.2 10.4 10.6] s.
- PreviewOnly; q-soft OFF.
- Q/R, state/input scales, hard input bounds, mission gates and Richardson thresholds are unchanged.
- The two Richardson certification failures remain certification FAIL; `usable=true` is only a diagnostic execution permission.
