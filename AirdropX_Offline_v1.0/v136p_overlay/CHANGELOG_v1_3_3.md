# v1.3.3

- Strong gust recovery now requires confidence + an abrupt estimated-wind jump; `step_detected` or wind-rate evidence alone can no longer latch full recovery.
- Recovery unlatches from recovered Va/height state after a quiet hold; it no longer depends on low wind-rate confidence.
- Ramp/sine wind can use only a weak continuous recovery assist (default cap 0.25); physical `Gw` preview remains the primary continuous-wind mechanism.
- Unified recovery Q/R multipliers are softened, especially throttle effort, to reduce prolonged saturation/overshoot.
- Adds bounded energy-aware recovery (default ±2.5 m temporary altitude reference shift) only when an abrupt-gust recovery is throttle-limited.
- Adds normalized disturbance deadband and exact legacy-solver fallback in near-calm conditions.
- Adds floating-point-only physical input snapping before JSBSim; true bound violations still error.
- Adds `abrupt_gust_trigger_count`, energy-recovery activity/shift, and input-snap diagnostics.
- Full and point PowerShell runners hide MATLAB child windows by default; `-ShowChildWindows` opts back into visible child windows.
- Retains v1.3.2 confidence-gated precision release, v1.3.1 fractional release, v1.3.0 JSBSim-calibrated `Gw`, and the frozen v1.1.3 wind estimator.
- Formal carrier thresholds are not loosened.
