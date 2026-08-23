# CHANGELOG — Physics-MPC v1.3.6-Paper v1.0

- Preserved v1.3.6 carrier controller, transient wind evidence, energy-aware gust recovery,
  fractional release, mass-refresh Oracle and sine settling semantics.
- Added unbiased paper sensor layer: GNSS/INS, baro, pitot, AHRS/IMU and engine telemetry.
- Formal MPC now consumes `x_est`; formal release scheduling consumes estimated position,
  height, ground/vertical speed and causal wind estimate.
- JSBSim exact state/navigation/wind remains scoring-only.
- Removed fixed sensor biases, random walks, latency and dropout from the main paper baseline.
- Added ideal-state-feedback ablation, but it is not the formal paper path.
- Nominal v1.3.6 cargo scoring remains the main carrier-MPC paper metric; independent 2-D
  cargo scoring is an explicit sensitivity ablation.
- Split `release_target_residual_m` from actual `impact_prediction_error_m`.
- Split `paper_core_pass` from legacy strict `engineering_pass` so extreme-stress thresholds
  do not replace quantitative baseline comparison.
- Full runner uses common deterministic seeds for all controller comparisons and max 3 workers.
- Fixed Oracle marker behavior: a missing marker runs selftest; it no longer forces an unnecessary MEX rebuild.
- MATLAB child windows remain hidden by default.
