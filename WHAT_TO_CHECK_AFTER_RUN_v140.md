# What to check after v1.4.0

Run the sensor-realistic base-equivalence audit first, then `calm`, `headwind_12`, and `sine_longitudinal` point tests before the full suite.

## Base-equivalence audit

Expected in calm:

- `pass=1`
- `wind_mpc_active_fraction=0`
- `disturbance_evidence_active_fraction=0`
- control/state/estimate deltas at floating-point level
- cfg/release-event/release-phase mismatch counts = 0

This audit is now stricter than v1.3.x because the same random sensor stream must also produce the same estimated state in both control branches.

## Calm

Check:

- `gate_realistic_avionics=1`
- `gate_independent_cargo_truth=1`
- `wind_mpc_active_fraction=0`
- `onboard_state_estimation_p95_normalized`
- `onboard_position_error_p95_m`
- final/tail stability
- mass/CG/Iyy gates
- landing RMS/max with the independent cargo plant

## headwind_12

Check:

- wind-estimator error and confidence;
- transient event/switch count remains clean;
- 0.5/1/3 s gust residual;
- throttle/elevator saturation;
- energy-recovery activity;
- final/tail settling;
- landing accuracy under sensor-derived release state.

The 3 s recovery gate may remain physically unattainable for a 12 m/s instantaneous step; do not hide that by using truth feedback.

## sine_longitudinal

Check:

- forced-response RMS/peak;
- no strong recovery latch;
- tail is zero-wind and base-solver only;
- final/tail stability;
- release accuracy with realistic avionics.

## A/B realism diagnostics

The point runner has two explicit non-formal ablations:

- `-IdealStateFeedback`: restores direct plant state feedback to quantify sensor/estimator cost.
- `-SharedCargoTruth`: restores the older scoring model to quantify model-circularity cost.

These are diagnostics only and should not be used as the formal v1.4.0 result.
