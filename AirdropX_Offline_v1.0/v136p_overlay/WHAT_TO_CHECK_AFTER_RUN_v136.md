# What to check after v1.3.6 runs

## 1. Base equivalence first

Run `run_base_equivalence_audit_v136_D.ps1` before the point/full suites.

Expected calm structural result:

- `wind_mpc_active_fraction = 0`;
- `disturbance_evidence_active_fraction = 0`;
- control/state/release deltas at numerical precision;
- mass/CG/Iyy gates pass.

## 2. calm

Confirm:

- `carrier_transient_evidence_active_fraction = 0`;
- `wind_mpc_active_fraction = 0`;
- `recovery_active_fraction = 0`;
- final/tail remain at base-controller levels;
- Iyy error remains zero/within strict tolerance.

## 3. headwind_12

The important v1.3.6 structural target is not merely landing error. Inspect:

- `carrier_transient_switch_count`;
- `wind_mpc_switch_count`;
- `last_carrier_transient_time_s`;
- `last_wind_mpc_active_time_s`;
- final/tail and N1/N2 settling;
- throttle saturation and 0.5/1/3 s gust residuals.

The real v1.3.5 trace showed long-lived evidence and many solver transitions. v1.3.6 should collapse this to one transient episode with a clean return to base MPC after the step/recovery phase.

A remaining failure of the strict 3 s recovery gate can still be a physical-authority limitation and should be interpreted together with actuator saturation.

## 4. sine_longitudinal

Expected setup:

- mission duration = 60 s;
- forcing end = 45 s;
- zero wind begins = 47 s;
- formal tail = 55--60 s.

Formal sine tail context must report:

- `tail5_zero_truth_wind_fraction = 1`;
- `tail5_base_solver_fraction = 1`.

Then evaluate the unchanged strict tail RMS <= 0.05 and final <= 0.10 gates.

## 5. Full suite

Only launch the full runner after base equivalence plus `calm`, `headwind_12`, and `sine_longitudinal` are structurally correct.
