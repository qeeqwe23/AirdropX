# What to check after a v1.3.3 run

Priority order:

1. Infrastructure completeness: `all_three_way_missions_found=24/24`; `headwind_12_wind_mpc_aware` must not fail with `throttle must be in [0,1]`.
2. Calm false recovery: `abrupt_gust_trigger_count=0`; strong recovery should be effectively absent; compare final/tail against legacy calm.
3. Sine persistence: `recovery_active_fraction` should be much lower than v1.3.2's ~0.86 and strong gust latch should not remain active across smooth periodic wind.
4. Strong steps: compare 0.5/1/3 s residuals and recovery time against legacy for `headwind_12` and `step_bidirectional`.
5. Energy recovery: inspect `energy_altitude_shift_peak_m` and `energy_recovery_active_fraction`; energy shift should stay within ±2.5 m and occur only when throttle is at/near its limit during abrupt-gust recovery.
6. Numerical snap: `input_snap_max` should be tiny (floating-point scale). A material snap indicates a real control-bound bug and should not be accepted.
7. Release accuracy: verify the v1.3.2 gains are retained; do not retune the release model unless landing accuracy actually regresses.
8. Realtime: QP p95 must remain below the existing 100 ms gate; energy second-pass solves should only occur in throttle-limited recovery windows.

The formal thresholds were not loosened in v1.3.3.
