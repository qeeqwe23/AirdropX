# What to check after v1.3.5

1. `base_equivalence_summary.txt`
   - `pass=1`
   - `wind_mpc_active_fraction=0`
   - `disturbance_evidence_active_fraction=0`
   - control/state deltas at numerical precision.
2. calm point
   - `recovery_active_fraction=0`
   - `wind_mpc_active_fraction` should be 0 or extremely close to 0; with the formal seed it is expected to be exactly 0 if the evidence latch behaves as designed.
   - final/tail should now match the legacy paired run rather than a rate-noise-driven disturbance QP.
3. mass configuration
   - `Iyy_match_error_max_kgm2 <= 0.01` even when a release phase leaves only one 1/120-s substep.
   - Oracle selftest reports `short_transition_Iyy_error_kgm2 <= 0.01` and `mass_refresh_converged=1`.
4. headwind_12
   - input boundary overflow must remain gone;
   - inspect post-gust, recovery time, 0.5/1/3 s residual, throttle saturation, energy shift.
5. sine
   - `recovery_active_fraction=0` unless an actual abrupt event is detected;
   - inspect `forced_response_primary_rms` / `forced_response_peak_normalized` during forcing;
   - inspect final/tail only after the 45-47 s smooth wind shutdown.
