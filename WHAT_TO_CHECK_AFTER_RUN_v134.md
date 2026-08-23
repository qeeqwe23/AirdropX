# v1.3.4 three-point acceptance focus

Run calm, headwind_12 and sine_longitudinal first.

- calm: abrupt_gust_trigger_count = 0, recovery_active_fraction should be 0, wind-MPC control should fall back to base solver, final/tail should return close to legacy.
- headwind_12: strong recovery + energy exchange should remain active only after the abrupt gust; inspect recovery time, 0.5/1/3 s residuals, throttle saturation and elevator headroom.
- sine_longitudinal: abrupt_gust_trigger_count = 0 and recovery_active_fraction should be 0 by default; Gw preview should retain continuous-wind benefit without recovery-bank tail degradation.
- Iyy: inspect Iyy_reference_bias_kgm2 and Iyy_match_error_max_kgm2; the former may be a fixed provenance offset, while the latter must remain small across cfg transitions.
- plotting: no title-interpreter underscore warning should appear.
