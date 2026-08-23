# v1.3.3 rationale from the completed v1.3.2 run

The v1.3.2 run established the following implementation facts used to choose v1.3.3 changes:

- 23/24 three-way missions were found; the missing mission was `headwind_12_wind_mpc_aware`, which stopped because the Oracle rejected throttle outside `[0,1]` at the numerical boundary.
- Wind-MPC worst landing error was 2.2141 m; calm fractional release RMS was 1.1233 m, so release guidance is retained rather than retuned.
- Overall median legacy/wind-MPC carrier RMS ratio improved to about 1.0584, so physical `Gw` remains valuable.
- Calm wind-MPC recovery was active about 9.3% despite near-zero mean wind confidence, and calm final/tail metrics regressed badly relative to legacy. This motivates confidence + abrupt-delta recovery triggering and a normalized calm disturbance deadband.
- Sine wind showed strong RMS improvement but recovery remained active about 86% and throttle saturation reached about 39%, with very poor final/tail metrics. This motivates weak continuous assist plus state-based recovery unlatching.
- Maximum throttle headroom reached zero while elevator retained substantial headroom. This motivates bounded altitude/airspeed energy exchange rather than further lowering throttle penalty.
- Formal carrier thresholds remain unchanged in v1.3.3.
