AirdropX Physics MPC CL-1.5

Purpose
- Keep the validated v1.6 15/15 physics bank unchanged.
- Keep MPC Q/R, horizons, actuator limits, cfg scheduling, and fast-state safety gate unchanged.
- Fix only the outer altitude governor anti-windup structure.

Root cause from CL-1.4 V50 trace
- cfg2 MPC remained feasible/successful while altitude decayed.
- The old governor fed (actualVz - previousVzCmd) into the integral bias.
- Because actual vertical speed lagged the command, bias was driven to -1.2 m/s.
- At ~190.5 m altitude (about 9.5 m below target), vz_ref was still about -0.44 m/s (continued descent).
- MPC was only disabled later, at ~144.5 s, when |dVz| exceeded the 6 m/s fast-state gate.

CL-1.5 fix
- Integral: I += Ki * altitude_error * dt.
- raw = Kh * altitude_error + I.
- Apply own magnitude and slew limits.
- Back-calculate only with (realized governor output - raw governor output).
- actualVz is no longer used as an anti-windup realization signal.

Test first
  .\run_physics_mpc_fixed_stability_D_temp.ps1 -Speeds 50 -Workers 1

Output
  matlab/results/mpc_physics_v1/fixed_stability_cl15
