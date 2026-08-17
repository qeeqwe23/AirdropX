AirdropX Physics MPC CL-1 — Fixed H/V closed-loop stability certification
=======================================================================
Purpose
-------
This stage starts CLOSED-LOOP MPC validation. It does not modify trim, rebuild the
physics bank, tune Q/R, run BayesOpt, evaluate CARP, or run arbitrary H/V commands.

Test design
-----------
- 3 fixed-speed flights: 45, 50, 55 m/s, target altitude 200 m.
- Each flight begins in real cfg0 and performs four REAL payload releases:
  cfg0 -> cfg1 -> cfg2 -> cfg3 -> cfg4.
- Drop times: 50, 100, 150, 200 s. Stop: 255 s.
- Each cfg is scored only after a 15 s post-transition settling exclusion.
- Three speeds can run with 3 process workers.
- Parallel isolation: unique IC XML, unique model name, unique Simulink cache/codegen.

Formal steady-state gates (diagnostic, never auto-tuned)
--------------------------------------------------------
H RMS <= 1.5 m; max |H error| <= 3.0 m
Va RMS <= 0.75 m/s; max |Va error| <= 1.5 m/s
vz RMS <= 0.25 m/s
q RMS <= 0.25 deg/s (uses JSBSim q when available)
pitch std <= 0.60 deg
|H slope| <= 0.10 m/s
|Va slope| <= 0.05 m/s^2
|pitch slope| <= 0.05 deg/s
actuator saturation fraction <= 1%
MPC fail increment == 0 in the cfg segment

Hard-safety gates are separate and deliberately much looser. A formal failure does
not abort the other cfg/speed scans.

Run
---
1) Keep the completed v1.6 bank:
   matlab/results/mpc_physics_v1/airdropx_physics_mpc_bank.mat
2) Run:
   .\run_physics_mpc_fixed_stability_D_temp.ps1
3) Monitor if desired:
   .\check_physics_mpc_fixed_stability.ps1

Main outputs
------------
matlab/results/mpc_physics_v1/fixed_stability_cl1/fixed_stability_summary.csv
matlab/results/mpc_physics_v1/fixed_stability_cl1/fixed_stability_failures.csv
matlab/results/mpc_physics_v1/fixed_stability_cl1/fixed_stability_run_errors.csv
and per-speed V045/V050/V055 directories containing full closed-loop timeseries and
controller traces.

Important
---------
Do NOT run the dynamic H/V validation yet. CL-1 must first show which fixed-speed,
post-drop cfg segments are stable. Any later controller changes must be global and
physics-based, not cfg-specific patches.
