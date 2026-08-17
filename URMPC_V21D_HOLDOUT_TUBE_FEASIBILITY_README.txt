UR-MPC v2.1D — independent holdout + empirical tube feasibility

Purpose
-------
v2.1B/2.1A learned the final deltaA/deltaB from all trusted calibration samples.
Therefore the v2.1B projected 15/15 validation is useful for model-selection sanity,
but it is not an independent test of the final all-data correction.

v2.1D fixes that before any tube controller is deployed:

1) Run a second 45/50/55 x cfg0..4 nonlinear JSBSim experiment.
   - corrected v2.1B bank is frozen; NO refit.
   - same global amplitudes for every vertex.
   - different excitation order/sign/cross-combinations from v2.1A.
2) Build residual support ONLY from this independent holdout run.
   - p95/p99 and componentwise empirical max are reported.
   - the empirical max is a hard bound only for observed holdout samples.
     It is NOT advertised as a universal deterministic future bound.
3) Search one GLOBAL normalized ancillary-LQR R scale for the whole envelope.
   - Q = I in normalized state coordinates.
   - R = r_scale*I, same r_scale at every speed/cfg.
   - dlqr is scheduled on the known corrected A/B operating point.
4) Compute conservative local RPI box radii from the empirical W box.
5) Compute a stronger speed-common box for each cfg, valid under arbitrary
   switching among the 45:1:55 speed grid under the axis-aligned outer bound.
6) Audit MV tightening, model-trust-region occupancy, closed-loop poles and
   the soft rate-trust demand.

No flight-controller code is changed. No tube feedback/tightening is enabled.

Run
---
  .\run_urmpc_v21d_holdout_tube_feasibility_D_temp.ps1

Reuse an already completed v2.1D holdout:
  .\run_urmpc_v21d_holdout_tube_feasibility_D_temp.ps1 -FeasibilityOnly

Main outputs
------------
matlab/results/mpc_physics_v1/urmpc_v21d_holdout_validation/
  urmpc_v21d_holdout_manifest.csv
  V045_cfg0/... V055_cfg4/...

matlab/results/mpc_physics_v1/urmpc_v21d_tube_feasibility/
  urmpc_v21d_holdout_residual_support.csv
  urmpc_v21d_ancillary_search.csv
  urmpc_v21d_local_rpi.csv
  urmpc_v21d_speed_common_rpi.csv
  urmpc_v21d_global_switch_box.csv
  urmpc_v21d_summary.csv

Interpretation
--------------
empirical_tube_feasible=1 means:
  - independent 15/15 holdout data exist,
  - a single normalized ancillary design rule stabilizes all 55 points,
  - local empirical RPI boxes fit inside MV headroom and the state trust region,
  - and a conservative per-cfg speed-common box also remains feasible.

It still does NOT constitute a deterministic guarantee for unseen future
residuals, because W is estimated from finite flight data. A later integration
stage must explicitly choose the desired statistical/deterministic support
policy before enabling tube tightening in the real flight controller.
