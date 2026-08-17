AirdropX UR-MPC v2.0.3 residual/estimator audit
================================================

Purpose
-------
This patch intentionally changes NO control law, MPC weight, constraint,
horizon, scheduling rule, authority rule, or disturbance model.

The v2.0.2 V50 run proved that the rate-trust fix removes the cfg1 bang-bang
regression.  However cfg2 first settles after the 100 s payload transition,
then a slow growing instability appears roughly after 120 s and becomes
catastrophic BEFORE the 150 s cfg2->cfg3 switch.  Therefore it is not rigorous
to declare the remaining failure to be a cfg3 release impulse or to size a
tube set W before separating plant-model residual from estimator dynamics.

Added telemetry (read-only)
---------------------------
The controller trace is extended from 34 to 43 columns with:
  est_plant_[h,va,pitch,vz,q]
  est_disturbance_1
  est_disturbance_2
  est_disturbance_norm
  cfg_changed

The mpcstate fields are only read. They are never overwritten by this patch.

Added offline audit
-------------------
airdropx_urmpc_residual_audit.m reconstructs the exact previous-sample LPV
A/B/nominal, computes the raw one-step residual

  e(k+1) = x_meas(k+1) - [x_nom + A(x_meas(k)-x_nom) + B(u(k)-u_nom)]

and projects that residual onto span(B).  This answers two separate questions:
  1) when and in which state channel does the nonlinear/model residual grow?
  2) can the existing two load-disturbance directions B*d represent it?

Outputs in each Vxxx run folder:
  urmpc_one_step_residual.csv
  urmpc_residual_summary.csv
  urmpc_residual_windows_5s.csv

Run instruction
---------------
Use SkipBuild=True.  The existing v2.0.2 MPC bank must be reused so the V50
trajectory is a controller-identical repeat with additional diagnostics only.
Do NOT rebuild or retune for this audit run.
