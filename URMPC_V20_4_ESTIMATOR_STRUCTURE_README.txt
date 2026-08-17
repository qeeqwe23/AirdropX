AirdropX UR-MPC v2.0.4 — estimator structure correction
=========================================================

Reason
------
v2.0.3 recorded only the first two disturbance states plus the norm of the
entire mpcstate.Disturbance vector.  The V50 audit showed that the unrecorded
tail dominated the wind-up.  Example:
  140-145 s: first-two p95 norm ~0.41, total p95 norm ~3.21
  145-150 s: first-two p95 norm ~1.22, total p95 norm ~6.82

The controller plant explicitly declares two unmeasured load inputs (B*d),
but the previous build never disabled the MPC Toolbox default output
disturbance model.  MATLAB may therefore append integrated disturbance
states on measured outputs when observable.  That silently violated the
intended "two integrated load disturbances only" architecture.

Change
------
1) setindist(ur_mpc,'integrators') explicitly retains the two B-load UDs.
2) setoutdist(... zero static model ...) explicitly removes all output
   disturbance integrators.
3) Build and preflight require exactly two disturbance states and zero
   output-integrator channels.
4) urmpc_estimator_design.csv records the resulting estimator structure.
5) Runtime trace adds est_disturbance_tail_norm and
   est_disturbance_state_count.  A correct v2.0.4 bank should report
   state_count=2 and tail_norm=0 throughout.

Unchanged
---------
- ONE adaptive MPC object
- 5 states [H Va pitch vz q]
- LPV A/B interpolation and moving nominal
- Np / ControlHorizon
- Bryson scales and all weights
- physical MV hard bounds
- certified soft MV-rate trust limits
- no recovery / TECS / H-PI / cfg-specific tuning
- the two load disturbance channels remain integrated (no leak added yet)

Run
---
Estimator structure is part of the MPC object, so rebuild:
  SkipBuild=False

Acceptance sequence
-------------------
1) build 55/55 vertex PASS
2) 12/12 linear transition PASS
3) urmpc_estimator_design.csv:
     input_disturbance_states  = 2
     output_disturbance_states = 0
4) V50 trace:
     est_disturbance_state_count = 2
     est_disturbance_tail_norm = 0
5) cfg0/cfg1 must not regress.
6) Compare cfg2 120-150 s residual/estimator growth with v2.0.3.

This version intentionally does NOT add tube MPC or a leaky disturbance
observer.  It first removes the unintended hidden estimator dynamics so the
next experiment isolates the actual two-load observer.
