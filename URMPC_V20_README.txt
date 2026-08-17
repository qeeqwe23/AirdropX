AirdropX Physics UR-MPC v2.0 — ONE unified adaptive/robust MPC
================================================================

v2.0 state/nominal consistency fix (2026-08-17)
--------------------------------------------------
The original dense preflight created every mpcstate at the controller-design
anchor (V50/cfg2) and changed only LastMove.  mpcstate.Plant is an ABSOLUTE
engineering state, so the first adaptive-MPC estimator update at all other
vertices started from the wrong operating point.  This produced the observed
nonzero nominal elevator/throttle while V50/cfg2 alone stayed at zero.

This patch initializes mpcstate.Plant at each moving nominal for independent
preflight cases, uses the absolute old-cfg state in transition certification,
and synchronizes the runtime estimator once on the first valid telemetry
frame.  It also writes design/model audit CSV files before certification so a
failed build still leaves urmpc_design_parameters.csv for diagnosis.

Purpose
-------
This branch deliberately stops the CL-1.x pattern of adding recovery/TECS
controllers around a weak MPC. UR-MPC v2.0 has exactly one control law at
runtime: one Model Predictive Controller object. Speed and payload cfg only
update that controller's prediction model and nominal operating point.

Nothing in this package rebuilds trim/ID or recompiles the JSBSim MEX.
It consumes the already-certified v1.6 15/15 physics bank.

Architecture
------------
Measured/estimated state used by the MPC:
    x = [ h, Va, pitch, vz, q ]

Physical manipulated variables:
    u = [ physical elevator, throttle ]

For every certified 4-state vertex x4=[Va,pitch,vz,q], the altitude state is
added exactly as:
    h(k+1) = h(k) + Ts*vz(k)

The 5-state model is therefore:
    x(k+1) = A(V,cfg) x(k) + B(V,cfg) u(k) + B(V,cfg) d(k)

where d is a pair of UNMEASURED LOAD DISTURBANCES handled by the MPC's own
state/disturbance estimator. This is not a second controller. It gives the
single MPC a physically meaningful way to estimate persistent trim/model
errors after a 300 kg release.

At runtime:
  1. cfg comes from the known payload mass progression.
  2. A/B/trim are interpolated between the two neighboring speed vertices.
  3. ONE mpcmoveAdaptive call solves ONE QP.
  4. The physical elevator command is converted once to the legacy MEX
     external elevator delta using the hidden trim that was latched at reset.

There is NO:
  - H->vz PI governor
  - recovery controller
  - TECS overlay
  - cfg-specific Q/R
  - speed-node command blending after two separate QPs
  - auto learning / BayesOpt

Why the old normal MPC was structurally weak
---------------------------------------------
The v1.x/CL-1.x controller had all of the following at the same time:
  * altitude outside the MPC;
  * output disturbance estimation explicitly disabled;
  * a tiny local MV deviation box (+/-0.10 elevator, +/-0.18 throttle);
  * one QP per speed node followed by physical-command interpolation;
  * opaque effective weighting (very strong vz, almost no pitch penalty).

CL-1.7 proved the QP could remain feasible while cfg2 slowly diverged before
any recovery trigger. UR-MPC v2.0 fixes the optimization problem itself.

Parameter derivation
--------------------
Ts = 0.1 s is inherited from the certified physics identification/model bank.

Prediction horizon:
  For each certified A matrix, finite non-neutral modal time constants are
  computed as tau = -Ts/log(|lambda|). Nearly neutral phugoid/integrator-like
  modes are excluded from horizon selection because altitude is now an
  explicit state and arbitrarily long local-linear prediction is not credible.
  tau_design is the 90th percentile across the 15 vertices.
  prediction_time = clamp(3*tau_design, 4 s, 8 s)
  Np = max(30, ceil(prediction_time/Ts)).

Control horizon:
  deterministic blocked moves [1 1 1 2 3 5 8 ...remainder]. This preserves
  fast authority immediately after a drop while reducing long-horizon degrees
  of freedom. There is one rule for the full envelope.

Output normalization (Bryson-style engineering limits):
  h       3.0 m
  Va      1.5 m/s
  pitch   6.0 deg  (pitch is natural trim, not a user-commanded angle)
  vz      0.50 m/s
  q       0.50 deg/s

All normalized output weights are exactly 1. Relative priority therefore comes
from transparent physical scales, not hidden cfg-specific weights.

MV absolute weight is 0. Move-rate weight is a small common 0.05 numerical
regularizer. It is NOT claimed to be a physical constant.

Physical / trustworthy MV constraints
--------------------------------------------
The controller does NOT blindly give a local linear model the entire physical
actuator range. One common design envelope is derived automatically from the
certified 15-point trim manifold:

  nominal hull = [min(all 15 trim MVs), max(all 15 trim MVs)]
  margin = max(largest adjacent cfg/speed trim jump, source ID excitation)
  design envelope = nominal hull +/- margin

This has a direct engineering meaning: the MPC can reach every certified trim
and has one additional worst observed operating-point transition of authority,
while not pretending the local A/B model is valid to full-control extremes.
The resulting numeric bounds and margins are stored in ur_meta.design_mv_bounds
and ur_meta.design_mv_margin for auditability.

The design envelope is then intersected online with the exact legacy MEX
physical reachability. Overall software limits remain throttle [0,1] and
physical elevator [-0.95,+0.95]. Since the MEX accepts external elevator delta
in [-0.85,+0.85] and adds the reset-latched hidden trim h0:
    physical_emin = max(-0.95, h0 - 0.85)
    physical_emax = min(+0.95, h0 + 0.85)

Actuator-rate treatment:
The project MQ9_Reaper/Controls.xml uses a direct aerosurface_scale for elevator
and a pure_gain for throttle. There is no actuator lag/rate element in the
current JSBSim plant. Therefore UR-MPC v2.0 does NOT impose the legacy artificial
hard rate caps (0.012 elevator / 0.020 throttle per 0.1 s) by default. Smoothness
is provided by the common move-rate cost only. This makes the controller match
the plant actually being simulated instead of inventing actuator physics.

If a real actuator datasheet/model is added later, set the physical dynamics or
rate constraints from that data. Do not tune a fake rate merely to pass tests.

Offline certification before flight
-----------------------------------
airdropx_urmpc_build performs:
  1. one-controller API/response/controllability checks on a dense 1 m/s scheduling grid (45..55 m/s, all cfg; 55 operating points, including all 15 certified vertices);
  2. sign check: low + descending state must not command weaker nose-up action;
  3. 12 linear cfg transition tests (cfg0->1->2->3->4 at 45/50/55) with the
     SAME controller, no recovery law.

The run script will not start the JSBSim V50 flight unless those certificates
all pass.

First runtime experiment
------------------------
Run only V50 first:
  .\run_urmpc_v20_D_temp.ps1

It builds/certifies the new controller, then runs one real 255 s mission:
  cfg0 --drop--> cfg1 --drop--> cfg2 --drop--> cfg3 --drop--> cfg4
at Hcmd=200 m, Vcmd=50 m/s.

Outputs
-------
Build/certification:
  matlab/results/mpc_physics_v1/unified_robust_mpc_v2/
    airdropx_unified_robust_mpc_bank.mat
    urmpc_model_envelope_report.csv
    urmpc_design_parameters.csv
    urmpc_vertex_preflight.csv
    urmpc_linear_drop_cert.csv
    URMPC_V2_MANIFEST.txt

V50 flight:
  matlab/results/mpc_physics_v1/fixed_stability_urmpc_v20/
    fixed_stability_summary.csv
    fixed_stability_failures.csv
    fixed_stability_run_errors.csv
    V050/urmpc_controller_trace.csv

Robustness terminology
----------------------
This is a rigorous engineering adaptive/LPV MPC with disturbance estimation
and vertex/transition certification. It is NOT a mathematical worst-case
robustness proof in the tube-MPC / min-max / mu / H-infinity sense.
A formal worst-case proof would require an explicit uncertainty polytope and a
robust invariant/tube or min-max formulation, which MATLAB's standard implicit
mpc object does not magically provide. Do not label a successful v2.0 test as
such a proof.
