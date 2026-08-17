AirdropX UR-MPC v2.0.1 - actuator authority separation
========================================================

Reason for this patch
---------------------
The first real V50 nonlinear run showed that loss of control begins BEFORE the
cfg2->cfg3 transition.  In cfg2, both MPC moves reach the trim-derived design
MV envelope at about t=144.1 s.  Slack starts growing shortly afterwards and
the aircraft reaches cfg3 already far from the certified operating region.

The v2.0 code used the trim-manifold trust envelope as a HARD actuator limit:
  elevator  [-0.609..., +0.041...]
  throttle  [ 0.454...,  0.673...]

However the project-level physical limits are:
  physical elevator [-0.95, +0.95], further tightened online by hidden +/-0.85
  throttle          [ 0.00,  1.00]

For the observed V50 hidden elevator offset ~= -0.497953, the actually
reachable physical elevator range is [-0.95, +0.352047].  Therefore v2.0 was
throwing away real control authority before the nonlinear plant had exhausted
its actuator.

What changed
------------
1) airdropx_urmpc_build.m
   - trim-derived MV envelope is retained as a TRUST/diagnostic envelope.
   - the single MPC object's hard MV constraints now use physical limits.
   - dense vertex preflight and linear cfg-transition certification use the
     same physical-reachable hard-authority policy as runtime.
   - design CSV records both trust bounds and physical controller hard bounds.
   - meta.version = urmpc_v2_0_1_authority_separation.

2) sfun_airdropx_urmpc_controller.m
   - runtime elevator bounds are only the intersection of the physical
     elevator limit and hidden-offset external-delta reachability.
   - runtime throttle bounds are the bank-declared physical throttle limits.
   - design_mv_bounds are no longer intersected into hard runtime constraints.

What did NOT change
-------------------
- one adaptive MPC object
- LPV speed interpolation
- 5 states [H Va pitch vz q]
- load-disturbance estimator
- Bryson output weights/scales
- Np / ControlHorizon
- MV / MV-rate weights
- elevator sign
- no recovery, no H-PI governor, no TECS, no cfg-specific tuning

Run requirement
---------------
This changes constraints stored in the MPC bank.  Rebuild is REQUIRED:
  SkipBuild=False
Do not reuse the previous unified bank with SkipBuild=True.

Expected validation sequence
----------------------------
1) 55/55 vertex preflight
2) 12/12 linear transitions
3) V50 255 s nonlinear closed loop

Interpretation after rerun
--------------------------
If cfg2 remains stable materially longer and active commands move beyond the
old [-0.609,0.041]/[0.454,0.673] trust envelope without QP failure, the
artificial authority cap was a primary blocker.  If the aircraft still departs
while substantial physical authority remains, then the next step is genuine
model-uncertainty treatment (polytopic/tube/constraint-aware robust MPC), not
another recovery layer.
