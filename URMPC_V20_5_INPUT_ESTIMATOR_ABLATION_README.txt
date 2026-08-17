AirdropX UR-MPC v2.0.5 — input-estimator persistence ablation

PURPOSE
-------
This is a causal ablation, not a new tuned final controller.
V2.0.4 proved that disabling hidden output-disturbance integrators did not
remove the cfg2 chronic divergence.  In the remaining 2-state estimator,
the two integrated B-load disturbance estimates grow together with the
one-step equivalent-load residual, while the controller applies almost the
opposite MV deviation.  Before selecting any arbitrary leak time constant,
v2.0.5 removes only disturbance persistence.

ONLY BEHAVIORAL CHANGE
----------------------
Default build policy:
  setindist(ur_mpc,'model',tf(eye(2)))

This keeps two unmeasured load channels entering the plant through B, but the
input-disturbance model has zero internal states (unity-gain static white
noise model).  Output disturbance model remains explicitly disabled.

Everything else is unchanged from v2.0.4:
  - one adaptive MPC object
  - 5 states [H Va pitch vz q]
  - LPV A/B interpolation
  - physical absolute MV limits
  - certified soft MV-rate trust limits
  - Bryson-style scaling / same weights
  - same Np/Nc
  - no recovery, TECS, H-PI or cfg-specific tuning

EXPECTED BUILD AUDIT
--------------------
urmpc_estimator_design.csv should show:
  input_disturbance_states  = 0
  output_disturbance_states = 0
  input_disturbance_policy  = static_white

The source still supports InputDisturbancePolicy='integrators' to reproduce
v2.0.4, but the default is static_white for this ablation.

RUN
---
Rebuild is mandatory because the MPC estimator model changes:
  .\run_urmpc_v20_D_temp.ps1
Do NOT use -SkipBuild for the first v2.0.5 run.

INTERPRETATION
--------------
1) If cfg0/cfg1 remain acceptable and cfg2 no longer develops the 120-150 s
   exponential self-excitation, the integrated input disturbance estimator
   is a principal amplifier.  Then the next design should learn stable/leaky
   disturbance dynamics from audited residual data, rather than use pure
   integrators.
2) If cfg2 still diverges on essentially the same time axis, stop tuning the
   estimator: the main problem is model/residual geometry and the next step
   should be local residual W(rho,cfg) / robust MPC work.
3) A larger steady offset without exponential divergence is an informative
   result, not a failure of the ablation: static white noise intentionally
   gives up integral offset rejection.
