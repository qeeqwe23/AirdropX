AirdropX UR-MPC v2.1A — nonlinear vertex residual learning (Stage A)
====================================================================

Why this stage exists
---------------------
v2.0.5 proved that removing persistent disturbance-estimator states does not
remove the cfg2/cfg3 divergence. Re-analysis of the v2.0.5 residual trace also
shows that, once cfg2 begins to leave the local operating point, the normalized
one-step residual is almost perfectly correlated with normalized state distance
(correlation about 0.99 in 115-145 s windows). This is evidence for missing
state/input-dependent dynamics (delta-A/delta-B), not a stationary additive W.

The full-drop V50 trace has no trustworthy near-trim cfg3/cfg4 data because
those cfgs inherit an already-diverged state. Therefore this patch DOES NOT
apply a cfg2 correction to cfg3/cfg4 and DOES NOT build a tube from failed data.

What this patch does
--------------------
1) Adds airdropx_urmpc_v21_vertex_calibration.m
   - same experiment for every speed x cfg vertex
   - reaches target cfg by an early short drop sequence
   - waits for settling
   - applies small common altitude/speed reference excitation
   - keeps only samples inside one common normalized trust radius
   - controller/bank are unchanged

2) Adds airdropx_urmpc_v21_fit_residual_models.m
   - fits residual = deltaA*dx + deltaB*du + w
   - no intercept, so the trim equilibrium is preserved
   - one globally selected ridge lambda for all vertices
   - blocked time validation, not random shuffle
   - same trust/sample/validation rule for all vertices
   - NEVER overwrites the flight bank in Stage A
   - writes candidate deltaA/deltaB and remaining-residual W diagnostics

3) Adds run_urmpc_v21_residual_learning_D_temp.ps1

Outputs
-------
matlab/results/mpc_physics_v1/urmpc_v21_residual_calibration/
  urmpc_v21_calibration_manifest.csv
  urmpc_v21_residual_fit_report.csv
  urmpc_v21_lambda_cv.csv
  urmpc_v21_residual_models_candidate.mat
  V045_cfg0/... through V055_cfg4/...

Deployment rule
---------------
Stage A never changes flight control. deploy_ready can become true only when
EVERY requested vertex has enough trusted data and the common residual learner
does not worsen blocked validation at any vertex. A later Stage B can then
apply the corrected local models, rerun 55-point/12-transition certification,
and construct W only from the corrected residual.

Run
---
PowerShell:
  .\run_urmpc_v21_residual_learning_D_temp.ps1

Refit without rerunning simulations:
  .\run_urmpc_v21_residual_learning_D_temp.ps1 -FitOnly


v2.1A.1 lambda-selection correction
----------------------------------
The original Stage-A implementation selected the one global ridge lambda by
minimum MEDIAN validation ratio, while deployment required EVERY vertex to
avoid validation degradation. Those objectives are inconsistent. v2.1A.1
selects lambda by a worst-vertex minimax rule aligned with the deployment gate.
If one or more lambdas satisfy worst_validation_ratio <= MaxValidationRatio,
selection is restricted to that feasible set. Otherwise the best minimax
diagnostic candidate is retained with deploy_ready=false.

Existing 15-vertex calibration data do NOT need to be regenerated. Refit only:
  .\run_urmpc_v21_residual_learning_D_temp.ps1 -FitOnly

Stage A still never changes the flight bank.
