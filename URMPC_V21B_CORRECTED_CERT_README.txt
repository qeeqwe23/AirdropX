AirdropX UR-MPC v2.1B — corrected LPV offline certification
============================================================
Purpose
-------
Stage-B does NOT fly the learned model yet. It creates a separate corrected
candidate bank and certifies it offline.

Inputs already produced by v2.1A.1
---------------------------------
- matlab/results/mpc_physics_v1/urmpc_v21_residual_calibration/
  urmpc_v21_residual_models_candidate.mat (deploy_ready=1, lambda=1 expected)
- existing UR-MPC v2.0.x flight bank

Structural projection
---------------------
The learned residual correction is projected before use:
- deltaA(1,:) = 0
- deltaA(:,1) = 0
- deltaB(1,:) = 0
This preserves exact h(k+1)=h(k)+Ts*vz(k) and prevents learned altitude-error
correlation from becoming artificial longitudinal dynamics.
The projected matrices are revalidated on the original blocked validation
segments. 15/15 must remain <= 1.0.

Outputs
-------
matlab/results/mpc_physics_v1/urmpc_v21_corrected_candidate/
- urmpc_v21_projected_validation.csv
- urmpc_v21_corrected_vertex_models.csv
- urmpc_v21_corrected_grid_report.csv
- urmpc_v21_corrected_w_envelope.csv
- urmpc_v21_corrected_vertex_preflight.csv
- urmpc_v21_corrected_linear_drop_cert.csv
- urmpc_v21b_summary.csv
- airdropx_urmpc_v21_corrected_candidate.mat

Deploy gate
-----------
deploy_ready=1 only when:
1) projected calibration validation is 15/15 PASS
2) corrected dense vertex preflight is 55/55 PASS
3) corrected linear cfg transitions are 12/12 PASS
4) controllability rank remains 5 across stored and dense corrected grid

W policy
--------
The W CSV is audit-only. For intermediate speeds it takes the elementwise
maximum p95 residual of the two bracketing calibrated speed vertices at the
same cfg. No tube tightening is applied in v2.1B and no arbitrary inflation
factor is introduced.

Run
---
PowerShell:
  .\run_urmpc_v21b_corrected_cert_D_temp.ps1

Do not run V50 from this candidate until the Stage-B deploy gate has passed
and the results have been reviewed.
