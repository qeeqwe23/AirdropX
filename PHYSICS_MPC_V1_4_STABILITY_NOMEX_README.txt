AirdropX Physics MPC v1.4 Stability NO-MEX
==========================================

Purpose
-------
Fix the v1.3.1 trim-design inconsistency before closed-loop MPC tuning.
This release requires NO MEX compilation and continues using 3 process workers.

Root causes fixed
-----------------
1. v1.3.1 treated requested initial pitch as a Gauss-Newton trim variable.
   In the legacy JSBSim MEX, initial pitch changes the internal 3 s
   autoTrimSettle elevator bias. This made the optimizer differentiate the
   hidden initialization routine instead of the physical steady-state model.
2. When a candidate passed, v1.3.1 replaced requested IC pitch with observed
   tail pitch. The certification rerun therefore used a different hidden
   elevator bias, so identical e/throttle values were not identical physical
   controls. This explains PASS-in-trace -> FAIL-on-recheck cases.
3. DirectCfgViaAircraftXml already removes cargo in a temporary aircraft XML.
   v1.4 forces airdropx_enable_initial_drop_count=0 in this mode so a newer
   existing MEX cannot also remove the payload a second time.
4. The old direct-cfg smoke test trusted relabelled legacy mass/drop outputs.
   v1.4 instead verifies that the cfg-specific temporary aircraft variant was
   selected; JSBSim LoadModel itself is the physical-load proof.

v1.4 trim formulation
----------------------
- Pitch is NOT optimized.
- Every base/probe/line-search/recheck run at one node uses the SAME immutable
  initial-pitch seed from the source v32 trim bank.
- Only elevator + throttle are solved.
- The deterministic overdetermined residual is:
    [Va-Vref, vz, q, height_slope, Va_slope]
  normalized by the unchanged hard equilibrium gates.
- A fixed finite-difference 5x2 damped Gauss-Newton step with line-search
  factors [1, 0.5, 0.25] is used.
- Observed steady pitch is recorded only as the MPC nominal state.
- No BayesOpt, no model-order search, no online/persistent learning.

Cache version
-------------
physics_mpc_v1_4_stability_nomex
All 15 nodes are rebuilt once so no v1.3.1 model is mixed into the bank.

Run
---
1. Overlay this patch on D:\vscode project\AirdropX
2. .\run_physics_mpc_preflight_D_temp.ps1
3. .\run_physics_mpc_build_D_temp.ps1

Do NOT run closed-loop validation until the 15-node build result is reviewed.
