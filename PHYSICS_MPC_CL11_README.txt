AirdropX Physics MPC CL-1.1 runtime fix

Purpose
-------
Fix the CL-1 result where mpc_fail_count increased every controller sample.
This patch does NOT rebuild the 15-node physics bank and does NOT tune Q/R.

Root cause
----------
The controller called mpcmove with:
  ym   = 4x1 (correct)
  rDev = 4x1 (incorrect)
For classical MPC with Ny=4, MATLAB requires the constant output reference r
to be 1x4 (or more generally p x Ny). The former call throws every sample;
the old catch hid the exception and the airplane stayed on trim feedforward.

Changes
-------
1. rDev is now forced to 1x4 before mpcmove.
2. QP status uses Info.Iterations / textual Info.QPCode (R2026a semantics).
3. Runtime exceptions are no longer silently swallowed; first unique errors print.
4. New mpcmove_preflight.csv executes all 15 stored controllers before flight.
5. On a runtime MPC failure, fallback ramps toward CURRENT-cfg trim nominal instead
   of freezing the previous cfg command forever.
6. Pitch-difference q estimate is angle-wrap safe.
7. Controller trace adds:
   mpc_success_count, mpc_exception_count, mpc_qp_fail_count, mpc_last_iterations.
8. Existing D-drive cache/job/temp routing is retained.

Run
---
  .\run_physics_mpc_fixed_stability_D_temp.ps1

Expected before flight
----------------------
  [PHYS-MPC CL1.1] mpcmove preflight: 15/15 PASS

Outputs
-------
  matlab/results/mpc_physics_v1/fixed_stability_cl11/mpcmove_preflight.csv
  matlab/results/mpc_physics_v1/fixed_stability_cl11/fixed_stability_summary.csv
  .../V045, V050, V055/v32_controller_trace.csv
