AirdropX Physics MPC CL-1.2 - runtime cfg-index safety fix

Scope:
- NO trim rebuild, NO MPC Q/R tuning, NO MEX rebuild.
- Keeps CL-1.1 mpcmove API fix and D-drive cache routing.
- Fixes a runtime crash path where a nonfinite/noisy mass-derived cfg or an
  inconsistent node schema could reach node.trim_bank(cfg).

Changes:
1) Runtime cfg is always finite integer 1..5. Nonfinite mass holds last cfg.
2) cfg is monotonic and advances by at most one payload state per 0.1 s sample.
3) All node nominal/controller accesses clamp/validate against both controller
   and trim-bank lengths; fail-safe nominal lookup is nonthrowing.
4) If no nominal is available, last finite physical command is preserved rather
   than terminating Simulink.
5) Preflight now checks all 15 controller+trim+physical-nominal entries.
6) Controller trace adds mass_kg_controller, cfg_mass_raw, cfg_used,
   cfg_invalid_count.

FIRST TEST (recommended):
  .\run_physics_mpc_fixed_stability_D_temp.ps1 -Speeds 50 -Workers 1

Only after V50 completes without run_error should the full scan be rerun:
  .\run_physics_mpc_fixed_stability_D_temp.ps1 -Speeds 45,50,55 -Workers 3
