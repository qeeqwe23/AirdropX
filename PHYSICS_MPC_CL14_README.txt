AirdropX Physics MPC CL-1.4 - runtime cfg helper fix

Purpose
- Fix deterministic CL-1.2/1.3 bug in local_safe_node_cfg.
- The previous helper overwrote cfg with NaN before validating it, so every runtime
  local_node_move/local_node_nominal lookup failed.
- Add runtime-path nominal self-test in S-function Start.
- Seed last_physical_cmd from the requested-speed/current-cfg physical nominal before
  the first MPC evaluation, providing a deterministic first-sample fallback.
- Preserve CL-1.3 startup finite-state protections and D-drive cache routing.
- No MPC Q/R, horizon, governor, trim bank, plant, or actuator limits are changed.

Recommended run
  .\run_physics_mpc_fixed_stability_D_temp.ps1 -Speeds 50 -Workers 1

Expected
- mpcmove preflight 15/15 PASS
- no RuntimeNominalSelfTestFailed / NoInitialNominal / NoSafeNominal
- V050 trace exists
- then evaluate actual closed-loop behavior.
