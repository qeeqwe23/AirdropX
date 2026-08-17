AirdropX Physics MPC CL-1.3 - startup-state / nonfinite telemetry safety

Purpose
-------
Fix the V50 first-sample crash seen in CL-1.2 without changing MPC Q/R,
trim bank, governor gains, actuator limits, or the 15 local physical models.

Root cause addressed
--------------------
A Level-2 controller Outputs call may occur before all six JSBSim feedback
signals are finite. CL-1.2 hardened mass->cfg indexing but could still pass a
nonfinite Va into speed-node scheduling. An empty speed bracket then caused
both MPC and nominal fallback paths to be unavailable on the first sample.

CL-1.3 behavior
---------------
1. A complete finite-state gate covers H, vz, Va, pitch, mass and cg.
2. Before the first complete state frame, no mpcmove/governor/cfg transition
   is attempted. The controller uses the commanded/reference speed to select
   the cfg0 physical trim nominal and outputs that safe nominal.
3. The JSBSim hidden elevator offset is latched from the same requested-speed
   bracket. It is not recomputed from a NaN measured Va.
4. After closed loop has become active, a transient nonfinite telemetry frame
   holds the last finite physical command exactly and does not advance MPC
   states or governors.
5. local_speed_bracket has its own NaN/Inf last-line defense, so an empty node
   index can never be generated.
6. q differentiator memory ignores nonfinite pitch samples.
7. Controller trace expands from 38 to 41 columns with:
      input_invalid_count, startup_hold_count, state_ready
   The invalid startup row is deliberately logged, so V50's first-frame
   values can be inspected instead of being lost in a run-level exception.

No tuning
---------
No MPC weights, horizons, model bank, equilibrium gates, height/speed governor
settings, actuator limits or cfg-specific parameters are changed.

Recommended run
---------------
  .\run_physics_mpc_fixed_stability_D_temp.ps1 -Speeds 50 -Workers 1

Expected first-stage success criteria
-------------------------------------
- mpcmove preflight 15/15 PASS
- no run_error / no NoSafeNominal
- V050/v32_controller_trace.csv exists
- cfg_used advances 0->1->2->3->4 during the real four-drop mission
- startup_hold_count may be >0, but mpc_exception_count should stay 0
- inspect the first rows' actual_v_mps and input_invalid_count to confirm the
  exact startup signal that was previously nonfinite.
