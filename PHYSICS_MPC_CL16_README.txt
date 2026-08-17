AirdropX Physics MPC CL-1.6 - Unified Recovery Envelope

Purpose
- Keep the validated v1.6 15/15 physics bank unchanged.
- Keep the CL-1.5 altitude-governor anti-windup fix.
- Keep normal MPC Q/R, prediction/control horizons, local plant models and nominal actuator limits unchanged.
- Replace the old binary "local MPC OR trim fallback" behavior with a three-region runtime architecture:
    NORMAL   = validated local MPC
    RECOVERY = one unified bounded physical-actuator recovery law
    HARD     = stay in bounded recovery; do not remove control authority by falling back to trim
- Same recovery rule for every speed and cfg. No per-cfg tuning or learning.

Why CL-1.6 is needed (from CL-1.5 V50 trace)
- cfg0 and cfg1 pass after the altitude-governor fix.
- cfg2 starts a slow sink while mpcmove remains feasible and successful.
- Before the old fast-state gate rejects MPC, the local MPC command reaches its own local deviation authority (roughly elevator +/-0.10 and throttle +/-0.18 around nominal).
- At about t=143.7 s, vertical-speed deviation crosses the old ~6 m/s trust limit. The old controller then stops MPC and walks toward trim nominal, exactly when additional recovery authority is required.
- Therefore simply widening the old gate is not sufficient: the normal local MPC is already outside its validated/available authority before the gate trip.

CL-1.6 recovery entry
Recovery begins when any of the following occurs:
1) state/reference tracking leaves the early recovery envelope
   [|dVa|, |dPitch|, |vz-vz_ref|, |q|] > [8 m/s, 15 deg, 3 m/s, 8 deg/s], or
2) the normal MPC command remains at >=95% of a local actuator-deviation limit for 8 samples while |vz-vz_ref| >= 1 m/s, or
3) the normal local-MPC trust gate rejects the state, or
4) mpcmove has a solver exception/QP failure.

Hard recovery
- A wider hard envelope [15 m/s, 35 deg, 15 m/s, 30 deg/s] is diagnostic, not a request to return to trim.
- Crossing it keeps bounded recovery active and increments recovery_hard_count.
- Nonfinite plant state is still handled by the existing startup/nonfinite hold logic.

Unified physical recovery law
- Recovery is centered on the current speed/cfg PHYSICAL trim nominal.
- It uses Va error, vz-vz_ref, pitch error from natural trim, and q.
- Positive MQ9 elevator is nose-down; excessive sink therefore requests negative/nose-up physical elevator and additional throttle.
- Recovery deviation is bounded to +/-0.30 elevator and +/-0.35 throttle around nominal.
- Existing final physical actuator limits and per-sample slew limits remain active.
- Control returns to normal MPC only after tracking is inside [3 m/s, 6 deg, 1 m/s, 3 deg/s] AND the required recovery command is back inside 90% of the normal local MPC actuator-deviation authority for 15 samples.

Important
- CL-1.6 does NOT rebuild the physics bank.
- CL-1.6 does NOT retune normal MPC Q/R or horizons.
- CL-1.6 does NOT remove the local-model trust concept; it changes what happens outside that trust region.
- The normal MPC remains the primary controller. Recovery is an engineering safety/recovery layer for large payload-release transients.

New trace diagnostics (51-column v32_controller_trace.csv)
- mpc_gate_reject_count
- recovery_count
- recovery_mode
- recovery_reason_code
- recovery_hard_count
- authority_limit_count
- authority_limit_streak
- command_deviation_elevator
- command_deviation_throttle
- recovery_enter_count

Recovery reason codes
0 = normal MPC
1 = state/reference recovery-envelope entry
2 = sustained local-MPC actuator-deviation authority limit
3 = local-MPC fast trust-gate rejection
4 = QP/exception recovery
5 = hard recovery envelope

Segment summary diagnostics (not formal pass/fail gates)
- recovery_fraction
- recovery_count_increment
- recovery_hard_increment
- recovery_enter_increment
- mpc_gate_reject_increment
- authority_limit_increment
- authority_limit_fraction

Run V50 first
  .\run_physics_mpc_fixed_stability_D_temp.ps1 -Speeds 50 -Workers 1

Monitor
  .\check_physics_mpc_fixed_stability.ps1

Output
  matlab/results/mpc_physics_v1/fixed_stability_cl16

First questions to answer from CL-1.6 V50
1) Does cfg2 avoid the old t~143.7 loss-of-closed-loop event?
2) Which recovery reason first triggers: authority (2), state envelope (1), or trust gate (3)?
3) Does recovery bring |vz-vz_ref| back below 1 m/s and hand back to MPC?
4) Do cfg3/cfg4 become evaluable from a non-diverged incoming state?
5) How much of each steady-state scoring window is spent in recovery? Frequent recovery means the normal local MPC still needs a later robustness redesign; it is not hidden by the formal metrics.
