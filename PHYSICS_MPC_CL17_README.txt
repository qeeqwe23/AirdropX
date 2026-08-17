AirdropX Physics MPC CL-1.7 - Early Tracking-Loss Recovery + TECS-Style Energy Sharing

Purpose
-------
Continue from the validated v1.6 15/15 physics bank and CL-1.6 runtime chain.
CL-1.7 does NOT rebuild trim/models and does NOT retune the normal MPC Q/R,
Np/Nc, normal MV constraints, or actuator slew guards.

What CL-1.6 proved on V50
-------------------------
- normal MPC + recovery remained closed-loop; mpcFail no longer caused cfg2 divergence;
- cfg2 recovery entered at about t=137.5 s by authority-limit reason=2;
- by entry, altitude was already ~10.4 m low and vz-vz_ref was about -2.27 m/s;
- CL-1.6 recovery initially requested only about -0.12 elevator deviation, so the
  target itself (not the 0.012/sample actuator slew guard) was too weak;
- throttle remained positively biased while Va later rose well above 50 m/s.

CL-1.7 changes
--------------
1) Persistent early tracking-loss detector (reason=6)
   Recovery is armed only when ALL are true for a hold interval:
     abs(height error) >= 1.5 m
     aircraft vertical motion is away from the target
     abs(vz-vz_ref) >= 0.35 m/s and has the same wrong-direction sign
   Default hold: 10 samples = 1.0 s.
   On the CL-1.6 V50 trace this condition would have matured near t=123.3 s,
   roughly 14 s before the old authority-limit entry.

2) Bumpless authority extension
   The early recovery handoff keeps the valid MPC command from that sample as
   an anchor.  Recovery is never allowed to weaken elevator correction in the
   direction needed to return toward the requested altitude.

3) Simplified TECS-style recovery allocation
   Elevator correction uses:
     vertical-speed tracking error + bounded altitude error + pitch + q damping.
   Throttle uses total specific-energy error:
     g*(Hcmd-H) + 0.5*(Vref^2 - Va^2)
   Thus overspeed automatically suppresses unnecessary throttle boost while
   elevator can convert kinetic energy into altitude.

4) Existing actuator slew guards are unchanged
   elevator <= 0.012/sample
   throttle <= 0.020/sample
   CL-1.7 intentionally does NOT solve the problem by making the actuators jump.

Trace additions
---------------
The controller trace now has 56 columns. New CL-1.7 diagnostics:
  tracking_loss_count
  tracking_loss_streak
  recovery_energy_error_jpkg
  recovery_target_deviation_elevator
  recovery_target_deviation_throttle

Recovery reason codes
---------------------
0 normal MPC
1 state recovery envelope
2 normal MPC sustained authority limit
3 normal-MPC trust gate reject
4 mpcmove/QP fault
5 hard recovery
6 persistent vertical tracking loss (CL-1.7 early entry)

Run V50 first
-------------
  .\run_physics_mpc_fixed_stability_D_temp.ps1 -Speeds 50 -Workers 1

Monitor
-------
  .\check_physics_mpc_fixed_stability.ps1

Output
------
  matlab/results/mpc_physics_v1/fixed_stability_cl17

Primary question for the next run
---------------------------------
Does cfg2 enter reason=6 near the first persistent wrong-way vertical motion,
well before the previous t~137.5 s reason=2 entry, and does that prevent the
cfg2->cfg3 chain from starting from a deeply descending state?
