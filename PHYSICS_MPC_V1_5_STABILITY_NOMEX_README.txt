AirdropX Physics MPC v1.5 Stability NO-MEX
===============================================

Purpose
-------
This patch does NOT tune the final MPC yet. It fixes the physical operating-point
pipeline so the later MPC is built on correct, repeatable physical coordinates.

No MEX rebuild is required. Existing sfun_airdropx_jsbsim.mexw64 is reused.

Main v1.5 corrections
---------------------
1. True elevator coordinates
   The legacy MEX applies:
       physical elevator = hidden trimElevator_ + external elevator delta
   Earlier Physics-MPC code called the external delta "physical".
   v1.5 exposes BOTH:
       elevator_external_delta_actual
       elevator_physical_actual
   The model, nominal inputs, continuation and MPC bank use the true physical value.

2. Speed-row parallelism + cfg continuation
   Three process workers are retained:
       worker A: V45 cfg0 -> cfg1 -> cfg2 -> cfg3 -> cfg4
       worker B: V50 cfg0 -> cfg1 -> cfg2 -> cfg3 -> cfg4
       worker C: V55 cfg0 -> cfg1 -> cfg2 -> cfg3 -> cfg4
   The seed passed between cfgs is:
       physical elevator, throttle, equilibrium pitch
   not the old v32 external-delta seed.

3. Hidden elevator compensation
   For every target cfg/pitch seed, v1.5 measures the target MEX hidden elevator
   offset with a short zero-external-delta run. A requested physical elevator is
   converted back to the old MEX coordinate:
       external delta = requested physical elevator - hidden offset

4. Pitch is a state, not an actuator
   Elevator/throttle solve only:
       Va error, vz, q, height slope, Va slope
   pitchStd is NOT in the actuator optimization score.
   If those five physical residuals pass but pitchStd is still high, v1.5 keeps
   the exact same physical elevator/throttle, restarts from the observed steady
   pitch, remeasures hidden elevator, and repeats the certification.
   This removes false long-period transients without using pitch as a control input.

5. Runtime hidden elevator is latched once
   The current MEX computes hidden trimElevator_ only at reset. It does NOT change
   when speed changes later. Therefore the runtime controller now interpolates the
   cfg0 hidden offset once at the initial measured speed and latches it for the
   whole flight. Speed changes no longer create a fictitious elevator bias.

6. NO-MEX direct cfg remains
   Offline cfg1..cfg4 aircraft variants are temporary XML copies with already-dropped
   cargo point masses set to zero. Real mission validation still starts from cfg0
   and performs real drops.

7. Hard gates remain unchanged
       |Va error| <= 0.50 m/s
       |vz|       <= 0.15 m/s
       |q|        <= 0.15 deg/s
       |h slope|  <= 0.15 m/s
       |Va slope| <= 0.05 m/s^2
       pitch std  <= 0.50 deg
   Repeatability, regressor rank, controllability rank=4 and held-out validation
   still apply. No gate is relaxed to force a pass.

How to run
----------
Copy the patch contents into:
    D:\vscode project\AirdropX

Then:
    .\run_physics_mpc_preflight_D_temp.ps1
    .\run_physics_mpc_build_D_temp.ps1

Do NOT run final validation yet unless the build completes 15/15.

Important outputs
-----------------
matlab\results\mpc_physics_v1\physics_mpc_build_failures.csv
matlab\results\mpc_physics_v1\physics_mpc_model_report.csv

Per-node diagnostics now also include:
    continuation_seed.csv
    pitch_state_consistency.csv
    hidden_probe_...\auto_id_timeseries.csv
    deterministic_retrim_pcXX\deterministic_trim_trace.csv

Expected startup smoke
----------------------
The NO-MEX smoke prints:
    hiddenE=<finite number>
    coordPASS=1
    PASS=1

This proves that the run can distinguish legacy external elevator delta from the
true physical elevator reported by the existing MEX.
