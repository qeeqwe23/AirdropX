# v1.3.2

Result-driven refinement of v1.3.1.  No formal gate is relaxed and the successful v1.2.1 ballistic model / v1.3.0 physical `Gw` calibration are retained.

## Why this version exists
The completed v1.3.1 24/24 three-way validation showed three distinct facts:
- wind-aware landing remained excellent (worst max miss about 3.92 m versus about 78.96 m without wind compensation);
- `Gw` disturbance MPC materially helped ramp/sine wind but only slightly improved hard step recovery;
- calm wind-aware release remained worse than the no-wind release baseline, indicating estimator-noise compensation, while the fractional-release A/B remained strongly beneficial.

## Changes
- Retains the real-run fix that aligns fractional `tau` to the wind Oracle native `1/120 s` grid before physical split propagation.
- Adds `airdropx_wind_confidence_v132`: a smooth scenario-agnostic significance gate using estimated wind magnitude, estimated wind rate and estimator uncertainty. Small noise-like estimates shrink toward zero; 5/12 m/s winds remain essentially unchanged.
- Adds causal adaptive filtering of the release-guidance Vg/Va/Vz measurements. Slow noise is reduced, while >3-sigma motion uses a fast update to avoid gust/maneuver lag.
- Adds an exact calm fallback: when wind/residual evidence is negligible and recovery is inactive, the original certified no-wind solver is called directly.
- Adds residual-observer innovation deadband/full-scale shrinkage so calm measurement/model noise is not learned as a disturbance.
- Adds a unified gust-recovery MPC cost bank shared by cfg0..cfg4.  It changes only transient stage Q/R weights; A/B, trim, hard bounds, horizon and certified terminal P remain unchanged.  Recovery is triggered by causal wind evidence, not by payload-drop transients.
- Recovery strengthens Va correction, protects gamma/theta/q, and lowers transient throttle/elevator effort penalties while preserving hard actuator limits.
- Keeps strict v1.3.1 carrier gates unchanged. No pass is manufactured by threshold relaxation.
- Adds explicit 0.5 s / 1 s / 3 s gust residual metrics, actuator saturation fractions/headroom, wind-MPC activity, recovery activity and confidence traces.
- Makes the formal fractional-release predicted-impact gate more strictly causal: it uses the filtered state extrapolated from the scheduling sample, not the nonlinear Oracle state reached at the fractional release instant.
- Integrates the startup fixes discovered during the real v1.3.1 run: install self-copy skip, no `$pid/$PID` collision, clean `PollCase` braces, and Start-Process capture so harmless JSBSim stderr notices are not treated as PowerShell fatal errors.
