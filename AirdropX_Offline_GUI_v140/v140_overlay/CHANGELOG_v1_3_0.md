# AirdropX Physics-MPC v1.3.0

## Goal
Integrate longitudinal anti-wind prediction into the existing unified Physics-MPC without changing the frozen v1.1.3 wind estimator or the already successful v1.2.1 wind-aware release guidance.

## Controller change
The carrier model remains the certified 7-state / 2-input Physics-MPC with the same Q/R, scales, hard input bounds and Np=Nc=100. v1.3.0 adds a measured additive disturbance channel:

`dx(k+1) = A dx(k) + B du(k) + g(k)`

where

`g(k) = Gw * DeltaWind(k) + dResidual(k)`.

`Gw` is NOT guessed. It is identified at H=200 m / Va=50 m/s for cfg0..cfg4 by restarting the persistent nonlinear JSBSim Oracle at the exact trim and applying symmetric +/-0.5, +/-1 and +/-2 m/s longitudinal wind increments for one 0.1 s sample. The +/-1 m/s central slope is deployed and the multi-probe spread is reported.

## Why DeltaWind, not absolute wind
For a uniform wind field and air-relative states, a constant wind changes ground speed but is not a permanent aerodynamic force. A wind change/gust changes relative airspeed and creates the carrier transient. Therefore v1.3.0 previews wind increments from the estimated wind-rate instead of injecting absolute wind as a fake persistent state disturbance.

## Residual disturbance observer
A causal one-step residual observer estimates remaining local model mismatch after subtracting the identified `Gw * observed DeltaWind` term. The residual is clipped in normalized state units and decays across the prediction horizon. It resets around payload cfg transitions so a 300 kg mass drop is not learned as “wind”.

## QP implementation
The original Hessian and box input constraints are unchanged. A generic disturbance prediction matrix is condensed once per cfg; only the QP linear term changes online. With zero disturbance sequence, a self-test requires the first control action to match the legacy MPC to <=1e-10.

## Validation change
Unknown instantaneous wind steps are causal disturbances. Their first-sample `peak_primary_normalized` is still logged but is not treated as a command the controller must cancel before it is sensed. Formal carrier wind gates are:
- post-gust peak after a fixed 1.0 s causal exclusion window <= 1.25 normalized;
- recovery to that bound and stay there for 0.5 s within 3.0 s;
- original final and tail gates;
- original input/feasibility/realtime gates.

The runner executes 24 missions: for each of 8 wind profiles it compares
1. `wind_mpc_aware`: new wind-disturbance MPC + wind-aware release;
2. `legacy_mpc_aware`: legacy MPC + the same wind-aware release;
3. `legacy_mpc_nowind`: legacy MPC + no-wind release baseline.

This separates carrier anti-wind benefit from release-guidance benefit.
