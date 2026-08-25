# AirdropX Physics-MPC v1.3.1

## Goal
Finish the longitudinal anti-wind integration while also reducing the residual calm-air landing error without scenario-specific offsets.

## Anti-wind MPC retained from v1.3.0
- Per-cfg longitudinal wind-increment map `Gw` is identified from the persistent nonlinear JSBSim Oracle with symmetric `+/-0.5`, `+/-1`, `+/-2 m/s` one-sample experiments.
- MPC prediction uses `dx+ = A dx + B du + Gw*dWind + dResidual`.
- Future `dWind` is generated causally from the frozen v1.1.3 wind-rate estimate; no JSBSim truth wind is read by the controller.
- A clipped, decaying one-step residual observer handles remaining local model mismatch and is reset across payload cfg changes.
- Q/R, Bryson scales, hard input limits, `Np=Nc=100`, and the validated wind estimator are unchanged.
- The impossible first-sample gust peak remains report-only; formal carrier scoring uses post-gust peak, recovery time, final error and tail RMS.

## New: sub-sample physical release timing
At 50 m/s, a 0.1 s MPC sample corresponds to about 5 m of travel. The v1.2.1/v1.3.0 release event could therefore only occur on a roughly 5 m spatial grid, leaving an avoidable ~meter-to-few-meter calm miss even when the ballistic model is otherwise correct.

v1.3.1 keeps MPC at 10 Hz but decouples the payload release event from the controller update boundary:
1. use only the current measured carrier state and estimated wind/rate;
2. predict impact now and one MPC sample ahead with a short causal kinematic extrapolation;
3. if the target lies between them, solve a fractional timer `tau in [0, Ts)`;
4. hold the already-computed MPC command;
5. advance the nonlinear JSBSim carrier for `tau` with the old cfg;
6. remove the real payload point mass at that actual intermediate carrier state;
7. advance the remaining `Ts-tau` with the new cfg.

No future carrier truth is used to choose `tau`, and no calm-only range bias is introduced.

## Validation additions
The full runner still executes the 8 wind profiles x 3 main controller/release modes (24 formal missions), plus one diagnostic calm ablation that disables fractional timing. The final report therefore separates:
- anti-wind carrier benefit (`wind_mpc_aware` vs `legacy_mpc_aware`);
- wind-aware release benefit (`legacy_mpc_aware` vs `legacy_mpc_nowind`);
- calm release-quantization benefit (fractional vs sampled release timing).
