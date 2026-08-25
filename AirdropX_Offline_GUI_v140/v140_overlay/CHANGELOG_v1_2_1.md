# v1.2.1

Full integration of the validated longitudinal wind estimator with the real nonlinear JSBSim carrier and precision airdrop mission chain.

- Adds a separate persistent `airdropx_jsbsim_wind_oracle_mex` so wind/gust history is physical across MPC samples; the validated v0.3.3 Oracle is not replaced.
- Retains the existing unified Physics-MPC Q/R and fixes the runtime horizon to 100.
- Wind estimate uses only GPS along-track ground speed, TAS and vertical speed. JSBSim wind truth is scoring-only.
- Four real JSBSim payload mass/CG/Iyy transitions are triggered automatically by predicted impact point, not fixed times.
- Four longitudinal targets are spaced 80 m by default, yielding roughly 1–2 s sequential releases while retaining cfg0->cfg4.
- Payload impact prediction uses estimated wind + estimated wind-rate forecast.
- Scoring cargo truth uses the AirdropX calibrated longitudinal ballistic model and the actual future wind profile after release.
- Runs 8 wind profiles in paired mode: wind-aware and no-wind baseline = 16 nonlinear carrier missions.
- Formal wind-aware gate includes four releases, landing error <=20 m, predicted impact at release <=10 m, wind p95 <=0.75 m/s, aircraft MPC gates and mass-property gates.
