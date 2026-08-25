function [wind_mps,wind_rate_mps2,sigma_mps,raw_mps,valid,step_detected]=airdropx_longitudinal_wind_estimator_runtime_v111(Va_mps,Vz_mps,Vg_mps,reset)
%AIRDROPX_LONGITUDINAL_WIND_ESTIMATOR_RUNTIME_V111 Persistent 10-Hz runtime wrapper.
% Intended for direct use from a MATLAB Function block or ordinary MATLAB loop.
persistent s
if isempty(s) || reset
    s=airdropx_longitudinal_wind_estimator_init_v111(0.1);
end
[s,o]=airdropx_longitudinal_wind_estimator_step_v111(s,Va_mps,Vz_mps,Vg_mps);
wind_mps=o.wind_est_mps;
wind_rate_mps2=o.wind_rate_est_mps2;
sigma_mps=o.wind_sigma_mps;
raw_mps=o.raw_wind_mps;
valid=o.valid;
step_detected=o.step_detected;
end
