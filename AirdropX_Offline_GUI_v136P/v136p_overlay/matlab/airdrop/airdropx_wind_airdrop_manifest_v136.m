function T=airdropx_wind_airdrop_manifest_v136(path)
%AIRDROPX_WIND_AIRDROP_MANIFEST_V136 Paired missions with a longer zero-wind settle for sine.
Scenario=["calm";"tailwind_5";"headwind_5";"tailwind_12";"headwind_12";"step_bidirectional";"ramp_minus10_plus10";"sine_longitudinal"];
Duration_s=[55;55;55;55;55;55;55;60];
TargetStart_m=repmat(1200,8,1); TargetSpacing_m=repmat(80,8,1); SensorNoiseSeed=(101:108)';
T=table(Scenario,Duration_s,TargetStart_m,TargetSpacing_m,SensorNoiseSeed);
if nargin>0 && strlength(string(path))>0, writetable(T,path); end
end
