function T=airdropx_wind_airdrop_manifest_v121(path)
%AIRDROPX_WIND_AIRDROP_MANIFEST_V121 Paired wind-aware/no-wind precision missions.
Scenario=["calm";"tailwind_5";"headwind_5";"tailwind_12";"headwind_12";"step_bidirectional";"ramp_minus10_plus10";"sine_longitudinal"];
Duration_s=repmat(55,8,1);
TargetStart_m=repmat(1200,8,1); TargetSpacing_m=repmat(80,8,1); SensorNoiseSeed=(101:108)';
T=table(Scenario,Duration_s,TargetStart_m,TargetSpacing_m,SensorNoiseSeed);
if nargin>0 && strlength(string(path))>0, writetable(T,path); end
end
