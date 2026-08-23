function T=airdropx_wind_profile_manifest_v111(path)
%AIRDROPX_WIND_PROFILE_MANIFEST_V111 Formal longitudinal-only wind validation scenarios.
Scenario=["calm";"tailwind_5";"headwind_5";"tailwind_12";"headwind_12";"step_bidirectional";"ramp_minus10_plus10";"sine_longitudinal"];
Duration_s=[30;30;30;30;30;35;35;45];
Description=["0 m/s";"0 to +5 m/s at 5s";"0 to -5 m/s at 5s";"0 to +12 m/s at 5s";"0 to -12 m/s at 5s";"0,+8,-8,+3 m/s steps";"-10 to +10 m/s ramp";"2+6*sin longitudinal wind"];
T=table(Scenario,Duration_s,Description);
if nargin>0 && strlength(string(path))>0
    writetable(T,path);
end
end
