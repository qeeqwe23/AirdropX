function r=airdropx_wind_airdrop_entry_v131(projectRoot,opts)
%AIRDROPX_WIND_AIRDROP_ENTRY_V131 Process-isolated integrated mission entry point.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.ScenarioName (1,1) string
    opts.UseWindCompensation (1,1) logical
    opts.UseWindDisturbanceMPC (1,1) logical
    opts.WindCalibrationPath (1,1) string
    opts.UseFractionalRelease (1,1) logical = true
    opts.Duration_s (1,1) double = 55
    opts.TargetStart_m (1,1) double = 1200
    opts.TargetSpacing_m (1,1) double = 80
    opts.SensorNoiseSeed (1,1) double = 1
end
addpath(fullfile(projectRoot,"matlab")); addpath(fullfile(projectRoot,"matlab","phys_mpc")); addpath(fullfile(projectRoot,"matlab","wind")); addpath(fullfile(projectRoot,"matlab","airdrop"));
r=airdropx_wind_airdrop_mission_v131(projectRoot,OutputRoot=opts.OutputRoot,ScenarioName=opts.ScenarioName,UseWindCompensation=opts.UseWindCompensation,UseWindDisturbanceMPC=opts.UseWindDisturbanceMPC,WindCalibrationPath=opts.WindCalibrationPath,UseFractionalRelease=opts.UseFractionalRelease, ...
    Duration_s=opts.Duration_s,TargetStart_m=opts.TargetStart_m,TargetSpacing_m=opts.TargetSpacing_m,SensorNoiseSeed=opts.SensorNoiseSeed,ThrowOnFail=false);
end
