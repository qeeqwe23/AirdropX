function result = airdropx_auto_run_best_closed_loop(varargin)
%AIRDROPX_AUTO_RUN_BEST_CLOSED_LOOP Reproduce the current best learned MPC run.
%
% This uses the R2026a mpc() bank learned from JSBSim data and the best
% pitch reference is taken automatically from trim_bank for each configuration.

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir);
addpath(paths.mpcDir);
addpath(paths.autoDir);
addpath(paths.sfuncDir);

bankPath = string(opts.MpcBankMat);
if strlength(bankPath) == 0
    bankPath = string(fullfile(paths.matlabDir, "results", "mpc_auto_train_from_r2_datatrim", "airdropx_learned_mpc.mat"));
end
if ~isfile(bankPath)
    error("AirdropX:AutoMPC:MissingBank", "Learned MPC bank not found: %s", bankPath);
end

S = load(bankPath, "trim_bank");
trim0 = S.trim_bank(1);
pitchRefDeg = double(trim0.pitch_deg);

assignin("base", "airdropx_auto_elevator_sign", 1.0);
assignin("base", "airdropx_auto_use_trim_pitch_reference", 1.0);
assignin("base", "airdropx_auto_pitch_kp", 0.0);
assignin("base", "airdropx_auto_pitch_kq", 0.0);
assignin("base", "airdropx_auto_pitch_damp_max", 0.0);
assignin("base", "airdropx_auto_throttle_alt_high_gain", 0.0);
assignin("base", "airdropx_auto_throttle_climb_gain", 0.0);
assignin("base", "airdropx_auto_elevator_safety_gain", 0.0);
assignin("base", "airdropx_auto_elevator_sink_gain", 0.0);
assignin("base", "airdropx_auto_elevator_safety_max", 0.0);
assignin("base", "airdropx_auto_throttle_safety_gain", 0.0);
assignin("base", "airdropx_auto_throttle_sink_gain", 0.0);
assignin("base", "airdropx_auto_throttle_safety_max", 0.0);

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    outputRoot = string(fullfile(paths.matlabDir, "results", "mpc_auto_best_closed_loop_p12"));
end

result = airdropx_auto_run_closed_loop( ...
    "ProjectRoot", paths.projectRoot, ...
    "MpcBankMat", char(bankPath), ...
    "OutputRoot", char(outputRoot), ...
    "CaseId", char(opts.CaseId), ...
    "StopTimeS", double(opts.StopTimeS), ...
    "AfterDropTime", 10.0, ...
    "FixedDropStartS", 10.0, ...
    "FixedDropIntervalS", 0.2, ...
    "FixedDropTotal", 4.0, ...
    "InitialAirspeedMps", double(trim0.airspeed_mps), ...
    "InitialAltitudeM", double(trim0.altitude_m), ...
    "InitialPitchDeg", pitchRefDeg, ...
    "InitialFlightPathDeg", 0.0, ...
    "InitialElevatorDelta", double(trim0.elevator_cmd), ...
    "InitialThrottleCmd", double(trim0.throttle_cmd), ...
    "TargetAltitudeM", double(trim0.altitude_m), ...
    "TargetAirspeedMps", double(trim0.airspeed_mps), ...
    "TargetPitchDeg", pitchRefDeg);

T = result.timeseries;
idx10 = T.time_s >= 10.0;
idx12 = T.time_s >= 12.0;
metrics = table( ...
    min(T.altitude_m), min(T.altitude_m(T.time_s <= 10.0)), max(T.altitude_m), T.altitude_m(end), ...
    std(T.pitch_deg(idx10)), max(T.pitch_deg(idx10)) - min(T.pitch_deg(idx10)), mean(T.pitch_deg(idx10)), ...
    std(T.pitch_deg(idx12)), max(T.pitch_deg(idx12)) - min(T.pitch_deg(idx12)), mean(T.pitch_deg(idx12)), ...
    T.pitch_deg(end), max(T.drop_count), ...
    'VariableNames', {'min_h','min_h_first10','max_h','last_h','pitch_std_after10','pitch_range_after10','pitch_mean_after10','pitch_std_after12','pitch_range_after12','pitch_mean_after12','last_pitch','max_drop_count'});
result.best_metrics = metrics;
writetable(metrics, fullfile(outputRoot, "best_metrics.csv"));
disp(metrics);
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.MpcBankMat = "";
opts.OutputRoot = "";
opts.CaseId = "auto_best_p12";
opts.StopTimeS = 24.0;
opts.PitchRefDeg = NaN;
if mod(numel(varargin), 2) ~= 0
    error("Options must be name-value pairs.");
end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name)
        error("Unknown option: %s", name);
    end
    opts.(name) = varargin{i + 1};
end
end

function paths = local_paths(projectRoot)
projectRoot = string(projectRoot);
if strlength(projectRoot) == 0
    thisDir = fileparts(mfilename("fullpath"));
    matlabDir = fileparts(thisDir);
    projectRoot = string(fileparts(matlabDir));
else
    matlabDir = fullfile(projectRoot, "matlab");
end
paths.projectRoot = char(projectRoot);
paths.matlabDir = char(matlabDir);
paths.mpcDir = char(fullfile(matlabDir, "mpc"));
paths.autoDir = char(fullfile(matlabDir, "mpc_auto"));
paths.sfuncDir = char(fullfile(matlabDir, "sfunc_jsbsim"));
end
