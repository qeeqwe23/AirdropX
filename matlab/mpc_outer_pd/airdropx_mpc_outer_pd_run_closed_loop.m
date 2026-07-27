function result = airdropx_mpc_outer_pd_run_closed_loop(varargin)
%AIRDROPX_MPC_OUTER_PD_RUN_CLOSED_LOOP Run MPC outer + PD inner closed loop.

opts = local_options(varargin{:});

thisFile = mfilename("fullpath");
outerDir = string(fileparts(thisFile));
matlabDir = string(fileparts(outerDir));

addpath(char(matlabDir));
addpath(char(fullfile(matlabDir, "mpc")));
addpath(char(outerDir));
addpath(char(fullfile(matlabDir, "sfunc_jsbsim")));
addpath(char(fullfile(matlabDir, "vr")));

modelName = string(opts.Model);
modelPath = fullfile(outerDir, modelName + ".slx");
if opts.RecreateModel || ~isfile(modelPath)
    airdropx_mpc_outer_pd_create_model("TargetModel", modelName, "DisableVR", false);
end

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    outputRoot = string(fullfile(matlabDir, "results", "mpc_outer_pd_closed_loop_" + stamp));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

local_setup(opts, modelName);

if bdIsLoaded(modelName)
    close_system(char(modelName), 0);
end
load_system(char(modelPath));
local_setup(opts, modelName);
if opts.DisableVRForBatch
    local_prepare_model_for_batch(modelName);
end
set_param(char(modelName), "InitFcn", local_setup_callback(opts, modelName));
set_param(char(modelName), ...
    "StopTime", "airdropx_stop_time_s", ...
    "FixedStep", "dt", ...
    "SolverName", "FixedStepDiscrete", ...
    "SignalLogging", "on", ...
    "SignalLoggingName", "logsout");

out = sim(char(modelName), ...
    "StopTime", num2str(local_sim_stop_time(opts), "%.15g"));
close_system(char(modelName), 0);

TFull = local_timeseries_table(out.logsout);
T = local_crop_warmup(TFull, opts.WarmupTimeS);
T.target_altitude_m = repmat(double(opts.TargetAltitudeM), height(T), 1);
T.target_airspeed_mps = repmat(double(opts.TargetAirspeedMps), height(T), 1);
T.target_pitch_deg = repmat(double(opts.TargetPitchDeg), height(T), 1);

timeseriesCsv = fullfile(outputRoot, "closed_loop_timeseries.csv");
fullTimeseriesCsv = fullfile(outputRoot, "closed_loop_timeseries_full.csv");
summaryCsv = fullfile(outputRoot, "summary.csv");
if double(opts.WarmupTimeS) > 0
    TFull.target_altitude_m = repmat(double(opts.TargetAltitudeM), height(TFull), 1);
    TFull.target_airspeed_mps = repmat(double(opts.TargetAirspeedMps), height(TFull), 1);
    TFull.target_pitch_deg = repmat(double(opts.TargetPitchDeg), height(TFull), 1);
    writetable(TFull, fullTimeseriesCsv);
end
writetable(T, timeseriesCsv);
summary = airdropx_mpc_evaluate_csv(timeseriesCsv, "OutputFile", summaryCsv);

result = struct();
result.output_root = outputRoot;
result.timeseries_csv = string(timeseriesCsv);
result.full_timeseries_csv = string(fullTimeseriesCsv);
result.summary_csv = string(summaryCsv);
result.summary = summary;
result.out = out;

fprintf("AirdropX MPC outer + PD inner result written:\n");
fprintf("  %s\n", timeseriesCsv);
fprintf("  %s\n", summaryCsv);
end

function local_setup(opts, modelName)
if evalin("base", "exist('airdropx_mpc_outer_pd_config_overrides','var')")
    evalin("base", "clear('airdropx_mpc_outer_pd_config_overrides')");
end
airdropx_mpc_outer_pd_setup_workspace( ...
    "Model", modelName, ...
    "StopTimeS", local_sim_stop_time(opts), ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", opts.TargetPitchDeg, ...
    "ControlAltitudeBiasM", opts.ControlAltitudeBiasM, ...
    "InitialAirspeedMps", opts.InitialAirspeedMps, ...
    "InitialAltitudeM", opts.InitialAltitudeM, ...
    "InitialPitchDeg", opts.InitialPitchDeg, ...
    "InitialFlightPathDeg", opts.InitialFlightPathDeg, ...
    "InitialElevatorDelta", opts.InitialElevatorDelta, ...
    "InitialThrottleCmd", opts.InitialThrottleCmd, ...
    "ConfigOverrides", opts.ConfigOverrides);
local_apply_warmup_schedule(opts);
end

function callback = local_setup_callback(opts, modelName)
callback = sprintf([ ...
    'airdropx_mpc_outer_pd_setup_workspace(''Model'',''%s'',' ...
    '''StopTimeS'',%.15g,' ...
    '''TargetAltitudeM'',%.15g,' ...
    '''TargetAirspeedMps'',%.15g,' ...
    '''TargetPitchDeg'',%.15g,' ...
    '''ControlAltitudeBiasM'',%.15g,' ...
    '''InitialAirspeedMps'',%.15g,' ...
    '''InitialAltitudeM'',%.15g,' ...
    '''InitialPitchDeg'',%.15g,' ...
    '''InitialFlightPathDeg'',%.15g,' ...
    '''InitialElevatorDelta'',%.15g,' ...
    '''InitialThrottleCmd'',%.15g);%s'], ...
    char(modelName), ...
    local_sim_stop_time(opts), ...
    double(opts.TargetAltitudeM), ...
    double(opts.TargetAirspeedMps), ...
    double(opts.TargetPitchDeg), ...
    double(opts.ControlAltitudeBiasM), ...
    double(opts.InitialAirspeedMps), ...
    double(opts.InitialAltitudeM), ...
    double(opts.InitialPitchDeg), ...
    double(opts.InitialFlightPathDeg), ...
    double(opts.InitialElevatorDelta), ...
    double(opts.InitialThrottleCmd), ...
    local_warmup_callback_suffix(opts));
end

function totalStopTimeS = local_sim_stop_time(opts)
totalStopTimeS = double(opts.StopTimeS) + max(0.0, double(opts.WarmupTimeS));
end

function local_apply_warmup_schedule(opts)
if ~opts.ShiftDropScheduleForWarmup
    return;
end
dropStartS = double(opts.FixedDropStartS) + max(0.0, double(opts.WarmupTimeS));
assignin("base", "airdropx_fixed_drop_start_s", dropStartS);
end

function suffix = local_warmup_callback_suffix(opts)
suffix = '';
if opts.ShiftDropScheduleForWarmup
    dropStartS = double(opts.FixedDropStartS) + max(0.0, double(opts.WarmupTimeS));
    suffix = sprintf("assignin('base','airdropx_fixed_drop_start_s',%.15g);", dropStartS);
end
end

function local_prepare_model_for_batch(modelName)
blocks = find_system(char(modelName), ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "RegExp", "on", ...
    "Name", ".*VR.*");
try
    blocks = [blocks; find_system(char(modelName), ...
        "LookUnderMasks", "all", ...
        "FollowLinks", "on", ...
        "BlockType", "S-Function", ...
        "FunctionName", "vrsfunc")];
catch
end
for i = 1:numel(blocks)
    try
        set_param(blocks{i}, "Commented", "on");
    catch
    end
end
end

function T = local_timeseries_table(logs)
signals = [
    "altitude_m"
    "vz_up_mps"
    "airspeed_mps"
    "groundspeed_mps"
    "pitch_deg"
    "roll_deg"
    "heading_deg"
    "mass_kg"
    "cg_x_m"
    "pos_n_m"
    "pos_e_m"
    "elevator_cmd_norm"
    "throttle_norm"
    "drop_count"
    ];

[tRef, ~] = local_signal(logs, signals(1));
if isempty(tRef)
    [tRef, ~] = local_signal(logs, local_signal_alias(signals(1)));
end
if isempty(tRef)
    error("Could not find reference log signal: %s", signals(1));
end

T = table(tRef(:), 'VariableNames', {'time_s'});
for i = 1:numel(signals)
    name = signals(i);
    [t, y] = local_signal(logs, name);
    if isempty(t)
        [t, y] = local_signal(logs, local_signal_alias(name));
    end
    T.(matlab.lang.makeValidName(name)) = local_sample_at_times(t, y, tRef);
end
end

function T = local_crop_warmup(TFull, warmupTimeS)
warmupTimeS = max(0.0, double(warmupTimeS));
if warmupTimeS <= 0
    T = TFull;
    return;
end
T = TFull(TFull.time_s >= warmupTimeS, :);
if isempty(T)
    error("WarmupTimeS %.3f leaves no samples to evaluate.", warmupTimeS);
end
T.time_s = T.time_s - warmupTimeS;
end

function alias = local_signal_alias(name)
switch string(name)
    case "altitude_m"
        alias = "mpc_outer_pd_state_1";
    case "vz_up_mps"
        alias = "mpc_outer_pd_state_2";
    case "airspeed_mps"
        alias = "mpc_outer_pd_state_3";
    case "pitch_deg"
        alias = "mpc_outer_pd_state_4";
    case "mass_kg"
        alias = "mpc_outer_pd_state_5";
    case "cg_x_m"
        alias = "mpc_outer_pd_state_6";
    otherwise
        alias = "";
end
end

function [t, y] = local_signal(logs, name)
t = [];
y = [];
if isa(logs, "Simulink.SimulationOutput")
    [t, y] = local_signal(logs.logsout, name);
    return;
end
if isa(logs, "Simulink.SimulationData.Dataset")
    for i = 1:logs.numElements
        el = logs.get(i);
        if string(el.Name) == string(name)
            ts = el.Values;
            t = ts.Time(:);
            y = squeeze(ts.Data);
            y = y(:);
            return;
        end
    end
end
end

function yq = local_sample_at_times(t, y, tq)
if isempty(t) || isempty(y)
    yq = NaN(size(tq));
    return;
end
t = t(:);
y = y(:);
yq = NaN(size(tq));
for i = 1:numel(tq)
    [~, idx] = min(abs(t - tq(i)));
    yq(i) = y(idx);
end
end

function opts = local_options(varargin)
opts.Model = "airdropx_mpc_outer_pd_closed_loop";
opts.OutputRoot = "";
opts.StopTimeS = 22.0;
opts.WarmupTimeS = 0.0;
opts.FixedDropStartS = 10.0;
opts.ShiftDropScheduleForWarmup = true;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.ControlAltitudeBiasM = 0.95;
opts.InitialAirspeedMps = 55.0;
opts.InitialAltitudeM = NaN;
opts.InitialPitchDeg = NaN;
opts.InitialFlightPathDeg = 2.4;
opts.InitialElevatorDelta = 0.0;
opts.InitialThrottleCmd = 0.80;
opts.ConfigOverrides = [];
opts.RecreateModel = true;
opts.DisableVRForBatch = true;

if mod(numel(varargin), 2) ~= 0
    error("Options must be name-value pairs.");
end

for i = 1:2:numel(varargin)
    name = string(varargin{i});
    value = varargin{i + 1};
    if ~isfield(opts, name)
        error("Unknown option: %s", name);
    end
    opts.(name) = value;
end
end
