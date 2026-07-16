function result = airdropx_mpc_run_closed_loop(varargin)
%AIRDROPX_MPC_RUN_CLOSED_LOOP Run the standalone MPC closed-loop model.
%
% This does not modify matlab/untitled1.slx. It simulates
% matlab/mpc/airdropx_mpc_closed_loop.slx and exports a compact CSV.

opts = local_options(varargin{:});

thisFile = mfilename("fullpath");
mpcDir = string(fileparts(thisFile));
matlabDir = string(fileparts(mpcDir));
projectRoot = string(fileparts(matlabDir));

addpath(char(matlabDir));
addpath(char(mpcDir));
addpath(char(fullfile(matlabDir, "sfunc_jsbsim")));
addpath(char(fullfile(matlabDir, "vr")));

modelName = string(opts.Model);
modelPath = fullfile(mpcDir, modelName + ".slx");
if opts.RecreateModel || ~isfile(modelPath)
    airdropx_mpc_create_closed_loop_model( ...
        "TargetModel", modelName, ...
        "ModelMat", opts.ModelMat);
end

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    outputRoot = string(fullfile(matlabDir, "results", "mpc_closed_loop_" + stamp));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

airdropx_mpc_setup_id_workspace( ...
    "Model", modelName, ...
    "StopTimeS", opts.StopTimeS, ...
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
    "Force", true);

if bdIsLoaded(modelName)
    close_system(char(modelName), 0);
end
load_system(char(modelPath));
airdropx_mpc_setup_id_workspace( ...
    "Model", modelName, ...
    "StopTimeS", opts.StopTimeS, ...
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
    "Force", true);
local_prepare_model_for_batch(modelName);
set_param(char(modelName), "InitFcn", local_setup_callback(opts, modelName));
set_param(char(modelName), ...
    "StopTime", "airdropx_stop_time_s", ...
    "FixedStep", "dt", ...
    "SolverName", "FixedStepDiscrete", ...
    "SignalLogging", "on", ...
    "SignalLoggingName", "logsout");

out = sim(char(modelName), ...
    "StopTime", num2str(double(opts.StopTimeS), "%.15g"));
close_system(char(modelName), 0);

T = local_timeseries_table(out.logsout);
T.target_altitude_m = repmat(double(opts.TargetAltitudeM), height(T), 1);
T.target_airspeed_mps = repmat(double(opts.TargetAirspeedMps), height(T), 1);
T.target_pitch_deg = repmat(double(opts.TargetPitchDeg), height(T), 1);

timeseriesCsv = fullfile(outputRoot, "closed_loop_timeseries.csv");
summaryCsv = fullfile(outputRoot, "summary.csv");
writetable(T, timeseriesCsv);
summary = airdropx_mpc_evaluate_csv(timeseriesCsv, "OutputFile", summaryCsv);

result = struct();
result.output_root = outputRoot;
result.timeseries_csv = string(timeseriesCsv);
result.summary_csv = string(summaryCsv);
result.summary = summary;
result.out = out;

fprintf("AirdropX MPC closed-loop result written:\n");
fprintf("  %s\n", timeseriesCsv);
fprintf("  %s\n", summaryCsv);
end

function callback = local_setup_callback(opts, modelName)
callback = sprintf([ ...
    'airdropx_mpc_setup_id_workspace(''Model'',''%s'',' ...
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
    '''InitialThrottleCmd'',%.15g,' ...
    '''Force'',true);'], ...
    char(modelName), ...
    double(opts.StopTimeS), ...
    double(opts.TargetAltitudeM), ...
    double(opts.TargetAirspeedMps), ...
    double(opts.TargetPitchDeg), ...
    double(opts.ControlAltitudeBiasM), ...
    double(opts.InitialAirspeedMps), ...
    double(opts.InitialAltitudeM), ...
    double(opts.InitialPitchDeg), ...
    double(opts.InitialFlightPathDeg), ...
    double(opts.InitialElevatorDelta), ...
    double(opts.InitialThrottleCmd));
end

function local_prepare_model_for_batch(modelName)
blocks = find_system(char(modelName), ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "RegExp", "on", ...
    "Name", ".*VR.*");
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

function alias = local_signal_alias(name)
switch string(name)
    case "altitude_m"
        alias = "mpc_state_1";
    case "vz_up_mps"
        alias = "mpc_state_2";
    case "airspeed_mps"
        alias = "mpc_state_3";
    case "pitch_deg"
        alias = "mpc_state_4";
    case "mass_kg"
        alias = "mpc_state_5";
    case "cg_x_m"
        alias = "mpc_state_6";
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
opts.Model = "airdropx_mpc_closed_loop";
opts.ModelMat = "";
opts.OutputRoot = "";
opts.StopTimeS = 22.0;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.ControlAltitudeBiasM = 0.95;
opts.InitialAirspeedMps = 55.5;
opts.InitialAltitudeM = NaN;
opts.InitialPitchDeg = NaN;
opts.InitialFlightPathDeg = 2.4;
opts.InitialElevatorDelta = 0.0;
opts.InitialThrottleCmd = 0.80;
opts.RecreateModel = true;

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
