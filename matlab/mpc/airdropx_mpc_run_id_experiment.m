function result = airdropx_mpc_run_id_experiment(varargin)
%AIRDROPX_MPC_RUN_ID_EXPERIMENT Run preserved ID SLX and export measured CSV.

opts = local_options(varargin{:});
thisDir = fileparts(mfilename("fullpath"));
matlabDir = fileparts(thisDir);
projectRoot = fileparts(matlabDir);
addpath(matlabDir);
addpath(thisDir);
addpath(fullfile(matlabDir, "sfunc_jsbsim"));

modelName = string(opts.Model);
modelPath = fullfile(thisDir, modelName + ".slx");
load_system(modelPath);
oldInitFcn = get_param(char(modelName), "InitFcn");

try
    set_param(char(modelName), "InitFcn", "");
    simCfg = airdropx_sim_params( ...
        "ProjectRoot", projectRoot, ...
        "Model", modelName, ...
        "AssignBase", true, ...
        "InitialAirspeedMps", opts.InitialAirspeedMps, ...
        "InitialAltitudeM", opts.InitialAltitudeM, ...
        "InitialPitchDeg", opts.InitialPitchDeg, ...
        "InitialFlightPathDeg", opts.InitialFlightPathDeg, ...
        "InitialHeadingDeg", opts.InitialHeadingDeg);
    assignin("base", "airdropx_stop_time_s", double(opts.StopTimeS));
    assignin("base", "airdropx_drop_mode", double(opts.DropMode));
    assignin("base", "airdropx_fixed_drop_start_s", double(opts.FixedDropStartS));
    assignin("base", "airdropx_fixed_drop_interval_s", double(opts.FixedDropIntervalS));
    assignin("base", "airdropx_fixed_drop_total", double(opts.FixedDropTotal));

    profile = airdropx_mpc_excitation_profile( ...
        "Dt", simCfg.sim.dt_s, ...
        "StopTimeS", opts.StopTimeS, ...
        "ElevatorAmplitudes", opts.ElevatorAmplitudes, ...
        "ElevatorFrequenciesHz", opts.ElevatorFrequenciesHz, ...
        "ThrottleAmplitudes", opts.ThrottleAmplitudes, ...
        "ThrottleFrequenciesHz", opts.ThrottleFrequenciesHz, ...
        "ElevatorMin", opts.ElevatorMin, ...
        "ElevatorMax", opts.ElevatorMax, ...
        "ThrottleMin", opts.ThrottleMin, ...
        "ThrottleMax", opts.ThrottleMax);
    assignin("base", "airdropx_mpc_elevator_excitation", profile.elevator);
    assignin("base", "airdropx_mpc_throttle_excitation", profile.throttle);

    set_param(char(modelName), ...
        "StopTime", num2str(double(opts.StopTimeS), "%.15g"), ...
        "FixedStep", "dt", ...
        "SolverName", "FixedStepDiscrete", ...
        "SignalLogging", "on", ...
        "SignalLoggingName", "logsout");
    out = sim(char(modelName), ...
        "StopTime", num2str(double(opts.StopTimeS), "%.15g"), ...
        "FixedStep", num2str(simCfg.sim.dt_s, "%.15g"));
    set_param(char(modelName), "InitFcn", oldInitFcn);
    set_param(char(modelName), "Dirty", "off");
catch ME
    if bdIsLoaded(char(modelName))
        set_param(char(modelName), "InitFcn", oldInitFcn);
        set_param(char(modelName), "Dirty", "off");
    end
    rethrow(ME);
end

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    outputRoot = string(fullfile(matlabDir, "results", ...
        "mpc_id_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss"))));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

T = local_timeseries_table(out.logsout, string(opts.CaseId));
csvPath = fullfile(outputRoot, "id_timeseries.csv");
writetable(T, csvPath);

result = struct();
result.output_root = outputRoot;
result.timeseries_csv = string(csvPath);
result.timeseries = T;
result.out = out;
result.profile = profile;

fprintf("AirdropX MPC ID data exported:\n  %s\n", csvPath);
end

function T = local_timeseries_table(logs, caseId)
signals = [
    "altitude_m"
    "vz_up_mps"
    "airspeed_mps"
    "pitch_deg"
    "mass_kg"
    "cg_x_m"
    "elevator_delta"
    "elevator_cmd_norm"
    "throttle_norm"
    "drop_count"
    "pos_n_m"
    "pos_e_m"
    "heading_deg"
    "wind_n_mps"
    "wind_e_mps"
    ];

[tRef, ~] = local_signal(logs, signals(1));
data = table(tRef(:), repmat(caseId, numel(tRef), 1), ...
    'VariableNames', {'time_s', 'case_id'});
for i = 1:numel(signals)
    [t, y] = local_signal(logs, signals(i));
    if isempty(t)
        data.(char(signals(i))) = NaN(height(data), 1);
    elseif isequal(t(:), tRef(:))
        data.(char(signals(i))) = y(:);
    else
        data.(char(signals(i))) = interp1(t(:), y(:), tRef(:), "linear", "extrap");
    end
end
if all(isnan(data.elevator_delta)) && ~all(isnan(data.elevator_cmd_norm))
    data.elevator_delta = data.elevator_cmd_norm;
end
T = data;
end

function [t, y] = local_signal(logs, name)
t = [];
y = [];
try
    el = logs.get(string(name));
    if isempty(el)
        return;
    end
    values = el.Values;
    t = double(values.Time(:));
    y = double(values.Data(:));
catch
end
end

function opts = local_options(varargin)
opts.Model = "airdropx_mpc_id";
opts.OutputRoot = "";
opts.CaseId = "id_case_001";
opts.StopTimeS = 60.0;
opts.DropMode = 1.0;
opts.FixedDropStartS = 12.0;
opts.FixedDropIntervalS = 8.0;
opts.FixedDropTotal = 4.0;
opts.InitialAirspeedMps = 50.0;
opts.InitialAltitudeM = 35.0;
opts.InitialPitchDeg = 4.0;
opts.InitialFlightPathDeg = 0.0;
opts.InitialHeadingDeg = 0.0;
opts.ElevatorAmplitudes = [0.030; 0.022; 0.016];
opts.ElevatorFrequenciesHz = [0.09; 0.19; 0.33];
opts.ThrottleAmplitudes = [0.022; 0.016; 0.010];
opts.ThrottleFrequenciesHz = [0.07; 0.17; 0.29];
opts.ElevatorMin = -0.10;
opts.ElevatorMax = 0.10;
opts.ThrottleMin = -0.06;
opts.ThrottleMax = 0.06;
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
