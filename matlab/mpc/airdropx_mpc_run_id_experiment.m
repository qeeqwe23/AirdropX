function result = airdropx_mpc_run_id_experiment(varargin)
%AIRDROPX_MPC_RUN_ID_EXPERIMENT Run the standalone MPC ID Simulink model.
%
% This uses matlab/mpc/airdropx_mpc_id.slx, injects small actuator excitation,
% exports CSV data, and runs the offline identification/replay tools. It does
% not modify or use MPC as a Simulink controller.

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
    airdropx_mpc_create_id_model("TargetModel", modelName);
end

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    outputRoot = string(fullfile(matlabDir, "results", "mpc_id_experiment_" + stamp));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

cfg = airdropx_sim_params( ...
    "ProjectRoot", projectRoot, ...
    "Model", modelName, ...
    "AssignBase", true);
cfg.sim.stop_time_s = double(opts.StopTimeS);
cfg.control.target_altitude_m = double(opts.TargetAltitudeM);
cfg.control.pd_gains.v_ref_mps = double(opts.TargetAirspeedMps);
cfg.control.pd_gains.pitch_ref_deg = double(opts.TargetPitchDeg);
assignin("base", "airdropx_cfg", cfg);
assignin("base", "airdropx_stop_time_s", cfg.sim.stop_time_s);
assignin("base", "airdropx_target_altitude_m", cfg.control.target_altitude_m);
assignin("base", "airdropx_pd_v_ref_mps", cfg.control.pd_gains.v_ref_mps);
assignin("base", "airdropx_pd_pitch_ref_deg", cfg.control.pd_gains.pitch_ref_deg);

[elevExc, throttleExc] = local_default_excitation(opts.StopTimeS);
assignin("base", "airdropx_mpc_elevator_excitation", elevExc);
assignin("base", "airdropx_mpc_throttle_excitation", throttleExc);

if bdIsLoaded(modelName)
    close_system(char(modelName), 0);
end
load_system(char(modelPath));
local_prepare_id_model_for_batch(modelName);
set_param(char(modelName), ...
    "StopTime", "airdropx_stop_time_s", ...
    "FixedStep", "dt", ...
    "SolverName", "FixedStepDiscrete", ...
    "SignalLogging", "on", ...
    "SignalLoggingName", "logsout");

out = sim(char(modelName), ...
    "StopTime", num2str(cfg.sim.stop_time_s, "%.15g"), ...
    "FixedStep", num2str(cfg.sim.dt_s, "%.15g"));
close_system(char(modelName), 0);

logs = out.logsout;
T = local_timeseries_table(logs);
T.case_id = repmat(string(opts.CaseId), height(T), 1);
T.target_altitude_m = repmat(cfg.control.target_altitude_m, height(T), 1);
T.target_airspeed_mps = repmat(cfg.control.pd_gains.v_ref_mps, height(T), 1);
T.target_pitch_deg = repmat(cfg.control.pd_gains.pitch_ref_deg, height(T), 1);
T.mpc_elevator_excitation = local_sample_step(elevExc, T.time_s);
T.mpc_throttle_excitation = local_sample_step(throttleExc, T.time_s);

timeseriesCsv = fullfile(outputRoot, "id_timeseries.csv");
excitationCsv = fullfile(outputRoot, "excitation_inputs.csv");
modelMat = fullfile(outputRoot, "identified_model.mat");
summaryCsv = fullfile(outputRoot, "summary.csv");
writetable(T, timeseriesCsv);
writetable(table(elevExc(:, 1), elevExc(:, 2), ...
    'VariableNames', {'time_s', 'elevator_excitation'}), ...
    fullfile(outputRoot, "elevator_excitation.csv"));
writetable(table(throttleExc(:, 1), throttleExc(:, 2), ...
    'VariableNames', {'time_s', 'throttle_excitation'}), ...
    fullfile(outputRoot, "throttle_excitation.csv"));
writetable(local_combined_excitation_table(elevExc, throttleExc), excitationCsv);

summary = airdropx_mpc_evaluate_csv(timeseriesCsv, "OutputFile", summaryCsv);
identified = airdropx_mpc_identify_from_csv(timeseriesCsv, ...
    "OutputMat", modelMat, ...
    "UseLastTrim", false);
replay = airdropx_mpc_demo_offline(timeseriesCsv);

result = struct();
result.output_root = outputRoot;
result.timeseries_csv = string(timeseriesCsv);
result.excitation_csv = string(excitationCsv);
result.summary_csv = string(summaryCsv);
result.identified_model_mat = string(modelMat);
result.summary = summary;
result.identification = identified;
result.replay = replay;
result.out = out;

fprintf("AirdropX MPC ID experiment written:\n");
fprintf("  %s\n", timeseriesCsv);
fprintf("  %s\n", excitationCsv);
fprintf("  %s\n", modelMat);
end

function [elevExc, throttleExc] = local_default_excitation(stopTimeS)
stopTimeS = double(stopTimeS);
elevEvents = [
    0.0,  0.000
    4.0,  0.018
    4.7,  0.018
    4.8,  0.000
    6.2, -0.018
    6.9, -0.018
    7.0,  0.000
    11.4,  0.014
    12.0,  0.014
    12.1,  0.000
    14.0, -0.014
    14.6, -0.014
    14.7,  0.000
    ];
thrEvents = [
    0.0,  0.000
    5.0,  0.025
    6.1,  0.025
    6.2,  0.000
    8.0, -0.025
    9.1, -0.025
    9.2,  0.000
    13.0,  0.020
    14.1,  0.020
    14.2,  0.000
    16.0, -0.020
    17.1, -0.020
    17.2,  0.000
    ];

elevExc = local_clip_events(elevEvents, stopTimeS);
throttleExc = local_clip_events(thrEvents, stopTimeS);
end

function events = local_clip_events(events, stopTimeS)
events = events(events(:, 1) <= stopTimeS, :);
if events(end, 1) < stopTimeS
    events(end + 1, :) = [stopTimeS, 0.0];
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
    "h_err"
    "delta_m_signal"
    "drop_trim_bias"
    "u_total"
    "u_out"
    "saturated"
    ];

[tRef, ~] = local_signal(logs, signals(1));
if isempty(tRef)
    error("Could not find reference log signal: %s", signals(1));
end

T = table(tRef(:), 'VariableNames', {'time_s'});
for i = 1:numel(signals)
    name = signals(i);
    [t, y] = local_signal(logs, name);
    T.(matlab.lang.makeValidName(name)) = local_sample_at_times(t, y, tRef);
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

function values = local_sample_step(events, tq)
events = double(events);
tq = double(tq(:));
values = zeros(size(tq));
for i = 1:numel(tq)
    idx = find(events(:, 1) <= tq(i), 1, "last");
    if isempty(idx)
        values(i) = events(1, 2);
    else
        values(i) = events(idx, 2);
    end
end
end

function T = local_combined_excitation_table(elevExc, throttleExc)
times = unique([elevExc(:, 1); throttleExc(:, 1)]);
T = table(times, ...
    local_sample_step(elevExc, times), ...
    local_sample_step(throttleExc, times), ...
    'VariableNames', {'time_s', 'elevator_excitation', 'throttle_excitation'});
end

function local_prepare_id_model_for_batch(modelName)
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

for name = ["MPC_ID_elevator_excitation", "MPC_ID_throttle_excitation"]
    blockPath = char(string(modelName) + "/" + name);
    if getSimulinkBlockHandle(blockPath) >= 0
        try
            set_param(blockPath, "SampleTime", "dt");
        catch
        end
    end
end
end

function opts = local_options(varargin)
opts.Model = "airdropx_mpc_id";
opts.OutputRoot = "";
opts.StopTimeS = 22.0;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.CaseId = "id_mixed_pulses";
opts.RecreateModel = false;

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
