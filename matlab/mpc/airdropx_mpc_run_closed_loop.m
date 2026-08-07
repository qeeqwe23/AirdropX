function result = airdropx_mpc_run_closed_loop(varargin)
%AIRDROPX_MPC_RUN_CLOSED_LOOP Run preserved direct-MPC SLX and export CSV.

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
mpcBlock = char(modelName + "/MPC_Controller");
oldMpcParams = get_param(mpcBlock, "Parameters");

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

    local_assign_run_workspace(opts);
    local_set_mpc_block_parameter(mpcBlock, opts.ModelMat);

    set_param(char(modelName), ...
        "StopTime", num2str(double(opts.StopTimeS), "%.15g"), ...
        "FixedStep", "dt", ...
        "SolverName", "FixedStepDiscrete", ...
        "SignalLogging", "on", ...
        "SignalLoggingName", "logsout");

    out = sim(char(modelName), ...
        "StopTime", num2str(double(opts.StopTimeS), "%.15g"), ...
        "FixedStep", num2str(simCfg.sim.dt_s, "%.15g"));

    set_param(mpcBlock, "Parameters", oldMpcParams);
    set_param(char(modelName), "InitFcn", oldInitFcn);
    set_param(char(modelName), "Dirty", "off");
catch ME
    if bdIsLoaded(char(modelName))
        set_param(mpcBlock, "Parameters", oldMpcParams);
        set_param(char(modelName), "InitFcn", oldInitFcn);
        set_param(char(modelName), "Dirty", "off");
    end
    rethrow(ME);
end

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    outputRoot = string(fullfile(matlabDir, "results", ...
        "mpc_closed_loop_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss"))));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

T = local_timeseries_table(out.logsout, opts);
csvPath = fullfile(outputRoot, "closed_loop_timeseries.csv");
writetable(T, csvPath);

summaryPath = fullfile(outputRoot, "summary.csv");
dropPath = fullfile(outputRoot, "drop_details.csv");
report = airdropx_mpc_evaluate_csv(csvPath, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", opts.TargetPitchDeg, ...
    "AfterDropTime", opts.AfterDropTime, ...
    "TargetNorthM", opts.TargetNorthM, ...
    "TargetEastM", opts.TargetEastM, ...
    "DropTargetNorthM", opts.DropTargetNorthM, ...
    "DropTargetEastM", opts.DropTargetEastM, ...
    "OutputFile", summaryPath, ...
    "DropDetailsFile", dropPath);

result = struct();
result.output_root = outputRoot;
result.timeseries_csv = string(csvPath);
result.summary_csv = string(summaryPath);
result.drop_details_csv = string(dropPath);
result.timeseries = T;
result.summary = report.segments;
result.drop_table = report.drop_table;
result.out = out;

fprintf("AirdropX MPC closed-loop exported:\n  %s\n", csvPath);
end

function local_assign_run_workspace(opts)
assignin("base", "airdropx_stop_time_s", double(opts.StopTimeS));
assignin("base", "airdropx_target_altitude_m", double(opts.TargetAltitudeM));
assignin("base", "airdropx_pd_v_ref_mps", double(opts.TargetAirspeedMps));
controlPitchDeg = double(opts.TargetPitchDeg) + double(opts.ControlPitchBiasDeg);
assignin("base", "airdropx_pd_pitch_ref_deg", controlPitchDeg);
assignin("base", "airdropx_mpc_control_altitude_bias_m", double(opts.ControlAltitudeBiasM));
assignin("base", "airdropx_mpc_control_pitch_bias_deg", double(opts.ControlPitchBiasDeg));

assignin("base", "airdropx_drop_mode", double(opts.DropMode));
assignin("base", "airdropx_fixed_drop_start_s", double(opts.FixedDropStartS));
assignin("base", "airdropx_fixed_drop_interval_s", double(opts.FixedDropIntervalS));
assignin("base", "airdropx_fixed_drop_total", double(opts.FixedDropTotal));

assignin("base", "airdropx_carp_target_n_m", double(opts.TargetNorthM));
assignin("base", "airdropx_carp_target_e_m", double(opts.TargetEastM));
assignin("base", "airdropx_carp_target_offset_n_m", double(opts.DropTargetNorthM(:)));
assignin("base", "airdropx_carp_target_offset_e_m", double(opts.DropTargetEastM(:)));
for i = 1:min(4, numel(opts.DropTargetNorthM))
    assignin("base", sprintf("airdropx_carp_target_offset_n_%d_m", i), double(opts.DropTargetNorthM(i)));
end
for i = 1:min(4, numel(opts.DropTargetEastM))
    assignin("base", sprintf("airdropx_carp_target_offset_e_%d_m", i), double(opts.DropTargetEastM(i)));
end

assignin("base", "airdropx_initial_elevator_delta", double(opts.InitialElevatorDelta));
assignin("base", "airdropx_initial_throttle_cmd", double(opts.InitialThrottleCmd));

overrides = opts.ConfigOverrides;
if ~isstruct(overrides)
    overrides = struct();
end
overrides.reference.command_h_m = double(opts.TargetAltitudeM);
overrides.reference.altitude_bias_m = double(opts.ControlAltitudeBiasM);
overrides.reference.h_m = double(opts.TargetAltitudeM) + double(opts.ControlAltitudeBiasM);
overrides.reference.v_mps = double(opts.TargetAirspeedMps);
overrides.reference.pitch_deg = controlPitchDeg;
assignin("base", "airdropx_mpc_config_overrides", overrides);
end

function local_set_mpc_block_parameter(mpcBlock, modelMat)
modelMat = string(modelMat);
if strlength(modelMat) == 0
    set_param(mpcBlock, "Parameters", "''");
    return;
end
if ~isfile(modelMat)
    error("AirdropX:MPC:MissingModelMat", "MPC model mat not found: %s", modelMat);
end
assignin("base", "airdropx_mpc_model_mat_path", char(modelMat));
set_param(mpcBlock, "Parameters", "airdropx_mpc_model_mat_path");
end

function T = local_timeseries_table(logs, opts)
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
    "throttle_cmd"
    "drop_count"
    "selected_drop_cmd"
    "drop_cmd"
    "pos_n_m"
    "pos_e_m"
    "heading_deg"
    "wind_n_mps"
    "wind_e_mps"
    "actual_release_n_m"
    "actual_release_e_m"
    "actual_release_alt_m"
    "release_airspeed_mps"
    "release_heading_deg"
    "release_wind_n_mps"
    "release_wind_e_mps"
    "predicted_impact_n_m"
    "predicted_impact_e_m"
    "drop_trim_bias"
    "h_err"
    "saturated"
    "u_out"
    "u_pd"
    "u_total"
    ];

[tRef, ~] = local_signal(logs, "altitude_m");
if isempty(tRef)
    [tRef, ~] = local_signal(logs, "mpc_state_1");
end
if isempty(tRef)
    error("AirdropX:MPC:MissingLog", "logsout does not contain altitude_m or mpc_state_1.");
end
T = table(tRef(:), repmat(string(opts.CaseId), numel(tRef), 1), ...
    'VariableNames', {'time_s', 'case_id'});

for i = 1:numel(signals)
    [t, y] = local_signal_with_alias(logs, signals(i));
    if isempty(t)
        T.(char(signals(i))) = NaN(height(T), 1);
    elseif isequal(t(:), tRef(:))
        T.(char(signals(i))) = y(:);
    else
        T.(char(signals(i))) = interp1(t(:), y(:), tRef(:), "linear", "extrap");
    end
end
if all(isnan(T.elevator_delta)) && ~all(isnan(T.elevator_cmd_norm))
    T.elevator_delta = T.elevator_cmd_norm;
end
if all(isnan(T.throttle_cmd)) && ~all(isnan(T.throttle_norm))
    T.throttle_cmd = T.throttle_norm;
end

T.target_altitude_m = double(opts.TargetAltitudeM) * ones(height(T), 1);
T.target_airspeed_mps = double(opts.TargetAirspeedMps) * ones(height(T), 1);
T.target_pitch_deg = double(opts.TargetPitchDeg) * ones(height(T), 1);
end

function [t, y] = local_signal_with_alias(logs, name)
[t, y] = local_signal(logs, name);
if ~isempty(t)
    return;
end
switch string(name)
    case "altitude_m"
        [t, y] = local_signal(logs, "mpc_state_1");
    case "vz_up_mps"
        [t, y] = local_signal(logs, "mpc_state_2");
    case "airspeed_mps"
        [t, y] = local_signal(logs, "mpc_state_3");
    case "pitch_deg"
        [t, y] = local_signal(logs, "mpc_state_4");
    case "mass_kg"
        [t, y] = local_signal(logs, "mpc_state_5");
    case "cg_x_m"
        [t, y] = local_signal(logs, "mpc_state_6");
end
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
opts.Model = "airdropx_mpc_closed_loop";
opts.ModelMat = "";
opts.OutputRoot = "";
opts.CaseId = "closed_loop_001";
opts.StopTimeS = 35.0;
opts.AfterDropTime = 10.0;
opts.DropMode = 1.0;
opts.FixedDropStartS = 10.0;
opts.FixedDropIntervalS = 0.2;
opts.FixedDropTotal = 4.0;
opts.InitialAirspeedMps = 50.0;
opts.InitialAltitudeM = 20.0;
opts.InitialPitchDeg = 4.0;
opts.InitialFlightPathDeg = 0.0;
opts.InitialHeadingDeg = 0.0;
opts.InitialElevatorDelta = 0.0;
opts.InitialThrottleCmd = 0.80;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.TargetPitchDeg = 4.0;
opts.ControlAltitudeBiasM = 0.0;
opts.ControlPitchBiasDeg = 0.0;
opts.TargetNorthM = 1000.0;
opts.TargetEastM = 0.0;
opts.DropTargetNorthM = [0.8; 1.6; 2.4; 3.8];
opts.DropTargetEastM = [0.0; 0.0; 0.0; 0.0];
opts.ConfigOverrides = struct();
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
