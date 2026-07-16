function result = airdropx_run_and_export(varargin)
%AIRDROPX_RUN_AND_EXPORT Run the Simulink model and export CSV tuning data.
%
% Usage:
%   result = airdropx_run_and_export
%   result = airdropx_run_and_export("RunName", "theta7_thr056")
%   result = airdropx_run_and_export("Overrides", struct("target_altitude_m", 25))
%
% The output folder contains:
%   timeseries.csv  - logged signals sampled on one time vector
%   drop_table.csv  - one row per drop event
%   summary.csv     - altitude/drop summary metrics
%   params.csv      - key tuning parameters used for the run

opts = local_options(varargin{:});

cfg = local_setup_for_export(opts, false);
if opts.UpdateModel
    update_airdropx_model_architecture(cfg.model);
end
cfg = local_setup_for_export(opts, true);
local_configure_model_for_run(cfg);

out = sim(char(cfg.model), ...
    "StopTime", num2str(cfg.sim.stop_time_s, "%.15g"), ...
    "FixedStep", num2str(cfg.sim.dt_s, "%.15g"));
logs = out.logsout;

runName = string(opts.RunName);
if strlength(runName) == 0
    runName = "run_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
end

outputDir = string(opts.OutputDir);
if strlength(outputDir) == 0
    outputDir = string(fullfile(cfg.matlabDir, "results", runName));
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end

timeSeriesFile = fullfile(outputDir, "timeseries.csv");
dropTableFile = fullfile(outputDir, "drop_table.csv");
summaryFile = fullfile(outputDir, "summary.csv");
paramsFile = fullfile(outputDir, "params.csv");

timeTable = local_timeseries_table(logs);
writetable(timeTable, timeSeriesFile);

dropTable = airdropx_drop_table(logs, "OutputFile", dropTableFile);
report = airdropx_report(logs, "nw20", "HRef", cfg.control.target_altitude_m);
summaryTable = local_summary_table(report);
writetable(summaryTable, summaryFile);

paramsTable = local_params_table(cfg);
writetable(paramsTable, paramsFile);

result = struct();
result.cfg = cfg;
result.out = out;
result.report = report;
result.output_dir = outputDir;
result.timeseries_csv = string(timeSeriesFile);
result.drop_table_csv = string(dropTableFile);
result.summary_csv = string(summaryFile);
result.params_csv = string(paramsFile);
result.drop_table = dropTable;
result.timeseries = timeTable;

fprintf("AirdropX CSV exported:\n");
fprintf("  %s\n", timeSeriesFile);
fprintf("  %s\n", dropTableFile);
fprintf("  %s\n", summaryFile);
fprintf("  %s\n", paramsFile);
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.AircraftName = "MQ9_Reaper";
opts.IcName = "";
opts.Dt = [];
opts.Model = "untitled1";
opts.RunName = "";
opts.OutputDir = "";
opts.Overrides = struct();
opts.UpdateModel = true;

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

function cfg = local_setup_for_export(opts, applyOverrides)
if nargin < 2
    applyOverrides = true;
end

projectRoot = string(opts.ProjectRoot);
if strlength(projectRoot) == 0
    thisFile = mfilename("fullpath");
    matlabDir = fileparts(thisFile);
    projectRoot = string(fileparts(matlabDir));
else
    matlabDir = fullfile(projectRoot, "matlab");
end

addpath(char(matlabDir));
addpath(char(fullfile(matlabDir, "sfunc_jsbsim")));
addpath(char(fullfile(matlabDir, "vr")));

cfgArgs = { ...
    "ProjectRoot", projectRoot, ...
    "AircraftName", opts.AircraftName, ...
    "IcName", opts.IcName, ...
    "Model", opts.Model, ...
    "AssignBase", true};
if ~isempty(opts.Dt)
    cfgArgs = [cfgArgs, {"Dt", opts.Dt}];
end

cfg = airdropx_sim_params(cfgArgs{:});
if applyOverrides
    cfg = local_apply_overrides(cfg, opts.Overrides);
end
cd(char(projectRoot));

fprintf("AirdropX Simulink workspace initialized.\n");
fprintf("  projectRoot : %s\n", cfg.projectRoot);
fprintf("  aircraftName: %s\n", cfg.aircraftName);
fprintf("  icName      : %s\n", cfg.icName);
fprintf("  initial V   : %.3f m/s\n", cfg.initial.airspeed_mps);
fprintf("  initial theta: %.3f deg\n", cfg.initial.pitch_deg);
fprintf("  dt          : %.10g\n", cfg.sim.dt_s);
fprintf("  drop_mode   : %.0f (1=fixed, 2=CARP)\n", cfg.drop_mode);
end

function local_configure_model_for_run(cfg)
modelPath = fullfile(cfg.matlabDir, cfg.model + ".slx");
load_system(modelPath);
set_param(char(cfg.model), ...
    "StopTime", "airdropx_stop_time_s", ...
    "FixedStep", "dt", ...
    "SolverName", "FixedStepDiscrete");
end

function cfg = local_apply_overrides(cfg, overrides)
if isempty(overrides)
    local_publish_overrides(cfg);
    return;
end
if istable(overrides)
    overrides = table2struct(overrides, "ToScalar", true);
end
if ~isstruct(overrides)
    error("Overrides must be a scalar struct or table.");
end

cfg = local_set_if_present(cfg, overrides, "stop_time_s", ["sim", "stop_time_s"]);
cfg = local_set_if_present(cfg, overrides, "dt_s", ["sim", "dt_s"]);
cfg = local_set_if_present(cfg, overrides, "initial_airspeed_mps", ["initial", "airspeed_mps"]);
cfg = local_set_if_present(cfg, overrides, "initial_theta_deg", ["initial", "theta_deg"]);
cfg.initial.pitch_deg = cfg.initial.theta_deg;
cfg = local_set_if_present(cfg, overrides, "target_altitude_m", ["control", "target_altitude_m"]);
cfg = local_set_if_present(cfg, overrides, "initial_elevator_delta", ["control", "initial_elevator_delta"]);
cfg = local_set_if_present(cfg, overrides, "initial_throttle_cmd", ["control", "initial_throttle_cmd"]);
cfg = local_set_if_present(cfg, overrides, "drop_mass_signal_kg", ["control", "drop_mass_signal_kg"]);
cfg = local_set_if_present(cfg, overrides, "drop_mode", "drop_mode");
cfg = local_set_if_present(cfg, overrides, "wind_speed_mps", ["environment", "wind_speed_mps"]);
cfg = local_set_if_present(cfg, overrides, "wind_dir_from_deg", ["environment", "wind_dir_from_deg"]);
cfg = local_set_if_present(cfg, overrides, "fixed_drop_start_s", ["fixed_drop", "start_s"]);
cfg = local_set_if_present(cfg, overrides, "fixed_drop_interval_s", ["fixed_drop", "interval_s"]);

gainNames = [
    "Kp"
    "Kd"
    "u_limit"
    "u_rate_limit"
    "K_mass"
    "bias_rate_limit"
    "throttle_kp"
    "throttle_fixed"
    "throttle_alt_kp"
    "throttle_vz_kd"
    "v_ref_mps"
    "pitch_ref_deg"
    "pitch_kp"
    "pitch_limit"
    "pitch_rate_kd"
    "pitch_rate_limit"
    "dt_s"
    ];
for i = 1:numel(gainNames)
    name = gainNames(i);
    if isfield(overrides, name)
        cfg.control.pd_gains.(name) = double(overrides.(name));
    end
end
cfg.control.pd_gains.dt_s = cfg.sim.dt_s;
cfg.dt = cfg.sim.dt_s;
cfg.fixed_drop.pulse_s = cfg.sim.dt_s;

local_publish_overrides(cfg);
end

function cfg = local_set_if_present(cfg, overrides, name, path)
if ~isfield(overrides, name)
    return;
end
value = double(overrides.(name));
if isstring(path) && isscalar(path)
    cfg.(path) = value;
elseif numel(path) == 2
    cfg.(path(1)).(path(2)) = value;
else
    error("Unsupported override path for %s.", name);
end
end

function local_publish_overrides(cfg)
assignin("base", "airdropx_cfg", cfg);
assignin("base", "dt", cfg.sim.dt_s);
assignin("base", "airdropx_stop_time_s", cfg.sim.stop_time_s);
assignin("base", "airdropx_drop_mode", cfg.drop_mode);
assignin("base", "airdropx_wind_speed_mps", cfg.environment.wind_speed_mps);
assignin("base", "airdropx_wind_dir_from_deg", cfg.environment.wind_dir_from_deg);
assignin("base", "airdropx_initial_airspeed_mps", cfg.initial.airspeed_mps);
assignin("base", "airdropx_initial_theta_deg", cfg.initial.theta_deg);
assignin("base", "airdropx_initial_pitch_deg", cfg.initial.pitch_deg);
assignin("base", "airdropx_target_altitude_m", cfg.control.target_altitude_m);
assignin("base", "airdropx_initial_elevator_delta", cfg.control.initial_elevator_delta);
assignin("base", "airdropx_initial_throttle_cmd", cfg.control.initial_throttle_cmd);
assignin("base", "airdropx_drop_mass_signal_kg", cfg.control.drop_mass_signal_kg);
assignin("base", "airdropx_fixed_drop_start_s", cfg.fixed_drop.start_s);
assignin("base", "airdropx_fixed_drop_interval_s", cfg.fixed_drop.interval_s);
assignin("base", "airdropx_fixed_drop_pulse_s", cfg.fixed_drop.pulse_s);

g = cfg.control.pd_gains;
assignin("base", "airdropx_pd_gains", g);
assignin("base", "airdropx_pd_Kp", g.Kp);
assignin("base", "airdropx_pd_Kd", g.Kd);
assignin("base", "airdropx_pd_u_limit", g.u_limit);
assignin("base", "airdropx_pd_u_rate_limit", g.u_rate_limit);
assignin("base", "airdropx_pd_K_mass", g.K_mass);
assignin("base", "airdropx_pd_bias_rate_limit", g.bias_rate_limit);
assignin("base", "airdropx_pd_throttle_kp", g.throttle_kp);
assignin("base", "airdropx_pd_throttle_fixed", g.throttle_fixed);
assignin("base", "airdropx_pd_throttle_alt_kp", g.throttle_alt_kp);
assignin("base", "airdropx_pd_throttle_vz_kd", g.throttle_vz_kd);
assignin("base", "airdropx_pd_v_ref_mps", g.v_ref_mps);
assignin("base", "airdropx_pd_pitch_ref_deg", g.pitch_ref_deg);
assignin("base", "airdropx_pd_pitch_kp", g.pitch_kp);
assignin("base", "airdropx_pd_pitch_limit", g.pitch_limit);
assignin("base", "airdropx_pd_pitch_rate_kd", g.pitch_rate_kd);
assignin("base", "airdropx_pd_pitch_rate_limit", g.pitch_rate_limit);
assignin("base", "airdropx_pd_dt_s", g.dt_s);
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

function T = local_summary_table(report)
T = table( ...
    report.drop_count_final, ...
    report.h_err_mean, ...
    report.h_err_rms, ...
    report.h_err_max, ...
    report.h_err_p95, ...
    report.min_altitude, ...
    report.max_altitude, ...
    report.final_altitude, ...
    report.elevator_sat_rate, ...
    'VariableNames', { ...
        'drop_count_final', ...
        'h_err_mean_m', ...
        'h_err_rms_m', ...
        'h_err_max_m', ...
        'h_err_p95_m', ...
        'min_altitude_m', ...
        'max_altitude_m', ...
        'final_altitude_m', ...
        'elevator_sat_rate'});
end

function T = local_params_table(cfg)
names = [
    "initial_airspeed_mps"
    "initial_theta_deg"
    "target_altitude_m"
    "initial_elevator_delta"
    "initial_throttle_cmd"
    "pd_Kp"
    "pd_Kd"
    "pd_u_limit"
    "pd_u_rate_limit"
    "pd_K_mass"
    "pd_bias_rate_limit"
    "pd_throttle_kp"
    "pd_throttle_fixed"
    "pd_throttle_alt_kp"
    "pd_throttle_vz_kd"
    "pd_v_ref_mps"
    "pd_pitch_ref_deg"
    "pd_pitch_kp"
    "pd_pitch_limit"
    "pd_pitch_rate_kd"
    "pd_pitch_rate_limit"
    "pd_dt_s"
    "drop_mass_signal_kg"
    "drop_mode"
    ];

values = [
    cfg.initial.airspeed_mps
    cfg.initial.theta_deg
    cfg.control.target_altitude_m
    cfg.control.initial_elevator_delta
    cfg.control.initial_throttle_cmd
    cfg.control.pd_gains.Kp
    cfg.control.pd_gains.Kd
    cfg.control.pd_gains.u_limit
    cfg.control.pd_gains.u_rate_limit
    cfg.control.pd_gains.K_mass
    cfg.control.pd_gains.bias_rate_limit
    cfg.control.pd_gains.throttle_kp
    cfg.control.pd_gains.throttle_fixed
    cfg.control.pd_gains.throttle_alt_kp
    cfg.control.pd_gains.throttle_vz_kd
    cfg.control.pd_gains.v_ref_mps
    cfg.control.pd_gains.pitch_ref_deg
    cfg.control.pd_gains.pitch_kp
    cfg.control.pd_gains.pitch_limit
    cfg.control.pd_gains.pitch_rate_kd
    cfg.control.pd_gains.pitch_rate_limit
    cfg.control.pd_gains.dt_s
    cfg.control.drop_mass_signal_kg
    cfg.drop_mode
    ];

T = table(names, values, 'VariableNames', {'name', 'value'});
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

function [t, y] = local_signal(logs, name)
t = [];
y = [];

if isa(logs, "Simulink.SimulationOutput")
    try
        [t, y] = local_signal(logs.logsout, name);
    catch
    end
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
