function result = airdropx_run_and_export(varargin)
%AIRDROPX_RUN_AND_EXPORT Run the Simulink model and export CSV tuning data.
%
% Usage:
%   result = airdropx_run_and_export
%   result = airdropx_run_and_export("RunName", "theta7_thr056")
%
% The output folder contains:
%   timeseries.csv  - logged signals sampled on one time vector
%   drop_table.csv  - one row per drop event
%   summary.csv     - altitude/drop summary metrics
%   params.csv      - key tuning parameters used for the run

opts = local_options(varargin{:});

cfg = local_setup_for_export(opts);
update_airdropx_model_architecture(cfg.model);

out = sim(char(cfg.model));
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
report = airdropx_report(logs, "nw20");
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

function cfg = local_setup_for_export(opts)
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
    "pd_v_ref_mps"
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
    cfg.control.pd_gains.v_ref_mps
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
