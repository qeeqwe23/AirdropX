function result = airdropx_run_and_export(varargin)
%AIRDROPX_RUN_AND_EXPORT Run the Simulink model and export CSV tuning data.
%
% Usage:
%   result = airdropx_run_and_export
%   result = airdropx_run_and_export("RunName", "theta7_thr056")
%   result = airdropx_run_and_export("DropMode", 2, "StopTime", 30)
%   result = airdropx_run_and_export("Overrides", struct("target_altitude_m", 25))
%   result = airdropx_run_and_export("ShowPlots", true)
%
% The output folder contains:
%   timeseries.csv  - logged signals sampled on one time vector
%   drop_table.csv  - one row per drop event
%   dashboard.png   - repository dashboard plot
%   carp_cep.png    - CARP/CEP target-circle plot when CARP signals exist
%   carp_cep_points.csv - Monte Carlo scatter data behind carp_cep.png
%   summary.csv     - altitude/drop summary metrics
%   params.csv      - key tuning parameters used for the run

opts = local_options(varargin{:});

cfg = local_setup_for_export(opts, false);
if opts.UpdateModel
    update_airdropx_model_architecture(cfg.model);
end
cfg = local_setup_for_export(opts, true);
modelConfig = local_configure_model_for_run(cfg);
modelCleanup = onCleanup(@() local_restore_model_config(modelConfig));

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
dashboardFile = fullfile(outputDir, "dashboard.png");
carpCepFile = fullfile(outputDir, "carp_cep.png");
carpCepDataFile = fullfile(outputDir, "carp_cep_points.csv");
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

plotFiles = local_export_plots(logs, cfg, dashboardFile, carpCepFile, carpCepDataFile, opts.ShowPlots);

result = struct();
result.cfg = cfg;
result.out = out;
result.report = report;
result.output_dir = outputDir;
result.timeseries_csv = string(timeSeriesFile);
result.drop_table_csv = string(dropTableFile);
result.dashboard_png = string(dashboardFile);
result.carp_cep_png = string(carpCepFile);
result.carp_cep_points_csv = string(carpCepDataFile);
result.summary_csv = string(summaryFile);
result.params_csv = string(paramsFile);
result.drop_table = dropTable;
result.plot_files = plotFiles;
result.timeseries = timeTable;

fprintf("AirdropX CSV exported:\n");
fprintf("  %s\n", timeSeriesFile);
fprintf("  %s\n", dropTableFile);
fprintf("  %s\n", dashboardFile);
if isfield(plotFiles, "carp_cep_png") && strlength(plotFiles.carp_cep_png) > 0
    fprintf("  %s\n", carpCepFile);
end
if isfield(plotFiles, "carp_cep_points_csv") && strlength(plotFiles.carp_cep_points_csv) > 0
    fprintf("  %s\n", carpCepDataFile);
end
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
opts.DropMode = [];
opts.StopTime = [];
opts.UpdateModel = true;
opts.ShowPlots = true;

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

opts = local_apply_option_aliases(opts);
end

function opts = local_apply_option_aliases(opts)
if isempty(opts.DropMode) && isempty(opts.StopTime)
    return;
end
overrides = opts.Overrides;
if isempty(overrides)
    overrides = struct();
end
if istable(overrides)
    overrides = table2struct(overrides, "ToScalar", true);
end
if ~isstruct(overrides)
    error("Overrides must be a scalar struct or table.");
end
if ~isempty(opts.DropMode)
    overrides.drop_mode = double(opts.DropMode);
end
if ~isempty(opts.StopTime)
    overrides.stop_time_s = double(opts.StopTime);
end
opts.Overrides = overrides;
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
if applyOverrides && ~isempty(opts.Overrides)
    cfgArgs = local_append_initial_overrides(cfgArgs, opts.Overrides);
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

function modelConfig = local_configure_model_for_run(cfg)
modelPath = fullfile(cfg.matlabDir, cfg.model + ".slx");
load_system(modelPath);
modelConfig = struct();
modelConfig.model = char(cfg.model);
modelConfig.initFcn = get_param(char(cfg.model), "InitFcn");
set_param(char(cfg.model), ...
    "InitFcn", "", ...
    "StopTime", "airdropx_stop_time_s", ...
    "FixedStep", "dt", ...
    "SolverName", "FixedStepDiscrete");
end

function local_restore_model_config(modelConfig)
if isempty(modelConfig) || ~isstruct(modelConfig) || ~isfield(modelConfig, "model")
    return;
end
try
    if bdIsLoaded(modelConfig.model)
        set_param(modelConfig.model, "InitFcn", modelConfig.initFcn);
        set_param(modelConfig.model, "Dirty", "off");
    end
catch
end
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
cfg = local_set_if_present(cfg, overrides, "initial_altitude_m", ["initial", "altitude_m"]);
cfg = local_set_if_present(cfg, overrides, "initial_theta_deg", ["initial", "theta_deg"]);
cfg = local_set_if_present(cfg, overrides, "initial_heading_deg", ["initial", "heading_deg"]);
cfg = local_set_if_present(cfg, overrides, "initial_flight_path_deg", ["initial", "flight_path_deg"]);
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
cfg = local_set_if_present(cfg, overrides, "carp_target_n_m", ["carp", "target_n_m"]);
cfg = local_set_if_present(cfg, overrides, "carp_target_e_m", ["carp", "target_e_m"]);
cfg = local_set_if_present(cfg, overrides, "carp_release_window_s", ["carp", "release_window_s"]);
cfg = local_set_if_present(cfg, overrides, "carp_interval_s", ["carp", "interval_s"]);
cfg = local_set_if_present(cfg, overrides, "carp_drop_total", ["carp", "drop_total"]);
cfg = local_set_if_present(cfg, overrides, "carp_min_safe_alt_m", ["carp", "min_safe_alt_m"]);

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

function cfgArgs = local_append_initial_overrides(cfgArgs, overrides)
if istable(overrides)
    overrides = table2struct(overrides, "ToScalar", true);
end
if ~isstruct(overrides)
    return;
end
cfgArgs = local_append_cfg_arg_if_present(cfgArgs, overrides, "initial_airspeed_mps", "InitialAirspeedMps");
cfgArgs = local_append_cfg_arg_if_present(cfgArgs, overrides, "initial_altitude_m", "InitialAltitudeM");
cfgArgs = local_append_cfg_arg_if_present(cfgArgs, overrides, "initial_theta_deg", "InitialPitchDeg");
cfgArgs = local_append_cfg_arg_if_present(cfgArgs, overrides, "initial_flight_path_deg", "InitialFlightPathDeg");
cfgArgs = local_append_cfg_arg_if_present(cfgArgs, overrides, "initial_heading_deg", "InitialHeadingDeg");
end

function cfgArgs = local_append_cfg_arg_if_present(cfgArgs, overrides, overrideName, cfgName)
if isfield(overrides, overrideName)
    cfgArgs = [cfgArgs, {cfgName, double(overrides.(overrideName))}];
end
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
assignin("base", "airdropx_initial_altitude_m", cfg.initial.altitude_m);
assignin("base", "airdropx_initial_theta_deg", cfg.initial.theta_deg);
assignin("base", "airdropx_initial_pitch_deg", cfg.initial.pitch_deg);
assignin("base", "airdropx_initial_heading_deg", cfg.initial.heading_deg);
assignin("base", "airdropx_initial_flight_path_deg", cfg.initial.flight_path_deg);
assignin("base", "airdropx_target_altitude_m", cfg.control.target_altitude_m);
assignin("base", "airdropx_initial_elevator_delta", cfg.control.initial_elevator_delta);
assignin("base", "airdropx_initial_throttle_cmd", cfg.control.initial_throttle_cmd);
assignin("base", "airdropx_drop_mass_signal_kg", cfg.control.drop_mass_signal_kg);
assignin("base", "airdropx_fixed_drop_start_s", cfg.fixed_drop.start_s);
assignin("base", "airdropx_fixed_drop_interval_s", cfg.fixed_drop.interval_s);
assignin("base", "airdropx_fixed_drop_pulse_s", cfg.fixed_drop.pulse_s);
assignin("base", "airdropx_carp_target_n_m", cfg.carp.target_n_m);
assignin("base", "airdropx_carp_target_e_m", cfg.carp.target_e_m);
assignin("base", "airdropx_carp_release_window_s", cfg.carp.release_window_s);
assignin("base", "airdropx_carp_interval_s", cfg.carp.interval_s);
assignin("base", "airdropx_carp_drop_total", cfg.carp.drop_total);
assignin("base", "airdropx_carp_min_safe_alt_m", cfg.carp.min_safe_alt_m);

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
    "drop_cmd"
    "selected_drop_cmd"
    "fixed_drop_cmd"
    "carp_drop_cmd"
    "release_latched"
    "in_window"
    "low_alt_safe"
    "t_to_release_s"
    "release_n_m"
    "release_e_m"
    "predicted_impact_n_m"
    "predicted_impact_e_m"
    "miss_distance_m"
    "cep50_to_target_m"
    "actual_release_n_m"
    "actual_release_e_m"
    "actual_release_alt_m"
    "release_airspeed_mps"
    "release_heading_deg"
    "release_wind_n_mps"
    "release_wind_e_mps"
    "schedule_done"
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
    "initial_altitude_m"
    "initial_theta_deg"
    "initial_flight_path_deg"
    "initial_heading_deg"
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
    "carp_target_n_m"
    "carp_target_e_m"
    "carp_release_window_s"
    "carp_interval_s"
    "carp_drop_total"
    "carp_min_safe_alt_m"
    ];

values = [
    cfg.initial.airspeed_mps
    cfg.initial.altitude_m
    cfg.initial.theta_deg
    cfg.initial.flight_path_deg
    cfg.initial.heading_deg
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
    cfg.carp.target_n_m
    cfg.carp.target_e_m
    cfg.carp.release_window_s
    cfg.carp.interval_s
    cfg.carp.drop_total
    cfg.carp.min_safe_alt_m
    ];

T = table(names, values, 'VariableNames', {'name', 'value'});
end

function plotFiles = local_export_plots(logs, cfg, dashboardFile, carpCepFile, carpCepDataFile, showPlots)
plotFiles = struct();
showPlots = local_should_show_plots(showPlots);

try
    fig = airdropx_plot(logs, "dashboard");
    local_save_figure(fig, dashboardFile, [1200 900], showPlots);
    plotFiles.dashboard_png = string(dashboardFile);
catch err
    warning("AirdropX:PlotExportFailed", ...
        "Failed to export dashboard plot: %s", err.message);
end

if cfg.drop_mode < 1.5
    return;
end

try
    mc = airdropx_carp_monte_carlo(logs, ...
        "Mode", "fourdrop", ...
        "TargetN", cfg.carp.target_n_m, ...
        "TargetE", cfg.carp.target_e_m);
    scatterTable = local_monte_carlo_table(mc);
    writetable(scatterTable, carpCepDataFile);
    plotFiles.carp_cep_points_csv = string(carpCepDataFile);
    fig = airdropx_plot(logs, "carp", ...
        "TargetN", cfg.carp.target_n_m, ...
        "TargetE", cfg.carp.target_e_m, ...
        "MonteCarlo", mc);
catch
    fig = airdropx_plot(logs, "carp", ...
        "TargetN", cfg.carp.target_n_m, ...
        "TargetE", cfg.carp.target_e_m);
end

try
    local_save_figure(fig, carpCepFile, [1200 850], showPlots);
    plotFiles.carp_cep_png = string(carpCepFile);
catch err
    warning("AirdropX:PlotExportFailed", ...
        "Failed to export CARP/CEP plot: %s", err.message);
end
end

function T = local_monte_carlo_table(mc)
if isfield(mc, "batches") && ~isempty(mc.batches)
    dropIndex = [];
    sampleIndex = [];
    offsetE = [];
    offsetN = [];
    impactE = [];
    impactN = [];
    for k = 1:numel(mc.batches)
        n = numel(mc.batches(k).offset_e_m);
        dropIndex = [dropIndex; repmat(k, n, 1)]; %#ok<AGROW>
        sampleIndex = [sampleIndex; (1:n).']; %#ok<AGROW>
        offsetE = [offsetE; mc.batches(k).offset_e_m(:)]; %#ok<AGROW>
        offsetN = [offsetN; mc.batches(k).offset_n_m(:)]; %#ok<AGROW>
        impactE = [impactE; mc.batches(k).impact_e_m(:)]; %#ok<AGROW>
        impactN = [impactN; mc.batches(k).impact_n_m(:)]; %#ok<AGROW>
    end
else
    n = numel(mc.offset_e_m);
    dropIndex = ones(n, 1);
    sampleIndex = (1:n).';
    offsetE = mc.offset_e_m(:);
    offsetN = mc.offset_n_m(:);
    impactE = mc.impact_e_m(:);
    impactN = mc.impact_n_m(:);
end
radial = hypot(offsetE, offsetN);
T = table(dropIndex, sampleIndex, offsetE, offsetN, radial, impactE, impactN, ...
    'VariableNames', {'drop_index', 'sample_index', 'offset_e_m', 'offset_n_m', ...
    'radial_error_m', 'impact_e_m', 'impact_n_m'});
end

function showPlots = local_should_show_plots(requested)
showPlots = logical(requested);
if ~showPlots
    return;
end
try
    showPlots = usejava('awt');
catch
    showPlots = false;
end
end

function local_save_figure(fig, path, sizePx, keepOpen)
if isempty(fig) || ~ishandle(fig)
    return;
end
if nargin < 3 || isempty(sizePx)
    sizePx = [1000 750];
end
if nargin < 4
    keepOpen = false;
end
if keepOpen
    set(fig, "Visible", "on", "WindowStyle", "normal");
    figure(fig);
else
    set(fig, "Visible", "off");
    cleanup = onCleanup(@() close(fig));
end
pos = get(fig, "Position");
set(fig, "Position", [pos(1) pos(2) sizePx(1) sizePx(2)]);
drawnow;
outDir = fileparts(path);
if strlength(string(outDir)) > 0 && ~isfolder(outDir)
    mkdir(outDir);
end
try
    exportgraphics(fig, path, "Resolution", 180, "BackgroundColor", "current");
catch
    saveas(fig, path);
end
if keepOpen
    set(fig, "Visible", "on", "WindowStyle", "normal");
    figure(fig);
    drawnow;
    shg;
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
