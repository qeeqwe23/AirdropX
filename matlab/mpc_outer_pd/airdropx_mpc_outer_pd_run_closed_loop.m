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
T.target_n_m = repmat(double(opts.CarpTargetNorthM), height(T), 1);
T.target_e_m = repmat(double(opts.CarpTargetEastM), height(T), 1);
T.drop_mode = repmat(double(opts.DropMode), height(T), 1);

timeseriesCsv = fullfile(outputRoot, "closed_loop_timeseries.csv");
fullTimeseriesCsv = fullfile(outputRoot, "closed_loop_timeseries_full.csv");
summaryCsv = fullfile(outputRoot, "summary.csv");
dropDetailsCsv = fullfile(outputRoot, "drop_impact_points.csv");
dropScatterPng = fullfile(outputRoot, "drop_scatter.png");
dashboardPng = fullfile(outputRoot, "dashboard.png");
carpCepPng = fullfile(outputRoot, "carp_cep.png");
carpCepPointsCsv = fullfile(outputRoot, "carp_cep_points.csv");
if double(opts.WarmupTimeS) > 0
    TFull.target_altitude_m = repmat(double(opts.TargetAltitudeM), height(TFull), 1);
    TFull.target_airspeed_mps = repmat(double(opts.TargetAirspeedMps), height(TFull), 1);
    TFull.target_pitch_deg = repmat(double(opts.TargetPitchDeg), height(TFull), 1);
    TFull.target_n_m = repmat(double(opts.CarpTargetNorthM), height(TFull), 1);
    TFull.target_e_m = repmat(double(opts.CarpTargetEastM), height(TFull), 1);
    TFull.drop_mode = repmat(double(opts.DropMode), height(TFull), 1);
    writetable(TFull, fullTimeseriesCsv);
end
writetable(T, timeseriesCsv);
summary = airdropx_mpc_evaluate_csv(timeseriesCsv, ...
    "TargetNorthM", opts.CarpTargetNorthM, ...
    "TargetEastM", opts.CarpTargetEastM, ...
    "DropTargetNorthM", double(opts.CarpTargetNorthM) + double(opts.CarpTargetOffsetNorthM(:)), ...
    "DropTargetEastM", double(opts.CarpTargetEastM) + double(opts.CarpTargetOffsetEastM(:)), ...
    "DropMode", opts.DropMode, ...
    "OutputFile", summaryCsv, ...
    "DropDetailsFile", dropDetailsCsv, ...
    "PlotFile", dropScatterPng);
plotFiles = local_export_airdropx_plots(T, opts, dashboardPng, carpCepPng, carpCepPointsCsv, opts.ShowPlots);

result = struct();
result.output_root = outputRoot;
result.timeseries_csv = string(timeseriesCsv);
result.full_timeseries_csv = string(fullTimeseriesCsv);
result.summary_csv = string(summaryCsv);
result.drop_details_csv = string(dropDetailsCsv);
result.drop_scatter_png = string(dropScatterPng);
result.dashboard_png = string(dashboardPng);
result.carp_cep_png = string(carpCepPng);
result.carp_cep_points_csv = string(carpCepPointsCsv);
result.summary = summary;
result.plot_files = plotFiles;
result.out = out;

fprintf("AirdropX MPC outer + PD inner result written:\n");
fprintf("  %s\n", timeseriesCsv);
fprintf("  %s\n", summaryCsv);
fprintf("  %s\n", dropDetailsCsv);
fprintf("  %s\n", dropScatterPng);
if isfield(plotFiles, "dashboard_png")
    fprintf("  %s\n", dashboardPng);
end
if isfield(plotFiles, "carp_cep_png")
    fprintf("  %s\n", carpCepPng);
end
if isfield(plotFiles, "carp_cep_points_csv")
    fprintf("  %s\n", carpCepPointsCsv);
end
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
    "DropMode", opts.DropMode, ...
    "CarpTargetNorthM", opts.CarpTargetNorthM, ...
    "CarpTargetEastM", opts.CarpTargetEastM, ...
    "CarpReleaseWindowS", opts.CarpReleaseWindowS, ...
    "CarpIntervalS", opts.CarpIntervalS, ...
    "CarpDropTotal", opts.CarpDropTotal, ...
    "CarpMinSafeAltitudeM", opts.CarpMinSafeAltitudeM, ...
    "CarpTargetOffsetNorthM", opts.CarpTargetOffsetNorthM, ...
    "CarpTargetOffsetEastM", opts.CarpTargetOffsetEastM, ...
    "CarpReleaseDelayS", opts.CarpReleaseDelayS, ...
    "BallisticKDragScale", opts.BallisticKDragScale, ...
    "BallisticKDrag", opts.BallisticKDrag, ...
    "InitialAirspeedMps", opts.InitialAirspeedMps, ...
    "InitialAltitudeM", opts.InitialAltitudeM, ...
    "InitialPitchDeg", opts.InitialPitchDeg, ...
    "InitialFlightPathDeg", opts.InitialFlightPathDeg, ...
    "InitialHeadingDeg", opts.InitialHeadingDeg, ...
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
    '''DropMode'',%.15g,' ...
    '''CarpTargetNorthM'',%.15g,' ...
    '''CarpTargetEastM'',%.15g,' ...
    '''CarpReleaseWindowS'',%.15g,' ...
    '''CarpIntervalS'',%.15g,' ...
    '''CarpDropTotal'',%.15g,' ...
    '''CarpMinSafeAltitudeM'',%.15g,' ...
    '''InitialAirspeedMps'',%.15g,' ...
    '''InitialAltitudeM'',%.15g,' ...
    '''InitialPitchDeg'',%.15g,' ...
    '''InitialFlightPathDeg'',%.15g,' ...
    '''InitialHeadingDeg'',%.15g,' ...
    '''InitialElevatorDelta'',%.15g,' ...
    '''InitialThrottleCmd'',%.15g);%s'], ...
    char(modelName), ...
    local_sim_stop_time(opts), ...
    double(opts.TargetAltitudeM), ...
    double(opts.TargetAirspeedMps), ...
    double(opts.TargetPitchDeg), ...
    double(opts.ControlAltitudeBiasM), ...
    double(opts.DropMode), ...
    double(opts.CarpTargetNorthM), ...
    double(opts.CarpTargetEastM), ...
    double(opts.CarpReleaseWindowS), ...
    double(opts.CarpIntervalS), ...
    double(opts.CarpDropTotal), ...
    double(opts.CarpMinSafeAltitudeM), ...
    double(opts.InitialAirspeedMps), ...
    double(opts.InitialAltitudeM), ...
    double(opts.InitialPitchDeg), ...
    double(opts.InitialFlightPathDeg), ...
    double(opts.InitialHeadingDeg), ...
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
if double(opts.DropMode) >= 1.5
    return;
end
dropStartS = double(opts.FixedDropStartS) + max(0.0, double(opts.WarmupTimeS));
assignin("base", "airdropx_fixed_drop_start_s", dropStartS);
end

function suffix = local_warmup_callback_suffix(opts)
suffix = "";
suffix = suffix + sprintf("assignin('base','airdropx_carp_target_offset_n_m',%s);", ...
    mat2str(double(opts.CarpTargetOffsetNorthM(:))));
suffix = suffix + sprintf("assignin('base','airdropx_carp_target_offset_e_m',%s);", ...
    mat2str(double(opts.CarpTargetOffsetEastM(:))));
for i = 1:4
    offsetN = local_vector_value(opts.CarpTargetOffsetNorthM, i);
    offsetE = local_vector_value(opts.CarpTargetOffsetEastM, i);
    suffix = suffix + sprintf("assignin('base','airdropx_carp_target_offset_n_%d_m',%.15g);", i, offsetN);
    suffix = suffix + sprintf("assignin('base','airdropx_carp_target_offset_e_%d_m',%.15g);", i, offsetE);
end
suffix = suffix + sprintf("assignin('base','airdropx_carp_release_delay_s',%.15g);", ...
    double(opts.CarpReleaseDelayS));
if isfinite(double(opts.BallisticKDrag))
    suffix = suffix + sprintf("assignin('base','airdropx_ballistics_k_drag',%.15g);", ...
        double(opts.BallisticKDrag));
elseif double(opts.BallisticKDragScale) ~= 1.0
    suffix = suffix + sprintf("assignin('base','airdropx_ballistics_k_drag',airdropx_ballistics_k_drag*%.15g);", ...
        double(opts.BallisticKDragScale));
end
if opts.ShiftDropScheduleForWarmup && double(opts.DropMode) < 1.5
    dropStartS = double(opts.FixedDropStartS) + max(0.0, double(opts.WarmupTimeS));
    suffix = suffix + sprintf("assignin('base','airdropx_fixed_drop_start_s',%.15g);", dropStartS);
end
if ~isempty(opts.ConfigOverrides)
    suffix = suffix + sprintf("assignin('base','airdropx_mpc_outer_pd_config_overrides',%s);", ...
        local_value_expr(opts.ConfigOverrides));
end
end

function expr = local_value_expr(value)
if isstruct(value)
    names = fieldnames(value);
    parts = strings(1, 2 * numel(names));
    for i = 1:numel(names)
        parts(2 * i - 1) = "'" + string(names{i}) + "'";
        parts(2 * i) = local_value_expr(value.(names{i}));
    end
    expr = "struct(" + strjoin(parts, ",") + ")";
elseif isnumeric(value) || islogical(value)
    expr = string(mat2str(value));
elseif ischar(value) || isstring(value)
    expr = "'" + replace(string(value), "'", "''") + "'";
else
    error("Unsupported ConfigOverrides value type: %s", class(value));
end
end

function value = local_vector_value(x, idx)
x = double(x(:));
if numel(x) >= idx
    value = x(idx);
else
    value = 0.0;
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

function plotFiles = local_export_airdropx_plots(T, opts, dashboardFile, carpCepFile, carpCepDataFile, showPlots)
plotFiles = struct();
showPlots = local_should_show_plots(showPlots);
logs = local_table_to_logs(T);

try
    fig = airdropx_plot(logs, "dashboard");
    local_save_figure(fig, dashboardFile, [1200 900], showPlots);
    plotFiles.dashboard_png = string(dashboardFile);
catch err
    warning("AirdropX:MpcOuterPdPlotExportFailed", ...
        "Failed to export MPC+PD dashboard plot: %s", err.message);
end

if double(opts.DropMode) < 1.5
    return;
end

try
    mc = airdropx_carp_monte_carlo(logs, ...
        "Mode", "fourdrop", ...
        "TargetN", opts.CarpTargetNorthM, ...
        "TargetE", opts.CarpTargetEastM);
    scatterTable = local_monte_carlo_table(mc);
    writetable(scatterTable, carpCepDataFile);
    plotFiles.carp_cep_points_csv = string(carpCepDataFile);
    fig = airdropx_plot(logs, "carp", ...
        "TargetN", opts.CarpTargetNorthM, ...
        "TargetE", opts.CarpTargetEastM, ...
        "MonteCarlo", mc);
catch err
    warning("AirdropX:MpcOuterPdCarpScatterDataFailed", ...
        "Failed to compute MPC+PD CARP Monte Carlo scatter: %s", err.message);
    fig = airdropx_plot(logs, "carp", ...
        "TargetN", opts.CarpTargetNorthM, ...
        "TargetE", opts.CarpTargetEastM);
end

try
    local_save_figure(fig, carpCepFile, [1200 850], showPlots);
    plotFiles.carp_cep_png = string(carpCepFile);
catch err
    warning("AirdropX:MpcOuterPdPlotExportFailed", ...
        "Failed to export MPC+PD CARP/CEP plot: %s", err.message);
end
end

function logs = local_table_to_logs(T)
logs = struct();
logs.time = T.time_s(:);
names = string(T.Properties.VariableNames);
for i = 1:numel(names)
    name = names(i);
    if name == "time_s"
        continue;
    end
    logs.(char(name)) = T.(char(name));
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
opts.DropMode = 1.0;
opts.CarpTargetNorthM = 1000.0;
opts.CarpTargetEastM = 0.0;
opts.CarpReleaseWindowS = 0.7;
opts.CarpIntervalS = 0.5;
opts.CarpDropTotal = 4.0;
opts.CarpMinSafeAltitudeM = 15.0;
opts.CarpTargetOffsetNorthM = zeros(4, 1);
opts.CarpTargetOffsetEastM = zeros(4, 1);
opts.CarpReleaseDelayS = 0.0;
opts.BallisticKDragScale = 1.0;
opts.BallisticKDrag = NaN;
opts.InitialAirspeedMps = 55.0;
opts.InitialAltitudeM = NaN;
opts.InitialPitchDeg = NaN;
opts.InitialFlightPathDeg = 2.4;
opts.InitialHeadingDeg = 0.0;
opts.InitialElevatorDelta = 0.0;
opts.InitialThrottleCmd = 0.80;
opts.ConfigOverrides = [];
opts.RecreateModel = true;
opts.DisableVRForBatch = true;
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
end
