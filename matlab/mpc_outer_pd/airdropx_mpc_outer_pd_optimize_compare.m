function result = airdropx_mpc_outer_pd_optimize_compare(varargin)
%AIRDROPX_MPC_OUTER_PD_OPTIMIZE_COMPARE Optimize outer/inner MPC and compare.

opts = local_options(varargin{:});

thisFile = mfilename("fullpath");
outerDir = string(fileparts(thisFile));
matlabDir = string(fileparts(outerDir));

addpath(char(matlabDir));
addpath(char(fullfile(matlabDir, "mpc")));
addpath(char(outerDir));

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    outputRoot = string(fullfile(matlabDir, "results", "mpc_outer_pd_compare_" + stamp));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

baseline = local_run_direct_mpc_baseline(opts, outputRoot);
baselineMetrics = local_score(baseline.summary.segments, opts);

candidates = local_candidates(opts);
records = table();
best = [];
recreateModel = true;
for i = 1:min(opts.MaxIterations, height(candidates))
    rowCfg = candidates(i, :);
    runRoot = fullfile(outputRoot, sprintf("outer_pd_iter_%03d", i));
    overrides = local_overrides(rowCfg);
    fprintf("AirdropX outer+PD optimize %d/%d: v0=%.3f gamma=%.3f bias=%.3f kp=%.4f kd=%.4f\n", ...
        i, min(opts.MaxIterations, height(candidates)), ...
        rowCfg.initial_airspeed_mps, rowCfg.initial_flight_path_deg, ...
        rowCfg.control_altitude_bias_m, rowCfg.inner_kp, rowCfg.inner_kd);

    try
        simResult = airdropx_mpc_outer_pd_run_closed_loop( ...
            "OutputRoot", runRoot, ...
            "StopTimeS", opts.StopTimeS, ...
            "TargetAltitudeM", opts.TargetAltitudeM, ...
            "TargetAirspeedMps", opts.TargetAirspeedMps, ...
            "TargetPitchDeg", opts.TargetPitchDeg, ...
            "InitialAirspeedMps", rowCfg.initial_airspeed_mps, ...
            "InitialFlightPathDeg", rowCfg.initial_flight_path_deg, ...
            "ControlAltitudeBiasM", rowCfg.control_altitude_bias_m, ...
            "InitialElevatorDelta", opts.InitialElevatorDelta, ...
            "InitialThrottleCmd", opts.InitialThrottleCmd, ...
            "ConfigOverrides", overrides, ...
            "RecreateModel", recreateModel);
        recreateModel = false;
        metrics = local_score(simResult.summary.segments, opts);
        improved = metrics.score < baselineMetrics.score - opts.MinImprovement;
        rec = local_record_row(i, "ok", improved, rowCfg, metrics, string(runRoot), "");

        candidate = struct();
        candidate.score = metrics.score;
        candidate.improved = improved;
        candidate.params = rowCfg;
        candidate.metrics = metrics;
        candidate.output_root = string(simResult.output_root);
        candidate.summary_csv = string(simResult.summary_csv);
        candidate.timeseries_csv = string(simResult.timeseries_csv);
        if isempty(best) || candidate.score < best.score
            best = candidate;
            local_write_json(fullfile(outputRoot, "best_run.json"), best);
        end
    catch ME
        metrics = local_empty_metrics();
        rec = local_record_row(i, "failed", false, rowCfg, metrics, string(runRoot), ME.message);
    end

    records = [records; rec]; %#ok<AGROW>
    writetable(records, fullfile(outputRoot, "iteration_summary.csv"));

    if opts.StopOnPositive && ~isempty(best) && best.improved
        break;
    end
end

comparison = table( ...
    ["direct_mpc_baseline"; "outer_mpc_pd_inner_best"], ...
    [baselineMetrics.score; best.score], ...
    [false; best.improved], ...
    [baselineMetrics.after_drop_h_err_rms_m; best.metrics.after_drop_h_err_rms_m], ...
    [baselineMetrics.after_drop_v_err_rms_mps; best.metrics.after_drop_v_err_rms_mps], ...
    [baselineMetrics.after_drop_pitch_err_rms_deg; best.metrics.after_drop_pitch_err_rms_deg], ...
    [baselineMetrics.last_5s_h_err_rms_m; best.metrics.last_5s_h_err_rms_m], ...
    [string(baseline.output_root); best.output_root], ...
    'VariableNames', { ...
        'case_name', 'score', 'positive_optimization', ...
        'after_drop_h_err_rms_m', 'after_drop_v_err_rms_mps', ...
        'after_drop_pitch_err_rms_deg', 'last_5s_h_err_rms_m', 'output_root'});
writetable(comparison, fullfile(outputRoot, "comparison_summary.csv"));

if ~best.improved
    error("Outer MPC + PD inner did not beat direct MPC baseline. See %s", outputRoot);
end

result = struct();
result.output_root = outputRoot;
result.baseline = baseline;
result.baseline_metrics = baselineMetrics;
result.best = best;
result.iterations = records;
result.comparison = comparison;

fprintf("AirdropX outer MPC + PD inner positive optimization complete:\n");
fprintf("  baseline score: %.6f\n", baselineMetrics.score);
fprintf("  best score    : %.6f\n", best.score);
fprintf("  comparison    : %s\n", fullfile(outputRoot, "comparison_summary.csv"));
end

function baseline = local_run_direct_mpc_baseline(opts, outputRoot)
baselineRoot = fullfile(outputRoot, "direct_mpc_baseline");
baseline = airdropx_mpc_run_closed_loop( ...
    "OutputRoot", baselineRoot, ...
    "StopTimeS", opts.StopTimeS, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", opts.TargetPitchDeg, ...
    "InitialAirspeedMps", opts.BaselineInitialAirspeedMps, ...
    "InitialFlightPathDeg", opts.BaselineInitialFlightPathDeg, ...
    "ControlAltitudeBiasM", opts.BaselineControlAltitudeBiasM, ...
    "InitialElevatorDelta", opts.InitialElevatorDelta, ...
    "InitialThrottleCmd", opts.InitialThrottleCmd, ...
    "RecreateModel", true);
end

function candidates = local_candidates(opts)
base = table( ...
    [55.0; 55.5; 55.0; 55.0; 55.5; 54.5; 55.0; 55.2; 54.8; 55.0; 55.5; 54.5; 56.0; 55.0; 55.0], ...
    [2.4; 2.4; 2.6; 2.2; 2.6; 2.4; 2.4; 2.5; 2.5; 2.4; 2.6; 2.6; 2.4; 2.4; 2.4], ...
    [0.95; 0.95; 0.95; 0.95; 0.90; 0.95; 0.85; 0.95; 0.95; 0.90; 0.90; 1.00; 0.95; 0.95; 0.95], ...
    [0.30; 0.30; 0.30; 0.30; 0.30; 0.30; 0.30; 0.30; 0.30; 0.24; 0.24; 0.24; 0.30; 0.36; 0.42], ...
    [0.020; 0.020; 0.020; 0.020; 0.020; 0.020; 0.020; 0.020; 0.020; 0.018; 0.018; 0.018; 0.020; 0.024; 0.026], ...
    'VariableNames', { ...
        'initial_airspeed_mps', 'initial_flight_path_deg', ...
        'control_altitude_bias_m', 'inner_kp', 'inner_kd'});

candidates = base;
if ~isempty(opts.Candidates)
    candidates = opts.Candidates;
end
end

function overrides = local_overrides(rowCfg)
overrides = struct();
overrides.inner_pd = struct();
overrides.inner_pd.kp = rowCfg.inner_kp;
overrides.inner_pd.kd = rowCfg.inner_kd;
end

function metrics = local_score(S, opts)
allRows = local_segment(S, "all");
afterDrop = local_segment_or(S, "after_drop", allRows);
last5 = local_segment_or(S, "last_5s", allRows);

metrics = local_empty_metrics();
metrics.after_drop_h_err_rms_m = afterDrop.h_err_rms_m;
metrics.after_drop_v_err_rms_mps = afterDrop.v_err_rms_mps;
metrics.after_drop_pitch_err_rms_deg = afterDrop.pitch_err_rms_deg;
metrics.after_drop_h_min_m = afterDrop.h_min_m;
metrics.after_drop_h_max_m = afterDrop.h_max_m;
metrics.last_5s_h_err_rms_m = last5.h_err_rms_m;
metrics.last_5s_v_err_rms_mps = last5.v_err_rms_mps;
metrics.last_5s_pitch_err_rms_deg = last5.pitch_err_rms_deg;
metrics.all_h_min_m = allRows.h_min_m;
metrics.all_h_max_m = allRows.h_max_m;

lowPenalty = max(0.0, opts.MinSafeAltitudeM - afterDrop.h_min_m);
highPenalty = max(0.0, afterDrop.h_max_m - opts.MaxUsefulAltitudeM);
metrics.score = ...
    3.00 * afterDrop.h_err_rms_m + ...
    0.45 * afterDrop.v_err_rms_mps + ...
    0.80 * afterDrop.pitch_err_rms_deg + ...
    2.00 * last5.h_err_rms_m + ...
    0.25 * last5.v_err_rms_mps + ...
    0.60 * last5.pitch_err_rms_deg + ...
    8.00 * lowPenalty.^2 + ...
    2.00 * highPenalty.^2;
end

function row = local_record_row(iteration, status, improved, rowCfg, metrics, runRoot, message)
row = table( ...
    iteration, string(status), improved, metrics.score, ...
    rowCfg.initial_airspeed_mps, rowCfg.initial_flight_path_deg, ...
    rowCfg.control_altitude_bias_m, rowCfg.inner_kp, rowCfg.inner_kd, ...
    metrics.after_drop_h_err_rms_m, metrics.after_drop_v_err_rms_mps, ...
    metrics.after_drop_pitch_err_rms_deg, metrics.after_drop_h_min_m, ...
    metrics.last_5s_h_err_rms_m, string(runRoot), string(message), ...
    'VariableNames', { ...
        'iteration', 'status', 'positive_optimization', 'score', ...
        'initial_airspeed_mps', 'initial_flight_path_deg', ...
        'control_altitude_bias_m', 'inner_kp', 'inner_kd', ...
        'after_drop_h_err_rms_m', 'after_drop_v_err_rms_mps', ...
        'after_drop_pitch_err_rms_deg', 'after_drop_h_min_m', ...
        'last_5s_h_err_rms_m', 'output_root', 'message'});
end

function metrics = local_empty_metrics()
metrics = struct();
metrics.score = Inf;
metrics.after_drop_h_err_rms_m = NaN;
metrics.after_drop_v_err_rms_mps = NaN;
metrics.after_drop_pitch_err_rms_deg = NaN;
metrics.after_drop_h_min_m = NaN;
metrics.after_drop_h_max_m = NaN;
metrics.last_5s_h_err_rms_m = NaN;
metrics.last_5s_v_err_rms_mps = NaN;
metrics.last_5s_pitch_err_rms_deg = NaN;
metrics.all_h_min_m = NaN;
metrics.all_h_max_m = NaN;
end

function row = local_segment(S, name)
idx = find(string(S.segment) == string(name), 1);
if isempty(idx)
    error("Missing segment in summary: %s", name);
end
row = S(idx, :);
end

function row = local_segment_or(S, name, fallback)
idx = find(string(S.segment) == string(name), 1);
if isempty(idx)
    row = fallback;
else
    row = S(idx, :);
end
end

function local_write_json(path, value)
try
    text = jsonencode(value, "PrettyPrint", true);
catch
    text = jsonencode(value);
end
fid = fopen(path, "w");
if fid < 0
    error("Could not write %s", path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "%s\n", text);
end

function opts = local_options(varargin)
opts.OutputRoot = "";
opts.StopTimeS = 22.0;
opts.MaxIterations = 15;
opts.StopOnPositive = true;
opts.MinImprovement = 0.0;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.BaselineInitialAirspeedMps = 55.5;
opts.BaselineInitialFlightPathDeg = 2.4;
opts.BaselineControlAltitudeBiasM = 0.95;
opts.InitialElevatorDelta = 0.0;
opts.InitialThrottleCmd = 0.80;
opts.MinSafeAltitudeM = 16.0;
opts.MaxUsefulAltitudeM = 24.0;
opts.Candidates = [];

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
