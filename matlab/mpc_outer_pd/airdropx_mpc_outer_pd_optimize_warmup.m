function result = airdropx_mpc_outer_pd_optimize_warmup(varargin)
%AIRDROPX_MPC_OUTER_PD_OPTIMIZE_WARMUP Tune warm-up outer MPC + PD behavior.
%
% The score intentionally prioritizes altitude and pitch. Airspeed is improved
% only when it does not buy performance by diving or over-pitching.

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
    outputRoot = string(fullfile(matlabDir, "results", "mpc_outer_pd_warmup_opt_" + stamp));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

candidates = local_candidates(opts);
records = table();
best = [];
recreateModel = true;

for i = 1:min(opts.MaxIterations, height(candidates))
    rowCfg = candidates(i, :);
    runRoot = fullfile(outputRoot, sprintf("iter_%03d", i));
    overrides = local_overrides(rowCfg);
    fprintf("Warm-up tune %d/%d: bias=%.3f vGain=%.4f vQ=%.2f rThr=%.2f rdThr=%.2f thrMin=%.2f thrDu=%.3f\n", ...
        i, min(opts.MaxIterations, height(candidates)), ...
        rowCfg.control_altitude_bias_m, rowCfg.v_gain_throttle, ...
        rowCfg.v_q, rowCfg.r_throttle, rowCfg.rd_throttle, ...
        rowCfg.throttle_min, rowCfg.throttle_du);

    try
        simResult = airdropx_mpc_outer_pd_run_closed_loop( ...
            "OutputRoot", runRoot, ...
            "StopTimeS", opts.StopTimeS, ...
            "WarmupTimeS", opts.WarmupTimeS, ...
            "TargetAltitudeM", opts.TargetAltitudeM, ...
            "TargetAirspeedMps", opts.TargetAirspeedMps, ...
            "TargetPitchDeg", opts.TargetPitchDeg, ...
            "InitialAirspeedMps", opts.InitialAirspeedMps, ...
            "InitialAltitudeM", opts.InitialAltitudeM, ...
            "InitialPitchDeg", opts.InitialPitchDeg, ...
            "InitialFlightPathDeg", opts.InitialFlightPathDeg, ...
            "InitialElevatorDelta", opts.InitialElevatorDelta, ...
            "InitialThrottleCmd", opts.InitialThrottleCmd, ...
            "ControlAltitudeBiasM", rowCfg.control_altitude_bias_m, ...
            "ConfigOverrides", overrides, ...
            "RecreateModel", recreateModel, ...
            "DisableVRForBatch", true);
        recreateModel = false;

        metrics = local_score(simResult.summary.segments, simResult.timeseries_csv, opts);
        rec = local_record_row(i, "ok", rowCfg, metrics, string(runRoot), "");

        candidate = struct();
        candidate.score = metrics.score;
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
        rec = local_record_row(i, "failed", rowCfg, metrics, string(runRoot), ME.message);
    end

    records = [records; rec]; %#ok<AGROW>
    writetable(records, fullfile(outputRoot, "iteration_summary.csv"));
end

if isempty(best)
    error("Warm-up optimization produced no valid run. See %s", outputRoot);
end

result = struct();
result.output_root = outputRoot;
result.best = best;
result.iterations = records;

fprintf("Warm-up outer MPC + PD optimization complete:\n");
fprintf("  best score : %.6f\n", best.score);
fprintf("  best run   : %s\n", best.output_root);
fprintf("  iterations : %s\n", fullfile(outputRoot, "iteration_summary.csv"));
end

function candidates = local_candidates(opts)
bias = [1.05; 1.10; 1.15; 1.20; 1.25; 1.30];
vGain = [-0.012; -0.016; -0.020; -0.024; -0.028];
vQ = [4.0; 5.5; 7.0; 8.5];
rThr = [0.90; 0.75; 0.60];
rdThr = [7.0; 9.0; 11.0];
thrMin = [0.35; 0.40; 0.45];
thrDu = [0.025; 0.030; 0.035];

rows = [];
for i = 1:numel(bias)
    for j = 1:numel(vGain)
        rows = [rows; bias(i), vGain(j), vQ(min(j, numel(vQ))), ...
            rThr(1 + mod(i + j, numel(rThr))), ...
            rdThr(1 + mod(i + 2*j, numel(rdThr))), ...
            thrMin(1 + mod(i, numel(thrMin))), ...
            thrDu(1 + mod(j, numel(thrDu)))]; %#ok<AGROW>
    end
end

base = array2table(rows, 'VariableNames', { ...
    'control_altitude_bias_m', 'v_gain_throttle', 'v_q', ...
    'r_throttle', 'rd_throttle', 'throttle_min', 'throttle_du'});

seed = table( ...
    [1.20; 1.15; 1.10; 1.20; 1.25; 1.15], ...
    [-0.010; -0.018; -0.022; -0.026; -0.020; -0.028], ...
    [4.0; 6.5; 8.0; 9.0; 7.0; 8.5], ...
    [0.90; 0.75; 0.65; 0.60; 0.75; 0.60], ...
    [7.0; 9.0; 10.0; 11.0; 9.0; 11.0], ...
    [0.35; 0.40; 0.40; 0.45; 0.40; 0.45], ...
    [0.035; 0.030; 0.030; 0.025; 0.030; 0.025], ...
    'VariableNames', base.Properties.VariableNames);

candidates = [seed; base];
if ~isempty(opts.Candidates)
    candidates = opts.Candidates;
end
end

function overrides = local_overrides(rowCfg)
overrides = struct();
overrides.direct_mpc = struct();
overrides.direct_mpc.weights = struct();
overrides.direct_mpc.weights.Q = diag([16.0, 2.2, rowCfg.v_q, 0.28, 0.50, 0.0, 0.0]);
overrides.direct_mpc.weights.R = diag([0.48, rowCfg.r_throttle]);
overrides.direct_mpc.weights.Rd = diag([9.0, rowCfg.rd_throttle]);
overrides.direct_mpc.constraints = struct();
overrides.direct_mpc.constraints.u_min = [-0.75; rowCfg.throttle_min];
overrides.direct_mpc.constraints.u_max = [0.45; 0.85];
overrides.direct_mpc.constraints.du_min = [-0.045; -rowCfg.throttle_du];
overrides.direct_mpc.constraints.du_max = [0.045; rowCfg.throttle_du];
overrides.direct_mpc.integral_feedback = struct();
overrides.direct_mpc.integral_feedback.v_gain_throttle = rowCfg.v_gain_throttle;
overrides.direct_mpc.integral_feedback.h_gain_throttle = -0.010;
overrides.direct_mpc.integral_feedback.limit = [0.18; 0.18];
end

function metrics = local_score(S, timeseriesCsv, opts)
allRows = local_segment(S, "all");
afterDrop = local_segment_or(S, "after_drop", allRows);
last5 = local_segment_or(S, "last_5s", allRows);
T = readtable(timeseriesCsv);

metrics = local_empty_metrics();
metrics.after_drop_h_err_rms_m = afterDrop.h_err_rms_m;
metrics.after_drop_v_err_rms_mps = afterDrop.v_err_rms_mps;
metrics.after_drop_pitch_err_rms_deg = afterDrop.pitch_err_rms_deg;
metrics.after_drop_h_min_m = afterDrop.h_min_m;
metrics.after_drop_h_max_m = afterDrop.h_max_m;
metrics.last_5s_h_err_rms_m = last5.h_err_rms_m;
metrics.last_5s_v_err_rms_mps = last5.v_err_rms_mps;
metrics.last_5s_pitch_err_rms_deg = last5.pitch_err_rms_deg;
metrics.pitch_max_deg = allRows.pitch_max_deg;
metrics.pitch_min_deg = allRows.pitch_min_deg;
metrics.throttle_rate_rms = local_rate_rms(T.time_s, T.throttle_norm);
metrics.elevator_rate_rms = local_rate_rms(T.time_s, T.elevator_cmd_norm);

lowPenalty = max(0.0, opts.MinSafeAltitudeM - afterDrop.h_min_m);
highPenalty = max(0.0, afterDrop.h_max_m - opts.MaxUsefulAltitudeM);
pitchHighPenalty = max(0.0, max(abs(allRows.pitch_min_deg), abs(allRows.pitch_max_deg)) - opts.RealisticPitchAbsMaxDeg);

metrics.score = ...
    8.00 * afterDrop.h_err_rms_m + ...
    5.00 * last5.h_err_rms_m + ...
    4.00 * afterDrop.pitch_err_rms_deg + ...
    2.50 * last5.pitch_err_rms_deg + ...
    0.80 * afterDrop.v_err_rms_mps + ...
    0.40 * last5.v_err_rms_mps + ...
    25.0 * lowPenalty.^2 + ...
    8.0 * highPenalty.^2 + ...
    5.0 * pitchHighPenalty.^2;
end

function v = local_rate_rms(t, y)
t = double(t(:));
y = double(y(:));
dt = diff(t);
dy = diff(y);
valid = isfinite(dt) & dt > 0 & isfinite(dy);
if ~any(valid)
    v = NaN;
else
    v = rms(dy(valid) ./ dt(valid));
end
end

function row = local_record_row(iteration, status, rowCfg, metrics, runRoot, message)
row = table( ...
    iteration, string(status), metrics.score, ...
    rowCfg.control_altitude_bias_m, rowCfg.v_gain_throttle, rowCfg.v_q, ...
    rowCfg.r_throttle, rowCfg.rd_throttle, rowCfg.throttle_min, rowCfg.throttle_du, ...
    metrics.after_drop_h_err_rms_m, metrics.after_drop_v_err_rms_mps, ...
    metrics.after_drop_pitch_err_rms_deg, metrics.after_drop_h_min_m, ...
    metrics.last_5s_h_err_rms_m, metrics.last_5s_v_err_rms_mps, ...
    metrics.last_5s_pitch_err_rms_deg, metrics.pitch_min_deg, metrics.pitch_max_deg, ...
    metrics.throttle_rate_rms, metrics.elevator_rate_rms, string(runRoot), string(message), ...
    'VariableNames', { ...
        'iteration', 'status', 'score', ...
        'control_altitude_bias_m', 'v_gain_throttle', 'v_q', ...
        'r_throttle', 'rd_throttle', 'throttle_min', 'throttle_du', ...
        'after_drop_h_err_rms_m', 'after_drop_v_err_rms_mps', ...
        'after_drop_pitch_err_rms_deg', 'after_drop_h_min_m', ...
        'last_5s_h_err_rms_m', 'last_5s_v_err_rms_mps', ...
        'last_5s_pitch_err_rms_deg', 'pitch_min_deg', 'pitch_max_deg', ...
        'throttle_rate_rms', 'elevator_rate_rms', 'output_root', 'message'});
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
metrics.pitch_min_deg = NaN;
metrics.pitch_max_deg = NaN;
metrics.throttle_rate_rms = NaN;
metrics.elevator_rate_rms = NaN;
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
opts.StopTimeS = 30.0;
opts.WarmupTimeS = 20.0;
opts.MaxIterations = 18;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.InitialAirspeedMps = 45.0;
opts.InitialAltitudeM = 20.0;
opts.InitialPitchDeg = 4.0;
opts.InitialFlightPathDeg = 0.0;
opts.InitialElevatorDelta = -0.193;
opts.InitialThrottleCmd = 0.507;
opts.MinSafeAltitudeM = 18.8;
opts.MaxUsefulAltitudeM = 21.5;
opts.RealisticPitchAbsMaxDeg = 8.0;
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
