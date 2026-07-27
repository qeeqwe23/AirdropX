function result = airdropx_mpc_optimize_closed_loop(varargin)
%AIRDROPX_MPC_OPTIMIZE_CLOSED_LOOP Iteratively tune closed-loop MPC entry setup.
%
% The optimizer runs the standalone MPC Simulink model repeatedly, evaluates
% each exported CSV, and writes a ranked iteration log. It does not modify
% matlab/untitled1.slx.

opts = local_options(varargin{:});

thisFile = mfilename("fullpath");
mpcDir = string(fileparts(thisFile));
matlabDir = string(fileparts(mpcDir));

addpath(char(matlabDir));
addpath(char(mpcDir));

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    outputRoot = string(fullfile(matlabDir, "results", "mpc_optimize_" + stamp));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

best = [];
visited = strings(0, 1);
records = table();
center = [
    double(opts.InitialAirspeedMps)
    double(opts.InitialFlightPathDeg)
    double(opts.ControlAltitudeBiasM)
    ];
step = [
    double(opts.InitialAirspeedStepMps)
    double(opts.FlightPathStepDeg)
    double(opts.AltitudeBiasStepM)
    ];
minStep = [
    double(opts.MinInitialAirspeedStepMps)
    double(opts.MinFlightPathStepDeg)
    double(opts.MinAltitudeBiasStepM)
    ];

iteration = 0;
roundNo = 0;
recreateModel = true;

while iteration < opts.MaxIterations && any(step >= minStep)
    roundNo = roundNo + 1;
    candidates = local_round_candidates(center, step, opts);
    improved = false;

    for i = 1:size(candidates, 1)
        if iteration >= opts.MaxIterations
            break;
        end

        params = candidates(i, :).';
        key = local_candidate_key(params);
        if any(visited == key)
            continue;
        end
        visited(end + 1, 1) = key; %#ok<AGROW>
        iteration = iteration + 1;

        runName = sprintf("iter_%03d_v%05.2f_g%04.2f_b%04.2f", ...
            iteration, params(1), params(2), params(3));
        runName = regexprep(runName, "[^A-Za-z0-9_]+", "p");
        runRoot = fullfile(outputRoot, runName);

        fprintf("AirdropX MPC optimize %d/%d: v0=%.3f gamma=%.3f bias=%.3f\n", ...
            iteration, opts.MaxIterations, params(1), params(2), params(3));

        [row, candidate] = local_run_candidate(opts, runRoot, params, ...
            iteration, roundNo, recreateModel);
        recreateModel = false;
        records = [records; row]; %#ok<AGROW>
        writetable(records, fullfile(outputRoot, "iteration_summary.csv"));

        if strcmp(row.status, "ok") && (isempty(best) || candidate.score < best.score)
            best = candidate;
            center = params;
            improved = true;
            local_write_best(outputRoot, best);
        end

        if opts.StopOnPass && strcmp(row.status, "ok") && row.pass
            break;
        end
    end

    if opts.StopOnPass && ~isempty(best) && best.pass
        break;
    end
    if ~improved
        step = step * double(opts.StepShrink);
    end
end

if isempty(best)
    error("MPC optimization finished without a successful closed-loop run. See %s", outputRoot);
end

result = struct();
result.output_root = outputRoot;
result.iterations = records;
result.best = best;
result.best_summary_csv = best.summary_csv;
result.best_timeseries_csv = best.timeseries_csv;
result.iteration_summary_csv = string(fullfile(outputRoot, "iteration_summary.csv"));

fprintf("AirdropX MPC optimization finished:\n");
fprintf("  best score: %.6f\n", best.score);
fprintf("  best run  : %s\n", best.output_root);
fprintf("  summary   : %s\n", best.summary_csv);
end

function [row, candidate] = local_run_candidate(opts, runRoot, params, iteration, roundNo, recreateModel)
candidate = [];
try
    simResult = airdropx_mpc_run_closed_loop( ...
        "Model", opts.Model, ...
        "OutputRoot", runRoot, ...
        "StopTimeS", opts.StopTimeS, ...
        "TargetAltitudeM", opts.TargetAltitudeM, ...
        "TargetAirspeedMps", opts.TargetAirspeedMps, ...
        "TargetPitchDeg", opts.TargetPitchDeg, ...
        "ControlAltitudeBiasM", params(3), ...
        "InitialAirspeedMps", params(1), ...
        "InitialAltitudeM", opts.InitialAltitudeM, ...
        "InitialPitchDeg", opts.InitialPitchDeg, ...
        "InitialFlightPathDeg", params(2), ...
        "InitialElevatorDelta", opts.InitialElevatorDelta, ...
        "InitialThrottleCmd", opts.InitialThrottleCmd, ...
        "RecreateModel", recreateModel, ...
        "ConfigOverrides", opts.ConfigOverrides);

    metrics = local_score(simResult.summary.segments, opts);
    candidate = struct();
    candidate.score = metrics.score;
    candidate.pass = metrics.pass;
    candidate.params = struct( ...
        "InitialAirspeedMps", params(1), ...
        "InitialFlightPathDeg", params(2), ...
        "ControlAltitudeBiasM", params(3));
    candidate.output_root = string(simResult.output_root);
    candidate.summary_csv = string(simResult.summary_csv);
    candidate.timeseries_csv = string(simResult.timeseries_csv);
    candidate.metrics = metrics;

    row = local_record_row(iteration, roundNo, "ok", params, runRoot, metrics, "");
catch ME
    metrics = local_empty_metrics();
    row = local_record_row(iteration, roundNo, "failed", params, runRoot, metrics, ME.message);
end
end

function candidates = local_round_candidates(center, step, opts)
candidates = center.';
for i = 1:numel(center)
    plus = center;
    plus(i) = plus(i) + step(i);
    minus = center;
    minus(i) = minus(i) - step(i);
    candidates = [candidates; plus.'; minus.']; %#ok<AGROW>
end

candidates(:, 1) = min(max(candidates(:, 1), opts.InitialAirspeedMinMps), opts.InitialAirspeedMaxMps);
candidates(:, 2) = min(max(candidates(:, 2), opts.FlightPathMinDeg), opts.FlightPathMaxDeg);
candidates(:, 3) = min(max(candidates(:, 3), opts.AltitudeBiasMinM), opts.AltitudeBiasMaxM);
candidates = unique(round(candidates, 6), "rows", "stable");
end

function key = local_candidate_key(params)
key = sprintf("%.6f|%.6f|%.6f", params(1), params(2), params(3));
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
score = ...
    3.00 * afterDrop.h_err_rms_m + ...
    0.45 * afterDrop.v_err_rms_mps + ...
    0.80 * afterDrop.pitch_err_rms_deg + ...
    2.00 * last5.h_err_rms_m + ...
    0.25 * last5.v_err_rms_mps + ...
    0.60 * last5.pitch_err_rms_deg + ...
    8.00 * lowPenalty.^2 + ...
    2.00 * highPenalty.^2;

metrics.score = score;
metrics.pass = afterDrop.h_err_rms_m <= opts.PassAfterDropHErrRmsM && ...
    last5.h_err_rms_m <= opts.PassLast5HErrRmsM && ...
    afterDrop.h_min_m >= opts.MinSafeAltitudeM && ...
    afterDrop.pitch_err_rms_deg <= opts.PassAfterDropPitchErrRmsDeg;
end

function row = local_record_row(iteration, roundNo, status, params, runRoot, metrics, message)
row = table( ...
    iteration, ...
    roundNo, ...
    string(status), ...
    metrics.pass, ...
    metrics.score, ...
    params(1), ...
    params(2), ...
    params(3), ...
    metrics.after_drop_h_err_rms_m, ...
    metrics.after_drop_v_err_rms_mps, ...
    metrics.after_drop_pitch_err_rms_deg, ...
    metrics.after_drop_h_min_m, ...
    metrics.last_5s_h_err_rms_m, ...
    string(runRoot), ...
    string(message), ...
    'VariableNames', { ...
        'iteration', 'round', 'status', 'pass', 'score', ...
        'initial_airspeed_mps', 'initial_flight_path_deg', ...
        'control_altitude_bias_m', ...
        'after_drop_h_err_rms_m', 'after_drop_v_err_rms_mps', ...
        'after_drop_pitch_err_rms_deg', 'after_drop_h_min_m', ...
        'last_5s_h_err_rms_m', 'output_root', 'message'});
end

function metrics = local_empty_metrics()
metrics = struct();
metrics.score = Inf;
metrics.pass = false;
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
    error("Missing segment in MPC summary: %s", name);
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

function local_write_best(outputRoot, best)
bestFile = fullfile(outputRoot, "best_run.json");
try
    text = jsonencode(best, "PrettyPrint", true);
catch
    text = jsonencode(best);
end
fid = fopen(bestFile, "w");
if fid < 0
    error("Could not write %s", bestFile);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "%s\n", text);
end

function opts = local_options(varargin)
opts.Model = "airdropx_mpc_closed_loop";
opts.OutputRoot = "";
opts.StopTimeS = 22.0;
opts.MaxIterations = 9;
opts.StopOnPass = false;
opts.StepShrink = 0.5;

opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.InitialAirspeedMps = 55.5;
opts.InitialAirspeedStepMps = 0.5;
opts.InitialAirspeedMinMps = 53.0;
opts.InitialAirspeedMaxMps = 60.0;
opts.MinInitialAirspeedStepMps = 0.10;
opts.InitialAltitudeM = NaN;
opts.InitialPitchDeg = NaN;
opts.InitialFlightPathDeg = 2.4;
opts.FlightPathStepDeg = 0.2;
opts.FlightPathMinDeg = 1.5;
opts.FlightPathMaxDeg = 3.2;
opts.MinFlightPathStepDeg = 0.05;
opts.ControlAltitudeBiasM = 0.95;
opts.AltitudeBiasStepM = 0.10;
opts.AltitudeBiasMinM = 0.50;
opts.AltitudeBiasMaxM = 1.20;
opts.MinAltitudeBiasStepM = 0.025;
opts.InitialElevatorDelta = 0.0;
opts.InitialThrottleCmd = 0.80;
opts.ConfigOverrides = [];

opts.MinSafeAltitudeM = 16.0;
opts.MaxUsefulAltitudeM = 24.0;
opts.PassAfterDropHErrRmsM = 4.0;
opts.PassLast5HErrRmsM = 2.0;
opts.PassAfterDropPitchErrRmsDeg = 2.0;

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
