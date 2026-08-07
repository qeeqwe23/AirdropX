function result = airdropx_mpc_outer_pd_sweep_drag_targets(varargin)
%AIRDROPX_MPC_OUTER_PD_SWEEP_DRAG_TARGETS Test drag sensitivity with 4 targets.

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
    outputRoot = string(fullfile(matlabDir, "results", "mpc_outer_pd_drag_target_sweep_" + stamp));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

cases = local_cases(opts);
records = table();
best = [];
recreateModel = true;

for i = 1:min(double(opts.MaxCases), height(cases))
    c = cases(i, :);
    [offsetN, offsetE] = local_offsets(c.target_spread_m);
    runRoot = fullfile(outputRoot, sprintf("case_%03d_drag_%0.2f_spread_%0.1f", ...
        i, c.drag_scale, c.target_spread_m));

    fprintf("Drag/target sweep %d/%d: drag scale=%.3f target spread=%.2f m\n", ...
        i, min(double(opts.MaxCases), height(cases)), c.drag_scale, c.target_spread_m);

    try
        simResult = airdropx_mpc_outer_pd_run_optimized_warmup( ...
            "OutputRoot", runRoot, ...
            "CarpTargetOffsetNorthM", offsetN, ...
            "CarpTargetOffsetEastM", offsetE, ...
            "CarpReleaseDelayS", opts.CarpReleaseDelayS, ...
            "BallisticKDragScale", c.drag_scale, ...
            "RecreateModel", recreateModel, ...
            "DisableVRForBatch", true, ...
            "ShowPlots", opts.ShowPlots);
        recreateModel = false;

        metrics = local_case_metrics(simResult);
        rec = local_record(i, "ok", c.drag_scale, c.target_spread_m, metrics, string(runRoot), "");
        candidate = struct();
        candidate.score = metrics.score;
        candidate.drag_scale = c.drag_scale;
        candidate.target_spread_m = c.target_spread_m;
        candidate.output_root = string(runRoot);
        candidate.summary_csv = string(simResult.summary_csv);
        candidate.drop_details_csv = string(simResult.drop_details_csv);
        candidate.metrics = metrics;
        if isempty(best) || candidate.score < best.score
            best = candidate;
            local_write_json(fullfile(outputRoot, "best_drag_target_case.json"), best);
        end
    catch ME
        metrics = local_empty_metrics();
        rec = local_record(i, "failed", c.drag_scale, c.target_spread_m, metrics, string(runRoot), ME.message);
    end

    records = [records; rec]; %#ok<AGROW>
    writetable(records, fullfile(outputRoot, "sweep_summary.csv"));
end

result = struct();
result.output_root = outputRoot;
result.cases = records;
result.best = best;
result.summary_csv = string(fullfile(outputRoot, "sweep_summary.csv"));

fprintf("Drag/target sweep complete:\n");
fprintf("  %s\n", result.summary_csv);
if ~isempty(best)
    fprintf("  best score %.6f at drag %.3f spread %.2f m\n", ...
        best.score, best.drag_scale, best.target_spread_m);
end
end

function cases = local_cases(opts)
dragScales = double(opts.DragScales(:));
spreads = double(opts.TargetSpreadsM(:));
rows = [];
for i = 1:numel(dragScales)
    for j = 1:numel(spreads)
        rows = [rows; dragScales(i), spreads(j)]; %#ok<AGROW>
    end
end
cases = array2table(rows, 'VariableNames', {'drag_scale', 'target_spread_m'});
end

function [offsetN, offsetE] = local_offsets(spacingM)
spacingM = double(spacingM);
offsetN = spacingM * [1; 2; 3; 4.75];
offsetE = zeros(4, 1);
end

function metrics = local_case_metrics(simResult)
S = simResult.summary.segments;
allRows = local_segment(S, "all");
afterDrop = local_segment_or(S, "after_drop", allRows);
dropTable = simResult.summary.drop_table;

metrics = local_empty_metrics();
metrics.score = afterDrop.h_err_rms_m + 0.25 * afterDrop.pitch_err_rms_deg + ...
    0.10 * afterDrop.v_err_rms_mps + 0.35 * afterDrop.impact_miss_mean_m + ...
    0.20 * afterDrop.impact_miss_max_m;
metrics.drop_events = afterDrop.drop_events;
metrics.impact_miss_mean_m = afterDrop.impact_miss_mean_m;
metrics.impact_miss_max_m = afterDrop.impact_miss_max_m;
metrics.after_drop_h_err_rms_m = afterDrop.h_err_rms_m;
metrics.after_drop_v_err_rms_mps = afterDrop.v_err_rms_mps;
metrics.after_drop_pitch_err_rms_deg = afterDrop.pitch_err_rms_deg;
metrics.h_min_m = afterDrop.h_min_m;
metrics.per_drop_miss_m = NaN(1, 4);
if ~isempty(dropTable)
    n = min(4, height(dropTable));
    metrics.per_drop_miss_m(1:n) = double(dropTable.impact_radial_error_m(1:n)).';
end
end

function row = local_record(iteration, status, dragScale, targetSpreadM, metrics, runRoot, message)
row = table( ...
    iteration, string(status), double(dragScale), double(targetSpreadM), metrics.score, ...
    metrics.drop_events, metrics.impact_miss_mean_m, metrics.impact_miss_max_m, ...
    metrics.per_drop_miss_m(1), metrics.per_drop_miss_m(2), ...
    metrics.per_drop_miss_m(3), metrics.per_drop_miss_m(4), ...
    metrics.after_drop_h_err_rms_m, metrics.after_drop_v_err_rms_mps, ...
    metrics.after_drop_pitch_err_rms_deg, metrics.h_min_m, ...
    string(runRoot), string(message), ...
    'VariableNames', { ...
        'iteration', 'status', 'drag_scale', 'target_spread_m', 'score', ...
        'drop_events', 'impact_miss_mean_m', 'impact_miss_max_m', ...
        'drop1_miss_m', 'drop2_miss_m', 'drop3_miss_m', 'drop4_miss_m', ...
        'after_drop_h_err_rms_m', 'after_drop_v_err_rms_mps', ...
        'after_drop_pitch_err_rms_deg', 'h_min_m', 'output_root', 'message'});
end

function metrics = local_empty_metrics()
metrics = struct();
metrics.score = Inf;
metrics.drop_events = NaN;
metrics.impact_miss_mean_m = NaN;
metrics.impact_miss_max_m = NaN;
metrics.after_drop_h_err_rms_m = NaN;
metrics.after_drop_v_err_rms_mps = NaN;
metrics.after_drop_pitch_err_rms_deg = NaN;
metrics.h_min_m = NaN;
metrics.per_drop_miss_m = NaN(1, 4);
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
opts.DragScales = [0.85; 1.0; 1.15];
opts.TargetSpreadsM = [0.8];
opts.MaxCases = 3;
opts.CarpReleaseDelayS = 0.0;
opts.ShowPlots = false;

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
