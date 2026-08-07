function result = airdropx_mpc_evaluate_csv(csvPath, varargin)
%AIRDROPX_MPC_EVALUATE_CSV Compute tracking and four-drop impact metrics.

opts = local_options(varargin{:});
T = readtable(csvPath);

targetH = local_column(T, "target_altitude_m", opts.TargetAltitudeM);
targetV = local_column(T, "target_airspeed_mps", opts.TargetAirspeedMps);
targetPitch = local_column(T, "target_pitch_deg", opts.TargetPitchDeg);

segments = [
    local_segment(T, "all", true(height(T), 1), targetH, targetV, targetPitch, opts)
    local_segment(T, "after_drop", T.time_s >= double(opts.AfterDropTime), targetH, targetV, targetPitch, opts)
    local_segment(T, "last_10s", T.time_s >= max(T.time_s) - 10.0, targetH, targetV, targetPitch, opts)
    local_segment(T, "last_5s", T.time_s >= max(T.time_s) - 5.0, targetH, targetV, targetPitch, opts)
    ];

dropTable = local_drop_table(T, opts);
if ~isempty(dropTable)
    for i = 1:height(segments)
        segments.drop_events(i) = height(dropTable);
        segments.impact_miss_mean_m(i) = mean(dropTable.impact_radial_error_m, "omitnan");
        segments.impact_miss_max_m(i) = max(dropTable.impact_radial_error_m, [], "omitnan");
    end
end

result = struct();
result.csv_path = string(csvPath);
result.segments = segments;
result.drop_table = dropTable;

if strlength(string(opts.OutputFile)) > 0
    writetable(segments, opts.OutputFile);
end
if strlength(string(opts.DropDetailsFile)) > 0
    writetable(dropTable, opts.DropDetailsFile);
end
if strlength(string(opts.PlotFile)) > 0 && ~isempty(dropTable)
    local_plot_drops(dropTable, opts.PlotFile);
end
end

function row = local_segment(T, name, mask, targetH, targetV, targetPitch, opts)
mask = logical(mask(:));
if ~any(mask)
    mask = true(height(T), 1);
end
hErr = double(T.altitude_m(mask)) - double(targetH(mask));
vErr = double(T.airspeed_mps(mask)) - double(targetV(mask));
pitchErr = double(T.pitch_deg(mask)) - double(targetPitch(mask));
row = table( ...
    string(name), min(T.time_s(mask)), max(T.time_s(mask)), ...
    mean(T.altitude_m(mask), "omitnan"), local_rms(hErr), min(T.altitude_m(mask)), max(T.altitude_m(mask)), ...
    mean(T.airspeed_mps(mask), "omitnan"), local_rms(vErr), min(T.airspeed_mps(mask)), max(T.airspeed_mps(mask)), ...
    mean(T.pitch_deg(mask), "omitnan"), local_rms(pitchErr), min(T.pitch_deg(mask)), max(T.pitch_deg(mask)), ...
    mean(T.vz_up_mps(mask), "omitnan"), ...
    0, NaN, NaN, ...
    'VariableNames', { ...
    'segment', 'start_s', 'end_s', ...
    'h_mean_m', 'h_err_rms_m', 'h_min_m', 'h_max_m', ...
    'v_mean_mps', 'v_err_rms_mps', 'v_min_mps', 'v_max_mps', ...
    'pitch_mean_deg', 'pitch_err_rms_deg', 'pitch_min_deg', 'pitch_max_deg', ...
    'vz_mean_mps', 'drop_events', 'impact_miss_mean_m', 'impact_miss_max_m'});
end

function dropTable = local_drop_table(T, opts)
idx = local_drop_indices(T);
nDrops = numel(idx);
if nDrops == 0
    dropTable = table();
    return;
end
[targetN, targetE] = local_drop_targets(opts, nDrops);
rows = table();
for i = 1:nDrops
    k = idx(i);
    [impactN, impactE] = local_impact(T, k);
    errN = impactN - targetN(i);
    errE = impactE - targetE(i);
    rows = [rows; table( ...
        i, double(T.time_s(k)), targetN(i), targetE(i), impactN, impactE, errN, errE, hypot(errN, errE), ...
        'VariableNames', { ...
        'drop_index', 'time_s', 'target_n_m', 'target_e_m', ...
        'impact_n_m', 'impact_e_m', 'impact_error_n_m', 'impact_error_e_m', ...
        'impact_radial_error_m'})]; %#ok<AGROW>
end
dropTable = rows;
end

function idx = local_drop_indices(T)
vars = string(T.Properties.VariableNames);
idx = [];
if ismember("drop_count", vars)
    dc = double(T.drop_count(:));
    idx = find([false; diff(dc) > 0]);
end
if isempty(idx) && ismember("selected_drop_cmd", vars)
    cmd = double(T.selected_drop_cmd(:)) > 0.5;
    idx = find(cmd & [true; ~cmd(1:end-1)]);
elseif isempty(idx) && ismember("drop_cmd", vars)
    cmd = double(T.drop_cmd(:)) > 0.5;
    idx = find(cmd & [true; ~cmd(1:end-1)]);
end
idx = idx(:);
end

function [targetN, targetE] = local_drop_targets(opts, nDrops)
centerN = double(opts.TargetNorthM);
centerE = double(opts.TargetEastM);
if ~isempty(opts.DropTargetNorthM)
    northValues = double(opts.DropTargetNorthM(:));
    if max(abs(northValues - centerN), [], "omitnan") < max(25.0, 0.25 * max(abs(centerN), 1.0))
        offsetN = northValues - centerN;
    else
        offsetN = northValues;
    end
else
    offsetN = zeros(max(nDrops, 1), 1);
end
if ~isempty(opts.DropTargetEastM)
    eastValues = double(opts.DropTargetEastM(:));
    if max(abs(eastValues - centerE), [], "omitnan") < max(25.0, 0.25 * max(abs(centerE), 1.0))
        offsetE = eastValues - centerE;
    else
        offsetE = eastValues;
    end
else
    offsetE = zeros(max(nDrops, 1), 1);
end
[targetN, targetE] = airdropx_four_drop_targets(centerN, centerE, (1:max(nDrops, 1)).', offsetN, offsetE);
targetN = targetN(1:nDrops);
targetE = targetE(1:nDrops);
end
function [impactN, impactE] = local_impact(T, idx)
vars = string(T.Properties.VariableNames);
hasRelease = all(ismember([ ...
    "actual_release_n_m", "actual_release_e_m", "actual_release_alt_m", ...
    "release_airspeed_mps", "release_heading_deg"], vars));
if hasRelease
    releaseN = double(T.actual_release_n_m(idx));
    releaseE = double(T.actual_release_e_m(idx));
    releaseH = double(T.actual_release_alt_m(idx));
    airspeed = double(T.release_airspeed_mps(idx));
    heading = double(T.release_heading_deg(idx));
    releaseValid = all(isfinite([releaseN, releaseE, releaseH, airspeed, heading])) && ...
        releaseH > 0.0 && airspeed > 1.0;
    if releaseValid
        windN = 0.0;
        windE = 0.0;
        if ismember("release_wind_n_mps", vars)
            windN = double(T.release_wind_n_mps(idx));
        end
        if ismember("release_wind_e_mps", vars)
            windE = double(T.release_wind_e_mps(idx));
        end
        impact = airdropx_carp_release_point(0.0, 0.0, releaseH, airspeed, windE, windN, 0.0, heading);
        impactN = releaseN + impact.ballistic_n_m + impact.wind_drift_n_m;
        impactE = releaseE + impact.ballistic_e_m + impact.wind_drift_e_m;
        return;
    end
end
if ismember("predicted_impact_n_m", vars) && ismember("predicted_impact_e_m", vars)
    impactN = double(T.predicted_impact_n_m(idx));
    impactE = double(T.predicted_impact_e_m(idx));
elseif ismember("pos_n_m", vars) && ismember("pos_e_m", vars)
    impactN = double(T.pos_n_m(idx));
    impactE = double(T.pos_e_m(idx));
else
    impactN = NaN;
    impactE = NaN;
end
end

function x = local_column(T, name, fallback)
if ismember(name, string(T.Properties.VariableNames))
    x = double(T.(name)(:));
else
    x = double(fallback) * ones(height(T), 1);
end
end

function value = local_rms(x)
x = double(x(:));
x = x(isfinite(x));
if isempty(x)
    value = NaN;
else
    value = sqrt(mean(x .^ 2));
end
end

function local_plot_drops(dropTable, plotFile)
fig = figure("Visible", "off");
hold on;
grid on;
axis equal;
scatter(dropTable.target_e_m, dropTable.target_n_m, 45, "kx", "LineWidth", 1.5);
scatter(dropTable.impact_e_m, dropTable.impact_n_m, 45, "filled");
xlabel("east m");
ylabel("north m");
legend("target", "impact", "Location", "best");
title("AirdropX four-drop impact");
exportgraphics(fig, plotFile, "Resolution", 150);
close(fig);
end

function opts = local_options(varargin)
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.AfterDropTime = 10.6;
opts.TargetNorthM = 1000.0;
opts.TargetEastM = 0.0;
opts.DropTargetNorthM = [];
opts.DropTargetEastM = [];
opts.DropMode = 1.0;
opts.OutputFile = "";
opts.DropDetailsFile = "";
opts.PlotFile = "";
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
