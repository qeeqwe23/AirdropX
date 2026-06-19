function report = airdropx_report(logs, mode, varargin)
%AIRDROPX_REPORT Offline reports for AirdropX Simulink logs.
%
% Usage:
%   report = airdropx_report(out.logsout, "nw20")
%   report = airdropx_report(out.logsout, "carp")

if nargin < 2 || strlength(string(mode)) == 0
    mode = "nw20";
end

switch lower(string(mode))
    case "nw20"
        report = local_nw20_report(logs, varargin{:});
    case {"carp", "cep", "carp_cep"}
        report = local_carp_report(logs);
    otherwise
        error("Unknown report mode: %s", mode);
end
end

function report = local_nw20_report(logs, varargin)
cfg = airdropx_sim_params();
p = inputParser;
addParameter(p, "HRef", cfg.control.target_altitude_m);
parse(p, varargin{:});
hRef = double(p.Results.HRef);

[t, h] = local_signal(logs, "altitude_m");
if isempty(t)
    [t, h] = local_signal(logs, "altitude");
end
[~, dropCount] = local_signal(logs, "drop_count");
[~, elevatorDelta] = local_signal(logs, "elevator_delta");

if isempty(h)
    error("Could not find altitude_m/altitude in logs.");
end
if isempty(dropCount), dropCount = zeros(size(h)); end
if isempty(elevatorDelta), elevatorDelta = zeros(size(h)); end

e = h(:) - hRef;
absE = abs(e);

dropEdges = find([0; diff(dropCount(:))] > 0.5);
dropTimes = t(dropEdges);

report = struct();
report.h_err_mean = mean(e);
report.h_err_rms = sqrt(mean(e.^2));
report.h_err_max = max(absE);
report.h_err_p95 = prctile(absE, 95);
report.min_altitude = min(h);
report.max_altitude = max(h);
report.final_altitude = h(end);
report.drop_count_final = dropCount(end);
report.drop_times = dropTimes(:).';
report.elevator_sat_rate = mean(abs(elevatorDelta(:)) >= cfg.metrics.elevator_saturation_abs * 0.99);
report.drop_table = airdropx_drop_table(logs);

fprintf("=== AirdropX NW20 4-drop report ===\n");
fprintf("drop_count_final : %.0f\n", report.drop_count_final);
fprintf("drop_times_s     : %s\n", mat2str(report.drop_times, 3));
fprintf("h_err_mean_m     : %.4f\n", report.h_err_mean);
fprintf("h_err_rms_m      : %.4f\n", report.h_err_rms);
fprintf("h_err_max_m      : %.4f\n", report.h_err_max);
fprintf("h_err_p95_m      : %.4f\n", report.h_err_p95);
fprintf("min_altitude_m   : %.4f\n", report.min_altitude);
fprintf("max_altitude_m   : %.4f\n", report.max_altitude);
fprintf("sat_rate         : %.2f %%\n", 100.0 * report.elevator_sat_rate);
end

function report = local_carp_report(logs)
[~, dropCount] = local_signal(logs, "drop_count");
[t, miss] = local_signal(logs, "miss_distance_m");
[~, actualN] = local_signal(logs, "actual_release_n_m");
[~, actualE] = local_signal(logs, "actual_release_e_m");
[~, actualH] = local_signal(logs, "actual_release_alt_m");
[~, inWindow] = local_signal(logs, "in_window");

if isempty(dropCount)
    error("Could not find drop_count in logs.");
end
if isempty(miss), miss = []; end

dropEdges = find([0; diff(dropCount(:))] > 0.5);
dropTimes = [];
if ~isempty(t) && ~isempty(dropEdges)
    dropTimes = t(dropEdges);
end

validMiss = miss(isfinite(miss) & miss < 9000);

report = struct();
report.drop_count_final = dropCount(end);
report.drop_times = dropTimes(:).';
report.release_n_m = local_last_nonzero(actualN);
report.release_e_m = local_last_nonzero(actualE);
report.release_alt_m = local_last_nonzero(actualH);
report.last_miss_distance_m = local_last_or_nan(validMiss);
report.cep50_to_target_m = local_median_or_nan(validMiss);
report.in_window_rate = local_mean_or_nan(inWindow > 0.5);

fprintf("=== AirdropX CARP/CEP report ===\n");
fprintf("drop_count_final    : %.0f\n", report.drop_count_final);
fprintf("drop_times_s        : %s\n", mat2str(report.drop_times, 3));
fprintf("last_release_N/E/H  : %.2f / %.2f / %.2f\n", report.release_n_m, report.release_e_m, report.release_alt_m);
fprintf("last_miss_m         : %.3f\n", report.last_miss_distance_m);
fprintf("cep50_to_target_m   : %.3f\n", report.cep50_to_target_m);
fprintf("in_window_rate      : %.2f %%\n", 100.0 * report.in_window_rate);
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
    return;
end

if isstruct(logs)
    if isfield(logs, "time"), t = logs.time(:); end
    if isfield(logs, name)
        y = logs.(name)(:);
        if isempty(t), t = (0:numel(y)-1).'; end
    end
end
end

function v = local_last_nonzero(x)
if isempty(x), v = NaN; return; end
idx = find(abs(x(:)) > 0, 1, "last");
if isempty(idx), v = NaN; else, v = x(idx); end
end

function v = local_last_or_nan(x)
if isempty(x), v = NaN; else, v = x(end); end
end

function v = local_median_or_nan(x)
if isempty(x), v = NaN; else, v = median(x); end
end

function v = local_mean_or_nan(x)
if isempty(x), v = NaN; else, v = mean(double(x)); end
end
