function fig = airdropx_plot(logs, mode, varargin)
%AIRDROPX_PLOT Plot AirdropX Simulink logs.
%
% Usage:
%   fig = airdropx_plot(out.logsout, "dashboard")
%   fig = airdropx_plot(out.logsout, "carp")
%   fig = airdropx_plot(out.logsout, "carp", "MonteCarlo", mc)

if nargin < 2 || strlength(string(mode)) == 0
    mode = "dashboard";
end

switch lower(string(mode))
    case {"dashboard", "dash"}
        fig = local_dashboard(logs);
    case {"carp", "cep", "circle"}
        fig = local_carp_circle(logs, varargin{:});
    otherwise
        error("Unknown plot mode: %s", mode);
end
end

function fig = local_dashboard(logs)
cfg = airdropx_sim_params();
fig = figure('Name', 'AirdropX Simulink Dashboard', 'Color', 'w');
tl = tiledlayout(fig, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'AirdropX Simulink Dashboard');

nexttile;
local_plot_signal(logs, "altitude_m", "Altitude", "m", true);
hold on;
yline(cfg.control.target_altitude_m, '--', sprintf('%.1f m', cfg.control.target_altitude_m));

nexttile;
[~, hVz] = local_plot_signal(logs, "vz_up_mps", "Vertical Speed Up", "m/s", true);
hold on;
[~, hAirspeed] = local_plot_signal(logs, "airspeed_mps", "Airspeed", "m/s", false);
local_apply_legend([hVz; hAirspeed], ["vz up"; "airspeed"]);

nexttile;
[~, hMass] = local_plot_signal(logs, "mass_kg", "Mass", "kg", true);
hold on;
yyaxis right;
[~, hCg] = local_plot_signal(logs, "cg_x_m", "CG X", "m", false);
ylabel("cg_x_m (m)");
local_apply_legend([hMass; hCg], ["mass"; "cg x"]);

nexttile;
local_plot_signal(logs, "drop_count", "Drop Count", "count", true);
ylim padded;

nexttile;
[~, hElevDelta] = local_plot_signal(logs, "elevator_delta", "Elevator Delta", "norm", true);
hold on;
[~, hElevCmd] = local_plot_signal(logs, "elevator_cmd_norm", "Elevator Command", "norm", false);
local_apply_legend([hElevDelta; hElevCmd], ["delta"; "cmd"]);

nexttile;
[~, hThrottle] = local_plot_signal(logs, "throttle_norm", "Throttle", "norm", true);
hold on;
[~, hThrottleCmd] = local_plot_signal(logs, "throttle_cmd", "Throttle Cmd", "norm", false);
local_apply_legend([hThrottle; hThrottleCmd], ["actual"; "cmd"]);

nexttile;
[~, hErr] = local_plot_signal(logs, "h_err", "PD Diagnostics", "norm / m", true);
hold on;
[~, hUPd] = local_plot_signal(logs, "u_pd", "u pd", "", false);
[~, hDropTrim] = local_plot_signal(logs, "drop_trim_bias", "drop trim bias", "", false);
[~, hUOut] = local_plot_signal(logs, "u_out", "u out", "", false);
local_apply_legend([hErr; hUPd; hDropTrim; hUOut], ["h err"; "u pd"; "drop trim"; "u out"]);

nexttile;
[hasCarp, hTGo] = local_plot_signal(logs, "t_to_release_s", "CARP/CEP", "s / m", true);
hold on;
[~, hWindow] = local_plot_signal(logs, "in_window", "in window", "", false);
[~, hMiss] = local_plot_signal(logs, "miss_distance_m", "miss distance", "m", false);
if ~hasCarp
    text(0.5, 0.5, 'No CARP/CEP signals logged', 'Units', 'normalized', ...
        'HorizontalAlignment', 'center');
end
local_apply_legend([hTGo; hWindow; hMiss], ["t to release"; "in window"; "miss"]);
end

function fig = local_carp_circle(logs, varargin)
cfg = airdropx_sim_params();
p = inputParser;
addParameter(p, "TargetN", cfg.carp.target_n_m);
addParameter(p, "TargetE", cfg.carp.target_e_m);
addParameter(p, "MonteCarlo", []);
addParameter(p, "AcceptanceRadiusM", 20.0);
parse(p, varargin{:});

targetN = double(p.Results.TargetN);
targetE = double(p.Results.TargetE);
mc = p.Results.MonteCarlo;
acceptanceRadius = double(p.Results.AcceptanceRadiusM);

[~, releaseH] = local_signal(logs, "actual_release_alt_m");
[~, predictedN] = local_signal(logs, "predicted_impact_n_m");
[~, predictedE] = local_signal(logs, "predicted_impact_e_m");
[~, miss] = local_signal(logs, "miss_distance_m");

rh = local_last_nonzero(releaseH);
pn = local_last_finite(predictedN);
pe = local_last_finite(predictedE);
r = local_last_valid_miss(miss);

fig = figure('Name', 'AirdropX CARP/CEP Scatter', 'Color', [0.02 0.02 0.02]);
set(fig, 'InvertHardcopy', 'off');
ax = axes(fig);
hold(ax, 'on');
axis(ax, 'equal');
set(ax, 'Color', [0.035 0.035 0.035], ...
    'XColor', [0.0 0.85 1.0], ...
    'YColor', [0.0 0.85 1.0], ...
    'GridColor', [0.35 0.35 0.35], ...
    'MinorGridColor', [0.18 0.18 0.18], ...
    'GridAlpha', 0.45, ...
    'MinorGridAlpha', 0.35);
grid(ax, 'on');
ax.XMinorGrid = 'on';
ax.YMinorGrid = 'on';
xline(ax, 0, '-', 'Color', [0.80 0.80 0.80], 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(ax, 0, '-', 'Color', [0.80 0.80 0.80], 'LineWidth', 1.0, 'HandleVisibility', 'off');
xlabel(ax, 'E (m)', 'Color', [0.0 0.85 1.0], 'FontWeight', 'bold');
ylabel(ax, 'N (m)', 'Color', [0.0 0.85 1.0], 'FontWeight', 'bold');
title(ax, '蒙特卡洛落点散布', 'Color', [0.0 0.85 1.0], ...
    'FontWeight', 'bold', 'Interpreter', 'none');

plot(ax, 0, 0, '+', 'Color', [1.0 0.10 0.10], 'MarkerSize', 16, 'LineWidth', 2.2);

if isfinite(acceptanceRadius) && acceptanceRadius > 0
    th = linspace(0, 2*pi, 361);
    plot(ax, acceptanceRadius*cos(th), acceptanceRadius*sin(th), '-', ...
        'Color', [0.0 0.85 1.0], 'LineWidth', 2.0);
    text(ax, acceptanceRadius * 0.70, acceptanceRadius * 0.70, ...
        sprintf('%.0fm limit', acceptanceRadius), ...
        'Color', [0.0 0.85 1.0], 'FontWeight', 'bold');
end

if ~isempty(mc) && isfield(mc, "cep50_m")
    r = double(mc.cep50_m);
end

if isfinite(r)
    th = linspace(0, 2*pi, 361);
    plot(ax, r*cos(th), r*sin(th), '--', 'Color', [1.0 0.20 0.20], 'LineWidth', 1.8);
end

if isfinite(pn) && isfinite(pe)
    plot(ax, pe - targetE, pn - targetN, 'o', ...
        'Color', [0.0 0.75 1.0], 'MarkerFaceColor', [0.0 0.75 1.0], 'MarkerSize', 6);
end

if ~isempty(mc) && isfield(mc, "offset_e_m") && isfield(mc, "offset_n_m")
    local_plot_mc(ax, mc);
end

if ~isempty(mc) && isfield(mc, "batches") && ~isempty(mc.batches)
    subText = sprintf('4-drop MC  CEP50_to_target = %.2f m   CEP95 = %.2f m   samples = %d', ...
        mc.cep50_m, mc.cep95_m, mc.samples);
elseif ~isempty(mc) && isfield(mc, "cep50_m")
    subText = sprintf('MC  CEP50_to_target = %.2f m   CEP95 = %.2f m   samples = %d', ...
        mc.cep50_m, mc.cep95_m, mc.samples);
elseif isfinite(r)
    subText = sprintf('miss/CEP proxy = %.2f m   release H = %.2f m', r, rh);
else
    subText = 'No CARP/CEP impact data logged yet';
    text(ax, 0.5, 0.5, 'No impact scatter data', ...
        'Units', 'normalized', 'Color', [0.0 0.85 1.0], ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end
subtitle(ax, subText, 'Color', [0.0 0.85 1.0], 'Interpreter', 'none');

span = local_scatter_span(mc, pe - targetE, pn - targetN, r, acceptanceRadius);
xlim(ax, [-span, span]);
ylim(ax, [-span, span]);
end

function local_plot_mc(ax, mc)
if isfield(mc, "batches") && ~isempty(mc.batches)
    colors = [
        0.00 0.95 1.00
        0.95 0.55 0.10
        0.75 0.30 1.00
        0.10 1.00 0.35
        1.00 0.20 0.65
        0.80 0.90 0.10
        ];
    th = linspace(0, 2*pi, 361);
    for k = 1:numel(mc.batches)
        c = colors(1 + mod(k - 1, size(colors, 1)), :);
        scatter(ax, mc.batches(k).offset_e_m, mc.batches(k).offset_n_m, ...
            14, c, 'filled', 'MarkerFaceAlpha', 0.45, ...
            'MarkerEdgeAlpha', 0.15);
        cx = mc.batches(k).mean_e_m;
        cy = mc.batches(k).mean_n_m;
        cr = mc.batches(k).cep50_m;
        if isfinite(cx) && isfinite(cy) && isfinite(cr)
            plot(ax, cx + cr*cos(th), cy + cr*sin(th), '-', ...
                'Color', c, 'LineWidth', 1.4);
        end
    end
else
    scatter(ax, mc.offset_e_m, mc.offset_n_m, 14, [0.0 0.85 1.0], 'filled', ...
        'MarkerFaceAlpha', 0.45, 'MarkerEdgeAlpha', 0.15);
end
end

function span = local_scatter_span(mc, predE, predN, cepR, acceptanceRadius)
vals = [predE; predN; cepR; acceptanceRadius];
if ~isempty(mc) && isfield(mc, "offset_e_m") && isfield(mc, "offset_n_m")
    vals = [vals; mc.offset_e_m(:); mc.offset_n_m(:)];
elseif ~isempty(mc) && isfield(mc, "batches") && ~isempty(mc.batches)
    for k = 1:numel(mc.batches)
        vals = [vals; mc.batches(k).offset_e_m(:); mc.batches(k).offset_n_m(:)]; %#ok<AGROW>
    end
end
vals = vals(isfinite(vals));
if isempty(vals)
    span = 25;
else
    span = max(5, ceil(max(abs(vals)) * 1.25));
end
if isfinite(acceptanceRadius) && acceptanceRadius > 0
    span = max(span, ceil(acceptanceRadius * 1.20));
end
end

function [ok, lineHandle] = local_plot_signal(logs, name, plotTitle, yLabel, clearAxes)
ok = false;
lineHandle = gobjects(0, 1);
[t, y] = local_signal(logs, name);
if isempty(y)
    if clearAxes
        title(plotTitle);
        xlabel('time (s)');
        ylabel(yLabel);
        grid on;
    end
    return;
end
lineHandle = plot(t, y, 'LineWidth', 1.2);
lineHandle = lineHandle(:);
ok = true;
if clearAxes
    title(plotTitle);
    xlabel('time (s)');
    ylabel(yLabel);
    grid on;
end
end

function local_apply_legend(handles, labels)
valid = isgraphics(handles);
if any(valid)
    legend(handles(valid), labels(valid), 'Location', 'best');
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
idx = find(abs(x(:)) > 0, 1, 'last');
if isempty(idx), v = NaN; else, v = x(idx); end
end

function v = local_last_finite(x)
if isempty(x), v = NaN; return; end
x = x(:);
idx = find(isfinite(x), 1, 'last');
if isempty(idx), v = NaN; else, v = x(idx); end
end

function v = local_last_valid_miss(x)
if isempty(x), v = NaN; return; end
x = x(:);
idx = find(isfinite(x) & x < 9000, 1, 'last');
if isempty(idx), v = NaN; else, v = x(idx); end
end
