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
local_plot_signal(logs, "vz_up_mps", "Vertical Speed Up", "m/s", true);
hold on;
local_plot_signal(logs, "airspeed_mps", "Airspeed", "m/s", false);
legend('vz up', 'airspeed', 'Location', 'best');

nexttile;
local_plot_signal(logs, "mass_kg", "Mass", "kg", true);
hold on;
yyaxis right;
local_plot_signal(logs, "cg_x_m", "CG X", "m", false);
ylabel("cg_x_m (m)");
legend('mass', 'cg x', 'Location', 'best');

nexttile;
local_plot_signal(logs, "drop_count", "Drop Count", "count", true);
ylim padded;

nexttile;
local_plot_signal(logs, "elevator_delta", "Elevator Delta", "norm", true);
hold on;
local_plot_signal(logs, "elevator_cmd_norm", "Elevator Command", "norm", false);
legend('delta', 'cmd', 'Location', 'best');

nexttile;
local_plot_signal(logs, "throttle_norm", "Throttle", "norm", true);
hold on;
local_plot_signal(logs, "throttle_cmd", "Throttle Cmd", "norm", false);
legend('actual', 'cmd', 'Location', 'best');

nexttile;
local_plot_signal(logs, "h_err", "PD Diagnostics", "norm / m", true);
hold on;
local_plot_signal(logs, "u_pd", "u pd", "", false);
local_plot_signal(logs, "drop_trim_bias", "drop trim bias", "", false);
local_plot_signal(logs, "u_out", "u out", "", false);
legend('h err', 'u pd', 'drop trim', 'u out', 'Location', 'best');

nexttile;
hasCarp = local_plot_signal(logs, "t_to_release_s", "CARP/CEP", "s / m", true);
hold on;
local_plot_signal(logs, "in_window", "in window", "", false);
local_plot_signal(logs, "miss_distance_m", "miss distance", "m", false);
if ~hasCarp
    text(0.5, 0.5, 'No CARP/CEP signals logged', 'Units', 'normalized', ...
        'HorizontalAlignment', 'center');
end
legend('t to release', 'in window', 'miss', 'Location', 'best');
end

function fig = local_carp_circle(logs, varargin)
cfg = airdropx_sim_params();
p = inputParser;
addParameter(p, "TargetN", cfg.carp.target_n_m);
addParameter(p, "TargetE", cfg.carp.target_e_m);
addParameter(p, "MonteCarlo", []);
parse(p, varargin{:});

targetN = double(p.Results.TargetN);
targetE = double(p.Results.TargetE);
mc = p.Results.MonteCarlo;

[~, releaseN] = local_signal(logs, "actual_release_n_m");
[~, releaseE] = local_signal(logs, "actual_release_e_m");
[~, releaseH] = local_signal(logs, "actual_release_alt_m");
[~, predictedN] = local_signal(logs, "predicted_impact_n_m");
[~, predictedE] = local_signal(logs, "predicted_impact_e_m");
[~, miss] = local_signal(logs, "miss_distance_m");

rn = local_last_nonzero(releaseN);
re = local_last_nonzero(releaseE);
rh = local_last_nonzero(releaseH);
pn = local_last_finite(predictedN);
pe = local_last_finite(predictedE);
r = local_last_valid_miss(miss);

fig = figure('Name', 'AirdropX CARP/CEP Circle', 'Color', 'w');
ax = axes(fig);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'East offset from target (m)');
ylabel(ax, 'North offset from target (m)');
title(ax, 'CARP/CEP Target Circle');

plot(ax, 0, 0, 'r+', 'MarkerSize', 14, 'LineWidth', 2);
text(ax, 0, 0, '  Target', 'Color', 'r', 'FontWeight', 'bold');

if ~isempty(mc) && isfield(mc, "cep50_m")
    r = double(mc.cep50_m);
end

if isfinite(r)
    th = linspace(0, 2*pi, 361);
    plot(ax, r*cos(th), r*sin(th), 'r--', 'LineWidth', 1.2);
end

if isfinite(pn) && isfinite(pe)
    plot(ax, pe - targetE, pn - targetN, 'bo', 'MarkerFaceColor', 'b');
    text(ax, pe - targetE, pn - targetN, '  Predicted impact', 'Color', 'b');
end

if ~isempty(mc) && isfield(mc, "offset_e_m") && isfield(mc, "offset_n_m")
    local_plot_mc(ax, mc);
end

if isfinite(rn) && isfinite(re)
    plot(ax, re - targetE, rn - targetN, 'ks', 'MarkerFaceColor', [0.2 0.2 0.2]);
    text(ax, re - targetE, rn - targetN, '  Release point', 'Color', [0.1 0.1 0.1]);
end

if ~isempty(mc) && isfield(mc, "batches") && ~isempty(mc.batches)
    subtitle(ax, sprintf('4-drop MC: combined CEP50 = %.2f m, CEP95 = %.2f m, samples = %d', ...
        mc.cep50_m, mc.cep95_m, mc.samples));
elseif ~isempty(mc) && isfield(mc, "cep50_m")
    subtitle(ax, sprintf('MC CEP50 = %.2f m, CEP95 = %.2f m, samples = %d', ...
        mc.cep50_m, mc.cep95_m, mc.samples));
elseif isfinite(r)
    subtitle(ax, sprintf('miss/CEP proxy = %.2f m, release H = %.2f m', r, rh));
else
    subtitle(ax, 'No release/miss signal logged yet');
end

span = max([25, abs(pe - targetE), abs(pn - targetN), abs(re - targetE), abs(rn - targetN), r], [], 'omitnan');
xlim(ax, [-span, span]);
ylim(ax, [-span, span]);
legend(ax, {'Target', 'CEP/miss circle', 'Predicted impact', 'Release point'}, 'Location', 'bestoutside');
end

function local_plot_mc(ax, mc)
if isfield(mc, "batches") && ~isempty(mc.batches)
    colors = lines(numel(mc.batches));
    th = linspace(0, 2*pi, 361);
    for k = 1:numel(mc.batches)
        scatter(ax, mc.batches(k).offset_e_m, mc.batches(k).offset_n_m, ...
            14, colors(k, :), 'filled', 'MarkerFaceAlpha', 0.35, ...
            'MarkerEdgeAlpha', 0.2);
        cx = mc.batches(k).mean_e_m;
        cy = mc.batches(k).mean_n_m;
        cr = mc.batches(k).cep50_m;
        if isfinite(cx) && isfinite(cy) && isfinite(cr)
            plot(ax, cx + cr*cos(th), cy + cr*sin(th), '-', ...
                'Color', colors(k, :), 'LineWidth', 1.4);
            text(ax, cx, cy, sprintf('  #%d CEP50 %.1fm', k, cr), ...
                'Color', colors(k, :), 'FontWeight', 'bold');
        end
    end
else
    scatter(ax, mc.offset_e_m, mc.offset_n_m, 14, [0.1 0.45 0.9], 'filled', ...
        'MarkerFaceAlpha', 0.35, 'MarkerEdgeAlpha', 0.2);
end
end

function ok = local_plot_signal(logs, name, plotTitle, yLabel, clearAxes)
ok = false;
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
plot(t, y, 'LineWidth', 1.2);
ok = true;
if clearAxes
    title(plotTitle);
    xlabel('time (s)');
    ylabel(yLabel);
    grid on;
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
