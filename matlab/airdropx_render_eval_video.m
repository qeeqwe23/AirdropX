function outputFile = airdropx_render_eval_video(data, varargin)
%AIRDROPX_RENDER_EVAL_VIDEO Render an animated evaluation dashboard video.
%
% Usage:
%   airdropx_render_eval_video
%   airdropx_render_eval_video("matlab/results/release_smoke/timeseries.csv")
%   airdropx_render_eval_video(r.timeseries, "OutputFile", "eval.mp4")
%   airdropx_render_eval_video(..., "SecondsPerVideoSecond", 1.0)

if nargin < 1 || isempty(data)
    data = fullfile("matlab", "results", "release_smoke", "timeseries.csv");
end

opts = local_options(varargin{:});
T = local_table(data);

required = ["time_s", "altitude_m", "airspeed_mps", "roll_deg", "heading_deg", "elevator_cmd_norm", "u_out", "drop_count"];
for i = 1:numel(required)
    if ~ismember(required(i), string(T.Properties.VariableNames))
        error("AirdropX eval video missing column: %s", required(i));
    end
end

t = T.time_s(:);
alt = T.altitude_m(:);
airspeed = T.airspeed_mps(:);
hasGroundspeed = ismember("groundspeed_mps", string(T.Properties.VariableNames));
if hasGroundspeed
    groundspeed = T.groundspeed_mps(:);
else
    groundspeed = NaN(size(airspeed));
end
roll = T.roll_deg(:);
heading = T.heading_deg(:);
elev = T.elevator_cmd_norm(:);
uout = T.u_out(:);
dropCount = T.drop_count(:);

outputFile = string(opts.OutputFile);
if strlength(outputFile) == 0
    outputDir = fullfile("matlab", "results", "release_smoke");
    if ~isfolder(outputDir)
        mkdir(outputDir);
    end
    outputFile = string(fullfile(outputDir, "eval_process.mp4"));
end

durationVideoS = (t(end) - t(1)) / opts.SecondsPerVideoSecond;
frameCount = min(opts.MaxFrames, max(2, ceil(durationVideoS * opts.Fps)));
frameTimes = linspace(t(1), t(end), frameCount);

fig = figure("Name", "AirdropX Evaluation Process", ...
    "Color", "w", ...
    "Visible", opts.Visible, ...
    "Position", [80 80 opts.Width opts.Height]);

writer = VideoWriter(char(outputFile), "MPEG-4");
writer.FrameRate = opts.Fps;
writer.Quality = opts.Quality;
open(writer);
cleanup = onCleanup(@() local_cleanup(writer, fig));

for k = 1:numel(frameTimes)
    tk = frameTimes(k);
    idx = find(t <= tk, 1, "last");
    if isempty(idx), idx = 1; end

    clf(fig);
    tl = tiledlayout(fig, 4, 2, "TileSpacing", "compact", "Padding", "compact");
    title(tl, sprintf("AirdropX Evaluation Process  |  t = %.2f s", tk), ...
        "FontWeight", "bold");

    local_plot(ax_next(), t, alt, idx, "Altitude", "m", [16, 22], 20.0);
    local_plot_multi(ax_next(), t, {airspeed, groundspeed}, idx, ...
        ["airspeed", "groundspeed"], "Speed", "m/s", []);
    local_plot_multi(ax_next(), t, {roll, heading}, idx, ...
        ["roll", "heading"], "Attitude", "deg", []);
    local_plot(ax_next(), t, uout, idx, "Controller u out", "norm", [-0.45, 0.45], NaN);
    local_plot(ax_next(), t, elev, idx, "Elevator Command", "norm", [-0.75, 0.45], NaN);
    local_plot(ax_next(), t, dropCount, idx, "Drop Count", "count", [-0.2, 4.4], NaN);

    ax = ax_next();
    axis(ax, "off");
    text(ax, 0.02, 0.82, sprintf("Drops: %.0f / 4", dropCount(idx)), "FontSize", 14, "FontWeight", "bold");
    text(ax, 0.02, 0.66, sprintf("Altitude: %.2f m", alt(idx)), "FontSize", 12);
    text(ax, 0.02, 0.53, sprintf("Airspeed: %.2f m/s", airspeed(idx)), "FontSize", 12);
    text(ax, 0.02, 0.40, sprintf("Roll: %.2f deg", roll(idx)), "FontSize", 12);
    text(ax, 0.02, 0.27, sprintf("Heading: %.2f deg", heading(idx)), "FontSize", 12);
    text(ax, 0.02, 0.14, sprintf("Elevator: %.3f", elev(idx)), "FontSize", 12);

    drawnow;
    writeVideo(writer, getframe(fig));
end

close(writer);
if strcmpi(opts.Visible, "off")
    close(fig);
end
fprintf("AirdropX evaluation video written: %s\n", outputFile);
end

function ax = ax_next()
ax = nexttile;
end

function local_plot(ax, t, y, idx, plotTitle, yLabel, yLimits, refLine)
plot(ax, t(1:idx), y(1:idx), "LineWidth", 1.4);
hold(ax, "on");
plot(ax, t(idx), y(idx), "o", "MarkerFaceColor", [0.1 0.35 0.95], "MarkerEdgeColor", "none");
if isfinite(refLine)
    yline(ax, refLine, "--", "LineWidth", 1.0);
end
grid(ax, "on");
xlim(ax, [t(1), t(end)]);
if ~isempty(yLimits)
    ylim(ax, yLimits);
else
    ylim(ax, "padded");
end
title(ax, plotTitle);
xlabel(ax, "time (s)");
ylabel(ax, yLabel);
end

function local_plot_multi(ax, t, series, idx, labels, plotTitle, yLabel, yLimits)
hold(ax, "on");
for i = 1:numel(series)
    y = series{i};
    if all(isnan(y))
        continue;
    end
    plot(ax, t(1:idx), y(1:idx), "LineWidth", 1.4);
    plot(ax, t(idx), y(idx), "o", "MarkerEdgeColor", "none");
end
grid(ax, "on");
xlim(ax, [t(1), t(end)]);
if ~isempty(yLimits)
    ylim(ax, yLimits);
else
    ylim(ax, "padded");
end
title(ax, plotTitle);
xlabel(ax, "time (s)");
ylabel(ax, yLabel);
legend(ax, labels, "Location", "best");
end

function T = local_table(data)
if istable(data)
    T = data;
else
    path = string(data);
    if ~isfile(path)
        error("AirdropX eval video input not found: %s", path);
    end
    T = readtable(path);
end
end

function opts = local_options(varargin)
opts.OutputFile = "";
opts.Fps = 30;
opts.Width = 1280;
opts.Height = 720;
opts.Quality = 95;
opts.MaxFrames = 900;
opts.Visible = "on";
opts.SecondsPerVideoSecond = 1.0;

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

function local_cleanup(writer, fig)
try
    close(writer);
catch
end
try
    if isvalid(fig)
        close(fig);
    end
catch
end
end
