function videoFile = airdropx_render_video(logs, videoFile, varargin)
%AIRDROPX_RENDER_VIDEO Render a high quality MP4 from Simulink logs.
%
% Usage:
%   airdropx_render_video(out.logsout)
%   airdropx_render_video(out.logsout, "outputs/videos/demo.mp4", "Fps", 30)
%
% Required logged signals:
%   altitude_m, pos_n_m, pos_e_m, pitch_deg, roll_deg, heading_deg,
%   airspeed_mps, wind_n_mps, wind_e_mps, drop_count

opts = local_options(varargin{:});

if nargin < 2 || strlength(string(videoFile)) == 0
    videoFile = fullfile(pwd, "outputs", "videos", "airdropx_flight.mp4");
end
videoFile = char(videoFile);
outDir = fileparts(videoFile);
if ~isempty(outDir) && ~exist(outDir, 'dir')
    mkdir(outDir);
end

[t, altitude] = local_signal(logs, "altitude_m");
[~, posN] = local_signal(logs, "pos_n_m");
[~, posE] = local_signal(logs, "pos_e_m");
[~, pitch] = local_signal(logs, "pitch_deg");
[~, roll] = local_signal(logs, "roll_deg");
[~, heading] = local_signal(logs, "heading_deg");
[~, airspeed] = local_signal(logs, "airspeed_mps");
[~, windN] = local_signal(logs, "wind_n_mps");
[~, windE] = local_signal(logs, "wind_e_mps");
[~, dropCount] = local_signal(logs, "drop_count");

required = {t, altitude, posN, posE, pitch, roll, heading, airspeed, windN, windE, dropCount};
if any(cellfun(@isempty, required))
    error("airdropx_render_video:MissingSignal", ...
        "Missing one or more required logs. Log altitude_m, pos_n_m, pos_e_m, pitch_deg, roll_deg, heading_deg, airspeed_mps, wind_n_mps, wind_e_mps, drop_count.");
end

n = min(cellfun(@numel, required));
t = t(1:n);
altitude = altitude(1:n);
posN = posN(1:n);
posE = posE(1:n);
pitch = pitch(1:n);
roll = roll(1:n);
heading = heading(1:n);
airspeed = airspeed(1:n);
windN = windN(1:n);
windE = windE(1:n);
dropCount = dropCount(1:n);

frameTimes = t(1):1.0/opts.Fps:t(end);
if numel(frameTimes) > opts.MaxFrames
    frameTimes = linspace(t(1), t(end), opts.MaxFrames);
end

dropEvents = local_drop_events(t, posN, posE, altitude, airspeed, heading, windN, windE, dropCount);

fig = figure('Name', 'AirdropX Video Render', 'Color', 'w', ...
    'Position', [80 80 opts.Width opts.Height], 'Visible', opts.Visible);
ax = axes(fig);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, 35, 18);
xlabel(ax, 'East (m)');
ylabel(ax, 'North (m)');
zlabel(ax, 'Altitude (m)');
title(ax, 'AirdropX Flight and Cargo Drop');

groundZ = 0.0;
targetN = opts.TargetN;
targetE = opts.TargetE;

spanE = max(80, range(posE) * 0.35);
spanN = max(120, range(posN) * 0.35);
zMax = max(45, max(altitude) + 30);

writer = VideoWriter(videoFile, 'MPEG-4');
writer.FrameRate = opts.Fps;
writer.Quality = opts.Quality;
open(writer);

cleanupObj = onCleanup(@() local_close_writer(writer));

for i = 1:numel(frameTimes)
    ti = frameTimes(i);
    pe = interp1(t, posE, ti, 'linear', 'extrap');
    pn = interp1(t, posN, ti, 'linear', 'extrap');
    h = interp1(t, altitude, ti, 'linear', 'extrap');
    ph = interp1(t, pitch, ti, 'linear', 'extrap');
    rr = interp1(t, roll, ti, 'linear', 'extrap');
    hd = interp1(t, heading, ti, 'linear', 'extrap');

    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 35, 18);
    xlabel(ax, 'East (m)');
    ylabel(ax, 'North (m)');
    zlabel(ax, 'Altitude (m)');
    title(ax, sprintf('AirdropX flight replay   t = %.2f s', ti));

    local_draw_ground(ax, pe, pn, targetE, targetN, groundZ);
    plot3(ax, posE(1:max(1, find(t <= ti, 1, 'last'))), posN(1:max(1, find(t <= ti, 1, 'last'))), ...
        altitude(1:max(1, find(t <= ti, 1, 'last'))), 'Color', [0.1 0.35 0.95], 'LineWidth', 1.4);
    local_draw_target(ax, targetE, targetN, groundZ);

    for k = 1:numel(dropEvents)
        ev = dropEvents(k);
        if ti >= ev.t
            [cn, ce, ch] = airdropx_cargo_trajectory_point(ti - ev.t, ev.n, ev.e, ev.h, ev.v, ev.heading, ev.windN, ev.windE);
            local_draw_cargo(ax, ce, cn, ch, k);
            plot3(ax, ce, cn, max(ch, groundZ), '.', 'Color', local_color(k), 'MarkerSize', 18);
        end
    end

    local_draw_aircraft(ax, pe, pn, h, rr, ph, hd);

    xlim(ax, [pe - spanE, pe + spanE]);
    ylim(ax, [pn - spanN * 0.55, pn + spanN]);
    zlim(ax, [0, zMax]);

    camtarget(ax, [pe, pn + 55, max(5, h - 10)]);
    campos(ax, [pe - 125, pn - 185, h + 110]);
    camup(ax, [0 0 1]);

    drawnow;
    writeVideo(writer, getframe(fig));
end

close(writer);
delete(cleanupObj);
fprintf("Wrote video: %s\n", videoFile);
end

function opts = local_options(varargin)
cfg = airdropx_sim_params();
opts.Fps = cfg.video.fps;
opts.Width = cfg.video.width;
opts.Height = cfg.video.height;
opts.Quality = cfg.video.quality;
opts.MaxFrames = cfg.video.max_frames;
opts.TargetN = cfg.carp.target_n_m;
opts.TargetE = cfg.carp.target_e_m;
opts.Visible = char(cfg.video.visible);

if mod(numel(varargin), 2) ~= 0
    error("Name-value options must be pairs.");
end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    value = varargin{i+1};
    if isfield(opts, name)
        opts.(name) = value;
    else
        error("Unknown option: %s", name);
    end
end
end

function events = local_drop_events(t, posN, posE, altitude, airspeed, heading, windN, windE, dropCount)
events = struct('t', {}, 'n', {}, 'e', {}, 'h', {}, 'v', {}, 'heading', {}, 'windN', {}, 'windE', {});
prev = floor(dropCount(1));
for i = 2:numel(dropCount)
    dc = floor(dropCount(i));
    if dc > prev
        for k = (prev + 1):min(dc, 4)
            events(end+1) = struct( ...
                't', t(i), ...
                'n', posN(i), ...
                'e', posE(i), ...
                'h', altitude(i), ...
                'v', airspeed(i), ...
                'heading', heading(i), ...
                'windN', windN(i), ...
                'windE', windE(i)); %#ok<AGROW>
        end
    end
    prev = dc;
end
end

function local_draw_ground(ax, pe, pn, targetE, targetN, z)
eMin = min(pe - 260, targetE - 80);
eMax = max(pe + 260, targetE + 80);
nMin = min(pn - 260, targetN - 140);
nMax = max(pn + 360, targetN + 140);
patch(ax, [eMin eMax eMax eMin], [nMin nMin nMax nMax], [z z z z], ...
    [0.58 0.72 0.50], 'EdgeColor', 'none', 'FaceAlpha', 0.85);
end

function local_draw_target(ax, e, n, z)
plot3(ax, [e - 14, e + 14], [n, n], [z + 0.1, z + 0.1], 'r-', 'LineWidth', 2.2);
plot3(ax, [e, e], [n - 14, n + 14], [z + 0.1, z + 0.1], 'r-', 'LineWidth', 2.2);
text(ax, e + 5, n + 5, z + 1.5, 'Target', 'Color', 'r', 'FontWeight', 'bold');
end

function local_draw_aircraft(ax, e, n, h, rollDeg, pitchDeg, headingDeg)
R = local_aircraft_rotation(rollDeg, pitchDeg, headingDeg);
body = local_transform(R, [0 0 0; 9 0 0; -5 0 0].', [e; n; h]);
wings = local_transform(R, [0 -12 0; 0 12 0].', [e; n; h]);
tail = local_transform(R, [-4 -4 0; -4 4 0; -5 0 2.4].', [e; n; h]);

plot3(ax, body(1,:), body(2,:), body(3,:), 'Color', [0.05 0.12 0.75], 'LineWidth', 4.0);
plot3(ax, wings(1,:), wings(2,:), wings(3,:), 'Color', [0.15 0.22 0.85], 'LineWidth', 3.2);
plot3(ax, tail(1,1:2), tail(2,1:2), tail(3,1:2), 'Color', [0.12 0.16 0.70], 'LineWidth', 2.3);
plot3(ax, tail(1,[1 3]), tail(2,[1 3]), tail(3,[1 3]), 'Color', [0.12 0.16 0.70], 'LineWidth', 2.3);
plot3(ax, tail(1,[2 3]), tail(2,[2 3]), tail(3,[2 3]), 'Color', [0.12 0.16 0.70], 'LineWidth', 2.3);
scatter3(ax, body(1,2), body(2,2), body(3,2), 55, [0.02 0.05 0.25], 'filled');
end

function local_draw_cargo(ax, e, n, h, k)
c = local_color(k);
[x, y, z] = sphere(10);
surf(ax, e + 1.2*x, n + 1.2*y, h + 1.2*z, ...
    'FaceColor', c, 'EdgeColor', 'none', 'FaceAlpha', 0.95);
end

function c = local_color(k)
colors = [0.95 0.12 0.05;
          0.98 0.45 0.05;
          0.85 0.75 0.05;
          0.05 0.65 0.25];
c = colors(mod(k - 1, size(colors, 1)) + 1, :);
end

function R = local_aircraft_rotation(rollDeg, pitchDeg, headingDeg)
roll = deg2rad(double(rollDeg));
pitch = deg2rad(double(pitchDeg));
heading = deg2rad(double(headingDeg));
forward = [sin(heading); cos(heading); 0.0];
right = [cos(heading); -sin(heading); 0.0];
up = [0.0; 0.0; 1.0];
R_heading = [forward, right, up];
R = R_heading * local_rotx(roll) * local_roty(-pitch);
end

function R = local_rotx(a)
c = cos(a); s = sin(a);
R = [1 0 0; 0 c -s; 0 s c];
end

function R = local_roty(a)
c = cos(a); s = sin(a);
R = [c 0 s; 0 1 0; -s 0 c];
end

function p = local_transform(R, localPoints, origin)
p = R * localPoints + origin;
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
    if isfield(logs, "time")
        t = logs.time(:);
    end
    if isfield(logs, name)
        y = logs.(name)(:);
        if isempty(t)
            t = (0:numel(y)-1).';
        end
    end
end
end

function local_close_writer(writer)
try
    close(writer);
catch
end
end
