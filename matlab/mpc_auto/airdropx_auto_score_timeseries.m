function result = airdropx_auto_score_timeseries(data, varargin)
%AIRDROPX_AUTO_SCORE_TIMESERIES Score closed-loop JSBSim timeseries.

opts = local_options(varargin{:});
if istable(data)
    T = data;
else
    T = readtable(data);
end
T = local_fill_aliases(T);

h = double(T.altitude_m(:));
Va = double(T.airspeed_mps(:));
pitch = double(T.pitch_deg(:));
vz = double(T.vz_up_mps(:));
q = double(T.q_dps(:));
elevator = double(T.elevator_cmd(:));
throttle = double(T.throttle_cmd(:));

targetH = local_column(T, "target_altitude_m", opts.TargetAltitudeM);
targetVa = local_column(T, "target_airspeed_mps", opts.TargetAirspeedMps);

pitchViolation = max(0.0, pitch - double(opts.PitchMaxDeg)).^2 + ...
    max(0.0, double(opts.PitchMinDeg) - pitch).^2;
lowAltitudePenalty = 0.0;
minH = min(h, [], "omitnan");
if isfinite(minH) && minH < double(opts.MinSafeAltitudeM)
    lowAltitudePenalty = double(opts.LowAltitudeWeight) * (double(opts.MinSafeAltitudeM) - minH)^2;
end

score = ...
    double(opts.AltitudeWeight) * local_rms(h - targetH)^2 + ...
    double(opts.AirspeedWeight) * local_rms(Va - targetVa)^2 + ...
    double(opts.VzWeight) * local_rms(vz)^2 + ...
    double(opts.QWeight) * local_rms(q)^2 + ...
    double(opts.PitchStdWeight) * local_std_omitnan(pitch)^2 + ...
    double(opts.ElevatorRateWeight) * local_rms(diff(elevator))^2 + ...
    double(opts.ThrottleRateWeight) * local_rms(diff(throttle))^2 + ...
    double(opts.PitchViolationWeight) * mean(pitchViolation, "omitnan") + ...
    lowAltitudePenalty;

result = struct();
result.score = score;
result.metrics = table( ...
    score, local_rms(h - targetH), local_rms(Va - targetVa), local_rms(vz), local_rms(q), ...
    local_std_omitnan(pitch), minH, max(h, [], "omitnan"), local_rms(diff(elevator)), local_rms(diff(throttle)), ...
    mean(pitchViolation, "omitnan"), lowAltitudePenalty, ...
    'VariableNames', {'score', 'h_rms_m', 'airspeed_rms_mps', 'vz_rms_mps', 'q_rms_dps', ...
    'pitch_std_deg', 'min_altitude_m', 'max_altitude_m', 'elevator_rate_rms', 'throttle_rate_rms', ...
    'pitch_violation_mean', 'low_altitude_penalty'});
end

function T = local_fill_aliases(T)
vars = string(T.Properties.VariableNames);
if ~ismember("elevator_cmd", vars)
    if ismember("elevator_delta", vars)
        T.elevator_cmd = T.elevator_delta;
    elseif ismember("elevator_cmd_norm", vars)
        T.elevator_cmd = T.elevator_cmd_norm;
    else
        T.elevator_cmd = zeros(height(T), 1);
    end
end
if ~ismember("throttle_cmd", vars)
    if ismember("throttle_norm", vars)
        T.throttle_cmd = T.throttle_norm;
    else
        T.throttle_cmd = zeros(height(T), 1);
    end
end
if ~ismember("q_dps", vars)
    if ismember("pitch_deg", vars) && ismember("time_s", vars)
        T.q_dps = gradient(double(T.pitch_deg), double(T.time_s));
    else
        T.q_dps = zeros(height(T), 1);
    end
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

function value = local_std_omitnan(x)
x = double(x(:));
x = x(isfinite(x));
if isempty(x)
    value = NaN;
else
    value = std(x);
end
end

function opts = local_options(varargin)
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.MinSafeAltitudeM = 18.0;
opts.PitchMinDeg = 0.0;
opts.PitchMaxDeg = 8.0;
opts.AltitudeWeight = 8.0;
opts.AirspeedWeight = 5.0;
opts.VzWeight = 3.0;
opts.QWeight = 2.0;
opts.PitchStdWeight = 1.0;
opts.ElevatorRateWeight = 0.5;
opts.ThrottleRateWeight = 0.2;
opts.PitchViolationWeight = 20.0;
opts.LowAltitudeWeight = 100.0;
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
end
