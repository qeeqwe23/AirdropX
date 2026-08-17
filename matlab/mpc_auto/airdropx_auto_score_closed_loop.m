function result = airdropx_auto_score_closed_loop(data, varargin)
%AIRDROPX_AUTO_SCORE_CLOSED_LOOP Score auto-MPC closed-loop with transient care.
%
% The first StartTimeS seconds still count for safety and convergence speed,
% while the post-start window is weighted more heavily for pitch stability.

opts = local_options(varargin{:});
if istable(data)
    T = data;
else
    T = readtable(data);
end
T = local_fill_aliases(T);

if ~ismember("time_s", string(T.Properties.VariableNames))
    error("AirdropX:AutoMPC:MissingTime", "Timeseries must contain time_s.");
end

time = double(T.time_s(:));
maskAll = isfinite(time);
maskSteady = maskAll & time >= double(opts.StartTimeS);
maskTransient = maskAll & time < double(opts.StartTimeS);
if nnz(maskSteady) < 5
    maskSteady = maskAll;
end

h = double(T.altitude_m(:));
Va = double(T.airspeed_mps(:));
pitch = double(T.pitch_deg(:));
vz = double(T.vz_up_mps(:));
q = double(T.q_dps(:));
elevator = double(T.elevator_cmd(:));
throttle = double(T.throttle_cmd(:));

targetH = local_column(T, "target_altitude_m", opts.TargetAltitudeM);
targetVa = local_column(T, "target_airspeed_mps", opts.TargetAirspeedMps);

steadyHErr = h(maskSteady) - targetH(maskSteady);
steadyH = local_rms(steadyHErr);
steadyHMaxAbs = local_max_abs(steadyHErr);
steadyHDrift = local_edge_drift_abs(h(maskSteady));
steadyVa = local_rms(Va(maskSteady) - targetVa(maskSteady));
steadyVz = local_rms(vz(maskSteady));
steadyQ = local_rms(q(maskSteady));
steadyPitchMean = mean(pitch(maskSteady), "omitnan");
steadyPitchStd = local_std(pitch(maskSteady));
steadyPitchRange = local_range(pitch(maskSteady));
steadyPitchDrift = local_abs_slope(time(maskSteady), pitch(maskSteady));

% Explicit full-trajectory metrics. These make MPC bayesopt learn from the
% entire flight in addition to the transient/steady split.
fullHErr = h(maskAll) - targetH(maskAll);
fullH = local_rms(fullHErr);
fullHMaxAbs = local_max_abs(fullHErr);
fullVa = local_rms(Va(maskAll) - targetVa(maskAll));
fullVz = local_rms(vz(maskAll));
fullQ = local_rms(q(maskAll));

transientHErr = h(maskTransient) - targetH(maskTransient);
transientH = local_rms(transientHErr);
transientHMaxAbs = local_max_abs(transientHErr);
transientVa = local_rms(Va(maskTransient) - targetVa(maskTransient));
transientVz = local_rms(vz(maskTransient));
transientQ = local_rms(q(maskTransient));
transientPitchMean = mean(pitch(maskTransient), "omitnan");
transientPitchStd = local_std(pitch(maskTransient));

minHAll = min(h(maskAll), [], "omitnan");
minHSteady = min(h(maskSteady), [], "omitnan");
lowAltAll = local_low_altitude_penalty(minHAll, opts.MinSafeAltitudeM, opts.LowAltitudeWeight);
lowAltHard = local_low_altitude_penalty(minHAll, opts.HardFloorAltitudeM, opts.HardFloorWeight);
lowAltSteady = local_low_altitude_penalty(minHSteady, opts.SteadyMinAltitudeM, opts.SteadyLowAltitudeWeight);

settlePenalty = local_settle_penalty(time, h, pitch, vz, targetH, opts);
ratePenalty = double(opts.ElevatorRateWeight) * local_rms(diff(elevator)).^2 + ...
    double(opts.ThrottleRateWeight) * local_rms(diff(throttle)).^2;

scoreSteady = ...
    double(opts.SteadyAltitudeWeight) * (steadyH / double(opts.SteadyAltitudeScaleM)).^2 + ...
    double(opts.SteadyAltitudeMaxWeight) * (steadyHMaxAbs / double(opts.SteadyAltitudeMaxScaleM)).^2 + ...
    double(opts.SteadyAltitudeDriftWeight) * (steadyHDrift / double(opts.SteadyAltitudeDriftScaleM)).^2 + ...
    double(opts.SteadyAirspeedWeight) * (steadyVa / double(opts.SteadyAirspeedScaleMps)).^2 + ...
    double(opts.SteadyVzWeight) * (steadyVz / double(opts.SteadyVzScaleMps)).^2 + ...
    double(opts.SteadyQWeight) * (steadyQ / double(opts.SteadyQScaleDps)).^2 + ...
    double(opts.PitchStdWeight) * (steadyPitchStd / double(opts.PitchStdScaleDeg)).^2 + ...
    double(opts.PitchRangeWeight) * (steadyPitchRange / double(opts.PitchRangeScaleDeg)).^2 + ...
    double(opts.PitchDriftWeight) * (steadyPitchDrift / double(opts.PitchDriftScaleDegps)).^2;

scoreTransient = ...
    double(opts.TransientAltitudeWeight) * (transientH / double(opts.TransientAltitudeScaleM)).^2 + ...
    double(opts.TransientAltitudeMaxWeight) * (transientHMaxAbs / double(opts.TransientAltitudeMaxScaleM)).^2 + ...
    double(opts.TransientAirspeedWeight) * (transientVa / double(opts.TransientAirspeedScaleMps)).^2 + ...
    double(opts.TransientVzWeight) * (transientVz / double(opts.TransientVzScaleMps)).^2 + ...
    double(opts.TransientQWeight) * (transientQ / double(opts.TransientQScaleDps)).^2 + ...
    double(opts.TransientPitchStdWeight) * (transientPitchStd / double(opts.TransientPitchStdScaleDeg)).^2;

scoreFull = ...
    double(opts.FullAltitudeWeight) * (fullH / double(opts.FullAltitudeScaleM)).^2 + ...
    double(opts.FullAltitudeMaxWeight) * (fullHMaxAbs / double(opts.FullAltitudeMaxScaleM)).^2 + ...
    double(opts.FullAirspeedWeight) * (fullVa / double(opts.FullAirspeedScaleMps)).^2 + ...
    double(opts.FullVzWeight) * (fullVz / double(opts.FullVzScaleMps)).^2 + ...
    double(opts.FullQWeight) * (fullQ / double(opts.FullQScaleDps)).^2;

score = scoreSteady + double(opts.TransientWeight) * scoreTransient + ...
    double(opts.FullTrajectoryWeight) * scoreFull + ...
    lowAltAll + lowAltHard + lowAltSteady + settlePenalty + ratePenalty;

result = struct();
result.score = score;
result.metrics = table(score, scoreSteady, scoreTransient, scoreFull, fullH, fullHMaxAbs, fullVa, fullVz, fullQ, ...
    steadyH, steadyHMaxAbs, steadyHDrift, steadyVa, steadyVz, steadyQ, ...
    steadyPitchMean, steadyPitchStd, steadyPitchRange, steadyPitchDrift, transientH, transientHMaxAbs, transientVa, transientVz, transientQ, ...
    transientPitchMean, transientPitchStd, minHAll, minHSteady, lowAltAll, lowAltHard, lowAltSteady, settlePenalty, ...
    'VariableNames', {'score','score_steady','score_transient','score_full','full_h_rms_m','full_h_max_abs_m', ...
    'full_airspeed_rms_mps','full_vz_rms_mps','full_q_rms_dps','steady_h_rms_m','steady_h_max_abs_m','steady_h_drift_m', ...
    'steady_airspeed_rms_mps','steady_vz_rms_mps','steady_q_rms_dps','steady_pitch_mean_deg','steady_pitch_std_deg', ...
    'steady_pitch_range_deg','steady_pitch_drift_degps','transient_h_rms_m','transient_h_max_abs_m','transient_airspeed_rms_mps', ...
    'transient_vz_rms_mps','transient_q_rms_dps','transient_pitch_mean_deg','transient_pitch_std_deg','min_altitude_m', ...
    'steady_min_altitude_m','low_altitude_penalty','hard_floor_penalty','steady_low_altitude_penalty','settle_penalty'});
end

function penalty = local_settle_penalty(time, h, pitch, vz, targetH, opts)
mask = isfinite(time) & isfinite(h) & isfinite(pitch) & isfinite(vz);
time = time(mask);
h = h(mask);
pitch = pitch(mask);
vz = vz(mask);
targetH = targetH(mask);
if isempty(time)
    penalty = Inf;
    return;
end
hOk = abs(h - targetH) <= double(opts.SettleAltitudeBandM);
vzOk = abs(vz) <= double(opts.SettleVzBandMps);
pitchOk = movstd(pitch, max(3, round(double(opts.SettleWindowS) / max(median(diff(unique(time))), 0.1))), "omitnan") <= double(opts.SettlePitchStdDeg);
ok = hOk & vzOk & pitchOk;
idx = find(time >= double(opts.StartTimeS) & ok, 1, "first");
if isempty(idx)
    delay = max(time) - double(opts.StartTimeS) + double(opts.SettleWindowS);
else
    delay = max(0.0, time(idx) - double(opts.StartTimeS));
end
penalty = double(opts.SettleDelayWeight) * delay.^2;
end

function penalty = local_low_altitude_penalty(minH, floorH, weight)
if ~isfinite(minH) || minH >= double(floorH)
    penalty = 0.0;
else
    penalty = double(weight) * (double(floorH) - minH).^2;
end
end

function T = local_fill_aliases(T)
vars = string(T.Properties.VariableNames);
if ~ismember("elevator_cmd", vars)
    if ismember("elevator_cmd_norm", vars)
        T.elevator_cmd = T.elevator_cmd_norm;
    elseif ismember("elevator_delta", vars)
        T.elevator_cmd = T.elevator_delta;
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

function value = local_std(x)
x = double(x(:));
x = x(isfinite(x));
if numel(x) < 2
    value = 0.0;
else
    value = std(x);
end
end

function value = local_range(x)
x = double(x(:));
x = x(isfinite(x));
if isempty(x)
    value = NaN;
else
    value = max(x) - min(x);
end
end

function value = local_max_abs(x)
x = double(x(:));
x = x(isfinite(x));
if isempty(x), value = NaN; else, value = max(abs(x)); end
end

function value = local_edge_drift_abs(x)
x = double(x(:));
x = x(isfinite(x));
if numel(x) < 4
    value = 0.0;
    return;
end
n = numel(x);
e = max(1, round(0.20 * n));
a = median(x(1:e), "omitnan");
b = median(x(max(1,n-e+1):n), "omitnan");
value = abs(b - a);
end

function value = local_abs_slope(t, x)
t = double(t(:));
x = double(x(:));
mask = isfinite(t) & isfinite(x);
t = t(mask);
x = x(mask);
if numel(t) < 3 || range(t) <= 0
    value = 0.0;
else
    p = polyfit(t - t(1), x, 1);
    value = abs(p(1));
end
end

function opts = local_options(varargin)
opts.StartTimeS = 10.0;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.MinSafeAltitudeM = 8.0;
opts.HardFloorAltitudeM = 2.0;
opts.SteadyMinAltitudeM = 14.0;
% Height is the primary mission objective.  All score terms are normalized
% by an engineering scale so pitch cannot compensate for poor altitude hold.
opts.SteadyAltitudeWeight = 120.0;
opts.SteadyAltitudeMaxWeight = 60.0;
opts.SteadyAltitudeDriftWeight = 50.0;
opts.SteadyAirspeedWeight = 35.0;
opts.SteadyVzWeight = 20.0;
opts.SteadyQWeight = 10.0;
opts.PitchStdWeight = 8.0;
opts.PitchRangeWeight = 2.0;
opts.PitchDriftWeight = 8.0;
opts.SteadyAltitudeScaleM = 0.75;
opts.SteadyAltitudeMaxScaleM = 1.5;
opts.SteadyAltitudeDriftScaleM = 0.75;
opts.SteadyAirspeedScaleMps = 1.0;
opts.SteadyVzScaleMps = 0.50;
opts.SteadyQScaleDps = 0.75;
opts.PitchStdScaleDeg = 0.75;
opts.PitchRangeScaleDeg = 2.0;
opts.PitchDriftScaleDegps = 0.12;

opts.FullTrajectoryWeight = 0.50;
opts.FullAltitudeWeight = 70.0;
opts.FullAltitudeMaxWeight = 60.0;
opts.FullAirspeedWeight = 20.0;
opts.FullVzWeight = 12.0;
opts.FullQWeight = 6.0;
opts.FullAltitudeScaleM = 1.5;
opts.FullAltitudeMaxScaleM = 3.5;
opts.FullAirspeedScaleMps = 2.0;
opts.FullVzScaleMps = 1.0;
opts.FullQScaleDps = 1.5;

% The first StartTimeS seconds matter, but less than the final regulated
% window.  This rewards fast capture without making trim release dominate.
opts.TransientWeight = 0.35;
opts.TransientAltitudeWeight = 35.0;
opts.TransientAltitudeMaxWeight = 15.0;
opts.TransientAirspeedWeight = 12.0;
opts.TransientVzWeight = 8.0;
opts.TransientQWeight = 4.0;
opts.TransientPitchStdWeight = 2.0;
opts.TransientAltitudeScaleM = 2.0;
opts.TransientAltitudeMaxScaleM = 4.0;
opts.TransientAirspeedScaleMps = 3.0;
opts.TransientVzScaleMps = 2.0;
opts.TransientQScaleDps = 2.0;
opts.TransientPitchStdScaleDeg = 3.0;
opts.LowAltitudeWeight = 160.0;
opts.HardFloorWeight = 5000.0;
opts.SteadyLowAltitudeWeight = 300.0;
opts.ElevatorRateWeight = 0.5;
opts.ThrottleRateWeight = 0.2;
opts.SettleAltitudeBandM = 2.5;
opts.SettleVzBandMps = 0.6;
opts.SettlePitchStdDeg = 1.0;
opts.SettleWindowS = 2.0;
opts.SettleDelayWeight = 8.0;
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
end


