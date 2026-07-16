function result = airdropx_mpc_evaluate_csv(csvPath, varargin)
%AIRDROPX_MPC_EVALUATE_CSV Compute tracking metrics from an exported run.

opts = local_options(varargin{:});
cfg = airdropx_mpc_config( ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", opts.TargetPitchDeg);

T = readtable(csvPath);
segments = [
    "all", 0.0
    "after_drop", opts.AfterDropTime
    "last_10s", max(T.time_s) - 10.0
    "last_5s", max(T.time_s) - 5.0
    ];

S = table();
for i = 1:size(segments, 1)
    name = segments(i, 1);
    startTime = double(segments(i, 2));
    rows = T(T.time_s >= startTime, :);
    if isempty(rows)
        continue;
    end
    S = [S; local_segment_stats(name, rows, cfg)]; %#ok<AGROW>
end

lastRows = T(T.time_s >= max(T.time_s) - opts.TrimWindowS, :);
trim = struct();
trim.elevator_delta = mean(lastRows.elevator_cmd_norm, "omitnan");
trim.throttle_cmd = mean(lastRows.throttle_norm, "omitnan");
trim.altitude_m = mean(lastRows.altitude_m, "omitnan");
trim.airspeed_mps = mean(lastRows.airspeed_mps, "omitnan");
trim.pitch_deg = mean(lastRows.pitch_deg, "omitnan");

result = struct();
result.csv_path = string(csvPath);
result.cfg = cfg;
result.segments = S;
result.trim_estimate = trim;

if strlength(opts.OutputFile) > 0
    outDir = fileparts(opts.OutputFile);
    if strlength(string(outDir)) > 0 && ~isfolder(outDir)
        mkdir(outDir);
    end
    writetable(S, opts.OutputFile);
end
end

function row = local_segment_stats(name, T, cfg)
hRef = local_reference_column(T, "target_altitude_m", cfg.reference.h_m);
vRef = local_reference_column(T, "target_airspeed_mps", cfg.reference.v_mps);
pitchRef = local_reference_column(T, "target_pitch_deg", cfg.reference.pitch_deg);
hErr = T.altitude_m - hRef;
vErr = T.airspeed_mps - vRef;
pitchErr = T.pitch_deg - pitchRef;

row = table( ...
    string(name), ...
    T.time_s(1), ...
    T.time_s(end), ...
    mean(T.altitude_m, "omitnan"), ...
    rms(hErr), ...
    min(T.altitude_m), ...
    max(T.altitude_m), ...
    mean(T.airspeed_mps, "omitnan"), ...
    rms(vErr), ...
    min(T.airspeed_mps), ...
    max(T.airspeed_mps), ...
    mean(T.pitch_deg, "omitnan"), ...
    rms(pitchErr), ...
    min(T.pitch_deg), ...
    max(T.pitch_deg), ...
    mean(T.vz_up_mps, "omitnan"), ...
    'VariableNames', { ...
        'segment', 'start_s', 'end_s', ...
        'h_mean_m', 'h_err_rms_m', 'h_min_m', 'h_max_m', ...
        'v_mean_mps', 'v_err_rms_mps', 'v_min_mps', 'v_max_mps', ...
        'pitch_mean_deg', 'pitch_err_rms_deg', 'pitch_min_deg', 'pitch_max_deg', ...
        'vz_mean_mps'});
end

function ref = local_reference_column(T, name, fallback)
if ismember(name, string(T.Properties.VariableNames))
    ref = double(T.(name)(:));
else
    ref = fallback * ones(height(T), 1);
end
end

function opts = local_options(varargin)
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.AfterDropTime = 10.6;
opts.TrimWindowS = 5.0;
opts.OutputFile = "";

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
