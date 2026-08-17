function data = airdropx_auto_build_iddata(varargin)
%AIRDROPX_AUTO_BUILD_IDDATA Build clean multi-experiment iddata sets by config.
%
% v11 ID rules:
%   1) The verified trim bank remains the authoritative MPC nominal point.
%   2) Each ID run must contain a pre-excitation baseline window.
%   3) Each experiment is centered on its own measured pre-excitation baseline.
%   4) Raw JSBSim logging is resampled by time to the requested ID sample time.
%   5) Train/validation/test are kept independent whenever >=3 runs exist.
%
% This avoids two v10 failure modes: estimating cfg2's nominal point from an
% already-excited trajectory, and treating ~120 Hz raw samples as if they were
% 0.1 s samples.

opts = local_options(varargin{:});
requestedTrimBank = opts.TrimBank;
if isempty(requestedTrimBank)
    requestedTrimBank = airdropx_auto_default_trim_bank( ...
        "TargetAltitudeM", opts.TargetAltitudeM, ...
        "TargetAirspeedMps", opts.TargetAirspeedMps);
end

csvFiles = local_csv_files(opts);
if isempty(csvFiles)
    error("AirdropX:AutoMPC:NoCsv", "No auto_id_timeseries.csv files found.");
end

% Estimate a diagnostic ID acquisition operating point only from the true
% pre-excitation baseline.  Never let the excited-window median silently move
% the verified MPC trim.
idOperatingBank = local_estimate_operating_point_bank(csvFiles, requestedTrimBank, opts);
if logical(opts.UseVerifiedTrimForNominal) && ~isempty(opts.TrimBank)
    trimBank = requestedTrimBank;
elseif logical(opts.EstimateOperatingPointFromCsv)
    trimBank = idOperatingBank;
else
    trimBank = requestedTrimBank;
end

byConfig = repmat(struct("train", [], "validation", [], "test", [], ...
    "runs", strings(0,1), "accepted_runs", strings(0,1)), 5, 1);
qualityRows = table();

for cfgId = 0:4
    cfgFiles = local_files_for_config(csvFiles, cfgId);
    if isempty(cfgFiles)
        continue;
    end

    op = requestedTrimBank(cfgId + 1);
    Q = local_quality_files(cfgFiles, op, opts);
    qualityRows = [qualityRows; Q]; %#ok<AGROW>

    acceptedMask = true(numel(cfgFiles), 1);
    if logical(opts.RejectBadBaseline)
        acceptedMask = false(numel(cfgFiles), 1);
        for i = 1:numel(cfgFiles)
            qidx = find(string(Q.csv_file) == string(cfgFiles(i)), 1, "first");
            if ~isempty(qidx)
                acceptedMask(i) = logical(Q.baseline_pass(qidx));
                if logical(opts.RequirePreExcitationBaseline)
                    acceptedMask(i) = acceptedMask(i) && logical(Q.has_pre_excitation_baseline(qidx));
                end
            end
        end
    end
    acceptedFiles = cfgFiles(acceptedMask);

    fprintf("[ID DATA] cfg%d accepted %d/%d runs for identification.\n", ...
        cfgId, numel(acceptedFiles), numel(cfgFiles));
    if numel(acceptedFiles) < double(opts.MinAcceptedRunsPerConfig)
        error("AirdropX:AutoMPC:InsufficientCleanIdRuns", ...
            ['Config %d has only %d clean ID runs (need at least %d). ' ...
             'Regenerate data with a longer settle hold or inspect id_data_quality.csv.'], ...
            cfgId, numel(acceptedFiles), round(double(opts.MinAcceptedRunsPerConfig)));
    end

    [trainIdx, valIdx, testIdx] = local_split(numel(acceptedFiles), ...
        opts.TrainFraction, opts.ValidationFraction);

    byConfig(cfgId + 1).train = local_merge_files( ...
        local_limit_files(acceptedFiles(trainIdx), opts.MaxTrainRunsPerConfig), op, opts.Ts, opts);
    byConfig(cfgId + 1).validation = local_merge_files( ...
        local_limit_files(acceptedFiles(valIdx), opts.MaxValidationRunsPerConfig), op, opts.Ts, opts);
    byConfig(cfgId + 1).test = local_merge_files( ...
        local_limit_files(acceptedFiles(testIdx), opts.MaxTestRunsPerConfig), op, opts.Ts, opts);
    byConfig(cfgId + 1).runs = local_run_names(cfgFiles);
    byConfig(cfgId + 1).accepted_runs = local_run_names(acceptedFiles);
end

data = struct();
data.by_config = byConfig;
data.trim_bank = trimBank;
data.requested_trim_bank = requestedTrimBank;
data.id_operating_point_bank = idOperatingBank;
data.operating_points = local_operating_point_table(trimBank, requestedTrimBank);
data.id_operating_points = local_operating_point_table(idOperatingBank, requestedTrimBank);
data.quality = qualityRows;
data.csv_files = csvFiles;
data.Ts = double(opts.Ts);
data.output_names = ["altitude_m"; "airspeed_mps"; "pitch_deg"; "vz_up_mps"; "q_dps"];
data.input_names = ["elevator_cmd"; "throttle_cmd"];
data.preprocessing = struct( ...
    "resample_to_Ts", logical(opts.ResampleToTs), ...
    "baseline_duration_s", double(opts.BaselineDurationS), ...
    "center_each_run_on_baseline", true, ...
    "verified_trim_for_nominal", logical(opts.UseVerifiedTrimForNominal));

if strlength(string(opts.OutputMat)) > 0
    outDir = fileparts(string(opts.OutputMat));
    if strlength(outDir) > 0 && ~isfolder(outDir), mkdir(outDir); end
    save(opts.OutputMat, "data");
    writetable(data.operating_points, fullfile(outDir, "auto_operating_points.csv"));
    writetable(data.id_operating_points, fullfile(outDir, "auto_id_operating_points.csv"));
    writetable(data.quality, fullfile(outDir, "id_data_quality.csv"));
end
end

function names = local_run_names(files)
names = strings(numel(files), 1);
for i = 1:numel(files)
    try
        T = readtable(files(i));
        if height(T) > 0 && ismember("run_id", string(T.Properties.VariableNames))
            names(i) = string(T.run_id(1));
        else
            [~, names(i)] = fileparts(fileparts(files(i)));
        end
    catch
        [~, names(i)] = fileparts(fileparts(files(i)));
    end
end
end

function opBank = local_estimate_operating_point_bank(csvFiles, requestedTrimBank, opts)
opBank = requestedTrimBank;
for cfgId = 0:4
    cfgFiles = local_files_for_config(csvFiles, cfgId);
    if isempty(cfgFiles), continue; end

    rows = NaN(numel(cfgFiles), 7);
    keep = false(numel(cfgFiles), 1);
    for i = 1:numel(cfgFiles)
        try
            T = readtable(cfgFiles(i));
            T = local_fill_aliases(T);
            [baseline, meta] = local_run_baseline(T, requestedTrimBank(cfgId + 1), opts);
            if ~meta.has_pre_excitation && logical(opts.RequirePreExcitationBaseline)
                continue;
            end
            rows(i,:) = [baseline.altitude_m, baseline.airspeed_mps, baseline.pitch_deg, ...
                baseline.vz_up_mps, baseline.q_dps, baseline.elevator_cmd, baseline.throttle_cmd];
            keep(i) = all(isfinite(rows(i,:)));
        catch
        end
    end
    if ~any(keep), continue; end

    values = rows(keep,:);
    op = opBank(cfgId + 1);
    op.config_id = cfgId;
    % Acquisition altitude is intentionally high.  The MPC nominal altitude is
    % still the mission target; keep the diagnostic baseline altitude separate.
    op.altitude_m = median(values(:,1), "omitnan");
    op.airspeed_mps = median(values(:,2), "omitnan");
    op.pitch_deg = median(values(:,3), "omitnan");
    op.vz_up_mps = median(values(:,4), "omitnan");
    op.q_dps = median(values(:,5), "omitnan");
    op.elevator_cmd = median(values(:,6), "omitnan");
    op.throttle_cmd = median(values(:,7), "omitnan");
    opBank(cfgId + 1) = op;

    fprintf("[ID BASELINE OP] cfg%d h=%.3f V=%.3f pitch=%.3f vz=%.3f q=%.3f elevator=%.5f throttle=%.5f\n", ...
        cfgId, op.altitude_m, op.airspeed_mps, op.pitch_deg, op.vz_up_mps, ...
        op.q_dps, op.elevator_cmd, op.throttle_cmd);
end
end

function T = local_operating_point_table(observed, requested)
config_id = (0:4).';
altitude_m = arrayfun(@(s) double(s.altitude_m), observed(:));
airspeed_mps = arrayfun(@(s) double(s.airspeed_mps), observed(:));
pitch_deg = arrayfun(@(s) double(s.pitch_deg), observed(:));
vz_up_mps = arrayfun(@(s) local_struct_field(s, "vz_up_mps", 0.0), observed(:));
q_dps = arrayfun(@(s) local_struct_field(s, "q_dps", 0.0), observed(:));
elevator_cmd = arrayfun(@(s) double(s.elevator_cmd), observed(:));
throttle_cmd = arrayfun(@(s) double(s.throttle_cmd), observed(:));
requested_pitch_deg = arrayfun(@(s) double(s.pitch_deg), requested(:));
requested_elevator_cmd = arrayfun(@(s) double(s.elevator_cmd), requested(:));
requested_throttle_cmd = arrayfun(@(s) double(s.throttle_cmd), requested(:));
T = table(config_id, altitude_m, airspeed_mps, pitch_deg, vz_up_mps, q_dps, elevator_cmd, throttle_cmd, ...
    requested_pitch_deg, requested_elevator_cmd, requested_throttle_cmd);
end

function csvFiles = local_csv_files(opts)
if ~isempty(opts.CsvFiles)
    csvFiles = local_string_list(opts.CsvFiles);
    return;
end
root = string(opts.InputRoot);
if strlength(root) == 0
    error("InputRoot or CsvFiles is required.");
end
files = dir(fullfile(root, "**", "auto_id_timeseries.csv"));
csvFiles = strings(numel(files), 1);
for i = 1:numel(files)
    csvFiles(i) = string(fullfile(files(i).folder, files(i).name));
end
end

function values = local_string_list(value)
if ischar(value) || (isstring(value) && isscalar(value))
    values = string(value);
elseif iscell(value)
    values = string(value(:));
else
    values = string(value(:));
end
values = values(strlength(values) > 0);
end

function cfgFiles = local_files_for_config(csvFiles, cfgId)
keep = false(numel(csvFiles), 1);
for i = 1:numel(csvFiles)
    try
        T = readtable(csvFiles(i));
        keep(i) = height(T) > 0 && ismember("config_id", string(T.Properties.VariableNames)) && ...
            round(double(T.config_id(1))) == cfgId;
    catch
        keep(i) = false;
    end
end
cfgFiles = csvFiles(keep);
end

function Q = local_quality_files(files, op, opts)
Q = table();
for i = 1:numel(files)
    try
        row = local_quality_one(files(i), op, opts);
        Q = [Q; row]; %#ok<AGROW>
    catch ME
        warning("AirdropX:AutoMPC:QualityRunSkipped", ...
            "Skipping data-quality summary for %s: %s", files(i), ME.message);
    end
end
end

function row = local_quality_one(file, op, opts)
T = readtable(file);
T = local_fill_aliases(T);
[baseline, meta] = local_run_baseline(T, op, opts);

raw_dt_median_s = NaN;
if ismember("time_s", string(T.Properties.VariableNames)) && height(T) > 2
    dt = diff(double(T.time_s));
    dt = dt(isfinite(dt) & dt > 0);
    if ~isempty(dt), raw_dt_median_s = median(dt); end
end

idMask = local_id_mask(T, meta.excitation_start_s, op.config_id, opts);
T_id = T(idMask,:);
if isempty(T_id)
    T_id = T;
end
n_raw_samples = height(T_id);
if logical(opts.ResampleToTs)
    T_id_r = local_resample_table(T_id, opts.Ts);
else
    T_id_r = T_id;
end
n_id_samples = height(T_id_r);

run_id = string("");
if ismember("run_id", string(T.Properties.VariableNames)) && height(T) > 0
    run_id = string(T.run_id(1));
end
config_id = double(op.config_id);
csv_file = string(file);
has_pre_excitation_baseline = logical(meta.has_pre_excitation);
excitation_start_s = double(meta.excitation_start_s);
baseline_duration_s = double(meta.baseline_duration_s);
baseline_pass = logical(meta.pass);
baseline_h_m = baseline.altitude_m;
baseline_V_mps = baseline.airspeed_mps;
baseline_pitch_deg = baseline.pitch_deg;
baseline_vz_mps = baseline.vz_up_mps;
baseline_q_dps = baseline.q_dps;
baseline_elevator = baseline.elevator_cmd;
baseline_throttle = baseline.throttle_cmd;
baseline_h_slope_mps = meta.h_slope_mps;
baseline_V_error_mps = baseline.airspeed_mps - double(op.airspeed_mps);
baseline_pitch_error_deg = baseline.pitch_deg - double(op.pitch_deg);

if ~isempty(T_id_r)
    elevator_rms_dev = local_rms(double(T_id_r.elevator_cmd) - baseline.elevator_cmd);
    throttle_rms_dev = local_rms(double(T_id_r.throttle_cmd) - baseline.throttle_cmd);
    airspeed_median_dev_mps = local_median(double(T_id_r.airspeed_mps) - baseline.airspeed_mps);
    pitch_median_dev_deg = local_median(double(T_id_r.pitch_deg) - baseline.pitch_deg);
    vz_median_dev_mps = local_median(double(T_id_r.vz_up_mps) - baseline.vz_up_mps);
    q_median_dev_dps = local_median(double(T_id_r.q_dps) - baseline.q_dps);
    altitude_median_dev_m = local_median(double(T_id_r.altitude_m) - baseline.altitude_m);
else
    elevator_rms_dev = NaN; throttle_rms_dev = NaN;
    airspeed_median_dev_mps = NaN; pitch_median_dev_deg = NaN;
    vz_median_dev_mps = NaN; q_median_dev_dps = NaN; altitude_median_dev_m = NaN;
end

row = table(config_id, run_id, csv_file, raw_dt_median_s, double(opts.Ts), ...
    n_raw_samples, n_id_samples, has_pre_excitation_baseline, excitation_start_s, ...
    baseline_duration_s, baseline_pass, baseline_h_m, baseline_V_mps, baseline_pitch_deg, ...
    baseline_vz_mps, baseline_q_dps, baseline_elevator, baseline_throttle, ...
    baseline_h_slope_mps, baseline_V_error_mps, baseline_pitch_error_deg, ...
    altitude_median_dev_m, airspeed_median_dev_mps, pitch_median_dev_deg, ...
    vz_median_dev_mps, q_median_dev_dps, elevator_rms_dev, throttle_rms_dev);
end

function [baseline, meta] = local_run_baseline(T, op, opts)
T = local_fill_aliases(T);
N = height(T);
if N < 3 || ~ismember("time_s", string(T.Properties.VariableNames))
    error("AirdropX:AutoMPC:NoTimeForBaseline", "ID CSV has insufficient time samples.");
end

t = double(T.time_s);
excitationStart = local_excitation_start(T, opts);
hasPre = isfinite(excitationStart) && any(t < excitationStart - 1e-9);
if ~isfinite(excitationStart)
    excitationStart = min(t, [], "omitnan") + double(opts.BaselineDurationS);
end
baseStart = excitationStart - double(opts.BaselineDurationS);
mask = isfinite(t) & t >= baseStart & t < excitationStart - 1e-9;
if ismember("config_id", string(T.Properties.VariableNames))
    mask = mask & round(double(T.config_id)) == double(op.config_id);
end
if ismember("altitude_m", string(T.Properties.VariableNames))
    mask = mask & isfinite(double(T.altitude_m)) & double(T.altitude_m) >= double(opts.OperatingPointMinAltitudeM);
end

if nnz(mask) < double(opts.BaselineMinSamples)
    % Backward-compatible diagnostic fallback for old v10 CSVs.  This is not
    % accepted as a clean v11 baseline when RequirePreExcitationBaseline=true.
    valid = isfinite(t);
    if ismember("config_id", string(T.Properties.VariableNames))
        valid = valid & round(double(T.config_id)) == double(op.config_id);
    end
    idx = find(valid);
    take = min(numel(idx), max(round(double(opts.BaselineMinSamples)), 3));
    mask = false(N,1);
    if take > 0, mask(idx(1:take)) = true; end
    hasPre = false;
end

baseline = struct();
baseline.altitude_m = local_median(double(T.altitude_m(mask)));
baseline.airspeed_mps = local_median(double(T.airspeed_mps(mask)));
baseline.pitch_deg = local_median(double(T.pitch_deg(mask)));
baseline.vz_up_mps = local_median(double(T.vz_up_mps(mask)));
baseline.q_dps = local_median(double(T.q_dps(mask)));
baseline.elevator_cmd = local_median(double(T.elevator_cmd(mask)));
baseline.throttle_cmd = local_median(double(T.throttle_cmd(mask)));

hSlope = local_slope(t(mask), double(T.altitude_m(mask)));
Verr = baseline.airspeed_mps - double(op.airspeed_mps);
pitchErr = baseline.pitch_deg - double(op.pitch_deg);
pass = hasPre && ...
    isfinite(baseline.vz_up_mps) && abs(baseline.vz_up_mps) <= double(opts.BaselineMaxAbsVzMps) && ...
    isfinite(baseline.q_dps) && abs(baseline.q_dps) <= double(opts.BaselineMaxAbsQDps) && ...
    isfinite(Verr) && abs(Verr) <= double(opts.BaselineMaxAirspeedErrorMps) && ...
    isfinite(pitchErr) && abs(pitchErr) <= double(opts.BaselineMaxPitchErrorDeg) && ...
    isfinite(hSlope) && abs(hSlope) <= double(opts.BaselineMaxHeightSlopeMps) && ...
    isfinite(baseline.elevator_cmd) && abs(baseline.elevator_cmd - double(op.elevator_cmd)) <= double(opts.BaselineMaxElevatorError) && ...
    isfinite(baseline.throttle_cmd) && abs(baseline.throttle_cmd - double(op.throttle_cmd)) <= double(opts.BaselineMaxThrottleError);

meta = struct("has_pre_excitation", hasPre, "excitation_start_s", excitationStart, ...
    "baseline_duration_s", max(0.0, excitationStart - min(t(mask), [], "omitnan")), ...
    "h_slope_mps", hSlope, "pass", pass);
end

function t0 = local_excitation_start(T, opts)
t0 = NaN;
vars = string(T.Properties.VariableNames);
if ismember("elevator_excitation", vars) || ismember("throttle_excitation", vars)
    e = zeros(height(T),1); th = zeros(height(T),1);
    if ismember("elevator_excitation", vars), e = abs(double(T.elevator_excitation)); end
    if ismember("throttle_excitation", vars), th = abs(double(T.throttle_excitation)); end
    active = (e > double(opts.ExcitationZeroTolerance)) | (th > double(opts.ExcitationZeroTolerance));
    idx = find(active & isfinite(double(T.time_s)), 1, "first");
    if ~isempty(idx), t0 = double(T.time_s(idx)); end
end
if ~isfinite(t0) && ismember("requested_elevator_cmd", vars) && ismember("requested_elevator_trim", vars)
    e = abs(double(T.requested_elevator_cmd) - double(T.requested_elevator_trim));
    th = zeros(height(T),1);
    if ismember("requested_throttle_cmd", vars) && ismember("requested_throttle_trim", vars)
        th = abs(double(T.requested_throttle_cmd) - double(T.requested_throttle_trim));
    end
    idx = find((e > double(opts.ExcitationZeroTolerance) | th > double(opts.ExcitationZeroTolerance)) & ...
        isfinite(double(T.time_s)), 1, "first");
    if ~isempty(idx), t0 = double(T.time_s(idx)); end
end
end

function mask = local_id_mask(T, excitationStart, cfgId, opts)
mask = true(height(T),1);
if ismember("time_s", string(T.Properties.VariableNames)) && isfinite(excitationStart)
    mask = mask & double(T.time_s) >= excitationStart - 1e-9;
end
if ismember("config_id", string(T.Properties.VariableNames))
    mask = mask & round(double(T.config_id)) == double(cfgId);
end
if ismember("altitude_m", string(T.Properties.VariableNames))
    mask = mask & isfinite(double(T.altitude_m)) & double(T.altitude_m) >= double(opts.OperatingPointMinAltitudeM);
end
end

function files = local_limit_files(files, maxFiles)
maxFiles = round(double(maxFiles));
if isfinite(maxFiles) && maxFiles > 0 && numel(files) > maxFiles
    files = files(1:maxFiles);
end
end

function [trainIdx, valIdx, testIdx] = local_split(n, trainFrac, valFrac)
idx = (1:n).';
if n == 0
    trainIdx = idx; valIdx = idx; testIdx = idx; return;
elseif n == 1
    trainIdx = idx; valIdx = idx; testIdx = idx; return;
elseif n == 2
    trainIdx = idx(1); valIdx = idx(2); testIdx = idx(2); return;
end

% Keep validation and test independent.  The old floor(0.15*5)=0 behavior
% silently reused the training set as validation when RunsPerConfig=5.
nTrain = max(1, floor(double(trainFrac) * n));
nVal = max(1, round(double(valFrac) * n));
if nTrain + nVal >= n
    nTrain = max(1, n - 2);
    nVal = 1;
end
nTest = n - nTrain - nVal;
if nTest < 1
    nTest = 1;
    nTrain = max(1, n - nVal - nTest);
end
trainIdx = idx(1:nTrain);
valIdx = idx(nTrain + 1:nTrain + nVal);
testIdx = idx(nTrain + nVal + 1:end);
end

function merged = local_merge_files(files, nominalOp, Ts, opts)
merged = [];
for i = 1:numel(files)
    T = readtable(files(i));
    if height(T) < 3, continue; end
    T = local_fill_aliases(T);
    [baseline, meta] = local_run_baseline(T, nominalOp, opts);
    if logical(opts.RequirePreExcitationBaseline) && ~meta.has_pre_excitation
        warning("AirdropX:AutoMPC:MissingBaseline", "Skipping %s: no pre-excitation baseline.", files(i));
        continue;
    end
    if logical(opts.RejectBadBaseline) && ~meta.pass
        warning("AirdropX:AutoMPC:BadBaseline", "Skipping %s: baseline gate failed.", files(i));
        continue;
    end

    mask = local_id_mask(T, meta.excitation_start_s, nominalOp.config_id, opts);
    T = T(mask,:);
    if logical(opts.ResampleToTs)
        T = local_resample_table(T, Ts);
    end
    T = local_prepare_samples(T, opts);
    if height(T) < 3, continue; end

    Udev = [ ...
        double(T.elevator_cmd) - baseline.elevator_cmd, ...
        double(T.throttle_cmd) - baseline.throttle_cmd];
    Ydev = [ ...
        double(T.altitude_m) - baseline.altitude_m, ...
        double(T.airspeed_mps) - baseline.airspeed_mps, ...
        double(T.pitch_deg) - baseline.pitch_deg, ...
        double(T.vz_up_mps) - baseline.vz_up_mps, ...
        double(T.q_dps) - baseline.q_dps];

    valid = all(isfinite(Udev),2) & all(isfinite(Ydev),2);
    Udev = Udev(valid,:);
    Ydev = Ydev(valid,:);
    if size(Udev,1) < 3, continue; end

    runName = "id_run";
    if ismember("run_id", string(T.Properties.VariableNames)) && height(T) > 0
        runName = string(T.run_id(1));
    end
    d = iddata(Ydev, Udev, double(Ts), "Name", runName);
    d.OutputName = {'altitude_dev_m', 'airspeed_dev_mps', 'pitch_dev_deg', 'vz_dev_mps', 'q_dev_dps'};
    d.InputName = {'elevator_dev', 'throttle_dev'};
    if isempty(merged)
        merged = d;
    else
        merged = merge(merged, d);
    end
end
end

function T2 = local_resample_table(T, Ts)
if isempty(T) || height(T) < 2 || ~ismember("time_s", string(T.Properties.VariableNames))
    T2 = T; return;
end
T = sortrows(T, "time_s");
t = double(T.time_s);
[t, ia] = unique(t, "stable");
T = T(ia,:);
validT = isfinite(t);
t = t(validT);
T = T(validT,:);
if numel(t) < 2
    T2 = T; return;
end
N = floor((t(end) - t(1)) / double(Ts));
tq = t(1) + (0:N).' * double(Ts);
if numel(tq) < 2
    T2 = T; return;
end

T2 = table(tq, 'VariableNames', {'time_s'});
vars = string(T.Properties.VariableNames);
stateVars = ["altitude_m","airspeed_mps","pitch_deg","vz_up_mps","q_dps", ...
    "mass_kg","cg_x_m","pos_n_m","pos_e_m","heading_deg","wind_n_mps","wind_e_mps"];
inputVars = ["elevator_cmd","throttle_cmd","elevator_cmd_actual","throttle_cmd_actual", ...
    "elevator_excitation","throttle_excitation","requested_elevator_cmd","requested_throttle_cmd", ...
    "requested_elevator_trim","requested_throttle_trim","requested_pitch_trim_deg", ...
    "trim_altitude_m","trim_airspeed_mps","trim_pitch_deg","trim_elevator_cmd","trim_throttle_cmd", ...
    "drop_count","config_id"];
for name = stateVars
    if ismember(name, vars)
        T2.(char(name)) = interp1(t, double(T.(char(name))), tq, "linear", "extrap");
    end
end
for name = inputVars
    if ismember(name, vars)
        T2.(char(name)) = interp1(t, double(T.(char(name))), tq, "previous", "extrap");
    end
end
if ismember("run_id", vars)
    T2.run_id = repmat(string(T.run_id(1)), height(T2), 1);
end
% Ensure aliases exist after resampling.
T2 = local_fill_aliases(T2);
end

function T = local_prepare_samples(T, opts)
stride = max(1, round(double(opts.SampleStride)));
if stride > 1
    T = T(1:stride:end, :);
end
maxSamples = round(double(opts.MaxSamplesPerRun));
if isfinite(maxSamples) && maxSamples > 0 && height(T) > maxSamples
    T = T(1:maxSamples, :);
end
end

function T = local_fill_aliases(T)
vars = string(T.Properties.VariableNames);
if ismember("elevator_cmd_actual", vars)
    T.elevator_cmd = T.elevator_cmd_actual;
elseif ~ismember("elevator_cmd", vars)
    if ismember("elevator_delta", vars)
        T.elevator_cmd = T.elevator_delta;
    elseif ismember("elevator_cmd_norm", vars)
        T.elevator_cmd = T.elevator_cmd_norm;
    else
        T.elevator_cmd = NaN(height(T), 1);
    end
end
vars = string(T.Properties.VariableNames);
if ismember("throttle_cmd_actual", vars)
    T.throttle_cmd = T.throttle_cmd_actual;
elseif ~ismember("throttle_cmd", vars)
    if ismember("throttle_norm", vars)
        T.throttle_cmd = T.throttle_norm;
    else
        T.throttle_cmd = NaN(height(T), 1);
    end
end
vars = string(T.Properties.VariableNames);
if ~ismember("q_dps", vars) && ismember("pitch_deg", vars) && ismember("time_s", vars)
    T.q_dps = gradient(double(T.pitch_deg), double(T.time_s));
end
end

function value = local_median(x)
x = double(x(:)); x = x(isfinite(x));
if isempty(x), value = NaN; else, value = median(x); end
end

function value = local_rms(x)
x = double(x(:)); x = x(isfinite(x));
if isempty(x), value = NaN; else, value = sqrt(mean(x.^2)); end
end

function value = local_slope(t, y)
t = double(t(:)); y = double(y(:));
mask = isfinite(t) & isfinite(y);
if nnz(mask) < 3
    value = NaN; return;
end
p = polyfit(t(mask) - t(find(mask,1,"first")), y(mask), 1);
value = p(1);
end

function value = local_struct_field(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isfinite(double(s.(name)))
    value = double(s.(name));
else
    value = double(fallback);
end
end

function opts = local_options(varargin)
opts.InputRoot = "";
opts.CsvFiles = strings(0, 1);
opts.OutputMat = "";
opts.TrimBank = [];
opts.Ts = 0.1;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.TrainFraction = 0.70;
opts.ValidationFraction = 0.15;
opts.SampleStride = 1;
opts.MaxSamplesPerRun = Inf;
opts.MaxTrainRunsPerConfig = Inf;
opts.MaxValidationRunsPerConfig = Inf;
opts.MaxTestRunsPerConfig = Inf;
% Verified trims should remain authoritative for MPC nominal values.  Set this
% true only for legacy datasets that have no trusted trim bank.
opts.EstimateOperatingPointFromCsv = false;
opts.UseVerifiedTrimForNominal = true;
% Backward-compatible legacy option; v11 always centers every experiment on
% its measured pre-excitation baseline rather than the full-run altitude median.
opts.CenterAltitudePerRun = false;
opts.OperatingPointMinSamples = 5;
opts.ResampleToTs = true;
opts.RejectBadBaseline = true;
opts.RequirePreExcitationBaseline = true;
opts.MinAcceptedRunsPerConfig = 3;
opts.BaselineDurationS = 4.0;
opts.BaselineMinSamples = 20;
opts.BaselineMaxAbsVzMps = 0.30;
opts.BaselineMaxAbsQDps = 0.30;
opts.BaselineMaxAirspeedErrorMps = 1.0;
opts.BaselineMaxPitchErrorDeg = 1.25;
opts.BaselineMaxHeightSlopeMps = 0.30;
opts.BaselineMaxElevatorError = 0.015;
opts.BaselineMaxThrottleError = 0.025;
opts.ExcitationZeroTolerance = 1e-6;
opts.OperatingPointMinAltitudeM = 5.0;
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
end
