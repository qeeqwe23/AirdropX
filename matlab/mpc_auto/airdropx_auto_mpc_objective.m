function [score, constraints, userData] = airdropx_auto_mpc_objective(x, varargin)
%AIRDROPX_AUTO_MPC_OBJECTIVE Real-JSBSim objective with hard performance constraints.
%
% The optimizer is not asked to track a manually selected pitch angle.  Pitch
% reference comes from trim_bank inside the MPC adapter.  Here pitch is judged
% by stability: q, standard deviation and drift.
%
% Coupled constraints use the bayesopt convention: <= 0 is feasible.

opts = local_options(varargin{:});
if isempty(opts.EvaluationFcn)
    error("AirdropX:AutoMPC:MissingEvaluationFcn", ...
        "MPC tuning requires EvaluationFcn so candidates are tested on the real JSBSim closed loop.");
end

candidate = local_candidate_options(x, opts);
runRoot = string(opts.OutputRoot);
if strlength(runRoot) == 0
    runRoot = string(fullfile(tempdir, "airdropx_auto_mpc_candidates"));
end
if ~isfolder(runRoot), mkdir(runRoot); end
candidateRoot = string(tempname(runRoot));
mkdir(candidateRoot);
candidateMat = fullfile(candidateRoot, "airdropx_learned_mpc_candidate.mat");

try
    airdropx_auto_build_mpc_bank("Identified", opts.Identified, "IdentifiedMat", opts.IdentifiedMat, ...
        "OutputMat", candidateMat, "PredictionHorizon", candidate.Np, "ControlHorizon", candidate.Nc, ...
        "OutputWeights", candidate.OutputWeights, "MVWeights", candidate.MVWeights, ...
        "MVRateWeights", candidate.MVRateWeights, "PitchMinDeg", opts.PitchMinDeg, "PitchMaxDeg", opts.PitchMaxDeg);

    cases = opts.EvaluationCases;
    if isempty(cases), cases = struct("name", "nominal"); end
    rows = table();
    for i = 1:numel(cases)
        simOut = feval(opts.EvaluationFcn, candidateMat, cases(i));
        [T, csvPath] = local_timeseries_from_output(simOut);

        hardFail = local_hard_fail(T, opts);
        if hardFail.failed
            score = double(opts.HardFailScore);
            constraints = 1e6 * ones(1, 7);
            userData = struct("status", "hard_fail", "reason", hardFail.reason, ...
                "candidate_root", candidateRoot, "candidate_mat", string(candidateMat), ...
                "candidate", candidate, "case_name", local_case_name(cases(i), i), ...
                "timeseries_csv", string(csvPath), "hard_fail", hardFail);
            writetable(local_candidate_table(candidate), fullfile(candidateRoot, "candidate_parameters.csv"));
            fid = fopen(fullfile(candidateRoot, "hard_fail.txt"), "w");
            if fid >= 0
                fprintf(fid, "%s\n", char(hardFail.reason));
                fclose(fid);
            end
            return;
        end

        sr = airdropx_auto_score_closed_loop(T, ...
            "StartTimeS", opts.ScoreStartTimeS, ...
            "TargetAltitudeM", opts.TargetAltitudeM, ...
            "TargetAirspeedMps", opts.TargetAirspeedMps);
        row = sr.metrics;
        row = addvars(row, local_case_name(cases(i), i), string(csvPath), ...
            'Before', 1, 'NewVariableNames', {'case_name','timeseries_csv'});
        rows = [rows; row]; %#ok<AGROW>
    end

    caseScores = double(rows.score);
    score = mean(caseScores, "omitnan") + double(opts.WorstCaseWeight) * max(caseScores, [], "omitnan");

    worstFullH = max(rows.full_h_rms_m, [], "omitnan");
    worstFullHMax = max(rows.full_h_max_abs_m, [], "omitnan");
    worstFullV = max(rows.full_airspeed_rms_mps, [], "omitnan");
    worstFullVz = max(rows.full_vz_rms_mps, [], "omitnan");
    worstFullQ = max(rows.full_q_rms_dps, [], "omitnan");
    worstH = max(rows.steady_h_rms_m, [], "omitnan");
    worstHMax = max(rows.steady_h_max_abs_m, [], "omitnan");
    worstHDrift = max(rows.steady_h_drift_m, [], "omitnan");
    worstV = max(rows.steady_airspeed_rms_mps, [], "omitnan");
    worstVz = max(rows.steady_vz_rms_mps, [], "omitnan");
    worstQ = max(rows.steady_q_rms_dps, [], "omitnan");
    worstPitchStd = max(rows.steady_pitch_std_deg, [], "omitnan");
    worstPitchDrift = max(rows.steady_pitch_drift_degps, [], "omitnan");
    minAltitude = min(rows.min_altitude_m, [], "omitnan");

    altitudeConstraint = max([ ...
        worstFullH - double(opts.MaxFullAltitudeRmsM), ...
        worstFullHMax - double(opts.MaxFullAltitudeMaxAbsM), ...
        worstH - double(opts.MaxSteadyAltitudeRmsM), ...
        worstHMax - double(opts.MaxSteadyAltitudeMaxAbsM), ...
        worstHDrift - double(opts.MaxSteadyAltitudeDriftM)]);
    airspeedConstraint = max([ ...
        worstFullV - double(opts.MaxFullAirspeedRmsMps), ...
        worstV - double(opts.MaxSteadyAirspeedRmsMps)]);
    vzConstraint = max([ ...
        worstFullVz - double(opts.MaxFullVzRmsMps), ...
        worstVz - double(opts.MaxSteadyVzRmsMps)]);
    qConstraint = max([ ...
        worstFullQ - double(opts.MaxFullQRmsDps), ...
        worstQ - double(opts.MaxSteadyQRmsDps)]);

    constraints = [ ...
        altitudeConstraint, ...
        airspeedConstraint, ...
        vzConstraint, ...
        qConstraint, ...
        worstPitchStd - double(opts.MaxPitchStdDeg), ...
        worstPitchDrift - double(opts.MaxPitchDriftDegps), ...
        double(opts.MinAltitudeM) - minAltitude];

    userData = struct();
    userData.status = "ok";
    userData.candidate_root = candidateRoot;
    userData.candidate_mat = string(candidateMat);
    userData.candidate = candidate;
    userData.case_metrics = rows;
    userData.worst = struct("full_altitude_rms_m", worstFullH, "full_altitude_max_abs_m", worstFullHMax, ...
        "full_airspeed_rms_mps", worstFullV, "full_vz_rms_mps", worstFullVz, "full_q_rms_dps", worstFullQ, ...
        "altitude_rms_m", worstH, "altitude_max_abs_m", worstHMax, ...
        "altitude_drift_m", worstHDrift, "airspeed_rms_mps", worstV, ...
        "vz_rms_mps", worstVz, "q_rms_dps", worstQ, "pitch_std_deg", worstPitchStd, ...
        "pitch_drift_degps", worstPitchDrift, "min_altitude_m", minAltitude);
    writetable(rows, fullfile(candidateRoot, "candidate_score.csv"));
    writetable(local_candidate_table(candidate), fullfile(candidateRoot, "candidate_parameters.csv"));
catch ME
    score = realmax("double") / 1000;
    constraints = 1e6 * ones(1, 7);
    userData = struct("status", "failed", "message", string(ME.message), ...
        "candidate_root", candidateRoot, "candidate_mat", string(candidateMat), "candidate", candidate);
    warning("AirdropX:AutoMPC:CandidateFailed", "Candidate failed: %s", ME.message);
end

if ~isfinite(score), score = realmax("double") / 1000; end
constraints(~isfinite(constraints)) = 1e6;
end

function result = local_hard_fail(T, opts)
result = struct("failed", false, "reason", "", "min_altitude_m", NaN, ...
    "elevator_saturation_s", 0.0, "throttle_saturation_s", 0.0);
if isempty(T) || height(T) == 0
    result.failed = true;
    result.reason = "empty closed-loop timeseries";
    return;
end

time = local_table_column(T, ["time_s"], NaN);
h = local_table_column(T, ["altitude_m"], NaN);
finiteH = h(isfinite(h));
if isempty(finiteH)
    result.failed = true;
    result.reason = "altitude signal is missing or non-finite";
    return;
end
result.min_altitude_m = min(finiteH);
if result.min_altitude_m < double(opts.HardFloorAltitudeM)
    result.failed = true;
    result.reason = sprintf("hard floor violated: min altitude %.3f m < %.3f m", ...
        result.min_altitude_m, double(opts.HardFloorAltitudeM));
    return;
end

elevator = local_table_column(T, ["elevator_delta","elevator_cmd","elevator_cmd_norm"], NaN);
throttle = local_table_column(T, ["throttle_cmd","throttle_norm"], NaN);
result.elevator_saturation_s = local_longest_saturation(time, elevator, ...
    double(opts.ElevatorSaturationMin), double(opts.ElevatorSaturationMax), double(opts.SaturationTolerance));
result.throttle_saturation_s = local_longest_saturation(time, throttle, ...
    double(opts.ThrottleSaturationMin), double(opts.ThrottleSaturationMax), double(opts.SaturationTolerance));

if result.elevator_saturation_s >= double(opts.MaxElevatorSaturationDurationS)
    result.failed = true;
    result.reason = sprintf("elevator saturated for %.3f s (limit %.3f s)", ...
        result.elevator_saturation_s, double(opts.MaxElevatorSaturationDurationS));
    return;
end
if result.throttle_saturation_s >= double(opts.MaxThrottleSaturationDurationS)
    result.failed = true;
    result.reason = sprintf("throttle saturated for %.3f s (limit %.3f s)", ...
        result.throttle_saturation_s, double(opts.MaxThrottleSaturationDurationS));
end
end

function x = local_table_column(T, names, fallback)
for i = 1:numel(names)
    if ismember(names(i), string(T.Properties.VariableNames))
        x = double(T.(char(names(i)))(:));
        return;
    end
end
x = double(fallback) * ones(height(T), 1);
end

function duration = local_longest_saturation(time, u, lowerBound, upperBound, tol)
time = double(time(:));
u = double(u(:));
if isempty(time) || isempty(u) || numel(time) ~= numel(u) || all(~isfinite(u))
    duration = 0.0;
    return;
end
mask = isfinite(time) & isfinite(u) & (u <= lowerBound + tol | u >= upperBound - tol);
if ~any(mask)
    duration = 0.0;
    return;
end
validTime = time(isfinite(time));
dt = diff(validTime);
dt = dt(isfinite(dt) & dt > 0);
if isempty(dt)
    sampleTime = 0.1;
else
    sampleTime = median(dt);
end
edges = diff([false; mask; false]);
starts = find(edges == 1);
stops = find(edges == -1) - 1;
runSamples = stops - starts + 1;
duration = max(runSamples) * sampleTime;
end

function candidate = local_candidate_options(x, opts)
if istable(x), x = table2struct(x, "ToScalar", true); end
candidate = struct();
candidate.Np = round(local_field(x, "Np", opts.PredictionHorizon));
candidate.Nc = min(round(local_field(x, "Nc", opts.ControlHorizon)), candidate.Np);
candidate.OutputWeights = [ ...
    10 ^ local_field(x, "logWh", log10(opts.OutputWeights(1))) ...
    10 ^ local_field(x, "logWv", log10(opts.OutputWeights(2))) ...
    10 ^ local_field(x, "logWpitch", log10(opts.OutputWeights(3))) ...
    10 ^ local_field(x, "logWvz", log10(opts.OutputWeights(4))) ...
    10 ^ local_field(x, "logWq", log10(opts.OutputWeights(5)))];
candidate.MVWeights = [ ...
    10 ^ local_field(x, "logWmvE", log10(opts.MVWeights(1))) ...
    10 ^ local_field(x, "logWmvT", log10(opts.MVWeights(2)))];
candidate.MVRateWeights = [ ...
    10 ^ local_field(x, "logWduE", log10(opts.MVRateWeights(1))) ...
    10 ^ local_field(x, "logWduT", log10(opts.MVRateWeights(2)))];
end

function T = local_candidate_table(c)
T = table(c.Np, c.Nc, c.OutputWeights(1), c.OutputWeights(2), c.OutputWeights(3), ...
    c.OutputWeights(4), c.OutputWeights(5), c.MVWeights(1), c.MVWeights(2), ...
    c.MVRateWeights(1), c.MVRateWeights(2), ...
    'VariableNames', {'Np','Nc','Wh','Wv','Wpitch','Wvz','Wq','WmvE','WmvT','WduE','WduT'});
end

function value = local_field(s, name, fallback)
if isfield(s, name), value = double(s.(name)); else, value = double(fallback); end
end

function [T, csvPath] = local_timeseries_from_output(simOut)
csvPath = "";
if istable(simOut)
    T = simOut;
elseif isstring(simOut) || ischar(simOut)
    csvPath = string(simOut); T = readtable(csvPath);
elseif isstruct(simOut)
    if isfield(simOut, "timeseries") && istable(simOut.timeseries)
        T = simOut.timeseries;
        if isfield(simOut, "timeseries_csv"), csvPath = string(simOut.timeseries_csv); end
    elseif isfield(simOut, "timeseries_csv")
        csvPath = string(simOut.timeseries_csv); T = readtable(csvPath);
    elseif isfield(simOut, "csv")
        csvPath = string(simOut.csv); T = readtable(csvPath);
    else
        error("AirdropX:AutoMPC:BadSimOutput", "EvaluationFcn output has no timeseries data.");
    end
else
    error("AirdropX:AutoMPC:BadSimOutput", "Unsupported EvaluationFcn output.");
end
end

function name = local_case_name(caseDef, idx)
name = "case_" + string(idx);
if isstruct(caseDef) && isfield(caseDef, "name"), name = string(caseDef.name); end
end

function opts = local_options(varargin)
opts.Identified = [];
opts.IdentifiedMat = "";
opts.OutputRoot = "";
opts.EvaluationFcn = [];
opts.EvaluationCases = [];
opts.PredictionHorizon = 20;
opts.ControlHorizon = 6;
opts.OutputWeights = [8.0 5.0 0.01 5.0 5.0];
opts.MVWeights = [0.1 0.1];
opts.MVRateWeights = [2.0 1.0];
opts.PitchMinDeg = -10.0;
opts.PitchMaxDeg = 20.0;
opts.ScoreStartTimeS = 10.0;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.WorstCaseWeight = 0.25;
% Hard acceptance specs. bayesopt treats each as a coupled constraint.
opts.MaxFullAltitudeRmsM = 2.0;
opts.MaxFullAltitudeMaxAbsM = 4.0;
opts.MaxFullAirspeedRmsMps = 2.5;
opts.MaxFullVzRmsMps = 1.5;
opts.MaxFullQRmsDps = 2.0;
opts.MaxSteadyAltitudeRmsM = 1.0;
opts.MaxSteadyAltitudeMaxAbsM = 2.0;
opts.MaxSteadyAltitudeDriftM = 1.0;
opts.MaxSteadyAirspeedRmsMps = 1.0;
opts.MaxSteadyVzRmsMps = 0.70;
opts.MaxSteadyQRmsDps = 1.0;
opts.MaxPitchStdDeg = 0.75;
opts.MaxPitchDriftDegps = 0.12;
opts.MinAltitudeM = 15.0;
% Catastrophic candidates are excluded before normal scoring so a
% near-ground, actuator-saturated trajectory cannot look good merely because
% pitch becomes quiet after the vehicle has effectively lost control.
opts.HardFloorAltitudeM = 5.0;
opts.ElevatorSaturationMin = -0.85;
opts.ElevatorSaturationMax = 0.85;
opts.ThrottleSaturationMin = 0.0;
opts.ThrottleSaturationMax = 1.0;
opts.SaturationTolerance = 0.005;
opts.MaxElevatorSaturationDurationS = 1.0;
opts.MaxThrottleSaturationDurationS = 2.0;
opts.HardFailScore = 1e12;
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
if isempty(opts.Identified) && strlength(string(opts.IdentifiedMat)) == 0
    error("Identified or IdentifiedMat is required.");
end
end
