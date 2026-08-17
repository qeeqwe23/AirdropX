function result = airdropx_auto_tune_closed_loop_bridge(varargin)
%AIRDROPX_AUTO_TUNE_CLOSED_LOOP_BRIDGE Tune auto-MPC closed-loop bridge parameters.
%
% This tunes initial conditions and the small JSBSim adapter/protection layer
% around the learned MATLAB mpc() bank. It does not retrain the identified
% plants; each evaluation runs the real JSBSim closed loop and is scored with
% a transient-aware objective.

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir);
addpath(paths.mpcDir);
addpath(paths.autoDir);
addpath(paths.sfuncDir);

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    outputRoot = string(fullfile(paths.matlabDir, "results", ...
        "mpc_auto_closed_loop_tune_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss"))));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

rng(double(opts.Seed));
candidates = local_candidates(opts);
rows = table();
bestScore = Inf;
best = struct();

for i = 1:numel(candidates)
    c = candidates(i);
    evalRoot = fullfile(outputRoot, sprintf("eval_%03d", i));
    fprintf("Auto closed-loop eval %d/%d\n", i, numel(candidates));
    local_assign_candidate(c);
    try
        simResult = airdropx_auto_run_closed_loop( ...
            "MpcBankMat", opts.MpcBankMat, ...
            "OutputRoot", evalRoot, ...
            "StopTimeS", opts.StopTimeS, ...
            "CaseId", sprintf("auto_bridge_eval_%03d", i), ...
            "InitialAltitudeM", c.InitialAltitudeM, ...
            "InitialAirspeedMps", c.InitialAirspeedMps, ...
            "InitialPitchDeg", c.InitialPitchDeg, ...
            "InitialFlightPathDeg", c.InitialFlightPathDeg, ...
            "TargetAltitudeM", opts.TargetAltitudeM, ...
            "TargetAirspeedMps", opts.TargetAirspeedMps, ...
            "TargetPitchDeg", c.TargetPitchDeg, ...
            "FixedDropStartS", opts.FixedDropStartS, ...
            "FixedDropIntervalS", opts.FixedDropIntervalS, ...
            "FixedDropTotal", opts.FixedDropTotal);
        scoreResult = airdropx_auto_score_closed_loop(simResult.timeseries, ...
            "StartTimeS", opts.ScoreStartTimeS, ...
            "TargetAltitudeM", opts.TargetAltitudeM, ...
            "TargetAirspeedMps", opts.TargetAirspeedMps);
        score = scoreResult.score;
        metrics = scoreResult.metrics;
        status = "ok";
        message = "";
        csvPath = simResult.timeseries_csv;
    catch ME
        score = Inf;
        metrics = table();
        status = "failed";
        message = string(ME.message);
        csvPath = "";
        warning("AirdropX:AutoMPC:TuneEvalFailed", "Eval %d failed: %s", i, ME.message);
    end

    row = local_row(i, c, score, status, message, csvPath, metrics);
    rows = [rows; row]; %#ok<AGROW>
    writetable(rows, fullfile(outputRoot, "closed_loop_tuning_results.csv"));

    if score < bestScore
        bestScore = score;
        best = c;
        best.eval_index = i;
        best.timeseries_csv = string(csvPath);
        best.metrics = metrics;
        save(fullfile(outputRoot, "best_closed_loop_bridge.mat"), "best", "rows", "opts");
        fprintf("  New best score %.4g at eval %d\n", bestScore, i);
    else
        fprintf("  Score %.4g\n", score);
    end
end

result = struct();
result.output_root = outputRoot;
result.table = rows;
result.best = best;
result.best_score = bestScore;
save(fullfile(outputRoot, "closed_loop_tuning_result.mat"), "result", "rows", "opts");
end

function local_assign_candidate(c)
assignin("base", "airdropx_auto_elevator_sign", double(c.ElevatorSign));
assignin("base", "airdropx_auto_elevator_safety_gain", double(c.ElevatorSafetyGain));
assignin("base", "airdropx_auto_elevator_sink_gain", double(c.ElevatorSinkGain));
assignin("base", "airdropx_auto_elevator_safety_max", double(c.ElevatorSafetyMax));
assignin("base", "airdropx_auto_throttle_safety_gain", double(c.ThrottleSafetyGain));
assignin("base", "airdropx_auto_throttle_sink_gain", double(c.ThrottleSinkGain));
assignin("base", "airdropx_auto_throttle_safety_max", double(c.ThrottleSafetyMax));
assignin("base", "airdropx_auto_pitch_rate_filter_tau_s", double(c.PitchRateTauS));
end

function candidates = local_candidates(opts)
seeds = local_seed_candidates(opts);
nRandom = max(0, double(opts.MaxEvaluations) - numel(seeds));
randoms = repmat(local_empty_candidate(), nRandom, 1);
for i = 1:nRandom
    randoms(i).InitialAltitudeM = local_rand_range(opts.InitialAltitudeRange);
    randoms(i).InitialAirspeedMps = local_rand_range(opts.InitialAirspeedRange);
    randoms(i).InitialPitchDeg = local_rand_range(opts.InitialPitchRange);
    randoms(i).InitialFlightPathDeg = local_rand_range(opts.InitialFlightPathRange);
    randoms(i).TargetPitchDeg = local_rand_range(opts.TargetPitchRange);
    randoms(i).ElevatorSign = local_rand_choice([-1, 1]);
    randoms(i).ElevatorSafetyGain = local_rand_range(opts.ElevatorSafetyGainRange);
    randoms(i).ElevatorSinkGain = local_rand_range(opts.ElevatorSinkGainRange);
    randoms(i).ElevatorSafetyMax = local_rand_range(opts.ElevatorSafetyMaxRange);
    randoms(i).ThrottleSafetyGain = local_rand_range(opts.ThrottleSafetyGainRange);
    randoms(i).ThrottleSinkGain = local_rand_range(opts.ThrottleSinkGainRange);
    randoms(i).ThrottleSafetyMax = local_rand_range(opts.ThrottleSafetyMaxRange);
    randoms(i).PitchRateTauS = local_rand_range(opts.PitchRateTauRange);
end
candidates = [seeds(:); randoms(:)];
if numel(candidates) > double(opts.MaxEvaluations)
    candidates = candidates(1:double(opts.MaxEvaluations));
end
end

function candidates = local_seed_candidates(opts)
base = local_empty_candidate();
base.InitialAltitudeM = 28;
base.InitialAirspeedMps = 50;
base.InitialPitchDeg = 4;
base.InitialFlightPathDeg = 0;
base.TargetPitchDeg = 3;
base.ElevatorSign = 1;
base.ElevatorSafetyGain = 0.025;
base.ElevatorSinkGain = 0.050;
base.ElevatorSafetyMax = 0.18;
base.ThrottleSafetyGain = 0.040;
base.ThrottleSinkGain = 0.080;
base.ThrottleSafetyMax = 0.48;
base.PitchRateTauS = 0.35;

candidates = repmat(base, 6, 1);
candidates(2).InitialAltitudeM = 35; candidates(2).TargetPitchDeg = 2; candidates(2).ElevatorSafetyMax = 0.12;
candidates(3).InitialAltitudeM = 30; candidates(3).InitialPitchDeg = 6; candidates(3).TargetPitchDeg = 5; candidates(3).ElevatorSign = -1;
candidates(4).InitialAltitudeM = 32; candidates(4).TargetPitchDeg = 0; candidates(4).ElevatorSafetyGain = 0.010; candidates(4).ElevatorSafetyMax = 0.10;
candidates(5).InitialAltitudeM = 26; candidates(5).TargetPitchDeg = 6; candidates(5).ThrottleSafetyMax = 0.35;
candidates(6).InitialAltitudeM = 40; candidates(6).InitialPitchDeg = 2; candidates(6).TargetPitchDeg = 1; candidates(6).ElevatorSafetyMax = 0.08;

for i = 1:numel(candidates)
    candidates(i).InitialAltitudeM = min(max(candidates(i).InitialAltitudeM, opts.InitialAltitudeRange(1)), opts.InitialAltitudeRange(2));
end
end

function c = local_empty_candidate()
c = struct("InitialAltitudeM", NaN, "InitialAirspeedMps", NaN, "InitialPitchDeg", NaN, ...
    "InitialFlightPathDeg", NaN, "TargetPitchDeg", NaN, "ElevatorSign", NaN, ...
    "ElevatorSafetyGain", NaN, "ElevatorSinkGain", NaN, "ElevatorSafetyMax", NaN, ...
    "ThrottleSafetyGain", NaN, "ThrottleSinkGain", NaN, "ThrottleSafetyMax", NaN, ...
    "PitchRateTauS", NaN);
end

function row = local_row(idx, c, score, status, message, csvPath, metrics)
row = struct2table(c, "AsArray", true);
row = addvars(row, double(idx), double(score), string(status), string(message), string(csvPath), ...
    'Before', 1, 'NewVariableNames', {'eval_index','score','status','message','timeseries_csv'});
if ~isempty(metrics)
    metricNames = string(metrics.Properties.VariableNames);
    for k = 1:numel(metricNames)
        row.(char(metricNames(k))) = metrics.(char(metricNames(k)))(1);
    end
end
end

function value = local_rand_range(bounds)
value = double(bounds(1)) + rand() * (double(bounds(2)) - double(bounds(1)));
end

function value = local_rand_choice(values)
value = values(randi(numel(values)));
end

function paths = local_paths(projectRoot)
projectRoot = string(projectRoot);
if strlength(projectRoot) == 0
    thisDir = fileparts(mfilename("fullpath"));
    matlabDir = fileparts(thisDir);
    projectRoot = string(fileparts(matlabDir));
else
    matlabDir = fullfile(projectRoot, "matlab");
end
paths.projectRoot = char(projectRoot);
paths.matlabDir = char(matlabDir);
paths.mpcDir = char(fullfile(matlabDir, "mpc"));
paths.autoDir = char(fullfile(matlabDir, "mpc_auto"));
paths.sfuncDir = char(fullfile(matlabDir, "sfunc_jsbsim"));
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.OutputRoot = "";
opts.MpcBankMat = "matlab/results/mpc_auto_train_final_allruns/airdropx_learned_mpc.mat";
opts.MaxEvaluations = 10;
opts.Seed = 20260813;
opts.StopTimeS = 24.0;
opts.ScoreStartTimeS = 10.0;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.FixedDropStartS = 10.0;
opts.FixedDropIntervalS = 0.2;
opts.FixedDropTotal = 4.0;
opts.InitialAltitudeRange = [22.0 42.0];
opts.InitialAirspeedRange = [48.0 54.0];
opts.InitialPitchRange = [-1.0 8.0];
opts.InitialFlightPathRange = [-1.0 1.0];
opts.TargetPitchRange = [-2.0 8.0];
opts.ElevatorSafetyGainRange = [0.0 0.070];
opts.ElevatorSinkGainRange = [0.0 0.130];
opts.ElevatorSafetyMaxRange = [0.03 0.35];
opts.ThrottleSafetyGainRange = [0.0 0.070];
opts.ThrottleSinkGainRange = [0.0 0.140];
opts.ThrottleSafetyMaxRange = [0.10 0.50];
opts.PitchRateTauRange = [0.15 0.80];
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
end
