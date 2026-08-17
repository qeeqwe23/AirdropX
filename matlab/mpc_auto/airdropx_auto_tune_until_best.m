function result = airdropx_auto_tune_until_best(varargin)
%AIRDROPX_AUTO_TUNE_UNTIL_BEST Repeated constrained bayesopt until specs + plateau.
%
% "Best" cannot be mathematically proven for a nonlinear JSBSim black box.
% This routine therefore stops when either:
%   1) all hard flight-performance constraints are feasible and the best
%      feasible objective has stopped improving for StallBatches batches, or
%   2) MaxTotalEvaluations is reached.
%
% Every objective evaluation runs the real JSBSim closed loop.

opts = local_options(varargin{:});
if isempty(opts.EvaluationFcn)
    opts.EvaluationFcn = @airdropx_auto_eval_case;
end
if isempty(opts.EvaluationCases)
    opts.EvaluationCases = airdropx_auto_default_eval_cases( ...
        "TargetAltitudeM", opts.TargetAltitudeM, "TargetAirspeedMps", opts.TargetAirspeedMps, ...
        "StopTimeS", opts.StopTimeS);
end
if exist("bayesopt", "file") ~= 2
    error("Statistics and Machine Learning Toolbox bayesopt is required.");
end

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    outputRoot = string(fullfile(tempdir, "airdropx_auto_until_best_" + ...
        string(datetime("now", "Format", "yyyyMMdd_HHmmss"))));
end
if ~isfolder(outputRoot), mkdir(outputRoot); end
candidateRoot = fullfile(outputRoot, "candidates");
if ~isfolder(candidateRoot), mkdir(candidateRoot); end
checkpointFile = fullfile(outputRoot, "auto_tuning_checkpoint.mat");

vars = local_variables(opts);
objective = @(x) airdropx_auto_mpc_objective(x, ...
    "Identified", opts.Identified, "IdentifiedMat", opts.IdentifiedMat, ...
    "OutputRoot", candidateRoot, ...
    "EvaluationFcn", opts.EvaluationFcn, "EvaluationCases", opts.EvaluationCases, ...
    "PitchMinDeg", opts.PitchMinDeg, "PitchMaxDeg", opts.PitchMaxDeg, ...
    "ScoreStartTimeS", opts.ScoreStartTimeS, ...
    "TargetAltitudeM", opts.TargetAltitudeM, "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "MaxSteadyAltitudeRmsM", opts.MaxSteadyAltitudeRmsM, ...
    "MaxSteadyAirspeedRmsMps", opts.MaxSteadyAirspeedRmsMps, ...
    "MaxSteadyVzRmsMps", opts.MaxSteadyVzRmsMps, ...
    "MaxSteadyQRmsDps", opts.MaxSteadyQRmsDps, ...
    "MaxPitchStdDeg", opts.MaxPitchStdDeg, ...
    "MaxPitchDriftDegps", opts.MaxPitchDriftDegps, ...
    "MinAltitudeM", opts.MinAltitudeM);

rng(double(opts.Seed));
bo = [];
if opts.ResumeFromCheckpoint && isfile(checkpointFile)
    S = load(checkpointFile, "bo");
    if isfield(S, "bo")
        bo = S.bo;
        fprintf("[AUTO-TUNE] Resuming checkpoint with %d completed evaluations.\n", bo.NumObjectiveEvaluations);
    end
end

prevBest = Inf;
stallCount = 0;
batchIndex = 0;
stopReason = "max_evaluations";

while isempty(bo) || bo.NumObjectiveEvaluations < double(opts.MaxTotalEvaluations)
    batchIndex = batchIndex + 1;
    if isempty(bo)
        nThis = min(double(opts.BatchEvaluations), double(opts.MaxTotalEvaluations));
        fprintf("[AUTO-TUNE] Starting bayesopt batch %d with %d evaluations.\n", batchIndex, nThis);
        bo = bayesopt(objective, vars, ...
            "MaxObjectiveEvaluations", nThis, ...
            "NumCoupledConstraints", 7, ...
            "AreCoupledConstraintsDeterministic", repmat(logical(opts.IsObjectiveDeterministic),1,7), ...
            "IsObjectiveDeterministic", logical(opts.IsObjectiveDeterministic), ...
            "UseParallel", logical(opts.UseParallel), ...
            "AcquisitionFunctionName", "expected-improvement-plus", ...
            "Verbose", double(opts.Verbose), "PlotFcn", []);
    else
        remain = double(opts.MaxTotalEvaluations) - double(bo.NumObjectiveEvaluations);
        if remain <= 0, break; end
        nThis = min(double(opts.BatchEvaluations), remain);
        fprintf("[AUTO-TUNE] Resuming batch %d for %d more evaluations.\n", batchIndex, nThis);
        bo = resume(bo, "MaxObjectiveEvaluations", nThis, ...
            "Verbose", double(opts.Verbose), "PlotFcn", []);
    end

    save(checkpointFile, "bo", "opts", "-v7.3");
    [idxBest, isFeasible, violation] = local_best_index(bo);
    bestScore = double(bo.ObjectiveTrace(idxBest));
    ud = bo.UserDataTrace{idxBest};
    local_write_progress(outputRoot, bo, idxBest, isFeasible, violation, batchIndex);

    if isFeasible
        if isfinite(prevBest)
            relImprovement = max(0.0, (prevBest - bestScore) / max(abs(prevBest), 1.0));
            if relImprovement < double(opts.MinRelativeImprovement)
                stallCount = stallCount + 1;
            else
                stallCount = 0;
            end
        end
        prevBest = min(prevBest, bestScore);
        fprintf("[AUTO-TUNE] feasible best=%.6g, stall=%d/%d\n", bestScore, stallCount, opts.StallBatches);
        if stallCount >= double(opts.StallBatches)
            stopReason = "specs_met_and_plateau";
            break;
        end
    else
        fprintf("[AUTO-TUNE] no feasible controller yet; aggregate constraint violation=%.6g\n", violation);
    end
end

[idxBest, isFeasible, violation] = local_best_index(bo);
bestPoint = bo.XTrace(idxBest, :);
bestUser = bo.UserDataTrace{idxBest};
bestScore = double(bo.ObjectiveTrace(idxBest));
bestMat = fullfile(outputRoot, "airdropx_learned_mpc_best.mat");
if isstruct(bestUser) && isfield(bestUser, "candidate_mat") && isfile(bestUser.candidate_mat)
    copyfile(bestUser.candidate_mat, bestMat);
else
    build = local_point_to_build(bestPoint);
    airdropx_auto_build_mpc_bank("Identified", opts.Identified, "IdentifiedMat", opts.IdentifiedMat, ...
        "OutputMat", bestMat, "PredictionHorizon", build.Np, "ControlHorizon", build.Nc, ...
        "OutputWeights", build.OutputWeights, "MVWeights", build.MVWeights, ...
        "MVRateWeights", build.MVRateWeights, "PitchMinDeg", opts.PitchMinDeg, "PitchMaxDeg", opts.PitchMaxDeg);
end

% Expose both the trim references and the actually observed steady pitch means.
Sbest = load(bestMat, "trim_bank");
trimPitch = arrayfun(@(s) double(s.pitch_deg), Sbest.trim_bank(:));
caseMetrics = table();
if isstruct(bestUser) && isfield(bestUser, "case_metrics")
    caseMetrics = bestUser.case_metrics;
    writetable(caseMetrics, fullfile(outputRoot, "best_case_metrics.csv"));
end
writetable(bestPoint, fullfile(outputRoot, "best_mpc_parameters_logspace.csv"));

result = struct();
result.output_root = outputRoot;
result.optimization = bo;
result.best_index = idxBest;
result.best_point = bestPoint;
result.best_score = bestScore;
result.best_feasible = isFeasible;
result.best_constraint_violation = violation;
result.best_user_data = bestUser;
result.best_mpc_mat = string(bestMat);
result.trim_pitch_reference_deg = trimPitch;
result.best_case_metrics = caseMetrics;
result.stop_reason = stopReason;
result.total_evaluations = bo.NumObjectiveEvaluations;
save(fullfile(outputRoot, "auto_tune_until_best_result.mat"), "result", "opts", "-v7.3");

fprintf("\n[AUTO-TUNE] finished: %s\n", stopReason);
fprintf("[AUTO-TUNE] evaluations: %d\n", bo.NumObjectiveEvaluations);
fprintf("[AUTO-TUNE] feasible: %d, best score: %.6g\n", isFeasible, bestScore);
fprintf("[AUTO-TUNE] best MPC: %s\n", bestMat);
fprintf("[AUTO-TUNE] trim pitch refs cfg0..4 = %s deg\n", mat2str(trimPitch(:).',4));
end

function vars = local_variables(opts)
vars = [
    optimizableVariable("Np", double(opts.NpRange), "Type", "integer")
    optimizableVariable("Nc", double(opts.NcRange), "Type", "integer")
    optimizableVariable("logWh", double(opts.LogWhRange))
    optimizableVariable("logWv", double(opts.LogWvRange))
    optimizableVariable("logWpitch", double(opts.LogWpitchRange))
    optimizableVariable("logWvz", double(opts.LogWvzRange))
    optimizableVariable("logWq", double(opts.LogWqRange))
    optimizableVariable("logWmvE", double(opts.LogWmvERange))
    optimizableVariable("logWmvT", double(opts.LogWmvTRange))
    optimizableVariable("logWduE", double(opts.LogWduERange))
    optimizableVariable("logWduT", double(opts.LogWduTRange))
    ];
end

function [idxBest, feasible, violation] = local_best_index(bo)
obj = double(bo.ObjectiveTrace(:));
C = double(bo.ConstraintsTrace);
finiteObj = isfinite(obj);
feasMask = finiteObj & all(C <= 0, 2);
if any(feasMask)
    ids = find(feasMask);
    [~,j] = min(obj(ids));
    idxBest = ids(j);
    feasible = true;
    violation = 0.0;
else
    V = sum(max(C,0).^2,2);
    V(~finiteObj) = Inf;
    [violation, idxBest] = min(V);
    feasible = false;
end
end

function local_write_progress(root, bo, idxBest, feasible, violation, batchIndex)
T = bo.XTrace;
T.objective = bo.ObjectiveTrace;
C = bo.ConstraintsTrace;
for k=1:size(C,2)
    T.(sprintf("constraint_%d",k)) = C(:,k);
end
T.feasible = all(C <= 0,2);
writetable(T, fullfile(root, "optimization_trace.csv"));
summary = table(batchIndex, bo.NumObjectiveEvaluations, idxBest, double(bo.ObjectiveTrace(idxBest)), ...
    logical(feasible), double(violation), ...
    'VariableNames', {'batch','evaluations','best_index','best_objective','best_feasible','constraint_violation'});
writetable(summary, fullfile(root, "latest_progress.csv"));
end

function build = local_point_to_build(best)
s = table2struct(best, "ToScalar", true);
build.Np = round(double(s.Np));
build.Nc = min(round(double(s.Nc)), build.Np);
build.OutputWeights = [10^s.logWh 10^s.logWv 10^s.logWpitch 10^s.logWvz 10^s.logWq];
build.MVWeights = [10^s.logWmvE 10^s.logWmvT];
build.MVRateWeights = [10^s.logWduE 10^s.logWduT];
end

function opts = local_options(varargin)
opts.Identified = [];
opts.IdentifiedMat = "";
opts.OutputRoot = "";
opts.EvaluationFcn = [];
opts.EvaluationCases = [];
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.StopTimeS = 30.0;
opts.ScoreStartTimeS = 10.0;
opts.BatchEvaluations = 15;
opts.MaxTotalEvaluations = 90;
opts.StallBatches = 2;
opts.MinRelativeImprovement = 0.01;
opts.ResumeFromCheckpoint = true;
opts.Seed = 20260813;
opts.UseParallel = false;
opts.IsObjectiveDeterministic = true;
opts.Verbose = 1;
opts.NpRange = [12 35];
opts.NcRange = [2 10];
opts.LogWhRange = [0 2.5];
opts.LogWvRange = [0 2.3];
% Pitch itself is free to settle; this tiny weight only prevents pathological wandering.
opts.LogWpitchRange = [-4 -1];
opts.LogWvzRange = [-0.5 2.0];
opts.LogWqRange = [-0.5 2.0];
opts.LogWmvERange = [-2 0.5];
opts.LogWmvTRange = [-2 0.5];
opts.LogWduERange = [-1 2.5];
opts.LogWduTRange = [-1 2.0];
opts.PitchMinDeg = -10.0;
opts.PitchMaxDeg = 20.0;
opts.MaxSteadyAltitudeRmsM = 1.0;
opts.MaxSteadyAirspeedRmsMps = 1.0;
opts.MaxSteadyVzRmsMps = 0.70;
opts.MaxSteadyQRmsDps = 1.0;
opts.MaxPitchStdDeg = 0.75;
opts.MaxPitchDriftDegps = 0.12;
opts.MinAltitudeM = 15.0;
if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin)
    n=string(varargin{i});
    if ~isfield(opts,n), error("Unknown option: %s",n); end
    opts.(n)=varargin{i+1};
end
if isempty(opts.Identified) && strlength(string(opts.IdentifiedMat))==0
    error("Identified or IdentifiedMat is required.");
end
end
