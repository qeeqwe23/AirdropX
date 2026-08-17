function result = airdropx_auto_tune_mpc(varargin)
%AIRDROPX_AUTO_TUNE_MPC Tune MATLAB mpc() weights/horizons with bayesopt.

opts = local_options(varargin{:});
if isempty(opts.EvaluationFcn)
    error("AirdropX:AutoMPC:MissingEvaluationFcn", ...
        "Pass EvaluationFcn. bayesopt candidates must be tested on real JSBSim closed-loop runs.");
end
if exist("bayesopt", "file") ~= 2
    error("Statistics and Machine Learning Toolbox bayesopt is required for auto MPC tuning.");
end

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    outputRoot = string(fullfile(tempdir, "airdropx_auto_mpc_tune_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss"))));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

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

objective = @(x) airdropx_auto_mpc_objective(x, ...
    "Identified", opts.Identified, "IdentifiedMat", opts.IdentifiedMat, ...
    "OutputRoot", fullfile(outputRoot, "latest_candidate"), ...
    "EvaluationFcn", opts.EvaluationFcn, "EvaluationCases", opts.EvaluationCases, ...
    "PitchMinDeg", opts.PitchMinDeg, "PitchMaxDeg", opts.PitchMaxDeg);

bo = bayesopt(objective, vars, ...
    "MaxObjectiveEvaluations", double(opts.MaxObjectiveEvaluations), ...
    "UseParallel", logical(opts.UseParallel), ...
    "IsObjectiveDeterministic", logical(opts.IsObjectiveDeterministic), ...
    "Verbose", double(opts.Verbose));

best = bestPoint(bo);
bestMat = fullfile(outputRoot, "airdropx_learned_mpc_best.mat");
bestOptions = local_best_to_build_options(best);
airdropx_auto_build_mpc_bank("Identified", opts.Identified, "IdentifiedMat", opts.IdentifiedMat, ...
    "OutputMat", bestMat, "PredictionHorizon", bestOptions.Np, "ControlHorizon", bestOptions.Nc, ...
    "OutputWeights", bestOptions.OutputWeights, "MVWeights", bestOptions.MVWeights, ...
    "MVRateWeights", bestOptions.MVRateWeights, "PitchMinDeg", opts.PitchMinDeg, "PitchMaxDeg", opts.PitchMaxDeg);

result = struct();
result.output_root = outputRoot;
result.optimization = bo;
result.best_point = best;
result.best_mpc_mat = string(bestMat);
result.min_objective = bo.MinObjective;
save(fullfile(outputRoot, "auto_mpc_tuning.mat"), "result");
end

function build = local_best_to_build_options(best)
s = table2struct(best, "ToScalar", true);
build = struct();
build.Np = double(s.Np);
build.Nc = min(double(s.Nc), build.Np);
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
opts.MaxObjectiveEvaluations = 50;
opts.UseParallel = false;
opts.IsObjectiveDeterministic = false;
opts.Verbose = 1;
opts.NpRange = [12 30];
opts.NcRange = [3 10];
opts.LogWhRange = [-1 2];
opts.LogWvRange = [-1 2];
opts.LogWpitchRange = [-3 0];
opts.LogWvzRange = [-2 1];
opts.LogWqRange = [-2 1];
opts.LogWmvERange = [-2 1];
opts.LogWmvTRange = [-2 1];
opts.LogWduERange = [-1 2];
opts.LogWduTRange = [-1 2];
opts.PitchMinDeg = -10.0;
opts.PitchMaxDeg = 20.0;
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
