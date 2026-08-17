function result = airdropx_auto_train_all(varargin)
%AIRDROPX_AUTO_TRAIN_ALL Orchestrate the separate R2026a auto-MPC route.

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir);
addpath(paths.mpcDir);
addpath(paths.autoDir);

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    outputRoot = string(fullfile(paths.matlabDir, "results", "mpc_auto_train_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss"))));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

if opts.DoFindTrim
    trimResult = airdropx_auto_find_trim("ProjectRoot", paths.projectRoot, ...
        "ConfigIds", opts.ConfigIds, "OutputMat", fullfile(outputRoot, "auto_trim_bank.mat"), ...
        "WorkRoot", fullfile(outputRoot, "trim_search"), ...
        "ReuseVerifiedTrim", opts.ReuseVerifiedTrim, ...
        "ReuseFailedAsWarmStart", opts.ReuseFailedAsWarmStart, ...
        "PreviousTrimMat", opts.PreviousTrimMat, ...
        "CheckpointMat", opts.TrimCheckpointMat, ...
        "TargetAltitudeM", opts.TargetAltitudeM, "TargetAirspeedMps", opts.TargetAirspeedMps, ...
        "SearchAltitudeM", opts.TrimSearchAltitudeM, ...
        "StopTimeS", opts.TrimStopTimeS, "RecordStartS", opts.TrimRecordStartS, ...
        "MaxObjectiveEvaluations", opts.TrimMaxObjectiveEvaluations, "UseParallel", opts.TrimUseParallel);
    trimBank = trimResult.trim_bank;
else
    trimResult = [];
    trimBank = opts.TrimBank;
    if isempty(trimBank)
        trimBank = airdropx_auto_default_trim_bank("TargetAltitudeM", opts.TargetAltitudeM, "TargetAirspeedMps", opts.TargetAirspeedMps);
    end
end

if opts.DoGenerateData
    dataRun = airdropx_auto_generate_data("ProjectRoot", paths.projectRoot, ...
        "OutputRoot", fullfile(outputRoot, "data"), "TrimBank", trimBank, ...
        "ConfigIds", opts.ConfigIds, "RunsPerConfig", opts.RunsPerConfig, ...
        "StopTimeS", opts.StopTimeS, "RecordStartS", opts.RecordStartS, ...
        "IdentificationAltitudeM", opts.IdentificationAltitudeM, ...
        "TargetAltitudeM", opts.TargetAltitudeM, "TargetAirspeedMps", opts.TargetAirspeedMps);
    dataRoot = dataRun.output_root;
else
    dataRun = [];
    dataRoot = string(opts.DataRoot);
end

if strlength(dataRoot) == 0
    result = struct("output_root", outputRoot, "trim_result", trimResult, "trim_bank", trimBank, ...
        "message", "No data root provided. Set DoGenerateData=true or pass DataRoot to continue identification.");
    fprintf("%s\n", result.message);
    return;
end

iddataMat = fullfile(outputRoot, "auto_iddata.mat");
iddataResult = airdropx_auto_build_iddata("InputRoot", dataRoot, "TrimBank", trimBank, "OutputMat", iddataMat, ...
    "Ts", opts.IdentificationTs, ...
    "TargetAltitudeM", opts.TargetAltitudeM, "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "EstimateOperatingPointFromCsv", opts.EstimateOperatingPointFromCsv, ...
    "UseVerifiedTrimForNominal", true, ...
    "SampleStride", opts.SampleStride, "MaxSamplesPerRun", opts.MaxSamplesPerRun, ...
    "MaxTrainRunsPerConfig", opts.MaxTrainRunsPerConfig, ...
    "MaxValidationRunsPerConfig", opts.MaxValidationRunsPerConfig, ...
    "MaxTestRunsPerConfig", opts.MaxTestRunsPerConfig, ...
    "TrainFraction", opts.TrainFraction, "ValidationFraction", opts.ValidationFraction);
identifiedMat = fullfile(outputRoot, "auto_identified_plants.mat");
identified = airdropx_auto_identify("Data", iddataResult, "OutputMat", identifiedMat, "Orders", opts.Orders, ...
    "RefineWithSsest", opts.RefineWithSsest, "SsestMaxIterations", opts.SsestMaxIterations, ...
    "Focus", opts.IdentificationFocus, "EnforceStability", opts.EnforceStability, "N4Weight", opts.N4Weight);
validationFile = fullfile(outputRoot, "auto_model_validation.csv");
validation = airdropx_auto_validate_models("Identified", identified, "OutputFile", validationFile);
if logical(opts.RequireModelValidationPass)
    local_assert_model_quality(validation.table, opts);
end

mpcBankMat = fullfile(outputRoot, "airdropx_learned_mpc.mat");
mpcBank = airdropx_auto_build_mpc_bank("Identified", identified, "OutputMat", mpcBankMat, ...
    "PredictionHorizon", opts.PredictionHorizon, "ControlHorizon", opts.ControlHorizon);

if opts.DoTuneMpc
    tuning = airdropx_auto_tune_mpc("Identified", identified, ...
        "OutputRoot", fullfile(outputRoot, "tuning"), ...
        "EvaluationFcn", opts.EvaluationFcn, "EvaluationCases", opts.EvaluationCases, ...
        "MaxObjectiveEvaluations", opts.MaxObjectiveEvaluations, ...
        "UseParallel", opts.UseParallel, "IsObjectiveDeterministic", opts.IsObjectiveDeterministic);
else
    tuning = [];
end

if opts.DoFinalTest
    if ~isempty(tuning)
        finalMpcMat = tuning.best_mpc_mat;
    else
        finalMpcMat = string(mpcBankMat);
    end
    finalTest = airdropx_auto_final_test("MpcBankMat", finalMpcMat, ...
        "TimeseriesCsv", opts.FinalTimeseriesCsv, ...
        "SimulationFcn", opts.EvaluationFcn, "Cases", opts.FinalTestCases, ...
        "OutputFile", fullfile(outputRoot, "auto_final_test.csv"));
else
    finalTest = [];
end

result = struct();
result.output_root = outputRoot;
result.trim_result = trimResult;
result.trim_bank = trimBank;
result.data_run = dataRun;
result.iddata_mat = string(iddataMat);
result.identified_mat = string(identifiedMat);
result.validation_csv = string(validationFile);
result.learned_mpc_mat = string(mpcBankMat);
result.iddata = iddataResult;
result.identified = identified;
result.validation = validation;
result.mpc_bank = mpcBank;
result.tuning = tuning;
result.final_test = finalTest;
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
end

function local_assert_model_quality(T, opts)
if isempty(T), error("AirdropX:AutoMPC:ModelValidationFailed", "Validation report is empty."); end
step = double(opts.ModelGatePredictionStep);
cfgs = unique(double(T.config_id));
failed = strings(0,1);
for cfgId = cfgs(:).'
    v = T(double(T.config_id)==cfgId & string(T.split)=="validation" & double(T.prediction_steps)==step, :);
    q = T(double(T.config_id)==cfgId & string(T.split)=="test" & double(T.prediction_steps)==step, :);
    if isempty(v) || isempty(q) || ~isfinite(v.fit_mean_pct(1)) || ~isfinite(q.fit_mean_pct(1)) || ...
            v.fit_mean_pct(1) < double(opts.MinValidationFitPct) || q.fit_mean_pct(1) < double(opts.MinTestFitPct)
        if isempty(v), vf=NaN; else, vf=v.fit_mean_pct(1); end
        if isempty(q), tf=NaN; else, tf=q.fit_mean_pct(1); end
        failed(end+1,1) = sprintf("cfg%d val=%.1f%% test=%.1f%%", cfgId, vf, tf); %#ok<AGROW>
    end
end
if ~isempty(failed)
    error("AirdropX:AutoMPC:ModelValidationFailed", ...
        "Plant validation gate failed at %d-step: %s. MPC build/tuning stopped.", ...
        round(step), strjoin(failed, "; "));
end
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.OutputRoot = "";
opts.DataRoot = "";
opts.TrimBank = [];
opts.DoFindTrim = false;
opts.DoGenerateData = false;
opts.DoTuneMpc = false;
opts.DoFinalTest = false;
opts.ConfigIds = (0:4).';
opts.RunsPerConfig = 5;
opts.StopTimeS = 30.0;
opts.RecordStartS = 8.0;
opts.SampleStride = 1;
opts.IdentificationTs = 0.1;
opts.MaxSamplesPerRun = Inf;
opts.MaxTrainRunsPerConfig = Inf;
opts.MaxValidationRunsPerConfig = Inf;
opts.MaxTestRunsPerConfig = Inf;
opts.TrainFraction = 0.70;
opts.ValidationFraction = 0.15;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.TrimSearchAltitudeM = 200.0;
opts.TrimStopTimeS = 22.0;
opts.TrimRecordStartS = 10.0;
opts.TrimMaxObjectiveEvaluations = 60;
opts.TrimUseParallel = false;
opts.ReuseVerifiedTrim = true;
opts.ReuseFailedAsWarmStart = true;
opts.PreviousTrimMat = "";
opts.TrimCheckpointMat = "";
opts.IdentificationAltitudeM = 200.0;
opts.EstimateOperatingPointFromCsv = false;
opts.Orders = 3:10;
opts.RefineWithSsest = false;
opts.IdentificationFocus = "simulation";
opts.EnforceStability = true;
opts.N4Weight = "CVA";
opts.SsestMaxIterations = 20;
opts.RequireModelValidationPass = true;
opts.ModelGatePredictionStep = 5;
opts.MinValidationFitPct = 10.0;
opts.MinTestFitPct = 0.0;
opts.PredictionHorizon = 20;
opts.ControlHorizon = 6;
opts.EvaluationFcn = [];
opts.EvaluationCases = [];
opts.FinalTestCases = [];
opts.FinalTimeseriesCsv = strings(0, 1);
opts.MaxObjectiveEvaluations = 50;
opts.UseParallel = false;
opts.IsObjectiveDeterministic = false;
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
end
