function report = airdropx_auto_validate_models(varargin)
%AIRDROPX_AUTO_VALIDATE_MODELS Validate learned plants on held-out runs.
%
% v11 adds per-output fit columns so a poor altitude/vz/q channel cannot hide
% behind a single mean value.

opts = local_options(varargin{:});
learned = opts.Identified;
if isempty(learned)
    S = load(opts.IdentifiedMat);
    if isfield(S, "result")
        learned = S.result;
    else
        error("IdentifiedMat must contain variable result.");
    end
end
if ~isfield(learned, "data")
    error("Identified result does not contain iddata splits.");
end

steps = double(opts.PredictionSteps(:)).';
rows = table();
for cfgId = 0:4
    sys = learned.models{cfgId + 1};
    if isempty(sys), continue; end
    for split = ["validation", "test"]
        d = learned.data.by_config(cfgId + 1).(split);
        if isempty(d), continue; end
        for step = steps
            [fitMean, fitMin, fitByOutput, score, status, message] = local_compare_score(d, sys, step);
            fitOut = NaN(1,5);
            fitOut(1:min(5,numel(fitByOutput))) = fitByOutput(1:min(5,numel(fitByOutput)));
            rows = [rows; table(cfgId, split, step, fitMean, fitMin, ...
                fitOut(1), fitOut(2), fitOut(3), fitOut(4), fitOut(5), ...
                score, status, message, ...
                'VariableNames', {'config_id', 'split', 'prediction_steps', ...
                'fit_mean_pct', 'fit_min_pct', 'fit_altitude_pct', 'fit_airspeed_pct', ...
                'fit_pitch_pct', 'fit_vz_pct', 'fit_q_pct', ...
                'score', 'status', 'message'})]; %#ok<AGROW>
        end
    end
end

report = struct();
report.table = rows;
report.identified = learned;
if strlength(string(opts.OutputFile)) > 0
    writetable(rows, opts.OutputFile);
end
end

function [fitMean, fitMin, fitByOutput, score, status, message] = local_compare_score(d, sys, step)
fitMean = NaN;
fitMin = NaN;
fitByOutput = NaN(1,5);
score = Inf;
status = "failed";
message = "";
try
    [~, fit] = compare(d, sys, step);
    M = local_fit_matrix(fit);
    values = M(isfinite(M));
    if isempty(values)
        message = "No finite fit values.";
        return;
    end
    fitMean = mean(values, "omitnan");
    fitMin = min(values, [], "omitnan");
    fitByOutput = mean(M, 1, "omitnan");
    score = 100.0 - fitMean;
    status = "ok";
catch ME
    message = string(ME.message);
end
end

function M = local_fit_matrix(fit)
if iscell(fit)
    nExp = numel(fit);
    nOut = 0;
    for i = 1:nExp
        nOut = max(nOut, numel(fit{i}));
    end
    M = NaN(nExp, max(1,nOut));
    for i = 1:nExp
        v = double(fit{i}(:)).';
        M(i,1:numel(v)) = v;
    end
else
    A = double(fit);
    if isvector(A)
        M = reshape(A, 1, []);
    else
        M = A;
    end
end
end

function opts = local_options(varargin)
opts.Identified = [];
opts.IdentifiedMat = "";
opts.OutputFile = "";
opts.PredictionSteps = [5 10 20 25];
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
