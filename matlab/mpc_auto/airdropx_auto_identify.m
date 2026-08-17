function result = airdropx_auto_identify(varargin)
%AIRDROPX_AUTO_IDENTIFY Identify Plant0..Plant4 with n4sid/optional ssest.
%
% v11 fixes multi-experiment validation: compare() returns fit values in cells
% for merged experiments.  The old code attempted double(fit(:)), threw, caught
% the error, and silently assigned Inf to every order.  That forced the first
% successful (usually 3rd-order) model to win regardless of validation quality.

opts = local_options(varargin{:});
if isempty(opts.Data)
    S = load(opts.DataMat);
    if isfield(S, "data")
        data = S.data;
    else
        error("DataMat must contain variable data.");
    end
else
    data = opts.Data;
end

orders = double(opts.Orders(:)).';
steps = double(opts.ValidationSteps(:)).';
models = cell(5, 1);
records = table();

for cfgId = 0:4
    dTrain = data.by_config(cfgId + 1).train;
    dVal = data.by_config(cfgId + 1).validation;
    if isempty(dTrain)
        warning("AirdropX:AutoMPC:NoTrainData", "No training data for config %d.", cfgId);
        continue;
    end

    bestScore = Inf;
    bestSys = [];
    for nx = orders
        fitMean = NaN;
        fitMin = NaN;
        try
            n4opt = n4sidOptions("Display", "off", "Focus", opts.Focus, ...
                "EnforceStability", logical(opts.EnforceStability), "N4Weight", opts.N4Weight);
            fprintf("[IDENTIFY] cfg=%d order=%d: n4sid...\n", cfgId, nx);
            tOrder = tic;
            init = n4sid(dTrain, nx, "Ts", data.Ts, n4opt);
            sys = init;

            if opts.RefineWithSsest
                fprintf("[IDENTIFY] cfg=%d order=%d: ssest refine (max %d iters)...\n", ...
                    cfgId, nx, round(double(opts.SsestMaxIterations)));
                try
                    opt = ssestOptions("Display", "off");
                    opt.SearchOptions.MaxIterations = round(double(opts.SsestMaxIterations));
                    sys = ssest(dTrain, init, opt);
                catch MEss
                    warning("AirdropX:AutoMPC:SsestRefineFailed", ...
                        "ssest refinement failed for cfg=%d order=%d: %s. Keeping n4sid model.", ...
                        cfgId, nx, MEss.message);
                    sys = init;
                end
            end
            fprintf("[IDENTIFY] cfg=%d order=%d finished in %.1f s\n", cfgId, nx, toc(tOrder));

            [valScore, fitMean, fitMin] = local_validation_score(dVal, sys, steps, opts);
            score = valScore + double(opts.OrderComplexityPenalty) * nx;
            status = "ok";
            message = "";
        catch ME
            sys = [];
            score = Inf;
            status = "failed";
            message = string(ME.message);
        end

        records = [records; table(cfgId, nx, score, fitMean, fitMin, string(status), string(message), ...
            'VariableNames', {'config_id', 'order', 'validation_score', ...
            'validation_fit_mean_pct', 'validation_fit_min_pct', 'status', 'message'})]; %#ok<AGROW>

        if ~isempty(sys) && isfinite(score) && (isempty(bestSys) || score < bestScore)
            bestScore = score;
            bestSys = sys;
        end
    end

    if isempty(bestSys)
        warning("AirdropX:AutoMPC:NoFiniteModelScore", ...
            "No finite validation score for config %d. No plant is selected.", cfgId);
    else
        bestRow = records(records.config_id == cfgId & records.validation_score == bestScore, :);
        if ~isempty(bestRow)
            fprintf("[IDENTIFY] cfg=%d selected order=%d fitMean=%.2f%% fitMin=%.2f%% score=%.3f\n", ...
                cfgId, bestRow.order(1), bestRow.validation_fit_mean_pct(1), ...
                bestRow.validation_fit_min_pct(1), bestRow.validation_score(1));
        end
    end
    models{cfgId + 1} = bestSys;
end

plant_bank = cell(5, 1);
for k = 1:5
    if ~isempty(models{k})
        plant_bank{k} = ss(models{k});
    end
end

result = struct();
result.models = models;
result.plant_bank = plant_bank;
result.trim_bank = data.trim_bank;
result.validation = records;
result.Ts = data.Ts;
result.data = data;

if strlength(string(opts.OutputMat)) > 0
    save(opts.OutputMat, "result", "plant_bank");
    outDir = fileparts(string(opts.OutputMat));
    if strlength(outDir) > 0
        writetable(records, fullfile(outDir, "identification_order_records.csv"));
    end
end
end

function [score, fitMean, fitMin] = local_validation_score(dVal, sys, steps, opts)
score = Inf;
fitMean = NaN;
fitMin = NaN;
if isempty(dVal), return; end

allFit = [];
for i = 1:numel(steps)
    try
        [~, fit] = compare(dVal, sys, steps(i));
        values = local_fit_values(fit);
        values = values(isfinite(values));
        if ~isempty(values)
            allFit = [allFit; values(:)]; %#ok<AGROW>
        end
    catch ME
        warning("AirdropX:AutoMPC:ValidationCompareFailed", ...
            "Validation compare failed at %d steps: %s", steps(i), ME.message);
    end
end
if isempty(allFit), return; end

fitMean = mean(allFit, "omitnan");
fitMin = min(allFit, [], "omitnan");
% Mean fit drives order selection; a small worst-output penalty prevents a
% model that completely misses one controlled output from winning on averages.
score = (100.0 - fitMean) + double(opts.MinFitPenaltyWeight) * max(0.0, 100.0 - fitMin);
if ~isfinite(score), score = Inf; end
end

function fit = local_fit_values(fit)
if iscell(fit)
    values = [];
    for i = 1:numel(fit)
        values = [values; double(fit{i}(:))]; %#ok<AGROW>
    end
    fit = values;
else
    fit = double(fit(:));
end
end

function opts = local_options(varargin)
opts.Data = [];
opts.DataMat = "";
opts.OutputMat = "";
opts.Orders = 3:10;
opts.ValidationSteps = [5 10 20 25];
opts.Focus = "simulation";
opts.EnforceStability = true;
opts.N4Weight = "CVA";
opts.RefineWithSsest = false;
opts.SsestMaxIterations = 20;
opts.OrderComplexityPenalty = 0.02;
opts.MinFitPenaltyWeight = 0.10;
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
if isempty(opts.Data) && strlength(string(opts.DataMat)) == 0
    error("Data or DataMat is required.");
end
end
