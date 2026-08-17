function result = airdropx_auto_final_test(varargin)
%AIRDROPX_AUTO_FINAL_TEST Score final held-out JSBSim closed-loop cases.
%
% Use either TimeseriesCsv for already exported runs, or SimulationFcn for a
% runner that executes the real JSBSim/Multiple-MPC closed loop.

opts = local_options(varargin{:});
rows = table();

if ~isempty(opts.TimeseriesCsv)
    files = local_string_list(opts.TimeseriesCsv);
    for i = 1:numel(files)
        score = airdropx_auto_score_closed_loop(files(i));
        [~, name] = fileparts(files(i));
        row = local_metric_row(string(name), score.metrics, files(i));
        rows = [rows; row]; %#ok<AGROW>
    end
else
    if isempty(opts.SimulationFcn)
        error("AirdropX:AutoMPC:NoEvaluation", ...
            "Pass TimeseriesCsv or SimulationFcn. Final testing must use JSBSim closed-loop data.");
    end
    cases = opts.Cases;
    if isempty(cases)
        cases = struct("name", "nominal");
    end
    for i = 1:numel(cases)
        caseName = local_case_name(cases(i), i);
        simOut = feval(opts.SimulationFcn, opts.MpcBankMat, cases(i));
        [T, csvPath] = local_timeseries_from_output(simOut);
        score = airdropx_auto_score_closed_loop(T);
        row = local_metric_row(caseName, score.metrics, csvPath);
        rows = [rows; row]; %#ok<AGROW>
    end
end

result = struct();
result.table = rows;
if isempty(rows)
    result.score = Inf;
else
    result.score = mean(rows.score, "omitnan") + 0.25 * max(rows.score, [], "omitnan");
end

if strlength(string(opts.OutputFile)) > 0
    outDir = fileparts(opts.OutputFile);
    if strlength(string(outDir)) > 0 && ~isfolder(outDir)
        mkdir(outDir);
    end
    writetable(rows, opts.OutputFile);
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

function row = local_metric_row(caseName, metrics, csvPath)
row = metrics;
row = addvars(row, string(caseName), string(csvPath), 'Before', 1, ...
    'NewVariableNames', {'case_name', 'timeseries_csv'});
end

function name = local_case_name(caseDef, idx)
name = "case_" + string(idx);
if isstruct(caseDef) && isfield(caseDef, "name")
    name = string(caseDef.name);
elseif istable(caseDef) && ismember("name", string(caseDef.Properties.VariableNames))
    name = string(caseDef.name(1));
end
end

function [T, csvPath] = local_timeseries_from_output(simOut)
csvPath = "";
if istable(simOut)
    T = simOut;
elseif isstring(simOut) || ischar(simOut)
    csvPath = string(simOut);
    T = readtable(csvPath);
elseif isstruct(simOut)
    if isfield(simOut, "timeseries") && istable(simOut.timeseries)
        T = simOut.timeseries;
    elseif isfield(simOut, "timeseries_csv")
        csvPath = string(simOut.timeseries_csv);
        T = readtable(csvPath);
    elseif isfield(simOut, "csv")
        csvPath = string(simOut.csv);
        T = readtable(csvPath);
    else
        error("AirdropX:AutoMPC:BadSimOutput", ...
            "SimulationFcn must return a table, CSV path, or struct with timeseries/timeseries_csv.");
    end
else
    error("AirdropX:AutoMPC:BadSimOutput", "Unsupported SimulationFcn output.");
end
end

function opts = local_options(varargin)
opts.MpcBankMat = "";
opts.TimeseriesCsv = strings(0, 1);
opts.SimulationFcn = [];
opts.Cases = [];
opts.OutputFile = "";
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
end
