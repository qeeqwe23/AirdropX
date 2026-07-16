function result = airdropx_mpc_identify_from_csv(csvPath, varargin)
%AIRDROPX_MPC_IDENTIFY_FROM_CSV Fit a local discrete model from exported data.
%
% Closed-loop data without excitation can be poorly conditioned. Check
% result.diagnostics before trusting the model.

opts = local_options(varargin{:});
cfg = airdropx_mpc_config( ...
    "Dt", opts.ModelDt, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", opts.TargetPitchDeg, ...
    "IncludeModel", false);

T = readtable(csvPath);
T = T(T.time_s >= opts.StartTime & T.time_s <= opts.EndTime, :);
if height(T) < 10
    error("Not enough CSV rows for identification.");
end

Xraw = airdropx_mpc_state_from_table(T, cfg);
Uraw = local_input_from_table(T);

stride = max(1, round(opts.ModelDt / median(diff(T.time_s))));
idx = 1:stride:height(T);
T = T(idx, :);
X = Xraw(idx, :);
U = Uraw(idx, :);

if size(X, 1) < 10
    error("Not enough resampled rows for identification.");
end

if opts.UseLastTrim
    trimRows = local_trim_rows(T, opts.TrimWindowS);
    uTrim = [mean(T.elevator_cmd_norm(trimRows), "omitnan"); ...
             mean(T.throttle_norm(trimRows), "omitnan")];
else
    uTrim = cfg.trim.u(:);
end

dU = U - uTrim.';
valid = local_valid_transition_rows(T);
Y = X(valid + 1, :);
M = [X(valid, :), dU(valid, :), ones(numel(valid), 1)];

lambda = double(opts.Regularization);
[theta, fitDiagnostics] = local_fit_scaled(M, Y, lambda);

n = numel(cfg.state_names);
m = numel(cfg.input_names);
A = theta(1:n, :).';
B = theta(n + (1:m), :).';
c = theta(end, :).';

Yhat = M * theta;
err = Y - Yhat;
rmse = sqrt(mean(err .^ 2, 1));

model = struct();
model.A = A;
model.B = B;
model.c = c;
model.u_trim = uTrim;
model.x_trim = zeros(n, 1);
model.dt_s = opts.ModelDt;
model.state_names = cfg.state_names;
model.input_names = cfg.input_names;
model.source = "csv_least_squares";
model.csv_path = string(csvPath);
model.notes = "Identified from closed-loop exported data; validate before controller use.";

diagnostics = struct();
diagnostics.rows_raw = height(T);
diagnostics.rows_resampled = size(X, 1);
diagnostics.rows_transitions = numel(valid);
diagnostics.stride = stride;
diagnostics.regularization = lambda;
diagnostics.regressor_rank = rank(M);
diagnostics.regressor_columns = size(M, 2);
diagnostics.regressor_condition_raw = cond(M' * M + lambda * eye(size(M, 2)));
diagnostics.regressor_condition = fitDiagnostics.scaled_condition;
diagnostics.regressor_scale = fitDiagnostics.regressor_scale;
diagnostics.rmse_by_state = array2table(rmse, 'VariableNames', cellstr(cfg.state_names));
diagnostics.affine_offset = c;
diagnostics.input_source = "elevator: u_out + mpc_elevator_excitation when available; throttle: throttle_norm";

result = struct();
result.cfg = cfg;
result.model = model;
result.diagnostics = diagnostics;

if strlength(opts.OutputMat) > 0
    outDir = fileparts(opts.OutputMat);
    if strlength(string(outDir)) > 0 && ~isfolder(outDir)
        mkdir(outDir);
    end
    save(opts.OutputMat, "model", "diagnostics", "cfg");
end

function [theta, diagnostics] = local_fit_scaled(M, Y, lambda)
if size(M, 2) < 2
    error("Regressor matrix must include at least one dynamic column and one intercept.");
end

dynamicCols = 1:(size(M, 2) - 1);
mu = mean(M(:, dynamicCols), 1, "omitnan");
scale = std(M(:, dynamicCols), 0, 1, "omitnan");
scale(~isfinite(scale) | scale < 1.0e-9) = 1.0;

Ms = M;
Ms(:, dynamicCols) = (M(:, dynamicCols) - mu) ./ scale;

Hs = Ms' * Ms + lambda * eye(size(Ms, 2));
thetaScaled = Hs \ (Ms' * Y);

theta = zeros(size(thetaScaled));
theta(dynamicCols, :) = thetaScaled(dynamicCols, :) ./ scale(:);
theta(end, :) = thetaScaled(end, :) - (mu ./ scale) * thetaScaled(dynamicCols, :);

diagnostics = struct();
diagnostics.scaled_condition = cond(Hs);
diagnostics.regressor_mean = mu;
diagnostics.regressor_scale = scale;
end

function trimRows = local_trim_rows(T, trimWindowS)
trimRows = false(height(T), 1);
if ismember("case_id", string(T.Properties.VariableNames))
    ids = string(T.case_id(:));
else
    ids = repmat("case_001", height(T), 1);
end

uniqueIds = unique(ids, "stable");
for i = 1:numel(uniqueIds)
    idx = find(ids == uniqueIds(i));
    tEnd = max(T.time_s(idx));
    trimRows(idx) = T.time_s(idx) >= tEnd - trimWindowS;
end
end

function valid = local_valid_transition_rows(T)
if ismember("case_id", string(T.Properties.VariableNames))
    ids = string(T.case_id(:));
else
    ids = repmat("case_001", height(T), 1);
end

sameCase = ids(1:end - 1) == ids(2:end);
timeForward = T.time_s(2:end) > T.time_s(1:end - 1);
valid = find(sameCase & timeForward);
end

function U = local_input_from_table(T)
names = string(T.Properties.VariableNames);

if ismember("u_out", names)
    elevator = double(T.u_out(:));
    if ismember("mpc_elevator_excitation", names)
        elevator = elevator + double(T.mpc_elevator_excitation(:));
    end
elseif ismember("elevator_delta", names)
    elevator = double(T.elevator_delta(:));
elseif ismember("elevator_cmd_norm", names)
    elevator = double(T.elevator_cmd_norm(:));
else
    error("CSV table is missing an elevator input column.");
end

if ismember("throttle_norm", names)
    throttle = double(T.throttle_norm(:));
    if ismember("mpc_throttle_excitation", names)
        % throttle_norm is the logged plant command in the current model.
        % Keep it as authoritative rather than adding excitation twice.
    end
elseif ismember("throttle_cmd", names)
    throttle = double(T.throttle_cmd(:));
else
    error("CSV table is missing a throttle input column.");
end

U = [elevator, throttle];
end
end

function opts = local_options(varargin)
opts.ModelDt = 0.10;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.StartTime = 0.0;
opts.EndTime = inf;
opts.TrimWindowS = 5.0;
opts.UseLastTrim = true;
opts.Regularization = 1.0e-5;
opts.OutputMat = "";

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
