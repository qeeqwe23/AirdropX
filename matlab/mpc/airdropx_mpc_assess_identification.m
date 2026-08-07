function report = airdropx_mpc_assess_identification(idResult, csvPath, varargin)
%AIRDROPX_MPC_ASSESS_IDENTIFICATION Assess Slegers-style MPC ID quality.

opts = local_options(varargin{:});
if ischar(idResult) || isstring(idResult)
    loaded = load(idResult);
    if isfield(loaded, "models")
        idResult = struct("models", loaded.models, "model_bank", {loaded.model_bank});
    elseif isfield(loaded, "id")
        idResult = loaded.id;
    else
        error("MAT file does not contain identification result fields.");
    end
end
if isfield(idResult, "cfg")
    cfg = idResult.cfg;
else
    cfg = airdropx_mpc_config( ...
        "TargetAirspeedMps", opts.TargetAirspeedMps, ...
        "TargetPitchDeg", opts.TargetPitchDeg, ...
        "IncludeModel", false);
end
if ~isfield(idResult, "model_bank")
    modelBank = airdropx_mpc_greybox_model(cfg, idResult);
else
    modelBank = idResult.model_bank;
end

T = readtable(csvPath);
T = local_filter_table(T, opts);
X = airdropx_mpc_state_from_table(T, cfg, "IncludeAux", true);
dropCount = local_drop_count(T, X(:, 6), cfg);
U = local_input_deviation(T, cfg, dropCount);
t = double(T.time_s(:));
step = local_prediction_step(t, cfg);

rows = table();
for k = 0:double(cfg.mass.drop_count_max)
    idx = find(dropCount(1:end-step) == k & dropCount((1:end-step) + step) == k);
    idx = idx(local_same_case(T, idx, step));
    if isempty(idx)
        rows = [rows; local_empty_row(k)]; %#ok<AGROW>
        continue;
    end
    model = modelBank{k + 1};
    xNow = X(idx, 1:5).';
    xNext = X(idx + step, 1:5).';
    uNow = U(idx, :).';
    xPred = double(model.A) * xNow + double(model.B) * uNow + double(model.c(:));
    err = xNext - xPred;
    rmse = sqrt(mean(err .^ 2, 2, "omitnan"));
    contEig = eig(double(model.continuous.A));
    discEig = eig(double(model.A));
    rows = [rows; table(k, numel(idx), ...
        rmse(1), rmse(2), rmse(3), rad2deg(rmse(4)), rad2deg(rmse(5)), ...
        max(real(contEig)), max(abs(discEig)), ...
        local_sign_score(model.force_parameters), ...
        'VariableNames', {'drop_count', 'samples', ...
        'h_rmse_m', 'vz_rmse_mps', 'va_rmse_mps', 'theta_rmse_deg', 'q_rmse_dps', ...
        'max_real_cont_eig', 'max_abs_disc_eig', 'sign_score'})]; %#ok<AGROW>
end

function T = local_filter_table(T, opts)
if ismember("time_s", string(T.Properties.VariableNames))
    t = double(T.time_s(:));
    mask = t >= double(opts.StartTimeS) & t <= double(opts.EndTimeS);
else
    mask = true(height(T), 1);
    t = (1:height(T)).';
end
if double(opts.DropTransitionPadS) > 0.0 && ismember("drop_count", string(T.Properties.VariableNames))
    dc = round(double(T.drop_count(:)));
    jumps = find(diff(dc) ~= 0) + 1;
    for i = 1:numel(jumps)
        mask = mask & abs(t - t(jumps(i))) > double(opts.DropTransitionPadS);
    end
end
T = T(mask, :);
end

report = struct();
report.csv_path = string(csvPath);
report.by_drop = rows;
report.ok = all(rows.samples >= double(opts.MinSamplesPerDrop)) && ...
    local_all_finite_le(rows.va_rmse_mps, double(opts.MaxVaRmseMps)) && ...
    local_all_finite_le(rows.theta_rmse_deg, double(opts.MaxThetaRmseDeg)) && ...
    local_all_finite_le(rows.q_rmse_dps, double(opts.MaxQRmseDps)) && ...
    local_all_finite_ge(rows.sign_score, double(opts.MinSignScore)) && ...
    all(isfinite(rows.max_abs_disc_eig));

if strlength(string(opts.OutputFile)) > 0
    writetable(rows, opts.OutputFile);
end

function tf = local_all_finite_le(x, threshold)
x = double(x(:));
x = x(isfinite(x));
tf = ~isempty(x) && all(x <= threshold);
end

function tf = local_all_finite_ge(x, threshold)
x = double(x(:));
x = x(isfinite(x));
tf = ~isempty(x) && all(x >= threshold);
end
end

function row = local_empty_row(k)
row = table(k, 0, NaN, NaN, NaN, NaN, NaN, NaN, NaN, 0.0, ...
    'VariableNames', {'drop_count', 'samples', ...
    'h_rmse_m', 'vz_rmse_mps', 'va_rmse_mps', 'theta_rmse_deg', 'q_rmse_dps', ...
    'max_real_cont_eig', 'max_abs_disc_eig', 'sign_score'});
end

function score = local_sign_score(params)
score = 0.0;
N = double(params.N(:));
X = double(params.X(:));
M = double(params.M(:));
checks = [
    N(2) > 0.0
    X(1) < 0.0
    X(6) > 0.0
    M(2) < 0.0
    M(3) < 0.0
    M(5) < 0.0
    ];
score = mean(double(checks));
end

function U = local_input_deviation(T, cfg, dropCount)
elevator = local_first_column(T, ["elevator_cmd_norm", "elevator_cmd", ...
    "elevator_actual", "elevator_delta"], cfg.trim.u(1));
throttle = local_first_column(T, ["throttle_actual", "throttle_cmd", ...
    "throttle_norm", "throttle_cmd_norm"], cfg.trim.u(2));
U = zeros(height(T), 2);
for i = 1:height(T)
    trim = cfg.trim.bank(dropCount(i) + 1);
    U(i, 1) = elevator(i) - double(trim.delta_e0);
    U(i, 2) = throttle(i) - double(trim.delta_t0);
end
end

function tf = local_same_case(T, idx, step)
tf = true(size(idx));
if ~ismember("case_id", string(T.Properties.VariableNames))
    return;
end
ids = string(T.case_id(:));
if isempty(idx)
    return;
end
tf = ids(idx) == ids(min(idx + step, numel(ids)));
end

function step = local_prediction_step(t, cfg)
dt = median(diff(unique(t)), "omitnan");
if ~isfinite(dt) || dt <= 0.0
    step = 1;
else
    step = max(1, round(double(cfg.dt_s) / dt));
end
end

function x = local_first_column(T, names, fallback)
vars = string(T.Properties.VariableNames);
for i = 1:numel(names)
    if ismember(names(i), vars)
        x = double(T.(names(i))(:));
        return;
    end
end
x = double(fallback) * ones(height(T), 1);
end

function dropCount = local_drop_count(T, massErr, cfg)
if ismember("drop_count", string(T.Properties.VariableNames))
    dropCount = round(double(T.drop_count(:)));
else
    cargoMass = mean(double(cfg.mass.cargo_mass_kg(:)), "omitnan");
    if ~isfinite(cargoMass) || cargoMass <= 0.0
        cargoMass = 300.0;
    end
    dropCount = round(max(0.0, -double(massErr(:)) / cargoMass));
end
dropCount = min(max(dropCount, 0), double(cfg.mass.drop_count_max));
end

function opts = local_options(varargin)
opts.TargetAirspeedMps = 50.0;
opts.TargetPitchDeg = 4.0;
opts.MinSamplesPerDrop = 200;
opts.MaxVaRmseMps = 0.50;
opts.MaxThetaRmseDeg = 0.35;
opts.MaxQRmseDps = 4.0;
opts.MinSignScore = 0.50;
opts.OutputFile = "";
opts.StartTimeS = 0.0;
opts.EndTimeS = Inf;
opts.DropTransitionPadS = 0.0;
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
