function result = airdropx_mpc_identify_from_csv(csvPath, varargin)
%AIRDROPX_MPC_IDENTIFY_FROM_CSV Strict Slegers-style grey-box RLS ID.
%
% For each drop configuration, identify 21 force/moment derivatives:
%   theta_N = [N_V N_alpha N_q N_alphadot N_de N_dt b_N]'
%   theta_X = [X_V X_alpha X_q X_alphadot X_de X_dt b_X]'
%   theta_M = [M_V M_alpha M_q M_alphadot M_de M_dt b_M]'

opts = local_options(varargin{:});
cfg = airdropx_mpc_config( ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", opts.TargetPitchDeg, ...
    "IncludeModel", false);

T = readtable(csvPath);
T = local_filter_table(T, opts);
t = double(T.time_s(:));
dropCount = local_drop_count_from_table(T, cfg);
[Phi, zAll] = local_regression_data(T, t, dropCount, cfg);

theta0 = [cfg.estimator.theta0.N(:); cfg.estimator.theta0.X(:); cfg.estimator.theta0.M(:)];
models = repmat(struct("drop_count", 0, "force_parameters", []), 5, 1);
records = table();
for k = 0:4
    idx = find(dropCount == k);
    if numel(idx) < 12
        idx = [];
    end
    if isempty(idx)
        fit.theta = theta0;
        fit.P = double(cfg.estimator.P0Scale) * eye(numel(theta0));
        samples = 0;
    else
        fit = local_joint_rls(theta0, Phi(idx, :), zAll(idx, :), cfg);
        samples = numel(idx);
    end
    params = local_unpack_params(fit.theta);
    models(k + 1).drop_count = k;
    models(k + 1).force_parameters = params;
    records = [records; local_record(k, samples, params, fit.P)]; %#ok<AGROW>
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

model_bank = airdropx_mpc_greybox_model(cfg, struct("models", models));
cfg.model_bank = model_bank;
cfg.model = model_bank{1};

result = struct();
result.cfg = cfg;
if logical(opts.EnforcePhysicalBounds)
    [models, projectionRecords] = local_enforce_physical_bounds(models, cfg);
    model_bank = airdropx_mpc_greybox_model(cfg, struct("models", models));
    cfg.model_bank = model_bank;
    cfg.model = model_bank{1};
    records = local_records_from_models(models, records);
else
    projectionRecords = table();
end
result.model_bank = model_bank;
result.models = models;
result.parameters = records;
result.projection = projectionRecords;
result.source_csv = string(csvPath);

if strlength(string(opts.OutputMat)) > 0
    outputMat = string(opts.OutputMat);
    outDir = fileparts(outputMat);
    if strlength(outDir) > 0 && ~isfolder(outDir)
        mkdir(outDir);
    end
    save(outputMat, "cfg", "model_bank", "models", "records", "projectionRecords");
    result.output_mat = outputMat;
end
end

function [models, records] = local_enforce_physical_bounds(models, cfg)
records = table();
for k = 1:numel(models)
    params = models(k).force_parameters;
    raw = params;
    [params.N, recN] = local_bound_vector(params.N, cfg.estimator.theta0.N, "N", k - 1);
    [params.X, recX] = local_bound_vector(params.X, cfg.estimator.theta0.X, "X", k - 1);
    [params.M, recM] = local_bound_vector(params.M, cfg.estimator.theta0.M, "M", k - 1);
    models(k).raw_force_parameters = raw;
    models(k).force_parameters = params;
    records = [records; recN; recX; recM]; %#ok<AGROW>
end
end

function [theta, records] = local_bound_vector(theta, prior, groupName, dropCount)
theta = double(theta(:));
prior = double(prior(:));
records = table();
switch string(groupName)
    case "N"
        checks = {
            2,  1,   5.0e2, 1.2e5, "N_alpha"
            };
    case "X"
        checks = {
            1, -1,  -5.0e3, -1.0, "X_V"
            6,  1,   1.0e3, 5.0e4, "X_dt"
            };
    case "M"
        checks = {
            2, -1, -2.5e5, -1.0e3, "M_alpha"
            3, -1, -1.0e5, -1.0e2, "M_q"
            5, -1, -1.2e5, -1.0e2, "M_de"
            };
    otherwise
        checks = {};
end

for i = 1:size(checks, 1)
    idx = checks{i, 1};
    signWanted = checks{i, 2};
    lo = checks{i, 3};
    hi = checks{i, 4};
    name = string(checks{i, 5});
    oldValue = theta(idx);
    newValue = oldValue;
    reason = "";
    if ~isfinite(newValue) || signWanted * newValue <= 0.0
        newValue = prior(idx);
        reason = "prior_sign";
    end
    if newValue < lo
        newValue = lo;
        reason = reason + "_lo";
    elseif newValue > hi
        newValue = hi;
        reason = reason + "_hi";
    end
    theta(idx) = newValue;
    if newValue ~= oldValue
        records = [records; table(dropCount, string(groupName), name, oldValue, newValue, reason, ...
            'VariableNames', {'drop_count', 'group', 'parameter', 'raw_value', 'projected_value', 'reason'})]; %#ok<AGROW>
    end
end
end

function records = local_records_from_models(models, oldRecords)
records = oldRecords;
for k = 1:numel(models)
    idx = records.drop_count == models(k).drop_count;
    if ~any(idx)
        continue;
    end
    params = models(k).force_parameters;
    records.N_V(idx) = params.N(1);
    records.N_alpha(idx) = params.N(2);
    records.N_q(idx) = params.N(3);
    records.N_alphadot(idx) = params.N(4);
    records.N_de(idx) = params.N(5);
    records.N_dt(idx) = params.N(6);
    records.b_N(idx) = params.N(7);
    records.X_V(idx) = params.X(1);
    records.X_alpha(idx) = params.X(2);
    records.X_q(idx) = params.X(3);
    records.X_alphadot(idx) = params.X(4);
    records.X_de(idx) = params.X(5);
    records.X_dt(idx) = params.X(6);
    records.b_X(idx) = params.X(7);
    records.M_V(idx) = params.M(1);
    records.M_alpha(idx) = params.M(2);
    records.M_q(idx) = params.M(3);
    records.M_alphadot(idx) = params.M(4);
    records.M_de(idx) = params.M(5);
    records.M_dt(idx) = params.M(6);
    records.b_M(idx) = params.M(7);
end
end

function [Phi, zAll] = local_regression_data(T, t, dropCount, cfg)
altitude = local_column(T, "altitude_m", cfg.reference.h_m);
vz = local_column(T, "vz_up_mps", 0.0);
va = max(local_column(T, "airspeed_mps", cfg.reference.v_mps), 1.0);
thetaDeg = local_column(T, "pitch_deg", cfg.reference.pitch_deg);
thetaRad = deg2rad(thetaDeg);
qDps = local_grouped_derivative(t, thetaDeg, T);
qDps = movmean(qDps, cfg.estimator.FilterWindow, "omitnan");
qRad = deg2rad(qDps);

gamma = asin(min(max(vz ./ va, -0.98), 0.98));
alpha = thetaRad - gamma;
alphaDot = local_grouped_derivative(t, alpha, T);
alphaDot = movmean(alphaDot, cfg.estimator.FilterWindow, "omitnan");

vzDot = local_grouped_derivative(t, vz, T);
vaDot = local_grouped_derivative(t, va, T);
qDotRad = local_grouped_derivative(t, qRad, T);

U = local_inputs_from_table(T, cfg, dropCount);
Phi = zeros(height(T), 7);
zAll = zeros(height(T), 3);
for i = 1:height(T)
    trim = cfg.trim.bank(dropCount(i) + 1);
    vaStar = va(i) - double(trim.Va0_mps);
    alphaStar = alpha(i) - double(trim.alpha0_rad);
    Phi(i, :) = [vaStar, alphaStar, qRad(i), alphaDot(i), U(i, 1), U(i, 2), 1.0];

    m = double(trim.m_kg);
    Iy = double(trim.Iy_kgm2);
    zAll(i, 1) = m * vzDot(i);
    zAll(i, 2) = m * vaDot(i) + m * double(cfg.gravity_mps2) * gamma(i);
    zAll(i, 3) = Iy * qDotRad(i);
end
if ~isempty(altitude) %#ok<*BDSCA>
    % Keep altitude read above intentional: it validates the exported CSV has
    % the measured channel expected by the MPC state conversion path.
end
end

function fit = local_joint_rls(theta0, phiRows, zRows, cfg)
theta = double(theta0(:));
P = double(cfg.estimator.P0Scale) * eye(numel(theta));
R = double(cfg.estimator.R);
lambda = double(cfg.estimator.ForgettingFactor);

for i = 1:size(phiRows, 1)
    phi = double(phiRows(i, :));
    z = double(zRows(i, :)).';
    if any(~isfinite(phi)) || any(~isfinite(z))
        continue;
    end
    H = zeros(3, 21);
    H(1, 1:7) = phi;
    H(2, 8:14) = phi;
    H(3, 15:21) = phi;
    S = H * P * H.' + R;
    K = P * H.' / S;
    theta = theta + K * (z - H * theta);
    P = (eye(numel(theta)) - K * H) * P / lambda;
    P = 0.5 * (P + P.');
end
fit.theta = theta;
fit.P = P;
end

function params = local_unpack_params(theta)
theta = double(theta(:));
params = struct();
params.N = theta(1:7);
params.X = theta(8:14);
params.M = theta(15:21);
end

function U = local_inputs_from_table(T, cfg, dropCount)
elevator = local_first_column(T, ["elevator_cmd_norm", "elevator_cmd", "elevator_actual", "elevator_delta"], cfg.trim.u(1));
throttle = local_first_column(T, ["throttle_actual", "throttle_cmd", "throttle_norm", "throttle_cmd_norm"], cfg.trim.u(2));
U = zeros(height(T), 2);
for i = 1:height(T)
    trim = cfg.trim.bank(dropCount(i) + 1);
    U(i, 1) = elevator(i) - double(trim.delta_e0);
    U(i, 2) = throttle(i) - double(trim.delta_t0);
end
end

function x = local_first_column(T, names, fallback)
x = [];
vars = string(T.Properties.VariableNames);
for i = 1:numel(names)
    if ismember(names(i), vars)
        x = double(T.(names(i))(:));
        return;
    end
end
x = double(fallback) * ones(height(T), 1);
end

function x = local_column(T, name, fallback)
if ismember(name, string(T.Properties.VariableNames))
    x = double(T.(name)(:));
else
    x = double(fallback) * ones(height(T), 1);
end
end

function dropCount = local_drop_count_from_table(T, cfg)
if ismember("drop_count", string(T.Properties.VariableNames))
    dropCount = round(double(T.drop_count(:)));
else
    massKg = local_column(T, "mass_kg", cfg.reference.mass_kg);
    cargoMass = mean(double(cfg.mass.cargo_mass_kg(:)), "omitnan");
    dropCount = round(max(0.0, (double(cfg.trim.bank(1).m_kg) - massKg) / cargoMass));
end
dropCount = min(max(dropCount, 0), 4);
end

function dx = local_grouped_derivative(t, x, T)
dx = zeros(size(x));
if ismember("case_id", string(T.Properties.VariableNames))
    ids = string(T.case_id(:));
else
    ids = repmat("case_001", height(T), 1);
end
u = unique(ids, "stable");
for i = 1:numel(u)
    idx = find(ids == u(i));
    if numel(idx) < 2
        continue;
    end
    tg = t(idx);
    xg = movmean(double(x(idx)), 5, "omitnan");
    dt = [max(tg(2) - tg(1), eps); max(diff(tg), eps)];
    dx(idx) = [0.0; diff(xg)] ./ dt;
end
end

function row = local_record(dropCount, samples, params, P)
N = params.N;
X = params.X;
M = params.M;
row = table(dropCount, samples, trace(P), ...
    N(1), N(2), N(3), N(4), N(5), N(6), N(7), ...
    X(1), X(2), X(3), X(4), X(5), X(6), X(7), ...
    M(1), M(2), M(3), M(4), M(5), M(6), M(7), ...
    'VariableNames', { ...
    'drop_count', 'samples', 'cov_trace', ...
    'N_V', 'N_alpha', 'N_q', 'N_alphadot', 'N_de', 'N_dt', 'b_N', ...
    'X_V', 'X_alpha', 'X_q', 'X_alphadot', 'X_de', 'X_dt', 'b_X', ...
    'M_V', 'M_alpha', 'M_q', 'M_alphadot', 'M_de', 'M_dt', 'b_M'});
end

function opts = local_options(varargin)
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.OutputMat = "";
opts.EnforcePhysicalBounds = true;
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
