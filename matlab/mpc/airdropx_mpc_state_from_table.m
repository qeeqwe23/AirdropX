function X = airdropx_mpc_state_from_table(T, cfg, varargin)
%AIRDROPX_MPC_STATE_FROM_TABLE Convert exported data to MPC reduced states.
%
% Main state order:
%   [h*, Vz, Va*, theta*_rad, q_radps]
%
% Set "IncludeAux", true to append [mass_err_kg, cg_x_err_m] for drop-count
% compatibility with the online controller wrapper.

opts = local_options(varargin{:});
if nargin < 2 || isempty(cfg)
    cfg = airdropx_mpc_config();
end
required = ["time_s", "altitude_m", "vz_up_mps", "airspeed_mps", "pitch_deg"];
for i = 1:numel(required)
    if ~ismember(required(i), string(T.Properties.VariableNames))
        error("CSV table is missing required column: %s", required(i));
    end
end

t = double(T.time_s(:));
h = double(T.altitude_m(:));
vz = double(T.vz_up_mps(:));
va = double(T.airspeed_mps(:));
thetaRad = deg2rad(double(T.pitch_deg(:)));
massKg = local_column(T, "mass_kg", cfg.reference.mass_kg);
cgXM = local_column(T, "cg_x_m", cfg.reference.cg_x_m);
dropCount = local_drop_count(T, massKg, cfg);

hRef = local_column(T, "target_altitude_m", cfg.reference.h_m);
vaRef = zeros(height(T), 1);
thetaRefRad = zeros(height(T), 1);
massRef = zeros(height(T), 1);
cgRef = zeros(height(T), 1);
for i = 1:height(T)
    trim = cfg.trim.bank(dropCount(i) + 1);
    vaRef(i) = double(trim.Va0_mps);
    thetaRefRad(i) = double(trim.theta0_rad);
    massRef(i) = double(cfg.trim.bank(1).m_kg);
    cgRef(i) = double(trim.xCG_m);
end

qRadps = local_grouped_derivative(t, thetaRad, T);
qRadps = local_moving_average(qRadps, 5);

X = [h - hRef, vz, va - vaRef, thetaRad - thetaRefRad, qRadps];
if logical(opts.IncludeAux)
    X = [X, massKg - massRef, cgXM - cgRef];
end
end

function opts = local_options(varargin)
opts.IncludeAux = false;
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

function x = local_column(T, name, fallback)
if ismember(name, string(T.Properties.VariableNames))
    x = double(T.(name)(:));
else
    x = double(fallback) * ones(height(T), 1);
end
end

function dropCount = local_drop_count(T, massKg, cfg)
if ismember("drop_count", string(T.Properties.VariableNames))
    dropCount = round(double(T.drop_count(:)));
else
    cargoMass = mean(double(cfg.mass.cargo_mass_kg(:)), "omitnan");
    if ~isfinite(cargoMass) || cargoMass <= 0.0
        cargoMass = 300.0;
    end
    dropCount = round(max(0.0, (double(cfg.trim.bank(1).m_kg) - double(massKg(:))) / cargoMass));
end
dropCount = min(max(dropCount, 0), double(cfg.mass.drop_count_max));
end

function dx = local_grouped_derivative(t, x, T)
dx = zeros(size(x));
groups = local_groups(T);
for g = 1:numel(groups)
    idx = groups{g};
    if numel(idx) < 2
        continue;
    end
    tg = t(idx);
    xg = double(x(idx));
    dt = [max(tg(2) - tg(1), eps); max(diff(tg), eps)];
    dx(idx) = [0.0; diff(xg)] ./ dt;
end
end

function groups = local_groups(T)
if ismember("case_id", string(T.Properties.VariableNames))
    ids = string(T.case_id(:));
else
    ids = repmat("case_001", height(T), 1);
end
u = unique(ids, "stable");
groups = cell(numel(u), 1);
for i = 1:numel(u)
    groups{i} = find(ids == u(i));
end
end

function y = local_moving_average(x, window)
window = max(1, round(double(window)));
if window <= 1
    y = double(x(:));
else
    y = movmean(double(x(:)), window, "omitnan");
end
end
