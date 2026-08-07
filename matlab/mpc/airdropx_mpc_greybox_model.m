function bank = airdropx_mpc_greybox_model(cfg, identified)
%AIRDROPX_MPC_GREYBOX_MODEL Build five strict longitudinal grey-box models.
%
% Identified force/moment parameters use SI angle units:
%   phi = [Va*, alpha*, q, alphadot*, delta_e*, delta_t*, 1]
%   theta_N = [N_V, N_alpha, N_q, N_alphadot, N_de, N_dt, b_N]
%   theta_X = [X_V, X_alpha, X_q, X_alphadot, X_de, X_dt, b_X]
%   theta_M = [M_V, M_alpha, M_q, M_alphadot, M_de, M_dt, b_M]
%
% Angle states use radians internally, matching the continuous derivation.

if nargin < 1 || isempty(cfg)
    cfg = airdropx_mpc_config("IncludeModel", false);
end
if nargin < 2
    identified = [];
end

dropMax = double(cfg.mass.drop_count_max);
bank = cell(dropMax + 1, 1);
for dropCount = 0:dropMax
    trim = cfg.trim.bank(dropCount + 1);
    params = local_params(cfg, identified, dropCount);
    cont = local_continuous_from_force_params(params, trim, cfg);
    model = local_discretize(cont, double(cfg.dt_s));
    model.drop_count = dropCount;
    model.trim = trim;
    model.force_parameters = params;
    model.state_names = cfg.state_names(:);
    model.input_names = cfg.input_names;
    model.u_trim = [trim.delta_e0; trim.delta_t0];
    model.source = "strict_greybox_default";
    if ~isempty(identified)
        model.source = "strict_greybox_identified";
    end
    model.continuous = cont;
    bank{dropCount + 1} = model;
end
end

function params = local_params(cfg, identified, dropCount)
params = struct();
if ~isempty(identified) && isfield(identified, "models") && numel(identified.models) >= dropCount + 1 && ...
        isfield(identified.models(dropCount + 1), "force_parameters")
    params = identified.models(dropCount + 1).force_parameters;
else
    params.N = double(cfg.estimator.theta0.N(:));
    params.X = double(cfg.estimator.theta0.X(:));
    params.M = double(cfg.estimator.theta0.M(:));
end
end

function cont = local_continuous_from_force_params(params, trim, cfg)
N = double(params.N(:));
X = double(params.X(:));
M = double(params.M(:));

m = max(double(trim.m_kg), 1.0);
Iy = max(double(trim.Iy_kgm2), 1.0);
Va0 = max(double(trim.Va0_mps), 1.0);
g = double(cfg.gravity_mps2);

N_V = N(1); N_a = N(2); N_q = N(3); N_adot = N(4); N_de = N(5); N_dt = N(6); b_N = N(7);
X_V = X(1); X_a = X(2); X_q = X(3); X_adot = X(4); X_de = X(5); X_dt = X(6); b_X = X(7);
M_V = M(1); M_a = M(2); M_q = M(3); M_adot = M(4); M_de = M(5); M_dt = M(6); b_M = M(7);

mz = m + N_adot / Va0;
if abs(mz) < 1.0
    mz = sign(mz + eps);
end

Z.Vz = -N_a / (mz * Va0);
Z.V  =  N_V / mz;
Z.th =  N_a / mz;
Z.q  = (N_q + N_adot) / mz;
Z.de =  N_de / mz;
Z.dt =  N_dt / mz;
Z.d  =  b_N / mz;

Xt.Vz = -(X_a + m * g) / (m * Va0);
Xt.V  =  X_V / m;
Xt.th =  X_a / m;
Xt.q  = (X_q + X_adot) / m;
Xt.de =  X_de / m;
Xt.dt =  X_dt / m;
Xt.d  =  b_X / m;
kX = X_adot / (m * Va0);

Xe.Vz = Xt.Vz - kX * Z.Vz;
Xe.V  = Xt.V  - kX * Z.V;
Xe.th = Xt.th - kX * Z.th;
Xe.q  = Xt.q  - kX * Z.q;
Xe.de = Xt.de - kX * Z.de;
Xe.dt = Xt.dt - kX * Z.dt;
Xe.d  = Xt.d  - kX * Z.d;

Mt.Vz = -M_a / (Iy * Va0);
Mt.V  =  M_V / Iy;
Mt.th =  M_a / Iy;
Mt.q  = (M_q + M_adot) / Iy;
Mt.de =  M_de / Iy;
Mt.dt =  M_dt / Iy;
Mt.d  =  b_M / Iy;
kM = M_adot / (Iy * Va0);

Me.Vz = Mt.Vz - kM * Z.Vz;
Me.V  = Mt.V  - kM * Z.V;
Me.th = Mt.th - kM * Z.th;
Me.q  = Mt.q  - kM * Z.q;
Me.de = Mt.de - kM * Z.de;
Me.dt = Mt.dt - kM * Z.dt;
Me.d  = Mt.d  - kM * Z.d;

Ac = zeros(5, 5);
Bc = zeros(5, 2);
dc = zeros(5, 1);

Ac(1, 2) = 1.0;
Ac(4, 5) = 1.0;

Ac(2, 2) = Z.Vz;
Ac(2, 3) = Z.V;
Ac(2, 4) = Z.th;
Ac(2, 5) = Z.q;
Bc(2, :) = [Z.de, Z.dt];
dc(2) = Z.d;

Ac(3, 2) = Xe.Vz;
Ac(3, 3) = Xe.V;
Ac(3, 4) = Xe.th;
Ac(3, 5) = Xe.q;
Bc(3, :) = [Xe.de, Xe.dt];
dc(3) = Xe.d;

Ac(5, 2) = Me.Vz;
Ac(5, 3) = Me.V;
Ac(5, 4) = Me.th;
Ac(5, 5) = Me.q;
Bc(5, :) = [Me.de, Me.dt];
dc(5) = Me.d;

cont = struct();
cont.A = Ac;
cont.B = Bc;
cont.d = dc;
cont.mz_kg = mz;
cont.effective = struct("Z", Z, "X", Xe, "M", Me);
cont.state_units = ["m"; "m/s"; "m/s"; "rad"; "rad/s"];
cont.input_units = ["elevator"; "throttle"];
end

function model = local_discretize(cont, dt)
Ac = double(cont.A);
Bc = double(cont.B);
dc = double(cont.d(:));
n = size(Ac, 1);
m = size(Bc, 2);
aug = zeros(n + m + 1, n + m + 1);
aug(1:n, 1:n) = Ac;
aug(1:n, n + (1:m)) = Bc;
aug(1:n, n + m + 1) = dc;
disc = expm(aug * dt);
model = struct();
model.A = disc(1:n, 1:n);
model.B = disc(1:n, n + (1:m));
model.c = disc(1:n, n + m + 1);
end
