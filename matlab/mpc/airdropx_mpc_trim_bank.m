function bank = airdropx_mpc_trim_bank(cfg, varargin)
%AIRDROPX_MPC_TRIM_BANK Build per-drop trim, mass, CG, and pitch inertia data.

opts = local_options(varargin{:});
cargoMass = double(cfg.mass.cargo_mass_kg(:));
cargoX = double(cfg.mass.cargo_x_m(:));
cargoZ = double(cfg.mass.cargo_z_m(:));
dropMax = min(4, numel(cargoMass));

bank = repmat(struct( ...
    "drop_count", 0, ...
    "m_kg", 0.0, ...
    "Iy_kgm2", 0.0, ...
    "xCG_m", 0.0, ...
    "zCG_m", 0.0, ...
    "Va0_mps", 0.0, ...
    "gamma0_deg", 0.0, ...
    "gamma0_rad", 0.0, ...
    "theta0_deg", 0.0, ...
    "theta0_rad", 0.0, ...
    "alpha0_deg", 0.0, ...
    "alpha0_rad", 0.0, ...
    "q0_dps", 0.0, ...
    "q0_radps", 0.0, ...
    "delta_e0", 0.0, ...
    "delta_t0", 0.0), dropMax + 1, 1);

for dropCount = 0:dropMax
    remaining = (dropCount + 1):dropMax;
    mParts = [double(cfg.mass.empty_mass_kg); cargoMass(remaining)];
    xParts = [double(cfg.mass.empty_cg_x_m); cargoX(remaining)];
    zParts = [double(cfg.mass.empty_cg_z_m); cargoZ(remaining)];
    totalMass = sum(mParts);
    xCG = sum(mParts .* xParts) / totalMass;
    zCG = sum(mParts .* zParts) / totalMass;
    Iy = double(cfg.mass.empty_Iy_kgm2) + double(cfg.mass.empty_mass_kg) * ...
        ((double(cfg.mass.empty_cg_x_m) - xCG) ^ 2 + (double(cfg.mass.empty_cg_z_m) - zCG) ^ 2);
    for j = 1:numel(remaining)
        idx = remaining(j);
        Iy = Iy + cargoMass(idx) * ((cargoX(idx) - xCG) ^ 2 + (cargoZ(idx) - zCG) ^ 2);
    end

    bank(dropCount + 1).drop_count = dropCount;
    bank(dropCount + 1).m_kg = totalMass;
    bank(dropCount + 1).Iy_kgm2 = Iy;
    bank(dropCount + 1).xCG_m = xCG;
    bank(dropCount + 1).zCG_m = zCG;
    bank(dropCount + 1).Va0_mps = local_profile_value(opts.Va0Mps, dropCount + 1);
    bank(dropCount + 1).theta0_deg = local_profile_value(opts.Theta0Deg, dropCount + 1);
    bank(dropCount + 1).theta0_rad = deg2rad(bank(dropCount + 1).theta0_deg);
    bank(dropCount + 1).alpha0_deg = bank(dropCount + 1).theta0_deg - bank(dropCount + 1).gamma0_deg;
    bank(dropCount + 1).alpha0_rad = bank(dropCount + 1).theta0_rad - bank(dropCount + 1).gamma0_rad;
    bank(dropCount + 1).delta_e0 = local_profile_value(opts.DeltaE0, dropCount + 1);
    bank(dropCount + 1).delta_t0 = local_profile_value(opts.DeltaT0, dropCount + 1);
end
end

function value = local_profile_value(x, idx)
x = double(x(:));
if isempty(x)
    value = 0.0;
elseif numel(x) == 1
    value = x(1);
elseif numel(x) >= idx
    value = x(idx);
else
    value = x(end);
end
end

function opts = local_options(varargin)
opts.Va0Mps = 45.0;
opts.Theta0Deg = 4.0;
opts.DeltaE0 = 0.0;
opts.DeltaT0 = 0.80;
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
