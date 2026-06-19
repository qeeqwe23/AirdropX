function [total_mass_kg, cg_x_m, cargo_onboard] = airdropx_mass_cg_update(drop_index, drop_trigger, cargo_onboard)
%AIRDROPX_MASS_CG_UPDATE MQ-9 cargo mass and CG update.
%
% drop_index is 1-based for MATLAB/Simulink. Pass 1..4.

cfg = airdropx_sim_params();
empty_mass_kg = cfg.mass.empty_mass_kg;
x_cg_empty_m = cfg.mass.empty_cg_x_m;
cargo_mass_kg = cfg.mass.cargo_mass_kg;
cargo_x_m = cfg.mass.cargo_x_m;

if nargin < 3 || isempty(cargo_onboard)
    cargo_onboard = true(1, 4);
end

if nargin >= 2 && drop_trigger
    idx = round(double(drop_index));
    if idx >= 1 && idx <= 4
        cargo_onboard(idx) = false;
    end
end

total_mass_kg = empty_mass_kg;
total_moment = empty_mass_kg * x_cg_empty_m;

for i = 1:4
    if cargo_onboard(i)
        total_mass_kg = total_mass_kg + cargo_mass_kg(i);
        total_moment = total_moment + cargo_mass_kg(i) * cargo_x_m(i);
    end
end

cg_x_m = total_moment / total_mass_kg;
end
