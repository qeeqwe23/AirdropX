function model = airdropx_mpc_nominal_model(cfg)
%AIRDROPX_MPC_NOMINAL_MODEL Conservative starting model for offline MPC work.
%
% The model is intentionally simple. Replace it with an identified model before
% judging controller quality.

if nargin < 1 || isempty(cfg)
    cfg = airdropx_mpc_config("IncludeModel", false);
end

dt = cfg.dt_s;
leak = cfg.integrator.leak;

n = numel(cfg.state_names);
m = numel(cfg.input_names);
A = zeros(n, n);
B = zeros(n, m);

% x = [h_err, vz, v_err, pitch_err, q_dps, mass_err, cg_x_err]
% u = [elevator_delta, throttle_cmd], used as deviation from u_trim.
A(1, 1) = 1.0;
A(1, 2) = dt;

A(2, 2) = 0.90;
A(2, 4) = 0.11;
A(2, 6) = -2.0e-4;
A(2, 7) = -0.10;
B(2, 1) = -1.10;

A(3, 3) = 0.96;
A(3, 4) = -0.030;
A(3, 6) = 4.0e-5;
B(3, 2) = 4.20;

A(4, 4) = 1.0;
A(4, 5) = dt;

A(5, 4) = -1.80;
A(5, 5) = 0.72;
A(5, 7) = -1.50;
B(5, 1) = -16.0;

A(6, 6) = 1.0;
A(7, 7) = 1.0;

model = struct();
model.A = A;
model.B = B;
model.u_trim = cfg.trim.u(:);
model.x_trim = zeros(n, 1);
model.mass_trim_kg = cfg.trim.mass_kg;
model.cg_x_trim_m = cfg.trim.cg_x_m;
model.dt_s = dt;
model.state_names = cfg.state_names;
model.input_names = cfg.input_names;
model.source = "nominal_hand_start";
model.notes = "Use only as a starting model; identify from JSBSim CSV before integration.";
end
