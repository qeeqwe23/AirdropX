function model = airdropx_mpc_outer_pd_nominal_model(cfg)
%AIRDROPX_MPC_OUTER_PD_NOMINAL_MODEL Local outer-loop model.
%
% The first input is pitch reference, not elevator. It represents the expected
% closed inner-loop pitch response.

if nargin < 1 || isempty(cfg)
    cfg = airdropx_mpc_outer_pd_config();
end

dt = cfg.dt_s;
n = numel(cfg.state_names);
m = numel(cfg.input_names);
A = zeros(n, n);
B = zeros(n, m);

% x = [h_err, vz, v_err, pitch_err, q_dps, mass_err, cg_x_err]
% u = [pitch_ref_deg, throttle_cmd], used as deviation from u_trim.
A(1, 1) = 1.0;
A(1, 2) = dt;

A(2, 2) = 0.90;
A(2, 4) = 0.10;
A(2, 5) = 0.012;
A(2, 6) = -1.8e-4;
A(2, 7) = -0.09;
B(2, 1) = 0.075;

A(3, 3) = 0.96;
A(3, 4) = -0.032;
A(3, 6) = 4.0e-5;
B(3, 1) = -0.040;
B(3, 2) = 4.15;

A(4, 4) = 0.80;
A(4, 5) = 0.045;
B(4, 1) = 0.23;

A(5, 4) = -0.85;
A(5, 5) = 0.48;
A(5, 7) = -1.20;
B(5, 1) = 2.60;

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
model.source = "outer_mpc_pitch_reference_nominal";
model.notes = "Outer MPC commands pitch_ref_deg and throttle; PD inner loop commands elevator.";
end
