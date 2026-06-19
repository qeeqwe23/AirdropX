function [n_m, e_m, h_m] = airdropx_cargo_trajectory_point(t, release_n_m, release_e_m, release_alt_m, airspeed_mps, heading_deg, wind_n_mps, wind_e_mps, k_drag)
%AIRDROPX_CARGO_TRAJECTORY_POINT Cargo ballistic trajectory point at time t.
%
% Matches core/cargo_trajectory.py for plotting and verification.

if nargin < 7 || isempty(wind_n_mps), wind_n_mps = 0.0; end
if nargin < 8 || isempty(wind_e_mps), wind_e_mps = 0.0; end

cfg = airdropx_sim_params();
G = cfg.ballistics.gravity_mps2;

if nargin < 9 || isempty(k_drag)
    k_drag = cfg.ballistics.k_drag_calibrated;
end

t = max(double(t), 0.0);
h0 = max(double(release_alt_m), 0.1);
t_fall_total = sqrt(2.0 * h0 / G);
t = min(t, t_fall_total);

heading_rad = deg2rad(double(heading_deg));
vas_n = double(airspeed_mps) * cos(heading_rad);
vas_e = double(airspeed_mps) * sin(heading_rad);

h_m = max(h0 - 0.5 * G * t ^ 2, 0.0);
drag_n = k_drag * vas_n ^ 2 * t;
drag_e = k_drag * vas_e ^ 2 * t;

n_m = double(release_n_m) + vas_n * t - drag_n + double(wind_n_mps) * t;
e_m = double(release_e_m) + vas_e * t - drag_e + double(wind_e_mps) * t;
end
