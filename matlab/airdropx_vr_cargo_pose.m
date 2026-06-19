function [cargo1_translation, cargo2_translation, cargo3_translation, cargo4_translation, ...
          cargo1_scale, cargo2_scale, cargo3_scale, cargo4_scale] = ...
          airdropx_vr_cargo_pose(t, pos_n_m, pos_e_m, altitude_m, airspeed_mps, heading_deg, wind_n_mps, wind_e_mps, drop_count, gravity_mps2, k_drag)
%AIRDROPX_VR_CARGO_POSE Four cargo VRML poses driven by drop_count edges.
%
% Inputs are the S-Function telemetry plus Clock. A cargo is released when
% drop_count increases. Before release it is hidden under the ground and
% scaled to zero. After release it follows the same calibrated trajectory
% used by the MATLAB CARP/CEP helpers.

if nargin < 10 || isempty(gravity_mps2) || nargin < 11 || isempty(k_drag)
    cfg = airdropx_sim_params();
    if nargin < 10 || isempty(gravity_mps2)
        gravity_mps2 = cfg.ballistics.gravity_mps2;
    end
    if nargin < 11 || isempty(k_drag)
        k_drag = cfg.ballistics.k_drag_calibrated;
    end
end

persistent prev_drop_count release_t release_n release_e release_h release_v release_heading release_wind_n release_wind_e active

if isempty(prev_drop_count)
    prev_drop_count = 0.0;
    release_t = zeros(4, 1);
    release_n = zeros(4, 1);
    release_e = zeros(4, 1);
    release_h = zeros(4, 1);
    release_v = zeros(4, 1);
    release_heading = zeros(4, 1);
    release_wind_n = zeros(4, 1);
    release_wind_e = zeros(4, 1);
    active = false(4, 1);
end

dc = floor(double(drop_count) + 1.0e-9);
prev_dc = floor(double(prev_drop_count) + 1.0e-9);

if dc < prev_dc
    active(:) = false;
    release_t(:) = 0.0;
end

if dc > prev_dc
    first_new = max(prev_dc + 1, 1);
    last_new = min(dc, 4);
    for k = first_new:last_new
        release_t(k) = double(t);
        release_n(k) = double(pos_n_m);
        release_e(k) = double(pos_e_m);
        release_h(k) = max(double(altitude_m), 0.1);
        release_v(k) = double(airspeed_mps);
        release_heading(k) = double(heading_deg);
        release_wind_n(k) = double(wind_n_mps);
        release_wind_e(k) = double(wind_e_mps);
        active(k) = true;
    end
end

prev_drop_count = double(drop_count);

[cargo1_translation, cargo1_scale] = cargo_pose(1, double(t), active, release_t, release_n, release_e, release_h, release_v, release_heading, release_wind_n, release_wind_e, double(gravity_mps2), double(k_drag));
[cargo2_translation, cargo2_scale] = cargo_pose(2, double(t), active, release_t, release_n, release_e, release_h, release_v, release_heading, release_wind_n, release_wind_e, double(gravity_mps2), double(k_drag));
[cargo3_translation, cargo3_scale] = cargo_pose(3, double(t), active, release_t, release_n, release_e, release_h, release_v, release_heading, release_wind_n, release_wind_e, double(gravity_mps2), double(k_drag));
[cargo4_translation, cargo4_scale] = cargo_pose(4, double(t), active, release_t, release_n, release_e, release_h, release_v, release_heading, release_wind_n, release_wind_e, double(gravity_mps2), double(k_drag));
end

function [translation, scale] = cargo_pose(k, t, active, release_t, release_n, release_e, release_h, release_v, release_heading, release_wind_n, release_wind_e, gravity_mps2, k_drag)
if ~active(k)
    translation = [0.0; -1000.0; 0.0];
    scale = [0.0; 0.0; 0.0];
    return;
end

[n_m, e_m, h_m] = cargo_trajectory(t - release_t(k), release_n(k), release_e(k), release_h(k), ...
    release_v(k), release_heading(k), release_wind_n(k), release_wind_e(k), gravity_mps2, k_drag);

translation = [e_m; h_m; -n_m];
scale = [1.0; 1.0; 1.0];
end

function [n_m, e_m, h_m] = cargo_trajectory(t, release_n_m, release_e_m, release_alt_m, airspeed_mps, heading_deg, wind_n_mps, wind_e_mps, gravity_mps2, k_drag)
G = double(gravity_mps2);
t = max(double(t), 0.0);
h0 = max(double(release_alt_m), 0.1);
t_fall_total = sqrt(2.0 * h0 / G);
t = min(t, t_fall_total);

heading_rad = deg2rad(double(heading_deg));
vas_n = double(airspeed_mps) * cos(heading_rad);
vas_e = double(airspeed_mps) * sin(heading_rad);

h_m = max(h0 - 0.5 * G * t ^ 2, 0.0);
drag_n = double(k_drag) * vas_n ^ 2 * t;
drag_e = double(k_drag) * vas_e ^ 2 * t;

n_m = double(release_n_m) + vas_n * t - drag_n + double(wind_n_mps) * t;
e_m = double(release_e_m) + vas_e * t - drag_e + double(wind_e_mps) * t;
end
