function out = airdropx_carp_release_point(target_e_m, target_n_m, altitude_m, airspeed_mps, wind_e_mps, wind_n_mps, release_delay_s, heading_deg, k_drag, gravity_mps2, side_wind_gain)
%AIRDROPX_CARP_RELEASE_POINT CARP v2 release point solver.
%
% This is a MATLAB port of core/carp_release_solver_v2.py.
% Units are SI. Heading convention: 0 deg = north, 90 deg = east.

if nargin < 5 || isempty(wind_e_mps), wind_e_mps = 0.0; end
if nargin < 6 || isempty(wind_n_mps), wind_n_mps = 0.0; end
if nargin < 7 || isempty(release_delay_s), release_delay_s = 0.0; end
if nargin < 8 || isempty(heading_deg), heading_deg = 0.0; end

if nargin < 9 || isempty(k_drag) || nargin < 10 || isempty(gravity_mps2) || nargin < 11 || isempty(side_wind_gain)
    cfg = airdropx_sim_params();
    if nargin < 9 || isempty(k_drag)
        k_drag = cfg.ballistics.k_drag_calibrated;
    end
    if nargin < 10 || isempty(gravity_mps2)
        gravity_mps2 = cfg.ballistics.gravity_mps2;
    end
    if nargin < 11 || isempty(side_wind_gain)
        side_wind_gain = cfg.ballistics.side_wind_gain;
    end
end

G = double(gravity_mps2);
h = max(double(altitude_m), 0.1);
t_fall = sqrt(2.0 * h / G);

heading_rad = deg2rad(double(heading_deg));
vas_n = double(airspeed_mps) * cos(heading_rad);
vas_e = double(airspeed_mps) * sin(heading_rad);

drag_n = k_drag * vas_n ^ 2 * t_fall;
drag_e = k_drag * vas_e ^ 2 * t_fall;

ballistic_n = vas_n * t_fall - drag_n;
ballistic_e = vas_e * t_fall - drag_e;

wind_drift_n = double(wind_n_mps) * t_fall;
wind_drift_e = double(side_wind_gain) * double(wind_e_mps) * t_fall;

gnd_n = vas_n + double(wind_n_mps);
gnd_e = vas_e + double(wind_e_mps);

delay_n = gnd_n * double(release_delay_s);
delay_e = gnd_e * double(release_delay_s);

release_n = double(target_n_m) - ballistic_n - wind_drift_n - delay_n;
release_e = double(target_e_m) - ballistic_e - wind_drift_e - delay_e;

out.release_e_m = release_e;
out.release_n_m = release_n;
out.predicted_impact_e_m = release_e + ballistic_e + wind_drift_e + delay_e;
out.predicted_impact_n_m = release_n + ballistic_n + wind_drift_n + delay_n;
out.ballistic_distance_m = hypot(ballistic_n, ballistic_e);
out.ballistic_n_m = ballistic_n;
out.ballistic_e_m = ballistic_e;
out.time_to_impact_s = t_fall;
out.delay_distance_m = hypot(delay_n, delay_e);
out.wind_drift_n_m = wind_drift_n;
out.wind_drift_e_m = wind_drift_e;
out.ground_speed_n_mps = gnd_n;
out.ground_speed_e_mps = gnd_e;
out.k_drag_used = k_drag;
out.model_version_id = 2.0;
end
