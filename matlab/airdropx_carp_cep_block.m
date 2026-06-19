function [drop_cmd, release_latched, in_window, low_alt_safe, t_to_release_s, ...
          release_n_m, release_e_m, predicted_impact_n_m, predicted_impact_e_m, ...
          miss_distance_m, cep50_to_target_m, actual_release_n_m, actual_release_e_m, actual_release_alt_m, ...
          release_airspeed_mps, release_heading_deg, release_wind_n_mps, release_wind_e_mps, schedule_done] = ...
          airdropx_carp_cep_block(t, pos_n_m, pos_e_m, altitude_m, airspeed_mps, heading_deg, ...
                                  wind_n_mps, wind_e_mps, drop_count, target_n_m, target_e_m, ...
                                  release_window_s, interval_s, drop_total, min_safe_alt_m, ...
                                  gravity_mps2, k_drag, side_wind_gain)
%AIRDROPX_CARP_CEP_BLOCK CARP-window gated 4-drop scheduler and deterministic impact metric.
%
% This is the Simulink-friendly CARP/CEP mode. It latches the first release
% when the aircraft enters the CARP release window and then performs a burst
% schedule at interval_s until drop_total is reached.

%#codegen

if nargin < 10 || isempty(target_n_m)
    cfg = airdropx_sim_params();
    target_n_m = cfg.carp.target_n_m;
end
if nargin < 11 || isempty(target_e_m)
    cfg = airdropx_sim_params();
    target_e_m = cfg.carp.target_e_m;
end
if nargin < 12 || isempty(release_window_s)
    cfg = airdropx_sim_params();
    release_window_s = cfg.carp.release_window_s;
end
if nargin < 13 || isempty(interval_s)
    cfg = airdropx_sim_params();
    interval_s = cfg.carp.interval_s;
end
if nargin < 14 || isempty(drop_total)
    cfg = airdropx_sim_params();
    drop_total = cfg.carp.drop_total;
end
if nargin < 15 || isempty(min_safe_alt_m)
    cfg = airdropx_sim_params();
    min_safe_alt_m = cfg.carp.min_safe_alt_m;
end
if nargin < 16 || isempty(gravity_mps2) || nargin < 17 || isempty(k_drag) || nargin < 18 || isempty(side_wind_gain)
    cfg = airdropx_sim_params();
    if nargin < 16 || isempty(gravity_mps2)
        gravity_mps2 = cfg.ballistics.gravity_mps2;
    end
    if nargin < 17 || isempty(k_drag)
        k_drag = cfg.ballistics.k_drag_calibrated;
    end
    if nargin < 18 || isempty(side_wind_gain)
        side_wind_gain = cfg.ballistics.side_wind_gain;
    end
end

persistent latched next_drop_t command_count prev_drop_count actual_n actual_e actual_h release_v release_heading release_wn release_we last_impact_n last_impact_e last_miss
if isempty(latched)
    latched = 0.0;
    next_drop_t = 0.0;
    command_count = 0.0;
    prev_drop_count = double(drop_count);
    actual_n = 0.0;
    actual_e = 0.0;
    actual_h = 0.0;
    release_v = 0.0;
    release_heading = 0.0;
    release_wn = 0.0;
    release_we = 0.0;
    last_impact_n = 0.0;
    last_impact_e = 0.0;
    last_miss = 9999.0;
end

res = airdropx_carp_release_point(double(target_e_m), double(target_n_m), ...
    double(altitude_m), double(airspeed_mps), double(wind_e_mps), double(wind_n_mps), ...
    0.0, double(heading_deg), double(k_drag), double(gravity_mps2), double(side_wind_gain));

release_n_m = res.release_n_m;
release_e_m = res.release_e_m;

gnd_n = res.ground_speed_n_mps;
gnd_e = res.ground_speed_e_mps;
gnd_speed = max(hypot(gnd_n, gnd_e), 0.5);

to_release_n = release_n_m - double(pos_n_m);
to_release_e = release_e_m - double(pos_e_m);
t_to_release_s = (to_release_n * gnd_n + to_release_e * gnd_e) / (gnd_speed * gnd_speed);
in_window = double(abs(t_to_release_s) <= double(release_window_s));
low_alt_safe = double(double(altitude_m) >= double(min_safe_alt_m));

drop_cmd = 0.0;

if latched < 0.5 && in_window > 0.5 && low_alt_safe > 0.5
    latched = 1.0;
    next_drop_t = double(t);
    command_count = 0.0;
end

if latched > 0.5 && double(t) >= next_drop_t && command_count < double(drop_total) && low_alt_safe > 0.5
    drop_cmd = 1.0;
    command_count = command_count + 1.0;
    next_drop_t = double(t) + double(interval_s);
    actual_n = double(pos_n_m);
    actual_e = double(pos_e_m);
    actual_h = double(altitude_m);
    release_v = double(airspeed_mps);
    release_heading = double(heading_deg);
    release_wn = double(wind_n_mps);
    release_we = double(wind_e_mps);

    impact = airdropx_carp_release_point(double(target_e_m), double(target_n_m), ...
        double(altitude_m), double(airspeed_mps), double(wind_e_mps), double(wind_n_mps), ...
        0.0, double(heading_deg), double(k_drag), double(gravity_mps2), double(side_wind_gain));
    % Translate the same ballistic offset from actual aircraft release point.
    last_impact_n = actual_n + impact.ballistic_n_m + impact.wind_drift_n_m;
    last_impact_e = actual_e + impact.ballistic_e_m + impact.wind_drift_e_m;
    last_miss = hypot(last_impact_n - double(target_n_m), last_impact_e - double(target_e_m));
end

if double(drop_count) < prev_drop_count
    latched = 0.0;
    command_count = 0.0;
end
if command_count >= double(drop_total)
    latched = 0.0;
end
prev_drop_count = double(drop_count);

predicted_impact_n_m = res.predicted_impact_n_m;
predicted_impact_e_m = res.predicted_impact_e_m;
miss_distance_m = last_miss;
cep50_to_target_m = last_miss;
actual_release_n_m = actual_n;
actual_release_e_m = actual_e;
actual_release_alt_m = actual_h;
release_airspeed_mps = release_v;
release_heading_deg = release_heading;
release_wind_n_mps = release_wn;
release_wind_e_mps = release_we;
release_latched = latched;
schedule_done = double(command_count >= double(drop_total));
end
