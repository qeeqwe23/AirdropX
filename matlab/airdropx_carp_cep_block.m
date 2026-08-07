function [drop_cmd, release_latched, in_window, low_alt_safe, t_to_release_s, ...
          release_n_m, release_e_m, predicted_impact_n_m, predicted_impact_e_m, ...
          miss_distance_m, cep50_to_target_m, actual_release_n_m, actual_release_e_m, actual_release_alt_m, ...
          release_airspeed_mps, release_heading_deg, release_wind_n_mps, release_wind_e_mps, schedule_done] = ...
          airdropx_carp_cep_block(t, pos_n_m, pos_e_m, altitude_m, airspeed_mps, heading_deg, ...
                                  wind_n_mps, wind_e_mps, drop_count, target_n_m, target_e_m, ...
                                  release_window_s, interval_s, drop_total, min_safe_alt_m, ...
                                  gravity_mps2, k_drag, side_wind_gain, ...
                                  target_offset_n_1_m, target_offset_n_2_m, ...
                                  target_offset_n_3_m, target_offset_n_4_m, ...
                                  target_offset_e_1_m, target_offset_e_2_m, ...
                                  target_offset_e_3_m, target_offset_e_4_m, release_delay_s)
%AIRDROPX_CARP_CEP_BLOCK CARP-window gated 4-drop scheduler and deterministic impact metric.
%
% This is the Simulink-friendly CARP/CEP mode. It latches the first release
% at the computed release point and then performs a burst schedule at
% interval_s until drop_total is reached.

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
if nargin < 19 || isempty(target_offset_n_1_m) || nargin < 20 || isempty(target_offset_n_2_m) || ...
        nargin < 21 || isempty(target_offset_n_3_m) || nargin < 22 || isempty(target_offset_n_4_m) || ...
        nargin < 23 || isempty(target_offset_e_1_m) || nargin < 24 || isempty(target_offset_e_2_m) || ...
        nargin < 25 || isempty(target_offset_e_3_m) || nargin < 26 || isempty(target_offset_e_4_m) || ...
        nargin < 27 || isempty(release_delay_s)
    if nargin < 19 || isempty(target_offset_n_1_m), target_offset_n_1_m = 0.8; end
    if nargin < 20 || isempty(target_offset_n_2_m), target_offset_n_2_m = 1.6; end
    if nargin < 21 || isempty(target_offset_n_3_m), target_offset_n_3_m = 2.4; end
    if nargin < 22 || isempty(target_offset_n_4_m), target_offset_n_4_m = 3.8; end
    if nargin < 23 || isempty(target_offset_e_1_m), target_offset_e_1_m = 0.0; end
    if nargin < 24 || isempty(target_offset_e_2_m), target_offset_e_2_m = 0.0; end
    if nargin < 25 || isempty(target_offset_e_3_m), target_offset_e_3_m = 0.0; end
    if nargin < 26 || isempty(target_offset_e_4_m), target_offset_e_4_m = 0.0; end
    if nargin < 27 || isempty(release_delay_s), release_delay_s = 0.0; end
end

persistent latched next_drop_t command_count prev_drop_count prev_t_to_release ...
    prev_pos_n prev_pos_e prev_alt prev_airspeed prev_heading prev_wn prev_we ...
    actual_n actual_e actual_h release_v release_heading release_wn release_we ...
    last_impact_n last_impact_e last_miss
if isempty(latched)
    latched = 0.0;
    next_drop_t = 0.0;
    command_count = 0.0;
    prev_drop_count = double(drop_count);
    prev_t_to_release = Inf;
    prev_pos_n = double(pos_n_m);
    prev_pos_e = double(pos_e_m);
    prev_alt = double(altitude_m);
    prev_airspeed = double(airspeed_mps);
    prev_heading = double(heading_deg);
    prev_wn = double(wind_n_mps);
    prev_we = double(wind_e_mps);
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

next_drop_index = min(max(floor(command_count) + 1.0, 1.0), double(drop_total));
[active_target_n_m, active_target_e_m] = local_target_for_drop( ...
    double(target_n_m), double(target_e_m), next_drop_index, ...
    double(target_offset_n_1_m), double(target_offset_n_2_m), ...
    double(target_offset_n_3_m), double(target_offset_n_4_m), ...
    double(target_offset_e_1_m), double(target_offset_e_2_m), ...
    double(target_offset_e_3_m), double(target_offset_e_4_m));

res = airdropx_carp_release_point(active_target_e_m, active_target_n_m, ...
    double(altitude_m), double(airspeed_mps), double(wind_e_mps), double(wind_n_mps), ...
    double(release_delay_s), double(heading_deg), double(k_drag), double(gravity_mps2), double(side_wind_gain));

release_n_m = res.release_n_m;
release_e_m = res.release_e_m;

gnd_n = res.ground_speed_n_mps;
gnd_e = res.ground_speed_e_mps;
gnd_speed = max(hypot(gnd_n, gnd_e), 0.5);

to_release_n = release_n_m - double(pos_n_m);
to_release_e = release_e_m - double(pos_e_m);
t_to_release_s = (to_release_n * gnd_n + to_release_e * gnd_e) / (gnd_speed * gnd_speed);
release_window_s = max(double(release_window_s), 0.0);
near_release = abs(t_to_release_s) <= release_window_s;
crossed_release = isfinite(prev_t_to_release) && prev_t_to_release > 0.0 && t_to_release_s <= 0.0;
in_window = double(near_release || crossed_release);
release_ready = crossed_release || (near_release && t_to_release_s <= 0.0);
low_alt_safe = double(double(altitude_m) >= double(min_safe_alt_m));

drop_cmd = 0.0;
sample_n = double(pos_n_m);
sample_e = double(pos_e_m);
sample_h = double(altitude_m);
sample_v = double(airspeed_mps);
sample_heading = double(heading_deg);
sample_wn = double(wind_n_mps);
sample_we = double(wind_e_mps);

if crossed_release && isfinite(prev_t_to_release)
    denom = prev_t_to_release - double(t_to_release_s);
    if abs(denom) > 1.0e-9
        alpha = min(max(prev_t_to_release / denom, 0.0), 1.0);
        sample_n = prev_pos_n + alpha * (double(pos_n_m) - prev_pos_n);
        sample_e = prev_pos_e + alpha * (double(pos_e_m) - prev_pos_e);
        sample_h = prev_alt + alpha * (double(altitude_m) - prev_alt);
        sample_v = prev_airspeed + alpha * (double(airspeed_mps) - prev_airspeed);
        sample_heading = prev_heading + alpha * (double(heading_deg) - prev_heading);
        sample_wn = prev_wn + alpha * (double(wind_n_mps) - prev_wn);
        sample_we = prev_we + alpha * (double(wind_e_mps) - prev_we);
    end
end

if latched < 0.5 && release_ready && low_alt_safe > 0.5
    latched = 1.0;
    next_drop_t = double(t);
    command_count = 0.0;
end

if latched > 0.5 && double(t) >= next_drop_t && command_count < double(drop_total) && low_alt_safe > 0.5
    drop_index = min(max(floor(command_count) + 1.0, 1.0), double(drop_total));
    [drop_target_n_m, drop_target_e_m] = local_target_for_drop( ...
        double(target_n_m), double(target_e_m), drop_index, ...
        double(target_offset_n_1_m), double(target_offset_n_2_m), ...
        double(target_offset_n_3_m), double(target_offset_n_4_m), ...
        double(target_offset_e_1_m), double(target_offset_e_2_m), ...
        double(target_offset_e_3_m), double(target_offset_e_4_m));
    drop_cmd = 1.0;
    command_count = command_count + 1.0;
    next_drop_t = double(t) + double(interval_s);
    actual_n = sample_n;
    actual_e = sample_e;
    actual_h = sample_h;
    release_v = sample_v;
    release_heading = sample_heading;
    release_wn = sample_wn;
    release_we = sample_we;

    impact = airdropx_carp_release_point(drop_target_e_m, drop_target_n_m, ...
        sample_h, sample_v, sample_we, sample_wn, ...
        double(release_delay_s), sample_heading, double(k_drag), double(gravity_mps2), double(side_wind_gain));
    % Translate the same ballistic offset from actual aircraft release point.
    last_impact_n = actual_n + impact.ballistic_n_m + impact.wind_drift_n_m;
    last_impact_e = actual_e + impact.ballistic_e_m + impact.wind_drift_e_m;
    last_miss = hypot(last_impact_n - drop_target_n_m, last_impact_e - drop_target_e_m);
end

if double(drop_count) < prev_drop_count
    latched = 0.0;
    command_count = 0.0;
    prev_t_to_release = Inf;
end
if command_count >= double(drop_total)
    latched = 0.0;
end
prev_drop_count = double(drop_count);
prev_t_to_release = double(t_to_release_s);
prev_pos_n = double(pos_n_m);
prev_pos_e = double(pos_e_m);
prev_alt = double(altitude_m);
prev_airspeed = double(airspeed_mps);
prev_heading = double(heading_deg);
prev_wn = double(wind_n_mps);
prev_we = double(wind_e_mps);

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

function [target_n, target_e] = local_target_for_drop(center_n, center_e, drop_index, ...
    offset_n_1, offset_n_2, offset_n_3, offset_n_4, ...
    offset_e_1, offset_e_2, offset_e_3, offset_e_4)
target_n = double(center_n);
target_e = double(center_e);
idx = max(1, round(double(drop_index)));
switch idx
    case 1
        target_n = target_n + double(offset_n_1);
        target_e = target_e + double(offset_e_1);
    case 2
        target_n = target_n + double(offset_n_2);
        target_e = target_e + double(offset_e_2);
    case 3
        target_n = target_n + double(offset_n_3);
        target_e = target_e + double(offset_e_3);
    otherwise
        target_n = target_n + double(offset_n_4);
        target_e = target_e + double(offset_e_4);
end
end
