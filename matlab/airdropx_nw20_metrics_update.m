function metrics = airdropx_nw20_metrics_update(t, altitude_m, vz_up_mps, elevator_delta, drop_count, reset, h_ref, dt_s, altitude_band_m, vz_band_mps, elevator_sat_abs)
%AIRDROPX_NW20_METRICS_UPDATE Online scalar metrics for NW20 height task.
% Feed this from a MATLAB Function block or call from logged data.

%#codegen

persistent n sum_err sum_err2 max_abs_err min_alt sat_steps drop_seen post_drop_time settling_time prev_drop_count
if nargin < 7 || isempty(h_ref)
    cfg = airdropx_sim_params();
    h_ref = cfg.control.target_altitude_m;
end
if nargin < 8 || isempty(dt_s)
    cfg = airdropx_sim_params();
    dt_s = cfg.sim.dt_s;
end
if nargin < 9 || isempty(altitude_band_m)
    cfg = airdropx_sim_params();
    altitude_band_m = cfg.metrics.settling_altitude_band_m;
end
if nargin < 10 || isempty(vz_band_mps)
    cfg = airdropx_sim_params();
    vz_band_mps = cfg.metrics.settling_vz_band_mps;
end
if nargin < 11 || isempty(elevator_sat_abs)
    cfg = airdropx_sim_params();
    elevator_sat_abs = cfg.metrics.elevator_saturation_abs;
end
if isempty(n) || reset > 0.5
    n = 0.0;
    sum_err = 0.0;
    sum_err2 = 0.0;
    max_abs_err = 0.0;
    min_alt = 1.0e9;
    sat_steps = 0.0;
    drop_seen = 0.0;
    post_drop_time = 0.0;
    settling_time = 0.0;
    prev_drop_count = double(drop_count);
end

e = double(altitude_m) - h_ref;
n = n + 1.0;
sum_err = sum_err + e;
sum_err2 = sum_err2 + e * e;
max_abs_err = max(max_abs_err, abs(e));
min_alt = min(min_alt, double(altitude_m));

if abs(double(elevator_delta)) >= double(elevator_sat_abs) * 0.99
    sat_steps = sat_steps + 1.0;
end

if double(drop_count) > prev_drop_count
    drop_seen = 1.0;
    post_drop_time = 0.0;
end
prev_drop_count = double(drop_count);

if drop_seen > 0.5
    post_drop_time = post_drop_time + max(double(t) * 0.0 + double(dt_s), 0.0);
    if settling_time == 0.0 && abs(e) < double(altitude_band_m) && abs(double(vz_up_mps)) < double(vz_band_mps)
        settling_time = post_drop_time;
    end
end

metrics.h_err_mean = sum_err / max(n, 1.0);
metrics.h_err_rms = sqrt(sum_err2 / max(n, 1.0));
metrics.h_err_max = max_abs_err;
metrics.min_altitude = min_alt;
metrics.elevator_sat_rate = sat_steps / max(n, 1.0);
metrics.settling_time = settling_time;
metrics.drop_count = double(drop_count);
end
