function [drop_cmd, next_drop_index, schedule_done] = airdropx_four_drop_schedule(t, start_s, interval_s, pulse_s, drop_total)
%AIRDROPX_FOUR_DROP_SCHEDULE Fixed-time 4-drop pulse scheduler.
% Mirrors the NW20 fixed schedule in ui/simulation_thread.py:
% first drop at start_s, then interval_s spacing, four drops total.

if nargin < 2 || isempty(start_s)
    cfg = airdropx_sim_params();
    start_s = cfg.fixed_drop.start_s;
end
if nargin < 3 || isempty(interval_s)
    cfg = airdropx_sim_params();
    interval_s = cfg.fixed_drop.interval_s;
end
if nargin < 4 || isempty(pulse_s)
    cfg = airdropx_sim_params();
    pulse_s = cfg.fixed_drop.pulse_s;
end
if nargin < 5 || isempty(drop_total)
    cfg = airdropx_sim_params();
    drop_total = cfg.fixed_drop.drop_total;
end

t = double(t);
start_s = double(start_s);
interval_s = double(interval_s);
pulse_s = max(double(pulse_s), eps);
drop_total = max(0.0, floor(double(drop_total)));

drop_cmd = 0.0;
next_drop_index = 0.0;

for i = 0:99
    if double(i) >= drop_total
        break;
    end
    ti = start_s + interval_s * double(i);
    if t >= ti && t < ti + pulse_s
        drop_cmd = 1.0;
        next_drop_index = double(i + 1);
        break;
    end
end

schedule_done = double(drop_total <= 0.0 || t >= start_s + interval_s * (drop_total - 1.0) + pulse_s);
end
