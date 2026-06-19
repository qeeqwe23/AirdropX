function [elevator_delta, throttle, state, diag] = airdropx_pd_controller(h_ref, h_current, v_z_up, v_true, delta_m_signal, mass, state, gains)
%AIRDROPX_PD_CONTROLLER PD baseline controller for Simulink MATLAB Function blocks.
%
% Port of core/pd_baseline_controller.py. Keep state as a struct between
% calls, or use a Unit Delay/Memory block around the returned state.

if nargin < 5 || isempty(delta_m_signal), delta_m_signal = 0.0; end
if nargin < 6 || isempty(mass), mass = 0.0; end
if nargin < 7 || isempty(state), state = airdropx_pd_init_state(); end
if nargin < 8 || isempty(gains), gains = airdropx_pd_default_gains(); end

if delta_m_signal > 0.0 && mass > 0.0
    bias_increment = gains.K_mass * double(delta_m_signal) / double(mass);
    max_step = gains.bias_rate_limit * 100.0;
    bias_increment = min(max(bias_increment, -max_step), max_step);
    state.drop_trim_bias = state.drop_trim_bias + bias_increment;
end

e_h = double(h_current) - double(h_ref);
u_pd = gains.Kp * e_h + gains.Kd * double(v_z_up);
u_total = u_pd + state.drop_trim_bias;

du = u_total - state.u_prev;
du_clamped = min(max(du, -gains.u_rate_limit), gains.u_rate_limit);
u_rate_limited = state.u_prev + du_clamped;

elevator_delta = min(max(u_rate_limited, -gains.u_limit), gains.u_limit);
state.u_prev = elevator_delta;

if gains.throttle_kp > 0.0
    v_ref = gains.v_ref_mps;
    throttle = gains.throttle_fixed + gains.throttle_kp * (v_ref - double(v_true));
    throttle = min(max(throttle, 0.0), 1.0);
else
    throttle = gains.throttle_fixed;
end

state.total_steps = state.total_steps + 1.0;
if abs(u_rate_limited) >= gains.u_limit * 0.99
    state.saturated_steps = state.saturated_steps + 1.0;
end

diag.e_h = e_h;
diag.v_z_up = double(v_z_up);
diag.u_pd = u_pd;
diag.drop_trim_bias = state.drop_trim_bias;
diag.u_total = u_total;
diag.u_out = elevator_delta;
diag.u_throttle = throttle;
diag.saturated = abs(u_rate_limited) >= gains.u_limit * 0.99;
end

function state = airdropx_pd_init_state()
state.u_prev = 0.0;
state.drop_trim_bias = 0.0;
state.total_steps = 0.0;
state.saturated_steps = 0.0;
end

function gains = airdropx_pd_default_gains()
cfg = airdropx_sim_params();
gains = cfg.control.pd_gains;
end
