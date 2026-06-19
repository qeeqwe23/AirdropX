function [elevator_delta, throttle, state, diag] = airdropx_pd_controller(h_ref, h_current, v_z_up, v_true, delta_m_signal, mass, state, gains, pitch_deg)
%AIRDROPX_PD_CONTROLLER PD baseline controller for Simulink MATLAB Function blocks.
%
% Port of core/pd_baseline_controller.py. Keep state as a struct between
% calls, or use a Unit Delay/Memory block around the returned state.

if nargin < 5 || isempty(delta_m_signal), delta_m_signal = 0.0; end
if nargin < 6 || isempty(mass), mass = 0.0; end
if nargin < 7 || isempty(state), state = airdropx_pd_init_state(); end
if nargin < 8 || isempty(gains), gains = airdropx_pd_default_gains(); end
if nargin < 9 || isempty(pitch_deg), pitch_deg = 0.0; end

if delta_m_signal > 0.0 && mass > 0.0
    bias_increment = gains.K_mass * double(delta_m_signal) / double(mass);
    max_step = gains.bias_rate_limit * 100.0;
    bias_increment = min(max(bias_increment, -max_step), max_step);
    state.drop_trim_bias = state.drop_trim_bias + bias_increment;
end

e_h = double(h_current) - double(h_ref);
u_pd = gains.Kp * e_h + gains.Kd * double(v_z_up);
u_pitch = 0.0;
u_pitch_rate = 0.0;
if isfield(gains, "pitch_kp") && gains.pitch_kp > 0.0
    pitch_ref_deg = local_gain_or_default(gains, "pitch_ref_deg", 0.0);
    pitch_limit = local_gain_or_default(gains, "pitch_limit", 0.0);
    u_pitch = gains.pitch_kp * (double(pitch_deg) - pitch_ref_deg);
    if pitch_limit > 0.0
        u_pitch = min(max(u_pitch, -pitch_limit), pitch_limit);
    end
end
if isfield(gains, "pitch_rate_kd") && gains.pitch_rate_kd > 0.0
    if state.pitch_initialized == 0.0
        state.prev_pitch_deg = double(pitch_deg);
        state.pitch_initialized = 1.0;
    else
        dt_s = max(local_gain_or_default(gains, "dt_s", 1.0 / 120.0), eps);
        pitch_rate_dps = (double(pitch_deg) - state.prev_pitch_deg) / dt_s;
        pitch_rate_limit = local_gain_or_default(gains, "pitch_rate_limit", 0.0);
        u_pitch_rate = gains.pitch_rate_kd * pitch_rate_dps;
        if pitch_rate_limit > 0.0
            u_pitch_rate = min(max(u_pitch_rate, -pitch_rate_limit), pitch_rate_limit);
        end
        state.prev_pitch_deg = double(pitch_deg);
    end
end
u_total = u_pd + state.drop_trim_bias + u_pitch + u_pitch_rate;

du = u_total - state.u_prev;
du_clamped = min(max(du, -gains.u_rate_limit), gains.u_rate_limit);
u_rate_limited = state.u_prev + du_clamped;

elevator_delta = min(max(u_rate_limited, -gains.u_limit), gains.u_limit);
state.u_prev = elevator_delta;

if gains.throttle_kp > 0.0
    v_ref = gains.v_ref_mps;
    throttle = gains.throttle_fixed + gains.throttle_kp * (v_ref - double(v_true));
else
    throttle = gains.throttle_fixed;
end
if isfield(gains, "throttle_alt_kp") && gains.throttle_alt_kp > 0.0
    alt_error = double(h_ref) - double(h_current);
    throttle = throttle + gains.throttle_alt_kp * alt_error;
end
if isfield(gains, "throttle_vz_kd") && gains.throttle_vz_kd > 0.0
    throttle = throttle - gains.throttle_vz_kd * double(v_z_up);
end
throttle = min(max(throttle, 0.0), 1.0);

state.total_steps = state.total_steps + 1.0;
if abs(u_rate_limited) >= gains.u_limit * 0.99
    state.saturated_steps = state.saturated_steps + 1.0;
end

diag.e_h = e_h;
diag.v_z_up = double(v_z_up);
diag.u_pd = u_pd;
diag.u_pitch = u_pitch;
diag.u_pitch_rate = u_pitch_rate;
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
state.prev_pitch_deg = 0.0;
state.pitch_initialized = 0.0;
end

function gains = airdropx_pd_default_gains()
cfg = airdropx_sim_params();
gains = cfg.control.pd_gains;
end

function value = local_gain_or_default(gains, name, defaultValue)
if isfield(gains, name)
    value = gains.(name);
else
    value = defaultValue;
end
end
