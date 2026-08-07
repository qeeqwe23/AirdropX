function cfg = airdropx_mpc_outer_pd_setup_workspace(varargin)
%AIRDROPX_MPC_OUTER_PD_SETUP_WORKSPACE Prepare workspace for outer/inner model.

opts = local_options(varargin{:});

thisFile = mfilename("fullpath");
outerDir = string(fileparts(thisFile));
matlabDir = string(fileparts(outerDir));
projectRoot = string(fileparts(matlabDir));

addpath(char(matlabDir));
addpath(char(fullfile(matlabDir, "mpc")));
addpath(char(outerDir));
addpath(char(fullfile(matlabDir, "sfunc_jsbsim")));
addpath(char(fullfile(matlabDir, "vr")));

cfg = airdropx_sim_params( ...
    "ProjectRoot", projectRoot, ...
    "Model", opts.Model, ...
    "InitialAirspeedMps", local_finite_or(opts.InitialAirspeedMps, opts.TargetAirspeedMps), ...
    "InitialAltitudeM", opts.InitialAltitudeM, ...
    "InitialPitchDeg", local_finite_or(opts.InitialPitchDeg, opts.TargetPitchDeg), ...
    "InitialFlightPathDeg", opts.InitialFlightPathDeg, ...
    "InitialHeadingDeg", opts.InitialHeadingDeg, ...
    "AssignBase", true);

cfg.sim.stop_time_s = double(opts.StopTimeS);
cfg.control.target_altitude_m = double(opts.TargetAltitudeM);
cfg.control.pd_gains.v_ref_mps = double(opts.TargetAirspeedMps);
cfg.control.pd_gains.pitch_ref_deg = double(opts.TargetPitchDeg);
cfg.drop_mode = double(opts.DropMode);
cfg.carp.target_n_m = double(opts.CarpTargetNorthM);
cfg.carp.target_e_m = double(opts.CarpTargetEastM);
cfg.carp.release_window_s = double(opts.CarpReleaseWindowS);
cfg.carp.interval_s = double(opts.CarpIntervalS);
cfg.carp.drop_total = double(opts.CarpDropTotal);
cfg.carp.min_safe_alt_m = double(opts.CarpMinSafeAltitudeM);
cfg.carp.target_offset_n_m = double(opts.CarpTargetOffsetNorthM(:));
cfg.carp.target_offset_e_m = double(opts.CarpTargetOffsetEastM(:));
cfg.carp.release_delay_s = double(opts.CarpReleaseDelayS);
if isfinite(double(opts.BallisticKDrag))
    cfg.ballistics.k_drag_calibrated = double(opts.BallisticKDrag);
else
    cfg.ballistics.k_drag_calibrated = cfg.ballistics.k_drag_calibrated * double(opts.BallisticKDragScale);
end
if isfinite(double(opts.InitialElevatorDelta))
    cfg.control.initial_elevator_delta = double(opts.InitialElevatorDelta);
end
if isfinite(double(opts.InitialThrottleCmd))
    cfg.control.initial_throttle_cmd = double(opts.InitialThrottleCmd);
end

[referenceMassKg, referenceCgXM] = local_reference_mass_cg(cfg);
assignin("base", "airdropx_cfg", cfg);
assignin("base", "airdropx_stop_time_s", cfg.sim.stop_time_s);
assignin("base", "airdropx_target_altitude_m", cfg.control.target_altitude_m);
assignin("base", "airdropx_pd_v_ref_mps", cfg.control.pd_gains.v_ref_mps);
assignin("base", "airdropx_pd_pitch_ref_deg", cfg.control.pd_gains.pitch_ref_deg);
assignin("base", "airdropx_initial_elevator_delta", cfg.control.initial_elevator_delta);
assignin("base", "airdropx_initial_throttle_cmd", cfg.control.initial_throttle_cmd);
assignin("base", "airdropx_drop_mode", cfg.drop_mode);
assignin("base", "airdropx_carp_target_n_m", cfg.carp.target_n_m);
assignin("base", "airdropx_carp_target_e_m", cfg.carp.target_e_m);
assignin("base", "airdropx_carp_release_window_s", cfg.carp.release_window_s);
assignin("base", "airdropx_carp_interval_s", cfg.carp.interval_s);
assignin("base", "airdropx_carp_drop_total", cfg.carp.drop_total);
assignin("base", "airdropx_carp_min_safe_alt_m", cfg.carp.min_safe_alt_m);
assignin("base", "airdropx_carp_target_offset_n_m", cfg.carp.target_offset_n_m);
assignin("base", "airdropx_carp_target_offset_e_m", cfg.carp.target_offset_e_m);
assignin("base", "airdropx_carp_release_delay_s", cfg.carp.release_delay_s);
for i = 1:4
    assignin("base", sprintf("airdropx_carp_target_offset_n_%d_m", i), local_index_or_zero(cfg.carp.target_offset_n_m, i));
    assignin("base", sprintf("airdropx_carp_target_offset_e_%d_m", i), local_index_or_zero(cfg.carp.target_offset_e_m, i));
end
assignin("base", "airdropx_mpc_reference_mass_kg", referenceMassKg);
assignin("base", "airdropx_mpc_reference_cg_x_m", referenceCgXM);
assignin("base", "airdropx_mpc_control_altitude_bias_m", double(opts.ControlAltitudeBiasM));
assignin("base", "airdropx_ballistics_k_drag", cfg.ballistics.k_drag_calibrated);

if ~isempty(opts.ConfigOverrides)
    if ~isstruct(opts.ConfigOverrides)
        error("ConfigOverrides must be a struct.");
    end
    assignin("base", "airdropx_mpc_outer_pd_config_overrides", opts.ConfigOverrides);
end
end

function opts = local_options(varargin)
opts.Model = "airdropx_mpc_outer_pd_closed_loop";
opts.StopTimeS = 22.0;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.ControlAltitudeBiasM = 0.95;
opts.DropMode = 1.0;
opts.CarpTargetNorthM = 1000.0;
opts.CarpTargetEastM = 0.0;
opts.CarpReleaseWindowS = 0.7;
opts.CarpIntervalS = 0.5;
opts.CarpDropTotal = 4.0;
opts.CarpMinSafeAltitudeM = 15.0;
opts.CarpTargetOffsetNorthM = zeros(4, 1);
opts.CarpTargetOffsetEastM = zeros(4, 1);
opts.CarpReleaseDelayS = 0.0;
opts.BallisticKDragScale = 1.0;
opts.BallisticKDrag = NaN;
opts.InitialAirspeedMps = 55.0;
opts.InitialAltitudeM = NaN;
opts.InitialPitchDeg = NaN;
opts.InitialFlightPathDeg = 2.4;
opts.InitialHeadingDeg = 0.0;
opts.InitialElevatorDelta = 0.0;
opts.InitialThrottleCmd = 0.80;
opts.ConfigOverrides = [];

if mod(numel(varargin), 2) ~= 0
    error("Options must be name-value pairs.");
end

for i = 1:2:numel(varargin)
    name = string(varargin{i});
    value = varargin{i + 1};
    if ~isfield(opts, name)
        error("Unknown option: %s", name);
    end
    opts.(name) = value;
end
end

function y = local_finite_or(value, fallback)
if isfinite(double(value))
    y = value;
else
    y = fallback;
end
end

function [massKg, cgXM] = local_reference_mass_cg(cfg)
massKg = cfg.mass.empty_mass_kg + sum(cfg.mass.cargo_mass_kg);
moment = cfg.mass.empty_mass_kg * cfg.mass.empty_cg_x_m + ...
    sum(cfg.mass.cargo_mass_kg .* cfg.mass.cargo_x_m);
cgXM = moment / massKg;
end

function value = local_index_or_zero(x, idx)
if numel(x) >= idx
    value = double(x(idx));
else
    value = 0.0;
end
end
