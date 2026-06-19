function cfg = airdropx_sim_params(varargin)
%AIRDROPX_SIM_PARAMS Central tuning/configuration entry point for Simulink.
%
% Edit this file when tuning the MATLAB/Simulink model. setup_airdropx_simulink
% publishes the scalar/struct fields below into the base workspace so the
% .slx model can reference variable names instead of hard-coded numbers.

opts = local_options(varargin{:});

projectRoot = string(opts.ProjectRoot);
if strlength(projectRoot) == 0
    thisFile = mfilename("fullpath");
    matlabDir = fileparts(thisFile);
    projectRoot = string(fileparts(matlabDir));
end

aircraftName = string(opts.AircraftName);
icName = string(opts.IcName);
icTemplateName = string(fullfile(projectRoot, "aircraft", aircraftName, "reset_20m"));
usingGeneratedIc = strlength(icName) == 0;
if usingGeneratedIc
    icName = string(fullfile(projectRoot, "aircraft", aircraftName, "generated", "reset_20m_runtime"));
end

dt = double(opts.Dt);

cfg = struct();
cfg.projectRoot = projectRoot;
cfg.aircraftName = aircraftName;
cfg.icName = icName;
cfg.icTemplateName = icTemplateName;

cfg.sim.dt_s = dt;
cfg.sim.stop_time_s = 30.0;
cfg.dt = dt;

cfg.initial.airspeed_mps = 45.0;
cfg.initial.theta_deg = 4.0;
cfg.initial.pitch_deg = cfg.initial.theta_deg;
cfg.initial.use_generated_ic = usingGeneratedIc;
cfg.initial.generated_ic_name = icName;
cfg.initial.template_ic_name = icTemplateName;

% 1 = fixed-time four-drop drives the plant; 2 = CARP/CEP drives the plant.
% Both schedulers still run in the model so their diagnostics stay available.
cfg.drop_mode = 1.0;

cfg.environment.wind_speed_mps = 0.0;
cfg.environment.wind_dir_from_deg = 270.0;
cfg.environment.reset_cmd = 0.0;

cfg.control.target_altitude_m = 22.8;
cfg.control.initial_elevator_delta = 0.10;
cfg.control.initial_throttle_cmd = 0.58;
cfg.control.pd_gains = struct( ...
    "Kp", 0.15, ...
    "Kd", 0.18, ...
    "u_limit", 0.85, ...
    "u_rate_limit", 0.050, ...
    "K_mass", 0.03, ...
    "bias_rate_limit", 0.0004, ...
    "throttle_kp", 0.055, ...
    "throttle_fixed", 0.58, ...
    "throttle_alt_kp", 0.030, ...
    "throttle_vz_kd", 0.010, ...
    "v_ref_mps", 45.0, ...
    "pitch_ref_deg", 4.0, ...
    "pitch_kp", 0.060, ...
    "pitch_limit", 0.30, ...
    "pitch_rate_kd", 0.012, ...
    "pitch_rate_limit", 0.16, ...
    "dt_s", dt);
cfg.control.drop_mass_signal_kg = 300.0;

cfg.mass.empty_mass_kg = 2223.0;
cfg.mass.empty_cg_x_m = 5.279;
cfg.mass.cargo_mass_kg = [300.0, 300.0, 300.0, 300.0];
cfg.mass.cargo_x_m = [4.826, 5.131, 5.436, 5.740];

cfg.fixed_drop.start_s = 10.0;
cfg.fixed_drop.interval_s = 0.2;
cfg.fixed_drop.pulse_s = dt;
cfg.fixed_drop.drop_total = 4.0;
cfg.fixed_drop.initial_cmd = 0.0;

cfg.carp.target_n_m = 1000.0;
cfg.carp.target_e_m = 0.0;
cfg.carp.release_window_s = 0.7;
cfg.carp.interval_s = 0.5;
cfg.carp.drop_total = 4.0;
cfg.carp.min_safe_alt_m = 15.0;

cfg.ballistics.gravity_mps2 = 9.80665;
cfg.ballistics.calibration_airspeed_mps = 78.6;
cfg.ballistics.calibration_altitude_m = 20.0;
cfg.ballistics.jsbsim_drop_distance_m = 150.7649;
cfg.ballistics.side_wind_gain = 0.0776;
calT = sqrt(2.0 * cfg.ballistics.calibration_altitude_m / cfg.ballistics.gravity_mps2);
calTheory = cfg.ballistics.calibration_airspeed_mps * calT;
cfg.ballistics.k_drag_calibrated = ...
    (calTheory - cfg.ballistics.jsbsim_drop_distance_m) / ...
    (cfg.ballistics.calibration_airspeed_mps ^ 2 * calT);

cfg.metrics.settling_altitude_band_m = 0.5;
cfg.metrics.settling_vz_band_mps = 0.3;
cfg.metrics.elevator_saturation_abs = 0.90;

cfg.monte_carlo.samples = 200;
cfg.monte_carlo.seed = 42;
cfg.monte_carlo.alt_std_m = 0.4;
cfg.monte_carlo.airspeed_std_mps = 1.0;
cfg.monte_carlo.heading_std_deg = 0.5;
cfg.monte_carlo.wind_std_mps = 1.5;

cfg.video.fps = 30;
cfg.video.width = 1920;
cfg.video.height = 1080;
cfg.video.quality = 95;
cfg.video.max_frames = 1800;
cfg.video.visible = "on";

cfg.matlabDir = string(fullfile(projectRoot, "matlab"));
cfg.sfuncDir = string(fullfile(cfg.matlabDir, "sfunc_jsbsim"));
cfg.vrDir = string(fullfile(cfg.matlabDir, "vr"));
cfg.model = string(opts.Model);

if cfg.initial.use_generated_ic
    local_write_initial_condition(cfg.initial.template_ic_name, ...
        cfg.initial.generated_ic_name, cfg.initial.airspeed_mps, ...
        cfg.initial.pitch_deg);
end

if opts.AssignBase
    local_assign_base(cfg);
end
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.AircraftName = "MQ9_Reaper";
opts.IcName = "";
opts.Dt = 1/120;
opts.Model = "untitled1";
opts.AssignBase = false;

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

function local_assign_base(cfg)
assignin("base", "airdropx_cfg", cfg);
assignin("base", "projectRoot", cfg.projectRoot);
assignin("base", "aircraftName", cfg.aircraftName);
assignin("base", "icName", cfg.icName);
assignin("base", "airdropx_ic_template_name", cfg.icTemplateName);
assignin("base", "dt", cfg.sim.dt_s);

assignin("base", "airdropx_stop_time_s", cfg.sim.stop_time_s);
assignin("base", "airdropx_drop_mode", cfg.drop_mode);

assignin("base", "airdropx_wind_speed_mps", cfg.environment.wind_speed_mps);
assignin("base", "airdropx_wind_dir_from_deg", cfg.environment.wind_dir_from_deg);
assignin("base", "airdropx_reset_cmd", cfg.environment.reset_cmd);

assignin("base", "airdropx_initial_airspeed_mps", cfg.initial.airspeed_mps);
assignin("base", "airdropx_initial_theta_deg", cfg.initial.theta_deg);
assignin("base", "airdropx_initial_pitch_deg", cfg.initial.pitch_deg);
assignin("base", "airdropx_target_altitude_m", cfg.control.target_altitude_m);
assignin("base", "airdropx_initial_elevator_delta", cfg.control.initial_elevator_delta);
assignin("base", "airdropx_initial_throttle_cmd", cfg.control.initial_throttle_cmd);
assignin("base", "airdropx_pd_gains", cfg.control.pd_gains);
assignin("base", "airdropx_pd_Kp", cfg.control.pd_gains.Kp);
assignin("base", "airdropx_pd_Kd", cfg.control.pd_gains.Kd);
assignin("base", "airdropx_pd_u_limit", cfg.control.pd_gains.u_limit);
assignin("base", "airdropx_pd_u_rate_limit", cfg.control.pd_gains.u_rate_limit);
assignin("base", "airdropx_pd_K_mass", cfg.control.pd_gains.K_mass);
assignin("base", "airdropx_pd_bias_rate_limit", cfg.control.pd_gains.bias_rate_limit);
assignin("base", "airdropx_pd_throttle_kp", cfg.control.pd_gains.throttle_kp);
assignin("base", "airdropx_pd_throttle_fixed", cfg.control.pd_gains.throttle_fixed);
assignin("base", "airdropx_pd_throttle_alt_kp", cfg.control.pd_gains.throttle_alt_kp);
assignin("base", "airdropx_pd_throttle_vz_kd", cfg.control.pd_gains.throttle_vz_kd);
assignin("base", "airdropx_pd_v_ref_mps", cfg.control.pd_gains.v_ref_mps);
assignin("base", "airdropx_pd_pitch_ref_deg", cfg.control.pd_gains.pitch_ref_deg);
assignin("base", "airdropx_pd_pitch_kp", cfg.control.pd_gains.pitch_kp);
assignin("base", "airdropx_pd_pitch_limit", cfg.control.pd_gains.pitch_limit);
assignin("base", "airdropx_pd_pitch_rate_kd", cfg.control.pd_gains.pitch_rate_kd);
assignin("base", "airdropx_pd_pitch_rate_limit", cfg.control.pd_gains.pitch_rate_limit);
assignin("base", "airdropx_pd_dt_s", cfg.control.pd_gains.dt_s);
assignin("base", "airdropx_drop_mass_signal_kg", cfg.control.drop_mass_signal_kg);

assignin("base", "airdropx_fixed_drop_start_s", cfg.fixed_drop.start_s);
assignin("base", "airdropx_fixed_drop_interval_s", cfg.fixed_drop.interval_s);
assignin("base", "airdropx_fixed_drop_pulse_s", cfg.fixed_drop.pulse_s);
assignin("base", "airdropx_fixed_drop_total", cfg.fixed_drop.drop_total);
assignin("base", "airdropx_initial_drop_cmd", cfg.fixed_drop.initial_cmd);

assignin("base", "airdropx_carp_target_n_m", cfg.carp.target_n_m);
assignin("base", "airdropx_carp_target_e_m", cfg.carp.target_e_m);
assignin("base", "airdropx_carp_release_window_s", cfg.carp.release_window_s);
assignin("base", "airdropx_carp_interval_s", cfg.carp.interval_s);
assignin("base", "airdropx_carp_drop_total", cfg.carp.drop_total);
assignin("base", "airdropx_carp_min_safe_alt_m", cfg.carp.min_safe_alt_m);

assignin("base", "airdropx_metrics_settling_altitude_band_m", cfg.metrics.settling_altitude_band_m);
assignin("base", "airdropx_metrics_settling_vz_band_mps", cfg.metrics.settling_vz_band_mps);
assignin("base", "airdropx_metrics_elevator_saturation_abs", cfg.metrics.elevator_saturation_abs);
assignin("base", "airdropx_ballistics_gravity_mps2", cfg.ballistics.gravity_mps2);
assignin("base", "airdropx_ballistics_k_drag", cfg.ballistics.k_drag_calibrated);
assignin("base", "airdropx_ballistics_side_wind_gain", cfg.ballistics.side_wind_gain);
end

function local_write_initial_condition(templateName, generatedName, airspeedMps, pitchDeg)
templatePath = local_xml_path(templateName);
generatedPath = local_xml_path(generatedName);

if ~isfile(templatePath)
    error("AirdropX initial condition template not found: %s", templatePath);
end

xmlText = fileread(templatePath);
ubodyExpr = '<ubody\s+unit="M/SEC">[^<]*</ubody>';
if isempty(regexp(xmlText, ubodyExpr, "once"))
    error("AirdropX initial condition template has no M/SEC ubody field: %s", templatePath);
end
xmlText = regexprep(xmlText, ubodyExpr, ...
    sprintf('<ubody unit="M/SEC">%.10g</ubody>', airspeedMps), "once");

thetaExpr = '<theta\s+unit="DEG">[^<]*</theta>';
if isempty(regexp(xmlText, thetaExpr, "once"))
    error("AirdropX initial condition template has no DEG theta field: %s", templatePath);
end
xmlText = regexprep(xmlText, thetaExpr, ...
    sprintf('<theta unit="DEG">%.10g</theta>', pitchDeg), "once");

generatedDir = fileparts(generatedPath);
if ~isfolder(generatedDir)
    mkdir(generatedDir);
end

fid = fopen(generatedPath, "w", "n", "UTF-8");
if fid < 0
    error("AirdropX cannot write generated initial condition: %s", generatedPath);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "%s", xmlText);
end

function xmlPath = local_xml_path(name)
xmlPath = char(name);
[~, ~, ext] = fileparts(xmlPath);
if strlength(string(ext)) == 0
    xmlPath = [xmlPath, '.xml'];
end
end
