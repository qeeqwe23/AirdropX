function cfg = airdropx_mpc_config(varargin)
%AIRDROPX_MPC_CONFIG Standalone MPC tuning configuration.
%
% This configuration is not connected to the existing Simulink model.

opts = local_options(varargin{:});

cfg = struct();
cfg.dt_s = double(opts.Dt);
cfg.prediction_horizon = double(opts.PredictionHorizon);
cfg.control_horizon = double(opts.ControlHorizon);

cfg.state_names = [
    "h_err_m"
    "vz_up_mps"
    "v_err_mps"
    "pitch_err_deg"
    "q_dps"
    "mass_err_kg"
    "cg_x_err_m"
    ];
cfg.input_names = [
    "elevator_delta"
    "throttle_cmd"
    ];

cfg.reference.command_h_m = double(opts.TargetAltitudeM);
cfg.reference.altitude_bias_m = double(opts.ControlAltitudeBiasM);
cfg.reference.h_m = double(opts.TargetAltitudeM) + double(opts.ControlAltitudeBiasM);
cfg.reference.v_mps = double(opts.TargetAirspeedMps);
cfg.reference.pitch_deg = double(opts.TargetPitchDeg);
cfg.reference.mass_kg = double(opts.ReferenceMassKg);
cfg.reference.cg_x_m = double(opts.ReferenceCgXM);

cfg.weights.Q = diag([16.0, 2.2, 4.0, 0.16, 0.40, 0.0, 0.0]);
cfg.weights.R = diag([0.45, 0.90]);
cfg.weights.Rd = diag([9.0, 7.0]);
cfg.weights.terminal_scale = 4.0;

cfg.constraints.u_min = [-0.75; 0.35];
cfg.constraints.u_max = [0.45; 0.85];
cfg.constraints.du_min = [-0.045; -0.035];
cfg.constraints.du_max = [0.045; 0.035];

cfg.integrator.leak = 0.999;
cfg.integrator.h_limit = 30.0;
cfg.integrator.v_limit = 20.0;
cfg.integral_feedback.enabled = true;
cfg.integral_feedback.h_gain_elevator = 0.036;
cfg.integral_feedback.pitch_gain_elevator = 0.004;
cfg.integral_feedback.v_gain_throttle = -0.010;
cfg.integral_feedback.h_gain_throttle = -0.012;
cfg.integral_feedback.limit = [0.18; 0.16];
cfg.safety_feedback.enabled = true;
cfg.safety_feedback.h_deadband_m = 0.30;
cfg.safety_feedback.vz_deadband_mps = 0.05;
cfg.safety_feedback.h_gain_elevator = 0.095;
cfg.safety_feedback.vz_gain_elevator = 0.075;
cfg.safety_feedback.h_gain_throttle = -0.200;
cfg.safety_feedback.vz_gain_throttle = -0.040;
cfg.safety_feedback.limit = [0.24; 0.35];
cfg.mass_feedback.enabled = true;
cfg.mass_feedback.mass_gain_elevator = -8.0e-5;
cfg.mass_feedback.mass_gain_throttle = 4.0e-5;
cfg.mass_feedback.cg_gain_elevator = 0.12;
cfg.mass_feedback.cg_gain_throttle = 0.00;
cfg.mass_feedback.limit = [0.08; 0.06];

cfg.trim.u = [0.0; 0.80];
cfg.trim.h_m = cfg.reference.h_m;
cfg.trim.v_mps = cfg.reference.v_mps;
cfg.trim.pitch_deg = cfg.reference.pitch_deg;
cfg.trim.mass_kg = cfg.reference.mass_kg;
cfg.trim.cg_x_m = cfg.reference.cg_x_m;

if opts.IncludeModel
    cfg.model = airdropx_mpc_nominal_model(cfg);
end
end

function opts = local_options(varargin)
opts.Dt = 0.10;
opts.PredictionHorizon = 25;
opts.ControlHorizon = 8;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.ControlAltitudeBiasM = 0.0;
opts.ReferenceMassKg = 3423.0;
opts.ReferenceCgXM = 5.28048992112182;
opts.IncludeModel = true;

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
