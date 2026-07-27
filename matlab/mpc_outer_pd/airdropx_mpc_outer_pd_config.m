function cfg = airdropx_mpc_outer_pd_config(varargin)
%AIRDROPX_MPC_OUTER_PD_CONFIG MPC outer-loop tuning for pitch-reference control.

opts = local_options(varargin{:});

cfg = airdropx_mpc_config( ...
    "Dt", opts.Dt, ...
    "PredictionHorizon", opts.PredictionHorizon, ...
    "ControlHorizon", opts.ControlHorizon, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", opts.TargetPitchDeg, ...
    "ControlAltitudeBiasM", opts.ControlAltitudeBiasM, ...
    "ReferenceMassKg", opts.ReferenceMassKg, ...
    "ReferenceCgXM", opts.ReferenceCgXM, ...
    "IncludeModel", false);

cfg.input_names = [
    "pitch_ref_deg"
    "throttle_cmd"
    ];

cfg.weights.Q = diag([18.0, 2.8, 4.2, 0.65, 0.55, 0.0, 0.0]);
cfg.weights.R = diag([0.18, 0.90]);
cfg.weights.Rd = diag([12.0, 7.0]);
cfg.weights.terminal_scale = 4.5;

cfg.constraints.u_min = [1.0; 0.35];
cfg.constraints.u_max = [6.6; 0.88];
cfg.constraints.du_min = [-0.14; -0.035];
cfg.constraints.du_max = [0.14; 0.035];

cfg.integral_feedback.enabled = true;
cfg.integral_feedback.h_gain_elevator = -0.035;
cfg.integral_feedback.pitch_gain_elevator = -0.012;
cfg.integral_feedback.v_gain_throttle = -0.010;
cfg.integral_feedback.h_gain_throttle = -0.010;
cfg.integral_feedback.limit = [0.80; 0.16];

cfg.safety_feedback.enabled = true;
cfg.safety_feedback.h_deadband_m = 0.30;
cfg.safety_feedback.vz_deadband_mps = 0.05;
cfg.safety_feedback.h_gain_elevator = -0.30;
cfg.safety_feedback.vz_gain_elevator = -0.25;
cfg.safety_feedback.h_gain_throttle = -0.16;
cfg.safety_feedback.vz_gain_throttle = -0.035;
cfg.safety_feedback.limit = [1.20; 0.35];

cfg.mass_feedback.enabled = true;
cfg.mass_feedback.mass_gain_elevator = 1.8e-4;
cfg.mass_feedback.mass_gain_throttle = 4.0e-5;
cfg.mass_feedback.cg_gain_elevator = -0.45;
cfg.mass_feedback.cg_gain_throttle = 0.00;
cfg.mass_feedback.limit = [0.70; 0.06];

cfg.trim.u = [cfg.reference.pitch_deg; 0.80];
cfg.model = airdropx_mpc_outer_pd_nominal_model(cfg);

cfg.inner_pd = struct();
cfg.inner_pd.kp = double(opts.InnerPitchKp);
cfg.inner_pd.kd = double(opts.InnerPitchKd);
cfg.inner_pd.elevator_limit = double(opts.InnerElevatorLimit);
cfg.inner_pd.elevator_rate_limit = double(opts.InnerElevatorRateLimit);
cfg.inner_pd.pitch_ref_min_deg = double(opts.PitchRefMinDeg);
cfg.inner_pd.pitch_ref_max_deg = double(opts.PitchRefMaxDeg);
cfg.inner_pd.mass_gain_elevator = double(opts.InnerMassGainElevator);
cfg.inner_pd.cg_gain_elevator = double(opts.InnerCgGainElevator);
cfg.inner_pd.trim_elevator = double(opts.InnerTrimElevator);
cfg.outer_pd_allocation.enabled = logical(opts.UseDirectMpcAllocation);
cfg.direct_mpc = airdropx_mpc_config( ...
    "Dt", opts.Dt, ...
    "PredictionHorizon", opts.PredictionHorizon, ...
    "ControlHorizon", opts.ControlHorizon, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", opts.TargetPitchDeg, ...
    "ControlAltitudeBiasM", opts.ControlAltitudeBiasM, ...
    "ReferenceMassKg", opts.ReferenceMassKg, ...
    "ReferenceCgXM", opts.ReferenceCgXM);
end

function opts = local_options(varargin)
opts.Dt = 0.10;
opts.PredictionHorizon = 25;
opts.ControlHorizon = 8;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.ControlAltitudeBiasM = 0.95;
opts.ReferenceMassKg = 3423.0;
opts.ReferenceCgXM = 5.28048992112182;
opts.InnerPitchKp = 0.30;
opts.InnerPitchKd = 0.020;
opts.InnerElevatorLimit = 0.85;
opts.InnerElevatorRateLimit = 0.20;
opts.InnerMassGainElevator = -2.0e-5;
opts.InnerCgGainElevator = 0.03;
opts.InnerTrimElevator = 0.0;
opts.PitchRefMinDeg = -8.0;
opts.PitchRefMaxDeg = 14.0;
opts.UseDirectMpcAllocation = true;

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
