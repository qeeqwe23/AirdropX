function cfg = airdropx_mpc_config(varargin)
%AIRDROPX_MPC_CONFIG Grey-box longitudinal MPC configuration.
%
% The controller uses the reduced state described in the technical route.
% Angle states use SI units internally:
%   x = [h*, Vz, Va*, theta*_rad, q_radps]'
% and input increments around trim:
%   u* = [delta_e*, throttle*]'.
%
% Five discrete models are kept, one for each released-cargo configuration.

opts = local_options(varargin{:});

cfg = struct();
cfg.dt_s = double(opts.Dt);
cfg.prediction_horizon = double(opts.PredictionHorizon);
cfg.control_horizon = double(opts.ControlHorizon);

cfg.state_names = [
    "h_err_m"
    "vz_up_mps"
    "airspeed_err_mps"
    "pitch_err_rad"
    "q_radps"
    ];
cfg.control_state_count = 5;
cfg.aux_state_names = ["mass_err_kg"; "cg_x_err_m"];
cfg.input_names = ["elevator_delta"; "throttle_cmd"];

cfg.reference.command_h_m = double(opts.TargetAltitudeM);
cfg.reference.altitude_bias_m = double(opts.ControlAltitudeBiasM);
cfg.reference.h_m = double(opts.TargetAltitudeM) + double(opts.ControlAltitudeBiasM);
cfg.reference.v_mps = double(opts.TargetAirspeedMps);
cfg.reference.pitch_deg = double(opts.TargetPitchDeg);
cfg.reference.mass_kg = double(opts.ReferenceMassKg);
cfg.reference.cg_x_m = double(opts.ReferenceCgXM);

cfg.trim.u = [double(opts.TrimElevatorDelta); double(opts.TrimThrottleCmd)];
cfg.trim.h_m = cfg.reference.h_m;
cfg.trim.v_mps = cfg.reference.v_mps;
cfg.trim.pitch_deg = cfg.reference.pitch_deg;
cfg.trim.pitch_rad = deg2rad(cfg.reference.pitch_deg);
cfg.trim.mass_kg = cfg.reference.mass_kg;
cfg.trim.cg_x_m = cfg.reference.cg_x_m;

cfg.gravity_mps2 = 9.80665;
cfg.mass.empty_mass_kg = double(opts.EmptyMassKg);
cfg.mass.empty_cg_x_m = double(opts.EmptyCgXM);
cfg.mass.empty_cg_z_m = double(opts.EmptyCgZM);
cfg.mass.empty_Iy_kgm2 = double(opts.EmptyIyKgm2);
cfg.mass.cargo_mass_kg = double(opts.CargoMassKg(:));
cfg.mass.cargo_x_m = double(opts.CargoXM(:));
cfg.mass.cargo_z_m = double(opts.CargoZM(:));
cfg.mass.drop_count_max = 4;
cfg.trim.bank = airdropx_mpc_trim_bank(cfg, ...
    "Va0Mps", opts.TrimAirspeedMps, ...
    "Theta0Deg", opts.TrimPitchDeg, ...
    "DeltaE0", opts.TrimElevatorDelta, ...
    "DeltaT0", opts.TrimThrottleCmd);

cfg.weights.Q = diag([30.0, 4.0, 5.0, 2400.0, 40.0]);
cfg.weights.R = diag([0.55, 1.10]);
cfg.weights.Rd = diag([14.0, 8.0]);
cfg.weights.terminal_scale = 5.0;

cfg.solver.type = "quadprog";
cfg.solver.fallback = "unconstrained";
cfg.solver.use_constraints = true;
cfg.solver.max_iterations = 80;

cfg.constraints.u_min = [-0.75; 0.35];
cfg.constraints.u_max = [ 0.45; 0.88];
cfg.constraints.du_min = [-0.045; -0.035];
cfg.constraints.du_max = [ 0.045;  0.035];
cfg.constraints.x_min = [-35; -8; -18; deg2rad(-12); deg2rad(-35)];
cfg.constraints.x_max = [ 35;  8;  18; deg2rad( 12); deg2rad( 35)];

cfg.integrator.enabled = true;
cfg.integrator.leak = 0.995;
cfg.integrator.limit = [25.0; 15.0; deg2rad(12.0)];
cfg.integrator.gain = [0.010, 0.000, 0.18; ...
                      -0.006, -0.010, 0.000];

cfg.drop_transition.enabled = true;
cfg.drop_transition.integral_reset_factor = 0.35;

cfg.safety_feedback.enabled = true;
cfg.safety_feedback.h_deadband_m = 1.0;
cfg.safety_feedback.vz_deadband_mps = 0.15;
cfg.safety_feedback.h_gain_elevator = 0.055;
cfg.safety_feedback.vz_gain_elevator = 0.090;
cfg.safety_feedback.h_gain_throttle = -0.030;
cfg.safety_feedback.vz_gain_throttle = -0.045;
cfg.safety_feedback.limit = [0.18; 0.14];

cfg.estimator.theta0.N = [0; 7000; 400; 0; -9000; 800; 0];
cfg.estimator.theta0.X = [-900; -300; 0; 0; 0; 18000; 0];
cfg.estimator.theta0.M = [0; -28000; -9000; 0; -70000; 0; 0];
cfg.estimator.P0Scale = 0.05;
cfg.estimator.R = diag([0.00475, 0.00050, 0.00200]);
cfg.estimator.ForgettingFactor = 0.995;
cfg.estimator.FilterWindow = 5;

cfg.greybox.state_units = ["m"; "m/s"; "m/s"; "rad"; "rad/s"];
cfg.greybox.input_units = ["normalized_or_actual_elevator"; "normalized_throttle"];
cfg.greybox.notes = "Slegers-style structured longitudinal model: hdot=Vz, thetadot=q, RLS estimates N/X/M derivatives and ZOH discretizes the continuous model.";

if opts.IncludeModel
    cfg.model_bank = airdropx_mpc_greybox_model(cfg);
    cfg.model = cfg.model_bank{1};
end
end

function opts = local_options(varargin)
opts.Dt = 0.10;
opts.PredictionHorizon = 25;
opts.ControlHorizon = 8;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.TrimAirspeedMps = [];
opts.TrimPitchDeg = 4.0;
opts.ControlAltitudeBiasM = 0.0;
opts.ReferenceMassKg = 3423.0;
opts.ReferenceCgXM = 5.28048992112182;
opts.EmptyMassKg = 2223.0;
opts.EmptyCgXM = 5.279;
opts.EmptyCgZM = -0.275;
opts.EmptyIyKgm2 = 6420.77 * 1.3558179483314004;
opts.CargoMassKg = [300.0; 300.0; 300.0; 300.0];
opts.CargoXM = [4.826; 5.131; 5.436; 5.740];
opts.CargoZM = [-0.305; -0.305; -0.305; -0.305];
opts.TrimElevatorDelta = 0.0;
opts.TrimThrottleCmd = 0.80;
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

if isempty(opts.TrimAirspeedMps)
    opts.TrimAirspeedMps = opts.TargetAirspeedMps;
end
end




