function cfg = airdropx_mpc_setup_closed_loop_workspace(varargin)
%AIRDROPX_MPC_SETUP_CLOSED_LOOP_WORKSPACE Prepare tuned standalone MPC defaults.
%
% Use this before manually opening/running matlab/mpc/airdropx_mpc_closed_loop.slx.

opts = local_options(varargin{:});

cfg = airdropx_mpc_setup_id_workspace( ...
    "Model", opts.Model, ...
    "StopTimeS", opts.StopTimeS, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", opts.TargetPitchDeg, ...
    "ControlAltitudeBiasM", opts.ControlAltitudeBiasM, ...
    "InitialAirspeedMps", opts.InitialAirspeedMps, ...
    "InitialAltitudeM", opts.InitialAltitudeM, ...
    "InitialPitchDeg", opts.InitialPitchDeg, ...
    "InitialFlightPathDeg", opts.InitialFlightPathDeg, ...
    "InitialElevatorDelta", opts.InitialElevatorDelta, ...
    "InitialThrottleCmd", opts.InitialThrottleCmd, ...
    "Force", opts.Force);
end

function opts = local_options(varargin)
opts.Model = "airdropx_mpc_closed_loop";
opts.StopTimeS = 22.0;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.ControlAltitudeBiasM = 0.95;
opts.InitialAirspeedMps = 55.5;
opts.InitialAltitudeM = NaN;
opts.InitialPitchDeg = NaN;
opts.InitialFlightPathDeg = 2.4;
opts.InitialElevatorDelta = 0.0;
opts.InitialThrottleCmd = 0.80;
opts.Force = true;

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
