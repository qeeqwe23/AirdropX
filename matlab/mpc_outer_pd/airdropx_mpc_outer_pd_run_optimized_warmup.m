function result = airdropx_mpc_outer_pd_run_optimized_warmup(varargin)
%AIRDROPX_MPC_OUTER_PD_RUN_OPTIMIZED_WARMUP Conservative optimized run.
%
% This preset prioritizes altitude hold and pitch over airspeed. It uses a
% 20 s warm-up, then exports and evaluates the following 30 s window.

opts = local_options(varargin{:});

result = airdropx_mpc_outer_pd_run_closed_loop( ...
    "OutputRoot", opts.OutputRoot, ...
    "StopTimeS", opts.StopTimeS, ...
    "WarmupTimeS", opts.WarmupTimeS, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", opts.TargetPitchDeg, ...
    "InitialAirspeedMps", opts.InitialAirspeedMps, ...
    "InitialAltitudeM", opts.InitialAltitudeM, ...
    "InitialPitchDeg", opts.InitialPitchDeg, ...
    "InitialFlightPathDeg", opts.InitialFlightPathDeg, ...
    "InitialElevatorDelta", opts.InitialElevatorDelta, ...
    "InitialThrottleCmd", opts.InitialThrottleCmd, ...
    "ControlAltitudeBiasM", opts.ControlAltitudeBiasM, ...
    "ConfigOverrides", opts.ConfigOverrides, ...
    "RecreateModel", opts.RecreateModel, ...
    "DisableVRForBatch", opts.DisableVRForBatch);
end

function opts = local_options(varargin)
opts.OutputRoot = "";
opts.StopTimeS = 30.0;
opts.WarmupTimeS = 20.0;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 45.0;
opts.TargetPitchDeg = 4.0;
opts.InitialAirspeedMps = 45.0;
opts.InitialAltitudeM = 20.0;
opts.InitialPitchDeg = 4.0;
opts.InitialFlightPathDeg = 0.0;
opts.InitialElevatorDelta = -0.193;
opts.InitialThrottleCmd = 0.507;
opts.ControlAltitudeBiasM = 1.20;
opts.ConfigOverrides = [];
opts.RecreateModel = false;
opts.DisableVRForBatch = true;

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
