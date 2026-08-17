function trims = airdropx_auto_default_trim_bank(varargin)
%AIRDROPX_AUTO_DEFAULT_TRIM_BANK Initial trim guesses for five drop configs.
%
% Replace these with airdropx_auto_find_trim results once JSBSim trim search is
% run. The fields are intentionally named for MATLAB mpc() nominal values.

opts = local_options(varargin{:});
trims = repmat(struct( ...
    "config_id", 0, ...
    "altitude_m", double(opts.TargetAltitudeM), ...
    "airspeed_mps", double(opts.TargetAirspeedMps), ...
    "pitch_deg", double(opts.PitchDeg), ...
    "vz_up_mps", 0.0, ...
    "q_dps", 0.0, ...
    "elevator_cmd", double(opts.ElevatorCmd), ...
    "throttle_cmd", double(opts.ThrottleCmd), ...
    "score", NaN), 5, 1);
for k = 1:5
    trims(k).config_id = k - 1;
end
end

function opts = local_options(varargin)
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.PitchDeg = 4.0;
opts.ElevatorCmd = 0.0;
opts.ThrottleCmd = 0.80;
if mod(numel(varargin), 2) ~= 0
    error("Options must be name-value pairs.");
end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name)
        error("Unknown option: %s", name);
    end
    opts.(name) = varargin{i + 1};
end
end
