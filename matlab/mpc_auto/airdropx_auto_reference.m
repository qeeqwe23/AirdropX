function ref = airdropx_auto_reference(drop_count, trim_bank, varargin)
%AIRDROPX_AUTO_REFERENCE Reference vector for Multiple MPC Controllers.
%
% ref = [altitude_ref; airspeed_ref; pitch_trim_for_config; vz_ref; q_ref]

opts = local_options(varargin{:});
idx = min(max(round(double(drop_count)) + 1, 1), numel(trim_bank));
ref = [
    double(opts.TargetAltitudeM)
    double(opts.TargetAirspeedMps)
    double(trim_bank(idx).pitch_deg)
    double(opts.TargetVzUpMps)
    double(opts.TargetQDps)
    ];
end

function opts = local_options(varargin)
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.TargetVzUpMps = 0.0;
opts.TargetQDps = 0.0;
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
end
