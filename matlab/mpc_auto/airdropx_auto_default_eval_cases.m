function cases = airdropx_auto_default_eval_cases(varargin)
%AIRDROPX_AUTO_DEFAULT_EVAL_CASES Fixed scenarios; these are NOT optimization variables.
opts = local_options(varargin{:});
base = struct("name", "nominal", ...
    "TargetAltitudeM", opts.TargetAltitudeM, "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "InitialAltitudeM", opts.TargetAltitudeM, "InitialAirspeedMps", opts.TargetAirspeedMps, ...
    "InitialPitchDeg", NaN, "InitialFlightPathDeg", 0.0, "StopTimeS", opts.StopTimeS, ...
    "FixedDropStartS", opts.FixedDropStartS, "FixedDropIntervalS", opts.FixedDropIntervalS, ...
    "FixedDropTotal", opts.FixedDropTotal);
cases = repmat(base, 2, 1);
cases(1).name = "nominal";
cases(1).InitialPitchDeg = [];
cases(2).name = "perturbed";
cases(2).InitialAltitudeM = opts.TargetAltitudeM + 2.0;
cases(2).InitialAirspeedMps = opts.TargetAirspeedMps - 3.0;
cases(2).InitialFlightPathDeg = -1.0;
cases(2).InitialPitchDeg = [];
end

function opts = local_options(varargin)
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.StopTimeS = 30.0;
opts.FixedDropStartS = 10.0;
opts.FixedDropIntervalS = 0.2;
opts.FixedDropTotal = 4.0;
if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin)
    n=string(varargin{i});
    if ~isfield(opts,n), error("Unknown option: %s",n); end
    opts.(n)=varargin{i+1};
end
end
