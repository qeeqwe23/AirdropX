function profile = airdropx_auto_make_excitation(varargin)
%AIRDROPX_AUTO_MAKE_EXCITATION Random-hold actuator excitation for ID data.
%
% The output format matches From Workspace time/value matrices:
%   profile.elevator = [time_s, elevator_cmd]
%   profile.throttle = [time_s, throttle_cmd]

opts = local_options(varargin{:});
rng(double(opts.Seed));
Ts = double(opts.Ts);
t = (0:Ts:double(opts.StopTimeS)).';
N = numel(t);

e = local_random_hold(N, opts.ElevatorHoldRange, double(opts.ElevatorAmplitude));
th = local_random_hold(N, opts.ThrottleHoldRange, double(opts.ThrottleAmplitude));

elevator = min(max(double(opts.ElevatorTrim) + e, double(opts.ElevatorMin)), double(opts.ElevatorMax));
throttle = min(max(double(opts.ThrottleTrim) + th, double(opts.ThrottleMin)), double(opts.ThrottleMax));

profile = struct();
profile.time_s = t;
profile.elevator = [t, elevator];
profile.throttle = [t, throttle];
profile.elevator_delta = e;
profile.throttle_delta = th;
profile.seed = double(opts.Seed);
profile.notes = "Zero-mean random held elevator/throttle excitation around trim.";
end

function y = local_random_hold(N, holdRange, amplitude)
y = zeros(N, 1);
k = 1;
holdRange = round(double(holdRange(:)));
if numel(holdRange) < 2
    holdRange = [3; 12];
end
lo = max(1, min(holdRange));
hi = max(lo, max(holdRange));
while k <= N
    holdN = randi([lo hi]);
    amp = amplitude * (2.0 * rand() - 1.0);
    j = min(N, k + holdN - 1);
    y(k:j) = amp;
    k = j + 1;
end
% Remove the finite-record DC bias.  With only tens of seconds of random
% holds, the old excitation could have a nonzero mean and push the aircraft
% away from the verified operating point during the whole ID run.
y = y - mean(y, "omitnan");
peak = max(abs(y), [], "omitnan");
if isfinite(peak) && peak > amplitude && peak > 0
    y = y * (amplitude / peak);
end
end

function opts = local_options(varargin)
opts.Ts = 0.1;
opts.StopTimeS = 30.0;
opts.Seed = 1;
opts.ElevatorTrim = 0.0;
opts.ThrottleTrim = 0.80;
opts.ElevatorAmplitude = 0.03;
opts.ThrottleAmplitude = 0.06;
opts.ElevatorHoldRange = [3 12];
opts.ThrottleHoldRange = [5 20];
opts.ElevatorMin = -0.75;
opts.ElevatorMax = 0.45;
opts.ThrottleMin = 0.35;
opts.ThrottleMax = 0.88;
opts = local_parse(opts, varargin{:});
end

function opts = local_parse(opts, varargin)
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
