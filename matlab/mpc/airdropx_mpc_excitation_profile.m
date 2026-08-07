function profile = airdropx_mpc_excitation_profile(varargin)
%AIRDROPX_MPC_EXCITATION_PROFILE Multi-sine actuator excitation for ID runs.

opts = local_options(varargin{:});
t = (0:double(opts.Dt):double(opts.StopTimeS)).';
elevator = local_multisine(t, double(opts.ElevatorAmplitudes(:)), double(opts.ElevatorFrequenciesHz(:)));
throttle = local_multisine(t, double(opts.ThrottleAmplitudes(:)), double(opts.ThrottleFrequenciesHz(:)));

profile = struct();
profile.time_s = t;
profile.elevator = [t, min(max(elevator, double(opts.ElevatorMin)), double(opts.ElevatorMax))];
profile.throttle = [t, min(max(throttle, double(opts.ThrottleMin)), double(opts.ThrottleMax))];
profile.notes = "Elevator and throttle use distinct multi-sine frequencies for grey-box identifiability.";
end

function y = local_multisine(t, amplitudes, frequenciesHz)
n = min(numel(amplitudes), numel(frequenciesHz));
y = zeros(size(t));
for i = 1:n
    y = y + amplitudes(i) * sin(2.0 * pi * frequenciesHz(i) * t);
end
end

function opts = local_options(varargin)
opts.Dt = 0.10;
opts.StopTimeS = 30.0;
opts.ElevatorAmplitudes = [0.035; 0.025; 0.018];
opts.ElevatorFrequenciesHz = [0.11; 0.23; 0.37];
opts.ThrottleAmplitudes = [0.025; 0.018; 0.012];
opts.ThrottleFrequenciesHz = [0.07; 0.17; 0.31];
opts.ElevatorMin = -0.12;
opts.ElevatorMax = 0.12;
opts.ThrottleMin = -0.08;
opts.ThrottleMax = 0.08;
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
