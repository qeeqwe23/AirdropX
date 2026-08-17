function simOut = airdropx_auto_eval_case(mpcBankMat, caseDef)
%AIRDROPX_AUTO_EVAL_CASE Default real-JSBSim evaluator for automatic MPC tuning.
%
% Signature intentionally matches airdropx_auto_final_test / bayesopt tuning:
%   simOut = airdropx_auto_eval_case(mpcBankMat, caseDef)
%
% Pitch reference is ALWAYS taken from trim_bank for the active drop/config.

if nargin < 2 || isempty(caseDef), caseDef = struct("name", "nominal"); end
S = load(mpcBankMat, "trim_bank");
trim0 = S.trim_bank(1);

% Remove contamination from previous manual scans and use automatic trim pitch.
assignin("base", "airdropx_auto_use_trim_pitch_reference", 1.0);
assignin("base", "airdropx_auto_elevator_sign", 1.0);
assignin("base", "airdropx_auto_pitch_kp", 0.0);
assignin("base", "airdropx_auto_pitch_kq", 0.0);
assignin("base", "airdropx_auto_pitch_damp_max", 0.0);
assignin("base", "airdropx_auto_throttle_alt_high_gain", 0.0);
assignin("base", "airdropx_auto_throttle_climb_gain", 0.0);
assignin("base", "airdropx_auto_elevator_safety_gain", 0.0);
assignin("base", "airdropx_auto_elevator_sink_gain", 0.0);
assignin("base", "airdropx_auto_elevator_safety_max", 0.0);
assignin("base", "airdropx_auto_throttle_safety_gain", 0.0);
assignin("base", "airdropx_auto_throttle_sink_gain", 0.0);
assignin("base", "airdropx_auto_throttle_safety_max", 0.0);
assignin("base", "airdropx_auto_pitch_rate_filter_tau_s", 0.35);

name = local_get(caseDef, "name", "nominal");
targetH = local_get(caseDef, "TargetAltitudeM", 20.0);
targetV = local_get(caseDef, "TargetAirspeedMps", 50.0);
initialH = local_get(caseDef, "InitialAltitudeM", targetH);
initialV = local_get(caseDef, "InitialAirspeedMps", targetV);
initialPitch = local_get(caseDef, "InitialPitchDeg", trim0.pitch_deg);
initialGamma = local_get(caseDef, "InitialFlightPathDeg", 0.0);
stopTime = local_get(caseDef, "StopTimeS", 30.0);
dropStart = local_get(caseDef, "FixedDropStartS", 10.0);
dropInterval = local_get(caseDef, "FixedDropIntervalS", 0.2);
dropTotal = local_get(caseDef, "FixedDropTotal", 4.0);

outRoot = string(local_get(caseDef, "OutputRoot", ""));
if strlength(outRoot) == 0
    [parent,~,~] = fileparts(mpcBankMat);
    outRoot = string(fullfile(parent, "eval_" + string(name)));
end

simOut = airdropx_auto_run_closed_loop( ...
    "MpcBankMat", mpcBankMat, ...
    "OutputRoot", outRoot, ...
    "CaseId", string(name), ...
    "StopTimeS", stopTime, ...
    "InitialAltitudeM", initialH, ...
    "InitialAirspeedMps", initialV, ...
    "InitialPitchDeg", initialPitch, ...
    "InitialFlightPathDeg", initialGamma, ...
    "InitialElevatorDelta", double(trim0.elevator_cmd), ...
    "InitialThrottleCmd", double(trim0.throttle_cmd), ...
    "TargetAltitudeM", targetH, ...
    "TargetAirspeedMps", targetV, ...
    "TargetPitchDeg", double(trim0.pitch_deg), ...
    "FixedDropStartS", dropStart, ...
    "FixedDropIntervalS", dropInterval, ...
    "FixedDropTotal", dropTotal);
end

function value = local_get(s, name, fallback)
value = fallback;
if isstruct(s) && isfield(s, name)
    x = s.(name);
    if ~isempty(x), value = x; end
elseif istable(s) && ismember(name, string(s.Properties.VariableNames))
    x = s.(name)(1);
    if ~isempty(x), value = x; end
end
end

