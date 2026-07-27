function sfun_airdropx_mpc_outer_pd_controller(block)
%SFUN_AIRDROPX_MPC_OUTER_PD_CONTROLLER MPC outer loop plus PD pitch inner loop.
%
% Input vector:
%   [altitude_m; vz_up_mps; airspeed_mps; pitch_deg; mass_kg; cg_x_m]
%
% Output vector:
%   [elevator_delta; throttle_cmd; pitch_ref_deg]

setup(block);
end

function setup(block)
block.NumDialogPrms = 1; % reserved model .mat path, or ""

block.NumInputPorts = 1;
block.NumOutputPorts = 1;

block.InputPort(1).Dimensions = 6;
block.InputPort(1).DatatypeID = 0;
block.InputPort(1).Complexity = "Real";
block.InputPort(1).DirectFeedthrough = true;

block.OutputPort(1).Dimensions = 3;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = "Real";

block.SampleTimes = [0.1 0.0];
block.SimStateCompliance = "DefaultSimState";

block.RegBlockMethod("Start", @Start);
block.RegBlockMethod("InitializeConditions", @InitializeConditions);
block.RegBlockMethod("Outputs", @Outputs);
end

function Start(block)
cfg = local_config_from_workspace();
cfg = local_apply_workspace_overrides(cfg);

data = struct();
data.cfg = cfg;
data.outer_state = [];
data.prev_pitch_deg = NaN;
data.prev_t = NaN;
data.prev_elevator = local_base_scalar("airdropx_initial_elevator_delta", 0.0);
local_memory("set", block, data);
end

function InitializeConditions(block)
data = local_memory("get", block);
if isempty(data)
    return;
end
data.outer_state = [];
data.prev_pitch_deg = NaN;
data.prev_t = NaN;
data.prev_elevator = local_base_scalar("airdropx_initial_elevator_delta", 0.0);
local_memory("set", block, data);
end

function Outputs(block)
data = local_memory("get", block);
if isempty(data)
    Start(block);
    data = local_memory("get", block);
end
cfg = data.cfg;

uIn = block.InputPort(1).Data;
altitudeM = double(uIn(1));
vzUpMps = double(uIn(2));
airspeedMps = double(uIn(3));
pitchDeg = double(uIn(4));
massKg = double(uIn(5));
cgXM = double(uIn(6));

t = block.CurrentTime;
if isnan(data.prev_pitch_deg) || isnan(data.prev_t) || t <= data.prev_t
    qDps = 0.0;
else
    dt = max(t - data.prev_t, eps);
    qDps = (pitchDeg - data.prev_pitch_deg) / dt;
end
data.prev_pitch_deg = pitchDeg;
data.prev_t = t;

x = [
    altitudeM - cfg.reference.h_m
    vzUpMps
    airspeedMps - cfg.reference.v_mps
    pitchDeg - cfg.reference.pitch_deg
    qDps
    massKg - cfg.reference.mass_kg
    cgXM - cfg.reference.cg_x_m
    ];

[pitchRefDeg, throttleCmd, outerState] = local_outer_command(x, pitchDeg, qDps, data.outer_state, cfg);
data.outer_state = outerState;
elevator = local_inner_pd(pitchDeg, pitchRefDeg, qDps, x, data.prev_elevator, cfg);
data.prev_elevator = elevator;

local_memory("set", block, data);
block.OutputPort(1).Data = [elevator; throttleCmd; pitchRefDeg];
end

function [pitchRefDeg, throttleCmd, outerState] = local_outer_command(x, pitchDeg, qDps, outerState, cfg)
if isfield(cfg, "outer_pd_allocation") && cfg.outer_pd_allocation.enabled
    [virtualCmd, outerState] = airdropx_mpc_controller(x, outerState, cfg.direct_mpc);
    desiredElevator = double(virtualCmd(1));
    throttleCmd = double(virtualCmd(2));
    pd = cfg.inner_pd;
    feedForward = pd.trim_elevator + ...
        pd.kd * double(qDps) + ...
        pd.mass_gain_elevator * double(x(6)) + ...
        pd.cg_gain_elevator * double(x(7));
    kp = max(abs(pd.kp), eps);
    pitchRefDeg = double(pitchDeg) - (desiredElevator - feedForward) / kp;
else
    [outerCmd, outerState] = airdropx_mpc_controller(x, outerState, cfg);
    pitchRefDeg = double(outerCmd(1));
    throttleCmd = double(outerCmd(2));
end
pitchRefDeg = min(max(pitchRefDeg, cfg.inner_pd.pitch_ref_min_deg), cfg.inner_pd.pitch_ref_max_deg);
end

function elevator = local_inner_pd(pitchDeg, pitchRefDeg, qDps, x, prevElevator, cfg)
pd = cfg.inner_pd;
raw = pd.trim_elevator + ...
    pd.kp * (double(pitchDeg) - double(pitchRefDeg)) + ...
    pd.kd * double(qDps) + ...
    pd.mass_gain_elevator * double(x(6)) + ...
    pd.cg_gain_elevator * double(x(7));

du = raw - double(prevElevator);
du = min(max(du, -pd.elevator_rate_limit), pd.elevator_rate_limit);
elevator = double(prevElevator) + du;
elevator = min(max(elevator, -pd.elevator_limit), pd.elevator_limit);
end

function cfg = local_config_from_workspace()
targetH = local_base_scalar("airdropx_target_altitude_m", 20.0);
targetV = local_base_scalar("airdropx_pd_v_ref_mps", 45.0);
targetPitch = local_base_scalar("airdropx_pd_pitch_ref_deg", 4.0);
refMass = local_base_scalar("airdropx_mpc_reference_mass_kg", 3423.0);
refCg = local_base_scalar("airdropx_mpc_reference_cg_x_m", 5.28048992112182);
altitudeBias = local_base_scalar("airdropx_mpc_control_altitude_bias_m", 0.95);
cfg = airdropx_mpc_outer_pd_config( ...
    "TargetAltitudeM", targetH, ...
    "TargetAirspeedMps", targetV, ...
    "TargetPitchDeg", targetPitch, ...
    "ControlAltitudeBiasM", altitudeBias, ...
    "ReferenceMassKg", refMass, ...
    "ReferenceCgXM", refCg);
end

function cfg = local_apply_workspace_overrides(cfg)
try
    if evalin("base", "exist('airdropx_mpc_outer_pd_config_overrides','var')")
        overrides = evalin("base", "airdropx_mpc_outer_pd_config_overrides");
        if ~isempty(overrides)
            if ~isstruct(overrides)
                error("airdropx_mpc_outer_pd_config_overrides must be a struct.");
            end
            cfg = local_merge_struct(cfg, overrides);
        end
    end
catch ME
    error("Failed to apply MPC outer/PD config overrides: %s", ME.message);
end
end

function base = local_merge_struct(base, overrides)
names = fieldnames(overrides);
for i = 1:numel(names)
    name = names{i};
    value = overrides.(name);
    if isstruct(value) && isfield(base, name) && isstruct(base.(name))
        base.(name) = local_merge_struct(base.(name), value);
    else
        base.(name) = value;
    end
end
end

function value = local_base_scalar(name, fallback)
try
    if evalin("base", "exist('" + name + "','var')")
        value = evalin("base", name);
        value = double(value);
        if isfinite(value)
            return;
        end
    end
catch
end
value = double(fallback);
end

function value = local_memory(action, block, value)
persistent store
if isempty(store)
    store = containers.Map("KeyType", "char", "ValueType", "any");
end

key = sprintf("%.15g", block.BlockHandle);
switch string(action)
    case "set"
        store(key) = value;
    case "get"
        if isKey(store, key)
            value = store(key);
        else
            value = [];
        end
    otherwise
        error("Unknown memory action: %s", action);
end
end
