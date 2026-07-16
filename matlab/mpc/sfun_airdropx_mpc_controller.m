function sfun_airdropx_mpc_controller(block)
%SFUN_AIRDROPX_MPC_CONTROLLER Interpreted MPC controller for Simulink tests.
%
% Input vector:
%   [altitude_m; vz_up_mps; airspeed_mps; pitch_deg; mass_kg; cg_x_m]
%
% Output vector:
%   [elevator_delta; throttle_cmd]

setup(block);
end

function setup(block)
block.NumDialogPrms = 1; % identified model .mat path, or ""

block.NumInputPorts = 1;
block.NumOutputPorts = 1;

block.InputPort(1).Dimensions = 6;
block.InputPort(1).DatatypeID = 0;
block.InputPort(1).Complexity = "Real";
block.InputPort(1).DirectFeedthrough = true;

block.OutputPort(1).Dimensions = 2;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = "Real";

block.SampleTimes = [0.1 0.0];
block.SimStateCompliance = "DefaultSimState";

block.RegBlockMethod("Start", @Start);
block.RegBlockMethod("InitializeConditions", @InitializeConditions);
block.RegBlockMethod("Outputs", @Outputs);
end

function Start(block)
modelMat = block.DialogPrm(1).Data;
if isstring(modelMat)
    modelMat = char(modelMat);
end

if isempty(modelMat) || ~isfile(modelMat)
    cfg = local_config_from_workspace();
    cfg.model = airdropx_mpc_nominal_model(cfg);
else
    loaded = load(modelMat);
    if ~isfield(loaded, "cfg") || ~isfield(loaded, "model")
        error("MPC model MAT must contain cfg and model: %s", modelMat);
    end
    cfg = loaded.cfg;
    cfg.model = loaded.model;
end

cfg.dt_s = 0.1;
cfg.prediction_horizon = 25;
cfg.control_horizon = 8;

data = struct();
data.cfg = cfg;
data.controller_state = [];
data.prev_pitch_deg = NaN;
data.prev_t = NaN;
local_memory("set", block, data);
end

function InitializeConditions(block)
data = local_memory("get", block);
if isempty(data)
    return;
end
data.controller_state = [];
data.prev_pitch_deg = NaN;
data.prev_t = NaN;
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

[cmd, controllerState] = airdropx_mpc_controller(x, data.controller_state, cfg);
data.controller_state = controllerState;
local_memory("set", block, data);

block.OutputPort(1).Data = cmd(:);
end

function cfg = local_config_from_workspace()
targetH = local_base_scalar("airdropx_target_altitude_m", 20.0);
targetV = local_base_scalar("airdropx_pd_v_ref_mps", 45.0);
targetPitch = local_base_scalar("airdropx_pd_pitch_ref_deg", 4.0);
refMass = local_base_scalar("airdropx_mpc_reference_mass_kg", 3423.0);
refCg = local_base_scalar("airdropx_mpc_reference_cg_x_m", 5.28048992112182);
altitudeBias = local_base_scalar("airdropx_mpc_control_altitude_bias_m", 0.0);
cfg = airdropx_mpc_config( ...
    "TargetAltitudeM", targetH, ...
    "TargetAirspeedMps", targetV, ...
    "TargetPitchDeg", targetPitch, ...
    "ControlAltitudeBiasM", altitudeBias, ...
    "ReferenceMassKg", refMass, ...
    "ReferenceCgXM", refCg);
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
