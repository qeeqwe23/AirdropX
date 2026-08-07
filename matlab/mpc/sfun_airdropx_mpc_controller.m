function sfun_airdropx_mpc_controller(block)
%SFUN_AIRDROPX_MPC_CONTROLLER Grey-box longitudinal MPC for existing SLX.
%
% Input vector, unchanged from the preserved Simulink model:
%   [altitude_m; vz_up_mps; airspeed_mps; pitch_deg; mass_kg; cg_x_m]
%
% The preserved SLX provides pitch in deg. This adapter converts the reduced
% MPC state to SI angle units: theta* in rad and q in rad/s.
%
% Output vector:
%   [elevator_delta; throttle_cmd]

setup(block);
end

function setup(block)
block.NumDialogPrms = 1; % optional identified model .mat path

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
cfg = local_config_from_workspace();
modelMat = block.DialogPrm(1).Data;
if isstring(modelMat)
    modelMat = char(modelMat);
end
if ~isempty(modelMat) && isfile(modelMat)
    loaded = load(modelMat);
    if isfield(loaded, "cfg")
        cfg = loaded.cfg;
    end
    if isfield(loaded, "model_bank")
        cfg.model_bank = loaded.model_bank;
    elseif isfield(loaded, "model")
        cfg.model_bank = repmat({loaded.model}, 5, 1);
    end
end
cfg = local_apply_workspace_overrides(cfg);

data = struct();
data.cfg = cfg;
data.controller_state = [];
data.prev_pitch_deg = NaN;
data.prev_time_s = NaN;
data.q_filt_radps = 0.0;
local_memory("set", block, data);
end

function InitializeConditions(block)
data = local_memory("get", block);
if isempty(data)
    return;
end
data.controller_state = [];
data.prev_pitch_deg = NaN;
data.prev_time_s = NaN;
data.q_filt_radps = 0.0;
local_memory("set", block, data);
end

function Outputs(block)
data = local_memory("get", block);
if isempty(data)
    Start(block);
    data = local_memory("get", block);
end
cfg = data.cfg;
uIn = double(block.InputPort(1).Data(:));

altitudeM = uIn(1);
vzUpMps = uIn(2);
airspeedMps = uIn(3);
pitchDeg = uIn(4);
massKg = uIn(5);
cgXM = uIn(6);
dropCount = local_drop_count_from_mass(massKg, cfg);
trim = cfg.trim.bank(dropCount + 1);

[qRadps, data] = local_filtered_pitch_rate(block.CurrentTime, pitchDeg, data, cfg);
x = [
    altitudeM - cfg.reference.h_m
    vzUpMps
    airspeedMps - double(trim.Va0_mps)
    deg2rad(pitchDeg - double(trim.theta0_deg))
    qRadps
    massKg - double(cfg.trim.bank(1).m_kg)
    cgXM - double(trim.xCG_m)
    ];

[cmd, controllerState] = airdropx_mpc_controller(x, data.controller_state, cfg);
data.controller_state = controllerState;
local_memory("set", block, data);
block.OutputPort(1).Data = cmd(:);
end

function [qRadps, data] = local_filtered_pitch_rate(t, pitchDeg, data, cfg)
if isnan(data.prev_pitch_deg) || isnan(data.prev_time_s) || t <= data.prev_time_s
    qRawRadps = 0.0;
else
    dt = max(double(t) - double(data.prev_time_s), eps);
    qRawRadps = deg2rad((double(pitchDeg) - double(data.prev_pitch_deg)) / dt);
end

tau = 0.35;
if isfield(cfg, "estimator") && isfield(cfg.estimator, "PitchRateFilterTauS")
    tau = double(cfg.estimator.PitchRateFilterTauS);
end
dtNom = max(double(cfg.dt_s), eps);
alpha = dtNom / max(tau + dtNom, dtNom);
data.q_filt_radps = (1.0 - alpha) * double(data.q_filt_radps) + alpha * qRawRadps;
data.prev_pitch_deg = double(pitchDeg);
data.prev_time_s = double(t);
qRadps = data.q_filt_radps;
end

function dropCount = local_drop_count_from_mass(massKg, cfg)
cargoMass = mean(double(cfg.mass.cargo_mass_kg(:)), "omitnan");
if ~isfinite(cargoMass) || cargoMass <= 0.0
    cargoMass = 300.0;
end
dropCount = round(max(0.0, (double(cfg.trim.bank(1).m_kg) - double(massKg)) / cargoMass));
dropCount = min(max(dropCount, 0), double(cfg.mass.drop_count_max));
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

function cfg = local_apply_workspace_overrides(cfg)
try
    if evalin("base", "exist('airdropx_mpc_config_overrides','var')")
        overrides = evalin("base", "airdropx_mpc_config_overrides");
        if ~isempty(overrides)
            if ~isstruct(overrides)
                error("airdropx_mpc_config_overrides must be a struct.");
            end
            cfg = local_merge_struct(cfg, overrides);
        end
    end
    pitchBiasDeg = local_base_scalar("airdropx_mpc_control_pitch_bias_deg", 0.0);
    if isfinite(pitchBiasDeg) && abs(pitchBiasDeg) > 0.0
        cfg = local_apply_trim_pitch_bias(cfg, pitchBiasDeg);
    end
catch ME
    error("Failed to apply MPC config overrides: %s", ME.message);
end
end

function cfg = local_apply_trim_pitch_bias(cfg, pitchBiasDeg)
if isfield(cfg, "trim")
    if isfield(cfg.trim, "pitch_deg")
        cfg.trim.pitch_deg = double(cfg.trim.pitch_deg) + double(pitchBiasDeg);
        cfg.trim.pitch_rad = deg2rad(cfg.trim.pitch_deg);
    end
    if isfield(cfg.trim, "bank")
        for k = 1:numel(cfg.trim.bank)
            cfg.trim.bank(k).theta0_deg = double(cfg.trim.bank(k).theta0_deg) + double(pitchBiasDeg);
            cfg.trim.bank(k).theta0_rad = deg2rad(cfg.trim.bank(k).theta0_deg);
            cfg.trim.bank(k).alpha0_deg = double(cfg.trim.bank(k).alpha0_deg) + double(pitchBiasDeg);
            cfg.trim.bank(k).alpha0_rad = deg2rad(cfg.trim.bank(k).alpha0_deg);
        end
    end
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
        value = double(evalin("base", name));
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
