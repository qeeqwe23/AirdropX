function sfun_airdropx_auto_mpc_controller(block)
%SFUN_AIRDROPX_AUTO_MPC_CONTROLLER Guarded deviation-coordinate MPC bridge.
%
% Controller input from preserved SLX:
%   [altitude_m; vz_up_mps; airspeed_mps; pitch_deg; mass_kg; cg_x_m]
%
% In v13 the learned model remains in the exact coordinates used for ID:
%   delta input -> delta output.
% The controller therefore returns delta-u.  Only after MPC computation do we
% add the physical operating-point command and remove the hidden elevator trim
% used internally by the JSBSim S-Function.

setup(block);
end

function setup(block)
block.NumDialogPrms = 1;
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
block.RegBlockMethod("Terminate", @Terminate);
end

function Start(block)
bankPath = block.DialogPrm(1).Data;
if isstring(bankPath), bankPath = char(bankPath); end
if isempty(bankPath)
    bankPath = local_base_value("airdropx_auto_mpc_bank_mat_path", "");
end
if isempty(bankPath) || ~isfile(bankPath)
    error("AirdropX:AutoMPC:MissingBank", "Learned MPC bank not found: %s", string(bankPath));
end

S = load(bankPath);
data = struct();
data.controllers = local_load_controllers(S);
data.trim_bank = S.trim_bank;
data.mpc_meta = local_meta(S, data.trim_bank);
data.states = cell(size(data.controllers));
for k = 1:numel(data.controllers)
    if isempty(data.controllers{k}), continue; end
    data.states{k} = mpcstate(data.controllers{k});
    try, data.states{k}.LastMove = zeros(2,1); catch, end
end
data.prev_pitch_deg = NaN;
data.prev_time_s = NaN;
data.q_filt_dps = 0.0;
data.last_du = zeros(2,1);
data.last_cfg = 1;
data.fail_count = 0;
data.height_error_integral = zeros(max(1,numel(data.controllers)),1);
% v31 keeps one controller-memory state across cfg changes.
data.v31_height_vz_bias = 0.0;
% v31.2 height governor state.  Height is controlled only by a bounded/slew-
% limited vertical-speed reference; the inner MPC no longer tracks altitude
% directly when the governor is enabled.
data.v31_height_vz_cmd = 0.0;
% v31.3 dynamic-reference state. The requested Va command may jump at any
% time, while the governed reference presented to the inner MPC is rate-limited.
data.v31_speed_ref_cmd = NaN;
data.scheduler = local_load_speed_scheduler();
data.scheduler_states = local_init_scheduler_states(data.scheduler);
data.v31_last_physical_cmd = [NaN; NaN];
% v31.3.1 controller-internal reference-path instrumentation. Rows are
% buffered inside the S-function and exported once from Terminate so the
% trace reflects the references ACTUALLY consumed by the controller.
data.v31_reference_trace = zeros(0,23);
local_memory("set", block, data);
end

function InitializeConditions(block)
data = local_memory("get", block);
if isempty(data), return; end
data.prev_pitch_deg = NaN;
data.prev_time_s = NaN;
data.q_filt_dps = 0.0;
data.last_du = zeros(2,1);
data.last_cfg = 1;
data.fail_count = 0;
data.height_error_integral = zeros(max(1,numel(data.controllers)),1);
data.v31_height_vz_bias = 0.0;
data.v31_height_vz_cmd = 0.0;
data.v31_speed_ref_cmd = NaN;
data.scheduler_states = local_init_scheduler_states(data.scheduler);
data.v31_last_physical_cmd = [NaN; NaN];
data.v31_reference_trace = zeros(0,23);
for k = 1:numel(data.controllers)
    if isempty(data.controllers{k}), continue; end
    data.states{k} = mpcstate(data.controllers{k});
    try, data.states{k}.LastMove = zeros(2,1); catch, end
end
local_memory("set", block, data);
end

function Outputs(block)
data = local_memory("get", block);
if isempty(data)
    Start(block);
    data = local_memory("get", block);
end

uIn = double(block.InputPort(1).Data(:));
altitudeM = uIn(1);
vzUpMps = uIn(2);
airspeedMps = uIn(3);
pitchDeg = uIn(4);
massKg = uIn(5);
[qDps, data] = local_filtered_pitch_rate(block.CurrentTime, pitchDeg, data);

cfgIdx = local_config_index_from_mass(massKg, numel(data.controllers));
fixedCfg = local_base_scalar("airdropx_auto_fixed_config_id", NaN);
if isfinite(fixedCfg)
    cfgIdx = min(max(round(fixedCfg), 0), numel(data.controllers)-1) + 1;
end

if cfgIdx ~= data.last_cfg
    oldCfgIdx = data.last_cfg;
    v31Enabled = local_base_scalar("airdropx_v31_continuous_controller_state_enabled", 0.0) > 0.5;
    bumplessEnabled = local_base_scalar("airdropx_auto_bumpless_transition_enabled", 1.0) > 0.5;
    if v31Enabled && string(data.mpc_meta.input_coordinate_mode) == "deviation_physical"
        % v31 does not learn a cfg-to-cfg transfer scale.  The new MPC state
        % starts in its own deviation coordinates, while the actuator bridge
        % below enforces continuity in PHYSICAL command coordinates and one
        % global altitude->vz bias survives the configuration change.
        data = local_v31_config_transition(data,cfgIdx);
    elseif bumplessEnabled && string(data.mpc_meta.input_coordinate_mode) == "deviation_physical"
        data = local_bumpless_config_transition(data, oldCfgIdx, cfgIdx);
    else
        if ~isempty(data.controllers{cfgIdx})
            data.states{cfgIdx} = mpcstate(data.controllers{cfgIdx});
            try, data.states{cfgIdx}.LastMove = zeros(2,1); catch, end
        end
        data.last_du = zeros(2,1);
        if ~isfield(data,"height_error_integral") || numel(data.height_error_integral) < numel(data.controllers)
            data.height_error_integral = zeros(max(1,numel(data.controllers)),1);
        end
        data.height_error_integral(cfgIdx) = 0.0;
    end
    data.last_cfg = cfgIdx;
end

mode = string(data.mpc_meta.input_coordinate_mode);
if mode == "deviation_physical"
    if local_scheduler_active(data.scheduler)
        [plantCmd, data] = local_scheduled_deviation_move(block, data, cfgIdx, ...
            altitudeM, vzUpMps, airspeedMps, pitchDeg, qDps);
    else
        [plantCmd, data] = local_deviation_physical_move(block, data, cfgIdx, ...
            altitudeM, vzUpMps, airspeedMps, pitchDeg, qDps);
    end
else
    [plantCmd, data] = local_legacy_move(data, cfgIdx, altitudeM, vzUpMps, airspeedMps, pitchDeg, qDps);
end

data = local_trace_reference_path(block,data,cfgIdx,altitudeM,vzUpMps,airspeedMps,pitchDeg,qDps,plantCmd);
local_memory("set", block, data);
block.OutputPort(1).Data = plantCmd(:);
end


function Terminate(block)
% v31.3.1: export the controller-internal trace exactly once after simulation.
data = local_memory("get", block);
if isempty(data), return; end
try
    enabled = local_base_scalar("airdropx_v31_3_reference_instrumentation_enabled",0.0)>0.5;
    if enabled && isfield(data,"v31_reference_trace")
        assignin("base","airdropx_v31_3_controller_reference_trace",double(data.v31_reference_trace));
    end
catch
end
end

function data = local_trace_reference_path(block,data,cfgIdx,altitudeM,vzUpMps,airspeedMps,pitchDeg,qDps,plantCmd)
if local_base_scalar("airdropx_v31_3_reference_instrumentation_enabled",0.0)<=0.5
    return;
end
try
    r = local_reference_abs(cfgIdx,data.trim_bank,block.CurrentTime);
    reqH = double(r(1)); reqV = double(r(2));
    govV = reqV;
    if isfield(data,"v31_speed_ref_cmd") && isfinite(data.v31_speed_ref_cmd)
        govV = double(data.v31_speed_ref_cmd);
    end
    [kh,ki,vzLim,loV,hiV,wHi] = local_trace_governor_params(data,cfgIdx,airspeedMps);
    hErr = reqH-double(altitudeM);
    bias = 0.0; if isfield(data,"v31_height_vz_bias") && isfinite(data.v31_height_vz_bias), bias=double(data.v31_height_vz_bias); end
    rawVz = kh*hErr+bias;
    limVz = min(max(rawVz,-vzLim),vzLim);
    slewVz = NaN; if isfield(data,"v31_height_vz_cmd") && isfinite(data.v31_height_vz_cmd), slewVz=double(data.v31_height_vz_cmd); end
    trim=data.trim_bank(cfgIdx);
    yNom=[double(trim.altitude_m);double(trim.airspeed_mps);double(trim.pitch_deg);local_trim_field(trim,"vz_up_mps",0);local_trim_field(trim,"q_dps",0)];
    yAbs=[double(altitudeM);double(airspeedMps);double(pitchDeg);double(vzUpMps);double(qDps)];
    trustOk=double(local_inside_trust_region(yAbs-yNom));
    sched=double(local_scheduler_active(data.scheduler));
    pc=double(plantCmd(:)); if numel(pc)<2,pc=[NaN;NaN];end
    row=[double(block.CurrentTime),double(cfgIdx-1),reqH,reqV,govV,double(altitudeM),double(airspeedMps),hErr,kh,ki,vzLim,bias,rawVz,limVz,slewVz,double(vzUpMps),trustOk,sched,loV,hiV,wHi,pc(1),pc(2)];
    if ~isfield(data,"v31_reference_trace") || size(data.v31_reference_trace,2)~=23
        data.v31_reference_trace=zeros(0,23);
    end
    data.v31_reference_trace(end+1,:)=row;
catch
    % Instrumentation is diagnostic only and must never alter flight control.
end
end

function [kh,ki,vzLim,loV,hiV,wHi] = local_trace_governor_params(data,cfgIdx,airspeedMps)
kh=local_cfg_base_scalar("airdropx_auto_height_to_vz_gain_by_config","airdropx_auto_height_to_vz_gain",cfgIdx,0.0);
ki=local_cfg_base_scalar("airdropx_auto_height_integral_gain_by_config","airdropx_auto_height_integral_gain",cfgIdx,0.0);
vzLim=abs(local_cfg_base_scalar("airdropx_auto_height_vz_ref_limit_by_config","airdropx_auto_height_vz_ref_limit_mps",cfgIdx,0.8));
loV=NaN; hiV=NaN; wHi=0.0;
if local_scheduler_active(data.scheduler)
    speeds=double(data.scheduler.speed_nodes(:));
    [i0,i1,wHi]=local_speed_bracket(speeds,double(airspeedMps)); loV=speeds(i0); hiV=speeds(i1);
    kh=local_sched_param(data.scheduler,i0,i1,wHi,'height_gain_by_cfg',cfgIdx,kh);
    ki=local_sched_param(data.scheduler,i0,i1,wHi,'height_integral_by_cfg',cfgIdx,ki);
    vzLim=abs(local_sched_param(data.scheduler,i0,i1,wHi,'height_vz_limit_by_cfg',cfgIdx,vzLim));
end
end

function [plantCmd, data] = local_deviation_physical_move(block, data, cfgIdx, altitudeM, vzUpMps, airspeedMps, pitchDeg, qDps)
trim = data.trim_bank(cfgIdx);
yNom = [double(trim.altitude_m); double(trim.airspeed_mps); double(trim.pitch_deg); ...
    local_trim_field(trim, "vz_up_mps", 0.0); local_trim_field(trim, "q_dps", 0.0)];
yAbs = [double(altitudeM); double(airspeedMps); double(pitchDeg); double(vzUpMps); double(qDps)];
yDev = yAbs - yNom;

rAbs = local_reference_abs(cfgIdx, data.trim_bank, block.CurrentTime);
[rAbs(2), data] = local_speed_reference_governor(block.CurrentTime, rAbs(2), airspeedMps, data);

% v18: altitude is a slow kinematic state.  Use a PI outer loop to convert
% altitude error into a bounded vertical-speed reference.  The integral term
% is deliberately small and anti-windup limited; it exists specifically to
% remove the ~1 m long-tail bias that remained after v16's proportional-only
% height loop.
heightToVzGain = local_cfg_base_scalar( ...
    "airdropx_auto_height_to_vz_gain_by_config", ...
    "airdropx_auto_height_to_vz_gain", cfgIdx, 0.0);
heightIntegralGain = local_cfg_base_scalar( ...
    "airdropx_auto_height_integral_gain_by_config", ...
    "airdropx_auto_height_integral_gain", cfgIdx, 0.0);
heightVzLimit = abs(local_cfg_base_scalar( ...
    "airdropx_auto_height_vz_ref_limit_by_config", ...
    "airdropx_auto_height_vz_ref_limit_mps", cfgIdx, 0.8));

ctrl = data.controllers{cfgIdx};
du = zeros(2,1);
enableTime = local_base_scalar("airdropx_auto_mpc_enable_time_s", 2.0);
trustOk = local_inside_trust_region(yDev);

if ~isfield(data,"height_error_integral") || numel(data.height_error_integral) < numel(data.controllers)
    data.height_error_integral = zeros(max(1,numel(data.controllers)),1);
end
v31Enabled = local_base_scalar("airdropx_v31_continuous_controller_state_enabled", 0.0) > 0.5;
if ~isfield(data,"v31_height_vz_bias") || ~isfinite(data.v31_height_vz_bias)
    data.v31_height_vz_bias = 0.0;
end
if ~isfield(data,"v31_height_vz_cmd") || ~isfinite(data.v31_height_vz_cmd)
    data.v31_height_vz_cmd = 0.0;
end
v31Governor = v31Enabled && local_base_scalar("airdropx_v31_height_governor_enabled",0.0) > 0.5;
if isfinite(heightToVzGain) && heightToVzGain > 0 && isfinite(heightVzLimit) && heightVzLimit > 0
    hErr = double(rAbs(1)) - double(altitudeM);
    deadband = max(0.0, local_base_scalar("airdropx_auto_height_integral_deadband_m", 0.03));
    iErr = hErr; if abs(iErr) < deadband, iErr = 0.0; end
    if v31Governor
        % v31.2 single-channel height governor:
        %   altitude error -> bounded vz command -> inner MPC.
        % Altitude itself is removed from the inner MPC objective (bank Wh=0)
        % and its reference is set to the measured altitude as an additional
        % guard against stale nonzero altitude weights in an old bank.
        rAbs(1) = double(altitudeM);
        leak = min(max(local_base_scalar("airdropx_v31_height_bias_leak",1.0),0.95),1.0);
        biasFrac = min(max(local_base_scalar("airdropx_v31_height_bias_fraction",0.70),0.05),0.95);
        slewRate = max(0.01,local_base_scalar("airdropx_v31_height_vz_slew_rate_mps2",0.30));
        dt = 0.1;
        data.v31_height_vz_bias = leak * data.v31_height_vz_bias;
        biasMax = biasFrac * heightVzLimit;

        % Conditional anti-windup. Integrate only when the command is not
        % saturated, or when the current altitude error drives it BACK toward
        % the admissible interval. This replaces v31.1's integrate-then-clip
        % behavior which could retain excess bias after a payload transient.
        rawBefore = heightToVzGain*hErr + data.v31_height_vz_bias;
        upperSat = rawBefore >= heightVzLimit;
        lowerSat = rawBefore <= -heightVzLimit;
        pushesFurtherIntoSat = (upperSat && iErr>0) || (lowerSat && iErr<0);
        if block.CurrentTime >= enableTime && trustOk && isfinite(heightIntegralGain) && ...
                heightIntegralGain > 0 && ~pushesFurtherIntoSat
            data.v31_height_vz_bias = data.v31_height_vz_bias + heightIntegralGain*iErr*dt;
        end
        data.v31_height_vz_bias = min(max(data.v31_height_vz_bias,-biasMax),biasMax);
        desiredVz = min(max(heightToVzGain*hErr + data.v31_height_vz_bias, ...
            -heightVzLimit),heightVzLimit);

        % Slew-limit the reference itself. Payload release is a discontinuous
        % plant change, but the commanded vertical speed does not need to jump.
        maxStep = slewRate*dt;
        prevVz = double(data.v31_height_vz_cmd);
        vzRef = min(max(desiredVz,prevVz-maxStep),prevVz+maxStep);
        data.v31_height_vz_cmd = vzRef;
    elseif v31Enabled
        % v31.1 compatibility path retained for non-v31.2 callers.
        leak = min(max(local_base_scalar("airdropx_auto_height_integral_leak", 0.999), 0.95), 1.0);
        iFrac = min(max(local_base_scalar("airdropx_auto_height_integral_fraction", 0.35), 0.05), 0.75);
        data.v31_height_vz_bias = leak * data.v31_height_vz_bias;
        if block.CurrentTime >= enableTime && trustOk && isfinite(heightIntegralGain) && heightIntegralGain > 0
            data.v31_height_vz_bias = data.v31_height_vz_bias + heightIntegralGain * iErr * 0.1;
        end
        biasMax = iFrac * heightVzLimit;
        data.v31_height_vz_bias = min(max(data.v31_height_vz_bias,-biasMax),biasMax);
        vzRef = heightToVzGain * hErr + data.v31_height_vz_bias;
    else
        leak = min(max(local_base_scalar("airdropx_auto_height_integral_leak", 0.999), 0.95), 1.0);
        iFrac = min(max(local_base_scalar("airdropx_auto_height_integral_fraction", 0.35), 0.05), 0.75);
        if block.CurrentTime >= enableTime && trustOk && isfinite(heightIntegralGain) && heightIntegralGain > 0
            data.height_error_integral(cfgIdx) = leak * data.height_error_integral(cfgIdx) + iErr * 0.1;
            iMax = (iFrac * heightVzLimit) / max(heightIntegralGain, 1.0e-9);
            data.height_error_integral(cfgIdx) = min(max(data.height_error_integral(cfgIdx), -iMax), iMax);
        end
        vzRef = heightToVzGain * hErr;
        if isfinite(heightIntegralGain) && heightIntegralGain > 0
            vzRef = vzRef + heightIntegralGain * data.height_error_integral(cfgIdx);
        end
    end
    rAbs(4) = min(max(vzRef, -heightVzLimit), heightVzLimit);
end
rDev = rAbs - yNom;
authority = min(max(local_cfg_base_scalar( ...
    "airdropx_auto_mpc_authority_by_config", ...
    "airdropx_auto_mpc_authority_scale", cfgIdx, 1.0), 0.0), 1.0);
if block.CurrentTime >= enableTime && trustOk && ~isempty(ctrl)
    try
        [duCandidate, info] = local_mpcmove(ctrl, data.states{cfgIdx}, yDev, rDev);
        if local_qp_failed(info)
            data.fail_count = data.fail_count + 1;
            % data.last_du is already the REAL move sent to the plant.
            % Do not apply authority a second time to a held fallback move.
            du = data.last_du;
        else
            du = authority * double(duCandidate(:));
        end
    catch
        data.fail_count = data.fail_count + 1;
        du = data.last_du;
    end
elseif block.CurrentTime >= enableTime && ~trustOk
    % Only the fast aerodynamic states define local-model validity in v14.
    % If one of them exits the identified neighborhood, retreat gradually
    % from the last REAL command instead of snapping to the biased nominal.
    du = 0.90 * data.last_du;
end

if numel(du) < 2, du = zeros(2,1); end

elevLim = local_meta_scalar(data.mpc_meta, "elevator_deviation_limit", 0.035);
thrLim = local_meta_scalar(data.mpc_meta, "throttle_deviation_limit", 0.060);
du(1) = min(max(du(1), -elevLim), elevLim);
du(2) = min(max(du(2), -thrLim), thrLim);

% Explicit rate guard in addition to the MPC MV-rate constraint. This is a
% final bridge-level guarantee that no estimator/QP anomaly can jump outside
% the local ID envelope in one sample.
eStep = local_base_scalar("airdropx_auto_elevator_dev_step_limit", ...
    local_meta_scalar(data.mpc_meta, "elevator_deviation_rate_limit", 0.006));
tStep = local_base_scalar("airdropx_auto_throttle_dev_step_limit", ...
    local_meta_scalar(data.mpc_meta, "throttle_deviation_rate_limit", 0.010));
if ~v31Enabled
    du(1) = min(max(du(1), data.last_du(1)-eStep), data.last_du(1)+eStep);
    du(2) = min(max(du(2), data.last_du(2)-tStep), data.last_du(2)+tStep);
    data.last_du = du;
    % Legacy/v30 state is stored in deviation coordinates.
    try, data.states{cfgIdx}.LastMove = du; catch, end
end

physicalNom = double(data.mpc_meta.physical_elevator_nominals(cfgIdx));
if ~isfinite(physicalNom)
    error("AirdropX:AutoMPC:MissingPhysicalElevatorNominal", ...
        "Physical elevator nominal is missing for cfg%d.", cfgIdx-1);
end
hiddenTrim = local_base_scalar("airdropx_auto_hidden_elevator_trim", NaN);
if ~isfinite(hiddenTrim)
    error("AirdropX:AutoMPC:MissingHiddenElevatorTrim", ...
        ['airdropx_auto_hidden_elevator_trim is not set. ' ...
         'Calibrate it for the current initial condition before closed-loop smoke testing.']);
end

physicalElevator = physicalNom + du(1);
throttleCmd = double(trim.throttle_cmd) + du(2);

if v31Enabled
    % v31 actuator memory lives in PHYSICAL command coordinates.  On a cfg
    % switch the new controller may immediately ask for its new trim/feedforward,
    % but the actual actuator command can only move by the same rate guard used
    % in normal operation. This is continuous without freezing the old cfg's
    % deviation state (the failure mode observed with full 1/1 transfer).
    if ~isfield(data,"v31_last_physical_cmd") || numel(data.v31_last_physical_cmd)<2
        data.v31_last_physical_cmd=[NaN;NaN];
    end
    lastPhys=double(data.v31_last_physical_cmd(:));
    if numel(lastPhys)>=2 && all(isfinite(lastPhys(1:2)))
        physicalElevator=min(max(physicalElevator,lastPhys(1)-eStep),lastPhys(1)+eStep);
        throttleCmd=min(max(throttleCmd,lastPhys(2)-tStep),lastPhys(2)+tStep);
    end
    data.v31_last_physical_cmd=[physicalElevator;throttleCmd];
    data.last_du=[physicalElevator-physicalNom; throttleCmd-double(trim.throttle_cmd)];
    try, data.states{cfgIdx}.LastMove=data.last_du; catch, end
end

% v16 certification disturbance is deliberately exogenous: it is not added
% to LastMove.  The controller must observe the resulting state error and
% recover with its own learned-model command.
[pulseElevator, pulseThrottle] = local_test_disturbance(block.CurrentTime);
physicalElevator = physicalElevator + pulseElevator;
throttleCmd = throttleCmd + pulseThrottle;
% Keep the TOTAL plant input (controller + exogenous certification pulse)
% inside the v11 local-ID envelope.
physicalElevator = min(max(physicalElevator, physicalNom - elevLim), physicalNom + elevLim);
throttleCmd = min(max(throttleCmd, double(trim.throttle_cmd) - thrLim), ...
    double(trim.throttle_cmd) + thrLim);

externalElevatorDelta = physicalElevator - hiddenTrim;
plantCmd = [externalElevatorDelta; throttleCmd];
plantCmd(1) = min(max(plantCmd(1), -0.85), 0.85);
plantCmd(2) = min(max(plantCmd(2), 0.0), 1.0);
end

function tf = local_inside_trust_region(yDev)
% v14: altitude error is NOT a local-model validity gate.
% Altitude is predominantly a kinematic translation state. Turning MPC off
% merely because |h-h_ref| exceeds 5 m removes the controller exactly when it
% must recover the altitude error. Local validity is enforced on the fast
% aerodynamic states instead.
%
% yDev order: [altitude; airspeed; pitch; vz; q]
fastDev = double(yDev(2:5));
limits = [ ...
    local_base_scalar("airdropx_auto_trust_V_mps", 4.0); ...
    local_base_scalar("airdropx_auto_trust_pitch_deg", 4.0); ...
    local_base_scalar("airdropx_auto_trust_vz_mps", 2.5); ...
    local_base_scalar("airdropx_auto_trust_q_dps", 4.0)];
tf = all(isfinite(fastDev)) && all(abs(fastDev(:)) <= limits);
end


function data = local_v31_config_transition(data,newCfgIdx)
% v31 universal cfg transition: controller model state is reinitialized in
% the new cfg coordinates, but physical actuator memory and the global
% altitude->vz bias are NOT reset. Physical rate continuity is enforced after
% the new controller computes its first command.
nCtrl=numel(data.controllers); newCfgIdx=min(max(round(double(newCfgIdx)),1),nCtrl);
if ~isempty(data.controllers{newCfgIdx})
    data.states{newCfgIdx}=mpcstate(data.controllers{newCfgIdx});
    try, data.states{newCfgIdx}.LastMove=zeros(2,1); catch, end
end
data.last_du=zeros(2,1);
if ~isfield(data,"v31_height_vz_bias")||~isfinite(data.v31_height_vz_bias), data.v31_height_vz_bias=0.0; end
if ~isfield(data,"v31_height_vz_cmd")||~isfinite(data.v31_height_vz_cmd), data.v31_height_vz_cmd=0.0; end
if ~isfield(data,"v31_last_physical_cmd")||numel(data.v31_last_physical_cmd)<2, data.v31_last_physical_cmd=[NaN;NaN]; end
% Reinitialize the new cfg state for every speed node as well.  The final
% physical command remains rate-continuous and is mapped back into each node's
% deviation coordinates after the scheduled blend is computed.
if local_scheduler_active(data.scheduler)
    for n=1:numel(data.scheduler.nodes)
        try
            ctrl=data.scheduler.nodes(n).controllers{newCfgIdx};
            if ~isempty(ctrl)
                data.scheduler_states{n,newCfgIdx}=mpcstate(ctrl);
                try, data.scheduler_states{n,newCfgIdx}.LastMove=zeros(2,1); catch, end
            end
        catch
        end
    end
end
end

function scheduler=local_load_speed_scheduler()
scheduler=struct('enabled',false,'speed_nodes',[],'nodes',struct([]));
if local_base_scalar("airdropx_v31_3_scheduler_enabled",0.0)<=0.5, return; end
p=string(local_base_value("airdropx_v31_3_scheduler_bank_mat_path",""));
if strlength(p)==0 || ~isfile(p), return; end
try
    S=load(char(p));
    if isfield(S,'scheduler'), C=S.scheduler; else, C=S; end
    if ~isfield(C,'speed_nodes') || ~isfield(C,'nodes'), return; end
    v=double(C.speed_nodes(:));
    if numel(v)<2 || numel(C.nodes)~=numel(v), return; end
    [v,ix]=sort(v); C.nodes=C.nodes(ix); C.speed_nodes=v;
    scheduler=C; scheduler.enabled=true;
catch
    scheduler=struct('enabled',false,'speed_nodes',[],'nodes',struct([]));
end
end

function tf=local_scheduler_active(scheduler)
tf=false;
try
    tf=isstruct(scheduler) && isfield(scheduler,'enabled') && logical(scheduler.enabled) && ...
        isfield(scheduler,'speed_nodes') && numel(scheduler.speed_nodes)>=2 && ...
        isfield(scheduler,'nodes') && numel(scheduler.nodes)>=2;
catch
end
end

function states=local_init_scheduler_states(scheduler)
states=cell(0,0);
if ~local_scheduler_active(scheduler), return; end
n=numel(scheduler.nodes); states=cell(n,5);
for i=1:n
    try
        ctrls=scheduler.nodes(i).controllers;
        for k=1:min(5,numel(ctrls))
            if isempty(ctrls{k}), continue; end
            states{i,k}=mpcstate(ctrls{k});
            try, states{i,k}.LastMove=zeros(2,1); catch, end
        end
    catch
    end
end
end

function [plantCmd,data]=local_scheduled_deviation_move(block,data,cfgIdx,altitudeM,vzUpMps,airspeedMps,pitchDeg,qDps)
% v31.3 continuous speed scheduling by command blending.  Each verified speed
% node owns its own MPC/trim/state.  The two nodes bracketing current Va are
% evaluated, their ABSOLUTE physical commands are linearly blended, then one
% global actuator rate guard is applied. This avoids hard controller switches.
S=data.scheduler; speeds=double(S.speed_nodes(:));
[i0,i1,w]=local_speed_bracket(speeds,double(airspeedMps));
[rReqH,rReqV]=local_requested_reference(block.CurrentTime,cfgIdx,data.trim_bank);
[rV,data]=local_speed_reference_governor(block.CurrentTime,rReqV,airspeedMps,data);
% Height-governor parameters are scheduled continuously with speed as well.
kh=local_sched_param(S,i0,i1,w,'height_gain_by_cfg',cfgIdx, ...
    local_cfg_base_scalar("airdropx_auto_height_to_vz_gain_by_config","airdropx_auto_height_to_vz_gain",cfgIdx,0.0));
ki=local_sched_param(S,i0,i1,w,'height_integral_by_cfg',cfgIdx, ...
    local_cfg_base_scalar("airdropx_auto_height_integral_gain_by_config","airdropx_auto_height_integral_gain",cfgIdx,0.0));
vzLim=abs(local_sched_param(S,i0,i1,w,'height_vz_limit_by_cfg',cfgIdx, ...
    local_cfg_base_scalar("airdropx_auto_height_vz_ref_limit_by_config","airdropx_auto_height_vz_ref_limit_mps",cfgIdx,0.8)));
[vzRef,data]=local_height_governor(block,data,rReqH,altitudeM,kh,ki,vzLim);

[cmd0,ok0,data]=local_scheduler_node_command(block,data,i0,cfgIdx,altitudeM,vzUpMps,airspeedMps,pitchDeg,qDps,rV,vzRef);
if i1==i0
    cmd=cmd0; ok=ok0;
else
    [cmd1,ok1,data]=local_scheduler_node_command(block,data,i1,cfgIdx,altitudeM,vzUpMps,airspeedMps,pitchDeg,qDps,rV,vzRef);
    if ok0 && ok1, cmd=(1-w)*cmd0+w*cmd1; ok=true;
    elseif ok0, cmd=cmd0; ok=true;
    elseif ok1, cmd=cmd1; ok=true;
    else, cmd=[NaN;NaN]; ok=false; end
end
if ~ok || numel(cmd)<2 || any(~isfinite(cmd(1:2)))
    last=double(data.v31_last_physical_cmd(:));
    if numel(last)>=2 && all(isfinite(last(1:2))), cmd=last(1:2);
    else
        node=S.nodes(i0); cmd=[local_node_physical_nominal(node,cfgIdx); double(node.trim_bank(cfgIdx).throttle_cmd)];
    end
end
% One physical actuator continuity guard after blending.
eStep=local_base_scalar("airdropx_auto_elevator_dev_step_limit",0.006);
tStep=local_base_scalar("airdropx_auto_throttle_dev_step_limit",0.010);
last=double(data.v31_last_physical_cmd(:));
if numel(last)>=2 && all(isfinite(last(1:2)))
    cmd(1)=min(max(cmd(1),last(1)-eStep),last(1)+eStep);
    cmd(2)=min(max(cmd(2),last(2)-tStep),last(2)+tStep);
end
% Synchronize active node LastMove states to the REAL blended command.
for n=unique([i0 i1])
    try
        node=S.nodes(n); nom=[local_node_physical_nominal(node,cfgIdx);double(node.trim_bank(cfgIdx).throttle_cmd)];
        du=cmd(:)-nom(:);
        em=local_node_meta_scalar(node,'elevator_deviation_limit',0.035);
        tm=local_node_meta_scalar(node,'throttle_deviation_limit',0.060);
        du(1)=min(max(du(1),-em),em); du(2)=min(max(du(2),-tm),tm);
        if ~isempty(data.scheduler_states{n,cfgIdx}), data.scheduler_states{n,cfgIdx}.LastMove=du; end
    catch
    end
end
data.v31_last_physical_cmd=cmd(:);
% Exogenous certification disturbance remains outside controller memory.
[pE,pT]=local_test_disturbance(block.CurrentTime); cmd=cmd(:)+[pE;pT];
hiddenTrim=local_base_scalar("airdropx_auto_hidden_elevator_trim",NaN);
if ~isfinite(hiddenTrim), error("AirdropX:AutoMPC:MissingHiddenElevatorTrim","airdropx_auto_hidden_elevator_trim is not set."); end
plantCmd=[min(max(cmd(1)-hiddenTrim,-0.85),0.85); min(max(cmd(2),0.0),1.0)];
end

function [cmd,ok,data]=local_scheduler_node_command(block,data,nodeIdx,cfgIdx,altitudeM,vzUpMps,airspeedMps,pitchDeg,qDps,vRef,vzRef)
node=data.scheduler.nodes(nodeIdx); ok=false; cmd=[NaN;NaN];
try
    trim=node.trim_bank(cfgIdx); ctrl=node.controllers{cfgIdx};
    if isempty(ctrl), return; end
    yNom=[double(trim.altitude_m);double(trim.airspeed_mps);double(trim.pitch_deg);local_trim_field(trim,"vz_up_mps",0);local_trim_field(trim,"q_dps",0)];
    yAbs=[double(altitudeM);double(airspeedMps);double(pitchDeg);double(vzUpMps);double(qDps)];
    yDev=yAbs-yNom;
    % Altitude is not an inner-MPC objective in v31.2+.
    rAbs=[double(altitudeM);double(vRef);double(trim.pitch_deg);double(vzRef);0.0];
    rDev=rAbs-yNom;
    if block.CurrentTime<local_base_scalar("airdropx_auto_mpc_enable_time_s",2.0) || ~local_inside_trust_region(yDev)
        du=zeros(2,1);
    else
        st=data.scheduler_states{nodeIdx,cfgIdx};
        [cand,info]=local_mpcmove(ctrl,st,yDev,rDev);
        if local_qp_failed(info), return; end
        auth=local_node_cfg_param(node,'authority_by_cfg',cfgIdx,1.0);
        du=auth*double(cand(:));
    end
    if numel(du)<2, du=zeros(2,1); end
    em=local_node_meta_scalar(node,'elevator_deviation_limit',0.035);
    tm=local_node_meta_scalar(node,'throttle_deviation_limit',0.060);
    du(1)=min(max(du(1),-em),em); du(2)=min(max(du(2),-tm),tm);
    nom=[local_node_physical_nominal(node,cfgIdx);double(trim.throttle_cmd)];
    cmd=nom+du(1:2); ok=all(isfinite(cmd));
catch
    ok=false;
end
end

function [vzRef,data]=local_height_governor(block,data,targetH,altitudeM,kh,ki,vzLim)
if ~isfinite(vzLim)||vzLim<=0||~isfinite(kh)||kh<=0, vzRef=0; return; end
if ~isfield(data,'v31_height_vz_bias')||~isfinite(data.v31_height_vz_bias), data.v31_height_vz_bias=0; end
if ~isfield(data,'v31_height_vz_cmd')||~isfinite(data.v31_height_vz_cmd), data.v31_height_vz_cmd=0; end
hErr=double(targetH)-double(altitudeM); dead=max(0,local_base_scalar("airdropx_auto_height_integral_deadband_m",0.03));
iErr=hErr; if abs(iErr)<dead, iErr=0; end
leak=min(max(local_base_scalar("airdropx_v31_height_bias_leak",1.0),0.95),1.0);
biasFrac=min(max(local_base_scalar("airdropx_v31_height_bias_fraction",0.70),0.05),0.95);
slew=max(0.01,local_base_scalar("airdropx_v31_height_vz_slew_rate_mps2",0.30)); dt=max(1e-6,local_base_scalar("airdropx_auto_mpc_sample_time_s",0.1));
data.v31_height_vz_bias=leak*data.v31_height_vz_bias; biasMax=biasFrac*vzLim;
raw=kh*hErr+data.v31_height_vz_bias; push=(raw>=vzLim&&iErr>0)||(raw<=-vzLim&&iErr<0);
if block.CurrentTime>=local_base_scalar("airdropx_auto_mpc_enable_time_s",2.0)&&isfinite(ki)&&ki>0&&~push
    data.v31_height_vz_bias=data.v31_height_vz_bias+ki*iErr*dt;
end
data.v31_height_vz_bias=min(max(data.v31_height_vz_bias,-biasMax),biasMax);
desired=min(max(kh*hErr+data.v31_height_vz_bias,-vzLim),vzLim);
step=slew*dt; prev=double(data.v31_height_vz_cmd);
vzRef=min(max(desired,prev-step),prev+step); data.v31_height_vz_cmd=vzRef;
end

function [h,v]=local_requested_reference(t,cfgIdx,trimBank)
r=local_reference_abs(cfgIdx,trimBank,t); h=r(1); v=r(2);
end

function [i0,i1,w]=local_speed_bracket(nodes,v)
n=numel(nodes); if v<=nodes(1),i0=1;i1=1;w=0;return;end
if v>=nodes(end),i0=n;i1=n;w=0;return;end
i1=find(nodes>=v,1,'first'); i0=i1-1; den=nodes(i1)-nodes(i0); if den<=0,w=0;else,w=(v-nodes(i0))/den;end
w=min(max(w,0),1);
end

function x=local_sched_param(S,i0,i1,w,field,cfgIdx,fallback)
a=local_node_cfg_param(S.nodes(i0),field,cfgIdx,fallback); b=local_node_cfg_param(S.nodes(i1),field,cfgIdx,a); x=(1-w)*a+w*b;
end
function x=local_node_cfg_param(node,field,cfgIdx,fallback)
x=fallback; try,v=double(node.(field));v=v(:);if cfgIdx<=numel(v)&&isfinite(v(cfgIdx)),x=v(cfgIdx);end,catch,end
end
function x=local_node_physical_nominal(node,cfgIdx)
x=NaN; try,v=double(node.mpc_meta.physical_elevator_nominals(:));x=v(cfgIdx);catch,end
if ~isfinite(x), x=double(node.trim_bank(cfgIdx).elevator_cmd); end
end
function x=local_node_meta_scalar(node,field,fallback)
x=fallback; try,v=double(node.mpc_meta.(field));if isscalar(v)&&isfinite(v),x=v;end,catch,end
end

function data = local_bumpless_config_transition(data, oldCfgIdx, newCfgIdx)
% v30.6: universal bumpless transfer for every cfg transition.
% Keep the REAL physical controller command continuous while changing the
% deviation-coordinate origin, and preserve the altitude-integral contribution
% rather than resetting it merely because a payload configuration changed.
%
% This contains no altitude/cfg special case: every physical cfg transition
% uses the same coordinate conversion and contribution-preserving rule.

nCtrl = numel(data.controllers);
oldCfgIdx = min(max(round(double(oldCfgIdx)),1),nCtrl);
newCfgIdx = min(max(round(double(newCfgIdx)),1),nCtrl);
if ~isfield(data,"height_error_integral") || numel(data.height_error_integral) < nCtrl
    data.height_error_integral = zeros(max(1,nCtrl),1);
end

% Recreate the new controller state because the Plant/controller dimensions
% may differ between cfgs, but seed LastMove with the physical command mapped
% into the NEW deviation coordinates instead of forcing a zero move.
if ~isempty(data.controllers{newCfgIdx})
    data.states{newCfgIdx} = mpcstate(data.controllers{newCfgIdx});
end

moveScale = min(max(local_base_scalar("airdropx_auto_transition_move_transfer_scale",0.0),0.0),1.5);
oldDu = double(data.last_du(:));
if numel(oldDu) < 2 || any(~isfinite(oldDu(1:2)))
    oldDu = zeros(2,1);
else
    oldDu = oldDu(1:2);
end

oldPhysicalNom = NaN; newPhysicalNom = NaN;
try
    oldPhysicalNom = double(data.mpc_meta.physical_elevator_nominals(oldCfgIdx));
    newPhysicalNom = double(data.mpc_meta.physical_elevator_nominals(newCfgIdx));
catch
end
oldThrottleNom = double(data.trim_bank(oldCfgIdx).throttle_cmd);
newThrottleNom = double(data.trim_bank(newCfgIdx).throttle_cmd);

if isfinite(oldPhysicalNom) && isfinite(newPhysicalNom) && ...
        isfinite(oldThrottleNom) && isfinite(newThrottleNom)
    % The command sent immediately before the drop, expressed in absolute
    % physical actuator coordinates.
    oldPhysicalCmd = [oldPhysicalNom + oldDu(1); oldThrottleNom + oldDu(2)];
    % Express exactly that command around the new cfg nominal.  moveScale=1
    % is full physical-command continuity; 0 is the legacy reset baseline and intermediate values retain only a
    % fraction of the transferred move and are available to the universal
    % mission near-pass learner (same rule for every context).
    newDuExact = [oldPhysicalCmd(1) - newPhysicalNom; oldPhysicalCmd(2) - newThrottleNom];
    newDu = moveScale * newDuExact;
else
    newDu = moveScale * oldDu;
end

% Respect the same local-ID deviation envelope used everywhere else.
elevLim = local_meta_scalar(data.mpc_meta, "elevator_deviation_limit", 0.035);
thrLim = local_meta_scalar(data.mpc_meta, "throttle_deviation_limit", 0.060);
newDu(1) = min(max(newDu(1),-elevLim),elevLim);
newDu(2) = min(max(newDu(2),-thrLim),thrLim);
data.last_du = newDu;
try
    if ~isempty(data.states{newCfgIdx})
        data.states{newCfgIdx}.LastMove = newDu;
    end
catch
end

% Transfer the actual vertical-speed contribution Ki*I, not the raw integral
% state.  This remains continuous even when adjacent cfg controllers learned
% different Ki values.  The new integral is then clipped by the SAME global
% anti-windup rule used during normal integration.
oldKi = local_cfg_base_scalar( ...
    "airdropx_auto_height_integral_gain_by_config", ...
    "airdropx_auto_height_integral_gain", oldCfgIdx, 0.0);
newKi = local_cfg_base_scalar( ...
    "airdropx_auto_height_integral_gain_by_config", ...
    "airdropx_auto_height_integral_gain", newCfgIdx, 0.0);
intScale = min(max(local_base_scalar("airdropx_auto_transition_integral_transfer_scale",0.0),0.0),2.0);
oldContribution = 0.0;
if isfinite(oldKi) && oldKi > 0 && isfinite(data.height_error_integral(oldCfgIdx))
    oldContribution = oldKi * data.height_error_integral(oldCfgIdx);
end
if isfinite(newKi) && newKi > 0
    newI = intScale * oldContribution / newKi;
    newVzLimit = abs(local_cfg_base_scalar( ...
        "airdropx_auto_height_vz_ref_limit_by_config", ...
        "airdropx_auto_height_vz_ref_limit_mps", newCfgIdx, 0.8));
    iFrac = min(max(local_base_scalar("airdropx_auto_height_integral_fraction",0.35),0.05),0.75);
    iMax = (iFrac * newVzLimit) / max(newKi,1.0e-9);
    newI = min(max(newI,-iMax),iMax);
    data.height_error_integral(newCfgIdx) = newI;
else
    data.height_error_integral(newCfgIdx) = 0.0;
end
end

function [plantCmd, data] = local_legacy_move(data, cfgIdx, altitudeM, vzUpMps, airspeedMps, pitchDeg, qDps)
ctrl = data.controllers{cfgIdx};
trimU = [double(data.trim_bank(cfgIdx).elevator_cmd); double(data.trim_bank(cfgIdx).throttle_cmd)];
if isempty(ctrl)
    cmd = trimU;
else
    y = [altitudeM; airspeedMps; pitchDeg; vzUpMps; qDps];
    r = local_reference_abs(cfgIdx, data.trim_bank, 0.0);
    try
        [cmd, info] = local_mpcmove(ctrl, data.states{cfgIdx}, y, r);
        if local_qp_failed(info), cmd = trimU; end
    catch
        cmd = trimU;
    end
end
plantCmd = double(cmd(:));
if numel(plantCmd) < 2, plantCmd = trimU; end
end

function [cmd, info] = local_mpcmove(ctrl, state, y, r)
info = struct();
try
    [cmd, info] = mpcmove(ctrl, state, y, r);
catch ME
    if contains(string(ME.message), "Too many output")
        cmd = mpcmove(ctrl, state, y, r);
    else
        rethrow(ME);
    end
end
end

function tf = local_qp_failed(info)
tf = false;
if isstruct(info) && isfield(info, "QPCode") && isnumeric(info.QPCode) && isscalar(info.QPCode)
    tf = double(info.QPCode) < 0;
end
end

function controllers = local_load_controllers(S)
if isfield(S, "controllers")
    controllers = S.controllers;
else
    controllers = cell(5,1);
    for k = 1:5
        name = sprintf("MPC%d", k-1);
        if isfield(S, name), controllers{k} = S.(name); end
    end
end
controllers = controllers(:);
if isempty(controllers) || all(cellfun(@isempty, controllers))
    error("AirdropX:AutoMPC:MissingControllers", "MPC bank has no controllers.");
end
end

function meta = local_meta(S, trimBank)
if isfield(S, "mpc_meta")
    meta = S.mpc_meta;
else
    meta = struct();
    meta.input_coordinate_mode = "legacy_absolute";
    meta.physical_elevator_nominals = arrayfun(@(s) double(s.elevator_cmd), trimBank(:));
    meta.elevator_deviation_limit = 0.035;
    meta.throttle_deviation_limit = 0.060;
    meta.elevator_deviation_rate_limit = 0.006;
    meta.throttle_deviation_rate_limit = 0.010;
end
end

function r = local_reference_abs(cfgIdx, trimBank, t)
if nargin<3, t=0.0; end
targetH = local_base_scalar("airdropx_target_altitude_m", trimBank(cfgIdx).altitude_m);
targetV = local_base_scalar("airdropx_pd_v_ref_mps", trimBank(cfgIdx).airspeed_mps);
if local_base_scalar("airdropx_v31_3_dynamic_reference_enabled",0.0)>0.5
    profile=local_reference_profile();
    if ~isempty(profile)
        idx=find(profile(:,1)<=double(t)+1e-12,1,'last');
        if isempty(idx), idx=1; end
        targetH=profile(idx,2); targetV=profile(idx,3);
    end
end
if local_base_scalar("airdropx_auto_use_trim_pitch_reference", 1.0) > 0.5
    targetPitch = trimBank(cfgIdx).pitch_deg;
else
    targetPitch = local_base_scalar("airdropx_pd_pitch_ref_deg", trimBank(cfgIdx).pitch_deg);
end
r = [targetH; targetV; targetPitch; 0.0; 0.0];
end

function profile=local_reference_profile()
profile=[];
try
    if evalin("base","exist('airdropx_v31_3_reference_profile','var')")
        P=double(evalin("base","airdropx_v31_3_reference_profile"));
        if size(P,2)==3 && ~isempty(P)
            P=P(all(isfinite(P),2),:); P=sortrows(P,1);
            profile=P;
        end
    end
catch
end
end

function [vRef,data]=local_speed_reference_governor(t,requestedV,actualV,data)
requestedV=double(requestedV);
if ~isfield(data,"v31_speed_ref_cmd") || ~isfinite(data.v31_speed_ref_cmd)
    if isfinite(actualV), data.v31_speed_ref_cmd=double(actualV); else, data.v31_speed_ref_cmd=requestedV; end
end
if local_base_scalar("airdropx_v31_3_dynamic_reference_enabled",0.0)<=0.5 || ...
        local_base_scalar("airdropx_v31_3_speed_governor_enabled",1.0)<=0.5
    data.v31_speed_ref_cmd=requestedV; vRef=requestedV; return;
end
acc=max(0.01,local_base_scalar("airdropx_v31_3_speed_accel_limit_mps2",0.75));
dec=max(0.01,local_base_scalar("airdropx_v31_3_speed_decel_limit_mps2",1.00));
dt=max(1e-6,local_base_scalar("airdropx_auto_mpc_sample_time_s",0.1));
dv=requestedV-double(data.v31_speed_ref_cmd);
if dv>=0, lim=acc*dt; else, lim=dec*dt; end
data.v31_speed_ref_cmd=double(data.v31_speed_ref_cmd)+min(max(dv,-lim),lim);
vRef=double(data.v31_speed_ref_cmd);
end

function [qDps, data] = local_filtered_pitch_rate(t, pitchDeg, data)
if isnan(data.prev_pitch_deg) || isnan(data.prev_time_s) || t <= data.prev_time_s
    qRawDps = 0.0;
else
    dt = max(double(t) - double(data.prev_time_s), eps);
    qRawDps = (double(pitchDeg) - double(data.prev_pitch_deg)) / dt;
end
tau = local_base_scalar("airdropx_auto_pitch_rate_filter_tau_s", 0.35);
dtNom = local_base_scalar("airdropx_auto_mpc_sample_time_s", 0.1);
alpha = dtNom / max(tau + dtNom, dtNom);
data.q_filt_dps = (1-alpha)*double(data.q_filt_dps) + alpha*qRawDps;
data.prev_pitch_deg = double(pitchDeg);
data.prev_time_s = double(t);
qDps = data.q_filt_dps;
end

function cfgIdx = local_config_index_from_mass(massKg, nControllers)
refMass = local_base_scalar("airdropx_mpc_reference_mass_kg", 3423.0);
cargoMass = local_base_scalar("airdropx_auto_cargo_mass_kg", 300.0);
if ~isfinite(cargoMass) || cargoMass <= 0, cargoMass = 300.0; end
dropCount = round(max(0.0, (refMass-double(massKg))/cargoMass));
dropCount = min(max(dropCount,0), nControllers-1);
cfgIdx = dropCount + 1;
end

function value = local_cfg_base_scalar(vectorName, scalarName, cfgIdx, fallback)
value = fallback;
try
    if evalin("base", "exist('" + vectorName + "','var')")
        candidate = double(evalin("base", vectorName));
        candidate = candidate(:);
        if cfgIdx >= 1 && cfgIdx <= numel(candidate) && isfinite(candidate(cfgIdx))
            value = candidate(cfgIdx);
            return;
        end
    end
catch
end
value = local_base_scalar(scalarName, fallback);
end

function [ePulse, tPulse] = local_test_disturbance(t)
ePulse = 0.0;
tPulse = 0.0;
for k = 1:2
    startS = local_base_scalar("airdropx_auto_test_pulse" + string(k) + "_start_s", Inf);
    durationS = max(0.0, local_base_scalar( ...
        "airdropx_auto_test_pulse" + string(k) + "_duration_s", 0.0));
    if isfinite(startS) && double(t) >= startS && double(t) < startS + durationS
        ePulse = ePulse + local_base_scalar( ...
            "airdropx_auto_test_pulse" + string(k) + "_elevator", 0.0);
        tPulse = tPulse + local_base_scalar( ...
            "airdropx_auto_test_pulse" + string(k) + "_throttle", 0.0);
    end
end
end

function value = local_trim_field(s, name, fallback)
if isstruct(s) && isfield(s,name) && ~isempty(s.(name)) && isfinite(double(s.(name)))
    value = double(s.(name));
else
    value = double(fallback);
end
end

function value = local_meta_scalar(meta, name, fallback)
value = fallback;
try
    candidate = double(meta.(name));
    if isscalar(candidate) && isfinite(candidate), value = candidate; end
catch
end
end

function value = local_base_scalar(name, fallback)
value = fallback;
try
    if evalin("base", "exist('" + name + "','var')")
        candidate = double(evalin("base", name));
        if isscalar(candidate) && isfinite(candidate), value = candidate; end
    end
catch
end
end

function value = local_base_value(name, fallback)
value = fallback;
try
    if evalin("base", "exist('" + name + "','var')"), value = evalin("base", name); end
catch
end
end

function value = local_memory(action, block, value)
persistent store
if isempty(store), store = containers.Map("KeyType","char","ValueType","any"); end
key = sprintf("%.15g", block.BlockHandle);
switch string(action)
    case "set"
        store(key) = value;
    case "get"
        if isKey(store,key), value = store(key); else, value = []; end
    otherwise
        error("Unknown memory action: %s", action);
end
end
