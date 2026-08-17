function sfun_airdropx_physics_mpc_controller(block)
%SFUN_AIRDROPX_PHYSICS_MPC_CONTROLLER Fixed-rule physics scheduled MPC controller.
%
% Physics-MPC CL-1.7 early-recovery/energy-sharing safety layer:
%   * old v29/v30/v31 banks/checkpoints are never consulted;
%   * the inner MPC tracks only Va/pitch/vz/q (direct altitude weight = 0);
%   * Hcmd is converted to vz_ref by one global downstream-aware governor;
%   * Vcmd is rate-governed independently;
%   * speed-node commands are blended continuously in PHYSICAL actuator space;
%   * cfg changes seed the new MPC LastMove from the last physical command.
%
% Input from preserved SLX:
%   [altitude_m; vz_up_mps; airspeed_mps; pitch_deg; mass_kg; cg_x_m]
% Output to plant:
%   [external_elevator_delta; throttle_cmd]
setup(block);
end

function setup(block)
block.NumDialogPrms = 1;
block.NumInputPorts = 1;
block.NumOutputPorts = 1;
block.InputPort(1).Dimensions = 6;
block.InputPort(1).DatatypeID = 0;
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).DirectFeedthrough = true;
block.OutputPort(1).Dimensions = 2;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';
block.SampleTimes = [0.1 0.0];
block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Outputs', @Outputs);
block.RegBlockMethod('Terminate', @Terminate);
end

function Start(block)
bankPath = block.DialogPrm(1).Data;
if isstring(bankPath), bankPath = char(bankPath); end
if isempty(bankPath), bankPath = local_base_value('airdropx_auto_mpc_bank_mat_path',''); end
if isempty(bankPath) || ~isfile(bankPath)
    error('AirdropX:PhysicsMPC:MissingBank','Physics MPC bank not found: %s',string(bankPath));
end
S = load(bankPath);
if ~isfield(S,'v32_nodes') || isempty(S.v32_nodes)
    error('AirdropX:PhysicsMPC:BadBank','Bank does not contain compatible scheduled nodes.');
end
D = struct();
D.nodes = S.v32_nodes(:);
D.speed_nodes = double([D.nodes.speed_mps]).';
[D.speed_nodes,ord] = sort(D.speed_nodes);
D.nodes = D.nodes(ord);
D.n_cfg = local_bank_cfg_count(D.nodes);
D.states = cell(numel(D.nodes),D.n_cfg);
for n=1:numel(D.nodes)
    ctrls=D.nodes(n).controllers;
    for k=1:min(D.n_cfg,numel(ctrls))
        if isempty(ctrls{k}), continue; end
        D.states{n,k}=mpcstate(ctrls{k});
        try, D.states{n,k}.LastMove=zeros(2,1); catch, end
    end
end
D.last_cfg = 1;
D.last_physical_cmd = [NaN;NaN];
D.height_bias = 0.0;
D.height_vz_cmd = 0.0;
D.speed_ref_cmd = NaN;
D.prev_pitch_deg = NaN;
D.prev_time_s = NaN;
D.q_filt_dps = 0.0;
D.fail_count = 0;
D.mpc_exception_count = 0;
D.mpc_qp_fail_count = 0;
D.mpc_success_count = 0;
D.last_mpc_iterations = NaN;
D.last_error_signature = "";
D.runtime_hidden_elevator = NaN;
D.cfg_invalid_count = 0;
D.cfg_mass_raw = 1;
D.state_ready = false;
D.startup_reported = false;
D.input_invalid_count = 0;
D.startup_hold_count = 0;
D.recovery_mode = 0;
D.recovery_reason_code = 0;
D.recovery_count = 0;
D.recovery_hard_count = 0;
D.recovery_enter_count = 0;
D.recovery_exit_streak = 0;
D.mpc_gate_reject_count = 0;
D.authority_limit_count = 0;
D.authority_limit_streak = 0;
D.tracking_loss_count = 0;
D.tracking_loss_streak = 0;
D.last_recovery_energy_error = NaN;
D.last_recovery_target_deviation = [NaN;NaN];
D.last_nominal_cmd = [NaN;NaN];
D.last_command_deviation = [NaN;NaN];
D.trace = zeros(0,56);

% Runtime-path self-test. This deliberately calls the SAME local nominal
% helper used by Outputs so helper-level indexing bugs are detected before
% the first control sample rather than surfacing as NoSafeNominal.
for nn=1:numel(D.nodes)
    for cc=1:D.n_cfg
        [nom,nomOK]=local_node_nominal(D.nodes(nn),cc);
        if ~nomOK || numel(nom)<2 || any(~isfinite(nom(1:2)))
            error('AirdropX:PhysicsMPC:RuntimeNominalSelfTestFailed', ...
                'Runtime nominal self-test failed at speed-node %d cfg%d.',nn,cc-1);
        end
    end
end

local_memory('set',block,D);
end

function InitializeConditions(block)
D=local_memory('get',block); if isempty(D), return; end
D.last_cfg=1; D.last_physical_cmd=[NaN;NaN];
D.height_bias=0; D.height_vz_cmd=0; D.speed_ref_cmd=NaN;
D.prev_pitch_deg=NaN; D.prev_time_s=NaN; D.q_filt_dps=0; D.fail_count=0;
D.mpc_exception_count=0; D.mpc_qp_fail_count=0; D.mpc_success_count=0; D.last_mpc_iterations=NaN; D.last_error_signature="";
D.runtime_hidden_elevator=NaN; D.cfg_invalid_count=0; D.cfg_mass_raw=1;
D.state_ready=false; D.startup_reported=false; D.input_invalid_count=0; D.startup_hold_count=0;
D.recovery_mode=0; D.recovery_reason_code=0; D.recovery_count=0; D.recovery_hard_count=0; D.recovery_enter_count=0;
D.recovery_exit_streak=0; D.mpc_gate_reject_count=0; D.authority_limit_count=0; D.authority_limit_streak=0;
D.tracking_loss_count=0; D.tracking_loss_streak=0; D.last_recovery_energy_error=NaN; D.last_recovery_target_deviation=[NaN;NaN];
D.last_nominal_cmd=[NaN;NaN]; D.last_command_deviation=[NaN;NaN]; D.trace=zeros(0,56);
for n=1:numel(D.nodes)
    for k=1:D.n_cfg
        try
            c=D.nodes(n).controllers{k};
            if isempty(c), continue; end
            D.states{n,k}=mpcstate(c); D.states{n,k}.LastMove=zeros(2,1);
        catch
        end
    end
end
local_memory('set',block,D);
end

function Outputs(block)
D=local_memory('get',block); if isempty(D), Start(block); D=local_memory('get',block); end
x=double(block.InputPort(1).Data(:));
if numel(x)<6, x(end+1:6,1)=NaN; end
h=x(1); vz=x(2); Va=x(3); pitch=x(4); mass=x(5); cgx=x(6);

% References are workspace commands and are valid before JSBSim necessarily
% publishes its first finite state sample.  They are therefore the correct
% scheduling fallback during startup -- never a NaN measured airspeed.
[reqH,reqV]=local_dynamic_reference(block.CurrentTime,h,Va);
scheduleV=local_startup_schedule_speed(reqV,Va,D.speed_nodes);
[iSched0,iSched1,wSched]=local_speed_bracket(D.speed_nodes,scheduleV);
innerMode=local_base_scalar('airdropx_v32_inner_reference_enabled',0)>0.5;

% Establish a finite cfg0/current-cfg physical command before any MPC or
% telemetry-dependent path is evaluated. This makes the first sample
% fail-safe deterministic even if mpcmove is temporarily unavailable.
if ~all(isfinite(D.last_physical_cmd))
    seedCfg=min(max(round(D.last_cfg),1),D.n_cfg);
    [seedCmd,seedOK]=local_blended_nominal(D,iSched0,iSched1,wSched,seedCfg);
    if ~seedOK || numel(seedCmd)<2 || any(~isfinite(seedCmd(1:2)))
        error('AirdropX:PhysicsMPC:NoInitialNominal', ...
            'Runtime bank has no finite initial physical nominal at schedule speed %.6g m/s cfg%d.',scheduleV,seedCfg-1);
    end
    D.last_physical_cmd=double(seedCmd(:));
end

% A Level-2 S-function can be called before every plant signal is finite.
% Treat that as a normal startup/telemetry state, not as an MPC failure and
% never pass nonfinite values into cfg indexing, speed scheduling or mpcmove.
stateValid=all(isfinite([h vz Va pitch mass cgx]));
if ~stateValid
    D.input_invalid_count=D.input_invalid_count+1;
    if ~D.state_ready, D.startup_hold_count=D.startup_hold_count+1; end

    cfg=D.last_cfg; cfgRaw=cfg;
    fixed=local_base_scalar('airdropx_auto_fixed_config_id',NaN);
    if isfinite(fixed), cfg=min(max(round(fixed),0),D.n_cfg-1)+1; cfgRaw=cfg; end

    % Before the first complete state frame, use the requested-speed cfg
    % nominal.  After the controller has once become live, a transient bad
    % telemetry frame holds the last physical command exactly.
    if D.state_ready && all(isfinite(D.last_physical_cmd))
        cmd=D.last_physical_cmd(:); nomOK=true;
    else
        [cmd,nomOK]=local_blended_nominal(D,iSched0,iSched1,wSched,cfg);
        if ~nomOK && all(isfinite(D.last_physical_cmd))
            cmd=D.last_physical_cmd(:); nomOK=true;
        end
    end
    if ~nomOK || numel(cmd)<2 || any(~isfinite(cmd(1:2)))
        error('AirdropX:PhysicsMPC:NoStartupNominal', ...
            'No finite startup/current-cfg nominal is available at requested schedule speed %.6g m/s.',scheduleV);
    end

    cmd(1)=min(max(cmd(1),-0.95),0.95); cmd(2)=min(max(cmd(2),0),1);
    if all(isfinite(D.last_physical_cmd)) && ~D.state_ready
        eStep=max(0.001,local_base_scalar('airdropx_v32_elevator_step_limit',0.012));
        tStep=max(0.001,local_base_scalar('airdropx_v32_throttle_step_limit',0.020));
        cmd(1)=min(max(cmd(1),D.last_physical_cmd(1)-eStep),D.last_physical_cmd(1)+eStep);
        cmd(2)=min(max(cmd(2),D.last_physical_cmd(2)-tStep),D.last_physical_cmd(2)+tStep);
    end
    D.last_physical_cmd=cmd(:);
    D=local_sync_last_moves(D,[iSched0 iSched1],cfg,cmd);
    [hidden,D]=local_runtime_hidden(D,iSched0,iSched1,wSched);
    plant=[cmd(1)-hidden; min(max(cmd(2),0),1)]; plant(1)=min(max(plant(1),-0.85),0.85);

    if ~D.startup_reported
        fprintf('[PHYS-MPC STARTUP HOLD] t=%.6f nonfinite state: H=%g vz=%g Va=%g pitch=%g mass=%g cg=%g; scheduleV=%.3f cfg=%d nominal=[%.6f %.6f]\n', ...
            double(block.CurrentTime),h,vz,Va,pitch,mass,cgx,scheduleV,cfg-1,cmd(1),cmd(2));
        D.startup_reported=true;
    end
    q=D.q_filt_dps;
    D=local_trace(D,block.CurrentTime,cfg,reqH,reqV,scheduleV,h,Va,vz,pitch,q,NaN,NaN,NaN,iSched0,iSched1,wSched,cmd,plant,innerMode,mass,cfgRaw);
    local_memory('set',block,D); block.OutputPort(1).Data=plant; return;
end

% First complete plant frame: enter closed-loop mode without differentiating
% pitch across the startup-invalid interval.
if ~D.state_ready
    D.state_ready=true;
    D.prev_pitch_deg=double(pitch); D.prev_time_s=double(block.CurrentTime); D.q_filt_dps=0; q=0;
    if D.startup_reported
        fprintf('[PHYS-MPC STARTUP READY] t=%.6f first finite state: H=%.3f vz=%.4f Va=%.4f pitch=%.4f mass=%.3f cg=%.5f\n', ...
            double(block.CurrentTime),h,vz,Va,pitch,mass,cgx);
    end
else
    [q,D]=local_filtered_q(block.CurrentTime,pitch,D);
end

[cfg,cfgRaw,cfgValid]=local_config_index_from_mass(mass,D.n_cfg,D.last_cfg);
D.cfg_mass_raw=cfgRaw;
if ~cfgValid, D.cfg_invalid_count=D.cfg_invalid_count+1; end
fixed=local_base_scalar('airdropx_auto_fixed_config_id',NaN);
if isfinite(fixed), cfg=min(max(round(fixed),0),D.n_cfg-1)+1; cfgRaw=cfg; end
% Payload configuration is physically monotonic within one mission.  A noisy,
% nonfinite, or fuel-affected mass sample must never jump backwards or skip
% multiple payload states in one 0.1-s controller sample.
cfg=max(D.last_cfg,min(cfg,D.last_cfg+1));
cfg=min(max(round(cfg),1),D.n_cfg);
if cfg~=D.last_cfg
    D=local_cfg_transition(D,cfg);
    D.last_cfg=cfg;
end

[govV,D]=local_speed_governor(reqV,Va,D);
if innerMode
    [govV,vzRef]=local_inner_reference(block.CurrentTime,govV);
    reqH=h;
    D.height_bias=0; D.height_vz_cmd=vzRef;
    rawVz=vzRef; limVz=vzRef;
else
    [vzRef,D,rawVz,limVz]=local_height_governor(reqH,h,vz,D);
end

[i0,i1,w]=local_speed_bracket(D.speed_nodes,Va);
[cmdNom,nomOK]=local_blended_nominal(D,i0,i1,w,cfg);
[xNomBlend,xNomOK]=local_blended_trim_state(D,i0,i1,w,cfg);
if ~nomOK || ~xNomOK
    error('AirdropX:PhysicsMPC:NoSafeNominal','No finite current-cfg nominal/state is available.');
end
D.last_nominal_cmd=cmdNom(:);

% CL-1.7: normal MPC + early tracking-loss recovery + hard recovery.
% RECOVERY uses one unified deterministic energy/attitude law in PHYSICAL
% actuator coordinates. HARD remains in bounded recovery instead of falling
% back to trim, which previously removed authority exactly when it was needed.
trackErr=[Va-govV; pitch-xNomBlend(2); vz-vzRef; q];
hErr=reqH-h;
hardNow=local_recovery_hard(trackErr);
entryNow=local_recovery_entry(trackErr);

% CL-1.7 early tracking-loss detector.  It does NOT fire merely because a
% height command is large.  It requires the aircraft to be persistently
% moving AWAY from the requested altitude while the inner vz loop is also
% lagging in the wrong direction.
trackingLossNow=local_vertical_tracking_loss(hErr,vz,trackErr(3));
if trackingLossNow
    D.tracking_loss_count=D.tracking_loss_count+1;
    D.tracking_loss_streak=D.tracking_loss_streak+1;
else
    D.tracking_loss_streak=max(0,D.tracking_loss_streak-1);
end
trackN=max(1,round(local_base_scalar('airdropx_physics_recovery_tracking_loss_hold_samples',10)));
trackingLossReady=D.tracking_loss_streak>=trackN;

succ0=D.mpc_success_count; exc0=D.mpc_exception_count; qp0=D.mpc_qp_fail_count; %#ok<NASGU>
cmd=[NaN;NaN]; ok=false;
if D.recovery_mode==0 && ~entryNow
    [cmd0,ok0,D]=local_node_move(D,i0,cfg,h,vz,Va,pitch,q,govV,vzRef);
    if i1==i0
        cmd=cmd0; ok=ok0;
    else
        [cmd1,ok1,D]=local_node_move(D,i1,cfg,h,vz,Va,pitch,q,govV,vzRef);
        if ok0 && ok1, cmd=(1-w)*cmd0+w*cmd1; ok=true;
        elseif ok0, cmd=cmd0; ok=true;
        elseif ok1, cmd=cmd1; ok=true;
        else, cmd=[NaN;NaN]; ok=false; end
    end

    solverFault=(D.mpc_exception_count>exc0)||(D.mpc_qp_fail_count>qp0);
    if ~ok && ~solverFault
        D.mpc_gate_reject_count=D.mpc_gate_reject_count+1;
        entryNow=true; D.recovery_reason_code=3;
    elseif ~ok && solverFault
        D.fail_count=D.fail_count+1;
        entryNow=true; D.recovery_reason_code=4;
    elseif ok
        [eLim,tLim]=local_blended_deviation_limits(D,i0,i1,w);
        duCmd=cmd(:)-cmdNom(:); D.last_command_deviation=duCmd(:);
        authorityLimited=(abs(duCmd(1))>=0.95*eLim)||(abs(duCmd(2))>=0.95*tLim);
        if authorityLimited
            D.authority_limit_count=D.authority_limit_count+1;
            D.authority_limit_streak=D.authority_limit_streak+1;
        else
            D.authority_limit_streak=max(0,D.authority_limit_streak-1);
        end
        authN=max(1,round(local_base_scalar('airdropx_physics_recovery_authority_hold_samples',8)));
        if D.authority_limit_streak>=authN && abs(trackErr(3))>=local_base_scalar('airdropx_physics_recovery_vz_error_entry_mps',1.0)
            entryNow=true; D.recovery_reason_code=2;
        elseif trackingLossReady
            % Keep this cycle's valid MPC command as the hand-off anchor.
            % Recovery is allowed only to ADD authority in the corrective
            % direction, never to weaken a command that MPC already made.
            entryNow=true; D.recovery_reason_code=6;
        end
    end
end

if entryNow && D.recovery_mode==0
    D.recovery_mode=1; D.recovery_enter_count=D.recovery_enter_count+1; D.recovery_exit_streak=0;
    if D.recovery_reason_code==0, D.recovery_reason_code=1; end
end
if hardNow
    D.recovery_mode=1; D.recovery_reason_code=5; D.recovery_hard_count=D.recovery_hard_count+1;
end

if D.recovery_mode>0
    anchorCmd=cmd;
    if numel(anchorCmd)<2 || any(~isfinite(anchorCmd(1:2)))
        anchorCmd=D.last_physical_cmd;
    end
    [cmd,D]=local_recovery_command(D,cmdNom,xNomBlend,Va,pitch,vz,q,govV,vzRef,hErr,anchorCmd);
    D.recovery_count=D.recovery_count+1; ok=true;
    % Hand back to the local MPC only when both the aircraft state and the
    % required physical command are back inside the NORMAL controller's
    % authority. This avoids an immediate recovery<->MPC chatter loop.
    [eExitLim,tExitLim]=local_blended_deviation_limits(D,i0,i1,w);
    duExit=cmd(:)-cmdNom(:);
    commandBackInside=abs(duExit(1))<=0.90*eExitLim && abs(duExit(2))<=0.90*tExitLim;
    if local_recovery_exit(trackErr) && commandBackInside
        D.recovery_exit_streak=D.recovery_exit_streak+1;
    else
        D.recovery_exit_streak=0;
    end
    exitN=max(1,round(local_base_scalar('airdropx_physics_recovery_exit_hold_samples',15)));
    if D.recovery_exit_streak>=exitN
        D.recovery_mode=0; D.recovery_reason_code=0; D.recovery_exit_streak=0; D.authority_limit_streak=0; D.tracking_loss_streak=0;
        D.last_recovery_energy_error=NaN; D.last_recovery_target_deviation=[NaN;NaN];
    end
end

if ~ok || numel(cmd)<2 || any(~isfinite(cmd(1:2)))
    D.fail_count=D.fail_count+1;
    if all(isfinite(D.last_physical_cmd)),cmd=D.last_physical_cmd(:);else,cmd=cmdNom(:);end
end
D.last_command_deviation=cmd(:)-cmdNom(:);
cmd(1)=min(max(cmd(1),-0.95),0.95);
cmd(2)=min(max(cmd(2),0.0),1.0);

% Physical-space actuator continuity. These are hard safety guards, not learned.
eStep=max(0.001,local_base_scalar('airdropx_v32_elevator_step_limit',0.012));
tStep=max(0.001,local_base_scalar('airdropx_v32_throttle_step_limit',0.020));
if all(isfinite(D.last_physical_cmd))
    cmd(1)=min(max(cmd(1),D.last_physical_cmd(1)-eStep),D.last_physical_cmd(1)+eStep);
    cmd(2)=min(max(cmd(2),D.last_physical_cmd(2)-tStep),D.last_physical_cmd(2)+tStep);
end
% Trace the command that is actually sent after rate limiting, not the
% pre-limiter recovery/MPC request.
D.last_command_deviation=cmd(:)-cmdNom(:);
D.last_physical_cmd=cmd(:);
D=local_sync_last_moves(D,[i0 i1],cfg,cmd);

[hidden,D]=local_runtime_hidden(D,i0,i1,w);
plant=[cmd(1)-hidden; min(max(cmd(2),0),1)]; plant(1)=min(max(plant(1),-0.85),0.85);
D=local_trace(D,block.CurrentTime,cfg,reqH,reqV,govV,h,Va,vz,pitch,q,rawVz,limVz,vzRef,i0,i1,w,cmd,plant,innerMode,mass,cfgRaw);
local_memory('set',block,D);
block.OutputPort(1).Data=plant;
end

function Terminate(block)
D=local_memory('get',block); if isempty(D), return; end
try, assignin('base','airdropx_v32_controller_trace',double(D.trace)); catch, end
end

function [cmd,ok,D]=local_node_move(D,nodeIdx,cfg,h,vz,Va,pitch,q,vRef,vzRef)
node=D.nodes(nodeIdx); ok=false; cmd=[NaN;NaN];
try
    cfg=local_safe_node_cfg(node,cfg);
    if ~isfinite(cfg), return; end
    ctrl=node.controllers{cfg}; trim=node.trim_bank(cfg);
    if isempty(ctrl), return; end
    % Inner MPC measured output y is a Ny-by-1 COLUMN vector.
    yNom=[double(trim.airspeed_mps);double(trim.pitch_deg);local_trim_field(trim,'vz_up_mps',0);local_trim_field(trim,'q_dps',0)];
    y=[Va;pitch;vz;q]; yDev=y-yNom;

    % MATLAB MPC API: measured output ym is Ny-by-1, while reference r is
    % p-by-Ny.  With Ny=4 and a constant reference, r MUST be 1-by-4.
    % The old 4-by-1 rDev made mpcmove throw on every controller sample.
    r=[vRef;double(trim.pitch_deg);vzRef;0]; rDev=(r-yNom).';

    if ~local_fast_state_safe(yDev), return; end
    st=D.states{nodeIdx,cfg};
    [du,info]=local_mpcmove(ctrl,st,yDev,rDev);

    its=local_info_iterations(info);
    D.last_mpc_iterations=its;
    if local_qp_failed(info)
        D.mpc_qp_fail_count=D.mpc_qp_fail_count+1;
        return;
    end
    if numel(du)<2 || any(~isfinite(du(1:2))), return; end

    D.mpc_success_count=D.mpc_success_count+1;
    du=double(du(:));
    em=local_meta_scalar(node.mpc_meta,'elevator_deviation_limit',0.08);
    tm=local_meta_scalar(node.mpc_meta,'throttle_deviation_limit',0.12);
    du(1)=min(max(du(1),-em),em); du(2)=min(max(du(2),-tm),tm);
    [nom,nomOK]=local_node_nominal(node,cfg); if ~nomOK, return; end; cmd=nom+du; ok=true;
catch ME
    D.mpc_exception_count=D.mpc_exception_count+1;
    sig=string(ME.identifier)+"|"+string(ME.message);
    if sig~=D.last_error_signature
        fprintf(2,'[PHYS-MPC RUNTIME] mpcmove exception node=%d cfg=%d: %s | %s\n', ...
            nodeIdx,cfg-1,char(string(ME.identifier)),char(string(ME.message)));
        D.last_error_signature=sig;
    end
    ok=false;
end
end

function [nom,ok]=local_blended_nominal(D,i0,i1,w,cfg)
nom=[NaN;NaN];ok=false;
[n0,ok0]=local_node_nominal(D.nodes(i0),cfg);
if i1==i0
    if ok0,nom=double(n0(:));ok=true;end
    return;
end
[n1,ok1]=local_node_nominal(D.nodes(i1),cfg);
if ok0 && ok1
    nom=double((1-w)*n0+w*n1);ok=all(isfinite(nom));
elseif ok0
    nom=double(n0(:));ok=true;
elseif ok1
    nom=double(n1(:));ok=true;
end
end

function D=local_cfg_transition(D,newCfg)
% Exact physical-command continuity: seed every speed-node controller for the
% new cfg with the last command expressed around that node's new nominal.
if ~all(isfinite(D.last_physical_cmd)), return; end
for n=1:numel(D.nodes)
    try
        c=D.nodes(n).controllers{newCfg}; if isempty(c), continue; end
        st=mpcstate(c); [nom,nomOK]=local_node_nominal(D.nodes(n),newCfg); if ~nomOK,continue;end
        du=D.last_physical_cmd(:)-nom(:);
        em=local_meta_scalar(D.nodes(n).mpc_meta,'elevator_deviation_limit',0.08);
        tm=local_meta_scalar(D.nodes(n).mpc_meta,'throttle_deviation_limit',0.12);
        du(1)=min(max(du(1),-em),em); du(2)=min(max(du(2),-tm),tm);
        try, st.LastMove=du; catch, end
        D.states{n,newCfg}=st;
    catch
    end
end
% Height governor states intentionally survive payload/config changes.
% A new payload configuration starts a fresh persistent-loss observation.
D.tracking_loss_streak=0;
D.last_recovery_energy_error=NaN; D.last_recovery_target_deviation=[NaN;NaN];
end

function D=local_sync_last_moves(D,nodes,cfg,physicalCmd)
for n=unique(nodes(:).')
    if n<1 || n>numel(D.nodes), continue; end
    try
        [nom,nomOK]=local_node_nominal(D.nodes(n),cfg); if ~nomOK,continue;end; du=physicalCmd(:)-nom(:);
        em=local_meta_scalar(D.nodes(n).mpc_meta,'elevator_deviation_limit',0.08);
        tm=local_meta_scalar(D.nodes(n).mpc_meta,'throttle_deviation_limit',0.12);
        du(1)=min(max(du(1),-em),em); du(2)=min(max(du(2),-tm),tm);
        if isempty(D.states{n,cfg}), D.states{n,cfg}=mpcstate(D.nodes(n).controllers{cfg}); end
        D.states{n,cfg}.LastMove=du;
    catch
    end
end
end

function [vzCmd,D,raw,limited]=local_height_governor(targetH,h,actualVz,D) %#ok<INUSD>
kh=max(0,local_base_scalar('airdropx_v32_height_kh',0.12));
ki=max(0,local_base_scalar('airdropx_v32_height_ki',0.004));
kaw=max(0,local_base_scalar('airdropx_v32_height_kaw',0.30));
vzMax=max(0.1,local_base_scalar('airdropx_v32_height_vz_max_mps',2.0));
slew=max(0.05,local_base_scalar('airdropx_v32_height_vz_slew_mps2',0.60));
biasMax=max(0.05,local_base_scalar('airdropx_v32_height_bias_max_mps',1.5));
dead=max(0,local_base_scalar('airdropx_v32_height_deadband_m',0.05));
dt=0.1; e=targetH-h; ei=e; if abs(ei)<dead, ei=0; end

% CL-1.5 structural fix:
% Do NOT back-calculate the outer-loop integrator from (actualVz-vzCmd).
% actualVz is a downstream PLANT RESPONSE, not the output of this governor's
% saturation element. Feeding that tracking lag into anti-windup caused the
% bias to acquire the wrong sign after payload drops (for example V50 cfg2
% hit -1.2 m/s while already below the height target), so the governor kept
% commanding descent until the local MPC trust gate was eventually exited.
%
% Anti-windup is now conventional and local to this governor: integrate the
% height error, form the raw request, apply magnitude + slew limits, and use
% only the governor's OWN realization residual (vzCmd-raw) for back-calculation.
D.height_bias=D.height_bias+ki*ei*dt;
D.height_bias=min(max(D.height_bias,-biasMax),biasMax);
raw=kh*e+D.height_bias;
limited=min(max(raw,-vzMax),vzMax);
step=slew*dt;
vzCmd=min(max(limited,D.height_vz_cmd-step),D.height_vz_cmd+step);
awResidual=vzCmd-raw;
D.height_bias=D.height_bias+kaw*awResidual*dt;
D.height_bias=min(max(D.height_bias,-biasMax),biasMax);
D.height_vz_cmd=vzCmd;
end

function [vCmd,D]=local_speed_governor(reqV,actualV,D)
if ~isfinite(D.speed_ref_cmd), D.speed_ref_cmd=actualV; end
acc=max(0.05,local_base_scalar('airdropx_v32_speed_accel_mps2',1.0));
dec=max(0.05,local_base_scalar('airdropx_v32_speed_decel_mps2',1.2));
d=reqV-D.speed_ref_cmd;
if d>=0, step=acc*0.1; else, step=dec*0.1; end
D.speed_ref_cmd=D.speed_ref_cmd+sign(d)*min(abs(d),step);
vCmd=D.speed_ref_cmd;
end

function [h,v]=local_dynamic_reference(t,hFallback,vFallback)
h=local_base_scalar('airdropx_target_altitude_m',hFallback);
v=local_base_scalar('airdropx_pd_v_ref_mps',vFallback);
P=local_base_matrix('airdropx_v32_dynamic_reference_profile',[]);
if isempty(P) || size(P,2)<3, return; end
P=double(P(:,1:3)); P=P(all(isfinite(P),2),:); if isempty(P), return; end
P=sortrows(P,1); idx=find(P(:,1)<=double(t)+1e-9,1,'last'); if isempty(idx), idx=1; end
h=P(idx,2); v=P(idx,3);
end

function [v,vz]=local_inner_reference(t,vFallback)
v=vFallback; vz=0;
P=local_base_matrix('airdropx_v32_inner_reference_profile',[]);
if isempty(P) || size(P,2)<3, return; end
P=double(P(:,1:3)); P=P(all(isfinite(P),2),:); if isempty(P), return; end
P=sortrows(P,1); idx=find(P(:,1)<=double(t)+1e-9,1,'last'); if isempty(idx), idx=1; end
v=P(idx,2); vz=P(idx,3);
end

function [xNom,ok]=local_blended_trim_state(D,i0,i1,w,cfg)
xNom=[NaN;NaN;NaN;NaN];ok=false;
try
    a=local_node_trim_state(D.nodes(i0),cfg);
    if i1==i0,xNom=a;ok=all(isfinite(a));return;end
    b=local_node_trim_state(D.nodes(i1),cfg);
    if all(isfinite(a))&&all(isfinite(b)),xNom=(1-w)*a+w*b;ok=true;
    elseif all(isfinite(a)),xNom=a;ok=true;
    elseif all(isfinite(b)),xNom=b;ok=true;end
catch
end
end

function x=local_node_trim_state(node,cfg)
cfg=local_safe_node_cfg(node,cfg);
if ~isfinite(cfg),x=[NaN;NaN;NaN;NaN];return;end
tr=node.trim_bank(cfg);
x=[double(tr.airspeed_mps);double(tr.pitch_deg);local_trim_field(tr,'vz_up_mps',0);local_trim_field(tr,'q_dps',0)];
end

function [eLim,tLim]=local_blended_deviation_limits(D,i0,i1,w)
e0=local_meta_scalar(D.nodes(i0).mpc_meta,'elevator_deviation_limit',0.10);
t0=local_meta_scalar(D.nodes(i0).mpc_meta,'throttle_deviation_limit',0.18);
if i1==i0,eLim=e0;tLim=t0;return;end
e1=local_meta_scalar(D.nodes(i1).mpc_meta,'elevator_deviation_limit',e0);
t1=local_meta_scalar(D.nodes(i1).mpc_meta,'throttle_deviation_limit',t0);
eLim=max(1e-3,(1-w)*e0+w*e1);tLim=max(1e-3,(1-w)*t0+w*t1);
end

function tf=local_recovery_entry(e)
% State-based entry remains a protection layer.  The CL-1.7 persistent
% tracking-loss trigger normally enters earlier during a slow divergence.
lim=[8;15;3;8];f=double(e(:));tf=~(all(isfinite(f))&&numel(f)==4&&all(abs(f)<=lim));
end
function tf=local_recovery_hard(e)
lim=[15;35;15;30];f=double(e(:));tf=all(isfinite(f))&&numel(f)==4&&any(abs(f)>lim);
end
function tf=local_recovery_exit(e)
lim=[3;6;1.0;3];f=double(e(:));tf=all(isfinite(f))&&numel(f)==4&&all(abs(f)<=lim);
end

function tf=local_vertical_tracking_loss(hErr,vz,evz)
% Unified early-loss detector for climb AND descent commands:
%   hErr*vz < 0  => aircraft is moving away from the requested altitude.
%   hErr*evz < 0 => inner vertical-speed response is lagging in the same
%                   wrong direction relative to the governor request.
hMin=max(0.2,local_base_scalar('airdropx_physics_recovery_tracking_herr_m',1.5));
vzMin=max(0.01,local_base_scalar('airdropx_physics_recovery_tracking_away_vz_mps',0.05));
eMin=max(0.05,local_base_scalar('airdropx_physics_recovery_tracking_vzerr_mps',0.35));
f=[hErr;vz;evz];
tf=all(isfinite(f)) && abs(hErr)>=hMin && abs(vz)>=vzMin && abs(evz)>=eMin && (hErr*vz<0) && (hErr*evz<0);
end

function [cmd,D]=local_recovery_command(D,nom,xNom,Va,pitch,vz,q,vRef,vzRef,hErr,anchorCmd)
% CL-1.7 simplified TECS-style recovery:
%   * elevator controls vertical-path/height distribution plus pitch/q damping;
%   * throttle controls TOTAL specific-energy deficit rather than blindly
%     adding power whenever vz is negative.
% Positive MQ9 elevator is nose-down, therefore positive altitude deficit
% and negative vertical-speed error both demand NEGATIVE elevator.
evz=double(vz-vzRef);ep=double(pitch-xNom(2));eq=double(q);hErr=double(hErr);
kVzE=local_base_scalar('airdropx_physics_recovery_elev_kvz',0.055);
kHE=local_base_scalar('airdropx_physics_recovery_elev_kh',0.006);
kPE=local_base_scalar('airdropx_physics_recovery_elev_kpitch',0.008);
kQE=local_base_scalar('airdropx_physics_recovery_elev_kq',0.015);
kEnergyT=local_base_scalar('airdropx_physics_recovery_thr_kenergy',0.0006);
eMax=max(0.12,local_base_scalar('airdropx_physics_recovery_elev_dev_max',0.30));
tMax=max(0.15,local_base_scalar('airdropx_physics_recovery_thr_dev_max',0.35));

% Height contribution is bounded separately so a very large altitude error
% cannot by itself slam the elevator to the recovery limit.
hTerm=min(max(-kHE*hErr,-0.12),0.12);
de=kVzE*evz+hTerm+kPE*ep+kQE*eq;

% Specific total-energy error [J/kg] = potential-energy deficit plus kinetic
% energy deficit.  If the aircraft is already fast while descending, the
% kinetic-energy surplus automatically suppresses excessive throttle boost.
g0=9.80665;
energyErr=g0*hErr+0.5*(double(vRef)^2-double(Va)^2);
dt=kEnergyT*energyErr;

de=min(max(de,-eMax),eMax);dt=min(max(dt,-tMax),tMax);
target=double(nom(:))+[de;dt];

% Bumpless authority extension: an early recovery handoff must never weaken
% the valid MPC command that was already correcting in the proper direction.
if numel(anchorCmd)>=2 && all(isfinite(anchorCmd(1:2)))
    a=double(anchorCmd(:));
    if de<0
        target(1)=min(target(1),a(1)); % recovery asks nose-up: never weaken an already stronger nose-up MPC command
    elseif de>0
        target(1)=max(target(1),a(1)); % recovery asks nose-down: never weaken an already stronger nose-down MPC command
    end
end

target(1)=min(max(target(1),-0.95),0.95);target(2)=min(max(target(2),0.0),1.0);
cmd=target;
D.last_recovery_energy_error=energyErr;
D.last_recovery_target_deviation=cmd(:)-double(nom(:));
D.last_command_deviation=cmd(:)-double(nom(:));
end

function tf=local_fast_state_safe(yDev)
% NORMAL-MPC trust envelope only. Rejection now transitions to recovery.
lim=[12;20;6;12]; f=double(yDev(:)); tf=all(isfinite(f))&&numel(f)==4&&all(abs(f)<=lim);
end

function v=local_startup_schedule_speed(reqV,actualV,speeds)
speeds=double(speeds(:));
if isfinite(reqV),v=double(reqV);
elseif isfinite(actualV),v=double(actualV);
elseif ~isempty(speeds),v=double(speeds(ceil(numel(speeds)/2)));
else,v=50;
end
if ~isempty(speeds)&&all(isfinite(speeds)),v=min(max(v,speeds(1)),speeds(end));end
end

function [i0,i1,w]=local_speed_bracket(speeds,v)
speeds=double(speeds(:)); n=numel(speeds);
if n<1,error('AirdropX:PhysicsMPC:NoSpeedNodes','Runtime bank contains no speed nodes.');end
if n<=1, i0=1;i1=1;w=0;return; end
% Last line of defense: a nonfinite plant measurement must never become an
% empty MATLAB index. Normal startup uses local_startup_schedule_speed().
if ~isfinite(v),k=ceil(n/2);i0=k;i1=k;w=0;return;end
if v<=speeds(1), i0=1;i1=1;w=0;return; end
if v>=speeds(end), i0=n;i1=n;w=0;return; end
i1=find(speeds>=v,1,'first');
if isempty(i1),i0=n;i1=n;w=0;return;end
i0=max(1,i1-1); den=speeds(i1)-speeds(i0); if den<=0,w=0;else,w=(v-speeds(i0))/den;end
end

function [nom,ok]=local_node_nominal(node,cfg)
nom=[NaN;NaN];ok=false;
cfg=local_safe_node_cfg(node,cfg);
if ~isfinite(cfg),return;end
try
    trim=node.trim_bank(cfg); elev=double(trim.elevator_cmd);
    try
        x=double(node.mpc_meta.physical_elevator_nominals(:)); if cfg<=numel(x)&&isfinite(x(cfg)),elev=x(cfg);end
    catch
    end
    thr=double(trim.throttle_cmd);
    if isscalar(elev)&&isscalar(thr)&&all(isfinite([elev thr]))
        nom=[elev;thr];ok=true;
    end
catch
end
end

function safeCfg=local_safe_node_cfg(node,cfgIn)
% Preserve the caller-supplied cfg. CL-1.2/1.3 accidentally overwrote the
% input with NaN before validating it, which made EVERY runtime nominal and
% MPC node lookup fail on the first sample.
safeCfg=NaN;
try
    n=min(numel(node.controllers),numel(node.trim_bank));
    if n<1,return;end
    x=double(cfgIn);
    if ~isscalar(x)||~isfinite(x),return;end
    safeCfg=min(max(round(x),1),n);
catch
end
end

function n=local_bank_cfg_count(nodes)
n=Inf;
for ii=1:numel(nodes)
    try
        ni=min(numel(nodes(ii).controllers),numel(nodes(ii).trim_bank));
    catch
        ni=0;
    end
    n=min(n,ni);
end
if ~isfinite(n)||n<5
    error('AirdropX:PhysicsMPC:IncompleteRuntimeBank','Scheduled bank has only %g complete cfg entries; 5 are required.',n);
end
n=min(5,round(n));
end

function [q,D]=local_filtered_q(t,pitch,D)
% Never contaminate derivative memory with a nonfinite telemetry sample.
if ~isfinite(pitch)||~isfinite(t)
    q=D.q_filt_dps;return;
end
qRaw=0;
if isfinite(D.prev_time_s)&&isfinite(D.prev_pitch_deg)&&t>D.prev_time_s
    % Wrap-safe pitch difference avoids a +/-180 deg representation jump.
    dp=mod((double(pitch)-D.prev_pitch_deg)+180,360)-180;
    qRaw=dp/(double(t)-D.prev_time_s);
end
alpha=0.25; D.q_filt_dps=(1-alpha)*D.q_filt_dps+alpha*qRaw;
D.prev_pitch_deg=double(pitch); D.prev_time_s=double(t); q=D.q_filt_dps;
end

function [cfg,rawCfg,valid]=local_config_index_from_mass(mass,nControllers,lastCfg)
% Robust fallback for the legacy 6-signal controller interface.  Payload cfg
% is discrete and monotonic, while mass can become nonfinite during a plant
% excursion and can include small continuous variations.  Never allow such a
% sample to become a MATLAB array index.
if nargin<3||~isfinite(lastCfg),lastCfg=1;end
nControllers=max(1,round(double(nControllers)));
lastCfg=min(max(round(double(lastCfg)),1),nControllers);
ref=local_base_scalar('airdropx_mpc_reference_mass_kg',3423); cargo=local_base_scalar('airdropx_auto_cargo_mass_kg',300);
if cargo<=0||~isfinite(cargo),cargo=300;end
valid=isfinite(mass)&&isfinite(ref);
if ~valid
    rawCfg=lastCfg;cfg=lastCfg;return;
end
dropRaw=(ref-double(mass))/cargo;
if ~isfinite(dropRaw)
    rawCfg=lastCfg;cfg=lastCfg;valid=false;return;
end
rawCfg=round(dropRaw)+1;
cfg=min(max(rawCfg,1),nControllers);
% Mission payload count cannot decrease and cannot skip more than one state in
% one controller sample.  This also prevents fuel/noise from causing a false
% backwards cfg switch.
cfg=max(lastCfg,cfg);
cfg=min(cfg,lastCfg+1);
cfg=min(max(round(cfg),1),nControllers);
end

function [cmd,info]=local_mpcmove(ctrl,state,y,r)
% R2026a supports the two-output syntax. Do not swallow the original
% exception by retrying a different output signature.
[cmd,info]=mpcmove(ctrl,state,y,r);
end

function it=local_info_iterations(info)
it=NaN;
try
    if isstruct(info)&&isfield(info,'Iterations')
        x=double(info.Iterations);
        if isscalar(x)&&isfinite(x),it=x;end
    end
catch
end
end

function tf=local_qp_failed(info)
% R2026a: Iterations > 0 = optimal; 0 = unreliable/nonconverged;
% -1 = infeasible; -2 = numerical failure. QPCode is TEXT, not numeric.
tf=false;
try
    if isstruct(info)&&isfield(info,'Iterations')
        x=double(info.Iterations);
        if isscalar(x)&&isfinite(x)
            tf=(x<=0);
            return;
        end
    end
    if isstruct(info)&&isfield(info,'QPCode')
        q=lower(string(info.QPCode));
        tf=(q~="feasible");
    end
catch
    tf=true;
end
end

function [hidden,D]=local_runtime_hidden(D,i0,i1,w)
% The legacy MEX computes trimElevator_ once at reset; latch the matching
% physical->external offset once and keep it fixed for the whole mission.
if ~isfinite(D.runtime_hidden_elevator)
    override=local_base_scalar('airdropx_physics_mpc_runtime_hidden_elevator',NaN);
    if isfinite(override)
        D.runtime_hidden_elevator=override;
    else
        h0=local_meta_scalar(D.nodes(i0).mpc_meta,'hidden_elevator_offset',NaN);
        if i1==i0
            hInit=h0;
        else
            h1=local_meta_scalar(D.nodes(i1).mpc_meta,'hidden_elevator_offset',NaN);
            if isfinite(h0)&&isfinite(h1),hInit=(1-w)*h0+w*h1;elseif isfinite(h0),hInit=h0;else,hInit=h1;end
        end
        D.runtime_hidden_elevator=hInit;
    end
end
hidden=D.runtime_hidden_elevator;
if ~isfinite(hidden)
    hidden=local_base_scalar('airdropx_auto_hidden_elevator_trim',NaN);
    D.runtime_hidden_elevator=hidden;
end
if ~isfinite(hidden),error('AirdropX:PhysicsMPC:MissingHiddenTrim','No startup JSBSim hidden elevator offset is available.');end
end

function D=local_trace(D,t,cfg,reqH,reqV,govV,h,Va,vz,pitch,q,rawVz,limVz,vzRef,i0,i1,w,cmd,plant,innerMode,mass,cfgRaw)
try
    row=[double(t),double(cfg-1),reqH,reqV,govV,h,Va,vz,pitch,q,reqH-h,rawVz,limVz,vzRef,D.height_bias,...
        double(i0),double(i1),double(w),double(D.speed_nodes(i0)),double(D.speed_nodes(i1)),cmd(1),cmd(2),plant(1),plant(2),...
        double(innerMode),double(D.fail_count),local_base_scalar('airdropx_v32_height_kh',0.12),...
        local_base_scalar('airdropx_v32_height_ki',0.004),local_base_scalar('airdropx_v32_height_kaw',0.30),...
        local_base_scalar('airdropx_v32_height_vz_max_mps',2.0),double(D.mpc_success_count),double(D.mpc_exception_count),...
        double(D.mpc_qp_fail_count),double(D.last_mpc_iterations),double(mass),double(cfgRaw-1),double(cfg-1),double(D.cfg_invalid_count),...
        double(D.input_invalid_count),double(D.startup_hold_count),double(D.state_ready),...
        double(D.mpc_gate_reject_count),double(D.recovery_count),double(D.recovery_mode),double(D.recovery_reason_code),...
        double(D.recovery_hard_count),double(D.authority_limit_count),double(D.authority_limit_streak),...
        double(D.last_command_deviation(1)),double(D.last_command_deviation(2)),double(D.recovery_enter_count),...
        double(D.tracking_loss_count),double(D.tracking_loss_streak),double(D.last_recovery_energy_error),...
        double(D.last_recovery_target_deviation(1)),double(D.last_recovery_target_deviation(2))];
    D.trace(end+1,:)=row;
catch
end
end

function value=local_trim_field(s,name,fallback)
value=fallback; try, x=double(s.(name)); if isscalar(x)&&isfinite(x),value=x;end, catch, end
end
function value=local_meta_scalar(s,name,fallback)
value=fallback; try, x=double(s.(name)); if isscalar(x)&&isfinite(x),value=x;end, catch, end
end
function value=local_base_scalar(name,fallback)
value=fallback; try, if evalin('base',sprintf("exist('%s','var')",name)),x=double(evalin('base',name));if isscalar(x)&&isfinite(x),value=x;end,end,catch,end
end
function value=local_base_value(name,fallback)
value=fallback; try, if evalin('base',sprintf("exist('%s','var')",name)),value=evalin('base',name);end,catch,end
end
function M=local_base_matrix(name,fallback)
M=fallback; try, if evalin('base',sprintf("exist('%s','var')",name)),M=evalin('base',name);end,catch,end
end

function value=local_memory(action,block,varargin)
persistent store
key=sprintf('b%d',round(double(block.BlockHandle)));
if isempty(store),store=struct();end
value=[];
switch char(action)
    case 'set'
        store.(key)=varargin{1}; value=varargin{1};
    case 'get'
        if isfield(store,key), value=store.(key); end
end
end
