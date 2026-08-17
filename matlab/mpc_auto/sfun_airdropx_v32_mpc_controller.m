function sfun_airdropx_v32_mpc_controller(block)
%SFUN_AIRDROPX_V32_MPC_CONTROLLER Clean-slate scheduled inner MPC controller.
%
% v32 design:
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
    error('AirdropX:V32:MissingBank','v32 MPC bank not found: %s',string(bankPath));
end
S = load(bankPath);
if ~isfield(S,'v32_nodes') || isempty(S.v32_nodes)
    error('AirdropX:V32:BadBank','Bank does not contain v32_nodes. Refusing legacy bank.');
end
D = struct();
D.nodes = S.v32_nodes(:);
D.speed_nodes = double([D.nodes.speed_mps]).';
[D.speed_nodes,ord] = sort(D.speed_nodes);
D.nodes = D.nodes(ord);
D.states = cell(numel(D.nodes),5);
for n=1:numel(D.nodes)
    ctrls=D.nodes(n).controllers;
    for k=1:min(5,numel(ctrls))
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
D.trace = zeros(0,30);
local_memory('set',block,D);
end

function InitializeConditions(block)
D=local_memory('get',block); if isempty(D), return; end
D.last_cfg=1; D.last_physical_cmd=[NaN;NaN];
D.height_bias=0; D.height_vz_cmd=0; D.speed_ref_cmd=NaN;
D.prev_pitch_deg=NaN; D.prev_time_s=NaN; D.q_filt_dps=0; D.fail_count=0; D.trace=zeros(0,30);
for n=1:numel(D.nodes)
    for k=1:5
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
h=x(1); vz=x(2); Va=x(3); pitch=x(4); mass=x(5);
[q,D]=local_filtered_q(block.CurrentTime,pitch,D);
cfg=local_config_index_from_mass(mass,5);
fixed=local_base_scalar('airdropx_auto_fixed_config_id',NaN);
if isfinite(fixed), cfg=min(max(round(fixed),0),4)+1; end
if cfg~=D.last_cfg
    D=local_cfg_transition(D,cfg);
    D.last_cfg=cfg;
end

[reqH,reqV]=local_dynamic_reference(block.CurrentTime,h,Va);
[govV,D]=local_speed_governor(reqV,Va,D);
innerMode=local_base_scalar('airdropx_v32_inner_reference_enabled',0)>0.5;
if innerMode
    [govV,vzRef]=local_inner_reference(block.CurrentTime,govV);
    reqH=h;
    D.height_bias=0; D.height_vz_cmd=vzRef;
    rawVz=vzRef; limVz=vzRef;
else
    [vzRef,D,rawVz,limVz]=local_height_governor(reqH,h,vz,D);
end

[i0,i1,w]=local_speed_bracket(D.speed_nodes,Va);
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
if ~ok || numel(cmd)<2 || any(~isfinite(cmd(1:2)))
    D.fail_count=D.fail_count+1;
    if all(isfinite(D.last_physical_cmd)), cmd=D.last_physical_cmd;
    else, cmd=local_node_nominal(D.nodes(i0),cfg); end
end
cmd(1)=min(max(cmd(1),-0.95),0.95);
cmd(2)=min(max(cmd(2),0.0),1.0);

% Physical-space actuator continuity. These are hard safety guards, not learned.
eStep=max(0.001,local_base_scalar('airdropx_v32_elevator_step_limit',0.012));
tStep=max(0.001,local_base_scalar('airdropx_v32_throttle_step_limit',0.020));
if all(isfinite(D.last_physical_cmd))
    cmd(1)=min(max(cmd(1),D.last_physical_cmd(1)-eStep),D.last_physical_cmd(1)+eStep);
    cmd(2)=min(max(cmd(2),D.last_physical_cmd(2)-tStep),D.last_physical_cmd(2)+tStep);
end
D.last_physical_cmd=cmd(:);
D=local_sync_last_moves(D,[i0 i1],cfg,cmd);

hidden=local_base_scalar('airdropx_auto_hidden_elevator_trim',NaN);
if ~isfinite(hidden), error('AirdropX:V32:MissingHiddenTrim','airdropx_auto_hidden_elevator_trim is required.'); end
plant=[cmd(1)-hidden; min(max(cmd(2),0),1)]; plant(1)=min(max(plant(1),-0.85),0.85);
D=local_trace(D,block.CurrentTime,cfg,reqH,reqV,govV,h,Va,vz,pitch,q,rawVz,limVz,vzRef,i0,i1,w,cmd,plant,innerMode);
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
    ctrl=node.controllers{cfg}; trim=node.trim_bank(cfg);
    if isempty(ctrl), return; end
    % v32 Inner MPC is purely aerodynamic: [Va, pitch, vz, q]. Absolute
    % altitude is intentionally absent from both estimator and objective.
    yNom=[double(trim.airspeed_mps);double(trim.pitch_deg);local_trim_field(trim,'vz_up_mps',0);local_trim_field(trim,'q_dps',0)];
    y=[Va;pitch;vz;q]; yDev=y-yNom;
    r=[vRef;double(trim.pitch_deg);vzRef;0]; rDev=r-yNom;
    if ~local_fast_state_safe(yDev), return; end
    st=D.states{nodeIdx,cfg};
    [du,info]=local_mpcmove(ctrl,st,yDev,rDev);
    if local_qp_failed(info) || numel(du)<2 || any(~isfinite(du(1:2))), return; end
    du=double(du(:));
    em=local_meta_scalar(node.mpc_meta,'elevator_deviation_limit',0.08);
    tm=local_meta_scalar(node.mpc_meta,'throttle_deviation_limit',0.12);
    du(1)=min(max(du(1),-em),em); du(2)=min(max(du(2),-tm),tm);
    nom=local_node_nominal(node,cfg); cmd=nom+du; ok=true;
catch
    ok=false;
end
end

function D=local_cfg_transition(D,newCfg)
% Exact physical-command continuity: seed every speed-node controller for the
% new cfg with the last command expressed around that node's new nominal.
if ~all(isfinite(D.last_physical_cmd)), return; end
for n=1:numel(D.nodes)
    try
        c=D.nodes(n).controllers{newCfg}; if isempty(c), continue; end
        st=mpcstate(c); nom=local_node_nominal(D.nodes(n),newCfg);
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
end

function D=local_sync_last_moves(D,nodes,cfg,physicalCmd)
for n=unique(nodes(:).')
    if n<1 || n>numel(D.nodes), continue; end
    try
        nom=local_node_nominal(D.nodes(n),cfg); du=physicalCmd(:)-nom(:);
        em=local_meta_scalar(D.nodes(n).mpc_meta,'elevator_deviation_limit',0.08);
        tm=local_meta_scalar(D.nodes(n).mpc_meta,'throttle_deviation_limit',0.12);
        du(1)=min(max(du(1),-em),em); du(2)=min(max(du(2),-tm),tm);
        if isempty(D.states{n,cfg}), D.states{n,cfg}=mpcstate(D.nodes(n).controllers{cfg}); end
        D.states{n,cfg}.LastMove=du;
    catch
    end
end
end

function [vzCmd,D,raw,limited]=local_height_governor(targetH,h,actualVz,D)
kh=max(0,local_base_scalar('airdropx_v32_height_kh',0.12));
ki=max(0,local_base_scalar('airdropx_v32_height_ki',0.004));
kaw=max(0,local_base_scalar('airdropx_v32_height_kaw',0.30));
vzMax=max(0.1,local_base_scalar('airdropx_v32_height_vz_max_mps',2.0));
slew=max(0.05,local_base_scalar('airdropx_v32_height_vz_slew_mps2',0.60));
biasMax=max(0.05,local_base_scalar('airdropx_v32_height_bias_max_mps',1.5));
dead=max(0,local_base_scalar('airdropx_v32_height_deadband_m',0.05));
dt=0.1; e=targetH-h; ei=e; if abs(ei)<dead, ei=0; end
% Back-calculation uses downstream tracking error. If the inner MPC fails to
% realize vzCmd, the integral bias is pulled back instead of winding up.
trackingErr=actualVz-D.height_vz_cmd;
D.height_bias=D.height_bias+(ki*ei+kaw*trackingErr)*dt;
D.height_bias=min(max(D.height_bias,-biasMax),biasMax);
raw=kh*e+D.height_bias; limited=min(max(raw,-vzMax),vzMax);
step=slew*dt; vzCmd=min(max(limited,D.height_vz_cmd-step),D.height_vz_cmd+step);
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

function tf=local_fast_state_safe(yDev)
% Safety envelope, not a narrow ID trust gate. The scheduler already selects
% the closest speed node; disabling control during a commanded maneuver is unsafe.
lim=[12;20;6;12]; f=double(yDev(:)); tf=all(isfinite(f))&&numel(f)==4&&all(abs(f)<=lim);
end

function [i0,i1,w]=local_speed_bracket(speeds,v)
speeds=double(speeds(:)); n=numel(speeds);
if n<=1, i0=1;i1=1;w=0;return; end
if v<=speeds(1), i0=1;i1=1;w=0;return; end
if v>=speeds(end), i0=n;i1=n;w=0;return; end
i1=find(speeds>=v,1,'first'); i0=i1-1; den=speeds(i1)-speeds(i0); if den<=0,w=0;else,w=(v-speeds(i0))/den;end
end

function nom=local_node_nominal(node,cfg)
trim=node.trim_bank(cfg); elev=double(trim.elevator_cmd);
try
    x=double(node.mpc_meta.physical_elevator_nominals(:)); if cfg<=numel(x)&&isfinite(x(cfg)),elev=x(cfg);end
catch
end
nom=[elev;double(trim.throttle_cmd)];
end

function [q,D]=local_filtered_q(t,pitch,D)
qRaw=0;
if isfinite(D.prev_time_s)&&isfinite(D.prev_pitch_deg)&&t>D.prev_time_s
    qRaw=(double(pitch)-D.prev_pitch_deg)/(double(t)-D.prev_time_s);
end
alpha=0.25; D.q_filt_dps=(1-alpha)*D.q_filt_dps+alpha*qRaw;
D.prev_pitch_deg=double(pitch); D.prev_time_s=double(t); q=D.q_filt_dps;
end

function cfg=local_config_index_from_mass(mass,nControllers)
ref=local_base_scalar('airdropx_mpc_reference_mass_kg',3423); cargo=local_base_scalar('airdropx_auto_cargo_mass_kg',300);
if cargo<=0||~isfinite(cargo),cargo=300;end
drop=round(max(0,(ref-double(mass))/cargo)); drop=min(max(drop,0),nControllers-1); cfg=drop+1;
end

function [cmd,info]=local_mpcmove(ctrl,state,y,r)
info=struct();
try, [cmd,info]=mpcmove(ctrl,state,y,r); catch, cmd=mpcmove(ctrl,state,y,r); end
end
function tf=local_qp_failed(info)
tf=false; try, if isstruct(info)&&isfield(info,'QPCode'), tf=double(info.QPCode)<0; end, catch, end
end

function D=local_trace(D,t,cfg,reqH,reqV,govV,h,Va,vz,pitch,q,rawVz,limVz,vzRef,i0,i1,w,cmd,plant,innerMode)
try
    row=[double(t),double(cfg-1),reqH,reqV,govV,h,Va,vz,pitch,q,reqH-h,rawVz,limVz,vzRef,D.height_bias,...
        double(i0),double(i1),double(w),double(D.speed_nodes(i0)),double(D.speed_nodes(i1)),cmd(1),cmd(2),plant(1),plant(2),...
        double(innerMode),double(D.fail_count),local_base_scalar('airdropx_v32_height_kh',0.12),...
        local_base_scalar('airdropx_v32_height_ki',0.004),local_base_scalar('airdropx_v32_height_kaw',0.30),...
        local_base_scalar('airdropx_v32_height_vz_max_mps',2.0)];
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
