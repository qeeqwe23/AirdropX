function sfun_airdropx_urmpc_controller(block)
%SFUN_AIRDROPX_URMPC_CONTROLLER One unified adaptive robust Physics MPC.
%
% Input (preserved 6-signal SLX interface):
%   [altitude_m; vz_up_mps; airspeed_mps; pitch_deg; mass_kg; cg_x_m]
% Output:
%   [external_elevator_delta; throttle_cmd]
%
% No outer height governor and no recovery controller are used. One MPC
% object is solved every 0.1 s; only its LPV prediction model/nominal are
% updated from the certified physics vertices.
setup(block);
end

function setup(block)
block.NumDialogPrms=1;block.NumInputPorts=1;block.NumOutputPorts=1;
block.InputPort(1).Dimensions=6;block.InputPort(1).DatatypeID=0;block.InputPort(1).Complexity='Real';block.InputPort(1).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=2;block.OutputPort(1).DatatypeID=0;block.OutputPort(1).Complexity='Real';
block.SampleTimes=[0.1 0];block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('Start',@Start);block.RegBlockMethod('InitializeConditions',@InitializeConditions);block.RegBlockMethod('Outputs',@Outputs);block.RegBlockMethod('Terminate',@Terminate);
end

function Start(block)
bankPath=block.DialogPrm(1).Data;if isstring(bankPath),bankPath=char(bankPath);end
if isempty(bankPath),bankPath=local_base_value('airdropx_auto_mpc_bank_mat_path','');end
if isempty(bankPath)||~isfile(bankPath),error('AirdropX:URMPC:MissingBank','Unified MPC bank not found: %s',string(bankPath));end
S=load(bankPath,'ur_mpc','ur_models','ur_meta');
if ~isfield(S,'ur_mpc')||~isfield(S,'ur_models')||~isfield(S,'ur_meta'),error('AirdropX:URMPC:BadBank','Bank lacks ur_mpc/ur_models/ur_meta.');end
D=struct();D.ctrl=S.ur_mpc;D.models=S.ur_models;D.meta=S.ur_meta;D.speeds=double(S.ur_meta.speed_nodes_mps(:));
D.n_cfg=size(D.models,2);D.state=mpcstate(D.ctrl);D.last_cfg=1;D.last_physical_cmd=[NaN;NaN];D.runtime_hidden_elevator=NaN;
D.prev_pitch_deg=NaN;D.prev_time_s=NaN;D.q_filt_dps=0;D.state_ready=false;D.startup_hold_count=0;D.input_invalid_count=0;
D.mpc_success_count=0;D.mpc_fail_count=0;D.mpc_exception_count=0;D.last_iterations=NaN;D.last_qp_code="";D.last_slack=NaN;D.last_cost=NaN;
D.cfg_invalid_count=0;D.cfg_mass_raw=1;D.cfg_changed=0;D.trace=zeros(0,45);D.last_error_signature="";
local_memory('set',block,D);
end

function InitializeConditions(block)
D=local_memory('get',block);if isempty(D),return;end
D.state=mpcstate(D.ctrl);D.last_cfg=1;D.last_physical_cmd=[NaN;NaN];D.runtime_hidden_elevator=NaN;
D.prev_pitch_deg=NaN;D.prev_time_s=NaN;D.q_filt_dps=0;D.state_ready=false;D.startup_hold_count=0;D.input_invalid_count=0;
D.mpc_success_count=0;D.mpc_fail_count=0;D.mpc_exception_count=0;D.last_iterations=NaN;D.last_qp_code="";D.last_slack=NaN;D.last_cost=NaN;
D.cfg_invalid_count=0;D.cfg_mass_raw=1;D.cfg_changed=0;D.trace=zeros(0,45);D.last_error_signature="";local_memory('set',block,D);
end

function Outputs(block)
D=local_memory('get',block);if isempty(D),Start(block);D=local_memory('get',block);end
x=double(block.InputPort(1).Data(:));if numel(x)<6,x(end+1:6)=NaN;end
h=x(1);vz=x(2);Va=x(3);pitch=x(4);mass=x(5);cgx=x(6);D.cfg_changed=0;
[reqH,reqV]=local_dynamic_reference(block.CurrentTime,h,Va);
schedV=local_schedule_speed(reqV,Va,D.speeds);

% Establish deterministic cfg0 nominal before first valid telemetry frame.
if ~all(isfinite(D.last_physical_cmd))
    [~,Nom0,u0,i0,i1,w]=local_interpolated_model(D,schedV,D.last_cfg,reqH); %#ok<ASGLU>
    if any(~isfinite(u0)),error('AirdropX:URMPC:NoInitialNominal','No finite startup physical nominal.');end
    D.last_physical_cmd=u0(:);D.state.LastMove=u0(:);
    % mpcstate() starts at the controller-design anchor (V50/cfg2).  Before
    % the first valid telemetry frame, move the absolute estimator state to
    % the startup scheduling nominal as well; LastMove alone is insufficient.
    try,if numel(D.state.Plant)==numel(Nom0.X),D.state.Plant=Nom0.X(:);end,catch,end
    [hidden,D]=local_runtime_hidden(D,i0,i1,w);
else
    [i0,i1,w]=local_speed_bracket(D.speeds,schedV);[hidden,D]=local_runtime_hidden(D,i0,i1,w);
end

stateValid=all(isfinite([h vz Va pitch mass cgx]));
if ~stateValid
    D.input_invalid_count=D.input_invalid_count+1;if ~D.state_ready,D.startup_hold_count=D.startup_hold_count+1;end
    cmd=D.last_physical_cmd(:);plant=local_to_plant(cmd,hidden);
    D=local_trace(D,block.CurrentTime,D.last_cfg,reqH,reqV,h,Va,vz,pitch,D.q_filt_dps,mass,cgx,i0,i1,w,NaN(5,1),NaN(2,1),cmd,plant);
    local_memory('set',block,D);block.OutputPort(1).Data=plant;return;
end

firstValid=~D.state_ready;
if firstValid
    D.state_ready=true;D.prev_pitch_deg=pitch;D.prev_time_s=double(block.CurrentTime);D.q_filt_dps=0;q=0;
else
    [q,D]=local_filtered_q(block.CurrentTime,pitch,D);
end

[cfg,cfgRaw,cfgValid]=local_config_index_from_mass(mass,D.n_cfg,D.last_cfg,D.meta);D.cfg_mass_raw=cfgRaw;if ~cfgValid,D.cfg_invalid_count=D.cfg_invalid_count+1;end
fixed=local_base_scalar('airdropx_auto_fixed_config_id',NaN);if isfinite(fixed),cfg=min(max(round(fixed),0),D.n_cfg-1)+1;cfgRaw=cfg;end
prevCfg=D.last_cfg;cfg=max(D.last_cfg,min(cfg,D.last_cfg+1));cfg=min(max(round(cfg),1),D.n_cfg);D.cfg_changed=double(cfg~=prevCfg);D.last_cfg=cfg;

% The prediction model is scheduled from CURRENT airspeed. References may be
% changed arbitrarily while the simulation is running.
[P,Nom,uNom,i0,i1,w]=local_interpolated_model(D,Va,cfg,reqH);
pitchRef=Nom.Y(3);
ym=[h Va pitch vz q];r=[reqH reqV pitchRef 0 0];
if firstValid
    % Initial-condition synchronization only (not a recovery/reset law).
    % mpcstate.Plant is absolute, so seed it from the first complete measured
    % state.  This prevents the V50/cfg2 design anchor from leaking into a
    % V45/V55 or non-cfg2 startup through the Kalman prior.
    try,if numel(D.state.Plant)==numel(ym),D.state.Plant=ym(:);end,catch,end
end

% Tighten elevator bound to the exact physical range reachable through the
% legacy MEX external-delta channel with its once-latched hidden trim.
opt=mpcmoveopt;[eMin,eMax]=local_elevator_bounds(hidden,D.meta);
[tMin,tMax]=local_throttle_bounds(D.meta);opt.MVMin=[eMin tMin];opt.MVMax=[eMax tMax];

cmd=D.last_physical_cmd(:);ok=false;
try
    [u,info]=mpcmoveAdaptive(D.ctrl,D.state,P,Nom,ym,r,[],opt);
    D.last_iterations=double(info.Iterations);D.last_qp_code=string(info.QPCode);
    if isfield(info,'Slack'),D.last_slack=double(info.Slack);else,D.last_slack=NaN;end
    if isfield(info,'Cost'),D.last_cost=double(info.Cost);else,D.last_cost=NaN;end
    ok=D.last_iterations>0&&numel(u)>=2&&all(isfinite(u(1:2)));
    if ok
        cmd=double(u(1:2));D.mpc_success_count=D.mpc_success_count+1;
    else
        D.mpc_fail_count=D.mpc_fail_count+1;
    end
catch ME
    D.mpc_exception_count=D.mpc_exception_count+1;D.mpc_fail_count=D.mpc_fail_count+1;
    sig=string(ME.identifier)+"|"+string(ME.message);if sig~=D.last_error_signature
        fprintf(2,'[UR-MPC] mpcmoveAdaptive exception cfg%d V=%.3f: %s | %s\n',cfg-1,Va,ME.identifier,ME.message);D.last_error_signature=sig;
    end
end

% There is intentionally NO alternate recovery law. A failed QP holds the
% previous valid MPC move. Physical hard clips are final numerical safety.
cmd(1)=min(max(cmd(1),eMin),eMax);cmd(2)=min(max(cmd(2),tMin),tMax);
D.last_physical_cmd=cmd(:);plant=local_to_plant(cmd,hidden);
D=local_trace(D,block.CurrentTime,cfg,reqH,reqV,h,Va,vz,pitch,q,mass,cgx,i0,i1,w,Nom.Y,uNom,cmd,plant);
local_memory('set',block,D);block.OutputPort(1).Data=plant;
end

function Terminate(block)
D=local_memory('get',block);if isempty(D),return;end
try,assignin('base','airdropx_urmpc_controller_trace',double(D.trace));catch,end
end

function [P,Nom,uNom,i0,i1,w]=local_interpolated_model(D,v,cfg,hNom)
[i0,i1,w]=local_speed_bracket(D.speeds,v);m0=D.models(i0,cfg);m1=D.models(i1,cfg);
if i0==i1
    A=m0.A;B=m0.B;x=m0.x_nominal;u=m0.u_nominal;
else
    A=(1-w)*m0.A+w*m1.A;B=(1-w)*m0.B+w*m1.B;x=(1-w)*m0.x_nominal+w*m1.x_nominal;u=(1-w)*m0.u_nominal+w*m1.u_nominal;
end
x(1)=double(hNom);P=ss(A,[B B],eye(5),zeros(5,4),double(D.meta.Ts));P=setmpcsignals(P,'MV',[1 2],'UD',[3 4]);
Nom=struct('X',x(:),'U',[u(:);0;0],'Y',x(:),'DX',zeros(5,1));uNom=u(:);
end
function plant=local_to_plant(cmd,hidden)
plant=[double(cmd(1))-double(hidden);double(cmd(2))];plant(1)=min(max(plant(1),-0.85),0.85);plant(2)=min(max(plant(2),0),1);
end
function [lo,hi]=local_elevator_bounds(hidden,meta)
% Runtime hard bounds are the ACTUAL reachable actuator limits.  The
% trim-manifold design_mv_bounds are a model-trust diagnostic, not a hard
% authority cap.  Intersecting them here caused the V50 cfg2 run to saturate
% artificially at -0.609 while the physical channel still had authority.
d=abs(double(meta.elevator_external_delta_limit));
p=double(meta.physical_elevator_limit(:));
lo=max(p(1),hidden-d);hi=min(p(2),hidden+d);
if ~(isfinite(lo)&&isfinite(hi)&&lo<hi),error('AirdropX:URMPC:BadRuntimeElevatorBounds','Invalid runtime elevator bounds.');end
end
function [lo,hi]=local_throttle_bounds(meta)
% Same separation for throttle: [0,1] (or bank-declared physical limits) is
% the hard plant envelope; the trim-derived range remains diagnostic only.
p=double(meta.throttle_limit(:));lo=p(1);hi=p(2);
if ~(isfinite(lo)&&isfinite(hi)&&lo<hi),error('AirdropX:URMPC:BadRuntimeThrottleBounds','Invalid runtime throttle bounds.');end
end
function [hidden,D]=local_runtime_hidden(D,i0,i1,w)
if ~isfinite(D.runtime_hidden_elevator)
    override=local_base_scalar('airdropx_physics_mpc_runtime_hidden_elevator',NaN);
    if isfinite(override),D.runtime_hidden_elevator=override;else
        h=double(D.meta.hidden_offsets_by_speed(:));if i0==i1,D.runtime_hidden_elevator=h(i0);else,D.runtime_hidden_elevator=(1-w)*h(i0)+w*h(i1);end
    end
end
hidden=D.runtime_hidden_elevator;if ~isfinite(hidden),hidden=local_base_scalar('airdropx_auto_hidden_elevator_trim',NaN);D.runtime_hidden_elevator=hidden;end
if ~isfinite(hidden),error('AirdropX:URMPC:MissingHiddenTrim','No latched JSBSim hidden elevator offset.');end
end
function [cfg,rawCfg,valid]=local_config_index_from_mass(mass,n,lastCfg,meta)
if nargin<3||~isfinite(lastCfg),lastCfg=1;end;n=max(1,round(double(n)));lastCfg=min(max(round(lastCfg),1),n);
ref=local_base_scalar('airdropx_mpc_reference_mass_kg',double(meta.reference_mass_kg));cargo=local_base_scalar('airdropx_auto_cargo_mass_kg',double(meta.cargo_mass_kg));
valid=isfinite(mass)&&isfinite(ref)&&isfinite(cargo)&&cargo>0;if ~valid,rawCfg=lastCfg;cfg=lastCfg;return;end
rawCfg=round((ref-double(mass))/cargo)+1;cfg=min(max(rawCfg,1),n);cfg=max(lastCfg,cfg);cfg=min(cfg,lastCfg+1);
end
function [q,D]=local_filtered_q(t,pitch,D)
if ~isfinite(pitch)||~isfinite(t),q=D.q_filt_dps;return;end;qRaw=0;
if isfinite(D.prev_time_s)&&isfinite(D.prev_pitch_deg)&&t>D.prev_time_s,dp=mod((double(pitch)-D.prev_pitch_deg)+180,360)-180;qRaw=dp/(double(t)-D.prev_time_s);end
alpha=0.25;D.q_filt_dps=(1-alpha)*D.q_filt_dps+alpha*qRaw;D.prev_pitch_deg=double(pitch);D.prev_time_s=double(t);q=D.q_filt_dps;
end
function [h,v]=local_dynamic_reference(t,hFallback,vFallback)
h=local_base_scalar('airdropx_target_altitude_m',hFallback);v=local_base_scalar('airdropx_pd_v_ref_mps',vFallback);P=local_base_matrix('airdropx_v32_dynamic_reference_profile',[]);
if isempty(P)||size(P,2)<3,return;end;P=double(P(:,1:3));P=P(all(isfinite(P),2),:);if isempty(P),return;end;P=sortrows(P,1);idx=find(P(:,1)<=double(t)+1e-9,1,'last');if isempty(idx),idx=1;end;h=P(idx,2);v=P(idx,3);
end
function v=local_schedule_speed(reqV,actualV,speeds)
if isfinite(actualV),v=actualV;elseif isfinite(reqV),v=reqV;else,v=median(speeds);end;v=min(max(double(v),min(speeds)),max(speeds));
end
function [i0,i1,w]=local_speed_bracket(speeds,v)
speeds=double(speeds(:));n=numel(speeds);if ~isfinite(v),i0=ceil(n/2);i1=i0;w=0;return;end
if v<=speeds(1),i0=1;i1=1;w=0;return;end;if v>=speeds(end),i0=n;i1=n;w=0;return;end
i1=find(speeds>=v,1,'first');i0=i1-1;w=(v-speeds(i0))/(speeds(i1)-speeds(i0));
end
function D=local_trace(D,t,cfg,reqH,reqV,h,Va,vz,pitch,q,mass,cgx,i0,i1,w,xNom,uNom,cmd,plant)
try
    if numel(xNom)<5,xNom=NaN(5,1);end;if numel(uNom)<2,uNom=NaN(2,1);end
    row=[double(t),double(cfg-1),reqH,reqV,h,Va,vz,pitch,q,mass,cgx,double(i0),double(i1),double(w), ...
        double(xNom(2)),double(xNom(3)),double(uNom(1)),double(uNom(2)),double(cmd(1)),double(cmd(2)),double(plant(1)),double(plant(2)), ...
        double(D.mpc_success_count),double(D.mpc_fail_count),double(D.mpc_exception_count),double(D.last_iterations),double(D.last_slack),double(D.last_cost), ...
        double(D.cfg_mass_raw),double(D.cfg_invalid_count),double(D.input_invalid_count),double(D.startup_hold_count),double(D.state_ready),double(D.runtime_hidden_elevator)];
    [xEst,d1,d2,dNorm,dTailNorm,dCount]=local_estimator_snapshot(D);
    row=[row,double(xEst(:).'),double(d1),double(d2),double(dNorm),double(dTailNorm),double(dCount),double(D.cfg_changed)];
    if numel(row)==45,D.trace(end+1,:)=row;end
catch
end
end

function [xEst,d1,d2,dNorm,dTailNorm,dCount]=local_estimator_snapshot(D)
% Read-only audit snapshot.  This function MUST NOT modify the MPC state.
% The disturbance-state count is architecture-dependent.  V2.0.4 uses two
% integrated load-UD states; the v2.0.5 static-white ablation intentionally
% uses zero persistent disturbance states.  Output-disturbance states remain
% disabled in both cases.
xEst=NaN(5,1);d1=NaN;d2=NaN;dNorm=NaN;dTailNorm=NaN;dCount=NaN;
try
    x=double(D.state.Plant(:));n=min(5,numel(x));if n>0,xEst(1:n)=x(1:n);end
catch
end
try
    d=double(D.state.Disturbance(:));d=d(isfinite(d));dCount=numel(d);
    if isempty(d)
        % v2.0.5 static-white ablation has no persistent disturbance states.
        dNorm=0;dTailNorm=0;
    else
        d1=d(1);dNorm=norm(d);
        if numel(d)>=2,d2=d(2);end
        if numel(d)>2,dTailNorm=norm(d(3:end));else,dTailNorm=0;end
    end
catch
end
end
function value=local_base_scalar(name,fallback)
value=fallback;try,if evalin('base',sprintf("exist('%s','var')",name)),x=double(evalin('base',name));if isscalar(x)&&isfinite(x),value=x;end,end,catch,end
end
function value=local_base_value(name,fallback)
value=fallback;try,if evalin('base',sprintf("exist('%s','var')",name)),value=evalin('base',name);end,catch,end
end
function M=local_base_matrix(name,fallback)
M=fallback;try,if evalin('base',sprintf("exist('%s','var')",name)),M=evalin('base',name);end,catch,end
end
function value=local_memory(action,block,varargin)
persistent store;key=sprintf('b%d',round(double(block.BlockHandle)));if isempty(store),store=struct();end;value=[];
switch char(action),case 'set',store.(key)=varargin{1};value=varargin{1};case 'get',if isfield(store,key),value=store.(key);end,end
end
