function result = airdropx_physics_linearize_node(varargin)
%AIRDROPX_PHYSICS_LINEARIZE_NODE Deterministic local linearization of JSBSim.
%
% Physics-MPC v1.6 certification-horizon method:
%   1) direct target-cfg XML, no MEX rebuild;
%   2) cfg continuation is expressed in PHYSICAL elevator/throttle/pitch;
%   3) the target cfg hidden legacy MEX elevator bias is measured explicitly;
%   4) physical elevator is converted back to external delta for the old MEX;
%   5) pitch is a STATE, not a Newton actuator. When actuator residuals are
%      already good but the IC pitch is inconsistent with the steady state,
%      restart from observed pitch while preserving the exact same physical
%      elevator. This removes false long-period pitch transients.
%
% Local model:
%   x=[Va pitch vz q], u=[physical elevator throttle]
% with pitch(k+1)=pitch(k)+Ts*q(k) imposed exactly.

opts=local_options(varargin{:});
root=local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));

bank=opts.TrimBank;
if isempty(bank)||numel(bank)<5,error('AirdropX:PhysicsMPC:MissingTrimBank','TrimBank with cfg0..cfg4 is required.');end
cfg=min(max(round(double(opts.ConfigId)),0),4);
trim=bank(cfg+1);vNode=double(opts.SpeedMps);
outRoot=char(string(opts.OutputRoot));
if isempty(outRoot),outRoot=fullfile(root,'matlab','results','mpc_physics_v1',sprintf('V%06.3f',vNode),sprintf('cfg%d',cfg));end
if ~isfolder(outRoot),mkdir(outRoot);end

% ---------- 0) Build a physically continuous seed.
pitchSeed=local_field(bank(cfg+1),'pitch_deg',local_field(bank(1),'pitch_deg',4.0));
continuation=opts.ContinuationSeed;
if local_valid_continuation(continuation)
    pitchSeed=double(continuation.pitch_deg);
    hidden=local_measure_hidden_offset(root,outRoot,bank,trim,cfg,vNode,pitchSeed,opts,900);
    requestedPhysicalElev=double(continuation.physical_elevator_cmd);
    trim.elevator_cmd=local_clip(requestedPhysicalElev-hidden,double(opts.ExternalElevatorBounds));
    trim.throttle_cmd=local_clip(double(continuation.throttle_cmd),double(opts.ThrottleBounds));
    local_write_seed(fullfile(outRoot,'continuation_seed.csv'),continuation,pitchSeed,hidden,trim);
    fprintf(['[PHYS-MPC] V=%.1f cfg%d physical continuation from cfg%d: pitch=%.4f ', ...
        'physE=%.6f hidden=%.6f -> extE=%.6f th=%.6f\n'], ...
        vNode,cfg,round(double(continuation.source_cfg)),pitchSeed,requestedPhysicalElev,hidden,double(trim.elevator_cmd),double(trim.throttle_cmd));
end

% v1.6 rule: the certifying trim solver MUST use the exact same observation
% horizon/tail window as the formal baseline certification. A shorter solver
% horizon may generate a transient near-pass that drifts outside the formal gate.
% ---------- 1) Equilibrium + pitch-state consistency.
eqPass=false;stats=struct();solveInfo=struct();consRows={};
for pc=0:round(double(opts.MaxPitchConsistencyIterations))
    baseRoot=fullfile(outRoot,sprintf('baseline_pc%02d',pc));
    base=local_run(root,baseRoot,bank,trim,cfg,vNode,0,0,opts.BaselineStopTimeS,Inf,local_seed(opts,cfg,10*pc),opts,pitchSeed);
    stats=local_baseline_stats(base.timeseries,vNode,opts);
    local_write_baseline(stats,fullfile(outRoot,sprintf('baseline_pc%02d_metrics.csv',pc)));
    if pc==0,local_write_baseline(stats,fullfile(outRoot,'baseline_metrics.csv'));end

    if ~stats.pass && logical(opts.AllowDeterministicRetrim)
        fprintf('[PHYS-MPC] V=%.1f cfg%d pc%d baseline not equilibrium; deterministic 5x2 solve at fixed pitch %.4f deg.\n',vNode,cfg,pc,pitchSeed);
        [trim,solveInfo]=airdropx_physics_trim_solve('ProjectRoot',root,'OutputRoot',fullfile(outRoot,sprintf('deterministic_retrim_pc%02d',pc)), ...
            'TrimBank',bank,'InitialTrim',trim,'ConfigId',cfg,'SpeedMps',vNode,'FixedInitialPitchDeg',pitchSeed,'ReturnBestOnFailure',true, ...
            'ReferenceAltitudeM',opts.ReferenceAltitudeM,'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg, ...
            'StopTimeS',opts.BaselineStopTimeS,'TailWindowS',opts.BaselineTailS,'MaxIterations',opts.RetrimMaxIterations, ...
            'ElevatorProbe',opts.RetrimElevatorProbe,'ThrottleProbe',opts.RetrimThrottleProbe, ...
            'MaxElevatorStep',opts.RetrimMaxElevatorStep,'MaxThrottleStep',opts.RetrimMaxThrottleStep, ...
            'MaxVaErrorMps',opts.MaxBaselineVaErrorMps,'MaxAbsVzMps',opts.MaxBaselineAbsVzMps, ...
            'MaxAbsQDps',opts.MaxBaselineAbsQDps,'MaxHeightSlopeMps',opts.MaxBaselineHeightSlopeMps, ...
            'MaxVaSlopeMps2',opts.MaxBaselineVaSlopeMps2,'MaxPitchStdDeg',opts.MaxBaselinePitchStdDeg);
        baseRoot=fullfile(outRoot,sprintf('baseline_after_retrim_pc%02d',pc));
        base=local_run(root,baseRoot,bank,trim,cfg,vNode,0,0,opts.BaselineStopTimeS,Inf,local_seed(opts,cfg,50+10*pc),opts,pitchSeed);
        stats=local_baseline_stats(base.timeseries,vNode,opts);
        local_write_baseline(stats,fullfile(outRoot,sprintf('baseline_after_retrim_pc%02d_metrics.csv',pc)));
        if pc==0,local_write_baseline(stats,fullfile(outRoot,'baseline_after_retrim_metrics.csv'));end
    end

    dynamicPass=local_dynamic_residual_pass(stats,opts);
    pitchMismatch=abs(double(stats.pitch_deg)-double(pitchSeed));
    consRows(end+1,:)={pc,pitchSeed,stats.pitch_deg,pitchMismatch,stats.elevator_physical,stats.throttle_physical, ...
        stats.va_error_mps,stats.vz_mps,stats.q_dps,stats.h_slope_mps,stats.va_slope_mps2,stats.pitch_std_deg,dynamicPass,stats.pass}; %#ok<AGROW>

    if stats.pass && pitchMismatch<=double(opts.PitchConsistencyTolDeg)
        eqPass=true;break;
    end

    % If the control-equilibrium residuals are already inside the hard gate,
    % a remaining pitchStd failure is an IC-state mismatch, not evidence that
    % elevator/throttle are wrong. Restart at observed steady pitch while
    % preserving EXACT physical elevator and throttle.
    if dynamicPass && isfinite(stats.pitch_deg) && pc<round(double(opts.MaxPitchConsistencyIterations))
        oldPitch=pitchSeed;physicalElev=stats.elevator_physical;throttle=stats.throttle_physical;
        pitchSeed=double(stats.pitch_deg);
        hidden=local_measure_hidden_offset(root,outRoot,bank,trim,cfg,vNode,pitchSeed,opts,950+pc);
        trim.elevator_cmd=local_clip(physicalElev-hidden,double(opts.ExternalElevatorBounds));
        trim.throttle_cmd=local_clip(throttle,double(opts.ThrottleBounds));
        fprintf(['[PHYS-MPC] V=%.1f cfg%d pitch-state restart: %.4f -> %.4f deg, ', ...
            'preserve physE=%.6f using hidden=%.6f -> extE=%.6f, th=%.6f\n'], ...
            vNode,cfg,oldPitch,pitchSeed,physicalElev,hidden,double(trim.elevator_cmd),double(trim.throttle_cmd));
        continue;
    end
    break;
end
local_write_consistency(consRows,fullfile(outRoot,'pitch_state_consistency.csv'));

if ~eqPass
    if isempty(fieldnames(stats))
        error('AirdropX:PhysicsMPC:TrimNotEquilibrium','V=%.1f cfg%d did not produce finite equilibrium statistics.',vNode,cfg);
    end
    error('AirdropX:PhysicsMPC:TrimNotEquilibrium', ...
        ['V=%.1f cfg%d is not a certified state-consistent physical equilibrium. ', ...
         'VaErr=%.3f m/s, vz=%.3f m/s, q=%.3f dps, hSlope=%.4f m/s, ', ...
         'VaSlope=%.4f m/s^2, pitchStd=%.3f deg, pitchSeed=%.3f observedPitch=%.3f.'], ...
        vNode,cfg,stats.va_error_mps,stats.vz_mps,stats.q_dps,stats.h_slope_mps,stats.va_slope_mps2, ...
        stats.pitch_std_deg,pitchSeed,stats.pitch_deg);
end

% ---------- 1b) Independent repeatability at the final state-consistent IC.
reproRoot=fullfile(outRoot,'baseline_repeatability');
repro=local_run(root,reproRoot,bank,trim,cfg,vNode,0,0,opts.BaselineStopTimeS,Inf,local_seed(opts,cfg,77),opts,pitchSeed);
reproStats=local_baseline_stats(repro.timeseries,vNode,opts);
[repeatPass,repeatDiff]=local_repeatability(stats,reproStats,opts);
local_write_repeatability(stats,reproStats,repeatDiff,repeatPass,fullfile(outRoot,'baseline_repeatability_metrics.csv'));
if ~reproStats.pass||~repeatPass
    error('AirdropX:PhysicsMPC:EquilibriumNotRepeatable', ...
        ['V=%.1f cfg%d equilibrium is not repeatable. SecondRunPass=%d, dVaErr=%.4f dVz=%.4f dQ=%.4f ', ...
         'dHSlope=%.4f dVaSlope=%.4f dPitch=%.4f dElev=%.6f dThrottle=%.6f.'], ...
        vNode,cfg,reproStats.pass,repeatDiff.va_error_mps,repeatDiff.vz_mps,repeatDiff.q_dps, ...
        repeatDiff.h_slope_mps,repeatDiff.va_slope_mps2,repeatDiff.pitch_deg,repeatDiff.elevator_physical,repeatDiff.throttle_physical);
end

xNom=[stats.va_mps;stats.pitch_deg;stats.vz_mps;stats.q_dps];
uNom=[stats.elevator_physical;stats.throttle_physical];
trimOut=trim;
trimOut.airspeed_mps=xNom(1);trimOut.pitch_deg=xNom(2);trimOut.vz_up_mps=xNom(3);trimOut.q_dps=xNom(4);
trimOut.throttle_cmd=uNom(2);trimOut.physical_elevator_cmd=uNom(1);

% ---------- 2) Deterministic perturbation records.
runData=cell(max(2,round(double(opts.RunsPerNode))),1);
for r=1:numel(runData)
    runRoot=fullfile(outRoot,sprintf('jacobian_run_%02d',r));
    stopS=double(opts.ExcitationStartS)+double(opts.ExcitationDurationS);
    rr=local_run(root,runRoot,bank,trimOut,cfg,vNode,opts.ElevatorAmplitude,opts.ThrottleAmplitude, ...
        stopS,double(opts.ExcitationStartS),local_seed(opts,cfg,r),opts,pitchSeed);
    runData{r}=local_dataset(rr.timeseries,xNom,uNom,double(opts.ExcitationStartS),double(opts.Ts),opts);
end
fitData=runData(1:end-1);valData=runData{end};
[Ad,Bd,fitDiag]=local_fit(fitData,double(opts.Ts),opts);
val=local_validate(Ad,Bd,valData,opts);
if logical(opts.FailOnPoorFit)&&~val.pass
    error('AirdropX:PhysicsMPC:PoorLocalModel', ...
        'V=%.1f cfg%d local model failed held-out validation. 1-step RMSE=[%s], 10-step RMSE=[%s].', ...
        vNode,cfg,local_vec(val.rmse_1step),local_vec(val.rmse_10step));
end

plant=ss(Ad,Bd,eye(4),zeros(4,2),double(opts.Ts));
plant.StateName={'dVa_mps','dPitch_deg','dVz_mps','dQ_dps'};
plant.InputName={'dElevatorPhysical','dThrottle'};
plant.OutputName={'dVa_mps','dPitch_deg','dVz_mps','dQ_dps'};
stateNames=string(plant.OutputName(:));
metrics=table(stateNames,val.rmse_1step(:),val.rmse_10step(:),val.r2_1step(:),'VariableNames',{'state','rmse_1step','rmse_10step','r2_1step'});
writetable(metrics,fullfile(outRoot,'linear_model_validation.csv'));

result=struct();
result.version='physics_mpc_v1_6_cert_horizon_nomex';
result.speed_mps=vNode;result.config_id=cfg;result.trim=trimOut;result.x_nominal=xNom;result.u_nominal=uNom;
result.final_initial_pitch_seed_deg=pitchSeed;result.Ad=Ad;result.Bd=Bd;result.plant=plant;
result.baseline=stats;result.baseline_repeat=reproStats;result.repeatability=repeatDiff;result.fit=fitDiag;result.validation=val;
result.continuation_seed=continuation;result.options=opts;
save(fullfile(outRoot,'physics_linear_model.mat'),'result','-v7.3');
fprintf('[PHYS-MPC] V=%.1f cfg%d PASS: pitchIC=%.4f rho(Ad)=%.4f, rank=%d, ctrlRank=%d, cond=%.3g, 10-step RMSE=[%s]\n', ...
    vNode,cfg,pitchSeed,max(abs(eig(Ad))),fitDiag.regressor_rank,fitDiag.controllability_rank,fitDiag.regressor_condition,local_vec(val.rmse_10step));
end

function rr=local_run(root,outRoot,bank,trim,cfg,vNode,eAmp,tAmp,stopS,exciteStart,seed,o,pitchSeed)
if ~isfolder(outRoot),mkdir(outRoot);end
rr=airdropx_auto_run_id_experiment('ProjectRoot',root,'OutputRoot',outRoot,'RunId',sprintf('phys_v%.1f_cfg%d_s%d',vNode,cfg,seed), ...
    'ConfigId',cfg,'InitialDropCount',cfg,'PrepareByDrops',false,'DirectCfgViaAircraftXml',true,'Trim',trim,'StopTimeS',stopS,'RecordStartS',0,'ExportStartS',0, ...
    'ExcitationStartS',exciteStart,'PrepDropStartS',stopS+100,'PrepDropIntervalS',1.0,'KeepFixedConfigurationOnly',true,'DirectIdMode',true,'PreparationTrimBank',bank, ...
    'UsePreparationTrimSchedule',false,'OperatingPointWindowS',3.0,'Seed',seed,'InitialAirspeedMps',vNode,'InitialAltitudeM',o.ReferenceAltitudeM, ...
    'InitialPitchDeg',double(pitchSeed),'InitialFlightPathDeg',0,'TargetAltitudeM',o.ReferenceAltitudeM,'TargetAirspeedMps',vNode, ...
    'ReferenceMassKg',o.ReferenceMassKg,'CargoMassKg',o.CargoMassKg,'IsolateGeneratedIc',true, ...
    'ElevatorAmplitude',eAmp,'ThrottleAmplitude',tAmp,'ElevatorHoldTimeRangeS',double(o.ElevatorHoldTimeRangeS),'ThrottleHoldTimeRangeS',double(o.ThrottleHoldTimeRangeS));
end

function hidden=local_measure_hidden_offset(root,outRoot,bank,trim,cfg,vNode,pitchSeed,o,seed)
probe=trim;probe.elevator_cmd=0.0;
probeRoot=fullfile(outRoot,sprintf('hidden_probe_pitch_%+.4f_seed%d',pitchSeed,seed));
rr=local_run(root,probeRoot,bank,probe,cfg,vNode,0,0,double(o.HiddenProbeStopTimeS),Inf,local_seed(o,cfg,seed),o,pitchSeed);
T=rr.timeseries;t=double(T.time_s(:));
e=local_first_col(T,{'elevator_physical_actual','elevator_cmd_norm','elevator_cmd_actual','elevator_delta'});
m=isfinite(t)&isfinite(e)&t<=min(max(t),min(t)+double(o.HiddenProbeWindowS));
if nnz(m)<3,m=isfinite(e);end
if nnz(m)<1,error('AirdropX:PhysicsMPC:HiddenOffsetProbeFailed','Cannot measure hidden elevator offset V=%.1f cfg%d.',vNode,cfg);end
hidden=median(e(m),'omitnan');
if ~isfinite(hidden),error('AirdropX:PhysicsMPC:HiddenOffsetProbeFailed','Non-finite hidden elevator offset V=%.1f cfg%d.',vNode,cfg);end
end

function tf=local_valid_continuation(s)
tf=isstruct(s)&&isfield(s,'physical_elevator_cmd')&&isfield(s,'throttle_cmd')&&isfield(s,'pitch_deg')&& ...
    all(isfinite([double(s.physical_elevator_cmd),double(s.throttle_cmd),double(s.pitch_deg)]));
end
function tf=local_dynamic_residual_pass(s,o)
tf=abs(s.va_error_mps)<=double(o.MaxBaselineVaErrorMps)&&abs(s.vz_mps)<=double(o.MaxBaselineAbsVzMps)&& ...
   abs(s.q_dps)<=double(o.MaxBaselineAbsQDps)&&abs(s.h_slope_mps)<=double(o.MaxBaselineHeightSlopeMps)&& ...
   abs(s.va_slope_mps2)<=double(o.MaxBaselineVaSlopeMps2);
end
function local_write_seed(path,s,pitch,hidden,trim)
sourceCfg=NaN;if isfield(s,'source_cfg'),sourceCfg=double(s.source_cfg);end
T=table(sourceCfg,double(s.physical_elevator_cmd),double(s.throttle_cmd),double(s.pitch_deg),double(pitch),double(hidden),double(trim.elevator_cmd), ...
    'VariableNames',{'source_cfg','requested_physical_elevator','requested_throttle','source_pitch_deg','target_pitch_seed_deg','target_hidden_elevator','target_external_elevator_delta'});
writetable(T,path);
end
function local_write_consistency(rows,path)
if isempty(rows),return;end
T=cell2table(rows,'VariableNames',{'iteration','pitch_seed_deg','observed_pitch_deg','pitch_mismatch_deg','elevator_physical','throttle_physical', ...
    'va_error_mps','vz_mps','q_dps','h_slope_mps','va_slope_mps2','pitch_std_deg','dynamic_residual_pass','full_pass'});
writetable(T,path);
end
function s=local_baseline_stats(T,vNode,o)
need={'time_s','altitude_m','airspeed_mps','pitch_deg','vz_up_mps','q_dps'};
for k=1:numel(need),if ~ismember(need{k},T.Properties.VariableNames),error('AirdropX:PhysicsMPC:MissingSignal','Baseline is missing %s.',need{k});end,end
t=double(T.time_s(:));if isempty(t),error('AirdropX:PhysicsMPC:EmptyBaseline','Empty baseline record.');end
startT=max(min(t),max(t)-double(o.BaselineTailS));m=isfinite(t)&t>=startT;
va=double(T.airspeed_mps(:));p=double(T.pitch_deg(:));vz=double(T.vz_up_mps(:));q=double(T.q_dps(:));h=double(T.altitude_m(:));
m=m&isfinite(va)&isfinite(p)&isfinite(vz)&isfinite(q)&isfinite(h);if nnz(m)<20,error('AirdropX:PhysicsMPC:ShortBaseline','Not enough finite baseline samples.');end
s.va_mps=median(va(m),'omitnan');s.pitch_deg=median(p(m),'omitnan');s.vz_mps=median(vz(m),'omitnan');s.q_dps=median(q(m),'omitnan');
s.va_error_mps=s.va_mps-vNode;s.pitch_std_deg=std(p(m),0,'omitnan');s.h_slope_mps=local_slope(t(m),h(m));s.va_slope_mps2=local_slope(t(m),va(m));
s.elevator_physical=local_median_field(T,m,{'elevator_physical_actual','elevator_cmd_norm','elevator_cmd_actual','elevator_delta'},NaN);
s.throttle_physical=local_median_field(T,m,{'throttle_physical_actual','throttle_norm','throttle_cmd_actual','throttle_cmd'},NaN);
if ~all(isfinite([s.elevator_physical s.throttle_physical])),error('AirdropX:PhysicsMPC:MissingActuatorFeedback','Cannot determine physical baseline actuator commands.');end
s.pass=local_dynamic_residual_pass(s,o)&&s.pitch_std_deg<=double(o.MaxBaselinePitchStdDeg);
end
function x=local_clip(x,b),x=min(max(x,min(b)),max(b));end

function D = local_dataset(T,xNom,uNom,excitationStart,Ts,o)
t = double(T.time_s(:));
va = local_col(T,'airspeed_mps'); p = local_col(T,'pitch_deg'); vz = local_col(T,'vz_up_mps'); q = local_col(T,'q_dps');
e = local_first_col(T,{'elevator_physical_actual','elevator_cmd_norm','elevator_cmd_actual','elevator_delta'});
th = local_first_col(T,{'throttle_physical_actual','throttle_norm','throttle_cmd_actual','throttle_cmd'});
allv = [t va p vz q e th];
m = all(isfinite(allv),2) & t >= excitationStart + double(o.DiscardAfterExcitationStartS);
t=t(m); va=va(m); p=p(m); vz=vz(m); q=q(m); e=e(m); th=th(m);
if numel(t)<20,error('AirdropX:PhysicsMPC:ShortJacobianRecord','Too few local-linearization samples.');end
[t,ia]=unique(t,'stable');va=va(ia);p=p(ia);vz=vz(ia);q=q(ia);e=e(ia);th=th(ia);
t0=ceil(t(1)/Ts)*Ts; t1=floor(t(end)/Ts)*Ts; tg=(t0:Ts:t1).';
if numel(tg)<15,error('AirdropX:PhysicsMPC:ShortResampledRecord','Too few 0.1-s samples.');end
va=interp1(t,va,tg,'linear');p=interp1(t,p,tg,'linear');vz=interp1(t,vz,tg,'linear');q=interp1(t,q,tg,'linear');
e=interp1(t,e,tg,'previous','extrap');th=interp1(t,th,tg,'previous','extrap');
% Mild fixed smoothing only suppresses the finite-difference q measurement
% noise.  It is not a tuned filter and is identical for every node.
q=movmean(q,3,'Endpoints','shrink');
X=[va.';p.';vz.';q.']-xNom(:);
U=[e.';th.']-uNom(:);
lim=double(o.LocalStateLimits(:));
keep=all(abs(X)<=lim,1) & all(isfinite(X),1) & all(isfinite(U),1);
kept=nnz(keep);total=numel(keep);
maxAbs=max(abs(X),[],2,'omitnan');
if kept<15
    error('AirdropX:PhysicsMPC:NoLocalSamples', ...
        'Local-state gate kept only %d/%d samples. max|dX|=[%s], limits=[%s].', ...
        kept,total,local_vec(maxAbs),local_vec(lim));
end
% Preserve the full time grid. Do not collapse holes created by the local
% state gate and then treat nonadjacent samples as one 0.1-s transition.
D=struct('time_s',tg(:),'X',X,'U',U,'keep',logical(keep(:).'));
end

function [Ad,Bd,d] = local_fit(data,Ts,o)
Z=[];Y=[];
for r=1:numel(data)
    X=data{r}.X;U=data{r}.U;t=data{r}.time_s(:).';keep=data{r}.keep;
    n=min(size(X,2)-1,size(U,2));
    if n<5,continue;end
    idx=find(keep(1:n) & keep(2:n+1) & abs(diff(t(1:n+1))-Ts)<=max(1e-9,Ts*1e-4));
    if numel(idx)<5,continue;end
    Z=[Z; [X(:,idx).' U(:,idx).']]; %#ok<AGROW>
    Y=[Y; X(:,idx+1).']; %#ok<AGROW>
end
if size(Z,1)<30,error('AirdropX:PhysicsMPC:InsufficientRegressionData','Insufficient local regression rows.');end
scale=std(Z,0,1);scale(~isfinite(scale)|scale<1e-5)=1;
Zs=Z./scale;
rnk=rank(Zs); cnd=cond(Zs.'*Zs+double(o.RidgeLambda)*eye(size(Zs,2)));
if rnk<6,error('AirdropX:PhysicsMPC:UnexcitedJacobian','Regressor rank %d < 6. Increase fixed small excitation.',rnk);end
Ad=zeros(4);Bd=zeros(4,2);
rows=[1 3 4];
for rr=rows
    theta=(Zs.'*Zs+double(o.RidgeLambda)*eye(6))\(Zs.'*Y(:,rr));
    beta=theta./scale(:);
    Ad(rr,:)=beta(1:4).';Bd(rr,:)=beta(5:6).';
end
% Exact pitch-rate kinematics in degrees: pitch[k+1]=pitch[k]+Ts*q[k].
Ad(2,:)=[0 1 0 Ts];Bd(2,:)=[0 0];
ctrlRank=rank(ctrb(Ad,Bd));
d=struct('regressor_rank',rnk,'regressor_condition',cnd,'rows',size(Z,1),'spectral_radius',max(abs(eig(Ad))),'controllability_rank',ctrlRank);
if ctrlRank<4
    error('AirdropX:PhysicsMPC:UncontrollableLocalModel','Local model controllability rank %d < 4.',ctrlRank);
end
if d.regressor_condition>double(o.MaxRegressorCondition)
    error('AirdropX:PhysicsMPC:IllConditionedJacobian','Regressor condition %.3g exceeds %.3g.',d.regressor_condition,double(o.MaxRegressorCondition));
end
end

function v = local_validate(Ad,Bd,D,o)
X=D.X;U=D.U;t=D.time_s(:).';keep=D.keep;n=min(size(X,2)-1,size(U,2));
idx=find(keep(1:n) & keep(2:n+1) & abs(diff(t(1:n+1))-double(o.Ts))<=max(1e-9,double(o.Ts)*1e-4));
if numel(idx)<5
    error('AirdropX:PhysicsMPC:NoValidationTransitions','Too few contiguous local transitions for held-out validation.');
end
P1=Ad*X(:,idx)+Bd*U(:,idx); E1=P1-X(:,idx+1);
rmse1=sqrt(mean(E1.^2,2,'omitnan'));
r2=zeros(4,1);
for i=1:4
    yy=X(i,idx+1);den=sum((yy-mean(yy,'omitnan')).^2,'omitnan');
    if den>1e-12,r2(i)=1-sum(E1(i,:).^2,'omitnan')/den;else,r2(i)=NaN;end
end
H=max(1,round(double(o.ValidationHorizonSteps)));sq=zeros(4,0);
for k=1:max(0,n-H)
    if ~all(keep(k:k+H)),continue;end
    if any(abs(diff(t(k:k+H))-double(o.Ts))>max(1e-9,double(o.Ts)*1e-4)),continue;end
    xp=X(:,k);
    for j=0:H-1,xp=Ad*xp+Bd*U(:,k+j);end
    sq(:,end+1)=(xp-X(:,k+H)).^2; %#ok<AGROW>
end
if isempty(sq),rmseH=Inf(4,1);else,rmseH=sqrt(mean(sq,2,'omitnan'));end
v=struct('rmse_1step',rmse1,'rmse_10step',rmseH,'r2_1step',r2, ...
    'pass',all(rmse1<=double(o.MaxOneStepRmse(:)))&&all(rmseH<=double(o.MaxTenStepRmse(:))));
end

function [pass,d]=local_repeatability(a,b,o)
names={'va_error_mps','vz_mps','q_dps','h_slope_mps','va_slope_mps2','pitch_deg','elevator_physical','throttle_physical'};
for k=1:numel(names),n=names{k};d.(n)=abs(double(a.(n))-double(b.(n)));end
pass=d.va_error_mps<=double(o.MaxRepeatVaErrorDiffMps) && ...
    d.vz_mps<=double(o.MaxRepeatVzDiffMps) && ...
    d.q_dps<=double(o.MaxRepeatQDiffDps) && ...
    d.h_slope_mps<=double(o.MaxRepeatHeightSlopeDiffMps) && ...
    d.va_slope_mps2<=double(o.MaxRepeatVaSlopeDiffMps2) && ...
    d.pitch_deg<=double(o.MaxRepeatPitchDiffDeg) && ...
    d.elevator_physical<=double(o.MaxRepeatElevatorDiff) && ...
    d.throttle_physical<=double(o.MaxRepeatThrottleDiff);
end

function local_write_repeatability(a,b,d,pass,path)
T=table(["first";"repeat";"abs_diff"], ...
    [a.va_error_mps;b.va_error_mps;d.va_error_mps], ...
    [a.vz_mps;b.vz_mps;d.vz_mps], ...
    [a.q_dps;b.q_dps;d.q_dps], ...
    [a.h_slope_mps;b.h_slope_mps;d.h_slope_mps], ...
    [a.va_slope_mps2;b.va_slope_mps2;d.va_slope_mps2], ...
    [a.pitch_deg;b.pitch_deg;d.pitch_deg], ...
    [a.elevator_physical;b.elevator_physical;d.elevator_physical], ...
    [a.throttle_physical;b.throttle_physical;d.throttle_physical], ...
    'VariableNames',{'run','va_error_mps','vz_mps','q_dps','h_slope_mps','va_slope_mps2','pitch_deg','elevator_physical','throttle_physical'});
T.repeatability_pass=repmat(logical(pass),height(T),1);writetable(T,path);
end

function local_write_baseline(s,path)
T=struct2table(rmfield(s,'pass'));T.pass=repmat(logical(s.pass),height(T),1);writetable(T,path);
end
function y=local_col(T,n),y=double(T.(n)(:));end
function y=local_first_col(T,names)
y=NaN(height(T),1);for k=1:numel(names),if ismember(names{k},T.Properties.VariableNames),z=double(T.(names{k})(:));if any(isfinite(z)),y=z;return;end,end,end
end
function x=local_median_field(T,m,names,fallback)
x=fallback;for k=1:numel(names),if ismember(names{k},T.Properties.VariableNames),z=double(T.(names{k})(:));z=z(m&isfinite(z));if ~isempty(z),x=median(z,'omitnan');return;end,end,end
end
function s=local_slope(t,y),p=polyfit(double(t(:))-double(t(1)),double(y(:)),1);s=p(1);end
function seed=local_seed(o,cfg,r),seed=round(double(o.SeedBase)+1000*double(o.SpeedMps)+100*cfg+r);end
function v=local_field(s,n,d),v=d;try,if isfield(s,n)&&~isempty(s.(n))&&isfinite(double(s.(n))),v=double(s.(n));end,catch,end,end
function s=local_vec(x),s=strtrim(sprintf('%.4g ',double(x(:))));end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end

function opts=local_options(varargin)
opts.ProjectRoot="";opts.OutputRoot="";opts.TrimBank=[];opts.ConfigId=0;opts.SpeedMps=50;opts.ContinuationSeed=[];
opts.ReferenceAltitudeM=200;opts.ReferenceMassKg=3423;opts.CargoMassKg=300;opts.Ts=0.1;
opts.BaselineStopTimeS=28;opts.BaselineTailS=10;opts.ExcitationStartS=18;opts.ExcitationDurationS=30;
opts.RunsPerNode=2;opts.SeedBase=3300;opts.ElevatorAmplitude=0.012;opts.ThrottleAmplitude=0.025;
opts.ElevatorHoldTimeRangeS=[0.50 1.40];opts.ThrottleHoldTimeRangeS=[0.90 2.40];opts.DiscardAfterExcitationStartS=0.5;
opts.LocalStateLimits=[5;12;3.5;8];opts.RidgeLambda=1e-6;opts.MaxRegressorCondition=1e6;
opts.MaxBaselineVaErrorMps=0.50;opts.MaxBaselineAbsVzMps=0.15;opts.MaxBaselineAbsQDps=0.15;
opts.MaxBaselineHeightSlopeMps=0.15;opts.MaxBaselineVaSlopeMps2=0.05;opts.MaxBaselinePitchStdDeg=0.50;
opts.ValidationHorizonSteps=10;opts.MaxOneStepRmse=[0.30;0.30;0.30;0.70];opts.MaxTenStepRmse=[1.50;1.50;1.00;2.50];opts.FailOnPoorFit=true;
opts.MaxRepeatVaErrorDiffMps=0.10;opts.MaxRepeatVzDiffMps=0.05;opts.MaxRepeatQDiffDps=0.05;
opts.MaxRepeatHeightSlopeDiffMps=0.05;opts.MaxRepeatVaSlopeDiffMps2=0.02;opts.MaxRepeatPitchDiffDeg=0.20;
opts.MaxRepeatElevatorDiff=5e-4;opts.MaxRepeatThrottleDiff=5e-4;
opts.AllowDeterministicRetrim=true;opts.RetrimMaxIterations=7;opts.RetrimStopTimeS=28;opts.RetrimTailWindowS=10;opts.RetrimElevatorProbe=0.008;opts.RetrimThrottleProbe=0.015;opts.RetrimMaxElevatorStep=0.05;opts.RetrimMaxThrottleStep=0.08;
opts.MaxPitchConsistencyIterations=3;opts.PitchConsistencyTolDeg=0.20;opts.HiddenProbeStopTimeS=0.8;opts.HiddenProbeWindowS=0.30;
opts.ExternalElevatorBounds=[-0.75 0.45];opts.ThrottleBounds=[0.25 0.95];
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
