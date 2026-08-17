function result = airdropx_physics_mpc_fixed_stability_scan(varargin)
%AIRDROPX_PHYSICS_MPC_FIXED_STABILITY_SCAN Fixed-H/V closed-loop certification.
%
% Stage CL-1.7 recovery-envelope validation. No CARP, no dynamic H/V commands and no controller tuning.
% One real four-drop flight is run at each speed. cfg0->cfg4 therefore uses
% the actual payload-release dynamics and mass-based controller switching.
% Speeds may run in parallel, but each speed uses a unique IC XML, SLX model
% name, output directory and Simulink cache/codegen directory.
opts=local_options(varargin{:});root=local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));
bank=local_resolve(root,opts.BankMat);if ~isfile(bank),error('AirdropX:PhysicsMPC:MissingBank','Missing bank: %s',bank);end
local_validate_bank(bank,opts.SpeedsMps);
outRoot=local_resolve(root,opts.OutputRoot);if ~isfolder(outRoot),mkdir(outRoot);end
speeds=sort(unique(double(opts.SpeedsMps(:))));
rows=cell(numel(speeds),1);errs=cell(numel(speeds),1);runInfo=cell(numel(speeds),1);
workers=max(1,min(numel(speeds),round(double(opts.ParallelWorkers))));
if workers>1
    local_prepare_pool(workers,root,opts.JobStorageRoot);
    parfor i=1:numel(speeds)
        [rows{i},errs{i},runInfo{i}]=local_run_speed(root,bank,outRoot,speeds(i),opts);
    end
else
    for i=1:numel(speeds)
        [rows{i},errs{i},runInfo{i}]=local_run_speed(root,bank,outRoot,speeds(i),opts);
    end
end
T=table();
for i=1:numel(rows),if ~isempty(rows{i}),T=[T;rows{i}];end,end %#ok<AGROW>
if isempty(T)
    error('AirdropX:PhysicsMPC:NoStabilityResults','No fixed-stability speed run completed.');
end
T=sortrows(T,{'speed_mps','cfg_id'});
writetable(T,fullfile(outRoot,'fixed_stability_summary.csv'));
F=T(~T.formal_pass,:);writetable(F,fullfile(outRoot,'fixed_stability_failures.csv'));
E=table('Size',[0 3],'VariableTypes',{'double','string','string'},'VariableNames',{'speed_mps','identifier','message'});
for i=1:numel(errs)
    if isempty(errs{i}),continue;end
    erow=table(speeds(i),string(errs{i}.identifier),string(errs{i}.message),'VariableNames',{'speed_mps','identifier','message'});E=[E;erow]; %#ok<AGROW>
end
writetable(E,fullfile(outRoot,'fixed_stability_run_errors.csv'));
passCount=sum(T.formal_pass);hardCount=sum(T.hard_pass);
fprintf('\n[PHYS-MPC CL1.7] FIXED STABILITY SCAN COMPLETE\n');
fprintf('  Formal PASS: %d / %d cfg-speed segments\n',passCount,height(T));
fprintf('  Hard-safety PASS: %d / %d\n',hardCount,height(T));
fprintf('  Summary: %s\n',fullfile(outRoot,'fixed_stability_summary.csv'));
if height(F)>0
    fprintf('  Formal failures: %d (all runs were still completed).\n',height(F));
    disp(F(:,{'speed_mps','cfg_id','h_rms_m','va_rms_mps','vz_rms_mps','q_rms_dps','pitch_std_deg','h_slope_mps','va_slope_mps2','recovery_fraction','authority_limit_fraction','formal_pass'}));
end
result=struct('summary',T,'failures',F,'run_errors',E,'runs',{runInfo},'output_root',string(outRoot),'options',opts);
end

function [T,E,info]=local_run_speed(root,bank,outRoot,v,o)
T=table();E=[];info=struct();
try
    tag=sprintf('V%03.0f',v);shortRoot=local_short_root(root,o.ShortFileGenRoot,tag);
    if logical(o.RequireDDriveTemp),local_assert_d_temp();end
    cacheDir=fullfile(shortRoot,'cache');codegenDir=fullfile(shortRoot,'codegen');icDir=fullfile(shortRoot,'ic');
    if ~isfolder(cacheDir),mkdir(cacheDir);end;if ~isfolder(codegenDir),mkdir(codegenDir);end;if ~isfolder(icDir),mkdir(icDir);end
    try,Simulink.fileGenControl('set','CacheFolder',cacheDir,'CodeGenFolder',codegenDir,'createDir',true);catch ME,fprintf('[PHYS-MPC CL1.7] fileGen warning V=%.1f: %s\n',v,ME.message);end
    B=load(bank,'v32_nodes');[~,ni]=min(abs(double([B.v32_nodes.speed_mps])-v));node=B.v32_nodes(ni);
    if abs(double(node.speed_mps)-v)>1e-6,error('AirdropX:PhysicsMPC:SpeedNodeMissing','Bank lacks speed %.1f.',v);end
    pitch0=double(node.trim_bank(1).pitch_deg);
    icPath=fullfile(icDir,sprintf('reset_H%.0f_V%.0f.xml',double(o.TargetAltitudeM),v));
    airdropx_physics_mpc_make_ic('ProjectRoot',root,'AircraftName',o.AircraftName,'OutputFile',icPath, ...
        'AirspeedMps',v,'AltitudeM',o.TargetAltitudeM,'PitchDeg',pitch0,'FlightPathDeg',0,'HeadingDeg',0);
    modelName=sprintf('airdropx_physics_mpc_cl17_%s',tag);
    speedOut=fullfile(outRoot,tag);if isfolder(speedOut),rmdir(speedOut,'s');end;mkdir(speedOut);
    fprintf('[PHYS-MPC CL1.7] START V=%.1f H=%.1f, real cfg0->cfg4 drops\n',v,o.TargetAltitudeM);
    R=airdropx_physics_mpc_run('ProjectRoot',root,'BankMat',bank,'OutputRoot',speedOut,'CaseId',sprintf('cl17_%s',tag), ...
        'ModelName',modelName,'AircraftName',o.AircraftName,'IcName',icPath, ...
        'StopTimeS',o.StopTimeS,'AfterDropTime',o.AfterDropTime,'FixedConfigId',NaN,'FixedDropTotal',4, ...
        'FixedDropStartS',o.DropStartS,'FixedDropIntervalS',o.DropIntervalS, ...
        'InitialAltitudeM',o.TargetAltitudeM,'InitialAirspeedMps',v,'TargetAltitudeM',o.TargetAltitudeM,'TargetAirspeedMps',v, ...
        'MpcEnableTimeS',o.MpcEnableTimeS,'HeightKh',o.HeightKh,'HeightKi',o.HeightKi,'HeightKaw',o.HeightKaw, ...
        'HeightVzMaxMps',o.HeightVzMaxMps,'HeightVzSlewMps2',o.HeightVzSlewMps2,'HeightBiasMaxMps',o.HeightBiasMaxMps, ...
        'SpeedAccelMps2',o.SpeedAccelMps2,'SpeedDecelMps2',o.SpeedDecelMps2,'ElevatorStepLimit',o.ElevatorStepLimit,'ThrottleStepLimit',o.ThrottleStepLimit);
    T=local_evaluate_speed(R,v,o);
    writetable(T,fullfile(speedOut,'fixed_stability_segments.csv'));
    info=struct('speed_mps',v,'output_root',string(speedOut),'timeseries_csv',R.timeseries_csv,'trace_csv',R.v32_controller_trace_csv);
    fprintf('[PHYS-MPC CL1.7] DONE V=%.1f: %d/5 formal segments pass\n',v,sum(T.formal_pass));
catch ME
    E=struct('identifier',string(ME.identifier),'message',string(ME.message));
    fprintf(2,'[PHYS-MPC CL1.7] ERROR V=%.1f: %s | %s\n',v,ME.identifier,ME.message);
    T=local_error_rows(v,E);
end
end

function T=local_evaluate_speed(R,v,o)
C=R.v32_controller_trace;P=R.timeseries;
if isempty(C),error('AirdropX:PhysicsMPC:NoTrace','Controller trace is empty at V=%.1f.',v);end
qActual=local_interp_col(P,'q_dps',double(C.time_s),double(C.q_dps));
rows=cell(5,1);
for cfg=0:4
    cfgMask=round(double(C.cfg_id))==cfg;
    if ~any(cfgMask)
        rows{cfg+1}=local_metric_row(v,cfg,NaN,NaN,0,false,false,"missing_cfg_segment",NaN(1,32));continue;
    end
    tc=double(C.time_s(cfgMask));t0=min(tc);t1=max(tc);
    evalStart=t0+double(o.SettleAfterCfgS);evalEnd=t1-double(o.EndMarginS);
    m=cfgMask & double(C.time_s)>=evalStart & double(C.time_s)<=evalEnd;
    if sum(m)<10 || evalEnd-evalStart<double(o.MinEvalDurationS)
        rows{cfg+1}=local_metric_row(v,cfg,t0,t1,sum(m),false,false,"insufficient_steady_window",NaN(1,32));continue;
    end
    tt=double(C.time_s(m));h=double(C.actual_h_m(m));va=double(C.actual_v_mps(m));vz=double(C.actual_vz_mps(m));pitch=double(C.pitch_deg(m));q=qActual(m);
    hErr=h-double(o.TargetAltitudeM);vErr=va-v;
    finite=all(isfinite([tt,h,va,vz,pitch,q]),2);finiteFrac=mean(finite);
    if sum(finite)<10
        rows{cfg+1}=local_metric_row(v,cfg,t0,t1,sum(m),false,false,"nonfinite_segment",NaN(1,32));continue;
    end
    tt=tt(finite);h=h(finite);va=va(finite);vz=vz(finite);pitch=pitch(finite);q=q(finite);hErr=hErr(finite);vErr=vErr(finite);
    hRms=sqrt(mean(hErr.^2));hMax=max(abs(hErr));vRms=sqrt(mean(vErr.^2));vMax=max(abs(vErr));vzRms=sqrt(mean(vz.^2));vzMax=max(abs(vz));qRms=sqrt(mean(q.^2));qMax=max(abs(q));
    pitchStd=std(pitch);pitchMax=max(abs(pitch));hSlope=local_slope(tt,h);vSlope=local_slope(tt,va);pitchSlope=local_slope(tt,pitch);
    elev=double(C.physical_elevator_cmd(m));thr=double(C.physical_throttle_cmd(m));elev=elev(finite);thr=thr(finite);
    eSat=mean(abs(elev)>=double(o.ElevatorSaturationAbs));tSat=mean(thr<=double(o.ThrottleSaturationLow)|thr>=double(o.ThrottleSaturationHigh));
    wholeIdx=find(cfgMask);fc=double(C.mpc_fail_count(wholeIdx));failInc=max(fc)-min(fc);
    qCtl=double(C.q_dps(m));qCtl=qCtl(finite);qEst=sqrt(mean((qCtl-q).^2));
    settle=local_settle_time(C,qActual,cfg,t0,t1,v,o);
    % CL-1.7 recovery diagnostics are observability only. They are not added
    % to the formal gate; the same H/V/vz/q/pitch/actuator criteria remain.
    recFrac=local_trace_fraction(C,'recovery_mode',m,@(z) z>0);
    recInc=local_counter_increment(C,'recovery_count',wholeIdx);
    recHardInc=local_counter_increment(C,'recovery_hard_count',wholeIdx);
    recEnterInc=local_counter_increment(C,'recovery_enter_count',wholeIdx);
    gateInc=local_counter_increment(C,'mpc_gate_reject_count',wholeIdx);
    authInc=local_counter_increment(C,'authority_limit_count',wholeIdx);
    authFrac=local_trace_fraction(C,'authority_limit_streak',m,@(z) z>0);
    trackInc=local_counter_increment(C,'tracking_loss_count',wholeIdx);
    trackFrac=local_trace_fraction(C,'tracking_loss_streak',m,@(z) z>0);
    hard=finiteFrac>=0.999 && pitchMax<o.HardMaxPitchDeg && qMax<o.HardMaxQDps && vzMax<o.HardMaxVzMps && eSat<o.HardMaxSatFraction && tSat<o.HardMaxSatFraction;
    formal=hard && hRms<=o.MaxHRmsM && hMax<=o.MaxHAbsM && vRms<=o.MaxVaRmsMps && vMax<=o.MaxVaAbsMps && ...
        vzRms<=o.MaxVzRmsMps && qRms<=o.MaxQRmsDps && pitchStd<=o.MaxPitchStdDeg && abs(hSlope)<=o.MaxHSlopeMps && ...
        abs(vSlope)<=o.MaxVaSlopeMps2 && abs(pitchSlope)<=o.MaxPitchSlopeDps && eSat<=o.MaxSatFraction && tSat<=o.MaxSatFraction && failInc==0;
    reason="PASS";if ~formal,reason=local_failure_reason(hRms,hMax,vRms,vMax,vzRms,qRms,pitchStd,hSlope,vSlope,pitchSlope,eSat,tSat,failInc,o);end
    vals=[finiteFrac,hRms,hMax,vRms,vMax,vzRms,vzMax,qRms,qMax,pitchStd,pitchMax,hSlope,vSlope,pitchSlope,eSat,tSat,failInc,qEst,settle,median(hErr),median(vErr),median(vz),median(q), ...
        recFrac,recInc,recHardInc,recEnterInc,gateInc,authInc,authFrac,trackInc,trackFrac];
    rows{cfg+1}=local_metric_row(v,cfg,t0,t1,numel(tt),hard,formal,reason,vals);
end
T=vertcat(rows{:});
end

function row=local_metric_row(v,cfg,t0,t1,n,hard,formal,reason,x)
names={'finite_fraction','h_rms_m','h_max_abs_m','va_rms_mps','va_max_abs_mps','vz_rms_mps','vz_max_abs_mps','q_rms_dps','q_max_abs_dps','pitch_std_deg','pitch_max_abs_deg','h_slope_mps','va_slope_mps2','pitch_slope_dps','elevator_sat_fraction','throttle_sat_fraction','mpc_fail_increment','q_est_rmse_dps','settling_time_s','tail_h_error_m','tail_va_error_mps','tail_vz_mps','tail_q_dps', ...
    'recovery_fraction','recovery_count_increment','recovery_hard_increment','recovery_enter_increment','mpc_gate_reject_increment','authority_limit_increment','authority_limit_fraction', ...
    'tracking_loss_count_increment','tracking_loss_fraction'};
S=struct('speed_mps',v,'cfg_id',cfg,'cfg_start_s',t0,'cfg_end_s',t1,'samples',n,'hard_pass',logical(hard),'formal_pass',logical(formal),'reason',string(reason));
for k=1:numel(names),S.(names{k})=x(k);end
row=struct2table(S);
end

function T=local_error_rows(v,ME)
T=table();for cfg=0:4,T=[T;local_metric_row(v,cfg,NaN,NaN,0,false,false,"run_error:"+string(ME.identifier),NaN(1,32))];end %#ok<AGROW>
end

function q=local_interp_col(P,name,t,fallback)
q=fallback;
try
    if ~ismember(name,P.Properties.VariableNames),return;end
    tp=double(P.time_s);yp=double(P.(name));m=isfinite(tp)&isfinite(yp);tp=tp(m);yp=yp(m);if numel(tp)<2,return;end
    [tp,ia]=unique(tp,'stable');yp=yp(ia);q=interp1(tp,yp,t,'linear','extrap');
catch
    q=fallback;
end
end
function x=local_counter_increment(C,name,idx)
x=NaN;try
    if ~ismember(name,C.Properties.VariableNames)||isempty(idx),return;end
    z=double(C.(name)(idx));z=z(isfinite(z));if isempty(z),return;end;x=max(z)-min(z);
catch
end
end
function f=local_trace_fraction(C,name,mask,pred)
f=NaN;try
    if ~ismember(name,C.Properties.VariableNames),return;end
    z=double(C.(name)(mask));z=z(isfinite(z));if isempty(z),return;end;f=mean(pred(z));
catch
end
end

function s=local_slope(t,y)
s=NaN;try,t=double(t(:));y=double(y(:));m=isfinite(t)&isfinite(y);if sum(m)<2,return;end,t=t(m);y=y(m);t=t-t(1);p=polyfit(t,y,1);s=p(1);catch,end
end
function st=local_settle_time(C,qActual,cfg,t0,t1,v,o)
st=NaN;try
    m=round(double(C.cfg_id))==cfg & double(C.time_s)>=t0 & double(C.time_s)<=t1;
    t=double(C.time_s(m));if numel(t)<5,return;end
    hErr=double(C.actual_h_m(m))-o.TargetAltitudeM;vErr=double(C.actual_v_mps(m))-v;vz=double(C.actual_vz_mps(m));q=qActual(m);
    good=abs(hErr)<=o.SettleHBandM & abs(vErr)<=o.SettleVaBandMps & abs(vz)<=o.SettleVzBandMps & abs(q)<=o.SettleQBandDps;
    dt=median(diff(t),'omitnan');if ~isfinite(dt)||dt<=0,dt=0.1;end;n=max(1,ceil(o.SettleHoldS/dt));
    z=conv(double(good),ones(n,1),'valid');k=find(z>=n-1e-9,1,'first');if ~isempty(k),st=t(k)-t0;end
catch
end
end
function r=local_failure_reason(hR,hM,vR,vM,vzR,qR,pStd,hSl,vSl,pSl,eSat,tSat,failInc,o)
a=strings(0,1);if hR>o.MaxHRmsM||hM>o.MaxHAbsM,a(end+1)="H";end;if vR>o.MaxVaRmsMps||vM>o.MaxVaAbsMps,a(end+1)="Va";end;if vzR>o.MaxVzRmsMps,a(end+1)="vz";end;if qR>o.MaxQRmsDps,a(end+1)="q";end;if pStd>o.MaxPitchStdDeg,a(end+1)="pitchStd";end;if abs(hSl)>o.MaxHSlopeMps,a(end+1)="hSlope";end;if abs(vSl)>o.MaxVaSlopeMps2,a(end+1)="VaSlope";end;if abs(pSl)>o.MaxPitchSlopeDps,a(end+1)="pitchSlope";end;if eSat>o.MaxSatFraction,a(end+1)="elevSat";end;if tSat>o.MaxSatFraction,a(end+1)="thrSat";end;if failInc>0,a(end+1)="mpcFail";end;if isempty(a),r="hard_safety";else,r=strjoin(a,"+");end
end
function local_validate_bank(path,speeds)
S=load(path,'v32_nodes','physics_mpc_meta');if ~isfield(S,'v32_nodes')||numel(S.v32_nodes)<numel(speeds),error('AirdropX:PhysicsMPC:IncompleteBank','Bank is incomplete.');end
for v=double(speeds(:)).'
    [d,i]=min(abs(double([S.v32_nodes.speed_mps])-v));if d>1e-6,error('AirdropX:PhysicsMPC:SpeedNodeMissing','Missing speed %.1f.',v);end
    if numel(S.v32_nodes(i).controllers)<5||any(cellfun(@isempty,S.v32_nodes(i).controllers(1:5))),error('AirdropX:PhysicsMPC:IncompleteBank','Speed %.1f lacks 5 MPC controllers.',v);end
end
end
function local_prepare_pool(n,root,jobRoot)
jobRoot=local_resolve(root,jobRoot);if ~isfolder(jobRoot),mkdir(jobRoot);end
p=gcp('nocreate');
if ~isempty(p)
    try
        % Reuse only if it is already a process pool with the requested size.
        % JobStorageLocation is checked before pool creation below; an existing
        % pool from another job is intentionally replaced to avoid C: storage.
        if p.NumWorkers==n && ~contains(class(p),'ThreadPool')
            delete(p);
        else
            delete(p);
        end
    catch
        try,delete(p);catch,end
    end
end
c=parcluster('Processes');c.JobStorageLocation=jobRoot;
fprintf('[PHYS-MPC CL1.7] Parallel JobStorageLocation = %s\n',c.JobStorageLocation);
parpool(c,n);
end
function local_assert_d_temp()
td=char(tempdir);p=lower(strrep(td,'/','\'));
if ~startsWith(p,'d:\')
    error('AirdropX:PhysicsMPC:WorkerTempStillOnC','Parallel worker tempdir is not on D: %s',td);
end
end
function p=local_short_root(root,x,tag)
p=char(string(x));if isempty(p),p=fullfile(root,'matlab','results','mpc_physics_v1','cl1_filegen',tag);else,p=fullfile(p,tag);end
if ~isfolder(p),mkdir(p);end
end
function p=local_resolve(root,x)
p=char(string(x));if isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),p=fullfile(root,p);end
end
function root=local_root(x)
if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end
end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.JobStorageRoot="D:\MATLAB_TEMP\AirdropX_physics_mpc_cl17\jobs";opts.RequireDDriveTemp=true;opts.BankMat="matlab/results/mpc_physics_v1/airdropx_physics_mpc_bank.mat";opts.OutputRoot="matlab/results/mpc_physics_v1/fixed_stability_cl17";opts.ShortFileGenRoot="D:\AXC\phys_cl17";opts.AircraftName="MQ9_Reaper";
opts.SpeedsMps=[45;50;55];opts.TargetAltitudeM=200;opts.StopTimeS=255;opts.DropStartS=50;opts.DropIntervalS=50;opts.AfterDropTime=10;opts.MpcEnableTimeS=2;opts.SettleAfterCfgS=15;opts.EndMarginS=5;opts.MinEvalDurationS=15;opts.ParallelWorkers=3;
% Controller values are fixed engineering settings; this scan NEVER tunes them.
opts.HeightKh=0.08;opts.HeightKi=0.0015;opts.HeightKaw=0.25;opts.HeightVzMaxMps=2.5;opts.HeightVzSlewMps2=0.50;opts.HeightBiasMaxMps=1.2;opts.SpeedAccelMps2=0.8;opts.SpeedDecelMps2=1.0;opts.ElevatorStepLimit=0.012;opts.ThrottleStepLimit=0.020;
% Formal fixed-flight steady-state gates.
opts.MaxHRmsM=1.5;opts.MaxHAbsM=3.0;opts.MaxVaRmsMps=0.75;opts.MaxVaAbsMps=1.5;opts.MaxVzRmsMps=0.25;opts.MaxQRmsDps=0.25;opts.MaxPitchStdDeg=0.60;opts.MaxHSlopeMps=0.10;opts.MaxVaSlopeMps2=0.05;opts.MaxPitchSlopeDps=0.05;opts.MaxSatFraction=0.01;
opts.ElevatorSaturationAbs=0.94;opts.ThrottleSaturationLow=0.01;opts.ThrottleSaturationHigh=0.99;opts.HardMaxPitchDeg=25;opts.HardMaxQDps=10;opts.HardMaxVzMps=5;opts.HardMaxSatFraction=0.10;
opts.SettleHBandM=2.0;opts.SettleVaBandMps=1.0;opts.SettleVzBandMps=0.5;opts.SettleQBandDps=0.5;opts.SettleHoldS=5;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
