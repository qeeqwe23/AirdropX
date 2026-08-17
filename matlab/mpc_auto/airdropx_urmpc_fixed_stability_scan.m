function result = airdropx_urmpc_fixed_stability_scan(varargin)
%AIRDROPX_URMPC_FIXED_STABILITY_SCAN Fixed H/V real four-drop test of ONE MPC.
% No recovery controller, no height PI, no cfg-specific tuning.
opts=local_options(varargin{:});root=local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));
bank=local_resolve(root,opts.BankMat);if ~isfile(bank),error('AirdropX:URMPC:MissingBank','Missing bank: %s',bank);end
local_validate_bank(bank,opts.SpeedsMps);
outRoot=local_resolve(root,opts.OutputRoot);if ~isfolder(outRoot),mkdir(outRoot);end
speeds=sort(unique(double(opts.SpeedsMps(:))));rows=cell(numel(speeds),1);errs=cell(numel(speeds),1);runInfo=cell(numel(speeds),1);
workers=max(1,min(numel(speeds),round(double(opts.ParallelWorkers))));
if workers>1
    local_prepare_pool(workers,root,opts.JobStorageRoot);
    parfor i=1:numel(speeds),[rows{i},errs{i},runInfo{i}]=local_run_speed(root,bank,outRoot,speeds(i),opts);end
else
    for i=1:numel(speeds),[rows{i},errs{i},runInfo{i}]=local_run_speed(root,bank,outRoot,speeds(i),opts);end
end
T=table();for i=1:numel(rows),if ~isempty(rows{i}),T=[T;rows{i}];end,end %#ok<AGROW>
if isempty(T),error('AirdropX:URMPC:NoStabilityResults','No UR-MPC fixed-stability run completed.');end
T=sortrows(T,{'speed_mps','cfg_id'});writetable(T,fullfile(outRoot,'fixed_stability_summary.csv'));
F=T(~T.formal_pass,:);writetable(F,fullfile(outRoot,'fixed_stability_failures.csv'));
E=table('Size',[0 3],'VariableTypes',{'double','string','string'},'VariableNames',{'speed_mps','identifier','message'});
for i=1:numel(errs),if isempty(errs{i}),continue;end,E=[E;table(speeds(i),string(errs{i}.identifier),string(errs{i}.message),'VariableNames',{'speed_mps','identifier','message'})];end %#ok<AGROW>
writetable(E,fullfile(outRoot,'fixed_stability_run_errors.csv'));
fprintf('\n[UR-MPC v2.0] FIXED STABILITY COMPLETE\n  Formal PASS: %d/%d\n  Hard PASS: %d/%d\n  Summary: %s\n',sum(T.formal_pass),height(T),sum(T.hard_pass),height(T),fullfile(outRoot,'fixed_stability_summary.csv'));
if ~isempty(F),disp(F(:,{'speed_mps','cfg_id','h_rms_m','va_rms_mps','vz_rms_mps','q_rms_dps','pitch_std_deg','mpc_fail_increment','formal_pass','reason'}));end
result=struct('summary',T,'failures',F,'run_errors',E,'runs',{runInfo},'output_root',string(outRoot),'options',opts);
end

function [T,E,info]=local_run_speed(root,bank,outRoot,v,o)
T=table();E=[];info=struct();
try
    tag=sprintf('V%03.0f',v);shortRoot=local_short_root(root,o.ShortFileGenRoot,tag);if logical(o.RequireDDriveTemp),local_assert_d_temp();end
    cacheDir=fullfile(shortRoot,'cache');codegenDir=fullfile(shortRoot,'codegen');icDir=fullfile(shortRoot,'ic');
    for p={cacheDir,codegenDir,icDir},if ~isfolder(p{1}),mkdir(p{1});end,end
    try,Simulink.fileGenControl('set','CacheFolder',cacheDir,'CodeGenFolder',codegenDir,'createDir',true);catch ME,fprintf('[UR-MPC] fileGen warning V=%.1f: %s\n',v,ME.message);end
    B=load(bank,'ur_models','ur_meta');pitch0=local_interp_pitch(B.ur_models,double(B.ur_meta.speed_nodes_mps(:)),v,1);
    icPath=fullfile(icDir,sprintf('reset_H%.0f_V%.0f.xml',double(o.TargetAltitudeM),v));
    airdropx_physics_mpc_make_ic('ProjectRoot',root,'AircraftName',o.AircraftName,'OutputFile',icPath,'AirspeedMps',v,'AltitudeM',o.TargetAltitudeM,'PitchDeg',pitch0,'FlightPathDeg',0,'HeadingDeg',0);
    modelName=sprintf('airdropx_urmpc_v20_%s',tag);speedOut=fullfile(outRoot,tag);if isfolder(speedOut),rmdir(speedOut,'s');end;mkdir(speedOut);
    fprintf('[UR-MPC v2.0] START V=%.1f H=%.1f real cfg0->cfg4 drops\n',v,o.TargetAltitudeM);
    R=airdropx_urmpc_run('ProjectRoot',root,'BankMat',bank,'OutputRoot',speedOut,'CaseId',sprintf('urmpc_v20_%s',tag),'ModelName',modelName, ...
        'AircraftName',o.AircraftName,'IcName',icPath,'StopTimeS',o.StopTimeS,'AfterDropTime',o.AfterDropTime,'FixedConfigId',NaN,'FixedDropTotal',4, ...
        'FixedDropStartS',o.DropStartS,'FixedDropIntervalS',o.DropIntervalS,'InitialAltitudeM',o.TargetAltitudeM,'InitialAirspeedMps',v, ...
        'TargetAltitudeM',o.TargetAltitudeM,'TargetAirspeedMps',v);
    T=local_evaluate_speed(R,v,o);writetable(T,fullfile(speedOut,'fixed_stability_segments.csv'));
    info=struct('speed_mps',v,'output_root',string(speedOut),'timeseries_csv',R.timeseries_csv,'trace_csv',R.urmpc_controller_trace_csv);
    fprintf('[UR-MPC v2.0] DONE V=%.1f: %d/5 formal PASS\n',v,sum(T.formal_pass));
catch ME
    E=struct('identifier',string(ME.identifier),'message',string(ME.message));fprintf(2,'[UR-MPC v2.0] ERROR V=%.1f: %s | %s\n',v,ME.identifier,ME.message);T=local_error_rows(v,E);
end
end

function T=local_evaluate_speed(R,v,o)
C=R.urmpc_controller_trace;P=R.timeseries;if isempty(C),error('AirdropX:URMPC:NoTrace','Controller trace is empty at V=%.1f.',v);end
qActual=local_interp_col(P,'q_dps',double(C.time_s),double(C.q_est_dps));rows=cell(5,1);
for cfg=0:4
    cfgMask=round(double(C.cfg_id))==cfg;if ~any(cfgMask),rows{cfg+1}=local_metric_row(v,cfg,NaN,NaN,0,false,false,"missing_cfg_segment",NaN(1,27));continue;end
    tc=double(C.time_s(cfgMask));t0=min(tc);t1=max(tc);evalStart=t0+double(o.SettleAfterCfgS);evalEnd=t1-double(o.EndMarginS);m=cfgMask & double(C.time_s)>=evalStart & double(C.time_s)<=evalEnd;
    if sum(m)<10||evalEnd-evalStart<double(o.MinEvalDurationS),rows{cfg+1}=local_metric_row(v,cfg,t0,t1,sum(m),false,false,"insufficient_steady_window",NaN(1,27));continue;end
    tt=double(C.time_s(m));h=double(C.actual_h_m(m));va=double(C.actual_v_mps(m));vz=double(C.actual_vz_mps(m));pitch=double(C.pitch_deg(m));q=qActual(m);hErr=h-o.TargetAltitudeM;vErr=va-v;
    finite=all(isfinite([tt,h,va,vz,pitch,q]),2);finiteFrac=mean(finite);if sum(finite)<10,rows{cfg+1}=local_metric_row(v,cfg,t0,t1,sum(m),false,false,"nonfinite_segment",NaN(1,27));continue;end
    tt=tt(finite);h=h(finite);va=va(finite);vz=vz(finite);pitch=pitch(finite);q=q(finite);hErr=hErr(finite);vErr=vErr(finite);
    hR=sqrt(mean(hErr.^2));hM=max(abs(hErr));vR=sqrt(mean(vErr.^2));vM=max(abs(vErr));vzR=sqrt(mean(vz.^2));vzM=max(abs(vz));qR=sqrt(mean(q.^2));qM=max(abs(q));pStd=std(pitch);pMax=max(abs(pitch));hSl=local_slope(tt,h);vSl=local_slope(tt,va);pSl=local_slope(tt,pitch);
    elev=double(C.physical_elevator_cmd(m));thr=double(C.physical_throttle_cmd(m));elev=elev(finite);thr=thr(finite);eSat=mean(abs(elev)>=o.ElevatorSaturationAbs);tSat=mean(thr<=o.ThrottleSaturationLow|thr>=o.ThrottleSaturationHigh);
    wholeIdx=find(cfgMask);fc=double(C.mpc_fail_count(wholeIdx));failInc=max(fc)-min(fc);qCtl=double(C.q_est_dps(m));qCtl=qCtl(finite);qEst=sqrt(mean((qCtl-q).^2));settle=local_settle_time(C,qActual,cfg,t0,t1,v,o);
    its=double(C.mpc_last_iterations(m));its=its(finite&isfinite(its));meanIts=NaN;if ~isempty(its),meanIts=mean(its);end
    slack=double(C.mpc_last_slack(m));slack=slack(finite&isfinite(slack));maxSlack=NaN;if ~isempty(slack),maxSlack=max(slack);end
    dE=double(C.physical_elevator_cmd(m))-double(C.nominal_physical_elevator(m));dT=double(C.physical_throttle_cmd(m))-double(C.nominal_throttle(m));dE=dE(finite);dT=dT(finite);maxDE=max(abs(dE));maxDT=max(abs(dT));
    hard=finiteFrac>=0.999&&pMax<o.HardMaxPitchDeg&&qM<o.HardMaxQDps&&vzM<o.HardMaxVzMps&&eSat<o.HardMaxSatFraction&&tSat<o.HardMaxSatFraction;
    formal=hard&&hR<=o.MaxHRmsM&&hM<=o.MaxHAbsM&&vR<=o.MaxVaRmsMps&&vM<=o.MaxVaAbsMps&&vzR<=o.MaxVzRmsMps&&qR<=o.MaxQRmsDps&&pStd<=o.MaxPitchStdDeg&&abs(hSl)<=o.MaxHSlopeMps&&abs(vSl)<=o.MaxVaSlopeMps2&&abs(pSl)<=o.MaxPitchSlopeDps&&eSat<=o.MaxSatFraction&&tSat<=o.MaxSatFraction&&failInc==0;
    reason="PASS";if ~formal,reason=local_failure_reason(hR,hM,vR,vM,vzR,qR,pStd,hSl,vSl,pSl,eSat,tSat,failInc,o);end
    vals=[finiteFrac,hR,hM,vR,vM,vzR,vzM,qR,qM,pStd,pMax,hSl,vSl,pSl,eSat,tSat,failInc,qEst,settle,median(hErr),median(vErr),median(vz),median(q),meanIts,maxSlack,maxDE,maxDT];
    rows{cfg+1}=local_metric_row(v,cfg,t0,t1,numel(tt),hard,formal,reason,vals);
end
T=vertcat(rows{:});
end
function row=local_metric_row(v,cfg,t0,t1,n,hard,formal,reason,x)
names={'finite_fraction','h_rms_m','h_max_abs_m','va_rms_mps','va_max_abs_mps','vz_rms_mps','vz_max_abs_mps','q_rms_dps','q_max_abs_dps','pitch_std_deg','pitch_max_abs_deg','h_slope_mps','va_slope_mps2','pitch_slope_dps','elevator_sat_fraction','throttle_sat_fraction','mpc_fail_increment','q_est_rmse_dps','settling_time_s','tail_h_error_m','tail_va_error_mps','tail_vz_mps','tail_q_dps','mean_qp_iterations','max_slack','max_elevator_deviation','max_throttle_deviation'};
S=struct('speed_mps',v,'cfg_id',cfg,'cfg_start_s',t0,'cfg_end_s',t1,'samples',n,'hard_pass',logical(hard),'formal_pass',logical(formal),'reason',string(reason));for k=1:numel(names),S.(names{k})=x(k);end;row=struct2table(S);
end
function T=local_error_rows(v,ME),T=table();for cfg=0:4,T=[T;local_metric_row(v,cfg,NaN,NaN,0,false,false,"run_error:"+string(ME.identifier),NaN(1,27))];end,end %#ok<AGROW>
function q=local_interp_col(P,name,t,fallback),q=fallback;try,if ~ismember(name,P.Properties.VariableNames),return;end,tp=double(P.time_s);yp=double(P.(name));m=isfinite(tp)&isfinite(yp);tp=tp(m);yp=yp(m);if numel(tp)<2,return;end,[tp,ia]=unique(tp,'stable');yp=yp(ia);q=interp1(tp,yp,t,'linear','extrap');catch,q=fallback;end,end
function s=local_slope(t,y),s=NaN;try,t=double(t(:));y=double(y(:));m=isfinite(t)&isfinite(y);if sum(m)<2,return;end,t=t(m);y=y(m);t=t-t(1);p=polyfit(t,y,1);s=p(1);catch,end,end
function st=local_settle_time(C,qActual,cfg,t0,t1,v,o),st=NaN;try,m=round(double(C.cfg_id))==cfg&double(C.time_s)>=t0&double(C.time_s)<=t1;t=double(C.time_s(m));if numel(t)<5,return;end,hErr=double(C.actual_h_m(m))-o.TargetAltitudeM;vErr=double(C.actual_v_mps(m))-v;vz=double(C.actual_vz_mps(m));q=qActual(m);good=abs(hErr)<=o.SettleHBandM&abs(vErr)<=o.SettleVaBandMps&abs(vz)<=o.SettleVzBandMps&abs(q)<=o.SettleQBandDps;dt=median(diff(t),'omitnan');if ~isfinite(dt)||dt<=0,dt=0.1;end;n=max(1,ceil(o.SettleHoldS/dt));z=conv(double(good),ones(n,1),'valid');k=find(z>=n-1e-9,1,'first');if ~isempty(k),st=t(k)-t0;end,catch,end,end
function r=local_failure_reason(hR,hM,vR,vM,vzR,qR,pStd,hSl,vSl,pSl,eSat,tSat,failInc,o),a=strings(0,1);if hR>o.MaxHRmsM||hM>o.MaxHAbsM,a(end+1)="H";end;if vR>o.MaxVaRmsMps||vM>o.MaxVaAbsMps,a(end+1)="Va";end;if vzR>o.MaxVzRmsMps,a(end+1)="vz";end;if qR>o.MaxQRmsDps,a(end+1)="q";end;if pStd>o.MaxPitchStdDeg,a(end+1)="pitchStd";end;if abs(hSl)>o.MaxHSlopeMps,a(end+1)="hSlope";end;if abs(vSl)>o.MaxVaSlopeMps2,a(end+1)="VaSlope";end;if abs(pSl)>o.MaxPitchSlopeDps,a(end+1)="pitchSlope";end;if eSat>o.MaxSatFraction,a(end+1)="elevSat";end;if tSat>o.MaxSatFraction,a(end+1)="thrSat";end;if failInc>0,a(end+1)="mpcFail";end;if isempty(a),r="hard_safety";else,r=strjoin(a,"+");end,end
function local_validate_bank(path,speeds),S=load(path,'ur_mpc','ur_models','ur_meta');if ~isfield(S,'ur_mpc')||~isfield(S,'ur_models')||~isfield(S,'ur_meta'),error('AirdropX:URMPC:IncompleteBank','UR-MPC bank is incomplete.');end;sn=double(S.ur_meta.speed_nodes_mps(:));for v=double(speeds(:)).',if v<min(sn)-1e-9||v>max(sn)+1e-9,error('AirdropX:URMPC:SpeedOutsideEnvelope','Requested speed %.3f is outside certified %.1f..%.1f m/s envelope.',v,min(sn),max(sn));end,end,end
function p=local_interp_pitch(M,speeds,v,cfg),[speeds,ord]=sort(double(speeds(:)));M=M(ord,:);if v<=speeds(1),p=double(M(1,cfg).x_nominal(3));elseif v>=speeds(end),p=double(M(end,cfg).x_nominal(3));else,i1=find(speeds>=v,1,'first');i0=i1-1;w=(v-speeds(i0))/(speeds(i1)-speeds(i0));p=(1-w)*double(M(i0,cfg).x_nominal(3))+w*double(M(i1,cfg).x_nominal(3));end,end
function local_prepare_pool(n,root,jobRoot),jobRoot=local_resolve(root,jobRoot);if ~isfolder(jobRoot),mkdir(jobRoot);end;p=gcp('nocreate');if ~isempty(p),try,delete(p);catch,end,end;c=parcluster('Processes');c.JobStorageLocation=jobRoot;fprintf('[UR-MPC v2.0] JobStorage=%s\n',c.JobStorageLocation);parpool(c,n);end
function local_assert_d_temp(),td=char(tempdir);if ~startsWith(lower(strrep(td,'/','\')),'d:\'),error('AirdropX:URMPC:WorkerTempStillOnC','Worker tempdir is not D: %s',td);end,end
function p=local_short_root(root,x,tag),p=char(string(x));if isempty(p),p=fullfile(root,'matlab','results','mpc_physics_v1','urmpc_filegen',tag);else,p=fullfile(p,tag);end;if ~isfolder(p),mkdir(p);end,end
function p=local_resolve(root,x),p=char(string(x));if isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),p=fullfile(root,p);end,end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function o=local_options(varargin)
o.ProjectRoot="";o.JobStorageRoot="D:\MATLAB_TEMP\AirdropX_urmpc_v20\jobs";o.RequireDDriveTemp=true;o.BankMat="matlab/results/mpc_physics_v1/unified_robust_mpc_v2/airdropx_unified_robust_mpc_bank.mat";o.OutputRoot="matlab/results/mpc_physics_v1/fixed_stability_urmpc_v20";o.ShortFileGenRoot="D:\AXC\urmpc_v20";o.AircraftName="MQ9_Reaper";
o.SpeedsMps=50;o.TargetAltitudeM=200;o.StopTimeS=255;o.DropStartS=50;o.DropIntervalS=50;o.AfterDropTime=10;o.SettleAfterCfgS=15;o.EndMarginS=5;o.MinEvalDurationS=15;o.ParallelWorkers=1;
o.MaxHRmsM=1.5;o.MaxHAbsM=3.0;o.MaxVaRmsMps=0.75;o.MaxVaAbsMps=1.5;o.MaxVzRmsMps=0.25;o.MaxQRmsDps=0.25;o.MaxPitchStdDeg=0.60;o.MaxHSlopeMps=0.10;o.MaxVaSlopeMps2=0.05;o.MaxPitchSlopeDps=0.05;o.MaxSatFraction=0.01;
o.ElevatorSaturationAbs=0.94;o.ThrottleSaturationLow=0.01;o.ThrottleSaturationHigh=0.99;o.HardMaxPitchDeg=25;o.HardMaxQDps=10;o.HardMaxVzMps=5;o.HardMaxSatFraction=0.10;o.SettleHBandM=2.0;o.SettleVaBandMps=1.0;o.SettleVzBandMps=0.5;o.SettleQBandDps=0.5;o.SettleHoldS=5;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(o,n),error('Unknown option: %s',n);end,o.(n)=varargin{i+1};end
end
