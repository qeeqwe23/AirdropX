function result = airdropx_v31_3_dynamic_reference_validation(varargin)
%AIRDROPX_V31_3_DYNAMIC_REFERENCE_VALIDATION Validate in-flight H/V command changes.
%
% This function NEVER trains. It accepts a piecewise-constant command profile
% [time_s, Hcmd_m, Vcmd_mps]. H commands are converted by the v31.2+ height
% governor to vz_ref; V commands pass through the v31.3 acceleration governor.
% If a verified multi-speed scheduler bank exists it may be enabled so the
% inner controller is continuously scheduled with measured airspeed.

opts=local_options(varargin{:}); projectRoot=local_project_root(opts.ProjectRoot);
root=local_resolve(projectRoot,opts.ContextRoot); profile=local_profile(opts.ReferenceProfile);
fixedCfg=double(opts.FixedConfigId);
if logical(opts.EnableDrops), fixedCfg=NaN; end
[cp,bankPath]=local_controller_bank(projectRoot,root,fixedCfg,logical(opts.EnableDrops));
B=load(bankPath,'trim_bank','mpc_meta'); trimBank=B.trim_bank;
hidden=0; if isfield(cp,'hidden_elevator_trim')&&isfinite(double(cp.hidden_elevator_trim)),hidden=double(cp.hidden_elevator_trim);end
[auth,kh,ki,vzLim]=local_vectors(cp);
if logical(opts.EnableDrops)
    if any(~isfinite(auth))||any(~isfinite(kh))||any(~isfinite(ki))||any(~isfinite(vzLim)),error("AirdropX:V31_3:IncompleteControllerVectors","Full dynamic mission needs cfg0..cfg4 parameters.");end
    cfg0=1;
else
    cfg0=min(max(round(fixedCfg),0),4)+1;
    if any(~isfinite([auth(cfg0),kh(cfg0),ki(cfg0),vzLim(cfg0)])),error("AirdropX:V31_3:IncompleteControllerVectors","Selected cfg controller parameters are incomplete.");end
end
schedulerPath=string(opts.SchedulerBankMat);
if strlength(schedulerPath)==0
    candidate=fullfile(fileparts(fileparts(root)),'v31_3_speed_scheduler','v31_3_speed_scheduler_bank.mat');
    if isfile(candidate), schedulerPath=string(candidate); end
else
    schedulerPath=string(local_resolve(projectRoot,schedulerPath));
end
schedulerReady=false;
if isfile(schedulerPath)
    try,S=load(char(schedulerPath),'scheduler');schedulerReady=isfield(S,'scheduler')&&numel(S.scheduler.speed_nodes)>=2;catch,end
end
outRoot=local_resolve(projectRoot,opts.OutputRoot); if ~isfolder(outRoot),mkdir(outRoot);end
stopS=max(double(opts.StopTimeS),profile(end,1)+double(opts.PostProfileSettleS));
initialH=profile(1,2); initialV=profile(1,3);
initialPitch=double(trimBank(cfg0).pitch_deg);
physicalNom=local_physical_nominal(B,cfg0,hidden); initialElev=physicalNom-hidden; initialThrottle=double(trimBank(cfg0).throttle_cmd);
if logical(opts.EnableDrops), dropTotal=double(opts.TotalDropCount); else, dropTotal=0; end
fprintf("\n[V31.3-DYNAMIC] profile points=%d H/V command changes, drops=%d, scheduler=%d\n",size(profile,1),round(dropTotal),schedulerReady&&logical(opts.UseScheduler));
fprintf("[V31.3-DYNAMIC] bank=%s\n",bankPath);
R=airdropx_auto_run_closed_loop( ...
    'ProjectRoot',projectRoot,'MpcBankMat',bankPath,'OutputRoot',fullfile(outRoot,'simulation'), ...
    'CaseId','v31_3_dynamic_reference','StopTimeS',stopS, ...
    'FixedConfigId',fixedCfg,'FixedDropTotal',dropTotal,'FixedDropStartS',double(opts.DropStartS), ...
    'FixedDropIntervalS',double(opts.DropIntervalS), ...
    'InitialAltitudeM',initialH,'InitialAirspeedMps',initialV,'InitialPitchDeg',initialPitch, ...
    'InitialFlightPathDeg',0,'InitialElevatorDelta',initialElev,'InitialThrottleCmd',initialThrottle, ...
    'ReferenceMassKg',double(opts.ReferenceMassKg),'CargoMassKg',double(opts.CargoMassKg), ...
    'HiddenElevatorTrim',hidden,'MpcEnableTimeS',double(opts.MpcEnableTimeS), ...
    'MpcAuthorityScale',auth(cfg0),'MpcAuthorityByConfig',auth, ...
    'HeightToVzGain',kh(cfg0),'HeightToVzGainByConfig',kh, ...
    'HeightIntegralGain',ki(cfg0),'HeightIntegralGainByConfig',ki, ...
    'HeightVzRefLimitMps',vzLim(cfg0),'HeightVzRefLimitByConfig',vzLim, ...
    'BumplessTransitionEnabled',false,'TransitionMoveTransferScale',0,'TransitionIntegralTransferScale',0, ...
    'V31ContinuousControllerStateEnabled',true,'V31HeightGovernorEnabled',true, ...
    'V31HeightVzSlewRateMps2',double(opts.HeightVzSlewRateMps2), ...
    'V31HeightBiasFraction',double(opts.HeightBiasFraction),'V31HeightBiasLeak',double(opts.HeightBiasLeak), ...
    'V31DynamicReferenceEnabled',true,'DynamicReferenceProfile',profile, ...
    'V31SpeedGovernorEnabled',true,'V31SpeedAccelLimitMps2',double(opts.SpeedAccelLimitMps2), ...
    'V31SpeedDecelLimitMps2',double(opts.SpeedDecelLimitMps2), ...
    'V31SchedulerEnabled',logical(opts.UseScheduler)&&schedulerReady,'V31SchedulerBankMat',schedulerPath, ...
    'V31ReferenceInstrumentationEnabled',logical(opts.ReferenceInstrumentationEnabled), ...
    'TrustAltitudeM',1e6,'TrustAirspeedMps',double(opts.TrustAirspeedMps), ...
    'TrustPitchDeg',double(opts.TrustPitchDeg),'TrustVzMps',double(opts.TrustVzMps),'TrustQDps',double(opts.TrustQDps), ...
    'TargetAltitudeM',initialH,'TargetAirspeedMps',initialV,'TargetPitchDeg',initialPitch,'UseTrimPitchReference',1, ...
    'TestPulse1StartS',Inf,'TestPulse1DurationS',0,'TestPulse2StartS',Inf,'TestPulse2DurationS',0);
T=R.timeseries;
[diagSummary,diagProbes]=local_reference_path_diagnostics(R,profile);
if ~isempty(diagSummary),writetable(diagSummary,fullfile(outRoot,'reference_path_diagnostic_summary.csv'));end
if ~isempty(diagProbes),writetable(diagProbes,fullfile(outRoot,'reference_path_transition_probe.csv'));end
[summary,segments]=local_score(T,profile,opts);
writetable(summary,fullfile(outRoot,'dynamic_reference_summary.csv'));
writetable(segments,fullfile(outRoot,'reference_transition_summary.csv'));
writetable(array2table(profile,'VariableNames',{'time_s','requested_altitude_m','requested_airspeed_mps'}),fullfile(outRoot,'reference_profile.csv'));
local_plot(T,profile,fullfile(outRoot,'dynamic_reference_curves.png'));
result=struct('pass',logical(summary.dynamic_pass(1)),'summary',summary,'transition_summary',segments, ...
    'output_root',string(outRoot),'timeseries_csv',string(R.timeseries_csv),'controller_reference_trace_csv',string(R.controller_reference_trace_csv), ...
    'reference_path_diagnostic_summary',diagSummary,'reference_path_transition_probe',diagProbes, ...
    'scheduler_used',logical(opts.UseScheduler)&&schedulerReady);
if result.pass,fid=fopen(fullfile(outRoot,'DYNAMIC_REFERENCE_PASS.txt'),'w');else,fid=fopen(fullfile(outRoot,'DYNAMIC_REFERENCE_FAIL.txt'),'w');end
if fid>=0,fprintf(fid,'dynamic_pass=%d\n',result.pass);fprintf(fid,'timestamp=%s\n',char(string(datetime('now'))));fclose(fid);end
fprintf("[V31.3-DYNAMIC] PASS=%d overall settled H rms=%.3f m, V rms=%.3f m/s\n",result.pass,summary.settled_h_rms_m(1),summary.settled_V_rms_mps(1));
end


function [S,P]=local_reference_path_diagnostics(R,profile)
S=table(); P=table();
try
    path=string(R.controller_reference_trace_csv);
    if strlength(path)==0 || ~isfile(path), return; end
    T=readtable(path);
    if isempty(T), return; end
    t=double(T.time_s);
    hExpected=local_previous_profile(profile(:,1),profile(:,2),t);
    vExpected=local_previous_profile(profile(:,1),profile(:,3),t);
    hDiff=double(T.requested_h_internal_m)-hExpected;
    vDiff=double(T.requested_v_internal_mps)-vExpected;
    hPath=max(abs(hDiff),[],'omitnan'); vPath=max(abs(vDiff),[],'omitnan');
    cmdSeen=sum(abs(hDiff)<=1e-9 & abs(vDiff)<=1e-9);
    signMask=abs(double(T.slew_vz_ref_mps))>=0.05 & abs(double(T.actual_vz_internal_mps))>=0.03;
    if any(signMask), vzSignFrac=mean(sign(double(T.slew_vz_ref_mps(signMask)))==sign(double(T.actual_vz_internal_mps(signMask)))); else, vzSignFrac=NaN; end
    errMask=abs(double(T.height_error_internal_m))>=0.5 & abs(double(T.slew_vz_ref_mps))>=0.02;
    if any(errMask), refSignFrac=mean(sign(double(T.height_error_internal_m(errMask)))==sign(double(T.slew_vz_ref_mps(errMask)))); else, refSignFrac=NaN; end
    S=table(height(T),hPath,vPath,cmdSeen/height(T),refSignFrac,vzSignFrac, ...
        'VariableNames',{'trace_rows','max_internal_H_profile_diff_m','max_internal_V_profile_diff_mps','profile_match_fraction','height_error_to_vzref_sign_fraction','vzref_to_actual_vz_sign_fraction'});
    offsets=[0.2 1.0 5.0];
    rows=[];
    for k=2:size(profile,1)
        for j=1:numel(offsets)
            tq=profile(k,1)+offsets(j); [~,ix]=min(abs(t-tq));
            rows=[rows; k,profile(k,1),offsets(j),t(ix),profile(k,2),profile(k,3), ...
                double(T.requested_h_internal_m(ix)),double(T.requested_v_internal_mps(ix)),double(T.governed_v_internal_mps(ix)), ...
                double(T.actual_h_internal_m(ix)),double(T.actual_v_internal_mps(ix)),double(T.height_error_internal_m(ix)), ...
                double(T.height_bias_mps(ix)),double(T.raw_vz_ref_mps(ix)),double(T.limited_vz_ref_mps(ix)),double(T.slew_vz_ref_mps(ix)), ...
                double(T.actual_vz_internal_mps(ix)),double(T.trust_ok(ix)),double(T.scheduler_enabled(ix))]; %#ok<AGROW>
        end
    end
    if ~isempty(rows)
        P=array2table(rows,'VariableNames',{'command_index','command_time_s','probe_offset_s','sample_time_s','profile_H_m','profile_V_mps', ...
            'internal_requested_H_m','internal_requested_V_mps','internal_governed_V_mps','actual_H_m','actual_V_mps','height_error_m', ...
            'height_bias_mps','raw_vz_ref_mps','limited_vz_ref_mps','slew_vz_ref_mps','actual_vz_mps','trust_ok','scheduler_enabled'});
    end
catch
    S=table(); P=table();
end
end

function y=local_previous_profile(x,v,xq)
y=zeros(size(xq));
for k=1:numel(xq)
    idx=find(x<=xq(k)+1e-12,1,'last'); if isempty(idx),idx=1;end
    y(k)=v(idx);
end
end

function [S,D]=local_score(T,P,opts)
t=double(T.time_s); h=double(T.altitude_m); V=double(T.airspeed_mps); vz=double(T.vz_up_mps); q=double(T.q_dps);
hReq=double(T.requested_altitude_m); vReq=double(T.requested_airspeed_mps); vGov=double(T.governed_airspeed_ref_mps);
D=table(); settledMask=false(size(t));
for k=1:size(P,1)
    t0=P(k,1); if k<size(P,1),t1=P(k+1,1);else,t1=max(t);end
    seg=isfinite(t)&t>=t0&t<=t1; if nnz(seg)<5,continue;end
    if k==1, prevH=P(k,2);prevV=P(k,3);else,prevH=P(k-1,2);prevV=P(k-1,3);end
    dH=P(k,2)-prevH; dV=P(k,3)-prevV;
    hBand=max(double(opts.HeightSettleBandM),double(opts.HeightSettleFraction)*abs(dH));
    vBand=max(double(opts.SpeedSettleBandMps),double(opts.SpeedSettleFraction)*abs(dV));
    hSet=local_settle_time(t,h-P(k,2),t0,t1,hBand,double(opts.SettleHoldS));
    vSet=local_settle_time(t,V-P(k,3),t0,t1,vBand,double(opts.SettleHoldS));
    tail=seg&t>=max(t0,t1-double(opts.SegmentTailWindowS)); if nnz(tail)<3,tail=seg;end
    settledMask=settledMask|tail;
    hFinal=median(h(tail)-P(k,2),'omitnan'); vFinal=median(V(tail)-P(k,3),'omitnan');
    hOver=local_directional_overshoot(h(seg),prevH,P(k,2)); vOver=local_directional_overshoot(V(seg),prevV,P(k,3));
    row=table(k,t0,t1,P(k,2),P(k,3),dH,dV,hSet,vSet,hFinal,vFinal,hOver,vOver, ...
        local_rms(h(tail)-P(k,2)),local_rms(V(tail)-P(k,3)), ...
        'VariableNames',{'segment','command_time_s','segment_end_s','H_command_m','V_command_mps','delta_H_m','delta_V_mps', ...
        'H_settle_s','V_settle_s','tail_H_error_m','tail_V_error_mps','H_overshoot_m','V_overshoot_mps','tail_H_rms_m','tail_V_rms_mps'});
    if isempty(D),D=row;else,D=[D;row];end %#ok<AGROW>
end
if ~any(settledMask),settledMask=isfinite(t);end
hR=local_rms(h(settledMask)-hReq(settledMask)); vR=local_rms(V(settledMask)-vReq(settledMask));
maxVz=max(abs(vz(isfinite(vz))),[],'omitnan'); maxQ=max(abs(q(isfinite(q))),[],'omitnan');
segPass=true;
if ~isempty(D)
    changed=(abs(D.delta_H_m)>1e-9)|(abs(D.delta_V_mps)>1e-9);
    segPass=all(abs(D.tail_H_error_m)<=double(opts.PassTailHeightErrorM)+1e-12) && ...
        all(abs(D.tail_V_error_mps)<=double(opts.PassTailSpeedErrorMps)+1e-12) && ...
        all(~changed | (D.H_settle_s<=double(opts.MaxSettleTimeS) | abs(D.delta_H_m)<=1e-9)) && ...
        all(~changed | (D.V_settle_s<=double(opts.MaxSettleTimeS) | abs(D.delta_V_mps)<=1e-9));
end
pass=segPass&&hR<=double(opts.PassSettledHeightRmsM)&&vR<=double(opts.PassSettledSpeedRmsMps)&& ...
    maxVz<=double(opts.HardMaxVzMps)&&maxQ<=double(opts.HardMaxQDps);
S=table(logical(pass),hR,vR,maxVz,maxQ,height(D), ...
    'VariableNames',{'dynamic_pass','settled_h_rms_m','settled_V_rms_mps','max_abs_vz_mps','max_abs_q_dps','segments_scored'});
end
function s=local_settle_time(t,e,t0,t1,band,hold)
s=Inf; idx=find(isfinite(t)&t>=t0&t<=t1);
for ii=idx(:).'
    m=isfinite(t)&t>=t(ii)&t<=min(t1,t(ii)+hold); if nnz(m)<3,continue;end
    if max(abs(e(m)),[],'omitnan')<=band,s=t(ii)-t0;return;end
end
end
function o=local_directional_overshoot(x,x0,xf)
if abs(xf-x0)<1e-9,o=max(abs(x-xf),[],'omitnan');elseif xf>x0,o=max(0,max(x,[],'omitnan')-xf);else,o=max(0,xf-min(x,[],'omitnan'));end
end
function local_plot(T,P,file)
t=double(T.time_s);f=figure('Visible','off','Color','w','Position',[100 100 1400 850]);tl=tiledlayout(4,1,'Padding','compact');
nexttile;plot(t,T.altitude_m);hold on;stairs(P(:,1),P(:,2),'--','LineWidth',1.2);grid on;ylabel('H m');legend('actual','requested');
nexttile;plot(t,T.airspeed_mps);hold on;stairs(P(:,1),P(:,3),'--');plot(t,T.governed_airspeed_ref_mps,':');grid on;ylabel('Va m/s');legend('actual','requested','governed');
nexttile;plot(t,T.vz_up_mps);grid on;ylabel('vz m/s');
nexttile;plot(t,T.q_dps);grid on;ylabel('q deg/s');xlabel('time s');title(tl,'v31.3 dynamic H/V reference validation');exportgraphics(f,file,'Resolution',170);close(f);
end
function [cp,p]=local_controller_bank(projectRoot,root,fixedCfg,fullMission)
S=load(fullfile(root,'airdropx_200m_cfg_checkpoint.mat'),'checkpoint');cp=S.checkpoint;
if fullMission
    if numel(cp.status)<5||~all(string(cp.status(1:5))=="verified"),error("AirdropX:V31_3:ControllersNotReady","Full dynamic mission requires cfg0..cfg4 VERIFIED.");end;k=5;
else
    k=min(max(round(fixedCfg),0),4)+1;if numel(cp.status)<k||string(cp.status(k))~="verified",error("AirdropX:V31_3:ControllerNotReady","cfg%d is not VERIFIED.",k-1);end
end
p="";if isfield(cp,'best_bank_path')&&numel(cp.best_bank_path)>=k,p=string(cp.best_bank_path(k));end
c=[p;string(fullfile(root,sprintf('cfg%d',k-1),'best_mpc_bank_200m.mat'))];
for x=c.'
    if strlength(x)==0,continue;end;if isfile(x),p=char(x);return;end;x2=fullfile(projectRoot,char(x));if isfile(x2),p=x2;return;end
end
error("AirdropX:V31_3:MissingBank","Could not resolve verified bank for cfg%d.",k-1);
end
function [a,g,i,l]=local_vectors(cp)
a=NaN(5,1);g=NaN(5,1);i=NaN(5,1);l=NaN(5,1);
for k=1:min(5,numel(cp.best_candidate)),c=cp.best_candidate{k};if isempty(c),continue;end;a(k)=local_f(c,'Authority',NaN);g(k)=local_f(c,'HeightToVzGain',NaN);i(k)=local_f(c,'HeightIntegralGain',0);l(k)=local_f(c,'HeightVzLimit',NaN);end
end
function x=local_physical_nominal(B,k,hidden)
x=NaN;try,x=double(B.mpc_meta.physical_elevator_nominals(k));catch,end;if ~isfinite(x),x=hidden+double(B.trim_bank(k).elevator_cmd);end
end
function v=local_f(s,n,d),try,v=double(s.(n));if ~isscalar(v)||~isfinite(v),v=d;end,catch,v=d;end,end
function r=local_rms(x),x=double(x(:));x=x(isfinite(x));if isempty(x),r=NaN;else,r=sqrt(mean(x.^2));end,end
function P=local_profile(x)
if isempty(x),P=[0 200 50;30 190 50;70 190 45;110 180 55;160 200 48;215 200 50];elseif istable(x),P=[double(x{:,1}),double(x{:,2}),double(x{:,3})];else,P=double(x);end
if size(P,2)~=3||isempty(P)||any(~isfinite(P),'all'),error("ReferenceProfile must be finite Nx3 [time,H,V].");end;P=sortrows(P,1);if P(1,1)>0,P=[0 P(1,2:3);P];end
end
function p=local_project_root(x),if strlength(string(x))>0,p=char(string(x));else,t=fileparts(mfilename('fullpath'));p=fileparts(fileparts(t));end,end
function p=local_resolve(root,x),x=char(string(x));if isempty(x),p=root;elseif isfolder(x)||isfile(x)||~isempty(regexp(x,'^[A-Za-z]:[\\/]','once')),p=x;else,p=fullfile(root,x);end,end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.ContextRoot="matlab/results/mpc_auto_v31/reference_contexts/H200p000_V50p000";opts.OutputRoot="matlab/results/mpc_auto_v31/dynamic_reference_validation";
opts.ReferenceProfile=[];opts.FixedConfigId=0;opts.EnableDrops=false;opts.TotalDropCount=4;opts.DropStartS=85;opts.DropIntervalS=35;opts.ReferenceMassKg=3423;opts.CargoMassKg=300;opts.MpcEnableTimeS=2;opts.StopTimeS=0;opts.PostProfileSettleS=30;
opts.HeightVzSlewRateMps2=0.30;opts.HeightBiasFraction=0.70;opts.HeightBiasLeak=1.0;opts.SpeedAccelLimitMps2=0.75;opts.SpeedDecelLimitMps2=1.00;
opts.UseScheduler=true;opts.SchedulerBankMat="";opts.ReferenceInstrumentationEnabled=true;opts.TrustAirspeedMps=8;opts.TrustPitchDeg=6;opts.TrustVzMps=4;opts.TrustQDps=6;
opts.HeightSettleBandM=1.0;opts.SpeedSettleBandMps=0.5;opts.HeightSettleFraction=0.05;opts.SpeedSettleFraction=0.05;opts.SettleHoldS=2;opts.SegmentTailWindowS=8;opts.MaxSettleTimeS=60;
opts.PassTailHeightErrorM=1.5;opts.PassTailSpeedErrorMps=1.0;opts.PassSettledHeightRmsM=1.5;opts.PassSettledSpeedRmsMps=1.0;opts.HardMaxVzMps=5;opts.HardMaxQDps=6;
if mod(numel(varargin),2)~=0,error("Options must be name-value pairs.");end
for k=1:2:numel(varargin),n=string(varargin{k});if ~isfield(opts,n),error("Unknown option: %s",n);end,opts.(n)=varargin{k+1};end
end
