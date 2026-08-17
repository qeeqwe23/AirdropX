function result=airdropx_v32_inner_validation(varargin)
%AIRDROPX_V32_INNER_VALIDATION Direct Va/vz command certification with height loop bypassed.
opts=local_options(varargin{:});
P=double(opts.Profile);if isempty(P),P=[0 50 0;20 45 -0.8;50 55 0.8;80 50 -2.0;115 50 2.0;150 50 0];end
prepShift=0;if opts.ConfigId>0,prepShift=opts.PrepSettleS;P(:,1)=P(:,1)+prepShift;end
stop=max(opts.StopTimeS,P(end,1)+15);
R=airdropx_v32_run_closed_loop('ProjectRoot',opts.ProjectRoot,'BankMat',opts.BankMat,'OutputRoot',opts.OutputRoot,'CaseId',opts.CaseId,...
    'StopTimeS',stop,'FixedConfigId',NaN,'FixedDropTotal',opts.ConfigId,'FixedDropStartS',opts.PrepDropStartS,'FixedDropIntervalS',opts.PrepDropIntervalS,'InitialAltitudeM',opts.InitialAltitudeM,'InitialAirspeedMps',P(1,2),...
    'TargetAltitudeM',opts.InitialAltitudeM,'TargetAirspeedMps',P(1,2),'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg,...
    'HiddenElevatorTrim',opts.HiddenElevatorTrim,'InnerReferenceEnabled',true,'InnerReferenceProfile',P,'DynamicReferenceProfile',[0 opts.InitialAltitudeM P(1,2)]);
S=local_score(R.v32_controller_trace,P,opts);
if ~isfolder(opts.OutputRoot),mkdir(opts.OutputRoot);end
writetable(S,fullfile(opts.OutputRoot,'inner_tracking_summary.csv'));
writetable(array2table(P,'VariableNames',{'time_s','V_ref_mps','vz_ref_mps'}),fullfile(opts.OutputRoot,'inner_reference_profile.csv'));
result=struct('pass',logical(S.pass(1)),'gate_ratio',double(S.gate_ratio(1)),'summary',S,'run',R);
end

function S=local_score(T,P,o)
if isempty(T),S=local_fail(99,'missing_trace');return;end
t=double(T.time_s);V=double(T.actual_v_mps);vz=double(T.actual_vz_mps);q=double(T.q_dps);pitch=double(T.pitch_deg);
segV=[];segVz=[];settleV=[];settleVz=[];
for k=1:size(P,1)
    t0=P(k,1);if k<size(P,1),t1=P(k+1,1);else,t1=max(t);end
    tail=t>=max(t0,t1-o.TailWindowS)&t<=t1&isfinite(t);
    if nnz(tail)<5,continue;end
    segV(end+1)=local_rms(V(tail)-P(k,2)); %#ok<AGROW>
    segVz(end+1)=local_rms(vz(tail)-P(k,3)); %#ok<AGROW>
    settleV(end+1)=local_settle(t,V-P(k,2),t0,t1,o.VSettleBand,o.SettleHoldS); %#ok<AGROW>
    settleVz(end+1)=local_settle(t,vz-P(k,3),t0,t1,o.VzSettleBand,o.SettleHoldS); %#ok<AGROW>
end
if isempty(segV),S=local_fail(99,'no_segments');return;end
maxVR=max(segV);maxVzR=max(segVz);maxQ=max(abs(q(isfinite(q))),[],'omitnan');
p0=median(pitch(t<min(10,max(t))),'omitnan');maxPitchDev=max(abs(pitch-p0),[],'omitnan');
minH=min(double(T.actual_h_m),[],'omitnan');maxSetV=max(settleV);maxSetVz=max(settleVz);
gate=max([maxVR/o.PassVRms,maxVzR/o.PassVzRms,maxQ/o.PassMaxQ,maxPitchDev/o.PassPitchDev,maxSetV/o.PassSettleS,maxSetVz/o.PassSettleS]);
hard=~isfinite(gate)||minH<o.HardFloorM||maxQ>o.HardMaxQ||maxPitchDev>o.HardPitchDev;
pass=~hard&&gate<=1;
S=table(logical(pass),gate,maxVR,maxVzR,maxQ,maxPitchDev,maxSetV,maxSetVz,minH,string('ok'),...
    'VariableNames',{'pass','gate_ratio','max_tail_V_rms_mps','max_tail_vz_rms_mps','max_abs_q_dps','max_pitch_dev_deg','max_V_settle_s','max_vz_settle_s','min_altitude_m','status'});
end
function S=local_fail(g,msg)
S=table(false,g,Inf,Inf,Inf,Inf,Inf,Inf,-Inf,string(msg),'VariableNames',{'pass','gate_ratio','max_tail_V_rms_mps','max_tail_vz_rms_mps','max_abs_q_dps','max_pitch_dev_deg','max_V_settle_s','max_vz_settle_s','min_altitude_m','status'});
end
function s=local_settle(t,e,t0,t1,b,hold)
s=Inf;idx=find(t>=t0&t<=t1&isfinite(t));for ii=idx(:).',m=t>=t(ii)&t<=min(t1,t(ii)+hold);if nnz(m)>=3&&max(abs(e(m)),[],'omitnan')<=b,s=t(ii)-t0;return;end,end
end
function r=local_rms(x),x=x(isfinite(x));if isempty(x),r=Inf;else,r=sqrt(mean(x.^2));end,end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.BankMat="";opts.OutputRoot="";opts.CaseId="v32_inner";opts.ConfigId=0;opts.Profile=[];opts.StopTimeS=0;opts.InitialAltitudeM=200;opts.ReferenceMassKg=3423;opts.CargoMassKg=300;opts.HiddenElevatorTrim=0;
opts.PrepDropStartS=2;opts.PrepDropIntervalS=0.6;opts.PrepSettleS=35;opts.TailWindowS=7;opts.VSettleBand=0.35;opts.VzSettleBand=0.10;opts.SettleHoldS=2;opts.PassVRms=0.40;opts.PassVzRms=0.18;opts.PassMaxQ=1.5;opts.PassPitchDev=10;opts.PassSettleS=15;opts.HardFloorM=20;opts.HardMaxQ=8;opts.HardPitchDev=35;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
