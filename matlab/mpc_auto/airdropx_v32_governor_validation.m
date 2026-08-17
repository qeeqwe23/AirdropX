function result=airdropx_v32_governor_validation(varargin)
%AIRDROPX_V32_GOVERNOR_VALIDATION Dynamic height-command certification after inner MPC PASS.
opts=local_options(varargin{:});P=double(opts.Profile);if isempty(P),P=[0 200 50;30 190 50;80 205 50;140 180 50;210 200 50];end
prepShift=0;if opts.ConfigId>0,prepShift=opts.PrepSettleS;P(:,1)=P(:,1)+prepShift;end
stop=max(opts.StopTimeS,P(end,1)+25);
R=airdropx_v32_run_closed_loop('ProjectRoot',opts.ProjectRoot,'BankMat',opts.BankMat,'OutputRoot',opts.OutputRoot,'CaseId',opts.CaseId,...
    'StopTimeS',stop,'FixedConfigId',NaN,'FixedDropTotal',opts.ConfigId,'FixedDropStartS',opts.PrepDropStartS,'FixedDropIntervalS',opts.PrepDropIntervalS,'InitialAltitudeM',P(1,2),'InitialAirspeedMps',P(1,3),...
    'TargetAltitudeM',P(1,2),'TargetAirspeedMps',P(1,3),'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg,'HiddenElevatorTrim',opts.HiddenElevatorTrim,...
    'InnerReferenceEnabled',false,'DynamicReferenceProfile',P,'HeightKh',opts.HeightKh,'HeightKi',opts.HeightKi,'HeightKaw',opts.HeightKaw,...
    'HeightVzMaxMps',opts.HeightVzMaxMps,'HeightVzSlewMps2',opts.HeightVzSlewMps2,'HeightBiasMaxMps',opts.HeightBiasMaxMps,'SpeedAccelMps2',opts.SpeedAccelMps2,'SpeedDecelMps2',opts.SpeedDecelMps2);
S=local_score(R.v32_controller_trace,P,opts);
if ~isfolder(opts.OutputRoot),mkdir(opts.OutputRoot);end
writetable(S,fullfile(opts.OutputRoot,'governor_tracking_summary.csv'));
writetable(array2table(P,'VariableNames',{'time_s','H_ref_m','V_ref_mps'}),fullfile(opts.OutputRoot,'governor_reference_profile.csv'));
result=struct('pass',logical(S.pass(1)),'gate_ratio',double(S.gate_ratio(1)),'summary',S,'run',R);
end
function S=local_score(T,P,o)
if isempty(T),S=local_fail(99,'missing_trace');return;end
t=double(T.time_s);h=double(T.actual_h_m);V=double(T.actual_v_mps);vz=double(T.actual_vz_mps);q=double(T.q_dps);pitch=double(T.pitch_deg);
hR=[];vR=[];settle=[];
for k=1:size(P,1)
 t0=P(k,1);if k<size(P,1),t1=P(k+1,1);else,t1=max(t);end
 tail=t>=max(t0,t1-o.TailWindowS)&t<=t1&isfinite(t);if nnz(tail)<5,continue;end
 hR(end+1)=local_rms(h(tail)-P(k,2));vR(end+1)=local_rms(V(tail)-P(k,3)); %#ok<AGROW>
 settle(end+1)=local_settle(t,h-P(k,2),t0,t1,o.HSettleBand,o.SettleHoldS); %#ok<AGROW>
end
if isempty(hR),S=local_fail(99,'no_segments');return;end
maxHR=max(hR);maxVR=max(vR);maxSet=max(settle);maxVz=max(abs(vz),[],'omitnan');maxQ=max(abs(q),[],'omitnan');
p0=median(pitch(t<min(10,max(t))),'omitnan');maxPitch=max(abs(pitch-p0),[],'omitnan');minH=min(h,[],'omitnan');
gate=max([maxHR/o.PassHRms,maxVR/o.PassVRms,maxSet/o.PassSettleS,maxVz/o.PassMaxVz,maxQ/o.PassMaxQ,maxPitch/o.PassPitchDev]);
hard=~isfinite(gate)||minH<o.HardFloorM||maxVz>o.HardMaxVz||maxQ>o.HardMaxQ||maxPitch>o.HardPitchDev;
pass=~hard&&gate<=1;
S=table(logical(pass),gate,maxHR,maxVR,maxSet,maxVz,maxQ,maxPitch,minH,string('ok'),'VariableNames',{'pass','gate_ratio','max_tail_H_rms_m','max_tail_V_rms_mps','max_H_settle_s','max_abs_vz_mps','max_abs_q_dps','max_pitch_dev_deg','min_altitude_m','status'});
end
function S=local_fail(g,msg),S=table(false,g,Inf,Inf,Inf,Inf,Inf,Inf,-Inf,string(msg),'VariableNames',{'pass','gate_ratio','max_tail_H_rms_m','max_tail_V_rms_mps','max_H_settle_s','max_abs_vz_mps','max_abs_q_dps','max_pitch_dev_deg','min_altitude_m','status'});end
function s=local_settle(t,e,t0,t1,b,hold),s=Inf;idx=find(t>=t0&t<=t1&isfinite(t));for ii=idx(:).',m=t>=t(ii)&t<=min(t1,t(ii)+hold);if nnz(m)>=3&&max(abs(e(m)),[],'omitnan')<=b,s=t(ii)-t0;return;end,end,end
function r=local_rms(x),x=x(isfinite(x));if isempty(x),r=Inf;else,r=sqrt(mean(x.^2));end,end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.BankMat="";opts.OutputRoot="";opts.CaseId="v32_governor";opts.ConfigId=0;opts.Profile=[];opts.StopTimeS=0;opts.ReferenceMassKg=3423;opts.CargoMassKg=300;opts.HiddenElevatorTrim=0;
opts.HeightKh=0.12;opts.HeightKi=0.004;opts.HeightKaw=0.30;opts.HeightVzMaxMps=2.0;opts.HeightVzSlewMps2=0.60;opts.HeightBiasMaxMps=1.5;opts.SpeedAccelMps2=1.0;opts.SpeedDecelMps2=1.2;
opts.PrepDropStartS=2;opts.PrepDropIntervalS=0.6;opts.PrepSettleS=35;opts.TailWindowS=10;opts.HSettleBand=1.0;opts.SettleHoldS=3;opts.PassHRms=1.0;opts.PassVRms=0.50;opts.PassSettleS=45;opts.PassMaxVz=3.0;opts.PassMaxQ=2.5;opts.PassPitchDev=15;opts.HardFloorM=10;opts.HardMaxVz=6;opts.HardMaxQ=10;opts.HardPitchDev=40;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
