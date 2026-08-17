function result=airdropx_v32_dynamic_mission_validation(varargin)
%AIRDROPX_V32_DYNAMIC_MISSION_VALIDATION Dynamic H/V + 4 payload drops.
opts=local_options(varargin{:});P=double(opts.Profile);if isempty(P),P=[0 200 50;80 150 48;160 100 55;250 50 45;340 20 50;430 120 55;530 200 50];end
stop=max(opts.StopTimeS,P(end,1)+60);
R=airdropx_v32_run_closed_loop('ProjectRoot',opts.ProjectRoot,'BankMat',opts.BankMat,'OutputRoot',opts.OutputRoot,'CaseId','v32_dynamic_four_drop',...
    'StopTimeS',stop,'FixedConfigId',NaN,'FixedDropTotal',4,'FixedDropStartS',opts.DropStartS,'FixedDropIntervalS',opts.DropIntervalS,...
    'InitialAltitudeM',P(1,2),'InitialAirspeedMps',P(1,3),'TargetAltitudeM',P(1,2),'TargetAirspeedMps',P(1,3),...
    'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg,'HiddenElevatorTrim',opts.HiddenElevatorTrim,'InnerReferenceEnabled',false,'DynamicReferenceProfile',P,...
    'HeightKh',opts.HeightKh,'HeightKi',opts.HeightKi,'HeightKaw',opts.HeightKaw,'HeightVzMaxMps',opts.HeightVzMaxMps,'HeightVzSlewMps2',opts.HeightVzSlewMps2,'HeightBiasMaxMps',opts.HeightBiasMaxMps,...
    'SpeedAccelMps2',opts.SpeedAccelMps2,'SpeedDecelMps2',opts.SpeedDecelMps2);
S=local_score(R.timeseries,R.v32_controller_trace,P,opts);
if ~isfolder(opts.OutputRoot),mkdir(opts.OutputRoot);end
writetable(S,fullfile(opts.OutputRoot,'dynamic_mission_summary.csv'));
writetable(array2table(P,'VariableNames',{'time_s','H_ref_m','V_ref_mps'}),fullfile(opts.OutputRoot,'dynamic_mission_profile.csv'));
if logical(S.pass(1)),fid=fopen(fullfile(opts.OutputRoot,'MISSION_PASS.txt'),'w');else,fid=fopen(fullfile(opts.OutputRoot,'MISSION_FAIL.txt'),'w');end
if fid>=0,fprintf(fid,'pass=%d\ngate_ratio=%.6f\n',logical(S.pass(1)),double(S.gate_ratio(1)));fclose(fid);end
result=struct('pass',logical(S.pass(1)),'gate_ratio',double(S.gate_ratio(1)),'summary',S,'run',R);
end

function S=local_score(T,X,P,o)
if isempty(T)||isempty(X),S=local_fail(99,'missing_trace');return;end
t=double(X.time_s);h=double(X.actual_h_m);V=double(X.actual_v_mps);vz=double(X.actual_vz_mps);q=double(X.q_dps);pitch=double(X.pitch_deg);
hR=[];vR=[];hTail=[];vTail=[];settleH=[];settleV=[];
for k=1:size(P,1)
 t0=P(k,1);if k<size(P,1),t1=P(k+1,1);else,t1=max(t);end
 tail=t>=max(t0,t1-o.TailWindowS)&t<=t1&isfinite(t);if nnz(tail)<5,continue;end
 hR(end+1)=local_rms(h(tail)-P(k,2));vR(end+1)=local_rms(V(tail)-P(k,3)); %#ok<AGROW>
 hTail(end+1)=abs(median(h(tail)-P(k,2),'omitnan'));vTail(end+1)=abs(median(V(tail)-P(k,3),'omitnan')); %#ok<AGROW>
 settleH(end+1)=local_settle(t,h-P(k,2),t0,t1,o.HSettleBand,o.SettleHoldS); %#ok<AGROW>
 settleV(end+1)=local_settle(t,V-P(k,3),t0,t1,o.VSettleBand,o.SettleHoldS); %#ok<AGROW>
end
if isempty(hR),S=local_fail(99,'no_segments');return;end
maxHR=max(hR);maxVR=max(vR);maxHT=max(hTail);maxVT=max(vTail);maxSH=max(settleH);maxSV=max(settleV);
maxVz=max(abs(vz),[],'omitnan');maxQ=max(abs(q),[],'omitnan');p0=median(pitch(t<min(10,max(t))),'omitnan');maxPitch=max(abs(pitch-p0),[],'omitnan');minH=min(h,[],'omitnan');
drops=0;if ismember('drop_count',string(T.Properties.VariableNames)),drops=max(double(T.drop_count),[],'omitnan');end
hard=~isfinite(minH)||minH<o.HardFloorM||maxVz>o.HardMaxVz||maxQ>o.HardMaxQ||maxPitch>o.HardPitchDev||drops<4;
gate=max([maxHR/o.PassHRms,maxVR/o.PassVRms,maxHT/o.PassTailH,maxVT/o.PassTailV,maxSH/o.PassHSettleS,maxSV/o.PassVSettleS,maxVz/o.PassMaxVz,maxQ/o.PassMaxQ,4/max(drops,1)]);
pass=~hard&&gate<=1;
S=table(logical(pass),gate,drops,maxHR,maxVR,maxHT,maxVT,maxSH,maxSV,maxVz,maxQ,maxPitch,minH,string('ok'),...
 'VariableNames',{'pass','gate_ratio','drop_count','max_tail_H_rms_m','max_tail_V_rms_mps','max_tail_H_error_m','max_tail_V_error_mps','max_H_settle_s','max_V_settle_s','max_abs_vz_mps','max_abs_q_dps','max_pitch_dev_deg','min_altitude_m','status'});
end
function S=local_fail(g,msg),S=table(false,g,0,Inf,Inf,Inf,Inf,Inf,Inf,Inf,Inf,Inf,-Inf,string(msg),'VariableNames',{'pass','gate_ratio','drop_count','max_tail_H_rms_m','max_tail_V_rms_mps','max_tail_H_error_m','max_tail_V_error_mps','max_H_settle_s','max_V_settle_s','max_abs_vz_mps','max_abs_q_dps','max_pitch_dev_deg','min_altitude_m','status'});end
function s=local_settle(t,e,t0,t1,b,hold),s=Inf;idx=find(t>=t0&t<=t1&isfinite(t));for ii=idx(:).',m=t>=t(ii)&t<=min(t1,t(ii)+hold);if nnz(m)>=3&&max(abs(e(m)),[],'omitnan')<=b,s=t(ii)-t0;return;end,end,end
function r=local_rms(x),x=x(isfinite(x));if isempty(x),r=Inf;else,r=sqrt(mean(x.^2));end,end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.BankMat="";opts.OutputRoot="";opts.Profile=[];opts.StopTimeS=0;opts.DropStartS=110;opts.DropIntervalS=100;opts.ReferenceMassKg=3423;opts.CargoMassKg=300;opts.HiddenElevatorTrim=0;
opts.HeightKh=0.12;opts.HeightKi=0.004;opts.HeightKaw=0.30;opts.HeightVzMaxMps=2;opts.HeightVzSlewMps2=0.60;opts.HeightBiasMaxMps=1.5;opts.SpeedAccelMps2=1.0;opts.SpeedDecelMps2=1.2;
opts.TailWindowS=15;opts.HSettleBand=1.5;opts.VSettleBand=0.7;opts.SettleHoldS=4;opts.PassHRms=1.5;opts.PassVRms=0.7;opts.PassTailH=1.5;opts.PassTailV=0.7;opts.PassHSettleS=80;opts.PassVSettleS=30;opts.PassMaxVz=3.5;opts.PassMaxQ=3.5;opts.HardFloorM=5;opts.HardMaxVz=8;opts.HardMaxQ=12;opts.HardPitchDev=45;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
