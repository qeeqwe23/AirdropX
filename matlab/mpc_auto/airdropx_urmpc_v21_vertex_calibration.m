function result=airdropx_urmpc_v21_vertex_calibration(varargin)
%AIRDROPX_URMPC_V21_VERTEX_CALIBRATION Collect nonlinear near-trim residual data.
%
% Stage-A only: this function DOES NOT modify the MPC controller or bank.
% It runs the same UR-MPC at every requested speed x cfg operating point,
% reaches cfg by an early fixed-drop sequence, waits for settling, applies a
% small common H/V reference excitation, and records one-step residuals.
%
% The same experiment, trust metric, and acceptance rule are used for every
% cfg. No cfg-specific controller tuning is introduced.

opts=local_options(varargin{:});root=local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));
bank=local_resolve(root,opts.BankMat);if ~isfile(bank),error('AirdropX:URMPC:MissingBank','Missing bank: %s',bank);end
B=load(bank,'ur_models','ur_meta','ur_mpc');
if ~isfield(B,'ur_models')||~isfield(B,'ur_meta')||~isfield(B,'ur_mpc'),error('AirdropX:URMPC:BadBank','Calibration requires ur_models/ur_meta/ur_mpc.');end
allSpeeds=double(B.ur_meta.speed_nodes_mps(:));
speeds=double(opts.SpeedsMps(:));if isempty(speeds),speeds=allSpeeds;end
cfgs=unique(round(double(opts.CfgIds(:).')),'stable');
if any(cfgs<0|cfgs>=size(B.ur_models,2)),error('AirdropX:URMPC:BadCfg','CfgIds must be within 0..%d.',size(B.ur_models,2)-1);end
if any(speeds<min(allSpeeds)-1e-9|speeds>max(allSpeeds)+1e-9),error('AirdropX:URMPC:BadSpeed','Calibration speed outside bank envelope.');end
outRoot=local_resolve(root,opts.OutputRoot);if ~isfolder(outRoot),mkdir(outRoot);end
icRoot=fullfile(outRoot,'_ic');if ~isfolder(icRoot),mkdir(icRoot);end
rows={};ri=0;
for si=1:numel(speeds)
    v=speeds(si);[~,ni]=min(abs(allSpeeds-v));pitch0=double(B.ur_models(ni,1).x_nominal(3));
    icPath=fullfile(icRoot,sprintf('urmpc_v21_V%03d_ic.xml',round(v)));
    airdropx_physics_mpc_make_ic('ProjectRoot',root,'AircraftName',opts.AircraftName,'OutputFile',icPath, ...
        'AirspeedMps',v,'AltitudeM',opts.TargetAltitudeM,'PitchDeg',pitch0,'FlightPathDeg',0,'HeadingDeg',0);
    for ci=1:numel(cfgs)
        cfg=cfgs(ci);finalDrop=local_final_drop_time(cfg,opts.DropStartS,opts.DropIntervalS);
        fitStart=max(opts.MinimumFitStartS,finalDrop+opts.SettleAfterFinalDropS);
        profile=local_profile(fitStart,opts.TargetAltitudeM,v,min(allSpeeds),max(allSpeeds),opts);
        fitEnd=profile(end,1)+opts.PostExcitationSettleS;
        stopTime=fitEnd+opts.EndMarginS;
        tag=sprintf('V%03d_cfg%d',round(v),cfg);caseOut=fullfile(outRoot,tag);
        if isfolder(caseOut),rmdir(caseOut,'s');end;mkdir(caseOut);
        modelName=sprintf('airdropx_urmpc_v21_%s',tag);
        status="run_error";message="";trusted=0;fitSamples=0;finiteFraction=0;actualCfgTail=NaN;
        try
            R=airdropx_urmpc_run('ProjectRoot',root,'BankMat',bank,'OutputRoot',caseOut,'CaseId',"urmpc_v21_"+string(tag),'ModelName',modelName, ...
                'AircraftName',opts.AircraftName,'IcName',icPath,'StopTimeS',stopTime,'AfterDropTime',opts.AfterDropTime, ...
                'FixedConfigId',NaN,'FixedDropTotal',cfg,'FixedDropStartS',opts.DropStartS,'FixedDropIntervalS',opts.DropIntervalS, ...
                'InitialAltitudeM',opts.TargetAltitudeM,'InitialAirspeedMps',v,'TargetAltitudeM',opts.TargetAltitudeM,'TargetAirspeedMps',v, ...
                'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg,'HiddenElevatorTrim',NaN,'DynamicReferenceProfile',profile);
            T=R.urmpc_controller_trace;
            if ~isempty(T)
                m=double(T.time_s)>=fitStart & double(T.time_s)<=fitEnd & round(double(T.cfg_id))==cfg;
                fitSamples=sum(m);if fitSamples>0
                    X=local_state_norm(T(m,:),B.ur_meta);trusted=sum(isfinite(X)&X<=opts.TrustStateNormMax);
                    vals=[double(T.actual_h_m(m)),double(T.actual_v_mps(m)),double(T.actual_vz_mps(m)),double(T.pitch_deg(m)),double(T.q_est_dps(m))];
                    finiteFraction=mean(all(isfinite(vals),2));actualCfgTail=mode(round(double(T.cfg_id(m))));
                end
            end
            if fitSamples>=opts.MinFitSamples && trusted>=opts.MinTrustedSamples && finiteFraction>=opts.MinFiniteFraction
                status="PASS";
            else
                status="INSUFFICIENT_TRUSTED_DATA";
            end
        catch ME
            message=string(ME.identifier)+" | "+string(ME.message);
        end
        ri=ri+1;row=table(v,cfg,finalDrop,fitStart,fitEnd,stopTime,fitSamples,trusted,finiteFraction,actualCfgTail,status,message,string(caseOut), ...
            'VariableNames',{'speed_mps','cfg_id','final_drop_s','fit_start_s','fit_end_s','stop_time_s','fit_samples','trusted_samples','finite_fraction','actual_cfg_mode','status','message','output_dir'});
        rows{ri,1}=row; %#ok<AGROW>
        fprintf('[UR-MPC v2.1A CAL] %s V=%.1f cfg%d fit=%d trusted=%d finite=%.3f\n',char(status),v,cfg,fitSamples,trusted,finiteFraction);
    end
end
M=vertcat(rows{:});manifest=fullfile(outRoot,'urmpc_v21_calibration_manifest.csv');writetable(M,manifest);
result=struct('manifest',M,'manifest_csv',string(manifest),'all_pass',all(M.status=="PASS"),'output_root',string(outRoot));
end

function t=local_final_drop_time(cfg,startS,intervalS)
if cfg<=0,t=0;else,t=double(startS)+(double(cfg)-1)*double(intervalS);end
end
function P=local_profile(t0,h,v,vmin,vmax,o)
% Common bounded excitation; only envelope-edge clipping depends on speed.
dh=double(o.HeightExcitationM);dv=double(o.SpeedExcitationM);
vHi=min(vmax,v+dv);vLo=max(vmin,v-dv);
P=[0 h v; t0 h v; t0+5 h+dh v; t0+10 h v; t0+15 h-dh v; t0+20 h v; ...
   t0+25 h vHi; t0+30 h v; t0+35 h vLo; t0+40 h v];
% Remove duplicate consecutive references caused by speed-edge clipping.
keep=true(size(P,1),1);for k=2:size(P,1),if all(abs(P(k,2:3)-P(k-1,2:3))<1e-12),keep(k)=false;end,end;P=P(keep,:);
end
function n=local_state_norm(T,meta)
sc=[3 1.5 6 .5 .5];try,x=double(meta.output_scales(:).');if numel(x)==5&&all(isfinite(x))&&all(x>0),sc=x;end,catch,end
D=[double(T.actual_h_m)-double(T.requested_h_m),double(T.actual_v_mps)-double(T.nominal_va_mps), ...
   double(T.pitch_deg)-double(T.nominal_pitch_deg),double(T.actual_vz_mps),double(T.q_est_dps)];
n=sqrt(sum((D./sc).^2,2));
end
function p=local_resolve(root,x),p=char(string(x));if isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),p=fullfile(root,p);end,end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function o=local_options(varargin)
o.ProjectRoot="";o.BankMat="matlab/results/mpc_physics_v1/unified_robust_mpc_v2/airdropx_unified_robust_mpc_bank.mat";
o.OutputRoot="matlab/results/mpc_physics_v1/urmpc_v21_residual_calibration";o.AircraftName="MQ9_Reaper";o.SpeedsMps=[];o.CfgIds=0:4;
o.TargetAltitudeM=200;o.ReferenceMassKg=3423;o.CargoMassKg=300;o.DropStartS=5;o.DropIntervalS=1.5;o.SettleAfterFinalDropS=15;o.MinimumFitStartS=15;
o.HeightExcitationM=0.75;o.SpeedExcitationM=0.5;o.PostExcitationSettleS=5;o.EndMarginS=2;o.AfterDropTime=10;
o.TrustStateNormMax=1.0;o.MinFitSamples=200;o.MinTrustedSamples=120;o.MinFiniteFraction=0.995;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(o,n),error('Unknown option: %s',n);end,o.(n)=varargin{i+1};end
end
