function result = airdropx_urmpc_run(varargin)
%AIRDROPX_URMPC_RUN Convenience wrapper for one unified robust/adaptive MPC flight.
opts=local_options(varargin{:});root=local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));
bank=local_resolve(root,opts.BankMat);if ~isfile(bank),error('AirdropX:URMPC:MissingBank','Missing UR-MPC bank: %s',bank);end
S=load(bank,'ur_meta','ur_models');
hidden=double(opts.HiddenElevatorTrim);if ~isfinite(hidden),hidden=local_hidden_at_speed(S,double(opts.InitialAirspeedMps));end
if ~isfinite(hidden),error('AirdropX:URMPC:HiddenTrimUnknown','Hidden elevator offset is unavailable.');end
modelName=char(string(opts.ModelName));airdropx_urmpc_setup_model('ProjectRoot',root,'ModelName',modelName);
out=local_resolve(root,opts.OutputRoot);if ~isfolder(out),mkdir(out);end
result=airdropx_urmpc_run_closed_loop('ProjectRoot',root,'Model',modelName,'BankMat',bank,'OutputRoot',out,'CaseId',opts.CaseId, ...
    'AircraftName',opts.AircraftName,'IcName',opts.IcName,'StopTimeS',opts.StopTimeS,'AfterDropTime',opts.AfterDropTime, ...
    'FixedConfigId',opts.FixedConfigId,'FixedDropTotal',opts.FixedDropTotal,'FixedDropStartS',opts.FixedDropStartS,'FixedDropIntervalS',opts.FixedDropIntervalS, ...
    'InitialAltitudeM',opts.InitialAltitudeM,'InitialAirspeedMps',opts.InitialAirspeedMps,'TargetAltitudeM',opts.TargetAltitudeM,'TargetAirspeedMps',opts.TargetAirspeedMps, ...
    'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg,'HiddenElevatorTrim',hidden,'DynamicReferenceProfile',opts.DynamicReferenceProfile);
end
function h=local_hidden_at_speed(S,v)
if isfield(S,'ur_meta')&&isfield(S.ur_meta,'hidden_offsets_by_speed'),speeds=double(S.ur_meta.speed_nodes_mps(:));hh=double(S.ur_meta.hidden_offsets_by_speed(:));else,speeds=double(S.ur_meta.speed_nodes_mps(:));hh=arrayfun(@(i)double(S.ur_models(i,1).hidden_elevator_offset),(1:size(S.ur_models,1)).');end
[speeds,ord]=sort(speeds);hh=hh(ord);if v<=speeds(1),h=hh(1);elseif v>=speeds(end),h=hh(end);else,i1=find(speeds>=v,1,'first');i0=i1-1;w=(v-speeds(i0))/(speeds(i1)-speeds(i0));h=(1-w)*hh(i0)+w*hh(i1);end
end
function p=local_resolve(root,x),p=char(string(x));if isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),p=fullfile(root,p);end,end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function o=local_options(varargin)
o.ProjectRoot="";o.BankMat="matlab/results/mpc_physics_v1/unified_robust_mpc_v2/airdropx_unified_robust_mpc_bank.mat";o.OutputRoot="matlab/results/mpc_physics_v1/fixed_stability_urmpc_v20/run";o.CaseId="urmpc_v20";o.ModelName="airdropx_urmpc_closed_loop";o.AircraftName="MQ9_Reaper";o.IcName="";
o.StopTimeS=255;o.AfterDropTime=10;o.FixedConfigId=NaN;o.FixedDropTotal=4;o.FixedDropStartS=50;o.FixedDropIntervalS=50;o.InitialAltitudeM=200;o.InitialAirspeedMps=50;o.TargetAltitudeM=200;o.TargetAirspeedMps=50;o.DynamicReferenceProfile=[];o.ReferenceMassKg=3423;o.CargoMassKg=300;o.HiddenElevatorTrim=NaN;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(o,n),error('Unknown option: %s',n);end,o.(n)=varargin{i+1};end
end
