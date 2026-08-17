function result = airdropx_physics_mpc_run(varargin)
%AIRDROPX_PHYSICS_MPC_RUN Run the fixed-rule Physics MPC controller.
opts=local_options(varargin{:});root=local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));
bank=local_resolve(root,opts.BankMat);if ~isfile(bank),error('AirdropX:PhysicsMPC:MissingBank','Missing bank: %s',bank);end
S=load(bank,'v32_nodes','speed_nodes','physics_mpc_meta');
if ~isfield(S,'v32_nodes'),error('AirdropX:PhysicsMPC:BadBank','Scheduled physics nodes missing.');end
airdropx_physics_mpc_setup_model('ProjectRoot',root,'ModelName',opts.ModelName);
hidden=double(opts.HiddenElevatorTrim);
if ~isfinite(hidden),hidden=local_hidden_at_speed(S.v32_nodes,double(opts.InitialAirspeedMps));end
if ~isfinite(hidden),error('AirdropX:PhysicsMPC:HiddenTrimUnknown','Per-speed JSBSim hidden elevator offset is unavailable.');end
out=local_resolve(root,opts.OutputRoot);if ~isfolder(out),mkdir(out);end
result=airdropx_v32_run_closed_loop('ProjectRoot',root,'Model',opts.ModelName,'BankMat',bank,'OutputRoot',out,'CaseId',opts.CaseId, ...
    'StopTimeS',opts.StopTimeS,'AfterDropTime',opts.AfterDropTime,'FixedConfigId',opts.FixedConfigId,'FixedDropTotal',opts.FixedDropTotal, ...
    'FixedDropStartS',opts.FixedDropStartS,'FixedDropIntervalS',opts.FixedDropIntervalS, ...
    'InitialAltitudeM',opts.InitialAltitudeM,'InitialAirspeedMps',opts.InitialAirspeedMps,'InitialPitchDeg',NaN, ...
    'TargetAltitudeM',opts.TargetAltitudeM,'TargetAirspeedMps',opts.TargetAirspeedMps, ...
    'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg,'HiddenElevatorTrim',hidden,'MpcEnableTimeS',opts.MpcEnableTimeS,'AircraftName',opts.AircraftName,'IcName',opts.IcName, ...
    'DynamicReferenceProfile',double(opts.DynamicReferenceProfile),'InnerReferenceEnabled',false, ...
    'HeightKh',opts.HeightKh,'HeightKi',opts.HeightKi,'HeightKaw',opts.HeightKaw,'HeightVzMaxMps',opts.HeightVzMaxMps, ...
    'HeightVzSlewMps2',opts.HeightVzSlewMps2,'HeightBiasMaxMps',opts.HeightBiasMaxMps, ...
    'SpeedAccelMps2',opts.SpeedAccelMps2,'SpeedDecelMps2',opts.SpeedDecelMps2, ...
    'ElevatorStepLimit',opts.ElevatorStepLimit,'ThrottleStepLimit',opts.ThrottleStepLimit);
end

function h=local_hidden_at_speed(nodes,v)
speeds=double([nodes.speed_mps]);[speeds,ord]=sort(speeds);nodes=nodes(ord);
if numel(nodes)==1,h=double(nodes(1).mpc_meta.hidden_elevator_offset);return;end
if v<=speeds(1),h=double(nodes(1).mpc_meta.hidden_elevator_offset);return;end
if v>=speeds(end),h=double(nodes(end).mpc_meta.hidden_elevator_offset);return;end
i1=find(speeds>=v,1,'first');i0=i1-1;w=(v-speeds(i0))/(speeds(i1)-speeds(i0));
h=(1-w)*double(nodes(i0).mpc_meta.hidden_elevator_offset)+w*double(nodes(i1).mpc_meta.hidden_elevator_offset);
end
function p=local_resolve(root,x),p=char(string(x));if isempty(p),p=fullfile(root,'matlab','results','mpc_physics_v1','run');elseif isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),p=fullfile(root,p);end,end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.BankMat="matlab/results/mpc_physics_v1/airdropx_physics_mpc_bank.mat";opts.OutputRoot="matlab/results/mpc_physics_v1/run";opts.CaseId="physics_mpc";opts.ModelName="airdropx_physics_mpc_closed_loop";opts.AircraftName="MQ9_Reaper";opts.IcName="";
opts.StopTimeS=180;opts.AfterDropTime=10;opts.FixedConfigId=NaN;opts.FixedDropTotal=0;opts.FixedDropStartS=60;opts.FixedDropIntervalS=15;
opts.InitialAltitudeM=200;opts.InitialAirspeedMps=50;opts.TargetAltitudeM=200;opts.TargetAirspeedMps=50;opts.DynamicReferenceProfile=[];
opts.ReferenceMassKg=3423;opts.CargoMassKg=300;opts.HiddenElevatorTrim=NaN;opts.MpcEnableTimeS=2;
% Fixed engineering governor values; no automatic tuning.
opts.HeightKh=0.08;opts.HeightKi=0.0015;opts.HeightKaw=0.25;opts.HeightVzMaxMps=2.5;opts.HeightVzSlewMps2=0.50;opts.HeightBiasMaxMps=1.2;
opts.SpeedAccelMps2=0.8;opts.SpeedDecelMps2=1.0;opts.ElevatorStepLimit=0.012;opts.ThrottleStepLimit=0.020;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
