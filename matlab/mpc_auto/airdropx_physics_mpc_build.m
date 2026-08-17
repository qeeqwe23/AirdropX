function result = airdropx_physics_mpc_build(varargin)
%AIRDROPX_PHYSICS_MPC_BUILD Build fixed-rule scheduled MPC from JSBSim physics.
%
% Physics-MPC v1.6 certification-horizon fix:
%   * NO MEX rebuild;
%   * three process workers remain, but parallelism is across SPEED rows;
%   * inside each speed row cfg0->cfg4 is solved sequentially;
%   * continuation passes PHYSICAL elevator, throttle and equilibrium pitch;
%   * each target cfg measures/compensates its own legacy hidden elevator bias;
%   * no BayesOpt, model-order search, random/global learning or online tuning.

opts=local_options(varargin{:});root=local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));
outRoot=local_resolve(root,opts.OutputRoot);if ~isfolder(outRoot),mkdir(outRoot);end
v32Root=local_resolve(root,opts.ExistingV32Root);
speeds=sort(unique(double(opts.SpeedNodesMps(:))));
if isempty(speeds),error('AirdropX:PhysicsMPC:NoSpeedNodes','At least one speed node is required.');end

% Freeze source trim banks before starting workers.
banks=cell(numel(speeds),1);
for ni=1:numel(speeds),banks{ni}=local_load_trim_bank(v32Root,speeds(ni));end

rowResults=cell(numel(speeds),1);rowErrors=cell(numel(speeds),1);
workers=max(1,round(double(opts.ParallelWorkers)));
if workers>1 && numel(speeds)>1
    local_prepare_pool(min(workers,numel(speeds)));
    parfor ni=1:numel(speeds)
        [rowResults{ni},rowErrors{ni}]=local_execute_speed(root,banks{ni},speeds(ni),outRoot,opts);
    end
else
    for ni=1:numel(speeds)
        [rowResults{ni},rowErrors{ni}]=local_execute_speed(root,banks{ni},speeds(ni),outRoot,opts);
    end
end

% Always write all node failures. A failed cfg does not stop later cfg scans;
% later nodes use the most recent successful physical continuation seed.
failRows={};
for ni=1:numel(speeds)
    Erow=rowErrors{ni};
    if isempty(Erow),continue;end
    for cfg=0:4
        if numel(Erow)<cfg+1||isempty(Erow{cfg+1}),continue;end
        E=Erow{cfg+1};
        failRows(end+1,:)={speeds(ni),cfg,string(E.identifier),string(E.message)}; %#ok<AGROW>
    end
end
if isempty(failRows)
    F=table('Size',[0 4],'VariableTypes',{'double','double','string','string'},'VariableNames',{'speed_mps','cfg_id','identifier','message'});
else
    F=cell2table(failRows,'VariableNames',{'speed_mps','cfg_id','identifier','message'});
end
writetable(F,fullfile(outRoot,'physics_mpc_build_failures.csv'));

nodes=repmat(struct('speed_mps',NaN,'controllers',{{}},'trim_bank',[],'plant_bank',{{}},'mpc_meta',struct()),numel(speeds),1);
reportRows=[];
for ni=1:numel(speeds)
    v=speeds(ni);updated=banks{ni};ctrls=cell(5,1);plants=cell(5,1);physicalElev=NaN(5,1);
    Rrow=rowResults{ni};
    if isempty(Rrow),Rrow=cell(5,1);end
    for cfg=0:4
        if numel(Rrow)<cfg+1||isempty(Rrow{cfg+1}),continue;end
        L=Rrow{cfg+1};
        updated(cfg+1)=local_merge_trim(updated(cfg+1),L.trim);
        plants{cfg+1}=L.plant;physicalElev(cfg+1)=L.u_nominal(1);ctrls{cfg+1}=local_make_mpc(L.plant,opts);
        d=L.repeatability;
        reportRows=[reportRows; v cfg L.fit.spectral_radius L.fit.controllability_rank L.fit.regressor_condition L.validation.rmse_10step(:).' ...
            d.va_error_mps d.vz_mps d.q_dps d.h_slope_mps d.va_slope_mps2 d.pitch_deg d.elevator_physical d.throttle_physical]; %#ok<AGROW>
    end
    if any(cellfun(@isempty,plants)),continue;end
    hiddenByCfg=physicalElev(:)-arrayfun(@(x)double(x.elevator_cmd),updated(:));
    hiddenNode=hiddenByCfg(1);
    if ~isfinite(hiddenNode),error('AirdropX:PhysicsMPC:HiddenOffsetMissing','Cannot measure cfg0 JSBSim hidden elevator offset at V=%.1f.',v);end
    meta=struct('version','physics_mpc_v1_6_cert_horizon_nomex','architecture','scheduled_physical_jacobian_inner_mpc', ...
        'model_source','JSBSim deterministic local Jacobian with cfg-continuation physical trim', ...
        'input_coordinate_mode','deviation_physical','physical_elevator_nominals',physicalElev(:), ...
        'hidden_elevator_offset',hiddenNode,'offline_hidden_elevator_offset_by_cfg',hiddenByCfg(:), ...
        'throttle_nominals',arrayfun(@(x)double(x.throttle_cmd),updated(:)), ...
        'elevator_deviation_limit',double(opts.ElevatorDeviationLimit),'throttle_deviation_limit',double(opts.ThrottleDeviationLimit), ...
        'elevator_deviation_rate_limit',double(opts.ElevatorDeviationRateLimit),'throttle_deviation_rate_limit',double(opts.ThrottleDeviationRateLimit));
    nodes(ni)=struct('speed_mps',v,'controllers',{ctrls},'trim_bank',updated(:),'plant_bank',{plants},'mpc_meta',meta);
end

if ~isempty(reportRows)
    names={'speed_mps','cfg_id','spectral_radius','controllability_rank','regressor_condition','rmse10_Va','rmse10_pitch','rmse10_vz','rmse10_q', ...
        'repeat_dVaErr','repeat_dVz','repeat_dQ','repeat_dHSlope','repeat_dVaSlope','repeat_dPitch','repeat_dElevator','repeat_dThrottle'};
    R=array2table(reportRows,'VariableNames',names);writetable(R,fullfile(outRoot,'physics_mpc_model_report.csv'));
else
    R=table();
end

if height(F)>0
    fprintf('\n[PHYS-MPC] BUILD INCOMPLETE: %d node(s) failed. All speed rows were still scanned.\n',height(F));
    disp(F(:,{'speed_mps','cfg_id','identifier'}));
    error('AirdropX:PhysicsMPC:BuildIncomplete','%d node(s) failed. See %s.',height(F),fullfile(outRoot,'physics_mpc_build_failures.csv'));
end

hiddenOffsets=arrayfun(@(x)double(x.mpc_meta.hidden_elevator_offset),nodes(:));hidden=median(hiddenOffsets,'omitnan');
v32_nodes=nodes;speed_nodes=speeds;[~,mid]=min(abs(speeds-median(speeds)));trim_bank=nodes(mid).trim_bank;mpc_meta=nodes(1).mpc_meta; %#ok<NASGU>
physics_mpc_meta=struct('version','physics_mpc_v1_6_cert_horizon_nomex','created_at',datetime('now'),'speed_nodes_mps',speeds, ...
    'reference_altitude_m',opts.ReferenceAltitudeM,'reference_mass_kg',opts.ReferenceMassKg,'cargo_mass_kg',opts.CargoMassKg, ...
    'hidden_elevator_trim',hidden,'hidden_elevator_offsets_by_speed',hiddenOffsets(:), ...
    'runtime_controller','sfun_airdropx_physics_mpc_controller', ...
    'online_learning',false,'bayes_optimization',false,'model_order_search',false,'deterministic_equilibrium_solver',logical(opts.AllowDeterministicRetrim),'options',opts); %#ok<NASGU>
bankPath=fullfile(outRoot,'airdropx_physics_mpc_bank.mat');
save(bankPath,'v32_nodes','speed_nodes','trim_bank','mpc_meta','physics_mpc_meta','-v7.3');
local_write_manifest(outRoot,bankPath,hidden,opts);
result=struct('bank_mat',string(bankPath),'nodes',{nodes},'hidden_elevator_trim',hidden,'report',R,'failures',F,'options',opts);
fprintf('\n[PHYS-MPC] BUILD COMPLETE\n  Bank: %s\n  Hidden elevator offset: %.8f\n',bankPath,hidden);
fprintf('  No auto-learning/BO/model-order search was used.\n');
end

function [rowResults,rowErrors]=local_execute_speed(root,bank,v,outRoot,o)
rowResults=cell(5,1);rowErrors=cell(5,1);
continuation=[];
nodeRoot=fullfile(outRoot,'linearization',sprintf('V%06.3f',v));
if ~isfolder(nodeRoot),mkdir(nodeRoot);end
for cfg=0:4
    job=struct('speed_mps',v,'cfg',cfg,'output_root',fullfile(nodeRoot,sprintf('cfg%d',cfg)));
    [L,E]=local_execute_node(root,bank,job,o,continuation);
    rowResults{cfg+1}=L;rowErrors{cfg+1}=E;
    if isempty(E)&&~isempty(L)
        continuation=struct('source_cfg',cfg, ...
            'physical_elevator_cmd',double(L.u_nominal(1)), ...
            'throttle_cmd',double(L.u_nominal(2)), ...
            'pitch_deg',double(L.x_nominal(2)));
        fprintf('[PHYS-MPC] V=%.1f continuation cfg%d -> next: physElev=%.6f th=%.6f pitch=%.4f deg\n', ...
            v,cfg,continuation.physical_elevator_cmd,continuation.throttle_cmd,continuation.pitch_deg);
    end
end
end

function [L,E]=local_execute_node(root,bank,job,o,continuation)
L=[];E=[];
try
    if logical(o.ReuseExistingNodeResults) && ~logical(o.ForceRebuild)
        p=fullfile(job.output_root,'physics_linear_model.mat');
        if isfile(p)
            S=load(p,'result');
            if isfield(S,'result') && isstruct(S.result) && isfield(S.result,'version') && ...
                    local_cache_compatible(S.result,job)
                oldVersion=string(S.result.version);
                L=S.result;
                if oldVersion=="physics_mpc_v1_5_cfg_continuation_nomex"
                    fprintf('[PHYS-MPC] COMPAT-REUSE V=%.1f cfg%d from v1.5 formal certification\n',job.speed_mps,job.cfg);
                else
                    fprintf('[PHYS-MPC] REUSE V=%.1f cfg%d\n',job.speed_mps,job.cfg);
                end
                return;
            end
        end
    end
    if isfolder(job.output_root),try,rmdir(job.output_root,'s');catch,end,end
    if ~isfolder(job.output_root),mkdir(job.output_root);end
    if isempty(continuation)
        fprintf('\n[PHYS-MPC] Linearizing V=%.1f cfg%d (source-bank seed) ...\n',job.speed_mps,job.cfg);
    else
        fprintf('\n[PHYS-MPC] Linearizing V=%.1f cfg%d (physical continuation from cfg%d) ...\n',job.speed_mps,job.cfg,continuation.source_cfg);
    end
    L=airdropx_physics_linearize_node('ProjectRoot',root,'OutputRoot',job.output_root, ...
        'TrimBank',bank,'ConfigId',job.cfg,'SpeedMps',job.speed_mps,'ContinuationSeed',continuation, ...
        'ReferenceAltitudeM',o.ReferenceAltitudeM,'ReferenceMassKg',o.ReferenceMassKg,'CargoMassKg',o.CargoMassKg,'Ts',o.Ts, ...
        'BaselineStopTimeS',o.BaselineStopTimeS,'BaselineTailS',o.BaselineTailS, ...
        'ExcitationStartS',o.ExcitationStartS,'ExcitationDurationS',o.ExcitationDurationS, ...
        'RunsPerNode',o.RunsPerNode,'ElevatorAmplitude',o.ElevatorAmplitude,'ThrottleAmplitude',o.ThrottleAmplitude, ...
        'FailOnPoorFit',o.FailOnPoorFit,'AllowDeterministicRetrim',o.AllowDeterministicRetrim, ...
        'RetrimMaxIterations',o.RetrimMaxIterations,'RetrimStopTimeS',o.RetrimStopTimeS,'RetrimTailWindowS',o.RetrimTailWindowS, ...
        'RetrimElevatorProbe',o.RetrimElevatorProbe,'RetrimThrottleProbe',o.RetrimThrottleProbe, ...
        'RetrimMaxElevatorStep',o.RetrimMaxElevatorStep,'RetrimMaxThrottleStep',o.RetrimMaxThrottleStep, ...
        'MaxPitchConsistencyIterations',o.MaxPitchConsistencyIterations,'PitchConsistencyTolDeg',o.PitchConsistencyTolDeg, ...
        'MaxRepeatVaErrorDiffMps',o.MaxRepeatVaErrorDiffMps,'MaxRepeatVzDiffMps',o.MaxRepeatVzDiffMps, ...
        'MaxRepeatQDiffDps',o.MaxRepeatQDiffDps,'MaxRepeatHeightSlopeDiffMps',o.MaxRepeatHeightSlopeDiffMps, ...
        'MaxRepeatVaSlopeDiffMps2',o.MaxRepeatVaSlopeDiffMps2,'MaxRepeatPitchDiffDeg',o.MaxRepeatPitchDiffDeg, ...
        'MaxRepeatElevatorDiff',o.MaxRepeatElevatorDiff,'MaxRepeatThrottleDiff',o.MaxRepeatThrottleDiff);
catch ME
    E=struct('identifier',ME.identifier,'message',ME.message,'report',getReport(ME,'extended','hyperlinks','off'));
    fprintf(2,'[PHYS-MPC] FAIL V=%.1f cfg%d: %s\n',job.speed_mps,job.cfg,ME.message);
end
end

function local_prepare_pool(n)
p=gcp('nocreate');
if ~isempty(p)
    try
        if p.NumWorkers==n && ~contains(class(p),'ThreadPool'),return;end
    catch
    end
    delete(p);
end
parpool('Processes',n);
end

function C=local_make_mpc(P,o)
Ts=double(o.Ts);C=mpc(P,Ts,round(o.Np),round(o.Nc));
C.Model.Nominal.U=zeros(2,1);C.Model.Nominal.Y=zeros(4,1);
C.MV(1).Min=-o.ElevatorDeviationLimit;C.MV(1).Max=o.ElevatorDeviationLimit;
C.MV(2).Min=-o.ThrottleDeviationLimit;C.MV(2).Max=o.ThrottleDeviationLimit;
C.MV(1).RateMin=-o.ElevatorDeviationRateLimit;C.MV(1).RateMax=o.ElevatorDeviationRateLimit;
C.MV(2).RateMin=-o.ThrottleDeviationRateLimit;C.MV(2).RateMax=o.ThrottleDeviationRateLimit;
C.Weights.OutputVariables=[o.Wva o.Wpitch o.Wvz o.Wq];
C.Weights.ManipulatedVariables=[o.WmvElev o.WmvThrottle];
C.Weights.ManipulatedVariablesRate=[o.WrateElev o.WrateThrottle];
sf=[o.ScaleVa o.ScalePitch o.ScaleVz o.ScaleQ];for j=1:4,C.OV(j).ScaleFactor=sf(j);end
C.MV(1).ScaleFactor=o.ElevatorDeviationLimit;C.MV(2).ScaleFactor=o.ThrottleDeviationLimit;
lims=double(o.OutputDeviationLimits(:));for j=1:4,C.OV(j).Min=-lims(j);C.OV(j).Max=lims(j);C.OV(j).MinECR=1;C.OV(j).MaxECR=1;end
try,setoutdist(C,'model',tf(zeros(4,1)));catch,end
end

function bank=local_load_trim_bank(v32Root,v)
nodeRoot=fullfile(v32Root,'knowledge_bank','physics',sprintf('V%06.3f',v));
p=fullfile(nodeRoot,'v32_trim_bank.mat');
if isfile(p),S=load(p);if isfield(S,'bank'),bank=S.bank;return;end,end
p=fullfile(nodeRoot,'v32_physics_verified.mat');
if isfile(p),S=load(p);if isfield(S,'physics')&&isfield(S.physics,'trim_bank'),bank=S.physics.trim_bank;return;end,end
error('AirdropX:PhysicsMPC:MissingTrim','No v32 trim bank for V=%.1f under %s.',v,nodeRoot);
end

function out=local_merge_trim(a,b)
% Preserve the schema of the source trim-bank struct array. Linearization-only
% fields (for example physical_elevator_cmd) belong in the node result, not in
% the shared trim bank.
out=a;fa=fieldnames(a);fb=fieldnames(b);f=intersect(fa,fb,'stable');
for k=1:numel(f),out.(f{k})=b.(f{k});end
end
function p=local_resolve(root,x),p=char(string(x));if isempty(p),p=fullfile(root,'matlab','results','mpc_physics_v1');elseif ~local_abs(p),p=fullfile(root,p);end,end
function tf=local_abs(p),tf=~isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once'));end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function local_write_manifest(outRoot,bankPath,hidden,o)
fid=fopen(fullfile(outRoot,'PHYSICS_MPC_V1_MANIFEST.txt'),'w');if fid<0,return;end;c=onCleanup(@()fclose(fid));
fprintf(fid,'AirdropX Physics MPC v1.6\n');fprintf(fid,'bank=%s\n',bankPath);fprintf(fid,'speed_nodes_mps=%s\n',mat2str(double(o.SpeedNodesMps(:).')));
fprintf(fid,'state=[Va pitch vz q]\ninput=[physical_elevator throttle]\n');fprintf(fid,'height_is_outer_governor=true\npitch_fixed_reference=false\n');
fprintf(fid,'online_learning=false\nbayesopt=false\nmodel_order_search=false\ndeterministic_equilibrium_solver=%d\n',logical(o.AllowDeterministicRetrim));
fprintf(fid,'hidden_elevator_trim=%.10g\n',hidden);fprintf(fid,'Np=%d Nc=%d Ts=%.3f\n',round(o.Np),round(o.Nc),o.Ts);
end

function tf=local_cache_compatible(r,job)
% v1.6 changes only the trim-solver observation horizon. A v1.5 node is
% compatible ONLY when it already passed the formal (28 s / 10 s) baseline,
% the independent repeatability run, model validation, and controllability.
tf=false;
try
    v=string(r.version);
    versionOK=(v=="physics_mpc_v1_6_cert_horizon_nomex") || (v=="physics_mpc_v1_5_cfg_continuation_nomex");
    nodeOK=abs(double(r.speed_mps)-double(job.speed_mps))<1e-9 && double(r.config_id)==double(job.cfg);
    if ~(versionOK&&nodeOK),return;end
    if v=="physics_mpc_v1_6_cert_horizon_nomex",tf=true;return;end

    baselineOK=isfield(r,'baseline')&&isstruct(r.baseline)&&isfield(r.baseline,'pass')&&logical(r.baseline.pass);
    repeatBaseOK=isfield(r,'baseline_repeat')&&isstruct(r.baseline_repeat)&&isfield(r.baseline_repeat,'pass')&&logical(r.baseline_repeat.pass);
    repeatDiffOK=isfield(r,'repeatability')&&isstruct(r.repeatability);
    fitOK=isfield(r,'fit')&&isstruct(r.fit)&&isfield(r.fit,'controllability_rank')&&double(r.fit.controllability_rank)==4;
    valOK=isfield(r,'validation')&&isstruct(r.validation)&&isfield(r.validation,'rmse_10step')&&all(isfinite(double(r.validation.rmse_10step(:))));
    tf=baselineOK&&repeatBaseOK&&repeatDiffOK&&fitOK&&valOK;
catch
    tf=false;
end
end

function opts=local_options(varargin)
opts.ProjectRoot="";opts.OutputRoot="matlab/results/mpc_physics_v1";opts.ExistingV32Root="matlab/results/mpc_auto_v32_clean";
opts.SpeedNodesMps=[45;50;55];opts.ReferenceAltitudeM=200;opts.ReferenceMassKg=3423;opts.CargoMassKg=300;opts.Ts=0.1;
opts.BaselineStopTimeS=28;opts.BaselineTailS=10;opts.ExcitationStartS=18;opts.ExcitationDurationS=30;opts.RunsPerNode=2;
opts.ElevatorAmplitude=0.012;opts.ThrottleAmplitude=0.025;opts.FailOnPoorFit=true;
opts.ParallelWorkers=3;opts.ReuseExistingNodeResults=true;opts.ForceRebuild=false;opts.AllowDeterministicRetrim=true;
opts.RetrimMaxIterations=7;opts.RetrimStopTimeS=28;opts.RetrimTailWindowS=10;opts.RetrimElevatorProbe=0.008;opts.RetrimThrottleProbe=0.015;opts.RetrimMaxElevatorStep=0.05;opts.RetrimMaxThrottleStep=0.08;opts.MaxPitchConsistencyIterations=3;opts.PitchConsistencyTolDeg=0.20;
opts.MaxRepeatVaErrorDiffMps=0.10;opts.MaxRepeatVzDiffMps=0.05;opts.MaxRepeatQDiffDps=0.05;
opts.MaxRepeatHeightSlopeDiffMps=0.05;opts.MaxRepeatVaSlopeDiffMps2=0.02;opts.MaxRepeatPitchDiffDeg=0.20;
opts.MaxRepeatElevatorDiff=5e-4;opts.MaxRepeatThrottleDiff=5e-4;
opts.Np=30;opts.Nc=6;opts.Wva=6;opts.Wpitch=0.15;opts.Wvz=24;opts.Wq=3.0;opts.WmvElev=0.08;opts.WmvThrottle=0.08;opts.WrateElev=2.5;opts.WrateThrottle=1.8;
opts.ScaleVa=3;opts.ScalePitch=8;opts.ScaleVz=1.2;opts.ScaleQ=4;opts.OutputDeviationLimits=[12;15;5;10];
opts.ElevatorDeviationLimit=0.10;opts.ThrottleDeviationLimit=0.18;opts.ElevatorDeviationRateLimit=0.012;opts.ThrottleDeviationRateLimit=0.020;opts.MaxHiddenOffsetCfgSpread=0.01;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
