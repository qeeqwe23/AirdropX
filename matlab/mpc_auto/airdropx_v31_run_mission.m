function result = airdropx_v31_run_mission(varargin)
%AIRDROPX_V31_RUN_MISSION Standard on-demand v31 H/V entry point.
%
% A requested target altitude is never used as a controller-training location.
% The requested airspeed is first resolved at ReferenceAltitudeM using the fixed
% v31 context state machine. Once that reference-speed context is VERIFIED, the
% exact Plant/controller state is frozen and the requested altitude receives one
% complete cfg0->cfg4 mission qualification with learning disabled.
%
% Re-run the same call if status="REFERENCE_SPEED_PENDING"; each invocation
% advances at most one bounded architecture-requal / governor-local level.

opts=local_options(varargin{:});
projectRoot=local_project_root(opts.ProjectRoot);
H=double(opts.TargetAltitudeM); V=double(opts.TargetAirspeedMps); Href=double(opts.ReferenceAltitudeM);
if H<double(opts.MinimumAltitudeM)-1e-9 || H>double(opts.MaximumAltitudeM)+1e-9
    error("AirdropX:V31:AltitudeOutsideRequestedRange","TargetAltitudeM %.3f is outside %.3f..%.3f m.",H,opts.MinimumAltitudeM,opts.MaximumAltitudeM);
end
root=local_resolve_path(projectRoot,opts.OutputRoot); if ~isfolder(root), mkdir(root); end
% Ensure the shared v31 private backends are seeded even when this on-demand
% entry point is used before the curriculum runner.
airdropx_v31_import_legacy('ProjectRoot',projectRoot,'KnowledgeBankRoot',opts.KnowledgeBankRoot, ...
    'LegacyLearningBankRoot',opts.LegacyLearningBankRoot,'LegacyPlantBankRoot',opts.LegacyPlantBankRoot, ...
    'LegacyEnvelopeRoot',opts.LegacyEnvelopeRoot,'ReferenceMassKg',opts.ReferenceMassKg, ...
    'CargoMassKg',opts.CargoMassKg,'TotalDropCount',opts.TotalDropCount,'SkipIfMarkerExists',true);
refRoot=fullfile(root,'reference_contexts',local_context_tag(Href,V));

Rref=airdropx_v31_run_context( ...
    'ProjectRoot',projectRoot,'OutputRoot',refRoot,'KnowledgeBankRoot',opts.KnowledgeBankRoot, ...
    'BaseIdentifiedMat',opts.BaseIdentifiedMat,'TargetAltitudeM',Href,'TargetAirspeedMps',V, ...
    'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg,'TotalDropCount',opts.TotalDropCount, ...
    'UseParallel',opts.UseParallel,'ParallelWorkers',min(3,round(double(opts.ParallelWorkers))), ...
    'TransferSeedEvaluations',opts.TransferSeedEvaluations,'LocalPolishEvaluations',opts.LocalPolishEvaluations, ...
    'LocalBayesEvaluations',opts.LocalBayesEvaluations,'BroadBayesEvaluationsPerCall',opts.BroadBayesEvaluationsPerCall, ...
    'GovernorPolishEvaluations',opts.GovernorPolishEvaluations,'GovernorBayesEvaluations',opts.GovernorBayesEvaluations, ...
    'HeightVzSlewRateMps2',opts.HeightVzSlewRateMps2,'HeightBiasFraction',opts.HeightBiasFraction,'HeightBiasLeak',opts.HeightBiasLeak);

result=struct(); result.target_altitude_m=H; result.target_airspeed_mps=V;
result.reference_altitude_m=Href; result.reference_context_root=string(refRoot);
result.reference_state=string(Rref.state); result.reference_gate_ratio=double(Rref.gate_ratio);
result.output_root=string(root); result.mission_pass=false; result.mission_gate_ratio=Inf;

if string(Rref.state)=="BLOCKED"
    result.status="REFERENCE_SPEED_BLOCKED";
    result.failure_class=string(Rref.failure_class);
    fprintf("[V31.2-MISSION] V=%.3f reference context BLOCKED at H=%.1f; no target-H learning will be attempted.\n",V,Href);
    return;
elseif string(Rref.state)~="VERIFIED"
    result.status="REFERENCE_SPEED_PENDING"; result.failure_class=string(Rref.failure_class);
    fprintf("[V31.2-MISSION] V=%.3f reference context is %s retry=%s. Re-run the same command to advance one bounded level.\n", ...
        V,string(Rref.state),string(Rref.retry_level));
    return;
end

% At the reference altitude the VERIFIED context already contains an
% authoritative full mission result; no duplicate qualification is necessary.
if abs(H-Href)<1e-9
    result.status="MISSION_VERIFIED"; result.mission_pass=logical(Rref.mission_pass);
    result.mission_gate_ratio=double(Rref.gate_ratio); result.qualification_root=string(refRoot);
    return;
end

qRoot=fullfile(root,'qualification',local_context_tag(H,V));
Rq=airdropx_v31_qualify_height('ProjectRoot',projectRoot,'SourceVerifiedRoot',refRoot, ...
    'OutputRoot',qRoot,'KnowledgeBankRoot',opts.KnowledgeBankRoot,'TargetAltitudeM',H, ...
    'TargetAirspeedMps',V,'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg, ...
    'TotalDropCount',opts.TotalDropCount,'HeightVzSlewRateMps2',opts.HeightVzSlewRateMps2, ...
    'HeightBiasFraction',opts.HeightBiasFraction,'HeightBiasLeak',opts.HeightBiasLeak);
result.qualification_root=string(qRoot); result.mission_pass=logical(Rq.mission_pass);
result.mission_gate_ratio=double(Rq.mission_gate_ratio);
if result.mission_pass
    result.status="MISSION_VERIFIED"; result.failure_class="none";
else
    result.status="HEIGHT_GENERALIZATION_FAIL"; result.failure_class="MISSION_FAIL";
end
end

function tag=local_context_tag(H,V), tag="H"+local_num_tag(H)+"_V"+local_num_tag(V); end
function s=local_num_tag(x), s=sprintf('%.3f',double(x)); s=strrep(s,'-','m'); s=strrep(s,'.','p'); end
function root=local_project_root(root), root=string(root); if strlength(root)==0, root=string(pwd); end, end
function p=local_resolve_path(projectRoot,p), p=string(p); c=char(p); absPath=startsWith(c,'/')||startsWith(c,'\\')||~isempty(regexp(c,'^[A-Za-z]:[\\/]','once')); if ~absPath, p=string(fullfile(projectRoot,p)); end, end
function opts=local_options(varargin)
opts=struct(); opts.ProjectRoot=""; opts.OutputRoot="matlab/results/mpc_auto_v31";
opts.KnowledgeBankRoot="matlab/results/mpc_auto_v31_knowledge_bank";
opts.LegacyLearningBankRoot="matlab/results/mpc_auto_global_learning_bank"; opts.LegacyPlantBankRoot="matlab/results/mpc_auto_global_plant_bank";
opts.LegacyEnvelopeRoot="matlab/results/mpc_auto_flight_envelope_v30";
opts.BaseIdentifiedMat="matlab/results/mpc_auto_id_v11_clean_r1/identify/airdropx_identified_plants.mat";
opts.TargetAltitudeM=100; opts.TargetAirspeedMps=50; opts.ReferenceAltitudeM=200;
opts.MinimumAltitudeM=20; opts.MaximumAltitudeM=200; opts.ReferenceMassKg=3423; opts.CargoMassKg=300; opts.TotalDropCount=4;
opts.UseParallel=true; opts.ParallelWorkers=3; opts.TransferSeedEvaluations=3; opts.LocalPolishEvaluations=6;
opts.LocalBayesEvaluations=9; opts.BroadBayesEvaluationsPerCall=12;
opts.GovernorPolishEvaluations=6; opts.GovernorBayesEvaluations=6;
opts.HeightVzSlewRateMps2=0.30; opts.HeightBiasFraction=0.70; opts.HeightBiasLeak=1.0;
if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin), n=string(varargin{i}); if ~isfield(opts,n), error("Unknown option: %s",n); end, opts.(n)=varargin{i+1}; end
end
