function result = airdropx_v31_run_context(varargin)
%AIRDROPX_V31_RUN_CONTEXT One resumable v31 reference-altitude training context.
%
% v31.2 keeps the fixed context directory and explicit per-active-cfg state:
%   PHYSICS_PENDING -> CONTROLLER_PENDING -> MISSION_PENDING -> VERIFIED
% v31.2 separates controller-architecture requalification from governor learning:
%   architecture_requal (1 old-best certification) -> governor_local (3-D) -> BLOCKED.
% For genuinely new contexts zero_shot is still allowed before governor_local.
% A later cfg that has not been visited yet no longer makes the current cfg
% incorrectly appear PHYSICS_PENDING.

opts=local_options(varargin{:});
projectRoot=local_project_root(opts.ProjectRoot);
H=double(opts.TargetAltitudeM); V=double(opts.TargetAirspeedMps);
if strlength(string(opts.OutputRoot))==0
    root=fullfile(local_resolve_path(projectRoot,opts.ContextRoot),local_context_tag(H,V));
else
    root=local_resolve_path(projectRoot,opts.OutputRoot);
end
if ~isfolder(root), mkdir(root); end
kbRoot=local_resolve_path(projectRoot,opts.KnowledgeBankRoot);
airdropx_v31_knowledge_bank('Action','init','ProjectRoot',projectRoot,'Root',kbRoot);
backendLearning=fullfile(kbRoot,'backend_learning');
backendPlant=fullfile(kbRoot,'backend_plant');
if ~isfolder(backendLearning), mkdir(backendLearning); end
if ~isfolder(backendPlant), mkdir(backendPlant); end
stateFile=fullfile(root,'v31_state.mat');
state=local_load_state(stateFile,H,V);
[state,architectureMigrated]=local_v31_2_migrate_controller_architecture(state,stateFile,root);

% Authoritative checkpoint artifacts always outrank stale state text.
[p0,c0,m0,g0,missionResultPresent0,d0]=local_status(root);
[state,repairedLocal]=local_reconcile_state(state,root,p0,c0,m0,g0,missionResultPresent0,d0);
[transferN,nearEnabled,forceLocal,layeredLocal,localPolishN,localBayesN,broadN]= ...
    local_learning_level_budget(state.retry_level,opts);

fprintf("\n============================================================\n");
fprintf("[V31.2] reference-context training H=%.3f V=%.3f\n",H,V);
fprintf("[V31.2] fixed context root: %s\n",root);
fprintf("[V31.2] state=%s active_cfg=%s active_physics=%d all_physics=%d retry=%s\n", ...
    state.state,local_cfg_text(state.active_cfg),logical(state.active_cfg_physics_ready), ...
    logical(state.all_physics_ready),state.retry_level);
if architectureMigrated
    fprintf("[V31.2-MIGRATE] old controller certifications marked STALE for one-time height-governor architecture requalification.\n");
end
if repairedLocal
    fprintf("[V31.1-MIGRATE] legacy broad without a completed local round was repaired.\n");
end
fprintf("[V31.2] budget transfer=%d forceLocal=%d localDet=%d localBO=%d broadBO=%d\n", ...
    transferN,forceLocal,localPolishN,localBayesN,broadN);
fprintf("[V31.2] height path: h -> conditional-AW/slew-limited vz_ref -> inner MPC; direct altitude MPC weight=0.\n");
fprintf("============================================================\n");

if state.state=="VERIFIED"
    fprintf("[V31.2] context already VERIFIED -> no work.\n");
    result=local_result(state,root); return;
elseif state.state=="BLOCKED"
    fprintf("[V31.2] context is BLOCKED after bounded learning/mission work -> no blind retry.\n");
    result=local_result(state,root); return;
elseif state.state=="MISSION_PENDING"
    fprintf("[V31.2] all controllers VERIFIED -> mission validation only.\n");
    try
        Rm=airdropx_auto_final_mission_validation( ...
            'ProjectRoot',projectRoot,'OutputRoot',root,'ValidationSubdir','final_mission_validation', ...
            'TargetAltitudeM',H,'TargetAirspeedMps',V, ...
            'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg, ...
            'TotalDropCount',opts.TotalDropCount,'BumplessTransitionEnabled',false, ...
            'TransitionMoveTransferScale',0.0,'TransitionIntegralTransferScale',0.0, ...
            'V31ContinuousControllerStateEnabled',true, ...
            'V31HeightGovernorEnabled',true, ...
            'V31HeightVzSlewRateMps2',double(opts.HeightVzSlewRateMps2), ...
            'V31HeightBiasFraction',double(opts.HeightBiasFraction), ...
            'V31HeightBiasLeak',double(opts.HeightBiasLeak));
        state.mission_attempts=state.mission_attempts+1;
        state.mission_pass=logical(Rm.mission_pass);
        if isfield(Rm,'mission_gate_ratio'), state.gate_ratio=double(Rm.mission_gate_ratio); end
        if state.mission_pass
            state.state="VERIFIED"; state.failure_class="none"; state.retry_level="complete"; state.next_action="none";
        else
            state.state="BLOCKED"; state.failure_class="MISSION_FAIL"; state.next_action="mission_policy_or_architecture_review";
        end
    catch ME
        state.last_error=string(getReport(ME,'extended','hyperlinks','off'));
        state.failure_class=local_classify_result(root,ME);
        if state.failure_class=="INFRA_INVALID"
            state.state="MISSION_PENDING"; state.next_action="retry_same_mission_after_infra_fix";
        else
            state.state="BLOCKED"; state.next_action="mission_policy_or_architecture_review";
        end
    end
    state.updated_at=string(datetime('now')); local_save_state(stateFile,state); local_record_state(kbRoot,projectRoot,root,state);
    local_record_mission_if_present(kbRoot,projectRoot,root,opts);
    result=local_result(state,root); return;
end

activeCfgBefore=double(state.active_cfg);
if ~isfinite(activeCfgBefore)
    activeCfgBefore=0;
end
activeCfgBefore=round(activeCfgBefore);
evalBefore=local_valid_eval_count(root,activeCfgBefore);

% v31.1 executes only the current active cfg. Already VERIFIED lower cfgs are
% checkpoint dependencies, not tasks to revalidate on every learning level.
configIds=activeCfgBefore;
if logical(state.active_cfg_physics_ready)
    state.state="CONTROLLER_PENDING";
    state.next_action=local_controller_next_action(state.retry_level);
else
    state.state="PHYSICS_PENDING";
    state.next_action="physics_validation_or_rebuild";
end
state.updated_at=string(datetime('now')); local_save_state(stateFile,state); local_record_state(kbRoot,projectRoot,root,state);

try
    R=airdropx_auto_run_any_mission( ...
        'ProjectRoot',projectRoot,'OutputRoot',root, ...
        'BaseIdentifiedMat',opts.BaseIdentifiedMat, ...
        'LearningBankRoot',backendLearning, ...
        'PlantBankRoot',backendPlant, ...
        'TargetAltitudeM',H,'TargetAirspeedMps',V, ...
        'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg, ...
        'TotalDropCount',opts.TotalDropCount,'ConfigIds',configIds, ...
        'UseParallel',logical(opts.UseParallel),'ParallelWorkers',min(3,round(double(opts.ParallelWorkers))), ...
        'MaxAttempts',1,'RegisterPlant',true, ...
        'UnifiedTransferSeedEvaluations',transferN, ...
        'UnifiedAdditionalEvaluationsPerRun',broadN, ...
        'UnifiedControllerNearPassEnabled',logical(nearEnabled), ...
        'UnifiedControllerNearPassGateRatioMax',double(opts.ControllerNearPassGateRatioMax), ...
        'UnifiedControllerNearPassDeterministicEvaluations',localPolishN, ...
        'UnifiedControllerNearPassBayesEvaluations',localBayesN, ...
        'UnifiedControllerNearPassMaxRoundsPerContext',1, ...
        'UnifiedForceControllerLocalRefinement',logical(forceLocal), ...
        'UnifiedV31LayeredLocalRefinement',logical(layeredLocal), ...
        'UnifiedV31ArchitectureRequal',string(state.retry_level)=="architecture_requal", ...
        'V31HeightGovernorEnabled',true, ...
        'V31HeightVzSlewRateMps2',double(opts.HeightVzSlewRateMps2), ...
        'V31HeightBiasFraction',double(opts.HeightBiasFraction), ...
        'V31HeightBiasLeak',double(opts.HeightBiasLeak), ...
        'UniversalMissionNearPassEnabled',false, ...
        'BumplessTransitionEnabled',false, ...
        'TransitionMoveTransferScale',0.0,'TransitionIntegralTransferScale',0.0, ...
        'V31ContinuousControllerStateEnabled',true);
catch ME
    state.last_error=string(getReport(ME,'extended','hyperlinks','off'));
    failureClass=local_classify_result(root,ME);
    [p1,c1,m1,g1,missionPresent1,d1]=local_status(root);
    evalAfter=local_valid_eval_count(root,activeCfgBefore);
    state=local_after_learning(state,p1,c1,m1,g1,missionPresent1,d1,failureClass, ...
        activeCfgBefore,evalBefore,evalAfter);
    state.learning_calls=state.learning_calls+1; state.updated_at=string(datetime('now'));
    local_save_state(stateFile,state); local_record_state(kbRoot,projectRoot,root,state);
    local_record_mission_if_present(kbRoot,projectRoot,root,opts);
    local_refresh_backend_knowledge(kbRoot,projectRoot,root,backendLearning,backendPlant,opts);
    warning("AirdropX:V31_2:ContextCall","v31.2 bounded context call ended [%s]: %s",state.failure_class,ME.message);
    result=local_result(state,root); return;
end

failureClass="none";
if isfield(R,'infrastructure_failure') && logical(R.infrastructure_failure)
    failureClass="INFRA_INVALID";
    if isfield(R,'last_error'), state.last_error=string(R.last_error); end
elseif isfield(R,'failure_class') && strlength(string(R.failure_class))>0 && string(R.failure_class)~="none"
    rc=lower(string(R.failure_class));
    if contains(rc,"trim")||contains(rc,"plant"), failureClass="PHYSICS_FAIL";
    elseif contains(rc,"infra"), failureClass="INFRA_INVALID";
    else, failureClass="CONTROLLER_FAIL"; end
end
[p1,c1,m1,g1,missionPresent1,d1]=local_status(root);
evalAfter=local_valid_eval_count(root,activeCfgBefore);
state=local_after_learning(state,p1,c1,m1,g1,missionPresent1,d1,failureClass, ...
    activeCfgBefore,evalBefore,evalAfter);
state.learning_calls=state.learning_calls+1; state.updated_at=string(datetime('now'));
local_save_state(stateFile,state); local_record_state(kbRoot,projectRoot,root,state);
local_record_mission_if_present(kbRoot,projectRoot,root,opts);
local_refresh_backend_knowledge(kbRoot,projectRoot,root,backendLearning,backendPlant,opts);
result=local_result(state,root);
end

function [state,migrated]=local_v31_2_migrate_controller_architecture(state,stateFile,root)
% One-time v31.1 -> v31.2 migration. The Plant/trim and measured controller
% parameters remain valuable, but controller FORMAL PASS status is invalidated
% because the closed-loop architecture changed (direct altitude MPC weight is
% removed and a new governor/anti-windup/slew state is introduced).
migrated=false;
marker=fullfile(root,'V31_2_ARCHITECTURE_MIGRATED.txt');
if isfile(marker)
    state.schema_version="v31.2";
    return;
end
cpFile=fullfile(root,'airdropx_200m_cfg_checkpoint.mat');
if ~isfile(cpFile)
    state.schema_version="v31.2";
    return;
end
try
    S=load(cpFile,'checkpoint'); cp=S.checkpoint;
    backup=fullfile(root,'airdropx_200m_cfg_checkpoint_pre_v31_2.mat');
    if ~isfile(backup), copyfile(cpFile,backup); end
    if ~isfield(cp,'status')||numel(cp.status)<5, cp.status=repmat("pending",5,1); end
    if ~isfield(cp,'best_candidate')||numel(cp.best_candidate)<5, cp.best_candidate=cell(5,1); end
    if ~isfield(cp,'plant_ready')||numel(cp.plant_ready)<5, cp.plant_ready=false(5,1); end
    if ~isfield(cp,'plant_generation')||numel(cp.plant_generation)<5, cp.plant_generation=zeros(5,1); end
    if ~isfield(cp,'equilibrium_probe_pass')||numel(cp.equilibrium_probe_pass)<5, cp.equilibrium_probe_pass=false(5,1); end
    if ~isfield(cp,'equilibrium_probe_generation')||numel(cp.equilibrium_probe_generation)<5, cp.equilibrium_probe_generation=-ones(5,1); end
    for k=1:5
        if ~isempty(cp.best_candidate{k})
            cp.status(k)="stale_v31_2_architecture";
            % A measured old-best controller could only have been produced after
            % the old pipeline accepted this Plant generation. Architecture
            % migration must not reopen trim/ID merely because a legacy probe
            % bookkeeping flag is absent/stale.
            if logical(cp.plant_ready(k))
                cp.equilibrium_probe_pass(k)=true;
                cp.equilibrium_probe_generation(k)=cp.plant_generation(k);
            end
        else
            cp.status(k)="pending";
        end
    end
    if isfield(cp,'failed_certified_signatures'), cp.failed_certified_signatures=cell(5,1); end
    if isfield(cp,'verification_count'), cp.verification_count=zeros(5,1); end
    if isfield(cp,'last_metrics'), cp.last_metrics=cell(5,1); end
    if isfield(cp,'final_mission_pass'), cp.final_mission_pass=false; end
    cp.updated_at=string(datetime('now'));
    checkpoint=cp; %#ok<NASGU>
    save(cpFile,'checkpoint','-v7.3');
    % An old Final Mission result belongs to the previous controller
    % architecture. Preserve it for audit but remove it from authoritative
    % v31.2 state reconciliation so a fresh mission is mandatory.
    oldMission=fullfile(root,'final_mission_validation');
    if isfolder(oldMission)
        archived=fullfile(root,'final_mission_validation_pre_v31_2');
        if isfolder(archived)
            archived=fullfile(root,'final_mission_validation_pre_v31_2_'+char(datetime('now','Format','yyyyMMdd_HHmmss')));
        end
        movefile(oldMission,archived);
    end
    fid=fopen(marker,'w');
    if fid>=0
        fprintf(fid,'v31.2 controller architecture migration\n');
        fprintf(fid,'timestamp=%s\n',char(string(datetime('now'))));
        fprintf(fid,'old best candidates preserved; certification status invalidated once\n');
        fprintf(fid,'effective direct altitude MPC weight=0\n');
        fprintf(fid,'height governor=conditional anti-windup + slew-limited vz reference\n');
        fclose(fid);
    end
    state.schema_version="v31.2";
    state.state="CONTROLLER_PENDING";
    state.failure_class="none";
    state.retry_level="architecture_requal";
    state.next_action="v31_2_architecture_requalification";
    state.migration_note="v31.1 controller certifications invalidated once for v31.2 height-governor architecture requalification";
    state.updated_at=string(datetime('now'));
    local_save_state(stateFile,state);
    migrated=true;
catch ME
    warning('AirdropX:V31_2:ArchitectureMigration','Could not migrate controller architecture: %s',ME.message);
end
end

function local_refresh_backend_knowledge(kbRoot,projectRoot,root,backendLearning,backendPlant,opts)
try
    airdropx_v31_import_legacy('ProjectRoot',projectRoot,'KnowledgeBankRoot',kbRoot, ...
        'LegacyLearningBankRoot',backendLearning,'LegacyPlantBankRoot',backendPlant, ...
        'LegacyEnvelopeRoot',fileparts(fileparts(root)), ...
        'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg,'TotalDropCount',opts.TotalDropCount);
catch ME
    warning("AirdropX:V31:ImportAfterRun","Could not refresh v31 KnowledgeBank after context run: %s",ME.message);
end
end
function [physicsReady,controllersReady,missionPass,gate,missionResultPresent,detail]=local_status(root)
% physicsReady means the CURRENT active cfg physics is ready. all_physics_ready
% is reported separately so future cfgs no longer mask current controller work.
physicsReady=false; controllersReady=false; missionPass=false; gate=Inf; missionResultPresent=false;
detail=struct('active_cfg',NaN,'active_cfg_physics_ready',false,'all_physics_ready',false, ...
    'all_controllers_verified',false,'status',repmat("pending",5,1),'plant_ready',false(5,1));
cpFile=fullfile(root,'airdropx_200m_cfg_checkpoint.mat');
if isfile(cpFile)
    try
        S=load(cpFile,'checkpoint'); cp=S.checkpoint;
        if isfield(cp,'plant_ready')&&numel(cp.plant_ready)>=5
            detail.plant_ready=logical(cp.plant_ready(1:5));
        end
        if isfield(cp,'status')&&numel(cp.status)>=5
            detail.status=string(cp.status(1:5));
        end
        controllersReady=all(detail.status=="verified");
        detail.all_controllers_verified=controllersReady;
        detail.all_physics_ready=all(detail.plant_ready);
        k=find(detail.status~="verified",1,'first');
        if isempty(k)
            detail.active_cfg=NaN;
            detail.active_cfg_physics_ready=detail.all_physics_ready;
        else
            detail.active_cfg=k-1;
            activeReady=detail.plant_ready(k);
            st=lower(detail.status(k));
            if contains(st,"plant_probe_failed")||contains(st,"trim_failed")||contains(st,"plant_rebuild")
                activeReady=false;
            end
            if isfield(cp,'tuning_stage')&&numel(cp.tuning_stage)>=k
                ts=lower(string(cp.tuning_stage(k)));
                if contains(ts,"plant_rebuild"), activeReady=false; end
            end
            if isfield(cp,'equilibrium_probe_pass')&&numel(cp.equilibrium_probe_pass)>=k && detail.plant_ready(k)
                % A completed controller-learning status is proof that physics
                % validation was passed even if an old checkpoint lacks this flag.
                if ~contains(st,"unified_learning") && ~contains(st,"stale_v31_2_architecture") && ...
                        st~="failed" && st~="stale"
                    activeReady=activeReady && logical(cp.equilibrium_probe_pass(k));
                end
            end
            detail.active_cfg_physics_ready=logical(activeReady);
        end
        physicsReady=logical(detail.active_cfg_physics_ready);
        if isfield(cp,'final_mission_pass'), missionPass=logical(cp.final_mission_pass); end
    catch
    end
end
sumFile=fullfile(root,'final_mission_validation','final_mission_summary.csv');
if isfile(sumFile)
    try
        T=readtable(sumFile,'TextType','string');
        missionResultPresent=~isempty(T);
        if ismember('mission_pass',T.Properties.VariableNames), missionPass=local_bool(T.mission_pass(1)); end
        if ismember('mission_gate_ratio',T.Properties.VariableNames), gate=local_num(T.mission_gate_ratio(1)); end
    catch
    end
end
end

function [state,repairedLocal]=local_reconcile_state(state,root,physicsReady,controllersReady,missionPass,gate,missionPresent,detail)
repairedLocal=false;
oldActive=NaN;
if isfield(state,'active_cfg'), oldActive=double(state.active_cfg); end
state=local_apply_status_fields(state,physicsReady,controllersReady,missionPass,gate,detail);
state.schema_version="v31.2";
if missionPass
    state.state="VERIFIED"; state.retry_level="complete"; state.next_action="none"; return;
elseif controllersReady && missionPresent
    state.state="BLOCKED"; state.failure_class="MISSION_FAIL"; state.retry_level="mission";
    state.next_action="mission_policy_or_architecture_review"; return;
elseif controllersReady
    state.state="MISSION_PENDING"; state.retry_level="mission"; state.next_action="mission_validation"; return;
end
if isfinite(oldActive) && isfinite(double(detail.active_cfg)) && round(oldActive)~=round(double(detail.active_cfg))
    idx=round(double(detail.active_cfg))+1;
    if idx>=1 && idx<=numel(detail.status) && contains(lower(string(detail.status(idx))),"stale_v31_2_architecture")
        state.retry_level="architecture_requal";
    else
        state.retry_level="zero_shot";
    end
end
if ~isfield(state,'retry_level')||strlength(string(state.retry_level))==0||string(state.retry_level)=="complete"||string(state.retry_level)=="mission"
    state.retry_level="zero_shot";
end
% v31.0 bug migration: LOCAL never actually ran unless a refinement round
% exists.  Do not let a stale retry_level=broad skip the intended local layer.
if string(state.retry_level)=="broad" && isfinite(double(detail.active_cfg)) && ...
        logical(detail.active_cfg_physics_ready) && ~local_local_refinement_completed(root,double(detail.active_cfg))
    state.retry_level="local"; repairedLocal=true;
    state.migration_note="v31.0 broad rolled back to local because no completed local refinement round exists";
end
if logical(detail.active_cfg_physics_ready)
    state.state="CONTROLLER_PENDING"; state.next_action=local_controller_next_action(state.retry_level);
else
    state.state="PHYSICS_PENDING"; state.next_action="physics_validation_or_rebuild";
end
end

function state=local_apply_status_fields(state,physicsReady,controllersReady,missionPass,gate,detail)
state.physics_ready=logical(physicsReady);
state.controllers_ready=logical(controllersReady);
state.mission_pass=logical(missionPass);
state.gate_ratio=double(gate);
state.active_cfg=double(detail.active_cfg);
state.active_cfg_physics_ready=logical(detail.active_cfg_physics_ready);
state.all_physics_ready=logical(detail.all_physics_ready);
state.all_controllers_verified=logical(detail.all_controllers_verified);
end

function state=local_after_learning(state,physicsReady,controllersReady,missionPass,gate,missionPresent,detail,failureClass,activeCfgBefore,evalBefore,evalAfter)
state=local_apply_status_fields(state,physicsReady,controllersReady,missionPass,gate,detail);
state.failure_class=string(failureClass);
if state.failure_class=="none", state.last_error=""; end
if missionPass
    state.state="VERIFIED"; state.retry_level="complete"; state.next_action="none"; return;
elseif controllersReady && missionPresent
    state.state="BLOCKED"; state.failure_class="MISSION_FAIL"; state.retry_level="mission";
    state.next_action="mission_policy_or_architecture_review"; return;
elseif controllersReady
    state.state="MISSION_PENDING"; state.retry_level="mission"; state.next_action="mission_validation"; return;
end
activeAfter=double(detail.active_cfg);
if isfinite(activeAfter) && round(activeAfter)~=round(activeCfgBefore)
    % The previous cfg graduated. If the next cfg has an imported old best it
    % needs a one-time architecture requalification; otherwise start zero-shot.
    idx=round(activeAfter)+1;
    if idx>=1 && idx<=numel(detail.status) && contains(lower(string(detail.status(idx))),"stale_v31_2_architecture")
        state.retry_level="architecture_requal";
    else
        state.retry_level="zero_shot";
    end
elseif state.failure_class~="INFRA_INVALID" && evalAfter>evalBefore
    state.retry_level=local_next_retry(state.retry_level);
end
if logical(detail.active_cfg_physics_ready)
    state.state="CONTROLLER_PENDING";
    if state.failure_class=="none" && evalAfter>evalBefore, state.failure_class="CONTROLLER_FAIL"; end
    state.next_action=local_controller_next_action(state.retry_level);
else
    state.state="PHYSICS_PENDING";
    state.next_action="physics_validation_or_rebuild";
end
if string(state.retry_level)=="unresolved"
    state.state="BLOCKED"; state.next_action="inspect_height_governor_or_inner_vz_tracking";
end
end

function tf=local_local_refinement_completed(root,cfgId)
tf=false;
if ~isfinite(cfgId), return; end
fGov=fullfile(root,sprintf('cfg%d',round(cfgId)),'unified_learning','height_governor_refinement','rounds.csv');
fLegacy=fullfile(root,sprintf('cfg%d',round(cfgId)),'unified_learning','controller_nearpass_refinement','rounds.csv');
if isfile(fGov), f=fGov; elseif isfile(fLegacy), f=fLegacy; else, return; end
try
    T=readtable(f,'TextType','string');
    tf=~isempty(T)&&height(T)>0;
catch
    tf=false;
end
end

function n=local_valid_eval_count(root,cfgId)
% Count VALID v31.2 architecture certifications directly from immutable eval
% records.  Do not use unified_history.csv here: local_load_unified_history
% intentionally rewrites that convenience file for the current architecture,
% so its row count can shrink when v31.1 history is segregated from v31.2.
n=0;
if ~isfinite(cfgId), return; end
evalRoot=fullfile(root,sprintf('cfg%d',round(cfgId)),'unified_learning','evaluations');
if ~isfolder(evalRoot), return; end
D=dir(fullfile(evalRoot,'eval_*')); D=D([D.isdir]);
for i=1:numel(D)
    rec=fullfile(D(i).folder,D(i).name,'unified_record.csv');
    cert=fullfile(D(i).folder,D(i).name,'certification_summary.csv');
    if ~isfile(rec)||~isfile(cert), continue; end
    try
        R=readtable(rec,'TextType','string','VariableNamingRule','preserve');
        if isempty(R), continue; end
        vars=string(R.Properties.VariableNames);
        if ~ismember('context_signature',vars), continue; end
        if ~contains(string(R.context_signature(1)),'_Archv31p2_height_governor'), continue; end
        if ismember('infrastructure_fail',vars) && local_bool_vector(R.infrastructure_fail(1)), continue; end
        C=readtable(cert,'TextType','string','VariableNamingRule','preserve');
        if isempty(C)||~all(ismember(["formal_pass","hard_fail"],string(C.Properties.VariableNames))), continue; end
        n=n+1;
    catch
    end
end
end

function y=local_bool_vector(x)
if islogical(x), y=x; return; end
if isnumeric(x), y=x~=0; return; end
s=lower(strtrim(string(x))); y=s=="true"|s=="1"|s=="yes";
end

function a=local_controller_next_action(level)
level=string(level);
if level=="architecture_requal", a="v31_2_architecture_requalification";
elseif level=="zero_shot", a="zero_shot_transfer_certification";
elseif level=="governor_local", a="height_governor_local_refinement";
elseif level=="local", a="local_controller_refinement";
elseif level=="broad", a="broad_controller_search";
elseif level=="unresolved", a="inspect_height_governor_or_inner_vz_tracking";
else, a="controller_learning"; end
end

function s=local_cfg_text(x)
if isfinite(double(x)), s=string(sprintf('%d',round(double(x)))); else, s="none"; end
end

function cls=local_classify_result(root,ME)
% Structured artifacts outrank exception text.  Text is used only to identify
% infrastructure failures when no trustworthy flight/controller result exists.
[physicsReady,controllersReady,~,~,missionPresent]=local_status(root);
if controllersReady && missionPresent, cls="MISSION_FAIL"; return; end
cpFile=fullfile(root,'airdropx_200m_cfg_checkpoint.mat');
if isfile(cpFile)
    try
        S=load(cpFile,'checkpoint'); cp=S.checkpoint;
        if isfield(cp,'status')
            st=lower(string(cp.status(:)));
            if any(contains(st,"plant_probe_failed")|contains(st,"trim")|contains(st,"plant_"))
                cls="PHYSICS_FAIL"; return;
            end
            if any(contains(st,"unified_learning_failed")|contains(st,"controller"))
                cls="CONTROLLER_FAIL"; return;
            end
        end
    catch
    end
end
txt=lower(string(ME.identifier)+" "+string(ME.message));
if contains(txt,"path too long")||contains(txt,"filename too long")||contains(txt,"logsout")|| ...
        contains(txt,"worker")||contains(txt,"slprj")||contains(txt,"code generation")||contains(txt,"permission denied")|| ...
        contains(txt,"infranovalidevaluation")
    cls="INFRA_INVALID"; return;
end
if controllersReady, cls="MISSION_FAIL"; elseif physicsReady, cls="CONTROLLER_FAIL"; else, cls="PHYSICS_FAIL"; end
end
function [transferN,nearEnabled,forceLocal,layeredLocal,localDet,localBO,broadBO]=local_learning_level_budget(level,opts)
level=string(level);
transferN=0; nearEnabled=false; forceLocal=false; layeredLocal=false;
localDet=0; localBO=0; broadBO=0;
switch level
    case "architecture_requal"
        % Exactly one old/current best candidate is re-certified under the new
        % v31.2 closed-loop architecture. No BO and no governor perturbation.
        transferN=1;
    case "zero_shot"
        transferN=max(0,round(double(opts.TransferSeedEvaluations)));
    case "governor_local"
        % Freeze the complete inner MPC and refine only Kp/Ki/VzMax.
        forceLocal=true; layeredLocal=true; nearEnabled=true;
        localDet=max(0,round(double(opts.GovernorPolishEvaluations)));
        localBO=max(0,round(double(opts.GovernorBayesEvaluations)));
    case "local"
        forceLocal=true; layeredLocal=true; nearEnabled=true;
        localDet=max(0,round(double(opts.LocalPolishEvaluations)));
        localBO=max(0,round(double(opts.LocalBayesEvaluations)));
    case "broad"
        broadBO=max(0,round(double(opts.BroadBayesEvaluationsPerCall)));
end
end
function r=local_next_retry(r)
r=string(r);
if r=="architecture_requal", r="governor_local";
elseif r=="zero_shot", r="governor_local";
elseif r=="governor_local", r="unresolved";
elseif r=="local", r="broad";
elseif r=="broad", r="unresolved";
else, r="governor_local"; end
end
function state=local_load_state(file,H,V)
if isfile(file)
    try
        S=load(file,'state');
        if isfield(S,'state')
            state=S.state;
            state=local_upgrade_state(state,H,V);
            return;
        end
    catch
    end
end
state=local_upgrade_state(struct(),H,V);
end
function state=local_upgrade_state(state,H,V)
def=struct('schema_version',"v31.2",'target_altitude_m',H,'target_airspeed_mps',V, ...
    'state',"NEW",'failure_class',"none",'retry_level',"zero_shot",'physics_ready',false, ...
    'active_cfg',NaN,'active_cfg_physics_ready',false,'all_physics_ready',false, ...
    'controllers_ready',false,'all_controllers_verified',false,'mission_pass',false,'gate_ratio',Inf, ...
    'learning_calls',0,'mission_attempts',0,'next_action',"physics_validation", ...
    'migration_note',"",'last_error',"",'updated_at',string(datetime('now')));
fn=fieldnames(def);
for i=1:numel(fn)
    if ~isfield(state,fn{i}), state.(fn{i})=def.(fn{i}); end
end
state.schema_version="v31.2";
state.target_altitude_m=H; state.target_airspeed_mps=V;
end
function local_save_state(file,state), save(file,'state','-v7.3'); local_write_state_csv(file,state); end
function local_write_state_csv(file,state)
T=struct2table(state,'AsArray',true); writetable(T,fullfile(fileparts(file),'mission_state.csv'));
end
function local_record_state(kbRoot,projectRoot,root,state)
C=airdropx_v31_context('TargetAltitudeM',state.target_altitude_m,'TargetAirspeedMps',state.target_airspeed_mps);
row=table("state|"+local_stable_key(string(root)+"|"+state.updated_at+"|"+state.state),state.updated_at,string(root), ...
    C.mission_signature,state.state,state.failure_class,state.retry_level,logical(state.physics_ready), ...
    double(state.active_cfg),logical(state.active_cfg_physics_ready),logical(state.all_physics_ready), ...
    logical(state.controllers_ready),logical(state.all_controllers_verified),logical(state.mission_pass),double(state.gate_ratio),state.next_action, ...
    "v31.2 active-cfg height-governor state-machine update",'VariableNames',{'record_key','timestamp','context_root','mission_signature','state', ...
    'failure_class','retry_level','physics_ready','active_cfg','active_cfg_physics_ready','all_physics_ready', ...
    'controllers_ready','all_controllers_verified','mission_pass','gate_ratio','next_action','note'});
airdropx_v31_knowledge_bank('Action','append_state','ProjectRoot',projectRoot,'Root',kbRoot,'Row',row);
end
function local_record_mission_if_present(kbRoot,projectRoot,root,opts)
f=fullfile(root,'final_mission_validation','final_mission_summary.csv'); if ~isfile(f), return; end
try, S=readtable(f,'TextType','string'); catch, return; end
if isempty(S), return; end
C=airdropx_v31_context('TargetAltitudeM',opts.TargetAltitudeM,'TargetAirspeedMps',opts.TargetAirspeedMps, ...
    'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg,'TotalDropCount',opts.TotalDropCount);
gate=local_table_num(S,'mission_gate_ratio',Inf); pass=local_table_bool(S,'mission_pass',false); hard=local_table_bool(S,'hard_fail',false);
row=table("v31_train_mission|"+local_stable_key(string(root)+"|"+string(gate)+"|"+string(pass)),string(datetime('now')), ...
    "v31.2","v31_reference_training",C.mission_signature,C.physics_signature,double(opts.TargetAltitudeM),double(opts.TargetAirspeedMps), ...
    double(opts.ReferenceMassKg),double(opts.CargoMassKg),double(opts.TotalDropCount),string(root),double(gate),logical(pass),logical(hard), ...
    local_table_num(S,'mission_h_rms_m',NaN),local_table_num(S,'mission_h_max_abs_m',NaN),local_table_num(S,'mission_h_drift_m',NaN), ...
    local_table_num(S,'mission_Va_rms_mps',NaN),local_table_num(S,'mission_vz_rms_mps',NaN),local_table_num(S,'mission_q_rms_dps',NaN), ...
    local_table_num(S,'tail_h_error_m',NaN),local_table_num(S,'tail_vz_mps',NaN),local_table_num(S,'tail_q_dps',NaN), ...
    logical(isfinite(gate)&&~hard),"v31_2_single_channel_height_governor", ...
    'VariableNames',{'record_key','timestamp','source_version','source','mission_signature','physics_signature','target_altitude_m', ...
    'target_airspeed_mps','reference_mass_kg','cargo_mass_kg','total_drop_count','output_root','mission_gate_ratio','mission_pass','hard_fail', ...
    'mission_h_rms_m','mission_h_max_abs_m','mission_h_drift_m','mission_Va_rms_mps','mission_vz_rms_mps','mission_q_rms_dps', ...
    'tail_h_error_m','tail_vz_mps','tail_q_dps','valid_for_learning','controller_state_policy'});
airdropx_v31_knowledge_bank('Action','append_mission','ProjectRoot',projectRoot,'Root',kbRoot,'Row',row);
end
function v=local_table_num(T,n,d)
if ~ismember(n,T.Properties.VariableNames), v=d; return; end
x=T.(n); if isnumeric(x)||islogical(x), v=double(x(1)); else, v=str2double(string(x(1))); end
if ~isfinite(v), v=d; end
end
function v=local_table_bool(T,n,d)
if ~ismember(n,T.Properties.VariableNames), v=logical(d); return; end
x=T.(n); if islogical(x), v=x(1); elseif isnumeric(x), v=x(1)~=0; else, q=lower(strtrim(string(x(1)))); v=q=="true"||q=="1"||q=="yes"; end
end
function R=local_result(state,root), R=state; R.output_root=string(root); end
function x=local_num(x), if isnumeric(x)||islogical(x), x=double(x); else, x=str2double(string(x)); end, end
function x=local_bool(x), if islogical(x), return; elseif isnumeric(x), x=x~=0; else, s=lower(strtrim(string(x))); x=s=="true"||s=="1"||s=="yes"; end, end
function tag=local_context_tag(H,V), tag="H"+local_num_tag(H)+"_V"+local_num_tag(V); end
function s=local_num_tag(x), s=sprintf('%.3f',double(x)); s=strrep(s,'-','m'); s=strrep(s,'.','p'); end
function k=local_stable_key(s), s=char(string(s)); z=0; for i=1:numel(s), z=mod(z*131+double(uint8(s(i))),2^31-1); end, k=string(sprintf('%08x',round(z))); end
function root=local_project_root(root), root=string(root); if strlength(root)==0, root=string(pwd); end, end
function p=local_resolve_path(projectRoot,p), p=string(p); if strlength(p)==0, return; end, c=char(p); absPath=startsWith(c,'/')||startsWith(c,'\\')||~isempty(regexp(c,'^[A-Za-z]:[\\/]','once')); if ~absPath, p=string(fullfile(projectRoot,p)); end, end
function opts=local_options(varargin)
opts=struct(); opts.ProjectRoot=""; opts.OutputRoot=""; opts.ContextRoot="matlab/results/mpc_auto_v31/reference_contexts";
opts.KnowledgeBankRoot="matlab/results/mpc_auto_v31_knowledge_bank";
opts.LegacyLearningBankRoot="matlab/results/mpc_auto_global_learning_bank"; opts.LegacyPlantBankRoot="matlab/results/mpc_auto_global_plant_bank";
opts.BaseIdentifiedMat="matlab/results/mpc_auto_id_v11_clean_r1/identify/airdropx_identified_plants.mat";
opts.TargetAltitudeM=200; opts.TargetAirspeedMps=50; opts.ReferenceMassKg=3423; opts.CargoMassKg=300; opts.TotalDropCount=4;
opts.UseParallel=true; opts.ParallelWorkers=3; opts.TransferSeedEvaluations=3; opts.LocalPolishEvaluations=6;
opts.LocalBayesEvaluations=9; opts.BroadBayesEvaluationsPerCall=12; opts.ControllerNearPassGateRatioMax=1.30;
opts.GovernorPolishEvaluations=6; opts.GovernorBayesEvaluations=6;
opts.HeightVzSlewRateMps2=0.30; opts.HeightBiasFraction=0.70; opts.HeightBiasLeak=1.0;
if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin), n=string(varargin{i}); if ~isfield(opts,n), error("Unknown option: %s",n); end, opts.(n)=varargin{i+1}; end
end
