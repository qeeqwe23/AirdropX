function result = airdropx_v31_train_envelope(varargin)
%AIRDROPX_V31_TRAIN_ENVELOPE Unified v31 learning/qualification curriculum.
%
% Key rule: target altitude is a QUALIFICATION axis, not a training axis.
% Training occurs only at ReferenceAltitudeM while speed/mass/CG/payload vary.
% Every learned speed is then frozen and checked at the qualification heights.
% A failed height qualification blocks generalization; it never launches an
% altitude-specific trim/ID/controller BO.

opts=local_options(varargin{:});
projectRoot=local_project_root(opts.ProjectRoot);
root=local_resolve_path(projectRoot,opts.OutputRoot); if ~isfolder(root), mkdir(root); end
kbRoot=local_resolve_path(projectRoot,opts.KnowledgeBankRoot);
anchorRoot=local_resolve_path(projectRoot,opts.AnchorVerifiedRoot);
addpath(fullfile(projectRoot,'matlab')); addpath(fullfile(projectRoot,'matlab','mpc')); addpath(fullfile(projectRoot,'matlab','mpc_auto'));

% One idempotent migration at every start is cheap and keeps old successful
% v29/v30 observations available without letting old generation states drive v31.
airdropx_v31_import_legacy('ProjectRoot',projectRoot,'KnowledgeBankRoot',kbRoot, ...
    'LegacyLearningBankRoot',opts.LegacyLearningBankRoot,'LegacyPlantBankRoot',opts.LegacyPlantBankRoot, ...
    'LegacyEnvelopeRoot',opts.LegacyEnvelopeRoot,'ReferenceMassKg',opts.ReferenceMassKg, ...
    'CargoMassKg',opts.CargoMassKg,'TotalDropCount',opts.TotalDropCount,'SkipIfMarkerExists',true);

statusFile=fullfile(root,'v31_curriculum.csv'); T=local_read_status(statusFile);
[T,curriculumMigrated]=local_v31_2_migrate_curriculum(T,statusFile,root);
if curriculumMigrated, fprintf('[V31.2-MIGRATE] prior v31.0/v31.1 curriculum archived; reference training reopened for new controller architecture.\n'); end
runCount=0; maxCalls=max(1,round(double(opts.MaxTaskCallsPerInvocation)));

fprintf("\n============================================================\n");
fprintf("[V31.3-ENVELOPE] static reference-speed learning + dynamic-reference scheduler architecture\n");
fprintf("[V31.3-ENVELOPE] training altitude: %.1f m ONLY\n",opts.ReferenceAltitudeM);
fprintf("[V31.3-ENVELOPE] qualification heights: %s m\n",mat2str(double(opts.QualificationHeightsM(:).')));
fprintf("[V31.3-ENVELOPE] speed curriculum: %s m/s\n",mat2str(double(opts.SpeedCurriculumMps(:).')));
fprintf("[V31.3-ENVELOPE] max expensive task calls this invocation: %d\n",maxCalls);
fprintf("============================================================\n");

while runCount < maxCalls
    [task,stage]=local_next_task(T,opts,anchorRoot,root);
    if task.kind=="none", break; end
    fprintf("[V31.3-ENVELOPE] NEXT stage=%s kind=%s H=%.3f V=%.3f\n",stage,task.kind,task.H,task.V);
    try
        if task.kind=="qualify"
            qRoot=fullfile(root,'qualification',local_context_tag(task.H,task.V));
            R=airdropx_v31_qualify_height('ProjectRoot',projectRoot,'SourceVerifiedRoot',task.source_root, ...
                'OutputRoot',qRoot,'KnowledgeBankRoot',kbRoot,'TargetAltitudeM',task.H, ...
                'TargetAirspeedMps',task.V,'ReferenceMassKg',opts.ReferenceMassKg, ...
                'CargoMassKg',opts.CargoMassKg,'TotalDropCount',opts.TotalDropCount, ...
                'HeightVzSlewRateMps2',opts.HeightVzSlewRateMps2,'HeightBiasFraction',opts.HeightBiasFraction,'HeightBiasLeak',opts.HeightBiasLeak);
            row=local_task_row(task,stage,string(qRoot),true,logical(R.mission_pass),double(R.mission_gate_ratio), ...
                logical(R.mission_pass),"",string(task.source_root));
        else
            cRoot=fullfile(root,'reference_contexts',local_context_tag(opts.ReferenceAltitudeM,task.V));
            R=airdropx_v31_run_context('ProjectRoot',projectRoot,'OutputRoot',cRoot,'KnowledgeBankRoot',kbRoot, ...
                'LegacyLearningBankRoot',opts.LegacyLearningBankRoot,'LegacyPlantBankRoot',opts.LegacyPlantBankRoot, ...
                'BaseIdentifiedMat',opts.BaseIdentifiedMat,'TargetAltitudeM',opts.ReferenceAltitudeM, ...
                'TargetAirspeedMps',task.V,'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg, ...
                'TotalDropCount',opts.TotalDropCount,'UseParallel',opts.UseParallel,'ParallelWorkers',opts.ParallelWorkers, ...
                'TransferSeedEvaluations',opts.TransferSeedEvaluations,'LocalPolishEvaluations',opts.LocalPolishEvaluations, ...
                'LocalBayesEvaluations',opts.LocalBayesEvaluations,'BroadBayesEvaluationsPerCall',opts.BroadBayesEvaluationsPerCall, ...
                'GovernorPolishEvaluations',opts.GovernorPolishEvaluations,'GovernorBayesEvaluations',opts.GovernorBayesEvaluations, ...
                'HeightVzSlewRateMps2',opts.HeightVzSlewRateMps2,'HeightBiasFraction',opts.HeightBiasFraction,'HeightBiasLeak',opts.HeightBiasLeak);
            verified=string(R.state)=="VERIFIED";
            terminal=logical(verified)||string(R.state)=="BLOCKED";
            row=local_task_row(task,stage,string(cRoot),terminal,logical(verified),double(R.gate_ratio),logical(R.mission_pass), ...
                string(R.failure_class),string(cRoot));
        end
    catch ME
        row=local_task_row(task,stage,"",false,false,Inf,false,"runtime:"+string(ME.identifier),string(task.source_root));
        fid=fopen(fullfile(root,'v31_last_error.txt'),'w'); if fid>=0, fprintf(fid,'%s\n',getReport(ME,'extended','hyperlinks','off')); fclose(fid); end
    end
    T=local_upsert(T,row); writetable(T,statusFile);
    runCount=runCount+1;
end

% v31.3 keeps an always-refreshable scheduler artifact beside the curriculum.
% It is read-only with respect to learning: only fully VERIFIED reference-speed
% contexts become nodes, and runtime interpolation activates after >=2 nodes.
try
    airdropx_v31_3_build_scheduler_bank('ProjectRoot',projectRoot, ...
        'ContextRoot',fullfile(root,'reference_contexts'), ...
        'OutputFile',fullfile(root,'v31_3_speed_scheduler','v31_3_speed_scheduler_bank.mat'), ...
        'ReferenceAltitudeM',opts.ReferenceAltitudeM);
catch ME
    fprintf('[V31.3-SCHED] scheduler refresh skipped: %s\n',ME.message);
end

info=local_summary(T,opts);
local_write_envelope_status(fullfile(root,'v31_envelope_status.csv'),info,runCount);
if info.training_complete
    fid=fopen(fullfile(root,'V31_ENVELOPE_TRAINING_COMPLETE.txt'),'w'); if fid>=0, fprintf(fid,'completed=%s\n',char(datetime('now'))); fclose(fid); end
end
result=info; result.curriculum=T; result.output_root=string(root); result.knowledge_bank_root=string(kbRoot);
end

function [T,migrated]=local_v31_2_migrate_curriculum(T,statusFile,root)
migrated=false;
marker=fullfile(root,'V31_2_CURRICULUM_MIGRATED.txt');
if isfile(marker), return; end
try
    if ~isempty(T)
        backup=fullfile(root,'v31_curriculum_pre_v31_2.csv');
        if ~isfile(backup), writetable(T,backup); end
        % Every controller/qualification result depends on the old controller
        % architecture. Keep the backup for audit and reopen curriculum cleanly.
        T=T([],:);
        writetable(T,statusFile);
    end
    fid=fopen(marker,'w'); if fid>=0, fprintf(fid,'migrated=%s\n',char(datetime('now'))); fclose(fid); end
    migrated=true;
catch ME
    warning('AirdropX:V31_2:CurriculumMigration','Could not migrate curriculum: %s',ME.message);
end
end

function [task,stage]=local_next_task(T,opts,anchorRoot,root)
task=local_none(); stage="complete";
Hq=unique(double(opts.QualificationHeightsM(:)),'stable');
V0=double(opts.AnchorAirspeedMps); H0=double(opts.ReferenceAltitudeM);
% v31.2 changes the controller architecture itself (effective Wh=0 plus a
% new governor state), so a frozen legacy bank is no longer an authoritative
% anchor test. Always establish the canonical v31.2 policy through the fixed
% reference-context state machine, which preserves old parameters as A/B seeds
% but rebuilds/re-certifies the controller objects under the new architecture.
if ~local_task_pass(T,"anchor_train",H0,V0)
    if local_task_fail(T,"anchor_train",H0,V0)
        stage="REFERENCE_POLICY_BLOCKED"; return;
    end
    task=local_task("train",H0,V0,"","anchor_train"); stage="anchor_reference_training"; return;
end
anchorSource=local_anchor_source(T,opts,anchorRoot,root);
% Stage 1: altitude translation invariance at anchor speed. No learning.
for H=Hq(:).'
    if abs(H-H0)<1e-9, continue; end
    if ~local_task_pass(T,"height_qual",H,V0)
        if local_task_fail(T,"height_qual",H,V0)
            stage="HEIGHT_GENERALIZATION_BLOCKED"; return;
        end
        task=local_task("qualify",H,V0,anchorSource,"height_qual"); stage="height_translation_qualification"; return;
    end
end
% Stage 2: learn new SPEED contexts only at the reference altitude.
for V=double(opts.SpeedCurriculumMps(:).')
    if abs(V-V0)<1e-9, continue; end
    if local_speed_beyond_failed_boundary(T,V,V0,H0)
        continue;
    end
    if ~local_task_pass(T,"speed_train",H0,V)
        if local_task_fail(T,"speed_train",H0,V)
            % Bounded physics/controller learning exhausted at the reference
            % altitude. Treat this as a speed-envelope boundary observation;
            % never compensate by learning a special controller at another H.
            continue;
        end
        task=local_task("train",H0,V,"","speed_train"); stage="speed_reference_training"; return;
    end
    source=fullfile(root,'reference_contexts',local_context_tag(H0,V));
    % Stage 3: freeze the learned speed and qualify heights. Any fail blocks
    % this speed; it NEVER launches H-specific training.
    blocked=false;
    for H=Hq(:).'
        if abs(H-H0)<1e-9, continue; end
        if local_task_fail(T,"speed_height_qual",H,V), blocked=true; break; end
        if ~local_task_pass(T,"speed_height_qual",H,V)
            task=local_task("qualify",H,V,source,"speed_height_qual"); stage="speed_height_qualification"; return;
        end
    end
    if blocked
        % Continue to the other side of the speed curriculum; a failed speed
        % is evidence for the common envelope boundary, not a reason to train H.
        continue;
    end
end
end
function source=local_anchor_source(T,opts,legacyAnchorRoot,root)
H0=double(opts.ReferenceAltitudeM); V0=double(opts.AnchorAirspeedMps);
if local_task_pass(T,"anchor_train",H0,V0)
    source=fullfile(root,'reference_contexts',local_context_tag(H0,V0));
else
    source=legacyAnchorRoot;
end
end
function tf=local_speed_beyond_failed_boundary(T,V,V0,H0)
% Once a closer speed on one side of the anchor is a terminal real failure,
% farther speeds on that side are outside the currently demonstrated common
% envelope and are skipped. This prevents wasting runs beyond a known boundary.
tf=false; if isempty(T), return; end
failed=[];
for i=1:height(T)
    if logical(T.completed(i)) && ~logical(T.pass(i))
        role=string(T.role(i)); vv=double(T.target_airspeed_mps(i)); hh=double(T.target_altitude_m(i));
        if role=="speed_train" && abs(hh-H0)<1e-8
            failed(end+1)=vv; %#ok<AGROW>
        elseif role=="speed_height_qual"
            failed(end+1)=vv; %#ok<AGROW>
        end
    end
end
failed=failed(isfinite(failed));
if V<V0
    low=failed(failed<V0); if ~isempty(low), tf=V<=max(low)+1e-9; end
elseif V>V0
    high=failed(failed>V0); if ~isempty(high), tf=V>=min(high)-1e-9; end
end
end
function task=local_task(kind,H,V,source,role), task=struct('kind',string(kind),'H',double(H),'V',double(V),'source_root',string(source),'role',string(role)); end
function task=local_none(), task=local_task("none",NaN,NaN,"",""); end
function tf=local_task_pass(T,role,H,V), [found,row]=local_find(T,role,H,V); tf=found&&logical(row.pass(1)); end
function tf=local_task_fail(T,role,H,V), [found,row]=local_find(T,role,H,V); tf=found&&logical(row.completed(1))&&~logical(row.pass(1)); end
function [found,row]=local_find(T,role,H,V)
found=false; row=table(); if isempty(T), return; end
idx=find(string(T.role)==string(role)&abs(double(T.target_altitude_m)-H)<1e-8&abs(double(T.target_airspeed_mps)-V)<1e-8,1,'last');
if ~isempty(idx), found=true; row=T(idx,:); end
end
function row=local_task_row(task,stage,outputRoot,completed,pass,gate,missionPass,failureClass,sourceRoot)
row=table(string(datetime('now')),task.role,string(stage),task.kind,task.H,task.V,logical(completed),logical(pass), ...
    double(gate),logical(missionPass),string(failureClass),string(outputRoot),string(sourceRoot), ...
    'VariableNames',local_names());
end
function T=local_upsert(T,row)
if isempty(T), T=row; return; end
idx=find(string(T.role)==string(row.role(1))&abs(double(T.target_altitude_m)-double(row.target_altitude_m(1)))<1e-8& ...
    abs(double(T.target_airspeed_mps)-double(row.target_airspeed_mps(1)))<1e-8,1,'last');
if isempty(idx), T=[T;row]; else, T(idx,:)=row; end
end
function info=local_summary(T,opts)
info=struct(); info.stage="NEW"; info.training_complete=false; info.anchor_v31_pass=false; info.height_generalization_pass=false;
Hq=unique(double(opts.QualificationHeightsM(:)),'stable'); H0=double(opts.ReferenceAltitudeM); V0=double(opts.AnchorAirspeedMps);
info.anchor_v31_pass=local_task_pass(T,"anchor_train",H0,V0);
if ~info.anchor_v31_pass
    if local_task_fail(T,"anchor_train",H0,V0), info.stage="REFERENCE_POLICY_BLOCKED"; else, info.stage="anchor_reference_training"; end
    return;
end
hPass=true; hBlocked=false;
for H=Hq(:).'
    if abs(H-H0)<1e-9, continue; end
    hPass=hPass&&local_task_pass(T,"height_qual",H,V0); hBlocked=hBlocked||local_task_fail(T,"height_qual",H,V0);
end
info.height_generalization_pass=hPass;
if hBlocked, info.stage="HEIGHT_GENERALIZATION_BLOCKED"; return; end
if ~hPass, info.stage="height_translation_qualification"; return; end
Vgood=V0; Vbad=[];
allProcessed=true;
for V=double(opts.SpeedCurriculumMps(:).')
    if abs(V-V0)<1e-9, continue; end
    if local_speed_beyond_failed_boundary(T,V,V0,H0)
        continue;
    end
    if ~local_task_pass(T,"speed_train",H0,V)
        if local_task_fail(T,"speed_train",H0,V), Vbad(end+1)=V; else, allProcessed=false; end %#ok<AGROW>
        continue;
    end
    qPass=true; qFail=false;
    for H=Hq(:).'
        if abs(H-H0)<1e-9, continue; end
        qPass=qPass&&local_task_pass(T,"speed_height_qual",H,V); qFail=qFail||local_task_fail(T,"speed_height_qual",H,V);
    end
    if qPass, Vgood(end+1)=V; elseif qFail, Vbad(end+1)=V; else, allProcessed=false; end %#ok<AGROW>
end
info.qualified_speed_min_mps=min(Vgood); info.qualified_speed_max_mps=max(Vgood); info.failed_speed_contexts=Vbad;
if allProcessed, info.training_complete=true; info.stage="QUALIFIED_CURRICULUM_COMPLETE"; else, info.stage="speed_curriculum"; end
end
function local_write_envelope_status(file,info,calls)
T=table(string(datetime('now')),string(info.stage),logical(info.anchor_v31_pass),logical(info.height_generalization_pass), ...
    double(local_field(info,'qualified_speed_min_mps',NaN)),double(local_field(info,'qualified_speed_max_mps',NaN)), ...
    logical(info.training_complete),double(calls),'VariableNames',{'updated_at','stage','anchor_v31_pass', ...
    'height_generalization_pass','qualified_speed_min_mps','qualified_speed_max_mps','training_complete','task_calls_this_invocation'});
writetable(T,file);
end
function v=local_field(S,n,d), if isfield(S,n), v=S.(n); else, v=d; end, end
function T=local_read_status(file)
if ~isfile(file), T=local_empty(); return; end
try, T=readtable(file,'TextType','string'); catch, T=readtable(file); end
end
function T=local_empty()
T=table(strings(0,1),strings(0,1),strings(0,1),strings(0,1),zeros(0,1),zeros(0,1),false(0,1),false(0,1), ...
    zeros(0,1),false(0,1),strings(0,1),strings(0,1),strings(0,1),'VariableNames',local_names());
end
function n=local_names(), n={'updated_at','role','stage','kind','target_altitude_m','target_airspeed_mps','completed','pass','gate_ratio','mission_pass','failure_class','output_root','source_root'}; end
function tag=local_context_tag(H,V), tag="H"+local_num_tag(H)+"_V"+local_num_tag(V); end
function s=local_num_tag(x), s=sprintf('%.3f',double(x)); s=strrep(s,'-','m'); s=strrep(s,'.','p'); end
function root=local_project_root(root), root=string(root); if strlength(root)==0, root=string(pwd); end, end
function p=local_resolve_path(projectRoot,p), p=string(p); if strlength(p)==0, return; end, c=char(p); absPath=startsWith(c,'/')||startsWith(c,'\\')||~isempty(regexp(c,'^[A-Za-z]:[\\/]','once')); if ~absPath, p=string(fullfile(projectRoot,p)); end, end
function opts=local_options(varargin)
opts=struct(); opts.ProjectRoot=""; opts.OutputRoot="matlab/results/mpc_auto_v31";
opts.KnowledgeBankRoot="matlab/results/mpc_auto_v31_knowledge_bank";
opts.AnchorVerifiedRoot="matlab/results/mpc_auto_200m_all_cfg_v16"; opts.AnchorAirspeedMps=50; opts.ReferenceAltitudeM=200;
opts.QualificationHeightsM=[200;150;100;50;20]; opts.SpeedCurriculumMps=[45;55;40;60;35;65;30;70];
opts.LegacyLearningBankRoot="matlab/results/mpc_auto_global_learning_bank"; opts.LegacyPlantBankRoot="matlab/results/mpc_auto_global_plant_bank";
opts.LegacyEnvelopeRoot="matlab/results/mpc_auto_flight_envelope_v30";
opts.BaseIdentifiedMat="matlab/results/mpc_auto_id_v11_clean_r1/identify/airdropx_identified_plants.mat";
opts.ReferenceMassKg=3423; opts.CargoMassKg=300; opts.TotalDropCount=4; opts.UseParallel=true; opts.ParallelWorkers=3;
opts.MaxTaskCallsPerInvocation=1; opts.TransferSeedEvaluations=3; opts.LocalPolishEvaluations=6; opts.LocalBayesEvaluations=9; opts.BroadBayesEvaluationsPerCall=12;
opts.GovernorPolishEvaluations=6; opts.GovernorBayesEvaluations=6;
opts.HeightVzSlewRateMps2=0.30; opts.HeightBiasFraction=0.70; opts.HeightBiasLeak=1.0;
if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin), n=string(varargin{i}); if ~isfield(opts,n), error("Unknown option: %s",n); end, opts.(n)=varargin{i+1}; end
end
