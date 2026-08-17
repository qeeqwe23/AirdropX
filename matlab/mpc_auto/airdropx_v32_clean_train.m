function result=airdropx_v32_clean_train(varargin)
%AIRDROPX_V32_CLEAN_TRAIN Persistent clean-slate automatic MPC learning (v32.1.5).
%
% Memory policy:
%   * v29/v30/v31 results are NEVER imported.
%   * The first v32 invocation learns from zero.
%   * Subsequent invocations reuse ONLY v32-native physics/controller/governor
%     checkpoints and continue from the last incomplete stage.
%   * A fully VERIFIED controller is revalidated exactly once on the next
%     invocation. If that revalidation passes, later invocations load and skip
%     retraining by default.
%   * ResetLearning=true is the only path that erases v32 learning memory.
opts=local_options(varargin{:});
if logical(opts.SuppressFigures)
    try
        set(groot,'defaultFigureVisible','off');
        close all force;
    catch
    end
end
root=local_root(opts.ProjectRoot);out=local_resolve(root,opts.OutputRoot);
% Ensure the sibling v32 helpers are resolvable even when this function was
% invoked through an unusual path rather than the standard PowerShell runner.
addpath(fullfile(root,'matlab','mpc_auto'));
% v32.1.4 explicitly separates the 20-D controller context mass
% (dry aircraft + remaining payload) from JSBSim's actual dynamics mass,
% which also contains XML fuel. The precompiled MEX interface is left
% untouched; this metadata/fingerprint is used for certificate invalidation.
opts.PhysicsContext=airdropx_v32_physics_context(root,opts.ReferenceMassKg);
if logical(opts.ResetLearning) && isfolder(out)
    fprintf('[V32.1.5-MEMORY] explicit ResetLearning=true -> removing ONLY %s\n',out);
    rmdir(out,'s');
end
if ~isfolder(out),mkdir(out);end
local_write_physics_context(out,opts.PhysicsContext);
knowledgeRoot=fullfile(out,'knowledge_bank');checkpointRoot=fullfile(out,'checkpoints');runRootBase=fullfile(out,'runs');
for p={knowledgeRoot,checkpointRoot,runRootBase},if ~isfolder(p{1}),mkdir(p{1});end,end
runId=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));runRoot=fullfile(runRootBase,['run_' runId]);mkdir(runRoot);
stateFile=fullfile(checkpointRoot,'v32_memory_state.mat');
state=local_load_state(stateFile);
state.run_count=state.run_count+1;state.last_run_id=string(runId);state.last_updated=datetime('now');
local_save_state(stateFile,state);
local_memory_policy(out,opts,state);
local_status(out,'INITIALIZING',0,Inf,sprintf('persistent v32 memory; run=%s reset=%d',runId,logical(opts.ResetLearning)));
local_write_run_info(runRoot,opts,state);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));
airdropx_v32_setup_model('ProjectRoot',root,'ModelName',opts.Model);
local_parallel(opts.Workers,opts.ShortFileGenRoot);

try
    speeds=unique(double(opts.SpeedNodesMps(:)),'stable');
    if ~any(abs(speeds-opts.AnchorSpeedMps)<1e-9),speeds=[opts.AnchorSpeedMps;speeds(:)];end
    [~,anchorIdx]=min(abs(speeds-opts.AnchorSpeedMps));order=[anchorIdx,setdiff(1:numel(speeds),anchorIdx,'stable')];
    controllerSig=local_controller_signature(opts,speeds);
    if strlength(state.controller_signature)>0 && state.controller_signature~=controllerSig
        fprintf('[V32.1.5-MEMORY] controller context changed. Physics nodes are checked individually; controller/governor/final certificates marked stale.\n');
        state.inner_verified=false;state.governor_verified=false;state.final_verified=false;state.final_revalidated=false;
        state.inner_checkpoint="";state.governor_checkpoint="";state.final_controller="";state.mission_best_gate=Inf;state.mission_best_params=struct();
        local_delete_if_exists(fullfile(knowledgeRoot,'controller','inner_learning','inner_resume_checkpoint.mat'));
        local_delete_if_exists(fullfile(knowledgeRoot,'governor','governor_learning','governor_resume_checkpoint.mat'));
        local_delete_if_exists(fullfile(out,'V32_VERIFIED.txt'));
    end
    state.controller_signature=controllerSig;local_save_state(stateFile,state);

    % Fast path for a previously VERIFIED controller. Revalidate exactly once.
    if state.final_verified && strlength(state.final_controller)>0 && isfile(state.final_controller)
        F=load(char(state.final_controller));
        if isfield(F,'final')
            final=F.final;
            if logical(opts.RevalidateVerifiedOnce) && ~state.final_revalidated
                local_status(out,'VERIFIED_REVALIDATION',0,double(final.mission.gate_ratio),'one-time revalidation of stored v32 controller');
                revalRoot=fullfile(runRoot,'verified_revalidation');
                reval=local_run_mission(root,final.inner.bank_mat,final.hidden_elevator_trim,final.governor.params,opts,revalRoot);
                state.final_revalidation_attempted=true;state.last_gate_ratio=reval.gate_ratio;
                if reval.pass
                    state.final_revalidated=true;state.last_stage="VERIFIED_REVALIDATED";state.last_updated=datetime('now');local_save_state(stateFile,state);
                    final.mission=reval;save(char(state.final_controller),'final','-v7.3');
                    local_status(out,'VERIFIED_REVALIDATED',1,reval.gate_ratio,'one-time revalidation PASS; future starts skip retraining');
                    result=final;return;
                end
                fprintf('[V32.1.5-MEMORY] stored VERIFIED controller failed one-time revalidation gate=%.3f -> reopening from v32-native knowledge.\n',reval.gate_ratio);
                state.final_verified=false;state.final_revalidated=false;state.last_stage="REOPEN_AFTER_REVALIDATION_FAIL";local_delete_if_exists(fullfile(out,'V32_VERIFIED.txt'));local_save_state(stateFile,state);
            elseif state.final_revalidated || ~logical(opts.RevalidateVerifiedOnce)
                local_status(out,'MEMORY_REUSE_VERIFIED',1,double(final.mission.gate_ratio),'stored VERIFIED controller loaded; retraining skipped');
                result=final;return;
            end
        end
    end

    % -------- Persistent physics memory: each speed node is independently reusable.
    trimBanks=cell(numel(speeds),1);idMats=strings(numel(speeds),1);hiddenTrim=NaN;
    physicsRoot=fullfile(knowledgeRoot,'physics');if ~isfolder(physicsRoot),mkdir(physicsRoot);end
    for oo=1:numel(order)
        n=order(oo);v=speeds(n);nodeRoot=fullfile(physicsRoot,sprintf('V%06.3f',v));if ~isfolder(nodeRoot),mkdir(nodeRoot);end
        physicsCk=fullfile(nodeRoot,'v32_physics_verified.mat');nodeSig=local_physics_signature(opts,v);
        reused=false;oldPhysics=struct();
        if isfile(physicsCk)
            S=load(physicsCk);
            if isfield(S,'physics')
                if isfield(S.physics,'signature') && string(S.physics.signature)==nodeSig && isfield(S.physics,'identified_mat') && isfile(S.physics.identified_mat)
                    trimBanks{n}=local_v32_ensure_trim_fields(S.physics.trim_bank);idMats(n)=string(S.physics.identified_mat);reused=true;
                    fprintf('[V32.1.5-MEMORY] physics V=%.1f reused from exact VERIFIED certificate.\n',v);
                    local_status(out,sprintf('PHYSICS_REUSE_V%.1f',v),1,0,'v32.1.5 physics signature exact; skipped');
                elseif isfield(S.physics,'trim_bank') && isfield(S.physics,'identified_mat') && isfile(S.physics.identified_mat)
                    oldPhysics=S.physics;
                    fprintf('[V32.1.5-MIGRATE] physics V=%.1f certificate is stale under the new solver/physics signature. Revalidating trims once; old ID retained as a migration candidate.\n',v);
                end
            end
        end
        if reused,continue;end
        local_status(out,sprintf('TRIM_V%.1f',v),0,Inf,sprintf('v32.1.5 physics revalidation/learning node %d/%d',oo,numel(order)));
        % If an older v32 Physics certificate exists, its trim bank is a prior,
        % not a certificate. Each cfg checkpoint is revalidated once by
        % find_trim. This preserves useful experience without trusting stale
        % physics semantics.
        if ~isempty(fieldnames(oldPhysics)) && isfield(oldPhysics,'trim_bank')
            bank=local_v32_ensure_trim_fields(oldPhysics.trim_bank);
        else
            bank=airdropx_auto_default_trim_bank('TargetAltitudeM',opts.ReferenceAltitudeM,'TargetAirspeedMps',v);
            bank=local_v32_ensure_trim_fields(bank);
        end
        for cfg=0:4
            cfgRoot=fullfile(nodeRoot,'trim',sprintf('cfg%d',cfg));if ~isfolder(cfgRoot),mkdir(cfgRoot);end
            rr=local_find_trim_clean(root,cfgRoot,bank,cfg,v,opts);
            bank=local_v32_merge_trim_entry(bank,rr.trim_bank(cfg+1),cfg+1);
        end
        % v32.1.5: a trim is not considered usable for identification merely
        % because the shorter trim gate passed. Re-run every cfg with the exact
        % ID preparation schedule and a long zero-excitation baseline. Only
        % the cfgs that fail this ID-readiness gate are polished again.
        [bank,readinessChanged]=local_certify_trim_bank_id_ready(root,out,nodeRoot,bank,v,opts); %#ok<NASGU>
        trimBanks{n}=bank;save(fullfile(nodeRoot,'v32_trim_bank.mat'),'bank','-v7.3');

        % Preserve an old identified plant only when all revalidated trims are
        % materially unchanged. Otherwise regenerate excitation data and ID.
        canReuseOldId=false;
        if ~isempty(fieldnames(oldPhysics)) && isfield(oldPhysics,'trim_bank') && isfield(oldPhysics,'identified_mat') && isfile(oldPhysics.identified_mat)
            canReuseOldId=local_trim_banks_equivalent(bank,oldPhysics.trim_bank,opts);
        end
        if canReuseOldId
            idMat=char(string(oldPhysics.identified_mat));
            fprintf('[V32.1.5-MIGRATE] V=%.1f all trims revalidated and equivalent -> reusing existing identified plant.\n',v);
            rep=airdropx_auto_validate_models('IdentifiedMat',idMat,'OutputFile',fullfile(nodeRoot,'model_validation_v32_1_5.csv'),'PredictionSteps',[5 10 20 30]);
            local_model_quality_guard(rep.table,v,opts);idMats(n)=string(idMat);
            idReused=true;
        else
            local_status(out,sprintf('ID_DATA_V%.1f',v),0,Inf,'trim changed/new; generating v32.1.5 clean excitation data');
            dataRoot=fullfile(nodeRoot,'id_data');
            % Never mix excitation runs collected around an older operating
            % point with a materially changed trim. Historical files remain
            % represented by the old Physics certificate, but the active ID
            % dataset for this node is rebuilt atomically.
            try,if isfolder(dataRoot),rmdir(dataRoot,'s');end,catch,end
            bank=local_generate_id_data_with_recovery(root,out,nodeRoot,dataRoot,bank,v,opts);
            trimBanks{n}=bank;save(fullfile(nodeRoot,'v32_trim_bank.mat'),'bank','-v7.3');
            idDataMat=fullfile(nodeRoot,'iddata.mat');
            airdropx_auto_build_iddata('InputRoot',dataRoot,'OutputMat',idDataMat,'TrimBank',bank,'Ts',0.1,'TargetAltitudeM',opts.ReferenceAltitudeM,'TargetAirspeedMps',v,...
                'UseVerifiedTrimForNominal',true,'RejectBadBaseline',true,'RequirePreExcitationBaseline',true,'MinAcceptedRunsPerConfig',opts.MinCleanIdRuns);
            idMat=fullfile(nodeRoot,'identified.mat');
            airdropx_auto_identify('DataMat',idDataMat,'OutputMat',idMat,'Orders',opts.IdOrders,'ValidationSteps',[5 10 20 30],'EnforceStability',true,'RefineWithSsest',false);
            rep=airdropx_auto_validate_models('IdentifiedMat',idMat,'OutputFile',fullfile(nodeRoot,'model_validation.csv'),'PredictionSteps',[5 10 20 30]);
            local_model_quality_guard(rep.table,v,opts);idMats(n)=string(idMat);
            idReused=false;
        end
        physics=struct('version','v32.1.5','signature',nodeSig,'speed_mps',v,'trim_bank',bank,'identified_mat',string(idMat),...
            'verified_at',datetime('now'),'legacy_data_used',false,'migrated_id_reuse',logical(idReused),...
            'physics_context',opts.PhysicsContext,'trim_solver_version','joint_equilibrium_idready_v32_1_5'); %#ok<NASGU>
        save(physicsCk,'physics','-v7.3');
        state.physics_nodes_verified=unique([double(state.physics_nodes_verified(:));v]);state.last_stage=string(sprintf('PHYSICS_V%.1f_VERIFIED',v));local_save_state(stateFile,state);
    end

    % Hidden physical-elevator offset is also persistent and context checked.
    hiddenFile=fullfile(knowledgeRoot,'hidden_elevator_trim.mat');hiddenSig=local_hidden_signature(opts);
    if isfile(hiddenFile)
        H=load(hiddenFile);if isfield(H,'hidden')&&isfield(H.hidden,'signature')&&string(H.hidden.signature)==hiddenSig&&isfinite(H.hidden.value),hiddenTrim=H.hidden.value;end
    end
    if ~isfinite(hiddenTrim)
        bank=trimBanks{anchorIdx};hiddenTrim=local_calibrate_hidden_trim(root,bank(1),opts,fullfile(knowledgeRoot,'hidden_trim_calibration'));
        hidden=struct('value',hiddenTrim,'signature',hiddenSig,'verified_at',datetime('now')); %#ok<NASGU>
        save(hiddenFile,'hidden','-v7.3');
    end
    writetable(table(hiddenTrim,'VariableNames',{'hidden_elevator_trim'}),fullfile(knowledgeRoot,'hidden_elevator_trim.csv'));

    % -------- Persistent inner-MPC certificate.
    controllerRoot=fullfile(knowledgeRoot,'controller');if ~isfolder(controllerRoot),mkdir(controllerRoot);end
    innerFile=fullfile(controllerRoot,'v32_inner_verified.mat');inner=[];
    if state.inner_verified && strlength(state.inner_checkpoint)>0 && isfile(state.inner_checkpoint)
        S=load(char(state.inner_checkpoint));if isfield(S,'inner')&&isfield(S,'signature')&&string(S.signature)==controllerSig,inner=S.inner;end
    elseif isfile(innerFile)
        S=load(innerFile);if isfield(S,'inner')&&isfield(S,'signature')&&string(S.signature)==controllerSig,inner=S.inner;state.inner_verified=true;state.inner_checkpoint=string(innerFile);end
    end
    if isempty(inner)
        local_status(out,'INNER_MPC_LEARNING',0,Inf,'direct Va/vz certification; v32-native resume memory enabled');
        inner=airdropx_v32_tune_inner('ProjectRoot',root,'OutputRoot',fullfile(controllerRoot,'inner_learning'),'IdentifiedMats',idMats,'SpeedNodesMps',speeds,'ConfigIds',(0:4).',...
            'ReferenceAltitudeM',opts.ReferenceAltitudeM,'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg,'HiddenElevatorTrim',hiddenTrim,'BayesEvaluations',opts.InnerBayesEvaluations,'UseParallel',logical(opts.UseParallel),'ShortFileGenRoot',fullfile(opts.ShortFileGenRoot,'inner'));
        signature=controllerSig;save(innerFile,'inner','signature','-v7.3');state.inner_verified=true;state.inner_checkpoint=string(innerFile);state.last_stage="INNER_VERIFIED";local_save_state(stateFile,state);
    else
        local_status(out,'INNER_MPC_REUSE',1,inner.gate_ratio,'stored v32 inner certificate reused');
    end

    % -------- Persistent governor certificate.
    governorRoot=fullfile(knowledgeRoot,'governor');if ~isfolder(governorRoot),mkdir(governorRoot);end
    govFile=fullfile(governorRoot,'v32_governor_verified.mat');gov=[];
    if state.governor_verified && strlength(state.governor_checkpoint)>0 && isfile(state.governor_checkpoint)
        S=load(char(state.governor_checkpoint));if isfield(S,'gov')&&isfield(S,'signature')&&string(S.signature)==controllerSig,gov=S.gov;end
    elseif isfile(govFile)
        S=load(govFile);if isfield(S,'gov')&&isfield(S,'signature')&&string(S.signature)==controllerSig,gov=S.gov;state.governor_verified=true;state.governor_checkpoint=string(govFile);end
    end
    if isempty(gov)
        local_status(out,'HEIGHT_GOVERNOR_LEARNING',1,inner.gate_ratio,'inner certified; v32-native governor resume memory enabled');
        gov=airdropx_v32_tune_governor('ProjectRoot',root,'BankMat',inner.bank_mat,'OutputRoot',fullfile(governorRoot,'governor_learning'),'ConfigIds',(0:4).',...
            'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg,'HiddenElevatorTrim',hiddenTrim,'BayesEvaluations',opts.GovernorBayesEvaluations,'UseParallel',logical(opts.UseParallel),'ShortFileGenRoot',fullfile(opts.ShortFileGenRoot,'gov'));
        signature=controllerSig;save(govFile,'gov','signature','-v7.3');state.governor_verified=true;state.governor_checkpoint=string(govFile);state.last_stage="GOVERNOR_VERIFIED";local_save_state(stateFile,state);
    else
        local_status(out,'GOVERNOR_REUSE',1,gov.gate_ratio,'stored v32 governor certificate reused');
    end

    % -------- Final mission remembers best v32-native governor candidate across starts.
    missionGov=gov;
    if isfield(state,'mission_best_params') && ~isempty(fieldnames(state.mission_best_params)) && isfinite(state.mission_best_gate)
        missionGov.params=state.mission_best_params;
        fprintf('[V32.1.5-MEMORY] reusing mission best-so-far governor gate=%.3f before new refinement.\n',state.mission_best_gate);
    end
    local_status(out,'FINAL_DYNAMIC_MISSION',1,missionGov.gate_ratio,'dynamic H/V + four drops');
    finalRoot=fullfile(runRoot,'final_dynamic_mission');mission=local_run_mission(root,inner.bank_mat,hiddenTrim,missionGov.params,opts,finalRoot);
    if mission.gate_ratio<state.mission_best_gate
        state.mission_best_gate=mission.gate_ratio;state.mission_best_params=missionGov.params;local_save_state(stateFile,state);
    end
    if ~mission.pass && mission.gate_ratio<=opts.MissionNearPassGate
        local_status(out,'MISSION_LOCAL_REFINEMENT',0,mission.gate_ratio,'bounded governor polish; v32 best-so-far retained across starts');
        [mission,missionGov]=local_mission_refine(root,inner.bank_mat,hiddenTrim,missionGov,opts,runRoot,mission);
        if mission.gate_ratio<state.mission_best_gate
            state.mission_best_gate=mission.gate_ratio;state.mission_best_params=missionGov.params;local_save_state(stateFile,state);
        end
    end
    if ~mission.pass
        state.last_stage="BLOCKED";state.last_gate_ratio=mission.gate_ratio;state.last_error="bounded v32 learning exhausted; memory retained";local_save_state(stateFile,state);
        local_status(out,'BLOCKED',0,mission.gate_ratio,'bounded learning exhausted; all v32 knowledge retained for next start');
        error('AirdropX:V32:MissionBlocked','v32 persistent learning completed this invocation but final dynamic mission gate=%.3f',mission.gate_ratio);
    end

    final=struct('version','v32.1.5_joint_equilibrium_idready_persistent_memory','timestamp',datetime('now'),'speed_nodes_mps',speeds,'identified_mats',idMats,'inner',inner,'governor',missionGov,'mission',mission,'hidden_elevator_trim',hiddenTrim,'physics_context',opts.PhysicsContext,...
        'legacy_data_used',false,'reference_altitude_m',opts.ReferenceAltitudeM,'speed_envelope_mps',[min(speeds) max(speeds)],'memory_policy','v32-only persistent');
    finalDir=fullfile(knowledgeRoot,'verified');if ~isfolder(finalDir),mkdir(finalDir);end
    finalFile=fullfile(finalDir,'v32_final_controller.mat');save(finalFile,'final','-v7.3');
    state.final_verified=true;state.final_revalidated=false;state.final_revalidation_attempted=false;state.final_controller=string(finalFile);state.last_stage="VERIFIED";state.last_gate_ratio=mission.gate_ratio;state.last_error="";state.mission_best_gate=mission.gate_ratio;state.mission_best_params=missionGov.params;local_save_state(stateFile,state);
    local_status(out,'VERIFIED',1,mission.gate_ratio,'dynamic H/V + four-drop PASS; stored for one-time next-start revalidation');
    fid=fopen(fullfile(out,'V32_VERIFIED.txt'),'w');if fid>=0,fprintf(fid,'VERIFIED\nlegacy_data_used=0\npersistent_memory=1\nrevalidate_once_next_start=1\ngate_ratio=%.6f\nspeed_envelope=%.3f..%.3f m/s\n',mission.gate_ratio,min(speeds),max(speeds));fclose(fid);end
    result=final;
catch ME
    try
        state=local_load_state(stateFile);state.last_error=string(ME.identifier)+": "+string(ME.message);state.last_updated=datetime('now');local_save_state(stateFile,state);
        if ~isfile(fullfile(out,'V32_VERIFIED.txt')),local_status(out,'FAILED',0,Inf,state.last_error+"; v32 memory retained");end
        fid=fopen(fullfile(runRoot,'V32_FAILED.txt'),'w');if fid>=0,fprintf(fid,'%s\n%s\nmemory_retained=1\n',ME.identifier,ME.message);fclose(fid);end
    catch
    end
    rethrow(ME);
end
end

function [bank,changed]=local_certify_trim_bank_id_ready(root,out,nodeRoot,bank,v,o)
changed=false;
if ~logical(o.UseIdReadinessCertification),return;end
for cfg=0:4
    certRoot=fullfile(nodeRoot,'id_readiness',sprintf('cfg%d',cfg));
    cert=local_run_id_readiness(root,certRoot,bank,cfg,v,o,0);
    m=cert.metrics;
    local_status(out,sprintf('ID_READY_V%.1f_CFG%d',v,cfg),logical(cert.pass),local_id_ready_gate(m,o), ...
        sprintf('VaErr=%.3f VaSlope=%.4f pitchErr=%.3f vz=%.3f q=%.3f hSlope=%.4f hDrift=%.3f reason=%s', ...
        m.va_error_mps,m.va_slope_mps2,m.pitch_error_deg,m.vz_mps,m.q_dps,m.h_slope_mps,m.h_drift_m,char(string(m.fail_reason))));
    if cert.pass
        bank(cfg+1).id_settle_s=max(double(bank(cfg+1).id_settle_s),double(m.settle_s));
        continue;
    end
    before=bank(cfg+1);
    recovered=false;
    for k=1:max(1,round(double(o.IdReadinessRetrimRounds)))
        local_status(out,sprintf('ID_READY_POLISH_V%.1f_CFG%d',v,cfg),0,Inf, ...
            sprintf('long-horizon ID-readiness failed; polish round %d/%d',k,round(double(o.IdReadinessRetrimRounds))));
        cfgRoot=fullfile(nodeRoot,'trim',sprintf('cfg%d',cfg));
        rr=local_find_trim_id_recovery(root,cfgRoot,bank,cfg,v,o,k);
        bank=local_v32_merge_trim_entry(bank,rr.trim_bank(cfg+1),cfg+1);
        bank=local_v32_ensure_trim_fields(bank);
        cert=local_run_id_readiness(root,certRoot,bank,cfg,v,o,k);
        m=cert.metrics;
        local_status(out,sprintf('ID_READY_RECHECK_V%.1f_CFG%d',v,cfg),logical(cert.pass),local_id_ready_gate(m,o), ...
            sprintf('round=%d VaErr=%.3f VaSlope=%.4f vz=%.3f q=%.3f hSlope=%.4f reason=%s', ...
            k,m.va_error_mps,m.va_slope_mps2,m.vz_mps,m.q_dps,m.h_slope_mps,char(string(m.fail_reason))));
        if cert.pass,recovered=true;break;end
    end
    if ~recovered
        error('AirdropX:V32:TrimNotIdReady', ...
            'V=%.1f cfg%d trim failed long-horizon ID-readiness after %d polish round(s): %s', ...
            v,cfg,round(double(o.IdReadinessRetrimRounds)),char(string(m.fail_reason)));
    end
    bank(cfg+1).id_settle_s=max(double(bank(cfg+1).id_settle_s),double(m.settle_s));
    bank(cfg+1).acceptance_mode="v32_1_5_id_readiness";
    changed=changed || local_trim_entries_materially_different(before,bank(cfg+1),o);
    save(fullfile(nodeRoot,'v32_trim_bank.mat'),'bank','-v7.3');
end
end

function cert=local_run_id_readiness(root,certRoot,bank,cfg,v,o,attempt)
runRoot=fullfile(certRoot,sprintf('attempt_%02d',attempt));
cert=airdropx_v32_trim_id_readiness('ProjectRoot',root,'OutputRoot',runRoot,'TrimBank',bank,'ConfigId',cfg, ...
    'TargetAltitudeM',o.ReferenceAltitudeM,'TargetAirspeedMps',v,'ReferenceMassKg',o.ReferenceMassKg,'CargoMassKg',o.CargoMassKg, ...
    'PrepDropStartS',o.IdPrepDropStartS,'PrepDropIntervalS',o.IdPrepDropIntervalS, ...
    'SettleBaseS',o.IdReadinessSettleBaseS,'SettlePerConfigS',o.IdReadinessSettlePerConfigS, ...
    'BaselineDurationS',o.IdReadinessBaselineDurationS,'MaxAirspeedErrorMps',o.IdReadinessMaxAirspeedErrorMps, ...
    'MaxAirspeedSlopeMps2',o.IdReadinessMaxAirspeedSlopeMps2,'MaxPitchErrorDeg',o.IdReadinessMaxPitchErrorDeg, ...
    'MaxAbsVzMps',o.IdReadinessMaxAbsVzMps,'MaxAbsQDps',o.IdReadinessMaxAbsQDps, ...
    'MaxHeightSlopeMps',o.IdReadinessMaxHeightSlopeMps,'MaxHeightDriftM',o.IdReadinessMaxHeightDriftM,'Seed',o.Seed+round(v*10)+cfg+attempt*100);
end

function g=local_id_ready_gate(m,o)
vals=[abs(m.va_error_mps)/o.IdReadinessMaxAirspeedErrorMps;abs(m.va_slope_mps2)/o.IdReadinessMaxAirspeedSlopeMps2; ...
 abs(m.pitch_error_deg)/o.IdReadinessMaxPitchErrorDeg;abs(m.vz_mps)/o.IdReadinessMaxAbsVzMps;abs(m.q_dps)/o.IdReadinessMaxAbsQDps; ...
 abs(m.h_slope_mps)/o.IdReadinessMaxHeightSlopeMps;abs(m.h_drift_m)/o.IdReadinessMaxHeightDriftM];
if any(~isfinite(vals)),g=Inf;else,g=max(vals);end
end

function tf=local_trim_entries_materially_different(a,b,o)
tf=abs(double(a.elevator_cmd)-double(b.elevator_cmd))>double(o.IdReuseTrimElevatorTol) || ...
   abs(double(a.throttle_cmd)-double(b.throttle_cmd))>double(o.IdReuseTrimThrottleTol) || ...
   abs(double(a.pitch_deg)-double(b.pitch_deg))>double(o.IdReuseTrimPitchTolDeg) || ...
   abs(double(a.airspeed_mps)-double(b.airspeed_mps))>double(o.IdReuseTrimAirspeedTolMps);
end

function bank=local_generate_id_data_with_recovery(root,out,nodeRoot,dataRoot,bank,v,o)
% Generic v32 ID baseline recovery.  No speed/cfg special cases are allowed.
% Level 1 is handled inside airdropx_auto_generate_data by extending the
% settle period with the exact same IC.  If that still fails, Level 2 does
% one ID-oriented trim refinement only for the failing physical cfg, then
% regenerates this speed node's clean ID data from scratch.
maxRounds=max(0,round(double(o.IdBaselineRetrimRounds)));
roundIdx=0;
while true
    try
        local_generate_id_data_once(root,dataRoot,bank,v,o,roundIdx>0);
        if roundIdx>0
            local_status(out,sprintf('ID_BASELINE_RECOVERED_V%.1f',v),1,0,sprintf('recovered after %d ID-oriented retrim round(s)',roundIdx));
        end
        return;
    catch ME
        if ~strcmp(ME.identifier,'AirdropX:AutoMPC:IdBaselineNotSettled') || roundIdx>=maxRounds
            rethrow(ME);
        end
        cfg=local_parse_id_baseline_cfg(ME.message);
        if ~isfinite(cfg) || cfg<0 || cfg>4
            rethrow(ME);
        end
        roundIdx=roundIdx+1;
        local_status(out,sprintf('ID_BASELINE_RECOVERY_V%.1f_CFG%d',v,cfg),0,Inf,...
            sprintf('baseline remained unstable after adaptive settle; ID-oriented trim refinement round %d/%d',roundIdx,maxRounds));
        cfgRoot=fullfile(nodeRoot,'trim',sprintf('cfg%d',cfg));
        rr=local_find_trim_id_recovery(root,cfgRoot,bank,cfg,v,o,roundIdx);
        bank=local_v32_merge_trim_entry(bank,rr.trim_bank(cfg+1),cfg+1);
        bank=local_v32_ensure_trim_fields(bank);
        minSettle=double(o.IdBaselineRecoveryMinSettleS)+double(o.IdBaselineRecoveryPerConfigS)*cfg;
        bank(cfg+1).id_settle_s=max(double(bank(cfg+1).id_settle_s),minSettle);
        bank(cfg+1).acceptance_mode="v32_id_baseline_recovery";
        save(fullfile(nodeRoot,'v32_trim_bank.mat'),'bank','-v7.3');
        % Partial data from a failed generation must never leak into the
        % identification build. Recreate only this current v32 speed-node ID
        % workspace; trim/physics/controller knowledge remains untouched.
        try, if isfolder(dataRoot),rmdir(dataRoot,'s');end, catch, end
    end
end
end

function local_generate_id_data_once(root,dataRoot,bank,v,o,isRecovery)
adaptiveMax=double(o.IdAdaptiveSettleMaxS);
maxRetries=double(o.IdMaxSettleRetries);
retryStep=double(o.IdSettleRetryStepS);
if isRecovery
    adaptiveMax=max(adaptiveMax,double(o.IdAdaptiveSettleRecoveryMaxS));
    maxRetries=max(maxRetries,double(o.IdBaselineRecoveryExtraRetries));
    retryStep=max(retryStep,double(o.IdBaselineRecoverySettleStepS));
end
airdropx_auto_generate_data('ProjectRoot',root,'OutputRoot',dataRoot,'TrimBank',bank,'PreparationTrimBank',bank,'UsePreparationTrimSchedule',true,...
    'ConfigIds',(0:4).','RunsPerConfig',o.IdRunsPerConfig,'Seed',o.Seed+round(v*10),'TargetAltitudeM',o.ReferenceAltitudeM,'TargetAirspeedMps',v,'IdentificationAltitudeM',o.ReferenceAltitudeM,...
    'ReferenceMassKg',o.ReferenceMassKg,'CargoMassKg',o.CargoMassKg,'IdentificationDurationS',o.IdDurationS,'ElevatorAmplitude',o.IdElevatorAmplitude,'ThrottleAmplitude',o.IdThrottleAmplitude,...
    'PrepDropStartS',o.IdPrepDropStartS,'PrepDropIntervalS',o.IdPrepDropIntervalS,...
    'MaxSettleRetries',maxRetries,'SettleRetryStepS',retryStep,'AdaptiveSettleRecoveryEnabled',true,'AdaptiveSettleRecoveryMaxS',adaptiveMax);
end

function cfg=local_parse_id_baseline_cfg(message)
cfg=NaN;
tok=regexp(char(string(message)),'Config\s+(\d+)\s+run\s+\d+','tokens','once');
if ~isempty(tok),cfg=str2double(tok{1});end
end

function rr=local_find_trim_id_recovery(root,cfgRoot,bank,cfg,v,o,roundIdx)
% Re-search the same physical trim using stricter tail-equilibrium gates.
% This is generic across every speed and cfg and uses only v32-native data.
common={'ProjectRoot',root,'OutputMat',fullfile(cfgRoot,'trim_result.mat'),'WorkRoot',fullfile(cfgRoot,'work'),...
 'CheckpointMat',fullfile(cfgRoot,'trim_checkpoint.mat'),'PreviousTrimMat','',...
 'ReuseVerifiedTrim',false,'ConfigIds',cfg,'TargetAltitudeM',o.ReferenceAltitudeM,'TargetAirspeedMps',v,'SearchAltitudeM',o.ReferenceAltitudeM,...
 'ReferenceMassKg',o.ReferenceMassKg,'CargoMassKg',o.CargoMassKg,'PreparationTrimBank',local_v32_ensure_trim_fields(bank),'UsePreparationTrimSchedule',true,...
 'PrepDropStartS',o.TrimPrepDropStartS,'PrepDropIntervalS',o.TrimPrepDropIntervalS,...
 'UseParallel',logical(o.UseParallel),'Verbose',1,'ReuseFailedAsWarmStart',true,...
 'MaxObjectiveEvaluations',max(o.IdBaselineRetrimEvaluations,round((1+0.25*roundIdx)*o.TrimRetryMaxEvaluations)),...
 'MaxTailAbsVzMps',o.IdRecoveryMaxTailAbsVzMps,'MaxTailAbsQDps',o.IdRecoveryMaxTailAbsQDps,...
 'MaxTailHeightSlopeMps',o.IdRecoveryMaxTailHeightSlopeMps,'MaxTailAirspeedRmsMps',o.IdRecoveryMaxTailAirspeedRmsMps,...
 'TailRescueMinConfig',0,'ForceLongHorizonTailPolish',true,'TailRescueTriggerRatio',0.0,...
 'TailRescueStopTimeBaseS',o.IdReadinessPolishStopTimeBaseS,'TailRescueStopTimePerConfigS',o.IdReadinessPolishStopTimePerConfigS,...
 'TailRescueObjectiveEvaluations',o.IdReadinessPolishEvaluations,'TailRescueMaxAbsVzMps',o.IdReadinessMaxAbsVzMps,...
 'TailRescueMaxAbsQDps',o.IdReadinessMaxAbsQDps,'TailRescueMaxHeightSlopeMps',o.IdReadinessMaxHeightSlopeMps,...
 'TailRescueMaxAirspeedRmsMps',o.IdReadinessMaxAirspeedErrorMps,'JointMapPostConfigObserveS',o.IdRecoveryJointMapObserveS,...
 'JointMapVzScaleMps',o.IdRecoveryJointMapVzScaleMps,'JointMapVaErrorScaleMps',o.IdRecoveryJointMapVaErrorScaleMps,...
 'AdaptiveIdSettleEnabled',true,'AdaptiveIdSettleMinS',o.IdBaselineRecoveryMinSettleS,...
 'AdaptiveIdSettleMaxS',o.IdAdaptiveSettleRecoveryMaxS};
rr=airdropx_auto_find_trim(common{:});
end

function rr=local_find_trim_clean(root,cfgRoot,bank,cfg,v,o)
hasCheckpoint=isfile(fullfile(cfgRoot,'trim_checkpoint.mat'));
common={'ProjectRoot',root,'OutputMat',fullfile(cfgRoot,'trim_result.mat'),'WorkRoot',fullfile(cfgRoot,'work'),...
 'CheckpointMat',fullfile(cfgRoot,'trim_checkpoint.mat'),'PreviousTrimMat','',...
 'ReuseVerifiedTrim',hasCheckpoint,'ConfigIds',cfg,'TargetAltitudeM',o.ReferenceAltitudeM,'TargetAirspeedMps',v,'SearchAltitudeM',o.ReferenceAltitudeM,...
 'ReferenceMassKg',o.ReferenceMassKg,'CargoMassKg',o.CargoMassKg,'PreparationTrimBank',local_v32_ensure_trim_fields(bank),'UsePreparationTrimSchedule',true,...
 'PrepDropStartS',o.TrimPrepDropStartS,'PrepDropIntervalS',o.TrimPrepDropIntervalS,...
 'UseParallel',logical(o.UseParallel),'Verbose',1};
try
 rr=airdropx_auto_find_trim(common{:},'ReuseFailedAsWarmStart',hasCheckpoint,'MaxObjectiveEvaluations',o.TrimMaxEvaluations);
catch ME1
 fprintf('[V32.1.5-TRIM] V=%.1f cfg%d first clean search failed (%s). Retrying once with same-run warm start and larger budget.\n',v,cfg,ME1.identifier);
 rr=airdropx_auto_find_trim(common{:},'ReuseFailedAsWarmStart',true,'MaxObjectiveEvaluations',max(o.TrimRetryMaxEvaluations,round(1.5*o.TrimMaxEvaluations)));
end
end

function bank=local_v32_ensure_trim_fields(bank)
% Keep the v32 preparation bank schema compatible with the extended schema
% emitted by airdropx_auto_find_trim. Defaults intentionally mirror the
% shared trim search defaults (4 s base + 2 s per config ID).
if isempty(bank),return;end
n=numel(bank);
if ~isfield(bank,'score')
    [bank(1:n).score]=deal(NaN);
end
if ~isfield(bank,'initial_flight_path_deg')
    [bank(1:n).initial_flight_path_deg]=deal(0.0);
end
if ~isfield(bank,'id_settle_s')
    [bank(1:n).id_settle_s]=deal(NaN);
end
if ~isfield(bank,'acceptance_mode')
    [bank(1:n).acceptance_mode]=deal("");
end
if ~isfield(bank,'resume_seed_valid')
    [bank(1:n).resume_seed_valid]=deal(false);
end
for k=1:n
    if isempty(bank(k).score),bank(k).score=NaN;end
    if isempty(bank(k).initial_flight_path_deg),bank(k).initial_flight_path_deg=0.0;end
    if isempty(bank(k).id_settle_s)||~isfinite(double(bank(k).id_settle_s))
        bank(k).id_settle_s=4.0+2.0*max(0,k-1);
    end
    if isempty(bank(k).acceptance_mode)||all(strlength(string(bank(k).acceptance_mode))==0)
        bank(k).acceptance_mode="";
    end
    if isempty(bank(k).resume_seed_valid),bank(k).resume_seed_valid=false;end
end
end

function bank=local_v32_merge_trim_entry(bank,src,idx)
% Field-by-field merge deliberately avoids MATLAB heterogeneous-struct
% indexed assignment. It also preserves any future diagnostic fields that
% find_trim may add without requiring another v32 schema patch.
if ~isstruct(src)||numel(src)~=1
    error('AirdropX:V32:TrimSchema','Expected one trim struct entry for cfg index %d.',idx-1);
end
bank=local_v32_ensure_trim_fields(bank);
fn=fieldnames(src);
for i=1:numel(fn)
    bank(idx).(fn{i})=src.(fn{i});
end
bank=local_v32_ensure_trim_fields(bank);
end

function tf=local_trim_banks_equivalent(a,b,o)
% Reuse an identified plant only if every revalidated operating point stayed
% in essentially the same physical neighborhood as the old certificate.
tf=false;
try
    a=local_v32_ensure_trim_fields(a);b=local_v32_ensure_trim_fields(b);
    if numel(a)<5||numel(b)<5,return;end
    for k=1:5
        da=abs(double(a(k).elevator_cmd)-double(b(k).elevator_cmd));
        dt=abs(double(a(k).throttle_cmd)-double(b(k).throttle_cmd));
        dp=abs(double(a(k).pitch_deg)-double(b(k).pitch_deg));
        dv=abs(double(a(k).airspeed_mps)-double(b(k).airspeed_mps));
        if ~all(isfinite([da dt dp dv])) || da>double(o.IdReuseTrimElevatorTol) || ...
                dt>double(o.IdReuseTrimThrottleTol) || dp>double(o.IdReuseTrimPitchTolDeg) || ...
                dv>double(o.IdReuseTrimAirspeedTolMps)
            return;
        end
    end
    tf=true;
catch
    tf=false;
end
end

function m=local_run_mission(root,bank,hidden,g,o,outRoot)
m=airdropx_v32_dynamic_mission_validation('ProjectRoot',root,'BankMat',bank,'OutputRoot',outRoot,'Profile',o.FinalMissionProfile,'DropStartS',o.FinalDropStartS,'DropIntervalS',o.FinalDropIntervalS,...
 'ReferenceMassKg',o.ReferenceMassKg,'CargoMassKg',o.CargoMassKg,'HiddenElevatorTrim',hidden,'HeightKh',g.Kh,'HeightKi',g.Ki,'HeightKaw',g.Kaw,'HeightVzMaxMps',g.VzMax,'HeightVzSlewMps2',g.VzSlew,'HeightBiasMaxMps',g.BiasMax);
end
function [bestMission,gov]=local_mission_refine(root,bank,hidden,gov,o,out,baseline)
p=gov.params;F=[1 1 1 1 1 1;0.85 1 1.2 1 1 1;1.15 1 1.2 1 1.15 1;1 0.8 1.4 1 1 1;1 1.2 1.4 1.15 1.2 1.1;0.9 0.8 1.6 1.2 1.3 1.1];
bestMission=baseline;bestP=p;rows=table();refRoot=fullfile(out,'mission_refinement');if ~isfolder(refRoot),mkdir(refRoot);end
for k=1:size(F,1)
 q=p;q.Kh=p.Kh*F(k,1);q.Ki=p.Ki*F(k,2);q.Kaw=p.Kaw*F(k,3);q.VzMax=min(2.5,p.VzMax*F(k,4));q.VzSlew=min(1.8,p.VzSlew*F(k,5));q.BiasMax=min(3,p.BiasMax*F(k,6));
 r=local_run_mission(root,bank,hidden,q,o,fullfile(refRoot,sprintf('candidate_%02d',k)));
 rr=table(k,q.Kh,q.Ki,q.Kaw,q.VzMax,q.VzSlew,q.BiasMax,r.gate_ratio,r.pass,'VariableNames',{'candidate','Kh','Ki','Kaw','VzMax','VzSlew','BiasMax','gate_ratio','pass'});if isempty(rows),rows=rr;else,rows=[rows;rr];end %#ok<AGROW>
 if r.gate_ratio<bestMission.gate_ratio,bestMission=r;bestP=q;end
 if r.pass,break;end
end
writetable(rows,fullfile(refRoot,'summary.csv'));gov.params=bestP;gov.gate_ratio=bestMission.gate_ratio;gov.pass=bestMission.pass;
end

function hidden=local_calibrate_hidden_trim(root,trim,o,outDir)
if ~isfolder(outDir),mkdir(outDir);end
cal=airdropx_auto_run_id_experiment('ProjectRoot',root,'OutputRoot',outDir,'RunId','v32_hidden_trim','ConfigId',0,'Trim',trim,'StopTimeS',0.5,'RecordStartS',0,'ExportStartS',0,'ExcitationStartS',100,...
 'ElevatorAmplitude',0,'ThrottleAmplitude',0,'DirectIdMode',true,'KeepFixedConfigurationOnly',true,'InitialAltitudeM',o.ReferenceAltitudeM,'InitialAirspeedMps',o.AnchorSpeedMps,'InitialPitchDeg',trim.pitch_deg,'InitialFlightPathDeg',0,...
 'TargetAltitudeM',o.ReferenceAltitudeM,'TargetAirspeedMps',o.AnchorSpeedMps,'ReferenceMassKg',o.ReferenceMassKg,'CargoMassKg',o.CargoMassKg);
T=cal.timeseries;external=local_col(T,'requested_elevator_cmd');physical=local_col(T,'elevator_cmd_norm');t=local_col(T,'time_s');m=isfinite(external)&isfinite(physical)&isfinite(t)&t<=0.25;if nnz(m)<3,error('AirdropX:V32:HiddenTrimSamples','Not enough hidden trim samples.');end
hidden=median(physical(m)-external(m),'omitnan');fprintf('[V32] hidden elevator trim=%.6f\n',hidden);
end
function x=local_col(T,n),if ismember(n,string(T.Properties.VariableNames)),x=double(T.(n));else,x=NaN(height(T),1);end,end
function local_model_quality_guard(T,v,o)
for cfg=0:4
 R=T(T.config_id==cfg & T.prediction_steps==5 & T.split=="validation",:);if isempty(R),continue;end
 if all(R.fit_airspeed_pct<o.MinIdFitVaPct) || all(R.fit_vz_pct<o.MinIdFitVzPct)
  warning('AirdropX:V32:WeakID','V=%.1f cfg%d has weak 5-step ID fit (Va %.1f, vz %.1f). Inner certification will decide.',v,cfg,max(R.fit_airspeed_pct),max(R.fit_vz_pct));
 end
end
end
function local_parallel(n,shortRoot)
try,p=gcp('nocreate');if ~isempty(p)&&p.NumWorkers~=n,delete(p);p=[];end;if isempty(p),parpool('local',n);end,catch ME,warning('AirdropX:V32:Parallel','Parallel pool unavailable: %s',ME.message);end
try,c=fullfile(shortRoot,'main','c');g=fullfile(shortRoot,'main','g');if ~isfolder(c),mkdir(c);end;if ~isfolder(g),mkdir(g);end;Simulink.fileGenControl('set','CacheFolder',c,'CodeGenFolder',g,'createDir',true);catch,end
end
function local_write_physics_context(out,ctx)
try
    T=table(string(ctx.context_mass_semantics),double(ctx.context_reference_mass_kg),...
        double(ctx.initial_fuel_mass_kg),double(ctx.estimated_actual_reference_mass_kg),...
        string(ctx.fuel_source),string(ctx.aircraft_fingerprint),string(ctx.aircraft_xml_path),...
        'VariableNames',{'context_mass_semantics','context_reference_mass_kg','initial_fuel_mass_kg',...
        'estimated_actual_reference_mass_kg','fuel_source','aircraft_fingerprint','aircraft_xml_path'});
    writetable(T,fullfile(out,'v32_physics_context.csv'));
catch ME
    warning('AirdropX:V32:PhysicsContextWrite','Could not write physics-context report: %s',ME.message);
end
end

function local_memory_policy(out,o,state)
f=fullfile(out,'V32_MEMORY_POLICY.txt');
% Rewrite on every run so a schema upgrade cannot leave an old policy file
% claiming outdated physics/mass semantics.
fid=fopen(f,'w');if fid<0,return;end
 fprintf(fid,'AirdropX v32.1.5 persistent memory policy\ncreated=%s\nlegacy_data_used=0\nlegacy_v29_v30_v31_import=DISABLED\n',char(datetime('now')));
 fprintf(fid,'normal_start=RESUME_V32_MEMORY\nreset_requires_explicit_ResetLearning=true\nverified_policy=one_revalidation_then_skip\n');
 fprintf(fid,'speed_nodes_mps=%s\nreference_altitude_m=%.3f\n',mat2str(double(o.SpeedNodesMps(:).')),o.ReferenceAltitudeM);
 fprintf(fid,'trim_solver=joint_elevator_throttle_map_then_long_horizon_id_readiness\n');
 fprintf(fid,'context_mass_semantics=%s\n',char(string(o.PhysicsContext.context_mass_semantics)));
 fprintf(fid,'context_reference_mass_kg=%.6f\n',double(o.ReferenceMassKg));
 fprintf(fid,'initial_xml_fuel_mass_kg=%.6f\n',double(o.PhysicsContext.initial_fuel_mass_kg));
 fprintf(fid,'estimated_actual_reference_mass_kg=%.6f\n',double(o.PhysicsContext.estimated_actual_reference_mass_kg));
 fprintf(fid,'aircraft_fingerprint=%s\n',char(string(o.PhysicsContext.aircraft_fingerprint)));
 fprintf(fid,'id_baseline_recovery=adaptive_settle_then_single_strict_retrim\n');fclose(fid);
T=table(state.run_count,string(state.last_stage),logical(state.inner_verified),logical(state.governor_verified),logical(state.final_verified),logical(state.final_revalidated),double(state.last_gate_ratio),string(state.last_error),...
 'VariableNames',{'run_count','last_stage','inner_verified','governor_verified','final_verified','final_revalidated','last_gate_ratio','last_error'});
writetable(T,fullfile(out,'v32_memory_status.csv'));
end
function local_write_run_info(runRoot,o,state)
fid=fopen(fullfile(runRoot,'run_info.txt'),'w');if fid<0,return;end
fprintf(fid,'started=%s\nrun_count=%d\nreset_learning=%d\nlegacy_data_used=0\nworkers=%d\n',char(datetime('now')),state.run_count,logical(o.ResetLearning),o.Workers);fclose(fid);
end
function state=local_load_state(file)
state=struct('schema_version',1,'run_count',0,'last_run_id',"",'last_stage',"NEW",'last_updated',datetime('now'),...
 'controller_signature',"",'physics_nodes_verified',zeros(0,1),'inner_verified',false,'governor_verified',false,'final_verified',false,'final_revalidated',false,'final_revalidation_attempted',false,...
 'inner_checkpoint',"",'governor_checkpoint',"",'final_controller',"",'mission_best_gate',Inf,'mission_best_params',struct(),'last_gate_ratio',Inf,'last_error',"");
if ~isfile(file),return;end
try,S=load(file);if isfield(S,'state'),old=S.state;fn=fieldnames(old);for i=1:numel(fn),state.(fn{i})=old.(fn{i});end,end,catch,end
end
function local_save_state(file,state)
state.last_updated=datetime('now');tmp=[file '.tmp'];save(tmp,'state','-v7');movefile(tmp,file,'f');
try
 out=fileparts(fileparts(file));
 T=table(state.run_count,string(state.last_stage),logical(state.inner_verified),logical(state.governor_verified),logical(state.final_verified),logical(state.final_revalidated),double(state.last_gate_ratio),double(state.mission_best_gate),string(state.last_error),...
  'VariableNames',{'run_count','last_stage','inner_verified','governor_verified','final_verified','final_revalidated','last_gate_ratio','mission_best_gate','last_error'});
 writetable(T,fullfile(out,'v32_memory_status.csv'));
catch
end
end
function local_delete_if_exists(f)
try,if isfile(f),delete(f);end,catch,end
end
function s=local_physics_signature(o,v)
ctx=o.PhysicsContext;
s=string(sprintf(['v32.1.5_physics_joint2D_idready_V%.6f_Mctx%.6f_C%.6f_H%.6f_IDR%d_IDT%.3f_E%.4f_T%.4f_PD%.3f_' ...
    'IR%.3f_IS%.3f_IVZ%.3f_IQ%.3f_IH%.3f_FP%s'], ...
    v,o.ReferenceMassKg,o.CargoMassKg,o.ReferenceAltitudeM,o.IdRunsPerConfig,o.IdDurationS,o.IdElevatorAmplitude,o.IdThrottleAmplitude,...
    o.TrimPrepDropIntervalS,o.IdReadinessMaxAirspeedErrorMps,o.IdReadinessMaxAirspeedSlopeMps2,o.IdReadinessMaxAbsVzMps,...
    o.IdReadinessMaxAbsQDps,o.IdReadinessMaxHeightSlopeMps,char(string(ctx.aircraft_fingerprint))));
end
function s=local_hidden_signature(o)
s=string(sprintf('v32.1.5_hidden_V%.6f_Mctx%.6f_C%.6f_FP%s',o.AnchorSpeedMps,o.ReferenceMassKg,o.CargoMassKg,char(string(o.PhysicsContext.aircraft_fingerprint))));
end
function s=local_controller_signature(o,speeds)
s=string(sprintf('v32.1.5_inner4out_Mctx%.6f_C%.6f_Fuel%.6f_FP%s_S%s',o.ReferenceMassKg,o.CargoMassKg,...
    double(o.PhysicsContext.initial_fuel_mass_kg),char(string(o.PhysicsContext.aircraft_fingerprint)),strrep(mat2str(sort(double(speeds(:))).',6),' ','')));
end
function local_status(out,stage,pass,gate,note)
T=table(string(datetime('now','Format','yyyy-MM-dd HH:mm:ss')),string(stage),logical(pass),double(gate),string(note),'VariableNames',{'timestamp','stage','pass','gate_ratio','note'});
writetable(T,fullfile(out,'v32_status.csv'));histFile=fullfile(out,'v32_stage_history.csv');
try,if isfile(histFile),H=readtable(histFile,'TextType','string');H=[H;T];else,H=T;end,writetable(H,histFile);catch,end
fprintf('[V32-STATUS] stage=%s pass=%d gate=%.3f %s\n',char(string(stage)),pass,gate,char(string(note)));
end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function p=local_resolve(root,x),x=char(string(x));if isempty(regexp(x,'^[A-Za-z]:[\\/]','once'))&&~startsWith(x,filesep),p=fullfile(root,x);else,p=x;end,end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.OutputRoot="matlab/results/mpc_auto_v32_clean";opts.Model="airdropx_v32_mpc_closed_loop";opts.ResetLearning=false;opts.RevalidateVerifiedOnce=true;opts.Workers=3;opts.UseParallel=true;opts.ShortFileGenRoot="D:\AXC\v32";opts.SuppressFigures=true;
opts.ReferenceAltitudeM=200;opts.AnchorSpeedMps=50;opts.SpeedNodesMps=[45;50;55];opts.ReferenceMassKg=3423;opts.CargoMassKg=300;opts.Seed=3200;
opts.TrimPrepDropStartS=1.0;opts.TrimPrepDropIntervalS=2.0;opts.IdPrepDropStartS=1.0;opts.IdPrepDropIntervalS=2.0;
opts.TrimMaxEvaluations=70;opts.TrimRetryMaxEvaluations=120;opts.IdReuseTrimElevatorTol=0.01;opts.IdReuseTrimThrottleTol=0.015;opts.IdReuseTrimPitchTolDeg=0.30;opts.IdReuseTrimAirspeedTolMps=0.50;opts.IdRunsPerConfig=7;opts.IdDurationS=42;opts.IdElevatorAmplitude=0.05;opts.IdThrottleAmplitude=0.10;opts.IdMaxSettleRetries=2;opts.IdSettleRetryStepS=12;opts.IdAdaptiveSettleMaxS=90;opts.IdAdaptiveSettleRecoveryMaxS=120;opts.IdBaselineRetrimRounds=2;opts.IdBaselineRetrimEvaluations=180;opts.IdBaselineRecoveryMinSettleS=30;opts.IdBaselineRecoveryPerConfigS=5;opts.IdBaselineRecoveryExtraRetries=5;opts.IdBaselineRecoverySettleStepS=15;opts.IdRecoveryMaxTailAbsVzMps=0.15;opts.IdRecoveryMaxTailAbsQDps=0.15;opts.IdRecoveryMaxTailHeightSlopeMps=0.15;opts.IdRecoveryMaxTailAirspeedRmsMps=0.75;opts.MinCleanIdRuns=4;opts.IdOrders=4:10;opts.MinIdFitVaPct=20;opts.MinIdFitVzPct=10;
opts.UseIdReadinessCertification=true;opts.IdReadinessRetrimRounds=2;opts.IdReadinessSettleBaseS=55;opts.IdReadinessSettlePerConfigS=8;opts.IdReadinessBaselineDurationS=12;opts.IdReadinessMaxAirspeedErrorMps=0.75;opts.IdReadinessMaxAirspeedSlopeMps2=0.08;opts.IdReadinessMaxPitchErrorDeg=1.0;opts.IdReadinessMaxAbsVzMps=0.15;opts.IdReadinessMaxAbsQDps=0.15;opts.IdReadinessMaxHeightSlopeMps=0.15;opts.IdReadinessMaxHeightDriftM=1.5;opts.IdReadinessPolishStopTimeBaseS=75;opts.IdReadinessPolishStopTimePerConfigS=8;opts.IdReadinessPolishEvaluations=72;opts.IdRecoveryJointMapObserveS=14;opts.IdRecoveryJointMapVzScaleMps=0.50;opts.IdRecoveryJointMapVaErrorScaleMps=2.0;
opts.InnerBayesEvaluations=24;opts.GovernorBayesEvaluations=18;opts.MissionNearPassGate=1.5;
opts.FinalMissionProfile=[0 200 50;80 150 48;160 100 55;250 50 45;340 20 50;430 120 55;530 200 50];opts.FinalDropStartS=110;opts.FinalDropIntervalS=100;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
opts.Workers=min(max(1,round(double(opts.Workers))),3);
end
