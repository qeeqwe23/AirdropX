function result = airdropx_auto_envelope_train(varargin)
%AIRDROPX_AUTO_ENVELOPE_TRAIN v30 continuous H/V flight-envelope trainer.
%
% Goal:
%   One learning architecture for every mission in H=20..200 m and a
%   physically qualified common airspeed range. No cfg-specific learning
%   policies and no hand-created controller table per altitude.
%
% The trainer is resume-safe. Every H/V context has its own OutputRoot while
% all contexts share:
%   1) Controller LearningBank (v29)
%   2) PlantContextBank (v30)
%
% Stages:
%   A. Register the already-qualified 200 m / 50 m/s anchor.
%   B. Descend progressively at 50 m/s: 200 -> 180 -> ... -> 20 m.
%      If one level exhausts, insert a bridge point only down to the
%      configured minimum altitude spacing (v30.3 default: 10 m), then
%      retry the failed level from the closer learned context.
%   C. Only after the full altitude descent passes, expand airspeed outward
%      from 50 m/s at the 20 m stress altitude.
%   D. When a coarse pass/fail bracket exists, refine speed boundaries.
%   E. Train deterministic interior H/V points to fill the 2-D envelope.
%
% Re-running the same command resumes from mission checkpoints and skips
% contexts that already have full cfg0->cfg4 MISSION_PASS=1.

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir); addpath(paths.mpcDir); addpath(paths.autoDir);
if isfolder(paths.sfuncDir), addpath(paths.sfuncDir); end

envelopeRoot = local_resolve_path(paths.projectRoot,opts.EnvelopeRoot);
learningBankRoot = local_resolve_path(paths.projectRoot,opts.LearningBankRoot);
plantBankRoot = local_resolve_path(paths.projectRoot,opts.PlantBankRoot);
if ~isfolder(envelopeRoot), mkdir(envelopeRoot); end
if ~isfolder(learningBankRoot), mkdir(learningBankRoot); end
if ~isfolder(plantBankRoot), mkdir(plantBankRoot); end
historyFile = fullfile(envelopeRoot,"envelope_mission_history.csv");
statusFile = fullfile(envelopeRoot,"envelope_status.csv");

local_register_existing_anchor(paths,plantBankRoot,opts);
history = local_read_history(historyFile);
history = local_import_anchor_history(history,paths,opts);
[history,archivedSubresolution] = local_archive_subresolution_altitude_contexts(history,opts);
if archivedSubresolution>0
    fprintf("[V30.3-ALT] archived %d legacy nominal-speed context(s) below the new %.1f m altitude resolution; data kept, no new generation will be opened.\n", ...
        archivedSubresolution,opts.AltitudeRefineResolutionM);
end
% Mission-level recovery must be resolved BEFORE generic infrastructure
% reopening.  A completed all-VERIFIED Final Mission result is authoritative
% control evidence; stale/incomplete worker eval folders must never override it.
[history,reopenedMission] = local_reopen_legacy_mission_nearpass_for_v30_6(history,opts);
if reopenedMission>0
    fprintf("[V30.6-MISSION] reopened %d exhausted full-mission near-pass context(s) once for universal transition recovery.\n",reopenedMission);
end
[history,repairedMissionResume] = local_repair_v30_6_mission_resume_generations(history,opts);
if repairedMissionResume>0
    fprintf("[V30.6.2-MISSION] repaired %d partial/misclassified generation(s) back to mission-recovery-only mode.\n",repairedMissionResume);
end
[history,reopenedInfra] = local_reopen_infrastructure_generations(history,opts);
if reopenedInfra>0
    fprintf("[V30.3-INFRA] reopened %d H/V context generation(s) whose latest failure was infrastructure-invalid.\n",reopenedInfra);
end
[history,reopenedUniversal] = local_reopen_legacy_nearpass_for_universal_recovery(history,opts);
if reopenedUniversal>0
    fprintf("[V30.4-RECOVERY] reopened %d legacy exhausted near-pass context(s) once for the universal recovery path.\n",reopenedUniversal);
end
[history,reopenedController] = local_reopen_legacy_controller_nearpass_for_v30_5(history,opts);
if reopenedController>0
    fprintf("[V30.5-CTRL] reopened %d exhausted controller-near-pass context(s) once for universal local refinement.\n",reopenedController);
end
restoredGlobal = local_restore_real_trim_evaluations(learningBankRoot,history);
if restoredGlobal>0
    fprintf("[V30.3-DATA] restored %d valid evaluation row(s) previously scrubbed by v30.2 from real-trim-failure missions.\n",restoredGlobal);
end
prunedGlobal = local_scrub_global_infra_evaluations(learningBankRoot,history);
if prunedGlobal>0
    fprintf("[V30.3-INFRA] removed %d legacy infrastructure-invalid row(s) from global evaluations.csv (backup retained).\n",prunedGlobal);
end
local_write_history(historyFile,history);

newRuns = 0;
maxNewRuns = double(opts.MaxNewMissionRunsPerInvocation);
if ~isfinite(maxNewRuns), maxNewRuns = realmax; end

fprintf("\n============================================================\n");
fprintf("[V30-ENVELOPE] unified flight-envelope training\n");
fprintf("[V30-ENVELOPE] altitude range %.1f..%.1f m\n",opts.AltitudeMinM,opts.AltitudeMaxM);
fprintf("[V30-ENVELOPE] nominal airspeed %.1f m/s; search %.1f..%.1f m/s\n", ...
    opts.NominalAirspeedMps,opts.SpeedSearchMinMps,opts.SpeedSearchMaxMps);
fprintf("[V30-ENVELOPE] minimum altitude refinement spacing %.1f m\n",opts.AltitudeRefineResolutionM);
fprintf("[V30-ENVELOPE] common LearningBank: %s\n",learningBankRoot);
fprintf("[V30-ENVELOPE] PlantContextBank: %s\n",plantBankRoot);
fprintf("============================================================\n");

while newRuns < maxNewRuns
    [next,stageInfo] = local_select_next_context(history,opts);
    local_write_status(statusFile,history,stageInfo,opts);
    local_plot_history(history,fullfile(envelopeRoot,"envelope_training_map.png"),opts,stageInfo);
    if ~next.exists
        break;
    end

    attemptsAlready = local_attempts_for(history,next.H,next.V,next.generation);
    attemptsRemaining = max(0,round(double(opts.MaxTotalAttemptsPerContext))-attemptsAlready);
    if attemptsRemaining <= 0
        history = local_mark_exhausted(history,next.H,next.V,next.stage,next.generation,next.supportH);
        local_write_history(historyFile,history);
        continue;
    end
    attemptsThisCall = min(round(double(opts.MaxAttemptsPerMissionCall)),attemptsRemaining);

    missionTag = local_context_tag(next.H,next.V);
    if next.generation > 0
        missionTag = missionTag + "_g" + sprintf("%02d",next.generation);
    end
    missionRoot = fullfile(envelopeRoot,"missions",missionTag);
    missionRecoveryOnly = local_generation_is_mission_recovery(history,next.H,next.V,next.generation);
    if missionRecoveryOnly
        % Preserve the special mode in history even if the ordinary altitude/
        % speed selector described this context as a generic retry.
        next.stage="mission_nearpass_pending";
    end
    missionRecoverySourceRoot = "";
    if missionRecoveryOnly
        missionRecoverySourceRoot = local_previous_mission_nearpass_source_root(history,next.H,next.V,next.generation,opts);
        if strlength(missionRecoverySourceRoot)==0
            error("AirdropX:V30_6_1:MissingMissionRecoverySource", ...
                "Mission-recovery-only generation H=%.3f V=%.3f g%d has no prior all-VERIFIED source generation.", ...
                next.H,next.V,next.generation);
        end
        fprintf("[V30.6.1-MISSION] mission-recovery-only source: %s\n",missionRecoverySourceRoot);
    end
    fprintf("\n[V30-ENVELOPE] NEXT stage=%s H=%.3f V=%.3f gen=%d supportH=%.3f attempts=%d/%d\n", ...
        next.stage,next.H,next.V,next.generation,next.supportH,attemptsAlready,opts.MaxTotalAttemptsPerContext);
    r = airdropx_auto_run_any_mission( ...
        "ProjectRoot",paths.projectRoot, ...
        "BaseIdentifiedMat",opts.BaseIdentifiedMat, ...
        "OutputRoot",missionRoot, ...
        "LearningBankRoot",learningBankRoot, ...
        "PlantBankRoot",plantBankRoot, ...
        "TargetAltitudeM",next.H, ...
        "TargetAirspeedMps",next.V, ...
        "MinimumAllowedAltitudeM",opts.AltitudeMinM, ...
        "MaximumAllowedAltitudeM",opts.AltitudeMaxM, ...
        "ReferenceMassKg",opts.ReferenceMassKg, ...
        "CargoMassKg",opts.CargoMassKg, ...
        "TotalDropCount",opts.TotalDropCount, ...
        "UseParallel",opts.UseParallel, ...
        "ParallelWorkers",opts.ParallelWorkers, ...
        "MaxAttempts",attemptsThisCall, ...
        "MissionRecoveryOnly",missionRecoveryOnly, ...
        "MissionRecoverySourceRoot",missionRecoverySourceRoot, ...
        "UnifiedTransferSeedEvaluations",opts.UnifiedTransferSeedEvaluations, ...
        "UnifiedAdditionalEvaluationsPerRun",opts.UnifiedAdditionalEvaluationsPerRun, ...
        "UnifiedControllerNearPassEnabled",opts.UnifiedControllerNearPassEnabled, ...
        "UnifiedControllerNearPassGateRatioMax",opts.UnifiedControllerNearPassGateRatioMax, ...
        "UnifiedControllerNearPassDeterministicEvaluations",opts.UnifiedControllerNearPassDeterministicEvaluations, ...
        "UnifiedControllerNearPassBayesEvaluations",opts.UnifiedControllerNearPassBayesEvaluations, ...
        "UnifiedControllerNearPassMaxRoundsPerContext",opts.UnifiedControllerNearPassMaxRoundsPerContext, ...
        "BumplessTransitionEnabled",opts.BumplessTransitionEnabled, ...
        "TransitionMoveTransferScale",opts.TransitionMoveTransferScale, ...
        "TransitionIntegralTransferScale",opts.TransitionIntegralTransferScale, ...
        "UniversalMissionNearPassEnabled",opts.UniversalMissionNearPassEnabled, ...
        "UniversalMissionNearPassGateRatioMax",opts.UniversalMissionNearPassGateRatioMax, ...
        "UniversalMissionNearPassMaxNewEvaluationsPerAttempt",opts.UniversalMissionNearPassMaxNewEvaluationsPerAttempt, ...
        "UniversalMissionNearPassMoveScales",opts.UniversalMissionNearPassMoveScales, ...
        "UniversalMissionNearPassIntegralScales",opts.UniversalMissionNearPassIntegralScales, ...
        "UniversalRecoveryNearPassGateRatioMax",opts.UniversalRecoveryNearPassGateRatioMax, ...
        "UniversalRecoveryExtendedProbeDurationS",opts.UniversalRecoveryExtendedProbeDurationS, ...
        "UniversalRecoveryExtendedTailWindowS",opts.UniversalRecoveryExtendedTailWindowS, ...
        "UniversalRecoveryLocalRetrimEvaluations",opts.UniversalRecoveryLocalRetrimEvaluations);
    attemptsUsedNow = max(1,round(double(r.attempts_used)));
    newRuns = newRuns + attemptsUsedNow;
    infra=false;
    if isfield(r,"infrastructure_failure"), infra=logical(r.infrastructure_failure); end
    if infra
        % Infrastructure/runtime failures do not consume the H/V controller
        % attempt budget. Archive this generation and immediately create a
        % clean generation for the next invocation.
        history = local_archive_infra_generation(history,next,r,attemptsAlready);
        local_write_history(historyFile,history);
        fprintf("[V30.3-INFRA] H=%.3f V=%.3f gen=%d archived as infrastructure-invalid; retry generation opened without consuming attempt budget.\n", ...
            next.H,next.V,next.generation);
        break;
    end

    history = local_upsert_history(history,next,r,attemptsAlready+attemptsUsedNow,opts);
    local_write_history(historyFile,history);

    if ~r.mission_complete && attemptsAlready+attemptsUsedNow >= round(double(opts.MaxTotalAttemptsPerContext))
        history = local_mark_exhausted(history,next.H,next.V,next.stage,next.generation,next.supportH);
        local_write_history(historyFile,history);
    end
end

[~,stageInfo] = local_select_next_context(history,opts);
local_write_status(statusFile,history,stageInfo,opts);
local_plot_history(history,fullfile(envelopeRoot,"envelope_training_map.png"),opts,stageInfo);

complete = logical(stageInfo.training_complete);
flagPass = fullfile(envelopeRoot,"ENVELOPE_TRAINING_COMPLETE.txt");
flagPending = fullfile(envelopeRoot,"ENVELOPE_TRAINING_PENDING.txt");
if complete
    if isfile(flagPending), delete(flagPending); end
    local_write_flag(flagPass,stageInfo,opts);
else
    if isfile(flagPass), delete(flagPass); end
    local_write_flag(flagPending,stageInfo,opts);
end

result = struct();
result.envelope_root = string(envelopeRoot);
result.history_file = string(historyFile);
result.status_file = string(statusFile);
result.learning_bank_root = string(learningBankRoot);
result.plant_bank_root = string(plantBankRoot);
result.new_mission_runs_this_invocation = newRuns;
result.training_complete = complete;
result.stage = string(stageInfo.stage);
result.qualified_altitude_min_m = double(opts.AltitudeMinM);
result.qualified_altitude_max_m = double(opts.AltitudeMaxM);
result.qualified_speed_min_mps = double(stageInfo.qualified_speed_min_mps);
result.qualified_speed_max_mps = double(stageInfo.qualified_speed_max_mps);
result.speed_lower_boundary_closed = logical(stageInfo.lower_boundary_closed);
result.speed_upper_boundary_closed = logical(stageInfo.upper_boundary_closed);
result.history = history;
result.status = stageInfo;
save(fullfile(envelopeRoot,"v30_envelope_result.mat"),"result","opts","-v7.3");

fprintf("\n[V30-ENVELOPE] stage=%s complete=%d\n",stageInfo.stage,complete);
fprintf("[V30-ENVELOPE] conservative qualified speed interval currently %.3f..%.3f m/s\n", ...
    stageInfo.qualified_speed_min_mps,stageInfo.qualified_speed_max_mps);
if complete
    fprintf("[V30-ENVELOPE] COMPLETE: height anchors, speed-boundary search and interior H/V training all passed.\n");
else
    fprintf("[V30-ENVELOPE] Re-run the SAME command to resume unresolved contexts.\n");
end
end

% -------------------------------------------------------------------------
function [next,info] = local_select_next_context(Hist,opts)
next = local_next_struct(false,NaN,NaN,"",0,NaN);
info = local_stage_info(Hist,opts);

% Stage A/B: progressive nominal-speed descent. Required anchor levels are
% traversed from high to low. An exhausted level is NOT immediately treated
% as a permanent altitude boundary: if the nearest higher PASS has moved
% closer since the failed run, retry the same H in a fresh generation. If it
% has not, insert a midpoint support level and learn that first.
altPoints = sort(unique(double(opts.AltitudeTrainingPointsM(:).')),'descend');
for H = altPoints
    if local_context_state(Hist,H,opts.NominalAirspeedMps,opts) == "pass"
        continue;
    end
    [cand,blocked] = local_resolve_altitude_target(Hist,H,opts,false);
    if cand.exists
        next = cand;
        info.stage = string(cand.stage);
        return;
    end
    if blocked
        info.stage = "blocked_altitude_frontier";
        return;
    end
end

% Do not explore speed until all required nominal-speed altitude anchors pass.
if ~local_all_points_pass(Hist,altPoints,opts.NominalAirspeedMps,opts)
    info.stage = "blocked_altitude_frontier";
    return;
end

% Stage C: coarse speed expansion at the low-altitude stress point.
[coarse,coarseStage] = local_next_coarse_speed(Hist,opts);
if coarse.exists
    next = coarse; info.stage = coarseStage; return;
end

% Stage D: refine a discovered pass/fail speed bracket to configured tolerance.
[refine,refineStage] = local_next_boundary_refine(Hist,opts);
if refine.exists
    next = refine; info.stage = refineStage; return;
end

% Stage E: deterministic interior 2-D contexts. These are ordinary training
% contexts: transferred parameters are tried first, and v29 learns/rebuilds
% only when real certification says that is necessary.
info = local_stage_info(Hist,opts);
if ~info.speed_search_complete
    info.stage = "speed_search_blocked";
    return;
end
cross = local_cross_points(info,opts);
for i=1:size(cross,1)
    H=cross(i,1); V=cross(i,2);
    st=local_context_state(Hist,H,V,opts);
    if st=="pass", continue; end
    if st=="exhausted", continue; end
    gen=local_latest_generation(Hist,H,V);
    next=local_next_struct(true,H,V,"cross_envelope",gen,NaN);
    info.stage="cross_envelope";
    return;
end

% No runnable context remains. Completion requires all interior contexts pass;
% an exhausted interior failure keeps the envelope pending instead of silently
% declaring the full rectangle qualified.
if isempty(cross)
    crossPass=true;
else
    crossPass=true;
    for i=1:size(cross,1)
        crossPass = crossPass && local_context_state(Hist,cross(i,1),cross(i,2),opts)=="pass";
    end
end
info.cross_training_complete=logical(crossPass);
info.training_complete=logical(info.altitude_training_complete && info.speed_search_complete && crossPass);
if info.training_complete, info.stage="complete"; else, info.stage="cross_envelope_has_failures"; end
end

function [next,blocked] = local_resolve_altitude_target(Hist,H,opts,isRefine)
next=local_next_struct(false,NaN,NaN,"",0,NaN); blocked=false;
V=double(opts.NominalAirspeedMps);
st=local_context_state(Hist,H,V,opts);
if st=="pass", return; end
[supportH,hasSupport]=local_nearest_pass_above(Hist,H,V,opts);
if ~hasSupport
    blocked=true;
    return;
end
[idx,gen]=local_latest_row(Hist,H,V);
if gen<0, gen=0; end

if st=="unseen" || st=="retry"
    if isRefine, stage="altitude_refine"; else, stage="altitude_anchor"; end
    next=local_next_struct(true,H,V,stage,gen,supportH);
    return;
end

% The same altitude may have exhausted before a closer higher-altitude PASS
% existed. Retry it in a new output generation instead of reusing the stale
% trim checkpoint. All generations still share LearningBank/PlantBank.
lastSupport=NaN;
if idx>0 && ismember("support_altitude_m",string(Hist.Properties.VariableNames))
    lastSupport=double(Hist.support_altitude_m(idx));
end
if st=="exhausted" && logical(opts.RetryAltitudeAfterCloserPass) && ...
        isfinite(supportH) && (~isfinite(lastSupport) || ...
        supportH < lastSupport-double(opts.AltitudeSupportImprovementMinM))
    if isRefine, stage="altitude_refine_retry"; else, stage="altitude_retry_after_progress"; end
    next=local_next_struct(true,H,V,stage,gen+1,supportH);
    return;
end

% No improved support exists yet. Build one between the current PASS frontier
% and the failed target, but never refine below AltitudeRefineResolutionM.
% With the v30.3 default 10 m resolution: 200 -> 180 failure inserts 190;
% 190 -> 180 failure does NOT insert 185.
gap=supportH-double(H);
res=max(0.1,double(opts.AltitudeRefineResolutionM));
if st=="exhausted" && gap > res+1e-9
    mid=0.5*(supportH+double(H));
    mid=round(mid/res)*res;
    mid=min(supportH-res,max(double(H)+res,mid));
    if mid <= double(H)+1e-9 || mid >= supportH-1e-9
        blocked=true;
        return;
    end
    [bridge,bridgeBlocked]=local_resolve_altitude_target(Hist,mid,opts,true);
    if bridge.exists
        next=bridge;
        return;
    end
    if bridgeBlocked
        blocked=true;
        return;
    end
end

if st=="exhausted", blocked=true; end
end

function [supportH,found]=local_nearest_pass_above(Hist,H,V,opts)
supportH=NaN; found=false;
if isempty(Hist), return; end
alts=double(Hist.target_altitude_m);
vels=double(Hist.target_airspeed_mps);
pass=logical(Hist.mission_complete);
m=pass & abs(vels-double(V))<1e-6 & alts>double(H)+1e-6;
if ~any(m), return; end
supportH=min(alts(m));
found=isfinite(supportH);
if found && supportH>double(opts.AltitudeMaxM)+1e-6, found=false; supportH=NaN; end
end

function [next,stage] = local_next_coarse_speed(Hist,opts)
next=local_next_struct(false,NaN,NaN,"",0,NaN); stage="speed_coarse";
H=double(opts.SpeedProbeAltitudeM); V0=double(opts.NominalAirspeedMps); dV=double(opts.SpeedStepMps);
for dir=[-1 1]
    if dir<0
        vals=(V0-dV):-dV:double(opts.SpeedSearchMinMps);
    else
        vals=(V0+dV):dV:double(opts.SpeedSearchMaxMps);
    end
    for j=1:numel(vals)
        V=vals(j);
        if j>1, inner=vals(j-1); else, inner=V0; end
        innerState=local_context_state(Hist,H,inner,opts);
        if innerState~="pass", break; end
        st=local_context_state(Hist,H,V,opts);
        if st=="pass", continue; end
        if st=="exhausted", break; end
        gen=local_latest_generation(Hist,H,V);
        next=local_next_struct(true,H,V,"speed_coarse",gen,NaN);
        return;
    end
end
end

function [next,stage] = local_next_boundary_refine(Hist,opts)
next=local_next_struct(false,NaN,NaN,"",0,NaN); stage="speed_boundary_refine";
H=double(opts.SpeedProbeAltitudeM); tol=double(opts.SpeedBoundaryToleranceMps); V0=double(opts.NominalAirspeedMps);
for dir=[-1 1]
    [passV,failV,hasFail,open] = local_speed_bracket(Hist,opts,dir); %#ok<ASGLU>
    if ~hasFail, continue; end
    if abs(passV-failV) <= tol + 1e-12, continue; end
    V=0.5*(passV+failV);
    V=round(V/double(opts.SpeedBoundaryResolutionMps))*double(opts.SpeedBoundaryResolutionMps);
    if abs(V-passV)<1e-9 || abs(V-failV)<1e-9, continue; end
    st=local_context_state(Hist,H,V,opts);
    if st=="pass" || st=="exhausted", continue; end
    gen=local_latest_generation(Hist,H,V);
    next=local_next_struct(true,H,V,"speed_boundary_refine",gen,NaN);
    return;
end
if V0<0, stage="speed_boundary_refine"; end %#ok<UNRCH>
end

function next=local_next_struct(exists,H,V,stage,generation,supportH)
next=struct('exists',logical(exists),'H',double(H),'V',double(V), ...
    'stage',string(stage),'generation',max(0,round(double(generation))), ...
    'supportH',double(supportH));
end

function info = local_stage_info(Hist,opts)
info=struct();
altPoints=unique(double(opts.AltitudeTrainingPointsM(:).'),"stable");
info.altitude_training_complete=local_all_points_pass(Hist,altPoints,opts.NominalAirspeedMps,opts);
[lowPass,lowFail,lowHasFail,lowOpen]=local_speed_bracket(Hist,opts,-1);
[highPass,highFail,highHasFail,highOpen]=local_speed_bracket(Hist,opts,1);
info.qualified_speed_min_mps=lowPass;
info.qualified_speed_max_mps=highPass;
info.lower_failure_mps=lowFail;
info.upper_failure_mps=highFail;
info.lower_boundary_closed=logical(lowHasFail && abs(lowPass-lowFail)<=double(opts.SpeedBoundaryToleranceMps)+1e-12);
info.upper_boundary_closed=logical(highHasFail && abs(highPass-highFail)<=double(opts.SpeedBoundaryToleranceMps)+1e-12);
info.lower_boundary_open_at_search_limit=logical(lowOpen);
info.upper_boundary_open_at_search_limit=logical(highOpen);
lowDone=info.lower_boundary_closed || lowOpen;
highDone=info.upper_boundary_closed || highOpen;
info.speed_search_complete=logical(lowDone && highDone);
info.cross_training_complete=false;
info.training_complete=false;
if ~info.altitude_training_complete
    info.stage="altitude_anchor";
elseif ~info.speed_search_complete
    info.stage="speed_search";
else
    info.stage="cross_envelope";
end
end

function [passV,failV,hasFail,openAtLimit] = local_speed_bracket(Hist,opts,dir)
H=double(opts.SpeedProbeAltitudeM); V0=double(opts.NominalAirspeedMps);
passV=V0; failV=NaN; hasFail=false; openAtLimit=false;
% Include all attempted speeds on the appropriate side, including refined
% midpoints, so the bracket automatically tightens after each mission.
if isempty(Hist)
    sideV=[];
else
    m=abs(double(Hist.target_altitude_m)-H)<1e-6;
    sideV=double(Hist.target_airspeed_mps(m));
    if dir<0, sideV=sideV(sideV<V0-1e-9); else, sideV=sideV(sideV>V0+1e-9); end
end
if dir<0
    passCandidates=V0; failCandidates=[];
    for V=unique(sideV(:).')
        st=local_context_state(Hist,H,V,opts);
        if st=="pass", passCandidates(end+1)=V; end %#ok<AGROW>
        if st=="exhausted", failCandidates(end+1)=V; end %#ok<AGROW>
    end
    passV=min(passCandidates);
    validFail=failCandidates(failCandidates<passV);
    if ~isempty(validFail), failV=max(validFail); hasFail=true; end
    if ~hasFail && passV <= double(opts.SpeedSearchMinMps)+1e-9, openAtLimit=true; end
else
    passCandidates=V0; failCandidates=[];
    for V=unique(sideV(:).')
        st=local_context_state(Hist,H,V,opts);
        if st=="pass", passCandidates(end+1)=V; end %#ok<AGROW>
        if st=="exhausted", failCandidates(end+1)=V; end %#ok<AGROW>
    end
    passV=max(passCandidates);
    validFail=failCandidates(failCandidates>passV);
    if ~isempty(validFail), failV=min(validFail); hasFail=true; end
    if ~hasFail && passV >= double(opts.SpeedSearchMaxMps)-1e-9, openAtLimit=true; end
end
end

function P = local_cross_points(info,opts)
n=max(0,round(double(opts.CrossEnvelopeMissionCount)));
if n==0 || ~isfinite(info.qualified_speed_min_mps) || ~isfinite(info.qualified_speed_max_mps) || ...
        info.qualified_speed_max_mps <= info.qualified_speed_min_mps
    P=zeros(0,2); return;
end
Hmin=double(opts.AltitudeMinM); Hmax=double(opts.AltitudeMaxM);
Vmin=double(info.qualified_speed_min_mps); Vmax=double(info.qualified_speed_max_mps);
P=zeros(n,2);
phi=(sqrt(5)-1)/2;
psi=sqrt(2)-1;
for k=1:n
    fh=mod(0.13+k*phi,1); fv=mod(0.31+k*psi,1);
    fh=0.08+0.84*fh; fv=0.08+0.84*fv;
    H=Hmin+(Hmax-Hmin)*fh;
    V=Vmin+(Vmax-Vmin)*fv;
    H=round(H/double(opts.CrossAltitudeResolutionM))*double(opts.CrossAltitudeResolutionM);
    V=round(V/double(opts.CrossSpeedResolutionMps))*double(opts.CrossSpeedResolutionMps);
    H=min(max(H,Hmin),Hmax); V=min(max(V,Vmin),Vmax);
    P(k,:)=[H V];
end
P=unique(P,'rows','stable');
end

function [history,n] = local_reopen_infrastructure_generations(history,opts)
n=0;
if isempty(history), return; end
% Only inspect the latest generation for each H/V. Earlier failed generations
% remain as an audit trail and cannot repeatedly reopen the same context.
keys=string(round(double(history.target_altitude_m)*1e6)/1e6)+"|"+ ...
    string(round(double(history.target_airspeed_mps)*1e6)/1e6);
[uk,~]=unique(keys,"stable");
for k=1:numel(uk)
    rows=find(keys==uk(k));
    if isempty(rows), continue; end
    gens=double(history.generation(rows)); gens(~isfinite(gens))=0;
    g=max(gens); cand=rows(gens==g); idx=cand(end);
    if logical(history.mission_complete(idx)), continue; end
    if string(history.stage(idx))=="mission_nearpass_pending", continue; end
    H=double(history.target_altitude_m(idx)); V=double(history.target_airspeed_mps(idx));
    if local_is_nominal_subresolution_altitude(H,V,opts), continue; end
    out=string(history.output_root(idx));
    if strlength(out)==0 || ~local_mission_has_infrastructure_failure(out), continue; end
    history.exhausted(idx)=false;
    history.stage(idx)="infra_invalid_archived";
    newGen=g+1;
    if any(keys==uk(k) & double(history.generation)==newGen), continue; end
    row=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),"infra_recovery_pending", ...
        double(history.target_altitude_m(idx)),double(history.target_airspeed_mps(idx)),newGen,double(history.support_altitude_m(idx)),0, ...
        false,false,false,false,"","","","",'VariableNames',local_history_names());
    history=[history;row]; %#ok<AGROW>
    n=n+1;
end
end

function [history,n] = local_reopen_legacy_nearpass_for_universal_recovery(history,opts)
% v30.4 migration: OLD exhausted near-pass contexts get exactly one fresh
% generation so they can use the new universal recovery layer. Eligibility is
% derived from recorded flight/trim metrics, never from a hard-coded altitude.
n=0;
if isempty(history), return; end
keys=string(round(double(history.target_altitude_m)*1e6)/1e6)+"|"+ ...
    string(round(double(history.target_airspeed_mps)*1e6)/1e6);
[uk,~]=unique(keys,"stable");
for k=1:numel(uk)
    rows=find(keys==uk(k));
    if isempty(rows), continue; end
    gens=double(history.generation(rows)); gens(~isfinite(gens))=0;
    g=max(gens); cand=rows(gens==g); idx=cand(end);
    if logical(history.mission_complete(idx)) || ~logical(history.exhausted(idx)), continue; end
    H=double(history.target_altitude_m(idx)); V=double(history.target_airspeed_mps(idx));
    if local_is_nominal_subresolution_altitude(H,V,opts), continue; end
    root=string(history.output_root(idx));
    if strlength(root)==0 || ~isfolder(root), continue; end
    if local_mission_has_universal_recovery_artifact(root), continue; end
    if ~local_mission_is_legacy_nearpass(root,opts), continue; end

    newGen=g+1;
    sameNew=abs(double(history.target_altitude_m)-H)<1e-8 & ...
        abs(double(history.target_airspeed_mps)-V)<1e-8 & double(history.generation)==newGen;
    if any(sameNew), continue; end
    supportH=double(history.support_altitude_m(idx));
    if ~isfinite(supportH)
        [supportH,hasSupport]=local_nearest_pass_above(history,H,V,opts);
        if ~hasSupport, supportH=NaN; end
    end
    history.stage(idx)="legacy_nearpass_archived_for_v30_4";
    row=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),"universal_recovery_pending", ...
        H,V,newGen,supportH,0,false,false,false,false,"","","","", ...
        'VariableNames',local_history_names());
    history=[history;row]; %#ok<AGROW>
    n=n+1;
end
end


function [history,n] = local_reopen_legacy_mission_nearpass_for_v30_6(history,opts)
% v30.6 migration: any OLD exhausted context that already has cfg0..cfg4
% VERIFIED and only failed the complete mission inside the universal near-pass
% region receives one clean generation for bumpless/mission recovery.
% No altitude, speed, or cfg identity is hard-coded.
n=0;
if isempty(history) || ~logical(opts.UniversalMissionNearPassEnabled), return; end
keys=string(round(double(history.target_altitude_m)*1e6)/1e6)+"|"+ ...
    string(round(double(history.target_airspeed_mps)*1e6)/1e6);
[uk,~]=unique(keys,"stable");
for k=1:numel(uk)
    rows=find(keys==uk(k)); if isempty(rows), continue; end
    gens=double(history.generation(rows)); gens(~isfinite(gens))=0;
    g=max(gens); cand=rows(gens==g); idx=cand(end);
    if logical(history.mission_complete(idx)) || ~logical(history.exhausted(idx)), continue; end
    if ~logical(history.all_verified(idx)), continue; end
    H=double(history.target_altitude_m(idx)); V=double(history.target_airspeed_mps(idx));
    if local_is_nominal_subresolution_altitude(H,V,opts), continue; end
    root=string(history.output_root(idx));
    if strlength(root)==0 || ~isfolder(root), continue; end
    if local_mission_has_v30_6_artifact(root), continue; end
    if ~local_mission_is_final_nearpass(root,opts), continue; end

    newGen=g+1;
    sameNew=abs(double(history.target_altitude_m)-H)<1e-8 & ...
        abs(double(history.target_airspeed_mps)-V)<1e-8 & double(history.generation)==newGen;
    if any(sameNew), continue; end
    supportH=double(history.support_altitude_m(idx));
    if ~isfinite(supportH)
        [supportH,hasSupport]=local_nearest_pass_above(history,H,V,opts);
        if ~hasSupport, supportH=NaN; end
    end
    history.stage(idx)="legacy_mission_nearpass_archived_for_v30_6";
    row=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),"mission_nearpass_pending", ...
        H,V,newGen,supportH,0,false,false,false,false,"","","","", ...
        'VariableNames',local_history_names());
    history=[history;row]; %#ok<AGROW>
    n=n+1;
end
end

function [history,n] = local_repair_v30_6_mission_resume_generations(history,opts)
% v30.6.2 lineage repair.  Find the most recent EARLIER generation that has
% cfg0..cfg4 VERIFIED and a real Final-Mission near-pass.  Any later partial,
% generic-retry, or mistakenly infra-labelled generation is converted back to
% mission_nearpass_pending instead of opening yet another generation.
% No H/V/cfg identity is hard-coded.
n=0;
if isempty(history) || ~logical(opts.UniversalMissionNearPassEnabled), return; end
keys=string(round(double(history.target_altitude_m)*1e6)/1e6)+"|"+ ...
    string(round(double(history.target_airspeed_mps)*1e6)/1e6);
[uk,~]=unique(keys,"stable");
for k=1:numel(uk)
    rows=find(keys==uk(k)); if isempty(rows), continue; end
    gens=double(history.generation(rows)); gens(~isfinite(gens))=0;
    g=max(gens); cur=rows(gens==g); ci=cur(end);
    if logical(history.mission_complete(ci)), continue; end
    if string(history.stage(ci))=="mission_nearpass_pending", continue; end

    % Locate authoritative earlier near-pass source.  Do not require it to be
    % immediately previous: this repairs g07 after a polluted g06/g07 lineage.
    prior=rows(gens<g & logical(history.all_verified(rows)));
    sourceFound=false;
    if ~isempty(prior)
        pg=double(history.generation(prior)); [~,ord]=sort(pg,'descend'); prior=prior(ord);
        for j=1:numel(prior)
            root=string(history.output_root(prior(j)));
            if strlength(root)>0 && isfolder(root) && local_mission_is_final_nearpass(root,opts)
                sourceFound=true; break;
            end
        end
    end
    if ~sourceFound, continue; end

    history.stage(ci)="mission_nearpass_pending";
    history.attempts_total(ci)=0;
    history.exhausted(ci)=false;
    history.mission_complete(ci)=false;
    history.final_mission_pass(ci)=false;
    history.updated_at(ci)=string(datetime("now","Format","yyyy-MM-dd HH:mm:ss"));
    n=n+1;
end
end

function tf=local_mission_has_v30_6_artifact(root)
tf=false;
try
    tf=isfile(fullfile(root,"final_mission_validation","mission_nearpass_refinement","rounds.csv"));
catch
end
end

function tf=local_mission_is_final_nearpass(root,opts)
tf=false;
try
    sumFile=fullfile(root,"final_mission_validation","final_mission_summary.csv");
    gateFile=fullfile(root,"final_mission_validation","final_mission_gate_report.csv");
    if ~isfile(sumFile) || ~isfile(gateFile), return; end
    S=readtable(sumFile,'TextType','string');
    if isempty(S), return; end
    if ~ismember("mission_pass",string(S.Properties.VariableNames)) || ...
            ~ismember("hard_fail",string(S.Properties.VariableNames)), return; end
    if local_table_logical(S.mission_pass(1)) || local_table_logical(S.hard_fail(1)), return; end
    G=readtable(gateFile,'TextType','string');
    if height(G)<3 || ~all(ismember(["actual","limit"],string(G.Properties.VariableNames))), return; end
    a=double(G.actual(3:end)); lim=double(G.limit(3:end));
    r=a./max(lim,eps); r=r(isfinite(r)); if isempty(r), return; end
    ratio=max(r);
    tf=ratio>1.0 && ratio<=double(opts.UniversalMissionNearPassGateRatioMax);
catch
    tf=false;
end
end

function v=local_table_logical(x)
if islogical(x), v=logical(x); return; end
if isnumeric(x), v=logical(double(x)~=0); return; end
s=lower(strtrim(string(x))); v=(s=="true" || s=="1" || s=="yes");
end

function [history,n] = local_reopen_legacy_controller_nearpass_for_v30_5(history,opts)
% v30.5 migration: an exhausted mission whose blocking cfg ended in
% unified_learning_failed with a measured gate close to formal PASS gets one
% clean generation to use the new universal controller-refinement layer.
% No altitude, airspeed or cfg identity is hard-coded.
n=0;
if isempty(history) || ~logical(opts.UnifiedControllerNearPassEnabled), return; end
keys=string(round(double(history.target_altitude_m)*1e6)/1e6)+"|"+ ...
    string(round(double(history.target_airspeed_mps)*1e6)/1e6);
[uk,~]=unique(keys,"stable");
for k=1:numel(uk)
    rows=find(keys==uk(k)); if isempty(rows), continue; end
    gens=double(history.generation(rows)); gens(~isfinite(gens))=0;
    g=max(gens); cand=rows(gens==g); idx=cand(end);
    if logical(history.mission_complete(idx)) || ~logical(history.exhausted(idx)), continue; end
    H=double(history.target_altitude_m(idx)); V=double(history.target_airspeed_mps(idx));
    if local_is_nominal_subresolution_altitude(H,V,opts), continue; end
    root=string(history.output_root(idx));
    if strlength(root)==0 || ~isfolder(root), continue; end
    if local_mission_has_controller_nearpass_artifact(root), continue; end
    if ~local_mission_is_controller_nearpass(root,opts), continue; end

    newGen=g+1;
    sameNew=abs(double(history.target_altitude_m)-H)<1e-8 & ...
        abs(double(history.target_airspeed_mps)-V)<1e-8 & double(history.generation)==newGen;
    if any(sameNew), continue; end
    supportH=double(history.support_altitude_m(idx));
    if ~isfinite(supportH)
        [supportH,hasSupport]=local_nearest_pass_above(history,H,V,opts);
        if ~hasSupport, supportH=NaN; end
    end
    history.stage(idx)="legacy_controller_nearpass_archived_for_v30_5";
    row=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),"controller_nearpass_pending", ...
        H,V,newGen,supportH,0,false,false,false,false,"","","","", ...
        'VariableNames',local_history_names());
    history=[history;row]; %#ok<AGROW>
    n=n+1;
end
end

function tf=local_mission_has_controller_nearpass_artifact(root)
tf=false;
try
    f=dir(fullfile(root,"**","controller_nearpass_refinement","rounds.csv"));
    tf=~isempty(f);
catch
end
end

function tf=local_mission_is_controller_nearpass(root,opts)
tf=false; root=string(root);
% The blocking stage must actually be unified controller learning, otherwise a
% near-pass row from an earlier cfg must not reopen a trim/Plant failure.
statusFile=fullfile(root,"all_config_status.csv");
if ~isfile(statusFile), return; end
try
    S=readtable(statusFile,'TextType','string');
    if isempty(S) || ~all(ismember(["config_id","status"],string(S.Properties.VariableNames))), return; end
    last=S(end,:);
    if lower(string(last.status(1)))~="unified_learning_failed", return; end
    cfgId=round(double(last.config_id(1)));
catch
    return;
end
histFile=fullfile(root,sprintf("cfg%d",cfgId),"unified_learning","unified_history.csv");
if ~isfile(histFile), return; end
try
    U=readtable(histFile,'VariableNamingRule','preserve','TextType','string');
    vars=string(U.Properties.VariableNames);
    req=["gate_ratio","formal_pass","hard_fail"];
    if isempty(U) || ~all(ismember(req,vars)), return; end
    gate=local_num_col(U,"gate_ratio");
    formal=local_bool_col(U,"formal_pass");
    hard=local_bool_col(U,"hard_fail");
    valid=isfinite(gate) & gate>1.0 & gate<=double(opts.UnifiedControllerNearPassGateRatioMax) & ~formal & ~hard;
    if ismember("infrastructure_fail",vars), valid=valid & ~local_bool_col(U,"infrastructure_fail"); end
    tf=any(valid);
catch
    tf=false;
end
end

function x=local_num_col(T,name)
try
    raw=T.(char(name));
    if isnumeric(raw) || islogical(raw), x=double(raw); else, x=str2double(string(raw)); end
catch
    x=NaN(height(T),1);
end
x=double(x(:));
end

function x=local_bool_col(T,name)
try
    raw=T.(char(name));
    if islogical(raw), x=raw; elseif isnumeric(raw), x=raw~=0; else
        s=lower(strtrim(string(raw))); x=(s=="1"|s=="true"|s=="yes");
    end
catch
    x=false(height(T),1);
end
x=logical(x(:));
end

function tf = local_mission_has_universal_recovery_artifact(root)
tf=false;
try
    f=dir(fullfile(root,"**","universal_recovery_summary.csv"));
    tf=~isempty(f);
catch
end
end

function tf = local_mission_is_legacy_nearpass(root,opts)
tf=false; root=string(root);
% Explicit trim classifier from v30.3 is authoritative when available.
sumFile=fullfile(root,"v30_mission_summary.csv");
if isfile(sumFile)
    try
        T=readtable(sumFile,'TextType','string');
        if ~isempty(T) && ismember("failure_class",string(T.Properties.VariableNames))
            fc=lower(string(T.failure_class(1)));
            if fc=="near_pass_trim", tf=true; return; end
        end
    catch
    end
end
% A real trim exception may carry its gate ratio only in the error text.
errFile=fullfile(root,"v30_last_error.txt");
if isfile(errFile)
    try
        txt=lower(string(fileread(errFile)));
        if local_real_trim_text(txt)
            tok=regexp(char(txt),'gate ratio\s+([0-9]+(?:\.[0-9]+)?)','tokens','once');
            if ~isempty(tok)
                r=str2double(tok{1});
                if isfinite(r) && r<=double(opts.UniversalRecoveryNearPassGateRatioMax), tf=true; return; end
            end
        end
    catch
    end
end
% plant_probe_failed is often returned as a normal status rather than thrown.
% Inspect the newest real equilibrium-probe summary and use the same gate
% normalization as the new universal recovery path.
try
    F=dir(fullfile(root,"**","equilibrium_probe_summary.csv"));
    if isempty(F), return; end
    [~,ord]=sort([F.datenum],'descend'); F=F(ord);
    for i=1:numel(F)
        T=readtable(fullfile(F(i).folder,F(i).name));
        vars=string(T.Properties.VariableNames);
        req=["equilibrium_height_slope_mps","tail_vz_mps","steady_q_rms_dps", ...
            "equilibrium_airspeed_error_mps","hard_fail","equilibrium_pass"];
        if isempty(T) || ~all(ismember(req,vars)), continue; end
        if logical(T.equilibrium_pass(1)), return; end
        if logical(T.hard_fail(1)), return; end
        ratios=[ ...
            abs(double(T.equilibrium_height_slope_mps(1)))/max(double(opts.UniversalRecoveryProbeMaxHeightSlopeMps),eps); ...
            abs(double(T.tail_vz_mps(1)))/max(double(opts.UniversalRecoveryProbeMaxAbsVzMps),eps); ...
            abs(double(T.steady_q_rms_dps(1)))/max(double(opts.UniversalRecoveryProbeMaxQRmsDps),eps); ...
            abs(double(T.equilibrium_airspeed_error_mps(1)))/max(double(opts.UniversalRecoveryProbeMaxAirspeedErrorMps),eps)];
        ratios=ratios(isfinite(ratios));
        if ~isempty(ratios) && max(ratios)<=double(opts.UniversalRecoveryNearPassGateRatioMax)
            tf=true;
        end
        return;
    end
catch
end
end

function n = local_restore_real_trim_evaluations(bankRoot,history)
% Recover valid controller-evaluation rows that v30.2 may have removed after
% misclassifying a real NoUsableTrim failure as infrastructure. The v30.2
% scrub always created timestamped backups first, so restoration is lossless
% when such a backup is present. Deduplicate by context+candidate signature.
n=0;
file=fullfile(bankRoot,"evaluations.csv");
if isempty(history) || ~isfolder(bankRoot), return; end
realRoots=strings(0,1);
for i=1:height(history)
    out=string(history.output_root(i));
    if strlength(out)==0, continue; end
    if local_mission_has_real_trim_failure(out)
        realRoots(end+1,1)=local_norm_path(out); %#ok<AGROW>
    end
end
realRoots=unique(realRoots(strlength(realRoots)>0));
if isempty(realRoots), return; end
backups=dir(fullfile(bankRoot,"evaluations.csv.v30_2_infra_backup_*.csv"));
if isempty(backups), return; end
[~,ord]=sort([backups.datenum],'descend'); backups=backups(ord);
try
    if isfile(file)
        E=readtable(file,'VariableNamingRule','preserve','TextType','string');
    else
        E=table();
    end
    restored=table();
    for b=1:numel(backups)
        B=readtable(fullfile(backups(b).folder,backups(b).name), ...
            'VariableNamingRule','preserve','TextType','string');
        if isempty(B) || ~ismember("source_output_root",string(B.Properties.VariableNames)), continue; end
        src=local_norm_path(string(B.source_output_root));
        keep=false(height(B),1);
        for r=1:numel(realRoots), keep=keep | (src==realRoots(r)); end
        if ~any(keep), continue; end
        C=B(keep,:);
        if isempty(restored)
            restored=C;
        elseif isequal(string(restored.Properties.VariableNames),string(C.Properties.VariableNames))
            restored=[restored;C]; %#ok<AGROW>
        end
    end
    if isempty(restored), return; end
    varsR=string(restored.Properties.VariableNames);
    if ~ismember("context_signature",varsR) || ~ismember("candidate_signature",varsR), return; end
    rkey=string(restored.context_signature)+"|"+string(restored.candidate_signature);
    [~,ria]=unique(rkey,'stable'); restored=restored(sort(ria),:); rkey=rkey(sort(ria));
    if isempty(E)
        E=restored; n=height(restored);
    else
        if ~isequal(string(E.Properties.VariableNames),string(restored.Properties.VariableNames)), return; end
        ekey=string(E.context_signature)+"|"+string(E.candidate_signature);
        add=~ismember(rkey,ekey);
        n=nnz(add);
        if n>0, E=[E;restored(add,:)]; end %#ok<AGROW>
    end
    if n>0, writetable(E,file); end
catch ME
    warning("AirdropX:V30:RealTrimBankRestore", ...
        "Could not restore v30.2 misclassified real-trim evaluations: %s",ME.message);
    n=0;
end
end

function tf = local_mission_has_real_trim_failure(root)
tf=false; root=string(root); if strlength(root)==0 || ~isfolder(root), return; end
errFile=fullfile(root,"v30_last_error.txt");
if isfile(errFile)
    try
        if local_real_trim_text(lower(string(fileread(errFile)))), tf=true; return; end
    catch
    end
end
sumFile=fullfile(root,"v30_mission_summary.csv");
if isfile(sumFile)
    try
        T=readtable(sumFile,'TextType','string');
        if ~isempty(T) && ismember("failure_class",string(T.Properties.VariableNames))
            fc=lower(string(T.failure_class(1)));
            tf=(fc=="near_pass_trim" || fc=="real_trim_fail");
        end
    catch
    end
end
end

function n = local_scrub_global_infra_evaluations(bankRoot,history)
n=0;
file=fullfile(bankRoot,"evaluations.csv");
if ~isfile(file) || isempty(history), return; end
badRoots=strings(0,1);
for i=1:height(history)
    out=string(history.output_root(i));
    if strlength(out)==0, continue; end
    % v30.3: never scrub a whole mission merely because an old history row
    % says infra_invalid_archived. Re-classify from the mission artifacts.
    if local_mission_has_infrastructure_failure(out)
        badRoots(end+1,1)=local_norm_path(out); %#ok<AGROW>
    end
end
badRoots=unique(badRoots(strlength(badRoots)>0));
if isempty(badRoots), return; end
try
    E=readtable(file,'VariableNamingRule','preserve','TextType','string');
    if isempty(E) || ~ismember("source_output_root",string(E.Properties.VariableNames)), return; end
    src=local_norm_path(string(E.source_output_root));
    bad=false(height(E),1);
    for i=1:numel(badRoots), bad=bad | (src==badRoots(i)); end
    n=nnz(bad);
    if n==0, return; end
    backup=string(file)+".v30_2_infra_backup_"+string(datetime("now","Format","yyyyMMdd_HHmmss"))+".csv";
    copyfile(file,backup);
    E=E(~bad,:);
    writetable(E,file);
catch ME
    warning("AirdropX:V30:InfraBankScrub", ...
        "Could not scrub legacy infrastructure-invalid LearningBank evaluations: %s",ME.message);
    n=0;
end
end

function p = local_norm_path(p)
p=lower(strtrim(string(p)));
p=replace(p,"\\","/");
p=replace(p,"\","/");
while any(contains(p,"//")), p=replace(p,"//","/"); end
end

function history = local_archive_infra_generation(history,next,r,attemptsAlready)
% Store the failed output path on the current generation, but do not charge
% attempts. Then append a fresh generation so stale slprj/checkpoints cannot
% poison resume.
row=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),"infra_invalid_archived", ...
    double(next.H),double(next.V),round(double(next.generation)),double(next.supportH),round(double(attemptsAlready)), ...
    false,false,false,false,string(r.output_root),string(r.master_mat),string(r.plant_seed_mat),string(r.plant_seed_source), ...
    'VariableNames',local_history_names());
if isempty(history)
    history=row;
else
    same=abs(double(history.target_altitude_m)-next.H)<1e-8 & ...
        abs(double(history.target_airspeed_mps)-next.V)<1e-8 & ...
        double(history.generation)==round(double(next.generation));
    if any(same)
        idx=find(same,1,"last"); history(idx,:)=row;
    else
        history=[history;row]; %#ok<AGROW>
    end
end
newGen=round(double(next.generation))+1;
newRow=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),"infra_recovery_pending", ...
    double(next.H),double(next.V),newGen,double(next.supportH),0,false,false,false,false,"","","","", ...
    'VariableNames',local_history_names());
sameNew=abs(double(history.target_altitude_m)-next.H)<1e-8 & ...
    abs(double(history.target_airspeed_mps)-next.V)<1e-8 & double(history.generation)==newGen;
if ~any(sameNew), history=[history;newRow]; end %#ok<AGROW>
end

function tf = local_mission_has_infrastructure_failure(root)
tf=false; root=string(root); if strlength(root)==0 || ~isfolder(root), return; end

% v30.6.2: a readable Final Mission result after cfg0..cfg4 VERIFIED is
% authoritative REAL control evidence.  Stale worker folders (including a
% missing certification_summary.csv from an abandoned eval) cannot reclassify
% the whole mission as infrastructure failure.
if local_mission_has_authoritative_final_result(root), return; end

% v30.3: explicit REAL trim failure has priority over stale missing-summary
% artifacts or an old v30.2 summary that incorrectly labelled the mission as
% infrastructure-invalid.
errFile=fullfile(root,"v30_last_error.txt");
if isfile(errFile)
    try
        txt=lower(string(fileread(errFile)));
        if local_real_trim_text(txt), return; end
    catch
    end
end

% New v30.3 mission summary is authoritative when it contains a failure class.
sumFile=fullfile(root,"v30_mission_summary.csv");
if isfile(sumFile)
    try
        T=readtable(sumFile,'TextType','string');
        vars=string(T.Properties.VariableNames);
        if ~isempty(T) && ismember("failure_class",vars)
            fc=lower(string(T.failure_class(1)));
            if fc=="near_pass_trim" || fc=="real_trim_fail", return; end
            if startsWith(fc,"infra_") || fc=="infra_invalid", tf=true; return; end
        end
        if ~isempty(T) && ismember("infrastructure_failure",vars) && local_history_logical(T.infrastructure_failure(1))
            % Legacy v30.2 summary: only trust it when no explicit real trim
            % evidence was found above.
            tf=true; return;
        end
    catch
    end
end

% Legacy v30.1/v30.2: inspect actual evaluation artifacts. Before calling a
% missing summary infrastructure, scan errors for real trim evidence.
try
    e=[dir(fullfile(root,"**","error.txt")); dir(fullfile(root,"**","bayesopt_wrapper_error.txt"))];
    realTrimSeen=false; infraSeen=false;
    for i=1:numel(e)
        try
            txt=lower(string(fileread(fullfile(e(i).folder,e(i).name))));
            if local_real_trim_text(txt), realTrimSeen=true; end
            if local_infra_text(txt), infraSeen=true; end
        catch
        end
    end
    if realTrimSeen, return; end
    if infraSeen, tf=true; return; end
catch
end
try
    rec=dir(fullfile(root,"**","unified_record.csv"));
    for i=1:numel(rec)
        evalDir=rec(i).folder;
        if ~isfile(fullfile(evalDir,"certification_summary.csv"))
            tf=true; return;
        end
        try
            R=readtable(fullfile(rec(i).folder,rec(i).name));
            if ismember("infrastructure_fail",string(R.Properties.VariableNames)) && any(local_history_logical(R.infrastructure_fail))
                tf=true; return;
            end
        catch
        end
    end
catch
end
end

function tf = local_mission_has_authoritative_final_result(root)
tf=false;
try
    statusFile=fullfile(root,"all_config_status.csv");
    sumFile=fullfile(root,"final_mission_validation","final_mission_summary.csv");
    if ~isfile(statusFile) || ~isfile(sumFile), return; end
    C=readtable(statusFile,'TextType','string');
    if isempty(C) || ~all(ismember(["config_id","status"],string(C.Properties.VariableNames))), return; end
    ids=round(str2double(string(C.config_id))); st=lower(string(C.status));
    for cfg=0:4
        ii=find(ids==cfg,1,'last');
        if isempty(ii) || st(ii)~="verified", return; end
    end
    S=readtable(sumFile,'TextType','string');
    vars=string(S.Properties.VariableNames);
    if isempty(S) || ~all(ismember(["mission_pass","hard_fail"],vars)), return; end
    % PASS, near-pass, and real mission hard-fail are all completed control
    % observations.  None of them is infrastructure merely because an older
    % eval directory elsewhere under the root is incomplete.
    tf=true;
catch
    tf=false;
end
end

function tf = local_real_trim_text(txt)
txt=lower(string(txt));
markers=["airdropx:autompc:nousabletrim","trim failed best verification", ...
    "trim verification returned no samples","autompc:emptytrimrun"];
tf=false;
for i=1:numel(markers)
    if contains(txt,markers(i)), tf=true; return; end
end
end

function tf = local_infra_text(txt)
markers=["infrastructurefailure","infrastructure_exception","path too long", ...
    "filename or extension is too long","file name or extension is too long", ...
    "specified path, file name, or both are too long","260 character","max_path", ...
    "database is full","no space left","disk full","fetchnextfutureerrored","missinglogsout","missinglog"];
tf=false; for i=1:numel(markers), if contains(txt,markers(i)), tf=true; return; end, end
if contains(txt,"certification_summary.csv") && ...
        (contains(txt,"not found") || contains(txt,"does not exist") || contains(txt,"unable to find") || contains(txt,"no such file"))
    tf=true;
end
end

function [history,n] = local_archive_subresolution_altitude_contexts(history,opts)
% v30.3 migration: keep old 5 m refinement data (e.g. H185/H195) on disk
% and in history, but retire unfinished contexts from active scheduling when
% the new minimum altitude spacing is 10 m. Successful off-grid missions are
% never discarded and remain valid LearningBank/PlantBank knowledge.
n=0;
if isempty(history), return; end
keys=string(round(double(history.target_altitude_m)*1e6)/1e6)+"|"+ ...
    string(round(double(history.target_airspeed_mps)*1e6)/1e6);
[uk,~]=unique(keys,"stable");
for k=1:numel(uk)
    rows=find(keys==uk(k));
    if isempty(rows) || any(logical(history.mission_complete(rows))), continue; end
    gens=double(history.generation(rows)); gens(~isfinite(gens))=0;
    g=max(gens); cand=rows(gens==g); idx=cand(end);
    H=double(history.target_altitude_m(idx)); V=double(history.target_airspeed_mps(idx));
    if ~local_is_nominal_subresolution_altitude(H,V,opts), continue; end
    if string(history.stage(idx))~="subresolution_archived"
        history.stage(idx)="subresolution_archived";
        history.exhausted(idx)=false;
        history.updated_at(idx)=string(datetime("now","Format","yyyy-MM-dd HH:mm:ss"));
        n=n+1;
    end
end
end

function tf = local_is_nominal_subresolution_altitude(H,V,opts)
tf=false;
if abs(double(V)-double(opts.NominalAirspeedMps))>1e-6, return; end
res=max(0.1,double(opts.AltitudeRefineResolutionM));
anchor=double(opts.AnchorAltitudeM);
k=(anchor-double(H))/res;
tf=isfinite(k) && abs(k-round(k))>1e-6;
end

function history = local_upsert_history(history,next,r,totalAttempts,opts)
row=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),string(next.stage), ...
    double(next.H),double(next.V),round(double(next.generation)),double(next.supportH),round(double(totalAttempts)), ...
    logical(r.all_verified),logical(r.final_mission_pass),logical(r.mission_complete), ...
    false,string(r.output_root),string(r.master_mat),string(r.plant_seed_mat),string(r.plant_seed_source), ...
    'VariableNames',local_history_names());
if isempty(history), history=row; return; end
same=abs(double(history.target_altitude_m)-next.H)<1e-8 & ...
    abs(double(history.target_airspeed_mps)-next.V)<1e-8 & ...
    double(history.generation)==round(double(next.generation));
if any(same)
    idx=find(same,1,"last"); history(idx,:)=row;
    remove=find(same); remove(remove==idx)=[]; history(remove,:)=[];
else
    history=[history;row]; %#ok<AGROW>
end
% Exhaustion is set separately after the current generation budget is used.
if totalAttempts < round(double(opts.MaxTotalAttemptsPerContext)) && ~r.mission_complete
    same=abs(double(history.target_altitude_m)-next.H)<1e-8 & ...
        abs(double(history.target_airspeed_mps)-next.V)<1e-8 & ...
        double(history.generation)==round(double(next.generation));
    idx=find(same,1,"last");
    if ~isempty(idx), history.exhausted(idx)=false; end
end
end

function history = local_mark_exhausted(history,H,V,stage,generation,supportH)
row=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),string(stage), ...
    double(H),double(V),round(double(generation)),double(supportH),0,false,false,false,true,"","","","", ...
    'VariableNames',local_history_names());
if isempty(history), history=row; return; end
same=abs(double(history.target_altitude_m)-H)<1e-8 & ...
    abs(double(history.target_airspeed_mps)-V)<1e-8 & ...
    double(history.generation)==round(double(generation));
if any(same)
    idx=find(same,1,"last");
    history.exhausted(idx)=true;
    history.stage(idx)=string(stage);
    history.support_altitude_m(idx)=double(supportH);
    history.updated_at(idx)=string(datetime("now","Format","yyyy-MM-dd HH:mm:ss"));
else
    history=[history;row];
end
end

function n = local_attempts_for(history,H,V,generation)
n=0; if isempty(history), return; end
m=abs(double(history.target_altitude_m)-H)<1e-8 & ...
    abs(double(history.target_airspeed_mps)-V)<1e-8 & ...
    double(history.generation)==round(double(generation));
if any(m)
    n=max(double(history.attempts_total(m)),[],"omitnan");
    if ~isfinite(n), n=0; end
end
end

function tf = local_generation_is_mission_recovery(history,H,V,generation)
% v30.6.1: keep mission-only mode sticky across reruns. It is true either
% when the current generation was explicitly opened as mission_nearpass_pending
% or when its immediately previous generation was archived by v30.6 after an
% all-VERIFIED Final-Mission near-pass. This also repairs a partially-run g06
% produced by the old v30.6 bug, whose row may already have a generic stage.
tf=false;
if isempty(history), return; end
same=find(abs(double(history.target_altitude_m)-H)<1e-6 & ...
    abs(double(history.target_airspeed_mps)-V)<1e-6);
if isempty(same), return; end
cur=same(double(history.generation(same))==double(generation));
if ~isempty(cur)
    st=string(history.stage(cur(end)));
    if st=="mission_nearpass_pending"
        tf=true; return;
    end
end
prev=same(double(history.generation(same))==double(generation)-1);
if isempty(prev), return; end
i=prev(end); st=string(history.stage(i));
if logical(history.all_verified(i)) && contains(st,"mission_nearpass_archived_for_v30_6")
    tf=true;
end
end

function root = local_previous_mission_nearpass_source_root(history,H,V,currentGeneration,opts)
% v30.6.2: prefer the generation that actually established the authoritative
% all-VERIFIED Final-Mission near-pass baseline.  This prevents a later failed
% transition-policy generation (for example move=1/integral=1) from becoming
% the next recovery seed merely because its inherited cfg states are VERIFIED.
root="";
if isempty(history), return; end
m=find(abs(double(history.target_altitude_m)-H)<1e-6 & ...
    abs(double(history.target_airspeed_mps)-V)<1e-6 & ...
    double(history.generation)<double(currentGeneration) & logical(history.all_verified));
if isempty(m), return; end
g=double(history.generation(m)); g(~isfinite(g))=-Inf;
[~,ord]=sort(g,'descend'); m=m(ord);
for i=1:numel(m)
    r=string(history.output_root(m(i)));
    if strlength(r)>0 && isfolder(r) && local_mission_is_final_nearpass(r,opts)
        cp=fullfile(r,"airdropx_200m_cfg_checkpoint.mat");
        master=fullfile(r,"identified_plants_200m_master.mat");
        if isfile(cp) && isfile(master), root=r; return; end
    end
end
% Backward-compatible fallback when historical final-mission artifacts are
% unavailable but a valid all-VERIFIED generation exists.
root=local_previous_verified_generation_root(history,H,V,currentGeneration);
end

function root = local_previous_verified_generation_root(history,H,V,currentGeneration)
% v30.6.1: mission-only recovery inherits the most recent earlier generation
% that already had every cfg VERIFIED. No altitude/cfg special case is used.
root="";
if isempty(history), return; end
m=find(abs(double(history.target_altitude_m)-H)<1e-6 & ...
    abs(double(history.target_airspeed_mps)-V)<1e-6 & ...
    double(history.generation)<double(currentGeneration) & logical(history.all_verified));
if isempty(m), return; end
g=double(history.generation(m)); g(~isfinite(g))=-Inf;
[~,ord]=sort(g,'descend'); m=m(ord);
for i=1:numel(m)
    r=string(history.output_root(m(i)));
    if strlength(r)>0 && isfolder(r)
        cp=fullfile(r,"airdropx_200m_cfg_checkpoint.mat");
        master=fullfile(r,"identified_plants_200m_master.mat");
        if isfile(cp) && isfile(master)
            root=r; return;
        end
    end
end
end

function st = local_context_state(history,H,V,opts)
st="unseen"; if isempty(history), return; end
m=abs(double(history.target_altitude_m)-H)<1e-6 & abs(double(history.target_airspeed_mps)-V)<1e-6;
if ~any(m), return; end
if any(logical(history.mission_complete(m)))
    st="pass";
    return;
end
[idx,~]=local_latest_row(history,H,V);
if idx<=0, return; end
if logical(history.exhausted(idx)) || double(history.attempts_total(idx))>=round(double(opts.MaxTotalAttemptsPerContext))
    st="exhausted";
else
    st="retry";
end
end

function [idx,generation]=local_latest_row(history,H,V)
idx=0; generation=-1;
if isempty(history), return; end
m=find(abs(double(history.target_altitude_m)-H)<1e-6 & abs(double(history.target_airspeed_mps)-V)<1e-6);
if isempty(m), return; end
g=double(history.generation(m));
g(~isfinite(g))=0;
generation=max(g);
cand=m(g==generation);
idx=cand(end);
end

function generation=local_latest_generation(history,H,V)
[~,generation]=local_latest_row(history,H,V);
if generation<0, generation=0; end
end

function tf=local_all_points_pass(history,Hpoints,V,opts)
tf=true; for H=double(Hpoints(:).'), tf=tf && local_context_state(history,H,V,opts)=="pass"; end
end

function history=local_import_anchor_history(history,paths,opts)
root=local_resolve_path(paths.projectRoot,opts.AnchorOutputRoot);
[allVerified,missionPass]=local_root_status(root);
if ~(allVerified && missionPass), return; end
H=double(opts.AnchorAltitudeM); V=double(opts.AnchorAirspeedMps); master=fullfile(root,"identified_plants_200m_master.mat");
row=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),"existing_anchor",H,V,0,NaN,0,true,true,true,false,string(root),string(master),string(master),"existing_v29_anchor", ...
    'VariableNames',local_history_names());
if isempty(history), history=row; return; end
same=abs(double(history.target_altitude_m)-H)<1e-8 & abs(double(history.target_airspeed_mps)-V)<1e-8 & double(history.generation)==0;
if any(same), history(find(same,1,"last"),:)=row; else, history=[history;row]; end
end

function local_register_existing_anchor(paths,plantBankRoot,opts)
root=local_resolve_path(paths.projectRoot,opts.AnchorOutputRoot);
master=fullfile(root,"identified_plants_200m_master.mat");
[allVerified,missionPass]=local_root_status(root);
if isfile(master) && allVerified
    airdropx_auto_plant_context_bank("Action","register","ProjectRoot",paths.projectRoot,"Root",plantBankRoot, ...
        "TargetAltitudeM",opts.AnchorAltitudeM,"TargetAirspeedMps",opts.AnchorAirspeedMps, ...
        "ReferenceMassKg",opts.ReferenceMassKg,"CargoMassKg",opts.CargoMassKg,"TotalDropCount",opts.TotalDropCount, ...
        "OutputRoot",root,"MasterMat",master,"AllVerified",allVerified,"MissionPass",missionPass,"PlantValid",allVerified,"Source","existing_v29_anchor");
end
end

function [allVerified,missionPass]=local_root_status(root)
allVerified=false; missionPass=false;
cpFile=fullfile(root,"airdropx_200m_cfg_checkpoint.mat");
if isfile(cpFile)
    try
        S=load(cpFile,"checkpoint"); cp=S.checkpoint;
        if isfield(cp,"status") && numel(cp.status)>=5, allVerified=all(string(cp.status(1:5))=="verified"); end
        if isfield(cp,"final_mission_pass"), missionPass=logical(cp.final_mission_pass); end
    catch
    end
end
sumFile=fullfile(root,"final_mission_validation","final_mission_summary.csv");
if isfile(sumFile)
    try T=readtable(sumFile); if ~isempty(T)&&ismember("mission_pass",string(T.Properties.VariableNames)), missionPass=logical(T.mission_pass(1)); end, catch, end
end
end

function [nContexts,nPass,nExhausted]=local_context_status_counts(history)
nContexts=0; nPass=0; nExhausted=0; if isempty(history), return; end
keys=string(round(double(history.target_altitude_m)*1e6)/1e6)+"|"+string(round(double(history.target_airspeed_mps)*1e6)/1e6);
[uk,~]=unique(keys,"stable"); nContexts=numel(uk);
for k=1:numel(uk)
    rows=find(keys==uk(k));
    if any(logical(history.mission_complete(rows))), nPass=nPass+1; continue; end
    gens=double(history.generation(rows)); gens(~isfinite(gens))=0;
    g=max(gens); cand=rows(gens==g); idx=cand(end);
    if logical(history.exhausted(idx)), nExhausted=nExhausted+1; end
end
end

function local_write_status(file,history,info,opts)
[nContexts,nPass,nExhausted]=local_context_status_counts(history);
T=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),string(info.stage), ...
    logical(info.altitude_training_complete),logical(info.speed_search_complete),logical(info.cross_training_complete),logical(info.training_complete), ...
    double(opts.AltitudeMinM),double(opts.AltitudeMaxM),double(info.qualified_speed_min_mps),double(info.qualified_speed_max_mps), ...
    logical(info.lower_boundary_closed),logical(info.upper_boundary_closed),logical(info.lower_boundary_open_at_search_limit),logical(info.upper_boundary_open_at_search_limit), ...
    nContexts,nPass,nExhausted, ...
    'VariableNames',{'updated_at','stage','altitude_training_complete','speed_search_complete','cross_training_complete','training_complete', ...
    'altitude_min_m','altitude_max_m','qualified_speed_min_mps','qualified_speed_max_mps','lower_boundary_closed','upper_boundary_closed', ...
    'lower_boundary_open_at_search_limit','upper_boundary_open_at_search_limit','contexts_recorded','contexts_passed','contexts_exhausted'});
writetable(T,file);
end

function local_plot_history(Hist,file,opts,info)
if isempty(Hist), return; end
try
    fig=figure('Visible','off','Color','w','Position',[100 100 1200 760]); hold on; grid on; box on;
    pass=logical(Hist.mission_complete); fail=logical(Hist.exhausted)&~pass; pending=~pass&~fail;
    if any(pass), scatter(double(Hist.target_airspeed_mps(pass)),double(Hist.target_altitude_m(pass)),70,'o','filled','DisplayName','MISSION PASS'); end
    if any(fail), scatter(double(Hist.target_airspeed_mps(fail)),double(Hist.target_altitude_m(fail)),90,'x','LineWidth',1.8,'DisplayName','exhausted / boundary fail'); end
    if any(pending), scatter(double(Hist.target_airspeed_mps(pending)),double(Hist.target_altitude_m(pending)),70,'s','DisplayName','pending'); end
    xlabel('Target airspeed (m/s)'); ylabel('Target altitude (m)');
    title(sprintf('AirdropX v30 envelope training | current conservative V %.2f..%.2f m/s',info.qualified_speed_min_mps,info.qualified_speed_max_mps));
    xlim([double(opts.SpeedSearchMinMps)-2,double(opts.SpeedSearchMaxMps)+2]); ylim([double(opts.AltitudeMinM)-5,double(opts.AltitudeMaxM)+5]);
    legend('Location','best'); exportgraphics(fig,file,'Resolution',160); close(fig);
catch
end
end

function local_write_flag(file,info,opts)
fid=fopen(file,"w"); if fid<0, return; end
fprintf(fid,"training_complete=%d\n",logical(info.training_complete));
fprintf(fid,"stage=%s\n",char(string(info.stage)));
fprintf(fid,"altitude_min_m=%.9g\n",opts.AltitudeMinM); fprintf(fid,"altitude_max_m=%.9g\n",opts.AltitudeMaxM);
fprintf(fid,"qualified_speed_min_mps=%.9g\n",info.qualified_speed_min_mps); fprintf(fid,"qualified_speed_max_mps=%.9g\n",info.qualified_speed_max_mps);
fprintf(fid,"lower_boundary_closed=%d\n",logical(info.lower_boundary_closed)); fprintf(fid,"upper_boundary_closed=%d\n",logical(info.upper_boundary_closed));
fclose(fid);
end

function history=local_read_history(file)
if ~isfile(file), history=local_empty_history(); return; end
try
    history=readtable(file,'TextType','string');
catch
    history=readtable(file);
end
if isempty(history) && width(history)==0, history=local_empty_history(); return; end
% v30.1 schema migration: old v30 histories had no generation/support fields.
% Keep them valid so the previously exhausted 20/50 record can be retried
% automatically after closer altitude anchors have passed.
vars=string(history.Properties.VariableNames);
if ~ismember("generation",vars), history.generation=zeros(height(history),1); end
if ~ismember("support_altitude_m",vars), history.support_altitude_m=NaN(height(history),1); end
% Normalize CSV-inferred classes so old numeric 0/1 histories and newer
% logical histories can be assigned the same typed rows safely.
history.updated_at=string(history.updated_at); history.stage=string(history.stage);
history.target_altitude_m=double(history.target_altitude_m); history.target_airspeed_mps=double(history.target_airspeed_mps);
history.generation=double(history.generation); history.support_altitude_m=double(history.support_altitude_m); history.attempts_total=double(history.attempts_total);
history.all_verified=local_history_logical(history.all_verified); history.final_mission_pass=local_history_logical(history.final_mission_pass);
history.mission_complete=local_history_logical(history.mission_complete); history.exhausted=local_history_logical(history.exhausted);
history.output_root=string(history.output_root); history.master_mat=string(history.master_mat);
history.plant_seed_mat=string(history.plant_seed_mat); history.plant_seed_source=string(history.plant_seed_source);
names=local_history_names();
for i=1:numel(names)
    if ~ismember(string(names{i}),string(history.Properties.VariableNames))
        error("AirdropX:V30:HistorySchema","Missing history column after migration: %s",names{i});
    end
end
history=history(:,names);
end
function y=local_history_logical(x)
if islogical(x), y=x; return; end
if isnumeric(x), y=(x~=0); return; end
t=lower(strtrim(string(x))); y=(t=="true") | (t=="1") | (t=="yes");
end
function H=local_empty_history()
H=table(strings(0,1),strings(0,1),zeros(0,1),zeros(0,1),zeros(0,1),NaN(0,1),zeros(0,1), ...
    false(0,1),false(0,1),false(0,1),false(0,1), ...
    strings(0,1),strings(0,1),strings(0,1),strings(0,1),'VariableNames',local_history_names());
end
function local_write_history(file,H)
if ~isempty(H), writetable(H,file); end
end
function names=local_history_names()
names={'updated_at','stage','target_altitude_m','target_airspeed_mps','generation','support_altitude_m','attempts_total', ...
    'all_verified','final_mission_pass','mission_complete','exhausted','output_root','master_mat','plant_seed_mat','plant_seed_source'};
end
function tag=local_context_tag(H,V), tag="H"+local_num_tag(H)+"_V"+local_num_tag(V); end
function s=local_num_tag(x), s=sprintf("%.3f",double(x)); s=strrep(s,"-","m"); s=strrep(s,".","p"); end
function paths=local_paths(projectRoot)
projectRoot=string(projectRoot); if strlength(projectRoot)==0, projectRoot=string(pwd); end
paths.projectRoot=projectRoot; paths.matlabDir=fullfile(projectRoot,"matlab"); paths.mpcDir=fullfile(paths.matlabDir,"mpc"); paths.autoDir=fullfile(paths.matlabDir,"mpc_auto"); paths.sfuncDir=fullfile(paths.matlabDir,"sfunc_jsbsim");
end
function p=local_resolve_path(projectRoot,p)
p=string(p); if strlength(p)==0, return; end
c=char(p); isAbs=startsWith(c,'/')||startsWith(c,'\\')||~isempty(regexp(c,'^[A-Za-z]:[\\/]','once')); if ~isAbs, p=string(fullfile(projectRoot,p)); end
end

function opts=local_options(varargin)
opts=struct();
opts.ProjectRoot="";
opts.BaseIdentifiedMat="matlab/results/mpc_auto_id_v11_clean_r1/identify/airdropx_identified_plants.mat";
opts.AnchorOutputRoot="matlab/results/mpc_auto_200m_all_cfg_v16";
opts.AnchorAltitudeM=200.0;
opts.AnchorAirspeedMps=50.0;
opts.EnvelopeRoot="matlab/results/mpc_auto_flight_envelope_v30";
opts.LearningBankRoot="matlab/results/mpc_auto_global_learning_bank";
opts.PlantBankRoot="matlab/results/mpc_auto_global_plant_bank";
opts.AltitudeMinM=20.0; opts.AltitudeMaxM=200.0;
opts.NominalAirspeedMps=50.0;
% v30.1 progressive altitude curriculum: use neighboring verified contexts
% instead of jumping directly from 200 m to the 20 m stress point.
opts.AltitudeTrainingPointsM=200:-20:20;
opts.AltitudeRefineResolutionM=10.0;
opts.RetryAltitudeAfterCloserPass=true;
opts.AltitudeSupportImprovementMinM=0.5;
opts.SpeedProbeAltitudeM=20.0;
opts.SpeedSearchMinMps=30.0; opts.SpeedSearchMaxMps=70.0;
opts.SpeedStepMps=5.0;
opts.SpeedBoundaryToleranceMps=1.0;
opts.SpeedBoundaryResolutionMps=0.25;
opts.CrossEnvelopeMissionCount=8;
opts.CrossAltitudeResolutionM=1.0;
opts.CrossSpeedResolutionMps=0.25;
opts.ReferenceMassKg=3423.0; opts.CargoMassKg=300.0; opts.TotalDropCount=4;
opts.UseParallel=true; opts.ParallelWorkers=3;
opts.MaxAttemptsPerMissionCall=2;
opts.MaxTotalAttemptsPerContext=4;
opts.MaxNewMissionRunsPerInvocation=Inf;
opts.UnifiedTransferSeedEvaluations=5;
opts.UnifiedAdditionalEvaluationsPerRun=36;
% v30.5 universal controller near-pass refinement, identical for every H/V/cfg.
opts.UnifiedControllerNearPassEnabled=true;
opts.UnifiedControllerNearPassGateRatioMax=1.30;
opts.UnifiedControllerNearPassDeterministicEvaluations=16;
opts.UnifiedControllerNearPassBayesEvaluations=18;
opts.UnifiedControllerNearPassMaxRoundsPerContext=2;
% v30.6 universal mission-level recovery. The same cfg-transition bridge and
% near-pass classifier are used for every altitude/airspeed context.
opts.BumplessTransitionEnabled=true;
opts.TransitionMoveTransferScale=0.0;
opts.TransitionIntegralTransferScale=0.0;
opts.UniversalMissionNearPassEnabled=true;
opts.UniversalMissionNearPassGateRatioMax=1.30;
opts.UniversalMissionNearPassMaxNewEvaluationsPerAttempt=4;
opts.UniversalMissionNearPassMoveScales=[0.00;0.00;0.25;0.25;0.50;0.50;0.75;1.00];
opts.UniversalMissionNearPassIntegralScales=[0.00;0.50;0.00;0.50;0.00;0.50;0.50;1.00];
% v30.4 universal near-pass recovery is one policy for every H/V context.
opts.UniversalRecoveryNearPassGateRatioMax=2.0;
opts.UniversalRecoveryExtendedProbeDurationS=80.0;
opts.UniversalRecoveryExtendedTailWindowS=20.0;
opts.UniversalRecoveryLocalRetrimEvaluations=24;
% Mirrors the equilibrium-probe gates used by the unified MPC pipeline; only
% used to migrate OLD exhausted contexts into one v30.4 retry generation.
opts.UniversalRecoveryProbeMaxHeightSlopeMps=0.08;
opts.UniversalRecoveryProbeMaxAbsVzMps=0.10;
opts.UniversalRecoveryProbeMaxQRmsDps=0.25;
opts.UniversalRecoveryProbeMaxAirspeedErrorMps=1.0;
if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin), name=string(varargin{i}); if ~isfield(opts,name), error("Unknown option: %s",name); end, opts.(name)=varargin{i+1}; end
opts.ParallelWorkers=min(max(1,round(double(opts.ParallelWorkers))),3);
opts.AltitudeTrainingPointsM=unique([double(opts.AltitudeTrainingPointsM(:).') double(opts.AnchorAltitudeM)],"stable");
opts.AltitudeTrainingPointsM=opts.AltitudeTrainingPointsM(opts.AltitudeTrainingPointsM>=opts.AltitudeMinM & opts.AltitudeTrainingPointsM<=opts.AltitudeMaxM);
opts.AltitudeTrainingPointsM=sort(opts.AltitudeTrainingPointsM,'descend');
end
