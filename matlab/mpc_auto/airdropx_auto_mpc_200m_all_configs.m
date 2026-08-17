function result = airdropx_auto_mpc_200m_all_configs(varargin)
%AIRDROPX_AUTO_MPC_200M_ALL_CONFIGS Unified context-aware self-learning MPC.
%
% v29 changes the optimization architecture:
%   * cfg0..cfg4 no longer have different controller-learning strategies.
%   * Every unverified operating point uses the SAME unified learner.
%   * cfg/config number is only one context feature, not a strategy selector.
%   * The persistent LearningBank transfers verified controllers across
%     altitude, airspeed, mass/payload and later mission changes.
%   * A context-to-controller predictor uses inverse-distance transfer first
%     and automatically upgrades to GPR (fitrgp) once enough verified contexts
%     exist. The prediction becomes the next operating point's warm start.
%   * Full-certification Bayesian optimization then adapts ALL controller
%     parameters around the transferred seed using one formal-gate-aligned
%     objective. Existing evaluations for the SAME context are true prior
%     observations and are never repeated.
%   * Plant validity is handled separately and uniformly: every new/unverified
%     context runs an equilibrium probe; only a failed model/trim probe causes
%     trim -> clean ID -> Plant rebuild.
%   * Previously VERIFIED configurations still receive exactly one recheck.
%   * Existing v16-v28 Stage A/B/C/D/E files are kept as legacy evidence and
%     may supply seed controllers, but they no longer determine the strategy.
%
% The same entry point can therefore be used for future target altitude,
% target airspeed and payload changes. For a new mission use a new OutputRoot
% but keep the same LearningBankRoot so prior experience transfers.

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir);
addpath(paths.mpcDir);
addpath(paths.autoDir);
addpath(paths.sfuncDir);

fprintf("[V29] MATLAB tempdir = %s\n",tempdir);
tempRootForWarning = char(tempdir);
if ispc && numel(tempRootForWarning) >= 3 ...
        && (tempRootForWarning(1) == 'C' || tempRootForWarning(1) == 'c') ...
        && tempRootForWarning(2) == ':' ...
        && (tempRootForWarning(3) == '\' || tempRootForWarning(3) == '/')
    warning("AirdropX:UnifiedLearner:TempOnCDrive", ...
        "MATLAB tempdir is still on C:. With multi-worker Simulink this can create large .dmr files. Use run_v29_unified_D_temp.ps1 or set TEMP/TMP before MATLAB starts.");
end

outRoot = string(opts.OutputRoot);
if strlength(outRoot) == 0
    outRoot = string(fullfile(paths.matlabDir, "results", "mpc_auto_200m_all_cfg_v16"));
end
if ~isfolder(outRoot), mkdir(outRoot); end

checkpointFile = fullfile(outRoot, "airdropx_200m_cfg_checkpoint.mat");
masterFile = fullfile(outRoot, "identified_plants_200m_master.mat");
[master, masterSource] = local_load_master(opts.IdentifiedMat, masterFile);
checkpoint = local_load_checkpoint(checkpointFile);
checkpoint.master_source = string(masterSource);

% v29 persistent cross-mission learning bank. OutputRoot is mission/run
% state; LearningBankRoot is intentionally shared across missions.
learningBankRoot = local_resolve_learning_bank_root(paths,opts);
if logical(opts.UnifiedLearning) && ~isfolder(learningBankRoot)
    mkdir(learningBankRoot);
end

missionSignature = local_mission_signature(opts);
[checkpoint, missionChanged] = local_apply_mission_guard(checkpoint,missionSignature);
if missionChanged
    fprintf("[V29] Mission context changed -> old VERIFIED flags invalidated; old controller parameters remain transfer seeds.\n");
    local_save_checkpoint(checkpointFile,checkpoint);
end

% One deterministic hidden-trim calibration is enough for all configs because
% every old-MEX run resets at cfg0, 200 m, 50 m/s before prep drops.
if ~isfinite(checkpoint.hidden_elevator_trim) || logical(opts.RecalibrateHiddenTrimEachRun)
    checkpoint.hidden_elevator_trim = local_calibrate_hidden_trim(paths, master.trim_bank(1), opts, ...
        fullfile(outRoot, "hidden_trim_calibration"));
    checkpoint.updated_at = string(datetime("now"));
    local_save_checkpoint(checkpointFile, checkpoint);
else
    fprintf("[V21-200m] Reusing hidden trim %.6f from checkpoint.\n", checkpoint.hidden_elevator_trim);
end
hiddenTrim = double(checkpoint.hidden_elevator_trim);

if logical(opts.UnifiedLearning) && ~logical(opts.MissionRecoveryOnly)
    local_bootstrap_learning_bank(checkpoint,master,outRoot,learningBankRoot,opts);
end

parallelEnabled = false;
% v21: create the requested process pool before missing-Plant trim/ID work,
% not only when MPC Bayesopt starts. Missing-Plant trim/ID and controller
% learning share the same process pool (hard cap: three workers).
if logical(opts.UseParallel) && ~logical(opts.MissionRecoveryOnly)
    parallelEnabled = local_prepare_parallel_pool(paths, opts);
elseif logical(opts.MissionRecoveryOnly)
    fprintf("[V30.6.1-MISSION] MissionRecoveryOnly=1 -> no parallel controller/trim/ID workers are started.\n");
end

requestedCfgs = unique(round(double(opts.ConfigIds(:).')), "stable");
requestedCfgs = requestedCfgs(requestedCfgs >= 0 & requestedCfgs <= 4);
if isempty(requestedCfgs), error("ConfigIds must contain at least one cfg in 0..4."); end

cfgsToRun=requestedCfgs;
if logical(opts.MissionRecoveryOnly)
    if ~all(checkpoint.status(1:5)=="verified") || ~isfield(checkpoint,"best_candidate") || ...
            numel(checkpoint.best_candidate)<5 || any(cellfun(@isempty,checkpoint.best_candidate(1:5)))
        error("AirdropX:V30_6_1:MissionRecoveryNotReady", ...
            "MissionRecoveryOnly requires an inherited cfg0..cfg4 VERIFIED checkpoint with all best candidates.");
    end
    cfgsToRun=zeros(1,0);
    fprintf("[V30.6.1-MISSION] cfg0..cfg4 inherited VERIFIED -> single-cfg revalidation/BO/trim/ID SKIPPED.\n");
end

summaryRows = table();
for cfgId = cfgsToRun
    cfgRoot = fullfile(outRoot, sprintf("cfg%d", cfgId));
    if ~isfolder(cfgRoot), mkdir(cfgRoot); end
    fprintf("\n============================================================\n");
    fprintf("[V29] cfg%d context-aware learning\n", cfgId);
    fprintf("============================================================\n");

    % Strict sequence: never tune cfgN while a requested lower cfg is not verified.
    lower = requestedCfgs(requestedCfgs < cfgId);
    if any(checkpoint.status(lower + 1) ~= "verified")
        fprintf("[V29] cfg%d deferred because a lower requested configuration is not verified.\n", cfgId);
        break;
    end

    % On-demand plant training for cfg3/cfg4 or any missing bank entry.
    if local_plant_missing(master, cfgId)
        fprintf("[V21-200m] Plant%d is missing -> training only this missing config at 200 m.\n", cfgId);
        [master, plantInfo] = local_prepare_missing_plant(master, cfgId, paths, cfgRoot, opts, false);
        checkpoint.plant_ready(cfgId + 1) = true;
        checkpoint.plant_validation{cfgId + 1} = plantInfo.validation;
        checkpoint.updated_at = string(datetime("now"));
        local_save_master(masterFile, master);
        local_save_checkpoint(checkpointFile, checkpoint);
    else
        checkpoint.plant_ready(cfgId + 1) = true;
    end

    physicalNominals = local_physical_nominals(master, hiddenTrim);
    if ~isfinite(physicalNominals(cfgId + 1))
        error("AirdropX:AutoMPC200:PhysicalNominal", ...
            "Could not resolve physical elevator nominal for cfg%d.", cfgId);
    end
    writetable(local_nominal_table(master, physicalNominals, hiddenTrim), ...
        fullfile(outRoot, "physical_nominals_200m.csv"));

    % ------------------------------------------------------------------
    % Fast path: a previously verified cfg gets exactly ONE certification
    % run.  PASS -> skip Bayesopt.  FAIL -> stale + warm-start old best.
    % ------------------------------------------------------------------
    if logical(opts.ReuseVerifiedBest) && checkpoint.status(cfgId + 1) == "verified" && ...
            ~isempty(checkpoint.best_candidate{cfgId + 1})
        fprintf("[V21-200m] cfg%d previously VERIFIED -> one re-validation only.\n", cfgId);
        candidate = checkpoint.best_candidate{cfgId + 1};
        revalRoot = fullfile(cfgRoot, "revalidation");
        bankMat = fullfile(revalRoot, "revalidation_bank.mat");
        if ~isfolder(revalRoot), mkdir(revalRoot); end
        local_build_combined_bank(master, checkpoint, cfgId, candidate, ...
            physicalNominals, bankMat, opts);
        metrics = local_run_certification(paths, master, checkpoint, cfgId, candidate, ...
            physicalNominals, hiddenTrim, bankMat, revalRoot, "revalidate_once", ...
            opts.FinalWindowS, opts);
        writetable(metrics, fullfile(revalRoot, "revalidation_summary.csv"));
        checkpoint.last_metrics{cfgId + 1} = metrics;
        checkpoint.verification_count(cfgId + 1) = checkpoint.verification_count(cfgId + 1) + 1;
        checkpoint.updated_at = string(datetime("now"));
        if logical(metrics.formal_pass) && ~logical(metrics.hard_fail)
            fprintf("[V29] cfg%d re-validation PASS -> optimization SKIPPED.\n", cfgId);
            checkpoint.status(cfgId + 1) = "verified";
            local_save_checkpoint(checkpointFile, checkpoint);
            if logical(opts.UnifiedLearning)
                local_record_verified_controller(learningBankRoot,master,cfgId,candidate,metrics,outRoot,opts,"revalidation");
            end
            summaryRows = [summaryRows; local_summary_row(cfgId, "reused_verified", metrics, checkpoint)]; %#ok<AGROW>
            writetable(summaryRows, fullfile(outRoot, "all_config_status.csv"));
            continue;
        else
            fprintf("[V21-200m] cfg%d re-validation FAIL -> mark STALE and do not re-certify the same point.\n", cfgId);
            checkpoint.status(cfgId + 1) = "stale";
            failed = string(checkpoint.failed_certified_signatures{cfgId+1});
            failed(end+1,1) = string(local_candidate_signature(candidate));
            checkpoint.failed_certified_signatures{cfgId+1} = unique(failed,"stable");
            local_save_checkpoint(checkpointFile, checkpoint);
        end
    end

    % ------------------------------------------------------------------
    % v29 unified context-aware learning path.
    %
    % This SAME pipeline is used for every unverified cfg.  It first checks
    % whether the operating-point Plant/trim is valid, then transfers the
    % nearest learned controller from the persistent LearningBank and finally
    % runs one full-certification Bayesian optimizer over the complete
    % controller parameter vector.  There are no cfg-specific Stage A/B/C
    % learning rules in this path.
    % ------------------------------------------------------------------
    if logical(opts.UnifiedLearning)
        [master, checkpoint, physicalNominals, unifiedMetrics, unifiedPassed, unifiedStatus] = ...
            local_run_unified_pipeline(paths, master, checkpoint, cfgId, physicalNominals, ...
            hiddenTrim, cfgRoot, outRoot, learningBankRoot, masterFile, checkpointFile, opts);
        checkpoint.last_metrics{cfgId + 1} = unifiedMetrics;
        checkpoint.updated_at = string(datetime("now"));
        local_save_master(masterFile,master);
        local_save_checkpoint(checkpointFile,checkpoint);
        writetable(local_nominal_table(master,physicalNominals,hiddenTrim), ...
            fullfile(outRoot,"physical_nominals_200m.csv"));
        summaryRows = [summaryRows; local_summary_row(cfgId,unifiedStatus,unifiedMetrics,checkpoint)]; %#ok<AGROW>
        writetable(summaryRows,fullfile(outRoot,"all_config_status.csv"));
        if unifiedPassed
            fprintf("[V29] cfg%d VERIFIED by unified learner.\n",cfgId);
            continue;
        end
        fprintf("[V29] cfg%d unified learner stopped at status=%s.\n",cfgId,unifiedStatus);
        if logical(opts.StopOnConfigFailure), break; else, continue; end
    end

    % ------------------------------------------------------------------
    % v20 cfg2+ diagnosis + staged tuning.
    % Do not mix trim validity, inner MPC dynamics and height PI gains into
    % one large Bayesopt.  cfg0/cfg1 keep the proven v18 fast path above.
    % ------------------------------------------------------------------
    if cfgId >= round(double(opts.StagedTuningMinConfig))
        [master, checkpoint, physicalNominals, stageMetrics, stagePassed, stageStatus] = ...
            local_run_cfg2plus_pipeline(paths, master, checkpoint, cfgId, physicalNominals, ...
            hiddenTrim, cfgRoot, outRoot, masterFile, checkpointFile, opts);
        checkpoint.last_metrics{cfgId + 1} = stageMetrics;
        checkpoint.updated_at = string(datetime("now"));
        local_save_master(masterFile, master);
        local_save_checkpoint(checkpointFile, checkpoint);
        writetable(local_nominal_table(master, physicalNominals, hiddenTrim), ...
            fullfile(outRoot, "physical_nominals_200m.csv"));
        summaryRows = [summaryRows; local_summary_row(cfgId, stageStatus, stageMetrics, checkpoint)]; %#ok<AGROW>
        writetable(summaryRows, fullfile(outRoot, "all_config_status.csv"));
        if stagePassed
            fprintf("[V21-200m] cfg%d VERIFIED by staged pipeline.\n", cfgId);
            continue;
        end
        fprintf("[V21-200m] cfg%d staged pipeline stopped at status=%s.\n", cfgId, stageStatus);
        if logical(opts.StopOnConfigFailure), break; else, continue; end
    end

    % ------------------------------------------------------------------
    % Fast-resume search path for an unverified/stale cfg.
    % v20 never re-simulates points already present in this OutputRoot:
    % old v16 trajectories are rescored from candidate_metrics.csv and passed
    % to bayesopt as InitialX + InitialObjective (prior observations).
    % ------------------------------------------------------------------
    historyCsv = fullfile(cfgRoot, "optimization_history.csv");
    H = local_consolidate_history(cfgRoot, historyCsv, opts);
    nearPassSeedNeeded = local_should_seed_integral_probe(checkpoint, cfgId, H, opts);

    % Do not burn three more long certifications on old Ki=0 history when the
    % current controller is already a near-pass and the missing dimension has
    % never actually been evaluated.  In that case, probe Ki first.
    if ~nearPassSeedNeeded
        % Before spending new Bayesopt evaluations, long-certify a few distinct
        % historical points that looked best in the short search window.
        [checkpoint, rescueMetrics, rescuePassed] = local_try_history_rescue( ...
            paths, master, checkpoint, cfgId, H, physicalNominals, hiddenTrim, cfgRoot, opts);
        local_save_checkpoint(checkpointFile, checkpoint);
        if rescuePassed
            checkpoint.status(cfgId + 1) = "verified";
            checkpoint.last_metrics{cfgId + 1} = rescueMetrics;
            checkpoint.updated_at = string(datetime("now"));
            local_save_checkpoint(checkpointFile, checkpoint);
            writetable(rescueMetrics, fullfile(cfgRoot, "final_validation_summary.csv"));
            summaryRows = [summaryRows; local_summary_row(cfgId, "history_rescue_verified", rescueMetrics, checkpoint)]; %#ok<AGROW>
            writetable(summaryRows, fullfile(outRoot, "all_config_status.csv"));
            fprintf("[V21-200m] cfg%d VERIFIED from history rescue -> Bayesopt SKIPPED.\n", cfgId);
            continue;
        end
    end

    ctx = struct();
    ctx.paths = paths;
    ctx.master = master;
    ctx.checkpoint = checkpoint;
    ctx.cfgId = cfgId;
    ctx.cfgRoot = string(cfgRoot);
    ctx.physicalNominals = physicalNominals;
    ctx.hiddenTrim = hiddenTrim;
    ctx.opts = opts;

    % v18 near-pass rescue: v17 cfg0 ended with every gate passing except
    % |height drift|=1.028 m. All migrated history had Ki=0 because v17's
    % Bayesopt budget accidentally counted the prior points as the whole
    % MaxObjectiveEvaluations budget. Seed three small, real Ki probes once
    % before continuing Bayesian search.
    parallelEnabled = local_prepare_parallel_pool(paths, opts);
    if nearPassSeedNeeded
        fprintf("[V21-200m] cfg%d near-pass detected -> probing small nonzero Ki values first.\n", cfgId);
        local_run_integral_seed_probes(ctx, parallelEnabled, opts);
        H = local_consolidate_history(cfgRoot, historyCsv, opts);
        seedMask = abs(double(H.HeightIntegralGain)) > 1.0e-8;
        Hseed = H(seedMask,:);
        [checkpoint, seedMetrics, seedPassed] = local_certify_history_candidates( ...
            paths, master, checkpoint, cfgId, Hseed, physicalNominals, hiddenTrim, cfgRoot, ...
            "integral_seed_cert", opts.NearPassSeedCertificationTopK, opts);
        local_save_checkpoint(checkpointFile, checkpoint);
        if seedPassed
            checkpoint.status(cfgId + 1) = "verified";
            checkpoint.last_metrics{cfgId + 1} = seedMetrics;
            checkpoint.updated_at = string(datetime("now"));
            local_save_checkpoint(checkpointFile, checkpoint);
            writetable(seedMetrics, fullfile(cfgRoot, "final_validation_summary.csv"));
            summaryRows = [summaryRows; local_summary_row(cfgId, "integral_seed_verified", seedMetrics, checkpoint)]; %#ok<AGROW>
            writetable(summaryRows, fullfile(outRoot, "all_config_status.csv"));
            fprintf("[V21-200m] cfg%d VERIFIED by near-pass integral rescue -> Bayesopt SKIPPED.\n", cfgId);
            continue;
        end
    end

    vars = local_optimizable_variables();
    [initialX, initialObjective] = local_prior_observations(H, opts);
    addEval = local_cfg_value(opts.AdditionalObjectiveEvaluationsByConfig, cfgId, 15);
    priorCount = height(initialX);
    % MathWorks bayesopt counts InitialX/InitialObjective observations inside
    % MaxObjectiveEvaluations. Therefore total = prior observations + NEW calls.
    totalEvalLimit = priorCount + addEval;
    fprintf("[V21-200m] cfg%d Bayesopt starts: %d NEW evals, %d prior points, total limit=%d, parallel=%d workers=%d.\n", ...
        cfgId, addEval, priorCount, totalEvalLimit, parallelEnabled, ...
        local_parallel_worker_count(parallelEnabled, opts));
    objective = @(x) local_objective(x, ctx);

    args = { ...
        "MaxObjectiveEvaluations", totalEvalLimit, ...
        "IsObjectiveDeterministic", true, ...
        "UseParallel", logical(parallelEnabled), ...
        "AcquisitionFunctionName", "expected-improvement-plus", ...
        "Verbose", double(opts.BayesoptVerbose)};
    if logical(parallelEnabled)
        % Keep all requested workers occupied. With the requested process workers this
        % asks bayesopt to backfill whenever fewer than three are active.
        args = [args, {"MinWorkerUtilization", local_parallel_worker_count(true, opts)}]; %#ok<AGROW>
    end
    if ~isempty(initialX)
        args = [args, {"InitialX", initialX, "InitialObjective", initialObjective}]; %#ok<AGROW>
    end
    bo = bayesopt(objective, vars, args{:});
    save(fullfile(cfgRoot, "bayesopt_result_v21.mat"), "bo", "opts");

    % Workers write one evaluation_record.csv per unique evaluation directory;
    % consolidate on the client after Bayesopt to avoid parallel CSV races.
    H = local_consolidate_history(cfgRoot, historyCsv, opts);

    % Long certification, not short-window objective, decides graduation.
    [checkpoint, finalMetrics, finalPassed] = local_certify_top_history( ...
        paths, master, checkpoint, cfgId, H, physicalNominals, hiddenTrim, cfgRoot, opts);

    if isempty(finalMetrics)
        error("AirdropX:AutoMPC200:NoCertificationCandidate", ...
            "cfg%d produced no candidate that could be certified.", cfgId);
    end

    checkpoint.last_metrics{cfgId + 1} = finalMetrics;
    if finalPassed
        checkpoint.status(cfgId + 1) = "verified";
        fprintf("[V21-200m] cfg%d VERIFIED. Next run will only re-validate it once.\n", cfgId);
    else
        checkpoint.status(cfgId + 1) = "failed";
        fprintf("[V21-200m] cfg%d NOT VERIFIED. Next run adds only NEW points; old points will not be re-simulated.\n", cfgId);
    end
    checkpoint.updated_at = string(datetime("now"));
    local_save_checkpoint(checkpointFile, checkpoint);
    writetable(finalMetrics, fullfile(cfgRoot, "final_validation_summary.csv"));
    summaryRows = [summaryRows; local_summary_row(cfgId, checkpoint.status(cfgId + 1), finalMetrics, checkpoint)]; %#ok<AGROW>
    writetable(summaryRows, fullfile(outRoot, "all_config_status.csv"));

    if checkpoint.status(cfgId + 1) ~= "verified" && logical(opts.StopOnConfigFailure)
        break;
    end
end

allVerified = all(checkpoint.status(requestedCfgs + 1) == "verified");
allFiveVerified = all(checkpoint.status(1:5) == "verified");
checkpoint.all_verified = logical(allFiveVerified);
checkpoint.updated_at = string(datetime("now"));
local_save_checkpoint(checkpointFile, checkpoint);

% ----------------------------------------------------------------------
% Final mission acceptance test.
% This runs only after cfg0..cfg4 are all VERIFIED. It does not optimize or
% write LearningBank observations: it simply starts fully loaded, performs
% all four real drops, switches cfg0->cfg4, and judges the complete mission.
% ----------------------------------------------------------------------
finalMissionAttempted = false;
finalMissionPass = false;
finalMissionResult = struct();
if logical(opts.RunFinalMissionValidation) && allFiveVerified
    finalMissionAttempted = true;
    fprintf("\n[V29] cfg0..cfg4 all VERIFIED -> running FINAL MISSION VALIDATION.\n");
    try
        [moveScale,integralScale,transitionSeedSource] = local_checkpoint_transition_policy(checkpoint,learningBankRoot,opts);
        fprintf("[V30.6-MISSION] transition seed source=%s move=%.3f integral=%.3f.\n", ...
            transitionSeedSource,moveScale,integralScale);
        finalMissionResult = local_run_final_mission_with_policy(paths,outRoot,opts, ...
            moveScale,integralScale,"final_mission_validation");
        finalMissionPass = logical(finalMissionResult.mission_pass);

        % v30.6 mission-level near-pass is distinct from per-cfg tuning.  It
        % changes only the universal cfg-transition memory transfer, so all
        % already-VERIFIED single-cfg certifications remain valid.
        missionRefineNeeded = local_mission_nearpass_needed(finalMissionResult,opts);
        if ~finalMissionPass && local_mission_recovery_result_usable(finalMissionResult) && ...
                (logical(opts.MissionRecoveryOnly) || transitionSeedSource~="default_reset_baseline")
            % v30.6.2: recovery eligibility came from an authoritative earlier
            % near-pass baseline OR the current policy came from a transferred
            % verified transition seed.  Do not abort bounded refinement merely
            % because that first policy worsened the current-context gate.  The
            % candidate set always includes the reset baseline (move=0,int=0).
            missionRefineNeeded = true;
        end
        if ~finalMissionPass && missionRefineNeeded
            [refined,moveScale,integralScale,attempted] = ...
                local_universal_mission_nearpass_refinement(paths,outRoot,opts, ...
                    finalMissionResult,moveScale,integralScale);
            if attempted && local_mission_result_better(refined,finalMissionResult)
                finalMissionResult = refined;
                finalMissionPass = logical(refined.mission_pass);
            end
        end

        checkpoint.transition_move_transfer_scale = double(moveScale);
        checkpoint.transition_integral_transfer_scale = double(integralScale);
        if finalMissionPass
            checkpoint.transition_policy_source = "v30.6_mission_verified";
            local_record_verified_transition_policy(learningBankRoot,opts,moveScale,integralScale,finalMissionResult,outRoot);
        else
            checkpoint.transition_policy_source = "v30.6_mission_best_measured";
        end
        checkpoint.final_mission_attempted = true;
        checkpoint.final_mission_pass = finalMissionPass;
        checkpoint.final_mission_summary = finalMissionResult.summary;
        checkpoint.final_mission_updated_at = string(datetime("now"));
        local_save_checkpoint(checkpointFile,checkpoint);
    catch ME
        checkpoint.final_mission_attempted = true;
        checkpoint.final_mission_pass = false;
        checkpoint.final_mission_updated_at = string(datetime("now"));
        local_save_checkpoint(checkpointFile,checkpoint);
        warning("AirdropX:FinalMission:Failed", ...
            "Final mission validation did not complete: %s",ME.message);
    end
end

result = struct();
result.output_root = outRoot;
result.checkpoint_file = string(checkpointFile);
result.master_identified_file = string(masterFile);
result.status = checkpoint.status;
result.all_verified = logical(allFiveVerified);
result.requested_configs_verified = logical(allVerified);
result.summary = summaryRows;
result.learning_bank_root = string(learningBankRoot);
result.mission_signature = string(missionSignature);
result.final_mission_attempted = logical(finalMissionAttempted);
result.final_mission_pass = logical(finalMissionPass);
result.mission_complete = logical(allFiveVerified && finalMissionPass);
result.final_mission = finalMissionResult;
save(fullfile(outRoot, "v29_unified_learning_result.mat"), "result", "opts", "-v7.3");

fprintf("\n[V29] status: cfg0=%s cfg1=%s cfg2=%s cfg3=%s cfg4=%s\n", ...
    checkpoint.status(1), checkpoint.status(2), checkpoint.status(3), ...
    checkpoint.status(4), checkpoint.status(5));
if allFiveVerified
    if logical(opts.RunFinalMissionValidation)
        if finalMissionPass
            fprintf("[V29] FINAL SUCCESS: cfg0..cfg4 VERIFIED and complete four-drop MISSION_PASS=1.\n");
        else
            fprintf("[V29] cfg0..cfg4 are VERIFIED, but final four-drop mission validation has not passed yet.\n");
        end
    else
        fprintf("[V29] SUCCESS: cfg0..cfg4 are VERIFIED. Final mission validation is disabled.\n");
    end
elseif allVerified
    fprintf("[V29] All requested operating points are VERIFIED; cfg0..cfg4 are not all verified, so final mission validation was not run.\n");
else
    fprintf("[V29] Continue by rerunning the SAME command/OutputRoot; the LearningBank and same-context observations are reused.\n");
end
end



% ========================================================================
% v29 unified context-aware self-learning controller pipeline
% ========================================================================
function [master,checkpoint,physicalNominals,finalMetrics,passed,status] = ...
    local_run_unified_pipeline(paths,master,checkpoint,cfgId,physicalNominals, ...
    hiddenTrim,cfgRoot,outRoot,learningBankRoot,masterFile,checkpointFile,opts)

passed = false;
status = "unified_diagnosing";
finalMetrics = local_empty_cert_like_metrics(cfgId);

% Resume an interrupted Plant rebuild identically for every cfg.
if checkpoint.tuning_stage(cfgId+1) == "plant_rebuild_in_progress"
    fprintf("[V29] cfg%d resuming interrupted Plant rebuild.\n",cfgId);
    [master,plantInfo] = local_prepare_missing_plant(master,cfgId,paths,cfgRoot,opts,true);
    checkpoint.plant_ready(cfgId+1) = true;
    checkpoint.plant_validation{cfgId+1} = plantInfo.validation;
    checkpoint.plant_rebuild_count(cfgId+1) = checkpoint.plant_rebuild_count(cfgId+1) + 1;
    checkpoint.plant_generation(cfgId+1) = checkpoint.plant_generation(cfgId+1) + 1;
    checkpoint.tuning_stage(cfgId+1) = "equilibrium";
    checkpoint.equilibrium_probe_pass(cfgId+1) = false;
    checkpoint.equilibrium_probe_generation(cfgId+1) = 0;
    local_save_master(masterFile,master);
    local_save_checkpoint(checkpointFile,checkpoint);
    physicalNominals = local_physical_nominals(master,hiddenTrim);
end

% SAME Plant-validity rule for every operating point.  Controller learning is
% not allowed to compensate for a bad trim/model.
generation = checkpoint.plant_generation(cfgId+1);
needProbe = checkpoint.equilibrium_probe_generation(cfgId+1) ~= generation || ...
    ~logical(checkpoint.equilibrium_probe_pass(cfgId+1));

if needProbe
    probeRoot = fullfile(cfgRoot,"unified_equilibrium_probe");
    [P,probePass] = local_run_postdrop_equilibrium_probe(paths,master,checkpoint,cfgId, ...
        physicalNominals,hiddenTrim,probeRoot,opts);
    writetable(P,fullfile(probeRoot,"equilibrium_probe_summary.csv"));
    checkpoint.equilibrium_probe_pass(cfgId+1) = logical(probePass);
    checkpoint.equilibrium_probe_generation(cfgId+1) = generation;
    checkpoint.updated_at = string(datetime("now"));
    local_save_checkpoint(checkpointFile,checkpoint);

    if ~probePass
        fprintf("[V29] cfg%d operating-point probe FAIL (hSlope=%.4f, tailVz=%.4f). This is a Plant/trim problem, not a controller-search problem.\n", ...
            cfgId,double(P.equilibrium_height_slope_mps(1)),double(P.tail_vz_mps(1)));

        % v30.4 UNIVERSAL recovery.  This path is intentionally independent
        % of target altitude, airspeed and cfg number.  A near-pass operating
        % point gets the same sequence everywhere: longer observation ->
        % small local retrim around the nearest transferred/master seed ->
        % only then a full trim/ID/Plant rebuild.  No H-specific exceptions.
        [master,checkpoint,physicalNominals,P,probePass] = ...
            local_universal_equilibrium_recovery(paths,master,checkpoint,cfgId, ...
            physicalNominals,hiddenTrim,cfgRoot,masterFile,checkpointFile,P,opts);

        if probePass
            fprintf("[V30.4-RECOVERY] cfg%d universal equilibrium recovery PASS; full Plant rebuild avoided.\n",cfgId);
        else
            if checkpoint.plant_rebuild_count(cfgId+1) >= double(opts.MaxAutoPlantRebuildsPerConfig)
                checkpoint.status(cfgId+1) = "plant_probe_failed";
                status = "plant_probe_failed";
                finalMetrics = P;
                return;
            end

            backupRoot = local_archive_cfg_root(cfgRoot,outRoot,cfgId);
        fprintf("[V29] cfg%d previous mission-local cfg directory archived to %s\n",cfgId,backupRoot);
        checkpoint = local_reset_cfg_after_plant_rebuild(checkpoint,cfgId);
        checkpoint.tuning_stage(cfgId+1) = "plant_rebuild_in_progress";
        local_save_checkpoint(checkpointFile,checkpoint);
        if ~isfolder(cfgRoot), mkdir(cfgRoot); end

        [master,plantInfo] = local_prepare_missing_plant(master,cfgId,paths,cfgRoot,opts,true);
        checkpoint.plant_ready(cfgId+1) = true;
        checkpoint.plant_validation{cfgId+1} = plantInfo.validation;
        checkpoint.plant_rebuild_count(cfgId+1) = checkpoint.plant_rebuild_count(cfgId+1) + 1;
        checkpoint.plant_generation(cfgId+1) = checkpoint.plant_generation(cfgId+1) + 1;
        checkpoint.tuning_stage(cfgId+1) = "equilibrium";
        generation = checkpoint.plant_generation(cfgId+1);
        local_save_master(masterFile,master);
        local_save_checkpoint(checkpointFile,checkpoint);
        physicalNominals = local_physical_nominals(master,hiddenTrim);

        probeRoot2 = fullfile(cfgRoot,"unified_equilibrium_probe_after_rebuild");
        [P2,probePass2] = local_run_postdrop_equilibrium_probe(paths,master,checkpoint,cfgId, ...
            physicalNominals,hiddenTrim,probeRoot2,opts);
        writetable(P2,fullfile(probeRoot2,"equilibrium_probe_summary.csv"));
        checkpoint.equilibrium_probe_pass(cfgId+1) = logical(probePass2);
        checkpoint.equilibrium_probe_generation(cfgId+1) = generation;
        local_save_checkpoint(checkpointFile,checkpoint);
        if ~probePass2
            % v30.4 applies the SAME recovery after a freshly rebuilt Plant.
            % A near-pass rebuild is not discarded just because it occurred
            % on the second probe rather than the first one.
            [master,checkpoint,physicalNominals,P2,probePass2] = ...
                local_universal_equilibrium_recovery(paths,master,checkpoint,cfgId, ...
                physicalNominals,hiddenTrim,cfgRoot,masterFile,checkpointFile,P2,opts);
        end
        if ~probePass2
            checkpoint.status(cfgId+1) = "plant_probe_failed";
            status = "plant_probe_failed";
            finalMetrics = P2;
            return;
        end
        end % v30.4: full rebuild branch only when universal recovery did not pass
    end
else
    fprintf("[V29] cfg%d Plant generation %d equilibrium probe already PASS -> reuse.\n",cfgId,generation);
end

parallelEnabled = local_prepare_parallel_pool(paths,opts);
[bestC,finalMetrics,passed] = local_run_unified_learning( ...
    paths,master,checkpoint,cfgId,physicalNominals,hiddenTrim,cfgRoot,outRoot, ...
    learningBankRoot,parallelEnabled,opts);

checkpoint.tuning_stage(cfgId+1) = "unified_learning";
if passed
    checkpoint = local_commit_candidate(master,checkpoint,cfgId,bestC,finalMetrics,physicalNominals,cfgRoot,opts);
    checkpoint.status(cfgId+1) = "verified";
    checkpoint.tuning_stage(cfgId+1) = "verified";
    status = "verified";
    local_record_verified_controller(learningBankRoot,master,cfgId,bestC,finalMetrics,outRoot,opts,"unified");
else
    checkpoint = local_commit_candidate(master,checkpoint,cfgId,bestC,finalMetrics,physicalNominals,cfgRoot,opts);
    checkpoint.status(cfgId+1) = "unified_learning_failed";
    status = "unified_learning_failed";
end
end

function [bestC,bestM,passed] = local_run_unified_learning( ...
    paths,master,checkpoint,cfgId,physicalNominals,hiddenTrim,cfgRoot,outRoot, ...
    learningBankRoot,parallelEnabled,opts)

stageRoot = fullfile(cfgRoot,"unified_learning");
evalRoot = fullfile(stageRoot,"evaluations");
if ~isfolder(evalRoot), mkdir(evalRoot); end

context = local_operating_context(master,cfgId,opts);
contextSig = local_context_signature(context,opts);
fprintf("[V29] cfg%d context=%s\n",cfgId,contextSig);

% Import any completed local v29 evaluations into the persistent cross-mission
% bank before making the next decision.
H = local_load_unified_history(stageRoot,contextSig,opts);
local_sync_unified_evaluation_bank(learningBankRoot,H,context,outRoot,opts);

% Transfer from previously verified contexts.  This uses one identical
% mechanism for cfg0..cfg4 and for future altitude/airspeed/payload missions.
[seedCandidates,seedInfo] = local_collect_unified_seeds( ...
    learningBankRoot,checkpoint,cfgId,context,opts);
fprintf("[V29] cfg%d transfer predictor=%s verifiedBank=%d seeds=%d\n", ...
    cfgId,seedInfo.method,seedInfo.verified_count,numel(seedCandidates));

ctx = struct();
ctx.paths = paths;
ctx.master = master;
ctx.checkpoint = checkpoint;
ctx.cfgId = cfgId;
ctx.cfgRoot = string(stageRoot);
ctx.evalRoot = string(evalRoot);
ctx.physicalNominals = physicalNominals;
ctx.hiddenTrim = hiddenTrim;
ctx.context = context;
ctx.context_signature = contextSig;
ctx.opts = opts;

% Evaluate a small set of transferred controllers at the NEW context. Cross-
% context objective values are never reused as if they belonged to this
% context; only the controller parameters transfer.
doneSig = strings(0,1);
if ~isempty(H), doneSig = string(H.candidate_signature); end
todo = {};
maxTransfer = max(0,round(double(opts.UnifiedTransferSeedEvaluations)));
if maxTransfer > 0
    for i=1:numel(seedCandidates)
        sig = string(local_candidate_signature(seedCandidates{i}));
        if ~any(doneSig == sig)
            todo{end+1,1} = seedCandidates{i}; %#ok<AGROW>
        end
        if numel(todo) >= maxTransfer, break; end
    end
else
    fprintf("[V31.1-LEARN] cfg%d transfer seed evaluation disabled for this learning level.\n",cfgId);
end

if ~isempty(todo)
    fprintf("[V29] cfg%d evaluating %d transferred seeds at the current context.\n",cfgId,numel(todo));
    if logical(parallelEnabled) && numel(todo)>1
        parfor i=1:numel(todo)
            src=local_transfer_source_label(opts);
            local_unified_evaluate_candidate(todo{i},ctx,src);
        end
    else
        for i=1:numel(todo)
            src=local_transfer_source_label(opts);
            local_unified_evaluate_candidate(todo{i},ctx,src);
        end
    end
end

H = local_load_unified_history(stageRoot,contextSig,opts);
local_sync_unified_evaluation_bank(learningBankRoot,H,context,outRoot,opts);

if ~isempty(H)
    [bestC,bestM,passed] = local_select_unified_best(H,opts);
    if passed
        fprintf("[V29] cfg%d transferred/previous controller already FORMAL PASS -> Bayesian search skipped.\n",cfgId);
        local_write_unified_best(stageRoot,bestC,bestM,H,opts);
        return;
    end

    % v30.5: every H/V/cfg uses the SAME controller near-pass policy.
    % A context already close to the formal gate should not spend another
    % broad 10-D search first.  Refine around the best SAME-context measured
    % controller, preserving every completed certification as learning data.
    if local_controller_refinement_needed(bestM,opts)
        roundFile=local_controller_refinement_round_file(stageRoot,opts);
        roundsBefore=local_controller_nearpass_round_count(roundFile);
        [bestC,bestM,passed] = local_unified_controller_nearpass_refinement( ...
            bestC,bestM,H,ctx,parallelEnabled,opts);
        H = local_load_unified_history(stageRoot,contextSig,opts);
        local_sync_unified_evaluation_bank(learningBankRoot,H,context,outRoot,opts);
        if ~isempty(H)
            [bestC,bestM,passed] = local_select_unified_best(H,opts);
            local_write_unified_best(stageRoot,bestC,bestM,H,opts);
        end
        if passed
            fprintf("[V30.5-CTRL] cfg%d universal near-pass refinement reached FORMAL PASS -> broad BO skipped.\n",cfgId);
            return;
        end
        roundsAfter=local_controller_nearpass_round_count(roundFile);
        if roundsAfter>roundsBefore
            % Keep one invocation bounded. A measured near-pass gets at most
            % one local refinement round per mission attempt. The next resume
            % can use round 2; broad BO is only revisited after local rounds
            % are exhausted, avoiding 16+18+36 evaluations in one long run.
            fprintf("[V30.5-CTRL] cfg%d near-pass round completed without formal pass (gate=%.4f); preserving progress and returning before broad BO.\n", ...
                cfgId,local_formal_gate_ratio(bestM,opts));
            return;
        end
    end
else
    bestC = local_default_unified_candidate(opts);
    bestM = local_empty_cert_like_metrics(cfgId);
end

% Center the local search on the closest-to-graduation observation.  If no
% current-context observations exist, use the global transfer predictor.
center = bestC;
if isempty(H) && ~isempty(seedCandidates), center = seedCandidates{1}; end
[vars,bounds] = local_unified_variables(center,opts);
[X0,y0] = local_unified_prior(H,bounds,opts);

newEval = max(0,round(double(opts.UnifiedAdditionalEvaluationsPerRun)));
priorCount = height(X0);
% v31 may intentionally run a transfer-only or local-only learning level.
% Zero NEW broad evaluations is therefore a valid bounded decision, not an
% error and not a request to re-evaluate the existing priors.
if newEval <= 0
    fprintf("[V31-LEARN] cfg%d broad BO disabled for this learning level; preserving %d SAME-context observations.\n", ...
        cfgId,priorCount);
    if isempty(H)
        % Transfer/local-only level expected at least one real certification.
        % An empty readable history here means every attempted evaluation was
        % infrastructure-invalid; do not spend a learning level on it.
        error("AirdropX:V31:InfraNoValidEvaluation", ...
            "cfg%d produced no valid certification at this bounded learning level.",cfgId);
    end
    [bestC,bestM,passed] = local_select_unified_best(H,opts);
    local_write_unified_best(stageRoot,bestC,bestM,H,opts);
    return;
end
totalLimit = priorCount + newEval;
fprintf("[V29] cfg%d unified BO: %d SAME-context priors + %d NEW full certifications; workers=%d.\n", ...
    cfgId,priorCount,newEval,local_parallel_worker_count(parallelEnabled,opts));
fprintf("[V29] cfg%d search center: Np=%d Nc=%d Wh=%.3g Wvz=%.3g Wq=%.3g Rate=%.3g Auth=%.3g Kp=%.4g Ki=%.4g vzLim=%.3g\n", ...
    cfgId,center.Np,center.Nc,center.Wh,center.Wvz,center.Wq,center.RateScale, ...
    center.Authority,center.HeightToVzGain,center.HeightIntegralGain,center.HeightVzLimit);

objective = @(x)local_unified_objective(x,ctx);
args = { ...
    "MaxObjectiveEvaluations",totalLimit, ...
    "IsObjectiveDeterministic",true, ...
    "UseParallel",logical(parallelEnabled), ...
    "AcquisitionFunctionName","expected-improvement-plus", ...
    "Verbose",double(opts.BayesoptVerbose), ...
    "PlotFcn",[], ...
    "OutputFcn",@(results,state)local_unified_stop_on_formal(results,state,opts) ...
    };
if logical(parallelEnabled)
    args = [args,{"MinWorkerUtilization",min(local_parallel_worker_count(true,opts), ...
        round(double(opts.UnifiedMinWorkerUtilization)))}]; %#ok<AGROW>
end
if ~isempty(X0)
    args = [args,{"InitialX",X0,"InitialObjective",y0}]; %#ok<AGROW>
end

try
    bo = bayesopt(objective,vars,args{:});
    save(fullfile(stageRoot,"bayesopt_result_latest.mat"),"bo","opts","context","bounds","-v7.3");
catch ME
    % All worker objectives catch normal simulation failures.  If the parallel
    % scheduler itself fails, retain every completed evaluation so rerunning
    % the same command resumes instead of losing learning progress.
    fid=fopen(fullfile(stageRoot,"bayesopt_wrapper_error.txt"),"w");
    if fid>=0
        fprintf(fid,"%s\n",getReport(ME,"extended","hyperlinks","off"));
        fclose(fid);
    end
    warning("AirdropX:UnifiedLearner:BayesoptWrapper", ...
        "bayesopt wrapper stopped early; completed evaluations were preserved: %s",ME.message);
end

H = local_load_unified_history(stageRoot,contextSig,opts);
local_sync_unified_evaluation_bank(learningBankRoot,H,context,outRoot,opts);
if isempty(H)
    error("AirdropX:UnifiedLearner:NoEvaluation","cfg%d unified learner produced no readable evaluation.",cfgId);
end
[bestC,bestM,passed] = local_select_unified_best(H,opts);
local_write_unified_best(stageRoot,bestC,bestM,H,opts);

% v30.5 also catches a context that only entered the near-pass region during
% the broad search.  The same round counter prevents repeated unlimited polish.
if ~passed && local_controller_refinement_needed(bestM,opts)
    [bestC,bestM,passed] = local_unified_controller_nearpass_refinement( ...
        bestC,bestM,H,ctx,parallelEnabled,opts);
    H = local_load_unified_history(stageRoot,contextSig,opts);
    local_sync_unified_evaluation_bank(learningBankRoot,H,context,outRoot,opts);
    if ~isempty(H)
        [bestC,bestM,passed] = local_select_unified_best(H,opts);
        local_write_unified_best(stageRoot,bestC,bestM,H,opts);
    end
end

fprintf("[V29] cfg%d unified best gateRatio=%.4f formal=%d hRMS=%.4f hMax=%.4f Kp=%.5f Ki=%.6f\n", ...
    cfgId,local_formal_gate_ratio(bestM,opts),passed,double(bestM.steady_h_rms_m(1)), ...
    double(bestM.steady_h_max_abs_m(1)),bestC.HeightToVzGain,bestC.HeightIntegralGain);
end

function src=local_transfer_source_label(opts)
src="transfer_seed";
try
    if isfield(opts,'UnifiedV31ArchitectureRequal') && logical(opts.UnifiedV31ArchitectureRequal)
        src="v31_2_architecture_requal";
    end
catch
end
end

function tf = local_controller_refinement_needed(M,opts)
% v31.1: an explicit LOCAL learning level is authoritative.  A measured
% controller does not have to be inside the old <=1.30 near-pass band before
% trust-region refinement is allowed.  Legacy callers retain the old gate.
tf=false;
try
    if isempty(M) || height(M)<1 || logical(M.hard_fail(1)) || logical(M.formal_pass(1)), return; end
    ratio=local_formal_gate_ratio(M,opts);
    if ~isfinite(ratio) || ratio<=1.0, return; end
    if local_v31_force_local_refinement(opts)
        tf=true; return;
    end
catch
end
tf=local_controller_nearpass_needed(M,opts);
end

function tf = local_v31_force_local_refinement(opts)
tf=false;
try
    if isfield(opts,'UnifiedForceControllerLocalRefinement')
        tf=logical(opts.UnifiedForceControllerLocalRefinement);
    end
catch
    tf=false;
end
end

function tf = local_v31_layered_local_enabled(opts)
tf=false;
try
    if isfield(opts,'UnifiedV31LayeredLocalRefinement')
        tf=logical(opts.UnifiedV31LayeredLocalRefinement);
    end
catch
    tf=false;
end
end

function tf = local_controller_nearpass_needed(M,opts)
% v30.5 universal controller near-pass classifier. No altitude/cfg special cases.
tf=false;
try
    if ~logical(opts.UnifiedControllerNearPassEnabled), return; end
    if isempty(M) || height(M)<1 || logical(M.hard_fail(1)) || logical(M.formal_pass(1)), return; end
    ratio=local_formal_gate_ratio(M,opts);
    tf=isfinite(ratio) && ratio>1.0 && ratio<=double(opts.UnifiedControllerNearPassGateRatioMax);
catch
    tf=false;
end
end

function [bestC,bestM,passed] = local_unified_controller_nearpass_refinement( ...
    bestC,bestM,H,ctx,parallelEnabled,opts)
% v30.5 universal controller refinement for any H/V/cfg context.
% Round 1: deterministic one-/two-axis polish around measured same-context best.
% Round 2: narrow local Bayesian optimization, still using full certification.
stageRoot=char(ctx.cfgRoot);
if local_v31_height_governor_enabled(opts)
    refRoot=fullfile(stageRoot,"height_governor_refinement");
else
    refRoot=fullfile(stageRoot,"controller_nearpass_refinement");
end
if ~isfolder(refRoot), mkdir(refRoot); end
roundFile=fullfile(refRoot,"rounds.csv");
roundNo=local_controller_nearpass_round_count(roundFile)+1;
maxRounds=max(0,round(double(opts.UnifiedControllerNearPassMaxRoundsPerContext)));
if roundNo>maxRounds
    fprintf("[V30.5-CTRL] cfg%d near-pass gate=%.4f but %d/%d refinement rounds already used -> no repeated polish.\n", ...
        ctx.cfgId,local_formal_gate_ratio(bestM,opts),roundNo-1,maxRounds);
    passed=logical(bestM.formal_pass(1)) && ~logical(bestM.hard_fail(1));
    return;
end

startGate=local_formal_gate_ratio(bestM,opts);
if local_v31_height_governor_enabled(opts)
    fprintf("[V31.2-GOV] cfg%d measured best gate=%.4f -> single-channel height-governor round %d/%d.\n", ...
        ctx.cfgId,startGate,roundNo,maxRounds);
elseif local_v31_layered_local_enabled(opts)
    fprintf("[V31.1-LOCAL] cfg%d measured best gate=%.4f -> forced SAME-context trust-region round %d/%d.\n", ...
        ctx.cfgId,startGate,roundNo,maxRounds);
else
    fprintf("[V30.5-CTRL] cfg%d CONTROLLER_NEAR_PASS gate=%.4f -> universal refinement round %d/%d.\n", ...
        ctx.cfgId,startGate,roundNo,maxRounds);
end
fprintf("[V30.5-CTRL] cfg%d center Np=%d Nc=%d Wh=%.5g Wvz=%.5g Wq=%.5g Rate=%.5g Auth=%.5g Kp=%.7g Ki=%.8g vzLim=%.5g\n", ...
    ctx.cfgId,bestC.Np,bestC.Nc,bestC.Wh,bestC.Wvz,bestC.Wq,bestC.RateScale, ...
    bestC.Authority,bestC.HeightToVzGain,bestC.HeightIntegralGain,bestC.HeightVzLimit);

% Do not intentionally repeat a measured controller.
doneSig=strings(0,1);
if ~isempty(H) && ismember("candidate_signature",string(H.Properties.VariableNames))
    doneSig=string(H.candidate_signature);
end
if local_v31_height_governor_enabled(opts)
    cand=local_controller_v31_2_governor_deterministic_candidates(bestC,opts);
elseif local_v31_layered_local_enabled(opts)
    cand=local_controller_v31_deterministic_candidates(bestC,opts);
else
    cand=local_controller_nearpass_deterministic_candidates(bestC,opts);
end
todo={};
for i=1:numel(cand)
    c=local_clamp_unified_candidate(cand{i},opts);
    sig=string(local_candidate_signature(c));
    if any(doneSig==sig), continue; end
    todo{end+1,1}=c; %#ok<AGROW>
    if numel(todo)>=round(double(opts.UnifiedControllerNearPassDeterministicEvaluations)), break; end
end

nDet=numel(todo);
if nDet>0
    if local_v31_height_governor_enabled(opts)
        detSource="v31_2_governor_polish";
        fprintf("[V31.2-GOV] cfg%d governor deterministic polish: %d NEW full certifications; workers=%d.\n", ...
            ctx.cfgId,nDet,local_parallel_worker_count(parallelEnabled,opts));
    elseif local_v31_layered_local_enabled(opts)
        detSource="v31_local_polish";
        fprintf("[V31.1-LOCAL] cfg%d fixed-structure deterministic polish: %d NEW full certifications; workers=%d.\n", ...
            ctx.cfgId,nDet,local_parallel_worker_count(parallelEnabled,opts));
    else
        detSource="controller_nearpass_polish";
        fprintf("[V30.5-CTRL] cfg%d deterministic polish: %d NEW full certifications; workers=%d.\n", ...
            ctx.cfgId,nDet,local_parallel_worker_count(parallelEnabled,opts));
    end
    if logical(parallelEnabled) && nDet>1
        parfor i=1:nDet
            local_unified_safe_evaluate_candidate(todo{i},ctx,detSource);
        end
    else
        for i=1:nDet
            local_unified_safe_evaluate_candidate(todo{i},ctx,detSource);
        end
    end
end

H2=local_load_unified_history(stageRoot,string(ctx.context_signature),opts);
if ~isempty(H2)
    [bestC,bestM,passed]=local_select_unified_best(H2,opts);
else
    passed=false;
end
if passed
    local_append_controller_nearpass_round(roundFile,ctx.cfgId,roundNo,startGate, ...
        local_formal_gate_ratio(bestM,opts),nDet,0,true,"deterministic_pass");
    fprintf("[V30.5-CTRL] cfg%d deterministic polish FORMAL PASS gate=%.4f.\n", ...
        ctx.cfgId,local_formal_gate_ratio(bestM,opts));
    return;
end

% Only stay in the narrow local layer while the best measured result remains
% near-pass. If polish made it substantially worse, return to the normal learner.
if ~local_controller_refinement_needed(bestM,opts)
    local_append_controller_nearpass_round(roundFile,ctx.cfgId,roundNo,startGate, ...
        local_formal_gate_ratio(bestM,opts),nDet,0,false,"left_nearpass_region");
    return;
end

nNew=max(0,round(double(opts.UnifiedControllerNearPassBayesEvaluations)));
if nNew<=0
    local_append_controller_nearpass_round(roundFile,ctx.cfgId,roundNo,startGate, ...
        local_formal_gate_ratio(bestM,opts),nDet,0,false,"deterministic_only");
    return;
end
if local_v31_height_governor_enabled(opts)
    % v31.2 governor local level freezes the complete inner MPC. Only the
    % physical height governor parameters Kp/Ki/VzMax are refined; direct
    % altitude MPC weight is structurally zero and is not a search variable.
    [vars,bounds]=local_controller_v31_2_governor_variables(bestC,opts);
    X0=table(bestC.HeightToVzGain,bestC.HeightIntegralGain,bestC.HeightVzLimit, ...
        'VariableNames',{'HeightToVzGain','HeightIntegralGain','HeightVzLimit'});
    y0=local_unified_learning_objective(bestM,opts);
    priorCount=height(X0);
    objective=@(x)local_unified_objective_v31_2_governor(x,ctx,bestC);
    sourceLabel="v31_2_governor_bayes";
elseif local_v31_layered_local_enabled(opts)
    % v31.1 local level deliberately freezes controller structure/damping
    % terms and only learns the altitude-tracking subspace around the best
    % SAME-context measured controller.  This is a trust-region refinement,
    % not another transferred-controller or broad 10-D search.
    [vars,bounds]=local_controller_v31_local_variables(bestC,opts);
    X0=table(bestC.Wh,bestC.Wvz,bestC.HeightToVzGain,bestC.HeightIntegralGain,bestC.HeightVzLimit, ...
        'VariableNames',{'Wh','Wvz','HeightToVzGain','HeightIntegralGain','HeightVzLimit'});
    y0=local_unified_learning_objective(bestM,opts);
    priorCount=height(X0);
    objective=@(x)local_unified_objective_v31_local(x,ctx,bestC);
    sourceLabel="v31_local_bayes";
else
    [vars,bounds]=local_controller_nearpass_variables(bestC,opts);
    [X0,y0]=local_unified_prior(H2,bounds,opts);
    priorCount=height(X0);
    objective=@(x)local_unified_objective_with_source(x,ctx,"controller_nearpass_bayes");
    sourceLabel="controller_nearpass_bayes";
end
args={"MaxObjectiveEvaluations",priorCount+nNew, ...
    "IsObjectiveDeterministic",true, ...
    "UseParallel",logical(parallelEnabled), ...
    "AcquisitionFunctionName","expected-improvement-plus", ...
    "Verbose",double(opts.BayesoptVerbose),"PlotFcn",[], ...
    "OutputFcn",@(results,state)local_unified_stop_on_formal(results,state,opts)};
if logical(parallelEnabled)
    args=[args,{"MinWorkerUtilization",min(local_parallel_worker_count(true,opts), ...
        round(double(opts.UnifiedMinWorkerUtilization)))}]; %#ok<AGROW>
end
if ~isempty(X0), args=[args,{"InitialX",X0,"InitialObjective",y0}]; end %#ok<AGROW>

if local_v31_height_governor_enabled(opts)
    fprintf("[V31.2-GOV] cfg%d governor BO: %d measured-center prior + %d NEW 3-D certifications (%s).\n", ...
        ctx.cfgId,priorCount,nNew,char(sourceLabel));
else
    fprintf("[V31.1-CTRL] cfg%d local BO: %d measured-center prior + %d NEW trust-region certifications (%s).\n", ...
        ctx.cfgId,priorCount,nNew,char(sourceLabel));
end
try
    bo=bayesopt(objective,vars,args{:});
    save(fullfile(refRoot,sprintf("bayesopt_round_%02d.mat",roundNo)), ...
        "bo","bounds","opts","-v7.3");
catch ME
    fid=fopen(fullfile(refRoot,sprintf("bayesopt_round_%02d_error.txt",roundNo)),"w");
    if fid>=0, fprintf(fid,"%s\n",getReport(ME,"extended","hyperlinks","off")); fclose(fid); end
    warning("AirdropX:UnifiedLearner:ControllerNearPassBayes", ...
        "cfg%d near-pass local BO stopped early; completed evaluations are preserved: %s",ctx.cfgId,ME.message);
end

H3=local_load_unified_history(stageRoot,string(ctx.context_signature),opts);
if ~isempty(H3), [bestC,bestM,passed]=local_select_unified_best(H3,opts); end
endGate=local_formal_gate_ratio(bestM,opts);
local_append_controller_nearpass_round(roundFile,ctx.cfgId,roundNo,startGate,endGate,nDet,nNew,passed,"local_bayes_done");
fprintf("[V30.5-CTRL] cfg%d refinement round %d result gate=%.4f formal=%d.\n", ...
    ctx.cfgId,roundNo,endGate,passed);
end

function f=local_controller_refinement_round_file(stageRoot,opts)
if local_v31_height_governor_enabled(opts)
    f=fullfile(stageRoot,"height_governor_refinement","rounds.csv");
else
    f=fullfile(stageRoot,"controller_nearpass_refinement","rounds.csv");
end
end

function n=local_controller_nearpass_round_count(roundFile)
n=0;
if ~isfile(roundFile), return; end
try
    T=readtable(roundFile,'TextType','string');
    if ~isempty(T) && ismember("round",string(T.Properties.VariableNames))
        x=double(T.round); x=x(isfinite(x)); if ~isempty(x), n=max(x); end
    end
catch
end
end

function local_append_controller_nearpass_round(file,cfgId,roundNo,startGate,endGate,nDet,nBayes,passed,status)
row=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),double(cfgId),double(roundNo), ...
    double(startGate),double(endGate),double(nDet),double(nBayes),logical(passed),string(status), ...
    'VariableNames',{'timestamp','config_id','round','start_gate_ratio','end_gate_ratio', ...
    'deterministic_new_evaluations','local_bayes_budget','formal_pass','status'});
try
    if isfile(file)
        T=readtable(file,'TextType','string'); T=[T;row]; %#ok<AGROW>
    else
        T=row;
    end
    writetable(T,file);
catch
end
end

function tf=local_v31_height_governor_enabled(opts)
tf=false;
try
    tf=isfield(opts,'V31HeightGovernorEnabled') && logical(opts.V31HeightGovernorEnabled);
catch
    tf=false;
end
end

function cand=local_controller_v31_2_governor_deterministic_candidates(base,opts)
% v31.2: the inner MPC is frozen. Probe only the three height-governor
% parameters represented in the backward-compatible candidate schema.
base=local_clamp_unified_candidate(base,opts);
cand={};
for f=[0.80 1.20]
    c=base; c.HeightToVzGain=base.HeightToVzGain*f; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
end
ki=double(base.HeightIntegralGain);
if ki>1e-10
    for f=[0.50 1.50]
        c=base; c.HeightIntegralGain=ki*f; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
    end
else
    for v=[5e-4 2e-3]
        c=base; c.HeightIntegralGain=min(double(opts.UnifiedKiRange(2)),v); cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
    end
end
for f=[0.85 1.20]
    c=base; c.HeightVzLimit=base.HeightVzLimit*f; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
end
out={}; sigs=strings(0,1);
for i=1:numel(cand)
    sig=string(local_candidate_signature(cand{i}));
    if any(sigs==sig), continue; end
    sigs(end+1,1)=sig; out{end+1,1}=cand{i}; %#ok<AGROW>
end
cand=out;
end

function [vars,b]=local_controller_v31_2_governor_variables(center,opts)
center=local_clamp_unified_candidate(center,opts);
b.HeightToVzGain=local_mul_bounds(center.HeightToVzGain,opts.UnifiedKpRange,0.65,1.45);
kiLo=max(double(opts.UnifiedKiRange(1)),center.HeightIntegralGain*0.20);
kiHi=min(double(opts.UnifiedKiRange(2)),max(center.HeightIntegralGain*2.5,2e-3));
if kiHi<=kiLo, kiHi=min(double(opts.UnifiedKiRange(2)),kiLo+5e-4); end
b.HeightIntegralGain=[kiLo,kiHi];
b.HeightVzLimit=local_mul_bounds(center.HeightVzLimit,opts.UnifiedVzLimitRange,0.70,1.40);
vars=[ ...
    optimizableVariable("HeightToVzGain",b.HeightToVzGain,"Transform","log")
    optimizableVariable("HeightIntegralGain",b.HeightIntegralGain)
    optimizableVariable("HeightVzLimit",b.HeightVzLimit)
    ];
end

function score=local_unified_objective_v31_2_governor(x,ctx,base)
c=base;
c.HeightToVzGain=double(x.HeightToVzGain);
c.HeightIntegralGain=double(x.HeightIntegralGain);
c.HeightVzLimit=double(x.HeightVzLimit);
c=local_clamp_unified_candidate(c,ctx.opts);
score=local_unified_evaluate_candidate(c,ctx,"v31_2_governor_bayes");
end

function cand=local_controller_v31_deterministic_candidates(base,opts)
% v31.1 fixed-structure local probes.  Keep Np/Nc/Wq/Rate/Authority fixed
% and spend the small deterministic budget on the altitude-tracking terms.
base=local_clamp_unified_candidate(base,opts);
cand={};
ki=double(base.HeightIntegralGain);
if ki>1e-10
    c=base; c.HeightIntegralGain=ki*0.50; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
    c=base; c.HeightIntegralGain=ki*1.75; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
else
    c=base; c.HeightIntegralGain=min(double(opts.UnifiedKiRange(2)),max(2e-4,double(opts.UnifiedControllerNearPassKiMinUpperSpan)/4)); cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
    c=base; c.HeightIntegralGain=min(double(opts.UnifiedKiRange(2)),max(5e-4,double(opts.UnifiedControllerNearPassKiMinUpperSpan)/2)); cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
end
c=base; c.HeightToVzGain=base.HeightToVzGain*0.85; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
c=base; c.HeightToVzGain=base.HeightToVzGain*1.15; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
c=base; c.Wh=base.Wh*1.15; c.Wvz=base.Wvz*0.95; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
c=base; c.HeightVzLimit=base.HeightVzLimit*1.15; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
out={}; sigs=strings(0,1);
for i=1:numel(cand)
    sig=string(local_candidate_signature(cand{i}));
    if any(sigs==sig), continue; end
    sigs(end+1,1)=sig; out{end+1,1}=cand{i}; %#ok<AGROW>
end
cand=out;
end

function [vars,b]=local_controller_v31_local_variables(center,opts)
% Five-dimensional altitude-tracking trust region.  Controller horizon,
% Wq, rate scaling and authority remain exactly the measured center values.
center=local_clamp_unified_candidate(center,opts);
b.Wh=local_mul_bounds(center.Wh,opts.UnifiedWhRange,0.80,1.25);
b.Wvz=local_mul_bounds(center.Wvz,opts.UnifiedWvzRange,0.80,1.25);
b.HeightToVzGain=local_mul_bounds(center.HeightToVzGain,opts.UnifiedKpRange,0.70,1.35);
kiLo=max(double(opts.UnifiedKiRange(1)),center.HeightIntegralGain*0.25);
kiHi=min(double(opts.UnifiedKiRange(2)),max(center.HeightIntegralGain*3.0,double(opts.UnifiedControllerNearPassKiMinUpperSpan)));
if kiHi<=kiLo, kiHi=min(double(opts.UnifiedKiRange(2)),kiLo+max(1e-5,double(opts.UnifiedControllerNearPassKiMinUpperSpan)/4)); end
b.HeightIntegralGain=[kiLo,kiHi];
b.HeightVzLimit=local_mul_bounds(center.HeightVzLimit,opts.UnifiedVzLimitRange,0.80,1.25);
vars=[ ...
    optimizableVariable("Wh",b.Wh,"Transform","log")
    optimizableVariable("Wvz",b.Wvz,"Transform","log")
    optimizableVariable("HeightToVzGain",b.HeightToVzGain,"Transform","log")
    optimizableVariable("HeightIntegralGain",b.HeightIntegralGain)
    optimizableVariable("HeightVzLimit",b.HeightVzLimit)
    ];
end

function score=local_unified_objective_v31_local(x,ctx,base)
c=base;
c.Wh=double(x.Wh);
c.Wvz=double(x.Wvz);
c.HeightToVzGain=double(x.HeightToVzGain);
c.HeightIntegralGain=double(x.HeightIntegralGain);
c.HeightVzLimit=double(x.HeightVzLimit);
c=local_clamp_unified_candidate(c,ctx.opts);
score=local_unified_evaluate_candidate(c,ctx,"v31_local_bayes");
end

function cand=local_controller_nearpass_deterministic_candidates(base,opts)
% Ordered universal perturbations. These are deliberately not a Cartesian
% product: they cheaply probe the dimensions most likely to remove a small
% formal-gate violation without throwing away a good controller.
base=local_clamp_unified_candidate(base,opts);
cand={};
% Integral action first; use multiplicative moves so tiny but nonzero Ki is
% explored on a meaningful scale. Include zero as a bias-protection check.
for f=[0.5 2.0 4.0 0.25]
    c=base; c.HeightIntegralGain=base.HeightIntegralGain*f; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
end
if base.HeightIntegralGain>0
    c=base; c.HeightIntegralGain=0; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
end
for f=[0.85 1.15 0.70 1.30]
    c=base; c.HeightToVzGain=base.HeightToVzGain*f; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
end
for f=[0.85 1.20]
    c=base; c.Wh=base.Wh*f; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
end
for f=[0.90 1.10]
    c=base; c.Wvz=base.Wvz*f; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
end
for f=[0.85 1.15]
    c=base; c.HeightVzLimit=base.HeightVzLimit*f; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
end
c=base; c.HeightToVzGain=base.HeightToVzGain*1.15; c.HeightIntegralGain=base.HeightIntegralGain*2; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>
c=base; c.Wh=base.Wh*1.15; c.HeightIntegralGain=base.HeightIntegralGain*2; cand{end+1,1}=local_clamp_unified_candidate(c,opts); %#ok<AGROW>

% Deduplicate after clamping (important at global parameter bounds).
out={}; sigs=strings(0,1);
for i=1:numel(cand)
    sig=string(local_candidate_signature(cand{i}));
    if any(sigs==sig), continue; end
    sigs(end+1,1)=sig; out{end+1,1}=cand{i}; %#ok<AGROW>
end
cand=out;
end

function [vars,b]=local_controller_nearpass_variables(center,opts)
center=local_clamp_unified_candidate(center,opts);
b.Np=[max(opts.UnifiedNpRange(1),center.Np-1),min(opts.UnifiedNpRange(2),center.Np+1)];
b.Nc=[max(opts.UnifiedNcRange(1),center.Nc-1),min(opts.UnifiedNcRange(2),center.Nc+1)];
b.Wh=local_mul_bounds(center.Wh,opts.UnifiedWhRange,0.70,1.45);
b.Wvz=local_mul_bounds(center.Wvz,opts.UnifiedWvzRange,0.75,1.35);
b.Wq=local_mul_bounds(center.Wq,opts.UnifiedWqRange,0.80,1.25);
b.RateScale=local_mul_bounds(center.RateScale,opts.UnifiedRateScaleRange,0.75,1.20);
b.Authority=[max(opts.UnifiedAuthorityRange(1),center.Authority-0.10), ...
             min(opts.UnifiedAuthorityRange(2),center.Authority+0.10)];
b.HeightToVzGain=local_mul_bounds(center.HeightToVzGain,opts.UnifiedKpRange,0.55,1.70);
kiHi=min(double(opts.UnifiedKiRange(2)),max(center.HeightIntegralGain*6,double(opts.UnifiedControllerNearPassKiMinUpperSpan)));
b.HeightIntegralGain=[max(double(opts.UnifiedKiRange(1)),0),kiHi];
b.HeightVzLimit=local_mul_bounds(center.HeightVzLimit,opts.UnifiedVzLimitRange,0.70,1.35);
fields=fieldnames(b);
for i=1:numel(fields)
    f=fields{i}; rr=double(b.(f));
    if rr(2)<=rr(1), mid=mean(rr); rr=[mid-1e-6 mid+1e-6]; b.(f)=rr; end
end
vars=[ ...
    optimizableVariable("Np",b.Np,"Type","integer")
    optimizableVariable("Nc",b.Nc,"Type","integer")
    optimizableVariable("Wh",b.Wh,"Transform","log")
    optimizableVariable("Wvz",b.Wvz,"Transform","log")
    optimizableVariable("Wq",b.Wq,"Transform","log")
    optimizableVariable("RateScale",b.RateScale,"Transform","log")
    optimizableVariable("Authority",b.Authority)
    optimizableVariable("HeightToVzGain",b.HeightToVzGain,"Transform","log")
    optimizableVariable("HeightIntegralGain",b.HeightIntegralGain)
    optimizableVariable("HeightVzLimit",b.HeightVzLimit)
    ];
end

function score=local_unified_safe_evaluate_candidate(c,ctx,source)
% Keep one infrastructure-invalid candidate from aborting an entire
% deterministic polish batch. The evaluator already records the invalid row
% and excludes it from learning; this wrapper only contains the exception.
try
    score=local_unified_evaluate_candidate(c,ctx,source);
catch ME
    if string(ME.identifier)=="AirdropX:UnifiedLearner:InfrastructureFailure"
        score=double(ctx.opts.UnifiedHardFailLearningLoss);
        return;
    end
    rethrow(ME);
end
end

function score=local_unified_objective_with_source(x,ctx,source)
c=local_candidate_from_x(x,ctx.opts);
score=local_unified_evaluate_candidate(c,ctx,source);
end

function score = local_unified_objective(x,ctx)
c = local_candidate_from_x(x,ctx.opts);
score = local_unified_evaluate_candidate(c,ctx,"bayesopt");
end

function score = local_unified_evaluate_candidate(c,ctx,source)
local_prepare_worker_filegen(ctx);
c = local_upgrade_candidate(c);
[~,token] = fileparts(tempname);
tag = "eval_" + string(datetime("now","Format","yyyyMMdd_HHmmss_SSS")) + "_" + string(token);
evalDir = fullfile(ctx.evalRoot,tag);
if ~isfolder(evalDir), mkdir(evalDir); end
bankMat = fullfile(evalDir,"combined_mpc_bank.mat");
M = table();
metricsFile = fullfile(evalDir,"certification_summary.csv");
errorFile = fullfile(evalDir,"error.txt");
infrastructureFail = false;

try
    local_build_combined_bank(ctx.master,ctx.checkpoint,ctx.cfgId,c, ...
        ctx.physicalNominals,bankMat,ctx.opts);
    M = local_run_certification(ctx.paths,ctx.master,ctx.checkpoint,ctx.cfgId,c, ...
        ctx.physicalNominals,ctx.hiddenTrim,bankMat,evalDir,"unified_cert", ...
        ctx.opts.FinalWindowS,ctx.opts);
    writetable(M,metricsFile);
    score = local_unified_learning_objective(M,ctx.opts);
catch ME
    % v30.2: an exception is not a measured controller-quality result.
    % Genuine controller hard-fails must come back as a readable
    % certification_summary.csv with hard_fail=1. Any thrown exception
    % (path/codegen/logging/programming/etc.) is evaluation-invalid and must
    % never train BO/GPR or consume a persistent learning observation.
    infrastructureFail = true;
    score = double(ctx.opts.UnifiedHardFailLearningLoss);
    fid=fopen(errorFile,"w");
    if fid>=0
        fprintf(fid,"infrastructure_exception=1\nidentifier=%s\n",char(string(ME.identifier)));
        fprintf(fid,"%s\n",getReport(ME,"extended","hyperlinks","off"));
        fclose(fid);
    end
end

gate = inf; formal=false; hard=true; observedMass=NaN; observedCg=NaN;
if ~isempty(M)
    try
        gate = local_formal_gate_ratio(M,ctx.opts);
        formal = logical(M.formal_pass(1)) && ~logical(M.hard_fail(1));
        hard = logical(M.hard_fail(1));
        if ismember("context_mass_kg",string(M.Properties.VariableNames))
            observedMass=double(M.context_mass_kg(1));
        end
        if ismember("context_cg_x_m",string(M.Properties.VariableNames))
            observedCg=double(M.context_cg_x_m(1));
        end
    catch
    end
end
sig = string(local_candidate_signature(c));
row = table(string(tag),string(datetime("now","Format","yyyyMMdd_HHmmss_SSS")),string(ctx.context_signature),sig, ...
    ctx.cfgId,c.Np,c.Nc,c.Wh,c.Wvz,c.Wq,c.RateScale,c.Authority, ...
    c.HeightToVzGain,c.HeightIntegralGain,c.HeightVzLimit,double(score),double(gate), ...
    logical(hard),logical(formal),logical(infrastructureFail),double(observedMass),double(observedCg),string(source),string(metricsFile), ...
    'VariableNames',{'eval_tag','timestamp','context_signature','candidate_signature','config_id', ...
    'Np','Nc','Wh','Wvz','Wq','RateScale','Authority','HeightToVzGain', ...
    'HeightIntegralGain','HeightVzLimit','objective','gate_ratio','hard_fail', ...
    'formal_pass','infrastructure_fail','observed_mass_kg','observed_cg_x_m','source','metrics_file'});
try, writetable(row,fullfile(evalDir,"unified_record.csv")); catch, end

if infrastructureFail
    fprintf("[V30.2-INFRA] cfg%d invalid evaluation %s; excluded from learning/resume priors.\n", ...
        ctx.cfgId,char(tag));
    error("AirdropX:UnifiedLearner:InfrastructureFailure", ...
        "cfg%d evaluation %s ended by infrastructure/runtime exception; see %s", ...
        ctx.cfgId,char(tag),char(errorFile));
end

fprintf("[V29] cfg%d LEARN obj=%.4f gate=%.4f formal=%d Np=%d Nc=%d Wh=%.3g Wvz=%.3g Wq=%.3g Rate=%.3g Auth=%.3f Kp=%.5f Ki=%.6f vzLim=%.3f\n", ...
    ctx.cfgId,double(score),double(gate),formal,c.Np,c.Nc,c.Wh,c.Wvz,c.Wq, ...
    c.RateScale,c.Authority,c.HeightToVzGain,c.HeightIntegralGain,c.HeightVzLimit);
end

function tf=local_unified_is_infrastructure_exception(ME)
tf=false;
try
    id=lower(string(ME.identifier));
    msg=lower(string(ME.message));
    markers=["missinglog","missinglogsout","dmr","database is full", ...
        "fetchnextfutureerrored","remotecause","no space left","disk full", ...
        "path too long","filename or extension is too long", ...
        "file name or extension is too long","specified path, file name, or both are too long", ...
        "260 character","max_path","infrastructurefailure"];
    for i=1:numel(markers)
        if contains(id,markers(i)) || contains(msg,markers(i))
            tf=true;
            return;
        end
    end
catch
end
end

function score = local_unified_learning_objective(M,opts)
if isempty(M) || height(M)<1
    score = double(opts.UnifiedHardFailLearningLoss);
    return;
end
try
    if logical(M.hard_fail(1))
        score = double(opts.UnifiedHardFailLearningLoss);
        return;
    end
    r = local_formal_ratio_vector(M,opts);
    r = r(isfinite(r));
    if isempty(r)
        score = double(opts.UnifiedHardFailLearningLoss);
        return;
    end
    worst = max(r);
    violation = max(0,r-1);
    score = worst + double(opts.UnifiedMultiViolationWeight)*mean(violation.^2);
    if logical(M.formal_pass(1))
        score = min(score,double(opts.UnifiedFormalStopObjective));
    end
    if ~isfinite(score), score=double(opts.UnifiedHardFailLearningLoss); end
catch
    score = double(opts.UnifiedHardFailLearningLoss);
end
end

function stop = local_unified_stop_on_formal(results,state,opts)
stop = false;
if string(state) ~= "iteration", return; end
try
    v = double(results.MinObjective);
    if isfinite(v) && v <= double(opts.UnifiedFormalStopObjective)
        fprintf("[V29] unified BO observed formal-region objective %.5f -> requesting early stop.\n",v);
        stop = true;
    end
catch
end
end

function H = local_load_unified_history(stageRoot,contextSig,opts)
H = table();
evalDirs = dir(fullfile(stageRoot,"evaluations","eval_*"));
evalDirs = evalDirs([evalDirs.isdir]);
for i=1:numel(evalDirs)
    recordFile = fullfile(evalDirs(i).folder,evalDirs(i).name,"unified_record.csv");
    if ~isfile(recordFile), continue; end
    try
        R=readtable(recordFile,'VariableNamingRule','preserve','TextType','string');
        if isempty(R) || ~ismember("context_signature",string(R.Properties.VariableNames)), continue; end
        if string(R.context_signature(1)) ~= string(contextSig), continue; end

        % v30.2: only a readable certification summary is a controller
        % observation. Legacy path/codegen failures could leave a
        % unified_record.csv but no certification_summary.csv; those rows
        % must not become priors and must never be selected as best.
        evalDir = fullfile(evalDirs(i).folder,evalDirs(i).name);
        mf = fullfile(evalDir,"certification_summary.csv");
        if ismember("metrics_file",string(R.Properties.VariableNames))
            candidateMf = string(R.metrics_file(1));
            if strlength(candidateMf)>0 && isfile(candidateMf), mf=char(candidateMf); end
        end
        infraFlag=false;
        if ismember("infrastructure_fail",string(R.Properties.VariableNames))
            try, infraFlag=logical(R.infrastructure_fail(1)); catch, infraFlag=false; end
        end
        if infraFlag || ~isfile(mf) || ~local_unified_summary_readable(mf)
            continue;
        end
        R.metrics_file(:)=string(mf);
        if isempty(H), H=R; else, H=[H;R]; end %#ok<AGROW>
    catch
    end
end
if isempty(H), return; end

% Normalize types.
svars=["eval_tag","timestamp","context_signature","candidate_signature","source","metrics_file"];
for v=svars
    if ismember(v,string(H.Properties.VariableNames)), H.(char(v))=string(H.(char(v))); end
end
nvars=["config_id","Np","Nc","Wh","Wvz","Wq","RateScale","Authority", ...
    "HeightToVzGain","HeightIntegralGain","HeightVzLimit","objective","gate_ratio", ...
    "observed_mass_kg","observed_cg_x_m"];
for v=nvars
    if ismember(v,string(H.Properties.VariableNames)), H.(char(v))=double(H.(char(v))); end
end
H.hard_fail=logical(H.hard_fail); H.formal_pass=logical(H.formal_pass);
if ismember("infrastructure_fail",string(H.Properties.VariableNames))
    H.infrastructure_fail=logical(H.infrastructure_fail);
else
    H.infrastructure_fail=false(height(H),1);
end

% Infrastructure failures are not controller-quality observations and must
% never enter the self-learning prior/global bank.
valid=isfinite(H.objective) & ~H.infrastructure_fail;
H=H(valid,:);
if isempty(H), return; end

% Deduplicate same context + same controller. Keep formal pass first, then
% smallest formal-gate distance/objective.
H=sortrows(H,{'formal_pass','gate_ratio','objective'},{'descend','ascend','ascend'});
[~,ia]=unique(string(H.candidate_signature),"stable");
H=H(ia,:);
H=sortrows(H,{'formal_pass','gate_ratio','objective'},{'descend','ascend','ascend'});

try, writetable(H,fullfile(stageRoot,"unified_history.csv")); catch, end
end

function tf = local_unified_summary_readable(file)
tf=false;
try
    if ~isfile(file), return; end
    T=readtable(file,'VariableNamingRule','preserve');
    vars=string(T.Properties.VariableNames);
    tf=~isempty(T) && ismember("formal_pass",vars) && ismember("hard_fail",vars);
catch
    tf=false;
end
end

function [bestC,bestM,passed] = local_select_unified_best(H,opts)
if isempty(H), error("AirdropX:UnifiedLearner:NoReadableEvaluation","Unified history is empty."); end
H=sortrows(H,{'formal_pass','gate_ratio','objective'},{'descend','ascend','ascend'});
for pick=1:height(H)
    mf=string(H.metrics_file(pick));
    if ~local_unified_summary_readable(mf), continue; end
    bestC=local_candidate_from_history_row(H(pick,:),opts);
    bestM=readtable(mf,'VariableNamingRule','preserve');
    passed=logical(bestM.formal_pass(1)) && ~logical(bestM.hard_fail(1));
    return;
end
error("AirdropX:UnifiedLearner:NoReadableEvaluation", ...
    "Unified history contains no readable certification_summary.csv rows.");
end

function local_write_unified_best(stageRoot,bestC,bestM,H,opts)
try
    writetable(H,fullfile(stageRoot,"unified_history.csv"));
    writetable(local_candidate_table(double(bestM.config_id(1)),bestC, ...
        local_unified_learning_objective(bestM,opts)),fullfile(stageRoot,"best_candidate.csv"));
    writetable(bestM,fullfile(stageRoot,"best_validation_summary.csv"));
catch
end
end

function [X,y] = local_unified_prior(H,bounds,opts)
X=table(); y=[];
if isempty(H), return; end
keep=false(height(H),1);
for i=1:height(H)
    c=local_candidate_from_history_row(H(i,:),opts);
    keep(i)=local_candidate_in_bounds(c,bounds);
end
H=H(keep,:);
if isempty(H), return; end
H=sortrows(H,{'formal_pass','gate_ratio','objective'},{'descend','ascend','ascend'});
n=min(height(H),round(double(opts.UnifiedMaxSameContextPriors)));
H=H(1:n,:);
X=H(:,{'Np','Nc','Wh','Wvz','Wq','RateScale','Authority', ...
    'HeightToVzGain','HeightIntegralGain','HeightVzLimit'});
y=double(H.objective);
end

function [vars,b] = local_unified_variables(center,opts)
center=local_clamp_unified_candidate(center,opts);

b.Np=[max(opts.UnifiedNpRange(1),center.Np-opts.UnifiedNpHalfSpan), ...
      min(opts.UnifiedNpRange(2),center.Np+opts.UnifiedNpHalfSpan)];
b.Nc=[max(opts.UnifiedNcRange(1),center.Nc-opts.UnifiedNcHalfSpan), ...
      min(opts.UnifiedNcRange(2),center.Nc+opts.UnifiedNcHalfSpan)];
b.Wh=local_mul_bounds(center.Wh,opts.UnifiedWhRange,opts.UnifiedWeightLowerFactor,opts.UnifiedWeightUpperFactor);
b.Wvz=local_mul_bounds(center.Wvz,opts.UnifiedWvzRange,opts.UnifiedWeightLowerFactor,opts.UnifiedWeightUpperFactor);
b.Wq=local_mul_bounds(center.Wq,opts.UnifiedWqRange,opts.UnifiedWeightLowerFactor,opts.UnifiedWeightUpperFactor);
b.RateScale=local_mul_bounds(center.RateScale,opts.UnifiedRateScaleRange,opts.UnifiedRateLowerFactor,opts.UnifiedRateUpperFactor);
b.Authority=[max(opts.UnifiedAuthorityRange(1),center.Authority-opts.UnifiedAuthorityHalfSpan), ...
             min(opts.UnifiedAuthorityRange(2),center.Authority+opts.UnifiedAuthorityHalfSpan)];
b.HeightToVzGain=local_mul_bounds(center.HeightToVzGain,opts.UnifiedKpRange,opts.UnifiedKpLowerFactor,opts.UnifiedKpUpperFactor);
kiSpan=max(double(opts.UnifiedMinKiSpan),max(center.HeightIntegralGain,0.001)*double(opts.UnifiedKiSpanFactor));
b.HeightIntegralGain=[max(opts.UnifiedKiRange(1),center.HeightIntegralGain-kiSpan), ...
                      min(opts.UnifiedKiRange(2),center.HeightIntegralGain+kiSpan)];
b.HeightVzLimit=local_mul_bounds(center.HeightVzLimit,opts.UnifiedVzLimitRange,opts.UnifiedVzLowerFactor,opts.UnifiedVzUpperFactor);

% Avoid zero-width ranges.
fields=fieldnames(b);
for i=1:numel(fields)
    f=fields{i};
    rr=double(b.(f));
    if rr(2)<=rr(1)
        mid=mean(rr); rr=[mid-1e-6 mid+1e-6]; b.(f)=rr;
    end
end

vars=[ ...
    optimizableVariable("Np",b.Np,"Type","integer")
    optimizableVariable("Nc",b.Nc,"Type","integer")
    optimizableVariable("Wh",b.Wh,"Transform","log")
    optimizableVariable("Wvz",b.Wvz,"Transform","log")
    optimizableVariable("Wq",b.Wq,"Transform","log")
    optimizableVariable("RateScale",b.RateScale,"Transform","log")
    optimizableVariable("Authority",b.Authority)
    optimizableVariable("HeightToVzGain",b.HeightToVzGain,"Transform","log")
    optimizableVariable("HeightIntegralGain",b.HeightIntegralGain)
    optimizableVariable("HeightVzLimit",b.HeightVzLimit)
    ];
end

function rr=local_mul_bounds(v,globalRange,loFactor,hiFactor)
v=max(double(v),eps);
rr=[max(double(globalRange(1)),v*double(loFactor)), ...
    min(double(globalRange(2)),v*double(hiFactor))];
if rr(2)<=rr(1), rr=double(globalRange); end
end

function tf=local_candidate_in_bounds(c,b)
c=local_upgrade_candidate(c);
tf = c.Np>=b.Np(1) && c.Np<=b.Np(2) && ...
     c.Nc>=b.Nc(1) && c.Nc<=b.Nc(2) && ...
     c.Wh>=b.Wh(1) && c.Wh<=b.Wh(2) && ...
     c.Wvz>=b.Wvz(1) && c.Wvz<=b.Wvz(2) && ...
     c.Wq>=b.Wq(1) && c.Wq<=b.Wq(2) && ...
     c.RateScale>=b.RateScale(1) && c.RateScale<=b.RateScale(2) && ...
     c.Authority>=b.Authority(1) && c.Authority<=b.Authority(2) && ...
     c.HeightToVzGain>=b.HeightToVzGain(1) && c.HeightToVzGain<=b.HeightToVzGain(2) && ...
     c.HeightIntegralGain>=b.HeightIntegralGain(1) && c.HeightIntegralGain<=b.HeightIntegralGain(2) && ...
     c.HeightVzLimit>=b.HeightVzLimit(1) && c.HeightVzLimit<=b.HeightVzLimit(2);
end

function [seeds,info]=local_collect_unified_seeds(bankRoot,checkpoint,cfgId,context,opts)
seeds={};
info=struct("method","default","verified_count",0);

% v31.2 architecture requalification must test the CURRENT context's measured
% old-best controller first. Cross-context GPR/IDW seeds are not an A/B test of
% the architecture and therefore must not displace this one certification.
archRequal=false;
try, archRequal=isfield(opts,'UnifiedV31ArchitectureRequal')&&logical(opts.UnifiedV31ArchitectureRequal); catch, end
k=cfgId+1;
if archRequal && k<=numel(checkpoint.best_candidate) && ~isempty(checkpoint.best_candidate{k})
    seeds{end+1,1}=local_clamp_unified_candidate(checkpoint.best_candidate{k},opts); %#ok<AGROW>
    info.method="v31_2_old_best_requal";
    info.verified_count=0;
    return;
end

% v30.5: exact SAME-context measured evaluations are the highest-value resume
% seeds across mission generations. They may be near-pass failures, but only
% readable non-infrastructure, non-hard-fail certifications are eligible.
sameContextSeeds=local_same_context_evaluation_seeds(bankRoot,context,opts);
for i=1:numel(sameContextSeeds), seeds{end+1,1}=sameContextSeeds{i}; end %#ok<AGROW>

[pred,nearest,method] = local_predict_controller_from_learning_bank(bankRoot,context,opts);
info.method=method;
info.verified_count=height(nearest);
if ~isempty(pred), seeds{end+1,1}=pred; end %#ok<AGROW>

for i=1:min(height(nearest),round(double(opts.UnifiedNearestVerifiedSeeds)))
    try, seeds{end+1,1}=local_candidate_from_learning_row(nearest(i,:),opts); end %#ok<AGROW>
end

% Current mission's own prior candidates are valuable seeds, but they are not
% a separate strategy. They simply enter the same transfer pool.
k=cfgId+1;
if k<=numel(checkpoint.best_candidate) && ~isempty(checkpoint.best_candidate{k})
    seeds{end+1,1}=local_clamp_unified_candidate(checkpoint.best_candidate{k},opts); %#ok<AGROW>
end
if k<=numel(checkpoint.stageA_candidate) && ~isempty(checkpoint.stageA_candidate{k})
    seeds{end+1,1}=local_clamp_unified_candidate(checkpoint.stageA_candidate{k},opts); %#ok<AGROW>
end
if k<=numel(checkpoint.stageB_candidate) && ~isempty(checkpoint.stageB_candidate{k})
    seeds{end+1,1}=local_clamp_unified_candidate(checkpoint.stageB_candidate{k},opts); %#ok<AGROW>
end

% Once enough cross-context evaluations exist, use a global performance GPR
% to propose a few exploration/exploitation seeds.
gprSeeds=local_global_performance_seeds(bankRoot,context,pred,opts);
for i=1:numel(gprSeeds), seeds{end+1,1}=gprSeeds{i}; end %#ok<AGROW>

seeds{end+1,1}=local_default_unified_candidate(opts);

% Deduplicate and clamp.
out={}; sigs=strings(0,1);
for i=1:numel(seeds)
    c=local_clamp_unified_candidate(seeds{i},opts);
    s=string(local_candidate_signature(c));
    if ~any(sigs==s)
        out{end+1,1}=c; %#ok<AGROW>
        sigs(end+1,1)=s; %#ok<AGROW>
    end
end
seeds=out;
end

function seeds=local_same_context_evaluation_seeds(bankRoot,context,opts)
seeds={};
file=fullfile(bankRoot,"evaluations.csv");
if ~isfile(file), return; end
try, E=readtable(file,'VariableNamingRule','preserve','TextType','string'); catch, return; end
vars=string(E.Properties.VariableNames);
req=["context_signature","gate_ratio","hard_fail","formal_pass"];
if isempty(E) || ~all(ismember(req,vars)), return; end
sig=string(local_context_signature(context,opts));
keep=string(E.context_signature)==sig & ~local_table_bool(E,"hard_fail");
if ismember("infrastructure_fail",vars), keep=keep & ~local_table_bool(E,"infrastructure_fail"); end
gate=local_table_num(E,"gate_ratio"); keep=keep & isfinite(gate) & gate>0;
E=E(keep,:); if isempty(E), return; end
E=sortrows(E,{'formal_pass','gate_ratio'},{'descend','ascend'});
n=min(height(E),round(double(opts.UnifiedSameContextEvaluationSeeds)));
for i=1:n
    try, seeds{end+1,1}=local_candidate_from_learning_row(E(i,:),opts); end %#ok<AGROW>
end
end

function [pred,nearest,method]=local_predict_controller_from_learning_bank(bankRoot,context,opts)
pred=[];
nearest=table();
method="default";
file=fullfile(bankRoot,"verified_controllers.csv");
if ~isfile(file), return; end
try
    V=readtable(file,'VariableNamingRule','preserve','TextType','string');
catch
    return;
end
if isempty(V) || ~ismember("formal_pass",string(V.Properties.VariableNames)), return; end
V=V(local_table_bool(V,"formal_pass"),:);
if isempty(V), return; end

x0=local_context_matrix(context);
X=local_context_matrix(V);
d=sqrt(sum((X-x0).^2,2));
[~,ord]=sort(d,"ascend");
V=V(ord,:);
d=d(ord);
nearest=V(1:min(height(V),round(double(opts.UnifiedNearestVerifiedPool))),:);

names=local_unified_parameter_names();
Xphys=local_context_matrix(V);
Xkey=round(Xphys*1.0e6)/1.0e6;
nDistinctPhysical=size(unique(Xkey,'rows'),1);
if height(V)>=round(double(opts.UnifiedGprMinVerifiedPoints)) && ...
        nDistinctPhysical>=round(double(opts.UnifiedGprMinDistinctContexts))
    try
        vals=zeros(1,numel(names));
        Xall=Xphys;
        for j=1:numel(names)
            y=local_table_num(V,names(j));
            mdl=fitrgp(Xall,y,"KernelFunction","ardsquaredexponential", ...
                "Standardize",true,"BasisFunction","constant");
            vals(j)=predict(mdl,x0);
        end
        pred=local_candidate_from_parameter_vector(vals,opts);
        method="gpr_context_to_controller";
        return;
    catch
    end
end

% Data-sparse fallback: inverse-distance weighted transfer.  It already lets
% cfg4 inherit cfg3 experience and later lets a new altitude/speed inherit the
% closest previous mission.
n=min(height(V),round(double(opts.UnifiedIdwNeighborCount)));
w=1./max(d(1:n),double(opts.UnifiedIdwDistanceFloor)).^2;
w=w/sum(w);
vals=zeros(1,numel(names));
for j=1:numel(names)
    col=local_table_num(V,names(j));
    vals(j)=sum(w.*col(1:n));
end
pred=local_candidate_from_parameter_vector(vals,opts);
method="inverse_distance_transfer";
end

function seeds=local_global_performance_seeds(bankRoot,context,baseSeed,opts)
seeds={};
file=fullfile(bankRoot,"evaluations.csv");
if isempty(baseSeed) || ~isfile(file), return; end
try, E=readtable(file,'VariableNamingRule','preserve','TextType','string'); catch, return; end
if height(E)<round(double(opts.UnifiedGlobalGprMinEvaluations)), return; end
gateCol=local_table_num(E,"gate_ratio");
valid=isfinite(gateCol) & gateCol>0 & ...
    gateCol<=double(opts.UnifiedGlobalGprMaxGateRatio);
E=E(valid,:);
if height(E)<round(double(opts.UnifiedGlobalGprMinEvaluations)), return; end

try
    X=local_global_surrogate_matrix(E,opts);
    y=min(local_table_num(E,"gate_ratio"),double(opts.UnifiedGlobalGprMaxGateRatio));
    mdl=fitrgp(X,y,"KernelFunction","ardsquaredexponential", ...
        "Standardize",true,"BasisFunction","constant");

    % Deterministic local candidate pool around the transferred controller.
    nPool=round(double(opts.UnifiedGlobalGprCandidatePool));
    state=rng;
    cleanup=onCleanup(@()rng(state)); %#ok<NASGU>
    rng(29000 + round(double(context.config_id(1))));
    C=cell(nPool,1);
    Xq=zeros(nPool,size(X,2));
    for i=1:nPool
        c=local_random_candidate_near(baseSeed,opts);
        C{i}=c;
        Xq(i,:)=local_global_surrogate_vector(context,c);
    end
    [mu,sd]=predict(mdl,Xq);
    acquisition=double(mu)-double(opts.UnifiedGlobalGprExplorationBeta)*double(sd);
    [~,ord]=sort(acquisition,"ascend");
    n=min(round(double(opts.UnifiedGlobalGprSeedCount)),numel(ord));
    seeds=C(ord(1:n));
    try, save(fullfile(bankRoot,"global_performance_gpr_latest.mat"),"mdl","-v7.3"); catch, end
catch
    seeds={};
end
end

function c=local_random_candidate_near(base,opts)
base=local_clamp_unified_candidate(base,opts);
c=base;
c.Np=round(base.Np + randi([-3 3]));
c.Nc=round(base.Nc + randi([-1 2]));
c.Wh=base.Wh*exp(0.75*randn);
c.Wvz=base.Wvz*exp(0.65*randn);
c.Wq=base.Wq*exp(0.65*randn);
c.RateScale=base.RateScale*exp(0.45*randn);
c.Authority=base.Authority+0.12*randn;
c.HeightToVzGain=base.HeightToVzGain*exp(0.55*randn);
c.HeightIntegralGain=max(0,base.HeightIntegralGain+0.0015*randn);
c.HeightVzLimit=base.HeightVzLimit*exp(0.35*randn);
c=local_clamp_unified_candidate(c,opts);
end

function X=local_global_surrogate_matrix(T,opts)
n=height(T);
X=zeros(n,19);
for i=1:n
    ctx=T(i,:);
    c=local_candidate_from_learning_row(T(i,:),opts);
    X(i,:)=local_global_surrogate_vector(ctx,c);
end
end

function x=local_global_surrogate_vector(context,c)
x=[local_context_matrix(context), ...
    double(c.Np)/10, double(c.Nc)/4, log(max(c.Wh,1e-6)), ...
    log(max(c.Wvz,1e-6)), log(max(c.Wq,1e-6)), ...
    log(max(c.RateScale,1e-6)), double(c.Authority), ...
    log(max(c.HeightToVzGain,1e-6)), double(c.HeightIntegralGain)*500, ...
    double(c.HeightVzLimit)];
end

function c=local_candidate_from_learning_row(r,opts)
x=table();
names=local_unified_parameter_names();
for j=1:numel(names), x.(char(names(j)))=local_table_num(r,names(j)); end
c=local_candidate_from_x(x,opts);
end

function c=local_candidate_from_parameter_vector(v,opts)
names=local_unified_parameter_names();
x=table();
for j=1:numel(names), x.(char(names(j)))=double(v(j)); end
c=local_candidate_from_x(x,opts);
c=local_clamp_unified_candidate(c,opts);
end

function names=local_unified_parameter_names()
names=["Np","Nc","Wh","Wvz","Wq","RateScale","Authority", ...
    "HeightToVzGain","HeightIntegralGain","HeightVzLimit"];
end

function c=local_default_unified_candidate(opts)
c=struct("Np",10,"Nc",3,"Wh",1.0,"Wvz",10.0,"Wq",4.0, ...
    "RateScale",1.5,"Authority",0.70,"HeightToVzGain",0.10, ...
    "HeightIntegralGain",0.0010,"HeightVzLimit",0.90, ...
    "Wva",double(opts.FixedAirspeedWeight),"Wpitch",double(opts.FixedPitchWeight), ...
    "MVWeights",double(opts.MVWeights(:)).', ...
    "MVRateWeights",double(opts.MVRateWeights(:)).'*1.5);
c=local_clamp_unified_candidate(c,opts);
end

function c=local_clamp_unified_candidate(c,opts)
c=local_upgrade_candidate(c);
c.Np=min(max(round(double(c.Np)),opts.UnifiedNpRange(1)),opts.UnifiedNpRange(2));
c.Nc=min(max(round(double(c.Nc)),opts.UnifiedNcRange(1)),opts.UnifiedNcRange(2));
c.Nc=min(c.Nc,c.Np);
c.Wh=min(max(double(c.Wh),opts.UnifiedWhRange(1)),opts.UnifiedWhRange(2));
c.Wvz=min(max(double(c.Wvz),opts.UnifiedWvzRange(1)),opts.UnifiedWvzRange(2));
c.Wq=min(max(double(c.Wq),opts.UnifiedWqRange(1)),opts.UnifiedWqRange(2));
c.RateScale=min(max(double(c.RateScale),opts.UnifiedRateScaleRange(1)),opts.UnifiedRateScaleRange(2));
c.Authority=min(max(double(c.Authority),opts.UnifiedAuthorityRange(1)),opts.UnifiedAuthorityRange(2));
c.HeightToVzGain=min(max(double(c.HeightToVzGain),opts.UnifiedKpRange(1)),opts.UnifiedKpRange(2));
c.HeightIntegralGain=min(max(double(c.HeightIntegralGain),opts.UnifiedKiRange(1)),opts.UnifiedKiRange(2));
c.HeightVzLimit=min(max(double(c.HeightVzLimit),opts.UnifiedVzLimitRange(1)),opts.UnifiedVzLimitRange(2));
c.Wva=double(opts.FixedAirspeedWeight);
c.Wpitch=double(opts.FixedPitchWeight);
c.MVWeights=double(opts.MVWeights(:)).';
c.MVRateWeights=double(opts.MVRateWeights(:)).'*c.RateScale;
end

function context=local_operating_context(master,cfgId,opts)
mass=local_cfg_double(opts.ContextMassKgByConfig,cfgId,NaN);
if ~isfinite(mass)
    mass=double(opts.ReferenceMassKg)-double(cfgId)*double(opts.CargoMassKg);
end
cg=local_cfg_double(opts.ContextCgXByConfig,cfgId,NaN);
cgKnown=isfinite(cg);
if ~cgKnown
    try
        cg=local_trim_field(master.trim_bank(cfgId+1),"cg_x_m",NaN);
        cgKnown=isfinite(cg);
    catch
    end
end
if ~isfinite(cg), cg=0.0; end
payloadRemaining=max(0,(double(opts.TotalDropCount)-double(cfgId))*double(opts.CargoMassKg));
context=table(double(cfgId),double(opts.TargetAltitudeM),double(opts.TargetAirspeedMps), ...
    double(opts.ReferenceMassKg),double(opts.CargoMassKg),double(mass),double(cg),logical(cgKnown), ...
    double(opts.TotalDropCount),double(payloadRemaining),double(cfgId)/max(1,double(opts.TotalDropCount)), ...
    'VariableNames',{'config_id','target_altitude_m','target_airspeed_mps','reference_mass_kg', ...
    'cargo_mass_kg','estimated_mass_kg','cg_x_m','cg_known','total_drop_count', ...
    'payload_remaining_kg','drop_fraction'});
end

function X=local_context_matrix(T)
% v31 dimensionless PHYSICAL context features. Target altitude is intentionally
% absent: H is a mission reference/qualification axis, not an aerodynamic
% Plant/controller identity. cfg remains a physical-state feature, never a
% strategy switch. Legacy context_signature still keeps H for audit and to
% prevent cross-altitude objective values being reused as same-context priors.
cgKnown=local_table_bool(T,"cg_known");
X=[ ...
    local_table_num(T,"target_airspeed_mps")/10.0, ...
    local_table_num(T,"estimated_mass_kg")/500.0, ...
    local_table_num(T,"cargo_mass_kg")/300.0, ...
    local_table_num(T,"total_drop_count")/4.0, ...
    local_table_num(T,"payload_remaining_kg")/600.0, ...
    local_table_num(T,"drop_fraction"), ...
    local_table_num(T,"config_id")/4.0, ...
    local_table_num(T,"cg_x_m")/0.20 .* double(cgKnown), ...
    double(cgKnown) ...
    ];
end

function v=local_table_num(T,name)
x=T.(char(name));
if isnumeric(x) || islogical(x)
    v=double(x);
    return;
end
s=string(x);
v=str2double(s);
end

function v=local_table_bool(T,name)
x=T.(char(name));
if islogical(x)
    v=logical(x);
    return;
end
if isnumeric(x)
    v=logical(double(x)~=0);
    return;
end
s=lower(strtrim(string(x)));
v = s=="true" | s=="1" | s=="yes";
end

function sig=local_context_signature(context,opts)
arch="legacy";
try
    if isfield(opts,'V31HeightGovernorEnabled') && logical(opts.V31HeightGovernorEnabled)
        arch="v31p2_height_governor";
    elseif isfield(opts,'V31ContinuousControllerStateEnabled') && logical(opts.V31ContinuousControllerStateEnabled)
        arch="v31p1_continuous_state";
    end
catch
end
sig=sprintf("H%.4f_V%.4f_M%.3f_Cargo%.3f_CG%.5f_CGk%d_cfg%d_Arch%s", ...
    double(context.target_altitude_m(1)),double(context.target_airspeed_mps(1)), ...
    double(context.estimated_mass_kg(1)),double(context.cargo_mass_kg(1)), ...
    double(context.cg_x_m(1)),logical(context.cg_known(1)),round(double(context.config_id(1))),char(arch));
end

function sig=local_mission_signature(opts)
sig=sprintf("H%.4f_V%.4f_RefM%.3f_Cargo%.3f_Ndrop%d", ...
    double(opts.TargetAltitudeM),double(opts.TargetAirspeedMps), ...
    double(opts.ReferenceMassKg),double(opts.CargoMassKg),round(double(opts.TotalDropCount)));
end

function [cp,changed]=local_apply_mission_guard(cp,sig)
changed=false;
old="";
if isfield(cp,"mission_signature"), old=string(cp.mission_signature); end
if strlength(old)==0
    cp.mission_signature=string(sig);
    return;
end
if old==string(sig), return; end
changed=true;
cp.mission_signature=string(sig);
cp.status=repmat("pending",5,1);
cp.verification_count=zeros(5,1);
cp.best_bank_path=strings(5,1);
cp.last_metrics=cell(5,1);
cp.equilibrium_probe_pass=false(5,1);
cp.equilibrium_probe_generation=zeros(5,1);
cp.plant_rebuild_count=zeros(5,1);
cp.tuning_stage=repmat("pending",5,1);
cp.all_verified=false;
cp.final_mission_attempted=false;
cp.final_mission_pass=false;
cp.final_mission_summary=table();
cp.final_mission_updated_at="";
cp.transition_move_transfer_scale=NaN;
cp.transition_integral_transfer_scale=NaN;
cp.transition_policy_source="";
cp.hidden_elevator_trim=NaN;
% Deliberately KEEP best_candidate/stage candidates as transfer seeds only.
end

function root=local_resolve_learning_bank_root(paths,opts)
root=string(opts.LearningBankRoot);
if strlength(root)==0
    root=string(fullfile(paths.matlabDir,"results","mpc_auto_global_learning_bank"));
elseif ~local_is_absolute_path(root)
    root=string(fullfile(paths.projectRoot,char(root)));
end
root=char(root);
end

function tf=local_is_absolute_path(p)
p=char(string(p));
tf=false;
if isempty(p), return; end
if ispc
    tf = numel(p)>=2 && p(2)==':';
else
    tf = startsWith(p,'/');
end
end

function local_bootstrap_learning_bank(checkpoint,master,outRoot,bankRoot,opts)
if ~logical(opts.UnifiedLearning), return; end
for cfgId=0:4
    k=cfgId+1;
    if checkpoint.status(k)~="verified" || isempty(checkpoint.best_candidate{k}) || isempty(checkpoint.last_metrics{k})
        continue;
    end
    try
        M=checkpoint.last_metrics{k};
        if logical(M.formal_pass(1)) && ~logical(M.hard_fail(1))
            local_record_verified_controller(bankRoot,master,cfgId,checkpoint.best_candidate{k},M,outRoot,opts,"checkpoint_bootstrap");
        end
    catch
    end
end
end

function local_record_verified_controller(bankRoot,master,cfgId,c,M,outRoot,opts,source)
if isempty(M) || ~logical(M.formal_pass(1)) || logical(M.hard_fail(1)), return; end
if ~isfolder(bankRoot), mkdir(bankRoot); end
ctx=local_operating_context(master,cfgId,opts);
try
    if ismember("context_mass_kg",string(M.Properties.VariableNames)) && isfinite(double(M.context_mass_kg(1)))
        ctx.estimated_mass_kg(1)=double(M.context_mass_kg(1));
    end
    if ismember("context_cg_x_m",string(M.Properties.VariableNames)) && isfinite(double(M.context_cg_x_m(1)))
        ctx.cg_x_m(1)=double(M.context_cg_x_m(1));
        ctx.cg_known(1)=true;
    end
catch
end
contextSig=local_context_signature(ctx,opts);
candidateSig=string(local_candidate_signature(c));
c=local_clamp_unified_candidate(c,opts);
row=table(string(datetime("now")),string(local_mission_signature(opts)),string(contextSig),candidateSig, ...
    double(cfgId),double(ctx.target_altitude_m),double(ctx.target_airspeed_mps), ...
    double(ctx.reference_mass_kg),double(ctx.cargo_mass_kg),double(ctx.estimated_mass_kg), ...
    double(ctx.cg_x_m),logical(ctx.cg_known),double(ctx.total_drop_count), ...
    double(ctx.payload_remaining_kg),double(ctx.drop_fraction), ...
    c.Np,c.Nc,c.Wh,c.Wvz,c.Wq,c.RateScale,c.Authority,c.HeightToVzGain,c.HeightIntegralGain,c.HeightVzLimit, ...
    local_formal_gate_ratio(M,opts),logical(M.formal_pass(1)),string(source),string(outRoot), ...
    'VariableNames',{'timestamp','mission_signature','context_signature','candidate_signature', ...
    'config_id','target_altitude_m','target_airspeed_mps','reference_mass_kg','cargo_mass_kg', ...
    'estimated_mass_kg','cg_x_m','cg_known','total_drop_count','payload_remaining_kg','drop_fraction', ...
    'Np','Nc','Wh','Wvz','Wq','RateScale','Authority','HeightToVzGain','HeightIntegralGain', ...
    'HeightVzLimit','gate_ratio','formal_pass','source','source_output_root'});
local_append_learning_table(fullfile(bankRoot,"verified_controllers.csv"),row, ...
    ["context_signature","candidate_signature"]);
end

function local_sync_unified_evaluation_bank(bankRoot,H,context,outRoot,opts)
if isempty(H), return; end
if ~isfolder(bankRoot), mkdir(bankRoot); end
n=height(H);
ctxSig=repmat(string(local_context_signature(context,opts)),n,1);
missionSig=repmat(string(local_mission_signature(opts)),n,1);
massVec=repmat(double(context.estimated_mass_kg),n,1);
cgVec=repmat(double(context.cg_x_m),n,1);
cgKnownVec=repmat(logical(context.cg_known),n,1);
if ismember("observed_mass_kg",string(H.Properties.VariableNames))
    m=double(H.observed_mass_kg); good=isfinite(m); massVec(good)=m(good);
end
if ismember("observed_cg_x_m",string(H.Properties.VariableNames))
    g=double(H.observed_cg_x_m); good=isfinite(g); cgVec(good)=g(good); cgKnownVec(good)=true;
end
rows=table(string(H.timestamp),missionSig,ctxSig,string(H.candidate_signature), ...
    double(H.config_id),repmat(double(context.target_altitude_m),n,1), ...
    repmat(double(context.target_airspeed_mps),n,1),repmat(double(context.reference_mass_kg),n,1), ...
    repmat(double(context.cargo_mass_kg),n,1),massVec, ...
    cgVec,cgKnownVec,repmat(double(context.total_drop_count),n,1), ...
    repmat(double(context.payload_remaining_kg),n,1),repmat(double(context.drop_fraction),n,1), ...
    double(H.Np),double(H.Nc),double(H.Wh),double(H.Wvz),double(H.Wq),double(H.RateScale), ...
    double(H.Authority),double(H.HeightToVzGain),double(H.HeightIntegralGain),double(H.HeightVzLimit), ...
    double(H.objective),double(H.gate_ratio),logical(H.hard_fail),logical(H.formal_pass), ...
    string(H.source),repmat(string(outRoot),n,1), ...
    'VariableNames',{'timestamp','mission_signature','context_signature','candidate_signature', ...
    'config_id','target_altitude_m','target_airspeed_mps','reference_mass_kg','cargo_mass_kg', ...
    'estimated_mass_kg','cg_x_m','cg_known','total_drop_count','payload_remaining_kg','drop_fraction', ...
    'Np','Nc','Wh','Wvz','Wq','RateScale','Authority','HeightToVzGain','HeightIntegralGain', ...
    'HeightVzLimit','objective','gate_ratio','hard_fail','formal_pass','source','source_output_root'});
local_append_learning_table(fullfile(bankRoot,"evaluations.csv"),rows, ...
    ["context_signature","candidate_signature"]);
end

function local_append_learning_table(file,rows,keyNames)
if isempty(rows), return; end
try
    if isfile(file)
        old=readtable(file,'VariableNamingRule','preserve','TextType','string');
        if isequal(string(old.Properties.VariableNames),string(rows.Properties.VariableNames))
            T=[old;rows];
        else
            backup=string(file)+".schema_backup_"+string(datetime("now","Format","yyyyMMdd_HHmmss"))+".csv";
            copyfile(file,backup);
            T=rows;
        end
    else
        T=rows;
    end
    key=strings(height(T),1);
    for i=1:numel(keyNames)
        k=string(keyNames(i));
        key=key+"|"+string(T.(char(k)));
    end
    [~,ia]=unique(key,"last");
    T=T(sort(ia),:);
    writetable(T,file);
catch ME
    warning("AirdropX:UnifiedLearner:LearningBankWrite","Could not update %s: %s",file,ME.message);
end
end

% ========================================================================
% v20 cfg2+ diagnosis and staged tuning
% ========================================================================
function [master, checkpoint, physicalNominals, finalMetrics, passed, status] = ...
    local_run_cfg2plus_pipeline(paths, master, checkpoint, cfgId, physicalNominals, ...
    hiddenTrim, cfgRoot, outRoot, masterFile, checkpointFile, opts)

passed = false;
status = "diagnosing";
finalMetrics = local_empty_cert_like_metrics(cfgId);

% If a previous invocation stopped inside trim/ID/identify, resume that exact
% plant_training directory instead of archiving/restarting it.
if checkpoint.tuning_stage(cfgId+1) == "plant_rebuild_in_progress"
    fprintf("[V21-200m] cfg%d resuming interrupted Plant rebuild.\n",cfgId);
    [master, plantInfo] = local_prepare_missing_plant(master, cfgId, paths, cfgRoot, opts, true);
    checkpoint.plant_ready(cfgId+1) = true;
    checkpoint.plant_validation{cfgId+1} = plantInfo.validation;
    checkpoint.plant_rebuild_count(cfgId+1) = checkpoint.plant_rebuild_count(cfgId+1) + 1;
    checkpoint.plant_generation(cfgId+1) = checkpoint.plant_generation(cfgId+1) + 1;
    checkpoint.tuning_stage(cfgId+1) = "equilibrium";
    checkpoint.equilibrium_probe_pass(cfgId+1) = false;
    checkpoint.equilibrium_probe_generation(cfgId+1) = 0;
    local_save_master(masterFile,master);
    local_save_checkpoint(checkpointFile,checkpoint);
    physicalNominals = local_physical_nominals(master,hiddenTrim);
end

% One long post-drop nominal probe per current Plant generation.  cfgN uses
% verified cfg0..cfg(N-1) controllers during the preparatory drops, then cfgN
% authority is forced to zero so the aircraft sees exactly its nominal
% physical elevator/throttle for a long settling window.
generation = checkpoint.plant_generation(cfgId+1);
needProbe = checkpoint.equilibrium_probe_generation(cfgId+1) ~= generation || ...
    ~logical(checkpoint.equilibrium_probe_pass(cfgId+1));
if needProbe
    probeRoot = fullfile(cfgRoot, "postdrop_equilibrium_probe");
    [P, probePass] = local_run_postdrop_equilibrium_probe(paths, master, checkpoint, cfgId, ...
        physicalNominals, hiddenTrim, probeRoot, opts);
    writetable(P, fullfile(probeRoot, "equilibrium_probe_summary.csv"));
    checkpoint.equilibrium_probe_pass(cfgId+1) = logical(probePass);
    checkpoint.equilibrium_probe_generation(cfgId+1) = generation;
    checkpoint.updated_at = string(datetime("now"));
    local_save_checkpoint(checkpointFile, checkpoint);

    if ~probePass
        fprintf("[V21-200m] cfg%d nominal equilibrium probe FAIL: hSlope=%.4f m/s, vzTail=%.4f m/s.\n", ...
            cfgId, P.equilibrium_height_slope_mps(1), P.tail_vz_mps(1));

        if checkpoint.plant_rebuild_count(cfgId+1) >= double(opts.MaxAutoPlantRebuildsPerConfig)
            checkpoint.status(cfgId+1) = "plant_probe_failed";
            status = "plant_probe_failed";
            finalMetrics = P;
            return;
        end

        % The v18 cfg2 history belongs to the old non-equilibrium Plant/trim.
        % Preserve it, but never warm-start a new Plant from those objective
        % values because the model coordinate system has changed.
        backupRoot = local_archive_cfg_root(cfgRoot, outRoot, cfgId);
        fprintf("[V21-200m] cfg%d old search archived to %s\n", cfgId, backupRoot);
        checkpoint = local_reset_cfg_after_plant_rebuild(checkpoint, cfgId);
        checkpoint.tuning_stage(cfgId+1) = "plant_rebuild_in_progress";
        checkpoint.updated_at = string(datetime("now"));
        local_save_checkpoint(checkpointFile, checkpoint);
        if ~isfolder(cfgRoot), mkdir(cfgRoot); end

        fprintf("[V21-200m] cfg%d rebuilding trim -> clean ID -> n4sid at 200 m.\n", cfgId);
        [master, plantInfo] = local_prepare_missing_plant(master, cfgId, paths, cfgRoot, opts, true);
        checkpoint.plant_ready(cfgId+1) = true;
        checkpoint.plant_validation{cfgId+1} = plantInfo.validation;
        checkpoint.plant_rebuild_count(cfgId+1) = checkpoint.plant_rebuild_count(cfgId+1) + 1;
        checkpoint.plant_generation(cfgId+1) = checkpoint.plant_generation(cfgId+1) + 1;
        checkpoint.tuning_stage(cfgId+1) = "equilibrium";
        generation = checkpoint.plant_generation(cfgId+1);
        local_save_master(masterFile, master);
        local_save_checkpoint(checkpointFile, checkpoint);
        physicalNominals = local_physical_nominals(master, hiddenTrim);
        writetable(local_nominal_table(master, physicalNominals, hiddenTrim), ...
            fullfile(outRoot, "physical_nominals_200m.csv"));

        % The rebuilt trim must pass the same real sequential-drop equilibrium
        % probe before a single MPC tuning evaluation is allowed.
        probeRoot2 = fullfile(cfgRoot, "postdrop_equilibrium_probe_after_rebuild");
        [P2, probePass2] = local_run_postdrop_equilibrium_probe(paths, master, checkpoint, cfgId, ...
            physicalNominals, hiddenTrim, probeRoot2, opts);
        writetable(P2, fullfile(probeRoot2, "equilibrium_probe_summary.csv"));
        checkpoint.equilibrium_probe_pass(cfgId+1) = logical(probePass2);
        checkpoint.equilibrium_probe_generation(cfgId+1) = generation;
        checkpoint.updated_at = string(datetime("now"));
        local_save_checkpoint(checkpointFile, checkpoint);
        if ~probePass2
            fprintf("[V21-200m] cfg%d rebuilt nominal STILL fails equilibrium probe. Stop before MPC tuning.\n", cfgId);
            checkpoint.status(cfgId+1) = "plant_probe_failed";
            status = "plant_probe_failed";
            finalMetrics = P2;
            return;
        end
        fprintf("[V21-200m] cfg%d rebuilt nominal equilibrium PASS. Starting staged MPC tuning.\n", cfgId);
    else
        fprintf("[V21-200m] cfg%d nominal equilibrium probe PASS.\n", cfgId);
    end
else
    fprintf("[V21-200m] cfg%d equilibrium probe already PASS for Plant generation %d -> reuse.\n", ...
        cfgId, generation);
end

[checkpoint, finalMetrics, passed, status] = local_run_staged_tuning( ...
    paths, master, checkpoint, cfgId, physicalNominals, hiddenTrim, cfgRoot, opts);
end

function [P, pass] = local_run_postdrop_equilibrium_probe(paths, master, checkpoint, cfgId, ...
    physicalNominals, hiddenTrim, probeRoot, opts)
if ~isfolder(probeRoot), mkdir(probeRoot); end
c = local_probe_candidate(opts);
bankMat = fullfile(probeRoot, "equilibrium_probe_bank.mat");
local_build_combined_bank(master, checkpoint, cfgId, c, physicalNominals, bankMat, opts);

[authorityByCfg, gainByCfg, integralGainByCfg, vzLimByCfg] = local_control_vectors(checkpoint, cfgId, c);
authorityByCfg(cfgId+1) = 0.0;
gainByCfg(cfgId+1) = 0.0;
integralGainByCfg(cfgId+1) = 0.0;

reachS = local_config_reach_time(cfgId, opts);
stopS = reachS + double(opts.PostDropProbeDurationS);
initialElev = physicalNominals(1) - hiddenTrim;
simResult = airdropx_auto_run_closed_loop( ...
    "ProjectRoot", paths.projectRoot, ...
    "MpcBankMat", bankMat, ...
    "OutputRoot", fullfile(probeRoot, "run"), ...
    "CaseId", sprintf("cfg%d_equilibrium_probe",cfgId), ...
    "StopTimeS", stopS, ...
    "FixedConfigId", NaN, ...
    "FixedDropTotal", cfgId, ...
    "FixedDropStartS", opts.DropStartS, ...
    "FixedDropIntervalS", opts.DropIntervalS, ...
    "InitialAltitudeM", opts.TargetAltitudeM, ...
    "InitialAirspeedMps", opts.TargetAirspeedMps, ...
    "InitialPitchDeg", master.trim_bank(1).pitch_deg, ...
    "InitialFlightPathDeg", 0.0, ...
    "InitialElevatorDelta", initialElev, ...
    "InitialThrottleCmd", master.trim_bank(1).throttle_cmd, ...
    "ReferenceMassKg", opts.ReferenceMassKg, ...
    "CargoMassKg", opts.CargoMassKg, ...
    "HiddenElevatorTrim", hiddenTrim, ...
    "MpcEnableTimeS", opts.MpcEnableTimeS, ...
    "MpcAuthorityScale", 1.0, ...
    "MpcAuthorityByConfig", authorityByCfg, ...
    "HeightToVzGain", 0.0, ...
    "HeightToVzGainByConfig", gainByCfg, ...
    "HeightIntegralGain", 0.0, ...
    "HeightIntegralGainByConfig", integralGainByCfg, ...
    "HeightVzRefLimitMps", opts.StageAHeightVzLimit, ...
    "HeightVzRefLimitByConfig", vzLimByCfg, ...
    "V31ContinuousControllerStateEnabled", logical(opts.V31ContinuousControllerStateEnabled), ...
    "V31HeightGovernorEnabled", logical(opts.V31HeightGovernorEnabled), ...
    "V31HeightVzSlewRateMps2", double(opts.V31HeightVzSlewRateMps2), ...
    "V31HeightBiasFraction", double(opts.V31HeightBiasFraction), ...
    "V31HeightBiasLeak", double(opts.V31HeightBiasLeak), ...
    "TestPulse1StartS", Inf, "TestPulse1DurationS", 0.0, ...
    "TestPulse1Elevator", 0.0, "TestPulse1Throttle", 0.0, ...
    "TestPulse2StartS", Inf, "TestPulse2DurationS", 0.0, ...
    "TestPulse2Elevator", 0.0, "TestPulse2Throttle", 0.0, ...
    "ElevatorDevStepLimit", opts.ElevatorDeviationRateLimit, ...
    "ThrottleDevStepLimit", opts.ThrottleDeviationRateLimit, ...
    "TrustAltitudeM", 1.0e6, ...
    "TrustAirspeedMps", opts.TrustAirspeedMps, ...
    "TrustPitchDeg", opts.TrustPitchDeg, ...
    "TrustVzMps", opts.TrustVzMps, ...
    "TrustQDps", opts.TrustQDps, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", master.trim_bank(cfgId+1).pitch_deg, ...
    "UseTrimPitchReference", 1);

P = local_equilibrium_probe_metrics(simResult.timeseries, cfgId, reachS, opts);
pass = logical(P.equilibrium_pass(1));
local_plot(simResult.timeseries, physicalNominals(cfgId+1), ...
    master.trim_bank(cfgId+1).throttle_cmd, ...
    fullfile(probeRoot,"equilibrium_probe_curves.png"), ...
    sprintf("v29 cfg%d operating-point equilibrium probe",cfgId));
end

function P = local_equilibrium_probe_metrics(T,cfgId,reachS,opts)
t = double(T.time_s(:));
h = local_col(T,"altitude_m");
V = local_col(T,"airspeed_mps");
vz = local_col(T,"vz_up_mps");
q = local_col(T,"q_dps");
dropCount = local_col(T,"drop_count");

evalStart = reachS + double(opts.PostDropProbeSettleS);
mask = isfinite(t) & t >= evalStart & round(dropCount) >= cfgId;
if nnz(mask) < 20
    mask = isfinite(t) & round(dropCount) >= cfgId;
end
tEnd = max(t(mask),[],"omitnan");
tail = mask & t >= tEnd - double(opts.PostDropProbeTailWindowS);
if nnz(tail) < 10, tail = mask; end

hSlope = local_linear_slope(t(tail),h(tail));
vzMed = median(vz(tail),"omitnan");
qRms = local_rms(q(tail));
vaErr = median(V(tail),"omitnan") - double(opts.TargetAirspeedMps);
hErr = h - double(opts.TargetAltitudeM);
hRms = local_rms(hErr(mask));
vaRms = local_rms(V(mask)-double(opts.TargetAirspeedMps));
tailHErr = median(hErr(tail),"omitnan");
reachedCfg = any(round(dropCount(isfinite(dropCount))) >= cfgId);
hard = ~reachedCfg || any(~isfinite([hSlope vzMed qRms vaErr]));
pass = ~hard && ...
    abs(hSlope) <= double(opts.PostDropProbeMaxAbsHeightSlopeMps) && ...
    abs(vzMed) <= double(opts.PostDropProbeMaxAbsVzMps) && ...
    qRms <= double(opts.PostDropProbeMaxQRmsDps) && ...
    abs(vaErr) <= double(opts.PostDropProbeMaxAirspeedErrorMps);

% Include the normal certification columns so all_config_status.csv remains
% structurally consistent even when we intentionally stop at the diagnosis.
P = table(cfgId,min(h,[],"omitnan"),max(h,[],"omitnan"),hRms, ...
    max(abs(hErr(mask)),[],"omitnan"), ...
    median(h(tail),"omitnan")-median(h(mask & t <= evalStart+3.0),"omitnan"), ...
    vaRms,local_rms(vz(mask)),local_rms(q(mask)),tailHErr,vzMed,median(q(tail),"omitnan"), ...
    0.0,0.0,0.0,0.0,reachedCfg,hard,false,NaN, ...
    hSlope,vaErr,pass, ...
    'VariableNames',{'config_id','min_altitude_m','max_altitude_m','steady_h_rms_m', ...
    'steady_h_max_abs_m','steady_h_drift_m','steady_Va_rms_mps','steady_vz_rms_mps', ...
    'steady_q_rms_dps','tail_h_error_m','tail_vz_mps','tail_q_dps', ...
    'max_physical_elevator_deviation','max_throttle_deviation', ...
    'max_bridge_elevator_error','max_bridge_throttle_error','reached_config', ...
    'hard_fail','formal_pass','rank_score','equilibrium_height_slope_mps', ...
    'equilibrium_airspeed_error_mps','equilibrium_pass'});
end

function [master,checkpoint,physicalNominals,Pout,pass] = ...
    local_universal_equilibrium_recovery(paths,master,checkpoint,cfgId, ...
    physicalNominals,hiddenTrim,cfgRoot,masterFile,checkpointFile,Pin,opts)
% v30.4 universal near-pass recovery for every mission context.
% IMPORTANT: no target-altitude or cfg-specific branch belongs here.
Pout = Pin;
pass = logical(Pin.equilibrium_pass(1));
ratio0 = local_equilibrium_probe_gate_ratio(Pin,opts);
local_append_universal_recovery_log(cfgRoot,cfgId,"normal_probe",ratio0,pass,Pin,"");
if pass, return; end

hard = false;
try, hard = logical(Pin.hard_fail(1)); catch, end
if hard || ~isfinite(ratio0) || ratio0 > double(opts.UniversalRecoveryNearPassGateRatioMax)
    fprintf("[V30.4-RECOVERY] cfg%d probe ratio %.3f is outside universal near-pass region -> full Plant rebuild path.\n",cfgId,ratio0);
    return;
end

fprintf("[V30.4-RECOVERY] cfg%d near-pass ratio %.3f -> extended equilibrium observation (universal rule).\n",cfgId,ratio0);
extOpts = opts;
extOpts.PostDropProbeDurationS = max(double(opts.PostDropProbeDurationS),double(opts.UniversalRecoveryExtendedProbeDurationS));
extOpts.PostDropProbeTailWindowS = max(double(opts.PostDropProbeTailWindowS),double(opts.UniversalRecoveryExtendedTailWindowS));
extRoot = fullfile(cfgRoot,"unified_equilibrium_probe_extended");
[Pext,passExt] = local_run_postdrop_equilibrium_probe(paths,master,checkpoint,cfgId, ...
    physicalNominals,hiddenTrim,extRoot,extOpts);
writetable(Pext,fullfile(extRoot,"equilibrium_probe_summary.csv"));
ratioExt = local_equilibrium_probe_gate_ratio(Pext,opts);
local_append_universal_recovery_log(cfgRoot,cfgId,"extended_probe",ratioExt,passExt,Pext,"");
Pout = Pext;
pass = logical(passExt);
if pass
    checkpoint.equilibrium_probe_pass(cfgId+1) = true;
    checkpoint.updated_at = string(datetime("now"));
    local_save_checkpoint(checkpointFile,checkpoint);
    return;
end

% Do not spend a local-search budget if the longer run has moved clearly out
% of the near-pass region.  This same criterion applies at every H/V/cfg.
if ~isfinite(ratioExt) || ratioExt > double(opts.UniversalRecoveryNearPassGateRatioMax)
    fprintf("[V30.4-RECOVERY] cfg%d extended ratio %.3f left near-pass region -> full Plant rebuild path.\n",cfgId,ratioExt);
    return;
end

% The mission's master was seeded by the globally nearest PlantContextBank
% entry before this function is reached.  Therefore this local retrim is
% automatically centered on the nearest transferable verified experience,
% without any hard-coded 'higher altitude' or cfg rule.
fprintf("[V30.4-RECOVERY] cfg%d extended probe still near-pass (%.3f) -> small local retrim before ID/Plant rebuild.\n",cfgId,ratioExt);
try
    [master,retrimInfo] = local_universal_nearpass_retrim(master,cfgId,paths,cfgRoot,hiddenTrim,opts);
    local_save_master(masterFile,master);
    physicalNominals = local_physical_nominals(master,hiddenTrim);
    retrimRoot = fullfile(cfgRoot,"unified_equilibrium_probe_after_local_retrim");
    [Pret,passRet] = local_run_postdrop_equilibrium_probe(paths,master,checkpoint,cfgId, ...
        physicalNominals,hiddenTrim,retrimRoot,extOpts);
    writetable(Pret,fullfile(retrimRoot,"equilibrium_probe_summary.csv"));
    ratioRet = local_equilibrium_probe_gate_ratio(Pret,opts);
    local_append_universal_recovery_log(cfgRoot,cfgId,"local_retrim_probe",ratioRet,passRet,Pret,retrimInfo.source);
    Pout = Pret;
    pass = logical(passRet);
    if pass
        checkpoint.equilibrium_probe_pass(cfgId+1) = true;
        checkpoint.updated_at = string(datetime("now"));
        local_save_checkpoint(checkpointFile,checkpoint);
    end
catch ME
    % A failed local retrim is a genuine recovery attempt, not infrastructure
    % evidence.  Preserve it and continue into the normal full rebuild path.
    local_append_universal_recovery_log(cfgRoot,cfgId,"local_retrim_error",Inf,false,Pout,string(ME.identifier)+" "+string(ME.message));
    warning("AirdropX:V30_4:UniversalLocalRetrimFailed", ...
        "cfg%d universal local retrim did not pass; continuing to full Plant rebuild: %s",cfgId,ME.message);
end
end

function ratio = local_equilibrium_probe_gate_ratio(P,opts)
try
    if isempty(P) || logical(P.hard_fail(1))
        ratio = Inf;
        return;
    end
    ratios = [ ...
        abs(double(P.equilibrium_height_slope_mps(1))) / max(double(opts.PostDropProbeMaxAbsHeightSlopeMps),eps); ...
        abs(double(P.tail_vz_mps(1))) / max(double(opts.PostDropProbeMaxAbsVzMps),eps); ...
        abs(double(P.steady_q_rms_dps(1))) / max(double(opts.PostDropProbeMaxQRmsDps),eps); ...
        abs(double(P.equilibrium_airspeed_error_mps(1))) / max(double(opts.PostDropProbeMaxAirspeedErrorMps),eps)];
    ratios = ratios(isfinite(ratios));
    if isempty(ratios), ratio=Inf; else, ratio=max(ratios); end
catch
    ratio = Inf;
end
end

function [master,info] = local_universal_nearpass_retrim(master,cfgId,paths,cfgRoot,hiddenTrim,opts)
% Small, context-agnostic retrim.  For sequential cfg>0 preparation,
% airdropx_auto_find_trim already freezes Pitch0/Gamma0 and spends BO on the
% true steady controls (elevator/throttle).  cfg0 uses the same call path but
% retains the physically necessary initial-state degrees of freedom.
root = fullfile(cfgRoot,"universal_nearpass_retrim");
if ~isfolder(root), mkdir(root); end
seedFile = fullfile(root,"nearest_context_seed.mat");
% Tell the generic trim search that the transferred current-cfg trim is an
% intentional warm-start center.  This remains context-agnostic: whichever
% PlantContextBank mission was nearest supplies this seed.
seedMaster = master;
try, seedMaster.trim_bank(cfgId+1).resume_seed_valid = true; catch, end
result = seedMaster; %#ok<NASGU>
save(seedFile,"result","-v7.3");
outMat = fullfile(root,"auto_trim_bank.mat");
trimResult = airdropx_auto_find_trim( ...
    "ProjectRoot",paths.projectRoot, ...
    "OutputMat",outMat, ...
    "WorkRoot",fullfile(root,"search"), ...
    "PreviousTrimMat",seedFile, ...
    "ConfigIds",cfgId, ...
    "TargetAltitudeM",opts.TargetAltitudeM, ...
    "TargetAirspeedMps",opts.TargetAirspeedMps, ...
    "ReferenceMassKg",opts.ReferenceMassKg, ...
    "CargoMassKg",opts.CargoMassKg, ...
    "SearchAltitudeM",max(double(opts.TargetAltitudeM),double(opts.TrimSearchAltitudeM)), ...
    "StopTimeS",max(double(opts.TrimStopTimeS),double(opts.UniversalRecoveryLocalRetrimStopTimeS)), ...
    "MaxTrimStopTimeS",max(60.0,double(opts.UniversalRecoveryLocalRetrimStopTimeS)+20.0), ...
    "RecordStartS",opts.TrimRecordStartS, ...
    "MaxObjectiveEvaluations",round(double(opts.UniversalRecoveryLocalRetrimEvaluations)), ...
    "ReuseVerifiedTrim",false, ...
    "ReuseFailedAsWarmStart",true, ...
    "UseRecoverySearch",false, ...
    "UseTailEquilibriumRescue",false, ...
    "FocusedElevatorHalfWidth",double(opts.UniversalRecoveryElevatorHalfWidth), ...
    "FocusedThrottleHalfWidth",double(opts.UniversalRecoveryThrottleHalfWidth), ...
    "FocusedPitchHalfWidthDeg",double(opts.UniversalRecoveryPitchHalfWidthDeg), ...
    "FocusedGammaHalfWidthDeg",double(opts.UniversalRecoveryGammaHalfWidthDeg), ...
    "RefineElevatorHalfWidth",double(opts.UniversalRecoveryRefineElevatorHalfWidth), ...
    "RefineThrottleHalfWidth",double(opts.UniversalRecoveryRefineThrottleHalfWidth), ...
    "RefinePitchHalfWidthDeg",double(opts.UniversalRecoveryRefinePitchHalfWidthDeg), ...
    "RefineGammaHalfWidthDeg",double(opts.UniversalRecoveryRefineGammaHalfWidthDeg), ...
    "MaxTrimAbsVzMps",opts.RetrimMaxAbsVzMps, ...
    "MaxTailAbsVzMps",opts.RetrimMaxTailAbsVzMps, ...
    "MaxTailHeightSlopeMps",opts.RetrimMaxTailHeightSlopeMps, ...
    "TailRescueMaxAbsVzMps",opts.RetrimTailRescueMaxAbsVzMps, ...
    "TailRescueMaxHeightSlopeMps",opts.RetrimTailRescueMaxHeightSlopeMps, ...
    "PreparationTrimBank",master.trim_bank, ...
    "UsePreparationTrimSchedule",true, ...
    "MaxFullAltitudeRmsM",opts.RetrimMaxFullAltitudeRmsM, ...
    "MaxFullAltitudeMaxAbsM",opts.RetrimMaxFullAltitudeMaxAbsM, ...
    "MaxEarlyAltitudeRmsM",opts.RetrimMaxEarlyAltitudeRmsM, ...
    "MaxEarlyAltitudeMaxAbsM",opts.RetrimMaxEarlyAltitudeMaxAbsM, ...
    "UseParallel",logical(opts.UseParallel));
newTrim = trimResult.trim_bank(cfgId+1);
newTrim.altitude_m = double(opts.TargetAltitudeM);
newTrim.airspeed_mps = double(opts.TargetAirspeedMps);
% Keep the bridge's physical nominal synchronized with the newly accepted
% external trim command; otherwise an old seed's physical_elevator_cmd could
% silently override the retrim on the next probe.
newTrim.physical_elevator_cmd = double(hiddenTrim) + double(newTrim.elevator_cmd);
master.trim_bank = local_assign_struct_element_union(master.trim_bank,cfgId+1,newTrim);
info = struct("source","nearest_context_seed+universal_local_retrim","trim_mat",string(outMat));
end

function local_append_universal_recovery_log(cfgRoot,cfgId,stage,ratio,pass,P,note)
try
    file=fullfile(cfgRoot,"universal_recovery_summary.csv");
    hSlope=NaN; tailVz=NaN; qRms=NaN; vaErr=NaN; hard=false;
    if ~isempty(P)
        try, hSlope=double(P.equilibrium_height_slope_mps(1)); catch, end
        try, tailVz=double(P.tail_vz_mps(1)); catch, end
        try, qRms=double(P.steady_q_rms_dps(1)); catch, end
        try, vaErr=double(P.equilibrium_airspeed_error_mps(1)); catch, end
        try, hard=logical(P.hard_fail(1)); catch, end
    end
    row=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),cfgId,string(stage), ...
        double(ratio),logical(pass),logical(hard),hSlope,tailVz,qRms,vaErr,string(note), ...
        'VariableNames',{'timestamp','config_id','stage','gate_ratio','pass','hard_fail', ...
        'height_slope_mps','tail_vz_mps','q_rms_dps','airspeed_error_mps','note'});
    if isfile(file)
        old=readtable(file,'TextType','string');
        if isequal(string(old.Properties.VariableNames),string(row.Properties.VariableNames))
            row=[old;row]; %#ok<AGROW>
        end
    end
    writetable(row,file);
catch
end
end

function c = local_probe_candidate(opts)
c = struct("Np",10,"Nc",3,"Wh",1.0,"Wvz",10.0,"Wq",4.0, ...
    "RateScale",1.5,"Authority",0.0,"HeightToVzGain",0.0, ...
    "HeightIntegralGain",0.0,"HeightVzLimit",double(opts.StageAHeightVzLimit), ...
    "Wva",double(opts.FixedAirspeedWeight),"Wpitch",double(opts.FixedPitchWeight), ...
    "MVWeights",double(opts.MVWeights(:)).', ...
    "MVRateWeights",double(opts.MVRateWeights(:)).'*1.5);
end

function backupRoot = local_archive_cfg_root(cfgRoot,outRoot,cfgId)
stamp = string(datetime("now","Format","yyyyMMdd_HHmmss"));
backupRoot = fullfile(outRoot, sprintf("cfg%d_pre_plant_rebuild_%s",cfgId,char(stamp)));
suffix = 1;
while isfolder(backupRoot)
    backupRoot = fullfile(outRoot, sprintf("cfg%d_pre_plant_rebuild_%s_%02d",cfgId,char(stamp),suffix));
    suffix = suffix + 1;
end
if isfolder(cfgRoot)
    movefile(cfgRoot, backupRoot);
else
    mkdir(backupRoot);
end
mkdir(cfgRoot);
end

function checkpoint = local_reset_cfg_after_plant_rebuild(checkpoint,cfgId)
k = cfgId+1;
checkpoint.status(k) = "pending";
checkpoint.best_candidate{k} = [];
checkpoint.best_objective(k) = Inf;
checkpoint.best_bank_path(k) = "";
checkpoint.last_metrics{k} = [];
checkpoint.failed_certified_signatures{k} = strings(0,1);
checkpoint.verification_count(k) = 0;
checkpoint.equilibrium_probe_pass(k) = false;
checkpoint.equilibrium_probe_generation(k) = 0;
checkpoint.stageA_candidate{k} = [];
checkpoint.stageB_candidate{k} = [];
checkpoint.tuning_stage(k) = "equilibrium";
end

function [checkpoint, finalMetrics, passed, status] = local_run_staged_tuning( ...
    paths,master,checkpoint,cfgId,physicalNominals,hiddenTrim,cfgRoot,opts)
passed = false;
status = "inner_search";
finalMetrics = local_empty_cert_like_metrics(cfgId);
parallelEnabled = local_prepare_parallel_pool(paths,opts);

% ---------------- Stage A: inner learned-MPC dynamics -------------------
% Keep Kp fixed and Ki=0.  The only question is whether the Plant/MPC can
% command the correct vertical response while q and Va stay controlled.
[cA, MA, innerPass] = local_stageA_search(paths,master,checkpoint,cfgId, ...
    physicalNominals,hiddenTrim,cfgRoot,parallelEnabled,opts);
checkpoint.stageA_candidate{cfgId+1} = cA;
checkpoint.tuning_stage(cfgId+1) = "inner";
finalMetrics = MA;
if ~innerPass
    checkpoint.status(cfgId+1) = "inner_failed";
    status = "inner_failed";
    return;
end
fprintf("[V21-200m] cfg%d Stage A inner MPC PASS. Kp remains fixed, Ki=0.\n",cfgId);

% ---------------- Stage B: proportional height outer loop ---------------
[cB, MB, formalB, outerPass] = local_stageB_search(paths,master,checkpoint,cfgId,cA, ...
    physicalNominals,hiddenTrim,cfgRoot,parallelEnabled,opts);
checkpoint.stageB_candidate{cfgId+1} = cB;
checkpoint.tuning_stage(cfgId+1) = "outer";
finalMetrics = MB;
if formalB
    checkpoint = local_commit_candidate(master,checkpoint,cfgId,cB,MB,physicalNominals,cfgRoot,opts);
    checkpoint.status(cfgId+1) = "verified";
    checkpoint.tuning_stage(cfgId+1) = "verified";
    passed = true; status = "verified";
    return;
end
if ~outerPass
    checkpoint.status(cfgId+1) = "outer_failed";
    status = "outer_failed";
    return;
end
fprintf("[V26-200m] cfg%d Stage B outer Kp/vz-limit PASS -> Ki polish, then optional Kp+Ki graduation polish.\n",cfgId);

% ---------------- Stage C: final Ki polish -------------------------------
[cC, MC, passC] = local_stageC_integral_polish(paths,master,checkpoint,cfgId,cB, ...
    physicalNominals,hiddenTrim,cfgRoot,parallelEnabled,opts);
finalMetrics = MC;
checkpoint.tuning_stage(cfgId+1) = "integral";
if passC
    checkpoint = local_commit_candidate(master,checkpoint,cfgId,cC,MC,physicalNominals,cfgRoot,opts);
    checkpoint.status(cfgId+1) = "verified";
    checkpoint.tuning_stage(cfgId+1) = "verified";
    passed = true; status = "verified";
    return;
end

% ---------------- Stage D: tiny Kp + Ki 2-D graduation polish ------------
% Stage C has already shown that the learned MPC/Plant and the height outer
% loop are fundamentally healthy.  If the remaining miss is small, do not
% reopen Stage A/B and do not launch another broad Bayesian search.  Search
% only a small Kp/Ki rectangle around the Stage-C candidate that is closest
% to the FORMAL gate.
if local_stageD_is_nearpass(MC,opts)
    fprintf("[V26-200m] cfg%d Stage C near formal graduation (gateRatio=%.4f). ", ...
        cfgId,local_formal_gate_ratio(MC,opts));
    fprintf("Starting narrow Kp+Ki 2-D polish.\n");
    [cD,MD,passD] = local_stageD_kp_ki_polish(paths,master,checkpoint,cfgId,cC,MC, ...
        physicalNominals,hiddenTrim,cfgRoot,parallelEnabled,opts);
    finalMetrics = MD;
    checkpoint.tuning_stage(cfgId+1) = "kpki";
    if passD
        checkpoint = local_commit_candidate(master,checkpoint,cfgId,cD,MD,physicalNominals,cfgRoot,opts);
        checkpoint.status(cfgId+1) = "verified";
        checkpoint.tuning_stage(cfgId+1) = "verified";
        passed = true; status = "verified";
        fprintf("[V26-200m] cfg%d Stage D Kp+Ki polish FORMAL PASS.\n",cfgId);
        return;
    end

    % ---------------- Stage E: adaptive GP Bayesian learning of Kp/Ki ------
    % Stage D was only a fixed grid.  When that grid does not graduate, v27
    % learns a Gaussian-process surrogate from ALL prior Stage-C/Stage-D/
    % Stage-E full certifications and lets bayesopt choose the next Kp/Ki
    % locations adaptively.  Only Kp/Ki move; the learned MPC, Plant, Np/Nc,
    % weights, Authority and vz limit remain frozen.
    fprintf("[V28-200m] cfg%d Stage D did not formally pass. Starting adaptive GP learning of Kp/Ki.\n",cfgId);
    [cE,ME,passE] = local_stageE_adaptive_kpki(paths,master,checkpoint,cfgId,cD,MD, ...
        cB,physicalNominals,hiddenTrim,cfgRoot,parallelEnabled,opts);
    finalMetrics = ME;
    checkpoint.tuning_stage(cfgId+1) = "adaptive_kpki";
    if passE
        checkpoint = local_commit_candidate(master,checkpoint,cfgId,cE,ME,physicalNominals,cfgRoot,opts);
        checkpoint.status(cfgId+1) = "verified";
        checkpoint.tuning_stage(cfgId+1) = "verified";
        passed = true; status = "verified";
        fprintf("[V28-200m] cfg%d adaptive Kp/Ki Bayesian learning FORMAL PASS.\n",cfgId);
        return;
    end

    checkpoint.status(cfgId+1) = "adaptive_kpki_failed";
    status = "adaptive_kpki_failed";
    checkpoint = local_commit_candidate(master,checkpoint,cfgId,cE,ME,physicalNominals,cfgRoot,opts);
    return;
end

% Even if Stage C is outside the small fixed-grid trigger, v27 can still
% learn Kp/Ki adaptively when the controller is healthy and there is no hard
% failure.  This avoids getting permanently stuck on an arbitrary grid gate.
if ~logical(MC.hard_fail(1)) && local_formal_gate_ratio(MC,opts) <= double(opts.StageEAbsoluteTriggerGateRatio)
    fprintf("[V27-200m] cfg%d skipping fixed Stage D; starting adaptive GP Kp/Ki learning from Stage C.\n",cfgId);
    [cE,ME,passE] = local_stageE_adaptive_kpki(paths,master,checkpoint,cfgId,cC,MC, ...
        cB,physicalNominals,hiddenTrim,cfgRoot,parallelEnabled,opts);
    finalMetrics = ME;
    checkpoint.tuning_stage(cfgId+1) = "adaptive_kpki";
    if passE
        checkpoint = local_commit_candidate(master,checkpoint,cfgId,cE,ME,physicalNominals,cfgRoot,opts);
        checkpoint.status(cfgId+1) = "verified";
        checkpoint.tuning_stage(cfgId+1) = "verified";
        passed = true; status = "verified";
        return;
    end
    checkpoint.status(cfgId+1) = "adaptive_kpki_failed";
    status = "adaptive_kpki_failed";
    checkpoint = local_commit_candidate(master,checkpoint,cfgId,cE,ME,physicalNominals,cfgRoot,opts);
    return;
end

checkpoint.status(cfgId+1) = "integral_failed";
status = "integral_failed";
checkpoint = local_commit_candidate(master,checkpoint,cfgId,cC,MC,physicalNominals,cfgRoot,opts);
end

function [bestC,M,pass] = local_stageA_search(paths,master,checkpoint,cfgId, ...
    physicalNominals,hiddenTrim,cfgRoot,parallelEnabled,opts)
stageRoot = fullfile(cfgRoot,"stageA_inner");
if ~isfolder(stageRoot), mkdir(stageRoot); end
summaryFile = fullfile(stageRoot,"stageA_validation_summary.csv");
candidateFile = fullfile(stageRoot,"stageA_best_candidate.csv");
if isfile(summaryFile) && isfile(candidateFile)
    try
        M = readtable(summaryFile);
        if ismember("inner_pass",string(M.Properties.VariableNames)) && logical(M.inner_pass(1))
            C = readtable(candidateFile);
            bestC = local_candidate_from_history_row(C(1,:),opts);
            pass = true;
            fprintf("[V21-200m] cfg%d Stage A previously PASS -> reuse, no new inner Bayesopt.\n",cfgId);
            return;
        end
    catch
    end
end
historyCsv = fullfile(stageRoot,"optimization_history.csv");
H = local_consolidate_stage_history(stageRoot,historyCsv,opts);

ctx = local_stage_context(paths,master,checkpoint,cfgId,physicalNominals,hiddenTrim,stageRoot,opts);
ctx.stage_mode = "inner";
ctx.base_candidate = [];
vars = local_stageA_variables();
[initialX,initialObjective] = local_stage_prior(H, ...
    {'Np','Nc','Wh','Wvz','Wq','RateScale','Authority'},opts.ResumePriorPointCount);
newEval = local_cfg_value(opts.StageAAdditionalEvaluationsByConfig,cfgId,12);
bo = local_run_stage_bayesopt(@(x)local_stage_objective(x,ctx),vars,initialX,initialObjective, ...
    newEval,parallelEnabled,opts);
save(fullfile(stageRoot,"bayesopt_result.mat"),"bo","opts");
H = local_consolidate_stage_history(stageRoot,historyCsv,opts);
if isempty(H), error("cfg%d Stage A produced no valid candidate.",cfgId); end
bestC = local_candidate_from_history_row(H(1,:),opts);
bankMat = fullfile(stageRoot,"stageA_validation_bank.mat");
local_build_combined_bank(master,checkpoint,cfgId,bestC,physicalNominals,bankMat,opts);
M = local_run_certification(paths,master,checkpoint,cfgId,bestC,physicalNominals,hiddenTrim, ...
    bankMat,stageRoot,"stageA_validation",opts.StageAValidationWindowS,opts);
pass = local_inner_gate(M,opts);
M = addvars(M,logical(pass),'After','formal_pass','NewVariableNames','inner_pass');
writetable(M,fullfile(stageRoot,"stageA_validation_summary.csv"));
writetable(local_candidate_table(cfgId,bestC,double(H.objective(1))), ...
    fullfile(stageRoot,"stageA_best_candidate.csv"));
end

function [bestC,M,formalPass,outerPass] = local_stageB_search(paths,master,checkpoint,cfgId,baseC, ...
    physicalNominals,hiddenTrim,cfgRoot,parallelEnabled,opts)
stageRoot = fullfile(cfgRoot,"stageB_height_outer");
if ~isfolder(stageRoot), mkdir(stageRoot); end
summaryFile = fullfile(stageRoot,"stageB_validation_summary.csv");
candidateFile = fullfile(stageRoot,"stageB_best_candidate.csv");
if isfile(summaryFile) && isfile(candidateFile)
    try
        M = readtable(summaryFile);
        if ismember("outer_pass",string(M.Properties.VariableNames)) && logical(M.outer_pass(1))
            C = readtable(candidateFile);
            bestC = local_candidate_from_history_row(C(1,:),opts);
            formalPass = logical(M.formal_pass(1)) && ~logical(M.hard_fail(1));
            outerPass = true;
            fprintf("[V21-200m] cfg%d Stage B previously PASS -> reuse, no new Kp search.\n",cfgId);
            return;
        end
    catch
    end
end
historyCsv = fullfile(stageRoot,"optimization_history.csv");
H = local_consolidate_stage_history(stageRoot,historyCsv,opts);

ctx = local_stage_context(paths,master,checkpoint,cfgId,physicalNominals,hiddenTrim,stageRoot,opts);
ctx.stage_mode = "outer";
ctx.base_candidate = baseC;
vars = local_stageB_variables();
[initialX,initialObjective] = local_stage_prior(H,{'HeightToVzGain','HeightVzLimit'},opts.ResumePriorPointCount);
newEval = local_cfg_value(opts.StageBAdditionalEvaluationsByConfig,cfgId,9);
bo = local_run_stage_bayesopt(@(x)local_stage_objective(x,ctx),vars,initialX,initialObjective, ...
    newEval,parallelEnabled,opts);
save(fullfile(stageRoot,"bayesopt_result.mat"),"bo","opts");
H = local_consolidate_stage_history(stageRoot,historyCsv,opts);
if isempty(H), error("cfg%d Stage B produced no valid candidate.",cfgId); end
bestC = local_candidate_from_history_row(H(1,:),opts);
bankMat = fullfile(stageRoot,"stageB_validation_bank.mat");
local_build_combined_bank(master,checkpoint,cfgId,bestC,physicalNominals,bankMat,opts);
M = local_run_certification(paths,master,checkpoint,cfgId,bestC,physicalNominals,hiddenTrim, ...
    bankMat,stageRoot,"stageB_validation",opts.FinalWindowS,opts);
formalPass = logical(M.formal_pass(1)) && ~logical(M.hard_fail(1));
outerPass = local_outer_gate(M,opts);
M = addvars(M,logical(outerPass),'After','formal_pass','NewVariableNames','outer_pass');
writetable(M,fullfile(stageRoot,"stageB_validation_summary.csv"));
writetable(local_candidate_table(cfgId,bestC,double(H.objective(1))), ...
    fullfile(stageRoot,"stageB_best_candidate.csv"));
end

function [bestC,bestM,passed] = local_stageC_integral_polish(paths,master,checkpoint,cfgId,baseC, ...
    physicalNominals,hiddenTrim,cfgRoot,parallelEnabled,opts)
stageRoot = fullfile(cfgRoot,"stageC_integral_polish");
if ~isfolder(stageRoot), mkdir(stageRoot); end
gains = unique(double(opts.StageCIntegralGains(:)),"stable");
gains = gains(isfinite(gains) & gains >= 0);
records = local_load_stageC_records(stageRoot,opts);
tested = [];
if ~isempty(records), tested = double(records.HeightIntegralGain(:)); end
todo = gains(arrayfun(@(g)~any(abs(tested-g)<1e-10),gains));
ctx = local_stage_context(paths,master,checkpoint,cfgId,physicalNominals,hiddenTrim,stageRoot,opts);

if ~isempty(todo)
    if logical(parallelEnabled) && numel(todo)>1
        parfor i=1:numel(todo)
            local_stageC_one(todo(i),baseC,ctx);
        end
    else
        for i=1:numel(todo), local_stageC_one(todo(i),baseC,ctx); end
    end
end
records = local_load_stageC_records(stageRoot,opts);
if isempty(records), error("cfg%d Stage C produced no certification record.",cfgId); end
[r,bestM,passed,bestGateRatio,records] = local_select_formal_nearest_record(records,opts);
bestC = baseC;
bestC.HeightIntegralGain = double(r.HeightIntegralGain);
fprintf("[V26-200m] cfg%d Stage C selected Ki=%.7f formal=%d gateRatio=%.4f rank=%.4g\n", ...
    cfgId,bestC.HeightIntegralGain,passed,bestGateRatio,double(r.rank_score));

% v24 cfg3+ near-pass polish:
% The v23 cfg3 coarse Ki sweep found Ki=0.0015 with every formal metric
% passing except steady altitude RMS=1.053 m.  Do not reopen Stage B or a
% broad Bayesopt.  Instead make one narrow deterministic Ki sweep around
% the best certified coarse point.  This is generic for later cfg3/cfg4
% near-passes too.
if ~passed && local_stageC_is_nearpass(bestM,opts)
    bestKi = double(bestC.HeightIntegralGain);
    fineFactors = double(opts.StageCFineKiFactors(:));
    fineGains = bestKi .* fineFactors;
    fineGains = fineGains(isfinite(fineGains) & fineGains >= 0 & ...
        fineGains <= double(opts.StageCFineKiMax));
    fineGains = unique(round(fineGains,7),"stable");
    tested = double(records.HeightIntegralGain(:));
    fineTodo = fineGains(arrayfun(@(g)~any(abs(tested-g)<1e-10),fineGains));

    if ~isempty(fineTodo)
        fprintf("[V25-200m] cfg%d Stage C near-pass: hRMS=%.4f m at Ki=%.6f -> fine Ki sweep (%d NEW points).\n", ...
            cfgId,double(bestM.steady_h_rms_m(1)),bestKi,numel(fineTodo));
        if logical(parallelEnabled) && numel(fineTodo)>1
            parfor i=1:numel(fineTodo)
                local_stageC_one(fineTodo(i),baseC,ctx);
            end
        else
            for i=1:numel(fineTodo), local_stageC_one(fineTodo(i),baseC,ctx); end
        end

        records = local_load_stageC_records(stageRoot,opts);
        [r,bestM,passed,bestGateRatio,records] = local_select_formal_nearest_record(records,opts);
        bestC = baseC;
        bestC.HeightIntegralGain = double(r.HeightIntegralGain);
        fprintf("[V26-200m] cfg%d Stage C after Ki sweep: Ki=%.7f formal=%d gateRatio=%.4f rank=%.4g\n", ...
            cfgId,bestC.HeightIntegralGain,passed,bestGateRatio,double(r.rank_score));
    end
end

writetable(records,fullfile(stageRoot,"stageC_history.csv"));
writetable(local_candidate_table(cfgId,bestC,double(r.rank_score)), ...
    fullfile(stageRoot,"stageC_best_candidate.csv"));
writetable(bestM,fullfile(stageRoot,"stageC_best_validation_summary.csv"));
end

function tf = local_stageC_is_nearpass(M,opts)
% Fine Ki is only appropriate when the inner/outer controller is already
% healthy and the remaining miss is a small altitude-RMS/static-error miss.
try
    tf = ~logical(M.hard_fail(1)) && ...
        double(M.steady_h_rms_m(1)) <= double(opts.StageCFineNearPassAltitudeRmsM) && ...
        double(M.steady_h_max_abs_m(1)) <= double(opts.PassAltitudeMaxM) && ...
        abs(double(M.steady_h_drift_m(1))) <= double(opts.PassAltitudeDriftM) && ...
        double(M.steady_Va_rms_mps(1)) <= double(opts.PassAirspeedRmsMps) && ...
        double(M.steady_vz_rms_mps(1)) <= double(opts.PassVzRmsMps) && ...
        double(M.steady_q_rms_dps(1)) <= double(opts.PassQRmsDps) && ...
        abs(double(M.tail_h_error_m(1))) <= double(opts.PassTailAltitudeErrorM) && ...
        abs(double(M.tail_vz_mps(1))) <= double(opts.PassTailVzMps);
catch
    tf = false;
end
end

function local_stageC_one(Ki,baseC,ctx)
local_prepare_worker_filegen(ctx);
c = baseC;
c.HeightIntegralGain = double(Ki);
tag = sprintf("Ki_%0.6f",double(Ki));
evalRoot = fullfile(ctx.cfgRoot,tag);
if ~isfolder(evalRoot), mkdir(evalRoot); end
metricsFile = fullfile(evalRoot,"certification_summary.csv");
recordFile = fullfile(evalRoot,"stageC_record.csv");
if isfile(recordFile) && isfile(metricsFile), return; end
bankMat = fullfile(evalRoot,"bank.mat");
try
    local_build_combined_bank(ctx.master,ctx.checkpoint,ctx.cfgId,c,ctx.physicalNominals,bankMat,ctx.opts);
    M = local_run_certification(ctx.paths,ctx.master,ctx.checkpoint,ctx.cfgId,c, ...
        ctx.physicalNominals,ctx.hiddenTrim,bankMat,evalRoot,"integral_cert",ctx.opts.FinalWindowS,ctx.opts);
    score = local_score_from_metrics(M,ctx.opts);
    writetable(M,metricsFile);
catch ME
    score = double(ctx.opts.HardFailScore);
    M = local_empty_cert_like_metrics(ctx.cfgId);
    writetable(M,metricsFile);
    fid=fopen(fullfile(evalRoot,"error.txt"),"w");
    if fid>=0, fprintf(fid,"%s\n",getReport(ME,"extended","hyperlinks","off")); fclose(fid); end
end
row = table(double(Ki),double(score),string(metricsFile), ...
    'VariableNames',{'HeightIntegralGain','rank_score','metrics_file'});
writetable(row,recordFile);
end


function [r,M,passed,bestRatio,R] = local_select_formal_nearest_record(R,opts)
% v26: choose by graduation distance, not by legacy rank alone.
% A formal PASS always wins.  Otherwise minimize the maximum normalized
% formal-gate violation.  rank_score is only the tie-breaker.
if isempty(R)
    error("Stage certification record table is empty.");
end
n = height(R);
gate = inf(n,1);
isPass = false(n,1);
for ii = 1:n
    try
        Mi = readtable(string(R.metrics_file(ii)),'VariableNamingRule','preserve');
        gate(ii) = local_formal_gate_ratio(Mi,opts);
        isPass(ii) = logical(Mi.formal_pass(1)) && ~logical(Mi.hard_fail(1));
    catch
        gate(ii) = inf;
        isPass(ii) = false;
    end
end
if ~ismember("gate_ratio",string(R.Properties.VariableNames))
    R = addvars(R,gate,'NewVariableNames','gate_ratio');
else
    R.gate_ratio = gate;
end
if ~ismember("formal_record_pass",string(R.Properties.VariableNames))
    R = addvars(R,isPass,'NewVariableNames','formal_record_pass');
else
    R.formal_record_pass = isPass;
end

if any(isPass)
    idx = find(isPass);
    [~,ord] = sortrows([gate(idx), double(R.rank_score(idx))],[1 2]);
    pick = idx(ord(1));
else
    [~,ord] = sortrows([gate, double(R.rank_score)],[1 2]);
    pick = ord(1);
end
r = R(pick,:);
M = readtable(string(r.metrics_file),'VariableNamingRule','preserve');
passed = logical(M.formal_pass(1)) && ~logical(M.hard_fail(1));
bestRatio = gate(pick);
R = sortrows(R,{'formal_record_pass','gate_ratio','rank_score'}, ...
    {'descend','ascend','ascend'});
end

function ratio = local_formal_gate_ratio(M,opts)
% Max normalized distance to the unchanged formal certification gate.
% ratio <= 1 means every scalar formal metric is inside its limit.
try
    if isempty(M) || height(M)<1 || logical(M.hard_fail(1))
        ratio = inf;
        return;
    end
    vals = [ ...
        double(M.steady_h_rms_m(1)) / max(double(opts.PassAltitudeRmsM),eps), ...
        double(M.steady_h_max_abs_m(1)) / max(double(opts.PassAltitudeMaxM),eps), ...
        abs(double(M.steady_h_drift_m(1))) / max(double(opts.PassAltitudeDriftM),eps), ...
        double(M.steady_Va_rms_mps(1)) / max(double(opts.PassAirspeedRmsMps),eps), ...
        double(M.steady_vz_rms_mps(1)) / max(double(opts.PassVzRmsMps),eps), ...
        double(M.steady_q_rms_dps(1)) / max(double(opts.PassQRmsDps),eps), ...
        abs(double(M.tail_h_error_m(1))) / max(double(opts.PassTailAltitudeErrorM),eps), ...
        abs(double(M.tail_vz_mps(1))) / max(double(opts.PassTailVzMps),eps) ...
        ];
    ratio = max(vals);
    if ~isfinite(ratio), ratio = inf; end
catch
    ratio = inf;
end
end

function tf = local_stageD_is_nearpass(M,opts)
try
    tf = ~logical(M.hard_fail(1)) && ...
        local_formal_gate_ratio(M,opts) <= double(opts.StageDTriggerGateRatio);
catch
    tf = false;
end
end

function [bestC,bestM,passed] = local_stageD_kp_ki_polish(paths,master,checkpoint,cfgId,centerC,centerM, ...
    physicalNominals,hiddenTrim,cfgRoot,parallelEnabled,opts)
stageRoot = fullfile(cfgRoot,"stageD_kp_ki_polish");
if ~isfolder(stageRoot), mkdir(stageRoot); end

centerKp = double(centerC.HeightToVzGain);
centerKi = double(centerC.HeightIntegralGain);

kpVals = centerKp .* double(opts.StageDKpFactors(:));
kpVals = kpVals(isfinite(kpVals) & kpVals >= double(opts.StageDMinKp) & ...
    kpVals <= double(opts.StageDMaxKp));
kpVals = unique(round(kpVals,7),"stable");

if centerKi > 1e-10
    kiVals = centerKi .* double(opts.StageDKiFactors(:));
else
    kiVals = double(opts.StageDKiFallback(:));
end
kiVals = kiVals(isfinite(kiVals) & kiVals >= 0 & kiVals <= double(opts.StageDMaxKi));
kiVals = unique(round(kiVals,7),"stable");

% Full deterministic 2-D grid.  StageDKpFactors intentionally excludes 1.0,
% because the center-Kp Ki line was already certified in Stage C.
pairs = zeros(numel(kpVals)*numel(kiVals),2);
kk = 0;
for i = 1:numel(kpVals)
    for j = 1:numel(kiVals)
        kk = kk + 1;
        pairs(kk,:) = [kpVals(i),kiVals(j)];
    end
end

R0 = local_load_stageD_records(stageRoot,opts);
todoMask = true(size(pairs,1),1);
if ~isempty(R0)
    for i=1:size(pairs,1)
        already = abs(double(R0.HeightToVzGain)-pairs(i,1))<1e-10 & ...
                  abs(double(R0.HeightIntegralGain)-pairs(i,2))<1e-10;
        if any(already), todoMask(i)=false; end
    end
end
todo = pairs(todoMask,:);

fprintf("[V26-200m] cfg%d Stage D center Kp=%.7f Ki=%.7f; grid=%d, NEW=%d, workers<=%d\\n", ...
    cfgId,centerKp,centerKi,size(pairs,1),size(todo,1),double(opts.ParallelWorkers));

ctx = local_stage_context(paths,master,checkpoint,cfgId,physicalNominals,hiddenTrim,stageRoot,opts);
if ~isempty(todo)
    if logical(parallelEnabled) && size(todo,1)>1
        parfor i=1:size(todo,1)
            local_stageD_one(todo(i,1),todo(i,2),centerC,ctx);
        end
    else
        for i=1:size(todo,1)
            local_stageD_one(todo(i,1),todo(i,2),centerC,ctx);
        end
    end
end

R = local_load_stageD_records(stageRoot,opts);
if isempty(R)
    % Never return something worse merely because Stage-D I/O failed.
    bestC = centerC;
    bestM = centerM;
    passed = logical(centerM.formal_pass(1)) && ~logical(centerM.hard_fail(1));
    return;
end

% Prefer a formal pass. Otherwise choose the smallest normalized gate ratio.
isPass = logical(R.formal_record_pass);
if any(isPass)
    idx = find(isPass);
    [~,ord] = sortrows([double(R.gate_ratio(idx)),double(R.rank_score(idx))],[1 2]);
    pick = idx(ord(1));
else
    [~,ord] = sortrows([double(R.gate_ratio),double(R.rank_score)],[1 2]);
    pick = ord(1);
end
r = R(pick,:);
candM = readtable(string(r.metrics_file),'VariableNamingRule','preserve');

% Compare against the Stage-C center.  If the 2-D grid failed to improve
% graduation distance, preserve the existing center instead.
centerRatio = local_formal_gate_ratio(centerM,opts);
candRatio = local_formal_gate_ratio(candM,opts);
if ~logical(candM.formal_pass(1)) && candRatio >= centerRatio
    bestC = centerC;
    bestM = centerM;
    passed = logical(centerM.formal_pass(1)) && ~logical(centerM.hard_fail(1));
else
    bestC = centerC;
    bestC.HeightToVzGain = double(r.HeightToVzGain);
    bestC.HeightIntegralGain = double(r.HeightIntegralGain);
    bestM = candM;
    passed = logical(bestM.formal_pass(1)) && ~logical(bestM.hard_fail(1));
end

writetable(R,fullfile(stageRoot,"stageD_history.csv"));
writetable(local_candidate_table(cfgId,bestC,local_score_from_metrics(bestM,opts)), ...
    fullfile(stageRoot,"stageD_best_candidate.csv"));
writetable(bestM,fullfile(stageRoot,"stageD_best_validation_summary.csv"));

fprintf("[V26-200m] cfg%d Stage D result: Kp=%.7f Ki=%.7f gateRatio=%.4f formal=%d hRMS=%.4f hMax=%.4f\\n", ...
    cfgId,double(bestC.HeightToVzGain),double(bestC.HeightIntegralGain), ...
    local_formal_gate_ratio(bestM,opts),passed,double(bestM.steady_h_rms_m(1)), ...
    double(bestM.steady_h_max_abs_m(1)));
end

function local_stageD_one(Kp,Ki,baseC,ctx)
local_prepare_worker_filegen(ctx);
c = baseC;
c.HeightToVzGain = double(Kp);
c.HeightIntegralGain = double(Ki);
tag = sprintf("Kp_%0.7f_Ki_%0.7f",double(Kp),double(Ki));
evalRoot = fullfile(ctx.cfgRoot,tag);
if ~isfolder(evalRoot), mkdir(evalRoot); end
metricsFile = fullfile(evalRoot,"certification_summary.csv");
recordFile = fullfile(evalRoot,"stageD_record.csv");
if isfile(recordFile) && isfile(metricsFile), return; end

bankMat = fullfile(evalRoot,"bank.mat");
try
    local_build_combined_bank(ctx.master,ctx.checkpoint,ctx.cfgId,c,ctx.physicalNominals,bankMat,ctx.opts);
    M = local_run_certification(ctx.paths,ctx.master,ctx.checkpoint,ctx.cfgId,c, ...
        ctx.physicalNominals,ctx.hiddenTrim,bankMat,evalRoot,"kpki_cert",ctx.opts.FinalWindowS,ctx.opts);
    score = local_score_from_metrics(M,ctx.opts);
    gateRatio = local_formal_gate_ratio(M,ctx.opts);
    formalPass = logical(M.formal_pass(1)) && ~logical(M.hard_fail(1));
    writetable(M,metricsFile);
catch ME
    score = double(ctx.opts.HardFailScore);
    gateRatio = inf;
    formalPass = false;
    M = local_empty_cert_like_metrics(ctx.cfgId);
    writetable(M,metricsFile);
    fid=fopen(fullfile(evalRoot,"error.txt"),"w");
    if fid>=0
        fprintf(fid,"%s\\n",getReport(ME,"extended","hyperlinks","off"));
        fclose(fid);
    end
end

row = table(double(Kp),double(Ki),double(score),double(gateRatio),logical(formalPass), ...
    string(metricsFile),'VariableNames', ...
    {'HeightToVzGain','HeightIntegralGain','rank_score','gate_ratio','formal_record_pass','metrics_file'});
writetable(row,recordFile);
fprintf("[V26-200m] cfg%d StageD Kp=%.7f Ki=%.7f gate=%.4f obj=%.4g formal=%d\\n", ...
    ctx.cfgId,double(Kp),double(Ki),double(gateRatio),double(score),logical(formalPass));
end

function R = local_load_stageD_records(stageRoot,opts)
kp=[]; ki=[]; score=[]; gate=[]; pass=[]; metrics=strings(0,1);
if ~isfolder(stageRoot)
    R=table();
    return;
end
dirs=dir(fullfile(stageRoot,"Kp_*_Ki_*"));
dirs=dirs([dirs.isdir]);
for i=1:numel(dirs)
    tok=regexp(dirs(i).name,'^Kp_([-+0-9.eE]+)_Ki_([-+0-9.eE]+)$','tokens','once');
    if isempty(tok), continue; end
    thisKp=str2double(tok{1});
    thisKi=str2double(tok{2});
    if ~isfinite(thisKp) || ~isfinite(thisKi), continue; end

    evalRoot=fullfile(dirs(i).folder,dirs(i).name);
    mf=fullfile(evalRoot,"certification_summary.csv");
    if ~isfile(mf), continue; end
    try
        M=readtable(mf,'VariableNamingRule','preserve');
        thisScore=local_score_from_metrics(M,opts);
        thisGate=local_formal_gate_ratio(M,opts);
        thisPass=logical(M.formal_pass(1)) && ~logical(M.hard_fail(1));
    catch
        thisScore=double(opts.HardFailScore);
        thisGate=inf;
        thisPass=false;
    end
    kp(end+1,1)=thisKp; %#ok<AGROW>
    ki(end+1,1)=thisKi; %#ok<AGROW>
    score(end+1,1)=thisScore; %#ok<AGROW>
    gate(end+1,1)=thisGate; %#ok<AGROW>
    pass(end+1,1)=thisPass; %#ok<AGROW>
    metrics(end+1,1)=string(mf); %#ok<AGROW>
end
if isempty(kp)
    R=table();
    return;
end
R=table(double(kp),double(ki),double(score),double(gate),logical(pass),string(metrics), ...
    'VariableNames',{'HeightToVzGain','HeightIntegralGain','rank_score','gate_ratio','formal_record_pass','metrics_file'});
R=sortrows(R,{'formal_record_pass','gate_ratio','rank_score'}, {'descend','ascend','ascend'});
end


function [bestC,bestM,passed] = local_stageE_adaptive_kpki(paths,master,checkpoint,cfgId,seedC,seedM, ...
    stageBBase,physicalNominals,hiddenTrim,cfgRoot,parallelEnabled,opts)
% v27: sample-efficient self-learning Kp/Ki optimizer.
%
% The expensive simulator is treated as a black-box function:
%   (Kp, Ki) -> normalized formal certification loss.
%
% A Gaussian-process Bayesian optimizer is warm-started from every compatible
% Stage-C/D/E full certification already on disk.  Therefore every rerun
% improves the surrogate instead of repeating a fixed grid.

stageRoot = fullfile(cfgRoot,"stageE_adaptive_kpki");
if ~isfolder(stageRoot), mkdir(stageRoot); end

% Use the candidate currently closest to graduation as the center.  Keep all
% non-Kp/Ki controller settings frozen to the Stage-B controller.
centerRatio = local_formal_gate_ratio(seedM,opts);
centerKp = double(seedC.HeightToVzGain);
centerKi = max(double(seedC.HeightIntegralGain),double(opts.StageEMinPositiveKi));

% Gather prior data from Stage C, Stage D and any previous Stage-E run.
P = local_collect_kpki_learning_history(cfgRoot,stageBBase,opts);
infraCount = local_count_stageE_infrastructure_failures(fullfile(cfgRoot,"stageE_adaptive_kpki"));
if infraCount > 0
    fprintf("[V28-200m] cfg%d ignoring %d Stage-E infrastructure-failure records; they are eligible for re-run.\n", ...
        cfgId,infraCount);
end
if isempty(P)
    % At minimum include the fully-certified seed.
    seedObj = local_formal_learning_objective(seedM,opts);
    P = table(centerKp,centerKi,seedObj,centerRatio,logical(seedM.formal_pass(1)), ...
        "seed",string(""), ...
        'VariableNames',{'HeightToVzGain','HeightIntegralGain','learning_objective', ...
        'gate_ratio','formal_pass','source','metrics_file'});
end

% If history contains a better formal-nearest point than the incoming seed,
% center the trust region there.
finiteP = P(isfinite(P.learning_objective) & isfinite(P.gate_ratio),:);
if ~isempty(finiteP)
    finiteP = sortrows(finiteP,{'formal_pass','gate_ratio','learning_objective'}, ...
        {'descend','ascend','ascend'});
    centerKp = double(finiteP.HeightToVzGain(1));
    centerKi = max(double(finiteP.HeightIntegralGain(1)),double(opts.StageEMinPositiveKi));
    centerRatio = double(finiteP.gate_ratio(1));
end

% Adaptive trust region.  It is much wider than the v26 micro-grid, but still
% small enough that the same linear Plant/MPC operating region remains valid.
kpLow = max(double(opts.StageEMinKp), centerKp*double(opts.StageEKpLowerFactor));
kpHigh = min(double(opts.StageEMaxKp), centerKp*double(opts.StageEKpUpperFactor));
kiLow = max(double(opts.StageEMinPositiveKi), centerKi*double(opts.StageEKiLowerFactor));
kiHigh = min(double(opts.StageEMaxKi), max(centerKi*double(opts.StageEKiUpperFactor), ...
    centerKi + double(opts.StageEMinKiSpan)));
if kpHigh <= kpLow
    kpLow = double(opts.StageEMinKp);
    kpHigh = double(opts.StageEMaxKp);
end
if kiHigh <= kiLow
    kiLow = double(opts.StageEMinPositiveKi);
    kiHigh = double(opts.StageEMaxKi);
end

vars = [ ...
    optimizableVariable("HeightToVzGain",[kpLow kpHigh],"Transform","log")
    optimizableVariable("HeightIntegralGain",[kiLow kiHigh],"Transform","log")
    ];

% Only feed priors inside the current trust region.  Initial observations are
% true prior evaluations, so bayesopt must not call the simulator for them.
inside = P.HeightToVzGain >= kpLow & P.HeightToVzGain <= kpHigh & ...
         P.HeightIntegralGain >= kiLow & P.HeightIntegralGain <= kiHigh & ...
         isfinite(P.learning_objective);
Pin = P(inside,:);
Pin = local_deduplicate_kpki_prior(Pin,opts);

% Cap old priors by formal relevance to keep GP fitting inexpensive when this
% pipeline has accumulated hundreds of runs.
if height(Pin) > double(opts.StageEMaxPriorPoints)
    Pin = sortrows(Pin,{'formal_pass','gate_ratio','learning_objective'}, ...
        {'descend','ascend','ascend'});
    Pin = Pin(1:double(opts.StageEMaxPriorPoints),:);
end

initialX = table();
initialObjective = [];
if ~isempty(Pin)
    initialX = table(double(Pin.HeightToVzGain),double(Pin.HeightIntegralGain), ...
        'VariableNames',{'HeightToVzGain','HeightIntegralGain'});
    initialObjective = double(Pin.learning_objective);
end

ctx = local_stage_context(paths,master,checkpoint,cfgId,physicalNominals,hiddenTrim,stageRoot,opts);
ctx.base_candidate = stageBBase;
ctx.stage_mode = "adaptive_kpki";

nNew = round(double(opts.StageEAdditionalEvaluationsPerRun));
priorCount = height(initialX);
totalLimit = priorCount + nNew;

fprintf("[V28-200m] cfg%d adaptive GP Kp/Ki: center Kp=%.6f Ki=%.7f gate=%.4f\\n", ...
    cfgId,centerKp,centerKi,centerRatio);
fprintf("[V28-200m] cfg%d learned priors=%d, NEW budget=%d, Kp=[%.5f %.5f], Ki=[%.7f %.7f]\\n", ...
    cfgId,priorCount,nNew,kpLow,kpHigh,kiLow,kiHigh);

args = { ...
    "MaxObjectiveEvaluations",totalLimit, ...
    "IsObjectiveDeterministic",true, ...
    "UseParallel",logical(parallelEnabled), ...
    "AcquisitionFunctionName","expected-improvement-plus", ...
    "Verbose",double(opts.BayesoptVerbose), ...
    "OutputFcn",@(results,state)local_stageE_stop_on_formal(results,state,opts) ...
    };
if logical(parallelEnabled)
    % Do not force all 5 workers full at every instant.  MathWorks notes that
    % an aggressive MinWorkerUtilization can inject random points merely to
    % keep workers busy.  v27 favors GP-selected points over utilization.
    workers = local_parallel_worker_count(true,opts);
    minUtil = min(workers,max(1,round(double(opts.StageEMinWorkerUtilization))));
    args = [args,{ ...
        "MinWorkerUtilization",minUtil, ...
        "ParallelMethod","clipped-model-prediction"}]; %#ok<AGROW>
end
if ~isempty(initialX)
    args = [args,{"InitialX",initialX,"InitialObjective",initialObjective}]; %#ok<AGROW>
end

objective = @(x)local_stageE_objective(x,ctx);
bo = bayesopt(objective,vars,args{:});
save(fullfile(stageRoot,"bayesopt_result_latest.mat"),"bo","opts","kpLow","kpHigh","kiLow","kiHigh");

% Reload everything from disk, including evaluations that completed in
% parallel around the same time as the early-stop request.
P = local_collect_kpki_learning_history(cfgRoot,stageBBase,opts);
PE = P(P.source=="stageE",:);
if isempty(PE)
    % Preserve the best incoming candidate if no Stage-E simulation completed.
    bestC = seedC;
    bestM = seedM;
    passed = logical(seedM.formal_pass(1)) && ~logical(seedM.hard_fail(1));
    return;
end

PE = sortrows(PE,{'formal_pass','gate_ratio','learning_objective'}, ...
    {'descend','ascend','ascend'});
row = PE(1,:);
bestM = readtable(string(row.metrics_file),'VariableNamingRule','preserve');
bestC = stageBBase;
bestC.HeightToVzGain = double(row.HeightToVzGain);
bestC.HeightIntegralGain = double(row.HeightIntegralGain);
passed = logical(bestM.formal_pass(1)) && ~logical(bestM.hard_fail(1));

% Compare with the incoming seed so a failed learning round never overwrites
% a better controller with a worse one.
if ~passed && local_formal_gate_ratio(bestM,opts) >= local_formal_gate_ratio(seedM,opts)
    bestC = seedC;
    bestM = seedM;
    passed = logical(seedM.formal_pass(1)) && ~logical(seedM.hard_fail(1));
end

writetable(P,fullfile(stageRoot,"adaptive_learning_history.csv"));
writetable(local_candidate_table(cfgId,bestC,local_formal_learning_objective(bestM,opts)), ...
    fullfile(stageRoot,"adaptive_best_candidate.csv"));
writetable(bestM,fullfile(stageRoot,"adaptive_best_validation_summary.csv"));

fprintf("[V28-200m] cfg%d adaptive result Kp=%.7f Ki=%.7f gate=%.4f formal=%d hRMS=%.4f hMax=%.4f\\n", ...
    cfgId,double(bestC.HeightToVzGain),double(bestC.HeightIntegralGain), ...
    local_formal_gate_ratio(bestM,opts),passed,double(bestM.steady_h_rms_m(1)), ...
    double(bestM.steady_h_max_abs_m(1)));
end

function score = local_stageE_objective(x,ctx)
local_prepare_worker_filegen(ctx);
Kp = double(x.HeightToVzGain);
Ki = double(x.HeightIntegralGain);
c = ctx.base_candidate;
c.HeightToVzGain = Kp;
c.HeightIntegralGain = Ki;

% Deterministic folder name gives crash-safe resume and avoids exact repeats.
tag = sprintf("Kp_%0.8f_Ki_%0.9f",Kp,Ki);
evalRoot = fullfile(ctx.cfgRoot,tag);
if ~isfolder(evalRoot), mkdir(evalRoot); end
metricsFile = fullfile(evalRoot,"certification_summary.csv");
recordFile = fullfile(evalRoot,"stageE_record.csv");

if isfile(metricsFile) && ~local_stageE_infrastructure_failure(evalRoot)
    try
        M = readtable(metricsFile,'VariableNamingRule','preserve');
        score = local_formal_learning_objective(M,ctx.opts);
        return;
    catch
    end
end

% v28: v27 placeholder certifications caused by MissingLog are not valid
% controller observations. Re-run the same Kp/Ki after fixing logging.
if local_stageE_infrastructure_failure(evalRoot)
    try, delete(metricsFile); catch, end
    try, delete(recordFile); catch, end
end

bankMat = fullfile(evalRoot,"bank.mat");
try
    local_build_combined_bank(ctx.master,ctx.checkpoint,ctx.cfgId,c,ctx.physicalNominals,bankMat,ctx.opts);
    M = local_run_certification(ctx.paths,ctx.master,ctx.checkpoint,ctx.cfgId,c, ...
        ctx.physicalNominals,ctx.hiddenTrim,bankMat,evalRoot,"adaptive_kpki_cert", ...
        ctx.opts.FinalWindowS,ctx.opts);
    score = local_formal_learning_objective(M,ctx.opts);
    gate = local_formal_gate_ratio(M,ctx.opts);
    formal = logical(M.formal_pass(1)) && ~logical(M.hard_fail(1));
    writetable(M,metricsFile);
    try
        errFile = fullfile(evalRoot,"error.txt");
        if isfile(errFile), delete(errFile); end
    catch
    end
catch ME
    score = double(ctx.opts.HardFailScore);
    gate = inf;
    formal = false;
    M = local_empty_cert_like_metrics(ctx.cfgId);
    writetable(M,metricsFile);
    fid=fopen(fullfile(evalRoot,"error.txt"),"w");
    if fid>=0
        fprintf(fid,"%s\\n",getReport(ME,"extended","hyperlinks","off"));
        fclose(fid);
    end
end

row = table(Kp,Ki,double(score),double(gate),logical(formal),string(metricsFile), ...
    'VariableNames',{'HeightToVzGain','HeightIntegralGain','learning_objective', ...
    'gate_ratio','formal_pass','metrics_file'});
writetable(row,recordFile);
fprintf("[V28-200m] cfg%d LEARN Kp=%.7f Ki=%.8f loss=%.5f gate=%.4f formal=%d\\n", ...
    ctx.cfgId,Kp,Ki,double(score),double(gate),logical(formal));
end

function stop = local_stageE_stop_on_formal(results,state,opts)
% bayesopt calls OutputFcn on the client at the end of every iteration.
% With this objective, any formal-pass point has learning objective <= 1.
stop = false;
if strcmpi(string(state),"iteration")
    try
        if isfinite(results.MinObjective) && ...
                results.MinObjective <= double(opts.StageEFormalStopObjective)
            fprintf("[V28-200m] adaptive GP found a formal-pass objective %.6f -> stop search early.\\n", ...
                results.MinObjective);
            stop = true;
        end
    catch
    end
end
end

function obj = local_formal_learning_objective(M,opts)
% Objective aligned with graduation, not with the old hand-weighted rank.
% 1.0 is the formal boundary.  The squared excess term teaches the GP that
% points violating several gates are worse than a point missing only one.
ratio = local_formal_gate_ratio(M,opts);
if ~isfinite(ratio)
    obj = double(opts.HardFailScore);
    return;
end
try
    r = local_formal_ratio_vector(M,opts);
    excess = max(r-1.0,0.0);
    obj = ratio + double(opts.StageEMultiViolationWeight)*mean(excess.^2);
    if logical(M.formal_pass(1)) && ~logical(M.hard_fail(1))
        % Keep the optimizer strongly attracted to the interior of the valid
        % region once one valid controller has been found.
        obj = min(obj,ratio);
    end
catch
    obj = ratio;
end
if ~isfinite(obj), obj = double(opts.HardFailScore); end
end

function r = local_formal_ratio_vector(M,opts)
r = [ ...
    double(M.steady_h_rms_m(1)) / max(double(opts.PassAltitudeRmsM),eps), ...
    double(M.steady_h_max_abs_m(1)) / max(double(opts.PassAltitudeMaxM),eps), ...
    abs(double(M.steady_h_drift_m(1))) / max(double(opts.PassAltitudeDriftM),eps), ...
    double(M.steady_Va_rms_mps(1)) / max(double(opts.PassAirspeedRmsMps),eps), ...
    double(M.steady_vz_rms_mps(1)) / max(double(opts.PassVzRmsMps),eps), ...
    double(M.steady_q_rms_dps(1)) / max(double(opts.PassQRmsDps),eps), ...
    abs(double(M.tail_h_error_m(1))) / max(double(opts.PassTailAltitudeErrorM),eps), ...
    abs(double(M.tail_vz_mps(1))) / max(double(opts.PassTailVzMps),eps) ...
    ];
if logical(M.hard_fail(1)), r(:)=inf; end
end

function n = local_count_stageE_infrastructure_failures(stageRoot)
n=0;
if ~isfolder(stageRoot), return; end
dirs=dir(fullfile(stageRoot,"Kp_*_Ki_*"));
dirs=dirs([dirs.isdir]);
for k=1:numel(dirs)
    evalRoot=fullfile(dirs(k).folder,dirs(k).name);
    if local_stageE_infrastructure_failure(evalRoot), n=n+1; end
end
end

function tf = local_stageE_infrastructure_failure(evalRoot)
% True only for failures unrelated to controller quality. Such evaluations
% must not enter the GP training data and are safe to re-run.
tf=false;
errFile=fullfile(evalRoot,"error.txt");
if ~isfile(errFile), return; end
try
    txt=lower(string(fileread(errFile)));
    markers=[ ...
        "airdropx:autompc:missinglog", ...
        "airdropx:autompc:missinglogsout", ...
        "logsout does not contain altitude_m", ...
        "simulationoutput does not contain a usable logsout", ...
        "available logged signals:" ...
        ];
    for k=1:numel(markers)
        if contains(txt,markers(k)), tf=true; return; end
    end
catch
end
end

function P = local_collect_kpki_learning_history(cfgRoot,stageBBase,opts)
% Collect only certifications that share the same frozen Stage-B controller
% except for Kp/Ki.  Stage C and Stage D meet that requirement by design.
kp=[]; ki=[]; obj=[]; gate=[]; formal=[]; source=strings(0,1); metrics=strings(0,1);

% Stage C: Kp is the Stage-B value, Ki varies.
cRoot = fullfile(cfgRoot,"stageC_integral_polish");
RC = local_load_stageC_records(cRoot,opts);
for i=1:height(RC)
    try
        mf=string(RC.metrics_file(i));
        M=readtable(mf,'VariableNamingRule','preserve');
        kp(end+1,1)=double(stageBBase.HeightToVzGain); %#ok<AGROW>
        ki(end+1,1)=double(RC.HeightIntegralGain(i)); %#ok<AGROW>
        obj(end+1,1)=local_formal_learning_objective(M,opts); %#ok<AGROW>
        gate(end+1,1)=local_formal_gate_ratio(M,opts); %#ok<AGROW>
        formal(end+1,1)=logical(M.formal_pass(1)) && ~logical(M.hard_fail(1)); %#ok<AGROW>
        source(end+1,1)="stageC"; metrics(end+1,1)=mf; %#ok<AGROW>
    catch
    end
end

% Stage D fixed 2-D grid.
dRoot = fullfile(cfgRoot,"stageD_kp_ki_polish");
RD = local_load_stageD_records(dRoot,opts);
for i=1:height(RD)
    try
        mf=string(RD.metrics_file(i));
        M=readtable(mf,'VariableNamingRule','preserve');
        kp(end+1,1)=double(RD.HeightToVzGain(i)); %#ok<AGROW>
        ki(end+1,1)=double(RD.HeightIntegralGain(i)); %#ok<AGROW>
        obj(end+1,1)=local_formal_learning_objective(M,opts); %#ok<AGROW>
        gate(end+1,1)=local_formal_gate_ratio(M,opts); %#ok<AGROW>
        formal(end+1,1)=logical(M.formal_pass(1)) && ~logical(M.hard_fail(1)); %#ok<AGROW>
        source(end+1,1)="stageD"; metrics(end+1,1)=mf; %#ok<AGROW>
    catch
    end
end

% Prior or newly completed Stage-E adaptive evaluations.
eRoot = fullfile(cfgRoot,"stageE_adaptive_kpki");
if isfolder(eRoot)
    dirs=dir(fullfile(eRoot,"Kp_*_Ki_*"));
    dirs=dirs([dirs.isdir]);
    for i=1:numel(dirs)
        tok=regexp(dirs(i).name,'^Kp_([-+0-9.eE]+)_Ki_([-+0-9.eE]+)$','tokens','once');
        if isempty(tok), continue; end
        thisKp=str2double(tok{1}); thisKi=str2double(tok{2});
        if ~isfinite(thisKp) || ~isfinite(thisKi), continue; end
        evalRoot=fullfile(dirs(i).folder,dirs(i).name);
        mf=fullfile(evalRoot,"certification_summary.csv");
        if ~isfile(mf), continue; end
        % v28: do not teach the GP that a logging/infrastructure failure is
        % a bad controller. Those points are eligible for re-simulation.
        if local_stageE_infrastructure_failure(evalRoot), continue; end
        try
            M=readtable(mf,'VariableNamingRule','preserve');
            kp(end+1,1)=thisKp; %#ok<AGROW>
            ki(end+1,1)=thisKi; %#ok<AGROW>
            obj(end+1,1)=local_formal_learning_objective(M,opts); %#ok<AGROW>
            gate(end+1,1)=local_formal_gate_ratio(M,opts); %#ok<AGROW>
            formal(end+1,1)=logical(M.formal_pass(1)) && ~logical(M.hard_fail(1)); %#ok<AGROW>
            source(end+1,1)="stageE"; metrics(end+1,1)=string(mf); %#ok<AGROW>
        catch
        end
    end
end

if isempty(kp)
    P=table();
    return;
end
P=table(double(kp),double(ki),double(obj),double(gate),logical(formal),string(source),string(metrics), ...
    'VariableNames',{'HeightToVzGain','HeightIntegralGain','learning_objective', ...
    'gate_ratio','formal_pass','source','metrics_file'});
P=local_deduplicate_kpki_prior(P,opts);
P=sortrows(P,{'formal_pass','gate_ratio','learning_objective'}, ...
    {'descend','ascend','ascend'});
end

function P = local_deduplicate_kpki_prior(P,opts)
if isempty(P), return; end
% Quantize much finer than the physical usefulness of this controller so
% asynchronous duplicate suggestions do not overweight the GP.
kpTol = double(opts.StageEPriorKpQuantization);
kiTol = double(opts.StageEPriorKiQuantization);
sig = string(round(P.HeightToVzGain/kpTol))+"_"+string(round(P.HeightIntegralGain/kiTol));
[~,~,g]=unique(sig,"stable");
keep=false(height(P),1);
for k=1:max(g)
    idx=find(g==k);
    % Formal first, then gate distance, then learning objective.
    key=[-double(P.formal_pass(idx)), double(P.gate_ratio(idx)), ...
         double(P.learning_objective(idx))];
    [~,ord]=sortrows(key,[1 2 3]);
    keep(idx(ord(1)))=true;
end
P=P(keep,:);
end

function R = local_load_stageC_records(stageRoot,opts)
% v25: deterministic Stage-C resume loader.
%
% Do not depend on dir(stageRoot/**/stageC_record.csv) or on the relative
% metrics_file saved by a worker.  Stage C owns a simple on-disk contract:
%
%   stageC_integral_polish/
%       Ki_0.001500/
%           stageC_record.csv
%           certification_summary.csv
%
% Enumerate Ki_* directories directly, parse Ki from the directory name,
% and always resolve the metrics file locally inside that same directory.
% stageC_record.csv is only used as a source of the stored rank score.
%
% This also makes resume robust to:
%   - changed MATLAB current folder;
%   - Windows relative/backslash paths in historical CSVs;
%   - partially written or schema-mismatched stageC_record.csv files;
%   - an old record whose rank_score is Inf/NaN.
%
% If the stored rank is unavailable, recompute it from the certification
% summary using the CURRENT scoring function/options.

ki = [];
score = [];
metrics = strings(0,1);
source = strings(0,1);

if ~isfolder(stageRoot)
    R = table();
    return;
end

% Primary path: the actual Stage-C directory contract.
dirs = dir(fullfile(stageRoot,"Ki_*"));
dirs = dirs([dirs.isdir]);

% Backward-compatible fallback: if an older run nested Ki_* one level down,
% discover those directories explicitly.  Avoid **/stageC_record.csv because
% that recursive wildcard was the source of the v24 resume failure.
if isempty(dirs)
    level1 = dir(stageRoot);
    level1 = level1([level1.isdir]);
    level1 = level1(~ismember({level1.name},{'.','..'}));
    nested = struct([]);
    for jj = 1:numel(level1)
        d2 = dir(fullfile(level1(jj).folder,level1(jj).name,"Ki_*"));
        d2 = d2([d2.isdir]);
        if isempty(nested)
            nested = d2;
        else
            nested = [nested; d2]; %#ok<AGROW>
        end
    end
    dirs = nested;
end

fprintf("[V25-200m] Stage C resume scan: %d Ki directories under %s\n", ...
    numel(dirs),string(stageRoot));

for i = 1:numel(dirs)
    kiDir = fullfile(dirs(i).folder,dirs(i).name);

    % Ki is authoritative from the folder name, not from a CSV schema.
    tok = regexp(dirs(i).name,'^Ki_([-+0-9.eE]+)$','tokens','once');
    if isempty(tok), continue; end
    thisKi = str2double(tok{1});
    if ~isfinite(thisKi), continue; end

    metricsFile = fullfile(kiDir,"certification_summary.csv");
    if ~isfile(metricsFile)
        % No completed certification => do not mark this Ki as tested.
        continue;
    end

    thisScore = NaN;
    recordFile = fullfile(kiDir,"stageC_record.csv");

    % Try to reuse the stored score, but never require this file to have a
    % particular schema.
    if isfile(recordFile)
        try
            one = readtable(recordFile,'VariableNamingRule','preserve');
            vars = string(one.Properties.VariableNames);
            idx = find(strcmpi(vars,"rank_score"),1);
            if ~isempty(idx) && height(one) >= 1
                raw = one{1,idx};
                if iscell(raw), raw = raw{1}; end
                thisScore = double(raw);
            end
        catch
            thisScore = NaN;
        end
    end

    % Recompute missing/bad historical scores from the actual certification.
    if ~isfinite(thisScore)
        try
            M = readtable(metricsFile,'VariableNamingRule','preserve');
            thisScore = local_score_from_metrics(M,opts);
            thisSource = "recomputed_from_metrics";
        catch
            thisScore = double(opts.HardFailScore);
            thisSource = "metrics_read_failed";
        end
    else
        thisSource = "stageC_record";
    end

    ki(end+1,1) = double(thisKi); %#ok<AGROW>
    score(end+1,1) = double(thisScore); %#ok<AGROW>
    metrics(end+1,1) = string(metricsFile); %#ok<AGROW>
    source(end+1,1) = thisSource; %#ok<AGROW>
end

if isempty(ki)
    fprintf("[V25-200m] Stage C resume scan loaded 0 completed records.\n");
    R = table();
    return;
end

R = table(double(ki),double(score),string(metrics),string(source), ...
    'VariableNames',{'HeightIntegralGain','rank_score','metrics_file','resume_source'});

% One row per Ki.  Prefer the lowest rank if duplicate historical folders
% somehow exist.
R = sortrows(R,{'HeightIntegralGain','rank_score'},{'ascend','ascend'});
[~,ia] = unique(round(double(R.HeightIntegralGain),10),"stable");
R = R(ia,:);
R = sortrows(R,"rank_score","ascend");

fprintf("[V25-200m] Stage C resume loaded %d records; best Ki=%.7f rank=%.4g\n", ...
    height(R),double(R.HeightIntegralGain(1)),double(R.rank_score(1)));
end

function ctx = local_stage_context(paths,master,checkpoint,cfgId,physicalNominals,hiddenTrim,stageRoot,opts)
ctx=struct("paths",paths,"master",master,"checkpoint",checkpoint,"cfgId",cfgId, ...
    "cfgRoot",string(stageRoot),"physicalNominals",physicalNominals, ...
    "hiddenTrim",hiddenTrim,"opts",opts);
end

function score = local_stage_objective(x,ctx)
local_prepare_worker_filegen(ctx);
if string(ctx.stage_mode)=="inner"
    c = local_stageA_candidate_from_x(x,ctx.opts);
elseif string(ctx.stage_mode)=="outer"
    c = ctx.base_candidate;
    c.HeightToVzGain = double(x.HeightToVzGain);
    c.HeightIntegralGain = 0.0;
    c.HeightVzLimit = double(x.HeightVzLimit);
else
    error("Unknown stage mode.");
end
[~,token]=fileparts(tempname);
tag="eval_"+string(datetime("now","Format","yyyyMMdd_HHmmss_SSS"))+"_"+string(token);
evalRoot=fullfile(ctx.cfgRoot,"optimization",tag);
if ~isfolder(evalRoot), mkdir(evalRoot); end
bankMat=fullfile(evalRoot,"bank.mat");
M=table();
try
    local_build_combined_bank(ctx.master,ctx.checkpoint,ctx.cfgId,c,ctx.physicalNominals,bankMat,ctx.opts);
    M=local_run_certification(ctx.paths,ctx.master,ctx.checkpoint,ctx.cfgId,c, ...
        ctx.physicalNominals,ctx.hiddenTrim,bankMat,evalRoot,"candidate",ctx.opts.OptimizationWindowS,ctx.opts);
    if string(ctx.stage_mode)=="inner"
        score=local_inner_score_from_metrics(M,ctx.opts);
    else
        score=local_score_from_metrics(M,ctx.opts);
    end
    writetable(M,fullfile(evalRoot,"candidate_metrics.csv"));
catch ME
    score=double(ctx.opts.HardFailScore);
    fid=fopen(fullfile(evalRoot,"error.txt"),"w");
    if fid>=0, fprintf(fid,"%s\n",getReport(ME,"extended","hyperlinks","off")); fclose(fid); end
end
local_write_eval_record(evalRoot,ctx.cfgId,c,score,tag,M);
fprintf("[V21-200m] cfg%d stage=%s %s obj=%.4g Kp=%.4g Ki=%.4g\n", ...
    ctx.cfgId,string(ctx.stage_mode),tag,score,c.HeightToVzGain,c.HeightIntegralGain);
end

function c = local_stageA_candidate_from_x(x,opts)
c=struct();
c.Np=round(double(x.Np));
c.Nc=min(round(double(x.Nc)),c.Np);
c.Wh=double(x.Wh);
c.Wvz=double(x.Wvz);
c.Wq=double(x.Wq);
c.RateScale=double(x.RateScale);
c.Authority=double(x.Authority);
c.HeightToVzGain=double(opts.StageAFixedHeightKp);
c.HeightIntegralGain=0.0;
c.HeightVzLimit=double(opts.StageAHeightVzLimit);
c.Wva=double(opts.FixedAirspeedWeight);
c.Wpitch=double(opts.FixedPitchWeight);
c.MVWeights=double(opts.MVWeights(:)).';
c.MVRateWeights=double(opts.MVRateWeights(:)).'*c.RateScale;
end

function vars=local_stageA_variables()
vars=[ ...
    optimizableVariable("Np",[6 16],"Type","integer")
    optimizableVariable("Nc",[2 5],"Type","integer")
    optimizableVariable("Wh",[0.15 8.0],"Transform","log")
    optimizableVariable("Wvz",[4.0 50.0],"Transform","log")
    optimizableVariable("Wq",[1.5 15.0],"Transform","log")
    optimizableVariable("RateScale",[0.50 3.5],"Transform","log")
    optimizableVariable("Authority",[0.55 1.00])
    ];
end

function vars=local_stageB_variables()
vars=[ ...
    optimizableVariable("HeightToVzGain",[0.04 0.30],"Transform","log")
    optimizableVariable("HeightVzLimit",[0.45 1.50])
    ];
end

function bo=local_run_stage_bayesopt(objective,vars,initialX,initialObjective,newEval,parallelEnabled,opts)
priorCount=height(initialX);
total=priorCount+round(double(newEval));
args={"MaxObjectiveEvaluations",total,"IsObjectiveDeterministic",true, ...
    "UseParallel",logical(parallelEnabled),"AcquisitionFunctionName","expected-improvement-plus", ...
    "Verbose",double(opts.BayesoptVerbose)};
if logical(parallelEnabled)
    args=[args,{"MinWorkerUtilization",local_parallel_worker_count(true,opts)}]; %#ok<AGROW>
end
if ~isempty(initialX)
    args=[args,{"InitialX",initialX,"InitialObjective",initialObjective}]; %#ok<AGROW>
end
bo=bayesopt(objective,vars,args{:});
end

function H=local_consolidate_stage_history(stageRoot,historyCsv,opts)
H=local_empty_history();
if isfile(historyCsv)
    try, H=local_history_core(readtable(historyCsv)); catch, H=local_empty_history(); end
end
files=dir(fullfile(stageRoot,"optimization","**","evaluation_record.csv"));
for i=1:numel(files)
    try
        R=local_history_core(readtable(fullfile(files(i).folder,files(i).name)));
        H=[H;R]; %#ok<AGROW>
    catch
    end
end
if isempty(H), writetable(H,historyCsv); return; end
H=H(isfinite(H.objective),:);
if isempty(H), H=local_empty_history(); writetable(H,historyCsv); return; end
sig=strings(height(H),1);
for i=1:height(H)
    sig(i)=local_candidate_signature(local_candidate_from_history_row(H(i,:),opts));
end
[~,~,g]=unique(sig,"stable");
keep=false(height(H),1);
for k=1:max(g)
    idx=find(g==k); [~,j]=min(H.objective(idx)); keep(idx(j))=true;
end
H=sortrows(H(keep,:),"objective","ascend");
writetable(H,historyCsv);
end

function [X,y]=local_stage_prior(H,names,maxPoints)
X=table(); y=[];
if isempty(H), return; end
H=H(isfinite(H.objective),:);
if isempty(H), return; end
H=sortrows(H,"objective","ascend");
n=min(round(double(maxPoints)),height(H));
H=H(1:n,:);
X=H(:,names);
y=double(H.objective(:));
end

function score=local_inner_score_from_metrics(M,opts)
if isempty(M) || height(M)<1 || logical(M.hard_fail(1))
    score=double(opts.HardFailScore); return;
end
score = ...
    8.0*double(M.steady_vz_rms_mps(1))^2 + ...
    6.0*double(M.steady_q_rms_dps(1))^2 + ...
    12.0*abs(double(M.tail_vz_mps(1)))^2 + ...
    0.8*double(M.steady_h_rms_m(1))^2 + ...
    0.5*double(M.steady_Va_rms_mps(1))^2 + ...
    0.15*abs(double(M.tail_h_error_m(1)))^2;
if ~isfinite(score), score=double(opts.HardFailScore); end
end

function pass=local_inner_gate(M,opts)
pass=~logical(M.hard_fail(1)) && ...
    double(M.steady_vz_rms_mps(1)) <= double(opts.StageAInnerMaxVzRmsMps) && ...
    double(M.steady_q_rms_dps(1)) <= double(opts.StageAInnerMaxQRmsDps) && ...
    abs(double(M.tail_vz_mps(1))) <= double(opts.StageAInnerMaxTailVzMps) && ...
    double(M.steady_Va_rms_mps(1)) <= double(opts.StageAInnerMaxVaRmsMps) && ...
    double(M.steady_h_max_abs_m(1)) <= double(opts.StageAInnerMaxHeightErrorM);
end

function pass=local_outer_gate(M,opts)
pass=~logical(M.hard_fail(1)) && ...
    double(M.steady_h_rms_m(1)) <= double(opts.StageBOuterMaxHeightRmsM) && ...
    double(M.steady_h_max_abs_m(1)) <= double(opts.StageBOuterMaxHeightErrorM) && ...
    abs(double(M.steady_h_drift_m(1))) <= double(opts.StageBOuterMaxHeightDriftM) && ...
    abs(double(M.tail_h_error_m(1))) <= double(opts.StageBOuterMaxTailHeightErrorM) && ...
    abs(double(M.tail_vz_mps(1))) <= double(opts.StageBOuterMaxTailVzMps) && ...
    double(M.steady_q_rms_dps(1)) <= double(opts.StageBOuterMaxQRmsDps);
end

function checkpoint=local_commit_candidate(master,checkpoint,cfgId,c,M,physicalNominals,cfgRoot,opts)
score=local_score_from_metrics(M,opts);
bestBank=fullfile(cfgRoot,"best_mpc_bank_200m.mat");
local_build_combined_bank(master,checkpoint,cfgId,c,physicalNominals,bestBank,opts);
checkpoint.best_candidate{cfgId+1}=c;
checkpoint.best_objective(cfgId+1)=score;
checkpoint.best_bank_path(cfgId+1)=string(bestBank);
checkpoint.last_metrics{cfgId+1}=M;
checkpoint.verification_count(cfgId+1)=checkpoint.verification_count(cfgId+1)+1;
writetable(local_candidate_table(cfgId,c,score),fullfile(cfgRoot,"best_candidate.csv"));
writetable(M,fullfile(cfgRoot,"final_validation_summary.csv"));
end

function v=local_extract_physical_nominal_from_data(dataRoot,cfgId)
vals=[];
files=dir(fullfile(dataRoot,"**","auto_id_timeseries.csv"));
for i=1:numel(files)
    try
        T=readtable(fullfile(files(i).folder,files(i).name));
        if ~ismember("config_id",string(T.Properties.VariableNames)) || ...
                ~ismember("elevator_cmd_norm",string(T.Properties.VariableNames)), continue; end
        if round(median(double(T.config_id),"omitnan")) ~= cfgId, continue; end
        mask=true(height(T),1);
        if ismember("elevator_excitation",string(T.Properties.VariableNames))
            mask=abs(double(T.elevator_excitation))<1e-8;
        end
        v=double(T.elevator_cmd_norm(mask));
        v=v(isfinite(v));
        if ~isempty(v), vals(end+1,1)=median(v,"omitnan"); end %#ok<AGROW>
    catch
    end
end
if isempty(vals), v=NaN; else, v=median(vals,"omitnan"); end
end

function s=local_linear_slope(t,y)
t=double(t(:)); y=double(y(:));
m=isfinite(t)&isfinite(y);
if nnz(m)<3, s=NaN; return; end
t0=t(find(m,1,"first"));
pp=polyfit(t(m)-t0,y(m),1);
s=pp(1);
end

function M=local_empty_cert_like_metrics(cfgId)
M=table(cfgId,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN, ...
    NaN,NaN,NaN,NaN,false,false,false,NaN, ...
    'VariableNames',{'config_id','min_altitude_m','max_altitude_m','steady_h_rms_m', ...
    'steady_h_max_abs_m','steady_h_drift_m','steady_Va_rms_mps','steady_vz_rms_mps', ...
    'steady_q_rms_dps','tail_h_error_m','tail_vz_mps','tail_q_dps', ...
    'max_physical_elevator_deviation','max_throttle_deviation', ...
    'max_bridge_elevator_error','max_bridge_throttle_error','reached_config', ...
    'hard_fail','formal_pass','rank_score'});
end

% ========================================================================
% Missing plant preparation: trim -> clean ID -> identify -> validate
% ========================================================================
function [master, info] = local_prepare_missing_plant(master, cfgId, paths, cfgRoot, opts, forceRetrim)
if nargin < 6, forceRetrim = false; end
trainRoot = fullfile(cfgRoot, "plant_training");
if ~isfolder(trainRoot), mkdir(trainRoot); end
seedFile = fullfile(trainRoot, "master_seed.mat");
result = master; %#ok<NASGU>
save(seedFile, "result", "-v7.3");

trimRoot = fullfile(trainRoot, "trim");
trimMat = fullfile(trimRoot, "auto_trim_bank.mat");
trimResult = airdropx_auto_find_trim( ...
    "ProjectRoot", paths.projectRoot, ...
    "OutputMat", trimMat, ...
    "WorkRoot", fullfile(trimRoot, "search"), ...
    "PreviousTrimMat", seedFile, ...
    "ConfigIds", cfgId, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "ReferenceMassKg", opts.ReferenceMassKg, ...
    "CargoMassKg", opts.CargoMassKg, ...
    "SearchAltitudeM", max(double(opts.TargetAltitudeM), double(opts.TrimSearchAltitudeM)), ...
    "StopTimeS", opts.TrimStopTimeS, ...
    "RecordStartS", opts.TrimRecordStartS, ...
    "MaxObjectiveEvaluations", local_cfg_value(opts.TrimMaxObjectiveEvaluationsByConfig, cfgId, 90), ...
    "ReuseVerifiedTrim", ~logical(forceRetrim), ...
    "ReuseFailedAsWarmStart", true, ...
    "MaxTrimAbsVzMps", opts.RetrimMaxAbsVzMps, ...
    "MaxTailAbsVzMps", opts.RetrimMaxTailAbsVzMps, ...
    "MaxTailHeightSlopeMps", opts.RetrimMaxTailHeightSlopeMps, ...
    "TailRescueMaxAbsVzMps", opts.RetrimTailRescueMaxAbsVzMps, ...
    "TailRescueMaxHeightSlopeMps", opts.RetrimTailRescueMaxHeightSlopeMps, ...
    "PreparationTrimBank", master.trim_bank, ...
    "UsePreparationTrimSchedule", true, ...
    "MaxFullAltitudeRmsM", opts.RetrimMaxFullAltitudeRmsM, ...
    "MaxFullAltitudeMaxAbsM", opts.RetrimMaxFullAltitudeMaxAbsM, ...
    "MaxEarlyAltitudeRmsM", opts.RetrimMaxEarlyAltitudeRmsM, ...
    "MaxEarlyAltitudeMaxAbsM", opts.RetrimMaxEarlyAltitudeMaxAbsM, ...
    "UseParallel", logical(opts.UseParallel));
trimBank = trimResult.trim_bank;

% The aerodynamic trim search may run safely above the mission altitude.
% Re-anchor the resulting equilibrium to the requested mission coordinate
% before clean-ID / MPC construction; absolute altitude is not a trim state.
trimBank(cfgId + 1).altitude_m = double(opts.TargetAltitudeM);
trimBank(cfgId + 1).airspeed_mps = double(opts.TargetAirspeedMps);

dataRoot = fullfile(trainRoot, "data");
airdropx_auto_generate_data( ...
    "ProjectRoot", paths.projectRoot, ...
    "OutputRoot", dataRoot, ...
    "TrimBank", trimBank, ...
    "ConfigIds", cfgId, ...
    "RunsPerConfig", opts.RunsPerConfig, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "ReferenceMassKg", opts.ReferenceMassKg, ...
    "CargoMassKg", opts.CargoMassKg, ...
    "IdentificationAltitudeM", opts.TargetAltitudeM, ...
    "MaxSettleRetries", opts.IdMaxSettleRetries, ...
    "SettleRetryStepS", opts.IdSettleRetryStepS, ...
    "RequireStableBaseline", true, ...
    "BaselineDurationS", opts.StrictBaselineDurationS, ...
    "BaselineMaxAirspeedErrorMps", opts.StrictBaselineMaxAirspeedErrorMps, ...
    "BaselineMaxPitchErrorDeg", opts.StrictBaselineMaxPitchErrorDeg, ...
    "BaselineMaxAbsVzMps", opts.StrictBaselineMaxAbsVzMps, ...
    "BaselineMaxAbsQDps", opts.StrictBaselineMaxAbsQDps, ...
    "BaselineMaxHeightSlopeMps", opts.StrictBaselineMaxHeightSlopeMps, ...
    "PreparationTrimBank", trimBank, ...
    "UsePreparationTrimSchedule", true);

% Resolve the NEW physical elevator nominal from the stable pre-excitation
% baseline. This prevents old v11 cfg2 physical input data from overriding a
% freshly rebuilt trim.
physicalNominal = local_extract_physical_nominal_from_data(dataRoot, cfgId);
if isfinite(physicalNominal)
    trimBank(cfgId + 1).physical_elevator_cmd = physicalNominal;
end

idRoot = fullfile(trainRoot, "iddata");
if ~isfolder(idRoot), mkdir(idRoot); end
idMat = fullfile(idRoot, "airdropx_iddata.mat");
airdropx_auto_build_iddata( ...
    "InputRoot", dataRoot, ...
    "OutputMat", idMat, ...
    "TrimBank", trimBank, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "Ts", opts.IdentificationTs);

identRoot = fullfile(trainRoot, "identify");
if ~isfolder(identRoot), mkdir(identRoot); end
identMat = fullfile(identRoot, "airdropx_identified_plants.mat");
newIdent = airdropx_auto_identify( ...
    "DataMat", idMat, ...
    "OutputMat", identMat, ...
    "Orders", opts.IdentificationOrders, ...
    "RefineWithSsest", false);
val = airdropx_auto_validate_models( ...
    "Identified", newIdent, ...
    "OutputFile", fullfile(identRoot, "validation_report.csv"), ...
    "PredictionSteps", [5 10 20]);

R = val.table;
val5 = R(R.config_id == cfgId & string(R.split) == "validation" & R.prediction_steps == 5, :);
test5 = R(R.config_id == cfgId & string(R.split) == "test" & R.prediction_steps == 5, :);
if isempty(val5) || isempty(test5)
    error("AirdropX:AutoMPC200:PlantValidationMissing", ...
        "cfg%d missing independent validation/test 5-step results.", cfgId);
end
if val5.fit_mean_pct(1) < opts.MinPlantValidation5StepFitPct || ...
        test5.fit_mean_pct(1) < opts.MinPlantTest5StepFitPct
    error("AirdropX:AutoMPC200:PlantValidationFailed", ...
        "cfg%d Plant failed gate: validation5=%.1f%% test5=%.1f%%.", ...
        cfgId, val5.fit_mean_pct(1), test5.fit_mean_pct(1));
end

idx = cfgId + 1;
master.plant_bank{idx} = newIdent.plant_bank{idx};
if isfield(master, "models") && isfield(newIdent, "models")
    master.models{idx} = newIdent.models{idx};
end
master.trim_bank = local_assign_struct_element_union(master.trim_bank, idx, newIdent.trim_bank(idx));
master.trim_bank(idx).altitude_m = double(opts.TargetAltitudeM);
master.trim_bank(idx).airspeed_mps = double(opts.TargetAirspeedMps);
if isfinite(physicalNominal)
    master.trim_bank(idx).physical_elevator_cmd = physicalNominal;
end
info = struct();
info.validation = table(cfgId, val5.fit_mean_pct(1), test5.fit_mean_pct(1), ...
    val5.fit_min_pct(1), test5.fit_min_pct(1), ...
    'VariableNames', {'config_id','validation_5step_mean_pct','test_5step_mean_pct', ...
    'validation_5step_min_pct','test_5step_min_pct'});
writetable(info.validation, fullfile(trainRoot, "plant_gate_summary.csv"));
end


function bank = local_assign_struct_element_union(bank, idx, src)
% Assign one struct element even when a rebuilt trim adds fields that older
% checkpoint elements do not yet have. Preserve destination-only fields for
% the target config and add source-only fields to every element.
if isempty(bank)
    bank = src;
    return;
end
dstFields = string(fieldnames(bank));
srcFields = string(fieldnames(src));
for f = setdiff(srcFields, dstFields).'
    fn = char(f);
    for k = 1:numel(bank)
        bank(k).(fn) = [];
    end
end
dstFields = string(fieldnames(bank));
for f = setdiff(dstFields, string(fieldnames(src))).'
    fn = char(f);
    try
        src.(fn) = bank(idx).(fn);
    catch
        src.(fn) = [];
    end
end
bank(idx) = src;
end

% ========================================================================
% Bayesian objective and certification run
% ========================================================================
function score = local_objective(x, ctx)
local_prepare_worker_filegen(ctx);
c = local_candidate_from_x(x, ctx.opts);
[~, token] = fileparts(tempname);
tag = "eval_" + string(datetime("now","Format","yyyyMMdd_HHmmss_SSS")) + "_" + string(token);
evalRoot = fullfile(ctx.cfgRoot, "optimization", tag);
if ~isfolder(evalRoot), mkdir(evalRoot); end
bankMat = fullfile(evalRoot, "combined_mpc_bank.mat");
M = table();
try
    local_build_combined_bank(ctx.master, ctx.checkpoint, ctx.cfgId, c, ...
        ctx.physicalNominals, bankMat, ctx.opts);
    M = local_run_certification(ctx.paths, ctx.master, ctx.checkpoint, ctx.cfgId, c, ...
        ctx.physicalNominals, ctx.hiddenTrim, bankMat, evalRoot, "candidate", ...
        ctx.opts.OptimizationWindowS, ctx.opts);
    writetable(M, fullfile(evalRoot, "candidate_metrics.csv"));
    score = local_score_from_metrics(M, ctx.opts);
catch ME
    score = double(ctx.opts.HardFailScore);
    fid = fopen(fullfile(evalRoot, "error.txt"), "w");
    if fid >= 0
        fprintf(fid, "%s\n\n%s\n", ME.message, getReport(ME,"extended","hyperlinks","off"));
        fclose(fid);
    end
end

% Never append to one shared CSV from parallel workers.  Each worker writes
% only inside its unique evalRoot.  The client consolidates these records.
local_write_eval_record(evalRoot, ctx.cfgId, c, score, tag, M);
fprintf("[V21-200m] cfg%d %s obj=%.4g Np=%d Nc=%d Wh=%.3g Wvz=%.3g Wq=%.3g Kh=%.3g Ki=%.4g vzLim=%.2f auth=%.2f\n", ...
    ctx.cfgId, tag, score, c.Np, c.Nc, c.Wh, c.Wvz, c.Wq, ...
    c.HeightToVzGain, c.HeightIntegralGain, c.HeightVzLimit, c.Authority);
end

function M = local_run_certification(paths, master, checkpoint, cfgId, c, physicalNominals, ...
    hiddenTrim, bankMat, parentRoot, caseName, windowS, opts)
c = local_upgrade_candidate(c);
reachS = local_config_reach_time(cfgId, opts);
pulse1S = max(opts.BasePulseStartS, reachS + local_cfg_double(opts.PulseAfterConfigReachByConfigS, cfgId, opts.PulseAfterConfigReachS));
pulse2S = pulse1S + opts.PulseSeparationS;
stopS = pulse2S + max(windowS, opts.PostSecondPulseMinS);
scoreStartS = max(opts.MpcEnableTimeS + 1.0, pulse1S - 2.0);

[authorityByCfg, gainByCfg, integralGainByCfg, vzLimByCfg] = local_control_vectors(checkpoint, cfgId, c);
initialElev = physicalNominals(1) - hiddenTrim;
initialThrottle = double(master.trim_bank(1).throttle_cmd);

simResult = airdropx_auto_run_closed_loop( ...
    "ProjectRoot", paths.projectRoot, ...
    "MpcBankMat", bankMat, ...
    "OutputRoot", fullfile(parentRoot, string(caseName)), ...
    "CaseId", string(caseName), ...
    "StopTimeS", stopS, ...
    "FixedConfigId", NaN, ...
    "FixedDropTotal", cfgId, ...
    "FixedDropStartS", opts.DropStartS, ...
    "FixedDropIntervalS", opts.DropIntervalS, ...
    "InitialAltitudeM", opts.TargetAltitudeM, ...
    "InitialAirspeedMps", opts.TargetAirspeedMps, ...
    "InitialPitchDeg", master.trim_bank(1).pitch_deg, ...
    "InitialFlightPathDeg", 0.0, ...
    "InitialElevatorDelta", initialElev, ...
    "InitialThrottleCmd", initialThrottle, ...
    "ReferenceMassKg", opts.ReferenceMassKg, ...
    "CargoMassKg", opts.CargoMassKg, ...
    "HiddenElevatorTrim", hiddenTrim, ...
    "MpcEnableTimeS", opts.MpcEnableTimeS, ...
    "MpcAuthorityScale", c.Authority, ...
    "MpcAuthorityByConfig", authorityByCfg, ...
    "HeightToVzGain", c.HeightToVzGain, ...
    "HeightToVzGainByConfig", gainByCfg, ...
    "HeightIntegralGain", c.HeightIntegralGain, ...
    "HeightIntegralGainByConfig", integralGainByCfg, ...
    "HeightVzRefLimitMps", c.HeightVzLimit, ...
    "HeightVzRefLimitByConfig", vzLimByCfg, ...
    "V31ContinuousControllerStateEnabled", logical(opts.V31ContinuousControllerStateEnabled), ...
    "V31HeightGovernorEnabled", logical(opts.V31HeightGovernorEnabled), ...
    "V31HeightVzSlewRateMps2", double(opts.V31HeightVzSlewRateMps2), ...
    "V31HeightBiasFraction", double(opts.V31HeightBiasFraction), ...
    "V31HeightBiasLeak", double(opts.V31HeightBiasLeak), ...
    "TestPulse1StartS", pulse1S, ...
    "TestPulse1DurationS", opts.PulseDurationS, ...
    "TestPulse1Elevator", opts.PulseElevatorAmplitude, ...
    "TestPulse1Throttle", -opts.PulseThrottleAmplitude, ...
    "TestPulse2StartS", pulse2S, ...
    "TestPulse2DurationS", opts.PulseDurationS, ...
    "TestPulse2Elevator", -opts.PulseElevatorAmplitude, ...
    "TestPulse2Throttle", opts.PulseThrottleAmplitude, ...
    "ElevatorDevStepLimit", opts.ElevatorDeviationRateLimit, ...
    "ThrottleDevStepLimit", opts.ThrottleDeviationRateLimit, ...
    "TrustAltitudeM", 1.0e6, ...
    "TrustAirspeedMps", opts.TrustAirspeedMps, ...
    "TrustPitchDeg", opts.TrustPitchDeg, ...
    "TrustVzMps", opts.TrustVzMps, ...
    "TrustQDps", opts.TrustQDps, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", master.trim_bank(cfgId + 1).pitch_deg, ...
    "UseTrimPitchReference", 1);

M = local_metrics(simResult.timeseries, cfgId, physicalNominals(cfgId + 1), ...
    master.trim_bank(cfgId + 1).throttle_cmd, scoreStartS, pulse1S, pulse2S, opts);
M = addvars(M, string(caseName), pulse1S, pulse2S, stopS, 'Before', 1, ...
    'NewVariableNames', {'case_name','pulse1_s','pulse2_s','stop_time_s'});
if ~startsWith(string(caseName), "candidate")
    local_plot(simResult.timeseries, physicalNominals(cfgId + 1), ...
        master.trim_bank(cfgId + 1).throttle_cmd, ...
        fullfile(parentRoot, string(caseName) + "_curves.png"), ...
        sprintf("v29 cfg%d H=%.0fm V=%.1fmps %s", cfgId, opts.TargetAltitudeM, opts.TargetAirspeedMps, caseName));
end
end

function M = local_metrics(T, cfgId, eNom, tNom, scoreStartS, pulse1S, pulse2S, opts)
t = double(T.time_s(:));
h = local_col(T,"altitude_m");
V = local_col(T,"airspeed_mps");
vz = local_col(T,"vz_up_mps");
q = local_col(T,"q_dps");
pitch = local_col(T,"pitch_deg");
e = local_col(T,"elevator_cmd_norm");
th = local_col(T,"throttle_norm");
be = local_col(T,"bridge_elevator_error");
bt = local_col(T,"bridge_throttle_error");
dropCount = local_col(T,"drop_count");
mass = local_col(T,"mass_kg");
cg = local_col(T,"cg_x_m");

steady = isfinite(t) & t >= scoreStartS;
if nnz(steady) < 10, steady = isfinite(t); end
tailStart = max(max(t(isfinite(t))) - opts.TailWindowS, scoreStartS);
tail = isfinite(t) & t >= tailStart;
if nnz(tail) < 10, tail = steady; end
idx = find(steady);
nEdge = max(3,round(0.12*numel(idx)));
headIdx = idx(1:min(nEdge,numel(idx)));
tailIdx = idx(max(1,numel(idx)-nEdge+1):numel(idx));

hErr = h - opts.TargetAltitudeM;
vErr = V - opts.TargetAirspeedMps;
hRms = local_rms(hErr(steady));
hMax = max(abs(hErr(steady)),[],"omitnan");
hDrift = median(h(tailIdx),"omitnan") - median(h(headIdx),"omitnan");
vaRms = local_rms(vErr(steady));
vzRms = local_rms(vz(steady));
qRms = local_rms(q(steady));
tailHErr = median(hErr(tail),"omitnan");
tailVz = median(vz(tail),"omitnan");
tailQ = median(q(tail),"omitnan");
contextMass = median(mass(steady),"omitnan");
contextCg = median(cg(steady),"omitnan");
minH = min(h,[],"omitnan");
maxH = max(h,[],"omitnan");
maxPitch = max(abs(pitch),[],"omitnan");
maxED = max(abs(e(steady)-double(eNom)),[],"omitnan");
maxTD = max(abs(th(steady)-double(tNom)),[],"omitnan");
maxBE = max(abs(be),[],"omitnan");
maxBT = max(abs(bt),[],"omitnan");
reachedCfg = any(round(dropCount(isfinite(dropCount))) >= cfgId);

% Recovery time after each pulse: first time a short window remains inside the
% practical height/vz band.  NaN is penalized but not a separate hard fail.
rec1 = local_recovery_time(t,hErr,vz,pulse1S,opts);
rec2 = local_recovery_time(t,hErr,vz,pulse2S,opts);
recPenalty = local_recovery_penalty(rec1,opts) + local_recovery_penalty(rec2,opts);

hard = ~reachedCfg || ~isfinite(minH) || ~isfinite(maxH) || ...
    max(abs([minH maxH]-opts.TargetAltitudeM)) > opts.HardMaxAltitudeErrorM || ...
    qRms > opts.HardMaxQRmsDps || maxPitch > opts.HardMaxAbsPitchDeg || ...
    maxBE > opts.MaxBridgeError || maxBT > opts.MaxBridgeError;
formal = ~hard && ...
    hRms <= opts.PassAltitudeRmsM && hMax <= opts.PassAltitudeMaxM && ...
    abs(hDrift) <= opts.PassAltitudeDriftM && vaRms <= opts.PassAirspeedRmsMps && ...
    vzRms <= opts.PassVzRmsMps && qRms <= opts.PassQRmsDps && ...
    abs(tailHErr) <= opts.PassTailAltitudeErrorM && abs(tailVz) <= opts.PassTailVzMps;

% v18: make the search care about the exact metrics that blocked cfg0 in v16/v17.
% The smooth barriers start before the hard 1 m / 0.35 m/s graduation gates,
% so Bayesopt is rewarded for removing long-tail bias instead of merely
% reducing transient RMS.
driftPressure = opts.DriftPressureWeight * ...
    max(0.0, (abs(hDrift) - opts.DriftPressureStartM) / ...
    max(opts.PassAltitudeDriftM - opts.DriftPressureStartM, eps))^2;
tailPressure = opts.TailPressureWeight * ...
    max(0.0, (abs(tailHErr) - opts.TailPressureStartM) / ...
    max(opts.PassTailAltitudeErrorM - opts.TailPressureStartM, eps))^2;
tailVzPressure = opts.TailVzPressureWeight * ...
    max(0.0, (abs(tailVz) - opts.TailVzPressureStartMps) / ...
    max(opts.PassTailVzMps - opts.TailVzPressureStartMps, eps))^2;

rankScore = 12*hRms + 4*hMax + 10*abs(hDrift) + 14*abs(tailHErr) + ...
    4*vaRms + 5*vzRms + 2*qRms + 8*abs(tailVz) + 0.5*abs(tailQ) + ...
    1.5*(maxED/max(opts.ElevatorDeviationLimit,eps))^2 + ...
    1.0*(maxTD/max(opts.ThrottleDeviationLimit,eps))^2 + recPenalty + ...
    driftPressure + tailPressure + tailVzPressure;

M = table(cfgId,minH,maxH,hRms,hMax,hDrift,vaRms,vzRms,qRms,tailHErr,tailVz,tailQ, ...
    rec1,rec2,maxED,maxTD,maxBE,maxBT,logical(reachedCfg),logical(hard),logical(formal),rankScore, ...
    contextMass,contextCg, ...
    'VariableNames', {'config_id','min_altitude_m','max_altitude_m','steady_h_rms_m', ...
    'steady_h_max_abs_m','steady_h_drift_m','steady_Va_rms_mps','steady_vz_rms_mps', ...
    'steady_q_rms_dps','tail_h_error_m','tail_vz_mps','tail_q_dps', ...
    'pulse1_recovery_s','pulse2_recovery_s','max_physical_elevator_deviation', ...
    'max_throttle_deviation','max_bridge_elevator_error','max_bridge_throttle_error', ...
    'reached_config','hard_fail','formal_pass','rank_score','context_mass_kg','context_cg_x_m'});
end


function score = local_score_from_metrics(M,opts)
if isempty(M) || height(M) < 1
    score = double(opts.HardFailScore);
    return;
end
m = M(1,:);
try
    if logical(m.hard_fail)
        score = double(opts.HardFailScore);
        return;
    end
    % Recompute instead of trusting an old rank_score column so v16 history
    % can be migrated without rerunning any trajectory.
    hRms = double(m.steady_h_rms_m);
    hMax = double(m.steady_h_max_abs_m);
    hDrift = abs(double(m.steady_h_drift_m));
    vaRms = double(m.steady_Va_rms_mps);
    vzRms = double(m.steady_vz_rms_mps);
    qRms = double(m.steady_q_rms_dps);
    tailHErr = abs(double(m.tail_h_error_m));
    tailVz = abs(double(m.tail_vz_mps));
    tailQ = abs(double(m.tail_q_dps));
    maxED = double(m.max_physical_elevator_deviation);
    maxTD = double(m.max_throttle_deviation);
    recPenalty = local_recovery_penalty(double(m.pulse1_recovery_s),opts) + ...
        local_recovery_penalty(double(m.pulse2_recovery_s),opts);
    driftPressure = opts.DriftPressureWeight * ...
        max(0.0,(hDrift-opts.DriftPressureStartM) / ...
        max(opts.PassAltitudeDriftM-opts.DriftPressureStartM,eps))^2;
    tailPressure = opts.TailPressureWeight * ...
        max(0.0,(tailHErr-opts.TailPressureStartM) / ...
        max(opts.PassTailAltitudeErrorM-opts.TailPressureStartM,eps))^2;
    tailVzPressure = opts.TailVzPressureWeight * ...
        max(0.0,(tailVz-opts.TailVzPressureStartMps) / ...
        max(opts.PassTailVzMps-opts.TailVzPressureStartMps,eps))^2;
    score = 12*hRms + 4*hMax + 10*hDrift + 14*tailHErr + ...
        4*vaRms + 5*vzRms + 2*qRms + 8*tailVz + 0.5*tailQ + ...
        1.5*(maxED/max(opts.ElevatorDeviationLimit,eps))^2 + ...
        1.0*(maxTD/max(opts.ThrottleDeviationLimit,eps))^2 + recPenalty + ...
        driftPressure + tailPressure + tailVzPressure;
    if logical(m.formal_pass)
        score = opts.FormalPassObjectiveMultiplier * score;
    end
    if ~isfinite(score), score = double(opts.HardFailScore); end
catch
    score = double(opts.HardFailScore);
end
end

% ========================================================================
% Bank assembly / per-config controller parameters
% ========================================================================
function local_build_combined_bank(master, checkpoint, cfgId, current, physicalNominals, bankMat, opts)
overlay = local_200m_overlay(master, opts);
controllers = cell(5,1);
mpcMeta = struct();
parent = fileparts(bankMat);
if ~isfolder(parent), mkdir(parent); end

% Reuse already-certified controller objects from their saved banks instead of
% rebuilding MPC0..MPC(N-1) on every candidate evaluation.
for k = 0:cfgId-1
    loaded = false;
    oldBank = "";
    if k + 1 <= numel(checkpoint.best_bank_path)
        oldBank = string(checkpoint.best_bank_path(k + 1));
    end
    if strlength(oldBank) > 0 && isfile(oldBank)
        try
            B = load(oldBank, "controllers", "mpc_meta");
            if isfield(B,"controllers") && numel(B.controllers) >= k+1 && ~isempty(B.controllers{k+1})
                controllers{k+1} = B.controllers{k+1};
                if isfield(B,"mpc_meta"), mpcMeta = B.mpc_meta; end
                loaded = true;
            end
        catch
            loaded = false;
        end
    end
    if ~loaded
        c = checkpoint.best_candidate{k + 1};
        if isempty(c)
            error("AirdropX:AutoMPC200:PriorControllerMissing", ...
                "cfg%d requires verified cfg%d controller first.", cfgId, k);
        end
        tempMat = fullfile(parent, sprintf("_tmp_prior_cfg%d.mat", k));
        local_build_one_style_bank(overlay, physicalNominals, tempMat, c, k, opts);
        B = load(tempMat, "controllers", "mpc_meta");
        controllers{k+1} = B.controllers{k+1};
        if isfield(B,"mpc_meta"), mpcMeta = B.mpc_meta; end
        if isfile(tempMat), delete(tempMat); end
    end
end

% Build only the controller being optimized, not all five mpc() objects.
tempMat = fullfile(parent, sprintf("_tmp_current_cfg%d.mat", cfgId));
local_build_one_style_bank(overlay, physicalNominals, tempMat, current, cfgId, opts);
B = load(tempMat, "controllers", "mpc_meta");
controllers{cfgId+1} = B.controllers{cfgId+1};
if isfield(B,"mpc_meta"), mpcMeta = B.mpc_meta; end
if isfile(tempMat), delete(tempMat); end

trim_bank = overlay.trim_bank; %#ok<NASGU>
mpc_meta = mpcMeta; %#ok<NASGU>
mpc_meta.physical_elevator_nominals = physicalNominals(:);
mpc_meta.v20_combined_per_config = true;
save(bankMat, "controllers", "trim_bank", "mpc_meta", "-v7.3");
end

function local_build_one_style_bank(overlay, physicalNominals, bankMat, c, cfgId, opts)
effectiveWh=double(c.Wh);
if local_v31_height_governor_enabled(opts)
    effectiveWh=0.0;
end
airdropx_auto_build_mpc_bank( ...
    "Identified", overlay, "OutputMat", bankMat, "ConfigIds", cfgId, ...
    "PredictionHorizon", c.Np, "ControlHorizon", c.Nc, ...
    "InputCoordinateMode", "deviation_physical", ...
    "PhysicalElevatorNominals", physicalNominals, ...
    "RequirePhysicalElevatorNominals", false, ...
    "DerivePhysicalElevatorNominalsFromIdData", false, ...
    "ElevatorDeviationLimit", opts.ElevatorDeviationLimit, ...
    "ThrottleDeviationLimit", opts.ThrottleDeviationLimit, ...
    "ElevatorDeviationRateLimit", opts.ElevatorDeviationRateLimit, ...
    "ThrottleDeviationRateLimit", opts.ThrottleDeviationRateLimit, ...
    "OutputWeights", [effectiveWh c.Wva c.Wpitch c.Wvz c.Wq], ...
    "MVWeights", c.MVWeights, "MVRateWeights", c.MVRateWeights, ...
    "DisableOutputDisturbanceModel", true);
end

function [authority,gain,integralGain,vzLim] = local_control_vectors(checkpoint,cfgId,current)
authority = NaN(5,1); gain = NaN(5,1); integralGain = NaN(5,1); vzLim = NaN(5,1);
for k = 0:cfgId
    if k == cfgId, c = current; else, c = checkpoint.best_candidate{k+1}; end
    if isempty(c), continue; end
    c = local_upgrade_candidate(c);
    authority(k+1) = c.Authority;
    gain(k+1) = c.HeightToVzGain;
    integralGain(k+1) = c.HeightIntegralGain;
    vzLim(k+1) = c.HeightVzLimit;
end
end

function overlay = local_200m_overlay(master,opts)
overlay = master;
for k = 1:numel(overlay.trim_bank)
    overlay.trim_bank(k).altitude_m = opts.TargetAltitudeM;
    overlay.trim_bank(k).airspeed_mps = opts.TargetAirspeedMps;
    overlay.trim_bank(k).vz_up_mps = 0.0;
    overlay.trim_bank(k).q_dps = 0.0;
end
end

% ========================================================================
% Candidate/search / fast-resume helpers
% ========================================================================
function tf = local_should_seed_integral_probe(checkpoint,cfgId,H,opts)
tf = false;
if cfgId + 1 > numel(checkpoint.best_candidate) || isempty(checkpoint.best_candidate{cfgId+1})
    return;
end
if isempty(H) || ~ismember("HeightIntegralGain",string(H.Properties.VariableNames))
    return;
end
% Once any real nonzero-Ki objective has been evaluated, do not repeat this
% deterministic seed stage on later invocations.
if any(isfinite(double(H.HeightIntegralGain)) & abs(double(H.HeightIntegralGain)) > 1.0e-8)
    return;
end
if cfgId + 1 > numel(checkpoint.last_metrics) || isempty(checkpoint.last_metrics{cfgId+1})
    return;
end
M = checkpoint.last_metrics{cfgId+1};
if isempty(M) || height(M) < 1
    return;
end
m = M(1,:);
try
    if logical(m.hard_fail)
        return;
    end
    driftNear = abs(double(m.steady_h_drift_m)) <= ...
        double(opts.PassAltitudeDriftM) + double(opts.NearPassDriftMarginM);
    otherPass = ...
        double(m.steady_h_rms_m) <= opts.PassAltitudeRmsM && ...
        double(m.steady_h_max_abs_m) <= opts.PassAltitudeMaxM && ...
        double(m.steady_Va_rms_mps) <= opts.PassAirspeedRmsMps && ...
        double(m.steady_vz_rms_mps) <= opts.PassVzRmsMps && ...
        double(m.steady_q_rms_dps) <= opts.PassQRmsDps && ...
        abs(double(m.tail_h_error_m)) <= opts.PassTailAltitudeErrorM && ...
        abs(double(m.tail_vz_mps)) <= opts.PassTailVzMps;
    tf = logical(driftNear && otherPass);
catch
    tf = false;
end
end

function local_run_integral_seed_probes(ctx,parallelEnabled,opts)
base = ctx.checkpoint.best_candidate{ctx.cfgId+1};
base = local_upgrade_candidate(base);
gains = double(opts.NearPassIntegralSeedGains(:));
gains = gains(isfinite(gains) & gains > 0);
if isempty(gains), return; end
X = cell(numel(gains),1);
for i = 1:numel(gains)
    c = base;
    c.HeightIntegralGain = gains(i);
    X{i} = local_candidate_to_x(c);
end

if logical(parallelEnabled) && numel(gains) > 1
    scores = NaN(numel(gains),1); %#ok<NASGU>
    parfor i = 1:numel(gains)
        scores(i) = local_objective(X{i},ctx); %#ok<PFBNS>
    end
else
    for i = 1:numel(gains)
        local_objective(X{i},ctx);
    end
end
end

function X = local_candidate_to_x(c)
c = local_upgrade_candidate(c);
X = table(c.Np,c.Nc,c.Wh,c.Wvz,c.Wq,c.RateScale,c.Authority, ...
    c.HeightToVzGain,c.HeightIntegralGain,c.HeightVzLimit, ...
    'VariableNames',{'Np','Nc','Wh','Wvz','Wq','RateScale','Authority', ...
    'HeightToVzGain','HeightIntegralGain','HeightVzLimit'});
end

function vars = local_optimizable_variables()
vars = [ ...
    optimizableVariable("Np",[6 14],"Type","integer")
    optimizableVariable("Nc",[2 4],"Type","integer")
    optimizableVariable("Wh",[0.20 10.0],"Transform","log")
    optimizableVariable("Wvz",[3.0 40.0],"Transform","log")
    optimizableVariable("Wq",[2.0 20.0],"Transform","log")
    optimizableVariable("RateScale",[0.50 3.0],"Transform","log")
    optimizableVariable("Authority",[0.55 1.00])
    optimizableVariable("HeightToVzGain",[0.04 0.35],"Transform","log")
    optimizableVariable("HeightIntegralGain",[0.0 0.02])
    optimizableVariable("HeightVzLimit",[0.40 1.50])
    ];
end

function c = local_candidate_from_x(x,opts)
c = struct();
c.Np = round(double(x.Np));
c.Nc = min(round(double(x.Nc)),c.Np);
c.Wh = double(x.Wh);
c.Wvz = double(x.Wvz);
c.Wq = double(x.Wq);
c.RateScale = double(x.RateScale);
c.Authority = double(x.Authority);
c.HeightToVzGain = double(x.HeightToVzGain);
c.HeightIntegralGain = double(x.HeightIntegralGain);
c.HeightVzLimit = double(x.HeightVzLimit);
c.Wva = double(opts.FixedAirspeedWeight);
c.Wpitch = double(opts.FixedPitchWeight);
c.MVWeights = double(opts.MVWeights(:)).';
c.MVRateWeights = double(opts.MVRateWeights(:)).' * c.RateScale;
end

function c = local_candidate_from_history_row(r,opts)
x = table();
x.Np = double(r.Np);
x.Nc = double(r.Nc);
x.Wh = double(r.Wh);
x.Wvz = double(r.Wvz);
x.Wq = double(r.Wq);
x.RateScale = double(r.RateScale);
x.Authority = double(r.Authority);
x.HeightToVzGain = double(r.HeightToVzGain);
if ismember("HeightIntegralGain",string(r.Properties.VariableNames))
    x.HeightIntegralGain = double(r.HeightIntegralGain);
else
    x.HeightIntegralGain = 0.0;
end
x.HeightVzLimit = double(r.HeightVzLimit);
c = local_candidate_from_x(x,opts);
end

function c = local_upgrade_candidate(c)
if isempty(c), return; end
if ~isfield(c,"HeightIntegralGain") || isempty(c.HeightIntegralGain) || ~isfinite(double(c.HeightIntegralGain))
    c.HeightIntegralGain = 0.0;
end
end

function sig = local_candidate_signature(c)
c = local_upgrade_candidate(c);
sig = sprintf("%d|%d|%.10g|%.10g|%.10g|%.10g|%.10g|%.10g|%.10g|%.10g", ...
    round(c.Np),round(c.Nc),c.Wh,c.Wvz,c.Wq,c.RateScale,c.Authority, ...
    c.HeightToVzGain,c.HeightIntegralGain,c.HeightVzLimit);
end

function v = local_candidate_vector(c)
c = local_upgrade_candidate(c);
v = [c.Np c.Nc c.Wh c.Wvz c.Wq c.RateScale c.Authority ...
    c.HeightToVzGain c.HeightIntegralGain c.HeightVzLimit];
end

function T = local_candidate_table(cfgId,c,obj)
c = local_upgrade_candidate(c);
T = table(cfgId,c.Np,c.Nc,c.Wh,c.Wvz,c.Wq,c.RateScale,c.Authority, ...
    c.HeightToVzGain,c.HeightIntegralGain,c.HeightVzLimit,obj, ...
    'VariableNames',{'config_id','Np','Nc','Wh','Wvz','Wq','RateScale','Authority', ...
    'HeightToVzGain','HeightIntegralGain','HeightVzLimit','objective'});
end

function local_write_eval_record(evalRoot,cfgId,c,score,tag,M)
c = local_upgrade_candidate(c);
hard = false; formal = false; hRms = NaN; hDrift = NaN; tailH = NaN; tailVz = NaN;
if ~isempty(M) && height(M) >= 1
    try
        hard = logical(M.hard_fail(1)); formal = logical(M.formal_pass(1));
        hRms = double(M.steady_h_rms_m(1)); hDrift = double(M.steady_h_drift_m(1));
        tailH = double(M.tail_h_error_m(1)); tailVz = double(M.tail_vz_mps(1));
    catch
    end
end
row = table(string(tag),datetime("now"),cfgId,c.Np,c.Nc,c.Wh,c.Wvz,c.Wq,c.RateScale, ...
    c.Authority,c.HeightToVzGain,c.HeightIntegralGain,c.HeightVzLimit,double(score), ...
    "v20",hard,formal,hRms,hDrift,tailH,tailVz, ...
    'VariableNames',{'eval_tag','timestamp','config_id','Np','Nc','Wh','Wvz','Wq', ...
    'RateScale','Authority','HeightToVzGain','HeightIntegralGain','HeightVzLimit', ...
    'objective','source','hard_fail','formal_pass','h_rms_m','h_drift_m','tail_h_error_m','tail_vz_mps'});
try
    writetable(row,fullfile(evalRoot,"evaluation_record.csv"));
catch
end
end

function H = local_consolidate_history(cfgRoot,historyCsv,opts)
H = table();

% Import old v16 history exactly once, preserving a backup.
if isfile(historyCsv)
    try
        H = readtable(historyCsv);
        if ~ismember("HeightIntegralGain",string(H.Properties.VariableNames))
            backup = fullfile(fileparts(historyCsv),"optimization_history_v16_backup.csv");
            if ~isfile(backup), copyfile(historyCsv,backup); end
            H.HeightIntegralGain = zeros(height(H),1);
        end
        if ~ismember("source",string(H.Properties.VariableNames))
            H.source = repmat("legacy",height(H),1);
        end

        % Re-score legacy trajectories from their saved metrics.  This changes
        % the objective weights without repeating a single JSBSim simulation.
        for i = 1:height(H)
            try
                tag = string(H.eval_tag(i));
                metricFile = fullfile(cfgRoot,"optimization",tag,"candidate_metrics.csv");
                if isfile(metricFile)
                    M = readtable(metricFile);
                    H.objective(i) = local_score_from_metrics(M,opts);
                end
            catch
            end
        end
    catch
        H = table();
    end
end

% Parallel workers write unique record files; collect them only on the client.
files = dir(fullfile(cfgRoot,"optimization","**","evaluation_record.csv"));
for i = 1:numel(files)
    try
        R = readtable(fullfile(files(i).folder,files(i).name));
        R = local_history_core(R);
        if isempty(H)
            H = R;
        else
            H = [local_history_core(H); R]; %#ok<AGROW>
        end
    catch
    end
end

if isempty(H)
    H = local_empty_history();
    writetable(H,historyCsv);
    return;
end
H = local_history_core(H);
valid = isfinite(H.objective);
H = H(valid,:);
if isempty(H)
    H = local_empty_history();
    writetable(H,historyCsv);
    return;
end

% Deduplicate deterministic points.  v16 rerun2 contains repeated points; keep
% only the best recorded value for each exact candidate signature.
sig = strings(height(H),1);
for i = 1:height(H)
    sig(i) = local_candidate_signature(local_candidate_from_history_row(H(i,:),opts));
end
[~,~,g] = unique(sig,"stable");
keep = false(height(H),1);
for k = 1:max(g)
    idx = find(g==k);
    [~,j] = min(H.objective(idx));
    keep(idx(j)) = true;
end
H = H(keep,:);
H = sortrows(H,"objective","ascend");
writetable(H,historyCsv);
end

function H = local_history_core(H)
names = ["eval_tag","timestamp","config_id","Np","Nc","Wh","Wvz","Wq","RateScale", ...
    "Authority","HeightToVzGain","HeightIntegralGain","HeightVzLimit","objective","source"];
for n = names
    if ismember(n,string(H.Properties.VariableNames)), continue; end
    switch n
        case "eval_tag"
            H.eval_tag = strings(height(H),1);
        case "timestamp"
            H.timestamp = repmat(datetime("now"),height(H),1);
        case "source"
            H.source = repmat("legacy",height(H),1);
        case "HeightIntegralGain"
            H.HeightIntegralGain = zeros(height(H),1);
        otherwise
            H.(char(n)) = NaN(height(H),1);
    end
end
% Normalize types so legacy v16 CSV rows and parallel v17/v18 record rows can
% concatenate safely.
H.eval_tag = string(H.eval_tag);
H.source = string(H.source);
try
    H.timestamp = datetime(H.timestamp);
catch
    H.timestamp = repmat(datetime("now"),height(H),1);
end
numericNames = ["config_id","Np","Nc","Wh","Wvz","Wq","RateScale","Authority", ...
    "HeightToVzGain","HeightIntegralGain","HeightVzLimit","objective"];
for n = numericNames
    H.(char(n)) = double(H.(char(n)));
end
H = H(:,cellstr(names));
end

function H = local_empty_history()
H = table(strings(0,1),NaT(0,1),zeros(0,1),zeros(0,1),zeros(0,1), ...
    zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1), ...
    zeros(0,1),zeros(0,1),zeros(0,1),strings(0,1), ...
    'VariableNames',{'eval_tag','timestamp','config_id','Np','Nc','Wh','Wvz','Wq', ...
    'RateScale','Authority','HeightToVzGain','HeightIntegralGain','HeightVzLimit','objective','source'});
end

function [X,y] = local_prior_observations(H,opts)
X = table(); y = [];
if isempty(H), return; end
H = H(isfinite(H.objective),:);
if isempty(H), return; end
H = sortrows(H,"objective","ascend");
n = min(double(opts.ResumePriorPointCount),height(H));
H = H(1:n,:);
X = H(:,{'Np','Nc','Wh','Wvz','Wq','RateScale','Authority', ...
    'HeightToVzGain','HeightIntegralGain','HeightVzLimit'});
y = double(H.objective(:));
end

function [checkpoint,bestMetrics,passed] = local_try_history_rescue( ...
    paths,master,checkpoint,cfgId,H,physicalNominals,hiddenTrim,cfgRoot,opts)
bestMetrics = table(); passed = false;
if ~logical(opts.TryHistoryRescueBeforeSearch) || isempty(H), return; end
[checkpoint,bestMetrics,passed] = local_certify_history_candidates( ...
    paths,master,checkpoint,cfgId,H,physicalNominals,hiddenTrim,cfgRoot, ...
    "history_rescue",opts.HistoryRescueTopK,opts);
end

function [checkpoint,bestMetrics,passed] = local_certify_top_history( ...
    paths,master,checkpoint,cfgId,H,physicalNominals,hiddenTrim,cfgRoot,opts)
[checkpoint,bestMetrics,passed] = local_certify_history_candidates( ...
    paths,master,checkpoint,cfgId,H,physicalNominals,hiddenTrim,cfgRoot, ...
    "post_search_cert",opts.CertificationTopK,opts);
end

function [checkpoint,bestMetrics,passed] = local_certify_history_candidates( ...
    paths,master,checkpoint,cfgId,H,physicalNominals,hiddenTrim,cfgRoot,prefix,topK,opts)
bestMetrics = table(); passed = false;
if isempty(H), return; end
H = sortrows(H,"objective","ascend");
attempted = string(checkpoint.failed_certified_signatures{cfgId+1});
bestLongScore = Inf;
bestC = [];
bestObj = Inf;
bestBankSource = "";

nDone = 0;
for i = 1:height(H)
    c = local_candidate_from_history_row(H(i,:),opts);
    sig = string(local_candidate_signature(c));
    if any(attempted == sig), continue; end
    nDone = nDone + 1;
    if nDone > double(topK), break; end

    certRoot = fullfile(cfgRoot,sprintf("%s_%02d",prefix,nDone));
    if ~isfolder(certRoot), mkdir(certRoot); end
    bankMat = fullfile(certRoot,"certification_bank.mat");
    local_build_combined_bank(master,checkpoint,cfgId,c,physicalNominals,bankMat,opts);
    M = local_run_certification(paths,master,checkpoint,cfgId,c, ...
        physicalNominals,hiddenTrim,bankMat,certRoot, ...
        string(prefix)+"_"+string(nDone),opts.FinalWindowS,opts);
    writetable(M,fullfile(certRoot,"certification_summary.csv"));
    checkpoint.verification_count(cfgId+1) = checkpoint.verification_count(cfgId+1) + 1;

    longScore = local_score_from_metrics(M,opts);
    if longScore < bestLongScore
        bestLongScore = longScore;
        bestMetrics = M;
        bestC = c;
        bestObj = double(H.objective(i));
        bestBankSource = string(bankMat);
    end

    if logical(M.formal_pass) && ~logical(M.hard_fail)
        passed = true;
        bestMetrics = M; bestC = c; bestObj = double(H.objective(i));
        bestBankSource = string(bankMat);
        break;
    else
        attempted(end+1,1) = sig; %#ok<AGROW>
    end
end

checkpoint.failed_certified_signatures{cfgId+1} = unique(attempted,"stable");
if ~isempty(bestC)
    checkpoint.best_candidate{cfgId+1} = bestC;
    checkpoint.best_objective(cfgId+1) = bestObj;
    bestBank = fullfile(cfgRoot,"best_mpc_bank_200m.mat");
    if strlength(bestBankSource) > 0 && isfile(bestBankSource)
        copyfile(bestBankSource,bestBank);
    else
        local_build_combined_bank(master,checkpoint,cfgId,bestC,physicalNominals,bestBank,opts);
    end
    checkpoint.best_bank_path(cfgId+1) = string(bestBank);
    writetable(local_candidate_table(cfgId,bestC,bestObj),fullfile(cfgRoot,"best_candidate.csv"));
end
end

% ========================================================================
% Physical nominal / hidden trim
% ========================================================================
function hiddenTrim = local_calibrate_hidden_trim(paths,trim0,opts,outDir)
cal = airdropx_auto_run_id_experiment( ...
    "ProjectRoot",paths.projectRoot,"OutputRoot",outDir,"RunId","v16_hidden_trim_200m", ...
    "ConfigId",0,"Trim",trim0,"StopTimeS",0.5,"RecordStartS",0.0,"ExportStartS",0.0, ...
    "ExcitationStartS",100.0,"ElevatorAmplitude",0.0,"ThrottleAmplitude",0.0, ...
    "DirectIdMode",true,"KeepFixedConfigurationOnly",true, ...
    "InitialAltitudeM",opts.TargetAltitudeM,"InitialAirspeedMps",opts.TargetAirspeedMps, ...
    "InitialPitchDeg",trim0.pitch_deg,"InitialFlightPathDeg",0.0, ...
    "TargetAltitudeM",opts.TargetAltitudeM,"TargetAirspeedMps",opts.TargetAirspeedMps, ...
    "ReferenceMassKg",opts.ReferenceMassKg,"CargoMassKg",opts.CargoMassKg);
T = cal.timeseries;
external = local_col(T,"requested_elevator_cmd");
physical = local_col(T,"elevator_cmd_norm");
t = local_col(T,"time_s");
mask = isfinite(external) & isfinite(physical) & isfinite(t) & t <= 0.25;
if nnz(mask) < 3, error("Hidden trim calibration did not produce enough samples."); end
hiddenTrim = median(physical(mask)-external(mask),"omitnan");
fprintf("[V21-200m] hidden elevator trim = %.6f\n",hiddenTrim);
end

function p = local_physical_nominals(master,hiddenTrim)
p = NaN(5,1);
for cfgId = 0:4
    if cfgId + 1 > numel(master.trim_bank), continue; end
    % A freshly rebuilt cfg stores an explicit physical nominal on its trim.
    % Prefer it over historical master.data csv_files from the original v11
    % identification package.
    explicitPhysical = local_trim_field(master.trim_bank(cfgId+1), "physical_elevator_cmd", NaN);
    if isfinite(explicitPhysical)
        p(cfgId+1) = explicitPhysical;
        continue;
    end
    % Otherwise try clean-ID physical input data when available.
    vals = [];
    try
        files = string(master.data.csv_files(:));
        for i = 1:numel(files)
            if ~isfile(files(i)), continue; end
            T = readtable(files(i));
            if ~ismember("config_id",string(T.Properties.VariableNames)) || ...
                    ~ismember("elevator_cmd_norm",string(T.Properties.VariableNames))
                continue;
            end
            if round(median(double(T.config_id),"omitnan")) ~= cfgId, continue; end
            v = double(T.elevator_cmd_norm);
            if ismember("elevator_excitation",string(T.Properties.VariableNames))
                ex = abs(double(T.elevator_excitation));
                idx = find(ex > 1e-7,1,"first");
                if ~isempty(idx) && idx > 4, v = v(1:idx-1); end
            end
            v = v(isfinite(v));
            if ~isempty(v), vals(end+1,1) = median(v,"omitnan"); end %#ok<AGROW>
        end
    catch
    end
    if ~isempty(vals)
        p(cfgId+1) = median(vals,"omitnan");
    else
        ext = local_trim_field(master.trim_bank(cfgId+1),"elevator_cmd",NaN);
        if isfinite(ext), p(cfgId+1) = hiddenTrim + ext; end
    end
end
end

function T = local_nominal_table(master,p,hiddenTrim)
cfg = (0:4).'; ext = NaN(5,1); throttle = NaN(5,1); pitch = NaN(5,1);
for k = 1:5
    if k <= numel(master.trim_bank)
        ext(k) = local_trim_field(master.trim_bank(k),"elevator_cmd",NaN);
        throttle(k) = local_trim_field(master.trim_bank(k),"throttle_cmd",NaN);
        pitch(k) = local_trim_field(master.trim_bank(k),"pitch_deg",NaN);
    end
end
T = table(cfg,p(:),repmat(hiddenTrim,5,1),ext,throttle,pitch, ...
    'VariableNames',{'config_id','physical_elevator_nominal','hidden_elevator_trim', ...
    'external_trim_delta','throttle_nominal','pitch_nominal_deg'});
end

% ========================================================================
% Checkpoint/master persistence
% ========================================================================
function checkpoint = local_load_checkpoint(file)
checkpoint = local_new_checkpoint();
if ~isfile(file), return; end
try
    S = load(file,"checkpoint");
    if isfield(S,"checkpoint"), checkpoint = local_merge_checkpoint(checkpoint,S.checkpoint); end
    fprintf("[V21-200m] checkpoint loaded: %s\n",file);
catch ME
    warning("Checkpoint load failed: %s",ME.message);
end
end

function cp = local_new_checkpoint()
cp = struct();
cp.version = 29;
cp.mission_signature = "";
cp.status = repmat("pending",5,1);
cp.best_candidate = cell(5,1);
cp.best_objective = Inf(5,1);
cp.best_bank_path = strings(5,1);
cp.last_metrics = cell(5,1);
cp.plant_ready = false(5,1);
cp.plant_validation = cell(5,1);
cp.verification_count = zeros(5,1);
cp.failed_certified_signatures = cell(5,1);
cp.plant_generation = ones(5,1);
cp.plant_rebuild_count = zeros(5,1);
cp.equilibrium_probe_pass = false(5,1);
cp.equilibrium_probe_generation = zeros(5,1);
cp.stageA_candidate = cell(5,1);
cp.stageB_candidate = cell(5,1);
cp.tuning_stage = repmat("pending",5,1);
cp.hidden_elevator_trim = NaN;
cp.all_verified = false;
cp.final_mission_attempted = false;
cp.final_mission_pass = false;
cp.final_mission_summary = table();
cp.final_mission_updated_at = "";
% v30.6 learned mission-transition policy. The same policy mechanism is used
% for every H/V context; values are context results, not hard-coded cases.
cp.transition_move_transfer_scale = NaN;
cp.transition_integral_transfer_scale = NaN;
cp.transition_policy_source = "";
cp.master_source = "";
cp.updated_at = string(datetime("now"));
end

function cp = local_merge_checkpoint(cp,old)
fields = fieldnames(cp);
for i = 1:numel(fields)
    f = fields{i};
    if isfield(old,f), cp.(f) = old.(f); end
end
cp.version = 29;
if ~isfield(cp,"mission_signature"), cp.mission_signature=""; end
if ~isfield(cp,"final_mission_attempted"), cp.final_mission_attempted=false; end
if ~isfield(cp,"final_mission_pass"), cp.final_mission_pass=false; end
if ~isfield(cp,"final_mission_summary"), cp.final_mission_summary=table(); end
if ~isfield(cp,"final_mission_updated_at"), cp.final_mission_updated_at=""; end
if ~isfield(cp,"transition_move_transfer_scale"), cp.transition_move_transfer_scale=NaN; end
if ~isfield(cp,"transition_integral_transfer_scale"), cp.transition_integral_transfer_scale=NaN; end
if ~isfield(cp,"transition_policy_source"), cp.transition_policy_source=""; end
% Defensive resize for older/partial checkpoints.
cp.status = local_resize_string(cp.status,5,"pending");
cp.best_candidate = local_resize_cell(cp.best_candidate,5);
cp.best_objective = local_resize_numeric(cp.best_objective,5,Inf);
cp.best_bank_path = local_resize_string(cp.best_bank_path,5,"");
cp.last_metrics = local_resize_cell(cp.last_metrics,5);
cp.plant_ready = logical(local_resize_numeric(cp.plant_ready,5,0));
cp.plant_validation = local_resize_cell(cp.plant_validation,5);
cp.verification_count = local_resize_numeric(cp.verification_count,5,0);
cp.failed_certified_signatures = local_resize_cell(cp.failed_certified_signatures,5);
cp.plant_generation = local_resize_numeric(cp.plant_generation,5,1);
cp.plant_generation(cp.plant_generation < 1) = 1;
cp.plant_rebuild_count = local_resize_numeric(cp.plant_rebuild_count,5,0);
cp.equilibrium_probe_pass = logical(local_resize_numeric(cp.equilibrium_probe_pass,5,0));
cp.equilibrium_probe_generation = local_resize_numeric(cp.equilibrium_probe_generation,5,0);
cp.stageA_candidate = local_resize_cell(cp.stageA_candidate,5);
cp.stageB_candidate = local_resize_cell(cp.stageB_candidate,5);
cp.tuning_stage = local_resize_string(cp.tuning_stage,5,"pending");
for k = 1:5
    if ~isempty(cp.best_candidate{k}), cp.best_candidate{k} = local_upgrade_candidate(cp.best_candidate{k}); end
end
end

function local_save_checkpoint(file,checkpoint)
parent = fileparts(file); if ~isfolder(parent), mkdir(parent); end
save(file,"checkpoint","-v7.3");
end

function [master,source] = local_load_master(baseMat,masterFile)
if isfile(masterFile)
    S = load(masterFile,"result");
    master = S.result; source = string(masterFile);
else
    S = load(baseMat,"result");
    master = S.result; source = string(baseMat);
end
if ~isfield(master,"plant_bank") || ~isfield(master,"trim_bank")
    error("Identified result must contain plant_bank and trim_bank.");
end
end

function local_save_master(file,master)
result = master; %#ok<NASGU>
plant_bank = master.plant_bank; %#ok<NASGU>
save(file,"result","plant_bank","-v7.3");
end

function tf = local_plant_missing(master,cfgId)
tf = cfgId + 1 > numel(master.plant_bank) || isempty(master.plant_bank{cfgId+1});
end

% ========================================================================
% Misc metrics/plot/helpers
% ========================================================================
function tReach = local_config_reach_time(cfgId,opts)
if cfgId <= 0, tReach = 0.0; else, tReach = opts.DropStartS + opts.DropIntervalS*(cfgId-1); end
end

function rec = local_recovery_time(t,hErr,vz,pulseStart,opts)
rec = NaN;
if ~isfinite(pulseStart), return; end
for i = find(t >= pulseStart).'
    t0 = t(i);
    mask = t >= t0 & t <= t0 + opts.RecoveryHoldS;
    if nnz(mask) < 5, continue; end
    if all(abs(hErr(mask)) <= opts.RecoveryAltitudeBandM) && ...
            all(abs(vz(mask)) <= opts.RecoveryVzBandMps)
        rec = t0 - pulseStart;
        return;
    end
end
end

function p = local_recovery_penalty(rec,opts)
if isfinite(rec), p = opts.RecoveryPenaltyWeight*rec; else, p = opts.RecoveryPenaltyWeight*opts.RecoveryNoPassPenaltyS; end
end

function row = local_summary_row(cfgId,status,M,checkpoint)
row = table(cfgId,string(status),logical(M.formal_pass),logical(M.hard_fail), ...
    M.steady_h_rms_m,M.steady_Va_rms_mps,M.tail_h_error_m,M.tail_vz_mps, ...
    checkpoint.best_objective(cfgId+1),checkpoint.verification_count(cfgId+1), ...
    'VariableNames',{'config_id','status','formal_pass','hard_fail','h_rms_m', ...
    'Va_rms_mps','tail_h_error_m','tail_vz_mps','best_objective','verification_count'});
end

function local_plot(T,eNom,tNom,outFile,plotTitle)
fig = figure('Visible','off','Color','w','Position',[100 100 1300 1000]);
tl = tiledlayout(7,1,'Padding','compact','TileSpacing','compact');
t = T.time_s;
nexttile; plot(t,T.altitude_m); hold on; plot(t,T.target_altitude_m,'--'); grid on; ylabel('h m');
nexttile; plot(t,T.airspeed_mps); hold on; plot(t,T.target_airspeed_mps,'--'); grid on; ylabel('Va');
nexttile; plot(t,T.pitch_deg); grid on; ylabel('pitch');
nexttile; plot(t,T.vz_up_mps); yline(0,'--'); grid on; ylabel('vz');
nexttile; plot(t,T.q_dps); yline(0,'--'); grid on; ylabel('q');
nexttile; plot(t,T.elevator_cmd_norm); hold on; yline(eNom,'--'); grid on; ylabel('elev');
nexttile; plot(t,T.throttle_norm); hold on; yline(tNom,'--'); grid on; ylabel('thr'); xlabel('time s');
title(tl,plotTitle,'Interpreter','none');
exportgraphics(fig,outFile,'Resolution',150); close(fig);
end

function x = local_col(T,name)
if ismember(string(name),string(T.Properties.VariableNames)), x = double(T.(char(name))(:)); else, x = NaN(height(T),1); end
end

function v = local_rms(x)
x = double(x(:)); x = x(isfinite(x)); if isempty(x), v = NaN; else, v = sqrt(mean(x.^2)); end
end

function v = local_trim_field(s,name,fallback)
try
    v = double(s.(name));
    if isempty(v) || ~isscalar(v) || ~isfinite(v), v = fallback; end
catch
    v = fallback;
end
end

function value = local_cfg_value(v,cfgId,fallback)
v = double(v(:));
if cfgId+1 <= numel(v) && isfinite(v(cfgId+1)), value = round(v(cfgId+1)); else, value = round(fallback); end
end
function value = local_cfg_double(v,cfgId,fallback)
v = double(v(:));
if cfgId+1 <= numel(v) && isfinite(v(cfgId+1)), value = double(v(cfgId+1)); else, value = double(fallback); end
end

function out = local_resize_string(x,n,fill)
out = repmat(string(fill),n,1); x = string(x(:)); out(1:min(n,numel(x))) = x(1:min(n,numel(x)));
end
function out = local_resize_cell(x,n)
out = cell(n,1); if iscell(x), out(1:min(n,numel(x))) = x(1:min(n,numel(x))); end
end
function out = local_resize_numeric(x,n,fill)
out = repmat(double(fill),n,1); x = double(x(:)); out(1:min(n,numel(x))) = x(1:min(n,numel(x)));
end


function enabled = local_prepare_parallel_pool(paths,opts)
enabled = false;
if ~logical(opts.UseParallel), return; end
n = min(max(round(double(opts.ParallelWorkers)),1),round(double(opts.MaxParallelWorkers)));
if n <= 1, return; end
try
    p = gcp("nocreate");
    if ~isempty(p) && p.NumWorkers ~= n
        delete(p);
        p = [];
    end
    if isempty(p)
        extra = {paths.matlabDir,paths.mpcDir,paths.autoDir,paths.sfuncDir};
        p = parpool("Processes",n,"AdditionalPaths",extra);
    end
    enabled = ~isempty(p) && p.NumWorkers > 1;
    if enabled
        fprintf("[V21-200m] parallel pool ready: %d process workers (hard cap %d).\n", ...
            p.NumWorkers,opts.MaxParallelWorkers);
    end
catch ME
    warning("AirdropX:AutoMPC200:ParallelUnavailable", ...
        "Could not start process pool; falling back to serial: %s",ME.message);
    enabled = false;
end
end

function n = local_parallel_worker_count(enabled,opts)
if logical(enabled)
    n = min(max(round(double(opts.ParallelWorkers)),1),round(double(opts.MaxParallelWorkers)));
else
    n = 1;
end
end

function local_prepare_worker_filegen(ctx)
% Every process worker gets private Simulink cache/code-generation folders.
% This avoids slprj/cache collisions when up to three JSBSim simulations run
% concurrently from the same project.
persistent lastKey
wid = 0;
try
    task = getCurrentTask();
    if ~isempty(task), wid = double(task.ID); end
catch
end
baseRoot = string(getenv("AIRDROPX_FILEGEN_ROOT"));
if strlength(baseRoot)==0
    % Short fallback. The launcher sets AIRDROPX_FILEGEN_ROOT to a much
    % shorter drive-root path (for example D:\AXC\r123abc).
    baseRoot = string(fullfile(tempdir,"AXC"));
end
key = baseRoot + "|mpc_worker|" + string(wid);
if ~isempty(lastKey) && string(lastKey) == key, return; end
try
    cacheRoot = fullfile(baseRoot,sprintf("w%d",wid));
    cacheDir = fullfile(cacheRoot,"c");
    codegenDir = fullfile(cacheRoot,"g");
    if ~isfolder(cacheDir), mkdir(cacheDir); end
    if ~isfolder(codegenDir), mkdir(codegenDir); end
    Simulink.fileGenControl("set","CacheFolder",cacheDir, ...
        "CodeGenFolder",codegenDir,"createDir",true);
catch ME
    warning("AirdropX:AutoMPC200:WorkerFileGen", ...
        "Could not set short worker file-generation folders for worker %d: %s",wid,ME.message);
end
try
    addpath(ctx.paths.matlabDir);
    addpath(ctx.paths.mpcDir);
    addpath(ctx.paths.autoDir);
    addpath(ctx.paths.sfuncDir);
    cd(ctx.paths.projectRoot);
catch
end
lastKey = key;
end


% ========================================================================
% v30.6 universal bumpless transition + mission near-pass recovery
% ========================================================================
function [moveScale,integralScale,source] = local_checkpoint_transition_policy(checkpoint,bankRoot,opts)
moveScale = double(opts.TransitionMoveTransferScale);
integralScale = double(opts.TransitionIntegralTransferScale);
source="default_reset_baseline";
hasCheckpoint=false;
try
    vm=double(checkpoint.transition_move_transfer_scale);
    vi=double(checkpoint.transition_integral_transfer_scale);
    ps=""; try, ps=lower(string(checkpoint.transition_policy_source)); catch, end
    if isscalar(vm) && isscalar(vi) && isfinite(vm) && isfinite(vi) && contains(ps,"verified")
        moveScale=vm; integralScale=vi; hasCheckpoint=true; source="checkpoint_verified";
    end
catch
end
if ~hasCheckpoint
    [found,vm,vi]=local_nearest_verified_transition_policy(bankRoot,opts);
    if found
        moveScale=vm; integralScale=vi; source="learning_bank_nearest";
    end
end
moveScale=min(max(moveScale,0.0),1.5);
integralScale=min(max(integralScale,0.0),2.0);
end

function [found,moveScale,integralScale]=local_nearest_verified_transition_policy(bankRoot,opts)
found=false; moveScale=NaN; integralScale=NaN;
f=fullfile(bankRoot,"verified_transition_policies.csv");
if ~isfile(f), return; end
try
    T=readtable(f,'TextType','string');
    req=["target_altitude_m","target_airspeed_mps","reference_mass_kg","cargo_mass_kg", ...
        "total_drop_count","move_transfer_scale","integral_transfer_scale","mission_pass"];
    if isempty(T) || ~all(ismember(req,string(T.Properties.VariableNames))), return; end
    pass=local_table_bool(T,"mission_pass"); T=T(pass,:); if isempty(T), return; end
    dH=(double(T.target_altitude_m)-double(opts.TargetAltitudeM))/100.0;
    dV=(double(T.target_airspeed_mps)-double(opts.TargetAirspeedMps))/10.0;
    dM=(double(T.reference_mass_kg)-double(opts.ReferenceMassKg))/max(300.0,abs(double(opts.ReferenceMassKg))*0.25);
    dC=(double(T.cargo_mass_kg)-double(opts.CargoMassKg))/max(100.0,abs(double(opts.CargoMassKg)));
    dN=(double(T.total_drop_count)-double(opts.TotalDropCount))/max(1.0,double(opts.TotalDropCount));
    d=sqrt(dH.^2+dV.^2+dM.^2+dC.^2+dN.^2); d(~isfinite(d))=Inf;
    [best,idx]=min(d); if isempty(idx) || ~isfinite(best), return; end
    moveScale=double(T.move_transfer_scale(idx)); integralScale=double(T.integral_transfer_scale(idx));
    found=isfinite(moveScale)&&isfinite(integralScale);
catch
    found=false;
end
end

function local_record_verified_transition_policy(bankRoot,opts,moveScale,integralScale,R,outRoot)
try
    if ~logical(R.mission_pass), return; end
    f=fullfile(bankRoot,"verified_transition_policies.csv");
    row=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")), ...
        double(opts.TargetAltitudeM),double(opts.TargetAirspeedMps),double(opts.ReferenceMassKg), ...
        double(opts.CargoMassKg),round(double(opts.TotalDropCount)),double(moveScale),double(integralScale), ...
        local_mission_result_gate_ratio(R),true,string(outRoot), ...
        'VariableNames',{'updated_at','target_altitude_m','target_airspeed_mps','reference_mass_kg','cargo_mass_kg', ...
        'total_drop_count','move_transfer_scale','integral_transfer_scale','gate_ratio','mission_pass','output_root'});
    if isfile(f)
        try T=readtable(f,'TextType','string'); catch, T=table(); end
    else
        T=table();
    end
    if isempty(T)
        T=row;
    else
        % Keep the best measured policy per exact mission context/policy pair.
        T=[T;row];
        key=string(round(double(T.target_altitude_m)*1e6)/1e6)+"|"+ ...
            string(round(double(T.target_airspeed_mps)*1e6)/1e6)+"|"+ ...
            string(round(double(T.reference_mass_kg)*1e3)/1e3)+"|"+ ...
            string(round(double(T.cargo_mass_kg)*1e3)/1e3)+"|"+ ...
            string(double(T.total_drop_count))+"|"+ ...
            string(round(double(T.move_transfer_scale)*1e6)/1e6)+"|"+ ...
            string(round(double(T.integral_transfer_scale)*1e6)/1e6);
        [uk,~,g]=unique(string(key),'stable'); keep=false(height(T),1);
        for j=1:numel(uk)
            ids=find(g==j); [~,ii]=min(double(T.gate_ratio(ids))); keep(ids(ii))=true;
        end
        T=T(keep,:);
    end
    writetable(T,f);
catch ME
    warning("AirdropX:V30_6:TransitionLearningBank","Could not record verified transition policy: %s",ME.message);
end
end

function R = local_run_final_mission_with_policy(paths,outRoot,opts,moveScale,integralScale,subdir)
R = airdropx_auto_final_mission_validation( ...
    "ProjectRoot", paths.projectRoot, ...
    "OutputRoot", outRoot, ...
    "ValidationSubdir", string(subdir), ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "ReferenceMassKg", opts.ReferenceMassKg, ...
    "CargoMassKg", opts.CargoMassKg, ...
    "TotalDropCount", opts.TotalDropCount, ...
    "MpcEnableTimeS", opts.MpcEnableTimeS, ...
    "DropStartS", opts.DropStartS, ...
    "DropIntervalS", opts.DropIntervalS, ...
    "PostFinalDropS", opts.FinalMissionPostDropS, ...
    "FinalTailWindowS", opts.FinalMissionTailWindowS, ...
    "ElevatorDeviationRateLimit", opts.ElevatorDeviationRateLimit, ...
    "ThrottleDeviationRateLimit", opts.ThrottleDeviationRateLimit, ...
    "TrustAirspeedMps", opts.TrustAirspeedMps, ...
    "TrustPitchDeg", opts.TrustPitchDeg, ...
    "TrustVzMps", opts.TrustVzMps, ...
    "TrustQDps", opts.TrustQDps, ...
    "MaxBridgeError", opts.MaxBridgeError, ...
    "HardMaxAltitudeErrorM", opts.HardMaxAltitudeErrorM, ...
    "HardMaxQRmsDps", opts.HardMaxQRmsDps, ...
    "HardMaxAbsPitchDeg", opts.HardMaxAbsPitchDeg, ...
    "PassMissionAltitudeRmsM", opts.FinalMissionPassAltitudeRmsM, ...
    "PassMissionAltitudeMaxM", opts.FinalMissionPassAltitudeMaxM, ...
    "PassMissionAltitudeDriftM", opts.FinalMissionPassAltitudeDriftM, ...
    "PassMissionAirspeedRmsMps", opts.FinalMissionPassAirspeedRmsMps, ...
    "PassMissionVzRmsMps", opts.FinalMissionPassVzRmsMps, ...
    "PassMissionQRmsDps", opts.FinalMissionPassQRmsDps, ...
    "PassTailAltitudeErrorM", opts.PassTailAltitudeErrorM, ...
    "PassTailVzMps", opts.PassTailVzMps, ...
    "PassTailQDps", opts.PassQRmsDps, ...
    "RecoveryAltitudeBandM", opts.RecoveryAltitudeBandM, ...
    "RecoveryVzBandMps", opts.RecoveryVzBandMps, ...
    "RecoveryHoldS", opts.RecoveryHoldS, ...
    "BumplessTransitionEnabled", logical(opts.BumplessTransitionEnabled), ...
    "TransitionMoveTransferScale", double(moveScale), ...
    "TransitionIntegralTransferScale", double(integralScale), ...
    "V31ContinuousControllerStateEnabled", logical(opts.V31ContinuousControllerStateEnabled));
end

function tf = local_mission_recovery_result_usable(R)
% A mission-only recovery generation is allowed to continue its bounded
% transition-policy search after one worsened candidate as long as the full
% mission completed as a REAL, non-hard-fail observation.
tf=false;
try
    if isempty(R) || ~isstruct(R) || ~isfield(R,"summary") || isempty(R.summary), return; end
    S=R.summary; vars=string(S.Properties.VariableNames);
    if ~ismember("hard_fail",vars), return; end
    if logical(S.hard_fail(1)), return; end
    if ismember("drops_completed",vars)
        if double(S.drops_completed(1)) < 4, return; end
    end
    tf=true;
catch
    tf=false;
end
end

function tf = local_mission_nearpass_needed(R,opts)
tf=false;
try
    if ~logical(opts.UniversalMissionNearPassEnabled), return; end
    if isempty(R) || ~isstruct(R) || ~isfield(R,"summary") || isempty(R.summary), return; end
    if logical(R.summary.hard_fail(1)) || logical(R.mission_pass), return; end
    ratio=local_mission_result_gate_ratio(R);
    tf=isfinite(ratio) && ratio>1.0 && ratio<=double(opts.UniversalMissionNearPassGateRatioMax);
catch
    tf=false;
end
end

function ratio = local_mission_result_gate_ratio(R)
ratio=Inf;
try
    if isfield(R,"mission_gate_ratio") && isfinite(double(R.mission_gate_ratio))
        ratio=double(R.mission_gate_ratio); return;
    end
    if isfield(R,"summary") && ~isempty(R.summary) && ...
            ismember("mission_gate_ratio",string(R.summary.Properties.VariableNames))
        v=double(R.summary.mission_gate_ratio(1));
        if isfinite(v), ratio=v; return; end
    end
    if isfield(R,"gate_report") && ~isempty(R.gate_report)
        G=R.gate_report;
        a=double(G.actual(3:end)); lim=double(G.limit(3:end));
        rr=a./max(lim,eps); rr=rr(isfinite(rr));
        if ~isempty(rr), ratio=max(rr); end
    end
catch
    ratio=Inf;
end
end

function tf = local_mission_result_better(A,B)
tf=false;
try
    if logical(A.mission_pass) && ~logical(B.mission_pass), tf=true; return; end
    if ~logical(A.mission_pass) && logical(B.mission_pass), return; end
    tf=local_mission_result_gate_ratio(A) < local_mission_result_gate_ratio(B)-1e-9;
catch
end
end

function [bestR,bestMove,bestIntegral,attempted] = local_universal_mission_nearpass_refinement( ...
    paths,outRoot,opts,startR,startMove,startIntegral)
% Same deterministic transition-policy refinement for every mission context.
% It never changes Plant/trim or individual cfg controller parameters.  Thus
% existing VERIFIED single-cfg certifications remain authoritative.
bestR=startR; bestMove=double(startMove); bestIntegral=double(startIntegral); attempted=false;
root=fullfile(outRoot,"final_mission_validation","mission_nearpass_refinement");
if ~isfolder(root), mkdir(root); end
histFile=fullfile(root,"policy_evaluations.csv");
roundFile=fullfile(root,"rounds.csv");
H=local_read_mission_policy_history(histFile);
startGate=local_mission_result_gate_ratio(startR);

move=double(opts.UniversalMissionNearPassMoveScales(:));
integ=double(opts.UniversalMissionNearPassIntegralScales(:));
n=min(numel(move),numel(integ));
move=move(1:n); integ=integ(1:n);
maxNew=max(0,round(double(opts.UniversalMissionNearPassMaxNewEvaluationsPerAttempt)));
newCount=0;
for i=1:n
    if newCount>=maxNew, break; end
    m=move(i); q=integ(i);
    if abs(m-startMove)<1e-10 && abs(q-startIntegral)<1e-10, continue; end
    if local_mission_policy_seen(H,m,q), continue; end
    newCount=newCount+1; attempted=true;
    tag=sprintf("eval_%s_m%04d_i%04d",char(datetime("now","Format","yyyyMMdd_HHmmss_SSS")),round(1000*m),round(1000*q));
    subdir=fullfile("final_mission_validation","mission_nearpass_refinement",tag);
    fprintf("[V30.6-MISSION] policy %d/%d move=%.3f integral=%.3f full four-drop evaluation.\n",newCount,maxNew,m,q);
    try
        R=local_run_final_mission_with_policy(paths,outRoot,opts,m,q,subdir);
        gate=local_mission_result_gate_ratio(R);
        row=local_mission_policy_row(m,q,R,subdir,"ok");
        H=local_append_mission_policy_history(H,row);
        writetable(H,histFile);
        fprintf("[V30.6-MISSION] move=%.3f integral=%.3f gate=%.4f pass=%d.\n",m,q,gate,logical(R.mission_pass));
        if local_mission_result_better(R,bestR)
            bestR=R; bestMove=m; bestIntegral=q;
        end
        if logical(R.mission_pass), break; end
    catch ME
        row=local_mission_policy_error_row(m,q,subdir,ME.message);
        H=local_append_mission_policy_history(H,row);
        writetable(H,histFile);
        warning("AirdropX:V30_6:MissionPolicyEval", ...
            "Universal mission policy evaluation move=%.3f integral=%.3f failed to complete: %s",m,q,ME.message);
    end
end
endGate=local_mission_result_gate_ratio(bestR);
roundRow=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),startGate,endGate,newCount, ...
    logical(bestR.mission_pass),bestMove,bestIntegral, ...
    'VariableNames',{'updated_at','start_gate_ratio','end_gate_ratio','new_evaluations','mission_pass', ...
    'selected_move_transfer_scale','selected_integral_transfer_scale'});
if isfile(roundFile)
    try R0=readtable(roundFile,'TextType','string'); R0=[R0;roundRow]; writetable(R0,roundFile); catch, writetable(roundRow,roundFile); end
else
    writetable(roundRow,roundFile);
end
if local_mission_result_better(bestR,startR)
    local_promote_mission_result(bestR,outRoot,bestMove,bestIntegral);
end
end

function H=local_read_mission_policy_history(file)
H=table();
if ~isfile(file), return; end
try H=readtable(file,'TextType','string'); catch, H=table(); end
end

function tf=local_mission_policy_seen(H,m,q)
tf=false; if isempty(H), return; end
try
    mask=abs(double(H.move_transfer_scale)-m)<1e-10 & abs(double(H.integral_transfer_scale)-q)<1e-10;
    if ismember("status",string(H.Properties.VariableNames))
        mask=mask & lower(strtrim(string(H.status)))=="ok";
    end
    tf=any(mask);
catch
end
end

function row=local_mission_policy_row(m,q,R,subdir,status)
S=R.summary;
row=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),m,q, ...
    local_mission_result_gate_ratio(R),logical(R.mission_pass),logical(S.hard_fail(1)), ...
    double(S.mission_h_rms_m(1)),double(S.mission_h_max_abs_m(1)),double(S.mission_h_drift_m(1)), ...
    double(S.mission_Va_rms_mps(1)),double(S.mission_vz_rms_mps(1)),double(S.mission_q_rms_dps(1)), ...
    double(S.tail_h_error_m(1)),double(S.tail_vz_mps(1)),double(S.tail_q_dps(1)), ...
    string(subdir),string(status),"", ...
    'VariableNames',{'updated_at','move_transfer_scale','integral_transfer_scale','gate_ratio','mission_pass','hard_fail', ...
    'mission_h_rms_m','mission_h_max_abs_m','mission_h_drift_m','mission_Va_rms_mps','mission_vz_rms_mps','mission_q_rms_dps', ...
    'tail_h_error_m','tail_vz_mps','tail_q_dps','validation_subdir','status','error_message'});
end

function row=local_mission_policy_error_row(m,q,subdir,msg)
row=table(string(datetime("now","Format","yyyy-MM-dd HH:mm:ss")),m,q,Inf,false,true, ...
    NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,string(subdir),"error",string(msg), ...
    'VariableNames',{'updated_at','move_transfer_scale','integral_transfer_scale','gate_ratio','mission_pass','hard_fail', ...
    'mission_h_rms_m','mission_h_max_abs_m','mission_h_drift_m','mission_Va_rms_mps','mission_vz_rms_mps','mission_q_rms_dps', ...
    'tail_h_error_m','tail_vz_mps','tail_q_dps','validation_subdir','status','error_message'});
end

function H=local_append_mission_policy_history(H,row)
if isempty(H), H=row; return; end
try
    % Normalize legacy/string inference before append by writing through the
    % same column order. If types differ, rebuild from cell text is avoided;
    % this file is v30.6-only so the common case is homogeneous.
    H=[H;row];
catch
    H=row;
end
end

function local_promote_mission_result(R,outRoot,moveScale,integralScale)
% Promote the best measured near-pass candidate to the canonical final-mission
% folder so v30 status readers see the same authoritative result as checkpoint.
try
    src=char(string(R.output_root));
    dst=fullfile(outRoot,"final_mission_validation");
    if ~isfolder(dst), mkdir(dst); end
    files={"final_mission_summary.csv","final_mission_gate_report.csv","drop_transition_summary.csv","final_mission_curves.png","final_mission_result.mat"};
    for i=1:numel(files)
        f=fullfile(src,files{i}); if isfile(f), copyfile(f,fullfile(dst,files{i}),'f'); end
    end
    srcSim=fullfile(src,"simulation"); dstSim=fullfile(dst,"simulation");
    if isfolder(srcSim)
        if isfolder(dstSim), try rmdir(dstSim,'s'); catch, end, end
        copyfile(srcSim,dstSim,'f');
    end
    passFlag=fullfile(dst,"FINAL_MISSION_PASS.txt"); failFlag=fullfile(dst,"FINAL_MISSION_FAIL.txt");
    if logical(R.mission_pass)
        if isfile(failFlag), delete(failFlag); end
        srcFlag=fullfile(src,"FINAL_MISSION_PASS.txt"); if isfile(srcFlag), copyfile(srcFlag,passFlag,'f'); end
    else
        if isfile(passFlag), delete(passFlag); end
        srcFlag=fullfile(src,"FINAL_MISSION_FAIL.txt"); if isfile(srcFlag), copyfile(srcFlag,failFlag,'f'); end
    end
    P=table(double(moveScale),double(integralScale),local_mission_result_gate_ratio(R),logical(R.mission_pass), ...
        'VariableNames',{'move_transfer_scale','integral_transfer_scale','gate_ratio','mission_pass'});
    writetable(P,fullfile(dst,"selected_transition_policy.csv"));
catch ME
    warning("AirdropX:V30_6:PromoteMissionResult","Could not promote best mission result: %s",ME.message);
end
end

function paths = local_paths(projectRoot)
projectRoot = string(projectRoot);
if strlength(projectRoot) == 0
    thisDir = fileparts(mfilename("fullpath")); matlabDir = fileparts(thisDir); projectRoot = string(fileparts(matlabDir));
else
    matlabDir = fullfile(projectRoot,"matlab");
end
paths = struct("projectRoot",char(projectRoot),"matlabDir",char(matlabDir), ...
    "mpcDir",char(fullfile(matlabDir,"mpc")),"autoDir",char(fullfile(matlabDir,"mpc_auto")), ...
    "sfuncDir",char(fullfile(matlabDir,"sfunc_jsbsim")));
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.IdentifiedMat = "matlab/results/mpc_auto_id_v11_clean_r1/identify/airdropx_identified_plants.mat";
opts.OutputRoot = "matlab/results/mpc_auto_200m_all_cfg_v16";
opts.ConfigIds = (0:4).';
opts.TargetAltitudeM = 200.0;
opts.TargetAirspeedMps = 50.0;

% v29 mission/context variables. cfg is retained only as one context feature;
% the learner itself is identical for every configuration.
opts.ReferenceMassKg = 3423.0;
opts.CargoMassKg = 300.0;
opts.TotalDropCount = 4;
opts.ContextMassKgByConfig = NaN(5,1);
opts.ContextCgXByConfig = NaN(5,1);
opts.UnifiedLearning = true;
opts.LearningBankRoot = "matlab/results/mpc_auto_global_learning_bank";

% Unified transfer + full-certification Bayesian learning.
opts.UnifiedTransferSeedEvaluations = 5;
opts.UnifiedNearestVerifiedSeeds = 4;
opts.UnifiedNearestVerifiedPool = 12;
opts.UnifiedSameContextEvaluationSeeds = 3;
opts.UnifiedIdwNeighborCount = 5;
opts.UnifiedIdwDistanceFloor = 0.15;
opts.UnifiedGprMinVerifiedPoints = 6;
opts.UnifiedGprMinDistinctContexts = 4;
opts.UnifiedAdditionalEvaluationsPerRun = 36;
opts.UnifiedMaxSameContextPriors = 120;
opts.UnifiedMinWorkerUtilization = 3;
opts.UnifiedFormalStopObjective = 1.0;
opts.UnifiedHardFailLearningLoss = 8.0;
opts.UnifiedMultiViolationWeight = 0.20;

% v30.5 universal controller near-pass refinement. The classifier and search
% are identical for every altitude, airspeed, payload context and cfg.
opts.UnifiedControllerNearPassEnabled = true;
opts.UnifiedControllerNearPassGateRatioMax = 1.30;
opts.UnifiedControllerNearPassDeterministicEvaluations = 16;
opts.UnifiedControllerNearPassBayesEvaluations = 18;
opts.UnifiedControllerNearPassMaxRoundsPerContext = 2;
opts.UnifiedControllerNearPassKiMinUpperSpan = 8.0e-4;
% v31.1 layered learner controls.  Defaults preserve legacy v30 behavior;
% only the v31 state machine explicitly enables them for its LOCAL level.
opts.UnifiedForceControllerLocalRefinement = false;
opts.UnifiedV31LayeredLocalRefinement = false;

% v30.6 universal cfg-transition + mission near-pass recovery. No H/V/cfg
% identity appears in this policy: every complete mission uses the same
% bumpless bridge and the same measured gate-ratio classifier/refinement set.
opts.BumplessTransitionEnabled = true;
opts.TransitionMoveTransferScale = 0.0;
opts.TransitionIntegralTransferScale = 0.0;
opts.UniversalMissionNearPassEnabled = true;
opts.UniversalMissionNearPassGateRatioMax = 1.30;
opts.UniversalMissionNearPassMaxNewEvaluationsPerAttempt = 4;
opts.UniversalMissionNearPassMoveScales = [0.00;0.00;0.25;0.25;0.50;0.50;0.75;1.00];
opts.UniversalMissionNearPassIntegralScales = [0.00;0.50;0.00;0.50;0.00;0.50;0.50;1.00];
% v31: one continuous physical-command / global-vz-bias state across cfg changes.
% Disabled for legacy callers; the v31 state machine enables it explicitly.
opts.V31ContinuousControllerStateEnabled = false;
% v31.2 single-channel height governor. Direct altitude MPC tracking is
% suppressed structurally (effective Wh=0); these values are universal
% architecture settings, not H/cfg-specific rescue values.
opts.V31HeightGovernorEnabled = false;
opts.V31HeightVzSlewRateMps2 = 0.30;
opts.V31HeightBiasFraction = 0.70;
opts.V31HeightBiasLeak = 1.0;
opts.UnifiedV31ArchitectureRequal = false;

% Global performance GPR activates automatically once the cross-mission bank
% is large enough. It proposes extra transfer seeds; local bayesopt still
% performs the authoritative full certification.
opts.UnifiedGlobalGprMinEvaluations = 24;
opts.UnifiedGlobalGprMaxGateRatio = 6.0;
opts.UnifiedGlobalGprCandidatePool = 160;
opts.UnifiedGlobalGprSeedCount = 3;
opts.UnifiedGlobalGprExplorationBeta = 0.35;

% Unified controller parameter domain and context-adaptive local span.
opts.UnifiedNpRange = [5 18];
opts.UnifiedNcRange = [2 6];
opts.UnifiedNpHalfSpan = 3;
opts.UnifiedNcHalfSpan = 2;
opts.UnifiedWhRange = [0.10 15.0];
opts.UnifiedWvzRange = [1.5 60.0];
opts.UnifiedWqRange = [1.0 30.0];
opts.UnifiedRateScaleRange = [0.30 4.0];
opts.UnifiedAuthorityRange = [0.40 1.00];
opts.UnifiedKpRange = [0.02 0.40];
opts.UnifiedKiRange = [0.0 0.025];
opts.UnifiedVzLimitRange = [0.25 2.0];
opts.UnifiedWeightLowerFactor = 0.30;
opts.UnifiedWeightUpperFactor = 3.0;
opts.UnifiedRateLowerFactor = 0.45;
opts.UnifiedRateUpperFactor = 2.2;
opts.UnifiedAuthorityHalfSpan = 0.25;
opts.UnifiedKpLowerFactor = 0.35;
opts.UnifiedKpUpperFactor = 2.5;
opts.UnifiedMinKiSpan = 0.0020;
opts.UnifiedKiSpanFactor = 2.0;
opts.UnifiedVzLowerFactor = 0.50;
opts.UnifiedVzUpperFactor = 1.80;

opts.ReuseVerifiedBest = true;
opts.RecalibrateHiddenTrimEachRun = false;
opts.StopOnConfigFailure = true;
opts.UseParallel = true;
opts.ParallelWorkers = 3;
opts.MaxParallelWorkers = 3;
opts.BayesoptVerbose = 1;
% v18 requests NEW objective calls explicitly. Historical points are prior
% observations and are never re-simulated; the bayesopt total limit is
% priorCount + this configured NEW-call count.
opts.AdditionalObjectiveEvaluationsByConfig = [15;18;18;18;18];
opts.MaxObjectiveEvaluationsByConfig = [28;22;22;22;22]; % legacy, ignored
opts.ResumePriorPointCount = 200;
opts.ResumeBestPointCount = 6; % legacy, retained for old callers
opts.TryHistoryRescueBeforeSearch = true;
opts.HistoryRescueTopK = 3;
opts.CertificationTopK = 3;
opts.NearPassSeedCertificationTopK = 3;
opts.NearPassDriftMarginM = 0.15;
opts.NearPassIntegralSeedGains = [0.0015 0.0035 0.0075];
opts.PreviousCfg0HistoryCsv = "matlab/results/mpc_auto_mpc_200m_autotune_v15/optimization_history.csv";
opts.HardFailScore = 1e8;

% v20 cfg2+ diagnosis / model rebuild.
opts.StagedTuningMinConfig = 2;
opts.MaxAutoPlantRebuildsPerConfig = 1;
opts.PostDropProbeDurationS = 50.0;
opts.PostDropProbeSettleS = 20.0;
opts.PostDropProbeTailWindowS = 12.0;
opts.PostDropProbeMaxAbsHeightSlopeMps = 0.08;
opts.PostDropProbeMaxAbsVzMps = 0.10;
opts.PostDropProbeMaxQRmsDps = 0.25;
opts.PostDropProbeMaxAirspeedErrorMps = 1.0;

% v30.4 Universal Context Recovery. These thresholds apply identically to
% every mission altitude/airspeed and every cfg.  They do not encode any
% special case for a context that happened to fail during development.
opts.UniversalRecoveryNearPassGateRatioMax = 2.0;
opts.UniversalRecoveryExtendedProbeDurationS = 80.0;
opts.UniversalRecoveryExtendedTailWindowS = 20.0;
opts.UniversalRecoveryLocalRetrimEvaluations = 24;
opts.UniversalRecoveryLocalRetrimStopTimeS = 32.0;
opts.UniversalRecoveryElevatorHalfWidth = 0.045;
opts.UniversalRecoveryThrottleHalfWidth = 0.040;
opts.UniversalRecoveryPitchHalfWidthDeg = 0.75;
opts.UniversalRecoveryGammaHalfWidthDeg = 0.25;
opts.UniversalRecoveryRefineElevatorHalfWidth = 0.025;
opts.UniversalRecoveryRefineThrottleHalfWidth = 0.025;
opts.UniversalRecoveryRefinePitchHalfWidthDeg = 0.40;
opts.UniversalRecoveryRefineGammaHalfWidthDeg = 0.15;

% The new cfg trim/ID is intentionally stricter than v11 tail-equilibrium
% rescue: an MPC operating point must be a genuine long-horizon equilibrium,
% not merely a locally modelable slowly drifting trajectory.
opts.RetrimMaxAbsVzMps = 0.18;
opts.RetrimMaxTailAbsVzMps = 0.12;
opts.RetrimMaxTailHeightSlopeMps = 0.10;
opts.RetrimTailRescueMaxAbsVzMps = 0.12;
opts.RetrimTailRescueMaxHeightSlopeMps = 0.10;
% v20 permits the unavoidable short payload-release transient in the
% *full/early* trim trace, but the steady/tail gates remain strict around
% 200 m.  This prevents a real cfg2 equilibrium from being rejected merely
% because the aircraft moved briefly during the two preparatory drops.
opts.RetrimMaxFullAltitudeRmsM = 4.0;
opts.RetrimMaxFullAltitudeMaxAbsM = 7.0;
opts.RetrimMaxEarlyAltitudeRmsM = 4.0;
opts.RetrimMaxEarlyAltitudeMaxAbsM = 7.0;
opts.StrictBaselineDurationS = 8.0;
opts.StrictBaselineMaxAirspeedErrorMps = 1.0;
opts.StrictBaselineMaxPitchErrorDeg = 1.0;
opts.StrictBaselineMaxAbsVzMps = 0.10;
opts.StrictBaselineMaxAbsQDps = 0.20;
opts.StrictBaselineMaxHeightSlopeMps = 0.08;

% Staged cfg2+ tuning.  Kp is fixed and Ki=0 in Stage A.  Stage B tunes only
% Kp + vz limit after the inner learned MPC has proved it can control vertical
% motion.  Ki is reserved for Stage C final steady-error polish.
opts.StageAFixedHeightKp = 0.10;
opts.StageAHeightVzLimit = 1.00;
opts.StageAAdditionalEvaluationsByConfig = [0;0;12;12;12];
opts.StageBAdditionalEvaluationsByConfig = [0;0;9;9;9];
opts.StageAValidationWindowS = 18.0;
opts.StageAInnerMaxVzRmsMps = 0.55;
opts.StageAInnerMaxQRmsDps = 0.50;
opts.StageAInnerMaxTailVzMps = 0.30;
opts.StageAInnerMaxVaRmsMps = 1.0;
opts.StageAInnerMaxHeightErrorM = 8.0;
opts.StageBOuterMaxHeightRmsM = 2.0;
opts.StageBOuterMaxHeightErrorM = 4.0;
opts.StageBOuterMaxHeightDriftM = 2.5;
opts.StageBOuterMaxTailHeightErrorM = 2.0;
opts.StageBOuterMaxTailVzMps = 0.40;
opts.StageBOuterMaxQRmsDps = 0.60;
opts.StageCIntegralGains = [0 0.0015 0.0035 0.0075 0.012 0.018];
% v24: narrow final polish around an already near-passing coarse Ki.
% For Ki=0.0015 this evaluates approximately:
% 0.00075, 0.00105, 0.001275, 0.00135, 0.00165, 0.00177, 0.00195, 0.002175.
opts.StageCFineKiFactors = [0.50 0.70 0.85 0.90 1.10 1.18 1.30 1.45];
opts.StageCFineKiMax = 0.004;
opts.StageCFineNearPassAltitudeRmsM = 1.20;

% v26: once Stage C is already close, polish only Kp + Ki in a tiny 2-D box.
% Current cfg3 center is expected near Kp=0.1497, Ki=0.0015.
% This produces 5 x 3 = 15 NEW points (three batches with 5 workers):
% Kp approximately 0.1407, 0.1452, 0.1542, 0.1587, 0.1647
% Ki approximately 0.00135, 0.00150, 0.00165
opts.StageDTriggerGateRatio = 1.20;
opts.StageDKpFactors = [0.94 0.97 1.03 1.06 1.10];
opts.StageDKiFactors = [0.90 1.00 1.10];
opts.StageDKiFallback = [0.00075 0.00125 0.00175];
opts.StageDMinKp = 0.08;
opts.StageDMaxKp = 0.19;
opts.StageDMaxKi = 0.004;

% v27 adaptive Kp/Ki Gaussian-process Bayesian learning.
% Each invocation adds up to 24 genuinely NEW full certifications, but can
% stop early immediately after a formal-pass point is observed.
opts.StageEAdditionalEvaluationsPerRun = 24;
opts.StageEAbsoluteTriggerGateRatio = 1.50;
opts.StageEMinKp = 0.07;
opts.StageEMaxKp = 0.24;
opts.StageEMinPositiveKi = 0.00015;
opts.StageEMaxKi = 0.0060;
opts.StageEKpLowerFactor = 0.70;
opts.StageEKpUpperFactor = 1.40;
opts.StageEKiLowerFactor = 0.35;
opts.StageEKiUpperFactor = 2.50;
opts.StageEMinKiSpan = 0.0015;
opts.StageEMaxPriorPoints = 80;
opts.StageEMultiViolationWeight = 0.15;
opts.StageEFormalStopObjective = 1.0;
% Use at most five workers, but do not force all five busy with random points.
% With a 5-worker pool, 3 is a good quality/throughput compromise.
opts.StageEMinWorkerUtilization = 3;
opts.StageEPriorKpQuantization = 1e-6;
opts.StageEPriorKiQuantization = 1e-7;

% Missing cfg3/4 plant training.
% v20 sequential preparation freezes release pitch/gamma for cfg2+, making
% the actual retrim search effectively 2-D (elevator/throttle). Fewer
% evaluations are therefore sufficient and materially faster.
opts.TrimMaxObjectiveEvaluationsByConfig = [80;80;48;56;64];
% v30.1: keep low-altitude retrim acquisition safely high. Earlier v30
% accidentally overrode airdropx_auto_find_trim's 200 m SearchAltitudeM
% with TargetAltitudeM, so a 20 m mission could search only ~20 m above
% ground and converge near the hard floor. The trim problem is aerodynamic
% equilibrium, not absolute altitude; actual mission altitude is certified
% later by closed-loop MPC + Final Mission Validation.
opts.TrimSearchAltitudeM = 200.0;
opts.TrimStopTimeS = 24.0;
opts.TrimRecordStartS = 12.0;
opts.RunsPerConfig = 5;
opts.IdMaxSettleRetries = 6;
opts.IdSettleRetryStepS = 12.0;
opts.IdentificationTs = 0.1;
opts.IdentificationOrders = 3:8;
opts.MinPlantValidation5StepFitPct = 10.0;
opts.MinPlantTest5StepFitPct = 0.0;

% Sequential physical drop/certification timing.
opts.MpcEnableTimeS = 2.0;
opts.DropStartS = 5.0;
opts.DropIntervalS = 2.0;
opts.BasePulseStartS = 10.0;
opts.PulseAfterConfigReachS = 8.0; % fallback for old callers
opts.PulseAfterConfigReachByConfigS = [8;8;25;30;35];
opts.PulseSeparationS = 12.0;
opts.PulseDurationS = 1.2;
opts.PulseElevatorAmplitude = 0.012;
opts.PulseThrottleAmplitude = 0.025;
opts.OptimizationWindowS = 18.0;
opts.FinalWindowS = 22.0;
opts.PostSecondPulseMinS = 18.0;
opts.TailWindowS = 6.0;

% Final complete-mission acceptance test. This is a no-learning, no-pulse
% validation that runs after cfg0..cfg4 are all VERIFIED. Transition gates
% include all four real mass/CG discontinuities; the final tail gates reuse
% the strict v29 certification limits below.
opts.RunFinalMissionValidation = true;
% v30.6.1 mission-state resume. When true, OutputRoot must already contain
% an inherited cfg0..cfg4 VERIFIED checkpoint/master. The function skips all
% single-cfg work and performs only Final Mission + transition recovery.
opts.MissionRecoveryOnly = false;
opts.FinalMissionPostDropS = 50.0;
opts.FinalMissionTailWindowS = 10.0;
opts.FinalMissionPassAltitudeRmsM = 2.0;
opts.FinalMissionPassAltitudeMaxM = 4.0;
opts.FinalMissionPassAltitudeDriftM = 2.5;
opts.FinalMissionPassAirspeedRmsMps = 1.5;
opts.FinalMissionPassVzRmsMps = 1.0;
opts.FinalMissionPassQRmsDps = 1.0;

% MPC local envelope.
opts.FixedAirspeedWeight = 6.0;
opts.FixedPitchWeight = 0.02;
opts.MVWeights = [0.20 0.15];
opts.MVRateWeights = [8.0 4.0];
opts.ElevatorDeviationLimit = 0.035;
opts.ThrottleDeviationLimit = 0.060;
opts.ElevatorDeviationRateLimit = 0.006;
opts.ThrottleDeviationRateLimit = 0.010;
opts.TrustAirspeedMps = 4.0;
opts.TrustPitchDeg = 4.5;
opts.TrustVzMps = 2.5;
opts.TrustQDps = 4.0;
opts.MaxBridgeError = 0.01;

% Formal certification gate.
opts.HardMaxAltitudeErrorM = 50.0;
opts.HardMaxQRmsDps = 10.0;
opts.HardMaxAbsPitchDeg = 30.0;
opts.PassAltitudeRmsM = 1.0;
opts.PassAltitudeMaxM = 2.0;
opts.PassAltitudeDriftM = 1.0;
opts.PassAirspeedRmsMps = 1.0;
opts.PassVzRmsMps = 0.7;
opts.PassQRmsDps = 1.0;
opts.PassTailAltitudeErrorM = 1.0;
opts.PassTailVzMps = 0.35;
opts.RecoveryAltitudeBandM = 1.0;
opts.RecoveryVzBandMps = 0.35;
opts.RecoveryHoldS = 2.0;
opts.RecoveryPenaltyWeight = 0.35;
opts.RecoveryNoPassPenaltyS = 30.0;

% v18 search pressure: start penalizing long-tail bias before the 1 m gate.
opts.DriftPressureStartM = 0.50;
opts.TailPressureStartM = 0.55;
opts.TailVzPressureStartMps = 0.22;
opts.DriftPressureWeight = 60.0;
opts.TailPressureWeight = 45.0;
opts.TailVzPressureWeight = 15.0;
opts.FormalPassObjectiveMultiplier = 0.25;

if mod(numel(varargin),2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts,name), error("Unknown option: %s",name); end
    opts.(name) = varargin{i+1};
end
end
