function result = airdropx_auto_find_trim(varargin)
%AIRDROPX_AUTO_FIND_TRIM Search elevator/throttle/pitch trims with JSBSim.
%
% The optimizer searches requested commands, but the final trim_bank is built
% from the observed steady operating point of a verification run. This keeps
% trim/data/MPC nominal values on the same physical operating point.

opts = local_options(varargin{:});
trimBank = airdropx_auto_default_trim_bank("TargetAltitudeM", opts.TargetAltitudeM, "TargetAirspeedMps", opts.TargetAirspeedMps);
trimBank = local_ensure_trim_fields(trimBank, opts);

records = cell(5, 1);
verifications = cell(5, 1);
checkpoint = local_new_checkpoint(trimBank);
[trimBank, checkpoint] = local_load_resume_state(trimBank, checkpoint, opts);
% v32.1.3: a catastrophic failed trajectory is diagnostic evidence only.
% Never let its observed tail state or requested point bias the next search.
[trimBank, checkpoint] = local_isolate_catastrophic_resume_state(trimBank, checkpoint, opts);

for cfgId = double(opts.ConfigIds(:)).'
    % Reuse a previously verified trim instead of restarting bayesopt.  The
    % stored point is always re-verified once under the current code/options;
    % only a point that still passes is skipped.
    if logical(opts.ReuseVerifiedTrim) && local_checkpoint_is_verified(checkpoint, cfgId)
        fprintf("Re-validating saved trim for config %d before reuse...\n", cfgId);
        [reuseOk, reuseRun, reuseMetrics, reuseOp, reusePoint] = ...
            local_try_reuse_trim(cfgId, trimBank(cfgId + 1), checkpoint, opts);
        if reuseOk
            reuseMode = local_trim_field_string(trimBank(cfgId + 1), "acceptance_mode", "full_trajectory");
            trimBank(cfgId + 1) = local_store_trim_entry(trimBank(cfgId + 1), cfgId, reuseOp, reuseMetrics, opts, ...
                reuseMode);
            if reuseMode == "tail_equilibrium_rescue" && logical(opts.AdaptiveIdSettleEnabled)
                adaptiveSettleS = local_detect_id_settle_s(reuseRun.timeseries, local_tail_metric_opts(opts));
                if isfinite(adaptiveSettleS)
                    trimBank(cfgId + 1).id_settle_s = max(double(trimBank(cfgId + 1).id_settle_s), adaptiveSettleS);
                    fprintf("  cfg%d adaptive ID settle hold = %.1f s before excitation.\n", ...
                        cfgId, trimBank(cfgId + 1).id_settle_s);
                end
            end
            records{cfgId + 1} = struct("reused", true);
            verifications{cfgId + 1} = reuseRun;
            checkpoint = local_checkpoint_success(checkpoint, cfgId, trimBank(cfgId + 1), reuseMetrics, reusePoint);
            local_save_progress(trimBank, records, verifications, checkpoint, opts);
            fprintf("  cfg%d saved trim PASS -> reused, bayesopt skipped.\n", cfgId);
            continue;
        else
            % A formerly verified point may fail because the model/options changed.
            % Only a stable, finite near miss is allowed to become a warm start.
            if ~isempty(reuseMetrics) && ~local_failed_seed_eligible(reuseMetrics, opts)
                fprintf("  cfg%d saved trim failed catastrophically/far outside gate -> isolating it from warm-start.\n", cfgId);
                checkpoint.status(cfgId + 1) = "failed_catastrophic";
                if ~isempty(reuseOp), checkpoint.best_op{cfgId + 1} = reuseOp; end
                checkpoint.best_metrics{cfgId + 1} = reuseMetrics;
                if isempty(checkpoint.best_point{cfgId + 1}) && ~isempty(reusePoint)
                    checkpoint.best_point{cfgId + 1} = reusePoint;
                end
                trimBank = local_reset_failed_trim_to_continuation(trimBank, cfgId, opts);
            else
                fprintf("  cfg%d saved trim no longer passes -> retaining the exact saved verification point as bounded warm start.\n", cfgId);
                checkpoint.status(cfgId + 1) = "stale";
                % Keep checkpoint.best_point unchanged.  Never replace a requested
                % point with an observed tail state merely because the run failed.
                if ~isempty(reuseOp)
                    checkpoint.best_op{cfgId + 1} = reuseOp;
                    checkpoint.best_metrics{cfgId + 1} = reuseMetrics;
                end
                if isempty(checkpoint.best_point{cfgId + 1}) && ~isempty(reusePoint)
                    checkpoint.best_point{cfgId + 1} = reusePoint;
                end
            end
        end
    end

    % If the previous run failed this cfg, center the new search on its best
    % known point instead of returning to the default 0/0.8/4deg guess.
    if logical(opts.ReuseFailedAsWarmStart)
        [trimBank, warmUsed] = local_apply_failed_warm_start(trimBank, checkpoint, cfgId, opts);
        if warmUsed
            fprintf("Warm-starting cfg%d from previous best failed candidate.\n", cfgId);
        end
    end

    % v21 cfg3/cfg4 phase-robust rescue.  The v20 cfg3 winner is a damped
    % phugoid: over a full ~24 s cycle it is centered near 200 m with near-zero
    % mean vz/trend, but the old 5-10 s terminal window happened to end on the
    % descending phase and rejected it.  Before spending another bayesopt
    % budget, re-run the exact saved failed point once at long horizon and
    % accept only if a full-cycle gate proves that the oscillation is centered
    % and decaying.
    if logical(opts.UsePhugoidCycleConfirm) && cfgId >= double(opts.PhugoidConfirmMinConfig) && ...
            ~isempty(checkpoint.best_point{cfgId + 1}) && ...
            any(string(checkpoint.status(cfgId + 1)) == ["failed","stale"])
        fprintf("  cfg%d trying one long phugoid-cycle confirmation before new bayesopt...\n", cfgId);
        phasePoint = checkpoint.best_point{cfgId + 1};
        phaseRun = local_run_candidate(phasePoint, cfgId, trimBank(cfgId + 1), opts, "phugoid_confirm");
        local_check_input_consistency(phaseRun.timeseries, phasePoint, opts, cfgId);
        phaseMetrics = local_trim_metrics(phaseRun.timeseries, opts);
        [phaseOk, phaseCycle] = local_phugoid_cycle_accept(phaseRun.timeseries, phaseMetrics, opts);
        fprintf(['    cycle gate: hRMS=%.3f hMax=%.3f hSlope=%.4f meanVz=%.4f ' ...
            'vzRMS=%.3f qRMS=%.3f VaRMS=%.3f damping=%.3f -> %s\n'], ...
            phaseCycle.hRms, phaseCycle.hMaxAbs, phaseCycle.hSlope, phaseCycle.vzMean, ...
            phaseCycle.vzRms, phaseCycle.qRms, phaseCycle.vaRms, phaseCycle.vzDampingRatio, ...
            string(phaseOk));
        if phaseOk
            phaseOp = local_observed_phugoid_center(phaseRun.timeseries, phaseRun.operating_point, opts);
            trimBank(cfgId + 1) = local_store_trim_entry(trimBank(cfgId + 1), cfgId, ...
                phaseOp, phaseMetrics, opts, "phugoid_cycle_confirm");
            trimBank(cfgId + 1).id_settle_s = max(double(trimBank(cfgId + 1).id_settle_s), ...
                double(opts.PhugoidConfirmIdSettleS) + ...
                double(opts.PhugoidConfirmIdSettlePerConfigS) * max(0, cfgId - double(opts.PhugoidConfirmMinConfig)));
            records{cfgId + 1} = struct("phugoid_cycle_confirm", true, "cycle_metrics", phaseCycle);
            verifications{cfgId + 1} = phaseRun;
            checkpoint = local_checkpoint_success(checkpoint, cfgId, trimBank(cfgId + 1), phaseMetrics, phasePoint);
            local_save_progress(trimBank, records, verifications, checkpoint, opts);
            fprintf("  cfg%d phugoid-cycle confirmation PASS -> trim reused, bayesopt skipped.\n", cfgId);
            continue;
        end
    end


    % v22 history rescue: v21 proved that the single checkpoint winner can
    % fail the phase-robust gate while other already-simulated candidates in
    % the same search folder satisfy the full-cycle equilibrium criteria.
    % Scan saved trim-search trajectories first, rank only candidates that
    % already pass the phugoid-cycle gate, then re-run the best few once at
    % the long confirmation horizon.  This avoids another large bayesopt run
    % when a valid equilibrium has already been simulated.
    if logical(opts.UsePhugoidHistoryRescue) && cfgId >= double(opts.PhugoidConfirmMinConfig)
        [histPoints, histInfo] = local_find_phugoid_history_candidates(cfgId, opts);
        if ~isempty(histPoints)
            fprintf("  cfg%d found %d saved phugoid-pass history candidates; revalidating top %d...\n", ...
                cfgId, height(histPoints), min(height(histPoints), round(double(opts.PhugoidHistoryRescueTopK))));
            maxTry = min(height(histPoints), round(double(opts.PhugoidHistoryRescueTopK)));
            histAccepted = false;
            for hi = 1:maxTry
                historyPoint = histPoints(hi,:);
                fprintf("    history rescue %d/%d: e=%.6f thr=%.6f savedRatio=%.3f\n", ...
                    hi, maxTry, double(historyPoint.Elevator), double(historyPoint.Throttle), ...
                    double(histInfo.gate_ratio(hi)));
                historyRun = local_run_candidate(historyPoint, cfgId, trimBank(cfgId + 1), opts, ...
                    sprintf("phugoid_confirm_history_%02d", hi));
                local_check_input_consistency(historyRun.timeseries, historyPoint, opts, cfgId);
                historyMetrics = local_trim_metrics(historyRun.timeseries, opts);
                [historyOk, historyCycle] = local_phugoid_cycle_accept(historyRun.timeseries, historyMetrics, opts);
                fprintf(['      cycle gate: hRMS=%.3f hMax=%.3f hSlope=%.4f meanVz=%.4f ' ...
                    'vzRMS=%.3f qRMS=%.3f VaRMS=%.3f damping=%.3f -> %s\n'], ...
                    historyCycle.hRms, historyCycle.hMaxAbs, historyCycle.hSlope, historyCycle.vzMean, ...
                    historyCycle.vzRms, historyCycle.qRms, historyCycle.vaRms, ...
                    historyCycle.vzDampingRatio, string(historyOk));
                if historyOk
                    historyOp = local_observed_phugoid_center(historyRun.timeseries, historyRun.operating_point, opts);
                    trimBank(cfgId + 1) = local_store_trim_entry(trimBank(cfgId + 1), cfgId, ...
                        historyOp, historyMetrics, opts, "phugoid_cycle_confirm");
                    trimBank(cfgId + 1).id_settle_s = max(double(trimBank(cfgId + 1).id_settle_s), ...
                        double(opts.PhugoidHistoryRescueIdSettleS) + ...
                        double(opts.PhugoidConfirmIdSettlePerConfigS) * ...
                        max(0, cfgId - double(opts.PhugoidConfirmMinConfig)));
                    records{cfgId + 1} = struct("phugoid_history_rescue", true, ...
                        "saved_candidate_rank", hi, "cycle_metrics", historyCycle, ...
                        "saved_gate_ratio", double(histInfo.gate_ratio(hi)), ...
                        "source_csv", string(histInfo.source_csv(hi)));
                    verifications{cfgId + 1} = historyRun;
                    checkpoint = local_checkpoint_success(checkpoint, cfgId, ...
                        trimBank(cfgId + 1), historyMetrics, historyPoint);
                    local_save_progress(trimBank, records, verifications, checkpoint, opts);
                    fprintf("  cfg%d history phugoid rescue PASS -> trim accepted, bayesopt skipped.\n", cfgId);
                    histAccepted = true;
                    break;
                end
            end
            if histAccepted
                continue;
            end
        end
    end

    % v32.1.4 Stage 0: deterministic joint elevator/throttle equilibrium map.
    % The V55/cfg4 audit proved that pitch acceleration can cross zero while
    % speed equilibrium requires a different throttle.  A one-dimensional
    % continuation seed is therefore only a prior, never the search center.
    jointSeeds = table(zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1), ...
        'VariableNames',{'Elevator','Throttle','Pitch0','Gamma0'});
    jointMapTable = table();
    if logical(opts.UseJointControlMap) && cfgId >= double(opts.JointMapMinConfig)
        fprintf("  Stage 0 joint elevator/throttle equilibrium map...\n");
        [jointSeeds, jointMapTable] = local_joint_control_map(cfgId, trimBank, opts);
        if ~isempty(jointMapTable) && strlength(string(opts.WorkRoot)) > 0
            try
                mapDir = fullfile(string(opts.WorkRoot), sprintf("cfg%d",cfgId));
                if ~isfolder(mapDir), mkdir(mapDir); end
                writetable(jointMapTable, fullfile(mapDir,"joint_control_map.csv"));
            catch ME
                warning("AirdropX:AutoMPC:JointMapWrite", "Could not save joint map: %s", ME.message);
            end
        end
        if ~isempty(jointSeeds)
            fprintf("    joint map supplied %d physics-informed seeds.\n", height(jointSeeds));
        end
    end

    fprintf("Searching trim for config %d\n", cfgId);
    if exist("bayesopt", "file") ~= 2
        error("Statistics and Machine Learning Toolbox bayesopt is required for trim search.");
    end

    totalEvals = max(20, round(double(opts.MaxObjectiveEvaluations)));
    eqEvals = max(12, round(totalEvals * double(opts.EquilibriumSearchFraction)));
    refineEvals = max(8, totalEvals - eqEvals);

    % Stage A: search a compact, physically plausible level-flight region.
    % The v4 cfg0 data showed that 60 evaluations over the old very wide 4-D
    % box were too sparse.  This stage emphasizes tail equilibrium
    % derivatives first so elevator/throttle converge toward a true open-loop
    % equilibrium instead of a trajectory that merely crosses the target.
    eqBounds = local_focused_bounds(cfgId, trimBank, opts, jointSeeds);
    eqVars = local_vars_from_bounds(eqBounds);
    eqInitialX = local_initial_x(cfgId, trimBank, opts, eqBounds, jointSeeds);
    eqObjective = @(x) local_objective(x, cfgId, trimBank(cfgId + 1), opts, "equilibrium");
    fprintf("  Stage A equilibrium search: %d evaluations\n", eqEvals);
    boEq = bayesopt(eqObjective, eqVars, ...
        "MaxObjectiveEvaluations", eqEvals, ...
        "InitialX", eqInitialX, ...
        "XConstraintFcn", @(X)local_initial_state_constraint(X, opts), ...
        "IsObjectiveDeterministic", true, ...
        "UseParallel", logical(opts.UseParallel), ...
        "Verbose", double(opts.Verbose));
    bestEq = bestPoint(boEq);

    % Probe the Stage-A point once and use the observed state to center Stage B.
    % This is different from the old self-consistency loop: Stage B is still
    % allowed to change elevator and throttle, which is necessary when the
    % fixed inputs themselves cause a sustained climb/descent.
    probe = local_run_candidate(bestEq, cfgId, trimBank(cfgId + 1), opts, "eq_probe");
    local_check_input_consistency(probe.timeseries, bestEq, opts, cfgId);
    probeOp = local_observed_trim_point(probe.timeseries, probe.operating_point, opts);

    refineBounds = local_refine_bounds(bestEq, probeOp, opts);
    refineVars = local_vars_from_bounds(refineBounds);
    refineInitialX = local_refine_initial_x(bestEq, probeOp, refineBounds, opts);
    refineObjective = @(x) local_objective(x, cfgId, trimBank(cfgId + 1), opts, "trajectory");
    fprintf("  Stage B trajectory refinement: %d evaluations\n", refineEvals);
    boRefine = bayesopt(refineObjective, refineVars, ...
        "MaxObjectiveEvaluations", refineEvals, ...
        "InitialX", refineInitialX, ...
        "XConstraintFcn", @(X)local_initial_state_constraint(X, opts), ...
        "IsObjectiveDeterministic", true, ...
        "UseParallel", logical(opts.UseParallel), ...
        "Verbose", double(opts.Verbose));
    best = bestPoint(boRefine);
    records{cfgId + 1} = struct("joint_control_map", jointMapTable, "equilibrium", boEq, "refine", boRefine);

    % Verify the refined point.  Self-consistent initial-state passes are
    % optional, but unlike v4 we never blindly keep the last pass.  We retain
    % the best verification trajectory, so a worse observed-state iteration
    % cannot overwrite a better candidate.
    verifyRuns = {};
    verifyMetricsList = {};
    verifyOps = {};
    verifyPoints = {};

    verify = local_run_candidate(best, cfgId, trimBank(cfgId + 1), opts, "verify");
    local_check_input_consistency(verify.timeseries, best, opts, cfgId);
    verifyRuns{end+1} = verify; %#ok<AGROW>
    verifyMetricsList{end+1} = local_trim_metrics(verify.timeseries, opts); %#ok<AGROW>
    verifyOps{end+1} = local_observed_trim_point(verify.timeseries, verify.operating_point, opts); %#ok<AGROW>
    verifyPoints{end+1} = best; %#ok<AGROW>

    opIter = verifyOps{end};
    for pass = 1:double(opts.SelfConsistentVerifyPasses)
        if ~local_trim_accept_fail(verifyMetricsList{end}, opts)
            break;
        end
        selfPoint = local_point_from_op(opIter, opts);
        verifySc = local_run_candidate(selfPoint, cfgId, trimBank(cfgId + 1), opts, sprintf("verify_sc%d", pass));
        local_check_input_consistency(verifySc.timeseries, selfPoint, opts, cfgId);
        verifyRuns{end+1} = verifySc; %#ok<AGROW>
        verifyMetricsList{end+1} = local_trim_metrics(verifySc.timeseries, opts); %#ok<AGROW>
        verifyOps{end+1} = local_observed_trim_point(verifySc.timeseries, verifySc.operating_point, opts); %#ok<AGROW>
        verifyPoints{end+1} = selfPoint; %#ok<AGROW>
        opIter = verifyOps{end};
    end

    [bestVerifyIdx, feasibleVerify] = local_best_verification(verifyMetricsList, opts);
    verify = verifyRuns{bestVerifyIdx};
    verifyMetrics = verifyMetricsList{bestVerifyIdx};
    op = verifyOps{bestVerifyIdx};
    selectedPoint = verifyPoints{bestVerifyIdx};
    gateRatio = local_trim_gate_ratio(verifyMetrics, opts);

    % v6 adaptive recovery/polish.  cfg0 is already excellent in the latest
    % data, while cfg1 is close and cfg2 has a clear terminal descent.  Do not
    % immediately abort a near miss.  Re-open elevator/throttle around the best
    % verification and the sequential cfg prediction, then optimize the actual
    % failed gates.  A feasible but low-margin cfg is polished as well so the
    % next payload configuration gets a better continuation seed.
    needsRecovery = logical(opts.UseRecoverySearch) && ...
        (~feasibleVerify || gateRatio > double(opts.PolishTriggerRatio));
    if needsRecovery
        recoveryBounds = local_recovery_bounds(best, op, cfgId, trimBank, opts);
        recoveryVars = local_vars_from_bounds(recoveryBounds);
        recoveryInitialX = local_recovery_initial_x(best, op, cfgId, trimBank, recoveryBounds, opts);
        recoveryObjective = @(x) local_objective(x, cfgId, trimBank(cfgId + 1), opts, "recovery");
        recoveryEvals = max(12, round(double(opts.RecoveryObjectiveEvaluations) * ...
            (1.0 + double(opts.RecoveryConfigGrowth) * max(0, cfgId - 1))));
        fprintf("  Stage C adaptive recovery/polish: %d evaluations (gate ratio %.3f)\n", ...
            recoveryEvals, gateRatio);
        boRecovery = bayesopt(recoveryObjective, recoveryVars, ...
            "MaxObjectiveEvaluations", recoveryEvals, ...
            "InitialX", recoveryInitialX, ...
            "XConstraintFcn", @(X)local_initial_state_constraint(X, opts), ...
            "IsObjectiveDeterministic", true, ...
            "UseParallel", logical(opts.UseParallel), ...
            "Verbose", double(opts.Verbose));
        records{cfgId + 1}.recovery = boRecovery;

        recoveryBest = bestPoint(boRecovery);
        verifyRecovery = local_run_candidate(recoveryBest, cfgId, trimBank(cfgId + 1), opts, "verify_recovery");
        local_check_input_consistency(verifyRecovery.timeseries, recoveryBest, opts, cfgId);
        verifyRuns{end+1} = verifyRecovery; %#ok<AGROW>
        verifyMetricsList{end+1} = local_trim_metrics(verifyRecovery.timeseries, opts); %#ok<AGROW>
        verifyOps{end+1} = local_observed_trim_point(verifyRecovery.timeseries, verifyRecovery.operating_point, opts); %#ok<AGROW>
        verifyPoints{end+1} = recoveryBest; %#ok<AGROW>

        if logical(opts.RecoverySelfConsistentVerify)
            recoveryScPoint = local_point_from_op(verifyOps{end}, opts);
            verifyRecoverySc = local_run_candidate(recoveryScPoint, cfgId, trimBank(cfgId + 1), opts, "verify_recovery_sc");
            local_check_input_consistency(verifyRecoverySc.timeseries, recoveryScPoint, opts, cfgId);
            verifyRuns{end+1} = verifyRecoverySc; %#ok<AGROW>
            verifyMetricsList{end+1} = local_trim_metrics(verifyRecoverySc.timeseries, opts); %#ok<AGROW>
            verifyOps{end+1} = local_observed_trim_point(verifyRecoverySc.timeseries, verifyRecoverySc.operating_point, opts); %#ok<AGROW>
            verifyPoints{end+1} = recoveryScPoint; %#ok<AGROW>
        end

        [bestVerifyIdx, feasibleVerify] = local_best_verification(verifyMetricsList, opts);
        verify = verifyRuns{bestVerifyIdx};
        verifyMetrics = verifyMetricsList{bestVerifyIdx};
        op = verifyOps{bestVerifyIdx};
        selectedPoint = verifyPoints{bestVerifyIdx};
        gateRatio = local_trim_gate_ratio(verifyMetrics, opts);
    end

    % v9 Stage D: long-horizon terminal-equilibrium polish.  v8 showed two
    % separate issues: (1) the best cfg1 recovery point was lost when the
    % observed operating point was converted back into an initial point; and
    % (2) cfg2 Stage D froze Pitch0/Gamma0, while the all-candidate ranking
    % showed tail q stuck around 0.33 deg/s even when vz/hSlope were small.
    % Keep elevator/throttle as the true steady controls, but allow a *local*
    % Pitch0/Gamma0 adjustment so the lightly damped phugoid is not evaluated
    % at a bad phase.  A longer tail window prevents a single turning point
    % from masquerading as equilibrium.
    tailRescueAccepted = false;
    tailPolishAttempted = false;
    tailPolishPass = false;
    tailPolishRatio = Inf;
    tailPolishMetrics = struct();
    tailOpts = local_tail_metric_opts(opts);
    baselineTailMetrics = local_trim_metrics(verify.timeseries, tailOpts);
    baselineTailRatio = local_tail_gate_ratio(baselineTailMetrics, tailOpts);
    ordinaryGateRatioBeforeTailPolish = gateRatio;
    forceTailPolish = logical(opts.ForceLongHorizonTailPolish);
    if logical(opts.UseTailEquilibriumRescue) && cfgId >= double(opts.TailRescueMinConfig) && ...
            (forceTailPolish || ((~feasibleVerify) && local_tail_rescue_needed(baselineTailMetrics, tailOpts)))
        fixedPoint = selectedPoint;
        tailBounds = local_tail_rescue_bounds(selectedPoint, op, cfgId, trimBank, opts);
        tailVars = local_vars_from_bounds(tailBounds);
        tailInitialX = local_tail_rescue_initial_x(selectedPoint, op, cfgId, trimBank, tailBounds, opts);
        tailEvals = max(28, round(double(opts.TailRescueObjectiveEvaluations) + ...
            double(opts.TailRescueEvalGrowthPerConfig) * max(0, cfgId - double(opts.TailRescueMinConfig))));
        fprintf("  Stage D long-horizon tail polish: %d evaluations (tail gate ratio %.3f)\n", ...
            tailEvals, baselineTailRatio);
        tailObjective = @(u) local_tail_rescue_objective(u, fixedPoint, cfgId, trimBank(cfgId + 1), tailOpts);
        boTail = bayesopt(tailObjective, tailVars, ...
            "MaxObjectiveEvaluations", tailEvals, ...
            "InitialX", tailInitialX, ...
            "XConstraintFcn", @(X)local_initial_state_constraint(X, opts), ...
            "IsObjectiveDeterministic", true, ...
            "UseParallel", logical(opts.UseParallel), ...
            "Verbose", double(opts.Verbose));
        records{cfgId + 1}.tail_rescue = boTail;

        tailBest4 = bestPoint(boTail);
        tailPoint = local_tail_point(tailBest4, fixedPoint, opts);
        verifyTail = local_run_candidate(tailPoint, cfgId, trimBank(cfgId + 1), opts, "tail_rescue_verify");
        local_check_input_consistency(verifyTail.timeseries, tailPoint, opts, cfgId);
        tailMetrics = local_trim_metrics(verifyTail.timeseries, tailOpts);
        tailOp = local_observed_trim_point(verifyTail.timeseries, verifyTail.operating_point, tailOpts);

        % A single observed-state check is useful, but it is only retained if
        % it is actually better.  Never overwrite a good requested point with
        % a worse self-consistent point (the exact cfg1 failure seen in v8).
        tailScPoint = local_point_from_op(tailOp, opts);
        verifyTailSc = local_run_candidate(tailScPoint, cfgId, trimBank(cfgId + 1), opts, "tail_rescue_verify_sc");
        local_check_input_consistency(verifyTailSc.timeseries, tailScPoint, opts, cfgId);
        tailScMetrics = local_trim_metrics(verifyTailSc.timeseries, tailOpts);
        tailScOp = local_observed_trim_point(verifyTailSc.timeseries, verifyTailSc.operating_point, tailOpts);

        if local_tail_gate_ratio(tailScMetrics, tailOpts) < local_tail_gate_ratio(tailMetrics, tailOpts)
            tailMetrics = tailScMetrics;
            tailOp = tailScOp;
            verifyTail = verifyTailSc;
            tailPoint = tailScPoint;
        end

        tailRatio = local_tail_gate_ratio(tailMetrics, tailOpts);
        tailPolishAttempted = true;
        tailPolishMetrics = tailMetrics;
        tailPolishRatio = tailRatio;
        tailPolishPass = ~local_tail_accept_fail(tailMetrics, tailOpts);
        if tailPolishPass
            tailRescueAccepted = true;
            feasibleVerify = true;
            verify = verifyTail;
            verifyMetrics = tailMetrics;
            op = tailOp;
            selectedPoint = tailPoint;
            gateRatio = local_trim_gate_ratio(verifyMetrics, opts);
            fprintf(['  Stage D accepted static cfg%d equilibrium: tail vz=%.3f, q=%.3f, ' ...
                'hSlope=%.3f, VaRms=%.3f\n'], cfgId, verifyMetrics.tailVzMed, ...
                verifyMetrics.tailQMed, verifyMetrics.tailHeightSlope, verifyMetrics.tailAirspeedRms);
        elseif tailRatio < baselineTailRatio
            % Keep the best *actual verification trajectory* across A/B/C/D.
            % Stage D may be useful without passing, but it can no longer
            % replace a better recovery point merely because its BO objective
            % was smaller.
            verify = verifyTail;
            verifyMetrics = tailMetrics;
            op = tailOp;
            selectedPoint = tailPoint;
            gateRatio = local_trim_gate_ratio(verifyMetrics, opts);
        end
    end
    % v32.1.5: when long-horizon polish is explicitly required by the ID-readiness
    % recovery path, a short-horizon feasible point is NOT sufficient. The long
    % run itself must satisfy the static tail gate before this trim can be used.
    % v32.1.5 fix: forced ID-readiness polish is a second, long-horizon
    % certificate.  Do not infer its result from the ordinary trim gate or
    % from a path-state flag.  Use the actual Stage-D verification result.
    if forceTailPolish && (~tailPolishAttempted || ~tailPolishPass)
        feasibleVerify = false;
    end

    if ~feasibleVerify
        checkpoint = local_checkpoint_failure(checkpoint, cfgId, op, verifyMetrics, selectedPoint, opts);
        local_save_progress(trimBank, records, verifications, checkpoint, opts);
        if forceTailPolish && tailPolishAttempted && ~tailPolishPass
            tm = tailPolishMetrics;
            error("AirdropX:AutoMPC:NoUsableTrim", ...
                ['Config %d ordinary trim verification passed/near-passed (ordinary gate %.3f), but required ' ...
                 'long-horizon ID-readiness tail polish failed (tail gate %.3f): ' ...
                 'tail(vzMed/vzRms/qMed/qRms/Vrms/pitchStd/hSlope/Vslope/pitchSlope)=' ...
                 '%.4f/%.4f/%.4f/%.4f/%.4f/%.4f/%.5f/%.5f/%.5f. ' ...
                 'Do not interpret ordinary gate < 1 as a long-horizon ID_READY PASS.'], ...
                cfgId, ordinaryGateRatioBeforeTailPolish, tailPolishRatio, ...
                tm.tailVzMed, tm.tailVzRms, tm.tailQMed, tm.tailQRms, tm.tailAirspeedRms, ...
                tm.tailPitchStd, tm.tailHeightSlope, tm.tailAirspeedSlope, tm.tailPitchSlope);
        end
        error("AirdropX:AutoMPC:NoUsableTrim", ...
            ['Config %d trim failed best verification (gate ratio %.3f): fullH(RMS/max)=%.3f/%.3f, ' ...
             'earlyH(RMS/max)=%.3f/%.3f, steadyH(RMS/max/drift)=%.3f/%.3f/%.3f, ' ...
             'tail(vz/q/V/hSlope)=%.3f/%.3f/%.3f/%.3f, fullVaRms=%.3f, ' ...
             'steadyVaRms=%.3f, fullVzRms=%.3f, steadyVzRms=%.3f, ' ...
             'qRms=%.3f, pitchStd=%.3f, pitchDrift=%.3f deg/s.'], ...
            cfgId, gateRatio, verifyMetrics.fullHRms, verifyMetrics.fullHMaxAbs, ...
            verifyMetrics.earlyHRms, verifyMetrics.earlyHMaxAbs, ...
            verifyMetrics.hRms, verifyMetrics.hMaxAbs, verifyMetrics.hDrift, ...
            verifyMetrics.tailVzMed, verifyMetrics.tailQMed, verifyMetrics.tailAirspeedRms, ...
            verifyMetrics.tailHeightSlope, verifyMetrics.fullAirspeedRms, verifyMetrics.vaRms, ...
            verifyMetrics.fullVzRms, verifyMetrics.vzRms, verifyMetrics.qRms, ...
            verifyMetrics.pitchStd, verifyMetrics.pitchDriftDegps);
    end

    if tailRescueAccepted
        score = local_tail_equilibrium_score(verifyMetrics, opts);
    else
        score = local_trim_soft_score(verifyMetrics, opts);
    end
    if tailRescueAccepted
        acceptanceMode = "tail_equilibrium_rescue";
    else
        acceptanceMode = "full_trajectory";
    end
    trimBank(cfgId + 1) = local_store_trim_entry(trimBank(cfgId + 1), cfgId, op, verifyMetrics, opts, acceptanceMode);
    if tailRescueAccepted && logical(opts.AdaptiveIdSettleEnabled)
        adaptiveSettleS = local_detect_id_settle_s(verify.timeseries, tailOpts);
        if isfinite(adaptiveSettleS)
            trimBank(cfgId + 1).id_settle_s = max(double(trimBank(cfgId + 1).id_settle_s), adaptiveSettleS);
            fprintf("  cfg%d adaptive ID settle hold = %.1f s before excitation.\n", ...
                cfgId, trimBank(cfgId + 1).id_settle_s);
        end
    end
    trimBank(cfgId + 1).score = score;
    verifications{cfgId + 1} = verify;
    checkpoint = local_checkpoint_success(checkpoint, cfgId, trimBank(cfgId + 1), verifyMetrics, selectedPoint);
    local_save_progress(trimBank, records, verifications, checkpoint, opts);

    fprintf(['Verified cfg%d op: pitch=%.4f deg, gamma=%.4f deg, elevator=%.5f, throttle=%.5f, ' ...
        'fullH=%.3f/%.3f m, earlyH=%.3f/%.3f m, steadyH=%.3f/%.3f/drift %.3f m, ' ...
        'tail vz=%.3f q=%.3f hSlope=%.3f, V=%.3f m/s\n'], ...
        cfgId, op.pitch_deg, op.initial_flight_path_deg, op.elevator_cmd, op.throttle_cmd, ...
        verifyMetrics.fullHRms, verifyMetrics.fullHMaxAbs, verifyMetrics.earlyHRms, ...
        verifyMetrics.earlyHMaxAbs, verifyMetrics.hRms, verifyMetrics.hMaxAbs, ...
        verifyMetrics.hDrift, verifyMetrics.tailVzMed, verifyMetrics.tailQMed, ...
        verifyMetrics.tailHeightSlope, op.airspeed_mps);
end

result = struct("trim_bank", trimBank, "records", {records}, "verification_runs", {verifications}, ...
    "checkpoint", checkpoint);
local_save_progress(trimBank, records, verifications, checkpoint, opts);
end

function score = local_objective(x, cfgId, trim, opts, mode)
try
    r = local_run_candidate(x, cfgId, trim, opts, "search");
    T = r.timeseries;
    if height(T) < 3
        score = Inf;
        return;
    end
    local_check_input_consistency(T, x, opts, cfgId);
    metrics = local_trim_metrics(T, opts);
    if local_trim_catastrophic_fail(metrics, opts)
        score = double(opts.HardFailScore) + local_trim_soft_score(metrics, opts);
        return;
    end
    if string(mode) == "equilibrium"
        score = local_equilibrium_score(metrics, opts);
    elseif string(mode) == "recovery"
        score = local_recovery_score(metrics, opts);
    else
        score = local_trim_soft_score(metrics, opts);
    end
catch ME
    warning("AirdropX:AutoMPC:TrimEvalFailed", "Trim objective failed: %s", ME.message);
    score = Inf;
end
end

function r = local_run_candidate(x, cfgId, trim, opts, tag)
trim.elevator_cmd = double(x.Elevator);
trim.throttle_cmd = double(x.Throttle);
trim.pitch_deg = double(x.Pitch0);
trim.altitude_m = double(opts.SearchAltitudeM);
runId = sprintf("trim_%s_cfg%d_%s", tag, cfgId, char(string(datetime("now", "Format", "HHmmssSSS"))));
runStopTimeS = min(double(opts.MaxTrimStopTimeS), ...
    double(opts.StopTimeS) + double(opts.ExtraStopTimePerConfigS) * max(0, cfgId));
if contains(string(tag), "tail_rescue")
    runStopTimeS = max(runStopTimeS, double(opts.TailRescueStopTimeBaseS) + ...
        double(opts.TailRescueStopTimePerConfigS) * max(0, cfgId - double(opts.TailRescueMinConfig)));
end
if contains(string(tag), "phugoid_confirm")
    runStopTimeS = max(runStopTimeS, double(opts.PhugoidConfirmStopTimeS) + ...
        double(opts.PhugoidConfirmStopTimePerConfigS) * max(0, cfgId - double(opts.PhugoidConfirmMinConfig)));
end
r = airdropx_auto_run_id_experiment( ...
    "ProjectRoot", opts.ProjectRoot, "Model", opts.Model, "RunId", runId, ...
    "ConfigId", cfgId, "Trim", trim, "StopTimeS", runStopTimeS, ...
    "OutputRoot", local_eval_output_root(opts, cfgId, runId), ...
    "RecordStartS", opts.RecordStartS, "InitialAirspeedMps", opts.TargetAirspeedMps, ...
    "ReferenceMassKg", opts.ReferenceMassKg, "CargoMassKg", opts.CargoMassKg, ...
    "PrepDropStartS", opts.PrepDropStartS, "PrepDropIntervalS", opts.PrepDropIntervalS, ...
    "InitialAltitudeM", opts.SearchAltitudeM, ...
    "InitialPitchDeg", local_preparation_initial_pitch(x, cfgId, opts), ...
    "InitialFlightPathDeg", local_preparation_initial_gamma(x, cfgId, opts), ...
    "ElevatorAmplitude", 0.0, "ThrottleAmplitude", 0.0, ...
    "PreparationTrimBank", opts.PreparationTrimBank, ...
    "UsePreparationTrimSchedule", opts.UsePreparationTrimSchedule, ...
    "KeepFixedConfigurationOnly", true, "DirectIdMode", true, ...
    "ExportStartS", 0.0);
end

function pitchDeg = local_preparation_initial_pitch(x, cfgId, opts)
pitchDeg = double(x.Pitch0);
if cfgId > 0 && ~isempty(opts.PreparationTrimBank) && logical(opts.UsePreparationTrimSchedule)
    try
        pitchDeg = local_trim_field(opts.PreparationTrimBank(1), "pitch_deg", pitchDeg);
    catch
    end
end
end

function gammaDeg = local_preparation_initial_gamma(x, cfgId, opts)
gammaDeg = double(x.Gamma0);
if cfgId > 0 && ~isempty(opts.PreparationTrimBank) && logical(opts.UsePreparationTrimSchedule)
    try
        gammaDeg = local_trim_field(opts.PreparationTrimBank(1), "initial_flight_path_deg", 0.0);
    catch
        gammaDeg = 0.0;
    end
end
end

function bounds = local_freeze_preparation_state_bounds(bounds, cfgId, opts)
% Sequential preparation mode makes Pitch0/Gamma0 cfg0 release states, not
% target-cfg trim variables. Freeze those dimensions so bayesopt spends its
% budget on the true steady controls: elevator and throttle.
if cfgId <= 0 || isempty(opts.PreparationTrimBank) || ~logical(opts.UsePreparationTrimSchedule)
    return;
end
try
    p0 = local_trim_field(opts.PreparationTrimBank(1), "pitch_deg", mean(bounds(3,:)));
    g0 = local_trim_field(opts.PreparationTrimBank(1), "initial_flight_path_deg", 0.0);
    tiny = 1.0e-6;
    bounds(3,:) = [p0 - tiny, p0 + tiny];
    bounds(4,:) = [g0 - tiny, g0 + tiny];
catch
end
end

function op = local_observed_trim_point(T, fallback, opts)
op = fallback;
if isempty(T) || height(T) == 0
    return;
end

% Use the terminal equilibrium window, not the whole post-RecordStart window.
% For cfg1/cfg2 the trajectory can cross the target and then reverse direction;
% averaging the whole window gives a misleading gamma/pitch center for Stage B.
t = double(T.time_s);
valid = isfinite(t);
tMax = max(t(valid), [], "omitnan");
tailStart = max(double(opts.RecordStartS), tMax - double(opts.ObservedOpTailWindowS));
mask = valid & t >= tailStart;
if nnz(mask) < 5
    mask = valid & t >= double(opts.RecordStartS);
end
if nnz(mask) < 5
    mask = valid;
end

op.altitude_m = local_median_omitnan(T.altitude_m(mask));
op.airspeed_mps = local_median_omitnan(T.airspeed_mps(mask));
op.pitch_deg = local_median_omitnan(T.pitch_deg(mask));
op.vz_up_mps = local_median_omitnan(T.vz_up_mps(mask));
op.q_dps = local_median_omitnan(T.q_dps(mask));
if ismember("elevator_cmd_actual", string(T.Properties.VariableNames))
    op.elevator_cmd = local_median_omitnan(T.elevator_cmd_actual(mask));
elseif ismember("elevator_cmd", string(T.Properties.VariableNames))
    op.elevator_cmd = local_median_omitnan(T.elevator_cmd(mask));
end
if ismember("throttle_cmd_actual", string(T.Properties.VariableNames))
    op.throttle_cmd = local_median_omitnan(T.throttle_cmd_actual(mask));
elseif ismember("throttle_cmd", string(T.Properties.VariableNames))
    op.throttle_cmd = local_median_omitnan(T.throttle_cmd(mask));
end
ratio = double(T.vz_up_mps(mask)) ./ max(abs(double(T.airspeed_mps(mask))), 1e-6);
ratio = min(max(ratio, -1.0), 1.0);
gamma = asind(ratio);
op.initial_flight_path_deg = local_median_omitnan(gamma);
if ~isfinite(op.initial_flight_path_deg)
    op.initial_flight_path_deg = 0.0;
end
end


function [ok, m] = local_phugoid_cycle_accept(T, standardMetrics, opts)
% Judge later-config trim over roughly one full phugoid cycle instead of a
% short endpoint window. This does NOT relax the altitude target: it requires
% the cycle to remain centered close to 200 m, with low net vertical trend and
% decaying dynamic energy.
m = struct("hRms",Inf,"hMaxAbs",Inf,"hSlope",Inf,"vzMean",Inf,"vzRms",Inf, ...
    "qMean",Inf,"qRms",Inf,"vaRms",Inf,"pitchStd",Inf, ...
    "vzDampingRatio",Inf,"qDampingRatio",Inf,"windowS",0);
if isempty(T) || height(T) < 10
    ok = false;
    return;
end
t = double(T.time_s(:));
h = double(T.altitude_m(:));
v = double(T.airspeed_mps(:));
vz = double(T.vz_up_mps(:));
q = double(T.q_dps(:));
pitch = double(T.pitch_deg(:));
valid = isfinite(t)&isfinite(h)&isfinite(v)&isfinite(vz)&isfinite(q)&isfinite(pitch);
if nnz(valid) < 10
    ok = false;
    return;
end
tMax = max(t(valid));
tMin = min(t(valid));
windowS = min(double(opts.PhugoidConfirmWindowS), max(0.0,tMax-tMin));
startS = tMax - windowS;
mask = valid & t >= startS;
if nnz(mask) < 10
    ok = false;
    return;
end
tt = t(mask); hh = h(mask); vv = v(mask); vzz = vz(mask); qq = q(mask); pp = pitch(mask);
hErr = hh - double(opts.SearchAltitudeM);
m.windowS = windowS;
m.hRms = local_rms_omitnan(hErr);
m.hMaxAbs = local_max_abs(hErr);
m.hSlope = local_linear_slope_trim(tt,hh);
m.vzMean = local_mean_omitnan(vzz);
m.vzRms = local_rms_omitnan(vzz);
m.qMean = local_mean_omitnan(qq);
m.qRms = local_rms_omitnan(qq);
m.vaRms = local_rms_omitnan(vv-double(opts.TargetAirspeedMps));
m.pitchStd = local_std_omitnan(pp);

% Compare one complete current cycle with the preceding cycle whenever
% possible. This is less endpoint-phase-sensitive than splitting one cycle
% into two half-cycles.
prevMask = valid & t >= (startS-windowS) & t < startS;
if nnz(prevMask) >= 10
    vzPrev = local_rms_omitnan(vz(prevMask));
    qPrev = local_rms_omitnan(q(prevMask));
    m.vzDampingRatio = m.vzRms/max(vzPrev,1e-6);
    m.qDampingRatio = m.qRms/max(qPrev,1e-6);
else
    midT = 0.5*(min(tt)+max(tt));
    first = tt < midT;
    second = ~first;
    if nnz(first)>=5 && nnz(second)>=5
        vz1 = local_rms_omitnan(vzz(first));
        vz2 = local_rms_omitnan(vzz(second));
        q1 = local_rms_omitnan(qq(first));
        q2 = local_rms_omitnan(qq(second));
        m.vzDampingRatio = vz2/max(vz1,1e-6);
        m.qDampingRatio = q2/max(q1,1e-6);
    end
end

fullSafe = isfinite(standardMetrics.fullHRms) && ...
    standardMetrics.fullHRms <= double(opts.MaxFullAltitudeRmsM) && ...
    standardMetrics.fullHMaxAbs <= double(opts.MaxFullAltitudeMaxAbsM) && ...
    standardMetrics.fullAirspeedRms <= double(opts.MaxFullAirspeedRmsMps) && ...
    standardMetrics.fullVzRms <= double(opts.MaxFullVzRmsMps) && ...
    standardMetrics.fullQRms <= double(opts.MaxFullQRmsDps);

ok = fullSafe && ...
    m.hRms <= double(opts.PhugoidConfirmMaxAltitudeRmsM) && ...
    m.hMaxAbs <= double(opts.PhugoidConfirmMaxAltitudeMaxAbsM) && ...
    abs(m.hSlope) <= double(opts.PhugoidConfirmMaxHeightSlopeMps) && ...
    abs(m.vzMean) <= double(opts.PhugoidConfirmMaxMeanVzMps) && ...
    m.vzRms <= double(opts.PhugoidConfirmMaxVzRmsMps) && ...
    abs(m.qMean) <= double(opts.PhugoidConfirmMaxMeanQDps) && ...
    m.qRms <= double(opts.PhugoidConfirmMaxQRmsDps) && ...
    m.vaRms <= double(opts.PhugoidConfirmMaxAirspeedRmsMps) && ...
    m.pitchStd <= double(opts.PhugoidConfirmMaxPitchStdDeg) && ...
    m.vzDampingRatio <= double(opts.PhugoidConfirmMaxVzDampingRatio) && ...
    m.qDampingRatio <= double(opts.PhugoidConfirmMaxQDampingRatio);
end


function [points, info] = local_find_phugoid_history_candidates(cfgId, opts)
% Recover already-simulated trim candidates that satisfy the exact v21
% full-cycle gate.  The normal trim optimizer ranks endpoint/tail metrics, so
% a candidate can be physically acceptable over a complete phugoid cycle yet
% never become checkpoint.best_point.  This routine converts those saved
% trajectories back into requested points for one deterministic long recheck.
points = table(zeros(0,1), zeros(0,1), zeros(0,1), zeros(0,1), 'VariableNames', ...
    {'Elevator','Throttle','Pitch0','Gamma0'});
info = table(zeros(0,1), strings(0,1), 'VariableNames', {'gate_ratio','source_csv'});

root = string(opts.WorkRoot);
if strlength(root) == 0
    return;
end
cfgRoot = fullfile(root, sprintf("cfg%d", cfgId));
if ~isfolder(cfgRoot)
    return;
end

% v23: trim-search trajectories are stored under the Plant rebuild tree,
% e.g. cfg3/plant_training/trim/search/cfg3/trim_*/auto_id_timeseries.csv.
% v22 only scanned cfg3/trim_* and therefore silently missed the complete
% history.  Search the known current layout first, retain compatibility with
% older direct layouts, then fall back to a recursive scan.
searchRoot = fullfile(cfgRoot, "plant_training", "trim", "search", sprintf("cfg%d", cfgId));
files = dir(fullfile(searchRoot, "trim_*", "auto_id_timeseries.csv"));
files = [files; dir(fullfile(cfgRoot, "trim_*", "auto_id_timeseries.csv"))]; %#ok<AGROW>
if isempty(files)
    recursiveFiles = dir(fullfile(cfgRoot, "**", "auto_id_timeseries.csv"));
    keep = false(numel(recursiveFiles),1);
    for fi = 1:numel(recursiveFiles)
        [~, parentName] = fileparts(recursiveFiles(fi).folder);
        keep(fi) = startsWith(string(parentName), "trim_");
    end
    files = recursiveFiles(keep);
end
if isempty(files)
    return;
end

% De-duplicate in case the compatibility and current-layout scans overlap.
fileKeys = strings(numel(files),1);
for fi = 1:numel(files)
    fileKeys(fi) = string(fullfile(files(fi).folder, files(fi).name));
end
[~, fileIa] = unique(fileKeys, 'stable');
files = files(fileIa);

% Prefer already-verified/recovery/phugoid-confirm trajectories.  They are
% much fewer than the raw Bayesopt search files and are the most useful rescue
% seeds.  The full history remains available behind them.
priority = zeros(numel(files),1);
for fi = 1:numel(files)
    [~, parentName] = fileparts(files(fi).folder);
    pn = string(parentName);
    if contains(pn, "verify") || contains(pn, "phugoid_confirm") || contains(pn, "recovery")
        priority(fi) = 0;
    elseif contains(pn, "eq_probe")
        priority(fi) = 1;
    else
        priority(fi) = 2;
    end
end
[~, fileOrder] = sort(priority, 'ascend');
files = files(fileOrder);

candE = [];
candT = [];
candP = [];
candG = [];
candRatio = [];
candCsv = strings(0,1);

for i = 1:numel(files)
    csvFile = fullfile(files(i).folder, files(i).name);
    try
        T = readtable(csvFile);
        need = ["time_s","altitude_m","airspeed_mps","pitch_deg","vz_up_mps","q_dps", ...
            "requested_elevator_trim","requested_throttle_trim"];
        if ~all(ismember(need, string(T.Properties.VariableNames)))
            continue;
        end
        standardMetrics = local_trim_metrics(T, opts);
        [cycleOk, cycleMetrics] = local_phugoid_cycle_accept(T, standardMetrics, opts);
        if ~cycleOk
            continue;
        end

        elev = local_history_median(T, "requested_elevator_trim", NaN);
        thr = local_history_median(T, "requested_throttle_trim", NaN);
        pitch0 = local_history_median(T, "requested_pitch_trim_deg", NaN);
        if ~isfinite(pitch0)
            pitch0 = local_history_release_pitch(opts);
        end
        gamma0 = local_history_release_gamma(opts);
        if ~all(isfinite([elev,thr,pitch0,gamma0]))
            continue;
        end

        candE(end+1,1) = elev; %#ok<AGROW>
        candT(end+1,1) = thr; %#ok<AGROW>
        candP(end+1,1) = pitch0; %#ok<AGROW>
        candG(end+1,1) = gamma0; %#ok<AGROW>
        candRatio(end+1,1) = local_phugoid_gate_ratio(standardMetrics, cycleMetrics, opts); %#ok<AGROW>
        candCsv(end+1,1) = string(csvFile); %#ok<AGROW>

        % A modest pool is enough because every returned candidate is
        % deterministically re-run before acceptance.  Avoid reading hundreds
        % of 120 Hz Bayesopt CSVs when verified history already contains valid
        % cycle-pass points.
        if numel(candE) >= max(12, 4*round(double(opts.PhugoidHistoryRescueTopK)))
            break;
        end
    catch
        % A stale/partial parallel run must not block trim recovery.
    end
end

if isempty(candE)
    return;
end

rawPoints = table(candE, candT, candP, candG, ...
    'VariableNames', {'Elevator','Throttle','Pitch0','Gamma0'});
rawInfo = table(candRatio, candCsv, ...
    'VariableNames', {'gate_ratio','source_csv'});

% Best normalized gate margin first.  De-duplicate repeated deterministic
% evaluations of the same physical input pair before spending confirmation
% simulations.
[~, order] = sort(rawInfo.gate_ratio, 'ascend');
rawPoints = rawPoints(order,:);
rawInfo = rawInfo(order,:);
keys = compose("%.5f|%.5f", rawPoints.Elevator, rawPoints.Throttle);
[~, ia] = unique(keys, 'stable');
points = rawPoints(ia,:);
info = rawInfo(ia,:);
end

function ratio = local_phugoid_gate_ratio(standardMetrics, m, opts)
vals = [
    standardMetrics.fullHRms / double(opts.MaxFullAltitudeRmsM)
    standardMetrics.fullHMaxAbs / double(opts.MaxFullAltitudeMaxAbsM)
    standardMetrics.fullAirspeedRms / double(opts.MaxFullAirspeedRmsMps)
    standardMetrics.fullVzRms / double(opts.MaxFullVzRmsMps)
    standardMetrics.fullQRms / double(opts.MaxFullQRmsDps)
    m.hRms / double(opts.PhugoidConfirmMaxAltitudeRmsM)
    m.hMaxAbs / double(opts.PhugoidConfirmMaxAltitudeMaxAbsM)
    abs(m.hSlope) / double(opts.PhugoidConfirmMaxHeightSlopeMps)
    abs(m.vzMean) / double(opts.PhugoidConfirmMaxMeanVzMps)
    m.vzRms / double(opts.PhugoidConfirmMaxVzRmsMps)
    abs(m.qMean) / double(opts.PhugoidConfirmMaxMeanQDps)
    m.qRms / double(opts.PhugoidConfirmMaxQRmsDps)
    m.vaRms / double(opts.PhugoidConfirmMaxAirspeedRmsMps)
    m.pitchStd / double(opts.PhugoidConfirmMaxPitchStdDeg)
    m.vzDampingRatio / double(opts.PhugoidConfirmMaxVzDampingRatio)
    m.qDampingRatio / double(opts.PhugoidConfirmMaxQDampingRatio)
    ];
if any(~isfinite(vals))
    ratio = Inf;
else
    ratio = max(vals);
end
end

function x = local_history_median(T, name, fallback)
x = fallback;
if ~ismember(string(name), string(T.Properties.VariableNames))
    return;
end
try
    v = double(T.(char(name)));
    v = v(isfinite(v));
    if ~isempty(v)
        x = median(v);
    end
catch
end
end

function pitch0 = local_history_release_pitch(opts)
pitch0 = 0.0;
if ~isempty(opts.PreparationTrimBank)
    try
        pitch0 = local_trim_field(opts.PreparationTrimBank(1), "pitch_deg", pitch0);
    catch
    end
end
end

function gamma0 = local_history_release_gamma(opts)
gamma0 = 0.0;
if ~isempty(opts.PreparationTrimBank)
    try
        gamma0 = local_trim_field(opts.PreparationTrimBank(1), "initial_flight_path_deg", gamma0);
    catch
    end
end
end

function op = local_observed_phugoid_center(T, fallback, opts)
% Store the center of the last full phugoid cycle as the nominal operating
% point; do not use the instantaneous endpoint phase.
op = fallback;
if isempty(T) || height(T)==0, return; end
t = double(T.time_s(:));
valid = isfinite(t);
if ~any(valid), return; end
tMax = max(t(valid));
mask = valid & t >= tMax-double(opts.PhugoidConfirmWindowS);
if nnz(mask)<10, mask=valid; end
op.altitude_m = local_median_omitnan(T.altitude_m(mask));
op.airspeed_mps = local_median_omitnan(T.airspeed_mps(mask));
op.pitch_deg = local_median_omitnan(T.pitch_deg(mask));
op.vz_up_mps = local_mean_omitnan(T.vz_up_mps(mask));
op.q_dps = local_mean_omitnan(T.q_dps(mask));
if ismember("elevator_cmd_actual",string(T.Properties.VariableNames))
    op.elevator_cmd = local_median_omitnan(T.elevator_cmd_actual(mask));
elseif ismember("elevator_cmd",string(T.Properties.VariableNames))
    op.elevator_cmd = local_median_omitnan(T.elevator_cmd(mask));
end
if ismember("throttle_cmd_actual",string(T.Properties.VariableNames))
    op.throttle_cmd = local_median_omitnan(T.throttle_cmd_actual(mask));
elseif ismember("throttle_cmd",string(T.Properties.VariableNames))
    op.throttle_cmd = local_median_omitnan(T.throttle_cmd(mask));
end
ratio = double(T.vz_up_mps(mask))./max(abs(double(T.airspeed_mps(mask))),1e-6);
ratio = min(max(ratio,-1),1);
op.initial_flight_path_deg = local_median_omitnan(asind(ratio));
if ~isfinite(op.initial_flight_path_deg), op.initial_flight_path_deg=0.0; end
end

function s = local_linear_slope_trim(t,y)
t=double(t(:)); y=double(y(:));
m=isfinite(t)&isfinite(y);
if nnz(m)<3, s=Inf; return; end
t0=t(find(m,1,"first"));
p=polyfit(t(m)-t0,y(m),1);
s=p(1);
end

function v = local_mean_omitnan(x)
x=double(x(:)); x=x(isfinite(x));
if isempty(x), v=NaN; else, v=mean(x); end
end

function x = local_point_from_op(op, opts)
e = min(max(double(op.elevator_cmd), double(opts.ElevatorRange(1))), double(opts.ElevatorRange(2)));
th = min(max(double(op.throttle_cmd), double(opts.ThrottleRange(1))), double(opts.ThrottleRange(2)));
p = min(max(double(op.pitch_deg), double(opts.PitchRange(1))), double(opts.PitchRange(2)));
g = min(max(local_trim_field(op, "initial_flight_path_deg", 0.0), double(opts.FlightPathRange(1))), double(opts.FlightPathRange(2)));
alpha = p - g;
if alpha < double(opts.InitialAlphaRange(1))
    p = g + double(opts.InitialAlphaRange(1));
elseif alpha > double(opts.InitialAlphaRange(2))
    p = g + double(opts.InitialAlphaRange(2));
end
x = table(e, th, p, g, 'VariableNames', {'Elevator','Throttle','Pitch0','Gamma0'});
end

function value = local_trim_field(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isfinite(double(s.(name)))
    value = double(s.(name));
else
    value = double(fallback);
end
end

function local_check_input_consistency(T, x, opts, cfgId)
if isempty(T) || height(T) == 0
    error("AirdropX:AutoMPC:EmptyTrimRun", "Trim verification returned no samples for cfg%d.", cfgId);
end
if ismember("elevator_cmd_actual", string(T.Properties.VariableNames))
    e = double(T.elevator_cmd_actual);
elseif ismember("elevator_cmd", string(T.Properties.VariableNames))
    e = double(T.elevator_cmd);
else
    e = double(T.elevator_delta);
end
if ismember("throttle_cmd_actual", string(T.Properties.VariableNames))
    th = double(T.throttle_cmd_actual);
else
    th = double(T.throttle_cmd);
end
eMed = local_median_omitnan(e);
thMed = local_median_omitnan(th);
if ~isfinite(eMed) || abs(eMed - double(x.Elevator)) > double(opts.ElevatorInputTolerance)
    error("AirdropX:AutoMPC:TrimInputMismatch", ...
        "cfg%d elevator requested %.6f but observed median %.6f.", cfgId, double(x.Elevator), eMed);
end
if ~isfinite(thMed) || abs(thMed - double(x.Throttle)) > double(opts.ThrottleInputTolerance)
    error("AirdropX:AutoMPC:TrimInputMismatch", ...
        "cfg%d throttle requested %.6f but observed median %.6f.", cfgId, double(x.Throttle), thMed);
end
end

function metrics = local_trim_metrics(T, opts)
metrics = struct();
h = double(T.altitude_m(:));
v = double(T.airspeed_mps(:));
pitch = double(T.pitch_deg(:));
vz = double(T.vz_up_mps(:));
q = double(T.q_dps(:));
t = double(T.time_s(:));

valid = isfinite(t) & isfinite(h) & isfinite(v) & isfinite(pitch) & isfinite(vz) & isfinite(q);
if ~any(valid)
    metrics.minH = NaN;
    metrics.fullHRms = Inf;
    metrics.fullHMaxAbs = Inf;
    return;
end

% cfg1..cfg4 are reached by preparatory payload drops inside the ID model.
% Those drop transients are not the static trim problem; they are a later MPC
% closed-loop problem.  The latest v5 curves make this visible: cfg0 starts at
% t=0 and is excellent, cfg1 begins around 1 s after one prep drop, cfg2 around
% 1.5 s after two prep drops and carries a much larger phugoid.  Rebase time at
% the first sample of the requested fixed configuration and give that transient
% a short settling interval before judging the open-loop equilibrium.
cfgId = 0;
if ismember("config_id", string(T.Properties.VariableNames))
    cfgId = round(local_median_omitnan(T.config_id(valid)));
end
cfgStartS = min(t(valid), [], "omitnan");
tau = t - cfgStartS;
if cfgId > 0
    configSettleS = double(opts.ConfigSettleBaseS) + ...
        double(opts.ConfigSettlePerAdditionalDropS) * max(0, cfgId - 1);
else
    configSettleS = 0.0;
end
evalMask = valid & tau >= configSettleS;
if nnz(evalMask) < 5
    evalMask = valid;
end
steadyStartRelS = max(double(opts.RecordStartS), ...
    configSettleS + double(opts.MinSteadyAfterSettleS));
steadyMask = valid & tau >= steadyStartRelS;
if nnz(steadyMask) < 5
    steadyMask = evalMask;
end
earlyMask = evalMask & tau < steadyStartRelS;
prepMask = valid & tau < configSettleS;

hv = h(evalMask); vv = v(evalMask); vzv = vz(evalMask); qv = q(evalMask);
he = h(steadyMask); ve = v(steadyMask); pe = pitch(steadyMask); vze = vz(steadyMask); qe = q(steadyMask); te = t(steadyMask);

% Keep one altitude target for the entire trim-evaluation portion.  The
% preparation transient is monitored for safety but does not bias the static
% operating point. build_iddata later removes acquisition-altitude offset, so
% the resulting equilibrium can be re-anchored to the requested mission H.
hRef = double(opts.SearchAltitudeM);
metrics.hRef = hRef;
metrics.configId = cfgId;
metrics.configStartS = cfgStartS;
metrics.configSettleS = configSettleS;
metrics.minH = min(h(valid), [], "omitnan");
if any(prepMask)
    metrics.prepHMaxAbs = local_max_abs(h(prepMask) - hRef);
    metrics.prepVzRms = local_rms_omitnan(vz(prepMask));
    metrics.prepQRms = local_rms_omitnan(q(prepMask));
else
    metrics.prepHMaxAbs = 0.0;
    metrics.prepVzRms = 0.0;
    metrics.prepQRms = 0.0;
end

% "Full" now means the complete fixed-config trim-evaluation segment after
% preparation, not the payload-release impulse used only to reach cfg1..cfg4.
metrics.fullHRms = local_rms_omitnan(hv - hRef);
metrics.fullHMaxAbs = local_max_abs(hv - hRef);
metrics.fullAirspeedRms = local_rms_omitnan(vv - double(opts.TargetAirspeedMps));
metrics.fullVzRms = local_rms_omitnan(vzv);
metrics.fullQRms = local_rms_omitnan(qv);

% Steady/final window: strictest gate.
metrics.hMed = local_median_omitnan(he);
metrics.vaMed = local_median_omitnan(ve);
metrics.pitchMed = local_median_omitnan(pe);
metrics.pitchStd = local_std_omitnan(pe);
metrics.vzMed = local_median_omitnan(vze);
metrics.vzRms = local_rms_omitnan(vze);
metrics.qMed = local_median_omitnan(qe);
metrics.qRms = local_rms_omitnan(qe);
metrics.vaErr = metrics.vaMed - double(opts.TargetAirspeedMps);
metrics.vaRms = local_rms_omitnan(ve - double(opts.TargetAirspeedMps));
metrics.vaMaxAbs = local_max_abs(ve - double(opts.TargetAirspeedMps));
metrics.hRms = local_rms_omitnan(he - hRef);
metrics.hMaxAbs = local_max_abs(he - hRef);
[metrics.hStart, metrics.hEnd, metrics.hDrift] = local_edge_drift(he, opts.TrimEdgeFraction);

[~, ~, pitchDelta] = local_edge_drift(pe, opts.TrimEdgeFraction);
[timeStart, timeEnd, ~] = local_edge_drift(te, opts.TrimEdgeFraction);
dt = timeEnd - timeStart;
if isfinite(dt) && dt > 0
    metrics.pitchDriftDegps = pitchDelta / dt;
else
    metrics.pitchDriftDegps = Inf;
end


% Tail equilibrium window: a true trim must stop accelerating, not merely
% have a small integrated error.  This is the key v5 signal for separating
% "crosses 200 m" from "can actually remain at 200 m".
tMax = max(t(valid), [], "omitnan");
tailStart = max(double(opts.RecordStartS), tMax - double(opts.TailWindowS));
tailMask = valid & t >= tailStart;
if nnz(tailMask) < 5
    tailMask = steadyMask;
end
ht = h(tailMask); vt = v(tailMask); pt = pitch(tailMask);
vzt = vz(tailMask); qt = q(tailMask); tt = t(tailMask);
metrics.tailAirspeedRms = local_rms_omitnan(vt - double(opts.TargetAirspeedMps));
metrics.tailVzMed = local_median_omitnan(vzt);
metrics.tailVzRms = local_rms_omitnan(vzt);
metrics.tailQMed = local_median_omitnan(qt);
metrics.tailQRms = local_rms_omitnan(qt);
metrics.tailPitchStd = local_std_omitnan(pt);
metrics.tailHeightSlope = local_linear_slope(tt, ht);
metrics.tailAirspeedSlope = local_linear_slope(tt, vt);
metrics.tailPitchSlope = local_linear_slope(tt, pt);
metrics.tailVzSlope = local_linear_slope(tt, vzt);

% Early window is symmetric: both climb and descent are penalized. The latest
% cfg0 climbed about 14 m before 10 s while old earlyLoss was almost zero.
if any(earlyMask)
    hi = h(earlyMask); vi = v(earlyMask); vzi = vz(earlyMask); qi = q(earlyMask);
    earlyErr = hi - hRef;
    metrics.earlyHRms = local_rms_omitnan(earlyErr);
    metrics.earlyHMaxAbs = local_max_abs(earlyErr);
    metrics.earlyAltitudeLoss = max(0.0, hRef - min(hi, [], "omitnan"));
    metrics.earlyAltitudeGain = max(0.0, max(hi, [], "omitnan") - hRef);
    metrics.earlyAirspeedRms = local_rms_omitnan(vi - double(opts.TargetAirspeedMps));
    metrics.earlyVzRms = local_rms_omitnan(vzi);
    metrics.earlyQRms = local_rms_omitnan(qi);
    ti = t(earlyMask);
    metrics.earlyQAccelDps2 = local_linear_slope(ti, qi);
    metrics.earlyVaAccelMps2 = local_linear_slope(ti, vi);
else
    metrics.earlyHRms = 0.0;
    metrics.earlyHMaxAbs = 0.0;
    metrics.earlyAltitudeLoss = 0.0;
    metrics.earlyAltitudeGain = 0.0;
    metrics.earlyAirspeedRms = 0.0;
    metrics.earlyVzRms = 0.0;
    metrics.earlyQRms = 0.0;
    metrics.earlyQAccelDps2 = 0.0;
    metrics.earlyVaAccelMps2 = 0.0;
end
end

function tf = local_trim_catastrophic_fail(m, opts)
tf = ~isfinite(m.minH) || m.minH < double(opts.HardFloorAltitudeM) || ...
    ~isfinite(m.fullHRms) || ~isfinite(m.fullHMaxAbs) || ...
    ~isfinite(m.hRms) || ~isfinite(m.vaRms) || ~isfinite(m.vzRms) || ~isfinite(m.qRms) || ...
    ~isfinite(m.tailVzMed) || ~isfinite(m.tailQMed) || ~isfinite(m.tailAirspeedRms) || ...
    ~isfinite(m.tailHeightSlope) || ~isfinite(m.tailAirspeedSlope) || ~isfinite(m.tailPitchSlope) || ...
    ~isfinite(m.pitchStd) || ~isfinite(m.pitchDriftDegps) || ...
    m.fullQRms > double(opts.CatastrophicQRmsDps) || ...
    m.pitchStd > double(opts.CatastrophicPitchStdDeg) || ...
    ~isfinite(m.pitchMed) || m.pitchMed < double(opts.AcceptPitchRange(1)) || m.pitchMed > double(opts.AcceptPitchRange(2));
end

function tf = local_trim_accept_fail(m, opts)
tf = local_trim_catastrophic_fail(m, opts) || ...
    m.fullHRms > double(opts.MaxFullAltitudeRmsM) || ...
    m.fullHMaxAbs > double(opts.MaxFullAltitudeMaxAbsM) || ...
    m.earlyHRms > double(opts.MaxEarlyAltitudeRmsM) || ...
    m.earlyHMaxAbs > double(opts.MaxEarlyAltitudeMaxAbsM) || ...
    m.hRms > double(opts.MaxTrimAltitudeRmsM) || ...
    m.hMaxAbs > double(opts.MaxTrimAltitudeMaxAbsM) || ...
    abs(m.hDrift) > double(opts.MaxTrimAltitudeDriftM) || ...
    m.fullAirspeedRms > double(opts.MaxFullAirspeedRmsMps) || ...
    m.vaRms > double(opts.MaxTrimAirspeedRmsMps) || ...
    m.vaMaxAbs > double(opts.MaxTrimAirspeedMaxAbsMps) || ...
    abs(m.vzMed) > double(opts.MaxTrimAbsVzMps) || ...
    m.fullVzRms > double(opts.MaxFullVzRmsMps) || ...
    m.vzRms > double(opts.MaxTrimVzRmsMps) || ...
    m.fullQRms > double(opts.MaxFullQRmsDps) || ...
    m.qRms > double(opts.MaxTrimQRmsDps) || ...
    abs(m.tailVzMed) > double(opts.MaxTailAbsVzMps) || ...
    abs(m.tailQMed) > double(opts.MaxTailAbsQDps) || ...
    m.tailAirspeedRms > double(opts.MaxTailAirspeedRmsMps) || ...
    abs(m.tailHeightSlope) > double(opts.MaxTailHeightSlopeMps) || ...
    abs(m.tailAirspeedSlope) > double(opts.MaxTailAirspeedSlopeMps2) || ...
    abs(m.tailPitchSlope) > double(opts.MaxTailPitchSlopeDegps) || ...
    m.pitchStd > double(opts.MaxTrimPitchStdDeg) || ...
    abs(m.pitchDriftDegps) > double(opts.MaxTrimPitchDriftDegps);
end

function score = local_trim_soft_score(m, opts)
score = ...
    double(opts.FullAltitudeRmsWeight) * (m.fullHRms / double(opts.FullAltitudeRmsScaleM)).^2 + ...
    double(opts.FullAltitudeMaxWeight) * (m.fullHMaxAbs / double(opts.FullAltitudeMaxScaleM)).^2 + ...
    double(opts.EarlyAltitudeRmsWeight) * (m.earlyHRms / double(opts.EarlyAltitudeRmsScaleM)).^2 + ...
    double(opts.EarlyAltitudeMaxWeight) * (m.earlyHMaxAbs / double(opts.EarlyAltitudeMaxScaleM)).^2 + ...
    double(opts.TrimAltitudeRmsWeight) * (m.hRms / double(opts.TrimAltitudeRmsScaleM)).^2 + ...
    double(opts.TrimAltitudeMaxWeight) * (m.hMaxAbs / double(opts.TrimAltitudeMaxScaleM)).^2 + ...
    double(opts.TrimAltitudeDriftWeight) * (abs(m.hDrift) / double(opts.TrimAltitudeDriftScaleM)).^2 + ...
    double(opts.FullAirspeedWeight) * (m.fullAirspeedRms / double(opts.FullAirspeedScaleMps)).^2 + ...
    double(opts.EarlyAirspeedWeight) * (m.earlyAirspeedRms / double(opts.EarlyAirspeedScaleMps)).^2 + ...
    double(opts.TrimAirspeedWeight) * (m.vaRms / double(opts.TrimAirspeedScaleMps)).^2 + ...
    double(opts.FullVzWeight) * (m.fullVzRms / double(opts.FullVzScaleMps)).^2 + ...
    double(opts.EarlyVzWeight) * (m.earlyVzRms / double(opts.EarlyVzScaleMps)).^2 + ...
    double(opts.TrimVzWeight) * (m.vzRms / double(opts.TrimVzScaleMps)).^2 + ...
    double(opts.FullQWeight) * (m.fullQRms / double(opts.FullQScaleDps)).^2 + ...
    double(opts.EarlyQWeight) * (m.earlyQRms / double(opts.EarlyQScaleDps)).^2 + ...
    double(opts.TrimQWeight) * (m.qRms / double(opts.TrimQScaleDps)).^2 + ...
    double(opts.TailVzMedWeight) * (abs(m.tailVzMed) / double(opts.TailVzMedScaleMps)).^2 + ...
    double(opts.TailQMedWeight) * (abs(m.tailQMed) / double(opts.TailQMedScaleDps)).^2 + ...
    double(opts.TailAirspeedWeight) * (m.tailAirspeedRms / double(opts.TailAirspeedScaleMps)).^2 + ...
    double(opts.TailHeightSlopeWeight) * (abs(m.tailHeightSlope) / double(opts.TailHeightSlopeScaleMps)).^2 + ...
    double(opts.TailAirspeedSlopeWeight) * (abs(m.tailAirspeedSlope) / double(opts.TailAirspeedSlopeScaleMps2)).^2 + ...
    double(opts.TailPitchSlopeWeight) * (abs(m.tailPitchSlope) / double(opts.TailPitchSlopeScaleDegps)).^2 + ...
    double(opts.TrimPitchStdWeight) * (m.pitchStd / double(opts.TrimPitchStdScaleDeg)).^2 + ...
    double(opts.TrimPitchDriftWeight) * (abs(m.pitchDriftDegps) / double(opts.TrimPitchDriftScaleDegps)).^2;
if ~isfinite(score)
    score = double(opts.HardFailScore);
end
end


function score = local_equilibrium_score(m, opts)
% Stage-A score: first solve the actual open-loop equilibrium. Integrated
% altitude error is retained at reduced weight, while the last few seconds
% must have near-zero vertical/pitch rates and near-zero state slopes.
base = local_trim_soft_score(m, opts);
score = double(opts.EquilibriumTrajectoryBlend) * base + ...
    double(opts.EqTailVzMedWeight) * (abs(m.tailVzMed) / double(opts.EqTailVzMedScaleMps)).^2 + ...
    double(opts.EqTailVzRmsWeight) * (m.tailVzRms / double(opts.EqTailVzRmsScaleMps)).^2 + ...
    double(opts.EqTailQMedWeight) * (abs(m.tailQMed) / double(opts.EqTailQMedScaleDps)).^2 + ...
    double(opts.EqTailQRmsWeight) * (m.tailQRms / double(opts.EqTailQRmsScaleDps)).^2 + ...
    double(opts.EqTailAirspeedWeight) * (m.tailAirspeedRms / double(opts.EqTailAirspeedScaleMps)).^2 + ...
    double(opts.EqTailHeightSlopeWeight) * (abs(m.tailHeightSlope) / double(opts.EqTailHeightSlopeScaleMps)).^2 + ...
    double(opts.EqTailAirspeedSlopeWeight) * (abs(m.tailAirspeedSlope) / double(opts.EqTailAirspeedSlopeScaleMps2)).^2 + ...
    double(opts.EqTailPitchSlopeWeight) * (abs(m.tailPitchSlope) / double(opts.EqTailPitchSlopeScaleDegps)).^2 + ...
    double(opts.EqTailVzSlopeWeight) * (abs(m.tailVzSlope) / double(opts.EqTailVzSlopeScaleMps2)).^2 + ...
    double(opts.EqEarlyQAccelWeight) * (abs(m.earlyQAccelDps2) / double(opts.EqEarlyQAccelScaleDps2)).^2 + ...
    double(opts.EqEarlyVaAccelWeight) * (abs(m.earlyVaAccelMps2) / double(opts.EqEarlyVaAccelScaleMps2)).^2;
if ~isfinite(score)
    score = double(opts.HardFailScore);
end
end

function score = local_recovery_score(m, opts)
% Optimize the actual failed gates, not only the weighted trajectory score.
% The smooth gate penalty tells bayesopt whether an infeasible point is
% getting closer to acceptance instead of mapping every near miss to 1e8.
base = local_trim_soft_score(m, opts);
eq = local_equilibrium_score(m, opts);
ratio = local_trim_gate_ratio(m, opts);
violation = max(0.0, ratio - 1.0);
score = base + double(opts.RecoveryEquilibriumBlend) * eq + ...
    double(opts.RecoveryGatePenaltyWeight) * violation.^2;
if ~isfinite(score)
    score = double(opts.HardFailScore);
end
end

function score = local_tail_rescue_objective(u, fixedPoint, cfgId, trim, opts)
try
    point = local_tail_point(u, fixedPoint, opts);
    r = local_run_candidate(point, cfgId, trim, opts, "tail_rescue_search");
    T = r.timeseries;
    if height(T) < 3
        score = Inf;
        return;
    end
    local_check_input_consistency(T, point, opts, cfgId);
    m = local_trim_metrics(T, opts);
    if local_trim_catastrophic_fail(m, opts)
        score = double(opts.HardFailScore) + local_tail_equilibrium_score(m, opts);
        return;
    end
    ratio = local_tail_gate_ratio(m, opts);
    violation = max(0.0, ratio - 1.0);
    score = local_tail_equilibrium_score(m, opts) + ...
        double(opts.TailRescueGatePenaltyWeight) * violation.^2;
catch ME
    warning("AirdropX:AutoMPC:TailRescueEvalFailed", "Tail rescue failed: %s", ME.message);
    score = Inf;
end
end

function score = local_tail_equilibrium_score(m, opts)
% Static equilibrium score.  Deliberately ignores the preparatory-drop
% trajectory except for catastrophic safety checks; cfg2+ may carry a long
% phugoid that is not part of the steady trim itself.
score = ...
    double(opts.TailRescueVzMedWeight) * (abs(m.tailVzMed) / double(opts.TailRescueVzMedScaleMps)).^2 + ...
    double(opts.TailRescueVzRmsWeight) * (m.tailVzRms / double(opts.TailRescueVzRmsScaleMps)).^2 + ...
    double(opts.TailRescueQMedWeight) * (abs(m.tailQMed) / double(opts.TailRescueQMedScaleDps)).^2 + ...
    double(opts.TailRescueQRmsWeight) * (m.tailQRms / double(opts.TailRescueQRmsScaleDps)).^2 + ...
    double(opts.TailRescueAirspeedWeight) * (m.tailAirspeedRms / double(opts.TailRescueAirspeedScaleMps)).^2 + ...
    double(opts.TailRescuePitchStdWeight) * (m.tailPitchStd / double(opts.TailRescuePitchStdScaleDeg)).^2 + ...
    double(opts.TailRescueHeightSlopeWeight) * (abs(m.tailHeightSlope) / double(opts.TailRescueHeightSlopeScaleMps)).^2 + ...
    double(opts.TailRescueAirspeedSlopeWeight) * (abs(m.tailAirspeedSlope) / double(opts.TailRescueAirspeedSlopeScaleMps2)).^2 + ...
    double(opts.TailRescuePitchSlopeWeight) * (abs(m.tailPitchSlope) / double(opts.TailRescuePitchSlopeScaleDegps)).^2 + ...
    double(opts.TailRescueVzSlopeWeight) * (abs(m.tailVzSlope) / double(opts.TailRescueVzSlopeScaleMps2)).^2;
if ~isfinite(score)
    score = double(opts.HardFailScore);
end
end

function tf = local_tail_rescue_needed(m, opts)
tf = local_tail_gate_ratio(m, opts) > double(opts.TailRescueTriggerRatio);
end

function tf = local_tail_accept_fail(m, opts)
tf = local_trim_catastrophic_fail(m, opts) || ...
    abs(m.tailVzMed) > double(opts.TailRescueMaxAbsVzMps) || ...
    m.tailVzRms > double(opts.TailRescueMaxVzRmsMps) || ...
    abs(m.tailQMed) > double(opts.TailRescueMaxAbsQDps) || ...
    m.tailQRms > double(opts.TailRescueMaxQRmsDps) || ...
    m.tailAirspeedRms > double(opts.TailRescueMaxAirspeedRmsMps) || ...
    m.tailPitchStd > double(opts.TailRescueMaxPitchStdDeg) || ...
    abs(m.tailHeightSlope) > double(opts.TailRescueMaxHeightSlopeMps) || ...
    abs(m.tailAirspeedSlope) > double(opts.TailRescueMaxAirspeedSlopeMps2) || ...
    abs(m.tailPitchSlope) > double(opts.TailRescueMaxPitchSlopeDegps);
end

function ratio = local_tail_gate_ratio(m, opts)
if local_trim_catastrophic_fail(m, opts)
    ratio = Inf;
    return;
end
ratios = [ ...
    abs(m.tailVzMed) / double(opts.TailRescueMaxAbsVzMps); ...
    m.tailVzRms / double(opts.TailRescueMaxVzRmsMps); ...
    abs(m.tailQMed) / double(opts.TailRescueMaxAbsQDps); ...
    m.tailQRms / double(opts.TailRescueMaxQRmsDps); ...
    m.tailAirspeedRms / double(opts.TailRescueMaxAirspeedRmsMps); ...
    m.tailPitchStd / double(opts.TailRescueMaxPitchStdDeg); ...
    abs(m.tailHeightSlope) / double(opts.TailRescueMaxHeightSlopeMps); ...
    abs(m.tailAirspeedSlope) / double(opts.TailRescueMaxAirspeedSlopeMps2); ...
    abs(m.tailPitchSlope) / double(opts.TailRescueMaxPitchSlopeDegps)];
ratios = ratios(isfinite(ratios));
if isempty(ratios), ratio = Inf; else, ratio = max(ratios); end
end

function bounds = local_tail_rescue_bounds(best, op, cfgId, trimBank, opts)
bestRow = [double(best.Elevator), double(best.Throttle), double(best.Pitch0), double(best.Gamma0)];
opRow = [double(op.elevator_cmd), double(op.throttle_cmd), double(op.pitch_deg), ...
    local_trim_field(op, "initial_flight_path_deg", 0.0)];
pred = local_continuation_row(cfgId, trimBank, opts);
predRow = pred(1:4);
center = median([bestRow; opRow; predRow], 1);
half = [double(opts.TailRescueElevatorHalfWidth), double(opts.TailRescueThrottleHalfWidth), ...
    double(opts.TailRescuePitchHalfWidthDeg), double(opts.TailRescueGammaHalfWidthDeg)];
globalBounds = [
    double(opts.ElevatorRange(:)).'
    double(opts.ThrottleRange(:)).'
    double(opts.PitchRange(:)).'
    double(opts.FlightPathRange(:)).'
    ];
bounds = [center(:) - half(:), center(:) + half(:)];
for k = 1:4
    bounds(k,1) = max(bounds(k,1), globalBounds(k,1));
    bounds(k,2) = min(bounds(k,2), globalBounds(k,2));
    if bounds(k,2) <= bounds(k,1), bounds(k,:) = globalBounds(k,:); end
end
bounds = local_freeze_preparation_state_bounds(bounds, cfgId, opts);
end

function X = local_tail_rescue_initial_x(best, op, cfgId, trimBank, bounds, opts)
bestRow = [double(best.Elevator), double(best.Throttle), double(best.Pitch0), double(best.Gamma0)];
opRow = [double(op.elevator_cmd), double(op.throttle_cmd), double(op.pitch_deg), ...
    local_trim_field(op, "initial_flight_path_deg", 0.0)];
pred = local_continuation_row(cfgId, trimBank, opts);
predRow = pred(1:4);
c = median([bestRow; opRow; predRow], 1);
de = double(opts.TailRescueSeedElevatorStep);
dt = double(opts.TailRescueSeedThrottleStep);
dp = double(opts.TailRescueSeedPitchStepDeg);
dg = double(opts.TailRescueSeedGammaStepDeg);
rows = [
    bestRow; opRow; predRow; c; ...
    c + [ de, 0, 0, 0]; c + [-de, 0, 0, 0]; ...
    c + [0, dt, 0, 0]; c + [0, -dt, 0, 0]; ...
    c + [0, 0, dp, 0]; c + [0, 0, -dp, 0]; ...
    c + [0, 0, 0, dg]; c + [0, 0, 0, -dg]; ...
    c + [ de, -dt, 0, 0]; c + [-de, dt, 0, 0]; ...
    c + [0, 0, dp, -dg]; c + [0, 0, -dp, dg]];
for k = 1:4
    rows(:,k) = min(max(rows(:,k), bounds(k,1)), bounds(k,2));
end
for i = 1:size(rows,1)
    alpha = rows(i,3) - rows(i,4);
    if alpha < double(opts.InitialAlphaRange(1))
        rows(i,3) = rows(i,4) + double(opts.InitialAlphaRange(1));
    elseif alpha > double(opts.InitialAlphaRange(2))
        rows(i,3) = rows(i,4) + double(opts.InitialAlphaRange(2));
    end
    rows(i,3) = min(max(rows(i,3), bounds(3,1)), bounds(3,2));
end
rows = unique(rows, "rows", "stable");
X = array2table(rows, 'VariableNames', {'Elevator','Throttle','Pitch0','Gamma0'});
end

function point = local_tail_point(u, fixedPoint, opts)
e = min(max(double(u.Elevator), double(opts.ElevatorRange(1))), double(opts.ElevatorRange(2)));
th = min(max(double(u.Throttle), double(opts.ThrottleRange(1))), double(opts.ThrottleRange(2)));
if istable(u) && ismember("Pitch0", string(u.Properties.VariableNames))
    p = min(max(double(u.Pitch0), double(opts.PitchRange(1))), double(opts.PitchRange(2)));
else
    p = double(fixedPoint.Pitch0);
end
if istable(u) && ismember("Gamma0", string(u.Properties.VariableNames))
    g = min(max(double(u.Gamma0), double(opts.FlightPathRange(1))), double(opts.FlightPathRange(2)));
else
    g = double(fixedPoint.Gamma0);
end
point = table(e, th, p, g, 'VariableNames', {'Elevator','Throttle','Pitch0','Gamma0'});
end

function tailOpts = local_tail_metric_opts(opts)
tailOpts = opts;
tailOpts.TailWindowS = max(double(opts.TailWindowS), double(opts.TailRescueTailWindowS));
tailOpts.ObservedOpTailWindowS = max(double(opts.ObservedOpTailWindowS), double(opts.TailRescueTailWindowS));
end

function ratio = local_trim_gate_ratio(m, opts)
if local_trim_catastrophic_fail(m, opts)
    ratio = Inf;
    return;
end
ratios = [
    m.fullHRms / double(opts.MaxFullAltitudeRmsM)
    m.fullHMaxAbs / double(opts.MaxFullAltitudeMaxAbsM)
    m.earlyHRms / double(opts.MaxEarlyAltitudeRmsM)
    m.earlyHMaxAbs / double(opts.MaxEarlyAltitudeMaxAbsM)
    m.hRms / double(opts.MaxTrimAltitudeRmsM)
    m.hMaxAbs / double(opts.MaxTrimAltitudeMaxAbsM)
    abs(m.hDrift) / double(opts.MaxTrimAltitudeDriftM)
    m.fullAirspeedRms / double(opts.MaxFullAirspeedRmsMps)
    m.vaRms / double(opts.MaxTrimAirspeedRmsMps)
    m.vaMaxAbs / double(opts.MaxTrimAirspeedMaxAbsMps)
    abs(m.vzMed) / double(opts.MaxTrimAbsVzMps)
    m.fullVzRms / double(opts.MaxFullVzRmsMps)
    m.vzRms / double(opts.MaxTrimVzRmsMps)
    m.fullQRms / double(opts.MaxFullQRmsDps)
    m.qRms / double(opts.MaxTrimQRmsDps)
    abs(m.tailVzMed) / double(opts.MaxTailAbsVzMps)
    abs(m.tailQMed) / double(opts.MaxTailAbsQDps)
    m.tailAirspeedRms / double(opts.MaxTailAirspeedRmsMps)
    abs(m.tailHeightSlope) / double(opts.MaxTailHeightSlopeMps)
    abs(m.tailAirspeedSlope) / double(opts.MaxTailAirspeedSlopeMps2)
    abs(m.tailPitchSlope) / double(opts.MaxTailPitchSlopeDegps)
    m.pitchStd / double(opts.MaxTrimPitchStdDeg)
    abs(m.pitchDriftDegps) / double(opts.MaxTrimPitchDriftDegps)
    ];
ratios = ratios(isfinite(ratios));
if isempty(ratios)
    ratio = Inf;
else
    ratio = max(ratios);
end
end

function tf = local_initial_state_constraint(X, opts)
alpha = double(X.Pitch0) - double(X.Gamma0);
tf = isfinite(alpha) & alpha >= double(opts.InitialAlphaRange(1)) & alpha <= double(opts.InitialAlphaRange(2));
end

function [seedTable, mapTable] = local_joint_control_map(cfgId, trimBank, opts)
% Short deterministic map in the two real steady-control dimensions.  For
% cfg1+ Pitch0/Gamma0 are preparation states and are therefore fixed; the
% equilibrium variables are elevator and throttle.
seedTable = table(zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1), ...
    'VariableNames',{'Elevator','Throttle','Pitch0','Gamma0'});
mapTable = table();
try
    globalE = double(opts.ElevatorRange(:)).';
    globalT = double(opts.ThrottleRange(:)).';
    pred = local_continuation_row(cfgId, trimBank, opts);
    prevE = pred(1);
    if cfgId > 0
        try, prevE = double(trimBank(cfgId).elevator_cmd); catch, end
    end
    eLo = max(globalE(1), min(prevE,pred(1))-double(opts.JointMapElevatorSpan));
    eHi = min(globalE(2), max(prevE,pred(1))+double(opts.JointMapElevatorSpan));
    if eHi-eLo < 0.20
        c = 0.5*(eHi+eLo); eLo=max(globalE(1),c-0.10); eHi=min(globalE(2),c+0.10);
    end
    tr = double(opts.JointMapThrottleRange(:)).';
    tLo=max(globalT(1),tr(1)); tHi=min(globalT(2),tr(2));
    eVals=linspace(eLo,eHi,max(3,round(double(opts.JointMapElevatorSamples))));
    tVals=linspace(tLo,tHi,max(3,round(double(opts.JointMapThrottleSamples))));
    p0=pred(3); g0=pred(4);
    if cfgId>0 && ~isempty(opts.PreparationTrimBank)
        p0=local_trim_field(opts.PreparationTrimBank(1),'pitch_deg',p0);
        g0=local_trim_field(opts.PreparationTrimBank(1),'initial_flight_path_deg',g0);
    end
    [EE,TT]=ndgrid(eVals,tVals); n=numel(EE);
    rows=cell(n,1);
    usePar=logical(opts.UseParallel);
    if usePar
        parfor ii=1:n
            rows{ii}=local_joint_map_one(EE(ii),TT(ii),p0,g0,cfgId,trimBank,opts,ii);
        end
    else
        for ii=1:n
            rows{ii}=local_joint_map_one(EE(ii),TT(ii),p0,g0,cfgId,trimBank,opts,ii);
        end
    end
    mapTable=struct2table(vertcat(rows{:}));
    valid=logical(mapTable.run_ok) & isfinite(mapTable.score);
    if ~any(valid), return; end
    V=mapTable(valid,:); [~,ord]=sort(double(V.score),'ascend'); V=V(ord,:);
    k=min(height(V),max(1,round(double(opts.JointMapTopK)))); V=V(1:k,:);
    seedRows=[double(V.Elevator),double(V.Throttle),repmat(p0,k,1),repmat(g0,k,1)];
    % Fit local bilinear qdot/Vdot surfaces and add their predicted joint-zero
    % point.  This is only a seed; every candidate is still simulated.
    P=mapTable(valid,:);
    if height(P)>=6
        X=[ones(height(P),1),double(P.Elevator),double(P.Throttle),double(P.Elevator).*double(P.Throttle)];
        yq=double(P.qdot_early_deg_s2); yv=double(P.vdot_early_mps2);
        good=all(isfinite(X),2)&isfinite(yq)&isfinite(yv);
        if nnz(good)>=6
            bq=X(good,:)\yq(good); bv=X(good,:)\yv(good);
            start=[double(V.Elevator(1)),double(V.Throttle(1))];
            obj=@(z)local_joint_surrogate_cost(z,bq,bv,[eLo eHi],[tLo tHi],opts);
            z=fminsearch(obj,start,optimset('Display','off','MaxIter',100,'TolX',1e-5));
            z(1)=min(max(z(1),eLo),eHi); z(2)=min(max(z(2),tLo),tHi);
            seedRows=[z(1),z(2),p0,g0;seedRows]; %#ok<AGROW>
        end
    end
    seedRows=unique(seedRows,'rows','stable');
    seedTable=array2table(seedRows,'VariableNames',{'Elevator','Throttle','Pitch0','Gamma0'});
catch ME
    warning('AirdropX:AutoMPC:JointMapFailed','Joint control map failed for cfg%d: %s',cfgId,ME.message);
end
end

function c = local_joint_surrogate_cost(z,bq,bv,eBounds,tBounds,opts)
e=min(max(double(z(1)),eBounds(1)),eBounds(2));
t=min(max(double(z(2)),tBounds(1)),tBounds(2));
x=[1 e t e*t];
c=(x*bq/double(opts.JointMapQdotScaleDps2))^2 + ...
  (x*bv/double(opts.JointMapVdotScaleMps2))^2 + ...
  20*((double(z(1))-e)^2+(double(z(2))-t)^2);
end

function row = local_joint_map_one(elev,thr,p0,g0,cfgId,trimBank,opts,idx)
row=struct('index',idx,'Elevator',elev,'Throttle',thr,'physical_elevator',NaN, ...
    'qdot_early_deg_s2',NaN,'vdot_early_mps2',NaN,'q_mid_dps',NaN,'vz_mid_mps',NaN, ...
    'va_error_mid_mps',NaN,'tail_va_error_mps',NaN,'tail_vz_mps',NaN,'tail_q_dps',NaN, ...
    'tail_h_slope_mps',NaN,'tail_va_slope_mps2',NaN,'score',Inf,'run_ok',false,'error_message',"");
try
    x=table(elev,thr,p0,g0,'VariableNames',{'Elevator','Throttle','Pitch0','Gamma0'});
    trim=trimBank(cfgId+1); trim.elevator_cmd=elev; trim.throttle_cmd=thr; trim.pitch_deg=p0;
    trim.altitude_m=double(opts.SearchAltitudeM);
    cfgReach=double(opts.PrepDropStartS)+double(opts.JointMapPrepDropIntervalS)*max(0,cfgId-1);
    stop=max(double(opts.JointMapMinStopTimeS),cfgReach+double(opts.JointMapPostConfigObserveS));
    runId=sprintf('trim_jointmap_cfg%d_%03d_%s',cfgId,idx,char(string(datetime('now','Format','HHmmssSSS'))));
    r=airdropx_auto_run_id_experiment('ProjectRoot',opts.ProjectRoot,'Model',opts.Model,'RunId',runId, ...
        'ConfigId',cfgId,'Trim',trim,'StopTimeS',stop,'OutputRoot',local_eval_output_root(opts,cfgId,runId), ...
        'RecordStartS',0,'ExportStartS',0,'InitialAirspeedMps',opts.TargetAirspeedMps, ...
        'TargetAltitudeM',opts.SearchAltitudeM,'TargetAirspeedMps',opts.TargetAirspeedMps, ...
        'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg, ...
        'PrepDropStartS',opts.PrepDropStartS,'PrepDropIntervalS',opts.JointMapPrepDropIntervalS, ...
        'InitialAltitudeM',opts.SearchAltitudeM,'InitialPitchDeg',local_preparation_initial_pitch(x,cfgId,opts), ...
        'InitialFlightPathDeg',local_preparation_initial_gamma(x,cfgId,opts), ...
        'ElevatorAmplitude',0,'ThrottleAmplitude',0,'ExcitationStartS',stop+1, ...
        'PreparationTrimBank',opts.PreparationTrimBank,'UsePreparationTrimSchedule',opts.UsePreparationTrimSchedule, ...
        'KeepFixedConfigurationOnly',true,'DirectIdMode',true);
    T=r.timeseries; if isempty(T)||height(T)<8, return; end
    local_check_input_consistency(T,x,opts,cfgId);
    t=double(T.time_s(:)); t=t-t(1); q=double(T.q_dps(:)); va=double(T.airspeed_mps(:));
    vz=double(T.vz_up_mps(:)); h=double(T.altitude_m(:));
    dt=median(diff(t),'omitnan'); if ~isfinite(dt)||dt<=0,dt=1/120;end
    win=max(3,2*floor((double(opts.JointMapSmoothWindowS)/dt)/2)+1);
    if numel(q)>=win,q=movmean(q,win,'omitnan');va=movmean(va,win,'omitnan');end
    qdot=gradient(q,t); vdot=gradient(va,t);
    early=t>=double(opts.JointMapEarlyWindowS(1))&t<=double(opts.JointMapEarlyWindowS(2));
    mid=t>=double(opts.JointMapMidWindowS(1))&t<=double(opts.JointMapMidWindowS(2));
    if nnz(early)<3,early=t<=min(max(t),0.30);end
    if nnz(mid)<3,mid=t<=min(max(t),0.80);end
    tail=t>=max(0,max(t)-double(opts.JointMapTailWindowS));
    row.qdot_early_deg_s2=median(qdot(early),'omitnan');
    row.vdot_early_mps2=median(vdot(early),'omitnan');
    row.q_mid_dps=median(q(mid),'omitnan'); row.vz_mid_mps=median(vz(mid),'omitnan');
    row.va_error_mid_mps=median(va(mid)-double(opts.TargetAirspeedMps),'omitnan');
    row.tail_va_error_mps=median(va(tail)-double(opts.TargetAirspeedMps),'omitnan');
    row.tail_vz_mps=median(vz(tail),'omitnan');
    row.tail_q_dps=median(q(tail),'omitnan');
    row.tail_h_slope_mps=local_linear_slope(t(tail),h(tail));
    row.tail_va_slope_mps2=local_linear_slope(t(tail),va(tail));
    if ismember('elevator_cmd_norm',string(T.Properties.VariableNames))
        row.physical_elevator=median(double(T.elevator_cmd_norm(early)),'omitnan');
    end
    row.score=6*(row.qdot_early_deg_s2/double(opts.JointMapQdotScaleDps2))^2 + ...
        5*(row.vdot_early_mps2/double(opts.JointMapVdotScaleMps2))^2 + ...
        2*(row.q_mid_dps/double(opts.JointMapQScaleDps))^2 + ...
        3*(row.vz_mid_mps/double(opts.JointMapVzScaleMps))^2 + ...
        2*(row.va_error_mid_mps/double(opts.JointMapVaErrorScaleMps))^2 + ...
        2*(row.tail_va_error_mps/double(opts.JointMapVaErrorScaleMps))^2 + ...
        4*(row.tail_vz_mps/double(opts.JointMapTailVzScaleMps))^2 + ...
        3*(row.tail_q_dps/double(opts.JointMapTailQScaleDps))^2 + ...
        4*(row.tail_h_slope_mps/double(opts.JointMapTailHeightSlopeScaleMps))^2 + ...
        2*(row.tail_va_slope_mps2/double(opts.JointMapTailVaSlopeScaleMps2))^2;
    if ~all(isfinite([row.qdot_early_deg_s2,row.vdot_early_mps2,row.q_mid_dps,row.vz_mid_mps, ...
            row.tail_vz_mps,row.tail_q_dps,row.tail_h_slope_mps,row.tail_va_slope_mps2,row.score]))
        row.score=Inf;
    end
    if max(abs(double(T.q_dps)),[],'omitnan')>45 || max(abs(double(T.pitch_deg)),[],'omitnan')>60 || ...
            min(double(T.airspeed_mps),[],'omitnan')<15 || min(double(T.altitude_m),[],'omitnan')<=double(opts.HardFloorAltitudeM)
        row.score=row.score+1e4;
    end
    row.run_ok=true;
catch ME
    row.error_message=string(ME.identifier)+": "+string(ME.message);
end
end

function bounds = local_focused_bounds(cfgId, trimBank, opts, jointSeeds)
globalBounds = [
    double(opts.ElevatorRange(:)).'
    double(opts.ThrottleRange(:)).'
    double(opts.PitchRange(:)).'
    double(opts.FlightPathRange(:)).'
    ];

if cfgId == 0 && logical(opts.UseFocusedCfg0Search)
    bounds = [
        double(opts.Cfg0FocusedElevatorRange(:)).'
        double(opts.Cfg0FocusedThrottleRange(:)).'
        double(opts.Cfg0FocusedPitchRange(:)).'
        double(opts.Cfg0FocusedFlightPathRange(:)).'
        ];
else
    % Continue the verified trim sequence across payload configurations.
    % With cfg0/cfg1 available, linearly extrapolate the next operating point
    % instead of centering every search directly on the previous cfg.  The
    % latest data show a smooth input trend while cfg2 was under-corrected.
    continuationCenter = local_continuation_row(cfgId, trimBank, opts).';
    center = continuationCenter;
    if isfield(trimBank, "resume_seed_valid") && logical(trimBank(cfgId + 1).resume_seed_valid)
        cur = trimBank(cfgId + 1);
        resumeCenter = [double(cur.elevator_cmd); double(cur.throttle_cmd); double(cur.pitch_deg); ...
            local_trim_field(cur, "initial_flight_path_deg", 0.0)];
        % Stable finite near-misses may still accelerate convergence, but they
        % may not exclude the verified-sequence continuation branch.
        center = 0.75 * resumeCenter + 0.25 * continuationCenter;
    end
    span = [
        double(opts.FocusedElevatorHalfWidth)
        double(opts.FocusedThrottleHalfWidth)
        double(opts.FocusedPitchHalfWidthDeg)
        double(opts.FocusedGammaHalfWidthDeg)
        ];
    bounds = [center - span, center + span];
    % v32.1.3 invariant: the physical continuation prediction must always be
    % inside the Stage-A search box with a small margin.  A prior failed seed
    % can enlarge/shift the box, never remove this branch.
    continuationMargin = max(0.15 * span, [0.015;0.015;0.25;0.15]);
    bounds(:,1) = min(bounds(:,1), continuationCenter - continuationMargin);
    bounds(:,2) = max(bounds(:,2), continuationCenter + continuationMargin);
end

% Physics-informed joint-map seeds are authoritative evidence about the local
% elevator/throttle balance.  They may widen Stage A but can never shrink it.
if nargin >= 4 && ~isempty(jointSeeds) && height(jointSeeds) > 0
    try
        e = double(jointSeeds.Elevator); t = double(jointSeeds.Throttle);
        e = e(isfinite(e)); t = t(isfinite(t));
        if ~isempty(e)
            bounds(1,1) = min(bounds(1,1), min(e)-double(opts.JointMapSeedMarginElevator));
            bounds(1,2) = max(bounds(1,2), max(e)+double(opts.JointMapSeedMarginElevator));
        end
        if ~isempty(t)
            bounds(2,1) = min(bounds(2,1), min(t)-double(opts.JointMapSeedMarginThrottle));
            bounds(2,2) = max(bounds(2,2), max(t)+double(opts.JointMapSeedMarginThrottle));
        end
    catch
    end
end

for k = 1:4
    bounds(k,1) = max(bounds(k,1), globalBounds(k,1));
    bounds(k,2) = min(bounds(k,2), globalBounds(k,2));
    if bounds(k,2) <= bounds(k,1)
        bounds(k,:) = globalBounds(k,:);
    end
end
bounds = local_freeze_preparation_state_bounds(bounds, cfgId, opts);
end

function row = local_continuation_row(cfgId, trimBank, opts)
% Predict the next trim from the already verified configuration sequence.
if cfgId <= 0
    cur = trimBank(1);
    row = [double(cur.elevator_cmd), double(cur.throttle_cmd), double(cur.pitch_deg), ...
        local_trim_field(cur, "initial_flight_path_deg", 0.0)];
    row = local_clip_prediction(row, opts);
    return;
end

prev = trimBank(cfgId);
rowPrev = [double(prev.elevator_cmd), double(prev.throttle_cmd), double(prev.pitch_deg), ...
    local_trim_field(prev, "initial_flight_path_deg", 0.0)];
row = rowPrev;

if cfgId >= 2
    prev2 = trimBank(cfgId - 1);
    if isfield(prev, "score") && isfinite(double(prev.score)) && ...
            isfield(prev2, "score") && isfinite(double(prev2.score))
        rowPrev2 = [double(prev2.elevator_cmd), double(prev2.throttle_cmd), double(prev2.pitch_deg), ...
            local_trim_field(prev2, "initial_flight_path_deg", 0.0)];
        delta = rowPrev - rowPrev2;
        delta(1) = min(max(delta(1), -double(opts.MaxContinuationElevatorStep)), double(opts.MaxContinuationElevatorStep));
        delta(2) = min(max(delta(2), -double(opts.MaxContinuationThrottleStep)), double(opts.MaxContinuationThrottleStep));
        delta(3) = min(max(delta(3), -double(opts.MaxContinuationPitchStepDeg)), double(opts.MaxContinuationPitchStepDeg));
        delta(4) = min(max(delta(4), -double(opts.MaxContinuationGammaStepDeg)), double(opts.MaxContinuationGammaStepDeg));
        row = rowPrev + double(opts.ContinuationGain) * delta;
    end
end
row = local_clip_prediction(row, opts);
end

function row = local_clip_prediction(row, opts)
limits = [
    double(opts.ElevatorRange(:)).'
    double(opts.ThrottleRange(:)).'
    double(opts.PitchRange(:)).'
    double(opts.FlightPathRange(:)).'
    ];
for k = 1:4
    row(k) = min(max(row(k), limits(k,1)), limits(k,2));
end
alpha = row(3) - row(4);
if alpha < double(opts.InitialAlphaRange(1))
    row(3) = row(4) + double(opts.InitialAlphaRange(1));
elseif alpha > double(opts.InitialAlphaRange(2))
    row(3) = row(4) + double(opts.InitialAlphaRange(2));
end
row(3) = min(max(row(3), limits(3,1)), limits(3,2));
end

function bounds = local_refine_bounds(bestEq, probeOp, opts)
center = [
    double(bestEq.Elevator)
    double(bestEq.Throttle)
    double(probeOp.pitch_deg)
    local_trim_field(probeOp, "initial_flight_path_deg", 0.0)
    ];
half = [
    double(opts.RefineElevatorHalfWidth)
    double(opts.RefineThrottleHalfWidth)
    double(opts.RefinePitchHalfWidthDeg)
    double(opts.RefineGammaHalfWidthDeg)
    ];
globalBounds = [
    double(opts.ElevatorRange(:)).'
    double(opts.ThrottleRange(:)).'
    double(opts.PitchRange(:)).'
    double(opts.FlightPathRange(:)).'
    ];
bounds = [center - half, center + half];
for k = 1:4
    bounds(k,1) = max(bounds(k,1), globalBounds(k,1));
    bounds(k,2) = min(bounds(k,2), globalBounds(k,2));
    if bounds(k,2) <= bounds(k,1)
        bounds(k,:) = globalBounds(k,:);
    end
end
bounds = local_freeze_preparation_state_bounds(bounds, 1, opts);
end

function bounds = local_recovery_bounds(best, op, cfgId, trimBank, opts)
% Recovery is intentionally wider than Stage B and includes the sequential
% prediction.  It is still local enough to avoid returning to the huge global
% 4-D box that wasted many evaluations in v4.
bestRow = [double(best.Elevator), double(best.Throttle), double(best.Pitch0), double(best.Gamma0)];
opRow = [double(op.elevator_cmd), double(op.throttle_cmd), double(op.pitch_deg), ...
    local_trim_field(op, "initial_flight_path_deg", 0.0)];
predRow = local_continuation_row(cfgId, trimBank, opts);
center = median([bestRow; opRow; predRow], 1);
half = [double(opts.RecoveryElevatorHalfWidth), double(opts.RecoveryThrottleHalfWidth), ...
    double(opts.RecoveryPitchHalfWidthDeg), double(opts.RecoveryGammaHalfWidthDeg)];
globalBounds = [
    double(opts.ElevatorRange(:)).'
    double(opts.ThrottleRange(:)).'
    double(opts.PitchRange(:)).'
    double(opts.FlightPathRange(:)).'
    ];
bounds = [center(:) - half(:), center(:) + half(:)];
for k = 1:4
    bounds(k,1) = max(bounds(k,1), globalBounds(k,1));
    bounds(k,2) = min(bounds(k,2), globalBounds(k,2));
    if bounds(k,2) <= bounds(k,1)
        bounds(k,:) = globalBounds(k,:);
    end
end
bounds = local_freeze_preparation_state_bounds(bounds, cfgId, opts);
end

function vars = local_vars_from_bounds(bounds)
vars = [
    optimizableVariable("Elevator", bounds(1,:))
    optimizableVariable("Throttle", bounds(2,:))
    optimizableVariable("Pitch0", bounds(3,:))
    optimizableVariable("Gamma0", bounds(4,:))
    ];
end

function X = local_initial_x(cfgId, trimBank, opts, bounds, jointSeeds)
rows = zeros(0,4);
if nargin >= 5 && ~isempty(jointSeeds) && height(jointSeeds) > 0
    try
        rows = [rows; double(jointSeeds{:,{'Elevator','Throttle','Pitch0','Gamma0'}})]; %#ok<AGROW>
    catch
    end
end

% cfg0 seeds are taken from the region supported by the latest v4 data.
% They are only starting points; bayesopt is free to move anywhere in the
% focused bounds.  Keeping Gamma0 near level flight prevents wasting many
% evaluations on large release trajectories.
if cfgId == 0 && logical(opts.UseFocusedCfg0Search)
    rows = [
        rows
        0.150, 0.600, 6.0, 0.0
        0.175, 0.687, 6.4, 0.0
        0.162, 0.540, 6.0, 0.0
        0.125, 0.580, 6.0, 0.0
        0.190, 0.620, 5.5, 0.0
        ];
end

if cfgId > 0
    prev = trimBank(cfgId);
    if isfield(prev, "score") && isfinite(double(prev.score))
        g = local_trim_field(prev, "initial_flight_path_deg", 0.0);
        prevRow = [double(prev.elevator_cmd), double(prev.throttle_cmd), double(prev.pitch_deg), g];
        rows(end+1,:) = prevRow; %#ok<AGROW>
    else
        prevRow = [];
    end

    predRow = local_continuation_row(cfgId, trimBank, opts);
    rows(end+1,:) = predRow; %#ok<AGROW>
    if ~isempty(prevRow)
        rows(end+1,:) = 0.5 * (prevRow + predRow); %#ok<AGROW>
    end

    % Directional seeds around the continuation estimate make the local
    % response gradient visible to bayesopt immediately.  This is especially
    % useful for cfg1/cfg2 where the latest verification still had terminal
    % descent even though speed and q were already close.
    rows(end+1,:) = predRow + [ double(opts.SeedElevatorStep), 0, 0, 0]; %#ok<AGROW>
    rows(end+1,:) = predRow + [-double(opts.SeedElevatorStep), 0, 0, 0]; %#ok<AGROW>
    rows(end+1,:) = predRow + [0,  double(opts.SeedThrottleStep), 0, 0]; %#ok<AGROW>
    rows(end+1,:) = predRow + [0, -double(opts.SeedThrottleStep), 0, 0]; %#ok<AGROW>
    rows(end+1,:) = predRow + [0, 0,  double(opts.SeedPitchStepDeg), 0]; %#ok<AGROW>
    rows(end+1,:) = predRow + [0, 0, -double(opts.SeedPitchStepDeg), 0]; %#ok<AGROW>
end

cur = trimBank(cfgId + 1);
g = local_trim_field(cur, "initial_flight_path_deg", 0.0);
rows(end+1,:) = [double(cur.elevator_cmd), double(cur.throttle_cmd), double(cur.pitch_deg), g]; %#ok<AGROW>

rows = local_clip_rows(rows, bounds, opts);
X = array2table(rows, 'VariableNames', {'Elevator','Throttle','Pitch0','Gamma0'});
end

function X = local_refine_initial_x(bestEq, probeOp, bounds, opts)
opPoint = [
    double(probeOp.elevator_cmd), double(probeOp.throttle_cmd), ...
    double(probeOp.pitch_deg), local_trim_field(probeOp, "initial_flight_path_deg", 0.0)
    ];
bestPointRow = [double(bestEq.Elevator), double(bestEq.Throttle), double(bestEq.Pitch0), double(bestEq.Gamma0)];
blend = 0.5 * (opPoint + bestPointRow);
rows = [opPoint; bestPointRow; blend];
rows = local_clip_rows(rows, bounds, opts);
X = array2table(rows, 'VariableNames', {'Elevator','Throttle','Pitch0','Gamma0'});
end

function X = local_recovery_initial_x(best, op, cfgId, trimBank, bounds, opts)
bestRow = [double(best.Elevator), double(best.Throttle), double(best.Pitch0), double(best.Gamma0)];
opRow = [double(op.elevator_cmd), double(op.throttle_cmd), double(op.pitch_deg), ...
    local_trim_field(op, "initial_flight_path_deg", 0.0)];
predRow = local_continuation_row(cfgId, trimBank, opts);
blend = 0.5 * (bestRow + predRow);
rows = [
    bestRow
    opRow
    predRow
    blend
    predRow + [ double(opts.RecoverySeedElevatorStep), 0, 0, 0]
    predRow + [-double(opts.RecoverySeedElevatorStep), 0, 0, 0]
    predRow + [0,  double(opts.RecoverySeedThrottleStep), 0, 0]
    predRow + [0, -double(opts.RecoverySeedThrottleStep), 0, 0]
    predRow + [0, 0,  double(opts.RecoverySeedPitchStepDeg), 0]
    predRow + [0, 0, -double(opts.RecoverySeedPitchStepDeg), 0]
    ];
rows = local_clip_rows(rows, bounds, opts);
X = array2table(rows, 'VariableNames', {'Elevator','Throttle','Pitch0','Gamma0'});
end

function rows = local_clip_rows(rows, bounds, opts)
if isempty(rows)
    rows = mean(bounds, 2).';
end
for k = 1:4
    rows(:,k) = min(max(rows(:,k), bounds(k,1)), bounds(k,2));
end
for i = 1:size(rows,1)
    alpha = rows(i,3) - rows(i,4);
    if alpha < double(opts.InitialAlphaRange(1))
        rows(i,3) = rows(i,4) + double(opts.InitialAlphaRange(1));
    elseif alpha > double(opts.InitialAlphaRange(2))
        rows(i,3) = rows(i,4) + double(opts.InitialAlphaRange(2));
    end
    rows(i,3) = min(max(rows(i,3), bounds(3,1)), bounds(3,2));
end
rows = unique(rows, "rows", "stable");
end

function [bestIdx, feasibleFound] = local_best_verification(metricsList, opts)
n = numel(metricsList);
scores = inf(n,1);
gateRatios = inf(n,1);
feasible = false(n,1);
for i = 1:n
    scores(i) = local_trim_soft_score(metricsList{i}, opts);
    gateRatios(i) = local_trim_gate_ratio(metricsList{i}, opts);
    feasible(i) = ~local_trim_accept_fail(metricsList{i}, opts);
end
if any(feasible)
    feasibleFound = true;
    idx = find(feasible);
    [~, j] = min(scores(idx));
    bestIdx = idx(j);
else
    feasibleFound = false;
    % For failed verification passes, choose the point closest to satisfying
    % every hard gate first; use soft score only as a tie breaker.
    tableRank = [(1:n).', gateRatios, scores];
    tableRank = sortrows(tableRank, [2 3]);
    bestIdx = tableRank(1,1);
end
end

function settleS = local_detect_id_settle_s(T, opts)
% Estimate how long a tail-equilibrium configuration needs to settle before
% identification excitation is allowed to start.  The accepted Stage-D point
% may be a valid static equilibrium even when the payload-transition trajectory
% contains a long phugoid.  Starting n4sid excitation after a fixed 20 s would
% therefore re-contaminate the clean ID data.
settleS = double(opts.TailRescueIdSettleBaseS);
if isempty(T) || height(T) < 10
    return;
end

t = double(T.time_s(:));
valid = isfinite(t);
if nnz(valid) < 10
    return;
end
t0 = min(t(valid));
tEnd = max(t(valid));
windowS = max(double(opts.AdaptiveIdSettleWindowS), double(opts.TailRescueTailWindowS));
stepS = max(0.25, double(opts.AdaptiveIdSettleStepS));
firstStart = t0 + max(0.0, double(opts.AdaptiveIdSettleMinS));
lastStart = tEnd - windowS;
if lastStart < firstStart
    return;
end

starts = (firstStart:stepS:lastStart).';
if isempty(starts) || starts(end) < lastStart - 0.25 * stepS
    starts(end+1,1) = lastStart; %#ok<AGROW>
end
passes = false(size(starts));
ratios = inf(size(starts));
for i = 1:numel(starts)
    mask = valid & t >= starts(i) & t <= starts(i) + windowS;
    [passes(i), ratios(i)] = local_tail_window_pass(T, mask, opts);
end

% Require every subsequent window to stay inside the equilibrium gates.  This
% avoids choosing a single phugoid turning point that merely looks stationary.
stableIdx = [];
for i = 1:numel(starts)
    if all(passes(i:end))
        stableIdx = i;
        break;
    end
end

if isempty(stableIdx)
    % The final verification tail itself passed before this function is called,
    % but if sliding windows do not all pass, conservatively wait until the
    % start of the last analyzed window plus a margin.
    [~, bestIdx] = min(ratios);
    if isempty(bestIdx) || ~isfinite(ratios(bestIdx))
        stableStart = lastStart;
    else
        stableStart = max(starts(bestIdx), lastStart);
    end
else
    stableStart = starts(stableIdx);
end

relativeSettle = stableStart - t0 + double(opts.AdaptiveIdSettleMarginS);
settleS = max(settleS, relativeSettle);
settleS = min(settleS, double(opts.AdaptiveIdSettleMaxS));
end

function [pass, ratio] = local_tail_window_pass(T, mask, opts)
pass = false;
ratio = Inf;
if nnz(mask) < 5
    return;
end
h = double(T.altitude_m(mask));
va = double(T.airspeed_mps(mask));
pitch = double(T.pitch_deg(mask));
vz = double(T.vz_up_mps(mask));
q = double(T.q_dps(mask));
tt = double(T.time_s(mask));

vals = [
    abs(local_median_omitnan(vz)) / double(opts.TailRescueMaxAbsVzMps)
    local_rms_omitnan(vz) / double(opts.TailRescueMaxVzRmsMps)
    abs(local_median_omitnan(q)) / double(opts.TailRescueMaxAbsQDps)
    local_rms_omitnan(q) / double(opts.TailRescueMaxQRmsDps)
    local_rms_omitnan(va - double(opts.TargetAirspeedMps)) / double(opts.TailRescueMaxAirspeedRmsMps)
    local_std_omitnan(pitch) / double(opts.TailRescueMaxPitchStdDeg)
    abs(local_linear_slope(tt, h)) / double(opts.TailRescueMaxHeightSlopeMps)
    abs(local_linear_slope(tt, va)) / double(opts.TailRescueMaxAirspeedSlopeMps2)
    abs(local_linear_slope(tt, pitch)) / double(opts.TailRescueMaxPitchSlopeDegps)
    ];
if any(~isfinite(vals))
    return;
end
pitchMed = local_median_omitnan(pitch);
if ~isfinite(pitchMed) || pitchMed < double(opts.AcceptPitchRange(1)) || ...
        pitchMed > double(opts.AcceptPitchRange(2))
    return;
end
ratio = max(vals);
pass = ratio <= 1.0;
end

function [startValue, endValue, drift] = local_edge_drift(x, edgeFraction)
x = double(x(:));
n = numel(x);
if n == 0
    startValue = NaN; endValue = NaN; drift = NaN; return;
end
edgeN = max(1, min(n, round(double(edgeFraction) * n)));
startValue = local_median_omitnan(x(1:edgeN));
endValue = local_median_omitnan(x(max(1,n-edgeN+1):n));
drift = endValue - startValue;
end

function value = local_linear_slope(t, x)
t = double(t(:));
x = double(x(:));
mask = isfinite(t) & isfinite(x);
t = t(mask);
x = x(mask);
if numel(t) < 2 || max(t) <= min(t)
    value = NaN;
    return;
end
tc = t - mean(t);
den = sum(tc.^2);
if den <= eps
    value = NaN;
else
    value = sum(tc .* (x - mean(x))) / den;
end
end

function value = local_max_abs(x)
x = double(x(:));
x = x(isfinite(x));
if isempty(x), value = NaN; else, value = max(abs(x)); end
end

function value = local_median_omitnan(x)
x = double(x(:));
x = x(isfinite(x));
if isempty(x), value = NaN; else, value = median(x); end
end

function value = local_rms_omitnan(x)
x = double(x(:));
x = x(isfinite(x));
if isempty(x), value = NaN; else, value = sqrt(mean(x.^2)); end
end

function value = local_std_omitnan(x)
x = double(x(:));
x = x(isfinite(x));
if isempty(x)
    value = NaN;
else
    value = std(x);
end
end

function outputRoot = local_eval_output_root(opts, cfgId, runId)
root = string(opts.WorkRoot);
if strlength(root) == 0
    outputRoot = "";
else
    outputRoot = fullfile(root, sprintf("cfg%d", cfgId), runId);
end
end


function trimBank = local_ensure_trim_fields(trimBank, opts)
for k = 1:numel(trimBank)
    if ~isfield(trimBank, "initial_flight_path_deg") || isempty(trimBank(k).initial_flight_path_deg)
        trimBank(k).initial_flight_path_deg = 0.0;
    end
    if ~isfield(trimBank, "id_settle_s") || isempty(trimBank(k).id_settle_s)
        trimBank(k).id_settle_s = double(opts.DefaultIdSettleBaseS) + ...
            double(opts.DefaultIdSettlePerConfigS) * max(0, k - 1);
    end
    if ~isfield(trimBank, "acceptance_mode") || isempty(trimBank(k).acceptance_mode) || ...
            all(strlength(string(trimBank(k).acceptance_mode)) == 0)
        trimBank(k).acceptance_mode = "";
    end
    if ~isfield(trimBank, "resume_seed_valid") || isempty(trimBank(k).resume_seed_valid)
        trimBank(k).resume_seed_valid = false;
    end
end
end

function checkpoint = local_new_checkpoint(trimBank)
checkpoint = struct();
checkpoint.version = 3;
checkpoint.trim_bank = trimBank;
checkpoint.status = repmat("not_run", 5, 1);
checkpoint.best_point = cell(5, 1);
checkpoint.best_op = cell(5, 1);
checkpoint.best_metrics = cell(5, 1);
checkpoint.updated_at = string(datetime("now"));
end

function [trimBank, checkpoint] = local_load_resume_state(trimBank, checkpoint, opts)
paths = strings(0,1);
cp = local_checkpoint_path(opts);
if strlength(cp) > 0, paths(end+1,1) = cp; end %#ok<AGROW>
if strlength(string(opts.PreviousTrimMat)) > 0, paths(end+1,1) = string(opts.PreviousTrimMat); end %#ok<AGROW>
if strlength(string(opts.OutputMat)) > 0, paths(end+1,1) = string(opts.OutputMat); end %#ok<AGROW>
paths = unique(paths, "stable");
for i = 1:numel(paths)
    f = paths(i);
    if ~isfile(f), continue; end
    try
        S = load(f);
        if isfield(S, "checkpoint") && isstruct(S.checkpoint)
            checkpoint = local_merge_checkpoint(checkpoint, S.checkpoint);
        end
        sourceBank = [];
        if isfield(S, "trimBank")
            sourceBank = S.trimBank;
        elseif isfield(S, "result") && isstruct(S.result) && isfield(S.result, "trim_bank")
            sourceBank = S.result.trim_bank;
        elseif isfield(S, "trim_bank")
            sourceBank = S.trim_bank;
        end
        if ~isempty(sourceBank)
            [trimBank, checkpoint] = local_merge_trim_bank(trimBank, checkpoint, sourceBank);
        end
        fprintf("Loaded trim resume state: %s\n", f);
        break;
    catch ME
        warning("AirdropX:AutoMPC:ResumeLoadFailed", "Could not load %s: %s", f, ME.message);
    end
end
checkpoint.trim_bank = trimBank;
end

function checkpoint = local_merge_checkpoint(checkpoint, old)
if isfield(old, "status")
    n = min(numel(old.status), 5);
    checkpoint.status(1:n) = string(old.status(1:n));
end
for fieldName = ["best_point","best_op","best_metrics"]
    fn = char(fieldName);
    if isfield(old, fn) && iscell(old.(fn))
        n = min(numel(old.(fn)), 5);
        checkpoint.(fn)(1:n) = old.(fn)(1:n);
    end
end
end

function [trimBank, checkpoint] = local_merge_trim_bank(trimBank, checkpoint, sourceBank)
n = min(numel(sourceBank), numel(trimBank));
for k = 1:n
    src = sourceBank(k);
    if ~local_trim_entry_has_solution(src), continue; end
    trimBank(k) = local_copy_trim_fields(trimBank(k), src);
    if checkpoint.status(k) == "not_run"
        if iscell(checkpoint.best_point) && numel(checkpoint.best_point) >= k && ...
                ~isempty(checkpoint.best_point{k})
            checkpoint.status(k) = "verified";
        else
            % An observed trim entry alone is a good warm start, but v8 proved
            % it is not necessarily the exact requested point that produced the
            % winning verification trajectory.
            checkpoint.status(k) = "stale";
        end
    end
end
end

function tf = local_trim_entry_has_solution(s)
tf = isstruct(s) && isfield(s, "score") && ~isempty(s.score) && isfinite(double(s.score)) && ...
    isfield(s, "elevator_cmd") && isfinite(double(s.elevator_cmd)) && ...
    isfield(s, "throttle_cmd") && isfinite(double(s.throttle_cmd)) && ...
    isfield(s, "pitch_deg") && isfinite(double(s.pitch_deg));
end

function dst = local_copy_trim_fields(dst, src)
fields = ["config_id","altitude_m","airspeed_mps","pitch_deg","vz_up_mps","q_dps", ...
    "elevator_cmd","throttle_cmd","score","initial_flight_path_deg","id_settle_s","acceptance_mode","resume_seed_valid"];
for f = fields
    fn = char(f);
    if isfield(src, fn) && ~isempty(src.(fn))
        dst.(fn) = src.(fn);
    end
end
end

function tf = local_checkpoint_is_verified(checkpoint, cfgId)
k = cfgId + 1;
if k < 1 || k > numel(checkpoint.status)
    tf = false;
    return;
end
st = string(checkpoint.status(k));
tf = any(st == ["verified","verified_reused","tail_equilibrium_rescue","phugoid_cycle_confirm"]);
end

function [ok, run, metrics, op, point] = local_try_reuse_trim(cfgId, trim, checkpoint, opts)
ok = false; run = []; metrics = []; op = [];
point = local_checkpoint_best_point(checkpoint, cfgId, trim, opts);
try
    mode = local_trim_field_string(trim, "acceptance_mode", "full_trajectory");
    if mode == "tail_equilibrium_rescue"
        tag = "tail_rescue_reuse_verify";
    elseif mode == "phugoid_cycle_confirm"
        tag = "phugoid_confirm_reuse";
    else
        tag = "reuse_verify";
    end
    run = local_run_candidate(point, cfgId, trim, opts, tag);
    local_check_input_consistency(run.timeseries, point, opts, cfgId);
    metricOpts = opts;
    if mode == "tail_equilibrium_rescue"
        metricOpts = local_tail_metric_opts(opts);
    end
    metrics = local_trim_metrics(run.timeseries, metricOpts);
    if mode == "phugoid_cycle_confirm"
        [ok, ~] = local_phugoid_cycle_accept(run.timeseries, metrics, opts);
        op = local_observed_phugoid_center(run.timeseries, run.operating_point, opts);
    else
        op = local_observed_trim_point(run.timeseries, run.operating_point, metricOpts);
        if mode == "tail_equilibrium_rescue"
            ok = ~local_tail_accept_fail(metrics, metricOpts);
        else
            ok = ~local_trim_accept_fail(metrics, metricOpts);
        end
    end
catch ME
    warning("AirdropX:AutoMPC:ReuseVerifyFailed", "cfg%d saved trim verification failed: %s", cfgId, ME.message);
end
end

function point = local_checkpoint_best_point(checkpoint, cfgId, trim, opts)
k = cfgId + 1;
point = [];
if isstruct(checkpoint) && isfield(checkpoint, "best_point") && iscell(checkpoint.best_point) && ...
        numel(checkpoint.best_point) >= k && ~isempty(checkpoint.best_point{k})
    candidate = checkpoint.best_point{k};
    if istable(candidate) && height(candidate) >= 1 && ...
            all(ismember(["Elevator","Throttle","Pitch0","Gamma0"], string(candidate.Properties.VariableNames)))
        point = candidate(1,:);
    end
end
if isempty(point)
    point = local_point_from_op(trim, opts);
end
end

function [trimBank, used] = local_apply_failed_warm_start(trimBank, checkpoint, cfgId, opts)
used = false;
k = cfgId + 1;
if k > numel(checkpoint.status), return; end
st = string(checkpoint.status(k));
if ~any(st == ["failed","stale"]), return; end

% Failed warm starts are allowed only when the saved verification remained
% dynamically sane and reasonably close to the formal gate.  Catastrophic or
% very distant failures stay in checkpoint.best_* as diagnostics only.
metrics = [];
if isfield(checkpoint,"best_metrics") && iscell(checkpoint.best_metrics) && ...
        numel(checkpoint.best_metrics) >= k
    metrics = checkpoint.best_metrics{k};
end
if ~local_failed_seed_eligible(metrics, opts)
    trimBank(k).resume_seed_valid = false;
    return;
end

point = [];
if iscell(checkpoint.best_point) && numel(checkpoint.best_point) >= k && ...
        ~isempty(checkpoint.best_point{k})
    X = checkpoint.best_point{k};
    if istable(X) && height(X) >= 1
        point = X(1,:);
    end
end

op = [];
if iscell(checkpoint.best_op) && numel(checkpoint.best_op) >= k
    op = checkpoint.best_op{k};
end

if ~isempty(point) && all(ismember(["Elevator","Throttle","Pitch0","Gamma0"], string(point.Properties.VariableNames)))
    % The exact requested point gets priority.  best_op remains useful for
    % measured airspeed/vz/q, but must not overwrite the initial state/control
    % values that actually produced the best trajectory.
    trimBank(k).elevator_cmd = double(point.Elevator(1));
    trimBank(k).throttle_cmd = double(point.Throttle(1));
    trimBank(k).pitch_deg = double(point.Pitch0(1));
    trimBank(k).initial_flight_path_deg = double(point.Gamma0(1));
    if ~isempty(op) && isstruct(op)
        if isfield(op,"airspeed_mps"), trimBank(k).airspeed_mps = double(op.airspeed_mps); end
        if isfield(op,"vz_up_mps"), trimBank(k).vz_up_mps = double(op.vz_up_mps); end
        if isfield(op,"q_dps"), trimBank(k).q_dps = double(op.q_dps); end
    end
    trimBank(k).resume_seed_valid = true;
    used = true;
    return;
end

if isempty(op), return; end
trimBank(k).elevator_cmd = double(op.elevator_cmd);
trimBank(k).throttle_cmd = double(op.throttle_cmd);
trimBank(k).pitch_deg = double(op.pitch_deg);
trimBank(k).initial_flight_path_deg = local_trim_field(op, "initial_flight_path_deg", 0.0);
trimBank(k).resume_seed_valid = true;
used = true;
end

function entry = local_store_trim_entry(entry, cfgId, op, metrics, opts, acceptanceMode)
previousIdSettleS = local_trim_field(entry, "id_settle_s", 0.0);
entry.config_id = cfgId;
entry.altitude_m = double(opts.TargetAltitudeM);
entry.airspeed_mps = double(op.airspeed_mps);
entry.pitch_deg = double(op.pitch_deg);
entry.vz_up_mps = double(op.vz_up_mps);
entry.q_dps = double(op.q_dps);
entry.elevator_cmd = double(op.elevator_cmd);
entry.throttle_cmd = double(op.throttle_cmd);
entry.initial_flight_path_deg = local_trim_field(op, "initial_flight_path_deg", 0.0);
entry.acceptance_mode = string(acceptanceMode);
entry.resume_seed_valid = false;
if entry.acceptance_mode == "tail_equilibrium_rescue"
    entry.id_settle_s = max(previousIdSettleS, double(opts.TailRescueIdSettleBaseS) + ...
        double(opts.TailRescueIdSettlePerConfigS) * max(0, cfgId - double(opts.TailRescueMinConfig)));
    entry.score = local_tail_equilibrium_score(metrics, opts);
else
    entry.id_settle_s = max(previousIdSettleS, double(opts.DefaultIdSettleBaseS) + ...
        double(opts.DefaultIdSettlePerConfigS) * max(0, cfgId));
    entry.score = local_trim_soft_score(metrics, opts);
end
end

function value = local_trim_field_string(s, name, fallback)
value = string(fallback);
if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && strlength(string(s.(name))) > 0
    value = string(s.(name));
end
end

function checkpoint = local_checkpoint_success(checkpoint, cfgId, trim, metrics, point)
k = cfgId + 1;
checkpoint.status(k) = string(local_trim_field_string(trim, "acceptance_mode", "verified"));
if any(checkpoint.status(k) == ["full_trajectory","phugoid_cycle_confirm"]), checkpoint.status(k) = "verified"; end
checkpoint.trim_bank(k) = trim;
checkpoint.best_point{k} = point;
checkpoint.best_op{k} = struct("elevator_cmd",trim.elevator_cmd,"throttle_cmd",trim.throttle_cmd, ...
    "pitch_deg",trim.pitch_deg,"initial_flight_path_deg",trim.initial_flight_path_deg, ...
    "airspeed_mps",trim.airspeed_mps,"vz_up_mps",trim.vz_up_mps,"q_dps",trim.q_dps);
checkpoint.best_metrics{k} = metrics;
checkpoint.updated_at = string(datetime("now"));
end

function tf = local_failed_seed_eligible(metrics, opts)
% Only finite, dynamically sane, bounded near-misses may steer a later search.
tf = false;
if isempty(metrics) || ~isstruct(metrics), return; end
try
    if local_trim_catastrophic_fail(metrics, opts), return; end
    ratio = local_trim_gate_ratio(metrics, opts);
catch
    % Old/incomplete failure metrics are not trusted as optimization seeds.
    return;
end
tf = isfinite(ratio) && ratio <= double(opts.MaxFailedWarmStartGateRatio);
end

function [trimBank, checkpoint] = local_isolate_catastrophic_resume_state(trimBank, checkpoint, opts)
% Sanitize checkpoints written by older versions.  Keep their failure
% evidence, but remove the authority to steer the next optimization.
for cfgId = 0:min(4, numel(trimBank)-1)
    k = cfgId + 1;
    st = string(checkpoint.status(k));
    if ~any(st == ["failed","stale","failed_catastrophic"]), continue; end
    metrics = [];
    if isfield(checkpoint,"best_metrics") && iscell(checkpoint.best_metrics) && ...
            numel(checkpoint.best_metrics) >= k
        metrics = checkpoint.best_metrics{k};
    end
    if local_failed_seed_eligible(metrics, opts)
        continue;
    end
    checkpoint.status(k) = "failed_catastrophic";
    trimBank = local_reset_failed_trim_to_continuation(trimBank, cfgId, opts);
    fprintf(['  cfg%d previous failed seed isolated: catastrophic/far outside gate. ' ...
        'Diagnostic best_point/best_metrics retained; continuation branch restored.\n'], cfgId);
end
checkpoint.trim_bank = trimBank;
end

function trimBank = local_reset_failed_trim_to_continuation(trimBank, cfgId, opts)
% Replace only the working seed.  Historical checkpoint evidence remains.
k = cfgId + 1;
if cfgId <= 0
    fresh = airdropx_auto_default_trim_bank("TargetAltitudeM", opts.TargetAltitudeM, ...
        "TargetAirspeedMps", opts.TargetAirspeedMps);
    fresh = local_ensure_trim_fields(fresh, opts);
    continuation = [double(fresh(1).elevator_cmd), double(fresh(1).throttle_cmd), ...
        double(fresh(1).pitch_deg), local_trim_field(fresh(1), "initial_flight_path_deg", 0.0)];
else
    continuation = local_continuation_row(cfgId, trimBank, opts);
end
trimBank(k).config_id = cfgId;
trimBank(k).altitude_m = double(opts.TargetAltitudeM);
trimBank(k).airspeed_mps = double(opts.TargetAirspeedMps);
trimBank(k).pitch_deg = continuation(3);
trimBank(k).vz_up_mps = 0.0;
trimBank(k).q_dps = 0.0;
trimBank(k).elevator_cmd = continuation(1);
trimBank(k).throttle_cmd = continuation(2);
trimBank(k).initial_flight_path_deg = continuation(4);
trimBank(k).score = NaN;
trimBank(k).acceptance_mode = "";
trimBank(k).resume_seed_valid = false;
end

function checkpoint = local_checkpoint_failure(checkpoint, cfgId, op, metrics, point, opts)
k = cfgId + 1;
if local_failed_seed_eligible(metrics, opts)
    checkpoint.status(k) = "failed";
else
    checkpoint.status(k) = "failed_catastrophic";
end
% Preserve the exact failure evidence for diagnostics regardless of whether it
% is eligible for reuse.  Eligibility is controlled exclusively by status +
% metrics; diagnostic data are never silently destroyed.
checkpoint.best_point{k} = point;
checkpoint.best_op{k} = op;
checkpoint.best_metrics{k} = metrics;
checkpoint.updated_at = string(datetime("now"));
end

function checkpointPath = local_checkpoint_path(opts)
checkpointPath = string(opts.CheckpointMat);
if strlength(checkpointPath) > 0, return; end
if strlength(string(opts.OutputMat)) > 0
    outDir = string(fileparts(opts.OutputMat));
    if strlength(outDir) == 0, outDir = "."; end
    checkpointPath = fullfile(outDir, "auto_trim_checkpoint.mat");
elseif strlength(string(opts.WorkRoot)) > 0
    checkpointPath = fullfile(string(opts.WorkRoot), "auto_trim_checkpoint.mat");
else
    checkpointPath = "";
end
end

function local_save_progress(trimBank, records, verifications, checkpoint, opts)
checkpoint.trim_bank = trimBank;
checkpoint.updated_at = string(datetime("now"));
result = struct("trim_bank", trimBank, "records", {records}, "verification_runs", {verifications}, ...
    "checkpoint", checkpoint);
if strlength(string(opts.OutputMat)) > 0
    outDir = string(fileparts(opts.OutputMat));
    if strlength(outDir) > 0 && ~isfolder(outDir), mkdir(outDir); end
    save(opts.OutputMat, "result", "trimBank", "checkpoint", "-v7.3");
end
cp = local_checkpoint_path(opts);
if strlength(cp) > 0
    cpDir = string(fileparts(cp));
    if strlength(cpDir) > 0 && ~isfolder(cpDir), mkdir(cpDir); end
    save(cp, "checkpoint", "trimBank", "-v7.3");
end
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.Model = "airdropx_mpc_id";
opts.OutputMat = "";
opts.WorkRoot = "";
% Resume/reuse support. If PreviousTrimMat is empty, OutputMat and the
% adjacent checkpoint are checked automatically. A verified cfg gets one
% quick validation run and then skips bayesopt when it still passes.
opts.ReuseVerifiedTrim = true;
opts.ReuseFailedAsWarmStart = true;
% v32.1.3: only dynamically sane failed points with a finite gate ratio no
% worse than this may influence a later search.  Catastrophic failures remain
% diagnostic-only and are reset to configuration-continuation seeds.
opts.MaxFailedWarmStartGateRatio = 3.0;
% v32.1.4: deterministic 2-D control map before Bayesian search.  The map
% estimates pitch acceleration and airspeed acceleration directly from the
% existing q/V signals, so elevator and throttle are learned jointly.
opts.UseJointControlMap = true;
opts.JointMapMinConfig = 0;
opts.JointMapElevatorSamples = 5;
opts.JointMapThrottleSamples = 5;
opts.JointMapTopK = 10;
opts.JointMapElevatorSpan = 0.26;
opts.JointMapThrottleRange = [0.35 0.88];
opts.JointMapPrepDropIntervalS = 2.0;
opts.JointMapPostConfigObserveS = 12.0;
opts.JointMapMinStopTimeS = 12.0;
opts.JointMapEarlyWindowS = [0.08 0.30];
opts.JointMapMidWindowS = [0.30 0.80];
opts.JointMapSmoothWindowS = 0.05;
opts.JointMapQdotScaleDps2 = 5.0;
opts.JointMapVdotScaleMps2 = 2.0;
opts.JointMapQScaleDps = 3.0;
opts.JointMapVzScaleMps = 0.75;
opts.JointMapVaErrorScaleMps = 3.0;
opts.JointMapSeedMarginElevator = 0.04;
opts.JointMapSeedMarginThrottle = 0.04;
opts.JointMapTailWindowS = 4.0;
opts.JointMapTailVzScaleMps = 0.30;
opts.JointMapTailQScaleDps = 0.25;
opts.JointMapTailHeightSlopeScaleMps = 0.20;
opts.JointMapTailVaSlopeScaleMps2 = 0.12;
% Stage-A itself also penalizes residual angular/airspeed acceleration, so
% the joint map is not merely a good initializer; the optimizer continues to
% solve the same physical equilibrium condition.
opts.EqEarlyQAccelWeight = 8.0;
opts.EqEarlyQAccelScaleDps2 = 2.5;
opts.EqEarlyVaAccelWeight = 6.0;
opts.EqEarlyVaAccelScaleMps2 = 1.0;
opts.PreviousTrimMat = "";
opts.CheckpointMat = "";
opts.ConfigIds = (0:4).';
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.ReferenceMassKg = 3423.0;
opts.CargoMassKg = 300.0;
% Searching at 20 m gives bad candidates almost no room before ground
% contact. Find the aerodynamic equilibrium safely high, then reuse the
% equilibrium inputs/pitch at the mission altitude.
opts.SearchAltitudeM = 200.0;
opts.StopTimeS = 22.0;
opts.RecordStartS = 10.0;
% Use a slower real payload-preparation sequence during trim search.  The old
% 0.5 s interval stacked four drop transients before later-config equilibrium
% could be assessed.  ID can still wait longer after reaching the config.
opts.PrepDropStartS = 1.0;
opts.PrepDropIntervalS = 2.0;
% Later configs are created by preparatory drops in the ID model. Give the
% resulting phugoid time to decay before deciding whether the fixed inputs are
% a true trim; extend simulation duration gradually for cfg1..cfg4.
opts.ConfigSettleBaseS = 4.0;
opts.ConfigSettlePerAdditionalDropS = 2.0;
opts.MinSteadyAfterSettleS = 2.0;
opts.ExtraStopTimePerConfigS = 5.0;
opts.MaxTrimStopTimeS = 42.0;
opts.MaxObjectiveEvaluations = 80;
opts.UseParallel = false;
opts.Verbose = 1;
% v20: cfg1+ trim search can prepare the real payload sequence using the
% earlier configuration trims. The target cfg command is applied only after
% its preparatory drop instead of incorrectly from t=0.
opts.PreparationTrimBank = [];
opts.UsePreparationTrimSchedule = true;
opts.EquilibriumSearchFraction = 0.55;
opts.UseFocusedCfg0Search = true;
% v4 cfg0 data concentrated the useful region near e=0.12..0.18 and
% throttle=0.54..0.69, while the old global box was far too wide for only
% 60 samples. Stage A starts focused, Stage B refines locally.
opts.Cfg0FocusedElevatorRange = [0.08 0.24];
opts.Cfg0FocusedThrottleRange = [0.48 0.72];
opts.Cfg0FocusedPitchRange = [3.0 9.0];
opts.Cfg0FocusedFlightPathRange = [-0.75 0.75];
opts.FocusedElevatorHalfWidth = 0.10;
opts.FocusedThrottleHalfWidth = 0.12;
opts.FocusedPitchHalfWidthDeg = 4.0;
opts.FocusedGammaHalfWidthDeg = 0.75;
opts.RefineElevatorHalfWidth = 0.07;
opts.RefineThrottleHalfWidth = 0.09;
opts.RefinePitchHalfWidthDeg = 2.0;
opts.RefineGammaHalfWidthDeg = 0.50;
opts.ElevatorRange = [-0.75 0.45];
opts.ThrottleRange = [0.35 0.88];
opts.PitchRange = [-10 25];
% Keep the initial release physically plausible.  The latest cfg0 winner
% requested Pitch0=13.75 deg and Gamma0=-3.53 deg (about 17.3 deg initial
% angle of attack) even though it later settled near pitch=5.6 deg.
opts.FlightPathRange = [-4 4];
opts.InitialAlphaRange = [-2 12];
opts.AcceptPitchRange = [-15 30];
opts.ElevatorInputTolerance = 0.01;
opts.ThrottleInputTolerance = 0.02;
opts.HardFloorAltitudeM = 5.0;
opts.CatastrophicQRmsDps = 6.0;
opts.CatastrophicPitchStdDeg = 6.0;
opts.SelfConsistentVerifyPasses = 1;
opts.ObservedOpTailWindowS = 5.0;

% v21 later-config phase-robust confirmation.  cfg3 v20 data showed a
% decaying ~24 s phugoid centered near the target; a 5-10 s endpoint window
% can reject the same equilibrium solely because the run ends on a descending
% phase. Revalidate one saved failed point over a longer horizon before new BO.
opts.UsePhugoidCycleConfirm = true;
opts.PhugoidConfirmMinConfig = 3;
opts.PhugoidConfirmStopTimeS = 60.0;
opts.PhugoidConfirmStopTimePerConfigS = 8.0;
opts.PhugoidConfirmWindowS = 24.0;
opts.PhugoidConfirmMaxAltitudeRmsM = 1.0;
opts.PhugoidConfirmMaxAltitudeMaxAbsM = 2.0;
opts.PhugoidConfirmMaxHeightSlopeMps = 0.08;
opts.PhugoidConfirmMaxMeanVzMps = 0.10;
opts.PhugoidConfirmMaxVzRmsMps = 0.40;
opts.PhugoidConfirmMaxMeanQDps = 0.15;
opts.PhugoidConfirmMaxQRmsDps = 0.35;
opts.PhugoidConfirmMaxAirspeedRmsMps = 0.80;
opts.PhugoidConfirmMaxPitchStdDeg = 0.50;
opts.PhugoidConfirmMaxVzDampingRatio = 1.05;
opts.PhugoidConfirmMaxQDampingRatio = 1.05;
opts.PhugoidConfirmIdSettleS = 30.0;
opts.PhugoidConfirmIdSettlePerConfigS = 5.0;
% v22: after the single checkpoint point fails, scan already-simulated trim
% trajectories for other candidates that satisfy the exact full-cycle gate.
% Re-run only the best few once at the long horizon before any new bayesopt.
opts.UsePhugoidHistoryRescue = true;
opts.PhugoidHistoryRescueTopK = 3;
opts.PhugoidHistoryRescueIdSettleS = 48.0;

% Sequential continuation and adaptive recovery.  cfg0/cfg1/cfg2 from the
% latest v5 run show a smooth trim-input trend; use it as a prediction rather
% than restarting each payload configuration from the previous point only.
opts.ContinuationGain = 1.0;
opts.MaxContinuationElevatorStep = 0.10;
opts.MaxContinuationThrottleStep = 0.08;
opts.MaxContinuationPitchStepDeg = 2.0;
opts.MaxContinuationGammaStepDeg = 0.60;
opts.SeedElevatorStep = 0.035;
opts.SeedThrottleStep = 0.025;
opts.SeedPitchStepDeg = 0.75;

opts.UseRecoverySearch = true;
opts.PolishTriggerRatio = 0.82;
opts.RecoveryObjectiveEvaluations = 36;
opts.RecoveryConfigGrowth = 0.25;
opts.RecoverySelfConsistentVerify = true;
opts.RecoveryElevatorHalfWidth = 0.12;
opts.RecoveryThrottleHalfWidth = 0.10;
opts.RecoveryPitchHalfWidthDeg = 3.0;
opts.RecoveryGammaHalfWidthDeg = 1.0;
opts.RecoverySeedElevatorStep = 0.040;
opts.RecoverySeedThrottleStep = 0.030;
opts.RecoverySeedPitchStepDeg = 1.0;
opts.RecoveryEquilibriumBlend = 0.25;
opts.RecoveryGatePenaltyWeight = 4000.0;

% v7 tail-equilibrium rescue for cfg2+.  The latest v6 cfg2 recovery still
% ended with tail vz=-0.661 m/s and hSlope=-0.636 m/s, while cfg0/cfg1 were
% already good.  Reduce the last-mile problem to the two true steady inputs
% and let the long-horizon terminal state determine pitch/gamma.
opts.UseTailEquilibriumRescue = true;
% v32.1.5: ID-readiness recovery can require the long-horizon Stage-D result
% itself to PASS even when the ordinary short-horizon trim gate was feasible.
opts.ForceLongHorizonTailPolish = false;
opts.TailRescueMinConfig = 2;
opts.TailRescueTriggerRatio = 1.05;
opts.TailRescueObjectiveEvaluations = 56;
opts.TailRescueEvalGrowthPerConfig = 10;
opts.TailRescueStopTimeBaseS = 70.0;
opts.TailRescueStopTimePerConfigS = 8.0;
opts.TailRescueElevatorHalfWidth = 0.12;
opts.TailRescueThrottleHalfWidth = 0.10;
opts.TailRescuePitchHalfWidthDeg = 1.5;
opts.TailRescueGammaHalfWidthDeg = 0.75;
opts.TailRescueSeedElevatorStep = 0.025;
opts.TailRescueSeedThrottleStep = 0.020;
opts.TailRescueSeedPitchStepDeg = 0.50;
opts.TailRescueSeedGammaStepDeg = 0.25;
opts.TailRescueGatePenaltyWeight = 6000.0;
opts.TailRescueTailWindowS = 24.0;
opts.TailRescueMaxAbsVzMps = 0.20;
opts.TailRescueMaxVzRmsMps = 0.45;
opts.TailRescueMaxAbsQDps = 0.20;
opts.TailRescueMaxQRmsDps = 0.40;
opts.TailRescueMaxAirspeedRmsMps = 0.80;
opts.TailRescueMaxPitchStdDeg = 0.50;
opts.TailRescueMaxHeightSlopeMps = 0.20;
opts.TailRescueMaxAirspeedSlopeMps2 = 0.10;
opts.TailRescueMaxPitchSlopeDegps = 0.10;
opts.TailRescueVzMedWeight = 240.0;
opts.TailRescueVzRmsWeight = 80.0;
opts.TailRescueQMedWeight = 180.0;
opts.TailRescueQRmsWeight = 60.0;
opts.TailRescueAirspeedWeight = 100.0;
opts.TailRescuePitchStdWeight = 50.0;
opts.TailRescueHeightSlopeWeight = 260.0;
opts.TailRescueAirspeedSlopeWeight = 70.0;
opts.TailRescuePitchSlopeWeight = 80.0;
opts.TailRescueVzSlopeWeight = 60.0;
opts.TailRescueVzMedScaleMps = 0.15;
opts.TailRescueVzRmsScaleMps = 0.35;
opts.TailRescueQMedScaleDps = 0.15;
opts.TailRescueQRmsScaleDps = 0.30;
opts.TailRescueAirspeedScaleMps = 0.60;
opts.TailRescuePitchStdScaleDeg = 0.35;
opts.TailRescueHeightSlopeScaleMps = 0.15;
opts.TailRescueAirspeedSlopeScaleMps2 = 0.08;
opts.TailRescuePitchSlopeScaleDegps = 0.08;
opts.TailRescueVzSlopeScaleMps2 = 0.08;
% ID collection waits longer after cfg2+ prep drops when Stage D was needed.
opts.DefaultIdSettleBaseS = 4.0;
opts.DefaultIdSettlePerConfigS = 2.0;
opts.TailRescueIdSettleBaseS = 20.0;
opts.TailRescueIdSettlePerConfigS = 5.0;
opts.AdaptiveIdSettleEnabled = true;
opts.AdaptiveIdSettleWindowS = 10.0;
opts.AdaptiveIdSettleStepS = 1.0;
opts.AdaptiveIdSettleMinS = 10.0;
opts.AdaptiveIdSettleMarginS = 5.0;
opts.AdaptiveIdSettleMaxS = 75.0;

opts.HeightReferenceWindowS = 0.5;
opts.TrimEdgeFraction = 0.20;

% Final trim acceptance.  Height holding is deliberately the first gate.
opts.MaxTrimAltitudeErrorM = 8.0; % legacy option kept for old calls
opts.MaxTrimAltitudeRmsM = 1.0;
opts.MaxTrimAltitudeMaxAbsM = 2.0;
opts.MaxTrimAltitudeDriftM = 1.0;
opts.MaxTrimAirspeedErrorMps = 8.0; % legacy option kept for old calls
opts.MaxTrimAirspeedRmsMps = 1.5;
opts.MaxTrimAirspeedMaxAbsMps = 3.0;
opts.MaxTrimAbsVzMps = 0.35;
opts.MaxTrimVzRmsMps = 0.60;
opts.MaxTrimQRmsDps = 0.75;
opts.MaxTrimPitchStdDeg = 0.75;
opts.MaxTrimPitchDriftDegps = 0.12;
opts.MaxFullAltitudeRmsM = 2.5;
opts.MaxFullAltitudeMaxAbsM = 5.0;
opts.MaxEarlyAltitudeRmsM = 3.0;
opts.MaxEarlyAltitudeMaxAbsM = 5.0;
opts.MaxFullAirspeedRmsMps = 2.5;
opts.MaxFullVzRmsMps = 1.20;
opts.MaxFullQRmsDps = 1.50;
opts.TailWindowS = 5.0;
opts.MaxTailAbsVzMps = 0.25;
opts.MaxTailAbsQDps = 0.25;
opts.MaxTailAirspeedRmsMps = 1.0;
opts.MaxTailHeightSlopeMps = 0.30;
opts.MaxTailAirspeedSlopeMps2 = 0.15;
opts.MaxTailPitchSlopeDegps = 0.12;
opts.MaxEarlyAltitudeLossM = 5.0; % legacy option retained

opts.HardFailScore = 1e8;
opts.TrimAltitudeWeight = 20.0; % legacy option kept for old calls
opts.FullAltitudeRmsWeight = 80.0;
opts.FullAltitudeMaxWeight = 70.0;
opts.EarlyAltitudeRmsWeight = 25.0;
opts.EarlyAltitudeMaxWeight = 35.0;
opts.TrimAltitudeRmsWeight = 120.0;
opts.TrimAltitudeMaxWeight = 60.0;
opts.TrimAltitudeDriftWeight = 60.0;
opts.FullAirspeedWeight = 20.0;
opts.EarlyAirspeedWeight = 10.0;
opts.TrimAirspeedWeight = 35.0;
opts.FullVzWeight = 15.0;
opts.TrimVzWeight = 25.0;
opts.FullQWeight = 8.0;
opts.TrimQWeight = 12.0;
opts.TailVzMedWeight = 80.0;
opts.TailQMedWeight = 60.0;
opts.TailAirspeedWeight = 35.0;
opts.TailHeightSlopeWeight = 90.0;
opts.TailAirspeedSlopeWeight = 30.0;
opts.TailPitchSlopeWeight = 35.0;
opts.TrimPitchStdWeight = 6.0;
opts.TrimPitchDriftWeight = 6.0;
opts.EarlyAltitudeLossWeight = 0.0; % legacy, no longer used
opts.EarlyVzWeight = 8.0;
opts.EarlyQWeight = 5.0;

% Stage-A equilibrium score puts derivative/tail conditions ahead of
% integrated altitude error. Stage B then optimizes the full trajectory.
opts.EquilibriumTrajectoryBlend = 0.20;
opts.EqTailVzMedWeight = 180.0;
opts.EqTailVzRmsWeight = 80.0;
opts.EqTailQMedWeight = 160.0;
opts.EqTailQRmsWeight = 70.0;
opts.EqTailAirspeedWeight = 100.0;
opts.EqTailHeightSlopeWeight = 180.0;
opts.EqTailAirspeedSlopeWeight = 70.0;
opts.EqTailPitchSlopeWeight = 100.0;
opts.EqTailVzSlopeWeight = 70.0;
opts.EqTailVzMedScaleMps = 0.15;
opts.EqTailVzRmsScaleMps = 0.35;
opts.EqTailQMedScaleDps = 0.15;
opts.EqTailQRmsScaleDps = 0.30;
opts.EqTailAirspeedScaleMps = 0.75;
opts.EqTailHeightSlopeScaleMps = 0.15;
opts.EqTailAirspeedSlopeScaleMps2 = 0.08;
opts.EqTailPitchSlopeScaleDegps = 0.08;
opts.EqTailVzSlopeScaleMps2 = 0.10;

% Normalization scales make score terms comparable and keep the priority
% obvious: altitude > airspeed > vertical/pitch dynamics.
opts.FullAltitudeRmsScaleM = 2.0;
opts.FullAltitudeMaxScaleM = 4.0;
opts.EarlyAltitudeRmsScaleM = 2.5;
opts.EarlyAltitudeMaxScaleM = 4.0;
opts.TrimAltitudeRmsScaleM = 0.75;
opts.TrimAltitudeMaxScaleM = 1.5;
opts.TrimAltitudeDriftScaleM = 0.75;
opts.FullAirspeedScaleMps = 2.0;
opts.EarlyAirspeedScaleMps = 3.0;
opts.TrimAirspeedScaleMps = 1.0;
opts.FullVzScaleMps = 1.0;
opts.TrimVzScaleMps = 0.40;
opts.FullQScaleDps = 1.0;
opts.TrimQScaleDps = 0.50;
opts.TailVzMedScaleMps = 0.20;
opts.TailQMedScaleDps = 0.20;
opts.TailAirspeedScaleMps = 0.75;
opts.TailHeightSlopeScaleMps = 0.20;
opts.TailAirspeedSlopeScaleMps2 = 0.10;
opts.TailPitchSlopeScaleDegps = 0.08;
opts.TrimPitchStdScaleDeg = 0.50;
opts.TrimPitchDriftScaleDegps = 0.08;
opts.EarlyAltitudeLossScaleM = 3.0;
opts.EarlyVzScaleMps = 1.0;
opts.EarlyQScaleDps = 1.0;
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
end



