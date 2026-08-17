function result = airdropx_auto_run_any_mission(varargin)
%AIRDROPX_AUTO_RUN_ANY_MISSION v30 generic H/V mission learner + validator.
%
% Runs one arbitrary mission context through the existing v29 unified learner.
% A nearest Plant/trim master from the global PlantContextBank is used only as
% the initial seed. v29 still performs its equilibrium probe and automatically
% rebuilds trim/ID/Plant for any cfg whose seed is not valid at the new speed.
%
% Example:
%   r = airdropx_auto_run_any_mission( ...
%       "TargetAltitudeM",73, "TargetAirspeedMps",47.5);
%
% The same OutputRoot is reused across attempts, so v29 checkpoints,
% same-context evaluations and the shared LearningBank all resume naturally.

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir);
addpath(paths.mpcDir);
addpath(paths.autoDir);
if isfolder(paths.sfuncDir), addpath(paths.sfuncDir); end

H = double(opts.TargetAltitudeM);
V = double(opts.TargetAirspeedMps);
if ~isfinite(H) || H < double(opts.MinimumAllowedAltitudeM) || H > double(opts.MaximumAllowedAltitudeM)
    error("AirdropX:V30:AltitudeOutOfRange", ...
        "TargetAltitudeM %.3f is outside configured v30 range [%.3f, %.3f] m.", ...
        H,opts.MinimumAllowedAltitudeM,opts.MaximumAllowedAltitudeM);
end
if ~isfinite(V) || V <= 0
    error("AirdropX:V30:BadAirspeed", "TargetAirspeedMps must be positive and finite.");
end

if strlength(string(opts.OutputRoot)) == 0
    missionRoot = fullfile(local_resolve_path(paths.projectRoot,opts.AnyMissionRoot),local_context_tag(H,V));
else
    missionRoot = local_resolve_path(paths.projectRoot,opts.OutputRoot);
end
if ~isfolder(missionRoot), mkdir(missionRoot); end

missionRecoveryOnly = logical(opts.MissionRecoveryOnly);
missionRecoverySourceRoot = "";
if missionRecoveryOnly
    missionRecoverySourceRoot = local_resolve_path(paths.projectRoot,opts.MissionRecoverySourceRoot);
    if strlength(missionRecoverySourceRoot)==0 || ~isfolder(missionRecoverySourceRoot)
        error("AirdropX:V30_6_1:MissingMissionRecoverySource", ...
            "MissionRecoveryOnly requires an existing MissionRecoverySourceRoot.");
    end
    local_prepare_mission_recovery_state(paths.projectRoot,missionRecoverySourceRoot,missionRoot);
end

baseIdentified = local_resolve_path(paths.projectRoot,opts.BaseIdentifiedMat);
if ~isfile(baseIdentified)
    error("AirdropX:V30:MissingBaseIdentified", "BaseIdentifiedMat not found: %s", baseIdentified);
end
learningBankRoot = local_resolve_path(paths.projectRoot,opts.LearningBankRoot);
plantBankRoot = local_resolve_path(paths.projectRoot,opts.PlantBankRoot);
if ~isfolder(learningBankRoot), mkdir(learningBankRoot); end
if ~isfolder(plantBankRoot), mkdir(plantBankRoot); end

if missionRecoveryOnly
    identifiedSeed = string(fullfile(missionRoot,"identified_plants_200m_master.mat"));
    seedSource = "MissionRecoveryInheritedVerifiedState";
else
    seed = airdropx_auto_plant_context_bank( ...
        "Action","nearest", "ProjectRoot",paths.projectRoot, "Root",plantBankRoot, ...
        "TargetAltitudeM",H,"TargetAirspeedMps",V, ...
        "ReferenceMassKg",opts.ReferenceMassKg,"CargoMassKg",opts.CargoMassKg, ...
        "TotalDropCount",opts.TotalDropCount);
    if seed.found
        identifiedSeed = string(seed.master_mat);
        seedSource = "PlantContextBank";
    else
        identifiedSeed = string(baseIdentified);
        seedSource = "BaseIdentifiedMat";
    end
end

fprintf("\n============================================================\n");
fprintf("[V30] Generic mission H=%.3f m V=%.3f m/s\n",H,V);
fprintf("[V30] OutputRoot: %s\n",missionRoot);
fprintf("[V30] Plant seed (%s): %s\n",seedSource,identifiedSeed);
fprintf("[V30] Shared controller LearningBank: %s\n",learningBankRoot);
if missionRecoveryOnly
    fprintf("[V30.6.1-MISSION] RECOVERY-ONLY mode; inherited cfg0..cfg4 VERIFIED state from: %s\n",missionRecoverySourceRoot);
    fprintf("[V30.6.1-MISSION] trim/ID/Plant rebuild/controller BO are DISABLED for this generation.\n");
end
fprintf("============================================================\n");

last = struct();
lastError = "";
missionComplete = false;
infrastructureFailure = false;
failureClass = "none";
attemptsUsed = 0;
attemptLimit=max(1,round(double(opts.MaxAttempts)));
if missionRecoveryOnly
    % One mission-only pass already contains the bounded v30.6 transition
    % refinement budget. Do not repeat cfg-free mission recovery twice in one call.
    attemptLimit=1;
end
for attempt = 1:attemptLimit
    attemptsUsed = attempt;
    fprintf("\n[V30] mission attempt %d/%d for H=%.3f V=%.3f\n", ...
        attempt,attemptLimit,H,V);
    try
        last = airdropx_auto_mpc_unified_learning( ...
            "IdentifiedMat",identifiedSeed, ...
            "OutputRoot",missionRoot, ...
            "LearningBankRoot",learningBankRoot, ...
            "TargetAltitudeM",H, ...
            "TargetAirspeedMps",V, ...
            "ReferenceMassKg",double(opts.ReferenceMassKg), ...
            "CargoMassKg",double(opts.CargoMassKg), ...
            "TotalDropCount",round(double(opts.TotalDropCount)), ...
            "ConfigIds",opts.ConfigIds, ...
            "UseParallel",logical(opts.UseParallel), ...
            "ParallelWorkers",round(double(opts.ParallelWorkers)), ...
            "UnifiedTransferSeedEvaluations",round(double(opts.UnifiedTransferSeedEvaluations)), ...
            "UnifiedAdditionalEvaluationsPerRun",round(double(opts.UnifiedAdditionalEvaluationsPerRun)), ...
            "UnifiedControllerNearPassEnabled",logical(opts.UnifiedControllerNearPassEnabled), ...
            "UnifiedControllerNearPassGateRatioMax",double(opts.UnifiedControllerNearPassGateRatioMax), ...
            "UnifiedControllerNearPassDeterministicEvaluations",round(double(opts.UnifiedControllerNearPassDeterministicEvaluations)), ...
            "UnifiedControllerNearPassBayesEvaluations",round(double(opts.UnifiedControllerNearPassBayesEvaluations)), ...
            "UnifiedControllerNearPassMaxRoundsPerContext",round(double(opts.UnifiedControllerNearPassMaxRoundsPerContext)), ...
            "UnifiedForceControllerLocalRefinement",logical(opts.UnifiedForceControllerLocalRefinement), ...
            "UnifiedV31LayeredLocalRefinement",logical(opts.UnifiedV31LayeredLocalRefinement), ...
            "BumplessTransitionEnabled",logical(opts.BumplessTransitionEnabled), ...
            "TransitionMoveTransferScale",double(opts.TransitionMoveTransferScale), ...
            "TransitionIntegralTransferScale",double(opts.TransitionIntegralTransferScale), ...
            "UniversalMissionNearPassEnabled",logical(opts.UniversalMissionNearPassEnabled), ...
            "UniversalMissionNearPassGateRatioMax",double(opts.UniversalMissionNearPassGateRatioMax), ...
            "UniversalMissionNearPassMaxNewEvaluationsPerAttempt",round(double(opts.UniversalMissionNearPassMaxNewEvaluationsPerAttempt)), ...
            "UniversalMissionNearPassMoveScales",double(opts.UniversalMissionNearPassMoveScales(:)), ...
            "UniversalMissionNearPassIntegralScales",double(opts.UniversalMissionNearPassIntegralScales(:)), ...
            "UniversalRecoveryNearPassGateRatioMax",double(opts.UniversalRecoveryNearPassGateRatioMax), ...
            "UniversalRecoveryExtendedProbeDurationS",double(opts.UniversalRecoveryExtendedProbeDurationS), ...
            "UniversalRecoveryExtendedTailWindowS",double(opts.UniversalRecoveryExtendedTailWindowS), ...
            "UniversalRecoveryLocalRetrimEvaluations",round(double(opts.UniversalRecoveryLocalRetrimEvaluations)), ...
            "RunFinalMissionValidation",true, ...
            "MissionRecoveryOnly",missionRecoveryOnly, ...
            "V31ContinuousControllerStateEnabled",logical(opts.V31ContinuousControllerStateEnabled), ...
            "V31HeightGovernorEnabled",logical(opts.V31HeightGovernorEnabled), ...
            "V31HeightVzSlewRateMps2",double(opts.V31HeightVzSlewRateMps2), ...
            "V31HeightBiasFraction",double(opts.V31HeightBiasFraction), ...
            "V31HeightBiasLeak",double(opts.V31HeightBiasLeak), ...
            "UnifiedV31ArchitectureRequal",logical(opts.UnifiedV31ArchitectureRequal));
        if isfield(last,"mission_complete")
            missionComplete = logical(last.mission_complete);
        else
            missionComplete = local_read_mission_complete(missionRoot);
        end
        if missionComplete, break; end
    catch ME
        lastError = string(getReport(ME,"extended","hyperlinks","off"));
        local_write_text(fullfile(missionRoot,"v30_last_error.txt"),lastError);
        [infrastructureFailure,failureClass] = local_classify_failure(ME,missionRoot);
        if infrastructureFailure
            warning("AirdropX:V30:InfrastructureFailure", ...
                "H=%.3f V=%.3f attempt %d stopped by infrastructure/runtime failure; attempt will not be treated as a flight-envelope failure: %s", ...
                H,V,attempt,ME.message);
            break;
        else
            if failureClass=="near_pass_trim" || failureClass=="real_trim_fail"
                warning("AirdropX:V30:RealTrimFailure", ...
                    "H=%.3f V=%.3f attempt %d classified as %s (REAL flight/trim result, not infrastructure): %s", ...
                    H,V,attempt,failureClass,ME.message);
            else
                warning("AirdropX:V30:MissionAttempt", ...
                    "H=%.3f V=%.3f attempt %d stopped [%s]: %s",H,V,attempt,failureClass,ME.message);
            end
        end
    end
end

[allVerified,missionPass] = local_read_status(missionRoot,last);
masterMat = fullfile(missionRoot,"identified_plants_200m_master.mat");
plantRegistered = false;
if logical(opts.RegisterPlant) && isfile(masterMat) && allVerified
    airdropx_auto_plant_context_bank( ...
        "Action","register", "ProjectRoot",paths.projectRoot, "Root",plantBankRoot, ...
        "TargetAltitudeM",H,"TargetAirspeedMps",V, ...
        "ReferenceMassKg",opts.ReferenceMassKg,"CargoMassKg",opts.CargoMassKg, ...
        "TotalDropCount",opts.TotalDropCount, ...
        "OutputRoot",missionRoot,"MasterMat",masterMat, ...
        "AllVerified",allVerified,"MissionPass",missionPass, ...
        "PlantValid",allVerified,"Source","v30_any_mission");
    plantRegistered = true;
end

summary = table(H,V,string(seedSource),string(identifiedSeed),string(missionRoot), ...
    attemptsUsed,logical(allVerified),logical(missionPass),logical(allVerified && missionPass), ...
    logical(plantRegistered),logical(infrastructureFailure),string(failureClass),string(lastError), ...
    'VariableNames',{'target_altitude_m','target_airspeed_mps','plant_seed_source', ...
    'plant_seed_mat','output_root','attempts_used','all_verified','final_mission_pass', ...
    'mission_complete','plant_registered','infrastructure_failure','failure_class','last_error'});
writetable(summary,fullfile(missionRoot,"v30_mission_summary.csv"));

result = struct();
result.target_altitude_m = H;
result.target_airspeed_mps = V;
result.output_root = string(missionRoot);
result.plant_seed_source = string(seedSource);
result.plant_seed_mat = string(identifiedSeed);
result.attempts_used = attemptsUsed;
result.all_verified = logical(allVerified);
result.final_mission_pass = logical(missionPass);
result.mission_complete = logical(allVerified && missionPass);
result.master_mat = string(masterMat);
result.plant_registered = logical(plantRegistered);
result.infrastructure_failure = logical(infrastructureFailure);
result.failure_class = string(failureClass);
result.mission_recovery_only = logical(missionRecoveryOnly);
result.mission_recovery_source_root = string(missionRecoverySourceRoot);
result.last_error = string(lastError);
result.v29_result = last;
result.summary = summary;
save(fullfile(missionRoot,"v30_any_mission_result.mat"),"result","opts","-v7.3");

if result.mission_complete
    fprintf("[V30] MISSION PASS: H=%.3f m V=%.3f m/s cfg0->cfg4 complete.\n",H,V);
else
    fprintf("[V30] Mission not yet complete: H=%.3f m V=%.3f m/s. Rerun the same context to resume.\n",H,V);
end
end

function [allVerified,missionPass] = local_read_status(root,last)
allVerified = false; missionPass = false;
if isstruct(last)
    if isfield(last,"all_verified"), allVerified = logical(last.all_verified); end
    if isfield(last,"final_mission_pass"), missionPass = logical(last.final_mission_pass); end
end
checkpointFile = fullfile(root,"airdropx_200m_cfg_checkpoint.mat");
if isfile(checkpointFile)
    try
        S = load(checkpointFile,"checkpoint");
        cp = S.checkpoint;
        if isfield(cp,"status") && numel(cp.status)>=5
            allVerified = all(string(cp.status(1:5))=="verified");
        end
        if isfield(cp,"final_mission_pass")
            missionPass = logical(cp.final_mission_pass);
        end
    catch
    end
end
summaryFile = fullfile(root,"final_mission_validation","final_mission_summary.csv");
if isfile(summaryFile)
    try
        T=readtable(summaryFile);
        if ismember("mission_pass",string(T.Properties.VariableNames)) && ~isempty(T)
            missionPass = logical(T.mission_pass(1));
        end
    catch
    end
end
end

function tf = local_read_mission_complete(root)
[a,b]=local_read_status(root,struct()); tf=logical(a&&b);
end

function [infra,cls] = local_classify_failure(ME,root)
% v30.3 failure classifier.
% Priority matters: a completed trim verification is a REAL control/trim
% observation even when older/incomplete unified-evaluation artifacts exist
% elsewhere under the same mission root. Do not let stale missing-summary
% files override an explicit NoUsableTrim result.
infra=false; cls="runtime_or_control_failure";
txt="";
try, txt=lower(string(ME.identifier)+" "+string(ME.message)); catch, end

if local_real_trim_text(txt)
    cls=local_trim_failure_class(txt);
    return;
end
if local_infra_text(txt)
    infra=true; cls="infra_invalid"; return;
end

% Generic wrapper exceptions can hide the real cause. Inspect current error
% artifacts, but give REAL trim evidence precedence over infrastructure text.
realTrimSeen=false; infraSeen=false;
try
    files=[dir(fullfile(root,"**","error.txt")); dir(fullfile(root,"**","bayesopt_wrapper_error.txt"))];
    for i=1:numel(files)
        try
            etxt=lower(string(fileread(fullfile(files(i).folder,files(i).name))));
            if local_real_trim_text(etxt), realTrimSeen=true; end
            if local_infra_text(etxt), infraSeen=true; end
        catch
        end
    end
catch
end
if realTrimSeen
    cls="real_trim_fail";
    return;
end
if infraSeen
    infra=true; cls="infra_invalid"; return;
end

% A missing certification summary is infrastructure only when the current
% failure is not an explicit real trim failure. This protects valid trim
% failures from stale/in-progress eval folders left by prior generations.
try
    rec=dir(fullfile(root,"**","unified_record.csv"));
    for i=1:numel(rec)
        evalDir=rec(i).folder;
        if ~isfile(fullfile(evalDir,"certification_summary.csv"))
            infra=true; cls="infra_missing_cert_summary"; return;
        end
    end
catch
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

function cls = local_trim_failure_class(txt)
cls="real_trim_fail";
% Keep near-pass separate for diagnostics only. It still consumes the normal
% mission attempt budget and can become an exhausted/boundary observation.
try
    tok=regexp(char(txt),'gate ratio\s+([0-9]+(?:\.[0-9]+)?)','tokens','once');
    if ~isempty(tok)
        ratio=str2double(tok{1});
        if isfinite(ratio) && ratio <= 2.0, cls="near_pass_trim"; end
    end
catch
end
end

function tf = local_infra_text(txt)
markers=["infrastructurefailure","infrastructure_exception", ...
    "path too long","filename or extension is too long", ...
    "file name or extension is too long","specified path, file name, or both are too long", ...
    "260 character","max_path", ...
    "database is full","no space left","disk full", ...
    "fetchnextfutureerrored","missinglogsout","missinglog","infranovalidevaluation"];
tf=false;
for i=1:numel(markers)
    if contains(txt,markers(i)), tf=true; return; end
end
if contains(txt,"certification_summary.csv") && ...
        (contains(txt,"not found") || contains(txt,"does not exist") || contains(txt,"unable to find") || contains(txt,"no such file"))
    tf=true;
end
end

function local_prepare_mission_recovery_state(projectRoot,sourceRoot,destRoot)
% v30.6.1 mission-state resume. The source generation is read-only. Existing
% destination state from the old v30.6 bug is backed up once, then replaced by
% the exact cfg0..cfg4 VERIFIED checkpoint/master and cfg4 combined bank.
marker=fullfile(destRoot,"V30_6_2_MISSION_RECOVERY_STATE_PREPARED.txt");
if isfile(marker), return; end
sourceRoot=string(sourceRoot); destRoot=string(destRoot);
srcCp=fullfile(sourceRoot,"airdropx_200m_cfg_checkpoint.mat");
srcMaster=fullfile(sourceRoot,"identified_plants_200m_master.mat");
if ~isfile(srcCp) || ~isfile(srcMaster)
    error("AirdropX:V30_6_1:BadMissionRecoverySource", ...
        "Source generation is missing checkpoint/master: %s",sourceRoot);
end
S=load(srcCp,"checkpoint");
if ~isfield(S,"checkpoint")
    error("AirdropX:V30_6_1:BadMissionRecoveryCheckpoint","checkpoint variable missing in %s",srcCp);
end
cp=S.checkpoint;
if ~isfield(cp,"status") || numel(cp.status)<5 || ~all(string(cp.status(1:5))=="verified")
    error("AirdropX:V30_6_1:SourceNotVerified", ...
        "Mission recovery source must have cfg0..cfg4 VERIFIED.");
end
if ~isfield(cp,"best_candidate") || numel(cp.best_candidate)<5 || any(cellfun(@isempty,cp.best_candidate(1:5)))
    error("AirdropX:V30_6_1:SourceMissingCandidates", ...
        "Mission recovery source is missing one or more VERIFIED best candidates.");
end

backupRoot=fullfile(destRoot,"v30_6_2_pre_mission_resume_backup");
if ~isfolder(backupRoot), mkdir(backupRoot); end
dstCp=fullfile(destRoot,"airdropx_200m_cfg_checkpoint.mat");
dstMaster=fullfile(destRoot,"identified_plants_200m_master.mat");
if isfile(dstCp), copyfile(dstCp,fullfile(backupRoot,"airdropx_200m_cfg_checkpoint_before_v30_6_2.mat"),'f'); end
if isfile(dstMaster), copyfile(dstMaster,fullfile(backupRoot,"identified_plants_before_v30_6_2.mat"),'f'); end
copyfile(srcMaster,dstMaster,'f');

% Make the recovery generation self-contained for Final Mission: copy the
% source cfg4 combined bank (it contains cfg0..cfg4 controllers) locally.
srcBank=local_mission_recovery_cfg4_bank(projectRoot,sourceRoot,cp);
dstCfg4=fullfile(destRoot,"cfg4"); if ~isfolder(dstCfg4), mkdir(dstCfg4); end
dstBank=fullfile(dstCfg4,"best_mpc_bank_200m.mat");
copyfile(srcBank,dstBank,'f');
if ~isfield(cp,"best_bank_path") || numel(cp.best_bank_path)<5
    cp.best_bank_path=strings(5,1);
end
cp.best_bank_path(5)=string(dstBank);
cp.all_verified=true;
% v30.6.2: only a MISSION-VERIFIED transition policy may be inherited.
% A previous best-measured-but-failed policy (for example full 1/1 transfer)
% must not become the next recovery baseline.
policySource="";
try, policySource=lower(string(cp.transition_policy_source)); catch, end
if ~contains(policySource,"verified")
    cp.transition_move_transfer_scale=NaN;
    cp.transition_integral_transfer_scale=NaN;
    cp.transition_policy_source="";
end
cp.final_mission_attempted=false;
cp.final_mission_pass=false;
cp.final_mission_summary=table();
cp.final_mission_updated_at="";
cp.updated_at=string(datetime("now"));
checkpoint=cp; %#ok<NASGU>
save(dstCp,"checkpoint","-v7.3");

for f=["all_config_status.csv","physical_nominals_200m.csv"]
    src=fullfile(sourceRoot,f); if isfile(src), copyfile(src,fullfile(destRoot,f),'f'); end
end
local_write_text(marker, ...
    "v30.6.2 mission-recovery-only state inherited from: "+sourceRoot+newline+ ...
    "cfg0..cfg4 controller/Plant/trim state preserved; Final Mission state reset.");
fprintf("[V30.6.2-MISSION] inherited VERIFIED checkpoint/master/bank from %s\n",sourceRoot);
end

function bank=local_mission_recovery_cfg4_bank(projectRoot,sourceRoot,cp)
candidates=strings(0,1);
if isfield(cp,"best_bank_path") && numel(cp.best_bank_path)>=5
    candidates(end+1,1)=string(cp.best_bank_path(5)); %#ok<AGROW>
end
candidates(end+1,1)=string(fullfile(sourceRoot,"cfg4","best_mpc_bank_200m.mat"));
for i=1:numel(candidates)
    q=string(candidates(i)); if strlength(q)==0, continue; end
    if isfile(q), bank=q; return; end
    q2=fullfile(projectRoot,q); if isfile(q2), bank=string(q2); return; end
end
error("AirdropX:V30_6_1:MissingSourceCfg4Bank", ...
    "Could not resolve the source VERIFIED cfg4 combined MPC bank.");
end

function local_write_text(file,text)
parent=fileparts(file); if ~isfolder(parent), mkdir(parent); end
fid=fopen(file,"w");
if fid>=0, fprintf(fid,"%s\n",char(text)); fclose(fid); end
end

function tag = local_context_tag(H,V)
tag = "H" + local_num_tag(H) + "_V" + local_num_tag(V);
end

function s = local_num_tag(x)
s = sprintf("%.3f",double(x));
s = strrep(s,"-","m"); s = strrep(s,".","p");
end

function paths = local_paths(projectRoot)
projectRoot=string(projectRoot); if strlength(projectRoot)==0, projectRoot=string(pwd); end
paths.projectRoot=projectRoot;
paths.matlabDir=fullfile(projectRoot,"matlab");
paths.mpcDir=fullfile(paths.matlabDir,"mpc");
paths.autoDir=fullfile(paths.matlabDir,"mpc_auto");
paths.sfuncDir=fullfile(paths.matlabDir,"sfunc_jsbsim");
end

function p = local_resolve_path(projectRoot,p)
p=string(p); if strlength(p)==0, return; end
c=char(p); isAbs=startsWith(c,'/') || startsWith(c,'\\') || ~isempty(regexp(c,'^[A-Za-z]:[\\/]','once'));
if ~isAbs, p=string(fullfile(projectRoot,p)); end
end

function opts = local_options(varargin)
opts=struct();
opts.ProjectRoot="";
opts.BaseIdentifiedMat="matlab/results/mpc_auto_id_v11_clean_r1/identify/airdropx_identified_plants.mat";
opts.OutputRoot="";
opts.AnyMissionRoot="matlab/results/mpc_auto_any_mission_v30";
opts.LearningBankRoot="matlab/results/mpc_auto_global_learning_bank";
opts.PlantBankRoot="matlab/results/mpc_auto_global_plant_bank";
opts.TargetAltitudeM=100.0;
opts.TargetAirspeedMps=50.0;
opts.MinimumAllowedAltitudeM=20.0;
opts.MaximumAllowedAltitudeM=200.0;
opts.ReferenceMassKg=3423.0;
opts.CargoMassKg=300.0;
opts.TotalDropCount=4;
opts.ConfigIds=(0:4).';
opts.UseParallel=true;
opts.ParallelWorkers=3;
opts.MaxAttempts=2;
opts.RegisterPlant=true;
% v30.6.1: when an earlier generation already verified cfg0..cfg4 and only
% Final Mission was near-pass, inherit that exact verified state and run
% only mission/transition recovery. No single-cfg learning may execute.
opts.MissionRecoveryOnly=false;
opts.MissionRecoverySourceRoot="";
opts.V31ContinuousControllerStateEnabled=false;
opts.V31HeightGovernorEnabled=false;
opts.V31HeightVzSlewRateMps2=0.30;
opts.V31HeightBiasFraction=0.70;
opts.V31HeightBiasLeak=1.0;
opts.UnifiedV31ArchitectureRequal=false;
opts.UnifiedTransferSeedEvaluations=5;
opts.UnifiedAdditionalEvaluationsPerRun=36;
% v30.5 universal controller near-pass refinement, identical for every H/V/cfg.
opts.UnifiedControllerNearPassEnabled=true;
opts.UnifiedControllerNearPassGateRatioMax=1.30;
opts.UnifiedControllerNearPassDeterministicEvaluations=16;
opts.UnifiedControllerNearPassBayesEvaluations=18;
opts.UnifiedControllerNearPassMaxRoundsPerContext=2;
% v31.1 explicit LOCAL-level controls. Legacy callers leave these false.
opts.UnifiedForceControllerLocalRefinement=false;
opts.UnifiedV31LayeredLocalRefinement=false;
% v30.6 universal bumpless cfg transition + full-mission near-pass recovery.
opts.BumplessTransitionEnabled=true;
opts.TransitionMoveTransferScale=0.0;
opts.TransitionIntegralTransferScale=0.0;
opts.UniversalMissionNearPassEnabled=true;
opts.UniversalMissionNearPassGateRatioMax=1.30;
opts.UniversalMissionNearPassMaxNewEvaluationsPerAttempt=4;
opts.UniversalMissionNearPassMoveScales=[0.00;0.00;0.25;0.25;0.50;0.50;0.75;1.00];
opts.UniversalMissionNearPassIntegralScales=[0.00;0.50;0.00;0.50;0.00;0.50;0.50;1.00];
% v30.4 universal recovery controls.  These values are forwarded unchanged
% for every H/V mission; they never depend on a specific altitude.
opts.UniversalRecoveryNearPassGateRatioMax=2.0;
opts.UniversalRecoveryExtendedProbeDurationS=80.0;
opts.UniversalRecoveryExtendedTailWindowS=20.0;
opts.UniversalRecoveryLocalRetrimEvaluations=24;
if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin)
    name=string(varargin{i}); if ~isfield(opts,name), error("Unknown option: %s",name); end
    opts.(name)=varargin{i+1};
end
opts.ParallelWorkers=min(max(1,round(double(opts.ParallelWorkers))),3);
end
