function result = airdropx_auto_run_closed_loop(varargin)
%AIRDROPX_AUTO_RUN_CLOSED_LOOP Run JSBSim closed loop with learned mpc().

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir);
addpath(paths.mpcDir);
addpath(paths.autoDir);
addpath(paths.sfuncDir);

mpcBankMat = string(opts.MpcBankMat);
if strlength(mpcBankMat) == 0
    mpcBankMat = local_default_mpc_bank(paths.matlabDir);
end
if strlength(mpcBankMat) == 0 || ~isfile(mpcBankMat)
    error("AirdropX:AutoMPC:MissingBank", "Pass MpcBankMat or train a learned bank first.");
end
% Accept both the legacy learned-bank layout and the UR-MPC v2 unified
% bank.  The latter intentionally has no top-level trim_bank/mpc_meta; its
% operating points live in ur_models and must be selected by speed/cfg.
[trimInitial, mpcMeta] = local_initial_bank_context(mpcBankMat, opts);

modelName = string(opts.Model);
modelPath = fullfile(paths.autoDir, modelName + ".slx");
if ~isfile(modelPath)
    airdropx_auto_setup_closed_loop_model("ProjectRoot", paths.projectRoot, "ModelName", modelName);
end
load_system(modelPath);
oldInitFcn = get_param(char(modelName), "InitFcn");
mpcBlock = char(modelName + "/MPC_Controller");
oldMpcParams = get_param(mpcBlock, "Parameters");

try
    set_param(char(modelName), "InitFcn", "");
    simCfg = airdropx_sim_params("ProjectRoot", paths.projectRoot, "Model", modelName, "AssignBase", true, ...
        "InitialAirspeedMps", opts.InitialAirspeedMps, ...
        "InitialAltitudeM", opts.InitialAltitudeM, ...
        "InitialPitchDeg", local_default_if_nan(opts.InitialPitchDeg, trimInitial.pitch_deg), ...
        "InitialFlightPathDeg", opts.InitialFlightPathDeg, ...
        "InitialHeadingDeg", opts.InitialHeadingDeg);

    local_assign_run_workspace(opts, mpcBankMat, trimInitial, mpcMeta);
    set_param(mpcBlock, "Parameters", "airdropx_auto_mpc_bank_mat_path");

    % v28: explicitly mark every certification-critical signal for logging on
    % each process worker. Do not depend on persistent SLX instrumentation.
    local_force_required_signal_logging(char(modelName));
    local_enable_signal_logging(char(modelName + "/MPC_ElevatorDelay"), "mpc_elevator_to_plant");
    local_enable_signal_logging(char(modelName + "/MPC_ThrottleDelay"), "mpc_throttle_to_plant");

    set_param(char(modelName), ...
        "StopTime", num2str(double(opts.StopTimeS), "%.15g"), ...
        "FixedStep", "dt", ...
        "SolverName", "FixedStepDiscrete", ...
        "SignalLogging", "on", ...
        "SignalLoggingName", "logsout");

    out = sim(char(modelName), ...
        "StopTime", num2str(double(opts.StopTimeS), "%.15g"), ...
        "FixedStep", num2str(simCfg.sim.dt_s, "%.15g"));

    set_param(mpcBlock, "Parameters", oldMpcParams);
    set_param(char(modelName), "InitFcn", oldInitFcn);
    set_param(char(modelName), "Dirty", "off");
catch ME
    if bdIsLoaded(char(modelName))
        set_param(mpcBlock, "Parameters", oldMpcParams);
        set_param(char(modelName), "InitFcn", oldInitFcn);
        set_param(char(modelName), "Dirty", "off");
    end
    rethrow(ME);
end

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    outputRoot = string(fullfile(paths.matlabDir, "results", ...
        "mpc_auto_closed_loop_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss"))));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

logs = local_get_logsout(out);
T = local_timeseries_table(logs, opts);
csvPath = fullfile(outputRoot, "closed_loop_timeseries.csv");
writetable(T, csvPath);
tracePath = "";
if logical(opts.V31ReferenceInstrumentationEnabled)
    TI = local_controller_reference_trace_table();
    if ~isempty(TI)
        tracePath = string(fullfile(outputRoot,"controller_reference_trace.csv"));
        writetable(TI,tracePath);
    end
end

summaryPath = fullfile(outputRoot, "summary.csv");
dropPath = fullfile(outputRoot, "drop_details.csv");
report = airdropx_mpc_evaluate_csv(csvPath, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps, ...
    "TargetPitchDeg", opts.TargetPitchDeg, ...
    "AfterDropTime", opts.AfterDropTime, ...
    "TargetNorthM", opts.TargetNorthM, ...
    "TargetEastM", opts.TargetEastM, ...
    "DropTargetNorthM", opts.DropTargetNorthM, ...
    "DropTargetEastM", opts.DropTargetEastM, ...
    "OutputFile", summaryPath, ...
    "DropDetailsFile", dropPath);

autoScore = airdropx_auto_score_closed_loop(T, ...
    "TargetAltitudeM", opts.TargetAltitudeM, "TargetAirspeedMps", opts.TargetAirspeedMps);

result = struct();
result.output_root = outputRoot;
result.mpc_bank_mat = mpcBankMat;
result.timeseries_csv = string(csvPath);
result.controller_reference_trace_csv = string(tracePath);
result.summary_csv = string(summaryPath);
result.drop_details_csv = string(dropPath);
result.timeseries = T;
result.summary = report.segments;
result.drop_table = report.drop_table;
result.auto_score = autoScore;
result.out = out;

fprintf("AirdropX auto MPC closed-loop exported:\n  %s\n", csvPath);
end

function [trimInitial, mpcMeta] = local_initial_bank_context(mpcBankMat, opts)
vars = string(who('-file', char(mpcBankMat)));
if any(vars == "trim_bank")
    bank = load(mpcBankMat, "trim_bank", "mpc_meta");
    if isfinite(double(opts.FixedConfigId))
        fixedCfg = min(max(round(double(opts.FixedConfigId)), 0), numel(bank.trim_bank)-1);
    else
        % Physical mission reset starts in cfg0 before any payload release.
        fixedCfg = 0;
    end
    trimInitial = bank.trim_bank(fixedCfg + 1);
    mpcMeta = struct();
    if isfield(bank, "mpc_meta"), mpcMeta = bank.mpc_meta; end
    return;
end

if any(vars == "ur_models") && any(vars == "ur_meta")
    bank = load(mpcBankMat, "ur_models", "ur_meta");
    nCfg = size(bank.ur_models, 2);
    if isfinite(double(opts.FixedConfigId))
        fixedCfg = min(max(round(double(opts.FixedConfigId)), 0), nCfg-1);
    else
        fixedCfg = 0;
    end
    speeds = double(bank.ur_meta.speed_nodes_mps(:));
    if isempty(speeds) || any(~isfinite(speeds))
        error("AirdropX:URMPC:BadSpeedEnvelope", "UR-MPC bank has no finite speed nodes.");
    end
    [~, ni] = min(abs(speeds - double(opts.InitialAirspeedMps)));
    m = bank.ur_models(ni, fixedCfg + 1);
    xNom = double(m.x_nominal(:));
    uNom = double(m.u_nominal(:));
    if numel(xNom) < 3 || numel(uNom) < 2 || any(~isfinite([xNom(3);uNom(1:2)]))
        error("AirdropX:URMPC:BadInitialNominal", ...
            "UR-MPC initial nominal is invalid at V=%.3f cfg%d.", speeds(ni), fixedCfg);
    end
    hidden = 0;
    if isfield(m, "hidden_elevator_offset") && isfinite(double(m.hidden_elevator_offset))
        hidden = double(m.hidden_elevator_offset);
    end
    % Legacy runner only needs these three fields for initial conditions.
    % elevator_cmd is the plant delta coordinate; u_nominal(1) is physical.
    trimInitial = struct("pitch_deg", xNom(3), ...
        "elevator_cmd", uNom(1)-hidden, "throttle_cmd", uNom(2));
    mpcMeta = struct();
    return;
end

error("AirdropX:AutoMPC:UnsupportedBankLayout", ...
    "Bank must contain legacy trim_bank or UR-MPC v2 ur_models+ur_meta: %s", mpcBankMat);
end

function local_assign_run_workspace(opts, mpcBankMat, trimInitial, mpcMeta)
assignin("base", "airdropx_stop_time_s", double(opts.StopTimeS));
assignin("base", "airdropx_target_altitude_m", double(opts.TargetAltitudeM));
assignin("base", "airdropx_pd_v_ref_mps", double(opts.TargetAirspeedMps));
assignin("base", "airdropx_pd_pitch_ref_deg", double(opts.TargetPitchDeg) + double(opts.ControlPitchBiasDeg));
assignin("base", "airdropx_auto_mpc_bank_mat_path", char(mpcBankMat));
assignin("base", "airdropx_auto_mpc_sample_time_s", 0.1);
assignin("base", "airdropx_auto_pitch_rate_filter_tau_s", 0.35);
assignin("base", "airdropx_mpc_reference_mass_kg", double(opts.ReferenceMassKg));
assignin("base", "airdropx_auto_cargo_mass_kg", double(opts.CargoMassKg));
assignin("base", "airdropx_cargo_mass_kg", double(opts.CargoMassKg));
assignin("base", "airdropx_auto_fixed_config_id", double(opts.FixedConfigId));
assignin("base", "airdropx_auto_hidden_elevator_trim", double(opts.HiddenElevatorTrim));
assignin("base", "airdropx_auto_mpc_enable_time_s", double(opts.MpcEnableTimeS));
assignin("base", "airdropx_auto_mpc_authority_scale", double(opts.MpcAuthorityScale));
assignin("base", "airdropx_auto_mpc_authority_by_config", double(opts.MpcAuthorityByConfig(:)));
assignin("base", "airdropx_auto_height_to_vz_gain", double(opts.HeightToVzGain));
assignin("base", "airdropx_auto_height_to_vz_gain_by_config", double(opts.HeightToVzGainByConfig(:)));
assignin("base", "airdropx_auto_height_integral_gain", double(opts.HeightIntegralGain));
assignin("base", "airdropx_auto_height_integral_gain_by_config", double(opts.HeightIntegralGainByConfig(:)));
assignin("base", "airdropx_auto_height_vz_ref_limit_mps", double(opts.HeightVzRefLimitMps));
assignin("base", "airdropx_auto_height_vz_ref_limit_by_config", double(opts.HeightVzRefLimitByConfig(:)));
% v30.6 universal cfg-transition policy. These are global controller bridge
% settings and apply identically to every altitude/airspeed/cfg transition.
assignin("base", "airdropx_auto_bumpless_transition_enabled", double(opts.BumplessTransitionEnabled));
assignin("base", "airdropx_auto_transition_move_transfer_scale", double(opts.TransitionMoveTransferScale));
assignin("base", "airdropx_auto_transition_integral_transfer_scale", double(opts.TransitionIntegralTransferScale));
% v31 continuous controller-state policy. When enabled, cfg changes keep a
% physical-command rate-continuous actuator state and one global altitude->vz
% integral contribution. v30 transition transfer scales are ignored.
assignin("base", "airdropx_v31_continuous_controller_state_enabled", double(opts.V31ContinuousControllerStateEnabled));
% v31.2 single-channel height governor. These are controller-architecture
% settings, not mission/cfg-specific rescue knobs.
assignin("base", "airdropx_v31_height_governor_enabled", double(opts.V31HeightGovernorEnabled));
assignin("base", "airdropx_v31_height_vz_slew_rate_mps2", double(opts.V31HeightVzSlewRateMps2));
assignin("base", "airdropx_v31_height_bias_fraction", double(opts.V31HeightBiasFraction));
assignin("base", "airdropx_v31_height_bias_leak", double(opts.V31HeightBiasLeak));
% v31.3 dynamic-reference layer.  The S-function reads a numeric Nx3 profile
% [time_s, altitude_command_m, airspeed_command_mps] at every control sample.
% Requested speed is passed through an acceleration/deceleration governor before
% it reaches the inner MPC; altitude remains a command and the height governor
% converts its error into a slew-limited vertical-speed reference.
profile = local_normalize_reference_profile(opts.DynamicReferenceProfile, opts);
assignin("base", "airdropx_v31_3_dynamic_reference_enabled", double(opts.V31DynamicReferenceEnabled));
assignin("base", "airdropx_v31_3_reference_profile", double(profile));
assignin("base", "airdropx_v31_3_speed_governor_enabled", double(opts.V31SpeedGovernorEnabled));
assignin("base", "airdropx_v31_3_speed_accel_limit_mps2", double(opts.V31SpeedAccelLimitMps2));
assignin("base", "airdropx_v31_3_speed_decel_limit_mps2", double(opts.V31SpeedDecelLimitMps2));
assignin("base", "airdropx_v31_3_scheduler_enabled", double(opts.V31SchedulerEnabled));
assignin("base", "airdropx_v31_3_scheduler_bank_mat_path", char(string(opts.V31SchedulerBankMat)));
assignin("base", "airdropx_v31_3_reference_instrumentation_enabled", double(opts.V31ReferenceInstrumentationEnabled));
assignin("base", "airdropx_v31_3_controller_reference_trace", zeros(0,23));
assignin("base", "airdropx_auto_test_pulse1_start_s", double(opts.TestPulse1StartS));
assignin("base", "airdropx_auto_test_pulse1_duration_s", double(opts.TestPulse1DurationS));
assignin("base", "airdropx_auto_test_pulse1_elevator", double(opts.TestPulse1Elevator));
assignin("base", "airdropx_auto_test_pulse1_throttle", double(opts.TestPulse1Throttle));
assignin("base", "airdropx_auto_test_pulse2_start_s", double(opts.TestPulse2StartS));
assignin("base", "airdropx_auto_test_pulse2_duration_s", double(opts.TestPulse2DurationS));
assignin("base", "airdropx_auto_test_pulse2_elevator", double(opts.TestPulse2Elevator));
assignin("base", "airdropx_auto_test_pulse2_throttle", double(opts.TestPulse2Throttle));
assignin("base", "airdropx_auto_elevator_dev_step_limit", double(opts.ElevatorDevStepLimit));
assignin("base", "airdropx_auto_throttle_dev_step_limit", double(opts.ThrottleDevStepLimit));
assignin("base", "airdropx_auto_trust_h_m", double(opts.TrustAltitudeM));
assignin("base", "airdropx_auto_trust_V_mps", double(opts.TrustAirspeedMps));
assignin("base", "airdropx_auto_trust_pitch_deg", double(opts.TrustPitchDeg));
assignin("base", "airdropx_auto_trust_vz_mps", double(opts.TrustVzMps));
assignin("base", "airdropx_auto_trust_q_dps", double(opts.TrustQDps));
assignin("base", "airdropx_auto_use_trim_pitch_reference", double(opts.UseTrimPitchReference));

assignin("base", "airdropx_drop_mode", double(opts.DropMode));
assignin("base", "airdropx_fixed_drop_start_s", double(opts.FixedDropStartS));
assignin("base", "airdropx_fixed_drop_interval_s", double(opts.FixedDropIntervalS));
assignin("base", "airdropx_fixed_drop_total", double(opts.FixedDropTotal));

assignin("base", "airdropx_carp_target_n_m", double(opts.TargetNorthM));
assignin("base", "airdropx_carp_target_e_m", double(opts.TargetEastM));
assignin("base", "airdropx_carp_interval_s", double(opts.CarpIntervalS));
assignin("base", "airdropx_carp_target_offset_n_m", double(opts.DropTargetNorthM(:)));
assignin("base", "airdropx_carp_target_offset_e_m", double(opts.DropTargetEastM(:)));
for i = 1:min(4, numel(opts.DropTargetNorthM))
    assignin("base", sprintf("airdropx_carp_target_offset_n_%d_m", i), double(opts.DropTargetNorthM(i)));
end
for i = 1:min(4, numel(opts.DropTargetEastM))
    assignin("base", sprintf("airdropx_carp_target_offset_e_%d_m", i), double(opts.DropTargetEastM(i)));
end

if isfinite(double(opts.InitialElevatorDelta))
    initialElevator = double(opts.InitialElevatorDelta);
else
    initialElevator = double(trimInitial.elevator_cmd);
    try
        if isfield(mpcMeta, "input_coordinate_mode") && string(mpcMeta.input_coordinate_mode) == "deviation_physical" && ...
                isfield(mpcMeta, "physical_elevator_nominals") && isfinite(double(opts.HiddenElevatorTrim))
            physicalNominals = double(mpcMeta.physical_elevator_nominals(:));
            if isfinite(double(opts.FixedConfigId))
                cfgIdx = min(max(round(double(opts.FixedConfigId)), 0), numel(physicalNominals)-1) + 1;
            else
                cfgIdx = 1; % physical reset always begins in cfg0
            end
            initialElevator = physicalNominals(cfgIdx) - double(opts.HiddenElevatorTrim);
        end
    catch
    end
end
if isfinite(double(opts.InitialThrottleCmd))
    initialThrottle = double(opts.InitialThrottleCmd);
else
    initialThrottle = double(trimInitial.throttle_cmd);
end
assignin("base", "airdropx_initial_elevator_delta", initialElevator);
assignin("base", "airdropx_initial_throttle_cmd", initialThrottle);
end

function profile = local_normalize_reference_profile(profileIn, opts)
% Return sorted unique Nx3 [time,H,V].  Piecewise-constant commands are used
% intentionally: the reference governors, not interpolation of the user's
% request, determine the physically admissible transition.
if ~logical(opts.V31DynamicReferenceEnabled) || isempty(profileIn)
    profile = [0.0, double(opts.TargetAltitudeM), double(opts.TargetAirspeedMps)];
    return;
end
if istable(profileIn)
    names=string(profileIn.Properties.VariableNames);
    t=local_table_col(profileIn,names,["time_s","time","t"]);
    h=local_table_col(profileIn,names,["altitude_m","target_altitude_m","h_m","H"]);
    v=local_table_col(profileIn,names,["airspeed_mps","target_airspeed_mps","v_mps","V"]);
    profile=[t(:),h(:),v(:)];
elseif isstruct(profileIn)
    t=local_struct_col(profileIn,["time_s","time","t"]);
    h=local_struct_col(profileIn,["altitude_m","target_altitude_m","h_m","H"]);
    v=local_struct_col(profileIn,["airspeed_mps","target_airspeed_mps","v_mps","V"]);
    profile=[t(:),h(:),v(:)];
else
    profile=double(profileIn);
end
if size(profile,2)~=3 || isempty(profile)
    error("AirdropX:V31_3:BadReferenceProfile", ...
        "DynamicReferenceProfile must be Nx3 [time_s altitude_m airspeed_mps].");
end
profile=double(profile);
profile=profile(all(isfinite(profile),2),:);
if isempty(profile), error("AirdropX:V31_3:EmptyReferenceProfile","Reference profile has no finite rows."); end
profile(:,1)=max(profile(:,1),0.0);
profile=sortrows(profile,1);
[~,ia]=unique(profile(:,1),'last'); profile=profile(sort(ia),:);
if profile(1,1)>0
    profile=[0.0,profile(1,2),profile(1,3);profile];
end
end

function x=local_table_col(T,names,candidates)
x=[];
for q=candidates
    idx=find(strcmpi(names,q),1);
    if ~isempty(idx), x=double(T.(T.Properties.VariableNames{idx})); return; end
end
error("AirdropX:V31_3:BadReferenceProfile","Missing reference-profile column: %s",strjoin(candidates,"/"));
end

function x=local_struct_col(S,candidates)
x=[];
fn=string(fieldnames(S));
for q=candidates
    idx=find(strcmpi(fn,q),1);
    if ~isempty(idx), x=double(S.(char(fn(idx)))); return; end
end
error("AirdropX:V31_3:BadReferenceProfile","Missing reference-profile field: %s",strjoin(candidates,"/"));
end

function [hReq,vReq,vGov]=local_reference_trace(t,opts)
profile=local_normalize_reference_profile(opts.DynamicReferenceProfile,opts);
t=double(t(:));
hReq=local_previous_interp(profile(:,1),profile(:,2),t);
vReq=local_previous_interp(profile(:,1),profile(:,3),t);
vGov=vReq;
if logical(opts.V31DynamicReferenceEnabled) && logical(opts.V31SpeedGovernorEnabled) && ~isempty(t)
    vGov=zeros(size(vReq));
    vGov(1)=double(opts.InitialAirspeedMps);
    if ~isfinite(vGov(1)), vGov(1)=vReq(1); end
    for k=2:numel(t)
        dt=max(0.0,t(k)-t(k-1));
        dv=vReq(k)-vGov(k-1);
        if dv>=0, step=double(opts.V31SpeedAccelLimitMps2)*dt;
        else, step=double(opts.V31SpeedDecelLimitMps2)*dt; end
        step=max(0.0,step);
        vGov(k)=vGov(k-1)+min(max(dv,-step),step);
    end
end
end

function y=local_previous_interp(x,v,xq)
y=zeros(size(xq));
for k=1:numel(xq)
    idx=find(x<=xq(k)+1e-12,1,'last');
    if isempty(idx), idx=1; end
    y(k)=v(idx);
end
end

function value = local_default_if_nan(value, fallback)
if ~isfinite(double(value)), value = fallback; end
end

function local_force_required_signal_logging(modelName)
% v28 worker-safe signal logging.
% Root Demux port mapping is stable in AirdropX and is the authoritative
% source of the state/actuator signals used by certification.
demuxPath = char(string(modelName) + "/Demux");
if getSimulinkBlockHandle(demuxPath) < 0
    error("AirdropX:AutoMPC:MissingStateDemux", ...
        "Required root Demux block is missing: %s", demuxPath);
end
ph = get_param(demuxPath, "PortHandles");
if ~isfield(ph,"Outport") || isempty(ph.Outport)
    error("AirdropX:AutoMPC:MissingStateDemuxPorts", ...
        "Required root Demux has no output ports: %s", demuxPath);
end
spec = { ...
     2, "altitude_m"; ...
     3, "vz_up_mps"; ...
     4, "airspeed_mps"; ...
     6, "pitch_deg"; ...
     8, "heading_deg"; ...
    10, "mass_kg"; ...
    11, "cg_x_m"; ...
    12, "pos_n_m"; ...
    13, "pos_e_m"; ...
    14, "elevator_cmd_norm"; ...
    15, "throttle_norm"; ...
    16, "wind_n_mps"; ...
    17, "wind_e_mps"; ...
    18, "drop_count" ...
    };
for k=1:size(spec,1)
    portIndex=double(spec{k,1});
    signalName=string(spec{k,2});
    if portIndex>numel(ph.Outport) || ph.Outport(portIndex)<=0
        error("AirdropX:AutoMPC:MissingRequiredLoggedPort", ...
            "Demux output %d required for %s is unavailable.",portIndex,signalName);
    end
    set_param(ph.Outport(portIndex), ...
        "DataLogging","on", ...
        "DataLoggingNameMode","Custom", ...
        "DataLoggingName",char(signalName));
end
set_param(modelName,"SignalLogging","on","SignalLoggingName","logsout");
end

function logs = local_get_logsout(out)
logs=[];
try
    logs=out.logsout;
catch
    try, logs=out.get("logsout"); catch, end
end
if isempty(logs)
    error("AirdropX:AutoMPC:MissingLogsout", ...
        "SimulationOutput does not contain a usable logsout dataset.");
end
end

function local_enable_signal_logging(blockPath, signalName)
try
    ph = get_param(blockPath, "PortHandles");
    if isfield(ph, "Outport") && ~isempty(ph.Outport)
        p = ph.Outport(1);
        set_param(p, "DataLogging", "on", ...
            "DataLoggingNameMode", "Custom", ...
            "DataLoggingName", char(signalName));
    end
catch ME
    warning("AirdropX:AutoMPC:SignalLogging", ...
        "Could not enable logging for %s: %s", blockPath, ME.message);
end
end

function T = local_timeseries_table(logs, opts)
signals = [
    "altitude_m"
    "vz_up_mps"
    "airspeed_mps"
    "pitch_deg"
    "q_dps"
    "mass_kg"
    "cg_x_m"
    "mpc_elevator_to_plant"
    "mpc_throttle_to_plant"
    "elevator_delta"
    "elevator_cmd_norm"
    "throttle_norm"
    "throttle_cmd"
    "drop_count"
    "selected_drop_cmd"
    "drop_cmd"
    "pos_n_m"
    "pos_e_m"
    "heading_deg"
    "wind_n_mps"
    "wind_e_mps"
    "actual_release_n_m"
    "actual_release_e_m"
    "actual_release_alt_m"
    "release_airspeed_mps"
    "release_heading_deg"
    "release_wind_n_mps"
    "release_wind_e_mps"
    "predicted_impact_n_m"
    "predicted_impact_e_m"
    "drop_trim_bias"
    "h_err"
    "saturated"
    "u_out"
    "u_pd"
    "u_total"
    ];

[tRef, ~] = local_signal(logs, "altitude_m");
if isempty(tRef)
    [tRef, ~] = local_signal(logs, "mpc_state_1");
end
if isempty(tRef)
    names = local_logged_signal_names(logs);
    error("AirdropX:AutoMPC:MissingLog", ...
        "logsout does not contain altitude_m or mpc_state_1. Available logged signals: %s", ...
        strjoin(cellstr(names), ", "));
end
T = table(tRef(:), repmat(string(opts.CaseId), numel(tRef), 1), ...
    'VariableNames', {'time_s', 'case_id'});

for i = 1:numel(signals)
    [t, y] = local_signal_with_alias(logs, signals(i));
    if isempty(t)
        T.(char(signals(i))) = NaN(height(T), 1);
    elseif isequal(t(:), tRef(:))
        T.(char(signals(i))) = y(:);
    else
        T.(char(signals(i))) = interp1(t(:), y(:), tRef(:), "linear", "extrap");
    end
end
if all(isnan(T.q_dps)) && ~all(isnan(T.pitch_deg)) && height(T) > 1
    T.q_dps = gradient(double(T.pitch_deg), double(T.time_s));
end
if all(isnan(T.elevator_delta)) && ~all(isnan(T.elevator_cmd_norm))
    T.elevator_delta = T.elevator_cmd_norm;
end
if all(isnan(T.throttle_cmd)) && ~all(isnan(T.throttle_norm))
    T.throttle_cmd = T.throttle_norm;
end

T.hidden_elevator_trim = double(opts.HiddenElevatorTrim) * ones(height(T), 1);
T.bridge_physical_elevator_cmd = T.mpc_elevator_to_plant + double(opts.HiddenElevatorTrim);
T.bridge_elevator_error = T.elevator_cmd_norm - T.bridge_physical_elevator_cmd;
T.bridge_throttle_error = T.throttle_norm - T.mpc_throttle_to_plant;

[requestedH, requestedV, governedV] = local_reference_trace(double(T.time_s), opts);
T.requested_altitude_m = requestedH;
T.requested_airspeed_mps = requestedV;
T.governed_airspeed_ref_mps = governedV;
% Compatibility aliases.  In dynamic-reference runs these are time-varying.
T.target_altitude_m = requestedH;
T.target_airspeed_mps = requestedV;
T.target_pitch_deg = double(opts.TargetPitchDeg) * ones(height(T), 1);
end

function names = local_logged_signal_names(logs)
names = strings(0,1);
try
    if isa(logs,"Simulink.SimulationData.Dataset")
        names = strings(logs.numElements,1);
        for k=1:logs.numElements
            try
                el=logs.get(k);
                names(k)=string(el.Name);
            catch
                names(k)="<unreadable>";
            end
        end
    else
        try, names=string(logs.getElementNames()); catch, end
    end
catch
end
names=names(strlength(names)>0);
end

function [t, y] = local_signal_with_alias(logs, name)
[t, y] = local_signal(logs, name);
if ~isempty(t)
    return;
end
switch string(name)
    case "altitude_m"
        [t, y] = local_signal(logs, "mpc_state_1");
    case "vz_up_mps"
        [t, y] = local_signal(logs, "mpc_state_2");
    case "airspeed_mps"
        [t, y] = local_signal(logs, "mpc_state_3");
    case "pitch_deg"
        [t, y] = local_signal(logs, "mpc_state_4");
    case "mass_kg"
        [t, y] = local_signal(logs, "mpc_state_5");
    case "cg_x_m"
        [t, y] = local_signal(logs, "mpc_state_6");
end
end

function [t, y] = local_signal(logs, name)
t = [];
y = [];
try
    if isa(logs, "Simulink.SimulationData.Dataset")
        for k = 1:logs.numElements
            el = logs.get(k);
            if string(el.Name) == string(name)
                values = el.Values;
                t = double(values.Time(:));
                y = double(values.Data(:));
                return;
            end
        end
        return;
    end
    el = logs.get(string(name));
    if isempty(el)
        return;
    end
    values = el.Values;
    t = double(values.Time(:));
    y = double(values.Data(:));
catch
end
end

function bankPath = local_default_mpc_bank(matlabDir)
candidates = string(fullfile(matlabDir, "results", "mpc_auto_train_final_allruns", "airdropx_learned_mpc.mat"));
if isfile(candidates)
    bankPath = candidates;
    return;
end
files = dir(fullfile(matlabDir, "results", "mpc_auto_train*", "airdropx_learned_mpc.mat"));
if isempty(files)
    bankPath = "";
    return;
end
[~, idx] = max([files.datenum]);
bankPath = string(fullfile(files(idx).folder, files(idx).name));
end

function paths = local_paths(projectRoot)
projectRoot = string(projectRoot);
if strlength(projectRoot) == 0
    thisDir = fileparts(mfilename("fullpath"));
    matlabDir = fileparts(thisDir);
    projectRoot = string(fileparts(matlabDir));
else
    matlabDir = fullfile(projectRoot, "matlab");
end
paths.projectRoot = char(projectRoot);
paths.matlabDir = char(matlabDir);
paths.mpcDir = char(fullfile(matlabDir, "mpc"));
paths.autoDir = char(fullfile(matlabDir, "mpc_auto"));
paths.sfuncDir = char(fullfile(matlabDir, "sfunc_jsbsim"));
end


function T = local_controller_reference_trace_table()
T=table();
try
    if ~evalin("base","exist('airdropx_v31_3_controller_reference_trace','var')"), return; end
    X=double(evalin("base","airdropx_v31_3_controller_reference_trace"));
    if isempty(X)||size(X,2)~=23, return; end
    names={'time_s','cfg_id','requested_h_internal_m','requested_v_internal_mps','governed_v_internal_mps', ...
        'actual_h_internal_m','actual_v_internal_mps','height_error_internal_m','height_gain','height_integral_gain', ...
        'height_vz_limit_mps','height_bias_mps','raw_vz_ref_mps','limited_vz_ref_mps','slew_vz_ref_mps', ...
        'actual_vz_internal_mps','trust_ok','scheduler_enabled','scheduler_low_speed_mps','scheduler_high_speed_mps', ...
        'scheduler_weight_high','plant_elevator_delta','plant_throttle_cmd'};
    T=array2table(X,'VariableNames',names);
catch
    T=table();
end
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.Model = "airdropx_auto_mpc_closed_loop";
opts.MpcBankMat = "";
opts.OutputRoot = "";
opts.CaseId = "auto_closed_loop_001";
opts.StopTimeS = 35.0;
opts.AfterDropTime = 10.0;
opts.DropMode = 1.0;
opts.FixedDropStartS = 10.0;
opts.FixedDropIntervalS = 0.2;
opts.FixedDropTotal = 4.0;
opts.CarpIntervalS = 0.2;
opts.InitialAirspeedMps = 50.0;
opts.InitialAltitudeM = 20.0;
opts.InitialPitchDeg = NaN;
opts.InitialFlightPathDeg = 0.0;
opts.InitialHeadingDeg = 0.0;
opts.InitialElevatorDelta = NaN;
opts.InitialThrottleCmd = NaN;
opts.FixedConfigId = 0;
opts.HiddenElevatorTrim = NaN;
opts.MpcEnableTimeS = 2.0;
opts.MpcAuthorityScale = 1.0;
opts.MpcAuthorityByConfig = NaN(5,1);
opts.HeightToVzGain = 0.0;
opts.HeightToVzGainByConfig = NaN(5,1);
opts.HeightIntegralGain = 0.0;
opts.HeightIntegralGainByConfig = NaN(5,1);
opts.HeightVzRefLimitMps = 0.8;
opts.HeightVzRefLimitByConfig = NaN(5,1);
% v30.6 universal bumpless transition. 1/1 means exact physical-move and
% integral-contribution continuity; no target-height or cfg special cases.
opts.BumplessTransitionEnabled = true;
opts.TransitionMoveTransferScale = 1.0;
opts.TransitionIntegralTransferScale = 1.0;
opts.V31ContinuousControllerStateEnabled = false;
opts.V31HeightGovernorEnabled = false;
opts.V31HeightVzSlewRateMps2 = 0.30;
opts.V31HeightBiasFraction = 0.70;
opts.V31HeightBiasLeak = 1.0;
% v31.3 dynamic command/scheduling options. DynamicReferenceProfile accepts
% numeric Nx3 [time_s Hcmd_m Vcmd_mps], a table with equivalent columns, or
% a struct with time_s/altitude_m/airspeed_mps vectors.
opts.V31DynamicReferenceEnabled = false;
opts.DynamicReferenceProfile = [];
opts.V31SpeedGovernorEnabled = true;
opts.V31SpeedAccelLimitMps2 = 0.75;
opts.V31SpeedDecelLimitMps2 = 1.00;
opts.V31SchedulerEnabled = false;
opts.V31SchedulerBankMat = "";
opts.V31ReferenceInstrumentationEnabled = false;
opts.TestPulse1StartS = Inf;
opts.TestPulse1DurationS = 0.0;
opts.TestPulse1Elevator = 0.0;
opts.TestPulse1Throttle = 0.0;
opts.TestPulse2StartS = Inf;
opts.TestPulse2DurationS = 0.0;
opts.TestPulse2Elevator = 0.0;
opts.TestPulse2Throttle = 0.0;
opts.ElevatorDevStepLimit = 0.006;
opts.ThrottleDevStepLimit = 0.010;
opts.TrustAltitudeM = 5.0;
opts.TrustAirspeedMps = 4.0;
opts.TrustPitchDeg = 4.0;
opts.TrustVzMps = 2.5;
opts.TrustQDps = 4.0;
opts.UseTrimPitchReference = 1.0;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.TargetPitchDeg = 0.0;
opts.ControlPitchBiasDeg = 0.0;
opts.ReferenceMassKg = 3423.0;
opts.CargoMassKg = 300.0;
opts.TargetNorthM = 1000.0;
opts.TargetEastM = 0.0;
opts.DropTargetNorthM = [0.8; 1.6; 2.4; 3.8];
opts.DropTargetEastM = [0.0; 0.0; 0.0; 0.0];
if mod(numel(varargin), 2) ~= 0
    error("Options must be name-value pairs.");
end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name)
        error("Unknown option: %s", name);
    end
    opts.(name) = varargin{i + 1};
end
end
