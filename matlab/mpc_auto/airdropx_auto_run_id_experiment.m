function result = airdropx_auto_run_id_experiment(varargin)
%AIRDROPX_AUTO_RUN_ID_EXPERIMENT Export one fixed-configuration ID run.
%
% DirectIdMode (default true) temporarily bypasses the legacy
% PD_Controller -> Unit Delay1/2 base-command path. During the run, the ID
% Sum blocks receive a constant trim plus zero-centered excitation, so the
% JSBSim input is exactly the requested operating point plus excitation.
% The .slx file is not saved or permanently modified.

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir);
addpath(paths.mpcDir);
addpath(paths.autoDir);
addpath(paths.sfuncDir);

% v21: when trim bayesopt runs on a process pool, each worker receives its
% own Simulink cache/codegen folder. This lets cfg3/cfg4 trim safely use up to
% five workers without slprj/cache collisions.
local_prepare_parallel_filegen();

trim = opts.Trim;
if isempty(trim)
    bank = airdropx_auto_default_trim_bank("TargetAltitudeM", opts.TargetAltitudeM, "TargetAirspeedMps", opts.TargetAirspeedMps);
    trim = bank(double(opts.ConfigId) + 1);
end

% v1.3.1 NO-MEX direct-cfg mode.  Instead of requiring a rebuilt S-function
% to remove payloads before JSBSim autoTrimSettle(), create an isolated
% aircraft definition whose first cfg point masses have zero weight.  The
% existing compiled MEX therefore loads the correct mass/CG from t=0.
aircraftVariantCleanup = []; %#ok<NASGU>
if logical(opts.DirectCfgViaAircraftXml) && ~logical(opts.PrepareByDrops) && round(double(opts.ConfigId)) > 0
    [variantName,variantDir] = local_make_cfg_aircraft_variant(paths.projectRoot,char(string(opts.AircraftName)),round(double(opts.ConfigId)));
    opts.AircraftName = string(variantName);
    aircraftVariantCleanup = onCleanup(@() local_delete_dir_quiet(variantDir)); %#ok<NASGU>
end

modelName = string(opts.Model);
modelPath = fullfile(paths.mpcDir, modelName + ".slx");
if ~isfile(modelPath)
    error("AirdropX:AutoMPC:MissingModel", "ID model not found: %s", modelPath);
end

load_system(modelPath);
oldInitFcn = get_param(char(modelName), "InitFcn");
directPatch = struct("applied", false, "legacy_lines_removed", false);
try
    set_param(char(modelName), "InitFcn", "");
    isolatedIcName = "";
    isolatedIcCleanup = [];
    if logical(opts.IsolateGeneratedIc)
        % v1.2 stability fix: airdropx_sim_params normally writes every run
        % to one shared reset_20m_runtime.xml. Process-pool workers at
        % different speed nodes therefore race on the same file. Build a
        % per-run IC file and pass it explicitly so JSBSim can never read
        % another worker's initial speed/pitch/altitude.
        isolatedIcName = local_write_isolated_initial_condition(paths.projectRoot, opts);
        isolatedIcCleanup = onCleanup(@() local_delete_file_quiet(isolatedIcName)); %#ok<NASGU>
        simCfg = airdropx_sim_params("ProjectRoot", paths.projectRoot, "AircraftName", opts.AircraftName, ...
            "IcName", isolatedIcName, "Model", modelName, "AssignBase", true, ...
            "InitialAirspeedMps", opts.InitialAirspeedMps, "InitialAltitudeM", opts.InitialAltitudeM, ...
            "InitialPitchDeg", opts.InitialPitchDeg, "InitialFlightPathDeg", opts.InitialFlightPathDeg, ...
            "InitialHeadingDeg", opts.InitialHeadingDeg);
    else
        simCfg = airdropx_sim_params("ProjectRoot", paths.projectRoot, "AircraftName", opts.AircraftName, ...
            "Model", modelName, "AssignBase", true, ...
            "InitialAirspeedMps", opts.InitialAirspeedMps, "InitialAltitudeM", opts.InitialAltitudeM, ...
            "InitialPitchDeg", opts.InitialPitchDeg, "InitialFlightPathDeg", opts.InitialFlightPathDeg, ...
            "InitialHeadingDeg", opts.InitialHeadingDeg);
    end

    assignin("base", "airdropx_stop_time_s", double(opts.StopTimeS));
    assignin("base", "airdropx_drop_mode", 1.0);
    assignin("base", "airdropx_fixed_drop_start_s", double(opts.PrepDropStartS));
    assignin("base", "airdropx_fixed_drop_interval_s", double(opts.PrepDropIntervalS));
    oldDropInit=local_capture_base_drop_init(); %#ok<NASGU>
    dropInitCleanup=onCleanup(@() local_restore_base_drop_init(oldDropInit)); %#ok<NASGU>
    % v1.5 NO-MEX safety: DirectCfgViaAircraftXml already encodes cfg mass/CG
    % by zeroing the corresponding cargo point masses in a temporary aircraft
    % XML. Never also request native initial-drop handling. Some existing MEX
    % builds may already support airdropx_initial_drop_count; enabling both
    % mechanisms would double-remove payload and corrupt mass/CG.
    if logical(opts.DirectCfgViaAircraftXml) && ~logical(opts.PrepareByDrops)
        assignin("base", "airdropx_enable_initial_drop_count", 0.0);
        assignin("base", "airdropx_initial_drop_count", 0.0);
    else
        assignin("base", "airdropx_enable_initial_drop_count", double(~logical(opts.PrepareByDrops)));
        assignin("base", "airdropx_initial_drop_count", double(opts.InitialDropCount));
    end
    if logical(opts.PrepareByDrops)
        assignin("base", "airdropx_fixed_drop_total", double(opts.ConfigId));
    else
        assignin("base", "airdropx_fixed_drop_total", 0.0);
    end
    % v29 mission-context mass variables. These are used by the controller
    % config estimator and by any model component that consumes the shared
    % cargo/reference-mass workspace variables.
    assignin("base", "airdropx_mpc_reference_mass_kg", double(opts.ReferenceMassKg));
    assignin("base", "airdropx_auto_cargo_mass_kg", double(opts.CargoMassKg));
    assignin("base", "airdropx_cargo_mass_kg", double(opts.CargoMassKg));
    assignin("base", "airdropx_initial_elevator_delta", double(trim.elevator_cmd));
    assignin("base", "airdropx_initial_throttle_cmd", double(trim.throttle_cmd));

    % v20: cfg2+ trim/ID runs must not apply the target-configuration
    % command before the preparatory payload drops.  The old direct-ID path
    % used one Constant block from t=0, so a cfg2 candidate (about +0.31
    % external elevator) was incorrectly applied while the aircraft was
    % still cfg0/cfg1.  That contaminated the release transient and could
    % lose tens of metres before cfg2 was even evaluated.
    [elevatorBaseSchedule, throttleBaseSchedule, usePreparationSchedule] = ...
        local_preparation_base_schedule(opts, trim);
    assignin("base", "airdropx_id_elevator_base_schedule", elevatorBaseSchedule);
    assignin("base", "airdropx_id_throttle_base_schedule", throttleBaseSchedule);

    % Define excitation dwell times in physical seconds, not raw JSBSim
    % samples.  v10 logged at about 120 Hz while build_iddata labeled every raw
    % sample as Ts=0.1 s; the old [3 12]/[5 20] sample holds therefore became
    % unrealistically fast 25-170 ms pulses.
    elevatorHoldN = max(1, round(double(opts.ElevatorHoldTimeRangeS(:)).' / double(simCfg.sim.dt_s)));
    throttleHoldN = max(1, round(double(opts.ThrottleHoldTimeRangeS(:)).' / double(simCfg.sim.dt_s)));
    profile = airdropx_auto_make_excitation("Ts", simCfg.sim.dt_s, "StopTimeS", opts.StopTimeS, ...
        "Seed", opts.Seed, "ElevatorTrim", trim.elevator_cmd, "ThrottleTrim", trim.throttle_cmd, ...
        "ElevatorAmplitude", opts.ElevatorAmplitude, "ThrottleAmplitude", opts.ThrottleAmplitude, ...
        "ElevatorHoldRange", elevatorHoldN, "ThrottleHoldRange", throttleHoldN, ...
        "ElevatorMin", opts.ElevatorMin, "ElevatorMax", opts.ElevatorMax, ...
        "ThrottleMin", opts.ThrottleMin, "ThrottleMax", opts.ThrottleMax);

    excitationStartS = max(0.0, double(opts.ExcitationStartS));
    preExcitationMask = double(profile.time_s) < excitationStartS;
    if logical(opts.DirectIdMode)
        % The model Sum blocks already add the base command. Therefore feed
        % only zero-centered perturbations into their excitation ports.
        % Keep the excitation exactly zero while cfg1..cfg4 are being prepared
        % and settling; otherwise payload-drop motion and ID excitation become
        % inseparable in the identified model.
        eDev = double(profile.elevator(:,2)) - double(trim.elevator_cmd);
        thDev = double(profile.throttle(:,2)) - double(trim.throttle_cmd);
        activeMask = ~preExcitationMask;
        eDev(preExcitationMask) = 0.0;
        thDev(preExcitationMask) = 0.0;
        % Re-center the ACTUAL post-settle excitation segment.  make_excitation
        % is generated over the whole simulation; zeroing the long pre-ID part
        % can otherwise leave a finite-record DC bias in the remaining segment.
        eDev(activeMask) = local_zero_mean_bounded(eDev(activeMask), double(opts.ElevatorAmplitude));
        thDev(activeMask) = local_zero_mean_bounded(thDev(activeMask), double(opts.ThrottleAmplitude));
        profile.elevator(:,2) = double(trim.elevator_cmd) + eDev;
        profile.throttle(:,2) = double(trim.throttle_cmd) + thDev;
        elevatorExcitation = [profile.time_s, eDev];
        throttleExcitation = [profile.time_s, thDev];
    else
        % Preserve legacy absolute-input behavior, but hold the trim command
        % until the requested excitation start.
        elevatorExcitation = profile.elevator;
        throttleExcitation = profile.throttle;
        elevatorExcitation(preExcitationMask,2) = double(trim.elevator_cmd);
        throttleExcitation(preExcitationMask,2) = double(trim.throttle_cmd);
    end
    assignin("base", "airdropx_mpc_elevator_excitation", elevatorExcitation);
    assignin("base", "airdropx_mpc_throttle_excitation", throttleExcitation);

    if logical(opts.DirectIdMode)
        directPatch = local_enable_direct_id_mode(char(modelName), usePreparationSchedule);
    end

    set_param(char(modelName), "StopTime", num2str(double(opts.StopTimeS), "%.15g"), ...
        "FixedStep", "dt", "SolverName", "FixedStepDiscrete", "SignalLogging", "on", "SignalLoggingName", "logsout");
    out = sim(char(modelName), "StopTime", num2str(double(opts.StopTimeS), "%.15g"), ...
        "FixedStep", num2str(simCfg.sim.dt_s, "%.15g"));

    local_restore_direct_id_mode(char(modelName), directPatch);
    set_param(char(modelName), "InitFcn", oldInitFcn);
    set_param(char(modelName), "Dirty", "off");
catch ME
    if bdIsLoaded(char(modelName))
        local_restore_direct_id_mode(char(modelName), directPatch);
        set_param(char(modelName), "InitFcn", oldInitFcn);
        set_param(char(modelName), "Dirty", "off");
    end
    rethrow(ME);
end

outputRoot = string(opts.OutputRoot);
if strlength(outputRoot) == 0
    outputRoot = string(fullfile(paths.matlabDir, "results", "mpc_auto_data", "cfg" + string(opts.ConfigId), string(opts.RunId)));
end
if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

T = local_timeseries_table(out.logsout, opts, trim, profile);
if logical(opts.DirectCfgViaAircraftXml) && ~logical(opts.PrepareByDrops)
    % Legacy MEX builds keep a separate bookkeeping mass/drop counter that
    % assumes cfg0 even when JSBSim physically loaded a cfg-specific XML.
    % Therefore do NOT use those legacy bookkeeping outputs as proof of the
    % physical airframe.  The temporary XML is verified when it is created;
    % after simulation, relabel only the OFFLINE metadata so filtering and
    % reports describe the physical model that was actually loaded.
    cfgOffline = round(double(opts.ConfigId));
    expectedMass = double(opts.ReferenceMassKg) - double(opts.CargoMassKg)*cfgOffline;
    cargoX = [4.826 5.131 5.436 5.740];
    emptyMass = double(opts.ReferenceMassKg)-4*double(opts.CargoMassKg);
    emptyCg = 5.279;
    remain=(cfgOffline+1):4;
    expectedMoment=emptyMass*emptyCg;
    if ~isempty(remain),expectedMoment=expectedMoment+double(opts.CargoMassKg)*sum(cargoX(remain));end
    expectedCg=expectedMoment/expectedMass;
    if ismember('drop_count',T.Properties.VariableNames),T.drop_count(:)=cfgOffline;end
    if ismember('mass_kg',T.Properties.VariableNames),T.mass_kg(:)=expectedMass;end
    if ismember('cg_x_m',T.Properties.VariableNames),T.cg_x_m(:)=expectedCg;end
end
if opts.KeepFixedConfigurationOnly
    exportStartS = double(opts.ExportStartS);
    if ~isfinite(exportStartS)
        exportStartS = double(opts.RecordStartS);
    end
    mask = T.time_s >= exportStartS & round(T.drop_count) == double(opts.ConfigId);
    T = T(mask, :);
end
csvPath = fullfile(outputRoot, "auto_id_timeseries.csv");
writetable(T, csvPath);

opObserved = local_observed_operating_point(T, trim, opts);
result = struct("output_root", outputRoot, "timeseries_csv", string(csvPath), "timeseries", T, ...
    "trim", trim, "operating_point", opObserved, "profile", profile, "out", out, ...
    "direct_id_mode", logical(opts.DirectIdMode), ...
    "aircraft_name_used", string(opts.AircraftName), ...
    "direct_cfg_via_aircraft_xml", logical(opts.DirectCfgViaAircraftXml));
fprintf("AirdropX auto MPC ID data exported:\n  %s\n", csvPath);
if logical(opts.DirectIdMode)
    fprintf("  Direct ID path active: trim constant + zero-centered excitation; legacy PD/Unit Delay base bypassed.\n");
end
end

function patch = local_enable_direct_id_mode(modelName, usePreparationSchedule)
if nargin < 2, usePreparationSchedule = false; end
patch = struct("applied", false, "legacy_lines_removed", false, ...
    "elevator_block", string(modelName) + "/MPC_ID_elevator_trim_direct", ...
    "throttle_block", string(modelName) + "/MPC_ID_throttle_trim_direct", ...
    "logged_ports", []);
required = [string(modelName) + "/MPC_ID_elevator_sum", string(modelName) + "/MPC_ID_throttle_sum", ...
    string(modelName) + "/Unit Delay1", string(modelName) + "/Unit Delay2"];
for i = 1:numel(required)
    if getSimulinkBlockHandle(char(required(i))) < 0
        error("AirdropX:AutoMPC:DirectIdPathMissingBlock", "DirectIdMode requires block: %s", required(i));
    end
end

try
    local_delete_block_if_present(patch.elevator_block);
    local_delete_block_if_present(patch.throttle_block);

    % Remove only the branches driving the ID Sum blocks. The old PD chain
    % remains intact elsewhere in the loaded model.
    try, delete_line(modelName, "Unit Delay1/1", "MPC_ID_elevator_sum/1"); catch, end
    try, delete_line(modelName, "Unit Delay2/1", "MPC_ID_throttle_sum/1"); catch, end
    patch.legacy_lines_removed = true;

    if logical(usePreparationSchedule)
        add_block("simulink/Sources/From Workspace", char(patch.elevator_block), ...
            "VariableName", "airdropx_id_elevator_base_schedule", ...
            "SampleTime", "-1", "Interpolate", "off", ...
            "OutputAfterFinalValue", "Holding final value", ...
            "Position", [150 65 250 105]);
        add_block("simulink/Sources/From Workspace", char(patch.throttle_block), ...
            "VariableName", "airdropx_id_throttle_base_schedule", ...
            "SampleTime", "-1", "Interpolate", "off", ...
            "OutputAfterFinalValue", "Holding final value", ...
            "Position", [150 175 250 215]);
    else
        add_block("simulink/Sources/Constant", char(patch.elevator_block), ...
            "Value", "airdropx_initial_elevator_delta", "SampleTime", "dt", ...
            "Position", [175 70 225 100]);
        add_block("simulink/Sources/Constant", char(patch.throttle_block), ...
            "Value", "airdropx_initial_throttle_cmd", "SampleTime", "dt", ...
            "Position", [175 180 225 210]);
    end
    add_line(modelName, "MPC_ID_elevator_trim_direct/1", "MPC_ID_elevator_sum/1", "autorouting", "on");
    add_line(modelName, "MPC_ID_throttle_trim_direct/1", "MPC_ID_throttle_sum/1", "autorouting", "on");
    patch.logged_ports = [ ...
        local_enable_output_logging(modelName, "MPC_ID_elevator_saturation", "mpc_elevator_to_plant"), ...
        local_enable_output_logging(modelName, "MPC_ID_throttle_saturation", "mpc_throttle_to_plant")];
    patch.applied = true;
catch ME
    local_restore_direct_id_mode(modelName, patch);
    rethrow(ME);
end
end

function local_restore_direct_id_mode(modelName, patch)
if ~isstruct(patch) || ~bdIsLoaded(modelName)
    return;
end
if isfield(patch, "logged_ports") && ~isempty(patch.logged_ports)
    local_restore_output_logging(modelName, patch.logged_ports);
end
try, delete_line(modelName, "MPC_ID_elevator_trim_direct/1", "MPC_ID_elevator_sum/1"); catch, end
try, delete_line(modelName, "MPC_ID_throttle_trim_direct/1", "MPC_ID_throttle_sum/1"); catch, end
if isfield(patch, "elevator_block"), local_delete_block_if_present(patch.elevator_block); end
if isfield(patch, "throttle_block"), local_delete_block_if_present(patch.throttle_block); end
if isfield(patch, "legacy_lines_removed") && patch.legacy_lines_removed
    try, add_line(modelName, "Unit Delay1/1", "MPC_ID_elevator_sum/1", "autorouting", "on"); catch, end
    try, add_line(modelName, "Unit Delay2/1", "MPC_ID_throttle_sum/1", "autorouting", "on"); catch, end
end
end

function state = local_enable_output_logging(modelName, blockName, signalName)
blockPath = string(modelName) + "/" + string(blockName);
if getSimulinkBlockHandle(char(blockPath)) < 0
    error("AirdropX:AutoMPC:MissingLoggedInputBlock", ...
        "DirectIdMode cannot log missing block output: %s", blockPath);
end
ph = get_param(char(blockPath), "PortHandles");
if isempty(ph.Outport)
    error("AirdropX:AutoMPC:MissingLoggedInputPort", ...
        "DirectIdMode cannot log block with no output port: %s", blockPath);
end
port = ph.Outport(1);
state = struct("block", blockPath, ...
    "DataLogging", string(get_param(port, "DataLogging")), ...
    "DataLoggingNameMode", string(get_param(port, "DataLoggingNameMode")), ...
    "DataLoggingName", string(get_param(port, "DataLoggingName")));
set_param(port, "DataLogging", "on", ...
    "DataLoggingNameMode", "Custom", ...
    "DataLoggingName", char(signalName));
end

function local_restore_output_logging(modelName, states)
for i = 1:numel(states)
    blockPath = states(i).block;
    if getSimulinkBlockHandle(char(blockPath)) < 0
        continue;
    end
    try
        ph = get_param(char(blockPath), "PortHandles");
        if isempty(ph.Outport), continue; end
        set_param(ph.Outport(1), ...
            "DataLogging", char(states(i).DataLogging), ...
            "DataLoggingNameMode", char(states(i).DataLoggingNameMode), ...
            "DataLoggingName", char(states(i).DataLoggingName));
    catch ME
        warning("AirdropX:AutoMPC:RestoreSignalLoggingFailed", ...
            "Could not restore signal logging for %s in %s: %s", blockPath, modelName, ME.message);
    end
end
end

function local_delete_block_if_present(blockPath)
try
    if getSimulinkBlockHandle(char(blockPath)) >= 0
        delete_block(char(blockPath));
    end
catch
end
end

function y = local_zero_mean_bounded(y, amplitude)
y = double(y(:));
if isempty(y), return; end
y = y - mean(y, "omitnan");
peak = max(abs(y), [], "omitnan");
if isfinite(peak) && peak > amplitude && peak > 0
    y = y * (amplitude / peak);
end
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
paths = struct("projectRoot", char(projectRoot), "matlabDir", char(matlabDir), ...
    "mpcDir", char(fullfile(matlabDir, "mpc")), "autoDir", char(fullfile(matlabDir, "mpc_auto")), ...
    "sfuncDir", char(fullfile(matlabDir, "sfunc_jsbsim")));
end

function T = local_timeseries_table(logs, opts, trim, profile)
signals = ["altitude_m"; "airspeed_mps"; "pitch_deg"; "vz_up_mps"; "q_dps"; ...
    "elevator_delta"; "elevator_cmd_norm"; "throttle_cmd"; "throttle_norm"; ...
    "mpc_elevator_to_plant"; "mpc_throttle_to_plant"; ...
    "drop_count"; "mass_kg"; "cg_x_m"; "pos_n_m"; "pos_e_m"; "heading_deg"; "wind_n_mps"; "wind_e_mps"];
[tRef, ~] = local_signal(logs, "altitude_m");
if isempty(tRef)
    error("AirdropX:AutoMPC:MissingLog", "logsout does not contain altitude_m.");
end
runId = string(opts.RunId);
if strlength(runId) == 0
    runId = "cfg" + string(opts.ConfigId) + "_run" + string(opts.Seed);
end
T = table(tRef(:), repmat(runId, numel(tRef), 1), repmat(double(opts.ConfigId), numel(tRef), 1), ...
    'VariableNames', {'time_s', 'run_id', 'config_id'});
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

T.requested_elevator_trim = double(trim.elevator_cmd) * ones(height(T), 1);
T.requested_throttle_trim = double(trim.throttle_cmd) * ones(height(T), 1);
T.requested_pitch_trim_deg = double(trim.pitch_deg) * ones(height(T), 1);
T.elevator_excitation = local_profile_interp([profile.time_s, double(profile.elevator(:,2)) - double(trim.elevator_cmd)], tRef);
T.throttle_excitation = local_profile_interp([profile.time_s, double(profile.throttle(:,2)) - double(trim.throttle_cmd)], tRef);
T.requested_elevator_cmd = local_profile_interp(profile.elevator, tRef);
T.requested_throttle_cmd = local_profile_interp(profile.throttle, tRef);

% Keep legacy controller-output signals for diagnostics. For identification,
% expose the closest available signal to the command actually sent to JSBSim.
T.elevator_controller_cmd = T.elevator_delta;
T.throttle_controller_cmd = T.throttle_cmd;

actualElevator = T.mpc_elevator_to_plant;
if all(~isfinite(actualElevator)) && logical(opts.DirectIdMode)
    actualElevator = T.requested_elevator_cmd;
elseif all(~isfinite(actualElevator)) && any(isfinite(T.elevator_cmd_norm))
    actualElevator = T.elevator_cmd_norm;
elseif all(~isfinite(actualElevator))
    actualElevator = T.elevator_delta;
end

actualThrottle = T.mpc_throttle_to_plant;
if all(~isfinite(actualThrottle)) && logical(opts.DirectIdMode)
    actualThrottle = T.requested_throttle_cmd;
elseif all(~isfinite(actualThrottle)) && any(isfinite(T.throttle_norm))
    actualThrottle = T.throttle_norm;
elseif all(~isfinite(actualThrottle))
    actualThrottle = T.throttle_cmd;
end

% Keep the two elevator coordinates explicit.
%   elevator_external_delta_actual: command sent INTO the legacy MEX.
%   elevator_physical_actual: total elevator actually applied inside JSBSim,
%     i.e. hidden trimElevator_ + external delta, reported by the MEX output
%     elevator_cmd_norm (lastElevatorCmd_).
T.elevator_external_delta_actual = double(actualElevator);
physicalElevator = T.elevator_cmd_norm;
if all(~isfinite(physicalElevator))
    physicalElevator = T.elevator_external_delta_actual;
end
T.elevator_physical_actual = double(physicalElevator);

T.throttle_cmd_actual = double(actualThrottle);
physicalThrottle = T.throttle_norm;
if all(~isfinite(physicalThrottle))
    physicalThrottle = T.throttle_cmd_actual;
end
T.throttle_physical_actual = double(physicalThrottle);

% Preserve legacy column semantics for old auto-MPC code: elevator_cmd_actual
% remains the external command to the MEX. Physics-MPC v1.5 explicitly reads
% elevator_physical_actual instead.
T.elevator_cmd_actual = T.elevator_external_delta_actual;
T.elevator_cmd = T.elevator_cmd_actual;
T.throttle_cmd = T.throttle_cmd_actual;
T.input_elevator_error = T.elevator_external_delta_actual - T.requested_elevator_cmd;
T.input_throttle_error = T.throttle_cmd_actual - T.requested_throttle_cmd;

T.trim_altitude_m = double(trim.altitude_m) * ones(height(T), 1);
T.trim_airspeed_mps = double(trim.airspeed_mps) * ones(height(T), 1);
T.trim_pitch_deg = double(trim.pitch_deg) * ones(height(T), 1);
T.trim_elevator_cmd = double(trim.elevator_cmd) * ones(height(T), 1);
T.trim_throttle_cmd = double(trim.throttle_cmd) * ones(height(T), 1);
end

function values = local_profile_interp(profileMatrix, tRef)
if isempty(profileMatrix)
    values = NaN(numel(tRef), 1);
    return;
end
values = interp1(double(profileMatrix(:,1)), double(profileMatrix(:,2)), double(tRef(:)), "previous", "extrap");
end

function op = local_observed_operating_point(T, trim, opts)
op = trim;
if isempty(T) || height(T) == 0
    return;
end
time = double(T.time_s);
endTime = max(time, [], "omitnan");
startTime = max(double(opts.RecordStartS), endTime - double(opts.OperatingPointWindowS));
mask = isfinite(time) & time >= startTime;
if nnz(mask) < 3
    mask = isfinite(time);
end
op.config_id = double(opts.ConfigId);
op.altitude_m = local_median(T.altitude_m(mask), trim.altitude_m);
op.airspeed_mps = local_median(T.airspeed_mps(mask), trim.airspeed_mps);
op.pitch_deg = local_median(T.pitch_deg(mask), trim.pitch_deg);
op.vz_up_mps = local_median(T.vz_up_mps(mask), local_field(trim, "vz_up_mps", 0.0));
op.q_dps = local_median(T.q_dps(mask), local_field(trim, "q_dps", 0.0));
op.elevator_cmd = local_median(T.elevator_cmd(mask), trim.elevator_cmd);
op.throttle_cmd = local_median(T.throttle_cmd(mask), trim.throttle_cmd);
end

function value = local_median(x, fallback)
x = double(x(:));
x = x(isfinite(x));
if isempty(x), value = double(fallback); else, value = median(x); end
end

function value = local_field(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isfinite(double(s.(name)))
    value = double(s.(name));
else
    value = double(fallback);
end
end

function [t, y] = local_signal_with_alias(logs, name)
[t, y] = local_signal(logs, name);
if ~isempty(t), return; end
switch string(name)
    case "elevator_delta"
        [t, y] = local_signal(logs, "elevator_cmd_norm");
    case "throttle_cmd"
        [t, y] = local_signal(logs, "throttle_norm");
    case "q_dps"
        [t, y] = local_signal(logs, "pitch_rate_dps");
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
    if isempty(el), return; end
    values = el.Values;
    t = double(values.Time(:));
    y = double(values.Data(:));
catch
end
end

function [eSchedule, thSchedule, enabled] = local_preparation_base_schedule(opts, targetTrim)
% Build a zero-order-hold base-command schedule for cfg0 -> ... -> cfgN.
% Only the target cfg uses targetTrim; earlier configurations use the
% supplied PreparationTrimBank.  This preserves the correct actuator
% coordinate while the old MEX reaches cfgN through real payload drops.
enabled = false;
eSchedule = [0.0, double(targetTrim.elevator_cmd)];
thSchedule = [0.0, double(targetTrim.throttle_cmd)];

cfgId = max(0, round(double(opts.ConfigId)));
if ~logical(opts.PrepareByDrops) || cfgId <= 0 || isempty(opts.PreparationTrimBank) || ~logical(opts.UsePreparationTrimSchedule)
    return;
end

prep = opts.PreparationTrimBank;
if numel(prep) < cfgId
    warning("AirdropX:AutoMPC:PreparationTrimBankShort", ...
        "PreparationTrimBank has only %d entries for cfg%d; using legacy constant target command.", ...
        numel(prep), cfgId);
    return;
end

times = zeros(cfgId + 1, 1);
eVals = zeros(cfgId + 1, 1);
thVals = zeros(cfgId + 1, 1);
for k = 0:cfgId
    if k < cfgId
        one = prep(k + 1);
    else
        one = targetTrim;
    end
    eVals(k + 1) = local_numeric_struct_field(one, "elevator_cmd", double(targetTrim.elevator_cmd));
    thVals(k + 1) = local_numeric_struct_field(one, "throttle_cmd", double(targetTrim.throttle_cmd));
    if k == 0
        times(k + 1) = 0.0;
    else
        times(k + 1) = double(opts.PrepDropStartS) + ...
            double(opts.PrepDropIntervalS) * max(0, k - 1);
    end
end

% From Workspace with interpolation disabled implements an exact
% zero-order-hold switch at each payload-drop time.
eSchedule = [times, eVals];
thSchedule = [times, thVals];
enabled = true;
end

function value = local_numeric_struct_field(s, name, fallback)
value = double(fallback);
try
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isfinite(double(s.(name)))
        value = double(s.(name));
    end
catch
end
end


function local_prepare_parallel_filegen()
persistent preparedKey
try
    task = getCurrentTask();
catch
    task = [];
end
if isempty(task)
    return;
end
wid = double(task.ID);
baseRoot = string(getenv("AIRDROPX_FILEGEN_ROOT"));
if strlength(baseRoot)==0, baseRoot=string(fullfile(tempdir,"AXC")); end
key = baseRoot + "|airdropx_id_worker|" + string(wid);
if ~isempty(preparedKey) && string(preparedKey) == key
    return;
end
try
    root = fullfile(baseRoot, sprintf("id%d", wid));
    cacheDir = fullfile(root, "c");
    codegenDir = fullfile(root, "g");
    if ~isfolder(cacheDir), mkdir(cacheDir); end
    if ~isfolder(codegenDir), mkdir(codegenDir); end
    Simulink.fileGenControl("set", "CacheFolder", cacheDir, "CodeGenFolder", codegenDir, "createDir", true);
    preparedKey = key;
catch ME
    warning("AirdropX:AutoMPC:WorkerFileGen", ...
        "Could not set private Simulink file-generation folders on worker %d: %s", wid, ME.message);
end
end

function icName = local_write_isolated_initial_condition(projectRoot, opts)
aircraft = char(string(opts.AircraftName));
root = char(string(projectRoot));
templatePath = fullfile(root, "aircraft", aircraft, "reset_20m.xml");
generatedDir = fullfile(root, "aircraft", aircraft, "generated");
if ~isfile(templatePath)
    error("AirdropX:AutoMPC:MissingIcTemplate", "Initial-condition template not found: %s", templatePath);
end
if ~isfolder(generatedDir), mkdir(generatedDir); end
base = tempname(generatedDir);
icName = string(base) + ".xml";

xmlText = fileread(templatePath);
airspeedMps = double(opts.InitialAirspeedMps);
pitchDeg = double(opts.InitialPitchDeg);
flightPathDeg = double(opts.InitialFlightPathDeg);
altitudeM = double(opts.InitialAltitudeM);
headingDeg = double(opts.InitialHeadingDeg);
if ~isfinite(airspeedMps), airspeedMps = 45.0; end
if ~isfinite(pitchDeg), pitchDeg = 4.0; end
if ~isfinite(flightPathDeg), flightPathDeg = 0.0; end
if ~isfinite(altitudeM), altitudeM = 20.0; end
if ~isfinite(headingDeg), headingDeg = 0.0; end

alphaDeg = pitchDeg - flightPathDeg;
ubodyMps = airspeedMps * cosd(alphaDeg);
wbodyMps = airspeedMps * sind(alphaDeg);
xmlText = local_xml_replace(xmlText, '<ubody\s+unit="M/SEC">[^<]*</ubody>', ...
    sprintf('<ubody unit="M/SEC">%.10g</ubody>', ubodyMps), "ubody", templatePath);
xmlText = local_xml_replace(xmlText, '<wbody\s+unit="M/SEC">[^<]*</wbody>', ...
    sprintf('<wbody unit="M/SEC">%.10g</wbody>', wbodyMps), "wbody", templatePath);
xmlText = local_xml_replace(xmlText, '<theta\s+unit="DEG">[^<]*</theta>', ...
    sprintf('<theta unit="DEG">%.10g</theta>', pitchDeg), "theta", templatePath);
xmlText = local_xml_replace(xmlText, '<altitude\s+unit="M">[^<]*</altitude>', ...
    sprintf('<altitude unit="M">%.10g</altitude>', altitudeM), "altitude", templatePath);
xmlText = local_xml_replace(xmlText, '<psi\s+unit="DEG">[^<]*</psi>', ...
    sprintf('<psi unit="DEG">%.10g</psi>', headingDeg), "psi", templatePath);

fid = fopen(char(icName), "w", "n", "UTF-8");
if fid < 0
    error("AirdropX:AutoMPC:IcWriteFailed", "Cannot write isolated initial condition: %s", icName);
end
c = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, "%s", xmlText);
end

function out = local_xml_replace(in, expr, replacement, fieldName, templatePath)
if isempty(regexp(in, expr, "once"))
    error("AirdropX:AutoMPC:IcTemplateFieldMissing", ...
        "Initial-condition template %s is missing %s.", templatePath, fieldName);
end
out = regexprep(in, expr, replacement, "once");
end

function local_delete_file_quiet(pathValue)
try
    p = char(string(pathValue));
    if ~isempty(p) && isfile(p), delete(p); end
catch
end
end

function [variantName,variantDir] = local_make_cfg_aircraft_variant(projectRoot,baseAircraft,cfg)
cfg=max(0,min(4,round(double(cfg))));
srcDir=fullfile(char(string(projectRoot)),'aircraft',char(string(baseAircraft)));
if ~isfolder(srcDir),error('AirdropX:PhysicsMPC:AircraftVariantSourceMissing','Aircraft folder missing: %s',srcDir);end
[~,token]=fileparts(tempname);
variantName=sprintf('%s_PHYS_cfg%d_%s',char(string(baseAircraft)),cfg,token);
variantDir=fullfile(char(string(projectRoot)),'aircraft',variantName);
mkdir(variantDir);
try
    D=dir(srcDir);
    for kk=1:numel(D)
        if D(kk).isdir || startsWith(D(kk).name,'.'),continue;end
        copyfile(fullfile(srcDir,D(kk).name),fullfile(variantDir,D(kk).name));
    end
    srcXml=fullfile(variantDir,[char(string(baseAircraft)) '.xml']);
    dstXml=fullfile(variantDir,[variantName '.xml']);
    if ~isfile(srcXml),error('AirdropX:PhysicsMPC:AircraftVariantXmlMissing','Base aircraft XML missing after copy: %s',srcXml);end
    movefile(srcXml,dstXml,'f');
    xmlText=fileread(dstXml);
    for c=1:cfg
        cargo=sprintf('CARGO_%d',c);
        expr=['(<pointmass\s+name="' cargo '"[\s\S]*?<weight\s+unit="LBS">)\s*[-+0-9.eE]+\s*(</weight>)'];
        if isempty(regexp(xmlText,expr,'once'))
            error('AirdropX:PhysicsMPC:AircraftVariantCargoMissing','Cannot find %s point-mass weight in %s.',cargo,dstXml);
        end
        xmlText=regexprep(xmlText,expr,'$1 0 $2','once');
    end
    fid=fopen(dstXml,'w','n','UTF-8');
    if fid<0,error('AirdropX:PhysicsMPC:AircraftVariantWriteFailed','Cannot write %s.',dstXml);end
    cc=onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s',xmlText);
catch ME
    local_delete_dir_quiet(variantDir);
    rethrow(ME);
end
end
function local_delete_dir_quiet(pathValue)
try
    p=char(string(pathValue));
    if ~isempty(p) && isfolder(p),rmdir(p,'s');end
catch
end
end

function old=local_capture_base_drop_init()
old=struct('enable_exists',false,'enable',0,'count_exists',false,'count',0);
try,old.enable_exists=evalin('base','exist(''airdropx_enable_initial_drop_count'',''var'')')>0;if old.enable_exists,old.enable=evalin('base','airdropx_enable_initial_drop_count');end,catch,end
try,old.count_exists=evalin('base','exist(''airdropx_initial_drop_count'',''var'')')>0;if old.count_exists,old.count=evalin('base','airdropx_initial_drop_count');end,catch,end
end
function local_restore_base_drop_init(old)
try,if old.enable_exists,assignin('base','airdropx_enable_initial_drop_count',old.enable);else,evalin('base','clear(''airdropx_enable_initial_drop_count'')');end,catch,end
try,if old.count_exists,assignin('base','airdropx_initial_drop_count',old.count);else,evalin('base','clear(''airdropx_initial_drop_count'')');end,catch,end
end

function opts = local_options(varargin)
opts.ProjectRoot = "";
opts.Model = "airdropx_mpc_id";
opts.AircraftName = "MQ9_Reaper";
% When true, create a unique generated IC XML for this run. This is required
% for process-pool Physics-MPC builds; otherwise workers overwrite the same
% reset_20m_runtime.xml and the physical baseline is nondeterministic.
opts.IsolateGeneratedIc = false;
% Physics-MPC v1.3.1 fallback that needs no S-function rebuild.  A temporary
% aircraft XML directly represents cfg0..cfg4 mass/CG for offline trim/ID.
opts.DirectCfgViaAircraftXml = false;
opts.OutputRoot = "";
opts.RunId = "";
opts.ConfigId = 0;
% v1.3 offline physics mode can start JSBSim directly at cfg0..cfg4.
% This is only for trim/linearization; real mission validation still starts
% at cfg0 and uses actual drop commands.
opts.InitialDropCount = 0;
opts.PrepareByDrops = true;
opts.Trim = [];
opts.StopTimeS = 30.0;
opts.RecordStartS = 8.0;
% Delay actuator excitation until the requested fixed configuration has been
% reached and allowed to settle.  Zero preserves legacy behavior.
opts.ExcitationStartS = 0.0;
% ExportStartS controls how much trajectory is returned/written.  When NaN,
% preserve the legacy behavior and export only from RecordStartS.  Trim
% search sets this to zero so the optimizer can see the release transient.
opts.ExportStartS = NaN;
opts.PrepDropStartS = 1.0;
opts.PrepDropIntervalS = 0.5;
opts.KeepFixedConfigurationOnly = true;
opts.DirectIdMode = true;
% v20 sequential preparation support.  When supplied for cfg1+, the direct
% base command follows cfg0/cfg1/... trims until the requested cfg is reached,
% instead of applying the target cfg command from t=0.
opts.PreparationTrimBank = [];
opts.UsePreparationTrimSchedule = true;
opts.OperatingPointWindowS = 3.0;
opts.Seed = 1;
opts.InitialAirspeedMps = 50.0;
opts.InitialAltitudeM = 20.0;
opts.InitialPitchDeg = 4.0;
opts.InitialFlightPathDeg = 0.0;
opts.InitialHeadingDeg = 0.0;
opts.TargetAltitudeM = 20.0;
opts.TargetAirspeedMps = 50.0;
opts.ReferenceMassKg = 3423.0;
opts.CargoMassKg = 300.0;
opts.ElevatorAmplitude = 0.03;
opts.ThrottleAmplitude = 0.06;
% Physical random-hold dwell times.  They are converted to raw simulation
% samples using simCfg.sim.dt_s before calling make_excitation.
opts.ElevatorHoldTimeRangeS = [0.40 1.60];
opts.ThrottleHoldTimeRangeS = [0.80 3.00];
opts.ElevatorMin = -0.75;
opts.ElevatorMax = 0.45;
opts.ThrottleMin = 0.35;
opts.ThrottleMax = 0.88;
if mod(numel(varargin), 2) ~= 0, error("Options must be name-value pairs."); end
for i = 1:2:numel(varargin)
    name = string(varargin{i});
    if ~isfield(opts, name), error("Unknown option: %s", name); end
    opts.(name) = varargin{i + 1};
end
end
