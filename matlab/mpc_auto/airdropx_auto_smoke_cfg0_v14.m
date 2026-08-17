function result = airdropx_auto_smoke_cfg0_v14(varargin)
%AIRDROPX_AUTO_SMOKE_CFG0_V14 Deterministic cfg0 tests after v13 soft fail.
%
% No trim/ID/n4sid retraining and no bayesopt.
%
% A: Np=8,  no output disturbance model.
% B: Np=8,  default output disturbance model enabled.
% C: Np=12, default output disturbance model enabled.
%
% v14 controller removes altitude from the trust-exit gate and keeps MPC
% LastMove coherent with the authority-scaled move actually sent to JSBSim.

opts = local_options(varargin{:});
paths = local_paths(opts.ProjectRoot);
addpath(paths.matlabDir);
addpath(paths.mpcDir);
addpath(paths.autoDir);
addpath(paths.sfuncDir);

S = load(opts.IdentifiedMat, "result");
identified = S.result;
trim0 = identified.trim_bank(1);

outRoot = string(opts.OutputRoot);
if strlength(outRoot) == 0
    outRoot = string(fullfile(paths.matlabDir, "results", ...
        "mpc_auto_smoke_cfg0_v14_" + string(datetime("now","Format","yyyyMMdd_HHmmss"))));
end
if ~isfolder(outRoot), mkdir(outRoot); end

cases = [
    struct("name","A_Np8_noOD",   "Np",8,  "Nc",3, "disableOD",true,  "authority",opts.AuthorityScale)
    struct("name","B_Np8_withOD", "Np",8,  "Nc",3, "disableOD",false, "authority",opts.AuthorityScale)
    struct("name","C_Np12_withOD","Np",12, "Nc",3, "disableOD",false, "authority",opts.AuthorityScale)
    ];

% Build first bank to recover the same physical elevator nominal used in v13.
firstBankMat = fullfile(outRoot, "bank_A.mat");
bank = local_build_bank(identified, firstBankMat, cases(1), opts);
physicalNom = double(bank.mpc_meta.physical_elevator_nominals(1));

% Hidden trim calibration for the exact 20 m initial condition.
cal = airdropx_auto_run_id_experiment( ...
    "ProjectRoot", paths.projectRoot, ...
    "OutputRoot", fullfile(outRoot,"hidden_trim_calibration"), ...
    "RunId", "cfg0_hidden_trim_calibration_v14", ...
    "ConfigId", 0, "Trim", trim0, ...
    "StopTimeS", opts.CalibrationStopTimeS, ...
    "RecordStartS", 0.0, "ExportStartS", 0.0, ...
    "ExcitationStartS", 100.0, ...
    "ElevatorAmplitude", 0.0, "ThrottleAmplitude", 0.0, ...
    "DirectIdMode", true, "KeepFixedConfigurationOnly", true, ...
    "InitialAltitudeM", opts.InitialAltitudeM, ...
    "InitialAirspeedMps", opts.InitialAirspeedMps, ...
    "InitialPitchDeg", local_default_if_nan(opts.InitialPitchDeg, trim0.pitch_deg), ...
    "InitialFlightPathDeg", opts.InitialFlightPathDeg, ...
    "TargetAltitudeM", opts.TargetAltitudeM, ...
    "TargetAirspeedMps", opts.TargetAirspeedMps);

Tc = cal.timeseries;
external = local_col(Tc,"requested_elevator_cmd");
physical = local_col(Tc,"elevator_cmd_norm");
mask = isfinite(external) & isfinite(physical) & double(Tc.time_s) <= opts.CalibrationUseUntilS;
if nnz(mask) < 3
    error("AirdropX:AutoMPC:HiddenTrimCalibration","Not enough calibration samples.");
end
hiddenTrim = median(physical(mask)-external(mask),"omitnan");
initialDelta = physicalNom-hiddenTrim;

writetable(table(physicalNom,hiddenTrim,initialDelta,double(trim0.throttle_cmd), ...
    'VariableNames',{'physical_elevator_nominal','hidden_elevator_trim', ...
    'initial_external_delta','throttle_nominal'}), ...
    fullfile(outRoot,"hidden_trim_summary.csv"));

% Bridge-only preflight remains mandatory.
pre = airdropx_auto_run_closed_loop( ...
    "ProjectRoot",paths.projectRoot,"MpcBankMat",firstBankMat, ...
    "OutputRoot",fullfile(outRoot,"bridge_only_preflight"), ...
    "CaseId","cfg0_bridge_only_preflight_v14", ...
    "StopTimeS",opts.PreflightStopTimeS,"FixedConfigId",0, ...
    "FixedDropTotal",0,"FixedDropStartS",opts.PreflightStopTimeS+100, ...
    "InitialAltitudeM",opts.InitialAltitudeM, ...
    "InitialAirspeedMps",opts.InitialAirspeedMps, ...
    "InitialPitchDeg",local_default_if_nan(opts.InitialPitchDeg,trim0.pitch_deg), ...
    "InitialFlightPathDeg",opts.InitialFlightPathDeg, ...
    "InitialElevatorDelta",initialDelta,"InitialThrottleCmd",trim0.throttle_cmd, ...
    "HiddenElevatorTrim",hiddenTrim, ...
    "MpcEnableTimeS",opts.PreflightStopTimeS+100,"MpcAuthorityScale",0, ...
    "TargetAltitudeM",opts.TargetAltitudeM, ...
    "TargetAirspeedMps",opts.TargetAirspeedMps, ...
    "TargetPitchDeg",trim0.pitch_deg,"UseTrimPitchReference",1);

preRow = local_metrics(pre.timeseries,physicalNom,trim0.throttle_cmd,opts);
writetable(preRow,fullfile(outRoot,"bridge_only_preflight_summary.csv"));
if preRow.hard_fail || preRow.max_bridge_elevator_error > opts.MaxBridgeError || ...
        preRow.max_bridge_throttle_error > opts.MaxBridgeError
    error("AirdropX:AutoMPC:BridgePreflightFailed", ...
        "v14 bridge-only preflight failed. Stop before MPC.");
end

rows = table();
bestScore = Inf;
bestName = "";
for i = 1:numel(cases)
    c = cases(i);
    if i == 1
        bankMat = firstBankMat;
    else
        bankMat = fullfile(outRoot, "bank_" + string(c.name) + ".mat");
        local_build_bank(identified,bankMat,c,opts);
    end

    caseDir = fullfile(outRoot,string(c.name));
    fprintf("\n[V14] %s: Np=%d Nc=%d authority=%.2f outputDist=%d\n", ...
        c.name,c.Np,c.Nc,c.authority,~c.disableOD);

    simResult = airdropx_auto_run_closed_loop( ...
        "ProjectRoot",paths.projectRoot,"MpcBankMat",bankMat, ...
        "OutputRoot",caseDir,"CaseId",string(c.name), ...
        "StopTimeS",opts.StopTimeS,"FixedConfigId",0, ...
        "FixedDropTotal",0,"FixedDropStartS",opts.StopTimeS+100, ...
        "InitialAltitudeM",opts.InitialAltitudeM, ...
        "InitialAirspeedMps",opts.InitialAirspeedMps, ...
        "InitialPitchDeg",local_default_if_nan(opts.InitialPitchDeg,trim0.pitch_deg), ...
        "InitialFlightPathDeg",opts.InitialFlightPathDeg, ...
        "InitialElevatorDelta",initialDelta,"InitialThrottleCmd",trim0.throttle_cmd, ...
        "HiddenElevatorTrim",hiddenTrim, ...
        "MpcEnableTimeS",opts.MpcEnableTimeS, ...
        "MpcAuthorityScale",c.authority, ...
        "ElevatorDevStepLimit",opts.ElevatorDeviationRateLimit, ...
        "ThrottleDevStepLimit",opts.ThrottleDeviationRateLimit, ...
        "TrustAltitudeM",opts.LegacyTrustAltitudeM, ...
        "TrustAirspeedMps",opts.TrustAirspeedMps, ...
        "TrustPitchDeg",opts.TrustPitchDeg, ...
        "TrustVzMps",opts.TrustVzMps, ...
        "TrustQDps",opts.TrustQDps, ...
        "TargetAltitudeM",opts.TargetAltitudeM, ...
        "TargetAirspeedMps",opts.TargetAirspeedMps, ...
        "TargetPitchDeg",trim0.pitch_deg,"UseTrimPitchReference",1);

    m = local_metrics(simResult.timeseries,physicalNom,trim0.throttle_cmd,opts);
    m = addvars(m,string(c.name),double(c.Np),double(c.Nc),double(c.authority), ...
        logical(~c.disableOD),'Before',1, ...
        'NewVariableNames',{'candidate','Np','Nc','authority','output_disturbance_enabled'});
    rows = [rows; m]; %#ok<AGROW>
    writetable(rows,fullfile(outRoot,"v14_deterministic_smoke_summary.csv"));
    local_plot(simResult.timeseries,physicalNom,trim0.throttle_cmd, ...
        fullfile(caseDir,"v14_curves.png"),string(c.name));

    if ~m.hard_fail && m.rank_score < bestScore
        bestScore = m.rank_score;
        bestName = string(c.name);
    end

    if m.hard_fail
        fprintf("[V14] HARD FAIL in %s -> stop remaining tests.\n",c.name);
        break;
    elseif m.formal_pass
        fprintf("[V14] FORMAL PASS in %s -> stop; do not widen variables.\n",c.name);
        break;
    else
        fprintf("[V14] soft: hRMS=%.3f drift=%.3f tailVz=%.3f max dE=%.4f max dT=%.4f\n", ...
            m.steady_h_rms_m,m.steady_h_drift_m,m.tail_vz_mps, ...
            m.max_physical_elevator_deviation,m.max_throttle_deviation);
    end
end

result = struct();
result.output_root = outRoot;
result.hidden_trim = hiddenTrim;
result.physical_elevator_nominal = physicalNom;
result.rows = rows;
result.best_candidate = bestName;
save(fullfile(outRoot,"v14_smoke_result.mat"),"result","opts","cases");
fprintf("\n[V14] Best deterministic candidate: %s\n",bestName);
end

function bank = local_build_bank(identified,bankMat,c,opts)
bank = airdropx_auto_build_mpc_bank( ...
    "Identified",identified,"OutputMat",bankMat, ...
    "PredictionHorizon",c.Np,"ControlHorizon",c.Nc, ...
    "InputCoordinateMode","deviation_physical", ...
    "PhysicalElevatorNominals",opts.PhysicalElevatorNominals, ...
    "ElevatorDeviationLimit",opts.ElevatorDeviationLimit, ...
    "ThrottleDeviationLimit",opts.ThrottleDeviationLimit, ...
    "ElevatorDeviationRateLimit",opts.ElevatorDeviationRateLimit, ...
    "ThrottleDeviationRateLimit",opts.ThrottleDeviationRateLimit, ...
    "OutputWeights",opts.OutputWeights,"MVWeights",opts.MVWeights, ...
    "MVRateWeights",opts.MVRateWeights, ...
    "DisableOutputDisturbanceModel",c.disableOD);
end

function M = local_metrics(T,eNom,tNom,opts)
t=double(T.time_s(:)); h=double(T.altitude_m(:)); V=double(T.airspeed_mps(:));
vz=double(T.vz_up_mps(:)); q=double(T.q_dps(:)); pitch=double(T.pitch_deg(:));
e=double(T.elevator_cmd_norm(:)); th=double(T.throttle_norm(:));
be=double(T.bridge_elevator_error(:)); bt=double(T.bridge_throttle_error(:));

steady=t>=opts.ScoreStartTimeS;
if nnz(steady)<10, steady=isfinite(t); end
tail=t>=max(max(t)-opts.TailWindowS,0);
if nnz(tail)<10, tail=steady; end
idx=find(steady);
nEdge=max(3,round(0.15*numel(idx)));
headIdx=idx(1:min(nEdge,numel(idx)));
tailIdx=idx(max(1,numel(idx)-nEdge+1):numel(idx));

hRms=local_rms(h(steady)-opts.TargetAltitudeM);
hMax=max(abs(h(steady)-opts.TargetAltitudeM),[],"omitnan");
hDrift=median(h(tailIdx),"omitnan")-median(h(headIdx),"omitnan");
vaRms=local_rms(V(steady)-opts.TargetAirspeedMps);
vzRms=local_rms(vz(steady)); qRms=local_rms(q(steady));
tailVz=median(vz(tail),"omitnan"); tailQ=median(q(tail),"omitnan");
tailHErr=median(h(tail)-opts.TargetAltitudeM,"omitnan");
maxED=max(abs(e-eNom),[],"omitnan"); maxTD=max(abs(th-tNom),[],"omitnan");
maxBE=max(abs(be),[],"omitnan"); maxBT=max(abs(bt),[],"omitnan");
minH=min(h,[],"omitnan"); maxPitch=max(abs(pitch),[],"omitnan");

hard=~isfinite(minH) || minH<opts.HardFloorAltitudeM || ...
    qRms>opts.HardMaxQRmsDps || maxPitch>opts.HardMaxAbsPitchDeg || ...
    maxBE>opts.MaxBridgeError || maxBT>opts.MaxBridgeError;
formal=~hard && hRms<=opts.PassAltitudeRmsM && hMax<=opts.PassAltitudeMaxM && ...
    abs(hDrift)<=opts.PassAltitudeDriftM && vaRms<=opts.PassAirspeedRmsMps && ...
    vzRms<=opts.PassVzRmsMps && qRms<=opts.PassQRmsDps;

rankScore=10*hRms + 5*abs(hDrift) + 2*vaRms + 2*vzRms + qRms + ...
    2*abs(tailVz) + abs(tailHErr);

M=table(minH,hRms,hMax,hDrift,vaRms,vzRms,qRms,tailHErr,tailVz,tailQ, ...
    maxED,maxTD,maxBE,maxBT,logical(hard),logical(formal),rankScore, ...
    'VariableNames',{'min_altitude_m','steady_h_rms_m','steady_h_max_abs_m', ...
    'steady_h_drift_m','steady_Va_rms_mps','steady_vz_rms_mps','steady_q_rms_dps', ...
    'tail_h_error_m','tail_vz_mps','tail_q_dps','max_physical_elevator_deviation', ...
    'max_throttle_deviation','max_bridge_elevator_error','max_bridge_throttle_error', ...
    'hard_fail','formal_pass','rank_score'});
end

function local_plot(T,eNom,tNom,outFile,name)
fig=figure('Visible','off','Color','w','Position',[100 100 1300 1000]);
tl=tiledlayout(7,1,'Padding','compact','TileSpacing','compact'); t=T.time_s;
nexttile; plot(t,T.altitude_m); hold on; yline(T.target_altitude_m(1),'--'); grid on; ylabel('h m');
nexttile; plot(t,T.airspeed_mps); hold on; yline(T.target_airspeed_mps(1),'--'); grid on; ylabel('Va');
nexttile; plot(t,T.pitch_deg); grid on; ylabel('pitch');
nexttile; plot(t,T.vz_up_mps); yline(0,'--'); grid on; ylabel('vz');
nexttile; plot(t,T.q_dps); yline(0,'--'); grid on; ylabel('q');
nexttile; plot(t,T.elevator_cmd_norm); hold on; yline(eNom,'--'); grid on; ylabel('elev physical');
nexttile; plot(t,T.throttle_norm); hold on; yline(tNom,'--'); grid on; ylabel('throttle'); xlabel('time s');
title(tl,"AirdropX v14 "+name,'Interpreter','none');
exportgraphics(fig,outFile,'Resolution',160); close(fig);
end

function x=local_col(T,name)
if ismember(string(name),string(T.Properties.VariableNames))
    x=double(T.(char(name))(:));
else
    x=NaN(height(T),1);
end
end

function v=local_rms(x)
x=double(x(:)); x=x(isfinite(x));
if isempty(x), v=NaN; else, v=sqrt(mean(x.^2)); end
end

function v=local_default_if_nan(v,fallback)
if ~isfinite(double(v)), v=double(fallback); end
end

function paths=local_paths(projectRoot)
projectRoot=string(projectRoot);
if strlength(projectRoot)==0
    thisDir=fileparts(mfilename("fullpath"));
    matlabDir=fileparts(thisDir);
    projectRoot=string(fileparts(matlabDir));
else
    matlabDir=fullfile(projectRoot,"matlab");
end
paths=struct("projectRoot",char(projectRoot),"matlabDir",char(matlabDir), ...
    "mpcDir",char(fullfile(matlabDir,"mpc")),"autoDir",char(fullfile(matlabDir,"mpc_auto")), ...
    "sfuncDir",char(fullfile(matlabDir,"sfunc_jsbsim")));
end

function opts=local_options(varargin)
opts.ProjectRoot="";
opts.IdentifiedMat="matlab/results/mpc_auto_id_v11_clean_r1/identify/airdropx_identified_plants.mat";
opts.OutputRoot="matlab/results/mpc_auto_smoke_cfg0_v14";
opts.PhysicalElevatorNominals=[];
opts.InitialAltitudeM=20.0;
opts.InitialAirspeedMps=50.0;
opts.InitialPitchDeg=NaN;
opts.InitialFlightPathDeg=0.0;
opts.TargetAltitudeM=20.0;
opts.TargetAirspeedMps=50.0;
opts.CalibrationStopTimeS=0.5;
opts.CalibrationUseUntilS=0.25;
opts.PreflightStopTimeS=10.0;
opts.StopTimeS=45.0;
opts.ScoreStartTimeS=10.0;
opts.TailWindowS=5.0;
opts.MpcEnableTimeS=3.0;
opts.AuthorityScale=0.70;

opts.ElevatorDeviationLimit=0.035;
opts.ThrottleDeviationLimit=0.060;
opts.ElevatorDeviationRateLimit=0.006;
opts.ThrottleDeviationRateLimit=0.010;
opts.OutputWeights=[8.0 5.0 0.05 3.0 4.0];
opts.MVWeights=[0.20 0.15];
opts.MVRateWeights=[8.0 4.0];

% Kept only for run_closed_loop interface compatibility. v14 controller does
% not use altitude as a trust-exit condition.
opts.LegacyTrustAltitudeM=5.0;
opts.TrustAirspeedMps=4.0;
opts.TrustPitchDeg=4.0;
opts.TrustVzMps=2.5;
opts.TrustQDps=4.0;

opts.HardFloorAltitudeM=5.0;
opts.HardMaxQRmsDps=10.0;
opts.HardMaxAbsPitchDeg=30.0;
opts.MaxBridgeError=0.01;
opts.PassAltitudeRmsM=2.0;
opts.PassAltitudeMaxM=4.0;
opts.PassAltitudeDriftM=2.0;
opts.PassAirspeedRmsMps=2.0;
opts.PassVzRmsMps=1.0;
opts.PassQRmsDps=1.5;

if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin)
    name=string(varargin{i});
    if ~isfield(opts,name), error("Unknown option: %s",name); end
    opts.(name)=varargin{i+1};
end
end
