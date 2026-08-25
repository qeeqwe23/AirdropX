function report=airdropx_gui_backend_entry_v136p(projectRoot,configPath)
%AIRDROPX_GUI_BACKEND_ENTRY_V136P JSON bridge for the PyQt6 GUI using v1.3.6-Paper.
arguments
    projectRoot (1,1) string
    configPath (1,1) string
end
if ~isfile(configPath)
    error("AirdropX:GUI:ConfigMissing","Config not found: %s",configPath);
end
cfg=jsondecode(fileread(configPath));
localAddCleanMatlabPath(projectRoot);
H=double(cfg.target_altitude_m); V=double(cfg.target_speed_mps); fuel=double(cfg.fuel_scale);
out=string(cfg.output_root); if ~isfolder(out), mkdir(out); end
fprintf("[GUI] Physics-MPC v1.3.6-Paper backend start\n");
fprintf("[GUI] H=%.6g V=%.6g FuelScale=%.4g\n",H,V,fuel);

if string(cfg.wind.mode)=="formal"
    scenario=string(cfg.wind.kind);
    if ~ismember(scenario,["calm","tailwind_5","headwind_5","tailwind_12","headwind_12","step_bidirectional","ramp_minus10_plus10","sine_longitudinal"])
        error("AirdropX:GUI:BadFormalScenario","Unsupported v1.3.6-Paper scenario: %s",scenario);
    end
    seed=localPaperSeed(scenario,double(cfg.sensor_noise_seed));
    fprintf("[GUI] v1.3.6-Paper formal preset seed=%d\n",seed);
else
    scenario="gui_custom";
    seed=double(cfg.sensor_noise_seed);
    setappdata(0,"AirdropXGuiWindConfig",cfg.wind);
    cleanupObj=onCleanup(@()localClearGuiWind());
    fprintf("[GUI] custom wind kind=%s along=%+.6g m/s\n",string(cfg.wind.kind),double(cfg.wind.along_track_mps));
end

policy=string(cfg.model_policy);
M=airdropx_gui_prepare_model_v136p(projectRoot,H,V,fuel,policy);
fprintf("[GUI_MODEL] source=%s\n",M.source);
fprintf("[GUI_PROGRESS] 0.08\n");

% v1.3.6-Paper keeps the validated v1.3.6 controller/release logic but uses
% the paper fair-sensor path. The main paper cargo score remains the nominal
% v1.3.6 ballistic model; independent cargo truth is a sensitivity ablation.
report=airdropx_wind_airdrop_mission_v136p(projectRoot, ...
    BankPath=string(M.bankPath),WindCalibrationPath=string(M.windPath),OutputRoot=out, ...
    ScenarioName=scenario,H=H,V=V,FuelScale=fuel,Duration_s=double(cfg.duration_s), ...
    TargetStart_m=double(cfg.target_start_m),TargetSpacing_m=double(cfg.target_spacing_m), ...
    SensorNoiseSeed=seed,SineForcingEnd_s=double(cfg.wind.forcing_end_s), ...
    SineSettleRamp_s=double(cfg.wind.settle_ramp_s),UseWindCompensation=true,UseWindDisturbanceMPC=true, ...
    UseFractionalRelease=true,UseWindConfidenceGate=true,UseUnifiedGustRecovery=true, ...
    UsePaperSensorModel=true,UseIndependentCargoTruth=false,ThrowOnFail=false);

meta=struct();
meta.backend="v136p"; meta.backend_label="Physics-MPC v1.3.6-Paper";
meta.model_source=M.source; meta.scenario=scenario; meta.wind_mode=string(cfg.wind.mode);
meta.H=H; meta.V=V; meta.FuelScale=fuel; meta.sensor_noise_seed=seed;
meta.paper_core_pass=logical(report.pass); meta.engineering_gate_pass=logical(report.engineering_pass);
meta.software_execution_success=true;
localWriteJson(fullfile(out,"gui_backend_meta.json"),meta);

if exist("cleanupObj","var"), delete(cleanupObj); end
fprintf("[GUI_PROGRESS] 1.0\n");
fprintf("[GUI] mission complete paper_core_pass=%d engineering_gate_pass=%d\n",logical(report.pass),logical(report.engineering_pass));
end

function seed=localPaperSeed(scenario,fallback)
switch string(scenario)
    case "calm", seed=101;
    case "tailwind_5", seed=102;
    case "headwind_5", seed=103;
    case "tailwind_12", seed=104;
    case "headwind_12", seed=105;
    case "step_bidirectional", seed=106;
    case "ramp_minus10_plus10", seed=107;
    case "sine_longitudinal", seed=108;
    otherwise, seed=fallback;
end
end

function localClearGuiWind()
try
    if isappdata(0,"AirdropXGuiWindConfig")
        rmappdata(0,"AirdropXGuiWindConfig");
    end
catch
end
end

function localAddCleanMatlabPath(projectRoot)
root=fullfile(projectRoot,"matlab");
parts=string(strsplit(genpath(root),pathsep));
for k=1:numel(parts)
    p=parts(k);
    if strlength(p)==0, continue; end
    pl=lower(p);
    if contains(pl,"\_backup") || contains(pl,"/_backup") || contains(pl,"\results") || contains(pl,"/results") || contains(pl,"\outputs") || contains(pl,"/outputs")
        continue
    end
    addpath(char(p));
end
end

function localWriteJson(path,S)
fid=fopen(path,"w");
if fid<0, return; end
cleanupObj=onCleanup(@()fclose(fid));
fprintf(fid,"%s",jsonencode(S));
delete(cleanupObj);
end
