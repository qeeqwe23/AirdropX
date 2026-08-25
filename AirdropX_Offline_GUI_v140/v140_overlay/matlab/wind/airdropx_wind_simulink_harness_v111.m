function T=airdropx_wind_simulink_harness_v111(projectRoot,scenarioName,duration_s,opts)
%AIRDROPX_WIND_SIMULINK_HARNESS_V111 Dedicated JSBSim/S-Function longitudinal-wind truth harness.
% v1.1.3 keeps the v1.1.2 Simulink capability fix and repairs a second confirmed harness bug:
% the S-Function library block must receive its 4 dialog parameters before FunctionName is activated.
% Each MATLAB
% process creates a process-private IC file from the immutable reset template.
% Estimator inputs never use JSBSim wind truth channels. windN/windE are
% logged only for independent scoring.
arguments
    projectRoot (1,1) string
    scenarioName (1,1) string
    duration_s (1,1) double {mustBePositive}
    opts.CalibrationDuration_s (1,1) double {mustBePositive} = 2.0
    opts.WorkDir (1,1) string = ""
    opts.StatusFile (1,1) string = ""
end
localStatus(opts.StatusFile,"SIMULINK_PREFLIGHT_START");
localRequireSimulink(opts.StatusFile);
sfun=fullfile(projectRoot,"matlab","sfunc_jsbsim","sfun_airdropx_jsbsim.mexw64");
if ~isfile(sfun)
    error("AirdropX:Wind:MissingSFunction","Working sfun_airdropx_jsbsim.mexw64 not found: %s",sfun);
end

aircraftName="MQ9_Reaper";
dt=1/120;
templateIc=fullfile(projectRoot,"aircraft",aircraftName,"reset_20m.xml");
if ~isfile(templateIc)
    error("AirdropX:Wind:MissingIcTemplate","Wind harness IC template not found: %s",templateIc);
end
if strlength(opts.WorkDir)==0
    workDir=string(tempdir);
else
    workDir=opts.WorkDir;
end
if ~isfolder(workDir), mkdir(workDir); end
addpath(fullfile(projectRoot,"matlab","sfunc_jsbsim"));

pid=feature("getpid");
icPath=fullfile(workDir,sprintf("wind_ic_v113_%d_%06d.xml",pid,randi(999999)));
localWritePrivateIc(templateIc,icPath,45.0,4.0);
icCleanup=onCleanup(@()localDeleteFile(icPath)); %#ok<NASGU>
localStatus(opts.StatusFile,"PRIVATE_IC_READY "+string(icPath));

mdl="ax_wind_v113_"+string(pid)+"_"+string(randi(1e6));
cleanup=onCleanup(@()localCleanup(mdl)); %#ok<NASGU>
new_system(mdl);
add_block("simulink/Sources/From Workspace",mdl+"/Input","VariableName","wind_validation_input","Position",[30 70 180 110]);
plant=mdl+"/Plant";
param=sprintf("'%s','%s','%s',%.17g",localEscapeChar(projectRoot),localEscapeChar(aircraftName),localEscapeChar(icPath),dt);
% IMPORTANT: do not activate FunctionName while the dialog parameter list is still empty.
% The compiled S-Function declares ssSetNumSFcnParams(S,4); activating it with 0
% dialog parameters makes Simulink throw SFcnParamCountErr immediately.
add_block("simulink/User-Defined Functions/S-Function",plant,"Position",[250 45 470 135]);
set_param(plant,"Parameters",param);
set_param(plant,"FunctionName","sfun_airdropx_jsbsim");
if string(get_param(plant,"FunctionName"))~="sfun_airdropx_jsbsim" || strlength(string(get_param(plant,"Parameters")))==0
    error("AirdropX:Wind:SFunctionConfigurationFailed","Could not configure JSBSim S-Function name/4-parameter dialog atomically.");
end
localStatus(opts.StatusFile,"SFUNCTION_CONFIGURED parameter_count=4");
add_block("simulink/Sinks/To Workspace",mdl+"/Output","VariableName","wind_validation_y","SaveFormat","Structure With Time","Position",[540 70 690 110]);
add_line(mdl,"Input/1","Plant/1","autorouting","on");
add_line(mdl,"Plant/1","Output/1","autorouting","on");
set_param(mdl,"SolverType","Fixed-step","Solver","FixedStepDiscrete","FixedStep",sprintf("%.17g",dt));
localStatus(opts.StatusFile,"MODEL_UPDATE_START");
set_param(mdl,"SimulationCommand","update");
localStatus(opts.StatusFile,"MODEL_UPDATE_DONE");
localStatus(opts.StatusFile,"MODEL_READY");

% First run with zero wind to establish the actual longitudinal flight axis.
tc=(0:dt:opts.CalibrationDuration_s).';
Uc=zeros(numel(tc),6); Uc(:,2)=0.80; Uc(:,4)=0;
localStatus(opts.StatusFile,"CALIBRATION_SIM_START");
Yc=localSim(mdl,tc,Uc,opts.CalibrationDuration_s);
localStatus(opts.StatusFile,"CALIBRATION_SIM_DONE");
if size(Yc.data,2)~=20
    error("AirdropX:Wind:BadSFunctionOutput","Calibration expected 20 S-function outputs, got %d.",size(Yc.data,2));
end
psiTail=Yc.data(max(1,size(Yc.data,1)-round(0.5/dt)):end,8);
z=mean(exp(1i*deg2rad(psiTail)));
heading0=mod(rad2deg(angle(z)),360);
if ~isfinite(heading0)
    error("AirdropX:Wind:BadHeadingCalibration","Heading calibration returned non-finite heading.");
end
localStatus(opts.StatusFile,sprintf("HEADING_CALIBRATED %.6f deg",heading0));

t=(0:dt:duration_s).';
wCmd=airdropx_wind_profile_v111(scenarioName,t);
speed=abs(wCmd);
dirFrom=repmat(heading0,size(t));
tail=wCmd>0;
dirFrom(tail)=mod(heading0+180,360); % positive means wind blows with aircraft
U=zeros(numel(t),6);
U(:,1)=0; U(:,2)=0.80; U(:,3)=speed; U(:,4)=dirFrom; U(:,5)=0; U(:,6)=0;
localStatus(opts.StatusFile,"MAIN_SIM_START");
Y=localSim(mdl,t,U,duration_s);
localStatus(opts.StatusFile,"MAIN_SIM_DONE");
tt=Y.time; A=Y.data;
if size(A,2)~=20
    error("AirdropX:Wind:BadSFunctionOutput","Expected 20 S-function outputs, got %d.",size(A,2));
end
% C++ output enum is zero-based: MATLAB columns 3/4/5/8/16/17 below.
Vz=A(:,3); Va=A(:,4); Vg=A(:,5); heading=A(:,8); windN=A(:,16); windE=A(:,17);
psi0=deg2rad(heading0);
windLongTruth=windN*cos(psi0)+windE*sin(psi0);
windCrossTruth=-windN*sin(psi0)+windE*cos(psi0);
headingDev=rad2deg(atan2(sin(deg2rad(heading-heading0)),cos(deg2rad(heading-heading0))));
cmd=interp1(t,wCmd,tt,"previous","extrap");
T=table(tt,A(:,2),Vz,Va,Vg,A(:,6),A(:,7),heading,headingDev,windN,windE,windLongTruth,windCrossTruth,cmd,A(:,19), ...
    'VariableNames',{'t_s','h_m','Vz_mps','Va_mps','Vg_mps','pitch_deg','roll_deg','heading_deg','heading_dev_deg','windN_truth_mps','windE_truth_mps','wind_long_truth_mps','wind_cross_truth_mps','wind_cmd_mps','valid'});
end

function Y=localSim(mdl,t,U,stopTime)
ts=timeseries(U,t);
in=Simulink.SimulationInput(mdl);
in=in.setVariable("wind_validation_input",ts);
in=in.setModelParameter("StopTime",sprintf("%.17g",stopTime),"ReturnWorkspaceOutputs","on");
out=sim(in);
raw=out.get("wind_validation_y");
if ~isstruct(raw) || ~isfield(raw,"time") || ~isfield(raw,"signals")
    error("AirdropX:Wind:BadHarnessLog","To Workspace did not return Structure With Time.");
end
Y=struct("time",double(raw.time(:)),"data",double(raw.signals.values));
end

function localRequireSimulink(statusPath)
% Do not classify Simulink availability by the numeric return code of exist().
% In supported MATLAB releases sim may resolve through the Simulink engine/built-in
% dispatch even when that legacy check would reject a working installation.
v=ver("simulink");
if isempty(v)
    error("AirdropX:Wind:MissingSimulinkInstallation", ...
        "Simulink installation metadata was not found (ver('simulink') is empty).");
end
try
    licensed=license("test","Simulink");
catch ME
    error("AirdropX:Wind:SimulinkLicenseProbeFailed", ...
        "Could not test the Simulink license: %s",ME.message);
end
if ~licensed
    error("AirdropX:Wind:MissingSimulinkLicense", ...
        "Simulink is installed but the Simulink license test failed.");
end
try
    load_system("simulink");
catch ME
    error("AirdropX:Wind:SimulinkLoadFailed", ...
        "Simulink is installed/licensed but load_system('simulink') failed: %s",ME.message);
end
localStatus(statusPath,sprintf("SIMULINK_PREFLIGHT_OK version=%s release=%s", ...
    string(v(1).Version),string(v(1).Release)));
end

function localWritePrivateIc(templatePath,outPath,airspeedMps,pitchDeg)
xmlText=fileread(templatePath);
ubodyExpr='<ubody\s+unit="M/SEC">[^<]*</ubody>';
if isempty(regexp(xmlText,ubodyExpr,"once"))
    error("AirdropX:Wind:BadIcTemplate","IC template has no M/SEC ubody field: %s",templatePath);
end
xmlText=regexprep(xmlText,ubodyExpr,sprintf('<ubody unit="M/SEC">%.10g</ubody>',airspeedMps),"once");
thetaExpr='<theta\s+unit="DEG">[^<]*</theta>';
if isempty(regexp(xmlText,thetaExpr,"once"))
    error("AirdropX:Wind:BadIcTemplate","IC template has no DEG theta field: %s",templatePath);
end
xmlText=regexprep(xmlText,thetaExpr,sprintf('<theta unit="DEG">%.10g</theta>',pitchDeg),"once");
fid=fopen(outPath,"w","n","UTF-8");
if fid<0
    error("AirdropX:Wind:PrivateIcWriteFailed","Cannot write process-private IC: %s",outPath);
end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"%s",xmlText);
end

function s=localEscapeChar(x)
s=strrep(char(x),"'","''");
end

function localCleanup(mdl)
try
    if bdIsLoaded(mdl), close_system(mdl,0); end
catch
end
end

function localDeleteFile(path)
try
    if isfile(path), delete(path); end
catch
end
end

function localStatus(path,line)
if strlength(path)==0, return; end
fid=fopen(path,"a");
if fid>=0
    fprintf(fid,"%s  %s\n",char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")),char(line));
    fclose(fid);
end
end
