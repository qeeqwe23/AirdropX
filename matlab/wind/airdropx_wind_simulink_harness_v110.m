function T=airdropx_wind_simulink_harness_v110(projectRoot,scenarioName,duration_s,opts)
%AIRDROPX_WIND_SIMULINK_HARNESS_V110 Dedicated JSBSim/S-Function longitudinal-wind truth harness.
% Estimator inputs never use JSBSim wind truth channels. windN/windE are logged
% only for independent scoring.
arguments
    projectRoot (1,1) string
    scenarioName (1,1) string
    duration_s (1,1) double {mustBePositive}
    opts.CalibrationDuration_s (1,1) double {mustBePositive} = 2.0
end
if exist("sim","file")~=2
    error("AirdropX:Wind:MissingSimulink","Simulink is required for JSBSim wind validation.");
end
sfun=fullfile(projectRoot,"matlab","sfunc_jsbsim","sfun_airdropx_jsbsim.mexw64");
if ~isfile(sfun), error("AirdropX:Wind:MissingSFunction","Working sfun_airdropx_jsbsim.mexw64 not found: %s",sfun); end
if exist("airdropx_sim_params","file")~=2
    addpath(fullfile(projectRoot,"matlab"));
end
cfg=airdropx_sim_params("ProjectRoot",projectRoot,"AssignBase",false);
dt=double(cfg.sim.dt_s);
addpath(fullfile(projectRoot,"matlab","sfunc_jsbsim"));
pid=feature("getpid");
mdl="ax_wind_v110_"+string(pid)+"_"+string(randi(1e6));
cleanup=onCleanup(@()localCleanup(mdl)); %#ok<NASGU>
load_system("simulink");
new_system(mdl);
add_block("simulink/Sources/From Workspace",mdl+"/Input","VariableName","wind_validation_input","Position",[30 70 180 110]);
add_block("simulink/User-Defined Functions/S-Function",mdl+"/Plant","FunctionName","sfun_airdropx_jsbsim","Position",[250 45 470 135]);
param=sprintf("'%s','%s','%s',%.17g",localEscapeChar(projectRoot),localEscapeChar(cfg.aircraftName),localEscapeChar(cfg.icName),dt);
set_param(mdl+"/Plant","Parameters",param);
add_block("simulink/Sinks/To Workspace",mdl+"/Output","VariableName","wind_validation_y","SaveFormat","Structure With Time","Position",[540 70 690 110]);
add_line(mdl,"Input/1","Plant/1","autorouting","on");
add_line(mdl,"Plant/1","Output/1","autorouting","on");
set_param(mdl,"SolverType","Fixed-step","Solver","FixedStepDiscrete","FixedStep",sprintf("%.17g",dt));

% First run with zero wind to establish the actual longitudinal flight axis.
tc=(0:dt:opts.CalibrationDuration_s).';
Uc=zeros(numel(tc),6); Uc(:,2)=0.80; Uc(:,4)=0;
Yc=localSim(mdl,tc,Uc,opts.CalibrationDuration_s);
psiTail=Yc(max(1,end-round(0.5/dt)):end,8);
z=mean(exp(1i*deg2rad(psiTail)));
heading0=mod(rad2deg(angle(z)),360);

t=(0:dt:duration_s).';
wCmd=airdropx_wind_profile_v110(scenarioName,t);
speed=abs(wCmd);
dirFrom=repmat(heading0,size(t));
tail=wCmd>0;
dirFrom(tail)=mod(heading0+180,360); % positive: wind blows with aircraft
% negative: wind comes from aircraft heading and therefore opposes motion.
U=zeros(numel(t),6);
U(:,1)=0; U(:,2)=0.80; U(:,3)=speed; U(:,4)=dirFrom; U(:,5)=0; U(:,6)=0;
Y=localSim(mdl,t,U,duration_s);
tt=Y.time; A=Y.data;
if size(A,2)~=20, error("AirdropX:Wind:BadSFunctionOutput","Expected 20 S-function outputs, got %d.",size(A,2)); end
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

function s=localEscapeChar(x)
s=strrep(char(x),"'","''");
end

function localCleanup(mdl)
try
    if bdIsLoaded(mdl), close_system(mdl,0); end
catch
end
end
