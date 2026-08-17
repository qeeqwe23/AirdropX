function result=airdropx_urmpc_run_closed_loop(varargin)
%AIRDROPX_URMPC_RUN_CLOSED_LOOP Run the single adaptive UR-MPC and export trace.
opts=local_options(varargin{:});root=local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));
bank=local_resolve(root,opts.BankMat);if ~isfile(bank),error('AirdropX:URMPC:MissingBank','Missing UR-MPC bank: %s',bank);end
B=load(bank,'ur_mpc','ur_models','ur_meta');
if ~isfield(B,'ur_mpc')||~isfield(B,'ur_models')||~isfield(B,'ur_meta'),error('AirdropX:URMPC:BadBank','Not a UR-MPC v2 bank.');end
modelName=char(string(opts.Model));
modelPath=fullfile(root,'matlab','mpc_auto',[modelName '.slx']);
if ~isfile(modelPath),airdropx_urmpc_setup_model('ProjectRoot',root,'ModelName',modelName);end
cfg=1;if isfinite(opts.FixedConfigId),cfg=min(max(round(opts.FixedConfigId),0),size(B.ur_models,2)-1)+1;end
[~,ni]=min(abs(double(B.ur_meta.speed_nodes_mps(:))-double(opts.InitialAirspeedMps)));
m=B.ur_models(ni,cfg);
physical=double(m.u_nominal(1));
initialElev=physical-double(opts.HiddenElevatorTrim);initialThrottle=double(m.u_nominal(2));
if ~isfinite(opts.InitialPitchDeg),opts.InitialPitchDeg=double(m.x_nominal(3));end

% Only reference/profile + trace variables are supplied. There is NO height
% governor, recovery controller, cfg-specific authority multiplier or
% external actuator slew limiter in UR-MPC v2.
assignin('base','airdropx_v32_dynamic_reference_profile',double(opts.DynamicReferenceProfile));
assignin('base','airdropx_urmpc_controller_trace',zeros(0,45));
assignin('base','airdropx_physics_mpc_runtime_hidden_elevator',double(opts.HiddenElevatorTrim));
assignin('base','airdropx_auto_mpc_bank_mat_path',bank);

out=local_resolve(root,opts.OutputRoot);if ~isfolder(out),mkdir(out);end
R=airdropx_auto_run_closed_loop('ProjectRoot',root,'Model',modelName,'MpcBankMat',bank,'OutputRoot',out, ...
    'CaseId',opts.CaseId,'StopTimeS',opts.StopTimeS,'AfterDropTime',opts.AfterDropTime, ...
    'FixedConfigId',opts.FixedConfigId,'FixedDropTotal',opts.FixedDropTotal,'FixedDropStartS',opts.FixedDropStartS,'FixedDropIntervalS',opts.FixedDropIntervalS, ...
    'InitialAltitudeM',opts.InitialAltitudeM,'InitialAirspeedMps',opts.InitialAirspeedMps,'InitialPitchDeg',opts.InitialPitchDeg,'InitialFlightPathDeg',0, ...
    'InitialElevatorDelta',initialElev,'InitialThrottleCmd',initialThrottle,'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg, ...
    'HiddenElevatorTrim',opts.HiddenElevatorTrim,'MpcEnableTimeS',0, ...
    'MpcAuthorityScale',1.0,'MpcAuthorityByConfig',ones(5,1),'HeightToVzGain',0,'HeightIntegralGain',0,'HeightVzRefLimitMps',10, ...
    'BumplessTransitionEnabled',false,'V31ContinuousControllerStateEnabled',false,'V31HeightGovernorEnabled',false, ...
    'V31DynamicReferenceEnabled',false,'V31SchedulerEnabled',false,'V31ReferenceInstrumentationEnabled',false, ...
    'TargetAltitudeM',opts.TargetAltitudeM,'TargetAirspeedMps',opts.TargetAirspeedMps,'TargetPitchDeg',double(m.x_nominal(3)),'UseTrimPitchReference',1, ...
    'TrustAltitudeM',1e6,'TrustAirspeedMps',100,'TrustPitchDeg',90,'TrustVzMps',50,'TrustQDps',100, ...
    'TestPulse1StartS',Inf,'TestPulse1DurationS',0,'TestPulse2StartS',Inf,'TestPulse2DurationS',0);
Ttrace=local_trace_table();tracePath="";
if ~isempty(Ttrace),tracePath=string(fullfile(out,'urmpc_controller_trace.csv'));writetable(Ttrace,tracePath);end
audit=struct();
if ~isempty(Ttrace)
    try,audit=airdropx_urmpc_residual_audit(Ttrace,B,out);catch ME,warning('AirdropX:URMPC:ResidualAuditFailed','Residual audit failed: %s',ME.message);end
end
result=R;result.urmpc_controller_trace=Ttrace;result.urmpc_controller_trace_csv=tracePath;result.urmpc_residual_audit=audit;
end

function T=local_trace_table()
T=table();try,X=double(evalin('base','airdropx_urmpc_controller_trace'));catch,X=[];end
if isempty(X)||~ismember(size(X,2),[34 43 45]),return;end
names={'time_s','cfg_id','requested_h_m','requested_v_mps','actual_h_m','actual_v_mps','actual_vz_mps','pitch_deg','q_est_dps','mass_kg_controller','cg_x_m', ...
    'node_low_index','node_high_index','node_weight_high','nominal_va_mps','nominal_pitch_deg','nominal_physical_elevator','nominal_throttle', ...
    'physical_elevator_cmd','physical_throttle_cmd','plant_elevator_delta','plant_throttle_cmd','mpc_success_count','mpc_fail_count','mpc_exception_count', ...
    'mpc_last_iterations','mpc_last_slack','mpc_last_cost','cfg_mass_raw','cfg_invalid_count','input_invalid_count','startup_hold_count','state_ready','runtime_hidden_elevator'};
if size(X,2)==43
    names=[names,{'est_plant_h_m','est_plant_va_mps','est_plant_pitch_deg','est_plant_vz_mps','est_plant_q_dps', ...
        'est_disturbance_1','est_disturbance_2','est_disturbance_norm','cfg_changed'}];
elseif size(X,2)==45
    names=[names,{'est_plant_h_m','est_plant_va_mps','est_plant_pitch_deg','est_plant_vz_mps','est_plant_q_dps', ...
        'est_disturbance_1','est_disturbance_2','est_disturbance_norm','est_disturbance_tail_norm','est_disturbance_state_count','cfg_changed'}];
end
T=array2table(X,'VariableNames',names);
end
function p=local_resolve(root,x),p=char(string(x));if isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),p=fullfile(root,p);end,end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.Model="airdropx_urmpc_closed_loop";opts.BankMat="matlab/results/mpc_physics_v1/unified_robust_mpc_v2/airdropx_unified_robust_mpc_bank.mat";opts.OutputRoot="matlab/results/mpc_physics_v1/fixed_stability_urmpc_v20/run";opts.CaseId="urmpc_v20";opts.AircraftName="MQ9_Reaper";opts.IcName="";
opts.StopTimeS=255;opts.AfterDropTime=10;opts.FixedConfigId=NaN;opts.FixedDropTotal=4;opts.FixedDropStartS=50;opts.FixedDropIntervalS=50;
opts.InitialAltitudeM=200;opts.InitialAirspeedMps=50;opts.InitialPitchDeg=NaN;opts.TargetAltitudeM=200;opts.TargetAirspeedMps=50;opts.DynamicReferenceProfile=[];
opts.ReferenceMassKg=3423;opts.CargoMassKg=300;opts.HiddenElevatorTrim=NaN;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
