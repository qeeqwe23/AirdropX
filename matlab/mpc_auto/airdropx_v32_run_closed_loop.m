function result=airdropx_v32_run_closed_loop(varargin)
%AIRDROPX_V32_RUN_CLOSED_LOOP Run the clean v32 controller and export internal trace.
opts=local_options(varargin{:}); root=local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));
if ~isfile(opts.BankMat),error('AirdropX:V32:MissingBank','Missing v32 bank: %s',opts.BankMat);end
B=load(opts.BankMat,'v32_nodes','speed_nodes','trim_bank','mpc_meta');
if ~isfield(B,'v32_nodes'),error('AirdropX:V32:LegacyBankRejected','Only a clean v32 bank is accepted.');end
if ~isfile(fullfile(root,'matlab','mpc_auto',char(string(opts.Model)+".slx"))),airdropx_v32_setup_model('ProjectRoot',root,'ModelName',opts.Model);end
cfg=1;if isfinite(opts.FixedConfigId),cfg=min(max(round(opts.FixedConfigId),0),4)+1;end
[~,ni]=min(abs(double(B.speed_nodes(:))-double(opts.InitialAirspeedMps)));node=B.v32_nodes(ni);trim=node.trim_bank(cfg);
physical=double(node.mpc_meta.physical_elevator_nominals(cfg));
initialElev=physical-double(opts.HiddenElevatorTrim); initialThrottle=double(trim.throttle_cmd);
if ~isfinite(opts.InitialPitchDeg),opts.InitialPitchDeg=double(trim.pitch_deg);end

% v32-only runtime variables. No legacy bank/seed variable is read by the v32 S-function.
assignin('base','airdropx_v32_inner_reference_enabled',double(opts.InnerReferenceEnabled));
assignin('base','airdropx_v32_inner_reference_profile',double(opts.InnerReferenceProfile));
assignin('base','airdropx_v32_dynamic_reference_profile',double(opts.DynamicReferenceProfile));
assignin('base','airdropx_v32_height_kh',double(opts.HeightKh));
assignin('base','airdropx_v32_height_ki',double(opts.HeightKi));
assignin('base','airdropx_v32_height_kaw',double(opts.HeightKaw));
assignin('base','airdropx_v32_height_vz_max_mps',double(opts.HeightVzMaxMps));
assignin('base','airdropx_v32_height_vz_slew_mps2',double(opts.HeightVzSlewMps2));
assignin('base','airdropx_v32_height_bias_max_mps',double(opts.HeightBiasMaxMps));
assignin('base','airdropx_v32_speed_accel_mps2',double(opts.SpeedAccelMps2));
assignin('base','airdropx_v32_speed_decel_mps2',double(opts.SpeedDecelMps2));
assignin('base','airdropx_v32_elevator_step_limit',double(opts.ElevatorStepLimit));
assignin('base','airdropx_v32_throttle_step_limit',double(opts.ThrottleStepLimit));
assignin('base','airdropx_v32_controller_trace',zeros(0,56));

R=airdropx_auto_run_closed_loop('ProjectRoot',root,'Model',opts.Model,'MpcBankMat',opts.BankMat,'OutputRoot',opts.OutputRoot,...
    'CaseId',opts.CaseId,'StopTimeS',opts.StopTimeS,'AfterDropTime',opts.AfterDropTime,...
    'FixedConfigId',opts.FixedConfigId,'FixedDropTotal',opts.FixedDropTotal,'FixedDropStartS',opts.FixedDropStartS,'FixedDropIntervalS',opts.FixedDropIntervalS,...
    'InitialAltitudeM',opts.InitialAltitudeM,'InitialAirspeedMps',opts.InitialAirspeedMps,'InitialPitchDeg',opts.InitialPitchDeg,'InitialFlightPathDeg',0,...
    'InitialElevatorDelta',initialElev,'InitialThrottleCmd',initialThrottle,'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg,...
    'HiddenElevatorTrim',opts.HiddenElevatorTrim,'MpcEnableTimeS',opts.MpcEnableTimeS,'AircraftName',opts.AircraftName,'IcName',opts.IcName,...
    'MpcAuthorityScale',1.0,'MpcAuthorityByConfig',ones(5,1),'HeightToVzGain',0,'HeightIntegralGain',0,'HeightVzRefLimitMps',10,...
    'BumplessTransitionEnabled',false,'V31ContinuousControllerStateEnabled',false,'V31HeightGovernorEnabled',false,...
    'V31DynamicReferenceEnabled',false,'V31SchedulerEnabled',false,'V31ReferenceInstrumentationEnabled',false,...
    'TargetAltitudeM',opts.TargetAltitudeM,'TargetAirspeedMps',opts.TargetAirspeedMps,'TargetPitchDeg',double(trim.pitch_deg),'UseTrimPitchReference',1,...
    'TrustAltitudeM',1e6,'TrustAirspeedMps',50,'TrustPitchDeg',30,'TrustVzMps',10,'TrustQDps',20,...
    'TestPulse1StartS',Inf,'TestPulse1DurationS',0,'TestPulse2StartS',Inf,'TestPulse2DurationS',0);
Ttrace=local_trace_table(); tracePath="";
if ~isempty(Ttrace)
    tracePath=string(fullfile(opts.OutputRoot,'v32_controller_trace.csv'));writetable(Ttrace,tracePath);
end
result=R;result.v32_controller_trace=Ttrace;result.v32_controller_trace_csv=tracePath;
end

function T=local_trace_table()
T=table();try,X=double(evalin('base','airdropx_v32_controller_trace'));catch,X=[];end
if isempty(X)||size(X,2)~=56,return;end
names={'time_s','cfg_id','requested_h_m','requested_v_mps','governed_v_mps','actual_h_m','actual_v_mps','actual_vz_mps','pitch_deg','q_dps','h_error_m',...
    'raw_vz_ref_mps','limited_vz_ref_mps','vz_ref_mps','height_bias_mps','node_low_index','node_high_index','node_weight_high','node_low_speed_mps','node_high_speed_mps',...
    'physical_elevator_cmd','physical_throttle_cmd','plant_elevator_delta','plant_throttle_cmd','inner_reference_mode','mpc_fail_count','height_kh','height_ki','height_kaw','height_vz_max_mps',...
    'mpc_success_count','mpc_exception_count','mpc_qp_fail_count','mpc_last_iterations','mass_kg_controller','cfg_mass_raw','cfg_used','cfg_invalid_count',...
    'input_invalid_count','startup_hold_count','state_ready', ...
    'mpc_gate_reject_count','recovery_count','recovery_mode','recovery_reason_code','recovery_hard_count', ...
    'authority_limit_count','authority_limit_streak','command_deviation_elevator','command_deviation_throttle','recovery_enter_count', ...
    'tracking_loss_count','tracking_loss_streak','recovery_energy_error_jpkg','recovery_target_deviation_elevator','recovery_target_deviation_throttle'};
T=array2table(X,'VariableNames',names);
end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.Model="airdropx_v32_mpc_closed_loop";opts.BankMat="";opts.OutputRoot="";opts.CaseId="v32";opts.AircraftName="MQ9_Reaper";opts.IcName="";
opts.StopTimeS=120;opts.AfterDropTime=10;opts.FixedConfigId=0;opts.FixedDropTotal=0;opts.FixedDropStartS=60;opts.FixedDropIntervalS=60;
opts.InitialAltitudeM=200;opts.InitialAirspeedMps=50;opts.InitialPitchDeg=NaN;opts.TargetAltitudeM=200;opts.TargetAirspeedMps=50;
opts.ReferenceMassKg=3423;opts.CargoMassKg=300;opts.HiddenElevatorTrim=0;opts.MpcEnableTimeS=2;
opts.InnerReferenceEnabled=false;opts.InnerReferenceProfile=[];opts.DynamicReferenceProfile=[];
opts.HeightKh=0.12;opts.HeightKi=0.004;opts.HeightKaw=0.30;opts.HeightVzMaxMps=2.0;opts.HeightVzSlewMps2=0.60;opts.HeightBiasMaxMps=1.5;
opts.SpeedAccelMps2=1.0;opts.SpeedDecelMps2=1.2;opts.ElevatorStepLimit=0.012;opts.ThrottleStepLimit=0.020;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
if strlength(string(opts.OutputRoot))==0,opts.OutputRoot=fullfile(local_root(opts.ProjectRoot),'matlab','results','mpc_auto_v32_clean','run');end
end
