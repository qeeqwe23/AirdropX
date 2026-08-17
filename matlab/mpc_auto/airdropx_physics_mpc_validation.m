function result = airdropx_physics_mpc_validation(varargin)
%AIRDROPX_PHYSICS_MPC_VALIDATION One continuous H/V + four-drop engineering test.
opts=local_options(varargin{:});root=local_root(opts.ProjectRoot);
if isempty(opts.Profile)
    % Deliberately non-grid commands prove that runtime references are not
    % restricted to the 45/50/55 model nodes or a handful of altitude nodes.
    opts.Profile=[0 200 50; 25 137 47.3; 55 184 52.4; 90 76 49.1; 125 28 54.2; 165 121 46.6; 205 200 50];
end
R=airdropx_physics_mpc_run('ProjectRoot',root,'BankMat',opts.BankMat,'OutputRoot',opts.OutputRoot, ...
    'CaseId','physics_mpc_dynamic_4drop','StopTimeS',opts.StopTimeS,'FixedConfigId',NaN,'FixedDropTotal',4, ...
    'FixedDropStartS',opts.DropStartS,'FixedDropIntervalS',opts.DropIntervalS,'InitialAltitudeM',200,'InitialAirspeedMps',50, ...
    'TargetAltitudeM',200,'TargetAirspeedMps',50,'DynamicReferenceProfile',opts.Profile);
T=R.v32_controller_trace;if isempty(T),error('AirdropX:PhysicsMPC:NoTrace','Controller trace missing.');end
m=all(isfinite(T{:,{'actual_h_m','actual_v_mps','actual_vz_mps','pitch_deg','q_dps','physical_elevator_cmd','physical_throttle_cmd'}}),2);
S=struct();S.finite_fraction=mean(m);S.max_abs_pitch_deg=max(abs(T.pitch_deg(m)),[],'omitnan');S.max_abs_q_dps=max(abs(T.q_dps(m)),[],'omitnan');
S.max_abs_vz_mps=max(abs(T.actual_vz_mps(m)),[],'omitnan');S.max_mpc_fail_count=max(T.mpc_fail_count,[],'omitnan');
S.elevator_sat_fraction=mean(abs(T.physical_elevator_cmd(m))>0.94);S.throttle_sat_fraction=mean(T.physical_throttle_cmd(m)<0.01|T.physical_throttle_cmd(m)>0.99);
last=T.time_s>=max(T.time_s)-15;S.final_height_error_m=median(T.requested_h_m(last)-T.actual_h_m(last),'omitnan');S.final_speed_error_mps=median(T.requested_v_mps(last)-T.actual_v_mps(last),'omitnan');
S.pass=S.finite_fraction>0.999 && S.max_abs_pitch_deg<25 && S.max_abs_q_dps<12 && S.max_abs_vz_mps<6 && ...
    S.elevator_sat_fraction<0.10 && S.throttle_sat_fraction<0.10 && abs(S.final_height_error_m)<2.0 && abs(S.final_speed_error_mps)<1.0;
M=struct2table(rmfield(S,'pass'));M.pass=S.pass;writetable(M,fullfile(char(string(opts.OutputRoot)),'physics_mpc_validation_summary.csv'));
result=struct('run',R,'summary',S,'profile',opts.Profile);fprintf('[PHYS-MPC] dynamic validation pass=%d final H err=%.3f m final Va err=%.3f m/s\n',S.pass,S.final_height_error_m,S.final_speed_error_mps);
end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.BankMat="matlab/results/mpc_physics_v1/airdropx_physics_mpc_bank.mat";opts.OutputRoot="matlab/results/mpc_physics_v1/formal_dynamic_validation";
opts.Profile=[];opts.StopTimeS=240;opts.DropStartS=65;opts.DropIntervalS=18;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
