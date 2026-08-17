function report = airdropx_physics_mpc_direct_cfg_smoke(varargin)
%AIRDROPX_PHYSICS_MPC_DIRECT_CFG_SMOKE Verify NO-MEX direct-cfg aircraft selection.
%
% v1.5 does not trust the legacy MEX mass/drop bookkeeping here, because
% that bookkeeping may not know that cargo was removed in a temporary XML.
% Instead the smoke test verifies that a cfg-specific aircraft variant was
% actually selected. JSBSim LoadModel would throw if that variant could not
% be loaded, so a completed run plus the variant-name check is the proof.

opts=local_options(varargin{:});root=char(string(opts.ProjectRoot));if isempty(root),a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));
v32=char(string(opts.ExistingV32Root));if isempty(regexp(v32,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),v32=fullfile(root,v32);end
v=double(opts.SpeedMps);cfg=round(double(opts.ConfigId));
p=fullfile(v32,'knowledge_bank','physics',sprintf('V%06.3f',v),'v32_trim_bank.mat');S=load(p);if ~isfield(S,'bank'),error('AirdropX:PhysicsMPC:SmokeTrimBank','bank missing in %s',p);end
bank=S.bank;trim=bank(cfg+1);outRoot=fullfile(root,'matlab','results','mpc_physics_v1','preflight_direct_cfg');
if isfolder(outRoot),try,rmdir(outRoot,'s');catch,end,end
r=airdropx_auto_run_id_experiment('ProjectRoot',root,'OutputRoot',outRoot,'RunId','v15_direct_cfg_smoke', ...
    'ConfigId',cfg,'InitialDropCount',cfg,'PrepareByDrops',false,'DirectCfgViaAircraftXml',true,'Trim',trim,'StopTimeS',4,'RecordStartS',0,'ExportStartS',0,'ExcitationStartS',Inf, ...
    'KeepFixedConfigurationOnly',true,'DirectIdMode',true,'PreparationTrimBank',bank,'UsePreparationTrimSchedule',false, ...
    'InitialAirspeedMps',v,'InitialAltitudeM',200,'InitialPitchDeg',double(trim.pitch_deg),'InitialFlightPathDeg',0, ...
    'TargetAltitudeM',200,'TargetAirspeedMps',v,'IsolateGeneratedIc',true,'ElevatorAmplitude',0,'ThrottleAmplitude',0);
T=r.timeseries;if isempty(T),error('AirdropX:PhysicsMPC:DirectCfgInitFailed','Direct cfg smoke returned no samples.');end
used=string(r.aircraft_name_used);needle="_PHYS_cfg"+string(cfg)+"_";
coordPass=ismember('elevator_external_delta_actual',T.Properties.VariableNames) && ismember('elevator_physical_actual',T.Properties.VariableNames) && ismember('elevator_cmd_norm',T.Properties.VariableNames) && any(isfinite(double(T.elevator_cmd_norm(:))));
hidden=NaN;
if coordPass
    de=double(T.elevator_external_delta_actual(:)); ep=double(T.elevator_physical_actual(:));
    m=isfinite(de)&isfinite(ep);if any(m),hidden=median(ep(m)-de(m),'omitnan');end
end
pass=logical(r.direct_cfg_via_aircraft_xml) && contains(used,needle) && coordPass && isfinite(hidden);
report=struct('pass',pass,'config_id',cfg,'aircraft_name_used',used,'strategy','temporary_aircraft_xml_only', ...
    'elevator_coordinate_pass',coordPass,'measured_hidden_elevator_offset',hidden);
fprintf('[PHYS-MPC] direct-cfg NO-MEX smoke: cfg%d aircraft=%s strategy=XML_ONLY hiddenE=%.6f coordPASS=%d PASS=%d\n',cfg,used,hidden,coordPass,pass);
if ~pass,error('AirdropX:PhysicsMPC:DirectCfgInitFailed','Temporary cfg aircraft/elevator-coordinate smoke failed for cfg%d. Used=%s',cfg,used);end
end
function o=local_options(varargin)
o.ProjectRoot="";o.ExistingV32Root="matlab/results/mpc_auto_v32_clean";o.SpeedMps=50;o.ConfigId=3;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(o,n),error('Unknown option: %s',n);end,o.(n)=varargin{i+1};end
end
