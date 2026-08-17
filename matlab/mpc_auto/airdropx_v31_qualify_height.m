function result = airdropx_v31_qualify_height(varargin)
%AIRDROPX_V31_QUALIFY_HEIGHT Frozen-learning mission qualification at H/V.
%
% A VERIFIED source controller/Plant state is copied into an isolated
% qualification directory.  No trim, ID, Plant rebuild or controller BO is
% allowed.  The only executed operation is the full cfg0->cfg4 Final Mission
% using the v31.2 single-channel height-governor policy.

opts=local_options(varargin{:});
projectRoot=local_project_root(opts.ProjectRoot);
sourceRoot=local_resolve_path(projectRoot,opts.SourceVerifiedRoot);
if ~isfolder(sourceRoot), error("AirdropX:V31:MissingQualificationSource","SourceVerifiedRoot missing: %s",sourceRoot); end
H=double(opts.TargetAltitudeM); V=double(opts.TargetAirspeedMps);
if strlength(string(opts.OutputRoot))==0
    outRoot=fullfile(local_resolve_path(projectRoot,opts.QualificationRoot),local_context_tag(H,V));
else
    outRoot=local_resolve_path(projectRoot,opts.OutputRoot);
end
if ~isfolder(outRoot), mkdir(outRoot); end
local_prepare_verified_state(projectRoot,sourceRoot,outRoot);

fprintf("\n============================================================\n");
fprintf("[V31.2-QUALIFY] frozen-learning H=%.3f V=%.3f\n",H,V);
fprintf("[V31.2-QUALIFY] source VERIFIED state: %s\n",sourceRoot);
fprintf("[V31.2-QUALIFY] no trim / ID / Plant rebuild / controller BO is permitted.\n");
fprintf("============================================================\n");

R=airdropx_auto_final_mission_validation( ...
    'ProjectRoot',projectRoot,'OutputRoot',outRoot, ...
    'ValidationSubdir','final_mission_validation', ...
    'TargetAltitudeM',H,'TargetAirspeedMps',V, ...
    'ReferenceMassKg',double(opts.ReferenceMassKg), ...
    'CargoMassKg',double(opts.CargoMassKg), ...
    'TotalDropCount',round(double(opts.TotalDropCount)), ...
    'BumplessTransitionEnabled',false, ...
    'TransitionMoveTransferScale',0.0, ...
    'TransitionIntegralTransferScale',0.0, ...
    'V31ContinuousControllerStateEnabled',true, ...
    'V31HeightGovernorEnabled',true, ...
    'V31HeightVzSlewRateMps2',double(opts.HeightVzSlewRateMps2), ...
    'V31HeightBiasFraction',double(opts.HeightBiasFraction), ...
    'V31HeightBiasLeak',double(opts.HeightBiasLeak));

summary=R.summary;
gate=Inf; if isfield(R,'mission_gate_ratio'), gate=double(R.mission_gate_ratio); end
pass=logical(R.mission_pass);

% Record the authoritative qualification result in the append-only v31 bank.
C=airdropx_v31_context('TargetAltitudeM',H,'TargetAirspeedMps',V, ...
    'ReferenceMassKg',opts.ReferenceMassKg,'CargoMassKg',opts.CargoMassKg, ...
    'TotalDropCount',opts.TotalDropCount);
row=local_mission_row(C,outRoot,summary,gate,pass,false,'v31_2_height_qualification');
airdropx_v31_knowledge_bank('Action','append_mission','ProjectRoot',projectRoot, ...
    'Root',opts.KnowledgeBankRoot,'Row',row);

result=struct(); result.target_altitude_m=H; result.target_airspeed_mps=V;
result.source_verified_root=string(sourceRoot); result.output_root=string(outRoot);
result.mission_pass=pass; result.mission_gate_ratio=gate; result.summary=summary;
result.controller_state_policy="v31_2_single_channel_height_governor";
end

function row=local_mission_row(C,outRoot,S,gate,pass,hard,source)
row=table("v31_mission|"+local_stable_key(string(outRoot)+"|"+C.mission_signature+"|"+source), ...
    string(datetime('now')),"v31.2",string(source),C.mission_signature,C.physics_signature, ...
    C.target_altitude_m,C.target_airspeed_mps,C.reference_mass_kg,C.cargo_mass_kg,double(C.total_drop_count), ...
    string(outRoot),double(gate),logical(pass),logical(hard), ...
    local_num(S,'mission_h_rms_m'),local_num(S,'mission_h_max_abs_m'),local_num(S,'mission_h_drift_m'), ...
    local_num(S,'mission_Va_rms_mps'),local_num(S,'mission_vz_rms_mps'),local_num(S,'mission_q_rms_dps'), ...
    local_num(S,'tail_h_error_m'),local_num(S,'tail_vz_mps'),local_num(S,'tail_q_dps'), ...
    logical(isfinite(gate)&&~hard),"v31_2_single_channel_height_governor", ...
    'VariableNames',{'record_key','timestamp','source_version','source','mission_signature','physics_signature', ...
    'target_altitude_m','target_airspeed_mps','reference_mass_kg','cargo_mass_kg','total_drop_count', ...
    'output_root','mission_gate_ratio','mission_pass','hard_fail','mission_h_rms_m','mission_h_max_abs_m', ...
    'mission_h_drift_m','mission_Va_rms_mps','mission_vz_rms_mps','mission_q_rms_dps', ...
    'tail_h_error_m','tail_vz_mps','tail_q_dps','valid_for_learning','controller_state_policy'});
end
function v=local_num(S,n)
v=NaN; try
    if istable(S) && ismember(n,S.Properties.VariableNames), v=double(S.(n)(1));
    elseif isstruct(S)&&isfield(S,n), v=double(S.(n)); end
catch, v=NaN; end
end
function local_prepare_verified_state(projectRoot,sourceRoot,destRoot)
marker=fullfile(destRoot,'V31_FROZEN_VERIFIED_STATE.txt');
srcCp=fullfile(sourceRoot,'airdropx_200m_cfg_checkpoint.mat');
srcMaster=fullfile(sourceRoot,'identified_plants_200m_master.mat');
if ~isfile(srcCp)||~isfile(srcMaster), error("AirdropX:V31:BadVerifiedSource","Source lacks checkpoint/master: %s",sourceRoot); end
S=load(srcCp,'checkpoint'); if ~isfield(S,'checkpoint'), error("AirdropX:V31:BadCheckpoint","checkpoint missing"); end
cp=S.checkpoint;
if ~isfield(cp,'status')||numel(cp.status)<5||~all(string(cp.status(1:5))=="verified")
    error("AirdropX:V31:SourceNotVerified","Qualification source cfg0..cfg4 are not all VERIFIED.");
end
copyfile(srcMaster,fullfile(destRoot,'identified_plants_200m_master.mat'),'f');
srcBank=local_cfg4_bank(projectRoot,sourceRoot,cp);
dstCfg4=fullfile(destRoot,'cfg4'); if ~isfolder(dstCfg4), mkdir(dstCfg4); end
dstBank=fullfile(dstCfg4,'best_mpc_bank_200m.mat'); copyfile(srcBank,dstBank,'f');
if ~isfield(cp,'best_bank_path')||numel(cp.best_bank_path)<5, cp.best_bank_path=strings(5,1); end
cp.best_bank_path(5)=string(dstBank); cp.all_verified=true;
cp.final_mission_attempted=false; cp.final_mission_pass=false; cp.final_mission_summary=table();
cp.final_mission_updated_at=""; cp.updated_at=string(datetime('now'));
checkpoint=cp; %#ok<NASGU>
save(fullfile(destRoot,'airdropx_200m_cfg_checkpoint.mat'),'checkpoint','-v7.3');
for f=["all_config_status.csv","physical_nominals_200m.csv"]
    q=fullfile(sourceRoot,f); if isfile(q), copyfile(q,fullfile(destRoot,f),'f'); end
end
fid=fopen(marker,'w'); if fid>=0, fprintf(fid,'source=%s\nprepared=%s\n',char(sourceRoot),char(datetime('now'))); fclose(fid); end
end
function bank=local_cfg4_bank(projectRoot,sourceRoot,cp)
candidates=strings(0,1);
if isfield(cp,'best_bank_path')&&numel(cp.best_bank_path)>=5, candidates(end+1)=string(cp.best_bank_path(5)); end %#ok<AGROW>
candidates(end+1)=string(fullfile(sourceRoot,'cfg4','best_mpc_bank_200m.mat'));
for i=1:numel(candidates)
    q=candidates(i); if strlength(q)==0, continue; end
    if isfile(q), bank=q; return; end
    q2=fullfile(projectRoot,q); if isfile(q2), bank=string(q2); return; end
end
error("AirdropX:V31:MissingCfg4Bank","Cannot resolve cfg4 combined bank from %s",sourceRoot);
end
function tag=local_context_tag(H,V), tag="H"+local_num_tag(H)+"_V"+local_num_tag(V); end
function s=local_num_tag(x), s=sprintf('%.3f',double(x)); s=strrep(s,'-','m'); s=strrep(s,'.','p'); end
function k=local_stable_key(s)
s=char(string(s)); z=0; for i=1:numel(s), z=mod(z*131+double(uint8(s(i))),2^31-1); end, k=string(sprintf('%08x',round(z))); end
function root=local_project_root(root), root=string(root); if strlength(root)==0, root=string(pwd); end, end
function p=local_resolve_path(projectRoot,p)
p=string(p); if strlength(p)==0, return; end
c=char(p); absPath=startsWith(c,'/')||startsWith(c,'\\')||~isempty(regexp(c,'^[A-Za-z]:[\\/]','once'));
if ~absPath, p=string(fullfile(projectRoot,p)); end
end
function opts=local_options(varargin)
opts=struct(); opts.ProjectRoot=""; opts.SourceVerifiedRoot=""; opts.OutputRoot="";
opts.QualificationRoot="matlab/results/mpc_auto_v31/qualification";
opts.KnowledgeBankRoot="matlab/results/mpc_auto_v31_knowledge_bank";
opts.TargetAltitudeM=100; opts.TargetAirspeedMps=50; opts.ReferenceMassKg=3423;
opts.CargoMassKg=300; opts.TotalDropCount=4;
opts.HeightVzSlewRateMps2=0.30; opts.HeightBiasFraction=0.70; opts.HeightBiasLeak=1.0;
if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin), n=string(varargin{i}); if ~isfield(opts,n), error("Unknown option: %s",n); end, opts.(n)=varargin{i+1}; end
end
