function result = airdropx_v31_import_legacy(varargin)
%AIRDROPX_V31_IMPORT_LEGACY Import v29/v30 knowledge into v31 append-only bank.
%
% This importer is idempotent.  It never edits or deletes legacy files.
% Target altitude is retained for audit/mission history, but v31 physical
% signatures used for Plant/controller transfer deliberately exclude altitude.

opts=local_options(varargin{:});
projectRoot=local_project_root(opts.ProjectRoot);
kbRoot=local_resolve_path(projectRoot,opts.KnowledgeBankRoot);
legacyLearning=local_resolve_path(projectRoot,opts.LegacyLearningBankRoot);
legacyPlant=local_resolve_path(projectRoot,opts.LegacyPlantBankRoot);
legacyEnvelope=local_resolve_path(projectRoot,opts.LegacyEnvelopeRoot);
airdropx_v31_knowledge_bank('Action','init','ProjectRoot',projectRoot,'Root',kbRoot);
% Seed v31-owned low-level execution banks exactly once. After this copy,
% v31 writes only to these private backends; the old global banks are read-only.
backendLearning=fullfile(kbRoot,'backend_learning'); if ~isfolder(backendLearning), mkdir(backendLearning); end
backendPlant=fullfile(kbRoot,'backend_plant'); if ~isfolder(backendPlant), mkdir(backendPlant); end
local_seed_verified_backend(fullfile(legacyLearning,'verified_controllers.csv'),fullfile(backendLearning,'verified_controllers.csv'));
local_seed_file(fullfile(legacyLearning,'evaluations.csv'),fullfile(backendLearning,'evaluations.csv'));
local_seed_file(fullfile(legacyPlant,'plant_context_index.csv'),fullfile(backendPlant,'plant_context_index.csv'));
sourceMarker=fullfile(kbRoot,"V31_IMPORT_"+local_stable_key(legacyLearning+"|"+legacyPlant+"|"+legacyEnvelope)+".txt");
if logical(opts.SkipIfMarkerExists) && isfile(sourceMarker)
    result=struct('knowledge_bank_root',string(kbRoot),'backend_learning_root',string(backendLearning), ...
        'backend_plant_root',string(backendPlant),'physics_seen',0,'controller_seen',0,'mission_seen',0, ...
        'marker',string(sourceMarker),'skipped',true);
    return;
end

nPhysics=0; nController=0; nMission=0;
Prows=table(); Crows=table(); Mrows=table();

% -------------------------------------------------------------------------
% Physics models.
% -------------------------------------------------------------------------
plantFile=fullfile(legacyPlant,'plant_context_index.csv');
if isfile(plantFile)
    T=local_read(plantFile);
    for i=1:height(T)
        V=local_num_at(T,'target_airspeed_mps',i,NaN);
        H=local_num_at(T,'target_altitude_m',i,NaN);
        refM=local_num_at(T,'reference_mass_kg',i,3423);
        cargo=local_num_at(T,'cargo_mass_kg',i,300);
        nDrop=round(local_num_at(T,'total_drop_count',i,4));
        C=airdropx_v31_context('TargetAltitudeM',H,'TargetAirspeedMps',V, ...
            'ReferenceMassKg',refM,'CargoMassKg',cargo,'TotalDropCount',nDrop);
        outputRoot=local_str_at(T,'output_root',i,'');
        masterMat=local_str_at(T,'master_mat',i,'');
        valid=local_bool_at(T,'plant_valid',i,false) && strlength(masterMat)>0;
        row=table( ...
            "legacy_plant|"+local_stable_key(outputRoot+"|"+masterMat), ...
            local_str_at(T,'timestamp',i,string(datetime('now'))),"v30","legacy_plant_bank", ...
            C.physics_signature,V,refM,cargo,double(nDrop),H,outputRoot,masterMat, ...
            local_bool_at(T,'plant_valid',i,false),local_bool_at(T,'mission_pass',i,false),logical(valid), ...
            'VariableNames',local_physics_names());
        if isempty(Prows), Prows=row; else, Prows=[Prows;row]; end %#ok<AGROW>
        nPhysics=nPhysics+1;
    end
end

if ~isempty(Prows), airdropx_v31_knowledge_bank('Action','append_physics','ProjectRoot',projectRoot,'Root',kbRoot,'Row',Prows); end

% -------------------------------------------------------------------------
% Controller evaluations and verified controllers.
% -------------------------------------------------------------------------
for spec={ {'evaluations.csv',false}, {'verified_controllers.csv',true} }
    file=fullfile(legacyLearning,spec{1}{1}); verifiedSource=spec{1}{2};
    if ~isfile(file), continue; end
    T=local_read(file);
    for i=1:height(T)
        H=local_num_at(T,'target_altitude_m',i,NaN); V=local_num_at(T,'target_airspeed_mps',i,NaN);
        refM=local_num_at(T,'reference_mass_kg',i,3423); cargo=local_num_at(T,'cargo_mass_kg',i,300);
        nDrop=round(local_num_at(T,'total_drop_count',i,4)); cfg=round(local_num_at(T,'config_id',i,0));
        mass=local_num_at(T,'estimated_mass_kg',i,refM-cfg*cargo);
        cg=local_num_at(T,'cg_x_m',i,NaN); cgKnown=local_bool_at(T,'cg_known',i,isfinite(cg));
        if ~cgKnown, cg=NaN; end
        C=airdropx_v31_context('TargetAltitudeM',H,'TargetAirspeedMps',V, ...
            'ReferenceMassKg',refM,'CargoMassKg',cargo,'TotalDropCount',nDrop, ...
            'ConfigId',cfg,'EstimatedMassKg',mass,'CgXM',cg);
        gate=local_num_at(T,'gate_ratio',i,Inf); hard=local_bool_at(T,'hard_fail',i,false);
        formal=local_bool_at(T,'formal_pass',i,verifiedSource);
        % A verified legacy row remains valid knowledge even when an older CSV
        % did not persist gate_ratio. Evaluation-only rows still require a
        % finite measured gate and a non-hard-fail certification.
        valid=(verifiedSource || formal) && ~hard;
        if ~verifiedSource && ~formal, valid=isfinite(gate) && gate>0 && ~hard; end
        legacyCtx=local_str_at(T,'context_signature',i,'');
        cand=local_str_at(T,'candidate_signature',i,sprintf('row%d',i));
        srcRoot=local_str_at(T,'source_output_root',i,'');
        recordKey="legacy_ctrl|"+local_stable_key(string(spec{1}{1})+"|"+legacyCtx+"|"+cand+"|"+srcRoot);
        row=table(recordKey,local_str_at(T,'timestamp',i,string(datetime('now'))),"v30", ...
            string(spec{1}{1}),C.mission_signature,C.controller_physical_signature,legacyCtx, ...
            double(cfg),H,V,refM,cargo,mass,double(C.cg_x_m),logical(C.cg_known), ...
            double(C.payload_remaining_kg),double(C.drop_fraction), ...
            local_num_at(T,'Np',i,NaN),local_num_at(T,'Nc',i,NaN),local_num_at(T,'Wh',i,NaN), ...
            local_num_at(T,'Wvz',i,NaN),local_num_at(T,'Wq',i,NaN),local_num_at(T,'RateScale',i,NaN), ...
            local_num_at(T,'Authority',i,NaN),local_num_at(T,'HeightToVzGain',i,NaN), ...
            local_num_at(T,'HeightIntegralGain',i,NaN),local_num_at(T,'HeightVzLimit',i,NaN), ...
            gate,logical(hard),logical(formal),logical(verifiedSource || formal),logical(valid),srcRoot, ...
            'VariableNames',local_controller_names());
        if isempty(Crows), Crows=row; else, Crows=[Crows;row]; end %#ok<AGROW>
        nController=nController+1;
    end
end

if ~isempty(Crows), airdropx_v31_knowledge_bank('Action','append_controller','ProjectRoot',projectRoot,'Root',kbRoot,'Row',Crows); end

% -------------------------------------------------------------------------
% Final mission results.  These are authoritative mission-level outcomes;
% stale/partial controller eval folders do not change their classification.
% -------------------------------------------------------------------------
missionFiles=dir(fullfile(legacyEnvelope,'missions','**','final_mission_summary.csv'));
anchorCandidates=[ ...
    dir(fullfile(projectRoot,'matlab','results','mpc_auto_200m_all_cfg_v16','final_mission_validation','final_mission_summary.csv')); ...
    dir(fullfile(projectRoot,'matlab','results','mpc_auto_200m_all_cfg_v16','**','final_mission_summary.csv'))];
missionFiles=[missionFiles;anchorCandidates]; %#ok<AGROW>
seen=strings(0,1);
for i=1:numel(missionFiles)
    file=string(fullfile(missionFiles(i).folder,missionFiles(i).name));
    if any(seen==file), continue; end
    seen(end+1,1)=file; %#ok<AGROW>
    try, S=local_read(file); catch, continue; end
    if isempty(S), continue; end
    H=local_num_at(S,'target_altitude_m',1,local_guess_h(file));
    V=local_num_at(S,'target_airspeed_mps',1,local_guess_v(file));
    if ~isfinite(H) || ~isfinite(V), continue; end
    refM=double(opts.ReferenceMassKg); cargo=double(opts.CargoMassKg); nDrop=round(double(opts.TotalDropCount));
    C=airdropx_v31_context('TargetAltitudeM',H,'TargetAirspeedMps',V, ...
        'ReferenceMassKg',refM,'CargoMassKg',cargo,'TotalDropCount',nDrop);
    gate=local_num_at(S,'mission_gate_ratio',1,Inf); pass=local_bool_at(S,'mission_pass',1,false);
    hard=local_bool_at(S,'hard_fail',1,false); outputRoot=string(fileparts(fileparts(file)));
    row=table("legacy_mission|"+local_stable_key(file),string(missionFiles(i).date),"v30", ...
        "legacy_final_mission",C.mission_signature,C.physics_signature,H,V,refM,cargo,double(nDrop), ...
        outputRoot,gate,logical(pass),logical(hard), ...
        local_num_at(S,'mission_h_rms_m',1,NaN),local_num_at(S,'mission_h_max_abs_m',1,NaN), ...
        local_num_at(S,'mission_h_drift_m',1,NaN),local_num_at(S,'mission_Va_rms_mps',1,NaN), ...
        local_num_at(S,'mission_vz_rms_mps',1,NaN),local_num_at(S,'mission_q_rms_dps',1,NaN), ...
        local_num_at(S,'tail_h_error_m',1,NaN),local_num_at(S,'tail_vz_mps',1,NaN), ...
        local_num_at(S,'tail_q_dps',1,NaN),logical(isfinite(gate)&&~hard),"legacy_v30", ...
        'VariableNames',local_mission_names());
    if isempty(Mrows), Mrows=row; else, Mrows=[Mrows;row]; end %#ok<AGROW>
    nMission=nMission+1;
end

if ~isempty(Mrows), airdropx_v31_knowledge_bank('Action','append_mission','ProjectRoot',projectRoot,'Root',kbRoot,'Row',Mrows); end

marker=sourceMarker;
fid=fopen(marker,'w'); if fid>=0
    fprintf(fid,'Imported at %s\nphysics=%d\ncontroller=%d\nmission=%d\n',char(datetime('now')),nPhysics,nController,nMission);
    fclose(fid);
end
result=struct('knowledge_bank_root',string(kbRoot),'backend_learning_root',string(backendLearning), ...
    'backend_plant_root',string(backendPlant),'physics_seen',nPhysics, ...
    'controller_seen',nController,'mission_seen',nMission,'marker',string(marker),'skipped',false);
end

function names=local_physics_names()
names={'record_key','timestamp','source_version','source','physics_signature','target_airspeed_mps', ...
    'reference_mass_kg','cargo_mass_kg','total_drop_count','source_altitude_m','output_root','master_mat', ...
    'plant_valid','mission_pass','valid_for_learning'};
end
function names=local_controller_names()
names={'record_key','timestamp','source_version','source','mission_signature','physical_signature', ...
    'legacy_context_signature','config_id','target_altitude_m','target_airspeed_mps','reference_mass_kg', ...
    'cargo_mass_kg','estimated_mass_kg','cg_x_m','cg_known','payload_remaining_kg','drop_fraction', ...
    'Np','Nc','Wh','Wvz','Wq','RateScale','Authority','HeightToVzGain','HeightIntegralGain','HeightVzLimit', ...
    'gate_ratio','hard_fail','formal_pass','verified','valid_for_learning','source_output_root'};
end
function names=local_mission_names()
names={'record_key','timestamp','source_version','source','mission_signature','physics_signature', ...
    'target_altitude_m','target_airspeed_mps','reference_mass_kg','cargo_mass_kg','total_drop_count', ...
    'output_root','mission_gate_ratio','mission_pass','hard_fail','mission_h_rms_m','mission_h_max_abs_m', ...
    'mission_h_drift_m','mission_Va_rms_mps','mission_vz_rms_mps','mission_q_rms_dps', ...
    'tail_h_error_m','tail_vz_mps','tail_q_dps','valid_for_learning','controller_state_policy'};
end

function local_seed_verified_backend(src,dst)
% The audit bank retains every legacy VERIFIED row, but the active v31 backend
% keeps one representative per PHYSICAL controller context (H excluded). This
% prevents old altitude-specific tuning duplicates from masquerading as distinct
% aerodynamic contexts in transfer/GPR.
if isfile(dst) || ~isfile(src), return; end
try
    T=local_read(src);
    if isempty(T), copyfile(src,dst,'f'); return; end
    key=strings(height(T),1); score=inf(height(T),1);
    for i=1:height(T)
        V=local_num_at(T,'target_airspeed_mps',i,NaN); cfg=round(local_num_at(T,'config_id',i,0));
        refM=local_num_at(T,'reference_mass_kg',i,3423); cargo=local_num_at(T,'cargo_mass_kg',i,300);
        mass=local_num_at(T,'estimated_mass_kg',i,refM-cfg*cargo);
        cg=local_num_at(T,'cg_x_m',i,NaN); cgKnown=local_bool_at(T,'cg_known',i,isfinite(cg));
        if ~cgKnown, cg=0; end
        key(i)=sprintf('V%.6f|M%.4f|C%.4f|CG%.6f|CGk%d|cfg%d',V,mass,cargo,cg,cgKnown,cfg);
        g=local_num_at(T,'gate_ratio',i,Inf); if isfinite(g)&&g>0, score(i)=g; end
    end
    [uk,~,grp]=unique(key,'stable'); keep=false(height(T),1);
    for j=1:numel(uk)
        idx=find(grp==j); [~,q]=min(score(idx)); if isempty(q)||~isfinite(score(idx(q))), q=1; end
        keep(idx(q))=true;
    end
    writetable(T(keep,:),dst);
catch ME
    warning("AirdropX:V31:SeedVerifiedBackend","Could not curate verified backend; falling back to raw copy: %s",ME.message);
    try, copyfile(src,dst,'f'); catch, end
end
end
function local_seed_file(src,dst)
if isfile(dst) || ~isfile(src), return; end
try, copyfile(src,dst,'f'); catch ME, warning("AirdropX:V31:SeedBackend","Could not seed %s: %s",dst,ME.message); end
end
function T=local_read(file)
try, T=readtable(file,'TextType','string','VariableNamingRule','preserve'); catch, T=readtable(file); end
end
function v=local_num_at(T,n,i,d)
if ~ismember(n,T.Properties.VariableNames), v=d; return; end
x=T.(n); if isnumeric(x)||islogical(x), v=double(x(i)); else, v=str2double(string(x(i))); end
if ~isfinite(v), v=d; end
end
function v=local_str_at(T,n,i,d)
if ~ismember(n,T.Properties.VariableNames), v=string(d); else, v=string(T.(n)(i)); end
end
function v=local_bool_at(T,n,i,d)
if ~ismember(n,T.Properties.VariableNames), v=logical(d); return; end
x=T.(n); if islogical(x), v=x(i); elseif isnumeric(x), v=x(i)~=0; else, s=lower(strtrim(string(x(i)))); v=s=="true"||s=="1"||s=="yes"; end
end
function k=local_stable_key(s)
s=char(string(s)); z=0;
for j=1:numel(s), z=mod(z*131+double(uint8(s(j))),2^31-1); end
k=string(sprintf('%08x',round(z)));
end
function H=local_guess_h(file)
t=regexp(char(file),'H([0-9]+)p([0-9]+)_V','tokens','once');
if isempty(t), H=NaN; else, H=str2double(string(t{1})+"."+string(t{2})); end
end
function V=local_guess_v(file)
t=regexp(char(file),'_V([0-9]+)p([0-9]+)','tokens','once');
if isempty(t), V=NaN; else, V=str2double(string(t{1})+"."+string(t{2})); end
end
function root=local_project_root(root), root=string(root); if strlength(root)==0, root=string(pwd); end, end
function p=local_resolve_path(projectRoot,p)
p=string(p); if strlength(p)==0, return; end
c=char(p); absPath=startsWith(c,'/')||startsWith(c,'\\')||~isempty(regexp(c,'^[A-Za-z]:[\\/]','once'));
if ~absPath, p=string(fullfile(projectRoot,p)); end
end
function opts=local_options(varargin)
opts=struct(); opts.ProjectRoot=""; opts.KnowledgeBankRoot="matlab/results/mpc_auto_v31_knowledge_bank";
opts.LegacyLearningBankRoot="matlab/results/mpc_auto_global_learning_bank";
opts.LegacyPlantBankRoot="matlab/results/mpc_auto_global_plant_bank";
opts.LegacyEnvelopeRoot="matlab/results/mpc_auto_flight_envelope_v30";
opts.ReferenceMassKg=3423; opts.CargoMassKg=300; opts.TotalDropCount=4; opts.SkipIfMarkerExists=false;
if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin), n=string(varargin{i}); if ~isfield(opts,n), error("Unknown option: %s",n); end, opts.(n)=varargin{i+1}; end
end
