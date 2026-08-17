function result = airdropx_v31_knowledge_bank(varargin)
%AIRDROPX_V31_KNOWLEDGE_BANK Append-only v31 knowledge store.
%
% Actions:
%   init            - create the bank and empty tables.
%   append_physics  - append one physics-model record.
%   append_controller - append one controller observation/verified record.
%   append_mission  - append one mission result.
%   append_state    - append one state-machine event.
%   report          - return/read all tables.
%
% Raw records are never deleted.  Infrastructure-invalid observations remain
% present with valid_for_learning=false.  This avoids the v30 pattern of
% deleting/restoring CSV rows and makes every learning decision auditable.

opts = local_options(varargin{:});
root = local_resolve_path(local_project_root(opts.ProjectRoot),opts.Root);
if ~isfolder(root), mkdir(root); end
files = local_files(root);
local_ensure_files(files);
action = lower(string(opts.Action));

switch action
    case "init"
        result = local_report(root,files);
    case "append_physics"
        row = opts.Row;
        local_append(files.physics,row,local_physics_schema(),["record_key"]);
        result = local_report(root,files);
    case "append_controller"
        row = opts.Row;
        local_append(files.controller,row,local_controller_schema(),["record_key"]);
        result = local_report(root,files);
    case "append_mission"
        row = opts.Row;
        local_append(files.mission,row,local_mission_schema(),["record_key"]);
        result = local_report(root,files);
    case "append_state"
        row = opts.Row;
        local_append(files.state,row,local_state_schema(),["record_key"]);
        result = local_report(root,files);
    case "report"
        result = local_report(root,files);
    otherwise
        error("AirdropX:V31:BadKnowledgeAction","Unknown Action: %s",opts.Action);
end
end

function result=local_report(root,files)
result=struct();
result.root=string(root);
result.physics_models=local_read(files.physics,local_physics_schema());
result.controller_records=local_read(files.controller,local_controller_schema());
result.mission_results=local_read(files.mission,local_mission_schema());
result.state_events=local_read(files.state,local_state_schema());
result.counts=struct( ...
    'physics',height(result.physics_models), ...
    'controller',height(result.controller_records), ...
    'mission',height(result.mission_results), ...
    'state',height(result.state_events));
end

function local_ensure_files(files)
if ~isfile(files.physics), writetable(local_empty(local_physics_schema()),files.physics); end
if ~isfile(files.controller), writetable(local_empty(local_controller_schema()),files.controller); end
if ~isfile(files.mission), writetable(local_empty(local_mission_schema()),files.mission); end
if ~isfile(files.state), writetable(local_empty(local_state_schema()),files.state); end
end

function local_append(file,row,schema,keyNames)
if isempty(row), return; end
if ~istable(row), error("AirdropX:V31:KnowledgeRow","Row must be a table."); end
row=local_conform(row,schema);
T=local_read(file,schema);
T=[T;row]; %#ok<AGROW>
% Append-only semantics for raw information, but exact duplicate imports are
% collapsed by stable record_key so rerunning the importer is idempotent.
if ~isempty(keyNames) && all(ismember(keyNames,string(T.Properties.VariableNames)))
    key=strings(height(T),1);
    for k=1:numel(keyNames)
        key=key+"|"+string(T.(char(keyNames(k))));
    end
    [~,ia]=unique(key,'stable');
    T=T(sort(ia),:);
end
writetable(T,file);
end

function T=local_read(file,schema)
if ~isfile(file), T=local_empty(schema); return; end
try
    T=readtable(file,'TextType','string','VariableNamingRule','preserve');
catch
    T=readtable(file);
end
T=local_conform(T,schema);
end

function T=local_conform(T,schema)
names=string(fieldnames(schema));
for i=1:numel(names)
    n=char(names(i));
    if ~ismember(n,T.Properties.VariableNames)
        T.(n)=repmat(schema.(n),height(T),1);
    end
end
T=T(:,cellstr(names));
for i=1:numel(names)
    n=char(names(i)); proto=schema.(n);
    try
        if isstring(proto), T.(n)=string(T.(n));
        elseif islogical(proto), T.(n)=local_bool(T.(n));
        elseif isnumeric(proto), T.(n)=local_num(T.(n));
        end
    catch
    end
end
end

function T=local_empty(schema)
names=fieldnames(schema); vars=cell(1,numel(names));
for i=1:numel(names)
    p=schema.(names{i});
    if isstring(p), vars{i}=strings(0,1);
    elseif islogical(p), vars{i}=false(0,1);
    else, vars{i}=zeros(0,1);
    end
end
T=table(vars{:},'VariableNames',names);
end

function S=local_physics_schema()
S=struct();
S.record_key=""; S.timestamp=""; S.source_version=""; S.source="";
S.physics_signature=""; S.target_airspeed_mps=NaN; S.reference_mass_kg=NaN;
S.cargo_mass_kg=NaN; S.total_drop_count=NaN; S.source_altitude_m=NaN;
S.output_root=""; S.master_mat=""; S.plant_valid=false; S.mission_pass=false;
S.valid_for_learning=false;
end

function S=local_controller_schema()
S=struct();
S.record_key=""; S.timestamp=""; S.source_version=""; S.source="";
S.mission_signature=""; S.physical_signature=""; S.legacy_context_signature="";
S.config_id=NaN; S.target_altitude_m=NaN; S.target_airspeed_mps=NaN;
S.reference_mass_kg=NaN; S.cargo_mass_kg=NaN; S.estimated_mass_kg=NaN;
S.cg_x_m=NaN; S.cg_known=false; S.payload_remaining_kg=NaN; S.drop_fraction=NaN;
S.Np=NaN; S.Nc=NaN; S.Wh=NaN; S.Wvz=NaN; S.Wq=NaN; S.RateScale=NaN;
S.Authority=NaN; S.HeightToVzGain=NaN; S.HeightIntegralGain=NaN; S.HeightVzLimit=NaN;
S.gate_ratio=Inf; S.hard_fail=false; S.formal_pass=false; S.verified=false;
S.valid_for_learning=false; S.source_output_root="";
end

function S=local_mission_schema()
S=struct();
S.record_key=""; S.timestamp=""; S.source_version=""; S.source="";
S.mission_signature=""; S.physics_signature=""; S.target_altitude_m=NaN;
S.target_airspeed_mps=NaN; S.reference_mass_kg=NaN; S.cargo_mass_kg=NaN;
S.total_drop_count=NaN; S.output_root=""; S.mission_gate_ratio=Inf;
S.mission_pass=false; S.hard_fail=false; S.mission_h_rms_m=NaN;
S.mission_h_max_abs_m=NaN; S.mission_h_drift_m=NaN; S.mission_Va_rms_mps=NaN;
S.mission_vz_rms_mps=NaN; S.mission_q_rms_dps=NaN; S.tail_h_error_m=NaN;
S.tail_vz_mps=NaN; S.tail_q_dps=NaN; S.valid_for_learning=false;
S.controller_state_policy="";
end

function S=local_state_schema()
S=struct();
S.record_key=""; S.timestamp=""; S.context_root=""; S.mission_signature="";
S.state=""; S.failure_class=""; S.retry_level=""; S.physics_ready=false;
S.active_cfg=NaN; S.active_cfg_physics_ready=false; S.all_physics_ready=false;
S.controllers_ready=false; S.all_controllers_verified=false; S.mission_pass=false; S.gate_ratio=Inf;
S.next_action=""; S.note="";
end

function files=local_files(root)
files=struct();
files.physics=fullfile(root,"physics_models.csv");
files.controller=fullfile(root,"controller_records.csv");
files.mission=fullfile(root,"mission_results.csv");
files.state=fullfile(root,"state_events.csv");
end

function x=local_num(x)
if isnumeric(x)||islogical(x), x=double(x); else, x=str2double(string(x)); end
end
function x=local_bool(x)
if islogical(x), return; end
if isnumeric(x), x=x~=0; return; end
s=lower(strtrim(string(x))); x=s=="true"|s=="1"|s=="yes";
end
function root=local_project_root(root)
root=string(root); if strlength(root)==0, root=string(pwd); end
end
function p=local_resolve_path(projectRoot,p)
p=string(p); if strlength(p)==0, return; end
c=char(p); absPath=startsWith(c,'/')||startsWith(c,'\\')||~isempty(regexp(c,'^[A-Za-z]:[\\/]','once'));
if ~absPath, p=string(fullfile(projectRoot,p)); end
end
function opts=local_options(varargin)
opts=struct('Action',"report",'ProjectRoot',"", ...
    'Root',"matlab/results/mpc_auto_v31_knowledge_bank",'Row',table());
if mod(numel(varargin),2)~=0, error("Options must be name-value pairs."); end
for i=1:2:numel(varargin)
    n=string(varargin{i}); if ~isfield(opts,n), error("Unknown option: %s",n); end
    opts.(n)=varargin{i+1};
end
end
