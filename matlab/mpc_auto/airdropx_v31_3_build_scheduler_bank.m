function result = airdropx_v31_3_build_scheduler_bank(varargin)
%AIRDROPX_V31_3_BUILD_SCHEDULER_BANK Build continuous speed-scheduling bank.
%
% A speed node is accepted only when its reference-altitude context has all
% cfg0..cfg4 controllers VERIFIED.  Each node keeps its own MPC objects, trim
% bank and learned scalar parameters.  Runtime v31.3 evaluates the two nodes
% bracketing measured airspeed and blends ABSOLUTE physical actuator commands;
% no hard speed-controller switch is performed.
%
% The builder never trains or modifies a controller.

opts=local_options(varargin{:});
projectRoot=local_project_root(opts.ProjectRoot);
contextRoot=local_resolve(projectRoot,opts.ContextRoot);
outFile=local_resolve(projectRoot,opts.OutputFile);
if ~isfolder(contextRoot)
    error("AirdropX:V31_3:MissingContextRoot","Reference-context root not found: %s",contextRoot);
end
D=dir(fullfile(contextRoot,'H*_V*')); D=D([D.isdir]);
nodes=struct([]); speeds=[]; sourceRoots=strings(0,1);
for i=1:numel(D)
    root=fullfile(D(i).folder,D(i).name);
    [ok,V,node]=local_read_node(projectRoot,root,opts);
    if ~ok, continue; end
    speeds(end+1,1)=V; %#ok<AGROW>
    if isempty(nodes), nodes=node; else, nodes(end+1)=node; end %#ok<AGROW>
    sourceRoots(end+1,1)=string(root); %#ok<AGROW>
end
if isempty(speeds)
    error("AirdropX:V31_3:NoVerifiedSpeedNodes", ...
        "No reference-speed context has cfg0..cfg4 all VERIFIED under %s.",contextRoot);
end
[speeds,ix]=sort(speeds); nodes=nodes(ix); sourceRoots=sourceRoots(ix);
% Duplicate speed contexts are audit history, not separate scheduling nodes.
[uniqueV,~,g]=unique(round(speeds,9),'stable');
if numel(uniqueV)<numel(speeds)
    keep=zeros(numel(uniqueV),1);
    for k=1:numel(uniqueV)
        candidates=find(g==k); keep(k)=candidates(end); % latest directory order
    end
    speeds=speeds(keep); nodes=nodes(keep); sourceRoots=sourceRoots(keep);
end
scheduler=struct();
scheduler.schema_version="v31.3";
scheduler.architecture_signature="v31p3_dynamic_reference_continuous_speed_scheduler";
scheduler.created_at=string(datetime('now'));
scheduler.reference_altitude_m=double(opts.ReferenceAltitudeM);
scheduler.speed_nodes=double(speeds(:));
scheduler.nodes=nodes;
scheduler.source_context_roots=sourceRoots;
scheduler.interpolation="linear_physical_command_blend";
scheduler.scheduling_variable="measured_airspeed_mps";
scheduler.minimum_nodes_for_runtime_blend=2;
folder=fileparts(outFile); if ~isfolder(folder), mkdir(folder); end
save(outFile,'scheduler','-v7.3');
T=table(double(speeds(:)),sourceRoots,'VariableNames',{'airspeed_node_mps','source_context_root'});
writetable(T,fullfile(folder,'v31_3_scheduler_nodes.csv'));
result=struct('scheduler_bank_mat',string(outFile),'node_count',numel(speeds), ...
    'speed_nodes',double(speeds(:)),'runtime_blend_ready',numel(speeds)>=2);
fprintf("[V31.3-SCHED] built %d verified speed node(s): %s\n",numel(speeds),mat2str(speeds(:).'));
if numel(speeds)<2
    fprintf("[V31.3-SCHED] one node is valid for fallback, but continuous speed interpolation activates after >=2 verified nodes.\n");
end
end

function [ok,V,node]=local_read_node(projectRoot,root,opts)
ok=false; V=NaN; node=struct(); cpFile=fullfile(root,'airdropx_200m_cfg_checkpoint.mat');
if ~isfile(cpFile), return; end
try
    S=load(cpFile,'checkpoint'); cp=S.checkpoint;
    if ~isfield(cp,'status')||numel(cp.status)<5||~all(string(cp.status(1:5))=="verified"), return; end
    V=local_context_speed(root);
    if ~isfinite(V), return; end
    if isfield(cp,'target_altitude_m') && isfinite(double(cp.target_altitude_m)) && ...
            abs(double(cp.target_altitude_m)-double(opts.ReferenceAltitudeM))>1e-6, return; end
    bankPath=local_bank_path(projectRoot,root,cp);
    B=load(bankPath,'controllers','trim_bank','mpc_meta');
    if ~isfield(B,'controllers')||numel(B.controllers)<5||any(cellfun(@isempty,B.controllers(1:5))), return; end
    node=struct(); node.airspeed_mps=V; node.source_root=string(root); node.bank_path=string(bankPath);
    node.controllers=B.controllers(:); node.trim_bank=B.trim_bank(:);
    if isfield(B,'mpc_meta'), node.mpc_meta=B.mpc_meta; else, node.mpc_meta=struct(); end
    node.authority_by_cfg=NaN(5,1); node.height_gain_by_cfg=NaN(5,1);
    node.height_integral_by_cfg=NaN(5,1); node.height_vz_limit_by_cfg=NaN(5,1);
    for k=1:5
        if isfield(cp,'best_candidate')&&numel(cp.best_candidate)>=k&&~isempty(cp.best_candidate{k})
            c=cp.best_candidate{k};
            node.authority_by_cfg(k)=local_cand(c,["Authority","authority"],1.0);
            node.height_gain_by_cfg(k)=local_cand(c,["HeightToVzGain","Kp","height_to_vz_gain"],0.0);
            node.height_integral_by_cfg(k)=local_cand(c,["HeightIntegralGain","Ki","height_integral_gain"],0.0);
            node.height_vz_limit_by_cfg(k)=local_cand(c,["HeightVzLimit","VzLimit","height_vz_limit"],0.8);
        end
    end
    ok=true;
catch
    ok=false;
end
end
function p=local_bank_path(projectRoot,root,cp)
p="";
if isfield(cp,'best_bank_path')&&numel(cp.best_bank_path)>=5, p=string(cp.best_bank_path(5)); end
cands=[p;string(fullfile(root,'cfg4','best_mpc_bank_200m.mat'))];
for q=cands.'
    if strlength(q)==0, continue; end
    if isfile(q), p=char(q); return; end
    q2=fullfile(projectRoot,char(q)); if isfile(q2), p=q2; return; end
end
error("AirdropX:V31_3:MissingCombinedBank","No cfg4 combined bank under %s",root);
end
function V=local_context_speed(root)
[~,name]=fileparts(root); tok=regexp(name,'_V([mp0-9]+)$','tokens','once'); V=NaN;
if isempty(tok), return; end
s=strrep(tok{1},'p','.'); s=strrep(s,'m','-'); V=str2double(s);
end
function x=local_cand(c,names,fallback)
x=fallback;
for n=names
    try
        if isstruct(c)&&isfield(c,char(n)), v=double(c.(char(n))); elseif istable(c)&&ismember(char(n),c.Properties.VariableNames),v=double(c.(char(n))(1)); else,continue;end
        if isscalar(v)&&isfinite(v),x=v;return;end
    catch
    end
end
end
function p=local_project_root(x)
if strlength(string(x))>0,p=char(string(x));else,this=fileparts(mfilename('fullpath'));p=fileparts(fileparts(this));end
end
function p=local_resolve(projectRoot,x)
x=char(string(x)); if isempty(x),p=projectRoot;elseif isfolder(x)||isfile(x)||(~isempty(regexp(x,'^[A-Za-z]:[\\/]','once'))),p=x;else,p=fullfile(projectRoot,x);end
end
function opts=local_options(varargin)
opts.ProjectRoot=""; opts.ContextRoot="matlab/results/mpc_auto_v31/reference_contexts";
opts.OutputFile="matlab/results/mpc_auto_v31/v31_3_speed_scheduler/v31_3_speed_scheduler_bank.mat";
opts.ReferenceAltitudeM=200.0;
if mod(numel(varargin),2)~=0,error("Options must be name-value pairs.");end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error("Unknown option: %s",n);end,opts.(n)=varargin{i+1};end
end
