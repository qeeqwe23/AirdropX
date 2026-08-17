function T = airdropx_physics_mpc_mpcmove_preflight(varargin)
%AIRDROPX_PHYSICS_MPC_MPCMOVE_PREFLIGHT Verify all bank MPC objects before flight.
% This is a deterministic API/runtime smoke test only.  It does not tune MPC.
opts=local_options(varargin{:});root=local_root(opts.ProjectRoot);bank=local_resolve(root,opts.BankMat);
if ~isfile(bank),error('AirdropX:PhysicsMPC:MissingBank','Missing bank: %s',bank);end
S=load(bank,'v32_nodes');nodes=S.v32_nodes(:);
rows=cell(0,8);bad=false;
for n=1:numel(nodes)
    v=double(nodes(n).speed_mps);
    for cfg=1:5
        ok=false;its=NaN;qp="";msg="";mv1=NaN;mv2=NaN;
        try
            if numel(nodes(n).controllers)<cfg || numel(nodes(n).trim_bank)<cfg
                error('AirdropX:PhysicsMPC:IncompleteRuntimeBank','V%.1f cfg%d is missing controller/trim entry.',v,cfg-1);
            end
            trim=nodes(n).trim_bank(cfg);
            if ~isfield(trim,'throttle_cmd') || ~isfinite(double(trim.throttle_cmd))
                error('AirdropX:PhysicsMPC:BadRuntimeTrim','V%.1f cfg%d has invalid throttle nominal.',v,cfg-1);
            end
            pe=NaN;try,x=double(nodes(n).mpc_meta.physical_elevator_nominals(:));if cfg<=numel(x),pe=x(cfg);end,catch,end
            if ~isfinite(pe),error('AirdropX:PhysicsMPC:BadRuntimeTrim','V%.1f cfg%d has invalid physical elevator nominal.',v,cfg-1);end
            ctrl=nodes(n).controllers{cfg};
            st=mpcstate(ctrl);st.LastMove=zeros(2,1);
            ym=zeros(4,1);
            % r must be 1-by-Ny for one constant reference.
            r=zeros(1,4);
            [mv,info]=mpcmove(ctrl,st,ym,r);
            mv=double(mv(:));if numel(mv)>=2,mv1=mv(1);mv2=mv(2);end
            if isstruct(info)&&isfield(info,'Iterations'),its=double(info.Iterations);end
            if isstruct(info)&&isfield(info,'QPCode'),qp=string(info.QPCode);end
            ok=numel(mv)>=2&&all(isfinite(mv(1:2)))&&(~isfinite(its)||its>0)&&(strlength(qp)==0||lower(qp)=="feasible");
            if ~ok,msg="nonfeasible_or_nonfinite";end
        catch ME
            msg=string(ME.identifier)+" | "+string(ME.message);
        end
        rows(end+1,:)={v,cfg-1,ok,its,qp,mv1,mv2,msg}; %#ok<AGROW>
        if ~ok,bad=true;end
    end
end
T=cell2table(rows,'VariableNames',{'speed_mps','cfg_id','pass','iterations','qp_code','mv1','mv2','message'});
out=local_resolve(root,opts.OutputRoot);if ~isfolder(out),mkdir(out);end
writetable(T,fullfile(out,'mpcmove_preflight.csv'));
fprintf('[PHYS-MPC CL1.4] mpcmove/bank preflight: %d/%d PASS\n',sum(T.pass),height(T));
if bad
    disp(T(~T.pass,:));
    error('AirdropX:PhysicsMPC:MpcmovePreflightFailed','At least one stored MPC object cannot execute mpcmove. See mpcmove_preflight.csv.');
end
end
function p=local_resolve(root,x),p=char(string(x));if isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),p=fullfile(root,p);end,end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.BankMat="matlab/results/mpc_physics_v1/airdropx_physics_mpc_bank.mat";opts.OutputRoot="matlab/results/mpc_physics_v1/fixed_stability_cl11";
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
