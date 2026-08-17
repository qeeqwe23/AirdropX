function result = airdropx_urmpc_preflight(varargin)
%AIRDROPX_URMPC_PREFLIGHT Verify the built single-MPC bank and offline certificates.
o=local_options(varargin{:});root=local_root(o.ProjectRoot);bank=local_resolve(root,o.BankMat);out=local_resolve(root,o.BuildOutputRoot);
if ~isfile(bank),error('AirdropX:URMPC:MissingBank','Missing built bank: %s',bank);end
S=load(bank,'ur_mpc','ur_models','ur_meta');if ~isfield(S,'ur_mpc')||~isfield(S,'ur_models')||~isfield(S,'ur_meta'),error('AirdropX:URMPC:BadBank','Missing ur_mpc/ur_models/ur_meta.');end
stAudit=mpcstate(S.ur_mpc);
policy="integrators";try,policy=lower(string(S.ur_meta.input_disturbance_policy));catch,end
if policy=="static_white",expectedDistStates=0;else,expectedDistStates=2;end
if numel(stAudit.Disturbance)~=expectedDistStates
    error('AirdropX:URMPC:EstimatorStructureMismatch', ...
        'Input-disturbance policy %s expects %d persistent disturbance states, got %d.', ...
        policy,expectedDistStates,numel(stAudit.Disturbance));
end
try,[~,outCh]=getoutdist(S.ur_mpc);catch,outCh=[];end
if ~isempty(outCh),error('AirdropX:URMPC:EstimatorStructureMismatch','Output-disturbance integrators must be disabled.');end
if ~contains(string(S.ur_meta.version),'urmpc_v2_0'),error('AirdropX:URMPC:WrongVersion','Unexpected UR-MPC version: %s',string(S.ur_meta.version));end
vp=fullfile(out,'urmpc_vertex_preflight.csv');tp=fullfile(out,'urmpc_linear_drop_cert.csv');if ~isfile(vp)||~isfile(tp),error('AirdropX:URMPC:MissingCertificate','Build certificates are missing.');end
V=readtable(vp);T=readtable(tp);if ~all(logical(V.pass)),error('AirdropX:URMPC:VertexCertificateFailed','Dense scheduling-grid preflight is not all PASS.');end;if ~all(logical(T.pass)),error('AirdropX:URMPC:TransitionCertificateFailed','Linear cfg transition certificate is not all PASS.');end
% API sanity on the ONE stored controller.
m=S.ur_models(2,1);Nom=struct('X',m.x_nominal(:),'U',[m.u_nominal(:);0;0],'Y',m.x_nominal(:),'DX',zeros(5,1));
st=mpcstate(S.ur_mpc);st.LastMove=m.u_nominal(:);try,if numel(st.Plant)==numel(Nom.X),st.Plant=Nom.X(:);end,catch,end
[u,info]=mpcmoveAdaptive(S.ur_mpc,st,m.plant,Nom,m.x_nominal(:).',m.x_nominal(:).',[],mpcmoveopt);
if double(info.Iterations)<=0||numel(u)<2||any(~isfinite(u(1:2))),error('AirdropX:URMPC:ApiSmokeFailed','mpcmoveAdaptive API smoke failed.');end
nominalErr=max(abs(double(u(1:2))-double(m.u_nominal(:))));
if nominalErr>=0.03,error('AirdropX:URMPC:ApiSmokeFailed','mpcmoveAdaptive nominal equilibrium moved (max |dU|=%.6g).',nominalErr);end
fprintf('[UR-MPC v2.0.5] PREFLIGHT PASS: policy=%s; disturbance states=%d; vertices %d/%d; transitions %d/%d; API iterations=%d\n',policy,numel(stAudit.Disturbance),sum(V.pass),height(V),sum(T.pass),height(T),double(info.Iterations));
result=struct('pass',true,'vertex',V,'transitions',T,'meta',S.ur_meta,'api_iterations',double(info.Iterations));
end
function p=local_resolve(root,x),p=char(string(x));if isempty(regexp(p,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),p=fullfile(root,p);end,end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function o=local_options(varargin),o.ProjectRoot="";o.BankMat="matlab/results/mpc_physics_v1/unified_robust_mpc_v2/airdropx_unified_robust_mpc_bank.mat";o.BuildOutputRoot="matlab/results/mpc_physics_v1/unified_robust_mpc_v2";if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end;for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(o,n),error('Unknown option: %s',n);end,o.(n)=varargin{i+1};end,end
