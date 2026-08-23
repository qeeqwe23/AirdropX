function [vertex,meta]=airdropx_phys_interval_interpolate_vertex_v090(bank,H,V,cfgId,opts)
%AIRDROPX_PHYS_INTERVAL_INTERPOLATE_VERTEX_V090 Continuous HxV interpolation for one cfg.
arguments
    bank (1,1) struct
    H (1,1) double {mustBeFinite}
    V (1,1) double {mustBeFinite,mustBePositive}
    cfgId (1,1) double {mustBeInteger}
    opts.Horizon (1,1) double {mustBeInteger,mustBePositive} = 100
end
if H<20-1e-12 || H>200+1e-12, error("AirdropX:PhysMPC:IntervalHeight","H must be inside [20,200] m; no extrapolation is allowed."); end
if V<45-1e-12 || V>65+1e-12, error("AirdropX:PhysMPC:IntervalSpeed","V must be inside [45,65] m/s; no extrapolation is allowed."); end
if cfgId<0 || cfgId>4, error("AirdropX:PhysMPC:BadCfg","cfgId must be 0..4."); end
if ~isfield(bank,"vertices") || ~iscell(bank.vertices) || ~isfield(bank,"rows") || ~istable(bank.rows)
    error("AirdropX:PhysMPC:BadIntervalBank","Bank must contain vertices cell and rows table.");
end
r=bank.rows; need={'H_m','V_mps','cfg','fuel_scale','pass'};
for i=1:numel(need), if ~ismember(need{i},r.Properties.VariableNames), error("AirdropX:PhysMPC:BadIntervalBank","rows missing %s",need{i}); end, end
if ~ismember('usable',r.Properties.VariableNames), r.usable=logical(r.pass); end
hGrid=20:10:200; vGrid=[45 50 55 60 65];
[h0,h1,ah]=localBracket(H,hGrid); [v0,v1,av]=localBracket(V,vGrid);
coords=[h0 v0; h1 v0; h0 v1; h1 v1];
w=[(1-ah)*(1-av); ah*(1-av); (1-ah)*av; ah*av];
active=find(w>1e-14); coords=coords(active,:); w=w(active); w=w/sum(w);
source=cell(numel(active),1); rowIdx=zeros(numel(active),1); cert=false(numel(active),1); usable=false(numel(active),1);
for j=1:numel(active)
    mask=abs(r.H_m-coords(j,1))<=1e-9 & abs(r.V_mps-coords(j,2))<=1e-9 & r.cfg==cfgId & abs(r.fuel_scale-1)<=1e-12;
    k=find(mask); if numel(k)~=1, error("AirdropX:PhysMPC:IntervalCornerMissing","Expected one corner H=%.9g V=%.9g cfg=%d, found %d.",coords(j,1),coords(j,2),cfgId,numel(k)); end
    rowIdx(j)=k; cert(j)=logical(r.pass(k)); usable(j)=logical(r.usable(k));
    if ~usable(j), error("AirdropX:PhysMPC:IntervalCornerUnusable","Corner H=%.9g V=%.9g cfg=%d is not usable.",coords(j,1),coords(j,2),cfgId); end
    source{j}=bank.vertices{k};
    if isempty(source{j}), error("AirdropX:PhysMPC:IntervalCornerPayloadMissing","Corner payload is empty."); end
end
exactGrid=numel(source)==1 && abs(coords(1,1)-H)<=1e-12 && abs(coords(1,2)-V)<=1e-12;
if exactGrid
    vertex=source{1};
    meta=struct("version","v0.9.0 exact-node passthrough","H",H,"V",V,"cfg",cfgId,"exact_grid",true, ...
        "corner_HV",coords,"weights",w,"corner_row_indices",rowIdx,"corner_certified",cert,"corner_usable",usable, ...
        "source_certified_count",sum(cert),"rho",double(vertex.terminal.rho));
    return
end
Q0=double(source{1}.Q); R0=double(source{1}.R); ss0=double(source{1}.qrMeta.StateScale(:)); us0=double(source{1}.qrMeta.InputScale(:));
A=zeros(7); B=zeros(7,2); xref=zeros(7,1); uref=zeros(2,1);
maxQ=0; maxR=0; maxSS=0; maxUS=0;
for j=1:numel(source)
    s=source{j};
    maxQ=max(maxQ,norm(double(s.Q)-Q0,inf)); maxR=max(maxR,norm(double(s.R)-R0,inf));
    maxSS=max(maxSS,norm(double(s.qrMeta.StateScale(:))-ss0,inf)); maxUS=max(maxUS,norm(double(s.qrMeta.InputScale(:))-us0,inf));
    A=A+w(j)*double(s.lin.Ad); B=B+w(j)*double(s.lin.Bd); xref=xref+w(j)*double(s.trim.x(:)); uref=uref+w(j)*double(s.trim.u(:));
end
if max([maxQ maxR maxSS maxUS])>1e-12
    error("AirdropX:PhysMPC:IntervalControllerNotUnified","Interpolation corners do not share unified Q/R/scales.");
end
if any(~isfinite(A),'all') || any(~isfinite(B),'all') || any(~isfinite(xref)) || any(~isfinite(uref))
    error("AirdropX:PhysMPC:IntervalNonfinite","Interpolated A/B/trim contains nonfinite values.");
end
try
    [K,P,poles]=dlqr(A,B,Q0,R0);
catch ME
    error("AirdropX:PhysMPC:IntervalDAREFailed","Interpolated DARE failed at H=%.9g V=%.9g cfg=%d: %s",H,V,cfgId,ME.message);
end
rho=max(abs(poles));
if ~(isfinite(rho) && rho<1) || any(~isfinite(K),'all') || any(~isfinite(P),'all')
    error("AirdropX:PhysMPC:IntervalTerminalUnstable","Interpolated terminal model is not stable: rho=%.9g",rho);
end
if uref(1)<=-1 || uref(1)>=1 || uref(2)<=0 || uref(2)>=1
    error("AirdropX:PhysMPC:IntervalTrimInputBound","Interpolated trim input is not strictly inside hard bounds: u=[%.9g %.9g].",uref(1),uref(2));
end
vertex=source{1}; vertex.H=H; vertex.V=V; vertex.cfgId=cfgId; vertex.fuelScale=1;
vertex.Q=Q0; vertex.R=R0; vertex.qrMeta.StateScale=ss0; vertex.qrMeta.InputScale=us0;
vertex.p.cfgId=cfgId; vertex.p.fuelScale=1;
vertex.trim.x=xref; vertex.trim.u=uref; vertex.trim.xnext=xref; vertex.trim.pass=true;
if isfield(vertex.trim,'method'), vertex.trim.method="bilinear_HV_interpolated_v090"; end
if isfield(vertex.trim,'seed_source'), vertex.trim.seed_source="v082_usable_anchor_bank"; end
if isfield(vertex.trim,'diag')
    diag0=vertex.trim.diag; names={'mass_kg','cg_x_m','Iyy_kgm2'};
    for ii=1:numel(names)
        fn=names{ii}; vals=zeros(numel(source),1); ok=true;
        for j=1:numel(source), if ~isfield(source{j}.trim.diag,fn), ok=false; break; end; vals(j)=double(source{j}.trim.diag.(fn)); end
        if ok, diag0.(fn)=sum(w.*vals); end
    end
    vertex.trim.diag=diag0;
end
vertex.lin.Ad=A; vertex.lin.Bd=B; vertex.lin.converged=false; vertex.lin.richardson_relerr_A=NaN; vertex.lin.richardson_relerr_B=NaN;
if isfield(vertex.lin,'A_h'), vertex.lin.A_h=nan(size(A)); end
if isfield(vertex.lin,'A_h2'), vertex.lin.A_h2=nan(size(A)); end
if isfield(vertex.lin,'B_h'), vertex.lin.B_h=nan(size(B)); end
if isfield(vertex.lin,'B_h2'), vertex.lin.B_h2=nan(size(B)); end
vertex.lin.spectral_radius_open=max(abs(eig(A)));
vertex.terminal.P=P; vertex.terminal.K=K; vertex.terminal.rho=rho; vertex.terminal.Np=opts.Horizon; vertex.terminal.Nc=opts.Horizon;
vertex.interp_meta=struct("version","Physics-MPC v0.9.0 bilinear HxV","exact_grid",false,"H",H,"V",V,"cfg",cfgId, ...
    "corner_HV",coords,"weights",w,"corner_row_indices",rowIdx,"corner_certified",cert,"corner_usable",usable, ...
    "source_certified_count",sum(cert),"max_Q_diff",maxQ,"max_R_diff",maxR,"max_state_scale_diff",maxSS,"max_input_scale_diff",maxUS,"rho",rho);
meta=vertex.interp_meta;
end

function [lo,hi,a]=localBracket(x,g)
tol=1e-12; j=find(abs(g-x)<=tol,1);
if ~isempty(j), lo=g(j); hi=g(j); a=0; return; end
ilo=find(g<x,1,'last'); ihi=find(g>x,1,'first');
if isempty(ilo) || isempty(ihi), error("AirdropX:PhysMPC:IntervalBracket","Cannot bracket %.12g inside grid.",x); end
lo=g(ilo); hi=g(ihi); a=(x-lo)/(hi-lo);
end
