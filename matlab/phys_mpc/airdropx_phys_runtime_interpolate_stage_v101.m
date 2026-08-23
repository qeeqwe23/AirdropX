function stage=airdropx_phys_runtime_interpolate_stage_v101(bank,H,V,cfgId,opts)
%AIRDROPX_PHYS_RUNTIME_INTERPOLATE_STAGE_V101 Bilinear H/V interpolation of level-trim local physics.
arguments
    bank (1,1) struct
    H (1,1) double {mustBeFinite}
    V (1,1) double {mustBeFinite,mustBePositive}
    cfgId (1,1) double {mustBeInteger}
    opts.NeedTerminal (1,1) logical = false
end
if H<20-1e-12 || H>200+1e-12, error("AirdropX:PhysMPC:RuntimeHeight","H outside [20,200]."); end
if V<45-1e-12 || V>65+1e-12, error("AirdropX:PhysMPC:RuntimeSpeed","V outside [45,65]."); end
if cfgId<0 || cfgId>4, error("AirdropX:PhysMPC:BadCfg","cfgId must be 0..4."); end
[ih0,ih1,ah]=localBracket(H,bank.hGrid); [iv0,iv1,av]=localBracket(V,bank.vGrid);
subs=[ih0 iv0; ih1 iv0; ih0 iv1; ih1 iv1]; w=[(1-ah)*(1-av);ah*(1-av);(1-ah)*av;ah*av]; active=w>1e-14; subs=subs(active,:); w=w(active); w=w/sum(w);
A=zeros(7); B=zeros(7,2); xbar=zeros(7,1); ubar=zeros(2,1); cert=true(numel(w),1); rows=zeros(numel(w),1);
for j=1:numel(w)
    k=bank.index(subs(j,1),subs(j,2),cfgId+1); rows(j)=k; v=bank.vertices{k}; cert(j)=logical(bank.rows.pass(k));
    A=A+w(j)*double(v.lin.Ad); B=B+w(j)*double(v.lin.Bd); xbar=xbar+w(j)*double(v.trim.x(:)); ubar=ubar+w(j)*double(v.trim.u(:));
end
if any(~isfinite(A),'all') || any(~isfinite(B),'all') || any(~isfinite(xbar)) || any(~isfinite(ubar)), error("AirdropX:PhysMPC:RuntimeNonfinite","Interpolated stage nonfinite."); end
if any(ubar<=bank.umin) || any(ubar>=bank.umax), error("AirdropX:PhysMPC:RuntimeTrimBound","Interpolated trim input reaches a hard bound."); end
P=[]; K=[]; rho=NaN;
if opts.NeedTerminal
    [K,P,poles]=dlqr(A,B,bank.Q,bank.R); rho=max(abs(poles));
    if ~(isfinite(rho)&&rho<1) || any(~isfinite(P),'all') || any(~isfinite(K),'all'), error("AirdropX:PhysMPC:RuntimeTerminalUnstable","Terminal DARE failed/stable rho=%g.",rho); end
end
stage=struct("A",A,"B",B,"xbar",xbar,"ubar",ubar,"xref",xbar,"uref",ubar,"P",P,"K",K,"rho",rho,"H",H,"V",V,"cfg",cfgId,"source_rows",rows,"source_certified",cert,"source_certified_count",sum(cert));
end

function [i0,i1,a]=localBracket(x,g)
g=double(g(:)); tol=1e-12; ie=find(abs(g-x)<=tol,1);
if ~isempty(ie), i0=ie; i1=ie; a=0; return; end
i0=find(g<x,1,'last'); i1=find(g>x,1,'first'); if isempty(i0)||isempty(i1), error("AirdropX:PhysMPC:RuntimeBracket","Cannot bracket command."); end
a=(x-g(i0))/(g(i1)-g(i0));
end
