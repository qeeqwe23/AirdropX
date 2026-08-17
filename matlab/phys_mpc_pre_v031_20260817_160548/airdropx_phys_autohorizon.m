function info=airdropx_phys_autohorizon(Ad,Bd,Q,R,opts)
%AIRDROPX_PHYS_AUTOHORIZON Terminal LQR and Np derived from the single Q/R.
arguments
    Ad double
    Bd double
    Q double
    R double
    opts.TargetDecay (1,1) double {mustBePositive} = 0.02
    opts.MinNp (1,1) double {mustBeInteger,mustBePositive} = 15
    opts.MaxNp (1,1) double {mustBeInteger,mustBePositive} = 100
end
assert(opts.TargetDecay<1,"TargetDecay must be in (0,1).");
assert(opts.MaxNp>=opts.MinNp,"MaxNp must be >= MinNp.");
assert(all(isfinite(Ad),"all") && all(isfinite(Bd),"all"));
[K,P,clPoles]=dlqr(Ad,Bd,Q,R);
lam=eig(Ad-Bd*K); rho=max(abs(lam));
if ~isfinite(rho) || rho>=1
    error("AirdropX:PhysMPC:UnstableTerminal","Terminal LQR rho=%.9g >= 1 or non-finite.",rho);
end
Np=ceil(log(opts.TargetDecay)/log(max(rho,eps)));
Np=max(opts.MinNp,min(opts.MaxNp,Np));
info=struct("K",K,"P",P,"dlqr_poles",clPoles,"eig_cl",lam,"rho",rho,"Np",Np,"Nc",Np);
end
