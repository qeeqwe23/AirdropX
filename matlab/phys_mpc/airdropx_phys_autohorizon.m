function info=airdropx_phys_autohorizon(Ad,Bd,Q,R,opts)
%AIRDROPX_PHYS_AUTOHORIZON Terminal LQR and Np derived from one Q/R.
arguments
    Ad double
    Bd double
    Q double
    R double
    opts.TargetDecay (1,1) double = 0.02
    opts.MinNp (1,1) double = 15
    opts.MaxNp (1,1) double = 100
end
[K,P,~]=dlqr(Ad,Bd,Q,R);
lam=eig(Ad-Bd*K); rho=max(abs(lam));
if rho>=1, error("AirdropX:PhysMPC:UnstableTerminal","Terminal LQR rho=%.9g >= 1",rho); end
Np=ceil(log(opts.TargetDecay)/log(max(rho,eps)));
Np=max(opts.MinNp,min(opts.MaxNp,Np));
info=struct("K",K,"P",P,"eig_cl",lam,"rho",rho,"Np",Np,"Nc",Np);
end
