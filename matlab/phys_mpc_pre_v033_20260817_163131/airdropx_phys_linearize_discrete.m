function lin = airdropx_phys_linearize_discrete(x0,u0,p,opts)
%AIRDROPX_PHYS_LINEARIZE_DISCRETE Jacobian of exact JSBSim discrete map Phi_Ts.
% Central differences at h and h/2 + Richardson extrapolation. Input steps are
% automatically reduced near hard bounds so no hidden command clipping occurs.
arguments
    x0 (7,1) double {mustBeFinite}
    u0 (2,1) double {mustBeFinite}
    p (1,1) struct
    opts.AbsStepX (7,1) double = [5e-2;2e-2;2e-4;2e-4;2e-4;5e-2;5e-2]
    opts.RelStepX (7,1) double = [1e-5;1e-5;1e-5;1e-5;1e-5;1e-5;1e-5]
    opts.AbsStepU (2,1) double = [5e-4;5e-4]
    opts.RelStepU (2,1) double = [1e-5;1e-5]
    opts.InputLower (2,1) double = [-1;0]
    opts.InputUpper (2,1) double = [1;1]
    opts.MaxRichardsonRelErr (1,1) double {mustBePositive} = 5e-3
    opts.MinInputStep (2,1) double = [1e-6;1e-6]
end
assert(all(opts.AbsStepX>0) && all(opts.AbsStepU>0));
assert(all(opts.InputUpper>opts.InputLower));

% Determinism must be proven before interpreting finite differences as physics.
selftest=airdropx_phys_oracle_selftest(x0,u0,p);

hx=max(opts.AbsStepX,opts.RelStepX.*max(abs(x0),1));
hu=max(opts.AbsStepU,opts.RelStepU.*max(abs(u0),1));
margin=min(u0-opts.InputLower,opts.InputUpper-u0);
if any(margin<=0)
    error("AirdropX:PhysMPC:InputAtBound","Cannot central-difference an input on/outside a hard bound.");
end
hu=min(hu,0.45*margin);
if any(hu<opts.MinInputStep)
    error("AirdropX:PhysMPC:InputDerivativeStepTooSmall", ...
        "Input derivative step too small near bound: [%g %g].",hu(1),hu(2));
end

[A1,B1]=local_cd(x0,u0,p,hx,hu);
[A2,B2]=local_cd(x0,u0,p,hx/2,hu/2);
Ad=(4*A2-A1)/3;
Bd=(4*B2-B1)/3;
errA=norm(Ad-A2,"fro")/max(1,norm(Ad,"fro"));
errB=norm(Bd-B2,"fro")/max(1,norm(Bd,"fro"));
if any(~isfinite(Ad),"all") || any(~isfinite(Bd),"all")
    error("AirdropX:PhysMPC:NonfiniteJacobian","Non-finite Ad/Bd from physics oracle.");
end
[xn,diag]=airdropx_phys_step(x0,u0,p);
lin=struct("Ad",Ad,"Bd",Bd,"A_h",A1,"B_h",B1,"A_h2",A2,"B_h2",B2, ...
    "xnext0",xn,"diag",diag,"step_x",hx,"step_u",hu,"oracle_selftest",selftest, ...
    "richardson_relerr_A",errA,"richardson_relerr_B",errB, ...
    "converged",errA<=opts.MaxRichardsonRelErr && errB<=opts.MaxRichardsonRelErr, ...
    "spectral_radius_open",max(abs(eig(Ad))));
end

function [A,B]=local_cd(x,u,p,hx,hu)
A=zeros(7); B=zeros(7,2);
for i=1:7
    d=zeros(7,1); d(i)=hx(i);
    fp=airdropx_phys_step(x+d,u,p); fm=airdropx_phys_step(x-d,u,p);
    A(:,i)=(fp-fm)/(2*hx(i));
end
for j=1:2
    d=zeros(2,1); d(j)=hu(j);
    fp=airdropx_phys_step(x,u+d,p); fm=airdropx_phys_step(x,u-d,p);
    B(:,j)=(fp-fm)/(2*hu(j));
end
end
