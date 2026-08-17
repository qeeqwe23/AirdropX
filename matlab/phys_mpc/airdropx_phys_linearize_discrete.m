function lin = airdropx_phys_linearize_discrete(x0,u0,p,opts)
%AIRDROPX_PHYS_LINEARIZE_DISCRETE Jacobian of exact JSBSim discrete map Phi_Ts.
% Uses central differences at h and h/2 plus Richardson extrapolation.
arguments
    x0 (7,1) double
    u0 (2,1) double
    p struct
    opts.RelStepX (7,1) double = [2e-5;2e-5;3e-5;3e-5;3e-5;2e-5;2e-5]
    opts.AbsStepX (7,1) double = [1e-3;1e-3;1e-6;1e-6;1e-6;1e-3;1e-3]
    opts.RelStepU (2,1) double = [2e-5;2e-5]
    opts.AbsStepU (2,1) double = [1e-5;1e-5]
    opts.MaxRichardsonRelErr (1,1) double = 2e-3
end
hx=max(opts.AbsStepX,opts.RelStepX.*max(abs(x0),1));
hu=max(opts.AbsStepU,opts.RelStepU.*max(abs(u0),1));
[A1,B1]=local_cd(x0,u0,p,hx,hu);
[A2,B2]=local_cd(x0,u0,p,hx/2,hu/2);
Ad=(4*A2-A1)/3; Bd=(4*B2-B1)/3;
errA=norm(Ad-A2,"fro")/max(1,norm(Ad,"fro"));
errB=norm(Bd-B2,"fro")/max(1,norm(Bd,"fro"));
[xn,diag]=airdropx_phys_step(x0,u0,p);
lin=struct("Ad",Ad,"Bd",Bd,"xnext0",xn,"diag",diag,"step_x",hx,"step_u",hu, ...
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
