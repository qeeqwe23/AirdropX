function trim = airdropx_phys_trim_discrete(Href,Vref,p,z0,opts)
%AIRDROPX_PHYS_TRIM_DISCRETE Solve a fixed point of the exact JSBSim Ts-map.
% z=[theta_rad,elevator_abs,throttle,N1,N2]. gamma=q=0 by definition.
arguments
    Href (1,1) double
    Vref (1,1) double
    p struct
    z0 (5,1) double
    opts.StateScale (5,1) double = [0.05; 0.002; deg2rad(0.02); 0.02; 0.02]
    opts.MaxScaledResidual (1,1) double = 1e-5
end
assert(isfield(p,"Ts") && isfield(p,"cfgId"),"p.Ts and p.cfgId are required.");
fun=@(z)local_residual(z,Href,Vref,p,opts.StateScale);
solverOpts=optimoptions("fsolve","Display","off", ...
    "FunctionTolerance",1e-12,"StepTolerance",1e-12,"OptimalityTolerance",1e-12, ...
    "MaxFunctionEvaluations",1200,"MaxIterations",300);
[z,~,exitflag,output]=fsolve(fun,z0,solverOpts);
[r,x0,x1,diag]=local_raw(z,Href,Vref,p);
scaled=r./opts.StateScale;
trim=struct();
trim.z=z; trim.x=x0; trim.u=z(2:3); trim.xnext=x1; trim.diag=diag;
trim.residual=r; trim.scaled_residual=scaled; trim.exitflag=exitflag; trim.output=output;
trim.h_step_m=x1(1)-Href;
trim.pass=exitflag>0 && norm(scaled,inf)<=opts.MaxScaledResidual && abs(trim.h_step_m)<=0.01;
end

function s=local_residual(z,H,V,p,scale)
r=local_raw(z,H,V,p); s=r./scale;
end

function [r,x0,x1,diag]=local_raw(z,H,V,p)
theta=z(1); u=z(2:3); n1=z(4); n2=z(5);
x0=[H;V;0;theta;0;n1;n2];
[x1,diag]=airdropx_phys_step(x0,u,p);
r=[x1(2)-V; x1(3); x1(5); x1(6)-n1; x1(7)-n2];
end
