function vertex=airdropx_phys_build_vertex_diagnostic_v081(H,V,cfgId,fuelScale,zSeed,opts)
%AIRDROPX_PHYS_BUILD_VERTEX_DIAGNOSTIC_V081 Build a computational vertex without hiding certification failures.
%
% This diagnostic builder uses the exact same trim, finite-difference steps,
% Richardson threshold, unified Q/R, and terminal design as the certified
% builder. The only behavioral difference is that a finite Richardson result
% with lin.converged=false is retained as an *uncertified but diagnostic-usable*
% vertex instead of throwing immediately. No threshold is relaxed.
arguments
    H (1,1) double
    V (1,1) double
    cfgId (1,1) double
    fuelScale (1,1) double
    zSeed (5,1) double
    opts.Ts (1,1) double = 0.1
end
p=struct("cfgId",cfgId,"fuelScale",fuelScale,"Ts",opts.Ts);
trim=airdropx_phys_trim_discrete(H,V,p,zSeed);
if ~trim.pass
    error("AirdropX:PhysMPC:TrimFailed", ...
        "Physics trim failed H=%.3f V=%.3f cfg=%d: exit=%d maxScaledRate=%.3g hStep=%.3g.", ...
        H,V,cfgId,trim.exitflag,norm(trim.scaled_residual,inf),trim.h_step_m);
end
lin=airdropx_phys_linearize_discrete(trim.x,trim.u,p);
if any(~isfinite(lin.Ad),'all') || any(~isfinite(lin.Bd),'all')
    error("AirdropX:PhysMPC:NonfiniteJacobian","Diagnostic vertex has non-finite Ad/Bd.");
end
[Q,R,qrMeta]=airdropx_phys_bryson_qr;
hor=airdropx_phys_autohorizon(lin.Ad,lin.Bd,Q,R);
terminalFinite=isfinite(hor.rho) && all(isfinite(hor.P),'all') && all(isfinite(hor.K),'all');
diagnosticUsable=trim.pass && terminalFinite && hor.rho<1;
certificationPass=diagnosticUsable && lin.converged;
if ~diagnosticUsable
    error("AirdropX:PhysMPC:DiagnosticVertexUnusable", ...
        "Diagnostic vertex is not safely runnable: rho=%.9g terminalFinite=%d.",hor.rho,terminalFinite);
end
reason="";
if ~lin.converged
    reason=sprintf("RichardsonNonconverged A=%.9g B=%.9g",lin.richardson_relerr_A,lin.richardson_relerr_B);
end
certification=struct("pass",certificationPass,"diagnostic_usable",diagnosticUsable, ...
    "richardson_pass",logical(lin.converged),"reason",string(reason));
vertex=struct("H",H,"V",V,"cfgId",cfgId,"fuelScale",fuelScale,"p",p, ...
    "trim",trim,"lin",lin,"Q",Q,"R",R,"qrMeta",qrMeta,"terminal",hor, ...
    "certification",certification);
end
