function vertex=airdropx_phys_build_vertex(H,V,cfgId,fuelScale,zSeed,opts)
%AIRDROPX_PHYS_BUILD_VERTEX Trim + exact discrete Jacobian + unified Q/R + auto horizon.
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
if ~lin.converged
    error("AirdropX:PhysMPC:DerivativeNonconverged", ...
        "Richardson derivative check failed A=%.3g B=%.3g", ...
        lin.richardson_relerr_A,lin.richardson_relerr_B);
end
[Q,R,qrMeta]=airdropx_phys_bryson_qr;
hor=airdropx_phys_autohorizon(lin.Ad,lin.Bd,Q,R);
vertex=struct("H",H,"V",V,"cfgId",cfgId,"fuelScale",fuelScale,"p",p, ...
    "trim",trim,"lin",lin,"Q",Q,"R",R,"qrMeta",qrMeta,"terminal",hor);
end
