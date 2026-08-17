function report=airdropx_phys_smoke(projectRoot)
%AIRDROPX_PHYS_SMOKE Minimal gate before any long build.
info=airdropx_phys_oracle_init(projectRoot);
cleanup=onCleanup(@()airdropx_jsbsim_oracle_mex("close")); %#ok<NASGU>
p=struct("cfgId",0,"fuelScale",1.0,"Ts",0.1);
z0=airdropx_phys_seed_from_existing("N1",info.base_n1,"N2",info.base_n2);
trim=airdropx_phys_trim_discrete(200,50,p,z0);
report=struct("info",info,"trim",trim,"pass",trim.pass);
if trim.pass
    lin=airdropx_phys_linearize_discrete(trim.x,trim.u,p);
    report.lin=lin; report.pass=report.pass && lin.converged;
end
if ~report.pass, error("AirdropX:PhysMPC:SmokeFailed","Physics MPC smoke failed."); end
end
