function rows=airdropx_phys_mass_audit(H,V,opts)
%AIRDROPX_PHYS_MASS_AUDIT Read actual JSBSim mass/CG/Iyy for cfg0..4.
arguments
    H (1,1) double = 200
    V (1,1) double = 50
    opts.Ts (1,1) double = 0.1
    opts.FuelScale (1,1) double = 1.0
end
info=airdropx_jsbsim_oracle_mex("info");
rows=table();
for cfg=0:4
    x=[H;V;0;deg2rad(5);0;info.base_n1;info.base_n2];
    u=[0;0.8]; p=struct("cfgId",cfg,"fuelScale",opts.FuelScale,"Ts",opts.Ts);
    [~,d]=airdropx_phys_step(x,u,p);
    rows=[rows;table(cfg,d.mass_kg,d.cg_x_m,d.Iyy_kgm2,'VariableNames',{'cfg','mass_kg','cg_x_m','Iyy_kgm2'})]; %#ok<AGROW>
end
end
