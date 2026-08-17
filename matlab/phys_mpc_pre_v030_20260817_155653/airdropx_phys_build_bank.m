function result=airdropx_phys_build_bank(projectRoot,opts)
%AIRDROPX_PHYS_BUILD_BANK Build physics vertices by continuation; no per-cfg tuning.
arguments
    projectRoot (1,1) string
    opts.Heights double = 200:-10:20
    opts.Speeds double = 50
    opts.CfgIds double = 0:4
    opts.FuelScales double = 1.0
    opts.Ts (1,1) double = 0.1
    opts.OutputRoot (1,1) string = ""
    opts.StopOnFailure (1,1) logical = true
end
assert(all(isfinite(opts.Heights)) && all(opts.Heights>0),"Heights must be finite and positive.");
assert(all(isfinite(opts.Speeds)) && all(opts.Speeds>0),"Speeds must be finite and positive.");
assert(all(isfinite(opts.CfgIds)) && all(opts.CfgIds==round(opts.CfgIds)) && all(opts.CfgIds>=0 & opts.CfgIds<=4),"CfgIds must be integers 0..4.");
assert(all(isfinite(opts.FuelScales)) && all(opts.FuelScales>=0 & opts.FuelScales<=1.2),"FuelScales must be in [0,1.2].");
if opts.OutputRoot==""
    opts.OutputRoot=fullfile(projectRoot,"matlab","results","physics_mpc_v030");
end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
preflight=airdropx_phys_preflight(projectRoot); %#ok<NASGU>
mathSelftest=airdropx_phys_math_selftest(); %#ok<NASGU>
info=airdropx_phys_oracle_init(projectRoot);
if ~isfield(info,"version") || ~contains(string(info.version),"v0.3.0")
    error("AirdropX:PhysMPC:OldOracle","Build-bank requires Physics Oracle v0.3.0. Rebuild the Oracle first.");
end
cleanup=onCleanup(@()airdropx_jsbsim_oracle_mex("close")); %#ok<NASGU>
baseSeed=airdropx_phys_seed_from_existing("N1",info.base_n1,"N2",info.base_n2);
% A direct Build must not be able to bypass the structural mass/fuel gate merely
% because the user skipped Smoke. Audit once at a benign reference state before
% generating any certification vertex.
auditX=[max(opts.Heights(:)); max(opts.Speeds(:)); 0; baseSeed(1); 0; info.base_n1; info.base_n2];
auditU=baseSeed(2:3);
auditP=struct("cfgId",0,"fuelScale",1.0,"Ts",opts.Ts);
configurationAudit=airdropx_phys_oracle_config_audit(auditX,auditU,auditP,info); %#ok<NASGU>
lastByCfg=cell(5,1); for i=1:5, lastByCfg{i}=baseSeed; end
rows=table(); vertices={};
idx=0;
for fuel=opts.FuelScales(:)'
    for V=opts.Speeds(:)'
        for H=opts.Heights(:)'
            previousCfgSeed=[];
            for cfg=opts.CfgIds(:)'
                idx=idx+1;
                if ~isempty(previousCfgSeed), seed=previousCfgSeed; else, seed=lastByCfg{cfg+1}; end
                pass=false; msg=""; trimNorm=NaN; dA=NaN; dB=NaN; rho=NaN; Np=NaN;
                try
                    vertex=airdropx_phys_build_vertex(H,V,cfg,fuel,seed,Ts=opts.Ts);
                    pass=true; vertices{idx,1}=vertex; %#ok<AGROW>
                    previousCfgSeed=vertex.trim.z; lastByCfg{cfg+1}=vertex.trim.z;
                    trimNorm=norm(vertex.trim.scaled_residual,inf);
                    dA=vertex.lin.richardson_relerr_A; dB=vertex.lin.richardson_relerr_B;
                    rho=vertex.terminal.rho; Np=vertex.terminal.Np;
                    vdir=fullfile(opts.OutputRoot,sprintf("H%03.0f_V%05.1f_cfg%d_fuel%04.2f",H,V,cfg,fuel));
                    if ~isfolder(vdir), mkdir(vdir); end
                    save(fullfile(vdir,"physics_vertex.mat"),"vertex");
                catch ME
                    msg=string(ME.identifier)+": "+string(ME.message);
                    vertices{idx,1}=[]; %#ok<AGROW>
                    if opts.StopOnFailure
                        row=table(H,V,cfg,fuel,pass,trimNorm,dA,dB,rho,Np,msg, ...
                            'VariableNames',{'H_m','V_mps','cfg','fuel_scale','pass','trim_inf','dA_rel','dB_rel','rho_cl','Np','message'});
                        rows=[rows;row]; %#ok<AGROW>
                        writetable(rows,fullfile(opts.OutputRoot,"physics_vertex_summary.csv"));
                        save(fullfile(opts.OutputRoot,"physics_bank_partial.mat"),"vertices","rows","opts","info");
                        rethrow(ME);
                    end
                end
                row=table(H,V,cfg,fuel,pass,trimNorm,dA,dB,rho,Np,msg, ...
                    'VariableNames',{'H_m','V_mps','cfg','fuel_scale','pass','trim_inf','dA_rel','dB_rel','rho_cl','Np','message'});
                rows=[rows;row]; %#ok<AGROW>
                writetable(rows,fullfile(opts.OutputRoot,"physics_vertex_summary.csv"));
            end
        end
    end
end
save(fullfile(opts.OutputRoot,"physics_bank.mat"),"vertices","rows","opts","info", ...
    "configurationAudit","mathSelftest","-v7.3");
result=struct("rows",rows,"vertices",{vertices},"configurationAudit",configurationAudit, ...
    "mathSelftest",mathSelftest,"outputRoot",opts.OutputRoot,"pass",all(rows.pass));
end
