function report=airdropx_phys_preflight(projectRoot)
%AIRDROPX_PHYS_PREFLIGHT Cheap deterministic checks before JSBSim work.
arguments
    projectRoot (1,1) string
end
required={'lsqnonlin','dlqr','optimoptions'};
missing=strings(0,1);
for k=1:numel(required)
    if exist(required{k},"file")==0
        missing(end+1,1)=required{k}; %#ok<AGROW>
    end
end
if ~isempty(missing)
    error("AirdropX:PhysMPC:MissingToolbox","Missing required MATLAB functions: %s",strjoin(missing,", "));
end
physDir=fullfile(projectRoot,"matlab","phys_mpc");
requiredFiles=["airdropx_jsbsim_oracle_mex.cpp","airdropx_phys_trim_discrete.m", ...
    "airdropx_phys_linearize_discrete.m","airdropx_phys_step.m", ...
    "airdropx_phys_oracle_selftest.m","airdropx_phys_oracle_config_audit.m", ...
    "airdropx_phys_bryson_qr.m","airdropx_phys_autohorizon.m", ...
    "airdropx_phys_math_selftest.m"];
for f=requiredFiles
    if ~isfile(fullfile(physDir,f))
        error("AirdropX:PhysMPC:MissingSource","Missing Physics-MPC source: %s",f);
    end
end
report=struct("pass",true,"matlab",version,"mexext",mexext,"phys_dir",physDir);
end
