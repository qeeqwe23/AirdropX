function info=airdropx_phys_wind_oracle_init_v121(projectRoot,opts)
%AIRDROPX_PHYS_WIND_ORACLE_INIT_V121 Initialize separate persistent wind JSBSim oracle.
arguments
    projectRoot (1,1) string
    opts.AircraftName (1,1) string = "MQ9_Reaper"
    opts.IcName (1,1) string = ""
    opts.PlantDt (1,1) double = 1/120
end
if opts.IcName==""
    opts.IcName=fullfile(projectRoot,"aircraft",opts.AircraftName,"generated","reset_20m_runtime");
    if ~isfile(opts.IcName+".xml"), opts.IcName=fullfile(projectRoot,"aircraft",opts.AircraftName,"reset_20m"); end
end
info=airdropx_jsbsim_wind_oracle_mex("init",projectRoot,opts.AircraftName,opts.IcName,opts.PlantDt);
if ~contains(string(info.version),"v1.2.1")
    error("AirdropX:WindAirdrop:WrongWindOracle","Unexpected wind oracle: %s",string(info.version));
end
end
