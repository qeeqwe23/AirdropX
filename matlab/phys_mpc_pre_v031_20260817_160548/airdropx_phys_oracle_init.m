function info = airdropx_phys_oracle_init(projectRoot, opts)
arguments
    projectRoot (1,1) string
    opts.AircraftName (1,1) string = "MQ9_Reaper"
    opts.IcName (1,1) string = ""
    opts.PlantDt (1,1) double = 1/120
end
if opts.IcName==""
    opts.IcName=fullfile(projectRoot,"aircraft",opts.AircraftName,"generated","reset_20m_runtime");
    if ~isfile(opts.IcName+".xml")
        opts.IcName=fullfile(projectRoot,"aircraft",opts.AircraftName,"reset_20m");
    end
end
info=airdropx_jsbsim_oracle_mex("init",projectRoot,opts.AircraftName,opts.IcName,opts.PlantDt);
end
