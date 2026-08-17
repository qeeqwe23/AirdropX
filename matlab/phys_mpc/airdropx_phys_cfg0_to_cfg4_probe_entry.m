function report=airdropx_phys_cfg0_to_cfg4_probe_entry(projectRoot,opts)
%AIRDROPX_PHYS_CFG0_TO_CFG4_PROBE_ENTRY Dedicated-process wrapper for the jump probe.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.BankPath (1,1) string = ""
end
report=airdropx_phys_cfg0_to_cfg4_jump_probe(projectRoot,OutputRoot=opts.OutputRoot,BankPath=opts.BankPath);
end
