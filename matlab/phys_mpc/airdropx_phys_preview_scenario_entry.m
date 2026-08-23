function report=airdropx_phys_preview_scenario_entry(projectRoot,opts)
%AIRDROPX_PHYS_PREVIEW_SCENARIO_ENTRY One preview scenario per MATLAB process.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.ScenarioName (1,1) string
    opts.EnableQSoft (1,1) logical
    opts.BankPath (1,1) string = ""
end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
status=fullfile(opts.OutputRoot,"scenario_status.txt"); marker=fullfile(opts.OutputRoot,"scenario_complete.ok"); if isfile(marker), delete(marker); end
localStatus(status,"STARTED",sprintf("scenario=%s qSoft=%d",opts.ScenarioName,opts.EnableQSoft));
try
    args={"OutputRoot",opts.OutputRoot,"ScenarioName",opts.ScenarioName,"EnableQSoft",opts.EnableQSoft,"ThrowOnFail",false,"CloseOracleOnReturn",false};
    if opts.BankPath~="", args=[args,{"BankPath",opts.BankPath}]; end %#ok<AGROW>
    localStatus(status,"MISSION_START","known schedule 10/10.2/10.4/10.6; Oracle close deferred to process exit");
    report=airdropx_phys_preview_four_drop_closed_loop(projectRoot,args{:});
    localStatus(status,"MISSION_RESULT_SAVED",sprintf("pass=%d peakQ=%.6g",report.pass,report.metrics.peak_q_err_dps));
    fid=fopen(marker,"w"); if fid<0, error("AirdropX:PhysMPC:MarkerWriteFailed","Cannot write completion marker."); end
    fprintf(fid,"scenario=%s\npass=%d\ncompleted=%s\n",opts.ScenarioName,report.pass,char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS"))); fclose(fid);
    localStatus(status,"COMPLETE_MARKER_WRITTEN","all outputs durable; teardown may follow"); pause(0.05);
catch ME
    localStatus(status,"ERROR",sprintf("%s | %s",ME.identifier,ME.message)); rethrow(ME);
end
end
function localStatus(path,phase,detail)
fid=fopen(path,"a"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"%s | %s | %s\n",char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")),phase,detail);
end
