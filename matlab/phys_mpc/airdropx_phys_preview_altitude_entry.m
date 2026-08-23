function report=airdropx_phys_preview_altitude_entry(projectRoot,opts)
%AIRDROPX_PHYS_PREVIEW_ALTITUDE_ENTRY One altitude certification mission per MATLAB process.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.Height_m (1,1) double {mustBeFinite,mustBePositive}
    opts.BankPath (1,1) string = ""
end
validHeights=20:10:200;
if ~any(abs(validHeights-opts.Height_m)<=1e-12)
    error("AirdropX:PhysMPC:AltitudeGrid","v0.6.1 altitude certification requires H on 20:10:200 m grid.");
end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
status=fullfile(opts.OutputRoot,"scenario_status.txt");
marker=fullfile(opts.OutputRoot,"scenario_complete.ok");
if isfile(marker), delete(marker); end
localStatus(status,"STARTED",sprintf("H=%.1f V=50 PreviewOnly",opts.Height_m));
try
    args={"OutputRoot",opts.OutputRoot,"ScenarioName",string(sprintf("H%03d_V050_preview_only",round(opts.Height_m))), ...
        "H",opts.Height_m,"V",50,"EnableQSoft",false,"ThrowOnFail",false,"CloseOracleOnReturn",false, ...
        "Duration_s",40,"DropTimes_s",[10 10.2 10.4 10.6]};
    if opts.BankPath~="", args=[args,{"BankPath",opts.BankPath}]; end %#ok<AGROW>
    localStatus(status,"MISSION_START","PreviewOnly; fixed 0.2 s four-drop schedule; Oracle close deferred to process exit");
    report=airdropx_phys_preview_four_drop_closed_loop(projectRoot,args{:});
    localStatus(status,"MISSION_RESULT_SAVED",sprintf("pass=%d peakQ=%.6g peakNorm=%.6g final=%.6g", ...
        report.pass,report.metrics.peak_q_err_dps,report.metrics.peak_primary_normalized,report.metrics.final_normalized_inf));
    fid=fopen(marker,"w");
    if fid<0, error("AirdropX:PhysMPC:MarkerWriteFailed","Cannot write completion marker."); end
    fprintf(fid,"height_m=%.9g\nspeed_mps=50\npass=%d\ncompleted=%s\n",opts.Height_m,report.pass,char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")));
    fclose(fid);
    localStatus(status,"COMPLETE_MARKER_WRITTEN","all outputs durable; teardown may follow");
    pause(0.05);
catch ME
    localStatus(status,"ERROR",sprintf("%s | %s",ME.identifier,ME.message));
    rethrow(ME);
end
end

function localStatus(path,phase,detail)
fid=fopen(path,"a"); if fid<0, return; end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"%s | %s | %s\n",char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")),phase,detail);
end
