function report=airdropx_phys_drop_scenario_entry(projectRoot,opts)
%AIRDROPX_PHYS_DROP_SCENARIO_ENTRY Run exactly one drop-timing scenario per MATLAB process.
%
% v0.5.2 intentionally does not explicitly destroy the persistent JSBSim
% Oracle before returning.  The PowerShell orchestrator runs each scenario in
% its own MATLAB process, waits until all result files and the completion marker
% are durable, then allows normal exit or terminates only that child after a
% short grace period if JSBSim/MEX teardown stalls.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.ScenarioName (1,1) string
    opts.DropTimes_s (1,4) double
    opts.Duration_s (1,1) double {mustBePositive} = 40
    opts.BankPath (1,1) string = ""
end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
statusPath=fullfile(opts.OutputRoot,"scenario_status.txt");
markerPath=fullfile(opts.OutputRoot,"scenario_complete.ok");
if isfile(markerPath), delete(markerPath); end
localStatus(statusPath,"STARTED",sprintf("scenario=%s schedule=[%s]",opts.ScenarioName,localVec(opts.DropTimes_s)));
try
    localStatus(statusPath,"MISSION_START","calling airdropx_phys_four_drop_closed_loop with CloseOracleOnReturn=false");
    args={"OutputRoot",opts.OutputRoot,"ScenarioName",opts.ScenarioName,"DropTimes_s",opts.DropTimes_s, ...
        "Duration_s",opts.Duration_s,"ThrowOnFail",false,"CloseOracleOnReturn",false};
    if opts.BankPath~="", args=[args,{"BankPath",opts.BankPath}]; end %#ok<AGROW>
    report=airdropx_phys_four_drop_closed_loop(projectRoot,args{:});
    localStatus(statusPath,"MISSION_RESULT_SAVED",sprintf("pass=%d",logical(report.pass)));
    fid=fopen(markerPath,"w");
    if fid<0, error("AirdropX:PhysMPC:MarkerWriteFailed","Could not write %s.",markerPath); end
    fprintf(fid,"scenario=%s\npass=%d\ncompleted=%s\n",opts.ScenarioName,logical(report.pass),char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")));
    fclose(fid);
    localStatus(statusPath,"COMPLETE_MARKER_WRITTEN","all mission outputs are already saved; process teardown may follow");
    pause(0.05); % give filesystem buffers a deterministic flush point before process teardown
catch ME
    localStatus(statusPath,"ERROR",sprintf("%s | %s",ME.identifier,ME.message));
    rethrow(ME);
end
end

function localStatus(path,phase,detail)
fid=fopen(path,"a");
if fid<0, return; end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"%s | %s | %s\n",char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")),phase,detail);
end

function s=localVec(v)
s=strjoin(compose("%.10g",double(v(:).'))," ");
end
