function report=airdropx_phys_runtime_entry_v100(projectRoot,opts)
%AIRDROPX_PHYS_RUNTIME_ENTRY_V100 Isolated-process wrapper with durable completion marker.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.ScenarioName (1,1) string
    opts.MasterBankPath (1,1) string
    opts.Duration_s (1,1) double = 150
    opts.CommandPreviewMode (1,1) string = "known_reference"
end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
status=fullfile(opts.OutputRoot,'scenario_status.txt'); marker=fullfile(opts.OutputRoot,'scenario_complete.ok'); if isfile(marker), delete(marker); end
localStatus(status,'STARTED',opts.ScenarioName);
try
    report=airdropx_phys_runtime_command_closed_loop_v100(projectRoot,MasterBankPath=opts.MasterBankPath,OutputRoot=opts.OutputRoot,ScenarioName=opts.ScenarioName,Duration_s=opts.Duration_s,CommandPreviewMode=opts.CommandPreviewMode,ThrowOnFail=false);
    localStatus(status,'MISSION_RESULT_SAVED',sprintf('pass=%d peakNorm=%.6g totalP95=%.3fms',report.pass,report.metrics.peak_primary_normalized,report.metrics.total_compute_p95_ms));
    fid=fopen(marker,'w'); if fid<0, error("AirdropX:PhysMPC:MarkerWriteFailed","Cannot write completion marker."); end; fprintf(fid,'scenario=%s\npass=%d\ncompleted=%s\n',opts.ScenarioName,report.pass,char(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS'))); fclose(fid);
    localStatus(status,'COMPLETE_MARKER_WRITTEN','evidence durable; teardown may follow'); pause(0.05);
catch ME, localStatus(status,'ERROR',string(ME.identifier)+' | '+string(ME.message)); rethrow(ME); end
end
function localStatus(path,phase,detail)
fid=fopen(path,'a'); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>; fprintf(fid,'%s | %s | %s\n',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS')),phase,detail);
end
