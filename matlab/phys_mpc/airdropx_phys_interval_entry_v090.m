function report=airdropx_phys_interval_entry_v090(projectRoot,opts)
%AIRDROPX_PHYS_INTERVAL_ENTRY_V090 One arbitrary continuous H/V nonlinear PreviewOnly mission.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.Height_m (1,1) double {mustBeFinite}
    opts.Speed_mps (1,1) double {mustBeFinite,mustBePositive}
    opts.MasterBankPath (1,1) string
end
H=opts.Height_m; V=opts.Speed_mps;
if H<20 || H>200, error("AirdropX:PhysMPC:IntervalHeight","H must be inside [20,200] m."); end
if V<45 || V>65, error("AirdropX:PhysMPC:IntervalSpeed","V must be inside [45,65] m/s."); end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
status=fullfile(opts.OutputRoot,"scenario_status.txt"); marker=fullfile(opts.OutputRoot,"scenario_complete.ok"); if isfile(marker), delete(marker); end
localStatus(status,"STARTED",sprintf("continuous H=%.6g V=%.6g",H,V));
try
    localBank=fullfile(opts.OutputRoot,"interpolated_bank.mat");
    interpReport=airdropx_phys_interval_make_bank_v090(opts.MasterBankPath,H,V,localBank);
    localStatus(status,"INTERPOLATION_DONE",sprintf("exact=%d rho=[%.6g %.6g] certified=%d/5",interpReport.exact_grid,interpReport.bank_audit.rho_min,interpReport.bank_audit.rho_max,interpReport.bank_audit.certified_count));
    name=string(sprintf("continuous_H%.3f_V%.3f",H,V));
    report=airdropx_phys_preview_four_drop_closed_loop(projectRoot,"OutputRoot",opts.OutputRoot,"ScenarioName",name, ...
        "H",H,"V",V,"BankPath",localBank,"EnableQSoft",false,"ThrowOnFail",false,"CloseOracleOnReturn",false, ...
        "AllowUncertifiedVertices",true,"Horizon",100,"Duration_s",40,"DropTimes_s",[10 10.2 10.4 10.6]);
    intervalMeta=struct("version","Physics-MPC v0.9.0 continuous interval mission","H",H,"V",V,"interpolation",interpReport,"mission_pass",report.pass,"completed_at",datetime("now"));
    save(fullfile(opts.OutputRoot,"interval_metadata.mat"),"intervalMeta");
    localWrite(intervalMeta,report,fullfile(opts.OutputRoot,"interval_summary.txt"));
    localStatus(status,"MISSION_RESULT_SAVED",sprintf("pass=%d peakQ=%.6g norm=%.6g final=%.6g",report.pass,report.metrics.peak_q_err_dps,report.metrics.peak_primary_normalized,report.metrics.final_normalized_inf));
    fid=fopen(marker,"w"); if fid<0, error("AirdropX:PhysMPC:MarkerWriteFailed","Cannot write completion marker."); end
    fprintf(fid,"H=%.12g\nV=%.12g\npass=%d\ncompleted=%s\n",H,V,report.pass,char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS"))); fclose(fid);
    localStatus(status,"COMPLETE_MARKER_WRITTEN","evidence durable; teardown may follow"); pause(0.05);
catch ME
    localStatus(status,"ERROR",sprintf("%s | %s",ME.identifier,ME.message)); rethrow(ME);
end
end
function localStatus(path,phase,detail)
fid=fopen(path,"a"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"%s | %s | %s\n",char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")),phase,detail);
end
function localWrite(meta,r,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v0.9.0 continuous interval point\nH=%.9g\nV=%.9g\nmission_pass=%d\nexact_grid=%d\nsource_certified_cfg=%d/5\n",meta.H,meta.V,r.pass,meta.interpolation.exact_grid,meta.interpolation.bank_audit.certified_count);
fprintf(fid,"peak_q_dps=%.9g\npeak_norm=%.9g\npeak_h_m=%.9g\npeak_Va_mps=%.9g\nrecovery_s=%.9g\nqp_p95_ms=%.9g\nprediction_max=%.9g\n", ...
    r.metrics.peak_q_err_dps,r.metrics.peak_primary_normalized,r.metrics.peak_h_err_m,r.metrics.peak_Va_err_mps,r.metrics.recovery_time_after_last_drop_s,r.metrics.qp_time_p95_ms,r.metrics.prediction_error_norm_max);
end
