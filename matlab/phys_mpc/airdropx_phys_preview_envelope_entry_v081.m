function report=airdropx_phys_preview_envelope_entry_v081(projectRoot,opts)
%AIRDROPX_PHYS_PREVIEW_ENVELOPE_ENTRY_V081 Diagnostic mission that may use explicitly uncertified-but-usable vertices.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.Height_m (1,1) double {mustBeFinite,mustBePositive}
    opts.Speed_mps (1,1) double {mustBeFinite,mustBePositive}
    opts.BankPath (1,1) string
end
validH=20:10:200; validV=[45 50 55 60 65];
if ~any(abs(validH-opts.Height_m)<=1e-12), error("AirdropX:PhysMPC:EnvelopeHeight","Formal H must be 20:10:200 m."); end
if ~any(abs(validV-opts.Speed_mps)<=1e-12), error("AirdropX:PhysMPC:EnvelopeSpeed","Formal V must be one of [45 50 55 60 65] m/s."); end
if ~isfile(opts.BankPath), error("AirdropX:PhysMPC:BankMissing","Diagnostic envelope bank missing: %s",opts.BankPath); end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
status=fullfile(opts.OutputRoot,"scenario_status.txt"); marker=fullfile(opts.OutputRoot,"scenario_complete.ok"); if isfile(marker), delete(marker); end
localStatus(status,"STARTED",sprintf("H=%.1f V=%.1f PreviewOnly diagnostic allow-uncertified",opts.Height_m,opts.Speed_mps));
try
    B=load(opts.BankPath,"rows"); r=B.rows; mask=abs(r.H_m-opts.Height_m)<=1e-9 & abs(r.V_mps-opts.Speed_mps)<=1e-9 & r.cfg>=0 & r.cfg<=4;
    rr=r(mask,:); if height(rr)~=5, error("AirdropX:PhysMPC:MissingCfgRows","Expected 5 cfg rows for H=%.0f V=%.0f, found %d.",opts.Height_m,opts.Speed_mps,height(rr)); end
    [~,ord]=sort(rr.cfg); rr=rr(ord,:);
    if ismember('usable',rr.Properties.VariableNames), usable=logical(rr.usable); else, usable=logical(rr.pass); end
    certified=logical(rr.pass); usesUncertified=any(~certified & usable);
    if ~all(usable)
        badCfg=rr.cfg(~usable); report=struct("pass",false,"model_available",false,"uses_uncertified_model",usesUncertified, ...
            "certified_cfg_count",sum(certified),"unusable_cfg",badCfg,"H",opts.Height_m,"V",opts.Speed_mps,"completed_at",datetime("now"));
        save(fullfile(opts.OutputRoot,"preview_mission_unavailable.mat"),"report");
        localStatus(status,"MODEL_UNAVAILABLE",sprintf("unusable cfg=%s; mission not fabricated",mat2str(badCfg')));
    else
        localStatus(status,"MISSION_START",sprintf("PreviewOnly; certified cfg=%d/5; usesUncertified=%d; fixed 0.2 s four-drop",sum(certified),usesUncertified));
        name=string(sprintf("H%03d_V%03d_preview_only_diag",round(opts.Height_m),round(opts.Speed_mps)));
        report=airdropx_phys_preview_four_drop_closed_loop(projectRoot,"OutputRoot",opts.OutputRoot,"ScenarioName",name, ...
            "H",opts.Height_m,"V",opts.Speed_mps,"BankPath",opts.BankPath,"EnableQSoft",false,"ThrowOnFail",false,"CloseOracleOnReturn",false, ...
            "AllowUncertifiedVertices",true,"Duration_s",40,"DropTimes_s",[10 10.2 10.4 10.6]);
        localStatus(status,"MISSION_RESULT_SAVED",sprintf("pass=%d certifiedCfg=%d/5 peakQ=%.6g peakNorm=%.6g final=%.6g",report.pass,sum(certified),report.metrics.peak_q_err_dps,report.metrics.peak_primary_normalized,report.metrics.final_normalized_inf));
    end
    fid=fopen(marker,"w"); if fid<0, error("AirdropX:PhysMPC:MarkerWriteFailed","Cannot write completion marker."); end
    fprintf(fid,"height_m=%.9g\nspeed_mps=%.9g\npass=%d\nmodel_available=%d\ncertified_cfg_count=%d\nuses_uncertified_model=%d\ncompleted=%s\n", ...
        opts.Height_m,opts.Speed_mps,report.pass,all(usable),sum(certified),usesUncertified,char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS"))); fclose(fid);
    localStatus(status,"COMPLETE_MARKER_WRITTEN","diagnostic scenario evidence durable; teardown may follow"); pause(0.05);
catch ME
    localStatus(status,"ERROR",sprintf("%s | %s",ME.identifier,ME.message)); rethrow(ME);
end
end
function localStatus(path,phase,detail)
fid=fopen(path,"a"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"%s | %s | %s\n",char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")),phase,detail);
end
