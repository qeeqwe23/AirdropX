function report=airdropx_phys_finalize_interval_v090(projectRoot,opts)
%AIRDROPX_PHYS_FINALIZE_INTERVAL_V090 Aggregate dense audit + 144 off-grid missions + optional 95 grid reference.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.ManifestPath (1,1) string
    opts.DenseAuditRoot (1,1) string
    opts.GridEvidenceRoot (1,1) string = ""
end
M=readtable(opts.ManifestPath,TextType="string"); n=height(M);
Found=false(n,1); Pass=false(n,1); PeakQ=nan(n,1); PeakNorm=nan(n,1); PeakH=nan(n,1); PeakVa=nan(n,1); Recovery=nan(n,1); QpP95=nan(n,1); PredMax=nan(n,1); FinalNorm=nan(n,1); TailRms=nan(n,1); RhoMax=nan(n,1); SourceCertifiedCfg=nan(n,1);
for i=1:n
    dirCase=fullfile(opts.OutputRoot,M.CaseId(i)); p=fullfile(dirCase,"preview_mission.mat"); marker=fullfile(dirCase,"scenario_complete.ok"); metaP=fullfile(dirCase,"interval_metadata.mat");
    if isfile(p) && isfile(marker)
        S=load(p,"report"); r=S.report; Found(i)=true; Pass(i)=logical(r.pass); PeakQ(i)=r.metrics.peak_q_err_dps; PeakNorm(i)=r.metrics.peak_primary_normalized; PeakH(i)=r.metrics.peak_h_err_m; PeakVa(i)=r.metrics.peak_Va_err_mps; Recovery(i)=r.metrics.recovery_time_after_last_drop_s; QpP95(i)=r.metrics.qp_time_p95_ms; PredMax(i)=r.metrics.prediction_error_norm_max; FinalNorm(i)=r.metrics.final_normalized_inf; TailRms(i)=r.metrics.tail5s_normalized_rms;
        if isfile(metaP), Z=load(metaP,"intervalMeta"); RhoMax(i)=Z.intervalMeta.interpolation.bank_audit.rho_max; SourceCertifiedCfg(i)=Z.intervalMeta.interpolation.bank_audit.certified_count; end
    end
end
T=M; T.found=Found; T.pass=Pass; T.peak_q_dps=PeakQ; T.peak_primary_normalized=PeakNorm; T.peak_h_m=PeakH; T.peak_Va_mps=PeakVa; T.recovery_s=Recovery; T.qp_p95_ms=QpP95; T.prediction_error_max=PredMax; T.final_normalized_inf=FinalNorm; T.tail5s_normalized_rms=TailRms; T.interp_rho_max=RhoMax; T.source_certified_cfg_count=SourceCertifiedCfg;
writetable(T,fullfile(opts.OutputRoot,"interval_validation.csv"));
% Cell summary: two interior nonlinear points are required in each of 72 cells.
keys=compose("H%.0f_V%.0f",T.HCellLow,T.VCellLow); [uk,~,ic]=unique(keys,'stable'); nc=numel(uk); HCell=zeros(nc,1); VCell=zeros(nc,1); Cases=zeros(nc,1); CasesPass=zeros(nc,1); WorstNorm=nan(nc,1); WorstQ=nan(nc,1);
for j=1:nc, m=ic==j; HCell(j)=T.HCellLow(find(m,1)); VCell(j)=T.VCellLow(find(m,1)); Cases(j)=sum(m & Found); CasesPass(j)=sum(m & Found & Pass); if any(m & Found), WorstNorm(j)=max(PeakNorm(m & Found)); WorstQ(j)=max(PeakQ(m & Found)); end, end
Cells=table(HCell,VCell,Cases,CasesPass,WorstNorm,WorstQ,'VariableNames',{'H_cell_low_m','V_cell_low_mps','cases_found','cases_pass','worst_peak_norm','worst_q_dps'}); writetable(Cells,fullfile(opts.OutputRoot,"interval_cell_summary.csv"));
% Dense audit evidence.
denseP=fullfile(opts.DenseAuditRoot,"interval_dense_audit.mat"); if ~isfile(denseP), error("AirdropX:PhysMPC:DenseAuditMissing","Missing dense audit: %s",denseP); end
D=load(denseP,"report"); dense=D.report;
% Existing on-grid 95-case reference if available.
gridFound=0; gridPass=0; gridExpected=95;
if opts.GridEvidenceRoot~="" && isfolder(opts.GridEvidenceRoot)
    for V=[45 50 55 60 65], for H=20:10:200
        d=fullfile(opts.GridEvidenceRoot,sprintf("H%03d_V%03d",H,V)); p=fullfile(d,"preview_mission.mat"); mk=fullfile(d,"scenario_complete.ok");
        if isfile(p) && isfile(mk), gridFound=gridFound+1; S=load(p,"report"); gridPass=gridPass+double(logical(S.report.pass)); end
    end, end
end
offgridComplete=sum(Found)==n; offgridPass=sum(Pass & Found)==n; cellsComplete=nc==72 && all(Cases==2) && all(CasesPass==2);
gridPresent=gridFound>0; gridOK=(~gridPresent) || (gridFound==gridExpected && gridPass==gridExpected);
intervalPass=logical(dense.pass) && offgridComplete && offgridPass && cellsComplete && gridOK;
[worstNorm,iNorm]=max(PeakNorm); [worstQ,iQ]=max(PeakQ); [worstH,iH]=max(PeakH); [worstVa,iVa]=max(PeakVa); [worstRec,iRec]=max(Recovery); [worstQP,iQP]=max(QpP95); [worstPred,iPred]=max(PredMax);
worst=struct("peak_norm",worstNorm,"peak_norm_H",T.H_m(iNorm),"peak_norm_V",T.V_mps(iNorm),"q_dps",worstQ,"q_H",T.H_m(iQ),"q_V",T.V_mps(iQ), ...
    "h_m",worstH,"h_H",T.H_m(iH),"h_V",T.V_mps(iH),"Va_mps",worstVa,"Va_H",T.H_m(iVa),"Va_V",T.V_mps(iVa), ...
    "recovery_s",worstRec,"recovery_H",T.H_m(iRec),"recovery_V",T.V_mps(iRec),"qp_p95_ms",worstQP,"qp_H",T.H_m(iQP),"qp_V",T.V_mps(iQP), ...
    "prediction_max",worstPred,"prediction_H",T.H_m(iPred),"prediction_V",T.V_mps(iPred));
report=struct("version","Physics-MPC v0.9.0 continuous HxV interval validation","pass",intervalPass,"continuous_interval_validation_pass",intervalPass, ...
    "dense_model_audit_pass",logical(dense.pass),"dense_points_pass",dense.points_pass,"dense_points",dense.points, ...
    "offgrid_missions_found",sum(Found),"offgrid_missions_executed",sum(Found),"offgrid_missions_pass",sum(Pass & Found),"offgrid_expected",n, ...
    "cells_covered",sum(Cases==2),"cells_pass",sum(CasesPass==2),"cells_expected",72,"grid_reference_present",gridPresent,"grid_reference_found",gridFound,"grid_reference_pass",gridPass,"grid_reference_expected",gridExpected, ...
    "worst",worst,"completed_at",datetime("now"),"rows",T,"cells",Cells,"dense_audit",dense);
save(fullfile(opts.OutputRoot,"interval_validation.mat"),"report","T","Cells"); localWrite(report,fullfile(opts.OutputRoot,"interval_validation_summary.txt")); localPlot(T,fullfile(opts.OutputRoot,"interval_validation.png"));
end
function localWrite(r,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v0.9.0 continuous HxV interval validation\n");
fprintf(fid,"interval_H_m=[20,200]\ninterval_V_mps=[45,65]\nPreviewOnly=1\nq_soft=0\ndrop_schedule_s=10 10.2 10.4 10.6\nNp=100\nNc=100\n");
fprintf(fid,"continuous_interval_validation_pass=%d\n",r.pass);
fprintf(fid,"dense_model_audit_pass=%d\ndense_points_pass=%d/%d\n",r.dense_model_audit_pass,r.dense_points_pass,r.dense_points);
fprintf(fid,"offgrid_missions_found=%d/%d\noffgrid_missions_pass=%d/%d\ncells_covered=%d/%d\ncells_pass=%d/%d\n",r.offgrid_missions_found,r.offgrid_expected,r.offgrid_missions_pass,r.offgrid_expected,r.cells_covered,r.cells_expected,r.cells_pass,r.cells_expected);
fprintf(fid,"grid_reference_present=%d\ngrid_reference_found=%d/%d\ngrid_reference_pass=%d/%d\n",r.grid_reference_present,r.grid_reference_found,r.grid_reference_expected,r.grid_reference_pass,r.grid_reference_expected);
w=r.worst; fprintf(fid,"worst_peak_norm=%.9g at H=%.9g V=%.9g\nworst_q_dps=%.9g at H=%.9g V=%.9g\nworst_h_m=%.9g at H=%.9g V=%.9g\nworst_Va_mps=%.9g at H=%.9g V=%.9g\nworst_recovery_s=%.9g at H=%.9g V=%.9g\nworst_qp_p95_ms=%.9g at H=%.9g V=%.9g\nworst_prediction_error_max=%.9g at H=%.9g V=%.9g\n", ...
    w.peak_norm,w.peak_norm_H,w.peak_norm_V,w.q_dps,w.q_H,w.q_V,w.h_m,w.h_H,w.h_V,w.Va_mps,w.Va_H,w.Va_V,w.recovery_s,w.recovery_H,w.recovery_V,w.qp_p95_ms,w.qp_H,w.qp_V,w.prediction_max,w.prediction_H,w.prediction_V);
fprintf(fid,"NOTE: this is systematic continuous-interval interpolation/performance validation, not an analytic proof over infinitely many real-valued commands.\n");
end
function localPlot(T,path)
try
    f=figure('Visible','off','Position',[100 100 1500 900]); tl=tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact'); title(tl,'Physics-MPC v0.9 continuous HxV off-grid validation');
    nexttile; scatter(T.V_mps,T.H_m,42,T.peak_q_dps,'filled'); xlabel('Va (m/s)'); ylabel('H (m)'); title('peak |q| (deg/s)'); colorbar; grid on;
    nexttile; scatter(T.V_mps,T.H_m,42,T.peak_primary_normalized,'filled'); xlabel('Va (m/s)'); ylabel('H (m)'); title('peak normalized'); colorbar; grid on;
    nexttile; scatter(T.V_mps,T.H_m,42,T.peak_h_m,'filled'); xlabel('Va (m/s)'); ylabel('H (m)'); title('peak |h| (m)'); colorbar; grid on;
    nexttile; scatter(T.V_mps,T.H_m,42,T.recovery_s,'filled'); xlabel('Va (m/s)'); ylabel('H (m)'); title('recovery after last drop (s)'); colorbar; grid on;
    exportgraphics(f,path,'Resolution',160); close(f);
catch ME
    warning("AirdropX:PhysMPC:IntervalPlotFailed","Interval plot failed: %s",ME.message);
end
end
