function R=airdropx_wind_airdrop_finalize_v131(root,manifestPath)
%AIRDROPX_WIND_AIRDROP_FINALIZE_V131 Aggregate 3-way carrier/release comparisons.
arguments
    root (1,1) string
    manifestPath (1,1) string
end
M=readtable(manifestPath,TextType="string");
modes=["wind_mpc_aware","legacy_mpc_aware","legacy_mpc_nowind"];
rows=table();
for i=1:height(M)
    for mode=modes
        tag=M.Scenario(i)+"_"+mode; p=fullfile(root,tag,"wind_airdrop_mission.mat");
        found=isfile(p); pass=false; drops=0; maxLand=NaN; rmsLand=NaN; windP95=NaN; instPeak=NaN; causalPeak=NaN; rec=NaN; postRms=NaN; qp=NaN; pred=NaN; fracCount=NaN; schedResidual=NaN;
        if found
            S=load(p,"report"); r=S.report; pass=logical(r.pass); drops=r.metrics.drops_completed; maxLand=r.metrics.max_landing_error_m; rmsLand=r.metrics.rms_landing_error_m; windP95=r.metrics.wind_error_p95_mps;
            instPeak=r.metrics.instantaneous_peak_primary_normalized; causalPeak=r.metrics.post_gust_peak_normalized; rec=r.metrics.max_gust_recovery_time_s; postRms=r.metrics.post_wind_primary_rms; qp=r.metrics.qp_p95_ms; pred=r.metrics.prediction_error_norm_p95; if isfield(r.metrics,"fractional_release_count"), fracCount=r.metrics.fractional_release_count; end; if isfield(r.metrics,"release_scheduler_residual_max_m"), schedResidual=r.metrics.release_scheduler_residual_max_m; end;
        end
        one=table(M.Scenario(i),mode,found,pass,drops,maxLand,rmsLand,windP95,instPeak,causalPeak,rec,postRms,qp,pred,fracCount,schedResidual, ...
            'VariableNames',{'Scenario','Mode','Found','Pass','Drops','MaxLandingError_m','RmsLandingError_m','WindErrorP95_mps','InstantPeakNorm','PostGustPeakNorm','MaxRecovery_s','PostWindRmsNorm','QpP95_ms','PredictionP95','FractionalReleaseCount','SchedulerResidualMax_m'});
        rows=[rows;one]; %#ok<AGROW>
    end
end
A=rows(rows.Mode=="wind_mpc_aware",:); L=rows(rows.Mode=="legacy_mpc_aware",:); B=rows(rows.Mode=="legacy_mpc_nowind",:);
comparison=table();
for i=1:height(M)
    a=A(A.Scenario==M.Scenario(i),:); l=L(L.Scenario==M.Scenario(i),:); b=B(B.Scenario==M.Scenario(i),:);
    aRms=localScalar(a,"PostWindRmsNorm"); lRms=localScalar(l,"PostWindRmsNorm"); aRec=localScalar(a,"MaxRecovery_s"); lRec=localScalar(l,"MaxRecovery_s"); aLand=localScalar(a,"RmsLandingError_m"); bLand=localScalar(b,"RmsLandingError_m");
    carrierRmsRatio=NaN; carrierRecoveryDelta=NaN; releaseRatio=NaN;
    if isfinite(aRms) && aRms>0 && isfinite(lRms), carrierRmsRatio=lRms/aRms; end
    if isfinite(aRec) && isfinite(lRec), carrierRecoveryDelta=lRec-aRec; end
    if isfinite(aLand) && aLand>0 && isfinite(bLand), releaseRatio=bLand/aLand; end
    comparison=[comparison;table(M.Scenario(i),aRms,lRms,carrierRmsRatio,aRec,lRec,carrierRecoveryDelta,aLand,bLand,releaseRatio, ...
        'VariableNames',{'Scenario','WindMpcPostWindRms','LegacyPostWindRms','LegacyOverWindMpcCarrierRms','WindMpcRecovery_s','LegacyRecovery_s','RecoveryImprovement_s','AwareLandingRms_m','NoWindReleaseLandingRms_m','NoWindOverAwareLandingRms'})]; %#ok<AGROW>
end
passIntegrated=height(A)==height(M) && all(A.Found) && all(A.Pass);
allExecuted=height(rows)==3*height(M) && all(rows.Found);
% Comparison is diagnostic, not a hidden retuning gate. Report non-regression
% explicitly so the user can see whether the new disturbance channel helped.
finiteRat=comparison.LegacyOverWindMpcCarrierRms(isfinite(comparison.LegacyOverWindMpcCarrierRms));
medianCarrierRatio=median(finiteRat,'omitnan');
R=struct(); R.version="Physics-MPC v1.3.1 wind-disturbance-aware precision airdrop validation"; R.pass=passIntegrated && allExecuted; R.integrated_pass=passIntegrated; R.all_three_way_executed=allExecuted; R.rows=rows; R.comparison=comparison; R.median_legacy_over_wind_mpc_carrier_rms=medianCarrierRatio; R.completed_at=datetime("now");
% Optional v1.3.1 calm release-timing ablation: same wind-aware MPC, but payload
% release snapped to the old 0.1 s sample boundary. It is diagnostic only.
abPath=fullfile(root,"calm_wind_mpc_aware_sampled_release","wind_airdrop_mission.mat"); R.calm_release_timing_ablation=struct("found",false,"fractional_rms_m",NaN,"sampled_rms_m",NaN,"sampled_over_fractional_rms",NaN,"fractional_max_m",NaN,"sampled_max_m",NaN);
a=A(A.Scenario=="calm",:); if height(a)==1, R.calm_release_timing_ablation.fractional_rms_m=a.RmsLandingError_m; R.calm_release_timing_ablation.fractional_max_m=a.MaxLandingError_m; end
if isfile(abPath), Z=load(abPath,"report"); R.calm_release_timing_ablation.found=true; R.calm_release_timing_ablation.sampled_rms_m=Z.report.metrics.rms_landing_error_m; R.calm_release_timing_ablation.sampled_max_m=Z.report.metrics.max_landing_error_m; if isfinite(R.calm_release_timing_ablation.fractional_rms_m) && R.calm_release_timing_ablation.fractional_rms_m>0, R.calm_release_timing_ablation.sampled_over_fractional_rms=R.calm_release_timing_ablation.sampled_rms_m/R.calm_release_timing_ablation.fractional_rms_m; end, end
writetable(rows,fullfile(root,"wind_disturbance_validation.csv")); writetable(comparison,fullfile(root,"wind_disturbance_comparison.csv")); save(fullfile(root,"wind_disturbance_validation.mat"),"R","-v7.3");
localWrite(R,fullfile(root,"wind_disturbance_validation_summary.txt")); localPlot(comparison,root);
end
function localWrite(R,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v1.3.1 wind-disturbance-aware precision airdrop validation\nvalidation_pass=%d\nintegrated_wind_mpc_scenarios_pass=%d/8\nall_three_way_missions_found=%d/24\n",R.pass,sum(R.rows.Pass & R.rows.Mode=="wind_mpc_aware"),sum(R.rows.Found));
A=R.rows(R.rows.Mode=="wind_mpc_aware",:); L=R.rows(R.rows.Mode=="legacy_mpc_aware",:); B=R.rows(R.rows.Mode=="legacy_mpc_nowind",:);
fprintf(fid,"wind_mpc_worst_landing_error_m=%.9g\nwind_mpc_worst_instantaneous_peak_norm=%.9g\nwind_mpc_worst_post_gust_peak_norm=%.9g\nwind_mpc_worst_recovery_s=%.9g\nwind_mpc_worst_post_wind_rms_norm=%.9g\nwind_mpc_worst_qp_p95_ms=%.9g\n",max(A.MaxLandingError_m,[],'omitnan'),max(A.InstantPeakNorm,[],'omitnan'),max(A.PostGustPeakNorm,[],'omitnan'),max(A.MaxRecovery_s,[],'omitnan'),max(A.PostWindRmsNorm,[],'omitnan'),max(A.QpP95_ms,[],'omitnan'));
fprintf(fid,"legacy_worst_post_gust_peak_norm=%.9g\nlegacy_worst_post_wind_rms_norm=%.9g\nno_wind_release_worst_landing_error_m=%.9g\nmedian_legacy_over_wind_mpc_carrier_rms=%.9g\n",max(L.PostGustPeakNorm,[],'omitnan'),max(L.PostWindRmsNorm,[],'omitnan'),max(B.MaxLandingError_m,[],'omitnan'),R.median_legacy_over_wind_mpc_carrier_rms);
fprintf(fid,"calm_release_ablation_found=%d\ncalm_fractional_rms_m=%.9g\ncalm_sampled_rms_m=%.9g\ncalm_sampled_over_fractional_rms=%.9g\ncalm_fractional_max_m=%.9g\ncalm_sampled_max_m=%.9g\n",R.calm_release_timing_ablation.found,R.calm_release_timing_ablation.fractional_rms_m,R.calm_release_timing_ablation.sampled_rms_m,R.calm_release_timing_ablation.sampled_over_fractional_rms,R.calm_release_timing_ablation.fractional_max_m,R.calm_release_timing_ablation.sampled_max_m);
for i=1:height(R.comparison), q=R.comparison(i,:); fprintf(fid,"%s carrierRms windMPC=%.6g legacy=%.6g ratio=%.6g recovery windMPC=%.6g legacy=%.6g landingAware=%.6g noWind=%.6g\n",char(q.Scenario),q.WindMpcPostWindRms,q.LegacyPostWindRms,q.LegacyOverWindMpcCarrierRms,q.WindMpcRecovery_s,q.LegacyRecovery_s,q.AwareLandingRms_m,q.NoWindReleaseLandingRms_m); end
end
function localPlot(C,root)
try
    f=figure("Visible","off","Position",[80 80 1550 850]); tiledlayout(f,2,2,"TileSpacing","compact","Padding","compact");
    nexttile; bar(categorical(C.Scenario),[C.WindMpcPostWindRms C.LegacyPostWindRms]); ylabel("post-wind primary RMS"); legend("wind-disturbance MPC","legacy MPC"); grid on;
    nexttile; bar(categorical(C.Scenario),[C.WindMpcRecovery_s C.LegacyRecovery_s]); ylabel("gust recovery (s)"); legend("wind-disturbance MPC","legacy MPC"); grid on;
    nexttile; bar(categorical(C.Scenario),[C.AwareLandingRms_m C.NoWindReleaseLandingRms_m]); ylabel("landing RMS miss (m)"); legend("wind-aware release","no-wind release"); grid on;
    nexttile; bar(categorical(C.Scenario),C.LegacyOverWindMpcCarrierRms); yline(1,'--'); ylabel("legacy / wind-MPC carrier RMS"); grid on;
    exportgraphics(f,fullfile(root,"wind_disturbance_validation.png"),"Resolution",160); close(f);
catch ME
    warning("AirdropX:WindMPC:FinalizePlotFailed","Final plot failed: %s",ME.message);
end
end

function v=localScalar(T,name)
if height(T)==1, v=double(T.(name)(1)); else, v=NaN; end
end
