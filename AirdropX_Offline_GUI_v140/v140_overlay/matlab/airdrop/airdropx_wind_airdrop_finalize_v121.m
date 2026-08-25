function R=airdropx_wind_airdrop_finalize_v121(root,manifestPath)
%AIRDROPX_WIND_AIRDROP_FINALIZE_V121 Aggregate paired precision-airdrop missions.
arguments
    root (1,1) string
    manifestPath (1,1) string
end
M=readtable(manifestPath,TextType="string");
rows=table();
for i=1:height(M)
    for aware=[true false]
        mode="wind_aware"; if ~aware, mode="no_wind_baseline"; end
        tag=M.Scenario(i)+"_"+mode; p=fullfile(root,tag,"wind_airdrop_mission.mat");
        found=isfile(p); pass=false; maxLand=NaN; rmsLand=NaN; windP95=NaN; peak=NaN; qp=NaN; drops=0;
        if found
            S=load(p,"report"); r=S.report; pass=logical(r.pass); maxLand=r.metrics.max_landing_error_m; rmsLand=r.metrics.rms_landing_error_m; windP95=r.metrics.wind_error_p95_mps; peak=r.metrics.peak_primary_normalized; qp=r.metrics.qp_p95_ms; drops=r.metrics.drops_completed;
        end
        one=table(M.Scenario(i),mode,found,pass,drops,maxLand,rmsLand,windP95,peak,qp, ...
            'VariableNames',{'Scenario','Mode','Found','Pass','Drops','MaxLandingError_m','RmsLandingError_m','WindErrorP95_mps','PeakPrimaryNormalized','QpP95_ms'});
        rows=[rows;one]; %#ok<AGROW>
    end
end
aware=rows(rows.Mode=="wind_aware",:); base=rows(rows.Mode=="no_wind_baseline",:);
comparison=table();
for i=1:height(M)
    a=aware(aware.Scenario==M.Scenario(i),:); b=base(base.Scenario==M.Scenario(i),:);
    improvement=NaN; if height(a)==1 && height(b)==1 && isfinite(a.RmsLandingError_m) && a.RmsLandingError_m>0, improvement=b.RmsLandingError_m/a.RmsLandingError_m; end
    comparison=[comparison;table(M.Scenario(i),a.MaxLandingError_m,b.MaxLandingError_m,a.RmsLandingError_m,b.RmsLandingError_m,improvement, ...
        'VariableNames',{'Scenario','AwareMaxMiss_m','BaselineMaxMiss_m','AwareRmsMiss_m','BaselineRmsMiss_m','BaselineOverAwareRmsRatio'})]; %#ok<AGROW>
end
passAware=height(aware)==height(M) && all(aware.Found) && all(aware.Pass);
allExecuted=height(rows)==2*height(M) && all(rows.Found);
R=struct(); R.version="Physics-MPC v1.2.1 wind-aware precision airdrop validation"; R.pass=passAware && allExecuted; R.wind_aware_pass=passAware; R.all_paired_executed=allExecuted; R.rows=rows; R.comparison=comparison; R.completed_at=datetime("now");
writetable(rows,fullfile(root,"wind_airdrop_validation.csv")); writetable(comparison,fullfile(root,"wind_airdrop_comparison.csv")); save(fullfile(root,"wind_airdrop_validation.mat"),"R","-v7.3");
localWrite(R,fullfile(root,"wind_airdrop_validation_summary.txt")); localPlot(comparison,root);
end
function localWrite(R,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v1.2.1 wind-aware precision airdrop validation\nvalidation_pass=%d\nwind_aware_scenarios_pass=%d/8\npaired_missions_found=%d/16\n",R.pass,sum(R.rows.Pass & R.rows.Mode=="wind_aware"),sum(R.rows.Found));
a=R.rows(R.rows.Mode=="wind_aware",:); b=R.rows(R.rows.Mode=="no_wind_baseline",:);
fprintf(fid,"aware_worst_landing_error_m=%.9g\naware_worst_wind_p95_mps=%.9g\naware_worst_peak_primary_normalized=%.9g\naware_worst_qp_p95_ms=%.9g\n",max(a.MaxLandingError_m,[],'omitnan'),max(a.WindErrorP95_mps,[],'omitnan'),max(a.PeakPrimaryNormalized,[],'omitnan'),max(a.QpP95_ms,[],'omitnan'));
fprintf(fid,"baseline_worst_landing_error_m=%.9g\nmedian_baseline_over_aware_rms_ratio=%.9g\n",max(b.MaxLandingError_m,[],'omitnan'),median(R.comparison.BaselineOverAwareRmsRatio,'omitnan'));
for i=1:height(R.comparison), q=R.comparison(i,:); fprintf(fid,"%s awareMax=%.6g baselineMax=%.6g awareRms=%.6g baselineRms=%.6g ratio=%.6g\n",char(q.Scenario),q.AwareMaxMiss_m,q.BaselineMaxMiss_m,q.AwareRmsMiss_m,q.BaselineRmsMiss_m,q.BaselineOverAwareRmsRatio); end
end
function localPlot(C,root)
try
    f=figure("Visible","off","Position",[80 80 1450 720]); tiledlayout(f,1,2,"TileSpacing","compact","Padding","compact");
    nexttile; bar(categorical(C.Scenario),[C.AwareMaxMiss_m C.BaselineMaxMiss_m]); ylabel("max |landing error| (m)"); legend("wind-aware","no-wind baseline"); grid on;
    nexttile; bar(categorical(C.Scenario),C.BaselineOverAwareRmsRatio); yline(1,'--'); ylabel("baseline / aware RMS miss"); grid on;
    exportgraphics(f,fullfile(root,"wind_airdrop_validation.png"),"Resolution",160); close(f);
catch ME
    warning("AirdropX:WindAirdrop:FinalizePlotFailed","Final plot failed: %s",ME.message);
end
end
