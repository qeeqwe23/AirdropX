function R=airdropx_wind_finalize_v110(outputRoot,manifestPath)
%AIRDROPX_WIND_FINALIZE_V110 Aggregate eight real-JSBSim longitudinal wind tests.
M=readtable(manifestPath,TextType="string"); n=height(M);
Scenario=M.Scenario; Found=false(n,1); Pass=false(n,1); NoiselessRmse=nan(n,1); NoisyRmse=nan(n,1); NoisyP95=nan(n,1); Bias=nan(n,1); Settle90=nan(n,1); NoiseReduction=nan(n,1); SignAccuracy=nan(n,1); CrosswindP95=nan(n,1);
for i=1:n
    p=fullfile(outputRoot,Scenario(i),"wind_estimation_validation.mat");
    if ~isfile(p), continue; end
    S=load(p,"report"); r=S.report; Found(i)=true; Pass(i)=logical(r.pass); m=r.metrics;
    NoiselessRmse(i)=m.noiseless_rmse_mps; NoisyRmse(i)=m.noisy_rmse_mean_mps; NoisyP95(i)=m.noisy_p95_abs_mean_mps; Bias(i)=m.noisy_bias_abs_mean_mps; Settle90(i)=m.step_settling90_worst_s; NoiseReduction(i)=m.noise_reduction_ratio; SignAccuracy(i)=m.sign_accuracy_mean; CrosswindP95(i)=m.crosswind_truth_p95_mps;
end
T=table(Scenario,Found,Pass,NoiselessRmse,NoisyRmse,NoisyP95,Bias,Settle90,NoiseReduction,SignAccuracy,CrosswindP95);
writetable(T,fullfile(outputRoot,"wind_validation.csv"));
R=struct("version","Physics-MPC v1.1.0 longitudinal wind estimation validation","pass",all(Found)&all(Pass),"scenarios_found",sum(Found),"scenarios_pass",sum(Pass),"scenarios_expected",n,"rows",T,"completed_at",datetime("now"));
save(fullfile(outputRoot,"wind_validation.mat"),"R","T");
fid=fopen(fullfile(outputRoot,"wind_validation_summary.txt"),"w"); if fid>=0
    c=onCleanup(@()fclose(fid)); fprintf(fid,"Physics-MPC v1.1.0 longitudinal wind estimation validation\nvalidation_pass=%d\nscenarios_found=%d/%d\nscenarios_pass=%d/%d\n",R.pass,sum(Found),n,sum(Pass),n);
    if any(Found)
        fprintf(fid,"worst_noiseless_rmse_mps=%.9g\n",max(NoiselessRmse,[],'omitnan'));
        fprintf(fid,"worst_noisy_rmse_mean_mps=%.9g\n",max(NoisyRmse,[],'omitnan'));
        fprintf(fid,"worst_noisy_p95_mean_mps=%.9g\n",max(NoisyP95,[],'omitnan'));
        fprintf(fid,"worst_bias_mps=%.9g\n",max(Bias,[],'omitnan'));
        fprintf(fid,"worst_step_settling90_s=%.9g\n",max(Settle90,[],'omitnan'));
        fprintf(fid,"min_noise_reduction_ratio=%.9g\n",min(NoiseReduction,[],'omitnan'));
        fprintf(fid,"min_sign_accuracy=%.9g\n",min(SignAccuracy,[],'omitnan'));
        fprintf(fid,"worst_crosswind_truth_p95_mps=%.9g\n",max(CrosswindP95,[],'omitnan'));
    end
end
localPlot(T,outputRoot);
end
function localPlot(T,out)
try
    f=figure("Visible","off","Position",[100 100 1500 850]); tl=tiledlayout(f,2,2,"TileSpacing","compact","Padding","compact"); title(tl,"AirdropX v1.1.0 longitudinal wind sensing validation");
    nexttile; bar(categorical(T.Scenario),T.NoisyRmse); ylabel("MC RMSE (m/s)"); yline(.35,"--"); grid on;
    nexttile; bar(categorical(T.Scenario),T.NoisyP95); ylabel("MC p95 |error| (m/s)"); yline(.75,"--"); grid on;
    nexttile; bar(categorical(T.Scenario),T.Settle90); ylabel("90% step settle (s)"); yline(.5,"--"); grid on;
    nexttile; bar(categorical(T.Scenario),T.NoiseReduction); ylabel("raw/filter RMSE ratio"); yline(1.5,"--"); grid on;
    exportgraphics(f,fullfile(out,"wind_validation.png"),"Resolution",160); close(f);
catch ME, warning("AirdropX:Wind:PlotFailed","Aggregate wind plot failed: %s",ME.message); end
end
