function report=airdropx_phys_finalize_drop_timing_compare_v052(projectRoot,opts)
%AIRDROPX_PHYS_FINALIZE_DROP_TIMING_COMPARE_V052 Combine already-completed isolated scenarios.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.IntervalDir (1,1) string
    opts.SimultaneousDir (1,1) string
    opts.IncludeExistingBaseline (1,1) logical = true
end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
[ri,Ti]=localLoadScenario(opts.IntervalDir,"interval_2s");
[rs,Ts]=localLoadScenario(opts.SimultaneousDir,"simultaneous_4x");
rows=[localScenarioRow(ri,"interval_2s","10 12 14 16"); localScenarioRow(rs,"simultaneous_4x","10 10 10 10")];
baselineLoaded=false; baselineT=table();
if opts.IncludeExistingBaseline
    baseMat=fullfile(projectRoot,"matlab","results","physics_mpc_v050_four_drop","four_drop_mission.mat");
    if isfile(baseMat)
        B=load(baseMat);
        if isfield(B,"report") && isfield(B,"T")
            baselineLoaded=true; baselineT=B.T;
            rows=[localBaselineRow(B.report,B.T); rows]; %#ok<AGROW>
        end
    end
end
writetable(rows,fullfile(opts.OutputRoot,"drop_timing_comparison.csv"));
plotWarning=""; plotPath="";
try
    plotPath=localPlotComparison(Ti,Ts,baselineT,baselineLoaded,opts.OutputRoot);
catch ME
    plotWarning=string(ME.identifier)+": "+string(ME.message);
end
localWriteComparison(rows,fullfile(opts.OutputRoot,"drop_timing_comparison.txt"),baselineLoaded);
report=struct("version","Physics-MPC v0.5.2 isolated drop-timing comparison", ...
    "completed_at",datetime("now"),"interval_report",ri,"simultaneous_report",rs, ...
    "comparison",rows,"baseline_loaded",baselineLoaded,"plot",plotPath,"plot_warning",plotWarning, ...
    "pass_both_new",logical(ri.pass)&&logical(rs.pass));
save(fullfile(opts.OutputRoot,"drop_timing_comparison.mat"),"report","-v7.3");
disp(rows);
fprintf("=== Physics-MPC v0.5.2 ISOLATED DROP-TIMING COMPARISON COMPLETE ===\n");
fprintf("2 s spacing pass=%d | simultaneous pass=%d\n",logical(ri.pass),logical(rs.pass));
end

function [r,T]=localLoadScenario(dirPath,name)
matPath=fullfile(dirPath,"four_drop_mission.mat"); csvPath=fullfile(dirPath,"four_drop_timeseries.csv");
if ~isfile(matPath), error("AirdropX:PhysMPC:ScenarioMissing","Scenario %s result missing: %s",name,matPath); end
S=load(matPath,"report"); r=S.report;
if ~isfile(csvPath), error("AirdropX:PhysMPC:ScenarioCsvMissing","Scenario %s timeseries missing: %s",name,csvPath); end
T=readtable(csvPath);
end

function row=localScenarioRow(r,name,schedule)
m=r.metrics;
row=table(string(name),string(schedule),logical(r.pass), ...
    m.peak_h_err_m,m.peak_Va_err_mps,m.peak_gamma_err_deg,m.peak_theta_err_deg,m.peak_q_err_dps,m.peak_primary_normalized, ...
    m.final_normalized_inf,m.tail5s_normalized_rms,m.recovery_time_after_last_drop_s, ...
    m.qp_time_p95_ms,m.qp_time_max_ms,m.pred_error_norm_p95,m.pred_error_norm_max, ...
    m.elevator_saturation_fraction,m.throttle_saturation_fraction,m.total_observed_drop_kg,m.drop_mass_error_max_kg, ...
    'VariableNames',localVarNames());
end

function row=localBaselineRow(r,T)
stateScale=[4;2;deg2rad(3);deg2rad(5);deg2rad(2)];
primary=max(abs([T.h_err_m,T.Va_err_mps,T.gamma_err_rad,T.theta_err_rad,T.q_err_radps])./stateScale.',[],2);
lastDrop=10.6; threshold=0.05; recovery=NaN;
idx=find(T.t_s>=lastDrop-1e-10);
for j=1:numel(idx)
    k=idx(j); if primary(k)<=threshold && all(primary(k:end)<=threshold), recovery=T.t_s(k)-lastDrop; break; end
end
m=r.metrics; maxAbs=max(abs([T.h_err_m,T.Va_err_mps,T.gamma_err_rad,T.theta_err_rad,T.q_err_radps]),[],1);
row=table("baseline_0p2s","10 10.2 10.4 10.6",logical(r.pass), ...
    maxAbs(1),maxAbs(2),rad2deg(maxAbs(3)),rad2deg(maxAbs(4)),rad2deg(maxAbs(5)),max(primary), ...
    m.final_normalized_inf,m.tail5s_normalized_rms,recovery,m.qp_time_p95_ms,m.qp_time_max_ms,m.pred_error_norm_p95,m.pred_error_norm_max, ...
    m.elevator_saturation_fraction,m.throttle_saturation_fraction,sum(m.observed_drop_kg),m.drop_mass_error_max_kg, ...
    'VariableNames',localVarNames());
end

function names=localVarNames()
names={'scenario','drop_schedule_s','pass','peak_h_err_m','peak_Va_err_mps','peak_gamma_err_deg','peak_theta_err_deg','peak_q_err_dps', ...
    'peak_primary_normalized','final_normalized_inf','tail5s_normalized_rms','recovery_time_after_last_drop_s','qp_time_p95_ms','qp_time_max_ms', ...
    'prediction_error_norm_p95','prediction_error_norm_max','elevator_saturation_fraction','throttle_saturation_fraction','total_observed_drop_kg','drop_mass_error_max_kg'};
end

function path=localPlotComparison(Ti,Ts,B,hasB,out)
labels=strings(0,1); TT=cell(0,1);
if hasB, labels(end+1)="baseline 0.2 s"; TT{end+1}=B; end %#ok<AGROW>
labels(end+1)="interval 2 s"; TT{end+1}=Ti; %#ok<AGROW>
labels(end+1)="simultaneous 4x"; TT{end+1}=Ts; %#ok<AGROW>
f=figure('Visible','off','Color','w','Position',[80 80 1500 1000]);
tl=tiledlayout(f,3,2,'TileSpacing','compact','Padding','compact');
nexttile; hold on; for i=1:numel(TT), plot(TT{i}.t_s,TT{i}.h_err_m,'LineWidth',1.3); end; ylabel('h error (m)'); grid on; legend(labels,'Location','best');
nexttile; hold on; for i=1:numel(TT), plot(TT{i}.t_s,TT{i}.Va_err_mps,'LineWidth',1.3); end; ylabel('Va error (m/s)'); grid on;
nexttile; hold on; for i=1:numel(TT), plot(TT{i}.t_s,rad2deg(TT{i}.q_err_radps),'LineWidth',1.3); end; ylabel('q error (deg/s)'); grid on;
nexttile; hold on; for i=1:numel(TT), plot(TT{i}.t_s,TT{i}.mass_kg,'LineWidth',1.3); end; ylabel('mass (kg)'); grid on;
nexttile; hold on; for i=1:numel(TT), plot(TT{i}.t_s,TT{i}.elevator_cmd,'LineWidth',1.3); end; ylabel('elevator'); xlabel('t (s)'); grid on;
nexttile; hold on; for i=1:numel(TT), semilogy(TT{i}.t_s,max(TT{i}.pred_err_norm_inf,eps),'LineWidth',1.3); end; ylabel('1-step pred err / scale'); xlabel('t (s)'); grid on;
title(tl,'Physics-MPC drop timing comparison: isolated MATLAB processes');
path=fullfile(out,"drop_timing_comparison.png"); exportgraphics(f,path,'Resolution',170); close(f);
end

function localWriteComparison(T,path,baselineLoaded)
fid=fopen(path,'w'); if fid<0, return; end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v0.5.2 isolated drop timing comparison\n");
fprintf(fid,"Each new scenario ran in a separate MATLAB process; controller physics/Q/R/horizon/bounds unchanged.\n");
fprintf(fid,"Scenario A: 2 s spacing [10 12 14 16] s\n");
fprintf(fid,"Scenario B: simultaneous [10 10 10 10] s, cfg0->cfg4 in one sample\n");
fprintf(fid,"Existing v0.5.0 baseline included=%d\n\n",baselineLoaded);
for i=1:height(T)
    fprintf(fid,"[%s] pass=%d schedule=%s peakH=%.6g m peakVa=%.6g m/s peakGamma=%.6g deg peakTheta=%.6g deg peakQ=%.6g deg/s peakNorm=%.6g finalNorm=%.6g tail5=%.6g recovery=%.6g s QPp95=%.6g ms predP95=%.6g predMax=%.6g\n", ...
        T.scenario(i),T.pass(i),T.drop_schedule_s(i),T.peak_h_err_m(i),T.peak_Va_err_mps(i),T.peak_gamma_err_deg(i),T.peak_theta_err_deg(i),T.peak_q_err_dps(i), ...
        T.peak_primary_normalized(i),T.final_normalized_inf(i),T.tail5s_normalized_rms(i),T.recovery_time_after_last_drop_s(i),T.qp_time_p95_ms(i),T.prediction_error_norm_p95(i),T.prediction_error_norm_max(i));
end
end
