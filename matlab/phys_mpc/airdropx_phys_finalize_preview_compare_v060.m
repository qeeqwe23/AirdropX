function report=airdropx_phys_finalize_preview_compare_v060(projectRoot,opts)
%AIRDROPX_PHYS_FINALIZE_PREVIEW_COMPARE_V060 Compare reactive, preview and preview+q-soft.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.PreviewDir (1,1) string
    opts.QSoftDir (1,1) string = ""
end
rows=table();
% Prefer the v0.5.2 comparison CSV because it already contains the correctly
% recomputed non-overlapping baseline metrics. Fall back to the v0.5.0 MAT.
baseCsv=fullfile(projectRoot,"matlab","results","physics_mpc_v052_drop_timing_compare","drop_timing_comparison.csv");
if isfile(baseCsv)
    B=readtable(baseCsv,TextType="string");
    ib=find(B.scenario=="baseline_0p2s",1);
    if ~isempty(ib), rows=[rows; localRowFromTable(B(ib,:))]; end %#ok<AGROW>
else
    base=fullfile(projectRoot,"matlab","results","physics_mpc_v050_four_drop","four_drop_mission.mat");
    if isfile(base)
        S=load(base,"report");
        % Older v0.5.0 reports did not yet store component peaks/recovery.
        % Only include them when those fields exist; otherwise omit baseline
        % rather than inventing values.
        need={"peak_h_err_m","peak_Va_err_mps","peak_gamma_err_deg","peak_theta_err_deg","peak_q_err_dps","recovery_time_after_last_drop_s"};
        if all(cellfun(@(x)isfield(S.report.metrics,x),need)), rows=[rows; localRow("reactive_0p2s",S.report)]; end %#ok<AGROW>
    end
end
p=fullfile(opts.PreviewDir,"preview_mission.mat"); if ~isfile(p), error("AirdropX:PhysMPC:MissingPreviewResult","Missing %s",p); end
S=load(p,"report"); rows=[rows; localRow("preview_only",S.report)]; %#ok<AGROW>
if opts.QSoftDir~=""
    q=fullfile(opts.QSoftDir,"preview_mission.mat"); if isfile(q), S=load(q,"report"); rows=[rows; localRow("preview_qsoft",S.report)]; end %#ok<AGROW>
end
writetable(rows,fullfile(opts.OutputRoot,"preview_comparison.csv"));
fid=fopen(fullfile(opts.OutputRoot,"preview_comparison.txt"),"w");
if fid>=0
    c=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,"Physics-MPC v0.6.0 close-spacing preview comparison\nAll scenarios use drop schedule 10/10.2/10.4/10.6 s and unchanged common Q/R/horizon/input bounds.\n\n");
    for i=1:height(rows)
        fprintf(fid,"[%s] pass=%d peakQ=%.6g deg/s peakNorm=%.6g peakH=%.6g m peakVa=%.6g m/s final=%.6g recovery=%.6g s QPp95=%.6g ms predMax=%.6g slackMax=%.6g rad/s\n", ...
            rows.scenario(i),rows.pass(i),rows.peak_q_err_dps(i),rows.peak_primary_normalized(i),rows.peak_h_err_m(i),rows.peak_Va_err_mps(i),rows.final_normalized_inf(i),rows.recovery_time_s(i),rows.qp_p95_ms(i),rows.pred_max(i),rows.q_soft_slack_max_radps(i));
    end
end
save(fullfile(opts.OutputRoot,"preview_comparison.mat"),"rows");
try
    fig=figure('Visible','off','Color','w','Position',[100 100 1200 700]);
    tl=tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
    ax=nexttile(tl); bar(ax,categorical(rows.scenario),rows.peak_q_err_dps); hold(ax,'on'); yline(ax,2,'--'); ylabel(ax,'peak |q| (deg/s)'); grid(ax,'on');
    ax=nexttile(tl); bar(ax,categorical(rows.scenario),rows.peak_primary_normalized); hold(ax,'on'); yline(ax,1,'--'); ylabel(ax,'peak normalized'); grid(ax,'on');
    title(tl,'Physics-MPC v0.6.0 close-spacing comparison','Interpreter','none');
    exportgraphics(fig,fullfile(opts.OutputRoot,'preview_comparison.png'),'Resolution',150); close(fig);
catch
end
report=struct("comparison",rows,"pass_preview",rows.pass(rows.scenario=="preview_only"));
end
function r=localRowFromTable(T)
r=table(string(T.scenario(1)),logical(T.pass(1)),T.peak_h_err_m(1),T.peak_Va_err_mps(1),T.peak_gamma_err_deg(1),T.peak_theta_err_deg(1),T.peak_q_err_dps(1),T.peak_primary_normalized(1),T.final_normalized_inf(1),T.tail5s_normalized_rms(1),T.recovery_time_after_last_drop_s(1),T.qp_time_p95_ms(1),T.prediction_error_norm_p95(1),T.prediction_error_norm_max(1),NaN, ...
    'VariableNames',{'scenario','pass','peak_h_err_m','peak_Va_err_mps','peak_gamma_err_deg','peak_theta_err_deg','peak_q_err_dps','peak_primary_normalized','final_normalized_inf','tail5s_normalized_rms','recovery_time_s','qp_p95_ms','pred_p95','pred_max','q_soft_slack_max_radps'});
end

function r=localRow(name,rep)
m=rep.metrics;
sl=NaN; if isfield(m,"q_soft_slack_max_radps"), sl=m.q_soft_slack_max_radps; end
r=table(string(name),logical(rep.pass),m.peak_h_err_m,m.peak_Va_err_mps,m.peak_gamma_err_deg,m.peak_theta_err_deg,m.peak_q_err_dps,m.peak_primary_normalized,m.final_normalized_inf,m.tail5s_normalized_rms,m.recovery_time_after_last_drop_s,m.qp_time_p95_ms,m.prediction_error_norm_p95,m.prediction_error_norm_max,sl, ...
    'VariableNames',{'scenario','pass','peak_h_err_m','peak_Va_err_mps','peak_gamma_err_deg','peak_theta_err_deg','peak_q_err_dps','peak_primary_normalized','final_normalized_inf','tail5s_normalized_rms','recovery_time_s','qp_p95_ms','pred_p95','pred_max','q_soft_slack_max_radps'});
end
