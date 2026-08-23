function report=airdropx_phys_finalize_altitude_validation_v061(projectRoot,opts)
%AIRDROPX_PHYS_FINALIZE_ALTITUDE_VALIDATION_V061 Aggregate 20:10:200 m PreviewOnly missions.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.BankPath (1,1) string = ""
    opts.Heights_m (1,:) double = 20:10:200
    opts.V_mps (1,1) double {mustBeFinite,mustBePositive} = 50
end
if opts.BankPath=="", opts.BankPath=fullfile(projectRoot,"matlab","results","physics_mpc_v033","physics_bank.mat"); end
H=sort(unique(double(opts.Heights_m(:))));
expected=(20:10:200)';
if numel(H)~=numel(expected) || any(abs(H-expected)>1e-12)
    error("AirdropX:PhysMPC:AltitudeGrid","Formal v0.6.1 validation requires exactly H=20:10:200 m.");
end
if abs(opts.V_mps-50)>1e-12
    error("AirdropX:PhysMPC:SpeedLocked","v0.6.1 altitude validation deliberately locks V=50 m/s.");
end
n=numel(H);
ResultFound=false(n,1); MarkerFound=false(n,1); Pass=false(n,1);
PeakH=nan(n,1); PeakVa=nan(n,1); PeakGamma=nan(n,1); PeakTheta=nan(n,1); PeakQ=nan(n,1); PeakNorm=nan(n,1);
FinalNorm=nan(n,1); TailRms=nan(n,1); Recovery=nan(n,1); QpP95=nan(n,1); QpMax=nan(n,1); PredP95=nan(n,1); PredMax=nan(n,1);
MassErr=nan(n,1); CgErr=nan(n,1); IyyErr=nan(n,1); CargoErr=nan(n,1); QpSuccess=nan(n,1);
GateCommon=false(n,1); GateDrops=false(n,1); GateMass=false(n,1); GateCargo=false(n,1); GateQP=false(n,1); GateInput=false(n,1); GateFinite=false(n,1); GatePeak=false(n,1); GateFinal=false(n,1); GateTail=false(n,1); GateRealtime=false(n,1);
Scenario=strings(n,1);
for i=1:n
    dirH=fullfile(opts.OutputRoot,sprintf("H%03d_V050",round(H(i))));
    MarkerFound(i)=isfile(fullfile(dirH,"scenario_complete.ok"));
    p=fullfile(dirH,"preview_mission.mat");
    if ~isfile(p), continue; end
    S=load(p,"report"); if ~isfield(S,"report") || ~isstruct(S.report), continue; end
    r=S.report; ResultFound(i)=true; Pass(i)=logical(r.pass); Scenario(i)=string(r.scenario_name); m=r.metrics; g=r.gate;
    PeakH(i)=m.peak_h_err_m; PeakVa(i)=m.peak_Va_err_mps; PeakGamma(i)=m.peak_gamma_err_deg; PeakTheta(i)=m.peak_theta_err_deg; PeakQ(i)=m.peak_q_err_dps; PeakNorm(i)=m.peak_primary_normalized;
    FinalNorm(i)=m.final_normalized_inf; TailRms(i)=m.tail5s_normalized_rms; Recovery(i)=m.recovery_time_after_last_drop_s; QpP95(i)=m.qp_time_p95_ms; QpMax(i)=m.qp_time_max_ms;
    PredP95(i)=m.prediction_error_norm_p95; PredMax(i)=m.prediction_error_norm_max; MassErr(i)=m.mass_match_error_max_kg; CgErr(i)=m.cg_match_error_max_m; IyyErr(i)=m.Iyy_match_error_max_kgm2; CargoErr(i)=m.drop_mass_error_max_kg; QpSuccess(i)=m.qp_success_fraction;
    GateCommon(i)=g.common_controller; GateDrops(i)=g.four_drops; GateMass(i)=g.mass_configuration; GateCargo(i)=g.cargo_mass; GateQP(i)=g.qp_all_feasible; GateInput(i)=g.hard_input_bounds; GateFinite(i)=g.finite; GatePeak(i)=g.peak_primary; GateFinal(i)=g.final_normalized; GateTail(i)=g.tail5s; GateRealtime(i)=g.realtime_p95;
end
Rows=table(H,repmat(opts.V_mps,n,1),Scenario,ResultFound,MarkerFound,Pass,PeakH,PeakVa,PeakGamma,PeakTheta,PeakQ,PeakNorm,FinalNorm,TailRms,Recovery,QpSuccess,QpP95,QpMax,PredP95,PredMax,MassErr,CgErr,IyyErr,CargoErr, ...
    'VariableNames',{'H_m','V_mps','scenario','result_found','marker_found','pass','peak_h_err_m','peak_Va_err_mps','peak_gamma_err_deg','peak_theta_err_deg','peak_q_err_dps','peak_primary_normalized','final_normalized_inf','tail5s_normalized_rms','recovery_time_after_last_drop_s','qp_success_fraction','qp_time_p95_ms','qp_time_max_ms','prediction_error_norm_p95','prediction_error_norm_max','mass_match_error_max_kg','cg_match_error_max_m','Iyy_match_error_max_kgm2','drop_mass_error_max_kg'});
GateMatrix=table(H,GateCommon,GateDrops,GateMass,GateCargo,GateQP,GateInput,GateFinite,GatePeak,GateFinal,GateTail,GateRealtime,Pass, ...
    'VariableNames',{'H_m','common_controller','four_drops','mass_configuration','cargo_mass','qp_all_feasible','hard_input_bounds','finite','peak_primary','final_normalized','tail5s','realtime_p95','mission_pass'});
writetable(Rows,fullfile(opts.OutputRoot,"altitude_validation.csv"));
writetable(GateMatrix,fullfile(opts.OutputRoot,"altitude_gate_matrix.csv"));

% Cross-height bank audit: same Q/R/scales/horizon over all 95 H x cfg vertices.
base=[]; maxQ=0; maxR=0; maxSS=0; maxUS=0; sameHorizon=true; bankCount=0; rhoMin=inf; rhoMax=-inf;
for i=1:n
    for cfg=0:4
        [v,~,~]=airdropx_phys_mpc_get_vertex(opts.BankPath,H(i),opts.V_mps,cfg,1.0);
        q=double(v.Q); rr=double(v.R); ss=double(v.qrMeta.StateScale(:)); us=double(v.qrMeta.InputScale(:)); np=double(v.terminal.Np); nc=double(v.terminal.Nc); rho=double(v.terminal.rho);
        if isempty(base), base=struct("Q",q,"R",rr,"ss",ss,"us",us,"Np",np,"Nc",nc); else
            maxQ=max(maxQ,norm(q-base.Q,inf)); maxR=max(maxR,norm(rr-base.R,inf)); maxSS=max(maxSS,norm(ss-base.ss,inf)); maxUS=max(maxUS,norm(us-base.us,inf)); sameHorizon=sameHorizon && np==base.Np && nc==base.Nc;
        end
        rhoMin=min(rhoMin,rho); rhoMax=max(rhoMax,rho); bankCount=bankCount+1;
    end
end
bankAudit=struct("pass",bankCount==95 && maxQ<=1e-12 && maxR<=1e-12 && maxSS<=1e-12 && maxUS<=1e-12 && sameHorizon, ...
    "vertex_count",bankCount,"max_Q_diff",maxQ,"max_R_diff",maxR,"max_state_scale_diff",maxSS,"max_input_scale_diff",maxUS,"same_horizon",sameHorizon,"Np",base.Np,"Nc",base.Nc,"rho_min",rhoMin,"rho_max",rhoMax);

allResults=all(ResultFound & MarkerFound); allMissions=all(Pass); pass=allResults && allMissions && bankAudit.pass;
[worstPeakNorm,hPeakNorm]=localWorst(Rows.peak_primary_normalized,Rows.H_m);
[worstQ,hQ]=localWorst(Rows.peak_q_err_dps,Rows.H_m); [worstH,hH]=localWorst(Rows.peak_h_err_m,Rows.H_m); [worstVa,hVa]=localWorst(Rows.peak_Va_err_mps,Rows.H_m);
[worstFinal,hFinal]=localWorst(Rows.final_normalized_inf,Rows.H_m); [worstTail,hTail]=localWorst(Rows.tail5s_normalized_rms,Rows.H_m); [worstRecovery,hRecovery]=localWorst(Rows.recovery_time_after_last_drop_s,Rows.H_m);
[worstQP,hQP]=localWorst(Rows.qp_time_p95_ms,Rows.H_m); [worstPred,hPred]=localWorst(Rows.prediction_error_norm_max,Rows.H_m);
worst=struct("peak_primary_normalized",worstPeakNorm,"peak_primary_H_m",hPeakNorm,"peak_q_err_dps",worstQ,"peak_q_H_m",hQ,"peak_h_err_m",worstH,"peak_h_H_m",hH,"peak_Va_err_mps",worstVa,"peak_Va_H_m",hVa, ...
    "final_normalized_inf",worstFinal,"final_H_m",hFinal,"tail5s_normalized_rms",worstTail,"tail_H_m",hTail,"recovery_time_s",worstRecovery,"recovery_H_m",hRecovery,"qp_p95_ms",worstQP,"qp_H_m",hQP,"prediction_error_norm_max",worstPred,"prediction_H_m",hPred);
report=struct("version","Physics-MPC v0.6.1 altitude validation","pass",pass,"completed_at",datetime("now"),"rows",Rows,"gate_matrix",GateMatrix,"bank_audit",bankAudit,"worst",worst);
save(fullfile(opts.OutputRoot,"altitude_validation.mat"),"report","Rows","GateMatrix");
localWriteSummary(report,fullfile(opts.OutputRoot,"altitude_validation_summary.txt"));
localPlot(Rows,opts.OutputRoot);
end

function [v,h]=localWorst(x,H)
x=double(x); H=double(H); ok=isfinite(x); if ~any(ok), v=NaN; h=NaN; return; end
xx=x; xx(~ok)=-inf; [v,idx]=max(xx); h=H(idx);
end

function localWriteSummary(r,path)
fid=fopen(path,"w"); if fid<0, return; end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v0.6.1 PreviewOnly altitude validation\n");
fprintf(fid,"grid_H_m=20:10:200\nV_mps=50\ndrop_schedule_s=10 10.2 10.4 10.6\nq_soft=0\n");
fprintf(fid,"pass=%d\nmissions_found=%d/19\nmissions_pass=%d/19\nmarkers_found=%d/19\n",r.pass,sum(r.rows.result_found),sum(r.rows.pass),sum(r.rows.marker_found));
a=r.bank_audit; fprintf(fid,"bank_audit_pass=%d\nbank_vertices=%d\ncommon_Q_max_diff=%.9g\ncommon_R_max_diff=%.9g\ncommon_state_scale_max_diff=%.9g\ncommon_input_scale_max_diff=%.9g\nhorizon_same=%d\nNp=%d\nNc=%d\nrho_cl_range=[%.9g %.9g]\n",a.pass,a.vertex_count,a.max_Q_diff,a.max_R_diff,a.max_state_scale_diff,a.max_input_scale_diff,a.same_horizon,a.Np,a.Nc,a.rho_min,a.rho_max);
w=r.worst;
fprintf(fid,"worst_peak_primary_normalized=%.9g at H=%.0f m\n",w.peak_primary_normalized,w.peak_primary_H_m);
fprintf(fid,"worst_peak_q_err_dps=%.9g at H=%.0f m\n",w.peak_q_err_dps,w.peak_q_H_m);
fprintf(fid,"worst_peak_h_err_m=%.9g at H=%.0f m\n",w.peak_h_err_m,w.peak_h_H_m);
fprintf(fid,"worst_peak_Va_err_mps=%.9g at H=%.0f m\n",w.peak_Va_err_mps,w.peak_Va_H_m);
fprintf(fid,"worst_final_normalized_inf=%.9g at H=%.0f m\n",w.final_normalized_inf,w.final_H_m);
fprintf(fid,"worst_tail5s_normalized_rms=%.9g at H=%.0f m\n",w.tail5s_normalized_rms,w.tail_H_m);
fprintf(fid,"worst_recovery_time_s=%.9g at H=%.0f m\n",w.recovery_time_s,w.recovery_H_m);
fprintf(fid,"worst_qp_p95_ms=%.9g at H=%.0f m\n",w.qp_p95_ms,w.qp_H_m);
fprintf(fid,"worst_prediction_error_norm_max=%.9g at H=%.0f m\n",w.prediction_error_norm_max,w.prediction_H_m);
failH=r.rows.H_m(~r.rows.pass | ~r.rows.result_found | ~r.rows.marker_found); fprintf(fid,"failed_or_missing_heights_m="); fprintf(fid," %.0f",failH); fprintf(fid,"\n");
end

function localPlot(R,out)
try
    fig=figure('Visible','off','Color','w','Position',[100 100 1250 850]); tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
    ax=nexttile(tl); plot(ax,R.H_m,R.peak_q_err_dps,'-o'); hold(ax,'on'); yline(ax,2,'--'); xlabel(ax,'H (m)'); ylabel(ax,'peak |q| (deg/s)'); grid(ax,'on');
    ax=nexttile(tl); plot(ax,R.H_m,R.peak_primary_normalized,'-o'); hold(ax,'on'); yline(ax,1,'--'); xlabel(ax,'H (m)'); ylabel(ax,'peak normalized'); grid(ax,'on');
    ax=nexttile(tl); plot(ax,R.H_m,R.peak_h_err_m,'-o'); hold(ax,'on'); plot(ax,R.H_m,R.peak_Va_err_mps,'-s'); xlabel(ax,'H (m)'); ylabel(ax,'peak error'); legend(ax,{'|h| m','|Va| m/s'},'Location','best'); grid(ax,'on');
    ax=nexttile(tl); plot(ax,R.H_m,R.recovery_time_after_last_drop_s,'-o'); xlabel(ax,'H (m)'); ylabel(ax,'recovery after last drop (s)'); grid(ax,'on');
    title(tl,'Physics-MPC v0.6.1 PreviewOnly altitude validation: V=50 m/s, 0.2 s four-drop','Interpreter','none');
    exportgraphics(fig,fullfile(out,'altitude_validation.png'),'Resolution',150); close(fig);
catch
end
end
