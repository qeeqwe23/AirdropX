function report=airdropx_phys_finalize_envelope_v080(projectRoot,opts)
%AIRDROPX_PHYS_FINALIZE_ENVELOPE_V080 Aggregate 95 nonlinear PreviewOnly HxV missions.
arguments
    projectRoot (1,1) string %#ok<INUSA>
    opts.OutputRoot (1,1) string
    opts.BankPath (1,1) string
    opts.Heights_m (1,:) double = 20:10:200
    opts.Speeds_mps (1,:) double = [45 50 55 60 65]
end
H=sort(unique(double(opts.Heights_m(:)))); V=sort(unique(double(opts.Speeds_mps(:))));
if numel(H)~=19 || any(abs(H-(20:10:200)')>1e-12), error("AirdropX:PhysMPC:EnvelopeHeight","Formal envelope requires H=20:10:200 m."); end
if numel(V)~=5 || any(abs(V-[45;50;55;60;65])>1e-12), error("AirdropX:PhysMPC:EnvelopeSpeed","Formal envelope requires V=[45 50 55 60 65] m/s."); end
n=numel(H)*numel(V); Hcol=zeros(n,1); Vcol=zeros(n,1); Scenario=strings(n,1); ResultFound=false(n,1); MarkerFound=false(n,1); Pass=false(n,1);
PeakH=nan(n,1); PeakVa=nan(n,1); PeakGamma=nan(n,1); PeakTheta=nan(n,1); PeakQ=nan(n,1); PeakNorm=nan(n,1); FinalNorm=nan(n,1); TailRms=nan(n,1); Recovery=nan(n,1); QpSuccess=nan(n,1); QpP95=nan(n,1); QpMax=nan(n,1); PredP95=nan(n,1); PredMax=nan(n,1); MassErr=nan(n,1); CgErr=nan(n,1); IyyErr=nan(n,1); CargoErr=nan(n,1);
GateCommon=false(n,1); GateDrops=false(n,1); GateMass=false(n,1); GateCargo=false(n,1); GateQP=false(n,1); GateInput=false(n,1); GateFinite=false(n,1); GatePeak=false(n,1); GateFinal=false(n,1); GateTail=false(n,1); GateRealtime=false(n,1);
i=0;
for vv=V'
    for hh=H'
        i=i+1; Hcol(i)=hh; Vcol(i)=vv; dirHV=fullfile(opts.OutputRoot,sprintf("H%03d_V%03d",round(hh),round(vv)));
        MarkerFound(i)=isfile(fullfile(dirHV,"scenario_complete.ok")); p=fullfile(dirHV,"preview_mission.mat"); if ~isfile(p), continue; end
        S=load(p,"report"); if ~isfield(S,"report") || ~isstruct(S.report), continue; end
        r=S.report; ResultFound(i)=true; Pass(i)=logical(r.pass); Scenario(i)=string(r.scenario_name); m=r.metrics; g=r.gate;
        PeakH(i)=m.peak_h_err_m; PeakVa(i)=m.peak_Va_err_mps; PeakGamma(i)=m.peak_gamma_err_deg; PeakTheta(i)=m.peak_theta_err_deg; PeakQ(i)=m.peak_q_err_dps; PeakNorm(i)=m.peak_primary_normalized; FinalNorm(i)=m.final_normalized_inf; TailRms(i)=m.tail5s_normalized_rms; Recovery(i)=m.recovery_time_after_last_drop_s; QpSuccess(i)=m.qp_success_fraction; QpP95(i)=m.qp_time_p95_ms; QpMax(i)=m.qp_time_max_ms; PredP95(i)=m.prediction_error_norm_p95; PredMax(i)=m.prediction_error_norm_max; MassErr(i)=m.mass_match_error_max_kg; CgErr(i)=m.cg_match_error_max_m; IyyErr(i)=m.Iyy_match_error_max_kgm2; CargoErr(i)=m.drop_mass_error_max_kg;
        GateCommon(i)=g.common_controller; GateDrops(i)=g.four_drops; GateMass(i)=g.mass_configuration; GateCargo(i)=g.cargo_mass; GateQP(i)=g.qp_all_feasible; GateInput(i)=g.hard_input_bounds; GateFinite(i)=g.finite; GatePeak(i)=g.peak_primary; GateFinal(i)=g.final_normalized; GateTail(i)=g.tail5s; GateRealtime(i)=g.realtime_p95;
    end
end
Rows=table(Hcol,Vcol,Scenario,ResultFound,MarkerFound,Pass,PeakH,PeakVa,PeakGamma,PeakTheta,PeakQ,PeakNorm,FinalNorm,TailRms,Recovery,QpSuccess,QpP95,QpMax,PredP95,PredMax,MassErr,CgErr,IyyErr,CargoErr, ...
    'VariableNames',{'H_m','V_mps','scenario','result_found','marker_found','pass','peak_h_err_m','peak_Va_err_mps','peak_gamma_err_deg','peak_theta_err_deg','peak_q_err_dps','peak_primary_normalized','final_normalized_inf','tail5s_normalized_rms','recovery_time_after_last_drop_s','qp_success_fraction','qp_time_p95_ms','qp_time_max_ms','prediction_error_norm_p95','prediction_error_norm_max','mass_match_error_max_kg','cg_match_error_max_m','Iyy_match_error_max_kgm2','drop_mass_error_max_kg'});
GateMatrix=table(Hcol,Vcol,GateCommon,GateDrops,GateMass,GateCargo,GateQP,GateInput,GateFinite,GatePeak,GateFinal,GateTail,GateRealtime,Pass, ...
    'VariableNames',{'H_m','V_mps','common_controller','four_drops','mass_configuration','cargo_mass','qp_all_feasible','hard_input_bounds','finite','peak_primary','final_normalized','tail5s','realtime_p95','mission_pass'});
writetable(Rows,fullfile(opts.OutputRoot,"envelope_validation.csv")); writetable(GateMatrix,fullfile(opts.OutputRoot,"envelope_gate_matrix.csv"));
B=load(opts.BankPath,"bankAudit","rows"); if ~isfield(B,"bankAudit") || height(B.rows)~=475, error("AirdropX:PhysMPC:BadEnvelopeBank","Merged envelope bank audit missing or wrong size."); end; bankAudit=B.bankAudit;
allResults=all(ResultFound & MarkerFound); allMissions=all(Pass); pass=allResults && allMissions && bankAudit.pass;
[wp,hp,vp]=localWorst(Rows.peak_primary_normalized,Rows.H_m,Rows.V_mps); [wq,hq,vq]=localWorst(Rows.peak_q_err_dps,Rows.H_m,Rows.V_mps); [wh,hh,vh]=localWorst(Rows.peak_h_err_m,Rows.H_m,Rows.V_mps); [wv,hv,vv]=localWorst(Rows.peak_Va_err_mps,Rows.H_m,Rows.V_mps); [wf,hf,vf]=localWorst(Rows.final_normalized_inf,Rows.H_m,Rows.V_mps); [wt,ht,vt]=localWorst(Rows.tail5s_normalized_rms,Rows.H_m,Rows.V_mps); [wr,hr,vr]=localWorst(Rows.recovery_time_after_last_drop_s,Rows.H_m,Rows.V_mps); [wqp,hqp,vqp]=localWorst(Rows.qp_time_p95_ms,Rows.H_m,Rows.V_mps); [wpr,hpr,vpr]=localWorst(Rows.prediction_error_norm_max,Rows.H_m,Rows.V_mps);
worst=struct("peak_primary_normalized",wp,"peak_primary_H_m",hp,"peak_primary_V_mps",vp,"peak_q_err_dps",wq,"peak_q_H_m",hq,"peak_q_V_mps",vq,"peak_h_err_m",wh,"peak_h_H_m",hh,"peak_h_V_mps",vh,"peak_Va_err_mps",wv,"peak_Va_H_m",hv,"peak_Va_V_mps",vv,"final_normalized_inf",wf,"final_H_m",hf,"final_V_mps",vf,"tail5s_normalized_rms",wt,"tail_H_m",ht,"tail_V_mps",vt,"recovery_time_s",wr,"recovery_H_m",hr,"recovery_V_mps",vr,"qp_p95_ms",wqp,"qp_H_m",hqp,"qp_V_mps",vqp,"prediction_error_norm_max",wpr,"prediction_H_m",hpr,"prediction_V_mps",vpr);
% Every speed must pass all 19 heights; every height must pass all 5 speeds.
SpeedPass=false(numel(V),1); SpeedPassCount=zeros(numel(V),1); SpeedFoundCount=zeros(numel(V),1); SpeedWorstPeak=nan(numel(V),1); SpeedWorstQ=nan(numel(V),1); SpeedWorstRecovery=nan(numel(V),1);
for j=1:numel(V), mask=Rows.V_mps==V(j); ok=Rows.pass(mask)&Rows.result_found(mask)&Rows.marker_found(mask); SpeedPassCount(j)=sum(ok); SpeedFoundCount(j)=sum(Rows.result_found(mask)&Rows.marker_found(mask)); SpeedPass(j)=SpeedPassCount(j)==19; SpeedWorstPeak(j)=max(Rows.peak_primary_normalized(mask)); SpeedWorstQ(j)=max(Rows.peak_q_err_dps(mask)); SpeedWorstRecovery(j)=max(Rows.recovery_time_after_last_drop_s(mask)); end
PerSpeed=table(V,SpeedPassCount,SpeedFoundCount,SpeedPass,SpeedWorstPeak,SpeedWorstQ,SpeedWorstRecovery,'VariableNames',{'V_mps','pass_count','found_count','pass_all_19_heights','worst_peak_primary_normalized','worst_peak_q_err_dps','worst_recovery_time_s'}); writetable(PerSpeed,fullfile(opts.OutputRoot,"envelope_per_speed.csv"));
HeightPass=false(numel(H),1); HeightPassCount=zeros(numel(H),1); HeightWorstPeak=nan(numel(H),1); HeightWorstQ=nan(numel(H),1);
for j=1:numel(H), mask=Rows.H_m==H(j); ok=Rows.pass(mask)&Rows.result_found(mask)&Rows.marker_found(mask); HeightPassCount(j)=sum(ok); HeightPass(j)=HeightPassCount(j)==5; HeightWorstPeak(j)=max(Rows.peak_primary_normalized(mask)); HeightWorstQ(j)=max(Rows.peak_q_err_dps(mask)); end
PerHeight=table(H,HeightPassCount,HeightPass,HeightWorstPeak,HeightWorstQ,'VariableNames',{'H_m','pass_count','pass_all_5_speeds','worst_peak_primary_normalized','worst_peak_q_err_dps'}); writetable(PerHeight,fullfile(opts.OutputRoot,"envelope_per_height.csv"));
FailRows=Rows(~Rows.pass | ~Rows.result_found | ~Rows.marker_found,:); writetable(FailRows,fullfile(opts.OutputRoot,"envelope_failures.csv"));
report=struct("version","Physics-MPC v0.8.0 full HxV envelope validation","pass",pass,"completed_at",datetime("now"),"rows",Rows,"gate_matrix",GateMatrix,"per_speed",PerSpeed,"per_height",PerHeight,"bank_audit",bankAudit,"worst",worst);
save(fullfile(opts.OutputRoot,"envelope_validation.mat"),"report","Rows","GateMatrix","PerSpeed","PerHeight"); localWriteSummary(report,fullfile(opts.OutputRoot,"envelope_validation_summary.txt")); localPlot(Rows,opts.OutputRoot);
end
function [v,h,s]=localWorst(x,H,V)
x=double(x); ok=isfinite(x); if ~any(ok), v=NaN; h=NaN; s=NaN; return; end; xx=x; xx(~ok)=-inf; [v,i]=max(xx); h=H(i); s=V(i);
end
function localWriteSummary(r,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v0.8.0 PreviewOnly full HxV envelope validation\n"); fprintf(fid,"grid_H_m=20:10:200\ngrid_V_mps=45 50 55 60 65\ndrop_schedule_s=10 10.2 10.4 10.6\nq_soft=0\n");
fprintf(fid,"pass=%d\nmissions_found=%d/95\nmissions_pass=%d/95\nmarkers_found=%d/95\n",r.pass,sum(r.rows.result_found),sum(r.rows.pass),sum(r.rows.marker_found));
a=r.bank_audit; fprintf(fid,"bank_audit_pass=%d\nbank_vertices=%d\ncommon_Q_max_diff=%.9g\ncommon_R_max_diff=%.9g\ncommon_state_scale_max_diff=%.9g\ncommon_input_scale_max_diff=%.9g\nhorizon_same=%d\nNp=%d\nNc=%d\nrho_cl_range=[%.9g %.9g]\ndA_rel_max=%.9g\ndB_rel_max=%.9g\n",a.pass,a.vertex_count,a.max_Q_diff,a.max_R_diff,a.max_state_scale_diff,a.max_input_scale_diff,a.same_horizon,a.Np,a.Nc,a.rho_min,a.rho_max,a.dA_rel_max,a.dB_rel_max);
for j=1:height(r.per_speed), fprintf(fid,"speed_%.0f_pass=%d/19 found=%d/19 all_pass=%d worst_peak_norm=%.9g worst_q_dps=%.9g worst_recovery_s=%.9g\n",r.per_speed.V_mps(j),r.per_speed.pass_count(j),r.per_speed.found_count(j),r.per_speed.pass_all_19_heights(j),r.per_speed.worst_peak_primary_normalized(j),r.per_speed.worst_peak_q_err_dps(j),r.per_speed.worst_recovery_time_s(j)); end
fprintf(fid,"heights_pass_all_5_speeds=%d/19\n",sum(r.per_height.pass_all_5_speeds));
w=r.worst; fprintf(fid,"worst_peak_primary_normalized=%.9g at H=%.0f V=%.0f\n",w.peak_primary_normalized,w.peak_primary_H_m,w.peak_primary_V_mps); fprintf(fid,"worst_peak_q_err_dps=%.9g at H=%.0f V=%.0f\n",w.peak_q_err_dps,w.peak_q_H_m,w.peak_q_V_mps); fprintf(fid,"worst_peak_h_err_m=%.9g at H=%.0f V=%.0f\n",w.peak_h_err_m,w.peak_h_H_m,w.peak_h_V_mps); fprintf(fid,"worst_peak_Va_err_mps=%.9g at H=%.0f V=%.0f\n",w.peak_Va_err_mps,w.peak_Va_H_m,w.peak_Va_V_mps); fprintf(fid,"worst_final_normalized_inf=%.9g at H=%.0f V=%.0f\n",w.final_normalized_inf,w.final_H_m,w.final_V_mps); fprintf(fid,"worst_tail5s_normalized_rms=%.9g at H=%.0f V=%.0f\n",w.tail5s_normalized_rms,w.tail_H_m,w.tail_V_mps); fprintf(fid,"worst_recovery_time_s=%.9g at H=%.0f V=%.0f\n",w.recovery_time_s,w.recovery_H_m,w.recovery_V_mps); fprintf(fid,"worst_qp_p95_ms=%.9g at H=%.0f V=%.0f\n",w.qp_p95_ms,w.qp_H_m,w.qp_V_mps); fprintf(fid,"worst_prediction_error_norm_max=%.9g at H=%.0f V=%.0f\n",w.prediction_error_norm_max,w.prediction_H_m,w.prediction_V_mps);
fail=r.rows(~r.rows.pass | ~r.rows.result_found | ~r.rows.marker_found,{'H_m','V_mps'}); fprintf(fid,"failed_or_missing_HV="); for k=1:height(fail), fprintf(fid," H%.0f/V%.0f",fail.H_m(k),fail.V_mps(k)); end; fprintf(fid,"\n");
end
function localPlot(R,out)
try
    H=sort(unique(R.H_m)); V=sort(unique(R.V_mps));
    Zq=localGrid(R,H,V,'peak_q_err_dps'); Zn=localGrid(R,H,V,'peak_primary_normalized'); Zh=localGrid(R,H,V,'peak_h_err_m'); Zr=localGrid(R,H,V,'recovery_time_after_last_drop_s');
    fig=figure('Visible','off','Color','w','Position',[100 100 1300 900]); tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
    localHeat(nexttile(tl),V,H,Zq,'peak |q| (deg/s)'); localHeat(nexttile(tl),V,H,Zn,'peak normalized'); localHeat(nexttile(tl),V,H,Zh,'peak |h| (m)'); localHeat(nexttile(tl),V,H,Zr,'recovery after last drop (s)');
    title(tl,'Physics-MPC v0.8.0 PreviewOnly full HxV envelope: 0.2 s four-drop','Interpreter','none'); exportgraphics(fig,fullfile(out,'envelope_validation.png'),'Resolution',160); close(fig);
catch
end
end
function Z=localGrid(R,H,V,var)
Z=nan(numel(H),numel(V)); for i=1:numel(H), for j=1:numel(V), m=R.H_m==H(i)&R.V_mps==V(j); if nnz(m)==1, Z(i,j)=R.(var)(m); end, end, end
end
function localHeat(ax,V,H,Z,label)
imagesc(ax,V,H,Z); set(ax,'YDir','normal'); xlabel(ax,'Va (m/s)'); ylabel(ax,'H (m)'); title(ax,label); colorbar(ax); grid(ax,'on');
end
