function report=airdropx_phys_finalize_envelope_v081(projectRoot,opts)
%AIRDROPX_PHYS_FINALIZE_ENVELOPE_V081 Aggregate 95 diagnostic PreviewOnly HxV cases.
% Separates formal physics certification from observed nonlinear mission
% performance. An uncertified vertex is never relabeled PASS.
arguments
    projectRoot (1,1) string %#ok<INUSA>
    opts.OutputRoot (1,1) string
    opts.BankPath (1,1) string
    opts.Heights_m (1,:) double = 20:10:200
    opts.Speeds_mps (1,:) double = [45 50 55 60 65]
end
H=sort(unique(double(opts.Heights_m(:)))); V=sort(unique(double(opts.Speeds_mps(:))));
if numel(H)~=19 || any(abs(H-(20:10:200)')>1e-12), error("AirdropX:PhysMPC:EnvelopeHeight","Requires H=20:10:200 m."); end
if numel(V)~=5 || any(abs(V-[45;50;55;60;65])>1e-12), error("AirdropX:PhysMPC:EnvelopeSpeed","Requires V=[45 50 55 60 65] m/s."); end
B=load(opts.BankPath,"bankAudit","rows"); if ~isfield(B,"bankAudit") || height(B.rows)~=475, error("AirdropX:PhysMPC:BadEnvelopeBank","Diagnostic bank audit missing or wrong size."); end
bankAudit=B.bankAudit; bankRows=B.rows; if ~ismember('usable',bankRows.Properties.VariableNames), bankRows.usable=bankRows.pass; end
n=95; Hcol=zeros(n,1); Vcol=zeros(n,1); Scenario=strings(n,1); ResultFound=false(n,1); MarkerFound=false(n,1); MissionExecuted=false(n,1); Pass=false(n,1);
ModelCertified=false(n,1); ModelUsable=false(n,1); CertifiedCfgCount=zeros(n,1); UsesUncertified=false(n,1); UncertifiedCfgs=strings(n,1);
PeakH=nan(n,1); PeakVa=nan(n,1); PeakGamma=nan(n,1); PeakTheta=nan(n,1); PeakQ=nan(n,1); PeakNorm=nan(n,1); FinalNorm=nan(n,1); TailRms=nan(n,1); Recovery=nan(n,1); QpSuccess=nan(n,1); QpP95=nan(n,1); QpMax=nan(n,1); PredP95=nan(n,1); PredMax=nan(n,1); MassErr=nan(n,1); CgErr=nan(n,1); IyyErr=nan(n,1); CargoErr=nan(n,1);
GateCommon=false(n,1); GateDrops=false(n,1); GateMass=false(n,1); GateCargo=false(n,1); GateQP=false(n,1); GateInput=false(n,1); GateFinite=false(n,1); GatePeak=false(n,1); GateFinal=false(n,1); GateTail=false(n,1); GateRealtime=false(n,1);
i=0;
for vv=V'
    for hh=H'
        i=i+1; Hcol(i)=hh; Vcol(i)=vv;
        mb=abs(bankRows.H_m-hh)<=1e-9 & abs(bankRows.V_mps-vv)<=1e-9;
        br=bankRows(mb,:); if height(br)~=5, error("AirdropX:PhysMPC:BadEnvelopeBank","Expected 5 cfg rows at H=%.0f V=%.0f.",hh,vv); end
        ModelCertified(i)=all(br.pass); ModelUsable(i)=all(br.usable); CertifiedCfgCount(i)=sum(br.pass); UsesUncertified(i)=ModelUsable(i)&&~ModelCertified(i);
        bc=br.cfg(~br.pass); if ~isempty(bc), UncertifiedCfgs(i)=join(string(bc'),","); end
        dirHV=fullfile(opts.OutputRoot,sprintf("H%03d_V%03d",round(hh),round(vv))); MarkerFound(i)=isfile(fullfile(dirHV,"scenario_complete.ok"));
        p=fullfile(dirHV,"preview_mission.mat"); pu=fullfile(dirHV,"preview_mission_unavailable.mat");
        if isfile(p)
            S=load(p,"report"); if ~isfield(S,"report") || ~isstruct(S.report), continue; end
            r=S.report; ResultFound(i)=true; MissionExecuted(i)=true; Pass(i)=logical(r.pass); Scenario(i)=string(r.scenario_name); m=r.metrics; g=r.gate;
            PeakH(i)=m.peak_h_err_m; PeakVa(i)=m.peak_Va_err_mps; PeakGamma(i)=m.peak_gamma_err_deg; PeakTheta(i)=m.peak_theta_err_deg; PeakQ(i)=m.peak_q_err_dps; PeakNorm(i)=m.peak_primary_normalized; FinalNorm(i)=m.final_normalized_inf; TailRms(i)=m.tail5s_normalized_rms; Recovery(i)=m.recovery_time_after_last_drop_s; QpSuccess(i)=m.qp_success_fraction; QpP95(i)=m.qp_time_p95_ms; QpMax(i)=m.qp_time_max_ms; PredP95(i)=m.prediction_error_norm_p95; PredMax(i)=m.prediction_error_norm_max; MassErr(i)=m.mass_match_error_max_kg; CgErr(i)=m.cg_match_error_max_m; IyyErr(i)=m.Iyy_match_error_max_kgm2; CargoErr(i)=m.drop_mass_error_max_kg;
            GateCommon(i)=g.common_controller; GateDrops(i)=g.four_drops; GateMass(i)=g.mass_configuration; GateCargo(i)=g.cargo_mass; GateQP(i)=g.qp_all_feasible; GateInput(i)=g.hard_input_bounds; GateFinite(i)=g.finite; GatePeak(i)=g.peak_primary; GateFinal(i)=g.final_normalized; GateTail(i)=g.tail5s; GateRealtime(i)=g.realtime_p95;
        elseif isfile(pu)
            ResultFound(i)=true; MissionExecuted(i)=false; Scenario(i)="MODEL_UNAVAILABLE"; Pass(i)=false;
        end
    end
end
Rows=table(Hcol,Vcol,Scenario,ResultFound,MarkerFound,MissionExecuted,Pass,ModelCertified,ModelUsable,CertifiedCfgCount,UsesUncertified,UncertifiedCfgs, ...
    PeakH,PeakVa,PeakGamma,PeakTheta,PeakQ,PeakNorm,FinalNorm,TailRms,Recovery,QpSuccess,QpP95,QpMax,PredP95,PredMax,MassErr,CgErr,IyyErr,CargoErr, ...
    'VariableNames',{'H_m','V_mps','scenario','result_found','marker_found','mission_executed','pass','model_certified','model_usable','certified_cfg_count','uses_uncertified_model','uncertified_cfgs', ...
    'peak_h_err_m','peak_Va_err_mps','peak_gamma_err_deg','peak_theta_err_deg','peak_q_err_dps','peak_primary_normalized','final_normalized_inf','tail5s_normalized_rms','recovery_time_after_last_drop_s','qp_success_fraction','qp_time_p95_ms','qp_time_max_ms','prediction_error_norm_p95','prediction_error_norm_max','mass_match_error_max_kg','cg_match_error_max_m','Iyy_match_error_max_kgm2','drop_mass_error_max_kg'});
GateMatrix=table(Hcol,Vcol,ModelCertified,ModelUsable,UsesUncertified,GateCommon,GateDrops,GateMass,GateCargo,GateQP,GateInput,GateFinite,GatePeak,GateFinal,GateTail,GateRealtime,Pass, ...
    'VariableNames',{'H_m','V_mps','model_certified','model_usable','uses_uncertified_model','common_controller','four_drops','mass_configuration','cargo_mass','qp_all_feasible','hard_input_bounds','finite','peak_primary','final_normalized','tail5s','realtime_p95','mission_pass'});
writetable(Rows,fullfile(opts.OutputRoot,"envelope_validation.csv")); writetable(GateMatrix,fullfile(opts.OutputRoot,"envelope_gate_matrix.csv"));

flowComplete=all(ResultFound & MarkerFound); all95Executed=all(MissionExecuted); diagnosticMissionPass=all95Executed && all(Pass); formalPass=logical(bankAudit.certification_pass) && diagnosticMissionPass;
uncertMask=UsesUncertified & MissionExecuted; uncertMissionCount=sum(uncertMask); uncertMissionPassCount=sum(Pass(uncertMask));
[wp,hp,vp]=localWorst(PeakNorm,Hcol,Vcol); [wq,hq,vq]=localWorst(PeakQ,Hcol,Vcol); [wh,hh,vh]=localWorst(PeakH,Hcol,Vcol); [wv,hv,vv2]=localWorst(PeakVa,Hcol,Vcol); [wr,hr,vr]=localWorst(Recovery,Hcol,Vcol); [wqp,hqp,vqp]=localWorst(QpP95,Hcol,Vcol); [wpr,hpr,vpr]=localWorst(PredMax,Hcol,Vcol);
worst=struct("peak_primary_normalized",wp,"peak_primary_H_m",hp,"peak_primary_V_mps",vp,"peak_q_err_dps",wq,"peak_q_H_m",hq,"peak_q_V_mps",vq, ...
    "peak_h_err_m",wh,"peak_h_H_m",hh,"peak_h_V_mps",vh,"peak_Va_err_mps",wv,"peak_Va_H_m",hv,"peak_Va_V_mps",vv2, ...
    "recovery_time_s",wr,"recovery_H_m",hr,"recovery_V_mps",vr,"qp_p95_ms",wqp,"qp_H_m",hqp,"qp_V_mps",vqp, ...
    "prediction_error_norm_max",wpr,"prediction_H_m",hpr,"prediction_V_mps",vpr);

SpeedPassCount=zeros(5,1); SpeedExecuted=zeros(5,1); SpeedCertified=zeros(5,1); SpeedWorstPeak=nan(5,1); SpeedWorstQ=nan(5,1);
for j=1:5, m=Vcol==V(j); SpeedPassCount(j)=sum(Pass(m)&MissionExecuted(m)); SpeedExecuted(j)=sum(MissionExecuted(m)); SpeedCertified(j)=sum(ModelCertified(m)); SpeedWorstPeak(j)=max(PeakNorm(m),[],'omitnan'); SpeedWorstQ(j)=max(PeakQ(m),[],'omitnan'); end
PerSpeed=table(V,SpeedPassCount,SpeedExecuted,SpeedCertified,SpeedWorstPeak,SpeedWorstQ,'VariableNames',{'V_mps','mission_pass_count','mission_executed_count','certified_H_count','worst_peak_primary_normalized','worst_peak_q_err_dps'}); writetable(PerSpeed,fullfile(opts.OutputRoot,"envelope_per_speed.csv"));
HeightPassCount=zeros(19,1); HeightExecuted=zeros(19,1); HeightCertified=zeros(19,1);
for j=1:19, m=Hcol==H(j); HeightPassCount(j)=sum(Pass(m)&MissionExecuted(m)); HeightExecuted(j)=sum(MissionExecuted(m)); HeightCertified(j)=sum(ModelCertified(m)); end
PerHeight=table(H,HeightPassCount,HeightExecuted,HeightCertified,'VariableNames',{'H_m','mission_pass_count','mission_executed_count','certified_speed_count'}); writetable(PerHeight,fullfile(opts.OutputRoot,"envelope_per_height.csv"));
writetable(Rows(~Pass | ~MissionExecuted,:),fullfile(opts.OutputRoot,"envelope_failures.csv")); writetable(Rows(UsesUncertified,:),fullfile(opts.OutputRoot,"envelope_uncertified_model_missions.csv"));
report=struct("version","Physics-MPC v0.8.1 diagnostic full-flow HxV envelope","pass",formalPass,"formal_envelope_pass",formalPass,"physics_certification_pass",logical(bankAudit.certification_pass), ...
    "diagnostic_flow_complete",flowComplete,"diagnostic_all_95_missions_executed",all95Executed,"diagnostic_mission_performance_pass",diagnosticMissionPass, ...
    "uncertified_model_mission_count",uncertMissionCount,"uncertified_model_mission_pass_count",uncertMissionPassCount, ...
    "completed_at",datetime("now"),"rows",Rows,"gate_matrix",GateMatrix,"per_speed",PerSpeed,"per_height",PerHeight,"bank_audit",bankAudit,"worst",worst);
save(fullfile(opts.OutputRoot,"envelope_validation.mat"),"report","Rows","GateMatrix","PerSpeed","PerHeight"); localWriteSummary(report,fullfile(opts.OutputRoot,"envelope_validation_summary.txt")); localPlot(Rows,opts.OutputRoot);
end

function [v,h,s]=localWorst(x,H,V)
x=double(x); ok=isfinite(x); if ~any(ok), v=NaN; h=NaN; s=NaN; return; end; xx=x; xx(~ok)=-inf; [v,i]=max(xx); h=H(i); s=V(i);
end
function localWriteSummary(r,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v0.8.1 diagnostic continue-on-certification-failure HxV envelope\n");
fprintf(fid,"grid_H_m=20:10:200\ngrid_V_mps=45 50 55 60 65\ndrop_schedule_s=10 10.2 10.4 10.6\nq_soft=0\n");
fprintf(fid,"formal_envelope_pass=%d\nphysics_certification_pass=%d\ndiagnostic_flow_complete=%d\ndiagnostic_all_95_missions_executed=%d\ndiagnostic_mission_performance_pass=%d\n",r.formal_envelope_pass,r.physics_certification_pass,r.diagnostic_flow_complete,r.diagnostic_all_95_missions_executed,r.diagnostic_mission_performance_pass);
fprintf(fid,"missions_found=%d/95\nmissions_executed=%d/95\nmissions_pass=%d/95\nmarkers_found=%d/95\n",sum(r.rows.result_found),sum(r.rows.mission_executed),sum(r.rows.pass),sum(r.rows.marker_found));
a=r.bank_audit; fprintf(fid,"bank_certification_pass=%d\nphysics_diagnostic_runnable=%d\nphysics_vertices_attempted=%d/475\nphysics_vertices_certified=%d/475\nphysics_vertices_usable=%d/475\nphysics_uncertified_usable=%d\nphysics_hard_unusable=%d\n",a.certification_pass,a.diagnostic_runnable,a.vertex_count,a.certified_count,a.usable_count,a.uncertified_usable_count,a.hard_unusable_count);
fprintf(fid,"common_controller_pass=%d\ncommon_Q_max_diff=%.9g\ncommon_R_max_diff=%.9g\nhorizon_same=%d\nNp=%d\nNc=%d\nrho_cl_range=[%.9g %.9g]\ndA_rel_max=%.9g\ndB_rel_max=%.9g\n",a.common_controller_pass,a.max_Q_diff,a.max_R_diff,a.same_horizon,a.Np,a.Nc,a.rho_min,a.rho_max,a.dA_rel_max,a.dB_rel_max);
fprintf(fid,"missions_using_uncertified_model=%d\nuncertified_model_missions_pass=%d/%d\n",r.uncertified_model_mission_count,r.uncertified_model_mission_pass_count,r.uncertified_model_mission_count);
for j=1:height(r.per_speed), fprintf(fid,"speed_%.0f: missions_pass=%d/19 executed=%d/19 certified_H=%d/19 worst_peak_norm=%.9g worst_q_dps=%.9g\n",r.per_speed.V_mps(j),r.per_speed.mission_pass_count(j),r.per_speed.mission_executed_count(j),r.per_speed.certified_H_count(j),r.per_speed.worst_peak_primary_normalized(j),r.per_speed.worst_peak_q_err_dps(j)); end
w=r.worst; fprintf(fid,"worst_peak_primary_normalized=%.9g at H=%.0f V=%.0f\n",w.peak_primary_normalized,w.peak_primary_H_m,w.peak_primary_V_mps); fprintf(fid,"worst_peak_q_err_dps=%.9g at H=%.0f V=%.0f\n",w.peak_q_err_dps,w.peak_q_H_m,w.peak_q_V_mps); fprintf(fid,"worst_peak_h_err_m=%.9g at H=%.0f V=%.0f\n",w.peak_h_err_m,w.peak_h_H_m,w.peak_h_V_mps); fprintf(fid,"worst_peak_Va_err_mps=%.9g at H=%.0f V=%.0f\n",w.peak_Va_err_mps,w.peak_Va_H_m,w.peak_Va_V_mps); fprintf(fid,"worst_recovery_time_s=%.9g at H=%.0f V=%.0f\n",w.recovery_time_s,w.recovery_H_m,w.recovery_V_mps); fprintf(fid,"worst_qp_p95_ms=%.9g at H=%.0f V=%.0f\n",w.qp_p95_ms,w.qp_H_m,w.qp_V_mps); fprintf(fid,"worst_prediction_error_norm_max=%.9g at H=%.0f V=%.0f\n",w.prediction_error_norm_max,w.prediction_H_m,w.prediction_V_mps);
unc=r.rows(r.rows.uses_uncertified_model,{'H_m','V_mps','uncertified_cfgs','mission_executed','pass','peak_q_err_dps','peak_primary_normalized'}); fprintf(fid,"uncertified_model_cases="); for k=1:height(unc), fprintf(fid," H%.0f/V%.0f/cfg[%s]:executed=%d:pass=%d:q=%.6g:norm=%.6g",unc.H_m(k),unc.V_mps(k),unc.uncertified_cfgs(k),unc.mission_executed(k),unc.pass(k),unc.peak_q_err_dps(k),unc.peak_primary_normalized(k)); end; fprintf(fid,"\n");
end
function localPlot(R,out)
try
    H=sort(unique(R.H_m)); V=sort(unique(R.V_mps)); Zq=localGrid(R,H,V,'peak_q_err_dps'); Zn=localGrid(R,H,V,'peak_primary_normalized'); Zh=localGrid(R,H,V,'peak_h_err_m'); Zr=localGrid(R,H,V,'recovery_time_after_last_drop_s');
    fig=figure('Visible','off','Color','w','Position',[100 100 1300 900]); tl=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact'); localHeat(nexttile(tl),V,H,Zq,'peak |q| (deg/s)'); localHeat(nexttile(tl),V,H,Zn,'peak normalized'); localHeat(nexttile(tl),V,H,Zh,'peak |h| (m)'); localHeat(nexttile(tl),V,H,Zr,'recovery after last drop (s)'); title(tl,'Physics-MPC v0.8.1 diagnostic full HxV flow','Interpreter','none'); exportgraphics(fig,fullfile(out,'envelope_validation.png'),'Resolution',160); close(fig);
catch
end
end
function Z=localGrid(R,H,V,var), Z=nan(numel(H),numel(V)); for i=1:numel(H), for j=1:numel(V), m=R.H_m==H(i)&R.V_mps==V(j); if nnz(m)==1, Z(i,j)=R.(var)(m); end, end, end, end
function localHeat(ax,V,H,Z,label), imagesc(ax,V,H,Z); set(ax,'YDir','normal'); xlabel(ax,'Va (m/s)'); ylabel(ax,'H (m)'); title(ax,label); colorbar(ax); grid(ax,'on'); end
