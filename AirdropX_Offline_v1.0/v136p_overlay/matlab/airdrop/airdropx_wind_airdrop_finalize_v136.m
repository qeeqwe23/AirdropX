function R=airdropx_wind_airdrop_finalize_v136(root,manifestPath)
%AIRDROPX_WIND_AIRDROP_FINALIZE_V136 Aggregate v1.3.6 3-way comparisons, transient-evidence activity, and sustained-forcing diagnostics.
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
        tail5=NaN; finalNorm=NaN; r05=NaN; r10=NaN; r30=NaN; windActive=NaN; recoveryActive=NaN; recoveryPeak=NaN; elevSat=NaN; thrSat=NaN; elevHead=NaN; thrHead=NaN; confMean=NaN;
        abruptCount=NaN; energyActive=NaN; energyShift=NaN; inputSnap=NaN; disturbanceEvidence=NaN; forcedRms=NaN; forcedPeak=NaN;
        releaseEvidence=NaN; carrierEvidence=NaN; carrierSwitch=NaN; windSwitch=NaN; lastCarrier=NaN; lastWindMpc=NaN; tailBase=NaN; tailZero=NaN; zeroSettle=NaN;
        if found
            S=load(p,"report"); r=S.report; pass=logical(r.pass); drops=r.metrics.drops_completed; maxLand=r.metrics.max_landing_error_m; rmsLand=r.metrics.rms_landing_error_m; windP95=r.metrics.wind_error_p95_mps;
            instPeak=r.metrics.instantaneous_peak_primary_normalized; causalPeak=r.metrics.post_gust_peak_normalized; rec=r.metrics.max_gust_recovery_time_s; postRms=r.metrics.post_wind_primary_rms; qp=r.metrics.qp_p95_ms; pred=r.metrics.prediction_error_norm_p95;
            if isfield(r.metrics,"fractional_release_count"), fracCount=r.metrics.fractional_release_count; end
            if isfield(r.metrics,"release_scheduler_residual_max_m"), schedResidual=r.metrics.release_scheduler_residual_max_m; end
            if isfield(r.metrics,"tail5s_normalized_rms"), tail5=r.metrics.tail5s_normalized_rms; end
            if isfield(r.metrics,"final_normalized_inf"), finalNorm=r.metrics.final_normalized_inf; end
            if isfield(r.metrics,"gust_residual_0p5_normalized"), r05=r.metrics.gust_residual_0p5_normalized; end
            if isfield(r.metrics,"gust_residual_1p0_normalized"), r10=r.metrics.gust_residual_1p0_normalized; end
            if isfield(r.metrics,"gust_residual_3p0_normalized"), r30=r.metrics.gust_residual_3p0_normalized; end
            if isfield(r.metrics,"wind_mpc_active_fraction"), windActive=r.metrics.wind_mpc_active_fraction; end
            if isfield(r.metrics,"recovery_active_fraction"), recoveryActive=r.metrics.recovery_active_fraction; end
            if isfield(r.metrics,"recovery_level_peak"), recoveryPeak=r.metrics.recovery_level_peak; end
            if isfield(r.metrics,"elevator_saturation_fraction"), elevSat=r.metrics.elevator_saturation_fraction; end
            if isfield(r.metrics,"throttle_saturation_fraction"), thrSat=r.metrics.throttle_saturation_fraction; end
            if isfield(r.metrics,"elevator_min_headroom"), elevHead=r.metrics.elevator_min_headroom; end
            if isfield(r.metrics,"throttle_min_headroom"), thrHead=r.metrics.throttle_min_headroom; end
            if isfield(r.metrics,"wind_confidence_mean"), confMean=r.metrics.wind_confidence_mean; end
            if isfield(r.metrics,"abrupt_gust_trigger_count"), abruptCount=r.metrics.abrupt_gust_trigger_count; end
            if isfield(r.metrics,"energy_recovery_active_fraction"), energyActive=r.metrics.energy_recovery_active_fraction; end
            if isfield(r.metrics,"energy_altitude_shift_peak_m"), energyShift=r.metrics.energy_altitude_shift_peak_m; end
            if isfield(r.metrics,"input_snap_max"), inputSnap=r.metrics.input_snap_max; end
            if isfield(r.metrics,"disturbance_evidence_active_fraction"), disturbanceEvidence=r.metrics.disturbance_evidence_active_fraction; end
            if isfield(r.metrics,"forced_response_primary_rms"), forcedRms=r.metrics.forced_response_primary_rms; end
            if isfield(r.metrics,"forced_response_peak_normalized"), forcedPeak=r.metrics.forced_response_peak_normalized; end
            if isfield(r.metrics,"release_wind_evidence_active_fraction"), releaseEvidence=r.metrics.release_wind_evidence_active_fraction; end
            if isfield(r.metrics,"carrier_transient_evidence_active_fraction"), carrierEvidence=r.metrics.carrier_transient_evidence_active_fraction; end
            if isfield(r.metrics,"carrier_transient_switch_count"), carrierSwitch=r.metrics.carrier_transient_switch_count; end
            if isfield(r.metrics,"wind_mpc_switch_count"), windSwitch=r.metrics.wind_mpc_switch_count; end
            if isfield(r.metrics,"last_carrier_transient_time_s"), lastCarrier=r.metrics.last_carrier_transient_time_s; end
            if isfield(r.metrics,"last_wind_mpc_active_time_s"), lastWindMpc=r.metrics.last_wind_mpc_active_time_s; end
            if isfield(r.metrics,"tail5_base_solver_fraction"), tailBase=r.metrics.tail5_base_solver_fraction; end
            if isfield(r.metrics,"tail5_zero_truth_wind_fraction"), tailZero=r.metrics.tail5_zero_truth_wind_fraction; end
            if isfield(r.metrics,"zero_wind_settle_duration_s"), zeroSettle=r.metrics.zero_wind_settle_duration_s; end
        end
        one=table(M.Scenario(i),mode,found,pass,drops,maxLand,rmsLand,windP95,instPeak,causalPeak,rec,postRms,tail5,finalNorm,r05,r10,r30,qp,pred,fracCount,schedResidual,windActive,recoveryActive,recoveryPeak,elevSat,thrSat,elevHead,thrHead,confMean,abruptCount,energyActive,energyShift,inputSnap,disturbanceEvidence,forcedRms,forcedPeak,releaseEvidence,carrierEvidence,carrierSwitch,windSwitch,lastCarrier,lastWindMpc,tailBase,tailZero,zeroSettle, ...
            'VariableNames',{'Scenario','Mode','Found','Pass','Drops','MaxLandingError_m','RmsLandingError_m','WindErrorP95_mps','InstantPeakNorm','PostGustPeakNorm','MaxRecovery_s','PostWindRmsNorm','Tail5RmsNorm','FinalNormInf','Residual0p5Norm','Residual1p0Norm','Residual3p0Norm','QpP95_ms','PredictionP95','FractionalReleaseCount','SchedulerResidualMax_m','WindMpcActiveFraction','RecoveryActiveFraction','RecoveryLevelPeak','ElevatorSatFraction','ThrottleSatFraction','ElevatorMinHeadroom','ThrottleMinHeadroom','WindConfidenceMean','AbruptGustTriggerCount','EnergyRecoveryActiveFraction','EnergyAltitudeShiftPeak_m','InputSnapMax','DisturbanceEvidenceActiveFraction','ForcedResponseRms','ForcedResponsePeak','ReleaseWindEvidenceActiveFraction','CarrierTransientEvidenceActiveFraction','CarrierTransientSwitchCount','WindMpcSwitchCount','LastCarrierTransientTime_s','LastWindMpcActiveTime_s','Tail5BaseSolverFraction','Tail5ZeroTruthWindFraction','ZeroWindSettleDuration_s'});
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
    comparison=[comparison;table(M.Scenario(i),aRms,lRms,carrierRmsRatio,aRec,lRec,carrierRecoveryDelta,aLand,bLand,releaseRatio,localScalar(a,"Residual0p5Norm"),localScalar(a,"Residual1p0Norm"),localScalar(a,"Residual3p0Norm"),localScalar(a,"Tail5RmsNorm"),localScalar(a,"RecoveryActiveFraction"),localScalar(a,"ThrottleSatFraction"),localScalar(a,"ElevatorSatFraction"),localScalar(a,"AbruptGustTriggerCount"),localScalar(a,"EnergyRecoveryActiveFraction"),localScalar(a,"EnergyAltitudeShiftPeak_m"),localScalar(a,"InputSnapMax"),localScalar(a,"DisturbanceEvidenceActiveFraction"),localScalar(a,"ForcedResponseRms"),localScalar(a,"ForcedResponsePeak"),localScalar(a,"CarrierTransientEvidenceActiveFraction"),localScalar(a,"CarrierTransientSwitchCount"),localScalar(a,"WindMpcSwitchCount"),localScalar(a,"LastCarrierTransientTime_s"),localScalar(a,"LastWindMpcActiveTime_s"),localScalar(a,"Tail5BaseSolverFraction"),localScalar(a,"Tail5ZeroTruthWindFraction"),localScalar(a,"ZeroWindSettleDuration_s"), ...
        'VariableNames',{'Scenario','WindMpcPostWindRms','LegacyPostWindRms','LegacyOverWindMpcCarrierRms','WindMpcRecovery_s','LegacyRecovery_s','RecoveryImprovement_s','AwareLandingRms_m','NoWindReleaseLandingRms_m','NoWindOverAwareLandingRms','Residual0p5Norm','Residual1p0Norm','Residual3p0Norm','Tail5RmsNorm','RecoveryActiveFraction','ThrottleSatFraction','ElevatorSatFraction','AbruptGustTriggerCount','EnergyRecoveryActiveFraction','EnergyAltitudeShiftPeak_m','InputSnapMax','DisturbanceEvidenceActiveFraction','ForcedResponseRms','ForcedResponsePeak','CarrierTransientEvidenceActiveFraction','CarrierTransientSwitchCount','WindMpcSwitchCount','LastCarrierTransientTime_s','LastWindMpcActiveTime_s','Tail5BaseSolverFraction','Tail5ZeroTruthWindFraction','ZeroWindSettleDuration_s'})]; %#ok<AGROW>
end
passIntegrated=height(A)==height(M) && all(A.Found) && all(A.Pass);
allExecuted=height(rows)==3*height(M) && all(rows.Found);
finiteRat=comparison.LegacyOverWindMpcCarrierRms(isfinite(comparison.LegacyOverWindMpcCarrierRms)); medianCarrierRatio=median(finiteRat,'omitnan');
R=struct(); R.version="Physics-MPC v1.3.6 base-equivalent transient-evidence energy-aware gust-recovery precision airdrop validation"; R.pass=passIntegrated && allExecuted; R.integrated_pass=passIntegrated; R.all_three_way_executed=allExecuted; R.rows=rows; R.comparison=comparison; R.median_legacy_over_wind_mpc_carrier_rms=medianCarrierRatio; R.completed_at=datetime("now");
abPath=fullfile(root,"calm_wind_mpc_aware_sampled_release","wind_airdrop_mission.mat"); R.calm_release_timing_ablation=struct("found",false,"fractional_rms_m",NaN,"sampled_rms_m",NaN,"sampled_over_fractional_rms",NaN,"fractional_max_m",NaN,"sampled_max_m",NaN);
a=A(A.Scenario=="calm",:); if height(a)==1, R.calm_release_timing_ablation.fractional_rms_m=a.RmsLandingError_m; R.calm_release_timing_ablation.fractional_max_m=a.MaxLandingError_m; end
if isfile(abPath), Z=load(abPath,"report"); R.calm_release_timing_ablation.found=true; R.calm_release_timing_ablation.sampled_rms_m=Z.report.metrics.rms_landing_error_m; R.calm_release_timing_ablation.sampled_max_m=Z.report.metrics.max_landing_error_m; if isfinite(R.calm_release_timing_ablation.fractional_rms_m) && R.calm_release_timing_ablation.fractional_rms_m>0, R.calm_release_timing_ablation.sampled_over_fractional_rms=R.calm_release_timing_ablation.sampled_rms_m/R.calm_release_timing_ablation.fractional_rms_m; end, end
writetable(rows,fullfile(root,"wind_transient_energy_recovery_validation.csv")); writetable(comparison,fullfile(root,"wind_transient_energy_recovery_comparison.csv")); save(fullfile(root,"wind_transient_energy_recovery_validation.mat"),"R","-v7.3");
localWrite(R,fullfile(root,"wind_transient_energy_recovery_validation_summary.txt")); localPlot(comparison,root);
end
function localWrite(R,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v1.3.6 base-equivalent transient-evidence energy-aware gust-recovery precision airdrop validation\nvalidation_pass=%d\nintegrated_wind_mpc_scenarios_pass=%d/8\nall_three_way_missions_found=%d/24\n",R.pass,sum(R.rows.Pass & R.rows.Mode=="wind_mpc_aware"),sum(R.rows.Found));
A=R.rows(R.rows.Mode=="wind_mpc_aware",:); L=R.rows(R.rows.Mode=="legacy_mpc_aware",:); B=R.rows(R.rows.Mode=="legacy_mpc_nowind",:);
fprintf(fid,"wind_mpc_worst_landing_error_m=%.9g\nwind_mpc_worst_instantaneous_peak_norm=%.9g\nwind_mpc_worst_post_gust_peak_norm=%.9g\nwind_mpc_worst_recovery_s=%.9g\nwind_mpc_worst_post_wind_rms_norm=%.9g\nwind_mpc_worst_tail5_rms_norm=%.9g\nwind_mpc_worst_residual_0p5_norm=%.9g\nwind_mpc_worst_residual_1p0_norm=%.9g\nwind_mpc_worst_residual_3p0_norm=%.9g\nwind_mpc_worst_qp_p95_ms=%.9g\n",max(A.MaxLandingError_m,[],'omitnan'),max(A.InstantPeakNorm,[],'omitnan'),max(A.PostGustPeakNorm,[],'omitnan'),max(A.MaxRecovery_s,[],'omitnan'),max(A.PostWindRmsNorm,[],'omitnan'),max(A.Tail5RmsNorm,[],'omitnan'),max(A.Residual0p5Norm,[],'omitnan'),max(A.Residual1p0Norm,[],'omitnan'),max(A.Residual3p0Norm,[],'omitnan'),max(A.QpP95_ms,[],'omitnan'));
fprintf(fid,"wind_mpc_max_throttle_sat_fraction=%.9g\nwind_mpc_max_elevator_sat_fraction=%.9g\nwind_mpc_min_throttle_headroom=%.9g\nwind_mpc_min_elevator_headroom=%.9g\n",max(A.ThrottleSatFraction,[],'omitnan'),max(A.ElevatorSatFraction,[],'omitnan'),min(A.ThrottleMinHeadroom,[],'omitnan'),min(A.ElevatorMinHeadroom,[],'omitnan'));
fprintf(fid,"wind_mpc_max_energy_shift_m=%.9g\nwind_mpc_max_energy_active_fraction=%.9g\nwind_mpc_max_input_snap=%.9g\nwind_mpc_max_abrupt_trigger_count=%.9g\n",max(A.EnergyAltitudeShiftPeak_m,[],'omitnan'),max(A.EnergyRecoveryActiveFraction,[],'omitnan'),max(A.InputSnapMax,[],'omitnan'),max(A.AbruptGustTriggerCount,[],'omitnan'));
fprintf(fid,"legacy_worst_post_gust_peak_norm=%.9g\nlegacy_worst_post_wind_rms_norm=%.9g\nno_wind_release_worst_landing_error_m=%.9g\nmedian_legacy_over_wind_mpc_carrier_rms=%.9g\n",max(L.PostGustPeakNorm,[],'omitnan'),max(L.PostWindRmsNorm,[],'omitnan'),max(B.MaxLandingError_m,[],'omitnan'),R.median_legacy_over_wind_mpc_carrier_rms);
fprintf(fid,"calm_release_ablation_found=%d\ncalm_fractional_rms_m=%.9g\ncalm_sampled_rms_m=%.9g\ncalm_sampled_over_fractional_rms=%.9g\ncalm_fractional_max_m=%.9g\ncalm_sampled_max_m=%.9g\n",R.calm_release_timing_ablation.found,R.calm_release_timing_ablation.fractional_rms_m,R.calm_release_timing_ablation.sampled_rms_m,R.calm_release_timing_ablation.sampled_over_fractional_rms,R.calm_release_timing_ablation.fractional_max_m,R.calm_release_timing_ablation.sampled_max_m);
for i=1:height(R.comparison), q=R.comparison(i,:); fprintf(fid,"%s carrierRms windMPC=%.6g legacy=%.6g ratio=%.6g recovery windMPC=%.6g legacy=%.6g residual[0.5/1/3]=%.6g/%.6g/%.6g tail=%.6g recoveryActive=%.4g transientActive=%.4g transientSwitch=%g windMpcSwitch=%g lastTransient=%.3g lastWindMpc=%.3g tailBase=%.4g tailZeroWind=%.4g throttleSat=%.4g elevatorSat=%.4g abrupt=%g energyActive=%.4g energyShift=%.4g snap=%.3g forcedRms=%.6g forcedPeak=%.6g landingAware=%.6g noWind=%.6g\n",char(q.Scenario),q.WindMpcPostWindRms,q.LegacyPostWindRms,q.LegacyOverWindMpcCarrierRms,q.WindMpcRecovery_s,q.LegacyRecovery_s,q.Residual0p5Norm,q.Residual1p0Norm,q.Residual3p0Norm,q.Tail5RmsNorm,q.RecoveryActiveFraction,q.CarrierTransientEvidenceActiveFraction,q.CarrierTransientSwitchCount,q.WindMpcSwitchCount,q.LastCarrierTransientTime_s,q.LastWindMpcActiveTime_s,q.Tail5BaseSolverFraction,q.Tail5ZeroTruthWindFraction,q.ThrottleSatFraction,q.ElevatorSatFraction,q.AbruptGustTriggerCount,q.EnergyRecoveryActiveFraction,q.EnergyAltitudeShiftPeak_m,q.InputSnapMax,q.ForcedResponseRms,q.ForcedResponsePeak,q.AwareLandingRms_m,q.NoWindReleaseLandingRms_m); end
end
function localPlot(C,root)
try
    f=figure("Visible","off","Position",[80 80 1600 900]); tiledlayout(f,2,2,"TileSpacing","compact","Padding","compact");
    nexttile; bar(categorical(C.Scenario),[C.WindMpcPostWindRms C.LegacyPostWindRms]); ylabel("post-wind primary RMS"); legend("v1.3.6 transient-evidence MPC","legacy MPC"); grid on;
    nexttile; bar(categorical(C.Scenario),[C.WindMpcRecovery_s C.LegacyRecovery_s]); ylabel("gust recovery (s)"); legend("v1.3.6","legacy"); grid on;
    nexttile; bar(categorical(C.Scenario),[C.Residual0p5Norm C.Residual1p0Norm C.Residual3p0Norm]); ylabel("gust residual normalized"); legend("0.5 s","1 s","3 s"); grid on;
    nexttile; bar(categorical(C.Scenario),[C.AwareLandingRms_m C.NoWindReleaseLandingRms_m]); ylabel("landing RMS miss (m)"); legend("wind-aware release","no-wind release"); grid on;
    exportgraphics(f,fullfile(root,"wind_transient_energy_recovery_validation.png"),"Resolution",160); close(f);
catch ME
    warning("AirdropX:WindMPC:FinalizePlotFailed","Plot failed: %s",ME.message);
end
end
function v=localScalar(T,name)
if height(T)==1, v=double(T.(name)(1)); else, v=NaN; end
end
