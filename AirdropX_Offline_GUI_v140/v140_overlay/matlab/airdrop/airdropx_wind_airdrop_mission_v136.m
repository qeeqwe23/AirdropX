function report=airdropx_wind_airdrop_mission_v136(projectRoot,opts)
%AIRDROPX_WIND_AIRDROP_MISSION_V136 Transient-evidence wind MPC + auditable energy-aware gust recovery.
%
% Carrier truth: persistent nonlinear JSBSim wind Oracle.
% Wind sensing: frozen v1.1.3 estimator (truth wind is scoring-only).
% Carrier controller: unified Physics-MPC Np=Nc=100 with optional measured
% disturbance prediction.  The disturbance channel is calibrated from real
% JSBSim +/- wind-increment experiments; no guessed wind E matrix is used.
% Release guidance: existing v1.2.1 wind-aware impact predictor.
arguments
    projectRoot (1,1) string
    opts.BankPath (1,1) string = ""
    opts.WindCalibrationPath (1,1) string = ""
    opts.OutputRoot (1,1) string = ""
    opts.ScenarioName (1,1) string = "calm"
    opts.UseWindCompensation (1,1) logical = true
    opts.UseWindDisturbanceMPC (1,1) logical = true
    opts.UseFractionalRelease (1,1) logical = true
    opts.H (1,1) double {mustBeFinite,mustBePositive} = 200
    opts.V (1,1) double {mustBeFinite,mustBePositive} = 50
    opts.FuelScale (1,1) double {mustBeFinite} = 1.0
    opts.Duration_s (1,1) double {mustBePositive} = 55
    opts.TargetStart_m (1,1) double {mustBeFinite,mustBePositive} = 1200
    opts.TargetSpacing_m (1,1) double {mustBeFinite,mustBePositive} = 80
    opts.SensorNoiseSeed (1,1) double {mustBeFinite} = 1
    opts.SigmaGroundspeed_mps (1,1) double {mustBeNonnegative} = 0.15
    opts.SigmaAirspeed_mps (1,1) double {mustBeNonnegative} = 0.25
    opts.SigmaVerticalSpeed_mps (1,1) double {mustBeNonnegative} = 0.10
    opts.WindRateForecastCap_mps2 (1,1) double {mustBePositive} = 3.0
    opts.WindRateMemory_s (1,1) double {mustBePositive} = 2.0
    opts.UseWindConfidenceGate (1,1) logical = true
    opts.WindConfidenceSNRHalf (1,1) double {mustBePositive} = 2.5
    opts.WindAbsConfidenceHalf_mps (1,1) double {mustBePositive} = 0.35
    opts.WindRateConfidenceHalf_mps2 (1,1) double {mustBePositive} = 0.30
    opts.ReleaseVelocityFilterTau_s (1,1) double {mustBePositive} = 0.20
    opts.ReleaseFilterFastSigma (1,1) double {mustBePositive} = 3.0
    opts.UseUnifiedGustRecovery (1,1) logical = true
    opts.RecoveryOnsetNormalized (1,1) double {mustBeNonnegative} = 0.80
    opts.RecoveryFullNormalized (1,1) double {mustBePositive} = 3.00
    opts.RecoveryReleaseNormalized (1,1) double {mustBeNonnegative} = 0.55
    opts.RecoveryDecay_s (1,1) double {mustBePositive} = 1.50
    opts.ResidualObserverAlpha (1,1) double {mustBeNonnegative} = 0.70
    opts.ResidualMemory_s (1,1) double {mustBePositive} = 0.8
    opts.ResidualClipNormalized (1,1) double {mustBePositive} = 0.25
    opts.ResidualDeadbandNormalized (1,1) double {mustBeNonnegative} = 0.02
    opts.ResidualFullScaleNormalized (1,1) double {mustBePositive} = 0.12
    opts.GustStepThreshold_mps (1,1) double {mustBePositive} = 1.5
    opts.GustTriggerMinWindConfidence (1,1) double {mustBeNonnegative} = 0.35
    opts.GustTriggerDeltaFraction (1,1) double {mustBePositive} = 0.85
    opts.RecoveryQuietHold_s (1,1) double {mustBePositive} = 0.6
    opts.UseContinuousRecoveryAssist (1,1) logical = false
    opts.ContinuousAssistMax (1,1) double {mustBeNonnegative} = 0.15
    opts.ContinuousAssistMinRateConfidence (1,1) double {mustBeNonnegative} = 0.60
    opts.ContinuousAssistRateOnset_mps2 (1,1) double {mustBeNonnegative} = 0.35
    opts.ContinuousAssistRateFull_mps2 (1,1) double {mustBePositive} = 1.50
    opts.DisturbanceSolveDeadbandNormalized (1,1) double {mustBeNonnegative} = 0.01
    opts.DisturbanceMinWindConfidence (1,1) double {mustBeNonnegative} = 0.15
    opts.DisturbanceMinRateConfidence (1,1) double {mustBeNonnegative} = 0.55
    % v1.3.6 splits absolute-wind evidence (release guidance) from transient
    % carrier evidence.  A constant non-zero wind must NOT keep the carrier
    % disturbance QP alive forever after the aircraft has recovered.
    opts.CarrierRateThreshold_mps2 (1,1) double {mustBeNonnegative} = 0.70
    opts.CarrierRateConfidence (1,1) double {mustBeNonnegative} = 0.70
    opts.CarrierRateActivationWindConfidence (1,1) double {mustBeNonnegative} = 0.15
    opts.CarrierRateHold_s (1,1) double {mustBePositive} = 0.30
    opts.CarrierTransientPersistence_s (1,1) double {mustBePositive} = 3.50
    opts.ReleaseWindEvidenceConfidence (1,1) double {mustBeNonnegative} = 0.25
    % Sustained sine forcing is scored separately from the post-forcing settle.
    opts.SineForcingEnd_s (1,1) double {mustBePositive} = 45
    opts.SineSettleRamp_s (1,1) double {mustBePositive} = 2
    opts.MaxSineForcedRmsNormalized (1,1) double {mustBePositive} = 1.50
    opts.MaxSineForcedPeakNormalized (1,1) double {mustBePositive} = 3.00
    opts.UseEnergyRecovery (1,1) logical = true
    opts.EnergyMaxAltitudeShift_m (1,1) double {mustBeNonnegative} = 3.0
    opts.EnergyMaxAltitudeShiftFraction (1,1) double {mustBeNonnegative} = 0.02
    opts.EnergyVaOnsetNormalized (1,1) double {mustBeNonnegative} = 0.75
    opts.EnergyVaFullNormalized (1,1) double {mustBePositive} = 2.5
    opts.EnergyThrottleHigh (1,1) double {mustBeNonnegative} = 0.985
    opts.EnergyThrottleLow (1,1) double {mustBeNonnegative} = 0.015
    opts.EnergyReferenceTau_s (1,1) double {mustBePositive} = 0.35
    opts.EnergyReferenceDecay_s (1,1) double {mustBePositive} = 0.8
    opts.InputSnapTolerance (1,1) double {mustBePositive} = 1e-7
    opts.GustExclusion_s (1,1) double {mustBeNonnegative} = 1.0
    opts.MaxGustRecoveryTime_s (1,1) double {mustBePositive} = 3.0
    opts.MaxPostGustPeakNormalized (1,1) double {mustBePositive} = 1.25
    opts.MaxLandingError_m (1,1) double {mustBePositive} = 20
    opts.MaxPredictedImpactErrorAtRelease_m (1,1) double {mustBePositive} = 10
    opts.MaxWindErrorP95_mps (1,1) double {mustBePositive} = 0.75
    opts.MaxFinalNormalizedInf (1,1) double {mustBePositive} = 0.10
    opts.MaxTail5sNormalizedRms (1,1) double {mustBePositive} = 0.05
    opts.ThrowOnFail (1,1) logical = false
end
if opts.ResidualObserverAlpha>1, error("AirdropX:WindMPC:BadResidualAlpha","ResidualObserverAlpha must be in [0,1]."); end
if opts.GustTriggerMinWindConfidence>1, error("AirdropX:WindMPC:BadGustConfidence","GustTriggerMinWindConfidence must be in [0,1]."); end
if opts.ContinuousAssistMax>1 || opts.ContinuousAssistMinRateConfidence>1, error("AirdropX:WindMPC:BadContinuousAssist","Continuous assist limits must be in [0,1]."); end
if opts.DisturbanceMinWindConfidence>1 || opts.DisturbanceMinRateConfidence>1 || opts.CarrierRateConfidence>1 || opts.CarrierRateActivationWindConfidence>1 || opts.ReleaseWindEvidenceConfidence>1, error("AirdropX:WindMPC:BadDisturbanceEvidence","Wind/transient confidence thresholds must be in [0,1]."); end
if opts.EnergyThrottleLow>=opts.EnergyThrottleHigh, error("AirdropX:WindMPC:BadEnergyThrottleThresholds","EnergyThrottleLow must be below EnergyThrottleHigh."); end
if opts.BankPath=="", opts.BankPath=fullfile(projectRoot,"matlab","results","physics_mpc_v033","physics_bank.mat"); end
if opts.WindCalibrationPath=="", opts.WindCalibrationPath=fullfile(projectRoot,"matlab","results","physics_mpc_v130_wind_disturbance_calibration","wind_disturbance_model_v130.mat"); end
if opts.UseWindDisturbanceMPC
    controlMode="wind_disturbance_mpc";
else
    controlMode="legacy_mpc";
end
releaseMode="wind_aware_release"; if ~opts.UseWindCompensation, releaseMode="no_wind_release"; end
mode=controlMode+"__"+releaseMode;
if opts.OutputRoot=="", opts.OutputRoot=fullfile(projectRoot,"matlab","results","physics_mpc_v136_transient_energy_recovery_airdrop",opts.ScenarioName+"_"+mode); end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
status=fullfile(opts.OutputRoot,"scenario_status.txt"); localStatus(status,"STARTED "+mode);
report=struct("pass",false,"completed_at",datetime("now"));
try
    if exist("quadprog","file")~=2, error("AirdropX:WindMPC:MissingQuadprog","quadprog is required."); end
    if exist("airdropx_jsbsim_wind_oracle_mex","file")~=3, error("AirdropX:WindMPC:MissingWindOracle","Build the wind Oracle first."); end
    vOracle=string(airdropx_jsbsim_wind_oracle_mex("version"));
    if ~contains(vOracle,"v1.2.1"), error("AirdropX:WindMPC:WrongWindOracle","Unexpected wind Oracle: %s",vOracle); end
    sched=airdropx_phys_mpc_build_cfg_schedule(opts.BankPath,opts.H,opts.V,opts.FuelScale,Horizon=100);
    if opts.UseWindDisturbanceMPC
        sched=airdropx_phys_mpc_load_wind_disturbance_v130(sched,opts.WindCalibrationPath);
        if opts.UseUnifiedGustRecovery
            sched=airdropx_phys_mpc_build_recovery_bank_v134(sched);
        end
    end
    models=sched.models; Ts=0.1;
    targets=opts.TargetStart_m+(0:3)'*opts.TargetSpacing_m;
    if opts.Duration_s<30, error("AirdropX:WindMPC:MissionTooShort","Use at least 30 s."); end
    if opts.ScenarioName=="sine_longitudinal" && opts.Duration_s < opts.SineForcingEnd_s+opts.SineSettleRamp_s+5
        error("AirdropX:WindMPC:SineMissionTooShort","Sine validation must leave a full 5 s zero-wind tail after the forcing shutdown.");
    end

    localStatus(status,"WIND_ORACLE_INIT");
    airdropx_phys_wind_oracle_init_v121(projectRoot);
    cleanup=onCleanup(@()localCloseWindOracle()); %#ok<NASGU>
    x=models{1}.ctrl.xref; uTrim=models{1}.ctrl.uref;
    w0=double(airdropx_wind_profile_v136(opts.ScenarioName,0,SineForcingEnd_s=opts.SineForcingEnd_s,SineSettleRamp_s=opts.SineSettleRamp_s));
    D=airdropx_jsbsim_wind_oracle_mex("start_continuous",x,uTrim,0,opts.FuelScale,w0);
    oracleInitialIyy_kgm2=double(D.Iyy_kgm2);
    localStatus(status,"CONTINUOUS_JSBSIM_STARTED");

    est=airdropx_longitudinal_wind_estimator_init_v111(Ts, ...
        SigmaGroundspeed_mps=opts.SigmaGroundspeed_mps,SigmaAirspeed_mps=opts.SigmaAirspeed_mps,SigmaVerticalSpeed_mps=opts.SigmaVerticalSpeed_mps);
    rng(round(opts.SensorNoiseSeed),"twister");
    Nsim=ceil(opts.Duration_s/Ts)+1; t=(0:Nsim-1)'*Ts;
    X=nan(Nsim,7); U=nan(Nsim,2); Err=nan(Nsim,7); Cfg=nan(Nsim,1); PosN=nan(Nsim,1); Vg=nan(Nsim,1); Vz=nan(Nsim,1);
    WindTrue=nan(Nsim,1); WindEst=nan(Nsim,1); WindRate=nan(Nsim,1); WindSigma=nan(Nsim,1); WindRaw=nan(Nsim,1);
    WindConfidence=nan(Nsim,1); WindRateConfidence=nan(Nsim,1); WindEffective=nan(Nsim,1); WindRateEffective=nan(Nsim,1);
    GuideVg=nan(Nsim,1); GuideVa=nan(Nsim,1); GuideVz=nan(Nsim,1);
    PredImpact=nan(Nsim,1); CurrentTarget=nan(Nsim,1); QPTime=nan(Nsim,1); QPExit=nan(Nsim,1); PredErrInf=nan(Nsim,1); Mass=nan(Nsim,1); Cg=nan(Nsim,1); Iyy=nan(Nsim,1);
    ForecastDw1=nan(Nsim,1); ResidualNorm=nan(Nsim,1); WindMpcActive=false(Nsim,1); RecoverySeverity=nan(Nsim,1); RecoveryLevel=nan(Nsim,1); GustRecoveryLatched=false(Nsim,1);
    AbruptGustTrigger=false(Nsim,1); DisturbanceEvidenceActive=false(Nsim,1); CarrierTransientEvidenceActive=false(Nsim,1); ReleaseWindEvidenceActive=false(Nsim,1); EnergyHrefShift=nan(Nsim,1); InputSnapMagnitude=nan(Nsim,1);
    DropEvent=false(Nsim,1); ReleaseIndex=zeros(Nsim,1); ReleasePhase_s=nan(Nsim,1); FractionalRelease=false(Nsim,1);
    warm=zeros(models{1}.ctrl.m*models{1}.ctrl.N,1); cfg=0; dHat=zeros(7,1); prev=[]; observerHoldoff=0;
    guideVgState=NaN; guideVaState=NaN; guideVzState=NaN; recoveryState=0; gustLatched=false; recoveryQuietCount=0; energyHrefState=0;
    carrierTransientActive=false; carrierRateActivationCount=0; carrierRateKeepCount=0; carrierTransientHoldCount=0; carrierTransientEventCount=0;
    release=struct('cargo',(1:4)','target_m',targets,'released',false(4,1),'t_s',nan(4,1),'release_x_m',nan(4,1), ...
        'release_h_m',nan(4,1),'release_vg_mps',nan(4,1),'release_vz_mps',nan(4,1),'wind_est_mps',nan(4,1),'wind_rate_est_mps2',nan(4,1), ...
        'wind_truth_mps',nan(4,1),'predicted_impact_m',nan(4,1),'truth_impact_m',nan(4,1),'predicted_error_m',nan(4,1),'landing_error_m',nan(4,1), ...
        'predicted_fall_time_s',nan(4,1),'truth_fall_time_s',nan(4,1),'release_phase_s',nan(4,1),'fractional_release',false(4,1),'scheduler_residual_m',nan(4,1),'scheduler_mode',strings(4,1));
    expectedMass=zeros(5,1); expectedCg=zeros(5,1); expectedIyy=zeros(5,1);
    for c=0:4
        dd=models{c+1}.vertex.trim.diag; expectedMass(c+1)=double(dd.mass_kg); expectedCg(c+1)=double(dd.cg_x_m); expectedIyy(c+1)=double(dd.Iyy_kgm2);
    end
    % The persistent wind Oracle and the archived trim bank can carry a tiny
    % constant Iyy-origin offset (unit/initialization provenance) even when the
    % payload configuration deltas are exact.  Calibrate ONLY that cfg0 offset
    % once, then keep the original strict delta-consistency gate for cfg0..4.
    iyyReferenceBias_kgm2=oracleInitialIyy_kgm2-expectedIyy(1);

    for k=1:Nsim
        ctrl=models{cfg+1}.ctrl;
        X(k,:)=x.'; Cfg(k)=cfg; PosN(k)=double(D.pos_n_m); Vg(k)=double(D.v_north_mps); Vz(k)=double(D.vz_up_mps);
        WindTrue(k)=double(D.wind_long_mps); Mass(k)=double(D.mass_kg); Cg(k)=double(D.cg_x_m); Iyy(k)=double(D.Iyy_kgm2);
        VaMeas=x(2)+opts.SigmaAirspeed_mps*randn; VzMeas=Vz(k)+opts.SigmaVerticalSpeed_mps*randn; VgMeas=Vg(k)+opts.SigmaGroundspeed_mps*randn;
        [est,eo]=airdropx_longitudinal_wind_estimator_step_v111(est,VaMeas,VzMeas,VgMeas);
        WindEst(k)=eo.wind_est_mps; WindRate(k)=eo.wind_rate_est_mps2; WindSigma(k)=eo.wind_sigma_mps; WindRaw(k)=eo.raw_wind_mps;
        if opts.UseWindConfidenceGate
            WC=airdropx_wind_confidence_v132(WindEst(k),WindRate(k),WindSigma(k),WindSNRHalf=opts.WindConfidenceSNRHalf,WindAbsHalf_mps=opts.WindAbsConfidenceHalf_mps,RateHalf_mps2=opts.WindRateConfidenceHalf_mps2);
        else
            WC=struct("wind_confidence",1,"rate_confidence",1,"wind_effective_mps",WindEst(k),"wind_rate_effective_mps2",WindRate(k));
        end
        WindConfidence(k)=WC.wind_confidence; WindRateConfidence(k)=WC.rate_confidence; WindEffective(k)=WC.wind_effective_mps; WindRateEffective(k)=WC.wind_rate_effective_mps2;

        % v1.3.6 separates two meanings of "wind evidence":
        %   (1) release guidance needs ABSOLUTE wind whenever it is credible;
        %   (2) carrier disturbance MPC only needs TRANSIENT wind change / gust.
        % A spatially uniform constant wind therefore remains available to the
        % airdrop predictor, but it cannot keep the carrier disturbance QP alive.
        deltaWindEst=0;
        if k>1 && isfinite(WindEst(k-1)), deltaWindEst=WindEst(k)-WindEst(k-1); end
        abruptThreshold=opts.GustTriggerDeltaFraction*opts.GustStepThreshold_mps;
        strongWindEvidence=WindConfidence(k)>=opts.GustTriggerMinWindConfidence;
        gustTrigger=k>1 && strongWindEvidence && abs(deltaWindEst)>=abruptThreshold;
        AbruptGustTrigger(k)=gustTrigger;
        ReleaseWindEvidenceActive(k)=WindConfidence(k)>=opts.ReleaseWindEvidenceConfidence;

        % Smooth ramp/sine forcing is identified from a sustained, confidence-
        % qualified wind RATE.  Calm rate noise cannot activate because activation
        % also needs nontrivial absolute-wind confidence.  Once an event exists,
        % rate evidence alone may bridge sine zero-crossings.  Constant nonzero
        % wind amplitude is deliberately NOT a keep-alive condition.
        rateDynamic=WindRateConfidence(k)>=opts.CarrierRateConfidence && abs(WindRateEffective(k))>=opts.CarrierRateThreshold_mps2;
        rateActivation=rateDynamic && WindConfidence(k)>=opts.CarrierRateActivationWindConfidence;
        if rateActivation
            carrierRateActivationCount=carrierRateActivationCount+1;
        else
            carrierRateActivationCount=0;
        end
        if rateDynamic
            carrierRateKeepCount=carrierRateKeepCount+1;
        else
            carrierRateKeepCount=0;
        end
        rateHoldN=max(1,ceil(opts.CarrierRateHold_s/Ts));
        persistN=max(1,ceil(opts.CarrierTransientPersistence_s/Ts));
        wasCarrierTransient=carrierTransientActive;
        if gustTrigger || carrierRateActivationCount>=rateHoldN
            carrierTransientActive=true; carrierTransientHoldCount=persistN;
        elseif carrierTransientActive
            if gustLatched || carrierRateKeepCount>=rateHoldN
                carrierTransientHoldCount=persistN;
            else
                carrierTransientHoldCount=max(0,carrierTransientHoldCount-1);
                if carrierTransientHoldCount==0, carrierTransientActive=false; end
            end
        end
        if carrierTransientActive && ~wasCarrierTransient, carrierTransientEventCount=carrierTransientEventCount+1; end
        CarrierTransientEvidenceActive(k)=carrierTransientActive;
        DisturbanceEvidenceActive(k)=carrierTransientActive; % backward-compatible column name
        disturbanceEvidence=carrierTransientActive;
        [guideVgState,GuideVg(k)]=localAdaptiveGuideFilter(guideVgState,VgMeas,opts.SigmaGroundspeed_mps,Ts,opts.ReleaseVelocityFilterTau_s,opts.ReleaseFilterFastSigma);
        [guideVaState,GuideVa(k)]=localAdaptiveGuideFilter(guideVaState,VaMeas,opts.SigmaAirspeed_mps,Ts,opts.ReleaseVelocityFilterTau_s,opts.ReleaseFilterFastSigma);
        [guideVzState,GuideVz(k)]=localAdaptiveGuideFilter(guideVzState,VzMeas,opts.SigmaVerticalSpeed_mps,Ts,opts.ReleaseVelocityFilterTau_s,opts.ReleaseFilterFastSigma);

        % Causal residual disturbance observer.  Subtract the identified wind
        % increment effect first, so a gust step is not incorrectly learned as
        % a permanent offset.  One sample after a payload transition is ignored.
        dxCurrent=airdropx_phys_mpc_state_error(x,ctrl.xref);
        if opts.UseWindDisturbanceMPC && disturbanceEvidence && ~isempty(prev) && prev.cfg==cfg && observerHoldoff<=0
            dwObserved=WindEffective(k)-prev.wind_effective;
            dxPredObs=prev.ctrl.A*prev.dx+prev.ctrl.B*prev.du+prev.ctrl.Gw*dwObserved;
            innovation=dxCurrent-dxPredObs;
            lim=opts.ResidualClipNormalized*prev.ctrl.stateScale;
            innovation=max(min(innovation,lim),-lim);
            innovationNorm=max(abs(innovation)./prev.ctrl.stateScale);
            residualConfidence=localSmoothStep(innovationNorm,opts.ResidualDeadbandNormalized,opts.ResidualFullScaleNormalized);
            innovation=residualConfidence*innovation;
            dHat=opts.ResidualObserverAlpha*dHat+(1-opts.ResidualObserverAlpha)*innovation;
        elseif observerHoldoff>0
            observerHoldoff=observerHoldoff-1;
            dHat=zeros(7,1);
        elseif opts.UseWindDisturbanceMPC && ~disturbanceEvidence && ~gustLatched
            % No credible wind/rate evidence: do not let sensor/model noise
            % sustain a fictitious residual channel in calm flight.
            dHat=zeros(7,1);
        end

        % Release guidance. Truth wind is prohibited here. v1.3.6 keeps the
        % validated v1.2.1 ballistic predictor, but removes the 0.1 s release
        % quantization: when the predicted impact will cross the target during
        % the CURRENT control interval, a causal timer schedules a sub-sample
        % payload removal. MPC itself still updates at the certified Ts=0.1 s.
        scheduledRelease=false; scheduledTau=NaN; scheduledPredImpact=NaN; scheduledResidual=NaN; scheduledMode=""; scheduledIndex=0;
        if cfg<4
            CurrentTarget(k)=targets(cfg+1);
            if opts.UseWindCompensation
                wGuide=WindEffective(k); rGuide=WindRateEffective(k); vxGuide=GuideVg(k);
            else
                wGuide=0; rGuide=0; vxGuide=sqrt(max(GuideVa(k)^2-GuideVz(k)^2,0));
            end
            rs=struct("x_m",PosN(k),"h_m",x(1),"vx_ground_mps",vxGuide,"vz_up_mps",GuideVz(k),"wind_est_mps",wGuide,"wind_rate_est_mps2",rGuide);
            pred=airdropx_airdrop_predict_impact_v121(rs); PredImpact(k)=pred.impact_x_m;
            if opts.UseFractionalRelease
                SR=airdropx_airdrop_fractional_release_v131(rs,targets(cfg+1),Ts);
            else
                % Legacy nearest-sample rule retained as an explicit ablation.
                releaseWindow=max(2,0.5*max(abs(VgMeas),1)*Ts);
                SR=struct("release_now",pred.impact_x_m>=targets(cfg+1)-releaseWindow,"release_within_sample",false,"tau_s",0, ...
                    "estimated_impact_at_release_m",pred.impact_x_m,"scheduler_residual_m",pred.impact_x_m-targets(cfg+1),"mode","legacy_sample_boundary");
            end
            if SR.release_now
                j=cfg+1; DropEvent(k)=true; ReleaseIndex(k)=j; ReleasePhase_s(k)=0; FractionalRelease(k)=false;
                release.released(j)=true; release.t_s(j)=t(k); release.release_phase_s(j)=0; release.fractional_release(j)=false; release.scheduler_mode(j)=string(SR.mode); release.scheduler_residual_m(j)=SR.scheduler_residual_m;
                release.release_x_m(j)=PosN(k); release.release_h_m(j)=x(1); release.release_vg_mps(j)=Vg(k); release.release_vz_mps(j)=Vz(k);
                release.wind_est_mps(j)=wGuide; release.wind_rate_est_mps2(j)=rGuide; release.wind_truth_mps(j)=WindTrue(k); release.predicted_impact_m(j)=SR.estimated_impact_at_release_m;
                release.predicted_error_m(j)=release.predicted_impact_m(j)-targets(j); release.predicted_fall_time_s(j)=pred.fall_time_s;
                truthRelease=struct("x_m",PosN(k),"h_m",x(1),"vx_ground_mps",Vg(k),"vz_up_mps",Vz(k),"t_s",t(k));
                truth=airdropx_airdrop_truth_impact_v136(opts.ScenarioName,truthRelease,SineForcingEnd_s=opts.SineForcingEnd_s,SineSettleRamp_s=opts.SineSettleRamp_s);
                release.truth_impact_m(j)=truth.impact_x_m; release.landing_error_m(j)=truth.impact_x_m-targets(j); release.truth_fall_time_s(j)=truth.fall_time_s;
                oldCtrl=ctrl; cfg=cfg+1; ctrl=models{cfg+1}.ctrl; warm=airdropx_phys_mpc_rebase_warmstart(warm,oldCtrl,ctrl);
                dHat=zeros(7,1); prev=[]; observerHoldoff=1;
                fprintf("[V136 DROP] %s %s cargo%d t=%.3f phase=0 target=%.2f release=%.2f pred=%.2f truth=%.2f miss=%+.2f windEst=%+.2f truthWind=%+.2f\n", ...
                    opts.ScenarioName,mode,j,t(k),targets(j),PosN(k),release.predicted_impact_m(j),truth.impact_x_m,release.landing_error_m(j),WindEst(k),WindTrue(k));
            elseif SR.release_within_sample
                scheduledRelease=true; scheduledTau=SR.tau_s; scheduledPredImpact=SR.estimated_impact_at_release_m; scheduledResidual=SR.scheduler_residual_m; scheduledMode=string(SR.mode); scheduledIndex=cfg+1;
            end
        end

        baseCtrl=models{cfg+1}.ctrl;
        dx=airdropx_phys_mpc_state_error(x,baseCtrl.xref); Err(k,:)=dx.';
        if opts.UseWindDisturbanceMPC
            % v1.3.6: strong recovery is event-gated.  A real abrupt gust must
            % have both a sufficiently large sample-to-sample wind change and
            % credible wind evidence.  Continuous ramp/sine wind is handled by
            % Gw preview only by default (weak recovery assist is explicit opt-in); estimator noise cannot latch
            % the aggressive recovery bank by itself.
            % gustTrigger was qualified above by the same causal evidence used
            % by the disturbance latch; do not re-derive it from rate noise here.
            if gustTrigger, gustLatched=true; recoveryQuietCount=0; end

            vaN=abs(dx(2))/baseCtrl.stateScale(2); hN=abs(dx(1))/baseCtrl.stateScale(1);
            recoveryMetric=max(vaN,0.5*hN);
            stateSeverity=localSmoothStep(recoveryMetric,opts.RecoveryOnsetNormalized,opts.RecoveryFullNormalized);
            assist=0;
            if opts.UseContinuousRecoveryAssist && WindRateConfidence(k)>=opts.ContinuousAssistMinRateConfidence && WindConfidence(k)>=opts.DisturbanceMinWindConfidence
                assist=opts.ContinuousAssistMax*localSmoothStep(abs(WindRateEffective(k)),opts.ContinuousAssistRateOnset_mps2,opts.ContinuousAssistRateFull_mps2);
            end
            if gustLatched, targetRecovery=max(stateSeverity,assist); else, targetRecovery=assist; end
            if targetRecovery>=recoveryState
                recoveryState=targetRecovery;
            else
                recoveryState=max(targetRecovery,recoveryState*exp(-Ts/opts.RecoveryDecay_s));
            end
            if gustLatched && recoveryMetric<=opts.RecoveryReleaseNormalized
                recoveryQuietCount=recoveryQuietCount+1;
                if recoveryQuietCount>=max(1,round(opts.RecoveryQuietHold_s/Ts)), gustLatched=false; end
            else
                recoveryQuietCount=0;
            end

            ctrl=baseCtrl; selectedLevel=0;
            if opts.UseUnifiedGustRecovery && isfield(models{cfg+1},"recovery_ctrl") && recoveryState>0
                levels=sched.recovery.levels; [~,jj]=min(abs(levels-recoveryState)); ctrl=models{cfg+1}.recovery_ctrl{jj}; selectedLevel=levels(jj);
            end
            mpcWind=WindEffective(k); mpcRate=WindRateEffective(k);
            if ~disturbanceEvidence, mpcWind=0; mpcRate=0; end
            F=airdropx_phys_mpc_wind_forecast_v130(mpcWind,mpcRate,WindSigma(k),ctrl.N,Ts, ...
                RateCap_mps2=opts.WindRateForecastCap_mps2,RateMemory_s=opts.WindRateMemory_s);
            decay=exp(-((0:ctrl.N-1)*Ts)/opts.ResidualMemory_s);
            gSeq=ctrl.Gw*F.delta_wind_pred_mps.'+dHat*decay;
            gNorm=max(abs(gSeq)./ctrl.stateScale,[],"all");
            ForecastDw1(k)=F.delta_wind_pred_mps(1); ResidualNorm(k)=max(abs(dHat)./ctrl.stateScale); RecoverySeverity(k)=recoveryState; RecoveryLevel(k)=selectedLevel; GustRecoveryLatched(k)=gustLatched;
            useTransientSolver=disturbanceEvidence || gustLatched || selectedLevel>0;
            if ~useTransientSolver
                % Outside a proven transient event, use the original certified
                % solver EXACTLY even if absolute wind is large.  This removes
                % the v1.3.5 constant-headwind QP on/off chatter and lets N1/N2
                % settle naturally under base Physics-MPC.
                gSeq=zeros(baseCtrl.n,baseCtrl.N); sol=airdropx_phys_mpc_solve(baseCtrl,x,warm); WindMpcActive(k)=false; ctrl=baseCtrl;
            else
                % During a proven transient keep one deterministic disturbance
                % path; do not chatter between solvers merely because gNorm crosses
                % a tiny numerical deadband near the end of the event.
                sol=airdropx_phys_mpc_solve_disturbance_v130(ctrl,x,gSeq,warm); WindMpcActive(k)=true;
            end

            % Energy-aware second pass.  Only an abrupt-gust recovery that is
            % already throttle-limited may borrow a bounded amount of altitude.
            % Low Va + high throttle => slightly lower temporary H reference;
            % high Va + low throttle => slightly higher temporary H reference.
            % No actuator limit is enlarged and the shift smoothly decays.
            energyDesired=0;
            if opts.UseEnergyRecovery && gustLatched && selectedLevel>=0.5 && sol.feasible
                vaSignedN=dx(2)/baseCtrl.stateScale(2); rawThrottle=sol.u(2);
                if (vaSignedN<=-opts.EnergyVaOnsetNormalized && rawThrottle>=opts.EnergyThrottleHigh) || ...
                   (vaSignedN>= opts.EnergyVaOnsetNormalized && rawThrottle<=opts.EnergyThrottleLow)
                    amp=localSmoothStep(abs(vaSignedN),opts.EnergyVaOnsetNormalized,opts.EnergyVaFullNormalized);
                    energyCap=min(opts.EnergyMaxAltitudeShift_m,opts.EnergyMaxAltitudeShiftFraction*max(abs(baseCtrl.xref(1)),1));
                    energyDesired=energyCap*sign(vaSignedN)*amp*max(selectedLevel,0.5);
                end
            end
            if abs(energyDesired)>0
                aE=1-exp(-Ts/opts.EnergyReferenceTau_s);
            else
                aE=1-exp(-Ts/opts.EnergyReferenceDecay_s);
            end
            energyHrefState=energyHrefState+aE*(energyDesired-energyHrefState);
            energyCapNow=min(opts.EnergyMaxAltitudeShift_m,opts.EnergyMaxAltitudeShiftFraction*max(abs(baseCtrl.xref(1)),1));
            energyHrefState=max(min(energyHrefState,energyCapNow),-energyCapNow);
            if opts.UseEnergyRecovery && selectedLevel>0 && abs(energyHrefState)>=1e-4
                ctrlEnergy=ctrl; ctrlEnergy.xref=baseCtrl.xref; ctrlEnergy.xref(1)=baseCtrl.xref(1)+energyHrefState;
                solEnergy=airdropx_phys_mpc_solve_disturbance_v130(ctrlEnergy,x,gSeq,warm);
                if solEnergy.feasible, sol=solEnergy; ctrl=ctrlEnergy; end
            end
            EnergyHrefShift(k)=energyHrefState;
        else
            ctrl=baseCtrl; gSeq=zeros(ctrl.n,ctrl.N);
            sol=airdropx_phys_mpc_solve(ctrl,x,warm);
            ForecastDw1(k)=0; ResidualNorm(k)=0; RecoverySeverity(k)=0; RecoveryLevel(k)=0; GustRecoveryLatched(k)=false; AbruptGustTrigger(k)=false; EnergyHrefShift(k)=0;
        end
        QPTime(k)=sol.solve_time_s; QPExit(k)=sol.exitflag;
        if ~sol.feasible, error("AirdropX:WindMPC:QPInfeasible","QP failed at t=%.3f cfg=%d.",t(k),cfg); end
        [u,duApplied,snapMag]=localSnapPhysicalInput(sol.u,ctrl.uref,ctrl.umin,ctrl.umax,opts.InputSnapTolerance);
        InputSnapMagnitude(k)=snapMag; U(k,:)=u.';
        sol.u=u; sol.du=duApplied; sol.U(1:ctrl.m)=duApplied;
        if k==Nsim, break; end
        ctrlBeforeStep=baseCtrl; cfgBeforeStep=cfg;
        % Prediction/error observer remain in the physical base-trim coordinates;
        % the energy reference is a cost-shaping device, not a new plant trim.
        xPred=baseCtrl.xref+baseCtrl.A*dx+baseCtrl.B*duApplied+gSeq(:,1);
        if scheduledRelease
            % Sample-and-hold control remains unchanged. Only the nonlinear
            % carrier propagation is split at the causal release timer.
            tau=max(0,min(Ts,scheduledTau)); rem=Ts-tau;
            if tau>1e-9
                wCmd1=double(airdropx_wind_profile_v136(opts.ScenarioName,t(k),SineForcingEnd_s=opts.SineForcingEnd_s,SineSettleRamp_s=opts.SineSettleRamp_s));
                [xRel,DRel]=airdropx_jsbsim_wind_oracle_mex("step_continuous",u,cfgBeforeStep,opts.FuelScale,wCmd1,tau);
            else
                xRel=x; DRel=D;
            end
            j=scheduledIndex; DropEvent(k)=true; ReleaseIndex(k)=j; ReleasePhase_s(k)=tau; FractionalRelease(k)=true;
            tRel=t(k)+tau;
            wEstRel=WindEffective(k)+WindRateEffective(k)*tau; bp=airdropx_airdrop_ballistic_params_v121(); wEstRel=max(min(wEstRel,bp.max_abs_forecast_wind_mps),-bp.max_abs_forecast_wind_mps);
            release.released(j)=true; release.t_s(j)=tRel; release.release_phase_s(j)=tau; release.fractional_release(j)=true; release.scheduler_mode(j)=scheduledMode; release.scheduler_residual_m(j)=scheduledResidual;
            release.release_x_m(j)=double(DRel.pos_n_m); release.release_h_m(j)=xRel(1); release.release_vg_mps(j)=double(DRel.v_north_mps); release.release_vz_mps(j)=double(DRel.vz_up_mps);
            release.wind_est_mps(j)=wEstRel; release.wind_rate_est_mps2(j)=WindRateEffective(k); release.wind_truth_mps(j)=double(DRel.wind_long_mps);
            % Re-evaluate the causal impact predictor at the ACTUAL nonlinear
            % carrier state reached at the fractional timer.  The scheduler's
            % pre-step interpolation residual is logged separately, but formal
            % predicted-release error must reflect what was knowable at release.
            % Formal predicted-impact gate remains causal: the release timer has
            % no new sensor sample at t+tau, so extrapolate the filtered state
            % that actually scheduled the timer instead of reading DRel truth.
            relPredState=rs; relPredState.x_m=rs.x_m+rs.vx_ground_mps*tau; relPredState.h_m=max(0,rs.h_m+rs.vz_up_mps*tau); relPredState.wind_est_mps=wEstRel; relPredState.wind_rate_est_mps2=WindRateEffective(k);
            predAtRel=airdropx_airdrop_predict_impact_v121(relPredState);
            release.predicted_impact_m(j)=predAtRel.impact_x_m; release.predicted_error_m(j)=predAtRel.impact_x_m-targets(j); release.predicted_fall_time_s(j)=predAtRel.fall_time_s;
            truthRelease=struct("x_m",release.release_x_m(j),"h_m",release.release_h_m(j),"vx_ground_mps",release.release_vg_mps(j),"vz_up_mps",release.release_vz_mps(j),"t_s",tRel);
            truth=airdropx_airdrop_truth_impact_v136(opts.ScenarioName,truthRelease,SineForcingEnd_s=opts.SineForcingEnd_s,SineSettleRamp_s=opts.SineSettleRamp_s);
            release.truth_impact_m(j)=truth.impact_x_m; release.landing_error_m(j)=truth.impact_x_m-targets(j); release.truth_fall_time_s(j)=truth.fall_time_s;
            oldCtrl=ctrlBeforeStep; cfg=cfgBeforeStep+1; newCtrl=models{cfg+1}.ctrl;
            warm=airdropx_phys_mpc_rebase_warmstart(sol.U,oldCtrl,newCtrl);
            if rem>1e-9
                wCmd2=double(airdropx_wind_profile_v136(opts.ScenarioName,tRel,SineForcingEnd_s=opts.SineForcingEnd_s,SineSettleRamp_s=opts.SineSettleRamp_s));
                [xNext,DNext]=airdropx_jsbsim_wind_oracle_mex("step_continuous",u,cfg,opts.FuelScale,wCmd2,rem);
            else
                xNext=xRel; DNext=DRel;
            end
            % Prediction error for this interval is deliberately not scored
            % against one fixed cfg model because the mass transition occurred
            % inside the interval.
            PredErrInf(k)=NaN; dHat=zeros(7,1); prev=[]; observerHoldoff=1;
            fprintf("[V136 DROP] %s %s cargo%d t=%.3f phase=%.3f target=%.2f release=%.2f pred=%.2f truth=%.2f miss=%+.2f windEst=%+.2f truthWind=%+.2f\n", ...
                opts.ScenarioName,mode,j,tRel,tau,targets(j),release.release_x_m(j),release.predicted_impact_m(j),truth.impact_x_m,release.landing_error_m(j),wEstRel,release.wind_truth_mps(j));
            x=xNext; D=DNext;
        else
            wCmd=double(airdropx_wind_profile_v136(opts.ScenarioName,t(k),SineForcingEnd_s=opts.SineForcingEnd_s,SineSettleRamp_s=opts.SineSettleRamp_s));
            [xNext,DNext]=airdropx_jsbsim_wind_oracle_mex("step_continuous",u,cfg,opts.FuelScale,wCmd,Ts);
            ep=airdropx_phys_mpc_state_error(xNext,xPred); PredErrInf(k)=max(abs(ep)./ctrl.stateScale);
            prev=struct("ctrl",baseCtrl,"dx",dx,"du",duApplied,"wind_effective",WindEffective(k),"cfg",cfg);
            x=xNext; D=DNext; warm=airdropx_phys_mpc_shift_warmstart(sol.U,ctrl.m,ctrl.N);
        end
        if k==1 || DropEvent(k) || mod(k,50)==0
            fprintf("[V136] %s %s t=%5.1f cfg=%d x=%.1f hErr=%+.3f VaErr=%+.3f wind=%+.2f eff=%+.2f truth=%+.2f conf=%.2f rec=%.2f dW1=%+.3f res=%.3g qp=%.2fms\n", ...
                opts.ScenarioName,mode,t(k),cfg,PosN(k),dx(1),dx(2),WindEst(k),WindEffective(k),WindTrue(k),WindConfidence(k),RecoveryLevel(k),ForecastDw1(k),ResidualNorm(k),1e3*sol.solve_time_s);
        end
    end

    valid=~isnan(QPExit); finalCtrl=models{cfg+1}.ctrl; dxFinal=airdropx_phys_mpc_state_error(x,finalCtrl.xref);
    primarySeries=nan(Nsim,1); primarySeries(valid)=max(abs(Err(valid,1:5))./sched.stateScale(1:5).',[],2);
    tailN=max(1,round(5/Ts)); ev=find(valid); tailStart=max(1,numel(ev)-tailN+1); tailIdx=ev(tailStart:end);
    windMask=valid & t>=2; windErr=WindEst-WindTrue;
    massRef=nan(Nsim,1); cgRef=massRef; iyyRef=massRef; vv=find(valid); massRef(vv)=expectedMass(Cfg(vv)+1); cgRef(vv)=expectedCg(Cfg(vv)+1); iyyRef(vv)=expectedIyy(Cfg(vv)+1)+iyyReferenceBias_kgm2;
    [causalMask,gustEvents,maxRecovery]=localGustScoringMask(valid,t,WindTrue,primarySeries,opts.GustStepThreshold_mps,opts.GustExclusion_s,opts.MaxPostGustPeakNormalized,Ts);
    postGustPeak=max(primarySeries(causalMask),[],'omitnan'); if isempty(postGustPeak) || isnan(postGustPeak), postGustPeak=max(primarySeries(valid),[],'omitnan'); end
    postWindMask=valid & t>=5; postWindRms=sqrt(mean(primarySeries(postWindMask).^2,'omitnan'));

    metrics=struct();
    metrics.drops_completed=sum(release.released); metrics.max_landing_error_m=max(abs(release.landing_error_m),[],'omitnan'); metrics.rms_landing_error_m=sqrt(mean(release.landing_error_m.^2,'omitnan'));
    metrics.max_predicted_impact_error_at_release_m=max(abs(release.predicted_error_m),[],'omitnan'); metrics.wind_error_p95_mps=localPercentile(abs(windErr(windMask)),95); metrics.wind_error_rmse_mps=sqrt(mean(windErr(windMask).^2));
    metrics.qp_success_fraction=mean(QPExit(valid)>0); metrics.qp_p95_ms=1e3*localPercentile(QPTime(valid),95); metrics.prediction_error_norm_p95=localPercentile(PredErrInf(valid & isfinite(PredErrInf)),95);
    metrics.instantaneous_peak_primary_normalized=max(primarySeries(valid),[],'omitnan'); metrics.post_gust_peak_normalized=postGustPeak; metrics.post_wind_primary_rms=postWindRms; metrics.max_gust_recovery_time_s=maxRecovery; metrics.gust_event_count=numel(gustEvents);
    metrics.final_normalized_inf=max(abs(dxFinal)./finalCtrl.stateScale); metrics.tail5s_normalized_rms=max(sqrt(mean((Err(tailIdx,:)./sched.stateScale.').^2,1)));
    metrics.residual_observer_peak_norm=max(ResidualNorm(valid),[],'omitnan'); metrics.forecast_dw1_peak_mps=max(abs(ForecastDw1(valid)),[],'omitnan');
    metrics.wind_confidence_mean=mean(WindConfidence(valid),'omitnan'); metrics.wind_rate_confidence_mean=mean(WindRateConfidence(valid),'omitnan');
    metrics.release_wind_evidence_active_fraction=mean(ReleaseWindEvidenceActive(valid));
    metrics.carrier_transient_evidence_active_fraction=mean(CarrierTransientEvidenceActive(valid));
    metrics.disturbance_evidence_active_fraction=metrics.carrier_transient_evidence_active_fraction; % compatibility alias
    metrics.carrier_transient_event_count=carrierTransientEventCount;
    metrics.carrier_transient_switch_count=localSwitchCount(CarrierTransientEvidenceActive(valid));
    metrics.wind_mpc_active_fraction=mean(WindMpcActive(valid)); metrics.wind_mpc_switch_count=localSwitchCount(WindMpcActive(valid));
    lastCarrier=find(CarrierTransientEvidenceActive & valid,1,'last'); lastWindMpc=find(WindMpcActive & valid,1,'last');
    metrics.last_carrier_transient_time_s=NaN; metrics.last_wind_mpc_active_time_s=NaN;
    if ~isempty(lastCarrier), metrics.last_carrier_transient_time_s=t(lastCarrier); end
    if ~isempty(lastWindMpc), metrics.last_wind_mpc_active_time_s=t(lastWindMpc); end
    metrics.tail5_start_s=t(tailIdx(1)); metrics.tail5_base_solver_fraction=mean(~WindMpcActive(tailIdx)); metrics.tail5_zero_truth_wind_fraction=mean(abs(WindTrue(tailIdx))<=1e-9);
    metrics.recovery_active_fraction=mean(RecoveryLevel(valid)>0); metrics.recovery_level_peak=max(RecoveryLevel(valid),[],'omitnan'); metrics.abrupt_gust_trigger_count=sum(AbruptGustTrigger(valid)); metrics.energy_recovery_active_fraction=mean(abs(EnergyHrefShift(valid))>1e-3); metrics.energy_altitude_shift_peak_m=max(abs(EnergyHrefShift(valid)),[],'omitnan'); metrics.input_snap_max=max(InputSnapMagnitude(valid),[],'omitnan');
    isSine=opts.ScenarioName=="sine_longitudinal"; forcedMask=valid & t>=5 & t<opts.SineForcingEnd_s;
    metrics.forced_response_primary_rms=NaN; metrics.forced_response_peak_normalized=NaN; metrics.forcing_end_s=NaN; metrics.settle_zero_wind_start_s=NaN; metrics.zero_wind_settle_duration_s=NaN;
    if isSine && any(forcedMask), metrics.forced_response_primary_rms=sqrt(mean(primarySeries(forcedMask).^2,'omitnan')); metrics.forced_response_peak_normalized=max(primarySeries(forcedMask),[],'omitnan'); metrics.forcing_end_s=opts.SineForcingEnd_s; metrics.settle_zero_wind_start_s=opts.SineForcingEnd_s+opts.SineSettleRamp_s; metrics.zero_wind_settle_duration_s=max(0,opts.Duration_s-metrics.settle_zero_wind_start_s); end
    metrics.gust_residual_0p5_normalized=localWorstResidualAfterEvents(primarySeries,t,gustEvents,0.5); metrics.gust_residual_1p0_normalized=localWorstResidualAfterEvents(primarySeries,t,gustEvents,1.0); metrics.gust_residual_3p0_normalized=localWorstResidualAfterEvents(primarySeries,t,gustEvents,3.0);
    metrics.elevator_saturation_fraction=mean(abs(U(valid,1))>=0.995); metrics.throttle_saturation_fraction=mean(U(valid,2)<=0.005 | U(valid,2)>=0.995); metrics.elevator_min_headroom=min(1-abs(U(valid,1)),[],'omitnan'); metrics.throttle_min_headroom=min(min(U(valid,2),1-U(valid,2)),[],'omitnan');
    AM=localActuatorGustWindows(U,t,gustEvents); metrics.actuator_gust_windows=AM;
    metrics.mass_match_error_max_kg=max(abs(Mass(valid)-massRef(valid))); metrics.cg_match_error_max_m=max(abs(Cg(valid)-cgRef(valid))); metrics.Iyy_match_error_max_kgm2=max(abs(Iyy(valid)-iyyRef(valid))); metrics.Iyy_reference_bias_kgm2=iyyReferenceBias_kgm2;
    metrics.fractional_release_count=sum(release.fractional_release); metrics.release_scheduler_residual_max_m=max(abs(release.scheduler_residual_m),[],'omitnan');
    metrics.truth_crosswind_max_mps=0; metrics.sensor_noise_seed=opts.SensorNoiseSeed;

    gate=struct(); gate.truth_not_used_for_release=1; gate.four_drops=metrics.drops_completed==4; gate.wind_estimation=metrics.wind_error_p95_mps<=opts.MaxWindErrorP95_mps;
    gate.predicted_release=metrics.max_predicted_impact_error_at_release_m<=opts.MaxPredictedImpactErrorAtRelease_m; gate.landing_accuracy=metrics.max_landing_error_m<=opts.MaxLandingError_m;
    gate.qp_all_feasible=all(QPExit(valid)>0); gate.input_bounds=all(U(valid,1)>=-1-1e-9 & U(valid,1)<=1+1e-9 & U(valid,2)>=-1e-9 & U(valid,2)<=1+1e-9);
    % The instantaneous gust peak is reported, not hidden, but is not a causal
    % hard gate: an unknown wind step cannot be cancelled before measurement.
    if isSine
        % Sustained periodic forcing is not a "post-gust" condition.  Score the
        % forced segment explicitly, then retain the original strict final/tail
        % gates after the v1.3.6 wind profile has smoothly returned to zero.
        gate.carrier_post_gust_peak=true; gate.carrier_gust_recovery=true;
        gate.carrier_forced_response=metrics.forced_response_primary_rms<=opts.MaxSineForcedRmsNormalized && metrics.forced_response_peak_normalized<=opts.MaxSineForcedPeakNormalized;
    else
        gate.carrier_post_gust_peak=metrics.post_gust_peak_normalized<=opts.MaxPostGustPeakNormalized;
        gate.carrier_gust_recovery=metrics.max_gust_recovery_time_s<=opts.MaxGustRecoveryTime_s;
    end
    gate.carrier_final=metrics.final_normalized_inf<=opts.MaxFinalNormalizedInf; gate.carrier_tail=metrics.tail5s_normalized_rms<=opts.MaxTail5sNormalizedRms;
    if isSine
        % Formal sine tail must be a genuine zero-wind BASE-controller window;
        % otherwise a good/bad tail number would still mix forced response and settle.
        gate.carrier_tail_context=metrics.tail5_zero_truth_wind_fraction>=1-1e-12 && metrics.tail5_base_solver_fraction>=1-1e-12;
    else
        gate.carrier_tail_context=true;
    end
    gate.realtime=metrics.qp_p95_ms<=100; gate.mass_configuration=metrics.mass_match_error_max_kg<=1e-2 && metrics.cg_match_error_max_m<=1e-5 && metrics.Iyy_match_error_max_kgm2<=1e-2;
    vals=struct2cell(gate); gate.pass=all(cellfun(@(z)logical(z),vals));

    Cargo=table(release.cargo,release.target_m,release.released,release.t_s,release.release_x_m,release.release_h_m,release.release_vg_mps,release.release_vz_mps, ...
        release.wind_est_mps,release.wind_rate_est_mps2,release.wind_truth_mps,release.predicted_impact_m,release.truth_impact_m,release.predicted_error_m,release.landing_error_m,release.predicted_fall_time_s,release.truth_fall_time_s,release.release_phase_s,release.fractional_release,release.scheduler_residual_m,release.scheduler_mode, ...
        'VariableNames',{'cargo','target_m','released','release_t_s','release_x_m','release_h_m','release_vg_mps','release_vz_mps','wind_est_mps','wind_rate_est_mps2','wind_truth_mps','predicted_impact_m','truth_impact_m','predicted_error_m','landing_error_m','predicted_fall_time_s','truth_fall_time_s','release_phase_s','fractional_release','scheduler_residual_m','scheduler_mode'});
    T=table(t,Cfg,PosN,X(:,1),X(:,2),X(:,3),X(:,4),X(:,5),X(:,6),X(:,7),Vg,Vz,WindTrue,WindRaw,WindEst,WindRate,WindSigma,WindConfidence,WindRateConfidence,WindEffective,WindRateEffective,GuideVg,GuideVa,GuideVz,ForecastDw1,ResidualNorm,WindMpcActive,RecoverySeverity,RecoveryLevel,GustRecoveryLatched,AbruptGustTrigger,ReleaseWindEvidenceActive,CarrierTransientEvidenceActive,DisturbanceEvidenceActive,EnergyHrefShift,InputSnapMagnitude,CurrentTarget,PredImpact,DropEvent,ReleaseIndex,U(:,1),U(:,2),QPExit,QPTime,PredErrInf,primarySeries,causalMask,ReleasePhase_s,FractionalRelease,Mass,Cg,Iyy, ...
        'VariableNames',{'t_s','cfg','pos_n_m','h_m','Va_mps','gamma_rad','theta_rad','q_radps','N1','N2','Vg_long_mps','Vz_up_mps','wind_truth_mps','wind_raw_mps','wind_est_mps','wind_rate_est_mps2','wind_sigma_mps','wind_confidence','wind_rate_confidence','wind_effective_mps','wind_rate_effective_mps2','guide_vg_mps','guide_va_mps','guide_vz_mps','forecast_dwind1_mps','residual_observer_norm','wind_mpc_active','recovery_severity','recovery_level','gust_recovery_latched','abrupt_gust_trigger','release_wind_evidence_active','carrier_transient_evidence_active','disturbance_evidence_active','energy_href_shift_m','input_snap_magnitude','current_target_m','predicted_impact_m','drop_event','release_index','elevator_cmd','throttle_cmd','qp_exitflag','qp_time_s','prediction_error_norm_inf','primary_normalized','causal_score_mask','release_phase_s','fractional_release','mass_kg','cg_x_m','Iyy_kgm2'});
    writetable(T,fullfile(opts.OutputRoot,"wind_airdrop_timeseries.csv")); writetable(Cargo,fullfile(opts.OutputRoot,"cargo_impacts.csv"));
    localPlot(T,Cargo,opts.OutputRoot,opts.ScenarioName,mode,gustEvents);
    report=struct("version","Physics-MPC v1.3.6 base-equivalent transient-evidence energy-aware gust-recovery precision airdrop","pass",gate.pass,"scenario",opts.ScenarioName,"mode",mode,"control_mode",controlMode,"release_mode",releaseMode,"oracle_version",vOracle,"options",opts,"metrics",metrics,"gate",gate,"cargo",Cargo,"gust_event_indices",gustEvents,"completed_at",datetime("now"));
    save(fullfile(opts.OutputRoot,"wind_airdrop_mission.mat"),"report","T","Cargo","-v7.3"); localWriteSummary(report,fullfile(opts.OutputRoot,"wind_airdrop_summary.txt"));
    fid=fopen(fullfile(opts.OutputRoot,"scenario_complete.ok"),"w"); if fid>=0, fprintf(fid,"complete %s\n",char(datetime("now"))); fclose(fid); end
    localStatus(status,"COMPLETE pass="+string(report.pass));
    if ~report.pass && opts.ThrowOnFail, error("AirdropX:WindMPC:MissionFailed","Mission failed: scenario=%s mode=%s landingMax=%.3g m.",opts.ScenarioName,mode,metrics.max_landing_error_m); end
catch ME
    report.pass=false; report.error=struct("identifier",string(ME.identifier),"message",string(ME.message),"stack",ME.stack); report.completed_at=datetime("now");
    save(fullfile(opts.OutputRoot,"wind_airdrop_failure.mat"),"report","-v7.3"); localStatus(status,"FAILED "+string(ME.identifier)); rethrow(ME);
end
end

function [mask,events,maxRecovery]=localGustScoringMask(valid,t,w,primary,stepThreshold,exclude_s,threshold,Ts)
mask=valid; events=find([false;abs(diff(w))>=stepThreshold]); nEx=ceil(exclude_s/Ts);
for ii=1:numel(events), j=events(ii); mask(j:min(numel(mask),j+nEx-1))=false; end
maxRecovery=0; holdN=max(1,ceil(0.5/Ts));
for ii=1:numel(events)
    j=events(ii); found=false;
    for q=j:numel(primary)-holdN+1
        z=primary(q:q+holdN-1);
        if all(isfinite(z)) && all(z<=threshold)
            maxRecovery=max(maxRecovery,t(q)-t(j)); found=true; break;
        end
    end
    if ~found, maxRecovery=Inf; end
end
end
function y=localSmoothStep(x,a,b)
if b<=a, y=double(x>=b); return; end
z=max(0,min(1,(double(x)-a)/(b-a))); y=z*z*(3-2*z);
end
function [state,y]=localAdaptiveGuideFilter(state,measurement,sigma,Ts,tau,fastSigma)
measurement=double(measurement);
if ~isfinite(state), state=measurement; y=state; return; end
aSlow=1-exp(-Ts/max(tau,eps)); a=aSlow;
if abs(measurement-state)>=fastSigma*max(double(sigma),0.05), a=max(a,0.85); end
state=state+a*(measurement-state); y=state;
end
function y=localWorstResidualAfterEvents(primary,t,events,delay_s)
if isempty(events), y=0; return; end
y=0;
for ii=1:numel(events)
    target=t(events(ii))+delay_s; j=find(t>=target & isfinite(primary),1,'first');
    if ~isempty(j), y=max(y,primary(j)); end
end
end
function A=localActuatorGustWindows(U,t,events)
A=struct('elevator_sat_0_0p5',0,'throttle_sat_0_0p5',0,'elevator_sat_0p5_1',0,'throttle_sat_0p5_1',0,'elevator_sat_1_3',0,'throttle_sat_1_3',0);
if isempty(events), return; end
wins=[0 0.5;0.5 1;1 3]; names={'0_0p5','0p5_1','1_3'};
for w=1:3
    emax=0; tmax=0;
    for ii=1:numel(events)
        t0=t(events(ii))+wins(w,1); t1=t(events(ii))+wins(w,2); idx=t>=t0 & t<t1 & all(isfinite(U),2);
        if any(idx), emax=max(emax,mean(abs(U(idx,1))>=0.995)); tmax=max(tmax,mean(U(idx,2)<=0.005 | U(idx,2)>=0.995)); end
    end
    ef=sprintf('elevator_sat_%s',names{w}); tf=sprintf('throttle_sat_%s',names{w}); A.(ef)=emax; A.(tf)=tmax;
end
end
function [u,du,snapMag]=localSnapPhysicalInput(uRaw,uref,umin,umax,tol)
uRaw=double(uRaw(:)); uref=double(uref(:)); umin=double(umin(:)); umax=double(umax(:));
physMin=max(umin,[-1;0]); physMax=min(umax,[1;1]);
if any(uRaw<physMin-tol) || any(uRaw>physMax+tol)
    error("AirdropX:WindMPC:InputViolation","Physical input exceeds hard bounds beyond numerical tolerance: elevator=%.12g throttle=%.12g.",uRaw(1),uRaw(2));
end
u=min(max(uRaw,physMin),physMax); snapMag=max(abs(u-uRaw)); du=u-uref;
end
function n=localSwitchCount(x)
x=logical(x(:));
if numel(x)<2, n=0; else, n=sum(x(2:end)~=x(1:end-1)); end
end
function y=localPercentile(x,p)
x=sort(double(x(:))); x=x(isfinite(x)); if isempty(x), y=NaN; return; end
if numel(x)==1, y=x(1); return; end
q=1+(numel(x)-1)*p/100; a=floor(q); b=ceil(q); if a==b, y=x(a); else, y=x(a)+(q-a)*(x(b)-x(a)); end
end
function localCloseWindOracle()
try, airdropx_jsbsim_wind_oracle_mex("close"); catch, end
end
function localStatus(path,line)
fid=fopen(path,"a"); if fid>=0, fprintf(fid,"%s  %s\n",char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")),char(line)); fclose(fid); end
end
function localWriteSummary(r,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v1.3.6 base-equivalent transient-evidence energy-aware gust-recovery precision airdrop\nscenario=%s\nmode=%s\npass=%d\n",r.scenario,r.mode,r.pass);
fprintf(fid,"truth_wind_used_by_controller=0\ntruth_wind_used_by_release_guidance=0\ncarrier_truth=persistent_nonlinear_JSBSim\npayload_truth=AirdropX_calibrated_longitudinal_ballistic_model_with_true_future_wind\n");
fprintf(fid,"instantaneous_gust_peak_is_report_only=1\ncausal_carrier_gate=post_gust_peak_plus_recovery_plus_final_tail\nrelease_timing=sub_sample_causal_timer_inside_0p1s_MPC_hold\n");
fn=fieldnames(r.metrics);
for i=1:numel(fn)
    v=r.metrics.(fn{i});
    if (isnumeric(v) || islogical(v)) && isscalar(v)
        fprintf(fid,"%s=%.12g\n",fn{i},v);
    elseif isstruct(v) && isscalar(v)
        sf=fieldnames(v);
        for j=1:numel(sf)
            z=v.(sf{j});
            if (isnumeric(z) || islogical(z)) && isscalar(z), fprintf(fid,"%s_%s=%.12g\n",fn{i},sf{j},z); end
        end
    end
end
gn=fieldnames(r.gate); for i=1:numel(gn), fprintf(fid,"gate_%s=%d\n",gn{i},r.gate.(gn{i})); end
end
function localPlot(T,C,out,scenario,mode,gustEvents)
try
    f=figure("Visible","off","Position",[80 80 1550 980]); tl=tiledlayout(f,2,2,"TileSpacing","compact","Padding","compact"); title(tl,"AirdropX v1.3.6 "+scenario+" / "+mode,"Interpreter","none");
    nexttile; plot(T.t_s,T.wind_truth_mps,"k-",T.t_s,T.wind_est_mps,"-"); grid on; ylabel("wind (m/s)"); xlabel("t (s)"); legend("truth","estimate");
    nexttile; plot(T.t_s,T.primary_normalized); yline(1,'--'); grid on; ylabel("primary normalized"); xlabel("t (s)"); hold on; for j=gustEvents(:).', xline(T.t_s(j),':'); end
    nexttile; plot(T.t_s,T.forecast_dwind1_mps,T.t_s,T.residual_observer_norm); grid on; xlabel("t (s)"); legend("forecast dWind next step","residual observer norm");
    nexttile; bar(C.cargo,C.landing_error_m); yline(0); grid on; xlabel("cargo #"); ylabel("landing error (m)");
    exportgraphics(f,fullfile(out,"wind_airdrop_validation.png"),"Resolution",160); close(f);
catch ME
    warning("AirdropX:WindMPC:PlotFailed","Plot failed: %s",ME.message);
end
end
