from pathlib import Path
root=Path(__file__).resolve().parents[1]
checks=[]
def ck(name,cond): checks.append((name,bool(cond)))
def txt(rel): return (root/rel).read_text(encoding='utf-8',errors='ignore')
mission=txt('matlab/airdrop/airdropx_wind_airdrop_mission_v133.m')
recover=txt('matlab/phys_mpc/airdropx_phys_mpc_build_recovery_bank_v133.m')
reweight=txt('matlab/phys_mpc/airdropx_phys_mpc_reweight_v132.m')
runner=txt('run_wind_disturbance_airdrop_v133_D.ps1')
point=txt('run_wind_disturbance_airdrop_point_v133_D.ps1')
installer=txt('install_wind_disturbance_airdrop_v133.ps1')
entry=txt('matlab/airdrop/airdropx_wind_airdrop_entry_v133.m')
finalize=txt('matlab/airdrop/airdropx_wind_airdrop_finalize_v133.m')
conf=txt('matlab/wind/airdropx_wind_confidence_v132.m')

# Retained validated structure.
ck('v130 physical Gw retained','airdropx_phys_mpc_load_wind_disturbance_v130' in mission)
ck('v121 ballistic predictor retained','airdropx_airdrop_predict_impact_v121' in mission)
ck('fractional release retained','airdropx_airdrop_fractional_release_v131' in mission and 'step_continuous",u,cfgBeforeStep' in mission)
ck('confidence helper retained','airdropx_wind_confidence_v132' in mission and 'wind_truth' not in conf.lower())
ck('strict carrier thresholds unchanged',all(x in mission for x in ['MaxGustRecoveryTime_s (1,1) double {mustBePositive} = 3.0','MaxPostGustPeakNormalized (1,1) double {mustBePositive} = 1.25','MaxTail5sNormalizedRms (1,1) double {mustBePositive} = 0.05']))

# v1.3.3 recovery-state-machine corrections.
ck('abrupt trigger requires confidence','strongWindEvidence=WindConfidence(k)>=opts.GustTriggerMinWindConfidence' in mission)
ck('abrupt trigger requires measured wind jump','abs(deltaWindEst)>=abruptThreshold' in mission)
ck('no raw estimator step alone can latch','gustTrigger=logical(eo.step_detected)' not in mission)
ck('no wind-rate confidence exit dependency','WindRateConfidence(k)<0.10' not in mission)
ck('state recovery quiet hold','RecoveryQuietHold_s' in mission and 'recoveryMetric<=opts.RecoveryReleaseNormalized' in mission)
ck('continuous wind assist capped','ContinuousAssistMax' in mission and 'opts.ContinuousAssistMax*localSmoothStep' in mission and 'ContinuousAssistMinRateConfidence' in mission)
ck('continuous assist default <= quarter', 'opts.ContinuousAssistMax (1,1) double {mustBeNonnegative} = 0.25' in mission)
ck('calm disturbance deadband','DisturbanceSolveDeadbandNormalized' in mission and 'airdropx_phys_mpc_solve(baseCtrl,x,warm)' in mission)

# Energy-aware recovery without expanding actuator authority.
ck('energy recovery is abrupt-gust gated','opts.UseEnergyRecovery && gustLatched && selectedLevel>=0.5' in mission)
ck('energy recovery requires throttle limit','rawThrottle>=opts.EnergyThrottleHigh' in mission and 'rawThrottle<=opts.EnergyThrottleLow' in mission)
ck('bounded altitude shift','EnergyMaxAltitudeShift_m' in mission and 'max(min(energyHrefState,opts.EnergyMaxAltitudeShift_m),-opts.EnergyMaxAltitudeShift_m)' in mission)
ck('low airspeed lowers H reference','energyDesired=opts.EnergyMaxAltitudeShift_m*sign(vaSignedN)' in mission)
ck('energy reference does not change plant trim observer','prev=struct("ctrl",baseCtrl' in mission and 'xPred=baseCtrl.xref+baseCtrl.A*dx' in mission)
ck('recovery bank is unified across cfg','for cfg=0:4' in recover and 'opts.MaxQMultiplier' in recover and 'opts.MaxRMultiplier' in recover)
ck('recovery terminal P still preserved','Qbar(rr,rr)=ctrl.P' in reweight)
ck('hard actuator bounds are not widened','physMin=max(umin,[-1;0])' in mission and 'physMax=min(umax,[1;1])' in mission)

# Numerical-boundary robustness.
ck('input snap helper exists','localSnapPhysicalInput' in mission)
ck('input snap only within explicit tolerance','InputSnapTolerance' in mission and 'uRaw<physMin-tol' in mission and 'uRaw>physMax+tol' in mission)
ck('applied snapped input used in predictor','baseCtrl.B*duApplied' in mission)
ck('input snap diagnostic exported','input_snap_max' in mission and 'InputSnapMax' in finalize)

# Diagnostics.
ck('abrupt trigger diagnostic','abrupt_gust_trigger_count' in mission and 'AbruptGustTriggerCount' in finalize)
ck('energy shift diagnostic','energy_altitude_shift_peak_m' in mission and 'EnergyAltitudeShiftPeak_m' in finalize)
ck('0p5/1/3 gust residual diagnostics',all(x in mission for x in ['gust_residual_0p5_normalized','gust_residual_1p0_normalized','gust_residual_3p0_normalized']))
ck('actuator diagnostics retained','throttle_saturation_fraction' in mission and 'elevator_saturation_fraction' in mission)

# No-popup process runner.
ck('full runner defaults child windows hidden',"if(-not $ShowChildWindows){$h.WindowStyle='Hidden'}" in runner)
ck('parallel missions use hidden helper','MatlabStartArgs $cmd $stdout $stderr' in runner)
ck('stage0/finalizer capture uses hidden helper','MatlabStartArgs $cmd $stdout $stderr -Wait' in runner)
ck('point runner defaults hidden',"if(-not $ShowChildWindows){$sp.WindowStyle='Hidden'}" in point)
ck('debug window opt-in exists','[switch]$ShowChildWindows' in runner and '[switch]$ShowChildWindows' in point)
ck('stdout stderr still captured','RedirectStandardOutput' in runner and 'RedirectStandardError' in runner and 'RedirectStandardOutput' in point and 'RedirectStandardError' in point)

# Packaging / routing.
ck('entry calls v133 mission','airdropx_wind_airdrop_mission_v133' in entry)
ck('runner calls v133 entry/finalizer','airdropx_wind_airdrop_entry_v133' in runner and 'airdropx_wind_airdrop_finalize_v133' in runner)
ck('installer copies v133 runners','run_wind_disturbance_airdrop_v133_D.ps1' in installer and 'run_wind_disturbance_airdrop_point_v133_D.ps1' in installer)
ck('no precompiled mex',not any(root.rglob('*.mexw64')))

for n,v in checks: print(('PASS' if v else 'FAIL')+': '+n)
print(f'\nTOTAL {sum(v for _,v in checks)}/{len(checks)} PASS')
raise SystemExit(0 if all(v for _,v in checks) else 1)
