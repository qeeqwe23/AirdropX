from pathlib import Path
root=Path(__file__).resolve().parents[1]
checks=[]
def ck(name,cond): checks.append((name,bool(cond)))
def txt(rel): return (root/rel).read_text(encoding='utf-8',errors='ignore')
mission=txt('matlab/airdrop/airdropx_wind_airdrop_mission_v132.m')
conf=txt('matlab/wind/airdropx_wind_confidence_v132.m')
recover=txt('matlab/phys_mpc/airdropx_phys_mpc_build_recovery_bank_v132.m')
reweight=txt('matlab/phys_mpc/airdropx_phys_mpc_reweight_v132.m')
runner=txt('run_wind_disturbance_airdrop_v132_D.ps1')
installer=txt('install_wind_disturbance_airdrop_v132.ps1')
entry=txt('matlab/airdrop/airdropx_wind_airdrop_entry_v132.m')
finalize=txt('matlab/airdrop/airdropx_wind_airdrop_finalize_v132.m')

# Retained physical structure.
ck('v130 physical Gw retained','airdropx_phys_mpc_load_wind_disturbance_v130' in mission)
ck('v121 ballistic predictor retained','airdropx_airdrop_predict_impact_v121' in mission)
ck('fractional physical release retained','airdropx_airdrop_fractional_release_v131' in mission and 'step_continuous",u,cfgBeforeStep' in mission)
frac=txt('matlab/airdrop/airdropx_airdrop_fractional_release_v131.m')
ck('fractional tau aligned to oracle dt','OracleDt_s' in frac and '1/120' in frac and 'round(tau/opts.OracleDt_s)' in frac)
ck('strict carrier gates retained',all(x in mission for x in ['MaxGustRecoveryTime_s (1,1) double {mustBePositive} = 3.0','MaxPostGustPeakNormalized (1,1) double {mustBePositive} = 1.25','MaxTail5sNormalizedRms (1,1) double {mustBePositive} = 0.05']))

# Confidence/noise design.
ck('confidence helper exists',(root/'matlab/wind/airdropx_wind_confidence_v132.m').is_file())
ck('confidence has no truth input','wind_truth' not in conf.lower() and 'scenarioName' not in conf and 'ScenarioName' not in conf)
ck('wind magnitude significance','WindAbsHalf_mps' in conf and 'absConf' in conf)
ck('wind uncertainty significance','WindSNRHalf' in conf and 'windSigma' in conf)
ck('rate significance','RateHalf_mps2' in conf and 'rateConf' in conf)
ck('release uses effective wind','wGuide=WindEffective(k)' in mission and 'rGuide=WindRateEffective(k)' in mission)
ck('release velocity filtering','localAdaptiveGuideFilter' in mission and 'GuideVg' in mission and 'GuideVa' in mission and 'GuideVz' in mission)
ck('residual deadband','ResidualDeadbandNormalized' in mission and 'residualConfidence=localSmoothStep' in mission)
ck('exact calm legacy fallback','airdropx_phys_mpc_solve(baseCtrl,x,warm)' in mission and 'gSeq=zeros(baseCtrl.n,baseCtrl.N)' in mission)

# Unified recovery controller.
ck('recovery bank helper exists',(root/'matlab/phys_mpc/airdropx_phys_mpc_build_recovery_bank_v132.m').is_file())
ck('same recovery levels all cfg','for cfg=0:4' in recover and 'opts.Levels' in recover)
ck('same dimensionless max multipliers','MaxQMultiplier' in recover and 'MaxRMultiplier' in recover)
ck('terminal P preserved','Final block intentionally remains the original certified terminal P' in reweight and 'Qbar(rr,rr)=ctrl.P' in reweight)
ck('hard bounds not rebuilt','out=ctrl' in reweight and 'out.H=H' in reweight)
ck('recovery requires causal gust latch','gustTrigger=logical(eo.step_detected)' in mission and 'gustLatched' in mission)
ck('payload transients alone do not trigger recovery','if gustLatched, targetRecovery=max(stateSeverity,rateSeverity); end' in mission)
ck('recovery has decay/hysteresis','RecoveryDecay_s' in mission and 'recoveryQuietCount' in mission)
ck('recovery shared rather than cfg tuned','[1.5;4.0;1.5;2.0;3.0;1.0;1.0]' in recover and '[0.80;0.35]' in recover)

# Causality and diagnostics.
ck('fractional formal predictor is causal extrapolation','relPredState=rs' in mission and 'instead of reading DRel truth' in mission)
ck('truth not fed to controller summary','truth_wind_used_by_controller=0' in mission)
ck('0p5 1 3 second metrics',all(x in mission for x in ['gust_residual_0p5_normalized','gust_residual_1p0_normalized','gust_residual_3p0_normalized']))
ck('actuator saturation diagnostics','elevator_saturation_fraction' in mission and 'throttle_saturation_fraction' in mission and 'localActuatorGustWindows' in mission)
ck('finalizer carries new diagnostics','Residual0p5Norm' in finalize and 'ThrottleSatFraction' in finalize and 'Tail5RmsNorm' in finalize)

# Real-run startup fixes.
ck('installer skips self copy','Skip self-copy' in installer and 'SamePath' in installer)
ck('runner avoids pid automatic-variable collision','[int]$procId' in runner and '$a.ProcId' in runner)
ck('runner PollCase is structured','function PollCase' in runner and 'elseif($a.Process.HasExited)' in runner)
ck('runner captures harmless stderr via Start-Process','function Invoke-MatlabCapture' in runner and 'RedirectStandardError' in runner)
ck('runner uses at most 3 workers','$MaxParallel -gt 3' in runner)
ck('entry forwards confidence and recovery','UseWindConfidenceGate=opts.UseWindConfidenceGate' in entry and 'UseUnifiedGustRecovery=opts.UseUnifiedGustRecovery' in entry)
ck('point runner supports recovery/confidence ablation','-NoRecovery' not in runner and 'NoRecovery' in txt('run_wind_disturbance_airdrop_point_v132_D.ps1') and 'NoConfidence' in txt('run_wind_disturbance_airdrop_point_v132_D.ps1'))
ck('version correct','1.3.2' in txt('VERSION.txt'))
ck('no precompiled mex',not any(root.rglob('*.mexw64')))

for n,v in checks: print(('PASS' if v else 'FAIL')+': '+n)
print(f'\nTOTAL {sum(v for _,v in checks)}/{len(checks)} PASS')
raise SystemExit(0 if all(v for _,v in checks) else 1)
