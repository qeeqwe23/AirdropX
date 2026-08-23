from pathlib import Path
import sys
root=Path(__file__).resolve().parents[1]
wind=root/'matlab'/'wind'
checks=[]
def ck(n,c): checks.append((n,bool(c)))
required=[
'airdropx_longitudinal_wind_measurement_v111.m',
'airdropx_longitudinal_wind_estimator_init_v111.m',
'airdropx_longitudinal_wind_estimator_step_v111.m',
'airdropx_longitudinal_wind_estimator_runtime_v111.m',
'airdropx_wind_simulink_harness_v111.m',
'airdropx_wind_estimation_entry_v111.m',
'airdropx_wind_finalize_v111.m']
ck('required_files',all((wind/x).is_file() for x in required))
est=(wind/'airdropx_longitudinal_wind_estimator_step_v111.m').read_text()
meas=(wind/'airdropx_longitudinal_wind_measurement_v111.m').read_text()
harness=(wind/'airdropx_wind_simulink_harness_v111.m').read_text()
entry=(wind/'airdropx_wind_estimation_entry_v111.m').read_text()
runner=(root/'run_wind_estimator_validation_v111_D.ps1').read_text()
inst=(root/'install_wind_estimator_v111.ps1').read_text().lower()
ck('estimator_has_no_truth_channels','windN' not in est and 'windE' not in est and 'windN' not in meas and 'windE' not in meas)
ck('geometry_uses_ground_air_vertical','Vg_mps-Vah' in meas and 'Va_mps^2-Vz_mps^2' in meas)
ck('tailwind_positive_documented','Tailwind is positive' in meas)
ck('kalman_two_state','s.x=[0;0]' in (wind/'airdropx_longitudinal_wind_estimator_init_v111.m').read_text())
ck('innovation_step_detection','StepImmediateNisThreshold' in est and 'StepNisThreshold' in est)
ck('truth_only_in_harness_scoring','windN=A(:,16)' in harness and 'windE=A(:,17)' in harness)
ck('sfunction_output_width_20','size(A,2)~=20' in harness)
ck('estimator_rate_0p1','EstimatorTs_s' in entry and '= 0.1' in entry)
ck('mc_50','MonteCarloSeeds' in entry and '= 50' in entry)
ck('strict_rmse_gate','MaxNoisyRmseMean_mps' in entry and '= 0.35' in entry)
ck('strict_p95_gate','MaxNoisyP95Mean_mps' in entry and '= 0.75' in entry)
ck('strict_step_gate','MaxStepSettling90_s' in entry and '= 0.50' in entry)
ck('sign_gate','MinSignAccuracy' in entry and '= 0.99' in entry)
ck('noise_reduction_gate','MinNoiseReductionRatio' in entry and '= 1.50' in entry)
ck('eight_scenarios','step_bidirectional' in (wind/'airdropx_wind_profile_manifest_v111.m').read_text() and 'sine_longitudinal' in (wind/'airdropx_wind_profile_manifest_v111.m').read_text())
ck('private_ic','wind_ic_v111_' in harness and 'localWritePrivateIc' in harness)
ck('no_shared_sim_params','airdropx_sim_params' not in harness)
ck('status_markers',all(x in harness for x in ['PRIVATE_IC_READY','MODEL_READY','CALIBRATION_SIM_START','CALIBRATION_SIM_DONE','MAIN_SIM_START','MAIN_SIM_DONE']))
ck('entry_passes_private_workdir','WorkDir=opts.OutputRoot' in entry and 'StatusFile=status' in entry)
ck('serial_calm_preflight','SERIAL HARNESS PREFLIGHT' in runner and 'harness_preflight_failure.txt' in runner)
ck('runner_error_summary','ErrorSummary' in runner and 'STDERR:' in runner and 'STDOUT:' in runner)
ck('no_cpp_mex_in_package',not any(p.suffix.lower() in {'.cpp','.mexw64'} for p in root.rglob('*') if p.is_file()))
ck('installer_does_not_rebuild','build_sfun' not in inst and 'mex -' not in inst and 'mex(' not in inst and 'mex ' not in inst.replace('no mex',''))
ck('readme_truth_separation','truth for scoring' in (root/'README.md').read_text())
passed=sum(v for _,v in checks)
for n,v in checks: print(('PASS' if v else 'FAIL'),n)
print(f'TOTAL {passed}/{len(checks)} PASS')
sys.exit(0 if passed==len(checks) else 1)
