from pathlib import Path
import re
import sys

ROOT=Path(__file__).resolve().parents[1]
mission=(ROOT/'matlab/airdrop/airdropx_wind_airdrop_mission_v140.m').read_text()
avinit=(ROOT/'matlab/avionics/airdropx_avionics_init_v140.m').read_text()
avstep=(ROOT/'matlab/avionics/airdropx_avionics_step_v140.m').read_text()
cargo=(ROOT/'matlab/airdrop/airdropx_cargo_truth_plant_v140.m').read_text()
entry=(ROOT/'matlab/airdrop/airdropx_wind_airdrop_entry_v140.m').read_text()
full=(ROOT/'run_wind_disturbance_airdrop_v140_D.ps1').read_text()
point=(ROOT/'run_wind_disturbance_airdrop_point_v140_D.ps1').read_text()
audit=(ROOT/'matlab/airdrop/airdropx_phys_mpc_base_equivalence_audit_v140.m').read_text()
readme=(ROOT/'README.md').read_text()
dataaudit=(ROOT/'REAL_WORLD_DATA_AUDIT_v140.md').read_text()
flightfit=(ROOT/'matlab/phys_mpc/airdropx_phys_mpc_wind_disturbance_fit_flightlog_v140.m').read_text()

checks=[]
def ck(name, cond): checks.append((name,bool(cond)))

ck('formal realistic avionics default', 'opts.UseRealisticAvionics (1,1) logical = true' in mission)
ck('formal independent cargo default', 'opts.UseIndependentCargoTruth (1,1) logical = true' in mission)
ck('avionics layer initialized', 'airdropx_avionics_init_v140' in mission)
ck('avionics layer stepped from sensor stimulus', 'airdropx_avionics_step_v140(avionics,sensorTruth)' in mission)
ck('controller state is avionics estimate', 'xCtrl=av.x_est' in mission)
ck('release position is estimated', 'rs=struct("x_m",posCtrl,"h_m",xCtrl(1)' in mission)
ck('release fractional state is causal extrapolation', 'relPredState=rs' in mission and 'release timer has' in mission)
ck('base MPC receives xCtrl', 'airdropx_phys_mpc_solve(baseCtrl,xCtrl,warm)' in mission)
ck('disturbance MPC receives xCtrl', 'airdropx_phys_mpc_solve_disturbance_v130(ctrl,xCtrl,gSeq,warm)' in mission)
ck('energy MPC receives xCtrl', 'airdropx_phys_mpc_solve_disturbance_v130(ctrlEnergy,xCtrl,gSeq,warm)' in mission)
ck('no MPC solver receives xTruth', not re.search(r'airdropx_phys_mpc_solve(?:_disturbance_v130)?\([^\n;]*xTruth', mission))
ck('wind estimator uses measured Va/Vz/Vg', 'airdropx_longitudinal_wind_estimator_step_v111(est,VaMeas,VzMeas,VgMeas)' in mission)
ck('no online wGuide from truth wind', 'wGuide=WindTrue' not in mission and 'rGuide=WindTrue' not in mission)
ck('GNSS-style position sensor exists', 'gnss_pos_n_m' in avstep and 'SigmaGnssPos_m' in avinit)
ck('barometric altitude sensor exists', 'baro_h_m' in avstep and 'SigmaBaroAltitude_m' in avinit)
ck('pitot airspeed sensor exists', 'pitot_Va_mps' in avstep and 'SigmaAirspeed_mps' in avinit)
ck('AHRS pitch sensor exists', 'ahrs_theta_rad' in avstep)
ck('gyro pitch-rate sensor exists', 'gyro_q_radps' in avstep)
ck('engine telemetry exists', 'engine_N1' in avstep and 'engine_N2' in avstep)
ck('gamma derived from nav velocity', 'atan2(s.est.Vz_up_mps' in avstep)
ck('sensor bias random walk exists', 'BiasRw' in avinit and 'sqrt(Ts)*randn' in avstep)
ck('independent cargo plant default is used', 'airdropx_cargo_truth_plant_v140' in mission and 'opts.UseIndependentCargoTruth' in mission)
ck('independent cargo plant has vector drag', 'sp=hypot(vrx,vrz)' in cargo and 'az=-p.g_mps2-kd*sp*vrz' in cargo)
ck('truth release state stored separately', 'truth_release_x_m' in mission and 'release_x_est_m' in mission)
ck('truth/estimate timeseries pairs exist', 'pos_n_truth_m' in mission and 'pos_n_est_m' in mission and 'h_truth_m' in mission and 'h_est_m' in mission)
ck('truth isolation metrics emitted', 'jsbsim_state_truth_used_by_controller=0' in mission and 'jsbsim_navigation_truth_used_by_release=0' in mission)
ck('formal truth-isolation gates exist', 'gate.truth_not_used_for_release=1' in mission and 'gate.truth_not_used_by_controller=1' in mission)
ck('full runner forces realistic avionics', 'UseRealisticAvionics=true' in full)
ck('full runner forces independent cargo scoring', 'UseIndependentCargoTruth=true' in full)
ck('point runner ideal-state ablation is explicit', '[switch]$IdealStateFeedback' in point)
ck('point runner shared-cargo ablation is explicit', '[switch]$SharedCargoTruth' in point)
ck('hidden child windows preserved full runner', "WindowStyle='Hidden'" in full)
ck('hidden child windows preserved point runner', "WindowStyle='Hidden'" in point)
ck('base equivalence compares estimated state', 'h_est_m' in audit and 'Vg_est_mps' in audit)
ck('base equivalence uses realistic avionics', 'UseRealisticAvionics=true' in audit)
ck('base equivalence uses independent cargo scoring', 'UseIndependentCargoTruth=true' in audit)
ck('real-world audit documents offline A/B/Gw caveat', 'physics_bank.mat' in dataaudit and 'wind_disturbance_model_v130.mat' in dataaudit and 'flight-test system identification' in dataaudit)
ck('flight-log Gw replacement exists', 'sensor_estimated_flight_logs_not_JSBSim_truth' in flightfit and 'xTruth' not in flightfit and 'WindTrue' not in flightfit)
ck('README warns v140 errors not directly comparable', 'not directly comparable with v1.3.6' in readme)
ck('no precompiled executable artifacts in package', not any(p.suffix.lower() in {'.mexw64','.dll','.exe'} for p in ROOT.rglob('*') if p.is_file()))

# Online region: after sensor model acquisition and before metrics/scoring section.
start=mission.index('% ---------------- ONBOARD/CAUSAL SIDE ----------------')
end=mission.index('    valid=~isnan(QPExit)')
online=mission[start:end]
# Truth is permitted in plant propagation, truth-only release scoring, and log printf.
# Explicitly reject known causal/control assignments from JSBSim truth.
for pat, label in [
    (r'xCtrl\s*=\s*xTruth(?!;\s*posCtrl)', 'online xCtrl direct truth'),
    (r'posCtrl\s*=\s*PosN\(k\)', 'online posCtrl direct truth'),
    (r'wGuide\s*=\s*WindTrue', 'online release wind direct truth'),
    (r'VaMeas\s*=\s*xTruth\(2\)(?!\+opts\.SigmaAirspeed)', 'online Va direct truth'),
]:
    # These patterns may exist only in the explicit IdealStateFeedback branch before online marker.
    ck('reject '+label, re.search(pat, online) is None)

failed=[n for n,v in checks if not v]
for n,v in checks:
    print(f"{'PASS' if v else 'FAIL'}  {n}")
print(f"\nstatic_truth_isolation_audit_v140: {len(checks)-len(failed)}/{len(checks)} PASS")
if failed:
    print('FAILED:')
    for x in failed: print(' -',x)
    sys.exit(1)
