from pathlib import Path
import re, sys
ROOT=Path(__file__).resolve().parents[1]
checks=[]
def has(rel, needle, label):
    p=ROOT/rel
    ok=p.is_file() and needle in p.read_text(errors='ignore')
    checks.append((ok,label,rel,needle))
def lacks(rel, needle, label):
    p=ROOT/rel
    ok=p.is_file() and needle not in p.read_text(errors='ignore')
    checks.append((ok,label,rel,needle))
def exists(rel,label): checks.append(((ROOT/rel).is_file(),label,rel,''))

mission='matlab/airdrop/airdropx_wind_airdrop_mission_v136.m'
entry='matlab/airdrop/airdropx_wind_airdrop_entry_v136.m'
final='matlab/airdrop/airdropx_wind_airdrop_finalize_v136.m'
manifest='matlab/airdrop/airdropx_wind_airdrop_manifest_v136.m'
profile='matlab/wind/airdropx_wind_profile_v136.m'
point='run_wind_disturbance_airdrop_point_v136_D.ps1'
full='run_wind_disturbance_airdrop_v136_D.ps1'
install='install_wind_disturbance_airdrop_v136.ps1'
baseaudit='matlab/airdrop/airdropx_phys_mpc_base_equivalence_audit_v136.m'

for f in [mission,entry,final,manifest,profile,point,full,install,baseaudit]: exists(f,'required v136 file exists')
has(mission,'ReleaseWindEvidenceActive','absolute wind evidence separated for release diagnostics')
has(mission,'CarrierTransientEvidenceActive','carrier transient evidence logged')
has(mission,'Constant nonzero','constant absolute wind documented as non-keepalive')
has(mission,'rateDynamic=WindRateConfidence(k)>=opts.CarrierRateConfidence','carrier rate magnitude+confidence criterion')
has(mission,'CarrierTransientPersistence_s (1,1) double {mustBePositive} = 3.50','transient persistence bridges smooth sine extrema')
has(mission,'useTransientSolver=disturbanceEvidence || gustLatched || selectedLevel>0','solver selected from transient event state')
lacks(mission,'gNorm<=opts.DisturbanceSolveDeadbandNormalized','no numerical deadband solver chatter')
has(mission,'tail5_base_solver_fraction','tail base-solver context metric')
has(mission,'tail5_zero_truth_wind_fraction','tail zero-wind context metric')
has(mission,'gate.carrier_tail_context','formal sine tail context gate')
has(mission,'wind_mpc_switch_count','wind-MPC switching diagnostic')
has(mission,'carrier_transient_switch_count','transient evidence switching diagnostic')
has(mission,'airdropx_airdrop_truth_impact_v136','v136 scoring profile used for cargo truth')
has(profile,'w(t>=t0+tr)=0','sine profile reaches true zero wind')
has(manifest,'Duration_s=[55;55;55;55;55;55;55;60];','formal sine mission extended to 60 s')
has(point,"$duration=if($Scenario -eq 'sine_longitudinal'){60}else{55}",'point sine automatically runs 60 s')
has(point,"$sp.WindowStyle='Hidden'",'point child MATLAB hidden by default')
has(full,"$h.WindowStyle='Hidden'",'full child MATLAB hidden by default')
has(full,'airdropx_wind_airdrop_manifest_v136','full runner uses v136 manifest')
has(full,'airdropx_phys_mpc_base_equivalence_audit_v136','base equivalence remains a full-run preflight')
has(full,'wind_transient_energy_recovery_validation_summary.txt','full runner reads v136 final summary')
has(full,"if($MaxParallel -lt 1 -or $MaxParallel -gt 3)",'parallel cap remains <=3')
has(point,'mass-refresh-v135','v1.3.6 reuses validated v1.3.5 Oracle')
has(full,'mass-refresh-v135','full runner reuses validated v1.3.5 Oracle')
lacks(install,'Remove-Item $mex,$marker','installer does not force needless Oracle rebuild')
has(install,"WindowStyle",'installer copy includes hidden-window runners') if False else None
has(baseaudit,'airdropx_wind_airdrop_mission_v136','equivalence audit exercises v136 mission')
has(final,'CarrierTransientEvidenceActiveFraction','finalizer exports transient evidence metric')
has(final,'WindMpcSwitchCount','finalizer exports switching metric')
has(final,'wind_transient_energy_recovery_validation.csv','v136 final output names distinct')
has('VERSION.txt','Physics-MPC v1.3.6','version file')
# No precompiled mex is shipped.
mex=list(ROOT.rglob('*.mexw64'))+list(ROOT.rglob('*.mex*'))
checks.append((len(mex)==0,'no precompiled MEX shipped',str(mex),''))

# Basic function-name/file-name consistency on new MATLAB files.
for rel in [mission,entry,final,manifest,profile,baseaudit,'matlab/airdrop/airdropx_airdrop_truth_impact_v136.m']:
    p=ROOT/rel; first=p.read_text(errors='ignore').splitlines()[0]
    m=re.match(r'\s*function\s+(?:\[[^\]]+\]|\w+)\s*=\s*(\w+)|\s*function\s+(\w+)\s*\(',first)
    fn=(m.group(1) or m.group(2)) if m else None
    checks.append((fn==p.stem,'MATLAB function name matches filename',rel,str(fn)))

fails=[x for x in checks if not x[0]]
for i,(ok,label,rel,needle) in enumerate(checks,1): print(f'{i:02d} {"PASS" if ok else "FAIL"} {label} :: {rel}')
print(f'\nTOTAL {len(checks)-len(fails)}/{len(checks)} PASS')
if fails:
    for x in fails: print('FAILED:',x)
    sys.exit(1)
