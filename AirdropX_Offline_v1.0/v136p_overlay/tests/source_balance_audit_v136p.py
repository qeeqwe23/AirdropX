from pathlib import Path
import sys,re
ROOT=Path(__file__).resolve().parents[1]
files=[
 ROOT/'matlab/airdrop/airdropx_wind_airdrop_mission_v136p.m',
 ROOT/'matlab/airdrop/airdropx_wind_airdrop_entry_v136p.m',
 ROOT/'matlab/airdrop/airdropx_phys_mpc_base_equivalence_audit_v136p.m',
 ROOT/'matlab/airdrop/airdropx_wind_airdrop_finalize_v136p.m',
 ROOT/'matlab/airdrop/airdropx_cargo_truth_params_v136p.m',
 ROOT/'matlab/airdrop/airdropx_cargo_truth_plant_v136p.m',
 ROOT/'matlab/avionics/airdropx_paper_sensor_init_v136p.m',
 ROOT/'matlab/avionics/airdropx_paper_sensor_step_v136p.m',
 ROOT/'run_paper_validation_v136p_D.ps1',ROOT/'run_paper_validation_point_v136p_D.ps1',ROOT/'run_paper_equivalence_audit_v136p_D.ps1',ROOT/'install_paper_validation_v136p.ps1']
okall=True
for f in files:
    t=f.read_text()
    issues=[]
    for a,b in [('(',')'),('[',']'),('{','}')]:
        if t.count(a)!=t.count(b): issues.append(f'{a}{b}:{t.count(a)}!={t.count(b)}')
    if f.suffix.lower()=='.m':
        first=t.splitlines()[0].strip(); name=f.stem
        if not re.search(r'\b'+re.escape(name)+r'\s*\(',first): issues.append('function/filename mismatch')
    ok=not issues; okall &= ok
    print(f"{'PASS' if ok else 'FAIL'} {f.name}: "+('raw delimiter/function checks ok' if ok else '; '.join(issues)))
if not okall: sys.exit(1)
