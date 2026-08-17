from pathlib import Path
import re, sys
ROOT=Path(__file__).resolve().parents[1]
checks=[]
def ck(name, cond):
    checks.append((name,bool(cond)))

runner=(ROOT/'run_phys_mpc_drop_timing_compare_D.ps1').read_text()
mission=(ROOT/'matlab/phys_mpc/airdropx_phys_four_drop_closed_loop.m').read_text()
entry=(ROOT/'matlab/phys_mpc/airdropx_phys_drop_scenario_entry.m').read_text()
probe=(ROOT/'matlab/phys_mpc/airdropx_phys_cfg0_to_cfg4_jump_probe.m').read_text()
legacy=(ROOT/'matlab/phys_mpc/airdropx_phys_compare_drop_timing.m').read_text()
finalizer=(ROOT/'matlab/phys_mpc/airdropx_phys_finalize_drop_timing_compare_v052.m').read_text()

ck('mission exposes CloseOracleOnReturn', 'opts.CloseOracleOnReturn (1,1) logical = true' in mission)
ck('scenario entry disables explicit Oracle close', 'CloseOracleOnReturn",false' in entry)
ck('legacy in-process compare disabled', 'InProcessCompareDisabled' in legacy)
ck('runner uses Start-Process child PID', 'Start-Process' in runner and '$pid0=$p.Id' in runner)
ck('runner never blanket-kills MATLAB', 'Get-Process matlab' not in runner and 'Get-Process -Name MATLAB' not in runner)
ck('runner stop targets recorded child PID', 'Stop-Process -Id $pid0' in runner)
ck('runner has durable completion marker', 'scenario_complete.ok' in runner and 'probe_complete.ok' in runner)
ck('runner has shutdown grace after marker', 'ShutdownGraceSeconds' in runner and 'forced_exit_after_result' in runner)
ck('runner isolates cfg0->cfg4 probe', 'cfg0_to_cfg4_probe' in runner and 'airdropx_phys_cfg0_to_cfg4_probe_entry' in runner)
ck('runner isolates simultaneous mission', "DropTimes_s=[10 10 10 10]" in runner and "ScenarioName=string('simultaneous_4x')" in runner)
ck('runner retains 2 s schedule', 'DropTimes_s=[10 12 14 16]' in runner)
ck('probe evaluates cfg4 twice', probe.count('airdropx_phys_step(x0,u,p4)')==2)
ck('probe verifies repeatability', 'stateRepeat<=1e-12' in probe)
ck('probe verifies cfg4 mass/cg/Iyy', 'massErr' in probe and 'cgErr' in probe and 'iyyErr' in probe)
ck('probe refuses bad algebraic closure', 'algebraic_settle_converged' in probe)
ck('mission peak gate unchanged', 'opts.MaxPeakPrimaryNormalized (1,1) double {mustBePositive} = 1.0' in mission)
ck('mission final gate unchanged', 'opts.MaxFinalNormalizedInf (1,1) double {mustBePositive} = 0.10' in mission)
ck('mission tail gate unchanged', 'opts.MaxTail5sNormalizedRms (1,1) double {mustBePositive} = 0.05' in mission)
ck('mission 4x simultaneous semantics unchanged', 'cfgNow=sum(t(k)+1e-10>=opts.DropTimes_s)' in mission)
ck('finalizer never initializes Oracle', 'airdropx_phys_oracle_init' not in finalizer and 'airdropx_jsbsim_oracle_mex' not in finalizer)
ck('no packaged MEX', not any(ROOT.rglob('*.mexw64')))
ck('no packaged C++', not any(ROOT.rglob('*.cpp')))

bad=[n for n,p in checks if not p]
for n,p in checks: print(('PASS' if p else 'FAIL')+' | '+n)
print(f'\n{sum(p for _,p in checks)}/{len(checks)} checks passed')
sys.exit(1 if bad else 0)
