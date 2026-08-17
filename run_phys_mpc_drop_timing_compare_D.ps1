param(
  [ValidateSet('Compare','SimultaneousOnly')][string]$Mode='Compare',
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [double]$Duration=40,
  [int]$ScenarioTimeoutSeconds=180,
  [int]$ShutdownGraceSeconds=5,
  [switch]$RerunInterval2s,
  [switch]$NoBaseline
)
$ErrorActionPreference='Stop'

function Escape-MatlabString([string]$s) { return $s.Replace("'","''") }

function Invoke-IsolatedMatlab {
  param(
    [string]$Label,
    [string]$Command,
    [string]$OutDir,
    [string]$MarkerName,
    [string]$StatusName,
    [int]$TimeoutSeconds
  )
  New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
  $marker=Join-Path $OutDir $MarkerName
  $status=Join-Path $OutDir $StatusName
  Remove-Item $marker -Force -ErrorAction SilentlyContinue
  $stdout=Join-Path $OutDir ($Label+'_stdout.txt')
  $stderr=Join-Path $OutDir ($Label+'_stderr.txt')
  Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue
  Write-Host "[$Label] starting isolated MATLAB process..."
  $escapedCommand=$Command.Replace('\"','\\\"')
  $argLine="-batch `"$escapedCommand`""
  $p=Start-Process -FilePath $MatlabExe -ArgumentList $argLine -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
  $pid0=$p.Id
  $start=Get-Date
  $lastStatus=''
  $markerSeen=$false
  while ($true) {
    $p.Refresh()
    if (Test-Path $status) {
      $line=(Get-Content $status -Tail 1 -ErrorAction SilentlyContinue)
      if ($line -and $line -ne $lastStatus) { Write-Host "[$Label] $line"; $lastStatus=$line }
    }
    if (Test-Path $marker) { $markerSeen=$true; break }
    if ($p.HasExited) { break }
    if (((Get-Date)-$start).TotalSeconds -gt $TimeoutSeconds) {
      Write-Warning "[$Label] timeout after $TimeoutSeconds s; terminating only child PID $pid0."
      Stop-Process -Id $pid0 -Force -ErrorAction SilentlyContinue
      throw "[$Label] timed out before completion marker. See $stdout and $stderr and $status"
    }
    Start-Sleep -Milliseconds 750
  }
  if (-not $markerSeen) {
    $exitCode=$null
    try { $exitCode=$p.ExitCode } catch {}
    throw "[$Label] MATLAB exited before completion marker (exit=$exitCode). See $stdout and $stderr and $status"
  }

  # All result files are already saved before the marker is written. Allow a
  # normal MATLAB/MEX teardown briefly; if JSBSim destruction stalls, kill only
  # this child. This cannot corrupt the completed scenario files.
  $forced=$false
  $graceStart=Get-Date
  while ($true) {
    $p.Refresh()
    if ($p.HasExited) { break }
    if (((Get-Date)-$graceStart).TotalSeconds -ge $ShutdownGraceSeconds) {
      $forced=$true
      Write-Warning "[$Label] result is complete, but process teardown did not exit within ${ShutdownGraceSeconds}s; terminating only child PID $pid0."
      Stop-Process -Id $pid0 -Force -ErrorAction SilentlyContinue
      Start-Sleep -Milliseconds 300
      break
    }
    Start-Sleep -Milliseconds 250
  }
  $elapsed=((Get-Date)-$start).TotalSeconds
  Write-Host "[$Label] result complete in $([math]::Round($elapsed,2)) s; forced_exit_after_result=$forced"
  return [pscustomobject]@{Label=$Label;PID=$pid0;Elapsed_s=$elapsed;ForcedExitAfterResult=$forced;Stdout=$stdout;Stderr=$stderr;Status=$status;Marker=$marker}
}

if (-not (Test-Path $MatlabExe)) { throw "MATLAB not found: $MatlabExe" }
$phys=Join-Path $ProjectRoot 'matlab\phys_mpc'
$bank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v033\physics_bank.mat'
$mex=Join-Path $phys 'airdropx_jsbsim_oracle_mex.mexw64'
foreach($x in @($phys,$bank,$mex)) { if (-not (Test-Path $x)) { throw "Prerequisite missing: $x" } }
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v052_drop_timing_compare'
$probeDir=Join-Path $out 'simultaneous_cfg4_probe'
$simDir=Join-Path $out 'simultaneous_4x'
$intDir=Join-Path $out 'interval_2s'
New-Item -ItemType Directory -Force -Path $out,$probeDir,$simDir | Out-Null
$rootEsc=Escape-MatlabString $ProjectRoot
$bankEsc=Escape-MatlabString $bank
$records=@()

# Reuse the already completed v0.5.1 2 s result by default. This avoids paying
# again for a known-PASS scenario and, more importantly, avoids any dependency
# on its old process teardown behavior.
$existingInterval=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v051_drop_timing_compare\interval_2s'
if ($Mode -eq 'Compare') {
  $reuse=(-not $RerunInterval2s.IsPresent) -and (Test-Path (Join-Path $existingInterval 'four_drop_mission.mat')) -and (Test-Path (Join-Path $existingInterval 'four_drop_timeseries.csv'))
  if ($reuse) {
    $intDir=$existingInterval
    Write-Host "[interval_2s] reusing completed v0.5.1 result: $intDir"
  } else {
    New-Item -ItemType Directory -Force -Path $intDir | Out-Null
    $intEsc=Escape-MatlabString $intDir
    $cmdA="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_drop_scenario_entry(string(pwd),OutputRoot=string('$intEsc'),ScenarioName=string('interval_2s'),DropTimes_s=[10 12 14 16],Duration_s=$Duration,BankPath=string('$bankEsc')); disp(r.pass);"
    $records += Invoke-IsolatedMatlab -Label 'interval_2s' -Command $cmdA -OutDir $intDir -MarkerName 'scenario_complete.ok' -StatusName 'scenario_status.txt' -TimeoutSeconds $ScenarioTimeoutSeconds
  }
} else {
  if ((Test-Path (Join-Path $existingInterval 'four_drop_mission.mat')) -and (Test-Path (Join-Path $existingInterval 'four_drop_timeseries.csv'))) {
    $intDir=$existingInterval
  } elseif ((Test-Path (Join-Path $intDir 'four_drop_mission.mat')) -and (Test-Path (Join-Path $intDir 'four_drop_timeseries.csv'))) {
    # use v0.5.2 interval result if it exists
  } else {
    throw "SimultaneousOnly still needs an existing interval_2s result for final comparison. Run -Mode Compare once or keep the v0.5.1 interval result."
  }
}

# Probe the exact cfg0->cfg4 one-sample plant path in a fresh MATLAB process.
$probeEsc=Escape-MatlabString $probeDir
$cmdProbe="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_cfg0_to_cfg4_probe_entry(string(pwd),OutputRoot=string('$probeEsc'),BankPath=string('$bankEsc')); disp(r.pass);"
$records += Invoke-IsolatedMatlab -Label 'cfg0_to_cfg4_probe' -Command $cmdProbe -OutDir $probeDir -MarkerName 'probe_complete.ok' -StatusName 'probe_status.txt' -TimeoutSeconds ([math]::Min($ScenarioTimeoutSeconds,90))
$probeSummary=Join-Path $probeDir 'probe_summary.txt'
if (-not (Test-Path $probeSummary) -or -not (Select-String -Path $probeSummary -Pattern '^pass=1$' -Quiet)) {
  throw "Direct cfg0->cfg4 jump probe completed but did not PASS. Refusing the 40 s simultaneous mission. See $probeSummary"
}

# Run simultaneous four-payload release in another completely fresh process.
$simEsc=Escape-MatlabString $simDir
$cmdB="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_drop_scenario_entry(string(pwd),OutputRoot=string('$simEsc'),ScenarioName=string('simultaneous_4x'),DropTimes_s=[10 10 10 10],Duration_s=$Duration,BankPath=string('$bankEsc')); disp(r.pass);"
$records += Invoke-IsolatedMatlab -Label 'simultaneous_4x' -Command $cmdB -OutDir $simDir -MarkerName 'scenario_complete.ok' -StatusName 'scenario_status.txt' -TimeoutSeconds $ScenarioTimeoutSeconds

# Persist lifecycle evidence before finalization.
$life=Join-Path $out 'process_lifecycle_summary.txt'
$records | Format-Table -AutoSize | Out-String -Width 240 | Set-Content -Encoding UTF8 $life

# Finalization is intentionally a clean MATLAB process that only reads MAT/CSV
# files. It does not initialize or load the JSBSim Oracle.
$intEsc=Escape-MatlabString $intDir
$simEsc=Escape-MatlabString $simDir
$outEsc=Escape-MatlabString $out
$includeBaseline=(-not $NoBaseline.IsPresent).ToString().ToLower()
$finalLog=Join-Path $out 'finalize_terminal.txt'
$cmdF="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_finalize_drop_timing_compare_v052(string(pwd),OutputRoot=string('$outEsc'),IntervalDir=string('$intEsc'),SimultaneousDir=string('$simEsc'),IncludeExistingBaseline=$includeBaseline); disp(r.comparison);"
Write-Host '[finalize] combining isolated results...'
& $MatlabExe -batch $cmdF 2>&1 | Tee-Object -FilePath $finalLog
if ($LASTEXITCODE -ne 0) { throw "Finalizer failed with MATLAB exit code $LASTEXITCODE. See $finalLog" }
Write-Host "=== Physics-MPC v0.5.2 isolated comparison complete ==="
Write-Host "Comparison: $(Join-Path $out 'drop_timing_comparison.txt')"
Write-Host "Lifecycle:  $life"
