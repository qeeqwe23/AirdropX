param(
  [ValidateSet('Compare','PreviewOnly','PreviewQSoft')][string]$Mode='Compare',
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [int]$ScenarioTimeoutSeconds=300,
  [int]$ShutdownGraceSeconds=5
)
$ErrorActionPreference='Stop'
function Escape-MatlabString([string]$s) { return $s.Replace("'","''") }
function Invoke-IsolatedMatlab {
  param([string]$Label,[string]$Command,[string]$OutDir,[int]$TimeoutSeconds)
  New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
  $marker=Join-Path $OutDir 'scenario_complete.ok'; $status=Join-Path $OutDir 'scenario_status.txt'
  Remove-Item $marker -Force -ErrorAction SilentlyContinue
  $stdout=Join-Path $OutDir ($Label+'_stdout.txt'); $stderr=Join-Path $OutDir ($Label+'_stderr.txt')
  Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue
  Write-Host "[$Label] starting isolated MATLAB process..."
  $argLine="-batch `"$Command`""
  $p=Start-Process -FilePath $MatlabExe -ArgumentList $argLine -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
  $pid0=$p.Id; $start=Get-Date; $lastStatus=''; $markerSeen=$false
  while ($true) {
    $p.Refresh()
    if (Test-Path $status) {
      $line=Get-Content $status -Tail 1 -ErrorAction SilentlyContinue
      if ($line -and $line -ne $lastStatus) { Write-Host "[$Label] $line"; $lastStatus=$line }
    }
    if (Test-Path $marker) { $markerSeen=$true; break }
    if ($p.HasExited) { break }
    if (((Get-Date)-$start).TotalSeconds -gt $TimeoutSeconds) {
      Write-Warning "[$Label] timeout after $TimeoutSeconds s; terminating only PID $pid0."
      Stop-Process -Id $pid0 -Force -ErrorAction SilentlyContinue
      throw "[$Label] timed out. See $stdout / $stderr / $status"
    }
    Start-Sleep -Milliseconds 750
  }
  if (-not $markerSeen) {
    $exitCode=$null; try { $exitCode=$p.ExitCode } catch {}
    throw "[$Label] MATLAB exited before result marker (exit=$exitCode). See $stdout / $stderr / $status"
  }
  $forced=$false; $grace=Get-Date
  while ($true) {
    $p.Refresh(); if ($p.HasExited) { break }
    if (((Get-Date)-$grace).TotalSeconds -ge $ShutdownGraceSeconds) {
      $forced=$true
      Write-Warning "[$Label] outputs complete but MEX teardown did not exit in ${ShutdownGraceSeconds}s; terminating only PID $pid0."
      Stop-Process -Id $pid0 -Force -ErrorAction SilentlyContinue; Start-Sleep -Milliseconds 300; break
    }
    Start-Sleep -Milliseconds 250
  }
  $elapsed=((Get-Date)-$start).TotalSeconds
  Write-Host "[$Label] complete in $([math]::Round($elapsed,2)) s forced_exit_after_result=$forced"
  return [pscustomobject]@{Label=$Label;PID=$pid0;Elapsed_s=$elapsed;ForcedExitAfterResult=$forced;Stdout=$stdout;Stderr=$stderr;Status=$status}
}

if (-not (Test-Path $MatlabExe)) { throw "MATLAB not found: $MatlabExe" }
$phys=Join-Path $ProjectRoot 'matlab\phys_mpc'; $bank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v033\physics_bank.mat'; $mex=Join-Path $phys 'airdropx_jsbsim_oracle_mex.mexw64'
foreach($x in @($phys,$bank,$mex)) { if (-not (Test-Path $x)) { throw "Prerequisite missing: $x" } }
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v060_preview'; $pDir=Join-Path $out 'preview_only'; $qDir=Join-Path $out 'preview_qsoft'
New-Item -ItemType Directory -Force -Path $out | Out-Null
$rootEsc=Escape-MatlabString $ProjectRoot; $bankEsc=Escape-MatlabString $bank; $records=@()

if ($Mode -eq 'Compare' -or $Mode -eq 'PreviewOnly') {
  $dirEsc=Escape-MatlabString $pDir
  $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_preview_scenario_entry(string(pwd),OutputRoot=string('$dirEsc'),ScenarioName=string('preview_only'),EnableQSoft=false,BankPath=string('$bankEsc')); disp(r.pass);"
  $records += Invoke-IsolatedMatlab -Label 'preview_only' -Command $cmd -OutDir $pDir -TimeoutSeconds $ScenarioTimeoutSeconds
}
if ($Mode -eq 'Compare' -or $Mode -eq 'PreviewQSoft') {
  $dirEsc=Escape-MatlabString $qDir
  $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_preview_scenario_entry(string(pwd),OutputRoot=string('$dirEsc'),ScenarioName=string('preview_qsoft'),EnableQSoft=true,BankPath=string('$bankEsc')); disp(r.pass);"
  $records += Invoke-IsolatedMatlab -Label 'preview_qsoft' -Command $cmd -OutDir $qDir -TimeoutSeconds $ScenarioTimeoutSeconds
}
$life=Join-Path $out 'process_lifecycle_summary.txt'; $records | Format-Table -AutoSize | Out-String -Width 240 | Set-Content -Encoding UTF8 $life

# Finalizer never initializes JSBSim. If only q-soft was requested and preview_only does not exist,
# no comparison is attempted; the q-soft summary itself is the result.
if (($Mode -eq 'Compare') -or ($Mode -eq 'PreviewOnly')) {
  $pEsc=Escape-MatlabString $pDir; $qArg="string('')"
  if (Test-Path (Join-Path $qDir 'preview_mission.mat')) { $qEsc=Escape-MatlabString $qDir; $qArg="string('$qEsc')" }
  $outEsc=Escape-MatlabString $out
  $cmdF="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_finalize_preview_compare_v060(string(pwd),OutputRoot=string('$outEsc'),PreviewDir=string('$pEsc'),QSoftDir=$qArg); disp(r.comparison);"
  $finalLog=Join-Path $out 'finalize_terminal.txt'
  Write-Host '[finalize] comparing reactive baseline vs preview...'
  & $MatlabExe -batch $cmdF 2>&1 | Tee-Object -FilePath $finalLog
  if ($LASTEXITCODE -ne 0) { throw "Finalizer failed. See $finalLog" }
  Write-Host "Comparison: $(Join-Path $out 'preview_comparison.txt')"
}
Write-Host "Lifecycle: $life"
Write-Host '=== Physics-MPC v0.6.0 preview run complete ==='
