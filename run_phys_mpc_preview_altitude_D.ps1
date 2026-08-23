param(
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [ValidateRange(1,3)][int]$MaxParallel=3,
  [int]$ScenarioTimeoutSeconds=300,
  [int]$ShutdownGraceSeconds=5,
  [switch]$Force
)
$ErrorActionPreference='Stop'
function Escape-MatlabString([string]$s) { return $s.Replace("'","''") }
if (-not (Test-Path $MatlabExe)) { throw "MATLAB not found: $MatlabExe" }
$phys=Join-Path $ProjectRoot 'matlab\phys_mpc'
$bank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v033\physics_bank.mat'
$mex=Join-Path $phys 'airdropx_jsbsim_oracle_mex.mexw64'
foreach($x in @($phys,$bank,$mex)) { if (-not (Test-Path $x)) { throw "Prerequisite missing: $x" } }
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v061_altitude_validation'
New-Item -ItemType Directory -Force -Path $out | Out-Null
$heights=@(); for($h=20;$h -le 200;$h+=10){$heights += [double]$h}
$pending=New-Object System.Collections.Queue
foreach($h in $heights){$pending.Enqueue($h)}
$active=@(); $records=@(); $rootEsc=Escape-MatlabString $ProjectRoot; $bankEsc=Escape-MatlabString $bank

function Add-Record([object]$a,[bool]$forced,[bool]$infra,[bool]$timedOut,[bool]$reused) {
  $elapsed=((Get-Date)-$a.Start).TotalSeconds
  return [pscustomobject]@{H_m=$a.H;PID=$a.PID;Elapsed_s=[math]::Round($elapsed,3);ForcedExitAfterResult=$forced;InfraFail=$infra;Timeout=$timedOut;Reused=$reused;Stdout=$a.Stdout;Stderr=$a.Stderr;Status=$a.Status}
}

while($pending.Count -gt 0 -or $active.Count -gt 0) {
  while($pending.Count -gt 0 -and $active.Count -lt $MaxParallel) {
    $h=[double]$pending.Dequeue(); $tag=('H{0:D3}_V050' -f [int]$h); $dir=Join-Path $out $tag
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
    $marker=Join-Path $dir 'scenario_complete.ok'; $mat=Join-Path $dir 'preview_mission.mat'
    if((-not $Force) -and (Test-Path $marker) -and (Test-Path $mat)) {
      Write-Host "[$tag] reusing complete result"
      $dummy=[pscustomobject]@{H=$h;PID=0;Start=Get-Date;Stdout='';Stderr='';Status=Join-Path $dir 'scenario_status.txt'}
      $records += Add-Record $dummy $false $false $false $true
      continue
    }
    Remove-Item $marker -Force -ErrorAction SilentlyContinue
    $stdout=Join-Path $dir ($tag+'_stdout.txt'); $stderr=Join-Path $dir ($tag+'_stderr.txt'); Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue
    $dirEsc=Escape-MatlabString $dir
    $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_preview_altitude_entry(string(pwd),OutputRoot=string('$dirEsc'),Height_m=$h,BankPath=string('$bankEsc')); disp(r.pass);"
    $p=Start-Process -FilePath $MatlabExe -ArgumentList "-batch `"$cmd`"" -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
    $a=[pscustomobject]@{H=$h;Tag=$tag;Process=$p;PID=$p.Id;Start=Get-Date;Dir=$dir;Marker=$marker;Status=Join-Path $dir 'scenario_status.txt';Stdout=$stdout;Stderr=$stderr;LastStatus='';MarkerAt=$null}
    $active += $a; Write-Host "[$tag] started PID $($p.Id)"
  }

  $keep=@()
  foreach($a in $active) {
    $a.Process.Refresh()
    if(Test-Path $a.Status) {
      $line=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue
      if($line -and $line -ne $a.LastStatus){Write-Host "[$($a.Tag)] $line"; $a.LastStatus=$line}
    }
    $done=$false
    if(Test-Path $a.Marker) {
      if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}
      if($a.Process.HasExited) {
        $records += Add-Record $a $false $false $false $false; $done=$true
      } elseif(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds) {
        Write-Warning "[$($a.Tag)] result complete but teardown did not exit; terminating only PID $($a.PID)."
        Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue
        $records += Add-Record $a $true $false $false $false; $done=$true
      }
    } elseif($a.Process.HasExited) {
      Write-Warning "[$($a.Tag)] MATLAB exited before completion marker."
      $records += Add-Record $a $false $true $false $false; $done=$true
    } elseif(((Get-Date)-$a.Start).TotalSeconds -gt $ScenarioTimeoutSeconds) {
      Write-Warning "[$($a.Tag)] timeout; terminating only PID $($a.PID)."
      Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue
      $records += Add-Record $a $false $true $true $false; $done=$true
    }
    if(-not $done){$keep += $a}
  }
  $active=$keep
  if($pending.Count -gt 0 -or $active.Count -gt 0){Start-Sleep -Milliseconds 500}
}

$records | Sort-Object H_m | Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $out 'process_lifecycle.csv')
$records | Sort-Object H_m | Format-Table -AutoSize | Out-String -Width 260 | Set-Content -Encoding UTF8 (Join-Path $out 'process_lifecycle_summary.txt')

$outEsc=Escape-MatlabString $out
$cmdF="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_finalize_altitude_validation_v061(string(pwd),OutputRoot=string('$outEsc'),BankPath=string('$bankEsc')); disp(r.pass);"
$finalLog=Join-Path $out 'finalize_terminal.txt'
Write-Host '[finalize] aggregating all 19 heights and auditing common bank controller...'
& $MatlabExe -batch $cmdF 2>&1 | Tee-Object -FilePath $finalLog
if($LASTEXITCODE -ne 0){throw "Altitude finalizer failed. See $finalLog"}
$summary=Join-Path $out 'altitude_validation_summary.txt'
Write-Host "Summary: $summary"
Write-Host "CSV: $(Join-Path $out 'altitude_validation.csv')"
if(Test-Path $summary) {
  $passLine=Get-Content $summary | Where-Object {$_ -match '^pass='} | Select-Object -First 1
  if($passLine -ne 'pass=1'){throw "Physics-MPC v0.6.1 altitude validation completed with one or more FAIL/missing heights. See $summary"}
}
Write-Host '=== Physics-MPC v0.6.1 ALTITUDE VALIDATION PASS ==='
