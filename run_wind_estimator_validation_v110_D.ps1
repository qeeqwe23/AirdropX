param(
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [int]$MaxParallel=3,
  [int]$TimeoutSeconds=240,
  [int]$ShutdownGraceSeconds=5,
  [switch]$Force
)
$ErrorActionPreference='Stop'; if($MaxParallel -lt 1 -or $MaxParallel -gt 3){throw 'MaxParallel must be 1..3'}
function E([string]$s){return $s.Replace("'","''")}
if(-not(Test-Path $MatlabExe)){throw "MATLAB not found: $MatlabExe"}
$windDir=Join-Path $ProjectRoot 'matlab\wind'; $sfun=Join-Path $ProjectRoot 'matlab\sfunc_jsbsim\sfun_airdropx_jsbsim.mexw64'
if(-not(Test-Path $sfun)){throw "Working JSBSim S-function MEX missing: $sfun"}
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v110_longitudinal_wind_validation'; New-Item -ItemType Directory -Force -Path $out | Out-Null
$rootEsc=E $ProjectRoot; $outEsc=E $out
$self=Join-Path $out 'synthetic_selftest.txt'; $selfEsc=E $self
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/wind'); r=airdropx_wind_synthetic_selftest_v110(string('$selfEsc')); disp(r);"
& $MatlabExe -batch $cmd 2>&1 | Tee-Object -FilePath (Join-Path $out 'synthetic_selftest_terminal.txt')
if($LASTEXITCODE -ne 0){throw 'Wind estimator synthetic self-test failed.'}
$manifest=Join-Path $out 'wind_profile_manifest.csv'; $manifestEsc=E $manifest
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/wind'); T=airdropx_wind_profile_manifest_v110(string('$manifestEsc')); disp(T);"
& $MatlabExe -batch $cmd 2>&1 | Tee-Object -FilePath (Join-Path $out 'manifest_terminal.txt')
if($LASTEXITCODE -ne 0 -or -not(Test-Path $manifest)){throw 'Wind profile manifest failed.'}
$jobs=Import-Csv $manifest; $queue=New-Object System.Collections.Queue; foreach($j in $jobs){$queue.Enqueue($j)}; $active=@(); $records=@()
while($queue.Count -gt 0 -or $active.Count -gt 0){
  while($queue.Count -gt 0 -and $active.Count -lt $MaxParallel){
    $j=$queue.Dequeue(); $tag=[string]$j.Scenario; $dur=[double]$j.Duration_s; $dir=Join-Path $out $tag; New-Item -ItemType Directory -Force -Path $dir | Out-Null
    $marker=Join-Path $dir 'scenario_complete.ok'; $mat=Join-Path $dir 'wind_estimation_validation.mat'
    if((-not $Force) -and (Test-Path $marker) -and (Test-Path $mat)){Write-Host "[$tag] reuse"; $records += [pscustomobject]@{Scenario=$tag;PID=0;Elapsed_s=0;ForcedExitAfterResult=$false;Reused=$true;InfraFail=$false;Timeout=$false}; continue}
    Remove-Item $marker -Force -ErrorAction SilentlyContinue
    $stdout=Join-Path $dir ($tag+'_stdout.txt'); $stderr=Join-Path $dir ($tag+'_stderr.txt'); $dirEsc=E $dir
    $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/wind'); addpath('matlab/sfunc_jsbsim'); r=airdropx_wind_estimation_entry_v110(string(pwd),OutputRoot=string('$dirEsc'),ScenarioName=string('$tag'),Duration_s=$dur); disp(r);"
    $p=Start-Process -FilePath $MatlabExe -ArgumentList "-batch `"$cmd`"" -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
    $active += [pscustomobject]@{Scenario=$tag;Process=$p;PID=$p.Id;Start=Get-Date;Marker=$marker;Status=(Join-Path $dir 'scenario_status.txt');Last='';MarkerAt=$null}; Write-Host "[$tag] started PID $($p.Id)"
  }
  $keep=@()
  foreach($a in $active){
    $a.Process.Refresh(); if(Test-Path $a.Status){$ln=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue; if($ln -and $ln -ne $a.Last){Write-Host "[$($a.Scenario)] $ln"; $a.Last=$ln}}
    $done=$false
    if(Test-Path $a.Marker){
      if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}
      if($a.Process.HasExited){$records += [pscustomobject]@{Scenario=$a.Scenario;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$false;Timeout=$false}; $done=$true}
      elseif(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds){Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $records += [pscustomobject]@{Scenario=$a.Scenario;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$true;Reused=$false;InfraFail=$false;Timeout=$false}; $done=$true}
    } elseif($a.Process.HasExited){$records += [pscustomobject]@{Scenario=$a.Scenario;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$true;Timeout=$false}; $done=$true}
    elseif(((Get-Date)-$a.Start).TotalSeconds -gt $TimeoutSeconds){Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $records += [pscustomobject]@{Scenario=$a.Scenario;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$true;Timeout=$true}; $done=$true}
    if(-not $done){$keep += $a}
  }
  $active=$keep; if($queue.Count -gt 0 -or $active.Count -gt 0){Start-Sleep -Milliseconds 500}
}
$records | Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $out 'process_lifecycle.csv')
$records | Format-Table -AutoSize | Out-String -Width 220 | Set-Content -Encoding UTF8 (Join-Path $out 'process_lifecycle_summary.txt')
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/wind'); R=airdropx_wind_finalize_v110(string('$outEsc'),string('$manifestEsc')); disp(R);"
& $MatlabExe -batch $cmd 2>&1 | Tee-Object -FilePath (Join-Path $out 'finalize_terminal.txt')
if($LASTEXITCODE -ne 0){throw 'Wind validation finalizer failed.'}
Get-Content (Join-Path $out 'wind_validation_summary.txt')
