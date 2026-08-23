param(
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [int]$MaxParallel=3,
  [int]$MissionTimeoutSeconds=480,
  [int]$ShutdownGraceSeconds=5,
  [switch]$Force
)
$ErrorActionPreference='Stop'; if($MaxParallel -lt 1 -or $MaxParallel -gt 3){throw 'MaxParallel must be 1..3'}
function Escape-MatlabString([string]$s){return $s.Replace("'","''")}
if(-not (Test-Path $MatlabExe)){throw "MATLAB not found: $MatlabExe"}
$master=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v082_fixed_horizon_envelope_bank\physics_full_envelope_bank_diagnostic.mat'; if(-not (Test-Path $master)){throw "Master bank missing: $master"}
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v100_runtime_command_validation'; New-Item -ItemType Directory -Force -Path $out | Out-Null
$interval=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v090_continuous_interval_validation'
$rootEsc=Escape-MatlabString $ProjectRoot; $masterEsc=Escape-MatlabString $master; $outEsc=Escape-MatlabString $out; $intervalEsc=Escape-MatlabString $interval
$manifest=Join-Path $out 'runtime_command_manifest.csv'; $manifestEsc=Escape-MatlabString $manifest
$cmdM="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); T=airdropx_phys_runtime_scenario_manifest_v100(string('$manifestEsc')); disp(T);"; & $MatlabExe -batch $cmdM 2>&1 | Tee-Object -FilePath (Join-Path $out 'manifest_terminal.txt'); if($LASTEXITCODE -ne 0 -or -not (Test-Path $manifest)){throw 'Runtime scenario manifest failed.'}
$jobs=Import-Csv $manifest; if($jobs.Count -ne 6){throw "Expected six runtime scenarios, got $($jobs.Count)"}; $queue=New-Object System.Collections.Queue; foreach($j in $jobs){$queue.Enqueue($j)}; $active=@(); $records=@()
while($queue.Count -gt 0 -or $active.Count -gt 0){
  while($queue.Count -gt 0 -and $active.Count -lt $MaxParallel){
    $j=$queue.Dequeue(); $tag=[string]$j.Scenario; $duration=[double]$j.Duration_s; $mode=[string]$j.PreviewMode; $dir=Join-Path $out $tag; New-Item -ItemType Directory -Force -Path $dir | Out-Null; $marker=Join-Path $dir 'scenario_complete.ok'; $mat=Join-Path $dir 'runtime_command_mission.mat'
    if((-not $Force) -and (Test-Path $marker) -and (Test-Path $mat)){Write-Host "[$tag] reusing complete result"; $records += [pscustomobject]@{Scenario=$tag;PID=0;Elapsed_s=0;ForcedExitAfterResult=$false;Reused=$true;InfraFail=$false;Timeout=$false}; continue}
    Remove-Item $marker -Force -ErrorAction SilentlyContinue; $stdout=Join-Path $dir ($tag+'_stdout.txt'); $stderr=Join-Path $dir ($tag+'_stderr.txt'); Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue; $dirEsc=Escape-MatlabString $dir
    $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_runtime_entry_v100(string(pwd),OutputRoot=string('$dirEsc'),ScenarioName=string('$tag'),MasterBankPath=string('$masterEsc'),Duration_s=$duration,CommandPreviewMode=string('$mode')); disp(r);"
    $p=Start-Process -FilePath $MatlabExe -ArgumentList "-batch `"$cmd`"" -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru; $active += [pscustomobject]@{Scenario=$tag;Process=$p;PID=$p.Id;Start=Get-Date;Marker=$marker;Status=Join-Path $dir 'scenario_status.txt';LastStatus='';MarkerAt=$null}; Write-Host "[$tag] started PID $($p.Id)"
  }
  $keep=@(); foreach($a in $active){$a.Process.Refresh(); if(Test-Path $a.Status){$line=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue; if($line -and $line -ne $a.LastStatus){Write-Host "[$($a.Scenario)] $line"; $a.LastStatus=$line}}; $done=$false
    if(Test-Path $a.Marker){if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}; if($a.Process.HasExited){$records += [pscustomobject]@{Scenario=$a.Scenario;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$false;Timeout=$false}; $done=$true} elseif(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds){Write-Warning "[$($a.Scenario)] evidence complete but teardown did not exit; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $records += [pscustomobject]@{Scenario=$a.Scenario;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$true;Reused=$false;InfraFail=$false;Timeout=$false}; $done=$true}}
    elseif($a.Process.HasExited){Write-Warning "[$($a.Scenario)] MATLAB exited before completion marker."; $records += [pscustomobject]@{Scenario=$a.Scenario;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$true;Timeout=$false}; $done=$true}
    elseif(((Get-Date)-$a.Start).TotalSeconds -gt $MissionTimeoutSeconds){Write-Warning "[$($a.Scenario)] timeout; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $records += [pscustomobject]@{Scenario=$a.Scenario;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$true;Timeout=$true}; $done=$true}
    if(-not $done){$keep += $a}
  }; $active=$keep; if($queue.Count -gt 0 -or $active.Count -gt 0){Start-Sleep -Milliseconds 500}
}
$records | Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $out 'process_lifecycle.csv'); $records | Format-Table -AutoSize | Out-String -Width 240 | Set-Content -Encoding UTF8 (Join-Path $out 'process_lifecycle_summary.txt')
$cmdF="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_finalize_runtime_v100(string(pwd),OutputRoot=string('$outEsc'),ManifestPath=string('$manifestEsc'),IntervalEvidenceRoot=string('$intervalEsc')); disp(r);"; $final=Join-Path $out 'finalize_terminal.txt'; & $MatlabExe -batch $cmdF 2>&1 | Tee-Object -FilePath $final; if($LASTEXITCODE -ne 0){throw "Runtime finalizer failed. See $final"}
$summary=Join-Path $out 'runtime_command_validation_summary.txt'; if(-not (Test-Path $summary)){throw 'Missing runtime summary.'}; Write-Host '=== Physics-MPC v1.0.0 RUNTIME COMMAND FLOW COMPLETE ==='; Get-Content $summary
