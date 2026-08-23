param(
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [int]$MaxParallel=3,
  [int]$MissionTimeoutSeconds=180,
  [int]$ShutdownGraceSeconds=5,
  [switch]$ForceDense,
  [switch]$ForceMissions
)
$ErrorActionPreference='Stop'; if($MaxParallel -lt 1 -or $MaxParallel -gt 3){throw 'MaxParallel must be 1..3'}
function Escape-MatlabString([string]$s){return $s.Replace("'","''")}
if(-not (Test-Path $MatlabExe)){throw "MATLAB not found: $MatlabExe"}
$master=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v082_fixed_horizon_envelope_bank\physics_full_envelope_bank_diagnostic.mat'; if(-not (Test-Path $master)){throw "v0.8.2 master bank missing: $master"}
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v090_continuous_interval_validation'; $dense=Join-Path $out 'dense_model_audit'; New-Item -ItemType Directory -Force -Path $out,$dense | Out-Null
$gridEvidence=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v082_fixed_horizon_envelope_validation'
$rootEsc=Escape-MatlabString $ProjectRoot; $masterEsc=Escape-MatlabString $master; $outEsc=Escape-MatlabString $out; $denseEsc=Escape-MatlabString $dense; $gridEsc=Escape-MatlabString $gridEvidence
# Stage A: dense 1 m x 1 m computational interpolation audit.
$denseMarker=Join-Path $dense 'dense_audit_complete.ok'
if($ForceDense -or -not (Test-Path $denseMarker)){
  Write-Host '[dense] auditing H=20:1:200 x V=45:1:65 x cfg0..4 ...'
  $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_interval_dense_audit_v090(string(pwd),MasterBankPath=string('$masterEsc'),OutputRoot=string('$denseEsc')); disp(r);"
  & $MatlabExe -batch $cmd 2>&1 | Tee-Object -FilePath (Join-Path $dense 'dense_audit_terminal.txt'); if($LASTEXITCODE -ne 0){throw 'Dense interval audit failed structurally.'}
}else{Write-Host '[dense] reusing completed dense audit'}
# Stage B: deterministic two-point coverage of every one of the 72 interpolation cells.
$manifest=Join-Path $out 'interval_case_manifest.csv'; $manifestEsc=Escape-MatlabString $manifest
$cmdM="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); T=airdropx_phys_interval_case_manifest_v090(string('$manifestEsc')); disp(T(1:min(8,height(T)),:)); fprintf('cases=%d\n',height(T));"; & $MatlabExe -batch $cmdM 2>&1 | Tee-Object -FilePath (Join-Path $out 'manifest_terminal.txt'); if($LASTEXITCODE -ne 0 -or -not (Test-Path $manifest)){throw 'Interval case manifest generation failed.'}
$jobs=Import-Csv $manifest; if($jobs.Count -ne 144){throw "Expected 144 off-grid cases, got $($jobs.Count)"}; $queue=New-Object System.Collections.Queue; foreach($j in $jobs){$queue.Enqueue($j)}; $active=@(); $records=@()
while($queue.Count -gt 0 -or $active.Count -gt 0){
  while($queue.Count -gt 0 -and $active.Count -lt $MaxParallel){
    $j=$queue.Dequeue(); $tag=[string]$j.CaseId; $H=[double]$j.H_m; $V=[double]$j.V_mps; $dir=Join-Path $out $tag; New-Item -ItemType Directory -Force -Path $dir | Out-Null; $marker=Join-Path $dir 'scenario_complete.ok'; $mat=Join-Path $dir 'preview_mission.mat'
    if((-not $ForceMissions) -and (Test-Path $marker) -and (Test-Path $mat)){Write-Host "[$tag] reusing complete result"; $records += [pscustomobject]@{CaseId=$tag;H_m=$H;V_mps=$V;PID=0;Elapsed_s=0;ForcedExitAfterResult=$false;Reused=$true;InfraFail=$false;Timeout=$false}; continue}
    Remove-Item $marker -Force -ErrorAction SilentlyContinue; $stdout=Join-Path $dir ($tag+'_stdout.txt'); $stderr=Join-Path $dir ($tag+'_stderr.txt'); Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue; $dirEsc=Escape-MatlabString $dir
    $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_interval_entry_v090(string(pwd),OutputRoot=string('$dirEsc'),Height_m=$H,Speed_mps=$V,MasterBankPath=string('$masterEsc')); disp(r);"
    $p=Start-Process -FilePath $MatlabExe -ArgumentList "-batch `"$cmd`"" -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
    $active += [pscustomobject]@{CaseId=$tag;H=$H;V=$V;Process=$p;PID=$p.Id;Start=Get-Date;Dir=$dir;Marker=$marker;Status=Join-Path $dir 'scenario_status.txt';LastStatus='';MarkerAt=$null}; Write-Host "[$tag] started PID $($p.Id) H=$H V=$V"
  }
  $keep=@(); foreach($a in $active){$a.Process.Refresh(); if(Test-Path $a.Status){$line=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue; if($line -and $line -ne $a.LastStatus){Write-Host "[$($a.CaseId)] $line"; $a.LastStatus=$line}}; $done=$false
    if(Test-Path $a.Marker){if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}; if($a.Process.HasExited){$records += [pscustomobject]@{CaseId=$a.CaseId;H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$false;Timeout=$false}; $done=$true} elseif(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds){Write-Warning "[$($a.CaseId)] evidence complete but teardown did not exit; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $records += [pscustomobject]@{CaseId=$a.CaseId;H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$true;Reused=$false;InfraFail=$false;Timeout=$false}; $done=$true}}
    elseif($a.Process.HasExited){Write-Warning "[$($a.CaseId)] MATLAB exited before completion marker."; $records += [pscustomobject]@{CaseId=$a.CaseId;H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$true;Timeout=$false}; $done=$true}
    elseif(((Get-Date)-$a.Start).TotalSeconds -gt $MissionTimeoutSeconds){Write-Warning "[$($a.CaseId)] timeout; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $records += [pscustomobject]@{CaseId=$a.CaseId;H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$true;Timeout=$true}; $done=$true}
    if(-not $done){$keep += $a}
  }; $active=$keep; if($queue.Count -gt 0 -or $active.Count -gt 0){Start-Sleep -Milliseconds 500}
}
$records | Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $out 'process_lifecycle.csv'); $records | Format-Table -AutoSize | Out-String -Width 300 | Set-Content -Encoding UTF8 (Join-Path $out 'process_lifecycle_summary.txt')
# Stage C: aggregate dense + off-grid + existing 95 on-grid reference evidence.
$cmdF="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_finalize_interval_v090(string(pwd),OutputRoot=string('$outEsc'),ManifestPath=string('$manifestEsc'),DenseAuditRoot=string('$denseEsc'),GridEvidenceRoot=string('$gridEsc')); disp(r);"; $finalLog=Join-Path $out 'finalize_terminal.txt'; & $MatlabExe -batch $cmdF 2>&1 | Tee-Object -FilePath $finalLog; if($LASTEXITCODE -ne 0){throw "Interval finalizer failed structurally. See $finalLog"}
$summary=Join-Path $out 'interval_validation_summary.txt'; if(-not (Test-Path $summary)){throw "Missing interval summary: $summary"}; Write-Host '=== Physics-MPC v0.9.0 CONTINUOUS HxV INTERVAL FLOW COMPLETE ==='; Get-Content $summary
