param(
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [ValidateRange(1,3)][int]$MaxParallel=3,
  [int]$BuildTimeoutSeconds=1200,
  [int]$MissionTimeoutSeconds=300,
  [int]$ShutdownGraceSeconds=5,
  [switch]$ForceBank,
  [switch]$ForceMissions
)
$ErrorActionPreference='Stop'
function Escape-MatlabString([string]$s){return $s.Replace("'","''")}
if(-not (Test-Path $MatlabExe)){throw "MATLAB not found: $MatlabExe"}
$phys=Join-Path $ProjectRoot 'matlab\phys_mpc'; $mex=Join-Path $phys 'airdropx_jsbsim_oracle_mex.mexw64'; $baseBank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v033\physics_bank.mat'
$required=@($phys,$mex,$baseBank,(Join-Path $phys 'airdropx_phys_build_vertex_diagnostic_v081.m'),(Join-Path $phys 'airdropx_phys_mpc_get_vertex.m'),(Join-Path $phys 'airdropx_phys_preview_four_drop_closed_loop.m'))
foreach($x in $required){if(-not (Test-Path $x)){throw "Prerequisite missing: $x"}}
$bankRoot=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v081_diagnostic_envelope_bank'; $sliceRoot=Join-Path $bankRoot 'slices'; $envelopeBank=Join-Path $bankRoot 'physics_full_envelope_bank_diagnostic.mat'; New-Item -ItemType Directory -Force -Path $sliceRoot | Out-Null
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v081_diagnostic_envelope_validation'; New-Item -ItemType Directory -Force -Path $out | Out-Null
$oldBankRoot=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v080_full_envelope_bank'; $oldSliceRoot=Join-Path $oldBankRoot 'slices'
$rootEsc=Escape-MatlabString $ProjectRoot; $baseBankEsc=Escape-MatlabString $baseBank; $sliceRootEsc=Escape-MatlabString $sliceRoot; $envelopeBankEsc=Escape-MatlabString $envelopeBank

Write-Host '=== Physics-MPC v0.8.1 DIAGNOSTIC FULL-FLOW ENVELOPE ==='
Write-Host 'IMPORTANT: certification failures remain FAIL. This run only prevents early-stop so closed-loop effects can be observed.'

# Stage A: attempt every non-V50 vertex. A per-vertex certification failure is recorded and the scan continues.
$speedsBuild=@(45,55,60,65); $buildQueue=New-Object System.Collections.Queue; foreach($v in $speedsBuild){$buildQueue.Enqueue([double]$v)}; $active=@(); $buildRecords=@(); $buildFailures=@()
while($buildQueue.Count -gt 0 -or $active.Count -gt 0){
  while($buildQueue.Count -gt 0 -and $active.Count -lt $MaxParallel){
    $v=[double]$buildQueue.Dequeue(); $tag=('V{0:D3}' -f [int]$v); $dir=Join-Path $sliceRoot $tag; New-Item -ItemType Directory -Force -Path $dir | Out-Null; $marker=Join-Path $dir 'slice_complete.ok'; $mat=Join-Path $dir 'physics_speed_slice.mat'
    if((-not $ForceBank) -and (Test-Path $marker) -and (Test-Path $mat)){Write-Host "[$tag] reusing complete v0.8.1 diagnostic slice"; $buildRecords += [pscustomobject]@{Tag=$tag;PID=0;Elapsed_s=0;ForcedExitAfterResult=$false;Reused=$true;Source='v081';InfraFail=$false;Timeout=$false}; continue}
    # Reuse a fully completed strict v0.8.0 slice if available. Incomplete V65 is deliberately not reused.
    $oldDir=Join-Path $oldSliceRoot $tag; $oldMarker=Join-Path $oldDir 'slice_complete.ok'; $oldMat=Join-Path $oldDir 'physics_speed_slice.mat'
    if((-not $ForceBank) -and (Test-Path $oldMarker) -and (Test-Path $oldMat)){
      Write-Host "[$tag] importing previously completed strict v0.8.0 slice"
      Get-ChildItem $oldDir -File | ForEach-Object {Copy-Item -Force $_.FullName (Join-Path $dir $_.Name)}
      $buildRecords += [pscustomobject]@{Tag=$tag;PID=0;Elapsed_s=0;ForcedExitAfterResult=$false;Reused=$true;Source='v080';InfraFail=$false;Timeout=$false}; continue
    }
    Remove-Item $marker -Force -ErrorAction SilentlyContinue; $stdout=Join-Path $dir ($tag+'_stdout.txt'); $stderr=Join-Path $dir ($tag+'_stderr.txt'); Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue; $dirEsc=Escape-MatlabString $dir
    $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_build_speed_slice_v081(string(pwd),OutputRoot=string('$dirEsc'),Speed_mps=$v,BaseBankPath=string('$baseBankEsc')); disp(r);"
    $p=Start-Process -FilePath $MatlabExe -ArgumentList "-batch `"$cmd`"" -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
    $active += [pscustomobject]@{V=$v;Tag=$tag;Process=$p;PID=$p.Id;Start=Get-Date;Dir=$dir;Marker=$marker;Stdout=$stdout;Stderr=$stderr;Status=Join-Path $dir 'slice_status.txt';LastStatus='';MarkerAt=$null}; Write-Host "[$tag] diagnostic 95-vertex scan started PID $($p.Id)"
  }
  $keep=@(); foreach($a in $active){$a.Process.Refresh(); if(Test-Path $a.Status){$line=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue; if($line -and $line -ne $a.LastStatus){Write-Host "[$($a.Tag)] $line"; $a.LastStatus=$line}}; $done=$false
    if(Test-Path $a.Marker){if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}; if($a.Process.HasExited){$buildRecords += [pscustomobject]@{Tag=$a.Tag;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;Source='v081';InfraFail=$false;Timeout=$false}; $done=$true} elseif(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds){Write-Warning "[$($a.Tag)] scan complete but teardown did not exit; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $buildRecords += [pscustomobject]@{Tag=$a.Tag;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$true;Reused=$false;Source='v081';InfraFail=$false;Timeout=$false}; $done=$true}}
    elseif($a.Process.HasExited){Write-Warning "[$($a.Tag)] process exited before all 95 rows were attempted; this is infrastructure failure, not a certification FAIL."; $buildRecords += [pscustomobject]@{Tag=$a.Tag;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;Source='v081';InfraFail=$true;Timeout=$false}; $buildFailures += $a.Tag; $done=$true}
    elseif(((Get-Date)-$a.Start).TotalSeconds -gt $BuildTimeoutSeconds){Write-Warning "[$($a.Tag)] diagnostic slice timeout; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $buildRecords += [pscustomobject]@{Tag=$a.Tag;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;Source='v081';InfraFail=$true;Timeout=$true}; $buildFailures += $a.Tag; $done=$true}
    if(-not $done){$keep += $a}
  }; $active=$keep; if($buildQueue.Count -gt 0 -or $active.Count -gt 0){Start-Sleep -Milliseconds 500}
}
$buildRecords | Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $bankRoot 'build_process_lifecycle.csv')
if($buildFailures.Count -gt 0){$buildFailures | Set-Content -Encoding UTF8 (Join-Path $bankRoot 'build_infrastructure_failures.txt'); throw "Infrastructure failure prevented complete 95-row scan for: $($buildFailures -join ', '). Certification failures alone do not stop this runner."}

# Stage B: merge all attempted rows. Formal certification may be false; merge still produces a diagnostic bank.
$mergeCmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_merge_envelope_bank_v081(string(pwd),SliceRoot=string('$sliceRootEsc'),OutputBankPath=string('$envelopeBankEsc'),BaseBankPath=string('$baseBankEsc')); disp(r);"
$mergeLog=Join-Path $bankRoot 'merge_terminal.txt'; Write-Host '[bank] merging all 475 attempted rows into diagnostic bank...'; & $MatlabExe -batch $mergeCmd 2>&1 | Tee-Object -FilePath $mergeLog; if($LASTEXITCODE -ne 0 -or -not (Test-Path $envelopeBank)){throw "Diagnostic bank merge failed structurally. See $mergeLog"}

# Stage C: attempt all 95 nonlinear missions. Uncertified-but-usable vertices are explicitly allowed for diagnosis.
# If a H/V has a hard-unusable cfg, the entry writes MODEL_UNAVAILABLE evidence instead of fabricating a substitute model.
$heights=@(); for($h=20;$h -le 200;$h+=10){$heights += [double]$h}; $speeds=@(45,50,55,60,65); $queue=New-Object System.Collections.Queue; foreach($v in $speeds){foreach($h in $heights){$queue.Enqueue([pscustomobject]@{H=[double]$h;V=[double]$v})}}; $active=@(); $records=@()
while($queue.Count -gt 0 -or $active.Count -gt 0){
  while($queue.Count -gt 0 -and $active.Count -lt $MaxParallel){
    $j=$queue.Dequeue(); $tag=('H{0:D3}_V{1:D3}' -f [int]$j.H,[int]$j.V); $dir=Join-Path $out $tag; New-Item -ItemType Directory -Force -Path $dir | Out-Null; $marker=Join-Path $dir 'scenario_complete.ok'; $mat=Join-Path $dir 'preview_mission.mat'; $unavail=Join-Path $dir 'preview_mission_unavailable.mat'
    if((-not $ForceMissions) -and (Test-Path $marker) -and ((Test-Path $mat) -or (Test-Path $unavail))){Write-Host "[$tag] reusing complete diagnostic case"; $records += [pscustomobject]@{H_m=$j.H;V_mps=$j.V;PID=0;Elapsed_s=0;ForcedExitAfterResult=$false;Reused=$true;InfraFail=$false;Timeout=$false}; continue}
    Remove-Item $marker -Force -ErrorAction SilentlyContinue; $stdout=Join-Path $dir ($tag+'_stdout.txt'); $stderr=Join-Path $dir ($tag+'_stderr.txt'); Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue; $dirEsc=Escape-MatlabString $dir
    $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_preview_envelope_entry_v081(string(pwd),OutputRoot=string('$dirEsc'),Height_m=$($j.H),Speed_mps=$($j.V),BankPath=string('$envelopeBankEsc')); disp(r);"
    $p=Start-Process -FilePath $MatlabExe -ArgumentList "-batch `"$cmd`"" -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
    $active += [pscustomobject]@{H=$j.H;V=$j.V;Tag=$tag;Process=$p;PID=$p.Id;Start=Get-Date;Dir=$dir;Marker=$marker;Stdout=$stdout;Stderr=$stderr;Status=Join-Path $dir 'scenario_status.txt';LastStatus='';MarkerAt=$null}; Write-Host "[$tag] diagnostic mission started PID $($p.Id)"
  }
  $keep=@(); foreach($a in $active){$a.Process.Refresh(); if(Test-Path $a.Status){$line=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue; if($line -and $line -ne $a.LastStatus){Write-Host "[$($a.Tag)] $line"; $a.LastStatus=$line}}; $done=$false
    if(Test-Path $a.Marker){if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}; if($a.Process.HasExited){$records += [pscustomobject]@{H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$false;Timeout=$false}; $done=$true} elseif(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds){Write-Warning "[$($a.Tag)] evidence complete but teardown did not exit; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $records += [pscustomobject]@{H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$true;Reused=$false;InfraFail=$false;Timeout=$false}; $done=$true}}
    elseif($a.Process.HasExited){Write-Warning "[$($a.Tag)] MATLAB exited before diagnostic completion marker."; $records += [pscustomobject]@{H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$true;Timeout=$false}; $done=$true}
    elseif(((Get-Date)-$a.Start).TotalSeconds -gt $MissionTimeoutSeconds){Write-Warning "[$($a.Tag)] mission timeout; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $records += [pscustomobject]@{H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$true;Timeout=$true}; $done=$true}
    if(-not $done){$keep += $a}
  }; $active=$keep; if($queue.Count -gt 0 -or $active.Count -gt 0){Start-Sleep -Milliseconds 500}
}
$records | Sort-Object V_mps,H_m | Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $out 'process_lifecycle.csv'); $records | Sort-Object V_mps,H_m | Format-Table -AutoSize | Out-String -Width 280 | Set-Content -Encoding UTF8 (Join-Path $out 'process_lifecycle_summary.txt')

# Stage D: always aggregate. A nonzero formal FAIL is reported, not thrown away.
$outEsc=Escape-MatlabString $out; $cmdF="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_finalize_envelope_v081(string(pwd),OutputRoot=string('$outEsc'),BankPath=string('$envelopeBankEsc')); disp(r);"; $finalLog=Join-Path $out 'finalize_terminal.txt'; Write-Host '[finalize] aggregating all 95 diagnostic HxV cases...'; & $MatlabExe -batch $cmdF 2>&1 | Tee-Object -FilePath $finalLog; if($LASTEXITCODE -ne 0){throw "Diagnostic envelope finalizer failed structurally. See $finalLog"}
$summary=Join-Path $out 'envelope_validation_summary.txt'; if(-not (Test-Path $summary)){throw "Envelope summary missing: $summary"}
Write-Host '=== Physics-MPC v0.8.1 DIAGNOSTIC FULL FLOW COMPLETE ==='; Write-Host "Summary: $summary"; Write-Host "Bank: $envelopeBank"; Get-Content $summary
Write-Warning 'Do not interpret diagnostic_mission_performance_pass as formal certification when physics_certification_pass=0. Certification failures remain explicitly recorded.'
