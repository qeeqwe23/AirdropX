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
$required=@($phys,$mex,$baseBank,(Join-Path $phys 'airdropx_phys_build_vertex.m'),(Join-Path $phys 'airdropx_phys_mpc_get_vertex.m'),(Join-Path $phys 'airdropx_phys_preview_four_drop_closed_loop.m'))
foreach($x in $required){if(-not (Test-Path $x)){throw "Prerequisite missing: $x"}}
$bankRoot=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v080_full_envelope_bank'; $sliceRoot=Join-Path $bankRoot 'slices'; $envelopeBank=Join-Path $bankRoot 'physics_full_envelope_bank.mat'; New-Item -ItemType Directory -Force -Path $sliceRoot | Out-Null
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v080_full_envelope_validation'; New-Item -ItemType Directory -Force -Path $out | Out-Null
$rootEsc=Escape-MatlabString $ProjectRoot; $baseBankEsc=Escape-MatlabString $baseBank; $sliceRootEsc=Escape-MatlabString $sliceRoot; $envelopeBankEsc=Escape-MatlabString $envelopeBank

# Stage A: build four complete non-V50 speed slices. Each slice is 19 heights x 5 cfg = 95 vertices.
$speedsBuild=@(45,55,60,65); $buildQueue=New-Object System.Collections.Queue; foreach($v in $speedsBuild){$buildQueue.Enqueue([double]$v)}; $active=@(); $buildRecords=@(); $buildFailures=@()
while($buildQueue.Count -gt 0 -or $active.Count -gt 0){
  while($buildQueue.Count -gt 0 -and $active.Count -lt $MaxParallel){
    $v=[double]$buildQueue.Dequeue(); $tag=('V{0:D3}' -f [int]$v); $dir=Join-Path $sliceRoot $tag; New-Item -ItemType Directory -Force -Path $dir | Out-Null; $marker=Join-Path $dir 'slice_complete.ok'; $mat=Join-Path $dir 'physics_speed_slice.mat'
    if((-not $ForceBank) -and (Test-Path $marker) -and (Test-Path $mat)){Write-Host "[$tag] reusing complete 95-vertex speed slice"; $buildRecords += [pscustomobject]@{Tag=$tag;PID=0;Elapsed_s=0;ForcedExitAfterResult=$false;Reused=$true;InfraFail=$false;Timeout=$false}; continue}
    Remove-Item $marker -Force -ErrorAction SilentlyContinue; $stdout=Join-Path $dir ($tag+'_stdout.txt'); $stderr=Join-Path $dir ($tag+'_stderr.txt'); Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue; $dirEsc=Escape-MatlabString $dir
    $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_build_speed_slice_v080(string(pwd),OutputRoot=string('$dirEsc'),Speed_mps=$v,BaseBankPath=string('$baseBankEsc')); disp(r.pass);"
    $p=Start-Process -FilePath $MatlabExe -ArgumentList "-batch `"$cmd`"" -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
    $active += [pscustomobject]@{V=$v;Tag=$tag;Process=$p;PID=$p.Id;Start=Get-Date;Dir=$dir;Marker=$marker;Stdout=$stdout;Stderr=$stderr;Status=Join-Path $dir 'slice_status.txt';LastStatus='';MarkerAt=$null}; Write-Host "[$tag] full-slice build started PID $($p.Id)"
  }
  $keep=@(); foreach($a in $active){$a.Process.Refresh(); if(Test-Path $a.Status){$line=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue; if($line -and $line -ne $a.LastStatus){Write-Host "[$($a.Tag)] $line"; $a.LastStatus=$line}}; $done=$false
    if(Test-Path $a.Marker){if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}; if($a.Process.HasExited){$buildRecords += [pscustomobject]@{Tag=$a.Tag;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$false;Timeout=$false}; $done=$true} elseif(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds){Write-Warning "[$($a.Tag)] slice complete but teardown did not exit; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $buildRecords += [pscustomobject]@{Tag=$a.Tag;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$true;Reused=$false;InfraFail=$false;Timeout=$false}; $done=$true}}
    elseif($a.Process.HasExited){Write-Warning "[$($a.Tag)] speed slice exited before marker; see stdout/stderr."; $buildRecords += [pscustomobject]@{Tag=$a.Tag;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$true;Timeout=$false}; $buildFailures += $a.Tag; $done=$true}
    elseif(((Get-Date)-$a.Start).TotalSeconds -gt $BuildTimeoutSeconds){Write-Warning "[$($a.Tag)] speed slice timeout; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $buildRecords += [pscustomobject]@{Tag=$a.Tag;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$true;Timeout=$true}; $buildFailures += $a.Tag; $done=$true}
    if(-not $done){$keep += $a}
  }; $active=$keep; if($buildQueue.Count -gt 0 -or $active.Count -gt 0){Start-Sleep -Milliseconds 500}
}
$buildRecords | Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $bankRoot 'build_process_lifecycle.csv')
if($buildFailures.Count -gt 0){$buildFailures | Set-Content -Encoding UTF8 (Join-Path $bankRoot 'build_failures.txt'); throw "Full-envelope physics slice build failed for: $($buildFailures -join ', '). Other completed slices remain resumable."}

# Merge V45/V55/V60/V65 full slices + certified V50 95-point bank into 475 exact vertices.
$bankRootEsc=Escape-MatlabString $bankRoot
$mergeCmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_merge_envelope_bank_v080(string(pwd),SliceRoot=string('$sliceRootEsc'),OutputBankPath=string('$envelopeBankEsc'),BaseBankPath=string('$baseBankEsc')); disp(r.pass);"
$mergeLog=Join-Path $bankRoot 'merge_terminal.txt'; Write-Host '[bank] merging 475-point full HxV envelope bank...'; & $MatlabExe -batch $mergeCmd 2>&1 | Tee-Object -FilePath $mergeLog; if($LASTEXITCODE -ne 0 -or -not (Test-Path $envelopeBank)){throw "Full envelope bank merge failed. See $mergeLog"}

# Stage B: 95 nonlinear PreviewOnly missions = 19 heights x 5 speeds.
$heights=@(); for($h=20;$h -le 200;$h+=10){$heights += [double]$h}; $speeds=@(45,50,55,60,65); $queue=New-Object System.Collections.Queue; foreach($v in $speeds){foreach($h in $heights){$queue.Enqueue([pscustomobject]@{H=[double]$h;V=[double]$v})}}; $active=@(); $records=@()
while($queue.Count -gt 0 -or $active.Count -gt 0){
  while($queue.Count -gt 0 -and $active.Count -lt $MaxParallel){
    $j=$queue.Dequeue(); $tag=('H{0:D3}_V{1:D3}' -f [int]$j.H,[int]$j.V); $dir=Join-Path $out $tag; New-Item -ItemType Directory -Force -Path $dir | Out-Null; $marker=Join-Path $dir 'scenario_complete.ok'; $mat=Join-Path $dir 'preview_mission.mat'
    if((-not $ForceMissions) -and (Test-Path $marker) -and (Test-Path $mat)){Write-Host "[$tag] reusing complete envelope mission"; $records += [pscustomobject]@{H_m=$j.H;V_mps=$j.V;PID=0;Elapsed_s=0;ForcedExitAfterResult=$false;Reused=$true;InfraFail=$false;Timeout=$false}; continue}
    Remove-Item $marker -Force -ErrorAction SilentlyContinue; $stdout=Join-Path $dir ($tag+'_stdout.txt'); $stderr=Join-Path $dir ($tag+'_stderr.txt'); Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue; $dirEsc=Escape-MatlabString $dir
    $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_preview_envelope_entry_v080(string(pwd),OutputRoot=string('$dirEsc'),Height_m=$($j.H),Speed_mps=$($j.V),BankPath=string('$envelopeBankEsc')); disp(r.pass);"
    $p=Start-Process -FilePath $MatlabExe -ArgumentList "-batch `"$cmd`"" -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
    $active += [pscustomobject]@{H=$j.H;V=$j.V;Tag=$tag;Process=$p;PID=$p.Id;Start=Get-Date;Dir=$dir;Marker=$marker;Stdout=$stdout;Stderr=$stderr;Status=Join-Path $dir 'scenario_status.txt';LastStatus='';MarkerAt=$null}; Write-Host "[$tag] mission started PID $($p.Id)"
  }
  $keep=@(); foreach($a in $active){$a.Process.Refresh(); if(Test-Path $a.Status){$line=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue; if($line -and $line -ne $a.LastStatus){Write-Host "[$($a.Tag)] $line"; $a.LastStatus=$line}}; $done=$false
    if(Test-Path $a.Marker){if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}; if($a.Process.HasExited){$records += [pscustomobject]@{H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$false;Timeout=$false}; $done=$true} elseif(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds){Write-Warning "[$($a.Tag)] result complete but teardown did not exit; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $records += [pscustomobject]@{H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$true;Reused=$false;InfraFail=$false;Timeout=$false}; $done=$true}}
    elseif($a.Process.HasExited){Write-Warning "[$($a.Tag)] MATLAB exited before completion marker."; $records += [pscustomobject]@{H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$true;Timeout=$false}; $done=$true}
    elseif(((Get-Date)-$a.Start).TotalSeconds -gt $MissionTimeoutSeconds){Write-Warning "[$($a.Tag)] mission timeout; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $records += [pscustomobject]@{H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false;InfraFail=$true;Timeout=$true}; $done=$true}
    if(-not $done){$keep += $a}
  }; $active=$keep; if($queue.Count -gt 0 -or $active.Count -gt 0){Start-Sleep -Milliseconds 500}
}
$records | Sort-Object V_mps,H_m | Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $out 'process_lifecycle.csv'); $records | Sort-Object V_mps,H_m | Format-Table -AutoSize | Out-String -Width 280 | Set-Content -Encoding UTF8 (Join-Path $out 'process_lifecycle_summary.txt')

# Aggregate. Do not stop early on mission-performance FAIL: all 95 cases are collected first.
$outEsc=Escape-MatlabString $out; $cmdF="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_finalize_envelope_v080(string(pwd),OutputRoot=string('$outEsc'),BankPath=string('$envelopeBankEsc')); disp(r.pass);"; $finalLog=Join-Path $out 'finalize_terminal.txt'; Write-Host '[finalize] aggregating 95 full-envelope missions...'; & $MatlabExe -batch $cmdF 2>&1 | Tee-Object -FilePath $finalLog; if($LASTEXITCODE -ne 0){throw "Full envelope finalizer failed. See $finalLog"}
$summary=Join-Path $out 'envelope_validation_summary.txt'; if(-not (Test-Path $summary)){throw "Envelope summary missing: $summary"}; $passLine=Get-Content $summary | Where-Object {$_ -match '^pass='} | Select-Object -First 1; if($passLine -ne 'pass=1'){throw "Physics-MPC v0.8.0 full HxV envelope contains FAIL/missing cases. All completed evidence remains in $out. See $summary"}
Write-Host '=== Physics-MPC v0.8.0 FULL HxV ENVELOPE VALIDATION PASS ==='; Write-Host "Summary: $summary"; Write-Host "Bank: $envelopeBank"; Write-Host 'Formal discrete certification grid: H=20:10:200 m x V=45:5:65 m/s, cfg0:4, PreviewOnly 0.2 s four-drop.'
