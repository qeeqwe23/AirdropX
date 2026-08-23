param(
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [ValidateRange(1,3)][int]$MaxParallel=3,
  [int]$BuildTimeoutSeconds=900,
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
$bankRoot=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v070_speed_pilot_bank'; $sliceRoot=Join-Path $bankRoot 'slices'; $pilotBank=Join-Path $bankRoot 'physics_speed_pilot_bank.mat'; New-Item -ItemType Directory -Force -Path $sliceRoot | Out-Null
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v070_speed_pilot_validation'; New-Item -ItemType Directory -Force -Path $out | Out-Null
$rootEsc=Escape-MatlabString $ProjectRoot; $baseBankEsc=Escape-MatlabString $baseBank; $sliceRootEsc=Escape-MatlabString $sliceRoot; $pilotBankEsc=Escape-MatlabString $pilotBank

function Run-IsolatedBatch([string]$Tag,[string]$Cmd,[string]$Dir,[string]$Marker,[int]$TimeoutSeconds,[string]$StatusName){
  New-Item -ItemType Directory -Force -Path $Dir | Out-Null; $stdout=Join-Path $Dir ($Tag+'_stdout.txt'); $stderr=Join-Path $Dir ($Tag+'_stderr.txt'); Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue
  $p=Start-Process -FilePath $MatlabExe -ArgumentList "-batch `"$Cmd`"" -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru; $start=Get-Date; $markerAt=$null; $last=''; Write-Host "[$Tag] started PID $($p.Id)"
  while($true){
    Start-Sleep -Milliseconds 500; $p.Refresh(); $status=Join-Path $Dir $StatusName; if(Test-Path $status){$line=Get-Content $status -Tail 1 -ErrorAction SilentlyContinue; if($line -and $line -ne $last){Write-Host "[$Tag] $line"; $last=$line}}
    if(Test-Path $Marker){if($null -eq $markerAt){$markerAt=Get-Date}; if($p.HasExited){return [pscustomobject]@{Tag=$Tag;PID=$p.Id;Elapsed_s=[math]::Round(((Get-Date)-$start).TotalSeconds,3);ForcedExitAfterResult=$false;Stdout=$stdout;Stderr=$stderr}}; if(((Get-Date)-$markerAt).TotalSeconds -ge $ShutdownGraceSeconds){Write-Warning "[$Tag] outputs complete but teardown did not exit; terminating only PID $($p.Id)."; Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue; return [pscustomobject]@{Tag=$Tag;PID=$p.Id;Elapsed_s=[math]::Round(((Get-Date)-$start).TotalSeconds,3);ForcedExitAfterResult=$true;Stdout=$stdout;Stderr=$stderr}}}
    elseif($p.HasExited){throw "[$Tag] MATLAB exited before completion marker. See $stdout and $stderr"}
    elseif(((Get-Date)-$start).TotalSeconds -gt $TimeoutSeconds){Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue; throw "[$Tag] timeout after $TimeoutSeconds s. See $stdout and $stderr"}
  }
}

# Stage A: build only the four new non-V50 speed slices. V50 is reused from the validated 95-point bank.
$speedsBuild=@(40,45,55,60); $buildQueue=New-Object System.Collections.Queue; foreach($v in $speedsBuild){$buildQueue.Enqueue([double]$v)}; $active=@(); $buildRecords=@()
while($buildQueue.Count -gt 0 -or $active.Count -gt 0){
  while($buildQueue.Count -gt 0 -and $active.Count -lt $MaxParallel){
    $v=[double]$buildQueue.Dequeue(); $tag=('V{0:D3}' -f [int]$v); $dir=Join-Path $sliceRoot $tag; New-Item -ItemType Directory -Force -Path $dir | Out-Null; $marker=Join-Path $dir 'slice_complete.ok'; $mat=Join-Path $dir 'physics_speed_slice.mat'
    if((-not $ForceBank) -and (Test-Path $marker) -and (Test-Path $mat)){Write-Host "[$tag] reusing complete speed slice"; $buildRecords += [pscustomobject]@{Tag=$tag;PID=0;Elapsed_s=0;ForcedExitAfterResult=$false;Reused=$true}; continue}
    Remove-Item $marker -Force -ErrorAction SilentlyContinue; $stdout=Join-Path $dir ($tag+'_stdout.txt'); $stderr=Join-Path $dir ($tag+'_stderr.txt'); Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue; $dirEsc=Escape-MatlabString $dir
    $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_build_speed_slice_v070(string(pwd),OutputRoot=string('$dirEsc'),Speed_mps=$v,BaseBankPath=string('$baseBankEsc')); disp(r.pass);"
    $p=Start-Process -FilePath $MatlabExe -ArgumentList "-batch `"$cmd`"" -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
    $active += [pscustomobject]@{V=$v;Tag=$tag;Process=$p;PID=$p.Id;Start=Get-Date;Dir=$dir;Marker=$marker;Stdout=$stdout;Stderr=$stderr;Status=Join-Path $dir 'slice_status.txt';LastStatus='';MarkerAt=$null}; Write-Host "[$tag] build started PID $($p.Id)"
  }
  $keep=@(); foreach($a in $active){$a.Process.Refresh(); if(Test-Path $a.Status){$line=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue; if($line -and $line -ne $a.LastStatus){Write-Host "[$($a.Tag)] $line"; $a.LastStatus=$line}}; $done=$false
    if(Test-Path $a.Marker){if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}; if($a.Process.HasExited){$buildRecords += [pscustomobject]@{Tag=$a.Tag;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false}; $done=$true} elseif(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds){Write-Warning "[$($a.Tag)] slice complete but teardown did not exit; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $buildRecords += [pscustomobject]@{Tag=$a.Tag;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$true;Reused=$false}; $done=$true}}
    elseif($a.Process.HasExited){throw "[$($a.Tag)] speed slice build exited before marker. See $($a.Stdout) and $($a.Stderr)"} elseif(((Get-Date)-$a.Start).TotalSeconds -gt $BuildTimeoutSeconds){Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; throw "[$($a.Tag)] speed slice build timeout."}
    if(-not $done){$keep += $a}
  }; $active=$keep; if($buildQueue.Count -gt 0 -or $active.Count -gt 0){Start-Sleep -Milliseconds 500}
}
$buildRecords | Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $bankRoot 'build_process_lifecycle.csv')

# Merge 4 slices + V50 into one exact 75-vertex pilot bank and run common-controller audit.
$bankRootEsc=Escape-MatlabString $bankRoot
$mergeCmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_merge_speed_pilot_bank_v070(string(pwd),SliceRoot=string('$sliceRootEsc'),OutputBankPath=string('$pilotBankEsc'),BaseBankPath=string('$baseBankEsc')); disp(r.pass);"
$mergeLog=Join-Path $bankRoot 'merge_terminal.txt'; Write-Host '[bank] merging 75-point speed pilot bank...'; & $MatlabExe -batch $mergeCmd 2>&1 | Tee-Object -FilePath $mergeLog; if($LASTEXITCODE -ne 0 -or -not (Test-Path $pilotBank)){throw "Speed pilot bank merge failed. See $mergeLog"}

# Stage B: 15 nonlinear PreviewOnly missions = 3 altitude anchors x 5 speeds.
$heights=@(20,110,200); $speeds=@(40,45,50,55,60); $queue=New-Object System.Collections.Queue; foreach($v in $speeds){foreach($h in $heights){$queue.Enqueue([pscustomobject]@{H=[double]$h;V=[double]$v})}}; $active=@(); $records=@()
while($queue.Count -gt 0 -or $active.Count -gt 0){
  while($queue.Count -gt 0 -and $active.Count -lt $MaxParallel){
    $j=$queue.Dequeue(); $tag=('H{0:D3}_V{1:D3}' -f [int]$j.H,[int]$j.V); $dir=Join-Path $out $tag; New-Item -ItemType Directory -Force -Path $dir | Out-Null; $marker=Join-Path $dir 'scenario_complete.ok'; $mat=Join-Path $dir 'preview_mission.mat'
    if((-not $ForceMissions) -and (Test-Path $marker) -and (Test-Path $mat)){Write-Host "[$tag] reusing complete mission"; $records += [pscustomobject]@{H_m=$j.H;V_mps=$j.V;PID=0;Elapsed_s=0;ForcedExitAfterResult=$false;Reused=$true}; continue}
    Remove-Item $marker -Force -ErrorAction SilentlyContinue; $stdout=Join-Path $dir ($tag+'_stdout.txt'); $stderr=Join-Path $dir ($tag+'_stderr.txt'); Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue; $dirEsc=Escape-MatlabString $dir
    $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_preview_speed_entry_v070(string(pwd),OutputRoot=string('$dirEsc'),Height_m=$($j.H),Speed_mps=$($j.V),BankPath=string('$pilotBankEsc')); disp(r.pass);"
    $p=Start-Process -FilePath $MatlabExe -ArgumentList "-batch `"$cmd`"" -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
    $active += [pscustomobject]@{H=$j.H;V=$j.V;Tag=$tag;Process=$p;PID=$p.Id;Start=Get-Date;Dir=$dir;Marker=$marker;Stdout=$stdout;Stderr=$stderr;Status=Join-Path $dir 'scenario_status.txt';LastStatus='';MarkerAt=$null}; Write-Host "[$tag] mission started PID $($p.Id)"
  }
  $keep=@(); foreach($a in $active){$a.Process.Refresh(); if(Test-Path $a.Status){$line=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue; if($line -and $line -ne $a.LastStatus){Write-Host "[$($a.Tag)] $line"; $a.LastStatus=$line}}; $done=$false
    if(Test-Path $a.Marker){if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}; if($a.Process.HasExited){$records += [pscustomobject]@{H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$false;Reused=$false}; $done=$true} elseif(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds){Write-Warning "[$($a.Tag)] result complete but teardown did not exit; terminating only PID $($a.PID)."; Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $records += [pscustomobject]@{H_m=$a.H;V_mps=$a.V;PID=$a.PID;Elapsed_s=[math]::Round(((Get-Date)-$a.Start).TotalSeconds,3);ForcedExitAfterResult=$true;Reused=$false}; $done=$true}}
    elseif($a.Process.HasExited){throw "[$($a.Tag)] MATLAB exited before completion marker. See $($a.Stdout) and $($a.Stderr)"} elseif(((Get-Date)-$a.Start).TotalSeconds -gt $MissionTimeoutSeconds){Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; throw "[$($a.Tag)] mission timeout."}
    if(-not $done){$keep += $a}
  }; $active=$keep; if($queue.Count -gt 0 -or $active.Count -gt 0){Start-Sleep -Milliseconds 500}
}
$records | Sort-Object V_mps,H_m | Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $out 'process_lifecycle.csv'); $records | Sort-Object V_mps,H_m | Format-Table -AutoSize | Out-String -Width 260 | Set-Content -Encoding UTF8 (Join-Path $out 'process_lifecycle_summary.txt')

# Aggregate and hard-fail unless all 15 missions and all 75 bank vertices pass.
$outEsc=Escape-MatlabString $out; $cmdF="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_finalize_speed_pilot_v070(string(pwd),OutputRoot=string('$outEsc'),BankPath=string('$pilotBankEsc')); disp(r.pass);"; $finalLog=Join-Path $out 'finalize_terminal.txt'; Write-Host '[finalize] aggregating 15 speed-pilot missions...'; & $MatlabExe -batch $cmdF 2>&1 | Tee-Object -FilePath $finalLog; if($LASTEXITCODE -ne 0){throw "Speed pilot finalizer failed. See $finalLog"}
$summary=Join-Path $out 'speed_pilot_validation_summary.txt'; if(-not (Test-Path $summary)){throw "Speed pilot summary missing: $summary"}; $passLine=Get-Content $summary | Where-Object {$_ -match '^pass='} | Select-Object -First 1; if($passLine -ne 'pass=1'){throw "Physics-MPC v0.7.0 speed pilot contains FAIL/missing cases. See $summary"}
Write-Host '=== Physics-MPC v0.7.0 SPEED PILOT VALIDATION PASS ==='; Write-Host "Summary: $summary"; Write-Host 'Next only after 15/15 PASS: expand to full H=20:10:200 x V=40:5:60 x cfg0:4 bank and 95 nonlinear missions.'
