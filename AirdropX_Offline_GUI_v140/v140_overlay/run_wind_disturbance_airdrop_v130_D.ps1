param(
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [int]$MaxParallel=3,
  [int]$TimeoutSeconds=480,
  [int]$ShutdownGraceSeconds=5,
  [switch]$ForceBuild,
  [switch]$ForceCalibration,
  [switch]$ForceMissions
)
$ErrorActionPreference='Stop'
if($MaxParallel -lt 1 -or $MaxParallel -gt 3){throw 'MaxParallel must be 1..3'}
function E([string]$s){return $s.Replace("'","''")}
function TailText([string]$path,[int]$n=16){if(-not(Test-Path $path)){return ''}; return ((Get-Content $path -Tail $n -ErrorAction SilentlyContinue)-join ' | ').Replace('"',"'")}
function Rec([string]$tag,[int]$pid,[double]$elapsed,[bool]$forced,[bool]$reused,[bool]$infra,[bool]$timeout,[string]$err){[pscustomobject]@{Case=$tag;PID=$pid;Elapsed_s=[math]::Round($elapsed,3);ForcedExitAfterResult=$forced;Reused=$reused;InfraFail=$infra;Timeout=$timeout;ErrorSummary=$err}}
if(-not(Test-Path $MatlabExe)){throw "MATLAB not found: $MatlabExe"}
$bank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v033\physics_bank.mat'; if(-not(Test-Path $bank)){throw "Physics bank missing: $bank"}
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v130_wind_disturbance_airdrop_validation'; New-Item -ItemType Directory -Force -Path $out|Out-Null
$calRoot=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v130_wind_disturbance_calibration'; New-Item -ItemType Directory -Force -Path $calRoot|Out-Null
$cal=Join-Path $calRoot 'wind_disturbance_model_v130.mat'
$rootEsc=E $ProjectRoot; $outEsc=E $out; $bankEsc=E $bank; $calEsc=E $cal

# Stage 0: keep using the separate v1.2.1 continuous-wind Oracle. The validated
# v0.3.3 oracle is still untouched.
$mex=Join-Path $ProjectRoot 'matlab\phys_mpc\airdropx_jsbsim_wind_oracle_mex.mexw64'
if($ForceBuild -or -not(Test-Path $mex)){
  Write-Host '=== Building separate continuous-wind JSBSim Oracle ==='
  $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); build_airdropx_jsbsim_wind_oracle_v121;"
  & $MatlabExe -batch $cmd 2>&1 | Tee-Object -FilePath (Join-Path $out 'wind_oracle_build_terminal.txt')
  if($LASTEXITCODE -ne 0){throw 'Wind Oracle build failed.'}
}
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_wind_oracle_selftest_v121(string(pwd)); disp(r);"
& $MatlabExe -batch $cmd 2>&1 | Tee-Object -FilePath (Join-Path $out 'wind_oracle_selftest_terminal.txt')
if($LASTEXITCODE -ne 0){throw 'Continuous-wind Oracle self-test failed.'}

# Stage 1: real JSBSim delta-wind identification. This replaces guessed wind E
# matrices with a per-cfg physical measured-disturbance map Gw.
if($ForceCalibration -or -not(Test-Path $cal)){
  Write-Host '=== Calibrating Physics-MPC longitudinal wind-increment disturbance map ==='
  $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); W=airdropx_phys_mpc_wind_disturbance_calibrate_v130(string(pwd),string('$bankEsc'),string('$calEsc')); disp(W.table);"
  & $MatlabExe -batch $cmd 2>&1 | Tee-Object -FilePath (Join-Path $calRoot 'wind_disturbance_calibration_terminal.txt')
  if($LASTEXITCODE -ne 0 -or -not(Test-Path $cal)){throw 'Wind disturbance calibration failed.'}
}else{Write-Host "=== Reusing wind disturbance calibration: $cal ==="}

# Stage 2: ballistic manifest is unchanged from v1.2.1.
$manifest=Join-Path $out 'wind_airdrop_manifest.csv'; $manifestEsc=E $manifest
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/wind'); addpath('matlab/airdrop'); p=airdropx_airdrop_ballistic_params_v121; assert(abs(p.calibration_error_m)<1e-8); T=airdropx_wind_airdrop_manifest_v121(string('$manifestEsc')); disp(p); disp(T);"
& $MatlabExe -batch $cmd 2>&1 | Tee-Object -FilePath (Join-Path $out 'manifest_terminal.txt')
if($LASTEXITCODE -ne 0 -or -not(Test-Path $manifest)){throw 'Ballistic calibration/manifest preflight failed.'}
$profiles=Import-Csv $manifest
$jobs=@()
foreach($j in $profiles){
  $jobs += [pscustomobject]@{Scenario=$j.Scenario;Mode='wind_mpc_aware';WindMpc=$true;Aware=$true;Duration=[double]$j.Duration_s;TargetStart=[double]$j.TargetStart_m;Spacing=[double]$j.TargetSpacing_m;Seed=[int]$j.SensorNoiseSeed}
  $jobs += [pscustomobject]@{Scenario=$j.Scenario;Mode='legacy_mpc_aware';WindMpc=$false;Aware=$true;Duration=[double]$j.Duration_s;TargetStart=[double]$j.TargetStart_m;Spacing=[double]$j.TargetSpacing_m;Seed=[int]$j.SensorNoiseSeed}
  $jobs += [pscustomobject]@{Scenario=$j.Scenario;Mode='legacy_mpc_nowind';WindMpc=$false;Aware=$false;Duration=[double]$j.Duration_s;TargetStart=[double]$j.TargetStart_m;Spacing=[double]$j.TargetSpacing_m;Seed=[int]$j.SensorNoiseSeed}
}
$records=@()
function StartCase($j){
  $tag="$($j.Scenario)_$($j.Mode)"; $dir=Join-Path $out $tag; New-Item -ItemType Directory -Force -Path $dir|Out-Null
  $marker=Join-Path $dir 'scenario_complete.ok'; $mat=Join-Path $dir 'wind_airdrop_mission.mat'
  if((-not $ForceMissions) -and (Test-Path $marker) -and (Test-Path $mat)){return [pscustomobject]@{Reuse=$true;Tag=$tag;Dir=$dir}}
  Remove-Item $marker -Force -ErrorAction SilentlyContinue; Remove-Item (Join-Path $dir 'scenario_status.txt') -Force -ErrorAction SilentlyContinue
  $stdout=Join-Path $dir ($tag+'_stdout.txt'); $stderr=Join-Path $dir ($tag+'_stderr.txt'); $dirEsc=E $dir
  $aware=if($j.Aware){'true'}else{'false'}; $wm=if($j.WindMpc){'true'}else{'false'}
  $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/wind'); addpath('matlab/airdrop'); r=airdropx_wind_airdrop_entry_v130(string(pwd),OutputRoot=string('$dirEsc'),ScenarioName=string('$($j.Scenario)'),UseWindCompensation=$aware,UseWindDisturbanceMPC=$wm,WindCalibrationPath=string('$calEsc'),Duration_s=$($j.Duration),TargetStart_m=$($j.TargetStart),TargetSpacing_m=$($j.Spacing),SensorNoiseSeed=$($j.Seed)); disp(r);"
  $p=Start-Process -FilePath $MatlabExe -ArgumentList "-batch `"$cmd`"" -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
  [pscustomobject]@{Reuse=$false;Tag=$tag;Dir=$dir;Marker=$marker;Mat=$mat;Process=$p;PID=$p.Id;Start=Get-Date;Status=(Join-Path $dir 'scenario_status.txt');Stdout=$stdout;Stderr=$stderr;Last='';MarkerAt=$null}
}
function PollCase($a){
  $a.Process.Refresh(); if(Test-Path $a.Status){$ln=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue; if($ln -and $ln -ne $a.Last){Write-Host "[$($a.Tag)] $ln"; $a.Last=$ln}}
  if(Test-Path $a.Marker){if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}; if($a.Process.HasExited){return @{Done=$true;Record=(Rec $a.Tag $a.PID ((Get-Date)-$a.Start).TotalSeconds $false $false $false $false '')}; if(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds){Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; return @{Done=$true;Record=(Rec $a.Tag $a.PID ((Get-Date)-$a.Start).TotalSeconds $true $false $false $false '')}}
  } elseif($a.Process.HasExited){$err=(TailText $a.Status 14)+' || STDERR: '+(TailText $a.Stderr 24)+' || STDOUT: '+(TailText $a.Stdout 24); return @{Done=$true;Record=(Rec $a.Tag $a.PID ((Get-Date)-$a.Start).TotalSeconds $false $false $true $false $err)}
  } elseif(((Get-Date)-$a.Start).TotalSeconds -gt $TimeoutSeconds){Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $err='TIMEOUT || '+(TailText $a.Status 14)+' || STDERR: '+(TailText $a.Stderr 24); return @{Done=$true;Record=(Rec $a.Tag $a.PID ((Get-Date)-$a.Start).TotalSeconds $false $false $true $true $err)}
  return @{Done=$false;Record=$null}
}
# Full integrated calm case first. If the new disturbance QP wiring is broken,
# do not launch 23 more MATLAB processes.
$first=$jobs|Where-Object {$_.Scenario -eq 'calm' -and $_.Mode -eq 'wind_mpc_aware'}|Select-Object -First 1; $a=StartCase $first
if($a.Reuse){$records+=(Rec $a.Tag 0 0 $false $true $false $false '')}else{Write-Host "[$($a.Tag)] SERIAL FULL-CHAIN PREFLIGHT PID $($a.PID)"; while($true){Start-Sleep -Milliseconds 300; $q=PollCase $a; if($q.Done){$records+=$q.Record; break}}; if($q.Record.InfraFail){$records|Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $out 'process_lifecycle.csv'); throw 'Integrated calm preflight failed; remaining cases not launched.'}}
$queue=New-Object System.Collections.Queue; foreach($j in $jobs){if(-not($j.Scenario -eq 'calm' -and $j.Mode -eq 'wind_mpc_aware')){$queue.Enqueue($j)}}; $active=@()
while($queue.Count -gt 0 -or $active.Count -gt 0){
  while($queue.Count -gt 0 -and $active.Count -lt $MaxParallel){$j=$queue.Dequeue(); $a=StartCase $j; if($a.Reuse){$records+=(Rec $a.Tag 0 0 $false $true $false $false ''); Write-Host "[$($a.Tag)] reuse"}else{$active+=$a; Write-Host "[$($a.Tag)] PID $($a.PID)"}}
  $keep=@(); foreach($a in $active){$q=PollCase $a; if($q.Done){$records+=$q.Record}else{$keep+=$a}}; $active=$keep; if($queue.Count -gt 0 -or $active.Count -gt 0){Start-Sleep -Milliseconds 500}
}
$records|Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $out 'process_lifecycle.csv'); $records|Format-Table -AutoSize|Out-String -Width 340|Set-Content -Encoding UTF8 (Join-Path $out 'process_lifecycle_summary.txt')
$infra=$records|Where-Object {$_.InfraFail}; if($infra){$infra|Format-List|Out-String -Width 360|Set-Content -Encoding UTF8 (Join-Path $out 'infrastructure_failures.txt')}
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/airdrop'); R=airdropx_wind_airdrop_finalize_v130(string('$outEsc'),string('$manifestEsc')); disp(R);"
& $MatlabExe -batch $cmd 2>&1 | Tee-Object -FilePath (Join-Path $out 'finalize_terminal.txt')
if($LASTEXITCODE -ne 0){throw 'Wind-disturbance airdrop finalizer failed.'}
Get-Content (Join-Path $out 'wind_disturbance_validation_summary.txt')
