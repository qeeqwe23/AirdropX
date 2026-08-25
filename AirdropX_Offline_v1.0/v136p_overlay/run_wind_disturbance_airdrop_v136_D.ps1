param(
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [int]$MaxParallel=3,
  [int]$TimeoutSeconds=480,
  [int]$ShutdownGraceSeconds=5,
  [switch]$ForceBuild,
  [switch]$ForceCalibration,
  [switch]$ForceMissions,
  [switch]$ShowChildWindows
)
$ErrorActionPreference='Stop'
if($MaxParallel -lt 1 -or $MaxParallel -gt 3){throw 'MaxParallel must be 1..3'}
function E([string]$s){return $s.Replace("'","''")}
function TailText([string]$path,[int]$n=16){if(-not(Test-Path $path)){return ''}; return ((Get-Content $path -Tail $n -ErrorAction SilentlyContinue)-join ' | ').Replace('"',"'")}
function Rec([string]$tag,[int]$procId,[double]$elapsed,[bool]$forced,[bool]$reused,[bool]$infra,[bool]$timeout,[string]$err){[pscustomobject]@{Case=$tag;PID=$procId;Elapsed_s=[math]::Round($elapsed,3);ForcedExitAfterResult=$forced;Reused=$reused;InfraFail=$infra;Timeout=$timeout;ErrorSummary=$err}}
function MatlabStartArgs([string]$cmd,[string]$stdout,[string]$stderr,[switch]$Wait){
  $h=@{FilePath=$MatlabExe;ArgumentList="-batch `"$cmd`"";RedirectStandardOutput=$stdout;RedirectStandardError=$stderr;PassThru=$true}
  if($Wait){$h.Wait=$true}
  if(-not $ShowChildWindows){$h.WindowStyle='Hidden'}
  return $h
}
function Invoke-MatlabCapture([string]$cmd,[string]$logPath){
  $stdout=$logPath+'.stdout.tmp'; $stderr=$logPath+'.stderr.tmp'; Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue
  $sp=MatlabStartArgs $cmd $stdout $stderr -Wait; $p=Start-Process @sp
  $parts=@(); if(Test-Path $stdout){$parts+=Get-Content $stdout}; if(Test-Path $stderr){$parts+=Get-Content $stderr}; $parts|Set-Content -Encoding UTF8 $logPath
  if($parts){$parts|ForEach-Object {Write-Host $_}}
  Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue
  return $p.ExitCode
}
if(-not(Test-Path $MatlabExe)){throw "MATLAB not found: $MatlabExe"}
$bank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v033\physics_bank.mat'; if(-not(Test-Path $bank)){throw "Physics bank missing: $bank"}
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v136_transient_energy_recovery_airdrop_validation'; New-Item -ItemType Directory -Force -Path $out|Out-Null
$calRoot=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v130_wind_disturbance_calibration'; New-Item -ItemType Directory -Force -Path $calRoot|Out-Null
$cal=Join-Path $calRoot 'wind_disturbance_model_v130.mat'
$rootEsc=E $ProjectRoot; $outEsc=E $out; $bankEsc=E $bank; $calEsc=E $cal

# Stage 0. Use Start-Process capture so harmless JSBSim stderr notices do not
# become terminating NativeCommandError records under Windows PowerShell 5.1.
$mex=Join-Path $ProjectRoot 'matlab\phys_mpc\airdropx_jsbsim_wind_oracle_mex.mexw64'
$oracleMarker=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v135_oracle_selftest.ok'
if($ForceBuild -or -not(Test-Path $mex) -or -not(Test-Path $oracleMarker)){
  Write-Host '=== Building separate continuous-wind JSBSim Oracle ==='
  $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); build_airdropx_jsbsim_wind_oracle_v121;"
  $code=Invoke-MatlabCapture $cmd (Join-Path $out 'wind_oracle_build_terminal.txt'); if($code -ne 0){throw 'Wind Oracle build failed.'}
}
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_wind_oracle_selftest_v121(string(pwd)); assert(contains(string(r.version),'mass-refresh-v135')); disp(r);"
$code=Invoke-MatlabCapture $cmd (Join-Path $out 'wind_oracle_selftest_terminal.txt'); if($code -ne 0){throw 'Continuous-wind Oracle self-test failed.'}; Set-Content -Encoding ASCII -Path $oracleMarker -Value ("pass "+(Get-Date -Format o))

# Stage 1. Reuse the physical cfg0..4 Gw calibration unless explicitly forced.
if($ForceCalibration -or -not(Test-Path $cal)){
  Write-Host '=== Calibrating Physics-MPC longitudinal wind-increment disturbance map ==='
  $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); W=airdropx_phys_mpc_wind_disturbance_calibrate_v130(string(pwd),string('$bankEsc'),string('$calEsc')); disp(W.table);"
  $code=Invoke-MatlabCapture $cmd (Join-Path $calRoot 'wind_disturbance_calibration_terminal.txt'); if($code -ne 0 -or -not(Test-Path $cal)){throw 'Wind disturbance calibration failed.'}
}else{Write-Host "=== Reusing wind disturbance calibration: $cal ==="}

# Stage 2. Existing ballistic calibration remains unchanged.
$manifest=Join-Path $out 'wind_airdrop_manifest.csv'; $manifestEsc=E $manifest
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/wind'); addpath('matlab/airdrop'); p=airdropx_airdrop_ballistic_params_v121; assert(abs(p.calibration_error_m)<1e-8); T=airdropx_wind_airdrop_manifest_v136(string('$manifestEsc')); disp(p); disp(T);"
$code=Invoke-MatlabCapture $cmd (Join-Path $out 'manifest_terminal.txt'); if($code -ne 0 -or -not(Test-Path $manifest)){throw 'Ballistic calibration/manifest preflight failed.'}
$profiles=Import-Csv $manifest

# Stage 2.5. Before spending time on the 25-case suite, prove that calm with
# zero recovery/disturbance evidence is numerically the SAME base controller.
$eqRoot=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v136_base_equivalence_audit'; $eqEsc=E $eqRoot
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/wind'); addpath('matlab/airdrop'); R=airdropx_phys_mpc_base_equivalence_audit_v136(string(pwd),OutputRoot=string('$eqEsc'),WindCalibrationPath=string('$calEsc'),SensorNoiseSeed=101); disp(R); assert(R.pass);"
$code=Invoke-MatlabCapture $cmd (Join-Path $out 'base_equivalence_terminal.txt'); if($code -ne 0){throw 'v1.3.6 base-equivalence audit failed. Full suite intentionally not started.'}
$jobs=@()
foreach($j in $profiles){
  $jobs += [pscustomobject]@{Scenario=$j.Scenario;Mode='wind_mpc_aware';WindMpc=$true;Aware=$true;Fractional=$true;Confidence=$true;Recovery=$true;Duration=[double]$j.Duration_s;TargetStart=[double]$j.TargetStart_m;Spacing=[double]$j.TargetSpacing_m;Seed=[int]$j.SensorNoiseSeed}
  $jobs += [pscustomobject]@{Scenario=$j.Scenario;Mode='legacy_mpc_aware';WindMpc=$false;Aware=$true;Fractional=$true;Confidence=$true;Recovery=$false;Duration=[double]$j.Duration_s;TargetStart=[double]$j.TargetStart_m;Spacing=[double]$j.TargetSpacing_m;Seed=[int]$j.SensorNoiseSeed}
  $jobs += [pscustomobject]@{Scenario=$j.Scenario;Mode='legacy_mpc_nowind';WindMpc=$false;Aware=$false;Fractional=$true;Confidence=$true;Recovery=$false;Duration=[double]$j.Duration_s;TargetStart=[double]$j.TargetStart_m;Spacing=[double]$j.TargetSpacing_m;Seed=[int]$j.SensorNoiseSeed}
}
# Diagnostic only: preserve the sampled-release A/B on calm.
$calmRow=$profiles|Where-Object {$_.Scenario -eq 'calm'}|Select-Object -First 1
if($null -ne $calmRow){$jobs += [pscustomobject]@{Scenario='calm';Mode='wind_mpc_aware_sampled_release';WindMpc=$true;Aware=$true;Fractional=$false;Confidence=$true;Recovery=$true;Duration=[double]$calmRow.Duration_s;TargetStart=[double]$calmRow.TargetStart_m;Spacing=[double]$calmRow.TargetSpacing_m;Seed=[int]$calmRow.SensorNoiseSeed}}
$records=@()
function StartCase($j){
  $tag="$($j.Scenario)_$($j.Mode)"; $dir=Join-Path $out $tag; New-Item -ItemType Directory -Force -Path $dir|Out-Null
  $marker=Join-Path $dir 'scenario_complete.ok'; $mat=Join-Path $dir 'wind_airdrop_mission.mat'
  if((-not $ForceMissions) -and (Test-Path $marker) -and (Test-Path $mat)){return [pscustomobject]@{Reuse=$true;Tag=$tag;Dir=$dir}}
  Remove-Item $marker -Force -ErrorAction SilentlyContinue; Remove-Item (Join-Path $dir 'scenario_status.txt') -Force -ErrorAction SilentlyContinue
  $stdout=Join-Path $dir ($tag+'_stdout.txt'); $stderr=Join-Path $dir ($tag+'_stderr.txt'); $dirEsc=E $dir
  $aware=if($j.Aware){'true'}else{'false'}; $wm=if($j.WindMpc){'true'}else{'false'}; $frac=if($j.Fractional){'true'}else{'false'}; $conf=if($j.Confidence){'true'}else{'false'}; $rec=if($j.Recovery){'true'}else{'false'}
  $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/wind'); addpath('matlab/airdrop'); r=airdropx_wind_airdrop_entry_v136(string(pwd),OutputRoot=string('$dirEsc'),ScenarioName=string('$($j.Scenario)'),UseWindCompensation=$aware,UseWindDisturbanceMPC=$wm,UseFractionalRelease=$frac,UseWindConfidenceGate=$conf,UseUnifiedGustRecovery=$rec,WindCalibrationPath=string('$calEsc'),Duration_s=$($j.Duration),TargetStart_m=$($j.TargetStart),TargetSpacing_m=$($j.Spacing),SensorNoiseSeed=$($j.Seed)); disp(r);"
  $sp=MatlabStartArgs $cmd $stdout $stderr; $p=Start-Process @sp
  [pscustomobject]@{Reuse=$false;Tag=$tag;Dir=$dir;Marker=$marker;Mat=$mat;Process=$p;ProcId=$p.Id;Start=Get-Date;Status=(Join-Path $dir 'scenario_status.txt');Stdout=$stdout;Stderr=$stderr;Last='';MarkerAt=$null}
}
function PollCase($a){
  $a.Process.Refresh()
  if(Test-Path $a.Status){$ln=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue; if($ln -and $ln -ne $a.Last){Write-Host "[$($a.Tag)] $ln"; $a.Last=$ln}}
  if(Test-Path $a.Marker){
    if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}
    if($a.Process.HasExited){return @{Done=$true;Record=(Rec $a.Tag $a.ProcId ((Get-Date)-$a.Start).TotalSeconds $false $false $false $false '')}}
    if(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds){Stop-Process -Id $a.ProcId -Force -ErrorAction SilentlyContinue; return @{Done=$true;Record=(Rec $a.Tag $a.ProcId ((Get-Date)-$a.Start).TotalSeconds $true $false $false $false '')}}
  } elseif($a.Process.HasExited){
    $err=(TailText $a.Status 14)+' || STDERR: '+(TailText $a.Stderr 24)+' || STDOUT: '+(TailText $a.Stdout 24); return @{Done=$true;Record=(Rec $a.Tag $a.ProcId ((Get-Date)-$a.Start).TotalSeconds $false $false $true $false $err)}
  } elseif(((Get-Date)-$a.Start).TotalSeconds -gt $TimeoutSeconds){
    Stop-Process -Id $a.ProcId -Force -ErrorAction SilentlyContinue; $err='TIMEOUT || '+(TailText $a.Status 14)+' || STDERR: '+(TailText $a.Stderr 24); return @{Done=$true;Record=(Rec $a.Tag $a.ProcId ((Get-Date)-$a.Start).TotalSeconds $false $false $true $true $err)}
  }
  return @{Done=$false;Record=$null}
}
$first=$jobs|Where-Object {$_.Scenario -eq 'calm' -and $_.Mode -eq 'wind_mpc_aware'}|Select-Object -First 1; $a=StartCase $first
if($a.Reuse){$records+=(Rec $a.Tag 0 0 $false $true $false $false '')}else{Write-Host "[$($a.Tag)] SERIAL FULL-CHAIN PREFLIGHT PID $($a.ProcId)"; while($true){Start-Sleep -Milliseconds 300; $q=PollCase $a; if($q.Done){$records+=$q.Record; break}}; if($q.Record.InfraFail){$records|Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $out 'process_lifecycle.csv'); throw 'Integrated calm preflight failed; remaining cases not launched.'}}
$queue=New-Object System.Collections.Queue; foreach($j in $jobs){if(-not($j.Scenario -eq 'calm' -and $j.Mode -eq 'wind_mpc_aware')){$queue.Enqueue($j)}}; $active=@()
while($queue.Count -gt 0 -or $active.Count -gt 0){
  while($queue.Count -gt 0 -and $active.Count -lt $MaxParallel){$j=$queue.Dequeue(); $a=StartCase $j; if($a.Reuse){$records+=(Rec $a.Tag 0 0 $false $true $false $false ''); Write-Host "[$($a.Tag)] reuse"}else{$active+=$a; Write-Host "[$($a.Tag)] PID $($a.ProcId)"}}
  $keep=@(); foreach($a in $active){$q=PollCase $a; if($q.Done){$records+=$q.Record}else{$keep+=$a}}; $active=$keep; if($queue.Count -gt 0 -or $active.Count -gt 0){Start-Sleep -Milliseconds 500}
}
$records|Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $out 'process_lifecycle.csv'); $records|Format-Table -AutoSize|Out-String -Width 340|Set-Content -Encoding UTF8 (Join-Path $out 'process_lifecycle_summary.txt')
$infra=$records|Where-Object {$_.InfraFail}; if($infra){$infra|Format-List|Out-String -Width 360|Set-Content -Encoding UTF8 (Join-Path $out 'infrastructure_failures.txt')}
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/airdrop'); R=airdropx_wind_airdrop_finalize_v136(string('$outEsc'),string('$manifestEsc')); disp(R);"
$code=Invoke-MatlabCapture $cmd (Join-Path $out 'finalize_terminal.txt'); if($code -ne 0){throw 'v1.3.6 transient-energy finalizer failed.'}
Get-Content (Join-Path $out 'wind_transient_energy_recovery_validation_summary.txt')
