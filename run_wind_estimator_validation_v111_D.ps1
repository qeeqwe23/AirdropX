param(
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
  [int]$MaxParallel=3,
  [int]$TimeoutSeconds=240,
  [int]$ShutdownGraceSeconds=5,
  [switch]$Force
)
$ErrorActionPreference='Stop'
if($MaxParallel -lt 1 -or $MaxParallel -gt 3){throw 'MaxParallel must be 1..3'}
function E([string]$s){return $s.Replace("'","''")}
function TailText([string]$path,[int]$n=12){
  if(-not(Test-Path $path)){return ''}
  return ((Get-Content $path -Tail $n -ErrorAction SilentlyContinue) -join ' | ').Replace('"',"'")
}
function MakeRecord([string]$tag,[int]$pid,[double]$elapsed,[bool]$forced,[bool]$reused,[bool]$infra,[bool]$timeout,[string]$err){
  return [pscustomobject]@{Scenario=$tag;PID=$pid;Elapsed_s=[math]::Round($elapsed,3);ForcedExitAfterResult=$forced;Reused=$reused;InfraFail=$infra;Timeout=$timeout;ErrorSummary=$err}
}
if(-not(Test-Path $MatlabExe)){throw "MATLAB not found: $MatlabExe"}
$sfun=Join-Path $ProjectRoot 'matlab\sfunc_jsbsim\sfun_airdropx_jsbsim.mexw64'
if(-not(Test-Path $sfun)){throw "Working JSBSim S-function MEX missing: $sfun"}
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v111_longitudinal_wind_validation'
New-Item -ItemType Directory -Force -Path $out | Out-Null
$rootEsc=E $ProjectRoot; $outEsc=E $out

# 1) Estimator-only preflight.
$self=Join-Path $out 'synthetic_selftest.txt'; $selfEsc=E $self
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/wind'); r=airdropx_wind_synthetic_selftest_v111(string('$selfEsc')); disp(r);"
& $MatlabExe -batch $cmd 2>&1 | Tee-Object -FilePath (Join-Path $out 'synthetic_selftest_terminal.txt')
if($LASTEXITCODE -ne 0){throw 'Wind estimator synthetic self-test failed.'}

# 2) Manifest.
$manifest=Join-Path $out 'wind_profile_manifest.csv'; $manifestEsc=E $manifest
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/wind'); T=airdropx_wind_profile_manifest_v111(string('$manifestEsc')); disp(T);"
& $MatlabExe -batch $cmd 2>&1 | Tee-Object -FilePath (Join-Path $out 'manifest_terminal.txt')
if($LASTEXITCODE -ne 0 -or -not(Test-Path $manifest)){throw 'Wind profile manifest failed.'}
$jobs=Import-Csv $manifest
$records=@()

# Helper launches one scenario and returns process metadata.
function StartScenario($j){
  $tag=[string]$j.Scenario; $dur=[double]$j.Duration_s
  $dir=Join-Path $out $tag; New-Item -ItemType Directory -Force -Path $dir | Out-Null
  $marker=Join-Path $dir 'scenario_complete.ok'; $mat=Join-Path $dir 'wind_estimation_validation.mat'
  if((-not $Force) -and (Test-Path $marker) -and (Test-Path $mat)){
    return [pscustomobject]@{Reuse=$true;Scenario=$tag;Dir=$dir;Marker=$marker;Mat=$mat}
  }
  Remove-Item $marker -Force -ErrorAction SilentlyContinue
  Remove-Item (Join-Path $dir 'scenario_status.txt') -Force -ErrorAction SilentlyContinue
  $stdout=Join-Path $dir ($tag+'_stdout.txt'); $stderr=Join-Path $dir ($tag+'_stderr.txt'); $dirEsc=E $dir
  $cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/wind'); addpath('matlab/sfunc_jsbsim'); r=airdropx_wind_estimation_entry_v111(string(pwd),OutputRoot=string('$dirEsc'),ScenarioName=string('$tag'),Duration_s=$dur); disp(r);"
  $p=Start-Process -FilePath $MatlabExe -ArgumentList "-batch `"$cmd`"" -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
  return [pscustomobject]@{Reuse=$false;Scenario=$tag;Dir=$dir;Marker=$marker;Mat=$mat;Process=$p;PID=$p.Id;Start=Get-Date;Status=(Join-Path $dir 'scenario_status.txt');Stdout=$stdout;Stderr=$stderr;Last='';MarkerAt=$null}
}

function WaitScenario($a,[bool]$Preflight=$false){
  if($a.Reuse){$r=MakeRecord $a.Scenario 0 0 $false $true $false $false ''; return $r}
  Write-Host "[$($a.Scenario)] started PID $($a.PID)" $(if($Preflight){'[SERIAL HARNESS PREFLIGHT]'})
  while($true){
    Start-Sleep -Milliseconds 250
    $a.Process.Refresh()
    if(Test-Path $a.Status){
      $ln=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue
      if($ln -and $ln -ne $a.Last){Write-Host "[$($a.Scenario)] $ln"; $a.Last=$ln}
    }
    if(Test-Path $a.Marker){
      if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}
      if($a.Process.HasExited){$r=MakeRecord $a.Scenario $a.PID ((Get-Date)-$a.Start).TotalSeconds $false $false $false $false ''; return $r}
      if(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds){
        Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue
        $r=MakeRecord $a.Scenario $a.PID ((Get-Date)-$a.Start).TotalSeconds $true $false $false $false ''; return $r
      }
    } elseif($a.Process.HasExited){
      $err=(TailText $a.Status 8)+' || STDERR: '+(TailText $a.Stderr 16)+' || STDOUT: '+(TailText $a.Stdout 16)
      $r=MakeRecord $a.Scenario $a.PID ((Get-Date)-$a.Start).TotalSeconds $false $false $true $false $err; return $r
    } elseif(((Get-Date)-$a.Start).TotalSeconds -gt $TimeoutSeconds){
      Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue
      $err='TIMEOUT || '+(TailText $a.Status 8)+' || STDERR: '+(TailText $a.Stderr 16)
      $r=MakeRecord $a.Scenario $a.PID ((Get-Date)-$a.Start).TotalSeconds $false $false $true $true $err; return $r
    }
  }
}

# 3) Serial calm preflight. This prevents launching 8 identical infrastructure failures.
$calm=$jobs | Where-Object {$_.Scenario -eq 'calm'} | Select-Object -First 1
if($null -eq $calm){throw 'Manifest does not contain calm preflight scenario.'}
$calmJob=StartScenario $calm
$calmRec=WaitScenario $calmJob $true
$records += $calmRec
if($calmRec.InfraFail){
  $records | Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $out 'process_lifecycle.csv')
  $records | Format-List | Out-String -Width 260 | Set-Content -Encoding UTF8 (Join-Path $out 'harness_preflight_failure.txt')
  Write-Host '=== CALM HARNESS PREFLIGHT FAILED ==='
  Write-Host $calmRec.ErrorSummary
  throw 'JSBSim wind harness preflight failed. See harness_preflight_failure.txt and calm stdout/stderr; remaining scenarios were intentionally not launched.'
}
Write-Host '[calm] harness preflight PASS; launching remaining profiles.'

# 4) Remaining seven scenarios, at most MaxParallel.
$queue=New-Object System.Collections.Queue
foreach($j in $jobs){if([string]$j.Scenario -ne 'calm'){$queue.Enqueue($j)}}
$active=@()
while($queue.Count -gt 0 -or $active.Count -gt 0){
  while($queue.Count -gt 0 -and $active.Count -lt $MaxParallel){
    $j=$queue.Dequeue(); $a=StartScenario $j
    if($a.Reuse){$records += (MakeRecord $a.Scenario 0 0 $false $true $false $false ''); Write-Host "[$($a.Scenario)] reuse"}
    else{$active += $a; Write-Host "[$($a.Scenario)] started PID $($a.PID)"}
  }
  $keep=@()
  foreach($a in $active){
    $a.Process.Refresh()
    if(Test-Path $a.Status){$ln=Get-Content $a.Status -Tail 1 -ErrorAction SilentlyContinue; if($ln -and $ln -ne $a.Last){Write-Host "[$($a.Scenario)] $ln"; $a.Last=$ln}}
    $done=$false; $rec=$null
    if(Test-Path $a.Marker){
      if($null -eq $a.MarkerAt){$a.MarkerAt=Get-Date}
      if($a.Process.HasExited){$rec=MakeRecord $a.Scenario $a.PID ((Get-Date)-$a.Start).TotalSeconds $false $false $false $false ''; $done=$true}
      elseif(((Get-Date)-$a.MarkerAt).TotalSeconds -ge $ShutdownGraceSeconds){Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue; $rec=MakeRecord $a.Scenario $a.PID ((Get-Date)-$a.Start).TotalSeconds $true $false $false $false ''; $done=$true}
    } elseif($a.Process.HasExited){
      $err=(TailText $a.Status 8)+' || STDERR: '+(TailText $a.Stderr 16)+' || STDOUT: '+(TailText $a.Stdout 16)
      $rec=MakeRecord $a.Scenario $a.PID ((Get-Date)-$a.Start).TotalSeconds $false $false $true $false $err; $done=$true
    } elseif(((Get-Date)-$a.Start).TotalSeconds -gt $TimeoutSeconds){
      Stop-Process -Id $a.PID -Force -ErrorAction SilentlyContinue
      $err='TIMEOUT || '+(TailText $a.Status 8)+' || STDERR: '+(TailText $a.Stderr 16)
      $rec=MakeRecord $a.Scenario $a.PID ((Get-Date)-$a.Start).TotalSeconds $false $false $true $true $err; $done=$true
    }
    if($done){$records += $rec}else{$keep += $a}
  }
  $active=$keep
  if($queue.Count -gt 0 -or $active.Count -gt 0){Start-Sleep -Milliseconds 500}
}

$records | Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $out 'process_lifecycle.csv')
$records | Format-Table -AutoSize | Out-String -Width 300 | Set-Content -Encoding UTF8 (Join-Path $out 'process_lifecycle_summary.txt')
$infra=$records | Where-Object {$_.InfraFail}
if($infra){$infra | Format-List | Out-String -Width 320 | Set-Content -Encoding UTF8 (Join-Path $out 'infrastructure_failures.txt')}

# 5) Final metrics.
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/wind'); R=airdropx_wind_finalize_v111(string('$outEsc'),string('$manifestEsc')); disp(R);"
& $MatlabExe -batch $cmd 2>&1 | Tee-Object -FilePath (Join-Path $out 'finalize_terminal.txt')
if($LASTEXITCODE -ne 0){throw 'Wind validation finalizer failed.'}
Get-Content (Join-Path $out 'wind_validation_summary.txt')
