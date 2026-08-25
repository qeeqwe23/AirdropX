param(
 [string]$ProjectRoot='D:\vscode project\AirdropX',
 [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe',
 [int]$Seed=101,
 [switch]$ShowChildWindows
)
$ErrorActionPreference='Stop'; function E([string]$s){$s.Replace("'","''")}
function StartMatlab([string]$cmd,[string]$stdout,[string]$stderr){$sp=@{FilePath=$MatlabExe;ArgumentList="-batch `"$cmd`"";Wait=$true;PassThru=$true;RedirectStandardOutput=$stdout;RedirectStandardError=$stderr};if(-not $ShowChildWindows){$sp.WindowStyle='Hidden'};Start-Process @sp}
$root=E $ProjectRoot; $cal=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v130_wind_disturbance_calibration\wind_disturbance_model_v130.mat'; if(-not(Test-Path $cal)){throw 'Wind calibration missing.'}
$out=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v136p_paper_base_equivalence_audit'; New-Item -ItemType Directory -Force -Path $out|Out-Null
$mex=Join-Path $ProjectRoot 'matlab\phys_mpc\airdropx_jsbsim_wind_oracle_mex.mexw64'; $marker=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v135_oracle_selftest.ok'
if(-not(Test-Path $mex)){
  $so=Join-Path $out 'oracle_build_stdout.txt';$se=Join-Path $out 'oracle_build_stderr.txt';$cmd="cd('$root'); addpath('matlab'); addpath('matlab/phys_mpc'); build_airdropx_jsbsim_wind_oracle_v121;";$p=StartMatlab $cmd $so $se;if($p.ExitCode -ne 0){if(Test-Path $so){Get-Content $so};if(Test-Path $se){Get-Content $se};throw 'v1.3.5-compatible Oracle build failed.'}
}
if(-not(Test-Path $marker)){
  $so=Join-Path $out 'oracle_selftest_stdout.txt';$se=Join-Path $out 'oracle_selftest_stderr.txt';$cmd="cd('$root'); addpath('matlab'); addpath('matlab/phys_mpc'); r=airdropx_phys_wind_oracle_selftest_v121(string(pwd)); assert(contains(string(r.version),'mass-refresh-v135')); disp(r);";$p=StartMatlab $cmd $so $se;if($p.ExitCode -ne 0){if(Test-Path $so){Get-Content $so};if(Test-Path $se){Get-Content $se};throw 'v1.3.5-compatible Oracle selftest failed.'};Set-Content -Encoding ASCII -Path $marker -Value ("pass "+(Get-Date -Format o))
}
$cmd="cd('$root'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/wind'); addpath('matlab/airdrop'); addpath('matlab/avionics'); R=airdropx_phys_mpc_base_equivalence_audit_v136p(string(pwd),OutputRoot=string('$(E $out)'),WindCalibrationPath=string('$(E $cal)'),SensorNoiseSeed=$Seed); disp(R); if ~R.pass, error('AirdropX:WindMPC:BaseEquivalenceFailed','Base equivalence audit failed.'); end"
$stdout=Join-Path $out 'audit_stdout.txt'; $stderr=Join-Path $out 'audit_stderr.txt'; Remove-Item $stdout,$stderr -Force -ErrorAction SilentlyContinue
$p=StartMatlab $cmd $stdout $stderr; if(Test-Path $stdout){Get-Content $stdout}; if(Test-Path $stderr){Get-Content $stderr|Write-Host}; exit $p.ExitCode
