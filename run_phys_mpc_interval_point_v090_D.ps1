param(
  [double]$H=137.4,
  [double]$V=58.2,
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe'
)
$ErrorActionPreference='Stop'
if($H -lt 20 -or $H -gt 200){throw 'H must be in [20,200] m'}; if($V -lt 45 -or $V -gt 65){throw 'V must be in [45,65] m/s'}
function Escape-MatlabString([string]$s){return $s.Replace("'","''")}
$bank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v082_fixed_horizon_envelope_bank\physics_full_envelope_bank_diagnostic.mat'; if(-not (Test-Path $bank)){throw "Master bank missing: $bank"}
$tag=('H{0:D5}_V{1:D5}' -f [int][math]::Round(100*$H),[int][math]::Round(100*$V)); $out=Join-Path $ProjectRoot ("matlab\results\physics_mpc_v090_interval_point\"+$tag); New-Item -ItemType Directory -Force -Path $out | Out-Null
$rootEsc=Escape-MatlabString $ProjectRoot; $outEsc=Escape-MatlabString $out; $bankEsc=Escape-MatlabString $bank
$cmd="cd('$rootEsc'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); r=airdropx_phys_interval_entry_v090(string(pwd),OutputRoot=string('$outEsc'),Height_m=$H,Speed_mps=$V,MasterBankPath=string('$bankEsc')); disp(r);"
& $MatlabExe -batch $cmd
Write-Host "Result: $out\interval_summary.txt"; if(Test-Path (Join-Path $out 'interval_summary.txt')){Get-Content (Join-Path $out 'interval_summary.txt')}
