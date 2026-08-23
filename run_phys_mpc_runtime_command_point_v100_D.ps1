param(
  [ValidateSet('altitude_down_v45','altitude_up_v65','speed_up_h20','speed_down_h200','coupled_low_to_high','coupled_high_to_low')][string]$Scenario='coupled_low_to_high',
  [ValidateSet('known_reference','hold_current')][string]$PreviewMode='known_reference',
  [string]$ProjectRoot='D:\vscode project\AirdropX',
  [string]$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe'
)
$ErrorActionPreference='Stop'; function Escape-MatlabString([string]$s){return $s.Replace("'","''")}
$bank=Join-Path $ProjectRoot 'matlab\results\physics_mpc_v082_fixed_horizon_envelope_bank\physics_full_envelope_bank_diagnostic.mat'; if(-not (Test-Path $bank)){throw "Master bank missing: $bank"}; $out=Join-Path $ProjectRoot ("matlab\results\physics_mpc_v100_runtime_command_point\"+$Scenario+'_'+$PreviewMode); New-Item -ItemType Directory -Force -Path $out | Out-Null
$r=Escape-MatlabString $ProjectRoot; $b=Escape-MatlabString $bank; $o=Escape-MatlabString $out; $cmd="cd('$r'); addpath('matlab'); addpath('matlab/phys_mpc'); addpath('matlab/sfunc_jsbsim'); x=airdropx_phys_runtime_entry_v100(string(pwd),OutputRoot=string('$o'),ScenarioName=string('$Scenario'),MasterBankPath=string('$b'),Duration_s=150,CommandPreviewMode=string('$PreviewMode')); disp(x);"; & $MatlabExe -batch $cmd
$sum=Join-Path $out 'runtime_command_summary.txt'; if(Test-Path $sum){Get-Content $sum}
