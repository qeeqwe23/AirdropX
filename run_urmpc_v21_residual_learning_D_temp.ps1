param(
    [string]$Speeds = '45,50,55',
    [string]$Cfgs = '0,1,2,3,4',
    [switch]$FitOnly
)
$ErrorActionPreference='Stop'
$ProjectRoot='D:\vscode project\AirdropX'
$MatlabExe='D:\MATLAB R2026a\matlab\bin\matlab.exe'
$TempRoot='D:\MATLAB_TEMP\AirdropX_urmpc_v21'
$TempDir=Join-Path $TempRoot 'temp';$Cache=Join-Path $TempRoot 'cache';$Codegen=Join-Path $TempRoot 'codegen'
New-Item -ItemType Directory -Force -Path $TempRoot,$TempDir,$Cache,$Codegen | Out-Null
$env:TEMP=$TempDir;$env:TMP=$TempDir
function E([string]$s){$s.Replace("'","''")}
$pr=E $ProjectRoot;$ca=E $Cache;$cg=E $Codegen
$sv=@($Speeds -split ','|%{[double]($_.Trim())});$cv=@($Cfgs -split ','|%{[double]($_.Trim())})
$sm='['+(($sv|%{$_.ToString('0.############',[Globalization.CultureInfo]::InvariantCulture)})-join ';')+']'
$cm='['+(($cv|%{$_.ToString('0',[Globalization.CultureInfo]::InvariantCulture)})-join ';')+']'
$fit=if($FitOnly){'true'}else{'false'}
$cmd=@"
cd('$pr');addpath('matlab');addpath('matlab/mpc');addpath('matlab/mpc_auto');addpath('matlab/sfunc_jsbsim');
Simulink.fileGenControl('set','CacheFolder','$ca','CodeGenFolder','$cg','createDir',true);
fprintf('\n=== UR-MPC v2.1A RESIDUAL LEARNING ===\n');fprintf('tempdir=%s\n',char(tempdir));
if ~$fit
  c=airdropx_urmpc_v21_vertex_calibration('ProjectRoot',pwd,'SpeedsMps',$sm,'CfgIds',$cm);
  disp(c.manifest(:,{'speed_mps','cfg_id','fit_samples','trusted_samples','finite_fraction','status'}));
end
f=airdropx_urmpc_v21_fit_residual_models('ProjectRoot',pwd);
disp(f.fit_report(:,{'speed_mps','cfg_id','trusted_samples','validation_ratio','deltaA_fro','deltaB_fro','status'}));
fprintf('selected_lambda=%.6g selection_policy=%s worst=%.6f deploy_ready=%d\n',f.selected_lambda,char(f.selection_policy),f.selected_worst_validation_ratio,f.deploy_ready);
"@
Write-Host '=== AirdropX UR-MPC v2.1A: nonlinear vertex residual learning ==='
Write-Host "Speeds=$Speeds Cfgs=$Cfgs FitOnly=$($FitOnly.IsPresent)"
& $MatlabExe -batch $cmd
exit $LASTEXITCODE
