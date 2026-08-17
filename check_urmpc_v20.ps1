$root='D:\vscode project\AirdropX\matlab\results\mpc_physics_v1'
$build=Join-Path $root 'unified_robust_mpc_v2';$run=Join-Path $root 'fixed_stability_urmpc_v20'
Write-Output '=== DISK ===';Get-PSDrive C,D|Select-Object Name,@{N='FreeGB';E={[math]::Round($_.Free/1GB,2)}}
Write-Output '=== MATLAB ===';Get-Process MATLAB -ErrorAction SilentlyContinue|Select-Object Id,@{N='CPU_s';E={[math]::Round($_.CPU,1)}},@{N='RAM_GB';E={[math]::Round($_.WorkingSet64/1GB,2)}}
foreach($f in 'urmpc_vertex_preflight.csv','urmpc_linear_drop_cert.csv','urmpc_model_envelope_report.csv'){$p=Join-Path $build $f;if(Test-Path $p){Write-Output "=== $f ===";Get-Content $p|Select-Object -Last 20}}
foreach($f in 'fixed_stability_summary.csv','fixed_stability_run_errors.csv'){$p=Join-Path $run $f;if(Test-Path $p){Write-Output "=== $f ===";Get-Content $p|Select-Object -Last 20}}
$t=Join-Path $run 'V050\urmpc_controller_trace.csv';if(Test-Path $t){Write-Output '=== V050 UR-MPC TRACE TAIL ===';Get-Content $t|Select-Object -Last 5}
