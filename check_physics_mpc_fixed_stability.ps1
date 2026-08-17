$root='D:\vscode project\AirdropX\matlab\results\mpc_physics_v1\fixed_stability_cl17'
$cacheRoot='D:\MATLAB_TEMP\AirdropX_physics_mpc_cl17'

Write-Output '=== disk free ==='
Get-PSDrive C,D | Select-Object Name,@{N='FreeGB';E={[math]::Round($_.Free/1GB,2)}},@{N='UsedGB';E={[math]::Round($_.Used/1GB,2)}}

Write-Output '=== CL-1.7 D-drive cache sizes ==='
foreach ($p in @(
    "$cacheRoot\temp",
    "$cacheRoot\jobs",
    "$cacheRoot\main_cache",
    "$cacheRoot\main_codegen",
    'D:\AXC\phys_cl17'
)) {
    if (Test-Path $p) {
        $s=(Get-ChildItem $p -Recurse -File -ErrorAction SilentlyContinue | Measure-Object Length -Sum).Sum
        "{0,8:N2} GB  {1}" -f ($s/1GB),$p
    }
}

Write-Output '=== MATLAB procs ==='
Get-Process -Name MATLAB -ErrorAction SilentlyContinue | Select-Object Id,@{N='CPU_s';E={[math]::Round($_.CPU,1)}},@{N='RAM_GB';E={[math]::Round($_.WorkingSet64/1GB,2)}}
Write-Output '=== speed segment outputs ==='
Get-ChildItem $root -Recurse -Filter 'fixed_stability_segments.csv' -ErrorAction SilentlyContinue | ForEach-Object {
  Write-Output "--- $($_.FullName) ---"; Get-Content $_.FullName
}
Write-Output '=== aggregate summary ==='
Get-Content "$root\fixed_stability_summary.csv" -ErrorAction SilentlyContinue
Write-Output '=== run errors ==='
Get-Content "$root\fixed_stability_run_errors.csv" -ErrorAction SilentlyContinue


Write-Output '=== startup/nonfinite controller diagnostics ==='
Get-ChildItem $root -Recurse -Filter 'v32_controller_trace.csv' -ErrorAction SilentlyContinue | ForEach-Object {
    Write-Output "--- $($_.FullName) first 12 rows ---"
    $t = Import-Csv $_.FullName
    $t | Select-Object -First 12 time_s,cfg_id,requested_v_mps,actual_h_m,actual_v_mps,actual_vz_mps,pitch_deg,mass_kg_controller,cfg_mass_raw,cfg_used,input_invalid_count,startup_hold_count,state_ready,mpc_success_count,mpc_exception_count,mpc_qp_fail_count | Format-Table -AutoSize
    Write-Output "--- last controller diagnostics ---"
    $t | Select-Object -Last 1 time_s,cfg_id,cfg_used,cfg_invalid_count,input_invalid_count,startup_hold_count,state_ready,mpc_success_count,mpc_exception_count,mpc_qp_fail_count,mpc_last_iterations,mpc_gate_reject_count,recovery_count,recovery_mode,recovery_reason_code,recovery_hard_count,recovery_enter_count,authority_limit_count,authority_limit_streak,tracking_loss_count,tracking_loss_streak,recovery_energy_error_jpkg,recovery_target_deviation_elevator,recovery_target_deviation_throttle,command_deviation_elevator,command_deviation_throttle | Format-List
    Write-Output "--- recovery transitions (first 30 rows with recovery/authority activity) ---"
    $t | Where-Object { ([double]$_.recovery_mode -gt 0) -or ([double]$_.authority_limit_streak -gt 0) -or ([double]$_.tracking_loss_streak -gt 0) -or ([double]$_.mpc_gate_reject_count -gt 0) } | Select-Object -First 40 time_s,cfg_id,actual_h_m,actual_v_mps,actual_vz_mps,vz_ref_mps,h_error_m,pitch_deg,q_dps,physical_elevator_cmd,physical_throttle_cmd,recovery_mode,recovery_reason_code,tracking_loss_streak,authority_limit_streak,recovery_energy_error_jpkg,recovery_target_deviation_elevator,recovery_target_deviation_throttle,mpc_gate_reject_count | Format-Table -AutoSize
}
