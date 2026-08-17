param(
    [string]$ProjectRoot = "D:\vscode project\AirdropX"
)
$root = Join-Path $ProjectRoot 'matlab/results/mpc_auto_v31/dynamic_reference_validation/cfg0_height_trace_v31_3_1'
$sum = Join-Path $root 'reference_path_diagnostic_summary.csv'
$probe = Join-Path $root 'reference_path_transition_probe.csv'
$trace = Join-Path $root 'simulation/controller_reference_trace.csv'
Write-Output "=== v31.3.1 reference-path diagnostic ==="
if (Test-Path $sum) { Import-Csv $sum | Format-List * } else { Write-Output "summary missing: $sum" }
Write-Output "=== transition probes ==="
if (Test-Path $probe) {
    Import-Csv $probe | Select-Object command_time_s,probe_offset_s,internal_requested_H_m,actual_H_m,height_error_m,height_bias_mps,raw_vz_ref_mps,limited_vz_ref_mps,slew_vz_ref_mps,actual_vz_mps,trust_ok,scheduler_enabled | Format-Table -AutoSize
} else { Write-Output "probe missing: $probe" }
Write-Output "=== internal trace ==="
if (Test-Path $trace) { Write-Output $trace; Write-Output ((Import-Csv $trace).Count.ToString() + ' rows') } else { Write-Output "trace missing: $trace" }
