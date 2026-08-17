param(
    [string]$ProjectRoot = "D:\vscode project\AirdropX",
    [switch]$Watch,
    [int]$RefreshSeconds = 3
)

$ErrorActionPreference = "SilentlyContinue"
$root = Join-Path $ProjectRoot "matlab\results\mpc_auto_v31"

function Show-V31Once {
    Clear-Host
    Write-Host "===== MATLAB ====="
    Get-Process MATLAB -ErrorAction SilentlyContinue |
        Select-Object Id,@{N='CPU_s';E={[math]::Round($_.CPU,1)}},@{N='RAM_GB';E={[math]::Round($_.WorkingSet64/1GB,2)}} |
        Format-Table -AutoSize

    Write-Host "`n===== V31.3 ENVELOPE ====="
    $status = Join-Path $root "v31_envelope_status.csv"
    if (Test-Path -LiteralPath $status) { Import-Csv $status | Format-List } else { Write-Host "No v31_envelope_status.csv yet." }

    Write-Host "`n===== CURRICULUM ====="
    $curr = Join-Path $root "v31_curriculum.csv"
    if (Test-Path -LiteralPath $curr) {
        Import-Csv $curr | Select-Object updated_at,role,stage,kind,target_altitude_m,target_airspeed_mps,completed,pass,gate_ratio,failure_class | Format-Table -AutoSize
    } else { Write-Host "No v31_curriculum.csv yet." }

    Write-Host "`n===== ACTIVE V31 CONTEXT STATE ====="
    $states = Get-ChildItem $root -Recurse -Filter mission_state.csv -File | Sort-Object LastWriteTime -Descending
    if ($states) {
        $f = $states[0]
        Write-Host $f.FullName
        Import-Csv $f.FullName | Format-List
    } else { Write-Host "No mission_state.csv yet." }


    Write-Host "`n===== HEIGHT GOVERNOR REFINEMENT ====="
    $rounds = Get-ChildItem $root -Recurse -Filter rounds.csv -File | Where-Object { $_.FullName -like '*height_governor_refinement*' } | Sort-Object LastWriteTime -Descending
    if ($rounds) {
        Write-Host $rounds[0].FullName
        Import-Csv $rounds[0].FullName | Select-Object -Last 5 | Format-Table -AutoSize
    } else { Write-Host "No height-governor refinement round yet." }


    Write-Host "`n===== V31.3 SPEED SCHEDULER ====="
    $sched = Join-Path $root "v31_3_speed_scheduler\v31_3_scheduler_nodes.csv"
    if (Test-Path -LiteralPath $sched) { Import-Csv $sched | Format-Table -AutoSize } else { Write-Host "No verified multi-speed scheduler nodes yet." }

    Write-Host "`n===== V31.3 DYNAMIC REFERENCE ====="
    $dyn = Get-ChildItem $root -Recurse -Filter dynamic_reference_summary.csv -File | Sort-Object LastWriteTime -Descending
    if ($dyn) { Write-Host $dyn[0].FullName; Import-Csv $dyn[0].FullName | Format-List } else { Write-Host "No dynamic H/V validation result yet." }

    Write-Host "`n===== LATEST FILES ====="
    Get-ChildItem $root -Recurse -File | Sort-Object LastWriteTime -Descending | Select-Object -First 12 |
        ForEach-Object { "{0}  {1,10}  {2}" -f $_.LastWriteTime.ToString("HH:mm:ss"),$_.Length,$_.FullName }
}

if ($Watch) {
    while ($true) { Show-V31Once; Start-Sleep -Seconds ([Math]::Max(1,$RefreshSeconds)) }
} else {
    Show-V31Once
}
