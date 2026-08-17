param([switch]$Watch)
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$Root = Join-Path $ProjectRoot 'matlab\results\mpc_auto_v32_clean'
do {
    Clear-Host
    Write-Host '=== AirdropX v32.1 persistent-memory status ==='
    Get-Date -Format 'yyyy-MM-dd HH:mm:ss'
    Write-Host "`nMATLAB processes:"
    Get-Process MATLAB -ErrorAction SilentlyContinue | Select-Object Id,@{N='CPU_s';E={[math]::Round($_.CPU,1)}},@{N='RAM_GB';E={[math]::Round($_.WorkingSet64/1GB,2)}} | Format-Table -AutoSize
    $memory = Join-Path $Root 'v32_memory_status.csv'
    Write-Host "`nMemory:"
    if (Test-Path $memory) { Get-Content $memory } else { Write-Host '(no v32 memory yet)' }
    $status = Join-Path $Root 'v32_status.csv'
    Write-Host "`nCurrent status:"
    if (Test-Path $status) { Get-Content $status } else { Write-Host '(not started)' }
    $hist = Join-Path $Root 'v32_stage_history.csv'
    Write-Host "`nRecent stages:"
    if (Test-Path $hist) { Import-Csv $hist | Select-Object -Last 10 | Format-Table -AutoSize }
    Write-Host "`nRecent files:"
    if (Test-Path $Root) {
        Get-ChildItem $Root -Recurse -File -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending | Select-Object -First 18 | ForEach-Object {
            '{0} {1,10} {2}' -f $_.LastWriteTime.ToString('HH:mm:ss'),$_.Length,$_.FullName
        }
    }
    if ($Watch) { Start-Sleep -Seconds 5 }
} while ($Watch)
