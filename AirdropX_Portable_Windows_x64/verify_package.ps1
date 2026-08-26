$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$manifest = Join-Path $root 'PACKAGE_MANIFEST.sha256'
if (-not (Test-Path $manifest)) { throw "Missing manifest: $manifest" }
$bad = 0
$total = 0
$encoding = New-Object System.Text.UTF8Encoding($true)
foreach ($line in [System.IO.File]::ReadLines($manifest, $encoding)) {
    if ([string]::IsNullOrWhiteSpace($line)) { continue }
    if ($line -notmatch '^([0-9A-Fa-f]{64})\s\s(.+)$') { throw "Bad manifest line: $line" }
    $expected = $Matches[1].ToUpperInvariant()
    $rel = $Matches[2]
    $path = Join-Path $root $rel
    $total++
    if (-not (Test-Path -LiteralPath $path)) {
        Write-Host "MISSING  $rel" -ForegroundColor Red
        $bad++
        continue
    }
    $hashObj = Get-FileHash -Algorithm SHA256 -LiteralPath $path
    if ($null -eq $hashObj) {
        Write-Host "HASHERR  $rel" -ForegroundColor Red
        $bad++
        continue
    }
    $actual = $hashObj.Hash.ToUpperInvariant()
    if ($actual -ne $expected) {
        Write-Host "MISMATCH $rel" -ForegroundColor Red
        $bad++
    }
}
if ($bad -eq 0) {
    Write-Host "PACKAGE VERIFY PASS: $total files" -ForegroundColor Green
} else {
    throw "PACKAGE VERIFY FAIL: $bad / $total files mismatched"
}