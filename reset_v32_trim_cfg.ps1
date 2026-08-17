param(
    [double]$Speed = 55.0,
    [ValidateRange(0,4)][int]$ConfigId = 4,
    [switch]$Delete
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$resultsRoot = Join-Path $projectRoot 'matlab\results\mpc_auto_v32_clean'
$nodeName = 'V' + $Speed.ToString('F3',[System.Globalization.CultureInfo]::InvariantCulture)
$cfgPath = Join-Path $resultsRoot ("knowledge_bank\physics\{0}\trim\cfg{1}" -f $nodeName,$ConfigId)
$physicsVerified = Join-Path $resultsRoot ("knowledge_bank\physics\{0}\v32_physics_verified.mat" -f $nodeName)

Write-Host "Target trim state: $cfgPath"
if (Test-Path $physicsVerified) {
    throw "Refusing surgical cfg reset because this speed node is already PHYSICS VERIFIED: $physicsVerified. Use -ResetLearning only if you intentionally want to invalidate the verified node."
}
if (-not (Test-Path $cfgPath)) {
    Write-Host 'Nothing to reset; cfg trim directory does not exist.'
    exit 0
}

if ($Delete) {
    Remove-Item -LiteralPath $cfgPath -Recurse -Force
    Write-Host "Deleted only $nodeName cfg$ConfigId trim search/checkpoint data. Other v32 memory was preserved."
} else {
    $stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
    $archive = "${cfgPath}_quarantine_${stamp}"
    Move-Item -LiteralPath $cfgPath -Destination $archive
    Write-Host "Quarantined bad trim data to: $archive"
    Write-Host 'Re-run v32 normally; this cfg will start from the verified configuration-continuation branch.'
}
