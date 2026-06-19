param(
    [string]$OutputZip = ""
)

$ErrorActionPreference = "Stop"

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
if ([string]::IsNullOrWhiteSpace($OutputZip)) {
    $OutputZip = Join-Path $projectRoot "AirdropX_matlab_release.zip"
}

$stageRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("AirdropX_release_" + [guid]::NewGuid().ToString("N"))
$stageProject = Join-Path $stageRoot "AirdropX"

$excludeDirs = @(
    ".git",
    "__pycache__",
    "slprj",
    "matlab\slprj",
    "matlab\results",
    "matlab\outputs",
    "matlab_tmp",
    "matlab_clean_start"
)

$excludeFiles = @(
    "*.pyc",
    "*.pyo",
    "*.bak",
    "*.orig",
    "*.tmp",
    "*.log",
    "*.zip",
    "*.slxc",
    "*.slx.autosave",
    "Thumbs.db",
    ".DS_Store"
)

function Remove-StagedPath {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) {
        return
    }
    Get-ChildItem -LiteralPath $Path -Recurse -Force -ErrorAction SilentlyContinue | ForEach-Object {
        try {
            $_.Attributes = "Normal"
        } catch {
        }
    }
    Remove-Item -LiteralPath $Path -Recurse -Force
}

function Test-ExcludedFile {
    param([string]$Name)
    foreach ($pattern in $excludeFiles) {
        if ($Name -like $pattern) {
            return $true
        }
    }
    return $false
}

function Test-ExcludedDir {
    param([string]$Name, [string]$RelativePath)
    $norm = $RelativePath -replace '/', '\'
    if ($excludeDirs -contains $Name) {
        return $true
    }
    if ($excludeDirs -contains $norm) {
        return $true
    }
    return $false
}

function Copy-FilteredTree {
    param(
        [string]$SourceDir,
        [string]$DestinationDir,
        [string]$RelativePath = ""
    )

    if (-not (Test-Path -LiteralPath $DestinationDir)) {
        New-Item -ItemType Directory -Path $DestinationDir | Out-Null
    }

    Get-ChildItem -LiteralPath $SourceDir -Force | ForEach-Object {
        $childRel = $_.Name
        if (-not [string]::IsNullOrWhiteSpace($RelativePath)) {
            $childRel = Join-Path $RelativePath $_.Name
        }

        if ($_.PSIsContainer) {
            if (Test-ExcludedDir -Name $_.Name -RelativePath $childRel) {
                return
            }
            Copy-FilteredTree -SourceDir $_.FullName -DestinationDir (Join-Path $DestinationDir $_.Name) -RelativePath $childRel
        } else {
            if (Test-ExcludedFile -Name $_.Name) {
                return
            }
            Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $DestinationDir $_.Name) -Force
        }
    }
}

New-Item -ItemType Directory -Path $stageProject | Out-Null

Copy-FilteredTree -SourceDir $projectRoot -DestinationDir $stageProject

if (Test-Path -LiteralPath $OutputZip) {
    Remove-Item -LiteralPath $OutputZip -Force
}

Compress-Archive -LiteralPath $stageProject -DestinationPath $OutputZip -Force
try {
    Remove-StagedPath $stageRoot
} catch {
    Write-Warning "Package was created, but temporary cleanup failed: $stageRoot"
}

Write-Host "Created release package:"
Write-Host "  $OutputZip"
