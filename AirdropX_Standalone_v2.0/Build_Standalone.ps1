param(
    [string]$ProjectRoot = "",
    [switch]$SkipPip
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$AppRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($ProjectRoot)) {
    $parent = Split-Path -Parent $AppRoot
    if (Test-Path (Join-Path $parent 'aircraft')) { $ProjectRoot = $parent }
    elseif (Test-Path 'D:\vscode project\AirdropX\aircraft') { $ProjectRoot = 'D:\vscode project\AirdropX' }
    else { throw 'Cannot locate AirdropX project root. Re-run with -ProjectRoot "D:\...\AirdropX".' }
}
$ProjectRoot = (Resolve-Path $ProjectRoot).Path

function Invoke-Checked {
    param([string]$Exe, [string[]]$CommandArgs)
    Write-Host ("> " + $Exe + " " + ($CommandArgs -join ' ')) -ForegroundColor DarkGray
    & $Exe @CommandArgs
    $code = $LASTEXITCODE
    if ($null -eq $code) { $code = 0 }
    if ($code -ne 0) { throw "Command failed with exit code ${code}: $Exe" }
}

function Find-Python312 {
    $candidates = @(
        (Join-Path $ProjectRoot 'offline_gui_v140\.venv\Scripts\python.exe'),
        (Join-Path $ProjectRoot 'offline_gui_v136p\.venv\Scripts\python.exe'),
        (Join-Path $ProjectRoot 'AirdropX_Software\.venv\Scripts\python.exe'),
        (Join-Path $env:LOCALAPPDATA 'Programs\Python\Python312\python.exe')
    )
    foreach ($p in $candidates) { if ($p -and (Test-Path $p)) { return $p } }
    $py = Get-Command py.exe -ErrorAction SilentlyContinue
    if ($py) { return $py.Source }
    $python = Get-Command python.exe -ErrorAction SilentlyContinue
    if ($python -and $python.Source -notmatch 'WindowsApps') { return $python.Source }
    throw 'Python 3.12 was not found.'
}

Write-Host '=== AirdropX Standalone v2.0 build ===' -ForegroundColor Cyan
Write-Host "ProjectRoot: $ProjectRoot"
Write-Host "AppRoot:     $AppRoot"

$basePython = Find-Python312
$venv = Join-Path $AppRoot '.venv_standalone'
$venvPython = Join-Path $venv 'Scripts\python.exe'
if (-not (Test-Path $venvPython)) {
    if ((Split-Path -Leaf $basePython) -ieq 'py.exe') {
        Invoke-Checked $basePython @('-3.12','-m','venv',$venv)
    } else {
        Invoke-Checked $basePython @('-m','venv',$venv)
    }
}
if (-not (Test-Path $venvPython)) { throw "Failed to create venv: $venv" }
Invoke-Checked $venvPython @('-c','import sys; assert sys.version_info[:2] == (3,12), sys.version; print(sys.version)')

if (-not $SkipPip) {
    Invoke-Checked $venvPython @('-m','pip','install','--upgrade','pip')
    Invoke-Checked $venvPython @('-m','pip','install','-r',(Join-Path $AppRoot 'requirements.txt'))
}

# Convert the already-generated Physics-MPC bank into runtime-only NPZ and copy JSBSim model assets.
Invoke-Checked $venvPython @((Join-Path $AppRoot 'tools\prepare_runtime_assets.py'),'--project-root',$ProjectRoot,'--app-root',$AppRoot)

# Syntax and numerical/static regression checks run before the real plant smoke, so
# ordinary packaging mistakes fail with a clear message instead of masquerading as JSBSim errors.
Invoke-Checked $venvPython @('-m','compileall','-q',(Join-Path $AppRoot 'core'),(Join-Path $AppRoot 'ui'),(Join-Path $AppRoot 'tools'),(Join-Path $AppRoot 'main.py'))
Invoke-Checked $venvPython @('-m','unittest','discover','-s',(Join-Path $AppRoot 'tests'),'-p','test_*.py','-v')

# Real JSBSim + controller smoke runs before freezing. If the Python/native runtime does not
# match the expected aircraft/mass/wind semantics, no executable is produced.
Invoke-Checked $venvPython @((Join-Path $AppRoot 'tools\smoke_runtime.py'),'--app-root',$AppRoot)

$dist = Join-Path $AppRoot 'dist'
$build = Join-Path $AppRoot 'build'
$spec = Join-Path $AppRoot 'AirdropX.spec'
if (Test-Path $dist) { Remove-Item -Recurse -Force $dist }
if (Test-Path $build) { Remove-Item -Recurse -Force $build }
if (Test-Path $spec) { Remove-Item -Force $spec }

Invoke-Checked $venvPython @(
    '-m','PyInstaller','--noconfirm','--clean','--windowed',
    '--name','AirdropX',
    '--distpath',$dist,
    '--workpath',$build,
    '--specpath',$AppRoot,
    '--collect-all','jsbsim',
    '--add-data',((Join-Path $AppRoot 'assets') + ';assets'),
    '--hidden-import','scipy.optimize',
    (Join-Path $AppRoot 'main.py')
)

$exe = Join-Path $dist 'AirdropX\AirdropX.exe'
if (-not (Test-Path $exe)) { throw "Build completed without expected executable: $exe" }
Invoke-Checked $exe @('--smoke-runtime')
Write-Host '[exe] frozen runtime smoke PASS' -ForegroundColor Green
Write-Host ''
Write-Host 'BUILD PASS' -ForegroundColor Green
Write-Host "Standalone EXE: $exe"
Write-Host 'The dist\AirdropX folder can run without MATLAB and without the source project.'



