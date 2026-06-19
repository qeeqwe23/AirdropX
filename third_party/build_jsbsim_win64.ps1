param(
    [string]$Config = "Release",
    [string]$Generator = "",
    [string]$BuildDirName = "JSBSim-build"
)

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$sourceDir = Join-Path $root "JSBSim"
$buildDir = Join-Path $root $BuildDirName
$installDir = Join-Path $root "jsbsim-win64"

if (-not (Test-Path -LiteralPath $sourceDir)) {
    throw "JSBSim source directory not found: $sourceDir"
}

$configureArgs = @(
    "-S", $sourceDir,
    "-B", $buildDir,
    "-DCMAKE_INSTALL_PREFIX=$installDir",
    "-DCMAKE_BUILD_TYPE=$Config",
    "-DBUILD_SHARED_LIBS=OFF"
)

if ($Generator.Trim().Length -gt 0) {
    $configureArgs = @("-G", $Generator) + $configureArgs
}

cmake @configureArgs
if ($LASTEXITCODE -ne 0) {
    throw "CMake configure failed with exit code $LASTEXITCODE"
}
cmake --build $buildDir --config $Config
if ($LASTEXITCODE -ne 0) {
    throw "CMake build failed with exit code $LASTEXITCODE"
}
cmake --install $buildDir --config $Config
if ($LASTEXITCODE -ne 0) {
    throw "CMake install failed with exit code $LASTEXITCODE"
}

Write-Host "Installed JSBSim development files to $installDir"
