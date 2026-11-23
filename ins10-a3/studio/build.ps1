# Configures and builds INS10 Studio with one of the Qt kits on Windows.
#
#   ./build.ps1                  # MinGW + Ninja, Release
#   ./build.ps1 -Kit msvc        # Visual Studio 2026 + Qt msvc2022_64
#   ./build.ps1 -Run             # build, then start the application
#   ./build.ps1 -Run -RunArgs --demo
#
# The build directory is %LOCALAPPDATA%\ins10-build\studio-<kit>: outside the source
# tree (which lies in OneDrive and has non-ASCII characters in its path) and outside
# %TEMP% (MSBuild warns about incremental builds there).
param(
    [ValidateSet('mingw', 'msvc')][string]$Kit = 'mingw',
    [ValidateSet('Release', 'Debug', 'RelWithDebInfo')][string]$Config = 'Release',
    [string]$QtRoot = $(if ($env:INS10_QT_ROOT) { $env:INS10_QT_ROOT } else { 'C:\Qt\6.12.0' }),
    [string]$QtTools = $(if ($env:INS10_QT_TOOLS) { $env:INS10_QT_TOOLS } else { 'C:\Qt\Tools' }),
    [switch]$Clean,
    [switch]$Run,
    [string[]]$RunArgs = @()
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'scripts/kit.ps1')

$kitInfo = Get-Ins10Kit -Kit $Kit -QtRoot $QtRoot -QtTools $QtTools
$env:PATH = ($kitInfo.Path + @($env:PATH)) -join ';'

if ($Clean -and (Test-Path -LiteralPath $kitInfo.BuildDir)) {
    Remove-Item -LiteralPath $kitInfo.BuildDir -Recurse -Force
}

$configureArgs = @('-S', $PSScriptRoot, '-B', $kitInfo.BuildDir, "-DCMAKE_PREFIX_PATH=$($kitInfo.QtPrefix)")
$configureArgs += $kitInfo.ConfigureArgs
if (-not $kitInfo.MultiConfig) { $configureArgs += "-DCMAKE_BUILD_TYPE=$Config" }
# A copy of the Eigen archive kept next to the build folders saves a download per kit;
# CMake still checks its SHA-256.
$eigenArchive = Join-Path $env:LOCALAPPDATA 'ins10-build/downloads/eigen-3.4.0.tar.gz'
if (Test-Path -LiteralPath $eigenArchive) { $configureArgs += "-DINS10_EIGEN_URL=$($eigenArchive -replace '\\', '/')" }
& $kitInfo.CMake @configureArgs
if ($LASTEXITCODE -ne 0) { throw 'CMake configure failed.' }

& $kitInfo.CMake --build $kitInfo.BuildDir --config $Config --parallel
if ($LASTEXITCODE -ne 0) { throw 'Build failed.' }

$exe = Join-Path $kitInfo.BuildDir 'bin/ins10studio.exe'
if ($kitInfo.MultiConfig) {
    $candidate = Join-Path $kitInfo.BuildDir "bin/$Config/ins10studio.exe"
    if (Test-Path -LiteralPath $candidate) { $exe = $candidate }
}
Write-Output "Built: $exe"
if ($Run) { & $exe @RunArgs }
