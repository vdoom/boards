# Builds INS10 Studio (Release) and collects it with the Qt libraries it needs into a
# folder that runs on a PC without Qt, then starts the copy once with a bare PATH
# (simulated board, closes itself) to prove it.
#
#   ./deploy.ps1                       # MinGW kit -> %LOCALAPPDATA%\ins10-build\deploy-mingw\INS10 Studio
#   ./deploy.ps1 -Kit msvc -Output D:\INS10Studio
param(
    [ValidateSet('mingw', 'msvc')][string]$Kit = 'mingw',
    [string]$Output = '',
    [string]$QtRoot = $(if ($env:INS10_QT_ROOT) { $env:INS10_QT_ROOT } else { 'C:\Qt\6.12.0' }),
    [string]$QtTools = $(if ($env:INS10_QT_TOOLS) { $env:INS10_QT_TOOLS } else { 'C:\Qt\Tools' }),
    [switch]$NoBuild,
    [switch]$NoCheck
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'scripts/kit.ps1')

if (-not $NoBuild) {
    & (Join-Path $PSScriptRoot 'build.ps1') -Kit $Kit -Config Release -QtRoot $QtRoot -QtTools $QtTools
}
$kitInfo = Get-Ins10Kit -Kit $Kit -QtRoot $QtRoot -QtTools $QtTools
$exe = Join-Path $kitInfo.BuildDir 'bin/ins10studio.exe'
if ($kitInfo.MultiConfig) { $exe = Join-Path $kitInfo.BuildDir 'bin/Release/ins10studio.exe' }
if (-not (Test-Path -LiteralPath $exe)) { throw "Not built: $exe" }
if (-not $Output) { $Output = Join-Path (Split-Path -Parent $kitInfo.BuildDir) "deploy-$Kit\INS10 Studio" }

if (Test-Path -LiteralPath $Output) { Remove-Item -LiteralPath $Output -Recurse -Force }
New-Item -ItemType Directory -Force -Path $Output | Out-Null
Copy-Item -LiteralPath $exe -Destination $Output
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'README.md') -Destination $Output

# windeployqt finds the compiler runtime (MinGW) through PATH.
$savedPath = $env:PATH
$env:PATH = ($kitInfo.Path + @($env:PATH)) -join ';'
$deployArgs = @('--release', '--no-system-d3d-compiler', '--no-quick-import')
if ($Kit -eq 'mingw') { $deployArgs += '--compiler-runtime' }
& (Join-Path $kitInfo.QtBin 'windeployqt.exe') @deployArgs (Join-Path $Output 'ins10studio.exe')
$code = $LASTEXITCODE
$env:PATH = $savedPath
if ($code -ne 0) { throw 'windeployqt failed.' }
Write-Output "Deployed to: $Output"

if (-not $NoCheck) {
    # Nothing of Qt or the compiler on PATH: only what lies in the folder.
    $bare = "$env:SystemRoot\System32;$env:SystemRoot;$env:SystemRoot\System32\Wbem"
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = Join-Path $Output 'ins10studio.exe'
    $psi.Arguments = '--demo --quit-after 6'
    $psi.UseShellExecute = $false
    $psi.EnvironmentVariables['PATH'] = $bare
    foreach ($name in @('QT_PLUGIN_PATH', 'QT_QPA_PLATFORM_PLUGIN_PATH', 'QML2_IMPORT_PATH', 'QT_QPA_PLATFORM')) {
        $psi.EnvironmentVariables.Remove($name)
    }
    $process = [System.Diagnostics.Process]::Start($psi)
    if (-not $process.WaitForExit(60000)) {
        $process.Kill()
        throw 'The deployed application did not close by itself.'
    }
    if ($process.ExitCode -ne 0) {
        throw ('The deployed application failed to run without Qt on PATH (exit code 0x{0:X}).' -f $process.ExitCode)
    }
    Write-Output 'Check passed: the deployed folder runs without Qt on PATH.'
}
