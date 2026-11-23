# Builds and runs the host test suite (fusion library in simulation) and the Python
# calibration tests. Needs any host C compiler; on Windows, Visual Studio is found
# automatically by CMake.
param(
    [switch]$SkipPython
)
$ErrorActionPreference = 'Stop'
$projectPath = $PSScriptRoot
$picoTools = Join-Path $env:USERPROFILE '.pico-sdk'
$cmakeExe = $null
foreach ($candidate in @((Join-Path $picoTools 'cmake/v4.3.4/bin/cmake.exe'),
                         (Join-Path $picoTools 'cmake/v3.31.5/bin/cmake.exe'))) {
    if (Test-Path -LiteralPath $candidate) { $cmakeExe = $candidate; break }
}
if (-not $cmakeExe) { $cmakeExe = (Get-Command cmake).Source }
$ctestExe = Join-Path (Split-Path -Parent $cmakeExe) 'ctest.exe'
if (-not (Test-Path -LiteralPath $ctestExe)) { $ctestExe = (Get-Command ctest).Source }

# An ASCII build directory outside the project (some tools mishandle non-ASCII paths)
# and outside TEMP (MSBuild warns about incremental builds there).
$hashProvider = [Security.Cryptography.SHA256]::Create()
$pathHash = [BitConverter]::ToString($hashProvider.ComputeHash([Text.Encoding]::UTF8.GetBytes($projectPath))).Replace('-','').Substring(0,12)
$hashProvider.Dispose()
$buildPath = Join-Path $env:LOCALAPPDATA "ins10-build\host-$pathHash"

& $cmakeExe -S (Join-Path $projectPath 'tests') -B $buildPath
if ($LASTEXITCODE -ne 0) { throw 'CMake configure failed. Is a host C compiler installed?' }
& $cmakeExe --build $buildPath --config Release --parallel
if ($LASTEXITCODE -ne 0) { throw 'Host build failed.' }
& $ctestExe --test-dir $buildPath -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'Fusion tests failed.' }

if (-not $SkipPython) {
    & python (Join-Path $projectPath 'tools/tests/test_calib.py')
    if ($LASTEXITCODE -ne 0) { throw 'Calibration tool tests failed.' }
}
Write-Output "Host build directory: $buildPath"
