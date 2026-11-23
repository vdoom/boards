# Builds INS10 Studio and runs its test suite.
#
#   ./test.ps1                   # MinGW kit
#   ./test.ps1 -Kit msvc
#   ./test.ps1 -Filter calib     # only tests whose name matches the regular expression
#
# Widget tests run on the "offscreen" platform; the ones that need an OpenGL 3.3
# context skip themselves there.
param(
    [ValidateSet('mingw', 'msvc')][string]$Kit = 'mingw',
    [ValidateSet('Release', 'Debug', 'RelWithDebInfo')][string]$Config = 'Release',
    [string]$QtRoot = $(if ($env:INS10_QT_ROOT) { $env:INS10_QT_ROOT } else { 'C:\Qt\6.12.0' }),
    [string]$QtTools = $(if ($env:INS10_QT_TOOLS) { $env:INS10_QT_TOOLS } else { 'C:\Qt\Tools' }),
    [string]$Filter = '',
    [switch]$NoBuild
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'scripts/kit.ps1')

if (-not $NoBuild) {
    & (Join-Path $PSScriptRoot 'build.ps1') -Kit $Kit -Config $Config -QtRoot $QtRoot -QtTools $QtTools
}
$kitInfo = Get-Ins10Kit -Kit $Kit -QtRoot $QtRoot -QtTools $QtTools
$env:PATH = ($kitInfo.Path + @($env:PATH)) -join ';'
$env:QT_QPA_PLATFORM = 'offscreen'
$env:QT_FORCE_STDERR_LOGGING = '1'   # Qt Test output to the console, not the debugger

$ctestArgs = @('--test-dir', $kitInfo.BuildDir, '-C', $Config, '--output-on-failure')
if ($Filter) { $ctestArgs += @('-R', $Filter) }
& $kitInfo.CTest @ctestArgs
if ($LASTEXITCODE -ne 0) { throw 'Tests failed.' }
