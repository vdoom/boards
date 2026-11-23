# Builds the INS10 AHRS firmware for the selected board.
param(
    [ValidateSet('pico','pico2','both')][string]$Board = 'pico',
    [string]$SdkPath = $env:PICO_SDK_PATH
)
$ErrorActionPreference = 'Stop'
$projectPath = $PSScriptRoot
$picoTools = Join-Path $env:USERPROFILE '.pico-sdk'
if (-not $SdkPath) { $SdkPath = Join-Path $picoTools 'sdk/2.2.0' }
$SdkPath = (Resolve-Path -LiteralPath $SdkPath).Path
$cmakeExe = Join-Path $picoTools 'cmake/v3.31.5/bin/cmake.exe'
$ninjaExe = Join-Path $picoTools 'ninja/v1.12.1/ninja.exe'
if (-not (Test-Path -LiteralPath $cmakeExe)) { $cmakeExe = (Get-Command cmake).Source }
if (-not (Test-Path -LiteralPath $ninjaExe)) { $ninjaExe = (Get-Command ninja).Source }

$toolchainPath = Join-Path $picoTools 'toolchain/14_2_Rel1'
$picotoolPath = Join-Path $picoTools 'picotool/2.2.0-a4/picotool'
$buildArgs = @("-DPICO_SDK_PATH=$SdkPath", "-DCMAKE_MAKE_PROGRAM=$ninjaExe", '-DCMAKE_BUILD_TYPE=Release')
if (Test-Path -LiteralPath (Join-Path $toolchainPath 'bin/arm-none-eabi-gcc.exe')) {
    $buildArgs += "-DPICO_TOOLCHAIN_PATH=$toolchainPath"
} else {
    $toolchainPath = $env:PICO_TOOLCHAIN_PATH
}
if (Test-Path -LiteralPath (Join-Path $picotoolPath 'picotoolConfig.cmake')) {
    $buildArgs += "-Dpicotool_DIR=$picotoolPath"
}

# An ASCII build directory avoids GNU ld/objcopy failures on Unicode output paths.
# The SDK and toolchain are part of the key: a CMake cache from another SDK cannot be reused.
$buildKey = "$projectPath|$SdkPath|$toolchainPath"
$hashProvider = [Security.Cryptography.SHA256]::Create()
$pathHash = [BitConverter]::ToString($hashProvider.ComputeHash([Text.Encoding]::UTF8.GetBytes($buildKey))).Replace('-','').Substring(0,12)
$hashProvider.Dispose()
$buildRoot = Join-Path $env:TEMP "ins10-ahrs-$pathHash"
if ($buildRoot -match '[^\x00-\x7F]') { throw 'Set TEMP to an ASCII-only path for the build.' }
$targetBoards = if ($Board -eq 'both') { @('pico', 'pico2') } else { @($Board) }
foreach ($targetBoard in $targetBoards) {
    $buildPath = Join-Path $buildRoot $targetBoard
    & $cmakeExe -S $projectPath -B $buildPath -G Ninja "-DPICO_BOARD=$targetBoard" @buildArgs
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed for $targetBoard." }
    & $cmakeExe --build $buildPath --parallel
    if ($LASTEXITCODE -ne 0) { throw "Build failed for $targetBoard." }
    $outputPath = Join-Path $projectPath "build/$targetBoard"
    New-Item -ItemType Directory -Force -Path $outputPath | Out-Null
    foreach ($artifact in @('ins10_ahrs.uf2', 'ins10_ahrs.elf')) {
        Copy-Item -LiteralPath (Join-Path $buildPath $artifact) -Destination $outputPath -Force
    }
    Write-Output (Join-Path $outputPath 'ins10_ahrs.uf2')
}
