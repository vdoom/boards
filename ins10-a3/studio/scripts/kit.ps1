# Shared by build.ps1, test.ps1 and deploy.ps1: where the compiler, CMake and Qt of a
# kit live on this machine.
function Find-Ins10CMake([string[]]$Candidates) {
    foreach ($candidate in $Candidates) {
        if ($candidate -and (Test-Path -LiteralPath $candidate)) { return $candidate }
    }
    $fromPath = Get-Command cmake -ErrorAction SilentlyContinue
    if ($fromPath) { return $fromPath.Source }
    throw 'CMake not found.'
}

function Get-Ins10Kit([string]$Kit, [string]$QtRoot, [string]$QtTools) {
    $buildRoot = Join-Path $env:LOCALAPPDATA 'ins10-build'
    $picoCMake = Join-Path $env:USERPROFILE '.pico-sdk/cmake/v4.3.4/bin/cmake.exe'
    if ($Kit -eq 'mingw') {
        $qtPrefix = Join-Path $QtRoot 'mingw_64'
        # Qt's bundled CMake 3.24 is too old for Qt 6.12; the system CMake (3.28) and
        # the Pico SDK's CMake (4.3) both work with Ninja.
        $cmake = Find-Ins10CMake @('C:\Program Files\CMake\bin\cmake.exe', $picoCMake)
        $path = @((Join-Path $QtTools 'mingw1310_64\bin'), (Join-Path $QtTools 'Ninja'), (Join-Path $qtPrefix 'bin'))
        $configure = @('-G', 'Ninja', '-DCMAKE_CXX_COMPILER=g++', '-DCMAKE_C_COMPILER=gcc')
        $multi = $false
    } else {
        $qtPrefix = Join-Path $QtRoot 'msvc2022_64'
        # Only CMake 4.x knows Visual Studio 2026; it picks the generator by itself.
        $cmake = Find-Ins10CMake @($picoCMake)
        $path = @((Join-Path $qtPrefix 'bin'))
        $configure = @()
        $multi = $true
    }
    if (-not (Test-Path -LiteralPath $qtPrefix)) { throw "Qt kit not found: $qtPrefix" }
    $ctest = Join-Path (Split-Path -Parent $cmake) 'ctest.exe'
    [pscustomobject]@{
        Kit = $Kit
        CMake = $cmake
        CTest = $ctest
        QtPrefix = $qtPrefix
        QtBin = (Join-Path $qtPrefix 'bin')
        Path = $path
        ConfigureArgs = $configure
        MultiConfig = $multi
        BuildDir = (Join-Path $buildRoot "studio-$Kit")
    }
}
