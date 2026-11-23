# Rebuilds the calibration jig from calibration_jig.py: the two STLs (base and
# lid), STEP, FCStd and preview.png next to this script, after checking that the
# board, the Pico, the plug, the screws and the wires fit.
#   ./build.ps1                     uses FreeCAD 1.1 or 1.0 from Program Files
#   ./build.ps1 -FreeCad <path>     another freecadcmd.exe
param([string]$FreeCad)
$ErrorActionPreference = 'Stop'

if (-not $FreeCad) {
    $FreeCad = @($env:FREECADCMD,
                 'C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe',
                 'C:\Program Files\FreeCAD 1.0\bin\freecadcmd.exe') |
        Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
}
if (-not $FreeCad) { throw 'freecadcmd.exe not found; pass -FreeCad <path>' }

$here = $PSScriptRoot
$stl = Join-Path $here 'calibration-jig-lid.stl'
$start = Get-Date
# freecadcmd garbles non-ASCII characters in its arguments, so run it from this
# folder with a relative script name. It imports the script as a module, hence
# the __pycache__ to remove afterwards.
Push-Location $here
try {
    $log = & $FreeCad 'calibration_jig.py' 2>&1 | ForEach-Object { "$_" }
} finally {
    Pop-Location
    Remove-Item (Join-Path $here '__pycache__') -Recurse -Force -ErrorAction SilentlyContinue
}
# FreeCAD prints Python output to both streams and adds progress and STEP banners.
$log | Where-Object { $_ -notmatch '^\s*(\(\d+ %\))?\s*$|^\*+|^\s*\*\*|Step File Name|^(saving|Recompute|Checking)|^FreeCAD |^\(C\) 2001' } |
    Select-Object -Unique | Out-Host

if ($log -match 'Exception while processing|CHECKS FAILED|PROBLEM') { throw 'the jig did not build; see above' }
if (-not (Test-Path $stl) -or (Get-Item $stl).LastWriteTime -lt $start) { throw 'calibration-jig-lid.stl was not written' }
