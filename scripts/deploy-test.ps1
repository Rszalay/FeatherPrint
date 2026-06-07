# Quick test deploy - copies Release build directly into Cura without the installer.
# Run from the repo root: .\scripts\deploy-test.ps1
# To restore originals: .\scripts\deploy-test.ps1 -Restore
# Requires admin rights (will self-elevate if needed).

param(
    [switch]$Restore
)

# Self-elevate if not already admin
if (-not ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator)) {
    $argList = "-ExecutionPolicy Bypass -File `"$PSCommandPath`""
    if ($Restore) { $argList += " -Restore" }
    Start-Process powershell -Verb RunAs -ArgumentList $argList
    exit
}

# Warn if Cura is running (it locks DLLs)
if (Get-Process -Name "UltiMaker-Cura" -ErrorAction SilentlyContinue) {
    Write-Host "WARNING: UltiMaker Cura is running. Close it before deploying." -ForegroundColor Yellow
    Read-Host "Close Cura, then press Enter to continue (or Ctrl+C to abort)"
}

$CuraDir      = "C:\Program Files\UltiMaker Cura 5.13.0"
$BuildDir     = "$PSScriptRoot\..\build\Release"
$PackagingDir = "$PSScriptRoot\..\packaging"
$DefsDir      = "$CuraDir\share\cura\resources\definitions"

$BinFiles = @(
    "CuraEngine.exe",
    # cura-formulae-engine.dll: may be needed if versions differ; deploy if present in build output.
    "cura-formulae-engine.dll",
    "tbb12.dll",
    "tbbbind_2_5.dll",
    "tbbmalloc.dll",
    "tbbmalloc_proxy.dll"
    # Arcus.dll / polyclipping.dll: permanently excluded - replacing them breaks pyArcus.
)

function Deploy-File($src, $dst) {
    try {
        Copy-Item $src $dst -Force -ErrorAction Stop
        Write-Host "  OK  $([System.IO.Path]::GetFileName($dst))"
        return $true
    } catch {
        Write-Host "  FAIL $([System.IO.Path]::GetFileName($dst)): $_" -ForegroundColor Red
        return $false
    }
}

if ($Restore) {
    Write-Host "Restoring originals..."
    foreach ($f in $BinFiles) {
        $bak = "$CuraDir\$f.bak"
        $dst = "$CuraDir\$f"
        if (Test-Path $bak) {
            Deploy-File $bak $dst | Out-Null
        } else {
            Write-Host "  SKIP $f (no backup found)"
        }
    }
    $defBak = "$DefsDir\fdmprinter.def.json.bak"
    $defDst = "$DefsDir\fdmprinter.def.json"
    if (Test-Path $defBak) {
        Deploy-File $defBak $defDst | Out-Null
    }
    Write-Host "Done - originals restored."
    Read-Host "Press Enter to close"
    return
}

# Backup originals (once - skip if .bak already exists)
Write-Host "Backing up originals..."
foreach ($f in $BinFiles) {
    $src = "$CuraDir\$f"
    $bak = "$CuraDir\$f.bak"
    if ((Test-Path $src) -and -not (Test-Path $bak)) {
        Deploy-File $src $bak | Out-Null
    }
}
$defSrc = "$DefsDir\fdmprinter.def.json"
$defBak = "$DefsDir\fdmprinter.def.json.bak"
if ((Test-Path $defSrc) -and -not (Test-Path $defBak)) {
    Deploy-File $defSrc $defBak | Out-Null
}

# Deploy binaries
Write-Host ""
Write-Host "Deploying to $CuraDir ..."
$ok = $true
foreach ($f in $BinFiles) {
    $src = "$BuildDir\$f"
    if (Test-Path $src) {
        if (-not (Deploy-File $src "$CuraDir\$f")) { $ok = $false }
    }
}

# Deploy settings definition
if (-not (Deploy-File "$PackagingDir\fdmprinter.def.json" $defSrc)) { $ok = $false }

Write-Host ""
if ($ok) {
    Write-Host "Done. Launch Cura and check Print Settings for the Fuselage Slicer category." -ForegroundColor Green
} else {
    Write-Host "Completed with errors - see FAIL lines above." -ForegroundColor Yellow
}
Read-Host "Press Enter to close"
