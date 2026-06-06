# Quick test deploy — copies Release build directly into Cura without the installer.
# Run from the repo root: .\scripts\deploy-test.ps1
# To restore originals: .\scripts\deploy-test.ps1 -Restore

param(
    [switch]$Restore
)

$CuraDir  = "C:\Program Files\UltiMaker Cura 5.13.0"
$BuildDir = "$PSScriptRoot\..\build\Release"

$Files = @(
    "CuraEngine.exe",
    "Arcus.dll",
    "polyclipping.dll",
    "cura-formulae-engine.dll",
    "tbb12.dll",
    "tbbbind_2_5.dll",
    "tbbmalloc.dll",
    "tbbmalloc_proxy.dll"
)

if ($Restore) {
    Write-Host "Restoring originals..."
    foreach ($f in $Files) {
        $bak = "$CuraDir\$f.bak"
        $dst = "$CuraDir\$f"
        if (Test-Path $bak) {
            Copy-Item $bak $dst -Force
            Write-Host "  Restored $f"
        } else {
            Write-Host "  No backup for $f — skipping"
        }
    }
    Write-Host "Done."
    return
}

# Backup originals on first deploy (skip if .bak already exists)
foreach ($f in $Files) {
    $src = "$CuraDir\$f"
    $bak = "$CuraDir\$f.bak"
    if ((Test-Path $src) -and -not (Test-Path $bak)) {
        Copy-Item $src $bak -Force
        Write-Host "Backed up $f"
    }
}

# Copy build output
Write-Host "Deploying to $CuraDir ..."
foreach ($f in $Files) {
    $src = "$BuildDir\$f"
    if (Test-Path $src) {
        Copy-Item $src "$CuraDir\$f" -Force
        Write-Host "  Copied $f"
    }
}

Write-Host ""
Write-Host "Done. Test with: & '$CuraDir\CuraEngine.exe' help"
