if (-not ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Start-Process powershell -Verb RunAs -ArgumentList "-ExecutionPolicy Bypass -File `"$PSCommandPath`""
    exit
}

$d = "C:\Program Files\UltiMaker Cura 5.14.0"
foreach ($f in @("cura-formulae-engine.dll")) {
    $bak = "$d\$f.bak"
    if (Test-Path $bak) {
        Copy-Item $bak "$d\$f" -Force
        Write-Host "Restored $f"
    } else {
        Write-Host "No backup for $f"
    }
}
Write-Host "Done."
Read-Host "Press Enter to close"
