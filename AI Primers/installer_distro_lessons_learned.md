# Installer / Distro Lessons Learned — CuraFeather Session (June 2026)

> Supplements `CuraEngine_Distribution_Guide_Windows.md` with specific findings from building and testing the CuraFeather deploy toolchain against Cura 5.13.0 on Windows 10.

---

## 1. The Core Constraint: pyArcus Locks Two DLLs

Cura's Python frontend imports `pyArcus.pyd`, which is compiled against the specific ABI of Cura's shipped `Arcus.dll`. This creates a hard constraint:

**Never replace `Arcus.dll` or `polyclipping.dll` in the Cura install directory.**

Replacing either causes:
```
ImportError: DLL load failed while importing pyArcus: The specified procedure could not be found.
```
Cura's frontend then fails to start entirely. These two files must be excluded from every deploy script and every installer `[Files]` section permanently.

The workaround is to build Arcus as a static library, baking it into `CuraEngine.exe`. This eliminates any runtime dependency on `Arcus.dll` from the engine side. See `build_lessons_learned.md §3`.

---

## 2. DLLs That Must Be Deployed (5.14 build)

| File | Reason |
|---|---|
| `CuraEngine.exe` | The engine itself |
| `cura-formulae-engine.dll` | 5.14 requires v1.1.0; Cura 5.13 ships an incompatible older version |
| `tbb12.dll` | Intel TBB threading library |
| `tbbbind_2_5.dll` | TBB CPU binding |
| `tbbmalloc.dll` | TBB memory allocator |
| `tbbmalloc_proxy.dll` | TBB malloc proxy |

`Arcus.dll` and `polyclipping.dll` are explicitly excluded.

---

## 3. fdmprinter.def.json Must Also Be Deployed

Adding new print settings categories to Cura's UI requires deploying a modified `fdmprinter.def.json` to:

```
C:\Program Files\UltiMaker Cura 5.13.0\share\cura\resources\definitions\fdmprinter.def.json
```

This file is large (~10,000 lines). The safe workflow:
1. Copy the original from the Cura install as a base
2. Append your new category/settings as a child of the `settings` object
3. Validate JSON structure carefully — a malformed file silently breaks Cura's settings panel

Cura reads this file at startup. Changes take effect on next Cura launch (no hot-reload).

---

## 4. All Writes to Program Files Require UAC Elevation

Cura installs to `C:\Program Files\`, which is write-protected. Every deploy script must run as Administrator. PowerShell self-elevation pattern:

```powershell
if (-not ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator)) {
    $argList = "-ExecutionPolicy Bypass -File `"$PSCommandPath`""
    if ($Restore) { $argList += " -Restore" }
    Start-Process powershell -Verb RunAs -ArgumentList $argList
    exit
}
```

The `-Restore` switch pattern (passing parameters through elevation) is necessary if the script accepts arguments.

---

## 5. Check for Running Cura Before Deploying

Cura holds file locks on `CuraEngine.exe` and the DLLs while it is running. Attempting to overwrite them produces:

```
The process cannot access the file because it is being used by another process.
```

Add a guard at the top of deploy scripts:

```powershell
if (Get-Process -Name "UltiMaker-Cura" -ErrorAction SilentlyContinue) {
    Write-Host "WARNING: Close Cura before deploying." -ForegroundColor Yellow
    Read-Host "Press Enter to continue or Ctrl+C to abort"
}
```

---

## 6. Backup Before First Deploy — Skip If .bak Already Exists

The deploy script backs up originals on first run and skips if a `.bak` already exists. This prevents overwriting a good backup with a previously-deployed (already-modified) file:

```powershell
if ((Test-Path $src) -and -not (Test-Path $bak)) {
    Copy-Item $src $bak -Force
}
```

A `-Restore` switch re-copies all `.bak` files back. Keep the restore path in the same script to avoid maintaining two separate tools.

---

## 7. Inno Setup 6 — Specific Pitfalls Found

Working on `packaging/CuraFeather.iss`:

| Problem | Fix |
|---|---|
| `DirExistsAppend` is not a valid Inno Setup flag | Remove it — append behaviour is the default |
| `mbWarning` invalid in Pascal message box calls | Use `mbConfirmation` |
| `#13#10` on its own line treated as ISPP preprocessor directive | Keep `#13#10` on the same line as adjacent string literals |
| `LoadStringFromFile` requires `AnsiString` not `String` | Use intermediate `SavedDirA: AnsiString` then assign to String |
| `FileCopy` is deprecated | Use `CopyFile(src, dst, false)` |

---

## 8. Batch File Gotchas for Automated Builds

When driving `VsDevCmd.bat` + Conan + CMake from a `.bat` file:

- Use `cmd /c` not `cmd /k`. `/k` keeps the window open without completing the command sequence.
- `VsDevCmd.bat` ignores `>> logfile.txt 2>&1` placed on its `call` line — it writes to its own console handle. Put log redirects on the lines after the `call`.
- Chain steps with `if errorlevel 1 ( echo STEP_FAILED >> log && exit /b 1 )` so failures are captured rather than silently continuing.

---

## 9. Current deploy-test.ps1 Capabilities

Location: `scripts/deploy-test.ps1`

- Self-elevates to admin
- Warns if Cura is running
- Backs up originals on first run (skips if `.bak` exists)
- Deploys: `CuraEngine.exe`, `cura-formulae-engine.dll`, `tbb12.dll`, `tbbbind_2_5.dll`, `tbbmalloc.dll`, `tbbmalloc_proxy.dll`, `fdmprinter.def.json`
- `-Restore` switch restores all `.bak` files
- Excludes `Arcus.dll` and `polyclipping.dll` (with inline comment explaining why)

A separate `scripts/restore-dlls.ps1` exists as a standalone restore tool for the DLL subset only.

---

## 10. Inno Setup Installer Status

`packaging/CuraFeather.iss` exists and compiles. It stages files to `{tmp}\CuraFeatherStaged`, backs up originals, and copies in the new files. `Arcus.dll` and `polyclipping.dll` are excluded with an explanatory comment. The installer has not been fully end-to-end tested — it was set aside once the deploy script proved sufficient for development iteration.

---

## 11. Recommended Workflow for Next Session

1. Rebuild against CuraEngine 5.13 base
2. Re-run `deploy-test.ps1` — the DLL list may change (some TBB DLLs may no longer be needed, or Arcus may not need static linking)
3. Re-verify `fdmprinter.def.json` — the new settings we added (`top_bottom_skin_merge_distance`, `minimum_infill_line_length`, `material_density`) will be unnecessary on a 5.13 base and should be removed to keep the file clean
4. Complete the Inno Setup installer end-to-end test once the engine works correctly
