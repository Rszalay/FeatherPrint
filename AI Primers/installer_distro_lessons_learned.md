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

When engine and frontend are the same version (e.g. both 5.13), the Arcus ABI matches natively and static linking is not required. Keep the static Arcus option in `conanfile.py` as a precaution regardless.

---

## 2. DLLs That Must Be Deployed (5.13 build)

| File | Reason |
|---|---|
| `CuraEngine.exe` | The engine itself (~4.7 MB on 5.13, ~10 MB on 5.14 with static Arcus) |
| `cura-formulae-engine.dll` | Deploy if present in build output; skip if not built |
| `tbb12.dll` | Intel TBB threading library |
| `tbbbind_2_5.dll` | TBB CPU binding |
| `tbbmalloc.dll` | TBB memory allocator |
| `tbbmalloc_proxy.dll` | TBB malloc proxy |

`Arcus.dll` and `polyclipping.dll` are explicitly excluded permanently.

Binary size is a useful sanity check: 5.13 without static Arcus is ~4.7 MB; 5.14 with static Arcus was ~10.1 MB. A size regression after a conan change may mean static linking was accidentally dropped or gained.

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

On a 5.13 engine base, do NOT include the extra settings that were patched for 5.14 compatibility (`top_bottom_skin_merge_distance`, `minimum_infill_line_length`, `material_density`, `material_spool_cost`, `material_spool_weight`). These are harmless if present but add unnecessary noise.

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

**Critical:** `$PSCommandPath` is only set when the script is invoked with a path. If the script is run from the wrong directory or without a path, `$PSCommandPath` is empty and the elevated window opens and immediately closes with no error visible to the user. Always invoke from the repo root:

```powershell
cd C:\Users\ricsz\source\repos\CuraFeather
.\scripts\deploy-test.ps1
```

---

## 5. Check for Running Cura Before Deploying

Cura holds file locks on `CuraEngine.exe` and the DLLs while it is running. Attempting to overwrite them produces silent failures — the copy may succeed for some files but fail for locked ones, leaving a mixed state (old engine, new DLLs or vice versa). The user sees no error unless the script explicitly checks.

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

## 7. deploy-test.ps1 Version Drift

The script was cherry-picked from the main branch onto `curafeather-5.13`, but the cherry-picked version was an older draft that:
- Still included `Arcus.dll` and `polyclipping.dll` in the deploy list (breaking pyArcus)
- Had no self-elevation
- Had no fdmprinter.def.json deployment
- Used a smart-quote em-dash in a comment that caused `AmpersandNotAllowed` parse errors in PowerShell 5.1

**Rule:** After any branch switch or cherry-pick, verify `deploy-test.ps1` matches the known-good version before running it. The authoritative version is documented in this repo's `scripts/` directory on the `curafeather-5.13` branch after the fix commit.

---

## 8. Inno Setup 6 — Specific Pitfalls Found

Working on `packaging/CuraFeather.iss`:

| Problem | Fix |
|---|---|
| `DirExistsAppend` is not a valid Inno Setup flag | Remove it — append behaviour is the default |
| `mbWarning` invalid in Pascal message box calls | Use `mbConfirmation` |
| `#13#10` on its own line treated as ISPP preprocessor directive | Keep `#13#10` on the same line as adjacent string literals |
| `LoadStringFromFile` requires `AnsiString` not `String` | Use intermediate `SavedDirA: AnsiString` then assign to String |
| `FileCopy` is deprecated | Use `CopyFile(src, dst, false)` |

---

## 9. Batch File Gotchas for Automated Builds

When driving `VsDevCmd.bat` + Conan + CMake from a `.bat` file:

- Use `cmd /c` not `cmd /k`. `/k` keeps the window open without completing the command sequence.
- `VsDevCmd.bat` ignores `>> logfile.txt 2>&1` placed on its `call` line — it writes to its own console handle. Put log redirects on the lines after the `call`.
- Chain steps with `if errorlevel 1 ( echo STEP_FAILED >> log && exit /b 1 )` so failures are captured rather than silently continuing.

---

## 10. Current deploy-test.ps1 Capabilities

Location: `scripts/deploy-test.ps1`

- Self-elevates to admin
- Warns if Cura is running
- Backs up originals on first run (skips if `.bak` exists)
- Deploys: `CuraEngine.exe`, `cura-formulae-engine.dll`, `tbb12.dll`, `tbbbind_2_5.dll`, `tbbmalloc.dll`, `tbbmalloc_proxy.dll`, `fdmprinter.def.json`
- `-Restore` switch restores all `.bak` files
- Excludes `Arcus.dll` and `polyclipping.dll` (with inline comment explaining why)
- Must be run from the repo root for `$PSCommandPath` to resolve correctly

---

## 11. Engine Version Confirmation

After deploying, verify the correct binary is in place before testing:

```powershell
Get-Item "C:\Program Files\UltiMaker Cura 5.13.0\CuraEngine.exe" | Select-Object Length, LastWriteTime
```

Expected for a 5.13 build: ~4.7 MB. If the file is ~10 MB, the old 5.14 static-Arcus build is still deployed.

The Cura log also prints the engine version banner on every launch:
```
Cura_SteamEngine version 5.13.0
```
If this shows 5.14.0-alpha.0, the wrong binary is deployed.

---

## 12. Inno Setup Installer Status

`packaging/CuraFeather.iss` exists and compiles. It stages files to `{tmp}\CuraFeatherStaged`, backs up originals, and copies in the new files. `Arcus.dll` and `polyclipping.dll` are excluded with an explanatory comment. The installer has not been fully end-to-end tested — it was set aside once the deploy script proved sufficient for development iteration.
