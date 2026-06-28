# FeatherPrint

FeatherPrint is a modification of UltiMaker Cura that generates conformal geodetic skin structures for ultralight aerodynamic 3D prints — drones, model aircraft, wind turbines, and similar structures.

Instead of conventional infill, FeatherPrint produces a single-wall skin reinforced by a helical geodetic net of structural stringers, inspired by Barnes Wallis's Wellington bomber airframe. The result is a hollow, lightweight shell with excellent torsional stiffness.


---

## Requirements

- Windows 10 or later
- [UltiMaker Cura 5.13.x](https://ultimaker.com/software/ultimaker-cura) installed

---

## Installation

1. Download **FeatherPrint-0.1.0-Windows-x64-Setup.exe** from the [latest release](https://github.com/Rszalay/FeatherPrint/releases/latest)
2. Run the installer as Administrator
3. The installer will detect your Cura 5.13.x installation, back up the original engine files, and deploy FeatherPrint

To uninstall, run **Uninstall FeatherPrint** from Windows Add/Remove Programs. All original Cura files are restored from backup automatically.

---

## Usage

1. Open Cura and set up your printer as normal
2. Load your model — FeatherPrint works best with manifold (watertight) STL files
3. Under **Infill**, set **Infill Pattern** to **FeatherPrint**
4. Wall Line Count, Top Layers, and Bottom Layers will adjust automatically
5. Slice and preview — the geodetic stringer pattern should be visible in the layer view

### Recommended settings

| Setting | Value |
|---|---|
| Wall Line Count | 1 (auto) |
| Top Layers | 0 (auto) |
| Bottom Layers | 0 (auto) |
| Z Seam Alignment | Sharpest Corner (auto) |
| Layer Height | 0.2 mm |
| Material | LW-PLA (lightest) or PETG (toughest) |

---

## Status

FeatherPrint is an early work-in-progress. The following features are currently functional:

- **Skin** — single-wall perimeter extrusion
- **Stringer** — counter-rotating helical geodetic tubes
- **Lacing** — S-link collision feature where CCW and CW stringers meet

The following features are planned but not yet implemented:

- Former (transverse bracing bands at stringer crossings)
- Whip (termination at open mesh boundary edges)
- Gusset, Splay, Cuff (collision features)

---

## For Developers

The full technical specification is in the [`FeatherPrint Spec/`](FeatherPrint%20Spec/) folder:

- [`FeatherPrint_Spec_Jun28_2026.md`](FeatherPrint%20Spec/FeatherPrint_Spec_Jun28_2026.md) — main feature and geometry specification
- [`FeatherPrint_ConformalPlacement_Spec.md`](FeatherPrint%20Spec/FeatherPrint_ConformalPlacement_Spec.md) — Conformal Placement Transform detail

The active development branch is [`featherprint-5.13`](https://github.com/Rszalay/FeatherPrint/tree/featherprint-5.13).

---

## License

CuraEngine is released under the [AGPLv3](LICENSE). FeatherPrint modifications are released under the same license.
