# FeatherPrint

FeatherPrint is a modification of UltiMaker Cura that generates conformal geodetic skin structures for ultralight aerodynamic 3D prints — drones, model aircraft, wind turbines, and similar structures.

Instead of conventional infill, FeatherPrint produces a single-wall skin reinforced by a helical geodetic net of structural stringers, inspired by Barnes Wallis's Wellington bomber airframe. The result is a hollow, lightweight shell with excellent torsional stiffness.

---

## Requirements

- Windows 10 or later
- [UltiMaker Cura 5.13.x](https://ultimaker.com/software/ultimaker-cura) installed

---

## Installation

1. Download **FeatherPrint-0.6.0-Windows-x64-Setup.exe** from the [latest release](https://github.com/Rszalay/FeatherPrint/releases/latest)
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
- **Whip** — terminal feature at open mesh boundary edges
- **Flange / Flare / Miter** — ramped wall transitions and their rim/collision handling
- **Splay** — collision widening at stringer/terminal intersections
- **Anchor Distribution** — continuity-tracking phase placement so stringer anchors track smoothly layer to layer
- **Thin-Section Pruning** — suppresses stringers/lacing/flare rim on sections too thin to support them
- **Shore** — bridges internal overhangs (surfaces enclosed by the model's own hollow shell that ordinary tree support can't reach), generated as infill ahead of the true closure so top skin has something to build on
- **Punchout** *(opt-in)* — a support line + terminal printed into a hole in the model's side wall, so the hole's eventual flat-topped closure has something to rest on; its shape lofts between the real wall contours immediately below and above the hole (Contour Matching) rather than staying a plain straight chord
- **Interior Opening Carry-Through** — holes in a top- or bottom-facing horizontal surface are correctly left open in the top/bottom skin fill, including holes with little or no vertical depth (down to a hole flush with the build plate itself), rather than silently printed over

The following features are planned but not yet implemented:

- Former (transverse bracing bands at stringer crossings)
- Gusset, Cuff, Collar (collision features)

Curvature-Weighted Stringer Density (adaptive stringer spacing based on local surface curvature) was implemented and tested but is currently **disabled** pending a more reliable curvature estimation method; stringers use uniform arc-length spacing in the meantime.

A genuinely zero-depth hole (see Interior Opening Carry-Through, above) is correctly left unprinted-over but does not yet get its own printed Wall trace bounding it — flagged as a follow-up, not yet implemented.

---

## For Developers

The [`FeatherPrint Spec/`](FeatherPrint%20Spec/) folder in this repo archives past spec snapshots and as-built reports (e.g. [`FeatherPrint_AsBuilt_Report_REV2.4_Jul21_2026.md`](FeatherPrint%20Spec/FeatherPrint_AsBuilt_Report_REV2.4_Jul21_2026.md)). The live, actively-maintained specification is versioned separately and not part of this repo.

The active development branch is [`featherprint-5.13`](https://github.com/Rszalay/FeatherPrint/tree/featherprint-5.13).

---

## License

CuraEngine is released under the [AGPLv3](LICENSE). FeatherPrint modifications are released under the same license.
