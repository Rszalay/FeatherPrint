# FeatherPrint

FeatherPrint is a modification of UltiMaker Cura that generates conformal geodetic skin structures for ultralight aerodynamic 3D prints — drones, model aircraft, wind turbines, and similar structures.

As of v1.0.0, FeatherPrint ships **two mutually exclusive infill generators**, selected via the standard **Infill Pattern** setting:

- **FeatherPrint** — for single-continuous-surface meshes with no separate inner/outer wall (see below). Produces a single-wall skin reinforced by a helical geodetic net of structural stringers, inspired by Barnes Wallis's Wellington bomber airframe. The result is a hollow, lightweight shell with excellent torsional stiffness.
- **Corrugated** — for ordinary closed-manifold, shelled/thin-walled meshes that slice into conventional inner and outer perimeter walls. Discovers the wall-to-wall correspondence per slice (via a vendored VBCT/CDT pipeline) and generates corrugated geodesic support spanning the two walls, in place of conventional infill.

Only one generator runs per mesh, dispatched from the same engine build — pick whichever matches your model's geometry.

---

## Requirements

- Windows 10 or later
- [UltiMaker Cura 5.13.x](https://ultimaker.com/software/ultimaker-cura) installed

---

## Installation

1. Download **FeatherPrint-1.0.1-Windows-x64-Setup.exe** from the [latest release](https://github.com/Rszalay/FeatherPrint/releases/latest)
2. Run the installer as Administrator
3. The installer will detect your Cura 5.13.x installation, back up the original engine files, deploy FeatherPrint, and install a validated **"FeatherPrint Corrugated"** print profile (Print Settings → Profiles)

To uninstall, run **Uninstall FeatherPrint** from Windows Add/Remove Programs. All original Cura files are restored from backup automatically.

---

## Usage

1. Open Cura and set up your printer as normal
2. Load your model — both generators require manifold (watertight) STL files
3. Under **Infill**, set **Infill Pattern** to **FeatherPrint** or **Corrugated**, depending on your model's geometry (see above)
4. Wall Line Count, Top Layers, and Bottom Layers will adjust automatically
5. Slice and preview — the geodetic stringer (FeatherPrint) or corrugated wall-to-wall (Corrugated) pattern should be visible in the layer view

For Corrugated, the bundled **"FeatherPrint Corrugated"** print profile (installed automatically) sets the validated `corrugated_*` tuning settings for you — select it under Print Settings → Profiles rather than hand-tuning those settings on a stock profile.

### Recommended settings — FeatherPrint

| Setting | Value |
|---|---|
| Wall Line Count | 1 (auto) |
| Top Layers | 0 (auto) |
| Bottom Layers | 0 (auto) |
| Z Seam Alignment | Sharpest Corner (auto) |
| Layer Height | 0.2 mm |
| Material | LW-PLA (lightest) or PETG (toughest) |

### Recommended settings — Corrugated

Use the bundled **"FeatherPrint Corrugated"** profile as a starting point rather than setting these by hand; it covers wall count, line width, and every `corrugated_*` tuning setting for a typical thin-walled model. Material/temperature settings are left to whichever material profile you already use.

---

## Status

FeatherPrint is an early work-in-progress. The following features are currently functional:

### FeatherPrint generator

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
- **Former / Gusset / Cuff** — symmetric wall-stack bands, flush with the outer skin, generated at each synchronized stringer helix crossing (Lacing) for transverse bracing; Gusset integrates the crossing stringer into the band, and Cuff resolves a band meeting an open mesh boundary edge. **Former Spacing** lets you skip crossings (e.g. every other one) to trade stiffness for mass.
- **Collar / Placket** — a Former-style reinforcement band at every Whip Terminal (both ends of every hole/slot, and the model's own top/bottom where no Flange is present); Placket resolves a Collar band's own taper meeting an open mesh boundary edge, the same way Cuff does for Former.
- **Shelf** — a row of small support loops printed at a hole's own topmost Layer, wherever Punchout and Collar are both active, so the upper Collar band above the hole has something to build on instead of printing directly over open air.
- **Collision Pruning** — Flange/Former/Collar wall stacks otherwise have no bound on how deep they offset inward; at a thin neck (narrower than twice the deepest wall's offset), that produces a self-intersecting or Skin-crossing toolpath. Detects and prunes the invalid segment on both the closed-ring and open-arc wall-generation pipelines, welding the remaining valid geometry back together at the crossing point, and correctly places Gusset/Flare Rim geometry even where a thin neck splits a wall into disjoint lobes.

Curvature-Weighted Stringer Density (adaptive stringer spacing based on local surface curvature) was implemented and tested but is currently **disabled** pending a more reliable curvature estimation method; stringers use uniform arc-length spacing in the meantime.

A genuinely zero-depth hole (see Interior Opening Carry-Through, above) is correctly left unprinted-over but does not yet get its own printed Wall trace bounding it — flagged as a follow-up, not yet implemented.

### Corrugated generator

- **Wall correspondence discovery** — per-slice, via a vendored VBCT/CDT pipeline (Constrained Delaunay Triangulation), classifying regions as Ring (nested annulus), Chain (opposing wall pair), or Glob (no usable decomposition)
- **Corrugation Field** — the corrugated stringer geometry spanning the discovered inner/outer wall pair, for both Ring and Chain domains
- **Crosshatch** — a second, crossing stringer family for added stiffness (opt-in)
- **Linked corrugation skin** — continuity across Ring domains
- **Chain Wall A/B Canonical Assignment** — stable, non-thrashing wall-side identity across layers for Chain domains
- **Wall Strip** — post-hoc trim reaching back into wall generation's own output, for both Ring and Chain domains
- **Transition Layer** — per-domain solid-fill substitution triggered on a domain-count change
- **De Minimis Hole Threshold** — ignores small interior holes for domain-classification purposes
- **Tessellation diagonal artifact removal** (`meshfix_remove_diagonal_artifacts`) — a general mesh-fix, not corrugation-specific, benefiting every consumer of the raw per-layer contour

Not yet built: the Glob fallback, true N-way hub stitching, and the corrugation cross-section's own trough/peak arc shaping (see `FeatherPrint Corrugated - Spec REV 3.8` for the full status and open questions).

---

## For Developers

The [`FeatherPrint Spec/`](FeatherPrint%20Spec/) folder in this repo archives past spec snapshots and as-built reports (e.g. [`FeatherPrint_AsBuilt_Report_REV2.4_Jul21_2026.md`](FeatherPrint%20Spec/FeatherPrint_AsBuilt_Report_REV2.4_Jul21_2026.md)). The live, actively-maintained specification is versioned separately and not part of this repo.

The active development branch is [`featherprint-5.13`](https://github.com/Rszalay/FeatherPrint/tree/featherprint-5.13).

The Corrugated generator vendors the VBCT algorithm library (`src/vbct/`) and the CDT triangulation library it depends on (`include/third_party/CDT/`, MPL-2.0) directly into this repo — see their respective `VENDORED.md` files for provenance.

---

## License

CuraEngine is released under the [AGPLv3](LICENSE). FeatherPrint modifications are released under the same license.
