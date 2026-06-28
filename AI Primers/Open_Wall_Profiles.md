# Open Wall Profiles — FeatherPrint Pipeline Migration Assessment

> **Review status (added 28 Jun 26):** Verified against `Ultimaker/CuraEngine` `main`,
> version `5.14.0-alpha.0`, commit `e5b844b` (2026-06-25). The core thesis holds —
> **open polygons for whips genuinely survive the wall pipeline** — but three of the
> six "Required Changes" are wrong as written and the spiralize plan (#5) does not work
> at all in the current engine. The corrected architecture is summarised in the new
> **Codebase Verification** section at the bottom of this document. Read that section
> before implementing. Inline ⚠️ markers below flag the specific claims that changed.

## Current State

FeatherPrint is dispatched from `Infill::_generate()` when `pattern == FEATHERPRINT`.  
Output: a single `OpenPolyline` pushed to `result_lines` (the infill open-lines container).  
Problem: infill paths have no seam treatment, no z-seam placement, and no spiralize support.  
The travel move to the start of each layer's infill line creates a vertical blob at the seam position.  
Attempts to move the seam by adjusting the start arc-position have not succeeded because the blob is a property of the infill travel path, not something addressable within the infill generator.

---

## Why Wall Output Solves This

Walls in CuraEngine go through `InsetOrderOptimizer`, which provides:

- **Z-seam placement** — for closed `ExtrusionLine` (`is_closed_ = true`), the seam is computed via `InsetOrderOptimizer::insertSeamPoint()` and can be placed at a sharp corner, shortest travel, or user-specified position.
- **Spiralize (vase mode)** — `LayerPlan::spiralizeWallSlice()` applies a continuous Z-increment across the wall path, eliminating the layer seam entirely. This is the correct long-term solution for FeatherPrint.
- **Open polylines are supported** — `ExtrusionLine` has `is_closed_ = false`, which skips seam insertion and is output as a raw polyline. This preserves future whip support (whips require open path segments).
- **Scarf seam** — available for closed walls, ramps extrusion at start/end to reduce the blob if full spiralize is not used.

---

## Data Structures to Understand

| Type | Header | Role |
|---|---|---|
| `ExtrusionJunction` | `include/utils/ExtrusionJunction.h` | A single point: `Point2LL p`, `coord_t w` (local width), `size_t perimeter_index` |
| `ExtrusionLine` | `include/utils/ExtrusionLine.h` | Ordered vector of junctions + `inset_idx_`, `is_closed_`, `is_odd_` |
| `VariableWidthLines` | alias for `std::vector<ExtrusionLine>` | One inset level's worth of lines |
| `part.wall_toolpaths` | `include/sliceDataStorage.h` | `std::vector<VariableWidthLines>` stored per `SliceLayerPart` |

---

## Pipeline Flow (Wall Path → GCode)

```
FeatherPrintGenerator (proposed)
  ↓ produces std::vector<VariableWidthLines>
Injected into part.wall_toolpaths
  ↓
FffGcodeWriter::preProcessInsets()     [src/FffGcodeWriter.cpp ~line 2888]
  ↓
InsetOrderOptimizer::optimize()        [src/InsetOrderOptimizer.cpp]
  ↓ insertSeamPoint() for closed lines
  ↓ PathOrderOptimizer handles ordering
LayerPlan::addWall()                   [src/LayerPlan.cpp ~line 1882]
  ↓ spiralizeWallSlice() if magic_spiralize
GCodePath objects → GCode writer
```

---

## Required Changes

### 1. Change Generator Output Type

**Current**: `generate()` produces `OpenPolyline` pushed to `OpenLinesSet& result_lines`.  
**New**: `generate()` produces `VariableWidthLines` (a vector of `ExtrusionLine`).

Each point in the current `OpenPolyline` becomes an `ExtrusionJunction`:
```cpp
ExtrusionJunction j;
j.p_ = point2ll_position;
j.w_ = featherprint_line_width;   // constant width throughout
j.perimeter_index_ = 0;
```
⚠️ **Correction:** the actual members carry trailing underscores (`p_`, `w_`,
`perimeter_index_`), and `ExtrusionJunction` has no default constructor — it is
`ExtrusionJunction(Point2LL p, coord_t w, coord_t perimeter_index)`. Use
`junctions_.emplace_back(pos, width, 0);` rather than field assignment.
(`include/utils/ExtrusionJunction.h`)

The single continuous path becomes one `ExtrusionLine` with:
- `inset_idx_ = 0` (treat as outer wall)
- `is_closed_ = true` for the standard perimeter loop (enables seam treatment and spiralize)
- `is_closed_ = false` for whip segments when those are implemented

### 2. Change the Dispatch Point

**Current**: dispatched from `src/infill.cpp` `Infill::_generate()`.  
**New**: dispatched from `src/WallsComputation.cpp`, after the existing wall toolpath generation, or directly in `FffGcodeWriter::preProcessInsets()` before `InsetOrderOptimizer` is constructed.

The cleanest injection point is `FffGcodeWriter::preProcessInsets()` around line 2888:
```cpp
// After WallsComputation has run, before InsetOrderOptimizer:
if (mesh.settings.get<EFillMethod>("infill_pattern") == EFillMethod::FEATHERPRINT) {
    FeatherPrintGenerator fp;
    VariableWidthLines fp_lines = fp.generate(part, z, mesh.settings);
    if (!part.wall_toolpaths.empty())
        part.wall_toolpaths[0].insert(part.wall_toolpaths[0].end(),
                                      fp_lines.begin(), fp_lines.end());
    else
        part.wall_toolpaths.push_back(fp_lines);
}
```

⚠️ **Correction — this injection point does not work.** Two upstream mechanisms defeat it
(both verified below in the Codebase Verification section):
> 1. `preProcessInsets()` (now at **line 2542**, not 2888) early-returns `{}` when
>    `wall_line_count <= 0` (line 2555) — *before* the `InsetOrderOptimizer` is built at
>    line 2844. With the `wall_line_count = 0` requirement from change #4, the injected
>    toolpaths are never optimised or printed.
> 2. Even with `wall_line_count >= 1`, parts whose `wall_toolpaths` are empty at the end of
>    `WallsComputation::generateWalls(SliceLayer*)` are **deleted** from the layer
>    (the cull at line 108–115). If normal walls are suppressed without something filling
>    `wall_toolpaths`, the part is gone long before `FffGcodeWriter` runs.
>
> **The correct injection point is `WallsComputation::generateWalls(SliceLayerPart*)`**,
> replacing the `WallToolPaths` call so that FeatherPrint *populates* `part.wall_toolpaths`
> in-place. That keeps the part alive through the cull and lets the existing
> `preProcessInsets → InsetOrderOptimizer` path consume it unchanged. See the revised
> architecture below.

### 3. Remove the Infill Dispatch

Remove from `src/infill.cpp`:
- The `case EFillMethod::FEATHERPRINT:` block
- The `&& pattern_ != EFillMethod::FEATHERPRINT` guard on the `line_distance_ == 0` early return

The `FEATHERPRINT` enum value in `EnumSettings.h` and the string mapping in `Settings.cpp` stay — the setting is still used to identify the mode.

### 4. Suppress Normal Wall and Infill Generation

When FeatherPrint is active, the normal wall and infill machinery should be suppressed to prevent double-printing:

- `wall_line_count = 0` suppresses Arachne wall generation (already required by the current setup — this stays as a user requirement).
- `infill_sparse_density` can remain non-zero (it was only needed to prevent `line_distance_ == 0` early-exit in the infill path; once we're out of infill, it no longer matters).
- Top/bottom skin: `top_layers = 0`, `bottom_layers = 0` as before.

Alternatively, add an explicit check in `WallsComputation::generateWalls()` to skip normal wall generation when `infill_pattern == FEATHERPRINT`, making it self-contained without depending on user settings.

> ⚠️ **Correction — `wall_line_count = 0` is incompatible with wall output.** With the
> generator moved into the wall pipeline, `wall_line_count` must be **≥ 1**, because:
> - `WallsComputation::generateWalls(SliceLayerPart*)` early-outs at `wall_count == 0`
>   (line 41–46) and never reaches the toolpath branch.
> - `preProcessInsets()` early-returns at `wall_line_count <= 0` (line 2555), so the
>   `InsetOrderOptimizer` is never constructed.
>
> The right pattern is `wall_line_count = 1` **plus** a branch inside
> `generateWalls(SliceLayerPart*)` that, when `infill_pattern == FEATHERPRINT`, **replaces**
> the `WallToolPaths(...)` call with the FeatherPrint generator and assigns the result to
> `part->wall_toolpaths` (also setting `part->inner_area` / `part->print_outline`). This is
> "replace", not merely "suppress" — suppressing without replacing trips the part cull.

### 5. Spiralize

Once output is a closed `ExtrusionLine` in `part.wall_toolpaths`, Cura's built-in `magic_spiralize` setting works natively. No changes to `LayerPlan` or the GCode writer are needed — `spiralizeWallSlice()` already handles the Z-increment. The user enables spiralize in Cura settings and the seam disappears.

> ⚠️ **Correction — this is false in the current engine, and is fundamentally incompatible
> with whips.** `magic_spiralize` does **not** read `part.wall_toolpaths` at all on spiral
> layers:
> - `WallsComputation::generateWalls` computes a separate `part.spiral_wall` (a `Shape` of
>   **closed `Polygon`s** produced by offsetting `part.outline`, line 121) and the GCode
>   writer's spiralize branch (`endProcessInsets` → `processSpiralizedWall` →
>   `LayerPlan::spiralizeWallSlice`) consumes **only** `part.spiral_wall`. The
>   `walls_optimizer` holding FeatherPrint's toolpaths is used **only in the non-spiralized
>   `else` branch** (`FffGcodeWriter.cpp` line 2831, 2924). FeatherPrint geometry is
>   discarded on spiral layers and replaced by a plain outline offset.
> - `spiralizeWallSlice(... const Polygon& wall ...)` indexes a **single closed contour**
>   modulo `n_points`. An open polygon (a whip) has no closed loop to ramp around, and a
>   whip layer's multiple segments break the single-contour assumption outright.
> - `magic_spiralize` is a **mesh-group-global** setting, not per-mesh, so it cannot be
>   enabled for closed meshes and disabled for whip-bearing ones in the same print.
>
> **Conclusion:** a whip-bearing build **cannot** use `magic_spiralize`. The original
> seam-elimination goal must be met another way — **scarf seam** on closed layers (already
> wired through `InsetOrderOptimizer`, and it *does* read `wall_toolpaths`), or a custom
> continuous-Z ramp implemented inside the FeatherPrint generator. Native spiralize is only
> an option for the degenerate closed-only case, and even then only if FeatherPrint also
> populates `part.spiral_wall` — which it cannot do faithfully, since `spiral_wall` is a
> single closed polygon with no room for stringers, formers, or terminals.

### 6. Whip Compatibility

When whips are implemented, a single layer's output will be multiple segments: perimeter sections before and after whip lifts. Each segment becomes a separate `ExtrusionLine` with `is_closed_ = false`. Open `ExtrusionLine` objects are already handled by `InsetOrderOptimizer` (they skip seam insertion and are added as polylines). No further pipeline changes are needed to support whips.

> ✅ **Confirmed.** This is the one claim that holds exactly as written.
> `InsetOrderOptimizer::optimize()` branches on `line.is_closed_`: closed lines go through
> `insertSeamPoint()` + `addPolygon()`, open lines go straight to `addPolyline()`
> (`src/InsetOrderOptimizer.cpp` line 129–141). `addToLayer()` then emits both via
> `LayerPlan::addWall(...)` with `linked_path = !path.is_closed_` (line 196–211), so open
> whip segments are printed as polylines with no forced closure and no spurious seam. A
> whip layer = several `ExtrusionLine`s (open perimeter arcs + closed terminal loops) in a
> single `VariableWidthLines`; the optimiser iterates and dispatches each correctly.
> **This is the heart of the migration and it is sound — provided the geometry reaches the
> optimiser, which requires the change-#2/#4 corrections above and rules out spiralize (#5).**

---

## Effort Estimate

| Task | Complexity |
|---|---|
| Rewrite `generate()` to output `VariableWidthLines` | Medium — point-by-point conversion, constant width |
| Change injection point in `FffGcodeWriter.cpp` | Low — a few lines at a known location |
| Remove infill dispatch in `infill.cpp` | Trivial |
| Suppress normal wall generation | Low — conditional in `WallsComputation.cpp` or user-setting enforcement |
| Test spiralize end-to-end | Low — enable `magic_spiralize`, reslice |
| Adapt for whips (future) | Low — output multiple open `ExtrusionLine` objects instead of one closed one |

Total: approximately one focused session. The largest risk is getting the `ExtrusionJunction` width and `inset_idx_` values right so the GCode writer produces correct extrusion amounts. Since FeatherPrint uses constant width throughout, this is straightforward — `j.w = featherprint_line_width` for every junction.

---

## Files to Modify

| File | Change |
|---|---|
| `include/featherprint/FeatherPrintGenerator.h` | Change return type of `generate()` |
| `src/featherprint/FeatherPrintGenerator.cpp` | Rewrite output: `OpenPolyline` → `ExtrusionLine` |
| `src/FffGcodeWriter.cpp` | Add injection block before `InsetOrderOptimizer` construction |
| `src/infill.cpp` | Remove `FEATHERPRINT` dispatch case and guard |
| `src/WallsComputation.cpp` | Optionally suppress normal walls when FEATHERPRINT active |

---

# Codebase Verification (CuraEngine 5.14.0-alpha.0, commit `e5b844b`, 2026-06-25)

This section records a line-by-line check of every structural claim in the assessment
against the current `main` branch. Verdicts: ✅ holds · ⚠️ partially wrong · ❌ wrong.

## Verdict summary

| # | Claim | Verdict | Consequence |
|---|---|---|---|
| — | Data structures (`ExtrusionJunction`, `ExtrusionLine`, `VariableWidthLines`, `part.wall_toolpaths`) exist as described | ✅ (field names need `_` suffix) | Cosmetic code fix |
| 1 | Generator outputs `VariableWidthLines` | ✅ | No issue |
| 2 | Inject in `FffGcodeWriter::preProcessInsets()` | ❌ | Part is culled / guard skips it — move to `WallsComputation` |
| 3 | Remove infill dispatch in `infill.cpp` | ✅ | Trivial, correct |
| 4 | `wall_line_count = 0` suppresses Arachne and stays | ❌ | Must be `≥ 1`, or nothing prints |
| 5 | `magic_spiralize` works natively from `wall_toolpaths` | ❌ | Spiralize ignores `wall_toolpaths`; **incompatible with whips** |
| 6 | Open `ExtrusionLine`s carry whips through unchanged | ✅ | The migration's core premise is sound |

## Evidence

### Data structures — ✅ (with corrections)
- `include/utils/ExtrusionJunction.h`: members are `p_` (`Point2LL`), `w_` (`coord_t`),
  `perimeter_index_` (`size_t`); constructor `ExtrusionJunction(Point2LL, coord_t, coord_t)`.
  No default ctor — build with `emplace_back`.
- `include/utils/ExtrusionLine.h`: `inset_idx_`, `is_odd_`, `is_closed_`,
  `junctions_`; `using VariableWidthLines = std::vector<ExtrusionLine>;` (line 239).
  (The assessment's header path `include/utils/ExtrusionLine.h` is correct.)
- `include/sliceDataStorage.h` line 68: `std::vector<VariableWidthLines> wall_toolpaths;`
  "binned by inset_idx" — a single bin (`inset_idx_ = 0`) is fine for FeatherPrint, and a
  bin may freely mix closed and open lines (the optimiser checks `is_closed_` per line).

### Open polygons / whips — ✅ (verified end to end)
- `src/InsetOrderOptimizer.cpp` ~129–141:
  `if (line.is_closed_) { … insertSeamPoint(); addPolygon(); } else { addPolyline(&line); }`.
- `src/InsetOrderOptimizer.cpp` ~196–211: `addToLayer()` emits every path through
  `gcode_layer_.addWall(..., path.is_closed_, backwards, linked_path = !path.is_closed_, …)`.
- Net: an open `ExtrusionLine` skips seam insertion and is printed as an unclosed polyline.
  This is exactly the behaviour whips need.

### Injection point — ❌ (must move earlier)
- `src/FffGcodeWriter.cpp` 2542 `preProcessInsets()`; 2555 early-`return {}` when
  `wall_line_count <= 0`; 2844 `InsetOrderOptimizer` constructed with `part.wall_toolpaths`
  (2862) — *downstream* of the guard. Injecting here while `wall_line_count == 0` is dead code.
- `src/WallsComputation.cpp` 108–115: in `generateWalls(SliceLayer*)`, parts with empty
  `wall_toolpaths` **and** empty `spiral_wall` are erased when `wall_line_count >= 1`.
  Therefore FeatherPrint must fill `wall_toolpaths` *during* `WallsComputation`, not later.
- Correct site: `src/WallsComputation.cpp` `generateWalls(SliceLayerPart*)` ~38, replacing the
  `WallToolPaths wall_tool_paths(...); part->wall_toolpaths = wall_tool_paths.getToolPaths();`
  block (the non-spiralize branch ~84–88) with the FeatherPrint generator.

### `wall_line_count` — ❌ (needs ≥ 1)
- `src/WallsComputation.cpp` 41–46: `generateWalls(SliceLayerPart*)` early-outs at
  `wall_count == 0`.
- `src/FffGcodeWriter.cpp` 2555: `preProcessInsets()` early-outs at `wall_line_count <= 0`.
- Both gates must be open ⇒ set `wall_line_count = 1` and *replace* the Arachne call rather
  than zeroing the count. Arachne double-printing is avoided by the replacement, not by `=0`.

### Spiralize — ❌ (separate codepath; whip-incompatible)
- `include/sliceDataStorage.h` 65: `Shape spiral_wall;` — closed polygons only.
- `src/WallsComputation.cpp` 121: `part->spiral_wall = part->outline.offset(-line_width_0/2 - wall_0_inset);`
  — derived from the outline, never from FeatherPrint output.
- `src/FffGcodeWriter.cpp` 2561–2575: spiralize flag set from the mesh-group global
  `magic_spiralize`; 2831 the `InsetOrderOptimizer` (the `wall_toolpaths` consumer) is built
  only when `! result.spiralize`; 2886–2924 `endProcessInsets` spiralize branch uses
  `part.spiral_wall` exclusively.
- `src/LayerPlan.cpp` 2930: `spiralizeWallSlice(const Polygon& wall, …)` indexes one closed
  contour `% n_points`. Open contours are structurally unrepresentable.
- `magic_spiralize` is read from `scene.current_mesh_group->settings` (e.g.
  `FffGcodeWriter.cpp` 2561) — a global, so no per-mesh mixing.

### `FEATHERPRINT` enum — context note
- `grep` for `FEATHERPRINT` across upstream `src/` and `include/` returns nothing; upstream
  `EFillMethod` (`include/settings/EnumSettings.h`) ends `… OCTAGON, NONE, PLUGIN`. The enum
  value, its `Settings.cpp` string mapping, and the `infill.cpp` dispatch are all **fork-local
  additions**. Keeping `EFillMethod::FEATHERPRINT` purely as a *mode selector* (read in
  `WallsComputation`) is fine, but note it now selects a wall path, not an infill pattern, so
  the name `infill_pattern` is a slight misnomer going forward. If `FEATHERPRINT` is added
  before `NONE`, keep it out of any test that enumerates fill methods (see the `NONE`/`PLUGIN`
  ordering comment in `EnumSettings.h`).

## Revised architecture (recommended)

1. **Generate in `WallsComputation`.** In `generateWalls(SliceLayerPart*)`, branch on
   `settings_.get<EFillMethod>("infill_pattern") == EFillMethod::FEATHERPRINT`. In that branch,
   call the FeatherPrint generator and assign `part->wall_toolpaths` (one `VariableWidthLines`,
   `inset_idx_ = 0`), then set `part->inner_area` and `part->print_outline` from the outline so
   downstream area logic stays sane. Do **not** call `WallToolPaths`.
2. **Keep `wall_line_count = 1`.** Satisfies both early-out guards and the part-cull; the
   replacement in step 1 means no Arachne walls are produced, so no double-printing.
3. **Let the existing pipeline run unchanged.** `preProcessInsets` builds the
   `InsetOrderOptimizer` over `part.wall_toolpaths`; closed perimeters get seam/scarf
   treatment, open whip segments get `addPolyline`/`addWall`. No changes to
   `InsetOrderOptimizer`, `LayerPlan`, or the GCode writer.
4. **Drop the infill dispatch** (`infill.cpp` `FEATHERPRINT` case + the `line_distance_ == 0`
   guard); keep the enum/string mapping as a mode flag. (Assessment change #3, unchanged.)
5. **Seam strategy without spiralize.** For whip-bearing models, disable `magic_spiralize`
   (it is incompatible and would discard the geometry anyway) and rely on **scarf seam** for
   the closed perimeters, or implement a continuous-Z ramp inside the generator. Re-evaluate
   native spiralize only for a closed-only profile, and only if the generator can also emit a
   faithful single closed `part.spiral_wall` — which it cannot once stringers/formers/whips
   exist.

## Spec-compliance note

The FeatherPrint spec defines four slice-geometry primitives — Perimeter (closed), Hoop,
Trace, and **Terminal (open)** — plus the collision features (Lacing, Gusset, Splay, Cuff),
of which **Splay** and **Cuff** terminate against whip boundaries and therefore also produce
open geometry at the boundary layers. The verified `is_closed_ = false → addPolyline → addWall`
path is sufficient to represent every open primitive the spec requires, and a single layer may
mix closed loops (Hoops, Terminals that close on themselves) and open arcs (Perimeter segments
split by a whip lift) in one `VariableWidthLines` bin. The spec is therefore realisable on the
wall pipeline **as long as the migration follows the revised architecture above and does not
depend on `magic_spiralize`.**

## Revised effort estimate

| Task | Complexity | Change from original |
|---|---|---|
| Generator outputs `VariableWidthLines` (constant width, `inset_idx_=0`) | Medium | unchanged |
| Inject by **replacing** the `WallToolPaths` call in `WallsComputation` | Low–Medium | moved from `FffGcodeWriter`; must also set `inner_area`/`print_outline` |
| Keep `wall_line_count = 1`; remove `=0` requirement | Trivial | corrected from #4 |
| Remove infill dispatch in `infill.cpp` | Trivial | unchanged |
| Seam handling via scarf seam (closed layers) | Low | **replaces** native-spiralize plan |
| Native spiralize end-to-end | — | **dropped** (incompatible with whips) |
| Whip open-`ExtrusionLine` support | None (already works) | confirmed |

Net: still roughly one focused session, but the effort shifts from "wire up spiralize" to
"get the `WallsComputation` replacement + part-survival right." The principal risk is no longer
extrusion width — it is ensuring the FeatherPrint part is not culled and that `inner_area` /
`print_outline` are populated so later skin/infill/support area math does not misbehave.
