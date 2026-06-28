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
| `ExtrusionJunction` | `include/utils/ExtrusionJunction.h` | A single point: `Point2LL p_`, `coord_t w_` (local width), `size_t perimeter_index_` |
| `ExtrusionLine` | `include/utils/ExtrusionLine.h` | Ordered vector of junctions + `inset_idx_`, `is_closed_`, `is_odd_` |
| `VariableWidthLines` | alias for `std::vector<ExtrusionLine>` | One inset level's worth of lines |
| `part.wall_toolpaths` | `include/sliceDataStorage.h` | `std::vector<VariableWidthLines>` stored per `SliceLayerPart` |

---

## Pipeline Flow (Wall Path → GCode)

```
FeatherPrintGenerator (proposed)
  ↓ produces std::vector<VariableWidthLines>
Replaces WallToolPaths call in WallsComputation::generateWalls(SliceLayerPart*)
  ↓ part->wall_toolpaths populated in-place
FffGcodeWriter::preProcessInsets()     [src/FffGcodeWriter.cpp ~line 2542]
  ↓
InsetOrderOptimizer::optimize()        [src/InsetOrderOptimizer.cpp]
  ↓ insertSeamPoint() for closed lines
  ↓ PathOrderOptimizer handles ordering
LayerPlan::addWall()                   [src/LayerPlan.cpp]
  ↓ scarf seam on closed perimeter layers
GCodePath objects → GCode writer
```

---

## Required Changes

### 1. Change Generator Output Type

**Current**: `generate()` produces `OpenPolyline` pushed to `OpenLinesSet& result_lines`.  
**New**: `generate()` produces `VariableWidthLines` (a vector of `ExtrusionLine`).

Each point in the current `OpenPolyline` becomes an `ExtrusionJunction`:
```cpp
// Correct constructor — no default ctor exists
junctions_.emplace_back(point2ll_position, featherprint_line_width, 0);
// Fields: p_ (Point2LL), w_ (coord_t), perimeter_index_ (coord_t)
```

The single continuous path becomes one `ExtrusionLine` with:
- `inset_idx_ = 0` (treat as outer wall)
- `is_closed_ = true` for the standard perimeter loop (enables seam treatment)
- `is_closed_ = false` for whip segments when those are implemented

### 2. Change the Dispatch Point

**Current**: dispatched from `src/infill.cpp` `Infill::_generate()`.  
**New**: dispatched from `src/WallsComputation.cpp` inside `generateWalls(SliceLayerPart*)`,
replacing the `WallToolPaths` call — not injected later in `FffGcodeWriter`.

> ⚠️ **The `FffGcodeWriter::preProcessInsets()` injection point does not work.** Two upstream
> mechanisms defeat it:
> 1. `preProcessInsets()` early-returns `{}` when `wall_line_count <= 0` (line 2555) —
>    *before* `InsetOrderOptimizer` is constructed. With `wall_line_count = 0`, the toolpaths
>    are never optimised or printed.
> 2. In `generateWalls(SliceLayer*)`, parts with empty `wall_toolpaths` are culled (lines
>    108–115). If `wall_toolpaths` isn't populated during `WallsComputation`, the part is
>    deleted before `FffGcodeWriter` runs.
>
> **Correct site**: `WallsComputation::generateWalls(SliceLayerPart*)` ~line 38. When
> `infill_pattern == FEATHERPRINT`, replace the `WallToolPaths(...)` block (~lines 84–88)
> with the FeatherPrint generator. Also set `part->inner_area` and `part->print_outline`
> from the outline so downstream area logic stays sane.

```cpp
// In WallsComputation::generateWalls(SliceLayerPart*)
if (settings_.get<EFillMethod>("infill_pattern") == EFillMethod::FEATHERPRINT)
{
    FeatherPrintGenerator fp;
    part->wall_toolpaths.resize(1);
    part->wall_toolpaths[0] = fp.generate(part->outline, z, settings_);
    part->inner_area = part->outline;
    part->print_outline = part->outline;
    return; // skip Arachne
}
// ... existing WallToolPaths call below
```

### 3. Remove the Infill Dispatch

Remove from `src/infill.cpp`:
- The `case EFillMethod::FEATHERPRINT:` block
- The `&& pattern_ != EFillMethod::FEATHERPRINT` guard on the `line_distance_ == 0` early return

The `FEATHERPRINT` enum value in `EnumSettings.h` and the string mapping in `Settings.cpp` stay — the setting is now read by `WallsComputation` as a mode flag.

### 4. Wall Line Count

> ⚠️ **`wall_line_count = 0` is incompatible with wall output.** Both
> `WallsComputation::generateWalls(SliceLayerPart*)` (early-out at line 41–46) and
> `FffGcodeWriter::preProcessInsets()` (early-out at line 2555) gate on `wall_line_count <= 0`.
>
> Set `wall_line_count = 1`. The replacement in `generateWalls` means Arachne never runs,
> so there is no double-printing. Do not set to 0.

Update the Cura profile: `wall_line_count = 1`.  
`infill_sparse_density` and `infill_pattern` settings no longer drive execution once the dispatch moves to `WallsComputation`, but `infill_pattern = featherprint` is still how the mode is selected.

### 5. Seam Strategy

> ⚠️ **`magic_spiralize` does not work from `wall_toolpaths` and is incompatible with whips.**
>
> - `magic_spiralize` reads `part.spiral_wall` (a plain `Shape` derived from the outline offset),
>   not `part.wall_toolpaths`. The spiralize branch in `FffGcodeWriter` does not consume
>   `wall_toolpaths` at all — it is built only in the `!result.spiralize` branch.
> - `spiralizeWallSlice()` indexes a single closed contour; open whip segments have no closed
>   loop to ramp around.
> - `magic_spiralize` is mesh-group-global — it cannot be enabled per-mesh.
>
> **Seam strategy without spiralize:**
> - Closed layers: use **scarf seam** (already wired through `InsetOrderOptimizer` and it does
>   read `wall_toolpaths`). This ramps extrusion at start/end and substantially reduces the blob.
> - Long-term spiralize: implement a continuous-Z ramp inside the FeatherPrint generator itself,
>   independent of `magic_spiralize`. Re-evaluate native spiralize only after whips are fully
>   specified and only if the generator can also populate a faithful `part.spiral_wall`.

### 6. Whip Compatibility ✅

When whips are implemented, a single layer's output will be multiple segments. Each segment
becomes a separate `ExtrusionLine` with `is_closed_ = false`.

**Confirmed.** `InsetOrderOptimizer::optimize()` branches on `line.is_closed_`: closed lines go
through `insertSeamPoint()` + `addPolygon()`, open lines go straight to `addPolyline()`.
`addToLayer()` emits both via `LayerPlan::addWall(...)`. Open whip segments are printed as
polylines with no forced closure and no spurious seam. A whip layer = several `ExtrusionLine`s
(open perimeter arcs + closed terminal loops) in a single `VariableWidthLines`; the optimiser
iterates and dispatches each correctly. **This is the core premise of the migration and it is sound.**

---

## Files to Modify

| File | Change |
|---|---|
| `include/featherprint/FeatherPrintGenerator.h` | Change signature: returns `VariableWidthLines`, takes `Shape` outline |
| `src/featherprint/FeatherPrintGenerator.cpp` | Rewrite output: `OpenPolyline` → `ExtrusionLine` with `emplace_back` junctions |
| `src/WallsComputation.cpp` | Add FEATHERPRINT branch replacing `WallToolPaths` call; set `inner_area`/`print_outline` |
| `src/infill.cpp` | Remove `FEATHERPRINT` dispatch case and `line_distance_` guard |
| Cura profile | Change `wall_line_count` from 0 to 1; disable `magic_spiralize` |

---

## Effort Estimate

| Task | Complexity |
|---|---|
| Generator outputs `VariableWidthLines` (constant width, `inset_idx_=0`) | Medium |
| Replace `WallToolPaths` call in `WallsComputation`; set `inner_area`/`print_outline` | Low–Medium |
| Keep `wall_line_count = 1`; remove `= 0` requirement | Trivial |
| Remove infill dispatch in `infill.cpp` | Trivial |
| Seam handling via scarf seam on closed layers | Low |
| Whip open-`ExtrusionLine` support | None — already works in the pipeline |

**Total**: approximately one focused session. The principal risk is ensuring the FeatherPrint
part is not culled and that `inner_area`/`print_outline` are populated correctly so that skin,
infill, and support area math does not misbehave on subsequent layers.

---

## Codebase Verification (CuraEngine 5.14.0-alpha.0, commit `e5b844b`, 2026-06-25)

*Note: verified against upstream `main`, not the `featherprint-5.13` fork. Line numbers differ
slightly but the architectural patterns are unchanged between 5.13 and 5.14.*

### Verdict Summary

| # | Claim | Verdict | Consequence |
|---|---|---|---|
| Data structures exist as described | ✅ (field names need `_` suffix) | Cosmetic fix |
| 1 | Generator outputs `VariableWidthLines` | ✅ | No issue |
| 2 | Inject in `FffGcodeWriter::preProcessInsets()` | ❌ | Part culled / guard skips it — move to `WallsComputation` |
| 3 | Remove infill dispatch in `infill.cpp` | ✅ | Trivial, correct |
| 4 | `wall_line_count = 0` stays | ❌ | Must be ≥ 1, or nothing prints |
| 5 | `magic_spiralize` works natively from `wall_toolpaths` | ❌ | Spiralize ignores `wall_toolpaths`; incompatible with whips |
| 6 | Open `ExtrusionLine`s carry whips through unchanged | ✅ | Core premise is sound |

### Evidence

**Data structures** — `ExtrusionJunction` members: `p_`, `w_`, `perimeter_index_`; constructor
`ExtrusionJunction(Point2LL, coord_t, coord_t)`. No default ctor — use `emplace_back`.
`VariableWidthLines` typedef at `ExtrusionLine.h` line 239. `part.wall_toolpaths` at
`sliceDataStorage.h` line 68. A single bin (`inset_idx_ = 0`) may freely mix closed and open lines.

**Injection point** — `preProcessInsets()` at FffGcodeWriter.cpp 2542; early-return at 2555 when
`wall_line_count <= 0`; `InsetOrderOptimizer` constructed at 2844, downstream of the guard.
`generateWalls(SliceLayer*)` at WallsComputation.cpp 108–115 culls parts with empty
`wall_toolpaths`. Therefore FeatherPrint must fill `wall_toolpaths` during `WallsComputation`.

**`wall_line_count`** — `generateWalls(SliceLayerPart*)` early-outs at lines 41–46 when
`wall_count == 0`. `preProcessInsets()` early-outs at line 2555 when `wall_line_count <= 0`.

**Spiralize** — `part.spiral_wall` is a `Shape` at sliceDataStorage.h line 65; populated at
WallsComputation.cpp line 121 from `outline.offset(...)`, never from FeatherPrint output.
`InsetOrderOptimizer` (the `wall_toolpaths` consumer) is constructed only when
`! result.spiralize` (FffGcodeWriter.cpp 2831). The spiralize branch consumes `part.spiral_wall`
exclusively. `spiralizeWallSlice(const Polygon& wall, ...)` indexes one closed contour `% n_points`
— structurally incompatible with open polylines. `magic_spiralize` is mesh-group-global.

**Open polygons / whips** — `InsetOrderOptimizer.cpp` ~129–141: `is_closed_` branches to either
`insertSeamPoint()/addPolygon()` or `addPolyline()`. `addToLayer()` ~196–211 emits via
`LayerPlan::addWall(..., linked_path = !path.is_closed_, ...)`. Open segments print as unclosed
polylines with no forced seam.

**`FEATHERPRINT` enum** — fork-local addition; not present in upstream. Keeping it as a mode
selector read by `WallsComputation` is correct. Ensure it stays before `NONE` in `EnumSettings.h`
to avoid disturbing the `NONE`/`PLUGIN` ordering that the test suite depends on.

---

## Continuous Lift / Inter-Layer Seam Strategies

*Added 28 Jun 26 — outside reviewer recommendations for eliminating the inter-layer blob without
`magic_spiralize`. Ordered from zero-effort to architectural change.*

### Strategy 1 — Seam placement control (zero effort, implement now)

Place the Trace crossover (where the stringer loop re-joins the perimeter) at the same angular
position as the z-seam hint. If the travel distance from end-of-layer to start-of-next-layer is
short enough, Cura's combing router will route the inter-layer hop **inside** the perimeter
without retracting. No retract = no pressure drop = no blob on re-entry.

**What to do in the generator**: the path already starts at helix 0's departure. The end of the
layer should also terminate at helix 0's return point (anchor + w), not at an arbitrary wrap
position. Ensure `seam_s` (start) and the final closing segment both land at the same stringer's
departure/return positions on the perimeter, and set the z-seam hint to the world-space
position of that stringer anchor so `InsetOrderOptimizer` places the seam there.

**Cost**: small adjustment to closing logic already in the generator. No pipeline changes.

---

### Strategy 2 — Coasting + wipe distance (zero code, configure now)

`wall_0_wipe_dist` is already forwarded to `addWall()` through `InsetOrderOptimizer`. A small
wipe distance bleeds nozzle pressure before the end of the closed loop, reducing the pressure
spike on re-entry. Combined with a small coast distance (`coasting_volume`), this is often
sufficient for single-wall paths.

**Recommended starting values** (tune on the test print):
```
wall_0_wipe_dist = 0.2      # mm — bleed ~half a line width before closing
coasting_volume = 0.03      # mm³ — coast just before the seam
coasting_min_volume = 0.5   # mm³ — only coast if extrusion segment is long enough
```

Add these to the FeatherPrint Test Cura profile (`quality_changes/FeatherPrint Test.inst.cfg`).

**Cost**: zero — these are existing Cura settings, no code changes.

---

### Strategy 3 — Continuous path across layers (implement with wall migration)

The real seam fix: the layer's `ExtrusionLine` ends at **exactly** the point where the next
layer's line begins. Combined with a high retract threshold (so the short inter-layer hop
doesn't trigger a retract) and z-hop disabled at the seam position, the nozzle walks up from
one layer to the next without stopping.

**How to implement in the generator**:
- The helix advance is already computed per-layer: `advance = z_mm * pitch_deg_per_mm / 360.0 * arc.total`.
- The end of layer N's path is helix 0's return position: `arc.pointAt(helix_0_s + w_d)`.
- The start of layer N+1's path is helix 0's departure on the next layer: same helix, advanced
  by one layer's pitch.
- If the perimeter is approximately constant (tube), these two points are close in XY and one
  layer height apart in Z — exactly the short hop that combing will route through the interior.

**Settings to expose**:
```
featherprint_seam_retract_threshold = 2.0   # mm travel — don't retract below this
```

Set `retraction_hop_enabled = false` or `retraction_hop = 0` to prevent z-hop inflating the
travel distance at the seam.

**Dependency**: requires the wall pipeline migration (Strategy 3 only works once the path is a
closed `ExtrusionLine` in `wall_toolpaths` with a predictable start/end position per layer).

---

### Strategy 4 — Continuous multi-layer path (post-migration, ambitious)

Emit the entire model as one long continuous extrusion path with Z increments embedded as moves,
rather than as a stack of per-layer `ExtrusionLine` objects. This eliminates all inter-layer
transitions by construction.

**Architecture**:
- `LayerPlan` operates per-layer, so this cannot be done natively within CuraEngine's pipeline.
- Two viable approaches:
  1. **G-code post-processor**: slice normally per-layer, then run a post-processing script that
     stitches consecutive layer seams into continuous Z moves. Feasible because with Strategy 1
     the seam position is predictable. The stitcher finds each `G0` travel between layers at the
     seam angular position and replaces it with a `G1` move (extrusion) with interpolated Z.
  2. **Generator-level multi-layer path**: the FeatherPrint generator already has full helix
     geometry across all layers. A standalone path-planning mode (outside `LayerPlan`) could
     emit the entire helix as a single `OpenPolyline` with Z embedded as a third coordinate,
     then write G-code directly. This is a significant departure from the Cura pipeline and
     would bypass all of Cura's travel, support, skin, and retraction logic — only viable for
     geometry that is purely a single-wall tube with no support requirements.

**Recommended path**: implement the G-code post-processor version first. It is compatible with
the wall pipeline, requires no changes to `LayerPlan` or `FffGcodeWriter`, and can be
implemented as a Cura post-processing script (Python, loaded via `Extensions → Post Processing`).

**Cost**: medium. Requires a reliable seam-position estimate (Strategy 1 is a prerequisite),
a post-processing script that identifies the seam travel and rewrites it as an extruded Z move,
and testing that extrusion rates are correct across the stitch point.

---

### Implementation order

| Step | Strategy | Prerequisite | When |
|---|---|---|---|
| 1 | Seam at stringer crossover + seam hint | None | Now (infill pipeline) |
| 2 | Wipe + coast profile settings | None | Now |
| 3 | Continuous across-layer hop | Wall pipeline migration | After migration |
| 4 | G-code post-processor stitch | Strategy 1 + wall migration | Post-migration |
| 5 | Full multi-layer path | Strategy 4 architecture decision | Future |
