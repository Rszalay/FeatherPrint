# Layer-Shift Anchor Distribution — Implementation Primer (REV 2.2)

> **Spec reference:** `Specifications/FeatherPrint - Spec REV 2.2 - 260720.md`, section
> **Anchor Distribution (Layer-Shift Strategy)** (status: proposed). Read that section
> for the *design rationale*; this document is the *where-to-touch-the-code* companion,
> written from the as-built `9fa2e7fc7` codebase so you don't have to re-derive call
> sites from scratch. Verified against this repo at the time of writing — re-check line
> numbers if the file has moved on since.

## What this change is

Replaces the centroid/angle ray-cast used to locate and distribute Stringer anchors
around a Layer's perimeter (`ArcParam::referenceArcPos`, plus a second, independently
duplicated copy of the same ray-cast in `WallsComputation.cpp`) with a fixed-point
projection and pure arc-length walk. This is the last surviving star-convexity
dependency in the pipeline — the Skin-Normal Tangent-Blend Placement (REV 2.1) already
removed it for placing a feature's *body*; this removes it for locating the feature's
*anchor* in the first place.

**Not in scope for this pass:** Flare Rim's own blend-range derivation
(`theta ± x·w/R_oml`) stays angle-based for now — see REV 2.2 spec's note under
Perimeter Radius R(θ). Don't touch `appendFlareRim`'s anchor math beyond the one
mechanical relocation described in Step 4.

## The bug this fixes, concretely

`ArcParam::referenceArcPos()` (`src/featherprint/FeatherPrintGenerator.cpp:135`) casts
a ray from the centroid along `Y = centroid.Y, +X side` and returns the **first**
crossing found in vertex order. It has no "nearest to centroid" disambiguation — unlike
`arcLengthAtAngle()` (same file, line ~388), which already got that fix earlier this
session for Whip Terminal frame resolution. On a non-star-convex slice (a hole, a
concave tooth, anything the ray can cross more than once) this silently resolves the
reference landmark to the wrong point.

There is a **second, independent copy of this same ray-cast** in
`src/WallsComputation.cpp:257-275` (Step 5 of the full-ring construction, computing
`full_ring_arc_ref`), with the identical bug (first-found crossing, `break` on first
hit, no centroid-distance comparison). This is the one that feeds `generateOpen` /
`generateFlangeOpen` / Flare anchor placement for open-manifold (Whip/Miter) layers.
**Both copies need fixing, not just the `ArcParam` one.**

## Step 1 — New per-mesh state: the fixed world seed point

Add a field alongside the existing `mesh.fp_helix_phase` (declared
`include/sliceDataStorage.h:316`, populated in the pre-pass at
`src/FffPolygonGenerator.cpp:524-535`):

```cpp
Point2LL fp_phase_origin_seed;   // world XY, seeded once
bool     fp_phase_origin_seeded{ false };
```

Seed it inside the existing pre-pass loop (`FffPolygonGenerator.cpp`, the
`for (size_t layer_nr = 0; layer_nr < mesh_layer_count; layer_nr++)` loop that already
walks every layer sequentially to accumulate `fp_helix_phase`). Per the spec, "Layer 0"
for this purpose is **the first layer that actually carries a Stringer** — in practice,
the first layer whose largest polygon passes the same size guard `generate()` already
uses (`arc.total < 4.0 * w` around `FeatherPrintGenerator.cpp:1155` returns early
otherwise). On the first layer where that guard would pass, compute the point on that
layer's outer wall polygon at `x = 0` furthest in `+Y` (world/build-plate axes, not
centroid-relative), store it in `fp_phase_origin_seed`, and set `fp_phase_origin_seeded
= true`. Every layer above just reuses this stored point — it is never recomputed.

*Open question to confirm while implementing:* this codebase doesn't currently appear
to have a bottom-side Flange/exclusion concept (Flange is top-only per the spec), so in
practice this will almost always just be `layer_nr == 0`. Implement the general guard
anyway rather than hardcoding layer 0, in case that assumption is wrong or changes
later.

## Step 2 — Nearest-point projection (replaces both ray-casts)

Add a new method next to `ArcParam::referenceArcPos` (which should be deleted once this
lands, not kept as dead code):

```cpp
// Nearest point on this polyline to an arbitrary world-space target point.
// No centroid, no ray, no angle — a plain point-to-segment min-distance scan.
// Returns the arc-length position of the projection.
double ArcParam::nearestArcPos(const Point2LL& target) const;
```

Implement as an O(n) scan over every edge (same cost order as the ray-cast it
replaces): for each segment, project `target` onto it (clamped to the segment), track
the minimum squared distance, and return the arc-length position of the winning
projection (`cum_len[i] + t * segment_length`). `PolygonUtils::findClosest`
(`include/utils/polygonUtils.h:361`) is a ready-made alternative if you'd rather reuse
existing CuraEngine machinery instead of hand-rolling this — either is fine, the
project doesn't currently depend on `PolygonUtils` inside `FeatherPrintGenerator` so
hand-rolling keeps the dependency footprint the same as today.

Call-site replacements:
- `FeatherPrintGenerator.cpp:550` (`generateFlange`) — `arc_oml.referenceArcPos(centroid)` → `arc_oml.nearestArcPos(mesh's fp_phase_origin_seed)`.
- `FeatherPrintGenerator.cpp:1162` (`generate`) — same replacement.
- `WallsComputation.cpp:257-275` — replace the inline ray-cast loop with the equivalent nearest-point scan against `full_ring` (same target point, same output variable `full_ring_arc_ref`, just a different search).

You'll need to thread the seed point down from `mesh` into `FeatherPrintGenerator` — it
currently isn't a constructor/method parameter anywhere in that class. Follow the same
pattern `fp_helix_phase` already uses (passed as a plain `double` parameter into
`generate()` / `generateFlange()` / `generateOpen()` / `generateFlangeOpen()`, sourced
from `WallsComputation`'s own `fp_helix_phase_` member, itself passed in from the
constructor at `WallsComputation.cpp:36`) — add `fp_phase_origin_seed` the same way.

## Step 3 — CCW/CW mirrored anchor placement

No formula changes needed here — confirmed against the spec discussion that the
existing structure already matches the target design. In `generate()`
(`FeatherPrintGenerator.cpp:1159-1173`):

```cpp
const double arc_ref     = arc.referenceArcPos(centroid);       // → arc.nearestArcPos(seed)
const double ccw_advance = std::fmod(helix_frac * arc.total + arc_ref, arc.total);
const double cw_advance  = std::fmod(arc_ref - helix_frac * arc.total + arc.total, arc.total);
```

Only the `arc_ref` line changes (per Step 2); `ccw_advance`/`cw_advance` and the
per-stringer `i/N * arc.total` offsets below them are already the mirrored-shared-
landmark shape the spec calls for. Same for the open-layer equivalent in
`WallsComputation.cpp` (`ccw_adv_abs`/`cw_adv_abs`, computed from `full_ring_arc_ref`
in three places: `WallsComputation.cpp` open-layer Flange path, and
`FeatherPrintGenerator.cpp:771` and `:1431` which both consume
`params.full_ring_arc_ref` from the struct — these don't need their own edits since
they just read the corrected field, but worth a sanity check after Step 2 lands).

## Step 4 — Remove the theta_anchor/R_a round-trip

Anchors are already arc-length values (`anc.s`, `s_mid`, etc.) *before* the current
code converts them to an angle via `atan2` just to satisfy `buildBlendPlacement`'s
signature — which then immediately converts back to arc-length via one more ray-cast
internally. Delete the round-trip:

- **`buildBlendPlacement`** (`.h:166`, `.cpp:282`): change signature from
  `(theta_anchor, R_a, x_sign, x_s, x_e, centroid, arc, w)` to
  `(s_anchor, x_sign, x_s, x_e, centroid, arc, w)`. Delete the internal
  `arcLengthAtAngle(arc, centroid, theta_anchor)` call and the `(void)R_a;` line —
  `s_anchor` is now a direct parameter, not derived.
- **`appendTrace`** (`.h:228`, `.cpp:968`): same signature change. Call site in
  `generate()` (~line 1282-1292) already has `const double s_anchor = anc.s;` sitting
  right there — delete the `theta_anchor = atan2(dcy, dcx)` / `R_a = sqrt(...)` lines
  above it and pass `s_anchor` straight through.
- **`appendLacingTrace`** (`.h:272`, `.cpp:1014`): same change. Call site (~line
  1259-1274, the Lacing-collision branch inside `generate()`) already computes `s_mid`
  directly — delete the `R_a`/`theta_mid` computation and pass `s_mid`.
- **`appendTerminal`** (`.h:251`, `.cpp:1333`): same signature change. I did not trace
  every call site for this one (Whip endpoints / Miter / Splay stub paths) — grep for
  `appendTerminal(` and confirm each caller already has (or can trivially get) an `s`
  value before converting to `theta_anchor`, same pattern as the two above.
- **`appendFlareRim`** (`.h:296`, `.cpp:1090`): leave the function's own anchor math
  alone (out of scope, see above) — but its signature currently mirrors
  `buildBlendPlacement`'s old one (`theta_anchor, R_a, x_sign`) and just forwards those
  through. Once `buildBlendPlacement` takes `s_anchor` directly, `appendFlareRim` needs
  to do its own `arcLengthAtAngle(arc, centroid, theta_anchor)` conversion internally
  before calling it — a one-line relocation, not a deletion. Confirm this by reading
  `appendFlareRim`'s body before assuming it's a pure passthrough.

## Step 5 — Verification

- Build/slice a deliberately non-star-convex test model (something with a slot or
  concave region that makes a single `+X`-from-centroid ray cross the perimeter more
  than once) and confirm Stringer anchors now resolve where they previously would have
  silently landed on the wrong side of the part.
- Confirm the synchronized-crossing property still holds: on both a plain tube and a
  tapered/asymmetric shape, Lacing collisions should still all land on the same layer
  ring-wide, not drift pair to pair.
- Confirm the open-manifold (Whip/Miter) path still works end-to-end, since Step 2
  touches the shared `WallsComputation.cpp` ring-construction code both open and closed
  layers depend on.
- The debug logging left in `appendTerminal` from the earlier malformed-loop
  investigation (`fpDebugLog` / `[FP-Terminal]` / `[FP-TerminalRaw]`, still flagged as a
  loose end in REV 2.1) may be useful here if anchor placement misbehaves — don't
  remove it as part of this change, that's still a separate cleanup item.

## Explicitly reversible

Per the person requesting this: this is a real minor-revision change, not a discussion
draft, but if it turns out disastrous in practice the whole thing can just be deleted
and reverted to REV 2.1's angle-based anchor resolution — nothing else in the pipeline
depends on the new mechanism existing.
