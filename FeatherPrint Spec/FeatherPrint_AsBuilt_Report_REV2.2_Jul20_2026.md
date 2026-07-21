# FeatherPrint As-Built Report — Anchor Distribution (REV 2.2) — 20 Jul 2026

> **Purpose:** As-built record of this session's implementation of Spec REV 2.2's Anchor
> Distribution (Layer-Shift Strategy), for folding into the spec's next revision. Written
> against the code atop commit `1e45aa239` (uncommitted at the time of writing). Where the
> as-built behavior deviates from REV 2.2's literal text — one deviation does, significantly —
> the difference is called out explicitly, per the same convention the REV 2.1 as-built report
> used.

---

## 1. What was implemented, and what deviates from the spec

REV 2.2 (as written, "proposed") specifies: seed **one fixed world point** once at the first
Stringer-carrying layer, then have every layer **independently** project its own Phase Origin
against that same fixed point (no recursive walk, explicitly to avoid drift).

**As-built, this was replaced with a continuity-tracking walk** during testing: the fixed-point
scheme produced large, spurious Phase Origin jumps not just at genuine topology changes
(a hole starting) but at layers with otherwise smooth shape transitions. Root cause: the seed
point is defined at world `x=0`, which is also the symmetry axis of most fuselage/aero
cross-sections — the domain this system targets. A point on or near a shape's axis of symmetry
is nearly equidistant from the corresponding left- and right-side points of the boundary; a
small, smooth asymmetry between layers can flip which one is the *true* nearest point, jumping
the projection to the mirror point on the opposite side of the part (a jump of up to half the
perimeter length), even though the shape itself barely changed.

As-built: each layer's Phase Origin is the nearest point on **that layer's own** outer wall to
the **previous layer's own** Phase Origin — a genuine recursive walk, not independent
re-projection against a fixed target. This directly fixes the jump (the walking target now
drifts naturally with the shape instead of sitting fixed on a degenerate axis), at the
explicitly-accepted cost of reintroducing the drift risk REV 2.2's independent-per-layer design
was meant to avoid: a local shape distortion can now bias every layer above it, not just the one
layer it occurs on. This tradeoff was evaluated and chosen deliberately (not silently) after the
fixed-point version was tested and found to jump.

A **second, unplanned fix** was also made this session, found while investigating related
orientation robustness: the tangent/inward-normal orientation inside `resolveFrame` previously
used two *centroid-relative* geometric tests (unrelated to Anchor Distribution's ray-cast
removal, but the same class of fragility). These were replaced with a single global polygon-
winding test. See §4.

---

## 2. Anchor Distribution — as-built

### 2.1 New per-mesh, per-layer state

`include/sliceDataStorage.h`, `SliceMeshStorage`:

```cpp
std::vector<Point2LL> fp_phase_origin;
```

One entry per layer, alongside `fp_helix_phase`. Replaces the spec's literal single
`fp_phase_origin_seed`/`fp_phase_origin_seeded` (one fixed point + a seeded flag) — see §1 for
why.

### 2.2 The walk (`FffPolygonGenerator.cpp`, the existing helix-phase pre-pass)

Computed sequentially in the same pre-pass loop that already accumulates `fp_helix_phase`
(must run before the parallel wall-generation pass regardless, so no new sequential dependency
was introduced). For each layer, whichever ring of points would drive `arc_total_mm` — the
largest closed polygon part, or (for an interrupted boundary loop) the same virtual full ring
of arcs + gap chords already built for open-manifold helix-phase continuity — is captured into
a local point list (`fp_seed_ring_pts`).

- **"Layer 0"** = the first layer whose largest outline (or virtual ring) passes the same size
  guard `generate()` itself uses (`arc.total < 4.0 * w` returns early there). At that layer,
  the origin is seeded exactly as REV 2.2 specifies: the point on the ring at world `x=0`
  furthest in `+Y`. (Degenerate fallback, not addressed by the spec: if the ring never crosses
  `x=0` at all — the part doesn't straddle the build plate's X origin — falls back to the
  ring's own topmost point.)
- **Every layer after that** walks forward: nearest point on *this* layer's own ring to the
  *previous* layer's own origin (plain point-to-segment min-distance scan, same technique as
  `ArcParam::nearestArcPos`, hand-rolled here since this loop works on a raw point vector, not
  an `ArcParam`).
- A layer with no valid ring (e.g. a momentary gap) carries the previous origin forward
  unchanged rather than leaving a hole in the walk.

### 2.3 `ArcParam::nearestArcPos` replaces `referenceArcPos`

`referenceArcPos` (the old `+X`-from-centroid ray-cast, first-crossing-in-vertex-order, no
nearest-to-centroid disambiguation) is deleted. `ArcParam::nearestArcPos(target)` — a plain
O(n) point-to-segment min-distance scan, no centroid/ray/angle at all — replaces it at both
call sites (`generate()`, `generateFlange()`), each now called with `phase_origin` (this
layer's already-walked origin) instead of `centroid`.

**Both copies of the old bug were fixed, not just one.** `WallsComputation.cpp` had an
independently duplicated copy of the same ray-cast (Step 5 of its open-manifold full-ring
construction, computing `full_ring_arc_ref`) — same bug, same fix technique, hand-duplicated
there since it operates on `WallsComputation`'s own flat `full_ring` point array rather than an
`ArcParam`.

### 2.4 Plumbing

`fp_phase_origin` threads down the same path `fp_helix_phase` already uses:
`SliceMeshStorage::fp_phase_origin[layer_nr]` → `FffPolygonGenerator::processWalls` →
`WallsComputation`'s constructor (new `Point2LL fp_phase_origin` parameter, stored as member
`fp_phase_origin_`) → `fp.generate(...)` / `fp.generateFlange(...)` (new `Point2LL phase_origin`
parameter, default `Point2LL(0,0)`). `generateOpen`/`generateFlangeOpen` don't take it directly —
they consume `OpenLayerParams::full_ring_arc_ref`, which `WallsComputation.cpp` now computes
correctly per §2.3.

### 2.5 Removed the arc-length→angle→arc-length round-trip

Anchors (`anc.s`, `s_mid`, etc.) were already arc-length values before REV 2.2; the as-built
REV 2.1 code converted them to an angle via `atan2` purely to satisfy `buildBlendPlacement`'s
old `(theta_anchor, R_a, ...)` signature, which then immediately ray-cast back to arc-length
internally. This round-trip is now deleted:

- **`buildBlendPlacement`**: signature changed from `(theta_anchor, R_a, x_sign, x_s, x_e,
  centroid, arc, w)` to `(s_anchor, x_sign, x_s, x_e, centroid, arc, w)`. No internal
  `arcLengthAtAngle` call — `s_anchor` is a direct parameter.
- **`appendTrace`**, **`appendLacingTrace`**: same signature change (`theta_anchor, R_a` →
  `s_anchor`). Their call sites in `generate()`/`generateOpen()` already had the real arc-length
  value sitting right there (`anc.s`, `s_mid`) — the `atan2`/`sqrt` computation immediately
  above each call site was deleted along with the guard that used to skip degenerate
  centroid-coincident anchors (`R_a < 1.0`), which no longer has any role once `R_a` isn't
  used.
- **`appendTerminal`**: external signature unchanged (it already took `s_anchor` directly) —
  but internally it was computing `theta_anchor`/`R_a` from `s_anchor` *only* to hand them
  back to `buildBlendPlacement`, which converted them straight back to (approximately) the
  same `s_anchor` via ray-cast. That whole internal computation is deleted; `s_anchor` is
  passed straight through now.
- **`appendFlareRim`**: kept its own angle-based signature (`theta_anchor, R_a, x_sign, ...`)
  unchanged — explicitly out of scope per REV 2.2's own text (Flare Rim's blend-range
  derivation, `theta ± x·w/R_oml`, stays angle-based for now). Since `buildBlendPlacement` no
  longer performs the angle→arc-length conversion itself, `appendFlareRim` now does it
  internally, one line, immediately before its own `buildBlendPlacement` call — a mechanical
  relocation, not a behavior change.

---

## 3. Verification performed

- Rebuilt clean (Release) after each stage; zero new warnings/errors beyond the pre-existing
  unrelated `SupportInfillPart` conversion warnings.
- User tested in the dev build (pre-installer) after the Anchor Distribution implementation:
  "Already works shockingly well out of the box."
- After the continuity-tracking-walk fix for the symmetry-axis jump: "Worked perfectly."
- No formal non-star-convex stress-test model or synchronized-Lacing-crossing sanity check
  (both listed in the primer's own Step 5 verification checklist) was run this session — only
  the user's own real part(s) in the dev build. Flagging this as unverified, not assumed fine.

---

## 4. Unplanned fix found alongside this work: winding-based tangent/normal orientation

Not part of REV 2.2's own scope, but found and fixed in the same session while discussing
Anchor Distribution's robustness pattern, since it's the same underlying failure class.

**Problem:** `resolveFrame` oriented both the tangent (to match the "CCW = increasing
arc-length" convention) and the inward normal using two separate *centroid-relative* geometric
tests — cross product of the tangent against the radial vector `(S - centroid)` for the
tangent's sign, and a dot product against `(centroid - S)` for the normal's sign. Both assume
"toward/away from the centroid" reliably means "outward/inward." That assumption breaks down
in a concave region or near a hole, where the true material-inward direction and the
centroid-relative direction can differ — silently misplacing every feature (Stringer, Lacing,
Whip Terminal, Flare Rim — all of them route through `resolveFrame`) anchored nearby.

**Fix:** for a simple (non-self-intersecting) polygon with a *consistent* winding direction,
"inward" is a topological invariant, not a local geometric test — tangent rotated toward the
interior side (sign fixed by CCW-vs-CW winding) is always correct, at every point, regardless
of concavity. Added `ArcParam::ccw`, computed **once** per `ArcParam`:

- **Closed `Polygon`** (`buildArcParam`): shoelace signed-area sum over all edges;
  `ccw = (signed_area >= 0)`.
- **Open arc** (`buildArcParamOpen`): always `true` — open arcs have no independent signed
  area (not a closed shape), and the caller already guarantees CCW ordering for them (existing
  contract, stated in `generateOpen`'s own doc comment and relied on elsewhere, e.g.
  `WallsComputation.cpp`'s own arc-orientation step).

`resolveFrame` now does: `if (! arc.ccw) { tx = -tx; ty = -ty; }` (orient tangent), then
`nx = -ty, ny = tx` unconditionally (inward normal = tangent rotated +90°, always correct once
tangent orientation is fixed) — no centroid reference at all. The `centroid` parameter is now
unused inside `resolveFrame` (kept in the signature since every caller already has it in scope
for other purposes — marked `(void)centroid`, not removed, to avoid a wider signature-cleanup
cascade not requested this session).

**This is a generalization beyond what was asked for the specific symptom** (the user's
original question was about propagating orientation from "Stringer 1 CCW on layer 0"); the
winding-based fix is stateless and provably correct per-layer, so no propagation/seeding
mechanism was needed at all, unlike Anchor Distribution's Phase Origin (which has a genuine
ambiguity even on a smoothly-varying shape, near a symmetry axis) or Anchor Distribution's
original centroid-ray-cast (whose failure mode — a ray crossing more than once — the fixed-point
projection already replaced without state).

---

## 5. Known loose ends / not yet done

- **Spec status line**: REV 2.2 is still marked "proposed, not yet implemented" as of the spec
  file this report is written against — should be updated to as-built, and the fixed-point vs.
  continuity-walk deviation (§1) documented in the spec text itself, not just this report.
- **Installer not yet rebuilt** against this session's changes (still on 0.4.0, built before
  this session's work).
- **Not yet committed/pushed** — all changes in this report are uncommitted, atop `1e45aa239`.
- The primer's own Step 5 verification checklist (non-star-convex stress test, synchronized-
  crossing sanity check across a plain tube AND a tapered/asymmetric shape) was not formally
  run — see §3.
- The centroid-relative-orientation fix (§4) was scoped narrowly (only `resolveFrame`'s two
  tests). Worth a sweep for any other lingering centroid-relative "which side is inward/outward"
  logic elsewhere in the file before considering this class of bug fully closed — not done this
  session.
