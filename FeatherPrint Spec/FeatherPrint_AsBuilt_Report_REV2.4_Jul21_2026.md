# FeatherPrint As-Built Report — Thin-Section Pruning (REV 2.4) + Inner-Area Fix — 21 Jul 2026

> **Purpose:** As-built record of this session's implementation of Spec REV 2.4's Thin-Section
> Pruning, plus one unplanned pipeline fix found and requested alongside it (populating
> `SliceLayerPart::inner_area` so stock Cura `top_layers`/`bottom_layers` work on FeatherPrint
> parts). Written against the code atop commit `1e45aa239` (uncommitted at the time of
> writing, alongside the still-uncommitted REV 2.2/2.3 Anchor Distribution work from the prior
> session segment — see `FeatherPrint_AsBuilt_Report_REV2.2_Jul20_2026.md`).

---

## 1. Thin-Section Pruning — as-built

### 1.1 Scope: narrower than the spec's literal text

REV 2.4 states Thin-Section Pruning "applies generically to any feature with an anchor and a
Depth — Stringer, Lacing, Whip Terminal, Flare Rim." **As-built, Whip Terminal is deliberately
excluded.** Terminal closes a genuinely open perimeter end; unlike Stringer/Lacing/Flare (where
skipping the feature just means "no extra loop, the ordinary wall passes straight through
instead"), there is no ordinary-wall fallback for an open end — pruning it would leave that end
undrawn entirely, defeating the reason Whip exists (closing the end to avoid a stress riser at
an open filament tip). The spec itself only names Stringer, Lacing, and Flare as the "practical
motivating cases," so implementation was limited to those three. Flagged as a deliberate
narrowing, not an oversight.

### 1.2 `FeatherPrintGenerator::isThinSection` — shared detection helper

New private static method, called from all four anchor-processing sites that needed it
(`generate()`'s ordinary Stringer/Lacing loop, `generateOpen()`'s equivalent, and the Flare Rim
loops in both `generateFlange()` and `generateFlangeOpen()`):

```cpp
static bool isThinSection(const ArcParam& arc, const Point2LL& centroid, double s_anchor, double D, coord_t w);
```

Casts a ray from the anchor (`resolveFrame(arc, centroid, s_anchor)`, reusing the same
winding-based inward normal REV 2.3 already computes — no centroid-directed ray, per the
spec's own stated reasoning) of length `D * w`. Returns true (prune) if that ray hits the arc's
own real perimeter within that distance, subject to the two refinements below.

### 1.3 Integration: skip-and-fall-through, not delete-and-gap

At each of the four call sites, a pruned anchor's feature emission is skipped entirely and
`current_s` (or the Flare loop's equivalent departure/resume bookkeeping) is left untouched, so
the ordinary perimeter walk passes straight through that span exactly as if no anchor were there
— matching the spec's "deletion, not suppression" requirement (the anchor itself, e.g. `anc.s`
or the `FlareAnchor` struct, is never touched; only whether geometry gets emitted for it).

### 1.4 Detection radius and exclusion, as tuned this session

The spec flags both the detection radius (`== D`, no margin) and the anchor-neighborhood
exclusion as explicit, tentative starting points "to be revisited once tested against real
parts." This session did exactly that, through several rounds against a real test print:

**Round 1 (initial implementation):** plain distance-only ray-vs-perimeter hit test, with an
arc-length exclusion window (`2 * w` on each side of the anchor) to avoid a trivial near-zero
self-intersection. **Result:** large sections of Stringer pruned in error, on "flat-ish" (gently
curved but not truly thin) segments specifically — not on true flat sections.

**Root cause of Round 1's false positives:** the ray, cast straight in, could hit the **same**
wall a bit further around its own gentle curvature (or a tiny mesh-slicing kink just past the
exclusion window) — a false positive, since that hit segment is still part of the near wall
curving along broadly the same direction as the anchor's own wall, not a genuinely different far
wall bounding a real gap.

**Round 2 fix — opposing-normal filter:** a hit only counts if the hit segment's own inward
normal (same winding-based rule as `resolveFrame`, computed fresh for that segment) points
*opposite* the ray direction (`dot < 0`, i.e. past perpendicular). This fixed the false
positives.

**Round 2's own regression, found next:** with the opposing-normal filter in place, some
genuinely thin, very tight corners (subsequently identified as **less than 10 degrees**
included angle) stopped pruning — a real thin section going undetected, "features managed to
stick through" at sharp corners.

**Round 3, attempt A (not kept):** detect a sharp bend in the *anchor's own* wall (compare raw
tangent ±2·w before/after the anchor; if the turn exceeds 45°, skip the opposing-normal filter
for that anchor and fall back to plain distance). **Result: did not fix the reported case.**
Reverted rather than left in alongside a second mechanism, to avoid two overlapping,
unvalidated heuristics.

**Round 3, attempt B (not kept):** widen the opposing-normal filter's threshold itself, from
requiring the hit normal past perpendicular (`dot < cos(90°) = 0`) to accepting anything more
than 45° off the ray's own direction (`dot < cos(45°) ≈ 0.707`). **Result: made things worse**
(missed *more* layers of the tight corner) — a strong signal the acceptance angle was never the
actual mechanism at fault.

**Round 4 — actual root cause and fix:** the exclusion window itself, being **arc-length**-based,
was the bug. At a fold tighter than ~10°, the genuine opposing wall (the *other side* of the
point) is arc-length-close to the anchor precisely *because* the fold is so tight — so the
arc-length exclusion window was discarding the real hit as if it were the anchor's own immediate
neighborhood, before the opposing-normal test ever ran. (A world-distance-based window would
have failed the identical way, for the identical reason: world-closeness is exactly what makes
a hit genuine near a tight fold.) Fixed by changing the exclusion criterion from a metric window
(arc-length or world-distance) to **vertex/segment-index adjacency** — only the segment(s)
literally touching the anchor's own vertex (`kExclusionSegs = 1`, i.e. the anchor's own segment
plus its immediate neighbor on each side) are excluded, regardless of how close anything else is
in arc-length or world space. **Confirmed fixed** ("Okay, that got it") after this change.

**As-built exclusion is index/topology-based, not a tunable metric distance** — a structural
change from the original "tentative starting value" framing, not just a retuned constant.

**As-built opposing-normal threshold is the Round 3-B widened value** (45°, `dot < cos(45°)`),
left in place since Round 4's fix is what actually resolved the reported case; the widened
threshold was not re-verified against Round 1's original false-positive scenario (gently curved
flat-ish sections) after Round 4 landed. Flagging this as **not fully re-validated** — worth a
regression check against a model with genuinely gentle curvature (no tight corners at all)
before trusting both rounds' fixes are simultaneously correct.

### 1.5 Final `isThinSection` structure (as-built)

1. Resolve the anchor's frame (`S`, inward normal `dirx,diry`) via `resolveFrame`.
2. Determine the anchor's own segment index; exclude it and its immediate neighbor (index
   distance ≤ 1) from the hit test.
3. For every other segment: ray-vs-segment intersection (standard 2D parametric line
   intersection), requiring the hit to land within the segment (`u ∈ [0,1]`) and within ray
   length (`0 < t ≤ D·w`).
4. For a qualifying hit, compute that segment's own winding-based inward normal and require
   `dot(ray_dir, hit_normal) < cos(45°)` (i.e. more than 45° away from pointing the same way as
   the ray) before accepting it as a genuine opposing wall.
5. Any accepted hit → prune (return true). No hits after scanning all segments → not thin
   (return false).

---

## 2. Unplanned fix: `SliceLayerPart::inner_area` population

Not part of REV 2.4's own text — found and requested after a test print with
`top_layers = 2` produced no top skin at all.

**Root cause:** `skin.cpp`'s top/bottom skin computation seeds directly from
`part.inner_area` (`top_skin = Shape(part.inner_area); bottom_skin = Shape(part.inner_area);`).
`WallsComputation.cpp`'s FeatherPrint branch set `part->inner_area = Shape{}` (empty)
unconditionally for every closed-perimeter FeatherPrint layer, regardless of
`top_layers`/`bottom_layers` — a pre-existing characteristic of how FeatherPrint's wall
generation was wired up from the start (it never populated a "solid interior" concept the way
Arachne's ordinary wall generation does), not something this session's other work introduced or
broke.

**Fix, per explicit direction ("populate an inner area for when there is a surface there; if
there is no surface, the designer means for it to be open"):**

- Added `FeatherPrintGenerator::innerOffset()` (mirrors the existing `seamPoint()` accessor
  pattern) — the inward offset from the layer outline to the innermost printed Wall's own inner
  face, set during `generate()` (a single Wall wide: `inner_offset_ = w`) and `generateFlange()`
  (the full Wall-stack depth to the innermost Wall's inner face:
  `wd_inner.offset + wd_inner.width / 2`).
- `WallsComputation.cpp` now does `part->inner_area = Shape(gen_outline).offset(-fp.innerOffset())`
  instead of `Shape{}`, for closed-perimeter layers only.
- **Open-manifold (Whip) layers are untouched** — they're built via an entirely separate code
  path (`layer->open_polylines`, synthetic `SliceLayerPart`s) that already left `inner_area`
  empty, so an intentionally-open boundary edge (a hole/slot the designer left open) still gets
  no fabricated skin. This is the "if there is no surface, the designer means for it to be
  open" half of the request, satisfied by leaving existing behavior alone rather than by new
  code.

**Confirmed working**: "Printed great" after this fix, with `top_layers`/`bottom_layers` set.

---

## 3. Verification performed

- Rebuilt clean after every round of both fixes; zero new warnings/errors beyond the
  pre-existing unrelated `SupportInfillPart` conversion warnings.
- Thin-Section Pruning: tested against a real print across all 4 rounds described in §1.4;
  final round confirmed fixing the reported <10° tight-corner case.
- Inner-area fix: tested with a real print, `top_layers = 2` set, confirmed producing a capped
  top surface ("printed great").
- **Not verified**: a regression check of Round 4's combined state (vertex-adjacency exclusion
  + 45°-widened opposing-normal threshold) against Round 1's original false-positive scenario
  (gently curved, non-tight-corner sections). See §1.4's closing note.

---

## 4. Known loose ends / not yet done

- Spec status: REV 2.4 (Thin-Section Pruning) is still marked "proposed, not yet implemented"
  in the spec file this report is written against — needs updating to as-built, including the
  scope narrowing (§1.1) and the exclusion-criterion redesign (§1.4, a structural change from
  the spec's own framing, not just a retuned constant).
- The inner-area fix (§2) has no corresponding spec section at all yet — it isn't part of REV
  2.4's text. Worth deciding whether it belongs in REV 2.4 as a related fix, or as its own
  minor revision, when the spec is next updated.
- The 45°-widened opposing-normal threshold's interaction with Round 1's original false-positive
  case is unverified (§1.4, §3) — flagged, not assumed fine.
- Installer not yet rebuilt against this session's changes.
- Not yet committed/pushed — all changes in this report, and the prior REV 2.2/2.3 Anchor
  Distribution work, are uncommitted, atop `1e45aa239`.
