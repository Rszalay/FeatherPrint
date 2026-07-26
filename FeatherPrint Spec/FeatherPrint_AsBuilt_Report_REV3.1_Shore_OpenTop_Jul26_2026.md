# FeatherPrint As-Built Report — Shore (REV 3.1): Open-Top False-Positive Fix — 26 Jul 2026

> **Purpose:** As-built record of a follow-up fix to Shore (Internal Overhang Bridging, REV 3.1),
> shipped in the prior session at commit `f593a624d`. That version was confirmed working on
> real-print tests within the same session, but a distinct bug surfaced on further real-model
> testing this session: Shore fired under regions that never receive a real top skin at all.
> Written against the code atop `f593a624d`, with `src/FffPolygonGenerator.cpp` uncommitted at
> the time of writing.

---

## 1. Problem, as reported

Two related real-model complaints, investigated and fixed in sequence within this session:

1. "Open manifolds still try to generate supports under areas where no top layer generates."
2. After the first fix: "It still puts the shores at the topmost layer, even though the topmost
   layer of one of my test models is open."

Both trace to the same root issue: Shore's Top-Surface test (`T = layer_n \ layer_{n+1}`) reads
per-layer 2D slice geometry, but "the model's top is open" (in the sense the user, and
FeatherPrint's own Flange feature, mean it) is **not a fact visible in that geometry at all**.

---

## 2. Fix 1 — filtered outline, matching what WallsComputation actually prints

**Root cause:** Shore's own `rawSliceOutline()` read `SliceLayerPart::outline` directly — the
raw slice-time contour, before any filtering. `WallsComputation.cpp` (line ~74) already applies
a morphological-open filter (`offset(-w/2).offset(w/2)`) before generating any Wall at all, to
drop features narrower than one line width. On a non-manifold/non-watertight input mesh, the
slicer's own contour-stitching pass can leave thin spurious slivers in a layer's raw outline that
never become real printed geometry — Shore was detecting "Top Surface" in exactly this kind of
sliver, which gets silently dropped before any wall or skin is ever generated there.

**Fix:** new `filteredSliceOutline(layer, line_width)` helper, applying the identical
morphological-open filter (with the same "if opening empties it, fall back to the raw outline"
guard `WallsComputation.cpp` itself uses). Applied to both `outline_li`/`outline_above` (the T
computation) and `envelope_below` (the Enclosure test's OML extent at Layer n-1) — the latter was
susceptible to the same stitching-noise issue for the same reason.

**Confirmed via diagnostic log**, not assumed: temporary logging dumped `parts`/`open_polylines`
counts and `rawOutlineEmpty`/`filteredOutlineEmpty` for the top 5 layers of a real 400-layer test
model. This round wasn't independently re-verified as fully resolving the user's report before
moving to Fix 2 (the user's next report described a related but distinct symptom, addressed
below) — flagged as not separately confirmed in isolation.

---

## 3. Fix 2 (attempt, reverted in substance) — open_polylines-based apex guard

**Hypothesis:** an intentionally-open boundary (Whip Terminal's own open end, or more generally
any region represented via `open_polylines` rather than closed `SliceLayerPart`s) should not be
treated as a true top just because `outline_above` comes out empty. Added two guards: skip Layer
n if Layer n's own `open_polylines` is non-empty, and skip if Layer n+1's `open_polylines` is
non-empty.

**Result: did not fix the reported case.** The user's own topmost layer still produced a Shore
bridge. Rather than guess a third variation blind, temporary diagnostic logging was extended to
record, for the last 5 layers of the model (395–399 of 400), `parts`/`open_polylines` counts and
which guard fired.

**Real diagnostic result:** every one of the last 5 layers, including the true topmost layer
(399, the very last), showed `parts=1`, `open_polylines=0`, with `T` non-empty (cross-sectional
area genuinely shrinking layer to layer). **This proved the entire open_polylines hypothesis
false for this case:** the model's "open top" left zero trace in per-layer 2D topology — the
slicer's ray-casting always produces a closed 2D contour from whatever triangles it hits,
regardless of whether the source mesh has a real face capping that region in 3D. A tapering shape
with a missing cap face and a tapering shape with a genuine closed apex are geometrically
indistinguishable at the per-layer level; there is no 2D signal to detect here at all.

**As-built:** the Layer-n-own-`open_polylines` guard was removed entirely (dead code for this
case — reverted). The Layer-n+1 guard was **kept**, but re-scoped: it remains valid for its
original, narrower purpose (Whip Terminal's own genuinely-open boundary edge continuing upward),
which is a different feature from the open-top case that prompted this investigation.

---

## 4. Fix 3 (as-built) — Flange as the detection signal, not geometry

Per explicit user direction after the geometric dead end was confirmed: **"Let's use flange
generation on as a detection method."** The user already tells the engine "this top is open" by
enabling `featherprint_flange_enabled` — Shore should read that signal directly rather than
re-deriving open-ness from geometry that provably doesn't carry it.

**Implementation:**
- The Flange-start-layer computation (previously running *after* Shore's own pre-pass, per a
  known TODO in the prior session's as-built) was moved to run **before** Shore, so Shore can
  read `mesh.fp_flange_start_layer` directly. The old, now-duplicate copy of this computation
  later in the function was deleted rather than left as dead code.
- Shore's per-layer loop now skips any layer at or above `fp_flange_start_layer` whenever Flange
  is enabled, before ever computing `T` for that layer.
- Consequence: since Shore never runs inside the Flange ramp zone anymore, the previously-flagged
  TODO ("Enclosure's single-line-width inset doesn't account for a thicker Flange Wall stack at
  Layer n-1") is now **moot, not just deferred** — Layer n is already guaranteed below
  `fp_flange_start_layer` by the new guard, so Layer n-1 is strictly further below it too. Updated
  the surrounding comment accordingly rather than leaving a stale TODO.

**Confirmed working:** "Okay, that got it." — real-print test on the same model, Flange enabled.

---

## 5. Deviation from prior settings guidance

The REV 3.1 as-built (previous session) stated Shore "requires `featherprint_flange_enabled =
false`... to do anything useful on a genuinely closed top." **This is superseded.** Flange and
Shore are no longer mutually exclusive settings-wise — Flange enabled now actively *defines* the
boundary of Shore's own operating range (everything below the ramp), rather than being something
the user must disable to use Shore at all. Models with a genuinely closed top (no Flange needed)
continue to work exactly as before, since `fp_flange_start_layer` stays unset (`-1`) when Flange
is disabled and the new guard never fires.

---

## 6. Verification performed

- Rebuilt clean (0 errors) after every round of this session's changes; only the pre-existing,
  unrelated `SupportInfillPart` conversion warnings remain (expected, per build workflow notes).
- Fix 1 (filtered outline): built and handed off; not independently re-confirmed in isolation
  before Fix 2/3 (the user's subsequent reports were about the related-but-distinct open-top
  symptom, not a report that Fix 1 itself regressed).
- Fix 2 (open_polylines guards): confirmed **not** to fix the reported case via real diagnostic
  logging against a real 400-layer model — this is why it was reverted rather than iterated on
  blind.
- Fix 3 (Flange-based guard): confirmed working via real-print test on the same model that
  exposed the bug ("Okay, that got it").
- All temporary diagnostic logging (`shore_dbg`, writing to
  `C:\Users\ricsz\AppData\Local\Temp\FeatherPrint_shore_debug.log`) removed and confirmed absent
  via `grep -rn shore_dbg src/` returning nothing, before this report was written.

---

## 7. Known loose ends / not yet done

- Spec text: REV 3.1's own wording doesn't yet document the Flange-as-open-top-signal
  relationship, nor the settings-guidance reversal in §5 — worth updating next time the spec
  file itself is revised.
- Fix 1 (filtered-outline sliver fix) was validated only indirectly (no regression reported on
  a non-manifold model specifically re-tested after Fix 3 landed) — if a future non-manifold-mesh
  Shore bug appears, re-verify Fix 1 is still doing its job rather than assuming it from this
  report alone.
- The Whip-Terminal open_polylines guard (Layer n+1, kept from Fix 2) has not been specifically
  re-tested this session against a real Whip Terminal model — carried over from the prior
  session's implementation, not newly validated here.
- Installer not yet rebuilt against this session's changes.
- Not yet committed/pushed — `src/FffPolygonGenerator.cpp` is uncommitted, atop `f593a624d`.
