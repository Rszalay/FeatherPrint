# FeatherPrint As-Built Report — 20 Jul 2026 Session

> **Purpose:** This is an as-built record of what was actually implemented this session, for use in updating `FeatherPrint - Spec REV 2.0 - 260719.md`. Where the as-built behavior differs from what REV 2.0 (or the WIP Parametric Canonical Toolpaths doc) describes, the difference is called out explicitly — this report documents the code as it exists in `src/featherprint/FeatherPrintGenerator.cpp` / `include/featherprint/FeatherPrintGenerator.h` at commit `9fa2e7fc7`, not the spec's intent.

---

## 1. Skin-Normal Tangent-Blend Placement (replaces centroid-radial transform)

Replaces the old centroid-radial `conformPlace` transform outright — one code path, no fallback.

### 1.1 Frame resolution (`resolveFrame`)

A rigid frame `{S, tangent, inward-normal}` is resolved **directly from an arc-length position `s`** on the real perimeter polyline (`ArcParam::pointAt(s)` / `tangentAt(s)`), not from an angle ray-cast against the centroid. `resolveFrame` no longer takes an angle at all — as-built signature is `resolveFrame(arc, centroid, s)`.

- **Tangent** is sampled over a ±0.3mm arc-length window (`pointAt(s - 300) .. pointAt(s + 300)`, coord_t units) rather than read off the single polyline segment straddling `s`. This was added specifically to suppress sub-millimeter mesh-slicing discretization noise (see §5.3).
- **Inward normal**: tangent rotated 90°, sign resolved so it always points toward the centroid (robust regardless of polygon winding — no assumption of CCW).
- Convention: canonical **+Y = inward** (toward centroid) throughout, matching the WIP doc's Point Tables.

### 1.2 Blend placement (`buildBlendPlacement`) — width is now arc-length-exact

**This is a behavior change from the original implementation plan**, made in response to a user report that physical feature width varied with local skin curvature/angle relative to the centroid.

- `theta_anchor`/`R_a` still locate the **anchor itself** by angle + ray-cast (`arcLengthAtAngle`) — this positioning (which anchors go where around the ring) is unchanged.
- Copy A and Copy B (the two rigid frames a feature blends between) are now placed at `s_anchor ± x * w` **by walking the real perimeter's own arc-length** from the anchor's resolved arc-length position — not by converting `x` to an angular offset (`x*w/R_a`) and re-resolving via ray-cast.
- Result: the physical distance between Copy A and Copy B is exactly `(x_e - x_s) * w` mm, independent of local curvature or the anchor's angle relative to the centroid. The previous angle-based approach only preserved width where the perimeter happened to be locally circular about the centroid.

This fix is in the shared placement machinery, so it applies uniformly to all four features below (Stringer, Lacing, Whip Terminal, Flare Rim) since they all route through `buildBlendPlacement`.

---

## 2. Stringer Trace (`appendTrace`)

Settings: `featherprint_stringer_depth` (D), `featherprint_stringer_width` (W).

**As-built formula differs from the spec's literal Point Table** — the crossover geometry was re-derived to fix a real defect (see below).

```
G  = 0.5 (fixed, not a setting)
R2 = W / 2
R1 = R2 + G          <-- NOT "R1 = R2 - G" as an earlier spec reading had it
```

`R1 = R2 + G` never degenerates to ≤0 for any positive W — no lower bound on W is needed for that reason (unlike an `R1 = R2 - G` formulation, which would).

Point Table (canonical, +Y inward), CCW traversal:
```
Start(-G, 0)   P1(R2, R1)   P2(R2, D-R2)   P3(-R2, D-R2)   P4(-R2, R1)   End(G, 0)
Centres: C1(-G, R1)   C2(0, D-R2)   C3(G, R1)
Path: Arc1 Start->P1 (R1, CCW) ; Line1 P1->P2 ; Arc2 P2->P3 (R2, CCW) ;
      Line2 P3->P4 ; Arc3 P4->End (R1, CCW)
```
**Validity:** requires `D >= W + G`, or Line1/Line2 invert (negative length). At `D = W+G` exactly they're zero-length (a valid degenerate case — Arc1 tangent directly to Arc2).

**This profile is a genuine crossover** (P1/P4 sit on the opposite side of the centreline from Start/End), not merely a rejoin at the perimeter — Arc1/Arc3 actually cross the trace's own centerline near the top of the loop. This was the original bug report ("degenerate R1=0 default, not CW/CCW; user wants crossover added") and its resolution.

**Blend range** is widened to `±max(G, R2)` (the profile's actual canonical x-extent, since the arcs reach out to `±R2`), not just `±G` (the literal Start/End points) — using just `±G` would extrapolate the two end frames far beyond where they're geometrically valid, visibly bowing/flattening the trace's middle on curved skin.

**CW helix mirroring**: reverse traversal (End→P4→P3→P2→P1→Start), each arc's direction flipped, `x_sign=-1`, `x_s`/`x_e` swapped. Verified by direct substitution (not assumed/copied from the old asymmetric profile's treatment) — a naive `x_sign` flip alone misplaces the first-emitted point relative to what the caller's stitching requires, and swapping `x_s`/`x_e` alone without flipping `x_sign` fails to mirror the shape at all.

---

## 3. Lacing Trace (`appendLacingTrace`)

Settings: `featherprint_lacing_depth` (D), `featherprint_lacing_width` (W). Anchor = midpoint between two colliding Stringer anchors.

```
G  = 0.5
R1 = 0.5
R2 = max(0, (D - 1) / 2)     <-- Lw = 1 in w-units; profile is scaled by w downstream

x_start = G + R1   ( = C1.X = -C4.X )
c2x     = W/2 - R2 ( = C2.X = -C3.X )
```

Centres: `C1(x_start, R1)`, `C2(c2x, D-R2)`, `C3(-c2x, D-R2)`, `C4(-x_start, R1)`.

Traversed as the **reverse** (End→P7→…→Start) of the spec's own literal table — the spec's table has Start ahead of the anchor and End behind, backwards relative to the caller's stitching (which departs behind the anchor and resumes ahead of it). Each arc's direction is flipped accordingly. Verified tangent-continuous at all 8 junctions.

**Arc directions fixed this session**: originally implemented with reversed CW/CCW flags relative to what actually produces a continuous, correctly-oriented S-link profile; corrected per user report ("Looks like the arc directions are reversed now") by swapping the `cw_arc` boolean on all 4 arc calls.

---

## 4. Whip Terminal (`appendTerminal`)

`R1`, `R2` "match to Stringer" (passed in from the caller using Stringer's own `R1=R2+G`, `R2=W/2`), `D` from `featherprint_stringer_depth`, `W` from `featherprint_stringer_width`.

Point Table (+Y inward):
```
Start(R1,0)  P1(0,R1)  P2(0,D-R2)  P3(R2,D)  P4(W-R2,D)  P5(W,D-R2)  End(W,0)
Centres: C1(R1,R1)  C2(R2,D-R2)  C3(W-R2,D-R2)
Path: Arc1 Start->P1 (R1) ; Line1 P1->P2 ; Arc2 P2->P3 (R2) ;
      Line2 P3->P4 (widened by Splay, see below) ; Arc3 P4->P5 (R2) ; Line3 P5->End
```
**As-built deviation from the WIP doc**: End's y is taken as `0` (matching Start), not the WIP doc's literal `Lw/2` — read as a transcription slip, since `y=0` is what returns the path to the perimeter (matching the older spec's "returns to the perimeter surface" language). *Correction, applied during implementation*: Line3's actual End placement is `bp.place(Ws, 0.5, w)` — i.e. **End.y = 0.5 w-units (Lw/2), not 0** — this is the as-implemented value; the comment block above it is stale/inconsistent with the code and should be corrected when the spec is updated (flagging this discrepancy explicitly rather than silently resolving it, since I did not re-derive which value is structurally correct).

`Ws = W + splay_L` — widens Line2/Arc3/Line3 on the far side to accommodate the Splay collision feature (unchanged placeholder mechanism, not touched this session beyond the D-parameter fix below).

`x_sign = +1` for the start endpoint (`reversed=true` call), `-1` for the end endpoint (`reversed=false` call).

### 4.1 Bug fixed: hardcoded Depth

`appendTerminal` previously used a literal `const double D = 2.5;` instead of respecting `featherprint_stringer_depth`. Fixed by threading a `D` parameter through the function and all 4 call sites.

### 4.2 Bug fixed: malformed/oversized Terminal loops at isolated layers

User-reported: "malformed whip traces at seemingly random intervals... not associated with splays." Root-caused through a multi-stage investigation (see full detail in session history); as-built fixes:

1. **`arcLengthAtAngle` nearest-intersection fix**: the ray-cast from centroid at angle θ now picks the intersection *nearest* the centroid, not the first one found in vertex order — the boundary can be non-star-shaped (holes, concave regions), so a single ray may cross it more than once; picking an arbitrary far-side hit silently resolved a tangent frame to the wrong location.
2. **`resolveFrame` ray-cast-failure fallback fix**: when the ray-cast finds no intersection (normal for an open polyline when θ points past one of its open ends), an arc-length hint extrapolated from the anchor's own known position is used instead of unconditionally snapping to `s=0` (which could jump the far-end Terminal's frame to the polyline's *opposite* end). **Superseded** by the arc-length-only width rewrite in §1.2 — this fallback path no longer exists in the current code since `resolveFrame` no longer ray-casts at all.
3. **`ArcParam::tangentAt` degenerate-segment fix**: zero-length (duplicate-vertex) segments are now skipped when selecting the segment to derive a tangent from — a coincident vertex pair from slicing previously yielded a zero tangent, which the old fallback replaced with an arbitrary `(1,0)` direction unrelated to the real local skin.
4. **Tangent windowing** (§1.1): even after the above fixes, small (5–25°) but genuine vertex-level kinks in the mesh-slicing discretization (right next to an anchor, varying layer-to-layer even though the model geometry is unchanged) still distorted individual frames. Resolved by averaging the tangent over a ±0.3mm arc-length window instead of reading a single raw segment.

---

## 5. Flare Rim (`appendFlareRim`) — Stringer/Lacing × Flange collision

Settings: `featherprint_flare_depth` (D). Width matches the colliding feature's own Width (`featherprint_stringer_width` or `featherprint_lacing_width`).

### 5.1 As-built profile (5-segment, not the WIP doc's literal 3-segment table)

```
Q_eff = D - Q                              (Q defined in §5.2 — NOT a plain Wall count)
R1    = max(0, min(Q_eff, W/2))            (clamped)
drop  = max(0, Q_eff - W/2)                (depth the clamp would otherwise cut off)
```

Mirrored Point Table (canonical, the WIP doc's literal Start/End labels ran backwards relative to the caller's stitching — see §5.4):
```
Start(-W/2,0)  P1(-W/2,drop)  P2(-W/2+R1,drop+R1)  P3(W/2-R1,drop+R1)  P4(W/2,drop)  End(W/2,0)
Centres: C1(-W/2+R1, drop)   C2(W/2-R1, drop)
Path: Line-in Start->P1 ; Arc1 P1->P2 (R1, CW) ; Line-mid P2->P3 ;
      Arc2 P3->P4 (R1, CW) ; Line-out P4->End
```

**This 5-segment profile (with Line-in/Line-out) is new this session**, added specifically so the clamp on `R1` doesn't truncate the requested channel depth — see §5.5. It supersedes both the original WIP doc's 3-segment table (`Arc1→Line1→Arc2`, no lead-in/out) and an earlier this-session revision that had the clamp but no lead-in/out lines.

### 5.2 `Q` = Wall-stack thickness, excluding the innermost Wall — not a Wall count

**As-built deviates from the WIP doc's literal "Wall Number... count of Walls."** Two corrections were made this session:

1. **Thickness, not count**: `Q` is the sum of each Wall's own width (in w-units) across the ramp layer's stack — not a plain Wall count. A plain count under-charges any stack containing a half-width Wall (the outer half-wall at ramp 0, or a buried half-wall at even ramp indices ≥ 2): e.g. ramp 2's 3-Wall stack (full+half+full = 2.5w) would otherwise score the same `Q=3` as ramp 3's 3-Wall stack (full+full+full = 3.0w).
2. **Excludes the innermost Wall**: the sum only covers Walls *outside* the innermost one (`for wi in [0, n-1)`, not `[0, n)`). The innermost Wall is the one the Flare Rim itself replaces, and `oml_shift` (§5.3) already anchors Start/End at the innermost Wall's own centreline — including its width in `Q` double-counted it and undershot `R1` by one Wall's width. (User report: "It's 1Lw short. Probably need to account for the innermost wall being the one we're on.")

### 5.3 `oml_shift` — anchored to the innermost Wall, not the outer Wall

**Bug fixed this session.** `oml_shift` translates the profile's canonical `y=0` baseline into `oml_arc`'s own coordinate frame. It previously used the **outer** Wall's offset (near-zero, only relevant as a ramp-0 half-wall correction) instead of the **innermost** Wall's offset (the Wall the Flare Rim actually replaces, which can sit 1–2.5 line-widths deeper). This left Start/End sitting almost at the OML while the ordinary wall_line segments on either side ran at the innermost Wall's real (much deeper) offset — a large jump/zigzag at every Flare anchor, visually a sawtooth pattern with no visible arcs (the fillets were dwarfed by the jump). Fixed to reference the innermost Wall's own offset in both `generateFlange` and `generateFlangeOpen`.

### 5.4 Traversal direction

Traversed **left-to-right** (increasing `lx`), matching the wall's own increasing-arc-length walk direction — mirrored from the WIP doc's literal Start/End labels (`Start = +W/2, End = -W/2`), which read literally run right-to-left, backwards relative to both that requirement and the caller's own stitching. `x_sign = +1` unconditionally for both Stringer helix directions and for Lacing — the profile is left-right symmetric, so no CW helix mirror is needed (confirmed, not just assumed: applying `x_sign=-1` to a symmetric profile wouldn't change its shape but would reverse emission order, breaking the wall walk's fixed direction requirement).

### 5.5 `R1` clamp and depth preservation

**Bug fixed this session** (user report: "If D gets too large the endpoints of the arc cross over"). Without a clamp, `R1 = D - Q` could exceed `W/2`, making Line-mid's span (`W - 2*R1`) negative — the two end arcs' flat spans cross over each other instead of meeting at a point, self-intersecting the profile.

Fixed with `R1 = min(D - Q, W/2)`. **A second, immediately-following fix** (user report: "we're forcibly shortening the depth") restores the depth the clamp would otherwise cut off: any `Q_eff - W/2` beyond the clamp is picked up by straight vertical Line-in/Line-out segments at each end (§5.1), so the channel still reaches the full requested depth `D - Q` regardless of how `R1` clamps — only the fillet's own radius is capped, not the total depth.

---

## 6. Miter (Whip × Flange) — half-width outer Wall fix

**Bug fixed this session.** User report: "the outermost wall is half thickness [at the first double-wall Flange layer]. On a miter this results in a half size miter terminal and a gap forming at that layer."

Root cause: the Terminal call for the Miter's outer Wall passed `wd.width` (the Wall's own printed width — half-thickness on the first ramp layer) as **both** the canonical-shape scale factor and the printed bead width. The canonical `D`/`R1`/`R2`/`W` values are defined in units of the true FeatherPrint line width, not the local Wall's own width — scaling by `wd.width` shrank the whole Terminal loop to half its intended physical size on that layer, opening a gap against the full-size Terminal in the ordinary Whip layer immediately below.

Fixed in `generateFlangeOpen`: the Terminal's shape is now scaled by the global line width `w` (matching every other Terminal call site), and the emitted junctions' printed bead width is overwritten to `wd.width` afterward — so the bead still prints at half-thickness as intended, but the loop geometry stays full-size and lines up with neighboring layers.

---

## 7. New Cura settings (`packaging/fdmprinter.def.json`)

All in the existing `"featherprint"` category, `enabled: "infill_pattern == 'featherprint'"`, `settable_per_mesh`/`settable_globally: true`:

| Setting | Feature | Default | Units |
|---|---|---|---|
| `featherprint_stringer_depth` | Stringer | 2.5 | × line width |
| `featherprint_stringer_width` | Stringer | 1.5 | × line width |
| `featherprint_lacing_depth` | Lacing | 2.5 | × line width |
| `featherprint_lacing_width` | Lacing | 3.0 | × line width |
| `featherprint_flare_depth` | Flare | 2.5 | × line width |

Whip/Miter add no new settings — `R1`/`R2` derive from `featherprint_stringer_width` (matching Stringer). Flare's Width derives from whichever feature collided (`featherprint_stringer_width` or `featherprint_lacing_width`), not a separate setting.

---

## 8. Known loose ends / not yet cleaned up

- ~~Temporary diagnostic logging left in `appendTerminal`~~ — **removed.** The `fpDebugLog`/`g_fp_debug_z` helpers, the `[FP-Terminal]` bbox/tdot block, and the `[FP-TerminalRaw]` raw-vertex-dump block (added during the malformed-loop investigation, §4.2) have been stripped out; `<cstdio>`/`<cstdlib>`/`<limits>` were dropped from the includes as they had no other use. `generateOpen`/`generateFlangeOpen`'s unused `z` parameter is now explicitly discarded (`(void)z;`) rather than feeding the removed logging.
- **Whip Terminal End.y discrepancy** (§4, Point Table note): the doc-comment block above `appendTerminal` states `End.y = 0` while the actual code places End at `y = 0.5` (Lw/2). This wasn't reconciled this session — the spec update should pick one and correct the other.
- **Splay's exact resulting width** remains an open item (per spec) — untouched this session beyond `appendTerminal`'s D-parameter fix, which applies to Splay's widened Terminal identically since it's the same function.
- **Gusset** (Flare Rim reusing Stringer's own Width) is described in the spec as implemented via the same `appendFlareRim`, but no dedicated Gusset call site was reviewed/touched this session — confirm against the current `generateFlange`/`generateFlangeOpen` call sites before assuming it's wired up identically to the Stringer/Lacing Flare case.

---

## 9. Packaging

- Installer version bumped 0.3.0 → 0.4.0 (`packaging/FeatherPrint.iss`) — a minor bump, since this session's transform swap and toolpath parametrization is a substantial functional change over 0.3.0's Flange/Flare/Miter work, not a patch.
- `installer_output/FeatherPrint-0.4.0-Windows-x64-Setup.exe` compiled successfully against the Release build reflecting all changes above (commit `9fa2e7fc7`).
