# Conformal Placement Transform

*Specification section for FeatherPrint. Drafted 28 June 26. Intended to slot into the **Geometric Features** group, after **Centroid**, and to be referenced by all print features and collision features as the canonical placement method.*

---

## Purpose

Every print feature and collision feature is defined once, in canonical form, as a 2D extrusion-path profile in the Anchor Point UCS (perimeter along x, inward along −y, dimensions in units of line width *w*). The **Conformal Placement Transform** is the single, shared method that maps a canonical profile onto the actual skin of a given layer at a given anchor point, producing layer-plane toolpath geometry that lies flush against the real perimeter.

This method is feature-agnostic. Lacing, Gusset, Splay, Cuff, and the base Stringer/Whip Traces all use it; they differ only in the canonical profile supplied as input. Defining it once here avoids re-deriving the placement math per feature and guarantees that all features conform to the skin identically.

The transform is purely in-plane (layer XY). The z component of every feature is left flat at the layer plane; the crossover layer lift is applied downstream by the spiralize post-process and is out of scope for this method.

---

## Inputs

| Input | Description |
| --- | --- |
| Canonical profile | The feature's extrusion path in the Anchor Point UCS, as a sequence of line and arc primitives. Coordinates in *w*. |
| Centroid | The layer centroid, **C**, taken from the low-pass-filtered centroid line at this layer (not the raw per-layer bounding-box centre). |
| Perimeter polyline | The layer's slice perimeter as an ordered list of integer points (CuraEngine `Polygons`). The skin is represented as connected straight segments; no analytic curvature exists. |
| Anchor point | The point **A** on the perimeter at which the feature is placed, and its angular position θ_anchor measured about **C**. |
| Line width *w* | Resolves canonical units to mm at emit. |
| Chord tolerance ε | Maximum permitted deviation between a flattened arc and the true arc, in mm. |

---

## Coordinate Frames

The transform maps between two frames:

- **Canonical Cartesian** — the Anchor Point UCS. A canonical point is (x, y): x is the offset along the perimeter from the anchor, y is the inward depth toward the centroid. The anchor is the origin.
- **Layer Polar** — a polar frame centred on the layer centroid **C**. A point is (r, θ): r is the radial distance from **C**, θ is the angular position about **C**.

The skin perimeter radius is the function **R(θ)** — the distance from **C** to the perimeter along the ray at angle θ, found by intersecting that ray with the perimeter polyline. Because the skin is a polyline, R(θ) is piecewise-linear in θ, with slope discontinuities at every perimeter vertex.

**R(θ) is required to be single-valued.** A ray from the centroid must cross the perimeter exactly once. This holds when the slice is star-convex about the centroid. Slices that are reentrant about the centroid (a centroid ray crossing the perimeter more than once) are **disallowed**, and avoiding them is the responsibility of the user supplying the model. Behaviour on such slices is undefined.

---

## The Map

A canonical point (x, y) maps to layer-polar (r, θ) as follows.

### Angular component

$$\theta = \theta_{anchor} + \frac{x}{R_a}$$

where R_a = R(θ_anchor) is the perimeter radius **evaluated once at the anchor**. This converts the canonical along-perimeter offset x into a swept angle, using the anchor-point arc-length scale. Evaluating R at the anchor (rather than integrating R(θ) along the perimeter) keeps θ linear in x. This is justified because every feature is orders of magnitude narrower than the skin radius; the arc-length error introduced across a feature's span is negligible.

This linear-angle map preserves the canonical ratio relationships: for two points, x₁ / x₂ = θ₁' / θ₂' where θ' is the angular offset from the anchor.

### Radial component

$$r = R(\theta) - y$$

where R(θ) is evaluated **per point, at that point's own angle θ** (not at the anchor). This is the defining choice of the method: the baseline radius from which inward depth is subtracted tracks the real skin contour across the feature's angular span. Consequently the feature's perimeter edge (all points with y = 0) follows the actual perimeter polyline, remaining flush with the skin even where the skin curves or kinks within the feature's span. The feature bottom follows the skin; it does not sit on a circular approximation.

### Width behaviour

Because θ is linear in x while r decreases with inward depth y, lines of constant x converge as they descend inward — the feature narrows toward the centroid, subtending a fixed angle over diminishing arc length. The feature's physical width equals its canonical width **only at the skin radius**; inward of the skin it narrows. This convergence is geometrically correct for a feature conforming to a curved skin and is accepted. Its magnitude is negligible in practice, as feature depth is orders of magnitude smaller than R.

### Inverse to layer XY

The layer-plane toolpath point is recovered from (r, θ) about the centroid:

$$P = C + r\,(\cos\theta,\ \sin\theta)$$

---

## Arc Handling

Canonical profiles store arcs as primitives (centre, radius, endpoints), retained through storage and validation. **The Conformal Placement Transform does not preserve arcs.** Because R(θ) varies across a feature's span, the radial map is non-uniform, and a canonical circular arc does not map to any analytic arc in the layer plane — its image is a general curve.

Therefore arcs are **flattened during placement, not at emit**:

1. Each canonical arc is sampled into points in canonical Cartesian space.
2. Every point — line vertices and arc samples alike — is passed through the map above.
3. After deformation, the chord deviation of each flattened arc is checked against ε. Any arc segment exceeding ε is resampled at higher density and re-deformed.

The post-deformation tolerance check is required because the deformation **adds** curvature: the skin's curvature R(θ) compounds with the arc's own curvature. An arc adequately sampled against a flat reference may be under-sampled once laid on a tightly curved region of skin (e.g. a leading edge). Sampling density must be evaluated after deformation, or with a margin sized to the maximum skin curvature within the zone, to prevent visible faceting of loop interiors.

---

## Procedure

For a single feature placement:

1. Resolve the centroid **C** from the filtered centroid line at this layer.
2. Resolve the anchor **A** and its angular position θ_anchor about **C**. Compute R_a = R(θ_anchor).
3. For each primitive in the canonical profile:
   a. If an arc, sample to points at the canonical chord tolerance.
   b. For each point (x, y): compute θ = θ_anchor + x / R_a; compute R(θ) by single-crossing ray-cast against the perimeter polyline; compute r = R(θ) − y; recover P = C + r·(cos θ, sin θ).
4. Check post-deformation chord error of each flattened arc against ε; resample and repeat step 3 for any that exceed it.
5. Pass the resulting layer-plane polyline to emit, which resolves *w* to mm and writes extrusion moves. Z is left at the layer plane for the spiralize post-process.

---

## Notes and Boundary Conditions

- **Centroid source.** The filtered centroid line is used, per the Centroid definition. Using the raw per-layer bounding-box centre would introduce z-axis jitter into θ and R across layers.
- **R(θ) at vertices.** R(θ) has slope discontinuities at perimeter vertices. Under this method (per-point R) these are followed faithfully: a feature spanning a vertex creases to match the skin. This is intended. Severe kinking of features is avoided by the user not supplying geometry whose perimeter kinks sharply within a feature span; the method does not smooth or guard against it.
- **Disallowed geometry.** Reentrant slices (multivalued R(θ)) are the user's responsibility, as above. The method assumes single-crossing throughout.
- **Profile constancy.** For features whose canonical profile is constant across the zone (e.g. Lacing), the canonical sampling in step 3a need be performed once and reused; only the per-point R(θ) evaluation in 3b varies by layer, as the anchor and perimeter change.
