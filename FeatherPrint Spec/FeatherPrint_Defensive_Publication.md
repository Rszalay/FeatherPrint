# FeatherPrint: Conformal Geodetic Skin-Support Generation for Single-Wall 3D Prints

### A Defensive Publication and Technical Disclosure

---

**Author:** Rick Szalay
**First drafted:** 10 June 2026
**This consolidated disclosure:** 28 June 2026
**Project repository:** https://github.com/Rszalay/FeatherPrint
**Source documents:** `FeatherPrint_Spec` (system specification) and `FeatherPrint_ConformalPlacement_Spec` (placement transform), both in the `FeatherPrint Spec/` directory of the repository above.

---

## Notice of Defensive Publication

This document is published intentionally as a **defensive publication**. Its purpose is to place the techniques, algorithms, and geometric methods described herein into the public domain as of the publication date, establishing them as prior art. It is intended to be enabling — that is, to teach a person skilled in the fields of additive-manufacturing toolpath generation and slicer engineering how to make and use the methods described — so that these methods cannot subsequently be claimed as the exclusive invention of any other party.

This publication does not assert that the methods herein are necessarily novel or patentable; it asserts only that, as of the date of publication, they are disclosed to the public.

## Rights Dedication (CC0 1.0)

To the extent possible under law, the author has dedicated all copyright and related and neighbouring rights to this document and the methods it describes to the public domain under the **Creative Commons CC0 1.0 Universal Public Domain Dedication**. The work is published from the United States. See https://creativecommons.org/publicdomain/zero/1.0/ for the full text. No warranty of any kind is given, and no warranty of non-infringement of third-party rights is expressed or implied.

## How to Cite / Establish Date

This document is intended to be published as an attachment to a tagged GitHub Release of the FeatherPrint repository, and optionally archived to a timestamping service (e.g. Zenodo via its GitHub integration) to obtain an independent, citable date and DOI. Readers verifying priority should refer to the Release publication date and any associated DOI or archival timestamp rather than to mutable git commit metadata.

---

## Abstract

FeatherPrint is a set of algorithms for a layer-based fused-filament-fabrication (FFF) slicing engine that generate, from a manifold input mesh, a **conformal geodetic support structure welded to a single-line-width skin**, emitted as continuous spiralized extrusion. Rather than filling a shell with conventional volumetric infill, the engine produces complex manifold geometry from a single continuous curve: a single-wall outer **Skin**, transverse thickened bands (**Formers**), and a net of helical reinforcing tubes (**Stringers**) wound around the skin, together forming a geodetic net. The structure is generated as two counter-rotating helices forming a diamond net, with each helix laid down as a stack of inward-projecting loops (**Traces**) that bond to the skin and to the layer below at a crossover, where the spiralize layer lift is also taken.

Where developed features meet on the surface, the engine detects each intersection type in advance and substitutes a defined **collision feature** (Lacing, Gusset, Splay, Cuff) that produces predictable, well-bonded geometry rather than leaving the junction to arbitrary toolpath resolution. Open meshes (those with boundary edges) are closed and terminated by a dedicated **Whip** feature.

All features are defined once as canonical 2D extrusion-path profiles in a per-anchor local coordinate system, and are placed onto the real skin contour of each layer by a single shared **Conformal Placement Transform** that maps each canonical point to a world-space layer-plane coordinate, keeping the feature flush against the actual perimeter even where the skin curves or kinks.

This abstract and the disclosure that follows describe the system in enabling detail.

---

## Disclosure of Inventive Concepts

The following numbered concepts are disclosed individually and in all combinations. They are stated in broad, structured form deliberately, so that the disclosure reads on variations and is not limited to the specific embodiment detailed in the body of this document. The concepts are disclosed as engine-independent methods: each applies to any layer-based FFF slicing engine providing per-layer slice polygons, a perimeter-generation stage, and spiralized continuous extrusion, and is not limited to the CuraEngine reference implementation described in the body. References below to a "slicing engine" encompass any such engine. Terms in bold are defined in the body.

1. A method for generating, in a layer-based slicing engine, a reinforcing support structure for a single-line-width ("single-wall") 3D print, wherein the support structure and the outer skin are emitted as a single continuous extrusion curve with no volumetric infill and no stacked wall loops.

2. The method of concept 1, wherein the reinforcing structure comprises a **geodetic net** formed of two helical families of reinforcing tubes wound around the outer skin, the two families counter-rotating (one clockwise, one counter-clockwise about the build axis) to form a diamond pattern.

3. The method of concept 2, wherein each helical reinforcing tube is constructed layer-by-layer from a closed loop of extrusion ("**Trace**") that departs from the outer perimeter, projects inward toward a layer centroid, and returns to the perimeter, such that stacked Traces form a continuous tube, and wherein the loop closes at a **crossover** that bonds the tube both to the perimeter and to the layer below.

4. The method of concept 3, wherein the per-layer height increment ("layer lift") of a spiralized continuous toolpath is taken at the crossover of each Trace.

5. The method of concept 2, wherein the angular advance per layer of each helix is computed as a running integral over the actual per-layer perimeter length, such that the physical angle between the reinforcing tube and the perimeter is held constant across z even as the cross-sectional perimeter changes (e.g. on a tapered body), the integral being accumulated in a sequential pre-pass prior to a parallelized per-layer toolpath-generation step.

6. The method of concept 2, wherein helix phase is referenced on each layer to a geometrically defined landmark ("**Reference Ray**") — for example the first intersection of a fixed-direction ray cast from the layer centroid with the perimeter polygon — rather than to the polygon's stored starting vertex, thereby maintaining phase continuity across layers when the underlying slicer reindexes polygon vertices.

7. A method for generating a transverse reinforcing band ("**Former**") in a single-wall print by progressively thickening the wall across a span of layers, stepped by a fraction of a line width per layer symmetrically about the perimeter centreline, optionally rendered hollow where the peak thickness exceeds a threshold multiple of the line width.

8. A method, in a single-wall slicing engine, of resolving the intersection of two developed reinforcing features on the surface by **detecting the intersection type in advance and substituting a predefined collision feature** that merges the two parent features into a single continuous, bonded extrusion, rather than allowing a general toolpath planner to resolve the intersection arbitrarily.

9. The method of concept 8, wherein the collision feature substituted for the convergence of two counter-rotating helical Traces at a common perimeter position ("**Lacing**") is a single continuous S-link extrusion path comprising two interlocked loops occupying the interior space the two suppressed Traces would have occupied, the path crossing above the perimeter at the midpoint between the two suppressed anchor positions so that, under the spiralize layer lift, the two filament passes interlock without collision.

10. The method of concept 8, wherein the collision feature substituted where a helix passes through a Former ("**Gusset**") merges the thickened Former profile and the helix crossover into a single continuous extrusion at each affected layer, with the inward extent of the Trace reduced to terminate flush with the inner face of the thickened wall.

11. The method of concept 8, wherein the collision feature substituted where a helix approaches a boundary-edge termination ("**Splay**") progressively tapers the inward extent of the Trace to zero across an approach span and merges the residual Trace into the boundary-termination loop.

12. The method of concept 8, wherein the collision feature substituted where a Former reaches a boundary-edge termination ("**Cuff**") ramps the Former thickening back to one line width as it approaches the boundary and closes the truncated Former profile into the boundary-termination loop in a single continuous extrusion.

13. A method for terminating an open perimeter arising from a boundary edge of an open input mesh ("**Whip**") by turning the perimeter path, at an anchor point, into a small closed loop ("**Terminal**") that closes back on itself, such that stacked Terminals form a bonded column at the opening, eliminating the unbonded filament end that would otherwise form a stress riser; optionally bracketed above and below by automatically generated support stubs.

14. A method ("**Conformal Placement Transform**") for placing a feature, defined once as a canonical 2D extrusion-path profile in a local per-anchor coordinate system, onto the real skin contour of a layer, comprising: mapping the canonical along-perimeter coordinate to a swept angle about a layer centroid using an arc-length scale evaluated once at the anchor; and computing the radial coordinate of each mapped point by subtracting the canonical inward depth from the perimeter radius **evaluated per point at that point's own angle**, so that the feature's perimeter edge follows the actual perimeter polyline and remains flush with the skin across the feature's angular span.

15. The method of concept 14, wherein canonical arc primitives are flattened by sampling and the flattened arcs are checked for chord deviation **after** the mapping is applied, so that additional curvature introduced by the skin contour is accounted for and resampled to tolerance, rather than checking sampling adequacy only against the flat canonical reference.

16. The method of concept 14, wherein the perimeter radius function is required to be single-valued about the centroid (a ray from the centroid crossing the perimeter exactly once), and wherein the centroid used is taken from a centroid line low-pass filtered along the build axis rather than from the raw per-layer bounding-box centre.

17. The method of concept 14, wherein a mirrored canonical profile (reflected in the local coordinate system, with arc traversal order and direction reversed) is used to place the opposite-handed helix family from the same canonical definition.

---

*The remainder of this document is the consolidated, enabling technical specification, comprising the FeatherPrint system specification followed by the Conformal Placement Transform specification.*

---

# Part I — FeatherPrint System Specification

*The following is the consolidated technical specification of the FeatherPrint system, first drafted 10 June 2026 and revised 28 June 2026. Figures referenced below are dimensioned drawings held in the `images/` directory of the project repository; the accompanying prose is self-contained and enabling.*

## Description
This document specifies an algorithm for producing conformal, single-line-width skin structures in 3D prints, implemented within a layer-based FFF slicing engine. The method is engine-independent: it requires only the capabilities common to such engines (per-layer slice polygons, a perimeter/wall generation stage, and spiralized continuous extrusion), and is not specific to any one slicer. A reference implementation is built as a modification of the CuraEngine (the core slicer of Ultimaker's Cura), and Cura-specific details are given in the Implementation Notes as a concrete demonstration; they are illustrative of one realization, not requirements of the method. The intended application is the development of ultralight parts for drones and other aerodynamic structures, including model aircraft, wind turbines, and hydrodynamic surfaces.

## Input Model
FeatherPrint expects a manifold STL file as input. A **manifold** is a closed, watertight mesh in which every edge is shared by exactly two faces, with no gaps, holes, or self-intersections. This condition ensures that the mesh encloses a well-defined volume and that every cross-section at a given z-height produces a closed polygon.

A manifold is composed of **surfaces** — the individual planar or curved faces that together form the outer boundary of the model. In the context of FeatherPrint, surfaces are the model-space counterparts to the Skin feature: the faces of the input mesh from which the outer skin geometry is derived.

FeatherPrint also accepts open manifolds — meshes that contain one or more **boundary edges**, where an edge belongs to only one face rather than two. An open manifold produces one or more open polygons at the affected slice layers. These open polygons are handled by the Whip feature, which closes and terminates them. All other features assume a closed perimeter at each layer.

## Terminology
### Print Features

Print features are described across four spaces, corresponding to the pipeline from input model through to physical output.

| Print Feature | Manifold Feature | Developed Geometry | Slice Geometry |
| --- | --- | --- | --- |
| Skin | Surface | Skin | Perimeter |
| Former | N/A | Former | Hoop |
| Stringer | N/A | Stringer | Trace |
| Whip | Boundary Edge | Whip | Terminal |

#### Skin
Skin is the most basic feature, consisting of a stack of single-line-width extrusions. The specification uses "Skin" rather than the more conventional "Wall" to distinguish this feature from the other multilayer features defined below.

This distinction is necessary because, strictly speaking, all geometry generated by FeatherPrint is wall: the engine's purpose is to produce complex manifold geometry from a single continuous curve, with no infill or traditional wall-loop stacking. "Wall" alone is therefore too generic to single out any one feature. Skin specifically refers to the segments of that geometry corresponding to the model's surface — the portion of each slice that traces the outer perimeter, as opposed to internal features like Formers or Stringers.

#### Former
The Former is a feature consisting of a localised thickening of the skin at a set of slices. This thickening corresponds with the crossing of stringer helices, suppressing the geometric irregularity at the crossing while also serving as a transverse brace to the geodetic net.

Formers must be multilayer because the thickening occurs across several z layers, and because the stringers must cross over a span of layers to complete the helix transition. The thickness of the Former is stepped out by half a line width per layer. The peak thickness is therefore determined by the line width and the total number of layers comprising the Former. Where the peak thickness exceeds two line widths, the Former becomes hollow.

#### Stringer
A Stringer is a multilayer print feature which, together with the Former, forms the geodetic net of the conformal support. It consists of a tube running primarily in the z direction in a helix around the outer skin. At each layer, the tube is formed by a loop of extrusion — called a Trace — extending inward toward the centroid. When stacked across layers, these Traces form a continuous tube.

Each Trace requires a crossover to close the loop. This crossover firmly bonds the perimeter extrusions together across the Trace and welds the Trace to the layer below. Squeezout from the crossover fills the divot left by the radii of the loops forming the Trace.

The crossover required by each Trace is also where the layer lift occurs.

FeatherPrint generates two counter-rotating helices — one CCW and one CW — which together form the geodesic diamond pattern of the net. Each helix produces its own set of Traces on every layer. The two helices are interleaved: both start from the same reference ray on each layer, with the CCW helix advancing in the positive arc direction and the CW helix advancing by the same amount in the negative arc direction. When a CCW Trace and a CW Trace approach the same perimeter location, a Lacing collision feature replaces both Traces.

#### Whip
A Whip is a termination feature applied wherever the input manifold contains a boundary edge, producing an open perimeter polygon at the affected slice layers. At each such layer, the open end of the perimeter terminates at the anchor point, where the path turns into a small closed loop — called a Terminal — which closes back onto itself. This eliminates the stress riser that would otherwise occur at an open filament end, and forms a well-bonded column of material at the opening as Terminals stack across layers. A parametric cut-back of the perimeter end prior to the Terminal is reserved for a future revision.

---

### Collision Features

Collision features arise wherever two developed features meet on the surface. Rather than allowing the toolpath planner to resolve these intersections arbitrarily, FeatherPrint detects each collision type in advance and substitutes a defined collision feature that produces predictable geometry and preserves bond integrity at the junction.

Collision features are described across the same two spaces used for the print features that give rise to them — developed geometry and slice geometry. They have no independent manifold representation, as they are derived entirely from the spatial relationship between two parent features.

| Collision | Print Feature | Developed Geometry | Slice Geometry |
| --- | --- | --- | --- |
| Stringer × Stringer | Lacing | Lacing | Lacing Trace |
| Stringer × Former | Gusset | Gusset | Gusset Hoop |
| Stringer × Whip | Splay | Splay | Splay Terminal |
| Former × Whip | Cuff | Cuff | Cuff Terminal |

#### Lacing (Stringer × Stringer)

A Lacing occurs where the CCW and CW Stringer helices converge at the same angular position on the perimeter. Because each Stringer advances at a fixed helical pitch, the two helices will periodically arrive within one line width of each other. When a CCW Trace anchor and a CW Trace anchor are within one line width of each other in world space, both standard Traces are suppressed and the Lacing Trace is substituted.

The Lacing Trace is an S-link: a single continuous extrusion path symmetric about the midpoint between the two suppressed anchors, consisting of two interlocked loops that together occupy the same interior space as the two standard Traces would have. The path crosses 0.25w above the perimeter at the midpoint, bonding the two loops at the crossover. In the spiralised print the crossover is separated vertically by the layer-height lift, allowing the two filament passes to interlock without collision.

#### Gusset (Stringer × Former)

A Gusset occurs where a Stringer helix intersects a Former band. This is the most frequent collision type in a typical geodetic layout, as every Stringer must pass through every Former it encounters as it ascends the z layers. The Former already exists partly to manage this interaction, but the Gusset is the explicit developed feature that defines how the Stringer geometry transitions through the Former zone.

In developed geometry, the Gusset spans the same layer range as the Former. Within this range, the Stringer tube does not terminate; instead, its Trace geometry is modified to integrate with the thickened Former wall. The inward extent of each Trace is reduced so that the Trace terminates flush with the inner face of the Former rather than projecting past it. This prevents the Stringer loop from creating a void inside the Former wall at layers where the Former has grown beyond one line width.

At each layer within the Former zone, the Gusset Hoop substitutes for both the standard Hoop of the Former and the standard Trace of the Stringer. The Gusset Hoop traces the thickened perimeter profile of the Former while incorporating the Stringer crossover at the angular position of the Stringer. This merges the two features into a single continuous extrusion at each affected layer, eliminating the seam that would otherwise occur between the Hoop and the Trace.

#### Splay (Stringer × Whip)

A Splay occurs where a Stringer helix arrives at the angular position of a Whip — that is, where the ascending Stringer tube reaches the boundary edge of an open manifold. Because the Whip replaces the perimeter with a closed Terminal loop at the open edge, the Stringer cannot continue past it. The Splay is the developed feature that terminates the Stringer cleanly at the Whip boundary.

In developed geometry, the Splay spans the final layers of the Stringer's approach to the Whip, beginning at the layer where the Stringer's angular position first coincides with the Terminal column. Over this approach zone, the Stringer Trace is progressively reduced in inward extent across successive layers, tapering the tube cross-section down to zero depth at the layer where the Terminal takes over. This avoids an abrupt termination that would leave an unbonded filament end within the structure.

At the Terminal layer, the Splay Terminal merges the closing loop of the Whip with the final reduced Trace of the Stringer. The Splay Terminal is geometrically identical to a standard Terminal but is positioned at the Stringer's anchor point rather than at the ends of the open perimeter, and its diameter is sized to match the residual cross-section of the tapered Stringer tube at that layer.

#### Cuff (Former × Whip)

A Cuff occurs where a Former band reaches the angular extent of a Whip — that is, where the thickened Former zone runs up to a boundary edge on the open manifold. The Former cannot continue past the Whip boundary, and the Terminal column of the Whip sits within the angular range that the Former would otherwise occupy. The Cuff is the developed feature that resolves the transition between the Former's thickened wall and the Whip's Terminal column.

In developed geometry, the Cuff spans the layers of the Former zone that are adjacent to the Whip boundary. Within this range, the Former's stepped thickening is truncated at the angular position of the Whip, and the wall thickness is ramped back down to one line width as it approaches the boundary edge. This prevents the Former from attempting to thicken into the open perimeter region where no substrate exists.

At the boundary-adjacent layers, the Cuff Terminal substitutes for both the standard Hoop of the Former and the standard Terminal of the Whip. The Cuff Terminal traces the truncated Former profile up to the boundary position and then closes with the Terminal loop, bonding the end of the Former wall to the Terminal column in a single continuous extrusion. This ensures that the Former's transverse bracing function extends as close to the open edge as the geometry permits, with the Terminal column providing the closing structure.

---

### Geometric Features

#### Rays
Rays extend from the centroid of a layer outward to the perimeter.

##### Anchor Ray
The Anchor Ray extends from the centroid to a specific point on the perimeter. This point defines the origin and local coordinate system from which all points in a print feature are specified.

##### Reference Ray
The Reference Ray is a stable per-layer geometric landmark used to anchor helix phase across layers. It is defined as the first intersection of the +X ray from the layer centroid with the perimeter polygon, found by ray-casting. Because this intersection is defined geometrically rather than by polygon vertex order, it is invariant to changes in the starting vertex of the perimeter polygon between layers — eliminating the phase discontinuities that would otherwise occur when the slicer reindexes the polygon.

##### Fixed Ray
The Fixed Ray defines the helix of the Stringers as they ascend the z layers. It is indexed by the pitch of the helix at each layer, and serves as the starting ray from which all other rays are laid out. During the relaxation algorithm, the Fixed Ray does not move.

#### Anchor Point
An Anchor Point is a point on the perimeter of a layer that serves as the local coordinate system for the points making up a print feature's curve. All points required to print a feature are transformed from their feature-space definition to the Anchor Point before being connected.

The basis of the Anchor Point's UCS is defined as:
- **y** — the direction from the Anchor Point toward the centroid.
- **z** — parallel to the print z axis.
- **x** — mutually orthogonal to y and z, pointing in the counter-clockwise direction relative to the centroid.

#### Centroid
The centroid of a slice is the geometric centre of the bounding box encompassing all geometry on a given layer. The centroid is calculated per layer; the resulting centroid line — connecting all per-layer centres along z — is then low-pass filtered along the z axis.

#### Perimeter Radius R(θ)
The perimeter radius R(θ) is the distance from the centroid **C** to the perimeter along the ray at angle θ. It is computed by ray-casting from the centroid against the perimeter polyline at each queried angle. Because the perimeter is a polyline, R(θ) is piecewise-linear in θ, with slope discontinuities at perimeter vertices.

R(θ) is required to be single-valued: a ray from the centroid must intersect the perimeter exactly once. This holds for star-convex slices. Reentrant slices — in which a ray from the centroid crosses the perimeter more than once — are disallowed; behaviour on such slices is undefined.

#### Conformal Placement Transform

Every print feature and collision feature is defined once in canonical form as a 2D extrusion-path profile in the Anchor Point UCS (perimeter along x, inward along −y, dimensions in units of line width *w*). The **Conformal Placement Transform** maps each canonical point to a world-space layer-plane coordinate, placing the feature flush against the actual skin of the layer at the given anchor point.

The transform is feature-agnostic. All features — Stringer Trace, Lacing Trace, and future features — use the same method and differ only in the canonical profile supplied.

##### The Map

A canonical point (x, y) maps to a world-space point **P** as follows.

**Angular component:**

θ = θ_anchor + x / R_a

where R_a = R(θ_anchor) is the perimeter radius evaluated once at the anchor point. This converts the canonical along-perimeter offset x into a swept angle using the anchor-point arc-length scale. Evaluating R at the anchor (rather than integrating R(θ) across the feature) keeps θ linear in x, which is justified because every feature is orders of magnitude narrower than the perimeter radius.

**Radial component:**

r = R(θ) − y

where R(θ) is evaluated per point, at that point's own angle θ. This is the defining choice of the method: the baseline radius from which inward depth is subtracted tracks the real skin contour across the feature's angular span. The feature's perimeter edge (all points with y = 0) follows the actual perimeter polyline, remaining flush with the skin even where the skin curves or kinks within the feature's span.

**World coordinate:**

**P** = **C** + r (cos θ, sin θ)

##### Arc Handling

Canonical profiles store arcs as primitives (centre, radius, start/end angles). The Conformal Placement Transform does not preserve arcs — because R(θ) varies across a feature's span, a canonical circular arc does not map to any analytic arc in the layer plane. Arcs are therefore flattened by sampling at a fixed number of segments, with each sample point passed individually through the map above.

##### CW Helix Mirroring

For CW helix Traces, the canonical profile is the mirror image of the CCW profile about the y-axis (x → −x), achieved by setting x_sign = −1 in the angular component: θ = θ_anchor − x / R_a. The arc traversal order and arc direction (CW/CCW) are also reversed so that the mirrored trace departs from the same perimeter cutout position as the CCW trace (s_anchor − w) and returns to the same position (s_anchor + w), with the loop leaning in the CW direction.

---

## Feature Geometry

This section provides precise geometric descriptions of each print feature and collision feature. Each feature is documented across three views:

- **Profile** — a dimensioned 2D drawing of the extrusion path in the Anchor Point UCS. All dimensions are expressed in units of line width *w* and layer height *h*. The profile is the canonical geometric definition from which the toolpath is derived.
- **Slice** — a 3D illustration of the feature at a single layer, showing its position and extent on the perimeter in context with the centroid and surrounding geometry.
- **Developed** — a 3D illustration of the feature across its full layer stack, showing the complete spatial form of the developed structure.

All drawings share the coordinate conventions established in the Reference Drawing below.

---

### Reference Drawing

*[Drawing: coordinate system reference — layer plane, centroid, perimeter, Anchor Point UCS with x/y/z axes labelled, Fixed Ray, Reference Ray, and Anchor Ray shown.]*

---

### Former

#### Profile

**[Figure: Former — Profile — see `images/` in the project repository.]**

The profile is a cross-section through the Former wall in the z direction, showing the full layer stack at the anchor point. The base Skin extrusions below and above the Former band have width w and layer height h. Each ramp layer steps the wall outward by w/2 on each side relative to the layer below it, growing the wall symmetrically about the perimeter centreline.

The number of ramp layers *n* is a slicer parameter. This gives a total Former band height of (2n + 1) layers — n ramp-up layers, one peak layer, and n ramp-down layers — and a peak wall thickness of (1 + n)w. The profile shown uses n = 3, giving a band of 7 layers and a peak thickness of 4w. The w/2 step per ramp layer produces a 45° overhang on the outer face of the ramp, which is the printability limit for FFF without support.

Where the peak thickness exceeds 2w, the Hoop at the peak layer becomes hollow. The hollow condition prevents excess material accumulation at the apex while preserving the outer wall profile.

#### Developed

**[Figure: Former — Developed — see `images/` in the project repository.]**

The Developed view shows the Former band across its full layer span on a straight perimeter segment. The wall width increases symmetrically from Skin width toward the peak layer and returns symmetrically on the far side. The colour coding across layers corresponds to the ramp position: Skin layers at the outer edges of the band, ramp layers stepping inward to the peak. The band is continuous along the perimeter at each layer — the Hoop traces the full perimeter circuit at the thickened width before returning to Skin width on subsequent layers.

---

### Stringer

#### Profile

**[Figure: Stringer — Profile — see `images/` in the project repository.]**

The Trace profile is defined in the Anchor Point UCS, with the perimeter running along the x axis and the inward direction along −y. The perimeter cutout spans from x = −w to x = +w (one line width on each side of the anchor point). At the anchor point the perimeter extrusion splits, and a closed teardrop loop descends inward toward the centroid.

The loop is formed by three arcs:

| Arc | Centre (w units) | Radius | Direction | Span |
| --- | --- | --- | --- | --- |
| Left upper | (−1, −1.5) | 1.5w | CW | 90° → −41.4° |
| Lower | (0, −1.5) | 1.0w | CW | −82.8° → −97.2° |
| Right upper | (+1, −1.5) | 1.5w | CW | 221.4° → 90° |

The departure point is (−1, 0) and the return point is (+1, 0). The total inward depth of the loop is 2.5w. The crossover where the outbound and return paths re-merge is at the top of the loop; a small squeezout from the crossover fills the divot at the perimeter. The layer lift occurs at the crossover.

The CCW Trace departs from x = −w and the loop leans toward the negative-x (CCW-backward) direction. The CW Trace uses the mirror profile: the arc traversal order and arc directions are reversed, and x_sign = −1 is applied in the Conformal Placement Transform, so the loop leans toward the positive-x (CW-backward) direction. Both Traces use the same perimeter cutout (s_anchor − w to s_anchor + w).

#### Slice

**[Figure: Stringer — Slice — see `images/` in the project repository.]**

The Slice shows an isolated Trace in three dimensions. The perimeter extrusion runs as a continuous ribbon; at the anchor point the ribbon opens and the Trace loop descends inward, forming a closed torus-like ring below the perimeter plane. The crossover is visible at the top of the loop where the outbound and return paths re-merge with the perimeter. The loop interior is open — the void enclosed by the loop and the layer below it is not filled.

#### Developed

**[Figure: Stringer — Developed — see `images/` in the project repository.]**

The Developed view shows a Stringer across several successive layers. As the helix advances, the anchor point of each Trace steps along the perimeter at the fixed helical pitch, so successive Traces are offset laterally from one another. When stacked, the individual Trace loops coil into a continuous helical tube. The perimeter extrusions of each layer run as parallel ribbons offset by the layer height; the tube grows between them as the Traces accumulate.

#### Helix Phase and Angle

The angular advance of the helix per layer is determined by the **Helix Angle** parameter θ_h (degrees, default 45°). θ_h is the angle between the stringer tube axis and the skin surface: at 45° the CCW and CW helices cross at 90° to each other, forming square diamonds.

To maintain a constant intersection angle across tapered tubes (where the perimeter circumference changes with z), the helical pitch is not fixed in mm/layer but is computed as a running integral over the actual per-layer perimeter length:

> Δphase = layer_height / (perimeter_length × tan θ_h)

This integral is accumulated sequentially across all layers in a pre-pass before the parallel wall-generation step. The result is stored as a fractional helix phase per layer, which is passed to the per-layer toolpath generator. On a constant-diameter tube, this reduces to a fixed advance per layer. On a tapered tube, the advance per layer adjusts so that the physical angle between the stringer and the perimeter remains θ_h at every z height.

The phase reference for each layer is the **Reference Ray** (+X ray from centroid to perimeter), not the polygon's starting vertex. This ensures phase continuity across layers even when the slicer reindexes the polygon vertex order between layers.

---

### Whip

#### Profile

**[Figure: Whip — Profile — see `images/` in the project repository.]**

The Terminal profile is geometrically similar to the Stringer Trace — total loop depth 2.5w, upper radius R1.5w, lower radius R1w — but is asymmetric about the anchor point. The loop is tangent to the y-axis of the Anchor Point UCS: its leftmost edge sits flush with the anchor point, and the entire loop extends to the boundary side. The perimeter runs inward from the open end of the manifold to the anchor point and terminates there; the loop closes back onto itself at that point rather than bridging across two perimeter segments. The overall loop width of 2w is measured from the anchor point outward to the boundary side only.

#### Slice

**[Figure: Whip — Slice — see `images/` in the project repository.]**

The Slice shows a single Terminal in three dimensions. One perimeter arm extends away from the boundary edge; the boundary-side end terminates at the anchor point, where the path turns into the closing loop. The loop is a full closed ring, with the crossover at the top re-merging the path back onto the perimeter arm. The geometry on the open side of the anchor point is absent — no perimeter extrusion extends past the Terminal.

#### Developed

**[Figure: Whip — Developed — see `images/` in the project repository.]**

The Developed view shows the Terminal column stacking across layers at the boundary edge. The perimeter arms extend to the open side only; the boundary side shows the growing column of stacked Terminal loops. The transparent elements flanking the column are dedicated support stubs — short Stringer-like tubes generated automatically by the slicer at the Whip location. The stubs begin *n* layers below the first Terminal layer and extend *n* layers above the last, bracketing the full column on both sides. This prevents tearing at the upper and lower corners of the column where the Terminal transitions to and from the Skin. These stubs are not part of the geodetic net; they exist solely to provide a bonded substrate for the Terminal column throughout its height.

---

### Collision Features

#### Lacing (Stringer × Stringer)

##### Profile

**[Figure: Lacing — Profile — see `images/` in the project repository.]**

The Lacing Trace profile is symmetric about the y-axis of the Anchor Point UCS. The anchor point is the midpoint between the two suppressed Stringer anchors. The perimeter cutout spans from x = −0.75w to x = +0.75w.

The path is a single continuous extrusion forming an S-link. Starting from the left perimeter cutout edge and ending at the right:

| Segment | Description | Start | End |
| --- | --- | --- | --- |
| Left inner arc | Centre (−0.75, −0.5), R = 0.5w, CW 90°→ −90° | (−0.75, 0) | (−0.75, −1.0) |
| Left outer arc | Centre (−0.75, −1.75), R = 0.75w, CCW 90°→ 270° | (−0.75, −1.0) | (−0.75, −2.5) |
| Bottom segment | Horizontal at y = −2.5 | (−0.75, −2.5) | (+0.75, −2.5) |
| Right outer arc | Centre (+0.75, −1.75), R = 0.75w, CCW −90°→ 90° | (+0.75, −2.5) | (+0.75, −1.0) |
| Right inner arc | Centre (+0.75, −0.5), R = 0.5w, CW −90°→ 90° | (+0.75, −1.0) | (+0.75, 0) |

The inner arcs (R = 0.5w) each create a small eye that opens toward the centre of the feature (toward x = 0). The outer arcs (R = 0.75w) sweep outward to ±1.5w, forming the structural outer loops of the S-link. The outer arcs chain directly from the inner arc ends with no intervening straight segment; the arc radii together set the total feature depth of 2.5w. The path crosses 0.25w above the perimeter at the centre — this crossover is visible as a small nub at the top of the profile.

The total inward depth of 2.5w matches the standard Stringer Trace, preserving each Stringer's structural tube cross-section through the Lacing zone. The total lateral span of the feature is 3.0w (±1.5w from the anchor midpoint).

All arc endpoints are mapped to world-space toolpath coordinates via the Conformal Placement Transform with x_sign = +1. The lacing is not mirrored; its canonical geometry is explicitly symmetric.

##### Slice

**[Figure: Lacing — Entry Trace — see `images/` in the project repository.]**

The Entry Trace shows the last layer before the Lacing zone begins. Two standard Traces sit side by side at their coincident perimeter position — one CCW, one CW. On the next layer, these two separate Traces are replaced by the Lacing Trace.

**[Figure: Lacing — Lacing Trace — see `images/` in the project repository.]**

The Lacing Trace shows a single layer within the Lacing zone. The two loops interlock in an S-curve in the interior space beneath the perimeter. The crossover bonds the two loops together at the top. The perimeter ribbon continues uninterrupted through the anchor point on both sides, with the narrow cutout at ±0.75w.

##### Developed

**[Figure: Lacing — Developed — see `images/` in the project repository.]**

The Developed view shows the full Lacing zone across its layer span. Below the zone the CCW and CW Stringer tubes approach as separate helical columns. Within the zone the interlocked Lacing Traces stack into a double-coil form, bonding the two tubes together across every layer of contact. Above, the tubes resume their independent helices. The depth of the Lacing zone is determined by the helical geometry of the two parent Stringers and is not a fixed parameter.

---

#### Gusset (Stringer × Former)

##### Profile

*[Drawing: dimensioned 2D profile of the Gusset Hoop, showing the merged Former thickening and Stringer crossover in a single continuous extrusion. Dimensions in w and h.]*

##### Slice

*[Drawing: 3D view of a Gusset layer, showing the Gusset Hoop tracing the thickened Former profile with the Stringer crossover integrated at the Stringer's angular position.]*

##### Developed

*[Drawing: 3D view of the Gusset zone across the Former layer span, showing the Stringer passing through the Former band with integrated geometry throughout.]*

---

#### Splay (Stringer × Whip)

##### Profile

*[Drawing: dimensioned 2D profile of the Splay Terminal at the final approach layer, showing the merged Terminal loop and reduced Trace. Dimensions in w and h.]*

##### Slice

*[Drawing: 3D view of the Splay zone at the Terminal layer, showing the Splay Terminal at the Stringer's anchor point as the Stringer tube tapers to zero.]*

##### Developed

*[Drawing: 3D view of the Splay approach zone across its layer span, showing the progressive taper of the Stringer Trace down to the Splay Terminal at the Whip boundary.]*

---

#### Cuff (Former × Whip)

##### Profile

*[Drawing: dimensioned 2D profile of the Cuff Terminal, showing the truncated Former profile closing into the Terminal loop. Dimensions in w and h.]*

##### Slice

*[Drawing: 3D view of a Cuff layer, showing the Former wall truncated at the Whip boundary and closing into the Terminal column.]*

##### Developed

*[Drawing: 3D view of the Cuff zone across the Former layer span, showing the Former thickening ramping back down to one line width as it approaches the boundary edge, with the Terminal column at the opening.]*

---

## Parameters

| Parameter | Feature | Default | Units | Notes |
| --- | --- | --- | --- | --- |
| Line width (*w*) | All | — | mm | Sets all canonical w-unit dimensions |
| Layer height (*h*) | All | — | mm | |
| Stringer count | Stringer | 4 | — | Number of CCW helices; equal number of CW helices generated automatically |
| Helix angle (θ_h) | Stringer | 45 | ° | Angle between stringer tube and skin surface; constant across all z heights via running integral |
| Former ramp layers (*n*) | Former | — | layers | |
| Whip support stub extent (*n*) | Whip | — | layers | |

---

## Implementation Notes

*This section describes one concrete reference implementation of the algorithm, built as a modification of the CuraEngine. The setting names, data structures, and engine hooks below are specific to that implementation and are provided to demonstrate enablement. They are not requirements of the method, which applies to any layer-based FFF slicing engine offering equivalent per-layer slice polygons, a perimeter-generation stage, and spiralized continuous extrusion. Where a name below is engine-specific (e.g. `infill_pattern`, `wall_line_count`, `Polygons`), it stands for the corresponding facility in any such engine.*

### Slicer Setting
FeatherPrint is activated by setting `infill_pattern = featherprint` on a mesh with `wall_line_count ≥ 1`, `top_layers = 0`, and `bottom_layers = 0`. The helix angle is exposed as `featherprint_helix_angle` (float, degrees).

### Phase Pre-Pass
The running integral for helix phase is computed in a sequential pre-pass across all layers of the mesh before the parallel wall-generation step. The per-layer perimeter length is taken from the largest polygon in each layer's outline. Results are stored in `mesh.fp_helix_phase[]` and indexed by layer number.

### Collision Detection
For each layer, all CCW and CW anchor positions are sorted by arc position. Adjacent anchors of opposite helix direction that are within 2w of each other in arc distance are tested for world-space proximity. If the Euclidean distance between the two anchor points is less than w, both anchors are flagged and paired. During toolpath generation, the first anchor of each flagged pair emits a Lacing Trace centred at the midpoint between the two anchors; the second anchor is silently skipped.

### Conformal Placement Transform
Each canonical point is mapped to world space via the transform described in the Geometric Features section. The `radiusAt(centroid, θ)` method ray-casts from the centroid at angle θ and returns the intersection distance with the perimeter polygon. All feature arc primitives are tessellated at a fixed segment count and each sample is passed individually through the transform.

---

# Part II — Conformal Placement Transform Specification

*The following specifies the single shared placement method referenced throughout Part I. Drafted 28 June 2026.*

## Purpose

Every print feature and collision feature is defined once, in canonical form, as a 2D extrusion-path profile in the Anchor Point UCS (perimeter along x, inward along −y, dimensions in units of line width *w*). The **Conformal Placement Transform** is the single, shared method that maps a canonical profile onto the actual skin of a given layer at a given anchor point, producing layer-plane toolpath geometry that lies flush against the real perimeter.

This method is feature-agnostic. Lacing, Gusset, Splay, Cuff, and the base Stringer/Whip Traces all use it; they differ only in the canonical profile supplied as input. Defining it once here avoids re-deriving the placement math per feature and guarantees that all features conform to the skin identically.

The transform is purely in-plane (layer XY). The z component of every feature is left flat at the layer plane; the crossover layer lift is applied downstream by the spiralize post-process and is out of scope for this method.

## Inputs

| Input | Description |
| --- | --- |
| Canonical profile | The feature's extrusion path in the Anchor Point UCS, as a sequence of line and arc primitives. Coordinates in *w*. |
| Centroid | The layer centroid, **C**, taken from the low-pass-filtered centroid line at this layer (not the raw per-layer bounding-box centre). |
| Perimeter polyline | The layer's slice perimeter as an ordered list of integer points (in the reference implementation, a CuraEngine `Polygons` object; any engine's per-layer slice-polygon representation serves equally). The skin is represented as connected straight segments; no analytic curvature exists. |
| Anchor point | The point **A** on the perimeter at which the feature is placed, and its angular position θ_anchor measured about **C**. |
| Line width *w* | Resolves canonical units to mm at emit. |
| Chord tolerance ε | Maximum permitted deviation between a flattened arc and the true arc, in mm. |

## Coordinate Frames

The transform maps between two frames:

- **Canonical Cartesian** — the Anchor Point UCS. A canonical point is (x, y): x is the offset along the perimeter from the anchor, y is the inward depth toward the centroid. The anchor is the origin.
- **Layer Polar** — a polar frame centred on the layer centroid **C**. A point is (r, θ): r is the radial distance from **C**, θ is the angular position about **C**.

The skin perimeter radius is the function **R(θ)** — the distance from **C** to the perimeter along the ray at angle θ, found by intersecting that ray with the perimeter polyline. Because the skin is a polyline, R(θ) is piecewise-linear in θ, with slope discontinuities at every perimeter vertex.

**R(θ) is required to be single-valued.** A ray from the centroid must cross the perimeter exactly once. This holds when the slice is star-convex about the centroid. Slices that are reentrant about the centroid (a centroid ray crossing the perimeter more than once) are **disallowed**, and avoiding them is the responsibility of the user supplying the model. Behaviour on such slices is undefined.

## The Map

A canonical point (x, y) maps to layer-polar (r, θ) as follows.

### Angular component

> θ = θ_anchor + x / R_a

where R_a = R(θ_anchor) is the perimeter radius **evaluated once at the anchor**. This converts the canonical along-perimeter offset x into a swept angle, using the anchor-point arc-length scale. Evaluating R at the anchor (rather than integrating R(θ) along the perimeter) keeps θ linear in x. This is justified because every feature is orders of magnitude narrower than the skin radius; the arc-length error introduced across a feature's span is negligible.

This linear-angle map preserves the canonical ratio relationships: for two points, x₁ / x₂ = θ₁′ / θ₂′ where θ′ is the angular offset from the anchor.

### Radial component

> r = R(θ) − y

where R(θ) is evaluated **per point, at that point's own angle θ** (not at the anchor). This is the defining choice of the method: the baseline radius from which inward depth is subtracted tracks the real skin contour across the feature's angular span. Consequently the feature's perimeter edge (all points with y = 0) follows the actual perimeter polyline, remaining flush with the skin even where the skin curves or kinks within the feature's span. The feature bottom follows the skin; it does not sit on a circular approximation.

### Width behaviour

Because θ is linear in x while r decreases with inward depth y, lines of constant x converge as they descend inward — the feature narrows toward the centroid, subtending a fixed angle over diminishing arc length. The feature's physical width equals its canonical width **only at the skin radius**; inward of the skin it narrows. This convergence is geometrically correct for a feature conforming to a curved skin and is accepted. Its magnitude is negligible in practice, as feature depth is orders of magnitude smaller than R.

### Inverse to layer XY

The layer-plane toolpath point is recovered from (r, θ) about the centroid:

> P = C + r·(cos θ, sin θ)

## Arc Handling

Canonical profiles store arcs as primitives (centre, radius, endpoints), retained through storage and validation. **The Conformal Placement Transform does not preserve arcs.** Because R(θ) varies across a feature's span, the radial map is non-uniform, and a canonical circular arc does not map to any analytic arc in the layer plane — its image is a general curve.

Therefore arcs are **flattened during placement, not at emit**:

1. Each canonical arc is sampled into points in canonical Cartesian space.
2. Every point — line vertices and arc samples alike — is passed through the map above.
3. After deformation, the chord deviation of each flattened arc is checked against ε. Any arc segment exceeding ε is resampled at higher density and re-deformed.

The post-deformation tolerance check is required because the deformation **adds** curvature: the skin's curvature R(θ) compounds with the arc's own curvature. An arc adequately sampled against a flat reference may be under-sampled once laid on a tightly curved region of skin (e.g. a leading edge). Sampling density must be evaluated after deformation, or with a margin sized to the maximum skin curvature within the zone, to prevent visible faceting of loop interiors.

## Procedure

For a single feature placement:

1. Resolve the centroid **C** from the filtered centroid line at this layer.
2. Resolve the anchor **A** and its angular position θ_anchor about **C**. Compute R_a = R(θ_anchor).
3. For each primitive in the canonical profile:
   a. If an arc, sample to points at the canonical chord tolerance.
   b. For each point (x, y): compute θ = θ_anchor + x / R_a; compute R(θ) by single-crossing ray-cast against the perimeter polyline; compute r = R(θ) − y; recover P = C + r·(cos θ, sin θ).
4. Check post-deformation chord error of each flattened arc against ε; resample and repeat step 3 for any that exceed it.
5. Pass the resulting layer-plane polyline to emit, which resolves *w* to mm and writes extrusion moves. Z is left at the layer plane for the spiralize post-process.

## Notes and Boundary Conditions

- **Centroid source.** The filtered centroid line is used, per the Centroid definition. Using the raw per-layer bounding-box centre would introduce z-axis jitter into θ and R across layers.
- **R(θ) at vertices.** R(θ) has slope discontinuities at perimeter vertices. Under this method (per-point R) these are followed faithfully: a feature spanning a vertex creases to match the skin. This is intended. Severe kinking of features is avoided by the user not supplying geometry whose perimeter kinks sharply within a feature span; the method does not smooth or guard against it.
- **Disallowed geometry.** Reentrant slices (multivalued R(θ)) are the user's responsibility, as above. The method assumes single-crossing throughout.
- **Profile constancy.** For features whose canonical profile is constant across the zone (e.g. Lacing), the canonical sampling in step 3a need be performed once and reused; only the per-point R(θ) evaluation in 3b varies by layer, as the anchor and perimeter change.
