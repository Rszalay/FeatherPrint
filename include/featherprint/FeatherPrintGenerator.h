// Copyright (c) 2026 Rszalay
// CuraEngine is released under the terms of the AGPLv3 or higher
#pragma once

#include <vector>

#include "geometry/OpenPolyline.h"
#include "geometry/Polygon.h"
#include "geometry/Polyline.h"
#include "geometry/Shape.h"
#include "settings/Settings.h"
#include "utils/Coord_t.h"
#include "utils/ExtrusionLine.h"

namespace cura
{

/*!
 * Generates the FeatherPrint conformal stringer toolpath for one layer.
 *
 * Returns a VariableWidthLines (one ExtrusionLine) containing the full continuous
 * perimeter walk with embedded Stringer Trace and Lacing Trace features for this layer.
 *
 * Integration: called from WallsComputation::generateWalls(SliceLayerPart*) when
 * infill_pattern == FEATHERPRINT, replacing the WallToolPaths call.
 *
 * Cura settings consumed:
 *   wall_line_count          >= 1  (must be non-zero for pipeline to run)
 *   featherprint_line_width  (mm)
 *   featherprint_stringer_count
 *   featherprint_helix_pitch (deg/mm)
 *   top_layers = 0, bottom_layers = 0  (suppress skin)
 */
class FeatherPrintGenerator
{
public:
    /*!
     * Generate the wall toolpath for one layer.
     * \param outline   The outer contour of the slice (from SliceLayerPart::outline).
     * \param z         Layer z in µm.
     * \param settings  Mesh settings.
     * \return          One VariableWidthLines bin (inset_idx=0) with a single closed
     *                  ExtrusionLine representing the full perimeter+stringer path.
     *                  Returns empty if the contour is too small to slice.
     */
    VariableWidthLines generate(
        const Shape& outline,
        coord_t z,
        const Settings& settings,
        double helix_phase = 0.0,
        Point2LL phase_origin = Point2LL(0, 0));

    /*!
     * Generate the Flange wall stack for one closed-boundary-loop layer.
     * ramp_index: 0 = innermost (first) Flange layer, n-1 = topmost brim layer.
     * Returns multiple closed ExtrusionLines (one per Wall), innermost first, outermost last.
     */
    VariableWidthLines generateFlange(
        const Shape& outline,
        const Settings& settings,
        int ramp_index,
        double helix_phase = 0.0,
        Point2LL phase_origin = Point2LL(0, 0));

    // Parameters shared across all open-polyline arcs on the same layer.
    // Computed once from the full virtual ring (all arcs + gap chords) so that
    // every arc uses a consistent perimeter length, reference angle, and centroid.
    struct OpenLayerParams
    {
        Point2LL centroid;
        double full_ring_total{};    // perimeter of the full virtual ring
        double full_ring_arc_ref{};  // arc-length position of the +X reference ray intersection
        double arc_start_in_ring{};  // start position of this specific arc within the full ring
    };

    /*!
     * Generate the wall toolpath for one open-manifold layer (open polyline input).
     * open_poly must already be oriented CCW in math coordinates (caller's responsibility).
     * params carries the full-ring geometry shared across all arcs on this layer.
     */
    VariableWidthLines generateOpen(const OpenPolyline& open_poly, coord_t z, const Settings& settings, double helix_phase, const OpenLayerParams& params);

    /*!
     * Generate the Flange wall stack for one Layer where the boundary loop is interrupted
     * by a boundary edge (a slot/hole reaching the open top) — the Miter case. open_poly is
     * the single open arc for this Layer (same input generateOpen takes). The outer Wall is
     * closed with an ordinary Whip Terminal at each end, continuing the same Terminal column
     * as ordinary Whip layers below the Flange. Inner Walls (including the innermost, which
     * also carries Flare Rim insertion) are left open at both ends per the Miter rule — no
     * distinct Miter geometry is generated; the outer Wall's Terminal is relied on to weld
     * the inner Walls' open ends as a side effect of print-order (inner Walls first, outer
     * Wall last — the caller must reverse the usual Flange print order for this Layer).
     * Does not yet insert Flare Rims (first pass: get the wall-stack + Miter terminal
     * behaviour generating correctly before layering in Stringer/Lacing integration).
     */
    VariableWidthLines generateFlangeOpen(const OpenPolyline& open_poly, coord_t z, const Settings& settings, int ramp_index, double helix_phase, const OpenLayerParams& params);

    /*!
     * Returns the world-space position of the seam (helix 0 departure) for this layer.
     * Valid only after generate() has been called for the same z / settings combination.
     * Used by WallsComputation to set the z-seam hint on the part.
     */
    Point2LL seamPoint() const { return seam_pt_; }

    /*!
     * Returns the inward offset (from the outline passed to generate()/generateFlange(), in
     * the same coord_t units as w) from the OML to the innermost printed Wall's own inner
     * face for this layer — the region a caller should treat as this layer's "inner area"
     * (used for top/bottom skin generation: WallsComputation offsets the layer outline inward
     * by this amount to populate SliceLayerPart::inner_area, which was previously left empty
     * unconditionally, silently disabling top/bottom skin on every FeatherPrint layer
     * regardless of the featherprint_top_layers/bottom_layers... i.e. the stock Cura
     * top_layers/bottom_layers settings). For an ordinary (non-Flange) Layer this is a single
     * Wall's own width (w); for a Flange Layer it is the full Wall-stack depth to the
     * innermost Wall's inner face. Valid only after generate()/generateFlange() has been
     * called for the same settings; only meaningful for closed-perimeter Layers — an
     * open-manifold (Whip) Layer represents a boundary edge the designer left open, and the
     * caller should leave inner_area empty there rather than calling this at all.
     */
    coord_t innerOffset() const { return inner_offset_; }

private:
    // ---- Arc-length parameterization of a closed or open polyline -----------

    struct ArcParam
    {
        std::vector<double> cum_len;
        double total{};
        const Polyline* poly{};  // base-class pointer; works for Polygon and OpenPolyline
        bool is_open{ false };   // true for OpenPolyline: no closing segment, pointAt clamps
        // True if poly's stored vertex order is mathematically CCW (positive signed area, for
        // a closed Polygon) — a global, topology-level invariant, computed once when this
        // ArcParam is built rather than tested per-point. Used by resolveFrame to orient
        // tangent/inward-normal WITHOUT any centroid-relative test: for a simple polygon with
        // consistent winding, "inward" is always "tangent rotated toward the interior side"
        // regardless of concavity — a centroid-relative test (the previous approach) can pick
        // the wrong side in a concave region or near a hole, where "toward the centroid" isn't
        // reliably "toward the material." For an open arc (is_open), ccw is always true: the
        // caller already guarantees open_poly is oriented CCW in math coordinates (see
        // generateOpen's own doc comment and WallsComputation's arc-orientation step) — there's
        // no independent signed area to compute for an open (non-closed) polyline.
        bool ccw{ true };

        Point2LL pointAt(double s) const;
        Point2LL tangentAt(double s) const;

        // Nearest point on this polyline to an arbitrary world-space target point (Spec REV
        // 2.2 Anchor Distribution's Phase Origin). No centroid, no ray, no angle — a plain
        // point-to-segment min-distance scan over every edge, same cost order as the
        // centroid-ray-cast this replaces (referenceArcPos, removed). Returns the arc-length
        // position of the projection.
        double nearestArcPos(const Point2LL& target) const;
    };

    static ArcParam buildArcParam(const Polygon& poly);
    static ArcParam buildArcParamOpen(const OpenPolyline& poly);
    static Point2LL centroidBbox(const Polygon& poly);
    static const Polygon* largestPoly(const Shape& shape);

    // ---- Skin-Normal Tangent-Blend Placement (Self-Reference/03-conformal-placement-transform.md
    //      "Skin-Normal Variant") -----------------------------------------------------------
    //
    // Replaces the earlier centroid-radial transform (r = R(theta) - y about the layer centroid).
    // Anchor resolution (theta_anchor, R_a) is unchanged; only per-point placement differs: each
    // canonical point is placed by affine-blending between two RIGID frames built from the real
    // perimeter's own local tangent/normal at the feature's Start and End extents, rather than by
    // projecting radially toward the centroid. Convention: canonical +Y = inward (toward centroid),
    // matching the spec's "y - direction from anchor toward centroid" and the WIP Parametric
    // Canonical Toolpaths doc's Point Tables (opposite of this file's prior ly<0=inward convention).

    // A rigid world-space frame anchored at a point on the real perimeter: S = position, (tx,ty) =
    // unit tangent (direction of increasing arc-length), (nx,ny) = unit inward normal. Both are
    // derived purely from the polyline's own global winding (ArcParam::ccw), not a centroid-
    // relative test — robust on concave/non-star-convex shapes, where "toward the centroid"
    // isn't reliably "toward the material's interior" (see resolveFrame's fuller comment).
    struct TangentFrame
    {
        Point2LL S;
        double tx{}, ty{};
        double nx{}, ny{};
    };

    // Resolves S(s), T(s), N(s) directly from the real polyline's own arc-length parametrization
    // (ArcParam::pointAt / tangentAt) — NOT from an angle ray-cast against the centroid. Width
    // (the lateral extent between a feature's two blend frames) must equal the caller's requested
    // physical distance exactly regardless of local skin curvature or angle relative to the
    // centroid; an angle-then-ray-cast resolution only preserves that distance where the perimeter
    // happens to be locally circular about the centroid, and distorts it (roughly by 1/cos of the
    // tangent's angle off the centroid-perpendicular) everywhere else. Arc-length is intrinsic to
    // the curve, so it has no such distortion. (The anchor's OWN position is still resolved by
    // angle/ray-cast elsewhere — that's a separate concern, distributing anchors around the part.)
    static TangentFrame resolveFrame(const ArcParam& arc, const Point2LL& centroid, double s);

    // Two rigid placements (Copy A at the feature's Start, Copy B at its End) of the entire
    // canonical point cloud, affine-blended by t = (lx - x_s) / (x_e - x_s). x_s/x_e are the
    // canonical x-coordinates (w-units) of the FIRST and LAST points emitted in traversal order
    // (which for a CW-mirrored feature are the table's End/Start, swapped) - not necessarily the
    // profile's x-extrema. x_sign folds the CW helix mirror into the tangent component only (the
    // t-blend itself is direction-agnostic).
    struct BlendPlacement
    {
        TangentFrame A, B;
        double x_s{}, x_e{};
        double x_sign{ 1.0 };

        Point2LL place(double lx, double ly, coord_t w) const;
    };

    // s_anchor is the anchor's own arc-length position — located by Anchor Distribution (Spec
    // REV 2.2: fixed-point projection + arc-length walk), not by angle/ray-cast. Copy A/B are
    // placed at s_anchor +/- x*w along the real perimeter's own arc-length, so the physical
    // distance between them is exactly (x_e - x_s) * w regardless of local curvature or the
    // anchor's position relative to the centroid. (theta_anchor/R_a, and the internal
    // arc-length-via-angle round-trip they required, are gone as of REV 2.2 — s_anchor is
    // already known directly by every caller.)
    static BlendPlacement buildBlendPlacement(double s_anchor, double x_sign,
                                              double x_s, double x_e,
                                              const Point2LL& centroid, const ArcParam& arc, coord_t w);

    // One Wall of a Flange's wall stack: offset = inward distance from the OML to the
    // Wall's own toolpath centreline (µm); width = the Wall's own extrusion width (µm).
    struct FlangeWallDesc
    {
        coord_t offset;
        coord_t width;
    };

    // Shared by generateFlange() and generateFlangeOpen(): the Flange build-up rule from
    // the spec (1.5w at ramp 0, +0.5w total thickness per ramp layer thereafter), returned
    // outer-first (index 0 = outer Wall, back() = innermost Wall).
    static std::vector<FlangeWallDesc> buildFlangeWallStack(int ramp_index, coord_t w);

    // Maps a Wall's position in the outer-first wall-stack array (wi: 0=outer..n_walls-1=
    // inner) to the inset_idx that controls its actual print order (ascending inset_idx =
    // later in print order, per InsetOrderOptimizer's inside_out convention — see
    // FffPolygonGenerator's forced optimize_wall_printing_order=false/inset_direction=
    // inside_out override for FeatherPrint meshes). The OML (wi=0) always gets inset_idx 0
    // (prints last, required for the Miter weld). The IML (wi=n_walls-1) gets inset_idx 1
    // (prints second-to-last, not first) — it's the Wall most likely to be printing over an
    // overhang and benefits from having the other inner/middle Walls already laid down for
    // adhesion. Any buried middle Walls fill inset_idx 2..n_walls-1 (printed earliest, order
    // among themselves doesn't matter). Degenerates to the plain outer-then-inner-last order
    // when there are 2 or fewer Walls (no middle Walls to reorder around).
    static int flangePrintInsetIdx(int wi, int n_walls);

    // Approximates a perpendicular polygon offset by moving each point radially toward/away
    // from the centroid by `offset` (inward positive) — consistent with the same
    // radial-from-centroid approximation the Conformal Placement Transform already uses
    // everywhere else, and the only practical option for offsetting an OPEN polyline (Clipper
    // offsetting is only defined for closed Shapes).
    static OpenPolyline radialOffsetOpen(const OpenPolyline& poly, const Point2LL& centroid, coord_t offset);

    static void appendPolySegment(ExtrusionLine& line, const ArcParam& arc,
                                  double s0, double s1, coord_t w, bool add_start);

    // Tessellates one canonical arc through the tangent-blend placement and appends to line.
    // lx: along-perimeter offset from anchor (positive = CCW-forward).
    // ly: inward depth (positive = inward toward centroid, 0 = on perimeter).
    static void appendCanonicalArc(ExtrusionLine& line,
                                   double cx, double cy, double R,
                                   double a_start_deg, double a_end_deg,
                                   bool cw_arc, int segs,
                                   const BlendPlacement& blend,
                                   coord_t w, bool skip_first = false);

    // Ray from centroid at angle theta; returns arc-length of first polygon intersection, or -1.
    static double arcLengthAtAngle(const ArcParam& arc, const Point2LL& centroid, double theta);

    // Thin-Section Pruning (Spec REV 2.4, proposed). True if a ray cast from the anchor
    // (s_anchor on arc) along its own local inward normal — the winding-based normal
    // resolveFrame already computes (REV 2.3), not a centroid-directed ray, for the same
    // reason Anchor Distribution moved off centroid — hits arc's own real perimeter within
    // distance D*w. If so, the local material is thinner than this feature's Depth and its
    // geometry should be suppressed at this anchor for this Layer (the anchor position itself
    // is unaffected — callers must still use it for anchor continuity, only skip drawing).
    // A small arc-length window around the anchor is excluded from the hit test so the ray
    // doesn't trivially register a hit against its own immediate neighborhood. Both the
    // detection radius (== D, no margin) and the exclusion window width are explicit,
    // tentative starting points per the spec, not settled values — flagged for revisiting
    // once tested against real parts.
    static bool isThinSection(const ArcParam& arc, const Point2LL& centroid, double s_anchor, double D, coord_t w);

    // Stringer Trace — Spec REV 2.0 (corrected). G=0.5 (fixed default, not user-exposed),
    // R2 = featherprint_stringer_width/2, R1 = R2+G, D = featherprint_stringer_depth.
    // Point Table: Anchor(0,0) Start(-G,0) P1(R2,R1) P2(R2,D-R2) P3(-R2,D-R2) P4(-R2,R1) End(G,0).
    // P1/P4 sit on the opposite side from Start/End, so Arc1/Arc3 genuinely cross the
    // centreline near the top — a real self-intersecting crossover, per REV 2.0.
    // Path: Arc1 Start->P1 R1 CCW c(-G,R1); Line1 P1->P2; Arc2 P2->P3 R2 CCW c(0,D-R2);
    //       Line2 P3->P4; Arc3 P4->End R1 CCW c(G,R1).
    // CW helix: same centres/radii, traversed in reverse order with each arc's direction flipped
    // and x_sign=-1 (spec: CW Helix Mirroring).
    static void appendTrace(ExtrusionLine& line,
                            double s_anchor,
                            const Point2LL& centroid, const ArcParam& arc,
                            bool is_cw, coord_t w,
                            double D, double W, bool skip_first = false);

    // Whip Terminal — Spec REV 2.0. R1,R2 match Stringer's (R1=R2+G, R2=featherprint_stringer_width/2).
    // D = featherprint_stringer_depth ("matching Stringer's canonical depth", same as R1/R2).
    // Re-verified by hand against the corrected Stringer R1=R2+G formula (spec's own note had
    // flagged this table stale from before that fix): Arc2 and Arc3 share one centre/radius
    // (C2==C3 since R2=W/2 makes P3==P4), so together they form a single smooth 180° arc over
    // the top regardless of which of R1/R2 is larger — no self-intersection or tangent
    // discontinuity is introduced by R1 now exceeding R2. Validity mirrors Stringer's own
    // D >= W+G bound (D >= R1+R2, else Line1 inverts).
    // Point Table: Start(R1,0) P1(0,R1) P2(0,D-R2) P3(R2,D) P4(W-R2,D) P5(W,D-R2) End(W,Lw/2)
    // (End.y = Lw/2 = 0.5 in w-units — confirmed intentional per REV 2.0: "the path ends at
    // End, near the surface... on the far side", not exactly on the perimeter).
    // Path: Arc1 Start->P1 R1 CW c(R1,R1); Line1 P1->P2; Arc2 P2->P3 R2 CW c(R2,D-R2);
    //       Line2 P3->P4 (splay_L inserted here); Arc3 P4->P5 R2 CW c(W-R2,D-R2); Line3 P5->End.
    // s_anchor: arc-length position of the open endpoint.
    // x_sign  : +1 to extend forward (CCW, for the start endpoint), -1 to extend backward (CW, for the end endpoint).
    // splay_L > 0 widens the flat-bottom Line2 span by L*w, widening the terminal to cover the
    // deleted Stringer Trace footprint (Splay feature).
    static void appendTerminal(ExtrusionLine& line,
                               double s_anchor, double x_sign,
                               const ArcParam& arc, const Point2LL& centroid,
                               coord_t w,
                               double D, double R1, double R2, double W,
                               bool reversed = false,
                               double splay_L = 0.0);

    // Lacing Trace — per Spec REV 2.0's centers-first derivation, anchor at the midpoint between
    // two colliding stringers. R1 = Lw/2, R2 = (D - Lw)/2, D = featherprint_lacing_depth,
    // W = featherprint_lacing_width, G = 0.5 (fixed).
    // Centres: C1(G+R1,R1) C2(W/2-R2,D-R2) C3(-C2.x,C2.y) C4(-C1.x,C1.y).
    // Point Table: Start(C1.x,0) P1(C1.x,2R1) P2(C2.x,2R1) P3(C2.x,D) P4(0,D)
    //              P5(C3.x,D) P6(C3.x,2R1) P7(C4.x,2R1) End(C4.x,0).
    // Path (spec order): Arc1 Start->P1 R1 CCW C1; Line1 P1->P2; Arc2 P2->P3 R2 CW C2;
    //       Line2 P3->P4; Line3 P4->P5; Arc3 P5->P6 R2 CW C3;
    //       Line4 P6->P7; Arc4 P7->End R1 CCW C4.
    // Implementation walks the REVERSE of the spec's own table (End->P7->...->Start, each arc
    // direction flipped) because the spec's literal Start sits ahead of the anchor and End
    // behind — backwards relative to the caller's stitching direction. Symmetric profile:
    // x_sign = +1 unconditionally (no CW helix mirror needed).
    static void appendLacingTrace(ExtrusionLine& line,
                                  double s_anchor,
                                  const Point2LL& centroid, const ArcParam& arc,
                                  coord_t w, double D, double W);

    // Flare Rim (Stringer/Lacing × Flange) — per Parametric Canonical Toolpaths (WIP).md: a
    // single-fillet 3-segment channel (Arc1 -> Line1 -> Arc2) anchored at (0,0) on the true OML
    // (the outer wall's own toolpath centreline):
    //   Q = total thickness (w-units) of the current Flange ramp layer's Wall stack — the sum
    //       of each Wall's own width, not a plain Wall count, since some ramp layers include a
    //       half-width (buried or outer) Wall ("Wall Number" per the WIP doc, corrected: a
    //       count would under-charge a stack containing a half-width Wall, making the channel
    //       shallower than the Wall stack it's welding into actually is)
    //   D = featherprint_flare_depth setting (w-units); R1 = min(D - Q, W/2) (w-units) —
    //       clamped to W/2 so Line1's span (W - 2*R1) can't go negative and cross the two end
    //       arcs over each other when D is large relative to W
    //   W = 2.0 for a Stringer anchor, 3.0 for a Lacing anchor (flat span at y=R1)
    //   oml_shift = the gap (w-units) between oml_arc's own ly=0 reference (the raw slice
    //               polygon) and the true OML — 0 except at the first Flange ramp layer,
    //               where the outer wall is a half-width wall offset outward of the raw
    //               polygon. Every canonical y-value is placed at (y - oml_shift) in
    //               oml_arc's own frame so the profile still anchors to the true OML.
    // theta_anchor/R_a must be evaluated against the OML (oml_arc), not the innermost wall,
    // and oml_arc must also be passed as the `arc` used for the profile's tangent-blend frames.
    static void appendFlareRim(ExtrusionLine& line,
                               double theta_anchor, double R_a, double x_sign,
                               const Point2LL& centroid, const ArcParam& oml_arc,
                               double Q, double D, double oml_shift, double W, coord_t w, bool skip_first = false);

    struct Anchor
    {
        double s;
        bool   is_cw;
        bool   skip{ false };
        int    pair_idx{ -1 }; // index of paired opposite-direction anchor when skip=true
    };

    // Seam point written by generate(), read by seamPoint()
    Point2LL seam_pt_{};

    // Inner-area offset written by generate()/generateFlange(), read by innerOffset()
    coord_t inner_offset_{ 0 };
};

} // namespace cura
