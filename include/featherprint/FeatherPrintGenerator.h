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
    VariableWidthLines generate(const Shape& outline, coord_t z, const Settings& settings, double helix_phase = 0.0);

    /*!
     * Generate the Flange wall stack for one closed-boundary-loop layer.
     * ramp_index: 0 = innermost (first) Flange layer, n-1 = topmost brim layer.
     * Returns multiple closed ExtrusionLines (one per Wall), innermost first, outermost last.
     */
    VariableWidthLines generateFlange(const Shape& outline, const Settings& settings, int ramp_index, double helix_phase = 0.0);

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

private:
    // ---- Arc-length parameterization of a closed or open polyline -----------

    struct ArcParam
    {
        std::vector<double> cum_len;
        double total{};
        const Polyline* poly{};  // base-class pointer; works for Polygon and OpenPolyline
        bool is_open{ false };   // true for OpenPolyline: no closing segment, pointAt clamps

        Point2LL pointAt(double s) const;
        Point2LL tangentAt(double s) const;
        double referenceArcPos(const Point2LL& centroid) const;
        // Distance from centroid to the perimeter along the ray at angle theta.
        // Implements R(theta) for the Conformal Placement Transform.
        double radiusAt(const Point2LL& centroid, double theta) const;
    };

    static ArcParam buildArcParam(const Polygon& poly);
    static ArcParam buildArcParamOpen(const OpenPolyline& poly);
    static Point2LL centroidBbox(const Polygon& poly);
    static const Polygon* largestPoly(const Shape& shape);

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

    // Maps a canonical (lx, ly) point in w-units to a world Point2LL via the
    // Conformal Placement Transform (spec: FeatherPrint_ConformalPlacement_Spec.md).
    // lx: along-perimeter offset from anchor (positive = CCW-forward).
    // ly: inward depth (negative = inward toward centroid, 0 = on perimeter).
    // x_sign: +1 for CCW helix, -1 for CW helix (mirrors the canonical profile).
    static Point2LL conformPlace(double lx, double ly, double x_sign,
                                 double theta_anchor, double R_a,
                                 const Point2LL& centroid, const ArcParam& arc,
                                 coord_t w);

    // Tessellates one canonical arc through conformPlace and appends to line.
    static void appendCanonicalArc(ExtrusionLine& line,
                                   double cx, double cy, double R,
                                   double a_start_deg, double a_end_deg,
                                   bool cw_arc, int segs, double x_sign,
                                   double theta_anchor, double R_a,
                                   const Point2LL& centroid, const ArcParam& arc,
                                   coord_t w, bool skip_first = false);

    // Ray from centroid at angle theta; returns arc-length of first polygon intersection, or -1.
    static double arcLengthAtAngle(const ArcParam& arc, const Point2LL& centroid, double theta);

    static void appendTrace(ExtrusionLine& line,
                            double theta_anchor, double R_a,
                            const Point2LL& centroid, const ArcParam& arc,
                            bool is_cw, coord_t w, bool skip_first = false);

    // Terminal loop at one open endpoint of an open polyline.
    // Canonical profile: same arc radii as Stringer Trace, but the departure and return
    // both land at the anchor (x=0). The loop extends 2w in the -x_sign direction from
    // the anchor, dipping 2.5w inward. Followed by a closing perimeter segment that
    // retraces the loop back to the anchor (the crossover bond).
    // s_anchor: arc-length position of the open endpoint.
    // x_sign  : +1 to extend forward (CCW, for the start endpoint), -1 to extend backward (CW, for the end endpoint).
    // splay_L > 0 inserts a horizontal segment of length L·w at the loop bottom,
    // widening the terminal to cover the deleted Stringer Trace footprint (Splay feature).
    static void appendTerminal(ExtrusionLine& line,
                               double s_anchor, double x_sign,
                               const ArcParam& arc, const Point2LL& centroid,
                               coord_t w,
                               bool reversed = false,
                               double splay_L = 0.0);

    // Lacing Trace canonical profile (w-units, anchor at midpoint between two colliding stringers):
    //   Departure : (-0.75, 0)   Return : (+0.75, 0)
    //   Left inner arc  : centre (-0.75, -0.5), R=0.5,  CW  90°→-90°   (inner eye, opens right)
    //   Left outer arc  : centre (-0.75,-1.75), R=0.75, CCW 90°→270°   (outer loop, swings left to x=-1.5)
    //   Right outer arc : centre (+0.75,-1.75), R=0.75, CCW -90°→90°   (outer loop, swings right to x=+1.5)
    //   Right inner arc : centre (+0.75, -0.5), R=0.5,  CW  -90°→90°  (inner eye, opens left)
    static void appendLacingTrace(ExtrusionLine& line,
                                  double theta_anchor, double R_a,
                                  const Point2LL& centroid, const ArcParam& arc,
                                  coord_t w);

    // Flare Rim (Stringer/Lacing × Flange) — flat-bottomed filleted channel anchored at
    // (0,0) on the true OML (the outer wall's own toolpath centreline), so the channel
    // bottom stays pinned at a fixed 2.5w below the true OML regardless of the Flange's
    // current wall-stack thickness:
    //   T       = toolpath-centreline-to-toolpath-centreline OML-to-IML distance (w-units)
    //   D = 2.5 - T (channel depth below the true OML), R = 0.75 * (D / 2.5)
    //   W       = 2.0 for a Stringer anchor, 3.0 for a Lacing anchor (flat span at y=-T)
    //   oml_shift = the gap (w-units) between oml_arc's own ly=0 reference (the raw slice
    //               polygon) and the true OML — 0 except at the first Flange ramp layer,
    //               where the outer wall is a half-width wall offset outward of the raw
    //               polygon. Every canonical y-value is placed at (y - oml_shift) in
    //               oml_arc's own frame so the profile still anchors to the true OML.
    // theta_anchor/R_a must be evaluated against the OML (oml_arc), not the innermost wall,
    // and oml_arc must also be passed as the `arc` used for the profile's radial reference.
    static void appendFlareRim(ExtrusionLine& line,
                               double theta_anchor, double R_a, double x_sign,
                               const Point2LL& centroid, const ArcParam& oml_arc,
                               double T, double oml_shift, double W, coord_t w, bool skip_first = false);

    struct Anchor
    {
        double s;
        bool   is_cw;
        bool   skip{ false };
        int    pair_idx{ -1 }; // index of paired opposite-direction anchor when skip=true
    };

    // Seam point written by generate(), read by seamPoint()
    Point2LL seam_pt_{};
};

} // namespace cura
