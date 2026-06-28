// Copyright (c) 2026 Rszalay
// CuraEngine is released under the terms of the AGPLv3 or higher
#pragma once

#include <vector>

#include "geometry/Polygon.h"
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
     * Returns the world-space position of the seam (helix 0 departure) for this layer.
     * Valid only after generate() has been called for the same z / settings combination.
     * Used by WallsComputation to set the z-seam hint on the part.
     */
    Point2LL seamPoint() const { return seam_pt_; }

private:
    // ---- Arc-length parameterization of a closed polygon --------------------

    struct ArcParam
    {
        std::vector<double> cum_len;
        double total{};
        const Polygon* poly{};

        Point2LL pointAt(double s) const;
        Point2LL tangentAt(double s) const;
        double referenceArcPos(const Point2LL& centroid) const;
        // Distance from centroid to the perimeter along the ray at angle theta.
        // Implements R(theta) for the Conformal Placement Transform.
        double radiusAt(const Point2LL& centroid, double theta) const;
    };

    static ArcParam buildArcParam(const Polygon& poly);
    static Point2LL centroidBbox(const Polygon& poly);
    static const Polygon* largestPoly(const Shape& shape);

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

    static void appendTrace(ExtrusionLine& line,
                            double theta_anchor, double R_a,
                            const Point2LL& centroid, const ArcParam& arc,
                            bool is_cw, coord_t w, bool skip_first = false);

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
