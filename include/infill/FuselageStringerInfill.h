#pragma once

#include <optional>
#include <vector>

#include "geometry/OpenLinesSet.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Point2LL.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"
#include "settings/Settings.h"

namespace cura
{

// Generates radial stringer lines for the fuselage geodesic infill pattern.
// Called from Infill::_generate() when fuselage_enable is true.
//
// Phase 1 (radial geometry): N spokes from outer perimeter to inner perimeter
// (or centroid on single-polygon layers). Arc-length sampling ensures even
// spoke spacing regardless of cross-section shape.
class FuselageStringerInfill
{
public:
    FuselageStringerInfill(const Shape& infill_area, coord_t z, const Settings& settings);

    void generate(OpenLinesSet& result_lines);

    // A point on a polygon boundary together with which edge it lies on.
    struct SamplePt
    {
        Point2LL pt;
        size_t   edge_idx; // index i such that pt lies on edge poly[i]→poly[(i+1)%n]
    };

private:
    static Point2LL computeCentroid(const Polygon& poly);

    // Sample n evenly-spaced points along poly by arc length.
    // phase_deg: angle (degrees) from centroid that stringer 0 aims toward.
    static std::vector<SamplePt> arcLengthSamplePoints(
        const Polygon& poly,
        int n,
        double phase_deg,
        const Point2LL& centroid);

    // Walk the polygon boundary from from to to via the shorter arc, appending intermediate
    // vertices and to.pt.  Always takes the short path (< n/2 steps), so it works correctly
    // for both CCW outer polygons (forward walk) and CW inner polygons (backward walk).
    // nudge_toward/nudge_dist: every appended point is nudged toward nudge_toward by
    // nudge_dist microns.  Positive = toward (inward for trough arcs); negative = away
    // from nudge_toward (outward for peak arcs).
    static void appendArc(OpenPolyline& path, const Polygon& poly,
                          const SamplePt& from, const SamplePt& to,
                          const Point2LL& nudge_toward, coord_t nudge_dist);

    // First intersection of the ray (origin → direction_point, extended) with poly edges.
    // Returns the hit point and the edge index it lands on.
    static std::optional<SamplePt> rayIntersectPolygon(
        const Point2LL& origin,
        const Point2LL& direction_point,
        const Polygon& poly);

    const Shape& infill_area_;
    coord_t z_;
    const Settings& settings_;
};

} // namespace cura
