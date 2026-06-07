#pragma once

#include <optional>
#include <vector>

#include "geometry/OpenLinesSet.h"
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

private:
    // Compute the signed-area centroid of a polygon.
    static Point2LL computeCentroid(const Polygon& poly);

    // Sample n evenly-spaced points along poly by arc length.
    // phase_deg: angle (degrees) from centroid that stringer 0 aims toward.
    static std::vector<Point2LL> arcLengthSamplePoints(
        const Polygon& poly,
        int n,
        double phase_deg,
        const Point2LL& centroid);

    // First intersection of the ray (origin -> direction point, extended) with poly edges.
    // Returns nullopt if no forward intersection exists.
    static std::optional<Point2LL> rayIntersectPolygon(
        const Point2LL& origin,
        const Point2LL& direction_point,
        const Polygon& poly);

    const Shape& infill_area_;
    coord_t z_;               // layer Z in microns
    const Settings& settings_;
};

} // namespace cura
