// Stage 3 (constrained Delaunay triangulation) and Stage 4 (domain
// classification). Port of vbct/triangulate.py -- see spec REV 2.1 S:6-7.
//
// Backend: CDT (artem-ogre/CDT), not CGAL -- see the plan's packaging note.
// CDT's default IntersectingConstraintEdges::NotAllowed gives the same
// "reject crossing constraints" behavior as CGAL's
// No_constraint_intersection_tag; `eraseSuperTriangle()` (not
// eraseOuterTriangles[AndHoles]) is what keeps the whole convex-hull
// region filled -- including holes and hull-to-contour exterior -- for
// this stage's own classification to work on, matching what the
// `triangle` package's `"pc"` flags produced for the Python reference.
#pragma once

#include <array>
#include <set>
#include <stdexcept>
#include <vector>

#include "contour.hpp"
#include "geometry.hpp"

namespace vbct {

class TriangulationError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct Mesh {
    std::vector<Point2> vertices;                  // (V,) float, original units
    std::vector<std::array<int, 3>> triangles;      // (T,) vertex indices
    std::vector<bool> interior;                     // (T,) true = kept by Stage 4
    std::vector<std::array<int, 3>> neighbors;      // (T,) -1 = hull edge
    std::vector<bool> is_vbs_point;                 // (V,)
    std::set<Edge> constrained;                     // sorted-pair boundary edges
};

Mesh triangulate(const std::vector<Contour>& contours);

}  // namespace vbct
