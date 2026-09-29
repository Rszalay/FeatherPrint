// Medial-axis domain decomposition (FeatherPrint Corrugated extension, corrugated_decomposition = voronoi).
//
// An alternative to VBCT Stages 1-9 (VBS subdivision, CDT, Stage 5 corrections, skeleton, chains, walls, hubs) that
// produces the same Stage9Result: the exact medial axis of the outline from a segment Voronoi diagram
// (boost::polygon), pruned by one geometric rule, walked into Ring and Chain domains.
//
//   - Interior primary Voronoi edges only.
//   - Corner branches: an edge is kept only when its two generating boundary features' nearest points are far
//     apart along the outline relative to the local width (boundary distance / radius > kCornerRatio). A 90 degree
//     or blunter corner's bisector scores <= 2 and is dropped; a corridor's axis scores ~4+ even at a square end;
//     a sharp tip (an airfoil trailing edge) keeps its axis.
//   - Spurs: a leaf branch shorter than the prune threshold, hanging off a junction, is removed (Stage 9's rule).
//   - A component that is a pure cycle is a Ring (walls: the two whole contours). Everything else is split into
//     Chains between junctions and dead ends. Walls come from the boundary features either side of the axis; at a
//     dead end each wall runs on along the outline to its corner (a turn > 45 degrees) and the cap sits at the
//     midpoint of what is left, so a square end, a rounded end and a pointed end all get one well-defined ending.
#pragma once

#include <string>
#include <vector>

#include "contour.hpp"
#include "stage9.hpp"

namespace vbct {

struct VoronoiDebugEdge {
    std::vector<Point2> points;  // mm, discretized (curved edges sampled)
    int status{ 0 };             // 0 kept, 1 corner-pruned, 2 spur-pruned
    double ratio{ 0.0 };         // boundary distance / radius at the edge's deeper end
};

struct VoronoiDebug {
    std::vector<VoronoiDebugEdge> edges;
};

// Contours in VBCT's own fixed-point units (Point2i, SCALE per mm). Throws std::runtime_error when the diagram can't
// be built. `debug`, when given, receives every interior edge with its pruning status.
Stage9Result decompose_voronoi(const std::vector<Contour>& contours, double threshold_mm, VoronoiDebug* debug = nullptr);

}  // namespace vbct
