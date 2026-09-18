// Contour: a closed polygon in fixed-point integer coordinates.
// Direct port of vbct/contour.py -- see geometry.hpp for the SCALE rationale.
#pragma once

#include <string>
#include <vector>

#include "geometry.hpp"

namespace vbct {

struct Contour {
    std::vector<Point2i> points;
    std::string contour_id;
    // Parallel to `points`. Empty means "not tracked" -- treat as all-false
    // (a contour that never went through VBS). See spec REV 2.1 S:5.1.
    std::vector<bool> vbs_inserted;

    bool is_vbs_point(size_t i) const {
        return vbs_inserted.empty() ? false : vbs_inserted[i];
    }

    // One (p0, p1) boundary segment per point, wrapping around the end.
    struct IEdge {
        Point2i a, b;
    };
    std::vector<IEdge> edges() const {
        std::vector<IEdge> result;
        size_t n = points.size();
        result.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            result.push_back({points[i], points[(i + 1) % n]});
        }
        return result;
    }

    std::vector<Point2> to_float() const {
        std::vector<Point2> result;
        result.reserve(points.size());
        for (const auto& p : points) {
            result.push_back({static_cast<double>(p.x) / SCALE, static_cast<double>(p.y) / SCALE});
        }
        return result;
    }
};

// Build a Contour from float coordinates, snapping to the integer grid.
// Rounding matches Python's round() (half-to-even), not C++'s std::round
// (half-away-from-zero), so results are bit-exact against the reference --
// see vbs.cpp's `python_round` for the same concern in VBS itself.
Contour from_float_points(const std::vector<Point2>& points, const std::string& contour_id = "");

// Drops consecutive near-duplicate points (any two points, wrapping
// around the end, within `epsilon_scaled` fixed-point units of each
// other), keeping the first of each pair. Guards against feeding CDT two
// points that are, for practical purposes, the same physical location
// but differ by only a few units of SCALE quantization -- concretely,
// Clipper's own corner mitering (e.g. offsetting a real model's cross
// section by wall thickness before VBCT ever sees it) can insert two
// points a hair's-width apart that land in adjacent SCALE cells. Left
// alone, the neighboring triangle straddling that near-zero-length edge
// is a degenerate sliver with no meaningful "which wall" identity, which
// Stage 5's Correction 2.5 (bridging-triangle re-routing) correctly
// refuses to touch (its convexity guard rejects folding into a sliver,
// since there's no real corner there to re-triangulate) -- confirmed via
// a real case, see VBCT-Wall-Start-Point-Instability-Brief.md's sibling
// investigation and stage5.cpp's Correction 2.5 notes. This is Stage-1-
// adjacent input hygiene, not a triangulation or correction change:
// it runs once, before VBS, on the raw contour VBCT is handed, so every
// later stage only ever sees the cleaned-up point set.
//
// Widened from the original 10 units (0.01mm) to 50 (0.05mm): a second
// real case put a hub's own short-stub tip only 40 units (0.04mm) from
// its neighbour, at the exact same y-coordinate -- past the original
// epsilon, so it survived into Stage 5 as a genuinely degenerate sliver.
// Stage 5's Correction 2.7 (short-stub absorption) correctly detected
// the arm and its length, but declined to fan it: the sliver's own two
// near-coincident points fold the fan's last triangle back on itself,
// flipping its winding relative to the rest of the fan. Collapsing the
// pair here, before Stage 5 ever sees them, removes the degenerate tip
// entirely rather than teaching every later stage to route around it.
// 50 units (0.05mm) covers this case with a 20-unit margin and stays
// well below any real FDM feature size (nozzle diameters bottom out
// around 0.1mm) -- half that ceiling, same reasoning as the original
// choice.
//
// Returns `c` unchanged if collapsing would drop it below 3 points --
// letting a genuinely too-small contour fail Stage 1's own "fewer than 3
// points" check with a clear error, rather than silently producing an
// even-more-degenerate shape here.
Contour collapse_near_duplicate_points(const Contour& c, int64_t epsilon_scaled = 50);

}  // namespace vbct
