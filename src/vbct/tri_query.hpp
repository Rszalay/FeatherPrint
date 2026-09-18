// Small mesh-query helpers shared by Stage 5 and Stage 6, matching the
// Python reference where stage6.py directly imports stage5.py's
// leading-underscore "private" helpers (`_boundary_count`, `_dist`,
// `_edge_to_tris`, `_tri_edges`) rather than duplicating them. Header-only
// since each is a few lines.
#pragma once

#include <cmath>
#include <map>
#include <vector>

#include "geometry.hpp"
#include "stage5.hpp"  // for Triangle

namespace vbct {

// Unsorted edges opposite each vertex, in a fixed order: (v1,v2), (v2,v0),
// (v0,v1). Order matters to callers that pick "the first non-boundary
// edge encountered" (Stage 6 Phase 1) or store first-seen orientation
// (Stage 5 Correction 3) -- see tri_raw_edges' twin in stage5.cpp.
inline std::array<std::pair<int, int>, 3> tri_raw_edges(const Triangle& t) {
    return {{{t[1], t[2]}, {t[2], t[0]}, {t[0], t[1]}}};
}

inline int boundary_count(const Triangle& t, const std::set<Edge>& boundary_edges) {
    int c = 0;
    for (const auto& [a, b] : tri_raw_edges(t)) {
        if (boundary_edges.count(make_edge(a, b))) ++c;
    }
    return c;
}

inline std::map<Edge, std::vector<int>> edge_to_tris(const std::vector<Triangle>& triangles) {
    std::map<Edge, std::vector<int>> m;
    for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
        for (const auto& [a, b] : tri_raw_edges(triangles[i])) m[make_edge(a, b)].push_back(i);
    }
    return m;
}

inline double dist(const Point2& p, const Point2& q) { return std::hypot(p.x - q.x, p.y - q.y); }

}  // namespace vbct
