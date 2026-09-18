#include "mesh.hpp"

#include <CDT.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>

namespace vbct {
namespace {

struct Pslg {
    std::vector<Point2> vertices;
    std::vector<Edge> segments;  // vertex-index pairs, unsorted (ordered along contour)
    std::vector<bool> vbs_inserted;
};

Pslg build_pslg(const std::vector<Contour>& contours) {
    Pslg pslg;
    for (const auto& c : contours) {
        int base = static_cast<int>(pslg.vertices.size());
        int n = static_cast<int>(c.points.size());
        for (const auto& p : c.points) {
            pslg.vertices.push_back({static_cast<double>(p.x) / SCALE, static_cast<double>(p.y) / SCALE});
        }
        for (int i = 0; i < n; ++i) {
            pslg.segments.emplace_back(base + i, base + (i + 1) % n);
        }
        if (c.vbs_inserted.empty()) {
            pslg.vbs_inserted.insert(pslg.vbs_inserted.end(), n, false);
        } else {
            pslg.vbs_inserted.insert(pslg.vbs_inserted.end(), c.vbs_inserted.begin(), c.vbs_inserted.end());
        }
    }
    return pslg;
}

// Matches triangulate.py's `_triangle_neighbors`: neighbors[t][e] is the
// triangle across the edge opposite vertex `e` of triangle t, i.e. edge e
// runs between the *other* two vertices. -1 = hull edge. Recomputed from
// scratch here rather than trusting CDT's own internal neighbor indexing
// convention, so this logic stays backend-agnostic (and auditable against
// the spec) the way it already is in the Python reference.
std::vector<std::array<int, 3>> triangle_neighbors(const std::vector<std::array<int, 3>>& triangles) {
    std::map<Edge, std::vector<int>> edge_to_tris;
    for (int t = 0; t < static_cast<int>(triangles.size()); ++t) {
        const auto& tri = triangles[t];
        int verts[3] = {tri[0], tri[1], tri[2]};
        int pairs[3][2] = {{verts[1], verts[2]}, {verts[2], verts[0]}, {verts[0], verts[1]}};
        for (auto& pr : pairs) {
            edge_to_tris[make_edge(pr[0], pr[1])].push_back(t);
        }
    }

    std::vector<std::array<int, 3>> neighbors(triangles.size(), {-1, -1, -1});
    for (int t = 0; t < static_cast<int>(triangles.size()); ++t) {
        const auto& tri = triangles[t];
        int verts[3] = {tri[0], tri[1], tri[2]};
        int pairs[3][2] = {{verts[1], verts[2]}, {verts[2], verts[0]}, {verts[0], verts[1]}};
        for (int e = 0; e < 3; ++e) {
            Edge key = make_edge(pairs[e][0], pairs[e][1]);
            for (int other : edge_to_tris[key]) {
                if (other != t) neighbors[t][e] = other;
            }
        }
    }
    return neighbors;
}

// Ray-casting even-odd rule, cast along +x. True (odd) if `point` is
// inside an odd number of constrained edges.
bool crossing_parity(const Point2& point, const std::vector<Point2>& vertices, const std::set<Edge>& constrained) {
    bool odd = false;
    for (const auto& [a, b] : constrained) {
        double ax = vertices[a].x, ay = vertices[a].y;
        double bx = vertices[b].x, by = vertices[b].y;
        if ((ay > point.y) != (by > point.y)) {
            double x_at_y = ax + (point.y - ay) * (bx - ax) / (by - ay);
            if (x_at_y > point.x) odd = !odd;
        }
    }
    return odd;
}

// Degeneracy test for seed selection below -- deliberately duplicated from stage5.cpp's own
// triangle_height (same formula: 2*area/longest_edge, same "longest < 1e-12 -> 0" degenerate
// guard) rather than shared, since mesh.cpp is a lower-level file with no dependency on stage5.cpp.
double triangle_height(const Point2& a, const Point2& b, const Point2& c) {
    double area2 = std::abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
    double longest = std::max({std::hypot(b.x - a.x, b.y - a.y), std::hypot(c.x - b.x, c.y - b.y), std::hypot(a.x - c.x, a.y - c.y)});
    if (longest < 1e-12) return 0.0;
    return area2 / longest;
}

// Stage 4: BFS parity flood-fill from a seed face, then resolve the seed's
// true parity by ray-casting -- see spec REV 2.1 S:7 for why this is needed
// instead of seeding from a known-exterior face (a convex outer contour
// leaves no exterior face in this stage's raw triangulation).
//
// The seed must be a well-formed (non-degenerate) triangle: the ray-cast
// below tests a single point (the seed's own centroid) against every
// constrained edge, and calibrates the odd/even interpretation for the
// *entire* mesh from that one result. Confirmed directly on real capture
// (FPTF-45 - Body.stl, layer 58, a ring/annulus): CDT's own triangles[0]
// was a fully degenerate sliver (area 4.7e-15 - three near-collinear
// points), whose centroid sits essentially on the contour boundary rather
// than robustly inside or outside it, making the one ray-cast this
// calibration depends on a floating-point coin-flip -- it came up wrong and
// inverted interior/exterior for 509 of the mesh's 510 triangles (the whole
// annulus marked exterior, the inner hole marked interior). Area alone
// isn't a sufficient guard (a long, thin sliver can have non-trivial area
// while its centroid still sits on its own boundary) -- triangle height is
// the right metric, and picking the tallest triangle in the whole mesh as
// seed keeps its centroid robustly clear of every one of its own edges.
std::vector<bool> classify_domains(const std::vector<Point2>& vertices, const std::vector<std::array<int, 3>>& triangles,
                                    const std::vector<std::array<int, 3>>& neighbors, const std::set<Edge>& constrained) {
    int n = static_cast<int>(triangles.size());
    std::vector<int> depth(n, -1);

    int seed_idx = 0;
    double best_height = -1.0;
    for (int t = 0; t < n; ++t) {
        const auto& tri = triangles[t];
        double h = triangle_height(vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]);
        if (h > best_height) {
            best_height = h;
            seed_idx = t;
        }
    }

    depth[seed_idx] = 0;
    std::deque<int> queue{seed_idx};
    while (!queue.empty()) {
        int t = queue.front();
        queue.pop_front();
        const auto& tri = triangles[t];
        int verts[3] = {tri[0], tri[1], tri[2]};
        int pairs[3][2] = {{verts[1], verts[2]}, {verts[2], verts[0]}, {verts[0], verts[1]}};
        for (int e = 0; e < 3; ++e) {
            int nb = neighbors[t][e];
            if (nb == -1 || depth[nb] != -1) continue;
            bool crossed = constrained.count(make_edge(pairs[e][0], pairs[e][1])) > 0;
            depth[nb] = depth[t] + (crossed ? 1 : 0);
            queue.push_back(nb);
        }
    }

    Point2 seed_centroid{0.0, 0.0};
    for (int v : triangles[seed_idx]) {
        seed_centroid.x += vertices[v].x;
        seed_centroid.y += vertices[v].y;
    }
    seed_centroid.x /= 3.0;
    seed_centroid.y /= 3.0;

    bool seed_true_parity = crossing_parity(seed_centroid, vertices, constrained);
    bool seed_relative_parity = (depth[seed_idx] % 2) != 0;  // always false (depth[seed_idx] == 0)

    bool flip = seed_true_parity != seed_relative_parity;
    std::vector<bool> interior(n);
    for (int t = 0; t < n; ++t) {
        bool odd_depth = (depth[t] % 2) != 0;
        interior[t] = flip ? !odd_depth : odd_depth;
    }
    return interior;
}

}  // namespace

Mesh triangulate(const std::vector<Contour>& contours) {
    Pslg pslg = build_pslg(contours);
    if (pslg.vertices.size() < 3) {
        throw TriangulationError("fewer than 3 total vertices across all contours");
    }

    std::vector<CDT::V2d<double>> verts;
    verts.reserve(pslg.vertices.size());
    for (const auto& p : pslg.vertices) verts.emplace_back(p.x, p.y);

    std::vector<CDT::Edge> edges;
    edges.reserve(pslg.segments.size());
    for (const auto& [a, b] : pslg.segments) edges.emplace_back(CDT::VertInd(a), CDT::VertInd(b));

    CDT::Triangulation<double> cdt;
    try {
        cdt.insertVertices(verts);
        cdt.insertEdges(edges);
        cdt.eraseSuperTriangle();
    } catch (const std::exception& e) {
        throw TriangulationError(std::string("CDT triangulation failed: ") + e.what());
    }

    if (cdt.triangles.empty()) {
        throw TriangulationError("CDT produced no faces (degenerate input?)");
    }
    if (cdt.vertices.size() != pslg.vertices.size()) {
        throw TriangulationError(
            "CDT inserted Steiner points: " + std::to_string(pslg.vertices.size()) + " in, " +
            std::to_string(cdt.vertices.size()) +
            " out -- input likely violates stage-1 preconditions (simple, non-crossing contours)");
    }

    Mesh mesh;
    mesh.vertices.reserve(cdt.vertices.size());
    for (const auto& v : cdt.vertices) mesh.vertices.push_back({v.x, v.y});

    mesh.triangles.reserve(cdt.triangles.size());
    for (const auto& t : cdt.triangles) {
        mesh.triangles.push_back({static_cast<int>(t.vertices[0]), static_cast<int>(t.vertices[1]),
                                   static_cast<int>(t.vertices[2])});
    }

    for (const auto& s : pslg.segments) mesh.constrained.insert(make_edge(s.first, s.second));

    mesh.neighbors = triangle_neighbors(mesh.triangles);
    mesh.interior = classify_domains(mesh.vertices, mesh.triangles, mesh.neighbors, mesh.constrained);
    mesh.is_vbs_point = pslg.vbs_inserted;

    return mesh;
}

}  // namespace vbct
