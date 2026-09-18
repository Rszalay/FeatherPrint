#include "stage6.hpp"

#include <array>
#include <cmath>

#include "tri_query.hpp"

namespace vbct {
namespace {

constexpr double kCloseEps = 1e-6;

Point2 midpoint(const Point2& p, const Point2& q) { return {(p.x + q.x) / 2.0, (p.y + q.y) / 2.0}; }

Point2 sub(const Point2& p, const Point2& q) { return {p.x - q.x, p.y - q.y}; }

double dot_normalized(const Point2& u, const Point2& v) {
    double lu = std::hypot(u.x, u.y);
    double lv = std::hypot(v.x, v.y);
    if (lu == 0.0 || lv == 0.0) return -1.0;  // degenerate candidate; never preferred
    return (u.x * v.x + u.y * v.y) / (lu * lv);
}

bool is_close(const Point2& p, const Point2& q, double eps = kCloseEps) { return dist(p, q) < eps; }

// Rule 3's own reference direction ("the direction the ridge is arriving
// from", spec S:9.2) was originally just the one sleeve segment immediately
// adjacent to the terminal's own opening -- correct on paper, but on real,
// densely-tessellated geometry that single segment's own direction can
// swing noticeably between otherwise near-identical layers whenever the
// local triangulation reacts to real curvature nearby (confirmed directly
// against a real production part: the rule-3 winner flipped between all
// three candidates across six consecutive 0.2mm layers, with margins up to
// 0.75 -- not a numerical near-tie, but the reference direction itself
// being unstable). This walks further back through the same chain of
// sleeve triangles -- still entirely within this one layer's own
// triangulation, no cross-layer state -- and returns the point reached, so
// the caller can use the displacement from `mid_bc` over that whole window
// as a steadier reference than any single segment along the way. Stops
// before `max_hops` the moment it reaches a real vertex, a triangle that
// isn't itself a sleeve, or an edge with no further neighbour -- it never
// walks past a genuine corner or junction to keep averaging.
Point2 reference_far_point(int nb, const Point2& mid_bc, const Stage6Result& result, const std::vector<int>& counts,
                            const std::map<Edge, std::vector<int>>& e2t, int max_hops) {
    const auto& [p0, p1] = result.segments.at(nb);
    const auto& [n0, n1] = result.endpoints.at(nb);
    Point2 far_point = is_close(p0, mid_bc) ? p1 : p0;
    NodeId far_node = is_close(p0, mid_bc) ? n1 : n0;
    int cur_tri = nb;

    for (int hop = 1; hop < max_hops; ++hop) {
        if (far_node.kind != NodeId::EdgeNode) break;  // hit a real vertex -- stop
        Edge key = make_edge(far_node.u, far_node.v);
        auto it = e2t.find(key);
        if (it == e2t.end()) break;
        int next_tri = -1;
        for (int t : it->second)
            if (t != cur_tri) next_tri = t;
        if (next_tri == -1 || counts[next_tri] != 1) break;  // no further sleeve to continue through
        auto next_it = result.segments.find(next_tri);
        if (next_it == result.segments.end()) break;  // shouldn't happen; guard anyway
        const auto& [q0, q1] = next_it->second;
        const auto& [m0, m1] = result.endpoints.at(next_tri);
        if (is_close(q0, far_point)) {
            far_point = q1;
            far_node = m1;
        } else if (is_close(q1, far_point)) {
            far_point = q0;
            far_node = m0;
        } else {
            break;  // continuity guarantee violated; guard anyway
        }
        cur_tri = next_tri;
    }
    return far_point;
}

}  // namespace

Stage6Result run_stage6(const Stage5Result& stage5, int max_reference_hops) {
    const std::vector<Point2>& verts = stage5.vertices;
    const std::vector<Triangle>& triangles = stage5.triangles;
    const std::set<Edge>& boundary_edges = stage5.boundary_edges;
    // Rule 1 (S:9.2) needs to know specifically whether a terminal's tip is
    // a genuine Correction-3 hub apex -- `hub_edges` is populated only
    // inside `correct_junction_hubs_single_point` (stage5.cpp), never by
    // any other correction, so it's the precise test rule 1's own doc
    // comment describes. `contour_edges` (any edge not literally in the
    // original triangulation) was tried here before and is too broad:
    // Corrections 1/2/2.5/2.6/2.7 all promote ordinary wall edges too, and
    // any terminal triangle that happens to land on two such edges was
    // being treated as a hub apex and force-vertexed, permanently locking
    // out rule 3's own directional scoring for it -- confirmed directly
    // against real production geometry with zero actual hubs on the layer
    // (`hub_edges` empty) where this fired anyway.
    std::set<Edge> hub_edge_set(stage5.hub_edges.begin(), stage5.hub_edges.end());

    int T = static_cast<int>(triangles.size());
    std::vector<int> counts(T);
    for (int i = 0; i < T; ++i) counts[i] = boundary_count(triangles[i], boundary_edges);

    Stage6Result result;

    // Phase 1: sleeves (boundary-count 1) -- fully self-determined.
    for (int i = 0; i < T; ++i) {
        if (counts[i] != 1) continue;
        std::vector<std::pair<int, int>> opening;
        for (const auto& e : tri_raw_edges(triangles[i])) {
            if (!boundary_edges.count(make_edge(e.first, e.second))) opening.push_back(e);
        }
        Edge key0 = make_edge(opening[0].first, opening[0].second);
        Edge key1 = make_edge(opening[1].first, opening[1].second);
        result.segments[i] = {midpoint(verts[key0.first], verts[key0.second]),
                               midpoint(verts[key1.first], verts[key1.second])};
        result.endpoints[i] = {NodeId::edge(key0.first, key0.second), NodeId::edge(key1.first, key1.second)};
    }

    auto e2t = edge_to_tris(triangles);

    // Phase 2: terminals (boundary-count 2) -- reference only sleeve
    // segments (always already known) or a neighbor's classification
    // (never its computed segment), so processing order doesn't matter.
    for (int i = 0; i < T; ++i) {
        if (counts[i] != 2) continue;
        const Triangle& tri = triangles[i];

        std::vector<std::pair<int, int>> b_edges;
        for (const auto& e : tri_raw_edges(tri)) {
            if (boundary_edges.count(make_edge(e.first, e.second))) b_edges.push_back(e);
        }
        const auto& e1 = b_edges[0];
        const auto& e2 = b_edges[1];

        int tip_a = -1;
        for (int v : {e1.first, e1.second}) {
            if (v == e2.first || v == e2.second) tip_a = v;
        }

        std::vector<int> others;
        for (int v : tri)
            if (v != tip_a) others.push_back(v);
        int b = others[0], c = others[1];

        Edge e1_key = make_edge(e1.first, e1.second);
        Edge e2_key = make_edge(e2.first, e2.second);
        bool force_vertex = hub_edge_set.count(e1_key) && hub_edge_set.count(e2_key);
        Edge bc_key = make_edge(b, c);
        Point2 mid_bc = midpoint(verts[b], verts[c]);

        std::vector<int> neighbors;
        auto it = e2t.find(bc_key);
        if (it != e2t.end())
            for (int nb : it->second)
                if (nb != i) neighbors.push_back(nb);
        if (neighbors.empty()) {
            result.skipped.push_back(i);
            continue;
        }
        int nb = neighbors[0];
        int nb_count = counts[nb];

        if (nb_count == 2) {
            result.segments[i] = {mid_bc, verts[tip_a]};
            result.endpoints[i] = {NodeId::edge(bc_key.first, bc_key.second), NodeId::vertex(tip_a)};
        } else if (nb_count == 1 && force_vertex) {
            result.segments[i] = {mid_bc, verts[tip_a]};
            result.endpoints[i] = {NodeId::edge(bc_key.first, bc_key.second), NodeId::vertex(tip_a)};
        } else if (nb_count == 1) {
            auto seg_it = result.segments.find(nb);
            if (seg_it == result.segments.end()) {
                result.skipped.push_back(i);  // shouldn't happen; sleeves are Phase 1
                continue;
            }
            const auto& [p0, p1] = seg_it->second;
            Point2 sleeve_dir;
            if (is_close(p0, mid_bc) || is_close(p1, mid_bc)) {
                Point2 far_point = reference_far_point(nb, mid_bc, result, counts, e2t, max_reference_hops);
                sleeve_dir = sub(mid_bc, far_point);
            } else {
                result.skipped.push_back(i);  // continuity guarantee violated; guard anyway
                continue;
            }

            // Degenerate-tip guard (2026-09-13, Aerofoil-2412-Sweep trailing-edge investigation):
            // b and c are meant to be genuinely distinct points (the wall's two sides converging
            // toward tip_a), so candidates[1]/[2] are meant to be meaningfully different directional
            // choices from candidates[0]. When an upstream near-coincident-point defect (traced to
            // Arachne's own wall-inset output, part.infill_area, pinching two sides of a rapidly-
            // narrowing feature together to within a few microns) makes b and c the same point in
            // all but name, candidates[1] and candidates[2] both collapse onto the same wrong,
            // halfway-to-the-tip point instead - and since all three candidates then point in
            // nearly the same direction from mid_bc, the dot-product comparison below degenerates
            // into a coin-flip decided by floating-point noise (confirmed directly: one side of a
            // real domain reached the genuine tip, the other stopped at the phantom halfway point,
            // from the exact same geometry). Below this tolerance, skip the unreliable comparison
            // entirely and always walk all the way to the real vertex - never worse than the
            // halfway point (which isn't a real feature, just an artifact of b==c), and correct
            // whenever the degeneracy is genuine.
            constexpr double kDegenerateBcEps = 0.01;  // mm; real b/c separation is never this small
            if (dist(verts[b], verts[c]) < kDegenerateBcEps) {
                result.segments[i] = {mid_bc, verts[tip_a]};
                result.endpoints[i] = {NodeId::edge(bc_key.first, bc_key.second), NodeId::vertex(tip_a)};
                continue;
            }

            // Acute-tip guard (2026-09-13, Aerofoil-2412-Sweep trailing-edge investigation,
            // layer 329 vs 330): the two boundary edges at tip_a converge at whatever angle the
            // real geometry has there; the dot-product comparison below is meant to pick whichever
            // candidate best continues the established sleeve direction, but a genuinely sharp
            // (acute) tip legitimately kinks away from that direction by design (a trailing edge
            // is not collinear with the wall just before it) -- confirmed directly on real capture
            // data that this is not a numerical near-tie or an upstream data defect: candidate
            // vertex[tip_a] scored cosine 0.9932 against the sleeve direction, candidate
            // midpoint(tip_a, c) scored 0.9997, a real (if narrow) geometric difference that flips
            // sign between adjacent, otherwise near-identical layers as the local triangulation
            // shifts slightly. For a sufficiently acute tip (< 60 degrees, comfortably inside any
            // real trailing/leading edge and well outside a genuine broad kink mid-wall), the
            // vertex is always the correct choice regardless of how the comparison below would
            // score it, so skip the comparison entirely rather than let it decide.
            constexpr double kAcuteTipCosThreshold = 0.5;  // cos(60 degrees)
            const double tip_angle_cos = dot_normalized(sub(verts[b], verts[tip_a]), sub(verts[c], verts[tip_a]));
            if (tip_angle_cos > kAcuteTipCosThreshold) {
                result.segments[i] = {mid_bc, verts[tip_a]};
                result.endpoints[i] = {NodeId::edge(bc_key.first, bc_key.second), NodeId::vertex(tip_a)};
                continue;
            }

            std::array<std::pair<Point2, NodeId>, 3> candidates = {{
                {verts[tip_a], NodeId::vertex(tip_a)},
                {midpoint(verts[tip_a], verts[b]), NodeId::edge(tip_a, b)},
                {midpoint(verts[tip_a], verts[c]), NodeId::edge(tip_a, c)},
            }};
            int best_idx = 0;
            double best_val = dot_normalized(sub(candidates[0].first, mid_bc), sleeve_dir);
            for (int k = 1; k < 3; ++k) {
                double val = dot_normalized(sub(candidates[k].first, mid_bc), sleeve_dir);
                if (val > best_val) {
                    best_val = val;
                    best_idx = k;
                }
            }
            result.segments[i] = {mid_bc, candidates[best_idx].first};
            result.endpoints[i] = {NodeId::edge(bc_key.first, bc_key.second), candidates[best_idx].second};
        } else {
            // nb_count in {0, 3}: not covered by the spec -- 0 (junction)
            // is confirmed not to occur; 3 is the known circle-only gap.
            result.skipped.push_back(i);
        }
    }

    for (int i = 0; i < T; ++i) {
        if (counts[i] != 1 && counts[i] != 2) {
            bool already = false;
            for (int s : result.skipped)
                if (s == i) already = true;
            if (!already) result.skipped.push_back(i);
        }
    }

    return result;
}

}  // namespace vbct
