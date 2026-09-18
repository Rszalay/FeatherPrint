#include "stage5.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <optional>
#include <unordered_map>

#include "tri_query.hpp"

namespace vbct {
namespace {

// ---- small geometry helpers (module-private, matches stage5.py) -----------

double signed_area2(const Point2& p, const Point2& q, const Point2& r) {
    return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
}

int sign_eps(double x, double eps = 1e-9) {
    if (x > eps) return 1;
    if (x < -eps) return -1;
    return 0;
}

bool segments_properly_cross(const Point2& p1, const Point2& p2, const Point2& p3, const Point2& p4) {
    int o1 = sign_eps(signed_area2(p1, p2, p3));
    int o2 = sign_eps(signed_area2(p1, p2, p4));
    int o3 = sign_eps(signed_area2(p3, p4, p1));
    int o4 = sign_eps(signed_area2(p3, p4, p2));
    return o1 != o2 && o3 != o4 && o1 != 0 && o2 != 0 && o3 != 0 && o4 != 0;
}

// ---- Correction 0: sliver-triangle edge flip -------------------------------
//
// A near-collinear vertex (not a near-duplicate one -- edge lengths stay
// normal, only the triangle's own height collapses) produces a triangle with
// near-zero area sitting between two otherwise-ordinary triangles. Its own
// three edges are real interior diagonals (never boundary), so the *next*
// triangle over ends up with all three of its own edges non-boundary too --
// misclassified as a junction (S:8.3's bc==0 test) purely because of which
// diagonal the upstream triangulation happened to choose, not because of any
// real branch point. Confirmed directly against real production geometry
// (VBCT-corrugation-defect-investigation, "4-domain split" thread):
// tools/diagnose_prefan_junctions.cpp found exactly this pattern at both
// spurious hub clusters causing a real print's Chain domain to fracture into
// 4 pieces -- each hub's own triangles bordered a sliver ~1000x smaller than
// its real neighbours (area 0.001-0.004 vs 0.6-1.2), each sliver's own vertex
// sitting only ~0.01-0.05mm off the line between its other two.
//
// Stage 1's own near-duplicate-point collapse (contour.hpp) can't catch this
// -- it's a point-to-point distance test, and this is a point-to-*line*
// degeneracy. Fixed here instead, at the source, via a standard Delaunay
// edge flip: replace the sliver and its neighbour across the sliver's own
// degenerate (non-boundary) edge with the alternate diagonal of their
// combined quad, whenever that alternate diagonal produces two well-formed
// triangles. Run first, before every other correction and before the
// sleeve/terminal/junction tally, since a diagonal choice this bad taints
// every downstream classification that looks at this triangle.
struct SliverFlipResult {
    std::vector<Triangle> triangles;
    int flip_count = 0;
};

double triangle_height(const Point2& a, const Point2& b, const Point2& c) {
    double area2 = std::abs(signed_area2(a, b, c));
    double longest = std::max({dist(a, b), dist(b, c), dist(c, a)});
    if (longest < 1e-12) return 0.0;
    return area2 / longest;  // == 2*area/longest_edge
}

SliverFlipResult flip_sliver_triangles(const std::vector<Point2>& vertices, std::vector<Triangle> triangles,
                                        const std::set<Edge>& boundary_edges, double height_epsilon) {
    int flip_count = 0;
    bool changed = true;
    while (changed) {
        changed = false;
        auto e2t = edge_to_tris(triangles);
        for (int si = 0; si < static_cast<int>(triangles.size()) && !changed; ++si) {
            const Triangle& S = triangles[si];
            if (triangle_height(vertices[S[0]], vertices[S[1]], vertices[S[2]]) >= height_epsilon) continue;

            for (const auto& e : tri_raw_edges(S)) {
                Edge key = make_edge(e.first, e.second);
                if (boundary_edges.count(key)) continue;  // never flip a real wall edge
                auto it = e2t.find(key);
                if (it == e2t.end() || it->second.size() != 2) continue;
                int ni = (it->second[0] == si) ? it->second[1] : it->second[0];
                const Triangle& N = triangles[ni];

                int u = key.first, v = key.second;
                int a = -1, b = -1;
                for (int vtx : S) if (vtx != u && vtx != v) a = vtx;
                for (int vtx : N) if (vtx != u && vtx != v) b = vtx;
                if (a < 0 || b < 0) continue;

                // Quad (u,a,v,b) must be simple and convex-order for the
                // alternate diagonal (a,b) to be a valid replacement -- same
                // check Correction 2.7's own fan-validity test uses.
                if (!segments_properly_cross(vertices[a], vertices[b], vertices[u], vertices[v])) continue;

                Triangle t1{a, u, b}, t2{a, b, v};
                double h1 = triangle_height(vertices[t1[0]], vertices[t1[1]], vertices[t1[2]]);
                double h2 = triangle_height(vertices[t2[0]], vertices[t2[1]], vertices[t2[2]]);
                if (h1 < height_epsilon || h2 < height_epsilon) continue;  // would just create a new sliver

                int ref_sign = sign_eps(signed_area2(vertices[S[0]], vertices[S[1]], vertices[S[2]]));
                if (sign_eps(signed_area2(vertices[t1[0]], vertices[t1[1]], vertices[t1[2]])) != ref_sign) std::swap(t1[1], t1[2]);
                if (sign_eps(signed_area2(vertices[t2[0]], vertices[t2[1]], vertices[t2[2]])) != ref_sign) std::swap(t2[1], t2[2]);

                triangles[si] = t1;
                triangles[ni] = t2;
                ++flip_count;
                changed = true;
                break;
            }
        }
    }
    return {std::move(triangles), flip_count};
}

// ---- Correction 0c: near-duplicate dead-end tip-vertex collapse -----------
//
// A plain terminal triangle T=(P,Q,R) (boundary_count==2, edges (P,R) and
// (Q,R) on the boundary, (P,Q) internal) whose apex R sits within
// `collapse_eps` of P or Q is not a real feature -- it's CDT's own
// tie-break at a contour deviation too small for Cura's own polygon
// simplification to have caught (confirmed on real data, FPTF-45 (3)
// z=51.95: two 0.09mm steps on an otherwise-straight wall, symmetric about
// the tube, each producing exactly this shape). Left alone, whichever
// triangle sits across (P,Q) can never become an ordinary sleeve --
// Correction 2.7's own stub-absorption fallback finds the resulting short
// stub but rejects its replacement fan, because R sitting almost exactly on
// top of P or Q inverts one of the fan's triangles -- and Correction 3's
// junction fan then promotes it to a spurious hub (VBCT-Layer520-
// Fragmentation-Report-260914.md).
//
// Fix: drop R, remove (P,R)/(Q,R) from boundary_edges, and promote (P,Q) in
// their place. R's only interior-mesh triangle is T itself, by construction
// (boundary_count(T)==2 means both (P,R) and (Q,R) already have exactly one
// incident triangle -- T), so no other triangle references R and no vertex
// renumbering is needed anywhere else in the mesh. T's own sliver-thin area
// is discarded -- the same class of deliberate, bounded deletion Correction
// 0b already makes, for the same reason: below print resolution. Run right
// after Correction 0, before anything that classifies junctions, so the
// triangle across (P,Q) is never seen as anything but an ordinary sleeve.
//
// Scoped to only fire when promoting (P,Q) actually eliminates a junction --
// i.e. the triangle on the other side of (P,Q) currently has
// boundary_count==0 there, so promoting (P,Q) turns it into an ordinary
// sleeve (boundary_count==1). Tried unscoped first and confirmed via the
// naca2412 reference fixture that this is necessary, not just conservative:
// an airfoil's own sharp trailing-edge cusp legitimately brings two VBS
// points within `collapse_eps` of each other where the triangle across
// (P,Q) is *already* a correctly-classified sleeve -- promoting (P,Q) there
// doesn't remove a junction, it turns a real sleeve into a terminal
// (sleeve_count regressed 172->170 against the Python reference). Requiring
// the far triangle to actually be a junction beforehand targets exactly the
// spurious-hub shape this correction exists for and leaves a legitimate
// sharp corner untouched.
struct TipCollapseResult {
    std::vector<Triangle> triangles;
    std::set<Edge> boundary_edges;
    int collapse_count = 0;
};

TipCollapseResult collapse_tip_vertices(const std::vector<Point2>& vertices, std::vector<Triangle> triangles,
                                         std::set<Edge> boundary_edges, double collapse_eps) {
    int collapse_count = 0;
    bool changed = true;
    while (changed) {
        changed = false;
        auto e2t = edge_to_tris(triangles);
        for (int ti = 0; ti < static_cast<int>(triangles.size()); ++ti) {
            const Triangle& T = triangles[ti];
            if (boundary_count(T, boundary_edges) != 2) continue;

            int P = -1, Q = -1, R = -1;
            for (const auto& [a, b] : tri_raw_edges(T)) {
                if (!boundary_edges.count(make_edge(a, b))) {
                    P = a;
                    Q = b;
                    break;
                }
            }
            if (P < 0) continue;  // defensive; shouldn't happen for boundary_count==2
            for (int vtx : T)
                if (vtx != P && vtx != Q) R = vtx;

            if (dist(vertices[R], vertices[P]) > collapse_eps && dist(vertices[R], vertices[Q]) > collapse_eps) continue;

            auto it = e2t.find(make_edge(P, Q));
            if (it == e2t.end() || it->second.size() != 2) continue;  // (P,Q) not shared by exactly one other triangle
            int ni = (it->second[0] == ti) ? it->second[1] : it->second[0];
            if (boundary_count(triangles[ni], boundary_edges) != 0) continue;  // promoting (P,Q) wouldn't clear a junction

            triangles.erase(triangles.begin() + ti);
            boundary_edges.erase(make_edge(P, R));
            boundary_edges.erase(make_edge(Q, R));
            boundary_edges.insert(make_edge(P, Q));
            ++collapse_count;
            changed = true;
            break;  // indices stale after erase; restart the scan
        }
    }
    return {std::move(triangles), std::move(boundary_edges), collapse_count};
}

// ---- Correction 0b: isolated sliver removal --------------------------------
//
// Correction 0 above fixes a sliver by edge-flipping it against a neighbour
// across its own degenerate (non-boundary) edge. Some slivers have no such
// edge at all -- every one of their edges is either a literal boundary edge
// or otherwise shared with no other triangle currently in the mesh, so
// there's nothing to flip against; the triangle is a fully isolated,
// single-triangle island. Confirmed directly on real capture
// (Aerofoil-2412-Sweep layer 91, z=9.15mm): a thin double-walled airfoil
// cross-section near the sweep's root, where one skin's own thin strip
// contains triangles like (101.127,107.135)/(98.193,107.16)/(104.061,107.11)
// -- three near-collinear points forming a zero-area "ear"
// (height 0.0000-0.0007mm vs 0.6-1.2mm2 for real neighbours), with
// boundary_count==2 (NOT 3 -- boundary_count reflects the original-contour/
// promoted-edge bookkeeping, which doesn't by itself imply live mesh
// adjacency) and its one remaining edge shared with no other triangle.
// Stage 6's own terminal handling (boundary_count==2) requires that
// remaining edge to have a neighbor to continue the chain through
// (`neighbors.empty()` -> skip, stage6.cpp); with none, it's skipped
// entirely. Because it sits in the middle of what should be one continuous
// sleeve chain, skipping it breaks the chain in two, leaving neither half
// complete -- Stage 9 then only produces "glob" domains and Stage 10 has no
// wall pair to build stringers from (confirmed: 5 real layers with zero
// infill, 1 heavily deformed, all tracing to this exact pattern).
//
// A near-zero-height triangle contributes negligible area, so removing it is
// a safe, geometry-preserving cleanup (not a shape approximation) regardless
// of *why* boundary_edges ended up calling all three of its edges boundary.
// Fix: drop the triangle and connect its two real, meaningfully-separated
// vertices (the endpoints of its own longest edge) with a direct boundary
// edge, discarding the near-collinear third ("apex") vertex that sat between
// them. Mirrors Correction 1's own carefulness about a point's
// triangle-membership count: only act when the apex has no other incident
// triangle, so a point that (unexpectedly) turns out to be used elsewhere is
// left alone rather than guessed at.
struct SliverRemovalResult {
    std::vector<Triangle> triangles;
    std::set<Edge> boundary_edges;
    int removal_count = 0;
};

SliverRemovalResult remove_isolated_slivers(const std::vector<Point2>& vertices, std::vector<Triangle> triangles,
                                             std::set<Edge> boundary_edges, double height_epsilon) {
    int removal_count = 0;
    bool changed = true;
    while (changed) {
        changed = false;
        auto e2t = edge_to_tris(triangles);
        for (int ti = 0; ti < static_cast<int>(triangles.size()); ++ti) {
            const Triangle& T = triangles[ti];
            if (triangle_height(vertices[T[0]], vertices[T[1]], vertices[T[2]]) >= height_epsilon) continue;

            // Fully isolated: none of T's own 3 edges are shared with any other
            // triangle currently in the mesh (regardless of boundary_count,
            // which reflects the ORIGINAL-contour/promoted-edge bookkeeping,
            // not live mesh adjacency -- a fully isolated triangle can still
            // read boundary_count==2 if only 2 of its edges happen to be
            // literal boundary edges; what actually matters here is that its
            // remaining edge has no interior neighbor to connect to).
            bool isolated = true;
            for (const auto& [a, b] : tri_raw_edges(T)) {
                auto it = e2t.find(make_edge(a, b));
                if (it != e2t.end() && it->second.size() > 1) { isolated = false; break; }
            }
            if (!isolated) continue;

            // Identify the apex: the vertex not part of T's own longest edge.
            std::array<std::pair<int, int>, 3> edges = tri_raw_edges(T);
            int longest_idx = 0;
            double longest_len = -1.0;
            for (int k = 0; k < 3; ++k) {
                double len = dist(vertices[edges[k].first], vertices[edges[k].second]);
                if (len > longest_len) {
                    longest_len = len;
                    longest_idx = k;
                }
            }
            int u = edges[longest_idx].first, v = edges[longest_idx].second;
            int p = -1;
            for (int vtx : T)
                if (vtx != u && vtx != v) p = vtx;

            // Safety gate: only remove if the apex has no other incident triangle.
            int p_incidence = 0;
            for (int tj = 0; tj < static_cast<int>(triangles.size()); ++tj) {
                if (tj == ti) continue;
                for (int vtx : triangles[tj])
                    if (vtx == p) { ++p_incidence; break; }
            }
            if (p_incidence != 0) continue;

            triangles.erase(triangles.begin() + ti);
            boundary_edges.erase(make_edge(u, p));
            boundary_edges.erase(make_edge(p, v));
            boundary_edges.insert(make_edge(u, v));
            ++removal_count;
            changed = true;
            break;
        }
    }
    return {std::move(triangles), std::move(boundary_edges), removal_count};
}

// ---- Correction 1: straight-vertex edge merge ------------------------------

struct Correction1Result {
    std::vector<Triangle> triangles;
    std::set<Edge> boundary_edges;
    int merge_count;
};

// Insertion-order vertex->incident-triangle map, matching Python dict
// insertion-order semantics for `_vertex_to_tris`: keys appear in the
// order they are first seen while scanning triangles 0..n-1 (each
// triangle's vertices in tuple order). Correction 1's fixed-point loop
// processes the first eligible vertex in this order and restarts, so
// order fidelity (not just eligibility-set fidelity) is preserved here on
// purpose, even though it isn't proven necessary for the final result.
std::vector<std::pair<int, std::vector<int>>> vertex_to_tris_ordered(const std::vector<Triangle>& triangles) {
    std::unordered_map<int, size_t> index_of;
    std::vector<std::pair<int, std::vector<int>>> result;
    for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
        for (int v : triangles[i]) {
            auto it = index_of.find(v);
            if (it == index_of.end()) {
                index_of[v] = result.size();
                result.push_back({v, {}});
                it = index_of.find(v);
            }
            result[it->second].second.push_back(i);
        }
    }
    return result;
}

Correction1Result correct_straight_vertices(std::vector<Triangle> triangles, const std::vector<bool>& is_vbs_point,
                                             std::set<Edge> boundary_edges) {
    int merge_count = 0;

    while (true) {
        auto vtx_to_tris = vertex_to_tris_ordered(triangles);
        bool changed = false;

        for (auto& [p, tri_idxs] : vtx_to_tris) {
            if (!is_vbs_point[p] || tri_idxs.size() != 2) continue;
            int t1i = tri_idxs[0], t2i = tri_idxs[1];
            std::set<int> t1(triangles[t1i].begin(), triangles[t1i].end());
            std::set<int> t2(triangles[t2i].begin(), triangles[t2i].end());
            std::vector<int> shared_vec;
            std::set_intersection(t1.begin(), t1.end(), t2.begin(), t2.end(), std::back_inserter(shared_vec));
            std::set<int> shared(shared_vec.begin(), shared_vec.end());
            if (!shared.count(p) || shared.size() != 2) continue;

            int q = -1;
            for (int v : shared)
                if (v != p) q = v;
            int a = -1;
            for (int v : t1)
                if (!shared.count(v)) a = v;
            int b = -1;
            for (int v : t2)
                if (!shared.count(v)) b = v;

            Edge pa = make_edge(p, a), pb = make_edge(p, b);
            if (!boundary_edges.count(pa) || !boundary_edges.count(pb)) continue;

            int hi = std::max(t1i, t2i), lo = std::min(t1i, t2i);
            triangles.erase(triangles.begin() + hi);
            triangles.erase(triangles.begin() + lo);
            triangles.push_back({a, q, b});

            boundary_edges.erase(pa);
            boundary_edges.erase(pb);
            boundary_edges.insert(make_edge(a, b));
            ++merge_count;
            changed = true;
            break;
        }

        if (!changed) return {triangles, boundary_edges, merge_count};
    }
}

// ---- Correction 2: false junction elimination ------------------------------

struct Correction2Result {
    std::vector<Triangle> triangles;
    std::set<Edge> boundary_edges;
    int excision_count;
};

Correction2Result correct_false_junctions(const std::vector<Point2>& vertices, std::vector<Triangle> triangles,
                                           const std::vector<bool>& is_vbs_point, std::set<Edge> boundary_edges) {
    auto e2t = edge_to_tris(triangles);
    std::set<int> used;
    int excision_count = 0;
    std::set<int> to_remove;
    std::vector<Triangle> to_add;

    for (int ji = 0; ji < static_cast<int>(triangles.size()); ++ji) {
        if (used.count(ji) || boundary_count(triangles[ji], boundary_edges) != 0) continue;
        const Triangle& tri = triangles[ji];

        for (int vi = 0; vi < 3; ++vi) {
            int v2 = tri[vi];
            if (!is_vbs_point[v2]) continue;

            std::vector<int> others;
            for (int v : tri)
                if (v != v2) others.push_back(v);
            Edge e1 = make_edge(v2, others[0]);
            Edge e2 = make_edge(v2, others[1]);

            std::vector<int> n1, n2;
            auto it1 = e2t.find(e1);
            if (it1 != e2t.end())
                for (int t : it1->second)
                    if (t != ji && !used.count(t)) n1.push_back(t);
            auto it2 = e2t.find(e2);
            if (it2 != e2t.end())
                for (int t : it2->second)
                    if (t != ji && !used.count(t)) n2.push_back(t);
            if (n1.size() != 1 || n2.size() != 1) continue;

            int stub1_i = n1[0], stub2_i = n2[0];
            const Triangle& stub1 = triangles[stub1_i];
            const Triangle& stub2 = triangles[stub2_i];
            if (boundary_count(stub1, boundary_edges) != 2 || boundary_count(stub2, boundary_edges) != 2) continue;

            int v1 = others[0];
            int v3 = others[1];
            int a = -1;
            for (int v : stub1)
                if (v != v2 && v != v1) a = v;
            int b = -1;
            for (int v : stub2)
                if (v != v2 && v != v3) b = v;

            std::array<Point2, 4> quad = {vertices[a], vertices[v1], vertices[v3], vertices[b]};
            if (segments_properly_cross(quad[0], quad[1], quad[2], quad[3]) ||
                segments_properly_cross(quad[1], quad[2], quad[3], quad[0])) {
                continue;
            }

            double qsum = 0;
            for (int k = 0; k < 4; ++k) qsum += signed_area2(quad[k], quad[(k + 1) % 4], quad[(k + 2) % 4]);
            int qsign = sign_eps(qsum / 4.0);
            if (qsign == 0) continue;

            struct Candidate {
                double d;
                Triangle t1, t2;
            };
            std::vector<Candidate> candidates = {
                {dist(vertices[a], vertices[v3]), {a, v1, v3}, {a, v3, b}},
                {dist(vertices[v1], vertices[b]), {v1, v3, b}, {v1, b, a}},
            };
            std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate& x, const Candidate& y) {
                return x.d < y.d;
            });

            bool found = false;
            Triangle chosen1{}, chosen2{};
            for (const auto& c : candidates) {
                int s1 = sign_eps(signed_area2(vertices[c.t1[0]], vertices[c.t1[1]], vertices[c.t1[2]]));
                int s2 = sign_eps(signed_area2(vertices[c.t2[0]], vertices[c.t2[1]], vertices[c.t2[2]]));
                if (s1 == qsign && s2 == qsign) {
                    chosen1 = c.t1;
                    chosen2 = c.t2;
                    found = true;
                    break;
                }
            }
            if (!found) continue;

            to_remove.insert(ji);
            to_remove.insert(stub1_i);
            to_remove.insert(stub2_i);
            used.insert(ji);
            used.insert(stub1_i);
            used.insert(stub2_i);
            to_add.push_back(chosen1);
            to_add.push_back(chosen2);
            boundary_edges.erase(make_edge(v2, a));
            boundary_edges.erase(make_edge(v2, b));
            boundary_edges.insert(make_edge(a, b));
            ++excision_count;
            break;
        }
    }

    std::vector<Triangle> kept;
    for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
        if (!to_remove.count(i)) kept.push_back(triangles[i]);
    }
    kept.insert(kept.end(), to_add.begin(), to_add.end());
    return {kept, boundary_edges, excision_count};
}

// ---- Correction 2b: junction/terminal edge flip -----------------------------
//
// A boundary_count==0 junction triangle adjacent, across any ONE of its 3
// edges, to a boundary_count==2 terminal triangle can always be resolved by
// flipping their shared edge to the quad's other diagonal: a terminal
// triangle's own "tip" vertex has its two OTHER edges as the real boundary
// (the two wall edges converging at that tip, by the boundary_count==2
// contract), so redistributing one each into the two new triangles after
// the flip turns both the junction AND the terminal into ordinary sleeves
// in one move -- no vertex removed, no is_vbs_point tag required at all,
// unlike Correction 2's own (deliberately narrower) excision, which only
// fires when a candidate VBS vertex has terminal neighbors across BOTH of
// its own two edges.
//
// Confirmed necessary on real production geometry (FPTF-45 Body.stl,
// layer z=53.75): a junction triangle with exactly one terminal neighbor
// (its other two edges backing onto ordinary sleeves) falls straight
// through Correction 2's own symmetric gate for every possible choice of
// candidate vertex, and survives all the way to Correction 3's fan -- not
// a pipeline-ordering problem (Correction 2 already runs early, right
// before this one), just a shape Correction 2 was never designed to reach.
// Runs right after Correction 2 rather than folded into it, so Correction
// 2's own narrower, vertex-tag-gated contract stays exactly as-is.
//
// A second gate, beyond the geometric ones above: `a` (J's own surviving
// vertex) must not be part of any OTHER boundary_count==0 triangle besides
// J itself. Confirmed necessary against a real regression on the
// synthetic "irregular" case-library fixture: the terminal apex's own
// local angle turned out NOT to discriminate a spurious CDT cap from a
// genuine branch arm -- real, confirmed-safe production flips span angles
// from 21 to 141 degrees, so no fixed angle/collinearity threshold
// separates them. What *does* separate them: on every real production
// flip, `a` touches no junction triangle besides J (the whole 2-triangle
// pair is fully self-contained -- resolving it leaves hub_sizes empty at
// that spot). On the "irregular" fixture's own failing case, `a` was ALSO
// part of a second, separate junction triangle -- i.e. `a` is a genuine
// hub vertex with more than one branch face, and flipping away just one of
// its terminal neighbors silently severed a real arm (confirmed: Stage 9
// domain count dropped from 4 to 3, one real chain vanished, one other
// chain's own cap point ended up at a fabricated midpoint that isn't even
// a mesh vertex). Requiring J to be `a`'s *only* junction triangle keeps
// this correction scoped to fully local, self-contained anomalies -- the
// same discipline debridge (Correction 2.5) and Correction 1c's own
// chain-length guard already follow -- and leaves any vertex that's part
// of a larger real branch structure for Correction 2.6/3 to handle
// untouched.
struct JunctionTerminalFlipResult {
    std::vector<Triangle> triangles;
    int flip_count = 0;
};

JunctionTerminalFlipResult flip_junction_terminal_pairs(const std::vector<Point2>& vertices,
                                                          std::vector<Triangle> triangles,
                                                          const std::set<Edge>& boundary_edges) {
    int flip_count = 0;
    bool changed = true;
    while (changed) {
        changed = false;
        auto e2t = edge_to_tris(triangles);
        for (int ji = 0; ji < static_cast<int>(triangles.size()) && !changed; ++ji) {
            if (boundary_count(triangles[ji], boundary_edges) != 0) continue;
            const Triangle& J = triangles[ji];

            for (const auto& e : tri_raw_edges(J)) {
                Edge key = make_edge(e.first, e.second);
                if (boundary_edges.count(key)) continue;  // never flip a real wall edge
                auto it = e2t.find(key);
                if (it == e2t.end() || it->second.size() != 2) continue;
                int ti = (it->second[0] == ji) ? it->second[1] : it->second[0];
                const Triangle& T = triangles[ti];
                if (boundary_count(T, boundary_edges) != 2) continue;  // only a terminal neighbor qualifies

                int u = key.first, v = key.second;
                int a = -1, b = -1;
                for (int vtx : J) if (vtx != u && vtx != v) a = vtx;
                for (int vtx : T) if (vtx != u && vtx != v) b = vtx;
                if (a < 0 || b < 0) continue;

                // `a` must not belong to any OTHER junction triangle -- see
                // this correction's own rationale above.
                bool a_has_other_junction = false;
                for (int tk = 0; tk < static_cast<int>(triangles.size()); ++tk) {
                    if (tk == ji) continue;
                    if (boundary_count(triangles[tk], boundary_edges) != 0) continue;
                    const Triangle& other = triangles[tk];
                    if (other[0] == a || other[1] == a || other[2] == a) {
                        a_has_other_junction = true;
                        break;
                    }
                }
                if (a_has_other_junction) continue;

                // Quad (u,a,v,b) must be simple and convex-order for the
                // alternate diagonal (a,b) to be a valid replacement -- same
                // check Correction 0's own sliver flip uses.
                if (!segments_properly_cross(vertices[a], vertices[b], vertices[u], vertices[v])) continue;

                Triangle t1{a, u, b}, t2{a, b, v};
                if (triangle_height(vertices[t1[0]], vertices[t1[1]], vertices[t1[2]]) < 0.05 ||
                    triangle_height(vertices[t2[0]], vertices[t2[1]], vertices[t2[2]]) < 0.05) {
                    continue;  // would just create a new sliver
                }

                int ref_sign = sign_eps(signed_area2(vertices[J[0]], vertices[J[1]], vertices[J[2]]));
                if (sign_eps(signed_area2(vertices[t1[0]], vertices[t1[1]], vertices[t1[2]])) != ref_sign) std::swap(t1[1], t1[2]);
                if (sign_eps(signed_area2(vertices[t2[0]], vertices[t2[1]], vertices[t2[2]])) != ref_sign) std::swap(t2[1], t2[2]);

                // Defensive: confirm the flip actually clears the junction
                // rather than just relocating it -- by construction (the
                // terminal's own 2 boundary edges land one each in t1/t2)
                // this always holds when the checks above pass, but this
                // file's universal practice is to verify, not assume.
                if (boundary_count(t1, boundary_edges) == 0 || boundary_count(t2, boundary_edges) == 0) continue;

                triangles[ji] = t1;
                triangles[ti] = t2;
                ++flip_count;
                changed = true;
                break;
            }
        }
    }
    return {std::move(triangles), flip_count};
}

// ---- Correction 2.5: bridging-triangle re-routing --------------------------
//
// A "bridging" junction triangle has 0 boundary edges (so Correction 3 would
// fan it into a spurious 3-spoke hub) but its 3 vertices actually split 2-1
// across two different original contour walls -- e.g. one vertex on the
// outer wall, two nearby vertices on the inner wall -- rather than sitting
// together in genuinely open interior space. CDT can legitimately choose
// this "diagonal bridge" over the expected ladder rung wherever one wall's
// local point spacing is tight relative to the other's; confirmed via
// jitter testing (stable under perturbation up to 1e-3, unlike a genuine
// Delaunay tie) that this is a deterministic geometric feature of dense/
// uneven wall point ratios, not a tie. This is the concrete case behind the
// open item in spec REV 2.1 S:16.3 ("Correction 3 has no notion of 'this
// cluster is not real branching topology'").
//
// Fix: if the bridging triangle's lone-wall vertex `o` and its neighbor's
// (across the far edge shared by the two same-wall vertices) opposite
// vertex `p` are adjacent along `o`'s own contour -- i.e. edge (o,p) is a
// genuine boundary edge -- flipping the diagonal from (a,b) to (o,p)
// converts both triangles into proper ladder-rung sleeves (each gains
// exactly one boundary edge) instead of one 0-boundary-edge junction.
// Purely a local re-triangulation choice within the CDT-legal alternative;
// no algorithm change to Correction 3 itself.

// Union-find over vertices connected by a boundary edge -- one component
// per original contour wall.
struct WallUnionFind {
    std::unordered_map<int, int> parent;
    int find(int x) {
        auto it = parent.find(x);
        if (it == parent.end()) {
            parent[x] = x;
            return x;
        }
        if (it->second != x) it->second = find(it->second);
        return parent[x];
    }
    void unite(int a, int b) {
        int ra = find(a), rb = find(b);
        if (ra != rb) parent[ra] = rb;
    }
};

struct DebridgeResult {
    std::vector<Triangle> triangles;
    int debridge_count;
};

DebridgeResult debridge_junction_triangles(const std::vector<Point2>& vertices, std::vector<Triangle> triangles,
                                            const std::set<Edge>& boundary_edges) {
    WallUnionFind wall;
    for (const auto& e : boundary_edges) wall.unite(e.first, e.second);

    int debridge_count = 0;
    bool changed = true;
    while (changed) {
        changed = false;
        auto e2t = edge_to_tris(triangles);

        for (int ti = 0; ti < static_cast<int>(triangles.size()); ++ti) {
            if (boundary_count(triangles[ti], boundary_edges) != 0) continue;
            const Triangle& tri = triangles[ti];

            // Find the lone-wall vertex o vs. the two same-wall vertices a,b.
            int wids[3] = {wall.find(tri[0]), wall.find(tri[1]), wall.find(tri[2])};
            int o_slot = -1;
            if (wids[0] == wids[1] && wids[1] != wids[2]) o_slot = 2;
            else if (wids[1] == wids[2] && wids[2] != wids[0]) o_slot = 0;
            else if (wids[0] == wids[2] && wids[2] != wids[1]) o_slot = 1;
            if (o_slot < 0) continue;  // not a clean 2-1 wall split

            int o = tri[o_slot];
            int a = tri[(o_slot + 1) % 3];
            int b = tri[(o_slot + 2) % 3];

            Edge ab = make_edge(a, b);
            auto it = e2t.find(ab);
            if (it == e2t.end() || it->second.size() != 2) continue;
            int ni = (it->second[0] == ti) ? it->second[1] : it->second[0];
            const Triangle& nb = triangles[ni];
            int p = -1;
            for (int v : nb)
                if (v != a && v != b) p = v;
            if (p < 0) continue;

            // p is a wall point that (a,b) skipped over -- it belongs on
            // a/b's wall (the far wall from o), not o's own wall. Flipping
            // the diagonal from (a,b) to (o,p) folds p into o's fan.
            //
            // A single skipped point resolves in one flip: both (a,p) and
            // (p,b) are then genuine boundary edges, so both replacement
            // triangles become proper sleeves immediately. A *multi*-point
            // skip (density mismatch spanning more than one far-wall
            // point) only has one of the two true at this step -- the far
            // side still bridges over the remaining skipped points. Flip
            // anyway when at least one side resolves: the still-bridging
            // replacement triangle is a strictly smaller bridge (one fewer
            // skipped point) and gets re-picked up by the outer while loop
            // next pass, unzipping the skip one point at a time until it
            // fully resolves. Requiring *both* sides here (as the original
            // version of this fix did) silently missed every skip wider
            // than one point, leaving hubs -- and their necklace-merge
            // chain leftovers -- behind.
            if (wall.find(p) != wall.find(a)) continue;
            bool a_side = boundary_edges.count(make_edge(a, p)) != 0;
            bool b_side = boundary_edges.count(make_edge(p, b)) != 0;
            if (!a_side && !b_side) continue;

            // A diagonal flip is only geometrically valid if quad (a,p,b,o)
            // is convex, i.e. its two diagonals (a,b) and (o,p) actually
            // cross -- otherwise this flip would produce a self-intersecting
            // pair of triangles. Wider skips make this less automatic than
            // the single-skip case, so check explicitly rather than assume.
            if (!segments_properly_cross(vertices[a], vertices[b], vertices[o], vertices[p])) continue;

            // Preserve the mesh's winding convention: `tri` in (o,a,b)
            // order is just a rotation of the original triangle, so its
            // sign is the reference. Reorder each replacement to match it,
            // rather than assume a fixed vertex order -- CDT's own winding
            // isn't otherwise guaranteed consistent across triangles here.
            int ref_sign = sign_eps(signed_area2(vertices[o], vertices[a], vertices[b]));
            Triangle t1 = {o, a, p};
            if (sign_eps(signed_area2(vertices[t1[0]], vertices[t1[1]], vertices[t1[2]])) != ref_sign) {
                std::swap(t1[1], t1[2]);
            }
            Triangle t2 = {o, p, b};
            if (sign_eps(signed_area2(vertices[t2[0]], vertices[t2[1]], vertices[t2[2]])) != ref_sign) {
                std::swap(t2[1], t2[2]);
            }
            triangles[ti] = t1;
            triangles[ni] = t2;
            ++debridge_count;
            changed = true;
            break;  // e2t is stale after a flip; rebuild before continuing
        }
    }

    return {triangles, debridge_count};
}

// Total length of an edge set, e.g. the original contour's own perimeter
// (mesh.constrained, summed once before any correction runs) -- used by
// Correction 1c's own chain-length guard below.
double edge_path_length(const std::vector<Point2>& vertices, const std::set<Edge>& edges) {
    double total = 0;
    for (const auto& e : edges) total += dist(vertices[e.first], vertices[e.second]);
    return total;
}

// ---- Correction 1c: VBS-point link-polygon merge (K>=3 incident triangles) ------
//
// Same underlying operation as Correction 1 (remove an `is_vbs_point`
// vertex, retriangulate its link) but for the shape Correction 1's own
// `tri_idxs.size() == 2` gate can't reach: a VBS point with 3 or more
// incident triangles, produced when Correction 2.5's debridge introduces an
// extra internal diagonal one hop away from a genuine junction it resolves.
// Confirmed on real production geometry (FPTF-45 Body.stl layer 352): vertex
// 115 is `is_vbs_point` (safe by the same provenance signal Correction 1
// already trusts) with real boundary neighbours 114 and 117, but debridge
// gives it a third incident triangle via a new (115,112) diagonal -- so
// Correction 1 declines it even though removing it is exactly as safe as
// any 2-triangle case. Originally scoped to exactly K=3 (a quad link); later
// confirmed on the same model (layer 438, z=43.35-43.85, a stable 5-layer
// band) that debridge can leave a VBS point with 4 or even 5 incident
// triangles depending on the local curvature -- same provenance guarantee,
// same safety argument, just a bigger link polygon. Generalized to any
// K>=3 rather than special-cased again per K, on the same "the tag is what
// makes this safe, not the specific count" reasoning that motivated K=3
// in the first place.
//
// For K incident triangles, v's own "link" (the chain of non-v edges across
// all K triangles) is a (K+1)-vertex open chain X...Y, with v-X and v-Y its
// two genuine boundary edges. Removing v leaves this (K+1)-gon to
// retriangulate into K-1 triangles -- via `ear_clip_gated` below, which
// generalizes Correction 2's own two-diagonal quad logic (try both
// diagonals, shorter first, validate winding sign) to an arbitrary link
// size: at each step, try every candidate ear in order of its own new
// diagonal's length (shortest first, exactly reproducing the K=3
// short-diagonal preference), and take the first that clears winding sign,
// height, and chain-length. For K=3 specifically this produces the exact
// same 2-triangle result the original quad-only logic did (there are only
// two distinct diagonals in a quad, and shortest-first ordering is
// preserved) -- confirmed by keeping the existing K=3 unit tests unchanged.
//
// One addition Correction 2 doesn't need, reused unchanged from the
// original K=3 design: every resulting triangle's height (Correction 0's
// own `triangle_height` metric) must clear 0.05mm, not just a non-zero/
// correctly-signed area. On the real K=3 case, the shorter candidate
// diagonal (112-114, the chord of the gently-curving 112-113-114 run)
// passes a bare sign check but produces a height~0.01mm sliver; only the
// height gate reliably rejects it in favor of the other diagonal (113-117).
//
// Runs only after Correction 2.5 (debridge): the K>=3 shape doesn't exist
// until debridge creates the extra diagonal(s). Runs to a fixed point (the
// same `while(true)` re-scan-after-each-merge loop every other correction
// in this file uses) so a cascade -- one merge dropping a neighbouring
// vertex's own incident-triangle count until *it* becomes eligible too --
// is picked up automatically on the next pass, without any explicit
// multi-hop walk of its own.
//
// A second, independent gate: `chain_length_limit` (1/10 of the *original*
// contour's own total perimeter, computed once from mesh.constrained,
// before any correction touches it). Confirmed necessary on a plain
// rectangular bar with only 1 VBS-inserted point per long wall: Correction
// 1's own K=2 pass removes one of the two points first (pre-existing,
// already-shipped behavior), and the resulting merged triangle spans nearly
// the whole bar -- which is enough, on its own, to push the *other* VBS
// point from 2 incident triangles to 3. Correction 1c would then remove
// that second point too (equally "safe" by the is_vbs_point/height/sign
// gates alone), collapsing the entire wall down to its 2 original corners
// and completely erasing VBS's own edge-length-variance bound for this
// domain -- not a local junction fix at all, just VBS's own subdivision
// undone. Each individual merge is locally valid; the two gates above (tag,
// height) only protect against a *bad* merge, not a *globally destructive*
// one. Bounding the resulting triangles' own longest edge against a
// fraction of the whole contour's length catches exactly this: on the real
// production case the merged edge is a few mm against a perimeter of tens
// of mm (nowhere close to the limit); on the plain-bar case the merged edge
// is the *entire* 60mm wall against a 140mm perimeter (over 1/10, declined
// either way the diagonal is chosen).
struct Correction1cResult {
    std::vector<Triangle> triangles;
    std::set<Edge> boundary_edges;
    int quad_merge_count;
};

// p is treated as inside-or-on-boundary of triangle (a,b,c) (whose winding
// matches `poly_sign`) if every one of its three signed half-plane tests
// agrees with `poly_sign` or is exactly zero -- conservative on purpose (an
// ear vertex sitting exactly on the candidate ear's own boundary blocks the
// clip rather than risking a degenerate/overlapping triangulation).
bool point_in_triangle_strict(const Point2& p, const Point2& a, const Point2& b, const Point2& c, int poly_sign) {
    int s1 = sign_eps(signed_area2(a, b, p));
    int s2 = sign_eps(signed_area2(b, c, p));
    int s3 = sign_eps(signed_area2(c, a, p));
    return (s1 == poly_sign || s1 == 0) && (s2 == poly_sign || s2 == 0) && (s3 == poly_sign || s3 == 0);
}

// General ear-clipping triangulation of a simple polygon `poly` (indices
// into `vertices`, in winding order) into poly.size()-2 triangles --
// generalizes Correction 1c's own original quad-diagonal logic (K=3,
// exactly 2 candidate diagonals) to any link size. At each step, every
// candidate ear is tried in order of its own new diagonal's length
// (shortest first -- for K=3 this exactly reproduces the original
// shorter-diagonal-first preference, since a quad has only 2 distinct
// diagonals), and the first candidate that (a) matches the polygon's own
// overall winding sign, (b) clears `height_epsilon` (Correction 0's own
// triangle_height metric), (c) clears `chain_length_limit` on its own
// longest edge, and (d) contains no other remaining polygon vertex (the
// standard ear-clipping simplicity guard) is taken. Declines (returns
// std::nullopt) rather than guessing at a partial or best-effort
// triangulation if any step can't find a valid ear -- matching this file's
// universal convention of declining over guessing.
std::optional<std::vector<Triangle>> ear_clip_gated(const std::vector<int>& poly, const std::vector<Point2>& vertices,
                                                      double height_epsilon, double chain_length_limit) {
    int n = static_cast<int>(poly.size());
    if (n < 3) return std::nullopt;

    double shoelace = 0.0;
    for (int i = 0; i < n; ++i) {
        const Point2& a = vertices[poly[i]];
        const Point2& b = vertices[poly[(i + 1) % n]];
        shoelace += a.x * b.y - b.x * a.y;
    }
    int poly_sign = sign_eps(shoelace);
    if (poly_sign == 0) return std::nullopt;

    // Reject a self-intersecting input polygon up front. Ear-clipping's own
    // per-ear "no remaining vertex inside this triangle" check (below) does
    // NOT by itself guard against two non-adjacent *edges* crossing far from
    // any candidate ear -- confirmed necessary on real production geometry
    // (a FeatherPrint "tab" part, z=5.000mm): the K=3 case's own explicit
    // quad self-crossing check (segments_properly_cross on both diagonals)
    // had no K-general equivalent here, and a real K=4+ link whose own
    // vertices wind back across themselves was accepted, silently producing
    // two fresh junction triangles (a Ring domain fractured into 3) instead
    // of resolving the original one.
    for (int i = 0; i < n; ++i) {
        int a1 = poly[i], b1 = poly[(i + 1) % n];
        for (int j = i + 1; j < n; ++j) {
            int a2 = poly[j], b2 = poly[(j + 1) % n];
            if (a1 == a2 || a1 == b2 || b1 == a2 || b1 == b2) continue;  // adjacent edges share a vertex
            if (segments_properly_cross(vertices[a1], vertices[b1], vertices[a2], vertices[b2])) return std::nullopt;
        }
    }

    auto tri_max_edge = [&](int a, int b, int c) {
        return std::max({dist(vertices[a], vertices[b]), dist(vertices[b], vertices[c]), dist(vertices[c], vertices[a])});
    };

    std::vector<int> remaining = poly;
    std::vector<Triangle> result;
    int guard = 0;
    while (remaining.size() > 2 && guard++ < 1000) {
        int m = static_cast<int>(remaining.size());
        struct EarCandidate {
            double diag_len;
            int i;
        };
        std::vector<EarCandidate> ears;
        ears.reserve(m);
        for (int i = 0; i < m; ++i) {
            int ia = remaining[(i - 1 + m) % m];
            int ic = remaining[(i + 1) % m];
            ears.push_back({dist(vertices[ia], vertices[ic]), i});
        }
        std::stable_sort(ears.begin(), ears.end(),
                          [](const EarCandidate& a, const EarCandidate& b) { return a.diag_len < b.diag_len; });

        bool clipped = false;
        for (const auto& ec : ears) {
            int i = ec.i;
            int ia = remaining[(i - 1 + m) % m];
            int ib = remaining[i];
            int ic = remaining[(i + 1) % m];
            if (sign_eps(signed_area2(vertices[ia], vertices[ib], vertices[ic])) != poly_sign) continue;
            double h = triangle_height(vertices[ia], vertices[ib], vertices[ic]);
            if (h < height_epsilon) continue;
            if (tri_max_edge(ia, ib, ic) > chain_length_limit) continue;
            bool any_inside = false;
            for (int j = 0; j < m; ++j) {
                if (j == (i - 1 + m) % m || j == i || j == (i + 1) % m) continue;
                if (point_in_triangle_strict(vertices[remaining[j]], vertices[ia], vertices[ib], vertices[ic], poly_sign)) {
                    any_inside = true;
                    break;
                }
            }
            if (any_inside) continue;
            result.push_back({ia, ib, ic});
            remaining.erase(remaining.begin() + i);
            clipped = true;
            break;
        }
        if (!clipped) return std::nullopt;
    }
    return result;
}

Correction1cResult correct_straight_vertices_k3(std::vector<Triangle> triangles, const std::vector<Point2>& vertices,
                                                 const std::vector<bool>& is_vbs_point,
                                                 std::set<Edge> boundary_edges, double chain_length_limit) {
    int quad_merge_count = 0;

    while (true) {
        auto vtx_to_tris = vertex_to_tris_ordered(triangles);
        bool changed = false;

        for (auto& [v, tri_idxs] : vtx_to_tris) {
            // Upper bound on K, not just a lower one: a VBS point with a
            // huge number of incident triangles is not the debridge-created
            // "one hop from a junction" shape this correction targets -- it
            // is far more likely a genuine dense mesh hub (e.g. the tip of a
            // thin double-wall feature, where the CDT naturally fans many
            // triangles off one point). Confirmed on real production
            // geometry (a FeatherPrint "tab" part, z=5.000mm): a K=17/18
            // vertex there has a link that walks straight down an actual
            // long wall run and back, and retriangulating that huge a
            // region produced two fresh junction triangles (a clean Ring
            // domain fractured into 3) instead of resolving anything.
            // Every K directly confirmed safe so far (layer 352: K=3; layer
            // 438's own 5-layer band: K=4, once K=5) stays well under this;
            // kMaxK gives modest headroom above the largest of those (5)
            // rather than accepting arbitrary K on the strength of the
            // is_vbs_point tag alone -- same "fix only the confirmed shape"
            // discipline every other correction in this file already
            // follows.
            constexpr int kMaxK = 6;
            if (!is_vbs_point[v] || tri_idxs.size() < 3 || tri_idxs.size() > kMaxK) continue;
            const int K = static_cast<int>(tri_idxs.size());

            // Each incident triangle's own edge not touching v is one edge
            // of v's link. Chain the K outer edges into a simple open path.
            std::vector<Edge> outer_edges;
            bool shape_ok = true;
            for (int ti : tri_idxs) {
                std::vector<int> others;
                for (int x : triangles[ti])
                    if (x != v) others.push_back(x);
                if (others.size() != 2) {
                    shape_ok = false;
                    break;
                }
                outer_edges.push_back(make_edge(others[0], others[1]));
            }
            if (!shape_ok) continue;

            std::map<int, std::vector<int>> adj;
            for (const auto& e : outer_edges) {
                adj[e.first].push_back(e.second);
                adj[e.second].push_back(e.first);
            }
            if (static_cast<int>(adj.size()) != K + 1) continue;
            std::vector<int> endpoints;
            for (const auto& [vtx, nbrs] : adj) {
                if (nbrs.size() == 1)
                    endpoints.push_back(vtx);
                else if (nbrs.size() != 2) {
                    shape_ok = false;
                    break;
                }
            }
            if (!shape_ok || endpoints.size() != 2) continue;

            std::vector<int> chain = {endpoints[0]};
            std::set<int> visited = {endpoints[0]};
            int cur = endpoints[0], prev = -1;
            while (cur != endpoints[1]) {
                int next = -1;
                for (int n : adj[cur]) {
                    if (n != prev) {
                        next = n;
                        break;
                    }
                }
                if (next < 0 || visited.count(next)) {
                    shape_ok = false;
                    break;
                }
                chain.push_back(next);
                visited.insert(next);
                prev = cur;
                cur = next;
            }
            if (!shape_ok || static_cast<int>(chain.size()) != K + 1) continue;

            int X = chain.front(), Y = chain.back();
            Edge vX = make_edge(v, X), vY = make_edge(v, Y);
            if (!boundary_edges.count(vX) || !boundary_edges.count(vY)) continue;

            auto retriangulated = ear_clip_gated(chain, vertices, /*height_epsilon=*/0.05, chain_length_limit);
            if (!retriangulated.has_value()) continue;

            std::vector<int> desc = tri_idxs;
            std::sort(desc.rbegin(), desc.rend());
            for (int ti : desc) triangles.erase(triangles.begin() + ti);
            for (const Triangle& t : *retriangulated) triangles.push_back(t);

            boundary_edges.erase(vX);
            boundary_edges.erase(vY);
            boundary_edges.insert(make_edge(X, Y));
            ++quad_merge_count;
            changed = true;
            break;
        }

        if (!changed) return {triangles, boundary_edges, quad_merge_count};
    }
}

// ---- Correction 2.55: cap-corner false-junction fixup ---------------------
//
// At an obtuse end-cap corner, the CDT prefers a triangle that spans
// straight from a wall vertex to the terminal fan's own apex, skipping the
// true wall<->cap corner vertex. Because a wall edge of that region is
// constrained, the spanning triangle is exempt from the Delaunay property --
// the CDT output is *correct*, this is a labeling problem that must be fixed
// post-CDT (external write-up: cdt_endcap_false_junction_fixup.md, sections
// 2 and 3.4). The spanning triangle has bc==0, so Correction 3 fans it into
// a spurious 3-spoke hub, and that fan's own vertices/edges then perturb the
// Stage 6 skeleton and Stage 10 stringer placement -- confirmed on 45 real
// layers of a tube-with-sidewall-hole part (stringer counts flickering
// 14<->13 between adjacent layers, all at the same bottom corner).
//
// Debridge (Correction 2.5) and local re-triangulation (Correction 2.6) both
// miss this: they key off WallUnionFind over boundary_edges and need a clean
// 2-wall split. A pinched-open C-shape Chain domain has its outer *and*
// inner arc on one physical contour, so the union-find gives one component
// and both corrections decline -- the artifact falls straight through to the
// fan.
//
// This correction identifies the spanning triangle by its actual signature
// (lone bc==0, one vertex a terminal-fan apex, the other two boundary
// vertices far apart along the contour) and repairs it with a single Lawson
// flip toward the skipped corner (the write-up's section 3.2). The flip is
// accepted only when it is CDT-legal (quad strictly convex, edge not
// constrained) AND strictly improves -- both resulting triangles gain a
// boundary edge, so bc==0 count drops with no new junction. When no edge
// qualifies, the triangle is left for Correction 3 exactly as before -- this
// pass never makes the mesh worse. VBCT's Stage 5 contract (spec S:8.7: no
// bc==0 triangle survives) rules out the write-up's "reclassify only" path.

struct CapCornerResult {
    std::vector<Triangle> triangles;
    int fixup_count;
};

CapCornerResult fixup_cap_corner_false_junctions(const std::vector<Point2>& vertices,
                                                  std::vector<Triangle> triangles,
                                                  const std::set<Edge>& boundary_edges) {
    // u,v must be more than this many boundary-edge hops apart along the
    // contour -- i.e. the spanning edge skips a real corner, not a near-wall
    // chord. Small enough to still reject a genuine branch junction whose own
    // vertices sit within a few contour hops, large enough to clear a
    // rounded cap's own point spacing.
    constexpr int kNonAdjacentHopMin = 3;

    std::unordered_map<int, std::vector<int>> b_adj;
    std::set<int> b_verts;
    for (const auto& e : boundary_edges) {
        b_adj[e.first].push_back(e.second);
        b_adj[e.second].push_back(e.first);
        b_verts.insert(e.first);
        b_verts.insert(e.second);
    }
    auto within_hops = [&](int a, int b, int cap) -> bool {
        if (a == b) return true;
        std::set<int> seen{a};
        std::vector<int> frontier{a};
        for (int depth = 1; depth <= cap && !frontier.empty(); ++depth) {
            std::vector<int> next;
            for (int uu : frontier) {
                auto it = b_adj.find(uu);
                if (it == b_adj.end()) continue;
                for (int w : it->second) {
                    if (seen.count(w)) continue;
                    if (w == b) return true;
                    seen.insert(w);
                    next.push_back(w);
                }
            }
            frontier.swap(next);
        }
        return false;
    };

    auto tri_key = [](const Triangle& t) {
        std::array<int, 3> k{t[0], t[1], t[2]};
        std::sort(k.begin(), k.end());
        return k;
    };

    int fixup_count = 0;
    // Once a triangle with a given vertex set has been flipped, don't flip it
    // back -- guards against an A->B->A oscillation on a stubborn quad.
    std::set<std::array<int, 3>> spent;
    bool changed = true;
    int guard = 0;
    while (changed && guard++ < 200) {
        changed = false;
        auto e2t = edge_to_tris(triangles);

        for (int ti = 0; ti < static_cast<int>(triangles.size()); ++ti) {
            const Triangle& T = triangles[ti];
            if (boundary_count(T, boundary_edges) != 0) continue;
            if (spent.count(tri_key(T))) continue;

            // (1) lone bc==0, AND (2) at least one across-edge neighbour is a
            // terminal (bc==2) triangle. A genuine branch junction (tee,
            // plus, comb, y-branch) borders only sleeves -- never a terminal
            // cap fan -- so "lone bc==0 next to a terminal" is the cap-corner
            // spanning signature (write-up section 3.1/3.4).
            bool lone = true;
            bool terminal_neighbour = false;
            for (int slot = 0; slot < 3; ++slot) {
                auto it = e2t.find(make_edge(T[slot], T[(slot + 1) % 3]));
                if (it == e2t.end() || it->second.size() != 2) { lone = false; break; }
                int ni = (it->second[0] == ti) ? it->second[1] : it->second[0];
                int nbc = boundary_count(triangles[ni], boundary_edges);
                if (nbc == 0) lone = false;
                if (nbc == 2) terminal_neighbour = true;
            }
            if (!lone || !terminal_neighbour) continue;

            // (3) all three vertices are boundary vertices, and at least one
            // pair is far apart along the contour -- the spanning edge that
            // skips a real corner.
            if (!b_verts.count(T[0]) || !b_verts.count(T[1]) || !b_verts.count(T[2])) continue;
            bool spans_corner = false;
            for (int s = 0; s < 3; ++s) {
                if (!within_hops(T[s], T[(s + 1) % 3], kNonAdjacentHopMin)) { spans_corner = true; break; }
            }
            if (!spans_corner) continue;

            // Repair: try each non-constrained edge of T; flip toward the
            // neighbour's opposite vertex. Accept only a CDT-legal flip (quad
            // strictly convex) that strictly resolves -- BOTH replacement
            // triangles gain a boundary edge, so bc==0 count drops with no new
            // junction. When no edge qualifies (e.g. a leftover near-sliver
            // wedged against the same corner), the triangle is left for
            // Correction 3 exactly as today -- this pass never makes the mesh
            // worse. `spent` + the guard cap stop any pathological loop.
            int ref_sign = sign_eps(signed_area2(vertices[T[0]], vertices[T[1]], vertices[T[2]]));
            bool did_flip = false;
            for (int slot = 0; slot < 3 && !did_flip; ++slot) {
                int a = T[slot], b = T[(slot + 1) % 3];
                Edge ab = make_edge(a, b);
                if (boundary_edges.count(ab)) continue;  // never flip a constrained edge
                auto it = e2t.find(ab);
                if (it == e2t.end() || it->second.size() != 2) continue;
                int ni = (it->second[0] == ti) ? it->second[1] : it->second[0];
                const Triangle& N = triangles[ni];
                if (boundary_count(N, boundary_edges) == 0) continue;  // never unzip against another junction
                int c = -1;
                for (int w : N) if (w != a && w != b) c = w;
                if (c < 0) continue;
                int opp = T[(slot + 2) % 3];  // T's own third vertex

                // Quad (a, opp, b, c): diagonals (a,b) and (opp,c) must cross.
                if (!segments_properly_cross(vertices[a], vertices[b], vertices[opp], vertices[c])) continue;

                Triangle f1{opp, a, c};
                if (sign_eps(signed_area2(vertices[f1[0]], vertices[f1[1]], vertices[f1[2]])) != ref_sign)
                    std::swap(f1[1], f1[2]);
                Triangle f2{opp, c, b};
                if (sign_eps(signed_area2(vertices[f2[0]], vertices[f2[1]], vertices[f2[2]])) != ref_sign)
                    std::swap(f2[1], f2[2]);
                if (sign_eps(signed_area2(vertices[f1[0]], vertices[f1[1]], vertices[f1[2]])) == 0) continue;
                if (sign_eps(signed_area2(vertices[f2[0]], vertices[f2[1]], vertices[f2[2]])) == 0) continue;

                if (boundary_count(f1, boundary_edges) < 1 || boundary_count(f2, boundary_edges) < 1) continue;

                triangles[ti] = f1;
                triangles[ni] = f2;
                spent.insert(tri_key(T));
                ++fixup_count;
                did_flip = true;
                changed = true;
            }
            if (did_flip) break;  // e2t stale
        }
    }

    return {triangles, fixup_count};
}

// ---- Correction 2.7: short-stub absorption ---------------------------------
//
// A genuine junction triangle J (0 boundary edges) can have one of its three
// arms be a short run of sleeve triangles ending immediately in a real
// dead-end tip -- a genuine branch, just too short to matter (this is not a
// VBS artifact; Correction 2's straightness test doesn't apply here). Traced
// directly, on a real production part, to Stage 9's own splicing machinery
// repeatedly failing to correctly re-absorb such a short spoke once it
// crosses paths with a neighbouring hub's own independent splice (three
// separate attempts at fixing this in Stage 9 itself, each confirmed via
// real print as either ineffective or an outright regression). Absorbing
// the stub here, at the mesh level, sidesteps that machinery for this shape
// entirely: by the time Stage 6 ever looks at this location, it is an
// ordinary sleeve run, not a branch of any kind -- Stage 9 has nothing left
// here to prune or splice.
//
// Geometry: J=(A,B,C) with edge (A,B) leading into the short arm -- a chain
// of sleeve triangles whose own two sides are real wall-point sequences
// starting at A and at B, converging at a dead-end tip T after `prune_length`
// of real wall distance (`walk_stub_arm`, below, walks it and reports both
// chains). Removing every triangle from J out to the tip and re-filling the
// resulting pocket with a *fan from C* -- one triangle per consecutive pair
// along the combined path A -> (side-A's own points) -> T ->
// (side-B's own points, reversed) -> B -- turns every one of those triangles
// into a sleeve (each contributes exactly the one real wall edge between its
// own consecutive path points; every edge touching C is a fresh, non-
// promoted CDT diagonal). No boundary-edge bookkeeping is needed: every real
// wall edge along the path was already boundary, (C,A)/(C,B) stay whatever
// they already were (interior, shared with J's other two neighbours), and
// (A,B) simply ceases to exist as an edge (it was never boundary to begin
// with, since J had none). No area is deleted -- every real point on the
// arm keeps its position, so the resulting wall still traces the stub's own
// true boundary rather than cutting across it, the never-deletes-area
// property every correction in this file preserves. A single-triangle arm
// (T adjacent to J directly) is the `path = [A, T, B]` special case of this
// same fan -- exactly the shape this correction originally, more narrowly,
// handled.
//
// Scoped to a *lone* junction triangle (not part of a larger hub -- its
// other two neighbours must not themselves be junction triangles) with
// *exactly one* arm that both (a) reaches a dead end without passing through
// another junction, and (b) measures shorter than `prune_threshold` (the
// same corrugated-stringer prune threshold Stage 9 already applies to
// exactly this class of chain -- this correction only ever pre-empts a
// splice Stage 9 would otherwise have to attempt, never prunes something
// Stage 9 wouldn't have). `prune_threshold <= 0` disables this correction
// entirely (the default for every caller that has no such threshold, e.g.
// the case library). A hub of size >1, or a junction with more than one
// short arm, is left to Correction 3's general fan as before.
struct StubAbsorbResult {
    std::vector<Triangle> triangles;
    int absorbed_count = 0;
};

// One arm's own walk outward from J's arm-facing edge (A,B), through
// consecutive sleeve triangles, to a dead-end tip. `chain_a`/`chain_b` are
// each the real wall-point sequence on their own side, from A (resp. B)
// to the tip T inclusive on both. Returns !ok, without side effects, if the
// arm runs into another junction, a malformed/looping mesh, or anything
// else that isn't a plain, single dead-end run -- the caller leaves such an
// arm untouched rather than guess.
struct ArmWalkResult {
    bool ok = false;
    std::vector<int> chain_a;
    std::vector<int> chain_b;
    std::set<int> arm_triangles;
};

ArmWalkResult walk_stub_arm(int A, int B, int start_tri, const std::vector<Triangle>& triangles,
                             const std::set<Edge>& boundary_edges, const std::map<Edge, std::vector<int>>& e2t) {
    ArmWalkResult result;
    result.chain_a.push_back(A);
    result.chain_b.push_back(B);
    int u = A, v = B, cur = start_tri;
    int max_hops = static_cast<int>(triangles.size()) + 1;
    for (int hops = 0; hops <= max_hops; ++hops) {
        if (cur < 0 || cur >= static_cast<int>(triangles.size())) return {};
        if (result.arm_triangles.count(cur)) return {};  // revisited a triangle -- malformed/looping
        const Triangle& X = triangles[cur];
        int w = -1;
        for (int vtx : X)
            if (vtx != u && vtx != v) w = vtx;
        if (w < 0) return {};

        int bc = boundary_count(X, boundary_edges);
        bool uw_boundary = boundary_edges.count(make_edge(u, w)) != 0;
        bool vw_boundary = boundary_edges.count(make_edge(v, w)) != 0;

        if (bc == 2) {
            if (!uw_boundary || !vw_boundary) return {};  // not a plain tip on this shared edge
            result.arm_triangles.insert(cur);
            result.chain_a.push_back(w);
            result.chain_b.push_back(w);
            result.ok = true;
            return result;
        }
        if (bc != 1 || uw_boundary == vw_boundary) return {};  // junction, or malformed sleeve
        result.arm_triangles.insert(cur);
        Edge next_edge;
        if (uw_boundary) {
            result.chain_a.push_back(w);
            u = w;
        } else {
            result.chain_b.push_back(w);
            v = w;
        }
        next_edge = make_edge(u, v);
        auto it = e2t.find(next_edge);
        if (it == e2t.end() || it->second.size() != 2) return {};
        cur = (it->second[0] == cur) ? it->second[1] : it->second[0];
    }
    return {};  // hop limit -- shouldn't happen on a sane mesh; decline rather than risk an infinite walk
}

double chain_wall_length(const std::vector<Point2>& vertices, const std::vector<int>& chain) {
    double sum = 0;
    for (size_t i = 0; i + 1 < chain.size(); ++i) sum += dist(vertices[chain[i]], vertices[chain[i + 1]]);
    return sum;
}

StubAbsorbResult absorb_short_stubs(const std::vector<Point2>& vertices, std::vector<Triangle> triangles,
                                     const std::set<Edge>& boundary_edges, double prune_threshold) {
    if (prune_threshold <= 0) return {std::move(triangles), 0};
    int absorbed_count = 0;
    std::vector<Triangle>& tris = triangles;

    bool changed = true;
    int guard = 0;
    while (changed && guard++ < 500) {
        changed = false;
        auto e2t = edge_to_tris(tris);

        for (int ji = 0; ji < static_cast<int>(tris.size()); ++ji) {
            if (boundary_count(tris[ji], boundary_edges) != 0) continue;
            const Triangle& J = tris[ji];

            bool lone_hub = true;
            for (int slot = 0; slot < 3 && lone_hub; ++slot) {
                int u = J[slot], v = J[(slot + 1) % 3];
                auto it = e2t.find(make_edge(u, v));
                if (it == e2t.end() || it->second.size() != 2) { lone_hub = false; break; }
                int ni = (it->second[0] == ji) ? it->second[1] : it->second[0];
                if (boundary_count(tris[ni], boundary_edges) == 0) lone_hub = false;  // a neighbouring junction
            }
            if (!lone_hub) continue;

            struct QualifyingArm {
                int slot;
                ArmWalkResult walk;
                double len;
            };
            std::vector<QualifyingArm> qualifying;
            for (int slot = 0; slot < 3; ++slot) {
                int A = J[slot], B = J[(slot + 1) % 3];
                auto it = e2t.find(make_edge(A, B));
                if (it == e2t.end() || it->second.size() != 2) continue;
                int start_tri = (it->second[0] == ji) ? it->second[1] : it->second[0];
                ArmWalkResult w = walk_stub_arm(A, B, start_tri, tris, boundary_edges, e2t);
                if (!w.ok) continue;
                double len = (chain_wall_length(vertices, w.chain_a) + chain_wall_length(vertices, w.chain_b)) / 2.0;
                if (len >= prune_threshold) continue;
                qualifying.push_back({slot, std::move(w), len});
            }
            // 3 qualifying arms has no natural "kept" side to anchor a fan
            // from -- decline rather than guess, this file's universal
            // convention. 0 means nothing here is short enough to touch.
            if (qualifying.empty() || qualifying.size() > 2) continue;

            std::vector<Triangle> fan;
            std::set<int> free_tris;

            // Original single-arm case, factored out so the two-arm case
            // (below) can fall back to it: the OTHER two edges of J stay
            // completely untouched, and the pruned arm's own combined path
            // (A -> ... -> T -> ... -> B) fans from J's third vertex C,
            // which both untouched edges already connect to.
            int ref_sign = sign_eps(signed_area2(vertices[J[0]], vertices[J[1]], vertices[J[2]]));
            auto try_single_arm_fan = [&](const QualifyingArm& q) -> std::optional<std::vector<Triangle>> {
                int C = J[(q.slot + 2) % 3];
                // Combined path C's fan sweeps across: A -> side-A's own real
                // points -> T -> side-B's own points, reversed -> B. Both
                // chains already end at the tip T (walk_stub_arm's own
                // convention), so appending chain_b reversed, minus its own
                // last element (T, to avoid repeating it), joins cleanly.
                std::vector<int> path = q.walk.chain_a;
                for (auto it = q.walk.chain_b.rbegin() + 1; it != q.walk.chain_b.rend(); ++it) path.push_back(*it);
                std::vector<Triangle> f;
                for (size_t i = 0; i + 1 < path.size(); ++i) {
                    Triangle t{C, path[i], path[i + 1]};
                    double area = signed_area2(vertices[t[0]], vertices[t[1]], vertices[t[2]]);
                    if (sign_eps(area) != ref_sign || std::abs(area) < 1e-12) return std::nullopt;
                    f.push_back(t);
                }
                return f;
            };

            if (qualifying.size() == 1) {
                auto f = try_single_arm_fan(qualifying[0]);
                if (!f.has_value()) continue;
                fan = *f;
                free_tris = qualifying[0].walk.arm_triangles;
            } else {
                // Two qualifying (short) arms, one long arm left over.
                // Originally attempted as a single combined closure (walk
                // from the kept edge's own P, through both pruned arms' own
                // paths via their shared hinge vertex R, back to Q, then
                // hand the whole freed polygon to the same general
                // ear-clipping triangulator Correction 1c uses) -- but two
                // distinct real-production failure modes falsified that
                // approach: (1) R can be a genuine reflex vertex of the
                // combined region, making the original {P,Q,R} junction the
                // unavoidable closing triangle of ANY valid triangulation of
                // it (FPTF-45 Body.stl layer 433); (2) even when a
                // triangulation avoiding {P,Q,R} exists, ear_clip_gated's
                // own greedy shortest-diagonal search can still legitimately
                // fan multiple new diagonals through a single real wall
                // point elsewhere in the polygon, turning what was a lone
                // junction into a fresh 3-triangle junction *cluster* --
                // worse than doing nothing (FPTF-45 Body.stl layer 603,
                // z=60.25mm: hub_sizes [1] -> [3]). Both are inherent to
                // asking a general-purpose triangulator to reconstruct a
                // *specific* two-armed shape rather than a real fix.
                //
                // So: always fall back straight to the proven single-arm
                // mechanism, pruning only the shorter of the two qualifying
                // arms. Strictly less than "prune all but the longest", but
                // it reuses the exact same fan-from-the-opposite-vertex
                // logic already proven safe for the one-arm case (never
                // needs to close back to a persistent kept edge, so neither
                // failure mode above can occur), and still resolves the
                // junction outright: the leftover longer short arm becomes
                // an ordinary dead-end sleeve spur, not a junction.
                const QualifyingArm& shorter = (qualifying[0].len <= qualifying[1].len) ? qualifying[0] : qualifying[1];
                auto f = try_single_arm_fan(shorter);
                if (!f.has_value()) continue;
                fan = *f;
                free_tris = shorter.walk.arm_triangles;
            }

            // Replace {J} u the pruned arm(s)' own triangles with the fan.
            // ji's own slot takes the fan's first triangle; the rest either
            // reuse a freed slot or, if the fan is longer than the removed
            // set, get appended (a walk of n triangles removes n triangles
            // and any triangulation of a path of length n+1 always produces
            // exactly n triangles too, so for a single arm this never
            // happens; the two-arm case can legitimately differ since
            // ear_clip_gated doesn't fan from one point, so the append path
            // is load-bearing there, not just defensive).
            std::vector<int> free_slots(free_tris.begin(), free_tris.end());
            free_slots.push_back(ji);
            std::sort(free_slots.begin(), free_slots.end());
            size_t fi = 0;
            for (int slot : free_slots) {
                if (fi < fan.size())
                    tris[slot] = fan[fi++];
            }
            for (; fi < fan.size(); ++fi) tris.push_back(fan[fi]);

            ++absorbed_count;
            changed = true;
            break;  // e2t and every triangle index are stale after a replacement; rebuild before continuing
        }
    }

    return {std::move(tris), absorbed_count};
}

// ---- Correction 3: junction hub geometry (single_point only) --------------

struct ClusterResult {
    std::vector<std::set<int>> hubs;
};

ClusterResult cluster_junction_triangles(const std::vector<Triangle>& triangles, const std::set<Edge>& boundary_edges) {
    auto e2t = edge_to_tris(triangles);
    std::vector<int> junction_idx;
    for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
        if (boundary_count(triangles[i], boundary_edges) == 0) junction_idx.push_back(i);
    }
    std::set<int> junction_set(junction_idx.begin(), junction_idx.end());

    std::map<int, std::vector<int>> adj;
    for (int i : junction_idx) adj[i] = {};
    for (const auto& [key, tl] : e2t) {
        if (tl.size() == 2) {
            int i = tl[0], j = tl[1];
            if (junction_set.count(i) && junction_set.count(j)) {
                adj[i].push_back(j);
                adj[j].push_back(i);
            }
        }
    }

    std::set<int> seen;
    std::vector<std::set<int>> hubs;
    for (int i : junction_idx) {
        if (seen.count(i)) continue;
        std::vector<int> stack{i};
        std::set<int> comp;
        seen.insert(i);
        while (!stack.empty()) {
            int cur = stack.back();
            stack.pop_back();
            comp.insert(cur);
            for (int nb : adj[cur]) {
                if (!seen.count(nb)) {
                    seen.insert(nb);
                    stack.push_back(nb);
                }
            }
        }
        hubs.push_back(comp);
    }
    return {hubs};
}

// ---- Correction 2.6: general local re-triangulation of stubborn hubs ------
//
// Correction 2.5's debridge handles the common "single/skip-run bridging
// triangle" pattern via a targeted diagonal flip, but real-world wall
// generation (e.g. Cura's Arachne variable-width beading) can produce
// genuinely different local point topology at the same corner from one
// layer to the next -- not just coordinate jitter on the same points, but
// a different point count/arrangement -- for an otherwise Z-invariant
// model. Confirmed directly on a real part: the inner wall's points were
// bit-identical across several Z heights, but the *other* wall's points
// near the same corner had a different count and local arrangement at
// each height. Debridge's single-hop flip can't unwind an arbitrary such
// shape (it tries, but its convexity guard correctly refuses whatever
// isn't a valid single flip, leaving the hub in place).
//
// This pass instead re-triangulates each remaining junction cluster's own
// footprint from scratch via CDT (the same backend Stage 3 uses for the
// whole mesh), constrained to respect whatever genuine wall segments its
// own vertices already form -- looked up in `boundary_edges`, which
// (unlike the current triangle set) already reflects the *original*
// contour's real adjacency regardless of which diagonal any triangulation
// happened to choose. The result is the correct ladder triangulation for
// whatever shape the hub actually has, not a hand-tuned pattern match.
// Falls back to leaving the hub untouched (Correction 3 fans it, as
// before) if the cluster's footprint isn't a single simple polygon, or if
// even this general re-triangulation still can't produce an
// all-non-junction result -- the same safety net as before, just reached
// less often.

// Walks a hub's own outer boundary edges (each used by exactly one hub
// triangle) into a single ordered cycle. Returns empty if they don't form
// exactly one simple closed loop -- e.g. a vertex touched by more than 2
// such edges (a figure-eight pinch) -- a case this correction declines to
// guess at rather than risk mishandling.
std::vector<int> order_boundary_loop(const std::vector<Edge>& loop_edges) {
    std::map<int, std::vector<int>> adj;
    for (const auto& [a, b] : loop_edges) {
        adj[a].push_back(b);
        adj[b].push_back(a);
    }
    if (adj.empty()) return {};
    for (const auto& [v, nbrs] : adj) {
        if (nbrs.size() != 2) return {};
    }

    std::vector<int> loop;
    int start = adj.begin()->first;
    int prev = -1, cur = start;
    do {
        loop.push_back(cur);
        int a = adj[cur][0], b = adj[cur][1];
        int next = (a != prev) ? a : b;
        prev = cur;
        cur = next;
    } while (cur != start && loop.size() <= adj.size());

    if (cur != start || loop.size() != adj.size()) return {};  // didn't close cleanly, or revisited a vertex
    return loop;
}

// ---- Correction 1b: terminal-wedge collapse ------------------------------
//
// Correction 1 removes a VBS point incident to exactly 2 triangles fanning
// from one apex. At a flat dead-end corner the CDT can instead triangulate
// the wedge with VBS points that are incident to 3+ triangles, fanning from
// several different apexes -- a shape Correction 1's exact-2 rule can't
// reach. Stage 6 then threads one skeleton segment per sleeve triangle in
// that wedge, so the skeleton staircases up the flat wall to the corner,
// and (confirmed on real geometry) that wrap moves cap_start/cap_end onto
// the corner vertex, jumping ~2mm between otherwise-identical layers.
//
// The tip of the wedge is still a valid triangle: (corner, wallA_pt,
// wallB_pt) with the intermediate wall points collinear on two of its
// edges. Starting from the terminal triangle at the corner we absorb one
// neighbour at a time, inward across interior edges, re-checking after each
// step that the running union is still a near-triangle (3 corners, every
// other boundary-loop vertex collinear within kEps on a wall edge). The
// first absorption that would turn the union into a quad -- e.g. wrapping a
// real pocket corner, or a corridor ladder whose union is a quad strip --
// stops the walk, and we keep whatever triangle-shaped prefix we found
// (any number of internal apexes). That prefix collapses to its one
// triangle, which then has 2 boundary edges (a proper terminal) so Stage 6
// stops there cleanly. Backing off a step is why this catches wedges a
// grow-everything-then-check pass declines whole.

struct WedgeCollapseResult {
    std::vector<Triangle> triangles;
    std::set<Edge> boundary_edges;
    int collapse_count;
};

WedgeCollapseResult collapse_terminal_wedges(const std::vector<Point2>& vertices, std::vector<Triangle> triangles,
                                              const std::vector<bool>& is_vbs_point, std::set<Edge> boundary_edges) {
    constexpr double kEps = 0.01;         // mm; > flat-wall VBS noise (<=6um), < real curvature sagitta
    constexpr double kStraightDot = -0.9; // (x-v)*(y-v) normalised: near -1 = straight through a vertex
    constexpr double kStraightStep = 0.9995; // consecutive wall-segment unit dirs: >= this = same heading (~1.8 deg)
    constexpr double kWallDrift = 0.05;   // mm; absolute cap on a wall run's bow away from c's tangent line

    auto unit = [](const Point2& a, const Point2& b) -> Point2 {
        double dx = b.x - a.x, dy = b.y - a.y, L = std::hypot(dx, dy);
        return L < 1e-12 ? Point2{0, 0} : Point2{dx / L, dy / L};
    };
    // perpendicular distance to the *infinite* line through a,b (unlike
    // point_to_seg, does not blow up once p projects past an endpoint --
    // needed while walking a wall run past its current far point).
    auto point_to_line = [](const Point2& p, const Point2& a, const Point2& b) -> double {
        double dx = b.x - a.x, dy = b.y - a.y, L = std::hypot(dx, dy);
        if (L < 1e-12) return dist(p, a);
        return std::fabs((p.x - a.x) * dy - (p.y - a.y) * dx) / L;
    };
    auto point_to_seg = [](const Point2& p, const Point2& a, const Point2& b) -> double {
        double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
        if (len2 < 1e-18) return dist(p, a);
        double t = std::max(0.0, std::min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
        return dist(p, Point2{a.x + t * dx, a.y + t * dy});
    };

    int collapse_count = 0;
    bool changed = true;
    while (changed) {
        changed = false;

        // boundary-vertex adjacency
        std::unordered_map<int, std::vector<int>> b_adj;
        for (const auto& e : boundary_edges) {
            b_adj[e.first].push_back(e.second);
            b_adj[e.second].push_back(e.first);
        }
        std::unordered_map<int, std::vector<int>> inc;
        for (int ti = 0; ti < static_cast<int>(triangles.size()); ++ti)
            for (int v : triangles[ti]) inc[v].push_back(ti);
        auto e2t = edge_to_tris(triangles);

        for (const auto& [c, nbrs] : b_adj) {
            if (nbrs.size() != 2) continue;  // not a simple boundary vertex
            int x = nbrs[0], y = nbrs[1];
            // corner test: the two boundary directions from c are not
            // near-collinear (a straight-through vertex has dot ~ -1).
            Point2 ux = unit(vertices[c], vertices[x]), uy = unit(vertices[c], vertices[y]);
            if (ux.x * uy.x + ux.y * uy.y < kStraightDot) continue;

            // Walk each wall from c while it stays a single straight run
            // (each new point within kEps of the wall's own line from c).
            // Real contour vertices are walked *through* as long as they stay
            // on that line -- on a real cap wall the flat run is a mix of VBS
            // points and original contour vertices, and stopping at the first
            // contour vertex truncates the wedge (the reason this pass used to
            // miss layer 175's left notch). A genuine corner bends off the
            // line and stops the walk; a junction (deg != 2) stops it too.
            auto walk = [&](int first) -> std::vector<int> {
                std::vector<int> run{c, first};
                Point2 base = vertices[c];
                int prev = c, cur = first;
                while (true) {
                    auto it = b_adj.find(cur);
                    if (it == b_adj.end() || it->second.size() != 2) break;
                    int nxt = (it->second[0] != prev) ? it->second[0] : it->second[1];
                    if (nxt == prev) break;
                    // "straight run" = each step keeps direction (a genuine
                    // corner turns hard). Scale-free, so it isn't fooled by a
                    // short first segment the way an absolute offset is; the
                    // near-triangle loop check downstream bounds cumulative
                    // drift. Also cap the total offset from c's wall line as a
                    // slow-curve backstop.
                    Point2 pd = unit(vertices[prev], vertices[cur]), nd = unit(vertices[cur], vertices[nxt]);
                    if (pd.x * nd.x + pd.y * nd.y < kStraightStep) break;  // wall bends here
                    if (point_to_line(vertices[nxt], base, vertices[cur]) > kWallDrift) break;
                    run.push_back(nxt);
                    prev = cur;
                    cur = nxt;
                }
                return run;
            };
            std::vector<int> A = walk(x), B = walk(y);
            if (A.size() < 2 || B.size() < 2) continue;
            std::set<int> A_set(A.begin(), A.end()), B_set(B.begin(), B.end());

            // Is `tris`' outer boundary loop still a near-valid triangle --
            // exactly 3 corners {c, farthest-A-vertex, farthest-B-vertex},
            // every other loop vertex collinear within kEps on one of the two
            // wall edges? Sets am/bn to those two far vertices on success.
            auto near_triangle = [&](const std::set<int>& tris, int& am_out, int& bn_out) -> bool {
                int am = c, bn = c;
                double da = 0, db = 0;
                for (int ti : tris)
                    for (int v : triangles[ti]) {
                        if (v == c) continue;
                        double dv = dist(vertices[c], vertices[v]);
                        if (A_set.count(v) && dv > da) { da = dv; am = v; }
                        if (B_set.count(v) && dv > db) { db = dv; bn = v; }
                    }
                if (am == c || bn == c) return false;
                std::map<Edge, int> ecount;
                for (int ti : tris)
                    for (const auto& [p, q] : tri_raw_edges(triangles[ti])) ecount[make_edge(p, q)]++;
                std::vector<Edge> outer;
                for (const auto& [e, n] : ecount)
                    if (n == 1) outer.push_back(e);
                std::vector<int> loop = order_boundary_loop(outer);
                if (loop.size() < 3) return false;
                std::set<int> on_loop(loop.begin(), loop.end());
                if (!on_loop.count(c) || !on_loop.count(am) || !on_loop.count(bn)) return false;
                // am and bn must be adjacent on the loop -- the hypotenuse is
                // then the single existing edge (am,bn), so collapsing the
                // wedge changes only its own triangles and can't strand a
                // neighbour across a chord that didn't exist before.
                {
                    int L = static_cast<int>(loop.size()), pa = -1, pb = -1;
                    for (int i = 0; i < L; ++i) {
                        if (loop[i] == am) pa = i;
                        if (loop[i] == bn) pb = i;
                    }
                    int gap = std::abs(pa - pb);
                    if (gap != 1 && gap != L - 1) return false;
                }
                std::set<int> corners{c, am, bn};
                for (int lv : loop) {
                    if (corners.count(lv)) continue;
                    // a genuine pocket corner the union wraps lands far from
                    // both wall edges and fails here; a contour vertex merely
                    // collinear on a flat wall is inert and safe to dissolve.
                    double d = std::min(point_to_seg(vertices[lv], vertices[c], vertices[am]),
                                        point_to_seg(vertices[lv], vertices[c], vertices[bn]));
                    if (d > kEps) return false;
                }
                am_out = am;
                bn_out = bn;
                return true;
            };

            // Terminal triangle at c: the one carrying both first wall steps.
            int t0 = -1;
            for (int ti : inc[c]) {
                bool hasA = false, hasB = false;
                for (int v : triangles[ti]) {
                    if (v == A[1]) hasA = true;
                    if (v == B[1]) hasB = true;
                }
                if (hasA && hasB) { t0 = ti; break; }
            }
            if (t0 < 0) continue;

            // Absorb one triangle at a time, working inward from t0 across
            // interior edges. The FIRST triangle whose addition would turn
            // the running union into a quad stops the walk -- and we keep the
            // triangle-shaped prefix found so far (a grow-everything pass
            // instead declines the whole wedge). "check 1&2 merge, 1&3 merge,
            // 1&4 quad stop."
            std::set<int> region{t0};
            int am = c, bn = c;
            if (!near_triangle(region, am, bn)) continue;
            while (true) {
                int cand = -1;
                bool ambiguous = false;
                for (int ti : region) {
                    for (const auto& [p, q] : tri_raw_edges(triangles[ti])) {
                        Edge e = make_edge(p, q);
                        if (boundary_edges.count(e)) continue;  // never cross a wall
                        auto it = e2t.find(e);
                        if (it == e2t.end() || it->second.size() != 2) continue;
                        int nb = (it->second[0] == ti) ? it->second[1] : it->second[0];
                        if (region.count(nb)) continue;
                        bool on_walls = true;
                        for (int v : triangles[nb])
                            if (!A_set.count(v) && !B_set.count(v)) { on_walls = false; break; }
                        if (!on_walls) continue;
                        if (cand < 0) cand = nb;
                        else if (cand != nb) ambiguous = true;
                    }
                }
                if (cand < 0 || ambiguous) break;
                std::set<int> trial = region;
                trial.insert(cand);
                int tam = am, tbn = bn;
                if (!near_triangle(trial, tam, tbn)) break;  // would make a quad
                region = std::move(trial);
                am = tam;
                bn = tbn;
            }
            if (region.size() < 2) continue;

            // Build the replacement triangle, matched to the region's winding.
            int ref_sign = sign_eps(signed_area2(vertices[triangles[*region.begin()][0]],
                                                 vertices[triangles[*region.begin()][1]],
                                                 vertices[triangles[*region.begin()][2]]));
            Triangle merged{c, am, bn};
            if (sign_eps(signed_area2(vertices[merged[0]], vertices[merged[1]], vertices[merged[2]])) != ref_sign)
                std::swap(merged[1], merged[2]);
            if (sign_eps(signed_area2(vertices[merged[0]], vertices[merged[1]], vertices[merged[2]])) == 0) continue;

            // Apply: drop region triangles, add merged; collapse the A and B
            // wall runs to single boundary edges.
            std::vector<Triangle> kept;
            kept.reserve(triangles.size());
            for (int ti = 0; ti < static_cast<int>(triangles.size()); ++ti)
                if (!region.count(ti)) kept.push_back(triangles[ti]);
            kept.push_back(merged);
            triangles = std::move(kept);

            auto collapse_run = [&](const std::vector<int>& run, int last) {
                for (size_t k = 0; k + 1 < run.size(); ++k) {
                    boundary_edges.erase(make_edge(run[k], run[k + 1]));
                    if (run[k + 1] == last) break;
                }
                boundary_edges.insert(make_edge(c, last));
            };
            collapse_run(A, am);
            collapse_run(B, bn);
            collapse_count += static_cast<int>(region.size()) - 1;
            changed = true;
            break;
        }
    }

    return {std::move(triangles), std::move(boundary_edges), collapse_count};
}

struct LocalRetriResult {
    bool ok = false;
    std::set<int> region;             // final triangle-index footprint consumed (>= the original hub)
    std::vector<Triangle> triangles;  // replacement triangles for that footprint
};

// One attempt at re-triangulating a given triangle-index footprint (not
// necessarily just the junction triangles themselves -- see the caller's
// region-growing loop for why). Returns !ok without side effects on any
// failure -- the caller decides whether and how to grow the footprint and
// retry.
//
// Builds the ladder directly rather than handing the footprint to CDT and
// hoping it prefers the ladder: confirmed on a real case that it doesn't
// have to -- when one wall's points are locally much denser than the
// other's, CDT's own Delaunay-optimal choice can be a "chord" triangle
// cutting straight across the dense wall's own concave stretch (still
// touching no wall edge at all) instead of reaching across the gap, no
// matter how much of the surrounding mesh gets added to the candidate
// point set. A hand-built ladder has no such failure mode: walking both
// walls' own point chains and always advancing whichever gives the
// shorter cross-diagonal (the standard greedy strip triangulation)
// guarantees every triangle uses one real wall edge, by construction, not
// by hoping a general-purpose triangulator prefers it.
LocalRetriResult try_retriangulate_region(const std::vector<Point2>& vertices, const std::set<int>& region,
                                           const std::vector<Triangle>& triangles, const std::set<Edge>& boundary_edges) {
    std::map<Edge, int> edge_count;
    for (int ti : region) {
        for (const auto& [a, b] : tri_raw_edges(triangles[ti])) edge_count[make_edge(a, b)]++;
    }
    std::vector<Edge> outer_edges;
    for (const auto& [e, c] : edge_count) {
        if (c == 1) outer_edges.push_back(e);
    }
    std::vector<int> loop = order_boundary_loop(outer_edges);
    if (loop.size() < 3) return {};

    // Wall membership among the region's own vertices, from the *global*
    // boundary-edge set -- unlike the current triangle set, this already
    // reflects the original contour's real adjacency regardless of which
    // diagonal any triangulation chose.
    WallUnionFind wall;
    for (const auto& e : boundary_edges) wall.unite(e.first, e.second);

    // Split the ordered loop into maximal same-wall runs. A clean 2-wall
    // ladder has exactly 2 transitions (the two "cross" edges connecting
    // one wall's end to the other's start). Anything else -- a third
    // wall's points folded in, or more than 2 transitions -- isn't a shape
    // this correction knows how to handle; decline rather than guess.
    size_t n = loop.size();
    std::vector<size_t> transition_idx;
    for (size_t i = 0; i < n; ++i) {
        if (wall.find(loop[i]) != wall.find(loop[(i + 1) % n])) transition_idx.push_back(i);
    }
    if (transition_idx.size() != 2) return {};

    auto extract_run = [&](size_t from, size_t to) {
        std::vector<int> run;
        for (size_t i = from;; i = (i + 1) % n) {
            run.push_back(loop[i]);
            if (i == to) break;
        }
        return run;
    };
    std::vector<int> chain_a = extract_run((transition_idx[0] + 1) % n, transition_idx[1]);
    std::vector<int> chain_b = extract_run((transition_idx[1] + 1) % n, transition_idx[0]);
    // chain_a runs forward from just after one transition to the other; flip
    // chain_b so its own first/last line up with chain_a's via the two real
    // cross edges (the classic ladder-zipper setup: a0-b0 and am-bn are the
    // strip's two ends).
    std::reverse(chain_b.begin(), chain_b.end());
    if (chain_a.size() + chain_b.size() < 3) return {};

    // Every consecutive pair within a chain must be a genuine wall edge --
    // if region-growing didn't pull in a full contiguous wall run (a real
    // point still missing between two chain neighbors), this ladder would
    // otherwise silently fabricate a "wall" edge that isn't one. Checked
    // explicitly rather than assumed.
    for (const auto* chain : {&chain_a, &chain_b}) {
        for (size_t i = 0; i + 1 < chain->size(); ++i) {
            if (!boundary_edges.count(make_edge((*chain)[i], (*chain)[i + 1]))) return {};
        }
    }

    // Reference winding: match whichever orientation the region's own
    // original triangles already use, so downstream left/right wall-side
    // logic (Stage 8) stays consistent -- a fresh, hand-built triangle
    // order isn't otherwise guaranteed to align with the ambient mesh's
    // convention.
    const Triangle& ref_tri = triangles[*region.begin()];
    int ref_sign = sign_eps(signed_area2(vertices[ref_tri[0]], vertices[ref_tri[1]], vertices[ref_tri[2]]));
    auto oriented = [&](int p, int q, int r) -> Triangle {
        Triangle t{p, q, r};
        if (sign_eps(signed_area2(vertices[t[0]], vertices[t[1]], vertices[t[2]])) != ref_sign) std::swap(t[1], t[2]);
        return t;
    };

    // Classic greedy strip (ladder) triangulation: at each step, advance
    // whichever chain's next point makes the shorter cross-diagonal.
    std::vector<Triangle> result;
    size_t i = 0, j = 0;
    while (i + 1 < chain_a.size() || j + 1 < chain_b.size()) {
        bool advance_a;
        if (i + 1 >= chain_a.size()) {
            advance_a = false;
        } else if (j + 1 >= chain_b.size()) {
            advance_a = true;
        } else {
            double d_a = dist(vertices[chain_a[i + 1]], vertices[chain_b[j]]);
            double d_b = dist(vertices[chain_a[i]], vertices[chain_b[j + 1]]);
            advance_a = d_a <= d_b;
        }
        if (advance_a) {
            result.push_back(oriented(chain_a[i], chain_a[i + 1], chain_b[j]));
            ++i;
        } else {
            result.push_back(oriented(chain_a[i], chain_b[j], chain_b[j + 1]));
            ++j;
        }
    }

    // Should always hold by construction above, but verified explicitly
    // rather than trusted -- matching this file's existing practice for
    // every other correction.
    for (const auto& t : result) {
        if (boundary_count(t, boundary_edges) == 0) return {};
    }
    return {true, region, std::move(result)};
}

// Retries try_retriangulate_region with a growing footprint: if the
// junction cluster's own vertices aren't enough to close a valid ladder
// (the real wall point that would close the gap belongs to a neighboring
// triangle that isn't itself a junction triangle -- confirmed on a real
// case: a triangle in the result was still 0-boundary because the true
// wall neighbors of its vertices lay just outside the original hub),
// absorb the immediately adjacent ring of triangles (across the current
// footprint's own outer boundary) and try again. Bounded, not unbounded,
// growth -- past a few rings this stops being "the hub needed its
// immediate wall neighbors" and starts being "something else is wrong
// here", where falling back to the existing fan-hub behavior is safer
// than an ever-larger, less-local re-triangulation.
LocalRetriResult retriangulate_hub_locally(const std::vector<Point2>& vertices, const std::set<int>& hub,
                                            const std::vector<Triangle>& triangles, const std::set<Edge>& boundary_edges) {
    constexpr int kMaxExpansions = 3;
    auto e2t = edge_to_tris(triangles);
    std::set<int> region = hub;

    for (int round = 0; round <= kMaxExpansions; ++round) {
        LocalRetriResult attempt = try_retriangulate_region(vertices, region, triangles, boundary_edges);
        if (attempt.ok) return attempt;

        // Grow: absorb every triangle across the region's own current
        // outer boundary edges (found the same way try_retriangulate_region
        // does internally) that isn't already part of the region.
        std::map<Edge, int> edge_count;
        for (int ti : region) {
            for (const auto& [a, b] : tri_raw_edges(triangles[ti])) edge_count[make_edge(a, b)]++;
        }
        std::set<int> to_add;
        for (const auto& [e, c] : edge_count) {
            if (c != 1) continue;
            auto it = e2t.find(e);
            if (it == e2t.end()) continue;
            for (int t : it->second) {
                if (!region.count(t)) to_add.insert(t);
            }
        }
        if (to_add.empty()) break;  // nothing left to grow into
        region.insert(to_add.begin(), to_add.end());
    }
    return {};
}

struct Correction3Result {
    std::vector<Triangle> triangles;
    std::set<Edge> boundary_edges;
    std::vector<Edge> hub_edges;
    std::vector<int> hub_sizes;
};

Correction3Result correct_junction_hubs_single_point(std::vector<Point2>& vertices, std::vector<Triangle> triangles,
                                                      std::set<Edge> boundary_edges) {
    auto [hubs] = cluster_junction_triangles(triangles, boundary_edges);
    if (hubs.empty()) return {triangles, boundary_edges, {}, {}};

    std::set<int> hub_member;
    for (const auto& hub : hubs)
        for (int ti : hub) hub_member.insert(ti);

    std::vector<Edge> hub_edges;
    std::vector<int> hub_sizes;
    std::vector<Triangle> kept_new;

    for (const auto& hub : hubs) {
        hub_sizes.push_back(static_cast<int>(hub.size()));

        std::map<Edge, int> edge_count;
        std::map<Edge, std::pair<int, int>> edge_order;
        double total_area = 0.0;
        double wx = 0.0, wy = 0.0;

        for (int ti : hub) {
            const Triangle& tri = triangles[ti];
            const Point2& v0 = vertices[tri[0]];
            const Point2& v1 = vertices[tri[1]];
            const Point2& v2 = vertices[tri[2]];
            double area = std::abs(signed_area2(v0, v1, v2)) / 2.0;
            total_area += area;
            wx += area * (v0.x + v1.x + v2.x) / 3.0;
            wy += area * (v0.y + v1.y + v2.y) / 3.0;

            for (const auto& [ra, rb] : tri_raw_edges(tri)) {
                Edge key = make_edge(ra, rb);
                edge_count[key]++;
                if (!edge_order.count(key)) edge_order[key] = {ra, rb};
            }
        }

        Point2 centroid;
        if (total_area > 0) {
            centroid = {wx / total_area, wy / total_area};
        } else {
            double sx = 0, sy = 0;
            int n = 0;
            for (int ti : hub)
                for (int v : triangles[ti]) {
                    sx += vertices[v].x;
                    sy += vertices[v].y;
                    ++n;
                }
            centroid = {sx / n, sy / n};
        }
        int cidx = static_cast<int>(vertices.size());
        vertices.push_back(centroid);

        for (const auto& [key, count] : edge_count) {
            if (count != 1) continue;  // internal to the hub -- discard
            auto [a, b] = edge_order[key];
            kept_new.push_back({cidx, a, b});
            for (int v : {a, b}) {
                Edge e = make_edge(cidx, v);
                boundary_edges.insert(e);
                hub_edges.push_back(e);
            }
        }
    }

    std::vector<Triangle> kept;
    for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
        if (!hub_member.count(i)) kept.push_back(triangles[i]);
    }
    kept.insert(kept.end(), kept_new.begin(), kept_new.end());
    return {kept, boundary_edges, hub_edges, hub_sizes};
}

}  // namespace

Stage5Result run_stage5(const Mesh& mesh, double prune_threshold) {
    std::vector<Triangle> triangles;
    for (int i = 0; i < static_cast<int>(mesh.triangles.size()); ++i) {
        if (mesh.interior[i]) triangles.push_back(mesh.triangles[i]);
    }
    std::vector<Point2> vertices = mesh.vertices;
    const std::vector<bool>& is_vbs_point = mesh.is_vbs_point;
    std::set<Edge> boundary_edges = mesh.constrained;

    // Correction 0: sliver-triangle edge flip -- see its own rationale above.
    // Runs before anything else classifies these triangles. 0.05mm matches
    // collapse_near_duplicate_points' own "functionally coincident at print
    // resolution" epsilon (contour.hpp) -- not a new, independently-tuned
    // constant.
    auto c0 = flip_sliver_triangles(vertices, triangles, boundary_edges, /*height_epsilon=*/0.05);
    triangles = std::move(c0.triangles);

    // Correction 0c: collapse a near-duplicate dead-end tip vertex -- see its
    // own rationale above. Runs immediately after Correction 0, before
    // anything below classifies triangles into sleeve/terminal/junction.
    auto c0c = collapse_tip_vertices(vertices, triangles, boundary_edges, /*collapse_eps=*/0.15);
    triangles = std::move(c0c.triangles);
    boundary_edges = std::move(c0c.boundary_edges);

    int sleeve_count = 0, terminal_count = 0, junction_count = 0;
    for (const auto& t : triangles) {
        int c = boundary_count(t, boundary_edges);
        if (c == 1)
            ++sleeve_count;
        else if (c == 2)
            ++terminal_count;
        else if (c == 0)
            ++junction_count;
    }

    auto c1 = correct_straight_vertices(triangles, is_vbs_point, boundary_edges);
    // Correction 1b: collapse a flat dead-end wedge whose union is a valid
    // triangle but whose VBS wall points Correction 1's exact-2 rule can't
    // reach (see collapse_terminal_wedges' own rationale).
    auto c1b = collapse_terminal_wedges(vertices, c1.triangles, is_vbs_point, c1.boundary_edges);
    auto c2 = correct_false_junctions(vertices, c1b.triangles, is_vbs_point, c1b.boundary_edges);

    // Correction 2b: flip any remaining junction/terminal triangle pair --
    // see its own rationale above. Runs right after Correction 2 so a
    // junction it resolves down to a single terminal neighbor (rather than
    // Correction 2's own required two) gets caught immediately, before
    // debridge/1c/2.6 ever see it.
    auto c2b = flip_junction_terminal_pairs(vertices, c2.triangles, c2.boundary_edges);

    auto c2_5 = debridge_junction_triangles(vertices, c2b.triangles, c2.boundary_edges);

    // Correction 1c: extend Correction 1's own is_vbs_point-gated merge to a
    // 3-incident-triangle VBS vertex -- the shape debridge just above can
    // create one hop away from the junction it resolves (see its own
    // rationale above). Must run after debridge specifically: the K=3 shape
    // doesn't exist before it.
    double c1c_chain_length_limit = edge_path_length(vertices, mesh.constrained) / 10.0;
    auto c1c = correct_straight_vertices_k3(c2_5.triangles, vertices, is_vbs_point, c2.boundary_edges,
                                             c1c_chain_length_limit);

    // Correction 2.55: repair cap-corner false junctions -- the single-contour
    // analogue of debridge (see fixup_cap_corner_false_junctions' rationale),
    // so it runs in the same slot, before stub absorption and the fan.
    auto c2_55 = fixup_cap_corner_false_junctions(vertices, c1c.triangles, c1c.boundary_edges);

    // Correction 2.7: absorb any lone junction triangle whose only
    // remaining branch is a short stub (see rationale above) -- run before
    // 2.6's local retriangulation so a hub that collapses down to a clean
    // 2-wall shape here also becomes eligible for 2.6, and before
    // Correction 3's fan, which would otherwise commit this shape to a
    // genuine branch.
    auto c2_7 = absorb_short_stubs(vertices, c2_55.triangles, c1c.boundary_edges, prune_threshold);

    // Correction 2.6: whatever debridge and stub absorption couldn't
    // resolve gets one more, more general shot via local CDT
    // re-triangulation (see rationale above) before falling through to
    // Correction 3's fan.
    std::vector<Triangle> post_retri = c2_7.triangles;
    int retriangulation_count = 0;
    {
        auto [hubs] = cluster_junction_triangles(post_retri, c1c.boundary_edges);
        std::set<int> to_remove;
        std::vector<Triangle> to_add;
        for (const auto& hub : hubs) {
            LocalRetriResult attempt = retriangulate_hub_locally(vertices, hub, post_retri, c1c.boundary_edges);
            if (!attempt.ok) continue;
            // Remove the attempt's own (possibly hub-expanded) footprint, not
            // just the original junction cluster -- see retriangulate_hub_locally's
            // region-growing rationale.
            for (int ti : attempt.region) to_remove.insert(ti);
            to_add.insert(to_add.end(), attempt.triangles.begin(), attempt.triangles.end());
            ++retriangulation_count;
        }
        if (!to_remove.empty()) {
            std::vector<Triangle> kept;
            for (int i = 0; i < static_cast<int>(post_retri.size()); ++i) {
                if (!to_remove.count(i)) kept.push_back(post_retri[i]);
            }
            kept.insert(kept.end(), to_add.begin(), to_add.end());
            post_retri = std::move(kept);
        }
    }

    auto c3 = correct_junction_hubs_single_point(vertices, post_retri, c1c.boundary_edges);

    // Correction 0b: remove any fully-isolated (no live neighbor) sliver --
    // see its own rationale above. Deliberately run last, after every other
    // correction: on real capture, the triangles this catches passed through
    // every earlier correction's own eligibility gates unchanged and are
    // only recognizable as fully isolated once the whole mesh (and every
    // correction's own boundary-edge promotions) has reached its final
    // shape -- an earlier slot risks missing edges later corrections would
    // still promote to boundary status.
    auto c0b = remove_isolated_slivers(vertices, c3.triangles, c3.boundary_edges, /*height_epsilon=*/0.05);

    Stage5Result result;
    result.cap_corner_fixup_count = c2_55.fixup_count;
    result.debug_pre_c3_triangles = post_retri;
    result.debug_pre_c3_boundary_edges = c1c.boundary_edges;
    result.vertices = std::move(vertices);
    result.triangles = std::move(c0b.triangles);
    result.hub_edges = std::move(c3.hub_edges);
    result.sliver_flip_count = c0.flip_count;
    result.tip_collapse_count = c0c.collapse_count;
    result.sliver_removal_count = c0b.removal_count;
    result.merge_count = c1.merge_count;
    result.wedge_collapse_count = c1b.collapse_count;
    result.excision_count = c2.excision_count;
    result.junction_terminal_flip_count = c2b.flip_count;
    result.debridge_count = c2_5.debridge_count;
    result.quad_merge_count = c1c.quad_merge_count;
    result.stub_absorb_count = c2_7.absorbed_count;
    result.retriangulation_count = retriangulation_count;
    result.hub_sizes = std::move(c3.hub_sizes);
    result.sleeve_count = sleeve_count;
    result.terminal_count = terminal_count;
    result.junction_count = junction_count;
    result.boundary_edges = std::move(c0b.boundary_edges);
    result.contour_edges = mesh.constrained;
    return result;
}

}  // namespace vbct
