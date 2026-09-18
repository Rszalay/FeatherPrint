#include "stage8.hpp"

#include <map>
#include <unordered_map>
#include <unordered_set>

#include "tri_query.hpp"

namespace vbct {
namespace {

// A piece is a pair of Stage-5 vertex indices -- never an interpolated
// point, see stage8.hpp's header comment.
using Piece = std::pair<int, int>;

Point2 midpoint(const Point2& p, const Point2& q) { return {(p.x + q.x) / 2.0, (p.y + q.y) / 2.0}; }

std::string side_of(const Point2& ridge_start, const Point2& direction, const Piece& piece,
                     const std::vector<Point2>& verts) {
    Point2 m = midpoint(verts[piece.first], verts[piece.second]);
    double c = direction.x * (m.y - ridge_start.y) - direction.y * (m.x - ridge_start.x);
    return c > 0 ? "left" : "right";
}

std::vector<Piece> triangle_pieces(const Triangle& tri, const std::set<Edge>& boundary_edges,
                                    const std::pair<NodeId, NodeId>& endpoints_entry) {
    int count = boundary_count(tri, boundary_edges);
    std::vector<std::pair<int, int>> b_edges;
    for (const auto& e : tri_raw_edges(tri)) {
        if (boundary_edges.count(make_edge(e.first, e.second))) b_edges.push_back(e);
    }

    if (count == 1) {
        return {{b_edges[0].first, b_edges[0].second}};
    }

    // count == 2: terminal. tip_a is shared by both boundary edges; b, c
    // are the other two vertices.
    const auto& e1 = b_edges[0];
    const auto& e2 = b_edges[1];
    int tip_a = -1;
    for (int v : {e1.first, e1.second}) {
        if (v == e2.first || v == e2.second) tip_a = v;
    }
    int b = (e1.first != tip_a) ? e1.first : e1.second;
    int c = (e2.first != tip_a) ? e2.first : e2.second;
    Edge ab_key = make_edge(tip_a, b);
    Edge ac_key = make_edge(tip_a, c);
    Edge bc_key = make_edge(b, c);

    NodeId bc_node = NodeId::edge(bc_key.first, bc_key.second);
    NodeId cap_node = (endpoints_entry.first != bc_node) ? endpoints_entry.first : endpoints_entry.second;

    if (cap_node.kind == NodeId::VertexNode) {
        return {{b, tip_a}, {tip_a, c}};
    }
    // Dead-end midpoint cap: drop the split edge from both walls entirely
    // rather than dividing it between them -- see spec S:11.3.
    if (cap_node == NodeId::edge(ab_key.first, ab_key.second)) {
        return {{tip_a, c}};
    }
    // cap_node == ac_key
    return {{b, tip_a}};
}

// Chain pieces into one polyline by walking the vertex-index adjacency
// graph -- see stage8.hpp's header comment for why this needs no
// coordinate keys at all, unlike the Python reference's `_stitch`.
//
// `preferred_start`, when given, is the chain's own hub vertex (a
// branch/branch self-loop's shared start/end node). It matters only for
// a fully-closed piece graph (every point degree 2 -- no unambiguous
// endpoint to anchor on): without it, the arbitrary "first vertex seen"
// fallback has no notion that the hub vertex is special, and which
// vertex ends up first depends on incidental piece-construction order --
// which Stage 5's hub-processing order (S:8.5.1, not specified to be
// canonical) can and does change from one CDT implementation to another.
// Confirmed directly on `irregular_ring`: identical Stage 1-4 input,
// identical wall *content* (same 52-piece cycle through the same
// vertices, including the hub twice), but a different arbitrary start
// vertex than the Python reference happened to land on -- which broke
// Stage 9's self-loop-hub closure, since that logic checks
// `points.front()/back() == hub_point` and had no way to know the wall's
// data was otherwise completely correct. The Python reference works only
// by fluke of its own insertion order landing on the hub every time
// across the 34-case library, not by any actual guarantee in its
// `_stitch` -- preferring the known hub vertex explicitly here removes
// that fragility rather than reproducing it.
std::vector<int> stitch_indices(const std::vector<Piece>& pieces, const std::vector<Point2>& verts,
                                 std::optional<int> preferred_start = std::nullopt) {
    if (pieces.empty()) return {};

    std::unordered_map<int, std::vector<std::pair<int, int>>> adj;  // vertex -> (other vertex, piece index)
    std::vector<int> insertion_order;
    auto ensure = [&](int v) {
        if (adj.find(v) == adj.end()) {
            insertion_order.push_back(v);
            adj[v] = {};
        }
    };
    for (int idx = 0; idx < static_cast<int>(pieces.size()); ++idx) {
        auto [p, q] = pieces[idx];
        ensure(p);
        ensure(q);
        adj[p].push_back({q, idx});
        adj[q].push_back({p, idx});
    }

    // A piece graph fragmented by a local side_of misclassification (e.g. a real notch/flat
    // feature perturbing the skeleton's local tangent direction for a single step) can produce
    // more than one connected component here -- confirmed on real capture (a ring's inner-bore
    // notch, FPTF-45 - Body (3).stl): 187 correctly-classified "left" pieces split into one
    // 186-vertex component (the real, intact wall) and one isolated 2-vertex stray edge, and the
    // walk below -- having no notion the graph could be disconnected -- locked onto whichever
    // component's own vertex happened to appear first in insertion order (the 2-vertex stray, by
    // coincidence), silently discarding the real wall's 186 vertices. Keep only the largest
    // component before doing anything else; a fragment disconnected from the main wall is never a
    // meaningful contribution to it, so dropping it is strictly correct, not a data loss. A no-op
    // for the overwhelming common case (the graph is already a single component).
    {
        std::unordered_set<int> visited;
        std::vector<int> best_component;
        for (int v : insertion_order) {
            if (visited.count(v)) continue;
            std::vector<int> component;
            std::vector<int> stack{v};
            visited.insert(v);
            while (!stack.empty()) {
                int cur = stack.back();
                stack.pop_back();
                component.push_back(cur);
                for (const auto& [nb, idx] : adj[cur]) {
                    (void)idx;
                    if (!visited.count(nb)) {
                        visited.insert(nb);
                        stack.push_back(nb);
                    }
                }
            }
            if (component.size() > best_component.size()) best_component = std::move(component);
        }
        if (best_component.size() != insertion_order.size()) {
            std::unordered_set<int> keep(best_component.begin(), best_component.end());
            std::vector<Piece> filtered_pieces;
            for (const auto& [p, q] : pieces) {
                if (keep.count(p) && keep.count(q)) filtered_pieces.push_back({p, q});
            }
            return stitch_indices(filtered_pieces, verts, preferred_start);
        }
    }

    // A true endpoint (degree 1) for an open wall always wins -- it's the
    // wall's own unambiguous terminus, not an arbitrary choice. Failing
    // that (a fully-closed cycle), prefer the chain's own hub vertex when
    // it's actually part of this piece graph.
    //
    // A fully-closed chain with no hub at all (chain.closed straight out
    // of Stage 7 -- increasingly the common case for a clean ring now
    // that Stage 5's Correction 2.5 eliminates spurious hubs entirely)
    // has neither. Falling back to "whichever vertex was seen first" is
    // the same arbitrary-insertion-order fragility already fixed for the
    // hub case (see this function's own header comment and
    // VBCT-Wall-Start-Point-Instability-Brief.md): a wall's points[0]
    // deciding a downstream reference-angle search's stability can't
    // depend on incidental piece-construction order. Use the same
    // deterministic geometric rule as Stage 9's necklace-merge splice:
    // the vertex with lexicographically smallest (x, y), tie-broken by
    // vertex index for full determinism.
    int start = insertion_order[0];
    bool found_degree_one = false;
    for (int v : insertion_order) {
        if (adj[v].size() == 1) {
            start = v;
            found_degree_one = true;
            break;
        }
    }
    if (!found_degree_one) {
        if (preferred_start.has_value() && adj.count(*preferred_start)) {
            start = *preferred_start;
        } else {
            start = insertion_order[0];
            for (int v : insertion_order) {
                const Point2& best = verts[start];
                const Point2& cand = verts[v];
                bool better = (cand.x < best.x) || (cand.x == best.x && cand.y < best.y) ||
                              (cand.x == best.x && cand.y == best.y && v < start);
                if (better) start = v;
            }
        }
    }

    std::unordered_set<int> used;
    std::vector<int> result{start};
    int cur = start;
    while (true) {
        int chosen_pt = -1, chosen_idx = -1;
        for (const auto& [pt, idx] : adj[cur]) {
            if (!used.count(idx)) {
                chosen_pt = pt;
                chosen_idx = idx;
                break;
            }
        }
        if (chosen_idx == -1) break;
        used.insert(chosen_idx);
        result.push_back(chosen_pt);
        cur = chosen_pt;
    }
    return result;
}

// Drop pairs of pieces that are the exact same edge (endpoints equal,
// order-independent) -- see spec S:11.6's self-loop-chain hazard.
std::vector<Piece> dedupe_pieces(const std::vector<Piece>& pieces) {
    auto key = [](const Piece& p) { return make_edge(p.first, p.second); };
    std::map<Edge, int> counts;
    for (const auto& p : pieces) counts[key(p)]++;
    std::map<Edge, int> emitted;
    std::vector<Piece> result;
    for (const auto& p : pieces) {
        Edge k = key(p);
        int keep = counts[k] % 2;  // odd count -> keep exactly one; even -> drop all copies
        if (emitted[k] < keep) {
            result.push_back(p);
            ++emitted[k];
        }
    }
    return result;
}

Wall materialize_wall(const std::vector<Piece>& pieces, const std::vector<Point2>& verts,
                       std::optional<int> preferred_start = std::nullopt) {
    auto deduped = dedupe_pieces(pieces);
    auto idx_seq = stitch_indices(deduped, verts, preferred_start);
    Wall w;
    w.points.reserve(idx_seq.size());
    for (int idx : idx_seq) w.points.push_back(verts[idx]);
    return w;
}

// The chain's own hub vertex, when start/end is a real branch node --
// see stitch_indices' comment for why this is threaded all the way
// through from here.
std::optional<int> hub_vertex_hint(const Chain& chain) {
    if (chain.start_node.has_value() && chain.start_node->kind == NodeId::VertexNode) return chain.start_node->u;
    if (chain.end_node.has_value() && chain.end_node->kind == NodeId::VertexNode) return chain.end_node->u;
    return std::nullopt;
}

Domain walk_chain_domain(const Chain& chain, const Stage5Result& stage5, const Stage6Result& stage6) {
    const auto& verts = stage5.vertices;
    const auto& boundary_edges = stage5.boundary_edges;
    std::vector<Piece> left_pieces, right_pieces;

    for (size_t k = 0; k < chain.triangles.size(); ++k) {
        int tri_idx = chain.triangles[k];
        const Triangle& tri = stage5.triangles[tri_idx];
        const Point2& ridge_start = chain.points[k];
        Point2 direction = {chain.points[k + 1].x - ridge_start.x, chain.points[k + 1].y - ridge_start.y};
        for (const auto& piece : triangle_pieces(tri, boundary_edges, stage6.endpoints.at(tri_idx))) {
            std::string side = side_of(ridge_start, direction, piece, verts);
            (side == "left" ? left_pieces : right_pieces).push_back(piece);
        }
    }

    std::optional<int> hub_hint = hub_vertex_hint(chain);
    Wall left = materialize_wall(left_pieces, verts, hub_hint);
    Wall right = materialize_wall(right_pieces, verts, hub_hint);

    Domain d;
    d.left = std::move(left);
    d.right = std::move(right);
    if (chain.closed) {
        d.kind = "ring";
    } else {
        d.kind = "chain";
        d.cap_start = chain.points.front();
        d.cap_end = chain.points.back();
    }
    return d;
}

}  // namespace

Stage8Result run_stage8(const Stage5Result& stage5, const Stage6Result& stage6, const Stage7Result& stage7) {
    std::vector<Domain> domains;

    for (const auto& chain : stage7.chains) {
        if (chain.incomplete) {
            Domain d;
            d.kind = "glob";
            d.reason = "incomplete_chain";
            d.triangles = chain.triangles;
            domains.push_back(std::move(d));
            continue;
        }
        domains.push_back(walk_chain_domain(chain, stage5, stage6));
    }

    if (!stage6.skipped.empty()) {
        Domain d;
        d.kind = "glob";
        d.reason = "skipped_triangles";
        d.triangles = stage6.skipped;
        domains.push_back(std::move(d));
    }

    return {domains};
}

}  // namespace vbct
