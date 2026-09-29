#include "stage8.hpp"

#include <cmath>
#include <optional>
#include <string>
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

// Connected components of a piece list, as vertex -> dense component id; `count` is the number of
// components. Only membership matters to callers, never the id numbering.
std::unordered_map<int, int> piece_components(const std::vector<Piece>& pieces, int& count) {
    std::unordered_map<int, std::vector<int>> adj;
    for (const auto& [p, q] : pieces) {
        adj[p].push_back(q);
        adj[q].push_back(p);
    }
    std::unordered_map<int, int> comp;
    count = 0;
    for (const auto& kv : adj) {
        if (comp.count(kv.first)) continue;
        std::vector<int> stack{kv.first};
        comp[kv.first] = count;
        while (!stack.empty()) {
            int cur = stack.back();
            stack.pop_back();
            for (int nb : adj[cur]) {
                if (!comp.count(nb)) {
                    comp[nb] = count;
                    stack.push_back(nb);
                }
            }
        }
        ++count;
    }
    return comp;
}

// side_of classifies each piece by which side of the skeleton's *local* direction it falls on, so
// at a small notch/slot in a wall the direction perturbs for a step and the few pieces around the
// notch can land on the opposite wall's list. That splits this wall's piece graph in two, and
// stitch_indices then keeps only the larger half - silently truncating the wall (confirmed on
// FPTF-45 - Body.stl stood up, layers 477-555: a 134mm inner bore wall built as ~64mm, so every
// stringer's inner end bunched onto half the bore and the stringers fanned across the open
// middle). Reattach such pieces: when `side` is split into several components, any *stray*
// component of `other` (not its largest, i.e. not the real wall) that touches two or more of
// them is exactly the missing bridge, so move it across. Everything moved is otherwise discarded
// by stitch_indices as a non-largest component of `other`, so a layer where nothing bridges is
// completely unaffected.
void reattach_bridging_strays(std::vector<Piece>& side, std::vector<Piece>& other) {
    for (int guard = 0; guard < 8; ++guard) {
        int n_side = 0;
        const auto side_comp = piece_components(side, n_side);
        if (n_side < 2) return;
        int n_other = 0;
        const auto other_comp = piece_components(other, n_other);
        if (n_other < 2) return;  // `other` is a single component: it is the real wall, nothing stray

        std::vector<int> other_size(n_other, 0);
        for (const auto& p : other) ++other_size[other_comp.at(p.first)];
        int main_other = 0;
        for (int c = 1; c < n_other; ++c) {
            if (other_size[c] > other_size[main_other]) main_other = c;
        }

        std::vector<std::set<int>> touched(n_other);
        for (const auto& [v, c] : other_comp) {
            auto it = side_comp.find(v);
            if (it != side_comp.end()) touched[c].insert(it->second);
        }
        std::vector<char> move(n_other, 0);
        bool any = false;
        for (int c = 0; c < n_other; ++c) {
            // A stray is a tiny fragment (a notch's few pieces), never a substantial piece of a
            // wall in its own right - without this cap, when both walls are fragmented the
            // smaller half of one wall would count as a "stray" bridging the other's stubs.
            const bool is_stray = other_size[c] * 10 <= other_size[main_other];
            if (c != main_other && is_stray && touched[c].size() >= 2) {
                move[c] = 1;
                any = true;
            }
        }
        if (!any) return;

        std::vector<Piece> kept;
        for (const auto& p : other) {
            if (move[other_comp.at(p.first)]) {
                side.push_back(p);
            } else {
                kept.push_back(p);
            }
        }
        other = std::move(kept);
    }
}

// ---- Dead-end face normalization ---------------------------------------------------------
//
// A Chain that dead-ends at a square end face (e.g. a slot cut across a tube wall) has three
// equally valid skeleton endings - the face's outer corner, its inner corner, or a point on the
// face - and Stage 6 picks between them by scoring near-tied candidates against a noisy local
// sleeve direction, while CDT decides which corner even becomes the terminal tip. The pick
// flips from layer to layer, and with it which wall carries the face: ending at a corner makes
// the other wall wrap all the way across the face to meet it (confirmed on FPTF-45 Body: an
// inner wall jumping 118 <-> 131mm and the cap jumping ~4mm between adjacent layers). Every
// consumer downstream (stringer distribution, end tracking, Wall A/B) sees that as noise.
//
// Normalize after the walls are built, so the result no longer depends on which ending was
// picked: at each dead end, trim any straight trailing run of a wall that runs ACROSS the
// sleeve (the end face) back to its own corner, and put the cap at the midpoint of the two
// walls' resulting ends. Only applied when the geometry is unambiguously a flat face:
//   - one wall has a straight across-sleeve run and the other runs along the sleeve and ends
//     where the first run ends (skeleton ended at a corner), or
//   - both walls end in straight across-sleeve runs that lie on one common line (skeleton
//     ended on a subdivided face).
// A rounded end (both runs curve) or a pointed tip (both runs along the sleeve) is untouched.
namespace {

constexpr double kFaceAcrossCos = 0.5;    // |cos(segment, sleeve)| below this = runs across the sleeve
constexpr double kSideAlongCos = 0.8;     // |cos| above this = runs along the sleeve
constexpr double kFaceStraightTol = 0.05; // mm: a real face is straight to within this

double seg_cos(const Point2& a, const Point2& b, const Point2& dir) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double len = std::hypot(dx, dy);
    if (len < 1e-9) return 1.0;
    return std::abs((dx * dir.x + dy * dir.y) / len);
}

// Number of trailing points (counted from the wall end nearest `cap`) forming a straight run
// across the sleeve; 0 if none. `from_back` says which end of the point array that is.
size_t face_run(const std::vector<Point2>& w, bool from_back, const Point2& dir) {
    const size_t n = w.size();
    auto at = [&](size_t i) -> const Point2& { return from_back ? w[n - 1 - i] : w[i]; };
    size_t k = 0;
    while (k + 1 < n - 1 && seg_cos(at(k), at(k + 1), dir) < kFaceAcrossCos) ++k;
    if (k == 0) return 0;
    const Point2& e = at(0);
    const Point2& c = at(k);
    const double lx = c.x - e.x, ly = c.y - e.y, ll = std::hypot(lx, ly);
    if (ll < 1e-6) return 0;
    for (size_t i = 1; i < k; ++i) {
        const double d = std::abs((at(i).x - e.x) * ly - (at(i).y - e.y) * lx) / ll;
        if (d > kFaceStraightTol) return 0;  // curved: a rounded end, not a face
    }
    return k;
}

// Unit direction of a wall between arc lengths `a` and `b` back from its end nearest `cap`,
// pointing toward that end. Far enough back to be past any end face, so it runs along the
// sleeve regardless of which corner the skeleton ended at.
std::optional<Point2> wall_tangent(const std::vector<Point2>& w, bool from_back, double a, double b) {
    const size_t n = w.size();
    auto at = [&](size_t i) -> const Point2& { return from_back ? w[n - 1 - i] : w[i]; };
    std::optional<Point2> pa, pb;
    double acc = 0.0;
    for (size_t k = 1; k < n; ++k) {
        acc += std::hypot(at(k).x - at(k - 1).x, at(k).y - at(k - 1).y);
        if (! pa && acc >= a) pa = at(k);
        if (acc >= b) { pb = at(k); break; }
    }
    if (! pa || ! pb) return std::nullopt;
    const double dx = pa->x - pb->x, dy = pa->y - pb->y, len = std::hypot(dx, dy);
    if (len < 1e-6) return std::nullopt;
    return Point2{ dx / len, dy / len };
}

bool wall_end_is_back(const std::vector<Point2>& w, const Point2& cap) {
    return std::hypot(w.back().x - cap.x, w.back().y - cap.y) < std::hypot(w.front().x - cap.x, w.front().y - cap.y);
}

void trim(std::vector<Point2>& w, bool from_back, size_t k) {
    if (from_back) w.resize(w.size() - k);
    else w.erase(w.begin(), w.begin() + static_cast<std::ptrdiff_t>(k));
}

// The skeleton can also end a little way BEFORE a face corner, on one side wall: that wall then
// stops short, and the other wall runs the remaining stretch along the side to the corner and then
// down the face. Recognized as: both walls end at the same point, one wall's trailing run is a
// short along-sleeve piece collinear with the other wall's own direction, immediately followed by
// a straight across-sleeve face. The along piece is handed to the other wall (which then ends at
// the corner) and the face is trimmed, giving the same result as a clean corner ending.
std::optional<Point2> normalize_short_side_overrun(std::vector<Point2>& w, bool wb, std::vector<Point2>& o, bool ob, const Point2& dir) {
    constexpr double kMaxOverrun = 3.0;  // mm
    const size_t n = w.size();
    auto at = [&](size_t i) -> const Point2& { return wb ? w[n - 1 - i] : w[i]; };
    const Point2& oe = ob ? o.back() : o.front();
    const Point2& oprev = ob ? o[o.size() - 2] : o[1];
    if (std::hypot(at(0).x - oe.x, at(0).y - oe.y) > kFaceStraightTol) return std::nullopt;
    const double ox = oe.x - oprev.x, oy = oe.y - oprev.y, ol = std::hypot(ox, oy);
    if (ol < 1e-9) return std::nullopt;
    size_t a = 0;
    double run = 0.0;
    while (a + 1 < n - 1) {
        const Point2& p0 = at(a);
        const Point2& p1 = at(a + 1);
        const double sx = p1.x - p0.x, sy = p1.y - p0.y, sl = std::hypot(sx, sy);
        if (sl < 1e-9) break;
        if (seg_cos(p0, p1, dir) <= kSideAlongCos) break;
        if ((sx * ox + sy * oy) / (sl * ol) < 0.9) break;  // must continue the other wall's own direction
        run += sl;
        if (run > kMaxOverrun) return std::nullopt;
        ++a;
    }
    if (a == 0) return std::nullopt;
    size_t k = a;
    while (k + 1 < n - 1 && seg_cos(at(k), at(k + 1), dir) < kFaceAcrossCos) ++k;
    if (k == a) return std::nullopt;
    const Point2 corner = at(a);
    const Point2 inner = at(k);
    const double lx = inner.x - corner.x, ly = inner.y - corner.y, ll = std::hypot(lx, ly);
    if (ll < 1e-6) return std::nullopt;
    for (size_t i = a + 1; i < k; ++i) {
        if (std::abs((at(i).x - corner.x) * ly - (at(i).y - corner.y) * lx) / ll > kFaceStraightTol) return std::nullopt;
    }
    std::vector<Point2> handed;
    for (size_t i = 1; i <= a; ++i) handed.push_back(at(i));
    if (ob) o.insert(o.end(), handed.begin(), handed.end());
    else o.insert(o.begin(), handed.rbegin(), handed.rend());
    trim(w, wb, k);
    return Point2{ (corner.x + inner.x) / 2.0, (corner.y + inner.y) / 2.0 };
}

// Mirror of the overrun case: one wall ends at a face corner and runs down the face, while the other
// wall stops a little SHORT of that corner along its side (the stretch between belongs to neither
// wall). The short wall is extended to the corner and the face trimmed.
std::optional<Point2> normalize_short_side_gap(std::vector<Point2>& w, bool wb, std::vector<Point2>& o, bool ob, const Point2& dir) {
    constexpr double kMaxGap = 3.0;  // mm
    const size_t k = face_run(w, wb, dir);
    if (k == 0) return std::nullopt;
    const size_t n = w.size();
    const Point2 corner = wb ? w[n - 1] : w[0];
    const Point2 inner = wb ? w[n - 1 - k] : w[k];
    const Point2& oe = ob ? o.back() : o.front();
    const Point2& oprev = ob ? o[o.size() - 2] : o[1];
    const double gx = corner.x - oe.x, gy = corner.y - oe.y, gl = std::hypot(gx, gy);
    if (gl < kFaceStraightTol || gl > kMaxGap) return std::nullopt;
    const double ox = oe.x - oprev.x, oy = oe.y - oprev.y, ol = std::hypot(ox, oy);
    if (ol < 1e-9 || (gx * ox + gy * oy) / (gl * ol) < 0.9) return std::nullopt;  // gap continues the other wall
    if (std::abs((gx * dir.x + gy * dir.y) / gl) <= kSideAlongCos) return std::nullopt;  // and runs along the sleeve
    if (ob) o.push_back(corner);
    else o.insert(o.begin(), corner);
    trim(w, wb, k);
    return Point2{ (corner.x + inner.x) / 2.0, (corner.y + inner.y) / 2.0 };
}

// Returns the new cap point when the end was normalized.
std::optional<Point2> normalize_dead_end(std::vector<Point2>& left, std::vector<Point2>& right, const Point2& cap, const Point2& dir) {
    if (left.size() < 4 || right.size() < 4) return std::nullopt;
    const bool lb = wall_end_is_back(left, cap), rb = wall_end_is_back(right, cap);
    const Point2 le = lb ? left.back() : left.front();
    const Point2 re = rb ? right.back() : right.front();
    if (const auto c = normalize_short_side_overrun(right, rb, left, lb, dir)) return c;
    if (const auto c = normalize_short_side_overrun(left, lb, right, rb, dir)) return c;
    if (const auto c = normalize_short_side_gap(right, rb, left, lb, dir)) return c;
    if (const auto c = normalize_short_side_gap(left, lb, right, rb, dir)) return c;
    const size_t lk = face_run(left, lb, dir);
    const size_t rk = face_run(right, rb, dir);
    auto last_cos = [&](const std::vector<Point2>& w, bool b) {
        return b ? seg_cos(w[w.size() - 2], w.back(), dir) : seg_cos(w[1], w.front(), dir);
    };
    // A run is the end face only if the other wall's end lies on the same straight line: either
    // both walls meet at one face corner (Stage 6 ended the skeleton at that corner), or the other
    // wall ends partway along the face (Stage 6 ended it on a face sub-edge).
    auto on_line = [](const Point2& a, const Point2& b, const Point2& p) {
        const double lx = b.x - a.x, ly = b.y - a.y, ll = std::hypot(lx, ly);
        if (ll < 1e-6) return false;
        return std::abs((p.x - a.x) * ly - (p.y - a.y) * lx) / ll < kFaceStraightTol;
    };
    const Point2 lc = lk > 0 ? (lb ? left[left.size() - 1 - lk] : left[lk]) : le;
    const Point2 rc = rk > 0 ? (rb ? right[right.size() - 1 - rk] : right[rk]) : re;
    // Test the wall's end (where the skeleton split the face) against the whole face - from that wall's face corner to
    // the other wall's end - rather than extending the wall's own short face run out to the other wall: a run well
    // under a millimetre long, extrapolated several millimetres, turns a 0.01mm kink into a miss of the tolerance
    // (FPTF-45 Body top slot end, layers 148-186 and alternate layers 241-263).
    const bool l_face = lk > 0 && on_line(lc, re, le);
    const bool r_face = rk > 0 && on_line(rc, le, re);
    if (lk > 0 && rk > 0) {
        if (! (l_face && r_face)) return std::nullopt;
        trim(left, lb, lk);
        trim(right, rb, rk);
    } else if (l_face && last_cos(right, rb) > kSideAlongCos) {
        trim(left, lb, lk);
    } else if (r_face && last_cos(left, lb) > kSideAlongCos) {
        trim(right, rb, rk);
    } else {
        return std::nullopt;
    }
    const Point2 nle = lb ? left.back() : left.front();
    const Point2 nre = rb ? right.back() : right.front();
    return Point2{ (nle.x + nre.x) / 2.0, (nle.y + nre.y) / 2.0 };
}

}  // namespace

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

    reattach_bridging_strays(left_pieces, right_pieces);
    reattach_bridging_strays(right_pieces, left_pieces);

    std::optional<int> hub_hint = hub_vertex_hint(chain);
    Wall left = materialize_wall(left_pieces, verts, hub_hint);
    Wall right = materialize_wall(right_pieces, verts, hub_hint);

    Domain d;
    if (chain.closed) {
        d.kind = "ring";
    } else {
        d.kind = "chain";
        d.cap_start = chain.points.front();
        d.cap_end = chain.points.back();
    }
    d.left = std::move(left);
    d.right = std::move(right);
    return d;
}

}  // namespace

void normalize_chain_dead_ends(std::vector<Domain>& domains) {
    constexpr double kJunctionTol = 0.1;  // mm: a cap shared with another domain is a junction, not a dead end
    std::vector<Point2> caps;
    for (const Domain& d : domains) {
        if (d.kind != "chain") continue;
        if (d.cap_start) caps.push_back(*d.cap_start);
        if (d.cap_end) caps.push_back(*d.cap_end);
    }
    auto shared = [&](const Point2& c) {
        int n = 0;
        for (const Point2& q : caps)
            if (std::hypot(q.x - c.x, q.y - c.y) < kJunctionTol) ++n;
        return n > 1;
    };
    for (Domain& d : domains) {
        if (d.kind != "chain" || ! d.left || ! d.right || ! d.cap_start || ! d.cap_end) continue;
        for (const bool at_start : { true, false }) {
            Point2& cap = at_start ? *d.cap_start : *d.cap_end;
            if (shared(cap)) continue;
            constexpr double kTangentFrom = 5.0, kTangentTo = 8.0;  // mm back from the wall end: past a face, local enough for a curving wall
            const auto tl = wall_tangent(d.left->points, wall_end_is_back(d.left->points, cap), kTangentFrom, kTangentTo);
            const auto tr = wall_tangent(d.right->points, wall_end_is_back(d.right->points, cap), kTangentFrom, kTangentTo);
            if (! tl || ! tr) continue;
            const double sx = tl->x + tr->x, sy = tl->y + tr->y, sl = std::hypot(sx, sy);
            if (sl < 1e-6) continue;
            const Point2 dir{ sx / sl, sy / sl };
            if (const auto moved = normalize_dead_end(d.left->points, d.right->points, cap, dir)) cap = *moved;
        }
    }
}

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
