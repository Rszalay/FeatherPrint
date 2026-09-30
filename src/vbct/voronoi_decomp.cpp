#include "voronoi_decomp.hpp"

#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace vbct {
namespace {

namespace bp = boost::polygon;
using VD = bp::voronoi_diagram<double>;
using Seg = bp::segment_data<int>;

constexpr double kCornerRatio = 3.0;       // keep an axis edge when boundary distance / radius exceeds this
constexpr double kEndTurnRad = 0.785398;   // 45 degrees: a dead-end wall runs on until the outline turns this much

struct P {
    double x, y;
};
P operator-(P a, P b) { return { a.x - b.x, a.y - b.y }; }
P operator+(P a, P b) { return { a.x + b.x, a.y + b.y }; }
P operator*(double s, P a) { return { s * a.x, s * a.y }; }
double dot(P a, P b) { return a.x * b.x + a.y * b.y; }
double cross(P a, P b) { return a.x * b.y - a.y * b.x; }
double len(P a) { return std::hypot(a.x, a.y); }

// Boundary geometry in micrometres (VBCT's fixed-point units).
struct Boundary {
    std::vector<std::vector<P>> pts;     // per contour, open (no repeated closing point)
    std::vector<std::vector<double>> cum;  // cum[c][i] = arc length to pts[c][i]; cum[c][n] = total
    std::vector<std::pair<int, int>> seg_of;  // global segment index -> (contour, i): pts[i] -> pts[i+1]

    double total(int c) const { return cum[c].back(); }
    P at(int c, double s) const {
        const double L = total(c);
        s = std::fmod(s, L);
        if (s < 0) s += L;
        const auto& cc = cum[c];
        size_t i = static_cast<size_t>(std::upper_bound(cc.begin(), cc.end(), s) - cc.begin());
        i = std::clamp<size_t>(i, 1, cc.size() - 1) - 1;
        const P a = pts[c][i], b = pts[c][(i + 1) % pts[c].size()];
        const double seg = cc[i + 1] - cc[i];
        const double f = seg > 0 ? (s - cc[i]) / seg : 0.0;
        return a + f * (b - a);
    }
    // Signed shortest cyclic difference t - s on contour c.
    double delta(int c, double s, double t) const {
        const double L = total(c);
        double d = std::fmod(t - s, L);
        if (d > L / 2) d -= L;
        if (d < -L / 2) d += L;
        return d;
    }
};

// A boundary feature a Voronoi cell belongs to: a segment, or a vertex.
struct Feature {
    int contour{ -1 };
    int seg{ -1 };     // segment index within the contour, when a segment
    int vertex{ -1 };  // vertex index within the contour, when a point
};

Feature feature_of(const VD::cell_type& cell, const Boundary& B) {
    const auto [c, i] = B.seg_of[cell.source_index()];
    Feature f;
    f.contour = c;
    if (cell.contains_segment()) {
        f.seg = i;
    } else if (cell.source_category() == bp::SOURCE_CATEGORY_SEGMENT_START_POINT) {
        f.vertex = i;
    } else {
        f.vertex = (i + 1) % static_cast<int>(B.pts[c].size());
    }
    return f;
}

// Nearest point of `f` to `q`, as (arc position, point).
std::pair<double, P> foot(const Feature& f, P q, const Boundary& B) {
    const auto& pts = B.pts[f.contour];
    if (f.vertex >= 0) return { B.cum[f.contour][f.vertex], pts[f.vertex] };
    const P a = pts[f.seg], b = pts[(f.seg + 1) % pts.size()];
    const P ab = b - a;
    const double L2 = dot(ab, ab);
    const double t = L2 > 0 ? std::clamp(dot(q - a, ab) / L2, 0.0, 1.0) : 0.0;
    return { B.cum[f.contour][f.seg] + t * std::sqrt(L2), a + t * ab };
}

bool inside(P q, const Boundary& B) {
    bool in = false;
    for (const auto& c : B.pts) {
        for (size_t i = 0, j = c.size() - 1; i < c.size(); j = i++) {
            if ((c[i].y > q.y) != (c[j].y > q.y) && q.x < c[j].x + (q.y - c[j].y) * (c[i].x - c[j].x) / (c[i].y - c[j].y)) in = ! in;
        }
    }
    return in;
}

P vpt(const VD::vertex_type* v) { return { v->x(), v->y() }; }

// Points along a Voronoi edge from v0 to v1 (a parabola when one site is a point and the other a segment).
std::vector<P> edge_points(const VD::edge_type& e, const Boundary& B) {
    const P p0 = vpt(e.vertex0()), p1 = vpt(e.vertex1());
    if (! e.is_curved()) return { p0, p1 };
    const Feature f1 = feature_of(*e.cell(), B), f2 = feature_of(*e.twin()->cell(), B);
    const Feature& fp = f1.vertex >= 0 ? f1 : f2;
    const Feature& fs = f1.vertex >= 0 ? f2 : f1;
    if (fp.vertex < 0 || fs.seg < 0) return { p0, p1 };
    const auto& pts = B.pts[fs.contour];
    const P a = pts[fs.seg], b = pts[(fs.seg + 1) % pts.size()];
    const double L = len(b - a);
    if (L <= 0) return { p0, p1 };
    const P u = (1.0 / L) * (b - a);
    P n{ -u.y, u.x };
    const P pp = B.pts[fp.contour][fp.vertex];
    double pn = dot(pp - a, n);
    if (pn < 0) {
        n = -1.0 * n;
        pn = -pn;
    }
    if (pn <= 1e-9) return { p0, p1 };
    const double ps = dot(pp - a, u);
    const double s0 = dot(p0 - a, u), s1 = dot(p1 - a, u);
    const int k = std::clamp(static_cast<int>(std::abs(s1 - s0) / 200.0), 2, 40);  // ~0.2 mm steps
    std::vector<P> out{ p0 };
    for (int i = 1; i < k; ++i) {
        const double s = s0 + (s1 - s0) * i / k;
        const double h = ((s - ps) * (s - ps) + pn * pn) / (2 * pn);
        out.push_back(a + s * u + h * n);
    }
    out.push_back(p1);
    return out;
}

double polyline_len(const std::vector<P>& p) {
    double s = 0;
    for (size_t i = 1; i < p.size(); ++i) s += len(p[i] - p[i - 1]);
    return s;
}

Point2 to_mm(P p) { return { p.x / static_cast<double>(SCALE), p.y / static_cast<double>(SCALE) }; }

struct GEdge {
    const VD::edge_type* he;  // half-edge oriented from a to b
    int a, b;
    std::vector<P> pts;
    double length;
    bool alive{ true };
};

}  // namespace

Stage9Result decompose_voronoi(const std::vector<Contour>& contours_in, double threshold_mm, VoronoiDebug* debug) {
    // Boundary.
    Boundary B;
    std::vector<Seg> segs;
    for (const Contour& c0 : contours_in) {
        const Contour c = collapse_near_duplicate_points(c0);
        if (c.points.size() < 3) continue;
        const int ci = static_cast<int>(B.pts.size());
        B.pts.emplace_back();
        B.cum.emplace_back(1, 0.0);
        for (const Point2i& p : c.points) B.pts.back().push_back({ static_cast<double>(p.x), static_cast<double>(p.y) });
        const auto& pts = B.pts.back();
        for (size_t i = 0; i < pts.size(); ++i) {
            const P a = pts[i], b = pts[(i + 1) % pts.size()];
            B.cum.back().push_back(B.cum.back().back() + len(b - a));
            segs.emplace_back(bp::point_data<int>(static_cast<int>(a.x), static_cast<int>(a.y)), bp::point_data<int>(static_cast<int>(b.x), static_cast<int>(b.y)));
            B.seg_of.emplace_back(ci, static_cast<int>(i));
        }
    }
    if (segs.empty()) throw std::runtime_error("voronoi decomposition: no contours");

    VD vd;
    bp::construct_voronoi(segs.begin(), segs.end(), &vd);

    // Interior primary edges, each once (oriented arbitrarily), with the corner-ratio test.
    std::unordered_map<const VD::vertex_type*, int> vid;
    std::vector<P> vpos;
    auto vertex_id = [&](const VD::vertex_type* v) {
        auto [it, fresh] = vid.emplace(v, static_cast<int>(vpos.size()));
        if (fresh) vpos.push_back(vpt(v));
        return it->second;
    };
    std::vector<GEdge> edges;
    std::vector<GEdge> corner_edges;
    std::set<const VD::edge_type*> seen;
    for (const auto& e : vd.edges()) {
        if (! e.is_primary() || ! e.is_finite() || seen.count(&e)) continue;
        seen.insert(&e);
        seen.insert(e.twin());
        const std::vector<P> pts = edge_points(e, B);
        const P mid = pts.size() > 2 ? pts[pts.size() / 2] : 0.5 * (pts.front() + pts.back());
        if (! inside(mid, B)) continue;
        const Feature fl = feature_of(*e.cell(), B), fr = feature_of(*e.twin()->cell(), B);
        double ratio = 1e9;
        if (fl.contour == fr.contour) {
            // Evaluate at the deeper end.
            const P v0 = pts.front(), v1 = pts.back();
            const double r0 = len(foot(fl, v0, B).second - v0), r1 = len(foot(fl, v1, B).second - v1);
            const P q = r0 >= r1 ? v0 : v1;
            const double r = std::max(r0, r1);
            const double sl = foot(fl, q, B).first, sr = foot(fr, q, B).first;
            ratio = r > 1e-6 ? std::abs(B.delta(fl.contour, sl, sr)) / r : 0.0;
        }
        const bool keep = ratio > kCornerRatio;
        if (debug) {
            VoronoiDebugEdge de;
            for (const P& p : pts) de.points.push_back(to_mm(p));
            de.status = keep ? 0 : 1;
            de.ratio = std::min(ratio, 99.0);
            debug->edges.push_back(std::move(de));
        }
        GEdge g{ &e, vertex_id(e.vertex0()), vertex_id(e.vertex1()), pts, polyline_len(pts) / SCALE };
        (keep ? edges : corner_edges).push_back(std::move(g));
    }

    // A region with no corridor at all (a solid square or disc): every edge is a corner branch and nothing survives.
    // Fall back to the longest path through the whole medial axis as one Chain - the square's diagonal, a disc's
    // diameter - so the region still gets a domain rather than no infill.
    if (edges.empty() && ! corner_edges.empty()) {
        const int n = static_cast<int>(vpos.size());
        std::vector<std::vector<int>> cadj(n);
        for (int i = 0; i < static_cast<int>(corner_edges.size()); ++i) {
            cadj[corner_edges[i].a].push_back(i);
            cadj[corner_edges[i].b].push_back(i);
        }
        // Longest path between two leaves; ties (a square's four equal half-diagonals, a disc's spokes) broken toward
        // end points far apart in a straight line, so a square gets a true diagonal and a disc a true diameter.
        std::vector<int> leaves;
        for (int v = 0; v < n; ++v)
            if (cadj[v].size() == 1) leaves.push_back(v);
        double best_score = -1.0;
        int best_a = -1, best_b = -1;
        std::vector<int> via, best_via;
        for (int la : leaves) {
            std::vector<double> dist(n, -1.0);
            via.assign(n, -1);
            dist[la] = 0.0;
            std::vector<int> stack{ la };
            while (! stack.empty()) {
                const int v = stack.back();
                stack.pop_back();
                for (int ei : cadj[v]) {
                    const int w = corner_edges[ei].a == v ? corner_edges[ei].b : corner_edges[ei].a;
                    if (dist[w] >= 0.0) continue;
                    dist[w] = dist[v] + corner_edges[ei].length;
                    via[w] = ei;
                    stack.push_back(w);
                }
            }
            for (int lb : leaves) {
                if (lb == la || dist[lb] < 0.0) continue;
                const double score = dist[lb] + 1e-3 * len(vpos[lb] - vpos[la]) / SCALE;
                if (score > best_score) {
                    best_score = score;
                    best_a = la;
                    best_b = lb;
                    best_via = via;
                }
            }
        }
        for (int v = best_b; best_a >= 0 && v != best_a && best_via[v] >= 0;) {
            const GEdge& g = corner_edges[best_via[v]];
            edges.push_back(g);
            v = g.a == v ? g.b : g.a;
        }
    }

    // Adjacency and spur pruning.
    const int nv = static_cast<int>(vpos.size());
    auto build_adj = [&]() {
        std::vector<std::vector<int>> adj(nv);
        for (int i = 0; i < static_cast<int>(edges.size()); ++i) {
            if (! edges[i].alive) continue;
            adj[edges[i].a].push_back(i);
            adj[edges[i].b].push_back(i);
        }
        return adj;
    };
    auto other = [&](int ei, int v) { return edges[ei].a == v ? edges[ei].b : edges[ei].a; };
    for (bool changed = true; changed;) {
        changed = false;
        const auto adj = build_adj();
        for (int v = 0; v < nv; ++v) {
            if (adj[v].size() != 1) continue;
            std::vector<int> path;
            double length = 0;
            int cur = v, prev_e = -1;
            while (true) {
                int next_e = -1;
                for (int ei : adj[cur])
                    if (ei != prev_e) next_e = ei;
                if (next_e < 0) break;
                path.push_back(next_e);
                length += edges[next_e].length;
                cur = other(next_e, cur);
                prev_e = next_e;
                if (adj[cur].size() != 2) break;
            }
            if (adj[cur].size() >= 3 && length < threshold_mm) {
                for (int ei : path) edges[ei].alive = false;
                changed = true;
                break;
            }
        }
    }
    if (debug) {
        for (const GEdge& g : edges) {
            if (g.alive) continue;
            VoronoiDebugEdge de;
            for (const P& p : g.pts) de.points.push_back(to_mm(p));
            de.status = 2;
            debug->edges.push_back(std::move(de));
        }
    }

    // Walk into chains (between vertices of degree != 2) and pure cycles.
    const auto adj = build_adj();
    std::vector<char> used(edges.size(), 0);
    struct Walk {
        std::vector<int> edges;  // in order
        std::vector<int> verts;  // edges.size() + 1
        bool cycle{ false };
    };
    std::vector<Walk> walks;
    auto walk_from = [&](int start, int first_e) {
        Walk w;
        w.verts.push_back(start);
        int cur = start, e = first_e;
        while (true) {
            used[e] = 1;
            w.edges.push_back(e);
            cur = other(e, cur);
            w.verts.push_back(cur);
            if (adj[cur].size() != 2 || cur == start) break;
            int next = -1;
            for (int ei : adj[cur])
                if (! used[ei]) next = ei;
            if (next < 0) break;
            e = next;
        }
        w.cycle = w.verts.back() == start && adj[start].size() == 2;
        return w;
    };
    for (int v = 0; v < nv; ++v) {
        if (adj[v].size() == 2 || adj[v].empty()) continue;
        for (int ei : adj[v])
            if (! used[ei]) walks.push_back(walk_from(v, ei));
    }
    for (int ei = 0; ei < static_cast<int>(edges.size()); ++ei)
        if (edges[ei].alive && ! used[ei]) walks.push_back(walk_from(edges[ei].a, ei));

    Stage9Result result;
    for (const Walk& w : walks) {
        // Features either side, in walk direction: for each edge, the half-edge running along the walk has its own
        // cell on the left.
        struct SideFeet {
            std::vector<std::pair<int, double>> feet;  // (contour, arc position) at each walk vertex
        } left, right;
        for (size_t k = 0; k < w.edges.size(); ++k) {
            const GEdge& g = edges[w.edges[k]];
            const bool forward = g.a == w.verts[k];
            const VD::edge_type* he = forward ? g.he : g.he->twin();
            const Feature fl = feature_of(*he->cell(), B), fr = feature_of(*he->twin()->cell(), B);
            for (int end = (k == 0 ? 0 : 1); end < 2; ++end) {
                const P q = vpos[w.verts[k + end]];
                left.feet.emplace_back(fl.contour, foot(fl, q, B).first);
                right.feet.emplace_back(fr.contour, foot(fr, q, B).first);
            }
        }
        auto majority = [](const SideFeet& s) {
            std::map<int, int> n;
            for (const auto& f : s.feet) ++n[f.first];
            return std::max_element(n.begin(), n.end(), [](auto& a, auto& b) { return a.second < b.second; })->first;
        };
        const int cl = majority(left), cr = majority(right);

        Domain d;
        if (w.cycle) {
            if (cl == cr) continue;
            auto loop = [&](int c) {
                Wall wall;
                for (const P& p : B.pts[c]) wall.points.push_back(to_mm(p));
                wall.points.push_back(wall.points.front());
                return wall;
            };
            d.kind = "ring";
            d.left = loop(cl);
            d.right = loop(cr);
            result.domains.push_back(std::move(d));
            continue;
        }

        // Chain. Each wall spans its feet from the walk's start to its end along its contour.
        struct Span {
            int c;
            double s0, s1;  // start/end arc position; s1 - s0 signed (direction of travel), unwrapped
        };
        auto span_of = [&](const SideFeet& s, int c) -> std::optional<Span> {
            std::vector<double> ps;
            for (const auto& f : s.feet)
                if (f.first == c) ps.push_back(f.second);
            if (ps.size() < 2) return std::nullopt;
            double acc = 0;
            for (size_t i = 1; i < ps.size(); ++i) acc += B.delta(c, ps[i - 1], ps[i]);
            return Span{ c, ps.front(), ps.front() + acc };
        };
        auto sl = span_of(left, cl), sr = span_of(right, cr);
        if (! sl || ! sr) continue;

        const int v_start = w.verts.front(), v_end = w.verts.back();
        Point2 cap_start = to_mm(vpos[v_start]), cap_end = to_mm(vpos[v_end]);

        // Dead-end ending: extend both walls along the uncovered outline toward each other, each stopping at its
        // corner, and put the cap at the midpoint of the uncovered stretch.
        auto end_extension = [&](Span& a, Span& b, bool at_start, Point2& cap) {
            if (a.c != b.c) return;
            const int c = a.c;
            const double L = B.total(c);
            // Wall a's end at this dead end, and the direction pointing away from wall a (into the uncovered part).
            const double a_end = at_start ? a.s0 : a.s1;
            const double a_dir = (a.s1 >= a.s0 ? 1.0 : -1.0) * (at_start ? -1.0 : 1.0);
            const double b_end = at_start ? b.s0 : b.s1;
            double gap = std::fmod(a_dir * (b_end - a_end), L);
            if (gap < 0) gap += L;
            if (gap <= 0 || gap > L / 2) return;
            auto run_to_corner = [&](double from, double dir, double limit) {
                const double step = std::min(100.0, limit / 4 + 1e-9);  // 0.1 mm
                const P t0 = B.at(c, from + dir * step) - B.at(c, from);
                const double l0 = len(t0);
                double s = 0;
                while (s + step <= limit) {
                    const P t = B.at(c, from + dir * (s + step)) - B.at(c, from + dir * s);
                    const double lt = len(t);
                    if (l0 > 0 && lt > 0 && std::acos(std::clamp(dot(t0, t) / (l0 * lt), -1.0, 1.0)) > kEndTurnRad) break;
                    s += step;
                }
                return s;
            };
            const double ea = run_to_corner(a_end, a_dir, gap / 2);
            const double eb = run_to_corner(b_end, -a_dir, gap / 2);
            if (at_start) {
                a.s0 = a_end + a_dir * ea;
                b.s0 = b_end - a_dir * eb;
            } else {
                a.s1 = a_end + a_dir * ea;
                b.s1 = b_end - a_dir * eb;
            }
            const double mid = a_end + a_dir * (ea + (gap - ea - eb) / 2);
            cap = to_mm(B.at(c, mid));
        };
        if (adj[v_start].size() == 1) end_extension(*sl, *sr, true, cap_start);
        if (adj[v_end].size() == 1) end_extension(*sl, *sr, false, cap_end);

        auto wall_points = [&](const Span& s) {
            Wall wall;
            const auto& cc = B.cum[s.c];
            const double L = B.total(s.c);
            const double dir = s.s1 >= s.s0 ? 1.0 : -1.0;
            const double span = std::abs(s.s1 - s.s0);
            wall.points.push_back(to_mm(B.at(s.c, s.s0)));
            // Contour vertices strictly inside the span, in travel order.
            std::vector<std::pair<double, P>> inner;
            for (size_t i = 0; i < B.pts[s.c].size(); ++i) {
                double off = std::fmod(dir * (cc[i] - s.s0), L);
                if (off < 0) off += L;
                if (off > 1e-6 && off < span - 1e-6) inner.emplace_back(off, B.pts[s.c][i]);
            }
            std::sort(inner.begin(), inner.end(), [](auto& x, auto& y) { return x.first < y.first; });
            for (const auto& [o, p] : inner) wall.points.push_back(to_mm(p));
            wall.points.push_back(to_mm(B.at(s.c, s.s1)));
            return wall;
        };
        d.kind = "chain";
        d.left = wall_points(*sl);
        d.right = wall_points(*sr);
        d.cap_start = cap_start;
        d.cap_end = cap_end;
        if (d.left->points.size() < 2 || d.right->points.size() < 2) continue;
        result.domains.push_back(std::move(d));
    }
    return result;
}

}  // namespace vbct
