#include "harmonic.hpp"

#include <CDT.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace vbct {
namespace {

double wrap01(double x) {
    x = std::fmod(x, 1.0);
    return x < 0.0 ? x + 1.0 : x;
}

double pdist(const Point2& a, const Point2& b) { return std::hypot(a.x - b.x, a.y - b.y); }

// Point at fraction t (0..1) of the way along polyline `w` of total length `len`.
Point2 point_at(const std::vector<Point2>& w, double len, double t) {
    const double target = std::clamp(t, 0.0, 1.0) * len;
    double acc = 0.0;
    for (size_t k = 0; k + 1 < w.size(); ++k) {
        const double seg = pdist(w[k], w[k + 1]);
        if (acc + seg >= target && seg > 0.0) {
            const double f = (target - acc) / seg;
            return { w[k].x + f * (w[k + 1].x - w[k].x), w[k].y + f * (w[k + 1].y - w[k].y) };
        }
        acc += seg;
    }
    return w.back();
}

double shoelace(const std::vector<Point2>& p) {
    double a = 0.0;
    for (size_t i = 0; i < p.size(); ++i) {
        const Point2& u = p[i];
        const Point2& v = p[(i + 1) % p.size()];
        a += u.x * v.y - v.x * u.y;
    }
    return a / 2.0;
}

// Cumulative-flux curve along one wall: fractions `xs` (non-decreasing, unwrapped) with normalized cumulative flux
// `cs` (0..1, non-decreasing). inverse(c) gives the fraction reached at cumulative flux c.
struct FluxCurve {
    std::vector<double> xs;
    std::vector<double> cs;
    double inverse(double c) const {
        c = std::clamp(c, 0.0, 1.0);
        const size_t hi = std::clamp<size_t>(static_cast<size_t>(std::lower_bound(cs.begin(), cs.end(), c) - cs.begin()), 1, cs.size() - 1);
        const size_t lo = hi - 1;
        const double span = cs[hi] - cs[lo];
        const double f = span > 0.0 ? (c - cs[lo]) / span : 0.0;
        return xs[lo] + f * (xs[hi] - xs[lo]);
    }
};

// Node k of a wall carries flux f[k] spread evenly over its own share of the wall (half-way to each neighbour).
// Ring: nodes at start + k/n (k = 0..n-1), cumulative flux measured from `start`. Chain: nodes at k/n (k = 0..n).
std::optional<FluxCurve> flux_curve(const std::vector<double>& f, double start, bool ring) {
    double total = 0.0;
    for (double v : f) total += v;
    if (! (total > 0.0)) return std::nullopt;
    FluxCurve c;
    if (ring) {
        const double n = static_cast<double>(f.size());
        double s = 0.0;
        c.xs.push_back(start);
        c.cs.push_back(0.0);
        for (size_t k = 0; k < f.size(); ++k) {
            s += f[k];
            c.xs.push_back(start + (static_cast<double>(k) + 0.5) / n);
            c.cs.push_back((s - f[0] / 2.0) / total);
        }
        c.xs.push_back(start + 1.0);
        c.cs.push_back(1.0);
    } else {
        const double n = static_cast<double>(f.size() - 1);
        double s = 0.0;
        c.xs.push_back(0.0);
        c.cs.push_back(0.0);
        for (size_t k = 0; k + 1 < f.size(); ++k) {
            s += f[k];
            c.xs.push_back((static_cast<double>(k) + 0.5) / n);
            c.cs.push_back(s / total);
        }
        c.xs.push_back(1.0);
        c.cs.push_back(1.0);
    }
    for (size_t k = 1; k < c.cs.size(); ++k) c.cs[k] = std::max(c.cs[k], c.cs[k - 1]);
    return c;
}

}  // namespace

std::optional<HarmonicPairing> harmonic_wall_pairing(const std::vector<Point2>& left, double left_len, const std::vector<Point2>& right,
                                                     double right_len, bool ring, double left_start, int count) {
    if (left.size() < 2 || right.size() < 2 || ! (left_len > 0.0) || ! (right_len > 0.0) || count < 2) return std::nullopt;

    // Mesh size from the domain's mean width (area / mean wall length).
    double area;
    if (ring) {
        area = std::abs(std::abs(shoelace(left)) - std::abs(shoelace(right)));
    } else {
        std::vector<Point2> poly = left;
        poly.insert(poly.end(), right.rbegin(), right.rend());
        area = std::abs(shoelace(poly));
    }
    const double width = area / (0.5 * (left_len + right_len));
    if (! (width > 0.0)) return std::nullopt;
    const double h = std::clamp(width / 6.0, 0.15, 0.6);
    auto node_count = [&](double len) { return std::clamp(static_cast<int>(std::ceil(len / h)), 8, 2000); };
    const int nl = node_count(left_len);
    const int nr = node_count(right_len);

    // Boundary nodes. kind: 0 = left wall (phi 0), 1 = right wall (phi 1), 2 = free (a Chain cap, or a point where
    // the walls meet).
    std::vector<Point2> pts;
    std::vector<int> kind;
    std::vector<std::pair<int, int>> edges;
    std::vector<int> lidx, ridx;
    auto add = [&](const Point2& p, int k) {
        pts.push_back(p);
        kind.push_back(k);
        return static_cast<int>(pts.size()) - 1;
    };
    if (ring) {
        for (int k = 0; k < nl; ++k) lidx.push_back(add(point_at(left, left_len, wrap01(left_start + static_cast<double>(k) / nl)), 0));
        for (int k = 0; k < nr; ++k) ridx.push_back(add(point_at(right, right_len, static_cast<double>(k) / nr), 1));
        for (int k = 0; k < nl; ++k) edges.emplace_back(lidx[k], lidx[(k + 1) % nl]);
        for (int k = 0; k < nr; ++k) edges.emplace_back(ridx[k], ridx[(k + 1) % nr]);
    } else {
        for (int k = 0; k <= nl; ++k) lidx.push_back(add(point_at(left, left_len, static_cast<double>(k) / nl), 0));
        for (int k = 0; k <= nr; ++k) {
            const Point2 p = point_at(right, right_len, static_cast<double>(k) / nr);
            // Walls meeting at a pointed end share one free node.
            if ((k == 0 && pdist(p, pts[lidx.front()]) < 1e-6) || (k == nr && pdist(p, pts[lidx.back()]) < 1e-6)) {
                const int shared = k == 0 ? lidx.front() : lidx.back();
                kind[shared] = 2;
                ridx.push_back(shared);
            } else {
                ridx.push_back(add(p, 1));
            }
        }
        for (int k = 0; k < nl; ++k) edges.emplace_back(lidx[k], lidx[k + 1]);
        for (int k = 0; k < nr; ++k) edges.emplace_back(ridx[k], ridx[k + 1]);
        auto add_cap = [&](int a, int b) {
            if (a == b) return;
            const Point2 pa = pts[a], pb = pts[b];
            const int m = std::max(0, static_cast<int>(std::ceil(pdist(pa, pb) / h)) - 1);
            int prev = a;
            for (int i = 1; i <= m; ++i) {
                const double f = static_cast<double>(i) / (m + 1);
                const int cur = add({ pa.x + f * (pb.x - pa.x), pa.y + f * (pb.y - pa.y) }, 2);
                edges.emplace_back(prev, cur);
                prev = cur;
            }
            edges.emplace_back(prev, b);
        };
        add_cap(lidx.back(), ridx.back());
        add_cap(ridx.front(), lidx.front());
    }
    const size_t boundary_count = pts.size();

    // Interior points on a grid of spacing h, inside the boundary (even-odd per row) and clear of it by h/2.
    double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin, ymin = xmin, ymax = -xmin;
    for (const Point2& p : pts) {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }
    std::unordered_map<long long, std::vector<int>> buckets;
    auto cell = [&](double x, double y) {
        return (static_cast<long long>(std::floor((x - xmin) / h)) << 32) ^ static_cast<long long>(std::floor((y - ymin) / h) + 1e6);
    };
    for (size_t i = 0; i < boundary_count; ++i) buckets[cell(pts[i].x, pts[i].y)].push_back(static_cast<int>(i));
    auto near_boundary = [&](const Point2& p) {
        const long long cx = static_cast<long long>(std::floor((p.x - xmin) / h));
        const long long cy = static_cast<long long>(std::floor((p.y - ymin) / h) + 1e6);
        for (long long dx = -1; dx <= 1; ++dx)
            for (long long dy = -1; dy <= 1; ++dy) {
                const auto it = buckets.find(((cx + dx) << 32) ^ (cy + dy));
                if (it == buckets.end()) continue;
                for (int i : it->second)
                    if (pdist(p, pts[i]) < 0.5 * h) return true;
            }
        return false;
    };
    for (double y = ymin + h / 2.0; y < ymax; y += h) {
        std::vector<double> xs;
        for (const auto& [a, b] : edges) {
            const Point2& p = pts[a];
            const Point2& q = pts[b];
            if ((p.y > y) != (q.y > y)) xs.push_back(p.x + (y - p.y) * (q.x - p.x) / (q.y - p.y));
        }
        std::sort(xs.begin(), xs.end());
        for (size_t k = 0; k + 1 < xs.size(); k += 2) {
            for (double x = xmin + h / 2.0 + std::ceil((xs[k] - xmin - h / 2.0) / h) * h; x < xs[k + 1]; x += h) {
                const Point2 p{ x, y };
                if (! near_boundary(p)) add(p, 2);
            }
        }
    }

    // Triangulate.
    std::vector<std::array<int, 3>> tris;
    try {
        CDT::Triangulation<double> cdt;
        std::vector<CDT::V2d<double>> verts;
        verts.reserve(pts.size());
        for (const Point2& p : pts) verts.push_back({ p.x, p.y });
        std::vector<CDT::Edge> cdt_edges;
        for (const auto& [a, b] : edges) cdt_edges.emplace_back(CDT::VertInd(a), CDT::VertInd(b));
        cdt.insertVertices(verts);
        cdt.insertEdges(cdt_edges);
        cdt.eraseOuterTrianglesAndHoles();
        if (cdt.vertices.size() != pts.size() || cdt.triangles.empty()) return std::nullopt;
        for (const auto& t : cdt.triangles) tris.push_back({ static_cast<int>(t.vertices[0]), static_cast<int>(t.vertices[1]), static_cast<int>(t.vertices[2]) });
    } catch (...) {
        return std::nullopt;
    }

    // Cotangent stiffness matrix, rows as sorted (column, value) lists.
    const int n = static_cast<int>(pts.size());
    std::vector<std::vector<std::pair<int, double>>> rows(n);
    auto addk = [&](int i, int j, double v) {
        for (auto& e : rows[i])
            if (e.first == j) {
                e.second += v;
                return;
            }
        rows[i].emplace_back(j, v);
    };
    for (const auto& t : tris) {
        for (int c = 0; c < 3; ++c) {
            const int a = t[(c + 1) % 3], b = t[(c + 2) % 3], o = t[c];
            const double ux = pts[a].x - pts[o].x, uy = pts[a].y - pts[o].y;
            const double vx = pts[b].x - pts[o].x, vy = pts[b].y - pts[o].y;
            const double cross = std::abs(ux * vy - uy * vx);
            if (cross < 1e-14) continue;
            const double w = 0.5 * (ux * vx + uy * vy) / cross;
            addk(a, b, -w);
            addk(b, a, -w);
            addk(a, a, w);
            addk(b, b, w);
        }
    }

    // Solve for phi on the free nodes: K_ff phi_f = -K_fb phi_b (conjugate gradient, Jacobi preconditioner).
    std::vector<double> phi(n, 0.0);
    std::vector<char> fixed(n, 0);
    for (int i = 0; i < n; ++i) {
        if (kind[i] == 0 || kind[i] == 1) {
            fixed[i] = 1;
            phi[i] = kind[i] == 1 ? 1.0 : 0.0;
        }
    }
    std::vector<double> diag(n, 0.0), b(n, 0.0);
    for (int i = 0; i < n; ++i) {
        if (fixed[i]) continue;
        for (const auto& [j, v] : rows[i]) {
            if (j == i) diag[i] = v;
            else if (fixed[j]) b[i] -= v * phi[j];
        }
        if (! (diag[i] > 0.0)) return std::nullopt;
    }
    auto matvec = [&](const std::vector<double>& x, std::vector<double>& y) {
        for (int i = 0; i < n; ++i) {
            if (fixed[i]) {
                y[i] = 0.0;
                continue;
            }
            double s = 0.0;
            for (const auto& [j, v] : rows[i])
                if (! fixed[j]) s += v * x[j];
            y[i] = s;
        }
    };
    std::vector<double> x(n, 0.0), r = b, z(n), p(n), q(n);
    double bnorm = 0.0;
    for (int i = 0; i < n; ++i) bnorm += b[i] * b[i];
    bnorm = std::sqrt(bnorm);
    if (! (bnorm > 0.0)) return std::nullopt;
    for (int i = 0; i < n; ++i) z[i] = fixed[i] ? 0.0 : r[i] / diag[i];
    p = z;
    double rz = 0.0;
    for (int i = 0; i < n; ++i) rz += r[i] * z[i];
    const int max_iter = 4 * n + 200;
    bool converged = false;
    for (int it = 0; it < max_iter; ++it) {
        matvec(p, q);
        double pq = 0.0;
        for (int i = 0; i < n; ++i) pq += p[i] * q[i];
        if (! (pq > 0.0)) break;
        const double alpha = rz / pq;
        double rnorm = 0.0;
        for (int i = 0; i < n; ++i) {
            x[i] += alpha * p[i];
            r[i] -= alpha * q[i];
            rnorm += r[i] * r[i];
        }
        if (std::sqrt(rnorm) < 1e-10 * bnorm) {
            converged = true;
            break;
        }
        for (int i = 0; i < n; ++i) z[i] = fixed[i] ? 0.0 : r[i] / diag[i];
        double rz_new = 0.0;
        for (int i = 0; i < n; ++i) rz_new += r[i] * z[i];
        const double beta = rz_new / rz;
        rz = rz_new;
        for (int i = 0; i < n; ++i) p[i] = z[i] + beta * p[i];
    }
    if (! converged) return std::nullopt;
    for (int i = 0; i < n; ++i)
        if (! fixed[i]) phi[i] = x[i];

    // Flux through each wall node: the residual K phi at a fixed node.
    auto flux = [&](int i, bool is_left) {
        if (kind[i] != (is_left ? 0 : 1)) return 0.0;
        double s = 0.0;
        for (const auto& [j, v] : rows[i]) s += v * phi[j];
        return std::max(0.0, is_left ? -s : s);
    };
    std::vector<double> fl, fr;
    const int l_nodes = ring ? nl : nl + 1;
    const int r_nodes = ring ? nr : nr + 1;
    for (int k = 0; k < l_nodes; ++k) fl.push_back(flux(lidx[k], true));
    for (int k = 0; k < r_nodes; ++k) fr.push_back(flux(ridx[k], false));
    const auto cl = flux_curve(fl, ring ? left_start : 0.0, ring);
    const auto cr = flux_curve(fr, 0.0, ring);
    if (! cl || ! cr) return std::nullopt;

    HarmonicPairing out;
    if (! ring) {
        for (int g = 0; g <= count; ++g) {
            const double c = static_cast<double>(g) / count;
            out.tl.push_back(cl->inverse(c));
            out.tr.push_back(cr->inverse(c));
        }
        return out;
    }

    // Ring: the flux match fixes the pairing up to a rotation of the right wall; use the rotation with the shortest
    // total rung length. `count` samples per flux cycle, rotations in steps of one sample.
    std::vector<Point2> lp(count), rp(count);
    for (int g = 0; g < count; ++g) {
        const double c = static_cast<double>(g) / count;
        lp[g] = point_at(left, left_len, wrap01(cl->inverse(c)));
        rp[g] = point_at(right, right_len, wrap01(cr->inverse(c)));
    }
    int best_d = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (int d = 0; d < count; ++d) {
        double cost = 0.0;
        for (int g = 0; g < count && cost < best_cost; ++g) cost += pdist(lp[g], rp[(g + d) % count]);
        if (cost < best_cost) {
            best_cost = cost;
            best_d = d;
        }
    }
    auto right_unwrapped = [&](double c) {
        const double whole = std::floor(c);
        return whole + cr->inverse(c - whole);
    };
    const double shift = static_cast<double>(best_d) / count;
    for (int g = 0; g <= count; ++g) {
        const double c = static_cast<double>(g) / count;
        out.tl.push_back(cl->inverse(c));
        out.tr.push_back(right_unwrapped(c + shift));
    }
    return out;
}

}  // namespace vbct
