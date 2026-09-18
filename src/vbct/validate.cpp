#include "validate.hpp"

#include <algorithm>
#include <set>
#include <sstream>

namespace vbct {
namespace {

int64_t cross(const Point2i& o, const Point2i& a, const Point2i& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

int sign(int64_t v) { return (v > 0) - (v < 0); }

bool on_segment(const Point2i& p, const Point2i& q, const Point2i& r) {
    return std::min(p.x, q.x) <= r.x && r.x <= std::max(p.x, q.x) &&
           std::min(p.y, q.y) <= r.y && r.y <= std::max(p.y, q.y);
}

bool segments_intersect(const Point2i& p1, const Point2i& p2, const Point2i& p3, const Point2i& p4) {
    int d1 = sign(cross(p3, p4, p1));
    int d2 = sign(cross(p3, p4, p2));
    int d3 = sign(cross(p1, p2, p3));
    int d4 = sign(cross(p1, p2, p4));

    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        return true;
    }
    if (d1 == 0 && on_segment(p3, p4, p1)) return true;
    if (d2 == 0 && on_segment(p3, p4, p2)) return true;
    if (d3 == 0 && on_segment(p1, p2, p3)) return true;
    if (d4 == 0 && on_segment(p1, p2, p4)) return true;
    return false;
}

// True iff the two segments share a literal common endpoint -- the "exact
// coincidence" precondition (spec S:4.1).
bool exact_endpoint_touch(const Point2i& p1, const Point2i& p2, const Point2i& p3, const Point2i& p4) {
    return p1 == p3 || p1 == p4 || p2 == p3 || p2 == p4;
}

bool edges_adjacent(size_t i, size_t j, size_t n) {
    if (j == i + 1) return true;
    if (i == 0 && j == n - 1) return true;
    return false;
}

void check_simple(const Contour& c) {
    if (c.points.size() < 3) {
        throw ValidationError("contour '" + c.contour_id + "' has fewer than 3 points");
    }
    auto edges = c.edges();
    size_t n = edges.size();
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            if (edges_adjacent(i, j, n)) continue;
            if (segments_intersect(edges[i].a, edges[i].b, edges[j].a, edges[j].b)) {
                throw ValidationError("contour '" + c.contour_id + "' is self-intersecting");
            }
        }
    }
}

void check_pair(const Contour& a, const Contour& b) {
    auto edges_a = a.edges();
    auto edges_b = b.edges();
    for (const auto& ea : edges_a) {
        for (const auto& eb : edges_b) {
            if (!segments_intersect(ea.a, ea.b, eb.a, eb.b)) continue;
            if (exact_endpoint_touch(ea.a, ea.b, eb.a, eb.b)) continue;  // valid saddle touch
            std::ostringstream msg;
            msg << "contours '" << a.contour_id << "' and '" << b.contour_id
                << "' cross or touch without an exact shared vertex";
            throw ValidationError(msg.str());
        }
    }
}

}  // namespace

void validate_contours(const std::vector<Contour>& contours) {
    for (const auto& c : contours) check_simple(c);
    for (size_t i = 0; i < contours.size(); ++i) {
        for (size_t j = i + 1; j < contours.size(); ++j) {
            check_pair(contours[i], contours[j]);
        }
    }
}

}  // namespace vbct
