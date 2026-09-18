// Basic geometry types shared across all stages.
//
// Point2i (fixed-point integer, SCALE steps per input unit) is used through
// Stages 0-2, matching spec REV 2.1 S:2.3: VBS repeatedly splits a straight
// segment into collinear pieces, and integer arithmetic there avoids the
// near-collinearity drift that plain float splitting would introduce.
// Point2 (double) is used from Stage 3 onward, converted from Point2i by a
// plain per-component cast (`Contour::to_float`), never through arithmetic.
#pragma once

#include <cmath>
#include <cstdint>
#include <utility>

namespace vbct {

constexpr int64_t SCALE = 1000;

// Python's round() rounds half-to-even; C++'s std::round rounds
// half-away-from-zero. VBS and integer-snapping rely on exact parity with
// the Python reference, so this matches round() rather than std::round().
inline int64_t python_round(double v) {
    double floor_v = std::floor(v);
    double diff = v - floor_v;
    if (diff < 0.5) return static_cast<int64_t>(floor_v);
    if (diff > 0.5) return static_cast<int64_t>(floor_v) + 1;
    int64_t fi = static_cast<int64_t>(floor_v);
    return (fi % 2 == 0) ? fi : fi + 1;
}

struct Point2i {
    int64_t x = 0;
    int64_t y = 0;

    friend bool operator==(const Point2i& a, const Point2i& b) {
        return a.x == b.x && a.y == b.y;
    }
    friend bool operator!=(const Point2i& a, const Point2i& b) { return !(a == b); }
    friend bool operator<(const Point2i& a, const Point2i& b) {
        return std::tie(a.x, a.y) < std::tie(b.x, b.y);
    }
};

struct Point2 {
    double x = 0.0;
    double y = 0.0;
};

// Sorted-pair vertex-index edge, matching the Python reference's
// `frozenset[tuple[int, int]]` convention used for `constrained` /
// `boundary_edges` / `contour_edges` throughout the spec.
using Edge = std::pair<int, int>;

inline Edge make_edge(int a, int b) { return a < b ? Edge(a, b) : Edge(b, a); }

}  // namespace vbct

namespace std {
template <>
struct hash<vbct::Point2i> {
    size_t operator()(const vbct::Point2i& p) const noexcept {
        size_t h1 = std::hash<int64_t>()(p.x);
        size_t h2 = std::hash<int64_t>()(p.y);
        return h1 ^ (h2 * 0x9E3779B97F4A7C15ULL + 0x9E3779B9 + (h1 << 6) + (h1 >> 2));
    }
};
}  // namespace std
