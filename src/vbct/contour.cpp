#include "contour.hpp"

namespace vbct {

Contour from_float_points(const std::vector<Point2>& points, const std::string& contour_id) {
    Contour c;
    c.contour_id = contour_id;
    c.points.reserve(points.size());
    for (const auto& p : points) {
        c.points.push_back({python_round(p.x * SCALE), python_round(p.y * SCALE)});
    }
    return c;
}

Contour collapse_near_duplicate_points(const Contour& c, int64_t epsilon_scaled) {
    if (c.points.size() < 3) return c;

    const int64_t eps2 = epsilon_scaled * epsilon_scaled;
    std::vector<Point2i> kept;
    kept.reserve(c.points.size());
    for (const auto& p : c.points) {
        if (!kept.empty()) {
            int64_t dx = p.x - kept.back().x;
            int64_t dy = p.y - kept.back().y;
            if (dx * dx + dy * dy <= eps2) continue;  // near-duplicate of the last kept point
        }
        kept.push_back(p);
    }
    // Wrap-around: the closing edge from the last kept point back to the first.
    if (kept.size() >= 2) {
        int64_t dx = kept.front().x - kept.back().x;
        int64_t dy = kept.front().y - kept.back().y;
        if (dx * dx + dy * dy <= eps2) kept.pop_back();
    }

    if (kept.size() < 3) return c;  // collapsing would break the contour -- leave it for Stage 1 to reject clearly

    Contour result;
    result.points = std::move(kept);
    result.contour_id = c.contour_id;
    return result;
}

}  // namespace vbct
