#include "transform.hpp"

#include <cmath>

namespace vbct {
namespace {

Point2 centroid(const std::vector<Contour>& contours) {
    double sx = 0, sy = 0;
    int n = 0;
    for (const auto& c : contours) {
        for (const auto& p : c.to_float()) {
            sx += p.x;
            sy += p.y;
            ++n;
        }
    }
    return {sx / n, sy / n};
}

}  // namespace

std::vector<Contour> rotate_contours(const std::vector<Contour>& contours, double angle_rad,
                                      std::optional<Point2> center) {
    Point2 c = center.value_or(centroid(contours));
    double cos_a = std::cos(angle_rad), sin_a = std::sin(angle_rad);

    std::vector<Contour> rotated;
    rotated.reserve(contours.size());
    for (const auto& contour : contours) {
        std::vector<Point2> pts;
        pts.reserve(contour.points.size());
        for (const auto& p : contour.to_float()) {
            double dx = p.x - c.x, dy = p.y - c.y;
            pts.push_back({c.x + dx * cos_a - dy * sin_a, c.y + dx * sin_a + dy * cos_a});
        }
        rotated.push_back(from_float_points(pts, contour.contour_id));
    }
    return rotated;
}

}  // namespace vbct
