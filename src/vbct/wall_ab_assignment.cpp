#include "wall_ab_assignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "contour.hpp"

namespace vbct {

namespace {

double dist(const Point2& a, const Point2& b) { return std::hypot(a.x - b.x, a.y - b.y); }

Point2 mean_point(const std::vector<Point2>& pts) {
    Point2 m{0, 0};
    for (const auto& p : pts) { m.x += p.x; m.y += p.y; }
    if (!pts.empty()) { m.x /= static_cast<double>(pts.size()); m.y /= static_cast<double>(pts.size()); }
    return m;
}

// Shoelace signed area of a closed polygon (positive = CCW / outer
// silhouette convention used throughout stage10.cpp's own `signed_area`;
// negative = a hole, per Clipper's own convention -- see VbctAdapter's
// filterDeminimisHoles doc comment for the FeatherPrint-side precedent).
double signed_area(const std::vector<Point2>& pts) {
    double a = 0.0;
    size_t n = pts.size();
    for (size_t i = 0; i < n; ++i) {
        const auto& p = pts[i];
        const auto& q = pts[(i + 1) % n];
        a += p.x * q.y - q.x * p.y;
    }
    return a * 0.5;
}

// Nearest distance from a point to a closed polyline (as a set of segments).
double dist_to_contour(const Point2& p, const std::vector<Point2>& contour) {
    double best = std::numeric_limits<double>::max();
    size_t n = contour.size();
    if (n < 2) return best;
    for (size_t i = 0; i < n; ++i) {
        const auto& a = contour[i];
        const auto& b = contour[(i + 1) % n];
        double dx = b.x - a.x, dy = b.y - a.y;
        double len2 = dx * dx + dy * dy;
        double t = len2 > 1e-12 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
        t = std::clamp(t, 0.0, 1.0);
        Point2 proj{a.x + t * dx, a.y + t * dy};
        best = std::min(best, dist(p, proj));
    }
    return best;
}

constexpr double kContourMatchTolerance = 0.5;  // mm; same scale family as Stage 1's dedup epsilon

}  // namespace

WallAbAssignment assign_wall_ab_rule1(const std::vector<Point2>& left, const std::vector<Point2>& right,
                                       const std::vector<Contour>& contours) {
    auto best_contour = [&](const std::vector<Point2>& wall) -> std::optional<size_t> {
        for (size_t ci = 0; ci < contours.size(); ++ci) {
            std::vector<Point2> pts = contours[ci].to_float();
            bool all_close = true;
            for (const auto& p : wall) {
                if (dist_to_contour(p, pts) > kContourMatchTolerance) { all_close = false; break; }
            }
            if (all_close) return ci;
        }
        return std::nullopt;
    };

    WallAbAssignment r;
    r.rule_used = "rule1";

    auto cl = best_contour(left);
    auto cr = best_contour(right);
    if (!cl || !cr) {
        r.reason = "one or both walls did not confidently match any source contour -- ambiguous";
        r.rule_used.clear();
        return r;
    }
    if (*cl == *cr) {
        r.reason = "both walls matched the same contour '" + contours[*cl].contour_id + "' -- no hole to compare against";
        r.rule_used.clear();
        return r;
    }

    double al = signed_area(contours[*cl].to_float());
    double ar = signed_area(contours[*cr].to_float());
    if ((al > 0) == (ar > 0)) {
        r.reason = "matched contours '" + contours[*cl].contour_id + "'/'" + contours[*cr].contour_id +
                   "' have the same area sign -- ambiguous";
        r.rule_used.clear();
        return r;
    }

    r.left_label = (al > 0) ? WallLabel::A : WallLabel::B;
    r.reason = "left on contour '" + contours[*cl].contour_id + "' (area " + std::to_string(al) +
               "), right on '" + contours[*cr].contour_id + "' (area " + std::to_string(ar) + ")";
    return r;
}

WallAbAssignment assign_wall_ab_rule2(const std::vector<Point2>& left, const std::vector<Point2>& right) {
    // Signed area between a wall and the straight chord joining its own two
    // endpoints -- self-contained (never looks at the other wall's points),
    // so unlike a nearest-other-wall-point "which side is interior" test it
    // cannot be confused by the two walls converging toward each other
    // anywhere along their length. Both walls share the same two endpoints
    // by construction (both run cap_start -> cap_end), so the two areas are
    // directly comparable without any extra normalisation between them.
    auto chord_area = [](const std::vector<Point2>& wall) -> double {
        if (wall.size() < 2) return 0.0;
        double a = 0.0;
        for (size_t i = 0; i + 1 < wall.size(); ++i) a += wall[i].x * wall[i + 1].y - wall[i + 1].x * wall[i].y;
        a += wall.back().x * wall.front().y - wall.front().x * wall.back().y;  // close back to the wall's own start
        return a / 2.0;
    };

    double al = std::abs(chord_area(left));
    double ar = std::abs(chord_area(right));
    double chord_len = (left.empty() || right.empty()) ? 0.0 : dist(left.front(), left.back());
    double denom = chord_len * chord_len;

    // "Is there real curvature at all" floor, normalised by the shared
    // chord's own length^2 (a dimensionless bulge ratio) so it stays valid
    // at any physical scale -- an absolute mm^2 floor would fail the exact
    // way every absolute-threshold attempt did elsewhere in this
    // investigation (correct on the part it was tuned against, wrong at a
    // different scale). Measured >=1000x apart on real data: a sub-noise
    // bow scores ~3e-5, real signal (a synthetic fin, a real airfoil slice,
    // FPTF-45's own corridor) scores 0.05-2+.
    constexpr double kMinBulgeRatio = 1e-3;
    double winner = std::max(al, ar);

    WallAbAssignment r;
    if (denom < 1e-12 || winner / denom < kMinBulgeRatio) {
        r.reason = "neither wall bulges enough relative to its own chord to trust (|left|=" + std::to_string(al) +
                   ", |right|=" + std::to_string(ar) + ", chord=" + std::to_string(chord_len) + ")";
        return r;  // rule_used left empty => ambiguous
    }
    r.rule_used = "rule2";
    r.left_label = (al >= ar) ? WallLabel::A : WallLabel::B;
    r.reason = "left chord-area magnitude " + std::to_string(al) + ", right " + std::to_string(ar) +
               " (larger bulge relative to own chord = Wall A)";
    return r;
}

WallAbAssignment assign_wall_ab_rule3(const std::vector<Point2>& left, const std::vector<Point2>& right,
                                       const Point2& part_centroid) {
    double dl = dist(mean_point(left), part_centroid);
    double dr = dist(mean_point(right), part_centroid);
    WallAbAssignment r;
    r.rule_used = "rule3";  // always resolves -- true last resort
    r.left_label = (dl >= dr) ? WallLabel::A : WallLabel::B;
    r.reason = "left mean-to-centroid " + std::to_string(dl) + ", right mean-to-centroid " + std::to_string(dr) +
               " (farther = Wall A)";
    return r;
}

WallAbAssignment assign_wall_ab(const std::vector<Point2>& left, const std::vector<Point2>& right,
                                 const std::vector<Contour>& contours, const Point2& part_centroid) {
    if (auto r = assign_wall_ab_rule1(left, right, contours); !r.rule_used.empty()) return r;
    if (auto r = assign_wall_ab_rule2(left, right); !r.rule_used.empty()) return r;
    return assign_wall_ab_rule3(left, right, part_centroid);
}

}  // namespace vbct
