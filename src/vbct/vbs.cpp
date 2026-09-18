#include "vbs.hpp"

#include <cmath>

namespace vbct {

double edge_length(const IEdge& e) {
    double dx = static_cast<double>(e.b.x - e.a.x);
    double dy = static_cast<double>(e.b.y - e.a.y);
    return std::hypot(dx, dy);
}

std::vector<IEdge> split_edge(const IEdge& e, int k) {
    std::vector<Point2i> points;
    points.reserve(k + 1);
    points.push_back(e.a);
    for (int i = 1; i < k; ++i) {
        double t = static_cast<double>(i) / k;
        int64_t x = python_round(e.a.x + (e.b.x - e.a.x) * t);
        int64_t y = python_round(e.a.y + (e.b.y - e.a.y) * t);
        points.push_back({x, y});
    }
    points.push_back(e.b);

    std::vector<IEdge> result;
    result.reserve(k);
    for (int i = 0; i < k; ++i) {
        result.push_back({points[i], points[i + 1]});
    }
    return result;
}

std::vector<IEdge> vbs(const std::vector<IEdge>& lines, double x) {
    if (lines.empty()) return lines;

    double total = 0.0;
    for (const auto& l : lines) total += edge_length(l);
    double a0 = total / static_cast<double>(lines.size());
    double upper = a0 * (1.0 + x);

    std::vector<IEdge> result = lines;
    while (true) {
        std::vector<IEdge> next_result;
        next_result.reserve(result.size());
        bool changed = false;
        for (const auto& line : result) {
            double l = edge_length(line);
            if (l > upper) {
                int k = std::max(2, static_cast<int>(std::ceil(l / a0)));
                auto pieces = split_edge(line, k);
                next_result.insert(next_result.end(), pieces.begin(), pieces.end());
                changed = true;
            } else {
                next_result.push_back(line);
            }
        }
        result = std::move(next_result);
        if (!changed) return result;
    }
}

}  // namespace vbct
