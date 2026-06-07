// Copyright (c) 2026 Rszalay
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "infill/FuselageStringerInfill.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include <spdlog/spdlog.h>

namespace cura
{

FuselageStringerInfill::FuselageStringerInfill(const Shape& infill_area, coord_t z, const Settings& settings)
    : infill_area_(infill_area)
    , z_(z)
    , settings_(settings)
{
}

void FuselageStringerInfill::generate(OpenLinesSet& result_lines)
{
    if (infill_area_.empty())
        return;

    const int spoke_count = settings_.get<int>("fuselage_spoke_count");
    if (spoke_count < 2)
        return;

    const double helix_pitch  = settings_.get<double>("fuselage_helix_pitch");  // deg/mm
    const double anchor_deg   = settings_.get<double>("fuselage_phase_anchor_angle");
    const bool   dual_helix   = settings_.get<bool>("fuselage_dual_helix");

    // Phase for this layer.
    // CW family rotates +helix_pitch per mm of height.
    // CCW family counter-rotates (-helix_pitch) and starts offset by half a spoke
    // spacing (180/N degrees) so the two families interleave rather than overlap.
    // For even N, a 180-degree offset would land CCW spoke k exactly on CW spoke k+N/2;
    // the half-spoke offset (180/N) keeps all 2N spokes distinct and produces the
    // diamond crossing pattern of the geodesic structure.
    const double z_mm          = static_cast<double>(z_) / 1000.0;
    const double phase_cw_deg  = anchor_deg + z_mm * helix_pitch;
    const double phase_ccw_deg = (anchor_deg + 180.0 / spoke_count) - z_mm * helix_pitch;

    // Identify outer polygon (largest absolute area) and inner polygon (hole, if present).
    const Polygon* outer_poly = nullptr;
    const Polygon* inner_poly = nullptr;
    double max_area = 0.0;

    for (const Polygon& poly : infill_area_)
    {
        double a = std::abs(poly.area());
        if (a > max_area)
        {
            max_area   = a;
            outer_poly = &poly;
        }
    }
    if (! outer_poly || outer_poly->size() < 3)
        return;

    for (const Polygon& poly : infill_area_)
    {
        if (&poly != outer_poly && poly.size() >= 3 && poly.area() < 0)
        {
            inner_poly = &poly;
            break;
        }
    }

    const Point2LL centroid = computeCentroid(*outer_poly);

    // For clipping spokes at the inner wall, aim toward the inner polygon's own
    // centroid. The outer polygon's centroid can be outside the inner polygon on
    // asymmetric lofted cross-sections, causing rays to miss the hole entirely.
    const Point2LL clip_target = inner_poly ? computeCentroid(*inner_poly) : centroid;

    // Helper: generate one family of spokes
    auto emit_family = [&](double phase_deg)
    {
        std::vector<Point2LL> outer_pts = arcLengthSamplePoints(*outer_poly, spoke_count, phase_deg, centroid);

        for (const Point2LL& outer_pt : outer_pts)
        {
            Point2LL inner_pt;
            if (inner_poly)
            {
                auto hit = rayIntersectPolygon(outer_pt, clip_target, *inner_poly);
                inner_pt = hit.value_or(clip_target);
            }
            else
            {
                inner_pt = centroid;
            }
            result_lines.addSegment(outer_pt, inner_pt);
        }
    };

    emit_family(phase_cw_deg);
    if (dual_helix)
        emit_family(phase_ccw_deg);
}

// --- Static helpers ---------------------------------------------------------

Point2LL FuselageStringerInfill::computeCentroid(const Polygon& poly)
{
    // Standard signed-area centroid formula.
    double area = 0.0;
    double cx   = 0.0;
    double cy   = 0.0;

    for (size_t i = 0; i < poly.size(); ++i)
    {
        const size_t j    = (i + 1) % poly.size();
        const double cross = static_cast<double>(poly[i].X) * poly[j].Y
                           - static_cast<double>(poly[j].X) * poly[i].Y;
        area += cross;
        cx   += (poly[i].X + poly[j].X) * cross;
        cy   += (poly[i].Y + poly[j].Y) * cross;
    }

    area /= 2.0;
    if (std::abs(area) < 1.0)
        return poly[0]; // degenerate polygon

    cx /= (6.0 * area);
    cy /= (6.0 * area);
    return Point2LL(static_cast<coord_t>(cx), static_cast<coord_t>(cy));
}

std::vector<Point2LL> FuselageStringerInfill::arcLengthSamplePoints(
    const Polygon& poly,
    int n,
    double phase_deg,
    const Point2LL& centroid)
{
    const size_t vcount = poly.size();
    if (vcount < 2)
        return {};

    // Build cumulative arc-length table.
    std::vector<double> cum(vcount + 1, 0.0);
    for (size_t i = 0; i < vcount; ++i)
    {
        const size_t j = (i + 1) % vcount;
        const double dx = poly[j].X - poly[i].X;
        const double dy = poly[j].Y - poly[i].Y;
        cum[i + 1] = cum[i] + std::sqrt(dx * dx + dy * dy);
    }
    const double total = cum[vcount];
    if (total < 1.0)
        return {};

    // Find where the phase ray (centroid → phase_deg direction) hits the polygon.
    // Arc position is computed directly from the edge parameter u — avoids re-projecting
    // the integer-truncated hit point which introduces ~0.001 u-error for 1 mm edges.
    const double phase_rad = phase_deg * std::numbers::pi / 180.0;
    const double rx = std::cos(phase_rad);
    const double ry = std::sin(phase_rad);

    double phase_arc = 0.0;
    double best_phase_t = std::numeric_limits<double>::max();
    for (size_t i = 0; i < vcount; ++i)
    {
        const size_t j  = (i + 1) % vcount;
        const double sx = poly[j].X - poly[i].X;
        const double sy = poly[j].Y - poly[i].Y;
        const double dx = static_cast<double>(centroid.X) - poly[i].X;
        const double dy = static_cast<double>(centroid.Y) - poly[i].Y;
        const double denom = rx * sy - ry * sx;
        if (std::abs(denom) < 1e-10)
            continue;
        const double t = (dy * sx - dx * sy) / denom;
        const double u = (dy * rx - dx * ry) / denom;
        if (t > 1e-6 && u >= -1e-6 && u <= 1.0 + 1e-6 && t < best_phase_t)
        {
            best_phase_t = t;
            phase_arc    = cum[i] + std::clamp(u, 0.0, 1.0) * (cum[i + 1] - cum[i]);
        }
    }

    // Fallback: if ray cast failed (centroid outside polygon), pick the vertex
    // whose angular direction from centroid is closest to phase_deg.
    if (best_phase_t == std::numeric_limits<double>::max())
    {
        double best_angle_diff = std::numeric_limits<double>::max();
        for (size_t i = 0; i < vcount; ++i)
        {
            const double vx = poly[i].X - centroid.X;
            const double vy = poly[i].Y - centroid.Y;
            const double vertex_angle = std::atan2(vy, vx);
            double diff = std::abs(vertex_angle - phase_rad);
            if (diff > std::numbers::pi)
                diff = 2.0 * std::numbers::pi - diff;
            if (diff < best_angle_diff)
            {
                best_angle_diff = diff;
                phase_arc       = cum[i];
            }
        }
    }

    // Sample n evenly-spaced points starting at phase_arc.
    const double spacing = total / n;
    std::vector<Point2LL> result;
    result.reserve(n);

    for (int k = 0; k < n; ++k)
    {
        double target = std::fmod(phase_arc + k * spacing, total);
        if (target < 0.0)
            target += total;

        // Binary search in cum[] for the edge containing target.
        const auto it  = std::upper_bound(cum.begin(), cum.end(), target);
        size_t edge_idx = static_cast<size_t>(std::distance(cum.begin(), it)) - 1;
        edge_idx        = std::min(edge_idx, vcount - 1);

        const size_t j = (edge_idx + 1) % vcount;
        const double elen = cum[edge_idx + 1] - cum[edge_idx];
        const double t    = (elen > 0.0) ? (target - cum[edge_idx]) / elen : 0.0;

        result.emplace_back(
            static_cast<coord_t>(poly[edge_idx].X + t * (poly[j].X - poly[edge_idx].X)),
            static_cast<coord_t>(poly[edge_idx].Y + t * (poly[j].Y - poly[edge_idx].Y)));
    }

    return result;
}

std::optional<Point2LL> FuselageStringerInfill::rayIntersectPolygon(
    const Point2LL& origin,
    const Point2LL& direction_point,
    const Polygon& poly)
{
    const double rx = direction_point.X - origin.X;
    const double ry = direction_point.Y - origin.Y;

    std::optional<Point2LL> best;
    double best_t = std::numeric_limits<double>::max();

    for (size_t i = 0; i < poly.size(); ++i)
    {
        const size_t j  = (i + 1) % poly.size();
        const double sx = poly[j].X - poly[i].X;
        const double sy = poly[j].Y - poly[i].Y;
        const double dx = origin.X - poly[i].X;
        const double dy = origin.Y - poly[i].Y;

        // Solve: origin + t*R = poly[i] + u*S
        // Standard 2D formulas: t = (Q-P)×S / R×S,  u = (Q-P)×R / R×S
        // where Q-P = (-dx,-dy), so (Q-P)×S = dy*sx - dx*sy
        const double denom = rx * sy - ry * sx; // R × S
        if (std::abs(denom) < 1e-10)
            continue; // parallel

        const double t = (dy * sx - dx * sy) / denom;
        const double u = (dy * rx - dx * ry) / denom;

        if (t > 1e-6 && u >= -1e-6 && u <= 1.0 + 1e-6 && t < best_t)
        {
            best_t = t;
            best   = Point2LL(
                static_cast<coord_t>(origin.X + t * rx),
                static_cast<coord_t>(origin.Y + t * ry));
        }
    }

    return best;
}

} // namespace cura
