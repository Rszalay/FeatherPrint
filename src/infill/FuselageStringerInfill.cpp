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
    const bool   use_or_mode       = settings_.get<std::string>("fuselage_arc_mode") == "or";
    const bool   strip_outer_wall  = settings_.get<bool>("fuselage_strip_outer_wall");
    const bool   strip_inner_wall  = settings_.get<bool>("fuselage_strip_inner_wall");
    const bool   invert_stringer   = settings_.get<bool>("fuselage_invert_stringer");
    const double seam_scarf_mm     = settings_.get<double>("fuselage_seam_scarf_mm");

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

    // Layer number — used for seam scatter.
    const coord_t layer_height_um = settings_.get<coord_t>("layer_height");
    const int layer_nr = (layer_height_um > 0) ? static_cast<int>(z_ / layer_height_um) : 0;

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

    // Walls are always generated (so inner_area / skin / infill geometry is correct), but
    // when a strip flag is set FffGcodeWriter clears wall_toolpaths before printing.
    // inner_area is therefore still inset by wall_line_width_0 from the actual skin.
    // The stringer must shift outward by wall_line_width_0 to sit on the wall centerline.
    // nudge_pt: positive dist = toward centroid (inward); negative = outward.
    const coord_t nozzle_d   = static_cast<coord_t>(settings_.get<double>("machine_nozzle_size") * 1000.0);
    const coord_t wall_lw    = settings_.get<coord_t>("wall_line_width_0");
    const coord_t trough_off = strip_outer_wall ? -(wall_lw - nozzle_d) : nozzle_d;
    const coord_t peak_off   = strip_inner_wall  ? -(wall_lw - nozzle_d) : nozzle_d;

    // Nudge a point toward a reference point by dist microns.
    auto nudge_pt = [](const Point2LL& pt, const Point2LL& toward, coord_t dist) -> Point2LL
    {
        if (dist == 0)
            return pt;
        const double dx  = toward.X - pt.X;
        const double dy  = toward.Y - pt.Y;
        const double len = std::sqrt(dx * dx + dy * dy);
        if (len < 1.0)
            return pt;
        return Point2LL(pt.X + static_cast<coord_t>(dx / len * dist),
                        pt.Y + static_cast<coord_t>(dy / len * dist));
    };

    // Build one corrugated chain per helix family.
    //
    // Corrugation cycle (2 spokes):
    //   nudged_outer[k]  --wall-->  inner[k]
    //   inner[k]  --peak arc (inner poly, backward/CCW)-->  inner[k+1]
    //   inner[k+1]  --wall-->  nudged_outer[k+1]
    //   nudged_outer[k+1]  --trough arc (outer poly, forward/CCW, nudged inward)-->  nudged_outer[k+2]
    //
    // Inner polygon is CW (negative area), so traversing it CCW means walking backward
    // in vertex index order.  Outer polygon is CCW, so trough arcs walk forward.
    //
    // Inner points are ray-cast from each outer sample point so they are radially aligned —
    // arc-length sampling the CW inner polygon gives samples in the wrong angular order
    // relative to the CCW outer samples, which produces non-radial walls.
    auto emit_zigzag = [&](double phase_deg)
    {
        auto outer_pts = arcLengthSamplePoints(*outer_poly, spoke_count, phase_deg, centroid);

        // Inner points: ray-cast from each outer sample through clip_target to get a point
        // on the inner polygon at the same radial direction.  This guarantees outer[k] and
        // inner[k] lie on the same ray from the centroid → radial walls are truly radial.
        std::vector<SamplePt> inner_pts;
        if (inner_poly)
        {
            inner_pts.reserve(spoke_count);
            for (const SamplePt& op : outer_pts)
            {
                auto hit = rayIntersectPolygon(op.pt, clip_target, *inner_poly);
                inner_pts.push_back(hit.value_or(SamplePt{ clip_target, 0 }));
            }
        }
        else
        {
            inner_pts.assign(spoke_count, { centroid, 0 });
        }

        if (static_cast<int>(outer_pts.size()) < spoke_count
            || static_cast<int>(inner_pts.size()) < spoke_count)
            return;

        OpenPolyline path;
        path.reserve(spoke_count * 8);

        if (!invert_stringer)
        {
            path.push_back(nudge_pt(outer_pts[0].pt, centroid, trough_off));

            for (int k = 0; k + 1 < spoke_count; k += 2)
            {
                path.push_back(inner_pts[k].pt);

                if (inner_poly)
                    appendArc(path, *inner_poly, inner_pts[k], inner_pts[k + 1],
                              centroid, -peak_off);
                else
                    path.push_back(inner_pts[k + 1].pt);

                path.push_back(nudge_pt(outer_pts[k + 1].pt, centroid, trough_off));

                if (k + 2 < spoke_count)
                    appendArc(path, *outer_poly, outer_pts[k + 1], outer_pts[k + 2],
                              centroid, trough_off);
            }

            if (spoke_count % 2 == 1)
                path.push_back(inner_pts[spoke_count - 1].pt);

            if (spoke_count >= 2 && spoke_count % 2 == 0)
                appendArc(path, *outer_poly, outer_pts[spoke_count - 1], outer_pts[0],
                          centroid, trough_off);
        }
        else
        {
            // Inverted: trough arcs on inner polygon, peak walls touch outer polygon.
            if (!inner_poly) return;

            path.push_back(inner_pts[0].pt);

            for (int k = 0; k + 1 < spoke_count; k += 2)
            {
                path.push_back(nudge_pt(outer_pts[k].pt, centroid, trough_off));

                appendArc(path, *outer_poly, outer_pts[k], outer_pts[k + 1],
                          centroid, trough_off);

                path.push_back(inner_pts[k + 1].pt);

                if (k + 2 < spoke_count)
                    appendArc(path, *inner_poly, inner_pts[k + 1], inner_pts[k + 2],
                              centroid, -peak_off);
            }

            if (spoke_count % 2 == 1)
                path.push_back(nudge_pt(outer_pts[spoke_count - 1].pt, centroid, trough_off));

            if (spoke_count >= 2 && spoke_count % 2 == 0)
                appendArc(path, *inner_poly, inner_pts[spoke_count - 1], inner_pts[0],
                          centroid, -peak_off);
        }

        // Scarf overlap: retrace the first seam_scarf_mm of the path to bond the seam.
        if (path.size() >= 2 && seam_scarf_mm > 0.0)
        {
            const double scarf_um = seam_scarf_mm * 1000.0;
            const size_t orig_sz = path.size();
            double accumulated = 0.0;
            for (size_t i = 1; i < orig_sz && accumulated < scarf_um; ++i)
            {
                const Point2LL pt = path[i];
                const double dx = pt.X - static_cast<double>(path[i - 1].X);
                const double dy = pt.Y - static_cast<double>(path[i - 1].Y);
                accumulated += std::sqrt(dx * dx + dy * dy);
                path.push_back(pt);
            }
        }

        if (path.size() >= 2)
            result_lines.push_back(std::move(path));
    };

    // For dual helix, merge H1 and H2 stringer positions into a single 2N-wall corrugated
    // path so that their trough/peak zones interleave rather than overlap.
    //
    // Zone logic (Section 4, XOR mode): merged CCW order gives 2N zones whose truth values
    // always alternate T/F when H2 is offset by half a sector from H1.  T = trough (outer),
    // F = peak (inner).  Both helices are folded into ONE polyline; the two-helix geometry
    // emerges across layers from the opposite sign of the helix pitch terms.
    auto emit_merged_zigzag = [&]()
    {
        auto outer_h1 = arcLengthSamplePoints(*outer_poly, spoke_count, phase_cw_deg,  centroid);
        auto outer_h2 = arcLengthSamplePoints(*outer_poly, spoke_count, phase_ccw_deg, centroid);

        auto make_inner = [&](const std::vector<SamplePt>& outer_pts) -> std::vector<SamplePt>
        {
            std::vector<SamplePt> inner;
            inner.reserve(spoke_count);
            if (inner_poly)
            {
                for (const SamplePt& op : outer_pts)
                {
                    auto hit = rayIntersectPolygon(op.pt, clip_target, *inner_poly);
                    inner.push_back(hit.value_or(SamplePt{ clip_target, 0 }));
                }
            }
            else
            {
                inner.assign(spoke_count, { centroid, 0 });
            }
            return inner;
        };

        const auto inner_h1 = make_inner(outer_h1);
        const auto inner_h2 = make_inner(outer_h2);

        // Merge all 2N stringer positions in CCW angular order starting from phase_cw_deg.
        struct MergedPt
        {
            SamplePt outer;
            SamplePt inner;
            bool     is_h1; // true = H1 stringer; H1 points must land at even merged indices
        };

        const double ref_angle = phase_cw_deg * std::numbers::pi / 180.0;
        auto norm_angle = [&](double a) -> double
        {
            double d = a - ref_angle;
            while (d < 0.0)                      d += 2.0 * std::numbers::pi;
            while (d >= 2.0 * std::numbers::pi)  d -= 2.0 * std::numbers::pi;
            return d;
        };

        std::vector<std::pair<double, MergedPt>> tagged;
        tagged.reserve(2 * spoke_count);

        for (int k = 0; k < spoke_count; ++k)
        {
            double a = std::atan2(
                static_cast<double>(outer_h1[k].pt.Y - centroid.Y),
                static_cast<double>(outer_h1[k].pt.X - centroid.X));
            tagged.push_back({ norm_angle(a), { outer_h1[k], inner_h1[k], true } });
        }
        for (int k = 0; k < spoke_count; ++k)
        {
            double a = std::atan2(
                static_cast<double>(outer_h2[k].pt.Y - centroid.Y),
                static_cast<double>(outer_h2[k].pt.X - centroid.X));
            tagged.push_back({ norm_angle(a), { outer_h2[k], inner_h2[k], false } });
        }

        std::stable_sort(tagged.begin(), tagged.end(),
                         [](const auto& a, const auto& b) { return a.first < b.first; });

        std::vector<MergedPt> merged;
        merged.reserve(tagged.size());
        for (auto& t : tagged)
            merged.push_back(std::move(t.second));

        // Guarantee the list starts at an H1 point so that merged[0] always enters
        // a T (trough) zone.  If the sort placed an H2 point first (possible when the
        // H2 arc-length phase projects to an angle slightly before H1's on non-circular
        // cross-sections), rotate left by one to restore the H1-first invariant.
        {
            auto it = std::find_if(merged.begin(), merged.end(),
                                   [](const MergedPt& p) { return p.is_h1; });
            if (it != merged.begin() && it != merged.end())
                std::rotate(merged.begin(), it, merged.end());
        }

        const int mn = static_cast<int>(merged.size()); // == 2 * spoke_count
        if (mn < 2)
            return;

        // Same corrugation loop as emit_zigzag but over the 2N merged positions.
        OpenPolyline path;
        path.reserve(mn * 8);

        // Zone truth table — computed from actual H1/H2 sector indices, not from the
        // merged list position.  Using position (k%2, k/2) silently assumes perfect
        // H1/H2 alternation, which breaks when helix pitch drifts the phase offset away
        // from exactly Δ/2.  Sector-tracking handles arbitrary interleaving.
        //
        // H1 sector truth = (sector_index % 2 == 0).
        // H2 sector truth = (sector_index % 2 == 1)  [inverted parity — derived from XOR].
        // Zone truth = OPERATOR(h1_truth, h2_truth).
        //
        // cur_h2 is initialised to N-1 (H2 sector N-1 spans from H2[N-1] to H2[0]).
        // Since arcLengthSamplePoints returns H2 points in CCW arc-length order, H2[N-1]
        // is always the last H2 in the merged list — just before the wrap back to H1[0].
        std::vector<bool> zone_trough(mn);
        {
            int cur_h1 = 0;
            int cur_h2 = spoke_count - 1;
            int h1_seq = 0, h2_seq = 0;
            for (int k = 0; k < mn; ++k)
            {
                if (merged[k].is_h1)
                    cur_h1 = h1_seq++;
                else
                    cur_h2 = h2_seq++;
                const bool h1_truth = (cur_h1 % 2 == 0);
                const bool h2_truth = (cur_h2 % 2 == 1);
                zone_trough[k] = use_or_mode ? (h1_truth | h2_truth)
                                             : (h1_truth ^ h2_truth);
            }
        }
        if (invert_stringer)
            for (size_t i = 0; i < zone_trough.size(); ++i) zone_trough[i] = !zone_trough[i];

        // Seam scatter: rotate the starting zone by a layer-dependent step so the
        // open-polyline seam cycles through all spoke positions instead of drifting
        // slowly with the helix phase.  Both arrays rotate together so zone_trough[k]
        // still correctly describes the arc between the new merged[k] and merged[k+1].
        // Step of 2 per layer advances by one H1 spoke; completes a full cycle in
        // spoke_count layers (which is also one full CCW circuit of seam positions).
        {
            const int scatter = (layer_nr * 2) % mn;
            if (scatter > 0)
            {
                std::rotate(merged.begin(),     merged.begin()     + scatter, merged.end());
                std::rotate(zone_trough.begin(), zone_trough.begin() + scatter, zone_trough.end());
            }
        }

        // Build the corrugated path zone by zone.
        // Invariant: the path ends each zone at merged[k+1]'s trough or peak level,
        // matching zone_trough[k].  Walls are only emitted when the level changes.
        path.push_back(nudge_pt(merged[0].outer.pt, centroid, trough_off));
        bool at_trough = true;

        for (int k = 0; k < mn; ++k)
        {
            const int  nxt        = (k + 1) % mn;
            const bool want_trough = zone_trough[k];

            // Transition wall at merged[k] stringer if level changes.
            if (want_trough != at_trough)
            {
                if (want_trough)
                    path.push_back(nudge_pt(merged[k].outer.pt, centroid, trough_off));
                else
                    path.push_back(merged[k].inner.pt);
                at_trough = want_trough;
            }

            // Arc through zone k.
            if (want_trough)
                appendArc(path, *outer_poly, merged[k].outer, merged[nxt].outer,
                          centroid, trough_off);
            else if (inner_poly)
                appendArc(path, *inner_poly, merged[k].inner, merged[nxt].inner,
                          centroid, -peak_off);
            else
                path.push_back(merged[nxt].inner.pt);
        }

        // Scarf overlap: retrace the first seam_scarf_mm of the path to bond the seam.
        if (path.size() >= 2 && seam_scarf_mm > 0.0)
        {
            const double scarf_um = seam_scarf_mm * 1000.0;
            const size_t orig_sz = path.size();
            double accumulated = 0.0;
            for (size_t i = 1; i < orig_sz && accumulated < scarf_um; ++i)
            {
                const Point2LL pt = path[i];
                const double dx = pt.X - static_cast<double>(path[i - 1].X);
                const double dy = pt.Y - static_cast<double>(path[i - 1].Y);
                accumulated += std::sqrt(dx * dx + dy * dy);
                path.push_back(pt);
            }
        }

        if (path.size() >= 2)
            result_lines.push_back(std::move(path));
    };

    if (dual_helix)
        emit_merged_zigzag();
    else
        emit_zigzag(phase_cw_deg);

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

std::vector<FuselageStringerInfill::SamplePt> FuselageStringerInfill::arcLengthSamplePoints(
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
    std::vector<SamplePt> result;
    result.reserve(n);

    for (int k = 0; k < n; ++k)
    {
        double target = std::fmod(phase_arc + k * spacing, total);
        if (target < 0.0)
            target += total;

        // Binary search in cum[] for the edge containing target.
        const auto it   = std::upper_bound(cum.begin(), cum.end(), target);
        size_t edge_idx = static_cast<size_t>(std::distance(cum.begin(), it)) - 1;
        edge_idx        = std::min(edge_idx, vcount - 1);

        const size_t j    = (edge_idx + 1) % vcount;
        const double elen = cum[edge_idx + 1] - cum[edge_idx];
        const double t    = (elen > 0.0) ? (target - cum[edge_idx]) / elen : 0.0;

        result.push_back({
            Point2LL(
                static_cast<coord_t>(poly[edge_idx].X + t * (poly[j].X - poly[edge_idx].X)),
                static_cast<coord_t>(poly[edge_idx].Y + t * (poly[j].Y - poly[edge_idx].Y))),
            edge_idx});
    }

    return result;
}

void FuselageStringerInfill::appendArc(
    OpenPolyline& path,
    const Polygon& poly,
    const SamplePt& from,
    const SamplePt& to,
    const Point2LL& nudge_toward,
    coord_t nudge_dist)
{
    const size_t n = poly.size();
    if (n == 0)
        return;

    auto apply_nudge = [&](const Point2LL& pt) -> Point2LL
    {
        if (nudge_dist == 0)
            return pt;
        const double dx  = nudge_toward.X - pt.X;
        const double dy  = nudge_toward.Y - pt.Y;
        const double len = std::sqrt(dx * dx + dy * dy);
        if (len < 1.0)
            return pt;
        // Positive nudge_dist → move toward nudge_toward (inward for trough arcs).
        // Negative nudge_dist → move away from nudge_toward (outward for peak arcs).
        return Point2LL(pt.X + static_cast<coord_t>(dx / len * nudge_dist),
                        pt.Y + static_cast<coord_t>(dy / len * nudge_dist));
    };

    // Always walk the shorter arc (< n/2 steps).  This works correctly for both CCW outer
    // polygons (forward walk is shorter) and CW inner polygons (backward walk is shorter).
    const size_t steps_fwd = (to.edge_idx - from.edge_idx + n) % n;
    const size_t steps_bwd = n - steps_fwd;

    if (steps_fwd <= steps_bwd)
    {
        for (size_t s = 1; s <= steps_fwd; ++s)
            path.push_back(apply_nudge(poly[(from.edge_idx + s) % n]));
    }
    else
    {
        // Backward walk: start at poly[from.edge_idx] (the CW-adjacent vertex from from.pt),
        // stop at poly[to.edge_idx + 1] (the CW entry point of to's edge), then add to.pt.
        // Range s=0..steps_bwd-1 (not s=1..steps_bwd) avoids a CW-then-CCW kink at
        // poly[to.edge_idx] that would cause visible overshoot past the endpoint.
        for (size_t s = 0; s < steps_bwd; ++s)
            path.push_back(apply_nudge(poly[(from.edge_idx - s + n) % n]));
    }

    path.push_back(apply_nudge(to.pt));
}

std::optional<FuselageStringerInfill::SamplePt> FuselageStringerInfill::rayIntersectPolygon(
    const Point2LL& origin,
    const Point2LL& direction_point,
    const Polygon& poly)
{
    const double rx = direction_point.X - origin.X;
    const double ry = direction_point.Y - origin.Y;

    std::optional<SamplePt> best;
    double best_t = std::numeric_limits<double>::max();

    for (size_t i = 0; i < poly.size(); ++i)
    {
        const size_t j  = (i + 1) % poly.size();
        const double sx = poly[j].X - poly[i].X;
        const double sy = poly[j].Y - poly[i].Y;
        const double dx = origin.X - poly[i].X;
        const double dy = origin.Y - poly[i].Y;

        const double denom = rx * sy - ry * sx;
        if (std::abs(denom) < 1e-10)
            continue;

        const double t = (dy * sx - dx * sy) / denom;
        const double u = (dy * rx - dx * ry) / denom;

        if (t > 1e-6 && u >= -1e-6 && u <= 1.0 + 1e-6 && t < best_t)
        {
            best_t = t;
            best   = SamplePt{
                Point2LL(static_cast<coord_t>(origin.X + t * rx),
                         static_cast<coord_t>(origin.Y + t * ry)),
                i};
        }
    }

    return best;
}

} // namespace cura
