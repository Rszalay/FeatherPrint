// Copyright (c) 2026 Rszalay
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "featherprint/FeatherPrintGenerator.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "utils/AABB.h"
#include <spdlog/spdlog.h>

namespace cura
{

// ============================================================================
// ArcParam helpers
// ============================================================================

FeatherPrintGenerator::ArcParam FeatherPrintGenerator::buildArcParam(const Polygon& poly)
{
    ArcParam ap;
    ap.poly    = &poly;
    ap.is_open = false;
    int n = static_cast<int>(poly.size());
    ap.cum_len.resize(n, 0.0);

    for (int i = 1; i < n; i++)
    {
        const Point2LL& a = poly[i - 1];
        const Point2LL& b = poly[i];
        double dx = b.X - a.X, dy = b.Y - a.Y;
        ap.cum_len[i] = ap.cum_len[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    {
        const Point2LL& a = poly[n - 1];
        const Point2LL& b = poly[0];
        double dx = b.X - a.X, dy = b.Y - a.Y;
        ap.total = ap.cum_len[n - 1] + std::sqrt(dx * dx + dy * dy);
    }
    return ap;
}

FeatherPrintGenerator::ArcParam FeatherPrintGenerator::buildArcParamOpen(const OpenPolyline& poly)
{
    ArcParam ap;
    ap.poly    = &poly;
    ap.is_open = true;
    int n = static_cast<int>(poly.size());
    ap.cum_len.resize(n, 0.0);

    for (int i = 1; i < n; i++)
    {
        const Point2LL& a = poly[i - 1];
        const Point2LL& b = poly[i];
        double dx = b.X - a.X, dy = b.Y - a.Y;
        ap.cum_len[i] = ap.cum_len[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    ap.total = ap.cum_len[n - 1]; // No closing segment
    return ap;
}

Point2LL FeatherPrintGenerator::ArcParam::pointAt(double s) const
{
    if (is_open)
        s = std::max(0.0, std::min(total, s));
    else
    {
        s = std::fmod(s, total);
        if (s < 0.0) s += total;
    }

    const int n = static_cast<int>(poly->size());
    for (int i = 0; i < n; i++)
    {
        int j = (i + 1) % n;
        if (is_open && j == 0) break; // skip nonexistent closing segment
        double seg_end = (j == 0) ? total : cum_len[j];
        double seg_start = cum_len[i];
        if (s <= seg_end + 1e-6)
        {
            double seg_len = seg_end - seg_start;
            double t = (seg_len > 1e-6) ? (s - seg_start) / seg_len : 0.0;
            t = std::max(0.0, std::min(1.0, t));
            const Point2LL& a = (*poly)[i];
            const Point2LL& b = (*poly)[j];
            return Point2LL(
                a.X + static_cast<coord_t>((b.X - a.X) * t),
                a.Y + static_cast<coord_t>((b.Y - a.Y) * t));
        }
    }
    return is_open ? (*poly)[n - 1] : (*poly)[0];
}

Point2LL FeatherPrintGenerator::ArcParam::tangentAt(double s) const
{
    if (is_open)
        s = std::max(0.0, std::min(total, s));
    else
    {
        s = std::fmod(s, total);
        if (s < 0.0) s += total;
    }

    const int n = static_cast<int>(poly->size());
    for (int i = 0; i < n; i++)
    {
        int j = (i + 1) % n;
        if (is_open && j == 0) break;
        double seg_end = (j == 0) ? total : cum_len[j];
        if (s <= seg_end + 1e-6)
        {
            const Point2LL& a = (*poly)[i];
            const Point2LL& b = (*poly)[j];
            return b - a;
        }
    }
    return is_open ? ((*poly)[n - 1] - (*poly)[n - 2]) : ((*poly)[1] - (*poly)[0]);
}

double FeatherPrintGenerator::ArcParam::referenceArcPos(const Point2LL& centroid) const
{
    int n = static_cast<int>(poly->size());
    for (int i = 0; i < n; i++)
    {
        int j = (i + 1) % n;
        if (is_open && j == 0) continue; // no closing segment for open polylines
        const Point2LL& a = (*poly)[i];
        const Point2LL& b = (*poly)[j];
        double ay = static_cast<double>(a.Y - centroid.Y);
        double by = static_cast<double>(b.Y - centroid.Y);
        // Segment crosses the horizontal ray Y=0 (centroid-relative)
        if ((ay <= 0.0 && by > 0.0) || (ay > 0.0 && by <= 0.0))
        {
            double t  = ay / (ay - by);
            double ix = a.X + t * (b.X - a.X);
            if (ix > static_cast<double>(centroid.X)) // only the +X side
            {
                double seg_start = cum_len[i];
                double seg_end   = (j == 0) ? total : cum_len[j];
                return seg_start + t * (seg_end - seg_start);
            }
        }
    }
    return 0.0; // fallback: no crossing found
}

Point2LL FeatherPrintGenerator::centroidBbox(const Polygon& poly)
{
    AABB bb(poly);
    return Point2LL((bb.min_.X + bb.max_.X) / 2, (bb.min_.Y + bb.max_.Y) / 2);
}

const Polygon* FeatherPrintGenerator::largestPoly(const Shape& shape)
{
    const Polygon* best = nullptr;
    double best_area = 0.0;
    for (const Polygon& p : shape)
    {
        double a = std::abs(p.area());
        if (a > best_area) { best_area = a; best = &p; }
    }
    return best;
}

// ============================================================================
// Perimeter segment walk — appends polygon vertices between s0 and s1
// ============================================================================

void FeatherPrintGenerator::appendPolySegment(
    ExtrusionLine& line,
    const ArcParam& arc,
    double s0,
    double s1,
    coord_t w,
    bool add_start)
{
    if (add_start)
        line.junctions_.emplace_back(arc.pointAt(s0), w, 0);

    if (s1 <= s0)
        return;

    if (! arc.is_open && s1 > arc.total)
    {
        // Segment wraps past the polygon's closing point (s=0/total coincide): walk the
        // s0..total tail, then continue 0..(s1-total). Needed so a Flare/Trace anchor whose
        // occupied interval straddles the seam still gets full perimeter coverage instead of
        // being silently dropped.
        appendPolySegment(line, arc, s0, arc.total, w, /*add_start=*/false);
        appendPolySegment(line, arc, 0.0, s1 - arc.total, w, /*add_start=*/false);
        return;
    }

    int n = static_cast<int>(arc.poly->size());
    for (int i = 0; i < n; i++)
    {
        double sv = arc.cum_len[i];
        if (sv > s0 + 1e-3 && sv < s1 - 1e-3)
            line.junctions_.emplace_back((*arc.poly)[i], w, 0);
    }
    line.junctions_.emplace_back(arc.pointAt(s1), w, 0);
}

// ============================================================================
// Conformal Placement Transform (spec: FeatherPrint_ConformalPlacement_Spec.md)
// ============================================================================

double FeatherPrintGenerator::ArcParam::radiusAt(const Point2LL& centroid, double theta) const
{
    const double cos_t = std::cos(theta), sin_t = std::sin(theta);
    const int n = static_cast<int>(poly->size());
    for (int i = 0; i < n; i++)
    {
        int j = (i + 1) % n;
        if (is_open && j == 0) continue; // no closing segment for open polylines
        double ax = (*poly)[i].X - centroid.X, ay = (*poly)[i].Y - centroid.Y;
        double bx = (*poly)[j].X - centroid.X, by = (*poly)[j].Y - centroid.Y;
        double ex = bx - ax, ey = by - ay;
        double denom = sin_t * ex - cos_t * ey;
        if (std::abs(denom) < 1e-6) continue;
        double t_val = (ay * ex - ax * ey) / denom;
        double s_val = (ay * cos_t - ax * sin_t) / denom;
        if (t_val > 1e-6 && s_val >= -1e-9 && s_val <= 1.0 + 1e-9)
            return t_val;
    }
    // Fallback: return Euclidean distance from centroid to nearest endpoint
    const Point2LL& fp = is_open ? (*poly)[n - 1] : (*poly)[0];
    double fx = fp.X - centroid.X, fy = fp.Y - centroid.Y;
    return std::sqrt(fx * fx + fy * fy);
}

Point2LL FeatherPrintGenerator::conformPlace(
    double lx, double ly, double x_sign,
    double theta_anchor, double R_a,
    const Point2LL& centroid, const ArcParam& arc, coord_t w)
{
    double theta = theta_anchor + x_sign * lx * w / R_a;
    double r     = arc.radiusAt(centroid, theta) + ly * w; // ly < 0 = inward
    return Point2LL(
        centroid.X + static_cast<coord_t>(r * std::cos(theta)),
        centroid.Y + static_cast<coord_t>(r * std::sin(theta)));
}

void FeatherPrintGenerator::appendCanonicalArc(
    ExtrusionLine& line,
    double cx, double cy, double R,
    double a_start_deg, double a_end_deg,
    bool cw_arc, int segs, double x_sign,
    double theta_anchor, double R_a,
    const Point2LL& centroid, const ArcParam& arc,
    coord_t w, bool skip_first)
{
    const double deg2rad = std::numbers::pi / 180.0;
    double a_start = a_start_deg * deg2rad;
    double a_end   = a_end_deg   * deg2rad;

    double span;
    if (cw_arc)
    {
        span = a_start - a_end;
        while (span < 0.0)                    span += 2.0 * std::numbers::pi;
        while (span > 2.0 * std::numbers::pi) span -= 2.0 * std::numbers::pi;
    }
    else
    {
        span = a_end - a_start;
        while (span < 0.0)                    span += 2.0 * std::numbers::pi;
        while (span > 2.0 * std::numbers::pi) span -= 2.0 * std::numbers::pi;
    }

    const int start_i = skip_first ? 1 : 0;
    for (int i = start_i; i <= segs; i++)
    {
        double t     = static_cast<double>(i) / segs;
        double angle = cw_arc ? (a_start - t * span) : (a_start + t * span);
        double lx    = cx + R * std::cos(angle);
        double ly    = cy + R * std::sin(angle);
        line.junctions_.emplace_back(conformPlace(lx, ly, x_sign, theta_anchor, R_a, centroid, arc, w), w, 0);
    }
}

// ============================================================================
// Flange — multi-wall closed perimeter for open-top boundary loops
// ============================================================================
//
// Wall stack (ramp_index = 0 at bottom of Flange, n-1 at brim):
//   total_w = 1.0 + (ramp_index + 1) * 0.5 (in units of w)
//   has_half = (ramp_index % 2 == 0)
//
//   ramp 0 (1.5w): [outer half w/2 @ +0.25w] [inner full w @ +1.0w]
//   ramp 1 (2.0w): [outer full w @ +0.5w] [inner full w @ +1.5w]
//   ramp 2 (2.5w): [outer full w @ +0.5w] [buried half w/2 @ +1.25w] [inner full w @ +2.0w]
//   ramp 3 (3.0w): [outer full w @ +0.5w] [mid full w @ +1.5w] [inner full w @ +2.5w]
//   ...
//
// "offset" = distance from OML to wall centreline (inward positive).
// Walls are emitted innermost-first (printed first); outermost wall carries inset_idx=0 (printed last).

double FeatherPrintGenerator::arcLengthAtAngle(const ArcParam& arc, const Point2LL& centroid, double theta)
{
    const double cos_t = std::cos(theta), sin_t = std::sin(theta);
    const int n = static_cast<int>(arc.poly->size());
    for (int i = 0; i < n; i++)
    {
        int j = (i + 1) % n;
        if (arc.is_open && j == 0) continue;
        double ax = (*arc.poly)[i].X - centroid.X, ay = (*arc.poly)[i].Y - centroid.Y;
        double bx = (*arc.poly)[j].X - centroid.X, by = (*arc.poly)[j].Y - centroid.Y;
        double ex = bx - ax, ey = by - ay;
        double denom = sin_t * ex - cos_t * ey;
        if (std::abs(denom) < 1e-6) continue;
        double t_val = (ay * ex - ax * ey) / denom;
        double s_val = (ay * cos_t - ax * sin_t) / denom;
        if (t_val > 1e-6 && s_val >= -1e-9 && s_val <= 1.0 + 1e-9)
        {
            double seg_end = (j == 0) ? arc.total : arc.cum_len[j];
            return arc.cum_len[i] + s_val * (seg_end - arc.cum_len[i]);
        }
    }
    return -1.0;
}

std::vector<FeatherPrintGenerator::FlangeWallDesc> FeatherPrintGenerator::buildFlangeWallStack(int ramp_index, coord_t w)
{
    // Build ordered list of wall centreline offsets and widths, outer→inner.
    std::vector<FlangeWallDesc> walls_outer_first;

    const bool has_half  = (ramp_index % 2 == 0);
    const int  n_full    = 1 + (ramp_index + 1) / 2; // number of full-width walls

    if (ramp_index == 0)
    {
        // Special: half-width outer wall, full-width inner wall.
        // Full-width walls (all other ramp layers) trace at offset=0 so their outer face is
        // w/2 outside the OML polygon — matching generate()'s skin. To match that same outer
        // face for the half-width wall, expand the polygon outward by w/4 (offset = -w/4) so
        // the half-wall's outer face is also at w/2 outside OML.
        // Outer half: expanded polygon (inner face lands at OML polygon = offset 0).
        // Inner full: centre at w/2 inward from OML polygon.
        walls_outer_first.push_back({ -w / 4, w / 2 }); // outer (half): expand outward w/4
        walls_outer_first.push_back({  w / 2, w     }); // inner (full): centre w/2 in from OML
    }
    else
    {
        // Outer full-width wall traced on OML polygon (offset=0), matching generate().
        // Its inner face sits at w/2; subsequent walls are measured from there.
        walls_outer_first.push_back({ 0, w });
        coord_t cursor = w / 2; // inner face of outer wall

        if (has_half)
        {
            // Half-width wall buried just inside the outer wall.
            walls_outer_first.push_back({ cursor + w / 4, w / 2 });
            cursor += w / 2;
        }

        // Remaining full-width walls.
        for (int fi = 1; fi < n_full; fi++)
        {
            walls_outer_first.push_back({ cursor + w / 2, w });
            cursor += w;
        }
    }
    return walls_outer_first;
}

int FeatherPrintGenerator::flangePrintInsetIdx(int wi, int n_walls)
{
    if (wi == 0) return 0;               // OML: always last
    if (wi == n_walls - 1) return 1;     // IML: second-to-last (not first)
    return wi + 1;                       // buried middle Walls: earliest, order among themselves doesn't matter
}

OpenPolyline FeatherPrintGenerator::radialOffsetOpen(const OpenPolyline& poly, const Point2LL& centroid, coord_t offset)
{
    OpenPolyline result;
    for (const Point2LL& p : poly)
    {
        double dx = static_cast<double>(p.X - centroid.X);
        double dy = static_cast<double>(p.Y - centroid.Y);
        double R = std::sqrt(dx * dx + dy * dy);
        if (R < 1.0)
        {
            result.push_back(p);
            continue;
        }
        double scale = (R - static_cast<double>(offset)) / R;
        result.push_back(Point2LL(
            centroid.X + static_cast<coord_t>(dx * scale),
            centroid.Y + static_cast<coord_t>(dy * scale)));
    }
    return result;
}

VariableWidthLines FeatherPrintGenerator::generateFlange(const Shape& outline, const Settings& settings, int ramp_index, double helix_phase)
{
    const coord_t w = settings.get<coord_t>("featherprint_line_width");

    std::vector<FlangeWallDesc> walls_outer_first = buildFlangeWallStack(ramp_index, w);

    // ---- Compute Stringer/Lacing anchor positions on OML for Flare Rim insertion ----
    const Polygon* oml_poly = largestPoly(outline);
    const int N = settings.get<int>("featherprint_stringer_count");

    // theta/R_oml are evaluated on the OML (per spec, the Flare Rim is anchored there, not on
    // the innermost wall); s_left/s_right cut the innermost wall's own perimeter walk at the
    // exact theta the Flare Rim's own two endpoints land on (not an arc-length approximation),
    // so the ordinary-wall walk and the Flare Rim's first/last emitted points coincide.
    struct FlareAnchor { double s_left; double s_right; double theta; double R_oml; double x_sign; double W; };
    std::vector<FlareAnchor> flare_anchors;

    // Q per the Flare Rim spec is toolpath-CENTRELINE-to-toolpath-centreline (the outer
    // wall's own toolpath centre to the innermost wall's toolpath centre) — NOT the full
    // physical OML-face-to-IML-face span. Compute it directly from the actual constructed
    // wall offsets rather than the (1.5 + 0.5*ramp_index) physical-span formula, since the
    // outer wall is only full-width (offset=0) from ramp 1 onward — at ramp 0 it's a
    // half-width wall offset outward by w/4, so a constant correction is wrong there.
    const FlangeWallDesc& wd_outer_layer = walls_outer_first.front();
    const FlangeWallDesc& wd_inner_layer = walls_outer_first.back();
    const double Q_true = static_cast<double>(wd_inner_layer.offset - wd_outer_layer.offset) / static_cast<double>(w);
    // arc_oml/oml_poly is the raw slice polygon (ly=0 reference), which coincides with the
    // outer wall's own toolpath centreline only when its offset is 0 (true for ramp>=1, not
    // ramp 0). This is the gap to add back when placing canonical y-values (which are
    // defined relative to the true OML/outer-wall centreline) into arc_oml's own frame.
    const double oml_shift = -static_cast<double>(wd_outer_layer.offset) / static_cast<double>(w);
    // Flare Rim half-width in canonical lx (matches appendFlareRim's r = (2.5-Q_true)/2; the
    // profile's widest points, Arc1/Arc4's departure points, sit at lx = ±(W/2 + r)).
    const double flare_r = std::max(0.0, (2.5 - Q_true) / 2.0);

    if (oml_poly && oml_poly->size() >= 3 && N >= 1)
    {
        ArcParam arc_oml = buildArcParam(*oml_poly);
        const Point2LL centroid = centroidBbox(*oml_poly);
        const double w_d = static_cast<double>(w);
        const double helix_frac  = std::fmod(helix_phase, 1.0);
        const double arc_ref     = arc_oml.referenceArcPos(centroid);
        const double ccw_advance = std::fmod(helix_frac * arc_oml.total + arc_ref, arc_oml.total);
        const double cw_advance  = std::fmod(arc_ref - helix_frac * arc_oml.total + arc_oml.total, arc_oml.total);

        // OML anchor positions, CCW+CW interleaved and sorted by arc position so adjacent
        // opposite-direction anchors can be tested for a Lacing collision (same pattern as
        // the main generate() collision pass).
        struct OmlAnchor { double s; bool is_cw; bool skip{ false }; int pair_idx{ -1 }; };
        std::vector<OmlAnchor> oml_anchors;
        oml_anchors.reserve(2 * N);
        for (int i = 0; i < N; i++)
            oml_anchors.push_back({std::fmod(static_cast<double>(i) / N * arc_oml.total + ccw_advance, arc_oml.total), false});
        for (int i = 0; i < N; i++)
            oml_anchors.push_back({std::fmod(static_cast<double>(i) / N * arc_oml.total + cw_advance,  arc_oml.total), true });
        std::sort(oml_anchors.begin(), oml_anchors.end(), [](const OmlAnchor& a, const OmlAnchor& b){ return a.s < b.s; });

        // Two Stringer anchors must merge into one Lacing Rim once their *widened* Flare Rim
        // footprints (±(1+flare_r) each, wider than the plain 1w Stringer Trace cutout) would
        // physically overlap — not just when the plain-Trace threshold (w) is crossed. Using
        // the unwidened threshold here left near-tip layers (small circumference, closely
        // packed anchors) with two separate, overlapping/crossing Flare Rims instead of one
        // merged Lacing Rim.
        const double collision_w = (2.0 + 2.0 * flare_r) * w_d;

        const int total_anchors = static_cast<int>(oml_anchors.size());
        for (int i = 0; i < total_anchors; i++)
        {
            for (int j = i + 1; j < total_anchors; j++)
            {
                if (oml_anchors[j].s - oml_anchors[i].s > 2.0 * collision_w) break;
                if (oml_anchors[j].is_cw == oml_anchors[i].is_cw) continue;
                Point2LL pi = arc_oml.pointAt(oml_anchors[i].s);
                Point2LL pj = arc_oml.pointAt(oml_anchors[j].s);
                double dx = static_cast<double>(pi.X - pj.X), dy = static_cast<double>(pi.Y - pj.Y);
                if (std::sqrt(dx * dx + dy * dy) < collision_w)
                {
                    oml_anchors[i].skip = oml_anchors[j].skip = true;
                    oml_anchors[i].pair_idx = j;
                    oml_anchors[j].pair_idx = i;
                }
            }
        }

        // Compute innermost wall offset shape so we can project anchors onto it (for the
        // wall-walk cutout only — the Flare Rim's own geometry is anchored on the OML).
        const FlangeWallDesc& wd_inner = walls_outer_first.back();
        Shape inner_shape = Shape(outline).offset(-wd_inner.offset);
        const Polygon* inner_poly = largestPoly(inner_shape);
        if (inner_poly && inner_poly->size() >= 3)
        {
            ArcParam arc_inner = buildArcParam(*inner_poly);

            auto addAnchor = [&](double s_oml, double x_sign, double W)
            {
                Point2LL pt = arc_oml.pointAt(s_oml);
                double dx = static_cast<double>(pt.X - centroid.X);
                double dy = static_cast<double>(pt.Y - centroid.Y);
                double theta = std::atan2(dy, dx);
                double R_oml = std::sqrt(dx * dx + dy * dy);
                if (R_oml < 1.0)
                    return;
                // Project the Flare Rim's actual left/right endpoints (lx = ∓(W/2+flare_r),
                // per conformPlace's angular formula) onto the innermost wall, not just the
                // anchor's own theta — this is what appendFlareRim will itself emit as its
                // first/last points, so the ordinary-wall walk must stop exactly there.
                const double lx_max = W / 2.0 + flare_r;
                const double theta_l = theta - lx_max * static_cast<double>(w) / R_oml;
                const double theta_r = theta + lx_max * static_cast<double>(w) / R_oml;
                double s_l = arcLengthAtAngle(arc_inner, centroid, theta_l);
                double s_r = arcLengthAtAngle(arc_inner, centroid, theta_r);
                if (s_l < 0.0 || s_r < 0.0)
                    return; // anchor doesn't project onto the innermost wall at all
                flare_anchors.push_back({s_l, s_r, theta, R_oml, x_sign, W});
            };

            for (int i = 0; i < total_anchors; i++)
            {
                const OmlAnchor& oa = oml_anchors[i];
                if (oa.skip)
                {
                    if (oa.pair_idx > i)
                    {
                        // Lacing anchor: midpoint between the two colliding Stringer anchors,
                        // symmetric profile (x_sign=+1), W = 3w per spec.
                        double s_mid = (oa.s + oml_anchors[oa.pair_idx].s) * 0.5;
                        addAnchor(s_mid, 1.0, 3.0);
                    }
                    continue;
                }
                // Ordinary Stringer anchor: the Flare Rim's dovetail profile is left-right
                // symmetric (unlike the ordinary teardrop Trace), so it needs no CW/CCW
                // mirror — x_sign=-1 would instead reverse the low-s->high-s traversal
                // direction the wall-walk cutout (s_left/s_right) assumes, stitching it in
                // backwards for CW anchors. W = 2w.
                addAnchor(oa.s, 1.0, 2.0);
            }

            std::sort(flare_anchors.begin(), flare_anchors.end(),
                [](const FlareAnchor& a, const FlareAnchor& b){ return a.s_left < b.s_left; });
        }
    }

    // ---- Emit walls: innermost first (inset_idx = n_walls-1), outermost last (inset_idx = 0) ----
    const int n_walls = static_cast<int>(walls_outer_first.size());
    VariableWidthLines result;

    for (int wi = n_walls - 1; wi >= 0; wi--) // inner→outer emission order
    {
        const FlangeWallDesc& wd      = walls_outer_first[wi];
        const int       inset_idx = flangePrintInsetIdx(wi, n_walls);
        const bool      is_innermost = (wi == n_walls - 1);

        Shape offset_shape = Shape(outline).offset(-wd.offset);
        if (offset_shape.empty()) continue;
        const Polygon* poly = largestPoly(offset_shape);
        if (! poly || poly->size() < 3) continue;

        ArcParam ap = buildArcParam(*poly);
        if (ap.total < 1.0) continue;

        ExtrusionLine wall_line(inset_idx, /*is_odd=*/false, /*is_closed=*/true);

        if (is_innermost && ! flare_anchors.empty())
        {
            // Flare Rim: innermost-wall perimeter walk with a Flare Rim channel embedded at
            // each Stringer/Lacing anchor. The channel geometry itself is anchored on the OML
            // (arc_oml recomputed below) per spec, not on this offset wall's own surface.
            ArcParam arc_oml = buildArcParam(*oml_poly);
            const Point2LL centroid = centroidBbox(*oml_poly);
            double current_s = 0.0;
            for (const FlareAnchor& fa : flare_anchors)
            {
                double s_depart = fa.s_left;
                if (s_depart < current_s) s_depart = current_s;
                appendPolySegment(wall_line, ap, current_s, s_depart, wd.width, wall_line.empty());

                // appendPolySegment above always ends by emitting the true wall-polygon point
                // at s_depart (unconditionally, regardless of add_start), so the Flare Rim's
                // own analytically-computed first point must always be skipped — using
                // wall_line.empty() here (now false, since the call above already pushed
                // points) skipped nothing and left a duplicate/jog at every rim's entry.
                appendFlareRim(wall_line, fa.theta, fa.R_oml, fa.x_sign, centroid, arc_oml,
                               Q_true, oml_shift, fa.W, w, /*skip_first=*/true);

                current_s = std::max(fa.s_right, s_depart);
            }
            appendPolySegment(wall_line, ap, current_s, ap.total, wd.width, wall_line.empty());
        }
        else
        {
            appendPolySegment(wall_line, ap, 0.0, ap.total, wd.width, /*add_start=*/true);
        }

        if (! wall_line.empty())
            wall_line.junctions_.push_back(wall_line.junctions_.front());
        if (wall_line.size() >= 2)
            result.push_back(std::move(wall_line));
    }

    return result;
}

// ============================================================================
// Flange on an interrupted boundary loop (Miter case)
// ============================================================================
//
// The outer Wall's Terminal and the inner Walls' open ends interact per the Miter rule; the
// innermost Wall additionally carries Flare Rim insertion wherever a Stringer/Lacing anchor
// falls within this arc's span, mirroring the closed generateFlange() case but working in
// this arc's own (linear, non-wrapping) arc-length parameterization.
VariableWidthLines FeatherPrintGenerator::generateFlangeOpen(
    const OpenPolyline& open_poly, coord_t z, const Settings& settings,
    int ramp_index, double helix_phase, const OpenLayerParams& params)
{
    (void)z;
    if (open_poly.size() < 2)
        return {};

    const coord_t w = settings.get<coord_t>("featherprint_line_width");
    if (w <= 0)
        return {};

    const int N = settings.get<int>("featherprint_stringer_count");
    const double w_d = static_cast<double>(w);

    std::vector<FlangeWallDesc> walls_outer_first = buildFlangeWallStack(ramp_index, w);
    const Point2LL& centroid = params.centroid;

    const FlangeWallDesc& wd_outer_layer = walls_outer_first.front();
    const FlangeWallDesc& wd_inner_layer = walls_outer_first.back();
    // Same Q_true/oml_shift convention as generateFlange(): Q_true is the toolpath-centreline-
    // to-toolpath-centreline OML-to-IML distance; oml_shift corrects for open_poly (the ly=0
    // reference) not coinciding with the outer Wall's own centreline at ramp 0.
    const double Q_true = static_cast<double>(wd_inner_layer.offset - wd_outer_layer.offset) / w_d;
    const double oml_shift = -static_cast<double>(wd_outer_layer.offset) / w_d;
    const double flare_r = std::max(0.0, (2.5 - Q_true) / 2.0);

    // ---- Compute Stringer/Lacing anchors within this arc, for Flare Rim insertion ----
    ArcParam arc_oml = buildArcParamOpen(open_poly);
    const double s_end_oml = arc_oml.total;

    struct FlareAnchor { double s_left; double s_right; double theta; double R_oml; double x_sign; double W; };
    std::vector<FlareAnchor> flare_anchors;

    if (N >= 1 && s_end_oml > 2.0 * w_d)
    {
        const double helix_frac  = std::fmod(helix_phase, 1.0);
        const double ccw_adv_abs = std::fmod(helix_frac * params.full_ring_total + params.full_ring_arc_ref, params.full_ring_total);
        const double cw_adv_abs  = std::fmod(params.full_ring_arc_ref - helix_frac * params.full_ring_total + params.full_ring_total, params.full_ring_total);

        // OML anchor positions within this arc (same full-ring-to-local projection as
        // generateOpen()'s Stringer anchors), sorted so adjacent opposite-direction anchors
        // can be tested for a Lacing collision.
        struct OmlAnchor { double s; bool is_cw; bool skip{ false }; int pair_idx{ -1 }; };
        std::vector<OmlAnchor> oml_anchors;
        oml_anchors.reserve(2 * N);
        for (int i = 0; i < N; i++)
        {
            double s_abs = std::fmod(static_cast<double>(i) / N * params.full_ring_total + ccw_adv_abs, params.full_ring_total);
            double s_local = s_abs - params.arc_start_in_ring;
            if (s_local > w_d && s_local < s_end_oml - w_d)
                oml_anchors.push_back({ s_local, false });
        }
        for (int i = 0; i < N; i++)
        {
            double s_abs = std::fmod(static_cast<double>(i) / N * params.full_ring_total + cw_adv_abs, params.full_ring_total);
            double s_local = s_abs - params.arc_start_in_ring;
            if (s_local > w_d && s_local < s_end_oml - w_d)
                oml_anchors.push_back({ s_local, true });
        }
        std::sort(oml_anchors.begin(), oml_anchors.end(), [](const OmlAnchor& a, const OmlAnchor& b){ return a.s < b.s; });

        const double collision_w = (2.0 + 2.0 * flare_r) * w_d;
        const int total_anchors = static_cast<int>(oml_anchors.size());
        for (int i = 0; i < total_anchors; i++)
        {
            for (int j = i + 1; j < total_anchors; j++)
            {
                if (oml_anchors[j].s - oml_anchors[i].s > 2.0 * collision_w) break;
                if (oml_anchors[j].is_cw == oml_anchors[i].is_cw) continue;
                Point2LL pi = arc_oml.pointAt(oml_anchors[i].s);
                Point2LL pj = arc_oml.pointAt(oml_anchors[j].s);
                double dx = static_cast<double>(pi.X - pj.X), dy = static_cast<double>(pi.Y - pj.Y);
                if (std::sqrt(dx * dx + dy * dy) < collision_w)
                {
                    oml_anchors[i].skip = oml_anchors[j].skip = true;
                    oml_anchors[i].pair_idx = j;
                    oml_anchors[j].pair_idx = i;
                }
            }
        }

        OpenPolyline inner_poly = radialOffsetOpen(open_poly, centroid, wd_inner_layer.offset);
        if (inner_poly.size() >= 2)
        {
            ArcParam arc_inner = buildArcParamOpen(inner_poly);

            auto addAnchor = [&](double s_oml, double x_sign, double W)
            {
                Point2LL pt = arc_oml.pointAt(s_oml);
                double dx = static_cast<double>(pt.X - centroid.X);
                double dy = static_cast<double>(pt.Y - centroid.Y);
                double theta = std::atan2(dy, dx);
                double R_oml = std::sqrt(dx * dx + dy * dy);
                if (R_oml < 1.0) return;
                const double lx_max = W / 2.0 + flare_r;
                const double theta_l = theta - lx_max * w_d / R_oml;
                const double theta_r = theta + lx_max * w_d / R_oml;
                double s_l = arcLengthAtAngle(arc_inner, centroid, theta_l);
                double s_r = arcLengthAtAngle(arc_inner, centroid, theta_r);
                if (s_l < 0.0 || s_r < 0.0)
                    return; // anchor doesn't project onto the innermost wall's open span
                flare_anchors.push_back({ s_l, s_r, theta, R_oml, x_sign, W });
            };

            for (int i = 0; i < total_anchors; i++)
            {
                const OmlAnchor& oa = oml_anchors[i];
                if (oa.skip)
                {
                    if (oa.pair_idx > i)
                    {
                        double s_mid = (oa.s + oml_anchors[oa.pair_idx].s) * 0.5;
                        addAnchor(s_mid, 1.0, 3.0); // Lacing anchor, W=3w
                    }
                    continue;
                }
                addAnchor(oa.s, 1.0, 2.0); // ordinary Stringer anchor, W=2w (symmetric profile, no mirror needed)
            }
            std::sort(flare_anchors.begin(), flare_anchors.end(),
                [](const FlareAnchor& a, const FlareAnchor& b){ return a.s_left < b.s_left; });
        }
    }

    const int n_walls = static_cast<int>(walls_outer_first.size());
    VariableWidthLines result;

    for (int wi = n_walls - 1; wi >= 0; wi--) // inner→outer emission order, matching generateFlange
    {
        const FlangeWallDesc& wd = walls_outer_first[wi];
        const bool is_outer = (wi == 0);
        const bool is_innermost = (wi == n_walls - 1);

        OpenPolyline offset_poly = radialOffsetOpen(open_poly, centroid, wd.offset);
        if (offset_poly.size() < 2) continue;

        ArcParam arc_w = buildArcParamOpen(offset_poly);
        const double s_end = arc_w.total;
        if (s_end < 2.0 * w_d) continue;

        ExtrusionLine wall_line(flangePrintInsetIdx(wi, n_walls), /*is_odd=*/false, /*is_closed=*/false);

        if (is_outer)
        {
            // Outer Wall: ordinary Whip Terminal at both ends, continuing the same Terminal
            // column as ordinary Whip layers below the Flange (Miter rule).
            appendTerminal(wall_line, 0.0, +1.0, arc_w, centroid, wd.width, /*reversed=*/true);
            appendPolySegment(wall_line, arc_w, 2.0 * w_d, s_end - 1.5 * w_d, wd.width, wall_line.empty());
            appendTerminal(wall_line, s_end, -1.0, arc_w, centroid, wd.width, /*reversed=*/false);
        }
        else if (is_innermost && ! flare_anchors.empty())
        {
            // Innermost Wall: left open at both ends per the Miter rule, with a Flare Rim
            // channel embedded at each Stringer/Lacing anchor along the way.
            double current_s = 0.0;
            for (const FlareAnchor& fa : flare_anchors)
            {
                double s_depart = std::min(std::max(fa.s_left, current_s), s_end);
                appendPolySegment(wall_line, arc_w, current_s, s_depart, wd.width, wall_line.empty());

                appendFlareRim(wall_line, fa.theta, fa.R_oml, fa.x_sign, centroid, arc_oml,
                               Q_true, oml_shift, fa.W, w, /*skip_first=*/true);

                current_s = std::min(std::max(fa.s_right, s_depart), s_end);
            }
            appendPolySegment(wall_line, arc_w, current_s, s_end, wd.width, wall_line.empty());
        }
        else
        {
            // Buried middle Walls: left open at both ends per the Miter rule, no Flare Rim.
            appendPolySegment(wall_line, arc_w, 0.0, s_end, wd.width, /*add_start=*/true);
        }

        if (wall_line.size() >= 2)
            result.push_back(std::move(wall_line));
    }

    return result;
}

// ============================================================================
// Stringer Trace
// ============================================================================
//
// Canonical stringer trace (w-units, anchor at origin, y negative = inward):
//   Departure  : (−0.5, 0)   Return : (+0.5, 0)   Bottom : (0, −2.5)
//   Arc 1: CW, centre (−0.5, −1.5), R=1.5, 90°→  0°   → (−0.5,0) to ( 1.0,−1.5)
//   Arc 2: CW, centre (  0,  −1.5), R=1.0,  0°→180°   → ( 1.0,−1.5) to (−1.0,−1.5) via (0,−2.5)
//   Arc 3: CW, centre (+0.5, −1.5), R=1.5, 180°→ 90°  → (−1.0,−1.5) to ( 0.5, 0)
//
// For CW helix traces x_sign=-1 mirrors about x=0, arcs drawn in reverse order/direction.

void FeatherPrintGenerator::appendTrace(
    ExtrusionLine& line,
    double theta_anchor, double R_a,
    const Point2LL& centroid, const ArcParam& arc,
    bool is_cw, coord_t w, bool skip_first)
{
    if (! is_cw)
    {
        // CCW helix: departure at lx=−0.5, return at lx=+0.5.
        appendCanonicalArc(line, -0.5, -1.5, 1.5,  90.0,   0.0, true,  8,  1.0, theta_anchor, R_a, centroid, arc, w, skip_first);
        appendCanonicalArc(line,  0.0, -1.5, 1.0,   0.0, 180.0, true,  10, 1.0, theta_anchor, R_a, centroid, arc, w, true);
        appendCanonicalArc(line,  0.5, -1.5, 1.5, 180.0,  90.0, true,  8,  1.0, theta_anchor, R_a, centroid, arc, w, true);
    }
    else
    {
        // CW helix: arcs reversed in order and direction, x_sign=-1 mirrors about x=0.
        // Departure lx=+0.5 (canon 0.5 × −1 = −0.5 in world → +0.5 on arc), return lx=−0.5.
        appendCanonicalArc(line,  0.5, -1.5, 1.5,  90.0, 180.0, false, 8,  -1.0, theta_anchor, R_a, centroid, arc, w, skip_first);
        appendCanonicalArc(line,  0.0, -1.5, 1.0, 180.0, 360.0, false, 10, -1.0, theta_anchor, R_a, centroid, arc, w, true);
        appendCanonicalArc(line, -0.5, -1.5, 1.5,   0.0,  90.0, false, 8,  -1.0, theta_anchor, R_a, centroid, arc, w, true);
    }
}

// ============================================================================
// Lacing Trace
// ============================================================================
//
// Canonical S-link profile (w-units, anchor at midpoint between two colliding stringers):
//   Departure  : (-0.75,  0)    Return : (+0.75, 0)
//   Left inner arc  : centre (-0.75, -0.50), R=0.5,  CW  90°→ -90°  (inner eye, opens toward centre)
//   Left outer arc  : centre (-0.75, -1.75), R=0.75, CCW 90°→ 270°  (outer loop, swings left to x=-1.5)
//   Right outer arc : centre (+0.75, -1.75), R=0.75, CCW -90°→  90°  (outer loop, swings right to x=+1.5)
//   Right inner arc : centre (+0.75, -0.50), R=0.5,  CW  -90°→  90° (inner eye, opens toward centre)
//   Arcs chain directly; only bottom horizontal connects at y=-2.5.

void FeatherPrintGenerator::appendLacingTrace(
    ExtrusionLine& line,
    double theta_anchor, double R_a,
    const Point2LL& centroid, const ArcParam& arc,
    coord_t w)
{
    auto pt = [&](double lx, double ly) {
        line.junctions_.emplace_back(conformPlace(lx, ly, 1.0, theta_anchor, R_a, centroid, arc, w), w, 0);
    };

    // Left inner arc: CW 90°→-90°, centre (-0.75,-0.50), R=0.5
    // Departure at (-0.75, 0); arc swings right through (-0.25,-0.50) to (-0.75,-1.0)
    appendCanonicalArc(line, -0.75, -0.5, 0.5, 90.0, -90.0, true, 4, 1.0, theta_anchor, R_a, centroid, arc, w, false);

    // Left outer arc: CCW 90°→270°, centre (-0.75,-1.75), R=0.75
    // Chains directly from inner arc end (-0.75,-1.0); swings left through (-1.50,-1.75) to (-0.75,-2.5)
    appendCanonicalArc(line, -0.75, -1.75, 0.75, 90.0, 270.0, false, 6, 1.0, theta_anchor, R_a, centroid, arc, w, true);

    // Horizontal bottom: (-0.75,-2.5) → (0,-2.5)
    pt(0.0, -2.5);

    // Right outer arc: CCW -90°→90°, centre (+0.75,-1.75), R=0.75
    // skip_first=false to emit start (+0.75,-2.5) and complete bottom horizontal
    // Swings right through (+1.50,-1.75) to (+0.75,-1.0)
    appendCanonicalArc(line, 0.75, -1.75, 0.75, -90.0, 90.0, false, 6, 1.0, theta_anchor, R_a, centroid, arc, w, false);

    // Right inner arc: CW -90°→90°, centre (+0.75,-0.50), R=0.5
    // Chains directly from outer arc end (+0.75,-1.0); swings left through (+0.25,-0.50) to (+0.75, 0)
    appendCanonicalArc(line, 0.75, -0.5, 0.5, -90.0, 90.0, true, 4, 1.0, theta_anchor, R_a, centroid, arc, w, true);
}

// ============================================================================
// Flare Rim (Stringer/Lacing × Flange)
// ============================================================================
//
// Anchored at (0,0) on the true OML (the outer wall's own toolpath centreline — NOT
// oml_arc's own ly=0 reference, and not the innermost Flange wall's surface). T is Q: the
// toolpath-centreline-to-toolpath-centreline OML-to-IML distance, used (unshifted) for the
// fillet radius r = (2.5-Q)/2, so the channel bottom stays pinned at a fixed 2.5w below the
// true OML regardless of Q. oml_shift corrects only the placement (every canonical y-value
// is applied at y-oml_shift in oml_arc's own frame) — it must not feed into r, since r is
// defined relative to the true OML, not oml_arc's frame. d is the width of the colliding
// feature (2.0 for a Stringer anchor, 3.0 for a Lacing anchor). Mirror-symmetric dovetail
// cross-section.
//
// Traversed left-to-right (increasing lx) so it matches the wall's own increasing-arc-length
// walk direction — the caller stops the ordinary wall walk at s_left (the lx=-(d/2+r) end)
// and resumes it at s_right (the lx=+(d/2+r) end); emitting the arcs in the other order
// stitched this rim in backwards and crossed the incoming/outgoing wall segments:
//   Arc 1 CW  (-d/2-r,-Q)  -> (-d/2,-Q-r)     centre (-d/2-r,-Q-r)
//   Arc 2 CCW (-d/2,-Q-r)  -> (-d/2+r,-Q-2r)  centre (-d/2+r,-Q-r)
//   Line 1    (-d/2+r,-Q-2r) -> (d/2-r,-Q-2r)
//   Arc 3 CCW (d/2-r,-Q-2r) -> (d/2,-Q-r)     centre (d/2-r,-Q-r)
//   Arc 4 CW  (d/2,-Q-r)   -> (d/2+r,-Q)      centre (d/2+r,-Q-r)
void FeatherPrintGenerator::appendFlareRim(
    ExtrusionLine& line,
    double theta_anchor, double R_a, double x_sign,
    const Point2LL& centroid, const ArcParam& oml_arc,
    double T, double oml_shift, double W, coord_t w, bool skip_first)
{
    const double Q = T;
    const double d = W;
    const double r = std::max(0.0, (2.5 - Q) / 2.0);
    const double Qp = Q - oml_shift; // Q, translated into oml_arc's own ly=0 frame

    auto pt = [&](double lx, double ly) {
        line.junctions_.emplace_back(conformPlace(lx, ly, x_sign, theta_anchor, R_a, centroid, oml_arc, w), w, 0);
    };

    // Arc 1: CW, (-d/2-r,-Qp) -> (-d/2,-Qp-r), centre (-d/2-r,-Qp-r)
    appendCanonicalArc(line, -d / 2.0 - r, -Qp - r, r, 90.0, 0.0, true, 4, x_sign,
                       theta_anchor, R_a, centroid, oml_arc, w, skip_first);
    // Arc 2: CCW, (-d/2,-Qp-r) -> (-d/2+r,-Qp-2r), centre (-d/2+r,-Qp-r)
    appendCanonicalArc(line, -d / 2.0 + r, -Qp - r, r, 180.0, 270.0, false, 4, x_sign,
                       theta_anchor, R_a, centroid, oml_arc, w, /*skip_first=*/true);
    // Line 1: (-d/2+r,-Qp-2r) -> (d/2-r,-Qp-2r)
    pt(d / 2.0 - r, -Qp - 2.0 * r);
    // Arc 3: CCW, (d/2-r,-Qp-2r) -> (d/2,-Qp-r), centre (d/2-r,-Qp-r)
    appendCanonicalArc(line, d / 2.0 - r, -Qp - r, r, -90.0, 0.0, false, 4, x_sign,
                       theta_anchor, R_a, centroid, oml_arc, w, /*skip_first=*/true);
    // Arc 4: CW, (d/2,-Qp-r) -> (d/2+r,-Qp), centre (d/2+r,-Qp-r)
    appendCanonicalArc(line, d / 2.0 + r, -Qp - r, r, 180.0, 90.0, true, 4, x_sign,
                       theta_anchor, R_a, centroid, oml_arc, w, /*skip_first=*/true);
}

// ============================================================================
// Main generate
// ============================================================================

VariableWidthLines FeatherPrintGenerator::generate(
    const Shape& outline,
    coord_t z,
    const Settings& settings,
    double helix_phase)
{
    seam_pt_ = Point2LL(0, 0);

    const Polygon* outer = largestPoly(outline);
    if (! outer || outer->size() < 3)
        return {};

    const coord_t w = settings.get<coord_t>("featherprint_line_width");
    if (w <= 0)
        return {};

    const int N = settings.get<int>("featherprint_stringer_count");
    if (N < 1)
        return {};

    ArcParam arc = buildArcParam(*outer);
    if (arc.total < 4.0 * w)
        return {};

    const Point2LL centroid = centroidBbox(*outer);
    // helix_phase is the running integral Σ(dz/arc_total) from the pre-pass,
    // giving constant intersection angle across the full height of a tapered tube.
    const double helix_frac = std::fmod(helix_phase, 1.0);
    const double arc_ref    = arc.referenceArcPos(centroid);
    // CCW helix advances with z; CW helix retreats — geodesic interlocking pair.
    const double ccw_advance = std::fmod(helix_frac * arc.total + arc_ref, arc.total);
    const double cw_advance  = std::fmod(arc_ref - helix_frac * arc.total + arc.total, arc.total);
    const double w_d         = static_cast<double>(w);

    std::vector<Anchor> anchors;
    anchors.reserve(2 * N);
    for (int i = 0; i < N; i++)
        anchors.push_back({ std::fmod(static_cast<double>(i) / N * arc.total + ccw_advance, arc.total), false });
    for (int i = 0; i < N; i++)
        anchors.push_back({ std::fmod(static_cast<double>(i) / N * arc.total + cw_advance,  arc.total), true  });
    std::sort(anchors.begin(), anchors.end(), [](const Anchor& a, const Anchor& b) { return a.s < b.s; });

    const int total_anchors = static_cast<int>(anchors.size());

    // Cross-helix collision detection: if a CCW and CW anchor are within w
    // world-space distance they will intersect — mark both for lacing.
    // Currently: skip the traces to confirm detection visually.
    for (int i = 0; i < total_anchors; i++)
    {
        for (int j = i + 1; j < total_anchors; j++)
        {
            if (anchors[j].s - anchors[i].s > 2.0 * w_d) break; // outside arc window
            if (anchors[j].is_cw == anchors[i].is_cw) continue;  // same direction
            Point2LL pi = arc.pointAt(anchors[i].s);
            Point2LL pj = arc.pointAt(anchors[j].s);
            double dx = static_cast<double>(pi.X - pj.X);
            double dy = static_cast<double>(pi.Y - pj.Y);
            if (std::sqrt(dx * dx + dy * dy) < w_d)
            {
                anchors[i].skip     = true;
                anchors[j].skip     = true;
                anchors[i].pair_idx = j;
                anchors[j].pair_idx = i;
            }
        }
    }

    // Seam at the CCW helix-0 departure
    int start_idx = 0;
    {
        double min_dist = arc.total;
        for (int j = 0; j < total_anchors; j++)
        {
            if (anchors[j].is_cw) continue;
            double d = std::abs(anchors[j].s - ccw_advance);
            if (d < min_dist) { min_dist = d; start_idx = j; }
        }
    }
    const double seam_s = std::max(0.0, anchors[start_idx].s - w_d);

    seam_pt_ = arc.pointAt(seam_s);

    ExtrusionLine fp_line(/*inset_idx=*/0, /*is_odd=*/false, /*is_closed=*/true);

    auto processAnchors = [&](int i_begin, int i_end, double& current_s)
    {
        int i = i_begin;
        while (i < i_end)
        {
            const Anchor& anc = anchors[i];
            if (anc.skip)
            {
                // If this is the first of a detected collision pair, emit a lacing.
                // pair_idx points to the other anchor; only the first one (lower index) drives the lacing.
                if (anc.pair_idx > i)
                {
                    const double s_a   = anc.s;
                    const double s_b   = anchors[anc.pair_idx].s;
                    const double s_mid = (s_a + s_b) * 0.5;

                    double s_depart = s_mid - 0.75 * w_d;
                    if (s_depart < current_s) s_depart = current_s;

                    appendPolySegment(fp_line, arc, current_s, s_depart, w, fp_line.empty());

                    Point2LL mid_pt = arc.pointAt(s_mid);
                    double dcx = static_cast<double>(mid_pt.X - centroid.X);
                    double dcy = static_cast<double>(mid_pt.Y - centroid.Y);
                    double R_a = std::sqrt(dcx * dcx + dcy * dcy);
                    if (R_a > 1.0)
                    {
                        double theta_mid = std::atan2(dcy, dcx);
                        appendLacingTrace(fp_line, theta_mid, R_a, centroid, arc, w);
                    }

                    current_s = s_mid + 0.75 * w_d;
                }
                ++i;
                continue;
            }
            const double s_anchor = anc.s;
            double s_depart = s_anchor - w_d;
            if (s_depart < current_s) s_depart = current_s;

            Point2LL anchor_pt = arc.pointAt(s_anchor);
            double dcx = static_cast<double>(anchor_pt.X - centroid.X);
            double dcy = static_cast<double>(anchor_pt.Y - centroid.Y);
            double R_a = std::sqrt(dcx * dcx + dcy * dcy);
            if (R_a < 1.0) { ++i; continue; }
            double theta_anchor = std::atan2(dcy, dcx);

            appendPolySegment(fp_line, arc, current_s, s_depart, w, fp_line.empty());
            appendTrace(fp_line, theta_anchor, R_a, centroid, arc, anc.is_cw, w, false);
            current_s = s_anchor + w_d;
            ++i;
        }
    };

    double current_s = seam_s;
    processAnchors(start_idx, total_anchors, current_s);
    appendPolySegment(fp_line, arc, current_s, arc.total, w, fp_line.empty());
    current_s = 0.0;
    processAnchors(0, start_idx, current_s);
    appendPolySegment(fp_line, arc, current_s, seam_s, w, fp_line.empty());

    // Close: repeat the first junction to form a closed loop
    if (! fp_line.empty())
        fp_line.junctions_.push_back(fp_line.junctions_.front());

    if (fp_line.size() < 2)
        return {};

    VariableWidthLines result;
    result.push_back(std::move(fp_line));
    return result;
}

// ============================================================================
// Terminal loop — Whip feature for open-manifold boundary edges
// ============================================================================
//
// Canonical profile (w-units, anchor at lx=0):
//   Arc 1 : CCW 90°→180°, centre (1.5,-1.5), R=1.5 → (1.5,0) to (0,-1.5)
//   Arc 2a: CCW 180°→270°, centre (1.0,-1.5), R=1.0 → (0,-1.5) to (1.0,-2.5)
//   [Ins]  : horizontal at y=-2.5 from (1.0,-2.5) to (1.0+L,-2.5)  [Splay only, L>0]
//   Arc 2b: CCW 270°→360°, centre (1.0+L,-1.5), R=1.0 → (1.0+L,-2.5) to (2.0+L,-1.5)
//   Line 1: (2.0+L,-1.5) → (2.0+L,0)
// At L=0 Arc2a+Arc2b = standard Arc2 (CCW 180°→360°) and profile is the standard Terminal.
// x_sign=+1 for start endpoint (reversed=true path), -1 for end endpoint (reversed=false).

void FeatherPrintGenerator::appendTerminal(
    ExtrusionLine& line,
    double s_anchor, double x_sign,
    const ArcParam& arc, const Point2LL& centroid,
    coord_t w,
    bool reversed,
    double splay_L)
{
    Point2LL anchor_pt = arc.pointAt(s_anchor);
    double dcx = static_cast<double>(anchor_pt.X - centroid.X);
    double dcy = static_cast<double>(anchor_pt.Y - centroid.Y);
    double R_a = std::sqrt(dcx * dcx + dcy * dcy);
    if (R_a < 1.0)
        return;
    double theta_anchor = std::atan2(dcy, dcx);

    // Number of tessellation steps for the Insertion segment (straight line).
    const int n_ins = (splay_L > 1e-6) ? std::max(2, static_cast<int>(splay_L * 4.0 + 0.5)) : 0;

    auto cp = [&](double lx, double ly) -> Point2LL {
        return conformPlace(lx, ly, x_sign, theta_anchor, R_a, centroid, arc, w);
    };

    if (reversed)
    {
        // "Begins at terminal": Line1_rev → Arc2b_rev → Ins_rev → Arc2a_rev → Arc1_rev
        // Path starts at (2+L,0) on skin, ends at (1.5,0) for skin walk continuation.
        for (int k = 0; k <= 6; ++k)
            line.emplace_back(cp(2.0 + splay_L, -1.5 * k / 6.0), w, 0);
        // Arc 2b reversed: CW 360°→270°, centre (1.0+L,-1.5)
        appendCanonicalArc(line, 1.0 + splay_L, -1.5, 1.0, 360.0, 270.0, true, 5, x_sign,
                           theta_anchor, R_a, centroid, arc, w, /*skip_first=*/true);
        // Insertion reversed: (1.0+L,-2.5) → (1.0,-2.5)
        for (int k = 1; k <= n_ins; ++k)
            line.emplace_back(cp(1.0 + splay_L * (1.0 - static_cast<double>(k) / n_ins), -2.5), w, 0);
        // Arc 2a reversed: CW 270°→180°, centre (1.0,-1.5)
        appendCanonicalArc(line, 1.0, -1.5, 1.0, 270.0, 180.0, true, 5, x_sign,
                           theta_anchor, R_a, centroid, arc, w, /*skip_first=*/true);
        // Arc 1 reversed: CW 180°→90°, centre (1.5,-1.5)
        appendCanonicalArc(line, 1.5, -1.5, 1.5, 180.0, 90.0, true, 8, x_sign,
                           theta_anchor, R_a, centroid, arc, w, /*skip_first=*/true);
    }
    else
    {
        // "Ends at terminal": Arc1 → Arc2a → Ins → Arc2b → Line1
        // Path starts at (1.5,0) where skin walk stopped (s_anchor−1.5w).
        // Arc 1: CCW 90°→180°, centre (1.5,-1.5)
        appendCanonicalArc(line, 1.5, -1.5, 1.5, 90.0, 180.0, false, 8, x_sign,
                           theta_anchor, R_a, centroid, arc, w, /*skip_first=*/false);
        // Arc 2a: CCW 180°→270°, centre (1.0,-1.5) → reaches bottom (1.0,-2.5)
        appendCanonicalArc(line, 1.0, -1.5, 1.0, 180.0, 270.0, false, 5, x_sign,
                           theta_anchor, R_a, centroid, arc, w, /*skip_first=*/true);
        // Insertion: (1.0,-2.5) → (1.0+L,-2.5) at maximum depth
        for (int k = 1; k <= n_ins; ++k)
            line.emplace_back(cp(1.0 + splay_L * static_cast<double>(k) / n_ins, -2.5), w, 0);
        // Arc 2b: CCW 270°→360°, centre (1.0+L,-1.5) → arrives at (2.0+L,-1.5)
        appendCanonicalArc(line, 1.0 + splay_L, -1.5, 1.0, 270.0, 360.0, false, 5, x_sign,
                           theta_anchor, R_a, centroid, arc, w, /*skip_first=*/true);
        // Line 1: (2.0+L,-1.5) → (2.0+L,0)
        for (int k = 1; k <= 6; ++k)
            line.emplace_back(cp(2.0 + splay_L, -1.5 + 1.5 * k / 6.0), w, 0);
    }
}

// ============================================================================
// generateOpen — wall toolpath for an open-manifold layer
// ============================================================================

VariableWidthLines FeatherPrintGenerator::generateOpen(
    const OpenPolyline& open_poly,
    coord_t z,
    const Settings& settings,
    double helix_phase,
    const OpenLayerParams& params)
{
    // open_poly is already oriented CCW in math coordinates by the caller.
    if (open_poly.size() < 2)
        return {};

    const coord_t w = settings.get<coord_t>("featherprint_line_width");
    if (w <= 0)
        return {};

    const int N = settings.get<int>("featherprint_stringer_count");
    if (N < 1)
        return {};

    ArcParam arc_open = buildArcParamOpen(open_poly);
    const double s_end = arc_open.total;
    const double w_d   = static_cast<double>(w);

    if (s_end < 2.0 * w_d)
        return {};

    const Point2LL& centroid = params.centroid;

    // Anchor placement uses the full virtual ring shared across all arcs on this layer.
    // Anchors are computed in full-ring coordinates then shifted to arc-local coordinates.
    const double helix_frac  = std::fmod(helix_phase, 1.0);
    const double ccw_adv_abs = std::fmod(helix_frac * params.full_ring_total + params.full_ring_arc_ref, params.full_ring_total);
    const double cw_adv_abs  = std::fmod(params.full_ring_arc_ref - helix_frac * params.full_ring_total + params.full_ring_total, params.full_ring_total);

    std::vector<Anchor> anchors;
    anchors.reserve(2 * N);
    for (int i = 0; i < N; i++)
    {
        double s_abs   = std::fmod(static_cast<double>(i) / N * params.full_ring_total + ccw_adv_abs, params.full_ring_total);
        double s_local = s_abs - params.arc_start_in_ring;
        if (s_local > w_d && s_local < s_end - w_d)
            anchors.push_back({ s_local, false });
    }
    for (int i = 0; i < N; i++)
    {
        double s_abs   = std::fmod(static_cast<double>(i) / N * params.full_ring_total + cw_adv_abs, params.full_ring_total);
        double s_local = s_abs - params.arc_start_in_ring;
        if (s_local > w_d && s_local < s_end - w_d)
            anchors.push_back({ s_local, true });
    }
    std::sort(anchors.begin(), anchors.end(), [](const Anchor& a, const Anchor& b) { return a.s < b.s; });

    spdlog::debug("[FP-Open] s_end={:.2f}mm ring_total={:.2f}mm arc_start={:.2f}mm anchors={} ccw_abs={:.2f} cw_abs={:.2f}",
        s_end / 1000.0, params.full_ring_total / 1000.0, params.arc_start_in_ring / 1000.0,
        anchors.size(), ccw_adv_abs / 1000.0, cw_adv_abs / 1000.0);

    // Collision detection — same logic as generate(), using arc_open for world positions
    const int total_anchors = static_cast<int>(anchors.size());
    for (int i = 0; i < total_anchors; i++)
    {
        for (int j = i + 1; j < total_anchors; j++)
        {
            if (anchors[j].s - anchors[i].s > 2.0 * w_d) break;
            if (anchors[j].is_cw == anchors[i].is_cw) continue;
            Point2LL pi = arc_open.pointAt(anchors[i].s);
            Point2LL pj = arc_open.pointAt(anchors[j].s);
            double dx = static_cast<double>(pi.X - pj.X);
            double dy = static_cast<double>(pi.Y - pj.Y);
            if (std::sqrt(dx * dx + dy * dy) < w_d)
            {
                anchors[i].skip     = true;
                anchors[j].skip     = true;
                anchors[i].pair_idx = j;
                anchors[j].pair_idx = i;
            }
        }
    }

    // Terminal–stringer collision detection (Splay feature).
    // Any stringer within 4w of an endpoint is too close for the standard Terminal loop
    // to fit. Rather than suppressing the terminal, we widen it with the Splay insertion
    // (a horizontal segment of length L·w at the loop bottom, L = max(0, d/w - 2)).
    // The stringer trace itself is still suppressed (skip=true) because it is geometrically
    // subsumed by the widened terminal.
    bool draw_start_terminal = (s_end >= 2.0 * w_d);
    bool draw_end_terminal   = (s_end >= 2.0 * w_d);
    double start_splay_L = 0.0;
    double end_splay_L   = 0.0;
    for (auto& anc : anchors)
    {
        if (anc.skip) continue; // already handled by lacing
        if (anc.s < 3.0 * w_d)
        {
            // d/w = anc.s / w_d; L = max(0, d/w - 1)
            start_splay_L = std::max(start_splay_L, std::max(0.0, anc.s / w_d - 1.0));
            anc.skip = true;
        }
        if (anc.s > s_end - 3.0 * w_d)
        {
            end_splay_L = std::max(end_splay_L, std::max(0.0, (s_end - anc.s) / w_d - 1.0));
            anc.skip = true;
        }
    }

    // Open ExtrusionLine: walk from s=0 to s=s_end with embedded Traces/Lacings.
    // Uses arc_open throughout — arc_v was only needed for anchor placement.
    ExtrusionLine fp_line(/*inset_idx=*/0, /*is_odd=*/false, /*is_closed=*/false);

    double current_s = 0.0;
    if (draw_start_terminal)
    {
        // Start terminal: "begins at terminal" — draw reversed (Line→Arc2rev→Arc1rev).
        // Path starts at (2w,0) on skin, ends at (1.5w,0). Walk picks up from 1.5w.
        appendTerminal(fp_line, 0.0, +1.0, arc_open, centroid, w, /*reversed=*/true, start_splay_L);
        current_s = (2.0 + start_splay_L) * w_d;
    }
    int i = 0;
    while (i < total_anchors)
    {
        const Anchor& anc = anchors[i];
        if (anc.skip)
        {
            if (anc.pair_idx > i)
            {
                const double s_mid = (anc.s + anchors[anc.pair_idx].s) * 0.5;
                double s_depart = s_mid - 0.75 * w_d;
                if (s_depart < current_s) s_depart = current_s;

                appendPolySegment(fp_line, arc_open, current_s, s_depart, w, fp_line.empty());

                Point2LL mid_pt = arc_open.pointAt(s_mid);
                double dcx = static_cast<double>(mid_pt.X - centroid.X);
                double dcy = static_cast<double>(mid_pt.Y - centroid.Y);
                double R_a = std::sqrt(dcx * dcx + dcy * dcy);
                if (R_a > 1.0)
                    appendLacingTrace(fp_line, std::atan2(dcy, dcx), R_a, centroid, arc_open, w);

                current_s = s_mid + 0.75 * w_d;
            }
            ++i;
            continue;
        }

        const double s_anchor = anc.s;
        double s_depart = s_anchor - w_d;
        if (s_depart < current_s) s_depart = current_s;

        Point2LL anchor_pt = arc_open.pointAt(s_anchor);
        double dcx = static_cast<double>(anchor_pt.X - centroid.X);
        double dcy = static_cast<double>(anchor_pt.Y - centroid.Y);
        double R_a = std::sqrt(dcx * dcx + dcy * dcy);
        if (R_a < 1.0) { ++i; continue; }

        appendPolySegment(fp_line, arc_open, current_s, s_depart, w, fp_line.empty());
        appendTrace(fp_line, std::atan2(dcy, dcx), R_a, centroid, arc_open, anc.is_cw, w, false);
        current_s = s_anchor + w_d;
        ++i;
    }

    // End terminal: Arc1 departs from s_end - 1.5w (standard) or s_end - (1.5+L)w (splay).
    const double end_walk_stop = draw_end_terminal ? s_end - (1.5 + end_splay_L) * w_d : s_end;
    appendPolySegment(fp_line, arc_open, current_s, end_walk_stop, w, fp_line.empty());

    if (draw_end_terminal)
        appendTerminal(fp_line, s_end, -1.0, arc_open, centroid, w, /*reversed=*/false, end_splay_L);

    if (fp_line.size() < 2)
        return {};

    VariableWidthLines result;
    result.push_back(std::move(fp_line));
    return result;
}

} // namespace cura
