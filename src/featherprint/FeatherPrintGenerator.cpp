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
