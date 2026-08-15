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

    // Signed area via the shoelace formula — a single global winding test, exact for any
    // simple (non-self-intersecting) polygon regardless of concavity. Positive = CCW in
    // standard math convention (x-right, y-up), matching this file's atan2-based angle sense.
    double signed_area2 = 0.0;
    for (int i = 0; i < n; i++)
    {
        const Point2LL& a = poly[i];
        const Point2LL& b = poly[(i + 1) % n];
        signed_area2 += static_cast<double>(a.X) * b.Y - static_cast<double>(b.X) * a.Y;
    }
    ap.ccw = signed_area2 >= 0.0;
    return ap;
}

FeatherPrintGenerator::ArcParam FeatherPrintGenerator::buildArcParamOpen(const OpenPolyline& poly)
{
    ArcParam ap;
    ap.poly    = &poly;
    ap.is_open = true;
    // ccw stays at its default (true) — open arcs have no independent signed area; the caller
    // already guarantees CCW ordering (see ArcParam::ccw's declaration for the full rationale).
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
        double seg_start = cum_len[i];
        double seg_end = (j == 0) ? total : cum_len[j];
        // Skip degenerate (near-zero-length, duplicate-vertex) segments — a coincident vertex
        // pair from slicing yields a zero tangent here, and resolveFrame's fallback for that
        // substitutes an arbitrary (1,0) direction unrelated to the real local skin, producing
        // an isolated, layer-specific malformed Terminal loop. Falling through to the next real
        // segment gives the true local direction instead.
        if (seg_end - seg_start <= 1e-6) continue;
        if (s <= seg_end + 1e-6)
        {
            const Point2LL& a = (*poly)[i];
            const Point2LL& b = (*poly)[j];
            return b - a;
        }
    }
    // All remaining segments were degenerate (or s is past the last real one) — walk backward
    // from the end for the last non-degenerate segment instead of returning a zero vector.
    for (int i = n - 2; i >= 0; i--)
    {
        Point2LL d = (*poly)[i + 1] - (*poly)[i];
        if (std::abs(d.X) > 0 || std::abs(d.Y) > 0) return d;
    }
    return is_open ? ((*poly)[n - 1] - (*poly)[n - 2]) : ((*poly)[1] - (*poly)[0]);
}

double FeatherPrintGenerator::ArcParam::nearestArcPos(const Point2LL& target) const
{
    const int n = static_cast<int>(poly->size());
    double best_d2 = -1.0;
    double best_s  = 0.0;
    for (int i = 0; i < n; i++)
    {
        int j = (i + 1) % n;
        if (is_open && j == 0) break; // no closing segment for open polylines
        const Point2LL& a = (*poly)[i];
        const Point2LL& b = (*poly)[j];
        double ax = static_cast<double>(a.X), ay = static_cast<double>(a.Y);
        double ex = static_cast<double>(b.X - a.X), ey = static_cast<double>(b.Y - a.Y);
        double seg_len2 = ex * ex + ey * ey;
        double t = (seg_len2 > 1e-9)
            ? ((static_cast<double>(target.X) - ax) * ex + (static_cast<double>(target.Y) - ay) * ey) / seg_len2
            : 0.0;
        t = std::max(0.0, std::min(1.0, t));
        double px = ax + t * ex, py = ay + t * ey;
        double dx = static_cast<double>(target.X) - px, dy = static_cast<double>(target.Y) - py;
        double d2 = dx * dx + dy * dy;
        if (best_d2 < 0.0 || d2 < best_d2)
        {
            best_d2 = d2;
            double seg_start = cum_len[i];
            double seg_end   = (j == 0) ? total : cum_len[j];
            best_s = seg_start + t * (seg_end - seg_start);
        }
    }
    return best_s;
}

// ---- Curvature-Weighted Stringer Density (Spec REV 2.6) -----------------------------------

void FeatherPrintGenerator::ArcParam::ensureResampled(double step) const
{
    if (resample_built_)
        return;
    resample_built_ = true;
    resample_pts_.clear();
    if (total < 1.0)
        return;

    // Divide into a WHOLE number of equal-length intervals (not literal fixed-length steps) so
    // every resampled segment is exactly the same physical length on this ring, with no leftover
    // odd-length final segment -- that uniformity is the entire point of this method.
    const int n_intervals = std::max(is_open ? 2 : 3, static_cast<int>(std::llround(total / step)));
    const int n_points = is_open ? (n_intervals + 1) : n_intervals;
    resample_pts_.reserve(n_points);
    for (int i = 0; i < n_points; i++)
    {
        // Phase-locked to resample_phase_s (the caller-supplied Phase Origin arc-length), NOT
        // simply i/n_intervals*total from raw s=0 -- see resample_phase_s's declaration for why
        // an unlocked grid reintroduced the exact vertex-reindexing instability Anchor
        // Distribution was built to eliminate.
        double s = resample_phase_s + (static_cast<double>(i) / n_intervals) * total;
        if (! is_open)
        {
            s = std::fmod(s, total);
            if (s < 0.0) s += total;
        }
        resample_pts_.push_back(pointAt(s));
    }
}

double FeatherPrintGenerator::ArcParam::curvatureRadiusAt(double s) const
{
    // Estimated against a UNIFORMLY RESAMPLED copy of this ring (ensureResampled), not the raw
    // vertices. Three prior approaches here (a fixed arc-length window, a fixed vertex count, a
    // window that grows to a minimum physical span) all eventually failed on a real part's mesh
    // -- every one of them was really trying to reason about "how many real facets does my
    // window span" against a raw vertex spacing that turned out to be wildly non-uniform
    // (sub-mm in places, several mm in others, confirmed via diagnostic logging). Resampling
    // once to even spacing removes the question: every curvature estimate below is a plain
    // three-point turning angle between adjacent resampled points, which are the same physical
    // distance apart everywhere on this ring by construction, so there's no window-vs-facet
    // aliasing left to have a failure mode.
    constexpr double kResampleStep = 1000.0; // 1mm -- tentative, not yet validated against a tight-fillet case
    ensureResampled(kResampleStep);
    const int m = static_cast<int>(resample_pts_.size());
    if (m < 3)
        return 1.0e9;

    double sN = s;
    if (is_open)
        sN = std::max(0.0, std::min(total, sN));
    else
    {
        sN = std::fmod(sN, total);
        if (sN < 0.0) sN += total;
    }

    const int n_intervals = is_open ? (m - 1) : m;
    // Invert the same phase offset ensureResampled applied when building the grid, so this
    // lookup lands on the correct resampled index regardless of resample_phase_s.
    double rel = is_open ? sN : (sN - resample_phase_s);
    if (! is_open)
    {
        rel = std::fmod(rel, total);
        if (rel < 0.0) rel += total;
    }
    int k = static_cast<int>(std::llround(rel / total * n_intervals));
    int prev, next;
    if (is_open)
    {
        k = std::max(1, std::min(m - 2, k));
        prev = k - 1;
        next = k + 1;
    }
    else
    {
        k = ((k % m) + m) % m;
        prev = (k - 1 + m) % m;
        next = (k + 1) % m;
    }

    const Point2LL& a = resample_pts_[prev];
    const Point2LL& b = resample_pts_[k];
    const Point2LL& c = resample_pts_[next];
    double ex0 = static_cast<double>(b.X - a.X), ey0 = static_cast<double>(b.Y - a.Y);
    double ex1 = static_cast<double>(c.X - b.X), ey1 = static_cast<double>(c.Y - b.Y);
    const double l0 = std::sqrt(ex0 * ex0 + ey0 * ey0);
    const double l1 = std::sqrt(ex1 * ex1 + ey1 * ey1);
    if (l0 < 1e-6 || l1 < 1e-6)
        return 1.0e9;
    const double ds = l0 + l1;
    ex0 /= l0; ey0 /= l0;
    ex1 /= l1; ey1 /= l1;
    const double cross = ex0 * ey1 - ey0 * ex1;
    const double dot   = ex0 * ex1 + ey0 * ey1;
    const double dtheta = std::abs(std::atan2(cross, dot));
    if (dtheta < 1e-6)
        return 1.0e9; // no measurable turning here -- treat as straight
    return ds / dtheta;
}

void FeatherPrintGenerator::ArcParam::buildWarp(double R_ref)
{
    if (R_ref <= 0.0)
        return; // Curvature-Weighted Stringer Density inactive -- leave cum_warp empty (identity fallback)
    if (R_ref < 1.0)
        R_ref = 1.0;

    // Numerical safety clamps on rho = R_c/R_ref (not on curvatureRadiusAt's own large-value
    // return): the upper clamp is what actually prevents a straight segment (R_c -> infinity)
    // from producing an unbounded warped length; the lower clamp keeps a very tight curl from
    // collapsing a nonzero real span to ~zero warped length, which would break the monotonic
    // bijection buildWarp/fromWarped both depend on. Exact values are an implementation-level
    // tuning knob per the spec's own framing, not a designer-facing parameter.
    constexpr double kMinRho = 0.05;
    constexpr double kMaxRho = 20.0;

    const int n = static_cast<int>(poly->size());
    cum_warp.assign(n, 0.0);

    auto rhoAt = [&](double s)
    {
        double rho = curvatureRadiusAt(s) / R_ref;
        return std::min(std::max(rho, kMinRho), kMaxRho);
    };

    double running = 0.0;
    for (int i = 0; i < n; i++)
    {
        int j = (i + 1) % n;
        if (is_open && j == 0)
            break;
        double seg_start = cum_len[i];
        double seg_end   = (j == 0) ? total : cum_len[j];
        double seg_len = seg_end - seg_start;
        double rho0 = rhoAt(seg_start);
        double rho1 = rhoAt(seg_end);
        running += 0.5 * (rho0 + rho1) * seg_len;
        if (j != 0)
            cum_warp[j] = running;
    }
    total_warped = running;
}

double FeatherPrintGenerator::ArcParam::toWarped(double s) const
{
    if (cum_warp.empty())
        return s; // Curvature-Weighted Stringer Density inactive -- identity

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
        double seg_start = cum_len[i];
        double seg_end   = (j == 0) ? total : cum_len[j];
        if (s <= seg_end + 1e-6)
        {
            double seg_len = seg_end - seg_start;
            double t = (seg_len > 1e-6) ? (s - seg_start) / seg_len : 0.0;
            t = std::max(0.0, std::min(1.0, t));
            double w_start = cum_warp[i];
            double w_end   = (j == 0) ? total_warped : cum_warp[j];
            return w_start + t * (w_end - w_start);
        }
    }
    return is_open ? total_warped : 0.0;
}

double FeatherPrintGenerator::ArcParam::fromWarped(double w_target) const
{
    if (cum_warp.empty())
        return w_target; // Curvature-Weighted Stringer Density inactive -- identity

    if (is_open)
        w_target = std::max(0.0, std::min(total_warped, w_target));
    else
    {
        w_target = std::fmod(w_target, total_warped);
        if (w_target < 0.0) w_target += total_warped;
    }

    const int n = static_cast<int>(poly->size());
    for (int i = 0; i < n; i++)
    {
        int j = (i + 1) % n;
        if (is_open && j == 0) break;
        double w_start = cum_warp[i];
        double w_end   = (j == 0) ? total_warped : cum_warp[j];
        if (w_target <= w_end + 1e-6)
        {
            double seg_w = w_end - w_start;
            double t = (seg_w > 1e-9) ? (w_target - w_start) / seg_w : 0.0;
            t = std::max(0.0, std::min(1.0, t));
            double s_start = cum_len[i];
            double s_end   = (j == 0) ? total : cum_len[j];
            return s_start + t * (s_end - s_start);
        }
    }
    return is_open ? total : 0.0;
}

void FeatherPrintGenerator::accumulateCurvatureStats(const std::vector<Point2LL>& ring_pts, double& weighted_sum, double& length_sum)
{
    if (ring_pts.size() < 3)
        return;
    Polygon tmp;
    for (const Point2LL& p : ring_pts)
        tmp.push_back(p);
    ArcParam arc = buildArcParam(tmp);
    if (arc.total < 1e-6)
        return;

    // Absolute numerical safety cap on raw R_c -- R_ref isn't known yet at this stage (this IS
    // the pre-pass that determines R_avg, which R_ref is derived from), so the rho-based clamp
    // buildWarp uses can't apply here; a large fixed cap (100 m in coord_t/µm units) keeps a
    // near-straight segment from dominating the weighted average without needing R_ref first.
    constexpr double kAbsMaxRc = 1.0e8;
    const int n = static_cast<int>(tmp.size());
    for (int i = 0; i < n; i++)
    {
        int j = (i + 1) % n;
        double seg_start = arc.cum_len[i];
        double seg_end   = (j == 0) ? arc.total : arc.cum_len[j];
        double seg_len = seg_end - seg_start;
        double rc0 = std::min(arc.curvatureRadiusAt(seg_start), kAbsMaxRc);
        double rc1 = std::min(arc.curvatureRadiusAt(seg_end), kAbsMaxRc);
        weighted_sum += 0.5 * (rc0 + rc1) * seg_len;
        length_sum   += seg_len;
    }
}

void FeatherPrintGenerator::computeWarpedTotal(const std::vector<Point2LL>& ring_pts, double R_ref, const Point2LL& phase_origin, double& total, double& total_warped)
{
    total = 0.0;
    total_warped = 0.0;
    if (ring_pts.size() < 3)
        return;
    Polygon tmp;
    for (const Point2LL& p : ring_pts)
        tmp.push_back(p);
    ArcParam arc = buildArcParam(tmp);
    total = arc.total;
    if (R_ref > 0.0)
    {
        // Phase-lock the curvature-resampling grid to this Layer's own Phase Origin BEFORE
        // buildWarp triggers ensureResampled -- see resample_phase_s's declaration.
        arc.resample_phase_s = arc.nearestArcPos(phase_origin);
        arc.buildWarp(R_ref);
        total_warped = arc.total_warped;
    }
    else
    {
        total_warped = arc.total;
    }
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
// Skin-Normal Tangent-Blend Placement (Self-Reference/03-conformal-placement-transform.md
// "Skin-Normal Variant" — replaces the earlier centroid-radial transform)
// ============================================================================

FeatherPrintGenerator::TangentFrame FeatherPrintGenerator::resolveFrame(
    const ArcParam& arc, const Point2LL& centroid, double s, coord_t w)
{
    (void)centroid; // no longer used for orientation (see the winding-based fix below) — kept
                     // in the signature since every caller already has it in scope regardless.

    // s is resolved directly from the real perimeter's own arc-length parametrization (by the
    // caller, from the anchor's arc-length +/- a physical offset) — no angle/ray-cast involved,
    // so there is no distortion from local curvature or the point's angle relative to centroid.
    // pointAt/tangentAt are total (clamped for open polylines, wrapped for closed ones), so
    // there's no ray-cast-miss case to fall back from either.
    Point2LL S = arc.pointAt(s);

    // Tangent is sampled over a small arc-length WINDOW (pointAt(s-w)..pointAt(s+w)) rather
    // than read off the single polyline segment straddling s. A raw single-segment tangent
    // is sensitive to sub-millimeter mesh-slicing discretization noise: consecutive slice
    // layers land their boundary-edge vertices at slightly different spots along a curved
    // (e.g. hole) edge, so a real but tiny (5-25 degree) vertex kink can sit right next to a
    // Terminal/Trace anchor on some layers and not others, even though the underlying model
    // geometry is unchanged layer-to-layer. Averaging over a window smooths that out while
    // still tracking genuine, larger-scale skin curvature.
    // Was a fixed 300.0 (0.3mm each side) regardless of w -- tuned against a 0.4mm line width
    // (0.75x w there), but that ratio silently DOUBLES to 1.5x w at a 0.2mm line width, over-
    // smoothing local tangent direction right where Lacing's flat `d < w` collision test is
    // most sensitive to it. Expressing the window as a fraction of w keeps the same effective
    // smoothing behavior the original constant was tuned for, at any line width.
    const double kTangentWindowHalf = 0.75 * static_cast<double>(w);
    Point2LL Plo = arc.pointAt(s - kTangentWindowHalf);
    Point2LL Phi = arc.pointAt(s + kTangentWindowHalf);
    Point2LL Tv = Phi - Plo;
    double tx = static_cast<double>(Tv.X), ty = static_cast<double>(Tv.Y);
    double tlen = std::sqrt(tx * tx + ty * ty);
    if (tlen < 1e-6)
    {
        // Window collapsed to (near-)zero length — fall back to the raw single-segment
        // tangent (e.g. very short open polyline, or window clamped against both open ends).
        Tv = arc.tangentAt(s);
        tx = static_cast<double>(Tv.X); ty = static_cast<double>(Tv.Y);
        tlen = std::sqrt(tx * tx + ty * ty);
    }
    if (tlen < 1e-6) { tx = 1.0; ty = 0.0; tlen = 1.0; }
    tx /= tlen; ty /= tlen;

    // Orient T to match the anchor formula's CCW-increasing-theta convention (x_sign=+1
    // moves toward increasing theta), and derive the inward normal from it — using the
    // polygon's own global winding (arc.ccw), NOT a centroid-relative test. This used to
    // compare the tangent against the radial vector (S-centroid) rotated 90 degrees, and
    // separately resolve the normal's sign against (centroid-S); both assume "toward/away
    // from the centroid" reliably means "outward/inward," which breaks down in a concave
    // region or near a hole — the true inward side and the centroid-relative side can differ
    // there, silently misplacing every feature anchored nearby. Winding is a single global,
    // topology-level fact (computed once in buildArcParam/buildArcParamOpen, see ArcParam::ccw)
    // that stays correct at every point regardless of concavity: for a CCW-wound simple
    // polygon, increasing arc-length always runs with the interior on the left, so the inward
    // normal is always "tangent rotated +90 degrees" — no per-point test needed at all.
    if (! arc.ccw) { tx = -tx; ty = -ty; }
    double nx = -ty, ny = tx;

    return TangentFrame{ S, tx, ty, nx, ny };
}

FeatherPrintGenerator::BlendPlacement FeatherPrintGenerator::buildBlendPlacement(
    double s_anchor, double x_sign,
    double x_s, double x_e,
    const Point2LL& centroid, const ArcParam& arc, coord_t w)
{
    // s_anchor is already a real arc-length position (Anchor Distribution, Spec REV 2.2) —
    // Copy A/B are placed by walking the real perimeter's own arc-length from there, with no
    // angle/ray-cast round-trip, so no curvature/angle-dependent distortion of the physical
    // width between them.
    const double s_s = s_anchor + x_sign * x_s * w;
    const double s_e = s_anchor + x_sign * x_e * w;
    BlendPlacement bp;
    bp.A = resolveFrame(arc, centroid, s_s, w);
    bp.B = resolveFrame(arc, centroid, s_e, w);
    bp.x_s = x_s;
    bp.x_e = x_e;
    bp.x_sign = x_sign;
    return bp;
}

Point2LL FeatherPrintGenerator::BlendPlacement::place(double lx, double ly, coord_t w) const
{
    // Per spec: Copy A places the feature's OWN Start point (lx=x_s) exactly onto S(theta_start)
    // with no further offset; Copy B places lx=x_e exactly onto S(theta_end). That means each
    // copy's translation along T must be measured RELATIVE TO ITS OWN ANCHOR (lx-x_s for A,
    // lx-x_e for B) — not raw lx for both, which double-counts the anchor's own offset (S_A/S_B
    // are already offset from the feature's theta_anchor by x_sign*x_s*w / x_sign*x_e*w
    // respectively, via buildBlendPlacement's theta_s/theta_e) and was producing world
    // separations roughly double the intended canonical size for points sitting at the blend
    // range's own extremes.
    const double t = (x_e != x_s) ? (lx - x_s) / (x_e - x_s) : 0.0;
    const double wl_a = (lx - x_s) * static_cast<double>(w) * x_sign;
    const double wl_b = (lx - x_e) * static_cast<double>(w) * x_sign;
    const double wd = ly * static_cast<double>(w);

    const double ax = static_cast<double>(A.S.X) + wl_a * A.tx + wd * A.nx;
    const double ay = static_cast<double>(A.S.Y) + wl_a * A.ty + wd * A.ny;
    const double bx = static_cast<double>(B.S.X) + wl_b * B.tx + wd * B.nx;
    const double by = static_cast<double>(B.S.Y) + wl_b * B.ty + wd * B.ny;

    return Point2LL(
        static_cast<coord_t>(ax + t * (bx - ax)),
        static_cast<coord_t>(ay + t * (by - ay)));
}

void FeatherPrintGenerator::appendCanonicalArc(
    ExtrusionLine& line,
    double cx, double cy, double R,
    double a_start_deg, double a_end_deg,
    bool cw_arc, int segs,
    const BlendPlacement& blend,
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
        line.junctions_.emplace_back(blend.place(lx, ly, w), w, 0);
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
    // Ray-cast from the centroid at angle theta; the boundary can be non-star-shaped (holes,
    // concave teeth), so a single ray may cross it more than once. Must take the intersection
    // NEAREST the centroid (smallest positive t_val along the ray), not the first one found in
    // vertex order — picking an arbitrary far-side hit (e.g. across a hole) here silently
    // resolves a tangent frame to the wrong location, which showed up as a Whip Terminal loop
    // blown up and projecting past the skin into a nearby hole.
    const double cos_t = std::cos(theta), sin_t = std::sin(theta);
    const int n = static_cast<int>(arc.poly->size());
    double best_t = -1.0;
    double best_s = -1.0;
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
            if (best_t < 0.0 || t_val < best_t)
            {
                double seg_end = (j == 0) ? arc.total : arc.cum_len[j];
                best_t = t_val;
                best_s = arc.cum_len[i] + s_val * (seg_end - arc.cum_len[i]);
            }
        }
    }
    return best_s;
}

bool FeatherPrintGenerator::isThinSection(const ArcParam& arc, const Point2LL& centroid, double s_anchor, double D, coord_t w)
{
    if (D <= 0.0)
        return false;

    const TangentFrame tf = resolveFrame(arc, centroid, s_anchor, w);
    const double sx = static_cast<double>(tf.S.X), sy = static_cast<double>(tf.S.Y);
    const double dirx = tf.nx, diry = tf.ny; // unit inward normal, winding-based (REV 2.3)
    const double ray_len = D * static_cast<double>(w);

    const int n = static_cast<int>(arc.poly->size());

    // Exclusion is by VERTEX ADJACENCY (segment index distance from the anchor's own segment),
    // not an arc-length or world-distance window — this was the actual bug behind pruning
    // missing very tight corners (<10 degrees): at a fold that sharp, the genuine opposing wall
    // (the other side of the point) is arc-length-close to the anchor precisely BECAUSE the
    // fold is so tight, so an arc-length exclusion window threw out the real hit as if it were
    // the anchor's own immediate neighborhood. A world-distance window would fail the same way,
    // since world-closeness is exactly what makes a hit genuine near a tight fold. Only the
    // segment(s) literally touching the anchor's own vertex should be excluded, to avoid a
    // trivial t~0 self-intersection at the ray's own origin — anything else, however close in
    // arc-length or world space, is fair game for the hit test below.
    int anchor_seg = 0;
    {
        double sN = s_anchor;
        if (arc.is_open) sN = std::max(0.0, std::min(arc.total, sN));
        else { sN = std::fmod(sN, arc.total); if (sN < 0.0) sN += arc.total; }
        for (int k = 0; k < n; k++)
        {
            int kj = (k + 1) % n;
            if (arc.is_open && kj == 0) break;
            double kend = (kj == 0) ? arc.total : arc.cum_len[kj];
            if (sN <= kend + 1e-6) { anchor_seg = k; break; }
        }
    }
    constexpr int kExclusionSegs = 1; // segment index distance from the anchor's own segment
    auto segExcluded = [&](int idx)
    {
        int d = std::abs(idx - anchor_seg);
        if (! arc.is_open)
            d = std::min(d, n - d);
        return d <= kExclusionSegs;
    };

    for (int i = 0; i < n; i++)
    {
        int j = (i + 1) % n;
        if (arc.is_open && j == 0) break; // no closing segment on an open arc — real geometry
                                           // only, per spec (no virtual-ring gap chords here
                                           // since arc is always the real perimeter, never the
                                           // synthetic full ring)

        if (segExcluded(i))
            continue;

        const Point2LL& A = (*arc.poly)[i];
        const Point2LL& B = (*arc.poly)[j];
        const double ex = static_cast<double>(B.X - A.X), ey = static_cast<double>(B.Y - A.Y);
        const double seg_len = std::sqrt(ex * ex + ey * ey);
        if (seg_len < 1e-6)
            continue;
        const double denom = dirx * ey - diry * ex;
        if (std::abs(denom) < 1e-9)
            continue; // ray parallel to this segment
        const double asx = static_cast<double>(A.X) - sx, asy = static_cast<double>(A.Y) - sy;
        const double t = (asx * ey - asy * ex) / denom; // distance along the ray
        const double u = (asx * diry - asy * dirx) / denom; // position along the segment
        if (! (u >= -1e-9 && u <= 1.0 + 1e-9 && t > 1e-6 && t <= ray_len))
            continue;

        // A genuine thin section is where TWO DIFFERENT walls face each other closely — the
        // hit segment's own inward normal must point at least roughly opposite our ray, not
        // roughly the same way. Without this, a ray cast inward from a point on a gently
        // curved (but not truly thin) wall can "hit" that SAME wall a bit further around its
        // own curvature, or clip a tiny mesh-slicing kink just past the exclusion window —
        // both false positives: the hit segment there is still part of the near wall curving
        // along broadly the same direction as the anchor's own wall, not a distinct far wall
        // bounding a real gap. Using the same winding-based rule as resolveFrame (no
        // centroid): tangent = segment direction, flipped if the polygon is CW, inward normal
        // = tangent rotated +90.
        //
        // Threshold widened from a strict 90 degrees (dot < 0, i.e. only accepting hit normals
        // past perpendicular to the ray) down to kMinOpposingAngleDeg: a tight corner's
        // far-side segment doesn't always present a cleanly opposing normal the way a
        // straight/gently-curved thin gap does, so the strict version let features stick
        // through some sharp corners undetected. Accepting any hit normal more than
        // kMinOpposingAngleDeg away from pointing the SAME way as the ray (rather than
        // requiring it past perpendicular) catches those corners too, at the cost of accepting
        // more marginal, less-than-fully-opposing hits generally — a tentative tradeoff, per
        // the same "not a settled value" flagging as the rest of this check.
        constexpr double kMinOpposingAngleDeg = 45.0;
        const double kOpposingThreshold = std::cos(kMinOpposingAngleDeg * std::numbers::pi / 180.0);
        double htx = ex / seg_len, hty = ey / seg_len;
        if (! arc.ccw) { htx = -htx; hty = -hty; }
        const double hnx = -hty, hny = htx;
        if (dirx * hnx + diry * hny < kOpposingThreshold)
            return true;
    }
    return false;
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

std::vector<FeatherPrintGenerator::FlangeWallDesc> FeatherPrintGenerator::buildFormerWallStack(int r, coord_t w)
{
    // REV 3.6's own revision-history entry (the terse "what changed" summary, not the fuller
    // Profile/Slice body prose) is explicit: Former "reuses the Flange's own Wall-count/
    // arrangement rule directly" — the only structural difference from Flange is that the ramp
    // is mirrored in Z (built up over n Layers to a peak, then back down over n), not that the
    // Wall stack itself grows outward past the base skin surface. Former's outer face must stay
    // flush with the OML throughout, exactly like Flange's, for the same reason Flange's own
    // Profile section gives: "the outer face... does not move at any Layer." The Z-mirroring
    // is already handled upstream by the detection pre-pass (FffPolygonGenerator.cpp), which
    // reduces a Layer's own position within a Former band to a single ramp magnitude — 0 at the
    // band's own start/end, n at the peak — symmetric about the peak by construction. So this
    // function needs no logic of its own beyond that reduction: it collapses to Flange's own
    // rule at that magnitude.
    //
    // An earlier version of this function instead grew a Wall stack symmetrically both outward
    // AND inward from the base Wall's own centreline, per a literal reading of this spec
    // section's fuller Profile/Slice prose ("steps the wall outward by w/2 on each side...
    // growing the wall symmetrically about the perimeter centreline", "45 degree overhang on
    // the outer face") — that prose is now understood to be a spec-writing error introduced
    // when the terse revision-history note was expanded into full body text, contradicting the
    // revision-history's own "reuses Flange's rule directly" statement, and confirmed wrong by
    // real-print testing (the Former band visibly sat proud of the surrounding skin instead of
    // staying flush like Flange). Flagged for a spec correction to the Profile/Slice sections'
    // wording to match this.
    return buildFlangeWallStack(r, w);
}

int FeatherPrintGenerator::flangePrintInsetIdx(int wi, int n_walls)
{
    if (wi == 0) return 0;               // OML: always last
    if (wi == n_walls - 1) return 1;     // IML: second-to-last (not first)
    return wi + 1;                       // buried middle Walls: earliest, order among themselves doesn't matter
}

OpenPolyline FeatherPrintGenerator::normalOffsetOpen(const ArcParam& arc, const Point2LL& centroid, coord_t offset, coord_t w)
{
    OpenPolyline result;
    const double offset_d = static_cast<double>(offset);
    for (double s : arc.cum_len)
    {
        const TangentFrame tf = resolveFrame(arc, centroid, s, w);
        result.push_back(Point2LL(
            tf.S.X + static_cast<coord_t>(std::llround(tf.nx * offset_d)),
            tf.S.Y + static_cast<coord_t>(std::llround(tf.ny * offset_d))));
    }
    return result;
}

VariableWidthLines FeatherPrintGenerator::generateFlange(const Shape& outline, const Settings& settings, int ramp_index, double helix_phase, Point2LL phase_origin, double R_ref)
{
    const coord_t w = settings.get<coord_t>("featherprint_line_width");

    std::vector<FlangeWallDesc> walls_outer_first = buildFlangeWallStack(ramp_index, w);

    // Inner-area offset (see innerOffset()'s doc comment): the full Wall-stack depth to the
    // innermost Wall's own INNER face (offset + half its own width), i.e. the region actually
    // enclosed by the printed material at this Flange Layer.
    {
        const FlangeWallDesc& wd_inner = walls_outer_first.back();
        inner_offset_ = wd_inner.offset + wd_inner.width / 2;
    }

    // ---- Compute Stringer/Lacing anchor positions on OML for Flare Rim insertion ----
    const Polygon* oml_poly = largestPoly(outline);
    const int N = settings.get<int>("featherprint_stringer_count");
    // Flare Rim's Width matches the colliding feature's own Width (Spec REV 2.0), not a
    // fixed literal.
    const double stringer_flare_W = settings.get<double>("featherprint_stringer_width");
    const double lacing_flare_W   = settings.get<double>("featherprint_lacing_width");

    // theta/R_oml are evaluated on the OML (per spec, the Flare Rim is anchored there, not on
    // the innermost wall); s_left/s_right cut the innermost wall's own perimeter walk at the
    // exact theta the Flare Rim's own two endpoints land on (not an arc-length approximation),
    // so the ordinary-wall walk and the Flare Rim's first/last emitted points coincide.
    struct FlareAnchor { double s_left; double s_right; double theta; double R_oml; double x_sign; double W; };
    std::vector<FlareAnchor> flare_anchors;

    // Q per the WIP Parametric Canonical Toolpaths doc's Flare Rim table is the "Wall Number".
    // A plain Wall COUNT under-charges any ramp layer whose stack includes a half-width Wall
    // (the outer half-wall at ramp 0, or a buried half-wall at even ramp indices >= 2) — e.g.
    // ramp 2's 3-Wall stack (full+half+full = 2.5w) would score the same Q=3 as ramp 3's
    // 3-Wall stack (full+full+full = 3.0w), even though ramp 2's stack is only 2.5w thick.
    // Sum each Wall's own width instead, giving the stack's true thickness in w-units. The
    // innermost Wall itself is excluded from the sum — oml_shift already anchors Start/End at
    // the innermost Wall's own centreline, so Q should only cover the Walls OUTSIDE it (already
    // consumed depth before the Rim's own channel begins); including the innermost Wall's own
    // width double-counted it, undershooting R1 by one Wall's width.
    double Q = 0.0;
    for (size_t wi = 0; wi + 1 < walls_outer_first.size(); wi++)
        Q += static_cast<double>(walls_outer_first[wi].width) / static_cast<double>(w);
    const double flare_D = settings.get<double>("featherprint_feature_depth");
    const FlangeWallDesc& wd_inner_layer = walls_outer_first.back();
    // Start/End (canonical y=0) must land where the surrounding ordinary innermost-Wall path
    // actually runs — the innermost Wall's own offset from the OML — not at the OML itself.
    // The previous version of this shift used the OUTER Wall's (near-zero) offset, which is
    // only the right correction for arc_oml/oml_poly's own frame vs the true OML centreline;
    // it left Start/End sitting essentially AT the OML while the ordinary wall_line segments
    // on either side run at the innermost Wall's real (much deeper) offset, producing a large
    // spurious jump/zigzag at every Flare anchor instead of a smooth local fillet.
    const double oml_shift = -static_cast<double>(wd_inner_layer.offset) / static_cast<double>(w);
    // The new single-fillet Flare Rim profile doesn't extend past its declared flat span
    // W/2 (unlike the old dovetail's r-widened extent) — no extra collision widening needed.
    const double flare_r = 0.0;

    if (oml_poly && oml_poly->size() >= 3 && N >= 1)
    {
        ArcParam arc_oml = buildArcParam(*oml_poly);
        const Point2LL centroid = centroidBbox(*oml_poly);
        const double w_d = static_cast<double>(w);
        const double helix_frac  = std::fmod(helix_phase, 1.0);
        const double arc_ref     = arc_oml.nearestArcPos(phase_origin);
        // Phase-lock the curvature-resampling grid to Phase Origin BEFORE buildWarp triggers
        // ensureResampled -- see ArcParam::resample_phase_s's declaration for why this order
        // matters (an unlocked grid reintroduces vertex-reindexing instability).
        arc_oml.resample_phase_s = arc_ref;
        arc_oml.buildWarp(R_ref); // Curvature-Weighted Stringer Density (REV 2.6); no-op if R_ref <= 0
        const double arc_oml_total_w = arc_oml.cum_warp.empty() ? arc_oml.total : arc_oml.total_warped;
        const double origin_w    = arc_oml.toWarped(arc_ref);
        const double ccw_advance_w = std::fmod(helix_frac * arc_oml_total_w + origin_w, arc_oml_total_w);
        const double cw_advance_w  = std::fmod(origin_w - helix_frac * arc_oml_total_w + arc_oml_total_w, arc_oml_total_w);

        // OML anchor positions, CCW+CW interleaved and sorted by arc position so adjacent
        // opposite-direction anchors can be tested for a Lacing collision (same pattern as
        // the main generate() collision pass). Distributed in warped space (REV 2.6), then
        // inverted back to real arc-length via fromWarped -- everything downstream of this
        // (collision detection, Flare Rim placement) operates on real s exactly as before.
        struct OmlAnchor { double s; bool is_cw; bool skip{ false }; int pair_idx{ -1 }; };
        std::vector<OmlAnchor> oml_anchors;
        oml_anchors.reserve(2 * N);
        for (int i = 0; i < N; i++)
            oml_anchors.push_back({arc_oml.fromWarped(std::fmod(static_cast<double>(i) / N * arc_oml_total_w + ccw_advance_w, arc_oml_total_w)), false});
        for (int i = 0; i < N; i++)
            oml_anchors.push_back({arc_oml.fromWarped(std::fmod(static_cast<double>(i) / N * arc_oml_total_w + cw_advance_w,  arc_oml_total_w)), true });
        std::sort(oml_anchors.begin(), oml_anchors.end(), [](const OmlAnchor& a, const OmlAnchor& b){ return a.s < b.s; });

        // Two Stringer anchors must merge into one Lacing Rim once their *widened* Flare Rim
        // footprints (each spanning the Stringer's own Flare Rim Width, wider than the plain
        // 1w Stringer Trace cutout) would physically overlap — not just when the
        // plain-Trace threshold (w) is crossed. Using the unwidened threshold here left
        // near-tip layers (small circumference, closely packed anchors) with two separate,
        // overlapping/crossing Flare Rims instead of one merged Lacing Rim.
        const double collision_w = stringer_flare_W * w_d;

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
                // per the transform's angular formula theta = theta_anchor + x_sign*lx*w/R_a)
                // onto the innermost wall, not just the anchor's own theta — this is what
                // appendFlareRim will itself emit as its first/last points, so the
                // ordinary-wall walk must stop exactly there.
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
                        // symmetric profile (x_sign=+1), W = the Lacing's own Width setting.
                        double s_mid = (oa.s + oml_anchors[oa.pair_idx].s) * 0.5;
                        addAnchor(s_mid, 1.0, lacing_flare_W);
                    }
                    continue;
                }
                // Ordinary Stringer anchor: the Flare Rim's dovetail profile is left-right
                // symmetric (unlike the ordinary teardrop Trace), so it needs no CW/CCW
                // mirror — x_sign=-1 would instead reverse the low-s->high-s traversal
                // direction the wall-walk cutout (s_left/s_right) assumes, stitching it in
                // backwards for CW anchors. W = the Stringer's own Width setting.
                addAnchor(oa.s, 1.0, stringer_flare_W);
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
                // Thin-Section Pruning (Spec REV 2.4) — see generate()'s own comment for the
                // full rationale. Flare's own anchor is angle-based (out of REV 2.2/2.3's
                // arc-length scope, per spec), so convert to arc-length on arc_oml first, the
                // same relocation appendFlareRim itself already does internally.
                double s_anchor_oml = arcLengthAtAngle(arc_oml, centroid, fa.theta);
                if (s_anchor_oml < 0.0) s_anchor_oml = 0.0;
                if (isThinSection(arc_oml, centroid, s_anchor_oml, flare_D, w))
                    continue; // leave current_s untouched — ordinary wall passes straight through

                double s_depart = fa.s_left;
                if (s_depart < current_s) s_depart = current_s;
                appendPolySegment(wall_line, ap, current_s, s_depart, wd.width, wall_line.empty());

                // appendPolySegment above always ends by emitting the true wall-polygon point
                // at s_depart (unconditionally, regardless of add_start), so the Flare Rim's
                // own analytically-computed first point must always be skipped — using
                // wall_line.empty() here (now false, since the call above already pushed
                // points) skipped nothing and left a duplicate/jog at every rim's entry.
                appendFlareRim(wall_line, fa.theta, fa.R_oml, fa.x_sign, centroid, arc_oml,
                               Q, flare_D, oml_shift, fa.W, w, /*skip_first=*/true);

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
// Former (Spec REV 3.6) — a near-duplicate of generateFlange() by deliberate choice
// ============================================================================
//
// Reuses generateFlange()'s own wall-emission/Gusset(Flare-Rim)-embedding logic essentially
// verbatim, with buildFlangeWallStack swapped for buildFormerWallStack — the only structural
// difference is which wall-stack-shape function builds walls_outer_first; everything
// downstream (Q/oml_shift computation, anchor placement, Flare Rim embedding at the
// innermost Wall, per-Wall emission order) is identical, since both are pure functions of a
// {offset,width} Wall list agnostic to how that list was built. Duplicated rather than
// factored into a shared helper to avoid touching generateFlange()'s own already-tested code
// path — see this session's plan notes for the explicit risk/safety tradeoff.
VariableWidthLines FeatherPrintGenerator::generateFormer(const Shape& outline, const Settings& settings, int ramp_position, double helix_phase, Point2LL phase_origin, double R_ref)
{
    const coord_t w = settings.get<coord_t>("featherprint_line_width");

    std::vector<FlangeWallDesc> walls_outer_first = buildFormerWallStack(ramp_position, w);

    // Inner-area offset: same convention as generateFlange() — full Wall-stack depth to the
    // innermost Wall's own inner face.
    {
        const FlangeWallDesc& wd_inner = walls_outer_first.back();
        inner_offset_ = wd_inner.offset + wd_inner.width / 2;
    }

    // ---- Compute Stringer/Lacing anchor positions on the base perimeter for Gusset (Flare
    // Rim) insertion — "oml_poly" here is Former's own reference frame: the ordinary,
    // un-thickened perimeter this band grows from, playing the same role Flange's actual OML
    // plays for Flare. ----
    const Polygon* oml_poly = largestPoly(outline);
    const int N = settings.get<int>("featherprint_stringer_count");
    const double stringer_flare_W = settings.get<double>("featherprint_stringer_width");
    const double lacing_flare_W   = settings.get<double>("featherprint_lacing_width");

    struct FlareAnchor { double s_left; double s_right; double theta; double R_oml; double x_sign; double W; };
    std::vector<FlareAnchor> flare_anchors;

    double Q = 0.0;
    for (size_t wi = 0; wi + 1 < walls_outer_first.size(); wi++)
        Q += static_cast<double>(walls_outer_first[wi].width) / static_cast<double>(w);
    const double flare_D = settings.get<double>("featherprint_feature_depth");
    const FlangeWallDesc& wd_inner_layer = walls_outer_first.back();
    const double oml_shift = -static_cast<double>(wd_inner_layer.offset) / static_cast<double>(w);
    const double flare_r = 0.0;

    if (oml_poly && oml_poly->size() >= 3 && N >= 1)
    {
        ArcParam arc_oml = buildArcParam(*oml_poly);
        const Point2LL centroid = centroidBbox(*oml_poly);
        const double w_d = static_cast<double>(w);
        const double helix_frac  = std::fmod(helix_phase, 1.0);
        const double arc_ref     = arc_oml.nearestArcPos(phase_origin);
        arc_oml.resample_phase_s = arc_ref;
        arc_oml.buildWarp(R_ref);
        const double arc_oml_total_w = arc_oml.cum_warp.empty() ? arc_oml.total : arc_oml.total_warped;
        const double origin_w    = arc_oml.toWarped(arc_ref);
        const double ccw_advance_w = std::fmod(helix_frac * arc_oml_total_w + origin_w, arc_oml_total_w);
        const double cw_advance_w  = std::fmod(origin_w - helix_frac * arc_oml_total_w + arc_oml_total_w, arc_oml_total_w);

        struct OmlAnchor { double s; bool is_cw; bool skip{ false }; int pair_idx{ -1 }; };
        std::vector<OmlAnchor> oml_anchors;
        oml_anchors.reserve(2 * N);
        for (int i = 0; i < N; i++)
            oml_anchors.push_back({arc_oml.fromWarped(std::fmod(static_cast<double>(i) / N * arc_oml_total_w + ccw_advance_w, arc_oml_total_w)), false});
        for (int i = 0; i < N; i++)
            oml_anchors.push_back({arc_oml.fromWarped(std::fmod(static_cast<double>(i) / N * arc_oml_total_w + cw_advance_w,  arc_oml_total_w)), true });
        std::sort(oml_anchors.begin(), oml_anchors.end(), [](const OmlAnchor& a, const OmlAnchor& b){ return a.s < b.s; });

        const double collision_w = stringer_flare_W * w_d;

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
                const double lx_max = W / 2.0 + flare_r;
                const double theta_l = theta - lx_max * static_cast<double>(w) / R_oml;
                const double theta_r = theta + lx_max * static_cast<double>(w) / R_oml;
                double s_l = arcLengthAtAngle(arc_inner, centroid, theta_l);
                double s_r = arcLengthAtAngle(arc_inner, centroid, theta_r);
                if (s_l < 0.0 || s_r < 0.0)
                    return;
                flare_anchors.push_back({s_l, s_r, theta, R_oml, x_sign, W});
            };

            for (int i = 0; i < total_anchors; i++)
            {
                const OmlAnchor& oa = oml_anchors[i];
                if (oa.skip)
                {
                    if (oa.pair_idx > i)
                    {
                        double s_mid = (oa.s + oml_anchors[oa.pair_idx].s) * 0.5;
                        addAnchor(s_mid, 1.0, lacing_flare_W);
                    }
                    continue;
                }
                addAnchor(oa.s, 1.0, stringer_flare_W);
            }

            std::sort(flare_anchors.begin(), flare_anchors.end(),
                [](const FlareAnchor& a, const FlareAnchor& b){ return a.s_left < b.s_left; });
        }
    }

    const int n_walls = static_cast<int>(walls_outer_first.size());
    VariableWidthLines result;

    for (int wi = n_walls - 1; wi >= 0; wi--)
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
            ArcParam arc_oml = buildArcParam(*oml_poly);
            const Point2LL centroid = centroidBbox(*oml_poly);
            double current_s = 0.0;
            for (const FlareAnchor& fa : flare_anchors)
            {
                double s_anchor_oml = arcLengthAtAngle(arc_oml, centroid, fa.theta);
                if (s_anchor_oml < 0.0) s_anchor_oml = 0.0;
                if (isThinSection(arc_oml, centroid, s_anchor_oml, flare_D, w))
                    continue;

                double s_depart = fa.s_left;
                if (s_depart < current_s) s_depart = current_s;
                appendPolySegment(wall_line, ap, current_s, s_depart, wd.width, wall_line.empty());

                appendFlareRim(wall_line, fa.theta, fa.R_oml, fa.x_sign, centroid, arc_oml,
                               Q, flare_D, oml_shift, fa.W, w, /*skip_first=*/true);

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

// Former's own open-manifold case (Cuff, Spec REV 3.6) — near-duplicate of
// generateFlangeOpen() for the same reasons generateFormer() duplicates generateFlange():
// Cuff is mechanically identical to Miter (outer Wall gets an ordinary Whip Terminal at each
// end continuing the Terminal column, inner Wall(s) — including the innermost, which also
// carries Gusset — left open, welded by the outer Wall's Terminal sweep), so this is
// deliberately not a distinct design, just buildFlangeWallStack swapped for
// buildFormerWallStack.
VariableWidthLines FeatherPrintGenerator::generateFormerOpen(
    const OpenPolyline& open_poly, coord_t z, const Settings& settings,
    int ramp_position, double helix_phase, const OpenLayerParams& params)
{
    (void)z;
    if (open_poly.size() < 2)
        return {};

    const coord_t w = settings.get<coord_t>("featherprint_line_width");
    if (w <= 0)
        return {};

    const int N = settings.get<int>("featherprint_stringer_count");
    const double w_d = static_cast<double>(w);

    const double stringer_D = settings.get<double>("featherprint_feature_depth");
    const double stringer_W = settings.get<double>("featherprint_stringer_width");
    const double stringer_R2 = stringer_W / 2.0;
    const double stringer_R1 = stringer_R2 + 0.5; // matches Stringer Trace's corrected R1=R2+G
    const double flare_D = stringer_D;
    // Flare Rim's Width matches the colliding feature's own Width (Spec REV 2.0).
    const double lacing_flare_W = settings.get<double>("featherprint_lacing_width");

    std::vector<FlangeWallDesc> walls_outer_first = buildFormerWallStack(ramp_position, w);
    const Point2LL& centroid = params.centroid;

    const FlangeWallDesc& wd_inner_layer = walls_outer_first.back();
    double Q = 0.0;
    for (size_t wi = 0; wi + 1 < walls_outer_first.size(); wi++)
        Q += static_cast<double>(walls_outer_first[wi].width) / w_d;
    const double oml_shift = -static_cast<double>(wd_inner_layer.offset) / w_d;
    const double flare_r = 0.0;

    // ---- Compute Stringer/Lacing anchors within this arc, for Gusset (Flare Rim) insertion ----
    ArcParam arc_oml = buildArcParamOpen(open_poly);
    const double s_end_oml = arc_oml.total;

    struct FlareAnchor { double s_left; double s_right; double theta; double R_oml; double x_sign; double W; };
    std::vector<FlareAnchor> flare_anchors;

    if (N >= 1 && s_end_oml > 2.0 * w_d)
    {
        const double helix_frac  = std::fmod(helix_phase, 1.0);
        const double ccw_adv_abs = std::fmod(helix_frac * params.full_ring_total + params.full_ring_arc_ref, params.full_ring_total);
        const double cw_adv_abs  = std::fmod(params.full_ring_arc_ref - helix_frac * params.full_ring_total + params.full_ring_total, params.full_ring_total);

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

        const double collision_w = stringer_W * w_d;
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

        OpenPolyline inner_poly = normalOffsetOpen(arc_oml, centroid, wd_inner_layer.offset, w);
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
                    return;
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
                        addAnchor(s_mid, 1.0, lacing_flare_W);
                    }
                    continue;
                }
                addAnchor(oa.s, 1.0, stringer_W);
            }
            std::sort(flare_anchors.begin(), flare_anchors.end(),
                [](const FlareAnchor& a, const FlareAnchor& b){ return a.s_left < b.s_left; });
        }
    }

    const int n_walls = static_cast<int>(walls_outer_first.size());
    VariableWidthLines result;

    for (int wi = n_walls - 1; wi >= 0; wi--) // inner->outer emission order, matching generateFormer
    {
        const FlangeWallDesc& wd = walls_outer_first[wi];
        const bool is_outer = (wi == 0);
        const bool is_innermost = (wi == n_walls - 1);

        OpenPolyline offset_poly = normalOffsetOpen(arc_oml, centroid, wd.offset, w);
        if (offset_poly.size() < 2) continue;

        ArcParam arc_w = buildArcParamOpen(offset_poly);
        const double s_end = arc_w.total;
        if (s_end < 2.0 * w_d) continue;

        ExtrusionLine wall_line(flangePrintInsetIdx(wi, n_walls), /*is_odd=*/false, /*is_closed=*/false);

        if (is_outer)
        {
            // Outer Wall: ordinary Whip Terminal at both ends, continuing the same Terminal
            // column as ordinary Whip layers around the Former band (Cuff = Miter rule).
            {
                const size_t start1 = wall_line.junctions_.size();
                appendTerminal(wall_line, 0.0, +1.0, arc_w, centroid, w, stringer_D, stringer_R1, stringer_R2, stringer_W, /*reversed=*/true);
                for (size_t i = start1; i < wall_line.junctions_.size(); i++)
                    wall_line.junctions_[i].w_ = wd.width;
            }
            appendPolySegment(wall_line, arc_w, stringer_W * w_d, s_end - stringer_R1 * w_d, wd.width, wall_line.empty());
            {
                const size_t start2 = wall_line.junctions_.size();
                appendTerminal(wall_line, s_end, -1.0, arc_w, centroid, w, stringer_D, stringer_R1, stringer_R2, stringer_W, /*reversed=*/false);
                for (size_t i = start2; i < wall_line.junctions_.size(); i++)
                    wall_line.junctions_[i].w_ = wd.width;
            }
        }
        else if (is_innermost && ! flare_anchors.empty())
        {
            // Innermost Wall: left open at both ends per the Cuff/Miter rule, with a Gusset
            // (Flare Rim) channel embedded at each Stringer/Lacing anchor along the way.
            double current_s = 0.0;
            for (const FlareAnchor& fa : flare_anchors)
            {
                double s_anchor_oml = arcLengthAtAngle(arc_oml, centroid, fa.theta);
                if (s_anchor_oml < 0.0) s_anchor_oml = 0.0;
                if (isThinSection(arc_oml, centroid, s_anchor_oml, flare_D, w))
                    continue;

                double s_depart = std::min(std::max(fa.s_left, current_s), s_end);
                appendPolySegment(wall_line, arc_w, current_s, s_depart, wd.width, wall_line.empty());

                appendFlareRim(wall_line, fa.theta, fa.R_oml, fa.x_sign, centroid, arc_oml,
                               Q, flare_D, oml_shift, fa.W, w, /*skip_first=*/true);

                current_s = std::min(std::max(fa.s_right, s_depart), s_end);
            }
            appendPolySegment(wall_line, arc_w, current_s, s_end, wd.width, wall_line.empty());
        }
        else
        {
            // Buried middle Walls: left open at both ends per the Cuff/Miter rule, no Gusset.
            appendPolySegment(wall_line, arc_w, 0.0, s_end, wd.width, /*add_start=*/true);
        }

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

    const double stringer_D = settings.get<double>("featherprint_feature_depth");
    const double stringer_W = settings.get<double>("featherprint_stringer_width");
    const double stringer_R2 = stringer_W / 2.0;
    const double stringer_R1 = stringer_R2 + 0.5; // matches Stringer Trace's corrected R1=R2+G
    const double flare_D = stringer_D;
    // Flare Rim's Width matches the colliding feature's own Width (Spec REV 2.0).
    const double lacing_flare_W = settings.get<double>("featherprint_lacing_width");

    std::vector<FlangeWallDesc> walls_outer_first = buildFlangeWallStack(ramp_index, w);
    const Point2LL& centroid = params.centroid;

    const FlangeWallDesc& wd_inner_layer = walls_outer_first.back();
    // Same oml_shift convention as generateFlange(): Start/End (canonical y=0) must land at
    // the innermost Wall's own offset from the OML — where the surrounding ordinary
    // innermost-Wall path actually runs — not at the OML itself (see generateFlange's fuller
    // comment on this fix).
    // Q = the Wall stack's true thickness in w-units (sum of each Wall's own width), EXCLUDING
    // the innermost Wall itself — see generateFlange's fuller comment on this fix.
    double Q = 0.0;
    for (size_t wi = 0; wi + 1 < walls_outer_first.size(); wi++)
        Q += static_cast<double>(walls_outer_first[wi].width) / w_d;
    const double oml_shift = -static_cast<double>(wd_inner_layer.offset) / w_d;
    // The new single-fillet Flare Rim profile doesn't extend past its declared flat span
    // W/2 (unlike the old dovetail's r-widened extent) — no extra collision widening needed.
    const double flare_r = 0.0;

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

        const double collision_w = stringer_W * w_d;
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

        OpenPolyline inner_poly = normalOffsetOpen(arc_oml, centroid, wd_inner_layer.offset, w);
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
                        addAnchor(s_mid, 1.0, lacing_flare_W); // Lacing anchor, W = Lacing's own Width
                    }
                    continue;
                }
                addAnchor(oa.s, 1.0, stringer_W); // ordinary Stringer anchor, W = Stringer's own Width (symmetric profile, no mirror needed)
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

        OpenPolyline offset_poly = normalOffsetOpen(arc_oml, centroid, wd.offset, w);
        if (offset_poly.size() < 2) continue;

        ArcParam arc_w = buildArcParamOpen(offset_poly);
        const double s_end = arc_w.total;
        if (s_end < 2.0 * w_d) continue;

        ExtrusionLine wall_line(flangePrintInsetIdx(wi, n_walls), /*is_odd=*/false, /*is_closed=*/false);

        if (is_outer)
        {
            // Outer Wall: ordinary Whip Terminal at both ends, continuing the same Terminal
            // column as ordinary Whip layers below the Flange (Miter rule).
            //
            // The canonical D/R1/R2/W values are defined in units of the true FeatherPrint line
            // width (w), not this Wall's own printed width (wd.width) — at the first Flange ramp
            // layer the outer Wall is a HALF-width Wall (wd.width = w/2), so scaling the
            // Terminal's shape by wd.width shrank the whole loop to half its intended physical
            // size there, opening a gap against the full-size Terminal in the ordinary Whip
            // layer immediately below. Scale by w (matching every other Terminal call site) and
            // only apply wd.width to the emitted junctions' own printed bead width afterward.
            {
                const size_t start1 = wall_line.junctions_.size();
                appendTerminal(wall_line, 0.0, +1.0, arc_w, centroid, w, stringer_D, stringer_R1, stringer_R2, stringer_W, /*reversed=*/true);
                for (size_t i = start1; i < wall_line.junctions_.size(); i++)
                    wall_line.junctions_[i].w_ = wd.width;
            }
            appendPolySegment(wall_line, arc_w, stringer_W * w_d, s_end - stringer_R1 * w_d, wd.width, wall_line.empty());
            {
                const size_t start2 = wall_line.junctions_.size();
                appendTerminal(wall_line, s_end, -1.0, arc_w, centroid, w, stringer_D, stringer_R1, stringer_R2, stringer_W, /*reversed=*/false);
                for (size_t i = start2; i < wall_line.junctions_.size(); i++)
                    wall_line.junctions_[i].w_ = wd.width;
            }
        }
        else if (is_innermost && ! flare_anchors.empty())
        {
            // Innermost Wall: left open at both ends per the Miter rule, with a Flare Rim
            // channel embedded at each Stringer/Lacing anchor along the way.
            double current_s = 0.0;
            for (const FlareAnchor& fa : flare_anchors)
            {
                // Thin-Section Pruning (Spec REV 2.4) — see generate()'s own comment and
                // generateFlange()'s own Flare loop for the full rationale.
                double s_anchor_oml = arcLengthAtAngle(arc_oml, centroid, fa.theta);
                if (s_anchor_oml < 0.0) s_anchor_oml = 0.0;
                if (isThinSection(arc_oml, centroid, s_anchor_oml, flare_D, w))
                    continue;

                double s_depart = std::min(std::max(fa.s_left, current_s), s_end);
                appendPolySegment(wall_line, arc_w, current_s, s_depart, wd.width, wall_line.empty());

                appendFlareRim(wall_line, fa.theta, fa.R_oml, fa.x_sign, centroid, arc_oml,
                               Q, flare_D, oml_shift, fa.W, w, /*skip_first=*/true);

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
// Stringer Trace — Spec REV 2.0 Point Table, corrected (R1=R2+G, P1/P4 on the opposite
// side from Start/End so Arc1/Arc3 genuinely cross the centreline — the crossover is a
// real self-intersection near the top of the loop, not just the Start/End perimeter-level
// rejoin the first REV 2.0 reading implied).
// ============================================================================
//
// G=0.5 (fixed default). R2=W/2, R1=R2+G (never degenerates to <=0 for any R2>0 — no
// lower bound on W needed for this reason, unlike the R2-G formulation this replaces).
// D from featherprint_feature_depth (shared), W from featherprint_stringer_width.
// Point Table: Start(-G,0) P1(R2,R1) P2(R2,D-R2) P3(-R2,D-R2) P4(-R2,R1) End(G,0).
// Centres: C1(-G,R1) C2(0,D-R2) C3(G,R1).
// Path: Arc1 Start->P1 R1 CCW c=C1; Line1 P1->P2; Arc2 P2->P3 R2 CCW c=C2;
//       Line2 P3->P4; Arc3 P4->End R1 CCW c=C3.
// Validity: D >= W+G or Line1/Line2 invert (negative length); at D=W+G exactly they're
// zero-length (Arc1 tangent directly to Arc2), a valid degenerate case.
// Tangent-continuity verified at all four junctions (Arc1 end +Y = Line1 direction; Line1
// end +Y = Arc2 start; Arc2 end -Y = Line2 direction; Line2 end -Y = Arc3 start) — no cusps.
//
// The blend range is widened to the profile's actual canonical x-extent (max(G,R2), since
// Arc1/Arc2/Arc3/Line1/Line2 all reach out to +-R2, generally >G once R1=R2+G is
// comfortably nonzero) rather than just the true Start/End connection points at +-G. Using
// just +-G would extrapolate the two end frames far beyond where they're valid, visibly
// bowing/flattening the middle of the trace on curved skin.
//
// CW helix: per REV 2.0's flagged open item, resolved by direct derivation (not assumed):
// a naive x_sign flip alone (same call order, same x_s/x_e) misplaces the first-emitted
// point on the wrong side of the anchor relative to what the caller's stitching (walks to
// s_anchor-w_d before this call, resumes at s_anchor+w_d after) requires — and swapping
// x_s/x_e alone without also flipping x_sign stops the mapping from actually mirroring the
// shape at all (CCW and CW would trace identically). Reversed traversal + flipped arc
// direction + x_sign=-1 + swapped x_s/x_e (the same treatment the old asymmetric profiles
// needed) is the combination that satisfies both requirements simultaneously — verified by
// direct substitution, not merely carried over by default from the old profile's treatment.

void FeatherPrintGenerator::appendTrace(
    ExtrusionLine& line,
    double s_anchor,
    const Point2LL& centroid, const ArcParam& arc,
    bool is_cw, coord_t w,
    double D, double W, bool skip_first)
{
    const double G  = 0.5;
    const double R2 = W / 2.0;
    const double R1 = R2 + G;
    const double X_max = std::max(G, R2);

    if (! is_cw)
    {
        BlendPlacement bp = buildBlendPlacement(s_anchor, 1.0, -X_max, X_max, centroid, arc, w);
        appendCanonicalArc(line, -G,  R1, R1,  -90.0,   0.0, false, 8,  bp, w, skip_first);
        line.junctions_.emplace_back(bp.place(R2, D - R2, w), w, 0);
        appendCanonicalArc(line,  0.0, D - R2, R2,   0.0, 180.0, false, 10, bp, w, true);
        line.junctions_.emplace_back(bp.place(-R2, R1, w), w, 0);
        appendCanonicalArc(line,  G,   R1, R1,  180.0, -90.0, false, 8,  bp, w, true);
    }
    else
    {
        // Reverse traversal (End->P4->P3->P2->P1->Start), each arc's direction flipped,
        // x_sign=-1, x_s/x_e swapped — see derivation above.
        BlendPlacement bp = buildBlendPlacement(s_anchor, -1.0, X_max, -X_max, centroid, arc, w);
        appendCanonicalArc(line,  G,   R1, R1,  -90.0, 180.0, true, 8,  bp, w, skip_first);
        line.junctions_.emplace_back(bp.place(-R2, D - R2, w), w, 0);
        appendCanonicalArc(line,  0.0, D - R2, R2, 180.0,   0.0, true, 10, bp, w, true);
        line.junctions_.emplace_back(bp.place(R2, R1, w), w, 0);
        appendCanonicalArc(line, -G,   R1, R1,    0.0, -90.0, true, 8,  bp, w, true);
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
    double s_anchor,
    const Point2LL& centroid, const ArcParam& arc,
    coord_t w, double D, double W)
{
    // Spec REV 2.0's centers-first Lacing derivation (19 Jul 26 update): Lw=1 in w-units
    // (the whole profile is already scaled by w downstream), so R1=Lw/2=0.5 and
    // R2=(D-Lw)/2=(D-1)/2. Centres C1=(G+R1,R1), C2=(W/2-R2,D-R2), C3=-C2, C4=-C1; every
    // other point is an explicit R1/R2 offset from its own centre — see the spec's Point
    // Table for the full derivation. Verified tangent-continuous at all 8 junctions.
    //
    // The spec's own table has Start(=C1.X, positive/ahead-of-anchor) first and
    // End(=C4.X, negative/behind) last — backwards relative to the caller's stitching
    // (walks to s_mid-0.75w BEHIND the anchor before this call, resumes at s_mid+0.75w
    // AHEAD after), same directionality mismatch as appendTrace's and appendFlareRim's own
    // tables. Implemented as the reverse traversal (End->P7->P6->P5->P4->P3->P2->P1->Start),
    // each arc's direction flipped and x_s/x_e set to the actual first/last emitted point —
    // the same treatment already validated for those two features.
    const double G  = 0.5;
    const double R1 = 0.5;
    const double R2 = std::max(0.0, (D - 1.0) / 2.0);

    const double x_start = G + R1;   // = C1.X = -C4.X
    const double c2x     = W / 2.0 - R2; // = C2.X = -C3.X

    BlendPlacement bp = buildBlendPlacement(s_anchor, 1.0, -x_start, x_start, centroid, arc, w);

    auto pt = [&](double lx, double ly) {
        line.junctions_.emplace_back(bp.place(lx, ly, w), w, 0);
    };

    // Arc4 rev: End(-x_start,0) -> P7(-x_start,2*R1), R1, CW, centre C4(-x_start,R1)
    appendCanonicalArc(line, -x_start, R1, R1, -90.0, 90.0, false, 4, bp, w, false);
    // Line4 rev: -> P6(-c2x, 2*R1)
    pt(-c2x, 2.0 * R1);
    // Arc3 rev: -> P5(-c2x, D), R2, CCW, centre C3(-c2x, D-R2)
    appendCanonicalArc(line, -c2x, D - R2, R2, -90.0, 90.0, true, 6, bp, w, true);
    // Line3 rev: -> P4(0, D)
    pt(0.0, D);
    // Line2 rev: -> P3(c2x, D)
    pt(c2x, D);
    // Arc2 rev: -> P2(c2x, 2*R1), R2, CCW, centre C2(c2x, D-R2)
    appendCanonicalArc(line, c2x, D - R2, R2, 90.0, -90.0, true, 6, bp, w, true);
    // Line1 rev: -> P1(x_start, 2*R1)
    pt(x_start, 2.0 * R1);
    // Arc1 rev: -> Start(x_start, 0), R1, CW, centre C1(x_start, R1)
    appendCanonicalArc(line, x_start, R1, R1, 90.0, -90.0, false, 4, bp, w, true);
}

// ============================================================================
// Flare Rim (Stringer/Lacing × Flange)
// ============================================================================
//
// Anchored at (0,0) on the true OML (the outer wall's own toolpath centreline — NOT
// oml_arc's own ly=0 reference). Per Parametric Canonical Toolpaths (WIP).md: a 5-segment
// channel (Line-in, Arc1, Line-mid, Arc2, Line-out), R1 = min(D - Q, W/2) (w-units; Q = true
// thickness in w-units of this Flange ramp layer's Wall stack, i.e. the sum of each Wall's own
// width — NOT a plain Wall count, which under-charges a stack containing a half-width Wall —
// D = featherprint_feature_depth, shared). The W/2 clamp on R1 keeps Line-mid's span (W - 2*R1) from
// going negative and crossing the two end arcs over each other when D is large relative to W;
// the depth this clamp would otherwise cut off (drop = (D-Q) - W/2, when positive) is NOT
// dropped — it's picked up by the Line-in/Line-out vertical lead-in/out segments, so the
// channel still reaches the full requested depth D-Q regardless of how R1 clamps.
// oml_shift corrects only the placement (every canonical
// y-value is applied at y-oml_shift in oml_arc's own frame), same convention as before.
// W is the flat span at y=drop+R1 (2.0 for a Stringer anchor, 3.0 for a Lacing anchor).
// Traversed LEFT-to-right (increasing lx), matching the wall's own increasing-arc-length
// walk direction (spec requirement) — mirrored from the WIP doc's literal Start/End labels
// (Start = +W/2, End = -W/2), which read literally run right-to-left, backwards relative to
// both that stated requirement and the caller's own stitching (walks to s_left/behind first,
// resumes from s_right/ahead after) — the same issue as appendLacingTrace's Point Table.
// Mirrored Point Table: Start(-W/2,0) P1(-W/2,drop) P2(-W/2+R1,drop+R1) P3(W/2-R1,drop+R1)
// P4(W/2,drop) End(W/2,0). Centres: C1(-W/2+R1,drop) C2(W/2-R1,drop).
// Path: Line-in Start->P1; Arc1 P1->P2 R1 CW c=C1; Line-mid P2->P3;
//       Arc2 P3->P4 R1 CW c=C2; Line-out P4->End.
void FeatherPrintGenerator::appendFlareRim(
    ExtrusionLine& line,
    double theta_anchor, double R_a, double x_sign,
    const Point2LL& centroid, const ArcParam& oml_arc,
    double Q, double D, double oml_shift, double W, coord_t w, bool skip_first)
{
    (void)R_a; // out of scope for REV 2.2 (Flare Rim's own anchor math stays angle-based);
               // R_a was only ever forwarded to buildBlendPlacement, which no longer takes it.

    // Q_eff is the requested (unclamped) depth. R1 is clamped to W/2: past that, Line1's span
    // (W - 2*R1) goes negative and the two end arcs' flat spans cross over each other instead
    // of meeting at a single point, self-intersecting the profile. Any depth beyond what the
    // clamped R1 alone can reach is NOT dropped — it's picked up by a straight vertical
    // lead-in/out segment (Start->P1, P4->End) at each end, so the channel still reaches the
    // full requested depth D-Q; only the fillet's own radius is capped, not the total depth.
    const double Q_eff = D - Q;
    const double R1 = std::max(0.0, std::min(Q_eff, W / 2.0));
    const double drop = std::max(0.0, Q_eff - W / 2.0); // extra depth beyond the clamped fillet
    const double y0 = drop + R1 - oml_shift;   // channel floor (Line-mid) depth, in oml_arc's frame
    const double y_base = drop - oml_shift;    // depth where each end arc begins/ends (= foot of the lead-in/out line)
    const double y_top = -oml_shift;           // Start/End depth (0), in oml_arc's frame

    // Flare Rim's own anchor/blend-range derivation is still angle-based (Spec REV 2.2 leaves
    // this out of scope — see Perimeter Radius R(theta)'s note), so the arc-length conversion
    // buildBlendPlacement itself no longer performs has to happen here instead, one mechanical
    // relocation rather than a behavior change.
    double s_anchor = arcLengthAtAngle(oml_arc, centroid, theta_anchor);
    if (s_anchor < 0.0)
        s_anchor = 0.0;
    BlendPlacement bp = buildBlendPlacement(s_anchor, x_sign, -W / 2.0, W / 2.0, centroid, oml_arc, w);

    // Line-in: Start(-W/2,0) -> P1(-W/2,drop)
    if (! skip_first)
        line.junctions_.emplace_back(bp.place(-W / 2.0, y_top, w), w, 0);
    line.junctions_.emplace_back(bp.place(-W / 2.0, y_base, w), w, 0);
    // Arc1: P1(-W/2,drop) -> P2(-W/2+R1,drop+R1), R1, CW, centre C1(-W/2+R1,drop)
    appendCanonicalArc(line, -W / 2.0 + R1, y_base, R1, 180.0, 90.0, true, 4, bp, w, /*skip_first=*/true);
    // Line-mid: P2 -> P3(W/2-R1, drop+R1)
    line.junctions_.emplace_back(bp.place(W / 2.0 - R1, y0, w), w, 0);
    // Arc2: P3 -> P4(W/2,drop), R1, CW, centre C2(W/2-R1,drop)
    appendCanonicalArc(line, W / 2.0 - R1, y_base, R1, 90.0, 0.0, true, 4, bp, w, true);
    // Line-out: P4(W/2,drop) -> End(W/2,0)
    line.junctions_.emplace_back(bp.place(W / 2.0, y_top, w), w, 0);
}

// ============================================================================
// Lacing collision counting (Former-band detection support, Spec REV 3.6)
// ============================================================================
//
// Deliberately a separate, narrower function rather than a refactor of generate()'s own
// anchor/collision block into a shared helper: generate() is stable, already-tested code, and
// this query only needs a yes/no-plus-count answer, not the anchor list or any geometry.
// Duplicates the same anchor-placement and d < w collision test generate() uses internally
// (including the wraparound-seam handling), since that math depends on this class's own
// private ArcParam and can't be replicated outside it without exposing the whole machinery.
int FeatherPrintGenerator::countLacingCollisions(const Shape& outline, const Settings& settings, double helix_phase, Point2LL phase_origin, double R_ref)
{
    const Polygon* outer = largestPoly(outline);
    if (! outer || outer->size() < 3)
        return 0;

    const coord_t w = settings.get<coord_t>("featherprint_line_width");
    if (w <= 0)
        return 0;

    const int N = settings.get<int>("featherprint_stringer_count");
    if (N < 1)
        return 0;

    ArcParam arc = buildArcParam(*outer);
    if (arc.total < 4.0 * w)
        return 0;

    const double helix_frac = std::fmod(helix_phase, 1.0);
    const double arc_ref    = arc.nearestArcPos(phase_origin);
    arc.resample_phase_s = arc_ref;
    arc.buildWarp(R_ref);
    const double arc_total_w = (arc.cum_warp.empty()) ? arc.total : arc.total_warped;
    const double origin_w = arc.toWarped(arc_ref);
    const double ccw_advance_w = std::fmod(helix_frac * arc_total_w + origin_w, arc_total_w);
    const double cw_advance_w  = std::fmod(origin_w - helix_frac * arc_total_w + arc_total_w, arc_total_w);
    const double w_d = static_cast<double>(w);

    struct SimpleAnchor { double s; bool is_cw; };
    std::vector<SimpleAnchor> anchors;
    anchors.reserve(2 * N);
    for (int i = 0; i < N; i++)
        anchors.push_back({ arc.fromWarped(std::fmod(static_cast<double>(i) / N * arc_total_w + ccw_advance_w, arc_total_w)), false });
    for (int i = 0; i < N; i++)
        anchors.push_back({ arc.fromWarped(std::fmod(static_cast<double>(i) / N * arc_total_w + cw_advance_w, arc_total_w)), true });
    std::sort(anchors.begin(), anchors.end(), [](const SimpleAnchor& a, const SimpleAnchor& b) { return a.s < b.s; });

    const int total_anchors = static_cast<int>(anchors.size());
    const double collision_w = w_d;

    struct AnchorRef { double s; bool is_cw; int orig_idx; };
    std::vector<AnchorRef> ext;
    ext.reserve(total_anchors * 2);
    for (int i = 0; i < total_anchors; i++)
        ext.push_back({ anchors[i].s, anchors[i].is_cw, i });
    for (int i = 0; i < total_anchors; i++)
        if (anchors[i].s < 2.0 * collision_w)
            ext.push_back({ anchors[i].s + arc.total, anchors[i].is_cw, i });
    std::sort(ext.begin(), ext.end(), [](const AnchorRef& a, const AnchorRef& b) { return a.s < b.s; });

    std::vector<bool> collided(total_anchors, false);
    const int total_ext = static_cast<int>(ext.size());
    for (int i = 0; i < total_ext; i++)
    {
        for (int j = i + 1; j < total_ext; j++)
        {
            if (ext[j].s - ext[i].s > 2.0 * collision_w) break;
            if (ext[j].orig_idx == ext[i].orig_idx) continue;
            if (ext[j].is_cw == ext[i].is_cw) continue;
            Point2LL pi = arc.pointAt(ext[i].s);
            Point2LL pj = arc.pointAt(ext[j].s);
            double dx = static_cast<double>(pi.X - pj.X);
            double dy = static_cast<double>(pi.Y - pj.Y);
            if (std::sqrt(dx * dx + dy * dy) < collision_w)
            {
                collided[ext[i].orig_idx] = true;
                collided[ext[j].orig_idx] = true;
            }
        }
    }

    int count = 0;
    for (bool c : collided)
        if (c) count++;
    return count / 2; // each collision flags both partners; count pairs, not anchors
}

int FeatherPrintGenerator::countLacingCollisionsOpen(const OpenLinesSet& open_polylines, const Settings& settings, double helix_phase, Point2LL phase_origin)
{
    if (open_polylines.empty())
        return 0;

    const coord_t w = settings.get<coord_t>("featherprint_line_width");
    if (w <= 0)
        return 0;
    const int N = settings.get<int>("featherprint_stringer_count");
    if (N < 1)
        return 0;
    const double w_d = static_cast<double>(w);
    const double stringer_W = settings.get<double>("featherprint_stringer_width");

    // ---- Step 1: unified bounding-box centroid across all open polylines (mirrors
    // WallsComputation::generateWalls's own Step 1). ----
    coord_t uc_min_x{}, uc_max_x{}, uc_min_y{}, uc_max_y{};
    bool have_pt = false;
    for (const OpenPolyline& poly : open_polylines)
        for (const Point2LL& p : poly)
        {
            if (! have_pt) { uc_min_x = uc_max_x = p.X; uc_min_y = uc_max_y = p.Y; have_pt = true; }
            if (p.X < uc_min_x) uc_min_x = p.X; if (p.X > uc_max_x) uc_max_x = p.X;
            if (p.Y < uc_min_y) uc_min_y = p.Y; if (p.Y > uc_max_y) uc_max_y = p.Y;
        }
    if (! have_pt)
        return 0;
    const Point2LL centroid((uc_min_x + uc_max_x) / 2, (uc_min_y + uc_max_y) / 2);

    // ---- Step 2: orient each arc CCW (mirrors Step 2). ----
    struct OrientedArc { OpenPolyline poly; double start_angle{}; };
    std::vector<OrientedArc> arcs;
    for (const OpenPolyline& poly : open_polylines)
    {
        if (poly.size() < 2)
            continue;
        Polygon virt;
        for (const auto& p : poly) virt.push_back(p);
        OrientedArc oa;
        if (virt.area() < 0.0)
        {
            oa.poly.getPoints().resize(poly.size());
            std::reverse_copy(poly.begin(), poly.end(), oa.poly.getPoints().begin());
        }
        else
        {
            oa.poly.getPoints().assign(poly.begin(), poly.end());
        }
        double dx = static_cast<double>(oa.poly[0].X - centroid.X);
        double dy = static_cast<double>(oa.poly[0].Y - centroid.Y);
        oa.start_angle = std::atan2(dy, dx);
        arcs.push_back(std::move(oa));
    }
    if (arcs.empty())
        return 0;
    std::sort(arcs.begin(), arcs.end(), [](const OrientedArc& a, const OrientedArc& b) { return a.start_angle < b.start_angle; });

    // ---- Step 3: full virtual ring (mirrors Step 3). ----
    Polygon full_ring;
    std::vector<size_t> arc_vertex_start(arcs.size());
    for (size_t i = 0; i < arcs.size(); i++)
    {
        arc_vertex_start[i] = full_ring.size();
        for (const auto& p : arcs[i].poly)
            full_ring.push_back(p);
    }
    const size_t n_ring = full_ring.size();
    if (n_ring < 2)
        return 0;
    std::vector<double> cum_len(n_ring, 0.0);
    for (size_t i = 1; i < n_ring; i++)
    {
        double dx = full_ring[i].X - full_ring[i - 1].X;
        double dy = full_ring[i].Y - full_ring[i - 1].Y;
        cum_len[i] = cum_len[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    double full_ring_total;
    {
        double dx = full_ring[0].X - full_ring[n_ring - 1].X;
        double dy = full_ring[0].Y - full_ring[n_ring - 1].Y;
        full_ring_total = cum_len[n_ring - 1] + std::sqrt(dx * dx + dy * dy);
    }
    if (full_ring_total < 1.0)
        return 0;

    // ---- Step 4: Phase Origin projection onto the full ring (mirrors Step 5). ----
    double full_ring_arc_ref = 0.0;
    {
        double best_d2 = -1.0;
        for (size_t i = 0; i < n_ring; i++)
        {
            size_t j = (i + 1) % n_ring;
            double ax = static_cast<double>(full_ring[i].X), ay = static_cast<double>(full_ring[i].Y);
            double ex = static_cast<double>(full_ring[j].X - full_ring[i].X);
            double ey = static_cast<double>(full_ring[j].Y - full_ring[i].Y);
            double seg_len2 = ex * ex + ey * ey;
            double t = (seg_len2 > 1e-9)
                ? ((static_cast<double>(phase_origin.X) - ax) * ex + (static_cast<double>(phase_origin.Y) - ay) * ey) / seg_len2
                : 0.0;
            t = std::max(0.0, std::min(1.0, t));
            double px = ax + t * ex, py = ay + t * ey;
            double dx = static_cast<double>(phase_origin.X) - px;
            double dy = static_cast<double>(phase_origin.Y) - py;
            double d2 = dx * dx + dy * dy;
            if (best_d2 < 0.0 || d2 < best_d2)
            {
                best_d2 = d2;
                double seg_end = (j == 0) ? full_ring_total : cum_len[j];
                full_ring_arc_ref = cum_len[i] + t * (seg_end - cum_len[i]);
            }
        }
    }

    // ---- Step 5: per-arc anchor placement + collision counting (mirrors generateFlangeOpen's
    // own Flare-anchor collision block), summed across every arc on this Layer. ----
    const double helix_frac  = std::fmod(helix_phase, 1.0);
    const double ccw_adv_abs = std::fmod(helix_frac * full_ring_total + full_ring_arc_ref, full_ring_total);
    const double cw_adv_abs  = std::fmod(full_ring_arc_ref - helix_frac * full_ring_total + full_ring_total, full_ring_total);
    const double collision_w = stringer_W * w_d;

    int total_collisions = 0;
    for (size_t ai = 0; ai < arcs.size(); ai++)
    {
        ArcParam arc_oml = buildArcParamOpen(arcs[ai].poly);
        const double s_end_oml = arc_oml.total;
        if (s_end_oml <= 2.0 * w_d)
            continue;
        const double arc_start_in_ring = cum_len[arc_vertex_start[ai]];

        struct OmlAnchor { double s; bool is_cw; };
        std::vector<OmlAnchor> oml_anchors;
        oml_anchors.reserve(2 * N);
        for (int i = 0; i < N; i++)
        {
            double s_abs = std::fmod(static_cast<double>(i) / N * full_ring_total + ccw_adv_abs, full_ring_total);
            double s_local = s_abs - arc_start_in_ring;
            if (s_local > w_d && s_local < s_end_oml - w_d)
                oml_anchors.push_back({ s_local, false });
        }
        for (int i = 0; i < N; i++)
        {
            double s_abs = std::fmod(static_cast<double>(i) / N * full_ring_total + cw_adv_abs, full_ring_total);
            double s_local = s_abs - arc_start_in_ring;
            if (s_local > w_d && s_local < s_end_oml - w_d)
                oml_anchors.push_back({ s_local, true });
        }
        std::sort(oml_anchors.begin(), oml_anchors.end(), [](const OmlAnchor& a, const OmlAnchor& b) { return a.s < b.s; });

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
                    total_collisions++;
            }
        }
    }
    return total_collisions;
}

// ============================================================================
// Main generate
// ============================================================================

VariableWidthLines FeatherPrintGenerator::generate(
    const Shape& outline,
    coord_t z,
    const Settings& settings,
    double helix_phase,
    Point2LL phase_origin,
    double R_ref)
{
    seam_pt_ = Point2LL(0, 0);

    const Polygon* outer = largestPoly(outline);
    if (! outer || outer->size() < 3)
        return {};

    const coord_t w = settings.get<coord_t>("featherprint_line_width");
    if (w <= 0)
        return {};

    // Inner-area offset (see innerOffset()'s doc comment): a single, ordinary (non-Flange)
    // Layer is one Wall wide.
    inner_offset_ = w;

    const int N = settings.get<int>("featherprint_stringer_count");
    if (N < 1)
        return {};

    const double stringer_D = settings.get<double>("featherprint_feature_depth");
    const double stringer_W = settings.get<double>("featherprint_stringer_width");
    const double lacing_D   = stringer_D;
    const double lacing_W   = settings.get<double>("featherprint_lacing_width");

    ArcParam arc = buildArcParam(*outer);
    if (arc.total < 4.0 * w)
        return {};

    const Point2LL centroid = centroidBbox(*outer);
    // helix_phase is the running integral Σ(dz/arc_total) from the pre-pass,
    // giving constant intersection angle across the full height of a tapered tube.
    const double helix_frac = std::fmod(helix_phase, 1.0);
    const double arc_ref    = arc.nearestArcPos(phase_origin);

    // Curvature-Weighted Stringer Density (Spec REV 2.6): R_ref (= k_ref * R_avg) is passed in
    // already computed once for the whole mesh; buildWarp is a no-op (leaves cum_warp empty,
    // toWarped/fromWarped become the identity) when R_ref <= 0, so this is safe to call
    // unconditionally and behaves as "feature inactive" automatically. Phase-lock the
    // curvature-resampling grid to Phase Origin (arc_ref) BEFORE buildWarp triggers
    // ensureResampled -- see ArcParam::resample_phase_s's declaration; must run after arc_ref
    // is computed above, not before.
    arc.resample_phase_s = arc_ref;
    arc.buildWarp(R_ref);
    const double arc_total_w = (arc.cum_warp.empty()) ? arc.total : arc.total_warped;
    // Phase Origin itself is still a REAL arc-length landmark (Anchor Distribution's walk
    // operates in real space, unaffected by REV 2.6); only the per-stringer distribution below
    // is done in the warped coordinate, per the spec's own scoping.
    const double origin_w = arc.toWarped(arc_ref);
    // CCW helix advances with z; CW helix retreats — geodesic interlocking pair. Advance
    // magnitudes are expressed in warped units (arc_total_w), so a monotonic re-parametrization
    // can't break the ring-wide synchronized-crossing property (see spec's own argument for why).
    const double ccw_advance_w = std::fmod(helix_frac * arc_total_w + origin_w, arc_total_w);
    const double cw_advance_w  = std::fmod(origin_w - helix_frac * arc_total_w + arc_total_w, arc_total_w);
    const double w_d         = static_cast<double>(w);

    std::vector<Anchor> anchors;
    anchors.reserve(2 * N);
    for (int i = 0; i < N; i++)
        anchors.push_back({ arc.fromWarped(std::fmod(static_cast<double>(i) / N * arc_total_w + ccw_advance_w, arc_total_w)), false });
    for (int i = 0; i < N; i++)
        anchors.push_back({ arc.fromWarped(std::fmod(static_cast<double>(i) / N * arc_total_w + cw_advance_w,  arc_total_w)), true  });
    std::sort(anchors.begin(), anchors.end(), [](const Anchor& a, const Anchor& b) { return a.s < b.s; });

    const int total_anchors = static_cast<int>(anchors.size());

    // Cross-helix collision detection: if a CCW and CW anchor are within w world-space
    // distance they will intersect — mark both for lacing (d < w, per the original spec
    // value — confirmed correct as-is for Stringer x Stringer collisions).
    const double collision_w = w_d;
    // anchors is sorted by s in [0, arc.total) — a plain i<j scan with an early break never
    // considers a pair straddling the seam (one anchor near s~0, the other near s~arc.total,
    // physically adjacent via wraparound but far apart in this linear index order). Fixed by
    // scanning an extended list with wrapped duplicates of the near-start anchors appended
    // past the end (standard circular-array technique) — collision results are written back
    // to the real anchors via orig_idx, and a wrapped duplicate is never paired with its own
    // original (orig_idx equality guard) to avoid a false self-collision.
    struct AnchorRef { double s; bool is_cw; int orig_idx; };
    std::vector<AnchorRef> ext;
    ext.reserve(total_anchors + total_anchors);
    for (int i = 0; i < total_anchors; i++)
        ext.push_back({ anchors[i].s, anchors[i].is_cw, i });
    for (int i = 0; i < total_anchors; i++)
        if (anchors[i].s < 2.0 * collision_w)
            ext.push_back({ anchors[i].s + arc.total, anchors[i].is_cw, i });
    std::sort(ext.begin(), ext.end(), [](const AnchorRef& a, const AnchorRef& b) { return a.s < b.s; });

    const int total_ext = static_cast<int>(ext.size());
    for (int i = 0; i < total_ext; i++)
    {
        for (int j = i + 1; j < total_ext; j++)
        {
            if (ext[j].s - ext[i].s > 2.0 * collision_w) break; // outside arc window
            if (ext[j].orig_idx == ext[i].orig_idx) continue;   // wrapped duplicate of itself
            if (ext[j].is_cw == ext[i].is_cw) continue;         // same direction
            Point2LL pi = arc.pointAt(ext[i].s);
            Point2LL pj = arc.pointAt(ext[j].s);
            double dx = static_cast<double>(pi.X - pj.X);
            double dy = static_cast<double>(pi.Y - pj.Y);
            double dist = std::sqrt(dx * dx + dy * dy);
            bool hit = dist < collision_w;
            if (hit)
            {
                anchors[ext[i].orig_idx].skip     = true;
                anchors[ext[j].orig_idx].skip     = true;
                anchors[ext[i].orig_idx].pair_idx = ext[j].orig_idx;
                anchors[ext[j].orig_idx].pair_idx = ext[i].orig_idx;
            }
        }
    }

    // Seam at the CCW helix-0 departure. anchors[].s is real arc-length (already inverted via
    // fromWarped above); ccw_advance_w is warped, so it must be inverted the same way for this
    // real-space nearest-anchor comparison.
    const double ccw_advance = arc.fromWarped(ccw_advance_w);
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
                    const double s_a = anc.s;
                    const double s_b = anchors[anc.pair_idx].s;
                    // anchors is sorted by s, so s_a <= s_b normally — except for a pair the
                    // wraparound fix above matched across the seam (physically close via
                    // wraparound, but s_b - s_a appears large directly). In that case the
                    // naive midpoint (s_a+s_b)/2 lands on the wrong side of the part entirely;
                    // use the wrapped midpoint instead, folded back into [0, arc.total).
                    const double s_mid = (s_b - s_a > arc.total / 2.0)
                        ? std::fmod((s_a + s_b - arc.total) * 0.5 + arc.total, arc.total)
                        : (s_a + s_b) * 0.5;

                    // Thin-Section Pruning (Spec REV 2.4): skip drawing entirely if the local
                    // material at this Lacing's own midpoint anchor is thinner than its Depth
                    // — leave current_s untouched so the ordinary perimeter walk (this
                    // function's own catch-all / the next anchor's departure) passes straight
                    // through this span unmodified, as if no anchor were here at all.
                    if (! isThinSection(arc, centroid, s_mid, lacing_D, w))
                    {
                        double s_depart = s_mid - 0.75 * w_d;
                        if (s_depart < current_s) s_depart = current_s;

                        appendPolySegment(fp_line, arc, current_s, s_depart, w, fp_line.empty());
                        appendLacingTrace(fp_line, s_mid, centroid, arc, w, lacing_D, lacing_W);

                        current_s = s_mid + 0.75 * w_d;
                    }
                }
                ++i;
                continue;
            }
            const double s_anchor = anc.s;
            // Thin-Section Pruning (Spec REV 2.4): same treatment as the Lacing branch above.
            if (isThinSection(arc, centroid, s_anchor, stringer_D, w))
            {
                ++i;
                continue;
            }
            double s_depart = s_anchor - w_d;
            if (s_depart < current_s) s_depart = current_s;

            appendPolySegment(fp_line, arc, current_s, s_depart, w, fp_line.empty());
            appendTrace(fp_line, s_anchor, centroid, arc, anc.is_cw, w, stringer_D, stringer_W, false);
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
// Whip Terminal — Parametric Canonical Toolpaths (WIP).md
// ============================================================================
//
// R1,R2 match Stringer's (passed in). Point Table (+Y inward): Start(R1,0) P1(0,R1)
//   P2(0,D-R2) P3(R2,D) P4(W-R2,D) P5(W,D-R2) End(W,Lw/2) — End.y = 0.5 in w-units, per Spec
//   REV 2.0: the return point lands near, not exactly on, the perimeter (see Line3, below).
//   Centres: C1(R1,R1) C2(R2,D-R2) C3(W-R2,D-R2).
// Path: Arc1 Start->P1 R1 CW c=C1; Line1 P1->P2; Arc2 P2->P3 R2 CW c=C2;
//       Line2 P3->P4 (splay_L widens this span); Arc3 P4->P5 R2 CW c=C3; Line3 P5->End.
// x_sign=+1 for start endpoint (reversed=true path), -1 for end endpoint (reversed=false).

void FeatherPrintGenerator::appendTerminal(
    ExtrusionLine& line,
    double s_anchor, double x_sign,
    const ArcParam& arc, const Point2LL& centroid,
    coord_t w,
    double D, double R1, double R2, double W,
    bool reversed,
    double splay_L)
{
    const double Ws = W + splay_L; // W widened by the Splay insertion on the far (P4/P5/End) side

    if (! reversed)
    {
        BlendPlacement bp = buildBlendPlacement(s_anchor, x_sign, R1, Ws, centroid, arc, w);
        // Arc1: Start->P1, R1, CW, c=(R1,R1)
        appendCanonicalArc(line, R1, R1, R1, -90.0, 180.0, true, 8, bp, w, false);
        // Line1: P1->P2
        line.junctions_.emplace_back(bp.place(0.0, D - R2, w), w, 0);
        // Arc2: P2->P3, R2, CW, c=(R2,D-R2)
        appendCanonicalArc(line, R2, D - R2, R2, 180.0, 90.0, true, 5, bp, w, true);
        // Line2: P3->P4 (widened by splay)
        line.junctions_.emplace_back(bp.place(Ws - R2, D, w), w, 0);
        // Arc3: P4->P5, R2, CW, c=(Ws-R2,D-R2)
        appendCanonicalArc(line, Ws - R2, D - R2, R2, 90.0, 0.0, true, 5, bp, w, true);
        // Line3: P5->End (End.y = Lw/2 = 0.5 in w-units, per Spec REV 2.0 — the return point
        // lands near, not exactly on, the perimeter)
        line.junctions_.emplace_back(bp.place(Ws, 0.5, w), w, 0);
    }
    else
    {
        // Reverse traversal (End->P5->P4->P3->P2->P1->Start), each arc's direction flipped.
        BlendPlacement bp = buildBlendPlacement(s_anchor, x_sign, Ws, R1, centroid, arc, w);
        // Line3 rev: End->P5 (emit both endpoints — this is the first segment of the reversal)
        line.junctions_.emplace_back(bp.place(Ws, 0.5, w), w, 0);
        line.junctions_.emplace_back(bp.place(Ws, D - R2, w), w, 0);
        // Arc3 rev: P5->P4, R2, CCW, c=(Ws-R2,D-R2)
        appendCanonicalArc(line, Ws - R2, D - R2, R2, 0.0, 90.0, false, 5, bp, w, true);
        // Line2 rev: P4->P3
        line.junctions_.emplace_back(bp.place(R2, D, w), w, 0);
        // Arc2 rev: P3->P2, R2, CCW, c=(R2,D-R2)
        appendCanonicalArc(line, R2, D - R2, R2, 90.0, 180.0, false, 5, bp, w, true);
        // Line1 rev: P2->P1
        line.junctions_.emplace_back(bp.place(0.0, R1, w), w, 0);
        // Arc1 rev: P1->Start, R1, CCW, c=(R1,R1)
        appendCanonicalArc(line, R1, R1, R1, 180.0, -90.0, false, 8, bp, w, true);
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
    (void)z;
    // open_poly is already oriented CCW in math coordinates by the caller.
    if (open_poly.size() < 2)
        return {};

    const coord_t w = settings.get<coord_t>("featherprint_line_width");
    if (w <= 0)
        return {};

    const int N = settings.get<int>("featherprint_stringer_count");
    if (N < 1)
        return {};

    const double stringer_D = settings.get<double>("featherprint_feature_depth");
    const double stringer_W = settings.get<double>("featherprint_stringer_width");
    const double lacing_D   = stringer_D;
    const double lacing_W   = settings.get<double>("featherprint_lacing_width");
    const double stringer_R2 = stringer_W / 2.0;
    const double stringer_R1 = stringer_R2 + 0.5; // matches Stringer Trace's corrected R1=R2+G

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

    // Collision detection — same logic as generate() (d < w), using arc_open for world positions
    const int total_anchors = static_cast<int>(anchors.size());
    const double collision_w = w_d;
    for (int i = 0; i < total_anchors; i++)
    {
        for (int j = i + 1; j < total_anchors; j++)
        {
            if (anchors[j].s - anchors[i].s > 2.0 * collision_w) break;
            if (anchors[j].is_cw == anchors[i].is_cw) continue;
            Point2LL pi = arc_open.pointAt(anchors[i].s);
            Point2LL pj = arc_open.pointAt(anchors[j].s);
            double dx = static_cast<double>(pi.X - pj.X);
            double dy = static_cast<double>(pi.Y - pj.Y);
            double dist = std::sqrt(dx * dx + dy * dy);
            bool hit = dist < collision_w;
            if (hit)
            {
                anchors[i].skip     = true;
                anchors[j].skip     = true;
                anchors[i].pair_idx = j;
                anchors[j].pair_idx = i;
            }
        }
    }

    // Terminal–stringer collision detection (Splay feature).
    // Any stringer within 3w of an endpoint is too close for the standard Terminal loop
    // to fit. Rather than suppressing the terminal, we widen it with the Splay insertion
    // (a horizontal segment of length L·w at the loop bottom, L = max(0, d/w - 1)).
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

    // Combined-fit guard: on a short open segment (e.g. a slot edge sitting on a small
    // radius), the per-end checks above can each pass independently while the two Terminals'
    // own walk-in spans overlap, so they draw anyway and meet in the middle -- visually
    // closing a gap that's supposed to stay open. Mirrors generatePunchout()'s own joint
    // min_terminal_span/draw_terminals guard (see that function's comment) rather than
    // deciding each end in isolation. Suppress BOTH terminals together when they wouldn't fit;
    // the existing plain-walk appendPolySegment calls below already cover the fallback.
    const double min_both_terminal_span = (stringer_W + stringer_R1 + start_splay_L + end_splay_L) * w_d * 2.0 + w_d;
    if (s_end < min_both_terminal_span)
    {
        draw_start_terminal = false;
        draw_end_terminal   = false;
    }

    // Open ExtrusionLine: walk from s=0 to s=s_end with embedded Traces/Lacings.
    // Uses arc_open throughout — arc_v was only needed for anchor placement.
    ExtrusionLine fp_line(/*inset_idx=*/0, /*is_odd=*/false, /*is_closed=*/false);

    double current_s = 0.0;
    if (draw_start_terminal)
    {
        // Start terminal: "begins at terminal" — draw reversed (Line3rev→Arc3rev→Line2rev→Arc2rev→Line1rev→Arc1rev).
        // Path starts at (W+L,0) on skin, ends at (R1,0). Walk picks up from R1.
        appendTerminal(fp_line, 0.0, +1.0, arc_open, centroid, w, stringer_D, stringer_R1, stringer_R2, stringer_W, /*reversed=*/true, start_splay_L);
        current_s = (stringer_W + start_splay_L) * w_d;
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
                // Thin-Section Pruning (Spec REV 2.4) — see generate()'s own comment for the
                // full rationale; same treatment here.
                if (! isThinSection(arc_open, centroid, s_mid, lacing_D, w))
                {
                    double s_depart = s_mid - 0.75 * w_d;
                    if (s_depart < current_s) s_depart = current_s;

                    appendPolySegment(fp_line, arc_open, current_s, s_depart, w, fp_line.empty());
                    appendLacingTrace(fp_line, s_mid, centroid, arc_open, w, lacing_D, lacing_W);

                    current_s = s_mid + 0.75 * w_d;
                }
            }
            ++i;
            continue;
        }

        const double s_anchor = anc.s;
        // Thin-Section Pruning (Spec REV 2.4) — see generate()'s own comment.
        if (isThinSection(arc_open, centroid, s_anchor, stringer_D, w))
        {
            ++i;
            continue;
        }
        double s_depart = s_anchor - w_d;
        if (s_depart < current_s) s_depart = current_s;

        appendPolySegment(fp_line, arc_open, current_s, s_depart, w, fp_line.empty());
        appendTrace(fp_line, s_anchor, centroid, arc_open, anc.is_cw, w, stringer_D, stringer_W, false);
        current_s = s_anchor + w_d;
        ++i;
    }

    // End terminal: Arc1 departs from s_end - R1*w (standard) or s_end - (R1+L)w (splay).
    const double end_walk_stop = draw_end_terminal ? s_end - (stringer_R1 + end_splay_L) * w_d : s_end;
    appendPolySegment(fp_line, arc_open, current_s, end_walk_stop, w, fp_line.empty());

    if (draw_end_terminal)
        appendTerminal(fp_line, s_end, -1.0, arc_open, centroid, w, stringer_D, stringer_R1, stringer_R2, stringer_W, /*reversed=*/false, end_splay_L);

    if (fp_line.size() < 2)
        return {};

    VariableWidthLines result;
    result.push_back(std::move(fp_line));
    return result;
}

// ============================================================================
// Punchout (Spec REV 3.3) — Punchout Line + Terminal for one hole-gap
// ============================================================================
//
// See the header's doc comment for the full rationale, and WallsComputation.cpp's own
// "Step 7" comment for how prev_wall_poly/next_wall_poly are identified. In short: they are
// the two REAL open polylines the caller determined bound this hole in ring order — NOT one
// real open polyline's own front()/back(), which (once a layer has more than one hole)
// usually belong to two DIFFERENT holes rather than a matched pair. A previous attempt paired
// a single open polyline's own two endpoints directly, which — beyond ever having placed
// geometry against the wrong arc-length parametrization — also bridged across unrelated
// holes through solid wall whenever a layer had more than one hole open at once. This
// implementation builds a synthetic ArcParam over the cutback CHORD between P0/P1 and reuses
// appendTerminal/appendPolySegment against that chord exclusively.
VariableWidthLines FeatherPrintGenerator::generatePunchout(
    const OpenPolyline& prev_wall_poly,
    const OpenPolyline& next_wall_poly,
    coord_t z,
    const Settings& settings,
    const OpenLayerParams& params,
    const std::vector<Point2LL>& contour_pts,
    bool half_width_ends,
    Point2LL* out_q0,
    Point2LL* out_q1)
{
    (void)z;
    if (! settings.get<bool>("featherprint_punchout_enabled"))
        return {};
    if (prev_wall_poly.size() < 1 || next_wall_poly.size() < 1)
        return {};

    const coord_t w = settings.get<coord_t>("featherprint_line_width");
    if (w <= 0)
        return {};
    const double w_d = static_cast<double>(w);

    // Punchout Terminal reuses the Whip Terminal profile as-is (Spec REV 3.3), assumed to
    // derive its Width from featherprint_stringer_width — the same setting Whip Terminal
    // itself uses, since neither has a dedicated Width parameter of its own.
    const double D  = settings.get<double>("featherprint_feature_depth");
    const double W  = settings.get<double>("featherprint_stringer_width");
    const double R2 = W / 2.0;
    const double R1 = R2 + 0.5; // matches Stringer/Whip's own corrected R1=R2+G

    // The two true hole-edge anchors — P0 is the END of prev_wall_poly (the same point
    // Whip's Terminal is anchored to at its s_end) and P1 is the START of next_wall_poly
    // (Whip's s=0 anchor there). These are generally two DIFFERENT real polylines' endpoints,
    // not one polyline's own front()/back() — see this function's header comment.
    const Point2LL& P0 = prev_wall_poly.back();
    const Point2LL& P1 = next_wall_poly.front();

    const double dx = static_cast<double>(P1.X - P0.X);
    const double dy = static_cast<double>(P1.Y - P0.Y);
    const double chord_len = std::sqrt(dx * dx + dy * dy);
    if (chord_len < 1e-6)
        return {};
    const double ux = dx / chord_len, uy = dy / chord_len;

    // Cut back along the CHORD (into the hole), never along either real polyline's own
    // tangent — this is the direct fix for the "outside the hole" defect:
    // featherprint_punchout_gap is measured in this direction only. This cutback is a FLOOR,
    // not a fixed value — see the Angle-Adaptive Gap block just below for why. An earlier
    // attempt scaled it toward zero near a hole's own top/bottom (to weld the Terminal to the
    // real wall corner), which slid the entire Terminal loop (sized by D/R1/R2/W) so its
    // anchor coincided with the real Whip Terminal's own anchor at that same corner; both
    // loops bulge inward from there by a similar depth in roughly the same direction, so they
    // overlapped. A follow-up attempt fixed the overlap by adding a separate tangent-matched
    // "weld stub" curve instead of moving the Terminal — that stub was removed again after
    // real-print testing found it made the Punchout materially harder to break away, which
    // defeats the point of a removable support feature. The cutback gap is intentionally left
    // unwelded now; see Contour Matching, below, for how the middle of the Line still tracks
    // the wall shape without needing the ends themselves to touch it.
    const coord_t gap = settings.get<coord_t>("featherprint_punchout_gap");
    const double gap_d = static_cast<double>(gap);

    // Angle-Adaptive Gap: found via real-print testing on a hole with an angled edge — the
    // Punchout ended up sitting directly on the sidewall. Root cause: the real Whip Terminal
    // at P0/P1 (generated separately, anchored right there) recedes AWAY from the hole along
    // that wall's own local tangent in the ordinary case — a hole is just a gap in the wall's
    // own path, so the wall's tangent there runs nearly parallel to the chord, and the
    // Terminal's local +x axis (mapped by its own x_sign, per appendTerminal's own doc: -1 at
    // an end endpoint like P0, +1 at a start endpoint like P1) points back into solid
    // material, well clear of a flat gap tuned for that case. When the hole's edge is angled,
    // that tangent diverges from the chord direction, and the Terminal's own canonical (W, D)
    // bounding corner partly projects INTO the chord instead of fully receding.
    //
    // Fix: project each Terminal's own bounding corner (placed in world space the same way
    // appendTerminal itself does — x_sign*tangent*W + inward_normal*D from the anchor) onto
    // the chord direction TOWARD that Terminal's own hole side, and grow that end's own
    // cutback to at least clear it (plus one line width of margin). This is a floor via
    // std::max against the nominal setting, so the ordinary (near-parallel) case — where the
    // projection comes out negative, meaning the Terminal recedes away from the chord — is
    // completely unaffected; the gap only ever grows, never shrinks below what was asked for.
    auto tangentAt = [](const OpenPolyline& poly, bool at_end) -> std::pair<double, double>
    {
        if (poly.size() < 2)
            return { 0.0, 0.0 }; // degenerate polyline — caller falls back to the floor gap only
        const Point2LL& a = at_end ? poly[poly.size() - 2] : poly[0];
        const Point2LL& b = at_end ? poly[poly.size() - 1] : poly[1];
        double tx = static_cast<double>(b.X - a.X), ty = static_cast<double>(b.Y - a.Y);
        const double tlen = std::sqrt(tx * tx + ty * ty);
        if (tlen < 1e-6)
            return { 0.0, 0.0 };
        return { tx / tlen, ty / tlen };
    };

    double gap0_d = gap_d, gap1_d = gap_d;
    {
        const auto [t0x, t0y] = tangentAt(prev_wall_poly, /*at_end=*/true);
        const auto [t1x, t1y] = tangentAt(next_wall_poly, /*at_end=*/false);
        // Tangent rotated +90 degrees = inward normal — same convention resolveFrame uses
        // elsewhere (an open polyline's arc.ccw is always true in this codebase).
        const double n0x = -t0y, n0y = t0x;
        const double n1x = -t1y, n1y = t1x;

        // Terminal at P0 (x_sign=-1): bounding corner is P0 + (-T0)*W*w + N0*D*w. Projected
        // onto +C (the direction from P0 toward P1, i.e. toward the hole from this side):
        const double intrusion0 = -W * w_d * (t0x * ux + t0y * uy) + D * w_d * (n0x * ux + n0y * uy);
        // Terminal at P1 (x_sign=+1): bounding corner is P1 + T1*W*w + N1*D*w. Projected onto
        // -C (the direction from P1 toward P0, i.e. toward the hole from this side):
        const double intrusion1 = -W * w_d * (t1x * ux + t1y * uy) - D * w_d * (n1x * ux + n1y * uy);

        gap0_d = std::max(gap_d, intrusion0 + w_d);
        gap1_d = std::max(gap_d, intrusion1 + w_d);
    }

    if (gap0_d + gap1_d >= chord_len)
        return {}; // hole too narrow for the requested (possibly angle-grown) cutback — nothing safe to draw

    const Point2LL Q0(static_cast<coord_t>(std::llround(P0.X + ux * gap0_d)), static_cast<coord_t>(std::llround(P0.Y + uy * gap0_d)));
    const Point2LL Q1(static_cast<coord_t>(std::llround(P1.X - ux * gap1_d)), static_cast<coord_t>(std::llround(P1.Y - uy * gap1_d)));
    // Report the real cutback points for Shelf (Spec REV 4.2) — see this function's own
    // out_q0/out_q1 doc comment. Populated as soon as Q0/Q1 are known, even if this call later
    // returns an empty VariableWidthLines for some other reason downstream.
    if (out_q0) *out_q0 = Q0;
    if (out_q1) *out_q1 = Q1;

    // Synthetic open polyline over the cutback chord, in the SAME point order (Q0 first,
    // Q1 last) as the ring's own forward (CCW) traversal direction — P0 was the END of the
    // previous arc walking forward, P1 the START of the next. Because resolveFrame's
    // inward-normal formula (tangent rotated 90 degrees, for is_open which always has
    // ccw=true) is a purely local computation independent of which polyline it's applied
    // to, preserving this point order makes the chord's own inward normal automatically
    // match the wall's ring-wide convention — no separate sign-matching step needed, and no
    // risk of curling the Terminal back toward the wall. This relies on every real open
    // polyline already being individually oriented consistent with that same global ring
    // direction (Step 2 in WallsComputation.cpp), which the ring's own arc-length/Phase
    // Origin math already assumes elsewhere in this file.
    OpenPolyline chord_poly;
    chord_poly.push_back(Q0);
    chord_poly.push_back(Q1);
    ArcParam chord_arc = buildArcParamOpen(chord_poly);
    const double s_end = chord_arc.total;
    if (s_end < 1e-6)
        return {};

    const Point2LL& centroid = params.centroid;

    // Terminals need room for both loops plus at least one line width of straight chord
    // between them; otherwise fall back to a plain straight segment rather than emitting
    // self-overlapping Terminal geometry into a hole too small for it.
    const double min_terminal_span = (W + R1) * w_d + w_d;
    const bool draw_terminals = s_end >= min_terminal_span;

    ExtrusionLine fp_line(/*inset_idx=*/0, /*is_odd=*/false, /*is_closed=*/false);

    double current_s = 0.0;
    if (draw_terminals)
    {
        appendTerminal(fp_line, 0.0, +1.0, chord_arc, centroid, w, D, R1, R2, W, /*reversed=*/true);
        current_s = W * w_d;
    }

    // Contour matching (Spec REV 3.3): if the caller supplied a lofted point sequence (see
    // this function's header comment), the middle of the Line is exactly Start Terminal ->
    // contour_pts[0..N-1] -> End Terminal, with NO chord walk in between — contour_pts are
    // already absolute world-space points computed by the caller from the real wall contours
    // above/below this hole, not positions along chord_arc, so stitching them via the chord's
    // own coordinate frame would be meaningless. Falls back to the plain straight chord walk
    // (unchanged from before contour matching existed) when contour_pts is empty, or when the
    // hole is too narrow to draw Terminals at all (draw_terminals false) — contour_pts assumes
    // both Terminals are present to hand off to/from, and a hole that narrow is a degenerate
    // edge case regardless.
    if (! contour_pts.empty() && draw_terminals)
    {
        for (const Point2LL& p : contour_pts)
            fp_line.junctions_.emplace_back(p, w, 0);
    }
    else
    {
        const double end_walk_stop = draw_terminals ? s_end - R1 * w_d : s_end;
        appendPolySegment(fp_line, chord_arc, current_s, end_walk_stop, w, fp_line.empty());
    }

    if (draw_terminals)
        appendTerminal(fp_line, s_end, -1.0, chord_arc, centroid, w, D, R1, R2, W, /*reversed=*/false);

    if (fp_line.size() < 2)
        return {};

    // No weld stub: an earlier revision extended the Terminal's true endpoint (Q0/Q1) toward
    // the real wall corner (P0/P1) with a tangent-matched curve, tapered in near a hole's own
    // top/bottom. Removed after real-print testing found it made the Punchout materially
    // harder to break away — defeating the purpose of a removable support feature. The
    // intentional gap between Q0/Q1 and P0/P1 (featherprint_punchout_gap, above) is left
    // unwelded; Contour Matching (contour_pts, below/above) is what still makes the Line's
    // shape track the wall without needing the ends themselves to touch it.

    // Half-width ends (Spec REV 3.3): the Layer that forms the literal top of the hole's own
    // span, and the Layer that forms the literal bottom, print this entire Punchout
    // (Terminals included) at half the nominal line width — a discrete override on those two
    // Layers only, unrelated to Contour Matching, meant purely to weaken the connection and
    // make the whole feature easier to snap off. Geometry/shape is unaffected: only the
    // ExtrusionJunction width field (what actually controls the printed bead width) changes,
    // so R1/R2/D/W above are still derived from the full nominal w and the profile keeps its
    // normal proportions — just printed thinner.
    if (half_width_ends)
        for (ExtrusionJunction& j : fp_line)
            j.w_ = std::max<coord_t>(1, w / 2);

    VariableWidthLines result;
    result.push_back(std::move(fp_line));
    return result;
}

VariableWidthLines FeatherPrintGenerator::generateShelf(const Point2LL& Q0, const Point2LL& Q1, const Point2LL& centroid, const Settings& settings, int collar_ramp_peak)
{
    const coord_t w = settings.get<coord_t>("featherprint_line_width");
    if (w <= 0)
        return {};
    const double w_d = static_cast<double>(w);

    // Punchout Terminal's own W/R1/R2 (see generatePunchout()'s own use of the same values) --
    // reused verbatim, not re-derived, so the "available span" boundary below matches exactly
    // where the real Terminal geometry sits.
    const double stringer_W = settings.get<double>("featherprint_stringer_width");
    const double stringer_R2 = stringer_W / 2.0;
    const double stringer_R1 = stringer_R2 + 0.5;

    OpenPolyline chord_poly;
    chord_poly.push_back(Q0);
    chord_poly.push_back(Q1);
    ArcParam chord_arc = buildArcParamOpen(chord_poly);
    const double s_end = chord_arc.total;
    if (s_end < 2.0 * w_d)
        return {};

    const double span_start = stringer_W * w_d;
    const double span_end = s_end - stringer_R1 * w_d;
    const double available_span = span_end - span_start;
    if (available_span <= 0.0)
        return {};

    const int loop_count = static_cast<int>(std::floor(available_span / (stringer_W * w_d)));
    if (loop_count < 1)
        return {}; // hole too narrow for even one loop

    // Reach: the innermost Wall of the SAME Wall stack the upper Collar band above will use at
    // its own peak (buildFormerWallStack is Former's own rule, reused unmodified by Collar --
    // see buildFormerWallStack's own doc comment), less Lw/2 -- close enough to give that Wall
    // real contact to build on, without extending fully to or past it.
    const std::vector<FlangeWallDesc> collar_stack = buildFormerWallStack(collar_ramp_peak, w);
    if (collar_stack.empty())
        return {};
    const FlangeWallDesc& innermost = collar_stack.back();
    const double reach_w = (static_cast<double>(innermost.offset) + static_cast<double>(innermost.width) / 2.0) / w_d;
    const double D_shelf = reach_w - 0.5;
    if (D_shelf <= 0.0)
        return {}; // Collar band too thin at this ramp position to leave any real reach for a loop

    const double step = available_span / static_cast<double>(loop_count);

    // Each loop is one Stringer Trace crossover (appendTrace), laid out side by side along the
    // chord rather than stacked helically across Layers -- deliberately not a flat shelf (see
    // this feature's own spec doc comment on generateShelf, above): a row of small, localized
    // weld points gives the Collar's own Wall stack enough distributed contact to hold the
    // overhang during printing, while staying as breakable as the rest of Punchout. is_cw is
    // fixed false for every loop -- Shelf has no CCW/CW helix-pairing concept the way a real
    // Stringer does, so there is nothing to mirror.
    ExtrusionLine fp_line(/*inset_idx=*/0, /*is_odd=*/false, /*is_closed=*/false);
    double current_s = span_start;
    for (int i = 0; i < loop_count; i++)
    {
        const double s_anchor = span_start + (static_cast<double>(i) + 0.5) * step;
        const double s_depart = std::max(current_s, s_anchor - w_d);
        appendPolySegment(fp_line, chord_arc, current_s, s_depart, w, fp_line.empty());
        appendTrace(fp_line, s_anchor, centroid, chord_arc, /*is_cw=*/false, w, D_shelf, stringer_W, false);
        current_s = s_anchor + w_d;
    }
    appendPolySegment(fp_line, chord_arc, current_s, span_end, w, fp_line.empty());

    if (fp_line.size() < 2)
        return {};

    VariableWidthLines result;
    result.push_back(std::move(fp_line));
    return result;
}

} // namespace cura
