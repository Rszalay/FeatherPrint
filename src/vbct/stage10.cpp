#include "stage10.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>
#include <string>

namespace vbct {

// Not anonymous-namespace, unlike the rest of this file's helpers below: exposed via stage10.hpp
// so VbctAdapter::computeAnchorsForMesh's cross-layer continuity pre-pass (spec REV 1.4 S:5.3)
// can reuse this file's own wall-length/centroid/reference-angle logic for a fresh
// (no-previous-layer) anchor pick, instead of duplicating it. Otherwise unchanged from their
// original form; nothing about their behavior or their callers within this file differs.
double wall_length(const std::vector<Point2>& points) {
    double sum = 0;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        sum += std::hypot(points[i + 1].x - points[i].x, points[i + 1].y - points[i].y);
    }
    return sum;
}

// Orient so points[0] is whichever of the wall's own two endpoints is
// closer to `target` -- strict generalization of exact-match orientation
// that also handles a wall's two ends no longer necessarily converging
// (spec S:13.3 / S:11.3's dead-end midpoint cap revision).
//
// Not anonymous-namespace, like wall_length above: exposed via stage10.hpp so
// VbctAdapter::computeAnchorsForMesh's Chain end-position continuity tracking can orient a Chain
// domain's own left/right walls toward the same target this file's own prepare_domain_sampling
// uses, before running its own nearestPointOnWall search along them - the same "reuse this file's
// own logic instead of duplicating it" rationale wall_length/signed_area/match_winding/
// reference_angle_arc_length were already exported for.
std::vector<Point2> orient_toward(const std::vector<Point2>& points, const Point2& target) {
    double d0 = std::hypot(points.front().x - target.x, points.front().y - target.y);
    double d1 = std::hypot(points.back().x - target.x, points.back().y - target.y);
    if (d0 <= d1) return points;
    return std::vector<Point2>(points.rbegin(), points.rend());
}

// signed_area and match_winding, like wall_length above, are not anonymous-namespace: exposed via
// stage10.hpp so VbctAdapter::computeAnchorsForMesh can apply the *same* match_winding decision
// to the "other" wall before computing/tracking its own t0_frac, instead of computing that
// fraction against a point order domain_stringers itself may reverse - confirmed as a real,
// separate bug from the anchor-position tracking above: a real print showed ~95mm jumps at a
// single layer, symmetric around an anchor point that itself barely moved (<0.006 of a wrap)
// between that layer and its neighbors, which is exactly the signature of the *same* t0_frac
// value being interpreted against a wall walked in the opposite direction for one layer only
// (match_winding's own sign-comparison flipping on a near-tie), not a real anchor discontinuity.
double signed_area(const std::vector<Point2>& points) {
    double sum = 0;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        sum += points[i].x * points[i + 1].y - points[i + 1].x * points[i].y;
    }
    return sum / 2.0;
}

// A ring's two walls have no shared point to align t=0 on, but must still
// trace the same rotational direction -- spec S:13.3.
std::vector<Point2> match_winding(const std::vector<Point2>& left, const std::vector<Point2>& right) {
    if (signed_area(left) * signed_area(right) < 0) return std::vector<Point2>(right.rbegin(), right.rend());
    return right;
}

// polygon_centroid and reference_angle_arc_length, like wall_length above, are not
// anonymous-namespace: exposed via stage10.hpp for the same cross-layer continuity reason (spec
// REV 1.4 S:5.3) - otherwise unchanged, including all callers within this file.

// FeatherPrint Corrugated fix (2026-08-29): arc-length-weighted centroid, not a naive vertex
// average. A vertex average weights every *vertex* equally regardless of how much of the wall's
// actual boundary it represents - for a wall whose point density isn't uniform around the loop
// (confirmed happening in practice: VBS subdivision density varies with local edge length, and
// the Stage 9 hub-necklace merge splices together fragments that may carry different point
// densities of their own), the average gets pulled toward wherever points happen to be denser,
// which has nothing to do with the wall's actual geometric center. Confirmed directly: a plain,
// centered annulus's wall produced a vertex-average centroid over 10mm away from the true
// center on a 30mm-radius ring - large enough that the wall may no longer fully enclose it, at
// which point reference_angle_arc_length's crossing search can genuinely find no crossing at
// all over long stretches (not a rare edge case - confirmed as the *common* case, silently
// falling back to an uncorrected default almost everywhere, with the rare cases that do find a
// real crossing looking like the "anomaly" instead of the other way around). Weighting each
// point by the arc length it represents (half of each adjacent edge) fixes this: it's the same
// weighting principle this file already uses everywhere else (wall_length, sample_at_t), so a
// point that only represents a short stretch of boundary can no longer out-vote a point that
// represents a long one.
Point2 polygon_centroid(const std::vector<Point2>& points) {
    const int n = static_cast<int>(points.size()) - 1; // exclude the duplicated closing point
    if (n <= 0) return points.empty() ? Point2{0.0, 0.0} : points[0];

    std::vector<double> seg_len(n);
    for (int i = 0; i < n; ++i) {
        const Point2& p = points[i];
        const Point2& q = points[(i + 1) % n];
        seg_len[i] = std::hypot(q.x - p.x, q.y - p.y);
    }

    double cx = 0.0, cy = 0.0, total_weight = 0.0;
    for (int i = 0; i < n; ++i) {
        const double prev_len = seg_len[(i - 1 + n) % n];
        const double weight = 0.5 * (prev_len + seg_len[i]); // half of each adjacent edge
        cx += points[i].x * weight;
        cy += points[i].y * weight;
        total_weight += weight;
    }
    if (total_weight <= 0.0) return points[0]; // degenerate (all points coincide) - arbitrary but defined
    return {cx / total_weight, cy / total_weight};
}

// The continuous arc-length position (not a vertex index), measured from points[0] along
// `points`, at which the polygon boundary crosses the fixed reference direction (+X) from
// `centroid` - i.e. where the wall would meet t=0 if it were re-parametrized to start there.
//
// FeatherPrint Corrugated fix, replacing both the original closest-approach-between-walls
// alignment (spec S:13.3) and this project's own first attempt at fixing it (a per-wall
// nearest-vertex search - see git history). Both prior approaches choose a specific *vertex* as
// the t=0 anchor, which is a discrete, piecewise-constant choice: a real Ring wall's vertices
// come out of this project's own upstream Stages 2-4 (VBS subdivision + Constrained Delaunay
// Triangulation), and CDT is a classic numerically-degenerate case for the co-circular point
// sets a near-circular Ring wall naturally has - its core "is this point inside the circle
// through these three" test has near-zero margin for co-circular inputs, so floating-point
// noise alone can tip it either way and produce a genuinely different (still perfectly valid)
// triangulation topology for what is, at the wall level, identical geometry between layers.
// Confirmed in practice: raw wall vertices bit-for-bit identical across layers, yet the
// downstream Domain.left/right this function receives still occasionally differed enough to
// flip which vertex a nearest-vertex search picked, producing small but real per-layer "wobble"
// even after the first fix.
//
// A continuously-interpolated crossing point sidesteps vertex selection entirely: it moves
// smoothly with any small perturbation in the underlying triangulation instead of ever having
// to discretely choose between two candidate vertices, so there's no boundary left to
// (rarely, but visibly) land on the wrong side of. It's also still safe under this engine's
// parallel per-layer processing (run_multiple_producers_ordered_consumer,
// src/FffGcodeWriter.cpp) for the same reason as before: it depends only on this wall's own
// geometry, never on another layer's already-computed result.
//
// Reverted 2026-08-29: an unbiased global "nearest angle" search was tried here in place of the
// crossing search below, hoping to avoid the crossing-based all-or-nothing failure mode
// documented in the comment above (a real, if rare, near-cap instability - see
// VBCT-Ring-Chain-Misclassification-Brief.md and stage9.cpp's history for the misclassification
// bug that was masking it until fixed). It made things measurably worse: without the crossing
// search's implicit bias toward whichever candidate is nearest points[0] (it walks forward from
// arc-length 0 and returns the *first* hit, which is naturally stable), an unbiased global
// minimum has no such anchor and can get captured by different local near-candidates scattered
// around the wall - confirmed directly, jumping throughout the *entire* print, not just near
// the cap where the original problem was confined to a handful of layers. Reverted to the
// crossing search; the near-cap instability remains a known, accepted residual (see the brief),
// not solved by this round.
//
// If the boundary crosses the reference direction more than once (a non-star-shaped wall), the
// first crossing found while walking from points[0] wins - same implicit assumption the
// vertex-based approach made (a Ring wall has one clear candidate near any given direction).
// FeatherPrint Corrugated fix, round 3 (2026-08-29): the crossing test below used to check
// whether the running *unwrapped* angle crossed the literal value 0 - correct only when
// points[0]'s own starting angle happens to be positive (so a full sweep naturally passes
// through literal 0 on the way to -2*pi), but silently broken whenever points[0] starts
// negative: a full monotonic -360 degree sweep starting at, say, -179.95 degrees runs from
// -179.95 down to -539.95, which never touches literal 0 at all - it passes through -360
// (equivalent to 0 modulo 360, i.e. the true physical reference direction) instead. Confirmed
// directly: with the wall-start-point instability now fixed upstream (see
// VBCT-Wall-Start-Point-Instability-Brief.md), both walls' own points[0] land at essentially
// the same physical spot (~180 degrees from centroid) - but one wall's happened to round to
// +179.997 degrees and the other's to -179.952 degrees, an almost arbitrary hair's-width
// difference that decided whether this bug fired at all. Fix: check whether the running
// unwrapped angle crosses *any* multiple of 2*pi, not just literal 0.
double reference_angle_arc_length(const std::vector<Point2>& points, const Point2& centroid) {
    const double pi = std::acos(-1.0);
    const double two_pi = 2.0 * pi;
    auto signed_angle = [&](const Point2& p) {
        double a = std::atan2(p.y - centroid.y, p.x - centroid.x);
        while (a > pi) a -= two_pi;
        while (a <= -pi) a += two_pi;
        return a; // in (-pi, pi], i.e. signed angular distance from the +X reference direction
    };

    double acc = 0.0;
    double prev_delta = signed_angle(points[0]);
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        const double seg_len = std::hypot(points[i + 1].x - points[i].x, points[i + 1].y - points[i].y);
        double diff = signed_angle(points[i + 1]) - prev_delta;
        // Normalize so a wrap from just under +pi to just over -pi (or vice versa) reads as a
        // small step in the true rotational direction, not a spurious near-2pi jump.
        if (diff > pi) diff -= two_pi;
        if (diff < -pi) diff += two_pi;

        const double next_delta = prev_delta + diff; // unwrapped
        const double lo = std::min(prev_delta, next_delta);
        const double hi = std::max(prev_delta, next_delta);
        const double boundary = std::ceil(lo / two_pi) * two_pi; // nearest multiple of 2pi at or above lo
        if (boundary <= hi) {
            const double frac = (diff != 0.0) ? std::clamp((boundary - prev_delta) / diff, 0.0, 1.0) : 0.0;
            return acc + frac * seg_len;
        }
        acc += seg_len;
        prev_delta = next_delta;
    }
    return 0.0; // no crossing found (degenerate) - fall back to the wall's own start point
}

namespace {

// Wraps an arbitrary double (in particular, a negative one - std::fmod does not reliably land in
// [0, 1) for a negative dividend, e.g. fmod(-0.25, 1.0) == -0.25, not 0.75) into [0, 1). Needed by
// the crosshatch CW family's mirrored (negated) base_t below; every other user of fmod(x, 1.0) in
// this file only ever feeds it a non-negative x, so this helper is new rather than a fix to those.
double wrap01(double x) {
    double w = std::fmod(x, 1.0);
    if (w < 0.0) w += 1.0;
    return w;
}

// FeatherPrint Corrugated history note (spec REV 2.3 through spec REV 2.4's "fifth round"): Chain-
// specific Crosshatch originally slid a "rigid comb" of n points along the corridor, reflecting off
// both real ends (fold_bounce, a triangle wave - now removed) rather than wrapping like Ring's own
// crosshatch, on the theory that Chain's t=0/t=1 are real, physically distinct ends with nothing to
// wrap into. Four rounds of that construction (first: only the crosshatch family moved; second: both
// families moved as an exact mirror of each other, confirmed via real-print testing to make both
// reach their own turnaround extreme at the same Z but different physical positions - a visible
// tangled "bulge"; third: base family fixed again, rejected on sight for going static; fourth: the
// two families' own bounces decoupled by a quarter-cycle phase shift, which fixed the tangle but
// also capped their opposite-direction motion at only half the cycle, visibly weakening the X
// crossing effect) each traded one real, confirmed defect for another, all within the same
// "reflect at the ends" premise. Replaced entirely (see domain_stringers' and build_domain_events_
// for_domain's own Chain branches below) with Ring's own already-shipped construction instead -
// wrap01((i + phase_offset)/denominator) for one family, wrap01((i - phase_offset)/denominator)
// for the other (Crossover Pitch, 2026-09-15: phase_offset divided by each domain's own
// denominator here, rather than added directly, so the family-coincidence period is the same
// physical Z distance for every domain regardless of its own stringer count - see
// effective_phase_offset's own doc comment further down) - accepting a real, deliberate cost (a
// periodic jump in printed position, once per phase
// cycle, where a Chain's own t=0/t=1 are NOT actually the same physical point the way Ring's own
// wraparound assumes) in exchange for two benefits no version of the bounce ever had: the two
// families move in genuinely opposite directions 100% of the time (not just half), and coverage
// stays close to the full corridor at every phase (still not exactly guaranteed - min/max gap is
// bounded by roughly one sample spacing, small and worse only very near phase_offset=0 - see
// ChainCrosshatch's own coverage test in VbctAdapterTest.cpp) without any span/slack tuning at all.
//
// FeatherPrint Corrugated fix (spec REV 2.5): the wraparound family's own n samples, being evenly
// spaced around a *virtual closed loop*, structurally can never include both real corridor ends at
// once for any phase_offset - confirmed on a real print as the linked-skin path (and plain
// crosshatch stringers) stopping short of the domain's own end caps. Fixed by always adding two
// extra fixed points at the true t=0/t=1 ends, independent of phase - see the two "always add the
// domain's own true t=0/t=1 cap" additions in domain_stringers and build_domain_events_for_domain
// below for the actual fix.

Point2 sample_at_t(const std::vector<Point2>& points, double total_length, double t) {
    if (t <= 0.0) return points.front();
    if (t >= 1.0) return points.back();
    double target = t * total_length;
    double acc = 0.0;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        const Point2& p = points[i];
        const Point2& q = points[i + 1];
        double seg_len = std::hypot(q.x - p.x, q.y - p.y);
        if (acc + seg_len >= target) {
            double frac = (seg_len == 0.0) ? 0.0 : (target - acc) / seg_len;
            return {p.x + frac * (q.x - p.x), p.y + frac * (q.y - p.y)};
        }
        acc += seg_len;
    }
    return points.back();
}

std::vector<StringerEdge> boundary_edges(const std::string& kind, const std::vector<Point2>& left,
                                          const std::vector<Point2>& right) {
    if (kind == "ring") {
        std::vector<StringerEdge> edges;
        for (const auto* wall : {&left, &right}) {
            for (size_t i = 0; i + 1 < wall->size(); ++i) edges.emplace_back((*wall)[i], (*wall)[i + 1]);
        }
        return edges;
    }
    // Chain: one simple closed polygon walking left forward then right
    // backward -- spec S:13.5. No point dropped: the wrap-around join is
    // a real bridging edge across an uncorrugated sliver when the walls
    // diverge (S:11.3), or a harmless zero-length edge when they still
    // coincide.
    std::vector<Point2> loop = left;
    loop.insert(loop.end(), right.rbegin(), right.rend());
    std::vector<StringerEdge> edges;
    edges.reserve(loop.size());
    for (size_t i = 0; i < loop.size(); ++i) edges.emplace_back(loop[i], loop[(i + 1) % loop.size()]);
    return edges;
}

// Ray-casting even-odd rule, cast along +x -- same technique as Stage 4's
// classification, generalized to an arbitrary edge list.
bool point_in_domain(const Point2& point, const std::vector<StringerEdge>& edges) {
    bool odd = false;
    for (const auto& [a, b] : edges) {
        if ((a.y > point.y) != (b.y > point.y)) {
            double x_at_y = a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (x_at_y > point.x) odd = !odd;
        }
    }
    return odd;
}

// Parameter s in (0,1) along a1->a2 where it properly crosses b1->b2, or
// nullopt if they don't cross.
std::optional<double> segment_intersection_s(const Point2& a1, const Point2& a2, const Point2& b1, const Point2& b2) {
    double d1x = a2.x - a1.x, d1y = a2.y - a1.y;
    double d2x = b2.x - b1.x, d2y = b2.y - b1.y;
    double denom = d1x * d2y - d1y * d2x;
    if (std::abs(denom) < 1e-12) return std::nullopt;
    double diffx = b1.x - a1.x, diffy = b1.y - a1.y;
    double s = (diffx * d2y - diffy * d2x) / denom;
    double u = (diffx * d1y - diffy * d1x) / denom;
    constexpr double eps = 1e-9;
    if (s > eps && s < 1 - eps && u >= -eps && u <= 1 + eps) return s;
    return std::nullopt;
}

double round9(double s) {
    double scaled = s * 1e9;
    return static_cast<double>(python_round(scaled)) / 1e9;
}

Stringer clip_stringer(const Point2& v1t, const Point2& v2t, const std::vector<StringerEdge>& edges) {
    std::set<double> crossing_set;
    for (const auto& [b1, b2] : edges) {
        auto s = segment_intersection_s(v1t, v2t, b1, b2);
        if (s.has_value()) crossing_set.insert(round9(*s));
    }

    std::vector<double> bounds{0.0};
    bounds.insert(bounds.end(), crossing_set.begin(), crossing_set.end());
    bounds.push_back(1.0);

    double dx = v2t.x - v1t.x, dy = v2t.y - v1t.y;
    auto at = [&](double s) -> Point2 { return {v1t.x + s * dx, v1t.y + s * dy}; };

    Stringer result;
    for (size_t i = 0; i + 1 < bounds.size(); ++i) {
        double s0 = bounds[i], s1 = bounds[i + 1];
        if (s1 - s0 < 1e-9) continue;
        Point2 mid = at((s0 + s1) / 2.0);
        if (point_in_domain(mid, edges)) result.pieces.emplace_back(at(s0), at(s1));
    }
    return result;
}

// Everything domain_stringers and build_domain_events (stage10.hpp) both need before they can
// start sampling: the canonicalized walls, their t0 anchors, stringer count, and boundary edges.
// Factored out so these two functions - one producing independent stringer segments, the other
// producing a linked-skin crossing-event sequence - can never silently diverge on wall
// canonicalization, anchor overrides, or t0 computation, the exact class of bug this file's own
// git history is full of (see e.g. the comments on the length-canonicalization and shared-centroid
// fixes below - both were real, confirmed-in-production bugs from two pieces of code computing the
// same derived value slightly differently). Purely an internal factoring - not exposed via
// stage10.hpp - domain_stringers' own behavior is unchanged by this refactor (see its own body
// below, which reads these same fields under their same old local names).
struct DomainSampling {
    std::vector<Point2> left;
    std::vector<Point2> right;
    double left_len{ 0.0 };
    double right_len{ 0.0 };
    double left_t0_frac{ 0.0 };
    double right_t0_frac{ 0.0 };
    // Chain-only (spec REV 2.9-ish, "Chain domain end-position continuity"): the far-end
    // counterpart to left_t0_frac/right_t0_frac above, needed because a Chain's own t=0/t=1 are
    // two real, distinct wall caps (not a Ring's single wraparound anchor) - each end needs its
    // own independently-tracked position. Unused by Ring (always the default 1.0, never read by
    // Ring's own wraparound sampling).
    double left_t1_frac{ 1.0 };
    double right_t1_frac{ 1.0 };
    int n{ 2 };
    bool is_ring{ false };
    double effective_phase_offset{ 0.0 };
    // This domain's own real boundary edges only - never includes extra_clip_loops (below). Used
    // by build_domain_events_for_domain's own DomainEvent::clipped gate, which must keep meaning
    // exactly what it always meant before De Minimis Hole Threshold existed ("this crossing left
    // the domain's own real shape, Linked Corrugation Skin's v1 can't route around that") - not
    // "this crossing happens to pass through an ignored hole somewhere in the domain," which is
    // both expected and, unlike a real boundary exit, does not need to disable linking for the
    // whole domain (see hole_edges' own doc comment).
    std::vector<StringerEdge> edges;
    // De Minimis Hole Threshold (FeatherPrint Corrugated extension, spec REV 3.0/3.5/5.9): edges
    // from every currently-suppressed hole's own real boundary, kept separate from `edges` above
    // rather than merged into it. domain_stringers' own final stringer output (the unlinked
    // corrugate() path, which has no linkability gate to protect) clips against `edges` + these
    // combined, exactly as before this split. build_domain_events_for_domain deliberately does
    // NOT include these when deciding DomainEvent::clipped - see that field's own updated doc
    // comment for why, and VbctAdapter.cpp's own post-processing of the *finished* linked-skin
    // path for how a hole-crossing "rung" still ends up safe without disabling the whole domain's
    // link.
    std::vector<StringerEdge> hole_edges;
};

DomainSampling prepare_domain_sampling(
    const Domain& domain,
    double spacing,
    double phase_offset,
    std::optional<double> anchor_t0_frac,
    std::optional<double> other_wall_t0_frac,
    std::optional<bool> reverse_canonical_wall,
    std::optional<Point2> chain_anchor_point = std::nullopt,
    std::optional<double> chain_left_near_t_frac = std::nullopt,
    std::optional<double> chain_left_far_t_frac = std::nullopt,
    std::optional<double> chain_right_near_t_frac = std::nullopt,
    std::optional<double> chain_right_far_t_frac = std::nullopt,
    const std::vector<std::vector<Point2>>& extra_clip_loops = {},
    bool chain_swap_left_right = false) {
    std::vector<Point2> left = domain.left->points;
    std::vector<Point2> right = domain.right->points;

    // Chain wall outer/inner identity continuity (VBCT sign-oscillation investigation) - see
    // CorrugationAnchor::chain_swap_left_right's own doc comment for the full rationale. Unlike
    // Ring (canonicalized to the longer wall, just below), a Chain's two walls are close enough in
    // length that length alone can't discriminate - the caller (VbctAdapter::computeAnchorsForMesh)
    // has already determined, by cross-layer identity tracking, whether this layer's own
    // domain.left is the same physical wall the previous tracked layer called "left". Applied
    // before any other Chain-specific logic below, so everything downstream (orient_toward,
    // per-wall t0/t1) sees the identity-stable assignment.
    if (domain.kind == "chain" && chain_swap_left_right) {
        std::swap(left, right);
    }

    // FeatherPrint Corrugated fix: domain.left/domain.right's assignment (which physical wall -
    // typically outer vs. inner for a Ring - gets called "left" vs. "right") comes from this
    // project's own upstream Stage 8/9 domain classification and is not guaranteed stable
    // layer-to-layer - confirmed happening in practice on a real print (wall_length(left)
    // alternating between the outer and inner wall's own length across otherwise-unremarkable
    // layers, not just near any particular feature). Every per-wall computation below
    // (reference_angle_arc_length, t0_frac, sampling) is applied independently to "left" and
    // "right", so when their identity flips, "left" suddenly means the other physical wall,
    // producing a large spurious jump unrelated to anything about that layer's real geometry -
    // this is very likely the actual root cause behind most of the residual wobble investigated
    // this session, not something specific to VBS/CDT reconstruction noise near a cap. Ring
    // walls are almost never equal-length (a concentric Ring's outer/inner circumferences
    // differ by 2*pi*(gap width), a Chain's walls only match by coincidence), so canonicalizing
    // "left" to always be the longer wall - before any alignment math runs - gives a stable,
    // upstream-order-independent semantic with no dependency on Stage 8/9's own bookkeeping.
    if (domain.kind == "ring" && wall_length(left) < wall_length(right)) {
        std::swap(left, right);
    }

    // Cross-layer *direction* continuity (spec REV 1.4 S:5.3) - see reverse_canonical_wall's own
    // doc comment in stage10.hpp. Applied immediately after the length-based canonicalization
    // above and before match_winding, so match_winding's own decision (which depends on left's
    // sign) sees the direction-corrected wall, exactly mirroring what
    // VbctAdapter::computeAnchorsForMesh itself does when determining this override value.
    if (domain.kind == "ring" && reverse_canonical_wall.has_value() && *reverse_canonical_wall) {
        std::reverse(left.begin(), left.end());
    }

    // Ring's own t=0 reference, as a continuous arc-length offset per wall (see
    // reference_angle_arc_length above) rather than a rotated array - computed here, before
    // wall_length below, since it needs each wall in its original (post match_winding) point
    // order.
    //
    // FeatherPrint Corrugated fix: both walls are measured against the *same* shared reference
    // point (the outer/longer wall's own centroid, from the canonicalization above), not each
    // wall's own independent centroid. A Ring's two walls need not be concentric - an off-center
    // hole is completely ordinary geometry - so "angle 0 from its own centroid" on each wall
    // independently can point in unrelated physical directions when the walls aren't concentric,
    // producing a nonsensical correspondence between the two anchor points (confirmed breaking
    // badly on an off-center test). A single shared reference point keeps both walls' anchors
    // measured the same way, giving a sensible (if imperfect for extreme eccentricity)
    // radial-ish correspondence regardless of concentricity, while still keeping the
    // continuous-interpolation fix (see reference_angle_arc_length) that avoids ever snapping
    // to a single unstable vertex.
    double left_t0 = 0.0, right_t0 = 0.0;
    // Cross-layer continuity (spec REV 1.4 S:5.3): when the caller has already picked this
    // layer's wall anchor(s) by nearest-point tracking against the previous layer, use them
    // directly instead of reference_angle_arc_length's own fresh, purely-per-layer search - that
    // search is exactly what can't stay stable under sub-visual input noise (see this function's
    // caller, VbctAdapter::computeAnchorsForMesh, and the brief it's built from). Both walls are
    // tracked independently (each against its own previous-layer anchor) - an earlier version of
    // this fix tracked only the canonical (left) wall, leaving the other (right) wall's own
    // still-per-layer-absolute anchor free to keep producing the identical class of discontinuity
    // on its own; confirmed on a real print (zero topology-change resets logged, yet
    // orientation-dependent jumps persisted - see other_wall_t0_frac's own doc comment).
    bool left_t0_overridden = false;
    bool right_t0_overridden = false;
    if (domain.kind == "chain") {
        // Cross-layer continuity for Chain (spec REV 2.1 "Chain domain support"): when the caller
        // has already picked this layer's "same real end" point by nearest-point tracking against
        // the previous layer's own chosen point, orient both walls toward *that* instead of a
        // fresh domain.cap_start every layer. Unlike Ring, no separate direction override is
        // needed here - orient_toward's own nearest-endpoint search already is the direction fix,
        // once given a stable target (see chain_anchor_point's own doc comment in stage10.hpp for
        // why domain.cap_start alone can't be trusted to mean "the same physical end" across
        // layers: far_cap()'s assignment, stage9.cpp, has no cross-layer memory of its own).
        const Point2& target = chain_anchor_point.has_value() ? *chain_anchor_point : *domain.cap_start;
        left = orient_toward(left, target);
        right = orient_toward(right, target);
    } else {
        right = match_winding(left, right);
        const Point2 shared_centroid = polygon_centroid(left);
        if (anchor_t0_frac.has_value()) {
            left_t0_overridden = true;
        } else {
            left_t0 = reference_angle_arc_length(left, shared_centroid);
        }
        if (other_wall_t0_frac.has_value()) {
            right_t0_overridden = true;
        } else {
            right_t0 = reference_angle_arc_length(right, shared_centroid);
        }
    }

    double left_len = wall_length(left);
    double right_len = wall_length(right);
    double left_t0_frac;
    double right_t0_frac;
    double left_t1_frac = 1.0;
    double right_t1_frac = 1.0;
    if (domain.kind == "chain") {
        // Cross-layer *end-position* continuity for Chain (FeatherPrint Corrugated extension):
        // orient_toward above only fixes *which* end of each wall is near (the direction fix,
        // per this branch's own comment) - it says nothing about *where exactly* that end's own
        // raw vertex sits, which is free to jitter layer to layer under the same per-layer-
        // absolute VBCT noise Ring's own anchor_t0_frac/other_wall_t0_frac already exist to damp
        // (Stage 9's far_cap() has no cross-layer memory of its own). The caller
        // (VbctAdapter::computeAnchorsForMesh) is expected to have already found, independently
        // for each wall, the arc-length fraction nearest the previous layer's own tracked near
        // and far points (nearestPointOnWall) - both ends of both walls, since Chain's own t=0/
        // t=1 are two real, distinct wall caps, unlike Ring's single wraparound anchor. Defaults
        // (0.0 near, 1.0 far) reproduce today's exact pre-tracking behavior - the full wall, raw
        // endpoint to raw endpoint - on the first tracked layer or after any reset.
        left_t0_frac = chain_left_near_t_frac.value_or(0.0);
        right_t0_frac = chain_right_near_t_frac.value_or(0.0);
        left_t1_frac = chain_left_far_t_frac.value_or(1.0);
        right_t1_frac = chain_right_far_t_frac.value_or(1.0);
    } else {
        left_t0_frac = left_t0_overridden ? *anchor_t0_frac : ((left_len > 0.0) ? left_t0 / left_len : 0.0);
        right_t0_frac = right_t0_overridden ? *other_wall_t0_frac : ((right_len > 0.0) ? right_t0 / right_len : 0.0);
    }
    double governing_length = std::max(left_len, right_len);
    int n = 2;
    if (spacing > 0) {
        n = std::max(2, static_cast<int>(python_round(governing_length / spacing)) + 1);
    }
    auto edges = boundary_edges(domain.kind, left, right);

    // De Minimis Hole Threshold (FeatherPrint Corrugated extension, spec REV 3.0/3.5/5.9): kept in
    // a separate edge list from `edges` above, not merged into it - see DomainSampling::hole_edges'
    // own doc comment for why (build_domain_events_for_domain's own DomainEvent::clipped gate must
    // not fire on a hole crossing the way it correctly does on a real domain-boundary exit).
    // clip_stringer/point_in_domain (stage10.cpp) are both pure even-odd parity tests over a flat,
    // undirected edge list, with no connectivity/winding assumptions at all - appending a
    // suppressed-for-classification hole's own closed-loop edges to whichever list a given call
    // site actually uses is therefore sufficient by construction to make its interior register as
    // "outside the domain" for clipping purposes there, with no change needed to either function
    // itself. Which domain a given hole is physically inside isn't known until after
    // classification, so the caller passes every currently-suppressed hole to every domain
    // uniformly - harmless (a no-op) for a domain that doesn't actually contain the hole in
    // question, avoiding a more complex domain-to-hole containment resolution step.
    std::vector<StringerEdge> hole_edges;
    for (const std::vector<Point2>& loop : extra_clip_loops) {
        for (size_t i = 0; i < loop.size(); ++i) {
            hole_edges.emplace_back(loop[i], loop[(i + 1) % loop.size()]);
        }
    }

    // Chain is excluded from phase shift: its t=0/t=1 are real distinct wall ends, so wrapping
    // past t=1 back to t=0 would jump across the domain rather than continuing smoothly.
    //
    // A workaround applying phase_offset to Chain too lived here from 2026-08-29, through two
    // iterations of VBCT's Stage 9 merge_hub_necklaces fix (spec REV 2.3's original hub-degree
    // gate, then its bridge-count generalization - see VBCT-Ring-Chain-Misclassification-Brief.md
    // "Update 2" for what the first attempt missed). Reverted now that a genuine ring should no
    // longer misclassify as Chain here; if a misclassified Ring is ever observed again, that's
    // new information, not a reason to re-apply this workaround blindly.
    const bool is_ring = (domain.kind == "ring");
    const double effective_phase_offset = is_ring ? phase_offset : 0.0;

    return DomainSampling{
        std::move(left), std::move(right), left_len, right_len, left_t0_frac, right_t0_frac,
        left_t1_frac, right_t1_frac, n, is_ring, effective_phase_offset, std::move(edges), std::move(hole_edges)
    };
}

DomainStringers domain_stringers(
    const Domain& domain,
    double spacing,
    double phase_offset,
    std::optional<double> anchor_t0_frac,
    std::optional<double> other_wall_t0_frac,
    std::optional<bool> reverse_canonical_wall,
    bool crosshatch_enabled,
    std::optional<Point2> chain_anchor_point = std::nullopt,
    std::optional<double> chain_left_near_t_frac = std::nullopt,
    std::optional<double> chain_left_far_t_frac = std::nullopt,
    std::optional<double> chain_right_near_t_frac = std::nullopt,
    std::optional<double> chain_right_far_t_frac = std::nullopt,
    const std::vector<std::vector<Point2>>& extra_clip_loops = {},
    bool chain_swap_left_right = false) {
    const DomainSampling s = prepare_domain_sampling(
        domain, spacing, phase_offset, anchor_t0_frac, other_wall_t0_frac, reverse_canonical_wall, chain_anchor_point,
        chain_left_near_t_frac, chain_left_far_t_frac, chain_right_near_t_frac, chain_right_far_t_frac, extra_clip_loops,
        chain_swap_left_right);
    const std::vector<Point2>& left = s.left;
    const std::vector<Point2>& right = s.right;
    const double left_len = s.left_len;
    const double right_len = s.right_len;
    const double left_t0_frac = s.left_t0_frac;
    const double right_t0_frac = s.right_t0_frac;
    const double left_t1_frac = s.left_t1_frac;
    const double right_t1_frac = s.right_t1_frac;
    const int n = s.n;
    const bool is_ring = s.is_ring;
    const double effective_phase_offset = s.effective_phase_offset;
    // De Minimis Hole Threshold (spec REV 3.0/3.5/5.9): this function's own output (the unlinked
    // corrugate() path) has no linkability gate to protect - unlike build_domain_events_for_domain
    // - so it always needs full protection against both the domain's own real boundary and every
    // ignored hole's own real boundary. Combined here, not inside prepare_domain_sampling itself -
    // see DomainSampling::hole_edges' own doc comment for why the two stay separate there.
    std::vector<StringerEdge> edges = s.edges;
    edges.insert(edges.end(), s.hole_edges.begin(), s.hole_edges.end());

    // FeatherPrint Corrugated fix: a Ring's t=0 and t=1 land on the exact same physical point
    // (points.front() == points.back() for a closed loop), so sampling i/(n-1) over i in
    // [0, n-1] - upstream VBCT's own formula, correct for Chain's real distinct wall ends -
    // double-counts that seam point as two coincident, zero-benefit stringers for a Ring. Under
    // any phase_offset both still land on the exact same point (fmod(0+offset,1) ==
    // fmod(1+offset,1) == offset), so the duplicate pair just relocates around the ring rather
    // than resolving - the optimizer then sometimes strings the pair back-to-back, printing the
    // same span twice, which read as "malformed" doubled lines. Dividing by n instead of n-1
    // for Ring spaces samples evenly around the true n-way wrap with no repeated point; Chain
    // keeps the original n-1 (inclusive-endpoint) spacing since its ends are real, distinct wall
    // caps that should be sampled exactly.
    const double denominator = is_ring ? static_cast<double>(n) : static_cast<double>(n - 1);

    // Chain crosshatch's own wraparound sampling (below) needs n, not n-1 - see its own doc
    // comment at the t_left/t_right formula for why reusing n-1 there reproduces the exact
    // duplicate-endpoint bug the Ring-vs-Chain denominator split above already exists to avoid,
    // once wrapping (not clamping) is in play. Only used by the crosshatch-active formulas.
    const double chain_wrap_denominator = static_cast<double>(n);

    // Crosshatch (spec Section 3.1, "two counter-rotating helix families (CCW/CW)") - the CW
    // family reuses the exact same per-layer construction as the CCW family below (same forward
    // index i, same left_t0_frac/right_t0_frac, no reflection of either), with only the sign of
    // effective_phase_offset flipped for *both* walls together. Confirmed with the user (two
    // earlier, wrong attempts ruled this construction in by elimination - see git history for
    // both): the CW family's own per-layer (right - left) tilt is therefore identical to the CCW
    // family's, not mirrored - within a single layer the two families are the same repeating
    // diagonal shape, just carried at a different absolute rotational position around the ring
    // (offset by 2*effective_phase_offset/denominator, in theta-space - see below). The crossing
    // this produces is a genuinely 3D, over-Z effect, not a same-layer one: as Z rises,
    // effective_phase_offset grows, so the CCW family's whole pattern sweeps one rotational
    // direction around the ring while the CW family's sweeps the other - two helical bands
    // winding oppositely, crossing repeatedly over the print's height, the same way two
    // oppositely-wound helices on a lattice tower or gridshell cross. This means crosshatch has
    // no visible effect within an isolated layer, or anywhere effective_phase_offset is 0
    // (crossover pitch disabled, or this exact Z happens to land on a whole or half wrap) -
    // accepted and expected, not a bug: with no Z-phase progression there is no helix at all for
    // a second one to counter-rotate against. crosshatch_active below gates the whole per-index
    // loop on this, rather than repeating an equivalent per-i check n times - whether the two
    // families coincide at this domain's current effective_phase_offset doesn't depend on i at
    // all (the phase term cancels identically out of every index's own comparison).
    //
    // Crossover Pitch (2026-09-15, Multi-Domain crossover-synchronization design): effective_
    // phase_offset is now the *raw*, undivided crossover-phase argument (z_mm / (2 * pitch_mm),
    // shared unchanged across every domain in the mesh - see VbctAdapter.cpp's own computation)
    // rather than an already-per-domain-scaled theta value. Dividing it by this domain's own
    // `denominator` here - the same division every base_t/base_t_cw formula below now performs
    // by computing (i +- effective_phase_offset) / denominator instead of i / denominator +-
    // effective_phase_offset - is what makes the crosshatch coincidence period 1/(2*(1/(2*pitch)))
    // == pitch for *every* domain regardless of its own stringer count, instead of the old
    // per-domain-N-dependent period (1 / (2*N*phase_rate)) that made different domains reach
    // their first coincidence at different Z heights even though every domain shared the same
    // raw phase_offset - confirmed on real capture (Multi-Domain Two-Hole.stl) as the actual
    // mechanism behind three Chain domains crossing over at different, uncoordinated layers.
    double phase_shift_wrapped = wrap01(2.0 * effective_phase_offset / denominator);
    phase_shift_wrapped = std::min(phase_shift_wrapped, 1.0 - phase_shift_wrapped); // shortest distance around the wrap
    const bool crosshatch_active = crosshatch_enabled && is_ring && phase_shift_wrapped > 1e-9;

    // Chain-specific Crosshatch (spec REV 2.3 through spec REV 2.4's "fifth round" - see this
    // file's own top-of-file history note for the four earlier rounds and why each was replaced):
    // a second stringer family, built from Ring's own already-shipped crosshatch construction just
    // above (wrap01(i/denominator ± phase_offset)) rather than the earlier "rigid sliding comb that
    // reflects off both real ends" this file used to build. Chosen deliberately over any reflecting
    // construction after direct user report asked why Chain crosshatch needed to bounce at all when
    // Ring's own doesn't - the answer being that Chain's own t=0/t=1 are two real, physically
    // distinct ends (not the same point the way Ring's own wraparound is), so wrapping means a real
    // jump in printed position once per phase cycle, at whichever index's own wrapped t crosses the
    // domain's true boundary - a deliberate, accepted cost, not hidden or minimized, in exchange for
    // both families moving in genuinely opposite directions 100% of the time (unlike every "bounce"
    // round, which topped out at 50% at best) and full-ish coverage at every phase with no
    // span/slack tuning needed at all (a small residual gap, worst near phase_offset=0 where both
    // formulas coincide - see this family's own even-spacing test in VbctAdapterTest.cpp).
    //
    // Uses the raw phase_offset parameter directly (not effective_phase_offset, which
    // prepare_domain_sampling forces to 0.0 for Chain specifically so the *base* family's own
    // ordinary, non-crosshatch anchor stays fixed).
    const bool chain_crosshatch_active = crosshatch_enabled && !is_ring;

    // FeatherPrint Corrugated fix (spec REV 2.3): a Chain stringer sampled exactly at t=0 or t=1
    // runs along the same boundary edge boundary_edges() used to build this domain's own edge list
    // for clip_stringer's point-in-domain test below - a self-intersection test against the
    // domain's own edge, not a genuine "did this leave the domain" question. Floating-point noise
    // there can spuriously judge the whole (very short, cap-adjacent) segment as outside, silently
    // dropping a stringer that structurally can't have left the domain at all - confirmed directly
    // (not assumed) via a real test failure: the base family's own i=n-1 stringer (t=1 exactly)
    // dropped out, breaking an otherwise-exact stringer count. Mirrors build_domain_events_for_
    // domain's own identical cap-event exemption (stage10.cpp, Chain domain support) - applied here
    // too since domain_stringers has the same clip_stringer call, just never exercised precisely
    // enough at t=0/1 to surface it until Chain crosshatch's own test needed an exact count.
    // Exempts both the base family (t=0/1 at i=0/n-1 exactly) and Chain crosshatch (whose own
    // wrapped output can also land exactly on a cap for some phase_offset).
    auto is_chain_cap_t = [is_ring](double t) { return !is_ring && (t <= 1e-9 || t >= 1.0 - 1e-9); };
    auto make_chain_stringer = [&](double t, const Point2& v1t, const Point2& v2t) {
        if (is_chain_cap_t(t)) {
            Stringer stringer;
            stringer.pieces.emplace_back(v1t, v2t);
            return stringer;
        }
        return clip_stringer(v1t, v2t, edges);
    };

    DomainStringers result;
    for (int i = 0; i < n; ++i) {
        const double base_t = (static_cast<double>(i) + effective_phase_offset) / denominator;
        // Ring: each wall is sampled from its own continuous t=0 offset (left_t0_frac /
        // right_t0_frac), not a shared array-rotated index - see reference_angle_arc_length
        // above. Chain: both offsets are 0.0 (left/right were oriented to the shared cap_start
        // above instead), so this collapses to the original shared-t behavior.
        //
        // FeatherPrint Corrugated fix (Chain domain support): fmod's wraparound is only correct
        // for Ring, whose t=0 and t=1 are the same physical point by construction - for Chain,
        // t=1 is a real, distinct point (cap_end), and std::fmod(1.0, 1.0) == 0.0 silently
        // collapsed the very last stringer's sample back onto t=0 (duplicating the first stringer
        // and never actually sampling the domain's true closing end at all). Only Ring uses fmod's
        // wraparound now; Chain clamps into [0, 1] instead (a no-op for every value this loop ever
        // actually produces for Chain, since base_t is already exactly i/(n-1) with both t0
        // offsets fixed at 0.0 - this only changes the i = n-1 boundary case from 0.0 to the
        // intended 1.0).
        //
        // Chain, when chain_crosshatch_active: overridden to Ring's own wraparound formula instead
        // (this family's own doc comment above) - both walls still share the same t, exactly as the
        // always-fixed case does, just wrapping (advancing forward with phase_offset) instead of
        // staying constant.
        //
        // FeatherPrint Corrugated fix (spec REV 2.4, "fifth round"): uses chain_wrap_denominator
        // (== n, NOT denominator == n-1) - confirmed via a real test failure that reusing n-1 here
        // reproduces the exact class of bug Ring's own n-vs-n-1 choice above already exists to
        // avoid: with n-1, index i=0 and i=n-1 are exactly 1.0 apart in i/denom terms, so
        // wrap01(0/(...)+phase) and wrap01((n-1)/(n-1)+phase) == wrap01(1+phase) collapse to the
        // *same* wrapped value - a genuine duplicate stringer, not the two real, distinct samples
        // n-1 is supposed to guarantee. n avoids this exactly the way it already does for Ring.
        //
        // t_left and t_right are always equal when chain_crosshatch_active (the formula has no
        // left_t0_frac/right_t0_frac term to tell them apart, same as Chain's always-fixed case
        // below) - computed once into t_chain_wrap rather than calling wrap01 twice for an
        // identical result.
        const double t_chain_wrap = chain_crosshatch_active
            ? wrap01((static_cast<double>(i) + phase_offset) / chain_wrap_denominator) : 0.0;
        // Chain, base family (chain_crosshatch_active false): linearly interpolates between this
        // wall's own tracked near (left_t0_frac/right_t0_frac) and far (left_t1_frac/
        // right_t1_frac) fractions instead of the old "shift by t0_frac, clamp to [0,1]" formula -
        // see DomainSampling::left_t1_frac's own doc comment and prepare_domain_sampling's Chain
        // branch for why a single additive shift can't independently stabilize both of Chain's
        // real, distinct wall ends the way this does. base_t already runs 0..1 across i=0..n-1 for
        // Chain (effective_phase_offset forced 0), so this reproduces exactly today's t=0..1 full
        // wall span whenever near/far are left at their defaults (0.0/1.0).
        const double t_left = is_ring ? std::fmod(base_t + left_t0_frac, 1.0)
            : (chain_crosshatch_active ? t_chain_wrap : (left_t0_frac + base_t * (left_t1_frac - left_t0_frac)));
        const double t_right = is_ring ? std::fmod(base_t + right_t0_frac, 1.0)
            : (chain_crosshatch_active ? t_chain_wrap : (right_t0_frac + base_t * (right_t1_frac - right_t0_frac)));
        Point2 v1t = sample_at_t(left, left_len, t_left);
        Point2 v2t = sample_at_t(right, right_len, t_right);
        Stringer stringer = make_chain_stringer(t_left, v1t, v2t);
        if (!stringer.pieces.empty()) result.stringers.push_back(std::move(stringer));

        // Crosshatch's CW stringer at this same index - see crosshatch_active's own doc comment
        // above for the construction (same left_t0_frac/right_t0_frac as the CCW stringer above,
        // only effective_phase_offset's sign flipped for both walls).
        if (crosshatch_active) {
            const double base_t_cw = (static_cast<double>(i) - effective_phase_offset) / denominator;
            const double t_left_cw = wrap01(base_t_cw + left_t0_frac);
            const double t_right_cw = wrap01(base_t_cw + right_t0_frac);
            Point2 v1t_cw = sample_at_t(left, left_len, t_left_cw);
            Point2 v2t_cw = sample_at_t(right, right_len, t_right_cw);
            Stringer stringer_cw = clip_stringer(v1t_cw, v2t_cw, edges);
            if (!stringer_cw.pieces.empty()) result.stringers.push_back(std::move(stringer_cw));
        }

        // Chain-specific Crosshatch's own second stringer at this same index - see
        // chain_crosshatch_active's own doc comment above for the wraparound construction (Ring's
        // own formula, sign-flipped: advancing backward instead of forward, so the two families
        // move in genuinely opposite directions). Both walls sample from the same t (Chain's own
        // base family already does this too, since left_t0_frac/right_t0_frac are always 0.0 for
        // Chain).
        if (chain_crosshatch_active) {
            const double t_chain_cw = wrap01((static_cast<double>(i) - phase_offset) / chain_wrap_denominator);
            Point2 v1t_chain_cw = sample_at_t(left, left_len, t_chain_cw);
            Point2 v2t_chain_cw = sample_at_t(right, right_len, t_chain_cw);
            Stringer stringer_chain_cw = make_chain_stringer(t_chain_cw, v1t_chain_cw, v2t_chain_cw);
            if (!stringer_chain_cw.pieces.empty()) result.stringers.push_back(std::move(stringer_chain_cw));
        }
    }

    // FeatherPrint Corrugated fix (spec REV 2.5): the n wraparound-sampled points above, evenly
    // spaced by 1/chain_wrap_denominator (== n) around a *virtual* closed loop, can never include
    // both t=0 and t=1 at once for any phase_offset - by construction they only ever span
    // [phase_offset/n, phase_offset/n + (n-1)/n] (mod 1). That's correct for Ring, whose t=0 and
    // t=1 are the same physical point, but Chain's t=1 is a real, distinct end - so at every phase
    // value, at least one (often both) of Chain crosshatch's own real ends falls short of full
    // coverage by a real, structural amount (confirmed via direct numeric check: up to 1/n of the
    // corridor length, not a rare edge case) - the "stops short of the endcaps" defect reported on
    // a real print. The two lines below always add the domain's own true t=0/t=1 cap stringers,
    // independent of phase - cheap, constant additions that don't perturb the wraparound family's
    // own spacing at all, restoring the same "always touches both real ends" property the ordinary
    // non-crosshatch Chain family already has for free (its own i=0/i=n-1 samples land exactly on
    // t=0/t=1, since it never uses the wraparound formula).
    if (chain_crosshatch_active) {
        result.stringers.push_back(make_chain_stringer(0.0, left.front(), right.front()));
        result.stringers.push_back(make_chain_stringer(1.0, left.back(), right.back()));
    }
    return result;
}

// Builds one domain's crossing-event sequence for the linked corrugation skin - see
// build_domain_events' own doc comment in stage10.hpp for the full design (cyclic sort order for
// Ring, forced-even count for Ring, whole-domain coincidence detection for Ring, and Chain's own
// simpler open-path case, which needs none of the three). Returns DomainEvents{ok=false} for every
// Ring case this feature doesn't support: crosshatch_enabled false (no CW family to alternate
// against at all), or (defensively, should not happen given prepare_domain_sampling always returns
// n>=2) a degenerate stringer count - Chain domains have no such requirement, see below.
DomainEvents build_domain_events_for_domain(
    const Domain& domain,
    double spacing,
    double phase_offset,
    std::optional<double> anchor_t0_frac,
    std::optional<double> other_wall_t0_frac,
    std::optional<bool> reverse_canonical_wall,
    bool crosshatch_enabled,
    std::optional<Point2> chain_anchor_point = std::nullopt,
    bool chain_crosshatch_enabled = false,
    std::optional<double> chain_left_near_t_frac = std::nullopt,
    std::optional<double> chain_left_far_t_frac = std::nullopt,
    std::optional<double> chain_right_near_t_frac = std::nullopt,
    std::optional<double> chain_right_far_t_frac = std::nullopt,
    const std::vector<std::vector<Point2>>& extra_clip_loops = {},
    bool chain_swap_left_right = false) {
    DomainEvents result;
    if (domain.kind == "chain") {
        // Chain domain support (spec REV 2.1): no forced-even count - see this function's own
        // leading doc comment and build_domain_events' own "Chain domains" note in stage10.hpp for
        // why an open wall pair's own family/families, walked cap to cap, need none.
        const DomainSampling s = prepare_domain_sampling(
            domain, spacing, phase_offset, anchor_t0_frac, other_wall_t0_frac, reverse_canonical_wall, chain_anchor_point,
            chain_left_near_t_frac, chain_left_far_t_frac, chain_right_near_t_frac, chain_right_far_t_frac, extra_clip_loops,
            chain_swap_left_right);
        if (s.n < 2 || s.left_len <= 0.0) return result;  // defensive

        // Chain-specific Crosshatch + Linked Corrugation Skin integration (spec REV 2.4/"fifth
        // round"): reuses the exact same wraparound construction domain_stringers' own unlinked
        // output builds (this file's own top-of-file history note), rather than re-deriving it, so
        // a part sliced once with Crosshatch+Linked Skin and once with just Crosshatch shows the
        // same underlying sample positions.
        const bool chain_crosshatch_active = chain_crosshatch_enabled;
        const double denom = static_cast<double>(s.n - 1);  // matches domain_stringers' own Chain denominator
        // Matches domain_stringers' own chain_wrap_denominator (== n, not n-1) - see that
        // variable's own doc comment for why n-1 collides i=0 and i=n-1 into the same wrapped value.
        // Only used by the crosshatch-active wraparound formulas below.
        const double chain_wrap_denom = static_cast<double>(s.n);

        // Cap-event clip exemption (spec REV 2.3's own fix, applied here too): a Chain event
        // sampled exactly at t=0 or t=1 runs along the same boundary edge boundary_edges' own Chain
        // construction uses to bridge that cap - a self-intersection test against the domain's own
        // edge, not a genuine "did this leave the domain" question. Mirrors domain_stringers' own
        // is_chain_cap_t/make_chain_stringer exemption exactly, applied per-event here instead of
        // per-stringer since this function builds DomainEvent, not Stringer.
        auto is_chain_cap_t = [](double t) { return t <= 1e-9 || t >= 1.0 - 1e-9; };
        auto make_event = [&](double t_left, double t_right, bool is_cw) {
            const Point2 outer_pt = sample_at_t(s.left, s.left_len, t_left);
            const Point2 inner_pt = sample_at_t(s.right, s.right_len, t_right);
            // "clipped" means genuinely, entirely outside the domain (clip_stringer finds no piece
            // at all) - not merely "crosses the boundary somewhere along the way" (pieces.size()
            // == 2+, i.e. still has a real inside portion). Both outer_pt/inner_pt are themselves
            // real points on the domain's own walls - a straight chord between them grazing a
            // local concavity for part of its length is a minor rendering imprecision (the walk
            // still draws the raw outer_pt-inner_pt line either way), not a genuine "this crossing
            // doesn't belong to this domain" failure. Confirmed via real production geometry
            // (VBCT no-skin-links investigation): two evenly-spaced, ordinary mid-wall events
            // (t~0.20/0.80, not near either cap) on a real Chain domain's own upper curve were
            // rejected under the old `!= 1` test purely from a brief local graze, bailing the
            // whole layer's linked skin even though both endpoints, and most of the chord between
            // them, were genuinely inside.
            const bool clipped = ! is_chain_cap_t(t_left) && clip_stringer(outer_pt, inner_pt, s.edges).pieces.empty();
            // theta is just t_left here - already monotonically increasing with i *within* each
            // family (left's own near/far fraction range is always traversed monotonically), so
            // the sort below is what actually interleaves the two families into one walk-ready
            // order when both are built.
            return DomainEvent{ t_left, outer_pt, inner_pt, t_left, t_right, is_cw, clipped };
        };

        std::vector<DomainEvent> events;
        events.reserve(static_cast<size_t>(chain_crosshatch_active ? 2 * s.n + 2 : s.n));
        for (int i = 0; i < s.n; ++i) {
            // Base family: Ring's own wraparound formula when active (wrap01(i/chain_wrap_denom +
            // phase_offset), both walls sharing the same t - see this domain kind's own doc comment
            // in domain_stringers), or, when not, each wall's own near/far fraction range
            // interpolated independently over i/denom (denom == n-1, matching domain_stringers' own
            // Chain base denominator) - see that function's own comment on its t_left/t_right
            // formula for why a single additive shift can't do this.
            const double t_left = chain_crosshatch_active
                ? wrap01((static_cast<double>(i) + phase_offset) / chain_wrap_denom)
                : (s.left_t0_frac + (static_cast<double>(i) / denom) * (s.left_t1_frac - s.left_t0_frac));
            const double t_right = chain_crosshatch_active
                ? wrap01((static_cast<double>(i) + phase_offset) / chain_wrap_denom)
                : (s.right_t0_frac + (static_cast<double>(i) / denom) * (s.right_t1_frac - s.right_t0_frac));
            events.push_back(make_event(t_left, t_right, /*is_cw=*/false));
        }
        if (chain_crosshatch_active) {
            // Crosshatch family: Ring's own wraparound formula, sign-flipped (advancing backward
            // instead of forward) - see this domain kind's own doc comment in domain_stringers.
            // is_cw=true purely as a diagnostic/provenance marker here (unlike Ring, Chain's walk
            // doesn't branch on it) - mirrors domain_stringers' own second family exactly.
            for (int i = 0; i < s.n; ++i) {
                const double t = wrap01((static_cast<double>(i) - phase_offset) / chain_wrap_denom);
                events.push_back(make_event(t, t, /*is_cw=*/true));
            }

            // FeatherPrint Corrugated fix (spec REV 2.5): the n wraparound-sampled events above,
            // evenly spaced by 1/chain_wrap_denom around a *virtual* closed loop, can never include
            // both t=0 and t=1 at once for any phase_offset - see domain_stringers' own matching fix
            // and doc comment for the full root-cause explanation (this is the same structural gap,
            // confirmed on a real print as the linked-skin path stopping short of the domain's own
            // end caps at some layers). Always add the two true cap events, independent of phase -
            // the sort below places them in their correct position (first and last) automatically,
            // so no special-casing of the walk itself is needed. These are genuinely new points, not
            // a deletion/reorder of existing ones - unlike the reverted dedup attempt noted below,
            // this only ever shifts which wall governs each subsequent leg by a fixed two positions,
            // a legitimate consequence of two new real crossings existing.
            events.push_back(make_event(0.0, 0.0, /*is_cw=*/false));
            events.push_back(make_event(1.0, 1.0, /*is_cw=*/false));
        }

        // Unlike the single-family case, theta is no longer monotonic in build order once a second
        // family is merged in (its own wrapped t can land anywhere relative to the base family's
        // own) - this sort is load-bearing here, not a no-op safety net, interleaving both families
        // into the single ascending-t order the walk needs.
        //
        // FeatherPrint Corrugated note (history, now describing a fully-replaced construction - see
        // this file's own top-of-file history note): a version of this function briefly dropped the
        // later of any two adjacent near-coincident events here, on the theory that the walk's own
        // near-zero hop-and-cross-back at each such pair was an "errant bounce" notch a real print
        // showed. Reverted, confirmed via direct user report to make the render substantially
        // *worse* (a chaotic mesh of extra connecting lines, not a cleanup): removing events from
        // the middle of the list shifts buildLinkedSkinPathForDomain's own k%2 wall-alternation
        // parity for everything downstream, since that alternation is driven purely by position in
        // the list, not by anything intrinsic to an event - deleting from a parity-driven walk is
        // not the locally-scoped change it looks like. The notches' own real cause was never
        // isolated under the old bounce-based construction; whether they still occur under this
        // file's current wraparound construction is an open question for real-print testing.
        //
        // FeatherPrint Corrugated fix (2026-09-15, Multi-Domain Two-Hole crossing-glitch
        // investigation): std::sort -> std::stable_sort. The base and crosshatch families are two
        // counter-rotating helices that periodically sweep through exact alignment (confirmed on
        // real capture: every base-family theta bit-identical to its crosshatch-family
        // counterpart at these layers) - a real, expected geometric event, not a numerical
        // accident. std::sort gives no defined order for tied elements, so which of the two
        // tied events landed first could vary in a way inconsistent with the surrounding
        // (non-tied) events' own monotonic order - confirmed on real capture as a genuine
        // backtrack in the built path at exactly these layers (visits point A, then B, then back
        // to A, then forward past B to C), not merely a duplicate point. std::stable_sort
        // preserves this vector's own construction order (base family, then crosshatch family,
        // then the two cap events - each already monotonic on its own) for any tie, so a tied
        // cluster always resolves the same consistent way instead of whatever order the
        // unstable-sort implementation's own partitioning happened to produce.
        std::stable_sort(events.begin(), events.end(), [](const DomainEvent& a, const DomainEvent& b) { return a.theta < b.theta; });

        result.ok = true;
        result.is_ring = false;
        result.events = std::move(events);
        result.outer_wall = s.left;
        result.inner_wall = s.right;
        return result;
    }

    if (domain.kind != "ring" || !crosshatch_enabled) return result;  // ok stays false

    const DomainSampling s
        = prepare_domain_sampling(domain, spacing, phase_offset, anchor_t0_frac, other_wall_t0_frac, reverse_canonical_wall, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, extra_clip_loops);
    if (s.n < 2 || s.left_len <= 0.0) return result;  // defensive - see this function's own doc comment

    // Forced even so alternating on every crossing closes into one consistent loop (a 2-coloring
    // of a cycle graph only works on an even cycle) - see build_domain_events' own doc comment in
    // stage10.hpp. Independent of, and not shared with, domain_stringers/run_stage10's own n.
    const int n_even = s.n + (s.n % 2);
    const double denom = static_cast<double>(n_even);

    auto make_event = [&](int i, bool is_cw) {
        const double base_t = (static_cast<double>(i) + (is_cw ? -s.effective_phase_offset : s.effective_phase_offset)) / denom;
        const double theta = wrap01(base_t);
        const double t_left = wrap01(base_t + s.left_t0_frac);
        const double t_right = wrap01(base_t + s.right_t0_frac);
        const Point2 outer_pt = sample_at_t(s.left, s.left_len, t_left);
        const Point2 inner_pt = sample_at_t(s.right, s.right_len, t_right);
        // "clipped" means genuinely, entirely outside the domain - see the Chain event builder's
        // own identical fix and comment above for the full rationale (a chord that merely grazes a
        // local concavity for part of its length, pieces.size() >= 2, still has a real inside
        // portion and draws the same raw line regardless; only pieces.empty() is a real failure).
        const Stringer probe = clip_stringer(outer_pt, inner_pt, s.edges);
        const bool clipped = probe.pieces.empty();
        return DomainEvent{ theta, outer_pt, inner_pt, t_left, t_right, is_cw, clipped };
    };

    std::vector<DomainEvent> events;
    events.reserve(static_cast<size_t>(2 * n_even));
    for (int i = 0; i < n_even; ++i) events.push_back(make_event(i, false));
    for (int i = 0; i < n_even; ++i) events.push_back(make_event(i, true));

    // No coincidence detection/dedup here (an earlier version dropped the whole CW family when a
    // single representative index's CCW/CW pair landed close together, on the theory - correct in
    // exact arithmetic, per the bijection j = i+m mod n_even - that this generalized to every other
    // index too; confirmed by direct user report that this either wasn't reliable under real
    // floating-point evaluation or wasn't the actual mechanism behind "stringers skipped near
    // crossings" in the first place). Per the user's own direction: always emit both families and
    // always alternate walls at every single crossing, unconditionally - a genuinely coincident
    // pair just produces a near-zero-length arc immediately followed by a crossing back, not a
    // dropped stringer. Simpler, and removes an entire mechanism that only checked one
    // representative pair per domain rather than every pair.
    //
    // FeatherPrint Corrugated fix (2026-09-15): std::sort -> std::stable_sort - see the Chain
    // event builder's own identical fix and comment above for the full rationale (confirmed via
    // real Chain-domain capture: an unstable sort's undefined tie order, at a genuine CCW/CW
    // theta coincidence, produced a real backtrack in the built path, not merely a duplicate
    // point). Same construction-order argument applies here: CCW family (i=0..n_even-1) built
    // before CW family, each independently monotonic in theta.
    std::stable_sort(events.begin(), events.end(), [](const DomainEvent& a, const DomainEvent& b) { return a.theta < b.theta; });

    if (events.size() % 2 != 0) return result;  // defensive - see this function's own doc comment

    result.ok = true;
    result.is_ring = true;
    result.events = std::move(events);
    result.outer_wall = s.left;
    result.inner_wall = s.right;
    return result;
}

}  // namespace

Stage10Result run_stage10(
    const Stage9Result& stage9,
    double spacing,
    double phase_offset,
    std::optional<double> anchor_t0_frac,
    std::optional<double> other_wall_t0_frac,
    std::optional<bool> reverse_canonical_wall,
    bool crosshatch_enabled,
    std::optional<Point2> chain_anchor_point,
    std::optional<double> chain_left_near_t_frac,
    std::optional<double> chain_left_far_t_frac,
    std::optional<double> chain_right_near_t_frac,
    std::optional<double> chain_right_far_t_frac,
    const std::vector<std::vector<Point2>>& extra_clip_loops,
    bool chain_swap_left_right,
    bool transition_ring,
    const std::vector<Point2>& transition_chain_domain_identities,
    double transition_solid_fill_spacing) {
    Stage10Result result;
    bool anchor_applied = false;
    bool chain_anchor_applied = false;
    // Transition Layer (spec REV 3.3/3.6/5.10) - see run_stage10's own header doc for why this
    // tolerance only needs to absorb re-derivation noise between two passes over the same
    // geometry, not real cross-layer drift.
    constexpr double kTransitionDomainMatchCapMm = 2.0;
    for (const auto& d : stage9.domains) {
        if (d.kind != "ring" && d.kind != "chain") continue;
        // Only the first Ring domain gets the continuity override - see this function's own
        // header doc for why (the caller tracks exactly one anchor per mesh per layer). Chain
        // domain support (spec REV 2.1): same one-anchor-per-mesh-per-layer rule, tracked
        // independently since a layer's composition is either a Ring or a Chain, never both. The
        // four chain_*_t_frac overrides (Chain end-position continuity) share this exact same
        // apply_chain_anchor_here gate as chain_anchor_point - all five are the same single
        // tracked Chain domain's own continuity state. extra_clip_loops (De Minimis Hole
        // Threshold) is deliberately *not* gated the same way - every domain gets the same full
        // set of currently-suppressed holes, since which domain a given hole is physically inside
        // isn't known ahead of time (see prepare_domain_sampling's own comment on this).
        const bool apply_anchor_here = anchor_t0_frac.has_value() && ! anchor_applied && d.kind == "ring";
        if (apply_anchor_here) anchor_applied = true;
        const bool apply_chain_anchor_here = chain_anchor_point.has_value() && ! chain_anchor_applied && d.kind == "chain";
        if (apply_chain_anchor_here) chain_anchor_applied = true;

        // Transition Layer (spec REV 3.3/3.6/5.10): identify whether *this* domain is the one the
        // caller wants substituted with full-density solid fill this layer - see run_stage10's own
        // header doc. Unlike the anchor overrides above (at most one Ring, one Chain, applied to
        // the *first* match found), more than one Chain domain may independently transition at
        // once - matched fresh for every domain, not latched after the first hit.
        bool is_transition = false;
        if (d.kind == "ring" && transition_ring) {
            is_transition = true;
        } else if (d.kind == "chain" && ! transition_chain_domain_identities.empty()) {
            double sum_x = 0.0, sum_y = 0.0;
            size_t n = 0;
            for (const auto* wall : { &d.left, &d.right }) {
                for (const Point2& p : (*wall)->points) {
                    sum_x += p.x;
                    sum_y += p.y;
                    ++n;
                }
            }
            if (n > 0) {
                const Point2 mean{ sum_x / static_cast<double>(n), sum_y / static_cast<double>(n) };
                for (const Point2& target : transition_chain_domain_identities) {
                    const double dx = mean.x - target.x;
                    const double dy = mean.y - target.y;
                    if (dx * dx + dy * dy <= kTransitionDomainMatchCapMm * kTransitionDomainMatchCapMm) {
                        is_transition = true;
                        break;
                    }
                }
            }
        }
        const double effective_spacing = is_transition ? transition_solid_fill_spacing : spacing;
        const bool effective_crosshatch = is_transition ? false : crosshatch_enabled;

        result.domains.push_back(domain_stringers(
            d,
            effective_spacing,
            phase_offset,
            apply_anchor_here ? anchor_t0_frac : std::nullopt,
            apply_anchor_here ? other_wall_t0_frac : std::nullopt,
            apply_anchor_here ? reverse_canonical_wall : std::nullopt,
            effective_crosshatch,
            apply_chain_anchor_here ? chain_anchor_point : std::nullopt,
            apply_chain_anchor_here ? chain_left_near_t_frac : std::nullopt,
            apply_chain_anchor_here ? chain_left_far_t_frac : std::nullopt,
            apply_chain_anchor_here ? chain_right_near_t_frac : std::nullopt,
            apply_chain_anchor_here ? chain_right_far_t_frac : std::nullopt,
            extra_clip_loops,
            apply_chain_anchor_here ? chain_swap_left_right : false));
        result.domains.back().is_transition_layer = is_transition;
    }
    return result;
}

std::vector<DomainEvents> build_domain_events(
    const Stage9Result& stage9,
    double spacing,
    double phase_offset,
    std::optional<double> anchor_t0_frac,
    std::optional<double> other_wall_t0_frac,
    std::optional<bool> reverse_canonical_wall,
    bool crosshatch_enabled,
    std::optional<Point2> chain_anchor_point,
    bool chain_crosshatch_enabled,
    std::optional<double> chain_left_near_t_frac,
    std::optional<double> chain_left_far_t_frac,
    std::optional<double> chain_right_near_t_frac,
    std::optional<double> chain_right_far_t_frac,
    const std::vector<std::vector<Point2>>& extra_clip_loops,
    bool chain_swap_left_right) {
    std::vector<DomainEvents> result;
    bool anchor_applied = false;
    bool chain_anchor_applied = false;
    for (const auto& d : stage9.domains) {
        if (d.kind != "ring" && d.kind != "chain") continue;
        // Same "only the first Ring/Chain domain gets the continuity override" rule as
        // run_stage10's own loop above, for the same reason - see its own comment. extra_clip_loops
        // (De Minimis Hole Threshold) is not gated the same way - see run_stage10's own comment.
        const bool apply_anchor_here = anchor_t0_frac.has_value() && ! anchor_applied && d.kind == "ring";
        if (apply_anchor_here) anchor_applied = true;
        const bool apply_chain_anchor_here = chain_anchor_point.has_value() && ! chain_anchor_applied && d.kind == "chain";
        if (apply_chain_anchor_here) chain_anchor_applied = true;
        result.push_back(build_domain_events_for_domain(
            d,
            spacing,
            phase_offset,
            apply_anchor_here ? anchor_t0_frac : std::nullopt,
            apply_anchor_here ? other_wall_t0_frac : std::nullopt,
            apply_anchor_here ? reverse_canonical_wall : std::nullopt,
            crosshatch_enabled,
            apply_chain_anchor_here ? chain_anchor_point : std::nullopt,
            chain_crosshatch_enabled,
            apply_chain_anchor_here ? chain_left_near_t_frac : std::nullopt,
            apply_chain_anchor_here ? chain_left_far_t_frac : std::nullopt,
            apply_chain_anchor_here ? chain_right_near_t_frac : std::nullopt,
            apply_chain_anchor_here ? chain_right_far_t_frac : std::nullopt,
            extra_clip_loops,
            apply_chain_anchor_here ? chain_swap_left_right : false));
    }
    return result;
}

}  // namespace vbct
