// Copyright (c) 2026 FeatherPrint Corrugated
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "corrugated/WallStrip.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace cura
{

namespace
{

// Shoelace-formula magnitude, used only to compare relative sizes (a hole's own contour is always
// strictly smaller than the outer contour it sits inside), not for a signed-winding-direction
// decision - std::abs makes this indifferent to which way each contour happens to wind.
double enclosedArea(const ExtrusionLine& line)
{
    double sum = 0.0;
    const std::vector<ExtrusionJunction>& junctions = line.junctions_;
    for (size_t i = 0; i + 1 < junctions.size(); ++i)
    {
        sum += static_cast<double>(junctions[i].p_.X) * static_cast<double>(junctions[i + 1].p_.Y)
             - static_cast<double>(junctions[i + 1].p_.X) * static_cast<double>(junctions[i].p_.Y);
    }
    return std::abs(sum) / 2.0;
}

} // namespace

bool stripRingWalls(std::vector<VariableWidthLines>& wall_toolpaths, bool strip_wall_a, bool strip_wall_b)
{
    if (! strip_wall_a && ! strip_wall_b)
    {
        return false;
    }
    if (wall_toolpaths.empty())
    {
        return false;
    }

    // Fail-safe pre-check (see this function's own header doc): every inset depth must look like a
    // simple, single-hole Ring - exactly two closed ExtrusionLines - or this call is a complete
    // no-op, leaving wall_toolpaths untouched.
    for (const VariableWidthLines& inset : wall_toolpaths)
    {
        if (inset.size() != 2 || ! inset[0].is_closed_ || ! inset[1].is_closed_)
        {
            return false;
        }
    }

    bool stripped_anything = false;
    for (VariableWidthLines& inset : wall_toolpaths)
    {
        // Larger enclosed area = the outer contour (Wall A); the other = the hole (Wall B) - valid
        // because a hole is by definition strictly inside its own part's outer boundary, at every
        // inset depth independently, so no cross-depth identity tracking is needed.
        const bool line0_is_outer = enclosedArea(inset[0]) >= enclosedArea(inset[1]);
        const size_t outer_idx = line0_is_outer ? 0 : 1;
        const size_t inner_idx = line0_is_outer ? 1 : 0;

        std::vector<ExtrusionLine> kept;
        kept.reserve(2);
        if (! strip_wall_a)
        {
            kept.push_back(std::move(inset[outer_idx]));
        }
        else
        {
            stripped_anything = true;
        }
        if (! strip_wall_b)
        {
            kept.push_back(std::move(inset[inner_idx]));
        }
        else
        {
            stripped_anything = true;
        }
        inset = std::move(kept);
    }
    return stripped_anything;
}

namespace
{

double edgeLength(const Point2LL& a, const Point2LL& b)
{
    return std::hypot(static_cast<double>(b.X - a.X), static_cast<double>(b.Y - a.Y));
}

ExtrusionJunction interpolateJunction(const ExtrusionJunction& p, const ExtrusionJunction& q, double frac)
{
    frac = std::clamp(frac, 0.0, 1.0);
    const Point2LL pt(
        p.p_.X + static_cast<coord_t>(std::llround(frac * static_cast<double>(q.p_.X - p.p_.X))),
        p.p_.Y + static_cast<coord_t>(std::llround(frac * static_cast<double>(q.p_.Y - p.p_.Y))));
    const coord_t w = static_cast<coord_t>(std::llround(static_cast<double>(p.w_) + frac * static_cast<double>(q.w_ - p.w_)));
    return ExtrusionJunction(pt, w, p.perimeter_index_);
}

// A closed ExtrusionLine's own unique vertices (dropping the duplicated closing point Cura's own
// convention adds - see WallToolPaths::simplifyToolPaths' own "line_.front() != line_.back()"
// re-closing step), each one's own cumulative arc-length position measured from pts[0], and the
// loop's own total length (including the closing edge back from the last unique vertex to pts[0])
// - the shared basis every function below builds on, computed once per candidate line rather than
// repeatedly.
struct ClosedLineGeometry
{
    std::vector<ExtrusionJunction> pts; // unique vertices; pts.size() == line.size() - 1 for a valid closed line
    std::vector<double> pos; // pos[k] = arc length from pts[0] to pts[k], pos.size() == pts.size()
    double total_length{ 0.0 };
};

ClosedLineGeometry analyzeClosedLine(const ExtrusionLine& line)
{
    ClosedLineGeometry g;
    if (line.size() < 3)
    {
        return g;
    }
    g.pts.assign(line.begin(), std::prev(line.end())); // drop the duplicated closing junction
    const size_t n = g.pts.size();
    g.pos.resize(n);
    double acc = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        g.pos[i] = acc;
        acc += edgeLength(g.pts[i].p_, g.pts[(i + 1) % n].p_);
    }
    g.total_length = acc;
    return g;
}

// Nearest arc-length position on `g` to `target`, plus the squared distance there - projects onto
// every edge, including the closing edge back from the last unique vertex to pts[0].
std::pair<double, double> nearestPositionAndDistSq(const ClosedLineGeometry& g, const Point2LL& target)
{
    double best_pos = 0.0;
    double best_dist_sq = std::numeric_limits<double>::max();
    const size_t n = g.pts.size();
    for (size_t i = 0; i < n; ++i)
    {
        const Point2LL& p = g.pts[i].p_;
        const Point2LL& q = g.pts[(i + 1) % n].p_;
        const double dx = static_cast<double>(q.X - p.X);
        const double dy = static_cast<double>(q.Y - p.Y);
        const double seg_len_sq = dx * dx + dy * dy;
        double t = 0.0;
        if (seg_len_sq > 0.0)
        {
            t = (static_cast<double>(target.X - p.X) * dx + static_cast<double>(target.Y - p.Y) * dy) / seg_len_sq;
            t = std::clamp(t, 0.0, 1.0);
        }
        const double cx = static_cast<double>(p.X) + t * dx;
        const double cy = static_cast<double>(p.Y) + t * dy;
        const double ddx = cx - static_cast<double>(target.X);
        const double ddy = cy - static_cast<double>(target.Y);
        const double dist_sq = ddx * ddx + ddy * ddy;
        if (dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            best_pos = g.pos[i] + t * std::sqrt(seg_len_sq);
        }
    }
    return { best_pos, best_dist_sq };
}

// Interpolated junction at arc-length position `pos` along `g`, wrapped into [0, total_length).
ExtrusionJunction junctionAt(const ClosedLineGeometry& g, double pos)
{
    const size_t n = g.pts.size();
    pos = std::fmod(pos, g.total_length);
    if (pos < 0.0)
    {
        pos += g.total_length;
    }
    for (size_t i = 0; i < n; ++i)
    {
        const double seg_start = g.pos[i];
        const double seg_end = (i + 1 < n) ? g.pos[i + 1] : g.total_length;
        if (pos <= seg_end || i + 1 == n)
        {
            const double seg_len = seg_end - seg_start;
            const double frac = (seg_len > 0.0) ? (pos - seg_start) / seg_len : 0.0;
            return interpolateJunction(g.pts[i], g.pts[(i + 1) % n], frac);
        }
    }
    return g.pts.front();
}

// Walks forward (increasing arc length, wrapping past the loop's own total length back to 0 if
// `to_pos` is "before" `from_pos`) from `from_pos` to `to_pos` along `g`, collecting every
// original vertex strictly in between plus interpolated junctions at both ends - the open arc that
// survives after removing everything else. `line` supplies inset_idx_/is_odd_ for the result;
// `g` must have been built from that same `line`.
ExtrusionLine collectArc(const ExtrusionLine& line, const ClosedLineGeometry& g, double from_pos, double to_pos)
{
    ExtrusionLine result(line.inset_idx_, line.is_odd_, /*is_closed=*/false);
    from_pos = std::fmod(from_pos, g.total_length);
    if (from_pos < 0.0)
    {
        from_pos += g.total_length;
    }
    to_pos = std::fmod(to_pos, g.total_length);
    if (to_pos < 0.0)
    {
        to_pos += g.total_length;
    }
    double distance_to_walk = to_pos - from_pos;
    if (distance_to_walk < 0.0)
    {
        distance_to_walk += g.total_length;
    }

    result.emplace_back(junctionAt(g, from_pos));
    const size_t n = g.pts.size();
    size_t idx = 0;
    while (idx < n && g.pos[idx] <= from_pos)
    {
        ++idx;
    }
    for (size_t step = 0; step < n; ++step)
    {
        const size_t real_idx = idx % n;
        double forward_pos = g.pos[real_idx] - from_pos;
        if (forward_pos < 0.0)
        {
            forward_pos += g.total_length;
        }
        if (forward_pos >= distance_to_walk)
        {
            break;
        }
        result.emplace_back(g.pts[real_idx]);
        ++idx;
    }
    result.emplace_back(junctionAt(g, to_pos));
    return result;
}

// Given the arc-length positions (on the same candidate line) that a domain's own sampled wall
// points projected onto, finds the domain's own covered range by locating the single largest gap
// between consecutive (sorted, wraparound-aware) positions - everything *outside* that gap is
// where the domain's samples actually cluster, so it's the covered range; the gap itself is "the
// rest of the shared wall," left untouched. Returns the *kept* (untouched) arc's own
// [from_pos, to_pos), to be walked forward via collectArc - deliberately returning the kept range
// directly rather than the removed one, since collectArc's own "walk forward" semantics already
// produce the survivor without needing a separate complement step.
std::pair<double, double> findKeptArcFromLargestGap(std::vector<double> positions, double total_length)
{
    std::sort(positions.begin(), positions.end());
    const size_t n = positions.size();
    double best_gap = (positions.front() + total_length) - positions.back(); // the wrap gap, checked first
    size_t best_gap_start_idx = n - 1; // gap starts right after positions[n-1] (wraps to positions[0])
    for (size_t i = 0; i + 1 < n; ++i)
    {
        const double gap = positions[i + 1] - positions[i];
        if (gap > best_gap)
        {
            best_gap = gap;
            best_gap_start_idx = i;
        }
    }
    // The largest gap IS the kept (untouched) arc: [positions[best_gap_start_idx], positions[(best_gap_start_idx+1) % n]).
    return { positions[best_gap_start_idx], positions[(best_gap_start_idx + 1) % n] };
}

double wrapPos(double pos, double total_length)
{
    pos = std::fmod(pos, total_length);
    if (pos < 0.0)
    {
        pos += total_length;
    }
    return pos;
}

// Whether arc-length position `pos` lies within the forward-walked range [from, to) (wrapping past
// total_length if to < from, same convention collectArc's own from_pos/to_pos already use).
bool isWithinRange(double pos, double from, double to, double total_length)
{
    from = wrapPos(from, total_length);
    to = wrapPos(to, total_length);
    pos = wrapPos(pos, total_length);
    double span = to - from;
    if (span < 0.0)
    {
        span += total_length;
    }
    double rel = pos - from;
    if (rel < 0.0)
    {
        rel += total_length;
    }
    return rel < span;
}

// Splits `original` (a closed line, described by `g`) into however many open kept arcs remain
// after removing every range in `removed_ranges` (each a forward-walked [from, to) pair, same
// convention as collectArc) - 0 arcs if the ranges together cover the whole loop, 1 for the usual
// single-side-stripped case, 2+ when multiple separated ranges are removed from the same loop at
// once (both Wall A and Wall B stripped on a simple corridor, where they're two different arcs of
// the *same* physical loop). Works by collecting every range's own two boundary positions as "cut
// points" (at most 4 for the two-sides-stripped case this project actually needs), testing each
// resulting inter-cut segment's own midpoint for coverage, then walking around the circle once
// (starting from a known-removed segment, so an "all kept" wrap doesn't need special-casing) to
// group consecutive kept segments into contiguous arcs.
std::vector<ExtrusionLine> splitByRemovedRanges(const ExtrusionLine& original, const ClosedLineGeometry& g, const std::vector<std::pair<double, double>>& removed_ranges)
{
    std::vector<double> cuts;
    cuts.reserve(removed_ranges.size() * 2);
    for (const auto& [from, to] : removed_ranges)
    {
        cuts.push_back(wrapPos(from, g.total_length));
        cuts.push_back(wrapPos(to, g.total_length));
    }
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
    const size_t nc = cuts.size();
    if (nc == 0)
    {
        return {};
    }

    std::vector<bool> seg_removed(nc);
    for (size_t i = 0; i < nc; ++i)
    {
        const double a = cuts[i];
        const double b = cuts[(i + 1) % nc];
        double span = b - a;
        if (span < 0.0)
        {
            span += g.total_length;
        }
        const double mid = wrapPos(a + span / 2.0, g.total_length);
        bool removed = false;
        for (const auto& [from, to] : removed_ranges)
        {
            if (isWithinRange(mid, from, to, g.total_length))
            {
                removed = true;
                break;
            }
        }
        seg_removed[i] = removed;
    }

    size_t ref = nc;
    for (size_t i = 0; i < nc; ++i)
    {
        if (seg_removed[i])
        {
            ref = i;
            break;
        }
    }
    if (ref == nc)
    {
        // Nothing actually removed at any of the cut-derived segments - shouldn't happen given the
        // caller only reaches here with a genuinely nonempty removed_ranges, but fail safe by
        // reporting no change rather than guessing.
        return {};
    }

    std::vector<ExtrusionLine> result;
    size_t i = (ref + 1) % nc;
    size_t scanned = 0;
    while (scanned < nc)
    {
        if (seg_removed[i])
        {
            i = (i + 1) % nc;
            ++scanned;
            continue;
        }
        const size_t run_start = i;
        size_t run_len = 0;
        while (scanned < nc && ! seg_removed[i])
        {
            i = (i + 1) % nc;
            ++run_len;
            ++scanned;
        }
        ExtrusionLine arc = collectArc(original, g, cuts[run_start], cuts[(run_start + run_len) % nc]);
        if (arc.size() >= 2)
        {
            result.push_back(std::move(arc));
        }
    }
    return result;
}

} // namespace

bool stripWallArcLengthRanges(std::vector<VariableWidthLines>& wall_toolpaths, const std::vector<std::vector<Point2LL>>& domain_wall_point_sets)
{
    std::vector<const std::vector<Point2LL>*> valid_sets;
    for (const std::vector<Point2LL>& set : domain_wall_point_sets)
    {
        if (set.size() >= 2)
        {
            valid_sets.push_back(&set);
        }
    }
    if (valid_sets.empty() || wall_toolpaths.empty())
    {
        return false;
    }

    // Fail-safe dry run first (see this function's own header doc): find every set's own
    // best-matching contour and covered range at every inset depth without mutating anything, and
    // only commit if every single set succeeds everywhere. Sets are grouped by whichever contour
    // each one *individually* matches, not required to all share one - multiple domains (or
    // multiple sides of one domain) can land on the same shared contour (grouped and cut together,
    // the same multi-range removal already needed for one domain's own two sides) or on genuinely
    // different contours (handled as separate, independent groups) without either case forcing the
    // other to fail.
    struct PlannedGroup
    {
        size_t line_idx{ 0 };
        ClosedLineGeometry geometry;
        std::vector<std::pair<double, double>> removed_ranges;
    };
    std::vector<std::vector<PlannedGroup>> plan(wall_toolpaths.size()); // one group list per inset depth

    for (size_t depth = 0; depth < wall_toolpaths.size(); ++depth)
    {
        const VariableWidthLines& inset = wall_toolpaths[depth];
        std::vector<PlannedGroup>& groups = plan[depth];

        for (const std::vector<Point2LL>* set_ptr : valid_sets)
        {
            const std::vector<Point2LL>& domain_wall_points = *set_ptr;
            std::optional<size_t> best_line_idx;
            ClosedLineGeometry best_geometry;
            std::vector<double> best_positions;
            double best_avg_dist_sq = std::numeric_limits<double>::max();

            for (size_t li = 0; li < inset.size(); ++li)
            {
                if (! inset[li].is_closed_)
                {
                    continue;
                }
                const ClosedLineGeometry g = analyzeClosedLine(inset[li]);
                if (g.pts.size() < 3 || g.total_length <= 0.0)
                {
                    continue;
                }
                std::vector<double> positions;
                positions.reserve(domain_wall_points.size());
                double sum_dist_sq = 0.0;
                for (const Point2LL& target : domain_wall_points)
                {
                    const auto [pos, dist_sq] = nearestPositionAndDistSq(g, target);
                    positions.push_back(pos);
                    sum_dist_sq += dist_sq;
                }
                const double avg_dist_sq = sum_dist_sq / static_cast<double>(domain_wall_points.size());
                if (avg_dist_sq < best_avg_dist_sq)
                {
                    best_avg_dist_sq = avg_dist_sq;
                    best_line_idx = li;
                    best_geometry = g;
                    best_positions = std::move(positions);
                }
            }
            if (! best_line_idx.has_value())
            {
                return false; // no closed contour at this depth at all - fail safe
            }

            std::sort(best_positions.begin(), best_positions.end());
            best_positions.erase(std::unique(best_positions.begin(), best_positions.end()), best_positions.end());
            if (best_positions.size() < 2)
            {
                return false; // domain wall projected to a single point on the candidate - degenerate, fail safe
            }
            const auto [kept_from, kept_to] = findKeptArcFromLargestGap(best_positions, best_geometry.total_length);
            // The covered (to be removed) range is the complement of the kept one.
            const std::pair<double, double> removed_range{ kept_to, kept_from };

            // Find (or start) this depth's own group for the matched contour - sets matching the
            // same line_idx join the same group, so their own ranges get cut together.
            PlannedGroup* group = nullptr;
            for (PlannedGroup& candidate : groups)
            {
                if (candidate.line_idx == *best_line_idx)
                {
                    group = &candidate;
                    break;
                }
            }
            if (group == nullptr)
            {
                groups.push_back(PlannedGroup{ *best_line_idx, best_geometry, {} });
                group = &groups.back();
            }
            group->removed_ranges.push_back(removed_range);
        }
    }

    // Commit: apply every planned depth's own groups, each splitting its own matched contour into
    // however many open kept arcs remain (0, 1, or more). Every other ExtrusionLine at that depth
    // (a different physical contour untouched by any group) is left completely untouched.
    bool stripped_anything = false;
    for (size_t depth = 0; depth < wall_toolpaths.size(); ++depth)
    {
        VariableWidthLines& inset = wall_toolpaths[depth];
        const std::vector<PlannedGroup>& groups = plan[depth];

        std::vector<std::pair<size_t, std::vector<ExtrusionLine>>> group_results;
        group_results.reserve(groups.size());
        for (const PlannedGroup& group : groups)
        {
            group_results.emplace_back(group.line_idx, splitByRemovedRanges(inset[group.line_idx], group.geometry, group.removed_ranges));
        }

        std::vector<ExtrusionLine> new_inset;
        new_inset.reserve(inset.size());
        for (size_t li = 0; li < inset.size(); ++li)
        {
            const auto group_it = std::find_if(
                group_results.begin(),
                group_results.end(),
                [li](const std::pair<size_t, std::vector<ExtrusionLine>>& entry) { return entry.first == li; });
            if (group_it != group_results.end())
            {
                for (ExtrusionLine& arc : group_it->second)
                {
                    new_inset.push_back(std::move(arc));
                }
            }
            else
            {
                new_inset.push_back(std::move(inset[li]));
            }
        }
        inset = std::move(new_inset);
        stripped_anything = true;
    }
    return stripped_anything;
}

std::optional<Point2LL> findNearestOpenWallEndpoint(const std::vector<VariableWidthLines>& wall_toolpaths, const Point2LL target, const coord_t max_dist)
{
    std::optional<Point2LL> best;
    int64_t best_dist_sq = std::numeric_limits<int64_t>::max();
    const int64_t max_dist_sq = static_cast<int64_t>(max_dist) * static_cast<int64_t>(max_dist);

    auto consider = [&](const Point2LL& candidate)
    {
        const int64_t dx = static_cast<int64_t>(candidate.X - target.X);
        const int64_t dy = static_cast<int64_t>(candidate.Y - target.Y);
        const int64_t dist_sq = dx * dx + dy * dy;
        if (dist_sq <= max_dist_sq && dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            best = candidate;
        }
    };

    for (const VariableWidthLines& inset : wall_toolpaths)
    {
        for (const ExtrusionLine& line : inset)
        {
            if (line.is_closed_ || line.junctions_.empty())
            {
                continue;
            }
            consider(line.junctions_.front().p_);
            consider(line.junctions_.back().p_);
        }
    }
    return best;
}

} // namespace cura
