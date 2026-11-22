// Copyright (c) 2026 FeatherPrint Corrugated
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "corrugated/BeadLoop.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "utils/linearAlg2D.h"

namespace cura
{

double scarfFlowFactor(double t, double start_ratio, bool closure)
{
    t = std::clamp(t, 0.0, 1.0);
    start_ratio = std::clamp(start_ratio, 0.0, 1.0);
    return closure ? std::lerp(1.0 - start_ratio, 0.0, t) : std::lerp(start_ratio, 1.0, t);
}

coord_t polylineLength(const std::vector<Point2LL>& path)
{
    coord_t length = 0;
    for (size_t i = 1; i < path.size(); ++i)
    {
        length += vSize(path[i] - path[i - 1]);
    }
    return length;
}

std::vector<Point2LL> polylineSpan(const std::vector<Point2LL>& path, coord_t from, coord_t to)
{
    std::vector<Point2LL> result;
    if (path.size() < 2 || to <= from)
    {
        return result;
    }
    coord_t walked = 0;
    for (size_t i = 1; i < path.size(); ++i)
    {
        const Point2LL& a = path[i - 1];
        const Point2LL& b = path[i];
        const coord_t seg = vSize(b - a);
        const coord_t seg_start = walked;
        const coord_t seg_end = walked + seg;
        walked = seg_end;
        if (seg == 0 || seg_end < from)
        {
            continue;
        }
        auto at = [&](coord_t d)
        {
            return a + (b - a) * (std::clamp(d, seg_start, seg_end) - seg_start) / seg;
        };
        if (result.empty())
        {
            result.push_back(at(from));
        }
        if (seg_end >= to)
        {
            result.push_back(at(to));
            return result;
        }
        result.push_back(b);
    }
    return result;
}

std::vector<ScarfRampSegment> scarfRampSegments(const std::vector<Point2LL>& span, coord_t step, double start_ratio, bool closure)
{
    std::vector<ScarfRampSegment> result;
    const coord_t total = polylineLength(span);
    if (total <= 0)
    {
        return result;
    }
    step = std::max(step, coord_t(10));
    coord_t walked = 0;
    for (size_t i = 1; i < span.size(); ++i)
    {
        const Point2LL& a = span[i - 1];
        const Point2LL& b = span[i];
        const coord_t seg = vSize(b - a);
        if (seg == 0)
        {
            continue;
        }
        const coord_t pieces = std::max(coord_t(1), (seg + step - 1) / step);
        for (coord_t p = 0; p < pieces; ++p)
        {
            const coord_t d0 = walked + seg * p / pieces;
            const coord_t d1 = walked + seg * (p + 1) / pieces;
            const double t_mid = (static_cast<double>(d0) + static_cast<double>(d1)) / 2.0 / static_cast<double>(total);
            result.push_back({ a + (b - a) * (p + 1) / pieces, scarfFlowFactor(t_mid, start_ratio, closure) });
        }
        walked += seg;
    }
    return result;
}

namespace
{

coord_t distanceToPolyline(const Point2LL& p, const std::vector<Point2LL>& path)
{
    coord_t best = std::numeric_limits<coord_t>::max();
    for (size_t i = 1; i < path.size(); ++i)
    {
        best = std::min(best, static_cast<coord_t>(std::sqrt(static_cast<double>(LinearAlg2D::getDist2FromLineSegment(path[i - 1], p, path[i])))));
    }
    return best;
}

/*!
 * Close the cap at the BACK of \p skin against wall end \p wall_end (see buildChainBeadLoop).
 * On success skin.back() == wall_end.
 */
bool closeCapAtBack(
    std::vector<Point2LL>& skin,
    const std::vector<Point2LL>& wall,
    const Point2LL& wall_end,
    coord_t join_tolerance,
    coord_t on_wall_tolerance,
    coord_t max_cap_gap,
    coord_t max_reroute_run)
{
    const coord_t gap = vSize(skin.back() - wall_end);
    if (gap <= join_tolerance)
    {
        if (gap > 0)
        {
            skin.push_back(wall_end);
        }
        return true;
    }
    if (gap > max_cap_gap)
    {
        return false;
    }
    // Trailing run along the kept wall: [first_on_wall, end).
    size_t first_on_wall = skin.size();
    while (first_on_wall > 0 && distanceToPolyline(skin[first_on_wall - 1], wall) <= on_wall_tolerance)
    {
        --first_on_wall;
    }
    // Need a real run (the rung foot plus at least the end point) and the rung itself before it.
    if (first_on_wall + 2 > skin.size() || first_on_wall == 0)
    {
        return false;
    }
    const std::vector<Point2LL> run(skin.begin() + first_on_wall, skin.end());
    if (polylineLength(run) > max_reroute_run)
    {
        return false;
    }
    skin.erase(skin.begin() + first_on_wall + 1, skin.end()); // Keep the rung foot.
    skin.push_back(wall_end);
    return true;
}

double signedArea(const std::vector<Point2LL>& loop)
{
    double area = 0.0;
    for (size_t i = 0; i < loop.size(); ++i)
    {
        const Point2LL& a = loop[i];
        const Point2LL& b = loop[(i + 1) % loop.size()];
        area += static_cast<double>(a.X) * static_cast<double>(b.Y) - static_cast<double>(b.X) * static_cast<double>(a.Y);
    }
    return area / 2.0;
}

} // namespace

std::optional<ChainBeadLoop> buildChainBeadLoop(
    const std::vector<Point2LL>& wall,
    std::vector<Point2LL> skin,
    coord_t join_tolerance,
    coord_t on_wall_tolerance,
    coord_t max_cap_gap,
    coord_t max_reroute_run)
{
    if (wall.size() < 2 || skin.size() < 2)
    {
        return std::nullopt;
    }
    const Point2LL& wall_start = wall.front();
    const Point2LL& wall_end = wall.back();

    // Orient the skin so it runs from the wall's end back to the wall's start.
    const coord_t keep = vSize(skin.front() - wall_end) + vSize(skin.back() - wall_start);
    const coord_t flip = vSize(skin.front() - wall_start) + vSize(skin.back() - wall_end);
    if (flip < keep)
    {
        std::reverse(skin.begin(), skin.end());
    }

    // Close the cap at the wall's start (skin back), then the wall's end (skin front, via reversal).
    if (! closeCapAtBack(skin, wall, wall_start, join_tolerance, on_wall_tolerance, max_cap_gap, max_reroute_run))
    {
        return std::nullopt;
    }
    std::reverse(skin.begin(), skin.end());
    if (! closeCapAtBack(skin, wall, wall_end, join_tolerance, on_wall_tolerance, max_cap_gap, max_reroute_run))
    {
        return std::nullopt;
    }
    std::reverse(skin.begin(), skin.end());

    ChainBeadLoop result;
    std::vector<Point2LL> loop(wall.begin(), wall.end());
    loop.insert(loop.end(), skin.begin() + 1, skin.end() - 1);
    if (signedArea(loop) < 0.0)
    {
        // Run it the other way round: the wall back to front, then the skin from the wall's front.
        result.wall_reversed = true;
        std::reverse(skin.begin(), skin.end());
    }
    result.skin = std::move(skin);
    return result;
}

} // namespace cura
