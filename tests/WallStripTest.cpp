// Copyright (c) 2026 FeatherPrint Corrugated
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "corrugated/WallStrip.h" // Unit under test.

#include <gtest/gtest.h>

#include "geometry/Point2LL.h"
#include "utils/ExtrusionLine.h"

namespace cura
{

namespace
{

// A closed square ExtrusionLine at the given inset index, centered at the origin with the given
// half-width - mirrors this test file's own need for two clearly different-sized nested contours
// (an "outer" part boundary and an "inner" hole boundary) without depending on VBCT at all, since
// stripRingWalls operates purely on ExtrusionLine/VariableWidthLines - core engine types - and
// never runs VBCT itself (that classification already happened by the time this function is
// called; see its own header doc).
ExtrusionLine makeSquare(size_t inset_idx, coord_t half_width)
{
    ExtrusionLine line(inset_idx, /*is_odd=*/false, /*is_closed=*/true);
    line.emplace_back(Point2LL(-half_width, -half_width), 400, 0);
    line.emplace_back(Point2LL(half_width, -half_width), 400, 0);
    line.emplace_back(Point2LL(half_width, half_width), 400, 0);
    line.emplace_back(Point2LL(-half_width, half_width), 400, 0);
    line.emplace_back(Point2LL(-half_width, -half_width), 400, 0); // closing point, matching Cura's own convention
    return line;
}

// A simple single-hole Ring's own wall_toolpaths shape: wall_count insets, each holding exactly
// two closed contours - a shrinking outer square and a shrinking (smaller) inner/hole square,
// both nested inside the outer one at every depth, matching the real geometric property
// stripRingWalls' own area-comparison relies on (a hole is always strictly smaller than the outer
// boundary it sits inside, at every inset depth independently).
std::vector<VariableWidthLines> makeRingWallToolpaths(size_t wall_count)
{
    std::vector<VariableWidthLines> wall_toolpaths;
    for (size_t inset_idx = 0; inset_idx < wall_count; ++inset_idx)
    {
        const coord_t outer_half_width = 10000 - static_cast<coord_t>(inset_idx) * 400;
        const coord_t inner_half_width = 3000 + static_cast<coord_t>(inset_idx) * 400; // shrinks toward the hole's own center as depth increases, staying smaller than outer at every depth
        VariableWidthLines inset;
        inset.push_back(makeSquare(inset_idx, outer_half_width));
        inset.push_back(makeSquare(inset_idx, inner_half_width));
        wall_toolpaths.push_back(std::move(inset));
    }
    return wall_toolpaths;
}

double totalJunctionCount(const std::vector<VariableWidthLines>& wall_toolpaths)
{
    double count = 0;
    for (const VariableWidthLines& inset : wall_toolpaths)
    {
        for (const ExtrusionLine& line : inset)
        {
            count += static_cast<double>(line.size());
        }
    }
    return count;
}

} // namespace

TEST(WallStripTest, NeitherFlagSetIsANoOp)
{
    std::vector<VariableWidthLines> wall_toolpaths = makeRingWallToolpaths(3);
    const std::vector<VariableWidthLines> original = wall_toolpaths;

    const bool stripped = stripRingWalls(wall_toolpaths, /*strip_wall_a=*/false, /*strip_wall_b=*/false);

    EXPECT_FALSE(stripped);
    ASSERT_EQ(wall_toolpaths.size(), original.size());
    for (size_t i = 0; i < wall_toolpaths.size(); ++i)
    {
        EXPECT_EQ(wall_toolpaths[i].size(), original[i].size()) << "inset " << i;
    }
}

TEST(WallStripTest, StripWallARemovesOnlyTheLargerOuterContourAtEveryDepth)
{
    constexpr size_t kWallCount = 3;
    std::vector<VariableWidthLines> wall_toolpaths = makeRingWallToolpaths(kWallCount);

    const bool stripped = stripRingWalls(wall_toolpaths, /*strip_wall_a=*/true, /*strip_wall_b=*/false);

    ASSERT_TRUE(stripped);
    ASSERT_EQ(wall_toolpaths.size(), kWallCount);
    for (size_t inset_idx = 0; inset_idx < kWallCount; ++inset_idx)
    {
        ASSERT_EQ(wall_toolpaths[inset_idx].size(), 1u) << "inset " << inset_idx;
        // The one remaining contour should be the smaller (hole) one - its own half-width is well
        // under the outer contour's, at every depth this fixture builds.
        const ExtrusionLine& remaining = wall_toolpaths[inset_idx].front();
        const coord_t remaining_half_width = remaining[0].p_.X < 0 ? -remaining[0].p_.X : remaining[0].p_.X;
        EXPECT_LT(remaining_half_width, 6000) << "inset " << inset_idx << " - Wall A (outer) should have been stripped, leaving only Wall B (inner)";
    }
}

TEST(WallStripTest, StripWallBRemovesOnlyTheSmallerInnerContourAtEveryDepth)
{
    constexpr size_t kWallCount = 3;
    std::vector<VariableWidthLines> wall_toolpaths = makeRingWallToolpaths(kWallCount);

    const bool stripped = stripRingWalls(wall_toolpaths, /*strip_wall_a=*/false, /*strip_wall_b=*/true);

    ASSERT_TRUE(stripped);
    ASSERT_EQ(wall_toolpaths.size(), kWallCount);
    for (size_t inset_idx = 0; inset_idx < kWallCount; ++inset_idx)
    {
        ASSERT_EQ(wall_toolpaths[inset_idx].size(), 1u) << "inset " << inset_idx;
        const ExtrusionLine& remaining = wall_toolpaths[inset_idx].front();
        const coord_t remaining_half_width = remaining[0].p_.X < 0 ? -remaining[0].p_.X : remaining[0].p_.X;
        EXPECT_GT(remaining_half_width, 6000) << "inset " << inset_idx << " - Wall B (inner) should have been stripped, leaving only Wall A (outer)";
    }
}

TEST(WallStripTest, BothFlagsSetRemovesEveryLineAtEveryDepth)
{
    std::vector<VariableWidthLines> wall_toolpaths = makeRingWallToolpaths(3);

    const bool stripped = stripRingWalls(wall_toolpaths, /*strip_wall_a=*/true, /*strip_wall_b=*/true);

    ASSERT_TRUE(stripped);
    ASSERT_EQ(wall_toolpaths.size(), 3u);
    for (const VariableWidthLines& inset : wall_toolpaths)
    {
        EXPECT_TRUE(inset.empty());
    }
}

TEST(WallStripTest, FailsSafeToANoOpWhenTopologyIsNotASimpleSingleHoleRing)
{
    // Three closed contours at one inset depth (e.g. two holes) - not the simple single-hole Ring
    // shape this function's own fail-safe pre-check requires - must leave everything untouched
    // rather than guess which two of the three form the "real" outer/inner pair.
    std::vector<VariableWidthLines> wall_toolpaths = makeRingWallToolpaths(2);
    wall_toolpaths[0].push_back(makeSquare(0, 1000));
    const double original_junction_count = totalJunctionCount(wall_toolpaths);

    const bool stripped = stripRingWalls(wall_toolpaths, /*strip_wall_a=*/true, /*strip_wall_b=*/false);

    EXPECT_FALSE(stripped);
    EXPECT_DOUBLE_EQ(totalJunctionCount(wall_toolpaths), original_junction_count);
}

TEST(WallStripTest, EmptyWallToolpathsIsANoOp)
{
    std::vector<VariableWidthLines> wall_toolpaths;
    EXPECT_FALSE(stripRingWalls(wall_toolpaths, /*strip_wall_a=*/true, /*strip_wall_b=*/true));
    EXPECT_TRUE(wall_toolpaths.empty());
}

namespace
{

// A closed rectangle - a stand-in for a Tee's own single shared wall loop, where a Chain domain's
// own wall covers only one of the four sides (the top edge here), leaving the other three (right,
// bottom, left) as "the rest of the shared wall" a sibling domain's own segment already occupies.
ExtrusionLine makeRectangleLoop(size_t inset_idx, coord_t half_width, coord_t half_height, size_t points_per_side = 3)
{
    ExtrusionLine line(inset_idx, /*is_odd=*/false, /*is_closed=*/true);
    auto addSide = [&](Point2LL from, Point2LL to)
    {
        for (size_t i = 0; i < points_per_side; ++i)
        {
            const double frac = static_cast<double>(i) / static_cast<double>(points_per_side);
            line.emplace_back(
                Point2LL(
                    from.X + static_cast<coord_t>(std::llround(frac * static_cast<double>(to.X - from.X))),
                    from.Y + static_cast<coord_t>(std::llround(frac * static_cast<double>(to.Y - from.Y)))),
                400,
                0);
        }
    };
    const Point2LL bl(-half_width, -half_height), br(half_width, -half_height), tr(half_width, half_height), tl(-half_width, half_height);
    addSide(bl, br);
    addSide(br, tr);
    addSide(tr, tl);
    addSide(tl, bl);
    line.emplace_back(bl, 400, 0); // closing point
    return line;
}

// Whether any junction in `line` lies within `tolerance` of `target`.
bool lineHasPointNear(const ExtrusionLine& line, Point2LL target, coord_t tolerance)
{
    for (const ExtrusionJunction& j : line)
    {
        const double dist = std::hypot(static_cast<double>(j.p_.X - target.X), static_cast<double>(j.p_.Y - target.Y));
        if (dist <= static_cast<double>(tolerance))
        {
            return true;
        }
    }
    return false;
}

// Whether any junction in `line` has Y within `tolerance` of `y` - used to check "does this side
// of makeRectangleLoop's own fixture survive" without depending on that fixture's own exact vertex
// placement along the side (points_per_side's own fractional spacing, not necessarily including
// the side's own true midpoint).
bool lineHasPointNearY(const ExtrusionLine& line, coord_t y, coord_t tolerance)
{
    for (const ExtrusionJunction& j : line)
    {
        if (std::abs(j.p_.Y - y) <= tolerance)
        {
            return true;
        }
    }
    return false;
}

bool lineHasPointNearX(const ExtrusionLine& line, coord_t x, coord_t tolerance)
{
    for (const ExtrusionJunction& j : line)
    {
        if (std::abs(j.p_.X - x) <= tolerance)
        {
            return true;
        }
    }
    return false;
}

} // namespace

/*!
 * The simple, non-wrapping case: a domain's own wall runs along one full side of a shared
 * rectangular loop (the top edge), not touching the loop's own start/end seam point (a corner).
 * The resulting kept arc should cover the other three sides and specifically exclude the covered
 * side's own points.
 */
TEST(WallStripTest, StripWallArcLengthRangeRemovesOnlyTheCoveredSideOfASharedLoop)
{
    constexpr coord_t kHalfWidth = 10000, kHalfHeight = 2000;
    std::vector<VariableWidthLines> wall_toolpaths;
    wall_toolpaths.push_back({ makeRectangleLoop(0, kHalfWidth, kHalfHeight) });

    // The domain's own wall: five points along the top edge, from near one corner to the other -
    // not touching the loop's own seam point (bottom-left corner, where point index 0 sits).
    const std::vector<Point2LL> domain_wall_points = {
        Point2LL(-9000, kHalfHeight), Point2LL(-4500, kHalfHeight), Point2LL(0, kHalfHeight), Point2LL(4500, kHalfHeight), Point2LL(9000, kHalfHeight)
    };

    const bool stripped = stripWallArcLengthRanges(wall_toolpaths, { domain_wall_points });

    ASSERT_TRUE(stripped);
    ASSERT_EQ(wall_toolpaths[0].size(), 1u);
    const ExtrusionLine& kept = wall_toolpaths[0].front();
    EXPECT_FALSE(kept.is_closed_);
    // No surviving point should be strictly inside the top edge's own covered x-range (excluding
    // the two boundary points at the domain's own outermost samples, which the splice legitimately
    // keeps as its own cut boundary) - checked directly rather than by exact midpoint, since
    // makeRectangleLoop's own vertex placement (fractions 0, 1/3, 2/3 of each side) doesn't
    // necessarily land exactly on any side's true midpoint.
    for (const ExtrusionJunction& j : kept)
    {
        if (j.p_.Y == kHalfHeight)
        {
            EXPECT_TRUE(std::abs(j.p_.X) >= 8900) << "a surviving point sits inside the covered top edge's own interior: (" << j.p_.X << ", " << j.p_.Y << ")";
        }
    }
    EXPECT_TRUE(lineHasPointNearY(kept, -kHalfHeight, 100)) << "bottom edge (untouched sibling segment) should remain";
    EXPECT_TRUE(lineHasPointNearX(kept, kHalfWidth, 100)) << "right edge (untouched) should remain";
    EXPECT_TRUE(lineHasPointNearX(kept, -kHalfWidth, 100)) << "left edge (untouched) should remain";
}

/*!
 * A second covered side, this time adjacent to the loop's own arbitrary start/end seam point
 * (point index 0, at the bottom-left corner) rather than opposite it - exercises the kept arc's
 * own walk actually crossing back through that seam (`collectArc`'s own wraparound indexing),
 * unlike the first test above where the kept arc's own start/end never comes near it.
 */
TEST(WallStripTest, StripWallArcLengthRangeHandlesTheCoveredRangeWrappingThePolylineSeam)
{
    constexpr coord_t kHalfWidth = 10000, kHalfHeight = 2000;
    std::vector<VariableWidthLines> wall_toolpaths;
    wall_toolpaths.push_back({ makeRectangleLoop(0, kHalfWidth, kHalfHeight) });

    // The domain's own wall: the *bottom* edge this time - the loop's own seam (its first/last
    // junction, at the bottom-left corner) sits right at one end of this exact side, so a domain
    // sampled slightly inside both corners still has the seam corner itself excluded from its own
    // sample set, but the covered arc-length range still wraps through position 0/total_length
    // internally (the bottom-left corner is between two of the domain's own samples in the
    // underlying vertex list, even though it's not one of the sampled points itself).
    const std::vector<Point2LL> domain_wall_points = {
        Point2LL(-9000, -kHalfHeight), Point2LL(-4500, -kHalfHeight), Point2LL(4500, -kHalfHeight), Point2LL(9000, -kHalfHeight)
    };

    const bool stripped = stripWallArcLengthRanges(wall_toolpaths, { domain_wall_points });

    ASSERT_TRUE(stripped);
    ASSERT_EQ(wall_toolpaths[0].size(), 1u);
    const ExtrusionLine& kept = wall_toolpaths[0].front();
    EXPECT_FALSE(kept.is_closed_);
    for (const ExtrusionJunction& j : kept)
    {
        if (j.p_.Y == -kHalfHeight)
        {
            EXPECT_TRUE(std::abs(j.p_.X) >= 8900) << "a surviving point sits inside the covered bottom edge's own interior: (" << j.p_.X << ", " << j.p_.Y << ")";
        }
    }
    EXPECT_TRUE(lineHasPointNearY(kept, kHalfHeight, 100)) << "top edge (untouched) should remain";
    EXPECT_TRUE(lineHasPointNearX(kept, kHalfWidth, 100)) << "right edge (untouched) should remain";
    EXPECT_TRUE(lineHasPointNearX(kept, -kHalfWidth, 100)) << "left edge (untouched) should remain";
}

TEST(WallStripTest, StripWallArcLengthRangeFailsSafeOnTooFewDomainPoints)
{
    std::vector<VariableWidthLines> wall_toolpaths;
    wall_toolpaths.push_back({ makeRectangleLoop(0, 10000, 2000) });
    const double original_junction_count = totalJunctionCount(wall_toolpaths);

    EXPECT_FALSE(stripWallArcLengthRanges(wall_toolpaths, { { Point2LL(0, 2000) } })); // only one point in the one set
    EXPECT_DOUBLE_EQ(totalJunctionCount(wall_toolpaths), original_junction_count);

    EXPECT_FALSE(stripWallArcLengthRanges(wall_toolpaths, {})); // no sets at all
    EXPECT_DOUBLE_EQ(totalJunctionCount(wall_toolpaths), original_junction_count);
}

/*!
 * The real bug this test guards against: stripping both Wall A and Wall B of a simple corridor -
 * two different arcs of the *same* physical loop - in two separate calls left one side
 * un-stripped, since the first call already opened the loop and the second could no longer find a
 * *closed* candidate to match against. Both sides must be passed together in one call instead.
 */
TEST(WallStripTest, StripWallArcLengthRangeStripsBothSidesOfASimpleCorridorInOneCall)
{
    constexpr coord_t kHalfWidth = 10000, kHalfHeight = 2000;
    std::vector<VariableWidthLines> wall_toolpaths;
    wall_toolpaths.push_back({ makeRectangleLoop(0, kHalfWidth, kHalfHeight) });

    const std::vector<Point2LL> wall_a_points = {
        Point2LL(-9000, kHalfHeight), Point2LL(-4500, kHalfHeight), Point2LL(0, kHalfHeight), Point2LL(4500, kHalfHeight), Point2LL(9000, kHalfHeight)
    };
    const std::vector<Point2LL> wall_b_points = {
        Point2LL(-9000, -kHalfHeight), Point2LL(-4500, -kHalfHeight), Point2LL(0, -kHalfHeight), Point2LL(4500, -kHalfHeight), Point2LL(9000, -kHalfHeight)
    };

    const bool stripped = stripWallArcLengthRanges(wall_toolpaths, { wall_a_points, wall_b_points });

    ASSERT_TRUE(stripped);
    // Removing both long sides of a rectangle leaves two short, disjoint open arcs behind - one
    // for each end cap (left and right edge's own remaining un-stripped territory).
    ASSERT_EQ(wall_toolpaths[0].size(), 2u);
    for (const ExtrusionLine& kept : wall_toolpaths[0])
    {
        EXPECT_FALSE(kept.is_closed_);
        for (const ExtrusionJunction& j : kept)
        {
            EXPECT_TRUE(std::abs(j.p_.Y) != kHalfHeight || std::abs(j.p_.X) >= 8900)
                << "a surviving point sits inside a covered side's own interior: (" << j.p_.X << ", " << j.p_.Y << ")";
        }
    }
    EXPECT_TRUE(lineHasPointNearX(wall_toolpaths[0][0], kHalfWidth, 100) || lineHasPointNearX(wall_toolpaths[0][1], kHalfWidth, 100))
        << "right edge's own remaining territory should survive in one of the two kept arcs";
    EXPECT_TRUE(lineHasPointNearX(wall_toolpaths[0][0], -kHalfWidth, 100) || lineHasPointNearX(wall_toolpaths[0][1], -kHalfWidth, 100))
        << "left edge's own remaining territory should survive in one of the two kept arcs";
}

/*!
 * Wall Strip: multiple simultaneously-stripped Chain domains. Two sets that land on two genuinely
 * *different* physical contours (not sharing a wall loop at all - two separate, disjoint
 * rectangles, standing in for two domains on unrelated parts of a more complex shape) must each
 * still be stripped correctly in one call, rather than the whole call failing just because the
 * sets don't share a common contour - the real generalization this pass makes over the previous
 * "all sets must match one contour" behavior.
 */
TEST(WallStripTest, StripWallArcLengthRangeGroupsSetsOntoDifferentContoursIndependently)
{
    constexpr coord_t kHalfWidth = 10000, kHalfHeight = 2000;
    constexpr coord_t kSecondLoopOffsetX = 40000; // far enough that the two loops never interact
    std::vector<VariableWidthLines> wall_toolpaths;
    ExtrusionLine second_loop = makeRectangleLoop(0, kHalfWidth, kHalfHeight);
    for (ExtrusionJunction& j : second_loop)
    {
        j.p_.X += kSecondLoopOffsetX;
    }
    wall_toolpaths.push_back({ makeRectangleLoop(0, kHalfWidth, kHalfHeight), second_loop });

    // "Domain 1" strips the top edge of the first loop; "domain 2" strips the top edge of the
    // second (translated) loop - two unrelated physical contours.
    const std::vector<Point2LL> domain1_points = {
        Point2LL(-9000, kHalfHeight), Point2LL(-4500, kHalfHeight), Point2LL(0, kHalfHeight), Point2LL(4500, kHalfHeight), Point2LL(9000, kHalfHeight)
    };
    std::vector<Point2LL> domain2_points;
    for (const Point2LL& p : domain1_points)
    {
        domain2_points.emplace_back(p.X + kSecondLoopOffsetX, p.Y);
    }

    const bool stripped = stripWallArcLengthRanges(wall_toolpaths, { domain1_points, domain2_points });

    ASSERT_TRUE(stripped);
    ASSERT_EQ(wall_toolpaths[0].size(), 2u);
    for (const ExtrusionLine& kept : wall_toolpaths[0])
    {
        EXPECT_FALSE(kept.is_closed_) << "both loops' own top edges should have been stripped";
    }
    // Each loop's own remaining bottom edge should survive, at that loop's own X offset.
    EXPECT_TRUE(lineHasPointNearY(wall_toolpaths[0][0], -kHalfHeight, 100) || lineHasPointNearY(wall_toolpaths[0][1], -kHalfHeight, 100))
        << "first loop's own bottom edge (untouched) should remain";
    bool second_loop_bottom_found = false;
    for (const ExtrusionLine& kept : wall_toolpaths[0])
    {
        for (const ExtrusionJunction& j : kept)
        {
            if (std::abs(j.p_.Y - (-kHalfHeight)) <= 100 && std::abs(j.p_.X - kSecondLoopOffsetX) < kHalfWidth)
            {
                second_loop_bottom_found = true;
            }
        }
    }
    EXPECT_TRUE(second_loop_bottom_found) << "second loop's own bottom edge (untouched) should remain";
}

namespace
{

// A simple open ExtrusionLine of two points, standing in for the one-wall-stripped case's own
// surviving un-stripped-wall arc (stripWallArcLengthRanges' own real output shape) without
// depending on that function to build it - findNearestOpenWallEndpoint only ever looks at an open
// line's own front()/back(), so a minimal two-point line is sufficient.
ExtrusionLine makeOpenLine(size_t inset_idx, Point2LL from, Point2LL to)
{
    ExtrusionLine line(inset_idx, /*is_odd=*/false, /*is_closed=*/false);
    line.emplace_back(from, 400, 0);
    line.emplace_back(to, 400, 0);
    return line;
}

} // namespace

/*!
 * Chain end-linking connector (spec REV 2.6 design): findNearestOpenWallEndpoint must find the
 * true nearest open-wall endpoint across multiple inset depths and multiple lines at the same
 * depth, and must never consider a closed line's own points (a closed wall has no dangling
 * endpoint to connect to - nothing was stripped there).
 */
TEST(WallStripTest, FindNearestOpenWallEndpointFindsTheTrueNearestAcrossDepthsAndLines)
{
    std::vector<VariableWidthLines> wall_toolpaths;
    // Depth 0: one open line whose own back() is far from the target, plus one closed line (whose
    // own vertices - including one placed suspiciously close to the target - must be ignored).
    ExtrusionLine closed_decoy = makeSquare(0, 500);
    for (ExtrusionJunction& j : closed_decoy)
    {
        j.p_.X += 10000;
        j.p_.Y += 10000; // one of this closed square's own corners now sits right next to the target below
    }
    wall_toolpaths.push_back({ makeOpenLine(0, Point2LL(0, 0), Point2LL(100000, 100000)), closed_decoy });
    // Depth 1: the true nearest open endpoint, close to the target.
    wall_toolpaths.push_back({ makeOpenLine(1, Point2LL(10300, 10300), Point2LL(200000, 200000)) });

    const Point2LL target(10000, 10000);
    const std::optional<Point2LL> found = findNearestOpenWallEndpoint(wall_toolpaths, target, /*max_dist=*/1000);

    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(*found, Point2LL(10300, 10300)) << "should find depth 1's own open endpoint, not the closer-but-closed decoy vertex";
}

TEST(WallStripTest, FindNearestOpenWallEndpointReturnsNulloptBeyondMaxDist)
{
    std::vector<VariableWidthLines> wall_toolpaths;
    wall_toolpaths.push_back({ makeOpenLine(0, Point2LL(0, 0), Point2LL(50000, 0)) });

    const Point2LL target(0, 5000); // 5000 microns from the nearest open endpoint, Point2LL(0, 0)
    EXPECT_FALSE(findNearestOpenWallEndpoint(wall_toolpaths, target, /*max_dist=*/1000).has_value());
    EXPECT_TRUE(findNearestOpenWallEndpoint(wall_toolpaths, target, /*max_dist=*/6000).has_value()) << "sanity check: the same target should be found once max_dist covers it";
}

TEST(WallStripTest, FindNearestOpenWallEndpointReturnsNulloptWhenNoOpenLineExists)
{
    std::vector<VariableWidthLines> wall_toolpaths = makeRingWallToolpaths(2); // both contours closed
    EXPECT_FALSE(findNearestOpenWallEndpoint(wall_toolpaths, Point2LL(10000, 10000), /*max_dist=*/1000000).has_value());
}

} // namespace cura
