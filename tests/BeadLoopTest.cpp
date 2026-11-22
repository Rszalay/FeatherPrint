// Copyright (c) 2026 FeatherPrint Corrugated
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "corrugated/BeadLoop.h" // Unit under test.

#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

namespace cura
{

namespace
{

// A Chain domain shaped like one end of an A-40 two-hole half (2026-11-18), simplified to a straight
// strip: the kept wall runs along y = 0, the stripped side is y = 3000, end faces at x = 0 and x = 20000.
// After Wall Strip the kept wall climbs both end faces up to the stripped side's corners.
std::vector<Point2LL> keptWall()
{
    return { Point2LL(0, 3000), Point2LL(0, 0), Point2LL(20000, 0), Point2LL(20000, 3000) };
}

// The Linked Skin zig-zags between the two sides. It starts at the wall's end (x = 20000 cap, matched,
// as the end connector leaves it) and finishes on the KEPT side's corner at x = 0, an end face away from
// the wall's start: the systematic one-cap gap seen on every A-40 Chain layer.
std::vector<Point2LL> linkedSkin()
{
    return { Point2LL(20000, 3000), Point2LL(15000, 3000), Point2LL(15000, 0), Point2LL(10000, 0), Point2LL(10000, 3000),
             Point2LL(5000, 3000),  Point2LL(5000, 0),     Point2LL(0, 0) };
}

constexpr coord_t join = 1600; // 4 x 0.4 mm infill line
constexpr coord_t on_wall = 400;
constexpr coord_t max_gap = 17000;
constexpr coord_t max_run = 17000;

} // namespace

TEST(BeadLoopTest, OpeningAndClosingRampsSumToOneBead)
{
    for (const double start_ratio : { 0.0, 0.25, 0.5, 1.0 })
    {
        for (int i = 0; i <= 10; ++i)
        {
            const double t = i / 10.0;
            EXPECT_NEAR(scarfFlowFactor(t, start_ratio, false) + scarfFlowFactor(t, start_ratio, true), 1.0, 1e-12);
        }
        EXPECT_NEAR(scarfFlowFactor(0.0, start_ratio, false), start_ratio, 1e-12);
        EXPECT_NEAR(scarfFlowFactor(1.0, start_ratio, false), 1.0, 1e-12);
        EXPECT_NEAR(scarfFlowFactor(1.0, start_ratio, true), 0.0, 1e-12);
    }
}

TEST(BeadLoopTest, RampSegmentsCoverTheSpanInSteps)
{
    const std::vector<Point2LL> span{ Point2LL(0, 0), Point2LL(2500, 0), Point2LL(2500, 2500) };
    const std::vector<ScarfRampSegment> up = scarfRampSegments(span, 1000, 0.0, false);
    const std::vector<ScarfRampSegment> down = scarfRampSegments(span, 1000, 0.0, true);
    ASSERT_EQ(up.size(), 6u); // 3 pieces per 2.5 mm leg
    ASSERT_EQ(down.size(), up.size());
    EXPECT_EQ(up.back().to, span.back());
    Point2LL from = span.front();
    for (size_t i = 0; i < up.size(); ++i)
    {
        EXPECT_LE(vSize(up[i].to - from), 1000);
        EXPECT_NEAR(up[i].flow_factor + down[i].flow_factor, 1.0, 1e-12);
        if (i > 0)
        {
            EXPECT_GT(up[i].flow_factor, up[i - 1].flow_factor);
        }
        from = up[i].to;
    }
}

TEST(BeadLoopTest, PolylineSpanInterpolatesEnds)
{
    const std::vector<Point2LL> path{ Point2LL(0, 0), Point2LL(1000, 0), Point2LL(1000, 1000) };
    EXPECT_EQ(polylineLength(path), 2000);
    const std::vector<Point2LL> span = polylineSpan(path, 500, 1500);
    ASSERT_EQ(span.size(), 3u);
    EXPECT_EQ(span.front(), Point2LL(500, 0));
    EXPECT_EQ(span[1], Point2LL(1000, 0));
    EXPECT_EQ(span.back(), Point2LL(1000, 500));
}

TEST(BeadLoopTest, ChainCapGapIsClosedByReroutingTheSkinToTheLastRungFoot)
{
    const std::optional<ChainBeadLoop> loop = buildChainBeadLoop(keptWall(), linkedSkin(), join, on_wall, max_gap, max_run);
    ASSERT_TRUE(loop.has_value());
    EXPECT_FALSE(loop->wall_reversed); // wall then skin already runs counter-clockwise
    // Wall end -> ... -> last rung -> its foot -> straight to the wall's start (the end face stays the wall's).
    const std::vector<Point2LL> expected{ Point2LL(20000, 3000), Point2LL(15000, 3000), Point2LL(15000, 0),  Point2LL(10000, 0),
                                          Point2LL(10000, 3000), Point2LL(5000, 3000),  Point2LL(5000, 0), Point2LL(0, 3000) };
    EXPECT_EQ(loop->skin, expected);
}

TEST(BeadLoopTest, ClockwiseLoopIsReversedSoTheSeamStaysOnOneCap)
{
    std::vector<Point2LL> wall = keptWall();
    std::reverse(wall.begin(), wall.end());
    const std::optional<ChainBeadLoop> loop = buildChainBeadLoop(wall, linkedSkin(), join, on_wall, max_gap, max_run);
    ASSERT_TRUE(loop.has_value());
    EXPECT_TRUE(loop->wall_reversed);
    // Printed wall runs back to front, from (0, 3000) to (20000, 3000): the skin starts at its printed end.
    EXPECT_EQ(loop->skin.front(), wall.front());
    EXPECT_EQ(loop->skin.back(), wall.back());
}

TEST(BeadLoopTest, FallsBackWhenTheCapGapIsTooLarge)
{
    EXPECT_FALSE(buildChainBeadLoop(keptWall(), linkedSkin(), join, on_wall, /*max_cap_gap=*/2000, max_run).has_value());
}

TEST(BeadLoopTest, FallsBackWhenTheSkinDoesNotEndAlongTheKeptWall)
{
    std::vector<Point2LL> skin = linkedSkin();
    skin.back() = Point2LL(2500, 1500); // ends mid-domain: no trailing run along the wall to reroute
    EXPECT_FALSE(buildChainBeadLoop(keptWall(), skin, join, on_wall, max_gap, max_run).has_value());
}

} // namespace cura
