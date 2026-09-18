// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "WallToolPaths.h" // The unit under test.

#include <gtest/gtest.h>

#include "settings/Settings.h"
#include "utils/ExtrusionLine.h"
#include "utils/Simplify.h"

namespace cura
{

namespace
{
// simplifyToolPaths() is `protected` (it's an internal step of generate()); expose it here for
// direct unit testing without changing WallToolPaths' public surface.
struct WallToolPathsTestAccess : public WallToolPaths
{
    using WallToolPaths::simplifyToolPaths;
};

Settings makeRealPrintSimplifySettings()
{
    Settings settings;
    // Matches fdmprinter.def.json's default values for these settings (mm / um^2), which is what
    // was in effect on the real print where this defect was captured.
    settings.add("meshfix_maximum_resolution", "0.5");
    settings.add("meshfix_maximum_deviation", "0.025");
    settings.add("meshfix_maximum_extrusion_area_deviation", "50000");
    return settings;
}

// Returns true iff `line` (a closed loop) revisits the same location (within a small epsilon) at
// two non-adjacent indices, other than the one mandatory wraparound pair (index 0 / index n-1)
// every closed line legitimately has. Mirrors WallToolPaths.cpp's (anonymous-namespace, so not
// directly reachable from here) hasSelfTouchingPinch().
bool hasSelfTouchingPinch(const ExtrusionLine& line, const coord_t eps = 5)
{
    const size_t n = line.size();
    if (n < 4)
    {
        return false;
    }
    for (size_t i = 0; i < n; ++i)
    {
        for (size_t j = i + 2; j < n; ++j)
        {
            if (i == 0 && j == n - 1)
            {
                continue; // The mandatory closing-point wraparound pair.
            }
            if (shorterThan(line[i].p_ - line[j].p_, eps))
            {
                return true;
            }
        }
    }
    return false;
}

bool sameJunctions(const ExtrusionLine& a, const ExtrusionLine& b)
{
    if (a.size() != b.size())
    {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i)
    {
        if (a[i].p_ != b[i].p_)
        {
            return false;
        }
    }
    return true;
}
} // namespace

/*!
 * Real capture (layer ~843 of a swept-aerofoil trailing edge) reproduced verbatim: this exact
 * sequence of pre-simplify wall points is known to make Simplify::polygon() relocate a vertex
 * onto a spatially-close-but-array-index-distant part of the same closed loop, producing a
 * self-touching pinch (the loop revisits one point three times, with a real short excursion
 * sandwiched between two of the visits).
 *
 * This first assertion pins down that the fixture still reproduces the underlying Simplify
 * behaviour this fix works around; the second assertion is the actual regression test for the
 * fix in WallToolPaths::simplifyToolPaths.
 */
TEST(WallToolPathsTest, SimplifyAloneStillPinchesOnRealCapture)
{
    const Settings settings = makeRealPrintSimplifySettings();
    ExtrusionLine line(1, false, true);
#include "wall_tool_paths_test_layer843_wall1_fixture.inc"

    const Simplify simplifier(settings);
    const ExtrusionLine simplified = simplifier.polygon(line);

    EXPECT_TRUE(hasSelfTouchingPinch(simplified)) << "This fixture is expected to reproduce Simplify's known self-touching-pinch behaviour; "
                                                         "if this fails, the fixture may no longer exercise the case this fix targets.";
}

TEST(WallToolPathsTest, SimplifyToolPathsRepairsSelfTouchingPinch)
{
    const Settings settings = makeRealPrintSimplifySettings();
    ExtrusionLine line(1, false, true);
#include "wall_tool_paths_test_layer843_wall1_fixture.inc"
    const ExtrusionLine original_line = line;

    std::vector<VariableWidthLines> toolpaths;
    toolpaths.push_back({ line });

    WallToolPathsTestAccess::simplifyToolPaths(toolpaths, settings);

    ASSERT_EQ(toolpaths.size(), 1);
    ASSERT_EQ(toolpaths[0].size(), 1);
    // A genuine converging-tip wall naturally touches itself once even in the untouched
    // pre-simplify data (confirmed on this real capture), so the correct repair is to fall back
    // to exactly the original points for this line - not to merely reduce the duplicate count.
    EXPECT_TRUE(sameJunctions(toolpaths[0][0], original_line)) << "simplifyToolPaths should have fallen back to the exact pre-simplify points for this line "
                                                                   "rather than bake in Simplify's self-touching pinch.";
}

/*!
 * Confirms the fix does not blanket-disable simplification: a well-behaved closed loop (a
 * high-resolution circle, matching SimplifyTest's own approach) should still be simplified down
 * to noticeably fewer points, since it never triggers the pinch-detection fallback.
 */
TEST(WallToolPathsTest, SimplifyToolPathsStillSimplifiesWellBehavedLoops)
{
    const Settings settings = makeRealPrintSimplifySettings();
    ExtrusionLine line(0, false, true);
    constexpr coord_t radius = 100000;
    constexpr double segment_length = 500; // Comfortably below meshfix_maximum_resolution (500 um).
    constexpr double tau = 6.283185307179586476925286766559;
    const double increment = segment_length / radius;
    for (double angle = 0; angle < tau; angle += increment)
    {
        line.emplace_back(Point2LL(static_cast<coord_t>(std::cos(angle) * radius), static_cast<coord_t>(std::sin(angle) * radius)), 400, 0);
    }
    line.emplace_back(line.front());
    const size_t original_size = line.size();

    std::vector<VariableWidthLines> toolpaths;
    toolpaths.push_back({ line });

    WallToolPathsTestAccess::simplifyToolPaths(toolpaths, settings);

    ASSERT_EQ(toolpaths.size(), 1);
    ASSERT_EQ(toolpaths[0].size(), 1);
    EXPECT_LT(toolpaths[0][0].size(), original_size);
    EXPECT_FALSE(hasSelfTouchingPinch(toolpaths[0][0]));
}

} // namespace cura
