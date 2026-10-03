// FeatherPrint: the toolpath OML convention (FeatherPrintGenerator::toolpathOml) - the outer Wall's
// centreline sits half a line width inside the model surface, so its outer face prints on that
// surface, as a stock Cura wall does.

#include "featherprint/FeatherPrintGenerator.h"

#include <algorithm>

#include <gtest/gtest.h>

#include "geometry/OpenPolyline.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"

namespace cura
{

namespace
{

Shape square(coord_t x0, coord_t y0, coord_t x1, coord_t y1)
{
    Polygon poly;
    poly.push_back(Point2LL(x0, y0));
    poly.push_back(Point2LL(x1, y0));
    poly.push_back(Point2LL(x1, y1));
    poly.push_back(Point2LL(x0, y1));
    Shape shape;
    shape.push_back(poly);
    return shape;
}

} // namespace

TEST(FeatherPrintGeneratorTest, ToolpathOmlInsetsHalfALineWidth)
{
    constexpr coord_t w = 400;
    const Shape oml = FeatherPrintGenerator::toolpathOml(square(0, 0, 20000, 20000), w);
    ASSERT_EQ(oml.size(), 1);
    coord_t min_x = oml[0][0].X, max_x = min_x, min_y = oml[0][0].Y, max_y = min_y;
    for (const Point2LL& p : oml[0])
    {
        min_x = std::min(min_x, p.X);
        max_x = std::max(max_x, p.X);
        min_y = std::min(min_y, p.Y);
        max_y = std::max(max_y, p.Y);
    }
    // A w-wide line on this centreline spans exactly the 20 mm model surface.
    EXPECT_NEAR(min_x, w / 2, 2);
    EXPECT_NEAR(max_x, 20000 - w / 2, 2);
    EXPECT_NEAR(min_y, w / 2, 2);
    EXPECT_NEAR(max_y, 20000 - w / 2, 2);
}

TEST(FeatherPrintGeneratorTest, ToolpathOmlFallsBackForSubLineWidthPart)
{
    constexpr coord_t w = 400;
    const Shape sliver = square(0, 0, 20000, 300);
    const Shape oml = FeatherPrintGenerator::toolpathOml(sliver, w);
    ASSERT_EQ(oml.size(), 1);
    EXPECT_EQ(oml[0].size(), sliver[0].size());
}

TEST(FeatherPrintGeneratorTest, ToolpathOmlOpenMovesInwardAlongNormal)
{
    constexpr coord_t w = 400;
    // Bottom, right and top of a CCW square, open on the left: interior on the left of travel.
    OpenPolyline arc;
    arc.push_back(Point2LL(0, 0));
    arc.push_back(Point2LL(10000, 0));
    arc.push_back(Point2LL(20000, 0));
    arc.push_back(Point2LL(20000, 10000));
    arc.push_back(Point2LL(20000, 20000));
    arc.push_back(Point2LL(10000, 20000));
    arc.push_back(Point2LL(0, 20000));
    const OpenPolyline oml = FeatherPrintGenerator::toolpathOmlOpen(arc, w);
    ASSERT_EQ(oml.size(), arc.size());
    // The ends keep their position along the boundary and step w/2 into the material.
    EXPECT_NEAR(oml.front().X, 0, 2);
    EXPECT_NEAR(oml.front().Y, w / 2, 2);
    EXPECT_NEAR(oml.back().X, 0, 2);
    EXPECT_NEAR(oml.back().Y, 20000 - w / 2, 2);
    EXPECT_NEAR(oml[1].Y, w / 2, 2);
    EXPECT_NEAR(oml[3].X, 20000 - w / 2, 2);
    EXPECT_NEAR(oml[5].Y, 20000 - w / 2, 2);
}

} // namespace cura
