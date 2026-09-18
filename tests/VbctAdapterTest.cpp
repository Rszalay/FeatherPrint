// Copyright (c) 2026 FeatherPrint Corrugated
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "corrugated/VbctAdapter.h" //Unit under test.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <set>

#include <gtest/gtest.h>

#include "geometry/OpenLinesSet.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"
#include "mesh.h"
#include "settings/Settings.h"
#include "sliceDataStorage.h"

// Direct access to VBCT's own pipeline, to inspect intermediate domain
// composition (ring/chain/glob counts), not just corrugate()'s final
// stringer lines -- see the diagnostic test below.
#include "pipeline.hpp"
#include "stage5.hpp"
#include "stage6.hpp"
#include "stage7.hpp"
#include "stage8.hpp"
#include "stage9.hpp"

#include "real_tab_geometry.h"
#include "real_tab_geometry_multiz.h"

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{

namespace
{

Polygon makeCirclePolygon(const double center_x, const double center_y, const double radius, const size_t n_points, const double start_angle = 0.0)
{
    Polygon polygon;
    for (size_t i = 0; i < n_points; ++i)
    {
        const double angle = start_angle + 2.0 * std::numbers::pi * static_cast<double>(i) / static_cast<double>(n_points);
        polygon.push_back(Point2LL(static_cast<coord_t>(std::llround(center_x + radius * std::cos(angle))), static_cast<coord_t>(std::llround(center_y + radius * std::sin(angle)))));
    }
    return polygon;
}

Polygon makeRectPolygon(const coord_t x0, const coord_t y0, const coord_t x1, const coord_t y1)
{
    Polygon polygon;
    polygon.push_back(Point2LL(x0, y0));
    polygon.push_back(Point2LL(x1, y0));
    polygon.push_back(Point2LL(x1, y1));
    polygon.push_back(Point2LL(x0, y1));
    return polygon;
}

} // namespace

/*!
 * A genuine annulus (outer + inner ring, like Test Models - Ring.stl) should decompose to
 * VBCT's Ring domain and produce real corrugation stringers.
 */
TEST(VbctAdapterTest, RingShapeProducesStringers)
{
    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 64)); // 30mm outer radius.
    ring_shape.push_back(makeCirclePolygon(0, 0, 15000, 48)); // 15mm inner radius (a hole).

    const std::optional<OpenLinesSet> lines = VbctAdapter::corrugate(ring_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000);

    ASSERT_TRUE(lines.has_value());
    EXPECT_FALSE(lines->empty());
    for (const OpenPolyline& line : *lines)
    {
        EXPECT_GE(line.size(), 2u);
    }
}

/*!
 * Crosshatch (spec Section 3.1's "two counter-rotating helix families (CCW/CW)"): confirmed with
 * the user to be a genuinely 3D, over-Z effect - the CW family reuses the exact same per-layer
 * construction as the CCW family (same forward index, same left_t0_frac/right_t0_frac, no
 * reflection of either), only with effective_phase_offset's sign flipped for both walls together.
 * Within a single isolated layer (or anywhere effective_phase_offset is exactly 0 - no Z-phase
 * progression at all, e.g. crossover_pitch_mm 0.0) the two families exactly coincide, since the phase term
 * is the *only* thing distinguishing them - so crosshatch should add nothing there rather than
 * printing a redundant duplicate of every CCW stringer. See stage10.hpp's crosshatch_enabled doc
 * comment for the full account of why (two earlier, wrong same-layer-crossing designs were tried
 * and ruled out first).
 */
TEST(VbctAdapterTest, CrosshatchProducesNoExtraStringersAtZeroPhaseOffset)
{
    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 64)); // 30mm outer radius.
    ring_shape.push_back(makeCirclePolygon(0, 0, 15000, 48)); // 15mm inner radius (a hole).

    const std::optional<OpenLinesSet> lines_single = VbctAdapter::corrugate(ring_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000);
    const std::optional<OpenLinesSet> lines_cross = VbctAdapter::corrugate(
        ring_shape,
        /*x=*/0.3,
        /*threshold=*/800,
        /*spacing=*/2000,
        /*z=*/0,
        /*crossover_pitch_mm=*/0.0,
        /*anchor_t0_frac=*/std::nullopt,
        /*other_wall_t0_frac=*/std::nullopt,
        /*reverse_canonical_wall=*/std::nullopt,
        /*crosshatch_enabled=*/true);

    ASSERT_TRUE(lines_single.has_value());
    ASSERT_TRUE(lines_cross.has_value());
    EXPECT_EQ(lines_cross->size(), lines_single->size());
}

/*!
 * Away from the degenerate zero-phase case above, crosshatch should exactly double the stringer
 * count: every CW stringer is a genuinely distinct segment from its CCW sibling (see this file's
 * own CrosshatchFamilyDiffersFromCcwAtNonzeroPhaseOffset below), so none get skipped as
 * duplicates. z=5mm at crossover_pitch_mm=10mm gives phase_offset = 5/(2*10) = 0.25 - well clear
 * of every domain's own degenerate points (multiples of stringer_count/2 in phase_offset's own
 * raw, undivided units - see stage10.hpp's phase_offset doc comment).
 */
TEST(VbctAdapterTest, CrosshatchEnabledDoublesStringerCountAtNonzeroPhaseOffset)
{
    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 64));
    ring_shape.push_back(makeCirclePolygon(0, 0, 15000, 48));

    const std::optional<OpenLinesSet> lines_single = VbctAdapter::corrugate(ring_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000, /*z=*/5000, /*crossover_pitch_mm=*/10.0);
    const std::optional<OpenLinesSet> lines_cross = VbctAdapter::corrugate(
        ring_shape,
        /*x=*/0.3,
        /*threshold=*/800,
        /*spacing=*/2000,
        /*z=*/5000,
        /*crossover_pitch_mm=*/10.0,
        /*anchor_t0_frac=*/std::nullopt,
        /*other_wall_t0_frac=*/std::nullopt,
        /*reverse_canonical_wall=*/std::nullopt,
        /*crosshatch_enabled=*/true);

    ASSERT_TRUE(lines_single.has_value());
    ASSERT_TRUE(lines_cross.has_value());
    EXPECT_EQ(lines_cross->size(), lines_single->size() * 2);
}

/*!
 * At a nonzero phase offset, every one of crosshatch's CW stringers should be a genuinely distinct
 * (front, back) pair from every other line emitted - not a literal repeat of a CCW stringer (or of
 * another CW one).
 */
TEST(VbctAdapterTest, CrosshatchFamilyDiffersFromCcwAtNonzeroPhaseOffset)
{
    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 64));
    ring_shape.push_back(makeCirclePolygon(0, 0, 15000, 48));

    const std::optional<OpenLinesSet> lines_cross = VbctAdapter::corrugate(
        ring_shape,
        /*x=*/0.3,
        /*threshold=*/800,
        /*spacing=*/2000,
        /*z=*/5000,
        /*crossover_pitch_mm=*/10.0,
        /*anchor_t0_frac=*/std::nullopt,
        /*other_wall_t0_frac=*/std::nullopt,
        /*reverse_canonical_wall=*/std::nullopt,
        /*crosshatch_enabled=*/true);

    ASSERT_TRUE(lines_cross.has_value());
    std::set<std::pair<Point2LL, Point2LL>> distinct_pairs;
    for (const OpenPolyline& line : *lines_cross)
    {
        distinct_pairs.insert({ line.front(), line.back() });
    }
    EXPECT_EQ(distinct_pairs.size(), lines_cross->size());
}

/*!
 * Linked corrugation skin (VbctAdapter::corrugateLinkedSkin): on a plain concentric ring at a
 * nonzero phase offset (so the CW family it always builds internally is genuinely distinct from
 * the CCW one - see corrugateLinkedSkin's own doc comment), the result should be exactly one
 * closed polyline (front()==back()) that visits both walls, switches between them more than once
 * around the ring, but - since each skin arc includes the wall's own intermediate vertices between
 * crossings, not just the two endpoints - does *not* switch at every single point (that would mean
 * every arc was a single degenerate point, not a real wall-following stretch). No implausibly
 * large jumps between consecutive points (a legitimate crossing jump is close to the wall-to-wall
 * gap size, not zero - only a jump far beyond that, e.g. skipping partway around the ring, would
 * indicate a real bug). Wall classification uses distance from the ring's own center, valid
 * for this plain-concentric fixture specifically (a real off-center or eccentric wall would need a
 * proper nearest-point classification instead).
 */
TEST(VbctAdapterTest, LinkedSkinProducesClosedAlternatingLoopWithNoGaps)
{
    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 64)); // 30mm outer radius.
    ring_shape.push_back(makeCirclePolygon(0, 0, 15000, 48)); // 15mm inner radius (a hole).

    const std::optional<OpenLinesSet> lines
        = VbctAdapter::corrugateLinkedSkin(ring_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000, /*z=*/5000, /*crossover_pitch_mm=*/10.0);

    ASSERT_TRUE(lines.has_value());
    ASSERT_EQ(lines->size(), 1u) << "Linked skin should produce exactly one connected toolpath, not independent stringer segments.";
    const OpenPolyline& path = (*lines)[0];
    ASSERT_GE(path.size(), 4u);
    EXPECT_EQ(path.front(), path.back()) << "Linked skin should form one closed loop.";

    // No inboard offset any more (see corrugateLinkedSkin's own doc comment for why - infill_area,
    // which this fixture stands in for, is already the region Cura's own wall generation leaves
    // after the wall, no extra buffer needed on top) - the skin should run directly along the
    // input walls' own radii.
    const double outer_radius_expected = 30000.0;
    const double inner_radius_expected = 15000.0;
    auto classify_outer = [&](const Point2LL& p)
    {
        const double r = std::hypot(static_cast<double>(p.X), static_cast<double>(p.Y));
        return std::abs(r - outer_radius_expected) < std::abs(r - inner_radius_expected);
    };

    // Excludes the duplicated closing point (path.back() == path.front()) - classifications is
    // one entry per *distinct* point around the loop.
    std::vector<bool> classifications;
    classifications.reserve(path.size() - 1);
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        classifications.push_back(classify_outer(path[i]));
    }
    ASSERT_FALSE(classifications.empty());
    bool saw_outer = false;
    bool saw_inner = false;
    size_t transitions = 0;
    for (size_t i = 0; i < classifications.size(); ++i)
    {
        (classifications[i] ? saw_outer : saw_inner) = true;
        if (i > 0 && classifications[i] != classifications[i - 1])
        {
            ++transitions;
        }
    }
    if (classifications.back() != classifications.front()) // wrap-around transition, closing the loop
    {
        ++transitions;
    }
    EXPECT_TRUE(saw_outer);
    EXPECT_TRUE(saw_inner);
    EXPECT_GT(transitions, 1u) << "Should cross between the two walls more than once around the ring.";
    EXPECT_LT(transitions, classifications.size()) << "Wall changes should happen only at crossings, not at every single point - each skin arc should include several same-wall points.";

    // Not every consecutive point pair is a small "skin" step - a crossing jump (from one wall's
    // attachment to the other's) is legitimately close to the wall-to-wall gap size (here ~15mm).
    // The bound below only needs to catch a genuinely wrong jump (e.g. skipping halfway around the
    // ~94mm-radius-30mm ring's own ~188mm circumference), not rule out an ordinary crossing.
    double max_gap = 0.0;
    double total_length = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const Point2LL delta = path[i + 1] - path[i];
        const double seg_len = std::sqrt(static_cast<double>(delta.X) * delta.X + static_cast<double>(delta.Y) * delta.Y);
        max_gap = std::max(max_gap, seg_len);
        total_length += seg_len;
    }
    EXPECT_LT(max_gap, 20000.0) << "No consecutive point pair should be separated by an implausibly large jump.";

    // Each skin arc should walk the *shortest* way to the next crossing along its wall, not wrap
    // most of the way around the ring first - a real, confirmed bug in an earlier version of this
    // feature (traced through several distinct causes - see corrugateLinkedSkin's own doc comment
    // for the full account - before landing on the current per-vertex offset construction, which
    // sidesteps the whole class of problem by construction). The bound below is NOT "close to the
    // walls' own combined circumference" - with spacing=2000 on this fixture there are ~188
    // crossings, each legitimately close to the ~15mm wall-to-wall gap, so crossings alone
    // legitimately dominate the total at several thousand mm, an order of magnitude above the
    // walls' own ~278mm combined circumference (confirmed directly: a first version of this bound
    // was miscalibrated against the circumference alone and flagged this fixture's own correct,
    // already-fixed output as if it were still buggy). A real "long way around" bug would instead
    // replace most short skin hops with an almost-full-circumference detour, an order of magnitude
    // beyond even the legitimate crossing-dominated total - the generous bound below is picked to
    // sit well above the latter, not to approximate the former precisely.
    const double generous_crossing_count = 400.0; // comfortably above any count this file's fixtures produce
    const double generous_bound = 2.0 * (2.0 * std::numbers::pi * 30000.0 + 2.0 * std::numbers::pi * 15000.0) + generous_crossing_count * 40000.0;
    EXPECT_LT(total_length, generous_bound) << "Total path length should stay within a generous bound of legitimate circumference-plus-crossings, not blow up from repeatedly taking the long way around.";
}

/*!
 * Chain domain support (spec REV 2.1): a Chain domain (this elongated-bar fixture, with no inner
 * wall the way a Ring has - "outer"/"inner" here just means the bar's two long edges) should now
 * produce a real, *open* linked skin path - not a fallback to std::nullopt (that was the old
 * behavior before Chain support existed; see LinkedSkinFallsBackForMultipleChainDomains below for
 * the fallback case that still genuinely applies). Unlike Ring, a Chain domain does not need
 * Crosshatch's CW family (see build_domain_events' own "Chain domains" doc comment in
 * stage10.hpp) - this fixture's crossover_pitch_mm is left at 0 deliberately, to confirm that.
 */
TEST(VbctAdapterTest, LinkedSkinProducesOpenAlternatingPathForChainDomain)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar - a Chain domain.

    const std::optional<OpenLinesSet> lines
        = VbctAdapter::corrugateLinkedSkin(bar_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000, /*z=*/0, /*crossover_pitch_mm=*/0.0);

    ASSERT_TRUE(lines.has_value());
    ASSERT_EQ(lines->size(), 1u) << "Linked skin should produce exactly one connected toolpath, not independent stringer segments.";
    const OpenPolyline& path = (*lines)[0];
    ASSERT_GE(path.size(), 4u);
    EXPECT_NE(path.front(), path.back()) << "A Chain's own linked skin should be an OPEN path (it runs from one end of the corridor to the other), not a closed loop.";

    // Classify by distance from the bar's own long centerline (y=5000) - "outer"/"inner" here are
    // just the bar's two long edges (y near 0 and y near 10000).
    auto classify_upper = [&](const Point2LL& p) { return p.Y > 5000; };
    std::vector<bool> classifications;
    classifications.reserve(path.size());
    for (const Point2LL& p : path)
    {
        classifications.push_back(classify_upper(p));
    }
    bool saw_upper = false;
    bool saw_lower = false;
    size_t transitions = 0;
    for (size_t i = 0; i < classifications.size(); ++i)
    {
        (classifications[i] ? saw_upper : saw_lower) = true;
        if (i > 0 && classifications[i] != classifications[i - 1])
        {
            ++transitions;
        }
    }
    EXPECT_TRUE(saw_upper);
    EXPECT_TRUE(saw_lower);
    EXPECT_GT(transitions, 1u) << "Should cross between the two long edges more than once along the bar's own length.";
    EXPECT_LT(transitions, classifications.size()) << "Wall changes should happen only at crossings, not at every single point.";

    // No implausibly large jump between consecutive points - same reasoning as
    // LinkedSkinProducesClosedAlternatingLoopWithNoGaps's own bound, calibrated to this fixture's
    // ~10mm short axis instead of that test's ~15mm gap.
    double max_gap = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const Point2LL delta = path[i + 1] - path[i];
        const double seg_len = std::sqrt(static_cast<double>(delta.X) * delta.X + static_cast<double>(delta.Y) * delta.Y);
        max_gap = std::max(max_gap, seg_len);
    }
    EXPECT_LT(max_gap, 20000.0) << "No consecutive point pair should be separated by an implausibly large jump.";
}

/*!
 * Chain junction (Hub) support (spec REV 2.1): multiple Chain domains in the same layer are now
 * linked *independently* of each other, not just as a single all-or-nothing "exactly one domain"
 * requirement - two disjoint bars (no shared cap point at all, so mergeStraightPassThroughChains
 * has nothing to merge) should each get their own open linked-skin path.
 */
TEST(VbctAdapterTest, LinkedSkinLinksMultipleDisjointChainDomainsIndependently)
{
    Shape two_bars_shape;
    two_bars_shape.push_back(makeRectPolygon(0, 0, 60000, 10000)); // First 60mm x 10mm bar.
    two_bars_shape.push_back(makeRectPolygon(0, 40000, 60000, 50000)); // Second, disjoint bar, well clear of the first.

    const std::optional<OpenLinesSet> lines
        = VbctAdapter::corrugateLinkedSkin(two_bars_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000, /*z=*/0, /*crossover_pitch_mm=*/0.0);

    ASSERT_TRUE(lines.has_value());
    ASSERT_EQ(lines->size(), 2u) << "Two disjoint Chain domains should each get their own independent linked-skin path.";
    for (const OpenPolyline& path : *lines)
    {
        EXPECT_GE(path.size(), 4u);
        EXPECT_NE(path.front(), path.back()) << "Each Chain domain's own path should be open, not a closed loop.";
    }
}

namespace
{

// A T-shaped fixture: a 120mm x 20mm horizontal crossbar (y in [40000,60000]) with a 10mm x 40mm
// vertical stem centered under it (x in [55000,65000], y in [0,40000]) - matches the general shape
// of the real Tee.stl model whose own topology (spec REV 2.1, Chain junction investigation)
// motivated this feature: a straight crossbar VBCT's own Stage 8/9 splits into two Chain domains
// at the point the stem attaches, plus the stem itself as a third domain, all three sharing one
// exact hub point. Perfectly straight here (crossbar halves at exactly 180 degrees, stem at
// exactly 90) rather than the real model's ~177/~90 - a clean unit-test case; the angle-tolerance
// setting exists specifically to also cover the not-quite-perfectly-straight real case.
Polygon makeTeePolygon()
{
    Polygon polygon;
    polygon.push_back(Point2LL(0, 40000));
    polygon.push_back(Point2LL(0, 60000));
    polygon.push_back(Point2LL(120000, 60000));
    polygon.push_back(Point2LL(120000, 40000));
    polygon.push_back(Point2LL(65000, 40000));
    polygon.push_back(Point2LL(65000, 0));
    polygon.push_back(Point2LL(55000, 0));
    polygon.push_back(Point2LL(55000, 40000));
    return polygon;
}

} // namespace

/*!
 * Chain junction (Hub) support (spec REV 2.1): the Tee's own crossbar (two Chain domains meeting
 * at 180 degrees) should merge into one continuous path spanning close to the full 120mm crossbar
 * length, not two ~60mm stubs - the direct, real-topology-derived regression test for the bug
 * report this feature fixes ("no linked skin for any of the chain domains on the Tee mesh").
 */
TEST(VbctAdapterTest, LinkedSkinMergesStraightPassThroughChainPairAtJunction)
{
    Shape tee_shape;
    tee_shape.push_back(makeTeePolygon());

    const std::optional<OpenLinesSet> lines = VbctAdapter::corrugateLinkedSkin(
        tee_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000, /*z=*/0, /*crossover_pitch_mm=*/0.0,
        /*anchor_t0_frac=*/std::nullopt, /*other_wall_t0_frac=*/std::nullopt, /*reverse_canonical_wall=*/std::nullopt,
        /*chain_anchor_point=*/std::nullopt, /*chain_junction_merge_angle_deg=*/30.0);

    ASSERT_TRUE(lines.has_value());
    ASSERT_EQ(lines->size(), 2u) << "Merged crossbar + separate stem = two independent paths.";

    // The merged crossbar's own path should span close to the full 120mm crossbar - not be capped
    // near 60mm the way an unmerged half would be.
    double max_x_span = 0.0;
    for (const OpenPolyline& path : *lines)
    {
        coord_t min_x = std::numeric_limits<coord_t>::max();
        coord_t max_x = std::numeric_limits<coord_t>::min();
        for (const Point2LL& p : path)
        {
            min_x = std::min(min_x, p.X);
            max_x = std::max(max_x, p.X);
        }
        max_x_span = std::max(max_x_span, static_cast<double>(max_x - min_x));
    }
    EXPECT_GT(max_x_span, 100000.0) << "The merged crossbar's own path should span most of the full 120mm crossbar length.";
}

/*!
 * Chain junction (Hub) support (spec REV 2.1): the angle tolerance actually gates whether a
 * pass-through pair merges. A T whose crossbar is bent 150 degrees (30 degrees off straight, not
 * 180) should merge at a loose 40-degree tolerance but stay as two separate stubs at a tight
 * 10-degree tolerance.
 */
TEST(VbctAdapterTest, ChainJunctionMergeAngleToleranceControlsWhetherABentCrossbarMerges)
{
    // Same T as makeTeePolygon, but the right half of the crossbar is bent downward by 30 degrees
    // at the junction (150 degrees total, not a perfectly straight 180).
    Polygon bent_tee;
    bent_tee.push_back(Point2LL(0, 40000));
    bent_tee.push_back(Point2LL(0, 60000));
    bent_tee.push_back(Point2LL(65000, 60000));
    bent_tee.push_back(Point2LL(95000, 40000)); // right half's own end bent down and in.
    bent_tee.push_back(Point2LL(85000, 26000));
    bent_tee.push_back(Point2LL(65000, 40000));
    bent_tee.push_back(Point2LL(65000, 0));
    bent_tee.push_back(Point2LL(55000, 0));
    bent_tee.push_back(Point2LL(55000, 40000));
    Shape bent_tee_shape;
    bent_tee_shape.push_back(bent_tee);

    auto max_span = [&](double tolerance_deg) -> double
    {
        const std::optional<OpenLinesSet> lines = VbctAdapter::corrugateLinkedSkin(
            bent_tee_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000, /*z=*/0, /*crossover_pitch_mm=*/0.0,
            /*anchor_t0_frac=*/std::nullopt, /*other_wall_t0_frac=*/std::nullopt, /*reverse_canonical_wall=*/std::nullopt,
            /*chain_anchor_point=*/std::nullopt, tolerance_deg);
        if (! lines.has_value())
        {
            return -1.0;
        }
        double best = 0.0;
        for (const OpenPolyline& path : *lines)
        {
            double len = 0.0;
            for (size_t i = 0; i + 1 < path.size(); ++i)
            {
                const Point2LL delta = path[i + 1] - path[i];
                len += std::sqrt(static_cast<double>(delta.X) * delta.X + static_cast<double>(delta.Y) * delta.Y);
            }
            best = std::max(best, len);
        }
        return best;
    };

    const double loose_tolerance_span = max_span(40.0);
    const double tight_tolerance_span = max_span(10.0);
    ASSERT_GE(loose_tolerance_span, 0.0);
    ASSERT_GE(tight_tolerance_span, 0.0);
    EXPECT_GT(loose_tolerance_span, tight_tolerance_span)
        << "A looser angle tolerance should merge the bent crossbar into a longer single path than a tighter one that leaves it split.";
}

/*!
 * vbct::build_domain_events requires crosshatch_enabled - with only the CCW family available,
 * every stringer runs outer-to-inner and none run the other way, so there's no "return trip" to
 * alternate against at all (see build_domain_events' own doc comment in stage10.hpp).
 */
TEST(VbctAdapterTest, BuildDomainEventsRequiresCrosshatchEnabled)
{
    std::vector<vbct::Contour> contours;
    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 64));
    ring_shape.push_back(makeCirclePolygon(0, 0, 15000, 48));
    contours = VbctAdapter::shapeToContours(ring_shape);

    const vbct::VbctResult r14 = vbct::run_vbct(contours, 0.3);
    const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
    const vbct::Stage6Result r6 = vbct::run_stage6(r5);
    const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
    const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
    const vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, 0.8);

    const std::vector<vbct::DomainEvents> without_crosshatch = vbct::build_domain_events(r9, 2.0, /*phase_offset=*/0.15, std::nullopt, std::nullopt, std::nullopt, /*crosshatch_enabled=*/false);
    ASSERT_EQ(without_crosshatch.size(), 1u);
    EXPECT_FALSE(without_crosshatch[0].ok);

    const std::vector<vbct::DomainEvents> with_crosshatch = vbct::build_domain_events(r9, 2.0, /*phase_offset=*/0.15, std::nullopt, std::nullopt, std::nullopt, /*crosshatch_enabled=*/true);
    ASSERT_EQ(with_crosshatch.size(), 1u);
    EXPECT_TRUE(with_crosshatch[0].ok);
}

/*!
 * Both families' events are always built, unconditionally - no coincidence detection/dedup, even
 * at phase_offset=0 where the two families' own positions coincide exactly (see
 * build_domain_events' own doc comment for why an earlier version's single-representative-index
 * dedup was removed rather than further patched, per direct user report and direction). Event
 * count should be the same (2*n_even) regardless of phase_offset.
 */
TEST(VbctAdapterTest, BuildDomainEventsAlwaysIncludesBothFamiliesRegardlessOfPhase)
{
    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 64));
    ring_shape.push_back(makeCirclePolygon(0, 0, 15000, 48));
    const std::vector<vbct::Contour> contours = VbctAdapter::shapeToContours(ring_shape);

    const vbct::VbctResult r14 = vbct::run_vbct(contours, 0.3);
    const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
    const vbct::Stage6Result r6 = vbct::run_stage6(r5);
    const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
    const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
    const vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, 0.8);

    const std::vector<vbct::DomainEvents> at_zero_phase = vbct::build_domain_events(r9, 2.0, /*phase_offset=*/0.0, std::nullopt, std::nullopt, std::nullopt, /*crosshatch_enabled=*/true);
    const std::vector<vbct::DomainEvents> at_nonzero_phase = vbct::build_domain_events(r9, 2.0, /*phase_offset=*/0.15, std::nullopt, std::nullopt, std::nullopt, /*crosshatch_enabled=*/true);

    ASSERT_EQ(at_zero_phase.size(), 1u);
    ASSERT_EQ(at_nonzero_phase.size(), 1u);
    ASSERT_TRUE(at_zero_phase[0].ok);
    ASSERT_TRUE(at_nonzero_phase[0].ok);
    EXPECT_EQ(at_zero_phase[0].events.size(), at_nonzero_phase[0].events.size())
        << "Event count should not depend on phase_offset now that coincidence dedup has been removed.";
}

/*!
 * Chain domain support (spec REV 2.1): unlike Ring, build_domain_events should produce a real,
 * linkable (ok=true, is_ring=false) event sequence for a Chain domain *without* requiring
 * crosshatch_enabled - see build_domain_events' own "Chain domains" doc comment in stage10.hpp for
 * why an open wall pair's own single family already suffices (no "return trip" problem the way
 * Ring's closed-loop single family has). Event count should equal the domain's own stringer count
 * exactly (no forced-even rounding, unlike Ring).
 */
TEST(VbctAdapterTest, BuildDomainEventsChainDoesNotRequireCrosshatch)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar - a Chain domain.
    const std::vector<vbct::Contour> contours = VbctAdapter::shapeToContours(bar_shape);

    const vbct::VbctResult r14 = vbct::run_vbct(contours, 0.3);
    const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
    const vbct::Stage6Result r6 = vbct::run_stage6(r5);
    const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
    const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
    const vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, 0.8);

    const std::vector<vbct::DomainEvents> without_crosshatch = vbct::build_domain_events(r9, 2.0, /*phase_offset=*/0.0, std::nullopt, std::nullopt, std::nullopt, /*crosshatch_enabled=*/false);
    ASSERT_EQ(without_crosshatch.size(), 1u);
    ASSERT_TRUE(without_crosshatch[0].ok) << "A Chain domain should be linkable without crosshatch_enabled, unlike Ring.";
    EXPECT_FALSE(without_crosshatch[0].is_ring);
    EXPECT_GE(without_crosshatch[0].events.size(), 2u);

    // crosshatch_enabled should have no effect on a Chain domain's own event count (it never builds
    // a CW family for Chain - see prepare_domain_sampling's own effective_phase_offset=0.0 for
    // Chain, and this function's own doc comment).
    const std::vector<vbct::DomainEvents> with_crosshatch = vbct::build_domain_events(r9, 2.0, /*phase_offset=*/0.0, std::nullopt, std::nullopt, std::nullopt, /*crosshatch_enabled=*/true);
    ASSERT_EQ(with_crosshatch.size(), 1u);
    ASSERT_TRUE(with_crosshatch[0].ok);
    EXPECT_EQ(with_crosshatch[0].events.size(), without_crosshatch[0].events.size());
}

/*!
 * Chain Crosshatch + Linked Corrugation Skin integration (spec REV 2.4): the new
 * chain_crosshatch_enabled parameter defaults to false and should reproduce exactly today's
 * single-family Chain event sequence - a regression guard that this integration doesn't disturb
 * the already-shipped Chain Linked Skin behavior (spec REV 2.1/2.2) for the common case of the
 * feature being off.
 */
TEST(VbctAdapterTest, BuildDomainEventsChainCrosshatchDisabledMatchesBaseline)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar - a Chain domain.
    const std::vector<vbct::Contour> contours = VbctAdapter::shapeToContours(bar_shape);

    const vbct::VbctResult r14 = vbct::run_vbct(contours, 0.3);
    const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
    const vbct::Stage6Result r6 = vbct::run_stage6(r5);
    const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
    const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
    const vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, 0.8);

    // Default (chain_crosshatch_enabled omitted, defaults false) at a nonzero phase - the same
    // phase that, with the parameter explicitly true below, produces a doubled event count.
    const std::vector<vbct::DomainEvents> default_param
        = vbct::build_domain_events(r9, 2.0, /*phase_offset=*/0.31, std::nullopt, std::nullopt, std::nullopt, /*crosshatch_enabled=*/false);
    const std::vector<vbct::DomainEvents> explicit_false = vbct::build_domain_events(
        r9, 2.0, /*phase_offset=*/0.31, std::nullopt, std::nullopt, std::nullopt, /*crosshatch_enabled=*/false, std::nullopt, /*chain_crosshatch_enabled=*/false);

    ASSERT_EQ(default_param.size(), 1u);
    ASSERT_EQ(explicit_false.size(), 1u);
    ASSERT_TRUE(default_param[0].ok);
    ASSERT_TRUE(explicit_false[0].ok);
    EXPECT_EQ(default_param[0].events.size(), explicit_false[0].events.size());
    ASSERT_EQ(default_param[0].events.size(), explicit_false[0].events.size());
    for (size_t i = 0; i < default_param[0].events.size(); ++i)
    {
        EXPECT_DOUBLE_EQ(default_param[0].events[i].theta, explicit_false[0].events[i].theta) << "index " << i;
    }
}

/*!
 * Chain Crosshatch + Linked Corrugation Skin integration (spec REV 2.4): at a genuinely nonzero
 * phase, chain_crosshatch_enabled=true should exactly double the event count (base family + a
 * second, merged-and-sorted crosshatch family) - mirrors
 * ChainCrosshatchEnabledDoublesStringerCountAtNonzeroPhaseOffset's own domain_stringers-level check,
 * at the build_domain_events level instead. Also checks no event is spuriously marked clipped (the
 * same cap-adjacent clip-exemption bug class already found once for domain_stringers - checked here
 * explicitly, not assumed inherited) and that theta is still ascending after the merge sort.
 */
TEST(VbctAdapterTest, BuildDomainEventsChainCrosshatchDoublesEventCountAtNonzeroPhase)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar - a Chain domain.
    const std::vector<vbct::Contour> contours = VbctAdapter::shapeToContours(bar_shape);

    const vbct::VbctResult r14 = vbct::run_vbct(contours, 0.3);
    const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
    const vbct::Stage6Result r6 = vbct::run_stage6(r5);
    const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
    const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
    const vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, 0.8);

    const std::vector<vbct::DomainEvents> without_chain_crosshatch = vbct::build_domain_events(
        r9, 2.0, /*phase_offset=*/0.31, std::nullopt, std::nullopt, std::nullopt, /*crosshatch_enabled=*/false, std::nullopt, /*chain_crosshatch_enabled=*/false);
    const std::vector<vbct::DomainEvents> with_chain_crosshatch = vbct::build_domain_events(
        r9, 2.0, /*phase_offset=*/0.31, std::nullopt, std::nullopt, std::nullopt, /*crosshatch_enabled=*/false, std::nullopt, /*chain_crosshatch_enabled=*/true);

    ASSERT_EQ(without_chain_crosshatch.size(), 1u);
    ASSERT_EQ(with_chain_crosshatch.size(), 1u);
    ASSERT_TRUE(without_chain_crosshatch[0].ok);
    ASSERT_TRUE(with_chain_crosshatch[0].ok);
    EXPECT_FALSE(with_chain_crosshatch[0].is_ring);
    // *2 for the crosshatch family's own doubling, +2 for the always-added true t=0/t=1 cap events
    // (spec REV 2.5) - see build_domain_events_for_domain's own doc comment for why those two are
    // unconditionally present whenever chain_crosshatch_active, on top of the doubled count.
    ASSERT_EQ(with_chain_crosshatch[0].events.size(), without_chain_crosshatch[0].events.size() * 2 + 2);

    for (size_t i = 0; i < with_chain_crosshatch[0].events.size(); ++i)
    {
        EXPECT_FALSE(with_chain_crosshatch[0].events[i].clipped) << "index " << i;
        if (i > 0)
        {
            EXPECT_LE(with_chain_crosshatch[0].events[i - 1].theta, with_chain_crosshatch[0].events[i].theta) << "index " << i;
        }
    }
}

/*!
 * Chain Crosshatch + Linked Corrugation Skin integration (spec REV 2.4), full path: with a real,
 * non-aliasing crossover_pitch_mm/z and chain_crosshatch_enabled=true, corrugateLinkedSkin should still
 * produce one connected, alternating, open path - the same structural checks
 * LinkedSkinProducesOpenAlternatingPathForChainDomain already makes for the disabled case - now
 * with up to twice as many crossings.
 */
TEST(VbctAdapterTest, LinkedSkinCarriesChainCrosshatchDiagonalWhenEnabled)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar - a Chain domain.

    const std::optional<OpenLinesSet> lines_without = VbctAdapter::corrugateLinkedSkin(
        bar_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000, /*z=*/3100, /*crossover_pitch_mm=*/5.0, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
        /*chain_junction_merge_angle_deg=*/30.0, /*chain_crosshatch_enabled=*/false);
    const std::optional<OpenLinesSet> lines_with = VbctAdapter::corrugateLinkedSkin(
        bar_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000, /*z=*/3100, /*crossover_pitch_mm=*/5.0, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
        /*chain_junction_merge_angle_deg=*/30.0, /*chain_crosshatch_enabled=*/true);

    ASSERT_TRUE(lines_without.has_value());
    ASSERT_TRUE(lines_with.has_value());
    ASSERT_EQ(lines_without->size(), 1u);
    ASSERT_EQ(lines_with->size(), 1u);
    const OpenPolyline& path = (*lines_with)[0];
    ASSERT_GT(path.size(), (*lines_without)[0].size()) << "Enabling Chain crosshatch should add real crossings to the linked path, not leave it unchanged.";
    EXPECT_NE(path.front(), path.back()) << "Still an OPEN path with crosshatch enabled, same as without.";

    auto classify_upper = [&](const Point2LL& p) { return p.Y > 5000; };
    bool saw_upper = false;
    bool saw_lower = false;
    size_t transitions = 0;
    bool prev_upper = classify_upper(path[0]);
    for (size_t i = 0; i < path.size(); ++i)
    {
        const bool upper = classify_upper(path[i]);
        (upper ? saw_upper : saw_lower) = true;
        if (i > 0 && upper != prev_upper)
        {
            ++transitions;
        }
        prev_upper = upper;
    }
    EXPECT_TRUE(saw_upper);
    EXPECT_TRUE(saw_lower);
    EXPECT_GT(transitions, 1u) << "Should still cross between the two long edges more than once.";
}

/*!
 * Chain domain support (spec REV 2.1): chain_anchor_point overrides which physical end
 * (domain.cap_start vs. domain.cap_end) both of a Chain domain's walls get oriented toward - see
 * that parameter's own doc comment in stage10.hpp for why this is the entire continuity mechanism
 * (no separate direction flag needed the way Ring's reverse_canonical_wall is: orient_toward's own
 * nearest-endpoint search already is the direction fix, once given a stable target). Passing
 * domain.cap_end as the override should reproduce exactly the same stringers a domain whose own
 * cap_start/cap_end were swapped would produce with no override at all - i.e. every wall gets
 * walked in the opposite direction from the un-overridden default.
 */
TEST(VbctAdapterTest, BuildDomainEventsChainAnchorPointOverridesOrientation)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar - a Chain domain.
    const std::vector<vbct::Contour> contours = VbctAdapter::shapeToContours(bar_shape);

    const vbct::VbctResult r14 = vbct::run_vbct(contours, 0.3);
    const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
    const vbct::Stage6Result r6 = vbct::run_stage6(r5);
    const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
    const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
    const vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, 0.8);

    const vbct::Domain* chain = nullptr;
    for (const auto& dom : r9.domains)
    {
        if (dom.kind == "chain")
        {
            chain = &dom;
            break;
        }
    }
    ASSERT_NE(chain, nullptr);
    ASSERT_TRUE(chain->cap_start.has_value());
    ASSERT_TRUE(chain->cap_end.has_value());

    const std::vector<vbct::DomainEvents> default_events = vbct::build_domain_events(r9, 2.0, /*phase_offset=*/0.0, std::nullopt, std::nullopt, std::nullopt, /*crosshatch_enabled=*/false);
    const std::vector<vbct::DomainEvents> overridden_events = vbct::build_domain_events(
        r9, 2.0, /*phase_offset=*/0.0, std::nullopt, std::nullopt, std::nullopt, /*crosshatch_enabled=*/false, *chain->cap_end);
    ASSERT_EQ(default_events.size(), 1u);
    ASSERT_EQ(overridden_events.size(), 1u);
    ASSERT_TRUE(default_events[0].ok);
    ASSERT_TRUE(overridden_events[0].ok);
    ASSERT_EQ(default_events[0].events.size(), overridden_events[0].events.size());

    // The default's first event (t=0) should be oriented toward cap_start (closer to it than to
    // cap_end); the overridden one's first event should instead be oriented toward cap_end.
    const vbct::Point2& default_first = default_events[0].events.front().outer_pt;
    const vbct::Point2& overridden_first = overridden_events[0].events.front().outer_pt;
    auto dist = [](const vbct::Point2& a, const vbct::Point2& b) { return std::hypot(a.x - b.x, a.y - b.y); };
    EXPECT_LT(dist(default_first, *chain->cap_start), dist(default_first, *chain->cap_end)) << "Without an override, the first event should sit near cap_start (today's existing default).";
    EXPECT_LT(dist(overridden_first, *chain->cap_end), dist(overridden_first, *chain->cap_start)) << "With chain_anchor_point=cap_end, the first event should instead sit near cap_end.";
}

/*!
 * A Ring domain whose inner wall is NOT concentric with its outer wall (an off-center hole - an
 * entirely ordinary shape, not a degenerate edge case) should still produce sane, radially-scaled
 * stringers. Regression test for a real bug: an earlier version of the t=0 anchor fix measured
 * each wall's angle from its own independent centroid, which implicitly assumed concentricity -
 * for a genuinely off-center hole the two walls' centroids differ, so "angle 0 from its own
 * centroid" pointed in unrelated physical directions on each wall, producing a nonsensical
 * correspondence (confirmed breaking badly on a real off-center print). The fix measures both
 * walls' angles from one shared reference point instead.
 */
TEST(VbctAdapterTest, OffCenterHoleProducesSaneStringers)
{
    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 64)); // 30mm outer radius, centered at origin.
    ring_shape.push_back(makeCirclePolygon(5000, 0, 10000, 48)); // 10mm inner radius, centered 5mm off-axis.

    const std::optional<OpenLinesSet> lines = VbctAdapter::corrugate(ring_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000);

    ASSERT_TRUE(lines.has_value());
    EXPECT_FALSE(lines->empty());
    for (const OpenPolyline& line : *lines)
    {
        ASSERT_GE(line.size(), 2u);
        const Point2LL delta = line.back() - line.front();
        const double length = std::sqrt(static_cast<double>(delta.X) * delta.X + static_cast<double>(delta.Y) * delta.Y);
        // The gap between the two walls ranges from 10mm (nearest, where they're 5mm apart in
        // center offset plus the 10mm/20mm radius difference) up to 40mm (farthest); a broken
        // correspondence (mismatched anchor points) would produce stringers far outside that -
        // e.g. spanning close to the outer wall's own 188mm circumference.
        EXPECT_LT(length, 45000.0) << "Stringer length should track the local wall-to-wall gap, not jump across unrelated parts of the ring.";
    }
}

/*!
 * Regression test for a real, confirmed bug in reference_angle_arc_length (stage10.cpp): its
 * crossing search checked whether the running *unwrapped* swept angle crossed the literal value
 * 0, which only works when the wall's own starting vertex happens to sit at a positive angle
 * from the shared centroid. A wall whose starting vertex sits at a *negative* angle near -180
 * degrees sweeps monotonically from there down to roughly -540 degrees over one full loop -
 * entirely negative throughout, never touching literal 0, even though it genuinely passes
 * through the true physical reference direction (at -360 degrees, not 0). Confirmed happening in
 * production: the two walls' own starting vertices, both near the same physical spot after
 * VBCT's wall-start-point fix, differed by a hair (+179.997 vs -179.952 degrees) - enough to put
 * one wall on each side of this bug. This test constructs that exact asymmetry directly: outer
 * wall starting just above +180 degrees, inner wall starting just above -180 degrees.
 */
TEST(VbctAdapterTest, RingStartingNearNegative180DegreesProducesSaneStringers)
{
    Shape ring_shape;
    // Outer wall's first point at ~179 degrees (positive side); inner wall's first point at
    // ~-179 degrees (negative side) - the exact asymmetry that broke the old literal-zero check.
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 64, 179.0 * std::numbers::pi / 180.0));
    ring_shape.push_back(makeCirclePolygon(0, 0, 15000, 48, -179.0 * std::numbers::pi / 180.0));

    const std::optional<OpenLinesSet> lines = VbctAdapter::corrugate(ring_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000);

    ASSERT_TRUE(lines.has_value());
    ASSERT_FALSE(lines->empty());
    for (const OpenPolyline& line : *lines)
    {
        ASSERT_GE(line.size(), 2u);
        const Point2LL delta = line.back() - line.front();
        const double length = std::sqrt(static_cast<double>(delta.X) * delta.X + static_cast<double>(delta.Y) * delta.Y);
        // The true wall-to-wall gap is 15mm; a stringer anchored via the broken fallback (an
        // uncorrected, physically meaningless point near the wall's own arbitrary start) would
        // produce a length far outside any plausible gap for this shape.
        EXPECT_LT(length, 20000.0) << "Stringer length should track the 15mm wall-to-wall gap, not an uncorrected fallback anchor.";
    }
}

/*!
 * A plain concentric ring positioned away from the coordinate origin - matching a real print
 * placed somewhere other than the plate's own (0,0), e.g. bed-centered around (110mm,110mm) -
 * should still rotate its stringer family across Z the same way a ring at the origin does.
 * Regression test for a real bug report: a ring at the origin rotated correctly (confirmed via
 * PhaseRateShiftsStringersAcrossZ above), but the exact same ring moved off (110mm,110mm)
 * produced zero rotation between layers - every layer's stringers landed in the identical
 * position, confirmed both visually (Cura's layer-view showing perfectly stacked, non-twisting
 * spokes) and numerically (adjacent-layer cross-correlation measuring exactly 0.0 degrees of
 * shift, layer after layer, on the real exported g-code).
 */
TEST(VbctAdapterTest, TranslatedRingStillRotatesAcrossZ)
{
    constexpr coord_t plate_offset = 110000; // 110mm, matching a typical bed center.
    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(plate_offset, plate_offset, 30000, 64));
    ring_shape.push_back(makeCirclePolygon(plate_offset, plate_offset, 15000, 48));

    const std::optional<OpenLinesSet> lines_z0 = VbctAdapter::corrugate(ring_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000, /*z=*/0, /*crossover_pitch_mm=*/5.0);
    const std::optional<OpenLinesSet> lines_z5mm = VbctAdapter::corrugate(ring_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000, /*z=*/5000, /*crossover_pitch_mm=*/5.0);

    ASSERT_TRUE(lines_z0.has_value());
    ASSERT_TRUE(lines_z5mm.has_value());
    ASSERT_FALSE(lines_z0->empty());
    ASSERT_EQ(lines_z0->size(), lines_z5mm->size());

    bool any_moved = false;
    for (size_t i = 0; i < lines_z0->size(); ++i)
    {
        if ((*lines_z0)[i].front() != (*lines_z5mm)[i].front())
        {
            any_moved = true;
            break;
        }
    }
    EXPECT_TRUE(any_moved) << "Changing Z with a nonzero crossover_pitch_mm should shift the stringer family's sampling, even when the ring isn't centered at the origin.";
}

/*!
 * An elongated solid bar (like Test Models - Diagonal.stl, the case that broke this project's
 * earlier Ring-only correspondence algorithm) is a VBCT Chain domain, not a Ring - it should
 * still produce sane stringers, following the bar's own length, via VBCT's real skeleton-based
 * wall extraction instead of the old angular-about-centroid approximation.
 */
TEST(VbctAdapterTest, ElongatedBarShapeProducesStringers)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar.

    const std::optional<OpenLinesSet> lines = VbctAdapter::corrugate(bar_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000);

    ASSERT_TRUE(lines.has_value());
    EXPECT_FALSE(lines->empty());

    // Every stringer should span roughly the bar's short axis (~10mm), not its long axis - a
    // regression check against the old Ring-only algorithm's skew, which produced stringers at
    // arbitrary angles unrelated to the local wall-to-wall gap.
    for (const OpenPolyline& line : *lines)
    {
        ASSERT_GE(line.size(), 2u);
        const Point2LL delta = line.back() - line.front();
        const double length = std::sqrt(static_cast<double>(delta.X) * delta.X + static_cast<double>(delta.Y) * delta.Y);
        EXPECT_LT(length, 15000.0) << "Stringer length should track the bar's ~10mm short axis, not its 60mm long axis.";
    }
}

/*!
 * Chain-specific Crosshatch (spec REV 2.3/2.4): crosshatch_enabled=true now always doubles the
 * stringer count for a Chain domain, *including* at phase_offset exactly 0 (z=0, crossover_pitch_mm=0) -
 * this replaces an earlier version of this test (ChainCrosshatchProducesNoExtraStringersAt
 * ZeroPhaseOffset) that expected the opposite. That earlier expectation came from a per-layer
 * gate that turned crosshatch off whenever phase_offset itself was near a multiple of 1.0 -
 * confirmed via direct user report on a real print to cause a visible single-layer discontinuity
 * whenever an *animated* print's own phase_offset happened to coincide with a multiple of 1.0 at
 * one specific layer (the gate couldn't tell that case apart from a genuinely static whole print,
 * since both look identical from phase_offset alone - see stage10.cpp's own top-of-file history
 * note for the full account). Removing the per-layer gate fixes the discontinuity but means
 * a genuinely static print (crossover_pitch_mm=0.0) now also gets a constant, non-animated crosshatch
 * offset rather than silently falling back to the single-family default - a deliberate, accepted
 * trade-off, not an oversight.
 */
TEST(VbctAdapterTest, ChainCrosshatchAlwaysDoublesStringerCountIncludingAtZeroPhaseOffset)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000));

    const std::optional<OpenLinesSet> lines_single = VbctAdapter::corrugate(bar_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000);
    const std::optional<OpenLinesSet> lines_cross = VbctAdapter::corrugate(
        bar_shape,
        /*x=*/0.3,
        /*threshold=*/800,
        /*spacing=*/2000,
        /*z=*/0,
        /*crossover_pitch_mm=*/0.0,
        /*anchor_t0_frac=*/std::nullopt,
        /*other_wall_t0_frac=*/std::nullopt,
        /*reverse_canonical_wall=*/std::nullopt,
        /*crosshatch_enabled=*/true);

    ASSERT_TRUE(lines_single.has_value());
    ASSERT_TRUE(lines_cross.has_value());
    // *2 for the crosshatch family's own doubling, +2 for the always-added true t=0/t=1 cap
    // stringers (spec REV 2.5) - see domain_stringers' own doc comment for why those two are
    // unconditionally present whenever chain_crosshatch_active, on top of the doubled count.
    EXPECT_EQ(lines_cross->size(), lines_single->size() * 2 + 2);
}

namespace
{

// Runs Stages 1-9 directly (matching this file's own established pattern, e.g.
// BuildDomainEventsRequiresCrosshatchEnabled above) and returns vbct::run_stage10's own
// Stage10Result for a single-domain shape - used by the Chain crosshatch tests below to inspect
// individual Stringer objects directly, at the domain_stringers level, rather than the flattened
// OpenLinesSet corrugate() returns. This sidesteps a real fragility a first version of these tests
// had: corrugate()'s own output flattens each Stringer's own (possibly >1, if VBS-subdivision
// noise causes an occasional clip) pieces into separate consecutive lines, which breaks any
// "every Nth output line is a crosshatch stringer" assumption the moment any single stringer -
// base or crosshatch - happens to clip into more than one piece. domain_stringers itself still
// pushes exactly one Stringer per index regardless of its own later piece count, so indexing
// Stage10Result::domains[0].stringers directly is robust to that.
vbct::Stage10Result runStage10Directly(const Shape& shape, double x, double threshold_mm, double spacing_mm, double phase_offset, bool crosshatch_enabled)
{
    const std::vector<vbct::Contour> contours = VbctAdapter::shapeToContours(shape);
    const vbct::VbctResult r14 = vbct::run_vbct(contours, x);
    const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
    const vbct::Stage6Result r6 = vbct::run_stage6(r5);
    const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
    const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
    const vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, threshold_mm);
    return vbct::run_stage10(r9, spacing_mm, phase_offset, std::nullopt, std::nullopt, std::nullopt, crosshatch_enabled);
}

} // namespace

/*!
 * Chain-specific Crosshatch (spec REV 2.4, "fifth round" - Ring-style wraparound instead of the
 * earlier reflecting "bounce"): unlike every bounce-based round (which topped out at 50%
 * opposite-direction motion at best - the "fourth round"'s own trade-off), the two families now
 * move in genuinely opposite directions at EVERY phase step, matching Ring's own crosshatch
 * (base_t advances with +phase_offset, the CW family with -phase_offset). Checked directly and
 * numerically: samples the base and crosshatch families' own first (i=0) stringer X position at
 * many consecutive phase_offset pairs across a full cycle and requires the two families' own
 * deltas to have opposite sign at every single sampled step (not just on average) - away from the
 * wrap point itself, where a family's own position can jump a large distance in one step and
 * isn't a meaningful "direction" (see the wrap-crossing exclusion below).
 */
TEST(VbctAdapterTest, ChainCrosshatchFamiliesMoveOppositeDirectionsEveryPhaseStep)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar - a Chain domain.

    auto leading_edge_xs = [&](double phase_offset) {
        const vbct::Stage10Result cross = runStage10Directly(bar_shape, 0.3, 0.8, 2.0, phase_offset, /*crosshatch_enabled=*/true);
        const std::vector<vbct::Stringer>& stringers = cross.domains.at(0).stringers;
        return std::make_pair(stringers.at(0).pieces.front().first.x, stringers.at(1).pieces.front().first.x);
    };

    constexpr int kSamples = 200;
    int opposite_count = 0;
    int compared_count = 0;
    double prev_base = 0.0;
    double prev_cw = 0.0;
    for (int i = 0; i <= kSamples; ++i) {
        const double po = static_cast<double>(i) / kSamples;
        const auto [base_x, cw_x] = leading_edge_xs(po);
        if (i > 0) {
            const double d_base = base_x - prev_base;
            const double d_cw = cw_x - prev_cw;
            // Skip steps that straddle a wrap point (a large single-step jump in either family,
            // where "direction" isn't meaningful) - a real, deliberate cost of this construction,
            // not something this test is meant to characterize.
            const double corridor_estimate = 60.0; // mm, matches this fixture's own bar length
            if (std::abs(d_base) < corridor_estimate * 0.5 && std::abs(d_cw) < corridor_estimate * 0.5) {
                ++compared_count;
                if (d_base * d_cw < 0.0) ++opposite_count;
            }
        }
        prev_base = base_x;
        prev_cw = cw_x;
    }

    ASSERT_GT(compared_count, kSamples / 2) << "Too many steps excluded as wrap-crossings - sample count or exclusion threshold may need adjusting.";
    EXPECT_EQ(opposite_count, compared_count) << "Every non-wrap-crossing step should show the two families moving in opposite directions.";
}

/*!
 * Chain-specific Crosshatch (spec REV 2.4, "fifth round"): the wraparound construction (n
 * evenly-spaced points, rotated by ±phase_offset) does NOT provably guarantee full coverage the
 * way the mirrored bounce design's own algebraic complementarity once did (see this test's own
 * numeric derivation, done before implementing this round: n points spaced exactly 1/(n-1) apart,
 * linearly sorted after wrapping, leave a gap of up to one spacing unit near whichever real end
 * the current phase's own "seam" happens to land closest to) - but that gap is small (bounded by
 * roughly one sample spacing) and worst only very near phase_offset=0 (where both families'
 * formulas momentarily coincide), not sustained over a wide phase range the way the very first,
 * narrow-comb_span design once was. Checked directly and numerically: scans phase_offset across a
 * full cycle and requires the worst-case gap at either end (measured against a crosshatch-disabled
 * baseline run establishing the domain's own true corridor bounds, not a hardcoded length) to stay
 * under roughly two sample spacings - loose enough to allow the expected phase_offset=0 spike, not
 * a tight regression.
 */
TEST(VbctAdapterTest, ChainCrosshatchCoverageGapStaysBoundedAcrossFullPhaseCycle)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar - a Chain domain.

    const vbct::Stage10Result baseline = runStage10Directly(bar_shape, 0.3, 0.8, 2.0, /*phase_offset=*/0.0, /*crosshatch_enabled=*/false);
    const std::vector<vbct::Stringer>& baseline_stringers = baseline.domains.at(0).stringers;
    ASSERT_FALSE(baseline_stringers.empty());
    double corridor_min_x = std::numeric_limits<double>::max();
    double corridor_max_x = std::numeric_limits<double>::lowest();
    for (const vbct::Stringer& stringer : baseline_stringers) {
        corridor_min_x = std::min(corridor_min_x, stringer.pieces.front().first.x);
        corridor_max_x = std::max(corridor_max_x, stringer.pieces.front().first.x);
    }
    const double corridor_length = corridor_max_x - corridor_min_x;
    ASSERT_GT(corridor_length, 0.0);
    const double spacing_unit = corridor_length / static_cast<double>(baseline_stringers.size() - 1);
    const double max_allowed_gap = spacing_unit * 2.0;

    double worst_gap = 0.0;
    constexpr int kSamples = 400;
    for (int i = 0; i < kSamples; ++i) {
        const double po = static_cast<double>(i) / kSamples;
        const vbct::Stage10Result cross = runStage10Directly(bar_shape, 0.3, 0.8, 2.0, po, /*crosshatch_enabled=*/true);
        const std::vector<vbct::Stringer>& stringers = cross.domains.at(0).stringers;
        ASSERT_FALSE(stringers.empty()) << "po=" << po;
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        for (const vbct::Stringer& stringer : stringers) {
            for (const vbct::StringerEdge& piece : stringer.pieces) {
                min_x = std::min({ min_x, piece.first.x, piece.second.x });
                max_x = std::max({ max_x, piece.first.x, piece.second.x });
            }
        }
        const double gap_left = min_x - corridor_min_x;
        const double gap_right = corridor_max_x - max_x;
        worst_gap = std::max({ worst_gap, gap_left, gap_right });
    }

    EXPECT_LE(worst_gap, max_allowed_gap) << "worst_gap=" << worst_gap << " max_allowed=" << max_allowed_gap << " spacing_unit=" << spacing_unit;
}

/*!
 * Chain-specific Crosshatch (spec REV 2.5): unlike the bounded-but-nonzero gap the wraparound
 * family's own n samples leave near the ends (see ChainCrosshatchCoverageGapStaysBoundedAcross-
 * FullPhaseCycle just above - a real, accepted property of that family alone), the domain's own
 * two true end caps (t=0/t=1) themselves must now be exactly represented at every phase, via the
 * two always-added fixed cap stringers (domain_stringers) and fixed cap events
 * (build_domain_events_for_domain) - this is the actual fix for the real-print "stops short of the
 * endcaps" report, and is checked directly here rather than assumed from the bounded-gap test
 * above, which only characterizes the wraparound family's own samples, not the two fixed additions.
 */
TEST(VbctAdapterTest, ChainCrosshatchAlwaysReachesTrueEndCapsAcrossFullPhaseCycle)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar - a Chain domain.

    const vbct::Stage10Result baseline = runStage10Directly(bar_shape, 0.3, 0.8, 2.0, /*phase_offset=*/0.0, /*crosshatch_enabled=*/false);
    const std::vector<vbct::Stringer>& baseline_stringers = baseline.domains.at(0).stringers;
    ASSERT_FALSE(baseline_stringers.empty());
    double corridor_min_x = std::numeric_limits<double>::max();
    double corridor_max_x = std::numeric_limits<double>::lowest();
    for (const vbct::Stringer& stringer : baseline_stringers) {
        corridor_min_x = std::min(corridor_min_x, stringer.pieces.front().first.x);
        corridor_max_x = std::max(corridor_max_x, stringer.pieces.front().first.x);
    }
    ASSERT_GT(corridor_max_x - corridor_min_x, 0.0);
    constexpr double kEps = 1e-6; // mm - floating-point tolerance, not a real gap allowance

    const std::vector<vbct::Contour> contours = VbctAdapter::shapeToContours(bar_shape);
    const vbct::VbctResult r14 = vbct::run_vbct(contours, 0.3);
    const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
    const vbct::Stage6Result r6 = vbct::run_stage6(r5);
    const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
    const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
    const vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, 0.8);

    constexpr int kSamples = 200;
    for (int i = 0; i <= kSamples; ++i) {
        const double po = static_cast<double>(i) / kSamples;

        // domain_stringers: the two always-added cap stringers should place a point exactly at
        // each true corridor extreme.
        const vbct::Stage10Result cross = runStage10Directly(bar_shape, 0.3, 0.8, 2.0, po, /*crosshatch_enabled=*/true);
        const std::vector<vbct::Stringer>& stringers = cross.domains.at(0).stringers;
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        for (const vbct::Stringer& stringer : stringers) {
            for (const vbct::StringerEdge& piece : stringer.pieces) {
                min_x = std::min({ min_x, piece.first.x, piece.second.x });
                max_x = std::max({ max_x, piece.first.x, piece.second.x });
            }
        }
        EXPECT_NEAR(min_x, corridor_min_x, kEps) << "po=" << po;
        EXPECT_NEAR(max_x, corridor_max_x, kEps) << "po=" << po;

        // build_domain_events_for_domain: the two always-added cap events should carry theta (==
        // t_left for Chain) of exactly 0.0 and 1.0.
        const std::vector<vbct::DomainEvents> events_result = vbct::build_domain_events(
            r9, 2.0, po, std::nullopt, std::nullopt, std::nullopt, /*crosshatch_enabled=*/false, std::nullopt, /*chain_crosshatch_enabled=*/true);
        ASSERT_EQ(events_result.size(), 1u) << "po=" << po;
        ASSERT_TRUE(events_result[0].ok) << "po=" << po;
        const std::vector<vbct::DomainEvent>& events = events_result[0].events;
        ASSERT_FALSE(events.empty()) << "po=" << po;
        EXPECT_NEAR(events.front().theta, 0.0, kEps) << "po=" << po;
        EXPECT_NEAR(events.back().theta, 1.0, kEps) << "po=" << po;
    }
}

/*!
 * Away from the degenerate zero-phase case, Chain crosshatch should exactly double the stringer
 * count - mirrors Ring's own CrosshatchEnabledDoublesStringerCountAtNonzeroPhaseOffset.
 */
TEST(VbctAdapterTest, ChainCrosshatchEnabledDoublesStringerCountAtNonzeroPhaseOffset)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000));

    // 0.31 kept from this test's own earlier (bounce-based) history - not load-bearing for the
    // current wraparound construction (base_i = wrap01(i/denom+phase), cw_i = wrap01(i/denom-phase)
    // coincide only when 2*phase is itself a multiple of 1/denom for some index pair, which 0.31
    // avoids for this fixture's own point count) but kept as-is rather than picking a new value
    // with no real reason to prefer one over the other.
    const vbct::Stage10Result single = runStage10Directly(bar_shape, 0.3, 0.8, 2.0, /*phase_offset=*/0.31, /*crosshatch_enabled=*/false);
    const vbct::Stage10Result cross = runStage10Directly(bar_shape, 0.3, 0.8, 2.0, /*phase_offset=*/0.31, /*crosshatch_enabled=*/true);

    ASSERT_EQ(single.domains.size(), 1u);
    ASSERT_EQ(cross.domains.size(), 1u);
    // *2 for the crosshatch family's own doubling, +2 for the always-added true t=0/t=1 cap
    // stringers (spec REV 2.5) - see domain_stringers' own doc comment.
    ASSERT_EQ(cross.domains[0].stringers.size(), single.domains[0].stringers.size() * 2 + 2);

    // Every crosshatch stringer should be a genuinely distinct segment from every base one -
    // mirrors Ring's own CrosshatchFamilyDiffersFromCcwAtNonzeroPhaseOffset. Uses each stringer's
    // own first piece as its representative endpoints (a clipped stringer's own extra pieces are
    // still part of the same one stringer, not a separate one). Stage10Result's own points are in
    // VBCT's mm convention, not the engine's microns - scale by 1000 before rounding to Point2LL
    // (matching VbctAdapter.cpp's own toVbctMm/fromVbctMm convention), or every position within the
    // same whole millimeter collapses to one integer point and reads as a false duplicate.
    std::set<std::pair<Point2LL, Point2LL>> distinct_pairs;
    for (const vbct::Stringer& stringer : cross.domains[0].stringers)
    {
        ASSERT_FALSE(stringer.pieces.empty());
        const vbct::Point2& a = stringer.pieces.front().first;
        const vbct::Point2& b = stringer.pieces.front().second;
        distinct_pairs.insert({ Point2LL(static_cast<coord_t>(std::llround(a.x * 1000.0)), static_cast<coord_t>(std::llround(a.y * 1000.0))),
                                 Point2LL(static_cast<coord_t>(std::llround(b.x * 1000.0)), static_cast<coord_t>(std::llround(b.y * 1000.0))) });
    }
    EXPECT_EQ(distinct_pairs.size(), cross.domains[0].stringers.size());
}

/*!
 * Chain-specific Crosshatch's own explicit, user-required property: the crosshatch family's own
 * samples stay evenly distributed across the bar's own length at every phase, not compressed near
 * either real end. Under the current wraparound construction (spec REV 2.4's "fifth round" - see
 * stage10.cpp's own top-of-file history note) this holds by construction, even more precisely than
 * an earlier bounce-based design's own tighter tolerance suggests: n points evenly spaced by
 * 1/denominator around a mod-1 rotation stay evenly spaced by exactly that same amount when sorted
 * linearly, seam included, since a pure rotation can't change the spacing between points. Checked
 * directly and numerically: domain_stringers pushes exactly [base_i, crosshatch_i] per index i for
 * a Chain domain (see its own per-index loop body), so every odd-indexed Stringer (not output
 * line - see runStage10Directly's own doc comment for why that distinction matters) is a
 * crosshatch stringer. This test reads each one's own outer-wall (long top edge, y=10000)
 * attachment X coordinate directly as its own t position along the bar's length (a bar's own
 * "left"/"right" walls are each a single straight 2-point segment, so X is an exact linear
 * function of t - no reprojection needed), sorts them, and requires every consecutive gap to stay
 * close to every other, at two different phase values.
 */
TEST(VbctAdapterTest, ChainCrosshatchSamplesStayEvenlySpaced)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar - a Chain domain.

    for (const double phase_offset : { 0.25, 0.65 })
    {
        const vbct::Stage10Result cross = runStage10Directly(bar_shape, 0.3, 0.8, 2.0, phase_offset, /*crosshatch_enabled=*/true);
        ASSERT_EQ(cross.domains.size(), 1u) << "phase_offset=" << phase_offset;
        const std::vector<vbct::Stringer>& stringers = cross.domains[0].stringers;
        ASSERT_GE(stringers.size(), 6u) << "phase_offset=" << phase_offset;
        ASSERT_EQ(stringers.size() % 2, 0u) << "phase_offset=" << phase_offset;

        // The last two entries are the always-added true t=0/t=1 cap anchors (spec REV 2.5) -
        // appended after the interleaved [base_i, crosshatch_i] loop, not part of it, so they're
        // excluded here rather than mistaken for one more (oddly landed) crosshatch sample.
        const size_t interleaved_count = stringers.size() - 2;
        std::vector<double> xs;
        for (size_t k = 1; k < interleaved_count; k += 2)
        {
            const vbct::Stringer& stringer = stringers[k];
            ASSERT_FALSE(stringer.pieces.empty()) << "phase_offset=" << phase_offset;
            const vbct::Point2& a = stringer.pieces.front().first;
            const vbct::Point2& b = stringer.pieces.front().second;
            const double outer_x = (std::abs(a.y - 10.0) < std::abs(b.y - 10.0)) ? a.x : b.x;
            xs.push_back(outer_x);
        }
        std::sort(xs.begin(), xs.end());
        ASSERT_GE(xs.size(), 3u) << "phase_offset=" << phase_offset;

        double min_gap = std::numeric_limits<double>::max();
        double max_gap = 0.0;
        for (size_t i = 0; i + 1 < xs.size(); ++i)
        {
            const double gap = xs[i + 1] - xs[i];
            min_gap = std::min(min_gap, gap);
            max_gap = std::max(max_gap, gap);
        }
        EXPECT_GT(min_gap, 0.0) << "phase_offset=" << phase_offset;
        EXPECT_LT(max_gap, min_gap * 1.5)
            << "phase_offset=" << phase_offset << ": crosshatch samples should stay evenly spaced across the bar's own length, not compress near a reflection.";
    }
}

/*!
 * A nonzero crossover_pitch_mm should rotate a Ring domain's stringer family start point as z changes -
 * different z, same shape, same other tunables, should produce a genuinely different set of
 * stringer endpoints (not just a coincidentally-reordered same set), confirming the Z-layer
 * shift actually reaches VBCT's sampling instead of being silently dropped somewhere in the
 * adapter/pipeline plumbing.
 */
TEST(VbctAdapterTest, PhaseRateShiftsStringersAcrossZ)
{
    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 64)); // 30mm outer radius.
    ring_shape.push_back(makeCirclePolygon(0, 0, 15000, 48)); // 15mm inner radius (a hole).

    const std::optional<OpenLinesSet> lines_z0 = VbctAdapter::corrugate(ring_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000, /*z=*/0, /*crossover_pitch_mm=*/5.0);
    const std::optional<OpenLinesSet> lines_z5mm = VbctAdapter::corrugate(ring_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000, /*z=*/5000, /*crossover_pitch_mm=*/5.0);

    ASSERT_TRUE(lines_z0.has_value());
    ASSERT_TRUE(lines_z5mm.has_value());
    ASSERT_FALSE(lines_z0->empty());
    ASSERT_EQ(lines_z0->size(), lines_z5mm->size()); // Same shape/spacing -> same stringer count, just rotated.

    // At least one corresponding stringer's first endpoint should have moved - a crossover_pitch_mm
    // of 5mm over a 5mm Z step is a raw phase_offset step of 0.5 (half a crossover period), which
    // cannot land back on the exact same sample points for a well-populated ring.
    bool any_moved = false;
    for (size_t i = 0; i < lines_z0->size(); ++i)
    {
        if ((*lines_z0)[i].front() != (*lines_z5mm)[i].front())
        {
            any_moved = true;
            break;
        }
    }
    EXPECT_TRUE(any_moved) << "Changing Z with a nonzero crossover_pitch_mm should shift the stringer family's sampling.";
}

/*!
 * crossover_pitch_mm=0 (the default) must reproduce VBCT's own fixed t=0 behavior regardless of Z -
 * guards against the Z-layer shift accidentally becoming mandatory/always-on.
 */
TEST(VbctAdapterTest, ZeroPhaseRateIsZInvariant)
{
    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 64));
    ring_shape.push_back(makeCirclePolygon(0, 0, 15000, 48));

    const std::optional<OpenLinesSet> lines_z0 = VbctAdapter::corrugate(ring_shape, 0.3, 800, 2000, /*z=*/0, /*crossover_pitch_mm=*/0.0);
    const std::optional<OpenLinesSet> lines_z9mm = VbctAdapter::corrugate(ring_shape, 0.3, 800, 2000, /*z=*/9000, /*crossover_pitch_mm=*/0.0);

    ASSERT_TRUE(lines_z0.has_value());
    ASSERT_TRUE(lines_z9mm.has_value());
    ASSERT_EQ(lines_z0->size(), lines_z9mm->size());
    for (size_t i = 0; i < lines_z0->size(); ++i)
    {
        EXPECT_EQ((*lines_z0)[i].front(), (*lines_z9mm)[i].front());
    }
}

/*!
 * An empty Shape has nothing to corrugate.
 */
TEST(VbctAdapterTest, EmptyShapeReturnsNullopt)
{
    Shape empty_shape;
    EXPECT_FALSE(VbctAdapter::corrugate(empty_shape, 0.3, 800, 2000).has_value());
}

/*!
 * Regression test for a real production bug: "Part Studio 2 - Part 1 (2).stl"
 * has two small rectangular tabs on its inner wall. The raw model
 * cross-section corrugates cleanly, but Cura's own wall generation (inward
 * offset by wall_line_count walls) inserts corner-mitering points a
 * hair's-width apart at each tab corner -- confirmed directly: two points
 * differing by exactly one unit of VBCT's SCALE=1000 quantization (0.001mm).
 * That degenerate near-duplicate pair produced a sliver neighbor triangle
 * Stage 5's Correction 2.5 (bridging-triangle re-routing) correctly refused
 * to fold into (its convexity guard has no real corner there to
 * re-triangulate), leaving 8 spurious hubs -- 8 tiny independent Chain
 * domains at default threshold, and total Ring breakdown into 8 large
 * disconnected fragments once threshold exceeded their length. Fixed at the
 * actual source (collapse_near_duplicate_points, a Stage-1-adjacent input
 * hygiene pass in contour.cpp/pipeline.cpp) rather than by teaching Stage 5
 * to re-triangulate degenerate slivers.
 */
TEST(VbctAdapterTest, RealTabGeometryProducesCleanSingleRing)
{
    auto build_shape = [](const coord_t inset) {
        Shape shape;
        Polygon outer;
        for (int i = 0; i < real_tab_geometry::contour1_n; ++i)
        {
            const auto& p = real_tab_geometry::contour1[i];
            outer.push_back(Point2LL(static_cast<coord_t>(std::llround(p[0] * 1000.0)), static_cast<coord_t>(std::llround(p[1] * 1000.0))));
        }
        Polygon inner;
        for (int i = 0; i < real_tab_geometry::contour0_n; ++i)
        {
            const auto& p = real_tab_geometry::contour0[i];
            inner.push_back(Point2LL(static_cast<coord_t>(std::llround(p[0] * 1000.0)), static_cast<coord_t>(std::llround(p[1] * 1000.0))));
        }
        shape.push_back(outer);
        shape.push_back(inner);
        if (inset != 0)
        {
            shape = shape.offset(-inset);
        }
        return shape;
    };

    auto domainComposition = [](const Shape& shape, const double threshold_mm) {
        const std::vector<vbct::Contour> contours = VbctAdapter::shapeToContours(shape);
        vbct::VbctResult r14 = vbct::run_vbct(contours, 0.3);
        vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
        vbct::Stage6Result r6 = vbct::run_stage6(r5);
        vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
        vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
        vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, threshold_mm);
        int n_ring = 0, n_chain = 0;
        for (const auto& dom : r9.domains)
        {
            if (dom.kind == "ring") ++n_ring;
            else if (dom.kind == "chain") ++n_chain;
        }
        return std::pair{ n_ring, n_chain };
    };

    const Shape raw_shape = build_shape(0);
    const Shape offset_shape = build_shape(800); // ~0.8mm inset, 2x 0.4mm walls -- the real infill_area VBCT actually receives.

    for (const double threshold_mm : { 0.8, 5.0 })
    {
        const auto [raw_ring, raw_chain] = domainComposition(raw_shape, threshold_mm);
        EXPECT_EQ(raw_ring, 1) << "raw model, threshold=" << threshold_mm;
        EXPECT_EQ(raw_chain, 0) << "raw model, threshold=" << threshold_mm;

        const auto [offset_ring, offset_chain] = domainComposition(offset_shape, threshold_mm);
        EXPECT_EQ(offset_ring, 1) << "wall-offset inset, threshold=" << threshold_mm;
        EXPECT_EQ(offset_chain, 0) << "wall-offset inset, threshold=" << threshold_mm;
    }
}

/*!
 * Regression test for a real production report: "Part Studio 2 - Part 1 (2).stl" -- a
 * Z-invariant (straight-extruded) part -- was decomposing to Chain domains instead of a single
 * Ring at layers above ~10, even though the cross-section doesn't change with Z. Root cause was
 * the same corner-mitering near-duplicate-point issue fixed for a fixed Z height in
 * RealTabGeometryProducesCleanSingleRing above (see collapse_near_duplicate_points): Cura's own
 * wall-offset (ClipperLib, via Shape::offset -- the same call this test exercises) inserts a
 * near-duplicate point pair at each tab corner, and the raw slice's own per-Z floating-point
 * differences apparently pushed that pair's separation back and forth across whatever
 * (previously nonexistent) threshold decided if it got handled. Runs the full pipeline (through
 * Stage 10) at 9 real Z heights spanning the part, crossed with 6 wall-inset amounts (0 to 2mm,
 * covering plausible real bead-width strategies, not just the ~0.8mm default), and requires every
 * one of the 54 combinations to still produce exactly one Ring and zero Chain/Glob domains.
 */
TEST(VbctAdapterTest, PartStudio2ProducesCleanRingAcrossAllLayersAndInsets)
{
    struct Height
    {
        const char* name;
        const double (*outer)[2];
        int outer_n;
        const double (*inner)[2];
        int inner_n;
    };
    const Height heights[] = {
        { "z=2.500", real_tab_geometry_multiz::z_2p500_outer, real_tab_geometry_multiz::z_2p500_outer_n, real_tab_geometry_multiz::z_2p500_inner, real_tab_geometry_multiz::z_2p500_inner_n },
        { "z=5.000", real_tab_geometry_multiz::z_5p000_outer, real_tab_geometry_multiz::z_5p000_outer_n, real_tab_geometry_multiz::z_5p000_inner, real_tab_geometry_multiz::z_5p000_inner_n },
        { "z=7.500", real_tab_geometry_multiz::z_7p500_outer, real_tab_geometry_multiz::z_7p500_outer_n, real_tab_geometry_multiz::z_7p500_inner, real_tab_geometry_multiz::z_7p500_inner_n },
        { "z=10.000", real_tab_geometry_multiz::z_10p000_outer, real_tab_geometry_multiz::z_10p000_outer_n, real_tab_geometry_multiz::z_10p000_inner, real_tab_geometry_multiz::z_10p000_inner_n },
        { "z=12.500", real_tab_geometry_multiz::z_12p500_outer, real_tab_geometry_multiz::z_12p500_outer_n, real_tab_geometry_multiz::z_12p500_inner, real_tab_geometry_multiz::z_12p500_inner_n },
        { "z=15.000", real_tab_geometry_multiz::z_15p000_outer, real_tab_geometry_multiz::z_15p000_outer_n, real_tab_geometry_multiz::z_15p000_inner, real_tab_geometry_multiz::z_15p000_inner_n },
        { "z=17.500", real_tab_geometry_multiz::z_17p500_outer, real_tab_geometry_multiz::z_17p500_outer_n, real_tab_geometry_multiz::z_17p500_inner, real_tab_geometry_multiz::z_17p500_inner_n },
        { "z=20.000", real_tab_geometry_multiz::z_20p000_outer, real_tab_geometry_multiz::z_20p000_outer_n, real_tab_geometry_multiz::z_20p000_inner, real_tab_geometry_multiz::z_20p000_inner_n },
        { "z=22.500", real_tab_geometry_multiz::z_22p500_outer, real_tab_geometry_multiz::z_22p500_outer_n, real_tab_geometry_multiz::z_22p500_inner, real_tab_geometry_multiz::z_22p500_inner_n },
    };

    for (const Height& h : heights)
    {
        Shape shape;
        Polygon outer;
        for (int i = 0; i < h.outer_n; ++i)
        {
            outer.push_back(Point2LL(static_cast<coord_t>(std::llround(h.outer[i][0] * 1000.0)), static_cast<coord_t>(std::llround(h.outer[i][1] * 1000.0))));
        }
        Polygon inner;
        for (int i = 0; i < h.inner_n; ++i)
        {
            inner.push_back(Point2LL(static_cast<coord_t>(std::llround(h.inner[i][0] * 1000.0)), static_cast<coord_t>(std::llround(h.inner[i][1] * 1000.0))));
        }
        shape.push_back(outer);
        shape.push_back(inner);
        // Sweep the wall inset - the real infill_area's inset depends on wall_line_count and
        // varies with local bead-width strategy, not just a fixed 2x0.4mm - to see whether some
        // inset amount (not just the ~0.8mm default) changes domain composition.
        for (const coord_t inset : { coord_t{ 0 }, coord_t{ 400 }, coord_t{ 800 }, coord_t{ 1200 }, coord_t{ 1600 }, coord_t{ 2000 } })
        {
        const Shape offset_shape = inset == 0 ? shape : shape.offset(-inset);

        const std::vector<vbct::Contour> contours = VbctAdapter::shapeToContours(offset_shape);
        vbct::VbctResult r14 = vbct::run_vbct(contours, 0.3);
        vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
        vbct::Stage6Result r6 = vbct::run_stage6(r5);
        vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
        vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
        vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, /*threshold_mm=*/5.0);

        int n_ring = 0, n_chain = 0, n_glob = 0;
        for (const auto& dom : r9.domains)
        {
            if (dom.kind == "ring") ++n_ring;
            else if (dom.kind == "chain") ++n_chain;
            else if (dom.kind == "glob") ++n_glob;
        }

        EXPECT_EQ(n_ring, 1) << h.name << " inset=" << inset << "um";
        EXPECT_EQ(n_chain, 0) << h.name << " inset=" << inset << "um";
        EXPECT_EQ(n_glob, 0) << h.name << " inset=" << inset << "um";
        }
    }
}

/*!
 * Regression test for VbctAdapter::insetOutline (the "Corrugated Raw Outline Mode" setting):
 * confirms that corrugating a fixed-width inset of the raw outline -- computed independently of
 * Cura's own Arachne wall generation -- still produces a clean single Ring, across the same real
 * multi-Z geometry (real_tab_geometry_multiz.h) used by
 * PartStudio2ProducesCleanRingAcrossAllLayersAndInsets above. This is the direct check that
 * switching VBCT's input source away from the wall-generated Infill Area doesn't reintroduce what
 * Corrections 2.5/2.6 (VBCT Cpp's stage5.cpp) fixed for the Infill-Area path -- insetOutline's own
 * offset is a different code path (WallsComputation::generateSpiralInsets' pattern, not
 * WallToolPaths/Arachne), so this isn't otherwise implied by those tests passing.
 */
TEST(VbctAdapterTest, InsetOutlineProducesCleanRingAcrossAllLayers)
{
    // Settings::get<coord_t> always interprets the stored string as millimetres and converts to
    // microns itself (src/settings/Settings.cpp: MM2INT(get<double>(key))) -- these are mm, not
    // microns, regardless of what insetOutline's own coord_t-typed reads suggest.
    Settings settings;
    settings.add("wall_line_count", "2");
    settings.add("wall_line_width_0", "0.4");
    settings.add("wall_line_width_x", "0.4");
    settings.add("wall_0_inset", "0");

    struct Height
    {
        const char* name;
        const double (*outer)[2];
        int outer_n;
        const double (*inner)[2];
        int inner_n;
    };
    const Height heights[] = {
        { "z=2.500", real_tab_geometry_multiz::z_2p500_outer, real_tab_geometry_multiz::z_2p500_outer_n, real_tab_geometry_multiz::z_2p500_inner, real_tab_geometry_multiz::z_2p500_inner_n },
        { "z=5.000", real_tab_geometry_multiz::z_5p000_outer, real_tab_geometry_multiz::z_5p000_outer_n, real_tab_geometry_multiz::z_5p000_inner, real_tab_geometry_multiz::z_5p000_inner_n },
        { "z=7.500", real_tab_geometry_multiz::z_7p500_outer, real_tab_geometry_multiz::z_7p500_outer_n, real_tab_geometry_multiz::z_7p500_inner, real_tab_geometry_multiz::z_7p500_inner_n },
        { "z=10.000", real_tab_geometry_multiz::z_10p000_outer, real_tab_geometry_multiz::z_10p000_outer_n, real_tab_geometry_multiz::z_10p000_inner, real_tab_geometry_multiz::z_10p000_inner_n },
        { "z=12.500", real_tab_geometry_multiz::z_12p500_outer, real_tab_geometry_multiz::z_12p500_outer_n, real_tab_geometry_multiz::z_12p500_inner, real_tab_geometry_multiz::z_12p500_inner_n },
        { "z=15.000", real_tab_geometry_multiz::z_15p000_outer, real_tab_geometry_multiz::z_15p000_outer_n, real_tab_geometry_multiz::z_15p000_inner, real_tab_geometry_multiz::z_15p000_inner_n },
        { "z=17.500", real_tab_geometry_multiz::z_17p500_outer, real_tab_geometry_multiz::z_17p500_outer_n, real_tab_geometry_multiz::z_17p500_inner, real_tab_geometry_multiz::z_17p500_inner_n },
        { "z=20.000", real_tab_geometry_multiz::z_20p000_outer, real_tab_geometry_multiz::z_20p000_outer_n, real_tab_geometry_multiz::z_20p000_inner, real_tab_geometry_multiz::z_20p000_inner_n },
        { "z=22.500", real_tab_geometry_multiz::z_22p500_outer, real_tab_geometry_multiz::z_22p500_outer_n, real_tab_geometry_multiz::z_22p500_inner, real_tab_geometry_multiz::z_22p500_inner_n },
    };

    for (const Height& h : heights)
    {
        SliceLayerPart part;
        Polygon outer;
        for (int i = 0; i < h.outer_n; ++i)
        {
            outer.push_back(Point2LL(static_cast<coord_t>(std::llround(h.outer[i][0] * 1000.0)), static_cast<coord_t>(std::llround(h.outer[i][1] * 1000.0))));
        }
        Polygon inner;
        for (int i = 0; i < h.inner_n; ++i)
        {
            inner.push_back(Point2LL(static_cast<coord_t>(std::llround(h.inner[i][0] * 1000.0)), static_cast<coord_t>(std::llround(h.inner[i][1] * 1000.0))));
        }
        part.outline.push_back(outer);
        part.outline.push_back(inner);

        const Shape inset_shape = VbctAdapter::insetOutline(part, settings);
        const std::vector<vbct::Contour> contours = VbctAdapter::shapeToContours(inset_shape);
        vbct::VbctResult r14 = vbct::run_vbct(contours, 0.3);
        vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
        vbct::Stage6Result r6 = vbct::run_stage6(r5);
        vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
        vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
        vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, /*threshold_mm=*/0.8);

        int n_ring = 0, n_chain = 0, n_glob = 0;
        for (const auto& dom : r9.domains)
        {
            if (dom.kind == "ring") ++n_ring;
            else if (dom.kind == "chain") ++n_chain;
            else ++n_glob;
        }
        EXPECT_EQ(n_ring, 1) << h.name;
        EXPECT_EQ(n_chain, 0) << h.name;
        EXPECT_EQ(n_glob, 0) << h.name;
    }
}

/*!
 * Regression test for a real, confirmed bug: insetOutline's own total_inset formula only added
 * the final "half a line width past the last wall's own centerline" term inside its
 * wall_count>1 branch, so at wall_line_count=1 the inset stopped *exactly at* the wall's own
 * centerline instead of past it - direct user report was corrugation printed "on top of the
 * wall" in raw outline mode at wall_line_count=1 (only actually visible once a separate,
 * unrelated fix removed corrugateLinkedSkin's own inboard offset, which had been coincidentally
 * compensating for this bug). Checks the inset distance directly at wall_line_count=1 against
 * wall_line_count=2 (already covered indirectly by InsetOutlineProducesCleanRingAcrossAllLayers
 * above, but not at this exact inset-distance level) using a plain circle, where the inset
 * result's own radius is easy to compute independently.
 */
TEST(VbctAdapterTest, InsetOutlineGoesPastTheWallAtEveryWallCount)
{
    SliceLayerPart part;
    part.outline.push_back(makeCirclePolygon(0, 0, 30000, 128)); // 30mm radius, no hole.

    auto radius_after_inset = [&](const char* wall_count, const char* line_width_0, const char* line_width_x) -> double
    {
        Settings settings;
        settings.add("wall_line_count", wall_count);
        settings.add("wall_line_width_0", line_width_0);
        settings.add("wall_line_width_x", line_width_x);
        settings.add("wall_0_inset", "0");
        const Shape inset_shape = VbctAdapter::insetOutline(part, settings);
        double max_r = 0.0;
        for (const Polygon& polygon : inset_shape)
        {
            for (const Point2LL& p : polygon)
            {
                max_r = std::max(max_r, std::hypot(static_cast<double>(p.X), static_cast<double>(p.Y)));
            }
        }
        return max_r;
    };

    // wall_line_count=1, 0.4mm line width: should reach a full line width in (400 microns),
    // i.e. past wall 0's own centerline (200 microns) by another half width - not stop *at* the
    // centerline (the bug's own symptom: only 200 microns of inset instead of 400).
    const double radius_one_wall = radius_after_inset("1", "0.4", "0.4");
    EXPECT_NEAR(radius_one_wall, 30000.0 - 400.0, 5.0) << "wall_line_count=1 should inset a full line width, past the wall's own centerline.";

    // wall_line_count=2, same line widths: centerline of wall 0 (200) + full width of wall 1
    // (400) + half of wall 1's own width (200) = 800 microns - the already-correct case, kept
    // here as a same-fixture cross-check that the wall_count=1 fix didn't disturb it.
    const double radius_two_walls = radius_after_inset("2", "0.4", "0.4");
    EXPECT_NEAR(radius_two_walls, 30000.0 - 800.0, 5.0) << "wall_line_count=2 should inset past the second wall's own centerline too.";
}

namespace
{

// Fills a SliceMeshStorage's settings with the minimal set computeAnchorsForMesh reads, mirroring
// InsetOutlineProducesCleanRingAcrossAllLayers's own settings.add pattern above.
void addCorrugationAnchorSettings(Settings& settings)
{
    settings.add("corrugated_vbs_tolerance", "0.3");
    settings.add("corrugated_prune_threshold", "0.8");
    settings.add("corrugated_raw_outline_mode", "False");
    settings.add("corrugated_chain_junction_merge_angle", "30");
    // Wall Strip (spec REV 2.6) - read unconditionally by buildCorrugationInput (VbctAdapter.cpp),
    // which computeAnchorsForMesh calls for every part/layer, so these must be registered here too
    // even though none of these tests actually exercise stripping itself.
    settings.add("corrugated_strip_wall_a", "False");
    settings.add("corrugated_strip_wall_b", "False");
    // De Minimis Hole Threshold (spec REV 3.0/3.5/5.9) - read unconditionally by
    // computeAnchorsForMesh's own hole-detection pre-pass for every layer, so this must be
    // registered here too even though none of these tests actually have any holes. "0" reproduces
    // today's exact pre-feature behavior (no hole's own area is ever negative enough to be below
    // this threshold, so nothing is ever suppressed).
    settings.add("corrugated_deminimis_hole_area", "0");
    // Transition Layer (spec REV 3.3/3.6/5.10) - read unconditionally by computeAnchorsForMesh's
    // own Trigger 2 (domain-count-change) post-pass, so this must be registered here too even
    // though most of these tests don't exercise the feature itself. "True" matches the setting's
    // own real default, so tests that don't care about Transition Layer at all still exercise the
    // post-pass's own no-op path on ordinary geometry (no domain-count change) rather than
    // skipping it entirely.
    settings.add("corrugated_transition_layer_enabled", "True");
    // Stringer-count stability (corrugation-start-jitter investigation) - read unconditionally by
    // computeAnchorsForMesh's own stringer-count pre-pass for every layer, so this must be
    // registered here too even though most of these tests don't care about the exact pitch.
    settings.add("corrugated_stringer_pitch", "3");
}

} // namespace

/*!
 * Regression test for cross-layer anchor continuity (spec REV 1.4 S:5.3): computeAnchorsForMesh's
 * *first* tracked layer in a mesh (nothing to continue from yet) must reuse today's existing
 * absolute rule -- vbct::polygon_centroid + vbct::reference_angle_arc_length on the Ring's own
 * canonical (longer) wall -- exactly, not some new independent rule of its own. Verified by
 * calling that same oracle directly and comparing, rather than hardcoding an expected number.
 */
TEST(VbctAdapterTest, ComputeAnchorsForMeshFirstLayerMatchesAbsoluteRuleExactly)
{
    Mesh cura_mesh;
    addCorrugationAnchorSettings(cura_mesh.settings_);
    SliceMeshStorage mesh(&cura_mesh, /*slice_layer_count=*/1);

    SliceLayerPart part;
    part.outline.push_back(makeCirclePolygon(0, 0, 30000, 96));
    part.outline.push_back(makeCirclePolygon(0, 0, 15000, 64));
    part.infill_area.push_back(makeCirclePolygon(0, 0, 30000, 96));
    part.infill_area.push_back(makeCirclePolygon(0, 0, 15000, 64));
    mesh.layers[0].parts.push_back(part);

    const std::vector<CorrugationAnchor> anchors = VbctAdapter::computeAnchorsForMesh(mesh);
    ASSERT_EQ(anchors.size(), 1u);
    ASSERT_TRUE(anchors[0].has_ring);

    // Independent oracle: run the exact same Stages 1-9 VbctAdapter::computeAnchorsForMesh itself
    // ran, find the Ring, canonicalize to the longer wall the same way domain_stringers/
    // computeAnchorsForMesh both do, and compute the absolute rule directly.
    const std::vector<vbct::Contour> contours = VbctAdapter::shapeToContours(part.infill_area);
    const vbct::VbctResult r14 = vbct::run_vbct(contours, 0.3);
    const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
    const vbct::Stage6Result r6 = vbct::run_stage6(r5);
    const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
    const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
    const vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, 0.8);
    const vbct::Domain* ring = nullptr;
    for (const auto& dom : r9.domains)
    {
        if (dom.kind == "ring")
        {
            ring = &dom;
            break;
        }
    }
    ASSERT_NE(ring, nullptr);
    std::vector<vbct::Point2> canonical = ring->left->points;
    if (vbct::wall_length(canonical) < vbct::wall_length(ring->right->points))
    {
        canonical = ring->right->points;
    }
    const vbct::Point2 centroid = vbct::polygon_centroid(canonical);
    const double expected_t0_frac = vbct::reference_angle_arc_length(canonical, centroid) / vbct::wall_length(canonical);

    EXPECT_NEAR(anchors[0].t0_frac, expected_t0_frac, 1e-9);
}

/*!
 * Regression test for cross-layer anchor continuity (spec REV 1.4 S:5.3): confirms
 * computeAnchorsForMesh tracks the *nearest point to the previous layer's own anchor*, not a
 * fresh independent absolute-rule pick, once a first anchor exists.
 *
 * Constructed so the two rules provably disagree, modeled on the real mechanism stage10.cpp's
 * own polygon_centroid documents (an arc-length-weighted centroid is pulled toward wherever the
 * wall has more boundary weight): both layers are the same base circle, each with one small
 * region pushed outward into a "bulge" that pulls the centroid noticeably off-origin - layer 0's
 * bulge sits opposite (~180 degrees from) where its own absolute-rule anchor consequently lands,
 * so that quiet region of the boundary is virtually unperturbed between layers. Layer 1's bulge
 * moves to a different angle, shifting the centroid (and therefore the *fresh* absolute rule's
 * +X-from-centroid crossing point) to a materially different location - while the quiet region
 * near layer 0's actual anchor stays almost exactly where it was, so nearest-point tracking
 * should still land there instead of jumping to wherever the fresh rule now points.
 */
TEST(VbctAdapterTest, ComputeAnchorsForMeshTracksNearestPointNotFreshAbsoluteRule)
{
    Mesh cura_mesh;
    addCorrugationAnchorSettings(cura_mesh.settings_);
    SliceMeshStorage mesh(&cura_mesh, /*slice_layer_count=*/2);

    // A circle with one arc segment (centered on bulge_center_deg, bulge_width_deg wide) pushed
    // radially outward by bulge_height - deliberately a large, obvious bulge (not subtle
    // tessellation-scale noise) since this test is about proving the mechanism, not reproducing
    // the exact real-world trigger. bulge_width_deg widened 150->200 (VBCT Cpp's Stage 5
    // Correction 0, sliver-triangle edge flip): that fix changes which diagonal a handful of
    // otherwise-unremarkable triangles in this large synthetic mesh get triangulated with,
    // shifting the arc-length-weighted centroid slightly and shrinking this test's own
    // discrimination margin below its tuned 0.03 threshold at the old width - re-tuned per this
    // test's own "fix the geometry, not the assertion" rule below, not by relaxing that threshold.
    auto makeBulgedCirclePolygon = [](double radius, size_t n, double bulge_center_deg, double bulge_width_deg, double bulge_height) {
        Polygon polygon;
        for (size_t i = 0; i < n; ++i)
        {
            const double angle_deg = 360.0 * static_cast<double>(i) / static_cast<double>(n);
            double delta = std::fmod(angle_deg - bulge_center_deg + 540.0, 360.0) - 180.0; // signed, in (-180,180]
            double r = radius;
            if (std::abs(delta) < bulge_width_deg / 2.0)
            {
                // Smooth (cosine) bump, zero at the bulge's own edges, full height at its center.
                r += bulge_height * std::cos(delta / (bulge_width_deg / 2.0) * (std::numbers::pi / 2.0));
            }
            const double angle_rad = angle_deg * std::numbers::pi / 180.0;
            polygon.push_back(Point2LL(static_cast<coord_t>(std::llround(r * std::cos(angle_rad))), static_cast<coord_t>(std::llround(r * std::sin(angle_rad)))));
        }
        return polygon;
    };

    SliceLayerPart part0;
    part0.outline.push_back(makeBulgedCirclePolygon(30000, 256, /*bulge_center_deg=*/180.0, /*bulge_width_deg=*/200.0, /*bulge_height=*/26000));
    part0.outline.push_back(makeCirclePolygon(0, 0, 5000, 48));
    part0.infill_area.push_back(makeBulgedCirclePolygon(30000, 256, 180.0, 200.0, 26000));
    part0.infill_area.push_back(makeCirclePolygon(0, 0, 5000, 48));
    mesh.layers[0].parts.push_back(part0);

    // Deliberately not at 0 degrees (roughly where layer 0's own quiet-side anchor should land,
    // opposite its 180-degree bulge) - placing layer 1's bulge there would perturb the very
    // region nearest-point tracking is expected to stay on, defeating the point of this test.
    SliceLayerPart part1;
    part1.outline.push_back(makeBulgedCirclePolygon(30000, 256, /*bulge_center_deg=*/90.0, /*bulge_width_deg=*/200.0, /*bulge_height=*/26000));
    part1.outline.push_back(makeCirclePolygon(0, 0, 5000, 48));
    part1.infill_area.push_back(makeBulgedCirclePolygon(30000, 256, 90.0, 200.0, 26000));
    part1.infill_area.push_back(makeCirclePolygon(0, 0, 5000, 48));
    mesh.layers[1].parts.push_back(part1);

    const std::vector<CorrugationAnchor> anchors = VbctAdapter::computeAnchorsForMesh(mesh);
    ASSERT_EQ(anchors.size(), 2u);
    ASSERT_TRUE(anchors[0].has_ring);
    ASSERT_TRUE(anchors[1].has_ring);

    // Oracle: layer 1's own fresh absolute-rule pick, independent of layer 0 entirely.
    const std::vector<vbct::Contour> contours1 = VbctAdapter::shapeToContours(part1.infill_area);
    const vbct::VbctResult r14 = vbct::run_vbct(contours1, 0.3);
    const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
    const vbct::Stage6Result r6 = vbct::run_stage6(r5);
    const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
    const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
    const vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, 0.8);
    const vbct::Domain* ring = nullptr;
    for (const auto& dom : r9.domains)
    {
        if (dom.kind == "ring")
        {
            ring = &dom;
            break;
        }
    }
    ASSERT_NE(ring, nullptr);
    std::vector<vbct::Point2> canonical = ring->left->points;
    if (vbct::wall_length(canonical) < vbct::wall_length(ring->right->points))
    {
        canonical = ring->right->points;
    }
    const vbct::Point2 centroid = vbct::polygon_centroid(canonical);
    const double fresh_absolute_t0_frac = vbct::reference_angle_arc_length(canonical, centroid) / vbct::wall_length(canonical);

    // The two rules must provably disagree here (by construction - a 90 degree rotation of an
    // eccentric ellipse moves the absolute crossing point a large distance), else this test
    // wouldn't actually be discriminating between them.
    const double frac_diff = std::abs(anchors[1].t0_frac - fresh_absolute_t0_frac);
    const double wrapped_diff = std::min(frac_diff, 1.0 - frac_diff);
    // 0.03 of a full wrap (~11 degrees on the wall's own arc length) is well beyond what
    // floating-point noise or coincidence could produce - an arc-length-weighted centroid genuine
    // shift from a large, deliberate bulge only moves the crossing point so far before further
    // bulge height gives diminishing returns (confirmed empirically while tuning this fixture).
    EXPECT_GT(wrapped_diff, 0.03) << "test construction didn't actually make the two rules disagree - fix the geometry, not the assertion";

    // The actual property under test: layer 1's tracked anchor point should be much closer to
    // layer 0's own anchor point than to where the fresh absolute rule would have landed.
    const Point2LL layer0_anchor = anchors[0].anchor_point;
    const Point2LL layer1_anchor = anchors[1].anchor_point;
    const double dist_to_prev = std::hypot(static_cast<double>(layer1_anchor.X - layer0_anchor.X), static_cast<double>(layer1_anchor.Y - layer0_anchor.Y));

    // Physical point the fresh absolute rule would have used, converted back to engine units for
    // an apples-to-apples distance comparison.
    double acc = 0.0;
    vbct::Point2 fresh_absolute_point = canonical.front();
    const double target_arc = fresh_absolute_t0_frac * vbct::wall_length(canonical);
    for (size_t i = 0; i + 1 < canonical.size(); ++i)
    {
        const double seg_len = std::hypot(canonical[i + 1].x - canonical[i].x, canonical[i + 1].y - canonical[i].y);
        if (acc + seg_len >= target_arc)
        {
            const double frac = (seg_len == 0.0) ? 0.0 : (target_arc - acc) / seg_len;
            fresh_absolute_point = { canonical[i].x + frac * (canonical[i + 1].x - canonical[i].x), canonical[i].y + frac * (canonical[i + 1].y - canonical[i].y) };
            break;
        }
        acc += seg_len;
    }
    const Point2LL fresh_absolute_point_engine(static_cast<coord_t>(std::llround(fresh_absolute_point.x * 1000.0)), static_cast<coord_t>(std::llround(fresh_absolute_point.y * 1000.0)));
    const double dist_to_fresh_absolute
        = std::hypot(static_cast<double>(layer1_anchor.X - fresh_absolute_point_engine.X), static_cast<double>(layer1_anchor.Y - fresh_absolute_point_engine.Y));

    EXPECT_LT(dist_to_prev, dist_to_fresh_absolute) << "layer 1's anchor should track layer 0's anchor, not recompute a fresh absolute pick";
}

/*!
 * Regression test for cross-layer anchor continuity (spec REV 1.4 S:5.3.5): a layer with no
 * corrugatable Ring (here, no parts at all) must reset continuity tracking rather than reaching
 * back past it - the next tracked layer after the gap should use today's absolute rule again,
 * exactly like the very first layer of a mesh, not try to track continuity across the gap.
 */
TEST(VbctAdapterTest, ComputeAnchorsForMeshResetsTrackingAcrossATopologyChange)
{
    Mesh cura_mesh;
    addCorrugationAnchorSettings(cura_mesh.settings_);
    SliceMeshStorage mesh(&cura_mesh, /*slice_layer_count=*/3);

    SliceLayerPart ring_part;
    ring_part.outline.push_back(makeCirclePolygon(0, 0, 30000, 96));
    ring_part.outline.push_back(makeCirclePolygon(0, 0, 15000, 64));
    ring_part.infill_area.push_back(makeCirclePolygon(0, 0, 30000, 96));
    ring_part.infill_area.push_back(makeCirclePolygon(0, 0, 15000, 64));

    mesh.layers[0].parts.push_back(ring_part);
    // layer 1 left with no parts at all - a real gap (e.g. this mesh doesn't exist at this Z).
    mesh.layers[2].parts.push_back(ring_part);

    const std::vector<CorrugationAnchor> anchors = VbctAdapter::computeAnchorsForMesh(mesh);
    ASSERT_EQ(anchors.size(), 3u);
    EXPECT_TRUE(anchors[0].has_ring);
    EXPECT_FALSE(anchors[1].has_ring);
    ASSERT_TRUE(anchors[2].has_ring);

    // Layer 2, right after the gap, should match the same absolute-rule oracle as layer 0 did
    // (identical geometry to layer 0's part) - proving it started fresh, not "tracked" from
    // layer 0 across the gap (which would happen to give the same answer here anyway since the
    // geometry is identical, but the *value* matching the oracle - not merely matching layer 0 -
    // is what actually confirms the reset path ran, as opposed to some latent stale state).
    const std::vector<vbct::Contour> contours = VbctAdapter::shapeToContours(ring_part.infill_area);
    const vbct::VbctResult r14 = vbct::run_vbct(contours, 0.3);
    const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
    const vbct::Stage6Result r6 = vbct::run_stage6(r5);
    const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
    const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
    const vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, 0.8);
    const vbct::Domain* ring = nullptr;
    for (const auto& dom : r9.domains)
    {
        if (dom.kind == "ring")
        {
            ring = &dom;
            break;
        }
    }
    ASSERT_NE(ring, nullptr);
    std::vector<vbct::Point2> canonical = ring->left->points;
    if (vbct::wall_length(canonical) < vbct::wall_length(ring->right->points))
    {
        canonical = ring->right->points;
    }
    const vbct::Point2 centroid = vbct::polygon_centroid(canonical);
    const double expected_t0_frac = vbct::reference_angle_arc_length(canonical, centroid) / vbct::wall_length(canonical);
    EXPECT_NEAR(anchors[2].t0_frac, expected_t0_frac, 1e-9);
}

/*!
 * Chain domain support (spec REV 2.1): computeAnchorsForMesh should track a Chain domain's own
 * "same real end" point across layers (has_chain=true, chain_anchor_point tracked), independently
 * of Ring tracking (has_ring stays false throughout, since this fixture's layers are all Chain
 * domains). Two Z-invariant layers of the same bar-shaped Chain should land on essentially the
 * same physical anchor point both times, not drift to an unrelated location - the direct analogue
 * of Ring's own ComputeAnchorsForMeshTracksNearestPointNotFreshAbsoluteRule test, but for a Chain's
 * simpler "which cap" tracking mechanism rather than a t0 rotation fraction.
 *
 * This fixture cannot force Stage 9's own far_cap() to actually flip cap_start/cap_end identity
 * between layers the way a real print's own instability (chain_anchor_point's own doc comment)
 * does - that would require deeper control over Stage 9's internal splice ordering than this test
 * has - so it only exercises the "stable geometry stays stable" path directly. The override
 * mechanism itself (that a tracked point correctly steers orient_toward toward a *specific* cap)
 * is proven separately and more directly by BuildDomainEventsChainAnchorPointOverridesOrientation
 * above, which controls the override explicitly rather than relying on this pass's own tracking
 * decision.
 */
TEST(VbctAdapterTest, ComputeAnchorsForMeshTracksChainAcrossLayers)
{
    Mesh cura_mesh;
    addCorrugationAnchorSettings(cura_mesh.settings_);
    SliceMeshStorage mesh(&cura_mesh, /*slice_layer_count=*/2);

    SliceLayerPart bar_part;
    bar_part.outline.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar - a Chain domain.
    bar_part.infill_area.push_back(makeRectPolygon(0, 0, 60000, 10000));
    mesh.layers[0].parts.push_back(bar_part);
    mesh.layers[1].parts.push_back(bar_part); // Z-invariant: identical geometry both layers.

    const std::vector<CorrugationAnchor> anchors = VbctAdapter::computeAnchorsForMesh(mesh);
    ASSERT_EQ(anchors.size(), 2u);
    ASSERT_TRUE(anchors[0].has_chain);
    ASSERT_TRUE(anchors[1].has_chain);
    EXPECT_FALSE(anchors[0].has_ring) << "A Chain-only layer should never also report has_ring.";
    EXPECT_FALSE(anchors[1].has_ring);

    const double dist = std::hypot(
        static_cast<double>(anchors[1].chain_anchor_point.X - anchors[0].chain_anchor_point.X),
        static_cast<double>(anchors[1].chain_anchor_point.Y - anchors[0].chain_anchor_point.Y));
    EXPECT_LT(dist, 100.0) << "Two Z-invariant layers of the same Chain-shaped bar should track to essentially the same physical anchor point, not drift.";
}

/*!
 * Chain domain *end-position* continuity (extends chain_anchor_point's own "which end" role with
 * "exactly where does each real end sit" - see CorrugationAnchor::chain_left_near_t_frac's own
 * doc comment for the full rationale this was added from, including real-print diagnostic
 * evidence on a Z-invariant airfoil that chain_anchor_point alone still left both wall ends free
 * to jitter layer to layer). chain_position_tracked should become true from the second tracked
 * layer onward (nothing to search against on the first), and on perfectly Z-invariant geometry
 * with no VBCT-level per-layer noise to correct for, the four near/far fractions should stay close
 * to the full wall span (near~0.0, far~1.0) - tracking shouldn't need to move them meaningfully
 * away from their own pre-tracking defaults when nothing is actually unstable.
 */
TEST(VbctAdapterTest, ComputeAnchorsForMeshTracksChainEndPositionsAcrossLayers)
{
    Mesh cura_mesh;
    addCorrugationAnchorSettings(cura_mesh.settings_);
    SliceMeshStorage mesh(&cura_mesh, /*slice_layer_count=*/3);

    SliceLayerPart bar_part;
    bar_part.outline.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar - a Chain domain.
    bar_part.infill_area.push_back(makeRectPolygon(0, 0, 60000, 10000));
    mesh.layers[0].parts.push_back(bar_part);
    mesh.layers[1].parts.push_back(bar_part);
    mesh.layers[2].parts.push_back(bar_part); // Z-invariant across all three layers.

    const std::vector<CorrugationAnchor> anchors = VbctAdapter::computeAnchorsForMesh(mesh);
    ASSERT_EQ(anchors.size(), 3u);
    EXPECT_FALSE(anchors[0].chain_position_tracked) << "The first tracked layer has nothing previous to search against.";
    ASSERT_TRUE(anchors[1].chain_position_tracked);
    ASSERT_TRUE(anchors[2].chain_position_tracked);

    for (size_t i = 1; i < anchors.size(); ++i)
    {
        EXPECT_NEAR(anchors[i].chain_left_near_t_frac, 0.0, 0.05) << "layer " << i;
        EXPECT_NEAR(anchors[i].chain_left_far_t_frac, 1.0, 0.05) << "layer " << i;
        EXPECT_NEAR(anchors[i].chain_right_near_t_frac, 0.0, 0.05) << "layer " << i;
        EXPECT_NEAR(anchors[i].chain_right_far_t_frac, 1.0, 0.05) << "layer " << i;
    }
}

/*!
 * Chain domain end-position continuity's own actual sampling effect, tested directly against
 * VbctAdapter::corrugate() rather than through computeAnchorsForMesh's own tracking decision (a
 * synthetic fixture can't force real per-layer VBCT noise the tracking pre-pass would otherwise
 * correct for - see ComputeAnchorsForMeshTracksChainAcrossLayers' own doc comment for the same
 * caveat). Confirms the near/far fraction overrides actually confine each wall's own sampled span
 * to an inner sub-range instead of running the wall's full raw t=0..1 extent - the direct
 * regression test for the interpolation formula itself
 * (prepare_domain_sampling/domain_stringers' own Chain branch, stage10.cpp).
 */
TEST(VbctAdapterTest, CorrugateChainNearFarFractionsConfineSampledWallSpan)
{
    Shape bar_shape;
    bar_shape.push_back(makeRectPolygon(0, 0, 60000, 10000)); // 60mm x 10mm bar - a Chain domain.

    const std::optional<OpenLinesSet> full_lines = VbctAdapter::corrugate(bar_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000);
    ASSERT_TRUE(full_lines.has_value());
    ASSERT_FALSE(full_lines->empty());

    const std::optional<OpenLinesSet> confined_lines = VbctAdapter::corrugate(
        bar_shape,
        /*x=*/0.3,
        /*threshold=*/800,
        /*spacing=*/2000,
        /*z=*/0,
        /*crossover_pitch_mm=*/0.0,
        /*anchor_t0_frac=*/std::nullopt,
        /*other_wall_t0_frac=*/std::nullopt,
        /*reverse_canonical_wall=*/std::nullopt,
        /*crosshatch_enabled=*/false,
        // A near/far override is only applied to the same domain chain_anchor_point itself picks
        // (run_stage10/build_domain_events's own apply_chain_anchor_here gate, stage10.cpp) - so a
        // chain_anchor_point must be supplied for the near/far overrides below to take effect at
        // all. Pointing it at the bar's own near cap (x=0, mid-height) reproduces the same
        // orientation the no-override default above already picked on its own.
        /*chain_anchor_point=*/Point2LL(0, 5000),
        /*chain_left_near_t_frac=*/0.2,
        /*chain_left_far_t_frac=*/0.8,
        /*chain_right_near_t_frac=*/0.2,
        /*chain_right_far_t_frac=*/0.8);
    ASSERT_TRUE(confined_lines.has_value());
    ASSERT_FALSE(confined_lines->empty());

    coord_t full_min_x = std::numeric_limits<coord_t>::max();
    coord_t full_max_x = std::numeric_limits<coord_t>::min();
    for (const OpenPolyline& line : *full_lines)
    {
        for (const Point2LL& p : line)
        {
            full_min_x = std::min(full_min_x, p.X);
            full_max_x = std::max(full_max_x, p.X);
        }
    }

    coord_t confined_min_x = std::numeric_limits<coord_t>::max();
    coord_t confined_max_x = std::numeric_limits<coord_t>::min();
    for (const OpenPolyline& line : *confined_lines)
    {
        for (const Point2LL& p : line)
        {
            confined_min_x = std::min(confined_min_x, p.X);
            confined_max_x = std::max(confined_max_x, p.X);
        }
    }

    // Default (no override): samples reach essentially all the way to both raw caps (x=0, x=60000).
    EXPECT_LT(full_min_x, 2000) << "Default sampling should reach essentially to the near cap (x=0).";
    EXPECT_GT(full_max_x, 58000) << "Default sampling should reach essentially to the far cap (x=60000).";

    // Confined (near=0.2, far=0.8): samples should stay well clear of both raw caps, roughly within
    // [0.2, 0.8] of the 60mm span - i.e. within [12000, 48000] give or take.
    EXPECT_GT(confined_min_x, 8000) << "With near=0.2, sampling should stay clear of the raw near cap.";
    EXPECT_LT(confined_max_x, 52000) << "With far=0.8, sampling should stay clear of the raw far cap.";
}

/*!
 * Chain junction (Hub) support (spec REV 2.1): a Tee-shaped layer (crossbar merges into one Chain,
 * stem stays separate - see LinkedSkinMergesStraightPassThroughChainPairAtJunction) should still
 * get continuity tracking, not reset to has_chain=false just because there's more than one Chain
 * domain now. computeAnchorsForMesh tracks whichever domain has the greatest combined wall length
 * (the merged crossbar, ~120mm, versus the stem's own ~40mm) - a deliberately scoped residual (see
 * this function's own header doc), not full per-domain tracking.
 */
TEST(VbctAdapterTest, ComputeAnchorsForMeshTracksLongestChainAtATeeJunction)
{
    Mesh cura_mesh;
    addCorrugationAnchorSettings(cura_mesh.settings_);
    SliceMeshStorage mesh(&cura_mesh, /*slice_layer_count=*/2);

    SliceLayerPart tee_part;
    tee_part.outline.push_back(makeTeePolygon());
    tee_part.infill_area.push_back(makeTeePolygon());
    mesh.layers[0].parts.push_back(tee_part);
    mesh.layers[1].parts.push_back(tee_part); // Z-invariant.

    const std::vector<CorrugationAnchor> anchors = VbctAdapter::computeAnchorsForMesh(mesh);
    ASSERT_EQ(anchors.size(), 2u);
    ASSERT_TRUE(anchors[0].has_chain) << "A Tee's own multi-Chain composition should still be tracked, not reset.";
    ASSERT_TRUE(anchors[1].has_chain);
    EXPECT_FALSE(anchors[0].has_ring);

    const double dist = std::hypot(
        static_cast<double>(anchors[1].chain_anchor_point.X - anchors[0].chain_anchor_point.X),
        static_cast<double>(anchors[1].chain_anchor_point.Y - anchors[0].chain_anchor_point.Y));
    EXPECT_LT(dist, 100.0) << "Two Z-invariant layers of the same Tee should track to essentially the same physical anchor point.";
}

/*!
 * Wall Strip (spec REV 2.6/5.7): multiple simultaneously-stripped Chain domains. A Tee's own
 * crossbar-plus-stem composition (after mergeStraightPassThroughChains folds the crossbar's own two
 * halves into one) has *two* Chain domains at every layer - unlike has_chain/chain_anchor_point
 * (scoped to the single longest-combined-wall domain, tested above),
 * chain_domain_wall_identities should carry one entry per domain, and each domain's own
 * wall_a_is_left should stay stable (not flip) across Z-invariant layers, the same way the single
 * tracked domain's own anchor point already does.
 */
TEST(VbctAdapterTest, ComputeAnchorsForMeshTracksWallIdentityForEveryChainDomainAtATeeJunction)
{
    Mesh cura_mesh;
    addCorrugationAnchorSettings(cura_mesh.settings_);
    SliceMeshStorage mesh(&cura_mesh, /*slice_layer_count=*/3);

    SliceLayerPart tee_part;
    tee_part.outline.push_back(makeTeePolygon());
    tee_part.infill_area.push_back(makeTeePolygon());
    mesh.layers[0].parts.push_back(tee_part);
    mesh.layers[1].parts.push_back(tee_part); // Z-invariant.
    mesh.layers[2].parts.push_back(tee_part); // Z-invariant.

    const std::vector<CorrugationAnchor> anchors = VbctAdapter::computeAnchorsForMesh(mesh);
    ASSERT_EQ(anchors.size(), 3u);
    for (size_t layer = 0; layer < 3; ++layer)
    {
        ASSERT_EQ(anchors[layer].chain_domain_wall_identities.size(), 2u) << "layer " << layer << ": a Tee's own crossbar-plus-stem should track two Chain domains, not just the single longest one.";
    }

    // Match each of layer 1 and layer 2's own domains back to layer 0's own, by nearest
    // identity_point (mirroring how VbctAdapter itself matches across layers), and confirm both
    // the identity_point itself and wall_a_is_left stay stable - not flipping - across these
    // Z-invariant layers.
    for (size_t layer = 1; layer < 3; ++layer)
    {
        for (const ChainDomainWallIdentity& current : anchors[layer].chain_domain_wall_identities)
        {
            const ChainDomainWallIdentity* matched = nullptr;
            double best_dist_sq = std::numeric_limits<double>::max();
            for (const ChainDomainWallIdentity& prev : anchors[0].chain_domain_wall_identities)
            {
                const double dx = static_cast<double>(current.identity_point.X - prev.identity_point.X);
                const double dy = static_cast<double>(current.identity_point.Y - prev.identity_point.Y);
                const double dist_sq = dx * dx + dy * dy;
                if (dist_sq < best_dist_sq)
                {
                    best_dist_sq = dist_sq;
                    matched = &prev;
                }
            }
            ASSERT_NE(matched, nullptr);
            EXPECT_LT(std::sqrt(best_dist_sq), 100.0) << "layer " << layer << ": each domain's own identity_point should track to essentially the same physical location across Z-invariant layers.";
            EXPECT_EQ(current.wall_a_is_left, matched->wall_a_is_left) << "layer " << layer << ": a domain's own Wall A/B identity should not flip across Z-invariant layers.";
        }
    }
}

/*!
 * Investigation for VBCT-Stage8-Wall-Stitch-Nondeterminism-Brief.md: two fresh CuraEngine.exe
 * processes, byte-identical settings and input, produced a Ring wall that collapsed to 2
 * near-coincident points on different (and different-numbered) Z heights each run. The brief
 * hypothesizes iteration-order non-determinism in Stage 8's wall-stitch walk
 * (src/vbct/stage8.cpp's unordered_map/unordered_set-keyed `adj`/`used`).
 *
 * Direct code reading found no place in the walk that iterates an unordered container itself
 * (only indexed/counted lookups -- `adj[cur]`, `used.count(idx)` -- and the one direct iteration,
 * over `insertion_order`, is a plain std::vector, not the unordered_map). This test instead
 * checks the more basic property the brief's own suggested repro targets: running the identical
 * pipeline call on the identical real input, in the same process, many times in a row, should
 * always produce the identical wall. If it does (no collapse ever observed here), that rules out
 * VBCT's own single-threaded pipeline logic as the source and points at something in the
 * surrounding CuraEngine integration instead (e.g. state shared across layers on a reused worker
 * thread in the real multi-layer parallel processing this test doesn't exercise).
 */
TEST(VbctAdapterTest, StitchIsDeterministicAcrossRepeatedCalls)
{
    struct Height
    {
        const char* name;
        const double (*outer)[2];
        int outer_n;
        const double (*inner)[2];
        int inner_n;
    };
    const Height heights[] = {
        { "z=2.500", real_tab_geometry_multiz::z_2p500_outer, real_tab_geometry_multiz::z_2p500_outer_n, real_tab_geometry_multiz::z_2p500_inner, real_tab_geometry_multiz::z_2p500_inner_n },
        { "z=5.000", real_tab_geometry_multiz::z_5p000_outer, real_tab_geometry_multiz::z_5p000_outer_n, real_tab_geometry_multiz::z_5p000_inner, real_tab_geometry_multiz::z_5p000_inner_n },
        { "z=10.000", real_tab_geometry_multiz::z_10p000_outer, real_tab_geometry_multiz::z_10p000_outer_n, real_tab_geometry_multiz::z_10p000_inner, real_tab_geometry_multiz::z_10p000_inner_n },
        { "z=17.500", real_tab_geometry_multiz::z_17p500_outer, real_tab_geometry_multiz::z_17p500_outer_n, real_tab_geometry_multiz::z_17p500_inner, real_tab_geometry_multiz::z_17p500_inner_n },
        { "z=22.500", real_tab_geometry_multiz::z_22p500_outer, real_tab_geometry_multiz::z_22p500_outer_n, real_tab_geometry_multiz::z_22p500_inner, real_tab_geometry_multiz::z_22p500_inner_n },
    };
    constexpr int kRepeats = 50; // 5 heights x 50 reps confirmed 0/1500 mismatches in the investigation that added this test; kept lower here for routine CI runtime.

    for (const Height& h : heights)
    {
        Shape shape;
        Polygon outer;
        for (int i = 0; i < h.outer_n; ++i)
        {
            outer.push_back(Point2LL(static_cast<coord_t>(std::llround(h.outer[i][0] * 1000.0)), static_cast<coord_t>(std::llround(h.outer[i][1] * 1000.0))));
        }
        Polygon inner;
        for (int i = 0; i < h.inner_n; ++i)
        {
            inner.push_back(Point2LL(static_cast<coord_t>(std::llround(h.inner[i][0] * 1000.0)), static_cast<coord_t>(std::llround(h.inner[i][1] * 1000.0))));
        }
        shape.push_back(outer);
        shape.push_back(inner);
        const Shape offset_shape = shape.offset(-800); // ~0.8mm inset, matching the real infill_area.
        const std::vector<vbct::Contour> contours = VbctAdapter::shapeToContours(offset_shape);

        std::optional<size_t> first_left_n, first_right_n;
        std::optional<double> first_left_extent, first_right_extent;
        int n_mismatches = 0;

        for (int rep = 0; rep < kRepeats; ++rep)
        {
            vbct::VbctResult r14 = vbct::run_vbct(contours, 0.3);
            vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh);
            vbct::Stage6Result r6 = vbct::run_stage6(r5);
            vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
            vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
            vbct::Stage9Result r9 = vbct::run_stage9(r5, r6, r7, r8, /*threshold_mm=*/0.8);

            ASSERT_EQ(r9.domains.size(), 1u) << h.name << " rep=" << rep;
            const vbct::Domain& dom = r9.domains[0];
            ASSERT_EQ(dom.kind, "ring") << h.name << " rep=" << rep;
            ASSERT_TRUE(dom.left.has_value() && dom.right.has_value()) << h.name << " rep=" << rep;

            auto extent = [](const std::vector<vbct::Point2>& pts) {
                double min_x = pts[0].x, max_x = pts[0].x, min_y = pts[0].y, max_y = pts[0].y;
                for (const auto& p : pts)
                {
                    min_x = std::min(min_x, p.x);
                    max_x = std::max(max_x, p.x);
                    min_y = std::min(min_y, p.y);
                    max_y = std::max(max_y, p.y);
                }
                return std::hypot(max_x - min_x, max_y - min_y);
            };
            const size_t left_n = dom.left->points.size();
            const size_t right_n = dom.right->points.size();
            const double left_extent = extent(dom.left->points);
            const double right_extent = extent(dom.right->points);

            if (! first_left_n.has_value())
            {
                first_left_n = left_n;
                first_right_n = right_n;
                first_left_extent = left_extent;
                first_right_extent = right_extent;
            }
            else if (left_n != *first_left_n || right_n != *first_right_n)
            {
                ++n_mismatches;
                std::fprintf(
                    stderr,
                    "[nondeterminism] %s rep=%d: left_n=%zu (first=%zu) right_n=%zu (first=%zu) left_extent=%.6f "
                    "right_extent=%.6f (first left_extent=%.6f right_extent=%.6f)\n",
                    h.name,
                    rep,
                    left_n,
                    *first_left_n,
                    right_n,
                    *first_right_n,
                    left_extent,
                    right_extent,
                    *first_left_extent,
                    *first_right_extent);
            }
        }
        EXPECT_EQ(n_mismatches, 0) << h.name << ": wall point counts varied across " << kRepeats << " repeated in-process calls on identical input.";
    }
}

namespace
{

// A Ring-shaped SliceLayerPart with infill_area sitting a fixed distance inboard of outline, on
// both the outer and inner (hole) contour - mirrors the real relationship WallsComputation leaves
// behind (infill_area inboard of the whole wall stack), needed to distinguish "used infill_area"
// from "used outline" in the Wall Strip tests below.
SliceLayerPart makeRingPartForWallStripTest(coord_t wall_stack_width)
{
    // A hole's own polygon must wind opposite the outer contour for Clipper-based operations
    // (Shape::offset, used by buildCorrugationInput's stripped-side inset) to treat this as one
    // annulus with a hole rather than two separate solid disks - makeCirclePolygon always winds
    // CCW (increasing angle), so the hole polygon is reversed explicitly here. Existing tests that
    // build a Ring the same way without reversing (e.g. ComputeAnchorsForMesh*) get away with it
    // because they only ever feed this into VBCT's own orientation-tolerant CDT-based
    // classification, never into a raw Clipper offset the way this test's own unit under test does.
    Polygon hole_outline = makeCirclePolygon(0, 0, 15000, 48);
    hole_outline.reverse();
    Polygon hole_infill = makeCirclePolygon(0, 0, 15000 + wall_stack_width, 48);
    hole_infill.reverse();

    SliceLayerPart part;
    part.outline.push_back(makeCirclePolygon(0, 0, 30000, 64));
    part.outline.push_back(hole_outline);
    part.infill_area.push_back(makeCirclePolygon(0, 0, 30000 - wall_stack_width, 64));
    part.infill_area.push_back(hole_infill);
    return part;
}

Settings makeWallStripSettings(bool strip_wall_a, bool strip_wall_b, coord_t infill_line_width)
{
    Settings settings;
    settings.add("corrugated_raw_outline_mode", "False");
    settings.add("corrugated_strip_wall_a", strip_wall_a ? "True" : "False");
    settings.add("corrugated_strip_wall_b", strip_wall_b ? "True" : "False");
    // infill_line_width's own def.json entry is unit "mm" - Settings::get<coord_t>() expects the
    // raw stored string in that same unit (mm), converting to microns internally, so the string
    // added here must be in mm too, not a bare micron integer (a mistake this test's own first
    // version made, which - since it silently interpreted "400" as 400mm rather than 400 microns -
    // produced a wildly oversized inset that collapsed the test circles to nothing and masked the
    // real assertions behind a fail-safe fallback path instead of testing the intended values).
    settings.add("infill_line_width", std::to_string(static_cast<double>(infill_line_width) / 1000.0));
    return settings;
}

// The outer contour's own max distance from the origin - a simple, fixture-specific stand-in for
// "how far out does this contour reach", since every test polygon here is a circle.
coord_t maxRadius(const Polygon& polygon)
{
    coord_t max_r = 0;
    for (const Point2LL& p : polygon)
    {
        max_r = std::max(max_r, static_cast<coord_t>(std::llround(std::hypot(static_cast<double>(p.X), static_cast<double>(p.Y)))));
    }
    return max_r;
}

coord_t minRadius(const Polygon& polygon)
{
    coord_t min_r = std::numeric_limits<coord_t>::max();
    for (const Point2LL& p : polygon)
    {
        min_r = std::min(min_r, static_cast<coord_t>(std::llround(std::hypot(static_cast<double>(p.X), static_cast<double>(p.Y)))));
    }
    return min_r;
}

} // namespace

/*!
 * Wall Strip (spec REV 2.6/2.7): with neither strip setting on, buildCorrugationInput must return
 * infill_area completely unmodified - regression coverage for the "default off means
 * byte-identical to before this feature existed" convention this project always follows.
 */
TEST(VbctAdapterTest, BuildCorrugationInputNeitherStrippedMatchesInfillAreaExactly)
{
    const SliceLayerPart part = makeRingPartForWallStripTest(2000);
    const Settings settings = makeWallStripSettings(false, false, 400);

    const Shape result = VbctAdapter::buildCorrugationInput(part, settings);

    ASSERT_EQ(result.size(), part.infill_area.size());
    for (size_t i = 0; i < result.size(); ++i)
    {
        EXPECT_EQ(maxRadius(result[i]), maxRadius(part.infill_area[i])) << "contour " << i;
        EXPECT_EQ(minRadius(result[i]), minRadius(part.infill_area[i])) << "contour " << i;
    }
}

/*!
 * Wall Strip: stripping Wall A (the outer contour) moves that contour out to outline's own outer
 * radius, inset by half infill_line_width - not left at infill_area's own (much further inboard)
 * radius, and not run flush on outline's own raw radius either. Both real defects this feature
 * went through on a real print before this test existed - see this feature's own as-built spec
 * history. Wall B (the un-stripped inner/hole contour) must stay exactly at infill_area's own
 * radius, unaffected.
 */
TEST(VbctAdapterTest, BuildCorrugationInputStripWallAMovesOuterContourToHalfLineWidthInsideOutline)
{
    const coord_t wall_stack_width = 2000; // 2mm - deliberately much bigger than infill_line_width, so the two failure modes (infill_area's own radius, and outline's own raw un-inset radius) are clearly distinguishable from the correct answer.
    const coord_t infill_line_width = 400;
    const SliceLayerPart part = makeRingPartForWallStripTest(wall_stack_width);
    const Settings settings = makeWallStripSettings(/*strip_wall_a=*/true, /*strip_wall_b=*/false, infill_line_width);

    const Shape result = VbctAdapter::buildCorrugationInput(part, settings);

    ASSERT_EQ(result.size(), 2u);
    const size_t outer_idx = std::abs(result[0].area()) >= std::abs(result[1].area()) ? 0 : 1;
    const size_t inner_idx = 1 - outer_idx;

    const coord_t expected_outer_radius = 30000 - infill_line_width / 2;
    EXPECT_NEAR(maxRadius(result[outer_idx]), expected_outer_radius, 50)
        << "Wall A's own contour should sit half infill_line_width inside outline's own true radius (30000), not at infill_area's own radius ("
        << (30000 - wall_stack_width) << ") and not at outline's own raw radius (30000) either.";

    // Wall B (not stripped) unaffected - still exactly at infill_area's own inner radius.
    EXPECT_EQ(minRadius(result[inner_idx]), 15000 + wall_stack_width) << "Wall B's own contour should be untouched when only Wall A is stripped.";
}

/*!
 * Wall Strip: the symmetric case - stripping Wall B (the inner/hole contour) moves it out to
 * outline's own hole radius, inset (toward the hole's own open space) by half infill_line_width,
 * while leaving the un-stripped outer contour (Wall A) exactly at infill_area's own radius.
 */
TEST(VbctAdapterTest, BuildCorrugationInputStripWallBMovesInnerContourToHalfLineWidthInsideOutline)
{
    const coord_t wall_stack_width = 2000;
    const coord_t infill_line_width = 400;
    const SliceLayerPart part = makeRingPartForWallStripTest(wall_stack_width);
    const Settings settings = makeWallStripSettings(/*strip_wall_a=*/false, /*strip_wall_b=*/true, infill_line_width);

    const Shape result = VbctAdapter::buildCorrugationInput(part, settings);

    ASSERT_EQ(result.size(), 2u);
    const size_t outer_idx = std::abs(result[0].area()) >= std::abs(result[1].area()) ? 0 : 1;
    const size_t inner_idx = 1 - outer_idx;

    EXPECT_EQ(maxRadius(result[outer_idx]), 30000 - wall_stack_width) << "Wall A's own contour should be untouched when only Wall B is stripped.";

    const coord_t expected_inner_radius = 15000 + infill_line_width / 2;
    EXPECT_NEAR(minRadius(result[inner_idx]), expected_inner_radius, 50)
        << "Wall B's own contour should sit half infill_line_width inside outline's own true hole radius (15000), not at infill_area's own radius ("
        << (15000 + wall_stack_width) << ") and not at outline's own raw radius (15000) either.";
}

/*!
 * Wall Strip: corrugated_raw_outline_mode's own path (insetOutline) has no concept of a stripped
 * wall yet (see buildCorrugationInput's own doc comment) - strip settings must be silently ignored
 * for the corrugation boundary in that mode, not cause a crash or a malformed shape.
 */
TEST(VbctAdapterTest, BuildCorrugationInputIgnoresStripSettingsInRawOutlineMode)
{
    SliceLayerPart part = makeRingPartForWallStripTest(2000);
    Settings settings;
    settings.add("corrugated_raw_outline_mode", "True");
    settings.add("corrugated_strip_wall_a", "True");
    settings.add("corrugated_strip_wall_b", "True");
    settings.add("infill_line_width", "0.4"); // mm - see makeWallStripSettings' own doc comment for why
    settings.add("wall_line_width_0", "0.4");
    settings.add("wall_line_width_x", "0.4");
    settings.add("wall_line_count", "2");
    settings.add("wall_0_inset", "0");

    const Shape result = VbctAdapter::buildCorrugationInput(part, settings);

    EXPECT_EQ(result.size(), 2u);
}

namespace
{

// A closed rectangle with many points per side - deliberately denser than makeRingPartForWallStripTest's
// own circles, mirroring real Arachne-generated wall density more closely than a clean, sparse
// synthetic square. expandChainContourRange's own first version passed every test built on sparse
// squares but corrupted VBCT's domain segmentation on real geometry - this fixture exists so a
// denser, more realistic point count is exercised directly here too.
Polygon makeDenseRectanglePolygon(coord_t half_width, coord_t half_height, size_t points_per_side = 20)
{
    Polygon polygon;
    auto addSide = [&](Point2LL from, Point2LL to)
    {
        for (size_t i = 0; i < points_per_side; ++i)
        {
            const double frac = static_cast<double>(i) / static_cast<double>(points_per_side);
            polygon.push_back(Point2LL(
                from.X + static_cast<coord_t>(std::llround(frac * static_cast<double>(to.X - from.X))),
                from.Y + static_cast<coord_t>(std::llround(frac * static_cast<double>(to.Y - from.Y)))));
        }
    };
    const Point2LL bl(-half_width, -half_height), br(half_width, -half_height), tr(half_width, half_height), tl(-half_width, half_height);
    addSide(bl, br);
    addSide(br, tr);
    addSide(tr, tl);
    addSide(tl, bl);
    return polygon;
}

} // namespace

/*!
 * expandChainContourRange (Wall Strip, Chain increment): the real bug this test guards against -
 * a first version grafted in a differently-parametrized sub-arc from outline wholesale, which
 * corrupted VBCT's own domain segmentation on real (dense, Arachne-like) geometry, confirmed via
 * direct user report on a real print. Checks the fix directly: the covered range's own points move
 * to outline's own boundary, the untouched rest of the contour stays exactly where infill already
 * had it (byte-identical, not just close), and no vertex is duplicated at the splice's own two cut
 * seams (the second real bug this same test caught, before real-print testing could - a first fix
 * attempt left one duplicate point where the two open arcs were joined back into a closed Polygon).
 * The result carries the original vertex count plus exactly two new ones - the interpolated cut
 * points at each end of the covered range, which won't generally coincide with an existing vertex
 * - not the original count unchanged (cutting mid-edge always introduces new vertices there) and
 * not the original count plus an accidental extra duplicate either.
 */
TEST(VbctAdapterTest, ExpandChainContourRangeMovesOnlyTheCoveredSideAndPreservesVertexOrder)
{
    constexpr coord_t kOutlineHalfWidth = 10000, kOutlineHalfHeight = 2000;
    constexpr coord_t kInfillHalfWidth = 8000, kInfillHalfHeight = 1500; // simulating infill_area sitting inboard of the whole wall stack
    Shape corrugation_input;
    corrugation_input.push_back(makeDenseRectanglePolygon(kInfillHalfWidth, kInfillHalfHeight));
    const Polygon original_infill = corrugation_input[0];
    Shape outline;
    outline.push_back(makeDenseRectanglePolygon(kOutlineHalfWidth, kOutlineHalfHeight));

    // Wall A's own domain wall: sampled along infill's own top edge (close to, not exactly on, the
    // fixture's own vertex positions - matching how a real VBCT domain wall's own point set need
    // not exactly coincide with infill_area's own vertex placement).
    std::vector<Point2LL> domain_wall_points;
    for (int i = -8; i <= 8; ++i)
    {
        domain_wall_points.emplace_back(static_cast<coord_t>(i * 900), kInfillHalfHeight);
    }

    constexpr coord_t kHalfLineWidth = 200; // matches Ring's own convention: half of infill_line_width
    const bool expanded = VbctAdapter::expandChainContourRange(corrugation_input, outline, domain_wall_points, kHalfLineWidth);

    ASSERT_TRUE(expanded);
    ASSERT_EQ(corrugation_input.size(), 1u);
    const Polygon& result = corrugation_input[0];
    // Cutting mid-edge (the domain's own samples don't generally land exactly on existing
    // vertices) typically introduces up to two new interpolated vertices, one at each end of the
    // covered range - so the count should be close to (usually exactly) two more than the
    // original, never fewer (no original vertex is ever dropped) and never much more (no foreign
    // point set spliced in) - checked as a bounded range rather than an exact value, since a cut
    // point can, by numeric coincidence, land exactly on an existing vertex and add nothing new.
    EXPECT_GE(result.size(), original_infill.size());
    EXPECT_LE(result.size(), original_infill.size() + 2);
    // No two consecutive vertices (wrapping around, since Polygon closes implicitly) should coincide -
    // the exact defect an earlier version of this fix left at the splice's own closing seam.
    for (size_t i = 0; i < result.size(); ++i)
    {
        EXPECT_NE(result[i], result[(i + 1) % result.size()]) << "duplicate/zero-length edge at vertex " << i;
    }

    // Every bottom/left/right vertex from the original infill contour must still be present,
    // byte-identical - never touched by this splice.
    std::set<std::pair<coord_t, coord_t>> result_points;
    for (const Point2LL& p : result)
    {
        result_points.insert({ p.X, p.Y });
    }
    bool any_moved = false;
    bool any_reached_outline = false;
    for (const Point2LL& orig : original_infill)
    {
        if (orig.Y == kInfillHalfHeight)
        {
            if (result_points.find({ orig.X, orig.Y }) == result_points.end())
            {
                any_moved = true;
            }
        }
        else
        {
            EXPECT_NE(result_points.find({ orig.X, orig.Y }), result_points.end()) << "untouched vertex (" << orig.X << ", " << orig.Y << ") should still be present";
        }
    }
    // The result's own highest point should reach close to outline's true surface, inset by
    // kHalfLineWidth (confirming the covered range actually moved outward and landed at the
    // correct inset, not just changed slightly or landed flush on the raw surface) - checked via
    // the maximum Y across the whole result rather than per-point, since left/right edge vertices
    // legitimately span the rectangle's own full height and would otherwise be mistaken for
    // un-moved top-edge points by a per-point lower-bound check.
    coord_t max_y = std::numeric_limits<coord_t>::min();
    for (const Point2LL& p : result)
    {
        max_y = std::max(max_y, p.Y);
    }
    const coord_t expected_max_y = kOutlineHalfHeight - kHalfLineWidth;
    any_reached_outline = std::abs(max_y - expected_max_y) < 50;
    EXPECT_TRUE(any_moved) << "the covered range's own vertices should have moved";
    EXPECT_TRUE(any_reached_outline) << "at least some covered-range vertices should have reached outline's own true surface, inset by half infill_line_width - actual max_y=" << max_y
                                      << " expected~" << expected_max_y;
}

// ---------------------------------------------------------------------------------------------
// De Minimis Hole Threshold (spec REV 3.0/3.5/5.9)
// ---------------------------------------------------------------------------------------------

/*!
 * filterDeminimisHoles should remove exactly the hole matching a suppressed identity from the
 * returned Shape, and hand its own original geometry back via ignored_hole_loops - the outer
 * boundary (never a candidate, regardless of match) must survive untouched.
 */
TEST(VbctAdapterTest, FilterDeminimisHolesRemovesMatchingHoleAndReturnsItsGeometry)
{
    Polygon outer = makeCirclePolygon(0, 0, 30000, 64); // CCW -> positive area, never a hole candidate.
    Polygon hole = makeCirclePolygon(10000, 0, 1000, 24); // Small hole, off-centre.
    std::reverse(hole.begin(), hole.end()); // CW -> negative area, matching this engine's own hole convention.
    ASSERT_LT(hole.area(), 0.0) << "test fixture itself must produce a negative-area hole polygon";

    Shape shape;
    shape.push_back(outer);
    shape.push_back(hole);

    Point2LL sum(0, 0);
    for (const Point2LL& p : hole)
    {
        sum += p;
    }
    const Point2LL hole_identity_point = sum / static_cast<coord_t>(hole.size());

    std::vector<std::vector<Point2LL>> ignored_hole_loops;
    const Shape result = VbctAdapter::filterDeminimisHoles(shape, { DeminimisHoleIdentity{ hole_identity_point } }, ignored_hole_loops);

    ASSERT_EQ(result.size(), 1u) << "only the outer boundary should remain";
    EXPECT_GT(result[0].area(), 0.0) << "the surviving polygon should be the outer boundary, not the hole";
    ASSERT_EQ(ignored_hole_loops.size(), 1u) << "the suppressed hole's own geometry should be handed back for stringer clipping";
    EXPECT_EQ(ignored_hole_loops[0].size(), hole.size());
}

/*!
 * An empty suppressed_hole_identities list must be a cheap no-op reproducing the input Shape
 * unchanged - today's exact pre-feature behaviour when the setting is at its default/disabled
 * value (spec REV 3.0/3.5/5.9's own "cheap no-op" requirement).
 */
TEST(VbctAdapterTest, FilterDeminimisHolesIsNoOpWhenNoIdentitiesAreSuppressed)
{
    Polygon outer = makeCirclePolygon(0, 0, 30000, 64);
    Polygon hole = makeCirclePolygon(10000, 0, 1000, 24);
    std::reverse(hole.begin(), hole.end());

    Shape shape;
    shape.push_back(outer);
    shape.push_back(hole);

    std::vector<std::vector<Point2LL>> ignored_hole_loops;
    const Shape result = VbctAdapter::filterDeminimisHoles(shape, {}, ignored_hole_loops);

    ASSERT_EQ(result.size(), 2u);
    EXPECT_TRUE(ignored_hole_loops.empty());
}

/*!
 * End-to-end: a small hole inside a Ring domain's own annulus, suppressed via
 * suppressed_hole_identities, must (a) not fragment/reject the domain (corrugate() still
 * succeeds) and (b) still keep every stringer point out of the hole's own real interior - the
 * "print a real hole, but nothing ever crosses its own void" requirement (spec REV 3.0/3.5/5.9).
 */
TEST(VbctAdapterTest, CorrugateClipsStringersAroundASuppressedHole)
{
    const Point2LL hole_center(20000, 0);
    constexpr coord_t hole_radius = 900; // 0.9mm - small relative to the 15mm annulus width.

    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 96)); // 30mm outer radius.
    Polygon inner_wall = makeCirclePolygon(0, 0, 15000, 64); // 15mm inner radius - the Ring's own true second wall.
    std::reverse(inner_wall.begin(), inner_wall.end());
    ring_shape.push_back(inner_wall);
    Polygon small_hole = makeCirclePolygon(hole_center.X, hole_center.Y, hole_radius, 24);
    std::reverse(small_hole.begin(), small_hole.end());
    ring_shape.push_back(small_hole);

    const std::optional<OpenLinesSet> lines_unsuppressed = VbctAdapter::corrugate(ring_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000);
    ASSERT_TRUE(lines_unsuppressed.has_value()) << "sanity check: VBCT should accept this geometry at all";

    const std::optional<OpenLinesSet> lines = VbctAdapter::corrugate(
        ring_shape,
        /*x=*/0.3,
        /*threshold=*/800,
        /*spacing=*/2000,
        /*z=*/0,
        /*crossover_pitch_mm=*/0.0,
        /*anchor_t0_frac=*/std::nullopt,
        /*other_wall_t0_frac=*/std::nullopt,
        /*reverse_canonical_wall=*/std::nullopt,
        /*crosshatch_enabled=*/false,
        /*chain_anchor_point=*/std::nullopt,
        /*chain_left_near_t_frac=*/std::nullopt,
        /*chain_left_far_t_frac=*/std::nullopt,
        /*chain_right_near_t_frac=*/std::nullopt,
        /*chain_right_far_t_frac=*/std::nullopt,
        /*suppressed_hole_identities=*/{ DeminimisHoleIdentity{ hole_center } });
    ASSERT_TRUE(lines.has_value()) << "a suppressed hole must not make VBCT reject/fragment the domain";

    // A clip-generated endpoint legitimately lands *on* the hole's own boundary edge (that's the
    // whole point of clipping there) - VBCT's own mm-scale floating point and the mm<->micron
    // round-trip (kMicronsPerMm) can then place it a handful of microns to either side of the
    // true boundary. Checked against distance-from-centre with a small tolerance rather than
    // Shape::inside() directly, so this test asserts what actually matters (no point meaningfully
    // inside the void) without being sensitive to that sub-visual numerical fuzz.
    constexpr double kBoundaryToleranceMicrons = 50.0;
    for (const OpenPolyline& line : *lines)
    {
        for (const Point2LL& p : line)
        {
            const double dist_from_hole_center = std::hypot(static_cast<double>(p.X - hole_center.X), static_cast<double>(p.Y - hole_center.Y));
            EXPECT_GE(dist_from_hole_center, static_cast<double>(hole_radius) - kBoundaryToleranceMicrons)
                << "a stringer point (" << p.X << ", " << p.Y << ") fell meaningfully inside the suppressed hole's own real boundary";
        }
    }
}

/*!
 * Regression test for a real user-reported bug: with a suppressed hole present, Linked Corrugation
 * Skin produced no links at all (fell back to unlinked corrugate() every time). Root cause:
 * DomainEvent::clipped - which correctly disables linking for a genuine domain-boundary exit -
 * also fired on a crossing that merely passed through a suppressed hole, and with a stringer
 * sampled every 2mm around a 30mm-radius ring, at least one crossing was essentially guaranteed to
 * hit a hole placed directly on the annulus. Fixed by keeping the linkability gate keyed to the
 * domain's own real boundary only, and instead splitting the *finished* linked-skin path against
 * every ignored hole's own geometry afterwards. This test asserts both halves: linking still
 * succeeds (not a silent fallback to unlinked output), and the resulting path still never runs
 * through the hole's own real interior.
 */
TEST(VbctAdapterTest, CorrugateLinkedSkinStillLinksAndClipsAroundASuppressedHole)
{
    const Point2LL hole_center(22500, 0); // Directly on the annulus between the two walls.
    constexpr coord_t hole_radius = 900;

    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 96)); // 30mm outer radius.
    Polygon inner_wall = makeCirclePolygon(0, 0, 15000, 64); // 15mm inner radius - the Ring's own true second wall.
    std::reverse(inner_wall.begin(), inner_wall.end());
    ring_shape.push_back(inner_wall);
    Polygon small_hole = makeCirclePolygon(hole_center.X, hole_center.Y, hole_radius, 24);
    std::reverse(small_hole.begin(), small_hole.end());
    ring_shape.push_back(small_hole);

    const std::optional<OpenLinesSet> lines_unsuppressed = VbctAdapter::corrugateLinkedSkin(ring_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000);
    ASSERT_TRUE(lines_unsuppressed.has_value()) << "sanity check: this geometry should link fine with no suppression at all";

    const std::optional<OpenLinesSet> lines = VbctAdapter::corrugateLinkedSkin(
        ring_shape,
        /*x=*/0.3,
        /*threshold=*/800,
        /*spacing=*/2000,
        /*z=*/0,
        /*crossover_pitch_mm=*/0.0,
        /*anchor_t0_frac=*/std::nullopt,
        /*other_wall_t0_frac=*/std::nullopt,
        /*reverse_canonical_wall=*/std::nullopt,
        /*chain_anchor_point=*/std::nullopt,
        /*chain_junction_merge_angle_deg=*/30.0,
        /*chain_crosshatch_enabled=*/false,
        /*chain_left_near_t_frac=*/std::nullopt,
        /*chain_left_far_t_frac=*/std::nullopt,
        /*chain_right_near_t_frac=*/std::nullopt,
        /*chain_right_far_t_frac=*/std::nullopt,
        /*suppressed_hole_identities=*/{ DeminimisHoleIdentity{ hole_center } });
    ASSERT_TRUE(lines.has_value()) << "a suppressed hole crossing a stringer rung must not disable Linked Corrugation Skin entirely";
    EXPECT_FALSE(lines->empty());

    constexpr double kBoundaryToleranceMicrons = 50.0;
    bool any_point_checked = false;
    for (const OpenPolyline& line : *lines)
    {
        for (const Point2LL& p : line)
        {
            any_point_checked = true;
            const double dist_from_hole_center = std::hypot(static_cast<double>(p.X - hole_center.X), static_cast<double>(p.Y - hole_center.Y));
            EXPECT_GE(dist_from_hole_center, static_cast<double>(hole_radius) - kBoundaryToleranceMicrons)
                << "a linked-skin path point (" << p.X << ", " << p.Y << ") fell meaningfully inside the suppressed hole's own real boundary";
        }
    }
    EXPECT_TRUE(any_point_checked);
}

/*!
 * Regression test for a real user-reported bug: a Ring domain (outer + a true central hole) with
 * several small extra holes punched through the band, each below corrugated_deminimis_hole_area,
 * fragmented into dozens of tiny Chain domains instead of tracking as a clean single Ring the way
 * it does with no small holes at all. Root cause: filterDeminimisHoles' own nearest-identity match
 * had no distance cap, so once *any* hole in the Shape was suppressed, the real (much larger,
 * non-suppressed) central hole got swept up as "nearest" too, since an unmatched hole always
 * matches *something* once the suppressed set is non-empty. Built via actual Shape::difference()
 * (real Clipper winding, not hand-authored circles) to match production geometry exactly.
 */
TEST(VbctAdapterTest, ComputeAnchorsForMeshRingWithSuppressedSmallHolesStaysOneRing)
{
    Mesh cura_mesh;
    addCorrugationAnchorSettings(cura_mesh.settings_);
    cura_mesh.settings_.remove("corrugated_deminimis_hole_area");
    cura_mesh.settings_.add("corrugated_deminimis_hole_area", "25"); // mm^2, matching the user's own report.
    SliceMeshStorage mesh(&cura_mesh, /*slice_layer_count=*/1);

    Shape outer;
    outer.push_back(makeCirclePolygon(0, 0, 30000, 128)); // 60mm outer diameter.
    Shape center_hole;
    center_hole.push_back(makeCirclePolygon(0, 0, 15000, 96)); // A true central hole - the Ring's own second wall.
    Shape ring = outer.difference(center_hole);

    // 8 small 4mm-diameter through-holes (2mm radius, ~12.57mm^2 each - below the 25mm^2 cutoff),
    // arranged around the band between the two walls, exactly like the user's own test STL.
    Shape small_holes;
    for (int i = 0; i < 8; ++i)
    {
        const double angle = 2.0 * std::numbers::pi * static_cast<double>(i) / 8.0;
        const coord_t cx = static_cast<coord_t>(std::llround(22500.0 * std::cos(angle)));
        const coord_t cy = static_cast<coord_t>(std::llround(22500.0 * std::sin(angle)));
        small_holes.push_back(makeCirclePolygon(cx, cy, 2000, 24));
    }
    ring = ring.difference(small_holes);

    SliceLayerPart part;
    for (const Polygon& polygon : ring)
    {
        part.outline.push_back(polygon);
        part.infill_area.push_back(polygon);
    }
    mesh.layers[0].parts.push_back(part);

    const std::vector<CorrugationAnchor> anchors = VbctAdapter::computeAnchorsForMesh(mesh);
    ASSERT_EQ(anchors.size(), 1u);
    EXPECT_TRUE(anchors[0].has_ring) << "8 small suppressed holes should not prevent this from tracking as one clean Ring";
    EXPECT_EQ(anchors[0].suppressed_hole_identities.size(), 8u) << "exactly the 8 small holes should be suppressed - not the true central hole too";
}

// ---------------------------------------------------------------------------------------------
// Transition Layer (2026-09-15 full-layer redesign): a single per-layer trigger, true whenever
// the mesh's own total domain count (Ring present, 0 or 1, plus the number of Chain domains)
// differs from the immediately preceding layer's own total. No longer decided per-domain or
// applied inside corrugate()/corrugateLinkedSkin() at all - a triggered layer bypasses VBCT
// entirely in favor of a full-part concentric fill (FffGcodeWriter::processCorrugatedInfill).
// ---------------------------------------------------------------------------------------------

/*!
 * A part's own very first layer (no part at all the layer before) is a genuine "nothing to build
 * continuity from" event, caught by the whole-part part_counts signal (0 -> 1) regardless of which
 * domain kind it turns out to be - the whole-part concentric fill is the right fallback here, since
 * there's no prior part-level geometry of any kind to weld a per-domain solid fill onto.
 */
TEST(VbctAdapterTest, ComputeAnchorsForMeshTransitionLayerTriggeredWhenPartFirstAppears)
{
    Mesh cura_mesh;
    addCorrugationAnchorSettings(cura_mesh.settings_);
    SliceMeshStorage mesh(&cura_mesh, /*slice_layer_count=*/3);
    // Layer 0: no parts at all - part_counts 0, nothing to compare layer 1 against but "no part".

    SliceLayerPart ring_part;
    ring_part.outline.push_back(makeCirclePolygon(0, 0, 30000, 96));
    ring_part.outline.push_back(makeCirclePolygon(0, 0, 15000, 64));
    ring_part.infill_area.push_back(makeCirclePolygon(0, 0, 30000, 96));
    ring_part.infill_area.push_back(makeCirclePolygon(0, 0, 15000, 64));
    mesh.layers[1].parts.push_back(ring_part);
    mesh.layers[2].parts.push_back(ring_part);

    const std::vector<CorrugationAnchor> anchors = VbctAdapter::computeAnchorsForMesh(mesh);
    ASSERT_EQ(anchors.size(), 3u);
    ASSERT_TRUE(anchors[1].has_ring);
    ASSERT_TRUE(anchors[2].has_ring);
    EXPECT_TRUE(anchors[1].transition_layer_triggered) << "the part's own first appearance, after a layer with none, should trigger the whole-part fallback";
    EXPECT_FALSE(anchors[2].transition_layer_triggered) << "an already-established, stable Ring should not re-trigger every layer";
}

/*!
 * Per-domain solid-fill substitution restored (spec REV 3.3/3.6/5.10, Trigger 2): a Ring domain
 * appearing within an ALREADY-EXISTING part (the part's own composition switches from Chain to
 * Ring between layers, rather than the part itself appearing fresh) should trigger the new
 * per-domain signal. Unlike the pure domain-count-change case above (same domain kind, one more
 * domain), a Chain-to-Ring switch also legitimately changes which domain is "governing" for the
 * separate stringer-count-stability tracking, so the whole-part fallback may reasonably fire
 * alongside it here too - both mechanisms are allowed to agree on a genuine topology change; this
 * test only asserts the per-domain signal itself, not that the whole-part one stays silent.
 */
TEST(VbctAdapterTest, ComputeAnchorsForMeshRingTransitionLayerTriggeredWithinAnExistingPart)
{
    Mesh cura_mesh;
    addCorrugationAnchorSettings(cura_mesh.settings_);
    SliceMeshStorage mesh(&cura_mesh, /*slice_layer_count=*/2);

    SliceLayerPart chain_part;
    chain_part.outline.push_back(makeRectPolygon(0, 0, 60000, 10000));
    chain_part.infill_area.push_back(makeRectPolygon(0, 0, 60000, 10000));
    mesh.layers[0].parts.push_back(chain_part);

    SliceLayerPart ring_part;
    ring_part.outline.push_back(makeCirclePolygon(0, 0, 30000, 96));
    ring_part.outline.push_back(makeCirclePolygon(0, 0, 15000, 64));
    ring_part.infill_area.push_back(makeCirclePolygon(0, 0, 30000, 96));
    ring_part.infill_area.push_back(makeCirclePolygon(0, 0, 15000, 64));
    mesh.layers[1].parts.push_back(ring_part);

    const std::vector<CorrugationAnchor> anchors = VbctAdapter::computeAnchorsForMesh(mesh);
    ASSERT_EQ(anchors.size(), 2u);
    ASSERT_FALSE(anchors[0].has_ring);
    ASSERT_TRUE(anchors[1].has_ring);
    EXPECT_TRUE(anchors[1].ring_transition_layer_triggered) << "the Ring's first appearance within an already-existing part should trigger the per-domain signal";
}

/*!
 * A second Chain domain appearing on a layer alongside an already-tracked one changes the total
 * domain count (1 -> 2) and should trigger the whole layer - unlike the pre-redesign per-domain
 * identity matching, which domain specifically appeared no longer matters.
 */
TEST(VbctAdapterTest, ComputeAnchorsForMeshTransitionLayerTriggeredForNewlyAppearedChainDomain)
{
    Mesh cura_mesh;
    addCorrugationAnchorSettings(cura_mesh.settings_);
    SliceMeshStorage mesh(&cura_mesh, /*slice_layer_count=*/2);

    Shape bar_a;
    bar_a.push_back(makeRectPolygon(0, 0, 60000, 10000));
    SliceLayerPart part0;
    part0.outline.push_back(bar_a[0]);
    part0.infill_area.push_back(bar_a[0]);
    mesh.layers[0].parts.push_back(part0);

    // Layer 1: the same bar (bar_a), unchanged, plus a second bar (bar_b) - domain count 1 -> 2.
    Shape bar_b;
    bar_b.push_back(makeRectPolygon(0, 300000, 60000, 310000));
    SliceLayerPart part1;
    part1.outline.push_back(bar_a[0]);
    part1.outline.push_back(bar_b[0]);
    part1.infill_area.push_back(bar_a[0]);
    part1.infill_area.push_back(bar_b[0]);
    mesh.layers[1].parts.push_back(part1);

    const std::vector<CorrugationAnchor> anchors = VbctAdapter::computeAnchorsForMesh(mesh);
    ASSERT_EQ(anchors.size(), 2u);
    ASSERT_EQ(anchors[1].chain_domain_wall_identities.size(), 2u) << "layer 1 should have found both bars as independent Chain domains";

    // Per-domain solid-fill substitution restored (spec REV 3.3/3.6/5.10, Trigger 2): a domain
    // appearing within parts[0] now has a clean per-domain identity, so it drives
    // chain_transition_layer_domain_identities instead of the whole-part fallback - only the
    // newly-appeared bar should be flagged, not the already-present one, and the whole-part
    // transition_layer_triggered flag should NOT fire on a pure domain-count change any more (no
    // part-split, no stringer-count change here).
    EXPECT_FALSE(anchors[1].transition_layer_triggered)
        << "a pure domain-count change now drives the per-domain signal, not the whole-part fallback";
    ASSERT_EQ(anchors[1].chain_transition_layer_domain_identities.size(), 1u) << "only the newly-appeared bar should trigger, not the already-tracked one";
    const Point2LL& triggered = anchors[1].chain_transition_layer_domain_identities[0];
    EXPECT_GT(triggered.Y, 150000) << "the triggered domain's own identity point should be bar_b's (near Y=300000), not bar_a's (near Y=0)";
}

/*!
 * A domain whose hysteresis-stabilized stringer count decisively changes between layers (here, a
 * single Chain domain's own aspect ratio changes enough that domain_stringers' own governing
 * ratio crosses a full rounding step, not just an ordinary sub-percent wobble) must trigger, even
 * though it's still the same one domain, in the same one part, the whole time - domain count and
 * part count alone can't see this, since neither one changed.
 */
TEST(VbctAdapterTest, ComputeAnchorsForMeshTransitionLayerTriggeredWhenStringerCountChanges)
{
    Mesh cura_mesh;
    addCorrugationAnchorSettings(cura_mesh.settings_);
    SliceMeshStorage mesh(&cura_mesh, /*slice_layer_count=*/2);

    // Layer 0: a near-square bar - governing_length/spacing lands near the low end of its
    // achievable range for this shape family.
    Shape bar_a;
    bar_a.push_back(makeRectPolygon(0, 0, 60000, 60000));
    SliceLayerPart part0;
    part0.outline.push_back(bar_a[0]);
    part0.infill_area.push_back(bar_a[0]);
    mesh.layers[0].parts.push_back(part0);

    // Layer 1: a much more elongated bar - same single-Chain-domain, single-part composition, but
    // a decisively different aspect ratio pushes the ratio to the high end instead.
    Shape bar_b;
    bar_b.push_back(makeRectPolygon(0, 0, 600000, 10000));
    SliceLayerPart part1;
    part1.outline.push_back(bar_b[0]);
    part1.infill_area.push_back(bar_b[0]);
    mesh.layers[1].parts.push_back(part1);

    const std::vector<CorrugationAnchor> anchors = VbctAdapter::computeAnchorsForMesh(mesh);
    ASSERT_EQ(anchors.size(), 2u);
    ASSERT_TRUE(anchors[0].stringer_count_tracked);
    ASSERT_TRUE(anchors[1].stringer_count_tracked);
    ASSERT_EQ(anchors[0].chain_domain_wall_identities.size(), 1u) << "domain count should stay at 1 the whole time";
    ASSERT_EQ(anchors[1].chain_domain_wall_identities.size(), 1u) << "domain count should stay at 1 the whole time";
    ASSERT_NE(anchors[0].stringer_count, anchors[1].stringer_count) << "test setup should produce a real stringer-count change to exercise";

    EXPECT_TRUE(anchors[1].transition_layer_triggered)
        << "the governing domain's own stringer count changed (" << anchors[0].stringer_count << " -> " << anchors[1].stringer_count
        << ") even though domain count and part count both stayed the same - this alone should trigger";
}

/*!
 * A domain physically splitting into two disconnected SliceLayerParts (e.g. a V-shaped part whose
 * two legs fully separate at a notch, confirmed missing on a real print) must also trigger -
 * layer.parts[0] alone (the only part chain_domain_wall_identities ever looks at, per this
 * pre-pass' own scope limitation) stays a stable single Chain domain across the split, so the
 * domain-count signal alone can't see it; only the raw layer.parts.size() change can.
 */
TEST(VbctAdapterTest, ComputeAnchorsForMeshTransitionLayerTriggeredWhenPartPhysicallySplits)
{
    Mesh cura_mesh;
    addCorrugationAnchorSettings(cura_mesh.settings_);
    SliceMeshStorage mesh(&cura_mesh, /*slice_layer_count=*/2);

    Shape bar_a;
    bar_a.push_back(makeRectPolygon(0, 0, 60000, 10000));
    SliceLayerPart part0;
    part0.outline.push_back(bar_a[0]);
    part0.infill_area.push_back(bar_a[0]);
    mesh.layers[0].parts.push_back(part0);

    // Layer 1: the same bar (bar_a), unchanged, as its own separate SliceLayerPart, plus a second,
    // physically disconnected bar (bar_b) as a second SliceLayerPart - parts.size() 1 -> 2, but
    // each individual part is still just one stable Chain domain on its own.
    Shape bar_b;
    bar_b.push_back(makeRectPolygon(0, 300000, 60000, 310000));
    SliceLayerPart part1a;
    part1a.outline.push_back(bar_a[0]);
    part1a.infill_area.push_back(bar_a[0]);
    SliceLayerPart part1b;
    part1b.outline.push_back(bar_b[0]);
    part1b.infill_area.push_back(bar_b[0]);
    mesh.layers[1].parts.push_back(part1a);
    mesh.layers[1].parts.push_back(part1b);

    const std::vector<CorrugationAnchor> anchors = VbctAdapter::computeAnchorsForMesh(mesh);
    ASSERT_EQ(anchors.size(), 2u);
    ASSERT_EQ(anchors[1].chain_domain_wall_identities.size(), 1u)
        << "layer.parts[0] alone (part1a) is still just one stable Chain domain - the domain-count signal alone would miss this split";

    EXPECT_TRUE(anchors[1].transition_layer_triggered) << "layer.parts.size() changed (1 -> 2) - a physical split should trigger even though parts[0]'s own domain count didn't change";
}


// ---------------------------------------------------------------------------------------------
// Transition Layer, per-domain solid-fill substitution restored (spec REV 3.3/3.6/5.10)
// ---------------------------------------------------------------------------------------------

/*!
 * A Ring domain requested via TransitionLayerRequest::ring should get a solid-fill stringer count
 * far exceeding its ordinary spacing's own count, with its own lines routed to
 * transition_lines_out rather than the returned OpenLinesSet - since there's only one domain here,
 * the returned set should end up empty while transition_lines_out carries everything.
 */
TEST(VbctAdapterTest, CorrugateRingTransitionLayerRequestProducesSolidFillInSeparateOutput)
{
    Shape ring_shape;
    ring_shape.push_back(makeCirclePolygon(0, 0, 30000, 96)); // 30mm outer radius.
    Polygon inner_wall = makeCirclePolygon(0, 0, 15000, 64);
    std::reverse(inner_wall.begin(), inner_wall.end());
    ring_shape.push_back(inner_wall);

    const std::optional<OpenLinesSet> ordinary_lines = VbctAdapter::corrugate(ring_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000);
    ASSERT_TRUE(ordinary_lines.has_value());
    const size_t ordinary_count = ordinary_lines->size();

    VbctAdapter::TransitionLayerRequest request;
    request.ring = true;
    request.solid_fill_spacing = 400; // Small relative to ordinary spacing (2000) - solid-fill pitch.
    OpenLinesSet transition_lines;
    const std::optional<OpenLinesSet> result = VbctAdapter::corrugate(
        ring_shape,
        /*x=*/0.3,
        /*threshold=*/800,
        /*spacing=*/2000,
        /*z=*/0,
        /*crossover_pitch_mm=*/0.0,
        /*anchor_t0_frac=*/std::nullopt,
        /*other_wall_t0_frac=*/std::nullopt,
        /*reverse_canonical_wall=*/std::nullopt,
        /*crosshatch_enabled=*/false,
        /*chain_anchor_point=*/std::nullopt,
        /*chain_left_near_t_frac=*/std::nullopt,
        /*chain_left_far_t_frac=*/std::nullopt,
        /*chain_right_near_t_frac=*/std::nullopt,
        /*chain_right_far_t_frac=*/std::nullopt,
        /*suppressed_hole_identities=*/{},
        request,
        &transition_lines);

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->empty()) << "the only domain on this layer transitioned - the ordinary output should be empty";
    EXPECT_GT(transition_lines.size(), ordinary_count * 3) << "solid-fill spacing should produce far more stringers than the ordinary spacing did";
}

/*!
 * With two independent Chain domains on the same layer, a TransitionLayerRequest naming only one
 * of them (by its own identity point) should solid-fill *only* that domain - the other domain's
 * own stringers, at ordinary spacing, should land in the regular returned OpenLinesSet unaffected.
 */
TEST(VbctAdapterTest, CorrugateChainTransitionLayerRequestOnlyAffectsMatchedDomain)
{
    Shape two_bars_shape;
    two_bars_shape.push_back(makeRectPolygon(0, 0, 60000, 10000));
    two_bars_shape.push_back(makeRectPolygon(0, 200000, 60000, 210000));

    const std::optional<OpenLinesSet> ordinary_lines = VbctAdapter::corrugate(two_bars_shape, /*x=*/0.3, /*threshold=*/800, /*spacing=*/2000);
    ASSERT_TRUE(ordinary_lines.has_value());
    ASSERT_FALSE(ordinary_lines->empty());

    // Identify the first bar's own domain by its own actual mean point (the same "mean of left +
    // right wall points combined" identity vbct::run_stage10's own transition matching uses) -
    // computed via an independent oracle run of Stages 1-9, rather than assumed to equal the bar's
    // own geometric centroid, since VBS subdivision could in principle distribute wall points
    // unevenly enough to shift the mean away from a hand-computed centroid by more than the
    // live-rematch cap allows.
    const std::vector<vbct::Contour> oracle_contours = VbctAdapter::shapeToContours(two_bars_shape);
    const vbct::VbctResult oracle_r14 = vbct::run_vbct(oracle_contours, 0.3);
    const vbct::Stage5Result oracle_r5 = vbct::run_stage5(oracle_r14.mesh, 0.8);
    const vbct::Stage6Result oracle_r6 = vbct::run_stage6(oracle_r5);
    const vbct::Stage7Result oracle_r7 = vbct::run_stage7(oracle_r5, oracle_r6);
    const vbct::Stage8Result oracle_r8 = vbct::run_stage8(oracle_r5, oracle_r6, oracle_r7);
    const vbct::Stage9Result oracle_r9 = vbct::run_stage9(oracle_r5, oracle_r6, oracle_r7, oracle_r8, 0.8);
    const vbct::Domain* first_bar_domain = nullptr;
    for (const vbct::Domain& dom : oracle_r9.domains)
    {
        if (dom.kind == "chain" && dom.left && dom.right)
        {
            first_bar_domain = &dom;
            break;
        }
    }
    ASSERT_NE(first_bar_domain, nullptr);
    double sum_x = 0.0, sum_y = 0.0;
    size_t n = 0;
    for (const auto* wall : { &first_bar_domain->left, &first_bar_domain->right })
    {
        for (const vbct::Point2& p : (*wall)->points)
        {
            sum_x += p.x;
            sum_y += p.y;
            ++n;
        }
    }
    ASSERT_GT(n, 0u);
    const vbct::Point2 first_bar_identity_mm{ sum_x / static_cast<double>(n), sum_y / static_cast<double>(n) };
    const Point2LL first_bar_identity(static_cast<coord_t>(std::llround(first_bar_identity_mm.x * 1000.0)), static_cast<coord_t>(std::llround(first_bar_identity_mm.y * 1000.0)));

    VbctAdapter::TransitionLayerRequest request;
    request.chain_domain_identities = { first_bar_identity };
    request.solid_fill_spacing = 400;
    OpenLinesSet transition_lines;
    const std::optional<OpenLinesSet> result = VbctAdapter::corrugate(
        two_bars_shape,
        /*x=*/0.3,
        /*threshold=*/800,
        /*spacing=*/2000,
        /*z=*/0,
        /*crossover_pitch_mm=*/0.0,
        /*anchor_t0_frac=*/std::nullopt,
        /*other_wall_t0_frac=*/std::nullopt,
        /*reverse_canonical_wall=*/std::nullopt,
        /*crosshatch_enabled=*/false,
        /*chain_anchor_point=*/std::nullopt,
        /*chain_left_near_t_frac=*/std::nullopt,
        /*chain_left_far_t_frac=*/std::nullopt,
        /*chain_right_near_t_frac=*/std::nullopt,
        /*chain_right_far_t_frac=*/std::nullopt,
        /*suppressed_hole_identities=*/{},
        request,
        &transition_lines);

    ASSERT_TRUE(result.has_value());
    EXPECT_FALSE(transition_lines.empty()) << "the matched bar's own lines should have been routed to transition_lines_out";
    EXPECT_FALSE(result->empty()) << "the unmatched bar should still corrugate ordinarily, in the regular returned set";
    EXPECT_LT(result->size(), ordinary_lines->size()) << "the returned set should now hold only one bar's own ordinary stringers, not both";
}

} // namespace cura
// NOLINTEND(*-magic-numbers)
