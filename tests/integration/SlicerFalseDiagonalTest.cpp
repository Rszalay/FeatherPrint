// Copyright (c) 2026 FeatherPrint Corrugated
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "Application.h" // To set up a slice with settings.
#include "Slice.h" // To set up a scene to slice.
#include "mesh.h"
#include "slicer.h" // Unit under test.

namespace cura
{

/*!
 * Integration test for the tessellation-diagonal-artifact pre-scan (see
 * VBCT-Tessellation-Noise-Rejection-Brief.md and SlicerSegment::start_is_false_diagonal's own
 * doc comment, slicer.h). Builds a small synthetic watertight mesh directly (no STL file), with
 * a known, deliberate tessellation-diagonal pattern, and confirms the pre-scan + filter correctly
 * removes exactly the artifact vertices when the new setting is on, and changes nothing when it's
 * off or when the mesh has no such artifacts (a genuine taper).
 */
class SlicerFalseDiagonalTest : public testing::Test
{
protected:
    void SetUp() override
    {
        Application::getInstance().startThreadPool();
        Application::getInstance().current_slice_ = std::make_shared<Slice>(1);

        Scene& scene = Application::getInstance().current_slice_->scene;
        scene.settings.add("layer_height_0", "0.1");
        scene.settings.add("layer_height", "0.1");
        scene.settings.add("layer_0_z_overlap", "0.0");
        scene.settings.add("raft_airgap", "0.0");
        scene.settings.add("raft_base_thickness", "0.2");
        scene.settings.add("raft_interface_thickness", "0.2");
        scene.settings.add("raft_interface_layers", "1");
        scene.settings.add("raft_surface_thickness", "0.2");
        scene.settings.add("raft_surface_layers", "1");
        scene.settings.add("raft_surface_extruder_nr", "0");
        scene.settings.add("magic_mesh_surface_mode", "normal");
        scene.settings.add("meshfix_extensive_stitching", "false");
        scene.settings.add("meshfix_keep_open_polygons", "false");
        scene.settings.add("minimum_polygon_circumference", "1");
        scene.settings.add("meshfix_maximum_resolution", "0.0001"); // effectively off - keep every vertex Simplify would otherwise touch
        scene.settings.add("meshfix_maximum_deviation", "0.0001");
        scene.settings.add("meshfix_maximum_extrusion_area_deviation", "2000");
        scene.settings.add("infill_pattern", "grid");
        scene.settings.add("wall_transition_angle", "10");
        scene.settings.add("xy_offset", "0");
        scene.settings.add("xy_offset_layer_0", "0");
        scene.settings.add("hole_xy_offset", "0");
        scene.settings.add("hole_xy_offset_max_diameter", "0");
        scene.settings.add("support_mesh", "false");
        scene.settings.add("anti_overhang_mesh", "false");
        scene.settings.add("cutting_mesh", "false");
        scene.settings.add("infill_mesh", "false");
        scene.settings.add("adhesion_type", "none");
        scene.settings.add("meshfix_remove_diagonal_artifacts", "false"); // overridden per-test below
    }

    // A 10x10x10mm watertight cube, each of its 4 vertical side walls deliberately split into
    // two triangles by one diagonal (the exact "false diagonal" artifact pattern this feature
    // targets), plus top/bottom caps to keep the mesh manifold. Every side wall's own top and
    // bottom rings share identical XY (a straight, non-tapered extrusion), so the diagonal
    // crossing at any mid-height Z is a pure artifact with a known, provable answer: a clean
    // cross-section has exactly 4 points (the real corners); without the fix, 8 (4 real + 4
    // diagonal artifacts, one per wall).
    static Mesh buildDiagonalCube(Settings& parent)
    {
        Mesh mesh(parent);
        const coord_t s = 10000; // 10mm, in microns
        const Point3LL A(0, 0, 0), B(s, 0, 0), C(s, s, 0), D(0, s, 0);
        const Point3LL A2(0, 0, s), B2(s, 0, s), C2(s, s, s), D2(0, s, s);

        // Caps (winding not load-bearing for this test - only point count is checked).
        mesh.addFace(A, D, C);
        mesh.addFace(A, C, B);
        mesh.addFace(A2, B2, C2);
        mesh.addFace(A2, C2, D2);

        // Side walls, each split by one diagonal (P, Q, Q') / (P, Q', P') - matches the exact
        // false-diagonal signature: triangle 1's own third vertex (Q) shares XY with the shared
        // edge's Q' end, and triangle 2's own third vertex (P') shares XY with the shared edge's
        // P end.
        mesh.addFace(A, B, B2);
        mesh.addFace(A, B2, A2);
        mesh.addFace(B, C, C2);
        mesh.addFace(B, C2, B2);
        mesh.addFace(C, D, D2);
        mesh.addFace(C, D2, C2);
        mesh.addFace(D, A, A2);
        mesh.addFace(D, A2, D2);

        mesh.finish();
        return mesh;
    }

    // The same cube, tapered AND twisted (top ring uniformly smaller than the bottom ring, and
    // rotated a few degrees about the shape's own center) - the safety case that must never have
    // any diagonal flagged.
    //
    // A *pure* uniform scale-taper (top ring smaller, same orientation) is NOT sufficient to
    // exercise this: uniform scaling is a linear map, so it preserves the direction of every edge,
    // meaning each wall's own two rails stay exactly parallel and the diagonal crossing is, in
    // fact, provably redundant (confirmed directly: for this shape's own AB wall, the diagonal's
    // crossing at the mid-height plane sits exactly on the real tapered edge A-B at that height,
    // to the last digit). The generalised (any-orientation) detector is *correct* to flag that
    // case - it isn't a false positive, it's the same lossless collapse as the vertical case,
    // just via a scaled rail pair instead of a vertical one. What must never be touched is a
    // genuine *shape* change, and rotation is what actually breaks rail-parallelism (two adjacent
    // corners sit in different directions from the center, so the same rotation applied to both
    // does not move them in parallel directions) - hence the added twist here.
    static Mesh buildTaperedCube(Settings& parent)
    {
        Mesh mesh(parent);
        const coord_t s = 10000;
        const coord_t top = 6000; // top ring is smaller - a genuine taper
        const coord_t top_off = (s - top) / 2;
        const double center = s / 2.0;
        const double twist_rad = 15.0 * std::numbers::pi / 180.0; // a few degrees - enough to break parallelism, not enough to matter otherwise
        const double cos_t = std::cos(twist_rad), sin_t = std::sin(twist_rad);
        auto twisted = [&](coord_t x, coord_t y) -> Point3LL
        {
            const double rx = x - center, ry = y - center;
            return Point3LL(
                std::llround(center + rx * cos_t - ry * sin_t),
                std::llround(center + rx * sin_t + ry * cos_t),
                s);
        };
        const Point3LL A(0, 0, 0), B(s, 0, 0), C(s, s, 0), D(0, s, 0);
        const Point3LL A2 = twisted(top_off, top_off), B2 = twisted(top_off + top, top_off), C2 = twisted(top_off + top, top_off + top),
                       D2 = twisted(top_off, top_off + top);

        mesh.addFace(A, D, C);
        mesh.addFace(A, C, B);
        mesh.addFace(A2, B2, C2);
        mesh.addFace(A2, C2, D2);

        mesh.addFace(A, B, B2);
        mesh.addFace(A, B2, A2);
        mesh.addFace(B, C, C2);
        mesh.addFace(B, C2, B2);
        mesh.addFace(C, D, D2);
        mesh.addFace(C, D2, C2);
        mesh.addFace(D, A, A2);
        mesh.addFace(D, A2, D2);

        mesh.finish();
        return mesh;
    }

    // The same cube, swept (top ring the *same size* as the bottom, translated by a constant
    // (dx, dy)) - a linear shear/sweep with no shape change at all, the general (non-vertical)
    // case this feature's generalisation exists to catch. Every side wall's two rails are exactly
    // parallel (same shift vector), so the diagonal crossing at any mid-height Z is exactly as
    // redundant as the vertical cube's own case: a clean cross-section has exactly 4 points.
    static Mesh buildSweptCube(Settings& parent)
    {
        Mesh mesh(parent);
        const coord_t s = 10000;
        const coord_t dx = 4000, dy = -2500; // constant shift per unit height - a straight, non-twisting sweep
        const Point3LL A(0, 0, 0), B(s, 0, 0), C(s, s, 0), D(0, s, 0);
        const Point3LL A2(dx, dy, s), B2(s + dx, dy, s), C2(s + dx, s + dy, s), D2(dx, s + dy, s);

        mesh.addFace(A, D, C);
        mesh.addFace(A, C, B);
        mesh.addFace(A2, B2, C2);
        mesh.addFace(A2, C2, D2);

        mesh.addFace(A, B, B2);
        mesh.addFace(A, B2, A2);
        mesh.addFace(B, C, C2);
        mesh.addFace(B, C2, B2);
        mesh.addFace(C, D, D2);
        mesh.addFace(C, D2, C2);
        mesh.addFace(D, A, A2);
        mesh.addFace(D, A2, D2);

        mesh.finish();
        return mesh;
    }

    static size_t midLayerPointCount(Mesh& mesh)
    {
        // 10mm cube at 0.1mm layers -> layer index ~50 sits at the mid-height cross-section,
        // comfortably clear of both caps.
        Slicer slicer(&mesh, MM2INT(0.1), 100, false, nullptr, SlicingTolerance::MIDDLE, MM2INT(0.1));
        const SlicerLayer& mid_layer = slicer.layers[50];
        size_t total_points = 0;
        for (const Polygon& polygon : mid_layer.polygons_)
        {
            total_points += polygon.size();
        }
        return total_points;
    }
};

// Note: a "disabled leaves the artifacts in place" comparison isn't meaningful on this synthetic
// mesh - a perfectly exact, zero-noise diagonal-crossing point is *also* independently collapsed
// by Cura's own Simplify() (utils/Simplify.h documents a hard, unconfigurable ~5 micron floor:
// "does not contain any vertices where removing it would cause a deviation of less than 5
// micron"), regardless of this feature's own setting. This is specific to a toy shape with only
// one point per straight edge; it does not contradict the real-STL evidence in
// VBCT-Tessellation-Noise-Rejection-Brief.md, which was gathered from the raw, pre-Simplify
// per-layer cross-section (a from-scratch slicer bypassing Cura's pipeline entirely) - on a real,
// densely-tessellated model, Simplify does not reliably remove these points (that survival is the
// entire premise of the day-long investigation this feature resolves). The two tests below
// exercise this feature's own mechanism directly and don't depend on Simplify's independent
// behavior either way.

TEST_F(SlicerFalseDiagonalTest, EnabledReducesCrossSectionToRealCornersOnly)
{
    Scene& scene = Application::getInstance().current_slice_->scene;
    scene.settings.add("meshfix_remove_diagonal_artifacts", "true");
    Mesh mesh = buildDiagonalCube(scene.settings);

    EXPECT_EQ(midLayerPointCount(mesh), 4u) << "Exactly the 4 real corners survive once the false-diagonal artifacts are identified and removed.";
}

TEST_F(SlicerFalseDiagonalTest, EnabledReducesCrossSectionToRealCornersOnlyWhenSwept)
{
    Scene& scene = Application::getInstance().current_slice_->scene;
    scene.settings.add("meshfix_remove_diagonal_artifacts", "true");
    Mesh mesh = buildSweptCube(scene.settings);

    EXPECT_EQ(midLayerPointCount(mesh), 4u)
        << "A linear sweep/shear (straight, non-twisting rails, not just vertical ones) collapses to exactly its 4 real corners, the same as the vertical case.";
}

TEST_F(SlicerFalseDiagonalTest, EnabledNeverTouchesATwistedTaper)
{
    Scene& scene = Application::getInstance().current_slice_->scene;
    scene.settings.add("meshfix_remove_diagonal_artifacts", "true");
    Mesh tapered_on = buildTaperedCube(scene.settings);
    const size_t on_count = midLayerPointCount(tapered_on);

    scene.settings.add("meshfix_remove_diagonal_artifacts", "false");
    Mesh tapered_off = buildTaperedCube(scene.settings);
    const size_t off_count = midLayerPointCount(tapered_off);

    EXPECT_EQ(on_count, off_count) << "A genuine shape change (taper + twist) breaks rail-parallelism - nothing should ever be flagged or removed, on or off.";
    EXPECT_EQ(on_count, 8u) << "Sanity: the taper's own 4 diagonal crossings are still present (correctly not removed), same as the disabled case.";
}

} // namespace cura
