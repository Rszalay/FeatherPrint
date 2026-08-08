// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef SLICE_DATA_STORAGE_H
#define SLICE_DATA_STORAGE_H

#include <map>
#include <memory>
#include <optional>

#include "SupportInfillPart.h"
#include "TopSurface.h"
#include "WipeScriptConfig.h"
#include "geometry/MixedLinesSet.h"
#include "geometry/OpenLinesSet.h"
#include "geometry/Point2LL.h"
#include "geometry/Polygon.h"
#include "geometry/SingleShape.h"
#include "settings/Settings.h" //For MAX_EXTRUDERS.
#include "settings/types/Angle.h" //Infill angles.
#include "settings/types/LayerIndex.h"
#include "utils/AABB.h"
#include "utils/AABB3D.h"
#include "utils/NoCopy.h"

namespace cura
{

class Mesh;
class SierpinskiFillProvider;
class LightningGenerator;
class PrimeTower;
class TextureDataProvider;

/*!
 * A SkinPart is a connected area designated as top and/or bottom skin.
 * Surrounding each non-bridged skin area with an outline may result in better top skins.
 * It's filled during FffProcessor.processSliceData(.) and used in FffProcessor.writeGCode(.) to generate the final gcode.
 */
class SkinPart
{
public:
    SingleShape outline; //!< The skinOutline is the area which needs to be 100% filled to generate a proper top&bottom filling. It's filled by the "skin" module. Includes both
                         //!< roofing and non-roofing.
    Shape skin_fill; //!< The part of the skin which is not roofing.
    Shape roofing_fill; //!< The inner infill which has air directly above
    Shape flooring_fill; //!< The inner infill which has air directly below
};

/*!
    The SliceLayerPart is a single enclosed printable area for a single layer. (Also known as islands)
    It's filled during the FffProcessor.processSliceData(.), where each step uses data from the previous steps.
    Finally it's used in the FffProcessor.writeGCode(.) to generate the final gcode.
 */
class SliceLayerPart
{
public:
    AABB boundaryBox; //!< The boundaryBox is an axis-aligned boundary box which is used to quickly check for possible
                      //!< collision between different parts on different layers. It's an optimization used during
                      //!< skin calculations.
    SingleShape outline; //!< The outline is the first member that is filled, and it's filled with polygons that match
                         //!< a cross-section of the 3D model.
    Shape print_outline; //!< An approximation to the outline of what's actually printed, based on the outer wall.
                         //!< Too small parts will be omitted compared to the outline.
    Shape spiral_wall; //!< The centerline of the wall used by spiralize mode. Only computed if spiralize mode is enabled.
    Shape inner_area; //!< The area of the outline, minus the walls. This will be filled with either skin or infill.
    std::vector<SkinPart> skin_parts; //!< The skin parts which are filled for 100% with lines and/or insets.
    std::vector<VariableWidthLines> wall_toolpaths; //!< toolpaths for walls, will replace(?) the insets. Binned by inset_idx.
    std::vector<VariableWidthLines> infill_wall_toolpaths; //!< toolpaths for the walls of the infill areas. Binned by inset_idx.
    Shape top_most_surface; //!< Sub-part of the outline containing the area that is not covered by something above
    Shape bottom_most_surface; //!< Sub-part of the outline containing the area that has nothing below

    /*!
     * The areas inside of the mesh.
     * Like SliceLayerPart::outline, this class member is not used to actually determine the feature area,
     * but is used to compute the inside comb boundary.
     */
    Shape infill_area;

    /*!
     * The areas which need to be filled with sparse (0-99%) infill.
     * Like SliceLayerPart::outline, this class member is not used to actually determine the feature area,
     * but is used to compute the infill_area_per_combine_per_density.
     *
     * These polygons may be cleared once they have been used to generate gradual infill and/or infill combine.
     *
     * If these polygons are not initialized, simply use the normal infill area.
     */
    std::optional<Shape> infill_area_own;

    /*!
     * The areas which need to be filled with sparse (0-99%) infill for different thicknesses.
     * The infill_area is an array to support thicker layers of sparse infill and areas of different infill density.
     *
     * Take an example of infill_area[x][n], the meanings of the indexes are as follows:
     *   x  -  The sparsity of the infill area (0 - 99 in percentage). So, the areas in infill_area[x] are the most dense ones.
     *   n  -  The thickness (in number of layers) of the infill area. See example below:
     *          / ------ -- /         <- layer 3
     *        /-- ------ /            <- layer 2
     *          1   2     3
     * LEGEND: "-" means infill areas.
     * Numbers 1, 2 and 3 identifies 3 different infill areas. Infill areas 1 and 3 have a tickness of 1 layer, while area 2 has a thickness of 2.
     *
     * After the areas have been categoried into different densities, overlapping parts with the same density on multiple layers
     * will be combined into a single layer. Here is an illustration:
     *
     *                        *a group of 3 layers*
     *       NOT COMBINED                                   COMBINED
     *
     *       /22222222 2 2/                                  /22222222 2 2/     <--  the 2 layers next to the middle part are combined into a single layer
     *     /0 22222222 2/              ====>               /0 ........ 2/
     *   /0 1 22222222/                                  /0 1 ......../         <--  the 3 layers in the middle part are combined into a single layer
     *                                                      ^          ^
     *                                                      |          |
     *                                                      |          |-- those two density level 2 layers cannot not be combined in the current implementation.
     *                                                      |              this is because we separate every N layers into groups, and try to combine the layers
     *                                                      |              in each group starting from the bottom one. In this case, those two layers don't have a
     *                                                      |              bottom layer in this group, so they cannot be combined.
     *                                                      |              (TODO) this can be a good future work.
     *                                                      |
     *                                                      |-- ideally, those two layer can be combined as well, but it is not the case now.
     *                                                          (TODO) this can be a good future work.
     *
     * NOTES:
     *   - numbers represent the density levels of each infill
     *
     * This maximum number of layers we can combine is a user setting. This number, say "n", means the maximum number of layers we can combine into one.
     * On the combined layers, the extrusion amount will be higher than the normal extrusion amount because it needs to extrude for multiple layers instead of one.
     *
     * infill_area[x][n] is infill_area of (n+1) layers thick.
     *
     * infill_area[0] corresponds to the most dense infill area.
     * infill_area[x] will lie fully inside infill_area[x+1].
     * infill_area_per_combine_per_density.back()[0] == part.infill area initially
     */
    std::vector<std::vector<Shape>> infill_area_per_combine_per_density;

    /*!
     * Get the infill_area_own (or when it's not instantiated: the normal infill_area)
     * \see SliceLayerPart::infill_area_own
     * \return the own infill area
     */
    Shape& getOwnInfillArea();

    /*!
     * Get the infill_area_own (or when it's not instantiated: the normal infill_area)
     * \see SliceLayerPart::infill_area_own
     * \return the own infill area
     */
    const Shape& getOwnInfillArea() const;

    /*!
     * Searches whether the part has any walls in the specified inset index
     * \param inset_idx The index of the wall
     * \return true if there is at least one ExtrusionLine at the specified wall index, false otherwise
     */
    bool hasWallAtInsetIndex(size_t inset_idx) const;
};

/*!
    The SlicerLayer contains all the data for a single cross section of the 3D model.
 */
class SliceLayer
{
public:
    coord_t printZ; //!< The height at which this layer needs to be printed. Can differ from sliceZ due to the raft.
    coord_t thickness; //!< The thickness of this layer. Can be different when using variable layer heights.
    std::vector<SliceLayerPart> parts; //!< An array of LayerParts which contain the actual data. The parts are printed one at a time to minimize travel outside of the 3D model.
    OpenLinesSet open_polylines; //!< A list of lines which were never hooked up into a 2D polygon. (Currently unused in normal operation)
    std::shared_ptr<TextureDataProvider> texture_data_provider_; //!< Accessor to pre-sliced texture data

    /*!
     * \brief The parts of the model that are exposed at the very top of the
     * model.
     *
     * This is filled only when the top surface is needed.
     */
    TopSurface top_surface;

    /*!
     * \brief The parts of the model that are exposed at the bottom(s) of the model.
     *
     * Note: Filled only when needed.
     */
    Shape bottom_surface;

    /*!
     * Get the all outlines of all layer parts in this layer.
     *
     * \param external_polys_only Whether to only include the outermost outline of each layer part
     * \return A collection of all the outline polygons
     */
    Shape getOutlines(bool external_polys_only = false) const;

    /*!
     * Get the all outlines of all layer parts in this layer.
     * Add those polygons to @p result.
     *
     * \param external_polys_only Whether to only include the outermost outline of each layer part
     * \param result The result: a collection of all the outline polygons
     */
    void getOutlines(Shape& result, bool external_polys_only = false) const;

    ~SliceLayer();
};

/******************/


class SupportLayer
{
public:
    std::vector<SupportInfillPart> support_infill_parts; //!< a list of support infill parts
    Shape support_bottom; //!< Piece of support below the support and above the model. This must not overlap with any of the support_infill_parts or support_roof.
    Shape support_roof; //!< Piece of support above the support and below the model. This must not overlap with any of the support_infill_parts or support_bottom.
                        //   NOTE: This is _all_ of the support_roof, and as such, overlaps with support_fractional_roof!
    Shape support_fractional_roof; //!< If the support distance is not exactly a multiple of the layer height,
                                   //   the first part of support just underneath the model needs to be printed at a fracional layer height.
    Shape support_mesh_drop_down; //!< Areas from support meshes which should be supported by more support
    Shape support_mesh; //!< Areas from support meshes which should NOT be supported by more support
    Shape anti_overhang; //!< Areas where no overhang should be detected.

    /*!
     * Exclude the given polygons from the support infill areas and update the SupportInfillParts.
     *
     * \param exclude_polygons The polygons to exclude
     * \param exclude_polygons_boundary_box The boundary box for the polygons to exclude
     */
    void excludeAreasFromSupportInfillAreas(const Shape& exclude_polygons, const AABB& exclude_polygons_boundary_box);

    /* Fill up the infill parts for the support with the given support polygons. The support polygons will be split into parts.
     *
     * \param area The support polygon to fill up with infill parts.
     * \param support_fill_per_layer The support polygons to fill up with infill parts.
     * \param support_line_width Line width of the support extrusions.
     * \param wall_line_count Wall-line count around the fill.
     * \param use_fractional_config (optional, default to false) If the area should be added as fractional support.
     * \param unionAll (optional, default to false) Wether to 'union all' for the split into parts bit.
     * \param custom_line_distance (optional, default to 0) Distance between lines of the infill pattern. custom_line_distance of 0 means use the default instead.
     */
    void fillInfillParts(
        const Shape& area,
        const coord_t support_line_width,
        const coord_t wall_line_count,
        const bool use_fractional_config = false,
        const bool unionAll = false,
        const coord_t custom_line_distance = 0)
    {
        for (const SingleShape& island_outline : area.splitIntoParts(unionAll))
        {
            support_infill_parts.emplace_back(island_outline, support_line_width, use_fractional_config, wall_line_count, custom_line_distance);
        }
    }

    /* Fill up the infill parts for the support with the given support polygons. The support polygons will be split into parts. This also takes into account fractional-height
     * support layers.
     *
     * \param layer_nr The layer-index of the support layer to be filled.
     * \param support_fill_per_layer The support polygons to fill up with infill parts.
     * \param infill_layer_height The layer height of the support-fill.
     * \param meshes The model meshes to be supported, needed here to handle fractional support layer height.
     * \param support_line_width Line width of the support extrusions.
     * \param wall_line_count Wall-line count around the fill.
     * \param grow_layer_above (optional, default to 0) In cases where support shrinks per layer up, an appropriate offset may be nescesary.
     * \param unionAll (optional, default to false) Wether to 'union all' for the split into parts bit.
     * \param custom_line_distance (optional, default to 0) Distance between lines of the infill pattern. custom_line_distance of 0 means use the default instead.
     */
    void fillInfillParts(
        const LayerIndex layer_nr,
        const std::vector<Shape>& support_fill_per_layer,
        const coord_t infill_layer_height,
        const std::vector<std::shared_ptr<SliceMeshStorage>>& meshes,
        const coord_t support_line_width,
        const coord_t wall_line_count,
        const coord_t grow_layer_above = 0,
        const bool unionAll = false,
        const coord_t custom_line_distance = 0);
};

class SupportStorage
{
public:
    bool generated; //!< whether generateSupportGrid(.) has completed (successfully)

    int layer_nr_max_filled_layer; //!< the layer number of the uppermost layer with content

    std::vector<AngleDegrees> support_infill_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<AngleDegrees> support_infill_angles_layer_0; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<AngleDegrees> support_roof_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<AngleDegrees> support_bottom_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer

    std::vector<SupportLayer> supportLayers;
    std::shared_ptr<SierpinskiFillProvider> cross_fill_provider; //!< the fractal pattern for the cross (3d) filling pattern
    std::shared_ptr<LightningGenerator> lightning_generator; //!< Pre-computed structure for Lightning type infill

    SupportStorage();
    ~SupportStorage();
};
/******************/

class SubDivCube; // forward declaration to prevent dependency loop

class SliceMeshStorage
{
public:
    Settings& settings;
    std::vector<SliceLayer> layers;
    std::string mesh_name;

    LayerIndex layer_nr_max_filled_layer; //!< the layer number of the uppermost layer with content (modified while infill meshes are processed)

    // FeatherPrint: cumulative helix phase per layer (running integral of dz/arc_total).
    // Pre-computed sequentially before the parallel wall generation pass so that
    // each layer's generator can use the exact accumulated phase rather than the
    // instantaneous approximation, giving constant intersection angle on tapered tubes.
    std::vector<double> fp_helix_phase;

    // FeatherPrint: first layer index of the Flange zone (-1 = no Flange detected).
    // Layers [fp_flange_start_layer, layer_nr_max_filled_layer] are Flange layers.
    // Set during the helix pre-pass in FffPolygonGenerator.
    LayerIndex fp_flange_start_layer{ -1 };

    // FeatherPrint: per-layer Phase Origin for Anchor Distribution (Spec REV 2.2), one entry
    // per layer like fp_helix_phase. Deviates from REV 2.2's stated "recomputed independently
    // each Layer against one fixed world point" design: testing found that a fixed point
    // (seeded at world x=0, which is also the symmetry axis of most fuselage-like
    // cross-sections) produces large jumps whenever the true nearest point is nearly tied
    // between the two symmetric sides of the boundary — a small, smooth shape change between
    // layers can flip the winner to the mirror point on the opposite side of the part. Instead,
    // each layer's origin is the nearest point on THAT layer's own outer wall to the PREVIOUS
    // layer's own origin (a genuine continuity-tracking walk), seeded at the first layer that
    // carries a Stringer as the point on that layer's outer wall at world x=0 furthest in +Y.
    // This reintroduces the drift risk REV 2.2 explicitly tried to avoid — a local shape
    // distortion can now bias every layer above it, not just the one layer it occurs on — a
    // known, accepted tradeoff versus the large symmetry-tie jumps of the fixed-point scheme.
    // Set during the helix pre-pass in FffPolygonGenerator.
    std::vector<Point2LL> fp_phase_origin;

    // FeatherPrint: Curvature-Weighted Stringer Density (Spec REV 2.6) reference radius, in
    // the same coord_t-valued double units as ArcParam's arc-lengths (µm). Computed ONCE for
    // the whole mesh (not per layer, unlike fp_helix_phase/fp_phase_origin above) as
    // R_ref = k_ref * R_avg, where R_avg is a length-weighted average of local perimeter
    // radius-of-curvature sampled across every layer. 0.0 means "not computed" (feature
    // inactive or a degenerate mesh with no measurable perimeter) — callers must treat that as
    // "use raw arc-length, no warping" rather than passing it through. Set during the helix
    // pre-pass in FffPolygonGenerator, in a first sequential loop across all layers before the
    // existing per-layer phase/origin loop (which needs R_ref already known to compute
    // W_total(z) for the updated Δphase formula).
    double fp_r_ref{ 0.0 };

    // FeatherPrint: Shore (Internal Overhang Bridging, Spec REV 3.1, as rewritten 22 Jul 26).
    // Per-layer candidate regions ("O" in the spec) needing a bridge, from two simple, local,
    // two-layer tests -- no whole-mesh reachability computation, unlike this field's own prior
    // implementation (see below for why that was replaced):
    //   T (Top Surface) = layer_n \ layer_{n+1}   -- no angle filter, mirrors ordinary top-skin
    //                                                 detection (see generateTopAndBottomMostSurfaces,
    //                                                 skin.cpp) rather than testing a printable angle.
    //   Enclosure        = T's own footprint, at Layer n-1, falls within that Layer's own outer
    //                       envelope (OML extent, holes filled in) AND is hollow there -- hollow
    //                       meaning outside the one-line-width printed Wall band inset from that
    //                       envelope, NOT "not solid in the raw CAD mesh." FeatherPrint never
    //                       prints solid fill beyond that Wall band regardless of whether the
    //                       raw mesh is solid there -- using the raw mesh's own solidity here
    //                       made this test fire nowhere on any model without a CAD-modeled
    //                       cavity, confirmed via real diagnostic testing (a solid mesh's own
    //                       "outer envelope" and "solid area" are identical by construction).
    // O = T restricted to the portion passing Enclosure.
    //
    // Because FeatherPrint prints hollow with no infill, there is no separate concept of
    // "internal" print geometry -- what matters is only whether the space beneath a given
    // overhang is open exterior air or bounded by the model's own envelope, which is exactly
    // what Enclosure tests, directly and locally. An earlier implementation instead asked
    // whether new solid area extended the model's own silhouette outward (three escalating local
    // heuristics, then a whole-mesh reachability flood fill when those proved insufficient) --
    // technically correct for that older question, but real-print testing found it didn't match
    // the actual use case: a shelf fused to an unrelated wall, confirmed genuinely disconnected
    // from any enclosed cavity by the flood fill, still needed bridging. The spec's own
    // definition of "internal" was rewritten to reflect this rather than tuning the flood fill
    // further -- see the spec's own Revision History for the full account.
    //
    // Each candidate is a SingleShape (its own outer polygon plus any holes -- a literal hole
    // feature in the surface being capped, or a not-yet-closed portion of the same top-facing
    // region). Usually 0 or 1 per Layer, not assumed unique. Computed in a sequential per-mesh
    // pre-pass in FffPolygonGenerator.cpp that only needs per-layer outlines (no dependency on
    // fp_helix_phase/fp_phase_origin -- Shore has no relationship to Stringer at all).
    std::vector<std::vector<SingleShape>> fp_shore_overhangs;

    // FeatherPrint: Shore's resulting bridge line segments, recorded at the LAYER THEY SHOULD BE
    // PRINTED AT (n - top_layers, where n is the Layer the internal overhang was detected at),
    // not the detection Layer itself -- this places the bridge material early enough that
    // ordinary top skin (top_layers worth) can build up on it by the time the slicer reaches the
    // true closure. Consumed in FffGcodeWriter.cpp alongside ordinary infill processing, emitted
    // via LayerPlan::addLinesByOptimizer with the mesh's own infill config -- ordinary Infill
    // print-feature classification, no new machinery.
    std::vector<std::vector<std::pair<Point2LL, Point2LL>>> fp_shore_bridges;

    // FeatherPrint: Interior Opening Carry-Through (Spec REV 3.4 Phase 2). A hole in a
    // top-/bottom-facing horizontal surface with little or no vertical depth has boundary
    // edges coplanar with a Z-cutting-plane rather than crossing it, so it never produces an
    // ordinary sliced polygon anywhere (part->outline, open_polylines) -- confirmed via real
    // test print, the same fundamental "no 2D per-layer signal" limitation Shore's own REV 3.2
    // development already hit for a fully-open top. Detected instead from the raw mesh's own
    // boundary-edge topology (MeshFace::connected_face_index_, computed once at mesh-finish
    // time, independent of slicing) in FffPolygonGenerator::sliceModel() -- the only point in
    // the pipeline this data survives, before MeshGroup::clear() discards it. Each entry is a
    // 2D polygon (the coplanar loop, projected) already confirmed to lie strictly inside that
    // Layer's own outer contour (not coincident with it -- a loop matching the Layer's own
    // silhouette is the already-handled "whole face open" case, Flange's/Shore's territory).
    // Consumed in WallsComputation.cpp alongside part->outline's own hole polygons when
    // computing inner_area (Phase 1) -- this does not add any new printed Wall/rim/Terminal,
    // only prevents top/bottom skin fill from painting over the opening.
    std::vector<std::vector<Polygon>> fp_interior_holes;

    // FeatherPrint: Punchout (Spec REV 3.3) hole-span tracking. Every other FeatherPrint pass
    // (Whip, Shore) treats each Layer's open_polylines independently -- nothing previously
    // tracked which open-polyline gap on Layer n is the SAME physical hole as a gap on Layer
    // n+1. Punchout's blend-toward-the-wall behavior near a hole's own top/bottom needs that
    // span, so this pre-pass (FffPolygonGenerator.cpp, right after the Shore pre-pass) builds
    // it: each Layer's ring gaps (same "end of one open polyline -> start of the next, in ring
    // order" pairing WallsComputation's own Punchout dispatch uses) are chained across Layers
    // by nearest-endpoint-pair matching, and each chain's first/last Layer index is recorded
    // on every gap belonging to it.
    struct FpPunchoutGapSpan
    {
        Point2LL start; // end of the "previous" open polyline in ring order, this Layer
        Point2LL end;   // start of the "next" open polyline in ring order, this Layer
        LayerIndex hole_bottom{ -1 }; // first Layer index at which this same hole's gap exists
        LayerIndex hole_top{ -1 };    // last Layer index at which this same hole's gap exists

        // Contour matching: featherprint_punchout_contour_samples points sampled along the
        // real closed-perimeter wall immediately below (hole_bottom - 1) and immediately
        // above (hole_top + 1) this hole, ordered from the hole's own start edge to its end
        // edge so index i means the same relative position in both. Identical across every
        // Layer belonging to the same hole (populated once per hole in the pre-pass, copied
        // onto each member). Empty if the hole reaches the very top/bottom of the mesh (no
        // valid bounding closed Layer on that side) — callers must treat empty as "no contour
        // data available," falling back to a plain straight chord.
        std::vector<Point2LL> contour_below, contour_above;
        coord_t z_below{ 0 }, z_above{ 0 };
    };
    std::vector<std::vector<FpPunchoutGapSpan>> fp_punchout_gap_spans;

    std::vector<AngleDegrees> infill_angles; //!< a list of angle values which is cycled through to determine the infill angle of each layer
    std::vector<AngleDegrees> roofing_angles; //!< a list of angle values which is cycled through to determine the roofing angle of each layer
    std::vector<AngleDegrees> flooring_angles; //!< a list of angle values which is cycled through to determine the flooring angle of each layer
    std::vector<AngleDegrees> skin_angles; //!< a list of angle values which is cycled through to determine the skin angle of each layer
    std::vector<Shape> overhang_areas; //!< For each layer the areas that are classified as overhang on this mesh.
    std::vector<Shape> full_overhang_areas; //!< For each layer the full overhang without the tangent of the overhang angle removed, such that the overhang area adjoins the
                                            //!< areas of the next layers.
    std::vector<std::vector<Shape>> overhang_points; //!< For each layer a list of points where point-overhang is detected. This is overhang that hasn't got any surface area,
                                                     //!< such as a corner pointing downwards.
    AABB3D bounding_box; //!< the mesh's bounding box

    std::shared_ptr<SubDivCube> base_subdiv_cube;
    std::shared_ptr<SierpinskiFillProvider> cross_fill_provider; //!< the fractal pattern for the cross (3d) filling pattern

    std::shared_ptr<LightningGenerator> lightning_generator; //!< Pre-computed structure for Lightning type infill

    RetractionAndWipeConfig retraction_wipe_config; //!< Per-Object retraction and wipe settings.

    /*!
     * \brief Creates a storage space for slice results of a mesh.
     * \param mesh The mesh that the storage space belongs to.
     * \param slice_layer_count How many layers are needed to store the slice
     * results of the mesh. This needs to be at least as high as the highest
     * layer that contains a part of the mesh.
     */
    SliceMeshStorage(Mesh* mesh, const size_t slice_layer_count);

    /*!
     * \param extruder_nr The extruder for which to check
     * \return whether a particular extruder is used by this mesh
     */
    bool getExtruderIsUsed(const size_t extruder_nr) const;

    /*!
     * \param extruder_nr The extruder for which to check
     * \param layer_nr the layer for which to check
     * \return whether a particular extruder is used by this mesh on a particular layer
     */
    bool getExtruderIsUsed(const size_t extruder_nr, const LayerIndex& layer_nr) const;

    /*!
     * Gets whether this is a printable mesh (not an infill mesh, slicing mesh,
     * etc.)
     * \return True if it's a mesh that gets printed.
     */
    bool isPrinted() const;

    /*!
     * \return the mesh's user specified z seam hint
     */
    Point2LL getZSeamHint() const;
};

class SliceDataStorage : public NoCopy
{
public:
    size_t print_layer_count; //!< The total number of layers (except the raft and filler layers)

    Point3LL model_size, model_min, model_max;
    AABB3D machine_size; //!< The bounding box with the width, height and depth of the printer.
    std::vector<std::shared_ptr<SliceMeshStorage>> meshes;

    std::vector<RetractionAndWipeConfig> retraction_wipe_config_per_extruder; //!< Config for retractions, extruder switch retractions, and wipes, per extruder.

    SupportStorage support;

    std::vector<MixedLinesSet> skirt_brim[MAX_EXTRUDERS]; //!< Skirt/brim polygons per extruder, ordered from inner to outer polygons.
    ClosedLinesSet support_brim; //!< brim lines for support, going from the edge of the support inward. \note Not ordered by inset.

    // Storage for the outline of the raft-parts. Will be filled with lines when the GCode is generated.
    Shape raft_base_outline;
    Shape raft_interface_outline;
    Shape raft_surface_outline;

    int max_print_height_second_to_last_extruder; //!< Used in multi-extrusion: the layer number beyond which all models are printed with the same extruder
    std::vector<int> max_print_height_per_extruder; //!< For each extruder the highest layer number at which it is used.
    std::vector<size_t> max_print_height_order; //!< Ordered indices into max_print_height_per_extruder: back() will return the extruder number with the highest print height.

    std::vector<int> spiralize_seam_vertex_indices; //!< the index of the seam vertex for each layer
    std::vector<Shape*> spiralize_wall_outlines; //!< the wall outline polygons for each layer

    //!< Pointer to primer tower handler object (a null pointer indicates that there is no prime tower)
    PrimeTower* prime_tower_{ nullptr };

    std::vector<Shape> ooze_shield; // oozeShield per layer
    Shape draft_protection_shield; //!< The polygons for a heightened skirt which protects from warping by gusts of wind and acts as a heated chamber.

    /*!
     * \brief Creates a new slice data storage that stores the slice data of the
     * current mesh group.
     */
    SliceDataStorage();

    ~SliceDataStorage();

    /*!
     * Get all outlines within a given layer.
     *
     * \param layer_nr The index of the layer for which to get the outlines
     * (negative layer numbers indicate the raft).
     * \param include_support Whether to include support in the outline.
     * \param include_prime_tower Whether to include the prime tower in the outline.
     * \param include_models Whether to include the models in the outline
     * \param external_polys_only Whether to disregard all hole polygons.
     * \param extruder_nr (optional) only give back outlines for this extruder (where the walls are printed with this extruder)
     */
    Shape getLayerOutlines(
        const LayerIndex layer_nr,
        const bool include_support,
        const bool include_prime_tower,
        const bool external_polys_only = false,
        const int extruder_nr = -1,
        const bool include_models = true) const;

    /*!
     * Get the axis-aligned bounding-box of the complete model (all meshes).
     */
    AABB3D getModelBoundingBox() const;

    /*!
     * Get the extruders used.
     *
     * \return A vector of booleans indicating whether the extruder with the
     * corresponding index is used in the mesh group.
     */
    std::vector<bool> getExtrudersUsed() const;

    /*!
     * Get the extruders used on a particular layer.
     *
     * \param layer_nr the layer for which to check
     * \return a vector of bools indicating whether the extruder with corresponding index is used in this layer.
     */
    std::vector<bool> getExtrudersUsed(LayerIndex layer_nr) const;

    /*!
     * Gets whether prime blob is enabled for the given extruder number.
     *
     * \param extruder_nr the extruder number to check.
     * \return a bool indicating whether prime blob is enabled for the given extruder number.
     */
    bool getExtruderPrimeBlobEnabled(const size_t extruder_nr) const;

    /*!
     * Gets the border of the usable print area for this machine.
     *
     * \param extruder_nr The extruder for which to return the allowed areas. -1 if the areas allowed for all extruders should be returned.
     * \return the Shape representing the usable area of the print bed.
     */
    Shape getMachineBorder(int extruder_nr = -1) const;

    /*!
     * @return The raw outer build plate shape without any disallowed area
     */
    Shape getRawMachineBorder() const;

    void initializePrimeTower();

private:
    /*!
     * Construct the retraction_wipe_config_per_extruder
     */
    std::vector<RetractionAndWipeConfig> initializeRetractionAndWipeConfigs();
};

} // namespace cura

#endif // SLICE_DATA_STORAGE_H
