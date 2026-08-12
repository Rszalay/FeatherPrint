// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef WALLS_COMPUTATION_H
#define WALLS_COMPUTATION_H

#include <vector>

#include "geometry/Point2LL.h"
#include "settings/Settings.h"
#include "settings/types/LayerIndex.h"
#include "sliceDataStorage.h"
#include "utils/Coord_t.h"
#include "utils/section_type.h"

namespace cura
{

class SliceLayer;
class SliceLayerPart;

/*!
 * Function container for computing the outer walls / insets / perimeters polygons of a layer
 */
class WallsComputation
{
public:
    /*!
     * \brief Basic constructor initialising the parameters with which to
     * perform the walls computation.
     *
     * \param settings The per-mesh settings object to get setting values from.
     * \param layer_nr The layer index that these walls are generated for.
     */
    WallsComputation(
        const Settings& settings,
        const LayerIndex layer_nr,
        double fp_helix_phase = 0.0,
        LayerIndex fp_flange_start_layer = -1,
        int fp_former_ramp = -1,
        int fp_collar_ramp = -1,
        Point2LL fp_phase_origin = Point2LL(0, 0),
        double fp_r_ref = 0.0,
        std::vector<SliceMeshStorage::FpPunchoutGapSpan> fp_punchout_gap_spans = {},
        std::vector<Polygon> fp_interior_holes = {});

    /*!
     * \brief Generates the walls / inner area for all parts in a layer.
     *
     * Generates walls for all parts, by calling the generateWall for the individual parts.
     *
     * \param layer The layer for which to generate the walls and inner area.
     */
    void generateWalls(SliceLayer* layer, SectionType section);

private:
    /*!
     * \brief Settings container to get my settings from.
     *
     * Normally this is a mesh's settings.
     */
    const Settings& settings_;

    /*!
     * \brief The layer that these walls are generated for.
     */
    const LayerIndex layer_nr_;
    const double fp_helix_phase_;
    const LayerIndex fp_flange_start_layer_;
    // Former (Spec REV 3.6) this layer's own ramp position: -1 = not a Former layer, else the
    // magnitude of the ramp step (0 at a band's own start/end, up to n at its peak) — see
    // SliceMeshStorage::fp_former_ramp for the full derivation.
    const int fp_former_ramp_;
    // Collar (Spec REV 4.0) this layer's own ramp position — same shape/semantics as
    // fp_former_ramp_ above, and dispatched through the exact same generateFormer()/
    // generateFormerOpen() calls (see SliceMeshStorage::fp_collar_ramp for the full
    // derivation). Feature priority (Flange > Collar > Former) is already fully resolved
    // upstream by the pre-pass's own whole-band deletion, so at most one of fp_collar_ramp_/
    // fp_former_ramp_ is ever >= 0 on a non-Flange Layer — the dispatch guards below are
    // defensive redundancy, not load-bearing.
    const int fp_collar_ramp_;
    const Point2LL fp_phase_origin_;
    // Curvature-Weighted Stringer Density (Spec REV 2.6): k_ref * R_avg, computed once per
    // mesh (see SliceMeshStorage::fp_r_ref). <= 0 means the feature is inactive for this mesh.
    const double fp_r_ref_;
    // Punchout (Spec REV 3.3) hole-span data for THIS layer only (see
    // SliceMeshStorage::fp_punchout_gap_spans for how it's built) — used to taper the
    // Punchout cutback near a hole's own top/bottom.
    const std::vector<SliceMeshStorage::FpPunchoutGapSpan> fp_punchout_gap_spans_;
    // Interior Opening Carry-Through (Spec REV 3.4 Phase 2) hole polygons for THIS layer
    // only (see SliceMeshStorage::fp_interior_holes for how they're built) — subtracted from
    // inner_area alongside part->outline's own hole polygons.
    const std::vector<Polygon> fp_interior_holes_;

    /*!
     * Generates the walls / inner area for a single layer part.
     *
     * \param part The part for which to generate the insets.
     */
    void generateWalls(SliceLayerPart* part, SectionType section, coord_t print_z = 0);

    /*!
     * Generates the outer inset / perimeter used in spiralize mode for a single layer part. The spiral inset is
     * generated using offsets.
     *
     * \param part The part for which to generate the spiral inset.
     * \param line_width_0 The width of the outer (spiralized) wall.
     * \param wall_0_inset The part for which to generate the spiral inset.
     * \param recompute_outline_based_on_outer_wall Whether we need to recompute the print outline according to the
     *        generated spiral inset.
     */
    void generateSpiralInsets(SliceLayerPart* part, coord_t line_width_0, coord_t wall_0_inset, bool recompute_outline_based_on_outer_wall);
};
} // namespace cura

#endif // WALLS_COMPUTATION_H
