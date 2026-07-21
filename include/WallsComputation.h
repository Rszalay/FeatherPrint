// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef WALLS_COMPUTATION_H
#define WALLS_COMPUTATION_H

#include "geometry/Point2LL.h"
#include "settings/Settings.h"
#include "settings/types/LayerIndex.h"
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
        Point2LL fp_phase_origin = Point2LL(0, 0),
        double fp_r_ref = 0.0);

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
    const Point2LL fp_phase_origin_;
    // Curvature-Weighted Stringer Density (Spec REV 2.6): k_ref * R_avg, computed once per
    // mesh (see SliceMeshStorage::fp_r_ref). <= 0 means the feature is inactive for this mesh.
    const double fp_r_ref_;

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
