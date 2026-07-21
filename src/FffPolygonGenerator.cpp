// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream> // ifstream.good()
#include <map> // multimap (ordered map allowing duplicate keys)
#include <numbers>
#include <numeric>

#include <spdlog/spdlog.h>

// Code smell: Order of the includes is important here, probably due to some forward declarations which might be masking some undefined behaviours
// clang-format off
#include "Application.h"
#include "ConicalOverhang.h"
#include "ExtruderTrain.h"
#include "FffPolygonGenerator.h"
#include "infill.h"
#include "InterlockingGenerator.h"
#include "layerPart.h"
#include "MeshGroup.h"
#include "MeshMaterialSplitter.h"
#include "Mold.h"
#include "multiVolumes.h"
#include "PrintFeature.h"
#include "raft.h"
#include "skin.h"
#include "SkirtBrim.h"
#include "Slice.h"
#include "TextureDataProvider.h"
#include "sliceDataStorage.h"
#include "slicer.h"
#include "support.h"
#include "TopSurface.h"
#include "TreeSupport.h"
#include "WallsComputation.h"
#include "settings/EnumSettings.h"
#include "infill/DensityProvider.h"
#include "infill/ImageBasedDensityProvider.h"
#include "infill/LightningGenerator.h"
#include "infill/SierpinskiFillProvider.h"
#include "infill/SubDivCube.h"
#include "infill/UniformDensityProvider.h"
#include "progress/Progress.h"
#include "progress/ProgressEstimator.h"
#include "progress/ProgressEstimatorLinear.h"
#include "progress/ProgressStageEstimator.h"
#include "settings/AdaptiveLayerHeights.h"
#include "settings/types/Angle.h"
#include "settings/types/LayerIndex.h"
#include "utils/algorithm.h"
#include "utils/ThreadPool.h"
#include "utils/gettime.h"
#include "utils/math.h"
#include "PrimeTower/PrimeTower.h"
#include "geometry/OpenPolyline.h"
#include "utils/Simplify.h"
// clang-format on

namespace cura
{


bool FffPolygonGenerator::generateAreas(SliceDataStorage& storage, MeshGroup* meshgroup, TimeKeeper& timeKeeper)
{
    MeshMaterialSplitter::makeMaterialModifierMeshes(meshgroup);

    if (! sliceModel(meshgroup, timeKeeper, storage))
    {
        return false;
    }

    slices2polygons(storage, timeKeeper);

    return true;
}

size_t FffPolygonGenerator::getDraftShieldLayerCount(const size_t total_layers) const
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    if (! mesh_group_settings.get<bool>("draft_shield_enabled"))
    {
        return 0;
    }
    switch (mesh_group_settings.get<DraftShieldHeightLimitation>("draft_shield_height_limitation"))
    {
    case DraftShieldHeightLimitation::FULL:
        return total_layers;
    case DraftShieldHeightLimitation::LIMITED:
        return std::max(
            (coord_t)0,
            (mesh_group_settings.get<coord_t>("draft_shield_height") - mesh_group_settings.get<coord_t>("layer_height_0")) / mesh_group_settings.get<coord_t>("layer_height") + 1);
    default:
        spdlog::warn("A draft shield height limitation option was added without implementing the new option in getDraftShieldLayerCount.");
        return total_layers;
    }
}

bool FffPolygonGenerator::sliceModel(MeshGroup* meshgroup, TimeKeeper& timeKeeper, SliceDataStorage& storage) /// slices the model
{
    Progress::messageProgressStage(Progress::Stage::SLICING, &timeKeeper);

    storage.model_min = meshgroup->min();
    storage.model_max = meshgroup->max();
    storage.model_size = storage.model_max - storage.model_min;

    spdlog::info("Slicing model...");

    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;

    // regular layers
    int slice_layer_count = 0; // Use signed int because we need to subtract the initial layer in a calculation temporarily.

    // Initial layer height of 0 is not allowed. Negative layer height is nonsense.
    coord_t initial_layer_thickness = mesh_group_settings.get<coord_t>("layer_height_0");
    if (initial_layer_thickness <= 0)
    {
        spdlog::error("Initial layer height {} is disallowed.", initial_layer_thickness);
        return false;
    }

    // Layer height of 0 is not allowed. Negative layer height is nonsense.
    const coord_t layer_thickness = mesh_group_settings.get<coord_t>("layer_height");
    if (layer_thickness <= 0)
    {
        spdlog::error("Layer height {} is disallowed.\n", layer_thickness);
        return false;
    }

    // variable layers
    AdaptiveLayerHeights* adaptive_layer_heights = nullptr;
    const bool use_variable_layer_heights = mesh_group_settings.get<bool>("adaptive_layer_height_enabled");

    if (use_variable_layer_heights)
    {
        // Calculate adaptive layer heights
        const auto variable_layer_height_max_variation = mesh_group_settings.get<coord_t>("adaptive_layer_height_variation");
        const auto variable_layer_height_variation_step = mesh_group_settings.get<coord_t>("adaptive_layer_height_variation_step");
        const auto adaptive_threshold = mesh_group_settings.get<coord_t>("adaptive_layer_height_threshold");
        adaptive_layer_heights
            = new AdaptiveLayerHeights(layer_thickness, variable_layer_height_max_variation, variable_layer_height_variation_step, adaptive_threshold, meshgroup);

        // Get the amount of layers
        slice_layer_count = adaptive_layer_heights->getLayerCount();
    }
    else
    {
        // Find highest layer count according to each mesh's settings.
        for (const Mesh& mesh : meshgroup->meshes)
        {
            if (! mesh.isPrinted())
            {
                continue;
            }
            const coord_t mesh_height = mesh.max().z_;
            switch (mesh.settings_.get<SlicingTolerance>("slicing_tolerance"))
            {
            case SlicingTolerance::MIDDLE:
                if (storage.model_max.z_ < initial_layer_thickness)
                {
                    slice_layer_count = std::max(slice_layer_count, (mesh_height > initial_layer_thickness / 2) ? 1 : 0); // One layer if higher than half initial layer height.
                }
                else
                {
                    slice_layer_count = std::max(slice_layer_count, static_cast<int>(round_divide_signed(mesh_height - initial_layer_thickness, layer_thickness) + 1));
                }
                break;
            case SlicingTolerance::EXCLUSIVE:
            {
                int new_slice_layer_count = 0;
                if (mesh_height >= initial_layer_thickness) // If less than the initial layer thickness, leave it at 0.
                {
                    new_slice_layer_count = static_cast<int>(floor_divide_signed(mesh_height - 1 - initial_layer_thickness, layer_thickness) + 1);
                }
                if (new_slice_layer_count > 0) // If there is at least one layer already, then...
                {
                    new_slice_layer_count += 1; // ... need one extra, since we clear the top layer after the repeated intersections with the layer above.
                }
                slice_layer_count = std::max(slice_layer_count, new_slice_layer_count);
                break;
            }
            case SlicingTolerance::INCLUSIVE:
                if (mesh_height < initial_layer_thickness)
                {
                    slice_layer_count
                        = std::max(slice_layer_count, (mesh_height > 0) ? 1 : 0); // If less than the initial layer height, it always has 1 layer unless the height is truly zero.
                }
                else
                {
                    slice_layer_count = std::max(slice_layer_count, static_cast<int>(ceil_divide_signed(mesh_height - initial_layer_thickness, layer_thickness) + 1));
                }
                break;
            default:
                spdlog::error("Unknown slicing tolerance. Did you forget to add a case here?");
                return false;
            }
        }
    }

    // Model is shallower than layer_height_0, so not even the first layer is sliced. Return an empty model then.
    if (slice_layer_count <= 0)
    {
        return true; // This is NOT an error state!
    }

    std::vector<Slicer*> slicerList;
    for (unsigned int mesh_idx = 0; mesh_idx < meshgroup->meshes.size(); mesh_idx++)
    {
        // Check if adaptive layers is populated to prevent accessing a method on NULL
        std::vector<AdaptiveLayer>* adaptive_layer_height_values = {};
        if (adaptive_layer_heights != nullptr)
        {
            adaptive_layer_height_values = adaptive_layer_heights->getLayers();
        }

        Mesh& mesh = meshgroup->meshes[mesh_idx];

        const SlicingTolerance slicing_tolerance = mesh.settings_.get<SlicingTolerance>("slicing_tolerance");

        Slicer* slicer
            = new Slicer(&mesh, layer_thickness, slice_layer_count, use_variable_layer_heights, adaptive_layer_height_values, slicing_tolerance, initial_layer_thickness);

        slicerList.push_back(slicer);

        Progress::messageProgress(Progress::Stage::SLICING, mesh_idx + 1, meshgroup->meshes.size());
    }

    // Clear the mesh face and vertex data, it is no longer needed after this point, and it saves a lot of memory.
    meshgroup->clear();

    Mold::process(slicerList);

    const Scene& scene = Application::getInstance().current_slice_->scene;
    for (unsigned int mesh_idx = 0; mesh_idx < slicerList.size(); mesh_idx++)
    {
        const Mesh& mesh = scene.current_mesh_group->meshes[mesh_idx];
        if (mesh.settings_.get<bool>("conical_overhang_enabled") && ! mesh.settings_.get<bool>("anti_overhang_mesh"))
        {
            ConicalOverhang::apply(slicerList[mesh_idx], mesh);
        }
    }

    MultiVolumes::carveCuttingMeshes(slicerList, scene.current_mesh_group->meshes);

    Progress::messageProgressStage(Progress::Stage::PARTS, &timeKeeper);

    if (scene.current_mesh_group->settings.get<bool>("carve_multiple_volumes"))
    {
        carveMultipleVolumes(slicerList);
    }

    generateMultipleVolumesOverlap(slicerList);


    if (Application::getInstance().current_slice_->scene.current_mesh_group->settings.get<bool>("interlocking_enable"))
    {
        InterlockingGenerator::generateInterlockingStructure(slicerList);
    }

    storage.print_layer_count = 0;
    for (unsigned int meshIdx = 0; meshIdx < slicerList.size(); meshIdx++)
    {
        const Mesh& mesh = scene.current_mesh_group->meshes[meshIdx];
        Slicer* slicer = slicerList[meshIdx];
        if (! mesh.settings_.get<bool>("anti_overhang_mesh") && ! mesh.settings_.get<bool>("infill_mesh") && ! mesh.settings_.get<bool>("cutting_mesh"))
        {
            storage.print_layer_count = std::max(storage.print_layer_count, slicer->layers.size());
        }
    }
    storage.support.supportLayers.resize(storage.print_layer_count);

    storage.meshes.reserve(
        slicerList.size()); // causes there to be no resize in meshes so that the pointers in sliceMeshStorage._config to retraction_config don't get invalidated.
    for (unsigned int meshIdx = 0; meshIdx < slicerList.size(); meshIdx++)
    {
        Slicer* slicer = slicerList[meshIdx];
        const Mesh& mesh = scene.current_mesh_group->meshes[meshIdx];

        // always make a new SliceMeshStorage, so that they have the same ordering / indexing as meshgroup.meshes
        storage.meshes.push_back(std::make_shared<SliceMeshStorage>(&meshgroup->meshes[meshIdx], slicer->layers.size())); // new mesh in storage had settings from the Mesh
        SliceMeshStorage& meshStorage = *storage.meshes.back();

        // only create layer parts for normal meshes
        const bool is_support_modifier = AreaSupport::handleSupportModifierMesh(storage, mesh.settings_, slicer);
        if (! is_support_modifier)
        {
            createLayerParts(meshStorage, slicer);
        }

        // Do not add and process support _modifier_ meshes further, and ONLY skip support _modifiers_. They have been
        // processed in AreaSupport::handleSupportModifierMesh(), but other helper meshes such as infill meshes are
        // processed in a later stage, except for support mesh itself, so an exception is made for that.
        if (is_support_modifier && ! mesh.settings_.get<bool>("support_mesh"))
        {
            storage.meshes.pop_back();
            continue;
        }

        // check one if raft offset is needed
        const bool has_raft = mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT;

        // calculate the height at which each layer is actually printed (printZ)
        for (LayerIndex layer_nr = 0; layer_nr < meshStorage.layers.size(); layer_nr++)
        {
            SliceLayer& layer = meshStorage.layers[layer_nr];
            const SlicerLayer& slicer_layer = slicer->layers[layer_nr];
            if (slicer_layer.sliced_uv_coordinates_ && mesh.texture_ && mesh.texture_data_mapping_)
            {
                layer.texture_data_provider_ = std::make_shared<TextureDataProvider>(slicer_layer.sliced_uv_coordinates_, mesh.texture_, mesh.texture_data_mapping_);
            }

            if (use_variable_layer_heights)
            {
                meshStorage.layers[layer_nr].printZ = adaptive_layer_heights->getLayers()->at(layer_nr).z_position_;
                meshStorage.layers[layer_nr].thickness = adaptive_layer_heights->getLayers()->at(layer_nr).layer_height_;
            }
            else
            {
                meshStorage.layers[layer_nr].printZ = initial_layer_thickness + (layer_nr * layer_thickness);

                if (layer_nr == 0)
                {
                    meshStorage.layers[layer_nr].thickness = initial_layer_thickness;
                }
                else
                {
                    meshStorage.layers[layer_nr].thickness = layer_thickness;
                }
            }

            // add the raft offset to each layer
            if (has_raft)
            {
                const ExtruderTrain& train = mesh_group_settings.get<ExtruderTrain&>("raft_surface_extruder_nr");
                layer.printZ += Raft::getTotalThickness() + train.settings_.get<coord_t>("raft_airgap")
                              - train.settings_.get<coord_t>("layer_0_z_overlap"); // shift all layers (except 0) down

                if (layer_nr == 0)
                {
                    layer.printZ += train.settings_.get<coord_t>("layer_0_z_overlap"); // undo shifting down of first layer
                }
            }
        }

        delete slicerList[meshIdx];

        Progress::messageProgress(Progress::Stage::PARTS, meshIdx + 1, slicerList.size());
    }
    return true;
}

void FffPolygonGenerator::slices2polygons(SliceDataStorage& storage, TimeKeeper& time_keeper)
{
    // compute layer count and remove first empty layers
    // there is no separate progress stage for removeEmptyFirstLayer (TODO)
    unsigned int slice_layer_count = 0;
    for (std::shared_ptr<SliceMeshStorage>& mesh_ptr : storage.meshes)
    {
        auto& mesh = *mesh_ptr;
        if (! mesh.settings.get<bool>("infill_mesh") && ! mesh.settings.get<bool>("anti_overhang_mesh"))
        {
            slice_layer_count = std::max<unsigned int>(slice_layer_count, mesh.layers.size());
        }
    }

    // handle meshes
    std::vector<double> mesh_timings;
    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        mesh_timings.push_back(1.0); // TODO: have a more accurate estimate of the relative time it takes per mesh, based on the height and number of polygons
    }
    ProgressStageEstimator inset_skin_progress_estimate(mesh_timings);

    Progress::messageProgressStage(Progress::Stage::INSET_SKIN, &time_keeper);
    std::vector<size_t> mesh_order;
    { // compute mesh order
        std::multimap<int, size_t> order_to_mesh_indices;
        for (size_t mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
        {
            order_to_mesh_indices.emplace(storage.meshes[mesh_idx]->settings.get<int>("infill_mesh_order"), mesh_idx);
        }
        for (std::pair<const int, size_t>& order_and_mesh_idx : order_to_mesh_indices)
        {
            mesh_order.push_back(order_and_mesh_idx.second);
        }
    }
    for (size_t mesh_order_idx = 0; mesh_order_idx < mesh_order.size(); ++mesh_order_idx)
    {
        processBasicWallsSkinInfill(storage, mesh_order_idx, mesh_order, inset_skin_progress_estimate);
        Progress::messageProgress(Progress::Stage::INSET_SKIN, mesh_order_idx + 1, storage.meshes.size());
    }

    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;

    // we need to remove empty layers after we have processed the insets
    // processInsets might throw away parts if they have no wall at all (cause it doesn't fit)
    // brim depends on the first layer not being empty
    // only remove empty layers if we haven't generate support, because then support was added underneath the model.
    //   for some materials it's better to print on support than on the build plate.
    const auto has_support = mesh_group_settings.get<bool>("support_enable") || mesh_group_settings.get<bool>("support_mesh");
    const auto remove_empty_first_layers = mesh_group_settings.get<bool>("remove_empty_first_layers") && ! has_support;
    if (remove_empty_first_layers)
    {
        removeEmptyFirstLayers(storage, storage.print_layer_count); // changes storage.print_layer_count!
    }
    if (storage.print_layer_count == 0)
    {
        spdlog::warn("Stopping process because there are no non-empty layers.");
        return;
    }

    Progress::messageProgressStage(Progress::Stage::SUPPORT, &time_keeper);

    AreaSupport::generateOverhangAreas(storage);
    AreaSupport::generateSupportAreas(storage);
    TreeSupport tree_support_generator(storage);
    tree_support_generator.generateSupportAreas(storage);

    // Pre-compute lightning fill
    if (mesh_group_settings.get<coord_t>("support_line_distance") > 0 && mesh_group_settings.get<EFillMethod>("support_pattern") == EFillMethod::LIGHTNING)
    {
        storage.support.lightning_generator = std::make_shared<LightningGenerator>(storage.support);
    }

    computePrintHeightStatistics(storage);

    // handle helpers
    storage.initializePrimeTower();

    spdlog::debug("Processing ooze shield");
    processOozeShield(storage);

    spdlog::debug("Processing draft shield");
    processDraftShield(storage);

    // This catches a special case in which the models are in the air, and then
    // the adhesion mustn't be calculated.
    if (! isEmptyLayer(storage, 0) || storage.prime_tower_)
    {
        spdlog::debug("Processing platform adhesion");
        processPlatformAdhesion(storage);
    }

    spdlog::debug("Meshes post-processing");
    // meshes post processing
    for (std::shared_ptr<SliceMeshStorage>& mesh : storage.meshes)
    {
        processDerivedWallsSkinInfill(*mesh);
    }

    spdlog::debug("Processing gradual support");
    // generate gradual support
    AreaSupport::generateSupportInfillFeatures(storage);
}

void FffPolygonGenerator::processBasicWallsSkinInfill(
    SliceDataStorage& storage,
    const size_t mesh_order_idx,
    const std::vector<size_t>& mesh_order,
    ProgressStageEstimator& inset_skin_progress_estimate)
{
    size_t mesh_idx = mesh_order[mesh_order_idx];
    SliceMeshStorage& mesh = *storage.meshes[mesh_idx];
    size_t mesh_layer_count = mesh.layers.size();
    if (mesh.settings.get<bool>("infill_mesh"))
    {
        processInfillMesh(storage, mesh_order_idx, mesh_order);
    }

    // TODO: make progress more accurate!!
    // note: estimated time for     insets : skins = 22.953 : 48.858
    std::vector<double> walls_vs_skin_timing({ 22.953, 48.858 });
    ProgressStageEstimator* mesh_inset_skin_progress_estimator = new ProgressStageEstimator(walls_vs_skin_timing);

    inset_skin_progress_estimate.nextStage(mesh_inset_skin_progress_estimator); // the stage of this function call

    ProgressEstimatorLinear* inset_estimator = new ProgressEstimatorLinear(mesh_layer_count);
    mesh_inset_skin_progress_estimator->nextStage(inset_estimator);

    struct
    {
        ProgressStageEstimator& progress_estimator;
        std::mutex mutex{};
        std::atomic<size_t> processed_layer_count = 0;

        void operator++(int)
        {
            std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
            if (lock)
            { // progress estimation is done only in one thread so that no two threads message progress at the same time
                size_t processed_layer_count_ = processed_layer_count.fetch_add(1, std::memory_order_relaxed);
                double progress = progress_estimator.progress(processed_layer_count_);
                Progress::messageProgress(Progress::Stage::INSET_SKIN, progress * 100, 100);
            }
            else
            {
                processed_layer_count.fetch_add(1, std::memory_order_release);
            }
        }
        void reset()
        {
            processed_layer_count.store(0, std::memory_order_relaxed);
        }
    } guarded_progress = { inset_skin_progress_estimate };

    // FeatherPrint pre-pass: accumulate helix phase sequentially so each layer has
    // the exact integral of dz/arc_total rather than the instantaneous approximation.
    // Must run before the parallel wall loop; only populated when the pattern is active.
    if (mesh.settings.get<EFillMethod>("infill_pattern") == EFillMethod::FEATHERPRINT)
    {
        // Force deterministic, strictly inner-to-outer wall printing order for this mesh.
        // The Miter weld structurally requires the OML (outer Wall, inset_idx 0) to print
        // strictly last so its Terminal sweeps over the already-placed inner Walls' open
        // ends — this can't be left to a user/profile preference. A `value` formula in
        // fdmprinter.def.json isn't enough: any quality/material/user profile that stores an
        // explicit value for these settings (optimize_wall_printing_order defaults to true in
        // virtually every stock quality profile) sits above the definition's own computed
        // value in Cura's settings resolution stack. Settings::add() inserts directly into
        // this mesh's own container, which is checked first, so it reliably wins.
        mesh.settings.add("optimize_wall_printing_order", "false");
        mesh.settings.add("inset_direction", "inside_out");
        mesh.settings.add("initial_layer_inset_direction", "inside_out");

        const double alpha_rad = mesh.settings.get<double>("featherprint_helix_angle") * std::numbers::pi / 180.0;
        const double tan_alpha = std::tan(alpha_rad);
        const coord_t fp_w = mesh.settings.get<coord_t>("featherprint_line_width");
        mesh.fp_helix_phase.resize(mesh_layer_count, 0.0);
        mesh.fp_phase_origin.resize(mesh_layer_count, Point2LL(0, 0));
        double phase = 0.0;
        Point2LL fp_prev_origin{};
        bool fp_have_prev_origin = false;
        for (size_t layer_nr = 0; layer_nr < mesh_layer_count; layer_nr++)
        {
            mesh.fp_helix_phase[layer_nr] = std::fmod(phase, 1.0);
            const SliceLayer& layer = mesh.layers[layer_nr];

            // Compute perimeter reference from the largest closed polygon part.
            double best_area    = 0.0;
            double arc_total_mm = 0.0;
            // Anchor Distribution (Spec REV 2.2): the outer wall's own point set for this
            // layer, gathered here (closed-polygon case below, open-manifold virtual-ring
            // case further down) so the one-time seed-point capture after this block can use
            // it regardless of which case applied — see the seeding block below.
            std::vector<Point2LL> fp_seed_ring_pts;
            for (const SliceLayerPart& part : layer.parts)
            {
                for (const Polygon& poly : part.outline)
                {
                    double a = std::abs(poly.area());
                    if (a > best_area)
                    {
                        best_area = a;
                        double len = 0.0;
                        for (size_t i = 0; i < poly.size(); i++)
                        {
                            const Point2LL& p0 = poly[i];
                            const Point2LL& p1 = poly[(i + 1) % poly.size()];
                            double dx = p1.X - p0.X, dy = p1.Y - p0.Y;
                            len += std::sqrt(dx * dx + dy * dy);
                        }
                        arc_total_mm = len / 1000.0;
                        fp_seed_ring_pts.assign(poly.begin(), poly.end());
                    }
                }
            }

            // Open-manifold layers: build the same full virtual ring that WallsComputation
            // and generateOpen use so the helix phase advances by the correct perimeter.
            // All arc fragments are concatenated in angular order (sorted by start angle from
            // the layer centroid); gap chords between consecutive arcs and the closing chord
            // are implicit polygon segments, matching the full_ring_total in generateOpen.
            if (arc_total_mm < 1e-6 && ! layer.open_polylines.empty())
            {
                // Centroid from bounding box of all open polylines
                coord_t bbx0{}, bbx1{}, bby0{}, bby1{};
                bool first = true;
                for (const OpenPolyline& poly : layer.open_polylines)
                    for (const Point2LL& p : poly)
                    {
                        if (first) { bbx0 = bbx1 = p.X; bby0 = bby1 = p.Y; first = false; }
                        if (p.X < bbx0) bbx0 = p.X; if (p.X > bbx1) bbx1 = p.X;
                        if (p.Y < bby0) bby0 = p.Y; if (p.Y > bby1) bby1 = p.Y;
                    }
                const Point2LL cx((bbx0 + bbx1) / 2, (bby0 + bby1) / 2);

                // Orient each arc (CCW in math = positive virtual-closed area) so that
                // poly[0] is the correct start point for angle-sorting and chord measurement.
                // Without this, reversed arcs sort by the wrong endpoint, producing wrong
                // inter-arc chord lengths and an inflated full_ring_total.
                struct OrientedRef
                {
                    std::vector<Point2LL> pts; // oriented points (may be reversed copy)
                    double angle{};
                };
                std::vector<OrientedRef> refs;
                for (const OpenPolyline& poly : layer.open_polylines)
                {
                    if (poly.size() < 2) continue;
                    // Build virtual closed polygon to check winding
                    double area2 = 0.0;
                    for (size_t k = 0; k < poly.size(); k++)
                    {
                        size_t j = (k + 1) % poly.size();
                        area2 += static_cast<double>(poly[k].X) * poly[j].Y
                               - static_cast<double>(poly[j].X) * poly[k].Y;
                    }
                    OrientedRef ref;
                    ref.pts.resize(poly.size());
                    if (area2 < 0.0)
                        std::reverse_copy(poly.begin(), poly.end(), ref.pts.begin());
                    else
                        std::copy(poly.begin(), poly.end(), ref.pts.begin());
                    double dx = ref.pts[0].X - cx.X, dy = ref.pts[0].Y - cx.Y;
                    ref.angle = std::atan2(dy, dx);
                    refs.push_back(std::move(ref));
                }
                std::sort(refs.begin(), refs.end(), [](const OrientedRef& a, const OrientedRef& b){ return a.angle < b.angle; });

                // Sum all arc segment lengths; inter-arc and closing chords are added via
                // the last-to-next-first distances, matching the implicit polygon edges.
                double total = 0.0;
                for (size_t i = 0; i < refs.size(); i++)
                {
                    const auto& pts = refs[i].pts;
                    for (size_t k = 0; k + 1 < pts.size(); k++)
                    {
                        double dx = pts[k + 1].X - pts[k].X, dy = pts[k + 1].Y - pts[k].Y;
                        total += std::sqrt(dx * dx + dy * dy);
                    }
                    // chord to next arc's start (or back to first arc's start for the last)
                    const auto& next_pts = refs[(i + 1) % refs.size()].pts;
                    double cdx = next_pts[0].X - pts.back().X;
                    double cdy = next_pts[0].Y - pts.back().Y;
                    total += std::sqrt(cdx * cdx + cdy * cdy);
                }
                arc_total_mm = total / 1000.0;

                fp_seed_ring_pts.clear();
                for (const OrientedRef& ref : refs)
                    fp_seed_ring_pts.insert(fp_seed_ring_pts.end(), ref.pts.begin(), ref.pts.end());
            }

            if (arc_total_mm > 1e-6)
            {
                const double layer_height_mm = static_cast<double>(layer.printZ) / 1000.0
                    - (layer_nr > 0 ? static_cast<double>(mesh.layers[layer_nr - 1].printZ) / 1000.0 : 0.0);
                phase += layer_height_mm / (arc_total_mm * tan_alpha);
            }

            // Anchor Distribution (Spec REV 2.2, continuity-tracking variant — see
            // fp_phase_origin's declaration for why this deviates from the spec's literal
            // fixed-point design). "Layer 0" is the first layer whose largest outline passes
            // the same size guard generate() itself uses (arc.total < 4.0 * w returns early
            // there); every layer from there up walks its own Phase Origin forward.
            if (arc_total_mm >= 4.0 * (static_cast<double>(fp_w) / 1000.0) && fp_seed_ring_pts.size() >= 2)
            {
                Point2LL origin{};
                if (! fp_have_prev_origin)
                {
                    // Seed: the point on this layer's outer wall at world x=0 furthest in +Y.
                    bool found = false;
                    double best_y = 0.0;
                    const size_t n = fp_seed_ring_pts.size();
                    for (size_t i = 0; i < n; i++)
                    {
                        const Point2LL& a = fp_seed_ring_pts[i];
                        const Point2LL& b = fp_seed_ring_pts[(i + 1) % n];
                        // Segment crosses world x=0
                        if ((a.X <= 0 && b.X > 0) || (a.X > 0 && b.X <= 0))
                        {
                            double t = static_cast<double>(-a.X) / static_cast<double>(b.X - a.X);
                            double y = a.Y + t * (b.Y - a.Y);
                            if (! found || y > best_y)
                            {
                                found  = true;
                                best_y = y;
                                origin = Point2LL(0, static_cast<coord_t>(std::llround(y)));
                            }
                        }
                    }
                    // Degenerate fallback: the outline never crosses world x=0 at all (the
                    // part doesn't straddle the build plate's X origin). Not addressed by the
                    // spec — fall back to the outline's own topmost point so seeding succeeds.
                    if (! found)
                    {
                        origin = fp_seed_ring_pts[0];
                        for (const Point2LL& p : fp_seed_ring_pts)
                            if (p.Y > origin.Y)
                                origin = p;
                    }
                }
                else
                {
                    // Walk: nearest point on THIS layer's outer wall to the PREVIOUS layer's
                    // own Phase Origin (world-space point-to-segment min-distance scan) — see
                    // fp_phase_origin's declaration for why this replaces projecting against
                    // one fixed point every layer.
                    double best_d2 = -1.0;
                    const size_t n = fp_seed_ring_pts.size();
                    for (size_t i = 0; i < n; i++)
                    {
                        const Point2LL& a = fp_seed_ring_pts[i];
                        const Point2LL& b = fp_seed_ring_pts[(i + 1) % n];
                        double ax = static_cast<double>(a.X), ay = static_cast<double>(a.Y);
                        double ex = static_cast<double>(b.X - a.X), ey = static_cast<double>(b.Y - a.Y);
                        double seg_len2 = ex * ex + ey * ey;
                        double t = (seg_len2 > 1e-9)
                            ? ((static_cast<double>(fp_prev_origin.X) - ax) * ex + (static_cast<double>(fp_prev_origin.Y) - ay) * ey) / seg_len2
                            : 0.0;
                        t = std::max(0.0, std::min(1.0, t));
                        double px = ax + t * ex, py = ay + t * ey;
                        double dx = static_cast<double>(fp_prev_origin.X) - px;
                        double dy = static_cast<double>(fp_prev_origin.Y) - py;
                        double d2 = dx * dx + dy * dy;
                        if (best_d2 < 0.0 || d2 < best_d2)
                        {
                            best_d2 = d2;
                            origin  = Point2LL(static_cast<coord_t>(std::llround(px)), static_cast<coord_t>(std::llround(py)));
                        }
                    }
                }
                mesh.fp_phase_origin[layer_nr] = origin;
                fp_prev_origin      = origin;
                fp_have_prev_origin = true;
            }
            else if (fp_have_prev_origin)
            {
                // No valid outer-wall points this layer (e.g. a momentary gap) — carry the
                // previous origin forward unchanged rather than leaving a (0,0) hole in the walk.
                mesh.fp_phase_origin[layer_nr] = fp_prev_origin;
            }
        }

        // Flange: unconditionally spans featherprint_flange_ramp_layers layers down from the
        // topmost layer with any boundary geometry (closed parts OR open_polylines — a
        // boundary loop interrupted by a slot/hole reaching the open top is represented
        // entirely as open_polylines at those layers, the same container the ordinary
        // single-slot Whip case already uses). No attempt is made to distinguish an open top
        // from a closed apex/taper here — that heuristic proved unreliable (it either misses
        // an open top interrupted by slots, since those top layers have no `parts` to measure
        // an area from, or misfires on an ordinary closed taper). Models with a genuinely
        // closed top (e.g. a nose cone) must instead disable the Flange explicitly via
        // featherprint_flange_enabled.
        if (mesh.settings.get<bool>("featherprint_flange_enabled"))
        {
            const size_t n_flange = mesh.settings.get<size_t>("featherprint_flange_ramp_layers");
            LayerIndex last_geom = -1;
            for (int li = static_cast<int>(mesh_layer_count) - 1; li >= 0; li--)
            {
                if (! mesh.layers[li].parts.empty() || ! mesh.layers[li].open_polylines.empty())
                {
                    last_geom = static_cast<LayerIndex>(li);
                    break;
                }
            }
            if (last_geom >= 0)
            {
                mesh.fp_flange_start_layer = static_cast<LayerIndex>(
                    std::max(LayerIndex(0), last_geom - static_cast<LayerIndex>(n_flange) + 1));
            }
        }
    }

    // walls
    cura::parallel_for<size_t>(
        0,
        mesh_layer_count,
        [&](size_t layer_number)
        {
            spdlog::debug("Processing insets for layer {} of {}", layer_number, mesh.layers.size());
            processWalls(mesh, layer_number);
            guarded_progress++;
        });

    ProgressEstimatorLinear* skin_estimator = new ProgressEstimatorLinear(mesh_layer_count);
    mesh_inset_skin_progress_estimator->nextStage(skin_estimator);

    bool process_infill = mesh.settings.get<coord_t>("infill_line_distance") > 0;
    if (! process_infill)
    { // do process infill anyway if it's modified by modifier meshes
        const Scene& scene = Application::getInstance().current_slice_->scene;
        for (size_t other_mesh_order_idx = mesh_order_idx + 1; other_mesh_order_idx < mesh_order.size(); ++other_mesh_order_idx)
        {
            const size_t other_mesh_idx = mesh_order[other_mesh_order_idx];
            SliceMeshStorage& other_mesh = *storage.meshes[other_mesh_idx];
            if (other_mesh.settings.get<bool>("infill_mesh"))
            {
                AABB3D aabb = scene.current_mesh_group->meshes[mesh_idx].getAABB();
                AABB3D other_aabb = scene.current_mesh_group->meshes[other_mesh_idx].getAABB();
                if (aabb.hit(other_aabb))
                {
                    process_infill = true;
                }
            }
        }
    }

    // skin & infill
    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    bool magic_spiralize = mesh_group_settings.get<bool>("magic_spiralize");
    size_t mesh_max_initial_bottom_layer_count = 0;
    if (magic_spiralize)
    {
        mesh_max_initial_bottom_layer_count = std::max(mesh_max_initial_bottom_layer_count, mesh.settings.get<size_t>("initial_bottom_layers"));
    }

    guarded_progress.reset();
    cura::parallel_for<size_t>(
        0,
        mesh_layer_count,
        [&](size_t layer_number)
        {
            spdlog::debug("Processing skins and infill layer {} of {}", layer_number, mesh.layers.size());
            if (! magic_spiralize || layer_number < mesh_max_initial_bottom_layer_count) // Only generate up/downskin and infill for the first X layers when spiralize is choosen.
            {
                processSkinsAndInfill(mesh, layer_number, process_infill);
            }
            guarded_progress++;
        });
}

void FffPolygonGenerator::processInfillMesh(SliceDataStorage& storage, const size_t mesh_order_idx, const std::vector<size_t>& mesh_order)
{
    size_t mesh_idx = mesh_order[mesh_order_idx];
    SliceMeshStorage& mesh = *storage.meshes[mesh_idx];
    coord_t surface_line_width = mesh.settings.get<coord_t>("wall_line_width_0");

    mesh.layer_nr_max_filled_layer = -1;
    for (LayerIndex layer_idx = 0; layer_idx < LayerIndex(mesh.layers.size()); layer_idx++)
    {
        SliceLayer& layer = mesh.layers[layer_idx];

        if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)
        {
            // break up polygons into polylines
            // they have to be polylines, because they might break up further when doing the cutting
            for (SliceLayerPart& part : layer.parts)
            {
                for (const Polygon& poly : part.outline)
                {
                    layer.open_polylines.push_back(poly.toPseudoOpenPolyline());
                }
            }
            layer.parts.clear();
        }

        std::vector<SingleShape> new_parts;
        OpenLinesSet new_polylines;

        for (const size_t other_mesh_idx : mesh_order)
        { // limit the infill mesh's outline to within the infill of all meshes with lower order
            if (other_mesh_idx == mesh_idx)
            {
                break; // all previous meshes have been processed
            }
            SliceMeshStorage& other_mesh = *storage.meshes[other_mesh_idx];
            if (layer_idx >= LayerIndex(other_mesh.layers.size()))
            { // there can be no interaction between the infill mesh and this other non-infill mesh
                continue;
            }

            SliceLayer& other_layer = other_mesh.layers[layer_idx];

            for (SliceLayerPart& other_part : other_layer.parts)
            {
                if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::SURFACE)
                {
                    for (SliceLayerPart& part : layer.parts)
                    { // limit the outline of each part of this infill mesh to the infill of parts of the other mesh with lower infill mesh order
                        if (! part.boundaryBox.hit(other_part.boundaryBox))
                        { // early out
                            continue;
                        }
                        Shape new_outline = part.outline.intersection(other_part.getOwnInfillArea());
                        if (new_outline.size() == 1)
                        { // we don't have to call splitIntoParts, because a single polygon can only be a single part
                            SingleShape outline_part_here;
                            outline_part_here.push_back(new_outline[0]);
                            new_parts.push_back(outline_part_here);
                        }
                        else if (new_outline.size() > 1)
                        { // we don't know whether it's a multitude of parts because of newly introduced holes, or because the polygon has been split up
                            std::vector<SingleShape> new_parts_here = new_outline.splitIntoParts();
                            for (SingleShape& new_part_here : new_parts_here)
                            {
                                new_parts.push_back(new_part_here);
                            }
                        }
                        // change the infill area of the non-infill mesh which is to be filled with e.g. lines
                        other_part.infill_area_own = other_part.getOwnInfillArea().difference(part.outline);
                        // note: don't change the part.infill_area, because we change the structure of that area, while the basic area in which infill is printed remains the same
                        //       the infill area remains the same for combing
                    }
                }
                if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
                {
                    const Shape& own_infill_area = other_part.getOwnInfillArea();
                    OpenLinesSet cut_lines = own_infill_area.intersection(layer.open_polylines);
                    new_polylines.push_back(cut_lines);
                    // NOTE: closed polygons will be represented as polylines, which will be closed automatically in the PathOrderOptimizer
                    if (! own_infill_area.empty())
                    {
                        other_part.infill_area_own = own_infill_area.difference(layer.open_polylines.offset(surface_line_width / 2));
                    }
                }
            }
        }

        layer.parts.clear();
        for (SingleShape& part : new_parts)
        {
            if (part.empty())
            {
                continue;
            }
            layer.parts.emplace_back();
            layer.parts.back().outline = part;
            layer.parts.back().boundaryBox.calculate(part);
        }

        if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
        {
            layer.open_polylines = new_polylines;
        }

        if (layer.parts.size() > 0 || (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer.open_polylines.size() > 0))
        {
            mesh.layer_nr_max_filled_layer = layer_idx; // last set by the highest non-empty layer
        }
    }
}

void FffPolygonGenerator::processDerivedWallsSkinInfill(SliceMeshStorage& mesh)
{
    if (mesh.settings.get<bool>("infill_support_enabled"))
    { // create gradual infill areas
        SkinInfillAreaComputation::generateInfillSupport(mesh);
    }

    // create gradual infill areas
    SkinInfillAreaComputation::generateGradualInfill(mesh);

    // SubDivCube Pre-compute Octree
    if (mesh.settings.get<coord_t>("infill_line_distance") > 0 && mesh.settings.get<EFillMethod>("infill_pattern") == EFillMethod::CUBICSUBDIV)
    {
        const Point3LL mesh_middle = mesh.bounding_box.getMiddle();
        const Point2LL infill_origin(mesh_middle.x_ + mesh.settings.get<coord_t>("infill_offset_x"), mesh_middle.y_ + mesh.settings.get<coord_t>("infill_offset_y"));
        SubDivCube::precomputeOctree(mesh, infill_origin);
    }

    // Pre-compute Cross Fractal
    if (mesh.settings.get<coord_t>("infill_line_distance") > 0
        && (mesh.settings.get<EFillMethod>("infill_pattern") == EFillMethod::CROSS || mesh.settings.get<EFillMethod>("infill_pattern") == EFillMethod::CROSS_3D))
    {
        const std::string cross_subdivision_spec_image_file = mesh.settings.get<std::string>("cross_infill_density_image");
        std::ifstream cross_fs(cross_subdivision_spec_image_file.c_str());
        if (! cross_subdivision_spec_image_file.empty() && cross_fs.good())
        {
            mesh.cross_fill_provider = std::make_shared<SierpinskiFillProvider>(
                mesh.bounding_box,
                mesh.settings.get<coord_t>("infill_line_distance"),
                mesh.settings.get<coord_t>("infill_line_width"),
                cross_subdivision_spec_image_file);
        }
        else
        {
            if (! cross_subdivision_spec_image_file.empty() && cross_subdivision_spec_image_file != " ")
            {
                spdlog::error("Cannot find density image: {}.", cross_subdivision_spec_image_file);
            }
            mesh.cross_fill_provider
                = std::make_shared<SierpinskiFillProvider>(mesh.bounding_box, mesh.settings.get<coord_t>("infill_line_distance"), mesh.settings.get<coord_t>("infill_line_width"));
        }
    }

    // Pre-compute lightning fill (aka minfill, aka ribbed support vaults)
    if (mesh.settings.get<coord_t>("infill_line_distance") > 0 && mesh.settings.get<EFillMethod>("infill_pattern") == EFillMethod::LIGHTNING)
    {
        // TODO: Make all of these into new type pointers (but the cross fill things need to happen too then, otherwise it'd just look weird).
        mesh.lightning_generator = std::make_shared<LightningGenerator>(mesh);
    }

    // combine infill
    SkinInfillAreaComputation::combineInfillLayers(mesh);

    // Fuzzy skin. Disabled when using interlocking structures, the internal interlocking walls become fuzzy.
    if (mesh.settings.get<bool>("magic_fuzzy_skin_enabled") && ! mesh.settings.get<bool>("interlocking_enable"))
    {
        processFuzzyWalls(mesh);
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * processInsets only reads and writes data for the current layer
 */
void FffPolygonGenerator::processWalls(SliceMeshStorage& mesh, size_t layer_nr)
{
    SliceLayer* layer = &mesh.layers[layer_nr];
    const double fp_phase = (layer_nr < mesh.fp_helix_phase.size()) ? mesh.fp_helix_phase[layer_nr] : 0.0;
    const Point2LL fp_phase_origin = (layer_nr < mesh.fp_phase_origin.size()) ? mesh.fp_phase_origin[layer_nr] : Point2LL(0, 0);
    WallsComputation walls_computation(mesh.settings, layer_nr, fp_phase, mesh.fp_flange_start_layer, fp_phase_origin);
    walls_computation.generateWalls(layer, SectionType::WALL);
}

bool FffPolygonGenerator::isEmptyLayer(SliceDataStorage& storage, const LayerIndex& layer_idx)
{
    if (storage.support.generated && layer_idx < storage.support.supportLayers.size())
    {
        SupportLayer& support_layer = storage.support.supportLayers[layer_idx];
        if (! support_layer.support_infill_parts.empty() || ! support_layer.support_bottom.empty() || ! support_layer.support_roof.empty())
        {
            return false;
        }
    }
    for (std::shared_ptr<SliceMeshStorage>& mesh_ptr : storage.meshes)
    {
        auto& mesh = *mesh_ptr;
        if (layer_idx >= mesh.layers.size())
        {
            continue;
        }
        SliceLayer& layer = mesh.layers[layer_idx];
        if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer.open_polylines.size() > 0)
        {
            return false;
        }
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.print_outline.size() > 0)
            {
                return false;
            }
        }
    }
    return true;
}

void FffPolygonGenerator::removeEmptyFirstLayers(SliceDataStorage& storage, size_t& total_layers)
{
    size_t n_empty_first_layers = 0;
    coord_t hightest_empty_layer = 0;
    for (size_t layer_idx = 0; layer_idx < total_layers; layer_idx++)
    {
        if (isEmptyLayer(storage, layer_idx))
        {
            n_empty_first_layers++;

            coord_t layer_highest_z = 0;
            for (const std::shared_ptr<SliceMeshStorage>& mesh_ptr : storage.meshes)
            {
                const auto& mesh = *mesh_ptr;
                layer_highest_z = layer_idx >= mesh.layers.size() ? layer_highest_z : std::max(layer_highest_z, mesh.layers[layer_idx].printZ);
            }
            hightest_empty_layer = std::max(hightest_empty_layer, layer_highest_z);
        }
        else
        {
            break;
        }
    }

    if (n_empty_first_layers > 0)
    {
        spdlog::info("Removing {} layers because they are empty", n_empty_first_layers);
        const coord_t layer_height = Application::getInstance().current_slice_->scene.current_mesh_group->settings.get<coord_t>("layer_height");
        for (auto& mesh_ptr : storage.meshes)
        {
            auto& mesh = *mesh_ptr;
            std::vector<SliceLayer>& layers = mesh.layers;
            if (layers.size() > n_empty_first_layers)
            {
                // transfer initial layer thickness to new initial layer
                layers[n_empty_first_layers].thickness = layers[0].thickness;
            }
            layers.erase(layers.begin(), layers.begin() + n_empty_first_layers);
            for (SliceLayer& layer : layers)
            {
                layer.printZ -= hightest_empty_layer;
            }
            mesh.layer_nr_max_filled_layer -= n_empty_first_layers;
        }
        total_layers -= n_empty_first_layers;
        storage.support.layer_nr_max_filled_layer -= n_empty_first_layers;
        std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
        support_layers.erase(support_layers.begin(), support_layers.begin() + n_empty_first_layers);
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateSkins read (depend on) data from mesh.layers[*].parts[*].insets and write mesh.layers[n].parts[*].skin_parts
 * generateInfill read mesh.layers[n].parts[*].{insets,skin_parts,boundingBox} and write mesh.layers[n].parts[*].infill_area
 *
 * processSkinsAndInfill read (depend on) mesh.layers[*].parts[*].{insets,boundingBox}.
 *                       write mesh.layers[n].parts[*].{skin_parts,infill_area}.
 */
void FffPolygonGenerator::processSkinsAndInfill(SliceMeshStorage& mesh, const LayerIndex layer_nr, bool process_infill)
{
    if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)
    {
        return;
    }

    SkinInfillAreaComputation skin_infill_area_computation(layer_nr, mesh, process_infill);
    skin_infill_area_computation.generateSkinsAndInfill();

    if (((mesh.settings.get<bool>("ironing_enabled") && (! mesh.settings.get<bool>("ironing_only_highest_layer"))) || mesh.layer_nr_max_filled_layer == layer_nr)
        || ! mesh.settings.get<bool>("small_skin_on_surface"))
    {
        // Generate the top surface to iron over.
        mesh.layers[layer_nr].top_surface.setAreasFromMeshAndLayerNumber(mesh, layer_nr);
    }

    if (layer_nr >= 0 && ! mesh.settings.get<bool>("small_skin_on_surface"))
    {
        // Generate the bottom surface.
        mesh.layers[layer_nr].bottom_surface = mesh.layers[layer_nr].getOutlines();
        if (layer_nr > 0)
        {
            mesh.layers[layer_nr].bottom_surface = mesh.layers[layer_nr].bottom_surface.difference(mesh.layers[layer_nr - 1].getOutlines());
        }
    }
}

void FffPolygonGenerator::computePrintHeightStatistics(SliceDataStorage& storage)
{
    const size_t extruder_count = Application::getInstance().current_slice_->scene.extruders.size();

    std::vector<int>& max_print_height_per_extruder = storage.max_print_height_per_extruder;
    assert(max_print_height_per_extruder.size() == 0 && "storage.max_print_height_per_extruder shouldn't have been initialized yet!");
    const int raft_layers = Raft::getTotalExtraLayers();
    max_print_height_per_extruder.resize(extruder_count, -(raft_layers + 1)); // Initialize all as -1 (or lower in case of raft).
    { // compute max_object_height_per_extruder
        // Height of the meshes themselves.
        for (std::shared_ptr<SliceMeshStorage>& mesh_ptr : storage.meshes)
        {
            auto& mesh = *mesh_ptr;
            if (mesh.settings.get<bool>("anti_overhang_mesh") || mesh.settings.get<bool>("support_mesh"))
            {
                continue; // Special type of mesh that doesn't get printed.
            }
            for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
            {
                for (LayerIndex layer_nr = LayerIndex(mesh.layers.size()) - 1; layer_nr > max_print_height_per_extruder[extruder_nr]; layer_nr--)
                {
                    if (mesh.getExtruderIsUsed(extruder_nr, layer_nr))
                    {
                        assert(max_print_height_per_extruder[extruder_nr] <= layer_nr);
                        max_print_height_per_extruder[extruder_nr] = layer_nr;
                    }
                }
            }
        }

        // Height of where the support reaches.
        Scene& scene = Application::getInstance().current_slice_->scene;
        const Settings& mesh_group_settings = scene.current_mesh_group->settings;
        const size_t support_infill_extruder_nr
            = mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr_; // TODO: Support extruder should be configurable per object.
        max_print_height_per_extruder[support_infill_extruder_nr] = std::max(max_print_height_per_extruder[support_infill_extruder_nr], storage.support.layer_nr_max_filled_layer);
        const size_t support_roof_extruder_nr
            = mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr_; // TODO: Support roof extruder should be configurable per object.
        max_print_height_per_extruder[support_roof_extruder_nr] = std::max(max_print_height_per_extruder[support_roof_extruder_nr], storage.support.layer_nr_max_filled_layer);
        const size_t support_bottom_extruder_nr
            = mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr_; // TODO: Support bottom extruder should be configurable per object.
        max_print_height_per_extruder[support_bottom_extruder_nr] = std::max(max_print_height_per_extruder[support_bottom_extruder_nr], storage.support.layer_nr_max_filled_layer);

        // Height of where the platform adhesion reaches.
        const EPlatformAdhesion adhesion_type = mesh_group_settings.get<EPlatformAdhesion>("adhesion_type");
        switch (adhesion_type)
        {
        case EPlatformAdhesion::SKIRT:
        case EPlatformAdhesion::BRIM:
        {
            const std::vector<ExtruderTrain*> skirt_brim_extruder_trains = mesh_group_settings.get<std::vector<ExtruderTrain*>>("skirt_brim_extruder_nr");
            for (ExtruderTrain* train : skirt_brim_extruder_trains)
            {
                const size_t skirt_brim_extruder_nr = train->extruder_nr_;
                max_print_height_per_extruder[skirt_brim_extruder_nr] = std::max(0, max_print_height_per_extruder[skirt_brim_extruder_nr]); // Includes layer 0.
            }
            break;
        }
        case EPlatformAdhesion::RAFT:
        {
            const size_t base_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("raft_base_extruder_nr").extruder_nr_;
            max_print_height_per_extruder[base_extruder_nr] = std::max(-raft_layers, max_print_height_per_extruder[base_extruder_nr]); // Includes the lowest raft layer.
            const size_t interface_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("raft_interface_extruder_nr").extruder_nr_;
            max_print_height_per_extruder[interface_extruder_nr]
                = std::max(-raft_layers + 1, max_print_height_per_extruder[interface_extruder_nr]); // Includes the second-lowest raft layer.
            const size_t surface_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("raft_surface_extruder_nr").extruder_nr_;
            max_print_height_per_extruder[surface_extruder_nr]
                = std::max(-1, max_print_height_per_extruder[surface_extruder_nr]); // Includes up to the first layer below the model (so -1).
            break;
        }
        default:
            break; // No adhesion, so no maximum necessary.
        }
    }

    storage.max_print_height_order = order(max_print_height_per_extruder);
    if (extruder_count >= 2)
    {
        int second_highest_extruder = storage.max_print_height_order[extruder_count - 2];
        storage.max_print_height_second_to_last_extruder = max_print_height_per_extruder[second_highest_extruder];
    }
    else
    {
        storage.max_print_height_second_to_last_extruder = -(raft_layers + 1);
    }
}


void FffPolygonGenerator::processOozeShield(SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    if (! mesh_group_settings.get<bool>("ooze_shield_enabled"))
    {
        return;
    }

    const coord_t ooze_shield_dist = mesh_group_settings.get<coord_t>("ooze_shield_dist");

    for (int layer_nr = 0; layer_nr <= storage.max_print_height_second_to_last_extruder; layer_nr++)
    {
        constexpr bool around_support = true;
        constexpr bool around_prime_tower = false;
        storage.ooze_shield.push_back(storage.getLayerOutlines(layer_nr, around_support, around_prime_tower).offset(ooze_shield_dist, ClipperLib::jtRound).getOutsidePolygons());
    }

    const AngleDegrees angle = mesh_group_settings.get<AngleDegrees>("ooze_shield_angle");
    if (angle <= 89)
    {
        const coord_t allowed_angle_offset
            = tan(mesh_group_settings.get<AngleRadians>("ooze_shield_angle")) * mesh_group_settings.get<coord_t>("layer_height"); // Allow for a 60deg angle in the oozeShield.
        for (LayerIndex layer_nr = 1; layer_nr <= storage.max_print_height_second_to_last_extruder; layer_nr++)
        {
            storage.ooze_shield[layer_nr] = storage.ooze_shield[layer_nr].unionPolygons(storage.ooze_shield[layer_nr - 1].offset(-allowed_angle_offset));
        }
        for (LayerIndex layer_nr = storage.max_print_height_second_to_last_extruder; layer_nr > 0; layer_nr--)
        {
            storage.ooze_shield[layer_nr - 1] = storage.ooze_shield[layer_nr - 1].unionPolygons(storage.ooze_shield[layer_nr].offset(-allowed_angle_offset));
        }
    }

    const double largest_printed_area = 1.0; // TODO: make var a parameter, and perhaps even a setting?
    for (LayerIndex layer_nr = 0; layer_nr <= storage.max_print_height_second_to_last_extruder; layer_nr++)
    {
        storage.ooze_shield[layer_nr].removeSmallAreas(largest_printed_area);
    }
    if (storage.prime_tower_)
    {
        coord_t max_line_width = 0;
        { // compute max_line_width
            const std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
            const auto& extruders = Application::getInstance().current_slice_->scene.extruders;
            for (int extruder_nr = 0; extruder_nr < int(extruders.size()); extruder_nr++)
            {
                if (! extruder_is_used[extruder_nr])
                    continue;
                max_line_width = std::max(max_line_width, extruders[extruder_nr].settings_.get<coord_t>("skirt_brim_line_width"));
            }
        }
        for (LayerIndex layer_nr = 0; layer_nr <= storage.max_print_height_second_to_last_extruder; layer_nr++)
        {
            storage.ooze_shield[layer_nr] = storage.ooze_shield[layer_nr].difference(storage.prime_tower_->getOccupiedOutline(layer_nr).offset(max_line_width / 2));
        }
    }
}

void FffPolygonGenerator::processDraftShield(SliceDataStorage& storage)
{
    const size_t draft_shield_layers = getDraftShieldLayerCount(storage.print_layer_count);
    if (draft_shield_layers <= 0)
    {
        return;
    }
    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");

    const LayerIndex layer_skip{ 500 / layer_height + 1 };

    Shape& draft_shield = storage.draft_protection_shield;
    for (LayerIndex layer_nr = 0; layer_nr < storage.print_layer_count && layer_nr < draft_shield_layers; layer_nr += layer_skip)
    {
        constexpr bool around_support = true;
        constexpr bool around_prime_tower = false;
        draft_shield = draft_shield.unionPolygons(storage.getLayerOutlines(layer_nr, around_support, around_prime_tower));
    }

    const coord_t draft_shield_dist = mesh_group_settings.get<coord_t>("draft_shield_dist");
    storage.draft_protection_shield = draft_shield.approxConvexHull(draft_shield_dist);

    // Extra offset has rounded joints, so simplify again.
    coord_t maximum_resolution = 0; // Draft shield is printed with every extruder, so resolve with the max() or min() of them to meet the requirements of all extruders.
    coord_t maximum_deviation = std::numeric_limits<coord_t>::max();
    for (const ExtruderTrain& extruder : Application::getInstance().current_slice_->scene.extruders)
    {
        maximum_resolution = std::max(maximum_resolution, extruder.settings_.get<coord_t>("meshfix_maximum_resolution"));
        maximum_deviation = std::min(maximum_deviation, extruder.settings_.get<coord_t>("meshfix_maximum_deviation"));
    }
    storage.draft_protection_shield = Simplify(maximum_resolution, maximum_deviation, 0).polygon(storage.draft_protection_shield);
    if (storage.prime_tower_)
    {
        coord_t max_line_width = 0;
        { // compute max_line_width
            const std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
            const auto& extruders = Application::getInstance().current_slice_->scene.extruders;
            for (int extruder_nr = 0; extruder_nr < int(extruders.size()); extruder_nr++)
            {
                if (! extruder_is_used[extruder_nr])
                    continue;
                max_line_width = std::max(max_line_width, extruders[extruder_nr].settings_.get<coord_t>("skirt_brim_line_width"));
            }
        }
        storage.draft_protection_shield = storage.draft_protection_shield.difference(storage.prime_tower_->getOccupiedGroundOutline().offset(max_line_width / 2));
    }
}

void FffPolygonGenerator::processPlatformAdhesion(SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    EPlatformAdhesion adhesion_type = mesh_group_settings.get<EPlatformAdhesion>("adhesion_type");

    if (adhesion_type == EPlatformAdhesion::RAFT)
    {
        Raft::generate(storage);
        return;
    }

    SkirtBrim skirt_brim(storage);
    if (adhesion_type != EPlatformAdhesion::NONE)
    {
        skirt_brim.generate();
    }

    if (mesh_group_settings.get<bool>("support_brim_enable"))
    {
        skirt_brim.generateSupportBrim();
    }
}


void FffPolygonGenerator::processFuzzyWalls(SliceMeshStorage& mesh)
{
    if (mesh.settings.get<size_t>("wall_line_count") == 0)
    {
        return;
    }

    const coord_t line_width = mesh.settings.get<coord_t>("line_width");
    const bool apply_outside_only = mesh.settings.get<bool>("magic_fuzzy_skin_outside_only");
    const coord_t fuzziness = mesh.settings.get<coord_t>("magic_fuzzy_skin_thickness");
    const coord_t avg_dist_between_points = mesh.settings.get<coord_t>("magic_fuzzy_skin_point_dist");
    const coord_t min_dist_between_points = avg_dist_between_points * 3 / 4; // hardcoded: the point distance may vary between 3/4 and 5/4 the supplied value
    const coord_t range_random_point_dist = avg_dist_between_points / 2;
    unsigned int start_layer_nr
        = (mesh.settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::BRIM) ? 1 : 0; // don't make fuzzy skin on first layer if there's a brim

    auto hole_area = Shape();
    std::function<bool(const bool&, const ExtrusionJunction&)> accumulate_is_in_hole
        = []([[maybe_unused]] const bool& prev_result, [[maybe_unused]] const ExtrusionJunction& junction)
    {
        return false;
    };

    for (LayerIndex layer_nr = start_layer_nr; layer_nr < LayerIndex(mesh.layers.size()); layer_nr++)
    {
        SliceLayer& layer = mesh.layers[layer_nr];
        for (SliceLayerPart& part : layer.parts)
        {
            std::vector<VariableWidthLines> result_paths;
            for (auto& toolpath : part.wall_toolpaths)
            {
                if (toolpath.front().inset_idx_ != 0)
                {
                    result_paths.push_back(toolpath);
                    continue;
                }

                auto& result_lines = result_paths.emplace_back();

                if (apply_outside_only)
                {
                    hole_area = part.print_outline.getOutsidePolygons().offset(-line_width);
                    accumulate_is_in_hole = [&hole_area](const bool& prev_result, const ExtrusionJunction& junction)
                    {
                        return prev_result || hole_area.inside(junction.p_);
                    };
                }
                for (auto& line : toolpath)
                {
                    if (apply_outside_only && std::accumulate(line.begin(), line.end(), false, accumulate_is_in_hole))
                    {
                        result_lines.push_back(line);
                        continue;
                    }

                    auto& result = result_lines.emplace_back(line.inset_idx_, line.is_odd_, line.is_closed_);

                    // generate points in between p0 and p1
                    int64_t dist_left_over
                        = (min_dist_between_points / 4) + rand() % (min_dist_between_points / 4); // the distance to be traversed on the line before making the first new point
                    auto* p0 = &line.front();
                    for (auto& p1 : line)
                    {
                        if (p0->p_ == p1.p_) // avoid seams
                        {
                            result.emplace_back(p1.p_, p1.w_, p1.perimeter_index_);
                            continue;
                        }

                        // 'a' is the (next) new point between p0 and p1
                        const Point2LL p0p1 = p1.p_ - p0->p_;
                        const int64_t p0p1_size = vSize(p0p1);
                        int64_t p0pa_dist = dist_left_over;
                        if (p0pa_dist >= p0p1_size)
                        {
                            const Point2LL p = p1.p_ - (p0p1 / 2);
                            const double width = (p1.w_ * vSize(p1.p_ - p) + p0->w_ * vSize(p0->p_ - p)) / p0p1_size;
                            result.emplace_back(p, width, p1.perimeter_index_);
                        }
                        for (; p0pa_dist < p0p1_size; p0pa_dist += min_dist_between_points + rand() % range_random_point_dist)
                        {
                            const int r = rand() % (fuzziness * 2) - fuzziness;
                            const Point2LL perp_to_p0p1 = turn90CCW(p0p1);
                            const Point2LL fuzz = normal(perp_to_p0p1, r);
                            const Point2LL pa = p0->p_ + normal(p0p1, p0pa_dist);
                            const double width = (p1.w_ * vSize(p1.p_ - pa) + p0->w_ * vSize(p0->p_ - pa)) / p0p1_size;
                            result.emplace_back(pa + fuzz, width, p1.perimeter_index_);
                        }
                        // p0pa_dist > p0p1_size now because we broke out of the for-loop
                        dist_left_over = p0pa_dist - p0p1_size;

                        p0 = &p1;
                    }
                    while (result.size() < 3)
                    {
                        size_t point_idx = line.size() - 2;
                        result.emplace_back(line[point_idx].p_, line[point_idx].w_, line[point_idx].perimeter_index_);
                        if (point_idx == 0)
                        {
                            break;
                        }
                        point_idx--;
                    }
                    if (result.size() < 3)
                    {
                        result.clear();
                        for (auto& p : line)
                        {
                            result.emplace_back(p.p_, p.w_, p.perimeter_index_);
                        }
                    }
                    if (line.back().p_ == line.front().p_) // avoid seams
                    {
                        result.back().p_ = result.front().p_;
                    }
                }
            }
            part.wall_toolpaths = result_paths;
        }
    }
}


} // namespace cura
