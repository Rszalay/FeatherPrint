// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream> // ifstream.good()
#include <map> // multimap (ordered map allowing duplicate keys)
#include <numbers>
#include <numeric>
#include <optional>

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
#include "featherprint/FeatherPrintGenerator.h"
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

namespace
{
// ============================================================================
// Shore (Internal Overhang Bridging, Spec REV 3.1) -- geometry helpers.
// Deliberately self-contained here (not routed through FeatherPrintGenerator) since Shore, as
// of REV 3.1, has no dependency on Stringer/Lacing/any other print feature at all -- it only
// needs per-layer outlines.
// ============================================================================

// True interior segment-segment crossing only -- a shared endpoint or collinear touch does not
// count, since a tangent line correctly built against one hole is expected to graze it exactly.
bool segmentsProperlyIntersect(const Point2LL& p1, const Point2LL& p2, const Point2LL& p3, const Point2LL& p4)
{
    auto cross = [](const Point2LL& o, const Point2LL& a, const Point2LL& b) -> double
    {
        return static_cast<double>(a.X - o.X) * static_cast<double>(b.Y - o.Y) - static_cast<double>(a.Y - o.Y) * static_cast<double>(b.X - o.X);
    };
    const double d1 = cross(p3, p4, p1);
    const double d2 = cross(p3, p4, p2);
    const double d3 = cross(p1, p2, p3);
    const double d4 = cross(p1, p2, p4);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

bool segmentCrossesHoleInterior(const Point2LL& a, const Point2LL& b, const Polygon& hole)
{
    const size_t n = hole.size();
    for (size_t i = 0; i < n; i++)
    {
        const size_t j = (i + 1) % n;
        if (segmentsProperlyIntersect(a, b, hole[i], hole[j]))
            return true;
    }
    return false;
}

double pointDistance(const Point2LL& a, const Point2LL& b)
{
    const double dx = static_cast<double>(b.X - a.X), dy = static_cast<double>(b.Y - a.Y);
    return std::sqrt(dx * dx + dy * dy);
}

// A point sampled on O's own outer boundary, with its winding-based inward normal (same
// convention as FeatherPrintGenerator::resolveFrame: tangent rotated +90 degrees, tangent sign
// flipped for a CW-wound polygon -- no centroid dependency).
struct ShoreRimPoint
{
    Point2LL pos;
    double nx, ny;
    int seg_idx; // originating segment index, for ray-cast self-exclusion
};

// Samples outer's boundary at ~step arc-length intervals. Interval is a tentative constant
// (see fp_shore_overhangs' doc comment in sliceDataStorage.h) -- not spec-mandated.
std::vector<ShoreRimPoint> sampleShoreRimPoints(const Polygon& outer, coord_t step)
{
    std::vector<ShoreRimPoint> pts;
    const size_t n = outer.size();
    if (n < 3 || step <= 0)
        return pts;

    double area2 = 0.0;
    std::vector<double> seg_len(n);
    double total = 0.0;
    for (size_t i = 0; i < n; i++)
    {
        const size_t j = (i + 1) % n;
        area2 += static_cast<double>(outer[i].X) * outer[j].Y - static_cast<double>(outer[j].X) * outer[i].Y;
        seg_len[i] = pointDistance(outer[i], outer[j]);
        total += seg_len[i];
    }
    if (total < 1.0)
        return pts;
    const bool ccw = area2 > 0.0;

    const int n_samples = std::max(3, static_cast<int>(std::llround(total / static_cast<double>(step))));
    size_t seg = 0;
    double seg_start = 0.0;
    for (int k = 0; k < n_samples; k++)
    {
        const double s = (static_cast<double>(k) / n_samples) * total;
        while (seg + 1 < n && seg_start + seg_len[seg] < s)
        {
            seg_start += seg_len[seg];
            seg++;
        }
        const size_t j = (seg + 1) % n;
        const double ex = static_cast<double>(outer[j].X - outer[seg].X), ey = static_cast<double>(outer[j].Y - outer[seg].Y);
        const double elen = seg_len[seg];
        const double t = (elen > 1e-6) ? (s - seg_start) / elen : 0.0;
        const double px = outer[seg].X + t * ex, py = outer[seg].Y + t * ey;
        double tx = (elen > 1e-6) ? ex / elen : 1.0, ty = (elen > 1e-6) ? ey / elen : 0.0;
        if (! ccw)
        {
            tx = -tx;
            ty = -ty;
        }
        const double nx = -ty, ny = tx;
        pts.push_back({ Point2LL(static_cast<coord_t>(std::llround(px)), static_cast<coord_t>(std::llround(py))), nx, ny, static_cast<int>(seg) });
    }
    return pts;
}

// Nearest intersection of the ray (from, dir) against outer's own boundary, excluding the
// segment(s) immediately adjacent to `from`'s own originating segment (index-adjacency
// exclusion, same pattern FeatherPrintGenerator::isThinSection uses to avoid a trivial self-hit).
std::optional<Point2LL> castShoreRay(const Point2LL& from, double dx, double dy, const Polygon& outer, int exclude_seg)
{
    const size_t n = outer.size();
    double best_t = -1.0;
    Point2LL best_pt{};
    for (size_t i = 0; i < n; i++)
    {
        if (exclude_seg >= 0)
        {
            int d = std::abs(static_cast<int>(i) - exclude_seg);
            d = std::min(d, static_cast<int>(n) - d);
            if (d <= 1)
                continue;
        }
        const size_t j = (i + 1) % n;
        const Point2LL& A = outer[i];
        const Point2LL& B = outer[j];
        const double ex = static_cast<double>(B.X - A.X), ey = static_cast<double>(B.Y - A.Y);
        const double denom = dx * ey - dy * ex;
        if (std::abs(denom) < 1e-9)
            continue;
        const double asx = static_cast<double>(A.X - from.X), asy = static_cast<double>(A.Y - from.Y);
        const double t = (asx * ey - asy * ex) / denom;
        const double u = (asx * dy - asy * dx) / denom;
        if (! (u >= -1e-9 && u <= 1.0 + 1e-9 && t > 1e-6))
            continue;
        if (best_t < 0.0 || t < best_t)
        {
            best_t = t;
            best_pt = Point2LL(static_cast<coord_t>(std::llround(from.X + t * dx)), static_cast<coord_t>(std::llround(from.Y + t * dy)));
        }
    }
    if (best_t < 0.0)
        return std::nullopt;
    return best_pt;
}

// Standard O(n) tangent-point scan: hull vertex i is a tangent point from external point P iff
// both its neighboring edges keep the other vertex on the same side of the line P->hull[i].
std::vector<size_t> findShoreTangentVertices(const std::vector<Point2LL>& hull, const Point2LL& P)
{
    std::vector<size_t> tangents;
    const size_t n = hull.size();
    if (n < 3)
        return tangents;
    auto cross2 = [](double ax, double ay, double bx, double by) { return ax * by - ay * bx; };
    for (size_t i = 0; i < n; i++)
    {
        const size_t prev = (i + n - 1) % n, next = (i + 1) % n;
        const double vx = static_cast<double>(hull[i].X - P.X), vy = static_cast<double>(hull[i].Y - P.Y);
        const double px = static_cast<double>(hull[prev].X - P.X), py = static_cast<double>(hull[prev].Y - P.Y);
        const double nx = static_cast<double>(hull[next].X - P.X), ny = static_cast<double>(hull[next].Y - P.Y);
        const double c1 = cross2(vx, vy, px, py);
        const double c2 = cross2(vx, vy, nx, ny);
        if (c1 * c2 >= 0.0)
            tangents.push_back(i);
    }
    return tangents;
}

// Raw slice-time outline (SliceLayerPart::outline, includes holes) for every part of a layer.
// NOT SliceLayer::getOutlines() -- that returns print_outline, which for a FeatherPrint mesh is
// only ever populated later, inside WallsComputation.cpp's wall-generation pass, which runs
// AFTER this pre-pass (confirmed the actual bug behind Shore detecting nothing on a real test:
// print_outline is empty at this point in the pipeline, so every basic_overhang came out empty).
Shape rawSliceOutline(const SliceLayer& layer)
{
    Shape result;
    for (const SliceLayerPart& part : layer.parts)
        result.push_back(part.outline);
    return result;
}

// Same as rawSliceOutline, but morphologically opened (offset in, then back out) by half a Wall
// width -- the identical filter WallsComputation.cpp applies (see its own gen_outline) before
// generating any Wall at all. On a non-manifold/non-watertight input mesh, the slicer's own
// contour-stitching pass can leave thin spurious slivers in a Layer's raw outline that don't
// correspond to any real geometry the mesh models; WallsComputation silently drops these before
// ever laying down a Wall, so no printed surface (and therefore no top skin) ever appears there.
// Shore must use the same filtered view, not the raw one, or it detects "Top Surface" in slivers
// that get filtered out before printing and never actually receive real top skin -- confirmed as
// the cause of Shore firing under regions with no real top layer on a non-watertight test mesh.
Shape filteredSliceOutline(const SliceLayer& layer, coord_t line_width)
{
    Shape raw = rawSliceOutline(layer);
    if (raw.empty())
        return raw;
    Shape opened = raw.offset(-line_width / 2).offset(line_width / 2);
    return opened.empty() ? raw : opened;
}

// Generates every surviving candidate bridge segment for one candidate O (Rim Point
// Distribution / Candidate Bridge Generation / Selection, Spec REV 3.1).
std::vector<std::pair<Point2LL, Point2LL>> generateShoreBridges(const SingleShape& O, coord_t rim_step)
{
    std::vector<std::pair<Point2LL, Point2LL>> result;
    if (O.empty())
        return result;
    const Polygon& outer = O.outerPolygon();
    if (outer.size() < 3)
        return result;

    std::vector<Polygon> holes;
    for (size_t i = 1; i < O.size(); i++)
        holes.push_back(O[i]);

    std::vector<std::vector<Point2LL>> hole_hulls;
    for (const Polygon& hole : holes)
    {
        Shape wrap;
        wrap.push_back(hole);
        Shape hull_shape = wrap.approxConvexHull(0);
        const Polygon* hull = nullptr;
        double best_a = 0.0;
        for (const Polygon& p : hull_shape)
        {
            const double a = std::abs(p.area());
            if (a > best_a)
            {
                best_a = a;
                hull = &p;
            }
        }
        if (hull)
            hole_hulls.emplace_back(hull->begin(), hull->end());
    }

    const std::vector<ShoreRimPoint> rim = sampleShoreRimPoints(outer, rim_step);

    struct Candidate
    {
        Point2LL a, b;
        double len;
    };
    std::vector<Candidate> candidates;

    auto tryCandidate = [&](const Point2LL& from, double dx, double dy, int seg_idx)
    {
        const std::optional<Point2LL> hit = castShoreRay(from, dx, dy, outer, seg_idx);
        if (! hit)
            return;
        for (const Polygon& hole : holes)
            if (segmentCrossesHoleInterior(from, *hit, hole))
                return;
        candidates.push_back({ from, *hit, pointDistance(from, *hit) });
    };

    for (const ShoreRimPoint& rp : rim)
    {
        tryCandidate(rp.pos, rp.nx, rp.ny, rp.seg_idx);
        for (const std::vector<Point2LL>& hull : hole_hulls)
        {
            for (size_t t : findShoreTangentVertices(hull, rp.pos))
            {
                double dx = static_cast<double>(hull[t].X - rp.pos.X), dy = static_cast<double>(hull[t].Y - rp.pos.Y);
                const double len = std::sqrt(dx * dx + dy * dy);
                if (len < 1e-6)
                    continue;
                dx /= len;
                dy /= len;
                tryCandidate(rp.pos, dx, dy, rp.seg_idx);
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) { return a.len < b.len; });

    std::vector<Candidate> kept;
    for (const Candidate& c : candidates)
    {
        bool crosses = false;
        for (const Candidate& k : kept)
            if (segmentsProperlyIntersect(c.a, c.b, k.a, k.b))
            {
                crosses = true;
                break;
            }
        if (! crosses)
            kept.push_back(c);
    }

    for (const Candidate& k : kept)
        result.emplace_back(k.a, k.b);
    return result;
}
} // namespace

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

        // Gathers this layer's outer-wall point ring: the largest closed polygon part, or (if
        // none) the same full virtual ring (real arcs + gap chords) WallsComputation and
        // generateOpen build for an open-manifold layer. Shared by the R_avg pre-pass below and
        // the existing phase/origin loop that follows it — both need the identical ring.
        //
        // Closed-polygon case is put through the SAME morphological open (offset in by w/2,
        // then out by w/2) WallsComputation applies before ever calling generate() — this is
        // NOT optional cosmetic parity: curvature is far more sensitive to small-scale mesh-
        // slicing facet noise than plain perimeter length is (which is all this ring was used
        // for before REV 2.6). Measuring curvature against the RAW, un-smoothed slice polygon
        // here while generate() itself places anchors against the cleaned gen_outline produces
        // two genuinely different curvature signals for "the same" layer — the phase integral
        // (built from raw-polygon noise) and the per-layer warp used for placement (built from
        // the cleaned polygon) end up uncorrelated, which reads as anchors "randomly" failing to
        // track no matter how the curvature estimator itself is refined. Fixed by smoothing here
        // to match, rather than by further tuning the estimator (which had no effect, since the
        // estimator wasn't the actual mismatch).
        const coord_t fp_w_smooth = fp_w;
        auto gatherRing = [fp_w_smooth](const SliceLayer& layer) -> std::vector<Point2LL>
        {
            double best_area = 0.0;
            std::vector<Point2LL> ring_pts;
            for (const SliceLayerPart& part : layer.parts)
                for (const Polygon& poly : part.outline)
                {
                    double a = std::abs(poly.area());
                    if (a > best_area)
                    {
                        best_area = a;
                        ring_pts.assign(poly.begin(), poly.end());
                    }
                }
            if (! ring_pts.empty())
            {
                Polygon raw_poly;
                for (const Point2LL& p : ring_pts)
                    raw_poly.push_back(p);
                Shape smoothed = Shape(raw_poly).offset(-fp_w_smooth / 2).offset(fp_w_smooth / 2);
                const Polygon* largest = nullptr;
                double la = 0.0;
                for (const Polygon& p : smoothed)
                {
                    double a = std::abs(p.area());
                    if (a > la) { la = a; largest = &p; }
                }
                if (largest)
                    ring_pts.assign(largest->begin(), largest->end());
                return ring_pts;
            }
            if (layer.open_polylines.empty())
                return ring_pts;

            // Open-manifold layers: build the same full virtual ring that WallsComputation
            // and generateOpen use. All arc fragments are concatenated in angular order
            // (sorted by start angle from the layer centroid); gap chords between consecutive
            // arcs and the closing chord are implicit polygon segments, matching the
            // full_ring_total in generateOpen.
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

            // Orient each arc (CCW in math = positive virtual-closed area) so that poly[0] is
            // the correct start point for angle-sorting and chord measurement. Without this,
            // reversed arcs sort by the wrong endpoint, producing wrong inter-arc chord lengths
            // and an inflated full_ring_total.
            struct OrientedRef
            {
                std::vector<Point2LL> pts; // oriented points (may be reversed copy)
                double angle{};
            };
            std::vector<OrientedRef> refs;
            for (const OpenPolyline& poly : layer.open_polylines)
            {
                if (poly.size() < 2) continue;
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

            for (const OrientedRef& ref : refs)
                ring_pts.insert(ring_pts.end(), ref.pts.begin(), ref.pts.end());
            return ring_pts;
        };

        auto ringLength = [](const std::vector<Point2LL>& pts) -> double
        {
            if (pts.size() < 2) return 0.0;
            double total = 0.0;
            const size_t n = pts.size();
            for (size_t i = 0; i + 1 < n; i++)
            {
                double dx = pts[i + 1].X - pts[i].X, dy = pts[i + 1].Y - pts[i].Y;
                total += std::sqrt(dx * dx + dy * dy);
            }
            double cdx = pts[0].X - pts[n - 1].X, cdy = pts[0].Y - pts[n - 1].Y;
            total += std::sqrt(cdx * cdx + cdy * cdy);
            return total;
        };

        // Curvature-Weighted Stringer Density (Spec REV 2.6) -- DISABLED as-built pending
        // further investigation. Six rounds of fixes to the per-layer curvature estimator
        // (fixed arc-length window at several sizes, fixed vertex count, a window grown to a
        // minimum physical span, uniform resampling, and finally resample-grid phase-locking to
        // Anchor Distribution's own Phase Origin -- a genuine structural bug, confirmed and
        // fixed) all failed to produce a stable W_total(z) on a real part: total_warped kept
        // swinging by a large factor between layers whose real geometry (arc_total_mm) was
        // essentially unchanged. The likely remaining cause is NOT a bug in this per-layer
        // estimator: different Z-layers slice through different mesh triangle edges even on a
        // smooth CAD surface, so each layer's cross-section polygon is built from a genuinely
        // different vertex set layer to layer -- no amount of smoothing a curvature estimate
        // reconstructed from one layer's own 2D slice fixes that; it would need curvature
        // derived from the source mesh's 3D geometry before slicing, which is a materially
        // bigger undertaking left for a dedicated future session. mesh.fp_r_ref stays 0.0 here
        // unconditionally, which every downstream caller (buildWarp, computeWarpedTotal) already
        // treats as "feature inactive" -- Δphase and anchor placement both fall back to plain
        // uniform arc-length spacing, matching pre-REV-2.6 behavior exactly.
        mesh.fp_r_ref = 0.0;

        double phase = 0.0;
        Point2LL fp_prev_origin{};
        bool fp_have_prev_origin = false;
        for (size_t layer_nr = 0; layer_nr < mesh_layer_count; layer_nr++)
        {
            mesh.fp_helix_phase[layer_nr] = std::fmod(phase, 1.0);
            const SliceLayer& layer = mesh.layers[layer_nr];

            // Anchor Distribution (Spec REV 2.2): the outer wall's own point set for this
            // layer (closed-polygon case, or the open-manifold virtual ring) — used both below
            // for the phase/W_total increment and by the seed-point capture that follows.
            std::vector<Point2LL> fp_seed_ring_pts = gatherRing(layer);
            const double arc_total_mm = ringLength(fp_seed_ring_pts) / 1000.0;

            // Anchor Distribution (Spec REV 2.2, continuity-tracking variant — see
            // fp_phase_origin's declaration for why this deviates from the spec's literal
            // fixed-point design). "Layer 0" is the first layer whose largest outline passes
            // the same size guard generate() itself uses (arc.total < 4.0 * w returns early
            // there); every layer from there up walks its own Phase Origin forward.
            //
            // Moved BEFORE the phase/W_total block below (was previously after it): Curvature-
            // Weighted Stringer Density's computeWarpedTotal call needs THIS layer's own Phase
            // Origin already known, to phase-lock its internal curvature-resampling grid to it
            // (see ArcParam::resample_phase_s) rather than to gatherRing's own arbitrary
            // (slicer-determined) starting vertex — using the origin computed here, not a stale
            // one, is what makes that phase-lock actually stable layer to layer.
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

            if (arc_total_mm > 1e-6)
            {
                const double layer_height_mm = static_cast<double>(layer.printZ) / 1000.0
                    - (layer_nr > 0 ? static_cast<double>(mesh.layers[layer_nr - 1].printZ) / 1000.0 : 0.0);
                // Curvature-Weighted Stringer Density (REV 2.6) is disabled as-built (see the
                // mesh.fp_r_ref = 0.0 assignment above) -- computeWarpedTotal always falls back
                // to total == total_warped == raw arc length, so this reduces exactly to the
                // pre-REV-2.6 formula. Still routed through computeWarpedTotal (rather than
                // simplified back to raw arc_total_mm directly) so re-enabling the feature later
                // only requires removing that one forced assignment, not restoring this call.
                double total_raw = 0.0, total_warped = 0.0;
                FeatherPrintGenerator::computeWarpedTotal(fp_seed_ring_pts, mesh.fp_r_ref, mesh.fp_phase_origin[layer_nr], total_raw, total_warped);
                const double w_total_mm = (total_warped > 1e-6) ? (total_warped / 1000.0) : arc_total_mm;
                phase += layer_height_mm / (w_total_mm * tan_alpha);
            }
        }

        // Shore (Internal Overhang Bridging, Spec REV 3.1, as rewritten 22 Jul 26): detection,
        // then bridge generation. Independent of the phase/origin loop above -- only needs
        // per-layer outlines, no Stringer/Lacing dependency at all.
        //
        // This supersedes an earlier, much more elaborate implementation (three successive
        // local single/double-layer heuristics, then a whole-mesh reachability flood fill) built
        // against the PREVIOUS wording of "internal overhang," which asked whether new solid
        // area extended the model's own silhouette outward. Real-print testing on this fork
        // found the flood fill technically correct but answering a question the actual use case
        // didn't have: a shelf fused to an unrelated wall, confirmed genuinely disconnected from
        // any enclosed cavity by the flood fill itself, still needed bridging in practice, since
        // FeatherPrint prints hollow with no infill -- there's no separate concept of "internal"
        // print geometry, only whether the space beneath a given overhang is open exterior air
        // or bounded by the model's own envelope. The spec was rewritten to reflect this
        // directly rather than the flood fill being tuned further.
        //
        // Two simple, local, two-layer tests, per the rewritten spec text:
        //   T (Top Surface)  = layer_n \ layer_{n+1}          -- no angle filter, mirrors
        //                                                          ordinary top-skin detection.
        //   Enclosure         = T's own footprint, at Layer n-1, falls within that Layer's own
        //                        outer envelope (OML extent) AND is hollow there (not already
        //                        solid) -- if already solid at n-1, ordinary skin already has
        //                        something to rest on, so Shore does not apply.
        // O = T restricted to the portion passing Enclosure. This single general test also
        // covers the "flat top of an otherwise-solid hollow-printed part" case directly (T there
        // is the whole top disk, since nothing exists at layer n+1; Enclosure resolves to the
        // wall ring's own inner hollow) -- no separate topmost-Layer special case is needed.
        //
        // Distinguishing a genuinely open top (no real surface to cap, so Shore is pointless
        // there) from an ordinary closed taper turned out to have NO signal in per-layer 2D
        // slice geometry at all: real-print testing found a mesh whose top the user modeled as
        // open still slices into a perfectly ordinary, monotonically-shrinking sequence of
        // closed loops all the way to a point, with zero open_polylines anywhere near the top --
        // ray-casting always produces a closed 2D contour regardless of whether the source mesh
        // actually has a face capping that region in 3D. Two earlier attempts at a geometric
        // signal (Layer n's own open_polylines, then Layer n+1's) were both confirmed dead ends
        // this way. Flange is instead used as the actual signal: the user already tells the
        // engine "this top is open" by enabling Flange (see the ramp ordering below) -- so Shore
        // simply must not fire within Flange's own ramp zone, computed first so Shore can read
        // it directly instead of re-deriving open-ness itself.
        {
            const size_t n_flange = mesh.settings.get<size_t>("featherprint_flange_ramp_layers");
            LayerIndex flange_last_geom = -1;
            for (int li = static_cast<int>(mesh_layer_count) - 1; li >= 0; li--)
            {
                if (! mesh.layers[li].parts.empty() || ! mesh.layers[li].open_polylines.empty())
                {
                    flange_last_geom = static_cast<LayerIndex>(li);
                    break;
                }
            }
            if (mesh.settings.get<bool>("featherprint_flange_enabled") && flange_last_geom >= 0)
            {
                mesh.fp_flange_start_layer = static_cast<LayerIndex>(
                    std::max(LayerIndex(0), flange_last_geom - static_cast<LayerIndex>(n_flange) + 1));
            }
        }
        {
            mesh.fp_shore_overhangs.assign(mesh_layer_count, {});
            mesh.fp_shore_bridges.assign(mesh_layer_count, {});

            const coord_t shore_rim_step = mesh.settings.get<coord_t>("featherprint_line_width");
            const size_t shore_top_layers = mesh.settings.get<size_t>("top_layers");

            for (size_t li = 0; li < mesh_layer_count; li++)
            {
                Shape outline_li = filteredSliceOutline(mesh.layers[li], shore_rim_step);
                if (outline_li.empty())
                    continue;

                // Flange's own ramp zone is the user's explicit signal that this top is open --
                // Shore must not fire anywhere Flange is already handling the same region.
                if (mesh.settings.get<bool>("featherprint_flange_enabled") && mesh.fp_flange_start_layer >= 0
                    && static_cast<LayerIndex>(li) >= mesh.fp_flange_start_layer)
                    continue;

                // If Layer n+1 has geometry that isn't represented as closed parts at all --
                // Whip's own intentionally-open boundary edge, carried as open_polylines rather
                // than a SliceLayerPart -- the model genuinely continues upward there, just not
                // as a closed loop. Unrelated to the Flange case above (Whip Terminal, not an
                // open top), but the same reasoning applies: no real top skin is ever generated
                // over that continuation either.
                if (li + 1 < mesh_layer_count && ! mesh.layers[li + 1].open_polylines.empty())
                    continue;

                Shape outline_above = (li + 1 < mesh_layer_count) ? filteredSliceOutline(mesh.layers[li + 1], shore_rim_step) : Shape{};
                Shape T = outline_li.difference(outline_above);
                if (T.empty())
                    continue;

                if (li == 0)
                    continue; // no Layer below to test Enclosure against

                // "Solid" here must mean what FeatherPrint actually PRINTS as solid, not what the
                // raw CAD mesh's own cross-section happens to be. FeatherPrint never prints
                // solid fill anywhere except within one line width of the traced OML --
                // regardless of whether the raw mesh is solid or hollow at that point. Using the
                // raw mesh's own solidity here (as an earlier version of this code did) made
                // hollow_below empty everywhere on any model with no CAD-modeled cavity, since a
                // solid mesh's own "outer envelope" and "solid area" are then identical by
                // construction -- confirmed via real diagnostic testing this session.
                //
                // li-1's own Wall stack is always the plain single-line-width kind here, never
                // Flange's thicker ramp: li itself is already confirmed above to be below
                // mesh.fp_flange_start_layer, so li-1 is strictly further below it too.
                const coord_t fp_w = mesh.settings.get<coord_t>("featherprint_line_width");
                // Same morphological-open filter as outline_li/outline_above above, applied to the
                // OML extent at li-1 (holes filled) -- otherwise a stitching sliver from a
                // non-manifold input mesh can pass the Enclosure test the same way it would
                // wrongly pass the Top Surface test, for the same reason.
                Shape envelope_below_raw = mesh.layers[li - 1].getOutlines(true);
                Shape envelope_below = envelope_below_raw.offset(-fp_w / 2).offset(fp_w / 2);
                if (envelope_below.empty())
                    envelope_below = envelope_below_raw;
                Shape hollow_below = envelope_below.offset(-fp_w);
                if (hollow_below.empty())
                    continue; // li-1's own envelope is narrower than one Wall -- nothing hollow to enclose T over

                Shape O_all = T.intersection(hollow_below);
                if (O_all.empty())
                    continue;

                for (const SingleShape& O : O_all.splitIntoParts())
                {
                    mesh.fp_shore_overhangs[li].push_back(O);
                    std::vector<std::pair<Point2LL, Point2LL>> bridges = generateShoreBridges(O, shore_rim_step);
                    if (bridges.empty())
                        continue;
                    const size_t target_layer = (li > shore_top_layers) ? (li - shore_top_layers) : 0;
                    auto& dst = mesh.fp_shore_bridges[target_layer];
                    dst.insert(dst.end(), bridges.begin(), bridges.end());
                }
            }
        }

        // Flange's own start layer (unconditionally spans featherprint_flange_ramp_layers layers
        // down from the topmost layer with any boundary geometry — closed parts OR
        // open_polylines) is now computed earlier, above, right before Shore's own pre-pass, so
        // Shore can read mesh.fp_flange_start_layer directly instead of re-deriving open-ness.
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
    WallsComputation walls_computation(mesh.settings, layer_nr, fp_phase, mesh.fp_flange_start_layer, fp_phase_origin, mesh.fp_r_ref);
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
