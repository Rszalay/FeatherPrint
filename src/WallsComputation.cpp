// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "WallsComputation.h"

#include <fstream>
#include <iostream>

#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <range/v3/to_container.hpp>
#include <range/v3/view/c_str.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>

#include <algorithm>
#include <cmath>

#include "Application.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "WallToolPaths.h"
#include "featherprint/FeatherPrintGenerator.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"
#include "geometry/SingleShape.h"
#include "settings/EnumSettings.h"
#include "settings/types/Ratio.h"
#include "sliceDataStorage.h"
#include "utils/Simplify.h" // We're simplifying the spiralized insets.

namespace cura
{

WallsComputation::WallsComputation(
    const Settings& settings,
    const LayerIndex layer_nr,
    double fp_helix_phase,
    LayerIndex fp_flange_start_layer,
    int fp_former_ramp,
    Point2LL fp_phase_origin,
    double fp_r_ref,
    std::vector<SliceMeshStorage::FpPunchoutGapSpan> fp_punchout_gap_spans,
    std::vector<Polygon> fp_interior_holes)
    : settings_(settings)
    , layer_nr_(layer_nr)
    , fp_helix_phase_(fp_helix_phase)
    , fp_flange_start_layer_(fp_flange_start_layer)
    , fp_former_ramp_(fp_former_ramp)
    , fp_phase_origin_(fp_phase_origin)
    , fp_r_ref_(fp_r_ref)
    , fp_punchout_gap_spans_(std::move(fp_punchout_gap_spans))
    , fp_interior_holes_(std::move(fp_interior_holes))
{
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateWalls only reads and writes data for the current layer
 */
void WallsComputation::generateWalls(SliceLayerPart* part, SectionType section_type, coord_t print_z)
{
    // FeatherPrint: replace Arachne entirely with the conformal stringer generator.
    // wall_line_count must be >= 1 so the pipeline guards don't early-out, but Arachne
    // is never called — the stringer path populates wall_toolpaths directly.
    if (settings_.get<EFillMethod>("infill_pattern") == EFillMethod::FEATHERPRINT)
    {
        // Initial bottom layers: skip FeatherPrint and fall through to normal wall + skin
        // generation so that solid fill is produced on the base of the print.
        const size_t init_bottom = settings_.get<size_t>("initial_bottom_layers");
        if (layer_nr_ >= static_cast<LayerIndex>(init_bottom))
        {
            const coord_t z_coord = print_z;
            const coord_t w = settings_.get<coord_t>("featherprint_line_width");

            // Morphological open: removes any feature narrower than the line width.
            Shape gen_outline = Shape(part->outline).offset(-w / 2).offset(w / 2);
            if (gen_outline.empty())
                gen_outline = Shape(part->outline);

            FeatherPrintGenerator fp;
            const bool is_flange_layer = (fp_flange_start_layer_ >= 0
                                          && layer_nr_ >= fp_flange_start_layer_);
            // Flange takes priority if a layer is ever flagged as both (edge case at the very
            // top of a short model — Flange is a top-of-model zone, Former bands are interior,
            // so this isn't expected in practice).
            const bool is_former_layer = (! is_flange_layer && fp_former_ramp_ >= 0);
            VariableWidthLines wl;
            if (is_flange_layer)
            {
                const int ramp_index = static_cast<int>(layer_nr_ - fp_flange_start_layer_);
                wl = fp.generateFlange(gen_outline, settings_, ramp_index, fp_helix_phase_, fp_phase_origin_, fp_r_ref_);
            }
            else if (is_former_layer)
            {
                wl = fp.generateFormer(gen_outline, settings_, fp_former_ramp_, fp_helix_phase_, fp_phase_origin_, fp_r_ref_);
            }
            else
            {
                wl = fp.generate(gen_outline, z_coord, settings_, fp_helix_phase_, fp_phase_origin_, fp_r_ref_);
            }
            part->wall_toolpaths = { std::move(wl) };
            // Inner area: the region enclosed by the innermost printed Wall at this Layer, so
            // top_layers/bottom_layers (stock Cura settings) can cap a FeatherPrint shell with
            // real skin where the designer left the surface closed. Previously left
            // unconditionally empty, which silently disabled top/bottom skin on every
            // FeatherPrint Layer regardless of those settings. This is only reached for a
            // closed-perimeter Layer (a SliceLayerPart with a real outline) — an open-manifold
            // (Whip) Layer, where the designer intentionally left a boundary edge open, is
            // handled entirely separately (the open_polylines branch below) and continues to
            // leave inner_area empty there; no surface exists for skin to cap.
            //
            // Interior Opening Carry-Through (Spec REV 3.4): part->outline is a SingleShape —
            // Cura's own "first polygon is the outer contour, the rest are holes"
            // representation, built by stock slicing/stitching and untouched by FeatherPrint.
            // A hole reaching this Layer with real depth already appears here as an inner
            // polygon — no new detection needed for that case. The blind
            // `Shape(gen_outline).offset(...)` this replaces treated the whole SingleShape
            // (outer + any hole) as one blob, which could distort or erase a hole near the
            // offset distance in size instead of leaving it alone; that silently painted
            // top/bottom skin fill straight over the opening. Holes are now subtracted back in
            // explicitly, unmodified (not separately offset), after the same offset used
            // before — "the hole should read no smaller than its own true boundary." This does
            // NOT touch gen_outline or Wall generation above (both generate()/generateFlange()
            // already only trace the largest polygon via largestPoly(), ignoring any hole
            // polygons in the Shape they're given, so there is nothing to change there) —
            // scoped to the skin-fill gap only, per the spec's own scoping. A genuinely
            // zero-depth interior boundary loop (no vertical wall at all) may or may not
            // survive Cura's own stitching into a part->outline hole polygon the same way a
            // real-depth hole does — unverified; the diagnostic log below is meant to answer
            // that from a real test print rather than guessing.
            Shape holes;
            for (size_t hole_i = 1; hole_i < part->outline.size(); hole_i++)
                holes.push_back(part->outline[hole_i]);
            // Interior Opening Carry-Through Phase 2: coplanar interior holes detected from
            // the raw mesh's own boundary-edge topology (SliceMeshStorage::fp_interior_holes,
            // built in FffPolygonGenerator::sliceModel() — see its own doc comment), for
            // holes that never reach part->outline at all because their boundary is coplanar
            // with a Z-cutting-plane rather than crossing it. Merged into the same hole set.
            for (const Polygon& extra_hole : fp_interior_holes_)
                holes.push_back(extra_hole);
            if (! holes.empty())
            {
                coord_t hole_min_x = holes[0][0].X, hole_max_x = holes[0][0].X;
                coord_t hole_min_y = holes[0][0].Y, hole_max_y = holes[0][0].Y;
                for (const Polygon& hole : holes)
                    for (const Point2LL& hp : hole)
                    {
                        hole_min_x = std::min(hole_min_x, hp.X); hole_max_x = std::max(hole_max_x, hp.X);
                        hole_min_y = std::min(hole_min_y, hp.Y); hole_max_y = std::max(hole_max_y, hp.Y);
                    }
                spdlog::info(
                    "FP-DIAG interior holes at z={} layer={}: count={} (from_outline={} from_coplanar={}) bbox=({},{})-({},{})",
                    z_coord, layer_nr_, holes.size(), holes.size() - fp_interior_holes_.size(), fp_interior_holes_.size(),
                    hole_min_x, hole_min_y, hole_max_x, hole_max_y);
            }
            Shape inner_area_raw = Shape(gen_outline).offset(-fp.innerOffset());
            part->inner_area     = holes.empty() ? inner_area_raw : inner_area_raw.difference(holes);
            part->print_outline  = part->outline;

            part->outline = SingleShape{ Simplify(settings_).polygon(part->outline) };
            part->print_outline = part->outline;
            return;
        }
        // layer_nr_ < init_bottom: fall through to normal wall generation below
    }

    // Interior Opening Carry-Through (Spec REV 3.4 Phase 2): fp_interior_holes_ is populated
    // from mesh-level coplanar boundary-loop detection, independent of whether this Layer
    // takes FeatherPrint's own wall-generation path above or falls through to stock Arachne
    // here (e.g. initial_bottom_layers, which deliberately bypasses FeatherPrint to build a
    // solid adhesion base — but a hole flush with the build plate, the exact case this
    // detection targets, sits AT layer 0 and would otherwise never see this subtraction at
    // all). part->outline's own hole polygons don't need repeating here — stock WallToolPaths
    // is general-purpose and already handles a multi-contour SingleShape with real holes
    // correctly, unlike FeatherPrint's own single-polygon generate().
    auto subtractInteriorHoles = [this](Shape& area)
    {
        if (fp_interior_holes_.empty())
            return;
        Shape holes;
        for (const Polygon& h : fp_interior_holes_)
            holes.push_back(h);
        area = area.difference(holes);
    };

    size_t wall_count = settings_.get<size_t>("wall_line_count");
    if (wall_count == 0) // Early out if no walls are to be generated
    {
        part->print_outline = part->outline;
        part->inner_area = part->outline;
        subtractInteriorHoles(part->inner_area);
        return;
    }

    const bool spiralize = settings_.get<bool>("magic_spiralize");
    const size_t alternate = ((layer_nr_ % 2) + 2) % 2;
    if (spiralize && layer_nr_ < LayerIndex(settings_.get<size_t>("initial_bottom_layers"))
        && alternate == 1) // Add extra insets every 2 layers when spiralizing. This makes bottoms of cups watertight.
    {
        wall_count += 5;
    }
    if (settings_.get<bool>("alternate_extra_perimeter"))
    {
        wall_count += alternate;
    }

    const bool first_layer = layer_nr_ == 0;
    const Ratio line_width_0_factor = first_layer ? settings_.get<ExtruderTrain&>("wall_0_extruder_nr").settings_.get<Ratio>("initial_layer_line_width_factor") : 1.0_r;
    const coord_t line_width_0 = settings_.get<coord_t>("wall_line_width_0") * line_width_0_factor;
    const coord_t wall_0_inset = settings_.get<coord_t>("wall_0_inset");

    const Ratio line_width_x_factor = first_layer ? settings_.get<ExtruderTrain&>("wall_x_extruder_nr").settings_.get<Ratio>("initial_layer_line_width_factor") : 1.0_r;
    const coord_t line_width_x = settings_.get<coord_t>("wall_line_width_x") * line_width_x_factor;

    // When spiralizing, generate the spiral insets using simple offsets instead of generating toolpaths
    if (spiralize)
    {
        const bool recompute_outline_based_on_outer_wall = settings_.get<bool>("support_enable") && ! settings_.get<bool>("fill_outline_gaps");

        generateSpiralInsets(part, line_width_0, wall_0_inset, recompute_outline_based_on_outer_wall);
        if (layer_nr_ <= static_cast<LayerIndex>(settings_.get<size_t>("initial_bottom_layers")))
        {
            WallToolPaths wall_tool_paths(part->outline, line_width_0, line_width_x, wall_count, wall_0_inset, settings_, layer_nr_, section_type);
            part->wall_toolpaths = wall_tool_paths.getToolPaths();
            part->inner_area = wall_tool_paths.getInnerContour();
            subtractInteriorHoles(part->inner_area);
        }
    }
    else
    {
        WallToolPaths wall_tool_paths(part->outline, line_width_0, line_width_x, wall_count, wall_0_inset, settings_, layer_nr_, section_type);
        part->wall_toolpaths = wall_tool_paths.getToolPaths();
        part->inner_area = wall_tool_paths.getInnerContour();
        subtractInteriorHoles(part->inner_area);
    }

    part->outline = SingleShape{ Simplify(settings_).polygon(part->outline) };
    part->print_outline = part->outline;
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateWalls only reads and writes data for the current layer
 */
void WallsComputation::generateWalls(SliceLayer* layer, SectionType section)
{
    for (SliceLayerPart& part : layer->parts)
    {
        generateWalls(&part, section, layer->printZ);
    }

    // Remove the parts which did not generate a wall. As these parts are too small to print,
    //  and later code can now assume that there is always minimal 1 wall line.
    bool check_wall_and_spiral = settings_.get<size_t>("wall_line_count") >= 1 && ! settings_.get<bool>("fill_outline_gaps");
    auto iterator_remove = std::remove_if(
        layer->parts.begin(),
        layer->parts.end(),
        [&check_wall_and_spiral](const SliceLayerPart& part)
        {
            return (check_wall_and_spiral && part.wall_toolpaths.empty() && part.spiral_wall.empty()) || part.outline.empty() || part.print_outline.empty();
        });
    layer->parts.erase(iterator_remove, layer->parts.end());

    // FeatherPrint: generate toolpaths for open-manifold layers.
    // open_polylines contains polylines that the slicer could not close into polygons —
    // these correspond to layers where the mesh has an open boundary edge (e.g. a cutout).
    // Each open arc is a fragment of the outer skin ring; all must be processed so that every
    // arc carries its stringer traces and whip terminals.  A synthetic SliceLayerPart is
    // created per arc so the pipeline treats each independently.
    if (settings_.get<EFillMethod>("infill_pattern") == EFillMethod::FEATHERPRINT
        && ! layer->open_polylines.empty())
    {
        // ---- Step 1: unified bounding-box centroid across all open polylines ----
        coord_t uc_min_x{}, uc_max_x{}, uc_min_y{}, uc_max_y{};
        {
            bool first = true;
            for (const OpenPolyline& poly : layer->open_polylines)
                for (const Point2LL& p : poly)
                {
                    if (first) { uc_min_x = uc_max_x = p.X; uc_min_y = uc_max_y = p.Y; first = false; }
                    if (p.X < uc_min_x) uc_min_x = p.X; if (p.X > uc_max_x) uc_max_x = p.X;
                    if (p.Y < uc_min_y) uc_min_y = p.Y; if (p.Y > uc_max_y) uc_max_y = p.Y;
                }
        }
        const Point2LL centroid((uc_min_x + uc_max_x) / 2, (uc_min_y + uc_max_y) / 2);

        // ---- Step 2: orient each arc (CCW in math = positive virtual-closed area) ----
        struct OrientedArc
        {
            OpenPolyline poly;
            double       start_angle{}; // angle of first point from centroid, for sorting
        };
        std::vector<OrientedArc> arcs;
        for (const OpenPolyline& poly : layer->open_polylines)
        {
            if (poly.size() < 2) continue;
            Polygon virt;
            for (const auto& p : poly) virt.push_back(p);
            OrientedArc oa;
            if (virt.area() < 0.0)
            {
                oa.poly.getPoints().resize(poly.size());
                std::reverse_copy(poly.begin(), poly.end(), oa.poly.getPoints().begin());
            }
            else
            {
                oa.poly.getPoints().assign(poly.begin(), poly.end());
            }
            double dx = static_cast<double>(oa.poly[0].X - centroid.X);
            double dy = static_cast<double>(oa.poly[0].Y - centroid.Y);
            oa.start_angle = std::atan2(dy, dx);
            arcs.push_back(std::move(oa));
        }
        if (arcs.empty()) return;
        std::sort(arcs.begin(), arcs.end(),
            [](const OrientedArc& a, const OrientedArc& b) { return a.start_angle < b.start_angle; });

        // ---- Step 3: build full virtual ring (arcs in order + implicit gap chords) ----
        // Concatenating all arc points into one polygon; the "closing segment" from last
        // point back to first, plus the inter-arc transitions, form the gap chords.
        Polygon full_ring;
        std::vector<size_t> arc_vertex_start(arcs.size());
        for (size_t i = 0; i < arcs.size(); i++)
        {
            arc_vertex_start[i] = full_ring.size();
            for (const auto& p : arcs[i].poly)
                full_ring.push_back(p);
        }

        // ---- Step 4: cumulative arc-length along the full ring ----
        const size_t n_ring = full_ring.size();
        std::vector<double> cum_len(n_ring, 0.0);
        for (size_t i = 1; i < n_ring; i++)
        {
            double dx = full_ring[i].X - full_ring[i - 1].X;
            double dy = full_ring[i].Y - full_ring[i - 1].Y;
            cum_len[i] = cum_len[i - 1] + std::sqrt(dx * dx + dy * dy);
        }
        {
            double dx = full_ring[0].X - full_ring[n_ring - 1].X;
            double dy = full_ring[0].Y - full_ring[n_ring - 1].Y;
            // full_ring_total includes the closing chord
        }
        double full_ring_total = 0.0;
        {
            double dx = full_ring[0].X - full_ring[n_ring - 1].X;
            double dy = full_ring[0].Y - full_ring[n_ring - 1].Y;
            full_ring_total = cum_len[n_ring - 1] + std::sqrt(dx * dx + dy * dy);
        }

        // ---- Step 5: Phase Origin — nearest point on the full ring to the fixed world seed
        // point (Spec REV 2.2 Anchor Distribution). Plain point-to-segment min-distance scan,
        // no centroid/ray/angle involved — replaces the old +X-from-centroid ray-cast, which
        // took the first crossing found in vertex order with no nearest-to-centroid
        // disambiguation and could silently resolve to the wrong point on a non-star-convex
        // ring (a ray crossing more than once). Same nearest-point technique as
        // ArcParam::nearestArcPos, duplicated here since this full_ring is WallsComputation's
        // own flat point array, not an ArcParam.
        double full_ring_arc_ref = 0.0;
        {
            double best_d2 = -1.0;
            for (size_t i = 0; i < n_ring; i++)
            {
                size_t j = (i + 1) % n_ring;
                double ax = static_cast<double>(full_ring[i].X), ay = static_cast<double>(full_ring[i].Y);
                double ex = static_cast<double>(full_ring[j].X - full_ring[i].X);
                double ey = static_cast<double>(full_ring[j].Y - full_ring[i].Y);
                double seg_len2 = ex * ex + ey * ey;
                double t = (seg_len2 > 1e-9)
                    ? ((static_cast<double>(fp_phase_origin_.X) - ax) * ex + (static_cast<double>(fp_phase_origin_.Y) - ay) * ey) / seg_len2
                    : 0.0;
                t = std::max(0.0, std::min(1.0, t));
                double px = ax + t * ex, py = ay + t * ey;
                double dx = static_cast<double>(fp_phase_origin_.X) - px;
                double dy = static_cast<double>(fp_phase_origin_.Y) - py;
                double d2 = dx * dx + dy * dy;
                if (best_d2 < 0.0 || d2 < best_d2)
                {
                    best_d2 = d2;
                    double seg_end = (j == 0) ? full_ring_total : cum_len[j];
                    full_ring_arc_ref = cum_len[i] + t * (seg_end - cum_len[i]);
                }
            }
        }

        // ---- Step 6: generate toolpaths for each arc ----
        FeatherPrintGenerator fp;
        for (size_t ai = 0; ai < arcs.size(); ai++)
        {
            FeatherPrintGenerator::OpenLayerParams params;
            params.centroid           = centroid;
            params.full_ring_total    = full_ring_total;
            params.full_ring_arc_ref  = full_ring_arc_ref;
            params.arc_start_in_ring  = cum_len[arc_vertex_start[ai]];

            const bool is_flange_layer = (fp_flange_start_layer_ >= 0 && layer_nr_ >= fp_flange_start_layer_);
            const bool is_former_layer = (! is_flange_layer && fp_former_ramp_ >= 0);
            VariableWidthLines wl;
            if (is_flange_layer)
            {
                const int ramp_index = static_cast<int>(layer_nr_ - fp_flange_start_layer_);
                wl = fp.generateFlangeOpen(arcs[ai].poly, layer->printZ, settings_, ramp_index, fp_helix_phase_, params);
            }
            else if (is_former_layer)
            {
                wl = fp.generateFormerOpen(arcs[ai].poly, layer->printZ, settings_, fp_former_ramp_, fp_helix_phase_, params);
            }
            else
            {
                wl = fp.generateOpen(arcs[ai].poly, layer->printZ, settings_, fp_helix_phase_, params);
            }

            if (wl.empty()) continue;

            coord_t min_x = arcs[ai].poly[0].X, max_x = arcs[ai].poly[0].X;
            coord_t min_y = arcs[ai].poly[0].Y, max_y = arcs[ai].poly[0].Y;
            for (const auto& p : arcs[ai].poly)
            {
                if (p.X < min_x) min_x = p.X; if (p.X > max_x) max_x = p.X;
                if (p.Y < min_y) min_y = p.Y; if (p.Y > max_y) max_y = p.Y;
            }
            Polygon bbox_poly;
            bbox_poly.push_back(Point2LL(min_x, min_y));
            bbox_poly.push_back(Point2LL(max_x, min_y));
            bbox_poly.push_back(Point2LL(max_x, max_y));
            bbox_poly.push_back(Point2LL(min_x, max_y));
            Shape bbox_shape;
            bbox_shape.push_back(bbox_poly);

            SliceLayerPart synthetic;
            synthetic.outline        = SingleShape(std::move(bbox_shape));
            synthetic.print_outline  = synthetic.outline;
            synthetic.inner_area     = Shape{};
            synthetic.wall_toolpaths = { std::move(wl) };
            layer->parts.push_back(std::move(synthetic));
        }

        // ---- Step 7: Punchout (Spec REV 3.3) — one per ring GAP, not per arc ----
        // A gap sits between the END of one open polyline and the START of the NEXT one in
        // ring order (arcs are already sorted by angle in Step 2), wrapping around at the
        // end of the list. This is generally NOT the same as one arc's own front()/back():
        // with more than one hole open at the same layer, a single open polyline typically
        // runs from one hole's edge, all the way around through solid wall material, to a
        // DIFFERENT hole's edge — pairing an arc's own two endpoints (as a previous attempt
        // did) bridges across unrelated holes through solid material instead of spanning an
        // actual hole. With exactly one arc (one hole on this layer), the wrap-around gap
        // (arcs[0].back() -> arcs[0].front(), i == (i+1)%1) correctly reduces to that one
        // hole's own two ends, which is why the single-hole case alone doesn't expose this.
        if (settings_.get<bool>("featherprint_punchout_enabled") && ! arcs.empty())
        {
            FeatherPrintGenerator fp_punchout;
            for (size_t gi = 0; gi < arcs.size(); gi++)
            {
                const OpenPolyline& prev_wall_poly = arcs[gi].poly;
                const OpenPolyline& next_wall_poly = arcs[(gi + 1) % arcs.size()].poly;
                const Point2LL gap_start = prev_wall_poly.back();
                const Point2LL gap_end   = next_wall_poly.front();

                FeatherPrintGenerator::OpenLayerParams gap_params;
                gap_params.centroid = centroid; // the only field generatePunchout() reads

                // Contour matching (Spec REV 3.3): find this gap's hole-span record (nearest
                // endpoint-pair match — not assuming gi lines up with the pre-pass's own gap
                // enumeration order), threaded in via the constructor
                // (SliceMeshStorage::fp_punchout_gap_spans). No weld/blend toward the wall
                // corner is applied here — that was tried and removed (see generatePunchout's
                // own doc comment): it made the Punchout harder to break away.
                std::vector<Point2LL> contour_pts;
                bool half_width_ends = false;
                if (! fp_punchout_gap_spans_.empty())
                {
                    double best_d2 = -1.0;
                    const SliceMeshStorage::FpPunchoutGapSpan* best = nullptr;
                    for (const auto& span : fp_punchout_gap_spans_)
                    {
                        const double dx1 = static_cast<double>(span.start.X - gap_start.X);
                        const double dy1 = static_cast<double>(span.start.Y - gap_start.Y);
                        const double dx2 = static_cast<double>(span.end.X - gap_end.X);
                        const double dy2 = static_cast<double>(span.end.Y - gap_end.Y);
                        const double d2 = dx1 * dx1 + dy1 * dy1 + dx2 * dx2 + dy2 * dy2;
                        if (best == nullptr || d2 < best_d2) { best_d2 = d2; best = &span; }
                    }

                    // Loft between the real wall contours
                    // immediately below/above this hole (SliceMeshStorage::fp_punchout_gap_spans'
                    // contour_below/contour_above, populated once per hole in the pre-pass),
                    // evaluated at this Layer's own Z. Empty (falls back to a straight chord in
                    // generatePunchout) if either side's contour data is unavailable.
                    if (best != nullptr && ! best->contour_below.empty() && best->contour_below.size() == best->contour_above.size()
                        && best->z_above > best->z_below)
                    {
                        const double t = std::clamp(
                            static_cast<double>(layer->printZ - best->z_below) / static_cast<double>(best->z_above - best->z_below),
                            0.0, 1.0);
                        contour_pts.reserve(best->contour_below.size());
                        for (size_t i = 0; i < best->contour_below.size(); i++)
                        {
                            contour_pts.emplace_back(
                                static_cast<coord_t>(std::llround(best->contour_below[i].X + t * (best->contour_above[i].X - best->contour_below[i].X))),
                                static_cast<coord_t>(std::llround(best->contour_below[i].Y + t * (best->contour_above[i].Y - best->contour_below[i].Y))));
                        }
                    }

                    // Half-width ends (Spec REV 3.3): the Layer forming the literal top of the
                    // hole's own span, and the one forming its literal bottom, print the whole
                    // Punchout at half nominal width to weaken those connections for easier
                    // removal — unrelated to Contour Matching above.
                    if (best != nullptr && (layer_nr_ == best->hole_bottom || layer_nr_ == best->hole_top))
                        half_width_ends = true;
                }

                VariableWidthLines punchout_wl = fp_punchout.generatePunchout(prev_wall_poly, next_wall_poly, layer->printZ, settings_, gap_params, contour_pts, half_width_ends);
                if (punchout_wl.empty()) continue;

                coord_t min_x = std::min(gap_start.X, gap_end.X), max_x = std::max(gap_start.X, gap_end.X);
                coord_t min_y = std::min(gap_start.Y, gap_end.Y), max_y = std::max(gap_start.Y, gap_end.Y);
                Polygon bbox_poly;
                bbox_poly.push_back(Point2LL(min_x, min_y));
                bbox_poly.push_back(Point2LL(max_x, min_y));
                bbox_poly.push_back(Point2LL(max_x, max_y));
                bbox_poly.push_back(Point2LL(min_x, max_y));
                Shape bbox_shape;
                bbox_shape.push_back(bbox_poly);

                SliceLayerPart synthetic;
                synthetic.outline        = SingleShape(std::move(bbox_shape));
                synthetic.print_outline  = synthetic.outline;
                synthetic.inner_area     = Shape{};
                synthetic.wall_toolpaths = { std::move(punchout_wl) };
                layer->parts.push_back(std::move(synthetic));
            }
        }
    }
}

void WallsComputation::generateSpiralInsets(SliceLayerPart* part, coord_t line_width_0, coord_t wall_0_inset, bool recompute_outline_based_on_outer_wall)
{
    part->spiral_wall = part->outline.offset(-line_width_0 / 2 - wall_0_inset);

    // Optimize the wall. This prevents buffer underruns in the printer firmware, and reduces processing time in CuraEngine.
    const ExtruderTrain& train_wall = settings_.get<ExtruderTrain&>("wall_0_extruder_nr");
    part->spiral_wall = Simplify(train_wall.settings_).polygon(part->spiral_wall);
    part->spiral_wall.removeDegenerateVerts();
    if (recompute_outline_based_on_outer_wall)
    {
        part->print_outline = part->spiral_wall.offset(line_width_0 / 2, ClipperLib::jtSquare);
    }
    else
    {
        part->print_outline = part->outline;
    }
}

} // namespace cura
