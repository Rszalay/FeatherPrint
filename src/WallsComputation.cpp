// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "WallsComputation.h"

#include <fstream>
#include <iostream>

#include <fmt/format.h>
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

WallsComputation::WallsComputation(const Settings& settings, const LayerIndex layer_nr, double fp_helix_phase)
    : settings_(settings)
    , layer_nr_(layer_nr)
    , fp_helix_phase_(fp_helix_phase)
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
            part->wall_toolpaths = { fp.generate(gen_outline, z_coord, settings_, fp_helix_phase_) };
            part->inner_area    = Shape{};
            part->print_outline = part->outline;

            part->outline = SingleShape{ Simplify(settings_).polygon(part->outline) };
            part->print_outline = part->outline;
            return;
        }
        // layer_nr_ < init_bottom: fall through to normal wall generation below
    }

    size_t wall_count = settings_.get<size_t>("wall_line_count");
    if (wall_count == 0) // Early out if no walls are to be generated
    {
        part->print_outline = part->outline;
        part->inner_area = part->outline;
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
        }
    }
    else
    {
        WallToolPaths wall_tool_paths(part->outline, line_width_0, line_width_x, wall_count, wall_0_inset, settings_, layer_nr_, section_type);
        part->wall_toolpaths = wall_tool_paths.getToolPaths();
        part->inner_area = wall_tool_paths.getInnerContour();
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

        // ---- Step 5: reference ray (+X from centroid) position in full ring ----
        double full_ring_arc_ref = 0.0;
        for (size_t i = 0; i < n_ring; i++)
        {
            size_t j = (i + 1) % n_ring;
            double ax = full_ring[i].X - centroid.X, ay = full_ring[i].Y - centroid.Y;
            double bx = full_ring[j].X - centroid.X, by = full_ring[j].Y - centroid.Y;
            if ((ay <= 0.0 && by > 0.0) || (ay > 0.0 && by <= 0.0))
            {
                double t  = ay / (ay - by);
                double ix = ax + t * (bx - ax);
                if (ix > 0.0)
                {
                    double seg_end = (j == 0) ? full_ring_total : cum_len[j];
                    full_ring_arc_ref = cum_len[i] + t * (seg_end - cum_len[i]);
                    break;
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

            VariableWidthLines wl = fp.generateOpen(arcs[ai].poly, layer->printZ, settings_, fp_helix_phase_, params);
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
