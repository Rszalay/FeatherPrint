// Copyright (c) 2026 FeatherPrint Corrugated
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "corrugated/VbctAdapter.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream> // TEMPORARY - dumpLayerContoursForInvestigation, remove alongside it
#include <limits>
#include <stdexcept>

#include <spdlog/spdlog.h>

#include "geometry/OpenLinesSet.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Point2LL.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"
#include "settings/Settings.h"
#include "sliceDataStorage.h"
#include "pipeline.hpp" // Vendored VBCT.
#include "stage5.hpp" // Individual pipeline stages - needed by computeAnchorsForMesh's cross-layer
#include "stage6.hpp" // continuity pre-pass (spec REV 1.4 S:5.3), which calls Stages 1-9 directly
#include "stage7.hpp" // rather than via run_full_pipeline, since it needs the Stage 9 domain data
#include "stage8.hpp" // itself (Ring/Chain/Glob composition, wall point arrays), not just Stage
#include "stage9.hpp" // 10's final stringers.
#include "wall_ab_assignment.hpp" // vbct::assign_wall_ab - Chain Wall A/B canonical assignment

namespace cura
{
namespace VbctAdapter
{

namespace
{

// This engine's native unit is microns (Point2LL, int64). VBCT's Contour is fixed-point with an
// ambiguous "input unit" (spec REV 2.1 S:2.3's SCALE=1000 doesn't name one) - going through
// vbct::from_float_points in millimeters, VBCT's own established convention (its Python
// reference and test fixtures are mm-scale), sidesteps that ambiguity entirely: converting
// microns -> mm -> VBCT's own integer snapping, then VBCT's output mm -> microns on the way
// back, is correct regardless of what VBCT's internal fixed-point representation actually means.
constexpr double kMicronsPerMm = 1000.0;

vbct::Point2 toVbctMm(const Point2LL& p)
{
    return vbct::Point2{ static_cast<double>(p.X) / kMicronsPerMm, static_cast<double>(p.Y) / kMicronsPerMm };
}

Point2LL fromVbctMm(const vbct::Point2& p)
{
    return Point2LL(static_cast<coord_t>(std::llround(p.x * kMicronsPerMm)), static_cast<coord_t>(std::llround(p.y * kMicronsPerMm)));
}

} // namespace

std::vector<vbct::Contour> shapeToContours(const Shape& shape)
{
    std::vector<vbct::Contour> contours;
    contours.reserve(shape.size());
    size_t next_id = 0;
    for (const Polygon& polygon : shape)
    {
        if (polygon.size() < 3)
        {
            continue;
        }
        std::vector<vbct::Point2> points;
        points.reserve(polygon.size());
        for (const Point2LL& p : polygon)
        {
            points.push_back(toVbctMm(p));
        }
        contours.push_back(vbct::from_float_points(points, "ring" + std::to_string(next_id++)));
    }
    return contours;
}

// TEMPORARY debug instrumentation for the FPTF-45 hole/chain-domain-instability
// investigation. Dumps every layer's own real contour input to VBCT (post-Simplify,
// post-meshfix, exactly what run_full_pipeline actually receives) to a JSON file matching the
// VBCT Cpp reference project's own case_library.json schema, so it can be fed straight into
// that project's diagnostic tools without re-deriving contours from the raw STL. One file per
// distinct Z height (overwrites on a repeat call for the same layer, e.g. the cross-layer
// continuity pre-pass's own shadow-copy re-run - harmless, since the content is the same input).
// Remove this function and its two call sites in corrugate()/corrugateLinkedSkin() once the
// investigation concludes.
void dumpLayerContoursForInvestigation(const std::vector<vbct::Contour>& contours, double x, double threshold_mm, double spacing_mm, double z_mm)
{
    static const std::string kDumpDir
        = "C:\\Users\\ricsz\\AppData\\Local\\Temp\\claude\\c--Users-ricsz-source-CoWork-Projects-Scratch-VBCT-Cpp\\"
          "74ac32a8-7672-4766-90f4-3fe3fe194d84\\scratchpad\\real_layer_dump\\";
    std::ofstream dump_out(kDumpDir + "layer_z" + std::to_string(z_mm) + ".json");
    if (! dump_out)
    {
        return; // directory doesn't exist on this machine, or investigation already concluded - silently skip
    }
    dump_out << "{\"contours\":[";
    for (size_t ci = 0; ci < contours.size(); ++ci)
    {
        if (ci)
        {
            dump_out << ",";
        }
        dump_out << "[";
        for (size_t pi = 0; pi < contours[ci].points.size(); ++pi)
        {
            if (pi)
            {
                dump_out << ",";
            }
            dump_out << "[" << (static_cast<double>(contours[ci].points[pi].x) / vbct::SCALE) << "," << (static_cast<double>(contours[ci].points[pi].y) / vbct::SCALE)
                      << "]";
        }
        dump_out << "]";
    }
    dump_out << "],\"x\":" << x << ",\"threshold\":" << threshold_mm << ",\"spacing\":" << spacing_mm << ",\"z\":" << z_mm << "}";
}

// TEMPORARY debug instrumentation for the layer-395 area-collapse investigation (2026-09-13):
// appends one CSV row per call site per layer, gated to a single target Z so this doesn't spam
// every layer of every print. Pure geometry (Shape::area(), no VBCT calls, nothing that can
// throw) - safe to call from anywhere, unlike dumpWallAbAssignmentForInvestigation before it.
// Remove this function and its call sites once the investigation concludes.
void dumpAreaForInvestigation(const std::string& stage, const Shape& shape, double z_mm)
{
    constexpr double kTargetZMm = 84.35; // retargeted to layer 843 (trailing-edge near-duplicate-point investigation)
    constexpr double kZEpsilonMm = 0.001;
    if (std::abs(z_mm - kTargetZMm) > kZEpsilonMm)
    {
        return;
    }
    static const std::string kDumpPath
        = "C:\\Users\\ricsz\\AppData\\Local\\Temp\\claude\\c--Users-ricsz-source-CoWork-Projects-Scratch-VBCT-Cpp\\"
          "74ac32a8-7672-4766-90f4-3fe3fe194d84\\scratchpad\\area_investigation_layer395.csv";
    std::ofstream dump_out(kDumpPath, std::ios::app);
    if (! dump_out)
    {
        return; // directory doesn't exist on this machine, or investigation already concluded - silently skip
    }
    const double area_mm2 = shape.area() / 1e6; // Shape::area() is in micron^2 (coord_t units)
    // Point count alongside area: a whole-surface's worth of real curve detail collapsing into a
    // single straight segment barely moves total enclosed area (a thin airfoil's camber deviates
    // only slightly from its own chord), so area alone can miss exactly this failure mode - see
    // this investigation's own findings. Total vertex count across every polygon in the shape
    // catches it directly instead.
    size_t total_points = 0;
    for (const Polygon& poly : shape)
    {
        total_points += poly.size();
    }
    dump_out << stage << "," << z_mm << "," << shape.size() << "," << total_points << "," << area_mm2 << "\n";
}

// TEMPORARY debug instrumentation, same investigation/gating/safety as dumpAreaForInvestigation
// above - dumps a Shape's own raw point coordinates (one JSON file per call site) so the actual
// polygon shape can be inspected/plotted, not just its area and point count. Remove alongside
// dumpAreaForInvestigation once the investigation concludes.
void dumpShapePointsForInvestigation(const std::string& stage, const Shape& shape, double z_mm)
{
    constexpr double kTargetZMm = 84.35; // retargeted to layer 843 (trailing-edge near-duplicate-point investigation)
    constexpr double kZEpsilonMm = 0.001;
    if (std::abs(z_mm - kTargetZMm) > kZEpsilonMm)
    {
        return;
    }
    const std::string kDumpPath
        = "C:\\Users\\ricsz\\AppData\\Local\\Temp\\claude\\c--Users-ricsz-source-CoWork-Projects-Scratch-VBCT-Cpp\\"
          "74ac32a8-7672-4766-90f4-3fe3fe194d84\\scratchpad\\shape_points_" + stage + ".json";
    std::ofstream dump_out(kDumpPath);
    if (! dump_out)
    {
        return;
    }
    dump_out << "{\"z_mm\":" << z_mm << ",\"polygons\":[";
    for (size_t pi = 0; pi < shape.size(); ++pi)
    {
        if (pi)
        {
            dump_out << ",";
        }
        dump_out << "[";
        const Polygon& poly = shape[pi];
        for (size_t vi = 0; vi < poly.size(); ++vi)
        {
            if (vi)
            {
                dump_out << ",";
            }
            dump_out << "[" << (static_cast<double>(poly[vi].X) / 1000.0) << "," << (static_cast<double>(poly[vi].Y) / 1000.0) << "]";
        }
        dump_out << "]";
    }
    dump_out << "]}";
}

Shape filterDeminimisHoles(const Shape& shape, const std::vector<DeminimisHoleIdentity>& suppressed_hole_identities, std::vector<std::vector<Point2LL>>& ignored_hole_loops)
{
    ignored_hole_loops.clear();
    if (suppressed_hole_identities.empty())
    {
        return shape;
    }

    Shape result;
    for (const Polygon& polygon : shape)
    {
        // Only ever a candidate for suppression if it's actually a hole (negative signed area,
        // Clipper convention) - an outer boundary is never suppressed regardless of how it
        // compares to suppressed_hole_identities' own contents.
        if (polygon.size() >= 3 && polygon.area() < 0.0)
        {
            Point2LL sum(0, 0);
            for (const Point2LL& p : polygon)
            {
                sum += p;
            }
            const Point2LL identity_point = sum / static_cast<coord_t>(polygon.size());

            // Match cap: unlike chain_domain_wall_identities' own nearest-match (always exactly
            // one real domain to match per layer, so "always matches something" is correct there),
            // this is a presence/absence check against a set that may legitimately contain zero
            // entries relevant to *this* hole - e.g. a real, non-suppressed structural hole
            // (a Ring's own second wall) sharing a Shape with several small suppressed holes
            // elsewhere. Without a cap, "nearest of the suppressed set" always finds *something*
            // once the set is non-empty, incorrectly sweeping up every other hole in the same Shape
            // too (confirmed as a real bug via direct user report - an 8-small-hole Ring collapsed
            // to a single outer boundary with the true central hole gone as well; a first fix
            // capping by the *candidate* hole's own bounding-box diagonal still let this exact case
            // through, since a large hole's own diagonal can easily exceed the distance to a small,
            // genuinely unrelated hole elsewhere in the same part). A fixed absolute cap instead: a
            // re-derivation of the *same* hole (across the tiny geometry changes
            // buildCorrugationInput's own infill_overlap_mm growth or Wall Strip's own displacement
            // introduce between where an identity was captured and where it's matched again) drifts
            // by, at most, a couple of line widths - nowhere near the multi-millimeter-plus spacing
            // between genuinely distinct holes in any realistic part. Deliberately not
            // settings-derived (this low-level function has no access to settings) - matches this
            // project's own established fixed-tolerance-cap precedent elsewhere (e.g.
            // WallStrip::findNearestOpenWallEndpoint's own infill_line_width*4-scale cap).
            constexpr double kMaxHoleIdentityDriftMicrons = 2000.0; // 2mm
            constexpr double kMaxHoleIdentityDriftMicronsSq = kMaxHoleIdentityDriftMicrons * kMaxHoleIdentityDriftMicrons;

            const DeminimisHoleIdentity* matched = nullptr;
            double best_dist_sq = std::numeric_limits<double>::max();
            for (const DeminimisHoleIdentity& candidate : suppressed_hole_identities)
            {
                const double dx = static_cast<double>(identity_point.X - candidate.identity_point.X);
                const double dy = static_cast<double>(identity_point.Y - candidate.identity_point.Y);
                const double dist_sq = dx * dx + dy * dy;
                if (dist_sq < best_dist_sq && dist_sq <= kMaxHoleIdentityDriftMicronsSq)
                {
                    best_dist_sq = dist_sq;
                    matched = &candidate;
                }
            }
            if (matched != nullptr)
            {
                std::vector<Point2LL> loop;
                loop.reserve(polygon.size());
                for (const Point2LL& p : polygon)
                {
                    loop.push_back(p);
                }
                ignored_hole_loops.push_back(std::move(loop));
                continue; // drop this hole from the result - it's suppressed
            }
        }
        result.push_back(polygon);
    }
    return result;
}

OpenLinesSet stage10ToLines(const vbct::Stage10Result& result, OpenLinesSet* transition_lines_out)
{
    if (transition_lines_out != nullptr)
    {
        *transition_lines_out = OpenLinesSet();
    }
    OpenLinesSet lines;
    for (const vbct::DomainStringers& domain : result.domains)
    {
        // Transition Layer (spec REV 3.3/3.6/5.10): route a transitioned domain's own lines to
        // the caller's own separate out-parameter instead, so it can apply Cura's own native
        // bridging print settings to just these lines - see this function's own doc comment.
        OpenLinesSet* const target = (transition_lines_out != nullptr && domain.is_transition_layer) ? transition_lines_out : &lines;
        for (const vbct::Stringer& stringer : domain.stringers)
        {
            for (const vbct::StringerEdge& piece : stringer.pieces)
            {
                OpenPolyline line;
                line.push_back(fromVbctMm(piece.first));
                line.push_back(fromVbctMm(piece.second));
                target->push_back(line);
            }
        }
    }
    return lines;
}

std::optional<OpenLinesSet> corrugate(
    const Shape& infill_area,
    const double x,
    const double threshold,
    const double spacing,
    const coord_t z,
    const double crossover_pitch_mm,
    const std::optional<double> anchor_t0_frac,
    const std::optional<double> other_wall_t0_frac,
    const std::optional<bool> reverse_canonical_wall,
    const bool crosshatch_enabled,
    const std::optional<Point2LL> chain_anchor_point,
    const std::optional<double> chain_left_near_t_frac,
    const std::optional<double> chain_left_far_t_frac,
    const std::optional<double> chain_right_near_t_frac,
    const std::optional<double> chain_right_far_t_frac,
    const std::vector<DeminimisHoleIdentity>& suppressed_hole_identities,
    const TransitionLayerRequest& transition_layer_request,
    OpenLinesSet* transition_lines_out)
{
    // TEMPORARY (layer-395 area-collapse investigation): z_mm isn't computed until further down,
    // and this function can return before reaching that point - compute a local copy up front so
    // the area dumps below always fire regardless of where this call ends up returning.
    const double dbg_z_mm = static_cast<double>(z) / kMicronsPerMm;
    dumpAreaForInvestigation("corrugate_infill_area_input", infill_area, dbg_z_mm);

    // De Minimis Hole Threshold (spec REV 3.0/3.5/5.9): applies this layer's own already-decided
    // suppression set (VbctAdapter::computeAnchorsForMesh's own pre-pass) before VBCT ever sees
    // the geometry - see filterDeminimisHoles' own doc comment. A no-op (returns infill_area
    // unchanged, ignored_hole_loops stays empty) when suppressed_hole_identities is empty, the
    // common case.
    std::vector<std::vector<Point2LL>> ignored_hole_loops;
    const Shape filtered_infill_area = filterDeminimisHoles(infill_area, suppressed_hole_identities, ignored_hole_loops);
    dumpAreaForInvestigation("corrugate_post_filterDeminimisHoles", filtered_infill_area, dbg_z_mm);
    const std::vector<vbct::Contour> contours = shapeToContours(filtered_infill_area);
    if (contours.empty())
    {
        return std::nullopt;
    }
    // Convert the two length-scale parameters into VBCT's mm convention, same rationale as
    // toVbctMm/fromVbctMm above.
    const double threshold_mm = threshold / kMicronsPerMm;
    const double spacing_mm = spacing / kMicronsPerMm;

    // Crossover Pitch (2026-09-15 redesign, replacing the old phase_rate "wraps per mm" setting -
    // see VbctAdapter.h's own crossover_pitch_mm doc comment for the full rationale): phase_offset
    // is now the *raw*, undivided crossover-phase argument z_mm / (2 * crossover_pitch_mm), not a
    // pre-wrapped fraction - every formula that consumes it (stage10.cpp) divides it by that
    // formula's own domain-specific denominator before use, which is what makes the base/
    // crosshatch family coincidence period equal crossover_pitch_mm for every domain regardless of
    // its own stringer count, instead of the old per-domain-N-dependent period that let different
    // domains reach their first coincidence at different, uncoordinated Z heights even under the
    // same shared rate (confirmed as a real defect on a real multi-Chain-domain part).
    // crossover_pitch_mm=0 (the default) collapses this to VBCT's own fixed t=0 anchor, same as
    // the old phase_rate=0.
    const double z_mm = static_cast<double>(z) / kMicronsPerMm;
    // Crosshatch degeneracy bias (2026-09-15, Multi-Domain Two-Hole crossing-glitch
    // investigation): the base and crosshatch stringer families - two counter-rotating helices -
    // become exactly, bit-for-bit coincident whenever 2*phase_offset lands exactly on an integer
    // (confirmed on real capture: every base-family theta bit-identical to its crosshatch-family
    // counterpart at those layers) - now a deliberate, frequent, desired event (every
    // crossover_pitch_mm of Z, synchronized across every domain), not a rare accident. The two
    // families' relative handedness genuinely must flip on either side of such a crossing - that
    // part is real, unavoidable geometry - but build_domain_events' own event sort (stage10.cpp)
    // has no way to know which side of the (measure-zero) crossing point a genuine tie should
    // resolve toward, so an *exact* tie could resolve either way inconsistently between layers
    // even with a stable sort backing it, confirmed on real print as the crossing layer's own
    // pattern reading as spuriously mirrored relative to its neighbors even though the layer
    // itself is well-formed. A tiny, fixed bias - far smaller than any real per-layer phase step -
    // keeps phase_offset off the exact degenerate value entirely, so every crossing resolves the
    // same consistent way (toward whichever side this bias' own sign favors) instead of leaving it
    // to the sort's own tie-break. 1e-7 (in raw phase_offset units, i.e. half-cycles) is many
    // orders of magnitude below the coarsest realistic per-layer phase step, so no real layer's
    // own geometry is perceptibly affected by it. Gated on crossover_pitch_mm != 0.0:
    // crossover_pitch_mm=0 is a deliberate, permanent "crosshatch fully degenerate" configuration
    // (both families locked exactly coincident forever, matching
    // CrosshatchProducesNoExtraStringersAtZeroPhaseOffset's own expectation) rather than a
    // transient crossing during ongoing rotation - the bias must not disturb that intentionally
    // exact case.
    constexpr double kCrosshatchDegeneracyBias = 1e-7;
    const double phase_offset = crossover_pitch_mm != 0.0 ? (z_mm / (2.0 * crossover_pitch_mm) + kCrosshatchDegeneracyBias) : 0.0;
    dumpLayerContoursForInvestigation(contours, x, threshold_mm, spacing_mm, z_mm);

    const std::optional<vbct::Point2> chain_anchor_point_mm
        = chain_anchor_point.has_value() ? std::make_optional(toVbctMm(*chain_anchor_point)) : std::nullopt;

    // De Minimis Hole Threshold: each ignored hole's own real boundary, converted to VBCT's own mm
    // space, threaded through as extra stringer-clip edges - see run_stage10's own doc comment for
    // exactly how these get used (appended to every domain's own boundary_edges, since
    // clip_stringer/point_in_domain are pure even-odd parity tests with no connectivity
    // assumptions).
    std::vector<std::vector<vbct::Point2>> extra_clip_loops_mm;
    extra_clip_loops_mm.reserve(ignored_hole_loops.size());
    for (const std::vector<Point2LL>& loop : ignored_hole_loops)
    {
        std::vector<vbct::Point2> loop_mm;
        loop_mm.reserve(loop.size());
        for (const Point2LL& p : loop)
        {
            loop_mm.push_back(toVbctMm(p));
        }
        extra_clip_loops_mm.push_back(std::move(loop_mm));
    }

    // Transition Layer (spec REV 3.3/3.6/5.10): convert the request's own domain identities and
    // solid-fill spacing into VBCT's mm convention - the same unit conversion every other
    // engine-native parameter above already goes through.
    std::vector<vbct::Point2> transition_chain_domain_identities_mm;
    transition_chain_domain_identities_mm.reserve(transition_layer_request.chain_domain_identities.size());
    for (const Point2LL& p : transition_layer_request.chain_domain_identities)
    {
        transition_chain_domain_identities_mm.push_back(toVbctMm(p));
    }
    const double transition_solid_fill_spacing_mm = static_cast<double>(transition_layer_request.solid_fill_spacing) / kMicronsPerMm;

    vbct::Stage10Result result;
    try
    {
        result = vbct::run_full_pipeline(
            contours,
            x,
            threshold_mm,
            spacing_mm,
            phase_offset,
            anchor_t0_frac,
            other_wall_t0_frac,
            reverse_canonical_wall,
            crosshatch_enabled,
            chain_anchor_point_mm,
            // Arc-length fractions - dimensionless, no unit conversion needed (unlike
            // chain_anchor_point_mm above).
            chain_left_near_t_frac,
            chain_left_far_t_frac,
            chain_right_near_t_frac,
            chain_right_far_t_frac,
            extra_clip_loops_mm,
            transition_layer_request.ring,
            transition_chain_domain_identities_mm,
            transition_solid_fill_spacing_mm);
    }
    catch (const std::runtime_error& e)
    {
        // VBCT rejects input it can't handle (e.g. a self-intersecting ring after this engine's
        // own polygon simplification) via ValidationError/TriangulationError, both
        // std::runtime_error. Treat as "nothing to corrugate here" rather than crashing the
        // slice - consistent with this pattern's existing no-fallback-to-normal-infill design
        // for ineligible regions.
        spdlog::warn("VBCT rejected infill area for corrugation: {}", e.what());
        return std::nullopt;
    }

    OpenLinesSet lines = stage10ToLines(result, transition_lines_out);
    if (lines.empty() && (transition_lines_out == nullptr || transition_lines_out->empty()))
    {
        return std::nullopt;
    }
    return lines;
}

Shape insetOutline(const SliceLayerPart& part, const Settings& settings)
{
    const size_t wall_count = settings.get<size_t>("wall_line_count");
    if (wall_count == 0)
    {
        return part.outline;
    }
    const coord_t line_width_0 = settings.get<coord_t>("wall_line_width_0");
    const coord_t line_width_x = settings.get<coord_t>("wall_line_width_x");
    const coord_t wall_0_inset = settings.get<coord_t>("wall_0_inset");
    // Deliberately a plain fixed-width sum, not Arachne's variable-width bead placement -- see
    // this function's own header doc for why that's the point, not an oversight. First wall
    // reaches its own centerline at wall_0_inset + line_width_0/2 from the outline; each
    // additional wall (if wall_count > 1) adds one more line_width_x to reach the *last* wall's
    // own centerline; the corrugation region then starts half of that last wall's own width
    // further in, past its centerline - the last wall's own width is line_width_x when there's
    // more than one wall, but line_width_0 itself when wall_count is 1 (there's no "x" wall at
    // all in that case, only wall 0).
    //
    // FeatherPrint Corrugated fix: a first version of this formula only added that final "half a
    // line past the centerline" term inside the wall_count>1 branch, so at wall_count==1 the
    // total_inset stopped *exactly at* wall 0's own centerline rather than past it - confirmed via
    // direct user report (corrugation running "on top of the wall" specifically in raw outline
    // mode, at wall_line_count=1) once a separate, unrelated fix (see corrugateLinkedSkin's own
    // doc comment) removed the extra inboard offset that had been coincidentally compensating for
    // this bug. Fixed by adding the last wall's own half-width unconditionally, not just when
    // wall_count>1.
    const coord_t last_wall_width = (wall_count > 1) ? line_width_x : line_width_0;
    const coord_t total_inset = wall_0_inset + line_width_0 / 2 + (wall_count > 1 ? (static_cast<coord_t>(wall_count) - 1) * line_width_x : 0) + last_wall_width / 2;
    return part.outline.offset(-total_inset);
}

// Wall Strip (spec REV 2.6, Section 5.7): part.infill_area sits inboard of the *entire* wall stack
// by construction (it's the boundary Cura's own wall generation leaves behind), which is exactly
// right when every wall is actually printed - the corrugation welds to the innermost wall's own
// inboard face, per spec Section 3.1's own load-path framing. But once a wall is stripped
// (FffGcodeWriter::addMeshPartToGCode's own pre-preProcessInsets step, before this function's
// caller ever runs), infill_area no longer means that for the stripped side: there's no wall there
// to weld against any more, and leaving the corrugation attached to infill_area's own unchanged
// boundary leaves a real, physical gap the full width of the removed wall stack - confirmed
// directly by the user on a real print ("the wall is stripped correctly, but the wall links do not
// move outboard to cover the area the wall previously covered").
//
// Fix: for whichever side is stripped, use part.outline's own corresponding contour (the part's
// true, unshelled model boundary - always available, untouched by wall stripping, unlike trying to
// recover the removed wall's own exact former centerline from wall_toolpaths after the fact) in
// place of infill_area's own for that side, so the corrugation's own wall-following path runs at
// the true outer surface instead - actually replacing the wall's structural role, not merely
// filling up to where its innermost line used to be. Falls back to infill_area entirely
// unmodified whenever the shape doesn't look like a simple single-hole Ring (not exactly two
// polygons in both infill_area and outline) - the same "never guess, fall back to prior behavior"
// fail-safe convention stripRingWalls itself already applies (WallStrip.cpp), rather than risk
// picking the wrong contour for a topology this pass was never scoped to handle.
namespace
{

Shape expandCorrugationInputForStrippedWalls(const SliceLayerPart& part, const Settings& settings, const bool strip_wall_a, const bool strip_wall_b)
{
    const Shape& infill = part.infill_area;
    const Shape& outline = part.outline;
    if (infill.size() != 2 || outline.size() != 2)
    {
        return infill;
    }

    // Larger enclosed area = the outer contour - same convention stripRingWalls (WallStrip.cpp)
    // already uses for the identical "which of these two is the hole" question, just applied here
    // to Shape/Polygon instead of ExtrusionLine.
    auto outerIndex = [](const Shape& shape) -> size_t
    {
        return std::abs(shape[0].area()) >= std::abs(shape[1].area()) ? 0 : 1;
    };
    const size_t infill_outer = outerIndex(infill);
    const size_t infill_inner = 1 - infill_outer;
    const size_t outline_outer = outerIndex(outline);
    const size_t outline_inner = 1 - outline_outer;

    // FeatherPrint Corrugated fix (Wall Strip, Chain increment): "exactly two closed contours" on
    // its own doesn't actually mean Ring - this function runs *before* VBCT ever classifies
    // anything (buildCorrugationInput's own output is what VBCT classifies), so it has no real
    // domain-kind information yet, only this contour count as a proxy. A Chain or Glob part whose
    // own cross-section happens to have two separate, non-nested loops (confirmed as a real
    // failure mode via direct user report of VBCT's own domain segmentation "breaking down" once
    // Chain Wall Strip testing began) would satisfy that proxy just as well as a genuine Ring's
    // outer+hole pair - and then get its two contours wrongly area-sorted into "outer"/"inner"
    // regardless, silently corrupting the very shape fed into VBCT afterward. A true Ring's own
    // "inner" contour is always nested *inside* its own "outer" one; two disjoint loops are not -
    // checked directly here (any point of the smaller contour landing inside the larger one) before
    // committing to the Ring-shaped substitution below, falling back to infill unmodified otherwise.
    if (! infill[infill_outer].inside(infill[infill_inner].front()) || ! outline[outline_outer].inside(outline[outline_inner].front()))
    {
        return infill;
    }

    // The corrugation's own linked-skin path prints at infill_line_width, not a wall's own line
    // width (InfillOrderOptimizer::addToLayer routes EFillMethod::CORRUGATED through
    // mesh_config.infill_config[0] - corrugation has always used the infill line-width convention,
    // not a new one specific to Wall Strip). Running its centerline directly on part.outline (the
    // raw, zero-inset model surface) would let half that line's own width hang past the true
    // surface, running "on the skin" instead of flush with it - confirmed directly by the user on
    // a real print. Inset by half that width first, the same convention an ordinary wall's own
    // first line already uses (see insetOutline's own half-line-width term, above) to keep its
    // extrusion flush with the model surface instead of straddling it.
    //
    // A single negative offset applied to the *whole* two-contour shape together correctly shrinks
    // the outer boundary inward and grows the hole boundary outward by the same amount, because
    // Shape::offset() resolves that direction from how the two contours relate to each other (an
    // outer solid with a nested hole), not from each contour's own winding read in isolation - see
    // this file's own established comment on that combined behavior, buildCorrugationInput's
    // caller in FffGcodeWriter.cpp. Only one side may need insetting here, though, so each stripped
    // contour is offset on its own, one at a time - and offsetting a single isolated contour is a
    // purely geometric operation with no hole/solid context at all: negative always moves that
    // contour's own boundary toward its own center, positive always moves it away, regardless of
    // which way it winds (confirmed directly via a failing test, not assumed - an isolated hole
    // contour offset by the same negative sign used for the outer contour shrank toward its own
    // center instead of growing away from it, the opposite of the desired "wall's own centerline
    // sits half a line width into the solid material" convention for a hole boundary). So the two
    // contours need *opposite* signs when isolated like this: negative for the outer contour (move
    // inward, into the solid, away from open air), positive for the hole contour (move outward,
    // away from its own center, which is also "into the solid" from the hole's own perspective).
    const coord_t half_line_width = settings.get<coord_t>("infill_line_width") / 2;
    auto insetContour = [](const Polygon& contour, const coord_t signed_offset) -> std::optional<Polygon>
    {
        Shape single;
        single.push_back(contour);
        const Shape result = single.offset(signed_offset);
        return (result.size() == 1) ? std::make_optional(result[0]) : std::nullopt;
    };

    Shape result;
    if (strip_wall_a)
    {
        const std::optional<Polygon> inset = insetContour(outline[outline_outer], -half_line_width);
        // Fail-safe: an inset that degenerates (e.g. a very thin feature collapsing) falls back to
        // infill_area's own contour for this side rather than a malformed/empty boundary - matches
        // this project's established "never guess, fall back to prior behavior" convention.
        result.push_back(inset.has_value() ? *inset : infill[infill_outer]);
    }
    else
    {
        result.push_back(infill[infill_outer]);
    }
    if (strip_wall_b)
    {
        const std::optional<Polygon> inset = insetContour(outline[outline_inner], half_line_width);
        result.push_back(inset.has_value() ? *inset : infill[infill_inner]);
    }
    else
    {
        result.push_back(infill[infill_inner]);
    }
    return result;
}

} // namespace

namespace
{

double polyEdgeLength(const Point2LL& a, const Point2LL& b)
{
    return std::hypot(static_cast<double>(b.X - a.X), static_cast<double>(b.Y - a.Y));
}

// The Polygon analogue of WallStrip.cpp's own ClosedLineGeometry - a polygon's own vertices (no
// duplicated closing point here, unlike ExtrusionLine's own convention - Polygon has none to drop)
// plus each one's own cumulative arc-length position and the loop's own total length.
struct PolygonGeometry
{
    std::vector<Point2LL> pts;
    std::vector<double> pos;
    double total_length{ 0.0 };
};

PolygonGeometry analyzePolygon(const Polygon& polygon)
{
    PolygonGeometry g;
    g.pts.assign(polygon.begin(), polygon.end());
    const size_t n = g.pts.size();
    if (n < 3)
    {
        g.pts.clear();
        return g;
    }
    g.pos.resize(n);
    double acc = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        g.pos[i] = acc;
        acc += polyEdgeLength(g.pts[i], g.pts[(i + 1) % n]);
    }
    g.total_length = acc;
    return g;
}

std::pair<double, double> nearestPolygonPositionAndDistSq(const PolygonGeometry& g, const Point2LL& target)
{
    double best_pos = 0.0;
    double best_dist_sq = std::numeric_limits<double>::max();
    const size_t n = g.pts.size();
    for (size_t i = 0; i < n; ++i)
    {
        const Point2LL& p = g.pts[i];
        const Point2LL& q = g.pts[(i + 1) % n];
        const double dx = static_cast<double>(q.X - p.X);
        const double dy = static_cast<double>(q.Y - p.Y);
        const double seg_len_sq = dx * dx + dy * dy;
        double t = 0.0;
        if (seg_len_sq > 0.0)
        {
            t = (static_cast<double>(target.X - p.X) * dx + static_cast<double>(target.Y - p.Y) * dy) / seg_len_sq;
            t = std::clamp(t, 0.0, 1.0);
        }
        const double cx = static_cast<double>(p.X) + t * dx;
        const double cy = static_cast<double>(p.Y) + t * dy;
        const double ddx = cx - static_cast<double>(target.X);
        const double ddy = cy - static_cast<double>(target.Y);
        const double dist_sq = ddx * ddx + ddy * ddy;
        if (dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            best_pos = g.pos[i] + t * std::sqrt(seg_len_sq);
        }
    }
    return { best_pos, best_dist_sq };
}

Point2LL polygonPointAt(const PolygonGeometry& g, double pos)
{
    const size_t n = g.pts.size();
    pos = std::fmod(pos, g.total_length);
    if (pos < 0.0)
    {
        pos += g.total_length;
    }
    for (size_t i = 0; i < n; ++i)
    {
        const double seg_start = g.pos[i];
        const double seg_end = (i + 1 < n) ? g.pos[i + 1] : g.total_length;
        if (pos <= seg_end || i + 1 == n)
        {
            const double seg_len = seg_end - seg_start;
            const double frac = (seg_len > 0.0) ? std::clamp((pos - seg_start) / seg_len, 0.0, 1.0) : 0.0;
            const Point2LL& p = g.pts[i];
            const Point2LL& q = g.pts[(i + 1) % n];
            return { p.X + static_cast<coord_t>(std::llround(frac * static_cast<double>(q.X - p.X))), p.Y + static_cast<coord_t>(std::llround(frac * static_cast<double>(q.Y - p.Y))) };
        }
    }
    return g.pts.front();
}

// Walks forward (wrapping past total_length back to 0 if needed) from from_pos to to_pos,
// collecting the interpolated start point, every original vertex strictly in between, and the
// interpolated end point - the Polygon/points-only analogue of WallStrip.cpp's own collectArc.
std::vector<Point2LL> collectPolygonArc(const PolygonGeometry& g, double from_pos, double to_pos)
{
    std::vector<Point2LL> result;
    from_pos = std::fmod(from_pos, g.total_length);
    if (from_pos < 0.0)
    {
        from_pos += g.total_length;
    }
    to_pos = std::fmod(to_pos, g.total_length);
    if (to_pos < 0.0)
    {
        to_pos += g.total_length;
    }
    double distance_to_walk = to_pos - from_pos;
    if (distance_to_walk < 0.0)
    {
        distance_to_walk += g.total_length;
    }

    result.push_back(polygonPointAt(g, from_pos));
    const size_t n = g.pts.size();
    size_t idx = 0;
    while (idx < n && g.pos[idx] <= from_pos)
    {
        ++idx;
    }
    for (size_t step = 0; step < n; ++step)
    {
        const size_t real_idx = idx % n;
        double forward_pos = g.pos[real_idx] - from_pos;
        if (forward_pos < 0.0)
        {
            forward_pos += g.total_length;
        }
        if (forward_pos >= distance_to_walk)
        {
            break;
        }
        result.push_back(g.pts[real_idx]);
        ++idx;
    }
    result.push_back(polygonPointAt(g, to_pos));
    return result;
}

// The point at arc-length position `pos` along `g`, moved `distance` inward - i.e. toward
// whichever side of the local boundary is actually inside `containing_shape` - rather than
// directly on `g`'s own zero-inset boundary. Needed so the corrugation's own centerline (which
// prints at `infill_line_width`, not zero width) sits flush with outline's own true surface
// instead of running half a line width past it - the same real defect already found and fixed
// once for Ring (`expandCorrugationInputForStrippedWalls`'s own `insetContour`, above), confirmed
// via direct user report to reproduce identically for Chain once the zero-inset version of this
// function was deployed.
//
// Determines "inward" empirically (testing both perpendicular candidates against
// `containing_shape.inside()`) rather than assuming a fixed winding convention the way Ring's own
// `insetContour` could (that function always insets a *whole* closed contour via `Shape::offset()`,
// which resolves direction from the contour's own relationship to any sibling hole automatically -
// this function moves one point at a time, with no such built-in context, so it has to ask
// directly). Falls back to the un-inset point if neither candidate direction tests as inside (a
// genuinely degenerate local segment) rather than guess and risk moving the point the wrong way.
Point2LL polygonInwardPointAt(const PolygonGeometry& g, double pos, const Shape& containing_shape, coord_t distance)
{
    const Point2LL point_here = polygonPointAt(g, pos);
    const size_t n = g.pts.size();
    if (n < 2 || distance == 0)
    {
        return point_here;
    }
    pos = std::fmod(pos, g.total_length);
    if (pos < 0.0)
    {
        pos += g.total_length;
    }
    size_t seg_i = n - 1;
    for (size_t i = 0; i < n; ++i)
    {
        const double seg_start = g.pos[i];
        const double seg_end = (i + 1 < n) ? g.pos[i + 1] : g.total_length;
        if (pos <= seg_end || i + 1 == n)
        {
            seg_i = i;
            break;
        }
    }
    const Point2LL& p = g.pts[seg_i];
    const Point2LL& q = g.pts[(seg_i + 1) % n];
    const double dx = static_cast<double>(q.X - p.X);
    const double dy = static_cast<double>(q.Y - p.Y);
    const double len = std::hypot(dx, dy);
    if (len <= 0.0)
    {
        return point_here;
    }
    const double ux = dx / len;
    const double uy = dy / len;
    const Point2LL candidate1(
        point_here.X + static_cast<coord_t>(std::llround(-uy * static_cast<double>(distance))),
        point_here.Y + static_cast<coord_t>(std::llround(ux * static_cast<double>(distance))));
    const Point2LL candidate2(
        point_here.X + static_cast<coord_t>(std::llround(uy * static_cast<double>(distance))),
        point_here.Y + static_cast<coord_t>(std::llround(-ux * static_cast<double>(distance))));
    if (containing_shape.inside(candidate1))
    {
        return candidate1;
    }
    if (containing_shape.inside(candidate2))
    {
        return candidate2;
    }
    return point_here;
}

} // namespace

// Wall Strip (spec REV 2.6/5.7, Chain increment): the sub-range analogue of
// expandCorrugationInputForStrippedWalls above, for a Chain domain whose own wall is often only
// part of a larger shared contour rather than a whole one on its own. Finds the domain's own
// covered arc-length range on the matching contour in `corrugation_input` (== part.infill_area by
// the time this runs) the same way stripWallArcLengthRange finds it on the real wall
// (WallStrip.cpp - largest gap between the domain's own projected sample positions is the *kept*
// arc; everything else is the domain's own covered range), then moves each of *infill's own*
// vertices within that range outward to its own nearest point on `outline`, leaving the rest of
// that same contour (any un-stripped sibling domain's own segment) completely untouched.
//
// Unlike Ring's own outer/hole pair, a Chain domain's own two walls are, in the common (no-hole)
// case, just two different arcs of the *same single* contour - not genuinely separate contours -
// so both Wall A and Wall B may end up matched to and moved within that one shared contour, one
// after the other, if both are stripped.
//
// FeatherPrint Corrugated fix (Wall Strip, Chain increment): a first version of this function
// grafted in `outline`'s own corresponding sub-arc wholesale - a differently-parametrized point
// set, its own vertex density/curvature generally unrelated to infill's own - in place of the
// covered range, joined to the untouched rest of infill's own contour by two new edges at the cut
// points. Confirmed via direct user report on a real print (VBCT's own domain segmentation
// fragmenting into many short spurious domains, visible as a repeating "fan" pattern) that this
// produced geometry irregular enough (very likely self-intersecting, or locally reversing
// curvature against the rest of the contour) to corrupt VBCT's own subsequent classification of
// it - a defect a clean, sparse, synthetic-square-only test never exercised. Fixed by displacing
// infill's *own* covered-range vertices individually instead of substituting a foreign point set:
// same vertex count, order, and local connectivity as infill's own original contour throughout,
// just moved outward - far less likely to self-intersect or introduce wild curvature changes,
// since nothing about the contour's own topology changes, only individual vertex positions.
//
// Deliberately projects to outline's own true (zero-inset) surface for now, not yet the
// half-infill_line_width inset expandCorrugationInputForStrippedWalls uses for Ring - flagged as a
// known simplification to refine once real-print testing confirms whether it matters here the
// same way it did for Ring (see this file's own Wall Strip history), now that the more urgent
// segmentation-corruption defect above is fixed.
bool expandChainContourRange(Shape& corrugation_input, const Shape& outline, const std::vector<Point2LL>& domain_wall_points, coord_t half_line_width)
{
    if (domain_wall_points.size() < 2 || corrugation_input.size() == 0 || outline.size() == 0)
    {
        return false;
    }

    auto findBestMatch = [&](const Shape& shape) -> std::pair<size_t, PolygonGeometry>
    {
        size_t best_idx = 0;
        double best_avg_dist_sq = std::numeric_limits<double>::max();
        PolygonGeometry best_geom;
        for (size_t ci = 0; ci < shape.size(); ++ci)
        {
            const PolygonGeometry g = analyzePolygon(shape[ci]);
            if (g.pts.empty())
            {
                continue;
            }
            double sum_dist_sq = 0.0;
            for (const Point2LL& p : domain_wall_points)
            {
                sum_dist_sq += nearestPolygonPositionAndDistSq(g, p).second;
            }
            const double avg = sum_dist_sq / static_cast<double>(domain_wall_points.size());
            if (avg < best_avg_dist_sq)
            {
                best_avg_dist_sq = avg;
                best_idx = ci;
                best_geom = g;
            }
        }
        return { best_idx, best_geom };
    };

    const auto [infill_idx, infill_geom] = findBestMatch(corrugation_input);
    if (infill_geom.pts.empty())
    {
        return false;
    }
    const auto [outline_idx, outline_geom] = findBestMatch(outline);
    if (outline_geom.pts.empty())
    {
        return false;
    }

    std::vector<double> positions;
    positions.reserve(domain_wall_points.size());
    for (const Point2LL& p : domain_wall_points)
    {
        positions.push_back(nearestPolygonPositionAndDistSq(infill_geom, p).first);
    }
    std::sort(positions.begin(), positions.end());
    positions.erase(std::unique(positions.begin(), positions.end()), positions.end());
    if (positions.size() < 2)
    {
        return false;
    }

    const size_t n = positions.size();
    double best_gap = (positions.front() + infill_geom.total_length) - positions.back();
    size_t best_gap_start_idx = n - 1;
    for (size_t i = 0; i + 1 < n; ++i)
    {
        const double gap = positions[i + 1] - positions[i];
        if (gap > best_gap)
        {
            best_gap = gap;
            best_gap_start_idx = i;
        }
    }
    // The domain's own covered range is the complement of the largest gap (the largest gap is the
    // untouched rest of the shared contour) - same convention as WallStrip.cpp's own
    // findKeptArcFromLargestGap, inverted here since this function needs the covered range, not
    // the kept one.
    const double covered_from = positions[(best_gap_start_idx + 1) % n];
    const double covered_to = positions[best_gap_start_idx];

    // The covered range's own vertices, in infill's own original order/density - each one gets
    // moved to its own nearest point on outline below; nothing about vertex count or connectivity
    // changes, only individual positions.
    std::vector<Point2LL> covered_arc = collectPolygonArc(infill_geom, covered_from, covered_to);
    if (covered_arc.size() < 2)
    {
        return false;
    }
    for (Point2LL& p : covered_arc)
    {
        const double outline_pos = nearestPolygonPositionAndDistSq(outline_geom, p).first;
        p = polygonInwardPointAt(outline_geom, outline_pos, outline, half_line_width);
    }

    // Splice: infill's own contour, from covered_to (the range's own end) forward around to
    // covered_from (its own start) - i.e. everything *outside* the covered range, completely
    // unmodified - followed by covered_arc (now displaced outward) grafted back in where it was.
    std::vector<Point2LL> kept_infill_arc = collectPolygonArc(infill_geom, covered_to, covered_from);
    if (kept_infill_arc.size() < 2)
    {
        return false;
    }
    // Drop one end from each open arc before joining them into a single closed Polygon: kept_infill_arc's
    // own last point (interpolated at covered_from) duplicates covered_arc's own first point (the
    // same position, before displacement) - and covered_arc's own last point (interpolated at
    // covered_to) duplicates kept_infill_arc's own first point, which is where a Polygon's own
    // *implicit* closure (last vertex connects back to the first, no explicit repeat needed) already
    // reconnects to. Keeping both would leave a real, if usually harmless, zero-length duplicate
    // edge right at the closing seam - trimmed here rather than left in, since this whole function
    // exists specifically to avoid feeding VBCT anything less than clean, simple geometry (see this
    // function's own history above).
    kept_infill_arc.pop_back();
    covered_arc.pop_back();
    kept_infill_arc.insert(kept_infill_arc.end(), covered_arc.begin(), covered_arc.end());

    Polygon spliced;
    for (const Point2LL& p : kept_infill_arc)
    {
        spliced.push_back(p);
    }
    corrugation_input[infill_idx] = spliced;
    return true;
}

Shape buildCorrugationInput(const SliceLayerPart& part, const Settings& settings)
{
    const bool raw_outline_mode = settings.get<bool>("corrugated_raw_outline_mode");
    if (raw_outline_mode)
    {
        // Wall Strip (spec REV 2.6) does not yet reach the raw-outline-mode path - insetOutline
        // below computes a fixed-width inset of the raw pre-Arachne slice outline with no concept
        // of a stripped wall at all, unlike the ordinary infill_area path just below, which does
        // (expandCorrugationInputForStrippedWalls above). Confirmed directly via a real-print
        // report + diagnostic: with corrugated_raw_outline_mode on, the wall itself was still
        // stripped correctly (that step, addMeshPartToGCode, doesn't check this setting at all),
        // but the corrugation's own boundary silently never moved outboard to cover the gap, since
        // this early return skips expandCorrugationInputForStrippedWalls entirely - not a design
        // decision to combine gracefully, just an unimplemented combination. Explicitly out of
        // scope for this pass (see this project's own Wall Strip plan) - if strip settings are on
        // here too, they're silently ignored for the corrugation boundary (though the wall is
        // still physically stripped), matching this project's established "fall back to prior,
        // unmodified behavior for a combination not yet supported" convention rather than guessing
        // at an interaction rule.
        return insetOutline(part, settings);
    }

    const bool strip_wall_a = settings.get<bool>("corrugated_strip_wall_a");
    const bool strip_wall_b = settings.get<bool>("corrugated_strip_wall_b");
    if (! strip_wall_a && ! strip_wall_b)
    {
        return part.infill_area;
    }
    return expandCorrugationInputForStrippedWalls(part, settings, strip_wall_a, strip_wall_b);
}

namespace
{

// Nearest point to `target` lying anywhere along `points` (not just at a vertex), plus that
// point's own arc-length position measured from points.front() - the continuous analogue of
// PolygonUtils::findNearestVert (used by FffGcodeWriter::findSpiralizedLayerSeamVertexIndex, the
// existing precedent this cross-layer continuity pass mirrors), generalized to a continuous
// position along the wall for the same reason stage10.cpp's reference_angle_arc_length is
// continuous rather than vertex-based - see that function's own doc comment.
struct NearestPointOnWall
{
    vbct::Point2 point;
    double arc_length;
};

NearestPointOnWall nearestPointOnWall(const std::vector<vbct::Point2>& points, const vbct::Point2& target)
{
    NearestPointOnWall best{ points.empty() ? vbct::Point2{ 0.0, 0.0 } : points.front(), 0.0 };
    double best_dist_sq = std::numeric_limits<double>::max();
    double acc = 0.0;
    for (size_t i = 0; i + 1 < points.size(); ++i)
    {
        const vbct::Point2& p = points[i];
        const vbct::Point2& q = points[i + 1];
        const double dx = q.x - p.x;
        const double dy = q.y - p.y;
        const double seg_len_sq = dx * dx + dy * dy;
        const double seg_len = std::sqrt(seg_len_sq);
        double t = 0.0;
        if (seg_len_sq > 0.0)
        {
            t = ((target.x - p.x) * dx + (target.y - p.y) * dy) / seg_len_sq;
            t = std::clamp(t, 0.0, 1.0);
        }
        const vbct::Point2 candidate{ p.x + t * dx, p.y + t * dy };
        const double ddx = candidate.x - target.x;
        const double ddy = candidate.y - target.y;
        const double dist_sq = ddx * ddx + ddy * ddy;
        if (dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            best.point = candidate;
            best.arc_length = acc + t * seg_len;
        }
        acc += seg_len;
    }
    return best;
}

// Windowed variant of nearestPointOnWall (Chain domain end-position continuity): restricts the
// search to segments whose own accumulated arc-length position falls within
// [expected_arc_length - window, expected_arc_length + window], instead of searching `points`'
// entire length. Needed specifically near a Chain domain's own real wall-pair cusp (both walls
// converging toward, or meeting at, a single point - a thin airfoil's leading edge is the
// confirmed real example): a plain global nearestPointOnWall search picks whichever candidate
// segment is closest in straight-line distance, but right at a cusp the wall can pass close to
// the same XY neighborhood more than once at very different arc-length positions (the approach
// and the departure sides of a sharp local turn) - confirmed via direct real-print diagnostic on
// a Z-invariant airfoil that this let the *global* search jump between two similarly-close-by-XY
// but arc-length-distant candidates from layer to layer, each an individually valid nearest point
// but not the one continuous with the previous layer's own tracked position (visible as the
// leading-edge point moving up to ~1.8mm on ~10% of layers even with the same-wall identity
// already stable - see the caller's own comment). Constraining the search to a neighborhood
// around where the previous layer's own tracked arc-length position (rescaled to this layer's own
// wall length, since it varies slightly layer to layer) suggests the true point should be rules
// out the far, wrong-side candidate before it can ever be picked, at the cost of assuming the true
// position never drifts more than `window` in one layer - acceptable given corrugation anchors are
// already computed from geometry that changes smoothly (or not at all) layer to layer by this
// project's own scope (spec Section 4).
//
// Falls back to the unconstrained global search when the window would exclude every candidate
// (a defensive fallback, not expected to fire given `window` is chosen generously relative to
// realistic per-layer drift), and *also* when the window did contain a candidate but a
// meaningfully better one exists outside it (VBCT-Hub-Fan-Fragmentation-Brief.md's own
// follow-up): `expected_arc_length` is itself derived from the *previous* layer's own tracked
// fraction, rescaled to this layer's own wall length - if that previous fraction was captured
// under a genuinely different local domain shape (a hub that was fragmented into several small
// domains on one layer and correctly merged into one long one the next, say), the window can
// center on a location nowhere near where the true near/far end actually is, and every candidate
// inside it is a poor match by construction, not a good one the window is right to prefer. This
// generalizes the existing empty-window fallback to also catch a *non-empty but bad* window,
// without reopening the cusp confusion the window exists to prevent: at a real cusp both the
// windowed and the true continuous point sit close together in raw distance (that's the whole
// premise of a cusp - two wall passes are physically near each other), so the windowed match's own
// distance stays small there and this check does not fire; it only fires when the windowed match
// is a poor one *and* a real full-wall search finds something both meaningfully closer and clearly
// outside the window - i.e. a different, non-cusp-adjacent point the window's own prediction
// missed entirely. Never returns a worse answer than plain nearestPointOnWall would.
NearestPointOnWall nearestPointOnWallNear(const std::vector<vbct::Point2>& points, const vbct::Point2& target, const double expected_arc_length, const double window)
{
    const double lo = expected_arc_length - window;
    const double hi = expected_arc_length + window;
    NearestPointOnWall best{ points.empty() ? vbct::Point2{ 0.0, 0.0 } : points.front(), 0.0 };
    double best_dist_sq = std::numeric_limits<double>::max();
    bool found_any = false;
    double acc = 0.0;
    for (size_t i = 0; i + 1 < points.size(); ++i)
    {
        const vbct::Point2& p = points[i];
        const vbct::Point2& q = points[i + 1];
        const double dx = q.x - p.x;
        const double dy = q.y - p.y;
        const double seg_len_sq = dx * dx + dy * dy;
        const double seg_len = std::sqrt(seg_len_sq);
        const double seg_start = acc;
        const double seg_end = acc + seg_len;
        acc = seg_end;
        if (seg_end < lo || seg_start > hi)
        {
            continue; // entirely outside the window - skip without even projecting onto it
        }
        double t = 0.0;
        if (seg_len_sq > 0.0)
        {
            t = ((target.x - p.x) * dx + (target.y - p.y) * dy) / seg_len_sq;
            t = std::clamp(t, 0.0, 1.0);
        }
        const vbct::Point2 candidate{ p.x + t * dx, p.y + t * dy };
        const double ddx = candidate.x - target.x;
        const double ddy = candidate.y - target.y;
        const double dist_sq = ddx * ddx + ddy * ddy;
        if (dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            best.point = candidate;
            best.arc_length = seg_start + t * seg_len;
            found_any = true;
        }
    }
    if (!found_any)
    {
        return nearestPointOnWall(points, target);
    }
    const NearestPointOnWall global = nearestPointOnWall(points, target);
    const bool global_outside_window = global.arc_length < lo || global.arc_length > hi;
    if (global_outside_window)
    {
        const double global_dist_sq = (global.point.x - target.x) * (global.point.x - target.x) + (global.point.y - target.y) * (global.point.y - target.y);
        // "Meaningfully better," not just marginally better - only a large gap (the windowed
        // match's own squared distance at least 4x the global one, i.e. >=2x the raw distance)
        // indicates the window's own prediction actually missed the true point, rather than two
        // genuinely close-by candidates where the window is right to prefer its own pick.
        if (global_dist_sq * 4.0 < best_dist_sq)
        {
            return global;
        }
    }
    return best;
}

// The point on `points` at arc-length fraction `t` of `total_length` - the same algorithm
// stage10.cpp's own (unexported) sample_at_t uses, duplicated here rather than exported since,
// unlike wall_length/polygon_centroid/reference_angle_arc_length, it carries no investigative
// history worth sharing and is small enough that duplicating it is clearer than threading a
// fourth function through stage10.hpp's public surface for one caller.
vbct::Point2 pointAtFraction(const std::vector<vbct::Point2>& points, const double total_length, const double t)
{
    if (t <= 0.0 || points.empty())
    {
        return points.empty() ? vbct::Point2{ 0.0, 0.0 } : points.front();
    }
    if (t >= 1.0)
    {
        return points.back();
    }
    const double target = t * total_length;
    double acc = 0.0;
    for (size_t i = 0; i + 1 < points.size(); ++i)
    {
        const vbct::Point2& p = points[i];
        const vbct::Point2& q = points[i + 1];
        const double seg_len = std::hypot(q.x - p.x, q.y - p.y);
        if (acc + seg_len >= target)
        {
            const double frac = (seg_len == 0.0) ? 0.0 : (target - acc) / seg_len;
            return { p.x + frac * (q.x - p.x), p.y + frac * (q.y - p.y) };
        }
        acc += seg_len;
    }
    return points.back();
}

// Chain junction (Hub) support (spec REV 2.1, "Chain junction support"): merges pairs of Chain
// domains that share a cap point and are nearly opposite in direction there - VBCT's own Stage 8/9
// splits a single straight corridor into two domain halves wherever a third domain (a genuine
// branch) attaches to it partway along, per its own per-hub splicing logic (stage9.cpp's
// through_wall/far_cap, private to that file - this reimplements the same splice operation against
// the public Domain type, since VBCT itself only performs it when its own internal pruning happens
// to leave exactly two survivors at a hub, not generally). Confirmed via a real print's own
// topology (a T-shaped part): the crossbar is genuinely two Chain domains meeting at one shared cap
// point at ~177 degrees to each other - corrugating them independently produces two stubs meeting
// awkwardly near that point instead of one continuous run the whole crossbar's own length.
//
// Domain carries no hub identity of its own (Stage 7's real NodeId hub identity,
// Chain::start_node/end_node in stage7.hpp, is discarded once Stage 8's walk_chain_domain builds
// the final cap_start/cap_end Point2s) - detection here works the same way VBCT's own Stage 9 does
// internally: coordinate proximity, not a step down in rigor from VBCT's own approach, just
// working from the same public data VBCT itself exposes.
//
// Direction is measured whole-domain (far_cap - hub_point), not a local wall tangent -
// deliberately reusing this project's own established lesson (cross-layer continuity
// investigation): a whole-domain-integrated signal is robust where a local single-point probe is
// not (the same reasoning that made signed_area's whole-wall sign work where a local tangent
// comparison made things worse for Ring's own direction continuity).
//
// A hub with more than two domains only merges the single best-matching (closest to exactly
// opposite) pair, if any qualifies within angle_tolerance_deg; every other domain at that same hub
// - a genuine branch, like a Tee's own stem - is left unmerged, for the caller's own multi-domain
// tracking/linking to handle independently. Does not attempt to stitch a real 3+-way branch into
// one path; see this function's own header doc for that explicit scope boundary.
std::vector<vbct::Domain> mergeStraightPassThroughChains(std::vector<vbct::Domain> domains, const double angle_tolerance_deg)
{
    // Matches VBCT's own general close-point magnitude (stage9.cpp's kCloseEps), in mm - the same
    // units this function's own Point2 data is already in.
    constexpr double kHubPointCloseMm = 1e-3;
    auto close = [](const vbct::Point2& a, const vbct::Point2& b) { return std::hypot(a.x - b.x, a.y - b.y) < kHubPointCloseMm; };
    // Orient so pts.front() is whichever end is closer to target - same operation as stage10.cpp's
    // own (unexported) orient_toward, reimplemented here against public Domain data for the same
    // reason pointAtFraction above duplicates stage10.cpp's sample_at_t.
    auto orient_toward = [](std::vector<vbct::Point2> pts, const vbct::Point2& target)
    {
        const double d0 = std::hypot(pts.front().x - target.x, pts.front().y - target.y);
        const double d1 = std::hypot(pts.back().x - target.x, pts.back().y - target.y);
        if (d0 > d1)
        {
            std::reverse(pts.begin(), pts.end());
        }
        return pts;
    };
    // Appends b onto a, dropping b's own first point if it nearly duplicates a's last - matching
    // Stage 9's own trim/append_contig convention (stage9.cpp) for splicing two walls at a shared
    // point without leaving a zero-length segment at the join.
    auto concat_dedup = [&](std::vector<vbct::Point2> a, const std::vector<vbct::Point2>& b)
    {
        const size_t start = (! a.empty() && ! b.empty() && close(a.back(), b.front())) ? 1 : 0;
        a.insert(a.end(), b.begin() + static_cast<std::ptrdiff_t>(start), b.end());
        return a;
    };

    const double pi = std::acos(-1.0);
    // A pair qualifies when dot(dir_a, dir_b) <= cos_tolerance - i.e. within angle_tolerance_deg of
    // exactly opposite, where cos(180 degrees) == -1 is a perfectly straight line.
    const double cos_tolerance = std::cos((180.0 - angle_tolerance_deg) * pi / 180.0);

    std::vector<bool> consumed(domains.size(), false);
    std::vector<vbct::Domain> result;
    result.reserve(domains.size());

    for (size_t i = 0; i < domains.size(); ++i)
    {
        if (consumed[i] || domains[i].kind != "chain" || ! domains[i].cap_start || ! domains[i].cap_end || ! domains[i].left
            || ! domains[i].right)
        {
            if (! consumed[i])
            {
                result.push_back(domains[i]);
            }
            continue;
        }

        size_t best_j = domains.size();
        bool best_hub_is_start_of_i = true; // whether the shared hub is domains[i]'s cap_start (vs cap_end)
        double best_cos = 1.0; // most-negative (closest to -1, i.e. closest to exactly opposite) wins

        for (size_t j = i + 1; j < domains.size(); ++j)
        {
            if (consumed[j] || domains[j].kind != "chain" || ! domains[j].cap_start || ! domains[j].cap_end || ! domains[j].left
                || ! domains[j].right)
            {
                continue;
            }
            for (const bool i_at_start : { true, false })
            {
                const vbct::Point2& hub_i = i_at_start ? *domains[i].cap_start : *domains[i].cap_end;
                const vbct::Point2& far_i = i_at_start ? *domains[i].cap_end : *domains[i].cap_start;
                for (const bool j_at_start : { true, false })
                {
                    const vbct::Point2& hub_j = j_at_start ? *domains[j].cap_start : *domains[j].cap_end;
                    const vbct::Point2& far_j = j_at_start ? *domains[j].cap_end : *domains[j].cap_start;
                    if (! close(hub_i, hub_j))
                    {
                        continue;
                    }
                    const double dix = far_i.x - hub_i.x, diy = far_i.y - hub_i.y;
                    const double djx = far_j.x - hub_j.x, djy = far_j.y - hub_j.y;
                    const double li = std::hypot(dix, diy), lj = std::hypot(djx, djy);
                    if (li <= 0.0 || lj <= 0.0)
                    {
                        continue;
                    }
                    const double cos_angle = (dix * djx + diy * djy) / (li * lj);
                    if (cos_angle <= cos_tolerance && cos_angle < best_cos)
                    {
                        best_cos = cos_angle;
                        best_j = j;
                        best_hub_is_start_of_i = i_at_start;
                    }
                }
            }
        }

        if (best_j == domains.size())
        {
            result.push_back(domains[i]); // no qualifying straight-through partner - passes through unmerged
            continue;
        }

        consumed[i] = true;
        consumed[best_j] = true;
        const vbct::Domain& A = domains[i];
        const vbct::Domain& B = domains[best_j];
        const vbct::Point2 hub = best_hub_is_start_of_i ? *A.cap_start : *A.cap_end;
        const vbct::Point2 far_a = best_hub_is_start_of_i ? *A.cap_end : *A.cap_start;
        const bool b_hub_is_start = close(*B.cap_start, hub);
        const vbct::Point2 far_b = b_hub_is_start ? *B.cap_end : *B.cap_start;

        // Each domain contributes two wall endpoints near the hub (its own left and right walls'
        // respective near-hub ends) - match A's against B's by nearest-neighbor, not by assuming
        // "left continues into left" (Domain's own left/right orientation relative to
        // cap_start/cap_end is never guaranteed - see this function's own header doc).
        const std::vector<vbct::Point2> a_left = orient_toward(A.left->points, far_a); // far_a at front, hub at back
        const std::vector<vbct::Point2> a_right = orient_toward(A.right->points, far_a);
        const std::vector<vbct::Point2> b_left = orient_toward(B.left->points, hub); // hub at front, far_b at back
        const std::vector<vbct::Point2> b_right = orient_toward(B.right->points, hub);

        const double d_left_left = std::hypot(a_left.back().x - b_left.front().x, a_left.back().y - b_left.front().y);
        const double d_left_right = std::hypot(a_left.back().x - b_right.front().x, a_left.back().y - b_right.front().y);

        vbct::Domain merged;
        merged.kind = "chain";
        if (d_left_left <= d_left_right)
        {
            merged.left = vbct::Wall{ concat_dedup(a_left, b_left) };
            merged.right = vbct::Wall{ concat_dedup(a_right, b_right) };
        }
        else
        {
            merged.left = vbct::Wall{ concat_dedup(a_left, b_right) };
            merged.right = vbct::Wall{ concat_dedup(a_right, b_left) };
        }
        merged.cap_start = far_a;
        merged.cap_end = far_b;
        result.push_back(std::move(merged));
    }

    return result;
}

} // namespace

std::vector<ChainWallPoints> findAllChainDomainWalls(
    const SliceLayerPart& part,
    const Settings& settings,
    const std::vector<ChainDomainWallIdentity>& chain_domain_wall_identities,
    const std::vector<DeminimisHoleIdentity>& suppressed_hole_identities)
{
    std::vector<ChainWallPoints> result;
    if (chain_domain_wall_identities.empty())
    {
        return result;
    }

    // Same per-layer VBCT invocation computeAnchorsForMesh's own pre-pass already performs (see
    // that function's own Chain branch) - deliberately re-derived here rather than shared/cached,
    // since CorrugationAnchor only carries each domain's own identity *point* forward, not the
    // domain's own full wall point arrays this function actually needs. A real second VBCT
    // invocation for this one layer, gated entirely behind the two Wall Strip settings by this
    // function's own callers (FffGcodeWriter.cpp) - costs nothing when the feature isn't in use.
    const double x = settings.get<double>("corrugated_vbs_tolerance");
    const coord_t threshold = settings.get<coord_t>("corrugated_prune_threshold");
    const double threshold_mm = threshold / kMicronsPerMm;
    const double chain_junction_merge_angle_deg = settings.get<double>("corrugated_chain_junction_merge_angle");

    // De Minimis Hole Threshold (spec REV 3.0/3.5/5.9): must apply the exact same suppression this
    // layer's own pre-pass and live corrugate()/corrugateLinkedSkin() calls already applied, so
    // this function's own domain composition agrees with theirs - see filterDeminimisHoles' own
    // doc comment. This function never reaches Stage 10, so it has no need for the ignored holes'
    // own geometry (only corrugate()/corrugateLinkedSkin() thread that through for stringer
    // clipping).
    const Shape corrugation_input_raw = buildCorrugationInput(part, settings);
    std::vector<std::vector<Point2LL>> unused_ignored_hole_loops;
    const Shape corrugation_input = filterDeminimisHoles(corrugation_input_raw, suppressed_hole_identities, unused_ignored_hole_loops);
    const std::vector<vbct::Contour> contours = shapeToContours(corrugation_input);
    if (contours.empty())
    {
        return result;
    }

    vbct::Stage9Result r9;
    try
    {
        const vbct::VbctResult r14 = vbct::run_vbct(contours, x);
        const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh, threshold_mm);
        const vbct::Stage6Result r6 = vbct::run_stage6(r5);
        const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
        const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
        r9 = vbct::run_stage9(r5, r6, r7, r8, threshold_mm);
    }
    catch (const std::runtime_error&)
    {
        // VBCT rejects input it can't handle - not this function's job to log it, since
        // processCorrugatedInfill's own call for this exact layer hits the identical rejection and
        // logs it once, from one place (see computeAnchorsForMesh's own identical rationale).
        return result;
    }
    r9.domains = mergeStraightPassThroughChains(std::move(r9.domains), chain_junction_merge_angle_deg);

    auto meanPoint = [](const std::vector<vbct::Point2>& points) -> vbct::Point2
    {
        double sum_x = 0.0, sum_y = 0.0;
        for (const vbct::Point2& p : points)
        {
            sum_x += p.x;
            sum_y += p.y;
        }
        const double n = static_cast<double>(std::max<size_t>(points.size(), 1));
        return { sum_x / n, sum_y / n };
    };

    // Every Chain domain this layer has, matched back to the caller's own tracked identity list by
    // nearest identity_point - the same matching computeAnchorsForMesh's own pre-pass already
    // performed to produce that list in the first place, re-run here since this is a genuinely
    // separate VBCT invocation with its own freshly-derived domain objects.
    for (const auto& dom : r9.domains)
    {
        if (dom.kind != "chain" || ! dom.left || ! dom.right || dom.left->points.empty() || dom.right->points.empty())
        {
            continue;
        }
        std::vector<vbct::Point2> combined = dom.left->points;
        combined.insert(combined.end(), dom.right->points.begin(), dom.right->points.end());
        const vbct::Point2 identity_point = meanPoint(combined);
        const Point2LL identity_point_microns = fromVbctMm(identity_point);

        const ChainDomainWallIdentity* matched = nullptr;
        double best_dist_sq = std::numeric_limits<double>::max();
        for (const ChainDomainWallIdentity& tracked : chain_domain_wall_identities)
        {
            const double dx = static_cast<double>(identity_point_microns.X - tracked.identity_point.X);
            const double dy = static_cast<double>(identity_point_microns.Y - tracked.identity_point.Y);
            const double dist_sq = dx * dx + dy * dy;
            if (dist_sq < best_dist_sq)
            {
                best_dist_sq = dist_sq;
                matched = &tracked;
            }
        }
        if (matched == nullptr)
        {
            // No tracked identity for this domain at all (shouldn't normally happen, since this
            // function's own caller always builds chain_domain_wall_identities from the identical
            // per-layer domain composition just moments earlier in the same parallel gcode task) -
            // fail safe by skipping this domain entirely rather than guessing its own Wall A/B.
            continue;
        }

        const std::vector<vbct::Point2>& left_pts = dom.left->points;
        const std::vector<vbct::Point2>& right_pts = dom.right->points;
        const std::vector<vbct::Point2>& a_pts = matched->wall_a_is_left ? left_pts : right_pts;
        const std::vector<vbct::Point2>& b_pts = matched->wall_a_is_left ? right_pts : left_pts;

        ChainWallPoints walls;
        walls.found = true;
        walls.wall_a.reserve(a_pts.size());
        for (const vbct::Point2& p : a_pts)
        {
            walls.wall_a.push_back(fromVbctMm(p));
        }
        walls.wall_b.reserve(b_pts.size());
        for (const vbct::Point2& p : b_pts)
        {
            walls.wall_b.push_back(fromVbctMm(p));
        }
        result.push_back(std::move(walls));
    }
    return result;
}

std::vector<CorrugationAnchor> computeAnchorsForMesh(const SliceMeshStorage& mesh)
{
    const double x = mesh.settings.get<double>("corrugated_vbs_tolerance");
    const coord_t threshold = mesh.settings.get<coord_t>("corrugated_prune_threshold");
    const double threshold_mm = threshold / kMicronsPerMm;
    // Chain junction (Hub) support (spec REV 2.1) - see mergeStraightPassThroughChains' own doc
    // comment for what this controls.
    const double chain_junction_merge_angle_deg = mesh.settings.get<double>("corrugated_chain_junction_merge_angle");
    // Stringer-count stability - see CorrugationAnchor::stringer_spacing_override's own doc
    // comment for the full mechanism. Reads the same corrugated_stringer_pitch setting
    // corrugate()/corrugateLinkedSkin()'s own live per-layer call site uses directly (no
    // count-to-spacing estimation step any more), so the stabilized count this pre-pass decides
    // matches what the live per-layer call would otherwise have computed fresh.
    const coord_t vbct_stringer_pitch = mesh.settings.get<coord_t>("corrugated_stringer_pitch");

    std::vector<CorrugationAnchor> result(mesh.layers.size());
    // Both walls are tracked independently (see this function's own header doc for why tracking
    // only the canonical wall wasn't sufficient in practice) - both reset together whenever
    // tracking breaks, since they always come from the same Ring domain at the same layer.
    std::optional<vbct::Point2> prev_canonical_anchor_point;
    std::optional<vbct::Point2> prev_other_anchor_point;
    // Canonical wall's own signed area from the previous tracked layer, used only for its *sign*
    // (CW vs CCW) - see the "direction continuity" comment at this function's canonical_wall
    // reversal check for why position tracking alone isn't enough to catch a wall walked in the
    // opposite direction. A local tangent-probe version of this check was tried first and made
    // things worse (misfired on already-correct layers, per direct user testing) - signed_area
    // integrates over the whole wall, so it isn't thrown off by a single noisy local segment the
    // way a one-point tangent comparison can be.
    std::optional<double> prev_canonical_signed_area;

    // Chain domain support (spec REV 2.1, "Chain domain support"): the tracked "same real end"
    // point for a Chain's cap_start/cap_end pair - see CorrugationAnchor::chain_anchor_point's own
    // doc comment for what this is used for and why no separate direction flag is needed the way
    // reverse_canonical_wall is for Ring. Reset independently from the Ring state above whenever a
    // layer turns out not to be a (single) Chain, and vice versa - a layer's composition is either
    // Ring or Chain in the cases this function tracks continuity for, never both.
    std::optional<vbct::Point2> prev_chain_anchor_point;

    // Chain domain end-position continuity (extends chain_anchor_point's own "which end" role,
    // above, with "exactly where does each real end sit"): the tracked single Chain domain's own
    // near-end and far-end points, for each of its two walls independently - see
    // CorrugationAnchor::chain_left_near_t_frac's own doc comment for the full rationale (Ring's
    // own dual-wall anchor tracking, above, extended to Chain's two real, distinct wall ends
    // rather than Ring's one wraparound anchor). "left"/"right" here match whichever of the
    // domain's own left/right wall each point came from *at the time it was tracked* - the
    // same/swapped pairing check below (mirroring canonical_wall/other_wall's own role-swap
    // protection, further down this function) re-associates them correctly if Stage 9's own
    // left/right labeling flips between layers. Reset alongside prev_chain_anchor_point, for the
    // same reasons.
    std::optional<vbct::Point2> prev_chain_left_near;
    std::optional<vbct::Point2> prev_chain_left_far;
    std::optional<vbct::Point2> prev_chain_right_near;
    std::optional<vbct::Point2> prev_chain_right_far;

    // Each point above's own arc-length fraction along the wall it was found on, kept alongside
    // it purely so the *next* layer's own search can be windowed (nearestPointOnWallNear) around
    // where the point is expected to fall, rather than searching this layer's entire wall - see
    // that function's own doc comment for why an unconstrained global search was confirmed (real
    // print diagnostic) to occasionally jump to a similarly-close-by-XY but arc-length-distant
    // candidate near a Chain domain's own cusp. Only meaningful when the corresponding
    // prev_chain_*_near/far optional above has a value; harmless, unread, stale data otherwise.
    double prev_chain_left_near_frac = 0.0;
    double prev_chain_left_far_frac = 1.0;
    double prev_chain_right_near_frac = 0.0;
    double prev_chain_right_far_frac = 1.0;

    // Which *domain* is "the tracked Chain domain" this pre-pass follows, across layers - not
    // just that domain's own internal position/end tracking above, but its own identity among
    // however many Chain domains a layer has. Confirmed as a real, distinct root cause on a real
    // print (a thin airfoil, raw-outline-mode): once the model's own true cross-section produced
    // two genuinely separate Chain domains of very similar combined length (within ~2% of each
    // other), the "pick whichever is longest this layer" rule below re-decided completely fresh
    // every layer with no memory at all - when the two domains' own lengths crossed over each
    // other from ordinary per-layer noise, the "longest" pick silently flipped which *physical*
    // domain was being tracked, restarting every piece of this pre-pass's own continuity
    // (chain_anchor_point, the near/far end tracking above) from scratch on that one layer, with
    // no reset event ever logged - exactly the failure signature (a jump that looks like fresh,
    // uncorrelated per-layer geometry) this whole pre-pass exists to prevent, just one level up
    // from where every fix so far had been looking. Tracked the same way
    // chain_domain_wall_identities' own per-domain matching works (nearest whole-domain mean
    // point - "identity_point" - to the previous tracked layer's own), computed independently
    // here to keep this block self-contained per this function's own established convention.
    std::optional<vbct::Point2> prev_tracked_chain_identity_point;

    // The role-swap decision itself (which of this layer's own left/right walls corresponds to
    // which of the previous layer's own tracked left/right) is based on each wall's own
    // whole-wall mean point, not either wall's own near-end position - see the swap-check's own
    // comment further down this function for why a near-end-based version of this check was
    // confirmed (via direct user report on a real thin-airfoil print) to misfire on a large
    // fraction of layers, averaging away the same per-vertex cusp noise
    // chain_domain_wall_identities' own wall_a_is_left already avoids the same way. Reset
    // alongside the near/far points above.
    std::optional<vbct::Point2> prev_chain_left_mean;
    std::optional<vbct::Point2> prev_chain_right_mean;

    // Wall Strip (spec REV 2.6/5.7) previously tracked wall_a_is_left as a cross-layer nearest-
    // point identity (mirroring the pre-Section-8 chain_swap_left_right design) - superseded by a
    // direct per-layer vbct::assign_wall_ab call at the live call site below, for the same reason
    // chain_swap_left_right was: a canonical rule computable fresh from each layer's own geometry
    // has no "miss one comparison, desync forever" failure mode a cross-layer tracker does (see
    // chain_domain_stability_and_sign_oscillation lessons learned, Section 8). No per-layer state
    // is needed here any more.

    // De Minimis Hole Threshold (spec REV 3.0/3.5/5.9): every hole this mesh's own layers have
    // seen so far, with its own last-decided suppressed/not-suppressed state - see
    // CorrugationAnchor::suppressed_hole_identities' own doc comment for the full rationale.
    // Deliberately independent of every Ring/Chain-specific tracker above: a hole's own
    // suppression state has nothing to do with which domain kind the rest of the layer turns out
    // to be, so this is never reset by a Ring/Chain-specific topology change, only by the same
    // "no real geometry at all this layer" guards those trackers themselves reset on.
    struct PrevHoleTrack
    {
        Point2LL identity_point; // engine-native microns - a hole's own identity never needs VBCT's
                                  // own mm space at all, since it's decided entirely from the raw
                                  // Cura Shape, before any VBCT conversion ever happens.
        bool suppressed{ false };
    };
    std::vector<PrevHoleTrack> prev_hole_tracks;

    // Stringer-count stability - see CorrugationAnchor::stringer_spacing_override's own doc
    // comment. Deliberately independent of every Ring/Chain identity tracker above: unlike
    // position/orientation continuity, this doesn't need to know *which* domain is being tracked,
    // only "whichever domain governs this layer's own stringer count" (mirroring
    // domain_stringers' own per-layer, identity-agnostic computation) - unconditionally kept
    // across layers, resetting only when a layer has no corrugatable domain to measure at all.
    std::optional<int> prev_stringer_n; // the previously *chosen* round(governing_length/spacing)

    // Transition Layer (2026-09-15 full-layer redesign): this mesh's own layer.parts.size() at
    // every layer, recorded here (not inside the domain-tracking logic below, which only ever
    // looks at parts[0] - see that scope limitation's own comment) so the post-pass further down
    // can catch a domain physically splitting into two disconnected SliceLayerParts, not just a
    // domain splitting/appearing *within* the single part this pre-pass otherwise tracks. Confirmed
    // as a real gap on a real print (a V-shaped part whose two legs fully separate at a notch): the
    // chain_domain_wall_identities-based count below stays at 1 straight through that split, since
    // parts[1] (the newly-separated second leg) is never looked at by anything else in this
    // function - only a raw part-count change catches it.
    std::vector<size_t> part_counts(mesh.layers.size(), 0);

    for (size_t layer_nr = 0; layer_nr < mesh.layers.size(); ++layer_nr)
    {
        // Scope limitation (confirmed acceptable - see this function's own header doc): only
        // parts[0] is tracked, matching findLayerSeamsForSpiralize's own precedent. A layer with
        // no parts at all, or whose parts[0] yields nothing, resets tracking rather than reaching
        // back past the gap.
        const SliceLayer& layer = mesh.layers[layer_nr];
        part_counts[layer_nr] = layer.parts.size();
        if (layer.parts.empty())
        {
            prev_canonical_anchor_point.reset();
            prev_other_anchor_point.reset();
            prev_canonical_signed_area.reset();
            prev_chain_anchor_point.reset();
            prev_chain_left_near.reset();
            prev_chain_left_far.reset();
            prev_chain_right_near.reset();
            prev_chain_right_far.reset();
            prev_chain_left_mean.reset();
            prev_chain_right_mean.reset();
            prev_tracked_chain_identity_point.reset();
            prev_hole_tracks.clear();
            prev_stringer_n.reset();
            continue;
        }
        const SliceLayerPart& part = layer.parts[0];
        const Shape corrugation_input_raw = buildCorrugationInput(part, mesh.settings);

        // De Minimis Hole Threshold (spec REV 3.0/3.5/5.9): decide, with hysteresis, which of
        // this layer's own holes to treat as ignored for domain-classification purposes, before
        // anything else in this pre-pass (or any live per-layer call site) ever sees the geometry
        // - see CorrugationAnchor::suppressed_hole_identities' own doc comment for why this has to
        // be decided once, here, and only ever applied (never re-decided) everywhere else.
        {
            const double deminimis_area_mm2 = mesh.settings.get<double>("corrugated_deminimis_hole_area");
            std::vector<PrevHoleTrack> current_hole_tracks;
            for (const Polygon& polygon : corrugation_input_raw)
            {
                if (polygon.size() < 3 || polygon.area() >= 0.0)
                {
                    continue; // not a hole (Clipper convention: a hole's own signed area is negative)
                }
                Point2LL sum(0, 0);
                for (const Point2LL& p : polygon)
                {
                    sum += p;
                }
                const Point2LL identity_point = sum / static_cast<coord_t>(polygon.size());
                const double area_mm2 = std::abs(polygon.area()) / 1.0e6; // engine units are microns; 1mm^2 == 1e6 micron^2

                // Match cap - same rationale/fix as filterDeminimisHoles' own identical fixed cap
                // (this is the same "presence/absence against a set that may hold entries irrelevant
                // to this particular hole" problem, one layer apart in Z rather than one
                // re-derivation apart in geometry): without it, a genuinely new/unrelated hole always
                // matches *some* previous hole once prev_hole_tracks is non-empty, letting its own
                // suppressed state leak across via hysteresis rather than starting fresh from the
                // bare-threshold rule. A hole's own layer-to-layer drift is normally sub-millimeter
                // even for a tapering model; this stays well clear of that while still rejecting a
                // different, unrelated hole several millimeters away.
                constexpr double kMaxHoleIdentityDriftMicrons = 2000.0; // 2mm
                constexpr double kMaxHoleIdentityDriftMicronsSq = kMaxHoleIdentityDriftMicrons * kMaxHoleIdentityDriftMicrons;

                const PrevHoleTrack* matched = nullptr;
                double best_dist_sq = std::numeric_limits<double>::max();
                for (const PrevHoleTrack& prev : prev_hole_tracks)
                {
                    const double dx = static_cast<double>(identity_point.X - prev.identity_point.X);
                    const double dy = static_cast<double>(identity_point.Y - prev.identity_point.Y);
                    const double dist_sq = dx * dx + dy * dy;
                    if (dist_sq < best_dist_sq && dist_sq <= kMaxHoleIdentityDriftMicronsSq)
                    {
                        best_dist_sq = dist_sq;
                        matched = &prev;
                    }
                }

                // Hysteresis (spec REV 3.0's own confirmed design decision - "require a decisive
                // change, not a bare threshold crossing," the same principle this project's other
                // continuity fixes already use): a hole already suppressed stays suppressed until
                // its own area grows decisively *past* the threshold; a hole not currently
                // suppressed only becomes suppressed once its own area shrinks decisively *below*
                // it. A hole with no previous match (first tracked layer, a genuinely new hole, or
                // immediately after a reset) uses today's bare-threshold default, matching this
                // project's own established "new entry picks an arbitrary but deterministic
                // default" convention. The exact margin is not yet real-print-validated - see this
                // feature's own spec section.
                constexpr double kHysteresisMarginFraction = 0.2;
                const double margin_mm2 = deminimis_area_mm2 * kHysteresisMarginFraction;
                bool suppressed;
                if (matched != nullptr)
                {
                    suppressed = matched->suppressed ? (area_mm2 < deminimis_area_mm2 + margin_mm2) : (area_mm2 < deminimis_area_mm2 - margin_mm2);
                }
                else
                {
                    suppressed = area_mm2 < deminimis_area_mm2;
                }

                current_hole_tracks.push_back(PrevHoleTrack{ identity_point, suppressed });
                if (suppressed)
                {
                    result[layer_nr].suppressed_hole_identities.push_back(DeminimisHoleIdentity{ identity_point });
                }
            }
            prev_hole_tracks = std::move(current_hole_tracks);
        }

        std::vector<std::vector<Point2LL>> unused_ignored_hole_loops; // this pre-pass only needs the classification-time filter, not the geometry itself
        const Shape corrugation_input = filterDeminimisHoles(corrugation_input_raw, result[layer_nr].suppressed_hole_identities, unused_ignored_hole_loops);
        const std::vector<vbct::Contour> contours = shapeToContours(corrugation_input);
        if (contours.empty())
        {
            prev_canonical_anchor_point.reset();
            prev_other_anchor_point.reset();
            prev_canonical_signed_area.reset();
            prev_chain_anchor_point.reset();
            prev_chain_left_near.reset();
            prev_chain_left_far.reset();
            prev_chain_right_near.reset();
            prev_chain_right_far.reset();
            prev_chain_left_mean.reset();
            prev_chain_right_mean.reset();
            prev_tracked_chain_identity_point.reset();
            continue;
        }

        vbct::Stage9Result r9;
        try
        {
            const vbct::VbctResult r14 = vbct::run_vbct(contours, x);
            const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh, threshold_mm);
            const vbct::Stage6Result r6 = vbct::run_stage6(r5);
            const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
            const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
            r9 = vbct::run_stage9(r5, r6, r7, r8, threshold_mm);
        }
        catch (const std::runtime_error&)
        {
            // VBCT rejects input it can't handle (a self-intersecting ring after this engine's
            // own polygon simplification, etc.) - not this function's job to log it, since the
            // per-layer corrugate() call for this exact layer hits the identical rejection and
            // logs it once, from one place.
            prev_canonical_anchor_point.reset();
            prev_other_anchor_point.reset();
            prev_canonical_signed_area.reset();
            prev_chain_anchor_point.reset();
            prev_chain_left_near.reset();
            prev_chain_left_far.reset();
            prev_chain_right_near.reset();
            prev_chain_right_far.reset();
            prev_chain_left_mean.reset();
            prev_chain_right_mean.reset();
            prev_tracked_chain_identity_point.reset();
            prev_stringer_n.reset();
            continue;
        }

        // Chain junction (Hub) support (spec REV 2.1): fold any straight pass-through pair of
        // Chain domains sharing a hub point back into one continuous Chain, before any of this
        // function's own domain-composition checks below run - see mergeStraightPassThroughChains'
        // own doc comment for why this has to happen here (post-Stage-9, in the adapter layer) and
        // what it does and doesn't attempt.
        r9.domains = mergeStraightPassThroughChains(std::move(r9.domains), chain_junction_merge_angle_deg);

        // Stringer-count stability - see CorrugationAnchor::stringer_spacing_override's own doc
        // comment for the full mechanism and the real symptom this fixes. Applies to whichever
        // domain governs this layer's own stringer count - domain_stringers' own per-domain,
        // identity-agnostic scope, deliberately not tied to this pre-pass's own Ring/Chain
        // identity tracking above (that exists for position/orientation continuity, a separate
        // concern). Picks whichever domain has the longest combined wall length, matching this
        // file's own established "longest wins" convention elsewhere for picking among multiple
        // Chain domains.
        {
            const vbct::Domain* governing_domain = nullptr;
            double best_combined_len = -1.0;
            for (const auto& dom : r9.domains)
            {
                if ((dom.kind != "ring" && dom.kind != "chain") || ! dom.left || ! dom.right || dom.left->points.empty() || dom.right->points.empty())
                {
                    continue;
                }
                const double combined_len = vbct::wall_length(dom.left->points) + vbct::wall_length(dom.right->points);
                if (combined_len > best_combined_len)
                {
                    best_combined_len = combined_len;
                    governing_domain = &dom;
                }
            }
            if (governing_domain == nullptr)
            {
                prev_stringer_n.reset();
            }
            else
            {
                const double governing_length_mm
                    = std::max(vbct::wall_length(governing_domain->left->points), vbct::wall_length(governing_domain->right->points));
                const double spacing_raw_mm = static_cast<double>(vbct_stringer_pitch) / kMicronsPerMm;
                if (spacing_raw_mm > 0.0 && governing_length_mm > 0.0)
                {
                    // Pre-round quantity domain_stringers itself computes as governing_length /
                    // spacing - its own chosen n is round(m) + 1.
                    const double m = governing_length_mm / spacing_raw_mm;
                    // Hysteresis margin (same "require a decisive change, not a bare threshold
                    // crossing" principle as suppressed_hole_identities/role_swapped elsewhere in
                    // this file): once locked onto a chosen round(m), only release it once the
                    // live ratio has moved decisively past that choice's own +-0.5 rounding
                    // boundary, not on every ordinary sub-percent wobble. Not yet real-print-
                    // calibrated beyond "comfortably wider than the sub-1% noise this
                    // investigation's own real capture showed" - see this feature's own
                    // stringer_spacing_override doc comment for the measured case.
                    constexpr double kHysteresisMargin = 0.2;
                    int chosen_round_m;
                    if (prev_stringer_n.has_value() && std::abs(m - static_cast<double>(*prev_stringer_n)) <= 0.5 + kHysteresisMargin)
                    {
                        chosen_round_m = *prev_stringer_n; // within the hysteresis band - stick with the previous choice
                    }
                    else
                    {
                        // No previous choice yet, or moved decisively past it - fall back to a
                        // bare round (std::lround's half-away-from-zero tie-break, not
                        // stage10.cpp's own python_round half-to-even, differs only on an exact
                        // .5 tie on this one reset/first-layer frame - harmless, since hysteresis
                        // takes over identically from the next layer on regardless of which way
                        // this single frame's tie broke).
                        chosen_round_m = static_cast<int>(std::lround(m));
                    }
                    prev_stringer_n = chosen_round_m;
                    const int stringer_count = std::max(2, chosen_round_m + 1);
                    const double spacing_override_mm = governing_length_mm / static_cast<double>(stringer_count - 1);
                    result[layer_nr].stringer_count_tracked = true;
                    result[layer_nr].stringer_spacing_override = static_cast<coord_t>(std::llround(spacing_override_mm * kMicronsPerMm));
                    result[layer_nr].stringer_count = stringer_count;
                }
                else
                {
                    prev_stringer_n.reset();
                }
            }
        }

        // Topology-change reset point (spec REV 1.4 S:5.3.5): a real Ring must be exactly one
        // domain here, or continuity has nothing well-defined to track against.
        int n_ring = 0;
        const vbct::Domain* ring_domain = nullptr;
        for (const auto& dom : r9.domains)
        {
            if (dom.kind == "ring")
            {
                ++n_ring;
                if (ring_domain == nullptr)
                {
                    ring_domain = &dom;
                }
            }
        }
        if (n_ring != 1 || ! ring_domain->left || ! ring_domain->right || ring_domain->left->points.empty())
        {
            // Not a (single, valid) Ring this layer - Ring tracking has nothing to track against.
            prev_canonical_anchor_point.reset();
            prev_other_anchor_point.reset();
            prev_canonical_signed_area.reset();

            // Chain domain support (spec REV 2.1), extended for Chain junctions: check for at
            // least one valid Chain instead of giving up entirely. Unlike Ring, this doesn't
            // require *exactly* one - a layer with more than one Chain domain (e.g. a Tee's own
            // crossbar-plus-stem, after mergeStraightPassThroughChains has already folded any
            // straight pass-through pair into one) still gets continuity tracking, but only for
            // one domain. Every other Chain domain at this same layer still corrugates and links
            // (corrugateLinkedSkin loops over all of them independently) but without this
            // pre-pass's own cross-layer position tracking - a real, deliberately scoped residual
            // for this pass, not a silent gap: a Chain domain with no tracked anchor already has a
            // well-defined fallback (today's existing per-layer-absolute domain.cap_start
            // default), it just isn't protected from the same tessellation-noise/identity-flip
            // risk Ring's own continuity fix addressed.
            //
            // Which domain that is: tracked by *identity* (nearest whole-domain mean point to the
            // previously tracked domain's own - see prev_tracked_chain_identity_point's own doc
            // comment for why this matters, confirmed as a real, distinct bug on a real print) when
            // a previous identity exists; falls back to "greatest combined wall length" (Ring's own
            // "canonical = longer wall" precedent) only on the first tracked layer in a run, or
            // when nothing this layer looks like a reasonable match for a real topology change.
            auto meanPointOfDomain = [](const vbct::Domain& dom) -> vbct::Point2
            {
                double sum_x = 0.0, sum_y = 0.0;
                size_t n = 0;
                for (const auto* wall : { &dom.left, &dom.right })
                {
                    for (const vbct::Point2& p : (*wall)->points)
                    {
                        sum_x += p.x;
                        sum_y += p.y;
                        ++n;
                    }
                }
                return { sum_x / static_cast<double>(std::max<size_t>(n, 1)), sum_y / static_cast<double>(std::max<size_t>(n, 1)) };
            };
            const vbct::Domain* chain_domain = nullptr;
            if (prev_tracked_chain_identity_point.has_value())
            {
                double best_dist_sq = std::numeric_limits<double>::max();
                for (const auto& dom : r9.domains)
                {
                    if (dom.kind != "chain" || ! dom.left || ! dom.right || dom.left->points.empty() || dom.right->points.empty())
                    {
                        continue;
                    }
                    const vbct::Point2 identity_point = meanPointOfDomain(dom);
                    const double dx = identity_point.x - prev_tracked_chain_identity_point->x;
                    const double dy = identity_point.y - prev_tracked_chain_identity_point->y;
                    const double dist_sq = dx * dx + dy * dy;
                    if (dist_sq < best_dist_sq)
                    {
                        best_dist_sq = dist_sq;
                        chain_domain = &dom;
                    }
                }
            }
            if (chain_domain == nullptr)
            {
                // First tracked layer in this run, or no previous identity to match against -
                // today's existing absolute default.
                double best_chain_len = -1.0;
                for (const auto& dom : r9.domains)
                {
                    if (dom.kind != "chain" || ! dom.left || ! dom.right || dom.left->points.empty())
                    {
                        continue;
                    }
                    const double len = vbct::wall_length(dom.left->points) + vbct::wall_length(dom.right->points);
                    if (len > best_chain_len)
                    {
                        best_chain_len = len;
                        chain_domain = &dom;
                    }
                }
            }
            prev_tracked_chain_identity_point = (chain_domain != nullptr) ? std::make_optional(meanPointOfDomain(*chain_domain)) : std::nullopt;
            if (chain_domain != nullptr && chain_domain->cap_start.has_value() && chain_domain->cap_end.has_value())
            {
                const vbct::Point2& cap_start = *chain_domain->cap_start;
                const vbct::Point2& cap_end = *chain_domain->cap_end;
                vbct::Point2 chosen_anchor;
                if (prev_chain_anchor_point.has_value())
                {
                    // Pick whichever cap is nearer the previous tracked layer's own chosen point -
                    // see CorrugationAnchor::chain_anchor_point's own doc comment for why
                    // domain.cap_start alone (Stage 9's far_cap() assignment, stage9.cpp, which
                    // has no cross-layer memory of its own) can't be trusted to mean "the same
                    // physical end" across layers on its own.
                    const double d_start = std::hypot(cap_start.x - prev_chain_anchor_point->x, cap_start.y - prev_chain_anchor_point->y);
                    const double d_end = std::hypot(cap_end.x - prev_chain_anchor_point->x, cap_end.y - prev_chain_anchor_point->y);
                    chosen_anchor = (d_start <= d_end) ? cap_start : cap_end;
                }
                else
                {
                    // First tracked layer in this run: today's existing absolute default
                    // (domain_stringers' own Chain branch orients toward cap_start when no
                    // override is given).
                    chosen_anchor = cap_start;
                }

                result[layer_nr].has_chain = true;
                result[layer_nr].chain_anchor_point = fromVbctMm(chosen_anchor);
                prev_chain_anchor_point = chosen_anchor;

                // Chain domain end-position continuity - see CorrugationAnchor::
                // chain_left_near_t_frac's own doc comment for the full rationale. Orients this
                // domain's own left/right walls toward the same chosen_anchor target the live
                // per-layer call will use, then finds each wall's own near (t=0 side) and far
                // (t=1 side) point via the same "try same-pairing vs swapped-pairing, keep
                // whichever moves less" role-swap protection Ring's own canonical/other tracking
                // below already uses - Chain's own left/right labeling is not guaranteed stable
                // layer to layer either (confirmed directly: raw per-layer left/right point
                // counts swap which one is larger).
                //
                // The swap decision itself is deliberately based on each wall's own *whole-wall
                // mean* point, not either wall's own near-end (front()) position - reusing the
                // exact same stable-identity technique chain_domain_wall_identities' own
                // wall_a_is_left already uses for Wall Strip (see its own doc comment). An earlier
                // version of this fix used front()-based nearestPointOnWall distances instead and
                // was confirmed via direct user report on this exact real print to make things
                // *worse*, not better: on a thin airfoil, both walls converge to a narrow cusp at
                // both real ends, so each wall's own raw endpoint vertex - whichever one Stage 9
                // happened to terminate its point array on that layer, itself noisy near a cusp,
                // the same per-layer-absolute instability class chain_anchor_point's own doc
                // comment already documents - sits close enough to *both* previous tracked near
                // points that the swap decision flipped on a large fraction of layers (confirmed:
                // 108/598 layers, each a confident-looking but spurious "swap") even though the two
                // physical walls never actually swap identity. A whole-wall mean averages out that
                // per-vertex cusp noise the same way it already does for Wall Strip's own identity
                // tracking, leaving a far more stable signal to decide the swap from.
                const std::vector<vbct::Point2> oriented_left = vbct::orient_toward(chain_domain->left->points, chosen_anchor);
                const std::vector<vbct::Point2> oriented_right = vbct::orient_toward(chain_domain->right->points, chosen_anchor);
                const double oriented_left_len = vbct::wall_length(oriented_left);
                const double oriented_right_len = vbct::wall_length(oriented_right);
                if (oriented_left_len > 0.0 && oriented_right_len > 0.0)
                {
                    auto meanPointForSwapCheck = [](const std::vector<vbct::Point2>& points) -> vbct::Point2
                    {
                        double sum_x = 0.0, sum_y = 0.0;
                        for (const vbct::Point2& p : points)
                        {
                            sum_x += p.x;
                            sum_y += p.y;
                        }
                        const double n = static_cast<double>(std::max<size_t>(points.size(), 1));
                        return { sum_x / n, sum_y / n };
                    };
                    const vbct::Point2 left_mean = meanPointForSwapCheck(oriented_left);
                    const vbct::Point2 right_mean = meanPointForSwapCheck(oriented_right);

                    // Chain wall outer/inner identity continuity - see CorrugationAnchor::
                    // chain_swap_left_right's own doc comment for the full rationale. Computed
                    // unconditionally here (not gated on previous-layer tracking data existing,
                    // unlike the role_swapped block below) because vbct::assign_wall_ab needs no
                    // cross-layer history at all - it's a canonical, per-layer-computable rule
                    // (Section 3.7/5.11), so it applies correctly even on a Chain domain's very
                    // first tracked layer. Deliberately NOT derived from role_swapped below (which
                    // only protects the four near/far t-fraction pairings' own correct pairing, a
                    // separate concern) - confirmed via real capture data (chain_domain_stability_
                    // and_sign_oscillation investigation) that role_swapped's own cross-layer
                    // tracking can miss a swap-back after catching the initial swap, and its running
                    // reference then desyncs and thrashes 0/1 even while the real geometry stays
                    // completely stable. assign_wall_ab was confirmed stable instead (99.45%,
                    // 544/547 real adjacent-layer transitions) when re-evaluated fresh, independently,
                    // on every single real layer of the same capture, including every point the
                    // tracked version got confused.
                    {
                        vbct::Point2 wall_ab_centroid{ 0.0, 0.0 };
                        size_t wall_ab_total_pts = 0;
                        for (const auto& c : contours)
                        {
                            for (const auto& p : c.to_float())
                            {
                                wall_ab_centroid.x += p.x;
                                wall_ab_centroid.y += p.y;
                                ++wall_ab_total_pts;
                            }
                        }
                        if (wall_ab_total_pts > 0)
                        {
                            wall_ab_centroid.x /= static_cast<double>(wall_ab_total_pts);
                            wall_ab_centroid.y /= static_cast<double>(wall_ab_total_pts);
                        }
                        const vbct::WallAbAssignment wall_ab
                            = vbct::assign_wall_ab(chain_domain->left->points, chain_domain->right->points, contours, wall_ab_centroid);
                        result[layer_nr].chain_swap_left_right = (wall_ab.left_label != vbct::WallLabel::A);
                    }

                    if (prev_chain_left_near.has_value() && prev_chain_left_far.has_value() && prev_chain_right_near.has_value()
                        && prev_chain_right_far.has_value() && prev_chain_left_mean.has_value() && prev_chain_right_mean.has_value())
                    {
                        const double same_pairing_dist = std::hypot(left_mean.x - prev_chain_left_mean->x, left_mean.y - prev_chain_left_mean->y)
                            + std::hypot(right_mean.x - prev_chain_right_mean->x, right_mean.y - prev_chain_right_mean->y);
                        const double swapped_pairing_dist = std::hypot(left_mean.x - prev_chain_right_mean->x, left_mean.y - prev_chain_right_mean->y)
                            + std::hypot(right_mean.x - prev_chain_left_mean->x, right_mean.y - prev_chain_left_mean->y);

                        // Hysteresis margin (corrugation-wave-distortion investigation): a bare
                        // "whichever is closer" comparison here was confirmed, on a real narrow
                        // Chain domain (a small hole/cutout, ~30mm across), to flip on 4 out of 4
                        // consecutive layers even though the true physical wall identity wasn't
                        // actually changing that often - `prev_chain_left_mean`/`prev_chain_right_
                        // mean` are always just the *immediately preceding* layer's own raw labels
                        // (reassigned unconditionally below, every layer), so this decision has no
                        // memory beyond one frame and nothing stops ordinary per-layer noise from
                        // deciding it when the two candidate pairings are close. Confirmed via a
                        // direct stringer-geometry replay (not just position-tracking numbers) that
                        // this thrashing produces a real, alternating "short/long" distortion in
                        // the domain's own far-end stringer (~2.35mm vs ~2.9mm, alternating every
                        // other layer) - not merely a relabeling with no visible effect. Requiring
                        // the swapped pairing to win by a clear margin, not just numerically, is
                        // the same "require a decisive change, not a bare threshold crossing"
                        // principle this file's own suppressed_hole_identities already uses.
                        // Relative (not absolute) so it scales with this domain's own size instead
                        // of needing a per-model tuning constant - 20% chosen as comfortably above
                        // the sub-1% noise-driven gap observed in the real case that motivated this
                        // (same/swapped differing by single-digit percent), while still well below
                        // the gap a genuine identity swap produces (confirmed dramatically larger
                        // in every real case this file's own history already documents).
                        constexpr double kRoleSwapMargin = 0.8; // require swapped <= 80% of same
                        const bool role_swapped = swapped_pairing_dist < same_pairing_dist * kRoleSwapMargin;
                        // Note: chain_swap_left_right is computed separately, above, from a direct
                        // per-layer vbct::assign_wall_ab call - NOT from role_swapped here, which is
                        // used below only to pair the four near/far t-fraction values correctly.
                        prev_chain_left_mean = left_mean;
                        prev_chain_right_mean = right_mean;
                        const vbct::Point2& target_left_near = role_swapped ? *prev_chain_right_near : *prev_chain_left_near;
                        const vbct::Point2& target_left_far = role_swapped ? *prev_chain_right_far : *prev_chain_left_far;
                        const vbct::Point2& target_right_near = role_swapped ? *prev_chain_left_near : *prev_chain_right_near;
                        const vbct::Point2& target_right_far = role_swapped ? *prev_chain_left_far : *prev_chain_right_far;
                        const double expected_left_near_frac = role_swapped ? prev_chain_right_near_frac : prev_chain_left_near_frac;
                        const double expected_left_far_frac = role_swapped ? prev_chain_right_far_frac : prev_chain_left_far_frac;
                        const double expected_right_near_frac = role_swapped ? prev_chain_left_near_frac : prev_chain_right_near_frac;
                        const double expected_right_far_frac = role_swapped ? prev_chain_left_far_frac : prev_chain_right_far_frac;

                        // Windowed search (nearestPointOnWallNear's own doc comment): constrained
                        // to a neighborhood of where the previous layer's own tracked fraction,
                        // rescaled to this layer's own wall length, predicts the point should
                        // fall - rather than an unconstrained search of the whole wall, which was
                        // confirmed (real print diagnostic) to occasionally jump to a
                        // similarly-close-by-XY but arc-length-distant candidate near this
                        // domain's own cusp. 25% of this layer's own wall length is generous
                        // relative to the sub-5% steps real per-layer tracking data showed once
                        // the swap decision itself was already fixed, while still excluding a
                        // jump to the wall's own opposite end.
                        const double left_window = 0.25 * oriented_left_len;
                        const double right_window = 0.25 * oriented_right_len;
                        const NearestPointOnWall left_near = nearestPointOnWallNear(oriented_left, target_left_near, expected_left_near_frac * oriented_left_len, left_window);
                        const NearestPointOnWall left_far = nearestPointOnWallNear(oriented_left, target_left_far, expected_left_far_frac * oriented_left_len, left_window);
                        const NearestPointOnWall right_near = nearestPointOnWallNear(oriented_right, target_right_near, expected_right_near_frac * oriented_right_len, right_window);
                        const NearestPointOnWall right_far = nearestPointOnWallNear(oriented_right, target_right_far, expected_right_far_frac * oriented_right_len, right_window);

                        result[layer_nr].chain_position_tracked = true;
                        result[layer_nr].chain_left_near_t_frac = left_near.arc_length / oriented_left_len;
                        result[layer_nr].chain_left_far_t_frac = left_far.arc_length / oriented_left_len;
                        result[layer_nr].chain_right_near_t_frac = right_near.arc_length / oriented_right_len;
                        result[layer_nr].chain_right_far_t_frac = right_far.arc_length / oriented_right_len;

                        prev_chain_left_near = left_near.point;
                        prev_chain_left_far = left_far.point;
                        prev_chain_right_near = right_near.point;
                        prev_chain_right_far = right_far.point;

                        // Tried and reverted: feeding this layer's own *refined* near points back
                        // into prev_chain_anchor_point (and result[layer_nr].chain_anchor_point),
                        // replacing the raw cap_start/cap_end candidate chosen_anchor above was set
                        // to. The intent was to damp chosen_anchor's own remaining ~0.8mm jitter the
                        // same way left_near/right_near are damped - but confirmed via direct user
                        // report on a real print to make the *visible* defect worse, not better:
                        // averaging in a smoothed value gives the anchor inertia, so once it drifted
                        // to a compromise sitting between the two raw candidates that were actually
                        // being chosen between, the distance comparison started favoring the *same*
                        // candidate consistently for a long run of layers instead of self-correcting
                        // the next layer the way the unsmoothed raw-candidate comparison already did
                        // - trading brief, single-layer, self-reverting glitches for a longer,
                        // systematic run of layers landing at the same wrong spot, which reads as
                        // *more* visually disruptive even though the raw per-layer step-size metric
                        // this was tuned against improved (~3x fewer >0.5mm single-layer jumps).
                        // chosen_anchor's own raw, unsmoothed tracking (above) stays as the
                        // deliberately-chosen design - see its own doc comment.

                        prev_chain_left_near_frac = result[layer_nr].chain_left_near_t_frac;
                        prev_chain_left_far_frac = result[layer_nr].chain_left_far_t_frac;
                        prev_chain_right_near_frac = result[layer_nr].chain_right_near_t_frac;
                        prev_chain_right_far_frac = result[layer_nr].chain_right_far_t_frac;
                    }
                    else
                    {
                        // First tracked layer in this run (or immediately after a reset): seed
                        // next layer's own tracking from this layer's raw oriented endpoints -
                        // chain_position_tracked stays false, so the live per-layer call passes
                        // std::nullopt for all four fractions, reproducing today's exact
                        // pre-tracking behavior (each wall's own full raw t=0..1 span).
                        prev_chain_left_near = oriented_left.front();
                        prev_chain_left_far = oriented_left.back();
                        prev_chain_right_near = oriented_right.front();
                        prev_chain_right_far = oriented_right.back();
                        prev_chain_left_mean = left_mean;
                        prev_chain_right_mean = right_mean;
                        // Matches the points seeded just above (front is arc-length 0, back is
                        // this wall's own full length) - without this, a stale fraction left over
                        // from before a topology-change reset could wrongly constrain the very
                        // next layer's own windowed search (nearestPointOnWallNear).
                        prev_chain_left_near_frac = 0.0;
                        prev_chain_left_far_frac = 1.0;
                        prev_chain_right_near_frac = 0.0;
                        prev_chain_right_far_frac = 1.0;
                    }
                }
                else
                {
                    prev_chain_left_near.reset();
                    prev_chain_left_far.reset();
                    prev_chain_right_near.reset();
                    prev_chain_right_far.reset();
                    prev_chain_left_mean.reset();
                    prev_chain_right_mean.reset();
                }
            }
            else
            {
                prev_chain_anchor_point.reset();
                prev_chain_left_near.reset();
                prev_chain_left_far.reset();
                prev_chain_right_near.reset();
                prev_chain_right_far.reset();
                prev_chain_left_mean.reset();
                prev_chain_right_mean.reset();
            }

            // Wall Strip (spec REV 2.6/5.7): assign Wall A/B for *every* Chain domain this layer
            // has, not just the single tracked one - see CorrugationAnchor::
            // chain_domain_wall_identities' own doc comment for the full context. Deliberately runs
            // over the *whole* r9.domains list, independent of chain_domain/has_chain above (that
            // pair is scoped to a single tracked domain for position continuity; this covers every
            // domain, so more than one can be stripped simultaneously - e.g. a Tee's own crossbar
            // and stem at once).
            //
            // Computed via a direct per-layer vbct::assign_wall_ab call, the same fix for the same
            // reason as chain_swap_left_right's own call site above (see its doc comment, and the
            // chain_domain_stability_and_sign_oscillation lessons-learned Section 8): a canonical
            // rule re-evaluated fresh from each layer's own geometry has no "miss one comparison,
            // desync forever" failure mode the cross-layer nearest-point tracker this replaced had
            // structurally, and real capture data confirmed it needs no history to be stable
            // (99.45% self-consistent with zero tracking at all).
            vbct::Point2 wall_ab_centroid{ 0.0, 0.0 };
            size_t wall_ab_total_pts = 0;
            for (const auto& c : contours)
            {
                for (const auto& p : c.to_float())
                {
                    wall_ab_centroid.x += p.x;
                    wall_ab_centroid.y += p.y;
                    ++wall_ab_total_pts;
                }
            }
            if (wall_ab_total_pts > 0)
            {
                wall_ab_centroid.x /= static_cast<double>(wall_ab_total_pts);
                wall_ab_centroid.y /= static_cast<double>(wall_ab_total_pts);
            }

            auto meanPoint = [](const std::vector<vbct::Point2>& points) -> vbct::Point2
            {
                double sum_x = 0.0, sum_y = 0.0;
                for (const vbct::Point2& p : points)
                {
                    sum_x += p.x;
                    sum_y += p.y;
                }
                const double n = static_cast<double>(std::max<size_t>(points.size(), 1));
                return { sum_x / n, sum_y / n };
            };
            std::vector<ChainDomainWallIdentity> current_identities;
            for (const auto& dom : r9.domains)
            {
                if (dom.kind != "chain" || ! dom.left || ! dom.right || dom.left->points.empty() || dom.right->points.empty())
                {
                    continue;
                }
                std::vector<vbct::Point2> combined = dom.left->points;
                combined.insert(combined.end(), dom.right->points.begin(), dom.right->points.end());
                const vbct::Point2 identity_point = meanPoint(combined);

                const vbct::WallAbAssignment wall_ab
                    = vbct::assign_wall_ab(dom.left->points, dom.right->points, contours, wall_ab_centroid);

                ChainDomainWallIdentity identity;
                identity.identity_point = fromVbctMm(identity_point);
                identity.wall_a_is_left = (wall_ab.left_label == vbct::WallLabel::A);
                current_identities.push_back(identity);
            }
            result[layer_nr].chain_domain_wall_identities = std::move(current_identities);
            continue;
        }

        // This layer is a valid Ring - Chain tracking has nothing to track against.
        prev_chain_anchor_point.reset();
        prev_chain_left_near.reset();
        prev_chain_left_far.reset();
        prev_chain_right_near.reset();
        prev_chain_right_far.reset();
        prev_chain_left_mean.reset();
        prev_chain_right_mean.reset();
        prev_tracked_chain_identity_point.reset();

        // Canonicalize to the longer wall, exactly matching stage10.cpp's domain_stringers - the
        // anchors tracked here must be the same physical walls Stage 10 will actually sample
        // from. Both walls are tracked (see this function's own header doc for why), so both are
        // kept as owned vectors here rather than one being left as a reference into ring_domain.
        std::vector<vbct::Point2> canonical_wall = ring_domain->left->points;
        std::vector<vbct::Point2> raw_other_wall = ring_domain->right->points;
        if (vbct::wall_length(canonical_wall) < vbct::wall_length(raw_other_wall))
        {
            std::swap(canonical_wall, raw_other_wall);
        }

        // Direction continuity (confirmed as a real, distinct bug from both match_winding and
        // canonical/other role-swapping, above): Stage 9's own wall-stitching can emit the
        // canonical wall's points walked in *either* absolute direction from one layer to the
        // next - unlike a points[0] position shift, arc-length tracking alone can't detect this,
        // since nearestPointOnWall finds the same physical anchor point regardless of which way
        // the array is ordered. But every *other* stringer (i != anchor) is sampled at
        // i/n + t0_frac, which implicitly assumes this wall's own forward direction stays the
        // same layer to layer - if it flips, every non-anchor stringer mirrors around the anchor,
        // growing worse with distance from it. Confirmed directly on a real print: consecutive-
        // stringer steps traced a smooth loop *within* the affected layer (no malformed
        // geometry), t0_frac/wall-length/point-count were all stable across it, yet index-matched
        // stringers jumped ~95mm at that one layer and reverted the next - the exact signature of
        // a direction flip, not a position or role discontinuity.
        //
        // Detected here via signed_area's own sign (CW vs CCW), not a single local tangent probe
        // - a tangent-based version of this check was tried first and confirmed (by direct user
        // testing on a real print) to make things worse: it introduced *new* misalignment on
        // layers that were already correct, consistent with a one-point tangent comparison being
        // vulnerable to local noise (a probe landing near a segment boundary, a nearly-straight
        // stretch where direction is only weakly determined by one segment, etc.). signed_area
        // integrates the whole wall's own shoelace sum, so it's a single, stable, whole-wall
        // orientation signal - continuous with respect to small per-layer geometry changes, and
        // not sensitive to exactly where along the wall the anchor happens to sit. Corrected by
        // reversing canonical_wall *before* match_winding runs (so other_wall's own winding,
        // defined relative to canonical_wall's, follows automatically - no separate check needed
        // for it).
        bool reverse_canonical_wall = false;
        if (prev_canonical_signed_area.has_value() && (*prev_canonical_signed_area) * vbct::signed_area(canonical_wall) < 0.0)
        {
            std::reverse(canonical_wall.begin(), canonical_wall.end());
            reverse_canonical_wall = true;
        }

        // Apply the *same* winding-match decision domain_stringers itself will make
        // (`right = match_winding(left, right)`) before computing/tracking the other wall's own
        // t0_frac - otherwise that fraction gets tracked against one point order here but applied
        // against a possibly-reversed one there, which a real print showed produces large
        // (~95mm), single-layer, anchor-adjacent-but-not-anchor-itself jumps whenever
        // match_winding's own sign comparison flips on a near-tie (see match_winding's own doc
        // comment in stage10.hpp/.cpp for the full evidence). match_winding is a pure function of
        // both walls' own shapes, so calling it here with the identical two wall vectors
        // domain_stringers will use guarantees the identical decision. Uses canonical_wall
        // *after* the direction-continuity correction above, since domain_stringers' own
        // match_winding call will see whatever direction Stage 9 actually emitted this layer -
        // this function's corrected canonical_wall is a local continuity aid, not a claim about
        // what Stage 9 itself produced, but the *relative* winding match_winding computes is
        // invariant to a simultaneous reversal of its own reference input, so this is still the
        // correct pairing.
        std::vector<vbct::Point2> other_wall = vbct::match_winding(canonical_wall, raw_other_wall);
        const double canonical_len = vbct::wall_length(canonical_wall);
        const double other_len = vbct::wall_length(other_wall);
        if (canonical_len <= 0.0 || other_len <= 0.0)
        {
            prev_canonical_anchor_point.reset();
            prev_other_anchor_point.reset();
            prev_canonical_signed_area.reset();
            prev_chain_anchor_point.reset();
            prev_chain_left_near.reset();
            prev_chain_left_far.reset();
            prev_chain_right_near.reset();
            prev_chain_right_far.reset();
            prev_chain_left_mean.reset();
            prev_chain_right_mean.reset();
            prev_tracked_chain_identity_point.reset();
            continue;
        }

        // Both walls' first-layer-in-a-run picks use the *canonical* wall's own centroid as their
        // shared reference point, exactly matching domain_stringers' shared_centroid - the other
        // wall's own independent centroid is never used, for the same off-center-hole reasons
        // domain_stringers' own comment gives.
        const bool tracked = prev_canonical_anchor_point.has_value() && prev_other_anchor_point.has_value();

        vbct::Point2 canonical_anchor_point;
        double canonical_t0_frac;
        vbct::Point2 other_anchor_point;
        double other_t0_frac;
        if (tracked)
        {
            // Track by physical proximity, not by this layer's length-based canonical/other
            // *label* - confirmed as a real, separate bug from match_winding on a real print: at
            // a layer where the two walls' lengths are nearly tied, which one is "canonical"
            // (longer) can flip even though neither physical wall actually changed, and pairing
            // blindly by label (canonical_wall against prev_canonical_anchor_point, every time)
            // then compares the anchor against the *wrong* physical wall for that one layer -
            // producing a large, spurious, single-layer jump that reverts the moment the label
            // flips back (exactly the ~95mm-then-reverts pattern found in real per-layer stringer
            // dumps, with both wall point *sets* essentially unchanged - only the labeling
            // flipped). Try both pairings and keep whichever moves less in total.
            const NearestPointOnWall same_canonical = nearestPointOnWall(canonical_wall, *prev_canonical_anchor_point);
            const NearestPointOnWall same_other = nearestPointOnWall(other_wall, *prev_other_anchor_point);
            const double same_pairing_dist = std::hypot(same_canonical.point.x - prev_canonical_anchor_point->x, same_canonical.point.y - prev_canonical_anchor_point->y)
                + std::hypot(same_other.point.x - prev_other_anchor_point->x, same_other.point.y - prev_other_anchor_point->y);

            const NearestPointOnWall swapped_canonical = nearestPointOnWall(canonical_wall, *prev_other_anchor_point);
            const NearestPointOnWall swapped_other = nearestPointOnWall(other_wall, *prev_canonical_anchor_point);
            const double swapped_pairing_dist
                = std::hypot(swapped_canonical.point.x - prev_other_anchor_point->x, swapped_canonical.point.y - prev_other_anchor_point->y)
                + std::hypot(swapped_other.point.x - prev_canonical_anchor_point->x, swapped_other.point.y - prev_canonical_anchor_point->y);

            const bool role_swapped = swapped_pairing_dist < same_pairing_dist;
            const NearestPointOnWall& canonical_nearest = role_swapped ? swapped_canonical : same_canonical;
            const NearestPointOnWall& other_nearest = role_swapped ? swapped_other : same_other;

            canonical_anchor_point = canonical_nearest.point;
            canonical_t0_frac = canonical_nearest.arc_length / canonical_len;
            other_anchor_point = other_nearest.point;
            other_t0_frac = other_nearest.arc_length / other_len;
        }
        else
        {
            // First tracked layer in this run: today's existing absolute rule, reused (not
            // reimplemented) via stage10.hpp's now-exported helpers.
            const vbct::Point2 shared_centroid = vbct::polygon_centroid(canonical_wall);

            const double canonical_arc = vbct::reference_angle_arc_length(canonical_wall, shared_centroid);
            canonical_t0_frac = canonical_arc / canonical_len;
            canonical_anchor_point = pointAtFraction(canonical_wall, canonical_len, canonical_t0_frac);

            const double other_arc = vbct::reference_angle_arc_length(other_wall, shared_centroid);
            other_t0_frac = other_arc / other_len;
            other_anchor_point = pointAtFraction(other_wall, other_len, other_t0_frac);
        }

        result[layer_nr].has_ring = true;
        result[layer_nr].anchor_point = fromVbctMm(canonical_anchor_point);
        result[layer_nr].t0_frac = canonical_t0_frac;
        result[layer_nr].other_wall_anchor_point = fromVbctMm(other_anchor_point);
        result[layer_nr].other_wall_t0_frac = other_t0_frac;
        result[layer_nr].reverse_canonical_wall = reverse_canonical_wall;
        prev_canonical_anchor_point = canonical_anchor_point;
        prev_other_anchor_point = other_anchor_point;
        prev_canonical_signed_area = vbct::signed_area(canonical_wall);
    }

    // Transition Layer whole-part fallback (2026-09-15 full-layer redesign; narrowed when
    // per-domain solid-fill substitution was restored - see the per-domain post-pass below): a
    // separate post-pass over the now-fully-computed per-layer results above, rather than
    // interleaved into the per-layer loop itself - keeps this trigger fully decoupled from the
    // delicate existing Ring/Chain continuity tracking it must not disturb, and lets it simply
    // compare each layer's own already-finished CorrugationAnchor against the previous one's. Two
    // independent count-change signals, either one sufficient: the layer's own raw part_counts
    // entry - which catches a domain physically splitting into two separate SliceLayerParts
    // (confirmed necessary on a real V-shaped part whose legs fully separate at a notch:
    // chain_domain_wall_identities alone never changed, since it only ever looks at parts[0] - see
    // part_counts' own comment above; has no natural per-domain identity of its own either, since a
    // second SliceLayerPart isn't tracked by any of this pipeline's per-domain machinery at all) -
    // and the governing domain's own hysteresis-stabilized stringer_count changing
    // (CorrugationAnchor::stringer_count's own doc comment): when adjacent layers resolve to a
    // different stringer count, no stringer on this layer lines up with the corresponding stringer
    // below any more, the same kind of structural discontinuity a domain appearing/splitting is.
    // Domain-count-change is deliberately NOT one of this whole-part signal's own conditions any
    // more - see the per-domain post-pass immediately below for why.
    if (mesh.settings.get<bool>("corrugated_transition_layer_enabled"))
    {
        auto stringer_count_key = [](const CorrugationAnchor& anchor) -> std::optional<int>
        {
            return anchor.stringer_count_tracked ? std::make_optional(anchor.stringer_count) : std::nullopt;
        };

        for (size_t layer_nr = 0; layer_nr < result.size(); ++layer_nr)
        {
            const size_t prev_parts = (layer_nr > 0) ? part_counts[layer_nr - 1] : 0;
            const std::optional<int> prev_stringer_count = (layer_nr > 0) ? stringer_count_key(result[layer_nr - 1]) : std::nullopt;
            result[layer_nr].transition_layer_triggered
                = (part_counts[layer_nr] != prev_parts) || (stringer_count_key(result[layer_nr]) != prev_stringer_count);
        }

        // Transition Layer, per-domain solid-fill substitution (spec REV 3.3/3.6/5.10, Trigger 2 -
        // "a change in the number of domains present"): unlike the whole-part fallback above, a
        // domain appearing within parts[0] has a clean per-domain identity, so it gets its own
        // signal here instead - see CorrugationAnchor::ring_transition_layer_triggered/
        // chain_transition_layer_domain_identities' own doc comments for the full rationale
        // (including why this match must be capped, unlike chain_domain_wall_identities' own
        // uncapped nearest-match). A separate loop from the whole-part one above, not merged with
        // it, so the two trigger mechanisms stay independently reasoned-about.
        constexpr double kChainDomainAppearCapMicrons = 10000.0; // 10mm - materially larger than
            // De Minimis Hole Threshold's own 2mm cap (VbctAdapter.cpp's filterDeminimisHoles): a
            // whole domain's own mean point can move meaningfully more between layers than a small
            // hole's own centroid does. Placeholder value, pending real-print tuning like every
            // other first-shipped tuning constant in this project.
        constexpr double kChainDomainAppearCapMicronsSq = kChainDomainAppearCapMicrons * kChainDomainAppearCapMicrons;

        for (size_t layer_nr = 0; layer_nr < result.size(); ++layer_nr)
        {
            const bool prev_has_ring = (layer_nr > 0) && result[layer_nr - 1].has_ring;
            result[layer_nr].ring_transition_layer_triggered = result[layer_nr].has_ring && ! prev_has_ring;

            if (! result[layer_nr].chain_domain_wall_identities.empty())
            {
                const std::vector<ChainDomainWallIdentity> empty_prev_identities;
                const std::vector<ChainDomainWallIdentity>& prev_identities
                    = (layer_nr > 0) ? result[layer_nr - 1].chain_domain_wall_identities : empty_prev_identities;
                std::vector<Point2LL> appeared;
                for (const ChainDomainWallIdentity& current : result[layer_nr].chain_domain_wall_identities)
                {
                    bool matched_prev = false;
                    for (const ChainDomainWallIdentity& prev : prev_identities)
                    {
                        const double dx = static_cast<double>(current.identity_point.X - prev.identity_point.X);
                        const double dy = static_cast<double>(current.identity_point.Y - prev.identity_point.Y);
                        if (dx * dx + dy * dy <= kChainDomainAppearCapMicronsSq)
                        {
                            matched_prev = true;
                            break;
                        }
                    }
                    if (! matched_prev)
                    {
                        appeared.push_back(current.identity_point);
                    }
                }
                result[layer_nr].chain_transition_layer_domain_identities = std::move(appeared);
            }
        }
    }

    return result;
}

namespace
{

// Offsets `wall` inboard by `distance_mm` (one line width, toward the annular gap's interior) by
// moving each vertex individually along a locally-estimated inward normal - deliberately *not* a
// whole-loop Clipper offset (Shape::offset(), the same primitive insetOutline uses for a similar
// purpose). Three real, confirmed-via-direct-user-report bugs were traced to relying on a
// Clipper-produced offset polygon's own output array here in earlier versions of this function:
// Clipper doesn't promise where its output array starts (a whole-wall rotation relative to the
// input - "anchor points jumping to the other side"), doesn't preserve arc-length parametrization
// affinely under offset (uneven vertex density/miter joins - "stringers skipped near crossings"),
// and neither of two different correction attempts (reversing the array when winding-direction
// disagreed; then calibrating one single rotational alignment shift) fully closed the gap, still
// producing "tangles... at every layer and every stringer" even on Z-invariant parts - i.e. a
// purely within-one-layer geometric defect, not a cross-layer consistency one, meaning the
// remaining drift was accumulating *around the ring* between the single calibration point and
// wherever an event happened to sit, not something a single constant correction could fix.
//
// A per-vertex offset sidesteps the entire class of problem: the result has *exactly* the same
// vertex count and order as `wall` itself (see the m==wall.size()-1 loop below), so
// DomainEvent::outer_t_frac/inner_t_frac - arc-length fractions computed on the *original* wall -
// remain valid, correctly-ordered, correctly-*located* (up to each edge's own small length change
// under offset, a purely local effect that cannot accumulate around the ring the way the previous
// approaches' errors did) positions on this result too, with no separate Clipper-produced array to
// realign or reproject against at all. The trade-off, accepted deliberately given three
// Clipper-based attempts already failed in practice: no self-intersection guard at a sharp concave
// feature (Clipper's own offset algorithm exists specifically to handle that correctly, which a
// naive per-vertex push does not) - acceptable for VBS-subdivided walls, which are usually fairly
// smooth, not a general-purpose robust polygon offset.
//
// \param prefer_smaller "Inboard" points in *opposite* physical directions for the two walls of a
// Ring: the outer (canonical) wall moves inboard by shrinking (toward the ring's own center, into
// the annular gap), but the inner wall moves inboard by *growing* (away from the ring's center,
// also into the gap - the gap is on the *outside* of the inner wall's own boundary, unlike the
// outer wall's). true for the outer wall, false for the inner wall.
std::optional<std::vector<vbct::Point2>> offsetWallInboard(const std::vector<vbct::Point2>& wall, const coord_t line_width, const bool prefer_smaller)
{
    // Chain domain support (spec REV 2.1) / REV 1.7's own no-offset fix: every real call site
    // (corrugateLinkedSkin) always passes line_width=0 now - the offset itself was removed as
    // redundant (see corrugateLinkedSkin's own doc comment), and this function is kept in the call
    // chain purely for a minimal validity check. At distance 0 the result is `wall` unchanged
    // regardless of prefer_smaller, so skip straight past the signed_area/push_with_sign machinery
    // below entirely - it assumes a *closed* loop (wall.back() == wall.front(), the m==wall.size()-1
    // convention), which holds for Ring's own canonicalized walls but not for a Chain's open wall
    // pair, and signed_area's shoelace sum is not a meaningful "is this degenerate" test for an
    // open path in the first place (it can land near zero for an ordinary straight or near-straight
    // Chain wall with no actual defect, unlike a genuinely collapsed closed loop).
    if (line_width == 0)
    {
        return (wall.size() < 2) ? std::nullopt : std::make_optional(wall);
    }
    if (wall.size() < 4) // need at least 3 distinct points plus the closing duplicate
    {
        return std::nullopt;
    }
    const double distance_mm = static_cast<double>(line_width) / kMicronsPerMm;
    const double original_area = std::abs(vbct::signed_area(wall));
    if (original_area <= 0.0)
    {
        return std::nullopt;
    }

    const size_t m = wall.size() - 1; // distinct vertex count (wall.back() duplicates wall.front())

    auto push_with_sign = [&](const double push_sign)
    {
        std::vector<vbct::Point2> result(wall.size());
        for (size_t i = 0; i < m; ++i)
        {
            const vbct::Point2& prev = wall[(i + m - 1) % m];
            const vbct::Point2& curr = wall[i];
            const vbct::Point2& next = wall[(i + 1) % m];

            auto left_normal = [](const vbct::Point2& a, const vbct::Point2& b) -> vbct::Point2
            {
                const double dx = b.x - a.x;
                const double dy = b.y - a.y;
                const double len = std::hypot(dx, dy);
                return (len > 0.0) ? vbct::Point2{ -dy / len, dx / len } : vbct::Point2{ 0.0, 0.0 };
            };
            const vbct::Point2 n1 = left_normal(prev, curr);
            const vbct::Point2 n2 = left_normal(curr, next);
            vbct::Point2 normal{ n1.x + n2.x, n1.y + n2.y };
            const double normal_len = std::hypot(normal.x, normal.y);
            if (normal_len > 1e-9)
            {
                normal.x /= normal_len;
                normal.y /= normal_len;
            }
            else
            {
                normal = n1; // two adjacent edges folded back on each other - fall back to one side
            }

            result[i] = vbct::Point2{ curr.x + push_sign * distance_mm * normal.x, curr.y + push_sign * distance_mm * normal.y };
        }
        result[m] = result[0]; // restore this file's own closed-loop convention
        return result;
    };

    // Which raw sign actually shrinks vs grows the wall is decided empirically, not assumed from
    // winding direction - a first version derived it from signed_area's own sign (interior to the
    // left of a CCW loop's own direction of travel, the standard convention), which is only
    // correct if this codebase's coordinate system and VBCT's own wall-winding output agree with
    // that convention consistently - confirmed, via direct user report and a targeted diagnostic
    // (average vertex distance to the wall's own centroid, before vs. after), that it does *not*
    // always: the outer wall was measurably pushed *outward* by exactly one line width instead of
    // inward on a real part with wall_line_count=1, even though the identical logic had shrunk
    // correctly on an earlier test part - i.e. not a fixed, safely-assumable convention at all.
    // Sidesteps needing to get that convention right in the first place: build the offset with a
    // first-guess sign, measure whether the *average* vertex distance to the wall's own centroid
    // actually moved the intended direction, and rebuild with the opposite sign if not.
    const vbct::Point2 centroid = vbct::polygon_centroid(wall);
    auto avg_centroid_distance = [&](const std::vector<vbct::Point2>& points)
    {
        double sum = 0.0;
        for (size_t i = 0; i < m; ++i)
        {
            sum += std::hypot(points[i].x - centroid.x, points[i].y - centroid.y);
        }
        return sum / static_cast<double>(m);
    };
    const double before_dist = avg_centroid_distance(wall);
    std::vector<vbct::Point2> offset_wall = push_with_sign(1.0);
    const double after_dist_positive = avg_centroid_distance(offset_wall);
    const bool positive_shrank = after_dist_positive < before_dist;
    if (positive_shrank != prefer_smaller)
    {
        offset_wall = push_with_sign(-1.0);
    }

    const double result_area = std::abs(vbct::signed_area(offset_wall));
    // Generous sanity range covering both the shrink (outer wall) and grow (inner wall) cases -
    // catches a genuinely collapsed/degenerate offset (an eccentric ring, or a gap narrower than
    // roughly two line widths) without assuming which direction "collapsed" means here.
    if (result_area <= 0.0 || result_area < 0.3 * original_area || result_area > 3.0 * original_area)
    {
        return std::nullopt; // collapsed too much - see this function's own doc comment
    }
    return offset_wall;
}

// The wall vertices (of `wall`, already offset/inboard, its own closed loop of total arc length
// `wall_len`) strictly between arc-length positions `from_pos` and `to_pos`, walking forward and
// wrapping past `wall_len` back to 0 if needed, followed by `to_pt` (the exact reprojected point
// at `to_pos` - passed in rather than re-sampled, so the path's own point is bit-identical to
// whatever nearestPointOnWall found, not a resampling that could differ by floating-point noise).
// Does not include `from_pos`'s own point - the caller is expected to already have appended it as
// the previous segment's own last point.
//
// Computes every position (both `to_pos` and each vertex's own) as a *forward distance from
// from_pos*, modulo wall_len, rather than a boolean "did this wrap" test compared separately
// against two raw positions - a first version used a `to_pos <= from_pos` boolean to decide
// whether to walk "forward to wall_len, then 0 to to_pos" or just "from_pos to to_pos" directly,
// which is fragile exactly when the two positions are very close together: if `to_pos` ends up
// even slightly *below* `from_pos` from floating-point noise around what should be a near-zero-
// length arc (two crossings landing extremely close together on this wall, not full domain-wide
// coincidences - see build_domain_events' own coincidence handling for that separate, unrelated
// case), that boolean test incorrectly read as "wrap almost the entire way around" instead of
// "step forward almost not at all" - silently producing one arc that swallowed most of the wall's
// own length, absorbing every other crossing along the way as ordinary skin waypoints instead of
// their own dedicated crossings. Confirmed via direct user report on real sliced output:
// "misses" where a stringer near a crossing gets skipped over, at specific layers. The forward-
// distance formula below (`fmod(fmod(x, wall_len) + wall_len, wall_len)`, always landing in
// [0, wall_len) regardless of x's own sign) treats "from_pos and to_pos nearly coincide" as a
// forward distance near 0 either way, not near wall_len, so it can no longer flip between the two
// extremes on sub-epsilon noise. Vertices are collected with their own forward distance and
// sorted by it before being returned, since raw array index order does not correspond to forward-
// distance order once wrapping is involved (a vertex just past the wrap point has a *small*
// forward distance despite a *small* raw arc-length position, the opposite of a vertex just before
// it that hasn't wrapped yet).
std::vector<vbct::Point2> wallArcForward(
    const std::vector<vbct::Point2>& wall, const double wall_len, const double from_pos, const double to_pos, const vbct::Point2& to_pt)
{
    if (wall_len <= 0.0)
    {
        return { to_pt };
    }
    auto forward_distance = [&](const double pos) { return std::fmod(std::fmod(pos - from_pos, wall_len) + wall_len, wall_len); };
    const double to_delta = forward_distance(to_pos);

    std::vector<std::pair<double, vbct::Point2>> in_range; // (forward distance from from_pos, point)
    double acc = 0.0;
    for (size_t i = 0; i + 1 < wall.size(); ++i)
    {
        acc += std::hypot(wall[i + 1].x - wall[i].x, wall[i + 1].y - wall[i].y);
        const double vertex_delta = forward_distance(acc);
        if (vertex_delta > 0.0 && vertex_delta < to_delta)
        {
            in_range.emplace_back(vertex_delta, wall[i + 1]);
        }
    }
    std::sort(in_range.begin(), in_range.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

    std::vector<vbct::Point2> out;
    out.reserve(in_range.size() + 1);
    for (const auto& entry : in_range)
    {
        out.push_back(entry.second);
    }
    out.push_back(to_pt);
    return out;
}

// Builds one domain's own linked-skin path from its already-built DomainEvents - the per-domain
// core of corrugateLinkedSkin, factored out so it can run once per domain (spec REV 2.1, Chain
// junction support: generalized from "exactly one linkable domain" to "every domain
// independently", so a multi-domain Chain layer - e.g. a Tee's crossbar-plus-stem, after
// mergeStraightPassThroughChains has folded any straight pass-through pair together - gets one
// path per domain instead of an all-or-nothing single path). Returns std::nullopt for any reason
// this one domain isn't linkable yet - see corrugateLinkedSkin's own header doc for the full list
// (a clipped stringer, a collapsed offset).
std::optional<std::vector<Point2LL>> buildLinkedSkinPathForDomain(const vbct::DomainEvents& chosen)
{
    // v1 doesn't re-clip a shifted crossing segment against non-convex domain edges - see
    // corrugateLinkedSkin's own header doc. Any clipped event means this domain isn't linkable yet.
    // (DomainEvent::clipped itself was narrowed this session to "genuinely, entirely outside the
    // domain" - see build_domain_events' own Chain event builder comment, stage10.cpp - a mere
    // graze that still leaves a real inside portion no longer trips this at all.)
    for (const vbct::DomainEvent& event : chosen.events)
    {
        if (event.clipped)
        {
            return std::nullopt;
        }
    }

    // No additional inboard offset here (line_width=0) - see corrugateLinkedSkin's own header doc
    // for why: `chosen.outer_wall`/`inner_wall` already come from `part.infill_area`, which is
    // already the region Cura's own wall generation leaves after the wall's own footprint. Still
    // routed through offsetWallInboard (rather than using chosen.outer_wall/inner_wall directly)
    // so its own degenerate/empty-wall checks still apply - at distance 0 it returns the wall
    // unchanged, regardless of prefer_smaller.
    const std::optional<std::vector<vbct::Point2>> outer_offset = offsetWallInboard(chosen.outer_wall, /*line_width=*/0, /*prefer_smaller=*/true);
    const std::optional<std::vector<vbct::Point2>> inner_offset = offsetWallInboard(chosen.inner_wall, /*line_width=*/0, /*prefer_smaller=*/false);
    if (! outer_offset.has_value() || ! inner_offset.has_value())
    {
        return std::nullopt;
    }
    const double outer_offset_len = vbct::wall_length(*outer_offset);
    const double inner_offset_len = vbct::wall_length(*inner_offset);
    if (outer_offset_len <= 0.0 || inner_offset_len <= 0.0)
    {
        return std::nullopt;
    }

    // Reproject every event's raw attachment point onto the offset wall it belongs to, by sampling
    // that offset wall at the exact same arc-length fraction (DomainEvent::outer_t_frac/
    // inner_t_frac) - see corrugateLinkedSkin's own header doc for why this is valid directly, with
    // no reprojection search or alignment correction needed.
    struct Reprojected
    {
        vbct::Point2 outer_pt;
        double outer_pos;
        vbct::Point2 inner_pt;
        double inner_pos;
    };
    std::vector<Reprojected> reprojected;
    reprojected.reserve(chosen.events.size());
    for (const vbct::DomainEvent& event : chosen.events)
    {
        const vbct::Point2 outer_pt = pointAtFraction(*outer_offset, outer_offset_len, event.outer_t_frac);
        const vbct::Point2 inner_pt = pointAtFraction(*inner_offset, inner_offset_len, event.inner_t_frac);
        reprojected.push_back({ outer_pt, event.outer_t_frac * outer_offset_len, inner_pt, event.inner_t_frac * inner_offset_len });
    }
    const size_t event_count = reprojected.size();
    const bool is_ring = chosen.is_ring;
    if (event_count < 2 || (is_ring && event_count % 2 != 0))
    {
        // Ring: build_domain_events already guarantees an even count. Chain: no such parity
        // requirement - only the event_count < 2 defensive check applies.
        return std::nullopt;
    }

    // Walk the alternating skin+cross path - see corrugateLinkedSkin's own header doc for the full
    // construction.
    //
    // Ring closes the loop: num_segments == event_count, and next_k wraps from the last event back
    // to event 0, so the path ends back at its own starting point (reprojected[0].outer_pt).
    // Chain does not: num_segments == event_count - 1, so the walk simply stops at the last
    // event's own point on whichever wall the alternation lands on - the domain's other cap.
    //
    // FeatherPrint Corrugated note (spec REV 2.4, Chain Crosshatch + Linked Skin integration): a
    // version of this function briefly tried giving Chain's merged (base + crosshatch) events two
    // different roles in the walk - only "primary" (base) events flipping which wall governs
    // travel, "secondary" (crosshatch) events treated as an immediate there-and-back spike that
    // left the primary rhythm untouched - on the theory that the *plain* per-event alternation
    // below was what produced small extra zigzag notches near crossing points, confirmed via direct
    // user report on a real print. That was wrong, also confirmed via direct user report on a real
    // print: it replaced the correct, expected diamond/X crossing pattern across layers with a much
    // flatter chevron-column look, meaning the plain alternation below - unchanged since spec REV
    // 2.1, unmodified by this whole integration - is NOT actually the source of the notches; it's
    // load-bearing for the crossing shape itself. Reverted. The notches' own real cause is still
    // open - see this file's own history/commit log for where that investigation left off.
    const size_t num_segments = is_ring ? event_count : (event_count - 1);

    std::vector<Point2LL> path;
    path.push_back(fromVbctMm(reprojected[0].outer_pt));
    for (size_t k = 0; k < num_segments; ++k)
    {
        const size_t next_k = is_ring ? (k + 1) % event_count : (k + 1);
        const bool on_outer = (k % 2 == 0);
        const std::vector<vbct::Point2>& wall = on_outer ? *outer_offset : *inner_offset;
        const double wall_len = on_outer ? outer_offset_len : inner_offset_len;
        const double from_pos = on_outer ? reprojected[k].outer_pos : reprojected[k].inner_pos;
        const double to_pos = on_outer ? reprojected[next_k].outer_pos : reprojected[next_k].inner_pos;
        const vbct::Point2& to_pt = on_outer ? reprojected[next_k].outer_pt : reprojected[next_k].inner_pt;

        // wallArcForward's forward-distance formula is safe here for Chain too - see its own doc
        // comment and corrugateLinkedSkin's own header doc for why.
        for (const vbct::Point2& p : wallArcForward(wall, wall_len, from_pos, to_pos, to_pt))
        {
            path.push_back(fromVbctMm(p));
        }

        // Cross via next_k's own stringer to the opposite wall.
        const vbct::Point2& cross_to_pt = on_outer ? reprojected[next_k].inner_pt : reprojected[next_k].outer_pt;
        path.push_back(fromVbctMm(cross_to_pt));
    }

    // Collapse consecutive near-duplicate points (2026-09-15, Multi-Domain Two-Hole
    // crossing-glitch investigation): confirmed via real capture that whenever the base and
    // crosshatch stringer families land on an exact (or near-exact) coincidence - a genuine,
    // periodic event as the two counter-rotating families sweep past each other over Z - several
    // consecutive events near a domain's own cap can all resolve to the same physical point,
    // producing a real run of repeated points in the raw walk above (confirmed on the real part:
    // the same coordinate repeated 5 times in a row at the exact crossing layer, vs. genuine small
    // steps between distinct points on every neighboring layer). Each individual repeat is a
    // zero-length segment, technically harmless on its own, but a run of them makes the printer
    // stall at one spot instead of taking its usual small steps - the real mechanism behind the
    // reported "wall connection reverses" glitch, confirmed to occur only at these crossing
    // layers. `0.05mm` matches this project's own general "functionally coincident at print
    // resolution" epsilon (Correction 0's own sliver threshold, `contour.hpp`'s
    // `collapse_near_duplicate_points`), not an independently-tuned constant.
    constexpr coord_t kPathDedupEpsilonMicrons = 50; // 0.05mm
    std::vector<Point2LL> deduped;
    deduped.reserve(path.size());
    for (const Point2LL& p : path)
    {
        if (deduped.empty() || vSize(p - deduped.back()) > kPathDedupEpsilonMicrons)
        {
            deduped.push_back(p);
        }
    }
    path = std::move(deduped);

    if (path.size() < 2)
    {
        return std::nullopt;
    }
    return path;
}

// De Minimis Hole Threshold (spec REV 3.0/3.5/5.9): splits one finished linked-skin path into the
// pieces that stay clear of every currently-ignored hole's own real boundary, rather than gating
// the whole domain's own linkability on it the way DomainEvent::clipped does for a genuine
// domain-boundary exit (see that field's own updated doc comment for why the two cases need
// different treatment). A crossing "rung" that happens to pass through an ignored hole is expected
// - the hole is scattered somewhere in the domain's own interior independent of where stringers
// happen to land - and should not cost the domain its own link; it should just leave a small,
// physically-correct gap exactly where there's no material to print into.
//
// Implemented via Clipper boolean ops (Shape::difference/Shape::intersection) rather than a
// custom line-vs-polygon clip: build a bounding rectangle around the path (generous enough that
// its own edges never interact with the path), subtract every ignored hole's own loop from it (the
// "allowed" region), then intersect that region against the path itself - Shape::intersection's own
// restitch behavior naturally reassembles each surviving run into one continuous piece, so a path
// crossing zero holes comes back as a single, unmodified polyline (the common case), and a path
// only grazing a hole's edge without truly leaving the allowed region isn't spuriously split.
OpenLinesSet clipLinkedSkinPathAgainstHoles(const std::vector<Point2LL>& path, const std::vector<std::vector<Point2LL>>& ignored_hole_loops)
{
    OpenLinesSet result;
    if (ignored_hole_loops.empty())
    {
        result.push_back(OpenPolyline(path));
        return result;
    }

    Point2LL min_pt = path.front();
    Point2LL max_pt = path.front();
    for (const Point2LL& p : path)
    {
        min_pt.X = std::min(min_pt.X, p.X);
        min_pt.Y = std::min(min_pt.Y, p.Y);
        max_pt.X = std::max(max_pt.X, p.X);
        max_pt.Y = std::max(max_pt.Y, p.Y);
    }
    constexpr coord_t kBoundsMargin = 1000; // 1mm - only needs to clear the path's own extent.
    Polygon bounds;
    bounds.push_back(Point2LL(min_pt.X - kBoundsMargin, min_pt.Y - kBoundsMargin));
    bounds.push_back(Point2LL(max_pt.X + kBoundsMargin, min_pt.Y - kBoundsMargin));
    bounds.push_back(Point2LL(max_pt.X + kBoundsMargin, max_pt.Y + kBoundsMargin));
    bounds.push_back(Point2LL(min_pt.X - kBoundsMargin, max_pt.Y + kBoundsMargin));
    Shape bounds_shape;
    bounds_shape.push_back(bounds);

    Shape holes_shape;
    for (const std::vector<Point2LL>& loop : ignored_hole_loops)
    {
        Polygon hole_polygon;
        for (const Point2LL& p : loop)
        {
            hole_polygon.push_back(p);
        }
        holes_shape.push_back(hole_polygon);
    }

    const Shape allowed = bounds_shape.difference(holes_shape);
    OpenLinesSet original;
    original.push_back(OpenPolyline(path));
    return allowed.intersection(original);
}

} // namespace

std::optional<OpenLinesSet> corrugateLinkedSkin(
    const Shape& infill_area,
    const double x,
    const double threshold,
    const double spacing,
    const coord_t z,
    const double crossover_pitch_mm,
    const std::optional<double> anchor_t0_frac,
    const std::optional<double> other_wall_t0_frac,
    const std::optional<bool> reverse_canonical_wall,
    const std::optional<Point2LL> chain_anchor_point,
    const double chain_junction_merge_angle_deg,
    const bool chain_crosshatch_enabled,
    const std::optional<double> chain_left_near_t_frac,
    const std::optional<double> chain_left_far_t_frac,
    const std::optional<double> chain_right_near_t_frac,
    const std::optional<double> chain_right_far_t_frac,
    const std::vector<DeminimisHoleIdentity>& suppressed_hole_identities,
    const bool chain_swap_left_right,
    const TransitionLayerRequest& transition_layer_request)
{
    // TEMPORARY (layer-395 area-collapse investigation) - see corrugate()'s own identical comment.
    const double dbg_z_mm = static_cast<double>(z) / kMicronsPerMm;
    dumpAreaForInvestigation("corrugateLinkedSkin_infill_area_input", infill_area, dbg_z_mm);
    dumpShapePointsForInvestigation("corrugateLinkedSkin_infill_area_input", infill_area, dbg_z_mm);

    // De Minimis Hole Threshold (spec REV 3.0/3.5/5.9) - see corrugate()'s own identical comment.
    std::vector<std::vector<Point2LL>> ignored_hole_loops;
    const Shape filtered_infill_area = filterDeminimisHoles(infill_area, suppressed_hole_identities, ignored_hole_loops);
    dumpAreaForInvestigation("corrugateLinkedSkin_post_filterDeminimisHoles", filtered_infill_area, dbg_z_mm);
    dumpShapePointsForInvestigation("corrugateLinkedSkin_post_filterDeminimisHoles", filtered_infill_area, dbg_z_mm);
    const std::vector<vbct::Contour> contours = shapeToContours(filtered_infill_area);
    if (contours.empty())
    {
        return std::nullopt;
    }
    const double threshold_mm = threshold / kMicronsPerMm;
    const double spacing_mm = spacing / kMicronsPerMm;
    const double z_mm = static_cast<double>(z) / kMicronsPerMm;
    // Crossover Pitch / crosshatch degeneracy bias - see computeAnchorsForMesh's own identical
    // computation and doc comment for the full rationale. Must match that call site exactly - both
    // derive phase_offset from the same z_mm/crossover_pitch_mm for the same real layer, and a
    // mismatch would reintroduce a real (if tiny) difference between this function's own event
    // geometry and the pre-pass's own, defeating the whole point of a shared, deterministic
    // resolution. Also gated on crossover_pitch_mm != 0.0 - see the other call site's own doc
    // comment for why.
    constexpr double kCrosshatchDegeneracyBias = 1e-7;
    const double phase_offset = crossover_pitch_mm != 0.0 ? (z_mm / (2.0 * crossover_pitch_mm) + kCrosshatchDegeneracyBias) : 0.0;
    dumpLayerContoursForInvestigation(contours, x, threshold_mm, spacing_mm, z_mm);

    std::vector<std::vector<vbct::Point2>> extra_clip_loops_mm;
    extra_clip_loops_mm.reserve(ignored_hole_loops.size());
    for (const std::vector<Point2LL>& loop : ignored_hole_loops)
    {
        std::vector<vbct::Point2> loop_mm;
        loop_mm.reserve(loop.size());
        for (const Point2LL& p : loop)
        {
            loop_mm.push_back(toVbctMm(p));
        }
        extra_clip_loops_mm.push_back(std::move(loop_mm));
    }

    vbct::Stage9Result r9;
    try
    {
        const vbct::VbctResult r14 = vbct::run_vbct(contours, x);
        const vbct::Stage5Result r5 = vbct::run_stage5(r14.mesh, threshold_mm);
        const vbct::Stage6Result r6 = vbct::run_stage6(r5);
        const vbct::Stage7Result r7 = vbct::run_stage7(r5, r6);
        const vbct::Stage8Result r8 = vbct::run_stage8(r5, r6, r7);
        r9 = vbct::run_stage9(r5, r6, r7, r8, threshold_mm);
    }
    catch (const std::runtime_error& e)
    {
        spdlog::warn("VBCT rejected infill area for linked corrugation skin: {}", e.what());
        return std::nullopt;
    }

    // Chain junction (Hub) support (spec REV 2.1): fold any straight pass-through pair of Chain
    // domains sharing a hub point back into one continuous Chain, before build_domain_events runs -
    // see mergeStraightPassThroughChains' own doc comment for what this does and doesn't attempt.
    r9.domains = mergeStraightPassThroughChains(std::move(r9.domains), chain_junction_merge_angle_deg);

    // Transition Layer (spec REV 3.3/3.6/5.10): exclude any domain matching the caller's own
    // transition_layer_request from this function's own domain loop entirely - see this function's
    // own header doc for why (Crosshatch/Linked Corrugation Skin's own gating logic has no defined
    // meaning for a full-density solid-fill substitution). Live-re-matched the same way
    // vbct::run_stage10's own transition parameters are (a re-derivation-noise-only tolerance, not
    // a cross-layer drift cap) - see TransitionLayerRequest's own doc comment.
    if (transition_layer_request.ring || ! transition_layer_request.chain_domain_identities.empty())
    {
        constexpr double kTransitionDomainMatchCapMm = 2.0;
        constexpr double kTransitionDomainMatchCapMmSq = kTransitionDomainMatchCapMm * kTransitionDomainMatchCapMm;
        std::vector<vbct::Point2> transition_chain_domain_identities_mm;
        transition_chain_domain_identities_mm.reserve(transition_layer_request.chain_domain_identities.size());
        for (const Point2LL& p : transition_layer_request.chain_domain_identities)
        {
            transition_chain_domain_identities_mm.push_back(toVbctMm(p));
        }
        std::vector<vbct::Domain> filtered_domains;
        filtered_domains.reserve(r9.domains.size());
        for (vbct::Domain& dom : r9.domains)
        {
            if (dom.kind == "ring" && transition_layer_request.ring)
            {
                continue; // excluded - handled via corrugate()'s own transition_lines_out instead
            }
            if (dom.kind == "chain" && dom.left && dom.right && ! transition_chain_domain_identities_mm.empty())
            {
                double sum_x = 0.0, sum_y = 0.0;
                size_t n = 0;
                for (const auto* wall : { &dom.left, &dom.right })
                {
                    for (const vbct::Point2& p : (*wall)->points)
                    {
                        sum_x += p.x;
                        sum_y += p.y;
                        ++n;
                    }
                }
                if (n > 0)
                {
                    const vbct::Point2 mean{ sum_x / static_cast<double>(n), sum_y / static_cast<double>(n) };
                    bool matches_transition = false;
                    for (const vbct::Point2& target : transition_chain_domain_identities_mm)
                    {
                        const double dx = mean.x - target.x;
                        const double dy = mean.y - target.y;
                        if (dx * dx + dy * dy <= kTransitionDomainMatchCapMmSq)
                        {
                            matches_transition = true;
                            break;
                        }
                    }
                    if (matches_transition)
                    {
                        continue; // excluded
                    }
                }
            }
            filtered_domains.push_back(std::move(dom));
        }
        r9.domains = std::move(filtered_domains);
    }

    // Linked skin always needs the CW family to alternate against for a Ring domain, regardless of
    // the caller's own corrugated_crosshatch_enabled setting - see this function's own header doc
    // for why this is a deliberate exception to this project's usual "respect every setting
    // independently" pattern. Has no effect on a Chain domain's own events (see build_domain_events'
    // own "Chain domains" doc comment in stage10.hpp for why a Chain never needs a CW family).
    const std::optional<vbct::Point2> chain_anchor_point_mm
        = chain_anchor_point.has_value() ? std::make_optional(toVbctMm(*chain_anchor_point)) : std::nullopt;
    const std::vector<vbct::DomainEvents> domain_events = vbct::build_domain_events(
        r9, spacing_mm, phase_offset, anchor_t0_frac, other_wall_t0_frac, reverse_canonical_wall, /*crosshatch_enabled=*/true, chain_anchor_point_mm,
        chain_crosshatch_enabled, chain_left_near_t_frac, chain_left_far_t_frac, chain_right_near_t_frac, chain_right_far_t_frac, extra_clip_loops_mm,
        chain_swap_left_right);

    // TEMPORARY (isolated-sliver-removal investigation, layers 91/157/235/486/567/714): dump the
    // real domain composition and per-domain events.ok this call actually sees, right where the
    // real linked-skin path gets built - remove alongside every other dump this investigation added.
    if (dbg_z_mm > 9.0 && dbg_z_mm < 9.3)
    {
        FILE* f = fopen(
            "C:\\Users\\ricsz\\AppData\\Local\\Temp\\claude\\c--Users-ricsz-source-CoWork-Projects-Scratch-VBCT-Cpp\\"
            "74ac32a8-7672-4766-90f4-3fe3fe194d84\\scratchpad\\corrugateLinkedSkin_debug_layer91.txt",
            "w");
        if (f)
        {
            fprintf(f, "dbg_z_mm=%f n_domains=%zu\n", dbg_z_mm, r9.domains.size());
            for (size_t i = 0; i < r9.domains.size(); ++i)
            {
                const vbct::Domain& dom = r9.domains[i];
                fprintf(
                    f,
                    "domain[%zu] kind=%s has_left=%d has_right=%d left_pts=%zu right_pts=%zu\n",
                    i,
                    dom.kind.c_str(),
                    dom.left.has_value(),
                    dom.right.has_value(),
                    dom.left ? dom.left->points.size() : 0,
                    dom.right ? dom.right->points.size() : 0);
            }
            fprintf(f, "n_domain_events=%zu\n", domain_events.size());
            for (size_t i = 0; i < domain_events.size(); ++i)
            {
                fprintf(f, "domain_events[%zu] ok=%d is_ring=%d n_events=%zu\n", i, domain_events[i].ok, domain_events[i].is_ring, domain_events[i].events.size());
            }
            fclose(f);
        }
    }

    // Chain junction (Hub) support (spec REV 2.1): generalized from "exactly one linkable domain"
    // to "every domain independently" - a multi-domain Chain layer (e.g. a Tee's crossbar-plus-stem
    // after mergeStraightPassThroughChains) gets one path per domain. Deliberately conservative
    // all-or-nothing contract, unchanged in spirit from before this generalization: if *any* domain
    // isn't independently linkable, the *whole layer* falls back to today's unlinked corrugate()
    // output for every domain, same as a single-domain layer already did - not a new
    // per-domain-mixed-output architecture.
    bool any_ok = false;
    std::vector<std::vector<Point2LL>> paths;
    for (const vbct::DomainEvents& de : domain_events)
    {
        if (! de.ok)
        {
            continue;
        }
        any_ok = true;
        std::optional<std::vector<Point2LL>> path = buildLinkedSkinPathForDomain(de);
        if (! path.has_value())
        {
            return std::nullopt;
        }

        paths.push_back(std::move(*path));
    }
    if (! any_ok || paths.empty())
    {
        return std::nullopt;
    }

    // De Minimis Hole Threshold (spec REV 3.0/3.5/5.9): each finished path above is built without
    // any awareness of an ignored hole's own real boundary (buildLinkedSkinPathForDomain's own
    // linkability gate deliberately ignores hole crossings - see DomainEvent::clipped's own doc
    // comment) - split against every ignored hole's own geometry here, once, after the path is
    // otherwise finished, rather than disabling the whole domain's own link over it.
    OpenLinesSet lines;
    for (const std::vector<Point2LL>& path : paths)
    {
        for (OpenPolyline& piece : clipLinkedSkinPathAgainstHoles(path, ignored_hole_loops))
        {
            lines.push_back(std::move(piece));
        }
    }
    return lines;
}

} // namespace VbctAdapter
} // namespace cura
