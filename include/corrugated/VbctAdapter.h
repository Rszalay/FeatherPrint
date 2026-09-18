// Copyright (c) 2026 FeatherPrint Corrugated
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CORRUGATED_VBCT_ADAPTER_H
#define CORRUGATED_VBCT_ADAPTER_H

#include <optional>
#include <string>
#include <vector>

#include "contour.hpp" // Vendored VBCT (src/vbct/, on the include path - see its VENDORED.md).
#include "corrugated/CorrugationAnchor.h"
#include "stage10.hpp"
#include "utils/Coord_t.h"

namespace cura
{

class Shape;
class OpenLinesSet;
class SliceLayerPart;
class SliceMeshStorage;
class Settings;

namespace VbctAdapter
{

/*!
 * \brief Convert this engine's polygon-with-holes Shape (Point2LL, integer microns) into
 * VBCT's contour set (Point2i, fixed-point mm via vbct::from_float_points).
 *
 * Each ring in \p shape (outer boundary and every hole, VBCT does not care which) becomes one
 * vbct::Contour. Rings with fewer than 3 points are dropped rather than passed through, since
 * they would fail VBCT's own Stage 1 validation anyway.
 */
std::vector<vbct::Contour> shapeToContours(const Shape& shape);

/*!
 * \brief De Minimis Hole Threshold (spec REV 3.0/3.5/5.9): removes every hole in \p shape whose
 * own identity matches an entry in \p suppressed_hole_identities, returning the filtered Shape and
 * (separately) each removed hole's own point loop, in the engine's native units (microns) - so a
 * caller can still clip corrugation stringers against the hole's own real, physical boundary
 * (Section 5.9's own requirement) even though it no longer participates in VBCT's own domain
 * classification.
 *
 * \p suppressed_hole_identities is expected to already be the *decided* set for this exact layer -
 * computed once, with cross-layer hysteresis, by VbctAdapter::computeAnchorsForMesh's own pre-pass
 * (CorrugationAnchor::suppressed_hole_identities) - this function only ever *applies* that
 * decision (matching each candidate hole in \p shape to the nearest entry, by whole-polygon mean
 * vertex - the same identity-matching technique `ChainDomainWallIdentity`/`DeminimisHoleIdentity`
 * already use elsewhere), never re-decides it. Calling this with an empty
 * \p suppressed_hole_identities (no holes currently suppressed) is a cheap no-op that returns
 * \p shape unchanged.
 *
 * Every polygon in \p shape with a non-negative signed area (Clipper convention: an outer
 * boundary, never a hole) is never a candidate for suppression and is always kept as-is,
 * regardless of \p suppressed_hole_identities' own contents.
 *
 * \param shape This layer's own corrugation input, before VBCT ever sees it (i.e. the same Shape
 * `shapeToContours` would otherwise be called on directly).
 * \param suppressed_hole_identities This exact layer's own already-decided suppressed-hole set.
 * \param[out] ignored_hole_loops Each removed hole's own point loop (one entry per hole actually
 * removed), in the engine's native units (microns) - cleared at the start of this call. Empty
 * when \p suppressed_hole_identities is empty or matches nothing in \p shape.
 * \return \p shape with every matched hole removed.
 */
Shape filterDeminimisHoles(const Shape& shape, const std::vector<DeminimisHoleIdentity>& suppressed_hole_identities, std::vector<std::vector<Point2LL>>& ignored_hole_loops);

/*!
 * \brief Transition Layer (spec REV 3.3/3.6/5.10): identifies which domain(s), if any, this exact
 * layer's corrugate() call should substitute with full-density solid infill instead of ordinary
 * corrugation, and at what spacing. Built by FffGcodeWriter::processCorrugatedInfill from this
 * layer's own CorrugationAnchor (CorrugationAnchor::ring_transition_layer_triggered/
 * chain_transition_layer_domain_identities, decided once with cross-layer tracking by
 * computeAnchorsForMesh's own pre-pass) plus infill_line_width - never decided by corrugate()
 * itself, which only ever *applies* this request via a fresh, live re-match against its own
 * current Stage 9 domain composition (vbct::run_stage10's own transition_* parameters).
 */
struct TransitionLayerRequest
{
    //! True when this layer's own Ring domain should get a Transition Layer. Meaningless (and has
    //! no effect) when the layer has no Ring domain at all.
    bool ring{ false };

    //! Identity points (matching CorrugationAnchor::chain_transition_layer_domain_identities,
    //! engine-native microns) of every Chain domain that should get a Transition Layer this layer.
    //! Independent domains may each transition at once - unlike the cross-layer anchor overrides
    //! elsewhere in this file, which apply only to a single tracked domain.
    std::vector<Point2LL> chain_domain_identities;

    //! Target spacing for a Transition Layer's own full-density stringers, in the engine's native
    //! units (microns) - normally infill_line_width, so successive members sit edge to edge with
    //! (approximately) no gap. Ignored when neither \c ring nor \c chain_domain_identities selects
    //! anything for this call.
    coord_t solid_fill_spacing{ 0 };
};

/*!
 * \brief Flatten a VBCT Stage10Result's stringers into this engine's native line-segment
 * container, converting back from VBCT's Point2 (double, mm) to Point2LL (integer microns).
 *
 * One OpenPolyline per StringerEdge piece (a stringer clipped by a non-convex domain can have
 * more than one piece; each becomes its own line here, not one continuous polyline).
 *
 * \param[out] transition_lines_out Transition Layer (spec REV 3.3/3.6/5.10): when non-null, every
 * domain \p result marks as vbct::DomainStringers::is_transition_layer is routed here instead of
 * the returned OpenLinesSet - so a caller (corrugate()) can apply Cura's own native bridging print
 * settings to just those lines. Cleared at the start of this call when non-null. Left untouched
 * (and every domain's own lines returned in the one OpenLinesSet, today's exact pre-feature
 * behavior) when null, the default.
 */
OpenLinesSet stage10ToLines(const vbct::Stage10Result& result, OpenLinesSet* transition_lines_out = nullptr);

/*!
 * \brief Run VBCT's full pipeline (Stages 1-10) on \p infill_area and return the resulting
 * corrugation stringers as printable line segments, or std::nullopt if VBCT rejected the input
 * (e.g. a degenerate or self-intersecting ring after this engine's own polygon simplification)
 * or produced nothing.
 *
 * \param infill_area The area to corrugate - this project's spec places VBCT downstream of
 * CuraEngine's own wall generation, corrugating only the region left after walls, not replacing
 * wall generation itself.
 * \param x VBS edge-length tolerance fraction (spec REV 2.1 S:2.1).
 * \param threshold Skeleton chain pruning length threshold, in the engine's native units
 * (microns).
 * \param spacing Target stringer spacing, in the engine's native units (microns).
 * \param z This layer's real print height, in the engine's native units (microns) - used with
 * \p crossover_pitch_mm to compute the Z-driven crossover phase described below. Ignored when
 * \p crossover_pitch_mm is 0.
 * \param crossover_pitch_mm Crossover Pitch (2026-09-15 redesign, replacing the old
 * phase_rate/"wraps per mm" setting - see this project's own crossover-synchronization design
 * notes): the Z distance, in mm, between successive base/crosshatch family coincidences. 0 (the
 * default) reproduces VBCT's own fixed t=0 anchor on every layer, same as the old phase_rate=0.
 * A positive value derives \c vbct::stage10's own \c phase_offset parameter as
 * \c z_mm / (2 * crossover_pitch_mm) - a raw, undivided argument each domain's own formula
 * divides by its own stringer count before use, which is what makes every domain (regardless of
 * its own stringer count/density) reach its family coincidence at the *same* Z, every
 * crossover_pitch_mm of height, rather than each domain's own coincidence period depending on
 * its own density the way a flat "wraps per mm" rate did (confirmed as a real defect on a real
 * multi-Chain-domain part: two of three domains crossed over together while the third, sparser
 * domain didn't reach its own first coincidence until a much later layer). This also drives Ring's
 * own single-family stringer-start continuity sweep (spec Section 3.1) - see stage10.hpp's
 * phase_offset note for the exact formula. Has no effect on a Chain domain's own non-crosshatch
 * base family - see stage10.hpp's phase_offset note.
 * \param anchor_t0_frac Cross-layer continuity override (spec REV 1.4 S:5.3), forwarded straight
 * to vbct::run_stage10's own parameter of the same name - see its doc comment. Normally supplied
 * from FffGcodeWriter::computeCorrugationAnchors' precomputed CorrugationAnchor for this exact
 * layer (\c std::nullopt when that layer had no tracked Ring, e.g. a topology change or no
 * corrugation_anchors entry at all - both mean "fall back to today's per-layer absolute rule").
 * \param other_wall_t0_frac Same continuity override, for the Ring's other (shorter) wall -
 * forwarded straight to vbct::run_stage10's parameter of the same name. See its own doc comment
 * for why tracking only the canonical wall wasn't sufficient in practice.
 * \param reverse_canonical_wall Cross-layer *direction* continuity override, forwarded straight
 * to vbct::run_stage10's parameter of the same name - see its own doc comment for why position
 * tracking alone (the two parameters above) wasn't sufficient either.
 * \param crosshatch_enabled Second ("CW") stringer family, forwarded straight to
 * vbct::run_stage10's parameter of the same name - see its own doc comment for the exact
 * opposite-Z-phase construction (confirmed with the user; two earlier, same-layer-crossing designs
 * were tried and ruled out first). Completes spec Section 3.1's "two counter-rotating helix
 * families (CCW/CW)" design; today's single-family output (the default, false) is only the CCW
 * half of that design. Has no visible effect when \p crossover_pitch_mm is 0 (or at a Z where the
 * two families' phase happens to coincide) - crosshatch is fundamentally a property of how the
 * pattern changes over print height, not of any single layer's own 2D geometry.
 * \param chain_anchor_point Cross-layer continuity override for Chain domains (spec REV 2.1
 * "Chain domain support"), in the engine's native units (microns) - converted internally and
 * forwarded straight to vbct::run_stage10's own parameter of the same name; see its doc comment.
 * Normally supplied from FffGcodeWriter::computeCorrugationAnchors' precomputed CorrugationAnchor
 * for this exact layer (\c std::nullopt when that layer had no tracked Chain, e.g. a topology
 * change or no corrugation_anchors entry at all - both mean "fall back to today's per-layer
 * absolute rule", i.e. orient toward domain.cap_start fresh). Has no effect on Ring domains.
 * \param chain_left_near_t_frac, \param chain_left_far_t_frac, \param chain_right_near_t_frac,
 * \param chain_right_far_t_frac Chain domain *end-position* continuity (as opposed to
 * chain_anchor_point's own "which end" role): each wall's own base-family sampling interpolates
 * between its own tracked near and far arc-length fraction instead of running its full raw t=0..1
 * span - forwarded straight to \c vbct::run_full_pipeline/run_stage10's own parameters of the same
 * names, see their doc comment for the full account. Sourced from \c
 * CorrugationAnchor::chain_left_near_t_frac etc. when \c chain_position_tracked is true,
 * \c std::nullopt otherwise (today's exact pre-tracking behavior). Has no effect on Ring domains.
 * \param suppressed_hole_identities De Minimis Hole Threshold (spec REV 3.0/3.5/5.9): this exact
 * layer's own already-decided set of holes to ignore for domain classification (\c
 * CorrugationAnchor::suppressed_hole_identities, computed with cross-layer hysteresis by \c
 * computeAnchorsForMesh's own pre-pass - see that field's own doc comment). Applied via \c
 * filterDeminimisHoles before VBCT ever sees \p infill_area; each ignored hole's own real boundary
 * is still threaded through as an extra stringer-clip edge loop, so nothing is ever printed through
 * the void even though it no longer participates in classification. Empty (the default) is a
 * cheap no-op reproducing today's exact pre-feature behavior.
 * \param transition_layer_request Transition Layer (spec REV 3.3/3.6/5.10): this exact layer's own
 * already-decided set of domains, if any, to substitute with full-density solid infill instead of
 * ordinary corrugation - see TransitionLayerRequest's own doc comment. Forwarded to
 * vbct::run_full_pipeline's own transition_* parameters. Default-constructed (the default) is a
 * cheap no-op reproducing today's exact pre-feature behavior for every domain.
 * \param[out] transition_lines_out Transition Layer: when non-null, every transitioned domain's
 * own solid-fill lines are routed here instead of the returned OpenLinesSet - see
 * stage10ToLines's own doc comment. Cleared at the start of this call when non-null; left
 * untouched when null, the default.
 */
std::optional<OpenLinesSet> corrugate(
    const Shape& infill_area,
    double x,
    double threshold,
    double spacing,
    coord_t z = 0,
    double crossover_pitch_mm = 0.0,
    std::optional<double> anchor_t0_frac = std::nullopt,
    std::optional<double> other_wall_t0_frac = std::nullopt,
    std::optional<bool> reverse_canonical_wall = std::nullopt,
    bool crosshatch_enabled = false,
    std::optional<Point2LL> chain_anchor_point = std::nullopt,
    std::optional<double> chain_left_near_t_frac = std::nullopt,
    std::optional<double> chain_left_far_t_frac = std::nullopt,
    std::optional<double> chain_right_near_t_frac = std::nullopt,
    std::optional<double> chain_right_far_t_frac = std::nullopt,
    const std::vector<DeminimisHoleIdentity>& suppressed_hole_identities = {},
    const TransitionLayerRequest& transition_layer_request = {},
    OpenLinesSet* transition_lines_out = nullptr);

/*!
 * \brief Experimental (see the "Corrugated Raw Outline Mode" setting): a simple, fixed-width
 * inset of \p part's raw slice outline (its cross-section *before* this engine's own Arachne
 * wall generation ever runs), as a Z-invariant alternative to corrugating the wall-generated
 * Infill Area.
 *
 * Deliberately does not reproduce Arachne's variable-width bead placement -- that variability
 * (a genuinely different local wall topology at different Z heights for a part whose
 * cross-section doesn't actually change with height) is exactly what this mode trades away, in
 * exchange for input VBCT's own domain classification can rely on being the same shape at every
 * layer. Mirrors WallsComputation::generateSpiralInsets' pattern (a single Shape::offset() call)
 * rather than invoking WallToolPaths/Arachne. Does not exclude top/bottom skin area.
 *
 * \param part The layer part whose raw \c outline to inset. If \c wall_line_count is 0, returns
 * \c part.outline unmodified (no walls to inset by).
 * \param settings Read for \c wall_line_count, \c wall_line_width_0, \c wall_line_width_x, and
 * \c wall_0_inset -- the same settings WallsComputation::generateWalls itself reads for the
 * first/subsequent wall widths.
 */
Shape insetOutline(const SliceLayerPart& part, const Settings& settings);

/*!
 * \brief Build the Shape corrugate() should actually corrugate for one layer part: either
 * insetOutline's raw-outline inset, or \p part's own wall-generated \c infill_area unchanged,
 * chosen by the \c corrugated_raw_outline_mode setting.
 *
 * Factored out so FffGcodeWriter::processCorrugatedInfill's per-layer call and
 * computeAnchorsForMesh's cross-layer pre-pass (spec REV 1.4 S:5.3) build the exact same input
 * for the exact same layer, rather than risk the two call sites drifting apart.
 *
 * \param part The layer part to corrugate.
 * \param settings Read for \c corrugated_raw_outline_mode, and (when that's set)
 * insetOutline's own settings.
 */
Shape buildCorrugationInput(const SliceLayerPart& part, const Settings& settings);

/*!
 * \brief TEMPORARY debug instrumentation for the layer-395 area-collapse investigation
 * (2026-09-13): appends "<stage>,<z_mm>,<polygon_count>,<area_mm2>" to a fixed scratchpad CSV,
 * gated to a single hardcoded target Z so it doesn't fire on every layer of every print. Pure
 * geometry only (Shape::area()) - never throws, safe to call from any pipeline stage. Remove
 * this declaration and its definition/call sites once the investigation concludes.
 */
void dumpAreaForInvestigation(const std::string& stage, const Shape& shape, double z_mm);

/*!
 * \brief TEMPORARY debug instrumentation, same investigation/gating/safety as
 * dumpAreaForInvestigation above - dumps a Shape's own raw point coordinates (one JSON file per
 * call site) so the actual polygon shape can be inspected/plotted. Remove alongside
 * dumpAreaForInvestigation once the investigation concludes.
 */
void dumpShapePointsForInvestigation(const std::string& stage, const Shape& shape, double z_mm);

/*!
 * \brief Cross-layer anchor continuity pre-pass (spec REV 1.4 S:5.3): walks \p mesh's layers in Z
 * order, single-threaded, and for each one that has a corrugatable Ring domain (via
 * buildCorrugationInput on that layer's \c parts[0] and \p mesh's own \c corrugated_vbs_tolerance
 * / \c corrugated_prune_threshold settings), picks a t=0 anchor on *both* of the Ring's walls
 * (tracked independently - see CorrugationAnchor's own doc comment for why a first version that
 * tracked only the canonical wall wasn't sufficient): the *first* such layer in the mesh, or the
 * first one after a gap, via today's existing absolute rule (vbct::reference_angle_arc_length
 * from the canonical wall's own shared centroid - the same rule domain_stringers itself falls
 * back to when no override is given) for each wall; every subsequent one via the point on that
 * layer's corresponding wall nearest to the *immediately preceding* tracked layer's own chosen
 * anchor point for that same wall, converted to an arc-length fraction.
 *
 * This is the only place in this codebase that computes an anchor by referencing another layer's
 * already-computed result - safe here specifically because this function runs before CuraEngine's
 * parallel per-layer producer/consumer loop starts (see FffGcodeWriter::computeCorrugationAnchors'
 * own doc comment, and CuraEngine's own findLayerSeamsForSpiralize for the existing precedent this
 * mirrors), not from inside it.
 *
 * A layer resets Ring tracking (returns \c has_ring=false, and the next Ring-tracked layer starts
 * fresh via the absolute rule again) whenever it has no parts, buildCorrugationInput's result is
 * empty, VBCT's Stages 1-9 reject the input, or the resulting domain composition isn't exactly one
 * Ring (a Chain/Glob/multi-Ring layer) - a real topology change is a legitimate reset point, not
 * something this pass tries to paper over (spec REV 1.4 S:5.3.5).
 *
 * Chain domain support (spec REV 2.1, "Chain domain support"): a layer whose composition includes
 * at least one Chain domain (instead of exactly one Ring) is tracked independently, via
 * \c CorrugationAnchor::has_chain/chain_anchor_point - see that field's own doc comment for what
 * gets tracked (a single "same real end" point, no separate direction override needed the way
 * Ring's reverse_canonical_wall is, since \c orient_toward's own nearest-endpoint search already
 * is the direction fix once given a stable target). Ring and Chain tracking reset each other:
 * whichever kind a layer *isn't* gets treated as a topology-change reset for that kind's own
 * tracking, even on a layer where the other kind is successfully tracked.
 *
 * Chain junction (Hub) support (spec REV 2.1): before any of the above runs, any straight
 * pass-through pair of Chain domains sharing a hub point is folded into one continuous Chain (see
 * \c mergeStraightPassThroughChains' own doc comment in the .cpp) - so a Tee-shaped part's own
 * crossbar tracks as a single Chain, not two independent halves. Unlike Ring, this function does
 * *not* require exactly one Chain domain after that merge: if more than one remains (e.g. a Tee's
 * crossbar and its stem), the one with the greatest combined wall length is tracked - a
 * deliberately scoped residual, not full N-domain tracking; every other Chain domain at that same
 * layer still corrugates and links independently (VbctAdapter::corrugateLinkedSkin loops over all
 * of them), just without this pre-pass's own cross-layer position tracking for itself.
 *
 * Scope limitation (confirmed acceptable for this pass): only \c parts[0] of each layer is
 * tracked, matching \c findLayerSeamsForSpiralize's own scope - any additional disjoint part on a
 * corrugated mesh's layer gets no continuity tracking (its entry stays \c has_ring=false always),
 * falling back to today's absolute per-layer rule for that part, same as before this feature
 * existed. There is no cross-layer part-identity mechanism anywhere in this codebase to build on
 * for the general multi-part case.
 *
 * \param mesh The mesh to compute anchors for - typically only called for meshes whose
 * \c infill_pattern is EFillMethod::CORRUGATED.
 * \return One CorrugationAnchor per layer, sized to \c mesh.layers.size().
 */
std::vector<CorrugationAnchor> computeAnchorsForMesh(const SliceMeshStorage& mesh);

/*!
 * \brief Wall Strip (spec REV 2.6/5.7): one Chain domain's own two walls for one specific layer, in
 * the engine's native units (microns), already labeled Wall A/Wall B.
 */
struct ChainWallPoints
{
    bool found{ false };
    std::vector<Point2LL> wall_a;
    std::vector<Point2LL> wall_b;
};

/*!
 * \brief Wall Strip (spec REV 2.6/5.7): every Chain domain's own two walls for one specific layer,
 * each already labeled Wall A/Wall B per the cross-layer identity already resolved by
 * computeAnchorsForMesh's own pre-pass (\p chain_domain_wall_identities,
 * CorrugationAnchor::chain_domain_wall_identities) - supports stripping more than one Chain domain
 * at a junction simultaneously (e.g. a Tee's own crossbar and stem at once), not just a single
 * tracked domain.
 *
 * Re-derives the same VBCT domain classification computeAnchorsForMesh's own pre-pass already
 * performs - deliberately, since CorrugationAnchor only carries each domain's own identity *point*
 * forward across layers, not its full wall point arrays. A genuine second VBCT invocation for this
 * one layer; callers are responsible for gating this behind the two Wall Strip settings so it costs
 * nothing otherwise (see FffGcodeWriter::addMeshPartToGCode's own strip step for the intended call
 * site).
 *
 * \param part The layer's own part (typically \c parts[0], matching this project's established
 * single-part-per-layer continuity scope).
 * \param settings The mesh's own settings.
 * \param chain_domain_wall_identities This layer's own tracked Chain domain identities, from
 * \c CorrugationAnchor::chain_domain_wall_identities.
 * \param suppressed_hole_identities De Minimis Hole Threshold (spec REV 3.0/3.5/5.9): this layer's
 * own tracked suppressed-hole identities, from \c CorrugationAnchor::suppressed_hole_identities.
 * Applied to this function's own re-derived \c corrugation_input the same way it is at every other
 * VBCT call site, so this function's own domain composition agrees with theirs - see
 * \c filterDeminimisHoles' own doc comment. Defaults to empty (no holes suppressed).
 * \return One entry per Chain domain this layer actually has that could be matched back to \p
 * chain_domain_wall_identities (normally all of them) - empty if \p chain_domain_wall_identities is
 * empty, VBCT rejects the input, or no parts/contours exist this layer.
 */
std::vector<ChainWallPoints> findAllChainDomainWalls(
    const SliceLayerPart& part,
    const Settings& settings,
    const std::vector<ChainDomainWallIdentity>& chain_domain_wall_identities,
    const std::vector<DeminimisHoleIdentity>& suppressed_hole_identities = {});

/*!
 * \brief Wall Strip (spec REV 2.6/5.7, Chain increment): moves \p corrugation_input's own matching
 * contour's covered-range vertices outward, inset by \p half_line_width from \p outline's own true
 * surface, so the corrugation's own boundary moves outboard to fill where a stripped Chain wall
 * used to be, with its own centerline running flush with the true surface rather than half a line
 * width past it (the same real defect already found and fixed once for Ring,
 * `expandCorrugationInputForStrippedWalls`'s own `insetContour` above) - the sub-range analogue of
 * `buildCorrugationInput`'s own whole-contour swap for Ring. See the .cpp's own doc comment for the
 * full design, including why this displaces infill's own existing vertices individually (same
 * order/connectivity throughout) rather than grafting in a foreign point set from outline.
 *
 * \param corrugation_input The Shape about to be fed to VBCT for corrugation, mutated in place
 * (its matched contour is replaced) - typically `part.infill_area` (or a copy of it).
 * \param outline The part's own true, unshelled model boundary (`part.outline`) to expand toward.
 * \param domain_wall_points The domain's own wall (one side) being stripped, in the engine's
 * native units (microns) - same convention `stripWallArcLengthRanges` uses.
 * \param half_line_width Half of `infill_line_width` (microns) - how far in from outline's own
 * true surface the corrugation's own centerline should sit, matching the convention an ordinary
 * wall's own outermost line already follows.
 * \return Whether anything was actually moved.
 */
bool expandChainContourRange(Shape& corrugation_input, const Shape& outline, const std::vector<Point2LL>& domain_wall_points, coord_t half_line_width);

/*!
 * \brief Linked corrugation skin: builds one continuous printed toolpath that alternates between
 * a Ring domain's outer and inner wall every time it crosses a stringer, instead of printing every
 * stringer as an independent segment (\c corrugate()'s own output) - the "Corrugated Linked Skin"
 * setting. Runs one line width inboard of each wall (spec Section 3.1's "trough... offset one line
 * width inboard of the outer Wall so it can weld without overlapping the wall path"), so skin and
 * crossing meet without a kink.
 *
 * Built on \c vbct::build_domain_events (stage10.hpp) - see its own doc comment for the underlying
 * math (why sorting events by their shared \c theta gives one cyclic order valid for both walls at
 * once, why "two stringers, one from each family, overlapping" is a whole-domain phenomenon
 * detected once rather than per pair, and why the domain's own stringer count is rounded up to
 * even before building events). This function does everything build_domain_events itself can't
 * (it only has VBCT's own types to work with): routes each wall through \c offsetWallInboard with
 * zero distance (a per-vertex normal push, deliberately *not* a whole-loop Clipper offset - see its
 * own doc comment for the three real, confirmed-via-direct-user-report bugs traced to relying on a
 * Clipper-produced offset polygon's own output array) purely to reuse its degenerate/empty-wall
 * checks, not to actually offset anything: \p infill_area (via \c corrugation_input,
 * \c buildCorrugationInput) is *already* the region Cura's own wall generation leaves after the
 * wall's own footprint - the same boundary every other infill pattern fills directly, with no extra
 * buffer of its own. An earlier version of this function applied a *further* one-line-width inboard
 * push here, on the theory that spec Section 3.1's "offset one line width inboard of the outer
 * Wall" meant offsetting inboard of \p infill_area's own boundary - confirmed wrong via direct user
 * report and a bounding-box diagnostic at \c wall_line_count=1: the two buffers stacked, pushing
 * the skin roughly *twice* as far inboard of the true wall as intended. Section 3.1's own language
 * describes the relationship to the actual printed wall path, which \p infill_area is already
 * offset from by Cura's own wall generation - not a second buffer this function needs to add.
 * Every event is then sampled from its corresponding wall at the exact same arc-length fraction the
 * raw attachment point was found at (\c DomainEvent::outer_t_frac/inner_t_frac) - valid directly,
 * without any reprojection search or alignment correction, specifically because
 * offsetWallInboard's per-vertex construction guarantees its output matches the input one-to-one -
 * then walks the sorted events, alternating which wall's own vertices are traced between
 * consecutive events, with a straight 2-point jump for each stringer crossing.
 *
 * Requires crosshatch for Ring domains: this function always builds its events with VBCT's CW
 * family enabled, regardless of the caller's own \c corrugated_crosshatch_enabled setting (see
 * this feature's own settings-category doc for why a Ring's linking is meaningless without a
 * "return trip" family to alternate against) - not this project's usual pattern of respecting
 * every setting a caller has independent control over, called out explicitly here since it is an
 * exception to that pattern. A Chain domain does not need this (see build_domain_events' own
 * "Chain domains" doc comment in stage10.hpp for why an open wall pair's own single family
 * already suffices) - crosshatch_enabled is passed the same way for both, it just has no effect
 * on a Chain domain's own events.
 *
 * Chain domain support (spec REV 2.1): produces an *open* path (front() != back(), starting near
 * one of the domain's own caps and ending near the other) rather than Ring's closed loop - the
 * walk below branches on \c DomainEvents::is_ring to know whether to close the alternation back to
 * its own start or simply stop at the last event.
 *
 * Chain junction (Hub) support (spec REV 2.1): before building events, folds any straight
 * pass-through pair of Chain domains sharing a hub point back into one continuous Chain (see
 * \c mergeStraightPassThroughChains' own doc comment in the .cpp for what this does and doesn't
 * attempt), then builds one path *per remaining domain* rather than requiring exactly one -
 * \c infill_area may therefore produce more than one \c OpenPolyline in the result (e.g. a Tee's
 * own crossbar and stem each getting their own path). Deliberately conservative: if *any* domain
 * in the resulting list isn't independently linkable, the whole layer still falls back to
 * \c std::nullopt for every domain, not a per-domain-mixed-output result.
 *
 * Falls back to \c std::nullopt (never blank, never malformed output) for every case this version
 * doesn't support yet: no linkable domain at all in \p infill_area (a Glob domain, or every domain
 * failing independently), a stringer whose straight line would leave the domain (this version
 * doesn't re-clip a shifted crossing segment against non-convex domain edges), or either wall's
 * own offset collapsing (a gap narrower than roughly two line widths). The caller
 * (FffGcodeWriter::processCorrugatedInfill) is expected to fall back to \c corrugate()'s own
 * unlinked output whenever this returns \c std::nullopt, never to blank output.
 *
 * \param infill_area, \param x, \param threshold, \param spacing, \param z, \param
 * crossover_pitch_mm, \param anchor_t0_frac, \param other_wall_t0_frac, \param
 * reverse_canonical_wall, \param chain_anchor_point Same meaning as \c corrugate()'s own
 * parameters of the same names.
 * \param chain_junction_merge_angle_deg Chain junction (Hub) support (spec REV 2.1): the angle
 * tolerance, in degrees from a perfectly straight 180, within which two Chain domains sharing a
 * hub point are considered a straight pass-through and merged - see
 * \c mergeStraightPassThroughChains' own doc comment in the .cpp for the full algorithm. Normally
 * supplied from the \c corrugated_chain_junction_merge_angle setting.
 * \param chain_crosshatch_enabled Chain-specific Crosshatch + Linked Corrugation Skin integration
 * (spec REV 2.4): when true, a Chain domain's own events also include the crosshatch "rigid
 * sliding comb" family (spec REV 2.3), merged with the base family and sorted, so the linked path
 * carries the crosshatch diagonal across layers instead of the plain single-family path. Passed
 * straight through to \c vbct::build_domain_events' own parameter of the same name - see its doc
 * comment for why this is a separate parameter from the \c crosshatch_enabled=true this function
 * always hardcodes for Ring above (that one is a hard Ring-only requirement; this one reflects the
 * user's own \c corrugated_crosshatch_enabled setting and has no effect on Ring domains). Defaults
 * to false (today's single-family Chain linked-skin behavior, unchanged).
 * \param chain_left_near_t_frac, \param chain_left_far_t_frac, \param chain_right_near_t_frac,
 * \param chain_right_far_t_frac Same Chain end-position continuity overrides as \c corrugate()'s
 * own parameters of the same names - see its own doc comment for the full account.
 * \param suppressed_hole_identities Same De Minimis Hole Threshold suppression set as \c
 * corrugate()'s own parameter of the same name - see its own doc comment for the full account.
 * \param transition_layer_request Transition Layer (spec REV 3.3/3.6/5.10): any domain matching
 * this request is excluded from this function's own domain loop entirely, rather than linked -
 * v1 scope decision, since Crosshatch/Linked Corrugation Skin's own gating logic (this function's
 * always-on CW family requirement for Ring, the "not yet linkable" clipped-event gate) has no
 * defined meaning for a full-density solid-fill substitution. A transitioned domain's own lines
 * always come from \c corrugate()'s own \c transition_lines_out parameter instead, regardless of
 * the caller's own \c corrugated_skin_linked setting. Matched the same live-re-match way \c
 * vbct::run_stage10's own transition parameters are (see \c TransitionLayerRequest's own doc
 * comment) - default-constructed (the default) excludes nothing, today's exact pre-feature
 * behavior.
 */
std::optional<OpenLinesSet> corrugateLinkedSkin(
    const Shape& infill_area,
    double x,
    double threshold,
    double spacing,
    coord_t z = 0,
    double crossover_pitch_mm = 0.0,
    std::optional<double> anchor_t0_frac = std::nullopt,
    std::optional<double> other_wall_t0_frac = std::nullopt,
    std::optional<bool> reverse_canonical_wall = std::nullopt,
    std::optional<Point2LL> chain_anchor_point = std::nullopt,
    double chain_junction_merge_angle_deg = 30.0,
    bool chain_crosshatch_enabled = false,
    std::optional<double> chain_left_near_t_frac = std::nullopt,
    std::optional<double> chain_left_far_t_frac = std::nullopt,
    std::optional<double> chain_right_near_t_frac = std::nullopt,
    std::optional<double> chain_right_far_t_frac = std::nullopt,
    const std::vector<DeminimisHoleIdentity>& suppressed_hole_identities = {},
    // Chain wall outer/inner identity continuity (VBCT sign-oscillation investigation) - see
    // CorrugationAnchor::chain_swap_left_right's own doc comment for the full rationale. Pass
    // mesh.corrugation_anchors[layer_nr].chain_swap_left_right directly.
    bool chain_swap_left_right = false,
    const TransitionLayerRequest& transition_layer_request = {});

} // namespace VbctAdapter
} // namespace cura

#endif // CORRUGATED_VBCT_ADAPTER_H
