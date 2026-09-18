// Copyright (c) 2026 FeatherPrint Corrugated
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CORRUGATED_WALL_STRIP_H
#define CORRUGATED_WALL_STRIP_H

#include <optional>
#include <vector>

#include "utils/Coord_t.h"
#include "utils/ExtrusionLine.h"

namespace cura
{

/*!
 * \brief Wall Strip (spec REV 2.6, Section 5.7) - Ring domains only, Phase 1.
 *
 * Removes the wall lines belonging to a Ring's own outer contour (Wall A) and/or inner/hole
 * contour (Wall B) from every inset depth in \p wall_toolpaths, so the corrugation's own Linked
 * Corrugation Skin wall-following path can substitute for the removed wall's structural role
 * instead of merely welding alongside it. Operates directly on the already-finished
 * `part.wall_toolpaths` (a post-hoc trim of Arachne's own output, not a modification of wall
 * generation itself) - see this project's own Wall Strip investigation for why: Arachne's real
 * output unit, `ExtrusionLine`, already has an `is_closed_` flag and every downstream consumer
 * (`InsetOrderOptimizer`, `LayerPlan::addWall`) already handles open and closed walls uniformly,
 * so a whole-contour omission needs no new infrastructure anywhere else in the pipeline.
 *
 * `wall_toolpaths` is binned by inset depth, not by which physical contour a wall belongs to -
 * `wall_toolpaths[0]` (a `VariableWidthLines`, itself `std::vector<ExtrusionLine>`) holds every
 * contour's own outermost wall together in one bin, so a Ring part's outer boundary and its own
 * hole boundary both show up as separate `ExtrusionLine`s at the *same* `inset_idx_ == 0`, with no
 * existing flag distinguishing them (`ExtrusionLine::is_outer_wall()` only checks `inset_idx_ ==
 * 0`, not which physical contour that is). Distinguished here by enclosed area instead, computed
 * independently at each inset depth rather than tracked across depths: a hole is by definition
 * strictly inside its own part's outer boundary, so it always encloses less area, at every inset
 * depth - no cross-layer or cross-inset identity tracking is needed for this to hold.
 *
 * Fails safe: if any inset depth's own line set doesn't look like a simple, single-hole Ring -
 * exactly two closed `ExtrusionLine`s - the whole call is a no-op, leaving `wall_toolpaths`
 * completely untouched and returning false. This matches this project's established "never guess,
 * fall back to prior behavior" convention (the same rule `corrugateLinkedSkin` already applies
 * whenever a layer/domain isn't linkable yet) rather than risk stripping the wrong contour or
 * leaving some inset depths stripped and others not.
 *
 * Chain and Glob domains are entirely out of scope for this pass - callers are responsible for
 * only calling this when the layer's own part is known to have exactly one corrugatable Ring
 * domain (e.g. via the existing `CorrugationAnchor::has_ring`, already computed by
 * `computeCorrugationAnchors`'s pre-pass); this function has no domain-classification concept of
 * its own and will happily "succeed" against any part that merely looks topologically like a
 * single-hole Ring, whether or not VBCT actually corrugated it.
 *
 * \param wall_toolpaths The part's own wall toolpaths (as built by `WallsComputation`/
 * `WallToolPaths`), mutated in place. Safe to mutate directly: each layer's own `SliceLayerPart`
 * (and therefore its own `wall_toolpaths`) is exclusively owned by that layer's parallel gcode
 * task by the time this runs - `WallsComputation`'s own earlier slicing pass never re-runs here.
 * \param strip_wall_a Remove the outer contour's own wall lines (Wall A, spec Section 3.4).
 * \param strip_wall_b Remove the inner/hole contour's own wall lines (Wall B, spec Section 3.4).
 * \return Whether anything was actually stripped (false if both flags are false, if
 * `wall_toolpaths` is empty, or if the fail-safe topology check above didn't pass).
 */
bool stripRingWalls(std::vector<VariableWidthLines>& wall_toolpaths, bool strip_wall_a, bool strip_wall_b);

/*!
 * \brief Wall Strip (spec REV 2.6, Section 5.7) - Chain domains, Phase 2.
 *
 * Generalizes `stripRingWalls` from "omit a whole contour" to "cut an arc-length sub-range out of a
 * contour" - needed because a Chain domain's own wall is very often only *part* of a larger wall
 * polygon Cura's Arachne pass generates as one continuous closed loop (the clearest real example:
 * a Tee's own crossbar, where a single physical wall loop is shared across multiple VBCT Chain
 * domains meeting at a junction). Ring's own case (the wall *is* the domain) is the degenerate
 * special case of this where the covered range is the entire loop - `stripRingWalls` stays the
 * entry point for that case (already tested, zero regression risk); this function is for the
 * genuinely partial case.
 *
 * \p domain_wall_point_sets holds one entry per side being stripped this call (the Chain domain's
 * own `domain.left` and/or `.right`, reprojected into the engine's native units, microns) - not
 * necessarily the same point sequence as the real, already-printed wall, the same gap
 * `stripRingWalls`' own boundary companion (`buildCorrugationInput`'s expansion,
 * `VbctAdapter.cpp`) already bridges for Ring, here via nearest-point-on-polyline projection
 * instead of whole-contour substitution. Each set's *entire* sampled wall is used, not just its
 * two end points, deliberately: a corridor's two side walls (Wall A and Wall B) both terminate
 * near the *same* two physical end-cap locations, so end caps alone can't tell them apart - only
 * their own middle portions, which run along genuinely different physical contours (or genuinely
 * different arcs of the *same* one), can. Matching (and each set's own resulting covered
 * arc-length range) is therefore determined by averaging every sample's own nearest-projected
 * position, not just interpolating between two endpoints.
 *
 * Every set to strip is passed in one call, not one call per set, deliberately: for a simple
 * (non-junction) Chain domain, Wall A and Wall B are typically two different arcs of the *same
 * single* physical contour, not genuinely separate ones - cutting one out first would leave that
 * contour open, and a second, independent call could no longer find a *closed* candidate to match
 * against at all (this function only ever matches closed contours - see below). The same is true
 * across domains at a junction: a Tee's crossbar and stem can each have their own Wall A/B share
 * one physical wall loop, so all of it - every domain's every requested side - has to be resolved
 * against that loop's own still-closed original shape before anything is cut.
 *
 * At every inset depth, independently (mirroring `stripRingWalls`' own per-depth independence -
 * see its own doc comment for why this is safe and sufficient, not a shortcut): every set in \p
 * domain_wall_point_sets is matched to whichever `ExtrusionLine` in that depth's own bin it best
 * fits - not forced to share one common contour with every other set in the call. Sets that match
 * the same contour are grouped and have their covered ranges removed together (same mechanism as
 * before: however many open kept arcs remain - zero if the combined ranges cover the whole loop,
 * one for the ordinary single-side-stripped case, or more when multiple separated ranges are cut
 * from the same loop at once); sets that match genuinely different contours are processed as
 * separate, independent groups and no longer block each other. Every contour with no matching set,
 * and every other domain's own already-un-stripped segment of a shared contour, is left completely
 * untouched.
 *
 * Fails safe per inset depth (matching `stripRingWalls`' own convention, generalized to groups): if
 * any individual set fails to match any contour at a given depth, or projects to a single
 * degenerate point, the whole call is a no-op, leaving `wall_toolpaths` completely untouched - a
 * failure anywhere still aborts everything, just no longer because two unrelated sets happened not
 * to share a contour.
 *
 * \param wall_toolpaths The part's own wall toolpaths, mutated in place - see `stripRingWalls`' own
 * doc comment for why this is safe.
 * \param domain_wall_point_sets One entry per side being stripped this call, each the domain's own
 * wall for that side, in the engine's native units (microns). Entries with fewer than 2 points are
 * ignored; if none remain, this is a no-op.
 * \return Whether anything was actually stripped.
 */
bool stripWallArcLengthRanges(std::vector<VariableWidthLines>& wall_toolpaths, const std::vector<std::vector<Point2LL>>& domain_wall_point_sets);

/*!
 * \brief Chain end-linking connector (spec REV 2.6 design, implemented here): finds the open wall
 * endpoint nearest \p target, across every inset depth in \p wall_toolpaths - used to locate a
 * one-wall-stripped Chain domain's own un-stripped wall endpoint nearest the corrugation's own
 * Linked Corrugation Skin terminal point there, so a short connector segment can close what would
 * otherwise be a visible open seam at that real end.
 *
 * Only considers *open* (`is_closed_ == false`) `ExtrusionLine`s - the one-wall-stripped case is
 * exactly what leaves an open arc behind (`stripWallArcLengthRanges`' own doc comment); a closed
 * wall (nothing stripped there) has no dangling endpoint to connect to and is skipped. Only an
 * `ExtrusionLine`'s own two true endpoints (`front()`/`back()`) are considered, never an interior
 * vertex - the connector is meant to close the domain's own real end, not attach mid-wall.
 *
 * Fails safe: returns `std::nullopt` if \p wall_toolpaths has no open line, or if the nearest open
 * endpoint found is farther than \p max_dist from \p target - matching this project's own "never
 * guess, fall back to no connector" convention used throughout Wall Strip, rather than bridge two
 * points that likely aren't actually the same physical seam.
 *
 * \param wall_toolpaths The part's own wall toolpaths, read only (not mutated - unlike
 * `stripRingWalls`/`stripWallArcLengthRanges`, the connector itself is applied to the corrugation's
 * own toolpath, not the wall - see this project's own investigation for why).
 * \param target The point to search near - normally the corrugation's own Linked Skin path
 * terminal point at one of the domain's real ends.
 * \param max_dist The largest distance, in the engine's native units (microns), at which a found
 * endpoint is still considered "the same seam" rather than an unrelated wall elsewhere in the part.
 * \return The nearest open wall endpoint within \p max_dist, or `std::nullopt` if none qualifies.
 */
std::optional<Point2LL> findNearestOpenWallEndpoint(const std::vector<VariableWidthLines>& wall_toolpaths, Point2LL target, coord_t max_dist);

} // namespace cura

#endif // CORRUGATED_WALL_STRIP_H
