// Copyright (c) 2026 FeatherPrint Corrugated
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CORRUGATED_BEAD_LOOP_H
#define CORRUGATED_BEAD_LOOP_H

#include <optional>
#include <vector>

#include "geometry/Point2LL.h"
#include "utils/Coord_t.h"

namespace cura
{

/*!
 * \brief Bead Loop Scarf (Corrugated spec REV 3.9, Sections 3.8 / 5.12).
 *
 * A Bead Loop is a run of printed beads that starts and ends at the same point, so it can be printed
 * as one continuous extrusion. Its seam gets a flow-only scarf: the first \c length of the loop ramps
 * flow up from \c start_ratio to 1, and after the loop is complete the nozzle re-prints that same
 * span with the complementary ramp, (1 - start_ratio) down to 0. Z never moves.
 *
 * Why: LW-PLA's foaming melt is compressible, so after a retract/travel/unretract the bead comes out
 * thin for a long distance. Without this, Corrugated's outer wall was the first thing extruded after
 * every restart (A-40 Fuselage, 2026-11-18: evenly spread holes between outer-wall layer lines).
 */

/*!
 * Flow factor at fraction \p t (0..1) along a Bead Loop Scarf ramp.
 *
 * Opening ramp: lerp(start_ratio, 1, t). Closing overprint: lerp(1 - start_ratio, 0, t). For every
 * t the two sum to exactly 1, so the seam span carries one bead in total.
 *
 * This is the one ramp shared by every Bead Loop Scarf (closed walls in LayerPlan, the Linked Skin
 * loop and the Chain wall+skin loop in FffGcodeWriter). Do not re-implement it elsewhere.
 */
double scarfFlowFactor(double t, double start_ratio, bool closure);

//! Total length of an open polyline.
coord_t polylineLength(const std::vector<Point2LL>& path);

/*!
 * The part of \p path between arc-length positions \p from and \p to (clamped to the path), with
 * interpolated end points. Returns at least the two end points when from < to.
 */
std::vector<Point2LL> polylineSpan(const std::vector<Point2LL>& path, coord_t from, coord_t to);

/*!
 * One sub-segment of a scarf ramp: extrude to \c to at \c flow_factor (relative to normal flow).
 */
struct ScarfRampSegment
{
    Point2LL to;
    double flow_factor;
};

/*!
 * Split \p span (a polyline that is the whole ramp, start to end) into sub-segments no longer than
 * \p step, each carrying the ramp's flow factor at its mid-point (scarfFlowFactor). The first
 * sub-segment starts at span.front(), which the caller must already be at.
 */
std::vector<ScarfRampSegment> scarfRampSegments(const std::vector<Point2LL>& span, coord_t step, double start_ratio, bool closure);

/*!
 * Case 1 of spec REV 3.9 Section 3.8: a Chain domain with exactly one wall stripped. The kept wall
 * and the Linked Skin together form one Bead Loop.
 */
struct ChainBeadLoop
{
    //! Print the kept wall back to front, so the loop runs counter-clockwise.
    bool wall_reversed = false;
    //! The skin as printed after the wall: front() is the wall's (printed) end, back() its (printed)
    //! start, so wall + skin closes. Includes the cap connectors.
    std::vector<Point2LL> skin;
};

/*!
 * Build the Bead Loop for one kept (open) Chain wall and its Linked Skin, or nothing if they do
 * not form a clean loop (the caller then keeps today's separate wall + skin emission).
 *
 * Cap closure (user decision 2026-11-18): at each cap the skin end is joined to the wall end when
 * they are within \p join_tolerance. Otherwise the skin's trailing run along the kept wall (points
 * within \p on_wall_tolerance of it) is dropped down to its first point, the foot of the last
 * rung, and the wall end is joined straight to that foot. The wall has already printed the end
 * face, and the dropped run only doubled the kept wall. This applies only when the gap is at most
 * \p max_cap_gap and the dropped run at most \p max_reroute_run long.
 *
 * The loop is oriented counter-clockwise, which fixes the seam (the printed wall's start) on the
 * same cap from layer to layer.
 */
std::optional<ChainBeadLoop> buildChainBeadLoop(
    const std::vector<Point2LL>& wall,
    std::vector<Point2LL> skin,
    coord_t join_tolerance,
    coord_t on_wall_tolerance,
    coord_t max_cap_gap,
    coord_t max_reroute_run);

} // namespace cura

#endif // CORRUGATED_BEAD_LOOP_H
