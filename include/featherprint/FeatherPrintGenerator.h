// Copyright (c) 2026 Rszalay
// CuraEngine is released under the terms of the AGPLv3 or higher
#pragma once

#include <vector>

#include "geometry/OpenLinesSet.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Polygon.h"
#include "geometry/Polyline.h"
#include "geometry/Shape.h"
#include "settings/Settings.h"
#include "utils/Coord_t.h"
#include "utils/ExtrusionLine.h"

namespace cura
{

/*!
 * Generates the FeatherPrint conformal stringer toolpath for one layer.
 *
 * Returns a VariableWidthLines (one ExtrusionLine) containing the full continuous
 * perimeter walk with embedded Stringer Trace and Lacing Trace features for this layer.
 *
 * Integration: called from WallsComputation::generateWalls(SliceLayerPart*) when
 * infill_pattern == FEATHERPRINT, replacing the WallToolPaths call.
 *
 * Cura settings consumed:
 *   wall_line_count          >= 1  (must be non-zero for pipeline to run)
 *   featherprint_line_width  (mm)
 *   featherprint_stringer_count
 *   featherprint_helix_pitch (deg/mm)
 *   top_layers = 0, bottom_layers = 0  (suppress skin)
 */
class FeatherPrintGenerator
{
public:
    /*!
     * Generate the wall toolpath for one layer.
     * \param outline   The outer contour of the slice (from SliceLayerPart::outline).
     * \param z         Layer z in µm.
     * \param settings  Mesh settings.
     * \return          One VariableWidthLines bin (inset_idx=0) with a single closed
     *                  ExtrusionLine representing the full perimeter+stringer path.
     *                  Returns empty if the contour is too small to slice.
     */
    VariableWidthLines generate(
        const Shape& outline,
        coord_t z,
        const Settings& settings,
        double helix_phase = 0.0,
        Point2LL phase_origin = Point2LL(0, 0),
        double R_ref = 0.0);

    /*!
     * Generate the Flange wall stack for one closed-boundary-loop layer.
     * ramp_index: 0 = innermost (first) Flange layer, n-1 = topmost brim layer.
     * Returns multiple closed ExtrusionLines (one per Wall), innermost first, outermost last.
     */
    VariableWidthLines generateFlange(
        const Shape& outline,
        const Settings& settings,
        int ramp_index,
        double helix_phase = 0.0,
        Point2LL phase_origin = Point2LL(0, 0),
        double R_ref = 0.0);

    /*!
     * Former (Spec REV 3.6, as corrected REV 3.7): a ramped Wall stack that builds up to a peak
     * then back down, mirrored in z, rather than Flange's one-directional ramp from a fixed
     * OML — but like Flange, the outer face stays flush with the OML throughout; all added
     * thickness builds inward (`buildFormerWallStack()` is a thin wrapper around
     * `buildFlangeWallStack()`, unmodified). Not pinned to the model's top; can occur at any
     * closed-perimeter Layer. `ramp_position` is 0 at the first/last Layer of the band (no
     * growth yet, ordinary single Wall) through n at the peak Layer — see
     * buildFormerWallStack(). Gusset (Stringer/Lacing x Former) is embedded exactly like
     * Flare Rim is for Flange, at every Layer of the band (both ramps), since a Stringer
     * passing through crosses both.
     *
     * Also used, unmodified, for Collar (Spec REV 4.0): Collar is a Former-style band whose
     * `ramp_position` comes from a different pre-pass (peaks at boundary-edge Z-spans instead
     * of Lacing crossings — see SliceMeshStorage::fp_collar_ramp) rather than different
     * geometry. This function has no notion of which feature is calling it; both just pass a
     * ramp magnitude.
     */
    VariableWidthLines generateFormer(
        const Shape& outline,
        const Settings& settings,
        int ramp_position,
        double helix_phase = 0.0,
        Point2LL phase_origin = Point2LL(0, 0),
        double R_ref = 0.0);

    /*!
     * Returns the number of Lacing (Stringer x Stringer) collisions this Layer's outline would
     * produce at the given helix phase — a read-only query reusing the same anchor-placement
     * machinery generate() uses internally, without emitting any geometry or touching
     * seamPoint()/innerOffset() state. Used by the Former-band detection pre-pass
     * (FffPolygonGenerator.cpp) to locate Former peaks (Layers where Lacing fires) without
     * duplicating the arc-length/anchor math externally — that math depends on this class's
     * own private ArcParam, so it can't be replicated outside it without either exposing the
     * whole machinery or accepting the duplication risk; this single-purpose query is the
     * narrower, safer alternative. Static since it needs no instance state.
     */
    static int countLacingCollisions(const Shape& outline, const Settings& settings, double helix_phase, Point2LL phase_origin, double R_ref);

    /*!
     * Open-manifold counterpart to countLacingCollisions(), for a Layer whose boundary loop is
     * interrupted (Whip/hole case) rather than a plain closed Shape. Without this, the Former
     * detection pre-pass was blind to any Lacing crossing whose Z happened to fall within an
     * open-manifold Layer's own span (a hole in the side wall) — since the pre-pass only tested
     * closed Layers, a crossing landing there was never evaluated at all, not merely
     * de-prioritized, so no Former band ever appeared anywhere near that hole even though one
     * should have. Builds the same full virtual ring (real arcs + gap chords) WallsComputation
     * itself builds for actual generation (centroid, oriented arcs, cumulative arc-length,
     * Phase-Origin projection), then runs the same per-arc anchor-placement/collision test
     * generateFlangeOpen's own Flare-anchor code uses — restricted to anchors that fall within a
     * real arc's own span (not on a virtual gap chord), matching how real geometry is actually
     * placed. Curvature-Weighted Stringer Density is not applied here, consistent with
     * OpenLayerParams' own documented scope narrowing for open-manifold Layers generally.
     * Returns the number of colliding pairs found across every arc on this Layer.
     */
    static int countLacingCollisionsOpen(const OpenLinesSet& open_polylines, const Settings& settings, double helix_phase, Point2LL phase_origin);

    // Parameters shared across all open-polyline arcs on the same layer.
    // Computed once from the full virtual ring (all arcs + gap chords) so that
    // every arc uses a consistent perimeter length, reference angle, and centroid.
    struct OpenLayerParams
    {
        Point2LL centroid;
        double full_ring_total{};    // perimeter of the full virtual ring
        double full_ring_arc_ref{};  // arc-length position of the +X reference ray intersection
        double arc_start_in_ring{};  // start position of this specific arc within the full ring
    };
    // NOTE: Curvature-Weighted Stringer Density (Spec REV 2.6) is NOT applied on open-manifold
    // (Whip/Miter) layers in this pass — anchor spacing there stays raw-arc-length, uniform, as
    // before. Warping the full virtual ring (real arcs + gap chords, built by the caller across
    // possibly several independent OpenPolyline arcs) would require sharing a warp lookup table
    // across every arc call for the layer, not just a couple of scalars in OpenLayerParams — a
    // real extension, deliberately deferred rather than half-implemented. Flagged as an explicit
    // scope narrowing, consistent with several other features here that already accept a
    // narrower or approximate treatment specifically for the open-manifold case (e.g. Miter's
    // radial-offset Wall approximation).

    /*!
     * Generate the wall toolpath for one open-manifold layer (open polyline input).
     * open_poly must already be oriented CCW in math coordinates (caller's responsibility).
     * params carries the full-ring geometry shared across all arcs on this layer.
     */
    VariableWidthLines generateOpen(const OpenPolyline& open_poly, coord_t z, const Settings& settings, double helix_phase, const OpenLayerParams& params);

    /*!
     * Generate the Flange wall stack for one Layer where the boundary loop is interrupted
     * by a boundary edge (a slot/hole reaching the open top) — the Miter case. open_poly is
     * the single open arc for this Layer (same input generateOpen takes). The outer Wall is
     * closed with an ordinary Whip Terminal at each end, continuing the same Terminal column
     * as ordinary Whip layers below the Flange. Inner Walls (including the innermost, which
     * also carries Flare Rim insertion) are left open at both ends per the Miter rule — no
     * distinct Miter geometry is generated; the outer Wall's Terminal is relied on to weld
     * the inner Walls' open ends as a side effect of print-order (inner Walls first, outer
     * Wall last — the caller must reverse the usual Flange print order for this Layer).
     * Does not yet insert Flare Rims (first pass: get the wall-stack + Miter terminal
     * behaviour generating correctly before layering in Stringer/Lacing integration).
     */
    VariableWidthLines generateFlangeOpen(const OpenPolyline& open_poly, coord_t z, const Settings& settings, int ramp_index, double helix_phase, const OpenLayerParams& params, int diag_layer_nr = 0);

    /*!
     * Former's own Miter-equivalent (Cuff, Spec REV 3.6): mechanically identical to
     * generateFlangeOpen's own Miter handling — outer Wall gets an ordinary Whip Terminal at
     * each end, continuing the same Terminal column; inner Wall(s), including the innermost
     * (which also carries Gusset/Flare-Rim insertion), are left open, welded by the outer
     * Wall's Terminal sweep (print order: inner Walls first, outer Wall last, same as Miter).
     * REV 3.6 states Cuff introduces no geometry of its own beyond this, matching Miter's own
     * precedent exactly — this function is deliberately a near-duplicate of
     * generateFlangeOpen for that reason, not a distinct design.
     *
     * Also used, unmodified, for Placket (Collar x Whip, Spec REV 4.0): a Collar band's own
     * ramp continuing past a boundary Layer into the hole's open span is exactly this
     * situation — a tapering multi-Wall ring meeting a locally open arc — and Placket's own
     * spec text says it reuses Miter's mechanism exactly, substituting the Collar's own
     * (Layer-by-Layer decreasing) Wall count for the Flange's fixed one. That decrease falls
     * out for free from `ramp_position` shrinking Layer by Layer as the Collar pre-pass's own
     * ramp tapers — no separate Placket code path exists or is needed.
     */
    VariableWidthLines generateFormerOpen(const OpenPolyline& open_poly, coord_t z, const Settings& settings, int ramp_position, double helix_phase, const OpenLayerParams& params, int diag_layer_nr = 0);

    /*!
     * Generate the Punchout Line + Terminal geometry for one hole-gap on an open-manifold
     * layer (Spec REV 3.3), gated by featherprint_punchout_enabled.
     *
     * prev_wall_poly and next_wall_poly are the two REAL open polylines the caller
     * (WallsComputation) determined bound this hole in ring order: P0 = prev_wall_poly.back()
     * is the END of one real open polyline and P1 = next_wall_poly.front() is the START of
     * the NEXT, around the layer's full virtual ring. This distinction matters: with more
     * than one hole open on the same layer, a single real open polyline typically runs from
     * one hole's edge, around through solid wall material, to a DIFFERENT hole's edge — its
     * own front()/back() are usually NOT a matched pair bounding one hole. The ring-adjacent
     * pairing the caller performs is what actually identifies each hole's own two edges; only
     * in the degenerate single-hole/single-arc-on-this-layer case does prev_wall_poly and
     * next_wall_poly happen to be the same polyline.
     *
     * Internally this builds a synthetic ArcParam over the straight CHORD between P0 and P1
     * and reuses the same tangent-blend/Terminal machinery Whip Terminal uses — but
     * parametrized against that chord only, never against either real polyline's own
     * arc-length (which is solid wall, not hole interior — placing Punchout geometry there,
     * as an earlier attempt did, puts it outside the hole).
     *
     * Returns empty if Punchout is disabled, the endpoints are closer together than 2x the
     * requested featherprint_punchout_gap cutback, or the resulting chord has no room left
     * for a Terminal loop at each end (falls back to a plain straight segment in that case).
     *
     * The cutback gap between each Terminal's own true endpoint (Q0/Q1) and the real wall
     * corner (P0/P1) is intentionally left unwelded — an earlier revision closed it with a
     * tangent-matched "weld stub" curve, tapered in near a hole's own top/bottom, but removed
     * it again after real-print testing found the stub made the Punchout materially harder to
     * break away, defeating the point of a removable support feature.
     *
     * contour_pts, if non-empty, replaces the straight middle segment between the two
     * Terminals with a plain polyline through these world-space points instead: the line
     * becomes exactly Start Terminal -> contour_pts[0] -> ... -> contour_pts[N-1] -> End
     * Terminal, no chord walk in between. The caller (WallsComputation) computes these by
     * lofting between the real wall contours immediately below and immediately above this
     * hole (SliceMeshStorage::fp_punchout_gap_spans' contour_below/contour_above, sampled once
     * per hole in the pre-pass) at this Layer's own Z — so the Line's shape gradually matches
     * the wall it meets at the hole's own top and bottom. Pass empty to keep the plain straight
     * chord (e.g. when the hole reaches the very top/bottom of the mesh and no bounding closed
     * Layer exists on one side).
     *
     * half_width_ends prints this entire Punchout (Terminals included) at half the nominal
     * line width instead of full width — the caller sets this exactly on the Layer that forms
     * the literal top of the hole's own span and the Layer that forms its literal bottom, per
     * spec, to weaken those connections and make the whole feature easier to break away.
     * Geometry is unaffected (R1/R2/D/W are still derived from the full nominal width, so the
     * profile keeps its normal proportions) — only the printed bead width changes.
     *
     * out_q0/out_q1, if non-null, receive the actual (angle-adaptive-gap-adjusted) cutback
     * points Q0/Q1 this call computed — the same points the real Punchout Terminal loops are
     * anchored to. Added for Shelf (Spec REV 4.2, see generateShelf() below), which needs to
     * place its own loops clear of those exact points, not re-derive its own approximation of
     * them (the two would drift out of sync with any future change to the gap logic here).
     * Left untouched (not zeroed) if this call returns empty before Q0/Q1 are computed.
     */
    VariableWidthLines generatePunchout(const OpenPolyline& prev_wall_poly, const OpenPolyline& next_wall_poly, coord_t z, const Settings& settings, const OpenLayerParams& params, const std::vector<Point2LL>& contour_pts = {}, bool half_width_ends = false, Point2LL* out_q0 = nullptr, Point2LL* out_q1 = nullptr);

    /*!
     * Shelf (Spec REV 4.2): mandatory support for the upper Collar's own peak-thickness Wall
     * stack, which sits directly above a hole's own local arc-opening at the hole's topmost
     * Layer — the one place along the ring where the Placket taper's usual assumption (the
     * still-closed remainder of the ring can carry the Wall stack gradually into the open
     * span) doesn't hold, since the hole removes exactly that supporting material. Reuses the
     * Stringer Trace's own loop/crossover profile (appendTrace()) — not a flat surface,
     * deliberately, so the support stays as breakable as the rest of Punchout (see this
     * function's own Reach/loop-count derivation in the .cpp for why loops, not a solid shelf).
     *
     * Q0/Q1 must be the exact same cutback points generatePunchout() computed for this same
     * hole/Layer (see its own out_q0/out_q1 parameters) — the real Punchout Terminal is
     * anchored there, not at the raw wall corners, so Shelf's own loops need the same reference
     * points to actually stay clear of it.
     *
     * collar_ramp_peak is the ramp position (== featherprint_collar_layers) the upper Collar
     * band one Layer above will peak at — used to derive each loop's own inward Reach via
     * buildFormerWallStack(collar_ramp_peak, w)'s own innermost Wall offset, less Lw/2. The
     * caller is responsible for only calling this when that Collar band genuinely peaks there
     * (see SliceMeshStorage::fp_collar_ramp) — this function does not re-check that itself.
     *
     * Loop count is floor(available_span / (featherprint_stringer_width * w)), evenly spaced,
     * where available_span excludes the same end zones generatePunchout()'s own Terminals
     * occupy (matching its own W/R1-based boundary convention exactly, not re-derived).
     * Returns empty if the available span holds fewer than one loop.
     */
    VariableWidthLines generateShelf(const Point2LL& Q0, const Point2LL& Q1, const Point2LL& centroid, const Settings& settings, int collar_ramp_peak);

    /*!
     * Returns the world-space position of the seam (helix 0 departure) for this layer.
     * Valid only after generate() has been called for the same z / settings combination.
     * Used by WallsComputation to set the z-seam hint on the part.
     */
    Point2LL seamPoint() const { return seam_pt_; }

    /*!
     * Returns the inward offset (from the outline passed to generate()/generateFlange(), in
     * the same coord_t units as w) from the OML to the innermost printed Wall's own inner
     * face for this layer — the region a caller should treat as this layer's "inner area"
     * (used for top/bottom skin generation: WallsComputation offsets the layer outline inward
     * by this amount to populate SliceLayerPart::inner_area, which was previously left empty
     * unconditionally, silently disabling top/bottom skin on every FeatherPrint layer
     * regardless of the featherprint_top_layers/bottom_layers... i.e. the stock Cura
     * top_layers/bottom_layers settings). For an ordinary (non-Flange) Layer this is a single
     * Wall's own width (w); for a Flange Layer it is the full Wall-stack depth to the
     * innermost Wall's inner face. Valid only after generate()/generateFlange() has been
     * called for the same settings; only meaningful for closed-perimeter Layers — an
     * open-manifold (Whip) Layer represents a boundary edge the designer left open, and the
     * caller should leave inner_area empty there rather than calling this at all.
     */
    coord_t innerOffset() const { return inner_offset_; }

    /*!
     * Curvature-Weighted Stringer Density (Spec REV 2.6): accumulates one Layer's
     * length-weighted local-radius-of-curvature contribution into running sums, for the
     * whole-mesh R_avg pre-pass (R_avg = weighted_sum / length_sum once every Layer's
     * contribution has been added — computed once, sequentially, before per-layer wall
     * generation). ring_pts is the same closed point ring the Phase Pre-Pass already gathers
     * for helix-phase purposes — a real closed polygon for an ordinary Layer, or Anchor
     * Distribution's own virtual closed ring (real arcs + gap chords) for an open-manifold
     * Layer; both are handled identically here for the same reason the existing phase integral
     * already treats them the same way (a warped-length re-parametrization doesn't care whether
     * an edge is real material or a virtual gap chord).
     */
    static void accumulateCurvatureStats(const std::vector<Point2LL>& ring_pts, double& weighted_sum, double& length_sum);

    /*!
     * Real total and warped total arc-length of ring_pts, given R_ref = k_ref * R_avg. Used by
     * the Phase Pre-Pass to compute this Layer's contribution to the helix-phase integral as
     * layer_height / (W_total(z) * tan(theta_h)) in place of raw L(z), once Curvature-Weighted
     * Stringer Density is active. Pass R_ref <= 0 to get total == total_warped (feature
     * inactive, or R_avg not yet known / degenerate). phase_origin is this Layer's own Anchor
     * Distribution Phase Origin (see SliceMeshStorage::fp_phase_origin) — required so the
     * internal curvature-resampling grid is phase-locked to that stable landmark rather than to
     * ring_pts' own arbitrary (slicer-determined) starting vertex; without this, the resampling
     * grid's phase drifts unpredictably layer to layer even on an unchanging true shape, which
     * aliases a smooth curvature field into an apparently uncorrelated signal. Callers must
     * compute this Layer's own Phase Origin BEFORE calling this function.
     */
    static void computeWarpedTotal(const std::vector<Point2LL>& ring_pts, double R_ref, const Point2LL& phase_origin, double& total, double& total_warped);

private:
    // ---- Arc-length parameterization of a closed or open polyline -----------

    struct ArcParam
    {
        std::vector<double> cum_len;
        double total{};
        const Polyline* poly{};  // base-class pointer; works for Polygon and OpenPolyline
        bool is_open{ false };   // true for OpenPolyline: no closing segment, pointAt clamps
        // True if poly's stored vertex order is mathematically CCW (positive signed area, for
        // a closed Polygon) — a global, topology-level invariant, computed once when this
        // ArcParam is built rather than tested per-point. Used by resolveFrame to orient
        // tangent/inward-normal WITHOUT any centroid-relative test: for a simple polygon with
        // consistent winding, "inward" is always "tangent rotated toward the interior side"
        // regardless of concavity — a centroid-relative test (the previous approach) can pick
        // the wrong side in a concave region or near a hole, where "toward the centroid" isn't
        // reliably "toward the material." For an open arc (is_open), ccw is always true: the
        // caller already guarantees open_poly is oriented CCW in math coordinates (see
        // generateOpen's own doc comment and WallsComputation's arc-orientation step) — there's
        // no independent signed area to compute for an open (non-closed) polyline.
        bool ccw{ true };

        Point2LL pointAt(double s) const;
        Point2LL tangentAt(double s) const;

        // Nearest point on this polyline to an arbitrary world-space target point (Spec REV
        // 2.2 Anchor Distribution's Phase Origin). No centroid, no ray, no angle — a plain
        // point-to-segment min-distance scan over every edge, same cost order as the
        // centroid-ray-cast this replaces (referenceArcPos, removed). Returns the arc-length
        // position of the projection.
        double nearestArcPos(const Point2LL& target) const;

        // ---- Curvature-Weighted Stringer Density (Spec REV 2.6) --------------------------
        //
        // Local radius of curvature at arc-length s, estimated against a UNIFORMLY RESAMPLED
        // copy of this polyline (see ensureResampled), not the raw vertices directly. Every
        // prior approach here — a fixed arc-length window, a fixed vertex count, a window that
        // grows to a minimum physical span — eventually failed on a real part's mesh, and all
        // of them failed for the same underlying reason: the raw polyline's vertex spacing is
        // wildly non-uniform (sub-mm in some places, several mm in others, confirmed via
        // diagnostic logging), and every windowing scheme's failure mode was really just a
        // symptom of trying to reason about "how many real facets does my window span" against
        // that non-uniform spacing. Resampling once, up front, to even spacing removes the
        // question entirely: every subsequent curvature estimate is a plain three-point turning
        // angle between adjacent resampled points, which are guaranteed (by construction) to be
        // close to the same physical distance apart everywhere on this ring. Returns a large
        // fixed value (1e9) for a resampled window with no measurable turning (genuinely
        // straight, or the ring/arc is too degenerate to resample at all) rather than letting
        // ds/turn blow up — a numerical safety measure, not a designer-facing limit.
        double curvatureRadiusAt(double s) const;

        // Lazily builds resample_pts_: this ring's own shape sampled at points evenly spaced by
        // arc-length (via pointAt), roughly `step` apart (closed rings divide the total into a
        // whole number of equal intervals; open arcs sample step-spaced points from 0 to total
        // inclusive). Cached per ArcParam instance (mutable — called from the const
        // curvatureRadiusAt) since a single ArcParam is queried many times during one buildWarp
        // or accumulateCurvatureStats pass.
        //
        // Sample 0 sits at arc-length resample_phase_s, not at s=0 (the slicer's own, arbitrary
        // vertex order) — this was a real correctness bug, not just noise: cum_len[0]/s=0 is
        // defined by whichever vertex the slicer happened to emit first for that Layer, which
        // can reindex layer to layer with no relation to the actual shape. Building the resample
        // grid from that raw origin meant the grid's own phase shifted unpredictably between
        // layers, aliasing even a smooth true curvature field into an apparently uncorrelated
        // signal — exactly the class of bug Anchor Distribution (REV 2.2/2.3) already fixed once
        // for anchor placement, crept back in here one layer removed. Callers MUST set
        // resample_phase_s (typically via nearestArcPos(phase_origin), the same stable landmark
        // Anchor Distribution itself uses) BEFORE the first call that triggers ensureResampled
        // (buildWarp, or curvatureRadiusAt directly) — see buildWarp's call sites in generate()/
        // generateFlange()/computeWarpedTotal for the required call order.
        void ensureResampled(double step) const;
        mutable std::vector<Point2LL> resample_pts_;
        mutable bool resample_built_{ false };
        double resample_phase_s{ 0.0 };

        // Populates cum_warp/total_warped: the warped arc-length coordinate W(s) = integral of
        // rho(s') ds' from 0 to s, where rho(s) = clamp(curvatureRadiusAt(s) / R_ref, kMinRho,
        // kMaxRho). R_ref = k_ref * R_avg is the caller's already-computed whole-part reference
        // radius (see SliceMeshStorage::fp_r_ref). The clamp on rho (not on curvatureRadiusAt's
        // own large-value return) is what actually prevents a straight segment from producing
        // an unbounded warped length, and also floors rho away from zero so a very tight curl
        // can't collapse a nonzero real span to zero warped length. No-op (cum_warp left empty)
        // if R_ref <= 0 — toWarped/fromWarped fall back to the identity (raw arc-length) in
        // that case, so callers can unconditionally call buildWarp and check nothing further.
        void buildWarp(double R_ref);

        // Real arc-length s -> warped arc-length W(s), by linear interpolation within the
        // segment s falls in (mirrors pointAt's own segment-walk structure exactly, replacing
        // cum_len/total with cum_warp/total_warped). Identity (returns s unchanged) if
        // buildWarp was never called (cum_warp empty) — i.e. Curvature-Weighted Density is
        // inactive for this ArcParam.
        double toWarped(double s) const;

        // Inverse of toWarped: warped arc-length W -> real arc-length s. Identity if buildWarp
        // was never called.
        double fromWarped(double w) const;

        std::vector<double> cum_warp;     // parallel to cum_len, in warped units; empty until buildWarp() is called
        double total_warped{ 0.0 };       // W_total for this Layer's ring; meaningless while cum_warp is empty
    };

    static ArcParam buildArcParam(const Polygon& poly);
    static ArcParam buildArcParamOpen(const OpenPolyline& poly);
    static Point2LL centroidBbox(const Polygon& poly);
    static const Polygon* largestPoly(const Shape& shape);

    // ---- Skin-Normal Tangent-Blend Placement (Self-Reference/03-conformal-placement-transform.md
    //      "Skin-Normal Variant") -----------------------------------------------------------
    //
    // Replaces the earlier centroid-radial transform (r = R(theta) - y about the layer centroid).
    // Anchor resolution (theta_anchor, R_a) is unchanged; only per-point placement differs: each
    // canonical point is placed by affine-blending between two RIGID frames built from the real
    // perimeter's own local tangent/normal at the feature's Start and End extents, rather than by
    // projecting radially toward the centroid. Convention: canonical +Y = inward (toward centroid),
    // matching the spec's "y - direction from anchor toward centroid" and the WIP Parametric
    // Canonical Toolpaths doc's Point Tables (opposite of this file's prior ly<0=inward convention).

    // A rigid world-space frame anchored at a point on the real perimeter: S = position, (tx,ty) =
    // unit tangent (direction of increasing arc-length), (nx,ny) = unit inward normal. Both are
    // derived purely from the polyline's own global winding (ArcParam::ccw), not a centroid-
    // relative test — robust on concave/non-star-convex shapes, where "toward the centroid"
    // isn't reliably "toward the material's interior" (see resolveFrame's fuller comment).
    struct TangentFrame
    {
        Point2LL S;
        double tx{}, ty{};
        double nx{}, ny{};
    };

    // Resolves S(s), T(s), N(s) directly from the real polyline's own arc-length parametrization
    // (ArcParam::pointAt / tangentAt) — NOT from an angle ray-cast against the centroid. Width
    // (the lateral extent between a feature's two blend frames) must equal the caller's requested
    // physical distance exactly regardless of local skin curvature or angle relative to the
    // centroid; an angle-then-ray-cast resolution only preserves that distance where the perimeter
    // happens to be locally circular about the centroid, and distorts it (roughly by 1/cos of the
    // tangent's angle off the centroid-perpendicular) everywhere else. Arc-length is intrinsic to
    // the curve, so it has no such distortion. (The anchor's OWN position is still resolved by
    // angle/ray-cast elsewhere — that's a separate concern, distributing anchors around the part.)
    //
    // w (nominal line width) scales the internal tangent-smoothing window (see the .cpp's own
    // comment) — it was originally a fixed 0.3mm regardless of w, which is fine at the 0.4mm
    // line width the ratio was tuned against but becomes a much LARGER fraction of w (1.5x
    // instead of 0.75x) at a finer 0.2mm line width, over-smoothing local tangent direction
    // right where Lacing's collision detection (a flat `d < w` test) is most sensitive to it —
    // confirmed as a real contributor to increased Stringer/Lacing overlap at fine line widths.
    static TangentFrame resolveFrame(const ArcParam& arc, const Point2LL& centroid, double s, coord_t w);

    // Two rigid placements (Copy A at the feature's Start, Copy B at its End) of the entire
    // canonical point cloud, affine-blended by t = (lx - x_s) / (x_e - x_s). x_s/x_e are the
    // canonical x-coordinates (w-units) of the FIRST and LAST points emitted in traversal order
    // (which for a CW-mirrored feature are the table's End/Start, swapped) - not necessarily the
    // profile's x-extrema. x_sign folds the CW helix mirror into the tangent component only (the
    // t-blend itself is direction-agnostic).
    struct BlendPlacement
    {
        TangentFrame A, B;
        double x_s{}, x_e{};
        double x_sign{ 1.0 };

        Point2LL place(double lx, double ly, coord_t w) const;
    };

    // s_anchor is the anchor's own arc-length position — located by Anchor Distribution (Spec
    // REV 2.2: fixed-point projection + arc-length walk), not by angle/ray-cast. Copy A/B are
    // placed at s_anchor +/- x*w along the real perimeter's own arc-length, so the physical
    // distance between them is exactly (x_e - x_s) * w regardless of local curvature or the
    // anchor's position relative to the centroid. (theta_anchor/R_a, and the internal
    // arc-length-via-angle round-trip they required, are gone as of REV 2.2 — s_anchor is
    // already known directly by every caller.)
    static BlendPlacement buildBlendPlacement(double s_anchor, double x_sign,
                                              double x_s, double x_e,
                                              const Point2LL& centroid, const ArcParam& arc, coord_t w);

    // One Wall of a Flange's wall stack: offset = inward distance from the OML to the
    // Wall's own toolpath centreline (µm); width = the Wall's own extrusion width (µm).
    struct FlangeWallDesc
    {
        coord_t offset;
        coord_t width;
    };

    // Shared by generateFlange() and generateFlangeOpen(): the Flange build-up rule from
    // the spec (1.5w at ramp 0, +0.5w total thickness per ramp layer thereafter), returned
    // outer-first (index 0 = outer Wall, back() = innermost Wall).
    static std::vector<FlangeWallDesc> buildFlangeWallStack(int ramp_index, coord_t w);

    // Former's own wall-stack rule (Spec REV 3.6, as corrected REV 3.7): a thin wrapper
    // calling buildFlangeWallStack(ramp_position, w) directly, unmodified. Former's outer face
    // stays flush with the OML throughout, exactly like Flange's — the only structural
    // difference from Flange is that the caller (FffPolygonGenerator's own pre-pass) mirrors
    // ramp_position in z about a peak, rather than ramping one-directionally, so this function
    // itself needs no ramp-shape logic of its own. (An earlier version of this function grew
    // the stack symmetrically outward AND inward from the base Wall's own centreline, per a
    // since-corrected misreading of REV 3.6's own Profile text — see REV 3.7's as-built entry
    // — which produced a Former band visibly sitting proud of the surrounding skin.)
    //
    // Also used, unmodified, for Collar (Spec REV 4.0) — see generateFormer()'s own doc
    // comment for how Collar reuses this whole chain.
    static std::vector<FlangeWallDesc> buildFormerWallStack(int ramp_position, coord_t w);

    // Maps a Wall's position in the outer-first wall-stack array (wi: 0=outer..n_walls-1=
    // inner) to the inset_idx that controls its actual print order (ascending inset_idx =
    // later in print order, per InsetOrderOptimizer's inside_out convention — see
    // FffPolygonGenerator's forced optimize_wall_printing_order=false/inset_direction=
    // inside_out override for FeatherPrint meshes). The OML (wi=0) always gets inset_idx 0
    // (prints last, required for the Miter weld). The IML (wi=n_walls-1) gets inset_idx 1
    // (prints second-to-last, not first) — it's the Wall most likely to be printing over an
    // overhang and benefits from having the other inner/middle Walls already laid down for
    // adhesion. Any buried middle Walls fill inset_idx 2..n_walls-1 (printed earliest, order
    // among themselves doesn't matter). Degenerates to the plain outer-then-inner-last order
    // when there are 2 or fewer Walls (no middle Walls to reorder around).
    static int flangePrintInsetIdx(int wi, int n_walls);

    // Perpendicular offset of an open polyline, point by point, along EACH point's own local
    // inward normal (resolveFrame's smoothed-tangent-window + global-winding normal) rather than
    // toward a single Layer-wide centroid -- see resolveFrame's own doc comment for why the
    // radial-toward-centroid approximation this replaces (Aug 2026) breaks down on an elongated
    // or off-axis hole, letting Wall material extend into the hole instead of retreating into
    // solid material. arc must already be built from `poly` (buildArcParamOpen(poly)) -- callers
    // already have this in scope for other purposes and should reuse it, not rebuild it.
    static OpenPolyline normalOffsetOpen(const ArcParam& arc, const Point2LL& centroid, coord_t offset, coord_t w);

    static void appendPolySegment(ExtrusionLine& line, const ArcParam& arc,
                                  double s0, double s1, coord_t w, bool add_start);

    // Tessellates one canonical arc through the tangent-blend placement and appends to line.
    // lx: along-perimeter offset from anchor (positive = CCW-forward).
    // ly: inward depth (positive = inward toward centroid, 0 = on perimeter).
    static void appendCanonicalArc(ExtrusionLine& line,
                                   double cx, double cy, double R,
                                   double a_start_deg, double a_end_deg,
                                   bool cw_arc, int segs,
                                   const BlendPlacement& blend,
                                   coord_t w, bool skip_first = false);

    // Ray from centroid at angle theta; returns arc-length of first polygon intersection, or -1.
    static double arcLengthAtAngle(const ArcParam& arc, const Point2LL& centroid, double theta);

    // Thin-Section Pruning (Spec REV 2.4, proposed). True if a ray cast from the anchor
    // (s_anchor on arc) along its own local inward normal — the winding-based normal
    // resolveFrame already computes (REV 2.3), not a centroid-directed ray, for the same
    // reason Anchor Distribution moved off centroid — hits arc's own real perimeter within
    // distance D*w. If so, the local material is thinner than this feature's Depth and its
    // geometry should be suppressed at this anchor for this Layer (the anchor position itself
    // is unaffected — callers must still use it for anchor continuity, only skip drawing).
    // A small arc-length window around the anchor is excluded from the hit test so the ray
    // doesn't trivially register a hit against its own immediate neighborhood. Both the
    // detection radius (== D, no margin) and the exclusion window width are explicit,
    // tentative starting points per the spec, not settled values — flagged for revisiting
    // once tested against real parts.
    static bool isThinSection(const ArcParam& arc, const Point2LL& centroid, double s_anchor, double D, coord_t w);

    // Collision Pruning (Spec REV 4.5) Part 2 — signed-area self-intersection pruning for an
    // open-arc Wall's own candidate polyline (generateFormerOpen()/generateFlangeOpen()). At a
    // thin neck the naive per-point offset (normalOffsetOpen()) can fold back on itself; the
    // folded region is a genuine, substantial negative (reversed-winding) area, not a small
    // toolpath artifact (see the .cpp definition's own doc comment for the full rationale).
    // main_line: the pruned candidate path, same two original free ends unless entirely
    // consumed. extra_rings: zero or more same-winding nested sub-loops (e.g. a far lobe's own
    // valid Wall material, disconnected from the main path by the neck) that must still be
    // emitted, just as their own separate closed rings.
    struct PruneResult
    {
        std::vector<Point2LL> main_line;
        std::vector<std::vector<Point2LL>> extra_rings;
    };
    static PruneResult pruneSelfIntersectionsSignedArea(const std::vector<Point2LL>& pts_in, bool ccw_ref, coord_t w, const char* diag_tag = nullptr);

    // Collision Pruning (Spec REV 4.5) Part 3 — Skin-crossing pass. Tests the Wall's own
    // candidate path against a separate reference curve (the plain ordinary Skin ring); any
    // crossing is a genuine defect (no winding/area ambiguity, unlike the self-pass above, since
    // the Skin is the actual boundary of legitimate local material). Prunes the excursion
    // between each pair of consecutive crossings. wall_closed/skin_closed let one implementation
    // serve both the closed-ring generators (Polygon Skin) and the open-arc ones (OpenPolyline
    // Skin, built via normalOffsetOpen() like the Walls themselves — never Shape::offset(),
    // which can silently "bridge" across the same thin necks this pass exists to catch).
    static std::vector<Point2LL> pruneAgainstSkinPoints(std::vector<Point2LL> pts, bool wall_closed, const std::vector<Point2LL>& skin_pts, bool skin_closed, const char* diag_tag = nullptr);

    // Stringer Trace — Spec REV 2.0 (corrected). G=0.5 (fixed default, not user-exposed),
    // R2 = featherprint_stringer_width/2, R1 = R2+G, D = featherprint_feature_depth (shared).
    // Point Table: Anchor(0,0) Start(-G,0) P1(R2,R1) P2(R2,D-R2) P3(-R2,D-R2) P4(-R2,R1) End(G,0).
    // P1/P4 sit on the opposite side from Start/End, so Arc1/Arc3 genuinely cross the
    // centreline near the top — a real self-intersecting crossover, per REV 2.0.
    // Path: Arc1 Start->P1 R1 CCW c(-G,R1); Line1 P1->P2; Arc2 P2->P3 R2 CCW c(0,D-R2);
    //       Line2 P3->P4; Arc3 P4->End R1 CCW c(G,R1).
    // CW helix: same centres/radii, traversed in reverse order with each arc's direction flipped
    // and x_sign=-1 (spec: CW Helix Mirroring).
    static void appendTrace(ExtrusionLine& line,
                            double s_anchor,
                            const Point2LL& centroid, const ArcParam& arc,
                            bool is_cw, coord_t w,
                            double D, double W, bool skip_first = false);

    // Whip Terminal — Spec REV 2.0. R1,R2 match Stringer's (R1=R2+G, R2=featherprint_stringer_width/2).
    // D = featherprint_feature_depth (shared — same value used by Stringer, matching R1/R2).
    // Re-verified by hand against the corrected Stringer R1=R2+G formula (spec's own note had
    // flagged this table stale from before that fix): Arc2 and Arc3 share one centre/radius
    // (C2==C3 since R2=W/2 makes P3==P4), so together they form a single smooth 180° arc over
    // the top regardless of which of R1/R2 is larger — no self-intersection or tangent
    // discontinuity is introduced by R1 now exceeding R2. Validity mirrors Stringer's own
    // D >= W+G bound (D >= R1+R2, else Line1 inverts).
    // Point Table: Start(R1,0) P1(0,R1) P2(0,D-R2) P3(R2,D) P4(W-R2,D) P5(W,D-R2) End(W,Lw/2)
    // (End.y = Lw/2 = 0.5 in w-units — confirmed intentional per REV 2.0: "the path ends at
    // End, near the surface... on the far side", not exactly on the perimeter).
    // Path: Arc1 Start->P1 R1 CW c(R1,R1); Line1 P1->P2; Arc2 P2->P3 R2 CW c(R2,D-R2);
    //       Line2 P3->P4 (splay_L inserted here); Arc3 P4->P5 R2 CW c(W-R2,D-R2); Line3 P5->End.
    // s_anchor: arc-length position of the open endpoint.
    // x_sign  : +1 to extend forward (CCW, for the start endpoint), -1 to extend backward (CW, for the end endpoint).
    // splay_L > 0 widens the flat-bottom Line2 span by L*w, widening the terminal to cover the
    // deleted Stringer Trace footprint (Splay feature).
    static void appendTerminal(ExtrusionLine& line,
                               double s_anchor, double x_sign,
                               const ArcParam& arc, const Point2LL& centroid,
                               coord_t w,
                               double D, double R1, double R2, double W,
                               bool reversed = false,
                               double splay_L = 0.0);

    // Lacing Trace — per Spec REV 2.0's centers-first derivation, anchor at the midpoint between
    // two colliding stringers. R1 = Lw/2, R2 = (D - Lw)/2, D = featherprint_feature_depth (shared),
    // W = featherprint_lacing_width, G = 0.5 (fixed).
    // Centres: C1(G+R1,R1) C2(W/2-R2,D-R2) C3(-C2.x,C2.y) C4(-C1.x,C1.y).
    // Point Table: Start(C1.x,0) P1(C1.x,2R1) P2(C2.x,2R1) P3(C2.x,D) P4(0,D)
    //              P5(C3.x,D) P6(C3.x,2R1) P7(C4.x,2R1) End(C4.x,0).
    // Path (spec order): Arc1 Start->P1 R1 CCW C1; Line1 P1->P2; Arc2 P2->P3 R2 CW C2;
    //       Line2 P3->P4; Line3 P4->P5; Arc3 P5->P6 R2 CW C3;
    //       Line4 P6->P7; Arc4 P7->End R1 CCW C4.
    // Implementation walks the REVERSE of the spec's own table (End->P7->...->Start, each arc
    // direction flipped) because the spec's literal Start sits ahead of the anchor and End
    // behind — backwards relative to the caller's stitching direction. Symmetric profile:
    // x_sign = +1 unconditionally (no CW helix mirror needed).
    static void appendLacingTrace(ExtrusionLine& line,
                                  double s_anchor,
                                  const Point2LL& centroid, const ArcParam& arc,
                                  coord_t w, double D, double W);

    // Flare Rim (Stringer/Lacing × Flange) — per Parametric Canonical Toolpaths (WIP).md: a
    // single-fillet 3-segment channel (Arc1 -> Line1 -> Arc2) anchored at (0,0) on the true OML
    // (the outer wall's own toolpath centreline):
    //   Q = total thickness (w-units) of the current Flange ramp layer's Wall stack — the sum
    //       of each Wall's own width, not a plain Wall count, since some ramp layers include a
    //       half-width (buried or outer) Wall ("Wall Number" per the WIP doc, corrected: a
    //       count would under-charge a stack containing a half-width Wall, making the channel
    //       shallower than the Wall stack it's welding into actually is)
    //   D = featherprint_feature_depth setting (shared, w-units); R1 = min(D - Q, W/2) (w-units) —
    //       clamped to W/2 so Line1's span (W - 2*R1) can't go negative and cross the two end
    //       arcs over each other when D is large relative to W
    //   W = 2.0 for a Stringer anchor, 3.0 for a Lacing anchor (flat span at y=R1)
    //   oml_shift = the gap (w-units) between oml_arc's own ly=0 reference (the raw slice
    //               polygon) and the true OML — 0 except at the first Flange ramp layer,
    //               where the outer wall is a half-width wall offset outward of the raw
    //               polygon. Every canonical y-value is placed at (y - oml_shift) in
    //               oml_arc's own frame so the profile still anchors to the true OML.
    // theta_anchor/R_a must be evaluated against the OML (oml_arc), not the innermost wall,
    // and oml_arc must also be passed as the `arc` used for the profile's tangent-blend frames.
    static void appendFlareRim(ExtrusionLine& line,
                               double theta_anchor, double R_a, double x_sign,
                               const Point2LL& centroid, const ArcParam& oml_arc,
                               double Q, double D, double oml_shift, double W, coord_t w, bool skip_first = false);

    struct Anchor
    {
        double s;
        bool   is_cw;
        bool   skip{ false };
        int    pair_idx{ -1 }; // index of paired opposite-direction anchor when skip=true
    };

    // Seam point written by generate(), read by seamPoint()
    Point2LL seam_pt_{};

    // Inner-area offset written by generate()/generateFlange(), read by innerOffset()
    coord_t inner_offset_{ 0 };
};

} // namespace cura
