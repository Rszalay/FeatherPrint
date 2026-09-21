// Copyright (c) 2026 FeatherPrint Corrugated
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CORRUGATED_CORRUGATION_ANCHOR_H
#define CORRUGATED_CORRUGATION_ANCHOR_H

#include <vector>

#include "geometry/Point2LL.h"

namespace cura
{

//! Wall Strip (spec REV 2.6/5.7): cross-layer Wall A/B identity for one Chain domain - see
//! CorrugationAnchor::chain_domain_wall_identities' own doc comment for the full context.
struct ChainDomainWallIdentity
{
    //! The mean of this domain's own `left` and `right` wall points combined, in the engine's
    //! native units (microns) - used only to match this same physical domain against the previous
    //! tracked layer's own list of these, not for anything geometric otherwise. Deliberately not a
    //! cap point (Stage 9's own cap_start/cap_end assignment can flip which end is which between
    //! layers, the same instability CorrugationAnchor::chain_anchor_point already works around) and
    //! not each wall's own separate mean either (left/right can also relabel between layers, the
    //! same instability wall_a_is_left itself exists to work around) - a whole-domain mean is
    //! insensitive to either kind of relabeling, so it stays a stable identity for matching even
    //! when the domain's own internal bookkeeping shifts layer to layer.
    Point2LL identity_point;

    //! True if this domain's own `left` wall is Wall A this layer (and `right` is Wall B); false
    //! means the reverse. See chain_domain_wall_identities' own doc comment for how this is kept
    //! stable across layers.
    bool wall_a_is_left{ true };
};

//! De Minimis Hole Threshold (spec REV 3.0/3.5/5.9): cross-layer identity for one hole this layer
//! is currently treating as ignored (below `corrugated_deminimis_hole_area`, with hysteresis - see
//! CorrugationAnchor::suppressed_hole_identities' own doc comment for the full context).
struct DeminimisHoleIdentity
{
    //! The mean of this hole's own polygon vertices, in the engine's native units (microns) - used
    //! only to re-match this same physical hole against a freshly-built Shape's own holes at each
    //! live per-layer call site (VbctAdapter::filterDeminimisHoles), the same "stable identity for
    //! matching, not a claim about anything geometric otherwise" role
    //! ChainDomainWallIdentity::identity_point already plays for Chain domains.
    Point2LL identity_point;
};

/*!
 * \brief One layer's cross-layer-continuity-tracked corrugation anchor (spec REV 1.4 S:5.3),
 * computed once per mesh by FffGcodeWriter::computeCorrugationAnchors (via
 * VbctAdapter::computeAnchorsForMesh) in a single-threaded pre-pass before CuraEngine's parallel
 * per-layer gcode generation begins - see that function's own doc comment for why this can't be
 * computed from inside the parallel per-layer path itself (VBCT's wall anchor/orientation rules
 * are otherwise a from-scratch, purely-per-layer computation with no memory of what the previous
 * layer chose, which is exactly what makes them unstable under sub-visual input noise).
 *
 * Deliberately plain data - no vendored VBCT geometry types - so this header stays includable
 * from sliceDataStorage.h without coupling it to src/vbct/'s vendored types.
 */
struct CorrugationAnchor
{
    //! Whether this layer had exactly one Ring domain to anchor. False for every layer with no
    //! corrugatable Ring here (no parts, VBCT rejected the input, or a Chain/Glob/multi-Ring
    //! composition) - a legitimate topology-change reset point per spec REV 1.4 S:5.3.5, not an
    //! error. When false, every other field is meaningless and must not be read.
    bool has_ring{ false };

    //! The chosen anchor point on the Ring's canonical (longer) wall, in the engine's native
    //! units (microns) - kept only so the *next* layer's continuity pass can find its own
    //! nearest point to this one. Meaningless when has_ring is false.
    Point2LL anchor_point;

    //! anchor_point's position as a continuous arc-length fraction in [0, 1) along the canonical
    //! wall, in VBCT's own t-parametrization - what actually gets passed to
    //! vbct::run_stage10's anchor_t0_frac parameter. Meaningless when has_ring is false.
    double t0_frac{ 0.0 };

    //! Same as anchor_point/t0_frac, but for the Ring's *other* (shorter) wall - tracked
    //! independently, since it has its own separate continuity requirement (see stage10.hpp's
    //! other_wall_t0_frac doc comment for why tracking only the canonical wall wasn't enough in
    //! practice). Meaningless when has_ring is false.
    Point2LL other_wall_anchor_point;
    double other_wall_t0_frac{ 0.0 };

    //! Whether Stage 9 emitted the canonical wall's points walked in the opposite absolute
    //! direction from the immediately preceding tracked layer - detected via signed_area's own
    //! sign (see stage10.hpp's reverse_canonical_wall doc comment for the full evidence this was
    //! added from: anchor position tracking alone left ~640 sign flips across one real print
    //! uncorrected, since a position override can't fix *which way* the non-anchor stringers walk
    //! from it). When true, gets forwarded to vbct::run_stage10's parameter of the same name so
    //! the *live* per-layer call reverses its own freshly-derived canonical wall to match, not
    //! just this precomputed anchor's own internal bookkeeping. Meaningless when has_ring is
    //! false.
    bool reverse_canonical_wall{ false };

    //! Whether this layer had exactly one Chain domain to anchor (spec REV 2.1, "Chain domain
    //! support") - mutually exclusive with has_ring (a layer's composition is either a Ring or a
    //! Chain in the cases this project tracks continuity for, never both). False for every layer
    //! with no corrugatable Chain here, same legitimate-reset-point meaning as has_ring's own doc
    //! comment. When false, chain_anchor_point is meaningless and must not be read.
    bool has_chain{ false };

    //! The tracked "same real end" point for the Chain's own cap_start/cap_end pair, in the
    //! engine's native units (microns) - kept only so the *next* layer's continuity pass can find
    //! its own nearest match to this one. Passed to vbct::run_stage10's chain_anchor_point
    //! parameter, which orients both of the Chain's walls toward this point instead of a fresh
    //! domain.cap_start every layer - see that parameter's own doc comment in stage10.hpp for why
    //! domain.cap_start alone can't be trusted to mean "the same physical end" across layers.
    //! Unlike Ring, no separate direction flag is needed here: orient_toward's own
    //! nearest-endpoint search already is the direction fix, once given a stable target.
    //! Meaningless when has_chain is false.
    Point2LL chain_anchor_point;

    //! Chain domain *end-position* continuity (as opposed to chain_anchor_point's own "which end"
    //! role): true once a previous tracked layer's own near/far wall points exist to search
    //! against - i.e. this is the Chain analogue of "tracked" further up this function's own
    //! computeAnchorsForMesh implementation for Ring (prev_canonical_anchor_point.has_value()).
    //! When false (first tracked layer in a run, or immediately after any reset), the four
    //! fractions below are meaningless and must not be read - the live per-layer call falls back
    //! to std::nullopt for all four (today's exact pre-tracking behavior: each wall's own full raw
    //! t=0..1 span, oriented via chain_anchor_point alone). Meaningless (and always false) when
    //! has_chain is false.
    bool chain_position_tracked{ false };

    //! The four independently-tracked arc-length fractions (in VBCT's own t-parametrization, one
    //! wall x one end each) that stabilize *where exactly* the Chain's own two real wall ends sit,
    //! not just *which* end is which (chain_anchor_point's own job). Needed because
    //! chain_anchor_point/orient_toward alone only fix the direction/role of each wall - the exact
    //! raw vertex VBCT computes for each end is still free to jitter layer to layer under the same
    //! per-layer-absolute noise Ring's own t0_frac/other_wall_t0_frac already exist to damp (Stage
    //! 9's far_cap() has no cross-layer memory of its own). Confirmed via direct real-print
    //! diagnostic on a Z-invariant hollow airfoil: the tracked near end alone (pre-fix) still
    //! jumped >0.5mm on 42/598 layers, and the untracked far end jumped up to 2.34mm - the same
    //! "both walls need independent tracking, not just the canonical one" lesson Ring's own anchor
    //! continuity fix already learned (Section 5.3), extended here to Chain's own two real ends
    //! rather than Ring's one wraparound anchor. "Near"/"far" is relative to chain_anchor_point
    //! (near = the tracked end, far = the domain's other real end), not a fixed left/right label -
    //! left/right is Stage 9's own upstream bookkeeping and not guaranteed stable layer to layer
    //! either (confirmed directly: raw per-layer point counts swap which of left/right is larger),
    //! so these are matched to whichever of the domain's own left/right wall is nearer the previous
    //! layer's own tracked point for that wall, exactly mirroring how anchor_point/
    //! other_wall_anchor_point above protect Ring against the same class of role swap. Passed to
    //! vbct::run_stage10/build_domain_events's own chain_left_near_t_frac/chain_left_far_t_frac/
    //! chain_right_near_t_frac/chain_right_far_t_frac parameters - see their own doc comment in
    //! stage10.hpp for exactly how each wall's own base-family sampling interpolates between them.
    //! Meaningless when chain_position_tracked is false.
    double chain_left_near_t_frac{ 0.0 };
    double chain_left_far_t_frac{ 1.0 };
    double chain_right_near_t_frac{ 0.0 };
    double chain_right_far_t_frac{ 1.0 };

    //! Chain wall outer/inner identity continuity (VBCT sign-oscillation investigation): true when
    //! this layer's own domain.left should be swapped with domain.right before
    //! vbct::run_stage10/build_domain_events treat one of them as "left" when assigning outer/inner
    //! roles for Linked Corrugation Skin's own alternating walk. Confirmed via real production
    //! geometry: Stage 9's own domain.left/domain.right identity for a Chain (unlike Ring, which
    //! canonicalizes to the longer wall - a Chain's two walls are close enough in length that
    //! length alone doesn't discriminate) can flip layer to layer with no real geometry change,
    //! silently reversing which wall the linked-skin walk calls "outer".
    //!
    //! As of the chain_domain_stability_and_sign_oscillation investigation's second round, this is
    //! computed by a direct, per-layer vbct::assign_wall_ab call (Section 3.7/5.11's canonical Wall
    //! A/B rule set) on that layer's own real contours + domain.left/right - deliberately NOT via
    //! cross-layer position tracking (the same role-swap detection chain_left_near_t_frac's own
    //! computation performs internally, purely to keep those four fractions correctly paired, a
    //! separate concern). An earlier version of this field reused that same cross-layer tracking
    //! for this purpose too; confirmed via real capture data that it can catch an initial identity
    //! swap but miss the swap-back, after which its own running reference desyncs and thrashes
    //! between 0/1 even while the real geometry stays completely stable. assign_wall_ab needs no
    //! cross-layer history at all, so there is nothing to desync - confirmed stable (99.45%,
    //! 544/547 real adjacent-layer transitions) when re-evaluated fresh, independently, on every
    //! single real layer of the same capture, including every point the tracked version got
    //! confused. Applies from a Chain domain's very first tracked layer onward - unlike the
    //! position-tracking fields above, this does not require chain_position_tracked to be true
    //! first, since assign_wall_ab needs no previous layer to compare against.
    bool chain_swap_left_right{ false };

    //! Wall Strip (spec REV 2.6/5.7): cross-layer Wall A/B identity for *every* Chain domain this
    //! layer has, not just the single one has_chain/chain_anchor_point track above - a Chain
    //! domain's two walls have no fixed outer/inner identity the way a Ring's do (Section 3.4), so
    //! which physical wall a given domain calls "Wall A" (vs "Wall B") must be tracked cross-layer,
    //! the same way reverse_canonical_wall tracks a Ring's own wall-direction identity - Stage 9's
    //! own domain.left/domain.right assignment is upstream bookkeeping, not a stable physical
    //! label, the same instability class documented at prepare_domain_sampling's own
    //! domain.left/right comment in stage10.cpp.
    //!
    //! Deliberately independent of has_chain/chain_anchor_point's own single-tracked-domain scope
    //! (that pair exists for Chain end-point *position* continuity, unrelated to Wall Strip, and
    //! stays scoped to one domain per this project's own established continuity-tracking limit -
    //! see Section 5.6): every Chain domain present this layer gets an entry here, supporting
    //! multiple domains at a junction (e.g. a Tee's own crossbar and stem) each independently
    //! having Wall A/B stripped at the same time. Empty when this layer has no Chain domains at
    //! all; each entry's own ChainDomainWallIdentity::identity_point is how VbctAdapter's own
    //! per-layer domain lookup (findAllChainDomainWalls) matches a freshly-derived VBCT domain back
    //! to the correct entry here.
    std::vector<ChainDomainWallIdentity> chain_domain_wall_identities;

    //! De Minimis Hole Threshold (spec REV 3.0/3.5/5.9): every hole this layer is currently
    //! treating as ignored for VBCT's own domain classification (below
    //! `corrugated_deminimis_hole_area`, with cross-layer hysteresis already applied - a hole
    //! whose area sits right at the threshold does not flip in and out of suppression on ordinary
    //! per-layer area noise the way a bare per-layer threshold check would, the same
    //! "require a decisive change, not a bare threshold crossing" principle this project's other
    //! continuity fixes already use for position/identity tracking). Computed once here, in this
    //! same single-threaded pre-pass, and applied - never re-decided - at every live per-layer call
    //! site (VbctAdapter::filterDeminimisHoles, called consistently by corrugate(),
    //! corrugateLinkedSkin(), and findAllChainDomainWalls(), matching each entry's own
    //! identity_point against that layer's own freshly-built Shape's own holes by nearest point).
    //! Independent of has_ring/has_chain and every other field above - a layer can have suppressed
    //! holes regardless of its own Ring/Chain composition. Empty when this layer has no holes below
    //! threshold at all (the common case).
    std::vector<DeminimisHoleIdentity> suppressed_hole_identities;

    //! Transition Layer whole-part fallback (2026-09-15 full-layer redesign; narrowed again when
    //! per-domain solid-fill substitution was restored - see ring_transition_layer_triggered/
    //! chain_transition_layer_domain_identities below): true when either of two signals differs
    //! from the immediately preceding layer - this layer's own raw layer.parts.size() (catching a
    //! domain physically splitting into two disconnected SliceLayerParts; confirmed necessary on a
    //! real V-shaped part whose two legs fully separate at a notch, and has no natural per-domain
    //! identity of its own, since this pipeline's own domain-composition tracking only ever looks
    //! at parts[0] - a second SliceLayerPart isn't tracked by any of it at all), or the governing
    //! domain's own hysteresis-stabilized stringer_count changing (see its own doc comment -
    //! adjacent layers no longer have the same number of stringers at all, so no stringer on this
    //! layer can line up with the corresponding one below). A triggered layer replaces the
    //! *entire* part's corrugation for that layer with a full-part concentric fill (see
    //! FffGcodeWriter::processCorrugatedInfill), the coarse fallback for cases with no clean
    //! per-domain story. Domain-count-change (a domain appearing/disappearing within parts[0]) is
    //! deliberately NOT one of this field's own signals any more - it has a clean per-domain
    //! identity, so it drives ring_transition_layer_triggered/
    //! chain_transition_layer_domain_identities instead, substituting solid fill for just the
    //! affected domain rather than falling back to this whole-part mechanism.
    bool transition_layer_triggered{ false };

    //! Transition Layer, per-domain solid-fill substitution, Ring's own domain-count-change signal
    //! (spec REV 3.3/3.6/5.10, Trigger 2): true when this layer's own Ring domain just appeared
    //! relative to the immediately preceding layer (has_ring flipped false->true) - this project
    //! tracks at most one Ring domain per layer (this function's own established scope
    //! limitation), so that flip *is* the domain-count-change event; no separate identity match is
    //! needed the way Chain's own multi-domain case requires. Meaningless (and always false) when
    //! has_ring is false this layer - a domain that just disappeared has no "this layer" of its own
    //! to substitute solid fill into.
    bool ring_transition_layer_triggered{ false };

    //! Transition Layer, per-domain solid-fill substitution, Chain's own domain-count-change signal
    //! (spec REV 3.3/3.6/5.10, Trigger 2): the subset of this exact layer's own
    //! chain_domain_wall_identities whose own identity_point has no sufficiently close match among
    //! the previous tracked layer's own list - i.e. a genuinely new Chain domain this layer, not
    //! one already being tracked. Deliberately a SEPARATE, capped match from
    //! chain_domain_wall_identities' own (uncapped) nearest-match above - that one is a
    //! resequencing question (there is always some domain to re-identify against; a wrong match
    //! only costs a mislabeled Wall A/B, self-correcting from the domain's own current geometry),
    //! this one is a presence/absence question (does this domain actually correspond to anything
    //! that existed last layer, or not) - the exact "an uncapped nearest-match always finds
    //! *something* once the candidate set is non-empty" bug class documented in
    //! Lessons Learned/de_minimis_hole_threshold_lessons_learned_260906.md's own Lesson 1, applied
    //! here before a real bug forces it rather than after. A domain that *disappeared* has no
    //! "this layer" of its own to substitute solid fill into, so only the appearance direction is
    //! checked. Empty when no Chain domain appeared this layer (the common case) or when this
    //! layer has no Chain domains at all.
    std::vector<Point2LL> chain_transition_layer_domain_identities;

    //! Stringer-count stability (corrugation-start-jitter investigation, 2026-09-13): true once a
    //! previous tracked layer's own stringer count exists to compare against - same
    //! "position_tracked" pattern as chain_position_tracked. When false, stringer_spacing_override
    //! is meaningless and the live per-layer call falls back to today's bare
    //! corrugated_stringer_pitch setting - a legitimate reset point (first tracked layer, or
    //! immediately after a topology change), not an error.
    bool stringer_count_tracked{ false };

    //! An effective `spacing` value (engine-native microns) that, fed into
    //! vbct::run_stage10/domain_stringers in place of the bare corrugated_stringer_pitch setting,
    //! reproduces this layer's *stabilized* stringer count rather than whatever a fresh,
    //! independent `round(governing_length / spacing)` would give.
    //!
    //! Root cause this exists to fix: domain_stringers' own stringer count
    //! (`n = round(governing_length / spacing) + 1`) is recomputed from scratch every layer, from
    //! that layer's own instantaneous wall length and spacing - both vary smoothly with real
    //! taper, but round() introduces a hard decision boundary. Confirmed directly (Aerofoil-2412-
    //! Sweep, corrugation-start-jitter investigation): governing_length/spacing sat close to a
    //! half-integer, so ordinary per-layer floating-point noise (not even real taper) flipped the
    //! chosen n between adjacent layers - and since every stringer's own t = i/(n-1) position
    //! depends on n *globally*, a single flip reshuffles the entire stringer layout for that
    //! layer, not just adds/removes one at an end. This is the exact "massive stringer distortion"
    //! symptom reported, one level higher in the pipeline than every earlier jitter fix in this
    //! investigation (CDT-triangulation level) - same underlying class of bug, different stage.
    //!
    //! Computed once here with hysteresis (same "require a decisive change, not a bare threshold
    //! crossing" principle as suppressed_hole_identities/role_swapped elsewhere in this file): the
    //! chosen integer stringer count only changes when the live ratio has moved decisively past
    //! the previous choice's own rounding boundary, not on every ordinary sub-percent wobble.
    //! Meaningless when stringer_count_tracked is false.
    coord_t stringer_spacing_override{ 0 };

    //! The hysteresis-stabilized integer stringer count (stringer_spacing_override's own
    //! "chosen_round_m + 1") this layer's own governing domain resolved to - kept alongside
    //! stringer_spacing_override so CorrugationAnchor::transition_layer_triggered's own post-pass
    //! (VbctAdapter::computeAnchorsForMesh) can compare it against the previous layer's own choice.
    //! A real, decisive change here means adjacent layers no longer have the same number of
    //! stringers at all, so no stringer on this layer can align with the corresponding stringer on
    //! the layer below any more - a structural discontinuity Transition Layer treats the same way
    //! as a domain appearing/disappearing/splitting. Meaningless when stringer_count_tracked is
    //! false.
    int stringer_count{ 0 };
};

} // namespace cura

#endif // CORRUGATED_CORRUGATION_ANCHOR_H
