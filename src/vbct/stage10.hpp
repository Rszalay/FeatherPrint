// Stage 10: corrugation stringer generation. Port of vbct/stage10.py --
// see spec REV 2.1 S:13. For each Ring/Chain domain (Glob domains are
// skipped), reparametrizes both walls by arc length and emits the
// straight stringer segment v1(t)-v2(t) for a sampled set of t values,
// clipped to stay inside the domain.
#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "geometry.hpp"
#include "stage9.hpp"

namespace vbct {

using StringerEdge = std::pair<Point2, Point2>;

struct Stringer {
    std::vector<StringerEdge> pieces;  // >=1 segment; >1 only when clipping split it
};

struct DomainStringers {
    std::vector<Stringer> stringers;
    // Transition Layer (FeatherPrint Corrugated extension, spec REV 3.3/3.6/5.10): true when this
    // domain's own stringers above were built at the caller's own solid-fill spacing/crosshatch-off
    // override (run_stage10's own transition_* parameters), not the caller's ordinary spacing -
    // i.e. this is a full-density Transition Layer substitution for this one domain on this one
    // layer, not ordinary corrugation. Lets VbctAdapter::stage10ToLines split the two apart (a
    // Transition Layer needs Cura's own native bridging print settings, applied only to its own
    // lines). False for every domain when the caller passes no transition override at all (today's
    // exact pre-feature behavior).
    bool is_transition_layer{ false };
};

struct Stage10Result {
    std::vector<DomainStringers> domains;
};

// The following three functions are this file's own internal wall-length/centroid/reference-
// angle helpers (used by domain_stringers below), given external linkage - rather than kept
// anonymous-namespace-private like the rest of this file's helpers - specifically so
// VbctAdapter::computeAnchorsForMesh's cross-layer continuity pre-pass (spec REV 1.4 S:5.3) can
// reuse them for its own "first layer in a run" absolute-anchor pick instead of duplicating this
// logic. Behavior and definitions are otherwise unchanged from before this exposure.
double wall_length(const std::vector<Point2>& points);
Point2 polygon_centroid(const std::vector<Point2>& points);
double reference_angle_arc_length(const std::vector<Point2>& points, const Point2& centroid);
double signed_area(const std::vector<Point2>& points);
std::vector<Point2> match_winding(const std::vector<Point2>& left, const std::vector<Point2>& right);
// Orients `points` so points[0] is whichever of its own two endpoints is nearer `target` -
// exported (spec, Chain domain end-position continuity) so
// VbctAdapter::computeAnchorsForMesh's own tracking can orient a Chain domain's left/right walls
// toward the same target this file's own prepare_domain_sampling uses, before searching along
// them - see this function's own doc comment in stage10.cpp for the full rationale.
std::vector<Point2> orient_toward(const std::vector<Point2>& points, const Point2& target);

// \param phase_offset Crossover Pitch (2026-09-15 redesign): the *raw*, undivided crossover-phase
// argument z_mm / (2 * pitch_mm) - not itself a fraction of a wrap, and not pre-divided by any
// domain's own stringer count. Every formula that uses this value (base_t = (i +-
// phase_offset)/denominator, both here and in build_domain_events_for_domain) divides it by that
// specific formula's own denominator itself, rather than adding it directly to i/denominator the
// way the pre-redesign phase_offset (already a wrap fraction) did - this is what makes the
// crosshatch family-coincidence period ( = pitch_mm) the same physical Z distance for every
// domain regardless of its own stringer count, rather than the old per-domain-N-dependent period
// that let different domains in the same mesh reach their first coincidence at different,
// uncoordinated Z heights even though every domain shared the same raw phase_offset - confirmed
// as the actual mechanism behind that discrepancy on real capture (Multi-Domain Two-Hole.stl).
// FeatherPrint Corrugated extension, not present in upstream VBCT: lets a caller advance the
// stringer family's start point layer-by-layer (driven by Z) for print-rising helical continuity,
// the same phase-anchoring concept FeatherPrint's own Stringer feature and the Fuselage Slicer
// predecessor use. Ignored for a Chain domain's own non-crosshatch base family (Chain's wall
// parametrization has real open ends, so "rotate the start point" isn't well-defined there the
// way it is for a Ring's closed loop - that family is always sampled from its own t=0 regardless
// of this parameter), but used directly (undivided at the call site, divided by chain_wrap_denom
// inside the formula) by both of Chain's own crosshatch wraparound families when active - see that
// family's own doc comment. Defaults to 0.0 (upstream VBCT's own always-t=0 behavior; also what
// the caller passes whenever Crossover Pitch is disabled, mirroring the old phase_rate=0
// convention).
//
// \param anchor_t0_frac Cross-layer continuity override (FeatherPrint Corrugated extension, spec
// REV 1.4 S:5.3): when present, the *first* Ring domain's canonical (longer) wall uses this
// arc-length fraction in [0, 1) as its t=0 anchor directly, instead of computing one fresh via
// reference_angle_arc_length. The caller (VbctAdapter::computeAnchorsForMesh) is expected to have
// already picked this value by nearest-point tracking against the immediately-preceding layer's
// own chosen anchor point - this function has no cross-layer memory of its own and does not
// validate the value beyond using it as given. Ignored for every domain but the first Ring (Stage
// 9's domain composition rarely produces more than one, and this parameter's caller only tracks
// one anchor per mesh per layer - see the caller's own doc comment for the multi-Ring/multi-part
// scope limitation) and, like phase_offset, has no effect on Chain domains. std::nullopt (the
// default) reproduces today's fully-per-layer reference_angle_arc_length behavior exactly.
//
// \param other_wall_t0_frac Same continuity override, for the Ring's *other* (shorter) wall -
// added after real testing showed tracking only the canonical wall left the shorter wall's own
// still-per-layer-absolute anchor free to keep producing exactly the same class of discontinuity
// (confirmed: after anchor_t0_frac alone shipped, a real multi-layer print still showed jumps
// with zero topology-change resets logged, and the jumps were orientation-dependent - consistent
// with the shorter wall's own fixed-world-+X-relative crossing search hitting a near-degenerate
// case that moves with the model's rotation). Same semantics as anchor_t0_frac otherwise -
// std::nullopt falls back to reference_angle_arc_length for this wall too.
//
// \param reverse_canonical_wall Cross-layer *direction* continuity override (FeatherPrint
// Corrugated extension, spec REV 1.4 S:5.3) - added after anchor_t0_frac/other_wall_t0_frac alone
// were confirmed (via direct testing on a real print, ~640 sign flips logged across one part)
// insufficient: Stage 9 has no preference for which absolute direction (CW/CCW) it emits the
// canonical wall's points in, and that raw direction can flip from one layer to the next with no
// real geometric change behind it. anchor_t0_frac alone only relocates *where* t=0 sits - it
// can't fix *which way* t increases from there, since every other stringer (i != 0) is sampled at
// i/n + t0_frac, walking forward through *this* wall's own point order regardless of what t0_frac
// says. When present and true, the canonical (post length-swap) wall is reversed before
// match_winding and before either t0 override is applied - VbctAdapter::computeAnchorsForMesh has
// already determined (via signed_area's own sign, continuous with the previous tracked layer)
// that this layer's raw direction doesn't match where the continuity chain currently is.
// std::nullopt or false leaves today's per-layer direction exactly as Stage 9 emitted it.
//
// \param chain_anchor_point Cross-layer continuity override for Chain domains (FeatherPrint
// Corrugated extension, spec REV 2.1 S:5.3 "Chain domain support"): when present, the *first*
// Chain domain's left/right walls are both oriented toward this point (via orient_toward) instead
// of domain.cap_start fresh every layer. Unlike Ring's anchor_t0_frac/other_wall_t0_frac, no
// separate "direction" override is needed here - orient_toward's own nearest-endpoint search
// already is the direction fix, once given a stable target: Stage 9's own cap_start/cap_end
// identity assignment (far_cap(), stage9.cpp) has no cross-layer memory and can plausibly flip
// which physical end gets called cap_start between layers with no real geometric change, the same
// class of risk Ring's reverse_canonical_wall addressed for wall direction - VbctAdapter::
// computeAnchorsForMesh is expected to have already picked this point by nearest-point tracking
// against the immediately-preceding layer's own chosen point (defaulting to domain.cap_start on
// the first tracked layer). Ignored for Ring domains and for every Chain domain but the first
// (same one-anchor-per-mesh-per-layer scope as anchor_t0_frac). std::nullopt (the default)
// reproduces today's fully-per-layer domain.cap_start behavior exactly.
//
// \param crosshatch_enabled FeatherPrint Corrugated extension (spec Section 3.1's "two
// counter-rotating helix families (CCW/CW)" design, not present in upstream VBCT): when true, a
// Ring domain emits a second stringer per sample index i, alongside today's single ("CCW") family.
// The new ("CW") member reuses the *exact same* construction as the CCW one at that index - same
// forward index i, same left_t0_frac/right_t0_frac for both walls, no reflection of either - with
// only the sign of effective_phase_offset flipped for both walls together (base_t_cw =
// (i - effective_phase_offset)/denominator, vs. the CCW family's (i +
// effective_phase_offset)/denominator - see effective_phase_offset's own doc comment above for
// why the division happens here rather than adding effective_phase_offset directly to
// i/denominator). Confirmed directly with the user (design decided by elimination after
// two earlier, wrong attempts - see git history for both): the CW family's own per-layer (right -
// left) tilt is therefore identical to the CCW family's, not mirrored - within a single layer the
// two families are the same repeating diagonal shape, just carried at a different absolute
// rotational position around the ring (offset by 2*effective_phase_offset/denominator, in
// theta-space, from the CCW family's own position). The crossing this produces is a genuinely 3D,
// over-Z effect, not a same-layer one: as Z rises, effective_phase_offset grows, so the CCW
// family's whole pattern sweeps one rotational direction around the ring while the CW family's
// sweeps the other - two helical bands winding oppositely, crossing repeatedly over the print's
// height, the same way two oppositely-wound helices on a lattice tower or gridshell cross. This
// means crosshatch has **no visible effect within a single isolated layer**, or anywhere
// effective_phase_offset is 0 (Crossover Pitch disabled, or this exact Z happens to land on a
// whole or half crossover period) - the two families exactly coincide there, and are silently not
// doubled (see run_stage10's own crosshatch_active gate in the .cpp) rather than printed as a
// redundant duplicate. This is accepted and expected design, confirmed with the user, not a bug:
// with no Z-phase progression there is no helix at all for a second one to counter-rotate against
// - crosshatch is fundamentally a property of the
// print's height, not of any single layer's own 2D geometry. Ignored for Chain domains, same as
// phase_offset and the two t0-fraction overrides above: Chain's left/right walls are both oriented
// toward the same cap_start with t0=0 for both, so "CW/CCW" isn't a meaningful distinction for its
// "ladder rung" sampling the way it is for a Ring's closed loop - Chain keeps today's single family
// regardless of this parameter. Defaults to false (today's single-family behavior, unchanged).
//
// \param chain_left_near_t_frac, \param chain_left_far_t_frac, \param chain_right_near_t_frac,
// \param chain_right_far_t_frac Cross-layer *end-position* continuity overrides for Chain domains
// (FeatherPrint Corrugated extension, Chain domain end-position continuity fix): unlike Ring's
// single wraparound anchor, a Chain's own t=0 and t=1 are two real, distinct wall caps - so
// chain_anchor_point alone (which only fixes *which* end orient_toward calls "near," the direction
// fix) leaves both the near AND far end's own exact raw vertex position free to jitter layer to
// layer under the same per-layer-absolute VBCT noise anchor_t0_frac/other_wall_t0_frac already
// exist to damp for Ring (Stage 9's far_cap() has no cross-layer memory of its own). When present,
// each wall's own base-family sampling interpolates between its own tracked near and far fraction
// instead of running the wall's full raw t=0..1 span - see prepare_domain_sampling's own Chain
// branch (stage10.cpp) for exactly how. The caller (VbctAdapter::computeAnchorsForMesh) is expected
// to have already found each value by nearest-point tracking (nearestPointOnWall) against the
// previous tracked layer's own corresponding point, independently for each of the four
// (wall x end) combinations - mirroring anchor_t0_frac/other_wall_t0_frac's own dual-wall tracking,
// doubled for Chain's two real ends. Applied only to the first Chain domain found (same
// one-anchor-per-mesh-per-layer scope as chain_anchor_point). Ignored for Ring domains and for
// Chain's own crosshatch wraparound family (which already reaches the domain's true end caps a
// different way). std::nullopt (the default) reproduces today's exact pre-tracking behavior: near
// 0.0, far 1.0 - the wall's own full raw span, oriented via chain_anchor_point/domain.cap_start as
// before.
//
// \param extra_clip_loops De Minimis Hole Threshold (FeatherPrint Corrugated extension, not
// present in upstream VBCT): closed point loops (each one a hole's own boundary, in this file's
// own mm coordinate space) that are no longer part of domain classification (the caller has
// already filtered them out of VBCT's own input contours) but are still real, physical voids that
// a stringer must not cross uncut. Appended to every domain's own boundary_edges (see
// prepare_domain_sampling's own comment for why this is sufficient - clip_stringer/point_in_domain
// are pure even-odd parity tests over a flat edge list, with no connectivity assumptions at all)
// so any stringer crossing an ignored hole is split/dropped exactly the same way one crossing a
// non-convex domain edge already is. Applied to every domain uniformly, not gated by which domain
// is "the tracked one" the way the anchor overrides above are - which domain a given hole is
// physically inside isn't known ahead of VBCT's own classification. Empty (the default) reproduces
// today's exact pre-feature behavior.
Stage10Result run_stage10(
    const Stage9Result& stage9,
    double spacing,
    double phase_offset = 0.0,
    std::optional<double> anchor_t0_frac = std::nullopt,
    std::optional<double> other_wall_t0_frac = std::nullopt,
    std::optional<bool> reverse_canonical_wall = std::nullopt,
    bool crosshatch_enabled = false,
    std::optional<Point2> chain_anchor_point = std::nullopt,
    std::optional<double> chain_left_near_t_frac = std::nullopt,
    std::optional<double> chain_left_far_t_frac = std::nullopt,
    std::optional<double> chain_right_near_t_frac = std::nullopt,
    std::optional<double> chain_right_far_t_frac = std::nullopt,
    const std::vector<std::vector<Point2>>& extra_clip_loops = {},
    // Chain wall outer/inner identity continuity (VBCT sign-oscillation investigation) - see
    // CorrugationAnchor::chain_swap_left_right's own doc comment. Swaps a Chain domain's own
    // left/right at the top of prepare_domain_sampling when the caller's own cross-layer tracking
    // (VbctAdapter::computeAnchorsForMesh) has determined this layer's domain.left is the physical
    // wall the previous tracked layer called "right". Applied only to the same single tracked
    // Chain domain chain_anchor_point/chain_left_near_t_frac already scope to - see this
    // function's own apply_chain_anchor_here gate. Ignored for Ring domains.
    bool chain_swap_left_right = false,
    // Transition Layer (FeatherPrint Corrugated extension, spec REV 3.3/3.6/5.10, not present in
    // upstream VBCT): substitutes a full-density solid-infill pass for ordinary corrugation, for
    // one specific domain only, on this one call. transition_ring, when true, matches the Ring
    // domain (this project tracks at most one Ring per layer); transition_chain_domain_identities
    // matches whichever Chain domain's own mean point (of its combined left+right wall points, in
    // this file's own mm space) lands within a small live-re-match tolerance of one of these
    // caller-supplied points - the caller (VbctAdapter::corrugate) has already decided, in an
    // earlier pass over this exact same underlying geometry, which domain(s) should transition;
    // this is only a live re-match against *this* call's own fresh Stage 9 classification, not a
    // fresh decision, so the tolerance only needs to absorb ordinary floating-point/re-derivation
    // noise between the two passes, not real cross-layer drift. More than one Chain domain may
    // independently transition at once (unlike the anchor overrides above, which apply only to the
    // first match found). For whichever domain(s) match: domain_stringers is called with
    // transition_solid_fill_spacing in place of this function's own spacing parameter, and
    // crosshatch_enabled forced off for that domain regardless of this function's own
    // crosshatch_enabled parameter - see DomainStringers::is_transition_layer for how the caller
    // tells which output is which. transition_chain_domain_identities empty and transition_ring
    // false (the default for both) reproduces today's exact pre-feature behavior for every domain.
    bool transition_ring = false,
    const std::vector<Point2>& transition_chain_domain_identities = {},
    double transition_solid_fill_spacing = 0.0);

// FeatherPrint Corrugated extension (not present in upstream VBCT): one crossing event - a single
// place where the linked corrugation skin (VbctAdapter::corrugateLinkedSkin) switches from
// following the outer wall to following the inner wall, or vice versa - built by
// build_domain_events below. Unlike Stringer/StringerEdge above, an event retains which wall each
// endpoint sits on (outer_pt/inner_pt, not an unordered pair) and enough metadata for the caller
// to detect cases it can't yet handle (clipped) without recomputing anything.
struct DomainEvent {
    double theta{ 0.0 };  // this event's own position in the shared cyclic sort order used to walk
                          // both families' events consistently (see build_domain_events' own doc
                          // comment for why sorting by this raw combined-family parameter, rather
                          // than by either wall's own attachment position, is what makes a single
                          // walk valid for both walls at once). Not itself a wall attachment
                          // position - only meaningful for sorting/ordering purposes.
    Point2 outer_pt;      // this event's attachment point on the canonical (longer, "outer") wall.
    Point2 inner_pt;      // this event's attachment point on the other ("inner") wall.
    double outer_t_frac{ 0.0 }; // outer_pt's own arc-length fraction (in [0, 1)) along the outer
                          // wall - the exact value sample_at_t used to produce outer_pt. Exposed
                          // so a caller building a linked skin path from an *offset* copy of the
                          // wall (VbctAdapter::corrugateLinkedSkin) can sample that offset wall at
                          // this same fraction directly, rather than nearest-point-searching for
                          // outer_pt's own physical location on it - see corrugateLinkedSkin's own
                          // doc comment for why nearest-point search there was confirmed (via
                          // direct user report on real output) not to reliably preserve this
                          // struct's own theta-driven event order, since a whole-loop Clipper
                          // offset can distort a wall's local geometry (sharp miter joins, uneven
                          // vertex density) enough that two theta-adjacent events' nearest points
                          // occasionally land in the wrong relative order on the offset wall,
                          // however accurately each one's own position was found individually.
                          // Sampling by this same fraction instead is trivially order-preserving
                          // (sample_at_t's own t argument, applied to any wall in the same
                          // direction), at the cost of slightly less positionally-accurate "one
                          // line width inboard" placement, per Clipper not preserving arc-length
                          // parametrization affinely under offset - a deliberate, confirmed
                          // trade-off, not an oversight.
    double inner_t_frac{ 0.0 }; // Same as outer_t_frac, for inner_pt on the inner wall.
    bool is_cw{ false };  // which stringer family this event came from - CCW (false) or the
                          // phase-mirrored CW family (true, spec Section 3.1's second
                          // counter-rotating helix). Diagnostic/test use; the walk itself doesn't
                          // need to know which family an event belongs to.
    bool clipped{ false }; // true if the straight outer_pt-inner_pt segment would leave this
                          // domain's own *real boundary* somewhere (i.e. clip_stringer against
                          // DomainSampling::edges alone would split it into more than one piece) -
                          // a caller building a linked skin path from these events should treat any
                          // domain with a clipped event as not yet linkable (see build_domain_events'
                          // own doc comment) rather than silently drawing a straight crossing that
                          // cuts outside the domain. Deliberately does NOT consider De Minimis Hole
                          // Threshold's own extra_clip_loops (spec REV 3.0/3.5/5.9) - a crossing
                          // that only passes through an ignored hole is expected and does not make
                          // this domain unlinkable; VbctAdapter's own corrugateLinkedSkin instead
                          // post-processes the finished linked-skin path against those hole loops
                          // directly, splitting it into safe pieces only where one is actually
                          // crossed (see that function's own doc comment).
};

// One domain's full set of crossing events for the linked corrugation skin, sorted into one
// walk-ready cyclic order.
struct DomainEvents {
    bool ok{ false };      // false for every domain this can't build a linked skin path for: a
                           // Ring domain where crosshatch_enabled is false or the CW family fully
                           // coincides with the CCW family at this domain's current phase_offset
                           // (both cases mean there is no genuine "return trip" family to
                           // alternate against - see events' own doc comment), or a Glob domain.
                           // Chain domains do NOT require crosshatch_enabled (see
                           // build_domain_events' own doc comment for why a Chain's single family
                           // is already sufficient) - a Chain domain's own ok is false only when
                           // prepare_domain_sampling's defensive n<2/degenerate-length checks fire.
                           // Every other field is meaningless when this is false.
    bool is_ring{ false }; // true when ok is true and this came from a Ring domain; false when ok
                           // is true and this came from a Chain domain (see events' own resulting
                           // shape: a closed cyclic walk for Ring, an open cap-to-cap walk for
                           // Chain - a caller building a toolpath from this must branch on this
                           // field to know whether to close the loop). Also false, meaninglessly,
                           // whenever ok is false.
    std::vector<DomainEvent> events; // sorted by theta ascending (the shared cyclic order - see
                           // DomainEvent::theta). Always an even count when ok is true: either
                           // n_even (CW fully coincides with CCW at this phase - only the CCW
                           // family's events survive, spaced to close on their own) or 2*n_even
                           // (both families present) - see build_domain_events' own doc comment
                           // for why an even count is required, not just typical.
    std::vector<Point2> outer_wall; // the same canonicalized (longer-wall, direction-corrected)
                           // wall point arrays domain_stringers/prepare_domain_sampling used to
                           // build events' own outer_pt/inner_pt above - exposed here (duplicated,
                           // not referenced, since a caller may outlive the Stage9Result these
                           // came from) so a caller building a linked skin path (which needs to
                           // walk the *whole* wall between two events, not just their own
                           // endpoints) doesn't have to re-derive the same canonicalization
                           // independently - see prepare_domain_sampling's own doc comment in
                           // stage10.cpp for why re-deriving it separately is a real, confirmed
                           // source of bugs in this file's own history.
    std::vector<Point2> inner_wall;
};

// FeatherPrint Corrugated extension (not present in upstream VBCT, and not used by run_stage10
// above): builds the crossing-event sequence VbctAdapter::corrugateLinkedSkin needs to walk a
// single continuous "skin" toolpath that alternates between a Ring domain's two walls every time
// it crosses a stringer (spec Section 3.1's trough/rise/peak/fall cross-section, finally linked
// into one path instead of independent stringer segments). Shares run_stage10/domain_stringers'
// own wall-canonicalization, anchor-override, and t0 logic exactly (via the internal
// prepare_domain_sampling helper both now call) - this function and run_stage10 can never silently
// diverge on those decisions.
//
// Chain domains (FeatherPrint Corrugated extension, spec REV 2.1 "Chain domain support"): unlike
// Ring, a Chain domain does NOT require crosshatch_enabled and never builds a CW family - its own
// single family's n stringers already sit at distinct, monotonically increasing t values along an
// open (not cyclic) wall pair, which is sufficient on its own to alternate a walk between them.
// This differs from Ring's own requirement below for a real reason, not an inconsistency: Ring's
// events all share one t=0/t=1 identity point (a closed loop), so a single family's own events are
// evenly spaced but all "face the same way" (every one goes outer-to-inner, never the reverse) -
// alternating outer/inner wall between consecutive same-direction crossings around a closed loop
// is exactly the "no return trip" problem crosshatch solves. A Chain's own two walls have real,
// distinct endpoints (no shared t=0/t=1 identity to be trapped by) - walking from one cap to the
// other, alternating which wall governs the skin at each successive stringer crossing along the
// way, is a well-defined path regardless of how many stringers there are, with no "return trip"
// needed at all. Nor does a Chain get the forced-even stringer count below - that rule exists only
// to let an alternating walk close consistently back onto itself (a 2-coloring of a *cycle*
// graph), and a Chain's own walk is an open path, not a cycle, so no closure parity constraint
// applies to it either.
//
// Requires crosshatch_enabled for Ring: with only the CCW family available, every stringer runs
// outer-to-inner and none run the other way, so there is no "return trip" to alternate against at
// all - a single-family Ring domain (crosshatch_enabled false) always gets ok=false, not a
// degenerate one-directional path.
//
// No coincidence detection or dedup: both families' events are always built and always alternate
// walls at every single crossing, unconditionally. An earlier version dropped the entire CW family
// whenever a single representative index's own CCW/CW pair landed close together, on the theory
// (correct in exact arithmetic, via the bijection j = i+m mod n_even for a fixed phase_offset) that
// this generalized to every other index too - confirmed by direct user report ("stringers skipped
// near crossings") that this either wasn't reliable under real floating-point evaluation, or wasn't
// the actual cause of that report in the first place, and per the user's own direction was removed
// rather than further patched: a genuinely coincident pair now just produces a near-zero-length arc
// immediately followed by a crossing back, not a dropped stringer.
//
// Always rounds the domain's own stringer count up to even before building events (n_even = n, or
// n+1 if n is odd) - alternating which wall governs the skin on every single crossing is exactly a
// 2-coloring of a cycle graph, which only closes consistently on an even cycle. This is
// independent of (and does not affect) domain_stringers/run_stage10's own odd-or-even n - the two
// functions are free to use slightly different stringer counts/spacing for the same domain, since
// they produce structurally different output (independent segments vs. one linked path).
//
// \param spacing Same meaning as run_stage10's own parameter - target arc-length spacing driving
// the (pre-even-rounding) stringer count for this domain.
// \param phase_offset Same meaning as run_stage10's own parameter.
// \param anchor_t0_frac, \param other_wall_t0_frac, \param reverse_canonical_wall Same
// cross-layer-continuity overrides as run_stage10's own parameters of the same names, applied only
// to the first Ring domain found, for the same reason (the caller tracks exactly one anchor per
// mesh per layer) - see run_stage10's own doc comments for the full account.
// \param crosshatch_enabled Same meaning as run_stage10's own parameter - see this function's own
// "Requires crosshatch_enabled for Ring" note above for why a false value here always yields
// ok=false for every *Ring* domain (Chain domains are unaffected by this parameter - see this
// function's own "Chain domains" note above).
// \param chain_anchor_point Same cross-layer-continuity override as run_stage10's own parameter of
// the same name, applied only to the first Chain domain found - see run_stage10's own doc comment
// for the full account.
// \param chain_crosshatch_enabled FeatherPrint Corrugated extension (spec REV 2.4, "Chain-specific
// Crosshatch + Linked Corrugation Skin integration"): when true, a Chain domain's own events are
// built from BOTH families domain_stringers already builds for its unlinked output (spec REV 2.3
// through 2.4's "fifth round" - see stage10.cpp's own top-of-file history note for the wraparound
// construction currently in use), not just the fixed base family - merged into one theta-sorted
// list (up to 2n events instead of n), so the same alternating outer/inner walk used for the
// single-family case also carries the crosshatch diagonal into the linked path. Deliberately a
// SEPARATE parameter from crosshatch_enabled above, not a reuse of it: crosshatch_enabled's true
// value is a hard Ring-only requirement unrelated to the user's own Crosshatch toggle (a
// single-family Ring simply has no "return trip" to link at all), while this parameter reflects
// the user's own corrugated_crosshatch_enabled setting and is genuinely optional for Chain - Chain
// linking already works with just the base family (spec REV 2.1/2.2), so tangling the two into one
// boolean would give Chain's optional behavior a meaning it was never designed to carry for Ring.
// Ignored for Ring domains. Defaults to false (today's single-family Chain behavior, unchanged).
// \return One DomainEvents per Ring/Chain domain in \p stage9, in the same order run_stage10 would
// produce Stage10Result::domains for the same input - a caller wanting both can zip them.
// \param chain_left_near_t_frac, \param chain_left_far_t_frac, \param chain_right_near_t_frac,
// \param chain_right_far_t_frac Same Chain end-position continuity overrides as run_stage10's own
// parameters of the same names - see its own doc comment for the full account. Applied only to the
// first Chain domain found, same scope as chain_anchor_point.
// \param extra_clip_loops Same De Minimis Hole Threshold clip-edge extension as run_stage10's own
// parameter of the same name - see its own doc comment for the full account.
std::vector<DomainEvents> build_domain_events(
    const Stage9Result& stage9,
    double spacing,
    double phase_offset = 0.0,
    std::optional<double> anchor_t0_frac = std::nullopt,
    std::optional<double> other_wall_t0_frac = std::nullopt,
    std::optional<bool> reverse_canonical_wall = std::nullopt,
    bool crosshatch_enabled = false,
    std::optional<Point2> chain_anchor_point = std::nullopt,
    bool chain_crosshatch_enabled = false,
    std::optional<double> chain_left_near_t_frac = std::nullopt,
    std::optional<double> chain_left_far_t_frac = std::nullopt,
    std::optional<double> chain_right_near_t_frac = std::nullopt,
    std::optional<double> chain_right_far_t_frac = std::nullopt,
    const std::vector<std::vector<Point2>>& extra_clip_loops = {},
    // Chain wall outer/inner identity continuity - see run_stage10's own doc comment for this
    // same parameter for the full rationale.
    bool chain_swap_left_right = false);

}  // namespace vbct
