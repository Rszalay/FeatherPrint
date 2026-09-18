// Chain Wall A/B canonical assignment. Spec REV 3.4, Section 3.7/5.11 -- a
// geometry-driven rule set deciding which of a Chain domain's two physical
// walls is canonically "Wall A" vs "Wall B". Needs no cross-layer history
// (see the chain_domain_stability_and_sign_oscillation lessons-learned,
// Section 8): re-evaluating it fresh, per layer, from that layer's own
// geometry is what keeps FeatherPrint's own adapter-side consumers
// (chain_swap_left_right, wall_a_is_left) stable layer-to-layer within one
// model -- both call this directly at their live per-layer call site rather
// than tracking a pick across layers.
//
// Developed and unit-tested here first per this project's own established
// convention; ported verbatim into FeatherPrint Corrugated's own
// src/vbct/wall_ab_assignment.{hpp,cpp}. See
// tools/diagnose_wall_ab_assignment.cpp for an ad-hoc CLI wrapper and
// tests/test_wall_ab_assignment.cpp for the regression suite.
#pragma once

#include <string>
#include <vector>

#include "geometry.hpp"

namespace vbct {

struct Contour;  // contour.hpp

enum class WallLabel { A, B };

struct WallAbAssignment {
    // Label assigned to the `left` wall passed to assign_wall_ab (the
    // `right` wall always gets the other label).
    WallLabel left_label = WallLabel::A;
    // Which rule actually resolved the assignment: "rule1" | "rule2" | "rule3".
    std::string rule_used;
    // Human-readable explanation, for diagnostics/test failure messages.
    std::string reason;
};

// Rule 1 (primary): contour signed-area. A wall is Wall A if it lies on the
// part's outer-silhouette contour (positive area, shoelace/CCW convention),
// Wall B if it lies on a hole contour (negative area). Fires only when both
// walls confidently match distinct contours with opposite-sign area.
WallAbAssignment assign_wall_ab_rule1(const std::vector<Point2>& left, const std::vector<Point2>& right,
                                       const std::vector<Contour>& contours);

// Rule 2 (fallback): self-contained convexity. Each wall shares its two
// endpoints with the other (both run cap_start -> cap_end), so each wall's
// own bulge relative to the straight chord between those two shared
// endpoints is directly comparable between the two walls without ever
// looking at the other wall's own points. The wall with the larger bulge
// (chord-area magnitude, normalised by chord length^2 to stay scale-
// invariant) is Wall A. Superseded a nearest-other-wall-point "which side is
// interior" test that broke down whenever the two walls converge toward
// each other anywhere along their length, not just at a domain's two end
// caps -- an airfoil's upper/lower walls converge at *both* the nose and
// the tail, which produced wildly unstable results (confirmed directly: up
// to +-13 radians, sign-flipping almost every layer, on a real 59-layer
// sweep of an airfoil's full height). This measure stayed smooth and
// unambiguous (gap ratio >= 1.34) across the same sweep.
WallAbAssignment assign_wall_ab_rule2(const std::vector<Point2>& left, const std::vector<Point2>& right);

// Rule 3 (last resort, always resolves): whichever wall's own mean point
// sits farther from the part's overall centroid is Wall A.
WallAbAssignment assign_wall_ab_rule3(const std::vector<Point2>& left, const std::vector<Point2>& right,
                                       const Point2& part_centroid);

// Full priority-ordered dispatch: Rule 1, else Rule 2, else Rule 3.
WallAbAssignment assign_wall_ab(const std::vector<Point2>& left, const std::vector<Point2>& right,
                                 const std::vector<Contour>& contours, const Point2& part_centroid);

}  // namespace vbct
