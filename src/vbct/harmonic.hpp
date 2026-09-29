// Harmonic wall pairing (FeatherPrint Corrugated extension, corrugated_wall_pairing = harmonic).
//
// Solves Laplace's equation on a domain's own area - phi = 0 on the left wall, phi = 1 on the right wall, no flux
// through a Chain's end caps - and pairs each point on one wall with the point on the other wall reached by the same
// gradient line of phi. Gradient lines never cross and conserve flux, so the gradient line from a left-wall point
// lands at the right-wall point with the same cumulative flux: the whole pairing comes from the flux along each wall,
// with no streamline tracing. For a Ring the flux match is only fixed up to a rotation of the right wall; the rotation
// with the shortest total rung length is used.
#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "geometry.hpp"

namespace vbct {

struct HarmonicPairing {
    // Matched wall fractions, k = 0..count, both non-decreasing and unwrapped: left runs left_start..left_start+1 for
    // a Ring (0..1 for a Chain), right likewise from its own matched start.
    std::vector<double> tl;
    std::vector<double> tr;
};

// `left` and `right` must run the same way: for a Chain both from the start cap, for a Ring both closed loops with
// the same winding. `left_start` is the Ring's anchor fraction on the left wall (ignored for a Chain). nullopt when
// the domain can't be meshed or the solve fails - the caller falls back to arc-length pairing.
std::optional<HarmonicPairing> harmonic_wall_pairing(const std::vector<Point2>& left, double left_len, const std::vector<Point2>& right,
                                                     double right_len, bool ring, double left_start, int count);

}  // namespace vbct
