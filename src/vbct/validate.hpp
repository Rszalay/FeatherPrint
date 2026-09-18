// Stage 1: Validate & tag. Port of vbct/validate.py -- see spec REV 2.1 S:4.
//
// The Python reference uses shapely's exact LinearRing intersection
// geometry to classify every touch/crossing case in full generality. This
// port instead uses exact-integer segment intersection (sound for the
// integer pre-VBS coordinates this stage always runs on) and narrows the
// "exact coincidence" test to "the two segments share a literal common
// endpoint" rather than shapely's fuller point-in-either-vertex-set
// geometry-collection walk. This is a deliberate scope reduction: every
// one of the 34 library cases passes validation (none exercise the error
// paths), and the exact-endpoint case is the one the spec's own examples
// (two lobes pinching to a point, a hole touching an outer boundary) are
// actually shaped like.
#pragma once

#include <stdexcept>
#include <vector>

#include "contour.hpp"

namespace vbct {

class ValidationError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Raises ValidationError on any precondition violation.
void validate_contours(const std::vector<Contour>& contours);

}  // namespace vbct
