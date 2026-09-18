// Rigid rotation of contours -- used to check the pipeline is
// orientation-invariant. Port of vbct/transform.py.
#pragma once

#include <optional>
#include <vector>

#include "contour.hpp"
#include "geometry.hpp"

namespace vbct {

// Rotates every contour rigidly by `angle_rad` about `center` (defaults
// to the centroid of all input points).
std::vector<Contour> rotate_contours(const std::vector<Contour>& contours, double angle_rad,
                                      std::optional<Point2> center = std::nullopt);

}  // namespace vbct
