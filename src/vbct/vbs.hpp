// Variance-Bounded Split. Direct port of vbct/vbs.py + the `Edge`
// adaptation in vbct/pipeline.py's apply_vbs -- see spec REV 2.1 S:3.
//
// The Python reference keeps `vbs()` generic over an abstract "line" via
// caller-supplied length/split callbacks; VBCT's own use is always a
// straight integer contour edge, so this port specializes directly to
// `Contour::IEdge` rather than reproducing that generality for a single
// caller.
#pragma once

#include <vector>

#include "contour.hpp"

namespace vbct {

using IEdge = Contour::IEdge;

double edge_length(const IEdge& e);

// Splits `e` into `k` pieces. Matches pipeline.py's `_split_edge`: interior
// points are `round(x0 + (x1-x0)*i/k)` (python_round, i.e. half-to-even),
// not an evenly-spaced construction independent of rounding.
std::vector<IEdge> split_edge(const IEdge& e, int k);

// Splits overlong items until all lengths are within
// [A0*(1-x), A0*(1+x)], A0 = mean length of the *input* set, frozen once.
// See spec S:3.3 for why A0 must not be recomputed mid-run.
std::vector<IEdge> vbs(const std::vector<IEdge>& lines, double x);

}  // namespace vbct
