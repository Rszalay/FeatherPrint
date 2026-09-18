// Stage 7: skeleton chaining. Port of vbct/stage7.py -- see spec REV 2.1
// S:10. Walks Stage 6's segments into explicit polylines and classifies
// every point where a chain stops.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "geometry.hpp"
#include "stage5.hpp"
#include "stage6.hpp"

namespace vbct {

struct Chain {
    std::vector<Point2> points;
    std::vector<int> triangles;  // len(points)-1; triangles[k] produced points[k]->points[k+1]
    bool closed;
    std::optional<std::string> start_class;  // "dead_end" | "branch" | "incomplete"; nullopt if closed
    std::optional<std::string> end_class;
    std::optional<NodeId> start_node;  // nullopt if closed
    std::optional<NodeId> end_node;
    bool incomplete;
};

struct Stage7Result {
    std::vector<Chain> chains;
};

Stage7Result run_stage7(const Stage5Result& stage5, const Stage6Result& stage6);

}  // namespace vbct
