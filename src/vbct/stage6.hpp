// Stage 6: skeleton segment generation. Port of vbct/stage6.py -- see spec
// REV 2.1 S:9. Every surviving Stage-5 triangle gets exactly one skeleton
// segment, computed from local shape and edge classification alone.
#pragma once

#include <map>
#include <utility>
#include <vector>

#include "geometry.hpp"
#include "stage5.hpp"

namespace vbct {

// A segment endpoint's symbolic identity: an opening-edge midpoint
// (sorted vertex-index pair) or a tip vertex. Stage 7 keys its chain
// graph on this rather than on raw float coordinates -- spec S:9.5/S:10.1.
struct NodeId {
    enum Kind { EdgeNode, VertexNode } kind;
    int u = 0;
    int v = -1;  // unused (-1) for VertexNode

    static NodeId edge(int a, int b) {
        Edge e = make_edge(a, b);
        return {EdgeNode, e.first, e.second};
    }
    static NodeId vertex(int idx) { return {VertexNode, idx, -1}; }

    bool operator==(const NodeId& o) const { return kind == o.kind && u == o.u && v == o.v; }
    bool operator!=(const NodeId& o) const { return !(*this == o); }
    bool operator<(const NodeId& o) const {
        if (kind != o.kind) return kind < o.kind;
        if (u != o.u) return u < o.u;
        return v < o.v;
    }
};

using Segment = std::pair<Point2, Point2>;

struct Stage6Result {
    std::map<int, Segment> segments;                  // triangle index -> skeleton segment
    std::map<int, std::pair<NodeId, NodeId>> endpoints;  // triangle index -> (node, node), same order as segments[i]
    std::vector<int> skipped;                          // triangle indices with no computed segment
};

// `max_reference_hops`: for a terminal triangle whose neighbour across its
// opening is a sleeve (spec S:9.2 rule 3), how many further sleeve
// triangles the reference direction is walked back through -- see rule
// 3's own doc comment in stage6.cpp. 1 reproduces the original spec (only
// the immediately adjacent sleeve segment). Exposed as a parameter (not a
// hardcoded constant) so tests can probe the walk directly; production
// callers should rely on the default.
Stage6Result run_stage6(const Stage5Result& stage5, int max_reference_hops = 4);

}  // namespace vbct
