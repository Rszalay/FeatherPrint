#include "stage7.hpp"

#include <algorithm>
#include <iterator>
#include <map>
#include <set>

namespace vbct {
namespace {

std::string classify_node(const NodeId& node, int degree, const std::set<Edge>& boundary_edges) {
    if (node.kind == NodeId::EdgeNode) {
        Edge key{node.u, node.v};
        if (boundary_edges.count(key)) {
            // A promoted/contour edge only ever borders one triangle by
            // construction -- degree 1 here is expected, not an error.
            return "dead_end";
        }
        return degree == 2 ? "pass_through" : "incomplete";
    }
    // VertexNode
    return degree == 1 ? "dead_end" : "branch";
}

}  // namespace

Stage7Result run_stage7(const Stage5Result& stage5, const Stage6Result& stage6) {
    const std::set<Edge>& boundary_edges = stage5.boundary_edges;

    std::map<NodeId, Point2> node_point;
    std::map<NodeId, std::vector<int>> node_segments;
    for (const auto& [i, ab] : stage6.endpoints) {
        const auto& [a, b] = ab;
        const auto& [pa, pb] = stage6.segments.at(i);
        node_point[a] = pa;
        node_point[b] = pb;
        node_segments[a].push_back(i);
        node_segments[b].push_back(i);
    }

    std::map<NodeId, std::string> node_class;
    for (const auto& [node, segs] : node_segments) {
        node_class[node] = classify_node(node, static_cast<int>(segs.size()), boundary_edges);
    }

    auto other_node = [&](int seg_idx, const NodeId& node) -> NodeId {
        const auto& [a, b] = stage6.endpoints.at(seg_idx);
        return a == node ? b : a;
    };

    std::set<int> visited;
    std::vector<Chain> chains;

    // Open chains: walk out from every stopping (non-pass-through) node.
    for (const auto& [node, cls] : node_class) {
        if (cls == "pass_through") continue;
        for (int seg : node_segments[node]) {
            if (visited.count(seg)) continue;

            std::vector<Point2> points{node_point[node]};
            std::vector<int> step_triangles;
            NodeId cur_node = node;
            int cur_seg = seg;
            NodeId end_node = node;  // overwritten before use
            while (true) {
                visited.insert(cur_seg);
                step_triangles.push_back(cur_seg);
                NodeId nxt_node = other_node(cur_seg, cur_node);
                points.push_back(node_point[nxt_node]);
                if (node_class[nxt_node] != "pass_through") {
                    end_node = nxt_node;
                    break;
                }
                std::vector<int> remaining;
                for (int s : node_segments[nxt_node])
                    if (s != cur_seg) remaining.push_back(s);
                cur_node = nxt_node;
                cur_seg = remaining[0];
            }
            std::string start_class = cls;
            std::string end_class = node_class[end_node];
            chains.push_back(Chain{points, step_triangles, false, start_class, end_class, node, end_node,
                                    start_class == "incomplete" || end_class == "incomplete"});
        }
    }

    // Closed loops: whatever's left is a pure pass-through cycle with no
    // stopping node anywhere on it.
    std::set<int> all_segments;
    for (const auto& [i, _] : stage6.endpoints) all_segments.insert(i);
    std::set<int> remaining_segments;
    std::set_difference(all_segments.begin(), all_segments.end(), visited.begin(), visited.end(),
                         std::inserter(remaining_segments, remaining_segments.begin()));

    while (!remaining_segments.empty()) {
        int seg = *remaining_segments.begin();
        NodeId start_node = stage6.endpoints.at(seg).first;
        std::vector<Point2> points{node_point[start_node]};
        std::vector<int> step_triangles;
        NodeId cur_node = start_node;
        int cur_seg = seg;
        while (true) {
            visited.insert(cur_seg);
            remaining_segments.erase(cur_seg);
            step_triangles.push_back(cur_seg);
            NodeId nxt_node = other_node(cur_seg, cur_node);
            points.push_back(node_point[nxt_node]);
            if (nxt_node == start_node) break;
            std::vector<int> remaining;
            for (int s : node_segments[nxt_node])
                if (s != cur_seg) remaining.push_back(s);
            cur_node = nxt_node;
            cur_seg = remaining[0];
        }
        chains.push_back(
            Chain{points, step_triangles, true, std::nullopt, std::nullopt, std::nullopt, std::nullopt, false});
    }

    return {chains};
}

}  // namespace vbct
