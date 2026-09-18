// Stage 5: Junction correction. Port of vbct/stage5.py -- see spec REV 2.1
// S:8. Runs Corrections 1-3 once, in order, over Stage 4's interior mesh.
//
// Only the `"single_point"` hub algorithm (S:8.5.1) is ported -- it is the
// REV 2.1 default and production choice; `"incremental"`/`"fan_bridge"`
// are explicitly "retained for comparison only" in the spec and are a
// deliberate scoping omission here (see the plan).
#pragma once

#include <array>
#include <set>
#include <vector>

#include "geometry.hpp"
#include "mesh.hpp"

namespace vbct {

using Triangle = std::array<int, 3>;

struct Stage5Result {
    std::vector<Point2> vertices;   // mesh vertices + Correction-3 centroids
    std::vector<Triangle> triangles;
    std::vector<Edge> hub_edges;    // Correction-3 promoted geometry (real output)
    int sliver_flip_count = 0;      // Correction 0: sliver-triangle edge flip
    int tip_collapse_count = 0;     // Correction 0c: near-duplicate dead-end tip-vertex collapse
    int sliver_removal_count = 0;   // Correction 0b: fully-isolated sliver removal
    int merge_count = 0;            // Correction 1
    int wedge_collapse_count = 0;   // Correction 1b: terminal-wedge collapse (triangles removed)
    int excision_count = 0;         // Correction 2
    int junction_terminal_flip_count = 0;  // Correction 2b: junction/terminal edge flip
    int debridge_count = 0;         // Correction 2.5: bridging-triangle re-routing
    int quad_merge_count = 0;       // Correction 1c: VBS-point link-polygon merge (K>=3 incident triangles)
    int cap_corner_fixup_count = 0; // Correction 2.55: cap-corner false junctions kept out of the fan
    int stub_absorb_count = 0;      // Correction 2.7: short-stub absorption (one or two short arms)
    int retriangulation_count = 0;  // Correction 2.6: general local CDT re-triangulation of stubborn hubs
    std::vector<int> hub_sizes;     // Correction 3: junction-triangle count per hub
    int sleeve_count = 0;
    int terminal_count = 0;
    int junction_count = 0;         // before corrections 2-3
    std::set<Edge> boundary_edges;  // contour edges + every correction's promotions
    std::set<Edge> contour_edges;   // Mesh.constrained, unmodified

    // Diagnostic surface (tools/diagnose_prefan_junctions.cpp): the mesh and boundary_edges set
    // exactly as Correction 3 itself sees them (i.e. after Corrections 0-2.7, before the
    // single_point fan) -- lets an external tool re-run cluster_junction_triangles' own bc==0
    // test and cross-check every "junction" triangle's 3 edges against edge_to_tris (how many
    // triangles in the whole pre-fan mesh actually share that edge), the same check that found
    // the sliver-triangle root cause Correction 0 now fixes. Kept as a small, always-populated
    // field rather than removed post-investigation, since a future hub-classification question
    // is exactly the kind of thing this session needed and didn't have.
    std::vector<Triangle> debug_pre_c3_triangles;
    std::set<Edge> debug_pre_c3_boundary_edges;
};

// `prune_threshold` <= 0 (the default) disables Correction 2.7's short-stub
// absorption entirely -- pass the same corrugated-stringer prune threshold
// Stage 9 uses to enable it (see stage5.cpp's own rationale comment).
Stage5Result run_stage5(const Mesh& mesh, double prune_threshold = 0.0);

}  // namespace vbct
