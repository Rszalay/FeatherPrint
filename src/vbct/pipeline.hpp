// vbct(contours, x) -> Mesh. Wires Stages 1-4 together. Port of
// vbct/pipeline.py -- see spec REV 2.1 S:2.2.
#pragma once

#include <optional>
#include <vector>

#include "contour.hpp"
#include "mesh.hpp"
#include "stage10.hpp"

namespace vbct {

// Stage 2: subdivide one contour's boundary edges via VBS.
Contour apply_vbs(const Contour& contour, double x);

struct VbctResult {
    Mesh mesh;
    std::vector<Contour> post_vbs;
};

// Stages 1-4: validate, VBS-subdivide, triangulate, classify.
// `dedup_epsilon` is forwarded to Stage 0.5's collapse_near_duplicate_points
// (see contour.hpp) -- the default matches that function's own default.
// Exposed here mainly so tests can pin the pre-widened value and keep
// isolated coverage of whatever a wider epsilon would otherwise paper over.
VbctResult run_vbct(const std::vector<Contour>& contours, double x, int64_t dedup_epsilon = 50);

// Stages 1-10, contours straight through to stringers. Has no Python
// equivalent -- the reference wires Stages 5-10 by hand in each render
// script (spec REV 2.1 S:2.2, open item S:16.8: "no top-level
// orchestration past Stage 4"). This closes that gap.
//
// \param phase_offset FeatherPrint Corrugated extension (not upstream VBCT) -- forwarded
// straight to run_stage10; see stage10.hpp for its meaning and the Ring-only scope note.
// \param anchor_t0_frac FeatherPrint Corrugated extension (spec REV 1.4 S:5.3) -- forwarded
// straight to run_stage10's parameter of the same name; see its own doc comment.
// \param other_wall_t0_frac FeatherPrint Corrugated extension (spec REV 1.4 S:5.3) -- forwarded
// straight to run_stage10's parameter of the same name; see its own doc comment.
// \param reverse_canonical_wall FeatherPrint Corrugated extension (spec REV 1.4 S:5.3) --
// forwarded straight to run_stage10's parameter of the same name; see its own doc comment.
// \param crosshatch_enabled FeatherPrint Corrugated extension (spec Section 3.1) -- forwarded
// straight to run_stage10's parameter of the same name; see its own doc comment.
// \param chain_anchor_point FeatherPrint Corrugated extension (spec REV 2.1 "Chain domain
// support") -- forwarded straight to run_stage10's parameter of the same name; see its own doc
// comment.
// \param chain_left_near_t_frac, \param chain_left_far_t_frac, \param chain_right_near_t_frac,
// \param chain_right_far_t_frac FeatherPrint Corrugated extension (Chain domain end-position
// continuity) -- forwarded straight to run_stage10's parameters of the same names; see their own
// doc comment.
// \param extra_clip_loops FeatherPrint Corrugated extension (De Minimis Hole Threshold) --
// forwarded straight to run_stage10's parameter of the same name; see its own doc comment.
// \param transition_ring, \param transition_chain_domain_identities, \param
// transition_solid_fill_spacing FeatherPrint Corrugated extension (Transition Layer, spec REV
// 3.3/3.6/5.10) -- forwarded straight to run_stage10's parameters of the same names; see their own
// doc comment.
Stage10Result run_full_pipeline(
    const std::vector<Contour>& contours,
    double x,
    double threshold,
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
    bool transition_ring = false,
    const std::vector<Point2>& transition_chain_domain_identities = {},
    double transition_solid_fill_spacing = 0.0);

}  // namespace vbct
